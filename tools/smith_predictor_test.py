"""
tools/smith_predictor_test.py

Tests whether a Smith predictor (delay-compensation observer) raises or
eliminates the Kp_ceiling for PID-controlled TVC rockets.

RESEARCH QUESTION
─────────────────
The gain ceiling Kp_max ≈ 380/latency is interpreted as a DELAY-MARGIN
effect under the DIPDT (delayed double-integrator) framework. If that is the
correct mechanism, a Smith predictor — which cancels the effect of delay on
the control loop — should substantially raise or eliminate the ceiling.

Two physically distinct hypotheses:
  H_delay: The ceiling is determined by delay-induced phase lag (linear
    dynamics). Smith predictor → ceiling rises, possibly approaching the
    no-latency limit.
  H_saturation: The ceiling is determined by bang-bang saturation dynamics
    (nonlinear). The delay determines WHEN the saturated pulse fires, but the
    fundamental constraint is the overshoot energy, not the phase lag. Smith
    predictor → ceiling unchanged (or only marginally improved).

SMITH PREDICTOR IMPLEMENTATION (discrete, forward-Euler)
──────────────────────────────────────────────────────────
  Internal plant model (disturbance-free): θ̈_m = keff_m · u_act
    Update each step:  q_m += dt · keff_m · u_act
                       θ_m += dt · q_m
  Correction to sensor measurements:
    Δθ = θ_m[now] − θ_m[t − L]
    Δq = q_m[now] − q_m[t − L]
    θ_smith = θ_ctrl_delayed + Δθ
    q_smith = q_ctrl_delayed + Δq

  Model uses keff_full (oracle, exact), so any failure is attributable to
  the saturation nonlinearity — not model mismatch.

  Key limitation: wind disturbance is NOT in the model. The predictor
  compensates for the delay-induced phase shift but not for uncertainty in
  the disturbance trajectory during the latency window.

PROTOCOL
────────
  3 reference designs at keff ≈ {5, 12, 25} × latency ∈ {3, 6}
  (total 6 design-latency combinations)
  For each: sweep 24 log-spaced Kp values [0.5, 500], best joint Kd from
  18×7 search (3 seeds), 20 eval seeds (full physics)
  Three controllers per design: PID, PID+Smith, ADRC (omega_c=5, reference)
  Output: Kp → SR surface for each architecture

KEY METRIC
──────────
  ceiling_smith / ceiling_pid : ratio tells us how much delay cancellation helps
  If > 2.0: Smith substantially raises ceiling → delay-margin is the mechanism
  If < 1.3: Smith barely helps → saturation dynamics are the real constraint
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from collections import deque
from joblib import Parallel, delayed as jdelayed

from design_space import (
    build_plant, build_actuator, build_sensor,
    build_disturbance, build_scenario,
)
from controller import PIDParams, PIDState, step_pid, ADRCParams, ADRCState, step_adrc
from fidelity_config import FidelityConfig, apply_fidelity_config
from simulator import ScenarioParams, SimResult
from actuator_model import ActuatorState, step_actuator
from sensor_model import SensorState, init_sensor_state, step_sensor
from disturbance_model import GustState, init_gust_state, step_disturbance
from plant_dynamics import PlantParams, step_plant
from units import (
    F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX,
    SUCCESS_MAX_THETA_DEG, SUCCESS_END_ERROR_DEG,
    SUCCESS_RMS_ERROR_DEG, SUCCESS_PEAK_ERROR_DEG,
    DIVERGE_THETA_DEG,
)

# ── Reference airframe ────────────────────────────────────────────────────────
L_NOZZLE    = 0.25   # m
CU_TO_RAD   = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX   # rad/CU

BASE_FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.5,
    max_gimbal_deg=10.0, servo_slew_deg_s=120.0,
    deadband=0.02, backlash=0.05,
)
WIND = 0.25

# Target keff levels and latencies
KEFF_TARGETS = [5.0, 12.0, 25.0]
LAT_VALUES   = [3, 6]

# ── Gain search / evaluation ──────────────────────────────────────────────────
KP_SRCH      = np.geomspace(1.0, 320.0, 18)
KD_SRCH      = np.geomspace(1.0,  64.0,  7)
SEARCH_SEEDS = [1, 2, 3]

KP_SWEEP     = np.geomspace(0.5, 500.0, 24)
N_EVAL       = 20
EVAL_SEED0   = 40001   # 40001-40020, disjoint from all prior
SR_THRESH    = 0.80

N_JOBS  = -1
OUT_CSV = Path('experiments/results/smith_predictor_test_py.csv')


# ── Physics config ────────────────────────────────────────────────────────────

def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _build_row(Iyy, latency):
    return dict(
        BASE_FIXED,
        Iyy=float(Iyy),
        wind_strength=WIND,
        latency_steps=int(latency),
    )


def _keff_for_Iyy(Iyy):
    return F15_AVG_THRUST_N * BASE_FIXED['motor_scale'] * CU_TO_RAD * L_NOZZLE / Iyy


def _Iyy_for_keff(keff_target):
    return F15_AVG_THRUST_N * BASE_FIXED['motor_scale'] * CU_TO_RAD * L_NOZZLE / keff_target


def _build_components(row_dict):
    plant = build_plant(row_dict)
    act   = build_actuator(row_dict)
    sen   = build_sensor(row_dict)
    dis   = build_disturbance(row_dict)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())
    return plant, act, sen, dis, sc


# ── Custom simulation loop with optional Smith predictor ──────────────────────

def simulate_with_smith(
    pid_params: PIDParams,
    plant, actuator, sensor, disturbance, scenario,
    seed: int,
    keff_model: float,
    use_smith: bool = False,
    adrc_params=None,
) -> dict:
    """
    Full-physics simulation with optional Smith predictor correction.

    Smith predictor: maintains an internal plant model (θ̈_m = keff_m · u_act)
    and applies the correction Δθ = θ_m[now] - θ_m[t-L] to the delayed
    measurement before feeding it to the PID controller.

    Returns a dict of scalar metrics.
    """
    rng = np.random.default_rng(seed)
    dt  = plant.dt
    N   = int(round(scenario.t_end / dt)) + 1
    t   = np.arange(N) * dt

    use_dyn_aero     = getattr(scenario, 'use_dyn_aero',     False)
    use_thrust_curve = getattr(scenario, 'use_thrust_curve', False)
    use_cg_shift     = getattr(scenario, 'use_cg_shift',     False)
    use_nonlinear    = getattr(scenario, 'use_nonlinear_aero', False)

    aero_params  = None
    motor_params = None
    motor_state  = None
    if use_dyn_aero or use_nonlinear or use_thrust_curve or use_cg_shift:
        from aero_model import AeroParams, dynamic_pressure, velocity_step
        from motor_model import MotorParams, MotorState, motor_thrust_and_mdot
        from motor_model import motor_vehicle_mass, motor_cg_position, motor_inertia_correction
        aero_params  = AeroParams.from_design(plant.Cm_alpha_signed, plant.static_margin)
        motor_params = MotorParams.F15(
            rocket_length=2.0 * plant.l_nozzle_m,
            scale=plant.motor_scale,
        )
        motor_state  = motor_state_cls = None
        from motor_model import MotorState as MS
        motor_state  = MS.initial(motor_params)

    v_flight = 0.01

    theta_arr = np.zeros(N)
    q_arr     = np.zeros(N)
    u_act_arr = np.zeros(N)

    theta0 = 0.0
    theta_arr[0] = theta0
    q_arr[0]     = 0.0
    u_act_arr[0] = 0.0

    act_state  = ActuatorState(u_servo=0.0, u_output=0.0)
    sens_state = init_sensor_state(sensor, theta0, rng)
    gust_state = init_gust_state(disturbance, rng)
    pid_state  = PIDState(i_state=0.0, q_ctrl_prev=0.0)
    adrc_state = (ADRCState(z1=theta0, z2=0.0, z3=0.0, u_prev=0.0)
                  if adrc_params is not None else None)

    theta_ctrl = sens_state.theta_integrated
    q_ctrl     = 0.0

    # ── Smith predictor state ──────────────────────────────────────────────────
    lat = sensor.sensor_latency_steps
    sm_theta_m = theta0
    sm_q_m     = 0.0
    sm_buf_theta = deque([theta0] * lat, maxlen=lat)
    sm_buf_q     = deque([0.0]    * lat, maxlen=lat)

    for k in range(1, N):
        post_fault  = t[k] >= scenario.fault_time_s
        keff_fault  = scenario.keff_fault_post if post_fault else 1.0
        damp_fault  = scenario.damp_fault_post if post_fault else 1.0
        bias_post   = scenario.disturb_bias_post if post_fault else 0.0

        # Physics quantities
        if use_thrust_curve and motor_params is not None:
            from motor_model import motor_thrust_and_mdot
            T_now, mdot = motor_thrust_and_mdot(motor_params, t[k])
            motor_state.advance(mdot, dt)
        else:
            T_now = F15_AVG_THRUST_N
            mdot  = 0.0

        if use_cg_shift and motor_params is not None and motor_state is not None:
            from motor_model import (motor_vehicle_mass, motor_cg_position,
                                     motor_inertia_correction)
            m_dry_casing = plant.m_kg + motor_params.m_casing
            m_total  = motor_vehicle_mass(plant.m_kg, motor_params, motor_state)
            x_cg_now = motor_cg_position(plant.l_nozzle_m, m_dry_casing, motor_params, motor_state)
            I_yy_now = motor_inertia_correction(
                plant.Iyy_kgm2, plant.l_nozzle_m,
                m_dry_casing, motor_params, motor_state, x_cg_now
            )
        else:
            m_total  = plant.m_kg
            I_yy_now = plant.Iyy_kgm2

        if use_dyn_aero and aero_params is not None:
            from aero_model import dynamic_pressure, velocity_step
            q_dyn_now = dynamic_pressure(v_flight)
            v_flight  = velocity_step(
                v_flight, m_total, T_now,
                theta_arr[k-1], q_dyn_now, aero_params, dt=dt
            )
        else:
            T_eff = F15_AVG_THRUST_N * plant.motor_scale
            a_net = max(0.5, (T_eff - plant.m_kg * 9.81) / max(0.1, plant.m_kg))
            v_mid = a_net * (scenario.t_end * 0.5)
            q_dyn_now = 0.5 * 1.20 * v_mid ** 2

        # Disturbance
        d_raw = step_disturbance(
            gust_state, disturbance, t[k], dt, rng,
            inertia_scale=plant.inertia_scale,
            mass_scale=plant.mass_scale,
        )
        d_eff = d_raw + bias_post * plant.inertia_scale / max(0.5, plant.mass_scale)

        # ── Smith predictor correction ─────────────────────────────────────────
        if use_smith and adrc_params is None:
            # Current model prediction
            theta_m_now = sm_theta_m
            q_m_now     = sm_q_m
            # Prediction from latency steps ago (oldest buffer entry)
            theta_m_del = sm_buf_theta[-1]
            q_m_del     = sm_buf_q[-1]
            # Corrected measurements
            theta_smith = theta_ctrl + (theta_m_now - theta_m_del)
            q_smith     = q_ctrl     + (q_m_now    - q_m_del)
            u_cmd = step_pid(pid_state, pid_params, theta_smith, q_smith,
                             scenario.theta_ref, dt)
        elif adrc_params is not None and adrc_state is not None:
            u_cmd = step_adrc(adrc_state, adrc_params, theta_ctrl, q_ctrl,
                              scenario.theta_ref, dt)
        else:
            u_cmd = step_pid(pid_state, pid_params, theta_ctrl, q_ctrl,
                             scenario.theta_ref, dt)

        # Actuator
        u_act = step_actuator(act_state, actuator, u_cmd, dt)
        u_act_arr[k] = u_act

        # Plant
        theta_new, q_new = step_plant(
            theta_arr[k-1], q_arr[k-1], u_act, d_eff,
            plant, keff_fault, damp_fault,
            aero_params   = aero_params   if (use_dyn_aero or use_nonlinear) else None,
            q_dyn         = q_dyn_now     if (use_dyn_aero or use_nonlinear) else None,
            v             = v_flight      if (use_dyn_aero or use_nonlinear) else None,
            T_now         = T_now         if (use_dyn_aero or use_nonlinear) else None,
            I_yy_now      = I_yy_now      if (use_dyn_aero or use_nonlinear) else None,
            nonlinear_aero = use_nonlinear,
        )
        theta_arr[k] = theta_new
        q_arr[k]     = q_new

        # Sensor (latency buffer updated here)
        q_ctrl_new, theta_ctrl_new = step_sensor(
            sens_state, sensor, q_new, theta_new, dt, rng
        )

        # ── Update Smith predictor model (using real applied u_act) ───────────
        if use_smith and adrc_params is None:
            # Push current model state to buffer BEFORE updating
            sm_buf_theta.appendleft(sm_theta_m)
            sm_buf_q.appendleft(sm_q_m)
            # Advance model by dt using the actual applied actuator output
            sm_q_m     += dt * keff_model * u_act
            sm_theta_m += dt * sm_q_m

        theta_ctrl = theta_ctrl_new
        q_ctrl     = q_ctrl_new

    theta_d = np.degrees(theta_arr)
    err_d   = theta_d - np.degrees(scenario.theta_ref)
    rms_e   = float(np.sqrt(np.mean(err_d ** 2)))
    peak_e  = float(np.max(np.abs(err_d)))
    end_e   = float(np.abs(err_d[-1]))
    max_th  = float(np.max(np.abs(theta_d)))

    success = (
        max_th  < SUCCESS_MAX_THETA_DEG
        and end_e   < SUCCESS_END_ERROR_DEG
        and rms_e   < SUCCESS_RMS_ERROR_DEG
        and peak_e  < SUCCESS_PEAK_ERROR_DEG
    )
    return {'success': success, 'rms': rms_e, 'max_theta': max_th}


# ── Joint Kp×Kd search ────────────────────────────────────────────────────────

def _search_gains(row_dict, u_max, controller='pid'):
    """Find best (Kp, Kd) or omega_c via grid search. Returns (Kp, Kd)."""
    plant, act, sen, dis, sc = _build_components(row_dict)
    keff = _keff_for_Iyy(row_dict['Iyy'])

    if controller == 'adrc':
        # Sweep omega_c; return equivalent (keff, 1) for storage
        best_sr = -1
        best_wc = 5.0
        for wc in [1.5, 2.0, 3.0, 5.0, 7.0, 10.0]:
            adrc = ADRCParams(omega_c=wc, omega0=5*wc, b0=keff, u_max=u_max)
            srs = []
            for sd in SEARCH_SEEDS:
                r = simulate_with_smith(
                    PIDParams(Kp=1.0, Kd=1.0, u_max=u_max),
                    plant, act, sen, dis, sc, seed=sd,
                    keff_model=keff, use_smith=False, adrc_params=adrc,
                )
                srs.append(float(r['success']))
            sr = np.mean(srs)
            if sr > best_sr:
                best_sr = sr
                best_wc = wc
        return best_wc, 1.0  # (omega_c, dummy_Kd)

    best_sr = -1.0
    best_rms = 1e9
    best_Kp = 40.0
    best_Kd = 1.0
    for Kp in KP_SRCH:
        for Kd in KD_SRCH:
            pid = PIDParams(Kp=float(Kp), Kd=float(Kd), u_max=u_max)
            srs, rmss = [], []
            for sd in SEARCH_SEEDS:
                r = simulate_with_smith(
                    pid, plant, act, sen, dis, sc, seed=sd,
                    keff_model=keff, use_smith=(controller == 'smith'),
                )
                srs.append(float(r['success']))
                rmss.append(r['rms'])
            sr = np.mean(srs)
            rms = np.mean(rmss)
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr = sr
                best_rms = rms
                best_Kp = Kp
                best_Kd = Kd
    return best_Kp, best_Kd


def _eval_kp(Kp, Kd, u_max, row_dict, keff, controller, adrc_wc=5.0):
    """Evaluate a single (Kp, Kd) over N_EVAL seeds. Returns SR and mean RMS."""
    plant, act, sen, dis, sc = _build_components(row_dict)
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), u_max=u_max)
    adrc = None
    if controller == 'adrc':
        adrc = ADRCParams(omega_c=adrc_wc, omega0=5*adrc_wc, b0=keff, u_max=u_max)

    results = [
        simulate_with_smith(
            pid, plant, act, sen, dis, sc,
            seed=EVAL_SEED0 + i,
            keff_model=keff,
            use_smith=(controller == 'smith'),
            adrc_params=adrc,
        )
        for i in range(N_EVAL)
    ]
    sr  = float(np.mean([r['success'] for r in results]))
    rms = float(np.mean([r['rms']     for r in results]))
    return sr, rms


def _run_design(keff_target, latency):
    """Full Kp sweep for one (keff, latency) combination."""
    Iyy = _Iyy_for_keff(keff_target)
    row = _build_row(Iyy, latency)
    plant, act, sen, dis, sc = _build_components(row)
    u_max = float(act.u_max)
    keff  = _keff_for_Iyy(Iyy)

    print(f"  keff_target={keff_target:.1f}  keff_actual={keff:.2f}  lat={latency}")

    # Search best gains for each controller type
    Kp_pid,   Kd_pid   = _search_gains(row, u_max, controller='pid')
    Kp_smith, Kd_smith = _search_gains(row, u_max, controller='smith')
    adrc_wc,  _        = _search_gains(row, u_max, controller='adrc')

    print(f"    best gains: PID=({Kp_pid:.1f},{Kd_pid:.2f})  "
          f"Smith=({Kp_smith:.1f},{Kd_smith:.2f})  ADRC wc={adrc_wc:.1f}")

    rows = []
    for Kp in KP_SWEEP:
        # PID
        sr_pid, rms_pid = _eval_kp(Kp, Kd_pid, u_max, row, keff, 'pid')
        # Smith
        sr_smith, rms_smith = _eval_kp(Kp, Kd_smith, u_max, row, keff, 'smith')
        # ADRC (sweep over omega_c using adrc_wc for each Kp-equivalent point)
        sr_adrc, rms_adrc = _eval_kp(adrc_wc, 1.0, u_max, row, keff, 'adrc', adrc_wc=adrc_wc)

        rows.append(dict(
            keff_target=keff_target,
            keff_actual=round(keff, 3),
            latency=latency,
            Kp=round(Kp, 3),
            Kd_pid=round(Kd_pid, 3),
            Kd_smith=round(Kd_smith, 3),
            adrc_wc=round(adrc_wc, 3),
            sr_pid=round(sr_pid, 4),
            sr_smith=round(sr_smith, 4),
            sr_adrc=round(sr_adrc, 4),
            rms_pid=round(rms_pid, 3),
            rms_smith=round(rms_smith, 3),
            rms_adrc=round(rms_adrc, 3),
        ))

    return rows


# ── Summary: extract floor, ceiling, window ───────────────────────────────────

def _window_stats(df_design, controller_col):
    sr = df_design[controller_col].values
    Kp = df_design['Kp'].values
    valid = sr >= SR_THRESH
    if valid.sum() < 2:
        return np.nan, np.nan, np.nan
    floor   = float(Kp[valid].min())
    ceiling = float(Kp[valid].max())
    window  = ceiling / floor if floor > 0 else np.nan
    return floor, ceiling, window


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ROOT = Path(__file__).resolve().parents[1]
    os.chdir(ROOT)

    print("Smith Predictor Test")
    print("=" * 60)
    print(f"Designs: keff={KEFF_TARGETS} x latency={LAT_VALUES}")
    print(f"Kp sweep: {len(KP_SWEEP)} pts [{KP_SWEEP[0]:.1f}, {KP_SWEEP[-1]:.1f}]")
    print(f"Eval seeds: {N_EVAL} per Kp  (seeds {EVAL_SEED0}-{EVAL_SEED0+N_EVAL-1})")
    print()

    all_rows = []
    combos = [(k, l) for k in KEFF_TARGETS for l in LAT_VALUES]

    results = Parallel(n_jobs=N_JOBS, verbose=5)(
        jdelayed(_run_design)(k, l) for k, l in combos
    )
    for chunk in results:
        all_rows.extend(chunk)

    df = pd.DataFrame(all_rows)
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(df)} rows -> {OUT_CSV}")

    # ── Summary table ────────────────────────────────────────────────────────
    print()
    print("=" * 70)
    print("WINDOW SUMMARY  (floor / ceiling / ratio  at SR>=0.80)")
    print("=" * 70)
    print(f"{'keff':>6} {'lat':>4}  {'PID ceil':>9} {'Smith ceil':>11} {'ratio':>7} "
          f"{'ADRC SR':>8}")
    print("-" * 70)

    for k in KEFF_TARGETS:
        for l in LAT_VALUES:
            sub = df[(df['keff_target'] == k) & (df['latency'] == l)]
            if sub.empty:
                continue
            fl_pid, cl_pid, w_pid     = _window_stats(sub, 'sr_pid')
            fl_sm,  cl_sm,  w_sm      = _window_stats(sub, 'sr_smith')
            # ADRC: SR at the best omega_c (single value per design)
            adrc_sr_max = sub['sr_adrc'].max()
            ratio = (cl_sm / cl_pid) if (cl_pid and cl_sm and not np.isnan(cl_pid)) else np.nan
            print(f"{k:>6.1f} {l:>4}  {cl_pid:>9.1f} {cl_sm:>11.1f} {ratio:>7.2f}  "
                  f"{adrc_sr_max:>8.3f}")

    print()
    print("KEY: ratio > 2.0 -> delay-margin is the dominant mechanism")
    print("     ratio < 1.3 -> saturation dynamics dominate (Smith does not help)")
    print()

    # ── Theoretical ceiling prediction ────────────────────────────────────────
    print("Theoretical ceiling (380/lat):")
    for l in LAT_VALUES:
        print(f"  lat={l}: theoretical ceiling = {380/l:.0f}")


if __name__ == '__main__':
    main()
