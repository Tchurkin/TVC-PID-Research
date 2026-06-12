"""
simulator.py — Main simulation loop for 1-DOF TVC rocket.

This module is the single integration point for all sub-models.
No physics, control law, or sensor logic lives here — only orchestration.

Call graph per timestep k
─────────────────────────
  disturbance_model.step_disturbance()  →  d_eff
  controller.step_pid()                 →  u_cmd
  actuator_model.step_actuator()        →  u_act
  plant_dynamics.step_plant()           →  (theta, q)
  sensor_model.step_sensor()            →  (theta_ctrl, q_ctrl)

The controller always reads the PREVIOUS step's delayed sensor output
(the latency buffer from step k-1) so that the pipeline is causal.

Fault injection
───────────────
  ScenarioParams supports optional midburn faults:
      fault_time_s        — when the fault activates
      keff_fault_post     — control effectiveness multiplier after fault
      damp_fault_post     — aero damping multiplier after fault
      disturb_bias_post   — additional constant disturbance after fault (rad/s²)
  Default values keep these disabled (keff_fault=1.0, damp_fault=1.0, etc.).

Outputs
───────
  SimResult contains:
      trajectory arrays: t, theta_true, q_true, u_act, theta_meas, q_meas
      scalar metrics:    success, rms_error_deg, peak_error_deg, end_error_deg,
                         max_theta_deg, u_cmd_sat_frac, slew_sat_frac,
                         settling_time_s, oscillation_score
  All angle outputs are in degrees.  Arrays are shape (N,) with N = t_end/dt + 1.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from units import (
    SUCCESS_MAX_THETA_DEG, SUCCESS_END_ERROR_DEG,
    SUCCESS_RMS_ERROR_DEG, SUCCESS_PEAK_ERROR_DEG,
    DIVERGE_THETA_DEG, BAND_30_THETA_DEG, BAND_20_THETA_DEG, BAND_10_THETA_DEG,
    F15_AVG_THRUST_N,
)
from plant_dynamics import PlantParams, step_plant
from actuator_model import ActuatorParams, ActuatorState, step_actuator
from sensor_model import SensorParams, SensorState, init_sensor_state, step_sensor
from disturbance_model import DisturbanceParams, GustState, init_gust_state, step_disturbance
from controller import PIDParams, PIDState, step_pid, ADRCParams, ADRCState, step_adrc


@dataclass
class ScenarioParams:
    t_end: float = 3.0                  # burn duration (s)
    theta_ref: float = 0.0              # constant pitch reference (rad); ignored if theta_step_deg != 0
    theta0_bias_std: float = 0.0        # std of random initial angle offset (rad)
    theta0_fixed_deg: float = 0.0       # deterministic initial angle (deg); overrides bias when non-zero
    # Optional step command (pitch tracking task)
    theta_step_deg: float = 0.0         # commanded step magnitude (deg); 0 = attitude hold only
    theta_step_time_s: float = float("inf")  # time at which step is commanded (s)
    # Optional midburn fault
    fault_time_s: float = float("inf")  # time at which fault activates (s)
    keff_fault_post: float = 1.0        # keff multiplier after fault
    damp_fault_post: float = 1.0        # aero damp multiplier after fault
    disturb_bias_post: float = 0.0      # extra constant disturbance after fault (rad/s²)


@dataclass
class SimResult:
    # Trajectory arrays (length N)
    t: np.ndarray
    theta_true: np.ndarray
    q_true: np.ndarray
    u_act: np.ndarray
    theta_meas: np.ndarray
    q_meas: np.ndarray
    # Scalar metrics
    success: bool
    rms_error_deg: float
    peak_error_deg: float
    end_error_deg: float
    max_theta_deg: float
    u_cmd_sat_frac: float
    slew_sat_frac: float
    settling_time_s: float
    oscillation_score: float
    # Divergence and progressive tracking bands
    diverged: bool       # max |theta| >= 90 deg — rocket out of control
    band_30_pass: bool   # max |theta| < 30 deg
    band_20_pass: bool   # max |theta| < 20 deg
    band_10_pass: bool   # max |theta| < 10 deg
    # Step tracking metrics (0.0 when no step command given)
    tracking_rms_deg: float = 0.0   # RMS error during post-step tracking phase
    rise_time_s: float = 0.0        # time from step to first crossing 90% of command
    overshoot_deg: float = 0.0      # max exceedance above command magnitude
    step_ref_arr: np.ndarray = None  # time-varying reference (None if constant)


def simulate(
    pid: PIDParams,
    plant: PlantParams,
    actuator: ActuatorParams,
    sensor: SensorParams,
    disturbance: DisturbanceParams,
    scenario: ScenarioParams,
    seed: int = 1,
    adrc: "ADRCParams | None" = None,
) -> SimResult:
    """
    Run one complete simulation from t=0 to t=t_end.

    Parameters are immutable after the call.  All random state is seeded from
    `seed`; same seed → identical trajectory (determinism guarantee).

    Returns a SimResult with trajectory arrays and scalar metrics.
    """
    rng = np.random.default_rng(seed)

    dt = plant.dt
    N = int(round(scenario.t_end / dt)) + 1
    t = np.arange(N) * dt

    # ── Detect high-fidelity physical modules ─────────────────────────────────
    # These are set on the scenario by apply_fidelity_config() when new flags are on.
    use_dyn_aero    = getattr(scenario, "use_dyn_aero",    False)
    use_thrust_curve = getattr(scenario, "use_thrust_curve", False)
    use_cg_shift    = getattr(scenario, "use_cg_shift",    False)
    use_nonlinear   = getattr(scenario, "use_nonlinear_aero", False)

    # Import new physics modules only when needed
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
        motor_state  = MotorState.initial(motor_params)

    # Flight state for dynamic pressure integration
    v_flight = 0.01  # m/s — small nonzero to avoid division by zero in damping

    # ── Allocate output arrays ────────────────────────────────────────────────
    theta_arr = np.zeros(N)
    q_arr     = np.zeros(N)
    u_act_arr = np.zeros(N)
    theta_m_arr = np.zeros(N)
    q_m_arr     = np.zeros(N)

    # ── Initial conditions ────────────────────────────────────────────────────
    if scenario.theta0_fixed_deg != 0.0:
        theta0 = np.deg2rad(scenario.theta0_fixed_deg)
    else:
        theta0 = scenario.theta0_bias_std * rng.standard_normal()
    theta_arr[0]   = theta0
    q_arr[0]       = 0.0
    u_act_arr[0]   = 0.0

    # ── Initialise sub-model states ───────────────────────────────────────────
    act_state   = ActuatorState(u_servo=0.0, u_output=0.0)
    sens_state  = init_sensor_state(sensor, theta0, rng)
    # Stationary gust initialisation — draws from N(0, gust_std) at t=0
    gust_state  = init_gust_state(disturbance, rng)
    pid_state   = PIDState(i_state=0.0, q_ctrl_prev=0.0)
    # ADRC state: initialise z1=theta0 so observer starts at true initial angle
    adrc_state  = ADRCState(z1=theta0, z2=0.0, z3=0.0, u_prev=0.0) if adrc is not None else None

    # Seed measurements at k=0 from initial condition
    theta_m_arr[0] = sens_state.theta_integrated
    q_m_arr[0]     = 0.0

    # The controller sees the PREVIOUS step's delayed measurements.
    # At k=1 the latency buffer holds the initial condition.
    theta_ctrl = theta_m_arr[0]
    q_ctrl     = q_m_arr[0]

    # ── Build time-varying reference trajectory ───────────────────────────────
    has_step = scenario.theta_step_deg != 0.0
    if has_step:
        step_rad = np.deg2rad(scenario.theta_step_deg)
        ref_arr  = np.where(t >= scenario.theta_step_time_s, step_rad, scenario.theta_ref)
    else:
        ref_arr  = None  # use scalar scenario.theta_ref in loop (avoids per-step indexing cost)

    # ── Main loop ─────────────────────────────────────────────────────────────
    for k in range(1, N):
        post_fault = t[k] >= scenario.fault_time_s
        keff_fault = scenario.keff_fault_post if post_fault else 1.0
        damp_fault = scenario.damp_fault_post if post_fault else 1.0
        bias_post  = scenario.disturb_bias_post if post_fault else 0.0

        # ── Resolve current-step physical quantities ──────────────────────────
        # These are constant in simple mode and time-varying in full mode.

        if use_thrust_curve and motor_params is not None:
            T_now, mdot = motor_thrust_and_mdot(motor_params, t[k])
            motor_state.advance(mdot, dt)
        else:
            # All designs use the F15 motor. When thrust_curve is off, model as
            # constant average thrust (F15 T_avg = 14.4 N). T_now only affects
            # dynamics when dyn_aero=True; in simple mode it is unused.
            T_now = F15_AVG_THRUST_N
            mdot  = 0.0

        if use_cg_shift and motor_params is not None and motor_state is not None:
            m_dry_casing = plant.m_kg + motor_params.m_casing
            m_total  = motor_vehicle_mass(plant.m_kg, motor_params, motor_state)
            x_cg_now = motor_cg_position(
                plant.l_nozzle_m,  # CG from nose = rocket_length − l_nozzle_m = l_nozzle_m
                m_dry_casing, motor_params, motor_state
            )
            I_yy_now = motor_inertia_correction(
                plant.Iyy_kgm2, plant.l_nozzle_m,
                m_dry_casing, motor_params, motor_state, x_cg_now
            )
        else:
            m_total  = plant.m_kg
            I_yy_now = plant.Iyy_kgm2

        if use_dyn_aero and aero_params is not None:
            q_dyn_now = dynamic_pressure(v_flight)
            # Advance velocity (translational dynamics)
            v_flight = velocity_step(
                v_flight, m_total, T_now,
                theta_arr[k - 1], q_dyn_now, aero_params, dt=dt
            )
        else:
            # Design-appropriate constant reference dynamic pressure.
            # Estimate average flight speed from net thrust and mass so that
            # the dyn_aero ablation measures "does q_dyn vary with speed?"
            # rather than "is the aerodynamic calibration correct?".
            #
            # Simple ballistic estimate: a_net = (T - m*g) / m, v_mid = a_net * t_end/2.
            # Clamped to a minimum of 0.5 m/s² net acceleration (covers T/W < 1 cases).
            T_eff  = F15_AVG_THRUST_N * plant.motor_scale
            a_net  = max(0.5, (T_eff - plant.m_kg * 9.81) / max(0.1, plant.m_kg))
            v_mid  = a_net * (scenario.t_end * 0.5)   # velocity at burn midpoint
            q_dyn_now = 0.5 * 1.20 * v_mid**2

        # 1. Disturbance
        d_raw = step_disturbance(
            gust_state, disturbance, t[k], dt, rng,
            inertia_scale=plant.inertia_scale,
            mass_scale=plant.mass_scale,
        )
        d_eff = d_raw + bias_post * plant.inertia_scale / max(0.5, plant.mass_scale)

        # 2. Controller (uses previous step's delayed measurements)
        theta_ref_k = ref_arr[k] if has_step else scenario.theta_ref
        if adrc is not None and adrc_state is not None:
            u_cmd = step_adrc(adrc_state, adrc, theta_ctrl, q_ctrl, theta_ref_k, dt)
        else:
            u_cmd = step_pid(pid_state, pid, theta_ctrl, q_ctrl, theta_ref_k, dt)

        # 3. Actuator
        u_act = step_actuator(act_state, actuator, u_cmd, dt)
        u_act_arr[k] = u_act

        # 4. Plant — route to simple or full physics based on scenario flags
        theta_new, q_new = step_plant(
            theta_arr[k - 1], q_arr[k - 1], u_act, d_eff,
            plant, keff_fault, damp_fault,
            # High-fidelity args (None → simple mode if flags are off)
            aero_params  = aero_params if (use_dyn_aero or use_nonlinear) else None,
            q_dyn        = q_dyn_now   if (use_dyn_aero or use_nonlinear) else None,
            v            = v_flight    if (use_dyn_aero or use_nonlinear) else None,
            T_now        = T_now       if (use_dyn_aero or use_nonlinear) else None,
            I_yy_now     = I_yy_now    if (use_dyn_aero or use_nonlinear) else None,
            nonlinear_aero = use_nonlinear,
        )
        theta_arr[k] = theta_new
        q_arr[k]     = q_new

        # 5. Sensor (advances latency buffer, returns delayed measurements)
        q_ctrl, theta_ctrl = step_sensor(
            sens_state, sensor, q_new, theta_new, dt, rng
        )
        q_m_arr[k]     = q_ctrl
        theta_m_arr[k] = theta_ctrl

    # ── Metrics ───────────────────────────────────────────────────────────────
    theta_d     = np.degrees(theta_arr)
    ref_arr_d   = np.degrees(ref_arr) if has_step else np.full(N, np.degrees(scenario.theta_ref))
    theta_ref_d = ref_arr_d  # used for error computation
    err_d       = theta_d - ref_arr_d

    rms_e  = float(np.sqrt(np.mean(err_d ** 2)))
    peak_e = float(np.max(np.abs(err_d)))
    end_e  = float(np.abs(err_d[-1]))
    max_th = float(np.max(np.abs(theta_d)))

    diverged     = max_th >= DIVERGE_THETA_DEG
    band_30_pass = max_th < BAND_30_THETA_DEG
    band_20_pass = max_th < BAND_20_THETA_DEG
    band_10_pass = max_th < BAND_10_THETA_DEG

    success = (
        max_th  < SUCCESS_MAX_THETA_DEG
        and end_e   < SUCCESS_END_ERROR_DEG
        and rms_e   < SUCCESS_RMS_ERROR_DEG
        and peak_e  < SUCCESS_PEAK_ERROR_DEG
    )

    # Saturation fractions
    u_sat_frac = float(np.mean(np.abs(u_act_arr) >= 0.95 * actuator.u_max))
    if actuator.slew_max < 1e6:
        slew_rates = np.abs(np.diff(u_act_arr)) / dt
        slew_sat_frac = float(np.mean(slew_rates >= 0.95 * actuator.slew_max))
    else:
        slew_sat_frac = 0.0

    # Settling time: last time |theta| ≥ 2 deg
    settled = np.abs(theta_d) < 2.0
    settling_time = float(t[-1])
    for i in range(N - 1, -1, -1):
        if not settled[i]:
            settling_time = float(t[min(i + 1, N - 1)])
            break
    else:
        settling_time = 0.0

    # Oscillation score: zero-crossing count of tracking error
    es = np.sign(err_d)
    nz = es[es != 0]
    osc_score = float(np.sum(np.abs(np.diff(nz)) > 0)) if len(nz) > 1 else 0.0

    # ── Step tracking metrics ─────────────────────────────────────────────────
    tracking_rms = 0.0
    rise_time    = 0.0
    overshoot    = 0.0
    if has_step and scenario.theta_step_time_s < t[-1]:
        step_idx = int(round(scenario.theta_step_time_s / dt))
        post_mask = t >= scenario.theta_step_time_s + 0.3  # skip first 0.3s transient
        if post_mask.any():
            tracking_rms = float(np.sqrt(np.mean(err_d[post_mask] ** 2)))
        # Rise time: first crossing of 90% of step command
        target_90pct = 0.9 * scenario.theta_step_deg
        crossed = np.where(
            (np.degrees(theta_arr) - np.degrees(scenario.theta_ref)) * np.sign(scenario.theta_step_deg)
            >= abs(target_90pct)
        )[0]
        if len(crossed) > 0:
            first_cross = crossed[crossed >= step_idx]
            rise_time = float(t[first_cross[0]] - scenario.theta_step_time_s) if len(first_cross) > 0 else float(t[-1])
        else:
            rise_time = float(t[-1])  # never reached
        # Overshoot: max exceedance above command
        step_d = scenario.theta_step_deg
        if step_d > 0:
            overshoot = max(0.0, float(np.max(theta_d[step_idx:])) - step_d)
        else:
            overshoot = max(0.0, step_d - float(np.min(theta_d[step_idx:])))

    return SimResult(
        t=t,
        theta_true=theta_arr,
        q_true=q_arr,
        u_act=u_act_arr,
        theta_meas=theta_m_arr,
        q_meas=q_m_arr,
        success=success,
        rms_error_deg=rms_e,
        peak_error_deg=peak_e,
        end_error_deg=end_e,
        max_theta_deg=max_th,
        u_cmd_sat_frac=u_sat_frac,
        slew_sat_frac=slew_sat_frac,
        settling_time_s=settling_time,
        oscillation_score=osc_score,
        diverged=diverged,
        band_30_pass=band_30_pass,
        band_20_pass=band_20_pass,
        band_10_pass=band_10_pass,
        tracking_rms_deg=tracking_rms,
        rise_time_s=rise_time,
        overshoot_deg=overshoot,
        step_ref_arr=ref_arr_d if has_step else None,
    )
