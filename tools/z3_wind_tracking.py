"""
z3_wind_tracking.py — Does the ADRC ESO's z3 state track actual wind disturbances?

The ESO estimates "total disturbance" z3 ≈ f(t) = wind + model error.
If z3 correlates well with actual d_eff (the wind disturbance injected into the plant),
ADRC implicitly provides a virtual anemometer — a real-time wind estimate from flight dynamics
with no dedicated wind sensor.

Method:
- Run full-physics simulation with ADRC, logging z3 and d_eff at every timestep
- Compute Pearson r(z3, d_eff), scale factor, phase lag
- Test across multiple designs and wind levels

This requires an instrumented simulation loop (not the standard simulate() call).
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

import numpy as np
import pandas as pd

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from controller import ADRCParams, ADRCState, step_adrc, PIDParams, PIDState
from actuator_model import step_actuator, ActuatorState
from plant_dynamics import step_plant
from sensor_model import init_sensor_state, step_sensor
from disturbance_model import init_gust_state, step_disturbance
from simulator import ScenarioParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25
N_SEEDS   = 5
T_END     = 3.0
OMEGA_C   = 5.0
OMEGA0    = 10.0

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)

def keff_full(d):
    return F15_AVG_THRUST_N * d['motor_scale'] * CU_TO_RAD * L_NOZZLE / d['Iyy']


def run_instrumented(design, seed, omega_c=OMEGA_C, omega0=OMEGA0):
    """
    Run full simulation with ADRC and log z3 + d_eff at every timestep.
    Returns arrays (t, z3, d_eff, theta, success).
    """
    plant  = build_plant(design)
    act    = build_actuator(design)
    sen    = build_sensor(design)
    dis    = build_disturbance(design)
    sc     = ScenarioParams(t_end=T_END)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, FIDELITY)

    b0   = keff_full(design)
    adrc = ADRCParams(omega_c=omega_c, omega0=omega0, b0=b0, u_max=act.u_max)

    rng  = np.random.default_rng(seed)
    dt   = plant.dt
    N    = int(round(sc.t_end / dt)) + 1
    t    = np.arange(N) * dt

    theta_arr = np.zeros(N)
    q_arr     = np.zeros(N)
    u_act_arr = np.zeros(N)
    z3_arr    = np.zeros(N)
    d_arr     = np.zeros(N)   # actual disturbance injected into plant

    theta0 = 0.0
    theta_arr[0] = theta0

    act_state  = ActuatorState(u_servo=0.0, u_output=0.0)
    sens_state = init_sensor_state(sen, theta0, rng)
    gust_state = init_gust_state(dis, rng)
    adrc_state = ADRCState(z1=theta0, z2=0.0, z3=0.0, u_prev=0.0)

    theta_ctrl = theta0
    q_ctrl     = 0.0

    # Needed for full-physics
    from aero_model import AeroParams, dynamic_pressure, velocity_step
    from motor_model import MotorParams, MotorState, motor_thrust_and_mdot
    aero_params  = AeroParams.from_design(plant.Cm_alpha_signed, plant.static_margin)
    motor_params = MotorParams.F15(rocket_length=2.0 * plant.l_nozzle_m, scale=plant.motor_scale)
    motor_state  = MotorState.initial(motor_params)
    v_flight     = 0.01

    for k in range(1, N):
        T_now, mdot = motor_thrust_and_mdot(motor_params, t[k])
        motor_state.advance(mdot, dt)
        q_dyn_now = dynamic_pressure(v_flight)
        v_flight = velocity_step(v_flight, plant.m_kg, T_now, theta_arr[k-1],
                                 q_dyn_now, aero_params, dt=dt)

        d_raw = step_disturbance(gust_state, dis, t[k], dt, rng,
                                 inertia_scale=plant.inertia_scale,
                                 mass_scale=plant.mass_scale)
        d_arr[k] = d_raw

        u_cmd = step_adrc(adrc_state, adrc, theta_ctrl, q_ctrl, sc.theta_ref, dt)
        z3_arr[k] = adrc_state.z3  # log AFTER update

        u_act = step_actuator(act_state, act, u_cmd, dt)
        u_act_arr[k] = u_act

        theta_new, q_new = step_plant(
            theta_arr[k-1], q_arr[k-1], u_act, d_raw, plant, 1.0, 1.0,
            aero_params=aero_params, q_dyn=q_dyn_now, v=v_flight,
            T_now=T_now, I_yy_now=plant.Iyy_kgm2, nonlinear_aero=True,
        )
        theta_arr[k] = theta_new
        q_arr[k]     = q_new

        q_ctrl, theta_ctrl = step_sensor(sens_state, sen, q_new, theta_new, dt, rng)

    max_th = float(np.max(np.abs(np.degrees(theta_arr))))
    success = max_th < 15.0

    # Exclude first few timesteps (ESO hasn't converged yet)
    WARMUP = int(0.2 / dt)  # 0.2s warmup
    return t[WARMUP:], z3_arr[WARMUP:], d_arr[WARMUP:], np.degrees(theta_arr[WARMUP:]), success


# ── Load designs ──────────────────────────────────────────────────────────────
df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                              'exp1_regime_index_py.csv'))
fragile  = df[df['regime_label'] == 'FRAGILE'].nlargest(10, 'wind_strength')
marginal = df[df['regime_label'] == 'MARGINAL']
easy     = df[df['regime_label'] == 'EASY'].nlargest(5, 'wind_strength')

test_designs = pd.concat([fragile, marginal, easy]).head(20)
print(f"Testing {len(test_designs)} designs (top-wind FRAGILE + all MARGINAL + top-wind EASY)")
print()

rows = []
for _, row in test_designs.iterrows():
    design = row.to_dict()
    rid    = design['rocket_id']
    regime = design['regime_label']
    ws     = design['wind_strength']

    corrs, scales, lags = [], [], []

    for seed in range(1, N_SEEDS + 1):
        t, z3, d_eff, theta_d, success = run_instrumented(design, seed)

        # Pearson correlation
        if np.std(d_eff) < 1e-6 or np.std(z3) < 1e-6:
            continue
        r = float(np.corrcoef(z3, d_eff)[0, 1])

        # Scale factor (regression slope z3 ~ alpha * d_eff)
        alpha = float(np.dot(z3, d_eff) / (np.dot(d_eff, d_eff) + 1e-9))

        # Phase lag: cross-correlation peak offset
        xcorr = np.correlate(z3 - z3.mean(), d_eff - d_eff.mean(), mode='full')
        lag_steps = int(np.argmax(xcorr)) - (len(z3) - 1)
        lag_s = lag_steps * 0.005  # dt=0.005s

        corrs.append(r)
        scales.append(alpha)
        lags.append(lag_s)

    if not corrs:
        continue

    mean_r     = np.mean(corrs)
    mean_alpha = np.mean(scales)
    mean_lag   = np.mean(lags)
    kf         = keff_full(design)

    print(f"{rid:6s} [{regime:8s}]  wind={ws:.3f}  keff={kf:.1f}  "
          f"r={mean_r:.3f}  scale={mean_alpha:.2f}  lag={mean_lag*1000:.0f}ms")

    rows.append({
        'rocket_id': rid, 'regime': regime, 'wind_strength': ws,
        'keff_full': kf, 'mean_r': mean_r, 'mean_scale': mean_alpha,
        'mean_lag_ms': mean_lag * 1000,
    })

res = pd.DataFrame(rows)
print()
print("=== z3 tracking summary by regime ===")
for regime in ['FRAGILE', 'MARGINAL', 'EASY']:
    sub = res[res['regime'] == regime]
    if len(sub) == 0:
        continue
    print(f"{regime:8s}  n={len(sub)}  "
          f"mean_r={sub['mean_r'].mean():.3f}  "
          f"scale={sub['mean_scale'].mean():.2f}  "
          f"lag={sub['mean_lag_ms'].mean():.0f}ms")

print()
print(f"Overall mean_r = {res['mean_r'].mean():.3f}")
print(f"  r > 0.80: {(res['mean_r'] > 0.80).mean():.0%} of designs")
print(f"  r > 0.90: {(res['mean_r'] > 0.90).mean():.0%} of designs")
print(f"  r > 0.95: {(res['mean_r'] > 0.95).mean():.0%} of designs")
print()

if res['mean_r'].mean() > 0.85:
    print("VERDICT: z3 tracks wind well — virtual anemometer hypothesis is supported")
else:
    print("VERDICT: z3 does NOT reliably track wind — virtual anemometer hypothesis is weak")

res.to_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                        'z3_tracking_py.csv'), index=False)
print("Saved to experiments/results/z3_tracking_py.csv")
