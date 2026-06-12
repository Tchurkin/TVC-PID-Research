"""
adrc_marginal_test.py — Test ADRC on MARGINAL designs (wind-energy-limited).

MARGINAL (n=6): nom_sr < 0.80, robustness=1.00 — wind-limited, not gain-limited.
Best PID gets ~0.667 SR for all 6.

Question: does ADRC fix them (like it fixes FRAGILE), or are they truly physics-limited?
If ADRC cannot fix MARGINAL, that identifies a real energy wall distinct from the
PID architecture problem in FRAGILE.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

import numpy as np
import pandas as pd

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from simulator import simulate, ScenarioParams
from controller import ADRCParams, PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25
N_SEEDS   = 7
T_END     = 3.0

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)

def keff_full(d):
    return F15_AVG_THRUST_N * d['motor_scale'] * CU_TO_RAD * L_NOZZLE / d['Iyy']

def eval_pid(design, Kp, Kd, seeds):
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = ScenarioParams(t_end=T_END)
    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    results = [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in seeds]
    return {
        'sr':       np.mean([r.success for r in results]),
        'rms':      np.mean([r.rms_error_deg for r in results]),
        'slew_sat': np.mean([r.slew_sat_frac for r in results]),
    }

def eval_adrc(design, omega_c, omega0_mult, b0_val, seeds):
    plant  = build_plant(design)
    act    = build_actuator(design)
    sen    = build_sensor(design)
    dis    = build_disturbance(design)
    sc     = ScenarioParams(t_end=T_END)
    omega0 = min(omega_c * omega0_mult, 25.0)
    adrc_p = ADRCParams(omega_c=omega_c, omega0=omega0, b0=b0_val, u_max=act.u_max)
    pid_dummy = PIDParams(Kp=2.0, Kd=1.0, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    results = [simulate(pid_dummy, plant, act2, sen2, dis2, sc2, seed=s, adrc=adrc_p)
               for s in seeds]
    return {
        'sr':       np.mean([r.success for r in results]),
        'rms':      np.mean([r.rms_error_deg for r in results]),
        'slew_sat': np.mean([r.slew_sat_frac for r in results]),
    }

# ── Load data ──────────────────────────────────────────────────────────────
df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                              'exp1_regime_index_py.csv'))
marginal = df[df['regime_label'] == 'MARGINAL'].copy()
seeds = list(range(1, N_SEEDS + 1))

print(f"MARGINAL n={len(marginal)}")
print()

rows = []
for _, row in marginal.iterrows():
    design = row.to_dict()
    rid    = design['rocket_id']
    kf     = keff_full(design)
    theta_ddot = F15_AVG_THRUST_N * design['motor_scale'] * np.sin(np.deg2rad(design['max_gimbal_deg'])) * L_NOZZLE / design['Iyy']

    print(f"{rid}  keff={kf:.2f}  theta_ddot_max={theta_ddot:.1f}  wind={design['wind_strength']:.3f}  slew={design['servo_slew_deg_s']:.0f}")

    pid2    = eval_pid(design, Kp=2.0,                 Kd=1.0,                    seeds=seeds)
    pidbest = eval_pid(design, Kp=design['best_Kp'],   Kd=design['best_Kd'],      seeds=seeds)
    adrc5   = eval_adrc(design, omega_c=5,  omega0_mult=2.0, b0_val=kf, seeds=seeds)
    adrc10  = eval_adrc(design, omega_c=10, omega0_mult=2.5, b0_val=kf, seeds=seeds)
    adrc20  = eval_adrc(design, omega_c=20, omega0_mult=1.2, b0_val=kf, seeds=seeds)  # higher bandwidth

    print(f"  PID@2    SR={pid2['sr']:.3f}  slew={pid2['slew_sat']:.3f}  RMS={pid2['rms']:.1f}")
    print(f"  PID@best SR={pidbest['sr']:.3f}  slew={pidbest['slew_sat']:.3f}  RMS={pidbest['rms']:.1f}")
    print(f"  ADRC wc=5  SR={adrc5['sr']:.3f}  slew={adrc5['slew_sat']:.3f}  RMS={adrc5['rms']:.1f}")
    print(f"  ADRC wc=10 SR={adrc10['sr']:.3f}  slew={adrc10['slew_sat']:.3f}  RMS={adrc10['rms']:.1f}")
    print(f"  ADRC wc=20 SR={adrc20['sr']:.3f}  slew={adrc20['slew_sat']:.3f}  RMS={adrc20['rms']:.1f}")
    print()

    rows.append({
        'rocket_id': rid, 'keff_full': kf, 'theta_ddot_max': theta_ddot,
        'wind': design['wind_strength'], 'Iyy': design['Iyy'],
        'slew': design['servo_slew_deg_s'],
        'pid2_sr': pid2['sr'], 'pidbest_sr': pidbest['sr'],
        'adrc5_sr': adrc5['sr'], 'adrc5_slew': adrc5['slew_sat'],
        'adrc10_sr': adrc10['sr'], 'adrc10_slew': adrc10['slew_sat'],
        'adrc20_sr': adrc20['sr'], 'adrc20_slew': adrc20['slew_sat'],
        'adrc5_rms': adrc5['rms'], 'adrc10_rms': adrc10['rms'],
    })

res = pd.DataFrame(rows)

print("=== MARGINAL summary ===")
print(f"PID@2       mean SR: {res['pid2_sr'].mean():.3f}")
print(f"PID@best    mean SR: {res['pidbest_sr'].mean():.3f}")
print(f"ADRC wc=5   mean SR: {res['adrc5_sr'].mean():.3f}  (mean slew_sat={res['adrc5_slew'].mean():.3f})")
print(f"ADRC wc=10  mean SR: {res['adrc10_sr'].mean():.3f}  (mean slew_sat={res['adrc10_slew'].mean():.3f})")
print(f"ADRC wc=20  mean SR: {res['adrc20_sr'].mean():.3f}  (mean slew_sat={res['adrc20_slew'].mean():.3f})")

res.to_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                        'adrc_marginal_py.csv'), index=False)
print("\nSaved to experiments/results/adrc_marginal_py.csv")
