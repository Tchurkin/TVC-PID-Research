"""
tools/adrc_bandwidth_ceiling.py

Novel cross-architecture design rule: PID has an empirically established gain
ceiling law, Kp_max ~ 380/latency_steps (CLAUDE.md "GAIN CEILING EQUATION"),
derived from a DIPDT phase-margin argument and confirmed by the boundary
experiment. ADRC's closed-loop bandwidth omega_c plays the same structural
role as PID's Kp (it sets how fast the controller reacts), and the ADRC
dissolution test (Section 6 of the paper / Finding 7) already showed that its
two residual failures were fixed by LOWERING omega_c for high-latency designs
-- i.e. the same delay-driven mechanism. This script tests whether that
mechanism is QUANTIFIABLE: does ADRC have an analogous omega_c_max(latency)
ceiling law, parallel to PID's Kp_max(latency)?

If so, this completes a genuinely novel, controller-agnostic design rule a
builder can use regardless of which architecture they choose:
    PID:  Kp in [Kp_floor, Kp_max(latency)]
    ADRC: omega_c <= omega_c_max(latency)

Method
------
3 representative hardware authority levels (low/med/high theta_ddot_max,
picked from the existing combined regression dataset) x latency_steps in
{1,2,3,4,6,8,10,12} (extended past 6 to cover the Arduino-class stress-test
range already established in CLAUDE.md) x a log-spaced omega_c grid, omega0
fixed at 5x omega_c (mid-point of the standard 3-10x ESO bandwidth
multiplier), 10 seeds, full physics with wind. For each (authority, latency)
cell, find the largest omega_c with success rate >= 0.80 -- the ADRC ceiling,
exactly analogous to the PID Kp_max definition used throughout this project.

Output: experiments/results/adrc_bandwidth_ceiling_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario, sample_lhs
from controller import PIDParams, ADRCParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

LATENCIES   = [1, 2, 3, 4, 6, 8, 10, 12]
OMEGA_C_GRID = [2, 3, 5, 7, 10, 15, 20, 30, 50, 80, 120]
OMEGA0_RATIO = 5.0
N_SEEDS     = 10
SEED_START  = 9001
TD_TARGETS  = [20.0, 90.0, 200.0]   # low / medium / high authority

OUT_CSV = Path('experiments/results/adrc_bandwidth_ceiling_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def compute_td(row: dict) -> float:
    T_avg = F15_AVG_THRUST_N * row['motor_scale']
    keff_full = T_avg * CU_TO_RAD * L_NOZZLE / row['Iyy']
    u_max = row['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff_full * u_max)


def pick_reference_designs():
    pool = sample_lhs(2000, seed=555)
    pool['td'] = pool.apply(lambda r: compute_td(r.to_dict()), axis=1)
    chosen = []
    for target in TD_TARGETS:
        idx = (pool['td'] - target).abs().idxmin()
        chosen.append(pool.loc[idx].to_dict())
    return chosen


def _eval_cell(design: dict, latency: int, omega_c: float) -> dict:
    row = dict(design)
    row['latency_steps'] = latency
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    b0 = float(row['motor_scale']) * F15_AVG_THRUST_N * CU_TO_RAD * L_NOZZLE / float(row['Iyy'])
    adrc = ADRCParams(omega_c=omega_c, omega0=OMEGA0_RATIO * omega_c, b0=b0, u_max=act.u_max)
    seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    runs = [simulate(PIDParams(Kp=0.0, Kd=0.0, u_max=act.u_max), plant, act, sen, dis, sc,
                      seed=s, adrc=adrc) for s in seeds]
    sr = float(np.mean([r.success for r in runs]))
    rms = float(np.mean([r.rms_error_deg for r in runs]))
    return dict(td=row['td'], latency_steps=latency, omega_c=omega_c, sr=sr, rms=rms)


def main():
    designs = pick_reference_designs()
    for d in designs:
        print(f"Reference design: td={d['td']:.1f} rad/s^2  Iyy={d['Iyy']:.4f}  "
              f"motor_scale={d['motor_scale']:.2f}  max_gimbal={d['max_gimbal_deg']:.1f}")

    tasks = [(d, lat, wc) for d in designs for lat in LATENCIES for wc in OMEGA_C_GRID]
    print(f"\nTotal cells: {len(tasks)} x {N_SEEDS} seeds = {len(tasks)*N_SEEDS} sims")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_cell)(d, lat, wc) for d, lat, wc in tasks
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)

    # For each (td, latency), find the largest omega_c with sr >= 0.80
    rows = []
    for (td, lat), sub in out.groupby(['td', 'latency_steps']):
        sub = sub.sort_values('omega_c')
        passing = sub[sub['sr'] >= 0.80]
        wc_max = float(passing['omega_c'].max()) if len(passing) else 0.0
        rows.append(dict(td=td, latency_steps=lat, omega_c_max=wc_max))
    ceiling = pd.DataFrame(rows)
    ceiling.to_csv('experiments/results/adrc_bandwidth_ceiling_summary_py.csv', index=False)

    print("\n=== ADRC BANDWIDTH CEILING (omega_c_max by td, latency) ===")
    print(ceiling.pivot(index='latency_steps', columns='td', values='omega_c_max').to_string())

    from scipy.stats import pearsonr
    valid = ceiling[ceiling['omega_c_max'] > 0]
    log_lat = np.log(valid['latency_steps'])
    log_wc  = np.log(valid['omega_c_max'])
    r, p = pearsonr(log_lat, log_wc)
    slope, intercept = np.polyfit(log_lat, log_wc, 1)
    print(f"\nlog-log fit: omega_c_max ~ {np.exp(intercept):.2f} x latency^{slope:.2f}  (r={r:.3f}, p={p:.2e})")

    print(f"\nSaved: {OUT_CSV}")
    print("Saved: experiments/results/adrc_bandwidth_ceiling_summary_py.csv")


if __name__ == '__main__':
    main()
