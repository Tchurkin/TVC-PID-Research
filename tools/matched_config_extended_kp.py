"""
tools/matched_config_extended_kp.py

Same single-airframe experiment as matched_configuration_test.py, but Kp swept
to 1280 (4x the previous 320 cap) to find the TRUE ceiling for high-authority
designs.  The original experiment showed ceiling=320 (grid cap) for the three
highest-td configurations, leaving open whether the window really narrows or
whether the floor is rising while the ceiling stays high.

Key question: at latency=3, does Kp=500-1280 still achieve SR>=0.80 for
td=89-189 designs, or does SR collapse somewhere in that range?

If ceiling is genuinely > 320:
  - window narrowing is floor-driven only (floor rises, ceiling stays high)
  - window ratio from original data is a lower bound, not a measurement
If ceiling falls at some Kp > 320:
  - true ceiling found; window is narrower than the original data showed

Uses fresh seeds (13001+) disjoint from original matched config (11001+).
static_margin now uses aerospace convention: +0.10 = slightly stable (CP aft of CG).

Output: experiments/results/matched_config_extended_kp_py.csv
        experiments/results/matched_config_extended_kp_summary_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# Identical airframe to matched_configuration_test.py, static_margin sign fixed
FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.5, servo_slew_deg_s=120.0,
    max_gimbal_deg=10.0, latency_steps=3, deadband=0.05, backlash=0.10,
    wind_strength=0.20,
)

IYY_VALUES = np.geomspace(0.005, 0.100, 18)   # 18 points for smoother curve
KP_GRID    = np.geomspace(1.0, 1280.0, 40)    # 40 points up to 4x the old cap
KD_FIXED   = 2.0
N_SEEDS    = 10
SEED_START = 13001   # fresh, disjoint from original (11001-)
SR_THRESH  = 0.80

OUT_CSV     = Path('experiments/results/matched_config_extended_kp_py.csv')
SUMMARY_CSV = Path('experiments/results/matched_config_extended_kp_summary_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def compute_td(Iyy: float) -> float:
    T_avg = F15_AVG_THRUST_N * FIXED['motor_scale']
    keff_full = T_avg * CU_TO_RAD * L_NOZZLE / Iyy
    u_max = FIXED['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff_full * u_max)


def _eval_point(Iyy: float, Kp: float) -> dict:
    row = dict(FIXED)
    row['Iyy'] = Iyy
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    pid  = PIDParams(Kp=Kp, Kd=KD_FIXED, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s)
            for s in range(SEED_START, SEED_START + N_SEEDS)]
    agg  = _aggregate(runs)
    return dict(Iyy=Iyy, td=compute_td(Iyy), Kp=Kp, sr=agg['success_rate'], rms=agg['rms_error_deg'])


def main():
    tasks = [(iyy, kp) for iyy in IYY_VALUES for kp in KP_GRID]
    total_sims = len(tasks) * N_SEEDS
    print(f"Extended Kp sweep: {len(IYY_VALUES)} Iyy x {len(KP_GRID)} Kp x {N_SEEDS} seeds = {total_sims} sims")
    print(f"Kp range: [{KP_GRID[0]:.1f}, {KP_GRID[-1]:.1f}]  (old cap was 320)")
    print(f"td range: [{compute_td(IYY_VALUES.max()):.1f}, {compute_td(IYY_VALUES.min()):.1f}] rad/s^2")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_point)(iyy, kp) for iyy, kp in tasks
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved raw: {OUT_CSV}")

    print("\n=== TRUE FLOOR / CEILING / WINDOW (Kp up to 1280) ===")
    rows = []
    for iyy in IYY_VALUES:
        sub = out[out['Iyy'].round(6) == round(iyy, 6)].sort_values('Kp')
        td  = compute_td(iyy)
        passing = sub[sub['sr'] >= SR_THRESH]
        if len(passing) == 0:
            floor_kp = ceil_kp = float('nan')
            window = 0.0
            ceil_capped = False
        else:
            floor_kp = float(passing['Kp'].min())
            ceil_kp  = float(passing['Kp'].max())
            window   = ceil_kp / floor_kp if floor_kp > 0 else float('inf')
            ceil_capped = (ceil_kp >= KP_GRID[-1] * 0.99)

        cap_note = " [CAPPED - true ceiling > 1280]" if ceil_capped else ""
        print(f"  Iyy={iyy:.4f} td={td:6.1f}  floor={floor_kp:6.1f}  "
              f"ceil={ceil_kp:6.1f}  window={window:7.1f}x{cap_note}")
        rows.append(dict(Iyy=iyy, td=td, floor_kp=floor_kp, ceil_kp=ceil_kp,
                         window_ratio=window, ceil_capped=ceil_capped))

    summary = pd.DataFrame(rows)
    summary.to_csv(SUMMARY_CSV, index=False)
    print(f"\nSaved summary: {SUMMARY_CSV}")

    # Key result: are the high-td ceilings still capped?
    high_td = summary[summary['td'] >= 80].sort_values('td')
    print("\n=== KEY RESULT: high-td designs (td >= 80) ===")
    n_capped = high_td['ceil_capped'].sum()
    if n_capped > 0:
        print(f"  {n_capped}/{len(high_td)} designs still hit the 1280 ceiling cap.")
        print("  Ceiling is genuinely above 1280; window narrowing is purely FLOOR-DRIVEN.")
    else:
        print(f"  All {len(high_td)} high-td designs found a TRUE ceiling within [1, 1280].")
        print("  Window narrowing has BOTH a floor and a ceiling component.")
    for _, r in high_td.iterrows():
        status = "CAPPED" if r['ceil_capped'] else f"true ceil={r['ceil_kp']:.0f}"
        print(f"    td={r['td']:.1f}  floor={r['floor_kp']:.1f}  ceil: {status}  window={r['window_ratio']:.1f}x")


if __name__ == '__main__':
    main()
