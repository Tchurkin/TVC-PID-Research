"""
tools/adrc_ceiling_extended.py — Expanded ADRC bandwidth ceiling sweep.

Extends adrc_bandwidth_ceiling.py from 3 authority reference levels to 6,
giving ~48 valid cells vs the original ~21. The goal is to check whether the
power law omega_c_max ~ 648 / (latency^0.66 * td^0.77) holds on a larger
sample and whether the exponents are stable.

New TD_TARGETS: [20, 50, 90, 130, 200, 300]  (6 levels vs original 3)
Same latencies [1,2,3,4,6,8,10,12] and omega_c grid.
Same omega0 = 5 x omega_c (fixed ratio).
Same N_SEEDS=10, full physics.

Uses DIFFERENT design seeds (pool seed=888) and DIFFERENT evaluation seeds
(SEED_START=9101) from the original tool to ensure independence.

Output:
  experiments/results/adrc_ceiling_extended_py.csv   (raw per-cell)
  experiments/results/adrc_ceiling_extended_summary_py.csv  (ceiling per cell)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario, sample_lhs
from controller import PIDParams, ADRCParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# ── Extended settings ─────────────────────────────────────────────────────────
LATENCIES    = [1, 2, 3, 4, 6, 8, 10, 12]
OMEGA_C_GRID = [2, 3, 5, 7, 10, 15, 20, 30, 50, 80, 120]
OMEGA0_RATIO = 5.0
N_SEEDS      = 10
SEED_START   = 9101     # different from original 9001
TD_TARGETS   = [20.0, 50.0, 90.0, 130.0, 200.0, 300.0]   # 6 levels

RAW_CSV     = Path('experiments/results/adrc_ceiling_extended_py.csv')
SUMMARY_CSV = Path('experiments/results/adrc_ceiling_extended_summary_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def compute_td(row: dict) -> float:
    T_avg = F15_AVG_THRUST_N * row['motor_scale']
    keff  = T_avg * CU_TO_RAD * L_NOZZLE / row['Iyy']
    u_max = row['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff * u_max)


def pick_reference_designs():
    pool = sample_lhs(3000, seed=888)
    pool['td'] = pool.apply(lambda r: compute_td(r.to_dict()), axis=1)
    chosen = []
    for target in TD_TARGETS:
        idx = (pool['td'] - target).abs().idxmin()
        row = pool.loc[idx].to_dict()
        chosen.append(row)
        print(f"  target={target:.0f}  found td={row['td']:.1f}  "
              f"Iyy={row['Iyy']:.4f}  motor_scale={row['motor_scale']:.2f}  "
              f"max_gimbal={row['max_gimbal_deg']:.1f}")
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

    b0 = (F15_AVG_THRUST_N * float(row['motor_scale'])
          * CU_TO_RAD * L_NOZZLE / float(row['Iyy']))
    adrc = ADRCParams(omega_c=omega_c, omega0=OMEGA0_RATIO * omega_c,
                      b0=b0, u_max=act.u_max)
    pid_dummy = PIDParams(Kp=0.0, Kd=0.0, u_max=act.u_max)
    seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    runs = [simulate(pid_dummy, plant, act, sen, dis, sc, seed=s, adrc=adrc)
            for s in seeds]
    sr   = float(np.mean([r.success for r in runs]))
    rms  = float(np.mean([r.rms_error_deg for r in runs]))
    return dict(td_target=design['td'], td_actual=row['td'],
                latency_steps=latency, omega_c=omega_c, sr=sr, rms=rms,
                Iyy=row['Iyy'], motor_scale=row['motor_scale'])


def extract_ceiling(cell_df, sr_thresh=0.80):
    """For each (td, latency) cell, find max omega_c achieving SR >= thresh."""
    summary = []
    for (td, lat), grp in cell_df.groupby(['td_actual', 'latency_steps']):
        passing = grp[grp['sr'] >= sr_thresh]['omega_c']
        ceiling = passing.max() if len(passing) else float('nan')
        summary.append({
            'td':           td,
            'latency_steps': lat,
            'omega_c_max':  ceiling,
            'n_passing':    len(passing),
            'max_sr_found': grp['sr'].max(),
        })
    return pd.DataFrame(summary)


def fit_power_law(summary_df):
    """Fit log(omega_c_max) ~ a*log(latency) + b*log(td) + c."""
    valid = summary_df.dropna(subset=['omega_c_max'])
    valid = valid[valid['omega_c_max'] > 0]
    if len(valid) < 5:
        print("Insufficient valid cells for regression.")
        return

    log_omax = np.log(valid['omega_c_max'])
    log_lat  = np.log(valid['latency_steps'].astype(float))
    log_td   = np.log(valid['td'])

    X = np.column_stack([log_lat, log_td, np.ones(len(valid))])
    b = np.linalg.lstsq(X, log_omax.values, rcond=None)[0]
    pred  = X @ b
    ss_res = np.sum((log_omax.values - pred)**2)
    ss_tot = np.sum((log_omax.values - log_omax.mean())**2)
    r2 = 1 - ss_res / ss_tot
    C  = np.exp(b[2])

    print(f"\n=== Power law fit (n={len(valid)} valid cells) ===")
    print(f"  omega_c_max ~ {C:.0f} / (latency^{-b[0]:.2f} * td^{-b[1]:.2f})")
    print(f"  exponents: latency={b[0]:.3f}, td={b[1]:.3f}, intercept={b[2]:.3f}")
    print(f"  R2 = {r2:.3f}")
    print(f"\n  Original result (n=21): ~ 648 / (lat^0.66 * td^0.77)")
    print(f"  Extended result (n={len(valid)}): ~ {C:.0f} / (lat^{-b[0]:.2f} * td^{-b[1]:.2f})")

    # Compare to PID: ceiling exponent on latency
    print(f"\n  PID comparison:")
    print(f"    Kp_max ~ 380 / latency^1.0  (keff coef ~-0.20, latency-dominated)")
    print(f"    ADRC: omega_c_max latency exponent = {-b[0]:.2f}")
    print(f"          ADRC td exponent             = {-b[1]:.2f}")
    print(f"  => PID: ceiling barely depends on authority (exp -0.20)")
    print(f"     ADRC: ceiling depends comparably on both" if abs(b[1]) > 0.4 else
          f"     ADRC: ceiling also latency-dominated")

    return b, r2


def main():
    print(f"Reference designs for {len(TD_TARGETS)} td levels:")
    designs = pick_reference_designs()
    print()

    tasks = [(d, lat, wc)
             for d in designs
             for lat in LATENCIES
             for wc in OMEGA_C_GRID]
    total_sims = len(tasks) * N_SEEDS
    print(f"Total cells: {len(tasks)}   Total sims: {total_sims}")
    print("Running (parallel)...\n")

    results = Parallel(n_jobs=-1, verbose=3)(
        delayed(_eval_cell)(d, lat, wc)
        for d, lat, wc in tasks
    )

    raw_df = pd.DataFrame(results)
    raw_df.to_csv(RAW_CSV, index=False)
    print(f"\nSaved {len(raw_df)} raw rows to {RAW_CSV.name}")

    summary = extract_ceiling(raw_df)
    summary.to_csv(SUMMARY_CSV, index=False)
    print(f"Saved {len(summary)} summary rows to {SUMMARY_CSV.name}")

    print("\n=== Ceiling table ===")
    pivot = summary.pivot(index='td', columns='latency_steps',
                          values='omega_c_max').round(1)
    print(pivot.to_string())

    fit_power_law(summary)


if __name__ == '__main__':
    main()
