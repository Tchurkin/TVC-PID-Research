"""
tools/lqr_controller_test.py — Controller-invariance test via discrete LQR.

Research question: Is the narrow-window phenomenon specific to PID parameterization,
or does it appear for any linear-feedback controller when expressed in equivalent
gain units?

Method:
  - Select 50 designs stratified by theta_ddot_max (5 bins x 10 designs)
  - For each design, compute discrete LQR gains K=[k1, k2] for a range of Q/R ratios
  - k1 maps to Kp in the existing PID interface; k2 maps to Kd (see below)
  - Evaluate each (Kp_lqr, Kd_lqr) pair with full physics, 10 seeds
  - Find the "LQR gain window" = range of Q/R achieving SR >= 0.80
  - Check: does theta_ddot_max predict LQR window width?

LQR -> PID mapping:
  Discrete plant: x = [theta, theta_dot]
    A = [[1, dt], [0, 1]]
    B = [[0.5*keff*dt^2], [keff*dt]]   (ZOH)
  LQR output: u = -K @ x = -k1*theta - k2*theta_dot
  PID output: u = Kp*e - Kd*theta_dot   (derivative on measurement)
  Equivalence: Kp_eff = k1  (in CU/rad, matching PID Kp units)
               Kd_eff = k2  (in CU/(rad/s), matching PID Kd units)
  Note: Kd from LQR depends on dt and keff; Kp/Kd ratio is NOT the PID heuristic default.

Physical constants:
  F15_avg = 14.4 N, L_nozzle = 0.25 m, CU_TO_RAD = 0.02182 rad/CU, dt = 0.005 s

Outputs:
  experiments/results/lqr_controller_test_py.csv  — per-design results
  experiments/results/lqr_gain_sweep_py.csv       — full Q/R sweep per design
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.linalg import solve_discrete_are
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from simulator import ScenarioParams, simulate, SimResult
from fidelity_config import FidelityConfig

ROOT       = Path(__file__).resolve().parents[1]
RESULTS    = ROOT / 'experiments' / 'results'

# ── Physical constants ────────────────────────────────────────────────────────
F15_AVG  = 14.4
L_NOZ    = 0.25
CU_RAD   = np.pi / 180 * 15.0 / 12.0
DT       = 1.0 / 200.0   # 200 Hz control loop

# ── Experiment settings ───────────────────────────────────────────────────────
N_DESIGNS      = 50       # 5 td bins x 10 designs
N_SEEDS        = 10
SEED_START     = 5001
SUCCESS_THRESH = 0.80
SR_PASS        = 0.80     # success rate threshold for window

# Q/R ratio grid (log-spaced): this determines how "aggressive" the LQR is
# High Q/R -> more aggressive (prioritize tracking, analog to high Kp)
# Low  Q/R -> more conservative
QR_RATIOS = np.logspace(-3, 5, 40)   # 40 values from 0.001 to 100000


def keff_full(row):
    T = F15_AVG * row['motor_scale']
    return T * CU_RAD * L_NOZ / row['Iyy']


def compute_lqr_gains(keff_val, qr_ratio):
    """
    Compute discrete LQR gains K=[k1,k2] for the rocket attitude plant.

    Plant:  x = [theta, theta_dot],  u = gimbal CU
    A = [[1, dt], [0, 1]]
    B = [[0.5*keff*dt^2], [keff*dt]]
    Q = diag([qr_ratio, 1])  (relative weight on angle vs. rate)
    R = 1.0 (scalar)

    Returns (Kp_lqr, Kd_lqr, ok) where:
      Kp_lqr = k1  (units: CU/rad, same as PID Kp)
      Kd_lqr = k2  (units: CU/(rad/s), same as PID Kd)
    """
    A = np.array([[1.0, DT], [0.0, 1.0]])
    B = np.array([[0.5 * keff_val * DT**2], [keff_val * DT]])
    Q = np.diag([qr_ratio, 1.0])
    R = np.array([[1.0]])

    try:
        P = solve_discrete_are(A, B, Q, R)
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
        k1, k2 = K[0, 0], K[0, 1]
        return k1, k2, True
    except Exception:
        return 0.0, 0.0, False


def evaluate_one(row, Kp, Kd, seeds):
    """Run full-physics simulation with given PD gains, return SR."""
    plant       = build_plant(row)
    actuator    = build_actuator(row)
    sensor      = build_sensor(row)
    disturbance = build_disturbance(row)
    scenario    = build_scenario(theta0_bias_std=0.0)

    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    from fidelity_config import apply_fidelity_config
    actuator, sensor, disturbance, scenario = apply_fidelity_config(
        actuator, sensor, disturbance, scenario, fid)

    controller = PIDParams(Kp=Kp, Kd=Kd)

    successes = 0
    for s in seeds:
        res = simulate(controller, plant, actuator, sensor, disturbance, scenario, seed=s)
        if res.success:
            successes += 1
    return successes / len(seeds)


def sweep_qr_for_design(row_dict):
    """Full Q/R sweep for one design. Returns list of (qr, Kp, Kd, sr) tuples."""
    seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    keff = keff_full(row_dict)
    results = []
    for qr in QR_RATIOS:
        k1, k2, ok = compute_lqr_gains(keff, qr)
        if not ok or k1 <= 0 or k2 < 0:
            results.append({'qr': qr, 'Kp': float('nan'), 'Kd': float('nan'), 'sr': float('nan')})
            continue
        sr = evaluate_one(row_dict, k1, k2, seeds)
        results.append({'qr': qr, 'Kp': k1, 'Kd': k2, 'sr': sr})
    return results


def select_designs(pop_df, n_total=50):
    """Stratified selection: 10 designs per td bin using fixed boundaries."""
    pop_df = pop_df[pop_df['final_label'].isin(['EASY', 'FRAGILE', 'INFEASIBLE'])].copy()
    # Fixed bins to ensure we sample the full td range including high-authority designs
    bins = [(0, 30), (30, 60), (60, 120), (120, 200), (200, 1000)]
    per_bin = n_total // len(bins)

    selected = []
    rng = np.random.default_rng(777)
    for lo, hi in bins:
        subset = pop_df[(pop_df['td'] >= lo) & (pop_df['td'] < hi)]
        n_pick = min(per_bin, len(subset))
        if n_pick == 0:
            print(f"  WARNING: no designs in td [{lo}, {hi})")
            continue
        idx = rng.choice(len(subset), size=n_pick, replace=False)
        selected.append(subset.iloc[idx])
        print(f"  td [{lo:3d}-{hi:4d}): picked {n_pick} designs "
              f"(regime: {subset.iloc[idx]['final_label'].value_counts().to_dict()})")

    return pd.concat(selected).reset_index(drop=True)


def process_design(row):
    """Process one design: sweep Q/R, extract window, return summary."""
    design_id = row.get('design_id', row.get('rocket_id', 'unknown'))
    print(f"  Processing {design_id} (td={row['td']:.1f}, lat={row['latency_steps']})...")

    sweep = sweep_qr_for_design(row)
    sweep_df = pd.DataFrame(sweep)
    sweep_df['rocket_id'] = row.get('rocket_id', 'unknown')
    sweep_df['td']        = row['td']
    sweep_df['keff']      = keff_full(row)
    sweep_df['latency']   = row['latency_steps']
    sweep_df['final_label'] = row.get('final_label', 'UNKNOWN')

    # Find LQR gain window
    valid = sweep_df[sweep_df['sr'] >= SR_PASS]
    if len(valid) == 0:
        qr_floor, qr_ceiling, window_qr, Kp_floor, Kp_ceiling = (
            float('nan'), float('nan'), float('nan'), float('nan'), float('nan'))
        peak_sr = sweep_df['sr'].max()
    else:
        qr_floor   = valid['qr'].min()
        qr_ceiling = valid['qr'].max()
        window_qr  = qr_ceiling / qr_floor if qr_floor > 0 else float('nan')
        Kp_floor   = valid['Kp'].min()
        Kp_ceiling = valid['Kp'].max()
        peak_sr    = sweep_df['sr'].max()

    summary = {
        'rocket_id':   row.get('rocket_id', 'unknown'),
        'final_label': row.get('final_label', 'UNKNOWN'),
        'td':          row['td'],
        'keff':        keff_full(row),
        'latency':     row['latency_steps'],
        'qr_floor':    qr_floor,
        'qr_ceiling':  qr_ceiling,
        'window_qr':   window_qr,
        'Kp_floor_lqr': Kp_floor,
        'Kp_ceiling_lqr': Kp_ceiling,
        'Kp_window_lqr': Kp_ceiling / Kp_floor if (Kp_floor and Kp_floor > 0) else float('nan'),
        'peak_sr':     peak_sr,
        'n_pass_qr':   len(valid),
    }
    return summary, sweep_df


def main():
    print("Loading population...")
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    designs = select_designs(pop, n_total=N_DESIGNS)
    print(f"Selected {len(designs)} designs")
    print("td range:", designs['td'].min(), "to", designs['td'].max())
    print("Regimes:", designs['final_label'].value_counts().to_dict())

    all_summaries = []
    all_sweeps    = []

    for _, row in designs.iterrows():
        summary, sweep_df = process_design(row.to_dict())
        all_summaries.append(summary)
        all_sweeps.append(sweep_df)

    # Save
    summary_df = pd.DataFrame(all_summaries)
    summary_df.to_csv(RESULTS / 'lqr_controller_test_py.csv', index=False)
    print(f"\nSaved {len(summary_df)} rows to lqr_controller_test_py.csv")

    sweep_all = pd.concat(all_sweeps, ignore_index=True)
    sweep_all.to_csv(RESULTS / 'lqr_gain_sweep_py.csv', index=False)
    print(f"Saved {len(sweep_all)} sweep rows to lqr_gain_sweep_py.csv")

    # Analysis
    print("\n=== LQR window width by td tier ===")
    summary_df['td_tier'] = pd.cut(summary_df['td'],
                                    bins=[0, 40, 80, 120, 200, 1e6],
                                    labels=['<40', '40-80', '80-120', '120-200', '>200'])
    tab = summary_df.groupby('td_tier', observed=True).agg(
        n=('window_qr', 'count'),
        mean_qr_window=('window_qr', 'mean'),
        median_qr_window=('window_qr', 'median'),
        mean_kp_window=('Kp_window_lqr', 'mean'),
        pct_infeasible=('peak_sr', lambda x: 100 * (x < SR_PASS).mean()),
    ).round(2)
    print(tab.to_string())

    from scipy import stats
    valid = summary_df.dropna(subset=['window_qr', 'td'])
    if len(valid) > 5:
        log_wr = np.log(valid['window_qr'].clip(lower=0.01))
        log_td = np.log(valid['td'].clip(lower=1.0))
        corr, pval = stats.spearmanr(log_td, log_wr)
        print(f"\nSpearman rho(log_td, log_window_qr): {corr:.3f}  p={pval:.2e}")

        # Linear fit for power law
        X = np.column_stack([log_td, np.ones(len(valid))])
        b = np.linalg.lstsq(X, log_wr.values, rcond=None)[0]
        ss_res = np.sum((log_wr.values - X @ b)**2)
        ss_tot = np.sum((log_wr.values - log_wr.mean())**2)
        r2 = 1 - ss_res / ss_tot
        print(f"Power law: window_qr ~ td^{b[0]:.3f}  (R2={r2:.3f})")

    print("\nDone.")


if __name__ == '__main__':
    main()
