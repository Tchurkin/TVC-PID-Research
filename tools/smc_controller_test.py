"""
tools/smc_controller_test.py — Sliding Mode Controller window test.

Research question: Does SMC escape the Pi = theta_ddot * latency^2 window-compression
constraint, or does it face the same physics as PID and LQR?

Key insight:
  SMC with a boundary layer is mathematically equivalent to PID constrained to a
  fixed Kd/Kp ratio = 1/lambda_s, saturating at u_max outside the boundary.
  No simulator modification needed — implemented as PID(Kp, Kd=Kp/lambda_s).

  u_smc = -u_max * clip(s/phi, -1, 1)   where s = theta_dot + lambda_s * theta
        = -u_max * clip((theta_dot + lambda_s * theta)/phi, -1, 1)

  In the linear region (|s| < phi):
        u_smc = -(u_max*lambda_s/phi)*theta - (u_max/phi)*theta_dot
              = -(Kp_eff)*theta - (Kd_eff)*theta_dot
  where Kp_eff = u_max*lambda_s/phi, Kd_eff = u_max/phi, Kd_eff/Kp_eff = 1/lambda_s.

  Outside boundary layer: both PID and SMC saturate to u_max.

Method:
  - Same 50 designs as LQR test (from lqr_controller_test_py.csv — reuse selection)
  - For each design: sweep lambda_s x Kp_eff on a 6x20 grid (Kd = Kp/lambda_s)
  - 10 seeds (seeds 6001-6010, disjoint from all prior experiments)
  - Window = number of (lambda_s, Kp) combos achieving SR >= 0.80
  - Also report: best per-design window (over all lambda_s) for max n_pass
  - Compare Spearman rho(Pi, n_pass_smc) with LQR result (-0.747)

Prediction: SMC faces same Pi constraint. Any controller without active
disturbance estimation (ADRC's ESO) cannot escape the ceiling/floor physics.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

# ── Constants ─────────────────────────────────────────────────────────────────
F15_AVG = 14.4
L_NOZ   = 0.25
CU_RAD  = np.pi / 180 * 15.0 / 12.0
U_MAX   = 12.0          # actuator saturation (code units)

# ── Experiment settings ───────────────────────────────────────────────────────
N_SEEDS        = 10
SEED_START     = 6001   # disjoint from all prior seed ranges
SR_PASS        = 0.80

# Sliding surface slopes to test.  lambda_s determines the Kd/Kp ratio (= 1/lambda_s).
# lambda_s=1 => Kd=Kp (equal weighting); lambda_s=5 => Kp=5*Kd (more proportional).
LAMBDA_S_GRID = np.array([0.2, 0.5, 1.0, 2.0, 5.0, 10.0])

# Kp_eff sweep (= u_max * lambda_s / phi, equivalently the proportional gain).
# Covers [1, 400] to span the full [floor, ceiling] range seen in the population.
KP_GRID = np.logspace(0, np.log10(400), 20)  # 20 values, 1..400


def keff_full(row):
    return F15_AVG * row['motor_scale'] * CU_RAD * L_NOZ / row['Iyy']


def evaluate_point(row_dict, Kp, Kd, seeds):
    """Run full-physics sim with given (Kp, Kd), return SR."""
    plant       = build_plant(row_dict)
    actuator    = build_actuator(row_dict)
    sensor      = build_sensor(row_dict)
    disturbance = build_disturbance(row_dict)
    scenario    = build_scenario(theta0_bias_std=0.0)

    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    actuator, sensor, disturbance, scenario = apply_fidelity_config(
        actuator, sensor, disturbance, scenario, fid)

    pid = PIDParams(Kp=Kp, Kd=Kd, u_max=U_MAX)
    successes = sum(
        simulate(pid, plant, actuator, sensor, disturbance, scenario, seed=s).success
        for s in seeds
    )
    return successes / len(seeds)


def sweep_design(row_dict):
    """
    Full SMC sweep for one design.
    Returns (summary_dict, detail_df).
    """
    seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    keff  = keff_full(row_dict)
    td    = keff * (row_dict['max_gimbal_deg'] * 12.0 / 15.0)
    lat   = row_dict['latency_steps']
    rid   = row_dict.get('rocket_id', 'unknown')
    label = row_dict.get('final_label', 'UNKNOWN')

    records = []
    for lam in LAMBDA_S_GRID:
        for Kp in KP_GRID:
            Kd  = Kp / lam        # SMC constraint: Kd/Kp = 1/lambda_s
            Kd  = max(0.01, Kd)   # numerical floor
            sr  = evaluate_point(row_dict, Kp, Kd, seeds)
            records.append({'lambda_s': lam, 'Kp': Kp, 'Kd': Kd, 'sr': sr})

    df = pd.DataFrame(records)
    df['rocket_id']   = rid
    df['td']          = td
    df['keff']        = keff
    df['latency']     = lat
    df['final_label'] = label

    # Overall window: count all (lambda_s, Kp) combos with SR >= 0.80
    n_pass_total = (df['sr'] >= SR_PASS).sum()

    # Per-lambda_s window: for each lambda_s, count Kp values passing
    per_lam = (
        df.groupby('lambda_s')['sr']
        .apply(lambda x: (x >= SR_PASS).sum())
        .reset_index(name='n_pass')
    )
    best_lam = per_lam.loc[per_lam['n_pass'].idxmax()]

    peak_sr = df['sr'].max()

    summary = {
        'rocket_id':       rid,
        'final_label':     label,
        'td':              td,
        'keff':            keff,
        'latency':         lat,
        'n_pass_total':    int(n_pass_total),   # out of 6*20=120 combos
        'n_pass_best_lam': int(best_lam['n_pass']),  # best per-lambda slice (out of 20)
        'best_lambda_s':   float(best_lam['lambda_s']),
        'peak_sr':         float(peak_sr),
    }
    return summary, df


def main():
    print("Loading LQR design selection (same 50 designs)...")
    lqr_df = pd.read_csv(RESULTS / 'lqr_controller_test_py.csv')
    pop_df  = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    # Merge to get all design parameters
    merged = lqr_df[['rocket_id']].merge(pop_df, on='rocket_id', how='left')
    if merged['Iyy'].isna().any():
        print("WARNING: some rocket_ids not found in population file")
        merged = merged.dropna(subset=['Iyy'])
    print(f"Designs loaded: {len(merged)}")
    print(f"td range: {merged['td'].min():.1f} to {merged['td'].max():.1f} rad/s^2")
    print(f"Regimes: {merged['final_label'].value_counts().to_dict()}")
    print()
    print(f"Grid: {len(LAMBDA_S_GRID)} lambda_s x {len(KP_GRID)} Kp = "
          f"{len(LAMBDA_S_GRID)*len(KP_GRID)} combos per design")
    print(f"Total sims: {len(merged) * len(LAMBDA_S_GRID) * len(KP_GRID) * N_SEEDS:,}")
    print("Running parallel sweep...")

    results_raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(sweep_design)(row.to_dict())
        for _, row in merged.iterrows()
    )

    summaries = [r[0] for r in results_raw]
    details   = pd.concat([r[1] for r in results_raw], ignore_index=True)

    summary_df = pd.DataFrame(summaries)
    summary_df.to_csv(RESULTS / 'smc_controller_test_py.csv', index=False)
    details.to_csv(RESULTS / 'smc_sweep_detail_py.csv', index=False)
    print(f"\nSaved {len(summary_df)} summary rows, {len(details)} detail rows")

    # ── Analysis ──────────────────────────────────────────────────────────────
    print("\n=== SMC window by td tier ===")
    summary_df['Pi'] = summary_df['td'] * summary_df['latency'] ** 2
    summary_df['td_tier'] = pd.cut(summary_df['td'],
                                   bins=[0, 40, 80, 120, 200, 1e6],
                                   labels=['<40', '40-80', '80-120', '120-200', '>200'])

    tab = summary_df.groupby('td_tier', observed=True).agg(
        n=('n_pass_total', 'count'),
        med_total=('n_pass_total', 'median'),
        med_best_lam=('n_pass_best_lam', 'median'),
        n_zero_total=('n_pass_total', lambda x: (x == 0).sum()),
    ).round(1)
    print(tab.to_string())

    print("\n=== Spearman correlations ===")
    rho_td, p_td = stats.spearmanr(summary_df['td'], summary_df['n_pass_total'])
    rho_pi, p_pi = stats.spearmanr(summary_df['Pi'], summary_df['n_pass_total'])
    rho_td_b, p_td_b = stats.spearmanr(summary_df['td'], summary_df['n_pass_best_lam'])
    rho_pi_b, p_pi_b = stats.spearmanr(summary_df['Pi'], summary_df['n_pass_best_lam'])
    print(f"rho(td,   n_pass_total)    = {rho_td:.3f}  p={p_td:.2e}")
    print(f"rho(Pi,   n_pass_total)    = {rho_pi:.3f}  p={p_pi:.2e}")
    print(f"rho(td,   n_pass_best_lam) = {rho_td_b:.3f}  p={p_td_b:.2e}")
    print(f"rho(Pi,   n_pass_best_lam) = {rho_pi_b:.3f}  p={p_pi_b:.2e}")

    print("\n=== Zero-window designs (peak_sr < 0.80 everywhere) ===")
    zero = summary_df[summary_df['peak_sr'] < SR_PASS].sort_values('td')
    if len(zero) == 0:
        print("  None — every design found at least one valid (lambda_s, Kp)")
    else:
        print(zero[['rocket_id', 'final_label', 'td', 'latency',
                     'n_pass_total', 'peak_sr']].to_string())

    print("\n=== Window by td x latency (median n_pass_best_lam) ===")
    summary_df['lat_tier'] = pd.cut(summary_df['latency'],
                                    bins=[0, 2, 4, 7],
                                    labels=['1-2', '3-4', '5-6'])
    tab2 = summary_df.groupby(['td_tier', 'lat_tier'], observed=True).agg(
        n=('n_pass_best_lam', 'count'),
        median_pass=('n_pass_best_lam', 'median'),
    ).round(1)
    print(tab2.to_string())

    print("\n=== LQR comparison (from lqr_controller_test_py.csv) ===")
    print(f"LQR rho(Pi, n_pass_qr) = -0.747  p=4.8e-10  (from prior experiment)")
    print(f"SMC rho(Pi, n_pass_total) = {rho_pi:.3f}  p={p_pi:.2e}")
    print(f"SMC rho(Pi, n_pass_best_lam) = {rho_pi_b:.3f}  p={p_pi_b:.2e}")
    print("\nDone.")


if __name__ == '__main__':
    main()
