"""
tools/kp_miss_scaling.py — Simulation-side S2R scaling law.

The "sim-to-real" mismatch in this project has two layers:
  Layer 1: simple_model -> full_physics  (gain selection error, already measured)
  Layer 2: full_physics  -> real_flight   (requires hardware, future work)

This script quantifies Layer 1 as a CONTINUOUS function of theta_ddot_max and
latency: for each design, compute Kp_miss = how far Kp_simple falls outside the
real [Kp_floor, Kp_ceiling] gain window.

If Kp_miss scales as theta_ddot_max^alpha * latency^beta, that is the
simulation-side S2R scaling law. Real flights would then test whether the
analogous full_physics -> real_flight gap has the same functional form.

Outputs
-------
experiments/results/kp_miss_scaling_py.csv  — per-design Kp_miss + predictors
(printed summary to stdout)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from scipy import stats

RESULTS = Path('experiments/results')

# ── Physical constants ────────────────────────────────────────────────────────
F15_AVG = 14.4          # N
L_NOZ   = 0.25          # m
CU_RAD  = np.pi / 180 * 15.0 / 12.0   # = 0.02182 rad/CU

# ── Gain window formula coefficients (window_ratio_v2 regression, CV R²=0.594)
# Kp_ceiling = 380 / latency_steps    (DIPDT + 2.1× bang-bang correction)
# Kp_floor   = 0.06 × keff^1.061 × latency^0.964
CEIL_CONST    = 380.0
FLOOR_CONST   = 0.060
FLOOR_KEFF_EX = 1.061
FLOOR_LAT_EX  = 0.964


def keff_full(row):
    """Compute keff_full = T_avg * CU_RAD * L_nozzle / Iyy."""
    T = F15_AVG * row['motor_scale']
    return T * CU_RAD * L_NOZ / row['Iyy']


def load_data():
    s2r = pd.read_csv(RESULTS / 'exp4_s2r_gains_final_py.csv')
    # final_label already in s2r; merge only for uncertain flag and td
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')
    pop_slim = pop[['rocket_id', 'td', 'uncertain']].copy()
    df = s2r.merge(pop_slim, on='rocket_id', how='left')
    return df


def compute_window_metrics(df):
    df = df.copy()
    df['keff'] = df.apply(keff_full, axis=1)
    # td column comes from population merge; recompute if missing
    if 'td' not in df.columns or df['td'].isna().all():
        df['td'] = df['keff'] * (df['max_gimbal_deg'] * 12.0 / 15.0)

    df['Kp_ceiling'] = CEIL_CONST / df['latency_steps']
    df['Kp_floor']   = (FLOOR_CONST
                        * df['keff'] ** FLOOR_KEFF_EX
                        * df['latency_steps'] ** FLOOR_LAT_EX)
    df['window_ratio'] = df['Kp_ceiling'] / df['Kp_floor']

    # Kp_miss: how far does Kp_simple fall outside [Kp_floor, Kp_ceiling]?
    under = np.maximum(0, df['Kp_floor'] - df['kp_simple'])
    over  = np.maximum(0, df['kp_simple'] - df['Kp_ceiling'])
    df['Kp_miss']          = under + over
    df['Kp_miss_fraction'] = df['Kp_miss'] / df['window_ratio'].clip(lower=1.0)
    df['miss_type']        = np.where(under > 0, 'undertuned',
                             np.where(over > 0, 'overtuned', 'inside'))

    # Binary: did the design end up with a usable gain?
    df['gain_ok'] = (df['kp_simple'] >= df['Kp_floor']) & \
                    (df['kp_simple'] <= df['Kp_ceiling'])
    return df


def fit_scaling_law(df):
    """Fit log(Kp_miss+1) ~ a*log(td) + b*log(latency) + const on all designs."""
    # Use all designs where Kp_miss > 0
    mask = df['Kp_miss'] > 0.5
    sub  = df[mask].copy()
    print(f"\nn (Kp_miss > 0.5): {mask.sum()} / {len(df)}")

    log_miss = np.log(sub['Kp_miss'] + 1.0)
    log_td   = np.log(sub['td'].clip(lower=1.0))
    log_lat  = np.log(sub['latency_steps'].astype(float))

    X = np.column_stack([log_td, log_lat, np.ones(len(sub))])
    res = np.linalg.lstsq(X, log_miss, rcond=None)
    a, b, c = res[0]

    pred   = X @ np.array([a, b, c])
    ss_res = np.sum((log_miss - pred) ** 2)
    ss_tot = np.sum((log_miss - log_miss.mean()) ** 2)
    r2     = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

    print(f"\nKp_miss scaling law (on designs with Kp_miss > 0.5):")
    print(f"  log(Kp_miss+1) ~ {a:.3f}*log(td) + {b:.3f}*log(latency) + {c:.3f}")
    print(f"  => Kp_miss proportional to td^{a:.3f} x latency^{b:.3f}  (R2={r2:.3f})")

    # Also: does the COMBINED td×latency product work?
    log_prod = log_td + log_lat
    X2 = np.column_stack([log_prod, np.ones(len(sub))])
    r2_2 = np.linalg.lstsq(X2, log_miss, rcond=None)
    pred2 = X2 @ r2_2[0]
    ss2   = np.sum((log_miss - pred2) ** 2)
    r2_prod = 1 - ss2 / ss_tot if ss_tot > 0 else 0.0
    print(f"  Single-product log(td*lat): R2={r2_prod:.3f}")

    # Single predictor: td alone
    X3 = np.column_stack([log_td, np.ones(len(sub))])
    r2_3 = np.linalg.lstsq(X3, log_miss, rcond=None)
    pred3 = X3 @ r2_3[0]
    ss3   = np.sum((log_miss - pred3) ** 2)
    r2_td = 1 - ss3 / ss_tot if ss_tot > 0 else 0.0
    print(f"  td alone:                  R2={r2_td:.3f}")

    return a, b, c, r2


def print_summary_table(df):
    print("\n=== Kp_miss by regime and td tier ===")
    df['td_tier'] = pd.cut(df['td'],
                           bins=[0, 40, 80, 120, 200, 1e6],
                           labels=['<40', '40-80', '80-120', '120-200', '>200'])

    tab = df.groupby(['td_tier', 'final_label'], observed=True).agg(
        n=('Kp_miss', 'count'),
        miss_gt0=('Kp_miss', lambda x: (x > 0.5).sum()),
        miss_pct=('Kp_miss', lambda x: 100.0 * (x > 0.5).mean()),
        mean_miss=('Kp_miss', 'mean'),
        median_miss=('Kp_miss', 'median'),
        mean_window=('window_ratio', 'mean'),
    ).round(1)
    print(tab.to_string())

    print("\n=== Miss type breakdown ===")
    print(df.groupby('miss_type')['Kp_miss'].describe().round(2))

    print("\n=== False rejection prediction from Kp_miss (Kp_miss > 0) ===")
    for label in ['EASY', 'FRAGILE', 'INFEASIBLE']:
        sub = df[df['final_label'] == label]
        if len(sub) == 0:
            continue
        miss_pct  = 100 * (sub['Kp_miss'] > 0.5).mean()
        fr_pct    = 100 * sub['false_rejection_v2'].mean()
        print(f"  {label:12s} n={len(sub):4d}: Kp_miss>0.5={miss_pct:.1f}%  "
              f"actual_FR={fr_pct:.1f}%")

    print("\n=== S2R scaling law: predicted sim-failure threshold ===")
    # At what td does Kp_miss/window_ratio exceed 1 (i.e., the gap > the window)?
    # Find: 0.06*keff^1.06*lat^0.96 >= 380/lat  =>  keff >= (380/0.06/lat^1.96)
    # For lat=3 (typical): keff >= 380/(0.06 * 3^1.96) = 380/(0.06*8.82) = 718
    # That's very high — most designs have keff~5-20
    # Better: find where Kp_miss/Kp_ceiling >= 0.5 (miss is half the headroom)
    df['miss_frac_of_ceiling'] = df['Kp_miss'] / df['Kp_ceiling'].clip(lower=1.0)
    for lat in [1, 3, 6]:
        sub = df[df['latency_steps'] == lat].sort_values('td')
        cross = sub[sub['miss_frac_of_ceiling'] > 0.5]['td']
        if len(cross):
            print(f"  lat={lat}: Kp_miss > 50%% of ceiling above td ~ {cross.iloc[0]:.0f} rad/s^2")
        else:
            print(f"  lat={lat}: Kp_miss never exceeds 50%% of ceiling in this population")


def main():
    print("Loading data...")
    df = load_data()
    df = compute_window_metrics(df)

    # Save full per-design table
    out_cols = ['rocket_id', 'final_label', 'td', 'keff', 'latency_steps',
                'kp_simple', 'kp_full', 'Kp_floor', 'Kp_ceiling', 'window_ratio',
                'Kp_miss', 'Kp_miss_fraction', 'miss_type', 'gain_ok',
                'sr_simple_in_full', 'sr_full_in_full', 'false_rejection_v2']
    out_cols = [c for c in out_cols if c in df.columns]
    df[out_cols].to_csv(RESULTS / 'kp_miss_scaling_py.csv', index=False)
    print(f"Saved {len(df)} rows to experiments/results/kp_miss_scaling_py.csv")

    print_summary_table(df)
    fit_scaling_law(df)

    # Key summary for paper
    print("\n=== KEY NUMBERS FOR PAPER ===")
    easy_miss = df[df['final_label'] == 'EASY']['Kp_miss']
    frag_miss = df[df['final_label'] == 'FRAGILE']['Kp_miss']
    print(f"EASY     median Kp_miss: {easy_miss.median():.1f}  "
          f"pct>0.5: {100*(easy_miss>0.5).mean():.1f}%")
    print(f"FRAGILE  median Kp_miss: {frag_miss.median():.1f}  "
          f"pct>0.5: {100*(frag_miss>0.5).mean():.1f}%  n={len(frag_miss)}")

    corr, pval = stats.spearmanr(df['td'], df['Kp_miss'])
    print(f"Spearman rho(td, Kp_miss):     {corr:.3f}  p={pval:.2e}")
    corr2, pval2 = stats.spearmanr(np.log(df['td'].clip(1)), df['Kp_miss'])
    print(f"Spearman rho(log_td, Kp_miss): {corr2:.3f}  p={pval2:.2e}")



if __name__ == '__main__':
    main()
