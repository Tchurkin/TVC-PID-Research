"""
tools/stress_test_analysis.py

Fast analysis from existing n=2400 CSV. No new simulations.

Tests:
  1. Iyy-mass physical consistency filter (Iyy >= mass * IYY_RATIO_MIN)
  2. Tighter success criterion (band_20: max_theta < 20 deg instead of 70 deg)
     Uses nominal_band_20_rate already computed in each run.
  3. Combined: both filters together

Reference: ChatGPT feedback 5 / stress-test design-space review.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.metrics import roc_auc_score
from scipy.stats import bootstrap

np.random.seed(42)

CSV = Path('experiments/results/exp1_regime_index_py.csv')
F15_T_AVG = 14.4
L_NOZZLE  = 0.25
CU_TO_RAD = (np.pi / 180) * (15 / 12)

# Physical constraint: Iyy >= mass * ratio
# Uniform rod (length=0.5m): Iyy_rod = mass * 0.5^2 / 12 = mass * 0.02083
# We use 0.010 (half the uniform rod minimum — allows moderately concentrated mass)
IYY_RATIO_MIN = 0.010


def theta_ddot(df):
    keff = F15_T_AVG * df['motor_scale'] * CU_TO_RAD * L_NOZZLE / df['Iyy']
    return keff * df['max_gimbal_deg'] * 12.0 / 15.0


def bootstrap_auc(y_true, y_score, n=5000, seed=0):
    rng = np.random.default_rng(seed)
    base = roc_auc_score(y_true, y_score)
    boots = []
    idx = np.arange(len(y_true))
    for _ in range(n):
        b = rng.choice(idx, size=len(idx), replace=True)
        if y_true[b].sum() in (0, len(b)):
            continue
        boots.append(roc_auc_score(y_true[b], y_score[b]))
    lo, hi = np.percentile(boots, [2.5, 97.5])
    return base, lo, hi


def report_regime_counts(label, df, regime_col='regime_label'):
    counts = df[regime_col].value_counts()
    n = len(df)
    print(f"\n{label} (n={n}):")
    for r in ['EASY','MARGINAL','FRAGILE','INFEASIBLE']:
        c = counts.get(r, 0)
        print(f"  {r:12s}: {c:5d}  ({100*c/n:.1f}%)")


def compute_auc(df, regime_col='regime_label'):
    y = (df[regime_col] == 'FRAGILE').astype(int).values
    if y.sum() < 3:
        return None, None, None
    td = theta_ddot(df).values
    return bootstrap_auc(y, td)


def main():
    df = pd.read_csv(CSV)
    print(f"Loaded {len(df)} designs")
    print(f"  Iyy/mass ratio: min={( df['Iyy']/df['mass']).min():.4f}  "
          f"max={( df['Iyy']/df['mass']).max():.4f}  "
          f"median={( df['Iyy']/df['mass']).median():.4f}")

    # ── 1. IYY-MASS FILTER ────────────────────────────────────────────────────
    print("\n" + "="*70)
    print("1. IYY-MASS PHYSICAL CONSISTENCY FILTER")
    print("="*70)

    df['iyy_mass_ratio'] = df['Iyy'] / df['mass']
    df_iyy = df[df['iyy_mass_ratio'] >= IYY_RATIO_MIN].copy()
    n_removed = len(df) - len(df_iyy)

    print(f"\nFilter: Iyy >= mass * {IYY_RATIO_MIN} (half uniform-rod minimum)")
    print(f"Uniform rod (0.5m) baseline: Iyy/mass = 0.0208")
    print(f"Designs removed: {n_removed} ({100*n_removed/len(df):.1f}%)")

    # Show what was removed
    removed = df[df['iyy_mass_ratio'] < IYY_RATIO_MIN]
    print(f"\nRemoved designs by regime:")
    for r in ['EASY','MARGINAL','FRAGILE','INFEASIBLE']:
        c = (removed['regime_label'] == r).sum()
        tot = (df['regime_label'] == r).sum()
        print(f"  {r:12s}: removed {c}/{tot} ({100*c/max(1,tot):.0f}%)")

    report_regime_counts("ORIGINAL (n=2400)", df)
    report_regime_counts(f"IYY FILTER (Iyy >= mass*{IYY_RATIO_MIN})", df_iyy)

    auc_orig, lo_orig, hi_orig = compute_auc(df)
    auc_filt, lo_filt, hi_filt = compute_auc(df_iyy)
    print(f"\nAUC (theta_ddot_max predicting FRAGILE):")
    print(f"  Original: {auc_orig:.3f} [{lo_orig:.3f}, {hi_orig:.3f}]")
    if auc_filt:
        print(f"  After Iyy filter: {auc_filt:.3f} [{lo_filt:.3f}, {hi_filt:.3f}]")
    else:
        print(f"  After Iyy filter: too few FRAGILE to compute")

    # Check if Iyy filter removes mostly low-authority or high-authority designs
    td_all = theta_ddot(df)
    td_removed = theta_ddot(removed) if len(removed) > 0 else pd.Series([])
    if len(td_removed) > 0:
        print(f"\ntheta_ddot of removed designs: mean={td_removed.mean():.1f}, "
              f"median={td_removed.median():.1f} rad/s^2")
        print(f"theta_ddot of kept designs: mean={theta_ddot(df_iyy).mean():.1f}, "
              f"median={theta_ddot(df_iyy).median():.1f} rad/s^2")
        fragile_removed = removed[removed['regime_label']=='FRAGILE']
        print(f"FRAGILE designs removed (td values): "
              f"{sorted(theta_ddot(fragile_removed).values)}")

    # ── 2. TIGHTER SUCCESS CRITERION (20 deg) ────────────────────────────────
    print("\n" + "="*70)
    print("2. TIGHTER SUCCESS CRITERION: max_theta < 20 deg (nominal SR)")
    print("="*70)
    print("\nUsing nominal_band_20_rate from existing simulations.")
    print("Standard criterion: max_theta < 70 deg, end_error < 15 deg, rms < 15 deg")
    print("Strict criterion: max_theta < 20 deg (band_20_pass threshold)")
    print("Note: autotune selected Kp for standard criterion, not 20 deg criterion.")
    print("This shows how many EASY designs are 'just passing' vs comfortably passing.")

    SR_THRESH = 0.80
    df['strict_nom_pass'] = df['nominal_band_20_rate'] >= SR_THRESH
    df['std_nom_pass']    = df['nominal_success_rate'] >= SR_THRESH

    n_easy = (df['regime_label'] == 'EASY').sum()
    n_easy_strict = ((df['regime_label'] == 'EASY') & df['strict_nom_pass']).sum()
    n_easy_fail_strict = n_easy - n_easy_strict

    print(f"\nOf {n_easy} EASY designs (pass standard criterion):")
    print(f"  Pass 20-deg criterion: {n_easy_strict} ({100*n_easy_strict/n_easy:.1f}%)")
    print(f"  Fail 20-deg criterion: {n_easy_fail_strict} ({100*n_easy_fail_strict/n_easy:.1f}%)")

    print(f"\nOf {(df['regime_label']=='FRAGILE').sum()} FRAGILE designs:")
    n_frag = (df['regime_label'] == 'FRAGILE').sum()
    n_frag_strict = ((df['regime_label'] == 'FRAGILE') & df['strict_nom_pass']).sum()
    print(f"  Pass 20-deg criterion: {n_frag_strict} ({100*n_frag_strict/max(1,n_frag):.1f}%)")
    print(f"  Fail 20-deg criterion: {n_frag - n_frag_strict} ({100*(n_frag-n_frag_strict)/max(1,n_frag):.1f}%)")

    # "Strict regime" classification: fail standard OR fail strict -> at_risk
    # EASY under strict = currently EASY AND passes 20-deg criterion
    # Downgraded = currently EASY but fails 20-deg criterion
    df['strict_regime'] = df['regime_label'].copy()
    mask_downgraded = (df['regime_label'] == 'EASY') & (~df['strict_nom_pass'])
    df.loc[mask_downgraded, 'strict_regime'] = 'FRAGILE_STRICT'

    print(f"\n--- Strict regime summary ---")
    n = len(df)
    for r in ['EASY','FRAGILE_STRICT','MARGINAL','FRAGILE','INFEASIBLE']:
        c = (df['strict_regime'] == r).sum()
        print(f"  {r:20s}: {c:5d} ({100*c/n:.1f}%)")

    print(f"\n  Combined 'at risk under strict criterion':")
    n_at_risk = ((df['strict_regime']=='FRAGILE_STRICT') |
                 (df['strict_regime']=='FRAGILE') |
                 (df['strict_regime']=='MARGINAL') |
                 (df['strict_regime']=='INFEASIBLE')).sum()
    print(f"  {n_at_risk} designs ({100*n_at_risk/n:.1f}%)")

    # Distribution of band_20 rates among EASY designs
    easy_b20 = df.loc[df['regime_label']=='EASY', 'nominal_band_20_rate']
    print(f"\nbandwidth_20 distribution for EASY designs:")
    for pct, val in [(10,None),(25,None),(50,None),(75,None),(90,None),(100,None)]:
        v = np.percentile(easy_b20, pct)
        print(f"  p{pct:3d}: {v:.3f}")

    # theta_ddot of downgraded designs
    downgraded_df = df[mask_downgraded]
    td_down = theta_ddot(downgraded_df)
    all_easy_td = theta_ddot(df[df['regime_label']=='EASY'])
    print(f"\ntheta_ddot of downgraded EASY designs: "
          f"mean={td_down.mean():.1f}, median={td_down.median():.1f} rad/s^2")
    print(f"theta_ddot of all EASY designs: "
          f"mean={all_easy_td.mean():.1f}, median={all_easy_td.median():.1f} rad/s^2")
    print(f"\nDo downgraded designs have higher theta_ddot? "
          f"{'YES' if td_down.mean() > all_easy_td.mean() else 'NO'} "
          f"(mean ratio: {td_down.mean()/all_easy_td.mean():.2f}x)")

    # AUC for predicting 'at risk under strict criterion'
    y_strict = ((df['strict_regime']=='FRAGILE_STRICT') | (df['regime_label']=='FRAGILE')).astype(int).values
    td_vals = theta_ddot(df).values
    auc_strict, lo_s, hi_s = bootstrap_auc(y_strict, td_vals)
    y_combined_lat = np.log(td_vals * df['latency_steps'].values)
    auc_strict_lat, lo_sl, hi_sl = bootstrap_auc(y_strict, y_combined_lat)
    print(f"\nAUC predicting 'at risk under strict criterion' (FRAGILE + downgraded EASY):")
    print(f"  theta_ddot alone:     {auc_strict:.3f} [{lo_s:.3f}, {hi_s:.3f}]  "
          f"(n_positive={y_strict.sum()})")
    print(f"  + latency combined:   {auc_strict_lat:.3f} [{lo_sl:.3f}, {hi_sl:.3f}]")

    # ── 3. COMBINED: IYY FILTER + STRICT CRITERION ──────────────────────────
    print("\n" + "="*70)
    print("3. COMBINED: IYY FILTER + STRICT 20-DEG CRITERION")
    print("="*70)

    df_both = df_iyy.copy()
    mask_down2 = (df_both['regime_label'] == 'EASY') & (df_both['nominal_band_20_rate'] < SR_THRESH)
    df_both.loc[mask_down2, 'strict_regime'] = 'FRAGILE_STRICT'

    n2 = len(df_both)
    print(f"\nDesigns remaining after Iyy filter: {n2}")
    for r in ['EASY','FRAGILE_STRICT','MARGINAL','FRAGILE','INFEASIBLE']:
        c = (df_both['strict_regime'] == r).sum()
        print(f"  {r:20s}: {c:5d} ({100*c/n2:.1f}%)")
    n_at_risk2 = ((df_both['strict_regime']!='EASY') & (df_both['strict_regime']!='MARGINAL')).sum()
    # Actually: at risk = any non-EASY under strict
    n_at_risk2 = (df_both['strict_regime'] != 'EASY').sum()
    print(f"\n  Total 'at risk' (any non-EASY under strict+filter): "
          f"{n_at_risk2} ({100*n_at_risk2/n2:.1f}%)")

    # AUC on filtered + strict set
    y3 = (df_both['strict_regime'] != 'EASY').astype(int).values
    td3 = theta_ddot(df_both).values
    if y3.sum() >= 3:
        auc3, lo3, hi3 = bootstrap_auc(y3, td3)
        print(f"  AUC (theta_ddot, filtered+strict 'at risk'): {auc3:.3f} [{lo3:.3f}, {hi3:.3f}]")

    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"\nOriginal (n=2400, standard 30-deg criterion):")
    print(f"  FRAGILE: 45 (1.9%)")
    print(f"  AUC: 0.944")

    frag_strict_n = (df['strict_regime'] == 'FRAGILE_STRICT').sum()
    total_at_risk = frag_strict_n + 45
    print(f"\nWith strict 20-deg criterion (existing simulations, same n=2400):")
    print(f"  Standard FRAGILE: 45")
    print(f"  Newly downgraded EASY (fail 20-deg nom): {frag_strict_n}")
    print(f"  Total at risk: {total_at_risk} ({100*total_at_risk/len(df):.1f}%)")
    print(f"  AUC (predicting combined at-risk): {auc_strict:.3f} [{lo_s:.3f}, {hi_s:.3f}]")

    n_iyy_removed_frag = ((removed['regime_label']=='FRAGILE') if len(removed)>0 else pd.Series([], dtype=str)).sum() if len(removed) > 0 else 0
    print(f"\nWith Iyy filter (Iyy >= mass*{IYY_RATIO_MIN}):")
    print(f"  Designs removed: {n_removed} ({100*n_removed/len(df):.1f}%)")
    print(f"  FRAGILE removed: {n_iyy_removed_frag}")
    print(f"  AUC after filter: {auc_filt:.3f} [{lo_filt:.3f}, {hi_filt:.3f}]" if auc_filt else "  AUC: too few FRAGILE")

    print("\nNOTE: Latency extension [1,12] requires a new run -- see exp1_stress_test.py")
    print("\nDone.")


if __name__ == '__main__':
    main()
