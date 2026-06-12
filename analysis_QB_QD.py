"""
analysis_QB_QD.py
Q-B: Do stable vs unstable designs fail in qualitatively different ways?
     (divergent failure vs oscillatory failure)
Q-D: Does the gain window structure change sharply at static_margin=0?
     (qualitative shift at the aerodynamic neutral-stability boundary)
"""
import sys, os; sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sim"))
import pandas as pd
import numpy as np
from scipy import stats

df = pd.read_csv("experiments/results/exp1_regime_index_py.csv")
s2r = pd.read_csv("experiments/results/exp4_s2r_gains_py.csv")

df["authority_inertia_ratio"] = (df.max_gimbal_deg * df.motor_scale) / df.Iyy

RULER = "=" * 70

# ── Q-B: FAILURE MODE TAXONOMY ────────────────────────────────────────────────
print(RULER)
print("Q-B: FAILURE MODE — DIVERGENCE vs OSCILLATION")
print(RULER)

# Design subgroups
stable   = df[df.static_margin < 0]
unstable = df[df.static_margin > 0]
stable_fail   = stable[stable.nominal_success_rate < 1.0]
unstable_fail = unstable[unstable.nominal_success_rate < 1.0]

print(f"\nDesigns with nom_sr < 1.0 (some failure):")
print(f"  Stable   (sm<0): {len(stable_fail)}/{len(stable)} ({len(stable_fail)/len(stable)*100:.1f}%)")
print(f"  Unstable (sm>0): {len(unstable_fail)}/{len(unstable)} ({len(unstable_fail)/len(unstable)*100:.1f}%)")

# Diverge rate comparison
print(f"\nDivergence rate (nominal_diverge_rate):")
print(f"  All stable:   mean={stable.nominal_diverge_rate.mean():.4f}, "
      f"nonzero={( stable.nominal_diverge_rate > 0).sum()} designs")
print(f"  All unstable: mean={unstable.nominal_diverge_rate.mean():.4f}, "
      f"nonzero={(unstable.nominal_diverge_rate > 0).sum()} designs")
t_div, p_div = stats.ttest_ind(stable.nominal_diverge_rate, unstable.nominal_diverge_rate)
print(f"  t-test: t={t_div:.3f}, p={p_div:.4f}")

# Oscillation score comparison
print(f"\nOscillation score:")
print(f"  All stable:   mean={stable.oscillation_score.mean():.3f}, "
      f"std={stable.oscillation_score.std():.3f}")
print(f"  All unstable: mean={unstable.oscillation_score.mean():.3f}, "
      f"std={unstable.oscillation_score.std():.3f}")
t_osc, p_osc = stats.ttest_ind(stable.oscillation_score, unstable.oscillation_score)
print(f"  t-test: t={t_osc:.3f}, p={p_osc:.4f}")

# Is there a crossover? Stable fails by oscillation, unstable by divergence?
print(f"\nAmong FAILING designs (nom_sr < 1.0):")
if len(stable_fail) > 0:
    print(f"  Stable   failures: mean diverge={stable_fail.nominal_diverge_rate.mean():.3f}, "
          f"mean osc_score={stable_fail.oscillation_score.mean():.2f}")
if len(unstable_fail) > 0:
    print(f"  Unstable failures: mean diverge={unstable_fail.nominal_diverge_rate.mean():.3f}, "
          f"mean osc_score={unstable_fail.oscillation_score.mean():.2f}")

# FRAGILE designs specifically
frag = df[df.regime_label == "FRAGILE"]
frag_s = frag[frag.static_margin < 0]
frag_u = frag[frag.static_margin > 0]
print(f"\nFRAGILE subgroup:")
print(f"  Stable   (n={len(frag_s)}): mean diverge={frag_s.nominal_diverge_rate.mean():.3f}, "
      f"mean osc={frag_s.oscillation_score.mean():.2f}")
print(f"  Unstable (n={len(frag_u)}): mean diverge={frag_u.nominal_diverge_rate.mean():.3f}, "
      f"mean osc={frag_u.oscillation_score.mean():.2f}")

# Over_diverge vs under_diverge comparison
print(f"\nDivergence at over-gain (doubled Kp) vs under-gain (halved Kp):")
print(f"  over_diverge_rate mean (stable):   {stable.over_diverge_rate.mean():.4f}")
print(f"  over_diverge_rate mean (unstable): {unstable.over_diverge_rate.mean():.4f}")
print(f"  under_diverge_rate mean (stable):  {stable.under_diverge_rate.mean():.4f}")
print(f"  under_diverge_rate mean (unstable):{unstable.under_diverge_rate.mean():.4f}")

# Saturation analysis: does high slew_sat_frac correlate with oscillation?
print(f"\nSlew saturation vs oscillation (all designs):")
r_sat_osc, p_sat_osc = stats.pearsonr(df.slew_sat_frac, df.oscillation_score)
r_sat_div, p_sat_div = stats.pearsonr(df.slew_sat_frac, df.nominal_diverge_rate)
print(f"  r(slew_sat, oscillation) = {r_sat_osc:+.3f}, p={p_sat_osc:.4f}")
print(f"  r(slew_sat, diverge)     = {r_sat_div:+.3f}, p={p_sat_div:.4f}")

r_ratio_osc, _ = stats.pearsonr(df.authority_inertia_ratio, df.oscillation_score)
r_ratio_div, _ = stats.pearsonr(df.authority_inertia_ratio, df.nominal_diverge_rate)
print(f"  r(authority/Iyy, oscillation) = {r_ratio_osc:+.3f}")
print(f"  r(authority/Iyy, diverge)     = {r_ratio_div:+.3f}")

# ── Q-D: GAIN WINDOW STRUCTURE AT STATIC_MARGIN=0 ────────────────────────────
print(f"\n{RULER}")
print("Q-D: GAIN WINDOW STRUCTURE AT AERODYNAMIC STABILITY BOUNDARY")
print(RULER)

# Gain tolerance measures:
# 1. n_success_gain_sets: how many of the 25 (Kp,Kd) combos work
# 2. over_success_rate: SR when gains doubled
# 3. under_success_rate: SR when gains halved
# 4. robustness = min(under_sr, over_sr, nom_sr) / best_sr — already computed

print(f"\nn_success_gain_sets (how many of 25 gain combos work):")
for label, sub in [("Stable (sm<0)", stable), ("Unstable (sm>0)", unstable)]:
    print(f"  {label}: mean={sub.n_success_gain_sets.mean():.1f}, "
          f"std={sub.n_success_gain_sets.std():.1f}, "
          f"median={sub.n_success_gain_sets.median():.0f}")
t_ngain, p_ngain = stats.ttest_ind(stable.n_success_gain_sets, unstable.n_success_gain_sets)
print(f"  t-test: t={t_ngain:.3f}, p={p_ngain:.4f}")

print(f"\nover_success_rate (gain doubled from optimal):")
for label, sub in [("Stable (sm<0)", stable), ("Unstable (sm>0)", unstable)]:
    print(f"  {label}: mean={sub.over_success_rate.mean():.3f}, "
          f"std={sub.over_success_rate.std():.3f}")
t_over, p_over = stats.ttest_ind(stable.over_success_rate, unstable.over_success_rate)
print(f"  t-test: t={t_over:.3f}, p={p_over:.4f}")

# Bin by static_margin to see if there's a sharp transition at 0
print(f"\nGain window metrics binned by static_margin:")
bins = [-0.30, -0.20, -0.10, -0.02, 0.0, 0.02, 0.10, 0.20, 0.30]
labels_sm = ["-0.30:-0.20", "-0.20:-0.10", "-0.10:-0.02",
             "-0.02:0.00", "0.00:0.02", "0.02:0.10", "0.10:0.20", "0.20:0.30"]
df["sm_bin"] = pd.cut(df.static_margin, bins=bins, labels=labels_sm)

print(f"\n  {'SM range':<15} {'n':>5}  {'n_gain_sets':>12}  "
      f"{'over_sr':>8}  {'under_sr':>9}  {'frag_rate':>10}")
for lbl, g in df.groupby("sm_bin", observed=True):
    frag_rate = (g.regime_label == "FRAGILE").mean() * 100
    print(f"  {str(lbl):<15} {len(g):>5}  "
          f"{g.n_success_gain_sets.mean():>12.1f}  "
          f"{g.over_success_rate.mean():>8.3f}  "
          f"{g.under_success_rate.mean():>9.3f}  "
          f"{frag_rate:>9.1f}%")

# Is there a sharp drop in n_success_gain_sets at sm=0?
near_stable   = df[df.static_margin.between(-0.05, 0.0)]
near_unstable = df[df.static_margin.between(0.0, 0.05)]
t_boundary, p_boundary = stats.ttest_ind(
    near_stable.n_success_gain_sets, near_unstable.n_success_gain_sets)
print(f"\nSharp transition test at sm=0 (|sm| < 0.05):")
print(f"  Near-stable  (n={len(near_stable)}): mean n_gain_sets="
      f"{near_stable.n_success_gain_sets.mean():.1f}")
print(f"  Near-unstable (n={len(near_unstable)}): mean n_gain_sets="
      f"{near_unstable.n_success_gain_sets.mean():.1f}")
print(f"  t-test: t={t_boundary:.3f}, p={p_boundary:.4f}")

# Continuous relationship: does gain window width vary with static_margin?
r_sm_ngain, p_sm_ngain = stats.pearsonr(df.static_margin, df.n_success_gain_sets)
r_sm_over, p_sm_over   = stats.pearsonr(df.static_margin, df.over_success_rate)
r_sm_under, p_sm_under = stats.pearsonr(df.static_margin, df.under_success_rate)
print(f"\nContinuous correlations with static_margin:")
print(f"  r(sm, n_success_gain_sets) = {r_sm_ngain:+.3f}, p={p_sm_ngain:.4f}")
print(f"  r(sm, over_success_rate)   = {r_sm_over:+.3f}, p={p_sm_over:.4f}")
print(f"  r(sm, under_success_rate)  = {r_sm_under:+.3f}, p={p_sm_under:.4f}")

# Robustness score vs static_margin
r_sm_rob, p_sm_rob = stats.pearsonr(df.static_margin, df.robustness)
print(f"  r(sm, robustness)          = {r_sm_rob:+.3f}, p={p_sm_rob:.4f}")

# Combined: does over_diverge_rate jump at sm=0?
print(f"\nover_diverge_rate (diverges at doubled gain) vs static_margin:")
r_sm_odiv, p_sm_odiv = stats.pearsonr(df.static_margin, df.over_diverge_rate)
print(f"  r(sm, over_diverge_rate) = {r_sm_odiv:+.3f}, p={p_sm_odiv:.4f}")
print(f"  Stable mean:   {stable.over_diverge_rate.mean():.4f}")
print(f"  Unstable mean: {unstable.over_diverge_rate.mean():.4f}")

# Best performance metrics at/near the stability boundary
print(f"\nPeak error and RMS at stability boundary:")
for label, sub in [("Stable (sm<-0.10)", df[df.static_margin < -0.10]),
                    ("Near-neutral (-0.10:0.10)", df[df.static_margin.between(-0.10, 0.10)]),
                    ("Unstable (sm>0.10)", df[df.static_margin > 0.10])]:
    print(f"  {label}: mean RMS={sub.rms_error_deg.mean():.3f}, "
          f"peak={sub.peak_error_deg.mean():.3f}, "
          f"n={len(sub)}")

# Q-D SUMMARY JUDGMENT
print(f"\n{'=' * 50}")
print("Q-D SUMMARY:")
if abs(p_ngain) < 0.05:
    print(f"  SIGNAL: Stable vs unstable n_gain_sets differs (p={p_ngain:.3f})")
else:
    print(f"  NULL: No significant stable/unstable difference in gain window width (p={p_ngain:.3f})")

if abs(p_sm_ngain) < 0.01:
    print(f"  CONTINUOUS SIGNAL: r(sm, n_gain_sets)={r_sm_ngain:+.3f} (p={p_sm_ngain:.4f})")
else:
    print(f"  NULL CONTINUOUS: r(sm, n_gain_sets)={r_sm_ngain:+.3f} (not significant)")

if abs(p_boundary) < 0.05:
    print(f"  SHARP TRANSITION near sm=0: p={p_boundary:.3f} -- INTERESTING!")
else:
    print(f"  No sharp transition near sm=0 (p={p_boundary:.3f})")
