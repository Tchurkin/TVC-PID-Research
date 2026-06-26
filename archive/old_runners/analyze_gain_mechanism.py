"""
analyze_gain_mechanism.py -- quick analysis of the Exp4 gain mechanism results.

Run after: python run_experiments.py exp4_mech_priority

Answers: which fidelity module(s) cause the tuner to select Kp=2 instead of Kp=40-80?
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sim"))
import pandas as pd
import numpy as np

RESULT_DIR = "experiments/results"
CSV = os.path.join(RESULT_DIR, "exp4_gain_mechanism_py.csv")

df = pd.read_csv(CSV)
print(f"Loaded: {len(df)} rows, {df.ablated_modules.nunique()} module conditions")
print(f"Designs: {df.rocket_id.nunique()}, Regimes: {df.regime_label.value_counts().to_dict()}\n")

# ── Overall summary table ──────────────────────────────────────────────────────
print("=" * 90)
print(f"  {'Module':<22}  {'N':>5}  {'Mean Kp':>8}  {'%Kp=2':>6}  {'%Kp>=80':>8}  "
      f"{'SR gap':>7}  {'False rej':>9}  {'kp_match':>8}")
print("=" * 90)

# Sort by % choosing Kp=2 descending (most disruptive first)
summary_rows = []
for module_key in df.ablated_modules.unique():
    sub = df[df.ablated_modules == module_key]
    row = dict(
        module=module_key,
        n=len(sub),
        mean_kp=sub.kp_ablated.mean(),
        pct_kp2=(sub.kp_ablated == 2).mean() * 100,
        pct_kp80=(sub.kp_ablated >= 80).mean() * 100,
        sr_gap=sub.sr_gap.mean(),
        false_rej=sub.false_rejection.mean() * 100,
        kp_match=(sub.kp_ablated == sub.kp_full).mean() * 100,
    )
    summary_rows.append(row)

summary_rows.sort(key=lambda r: r["pct_kp2"], reverse=True)
for r in summary_rows:
    print(f"  {r['module']:<22}  {r['n']:>5}  {r['mean_kp']:>8.1f}  {r['pct_kp2']:>5.1f}%  "
          f"{r['pct_kp80']:>7.1f}%    {r['sr_gap']:>6.3f}  {r['false_rej']:>8.1f}%  "
          f"{r['kp_match']:>7.1f}%")

# ── By regime ─────────────────────────────────────────────────────────────────
for regime in ["EASY", "MARGINAL", "FRAGILE"]:
    sub = df[df.regime_label == regime]
    if len(sub) == 0:
        continue
    print(f"\n--- {regime} (n={sub.rocket_id.nunique()} designs) ---")
    print(f"  {'Module':<22}  {'Mean Kp_abl':>12}  {'Mean Kp_full':>13}  "
          f"{'%Kp=2':>6}  {'SR gap':>7}  {'False rej':>9}")
    by_module = sub.groupby("ablated_modules")
    for key, grp in sorted(by_module, key=lambda x: (x[1].kp_ablated == 2).mean(), reverse=True):
        pct_kp2 = (grp.kp_ablated == 2).mean() * 100
        print(f"  {key:<22}  {grp.kp_ablated.mean():>12.1f}  {grp.kp_full.mean():>13.1f}  "
              f"{pct_kp2:>5.1f}%  {grp.sr_gap.mean():>6.3f}  "
              f"{grp.false_rejection.mean()*100:>8.1f}%")

# ── Kp distribution per module ─────────────────────────────────────────────────
print("\n--- Kp selection distribution per module ---")
kp_vals = sorted(df.kp_ablated.unique())
for module_key in sorted(df.ablated_modules.unique()):
    sub = df[df.ablated_modules == module_key]
    dist = {int(k): f"{(sub.kp_ablated == k).mean()*100:.0f}%" for k in kp_vals}
    print(f"  {module_key:<22}  {dist}")

# ── Key insight: does ANY single module cause Kp=2? ──────────────────────────
print("\n--- Key question: which single module causes Kp -> 2? ---")
single_modules = [m for m in df.ablated_modules.unique() if "+" not in m]
for m in sorted(single_modules, key=lambda m: (df[df.ablated_modules==m].kp_ablated == 2).mean(), reverse=True):
    sub = df[df.ablated_modules == m]
    pct_kp2_easy = (df[(df.ablated_modules==m) & (df.regime_label=="EASY")].kp_ablated == 2).mean() * 100
    pct_kp2_frag = (df[(df.ablated_modules==m) & (df.regime_label=="FRAGILE")].kp_ablated == 2).mean() * 100
    print(f"  Remove {m:<20}: overall {(sub.kp_ablated==2).mean()*100:.1f}% pick Kp=2 "
          f"| EASY {pct_kp2_easy:.1f}% | FRAGILE {pct_kp2_frag:.1f}%")

# ── Pairs vs singles ──────────────────────────────────────────────────────────
print("\n--- Pair interaction effects ---")
pair_keys = [m for m in df.ablated_modules.unique() if "+" in m]
for key in pair_keys:
    sub = df[df.ablated_modules == key]
    parts = key.split("+")
    part_subs = [df[df.ablated_modules == p] for p in parts if p in df.ablated_modules.values]
    pct_kp2_pair = (sub.kp_ablated == 2).mean() * 100
    part_strs = []
    for p, ps in zip(parts, part_subs):
        part_strs.append(f"remove({p})={( ps.kp_ablated==2).mean()*100:.1f}%")
    print(f"  {key}: {pct_kp2_pair:.1f}% pick Kp=2  |  singles: {', '.join(part_strs)}")
    additive = sum((ps.kp_ablated==2).mean() for ps in part_subs) * 100
    print(f"    Additive estimate: {additive:.1f}% | Actual pair: {pct_kp2_pair:.1f}% | "
          f"Super-additive: {pct_kp2_pair > additive}")
