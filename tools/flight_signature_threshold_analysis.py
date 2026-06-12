"""
flight_signature_threshold_analysis.py

Given the flight_signature_py.csv results, compute:
  1. Optimal detection thresholds (precision/recall/F1) for RMS and slew_sat
  2. Wind confound check: AUC by wind_strength tercile
  3. Spec-based (authority_inertia_ratio) vs flight-based detection comparison

This answers: "Can a builder detect gain-sensitivity from one test flight,
and is that better or worse than computing theta_ddot_max from specs?"
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))

import numpy as np
import pandas as pd
from sklearn.metrics import roc_auc_score, precision_recall_fscore_support

# Load flight signature results
df = pd.read_csv(os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "flight_signature_py.csv"))
exp1 = pd.read_csv(os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "exp1_regime_index_py.csv"))
qa   = pd.read_csv(os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "gain_window_sweep_py.csv"))

# Per-design 7-seed aggregate
agg = df.groupby(['rocket_id', 'regime', 'is_fragile'])[['rms', 'osc', 'slew_sat', 'peak']].mean().reset_index()
# Compute authority_inertia_ratio from exp1 columns
exp1['authority_inertia_ratio'] = (exp1['max_gimbal_deg'] * exp1['motor_scale']) / exp1['Iyy']
agg = agg.merge(exp1[['rocket_id', 'wind_strength', 'authority_inertia_ratio']], on='rocket_id', how='left')

print("=== 1. Optimal detection thresholds (7-seed aggregate) ===")
print()
for metric in ['rms', 'slew_sat', 'peak']:
    thresholds = np.percentile(agg[metric].dropna(), np.arange(5, 100, 5))
    best_f1 = 0; best_t = None; best_prec = None; best_rec = None
    for t in thresholds:
        pred = (agg[metric] > t).astype(int)
        p, r, f, _ = precision_recall_fscore_support(agg['is_fragile'], pred, average='binary', zero_division=0)
        if f > best_f1:
            best_f1, best_t, best_prec, best_rec = f, t, p, r
    auc = roc_auc_score(agg['is_fragile'], agg[metric])
    # Count at threshold
    pred_best = (agg[metric] > best_t).astype(int)
    tp = ((pred_best == 1) & (agg['is_fragile'] == 1)).sum()
    fp = ((pred_best == 1) & (agg['is_fragile'] == 0)).sum()
    fn = ((pred_best == 0) & (agg['is_fragile'] == 1)).sum()
    tn = ((pred_best == 0) & (agg['is_fragile'] == 0)).sum()
    print(f"  {metric}: AUC={auc:.3f}  Best threshold={best_t:.1f}")
    print(f"    Precision={best_prec:.2f}  Recall={best_rec:.2f}  F1={best_f1:.2f}")
    print(f"    TP={tp} FP={fp} FN={fn} TN={tn} (of 50 designs: 25F + 25E)")
    print()

# Also do single-seed analysis
print("=== 1b. Single-seed thresholds (1 test flight) ===")
single = df[df['seed'] == 1].copy()
for metric in ['rms', 'slew_sat', 'peak']:
    thresholds = np.percentile(single[metric].dropna(), np.arange(5, 100, 5))
    best_f1 = 0; best_t = None; best_prec = None; best_rec = None
    for t in thresholds:
        pred = (single[metric] > t).astype(int)
        p, r, f, _ = precision_recall_fscore_support(single['is_fragile'], pred, average='binary', zero_division=0)
        if f > best_f1:
            best_f1, best_t, best_prec, best_rec = f, t, p, r
    auc = roc_auc_score(single['is_fragile'], single[metric])
    print(f"  {metric}: AUC={auc:.3f}  Best threshold={best_t:.1f}  F1={best_f1:.2f}  Prec={best_prec:.2f}  Rec={best_rec:.2f}")
print()

# === 2. Wind confound check ===
print("=== 2. Wind confound: AUC by wind_strength tercile ===")
agg['wind_tercile'] = pd.qcut(agg['wind_strength'], 3, labels=['low', 'mid', 'high'])
for terc in ['low', 'mid', 'high']:
    sub = agg[agg['wind_tercile'] == terc]
    if sub['is_fragile'].nunique() < 2:
        print(f"  {terc} wind: only one class present (n={len(sub)}), skipping AUC")
        continue
    auc_rms  = roc_auc_score(sub['is_fragile'], sub['rms'])
    auc_slew = roc_auc_score(sub['is_fragile'], sub['slew_sat'])
    wmin = sub['wind_strength'].min(); wmax = sub['wind_strength'].max()
    nf = (sub['is_fragile'] == 1).sum(); ne = (sub['is_fragile'] == 0).sum()
    print(f"  {terc} wind [{wmin:.2f},{wmax:.2f}]: n=F{nf}/E{ne}  AUC(rms)={auc_rms:.3f}  AUC(slew)={auc_slew:.3f}")
print()

# === 3. Spec-based vs flight-based comparison on same 50 designs ===
print("=== 3. Spec-based (authority_inertia_ratio) vs flight-based on same 50 designs ===")

# authority_inertia_ratio from exp1 for these 50 designs
agg_spec = agg[['rocket_id', 'is_fragile', 'authority_inertia_ratio', 'rms', 'slew_sat', 'peak']].dropna()
print(f"  Designs with authority_inertia_ratio: {len(agg_spec)}")

if len(agg_spec) > 0 and agg_spec['is_fragile'].nunique() > 1:
    auc_ratio = roc_auc_score(agg_spec['is_fragile'], agg_spec['authority_inertia_ratio'])
    auc_rms   = roc_auc_score(agg_spec['is_fragile'], agg_spec['rms'])
    auc_slew  = roc_auc_score(agg_spec['is_fragile'], agg_spec['slew_sat'])
    auc_peak  = roc_auc_score(agg_spec['is_fragile'], agg_spec['peak'])
    print(f"  Spec-based:   AUC(authority_inertia_ratio) = {auc_ratio:.3f}")
    print(f"  Flight-based: AUC(rms 7-seed)              = {auc_rms:.3f}")
    print(f"  Flight-based: AUC(slew_sat 7-seed)         = {auc_slew:.3f}")
    print(f"  Flight-based: AUC(peak 7-seed)             = {auc_peak:.3f}")
    print()
    # Single-seed flight vs spec
    single_agg = df[df['seed']==1][['rocket_id','is_fragile','rms','slew_sat','peak']].copy()
    single_agg = single_agg.merge(agg_spec[['rocket_id','authority_inertia_ratio']], on='rocket_id', how='inner')
    if single_agg['is_fragile'].nunique() > 1:
        auc_rms1  = roc_auc_score(single_agg['is_fragile'], single_agg['rms'])
        auc_slew1 = roc_auc_score(single_agg['is_fragile'], single_agg['slew_sat'])
        auc_peak1 = roc_auc_score(single_agg['is_fragile'], single_agg['peak'])
        print(f"  Single-flight: AUC(rms)     = {auc_rms1:.3f}")
        print(f"  Single-flight: AUC(slew_sat) = {auc_slew1:.3f}")
        print(f"  Single-flight: AUC(peak)    = {auc_peak1:.3f}")
        print(f"  --> Spec formula AUC: {auc_ratio:.3f} | 1-flight RMS AUC: {auc_rms1:.3f}")
        if auc_rms1 > auc_ratio:
            print(f"  --> Flight-based outperforms spec by delta={auc_rms1-auc_ratio:.3f}")
        else:
            print(f"  --> Spec-based outperforms flight by delta={auc_ratio-auc_rms1:.3f}")

# === 4. Summary stats on class separation ===
print()
print("=== 4. Class separation at Kp=2 ===")
fragile = agg[agg['is_fragile']==1]
easy    = agg[agg['is_fragile']==0]
for metric in ['rms', 'slew_sat', 'peak']:
    fm = fragile[metric].mean(); fs = fragile[metric].std()
    em = easy[metric].mean();   es = easy[metric].std()
    ratio = fm/em if em > 0 else float('inf')
    print(f"  {metric}: FRAGILE={fm:.2f}+/-{fs:.2f}  EASY={em:.2f}+/-{es:.2f}  ratio={ratio:.1f}x")
print()
print(f"  Mean SR at Kp=2: FRAGILE={fragile['regime'].count()} designs,", end=' ')
fsr = df[df['is_fragile']==1]['success'].mean()
esr = df[df['is_fragile']==0]['success'].mean()
print(f"SR={fsr:.2f}  |  EASY SR={esr:.2f}")
