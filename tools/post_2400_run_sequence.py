"""
tools/post_2400_run_sequence.py

Quick stats summary after 2400-design Exp1 run.
Run this first to confirm the run succeeded before launching forensics + S2R.

Usage: python tools/post_2400_run_sequence.py
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path

sys.path.insert(0, 'sim')

CSV = Path('experiments/results/exp1_regime_index_py.csv')
assert CSV.exists(), f"{CSV} not found. Did exp1 save correctly?"

df = pd.read_csv(CSV)

F15 = 14.4; L = 0.25; CU2RAD = np.pi/180*15/12
df['keff_full'] = F15 * df['motor_scale'] * CU2RAD * L / df['Iyy']
df['u_max']     = df['max_gimbal_deg'] * 12/15
df['td']        = df['keff_full'] * df['u_max']

n = len(df)
counts = df['regime_label'].value_counts()

print(f"=== 2400-design Exp1 Results ===")
print(f"n = {n}")
for r in ['EASY','MARGINAL','FRAGILE','INFEASIBLE']:
    nr = counts.get(r, 0)
    print(f"  {r:12s}: {nr:4d} ({100*nr/n:.1f}%)")

easy = df[df['regime_label']=='EASY']['td']
frag = df[df['regime_label']=='FRAGILE']['td']
print(f"\ntheta_ddot_max:")
print(f"  EASY:    mean={easy.mean():.1f}  median={easy.median():.1f}  n={len(easy)}")
print(f"  FRAGILE: mean={frag.mean():.1f}  median={frag.median():.1f}  n={len(frag)}")
print(f"  Ratio:   {frag.mean()/easy.mean():.1f}x")

# Quick AUC
try:
    from sklearn.metrics import roc_auc_score
    y = df['regime_label'].isin(['FRAGILE']).astype(int)
    mask = df['regime_label'].isin(['EASY','MARGINAL','FRAGILE'])
    y_m = y[mask]
    s_m = df.loc[mask, 'td']
    auc = roc_auc_score(y_m, s_m)
    print(f"\n  AUC(theta_ddot): {auc:.3f}  (n_FRAGILE={y_m.sum()})")
except ImportError:
    print("  (sklearn not available for AUC)")

# Low-td anomalous FRAGILE
TD_THRESH = 30.0
frag_df = df[df['regime_label']=='FRAGILE']
anom = frag_df[frag_df['td'] < TD_THRESH]
if len(anom) > 0:
    print(f"\n  WARNING: {len(anom)} FRAGILE designs with td < {TD_THRESH} rad/s2:")
    for _, r in anom.iterrows():
        print(f"    {r['design_id']}: td={r['td']:.1f}  Kp={r['best_Kp']:.1f}  "
              f"max_gimbal={r['max_gimbal_deg']:.1f}deg  Iyy={r['Iyy']:.4f}  "
              f"nom_sr={r['nominal_success_rate']:.3f}  over_sr={r['over_success_rate']:.3f}")
    print(f"  Run anomalous_fragile_forensics.py to verify these.")
else:
    print(f"\n  No FRAGILE designs below {TD_THRESH} rad/s2.")
    print(f"  AUC should be higher without low-td anomalies.")

print("\n=== NEXT STEPS ===")
print("1. Run: python tools/anomalous_fragile_forensics.py  (if anomalous designs found)")
print("2. Run: python tools/post_exp1_analysis.py  (full stats)")
print("3. Launch S2R: python -c \"import sys; sys.path.insert(0,'sim');"
      " from experiment_runner import run_exp4_s2r_gains; run_exp4_s2r_gains()\"")
