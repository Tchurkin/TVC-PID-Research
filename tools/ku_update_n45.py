"""
tools/ku_update_n45.py

Updates the K_u (ultimate gain) separation analysis from n=41/old-space relay study
to n=45 FRAGILE labels using flight_sig_updated_py.csv (Kp=2, 7-seed, full physics).

Estimate: A_deg ≈ RMS_mean * sqrt(2)  [sinusoidal approximation]
          K_u = 4 * u_max / (pi * A_rad)

Compares EASY vs FRAGILE K_u on current n=45 labels.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from scipy import stats

F15_T_AVG = 14.4
L_NOZZLE  = 0.25
CU_TO_RAD = (np.pi / 180) * (15 / 12)

FS_CSV  = Path('experiments/results/flight_sig_updated_py.csv')
EXP_CSV = Path('experiments/results/exp1_regime_index_py.csv')


def main():
    print("Loading flight sig and exp1 data...")
    fs  = pd.read_csv(FS_CSV)
    exp = pd.read_csv(EXP_CSV)

    # Per-design 7-seed mean RMS
    per = (fs.groupby(['rocket_id', 'regime', 'is_fragile'])
           .agg(rms_mean=('rms', 'mean'), td_max=('td_max', 'first'))
           .reset_index())

    # Merge in u_max from exp1
    exp['u_max'] = exp['max_gimbal_deg'] * 12.0 / 15.0
    per = per.merge(exp[['rocket_id', 'u_max']], on='rocket_id', how='left')

    # Estimate oscillation amplitude and K_u
    per['A_deg'] = per['rms_mean'] * np.sqrt(2)   # sinusoidal approx
    per['A_rad'] = per['A_deg'] * np.pi / 180.0
    per['K_u']   = 4.0 * per['u_max'] / (np.pi * per['A_rad'].clip(lower=1e-4))

    frag = per[per['is_fragile'] == 1]['K_u']
    easy = per[per['is_fragile'] == 0]['K_u']

    print(f"\n=== K_u SEPARATION (n=45 FRAGILE labels, from flight_sig Kp=2 data) ===")
    print(f"Method: A_deg = RMS_mean x sqrt(2), K_u = 4*u_max / (pi*A_rad)")
    print(f"Note: This approximates bang-bang oscillation amplitude from RMS.")
    print()
    print(f"EASY    K_u:  median={easy.median():.0f}  mean={easy.mean():.0f}  "
          f"range=[{easy.min():.0f}, {easy.max():.0f}]  (n={len(easy)})")
    print(f"FRAGILE K_u:  median={frag.median():.0f}  mean={frag.mean():.0f}  "
          f"range=[{frag.min():.0f}, {frag.max():.0f}]  (n={len(frag)})")
    print(f"Separation: {easy.median()/frag.median():.1f}x median ratio")

    # Mann-Whitney test
    mw = stats.mannwhitneyu(easy, frag, alternative='greater')
    print(f"Mann-Whitney (EASY > FRAGILE): U={mw.statistic:.0f}, p={mw.pvalue:.4e}")

    # How this compares to old relay study
    print(f"\nComparison to old relay study (n=41, old Iyy range):")
    print(f"  OLD: EASY median K_u = 108, FRAGILE median = 39  (2.8x, p=0.0072)")
    print(f"  NEW: EASY median K_u = {easy.median():.0f}, FRAGILE median = {frag.median():.0f}  "
          f"({easy.median()/frag.median():.1f}x, p={mw.pvalue:.4e})")
    print(f"  Note: A_deg estimate from RMS is approximate (assumes sinusoidal).")
    print(f"  Direction should be correct; exact K_u values differ from bang-bang relay.")

    # Correlation between K_u and theta_ddot
    r_ku_td = np.corrcoef(np.log(per['K_u'].clip(lower=1)), np.log(per['td_max']))[0,1]
    print(f"\nCorrelation log(K_u) vs log(td_max):  r = {r_ku_td:.3f}")
    print(f"  (Negative expected: higher authority -> lower ceiling -> lower K_u)")

    # AUC of K_u for predicting FRAGILE
    try:
        from sklearn.metrics import roc_auc_score
        # Higher K_u -> less likely FRAGILE (so negate for AUC)
        auc_ku = roc_auc_score(per['is_fragile'], -per['K_u'])
        print(f"\nAUC(-K_u) for predicting FRAGILE: {auc_ku:.3f}")
        print(f"  (AUC of theta_ddot on same 90 designs: {roc_auc_score(per['is_fragile'], per['td_max']):.3f})")
    except ImportError:
        pass

    # Show the worst-separated designs (FRAGILE with high K_u = missed by ceiling compression)
    print(f"\nFRAGILE designs with highest K_u (least clear ceiling compression):")
    frag_df = per[per['is_fragile'] == 1].sort_values('K_u', ascending=False).head(5)
    for _, r in frag_df.iterrows():
        print(f"  {r['rocket_id']:8s}  td={r['td_max']:6.1f}  RMS={r['rms_mean']:.1f}  "
              f"K_u={r['K_u']:.0f}")

    print(f"\nEASY designs with lowest K_u (closest to FRAGILE territory):")
    easy_df = per[per['is_fragile'] == 0].sort_values('K_u').head(5)
    for _, r in easy_df.iterrows():
        print(f"  {r['rocket_id']:8s}  td={r['td_max']:6.1f}  RMS={r['rms_mean']:.1f}  "
              f"K_u={r['K_u']:.0f}")


if __name__ == '__main__':
    main()
