"""
tools/window_formula_validation.py

Validates the STITCHED window-ratio formula against held-out simulation data.

The two pieces were each fit on SEPARATE datasets:
  - Ceiling: K_u_ceiling ~ 380 / latency_steps   (DIPDT theory + 2.1x empirical
    correction, calibrated from a small power-law fit n=20 + 5 spot checks)
  - Floor:   Kp_floor ~ 0.35 * keff_full^0.70     (relay study, n=41)

Neither formula was fit on gain_window_v2_summary_py.csv (180 rows, 12 designs
x 5 wind x 3 latency). That makes this a genuine held-out test, not a refit.

This script checks:
  1. Ceiling formula vs measured kp_ceil (log-log R^2, only latency varies in formula)
  2. Floor formula vs measured kp_floor (log-log R^2, only keff varies in formula)
  3. Combined window_ratio = ceiling/floor vs measured window_ratio
  4. Whether including wind or group_type (EASY/FRAGILE) as a residual predictor
     explains additional variance the formula misses (residual diagnostic, not
     an extra fitted term)
"""

import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.metrics import r2_score
from scipy.stats import pearsonr

CSV = Path('experiments/results/gain_window_v2_summary_py.csv')


def main():
    df = pd.read_csv(CSV)
    df = df[~df['collapsed']].copy()
    print(f"Loaded {len(df)} non-collapsed conditions (of original rows in {CSV})")

    # ── Formula predictions (NOT fit on this data) ──────────────────────────
    df['ceil_pred']   = 380.0 / df['latency']
    df['floor_pred']  = 0.35 * df['keff'] ** 0.70
    df['window_pred'] = df['ceil_pred'] / df['floor_pred']

    print("\n" + "="*70)
    print("1. CEILING FORMULA:  K_u_ceiling = 380 / latency_steps")
    print("="*70)
    log_ceil_obs  = np.log(df['kp_ceil'])
    log_ceil_pred = np.log(df['ceil_pred'])
    r2_ceil = r2_score(log_ceil_obs, log_ceil_pred)
    r_ceil, p_ceil = pearsonr(log_ceil_obs, log_ceil_pred)
    print(f"  log-log R^2 = {r2_ceil:.3f}   r = {r_ceil:.3f}  (p={p_ceil:.2e})")
    print(f"  Mean abs log-ratio error: {np.abs(log_ceil_obs - log_ceil_pred).mean():.3f} "
          f"({np.exp(np.abs(log_ceil_obs - log_ceil_pred).mean()):.2f}x typical miss factor)")
    for lat in sorted(df['latency'].unique()):
        sub = df[df['latency'] == lat]
        print(f"    lat={lat}: predicted={380/lat:.0f}  observed median={sub['kp_ceil'].median():.0f}  "
              f"observed range=[{sub['kp_ceil'].min():.0f},{sub['kp_ceil'].max():.0f}]")

    print("\n" + "="*70)
    print("2. FLOOR FORMULA:  Kp_floor = 0.35 * keff_full^0.70")
    print("="*70)
    log_floor_obs  = np.log(df['kp_floor'])
    log_floor_pred = np.log(df['floor_pred'])
    r2_floor = r2_score(log_floor_obs, log_floor_pred)
    r_floor, p_floor = pearsonr(log_floor_obs, log_floor_pred)
    print(f"  log-log R^2 = {r2_floor:.3f}   r = {r_floor:.3f}  (p={p_floor:.2e})")
    print(f"  Mean abs log-ratio error: {np.abs(log_floor_obs - log_floor_pred).mean():.3f} "
          f"({np.exp(np.abs(log_floor_obs - log_floor_pred).mean()):.2f}x typical miss factor)")

    print("\n" + "="*70)
    print("3. COMBINED WINDOW RATIO:  window = ceiling/floor")
    print("="*70)
    log_win_obs  = np.log(df['window_ratio'])
    log_win_pred = np.log(df['window_pred'])
    r2_win = r2_score(log_win_obs, log_win_pred)
    r_win, p_win = pearsonr(log_win_obs, log_win_pred)
    print(f"  log-log R^2 = {r2_win:.3f}   r = {r_win:.3f}  (p={p_win:.2e})")
    print(f"  Mean abs log-ratio error: {np.abs(log_win_obs - log_win_pred).mean():.3f} "
          f"({np.exp(np.abs(log_win_obs - log_win_pred).mean()):.2f}x typical miss factor)")

    # Does the combined formula correctly RANK fragile vs easy windows?
    from sklearn.metrics import roc_auc_score
    y = (df['group_type'] == 'FRAGILE').astype(int)
    auc_pred_window = roc_auc_score(y, -df['window_pred'])  # narrower predicted window -> FRAGILE
    auc_obs_window  = roc_auc_score(y, -df['window_ratio'])
    print(f"\n  AUC(predicted window ranks FRAGILE): {auc_pred_window:.3f}")
    print(f"  AUC(observed window ranks FRAGILE):  {auc_obs_window:.3f}")

    print("\n" + "="*70)
    print("4. RESIDUAL DIAGNOSTIC: does wind or group_type explain leftover error?")
    print("="*70)
    df['resid'] = log_win_obs - log_win_pred
    r_wind, p_wind = pearsonr(df['resid'], df['wind'])
    print(f"  r(residual, wind) = {r_wind:.3f}  (p={p_wind:.3f})  "
          f"{'<- wind matters, formula misses it' if p_wind < 0.05 else '(not significant)'}")

    for gtype in ['EASY', 'FRAGILE']:
        sub = df[df['group_type'] == gtype]
        print(f"  {gtype:7s}: mean residual = {sub['resid'].mean():+.3f}  "
              f"(n={len(sub)}; {'formula overpredicts window' if sub['resid'].mean()<0 else 'formula underpredicts window'})")

    print("\n" + "="*70)
    print("5. BREAKDOWN BY td (theta_ddot) REGIME — where does the formula fail?")
    print("="*70)
    for td_lo, td_hi in [(0,70), (70,100), (100,250)]:
        sub = df[(df['td'] >= td_lo) & (df['td'] < td_hi)]
        if len(sub) < 3:
            continue
        r2_sub = r2_score(np.log(sub['window_ratio']), np.log(sub['window_pred']))
        print(f"  td in [{td_lo},{td_hi}): n={len(sub):3d}  log-log R^2={r2_sub:.3f}  "
              f"mean|resid|={np.abs(sub['resid']).mean():.3f}")

    print("\nDone.")


if __name__ == '__main__':
    main()
