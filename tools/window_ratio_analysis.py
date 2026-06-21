"""
tools/window_ratio_analysis.py

Post-hoc analysis of window_ratio_regression_py.csv.

Run AFTER window_ratio_regression.py to:
  1. Verify R² improvement over the over_sr regression (R²=0.33 baseline)
  2. Identify whether ceiling or floor is the binding bottleneck (determines
     what to improve next: floor mechanism study vs. ceiling mechanism study)
  3. Test if wind_strength or keff separation from td improves the fit
  4. Check whether window_ratio threshold predicts the FRAGILE label

Reads: experiments/results/window_ratio_regression_py.csv
       experiments/results/window_ratio_sweep_detail_py.csv
       experiments/results/exp1_final_population_py.csv (for FRAGILE labels)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.stats import pearsonr, spearmanr
from sklearn.metrics import roc_auc_score

ROOT = Path(__file__).resolve().parents[1]
RESULT_DIR = ROOT / 'experiments' / 'results'

WR_CSV    = RESULT_DIR / 'window_ratio_regression_py.csv'
SWEEP_CSV = RESULT_DIR / 'window_ratio_sweep_detail_py.csv'
POP_CSV   = RESULT_DIR / 'exp1_final_population_py.csv'


def _ols(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    y_hat = X @ beta
    ss_res = float(np.sum((y - y_hat)**2))
    ss_tot = float(np.sum((y - y.mean())**2))
    return beta, 1 - ss_res / ss_tot


def _cv_r2(X, y, k=5, seed=99):
    rng = np.random.RandomState(seed)
    idx = rng.permutation(len(y))
    folds = np.array_split(idx, k)
    r2s = []
    for i in range(k):
        val = folds[i]
        trn = np.concatenate([folds[j] for j in range(k) if j != i])
        beta, *_ = np.linalg.lstsq(X[trn], y[trn], rcond=None)
        yv, yh = y[val], X[val] @ beta
        ss_t = float(np.sum((yv - yv.mean())**2))
        r2s.append(1 - float(np.sum((yv - yh)**2)) / ss_t if ss_t > 0 else 0.0)
    return float(np.mean(r2s)), float(np.std(r2s))


def section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print('='*60)


def main():
    if not WR_CSV.exists():
        print(f"ERROR: {WR_CSV} not found. Run window_ratio_regression.py first.")
        return

    df = pd.read_csv(WR_CSV)
    sweep = pd.read_csv(SWEEP_CSV) if SWEEP_CSV.exists() else None

    valid = df.dropna(subset=['window_ratio']).copy()
    valid = valid[valid['window_ratio'] > 0]
    n = len(valid)
    print(f"Loaded {len(df)} designs, {n} valid (non-NaN window_ratio)")
    print(f"  Floor censored: {valid['floor_censored'].sum()}")
    print(f"  Ceil censored:  {valid['ceil_censored'].sum()}")

    wr  = valid['window_ratio'].values
    ltd = np.log(valid['td'].values)
    llat = np.log(valid['latency_steps'].values.astype(float))
    lkeff = np.log(valid['keff'].values)
    lumax = np.log(valid['u_max'].values)
    lwind = np.log(valid['wind_strength'].values)
    lslew = np.log(valid['servo_slew'].values)
    y = np.log(wr)
    ones = np.ones(n)

    # ── Section 1: Overview ───────────────────────────────────────────────────
    section("WINDOW RATIO DISTRIBUTION")
    print(f"  min={wr.min():.1f}×  p10={np.percentile(wr,10):.1f}×  "
          f"median={np.median(wr):.1f}×  p90={np.percentile(wr,90):.1f}×  max={wr.max():.1f}×")
    print(f"  Designs with window < 10×: {(wr < 10).sum()} ({100*(wr<10).mean():.1f}%)")
    print(f"  Designs with window < 20×: {(wr < 20).sum()} ({100*(wr<20).mean():.1f}%)")
    print(f"  Designs with window > 100×: {(wr > 100).sum()} ({100*(wr>100).mean():.1f}%)")

    # ── Section 2: Main regressions ───────────────────────────────────────────
    section("REGRESSION: log(window_ratio) on hardware specs")
    print("  Baseline comparison: over_sr regression had R²=0.33 (CV), now:")

    def show(label, X, y_arr=y):
        b, r2 = _ols(X, y_arr)
        cv, cv_s = _cv_r2(X, y_arr)
        coef = ' '.join(f'{v:+.3f}' for v in b)
        print(f"  {label:<36} R²={r2:.3f}  CV={cv:.3f}±{cv_s:.3f}  [{coef}]")
        return r2, cv

    show("log_td only",                    np.c_[ones, ltd])
    show("log_keff only",                  np.c_[ones, lkeff])
    show("log_td + log_lat",               np.c_[ones, ltd, llat])
    show("log_keff + log_lat",             np.c_[ones, lkeff, llat])
    show("log_keff+lat+umax",              np.c_[ones, lkeff, llat, lumax])
    show("log_td+lat+keff",                np.c_[ones, ltd, llat, lkeff])
    show("log_td+lat+wind",                np.c_[ones, ltd, llat, lwind])
    show("log_td+lat+wind+slew",           np.c_[ones, ltd, llat, lwind, lslew])
    best_r2, best_cv = show("log_keff+lat+wind",
                             np.c_[ones, lkeff, llat, lwind])
    show("log(td×lat) product",            np.c_[ones, ltd + llat])

    # ── Section 3: Ceiling vs Floor comparison ────────────────────────────────
    section("CEILING VS FLOOR: which drives the window?")

    vc = valid[~valid['ceil_censored'] & valid['kp_ceiling'].notna()].copy()
    vf = valid[~valid['floor_censored'] & valid['kp_floor'].notna()].copy()
    print(f"  Non-censored ceiling: n={len(vc)}  Non-censored floor: n={len(vf)}")

    if len(vc) >= 10:
        lc = np.log(vc['kp_ceiling'].values)
        lkc = np.log(vc['keff'].values); llc = np.log(vc['latency_steps'].values.astype(float))
        ltd_c = np.log(vc['td'].values); lw_c = np.log(vc['wind_strength'].values)
        oc = np.ones(len(vc))
        print(f"\n  Ceiling regressions (theory: keff^-0.20, lat^-1.10):")
        show("  ceil ~ keff+lat     ", np.c_[oc, lkc, llc], lc)
        show("  ceil ~ td+lat       ", np.c_[oc, ltd_c, llc], lc)
        show("  ceil ~ keff+lat+wind", np.c_[oc, lkc, llc, lw_c], lc)

    if len(vf) >= 10:
        lf = np.log(vf['kp_floor'].values)
        lkf = np.log(vf['keff'].values); llf = np.log(vf['latency_steps'].values.astype(float))
        ltd_f = np.log(vf['td'].values); lw_f = np.log(vf['wind_strength'].values)
        of = np.ones(len(vf))
        print(f"\n  Floor regressions (theory: keff^+0.70):")
        show("  floor ~ keff only   ", np.c_[of, lkf], lf)
        show("  floor ~ keff+lat    ", np.c_[of, lkf, llf], lf)
        show("  floor ~ td+lat      ", np.c_[of, ltd_f, llf], lf)
        show("  floor ~ keff+wind   ", np.c_[of, lkf, lw_f], lf)
        show("  floor ~ keff+lat+wind", np.c_[of, lkf, llf, lw_f], lf)

    # ── Section 4: Binned means by (td, latency) ─────────────────────────────
    section("BINNED MEANS: window_ratio by td and latency")
    for lat_val in sorted(valid['latency_steps'].unique()):
        sub_lat = valid[valid['latency_steps'] == lat_val]
        if len(sub_lat) < 3:
            continue
        td_labels = ['td<40', 'td40-80', 'td80-120', 'td120-180', 'td180-300', 'td>300']
        td_bins   = [0, 40, 80, 120, 180, 300, 3000]
        sub_lat = sub_lat.copy()
        sub_lat['tdb'] = pd.cut(sub_lat['td'], bins=td_bins, labels=td_labels, right=False)
        print(f"\n  latency={lat_val}:")
        for lbl in td_labels:
            cell = sub_lat[sub_lat['tdb'] == lbl]
            if len(cell) == 0:
                continue
            print(f"    {lbl:12}: n={len(cell):2d}  "
                  f"median={cell['window_ratio'].median():7.1f}×  "
                  f"mean={cell['window_ratio'].mean():7.1f}×")

    # ── Section 5: window_ratio as FRAGILE predictor ──────────────────────────
    section("WINDOW_RATIO AS FRAGILE PREDICTOR")
    print("  (comparing to AUC=0.975 for theta_ddot, 0.991 for theta_ddot*latency)")
    if POP_CSV.exists():
        pop = pd.read_csv(POP_CSV)
        label_col = 'final_label' if 'final_label' in pop.columns else 'regime_label'
        merged = valid.merge(pop[['rocket_id', label_col]].drop_duplicates(),
                             on='rocket_id', how='left')
        if label_col in merged.columns and merged[label_col].notna().sum() > 0:
            known = merged.dropna(subset=[label_col])
            if len(known) > 0:
                is_fragile = (known[label_col] == 'FRAGILE').astype(int)
                auc_wr  = roc_auc_score(is_fragile, -np.log(known['window_ratio'].values + 1e-6))
                auc_td  = roc_auc_score(is_fragile, known['td'].values)
                print(f"  n_with_labels={len(known)}  "
                      f"n_FRAGILE={is_fragile.sum()}")
                print(f"  AUC(window_ratio -> FRAGILE) = {auc_wr:.3f}")
                print(f"  AUC(td -> FRAGILE)           = {auc_td:.3f}")

                for thresh in [5, 10, 15, 20, 30, 50]:
                    pred = (known['window_ratio'] < thresh).astype(int)
                    tp = int(((pred==1) & (is_fragile==1)).sum())
                    fp = int(((pred==1) & (is_fragile==0)).sum())
                    fn = int(((pred==0) & (is_fragile==1)).sum())
                    tn = int(((pred==0) & (is_fragile==0)).sum())
                    prec = tp/(tp+fp) if (tp+fp)>0 else 0
                    rec  = tp/(tp+fn) if (tp+fn)>0 else 0
                    f1   = 2*prec*rec/(prec+rec) if (prec+rec)>0 else 0
                    print(f"  thresh={thresh:3d}×: TP={tp} FP={fp} FN={fn} TN={tn} "
                          f"prec={prec:.2f} rec={rec:.2f} F1={f1:.2f}")
        else:
            print("  No regime labels matched in the window_ratio sample.")
    else:
        print(f"  {POP_CSV} not found — skipping FRAGILE classifier test.")

    # ── Section 6: Sweep shape examples ──────────────────────────────────────
    if sweep is not None:
        section("SWEEP SHAPE EXAMPLES (SR vs Kp)")
        for td_target, label in [(20, 'EASY low-td'), (100, 'FRAGILE-zone'),
                                  (200, 'high-td')]:
            row_idx = (valid['td'] - td_target).abs().idxmin()
            rid = valid.loc[row_idx, 'rocket_id']
            row_data = valid.loc[row_idx]
            s = sweep[sweep['rocket_id'] == rid].sort_values('Kp')
            if len(s) == 0:
                continue
            print(f"\n  {label}: {rid}  td={row_data['td']:.0f}  "
                  f"lat={row_data['latency_steps']}  "
                  f"window={row_data['window_ratio']:.1f}×")
            for _, sr in s.iterrows():
                bar = '#' * int(sr['sr'] * 20)
                marker = ' <- floor' if abs(sr['Kp'] - row_data['kp_floor']) < sr['Kp']*0.1 else ''
                marker = marker or (' <- ceiling' if abs(sr['Kp'] - row_data['kp_ceiling']) < sr['Kp']*0.1 else '')
                print(f"    Kp={sr['Kp']:7.1f}  SR={sr['sr']:.2f}  {bar}{marker}")

    print("\nDone.")


if __name__ == '__main__':
    main()
