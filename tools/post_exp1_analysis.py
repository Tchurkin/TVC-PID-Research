"""
tools/post_exp1_analysis.py

Comprehensive post-Exp1 analysis pipeline.
Runs after any Exp1 CSV update (1200 or 2400 designs).

Outputs:
  1. Regime counts and comparison
  2. theta_ddot_max predictor: AUC, CI, Cohen d, Youden-J threshold
  3. H1-H5 hypothesis screening (correlations + AUC delta)
  4. Combined predictor (theta_ddot × latency): AUC
  5. FRAGILE subtype breakdown (ceiling vs floor vs both)
  6. Anomalous low-td FRAGILE analysis
  7. Simple summary table for CLAUDE.md updates
"""

import sys, warnings
import numpy as np
import pandas as pd
from pathlib import Path
from scipy import stats

sys.path.insert(0, 'sim')
warnings.filterwarnings('ignore')

# ── sklearn imports ───────────────────────────────────────────────────────────
try:
    from sklearn.metrics import roc_auc_score
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler
    from sklearn.model_selection import StratifiedKFold, cross_val_score
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False
    print("WARNING: sklearn not available. AUC computations skipped.")

# ── Constants ─────────────────────────────────────────────────────────────────
F15_AVG_N = 14.4
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * 15/12  # 0.02182 rad/CU
OVER_SCALE  = 1.40
ROBUSTNESS_RATE = 0.80
FRAGILE_RATE    = 0.35

CSV_PATH = Path('experiments/results/exp1_regime_index_py.csv')


def bootstrap_auc(y, scores, n_boot=10_000, rng_seed=0):
    rng = np.random.default_rng(rng_seed)
    base_auc = roc_auc_score(y, scores)
    boot_aucs = []
    n = len(y)
    for _ in range(n_boot):
        idx = rng.integers(0, n, size=n)
        ys, ss = y[idx], scores[idx]
        if len(np.unique(ys)) < 2:
            continue
        boot_aucs.append(roc_auc_score(ys, ss))
    lo, hi = np.percentile(boot_aucs, [2.5, 97.5])
    return base_auc, lo, hi


def cv_auc(X, y, n_splits=10, rng_seed=0):
    if not HAS_SKLEARN:
        return np.nan, np.nan
    scaler = StandardScaler()
    Xs = scaler.fit_transform(X.reshape(-1, 1) if X.ndim == 1 else X)
    clf = LogisticRegression(max_iter=2000, random_state=rng_seed)
    cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=rng_seed)
    scores = cross_val_score(clf, Xs, y, cv=cv, scoring='roc_auc')
    return scores.mean(), scores.std()


def sep(s='', width=65):
    print(f"\n{'='*width}" + (f' {s}' if s else ''))


# ═════════════════════════════════════════════════════════════════════════════
# Load and derive
# ═════════════════════════════════════════════════════════════════════════════
print(f"Loading {CSV_PATH}")
df = pd.read_csv(CSV_PATH)
n  = len(df)

df['keff_full']      = F15_AVG_N * df['motor_scale'] * CU_TO_RAD * L_NOZZLE / df['Iyy']
df['u_max']          = df['max_gimbal_deg'] * 12.0 / 15.0
df['td_max']         = df['keff_full'] * df['u_max']
df['authority_ratio']= df['max_gimbal_deg'] * df['motor_scale'] / df['Iyy']
df['log_td']         = np.log(df['td_max'])
df['log_keff']       = np.log(df['keff_full'])
df['log_wind']       = np.log(df['wind_strength'])
df['log_slew']       = np.log(df['servo_slew_deg_s'])
df['log_latency']    = np.log(df['latency_steps'].clip(lower=1))
df['td_x_latency']   = df['log_td'] + df['log_latency']
df['log_authority']  = np.log(df['authority_ratio'])
df['wind_x_keff']    = df['wind_strength'] * df['keff_full']
df['wind_x_td']      = df['wind_strength'] * df['td_max']
df['td_div_slew']    = df['td_max'] / df['servo_slew_deg_s']
df['keff_div_slew']  = df['keff_full'] / df['servo_slew_deg_s']

df['is_fragile'] = (df['regime_label'] == 'FRAGILE').astype(int)
df_model = df[df['regime_label'].isin(['EASY', 'MARGINAL', 'FRAGILE'])].copy()
y = df_model['is_fragile'].values


# ═════════════════════════════════════════════════════════════════════════════
# 1. Regime counts
# ═════════════════════════════════════════════════════════════════════════════
sep('REGIME COUNTS')
counts = df['regime_label'].value_counts()
for regime in ['EASY', 'MARGINAL', 'FRAGILE', 'INFEASIBLE']:
    n_r = counts.get(regime, 0)
    pct = 100 * n_r / n
    print(f"  {regime:12s}: n={n_r:5d} ({pct:.1f}%)")
print(f"  TOTAL: {n}")


# ═════════════════════════════════════════════════════════════════════════════
# 2. theta_ddot_max predictor
# ═════════════════════════════════════════════════════════════════════════════
sep('THETA_DDOT_MAX PREDICTOR (FINDING 2)')
for regime in ['EASY', 'FRAGILE', 'MARGINAL']:
    sub = df[df['regime_label'] == regime]['td_max']
    if len(sub) > 0:
        print(f"  {regime:12s}: mean={sub.mean():.1f}  median={sub.median():.1f}  "
              f"range=[{sub.min():.1f}, {sub.max():.1f}]  n={len(sub)}")

easy_td = df[df['regime_label'] == 'EASY']['td_max']
frag_td = df[df['regime_label'] == 'FRAGILE']['td_max']
if len(frag_td) > 0 and len(easy_td) > 0:
    ratio = frag_td.mean() / easy_td.mean()
    t_stat, p_val = stats.ttest_ind(frag_td, easy_td)
    pooled_std = np.sqrt((frag_td.std()**2 + easy_td.std()**2) / 2)
    d = (frag_td.mean() - easy_td.mean()) / pooled_std
    print(f"\n  FRAGILE/EASY mean ratio: {ratio:.2f}x")
    print(f"  Cohen d={d:.2f}, t={t_stat:.2f}, p={p_val:.2e}")

    if HAS_SKLEARN:
        td_scores = df_model['td_max'].values
        auc, lo, hi = bootstrap_auc(y, td_scores, n_boot=10_000)
        cv_mean, cv_std = cv_auc(td_scores, y)
        print(f"\n  AUC(theta_ddot_max): {auc:.3f} [{lo:.3f}, {hi:.3f}] 95% CI")
        print(f"  10-fold CV:          {cv_mean:.3f} ± {cv_std:.3f}")

        # Youden-J threshold
        from sklearn.metrics import roc_curve
        fpr, tpr, thresholds = roc_curve(y, td_scores)
        j_idx = np.argmax(tpr - fpr)
        youden_j = thresholds[j_idx]
        print(f"  Youden-J threshold: {youden_j:.1f} rad/s2 "
              f"(TPR={tpr[j_idx]:.2f}, FPR={fpr[j_idx]:.2f})")

        # Threshold at 70 (zero false approvals in historic space)
        for thresh in [50, 70, 100]:
            pred = (td_scores >= thresh).astype(int)
            tp = int(np.sum((pred == 1) & (y == 1)))
            fp = int(np.sum((pred == 1) & (y == 0)))
            fn = int(np.sum((pred == 0) & (y == 1)))
            tn = int(np.sum((pred == 0) & (y == 0)))
            prec = tp/(tp+fp) if (tp+fp) > 0 else 0
            rec  = tp/(tp+fn) if (tp+fn) > 0 else 0
            print(f"  threshold={thresh}: TP={tp} FP={fp} FN={fn} TN={tn} "
                  f"prec={prec:.3f} recall={rec:.3f}")


# ═════════════════════════════════════════════════════════════════════════════
# 3. H1-H5 hypothesis screening
# ═════════════════════════════════════════════════════════════════════════════
sep('HYPOTHESIS SCREENING (H1-H5)')
hypotheses = {
    'H1 wind_strength':       'wind_strength',
    'H2 servo_slew':          'servo_slew_deg_s',
    'H3 static_margin':       'static_margin',
    'H4 Cm_alpha':            'Cm_alpha',
    'H5 latency_steps':       'latency_steps',
    'H6 authority_ratio':     'authority_ratio',
}
base_scores = df_model['td_max'].values
base_auc = roc_auc_score(y, base_scores) if HAS_SKLEARN else np.nan

for name, col in hypotheses.items():
    r = np.corrcoef(df_model[col], df_model['is_fragile'])[0, 1]
    if HAS_SKLEARN:
        solo_auc = roc_auc_score(y, df_model[col])
        # Combined: log(td) + log(col) (where col > 0)
        col_vals = df_model[col]
        if col_vals.min() > 0:
            combined = df_model['log_td'] + np.log(col_vals)
        else:
            combined = df_model['log_td'] + col_vals
        comb_auc = roc_auc_score(y, combined)
        delta = comb_auc - base_auc
        verdict = 'CONFIRMED' if delta >= 0.03 else 'REJECTED'
        print(f"  {name:25s}: r={r:+.3f}  solo_AUC={solo_auc:.3f}  "
              f"delta={delta:+.3f}  [{verdict}]")
    else:
        print(f"  {name:25s}: r={r:+.3f}")

# Special: log(td × latency) as single variable
if HAS_SKLEARN:
    td_lat = df_model['td_x_latency'].values
    auc_comb, lo_c, hi_c = bootstrap_auc(y, td_lat, n_boot=5000)
    cv_c, cv_s = cv_auc(td_lat, y)
    print(f"\n  Combined log(td×latency):  AUC={auc_comb:.3f} [{lo_c:.3f},{hi_c:.3f}]  "
          f"CV={cv_c:.3f}±{cv_s:.3f}")
    print(f"  vs td_alone:               AUC={base_auc:.3f}, delta={auc_comb-base_auc:+.3f}")


# ═════════════════════════════════════════════════════════════════════════════
# 4. FRAGILE subtype breakdown
# ═════════════════════════════════════════════════════════════════════════════
sep('FRAGILE SUBTYPE BREAKDOWN')
frag = df[df['regime_label'] == 'FRAGILE'].copy()
if len(frag) > 0:
    has_over  = 'over_success_rate' in frag.columns
    has_under = 'under_success_rate' in frag.columns
    if has_over and has_under:
        ceiling_lim = ((frag['over_success_rate']  < ROBUSTNESS_RATE) &
                       (frag['under_success_rate'] >= ROBUSTNESS_RATE)).sum()
        floor_lim   = ((frag['over_success_rate']  >= ROBUSTNESS_RATE) &
                       (frag['under_success_rate']  < ROBUSTNESS_RATE)).sum()
        both_fail   = ((frag['over_success_rate']   < ROBUSTNESS_RATE) &
                       (frag['under_success_rate']  < ROBUSTNESS_RATE)).sum()
        n_frag = len(frag)
        print(f"  Total FRAGILE: {n_frag}")
        print(f"  Ceiling-limited (over fails, under passes): {ceiling_lim} ({100*ceiling_lim/n_frag:.0f}%)")
        print(f"  Floor-limited   (over passes, under fails): {floor_lim}   ({100*floor_lim/n_frag:.0f}%)")
        print(f"  Both-limited    (both fail):                {both_fail}   ({100*both_fail/n_frag:.0f}%)")

    print(f"\n  FRAGILE sorted by theta_ddot_max:")
    cols_show = ['design_id','td_max','best_Kp','max_gimbal_deg','Iyy','keff_full',
                 'latency_steps','wind_strength','nominal_success_rate']
    if has_over:
        cols_show += ['over_success_rate']
    if has_under:
        cols_show += ['under_success_rate']
    cols_show = [c for c in cols_show if c in frag.columns]
    print(frag.sort_values('td_max')[cols_show].to_string(index=False))


# ═════════════════════════════════════════════════════════════════════════════
# 5. Anomalous low-theta_ddot FRAGILE identification
# ═════════════════════════════════════════════════════════════════════════════
sep('ANOMALOUS LOW-theta_ddot FRAGILE (possible false positives)')
TD_ANOMALY_THRESHOLD = 30.0  # rad/s2
anomalous = frag[frag['td_max'] < TD_ANOMALY_THRESHOLD] if len(frag) > 0 else pd.DataFrame()
if len(anomalous) > 0:
    print(f"  {len(anomalous)} FRAGILE designs below {TD_ANOMALY_THRESHOLD} rad/s2:")
    for _, row in anomalous.iterrows():
        print(f"    {row.get('design_id','?')}: td={row['td_max']:.1f}  "
              f"Kp={row.get('best_Kp','?'):.1f}  max_gimbal={row['max_gimbal_deg']:.1f}deg  "
              f"wind={row['wind_strength']:.3f}  lat={row['latency_steps']}  "
              f"over_sr={row.get('over_success_rate','?'):.3f}  "
              f"nom_sr={row.get('nominal_success_rate','?'):.3f}")
    print(f"\n  Recommendation: run anomalous_fragile_forensics.py on these designs")
    print(f"  Note: 3-seed robustness test has ~14% false-positive rate per design")
    print(f"  at true SR@1.4x=0.95.  Multi-seed verification needed.")
else:
    print(f"  No FRAGILE designs below {TD_ANOMALY_THRESHOLD} rad/s2 — good!")


# ═════════════════════════════════════════════════════════════════════════════
# 6. Summary for CLAUDE.md
# ═════════════════════════════════════════════════════════════════════════════
sep('SUMMARY FOR CLAUDE.md UPDATE')
n_easy = counts.get('EASY', 0)
n_marg = counts.get('MARGINAL', 0)
n_frag = counts.get('FRAGILE', 0)
n_inf  = counts.get('INFEASIBLE', 0)

print(f"""
  CURRENT counts (exp1_regime_index_py.csv, n={n}):
  * EASY:       n={n_easy:5d} ({100*n_easy/n:.1f}%)
  * MARGINAL:   n={n_marg:5d} ({100*n_marg/n:.1f}%)
  * FRAGILE:    n={n_frag:5d} ({100*n_frag/n:.1f}%)
  * INFEASIBLE: n={n_inf:5d} ({100*n_inf/n:.1f}%)

  theta_ddot_max statistics:
    EASY:    mean={easy_td.mean():.1f}  median={easy_td.median():.1f} rad/s2
    FRAGILE: mean={frag_td.mean():.1f}  median={frag_td.median():.1f} rad/s2
    Ratio:   {frag_td.mean()/easy_td.mean():.1f}x  d={((frag_td.mean()-easy_td.mean())/np.sqrt((frag_td.std()**2+easy_td.std()**2)/2)):.2f}
""")
if HAS_SKLEARN:
    auc_td, lo_td, hi_td = bootstrap_auc(y, df_model['td_max'].values)
    print(f"  AUC(theta_ddot_max): {auc_td:.3f} [{lo_td:.3f}, {hi_td:.3f}]")
