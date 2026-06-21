"""
tools/fragile_residual_analysis.py

Investigates why 4 designs (R0804, R0047, R0680, R0452) are FRAGILE despite
low theta_ddot_max values that fall below the 70 rad/s2 predictor threshold.

Tests H1-H4 and fits logistic regression models.
Compares keff_full vs theta_ddot_max as predictors.
"""

import numpy as np
import pandas as pd
from pathlib import Path
import sys
sys.path.insert(0, 'sim')

from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import cross_val_score, StratifiedKFold
from sklearn.metrics import roc_auc_score
from sklearn.preprocessing import StandardScaler

# ------------------------------------------------------------------ constants
F15_AVG_N   = 14.4       # N  (motor_scale=1.0 baseline)
L_NOZZLE    = 0.25       # m
CU_TO_RAD   = np.pi / 180 * 15/12   # 0.02182 rad per CU
# u_max = max_gimbal_deg * 12/15  (verified from simulator)
THRESHOLD_TD = 70.0      # original theta_ddot_max predictor threshold

# ------------------------------------------------------------------ load data
df = pd.read_csv('experiments/results/exp1_regime_index_py.csv')

# Compute derived variables
df['keff_full']      = F15_AVG_N * df['motor_scale'] * CU_TO_RAD * L_NOZZLE / df['Iyy']
df['u_max']          = df['max_gimbal_deg'] * 12.0/15.0
df['td_max']         = df['keff_full'] * df['u_max']      # = theta_ddot_max
df['authority_ratio']= df['max_gimbal_deg'] * df['motor_scale'] / df['Iyy']

# Log variants (for logistic regression)
df['log_td']         = np.log(df['td_max'])
df['log_keff']       = np.log(df['keff_full'])
df['log_wind']       = np.log(df['wind_strength'])
df['log_slew']       = np.log(df['servo_slew_deg_s'])
df['log_authority']  = np.log(df['authority_ratio'])

# Interaction variables
df['wind_x_keff']    = df['wind_strength'] * df['keff_full']
df['wind_x_td']      = df['wind_strength'] * df['td_max']
df['td_div_slew']    = df['td_max'] / df['servo_slew_deg_s']
df['keff_div_slew']  = df['keff_full'] / df['servo_slew_deg_s']
df['wind_div_keff']  = df['wind_strength'] / df['keff_full']
df['log_latency']    = np.log(df['latency_steps'].clip(lower=1))
df['td_x_latency']   = np.log(df['td_max']) + df['log_latency']  # log(td × lat)

# Binary FRAGILE label (exclude INFEASIBLE from FRAGILE group)
df['is_fragile'] = (df['regime_label'] == 'FRAGILE').astype(int)
# For training: use EASY+MARGINAL as negatives, FRAGILE as positives (exclude INFEASIBLE)
df_model = df[df['regime_label'].isin(['EASY','MARGINAL','FRAGILE'])].copy()

y = df_model['is_fragile'].values
print(f"Dataset: n={len(df_model)}, FRAGILE={y.sum()}, non-FRAGILE={len(y)-y.sum()}")

# ------------------------------------------------------------------ helpers
def cv_auc_1d(col, y, n_splits=5, seed=42):
    """Cross-validated AUC for a single feature (log-transformed)."""
    x = df_model[col].values.reshape(-1, 1)
    sc = StandardScaler()
    xs = sc.fit_transform(x)
    lr = LogisticRegression(max_iter=1000, C=0.5)
    cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=seed)
    aucs = cross_val_score(lr, xs, y, scoring='roc_auc', cv=cv)
    return aucs.mean(), aucs.std()

def cv_auc_2d(col1, col2, y, n_splits=5, seed=42):
    """Cross-validated AUC for two features."""
    X = df_model[[col1, col2]].values
    sc = StandardScaler()
    Xs = sc.fit_transform(X)
    lr = LogisticRegression(max_iter=1000, C=0.5)
    cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=seed)
    aucs = cross_val_score(lr, Xs, y, scoring='roc_auc', cv=cv)
    return aucs.mean(), aucs.std()

def full_auc_1d(col, y):
    """Non-CV AUC for quick ranking."""
    x = df_model[col].values
    lr = LogisticRegression(max_iter=1000, C=0.5)
    sc = StandardScaler()
    xs = sc.fit_transform(x.reshape(-1,1))
    lr.fit(xs, y)
    prob = lr.predict_proba(xs)[:,1]
    return roc_auc_score(y, prob)

# ------------------------------------------------------------------ 1. False-negative vs true-positive table
fragile_idx = df['is_fragile'] == 1
fn_mask = fragile_idx & (df['td_max'] < THRESHOLD_TD)
tp_mask = fragile_idx & (df['td_max'] >= THRESHOLD_TD)

fn_ids = set(df[fn_mask]['rocket_id'].values)
print(f"\n{'='*70}")
print(f"FALSE NEGATIVES (FRAGILE, td < {THRESHOLD_TD}):  n={fn_mask.sum()}")
print(f"TRUE  POSITIVES (FRAGILE, td >= {THRESHOLD_TD}): n={tp_mask.sum()}")
print(f"{'='*70}")

show_cols = ['rocket_id','td_max','keff_full','max_gimbal_deg','motor_scale','Iyy',
             'wind_strength','servo_slew_deg_s','best_Kp',
             'nominal_success_rate','over_success_rate','under_success_rate']

print("\n--- False Negatives ---")
for _, row in df[fn_mask].sort_values('td_max').iterrows():
    print(f"  {row['rocket_id']:6s}  td={row['td_max']:6.1f}  keff={row['keff_full']:5.2f}  "
          f"u_max={row['u_max']:5.2f}  mgimbal={row['max_gimbal_deg']:5.2f}  "
          f"wind={row['wind_strength']:.3f}  slew={row['servo_slew_deg_s']:5.1f}  "
          f"bestKp={row['best_Kp']:6.1f}  nomSR={row['nominal_success_rate']:.3f}  "
          f"overSR={row['over_success_rate']:.1f}  underSR={row['under_success_rate']:.1f}")

print("\n--- True Positives (sorted by td_max) ---")
for _, row in df[tp_mask].sort_values('td_max').iterrows():
    print(f"  {row['rocket_id']:6s}  td={row['td_max']:6.1f}  keff={row['keff_full']:5.2f}  "
          f"u_max={row['u_max']:5.2f}  mgimbal={row['max_gimbal_deg']:5.2f}  "
          f"wind={row['wind_strength']:.3f}  slew={row['servo_slew_deg_s']:5.1f}  "
          f"bestKp={row['best_Kp']:6.1f}  nomSR={row['nominal_success_rate']:.3f}  "
          f"overSR={row['over_success_rate']:.1f}  underSR={row['under_success_rate']:.1f}")

# ------------------------------------------------------------------ 2. AUC comparison: primary predictors
print(f"\n{'='*70}")
print("AUC COMPARISON: PRIMARY PREDICTORS (n=16 FRAGILE population)")
print(f"{'='*70}")

candidates_primary = [
    ('log_td',        'theta_ddot_max (log)'),
    ('log_keff',      'keff_full (log) — per-CU sensitivity, max_gimbal-independent'),
    ('log_authority', 'authority_ratio (log) = theta_ddot / 0.0628'),
]

for col, label in candidates_primary:
    mu, sig = cv_auc_1d(col, y)
    fa = full_auc_1d(col, y)
    print(f"  {label:<55s}  CV-AUC={mu:.3f} ± {sig:.3f}  Full-AUC={fa:.3f}")

# ------------------------------------------------------------------ 3. H1: servo_slew
print(f"\n{'='*70}")
print("H1: Low servo_slew causes FRAGILE at modest theta_ddot")
print(f"{'='*70}")

# Compare servo_slew between FN, TP, and EASY
easy_slew  = df[df['regime_label']=='EASY']['servo_slew_deg_s'].values
fn_slew    = df[fn_mask]['servo_slew_deg_s'].values
tp_slew    = df[tp_mask]['servo_slew_deg_s'].values

print(f"  EASY slew:  mean={easy_slew.mean():.1f}  std={easy_slew.std():.1f}")
print(f"  FN   slew:  {fn_slew} (mean={fn_slew.mean():.1f})")
print(f"  TP   slew:  {tp_slew} (mean={tp_slew.mean():.1f})")

# r(servo_slew, is_fragile) in the full dataset
r_slew = np.corrcoef(df_model['servo_slew_deg_s'], y)[0,1]
print(f"\n  r(servo_slew, FRAGILE) = {r_slew:+.4f}  (near-zero = H1 rejected)")

mu_slew, sig_slew = cv_auc_1d('log_slew', y)
fa_slew = full_auc_1d('log_slew', y)
print(f"  solo AUC(servo_slew):  CV={mu_slew:.3f} ± {sig_slew:.3f}  Full={fa_slew:.3f}")

# Does adding slew help theta_ddot_max?
for base_col, base_name in [('log_td','theta_ddot'), ('log_keff','keff_full')]:
    mu2, sig2 = cv_auc_2d(base_col, 'log_slew', y)
    mu1, _ = cv_auc_1d(base_col, y)
    delta = mu2 - mu1
    print(f"  AUC({base_name} + slew): CV={mu2:.3f} ± {sig2:.3f}  delta={delta:+.3f}")

# ------------------------------------------------------------------ 4. H2: wind_strength
print(f"\n{'='*70}")
print("H2: High wind_strength causes FRAGILE at modest theta_ddot")
print(f"{'='*70}")

easy_wind = df[df['regime_label']=='EASY']['wind_strength'].values
fn_wind   = df[fn_mask]['wind_strength'].values
tp_wind   = df[tp_mask]['wind_strength'].values

print(f"  EASY  wind: mean={easy_wind.mean():.3f}  std={easy_wind.std():.3f}")
print(f"  FN    wind: {[f'{w:.3f}' for w in sorted(fn_wind)]}  mean={fn_wind.mean():.3f}")
print(f"  TP    wind: mean={tp_wind.mean():.3f}  std={tp_wind.std():.3f}")

r_wind = np.corrcoef(df_model['wind_strength'], y)[0,1]
print(f"\n  r(wind_strength, FRAGILE) = {r_wind:+.4f}")

mu_wind, sig_wind = cv_auc_1d('log_wind', y)
fa_wind = full_auc_1d('log_wind', y)
print(f"  solo AUC(wind_strength):  CV={mu_wind:.3f} ± {sig_wind:.3f}  Full={fa_wind:.3f}")

for base_col, base_name in [('log_td','theta_ddot'), ('log_keff','keff_full')]:
    mu2, sig2 = cv_auc_2d(base_col, 'log_wind', y)
    mu1, _ = cv_auc_1d(base_col, y)
    delta = mu2 - mu1
    print(f"  AUC({base_name} + wind):  CV={mu2:.3f} ± {sig2:.3f}  delta={delta:+.3f}")

# ------------------------------------------------------------------ 5. H3: aerodynamic stability
print(f"\n{'='*70}")
print("H3: Aerodynamic instability modifies threshold")
print(f"{'='*70}")

fn_sm  = df[fn_mask]['static_margin'].values
tp_sm  = df[tp_mask]['static_margin'].values
easy_sm = df[df['regime_label']=='EASY']['static_margin'].values

print(f"  EASY  static_margin: mean={easy_sm.mean():.3f}  std={easy_sm.std():.3f}")
print(f"  FN    static_margin: {[f'{v:.3f}' for v in fn_sm]}  mean={fn_sm.mean():.3f}")
print(f"  TP    static_margin: mean={tp_sm.mean():.3f}  std={tp_sm.std():.3f}")

r_sm = np.corrcoef(df_model['static_margin'], y)[0,1]
print(f"\n  r(static_margin, FRAGILE) = {r_sm:+.4f}")

# AUC: does aero stability add to theta_ddot?
for base_col, base_name in [('log_td','theta_ddot'), ('log_keff','keff_full')]:
    mu2, sig2 = cv_auc_2d(base_col, 'static_margin', y)
    mu1, _ = cv_auc_1d(base_col, y)
    delta = mu2 - mu1
    print(f"  AUC({base_name} + static_margin):  CV={mu2:.3f} ± {sig2:.3f}  delta={delta:+.3f}")

# ------------------------------------------------------------------ 6. H4: interaction ratios
print(f"\n{'='*70}")
print("H4: Residual explained by interaction or ratio variable")
print(f"{'='*70}")

ratio_candidates = [
    ('wind_x_keff',   'wind_strength × keff_full  (loop-floor proxy)'),
    ('wind_x_td',     'wind_strength × theta_ddot'),
    ('td_div_slew',   'theta_ddot / servo_slew'),
    ('keff_div_slew', 'keff_full / servo_slew'),
    ('wind_div_keff', 'wind_strength / keff_full'),
]

print("  Solo AUC for ratio variables:")
for col, label in ratio_candidates:
    df_model[col] = df_model[col]  # already computed
    # use log for positive ratios
    df_model['_tmp'] = np.log(df_model[col] + 1e-8)
    mu, sig = cv_auc_1d('_tmp', y)
    fa = full_auc_1d('_tmp', y)
    print(f"    {label:<50s}  CV={mu:.3f} ± {sig:.3f}  Full={fa:.3f}")

print("\n  AUC improvement over theta_ddot adding each ratio:")
for col, label in ratio_candidates:
    df_model['_tmp'] = np.log(df_model[col] + 1e-8)
    mu2, sig2 = cv_auc_2d('log_td', '_tmp', y)
    mu1, _ = cv_auc_1d('log_td', y)
    delta = mu2 - mu1
    print(f"    {label:<50s}  delta={delta:+.3f}  CV={mu2:.3f} ± {sig2:.3f}")

print("\n  AUC improvement over keff_full adding each ratio:")
for col, label in ratio_candidates:
    df_model['_tmp'] = np.log(df_model[col] + 1e-8)
    mu2, sig2 = cv_auc_2d('log_keff', '_tmp', y)
    mu1, _ = cv_auc_1d('log_keff', y)
    delta = mu2 - mu1
    print(f"    {label:<50s}  delta={delta:+.3f}  CV={mu2:.3f} ± {sig2:.3f}")

# ------------------------------------------------------------------ 7. Best two-variable model
print(f"\n{'='*70}")
print("BEST TWO-VARIABLE MODELS (exhaustive search)")
print(f"{'='*70}")

all_features = ['log_td', 'log_keff', 'log_wind', 'log_slew',
                'wind_x_keff', 'wind_x_td', 'td_div_slew', 'keff_div_slew',
                'static_margin', 'log_authority', 'log_latency', 'td_x_latency']

# Compute log versions for ratio columns
for col in all_features:
    if col not in df_model.columns:
        # try log of raw
        raw_col = col.replace('log_','')
        if raw_col in df_model.columns:
            df_model[col] = np.log(df_model[raw_col].clip(lower=1e-9))

best_results = []
baseline_mu, _ = cv_auc_1d('log_td', y)
print(f"  Baseline (theta_ddot alone): CV-AUC = {baseline_mu:.4f}")
print()

for i, f1 in enumerate(all_features):
    for j, f2 in enumerate(all_features):
        if j <= i: continue
        try:
            mu2, sig2 = cv_auc_2d(f1, f2, y)
            best_results.append((mu2, sig2, f1, f2))
        except Exception:
            pass

best_results.sort(reverse=True)
print("  Top-10 two-variable models:")
for k, (mu, sig, f1, f2) in enumerate(best_results[:10]):
    delta = mu - baseline_mu
    print(f"    [{k+1:2d}] {f1:<25s} + {f2:<25s}  CV={mu:.4f} ± {sig:.4f}  delta={delta:+.4f}")

# ------------------------------------------------------------------ 8. Physical interpretation of false negatives
print(f"\n{'='*70}")
print("PHYSICAL INTERPRETATION: WHY keff_full vs theta_ddot_max")
print(f"{'='*70}")

# Key insight: for a given keff_full, varying max_gimbal changes theta_ddot but not per-unit sensitivity
print()
print("  theta_ddot_max = keff_full × u_max = keff_full × (max_gimbal_deg × 12/15)")
print("  keff_full = T × motor_scale × CU_TO_RAD × l / Iyy  (INDEPENDENT of max_gimbal)")
print()
print("  For ceiling-limited FRAGILE (over_sr fails at doubled Kp):")
print("  The proportional term = Kp × keff_full × error  [not capped by u_max if error is small]")
print("  High keff_full → proportional gain causes oscillation at lower Kp → ceiling falls")
print("  Small max_gimbal → low theta_ddot_max but SAME keff_full → STILL ceiling-limited")
print()

print("  keff_full statistics by label:")
for label in ['EASY','MARGINAL','FRAGILE']:
    sub = df[df['regime_label']==label]['keff_full']
    print(f"    {label:8s}: mean={sub.mean():.2f}  median={sub.median():.2f}  "
          f"std={sub.std():.2f}  range=[{sub.min():.2f}, {sub.max():.2f}]  n={len(sub)}")

print()
print("  Comparison: FN vs TP keff_full:")
print(f"    FN keff_full = {df[fn_mask]['keff_full'].values.tolist()} (mean={df[fn_mask]['keff_full'].mean():.2f})")
print(f"    TP keff_full = {df[tp_mask]['keff_full'].values.tolist()}")
print(f"    TP mean keff = {df[tp_mask]['keff_full'].mean():.2f}")

# ------------------------------------------------------------------ 9. max_gimbal as a view of the residual
print(f"\n{'='*70}")
print("max_gimbal ANALYSIS — why small-gimbal designs are false negatives")
print(f"{'='*70}")

print()
print("  max_gimbal_deg by FRAGILE sub-type:")
print(f"    FN (td<70):  {sorted(df[fn_mask]['max_gimbal_deg'].values.tolist())}")
print(f"    TP (td>=70): {sorted(df[tp_mask]['max_gimbal_deg'].values.tolist())}")
print(f"    EASY:        mean={df[df['regime_label']=='EASY']['max_gimbal_deg'].mean():.2f}  "
      f"range=[{df[df['regime_label']=='EASY']['max_gimbal_deg'].min():.2f}, "
      f"{df[df['regime_label']=='EASY']['max_gimbal_deg'].max():.2f}]")
print()
print("  EASY designs with keff_full > 7.0 (compare to R0804 keff=7.29):")
easy_hi_keff = df[(df['regime_label']=='EASY') & (df['keff_full'] > 7.0)]
print(f"    n={len(easy_hi_keff)},  median wind={easy_hi_keff['wind_strength'].median():.3f}  "
      f"vs FN wind mean={df[fn_mask]['wind_strength'].mean():.3f}")

# What separates EASY (keff>7) from FN designs?
print()
print("  EASY vs FN: key differences at similar keff_full (both groups keff > 6.5):")
fn_keff_ok = df[fn_mask & (df['keff_full'] > 6.5)]
easy_keff_ok = df[(df['regime_label']=='EASY') & (df['keff_full'] > 6.5) & (df['keff_full'] < 10)]
for col in ['wind_strength','servo_slew_deg_s','td_max','max_gimbal_deg','Iyy']:
    fn_v = fn_keff_ok[col].mean()
    easy_v = easy_keff_ok[col].mean()
    print(f"    {col:<25s}: FN={fn_v:.3f}  EASY(keff>6.5,<10)={easy_v:.3f}  ratio={fn_v/easy_v:.2f}")

# ------------------------------------------------------------------ 10. Decision rule
print(f"\n{'='*70}")
print("DECISION RULE RECOMMENDATION")
print(f"{'='*70}")

auc_td, std_td   = cv_auc_1d('log_td', y)
auc_ke, std_ke   = cv_auc_1d('log_keff', y)
print(f"\n  AUC(theta_ddot_max alone): {auc_td:.4f} ± {std_td:.4f}")
print(f"  AUC(keff_full alone):       {auc_ke:.4f} ± {std_ke:.4f}")

# Best 2-var with td as base
top_with_td = [(mu,sig,f1,f2) for mu,sig,f1,f2 in best_results if 'log_td' in (f1,f2)]
if top_with_td:
    best_mu, best_sig, bf1, bf2 = top_with_td[0]
    other = bf2 if bf1=='log_td' else bf1
    delta = best_mu - auc_td
    print(f"\n  Best 2-var with theta_ddot: {bf1} + {bf2} = {best_mu:.4f} ± {best_sig:.4f} (delta={delta:+.4f})")

# Best 2-var with keff as base
top_with_ke = [(mu,sig,f1,f2) for mu,sig,f1,f2 in best_results if 'log_keff' in (f1,f2)]
if top_with_ke:
    best_mu, best_sig, bf1, bf2 = top_with_ke[0]
    delta = best_mu - auc_ke
    print(f"  Best 2-var with keff_full: {bf1} + {bf2} = {best_mu:.4f} ± {best_sig:.4f} (delta={delta:+.4f})")

print("\n  Rule: if max additional AUC improvement < 0.03 → keep single predictor")
print("        if improvement >= 0.03 → add second variable")

print(f"\n{'='*70}")
print("SUMMARY OF FINDINGS")
print(f"{'='*70}")
print()
