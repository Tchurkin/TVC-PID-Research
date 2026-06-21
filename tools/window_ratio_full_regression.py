"""
tools/window_ratio_full_regression.py  --  Exhaustive regression on all available design
parameters to measure how much unexplained variance in window_ratio is stochastic vs
a missing predictor.

Baseline: log(window) ~ log(keff) + log(latency)  [CV R² ≈ 0.616]
This script:
  1. Adds every available design parameter one at a time → delta_CV table
  2. Runs a full-model OLS with all factors → CV R²
  3. Checks residuals for correlation with parameter values → any non-linear effects?
  4. Estimates how much variance is likely irreducible stochastic noise vs missing physics

Available predictors in window_ratio_v2_py.csv:
  keff, u_max, latency_steps, Iyy, motor_scale, max_gimbal_deg,
  wind_strength, servo_slew, static_margin, opt_Kd
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import KFold, cross_val_score
from sklearn.preprocessing import StandardScaler
from scipy import stats

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

# ── Load data ─────────────────────────────────────────────────────────────────
df = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')

# Filter: non-censored only (floor and ceiling both detected within tested range)
df_valid = df[~df['floor_censored'] & ~df['ceil_censored']].copy()
# Drop rows where window_ratio is NaN or <= 0 (bad sweep rows despite non-censored flag)
df_valid = df_valid[df_valid['window_ratio'].notna() & (df_valid['window_ratio'] > 0)].copy()
print(f"Total designs: {len(df)}, Non-censored with valid window: {len(df_valid)}")
print(f"window_ratio range: {df_valid['window_ratio'].min():.1f} - {df_valid['window_ratio'].max():.0f}")
print()

# Log-transform all continuous predictors
df_valid['log_window']  = np.log(df_valid['window_ratio'])
df_valid['log_keff']    = np.log(df_valid['keff'])
df_valid['log_lat']     = np.log(df_valid['latency_steps'])
df_valid['log_iyy']     = np.log(df_valid['Iyy'])
df_valid['log_motor']   = np.log(df_valid['motor_scale'])
df_valid['log_gimbal']  = np.log(df_valid['max_gimbal_deg'])
df_valid['log_umax']    = np.log(df_valid['u_max'])
df_valid['log_wind']    = np.log(df_valid['wind_strength'])
df_valid['log_slew']    = np.log(df_valid['servo_slew'])
df_valid['log_kd']      = np.log(df_valid['opt_Kd'].clip(lower=0.5))
df_valid['static_margin'] = df_valid['static_margin']  # already signed, keep linear
df_valid['log_pi']      = df_valid['log_keff'] + 2 * df_valid['log_lat']   # Pi = keff * lat^2

y = df_valid['log_window'].values

kf = KFold(n_splits=5, shuffle=True, random_state=42)

def cv_r2(X):
    """5-fold CV R² for OLS on X → y."""
    model = LinearRegression()
    scores = cross_val_score(model, X, y, cv=kf, scoring='r2')
    return scores.mean(), scores.std()


def ols_r2(X):
    from sklearn.metrics import r2_score
    model = LinearRegression().fit(X, y)
    return r2_score(y, model.predict(X))


# ── Baseline model ─────────────────────────────────────────────────────────────
X_base = df_valid[['log_keff', 'log_lat']].values
r2_base, std_base = cv_r2(X_base)
r2_base_insample = ols_r2(X_base)
print("="*65)
print("BASELINE: log(window) ~ log(keff) + log(latency)")
print(f"  CV R² = {r2_base:.3f} ± {std_base:.3f}   |   In-sample R² = {r2_base_insample:.3f}")
print("="*65)
print()

# ── Single-variable additions to baseline ─────────────────────────────────────
extras = {
    'log_umax':      ('log(u_max = max_gimbal×12/15)',  'theory: should be ~0'),
    'log_gimbal':    ('log(max_gimbal_deg)',             'same as u_max (linear transform)'),
    'log_iyy':       ('log(Iyy)',                        'not in keff formula'),
    'log_motor':     ('log(motor_scale)',                'partially in keff'),
    'log_wind':      ('log(wind_strength)',              'affects floor magnitude'),
    'log_slew':      ('log(servo_slew)',                 'servo speed limit'),
    'log_kd':        ('log(opt_Kd)',                     'Kd choice; fixed during Kp sweep'),
    'static_margin': ('static_margin',                   'aero stability (signed)'),
    'log_pi':        ('log(Pi=keff*lat^2)',              'single-param baseline comparison'),
}

print(f"{'Predictor':<20}  {'Description':<40}  {'CV delta':>8}  {'CV R²':>7}")
print("-"*85)

results = []
for col, (desc, note) in extras.items():
    X_aug = df_valid[['log_keff', 'log_lat', col]].values
    r2_aug, std_aug = cv_r2(X_aug)
    delta = r2_aug - r2_base
    results.append((col, desc, r2_aug, std_aug, delta))
    flag = ' ***' if delta > 0.03 else (' **' if delta > 0.01 else ('  *' if delta > 0.005 else ''))
    print(f"  {col:<18}  {desc:<40}  {delta:+8.4f}  {r2_aug:7.3f}{flag}")

print()
print("  [***] > 0.03 delta: strong addition; [**] > 0.01; [*] > 0.005")

# ── Pi single-param vs keff+lat two-param ─────────────────────────────────────
print()
print("Pi single-param comparison:")
X_pi = df_valid[['log_pi']].values
r2_pi, std_pi = cv_r2(X_pi)
print(f"  Pi = keff*lat^2 single:  CV R² = {r2_pi:.3f} ± {std_pi:.3f}  (baseline: {r2_base:.3f})")

# ── Full model (all available predictors) ──────────────────────────────────────
all_cols = ['log_keff', 'log_lat', 'log_umax', 'log_iyy', 'log_motor',
            'log_wind', 'log_slew', 'log_kd', 'static_margin']
X_full = df_valid[all_cols].values
r2_full, std_full = cv_r2(X_full)
r2_full_insample  = ols_r2(X_full)
print()
print(f"FULL MODEL (all {len(all_cols)} predictors):")
print(f"  CV R² = {r2_full:.3f} ± {std_full:.3f}   |   In-sample R² = {r2_full_insample:.3f}")
print(f"  Overfitting gap: {r2_full_insample - r2_full:.3f}")

# ── Residual analysis ─────────────────────────────────────────────────────────
from sklearn.linear_model import LinearRegression
model_base = LinearRegression().fit(X_base, y)
residuals  = y - model_base.predict(X_base)

print()
print("="*65)
print("RESIDUAL CORRELATION ANALYSIS (vs baseline log(keff)+log(lat))")
print("="*65)
res_cols = {
    'log_umax':      'log(u_max)',
    'log_iyy':       'log(Iyy)',
    'log_wind':      'log(wind_strength)',
    'log_slew':      'log(servo_slew)',
    'log_kd':        'log(opt_Kd)',
    'static_margin': 'static_margin',
    'log_motor':     'log(motor_scale)',
}
for col, name in res_cols.items():
    r, p = stats.pearsonr(df_valid[col].values, residuals)
    rho, p_s = stats.spearmanr(df_valid[col].values, residuals)
    flag = ' *' if abs(r) > 0.15 and p < 0.10 else ''
    print(f"  {name:<20}  r={r:+.3f}  p={p:.2f}  rho={rho:+.3f}  p_s={p_s:.2f}{flag}")

# ── Variance decomposition ─────────────────────────────────────────────────────
total_var    = np.var(y)
explained    = r2_base * total_var
unexplained  = total_var - explained
print()
print("="*65)
print("VARIANCE DECOMPOSITION")
print("="*65)
print(f"  Total log(window) variance:    {total_var:.4f}")
print(f"  Explained by keff+lat (R²=CV): {r2_base:.3f} × {total_var:.4f} = {r2_base*total_var:.4f}")
print(f"  Unexplained residual:          {(1-r2_base):.3f} × {total_var:.4f} = {(1-r2_base)*total_var:.4f}")
print()

# Estimate noise floor from seed-to-seed variation on same design
# If we have multiple designs at same (keff, lat) group → within-group variance = noise
# Proxy: variance of residuals for designs with similar keff and lat
df_valid['keff_q'] = pd.qcut(df_valid['log_keff'], 3, labels=False)
df_valid['lat_q']  = pd.qcut(df_valid['log_lat'],  2, labels=False)
within_var = df_valid.groupby(['keff_q','lat_q'])['log_window'].var().mean()
print(f"  Within-cell variance (same keff × lat tier): {within_var:.4f}")
print(f"  (proxy for irreducible stochastic noise + unmodeled design variation)")
print()
print("INTERPRETATION:")
print(f"  Baseline CV R2 = {r2_base:.3f} -> {(1-r2_base)*100:.1f}% unexplained")
full_add = r2_full - r2_base
print(f"  Adding all {len(all_cols)-2} extra factors: delta CV R² = +{full_add:.3f}")
if full_add < 0.05:
    print("  CONCLUSION: The unexplained variance is largely STOCHASTIC (seed noise,")
    print("  per-design variation from unmodeled parameters like backlash/deadband).")
    print("  No single available design factor meaningfully improves prediction beyond keff+lat.")
else:
    sig = [r for r in results if r[4] > 0.03]
    print(f"  CONCLUSION: {len(sig)} factor(s) add >0.03 CV R²: {[r[0] for r in sig]}")
    print("  These represent genuine missing predictors worth investigating.")
