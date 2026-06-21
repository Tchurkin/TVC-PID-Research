"""
tools/pi_parameter_test.py — Test universal dimensionless parameter Pi = keff * lat^alpha.

Motivation: the pooled n=262 regression has coefficients
  log(over_sr) ~ -0.051*log(td) + -0.101*log(latency)
The 2:1 ratio suggests latency enters as tau^2 rather than tau.
The window_ratio regression has coefficients
  log(window) ~ -1.19*log(keff) - 1.88*log(latency)
implying Pi_window = keff * lat^(1.88/1.19) = keff * lat^1.58.

This script tests: for each candidate exponent alpha in {1.0, 1.58, 1.88, 2.0},
fit a SINGLE-variable model log(outcome) ~ beta * log(keff * lat^alpha) + const
and compare 5-fold CV R^2 to the two-variable baseline.

Done on:
  A) window_ratio_v2_py.csv  (n=120, CV R^2 target ~0.616)
  B) regression_pooled_py.csv  (n=262, over_sr, CV R^2 target ~0.325)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.linear_model import Ridge
from sklearn.model_selection import cross_val_score

RESULTS = Path('experiments/results')

ALPHAS = [0.5, 1.0, 1.5, 1.58, 1.88, 2.0, 2.5]


def cv_r2(X, y, n_splits=5, seed=42):
    """5-fold CV R^2 with Ridge(alpha=1e-6) (essentially OLS)."""
    from sklearn.model_selection import KFold
    from sklearn.metrics import r2_score
    kf = KFold(n_splits=n_splits, shuffle=True, random_state=seed)
    scores = []
    for tr, te in kf.split(X):
        X_tr, y_tr = X[tr], y[tr]
        X_te, y_te = X[te], y[te]
        b = np.linalg.lstsq(X_tr, y_tr, rcond=None)[0]
        pred = X_te @ b
        ss_res = np.sum((y_te - pred) ** 2)
        ss_tot = np.sum((y_te - y_te.mean()) ** 2)
        scores.append(1 - ss_res / ss_tot if ss_tot > 0 else 0.0)
    return np.mean(scores), np.std(scores)


def test_on_window_ratio():
    print("=" * 60)
    print("A) window_ratio_v2_py.csv  (n=120, log window_ratio ~ log keff + log lat)")
    print("=" * 60)
    df = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')
    # Use non-censored rows only (both floor and ceiling measured)
    df = df[(df['floor_censored'] == 0) & (df['ceil_censored'] == 0)].copy()
    df = df.dropna(subset=['window_ratio', 'keff'])
    print(f"n non-censored (valid): {len(df)}")

    log_wr  = np.log(df['window_ratio'].clip(lower=0.1))
    log_k   = np.log(df['keff'].clip(lower=0.1))
    log_lat = np.log(df['latency_steps'].astype(float))

    # Baseline: two-variable (keff + lat)
    X_base = np.column_stack([log_k, log_lat, np.ones(len(df))])
    cv_mean, cv_std = cv_r2(X_base, log_wr.values)
    print(f"\nBaseline (keff + lat):         CV R2 = {cv_mean:.3f} +/- {cv_std:.3f}")

    # In-sample
    b = np.linalg.lstsq(X_base, log_wr.values, rcond=None)[0]
    p = X_base @ b
    r2_is = 1 - np.sum((log_wr.values - p)**2) / np.sum((log_wr.values - log_wr.mean())**2)
    print(f"Baseline (keff + lat):       in-sample R2 = {r2_is:.3f}")
    print(f"  coefs: keff={b[0]:.3f}  lat={b[1]:.3f}  intercept={b[2]:.3f}")

    print(f"\nSingle-parameter Pi = keff * lat^alpha:")
    print(f"  {'alpha':>6}  {'in-samp R2':>12}  {'CV R2':>8}  {'CV std':>8}  {'coef':>8}")
    for alpha in ALPHAS:
        log_pi = log_k + alpha * log_lat
        X = np.column_stack([log_pi, np.ones(len(df))])
        cv_m, cv_s = cv_r2(X, log_wr.values)
        b2 = np.linalg.lstsq(X, log_wr.values, rcond=None)[0]
        p2 = X @ b2
        r2 = 1 - np.sum((log_wr.values - p2)**2) / np.sum((log_wr.values - log_wr.mean())**2)
        flag = "  <-- optimal?" if abs(cv_m - max(cv_r2(np.column_stack([log_k + a*log_lat, np.ones(len(df))]), log_wr.values)[0] for a in ALPHAS)) < 0.002 else ""
        print(f"  {alpha:>6.2f}  {r2:>12.3f}  {cv_m:>8.3f}  {cv_s:>8.3f}  {b2[0]:>8.3f}{flag}")


def test_on_over_sr():
    print("\n" + "=" * 60)
    print("B) regression_pooled_py.csv  (n=262, over_sr ~ log td + log lat)")
    print("=" * 60)
    df = pd.read_csv(RESULTS / 'regression_pooled_py.csv')
    df = df.dropna(subset=['td', 'latency_steps', 'over_sr'])
    print(f"n: {len(df)}")

    # over_sr is NOT log-transformed (it's a probability in [0,1])
    # use it directly; log-transform predictors
    y = df['over_sr'].values

    # Need keff to form Pi — compute from design params
    F15_AVG = 14.4
    L_NOZ   = 0.25
    CU_RAD  = np.pi / 180 * 15.0 / 12.0

    # keff is not in this CSV — td is. td = keff * u_max = keff * (max_gimbal*12/15)
    # We don't have individual max_gimbal in the pooled CSV to separate keff from u_max.
    # Use td directly as the authority predictor instead.
    log_td  = np.log(df['td'].clip(lower=1.0))
    log_lat = np.log(df['latency_steps'].astype(float))

    X_base = np.column_stack([log_td, log_lat, np.ones(len(df))])
    cv_mean, cv_std = cv_r2(X_base, y)
    b = np.linalg.lstsq(X_base, y, rcond=None)[0]
    p = X_base @ b
    r2_is = 1 - np.sum((y - p)**2) / np.sum((y - y.mean())**2)
    print(f"\nBaseline (log_td + log_lat):   CV R2 = {cv_mean:.3f} +/- {cv_std:.3f}  in-samp={r2_is:.3f}")
    print(f"  coefs: log_td={b[0]:.4f}  log_lat={b[1]:.4f}  intercept={b[2]:.4f}")
    print(f"  lat/td ratio: {abs(b[1]/b[0]):.2f} (2.0 would mean lat enters as tau^2)")

    print(f"\nSingle-parameter Pi = td * lat^alpha (treating td as authority proxy):")
    print(f"  {'alpha':>6}  {'in-samp R2':>12}  {'CV R2':>8}  {'CV std':>8}  {'coef':>8}")
    best_cv = -1
    for alpha in ALPHAS:
        log_pi = log_td + alpha * log_lat
        X = np.column_stack([log_pi, np.ones(len(df))])
        cv_m, cv_s = cv_r2(X, y)
        if cv_m > best_cv:
            best_cv = cv_m
        b2 = np.linalg.lstsq(X, y, rcond=None)[0]
        p2 = X @ b2
        r2 = 1 - np.sum((y - p2)**2) / np.sum((y - y.mean())**2)
        print(f"  {alpha:>6.2f}  {r2:>12.3f}  {cv_m:>8.3f}  {cv_s:>8.3f}  {b2[0]:>8.4f}")

    # Optimal alpha by grid search
    alphas_fine = np.linspace(0.5, 3.0, 50)
    cv_vals = []
    for alpha in alphas_fine:
        log_pi = log_td + alpha * log_lat
        X = np.column_stack([log_pi, np.ones(len(df))])
        cv_m, _ = cv_r2(X, y)
        cv_vals.append(cv_m)
    opt_alpha = alphas_fine[np.argmax(cv_vals)]
    print(f"\nOptimal alpha (grid search): {opt_alpha:.2f}  CV R2 = {max(cv_vals):.3f}")
    print(f"(Compare baseline CV R2 = {cv_mean:.3f})")


def main():
    test_on_window_ratio()
    test_on_over_sr()

    print("\n=== INTERPRETATION ===")
    print("If a single-parameter Pi beats the two-parameter baseline in CV R2,")
    print("it implies the two predictors (authority, latency) combine multiplicatively")
    print("with the found exponent -- a genuine dimensionless design number.")
    print("If single-param is worse, the two-variable form is not reducible.")


if __name__ == '__main__':
    main()
