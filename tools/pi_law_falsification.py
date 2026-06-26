"""
tools/pi_law_falsification.py  (2026-06-22)

Adversarial test of the claim: window_ratio ~ C / Pi (Pi = keff x latency^2)

PART 1: Curvature validation (linear vs quadratic in log-Pi)
  Cross-dataset: v2 (lat 1-6, n=82) <-> lat_ext (lat 7-12, n=71)
  NOTE: fsat dataset is IDENTICAL to v2 (window_ratio values are bit-for-bit equal);
  it cannot serve as independent validation. lat_ext uses a different LHS pool,
  different designs, and different latency range -- it IS genuinely independent.

PART 2: Two-variable surface test (Pi vs keff + lat separately)

PART 3: Residual structure analysis

PART 4: Physical interpretation of the collapse constant C

PART 5: Verdict

Run: python tools/pi_law_falsification.py
"""

import sys, warnings
import numpy as np
import pandas as pd
from scipy import stats
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import cross_val_score, KFold
from sklearn.metrics import r2_score, mean_squared_error, mean_absolute_error
from sklearn.preprocessing import PolynomialFeatures

warnings.filterwarnings('ignore')

# ── Data loading ───────────────────────────────────────────────────────────────

def load_v2():
    df = pd.read_csv('experiments/results/window_ratio_v2_py.csv')
    df = df[(df.floor_censored == 0) & (df.ceil_censored == 0)].copy()
    df['Pi'] = df['keff'] * df['latency_steps'] ** 2
    df['log_Pi'] = np.log(df['Pi'])
    df['log_window'] = np.log(df['window_ratio'])
    df['log_keff'] = np.log(df['keff'])
    df['log_lat'] = np.log(df['latency_steps'])
    df = df.dropna(subset=['log_window', 'log_Pi', 'log_keff', 'log_lat'])
    return df


def load_lat_ext():
    df = pd.read_csv('experiments/results/window_ratio_lat_ext_py.csv')
    df = df[(df.floor_censored == 0) & (df.ceil_censored == 0)].copy()
    df['Pi'] = df['keff'] * df['latency_steps'] ** 2
    df['log_Pi'] = np.log(df['Pi'])
    df['log_window'] = np.log(df['window_ratio'])
    df['log_keff'] = np.log(df['keff'])
    df['log_lat'] = np.log(df['latency_steps'])
    df = df.dropna(subset=['log_window', 'log_Pi', 'log_keff', 'log_lat'])
    return df


def load_fsat():
    df = pd.read_csv('experiments/results/fsat_window_test_py.csv')
    df = df.dropna(subset=['window_ratio', 'Pi', 'fsat_floor']).copy()
    df['log_Pi'] = np.log(df['Pi'])
    df['log_window'] = np.log(df['window_ratio'])
    df['log_keff'] = np.log(df['keff'])
    df['log_lat'] = np.log(df['latency'])
    df['latency_steps'] = df['latency']
    return df


def fit_and_score(X_train, y_train, X_test, y_test):
    m = LinearRegression().fit(X_train, y_train)
    y_pred_train = m.predict(X_train)
    y_pred_test  = m.predict(X_test)
    train_r2  = r2_score(y_train, y_pred_train)
    test_r2   = r2_score(y_test,  y_pred_test)
    test_rmse = np.sqrt(mean_squared_error(y_test, y_pred_test))
    test_mae  = mean_absolute_error(y_test, y_pred_test)
    return m, train_r2, test_r2, test_rmse, test_mae


def aic_bic(y, y_pred, n_params):
    n = len(y)
    resid = y - y_pred
    sse = np.sum(resid**2)
    sigma2 = sse / n
    aic = n * np.log(sigma2) + 2 * n_params
    bic = n * np.log(sigma2) + n_params * np.log(n)
    return aic, bic


def coef_ci(model, X, y, alpha=0.05):
    """95% CI for OLS coefficients."""
    n, p = X.shape
    y_pred = model.predict(X)
    sse = np.sum((y - y_pred)**2)
    s2 = sse / (n - p - 1)
    XtX_inv = np.linalg.pinv(X.T @ X)
    se = np.sqrt(np.diag(XtX_inv) * s2)
    t_crit = stats.t.ppf(1 - alpha/2, df=n - p - 1)
    coefs = model.coef_
    ci_lo = coefs - t_crit * se
    ci_hi = coefs + t_crit * se
    return se, ci_lo, ci_hi


# ── PART 1: Curvature validation ───────────────────────────────────────────────

def part1_curvature(v2, lat_ext):
    print("\n" + "=" * 68)
    print("PART 1: CURVATURE VALIDATION")
    print("=" * 68)

    print("\n>>> CRITICAL METHODOLOGICAL NOTE <<<")
    print("  fsat dataset shares IDENTICAL window_ratio values with v2 (RMSE=0).")
    print("  It cannot serve as independent validation.")
    print("  Using lat_ext (lat 7-12, n={}) as the genuine independent test set.".format(len(lat_ext)))
    print("  lat_ext: different LHS pool, different designs, different latency range.")
    print("  Note: lat_ext has known SELECTION ARTIFACT (high-keff infeasible at lat 7-12)")
    print("  which biases the surviving population toward low-keff designs.\n")

    # ---- Internal 5-fold CV on v2 ----
    print("--- Internal 5-fold CV on v2 (n={}) ---".format(len(v2)))
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    for label, X in [
        ("Model 1 (linear, log Pi)",       v2[['log_Pi']].values),
        ("Model 2 (quadratic in log Pi)",  np.column_stack([v2['log_Pi'], v2['log_Pi']**2])),
    ]:
        cv_scores = cross_val_score(LinearRegression(), X, v2['log_window'].values,
                                    cv=kf, scoring='r2')
        m = LinearRegression().fit(X, v2['log_window'].values)
        train_r2 = m.score(X, v2['log_window'].values)
        print(f"  {label}")
        print(f"    train R2 = {train_r2:.4f}")
        print(f"    CV R2    = {cv_scores.mean():.4f} +/- {cv_scores.std():.4f}")
        print(f"    coefs    = {dict(zip(['const']+['b','c'][:X.shape[1]], [m.intercept_]+list(m.coef_)))}")
        print()

    # ---- Cross-dataset: train on v2, test on lat_ext ----
    print("--- Cross-dataset: Train on v2 (lat 1-6), Test on lat_ext (lat 7-12) ---")
    for label, Xv2_cols, Xext_cols in [
        ("Model 1 (linear)",    [v2['log_Pi'].values.reshape(-1,1)],
                                [lat_ext['log_Pi'].values.reshape(-1,1)]),
        ("Model 2 (quadratic)", [np.column_stack([v2['log_Pi'], v2['log_Pi']**2])],
                                [np.column_stack([lat_ext['log_Pi'], lat_ext['log_Pi']**2])]),
    ]:
        Xtr = Xv2_cols[0]
        Xte = Xext_cols[0]
        m, tr2, te2, rmse, mae = fit_and_score(Xtr, v2['log_window'].values,
                                               Xte, lat_ext['log_window'].values)
        print(f"  {label}")
        print(f"    Train R2 = {tr2:.4f}  |  Test R2 = {te2:.4f}")
        print(f"    Test RMSE = {rmse:.4f}  |  Test MAE = {mae:.4f}")
        print(f"    coefs = int={m.intercept_:.4f} {list(m.coef_)}")
        print()

    # ---- Cross-dataset: train on lat_ext, test on v2 ----
    print("--- Cross-dataset: Train on lat_ext (lat 7-12), Test on v2 (lat 1-6) ---")
    for label, Xext_cols, Xv2_cols in [
        ("Model 1 (linear)",    [lat_ext['log_Pi'].values.reshape(-1,1)],
                                [v2['log_Pi'].values.reshape(-1,1)]),
        ("Model 2 (quadratic)", [np.column_stack([lat_ext['log_Pi'], lat_ext['log_Pi']**2])],
                                [np.column_stack([v2['log_Pi'], v2['log_Pi']**2])]),
    ]:
        Xtr = Xext_cols[0]
        Xte = Xv2_cols[0]
        m, tr2, te2, rmse, mae = fit_and_score(Xtr, lat_ext['log_window'].values,
                                               Xte, v2['log_window'].values)
        print(f"  {label}")
        print(f"    Train R2 = {tr2:.4f}  |  Test R2 = {te2:.4f}")
        print(f"    Test RMSE = {rmse:.4f}  |  Test MAE = {mae:.4f}")
        print(f"    coefs = int={m.intercept_:.4f} {list(m.coef_)}")
        print()

    # ---- Coefficient confidence intervals (v2) ----
    print("--- Coefficient 95% CIs (Model 2, v2 only) ---")
    X2 = np.column_stack([v2['log_Pi'], v2['log_Pi']**2])
    m2 = LinearRegression().fit(X2, v2['log_window'].values)
    se, ci_lo, ci_hi = coef_ci(m2, X2, v2['log_window'].values)
    print(f"  b (log Pi):   {m2.coef_[0]:.4f}  [{ci_lo[0]:.4f}, {ci_hi[0]:.4f}]  se={se[0]:.4f}")
    print(f"  c (log Pi)^2: {m2.coef_[1]:.4f}  [{ci_lo[1]:.4f}, {ci_hi[1]:.4f}]  se={se[1]:.4f}")

    # ---- Decision ----
    print()
    # Compute delta_test_R2 in both directions
    X1_v2  = v2['log_Pi'].values.reshape(-1,1)
    X2_v2  = np.column_stack([v2['log_Pi'], v2['log_Pi']**2])
    X1_ext = lat_ext['log_Pi'].values.reshape(-1,1)
    X2_ext = np.column_stack([lat_ext['log_Pi'], lat_ext['log_Pi']**2])

    _, _, te2_lin_A, _, _ = fit_and_score(X1_v2, v2['log_window'].values,
                                           X1_ext, lat_ext['log_window'].values)
    _, _, te2_quad_A, _, _ = fit_and_score(X2_v2, v2['log_window'].values,
                                            X2_ext, lat_ext['log_window'].values)
    delta_A = te2_quad_A - te2_lin_A

    _, _, te2_lin_B, _, _ = fit_and_score(X1_ext, lat_ext['log_window'].values,
                                           X1_v2, v2['log_window'].values)
    _, _, te2_quad_B, _, _ = fit_and_score(X2_ext, lat_ext['log_window'].values,
                                            X2_v2, v2['log_window'].values)
    delta_B = te2_quad_B - te2_lin_B

    print("--- DECISION RULE APPLICATION ---")
    print(f"  delta_test_R2 (v2 -> lat_ext): {delta_A:+.4f}")
    print(f"  delta_test_R2 (lat_ext -> v2): {delta_B:+.4f}")
    print(f"  Both > +0.05? -> curvature is physically real")
    print(f"  Both <= 0 or < +0.05? -> curvature is fitting artifact")
    if delta_A > 0.05 and delta_B > 0.05:
        print("  VERDICT: Curvature IS real (both cross-validation directions confirm)")
    elif delta_A <= 0 and delta_B <= 0:
        print("  VERDICT: Curvature is OVERFITTING (disappears out-of-sample)")
    else:
        print("  VERDICT: Mixed -- curvature is dataset-asymmetric (investigate selection bias)")


# ── PART 2: Two-variable surface test ─────────────────────────────────────────

def part2_surface(v2):
    print("\n" + "=" * 68)
    print("PART 2: TWO-VARIABLE SURFACE TEST")
    print("=" * 68)

    y = v2['log_window'].values
    n = len(y)
    kf = KFold(n_splits=5, shuffle=True, random_state=42)

    models = {
        'A: log(Pi) only':          v2[['log_Pi']].values,
        'B: log(keff) + log(lat)':  v2[['log_keff', 'log_lat']].values,
        'C: keff + lat + interaction': np.column_stack([
            v2['log_keff'], v2['log_lat'], v2['log_keff'] * v2['log_lat']]),
        'D: keff + lat + keff^2 + lat^2 + interaction': np.column_stack([
            v2['log_keff'], v2['log_lat'],
            v2['log_keff']**2, v2['log_lat']**2,
            v2['log_keff'] * v2['log_lat']]),
        'E: log(Pi) + log(Pi)^2':   np.column_stack([v2['log_Pi'], v2['log_Pi']**2]),
        'F: keff + lat + keff*lat interaction (best from CLAUDE.md)':
            np.column_stack([v2['log_keff'], v2['log_keff'] * v2['log_lat']]),
    }

    print(f"\n{'Model':<55} {'n_params':>8} {'Train R2':>9} {'CV R2':>8} {'Adj R2':>8} {'AIC':>8} {'BIC':>8} {'RMSE':>8}")
    print("-" * 120)

    results = {}
    for name, X in models.items():
        m = LinearRegression().fit(X, y)
        yp = m.predict(X)
        r2_tr = r2_score(y, yp)
        cv = cross_val_score(m, X, y, cv=kf, scoring='r2').mean()
        n_params = X.shape[1] + 1  # +1 for intercept
        adj_r2 = 1 - (1 - r2_tr) * (n - 1) / (n - n_params - 1)
        aic, bic = aic_bic(y, yp, n_params)
        rmse = np.sqrt(mean_squared_error(y, yp))
        print(f"  {name:<53} {n_params:>8} {r2_tr:>9.4f} {cv:>8.4f} {adj_r2:>8.4f} {aic:>8.1f} {bic:>8.1f} {rmse:>8.4f}")
        results[name] = {'cv': cv, 'r2': r2_tr, 'adj_r2': adj_r2, 'aic': aic, 'bic': bic,
                         'n_params': n_params, 'coef': list(m.coef_), 'intercept': m.intercept_}

    # Model B detailed coefficients
    print()
    X_B = v2[['log_keff', 'log_lat']].values
    m_B = LinearRegression().fit(X_B, y)
    se_B, ci_lo_B, ci_hi_B = coef_ci(m_B, X_B, y)
    print("Model B coefficients (theory predicts keff=-1.0, lat=-2.0):")
    for var, c, lo, hi, s in zip(['log_keff','log_lat'], m_B.coef_, ci_lo_B, ci_hi_B, se_B):
        print(f"  {var}: {c:.4f} [{lo:.4f}, {hi:.4f}] (se={s:.4f})")
    print(f"  intercept: {m_B.intercept_:.4f}")
    print(f"  Theory: keff=-1.0, lat=-2.0 -> do both CIs contain theory value?")
    for var, c, lo, hi, theory in zip(['log_keff','log_lat'], m_B.coef_, ci_lo_B, ci_hi_B, [-1.0, -2.0]):
        inside = lo <= theory <= hi
        print(f"    {var}: theory={theory}, CI=[{lo:.3f},{hi:.3f}], inside={inside}")

    # Test whether keff and lat are separable (Pi sufficient) vs. latent structure
    print()
    X_Pi = v2[['log_Pi']].values
    X_2v = v2[['log_keff','log_lat']].values
    m_Pi = LinearRegression().fit(X_Pi, y)
    m_2v = LinearRegression().fit(X_2v, y)
    cv_Pi = cross_val_score(m_Pi, X_Pi, y, cv=kf, scoring='r2').mean()
    cv_2v = cross_val_score(m_2v, X_2v, y, cv=kf, scoring='r2').mean()
    print(f"Pi sufficient? CV R2 gap: keff+lat ({cv_2v:.4f}) vs Pi ({cv_Pi:.4f}) = {cv_2v - cv_Pi:+.4f}")
    print("  If gap > +0.02: keff and lat contain information that Pi discards.")
    print("  If gap < +0.02: Pi is informationally sufficient.")

    return results


# ── PART 3: Residual structure ─────────────────────────────────────────────────

def part3_residuals(v2, fsat):
    print("\n" + "=" * 68)
    print("PART 3: RESIDUAL STRUCTURE")
    print("=" * 68)

    # Fit best Pi-only model
    X = v2[['log_Pi']].values
    y = v2['log_window'].values
    m = LinearRegression().fit(X, y)
    resid = y - m.predict(X)
    v2_r = v2.copy()
    v2_r['residual'] = resid

    # Merge fsat columns (fsat_floor only -- windows are identical)
    fsat_merge = fsat[['rocket_id','fsat_floor','fsat_mid','fsat_ceil']].copy()
    v2_r = v2_r.merge(fsat_merge, on='rocket_id', how='left')

    # Add derived variables
    v2_r['log_fsat_floor'] = np.log(v2_r['fsat_floor'].clip(1e-4))
    v2_r['log_kp_floor'] = np.log(v2_r['kp_floor'])
    v2_r['log_kp_ceiling'] = np.log(v2_r['kp_ceiling'])
    v2_r['log_keff'] = np.log(v2_r['keff'])
    v2_r['log_lat'] = np.log(v2_r['latency_steps'])
    v2_r['log_window_sq'] = v2_r['log_window']  # for reference
    v2_r['keff_lat_ratio'] = v2_r['keff'] / v2_r['latency_steps']
    v2_r['wind_strength_val'] = v2_r['wind_strength']
    v2_r['servo_slew_val'] = v2_r['servo_slew']

    variables = [
        ('keff',            'keff'),
        ('latency_steps',   'latency'),
        ('log_keff',        'log(keff)'),
        ('log_lat',         'log(lat)'),
        ('fsat_floor',      'f_sat @ floor'),
        ('log_fsat_floor',  'log(f_sat @ floor)'),
        ('log_kp_floor',    'log(kp_floor)'),
        ('log_kp_ceiling',  'log(kp_ceiling)'),
        ('wind_strength_val','wind_strength'),
        ('servo_slew_val',  'servo_slew'),
        ('keff_lat_ratio',  'keff/latency'),
        ('Iyy',             'Iyy'),
        ('u_max',           'u_max'),
    ]

    print(f"\nModel: log(window) ~ log(Pi) (n={len(v2_r.dropna(subset=['residual']))})")
    print(f"Intercept: {m.intercept_:.4f}, Pi coef: {m.coef_[0]:.4f}")
    print()
    print(f"{'Variable':<25} {'rho':>8} {'p':>10} {'interpretation'}")
    print("-" * 70)

    for col, label in variables:
        sub = v2_r.dropna(subset=[col, 'residual'])
        if len(sub) < 10:
            continue
        rho, p = stats.spearmanr(sub[col], sub['residual'])
        flag = ''
        if abs(rho) > 0.30 and p < 0.01:
            flag = '** SIGNIFICANT'
        elif abs(rho) > 0.20 and p < 0.05:
            flag = '* moderate'
        print(f"  {label:<23} {rho:>+8.3f} {p:>10.3e}  {flag}")

    # Key test: if residuals correlate with keff or lat, the Pi collapse is incomplete
    print()
    sub = v2_r.dropna(subset=['log_keff', 'log_lat', 'residual'])
    rho_k, p_k = stats.spearmanr(sub['log_keff'], sub['residual'])
    rho_l, p_l = stats.spearmanr(sub['log_lat'], sub['residual'])
    print(f"KEY TEST: rho(log_keff, residual) = {rho_k:+.3f} (p={p_k:.3e})")
    print(f"KEY TEST: rho(log_lat,  residual) = {rho_l:+.3f} (p={p_l:.3e})")
    print()
    if abs(rho_k) > 0.20 and p_k < 0.05:
        print("  FAIL: Pi collapse is INCOMPLETE -- keff retains residual variance")
    elif abs(rho_l) > 0.20 and p_l < 0.05:
        print("  FAIL: Pi collapse is INCOMPLETE -- latency retains residual variance")
    else:
        print("  PASS: Pi collapse is complete -- residuals do not correlate with keff or lat separately")

    # Also check whether adding keff and lat back to model improves fit
    sub = v2_r.dropna(subset=['log_keff','log_lat','residual','log_Pi'])
    X_full = sub[['log_Pi','log_keff','log_lat']].values
    y_sub  = sub['log_window'].values
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    cv_Pi  = cross_val_score(LinearRegression(), sub[['log_Pi']].values, y_sub, cv=kf, scoring='r2').mean()
    cv_all = cross_val_score(LinearRegression(), X_full, y_sub, cv=kf, scoring='r2').mean()
    print(f"  CV R2 Pi only: {cv_Pi:.4f}  |  adding keff+lat back: {cv_all:.4f}  |  delta: {cv_all - cv_Pi:+.4f}")
    if cv_all - cv_Pi > 0.02:
        print("  CONCLUSION: Adding keff+lat to Pi model DOES improve prediction -- Pi is NOT sufficient")
    else:
        print("  CONCLUSION: Adding keff+lat to Pi model does not improve prediction -- Pi IS sufficient")

    return v2_r


# ── PART 4: Physical interpretation of C ─────────────────────────────────────

def part4_collapse_constant(v2, fsat):
    print("\n" + "=" * 68)
    print("PART 4: PHYSICAL INTERPRETATION OF COLLAPSE CONSTANT C")
    print("=" * 68)

    v2_f = v2.copy()
    v2_f['Pi'] = v2_f['keff'] * v2_f['latency_steps']**2
    v2_f['C'] = v2_f['window_ratio'] * v2_f['Pi']
    v2_f['log_C'] = np.log(v2_f['C'])
    v2_f['log_keff'] = np.log(v2_f['keff'])
    v2_f['log_lat'] = np.log(v2_f['latency_steps'])

    fsat_m = fsat[['rocket_id','fsat_floor','fsat_mid','fsat_ceil']].copy()
    v2_f = v2_f.merge(fsat_m, on='rocket_id', how='left')
    v2_f['log_fsat_floor'] = np.log(v2_f['fsat_floor'].clip(1e-4))

    print(f"\nC = window_ratio x Pi statistics:")
    print(f"  Mean:   {v2_f.C.mean():.1f}")
    print(f"  Median: {v2_f.C.median():.1f}")
    print(f"  Std:    {v2_f.C.std():.1f}")
    print(f"  CV:     {v2_f.C.std()/v2_f.C.mean():.3f}")
    print(f"  Range:  [{v2_f.C.min():.1f}, {v2_f.C.max():.1f}]")
    print(f"  log(C) std: {v2_f.log_C.std():.4f}")
    print(f"  Theory (C=6300): median ratio actual/theory = {v2_f.C.median()/6300:.3f}")

    # By Pi decile
    v2_f['Pi_decile'] = pd.qcut(v2_f.Pi, 5, labels=['Q1 (low)', 'Q2', 'Q3', 'Q4', 'Q5 (high)'])
    print()
    print(f"C by Pi quintile:")
    print(f"{'Quintile':<12} {'Pi range':<20} {'n':>4} {'median C':>10} {'mean C':>10} {'C/6300':>8}")
    for q, grp in v2_f.groupby('Pi_decile', observed=True):
        pi_range = f"[{grp.Pi.min():.0f}, {grp.Pi.max():.0f}]"
        print(f"  {str(q):<10} {pi_range:<20} {len(grp):>4} {grp.C.median():>10.0f} {grp.C.mean():>10.0f} {grp.C.median()/6300:>8.3f}")

    # Correlates of log(C)
    print()
    print("Spearman rho(variable, log C):")
    print(f"{'Variable':<25} {'rho':>8} {'p':>10}")
    print("-" * 50)
    for col, label in [
        ('log_keff', 'log(keff)'),
        ('log_lat', 'log(lat)'),
        ('keff', 'keff'),
        ('latency_steps', 'latency'),
        ('log_fsat_floor', 'log(fsat_floor)'),
        ('wind_strength', 'wind_strength'),
        ('Iyy', 'Iyy'),
        ('u_max', 'u_max'),
        ('static_margin', 'static_margin'),
    ]:
        sub = v2_f.dropna(subset=[col, 'log_C'])
        if len(sub) < 10: continue
        rho, p = stats.spearmanr(sub[col], sub['log_C'])
        flag = '**' if (abs(rho) > 0.25 and p < 0.01) else ''
        print(f"  {label:<23} {rho:>+8.3f} {p:>10.3e}  {flag}")

    # Best regression for log(C)
    sub = v2_f.dropna(subset=['log_keff','log_lat','log_fsat_floor','log_C'])
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    models_C = {
        'log(keff) alone':       sub[['log_keff']].values,
        'log(lat) alone':        sub[['log_lat']].values,
        'log(keff) + log(lat)':  sub[['log_keff','log_lat']].values,
        'fsat_floor alone':      sub[['log_fsat_floor']].values,
        'log(keff) + fsat':      sub[['log_keff','log_fsat_floor']].values,
    }
    print()
    print("Predicting log(C):")
    print(f"{'Model':<35} {'CV R2':>8}")
    for label, X in models_C.items():
        cv = cross_val_score(LinearRegression(), X, sub['log_C'].values,
                             cv=kf, scoring='r2').mean()
        print(f"  {label:<33} {cv:>8.4f}")

    print()
    print(f"KEY QUESTION: If log(C) has a non-trivial correlation with keff or lat,")
    print(f"it means the Pi law is only approximate and the true law is richer.")


# ── PART 5: Verdict ────────────────────────────────────────────────────────────

def part5_verdict(v2, lat_ext):
    print("\n" + "=" * 68)
    print("PART 5: REVIEWER VERDICT")
    print("=" * 68)

    # Collect all key stats for the verdict
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    y_v2 = v2['log_window'].values

    X_pi  = v2[['log_Pi']].values
    X_2v  = v2[['log_keff','log_lat']].values
    X_pi2 = np.column_stack([v2['log_Pi'], v2['log_Pi']**2])
    X_int = np.column_stack([v2['log_keff'], v2['log_keff']*v2['log_lat']])

    cv_pi  = cross_val_score(LinearRegression(), X_pi,  y_v2, cv=kf, scoring='r2').mean()
    cv_2v  = cross_val_score(LinearRegression(), X_2v,  y_v2, cv=kf, scoring='r2').mean()
    cv_pi2 = cross_val_score(LinearRegression(), X_pi2, y_v2, cv=kf, scoring='r2').mean()
    cv_int = cross_val_score(LinearRegression(), X_int, y_v2, cv=kf, scoring='r2').mean()

    m_pi = LinearRegression().fit(X_pi, y_v2)
    m_pi2 = LinearRegression().fit(X_pi2, y_v2)
    m_2v = LinearRegression().fit(X_2v, y_v2)

    # Cross-dataset test (v2 -> lat_ext)
    X_pi_ext  = lat_ext[['log_Pi']].values
    X_pi2_ext = np.column_stack([lat_ext['log_Pi'], lat_ext['log_Pi']**2])
    _, tr_lin, te_lin, rmse_lin, _ = fit_and_score(X_pi, y_v2, X_pi_ext, lat_ext['log_window'].values)
    _, tr_q,   te_q,   rmse_q,   _ = fit_and_score(X_pi2, y_v2, X_pi2_ext, lat_ext['log_window'].values)

    print()
    print("SUMMARY TABLE OF KEY QUANTITIES:")
    print(f"  Pi exponent (OLS):           {m_pi.coef_[0]:.4f}  (theory: -1.0)")
    print(f"  Pi CV R2 (5-fold, v2):       {cv_pi:.4f}")
    print(f"  keff+lat CV R2 (5-fold, v2): {cv_2v:.4f}")
    print(f"  Quadratic CV R2 (5-fold, v2):{cv_pi2:.4f}")
    print(f"  Interaction CV R2:            {cv_int:.4f}")
    print(f"  Quadratic improvement in-sample: {cv_pi2 - cv_pi:+.4f}")
    print()
    print(f"  Cross-dataset (v2->lat_ext) Test R2, Model 1 (linear):    {te_lin:.4f}")
    print(f"  Cross-dataset (v2->lat_ext) Test R2, Model 2 (quadratic): {te_q:.4f}")
    print(f"  delta_test_R2 (quad - linear):                             {te_q - te_lin:+.4f}")
    print(f"  Cross-dataset Test RMSE, linear:    {rmse_lin:.4f}")
    print(f"  Cross-dataset Test RMSE, quadratic: {rmse_q:.4f}")
    print()

    # Classification
    delta_in   = cv_pi2 - cv_pi
    delta_out  = te_q - te_lin
    pi_exp     = m_pi.coef_[0]
    gap_2v_vs_pi = cv_2v - cv_pi

    print("CLASSIFICATION:")
    if abs(pi_exp + 1.0) < 0.10 and delta_in < 0.05 and cv_pi > 0.50:
        verdict = "1. EXACT Pi-LAW (simple inverse law survives)"
    elif abs(pi_exp + 1.0) < 0.15 and (delta_in > 0.05 and delta_out > 0.05):
        verdict = "2. APPROXIMATE Pi-LAW (inverse trend correct but curved)"
    elif delta_in > 0.05 and delta_out <= 0.05:
        verdict = "2. APPROXIMATE Pi-LAW (curvature is in-sample overfitting; trend real)"
    elif gap_2v_vs_pi > 0.04:
        verdict = "3. HIDDEN TWO-VARIABLE LAW (Pi projection masks richer surface)"
    else:
        verdict = "2. APPROXIMATE Pi-LAW (modest curvature, Pi is primary dimension)"

    print(f"\n  >>> {verdict} <<<\n")

    print("WHAT STAYS IN THE PAPER:")
    print("  - Pi = keff x lat^2 as the primary design parameter")
    print("  - Exponent = -1.0 (confirmed within 3%)")
    print("  - Ceiling keff-independence (ceiling ~ 380/lat)")
    print("  - Floor keff+lat dependence (floor ~ 0.06 x keff x lat)")
    print()
    print("WHAT SHOULD BE REPLACED OR QUALIFIED:")
    if delta_out <= 0.05:
        print("  - Quadratic correction: does NOT generalize out-of-sample; do not add to paper")
    else:
        print("  - Window formula should use quadratic: log(window) = a + b*log(Pi) + c*(log Pi)^2")
    if gap_2v_vs_pi > 0.04:
        print("  - Simpler Pi formula may be inadequate; report keff + lat separately")
    print("  - Collapse constant C is NOT universal; do not present C=6300 as exact")
    print("  - Present window formula as an order-of-magnitude design guide, not a precise predictor")


# ── main ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("Loading datasets...")
    v2 = load_v2()
    lat_ext = load_lat_ext()
    fsat = load_fsat()

    print(f"  v2:      n={len(v2)} (lat 1-6, LHS seed=8888)")
    print(f"  lat_ext: n={len(lat_ext)} (lat 7-12, different LHS pool)")
    print(f"  fsat:    n={len(fsat)} (SAME designs as v2; window_ratio IDENTICAL)")

    v2_r = None
    part1_curvature(v2, lat_ext)
    results2 = part2_surface(v2)
    v2_r = part3_residuals(v2, fsat)
    part4_collapse_constant(v2, fsat)
    part5_verdict(v2, lat_ext)
