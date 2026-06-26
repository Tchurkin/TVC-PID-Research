"""
tools/reanalysis_suite.py  (2026-06-22)

Five analyses on existing CSVs -- no new simulations required.

Analyses:
  A. Ceiling-floor decomposition   (window_ratio_v2, n=116)
  B. Scaling collapse              (fsat_window_test, n=82)
  C. Pi-decile residual analysis   (fsat + v2 combined, n=166)
  D. Cross-architecture rank-order (LQR + SMC + observer data)
  E. Ziegler-Nichols validity domain (window_ratio_v2)

Run individual analyses:
  python tools/reanalysis_suite.py --analysis A
  python tools/reanalysis_suite.py --analysis B
  ...or omit flag to run all.

Outputs: experiments/results/reanalysis_{A-E}_py.csv + console summary.
"""

import sys, os, warnings
warnings.filterwarnings('ignore')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.stats import spearmanr, pearsonr, kendalltau
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import cross_val_score
from sklearn.preprocessing import StandardScaler

ROOT    = Path(__file__).resolve().parents[1]
RES_DIR = ROOT / 'experiments' / 'results'

# ─────────────────────────────────────────────────────────────────────────────
# ANALYSIS A: Ceiling-floor decomposition
# ─────────────────────────────────────────────────────────────────────────────

def analysis_A():
    """
    Decompose window_ratio into separate ceiling and floor predictions.

    Theory predictions:
      ceiling(lat):        exponent on lat = -1.0, exponent on keff =  0.0
      floor(keff, lat):    exponent on keff = +1.0, exponent on lat = +1.0
      window = ceil/floor: exponent on keff = -1.0, exponent on lat = -2.0

    This directly answers reviewer attacks:
      - Attack 7: ceiling validated on only 5 spot-checks
      - Attack 3: architecture-invariant floor claim needs decomposition evidence
    """
    print("\n" + "="*70)
    print("ANALYSIS A: Ceiling-floor decomposition")
    print("="*70)

    v2 = pd.read_csv(RES_DIR / 'window_ratio_v2_py.csv')
    valid = v2[(v2.floor_censored == 0) & (v2.ceil_censored == 0)].copy()
    # Drop rows with NaN floor/ceiling (4 out of 86)
    valid = valid.dropna(subset=['kp_floor', 'kp_ceiling', 'window_ratio']).copy()
    n = len(valid)
    print(f"  Non-censored designs (after dropping NaN floor/ceiling): n={n}")

    valid['log_keff']    = np.log(valid['keff'])
    valid['log_lat']     = np.log(valid['latency_steps'])
    valid['log_ceil']    = np.log(valid['kp_ceiling'])
    valid['log_floor']   = np.log(valid['kp_floor'])
    valid['log_window']  = np.log(valid['window_ratio'])
    valid['log_Pi']      = np.log(valid['keff'] * valid['latency_steps']**2)

    def fit_log(df, X_cols, y_col, label):
        X = df[X_cols].values
        y = df[y_col].values
        m = LinearRegression().fit(X, y)
        r2 = m.score(X, y)
        cv = cross_val_score(m, X, y, cv=5, scoring='r2').mean()
        coefs = {c: round(m.coef_[i], 3) for i, c in enumerate(X_cols)}
        intercept = round(m.intercept_, 3)
        print(f"  {label:45s}  R2={r2:.3f}  CV_R2={cv:.3f}  coefs={coefs}  intercept={intercept}")
        return m, r2, cv, coefs

    print("\n--- Ceiling: log(kp_ceiling) ~ log(keff) + log(lat) ---")
    print(f"  Theory: keff exponent = 0.0, lat exponent = -1.0")
    mc, r2c, cvc, coef_c = fit_log(valid, ['log_keff', 'log_lat'], 'log_ceil', 'ceiling ~ keff + lat')
    _, _, _, _ = fit_log(valid, ['log_lat'], 'log_ceil', 'ceiling ~ lat only')
    _, _, _, _ = fit_log(valid, ['log_keff'], 'log_ceil', 'ceiling ~ keff only')

    # Theoretical prediction: ceiling = 380/lat -> log(ceil) = log(380) - log(lat)
    pred_theory_ceil = np.log(380) - valid['log_lat']
    resid_theory_ceil = valid['log_ceil'] - pred_theory_ceil
    print(f"  Theoretical (380/lat) residuals: mean={resid_theory_ceil.mean():.3f}  "
          f"std={resid_theory_ceil.std():.3f}  "
          f"MAPE={np.abs(np.exp(resid_theory_ceil)-1).mean()*100:.1f}%")

    print("\n--- Floor: log(kp_floor) ~ log(keff) + log(lat) ---")
    print(f"  Theory: keff exponent = +1.0, lat exponent = +1.0")
    mf, r2f, cvf, coef_f = fit_log(valid, ['log_keff', 'log_lat'], 'log_floor', 'floor ~ keff + lat')
    _, _, _, _ = fit_log(valid, ['log_keff'], 'log_floor', 'floor ~ keff only')
    _, _, _, _ = fit_log(valid, ['log_lat'], 'log_floor', 'floor ~ lat only')

    # Theoretical prediction: floor = 0.06 * keff * lat
    pred_theory_floor = np.log(0.06) + valid['log_keff'] + valid['log_lat']
    resid_theory_floor = valid['log_floor'] - pred_theory_floor
    print(f"  Theoretical (0.06*keff*lat) residuals: mean={resid_theory_floor.mean():.3f}  "
          f"std={resid_theory_floor.std():.3f}  "
          f"MAPE={np.abs(np.exp(resid_theory_floor)-1).mean()*100:.1f}%")

    print("\n--- Window: log(window) ~ log(keff) + log(lat) ---")
    print(f"  Theory: keff exponent = -1.0, lat exponent = -2.0")
    mw, r2w, cvw, coef_w = fit_log(valid, ['log_keff', 'log_lat'], 'log_window', 'window ~ keff + lat')
    _, _, _, _ = fit_log(valid, ['log_Pi'], 'log_window', 'window ~ log(Pi)')

    print("\n--- Residual correlation: do ceiling and floor share variance? ---")
    ceil_pred = mc.predict(valid[['log_keff', 'log_lat']].values)
    floor_pred = mf.predict(valid[['log_keff', 'log_lat']].values)
    ceil_resid = valid['log_ceil'].values - ceil_pred
    floor_resid = valid['log_floor'].values - floor_pred
    r_resid, p_resid = pearsonr(ceil_resid, floor_resid)
    print(f"  r(ceil_residual, floor_residual) = {r_resid:.3f}  p={p_resid:.3f}")
    print(f"  Interpretation: {'SHARED unexplained variance (latent confounder)' if abs(r_resid)>0.3 else 'Independent residuals -- ceiling and floor driven by different sources'}")

    print("\n--- Double-squeeze: do ceiling and floor both track latency? ---")
    for lat in sorted(valid.latency_steps.unique()):
        sub = valid[valid.latency_steps == lat]
        print(f"  lat={lat}: n={len(sub):3d}  "
              f"ceil_mean={sub.kp_ceiling.mean():.1f}  "
              f"floor_mean={sub.kp_floor.mean():.2f}  "
              f"window_mean={sub.window_ratio.mean():.1f}")

    print("\n--- keff-independence of ceiling (key theoretical prediction) ---")
    rho_ceil_keff, p_ceil_keff = spearmanr(valid.keff, valid.kp_ceiling)
    rho_ceil_lat, p_ceil_lat = spearmanr(valid.latency_steps, valid.kp_ceiling)
    print(f"  rho(keff, ceiling) = {rho_ceil_keff:.3f}  p={p_ceil_keff:.4f}"
          f"  {'CONFIRMED keff-independent (|rho|<0.15)' if abs(rho_ceil_keff)<0.15 else 'WARNING: keff affects ceiling'}")
    print(f"  rho(lat,  ceiling) = {rho_ceil_lat:.3f}  p={p_ceil_lat:.4e}  (should be strongly negative)")

    # Save
    out = valid[['rocket_id','keff','latency_steps','kp_floor','kp_ceiling',
                 'window_ratio','log_Pi']].copy()
    out['ceil_theory_pred'] = np.exp(pred_theory_ceil)
    out['floor_theory_pred'] = np.exp(pred_theory_floor)
    out['ceil_resid_frac'] = np.exp(resid_theory_ceil) - 1
    out['floor_resid_frac'] = np.exp(resid_theory_floor) - 1
    out.to_csv(RES_DIR / 'reanalysis_A_decomposition_py.csv', index=False)
    print(f"\n  Saved: reanalysis_A_decomposition_py.csv  (n={len(out)})")
    return out


# ─────────────────────────────────────────────────────────────────────────────
# ANALYSIS B: Scaling collapse
# ─────────────────────────────────────────────────────────────────────────────

def analysis_B():
    """
    Scaling collapse: log(window * Pi) should be approximately constant.

    If the law window = C/Pi holds, then window*Pi = C for ALL designs.
    The scatter around this constant is the residual variance (CV R2 gap).
    This is a theory-neutral presentation -- no regression required.

    Answers reviewer attack 2: "R2=0.55 is too low for a law."
    A scaling collapse showing all 82 designs cluster around log(C)=8.9
    demonstrates the law holds to within a predictable noise floor.
    """
    print("\n" + "="*70)
    print("ANALYSIS B: Scaling collapse  (window * Pi = C)")
    print("="*70)

    # Use fsat dataset (independent evaluation, fresh seeds 40001-40020)
    fsat = pd.read_csv(RES_DIR / 'fsat_window_test_py.csv').dropna()
    n = len(fsat)
    print(f"  Independent dataset (fresh seeds 40001-40020): n={n}")

    collapse = fsat['window_ratio'] * fsat['Pi']
    log_collapse = np.log(collapse)

    C_median = np.median(collapse)
    C_mean   = np.mean(collapse)
    log_C_std = log_collapse.std()
    # Factor-of-2 range: exp(+/-log_C_std) bounds
    factor_at_1std = np.exp(log_C_std)
    pct_within_3x = (collapse.between(C_median/3, C_median*3)).mean() * 100

    print(f"\n  Collapse constant C = window * Pi:")
    print(f"    Median:  {C_median:.0f}")
    print(f"    Mean:    {C_mean:.0f}")
    print(f"    Std(log): {log_C_std:.3f}  (+/-1sd range: [{C_median/factor_at_1std:.0f}, {C_median*factor_at_1std:.0f}])")
    print(f"    Theory (6300/keff^0 × lat^2 -> C=6300): {6300:.0f}")
    print(f"    From formula components (380/0.06): {380/0.06:.0f}")
    print(f"    Designs within 3× of median: {pct_within_3x:.1f}%")

    print(f"\n  Pi exponent confirmation (OLS log(window) ~ log(Pi)):")
    X = np.log(fsat['Pi'].values).reshape(-1,1)
    y = np.log(fsat['window_ratio'].values)
    m = LinearRegression().fit(X, y)
    cv = cross_val_score(m, X, y, cv=5, scoring='r2').mean()
    print(f"    Exponent = {m.coef_[0]:.4f}  (theory = -1.000)")
    print(f"    Intercept = {m.intercept_:.3f}  -> C = exp(intercept) = {np.exp(m.intercept_):.0f}")
    print(f"    R2 = {m.score(X,y):.3f}  CV_R2 = {cv:.3f}")
    print(f"    Deviation from theory: {abs(m.coef_[0] - (-1.0)):.4f}  ({abs(m.coef_[0]+1)/1*100:.1f}% of -1.0)")

    # Breakdown by Pi decile
    print(f"\n  Scaling collapse by Pi decile (does constant C hold across Pi range?):")
    fsat['Pi_decile'] = pd.qcut(fsat['Pi'], q=5, labels=False)
    for dec in range(5):
        sub = fsat[fsat.Pi_decile == dec]
        c_sub = (sub.window_ratio * sub.Pi)
        print(f"    Decile {dec+1} (Pi {sub.Pi.min():.0f}-{sub.Pi.max():.0f}): "
              f"C_median={c_sub.median():.0f}  std(log)={np.log(c_sub).std():.3f}  n={len(sub)}")

    # Also run on v2 dataset for comparison
    v2 = pd.read_csv(RES_DIR / 'window_ratio_v2_py.csv')
    v2_val = v2[(v2.floor_censored==0) & (v2.ceil_censored==0)].copy()
    v2_val['Pi'] = v2_val['keff'] * v2_val['latency_steps']**2
    v2_val['collapse'] = v2_val['window_ratio'] * v2_val['Pi']
    print(f"\n  Comparison -- v2 original dataset (n={len(v2_val)}):")
    print(f"    C_median = {v2_val.collapse.median():.0f}  std(log) = {np.log(v2_val.collapse).std():.3f}")
    print(f"    Agreement with fsat dataset: {abs(v2_val.collapse.median() - C_median)/C_median*100:.1f}% difference")

    out = fsat[['rocket_id','keff','latency','Pi','window_ratio']].copy()
    out['collapse_C'] = collapse
    out['log_collapse_C'] = log_collapse
    out.to_csv(RES_DIR / 'reanalysis_B_collapse_py.csv', index=False)
    print(f"\n  Saved: reanalysis_B_collapse_py.csv  (n={n})")
    return out


# ─────────────────────────────────────────────────────────────────────────────
# ANALYSIS C: Pi-decile residual analysis
# ─────────────────────────────────────────────────────────────────────────────

def analysis_C():
    """
    Examine residual variance as a function of Pi.

    Questions:
      - Is residual variance uniform across Pi levels (random noise)?
      - Or does it increase at high Pi (bang-bang chaos)?
      - Does floor mechanism transition (fsat) predict residual direction?

    Answers: confirms stochastic residual claim; shows where formula is most reliable.
    """
    print("\n" + "="*70)
    print("ANALYSIS C: Pi-decile residual analysis")
    print("="*70)

    fsat = pd.read_csv(RES_DIR / 'fsat_window_test_py.csv').dropna()

    fsat['log_Pi']     = np.log(fsat['Pi'])
    fsat['log_window'] = np.log(fsat['window_ratio'])
    fsat['log_Pi2']    = fsat['log_Pi']**2  # for quadratic fit check

    # Fit log(window) ~ log(Pi) and extract residuals
    X = fsat['log_Pi'].values.reshape(-1,1)
    y = fsat['log_window'].values
    m = LinearRegression().fit(X, y)
    fsat['pred_log_window'] = m.predict(X)
    fsat['residual']        = y - fsat['pred_log_window']
    fsat['abs_residual']    = fsat['residual'].abs()

    print(f"\n  Global residual statistics (n={len(fsat)}):")
    print(f"    Mean:    {fsat.residual.mean():.4f}  (bias; should be ~0)")
    print(f"    Std:     {fsat.residual.std():.3f}")
    print(f"    RMSE:    {np.sqrt((fsat.residual**2).mean()):.3f}")

    print(f"\n  Residual analysis by Pi quintile:")
    fsat['Pi_quintile'] = pd.qcut(fsat['Pi'], q=5, labels=False)
    for q in range(5):
        sub = fsat[fsat.Pi_quintile == q]
        r_fsat, p_fsat = spearmanr(sub['fsat_floor'], sub['residual'])
        print(f"    Quintile {q+1} (Pi {sub.Pi.min():.0f}-{sub.Pi.max():.0f}): "
              f"n={len(sub):2d}  "
              f"resid_std={sub.residual.std():.3f}  "
              f"resid_mean={sub.residual.mean():+.3f}  "
              f"rho(fsat,resid)={r_fsat:+.3f}")

    # Is residual variance heteroscedastic (increases with Pi)?
    rho_var_Pi, p_var_Pi = spearmanr(fsat.Pi, fsat.abs_residual)
    print(f"\n  Heteroscedasticity test:")
    print(f"    rho(Pi, |residual|) = {rho_var_Pi:+.3f}  p={p_var_Pi:.4f}")
    if p_var_Pi < 0.05 and rho_var_Pi > 0:
        print(f"    SIGNIFICANT: residual variance INCREASES with Pi (bang-bang chaos)")
    elif p_var_Pi < 0.05 and rho_var_Pi < 0:
        print(f"    SIGNIFICANT: residual variance DECREASES with Pi (unusual)")
    else:
        print(f"    NOT significant: residual variance is approximately uniform (consistent with stochastic noise)")

    # Does fsat predict residual DIRECTION? (i.e., does bang-bang regime systematically over/under predict)
    rho_dir, p_dir = spearmanr(fsat['fsat_floor'], fsat['residual'])
    print(f"\n  Floor mechanism and residual direction:")
    print(f"    rho(fsat_floor, residual) = {rho_dir:+.3f}  p={p_dir:.4f}")
    print(f"    Linear regime (fsat<0.1): mean resid = {fsat[fsat.fsat_floor<0.1].residual.mean():+.3f}")
    print(f"    Bang-bang regime (fsat>0.5): mean resid = {fsat[fsat.fsat_floor>0.5].residual.mean():+.3f}")
    print(f"    Interpretation: {'formula over-predicts easy designs' if rho_dir > 0.2 else 'formula is unbiased across regimes' if abs(rho_dir) < 0.15 else 'formula under-predicts bang-bang designs'}")

    # Quadratic check: does log(Pi)^2 add anything?
    X2 = fsat[['log_Pi', 'log_Pi2']].values
    m2 = LinearRegression().fit(X2, y)
    cv1 = cross_val_score(LinearRegression(), X, y, cv=5, scoring='r2').mean()
    cv2 = cross_val_score(LinearRegression(), X2, y, cv=5, scoring='r2').mean()
    print(f"\n  Linearity check (log scale):")
    print(f"    Linear (log Pi): CV R2 = {cv1:.3f}")
    print(f"    Quadratic (log Pi + log Pi^2): CV R2 = {cv2:.3f}  delta = {cv2-cv1:+.3f}")
    print(f"    Interpretation: {'nonlinear component improves fit' if cv2-cv1 > 0.02 else 'linear-in-log is adequate; no evidence of curvature'}")

    out = fsat[['rocket_id','keff','latency','Pi','window_ratio',
                'fsat_floor','residual','abs_residual','Pi_quintile']].copy()
    out.to_csv(RES_DIR / 'reanalysis_C_residuals_py.csv', index=False)
    print(f"\n  Saved: reanalysis_C_residuals_py.csv")
    return out


# ─────────────────────────────────────────────────────────────────────────────
# ANALYSIS D: Cross-architecture rank-order consistency
# ─────────────────────────────────────────────────────────────────────────────

def analysis_D():
    """
    For each architecture (LQR, SMC, PD/observer), compute per-design window metrics
    and test whether rho(Pi, window_metric) is consistent across architectures.

    Key distinction: LQR/SMC/PD are all the same PD class. Observer (ADRC) is different.
    The analysis should show:
      - LQR/SMC/PD: rho(Pi, metric) ≈ -0.75 (same class, same constraint)
      - ADRC: rho(Pi, n_pass) weaker (escapes the constraint)

    For LQR: derive floor = min Q/R achieving SR>=0.80, ceiling = max Q/R achieving SR>=0.80
    For SMC: derive floor and ceiling per lambda_s slice
    For Observer data: use peak SR and n_pass directly
    """
    print("\n" + "="*70)
    print("ANALYSIS D: Cross-architecture rank-order consistency")
    print("="*70)

    lqr = pd.read_csv(RES_DIR / 'lqr_gain_sweep_py.csv')
    smc = pd.read_csv(RES_DIR / 'smc_sweep_detail_py.csv')
    obs = pd.read_csv(RES_DIR / 'observer_universality_py.csv')

    # LQR floor/ceiling: for each design, find min and max Q/R achieving SR>=0.80
    PASS_THRESH = 0.80
    def extract_window(df, id_col, param_col, sr_col, design_meta_cols):
        rows = []
        for rid, g in df.groupby(id_col):
            meta = {c: g[c].iloc[0] for c in design_meta_cols}
            passing = g[g[sr_col] >= PASS_THRESH]
            if len(passing) < 2:
                n_pass = len(passing)
                floor_val = ceil_val = win_val = np.nan
            else:
                floor_val = passing[param_col].min()
                ceil_val  = passing[param_col].max()
                win_val   = ceil_val / floor_val
                n_pass    = len(passing)
            row = {id_col: rid, 'n_pass': n_pass,
                   f'{param_col}_floor': floor_val,
                   f'{param_col}_ceiling': ceil_val,
                   'window_equiv': win_val}
            row.update(meta)
            rows.append(row)
        return pd.DataFrame(rows)

    lqr_win = extract_window(lqr, 'rocket_id', 'qr', 'sr', ['td', 'keff', 'latency'])
    lqr_win['Pi'] = lqr_win['keff'] * lqr_win['latency']**2
    lqr_win['log_Pi'] = np.log(lqr_win['Pi'])

    # For SMC: compute window per lambda_s slice (best lambda_s defines "window")
    smc_best = smc.groupby(['rocket_id','td','keff','latency'])['sr'].max().reset_index()
    smc_best.columns = ['rocket_id','td','keff','latency','peak_sr_smc']
    smc_npass = smc.groupby('rocket_id').apply(lambda g: (g.sr >= PASS_THRESH).sum()).reset_index()
    smc_npass.columns = ['rocket_id','n_pass_smc']
    smc_win = smc_best.merge(smc_npass, on='rocket_id')
    smc_win['Pi'] = smc_win['keff'] * smc_win['latency']**2

    # Observer data: already has n_pass_pd, n_pass_pidi, n_pass_adrc, peak values
    obs['Pi'] = obs['keff'] * obs['latency']**2
    obs['log_Pi'] = np.log(obs['Pi'])

    print("\n--- LQR window analysis ---")
    lqr_valid = lqr_win.dropna(subset=['window_equiv'])
    print(f"  Designs with valid window: {len(lqr_valid)}/50")
    rho_lqr_npass, p1 = spearmanr(lqr_win['Pi'], lqr_win['n_pass'])
    rho_lqr_win, p2 = spearmanr(lqr_valid['Pi'], np.log(lqr_valid['window_equiv']))
    rho_lqr_ceil, p3 = spearmanr(lqr_valid['Pi'], np.log(lqr_valid['qr_ceiling']))
    rho_lqr_floor, p4 = spearmanr(lqr_valid['Pi'], np.log(lqr_valid['qr_floor']))
    print(f"  rho(Pi, n_pass_lqr)   = {rho_lqr_npass:+.3f}  p={p1:.2e}")
    print(f"  rho(Pi, log_window_lqr) = {rho_lqr_win:+.3f}  p={p2:.2e}")
    print(f"  rho(Pi, log_qr_ceiling) = {rho_lqr_ceil:+.3f}  p={p3:.2e}  (ceiling should DECREASE with Pi)")
    print(f"  rho(Pi, log_qr_floor)   = {rho_lqr_floor:+.3f}  p={p4:.2e}  (floor should INCREASE with Pi)")
    print(f"  NOTE: LQR Q/R is INVERSE of PID Kp -- higher Q/R = more aggressive")
    print(f"    -> LQR ceiling = max Q/R achieving SR>=0.80 (analogous to Kp_ceiling)")
    print(f"    -> LQR floor   = min Q/R achieving SR>=0.80 (analogous to Kp_floor)")
    print(f"    -> LQR window  = ceiling/floor = same interpretation as PID window_ratio")

    print("\n--- SMC analysis ---")
    rho_smc_peak, p1 = spearmanr(smc_win['Pi'], smc_win['peak_sr_smc'])
    rho_smc_npass, p2 = spearmanr(smc_win['Pi'], smc_win['n_pass_smc'])
    print(f"  rho(Pi, peak_sr_smc) = {rho_smc_peak:+.3f}  p={p1:.2e}")
    print(f"  rho(Pi, n_pass_smc)  = {rho_smc_npass:+.3f}  p={p2:.2e}")

    print("\n--- Observer universality: PD vs PID-I vs ADRC ---")
    for metric, label in [('n_pass_pd', 'PD'), ('n_pass_pidi', 'PID-I'), ('n_pass_adrc', 'ADRC')]:
        rho, p = spearmanr(obs['Pi'], obs[metric])
        rho_td, p_td = spearmanr(obs['td'], obs[metric])
        print(f"  rho(Pi_keff, {label:8s} n_pass) = {rho:+.3f}  p={p:.2e}  |  rho(td, {label}) = {rho_td:+.3f}")

    print("\n--- Cross-architecture summary (same 50 designs) ---")
    print(f"  Architecture           rho(Pi, performance_metric)   interpretation")
    print(f"  PD (observer study)    {spearmanr(obs.Pi, obs.n_pass_pd)[0]:+.3f}                        full Pi constraint")
    print(f"  PID-I                  {spearmanr(obs.Pi, obs.n_pass_pidi)[0]:+.3f}                        partial escape")
    print(f"  ADRC/ESO               {spearmanr(obs.Pi, obs.n_pass_adrc)[0]:+.3f}                        strongest escape")
    print(f"  LQR (indep. 50 des.)   {rho_lqr_npass:+.3f}                        same PD class")
    print(f"  SMC (indep. 50 des.)   {rho_smc_npass:+.3f}                        same PD class")

    # Rank-order consistency within the same-50-design observer study
    print("\n--- Rank-order agreement (PD vs PID-I vs ADRC, same 50 designs) ---")
    tau_pd_pidi, p_t1 = kendalltau(obs.n_pass_pd, obs.n_pass_pidi)
    tau_pd_adrc, p_t2 = kendalltau(obs.n_pass_pd, obs.n_pass_adrc)
    tau_pidi_adrc, p_t3 = kendalltau(obs.n_pass_pidi, obs.n_pass_adrc)
    print(f"  Kendall tau(PD, PID-I)  = {tau_pd_pidi:.3f}  p={p_t1:.2e}  (should be high: same class)")
    print(f"  Kendall tau(PD, ADRC)   = {tau_pd_adrc:.3f}  p={p_t2:.2e}  (should be lower: ESO breaks constraint)")
    print(f"  Kendall tau(PID-I,ADRC) = {tau_pidi_adrc:.3f}  p={p_t3:.2e}")

    # Save
    obs_out = obs[['rocket_id','td','keff','latency','Pi',
                   'n_pass_pd','n_pass_pidi','n_pass_adrc',
                   'peak_pd','peak_pidi','peak_adrc']].copy()
    obs_out.to_csv(RES_DIR / 'reanalysis_D_arch_py.csv', index=False)
    lqr_valid.to_csv(RES_DIR / 'reanalysis_D_lqr_window_py.csv', index=False)
    print(f"\n  Saved: reanalysis_D_arch_py.csv, reanalysis_D_lqr_window_py.csv")
    return obs_out, lqr_valid


# ─────────────────────────────────────────────────────────────────────────────
# ANALYSIS E: Ziegler-Nichols validity domain
# ─────────────────────────────────────────────────────────────────────────────

def analysis_E():
    """
    Determine for what Pi range the Z-N rule (Kp_ZN = 228/lat, Kd_ZN = 0.57)
    lands inside the measured gain window.

    Theory: Kp_ZN < Kp_ceiling requires 228/lat < 380/lat -> always true (228 < 380).
    Theory: Kp_ZN > Kp_floor requires 228/lat > 0.06*keff*lat -> Pi < 228/0.06 = 3800.

    Empirical test: for each design with valid floor and ceiling in window_ratio_v2,
    check whether Kp_ZN = 228/lat falls inside [kp_floor, kp_ceiling].

    Also compute a conservative version: Kp_conservative = 190/lat (0.5 × K_u).
    """
    print("\n" + "="*70)
    print("ANALYSIS E: Ziegler-Nichols validity domain")
    print("="*70)

    v2 = pd.read_csv(RES_DIR / 'window_ratio_v2_py.csv')
    valid = v2[(v2.floor_censored == 0) & (v2.ceil_censored == 0)].copy()
    valid = valid.dropna(subset=['kp_floor','kp_ceiling','window_ratio']).copy()
    valid['Pi'] = valid['keff'] * valid['latency_steps']**2
    valid['Kp_ZN']   = 228.0 / valid['latency_steps']
    valid['Kp_cons'] = 190.0 / valid['latency_steps']
    valid['Kp_floor_theory'] = 0.06 * valid['keff'] * valid['latency_steps']

    # Is Z-N inside window?
    valid['ZN_inside']   = (valid['Kp_ZN'] > valid['kp_floor']) & (valid['Kp_ZN'] < valid['kp_ceiling'])
    valid['cons_inside'] = (valid['Kp_cons'] > valid['kp_floor']) & (valid['Kp_cons'] < valid['kp_ceiling'])
    valid['ZN_above_floor'] = valid['Kp_ZN'] > valid['kp_floor']
    valid['ZN_below_ceil']  = valid['Kp_ZN'] < valid['kp_ceiling']

    n = len(valid)
    print(f"\n  Non-censored designs: n={n}")
    print(f"\n  Z-N rule (Kp = 228/lat) validity:")
    print(f"    Inside window:          {valid.ZN_inside.sum()}/{n}  ({valid.ZN_inside.mean()*100:.1f}%)")
    print(f"    Above floor (safe low): {valid.ZN_above_floor.sum()}/{n}  ({valid.ZN_above_floor.mean()*100:.1f}%)")
    print(f"    Below ceiling (safe hi): {valid.ZN_below_ceil.sum()}/{n}  ({valid.ZN_below_ceil.mean()*100:.1f}%)")
    print(f"\n  Conservative rule (Kp = 190/lat) validity:")
    print(f"    Inside window:          {valid.cons_inside.sum()}/{n}  ({valid.cons_inside.mean()*100:.1f}%)")

    # Pi threshold for Z-N failure
    zn_fails = valid[~valid.ZN_inside]
    print(f"\n  Designs where Z-N fails (n={len(zn_fails)}):")
    if len(zn_fails) > 0:
        fail_above_floor = zn_fails[~zn_fails.ZN_above_floor]
        fail_above_ceil  = zn_fails[~zn_fails.ZN_below_ceil]
        print(f"    Z-N below floor (overtuned? No -- ZN is conservative, so below floor means floor>228/lat):")
        print(f"      Count: {len(fail_above_floor)}")
        if len(fail_above_floor) > 0:
            print(f"      Pi range: {fail_above_floor.Pi.min():.0f} - {fail_above_floor.Pi.max():.0f}")
            print(f"      Mean Pi: {fail_above_floor.Pi.mean():.0f}")
        print(f"    Z-N above ceiling (undertuned after all? ZN>ceil means 228/lat > 380/lat -- impossible):")
        print(f"      Count: {len(fail_above_ceil)}")
        # Z-N above ceiling means kp_ceiling < 228/lat -- this happens at lat=1 if ceiling<228
        if len(fail_above_ceil) > 0:
            print(f"      Pi range: {fail_above_ceil.Pi.min():.0f} - {fail_above_ceil.Pi.max():.0f}")

    # Theoretical Pi threshold for floor crossing
    pi_threshold = 228 / 0.06  # = 3800
    print(f"\n  Theoretical Pi threshold (Z-N safe iff Pi < {pi_threshold:.0f}):")
    print(f"    Designs with Pi < {pi_threshold:.0f}: n={( valid.Pi < pi_threshold).sum()}")
    print(f"    Of those: ZN inside = {valid[valid.Pi < pi_threshold].ZN_inside.sum()} / {(valid.Pi < pi_threshold).sum()}")
    print(f"    Designs with Pi > {pi_threshold:.0f}: n={(valid.Pi >= pi_threshold).sum()}")
    print(f"    Of those: ZN inside = {valid[valid.Pi >= pi_threshold].ZN_inside.sum()} / {(valid.Pi >= pi_threshold).sum()}")

    print(f"\n  By latency group:")
    for lat in sorted(valid.latency_steps.unique()):
        sub = valid[valid.latency_steps == lat]
        print(f"    lat={lat} (Kp_ZN={228/lat:.0f}): "
              f"ZN_inside={sub.ZN_inside.sum()}/{len(sub)}  "
              f"Pi_range=[{sub.Pi.min():.0f},{sub.Pi.max():.0f}]")

    valid.to_csv(RES_DIR / 'reanalysis_E_ZN_py.csv', index=False)
    print(f"\n  Saved: reanalysis_E_ZN_py.csv  (n={n})")
    return valid


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--analysis', choices=['A','B','C','D','E','all'], default='all')
    args = parser.parse_args()

    funcs = {'A': analysis_A, 'B': analysis_B, 'C': analysis_C,
             'D': analysis_D, 'E': analysis_E}

    if args.analysis == 'all':
        for fn in funcs.values():
            fn()
    else:
        funcs[args.analysis]()


if __name__ == '__main__':
    main()
