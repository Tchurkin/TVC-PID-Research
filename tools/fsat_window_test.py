"""
tools/fsat_window_test.py  (2026-06-22)

RESEARCH QUESTION: Is slew saturation fraction (f_sat) the hidden state variable
underlying the Pi constraint?

HYPOTHESIS: f_sat measured at kp_floor (not at optimal Kp) mediates the relationship
between Pi and window_ratio. If true:
  1. Pi → f_sat (Pi predicts saturation level at floor)
  2. f_sat → window_ratio (saturation at floor predicts window width)
  3. After conditioning on f_sat, Pi adds no further predictive power

DESIGN: Take the 86 non-censored designs from window_ratio_v2 (LHS seed=8888).
For each design, measure slew_sat_frac at three reference Kp values:
  - kp_floor: the measured floor (where SR just crosses 0.80 going up)
  - kp_ceiling: the measured ceiling (where SR just falls below 0.80 going down)
  - kp_mid: geometric mean sqrt(kp_floor * kp_ceiling)
Use opt_Kd from window_ratio_v2 (frozen, same as in the sweep that defined floor/ceiling).
Evaluate with 20 fresh seeds (40001-40020, disjoint from all prior).

OUTPUT: experiments/results/fsat_window_test_py.csv
  columns: rocket_id, keff, latency, Pi, window_ratio, kp_floor, kp_ceiling,
            opt_Kd, fsat_floor, fsat_mid, fsat_ceil, usat_floor, usat_mid, usat_ceil

ANALYSIS: Mediation test
  R2(window ~ Pi) vs R2(window ~ fsat_floor) vs R2(window ~ Pi + fsat_floor)
  If R2(window ~ fsat_floor) > R2(window ~ Pi): f_sat is a better predictor
  If adding Pi to fsat model gives no R2 gain: f_sat mediates completely
  If Pi adds R2 after fsat: Pi captures physics beyond saturation

Also tests: f_sat at mid and ceil as alternatives to floor.
Connects to: Smith predictor regime (high f_sat -> Smith should fail)
             ADRC advantage (high f_sat -> larger ADRC-PID gap expected)

Seeds: eval 40001-40020 (disjoint from exp1 1-3, window_ratio v1 20001-20015,
       v2 22001-22015, v3 23001-23020, regression_extension 8001-8015,
       all prior experiments)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy.stats import pearsonr, spearmanr
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import cross_val_score

from design_space import (
    build_plant, build_actuator, build_sensor,
    build_disturbance, build_scenario, sample_lhs,
)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

# ── Constants ──────────────────────────────────────────────────────────────────
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

N_EVAL_SEEDS = 20
EVAL_SEED0   = 40001          # disjoint from all prior experiments

POOL_SEED = 8888              # same LHS pool used by window_ratio_regression.py
N_POOL    = 10_000

OUT_CSV = Path('experiments/results/fsat_window_test_py.csv')


def _physics_cfg() -> FidelityConfig:
    """Full physics - identical to window_ratio_regression.py"""
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _measure_fsat(plant, act, sen, dis, sc, Kp, Kd, seeds):
    """Run N_EVAL_SEEDS at (Kp, Kd), return mean slew_sat_frac and u_cmd_sat_frac."""
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['slew_sat_frac']), float(agg['u_cmd_sat_frac'])


def _process_design(row_dict, pool_row):
    """Measure f_sat at floor, mid, ceiling for one design."""
    rocket_id   = row_dict['rocket_id']
    kp_floor    = row_dict['kp_floor']
    kp_ceiling  = row_dict['kp_ceiling']
    kp_mid      = float(np.sqrt(kp_floor * kp_ceiling))
    opt_Kd      = row_dict['opt_Kd']
    window_ratio = row_dict['window_ratio']

    T_avg = F15_AVG_THRUST_N * pool_row['motor_scale']
    keff  = T_avg * CU_TO_RAD * L_NOZZLE / pool_row['Iyy']
    lat   = int(pool_row['latency_steps'])
    Pi    = float(keff * lat**2)

    plant = build_plant(pool_row)
    act   = build_actuator(pool_row)
    sen   = build_sensor(pool_row)
    dis   = build_disturbance(pool_row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL_SEEDS))

    fsat_floor, usat_floor = _measure_fsat(plant, act, sen, dis, sc, kp_floor,   opt_Kd, seeds)
    fsat_mid,   usat_mid   = _measure_fsat(plant, act, sen, dis, sc, kp_mid,     opt_Kd, seeds)
    fsat_ceil,  usat_ceil  = _measure_fsat(plant, act, sen, dis, sc, kp_ceiling, opt_Kd, seeds)

    return dict(
        rocket_id    = rocket_id,
        keff         = float(keff),
        latency      = lat,
        Pi           = Pi,
        window_ratio = float(window_ratio),
        kp_floor     = float(kp_floor),
        kp_ceiling   = float(kp_ceiling),
        kp_mid       = float(kp_mid),
        opt_Kd       = float(opt_Kd),
        fsat_floor   = fsat_floor,
        fsat_mid     = fsat_mid,
        fsat_ceil    = fsat_ceil,
        usat_floor   = usat_floor,
        usat_mid     = usat_mid,
        usat_ceil    = usat_ceil,
    )


def run():
    print("fsat_window_test.py: Measuring f_sat at floor/mid/ceiling for window_ratio designs")
    print(f"  Eval seeds: {EVAL_SEED0} - {EVAL_SEED0 + N_EVAL_SEEDS - 1}")

    # Load window_ratio_v2 non-censored designs
    v2 = pd.read_csv('experiments/results/window_ratio_v2_py.csv')
    valid = v2[(v2['floor_censored'] == 0) & (v2['ceil_censored'] == 0)].copy()
    print(f"  Non-censored designs: n={len(valid)}")

    # Reload the LHS pool that was used in window_ratio_regression.py
    # sample_lhs already assigns rocket_id = R0001..R{N_POOL} (1-indexed)
    print(f"  Loading LHS pool (seed={POOL_SEED}, n={N_POOL})...")
    pool_df = sample_lhs(N_POOL, seed=POOL_SEED)

    merged = valid.merge(pool_df, on='rocket_id', how='inner')
    print(f"  Designs matched to pool: n={len(merged)}")
    if len(merged) < len(valid):
        print(f"  WARNING: {len(valid) - len(merged)} designs not found in pool")

    print(f"  Running {len(merged)} designs x 3 Kp x {N_EVAL_SEEDS} seeds = "
          f"{len(merged) * 3 * N_EVAL_SEEDS} simulations...")

    # Run in parallel
    rows = merged.to_dict('records')
    pool_rows = {r['rocket_id']: r for r in pool_df.to_dict('records')}

    def process_one(row):
        try:
            return _process_design(row, pool_rows[row['rocket_id']])
        except Exception as e:
            print(f"  ERROR {row['rocket_id']}: {e}")
            return None

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(process_one)(row) for row in rows
    )
    results = [r for r in results if r is not None]

    df_out = pd.DataFrame(results)
    df_out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(df_out)} rows to {OUT_CSV}")

    # ── Immediate analysis ─────────────────────────────────────────────────────
    analyze(df_out)


def analyze(df=None):
    if df is None:
        df = pd.read_csv(OUT_CSV)

    df = df.dropna(subset=['fsat_floor', 'fsat_mid', 'fsat_ceil', 'window_ratio', 'Pi'])
    df['log_window'] = np.log(df['window_ratio'])
    df['log_Pi']     = np.log(df['Pi'])
    df['log_fsat_floor'] = np.log(df['fsat_floor'].clip(1e-4))
    df['log_fsat_mid']   = np.log(df['fsat_mid'].clip(1e-4))
    df['log_fsat_ceil']  = np.log(df['fsat_ceil'].clip(1e-4))

    n = len(df)
    print(f"\n=== f_sat MEDIATION ANALYSIS (n={n}) ===")

    print("\n--- Spearman correlations with log(window_ratio) ---")
    for col, label in [
        ('log_Pi',         'log(Pi)'),
        ('fsat_floor',     'f_sat @ floor'),
        ('fsat_mid',       'f_sat @ mid'),
        ('fsat_ceil',      'f_sat @ ceiling'),
        ('log_fsat_floor', 'log(f_sat @ floor)'),
        ('log_fsat_mid',   'log(f_sat @ mid)'),
        ('keff',           'keff'),
        ('latency',        'latency'),
    ]:
        rho, p = spearmanr(df[col], df['log_window'])
        print(f"  {label:25s}  rho={rho:+.3f}  p={p:.2e}")

    print("\n--- OLS R2 for log(window_ratio) ---")
    def cv_r2(X_cols):
        X = df[X_cols].values
        y = df['log_window'].values
        m = LinearRegression().fit(X, y)
        r2 = m.score(X, y)
        cv = cross_val_score(m, X, y, cv=5, scoring='r2').mean()
        coefs = dict(zip(X_cols, m.coef_))
        return r2, cv, coefs

    for X_cols, label in [
        (['log_Pi'],                         'log_Pi alone'),
        (['log_fsat_floor'],                 'log_fsat_floor alone'),
        (['log_fsat_mid'],                   'log_fsat_mid alone'),
        (['log_Pi', 'log_fsat_floor'],       'log_Pi + log_fsat_floor'),
        (['log_Pi', 'log_fsat_mid'],         'log_Pi + log_fsat_mid'),
        (['keff', 'latency'],                'keff + latency (raw)'),
    ]:
        r2, cv, coefs = cv_r2(X_cols)
        print(f"  {label:35s}  R2={r2:.3f}  CV_R2={cv:.3f}  coefs={coefs}")

    print("\n--- Mediation step 1: does Pi predict f_sat? ---")
    for col, label in [('fsat_floor','f_sat_floor'), ('fsat_mid','f_sat_mid'), ('fsat_ceil','f_sat_ceil')]:
        r, p = pearsonr(df['log_Pi'], np.log(df[col].clip(1e-4)))
        rho, prho = spearmanr(df['Pi'], df[col])
        print(f"  r(log_Pi, log_{label}) = {r:.3f} p={p:.2e}  |  rho={rho:.3f} p={prho:.2e}")

    print("\n--- f_sat at floor vs ceiling vs mid: descriptive statistics ---")
    for col in ['fsat_floor', 'fsat_mid', 'fsat_ceil']:
        print(f"  {col}: mean={df[col].mean():.3f}  median={df[col].median():.3f}  "
              f"  q10={df[col].quantile(0.1):.3f}  q90={df[col].quantile(0.9):.3f}")

    # Conditional independence test: partial correlation
    print("\n--- Partial correlation (key mediation test) ---")
    # r(fsat_floor, window | Pi): if near 0, Pi explains everything fsat captures
    from scipy.stats import pearsonr as pr
    # Residualize window on Pi, then correlate with fsat
    m_pi = LinearRegression().fit(df[['log_Pi']], df['log_window'])
    resid_w = df['log_window'] - m_pi.predict(df[['log_Pi']])
    r, p = pr(np.log(df['fsat_floor'].clip(1e-4)), resid_w)
    print(f"  r(log_fsat_floor, window | Pi) = {r:.3f}  p={p:.3f}")
    r2, p2 = pr(np.log(df['fsat_mid'].clip(1e-4)), resid_w)
    print(f"  r(log_fsat_mid,   window | Pi) = {r2:.3f}  p={p2:.3f}")

    # Residualize window on fsat, then correlate with Pi
    m_fsat = LinearRegression().fit(np.log(df[['fsat_floor']].clip(1e-4)), df['log_window'])
    resid_w2 = df['log_window'] - m_fsat.predict(np.log(df[['fsat_floor']].clip(1e-4)))
    r3, p3 = pr(df['log_Pi'], resid_w2)
    print(f"  r(log_Pi, window | fsat_floor) = {r3:.3f}  p={p3:.3f}")
    print()
    if abs(r) < 0.2 and abs(r2) < 0.2 and abs(r3) > 0.3:
        print("  CONCLUSION: f_sat does NOT mediate Pi->window; Pi is the primary variable")
    elif abs(r3) < 0.2 and abs(r) > 0.3:
        print("  CONCLUSION: f_sat IS the mediating variable; Pi operates through f_sat")
    else:
        print("  CONCLUSION: Mixed/partial mediation; both Pi and f_sat capture independent variance")


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        analyze()
    else:
        run()
