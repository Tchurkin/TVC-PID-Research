"""
tools/window_ratio_v5_constrained_kd.py  (2026-06-22)

PURPOSE
-------
Clean calibration of floor/ceiling formulas.

v2 artifact: frozen Kd=1.0 inflated floors 65% (all designs had opt_Kd=1.0, the
  grid minimum; true floor opt_Kd is 0.125-0.25 per artifact test).
v4 artifact: existential sweep included Kd=21.5, giving Kd/Kp=215 at low Kp;
  pure-derivative controllers collapsed floors to sweep minimum for 81/120 designs.

FIX: Constrain the Kd/Kp ratio per sweep point.
  For each Kp, test Kd in {0.10, 0.32, 1.0, 3.16} * Kp  UNION  {0.57}.
  0.57 = Z-N universal constant (Kp_ZN*T_u/8, T_u*lat cancel).
  Ratio range [0.1, 3.2] is well within physical operation:
    - At Kp=0.5 (near floor): Kd in {0.05, 0.16, 0.50, 1.58, 0.57} -- all practical
    - At Kp=50 (mid-window):  Kd in {5, 16, 50, 158->clip, 0.57}
  Kd is clipped to [0.01, 32.0].

DESIGN: 5 keff bins x 5 latency bins, 1 design per cell (n=25).
  keff bins: [2,6), [6,10), [10,16), [16,25), [25,45) rad/s^2/CU
  lat bins:  1, 2, 3, 4, 5-6 steps
  Strategy: pick the design closest to cell-median keff within each bin.

SEEDS: 80001-80015 (eval, 15 seeds; disjoint from all prior experiments).
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy.stats import spearmanr
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import cross_val_score


def _ols(X, y):
    """Numpy OLS: returns (coefs, r2). X has no constant; constant added internally."""
    Xc = np.column_stack([np.ones(len(y)), X])
    coef, _, _, _ = np.linalg.lstsq(Xc, y, rcond=None)
    yhat = Xc @ coef
    ss_res = np.sum((y - yhat)**2)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return coef, r2  # coef[0]=intercept, coef[1:]=slopes

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario, sample_lhs)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25

# ---- Sweep parameters ----
KP_SWEEP   = np.geomspace(0.05, 500.0, 28)     # 28 pts, step ~1.37x
KD_RATIOS  = np.array([0.10, 0.32, 1.00, 3.16]) # Kd/Kp ratios (log-spaced over [0.1, 3.2])
KD_ZN      = 0.57                               # Z-N universal constant
KD_CLIP    = (0.01, 32.0)                       # hard bounds on Kd

N_EVAL     = 15
EVAL_SEED0 = 80001
SR_THRESH  = 0.80

OUT_CSV   = Path('experiments/results/window_ratio_v5_py.csv')
OUT_SWEEP = Path('experiments/results/window_ratio_v5_sweep_py.csv')

# ---- Design grid ----
KEFF_BINS = [(2, 6), (6, 10), (10, 16), (16, 25), (25, 45)]
LAT_BINS  = [(1, 1), (2, 2),  (3, 3),   (4, 4),   (5, 6)]


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _keff(row):
    return float(F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy'])


def select_designs(pool):
    """Return list of rocket_ids: 1 per (keff_bin, lat_bin) cell."""
    pool = pool.copy()
    pool['keff_val'] = pool.apply(_keff, axis=1)
    selected = []
    for (klo, khi) in KEFF_BINS:
        for (llo, lhi) in LAT_BINS:
            mask = (
                (pool['keff_val'] >= klo) & (pool['keff_val'] < khi) &
                (pool['latency_steps'] >= llo) & (pool['latency_steps'] <= lhi)
            )
            sub = pool[mask]
            if len(sub) == 0:
                print(f"  WARNING: empty cell keff=[{klo},{khi}) lat=[{llo},{lhi}]")
                continue
            med_keff = (klo + khi) / 2.0
            idx = (sub['keff_val'] - med_keff).abs().idxmin()
            selected.append(sub.loc[idx, 'rocket_id'])
    return selected


def _kd_grid_for_kp(Kp):
    """5 Kd values to test at a given Kp (ratio-scaled + Z-N fixed)."""
    kds = list(KD_RATIOS * Kp) + [KD_ZN]
    kds = np.clip(kds, KD_CLIP[0], KD_CLIP[1])
    return np.unique(kds)


def _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds):
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['success_rate'])


def _eval_design(rocket_id, row):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    seeds  = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    keff   = _keff(row)
    lat    = int(row['latency_steps'])
    Pi     = keff * lat**2
    wind   = float(row.get('wind_strength', np.nan))

    sweep_rows = []
    max_sr_per_kp = {}

    for Kp in KP_SWEEP:
        kd_vals = _kd_grid_for_kp(Kp)
        best_sr, best_kd = -1.0, kd_vals[0]
        for Kd in kd_vals:
            sr = _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds)
            sweep_rows.append(dict(
                rocket_id=rocket_id, keff=keff, lat=lat, Pi=Pi,
                Kp=float(Kp), Kd=float(Kd), Kd_Kp_ratio=float(Kd/Kp), sr=sr
            ))
            if sr > best_sr:
                best_sr, best_kd = sr, float(Kd)
        max_sr_per_kp[float(Kp)] = (best_sr, best_kd)

    # Floor and ceiling
    passing_kp = [kp for kp, (sr, _) in max_sr_per_kp.items() if sr >= SR_THRESH]
    if len(passing_kp) == 0:
        floor = ceil = np.nan
        floor_censored = ceil_censored = True
        opt_kd_floor = opt_kd_ceil = np.nan
    else:
        floor = float(min(passing_kp))
        ceil  = float(max(passing_kp))
        floor_censored = floor <= KP_SWEEP[0] * 1.05
        ceil_censored  = ceil  >= KP_SWEEP[-1] * 0.95
        opt_kd_floor   = max_sr_per_kp[floor][1]
        opt_kd_ceil    = max_sr_per_kp[ceil][1]

    window_ratio = (ceil / floor) if (not floor_censored and not ceil_censored) else np.nan

    return dict(
        rocket_id=rocket_id, keff=keff, lat=lat, Pi=Pi, wind=wind,
        floor=floor, ceil=ceil, window_ratio=window_ratio,
        floor_censored=int(floor_censored), ceil_censored=int(ceil_censored),
        opt_kd_floor=opt_kd_floor, opt_kd_ceil=opt_kd_ceil,
    ), sweep_rows


def run():
    print("window_ratio_v5_constrained_kd.py")
    print(f"  Kd constraint: ratios {list(KD_RATIOS)} * Kp  UNION  KD_ZN={KD_ZN}")
    print(f"  Kd/Kp range: [{KD_RATIOS[0]:.2f}, {KD_RATIOS[-1]:.2f}]  (plus fixed 0.57)")
    print(f"  Kp sweep: {len(KP_SWEEP)} pts [{KP_SWEEP[0]:.3f}, {KP_SWEEP[-1]:.0f}]")
    print(f"  Eval seeds: {EVAL_SEED0} - {EVAL_SEED0 + N_EVAL - 1}  ({N_EVAL} seeds)")

    print("\nLoading 10k pool (seed=8888)...")
    pool = sample_lhs(10000, seed=8888)
    rocket_ids = select_designs(pool)
    print(f"Selected {len(rocket_ids)} designs")

    pool_idx = pool.set_index('rocket_id')
    n_kd_per_kp = len(KD_RATIOS) + 1
    n_sims = len(rocket_ids) * len(KP_SWEEP) * n_kd_per_kp * N_EVAL
    print(f"Est. sims: {n_sims:,}  (~{n_sims/200/3600:.1f} hrs at 200 sim/s)")
    print("Running...")

    results = Parallel(n_jobs=-1)(
        delayed(_eval_design)(rid, pool_idx.loc[rid])
        for rid in rocket_ids
        if rid in pool_idx.index
    )

    summary = [r[0] for r in results]
    sweeps  = [sw for r in results for sw in r[1]]

    df_out   = pd.DataFrame(summary)
    df_sweep = pd.DataFrame(sweeps)
    df_out.to_csv(OUT_CSV, index=False)
    df_sweep.to_csv(OUT_SWEEP, index=False)
    print(f"\nSaved {len(df_out)} rows to {OUT_CSV}")

    analyze(df_out)


def analyze(df=None):
    if df is None:
        df = pd.read_csv(OUT_CSV)

    print("\n" + "=" * 60)
    print("WINDOW RATIO v5 (CONSTRAINED Kd) RESULTS")
    print("=" * 60)

    n_fc = int(df.floor_censored.sum())
    n_cc = int(df.ceil_censored.sum())
    n_v  = int(df.window_ratio.notna().sum())
    print(f"n={len(df)}, valid={n_v}, floor_censored={n_fc}, ceil_censored={n_cc}")

    if n_v > 0:
        wrs = df.window_ratio.dropna()
        print(f"window_ratio: min={wrs.min():.1f}x  p25={wrs.quantile(0.25):.1f}x  "
              f"median={wrs.median():.1f}x  p75={wrs.quantile(0.75):.1f}x  max={wrs.max():.1f}x")

    # Show opt_kd distributions
    print(f"\nopt_Kd at floor: {df.opt_kd_floor.describe().to_dict()}")
    print(f"opt_Kd at ceil:  {df.opt_kd_ceil.describe().to_dict()}")

    # Floor regression
    df_f = df[(df.floor_censored == 0) & df.floor.notna()].copy()
    df_f['log_floor'] = np.log(df_f.floor)
    df_f['log_keff']  = np.log(df_f.keff)
    df_f['log_lat']   = np.log(df_f.lat)
    print(f"\nFloor regression (n={len(df_f)} non-censored):")
    if len(df_f) >= 5:
        X = df_f[['log_keff', 'log_lat']].values
        y = df_f['log_floor'].values
        coef, r2 = _ols(X, y)
        C = np.exp(coef[0])
        print(f"  floor ~ keff+lat: R2={r2:.3f}")
        print(f"  coefs: keff={coef[1]:+.3f}  lat={coef[2]:+.3f}  const={coef[0]:+.3f}")
        print(f"  Implied: floor ~ {C:.4f} * keff^{coef[1]:.2f} * lat^{coef[2]:.2f}")
        print(f"  v2 (biased Kd=1.0): 0.0600 * keff^1.06 * lat^0.96")
        print(f"  artifact-test pred: 0.0210 * keff^1.06 * lat^0.96  (0.35 * v2)")

    # Ceiling regression
    df_c = df[(df.ceil_censored == 0) & df.ceil.notna()].copy()
    df_c['log_ceil'] = np.log(df_c.ceil)
    df_c['log_keff'] = np.log(df_c.keff)
    df_c['log_lat']  = np.log(df_c.lat)
    print(f"\nCeiling regression (n={len(df_c)} non-censored):")
    if len(df_c) >= 5:
        X = df_c[['log_keff', 'log_lat']].values
        y = df_c['log_ceil'].values
        coef_c, r2_c = _ols(X, y)
        print(f"  ceil ~ keff+lat: R2={r2_c:.3f}")
        print(f"  coefs: keff={coef_c[1]:+.3f}  lat={coef_c[2]:+.3f}")
        print(f"  Theory: keff~0, lat~-1.0  (ceiling=380/lat, keff-independent)")

    # Window regression
    df_v = df[df.window_ratio.notna()].copy()
    df_v['log_window'] = np.log(df_v.window_ratio)
    df_v['log_keff']   = np.log(df_v.keff)
    df_v['log_lat']    = np.log(df_v.lat)
    df_v['log_Pi']     = np.log(df_v.Pi)
    print(f"\nWindow regression (n={len(df_v)} non-censored):")
    if len(df_v) >= 5:
        X_sk = df_v[['log_keff', 'log_lat']].values
        y_sk = df_v['log_window'].values
        lr   = LinearRegression().fit(X_sk, y_sk)
        cv_k = min(5, len(df_v) // 2)
        cv   = cross_val_score(lr, X_sk, y_sk, cv=cv_k, scoring='r2')
        coef_w, r2_w = _ols(X_sk, y_sk)
        C_w = np.exp(coef_w[0])
        print(f"  log_keff+log_lat: R2={r2_w:.3f}  CV={cv.mean():.3f}+/-{cv.std():.3f}")
        print(f"  coefs: keff={coef_w[1]:+.3f}  lat={coef_w[2]:+.3f}")
        print(f"  Implied: window ~ {C_w:.0f} * keff^{coef_w[1]:.2f} * lat^{coef_w[2]:.2f}")
        print(f"  v2 was: 8700 * keff^-1.19 * lat^-1.88  (CV=0.616)")

        coef_pi, r2_pi = _ols(df_v[['log_Pi']].values, y_sk)
        C_pi = np.exp(coef_pi[0])
        print(f"\n  Pi single:  R2={r2_pi:.3f}  coef={coef_pi[1]:+.3f}")
        print(f"  Implied: window ~ {C_pi:.0f} / Pi^{-coef_pi[1]:.2f}")
        print(f"  Theory: window ~ 18100 / Pi^1.0  (artifact-test corrected)")

    # C constant
    df_v2 = df[df.window_ratio.notna()].copy()
    df_v2['C'] = df_v2.window_ratio * df_v2.Pi
    print(f"\nC = window * Pi:  median={df_v2.C.median():.0f}  "
          f"p25={df_v2.C.quantile(0.25):.0f}  p75={df_v2.C.quantile(0.75):.0f}")
    print(f"  Theory (ceiling/floor formulas): C = 380/0.021 = {380/0.021:.0f}")
    print(f"  v2 empirical: C ~ 9391")

    # Correlation summary
    print("\nCorrelations with log(window_ratio):")
    for col, label in [('keff', 'log_keff'), ('lat', 'log_lat'), ('Pi', 'log_Pi')]:
        sub = df_v[df_v.window_ratio.notna() & df_v[col].notna()]
        if len(sub) < 5:
            continue
        rho, p = spearmanr(np.log(sub[col]), sub['log_window'])
        print(f"  rho({label}, log_window) = {rho:+.3f}  p={p:.2e}")


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        analyze()
    else:
        run()
