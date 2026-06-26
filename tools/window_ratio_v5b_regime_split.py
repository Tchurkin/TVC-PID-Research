"""
tools/window_ratio_v5b_regime_split.py  (2026-06-22)

PURPOSE
-------
Determine whether the floor law (Kp_floor ~ keff * lat) is universal or
regime-specific. v5 revealed that wind dominates floor measurements when
designs span the full LHS wind range (rho=+0.741, p=0.014). This experiment
controls wind to isolate the authority/latency signal.

KEY DESIGN CHOICES
------------------
1. FIXED wind_strength = 0.25 for all designs (wind as control variable, not covariate).
   Physical rationale: holding temperature constant to study P-V scaling.

2. Pi-stratified design selection, biased toward the transition region.
   Pi bins: [50,100], [100,250], [250,600], [600,1200], [1200,2500]
   5 designs per bin, varying latency within each bin.
   This directly stratifies on the window law's primary predictor.

3. Constrained Kd/Kp (v5 design): ratios {0.10, 0.32, 1.0, 3.16} * Kp + {0.57}.
   Prevents the v4 extreme-Kd artifact (Kd/Kp=215 at low Kp).

4. Kp sweep [0.02, 600], 30 pts. Kp_min=0.02 (not 0.005 — designs below this
   floor are "always controllable" and contribute little floor-exponent information).

5. Record fsat_floor at the measured floor Kp: allows post-hoc regime split.
   Split criterion: fsat_floor < 0.2 = linear, 0.2-0.5 = transitional, > 0.5 = bang-bang.

HYPOTHESIS
----------
Linear regime (fsat_floor < 0.2):  floor ~ wind/keff  (negative keff exponent)
Bang-bang regime (fsat_floor > 0.5): floor ~ keff * lat  (positive exponents, ~1.0 each)

SEEDS: 85001-85015 (eval), 85016-85018 (fsat measurement). All disjoint from prior.
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

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario, sample_lhs)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD  = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE   = 0.25

# ---- Experiment parameters ----
FIXED_WIND = 0.25          # constant across all designs
KP_SWEEP   = np.geomspace(0.02, 600.0, 30)   # 30 pts, step ~1.40x
KD_RATIOS  = np.array([0.10, 0.32, 1.00, 3.16])
KD_ZN      = 0.57
KD_CLIP    = (0.01, 32.0)

N_EVAL       = 15
EVAL_SEED0   = 85001
N_FSAT_SEEDS = 3
FSAT_SEED0   = 85016
SR_THRESH    = 0.80

OUT_CSV    = Path('experiments/results/window_ratio_v5b_py.csv')
OUT_SWEEP  = Path('experiments/results/window_ratio_v5b_sweep_py.csv')

# ---- Pi stratification ----
# 5 bins x 5 designs each = 25 total
# Within each bin, target latency spread [1,2,3,4,6] where possible
PI_BINS  = [(50, 100), (100, 250), (250, 600), (600, 1200), (1200, 2500)]
LAT_TARGETS = [1, 2, 3, 4, 6]   # one per design slot within each bin


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _keff(row):
    return float(F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy'])


def _kd_grid_for_kp(Kp):
    kds = list(KD_RATIOS * Kp) + [KD_ZN]
    return np.unique(np.clip(kds, KD_CLIP[0], KD_CLIP[1]))


def select_designs(pool):
    """
    For each Pi bin, pick 5 designs with latencies closest to LAT_TARGETS.
    Returns list of (rocket_id, pi_bin_label) tuples.
    """
    pool = pool.copy()
    pool['keff_val'] = pool.apply(_keff, axis=1)
    pool['Pi_val']   = pool['keff_val'] * pool['latency_steps']**2
    selected = []
    for (plo, phi) in PI_BINS:
        sub = pool[(pool['Pi_val'] >= plo) & (pool['Pi_val'] < phi)].copy()
        if len(sub) < 5:
            print(f"  WARNING: only {len(sub)} designs in Pi=[{plo},{phi})")
        for lat_target in LAT_TARGETS:
            # Pick design with latency closest to lat_target, not already selected
            used = {rid for rid, _ in selected}
            cands = sub[~sub['rocket_id'].isin(used)].copy()
            if len(cands) == 0:
                print(f"  WARNING: no candidates left for Pi=[{plo},{phi}), lat~{lat_target}")
                continue
            cands['lat_dist'] = (cands['latency_steps'] - lat_target).abs()
            # Among those with closest lat, pick closest to bin-midpoint Pi
            min_lat_dist = cands['lat_dist'].min()
            cands2 = cands[cands['lat_dist'] == min_lat_dist]
            pi_mid = (plo + phi) / 2.0
            idx = (cands2['Pi_val'] - pi_mid).abs().idxmin()
            selected.append((sub.loc[idx, 'rocket_id'], f"Pi{plo}-{phi}"))
    return selected


def _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds):
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['success_rate'])


def _eval_fsat(plant, act, sen, dis, sc, Kp, Kd, seeds):
    """Measure slew saturation fraction and u_cmd saturation fraction."""
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['slew_sat_frac']), float(agg['u_cmd_sat_frac'])


def _eval_design(rocket_id, pi_bin, row):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    # Override wind to fixed value (DisturbanceParams is a mutable dataclass)
    dis.gust_std = FIXED_WIND

    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    seeds_eval  = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    seeds_fsat  = list(range(FSAT_SEED0, FSAT_SEED0 + N_FSAT_SEEDS))

    keff = _keff(row)
    lat  = int(row['latency_steps'])
    Pi   = keff * lat**2

    # --- Kp sweep with constrained Kd ---
    sweep_rows = []
    max_sr_per_kp  = {}
    best_kd_per_kp = {}

    for Kp in KP_SWEEP:
        kd_vals = _kd_grid_for_kp(Kp)
        best_sr, best_kd = -1.0, kd_vals[0]
        for Kd in kd_vals:
            sr = _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds_eval)
            sweep_rows.append(dict(
                rocket_id=rocket_id, pi_bin=pi_bin, keff=keff, lat=lat, Pi=Pi,
                Kp=float(Kp), Kd=float(Kd), Kd_Kp_ratio=float(Kd/Kp), sr=sr
            ))
            if sr > best_sr:
                best_sr, best_kd = sr, float(Kd)
        max_sr_per_kp[float(Kp)]  = best_sr
        best_kd_per_kp[float(Kp)] = best_kd

    # --- Floor / ceiling ---
    passing_kp = [kp for kp, sr in max_sr_per_kp.items() if sr >= SR_THRESH]
    if len(passing_kp) == 0:
        floor = ceil = np.nan
        floor_censored = ceil_censored = True
        opt_kd_floor = opt_kd_ceil = np.nan
    else:
        floor = float(min(passing_kp))
        ceil  = float(max(passing_kp))
        floor_censored = floor <= KP_SWEEP[0] * 1.05
        ceil_censored  = ceil  >= KP_SWEEP[-1] * 0.95
        opt_kd_floor   = best_kd_per_kp[floor]
        opt_kd_ceil    = best_kd_per_kp[ceil]

    window_ratio = (ceil / floor) if (not floor_censored and not ceil_censored) else np.nan

    # --- fsat at floor (only if floor not censored) ---
    fsat_floor = usat_floor = np.nan
    if not floor_censored and not np.isnan(floor):
        fsat_floor, usat_floor = _eval_fsat(
            plant, act, sen, dis, sc, floor, opt_kd_floor, seeds_fsat
        )
        # regime label
    if not np.isnan(fsat_floor):
        if fsat_floor < 0.20:
            regime = 'linear'
        elif fsat_floor < 0.50:
            regime = 'transitional'
        else:
            regime = 'bang-bang'
    else:
        regime = 'censored'

    return dict(
        rocket_id=rocket_id, pi_bin=pi_bin, keff=keff, lat=lat, Pi=Pi,
        floor=floor, ceil=ceil, window_ratio=window_ratio,
        floor_censored=int(floor_censored), ceil_censored=int(ceil_censored),
        opt_kd_floor=opt_kd_floor, opt_kd_ceil=opt_kd_ceil,
        fsat_floor=fsat_floor, usat_floor=usat_floor, regime=regime,
    ), sweep_rows


def _ols(X, y):
    Xc = np.column_stack([np.ones(len(y)), X])
    coef, _, _, _ = np.linalg.lstsq(Xc, y, rcond=None)
    yhat = Xc @ coef
    ss_res = np.sum((y - yhat)**2)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return coef, r2


def run():
    print("window_ratio_v5b_regime_split.py")
    print(f"  Fixed wind = {FIXED_WIND}")
    print(f"  Kd constraint: ratios {list(KD_RATIOS)} * Kp  + KD_ZN={KD_ZN}")
    print(f"  Kp sweep: {len(KP_SWEEP)} pts [{KP_SWEEP[0]:.3f}, {KP_SWEEP[-1]:.0f}]")
    print(f"  Eval seeds: {EVAL_SEED0}–{EVAL_SEED0+N_EVAL-1}  fsat seeds: {FSAT_SEED0}–{FSAT_SEED0+N_FSAT_SEEDS-1}")

    print("\nLoading 10k pool (seed=8888)...")
    pool = sample_lhs(10000, seed=8888)
    design_list = select_designs(pool)
    print(f"Selected {len(design_list)} designs across {len(PI_BINS)} Pi bins")
    for pid, pbin in design_list:
        row = pool.set_index('rocket_id').loc[pid]
        keff_v = _keff(row)
        lat_v  = int(row['latency_steps'])
        print(f"  {pid}: keff={keff_v:.1f}  lat={lat_v}  Pi={keff_v*lat_v**2:.0f}  wind_orig={row['wind_strength']:.2f}  -> {pbin}")

    pool_idx = pool.set_index('rocket_id')
    n_kd_per_kp = len(KD_RATIOS) + 1
    n_sims = len(design_list) * len(KP_SWEEP) * n_kd_per_kp * N_EVAL
    print(f"\nEst. sims (sweep only): {n_sims:,}")
    print("Running...")

    results = Parallel(n_jobs=-1)(
        delayed(_eval_design)(rid, pbin, pool_idx.loc[rid])
        for rid, pbin in design_list
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

    print("\n" + "=" * 65)
    print("WINDOW RATIO v5b (FIXED WIND, REGIME SPLIT) RESULTS")
    print("=" * 65)

    n_fc = int(df.floor_censored.sum())
    n_cc = int(df.ceil_censored.sum())
    n_v  = int(df.window_ratio.notna().sum())
    print(f"n={len(df)}, valid_windows={n_v}, floor_censored={n_fc}, ceil_censored={n_cc}")

    # Design coverage
    print(f"\nPi bin coverage:")
    for pbin in df['pi_bin'].unique() if 'pi_bin' in df.columns else []:
        sub = df[df['pi_bin'] == pbin]
        print(f"  {pbin}: n={len(sub)}  floors_uncensored={int((sub.floor_censored==0).sum())}")

    # Regime distribution
    if 'regime' in df.columns:
        print(f"\nRegime distribution (at floor):")
        print(df['regime'].value_counts().to_string())
        print(f"  fsat_floor: min={df.fsat_floor.min():.2f}  median={df.fsat_floor.median():.2f}  max={df.fsat_floor.max():.2f}")

    # ---- Floor regression: all uncensored ----
    df_f = df[(df.floor_censored == 0) & df.floor.notna() & (df.floor > 0)].copy()
    df_f['log_floor'] = np.log(df_f.floor)
    df_f['log_keff']  = np.log(df_f.keff)
    df_f['log_lat']   = np.log(df_f.lat)
    df_f['log_Pi']    = np.log(df_f.Pi)

    print(f"\n--- Floor regression: ALL uncensored (n={len(df_f)}) ---")
    if len(df_f) >= 4:
        coef, r2 = _ols(df_f[['log_keff','log_lat']].values, df_f['log_floor'].values)
        C = np.exp(coef[0])
        print(f"  floor ~ keff+lat: R2={r2:.3f}  keff={coef[1]:+.3f}  lat={coef[2]:+.3f}")
        print(f"  Implied: floor ~ {C:.4f} * keff^{coef[1]:.2f} * lat^{coef[2]:.2f}")
        coef_pi, r2_pi = _ols(df_f[['log_Pi']].values, df_f['log_floor'].values)
        print(f"  floor ~ Pi:       R2={r2_pi:.3f}  Pi={coef_pi[1]:+.3f}")
        print(f"  v2 pred: 0.0600 * keff^1.06 * lat^0.96")
        print(f"  artifact-corrected pred: 0.0210 * keff^1.06 * lat^0.96")

    # ---- Floor regression: by regime ----
    if 'regime' in df.columns:
        for regime_label, fsat_crit in [('bang-bang (fsat>0.50)', 0.50),
                                         ('transitional (0.20-0.50)', -1),
                                         ('linear (fsat<0.20)', -99)]:
            if fsat_crit == 0.50:
                sub = df_f[df_f.fsat_floor > fsat_crit]
            elif fsat_crit == -1:
                sub = df_f[(df_f.fsat_floor >= 0.20) & (df_f.fsat_floor <= 0.50)]
            else:
                sub = df_f[df_f.fsat_floor < 0.20]

            print(f"\n--- Floor regression: {regime_label} (n={len(sub)}) ---")
            if len(sub) < 4:
                print(f"  Too few designs for regression.")
                if len(sub) > 0:
                    print(sub[['rocket_id','keff','lat','Pi','floor','fsat_floor']].to_string())
                continue
            coef_r, r2_r = _ols(sub[['log_keff','log_lat']].values, sub['log_floor'].values)
            C_r = np.exp(coef_r[0])
            print(f"  floor ~ keff+lat: R2={r2_r:.3f}  keff={coef_r[1]:+.3f}  lat={coef_r[2]:+.3f}")
            print(f"  Implied: floor ~ {C_r:.4f} * keff^{coef_r[1]:.2f} * lat^{coef_r[2]:.2f}")

    # ---- Ceiling regression ----
    df_c = df[(df.ceil_censored == 0) & df.ceil.notna() & (df.ceil > 0)].copy()
    df_c['log_ceil'] = np.log(df_c.ceil)
    df_c['log_keff'] = np.log(df_c.keff)
    df_c['log_lat']  = np.log(df_c.lat)
    print(f"\n--- Ceiling regression (n={len(df_c)} uncensored) ---")
    if len(df_c) >= 4:
        coef_c, r2_c = _ols(df_c[['log_keff','log_lat']].values, df_c['log_ceil'].values)
        print(f"  ceil ~ keff+lat: R2={r2_c:.3f}  keff={coef_c[1]:+.3f}  lat={coef_c[2]:+.3f}")
        print(f"  Theory: keff~0, lat~-1.0  (ceiling = 380/lat)")

    # ---- Window regression ----
    df_v = df[df.window_ratio.notna()].copy()
    df_v['log_window'] = np.log(df_v.window_ratio)
    df_v['log_keff']   = np.log(df_v.keff)
    df_v['log_lat']    = np.log(df_v.lat)
    df_v['log_Pi']     = np.log(df_v.Pi)
    print(f"\n--- Window regression (n={len(df_v)} both uncensored) ---")
    if len(df_v) >= 4:
        X_sk = df_v[['log_keff','log_lat']].values
        y_sk = df_v['log_window'].values
        coef_w, r2_w = _ols(X_sk, y_sk)
        C_w = np.exp(coef_w[0])
        cv_k = min(5, max(2, len(df_v)//2))
        if len(df_v) >= 4:
            cv = cross_val_score(LinearRegression(), X_sk, y_sk,
                                  cv=cv_k, scoring='r2')
            print(f"  log_keff+log_lat: R2={r2_w:.3f}  CV={cv.mean():.3f}+/-{cv.std():.3f}")
        print(f"  coefs: keff={coef_w[1]:+.3f}  lat={coef_w[2]:+.3f}")
        print(f"  Implied: window ~ {C_w:.0f} * keff^{coef_w[1]:.2f} * lat^{coef_w[2]:.2f}")
        coef_pi, r2_pi = _ols(df_v[['log_Pi']].values, y_sk)
        C_pi = np.exp(coef_pi[0])
        print(f"\n  Pi single: R2={r2_pi:.3f}  coef={coef_pi[1]:+.3f}")
        print(f"  Implied: window ~ {C_pi:.0f} / Pi^{-coef_pi[1]:.2f}")

    # ---- Design list for reference ----
    print("\n--- Design table ---")
    cols = ['rocket_id','pi_bin','keff','lat','Pi','floor','ceil','window_ratio',
            'floor_censored','ceil_censored','fsat_floor','regime']
    show_cols = [c for c in cols if c in df.columns]
    print(df[show_cols].sort_values('Pi').to_string(index=False))


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        analyze()
    else:
        run()
