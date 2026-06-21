"""
tools/window_ratio_regression.py

Measures GAIN WINDOW RATIO (Kp_ceiling / Kp_floor) directly for a
(theta_ddot × latency)-stratified sample, then regresses log(window_ratio)
on hardware specs.

The over_sr-based continuous regression (n=262, CV R²=0.33) has two
structural weaknesses:
  1. over_sr is right-censored at 1.0 — easy designs all score 0.98-1.00
     regardless of how wide their window actually is.
  2. It measures "probability at 1.4x gains," not the actual gain margin
     width the ceiling/floor theory predicts.

Window_ratio = Kp_ceiling / Kp_floor directly measures the gain margin width.
Theory predicts: log(window) ≈ const - 0.90*log(keff) - 1.10*log(latency).
Expected R²: 0.55–0.75 vs. current 0.33.

Method:
  - 10,000-design LHS pool (seed=8888)
  - Stratify into 6 td bins × 4 latency bins, 5 designs per cell (~120 total)
  - Per design: finer joint Kp×Kd search (18×7=126, seeds 1-3)
    then Kp sweep (16 log-spaced [0.5, 480], Kd frozen, seeds 20001-20015)
  - floor = min passing Kp (SR≥0.80), ceiling = max passing Kp
  - window_ratio = ceiling / floor
  - Regress log(window_ratio) on log_td, log_latency, log_keff, and extras
  - 5-fold CV

Seed isolation: sweep uses 20001-20015 (disjoint from all prior experiments)
Output: experiments/results/window_ratio_regression_py.csv
        experiments/results/window_ratio_sweep_detail_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy.stats import pearsonr, spearmanr

from design_space import (
    build_plant, build_actuator, build_sensor,
    build_disturbance, build_scenario, sample_lhs,
)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

# ── Constants ─────────────────────────────────────────────────────────────────
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

KP_SRCH_MIN, KP_SRCH_MAX, N_KP_SRCH = 1.0, 320.0, 18
KD_SRCH_MIN, KD_SRCH_MAX, N_KD_SRCH = 1.0, 64.0,  7
SEARCH_SEEDS = [1, 2, 3]

KP_SWEEP      = np.geomspace(0.5, 480.0, 16)   # 0.5→480, ~1.65× per step
N_SWEEP_SEEDS = 15
SWEEP_SEED0   = 20001                            # disjoint from all prior experiments
SR_THRESH     = 0.80

N_POOL     = 10_000
POOL_SEED  = 8888
N_PER_CELL = 5

TD_BINS   = [0,   40,  80, 120, 180, 300, 3000]
TD_LABELS = ['td0-40', 'td40-80', 'td80-120', 'td120-180', 'td180-300', 'td300+']
LAT_BINS  = [0, 1, 3, 4, 6]           # right-closed: {1}, {2,3}, {4}, {5,6}
LAT_LABELS = ['lat1', 'lat2-3', 'lat4', 'lat5-6']

OUT_CSV       = Path('experiments/results/window_ratio_regression_py.csv')
OUT_SWEEP_CSV = Path('experiments/results/window_ratio_sweep_detail_py.csv')


# ── Physics configuration ─────────────────────────────────────────────────────
def _physics_cfg() -> FidelityConfig:
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _compute_specs(row: dict):
    T_avg = F15_AVG_THRUST_N * row['motor_scale']
    keff  = T_avg * CU_TO_RAD * L_NOZZLE / row['Iyy']
    u_max = row['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff * u_max), float(keff), float(u_max)


# ── Gain search ───────────────────────────────────────────────────────────────
def _joint_search(plant, act, sen, dis, sc):
    kp_grid = np.geomspace(KP_SRCH_MIN, KP_SRCH_MAX, N_KP_SRCH)
    kd_grid = np.geomspace(KD_SRCH_MIN, KD_SRCH_MAX, N_KD_SRCH)
    best_kp, best_kd, best_sr, best_rms = kp_grid[0], kd_grid[0], -1.0, 1e9
    for Kp in kp_grid:
        for Kd in kd_grid:
            pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                            u_max=act.u_max, i_lim=act.u_max)
            agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s)
                               for s in SEARCH_SEEDS])
            if (agg['success_rate'] > best_sr or
                    (agg['success_rate'] == best_sr and
                     agg['rms_error_deg'] < best_rms)):
                best_kp, best_kd = float(Kp), float(Kd)
                best_sr, best_rms = agg['success_rate'], agg['rms_error_deg']
    return best_kp, best_kd


# ── Per-design evaluation ─────────────────────────────────────────────────────
def _eval_design(row: dict):
    td, keff, u_max = _compute_specs(row)

    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    opt_Kp, opt_Kd = _joint_search(plant, act, sen, dis, sc)

    sweep_seeds = list(range(SWEEP_SEED0, SWEEP_SEED0 + N_SWEEP_SEEDS))
    sr_vals    = []
    sweep_rows = []
    for Kp in KP_SWEEP:
        pid = PIDParams(Kp=float(Kp), Kd=opt_Kd, Ki=0.0,
                        u_max=act.u_max, i_lim=act.u_max)
        agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s)
                          for s in sweep_seeds])
        sr_vals.append(agg['success_rate'])
        sweep_rows.append(dict(
            rocket_id=row['rocket_id'], Kp=float(Kp), opt_Kd=opt_Kd,
            sr=agg['success_rate'], rms=agg['rms_error_deg'],
        ))

    sr_arr  = np.array(sr_vals)
    passing = KP_SWEEP[sr_arr >= SR_THRESH]
    if len(passing) == 0:
        kp_floor = kp_ceiling = window_ratio = float('nan')
        floor_cens = ceil_cens = False
    else:
        kp_floor   = float(passing.min())
        kp_ceiling = float(passing.max())
        window_ratio = kp_ceiling / kp_floor
        floor_cens   = kp_floor   <= KP_SWEEP[0]  * 1.05
        ceil_cens    = kp_ceiling >= KP_SWEEP[-1] * 0.95

    summary = dict(
        rocket_id      = row['rocket_id'],
        td             = td,
        keff           = keff,
        u_max          = u_max,
        latency_steps  = int(row['latency_steps']),
        Iyy            = row['Iyy'],
        motor_scale    = row['motor_scale'],
        max_gimbal_deg = row['max_gimbal_deg'],
        wind_strength  = row['wind_strength'],
        servo_slew     = row['servo_slew_deg_s'],
        static_margin  = row.get('static_margin', 0.0),
        opt_Kp         = opt_Kp,
        opt_Kd         = opt_Kd,
        kp_floor       = kp_floor,
        kp_ceiling     = kp_ceiling,
        window_ratio   = window_ratio,
        peak_sr        = float(sr_arr.max()),
        floor_censored = floor_cens,
        ceil_censored  = ceil_cens,
        td_bin         = str(row.get('td_bin', '')),
        lat_bin        = str(row.get('lat_bin', '')),
    )
    return summary, sweep_rows


def _safe_eval(row: dict):
    try:
        return _eval_design(row)
    except Exception as exc:
        print(f"  ERROR {row.get('rocket_id','?')}: {exc}")
        return None, []


# ── Statistics helpers ────────────────────────────────────────────────────────
def _ols(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1 - float(np.sum(resid**2)) / ss_tot
    return beta, r2


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
        ss_t = float(np.sum((yv - yv.mean()) ** 2))
        r2s.append(1 - float(np.sum((yv - yh)**2)) / ss_t if ss_t > 0 else 0.0)
    return float(np.mean(r2s)), float(np.std(r2s))


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    pool = sample_lhs(N_POOL, seed=POOL_SEED)

    # Vectorised td/keff/u_max computation (faster than apply())
    T_avg_arr       = F15_AVG_THRUST_N * pool['motor_scale']
    pool['keff']    = T_avg_arr * CU_TO_RAD * L_NOZZLE / pool['Iyy']
    pool['u_max']   = pool['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    pool['td']      = pool['keff'] * pool['u_max']

    pool['td_bin']  = pd.cut(pool['td'],            bins=TD_BINS,  labels=TD_LABELS,  right=False)
    pool['lat_bin'] = pd.cut(pool['latency_steps'], bins=LAT_BINS, labels=LAT_LABELS, right=True)

    rng = np.random.RandomState(321)
    chosen = []
    for td_lbl in TD_LABELS:
        for lat_lbl in LAT_LABELS:
            cell = pool[(pool['td_bin'] == td_lbl) & (pool['lat_bin'] == lat_lbl)]
            n = min(N_PER_CELL, len(cell))
            if n > 0:
                chosen.append(cell.sample(n=n, random_state=rng))
    selected = pd.concat(chosen).reset_index(drop=True)
    selected['td_bin']  = selected['td_bin'].astype(str)
    selected['lat_bin'] = selected['lat_bin'].astype(str)

    # Print cell coverage table
    print("Cell coverage (td × latency):")
    for td_lbl in TD_LABELS:
        row_counts = [str(len(selected[(selected['td_bin']==td_lbl) & (selected['lat_bin']==ll)]))
                      for ll in LAT_LABELS]
        print(f"  {td_lbl:14} | " + "  ".join(f"{LAT_LABELS[i]}:{row_counts[i]}" for i in range(len(LAT_LABELS))))

    sims = N_KP_SRCH * N_KD_SRCH * len(SEARCH_SEEDS) + len(KP_SWEEP) * N_SWEEP_SEEDS
    print(f"\n{len(selected)} designs selected  "
          f"td=[{selected['td'].min():.0f}, {selected['td'].max():.0f}]  "
          f"latency={sorted(selected['latency_steps'].unique().tolist())}")
    print(f"Kp sweep: {len(KP_SWEEP)} pts × {N_SWEEP_SEEDS} seeds  "
          f"Sims/design: {sims}  Total: ~{sims*len(selected):,}")
    print("Running...\n")

    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(_safe_eval)(row) for row in selected.to_dict('records')
    )
    summaries  = [r[0] for r in raw if r[0] is not None]
    all_sweep  = [s for r in raw for s in r[1]]

    out = pd.DataFrame(summaries)
    out.to_csv(OUT_CSV, index=False)
    pd.DataFrame(all_sweep).to_csv(OUT_SWEEP_CSV, index=False)

    # ── Analysis ──────────────────────────────────────────────────────────────
    valid = out.dropna(subset=['window_ratio'])
    valid = valid[valid['window_ratio'] > 0].copy()
    n_tot, n_val = len(out), len(valid)
    n_fc = int(valid['floor_censored'].sum())
    n_cc = int(valid['ceil_censored'].sum())

    print(f"\n=== WINDOW RATIO RESULTS ===")
    print(f"n_valid={n_val}/{n_tot}  floor_censored={n_fc}  ceil_censored={n_cc}")
    wr = valid['window_ratio']
    print(f"window_ratio: min={wr.min():.1f}×  p25={wr.quantile(0.25):.1f}×  "
          f"median={wr.median():.1f}×  p75={wr.quantile(0.75):.1f}×  max={wr.max():.1f}×")

    y     = np.log(wr.values)
    ltd   = np.log(valid['td'].values)
    llat  = np.log(valid['latency_steps'].values.astype(float))
    lkeff = np.log(valid['keff'].values)
    lumax = np.log(valid['u_max'].values)
    lwind = np.log(valid['wind_strength'].values)
    lslew = np.log(valid['servo_slew'].values)
    n_v   = n_val

    print(f"\nCorrelations with log(window_ratio):")
    for name, arr in [('log_td', ltd), ('log_lat', llat),
                      ('log_keff', lkeff), ('log_umax', lumax)]:
        r, p = pearsonr(arr, y)
        print(f"  r({name}, log_wr)={r:+.3f}  p={p:.2e}")
    rho, _ = spearmanr(valid['td'], wr)
    print(f"  rho(td, window_ratio)={rho:+.3f}")

    ones = np.ones(n_v)

    def show(label, X, y_use=None):
        yy = y if y_use is None else y_use
        beta, r2 = _ols(X, yy)
        cv, cv_s = _cv_r2(X, yy)
        print(f"  {label:<32} R²={r2:.3f}  CV={cv:.3f}±{cv_s:.3f}  "
              f"coefs=[{' '.join(f'{b:+.3f}' for b in beta)}]")
        return r2, cv

    print("\nOLS models (outcome: log(window_ratio)):")
    show("log_td only",                 np.c_[ones, ltd])
    show("log_keff only",               np.c_[ones, lkeff])
    show("log_td + log_lat",            np.c_[ones, ltd,  llat])
    show("log_keff + log_lat",          np.c_[ones, lkeff, llat])
    show("log_keff + log_lat + log_umax",  np.c_[ones, lkeff, llat, lumax])
    show("log_td + log_lat + log_keff", np.c_[ones, ltd,  llat, lkeff])
    show("log_td + log_lat + log_wind", np.c_[ones, ltd,  llat, lwind])
    show("log_td + log_lat + log_slew", np.c_[ones, ltd,  llat, lslew])
    show("log_td + log_lat + log_wind + log_slew",
                                        np.c_[ones, ltd,  llat, lwind, lslew])
    show("log(td*lat) product",         np.c_[ones, ltd + llat])

    # Separate ceiling regression (exclude right-censored)
    vc = valid[~valid['ceil_censored'] & valid['kp_ceiling'].notna()].copy()
    if len(vc) >= 10:
        print(f"\nCeiling regression (n={len(vc)}, excl. right-censored; theory: -0.20*keff, -1.10*lat):")
        lc   = np.log(vc['kp_ceiling'].values)
        lkc  = np.log(vc['keff'].values)
        llc  = np.log(vc['latency_steps'].values.astype(float))
        ltd_c = np.log(vc['td'].values)
        ones_c = np.ones(len(vc))
        show("ceil ~ keff + lat",  np.c_[ones_c, lkc, llc], lc)
        show("ceil ~ td  + lat",   np.c_[ones_c, ltd_c, llc], lc)

    # Separate floor regression (exclude left-censored)
    vf = valid[~valid['floor_censored'] & valid['kp_floor'].notna()].copy()
    if len(vf) >= 10:
        print(f"\nFloor regression (n={len(vf)}, excl. left-censored; theory: +0.70*keff):")
        lfl  = np.log(vf['kp_floor'].values)
        lkf  = np.log(vf['keff'].values)
        llf  = np.log(vf['latency_steps'].values.astype(float))
        ltd_f = np.log(vf['td'].values)
        ones_f = np.ones(len(vf))
        show("floor ~ keff + lat", np.c_[ones_f, lkf, llf], lfl)
        show("floor ~ td  + lat",  np.c_[ones_f, ltd_f, llf], lfl)

    # Binned means
    print(f"\nBinned window_ratio by td bin:")
    for lbl in TD_LABELS:
        sub = valid[valid['td_bin'] == lbl]
        if len(sub) == 0:
            continue
        lat_med = sub['latency_steps'].median()
        print(f"  {lbl:14}: n={len(sub):3d}  "
              f"median={sub['window_ratio'].median():7.1f}×  "
              f"mean={sub['window_ratio'].mean():7.1f}×  "
              f"lat_median={lat_med:.0f}")

    print(f"\nSaved: {OUT_CSV}")
    print(f"Saved sweep detail: {OUT_SWEEP_CSV}")


if __name__ == '__main__':
    main()
