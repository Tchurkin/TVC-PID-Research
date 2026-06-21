"""
tools/window_ratio_resweep_v2.py

Improved version of window_ratio_regression.py:
  - Same 120 designs (regenerated from same LHS pool seed=8888)
  - 32 Kp points instead of 16, over [0.1, 800] instead of [0.5, 480]
  - Step factor 1.34x instead of 1.58x (quantization noise halved in log space)
  - Extended range reduces ceiling/floor censoring
  - Reuses existing opt_Kd from window_ratio_regression_py.csv (skips joint search)
  - Fresh evaluation seeds 22001-22015 (disjoint from prior 20001-20015)

Motivation: v1 had 40/113 (35%) ceiling-censored and 39/113 (34%) floor-censored
designs. The 1.58x per Kp step also adds ~26% quantization noise to floor/ceiling
estimates in log space. v2 addresses both.

Expected improvement: CV R2 0.656 -> ~0.75-0.82 for log(keff) + log(latency) model.

Outputs:
  experiments/results/window_ratio_v2_py.csv          (summary, 120 rows)
  experiments/results/window_ratio_v2_sweep_py.csv    (per-Kp SR, 120*32=3840 rows)
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

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

KP_SWEEP      = np.geomspace(0.1, 800.0, 32)   # 1.34x per step vs 1.58x before
N_SWEEP_SEEDS = 15
SWEEP_SEED0   = 22001                            # fresh, disjoint from prior 20001-20015
SR_THRESH     = 0.80

N_POOL        = 10_000
POOL_SEED     = 8888                             # same as v1 to get identical designs

PRIOR_CSV = Path('experiments/results/window_ratio_regression_py.csv')
OUT_CSV   = Path('experiments/results/window_ratio_v2_py.csv')
OUT_SWEEP = Path('experiments/results/window_ratio_v2_sweep_py.csv')


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


def _eval_design(row: dict):
    td, keff, u_max = _compute_specs(row)

    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    # Use the previously-found opt_Kd (skip the expensive joint search)
    opt_Kd = float(row['opt_Kd'])

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
        import traceback; traceback.print_exc()
        return None, []


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


def main():
    if not PRIOR_CSV.exists():
        print(f"ERROR: {PRIOR_CSV} not found. Run window_ratio_regression.py first.")
        return

    prior = pd.read_csv(PRIOR_CSV)
    rocket_ids = set(prior['rocket_id'].tolist())
    print(f"v1 had {len(prior)} designs; reloading same {len(rocket_ids)} rocket_ids from pool")

    # Regenerate the same LHS pool used in v1
    pool = sample_lhs(N_POOL, seed=POOL_SEED)
    T_avg_arr    = F15_AVG_THRUST_N * pool['motor_scale']
    pool['keff'] = T_avg_arr * CU_TO_RAD * L_NOZZLE / pool['Iyy']
    pool['u_max'] = pool['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    pool['td']   = pool['keff'] * pool['u_max']

    selected = pool[pool['rocket_id'].isin(rocket_ids)].copy()
    print(f"Matched {len(selected)}/{len(rocket_ids)} rocket_ids in regenerated pool")

    # Merge in the opt_Kd from v1 (skip joint search)
    selected = selected.merge(prior[['rocket_id', 'opt_Kd', 'td_bin', 'lat_bin']],
                               on='rocket_id', how='left')
    missing_kd = selected['opt_Kd'].isna().sum()
    if missing_kd > 0:
        print(f"WARNING: {missing_kd} designs have no opt_Kd — will use default Kd=2.0")
        selected['opt_Kd'] = selected['opt_Kd'].fillna(2.0)

    sims = len(KP_SWEEP) * N_SWEEP_SEEDS
    print(f"\nKp sweep: {len(KP_SWEEP)} pts over [{KP_SWEEP[0]:.2f}, {KP_SWEEP[-1]:.1f}]")
    print(f"  Step factor: {KP_SWEEP[1]/KP_SWEEP[0]:.3f}x (was 1.581x)")
    print(f"  Seeds: {SWEEP_SEED0}-{SWEEP_SEED0+N_SWEEP_SEEDS-1}")
    print(f"  Sims/design: {sims}  Total: ~{sims*len(selected):,}")
    print("Running...\n")

    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(_safe_eval)(row) for row in selected.to_dict('records')
    )
    summaries = [r[0] for r in raw if r[0] is not None]
    all_sweep = [s for r in raw for s in r[1]]

    out = pd.DataFrame(summaries)
    out.to_csv(OUT_CSV, index=False)
    pd.DataFrame(all_sweep).to_csv(OUT_SWEEP, index=False)
    print(f"Saved {len(out)} rows to {OUT_CSV}")

    # Analysis
    valid = out.dropna(subset=['window_ratio'])
    valid = valid[valid['window_ratio'] > 0].copy()
    n_tot, n_val = len(out), len(valid)
    n_fc = int(valid['floor_censored'].sum())
    n_cc = int(valid['ceil_censored'].sum())

    print(f"\n=== WINDOW RATIO v2 RESULTS ===")
    print(f"n_valid={n_val}/{n_tot}  floor_censored={n_fc}  ceil_censored={n_cc}")
    print(f"  (v1 had: 113/120, floor_censored=39, ceil_censored=40)")
    wr = valid['window_ratio']
    print(f"window_ratio: min={wr.min():.1f}x  p25={wr.quantile(0.25):.1f}x  "
          f"median={wr.median():.1f}x  p75={wr.quantile(0.75):.1f}x  max={wr.max():.1f}x")

    y     = np.log(wr.values)
    ltd   = np.log(valid['td'].values)
    llat  = np.log(valid['latency_steps'].values.astype(float))
    lkeff = np.log(valid['keff'].values)
    lumax = np.log(valid['u_max'].values)
    lwind = np.log(valid['wind_strength'].values)
    lslew = np.log(valid['servo_slew'].values)
    ones  = np.ones(n_val)

    print(f"\nCorrelations with log(window_ratio):")
    for name, arr in [('log_td', ltd), ('log_lat', llat), ('log_keff', lkeff)]:
        r, p = pearsonr(arr, y)
        print(f"  r({name}, log_wr)={r:+.3f}  p={p:.2e}")

    def show(label, X, y_use=None):
        yy = y if y_use is None else y_use
        beta, r2 = _ols(X, yy)
        cv, cv_s = _cv_r2(X, yy)
        print(f"  {label:<36} R2={r2:.3f}  CV={cv:.3f}+/-{cv_s:.3f}  "
              f"coefs=[{' '.join(f'{b:+.3f}' for b in beta)}]")
        return r2, cv

    print("\nOLS models (outcome: log(window_ratio)):")
    show("log_td only",                    np.c_[ones, ltd])
    show("log_keff only",                  np.c_[ones, lkeff])
    show("log_td + log_lat",               np.c_[ones, ltd,   llat])
    show("log_keff + log_lat",             np.c_[ones, lkeff, llat])
    show("log_keff + log_lat + log_wind",  np.c_[ones, lkeff, llat, lwind])
    show("log(td*lat) product",            np.c_[ones, ltd + llat])

    print("\n--- Comparison with v1 ---")
    print("  v1: log_keff+log_lat  R2=0.682  CV=0.656+/-0.087")
    print("  v1: log(td*lat)       R2=0.556  CV=0.498+/-0.066")

    # Ceiling regression
    vc = valid[~valid['ceil_censored'] & valid['kp_ceiling'].notna()].copy()
    vf = valid[~valid['floor_censored'] & valid['kp_floor'].notna()].copy()
    print(f"\nCeiling regression (n={len(vc)}, non-censored; theory keff^-0.20 lat^-1.10):")
    if len(vc) >= 10:
        lc = np.log(vc['kp_ceiling'].values)
        lkc = np.log(vc['keff'].values); llc = np.log(vc['latency_steps'].values.astype(float))
        show("  ceil ~ keff + lat  ", np.c_[np.ones(len(vc)), lkc, llc], lc)
    print(f"\nFloor regression (n={len(vf)}, non-censored; theory keff^+0.70, lat~0):")
    if len(vf) >= 10:
        lf = np.log(vf['kp_floor'].values)
        lkf = np.log(vf['keff'].values); llf = np.log(vf['latency_steps'].values.astype(float))
        show("  floor ~ keff only  ", np.c_[np.ones(len(vf)), lkf], lf)
        show("  floor ~ keff + lat ", np.c_[np.ones(len(vf)), lkf, llf], lf)

    print(f"\nSaved: {OUT_CSV}")


if __name__ == '__main__':
    main()
