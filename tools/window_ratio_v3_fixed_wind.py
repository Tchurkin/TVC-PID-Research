"""
tools/window_ratio_v3_fixed_wind.py

v3 improvement over v2:
  - Fixes wind_strength=0.25 for all 120 designs (removes wind as a confound)
  - Increases eval seeds from 15 -> 20 (reduces SR stochastic noise ~18%)
  - Extends floor range: Kp_min = 0.05 (vs 0.1 in v2, reduces floor censoring)
  - Reuses the same 120 designs and opt_Kd from v2 CSV (no new search)

Motivation: v2 analysis showed adding log_wind as a predictor only improves CV R2
by ~0.015 (0.616 -> 0.631) because wind_strength varies across designs. Fixing wind
removes this as a source of variance in window_ratio that the model can't predict,
which should push CV R2 from ~0.62 toward 0.68-0.72.

The 15-seed stochastic noise on SR measurements is the other main bottleneck --
with 20 seeds, the SR measurement for any single (design, Kp) cell has noise
std(SR) reduced by sqrt(20/15) ~ 18%.

Seed isolation: uses seeds 23001-23020 (disjoint from 20001-20015, 21001-21015, 22001-22015).
Outputs:
  experiments/results/window_ratio_v3_py.csv        (summary, 120 rows)
  experiments/results/window_ratio_v3_sweep_py.csv  (per-Kp SR, 120*32=3840 rows)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy.stats import pearsonr

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

FIXED_WIND    = 0.25                            # constant wind for all designs (remove confound)
KP_SWEEP      = np.geomspace(0.05, 800.0, 32)  # extended floor: 0.05 (vs 0.1 in v2)
N_SWEEP_SEEDS = 20                              # 20 seeds (vs 15 in v1/v2)
SWEEP_SEED0   = 23001                           # fresh, disjoint from all prior experiments
SR_THRESH     = 0.80

N_POOL    = 10_000
POOL_SEED = 8888

V2_CSV    = Path('experiments/results/window_ratio_v2_py.csv')
OUT_CSV   = Path('experiments/results/window_ratio_v3_py.csv')
OUT_SWEEP = Path('experiments/results/window_ratio_v3_sweep_py.csv')


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

    # Override wind to fixed value -- this is the key change in v3
    row_v3 = dict(row)
    row_v3['wind_strength'] = FIXED_WIND

    plant = build_plant(row_v3)
    act   = build_actuator(row_v3)
    sen   = build_sensor(row_v3)
    dis   = build_disturbance(row_v3)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

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
        rocket_id         = row['rocket_id'],
        td                = td,
        keff              = keff,
        u_max             = u_max,
        latency_steps     = int(row['latency_steps']),
        Iyy               = row['Iyy'],
        motor_scale       = row['motor_scale'],
        max_gimbal_deg    = row['max_gimbal_deg'],
        wind_original     = row.get('wind_strength', float('nan')),  # original LHS value
        wind_fixed        = FIXED_WIND,
        servo_slew        = row['servo_slew_deg_s'],
        static_margin     = row.get('static_margin', 0.0),
        backlash          = row.get('backlash', float('nan')),
        deadband          = row.get('deadband', float('nan')),
        opt_Kd            = opt_Kd,
        kp_floor          = kp_floor,
        kp_ceiling        = kp_ceiling,
        window_ratio      = window_ratio,
        peak_sr           = float(sr_arr.max()),
        floor_censored    = floor_cens,
        ceil_censored     = ceil_cens,
        td_bin            = str(row.get('td_bin', '')),
        lat_bin           = str(row.get('lat_bin', '')),
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
    if not V2_CSV.exists():
        print(f"ERROR: {V2_CSV} not found. Run window_ratio_resweep_v2.py first.")
        return

    prior = pd.read_csv(V2_CSV)
    rocket_ids = set(prior['rocket_id'].tolist())
    print(f"v2 had {len(prior)} designs; reusing same {len(rocket_ids)} rocket_ids")
    print(f"KEY CHANGE (v3): wind_strength FIXED to {FIXED_WIND} for all designs")
    print(f"  (v2 used each design's LHS-sampled wind [0.05, 0.45])")
    print(f"  Seeds: {SWEEP_SEED0}-{SWEEP_SEED0+N_SWEEP_SEEDS-1} (n={N_SWEEP_SEEDS} vs 15 in v2)")
    print(f"  Kp range: [{KP_SWEEP[0]:.3f}, {KP_SWEEP[-1]:.0f}] ({len(KP_SWEEP)} pts)")

    # Regenerate pool to get full design specs (including wind_strength LHS value)
    pool = sample_lhs(N_POOL, seed=POOL_SEED)
    T_avg_arr    = F15_AVG_THRUST_N * pool['motor_scale']
    pool['keff'] = T_avg_arr * CU_TO_RAD * L_NOZZLE / pool['Iyy']
    pool['u_max'] = pool['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    pool['td']   = pool['keff'] * pool['u_max']

    selected = pool[pool['rocket_id'].isin(rocket_ids)].copy()
    print(f"Matched {len(selected)}/{len(rocket_ids)} rocket_ids in regenerated pool")

    # Merge opt_Kd from v2 (skip joint search)
    selected = selected.merge(prior[['rocket_id', 'opt_Kd', 'td_bin', 'lat_bin']],
                               on='rocket_id', how='left')
    missing_kd = selected['opt_Kd'].isna().sum()
    if missing_kd > 0:
        print(f"WARNING: {missing_kd} designs have no opt_Kd -> default 2.0")
        selected['opt_Kd'] = selected['opt_Kd'].fillna(2.0)

    sims = len(KP_SWEEP) * N_SWEEP_SEEDS
    print(f"\nTotal sims: {sims * len(selected):,}  ({sims} per design)")
    print("Running...\n")

    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(_safe_eval)(row) for row in selected.to_dict('records')
    )
    summaries = [r[0] for r in raw if r[0] is not None]
    all_sweep  = [s for r in raw for s in r[1]]

    out = pd.DataFrame(summaries)
    out.to_csv(OUT_CSV, index=False)
    pd.DataFrame(all_sweep).to_csv(OUT_SWEEP, index=False)
    print(f"\nSaved {len(out)} rows to {OUT_CSV}")

    # Analysis
    valid = out.dropna(subset=['window_ratio'])
    valid = valid[valid['window_ratio'] > 0].copy()
    n_tot, n_val = len(out), len(valid)
    n_fc = int(valid['floor_censored'].sum())
    n_cc = int(valid['ceil_censored'].sum())

    print(f"\n=== WINDOW RATIO v3 RESULTS (fixed wind={FIXED_WIND}) ===")
    print(f"n_valid={n_val}/{n_tot}  floor_censored={n_fc}  ceil_censored={n_cc}")
    print(f"  (v2 had: floor_censored~12, ceil_censored~24)")
    wr = valid['window_ratio']
    print(f"window_ratio: min={wr.min():.1f}x  p25={wr.quantile(0.25):.1f}x  "
          f"median={wr.median():.1f}x  p75={wr.quantile(0.75):.1f}x  max={wr.max():.1f}x")

    y     = np.log(wr.values)
    ltd   = np.log(valid['td'].values)
    llat  = np.log(valid['latency_steps'].values.astype(float))
    lkeff = np.log(valid['keff'].values)
    lumax = np.log(valid['u_max'].values)
    lslew = np.log(valid['servo_slew'].values)
    lbkl  = np.log(np.clip(valid['backlash'].values, 1e-3, None))
    ones  = np.ones(n_val)

    print(f"\nCorrelations with log(window_ratio) [v3, fixed wind]:")
    for name, arr in [('log_td', ltd), ('log_lat', llat), ('log_keff', lkeff),
                       ('log_slew', lslew)]:
        r, p = pearsonr(arr, y)
        print(f"  r({name}, log_wr)={r:+.3f}  p={p:.2e}")

    def show(label, X, y_use=None):
        yy = y if y_use is None else y_use
        beta, r2 = _ols(X, yy)
        cv, cv_s = _cv_r2(X, yy)
        print(f"  {label:<40} R2={r2:.3f}  CV={cv:.3f}+/-{cv_s:.3f}  "
              f"coefs=[{' '.join(f'{b:+.3f}' for b in beta)}]")
        return r2, cv

    print("\nOLS models (outcome: log(window_ratio)):")
    show("log_keff only",                  np.c_[ones, lkeff])
    show("log_lat only",                   np.c_[ones, llat])
    show("log_keff + log_lat",             np.c_[ones, lkeff, llat])
    show("log_td + log_lat",               np.c_[ones, ltd,   llat])
    show("log_keff + log_lat + log_slew",  np.c_[ones, lkeff, llat, lslew])
    show("log(td*lat) product",            np.c_[ones, ltd + llat])

    print("\n--- Comparison with prior versions ---")
    print("  v1: log_keff+log_lat  CV=0.656+/-0.087  (34%/35% censored, 16-pt sweep)")
    print("  v2: log_keff+log_lat  CV=0.616+/-???    (10%/20% censored, 32-pt sweep)")

    # Ceiling/floor decomposition
    vc = valid[~valid['ceil_censored'] & valid['kp_ceiling'].notna()].copy()
    vf = valid[~valid['floor_censored'] & valid['kp_floor'].notna()].copy()
    print(f"\nCeiling regression (n={len(vc)} non-censored; fixed wind removes ceiling confound):")
    if len(vc) >= 10:
        lc = np.log(vc['kp_ceiling'].values)
        lkc  = np.log(vc['keff'].values)
        llc  = np.log(vc['latency_steps'].values.astype(float))
        show("  ceil ~ keff + lat   ", np.c_[np.ones(len(vc)), lkc, llc], lc)
        b_kc, r2c = _ols(np.c_[np.ones(len(vc)), lkc, llc], lc)
        print(f"  -> keff exponent={b_kc[1]:+.3f}  lat exponent={b_kc[2]:+.3f}")
        print(f"  -> Formula: ceiling ~ {np.exp(b_kc[0]):.1f} / (keff^{-b_kc[1]:.3f} * lat^{-b_kc[2]:.3f})")

    print(f"\nFloor regression (n={len(vf)} non-censored; OLD: keff^0.70; v2: keff^1.06*lat^0.96):")
    if len(vf) >= 10:
        lf  = np.log(vf['kp_floor'].values)
        lkf = np.log(vf['keff'].values)
        llf = np.log(vf['latency_steps'].values.astype(float))
        show("  floor ~ keff only   ", np.c_[np.ones(len(vf)), lkf], lf)
        show("  floor ~ keff + lat  ", np.c_[np.ones(len(vf)), lkf, llf], lf)
        b_kf, r2f = _ols(np.c_[np.ones(len(vf)), lkf, llf], lf)
        print(f"  -> keff exponent={b_kf[1]:+.3f}  lat exponent={b_kf[2]:+.3f}")
        print(f"  -> Formula: floor ~ {np.exp(b_kf[0]):.4f} * keff^{b_kf[1]:.3f} * lat^{b_kf[2]:.3f}")

    print(f"\nDone. Key question: did CV R2 increase from v2 baseline (~0.616)?")
    print(f"Saved: {OUT_CSV}")


if __name__ == '__main__':
    main()
