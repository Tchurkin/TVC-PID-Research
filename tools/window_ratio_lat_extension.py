"""
tools/window_ratio_lat_extension.py

Extends the window_ratio dataset to latency_steps 7-12 (Arduino-class MCUs).

Current v2 dataset (n=82 non-censored) has latency 1-6 only.
The tau exponent from the free fit is -1.765, consistent with the theoretical -2
but not tight enough to distinguish tau^1.5 from tau^2 at n=82.
Extending to lat=7-12 gives:
  - 12x latency span instead of 6x
  - Expected ceilings 32-54 (vs 63-380 for lat 1-6)
  - Expected windows much narrower -> tests extreme end of the Π-window curve

Protocol (same as v2):
  - 6 td bins x 4 latency bins (lat 7/8/10/12), 5 designs per cell = 120 new designs
  - Fresh LHS pool (seed=7777), latency_steps restricted to [7, 12]
  - Joint 18x7 Kp x Kd grid (3 search seeds, seeds 1-3) for opt_Kd
  - 32-pt Kp sweep over [0.05, 400] (ceiling is ~32-54 at lat=12; extend lower)
  - 15 eval seeds 25001-25015 (disjoint from all prior ranges)
  - Same SR_THRESH=0.80, same full physics fidelity

Outputs:
  experiments/results/window_ratio_lat_ext_py.csv        (summary, 120 rows)
  experiments/results/window_ratio_lat_ext_sweep_py.csv  (per-Kp SR)
  experiments/results/window_ratio_pooled_py.csv         (v2 + lat-ext merged)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
sys.path.insert(0, os.path.dirname(__file__))  # for exp1_stress_test imports
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import (
    build_plant, build_actuator, build_sensor,
    build_disturbance, build_scenario,
)
# Use the stress-test sampler that clips latency to [1, 12]
from exp1_stress_test import sample_lhs_stress
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# Sweep: lower ceiling expected at lat 7-12, extend Kp range down
KP_SWEEP      = np.geomspace(0.05, 400.0, 32)   # 1.33x per step
N_SWEEP_SEEDS = 15
SWEEP_SEED0   = 25001

# Joint search grid (for opt_Kd)
KP_GRID = np.geomspace(1, 200, 18)   # lat 7-12: ceiling ~32-54, so cap at 200
KD_GRID = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]
SEARCH_SEEDS = [1, 2, 3]

SR_THRESH = 0.80

N_POOL   = 30_000
POOL_SEED = 7777

# Stratification: 6 td bins x 4 lat bins, 5 per cell
TD_BINS  = [(5, 30), (30, 70), (70, 120), (120, 200), (200, 350), (350, 600)]
LAT_BINS = [7, 8, 10, 12]
N_PER_CELL = 5


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _compute_specs(row):
    T_avg = F15_AVG_THRUST_N * row['motor_scale']
    keff  = T_avg * CU_TO_RAD * L_NOZZLE / row['Iyy']
    u_max = row['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff * u_max), float(keff), float(u_max)


def _joint_search(row):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    best_sr, best_kp, best_kd = -1.0, 10.0, 2.0
    for Kp in KP_GRID:
        for Kd in KD_GRID:
            pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                            u_max=act.u_max, i_lim=act.u_max)
            agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s)
                              for s in SEARCH_SEEDS])
            if agg['success_rate'] > best_sr:
                best_sr, best_kp, best_kd = agg['success_rate'], float(Kp), float(Kd)
    return best_kp, best_kd, best_sr


def _eval_design(row):
    td, keff, u_max = _compute_specs(row)

    # Joint search for opt_Kd
    _, opt_Kd, search_sr = _joint_search(row)

    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

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
        peak_sr = float(sr_arr.max())
    else:
        kp_floor   = float(passing.min())
        kp_ceiling = float(passing.max())
        window_ratio = kp_ceiling / kp_floor
        floor_cens   = kp_floor   <= KP_SWEEP[0]  * 1.05
        ceil_cens    = kp_ceiling >= KP_SWEEP[-1] * 0.95
        peak_sr = float(sr_arr.max())

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
        search_best_sr = search_sr,
        kp_floor       = kp_floor,
        kp_ceiling     = kp_ceiling,
        window_ratio   = window_ratio,
        peak_sr        = peak_sr,
        floor_censored = floor_cens,
        ceil_censored  = ceil_cens,
    )
    return summary, sweep_rows


def _safe_eval(row):
    try:
        return _eval_design(row)
    except Exception as exc:
        print(f"  ERROR {row.get('rocket_id','?')}: {exc}")
        import traceback; traceback.print_exc()
        return None, []


def _cv_r2(X, y, k=5, seed=42):
    from sklearn.linear_model import LinearRegression
    from sklearn.model_selection import KFold, cross_val_score
    return cross_val_score(LinearRegression(), X, y,
                           cv=KFold(n_splits=k, shuffle=True, random_state=seed),
                           scoring='r2').mean()


def main():
    print("Sampling LHS pool (latency 7-12)...")
    # Use stress-test sampler that allows latency up to 12
    pool = sample_lhs_stress(N_POOL, seed=POOL_SEED)

    T_avg_arr    = F15_AVG_THRUST_N * pool['motor_scale']
    pool['keff'] = T_avg_arr * CU_TO_RAD * L_NOZZLE / pool['Iyy']
    pool['u_max'] = pool['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    pool['td']   = pool['keff'] * pool['u_max']

    # Filter to high-latency only
    pool = pool[pool['latency_steps'] >= 7].copy()
    print(f"  High-latency designs in pool: {len(pool)}")
    print(f"  latency range: {pool['latency_steps'].min()}-{pool['latency_steps'].max()}")
    print(f"  td range: [{pool['td'].min():.1f}, {pool['td'].max():.1f}]")

    # Stratify: 6 td bins x 4 lat bins, pick N_PER_CELL per cell
    selected_rows = []
    rng = np.random.RandomState(POOL_SEED)
    for (td_lo, td_hi) in TD_BINS:
        for lat_val in LAT_BINS:
            cell = pool[
                (pool['td'] >= td_lo) & (pool['td'] < td_hi) &
                (pool['latency_steps'] == lat_val)
            ].copy()
            if len(cell) == 0:
                print(f"  WARNING: no designs in td=[{td_lo},{td_hi}), lat={lat_val}")
                # try adjacent latencies
                for lat_adj in [lat_val-1, lat_val+1, lat_val-2, lat_val+2]:
                    cell = pool[
                        (pool['td'] >= td_lo) & (pool['td'] < td_hi) &
                        (pool['latency_steps'] == lat_adj)
                    ].copy()
                    if len(cell) >= N_PER_CELL:
                        print(f"    -> using lat={lat_adj} as fallback")
                        break
            n_pick = min(N_PER_CELL, len(cell))
            if n_pick == 0:
                print(f"  SKIP: td=[{td_lo},{td_hi}), lat={lat_val} — no designs at all")
                continue
            chosen = cell.sample(n=n_pick, random_state=rng)
            for _, r in chosen.iterrows():
                d = r.to_dict()
                d['td_bin'] = f"td{td_lo}-{td_hi}"
                d['lat_bin'] = f"lat{int(d['latency_steps'])}"
                selected_rows.append(d)

    print(f"\nSelected {len(selected_rows)} designs across {len(TD_BINS)}x{len(LAT_BINS)} cells")
    print(f"Simulations: {len(selected_rows)} x {len(KP_GRID)*len(KD_GRID)} search + {len(KP_SWEEP)*N_SWEEP_SEEDS} sweep")
    print(f"  ~{len(selected_rows) * (len(KP_GRID)*len(KD_GRID)*len(SEARCH_SEEDS) + len(KP_SWEEP)*N_SWEEP_SEEDS):,} total sim runs")

    print("\nRunning evaluations (parallel)...")
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_safe_eval)(row) for row in selected_rows
    )

    summaries = []
    sweep_all = []
    for r in results:
        if r[0] is not None:
            summaries.append(r[0])
            sweep_all.extend(r[1])

    df = pd.DataFrame(summaries)
    df.to_csv(RESULTS / 'window_ratio_lat_ext_py.csv', index=False)
    print(f"\nSaved {len(df)} rows -> {RESULTS / 'window_ratio_lat_ext_py.csv'}")

    pd.DataFrame(sweep_all).to_csv(
        RESULTS / 'window_ratio_lat_ext_sweep_py.csv', index=False)

    # Merge with v2 for pooled analysis
    v2 = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')
    pooled = pd.concat([v2, df], ignore_index=True)
    pooled.to_csv(RESULTS / 'window_ratio_pooled_py.csv', index=False)
    print(f"Pooled (v2 + lat-ext): {len(pooled)} rows -> window_ratio_pooled_py.csv")

    # Quick regression on pooled data
    valid = pooled[
        ~pooled['floor_censored'] & ~pooled['ceil_censored'] &
        pooled['window_ratio'].notna() & (pooled['window_ratio'] > 0)
    ].copy()
    print(f"\nNon-censored for regression: {len(valid)} (v2 alone: 82)")
    print(f"  Latency range: {valid['latency_steps'].min()}-{valid['latency_steps'].max()}")

    y  = np.log(valid['window_ratio'].values)
    lk = np.log(valid['keff'].values)
    ll = np.log(valid['latency_steps'].values)
    lu = np.log(valid['u_max'].values)

    from sklearn.linear_model import LinearRegression
    from sklearn.model_selection import KFold, cross_val_score
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    def cv(X): return cross_val_score(LinearRegression(), X, y, cv=kf, scoring='r2').mean()
    def cv1(x): return cv(x.reshape(-1,1))

    m = LinearRegression().fit(np.c_[lk, ll], y)
    cv_base = cv(np.c_[lk, ll])
    print(f"\nPooled regression: keff={m.coef_[0]:.3f} (theory:-1)  lat={m.coef_[1]:.3f} (theory:-2)")
    print(f"  CV R2 = {cv_base:.3f}")

    print(f"\nTau exponent sweep (pooled, n={len(valid)}):")
    for b in [1.0, 1.5, 1.88, 2.0, 2.5]:
        c = cv1(lk + b*ll)
        flag = " <-- theory" if abs(b-2.0)<0.01 else ""
        print(f"  keff * lat^{b:.2f}: CV={c:.3f}{flag}")

    print(f"\nDoes u_max add? delta CV = {cv(np.c_[lk, ll, lu]) - cv_base:+.3f}")


if __name__ == '__main__':
    main()
