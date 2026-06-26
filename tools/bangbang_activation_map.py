"""
tools/bangbang_activation_map.py  (2026-06-22)

PURPOSE
-------
Find the critical wind level at which the bang-bang floor mechanism activates
for a high-Pi design. This answers the regime question left open by v5b:

  v5b at wind=0.25: fsat_floor=0.000 for all designs, floor~0 for all.
  Bang-bang transition (CLAUDE.md): td≈208, wind≈0.40 shows fsat≈0.63 at all Kp.

  Is there a sharp wind threshold, or a smooth transition?
  Does the transition depend primarily on wind, or on keff, or on their product?

DESIGN
------
Two designs to test keff dependence of the activation threshold:
  A: R2087  keff=44.9, lat=6, Pi=1616  (most extreme in v5b)
  B: R2920  keff=35.0, lat=6, Pi=1259  (same latency, lower keff)

Wind sweep: 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50

Per (design, wind):
  1. Kp sweep [0.02, 600], 28 pts, constrained Kd (same as v5b)
  2. At floor Kp: measure fsat_floor
  3. At mid Kp (geometric mean of floor/ceil): measure fsat_mid
  4. At 2× ceiling (above ceiling): measure fsat_above  [if ceil found]

  The fsat profile (floor → mid → above-ceiling) across wind levels shows
  whether the system transitions from linear → bang-bang smoothly or sharply.

Key output: fsat_floor(wind), floor(wind), ceil(wind), window_ratio(wind)

SEEDS: 90001-90015 (eval), 90016-90018 (fsat meas). All disjoint from prior.
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario, sample_lhs)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate

# ---- Experiment parameters ----
WIND_LEVELS = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
KP_SWEEP    = np.geomspace(0.02, 600.0, 28)   # step ≈ 1.46×
KD_RATIOS   = np.array([0.10, 0.32, 1.00, 3.16])
KD_ZN       = 0.57
KD_CLIP     = (0.01, 32.0)
N_EVAL      = 15
EVAL_SEED0  = 90001
N_FSAT      = 3
FSAT_SEED0  = 90016
SR_THRESH   = 0.80

# Target designs: R2087 (keff=44.9, lat=6) and R2920 (keff=35.0, lat=6)
TARGET_IDS = ['R2087', 'R2920']

OUT_CSV   = Path('experiments/results/bangbang_activation_map_py.csv')
OUT_SWEEP = Path('experiments/results/bangbang_activation_sweep_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _kd_grid_for_kp(Kp):
    kds = list(KD_RATIOS * Kp) + [KD_ZN]
    return np.unique(np.clip(kds, KD_CLIP[0], KD_CLIP[1]))


def _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds):
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['success_rate'])


def _eval_fsat(plant, act, sen, dis, sc, Kp, Kd, seeds):
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['slew_sat_frac']), float(agg['u_cmd_sat_frac'])


def _run_cell(rocket_id, wind, row):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    # Override wind
    dis.gust_std = wind
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    seeds_eval = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    seeds_fsat = list(range(FSAT_SEED0, FSAT_SEED0 + N_FSAT))

    # Kp sweep with constrained Kd
    sweep_rows = []
    max_sr_per_kp  = {}
    best_kd_per_kp = {}
    for Kp in KP_SWEEP:
        kd_vals = _kd_grid_for_kp(Kp)
        best_sr, best_kd = -1.0, kd_vals[0]
        for Kd in kd_vals:
            sr = _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds_eval)
            sweep_rows.append(dict(
                rocket_id=rocket_id, wind=wind, Kp=float(Kp), Kd=float(Kd),
                Kd_Kp_ratio=float(Kd/Kp), sr=sr
            ))
            if sr > best_sr:
                best_sr, best_kd = sr, float(Kd)
        max_sr_per_kp[float(Kp)]  = best_sr
        best_kd_per_kp[float(Kp)] = best_kd

    # Floor / ceiling
    passing_kp = [kp for kp, sr in max_sr_per_kp.items() if sr >= SR_THRESH]
    if not passing_kp:
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

    # fsat profile at three Kp points
    fsat_floor = fsat_mid = fsat_above = np.nan
    usat_floor = np.nan

    if not np.isnan(floor) and not floor_censored:
        fsat_floor, usat_floor = _eval_fsat(
            plant, act, sen, dis, sc, floor, opt_kd_floor, seeds_fsat)

    if not np.isnan(floor) and not np.isnan(ceil) and not floor_censored and not ceil_censored:
        kp_mid = np.sqrt(floor * ceil)
        # find closest Kp in sweep
        kp_mid_actual = KP_SWEEP[np.argmin(np.abs(KP_SWEEP - kp_mid))]
        kd_mid = best_kd_per_kp.get(float(kp_mid_actual), KD_ZN)
        fsat_mid, _ = _eval_fsat(
            plant, act, sen, dis, sc, kp_mid_actual, kd_mid, seeds_fsat)

    if not np.isnan(ceil) and not ceil_censored:
        kp_above = ceil * 2.0
        kd_above = best_kd_per_kp.get(float(ceil), KD_ZN)
        fsat_above, _ = _eval_fsat(
            plant, act, sen, dis, sc, kp_above, kd_above, seeds_fsat)

    # Regime label
    if np.isnan(fsat_floor):
        regime = 'censored'
    elif fsat_floor < 0.20:
        regime = 'linear'
    elif fsat_floor < 0.50:
        regime = 'transitional'
    else:
        regime = 'bang-bang'

    result = dict(
        rocket_id=rocket_id, wind=wind,
        floor=floor, ceil=ceil, window_ratio=window_ratio,
        floor_censored=int(floor_censored), ceil_censored=int(ceil_censored),
        opt_kd_floor=opt_kd_floor, opt_kd_ceil=opt_kd_ceil,
        fsat_floor=fsat_floor, usat_floor=usat_floor,
        fsat_mid=fsat_mid, fsat_above=fsat_above,
        regime=regime,
    )
    return result, sweep_rows


def run():
    print("bangbang_activation_map.py")
    print(f"  Wind levels: {WIND_LEVELS}")
    print(f"  Designs: {TARGET_IDS}")
    print(f"  Kp sweep: {len(KP_SWEEP)} pts [{KP_SWEEP[0]:.3f}, {KP_SWEEP[-1]:.0f}]")
    print(f"  Eval seeds: {EVAL_SEED0}–{EVAL_SEED0+N_EVAL-1}  fsat seeds: {FSAT_SEED0}–{FSAT_SEED0+N_FSAT-1}")

    pool = sample_lhs(10000, seed=8888)
    pool_idx = pool.set_index('rocket_id')
    for rid in TARGET_IDS:
        if rid not in pool_idx.index:
            print(f"  WARNING: {rid} not found in pool")
        else:
            row = pool_idx.loc[rid]
            from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX
            CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
            keff = float(F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * 0.25 / row['Iyy'])
            lat  = int(row['latency_steps'])
            print(f"  {rid}: keff={keff:.1f}  lat={lat}  Pi={keff*lat**2:.0f}  wind_orig={row['wind_strength']:.2f}")

    n_cells = len(TARGET_IDS) * len(WIND_LEVELS)
    n_sims_est = n_cells * len(KP_SWEEP) * (len(KD_RATIOS) + 1) * N_EVAL
    print(f"\nCells: {n_cells}  Est. sims: {n_sims_est:,}")
    print("Running in parallel...")

    tasks = [(rid, wind, pool_idx.loc[rid])
             for rid in TARGET_IDS if rid in pool_idx.index
             for wind in WIND_LEVELS]

    results = Parallel(n_jobs=-1)(
        delayed(_run_cell)(rid, wind, row) for rid, wind, row in tasks
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
    print("BANG-BANG ACTIVATION MAP RESULTS")
    print("=" * 65)

    for rid in df['rocket_id'].unique():
        sub = df[df['rocket_id'] == rid].sort_values('wind')
        print(f"\n{'='*50}")
        print(f"Design: {rid}")
        print(f"{'='*50}")
        print(f"{'wind':>6}  {'floor':>8}  {'ceil':>8}  {'window':>8}  {'fsat_fl':>8}  {'fsat_mid':>9}  {'fsat_ab':>8}  regime")
        for _, r in sub.iterrows():
            fl  = f"{r.floor:.3f}"   if not np.isnan(r.floor)          else "cens"
            ce  = f"{r.ceil:.1f}"    if not np.isnan(r.ceil)            else "cens"
            wr  = f"{r.window_ratio:.1f}x" if not np.isnan(r.window_ratio) else "—"
            fs  = f"{r.fsat_floor:.3f}"    if not np.isnan(r.fsat_floor)   else "—"
            fm  = f"{r.fsat_mid:.3f}"      if not np.isnan(r.fsat_mid)     else "—"
            fa  = f"{r.fsat_above:.3f}"    if not np.isnan(r.fsat_above)   else "—"
            print(f"  {r.wind:.2f}  {fl:>8}  {ce:>8}  {wr:>8}  {fs:>8}  {fm:>9}  {fa:>8}  {r.regime}")

        # Find transition wind (first wind where fsat_floor > 0.20)
        above_thresh = sub[sub.fsat_floor > 0.20]
        if len(above_thresh) > 0:
            print(f"\n  Regime transition: fsat_floor first exceeds 0.20 at wind={above_thresh.iloc[0].wind:.2f}")
        bb = sub[sub.fsat_floor > 0.50]
        if len(bb) > 0:
            print(f"  Bang-bang onset:   fsat_floor first exceeds 0.50 at wind={bb.iloc[0].wind:.2f}")
        else:
            print(f"  Bang-bang onset:   not reached within tested wind range")

        # Floor at bang-bang onset
        if len(bb) > 0:
            r0 = bb.iloc[0]
            print(f"  Floor at bang-bang onset: {r0.floor:.3f}")
            print(f"  Ceil  at bang-bang onset: {r0.ceil:.1f}")
            print(f"  Window at bang-bang onset: {r0.window_ratio:.1f}x")


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        analyze()
    else:
        run()
