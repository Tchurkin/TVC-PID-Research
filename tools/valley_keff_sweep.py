"""
tools/valley_keff_sweep.py

Maps the keff_opt = f(tau, wind) surface by sweeping control authority
across a wide range for a single fixed airframe at every combination of
latency and wind level.

Research question: Is there an optimal keff that maximises the gain window,
and how does keff_opt shift as tau and wind change?

Method:
  - Fixed airframe: mass=0.80, motor_scale=1.5, max_gimbal_deg=10.0
  - Sweep Iyy: 20 log-spaced values [0.003, 0.180] kg.m^2
    -> keff range: ~3 to ~120 rad/s^2/CU
  - 3 wind levels  : [0.10, 0.25, 0.40]
  - 3 latency levels: [1, 3, 6]
  - 9 conditions x 20 Iyy = 180 condition-design pairs

Per condition (adaptive search — no fixed grid):
  Phase 1 COARSE: 8-pt log Kp x 4 analytical-Kd (zeta=0,0.5,0.7,1.0)
    -> find best_Kd and approximate passing region (6 coarse seeds)
  Phase 2 BINARY FLOOR: geometric binary search for SR>=0.80 lower bound
  Phase 3 BINARY CEIL : geometric binary search for SR>=0.80 upper bound
  Phase 4 GOLDEN SECT : golden-section search for minimum RMSE in [floor,ceil]
    All use 12 eval seeds (disjoint from coarse).

Why this beats a fixed grid:
  - Binary search gives ~2^12 = 4096 Kp resolution vs 24-pt grid at 1.41x step
  - Golden section finds true RMSE minimum without quantization noise
  - Analytical Kd adapts to keff so comparisons across the keff sweep are fair

Output primary metric: window_ratio = kp_ceil / kp_floor (log gain margin)
Seed isolation: coarse=50001-50006, eval=50007-50018 (all disjoint)

Files:
  experiments/results/valley_keff_sweep_py.csv        (one row per condition)
  experiments/results/valley_keff_detail_py.csv       (coarse-grid raw data)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import (
    build_plant, build_actuator, build_sensor,
    build_disturbance, build_scenario,
)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

# ── Airframe constants ──────────────────────────────────────────────────────
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.5, servo_slew_deg_s=120.0,
    max_gimbal_deg=10.0, deadband=0.05, backlash=0.10,
)

# ── Experimental factors ────────────────────────────────────────────────────
IYY_VALUES  = np.geomspace(0.003, 0.180, 20)
WIND_LEVELS = [0.10, 0.25, 0.40]
LAT_LEVELS  = [1, 3, 6]

# ── Kp range ────────────────────────────────────────────────────────────────
KP_MIN = 0.05
KP_MAX = 600.0
# 8 coarse points spanning full range
KP_COARSE = np.geomspace(KP_MIN, KP_MAX, 8)

# ── Analytical Kd ───────────────────────────────────────────────────────────
ZETA_VALUES = [0.0, 0.5, 0.7, 1.0]

# ── Seed isolation ──────────────────────────────────────────────────────────
N_COARSE_SEEDS = 6
COARSE_SEED0   = 50001                         # 50001-50006
N_EVAL_SEEDS   = 12
EVAL_SEED0     = 50007                         # 50007-50018  (disjoint)

# ── Convergence ─────────────────────────────────────────────────────────────
SR_THRESH      = 0.80
BINARY_ITERS   = 12    # 2^12 = 4096 Kp resolution
GOLDEN_ITERS   = 15    # golden section iterations
N_JOBS         = -1

OUT_CSV    = Path('experiments/results/valley_keff_sweep_py.csv')
OUT_DET    = Path('experiments/results/valley_keff_detail_py.csv')


# ── Helpers ─────────────────────────────────────────────────────────────────

def _keff(Iyy: float) -> float:
    T = F15_AVG_THRUST_N * FIXED['motor_scale']
    return T * CU_TO_RAD * L_NOZZLE / Iyy


def _td(Iyy: float) -> float:
    u_max = FIXED['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return _keff(Iyy) * u_max


def _analytical_Kd(Kp: float, keff: float, zeta: float) -> float:
    """Critical-damping Kd for linearised plant: Kd = 2*zeta*sqrt(Kp/keff)."""
    if zeta == 0.0 or keff <= 0.0:
        return 0.0
    return 2.0 * zeta * np.sqrt(max(Kp, 1e-9) / keff)


def _physics_cfg() -> FidelityConfig:
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _build(Iyy: float, wind: float, lat: int):
    row = dict(FIXED, Iyy=float(Iyy), wind_strength=float(wind),
               latency_steps=int(lat))
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())
    return plant, act, sen, dis, sc


def _eval(Kp: float, Kd: float, plant, act, sen, dis, sc, seeds) -> tuple:
    """Returns (sr, rms)."""
    pid  = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    sr   = float(np.mean([r['success']        for r in runs]))
    rms  = float(np.mean([r['rms_error_deg']  for r in runs]))
    return sr, rms


# ── Phase 1: coarse survey ──────────────────────────────────────────────────

def _coarse_survey(plant, act, sen, dis, sc, keff: float):
    """
    8 Kp points x 4 zeta Kd options.
    Returns:
      best_Kd   — best Kd from the survey (for subsequent phases)
      table     — list of (Kp, Kd, sr, rms) for each coarse point
    """
    coarse_seeds = list(range(COARSE_SEED0, COARSE_SEED0 + N_COARSE_SEEDS))
    best_sr_global, best_rms_global = -1.0, 1e9
    best_Kd_global = 1.0
    table = []

    for Kp in KP_COARSE:
        best_sr_kp, best_rms_kp, best_Kd_kp = -1.0, 1e9, 1.0
        for zeta in ZETA_VALUES:
            Kd = _analytical_Kd(float(Kp), keff, zeta)
            sr, rms = _eval(float(Kp), Kd, plant, act, sen, dis, sc, coarse_seeds)
            if sr > best_sr_kp or (sr == best_sr_kp and rms < best_rms_kp):
                best_sr_kp, best_rms_kp, best_Kd_kp = sr, rms, Kd
        table.append((float(Kp), best_Kd_kp, best_sr_kp, best_rms_kp))
        if best_sr_kp > best_sr_global or (
                best_sr_kp == best_sr_global and best_rms_kp < best_rms_global):
            best_sr_global   = best_sr_kp
            best_rms_global  = best_rms_kp
            best_Kd_global   = best_Kd_kp

    return best_Kd_global, table


# ── Phase 2/3: binary search for floor and ceiling ─────────────────────────

def _binary_floor(plant, act, sen, dis, sc, Kd, lo, hi, seeds):
    """Geometric binary search: leftmost Kp where SR >= SR_THRESH."""
    lo, hi = np.log(lo), np.log(hi)
    for _ in range(BINARY_ITERS):
        mid     = (lo + hi) / 2
        sr, _   = _eval(np.exp(mid), Kd, plant, act, sen, dis, sc, seeds)
        if sr >= SR_THRESH:
            hi = mid     # passes -> floor at or below mid
        else:
            lo = mid     # fails  -> floor above mid
    return float(np.exp((lo + hi) / 2))


def _binary_ceil(plant, act, sen, dis, sc, Kd, lo, hi, seeds):
    """Geometric binary search: rightmost Kp where SR >= SR_THRESH."""
    lo, hi = np.log(lo), np.log(hi)
    for _ in range(BINARY_ITERS):
        mid     = (lo + hi) / 2
        sr, _   = _eval(np.exp(mid), Kd, plant, act, sen, dis, sc, seeds)
        if sr >= SR_THRESH:
            lo = mid     # passes -> ceiling at or above mid
        else:
            hi = mid     # fails  -> ceiling below mid
    return float(np.exp((lo + hi) / 2))


# ── Phase 4: golden section search for minimum RMSE ────────────────────────

def _golden_min_rms(plant, act, sen, dis, sc, Kd, lo, hi, seeds):
    """Golden section search for minimum RMSE in log-Kp space."""
    phi = (np.sqrt(5.0) - 1.0) / 2.0    # 0.6180...
    lo, hi = np.log(lo), np.log(hi)

    x1 = lo + (1.0 - phi) * (hi - lo)
    x2 = lo + phi          * (hi - lo)
    _, f1 = _eval(np.exp(x1), Kd, plant, act, sen, dis, sc, seeds)
    _, f2 = _eval(np.exp(x2), Kd, plant, act, sen, dis, sc, seeds)

    for _ in range(GOLDEN_ITERS):
        if f1 < f2:
            hi = x2
            x2, f2 = x1, f1
            x1 = lo + (1.0 - phi) * (hi - lo)
            _, f1 = _eval(np.exp(x1), Kd, plant, act, sen, dis, sc, seeds)
        else:
            lo = x1
            x1, f1 = x2, f2
            x2 = lo + phi * (hi - lo)
            _, f2 = _eval(np.exp(x2), Kd, plant, act, sen, dis, sc, seeds)

    opt_log = (lo + hi) / 2.0
    opt_Kp  = float(np.exp(opt_log))
    _, opt_rms = _eval(opt_Kp, Kd, plant, act, sen, dis, sc, seeds)
    return opt_Kp, opt_rms


# ── Main per-condition function ─────────────────────────────────────────────

def _run_condition(Iyy: float, wind: float, lat: int):
    ke    = _keff(Iyy)
    td    = _td(Iyy)
    plant, act, sen, dis, sc = _build(Iyy, wind, lat)
    eval_seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL_SEEDS))

    # Phase 1: coarse survey
    best_Kd, coarse_table = _coarse_survey(plant, act, sen, dis, sc, ke)

    kp_arr = np.array([r[0] for r in coarse_table])
    sr_arr = np.array([r[2] for r in coarse_table])
    passing_idx = np.where(sr_arr >= SR_THRESH)[0]

    base = dict(Iyy=Iyy, keff=ke, td=td, wind=wind, latency=lat,
                best_Kd=best_Kd,
                coarse_peak_sr=float(sr_arr.max()))

    if len(passing_idx) == 0:
        # Nothing passed coarse — design is infeasible at this condition
        return dict(**base, kp_floor=float('nan'), kp_ceil=float('nan'),
                    window_ratio=0.0, opt_Kp=float('nan'), opt_rms=float('nan'),
                    floor_cens=False, ceil_cens=False, infeasible=True)

    # Binary search brackets from coarse passing region
    i_lo = int(passing_idx.min())
    i_hi = int(passing_idx.max())

    # Floor bracket: [left-failing-coarse-point, first-passing-coarse-point]
    floor_cens = (i_lo == 0)   # first coarse point passes -> floor below KP_MIN
    if floor_cens:
        kp_floor = KP_MIN
    else:
        kp_floor = _binary_floor(plant, act, sen, dis, sc, best_Kd,
                                  lo=float(kp_arr[i_lo - 1]),
                                  hi=float(kp_arr[i_lo]),
                                  seeds=eval_seeds)

    # Ceiling bracket: [last-passing-coarse-point, right-failing-coarse-point]
    ceil_cens = (i_hi == len(kp_arr) - 1)   # last coarse point passes -> ceil above KP_MAX
    if ceil_cens:
        kp_ceil = KP_MAX
    else:
        kp_ceil = _binary_ceil(plant, act, sen, dis, sc, best_Kd,
                                lo=float(kp_arr[i_hi]),
                                hi=float(kp_arr[i_hi + 1]),
                                seeds=eval_seeds)

    window_ratio = kp_ceil / kp_floor if kp_floor > 0 else float('inf')

    # Golden section for minimum RMSE within [floor, ceil]
    gs_lo = max(kp_floor * 0.9, KP_MIN)
    gs_hi = min(kp_ceil  * 1.1, KP_MAX)
    opt_Kp, opt_rms = _golden_min_rms(
        plant, act, sen, dis, sc, best_Kd, gs_lo, gs_hi, eval_seeds
    )

    return dict(**base,
                kp_floor=kp_floor, kp_ceil=kp_ceil, window_ratio=window_ratio,
                opt_Kp=opt_Kp, opt_rms=opt_rms,
                floor_cens=floor_cens, ceil_cens=ceil_cens, infeasible=False)


def _safe_run(Iyy, wind, lat):
    try:
        return _run_condition(Iyy, wind, lat)
    except Exception as e:
        ke = _keff(Iyy)
        td = _td(Iyy)
        return dict(Iyy=Iyy, keff=ke, td=td, wind=wind, latency=lat,
                    best_Kd=float('nan'), coarse_peak_sr=float('nan'),
                    kp_floor=float('nan'), kp_ceil=float('nan'), window_ratio=float('nan'),
                    opt_Kp=float('nan'), opt_rms=float('nan'),
                    floor_cens=False, ceil_cens=False, infeasible=True,
                    error=str(e))


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    conditions = [
        (iyy, wind, lat)
        for lat  in LAT_LEVELS
        for wind in WIND_LEVELS
        for iyy  in IYY_VALUES
    ]

    keff_range = [_keff(IYY_VALUES.min()), _keff(IYY_VALUES.max())]
    # Phase 1: 8 Kp x 4 Kd x 6 seeds; Phases 2-4: (12+12+17) iters x 12 seeds
    sims_per_cond = 8 * 4 * N_COARSE_SEEDS + (BINARY_ITERS * 2 + GOLDEN_ITERS + 2) * N_EVAL_SEEDS
    print(f"Valley keff sweep: {len(IYY_VALUES)} Iyy x {len(WIND_LEVELS)} wind x {len(LAT_LEVELS)} lat")
    print(f"keff range: [{keff_range[1]:.1f}, {keff_range[0]:.1f}] rad/s^2/CU")
    print(f"Est sims per condition: ~{sims_per_cond}  |  total: ~{sims_per_cond * len(conditions):,}")
    print(f"Eval seeds: {EVAL_SEED0} - {EVAL_SEED0 + N_EVAL_SEEDS - 1} (disjoint from all prior)")
    print(f"Running {len(conditions)} conditions in parallel (N_JOBS={N_JOBS})...")

    results = Parallel(n_jobs=N_JOBS, verbose=10)(
        delayed(_safe_run)(iyy, wind, lat)
        for iyy, wind, lat in conditions
    )

    df = pd.DataFrame(results)
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(df)} rows to {OUT_CSV}")

    # Print valley summary
    print("\n=== VALLEY SUMMARY: keff_opt and window at each (latency, wind) ===")
    print(f"{'lat':>4}  {'wind':>5}  {'keff_opt':>9}  {'window':>9}  {'floor':>7}  {'ceil':>7}")
    print("-" * 55)
    for lat in LAT_LEVELS:
        for wind in WIND_LEVELS:
            sub = df[(df['latency'] == lat) & (df['wind'].round(3) == round(wind, 3))]
            sub = sub[~sub['infeasible'] & sub['window_ratio'].notna()].copy()
            if len(sub) == 0:
                print(f"{lat:>4}  {wind:>5.2f}  {'ALL INFEASIBLE':>9}")
                continue
            idx_opt = sub['window_ratio'].idxmax()
            r = sub.loc[idx_opt]
            print(f"{lat:>4}  {wind:>5.2f}  {r['keff']:>9.1f}  "
                  f"{r['window_ratio']:>9.0f}x  {r['kp_floor']:>7.2f}  {r['kp_ceil']:>7.1f}")

    # Infeasible count
    n_infeas = df['infeasible'].sum()
    if n_infeas:
        print(f"\nInfeasible conditions: {n_infeas}/{len(df)}")
        for _, r in df[df['infeasible']].iterrows():
            print(f"  Iyy={r['Iyy']:.4f} keff={r['keff']:.1f} wind={r['wind']:.2f} lat={int(r['latency'])}")

    # Floor/ceiling censoring
    n_floor_cens = df['floor_cens'].sum()
    n_ceil_cens  = df['ceil_cens'].sum()
    print(f"\nCensored: floor={n_floor_cens}/{len(df)} ({100*n_floor_cens/len(df):.0f}%), "
          f"ceil={n_ceil_cens}/{len(df)} ({100*n_ceil_cens/len(df):.0f}%)")


if __name__ == '__main__':
    main()
