"""
tools/valley_highseed_validation.py

High-seed targeted validation of the window ~ keff^alpha slope at lat=6, wind=0.40.

Prior runs found slope ~ -2.2 but with only 5 non-censored points and 10 eval seeds
(sigma_p ~ 0.13 at SR=0.80). This run asks: does the slope hold at 50 seeds?

Design:
  - 8 keff values in [10, 39] -- the zone that determines the slope
  - Fixed Kd from a proper joint 18x7 (Kp x Kd) search (3 seeds)
  - Binary search for floor and ceiling with 50 eval seeds
    (sigma_p ~ 0.057 at SR=0.80 -- ~2.3x tighter than prior run)
  - lat=6, wind=0.40 (same conditions as both prior runs)

This directly answers: does the ~-2 exponent survive more reliable SR estimates?
  - If slope stays in [-2.5, -1.5]: real nonlinear physics, consistent with
    interaction model (keff-dependent latency exponent)
  - If slope collapses toward [-1.2, -1.0]: prior results were threshold noise

Seed isolation: joint search seeds 1-3 (standard), eval seeds 70001-70050.
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

# ── Airframe (same fixed reference as all valley experiments) ───────────────
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.5, servo_slew_deg_s=120.0,
    max_gimbal_deg=10.0, deadband=0.05, backlash=0.10,
)

WIND    = 0.40
LATENCY = 6

# 8 keff targets in [10, 39]; computed as Iyy = keff_ref * Iyy_ref / keff_target
# Reference: at Iyy=0.003, keff=39.27 (from prior runs)
_KEFF_REF = 39.269908
_IYY_REF  = 0.003
KEFF_TARGETS = np.array([10.0, 12.5, 15.6, 19.5, 24.4, 30.5, 38.1, 39.3])
IYY_VALUES   = _IYY_REF * _KEFF_REF / KEFF_TARGETS   # inverse relationship

# ── Joint search grid (standard, same as window_ratio_regression.py) ────────
KP_SRCH = np.geomspace(1.0, 320.0, 18)
KD_SRCH = np.geomspace(1.0,  64.0,  7)
SEARCH_SEEDS = [1, 2, 3]

# ── Binary search settings ───────────────────────────────────────────────────
KP_MIN       = 0.05
KP_MAX       = 600.0
KP_COARSE    = np.geomspace(KP_MIN, KP_MAX, 10)
SR_THRESH    = 0.80
BINARY_ITERS = 12       # 2^12 Kp resolution
N_EVAL       = 50       # key parameter: sigma_p ~ 0.057 at SR=0.80
EVAL_SEED0   = 70001    # 70001-70050, disjoint from all prior runs

N_JOBS = -1
OUT_CSV = Path('experiments/results/valley_highseed_validation_py.csv')


# ── Helpers ──────────────────────────────────────────────────────────────────

def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _build(Iyy):
    row = dict(FIXED, Iyy=float(Iyy), wind_strength=WIND, latency_steps=LATENCY)
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())
    return plant, act, sen, dis, sc


def _keff(Iyy):
    return F15_AVG_THRUST_N * FIXED['motor_scale'] * CU_TO_RAD * L_NOZZLE / Iyy


def _eval_sr(Kp, Kd, plant, act, sen, dis, sc, seeds):
    pid  = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                     u_max=act.u_max, i_lim=act.u_max)
    runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    sr   = float(np.mean([r['success']       for r in runs]))
    rms  = float(np.mean([r['rms_error_deg'] for r in runs]))
    return sr, rms


# ── Joint search: find optimal (Kp, Kd) ──────────────────────────────────────

def _joint_search(plant, act, sen, dis, sc):
    best_sr, best_rms, best_Kp, best_Kd = -1.0, 1e9, 10.0, 2.0
    for Kp in KP_SRCH:
        for Kd in KD_SRCH:
            sr, rms = _eval_sr(float(Kp), float(Kd), plant, act, sen, dis, sc,
                                SEARCH_SEEDS)
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr, best_rms, best_Kp, best_Kd = sr, rms, float(Kp), float(Kd)
    return best_Kp, best_Kd, best_sr


# ── Binary search with fixed Kd ───────────────────────────────────────────────

def _binary_floor(Kd, plant, act, sen, dis, sc, lo, hi, seeds):
    lo, hi = np.log(lo), np.log(hi)
    for _ in range(BINARY_ITERS):
        mid = (lo + hi) / 2
        sr, _ = _eval_sr(np.exp(mid), Kd, plant, act, sen, dis, sc, seeds)
        if sr >= SR_THRESH:
            hi = mid
        else:
            lo = mid
    return float(np.exp((lo + hi) / 2))


def _binary_ceil(Kd, plant, act, sen, dis, sc, lo, hi, seeds):
    lo, hi = np.log(lo), np.log(hi)
    for _ in range(BINARY_ITERS):
        mid = (lo + hi) / 2
        sr, _ = _eval_sr(np.exp(mid), Kd, plant, act, sen, dis, sc, seeds)
        if sr >= SR_THRESH:
            lo = mid
        else:
            hi = mid
    return float(np.exp((lo + hi) / 2))


# ── Per-keff pipeline ─────────────────────────────────────────────────────────

def _run_keff(Iyy):
    ke   = _keff(float(Iyy))
    plant, act, sen, dis, sc = _build(float(Iyy))
    eval_seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))

    # Step 1: joint search to find best Kd
    opt_Kp, opt_Kd, search_sr = _joint_search(plant, act, sen, dis, sc)

    # Step 2: coarse sweep with frozen opt_Kd to locate window
    coarse_sr = np.array([
        _eval_sr(float(Kp), opt_Kd, plant, act, sen, dis, sc, eval_seeds[:10])[0]
        for Kp in KP_COARSE
    ])
    passing_idx = np.where(coarse_sr >= SR_THRESH)[0]

    base = dict(Iyy=Iyy, keff=ke, opt_Kp=opt_Kp, opt_Kd=opt_Kd, search_sr=search_sr)

    if len(passing_idx) == 0:
        return dict(**base, kp_floor=float('nan'), kp_ceil=float('nan'),
                    window_ratio=float('nan'), floor_cens=False, ceil_cens=False,
                    infeasible=True, peak_sr=float(coarse_sr.max()))

    i_lo = int(passing_idx.min())
    i_hi = int(passing_idx.max())

    # Step 3: binary floor
    floor_cens = (i_lo == 0)
    if floor_cens:
        kp_floor = KP_MIN
    else:
        kp_floor = _binary_floor(opt_Kd, plant, act, sen, dis, sc,
                                  float(KP_COARSE[i_lo - 1]),
                                  float(KP_COARSE[i_lo]),
                                  eval_seeds)

    # Step 4: binary ceiling
    ceil_cens = (i_hi == len(KP_COARSE) - 1)
    if ceil_cens:
        kp_ceil = KP_MAX
    else:
        kp_ceil = _binary_ceil(opt_Kd, plant, act, sen, dis, sc,
                                float(KP_COARSE[i_hi]),
                                float(KP_COARSE[i_hi + 1]),
                                eval_seeds)

    window_ratio = kp_ceil / kp_floor if kp_floor > 0 else float('inf')

    return dict(**base,
                kp_floor=kp_floor, kp_ceil=kp_ceil, window_ratio=window_ratio,
                floor_cens=floor_cens, ceil_cens=ceil_cens,
                infeasible=False, peak_sr=float(coarse_sr.max()))


def _safe_run(Iyy):
    try:
        return _run_keff(Iyy)
    except Exception as e:
        return dict(Iyy=Iyy, keff=_keff(float(Iyy)), error=str(e),
                    kp_floor=float('nan'), kp_ceil=float('nan'),
                    window_ratio=float('nan'), infeasible=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    sims_per = len(KP_SRCH)*len(KD_SRCH)*len(SEARCH_SEEDS) + \
               len(KP_COARSE)*10 + BINARY_ITERS*2*N_EVAL
    print(f"High-seed valley validation: {len(IYY_VALUES)} keff levels")
    print(f"Conditions: lat={LATENCY}, wind={WIND}")
    print(f"keff targets: {np.round(KEFF_TARGETS, 1).tolist()}")
    print(f"Eval seeds: {N_EVAL} seeds ({EVAL_SEED0}-{EVAL_SEED0+N_EVAL-1})")
    print(f"sigma_p at SR=0.80: {np.sqrt(0.8*0.2/N_EVAL):.3f}  (prior: {np.sqrt(0.8*0.2/10):.3f})")
    print(f"Est sims/condition: ~{sims_per}  |  total: ~{sims_per * len(IYY_VALUES):,}")

    results = Parallel(n_jobs=N_JOBS, verbose=10)(
        delayed(_safe_run)(iyy) for iyy in IYY_VALUES
    )

    df = pd.DataFrame(results).sort_values('keff').reset_index(drop=True)
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(df)} rows -> {OUT_CSV}")

    print(f"\n{'keff':>7}  {'opt_Kd':>7}  {'floor':>8}  {'ceil':>8}  {'window':>9}  notes")
    print("-" * 70)
    for _, r in df.iterrows():
        if r.get('infeasible', False):
            print(f"{r['keff']:>7.1f}  {'INFEASIBLE':>50}")
            continue
        notes = []
        if r['floor_cens']: notes.append("floor<KP_MIN")
        if r['ceil_cens']:  notes.append("ceil>KP_MAX")
        print(f"{r['keff']:>7.1f}  {r['opt_Kd']:>7.3f}  "
              f"{r['kp_floor']:>8.3f}  {r['kp_ceil']:>8.1f}  "
              f"{r['window_ratio']:>9.0f}x  {', '.join(notes)}")

    clean = df[~df.get('infeasible', pd.Series([False]*len(df), dtype=bool)) &
               ~df['floor_cens'] & ~df['ceil_cens']].copy()
    print(f"\nNon-censored points: {len(clean)}")
    if len(clean) >= 3:
        x = np.log(clean['keff'].values)
        y = np.log(clean['window_ratio'].values)
        coef  = np.polyfit(x, y, 1)
        resid = y - np.polyval(coef, x)
        print(f"Power law fit: window ~ keff^{coef[0]:.2f}  (R^2={1 - resid.var()/y.var():.3f})")
        print(f"  Theory prediction: -1.06")
        print(f"  Prior 10-seed run: -2.22")
        if coef[0] < -1.5:
            print("  -> Steeper than theory CONFIRMED: real nonlinear keff-latency coupling")
        elif coef[0] > -1.3:
            print("  -> Collapsed toward theory: prior result was threshold noise")
        else:
            print("  -> Intermediate: partial noise, partial physics (inconclusive)")

    n_fc = df['floor_cens'].sum()
    n_cc = df['ceil_cens'].sum()
    print(f"\nCensored: floor={n_fc}/{len(df)}, ceil={n_cc}/{len(df)}")


if __name__ == '__main__':
    main()
