"""
tools/valley_slope_validation.py

Targeted validation experiment: is the window~keff^(-2.33) slope observed in
valley_keff_sweep.py real physics, or an artifact of the adaptive-Kd method?

Setup:
  - Fixed airframe, lat=6, wind=0.40 (same as the cleanest slice from the prior run)
  - Same 20 Iyy levels -> keff range ~0.65 to ~39 rad/s^2/CU
  - 2D search: at each query Kp in the binary search, test a grid of Kd values and
    take the MAX SR over all Kd. This answers "does there EXIST a Kd making this Kp work?"
    -- which is what matters when a builder tunes both parameters.
  - Binary search gives ~2^12 = 4096 Kp resolution (no grid quantization artifact).

Window definition used here:
  floor  = min Kp for which any Kd achieves SR >= 0.80
  ceiling = max Kp for which any Kd achieves SR >= 0.80
  window_ratio = ceiling / floor

This is the most generous, builder-realistic window: the full Kp range a builder
could use if they also re-optimised Kd at each Kp.

If slope remains near -2.33: the effect is real physics (nonlinear keff coupling
at lat=6 is steeper than the population-average theory formula).
If slope collapses toward -1.06: the earlier result was a Kd-selection artifact.

Either outcome is informative and publishable.

Seed isolation: all seeds >= 60001 (disjoint from all prior experiments).
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

# ── Airframe (identical to valley_keff_sweep.py) ────────────────────────────
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.5, servo_slew_deg_s=120.0,
    max_gimbal_deg=10.0, deadband=0.05, backlash=0.10,
)

# ── Fixed experimental conditions ───────────────────────────────────────────
WIND        = 0.40
LATENCY     = 6
IYY_VALUES  = np.geomspace(0.003, 0.180, 20)

# ── 2D search grid ──────────────────────────────────────────────────────────
# At each Kp query point, test all of these Kd values and take max SR.
# Range covers: near-zero derivative (Kd=0.1) through heavy damping (Kd=32).
KD_GRID = np.array([0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0])

KP_MIN = 0.05
KP_MAX = 600.0

# Coarse grid: 10 Kp points to locate the approximate window region
KP_COARSE = np.geomspace(KP_MIN, KP_MAX, 10)

# ── Convergence settings ────────────────────────────────────────────────────
SR_THRESH        = 0.80
BINARY_ITERS     = 12      # 2^12 = 4096 Kp resolution
N_COARSE_SEEDS   = 6       # quick first pass
N_EVAL_SEEDS     = 10      # binary search
COARSE_SEED0     = 60001
EVAL_SEED0       = 60007   # 60007-60016

N_JOBS = -1

OUT_CSV = Path('experiments/results/valley_slope_validation_py.csv')


# ── Physics ─────────────────────────────────────────────────────────────────

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


def _td(Iyy):
    u_max = FIXED['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return _keff(Iyy) * u_max


# ── Core evaluation: max SR over all Kd values at a given Kp ─────────────

def _best_sr_at_kp(Kp, plant, act, sen, dis, sc, seeds):
    """
    Tests all KD_GRID values at this Kp. Returns:
      best_sr   -- max SR over all Kd
      best_Kd   -- the Kd that achieved it (ties broken by lower RMS)
      best_rms
    """
    best_sr, best_rms, best_Kd = -1.0, 1e9, KD_GRID[0]
    for Kd in KD_GRID:
        pid  = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                         u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
        sr   = float(np.mean([r['success']       for r in runs]))
        rms  = float(np.mean([r['rms_error_deg'] for r in runs]))
        if sr > best_sr or (sr == best_sr and rms < best_rms):
            best_sr, best_rms, best_Kd = sr, rms, float(Kd)
    return best_sr, best_Kd, best_rms


# ── Binary search for floor and ceiling ─────────────────────────────────────

def _binary_floor(plant, act, sen, dis, sc, lo, hi, seeds):
    """Leftmost Kp where best-over-Kd SR >= SR_THRESH."""
    lo, hi = np.log(lo), np.log(hi)
    for _ in range(BINARY_ITERS):
        mid = (lo + hi) / 2
        sr, _, _ = _best_sr_at_kp(np.exp(mid), plant, act, sen, dis, sc, seeds)
        if sr >= SR_THRESH:
            hi = mid
        else:
            lo = mid
    return float(np.exp((lo + hi) / 2))


def _binary_ceil(plant, act, sen, dis, sc, lo, hi, seeds):
    """Rightmost Kp where best-over-Kd SR >= SR_THRESH."""
    lo, hi = np.log(lo), np.log(hi)
    for _ in range(BINARY_ITERS):
        mid = (lo + hi) / 2
        sr, _, _ = _best_sr_at_kp(np.exp(mid), plant, act, sen, dis, sc, seeds)
        if sr >= SR_THRESH:
            lo = mid
        else:
            hi = mid
    return float(np.exp((lo + hi) / 2))


# ── Per-keff evaluation ──────────────────────────────────────────────────────

def _run_keff(Iyy):
    ke   = _keff(Iyy)
    td   = _td(Iyy)
    plant, act, sen, dis, sc = _build(Iyy)

    coarse_seeds = list(range(COARSE_SEED0, COARSE_SEED0 + N_COARSE_SEEDS))
    eval_seeds   = list(range(EVAL_SEED0,   EVAL_SEED0   + N_EVAL_SEEDS))

    # Coarse survey to locate window
    coarse_sr = []
    for Kp in KP_COARSE:
        sr, _, _ = _best_sr_at_kp(Kp, plant, act, sen, dis, sc, coarse_seeds)
        coarse_sr.append(sr)
    coarse_sr = np.array(coarse_sr)
    passing_idx = np.where(coarse_sr >= SR_THRESH)[0]

    if len(passing_idx) == 0:
        return dict(Iyy=Iyy, keff=ke, td=td,
                    kp_floor=float('nan'), kp_ceil=float('nan'),
                    window_ratio=float('nan'), floor_cens=False, ceil_cens=False,
                    infeasible=True, peak_sr=float(coarse_sr.max()))

    i_lo = int(passing_idx.min())
    i_hi = int(passing_idx.max())

    # Floor
    floor_cens = (i_lo == 0)
    if floor_cens:
        kp_floor = KP_MIN
    else:
        kp_floor = _binary_floor(plant, act, sen, dis, sc,
                                  lo=float(KP_COARSE[i_lo - 1]),
                                  hi=float(KP_COARSE[i_lo]),
                                  seeds=eval_seeds)

    # Ceiling
    ceil_cens = (i_hi == len(KP_COARSE) - 1)
    if ceil_cens:
        kp_ceil = KP_MAX
    else:
        kp_ceil = _binary_ceil(plant, act, sen, dis, sc,
                                lo=float(KP_COARSE[i_hi]),
                                hi=float(KP_COARSE[i_hi + 1]),
                                seeds=eval_seeds)

    window_ratio = kp_ceil / kp_floor if kp_floor > 0 else float('inf')

    # Best (Kp, Kd) for RMSE — from the coarse grid best point
    best_Kp_coarse = float(KP_COARSE[np.argmax(coarse_sr)])
    _, best_Kd_coarse, best_rms_coarse = _best_sr_at_kp(
        best_Kp_coarse, plant, act, sen, dis, sc, eval_seeds)

    return dict(
        Iyy=Iyy, keff=ke, td=td,
        kp_floor=kp_floor, kp_ceil=kp_ceil, window_ratio=window_ratio,
        floor_cens=floor_cens, ceil_cens=ceil_cens, infeasible=False,
        peak_sr=float(coarse_sr.max()),
        best_Kp=best_Kp_coarse, best_Kd=best_Kd_coarse, best_rms=best_rms_coarse,
    )


def _safe_run(Iyy):
    try:
        return _run_keff(Iyy)
    except Exception as e:
        return dict(Iyy=Iyy, keff=_keff(Iyy), td=_td(Iyy),
                    kp_floor=float('nan'), kp_ceil=float('nan'),
                    window_ratio=float('nan'), floor_cens=False, ceil_cens=False,
                    infeasible=True, peak_sr=float('nan'), error=str(e))


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    keff_vals = [_keff(iyy) for iyy in IYY_VALUES]
    # Coarse: 10 Kp × 9 Kd × 6 seeds; Binary: 12 iters × 2 sides × 9 Kd × 10 seeds
    sims_per = 10 * 9 * N_COARSE_SEEDS + BINARY_ITERS * 2 * len(KD_GRID) * N_EVAL_SEEDS
    print(f"Valley slope validation: {len(IYY_VALUES)} keff levels")
    print(f"Conditions: lat={LATENCY}, wind={WIND}")
    print(f"keff range: [{min(keff_vals):.2f}, {max(keff_vals):.2f}] rad/s^2/CU")
    print(f"2D search: {len(KD_GRID)} Kd values at each Kp query point")
    print(f"Est sims/condition: ~{sims_per}  |  total: ~{sims_per * len(IYY_VALUES):,}")
    print(f"Coarse seeds: {COARSE_SEED0}-{COARSE_SEED0+N_COARSE_SEEDS-1}")
    print(f"Eval seeds:   {EVAL_SEED0}-{EVAL_SEED0+N_EVAL_SEEDS-1}")

    results = Parallel(n_jobs=N_JOBS, verbose=10)(
        delayed(_safe_run)(iyy) for iyy in IYY_VALUES
    )

    df = pd.DataFrame(results).sort_values('keff').reset_index(drop=True)
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(df)} rows -> {OUT_CSV}")

    # Results table
    print(f"\n{'keff':>7}  {'td':>7}  {'floor':>8}  {'ceil':>8}  {'window':>9}  notes")
    print("-" * 65)
    for _, r in df.iterrows():
        if r['infeasible']:
            print(f"{r['keff']:>7.2f}  {r['td']:>7.1f}  {'INFEASIBLE':>8}")
            continue
        notes = []
        if r['floor_cens']: notes.append("floor<KP_MIN")
        if r['ceil_cens']:  notes.append("ceil>KP_MAX")
        note_str = ', '.join(notes) if notes else ''
        print(f"{r['keff']:>7.2f}  {r['td']:>7.1f}  "
              f"{r['kp_floor']:>8.3f}  {r['kp_ceil']:>8.1f}  "
              f"{r['window_ratio']:>9.0f}x  {note_str}")

    # Power law slope on non-censored rows
    clean = df[~df['infeasible'] & ~df['floor_cens'] & ~df['ceil_cens']].copy()
    print(f"\nNon-censored rows for slope fit: {len(clean)}")
    if len(clean) >= 3:
        x = np.log(clean['keff'].values)
        y = np.log(clean['window_ratio'].values)
        coef = np.polyfit(x, y, 1)
        print(f"Power law slope: {coef[0]:.2f}  (prior result: -2.33, theory: -1.06)")
        print(f"If slope near -1.06 -> prior result was Kd artifact")
        print(f"If slope near -2.33 -> real nonlinear physics at lat=6, wind=0.40")

    # Censoring summary
    n_fc = df['floor_cens'].sum()
    n_cc = df['ceil_cens'].sum()
    print(f"\nCensored: floor={n_fc}/{len(df)}, ceil={n_cc}/{len(df)}")


if __name__ == '__main__':
    main()
