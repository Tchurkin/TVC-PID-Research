"""
tools/floor_mechanism_test.py

Mechanism isolation for Kp_floor ≈ C·keff·S·τ/θ_max.

Derivation (see CLAUDE.md floor mechanism analysis):
  In the wind-driven bang-bang regime, the sensor delay τ creates a dead window
  during which the controller keeps commanding the wrong direction. At the moment
  the correction fires, the rocket has accumulated:

      ω_switch ≈ keff · S · τ   [slew-limited; keff·u_max·τ if amplitude-limited]

  The minimum Kp that catches the trajectory before θ > θ_max is:

      Kp_floor ≈ Kd · keff · S · τ / θ_max   ->   floor ∝ keff¹ · τ¹

  with S hidden in the coefficient (S was fixed at 120 deg/s in all prior experiments).

Predictions:
  floor ∝ S^+1.0   (new — S was never varied)
  floor ⊥ u_max    (amplitude is not the binding constraint; S·τ/u_max = 0.03)
  floor -> low/gone without slew (removing the key nonlinearity)

Three experiments at 3 keff × lat=6, wind=0.25:

  E1: Ablation
      baseline vs no_slew (FidelityConfig slew=False)
      Tests: is slew necessary for the floor?

  E2: Slew sweep, S ∈ {40, 80, 120, 160, 200} deg/s
      Tests: does floor ∝ S?

  E3: Gimbal/u_max sweep, max_gimbal ∈ {2, 5, 10, 15, 20} deg, Iyy compensated
      to hold keff fixed (keff = const·max_gimbal/Iyy).
      Tests: does floor depend on amplitude limit u_max at fixed keff, τ, S?

Kd frozen from baseline joint search — varies only one variable at a time.
Seeds: search 1-3, eval 80001-80030 (disjoint from all prior).
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

# ── Reference airframe ────────────────────────────────────────────────────────
L_NOZZLE  = 0.25
# CU_TO_RAD at reference max_gimbal (cancels in keff formula only when gimbal is fixed)
CU_TO_RAD_REF = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

BASE_FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.5,
    deadband=0.02, backlash=0.05,
)

BASE_GIMBAL = 10.0    # deg, reference
BASE_SLEW   = 120.0   # deg/s, reference
LATENCY     = 6
WIND        = 0.25

# keff = F15 * motor_scale * (pi/180 * max_gimbal / 12) * L / Iyy
# at BASE_GIMBAL: keff = _KC / Iyy
_KC = F15_AVG_THRUST_N * BASE_FIXED['motor_scale'] * CU_TO_RAD_REF * L_NOZZLE

KEFF_TARGETS = np.array([8.0, 15.0, 25.0])
IYY_BASE     = _KC / KEFF_TARGETS   # Iyy at BASE_GIMBAL for each keff target

# ── Sweep parameters ──────────────────────────────────────────────────────────
SLEW_VALUES   = [40.0, 80.0, 120.0, 160.0, 200.0]   # deg/s, E2
GIMBAL_VALUES = [2.0, 5.0, 10.0, 15.0, 20.0]         # deg, E3

# ── Gain search / eval ────────────────────────────────────────────────────────
KP_SRCH      = np.geomspace(1.0, 320.0, 18)
KD_SRCH      = np.geomspace(1.0,  64.0,  7)
SEARCH_SEEDS = [1, 2, 3]

KP_COARSE    = np.geomspace(0.05, 600.0, 14)
SR_THRESH    = 0.80
BINARY_ITERS = 14        # 2^14 Kp resolution
N_EVAL       = 30        # σ_p ≈ 0.073 at SR=0.80
EVAL_SEED0   = 80001     # 80001-80030, disjoint from all prior

N_JOBS  = -1
OUT_CSV = Path('experiments/results/floor_mechanism_test_py.csv')


# ── Helpers ───────────────────────────────────────────────────────────────────

def _keff_from_row(row):
    cu2r = np.pi / 180 * row['max_gimbal_deg'] / REF_U_MAX
    return F15_AVG_THRUST_N * row['motor_scale'] * cu2r * L_NOZZLE / row['Iyy']


def _physics_cfg(slew_on=True):
    return FidelityConfig(
        wind=True, backlash=True, slew=slew_on, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _build(row_dict, slew_on=True):
    plant = build_plant(row_dict)
    act   = build_actuator(row_dict)
    sen   = build_sensor(row_dict)
    dis   = build_disturbance(row_dict)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg(slew_on))
    return plant, act, sen, dis, sc


def _row(Iyy, max_gimbal=BASE_GIMBAL, slew_deg_s=BASE_SLEW):
    return dict(
        BASE_FIXED,
        Iyy=float(Iyy),
        max_gimbal_deg=float(max_gimbal),
        servo_slew_deg_s=float(slew_deg_s),
        wind_strength=WIND,
        latency_steps=LATENCY,
    )


def _eval_sr(Kp, Kd, u_max, plant, act, sen, dis, sc, seeds):
    pid  = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                     u_max=float(u_max), i_lim=float(u_max))
    runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    return float(np.mean([r['success'] for r in runs]))


# ── Joint Kd search (baseline only, Kd frozen across conditions) ─────────────

def _joint_search(plant, act, sen, dis, sc):
    best_sr, best_rms, best_Kp, best_Kd = -1.0, 1e9, 10.0, 2.0
    for Kp in KP_SRCH:
        for Kd in KD_SRCH:
            pid  = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                             u_max=act.u_max, i_lim=act.u_max)
            runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in SEARCH_SEEDS]
            sr   = float(np.mean([r['success'] for r in runs]))
            rms  = float(np.mean([r['rms_error_deg'] for r in runs]))
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr, best_rms, best_Kp, best_Kd = sr, rms, float(Kp), float(Kd)
    return best_Kp, best_Kd, best_sr


# ── Floor/ceiling measurement ─────────────────────────────────────────────────

def _measure_floor_ceil(row_dict, opt_Kd, eval_seeds, slew_on=True):
    """Binary-search floor and ceiling at frozen opt_Kd."""
    plant, act, sen, dis, sc = _build(row_dict, slew_on=slew_on)
    u_max = act.u_max

    # Coarse sweep (10 seeds for speed)
    coarse_sr = np.array([
        _eval_sr(float(Kp), opt_Kd, u_max, plant, act, sen, dis, sc, eval_seeds[:10])
        for Kp in KP_COARSE
    ])
    passing = np.where(coarse_sr >= SR_THRESH)[0]

    if len(passing) == 0:
        return dict(kp_floor=float('nan'), kp_ceil=float('nan'),
                    window=float('nan'), floor_cens=False, ceil_cens=False,
                    infeasible=True, peak_sr=float(coarse_sr.max()), u_max=u_max)

    i_lo, i_hi = int(passing.min()), int(passing.max())

    # Binary floor
    floor_cens = (i_lo == 0)
    if floor_cens:
        kp_floor = float(KP_COARSE[0])
    else:
        lo, hi = np.log(KP_COARSE[i_lo-1]), np.log(KP_COARSE[i_lo])
        for _ in range(BINARY_ITERS):
            mid = (lo + hi) / 2
            sr = _eval_sr(np.exp(mid), opt_Kd, u_max, plant, act, sen, dis, sc, eval_seeds)
            if sr >= SR_THRESH: hi = mid
            else:               lo = mid
        kp_floor = float(np.exp((lo + hi) / 2))

    # Binary ceiling
    ceil_cens = (i_hi == len(KP_COARSE) - 1)
    if ceil_cens:
        kp_ceil = float(KP_COARSE[-1])
    else:
        lo, hi = np.log(KP_COARSE[i_hi]), np.log(KP_COARSE[i_hi+1])
        for _ in range(BINARY_ITERS):
            mid = (lo + hi) / 2
            sr = _eval_sr(np.exp(mid), opt_Kd, u_max, plant, act, sen, dis, sc, eval_seeds)
            if sr >= SR_THRESH: lo = mid
            else:               hi = mid
        kp_ceil = float(np.exp((lo + hi) / 2))

    window = kp_ceil / kp_floor if kp_floor > 0 else float('inf')
    return dict(kp_floor=kp_floor, kp_ceil=kp_ceil, window=window,
                floor_cens=floor_cens, ceil_cens=ceil_cens,
                infeasible=False, peak_sr=float(coarse_sr.max()), u_max=u_max)


# ── Task dispatch ─────────────────────────────────────────────────────────────

def _run_task(keff_target, keff_actual, opt_Kd, row_dict, slew_on, condition, eval_seeds):
    try:
        result = _measure_floor_ceil(row_dict, opt_Kd, eval_seeds, slew_on=slew_on)
        return dict(
            keff_target=keff_target, keff_actual=keff_actual,
            condition=condition, opt_Kd=opt_Kd,
            slew_deg_s=row_dict['servo_slew_deg_s'],
            max_gimbal=row_dict['max_gimbal_deg'],
            Iyy=row_dict['Iyy'],
            slew_on=slew_on,
            **result,
        )
    except Exception as e:
        return dict(keff_target=keff_target, keff_actual=keff_actual,
                    condition=condition, opt_Kd=opt_Kd,
                    kp_floor=float('nan'), kp_ceil=float('nan'),
                    window=float('nan'), infeasible=True, error=str(e))


def main():
    eval_seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))

    # ── Step 1: Joint Kd search at baseline ──────────────────────────────────
    print("=" * 65)
    print("Step 1: Joint Kd search (baseline: gimbal=10, slew=120, lat=6)")
    print("=" * 65)
    ref = []
    for keff_t, iyy in zip(KEFF_TARGETS, IYY_BASE):
        r    = _row(iyy, max_gimbal=BASE_GIMBAL, slew_deg_s=BASE_SLEW)
        p, a, se, di, sc = _build(r, slew_on=True)
        opt_Kp, opt_Kd, sr0 = _joint_search(p, a, se, di, sc)
        ka = _keff_from_row(r)
        print(f"  keff_target={keff_t:.0f}  keff_actual={ka:.2f}  Iyy={iyy:.5f}  "
              f"opt_Kp={opt_Kp:.1f}  opt_Kd={opt_Kd:.2f}  search_sr={sr0:.2f}")
        ref.append({'keff_target': keff_t, 'keff_actual': ka, 'Iyy': iyy, 'opt_Kd': opt_Kd})

    # ── Step 2: Build task list ───────────────────────────────────────────────
    tasks = []
    for d in ref:
        keff_t = d['keff_target']
        keff_a = d['keff_actual']
        iyy    = d['Iyy']
        Kd     = d['opt_Kd']

        # E1a: baseline
        tasks.append((keff_t, keff_a, Kd,
                      _row(iyy, BASE_GIMBAL, BASE_SLEW), True, 'E1_baseline'))

        # E1b: no slew (slew=False in FidelityConfig, everything else same)
        tasks.append((keff_t, keff_a, Kd,
                      _row(iyy, BASE_GIMBAL, BASE_SLEW), False, 'E1_no_slew'))

        # E2: slew sweep (vary S, fixed Iyy/gimbal/tau)
        for s in SLEW_VALUES:
            tasks.append((keff_t, keff_a, Kd,
                          _row(iyy, BASE_GIMBAL, s), True, f'E2_slew{int(s):03d}'))

        # E3: gimbal/u_max sweep (vary max_gimbal, compensate Iyy to keep keff fixed)
        for g in GIMBAL_VALUES:
            # keff = F15 * ms * (pi/180 * g/12) * L / Iyy -> Iyy = _KC * (g/BASE_GIMBAL) / keff_t
            iyy_g = _KC * (g / BASE_GIMBAL) / keff_t
            keff_check = _keff_from_row(_row(iyy_g, g, BASE_SLEW))
            tasks.append((keff_t, keff_check, Kd,
                          _row(iyy_g, g, BASE_SLEW), True, f'E3_gimbal{int(g):02d}'))

    n_tasks = len(tasks)
    n_sims  = len(KP_COARSE)*10 + BINARY_ITERS*2*N_EVAL
    print(f"\nStep 2: {n_tasks} conditions × ~{n_sims} sims = ~{n_tasks*n_sims:,} total")
    print(f"Eval seeds: {N_EVAL} ({EVAL_SEED0}–{EVAL_SEED0+N_EVAL-1})")

    results = Parallel(n_jobs=N_JOBS, verbose=5)(
        delayed(_run_task)(*t, eval_seeds) for t in tasks
    )

    df = pd.DataFrame(results).sort_values(['keff_target', 'condition']).reset_index(drop=True)
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(df)} rows -> {OUT_CSV}\n")

    # ── Analysis ──────────────────────────────────────────────────────────────

    def _tag(r):
        if r.get('infeasible', False): return 'INFEASIBLE'
        parts = [f"floor={r['kp_floor']:.2f}"]
        if not r.get('ceil_cens', False): parts.append(f"ceil={r['kp_ceil']:.1f}")
        else: parts.append(f"ceil>{KP_COARSE[-1]:.0f}")
        if r.get('floor_cens', False): parts[0] = f"floor<{KP_COARSE[0]:.2f} [cens]"
        parts.append(f"win={r['window']:.0f}x")
        return '  '.join(parts)

    # ── E1: Ablation ──────────────────────────────────────────────────────────
    print("=" * 65)
    print("E1: Ablation — slew vs no_slew")
    print("=" * 65)
    e1 = df[df['condition'].isin(['E1_baseline', 'E1_no_slew'])]
    for keff_t in KEFF_TARGETS:
        rows = e1[np.isclose(e1['keff_target'], keff_t)]
        print(f"\n  keff={keff_t:.0f} rad/s²/CU  (Iyy={IYY_BASE[KEFF_TARGETS==keff_t][0]:.5f})")
        for _, r in rows.iterrows():
            print(f"    {r['condition']:20s}  {_tag(r)}")

    # ── E2: Slew sweep ────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("E2: Slew sweep — does floor ∝ S?")
    print(f"    Theory: Kp_floor = Kd·keff·(S/3000·lat)·lat / θ_max")
    print(f"    (S/3000 converts deg/s to CU/step; θ_max ≈ 0.262 rad)")
    print("=" * 65)

    e2 = df[df['condition'].str.startswith('E2_')]
    for keff_t in KEFF_TARGETS:
        rows = e2[np.isclose(e2['keff_target'], keff_t)].sort_values('slew_deg_s')
        valid = rows[~rows.get('infeasible', pd.Series(False, index=rows.index)) &
                     ~rows.get('floor_cens', pd.Series(False, index=rows.index))]
        print(f"\n  keff={keff_t:.0f}:")
        print(f"  {'S (deg/s)':>10}  {'S·τ (CU)':>9}  {'floor_Kp':>10}  "
              f"{'ceil_Kp':>9}  {'floor/S·τ':>10}  {'theory':>9}")
        for _, r in rows.iterrows():
            s_tau  = r['slew_deg_s'] / 3000 * LATENCY       # CU per delay window
            theory = float(r['opt_Kd']) * keff_t * s_tau / 0.262
            tag    = 'INFEASIBLE' if r.get('infeasible', False) else \
                     f"{r['kp_floor']:>10.3f}  {r['kp_ceil']:>9.1f}  " \
                     f"{r['kp_floor']/s_tau if s_tau>0 else 0:>10.3f}  {theory:>9.3f}"
            print(f"  {r['slew_deg_s']:>10.0f}  {s_tau:>9.4f}  {tag}")

        if len(valid) >= 3:
            x    = np.log(valid['slew_deg_s'].values)
            y    = np.log(valid['kp_floor'].values)
            coef = np.polyfit(x, y, 1)
            resid = y - np.polyval(coef, x)
            r2   = 1 - resid.var() / y.var()
            print(f"\n  Power law: floor ∝ S^{coef[0]:.2f}  R²={r2:.3f}")
            print(f"  Theory predicts exponent = +1.00")
            if abs(coef[0] - 1.0) < 0.3:
                print("  -> CONSISTENT with slew-driven floor")
            elif coef[0] < 0.3:
                print("  -> INCONSISTENT: floor nearly independent of S")
            else:
                print(f"  -> PARTIAL: exponent {coef[0]:.2f}, not cleanly +1.0")

    # ── E3: Amplitude/u_max sweep ─────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("E3: Amplitude sweep — does floor depend on u_max at fixed keff?")
    print(f"    Slew-dominant prediction: floor ∝ gimbal^0.00 (independent)")
    print(f"    Amplitude-dominant prediction: floor ∝ gimbal^+1.00")
    print("=" * 65)

    e3 = df[df['condition'].str.startswith('E3_')]
    for keff_t in KEFF_TARGETS:
        rows = e3[np.isclose(e3['keff_target'], keff_t)].sort_values('max_gimbal')
        valid = rows[~rows.get('infeasible', pd.Series(False, index=rows.index)) &
                     ~rows.get('floor_cens', pd.Series(False, index=rows.index))]
        print(f"\n  keff={keff_t:.0f}:")
        print(f"  {'gimbal(°)':>10}  {'u_max(CU)':>10}  {'Iyy':>8}  "
              f"{'keff_actual':>12}  {'floor_Kp':>10}  {'window':>9}")
        for _, r in rows.iterrows():
            umax = r['max_gimbal'] * 12.0 / 15.0
            if r.get('infeasible', False):
                tag = f"{'INFEASIBLE':>10}"
            else:
                tag = f"{r['kp_floor']:>10.3f}  {r['window']:>9.0f}x"
            print(f"  {r['max_gimbal']:>10.1f}  {umax:>10.2f}  {r['Iyy']:>8.5f}  "
                  f"{r['keff_actual']:>12.2f}  {tag}")

        if len(valid) >= 3:
            x    = np.log(valid['max_gimbal'].values)
            y    = np.log(valid['kp_floor'].values)
            coef = np.polyfit(x, y, 1)
            resid = y - np.polyval(coef, x)
            r2   = 1 - resid.var() / y.var()
            print(f"\n  Power law: floor ∝ gimbal^{coef[0]:.2f}  R²={r2:.3f}")
            print(f"  (u_max ∝ gimbal; slew-dominant predicts 0.00, amplitude-dominant +1.00)")
            if coef[0] < 0.3:
                print("  -> SLEW-DOMINANT: floor is independent of amplitude limit")
            elif coef[0] > 0.7:
                print("  -> AMPLITUDE-DOMINANT: floor scales with u_max")
            else:
                print(f"  -> MIXED: intermediate exponent {coef[0]:.2f}")

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("Summary: mechanism diagnosis")
    print("=" * 65)
    e1b = df[df['condition'] == 'E1_no_slew']
    e1a = df[df['condition'] == 'E1_baseline']
    for keff_t in KEFF_TARGETS:
        base = e1a[np.isclose(e1a['keff_target'], keff_t)]
        noslew = e1b[np.isclose(e1b['keff_target'], keff_t)]
        if len(base) and len(noslew):
            f_base   = base.iloc[0]['kp_floor']
            f_noslew = noslew.iloc[0]['kp_floor']
            if noslew.iloc[0].get('infeasible', False): f_noslew = float('nan')
            if base.iloc[0].get('floor_cens', False):   f_base   = float('nan')
            if noslew.iloc[0].get('floor_cens', False): f_noslew = float('nan')
            change = f"floor: {f_base:.2f} -> {f_noslew:.2f} ({f_noslew/f_base*100:.0f}%)" \
                     if not (np.isnan(f_base) or np.isnan(f_noslew)) else "data unavailable"
            print(f"  keff={keff_t:.0f}: removing slew: {change}")


if __name__ == '__main__':
    main()
