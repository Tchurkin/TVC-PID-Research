"""
tools/s_scaling_tradeoff.py

Experiment: software servo-output scaling tradeoff.

A builder with a FRAGILE (high-keff) rocket can scale servo output by
factor s in firmware, capping max commanded deflection at s * max_gimbal.
This changes:
    keff_eff       = s * keff         (floor drops by s)
    max torque_eff = s * T*sin(d)*L   (max corrective torque drops by s)
    ceiling        = 380 / latency    (UNCHANGED — keff-independent)
    window         ≈ window(1.0) / s  (WIDENS by 1/s)

But wind disturbance is UNCHANGED. Below some s_critical, the servo can
no longer reject wind → SR drops. Below s_min, the design is INFEASIBLE.

Implementation: keff_fault_post = s, fault_time_s = 0.0 scales M_TVC by s
from t=0 in both simple and full-physics modes, leaving wind unchanged.
This is exactly equivalent to capping servo output at s * max_gimbal.

Experiment design:
  - 5 FRAGILE designs spanning keff range (from exp1_final_population_py.csv)
  - 2 near-boundary EASY designs for comparison
  - s in [1.00, 0.70, 0.50, 0.35, 0.25, 0.15, 0.08]
  - For each (design, s) pair (parallel):
      1. Joint 18x7 Kp*Kd search (3 seeds) to find best gain at this s
      2. Kp sweep 24pts [0.05, 500] with Kd frozen (12 eval seeds)
      3. Record: floor, ceiling, window_ratio, SR_at_window_center

Outputs:
    experiments/results/s_scaling_tradeoff_py.csv      (per-design-per-s)
    experiments/results/s_scaling_sweep_py.csv         (per-Kp SR detail)

Predictions to verify:
    floor(s)   ≈ s  * floor(1.0)
    ceiling(s) ≈ ceiling(1.0)
    window(s)  ≈ window(1.0) / s
    SR_opt(s)  improves then degrades (optimal s* exists)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

# ── Experiment parameters ─────────────────────────────────────────────────────

S_VALUES       = [1.00, 0.70, 0.50, 0.35, 0.25, 0.15, 0.08]
KP_SWEEP       = np.geomspace(0.01, 500.0, 28)   # 28 pts, floor at 0.01, ~1.38x per step
N_SEARCH_SEEDS = 3
SEARCH_SEED0   = 1
N_EVAL_SEEDS   = 12
EVAL_SEED0     = 40001   # disjoint from all prior experiments (new seed range)
SR_THRESH      = 0.80
N_JOBS         = -1      # use all CPUs

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# ── Systematic Kd tuning (critical-damping formula) ────────────────────────────
# For theta_ddot = keff_eff * u, closed-loop characteristic:
#   s^2 + keff_eff*Kd*s + keff_eff*Kp = 0
# Critical damping (zeta=1): Kd = 2*sqrt(Kp/keff_eff)
# Slightly underdamped (zeta=0.7) balances speed vs overshoot.
ZETA_VALUES = [0.0, 0.5, 0.7, 1.0]  # 0.0 = pure P (Kd=0); test all, pick best SR→RMSE

def analytical_Kd(Kp, keff_eff, zeta):
    """Kd for damping ratio zeta. Returns 0 for pure-P (zeta=0)."""
    if zeta == 0.0 or keff_eff <= 0:
        return 0.0
    return 2.0 * zeta * np.sqrt(max(Kp, 1e-6) / keff_eff)

# ── Gain search grids (same as final correction) ───────────────────────────────

KP_COARSE = np.geomspace(0.5, 400.0, 18)
KD_GRID   = [0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0]  # 0.0 = pure proportional


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _keff_from_row(row):
    T = F15_AVG_THRUST_N * float(row['motor_scale'])
    return T * CU_TO_RAD * L_NOZZLE / float(row['Iyy'])


def _td_from_row(row):
    return _keff_from_row(row) * float(row['max_gimbal_deg']) * REF_U_MAX / REF_MAX_GIMBAL_DEG


# ── Single (design, s) worker ─────────────────────────────────────────────────

def _run_design_s(row_dict, s):
    """Full search + sweep for one design at one s value. Returns (summary_dict, sweep_list)."""
    rid   = row_dict['rocket_id']
    keff  = _keff_from_row(row_dict)
    td    = _td_from_row(row_dict)
    lat   = int(row_dict['latency_steps'])
    regime = row_dict.get('final_label', 'UNKNOWN')

    search_seeds = list(range(SEARCH_SEED0, SEARCH_SEED0 + N_SEARCH_SEEDS))
    eval_seeds   = list(range(EVAL_SEED0,   EVAL_SEED0   + N_EVAL_SEEDS))

    # Build simulator params
    plant = build_plant(row_dict)
    act   = build_actuator(row_dict)
    sen   = build_sensor(row_dict)
    dis   = build_disturbance(row_dict)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    # Apply servo scaling via keff_fault
    sc.fault_time_s    = 0.0
    sc.keff_fault_post = float(s)

    # ── 1. Joint Kp*Kd search ────────────────────────────────────────────────
    best_sr  = -1.0
    best_Kp  = 10.0
    best_Kd  = 1.0
    best_rms = float('inf')

    for Kp in KP_COARSE:
        for Kd in KD_GRID:
            pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                            u_max=act.u_max, i_lim=act.u_max)
            results = [_run_one(pid, plant, act, sen, dis, sc, seed=seed)
                       for seed in search_seeds]
            sr  = float(np.mean([r['success'] for r in results]))
            rms = float(np.mean([r['rms_error_deg'] for r in results]))
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr  = sr
                best_Kp  = float(Kp)
                best_Kd  = float(Kd)
                best_rms = rms

    # ── 2. Kp sweep with analytical adaptive Kd ──────────────────────────────
    # At each Kp: test ZETA_VALUES, pick (Kp, Kd) with best SR then lowest RMS.
    keff_eff   = keff * s
    sr_vals    = []
    kd_used    = []
    sweep_rows = []
    for Kp in KP_SWEEP:
        best_sr_kd  = -1.0
        best_rms_kd = float('inf')
        best_Kd_kp  = 0.0
        for zeta in ZETA_VALUES:
            Kd_z = analytical_Kd(float(Kp), keff_eff, zeta)
            pid  = PIDParams(Kp=float(Kp), Kd=Kd_z, Ki=0.0,
                             u_max=act.u_max, i_lim=act.u_max)
            results = [_run_one(pid, plant, act, sen, dis, sc, seed=seed)
                       for seed in eval_seeds]
            sr  = float(np.mean([r['success']       for r in results]))
            rms = float(np.mean([r['rms_error_deg'] for r in results]))
            if sr > best_sr_kd or (sr == best_sr_kd and rms < best_rms_kd):
                best_sr_kd  = sr
                best_rms_kd = rms
                best_Kd_kp  = Kd_z
        sr_vals.append(best_sr_kd)
        kd_used.append(best_Kd_kp)
        sweep_rows.append(dict(rocket_id=rid, s=float(s), Kp=float(Kp),
                               Kd=best_Kd_kp, sr=best_sr_kd, rms=best_rms_kd))

    sr_arr  = np.array(sr_vals)
    passing = KP_SWEEP[sr_arr >= SR_THRESH]

    if len(passing) == 0:
        kp_floor  = float('nan')
        kp_ceil   = float('nan')
        window    = float('nan')
        floor_cens = False
        ceil_cens  = False
        sr_center  = float(sr_arr.max())
        kp_center  = float(KP_SWEEP[sr_arr.argmax()])
    else:
        kp_floor   = float(passing.min())
        kp_ceil    = float(passing.max())
        window     = kp_ceil / kp_floor
        floor_cens = kp_floor  <= KP_SWEEP[0]  * 1.05
        ceil_cens  = kp_ceil   >= KP_SWEEP[-1] * 0.95
        kp_center  = float(np.sqrt(kp_floor * kp_ceil))
        idx_cen    = int(np.argmin(np.abs(KP_SWEEP - kp_center)))
        sr_center  = float(sr_arr[idx_cen])

    summary = dict(
        rocket_id       = rid,
        regime          = regime,
        keff            = keff,
        td              = td,
        latency_steps   = lat,
        s               = float(s),
        keff_eff        = keff_eff,
        td_eff          = td * s,
        best_Kp_search  = best_Kp,
        best_Kd_search  = best_Kd,
        search_sr       = best_sr,
        kp_floor        = kp_floor,
        kp_ceil         = kp_ceil,
        kd_at_floor     = kd_used[np.argmin(np.abs(KP_SWEEP - kp_floor))] if not np.isnan(kp_floor) else float('nan'),
        kd_at_ceil      = kd_used[np.argmin(np.abs(KP_SWEEP - kp_ceil))]  if not np.isnan(kp_ceil)  else float('nan'),
        window_ratio    = window,
        floor_censored  = floor_cens,
        ceil_censored   = ceil_cens,
        sr_at_center    = sr_center,
        kp_center       = kp_center,
    )
    return summary, sweep_rows


# ── Design selection ───────────────────────────────────────────────────────────

def select_designs(pop_csv):
    df = pd.read_csv(pop_csv)

    T_vals    = F15_AVG_THRUST_N * df['motor_scale']
    df['keff'] = T_vals * CU_TO_RAD * L_NOZZLE / df['Iyy']
    df['td']   = df['keff'] * df['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG

    fragile = df[df['final_label'] == 'FRAGILE'].copy().sort_values('keff')
    easy    = df[df['final_label'] == 'EASY'].copy().sort_values('keff', ascending=False)

    # 5 FRAGILE across keff quintiles
    n    = len(fragile)
    idxs = [int(n * q) for q in [0.10, 0.30, 0.50, 0.70, 0.90]]
    idxs = [min(i, n - 1) for i in idxs]
    chosen_fragile = fragile.iloc[idxs]

    # 2 highest-keff EASY (near boundary)
    chosen_easy = easy.head(2)

    chosen = pd.concat([chosen_fragile, chosen_easy], ignore_index=True)

    print(f"Selected {len(chosen)} designs:")
    for _, r in chosen.iterrows():
        print(f"  {r['rocket_id']}  {r['final_label']}  keff={r['keff']:.2f}"
              f"  td={r['td']:.1f}  lat={r['latency_steps']}")
    return chosen


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    pop_csv = RESULTS / 'exp1_final_population_py.csv'
    if not pop_csv.exists():
        raise FileNotFoundError(pop_csv)

    chosen = select_designs(pop_csv)

    # Build flat list of (row_dict, s) tasks
    tasks = [(row.to_dict(), s)
             for _, row in chosen.iterrows()
             for s in S_VALUES]

    n_sims_per_task = N_SEARCH_SEEDS * 18 * 7 + N_EVAL_SEEDS * 24
    print(f"\n{len(tasks)} tasks × ~{n_sims_per_task} sims = ~{len(tasks)*n_sims_per_task:,} total sims")
    print(f"Running in parallel (N_JOBS={N_JOBS})…\n")

    results = Parallel(n_jobs=N_JOBS, verbose=10)(
        delayed(_run_design_s)(row_dict, s) for row_dict, s in tasks
    )

    all_summary = [r[0] for r in results]
    all_sweeps  = [row for r in results for row in r[1]]

    df_sum = pd.DataFrame(all_summary)

    # ── Add predictions relative to s=1.0 ─────────────────────────────────────
    anchor = df_sum[df_sum['s'] == 1.0].set_index('rocket_id')[['kp_floor', 'kp_ceil', 'window_ratio']]
    df_sum['floor_at_s1']  = df_sum['rocket_id'].map(anchor['kp_floor'])
    df_sum['ceil_at_s1']   = df_sum['rocket_id'].map(anchor['kp_ceil'])
    df_sum['window_at_s1'] = df_sum['rocket_id'].map(anchor['window_ratio'])

    df_sum['pred_floor']  = df_sum['floor_at_s1'] * df_sum['s']
    df_sum['pred_ceil']   = df_sum['ceil_at_s1']
    df_sum['pred_window'] = df_sum['ceil_at_s1'] / (df_sum['floor_at_s1'] * df_sum['s'])

    out_sum   = RESULTS / 's_scaling_tradeoff_py.csv'
    out_sweep = RESULTS / 's_scaling_sweep_py.csv'
    df_sum.to_csv(out_sum, index=False)
    pd.DataFrame(all_sweeps).to_csv(out_sweep, index=False)
    print(f"\nSaved {len(df_sum)} rows → {out_sum.name}")
    print(f"Saved {len(all_sweeps)} rows → {out_sweep.name}")

    # ── Analysis ───────────────────────────────────────────────────────────────
    print("\n=== FORMULA VALIDATION (observed vs predicted window) ===")
    valid = df_sum[df_sum['window_ratio'].notna()
                   & ~df_sum['floor_censored']
                   & ~df_sum['ceil_censored']]

    for rid, grp in valid.groupby('rocket_id'):
        row0 = grp[grp['s'] == 1.0]
        if len(row0) == 0:
            continue
        w0     = row0.iloc[0]['window_ratio']
        regime = row0.iloc[0]['regime']
        keff   = row0.iloc[0]['keff']
        print(f"\n  {rid} ({regime}, keff={keff:.2f})  baseline window={w0:.1f}x")
        for _, r in grp.sort_values('s').iterrows():
            if np.isnan(r['window_ratio']):
                print(f"    s={r['s']:.2f}: NO WINDOW (insufficient authority) SR_best={r['sr_at_center']:.3f}")
            else:
                obs_p = r['window_ratio'] / r['pred_window'] if r['pred_window'] > 0 else float('nan')
                print(f"    s={r['s']:.2f}: window={r['window_ratio']:.1f}x "
                      f"pred={r['pred_window']:.1f}x (ratio={obs_p:.2f}) "
                      f"SR={r['sr_at_center']:.3f}")

    print("\n=== OPTIMAL S* PER DESIGN ===")
    for rid, grp in df_sum.groupby('rocket_id'):
        best   = grp.loc[grp['sr_at_center'].idxmax()]
        s1     = grp[grp['s'] == 1.0].iloc[0] if len(grp[grp['s'] == 1.0]) > 0 else None
        sr_s1  = s1['sr_at_center']    if s1 is not None else float('nan')
        w_s1   = s1['window_ratio']    if s1 is not None else float('nan')
        regime = grp.iloc[0]['regime']
        print(f"  {rid} ({regime}, keff={grp.iloc[0]['keff']:.2f}): "
              f"s*={best['s']:.2f}  SR*={best['sr_at_center']:.3f} "
              f"(vs SR@s=1: {sr_s1:.3f})  "
              f"window@s*={best['window_ratio']:.1f}x (vs {w_s1:.1f}x @s=1)")


if __name__ == '__main__':
    main()
