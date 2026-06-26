"""
tools/floor_formula_holdout.py

Held-out validation of the floor/ceiling/window formula on fresh LHS designs.

Validates:
  Kp_floor_predicted = 0.06 * keff^1.06 * lat^0.96    (fitted regression, v2)
  Kp_floor_simple    = 0.06 * keff * lat               (theoretical integers)
  Kp_floor_slew      = 0.06 * (servo_slew/120) * keff * lat  (slew-corrected)
  Kp_ceil_predicted  = 380 / lat                        (theoretical)
  Kp_ceil_empirical  = 520 / lat^0.86                   (empirical from regression)
  Window_predicted   = Kp_ceil / Kp_floor               (ratio)

IMPORTANT: Uses a completely fresh LHS pool (seed=5555) and eval seeds 50001-50020.
These are disjoint from ALL prior experiments:
  - Pool seeds used before: 8888 (v1/v2/v3), 999 (regression_extension), 7777 (lat_ext)
  - Eval seeds used before: 1-3 (search), 20001-20015 (v1), 22001-22015 (v2),
    23001-23020 (v3), 25001-25015 (lat_ext), 80001-80030 (floor_mechanism_test),
    plus many others in exp1 correction (1001-1030), frontier (7001-7015), etc.

Stratification: 5 keff bins x 4 lat bins, 4 per cell = 80 designs.
Uses keff bins (not td bins) since keff is the formula variable.

Outputs:
  experiments/results/floor_formula_holdout_py.csv   (80 designs, summary)
  experiments/results/floor_formula_sweep_py.csv     (80 x 32 Kp sweep rows)
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

# Gain search: 18x7 joint grid, 3 seeds (standard)
KP_SRCH_MIN, KP_SRCH_MAX, N_KP_SRCH = 1.0, 320.0, 18
KD_SRCH_MIN, KD_SRCH_MAX, N_KD_SRCH = 1.0, 64.0,  7
SEARCH_SEEDS = [1, 2, 3]

# Kp sweep: 32 points over [0.05, 800], 20 eval seeds
KP_SWEEP      = np.geomspace(0.05, 800.0, 32)   # 1.34x per step
N_SWEEP_SEEDS = 20
SWEEP_SEED0   = 50001                            # disjoint from ALL prior experiments
SR_THRESH     = 0.80

# LHS pool: fresh seed (not 8888, 999, or 7777)
N_POOL     = 10_000
POOL_SEED  = 5555
N_PER_CELL = 4

# Stratify on keff (the formula variable), not td
KEFF_BINS   = [0, 5, 12, 22, 35, 1000]
KEFF_LABELS = ['keff0-5', 'keff5-12', 'keff12-22', 'keff22-35', 'keff35+']
LAT_BINS    = [0, 2, 3, 5, 6]      # {1-2}, {3}, {4-5}, {6}
LAT_LABELS  = ['lat1-2', 'lat3', 'lat4-5', 'lat6']

OUT_CSV   = Path('experiments/results/floor_formula_holdout_py.csv')
OUT_SWEEP = Path('experiments/results/floor_formula_sweep_py.csv')

# Floor formula coefficients (fitted on window_ratio_v2, n=104 non-censored)
FLOOR_COEF  = 0.06
FLOOR_EXP_K = 1.06
FLOOR_EXP_L = 0.96

# Ceiling formula
CEIL_THEORY_A = 380.0   # Kp_ceil = A / lat
CEIL_EMP_A    = 520.0   # empirical: A / lat^0.86
CEIL_EMP_EXP  = 0.86


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
    lat = int(row['latency_steps'])
    slew_deg_s = float(row.get('servo_slew_deg_s', 120.0))

    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    # Joint Kd search
    opt_Kp, opt_Kd = _joint_search(plant, act, sen, dis, sc)

    # Kp sweep (fixed opt_Kd)
    sweep_seeds = list(range(SWEEP_SEED0, SWEEP_SEED0 + N_SWEEP_SEEDS))
    sr_vals     = []
    sweep_rows  = []
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
        infeasible = True
    else:
        kp_floor     = float(passing.min())
        kp_ceiling   = float(passing.max())
        window_ratio = kp_ceiling / kp_floor
        floor_cens   = kp_floor   <= KP_SWEEP[0]  * 1.05
        ceil_cens    = kp_ceiling >= KP_SWEEP[-1] * 0.95
        infeasible   = False

    # Formula predictions
    floor_fitted  = FLOOR_COEF * (keff ** FLOOR_EXP_K) * (lat ** FLOOR_EXP_L)
    floor_simple  = FLOOR_COEF * keff * lat
    floor_slew    = FLOOR_COEF * (slew_deg_s / 120.0) * keff * lat
    ceil_theory   = CEIL_THEORY_A / lat
    ceil_empirical = CEIL_EMP_A / (lat ** CEIL_EMP_EXP)
    window_fitted = ceil_theory / floor_fitted
    window_simple = ceil_theory / floor_simple

    summary = dict(
        rocket_id        = row['rocket_id'],
        td               = td,
        keff             = keff,
        u_max            = u_max,
        latency_steps    = lat,
        Iyy              = row['Iyy'],
        motor_scale      = row['motor_scale'],
        max_gimbal_deg   = row['max_gimbal_deg'],
        wind_strength    = row['wind_strength'],
        servo_slew       = slew_deg_s,
        static_margin    = row.get('static_margin', 0.0),
        opt_Kp           = opt_Kp,
        opt_Kd           = opt_Kd,
        kp_floor         = kp_floor,
        kp_ceiling       = kp_ceiling,
        window_ratio     = window_ratio,
        peak_sr          = float(sr_arr.max()),
        floor_censored   = floor_cens,
        ceil_censored    = ceil_cens,
        infeasible       = infeasible,
        # Predictions
        pred_floor_fitted    = floor_fitted,
        pred_floor_simple    = floor_simple,
        pred_floor_slew      = floor_slew,
        pred_ceil_theory     = ceil_theory,
        pred_ceil_empirical  = ceil_empirical,
        pred_window_fitted   = window_fitted,
        pred_window_simple   = window_simple,
        keff_bin         = str(row.get('keff_bin', '')),
        lat_bin          = str(row.get('lat_bin', '')),
    )
    return summary, sweep_rows


def _safe_eval(row: dict):
    try:
        return _eval_design(row)
    except Exception as exc:
        print(f"  ERROR {row.get('rocket_id','?')}: {exc}")
        import traceback; traceback.print_exc()
        return None, []


# ── Validation metrics ────────────────────────────────────────────────────────
def _validate(pred: np.ndarray, meas: np.ndarray, name: str):
    """Compute validation metrics for a floor/ceiling prediction."""
    valid = np.isfinite(pred) & np.isfinite(meas) & (meas > 0) & (pred > 0)
    p, m = pred[valid], meas[valid]
    n = int(valid.sum())

    log_p, log_m = np.log(p), np.log(m)
    r2_log = float(pearsonr(log_p, log_m)[0] ** 2)
    rho, p_val = spearmanr(p, m)

    ratio = p / m
    mare = float(np.median(np.abs(ratio - 1)))
    within_2x = float(np.mean((ratio >= 0.5) & (ratio <= 2.0)))
    within_3x = float(np.mean((ratio >= 1/3) & (ratio <= 3.0)))
    median_ratio = float(np.median(ratio))

    print(f"\n  {name} (n={n}):")
    print(f"    R2(log-log)  = {r2_log:.3f}")
    print(f"    Spearman rho = {rho:.3f}  (p={p_val:.3e})")
    print(f"    Median ratio = {median_ratio:.2f}x  (pred/meas)")
    print(f"    MARE         = {mare:.2f}  ({mare*100:.0f}% typical error)")
    print(f"    Within 2x: {within_2x*100:.0f}%   Within 3x: {within_3x*100:.0f}%")
    return dict(name=name, n=n, r2_log=r2_log, rho=rho, p_val=p_val,
                median_ratio=median_ratio, mare=mare,
                within_2x=within_2x, within_3x=within_3x)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print("=" * 70)
    print("Floor/Ceiling Formula Held-Out Validation")
    print(f"LHS pool seed: {POOL_SEED} (fresh, never used in formula fitting)")
    print(f"Eval seeds: {SWEEP_SEED0}-{SWEEP_SEED0+N_SWEEP_SEEDS-1} (disjoint from all prior)")
    print("=" * 70)

    # Generate fresh LHS pool
    pool = sample_lhs(N_POOL, seed=POOL_SEED)
    T_avg_arr     = F15_AVG_THRUST_N * pool['motor_scale']
    pool['keff']  = T_avg_arr * CU_TO_RAD * L_NOZZLE / pool['Iyy']
    pool['u_max'] = pool['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    pool['td']    = pool['keff'] * pool['u_max']

    pool['keff_bin'] = pd.cut(pool['keff'], bins=KEFF_BINS,
                               labels=KEFF_LABELS, right=False)
    pool['lat_bin']  = pd.cut(pool['latency_steps'], bins=LAT_BINS,
                               labels=LAT_LABELS, right=True)

    rng = np.random.RandomState(42)
    chosen = []
    for k_lbl in KEFF_LABELS:
        for l_lbl in LAT_LABELS:
            cell = pool[(pool['keff_bin'] == k_lbl) & (pool['lat_bin'] == l_lbl)]
            n = min(N_PER_CELL, len(cell))
            if n > 0:
                chosen.append(cell.sample(n=n, random_state=rng))
    selected = pd.concat(chosen).reset_index(drop=True)
    selected['keff_bin'] = selected['keff_bin'].astype(str)
    selected['lat_bin']  = selected['lat_bin'].astype(str)

    print(f"\nPool: {len(pool)} designs  keff=[{pool['keff'].min():.1f}, {pool['keff'].max():.1f}]")
    print(f"Selected: {len(selected)} designs")
    print(f"  keff range: [{selected['keff'].min():.1f}, {selected['keff'].max():.1f}]")
    print(f"  lat range: {sorted(selected['latency_steps'].unique().tolist())}")

    print("\nCell coverage (keff x lat):")
    for k_lbl in KEFF_LABELS:
        counts = [str(len(selected[(selected['keff_bin']==k_lbl) & (selected['lat_bin']==l_lbl)]))
                  for l_lbl in LAT_LABELS]
        print(f"  {k_lbl:12} | " + "  ".join(f"{LAT_LABELS[i]}:{counts[i]}" for i in range(len(LAT_LABELS))))

    sims_per = N_KP_SRCH * N_KD_SRCH * len(SEARCH_SEEDS) + len(KP_SWEEP) * N_SWEEP_SEEDS
    total = sims_per * len(selected)
    print(f"\nSims per design: {sims_per}  Total: ~{total:,}")
    print("Running...\n")

    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(_safe_eval)(row) for row in selected.to_dict('records')
    )
    summaries = [r[0] for r in raw if r[0] is not None]
    all_sweep = [s for r in raw for s in r[1]]

    out = pd.DataFrame(summaries)
    out.to_csv(OUT_CSV, index=False)
    pd.DataFrame(all_sweep).to_csv(OUT_SWEEP, index=False)
    print(f"\nSaved {len(out)} rows -> {OUT_CSV}")

    # ── Analysis ──────────────────────────────────────────────────────────────
    valid = out.dropna(subset=['kp_floor', 'kp_ceiling', 'window_ratio']).copy()
    valid = valid[valid['window_ratio'] > 0]

    # Exclude censored designs from floor validation (floor can't be measured below 0.05)
    no_fc = valid[~valid['floor_censored']]
    no_cc = valid[~valid['ceil_censored']]
    no_cens = valid[~valid['floor_censored'] & ~valid['ceil_censored']]

    n_tot   = len(out)
    n_inf   = int(out['infeasible'].sum()) if 'infeasible' in out.columns else 0
    n_val   = len(valid)
    n_fc    = int(valid['floor_censored'].sum())
    n_cc    = int(valid['ceil_censored'].sum())

    print(f"\n{'='*70}")
    print(f"RESULTS SUMMARY")
    print(f"{'='*70}")
    print(f"\nDesigns: total={n_tot}  infeasible={n_inf}  valid={n_val}")
    print(f"  Floor censored: {n_fc}/{n_val} ({n_fc/n_val*100:.0f}%)")
    print(f"  Ceiling censored: {n_cc}/{n_val} ({n_cc/n_val*100:.0f}%)")
    print(f"  Non-censored (both): {len(no_cens)}")

    # Formula predictions use keff+lat only (the two-variable formula)
    print(f"\n{'='*50}")
    print("FLOOR VALIDATION (non-floor-censored designs)")
    print(f"{'='*50}")
    if len(no_fc) > 0:
        meas_floor = no_fc['kp_floor'].values
        _validate(no_fc['pred_floor_fitted'].values,  meas_floor, 'Fitted (keff^1.06 x lat^0.96)')
        _validate(no_fc['pred_floor_simple'].values,  meas_floor, 'Simple (keff x lat)')
        _validate(no_fc['pred_floor_slew'].values,    meas_floor, 'Slew-corrected (S/120 x keff x lat)')

    print(f"\n{'='*50}")
    print("CEILING VALIDATION (non-ceiling-censored designs)")
    print(f"{'='*50}")
    if len(no_cc) > 0:
        meas_ceil = no_cc['kp_ceiling'].values
        _validate(no_cc['pred_ceil_theory'].values,   meas_ceil, 'Theory (380/lat)')
        _validate(no_cc['pred_ceil_empirical'].values, meas_ceil, 'Empirical (520/lat^0.86)')

    print(f"\n{'='*50}")
    print("WINDOW RATIO VALIDATION (both non-censored)")
    print(f"{'='*50}")
    if len(no_cens) > 0:
        meas_win = no_cens['window_ratio'].values
        _validate(no_cens['pred_window_fitted'].values, meas_win, 'Predicted window (380/lat / floor_fitted)')
        _validate(no_cens['pred_window_simple'].values, meas_win, 'Simple window (380/lat / floor_simple)')

    # Show keff x lat breakdown of floor residual
    print(f"\n{'='*50}")
    print("FLOOR RESIDUAL BY KEFF BIN (fitted formula, non-censored)")
    print(f"{'='*50}")
    for k_lbl in KEFF_LABELS:
        sub = no_fc[no_fc['keff_bin'] == k_lbl]
        if len(sub) < 2:
            continue
        ratio = sub['pred_floor_fitted'] / sub['kp_floor']
        print(f"  {k_lbl:12}: n={len(sub):2d}  median pred/meas={ratio.median():.2f}x  "
              f"IQR=[{ratio.quantile(0.25):.2f}, {ratio.quantile(0.75):.2f}]")

    print(f"\n{'='*50}")
    print("FLOOR RESIDUAL BY LAT BIN (fitted formula, non-censored)")
    print(f"{'='*50}")
    for l_lbl in LAT_LABELS:
        sub = no_fc[no_fc['lat_bin'] == l_lbl]
        if len(sub) < 2:
            continue
        ratio = sub['pred_floor_fitted'] / sub['kp_floor']
        print(f"  {l_lbl:8}: n={len(sub):2d}  median pred/meas={ratio.median():.2f}x  "
              f"IQR=[{ratio.quantile(0.25):.2f}, {ratio.quantile(0.75):.2f}]")

    print(f"\nDone. Data in {OUT_CSV}")


if __name__ == '__main__':
    main()
