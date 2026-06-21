"""
tools/regression_extension.py

Extends the continuous margin regression (combined_margin_regression_py.csv)
above theta_ddot_max > 300 rad/s^2, where only n=12 data points currently exist
(too few to characterise the high-authority tail reliably; CV R^2 CI is ±0.10).

This pass targets 40 designs with td > 300 using the SAME finer joint Kp x Kd
search and 15-seed evaluation used in the original two passes, with disjoint seeds.

Output is a separate CSV (regression_extension_py.csv) that can be merged with
combined_margin_regression_py.csv for a pooled re-fit.  No overwriting of the
original files.
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario, sample_lhs
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate, UNDER_SCALE, OVER_SCALE
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# ── Identical search and eval settings to the original passes ─────────────────
KP_MIN, KP_MAX, N_KP = 1.0, 320.0, 18
KD_MIN, KD_MAX, N_KD = 1.0, 64.0, 7
SEARCH_SEEDS  = [1, 2, 3]
N_FINAL_SEEDS = 15
FINAL_SEED_START = 8001       # disjoint from pass1 (5001-5015) and pass2 (6001-6015)

# ── Extension target ──────────────────────────────────────────────────────────
TD_MIN_EXT = 300.0            # only designs above this threshold
N_TARGET   = 40               # designs to evaluate
# td>300 requires extreme (low-Iyy + high-motor + high-gimbal), ~0.3% of LHS.
# Need 20k pool to collect ~60 qualifying designs, leaving room for 4-bin stratification.
N_POOL     = 20000
POOL_SEED  = 999

OUT_CSV = Path('experiments/results/regression_extension_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def compute_td(row: dict) -> float:
    T_avg = F15_AVG_THRUST_N * row['motor_scale']
    keff_full = T_avg * CU_TO_RAD * L_NOZZLE / row['Iyy']
    u_max = row['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff_full * u_max)


def _joint_search(plant, act, sen, dis, sc):
    kp_grid = np.geomspace(KP_MIN, KP_MAX, N_KP)
    kd_grid = np.geomspace(KD_MIN, KD_MAX, N_KD)
    best_kp, best_kd, best_sr, best_rms = float(kp_grid[0]), float(kd_grid[0]), -1.0, float("inf")
    for Kp in kp_grid:
        for Kd in kd_grid:
            pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
            runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in SEARCH_SEEDS]
            agg = _aggregate(runs)
            sr, rms = agg["success_rate"], agg["rms_error_deg"]
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr, best_rms, best_kp, best_kd = sr, rms, float(Kp), float(Kd)
    return best_kp, best_kd


def _eval_design(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    new_Kp, new_Kd = _joint_search(plant, act, sen, dis, sc)
    seeds = list(range(FINAL_SEED_START, FINAL_SEED_START + N_FINAL_SEEDS))

    def eval_cond(Kp, Kd):
        pid  = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
        return _aggregate(runs)

    nominal = eval_cond(new_Kp, new_Kd)
    under   = eval_cond(UNDER_SCALE * new_Kp, UNDER_SCALE * new_Kd)
    over    = eval_cond(OVER_SCALE  * new_Kp, OVER_SCALE  * new_Kd)

    return dict(
        rocket_id      = row.get('rocket_id', '?'),
        td             = row['td'],
        latency_steps  = row['latency_steps'],
        Iyy            = row['Iyy'],
        motor_scale    = row['motor_scale'],
        max_gimbal_deg = row['max_gimbal_deg'],
        new_Kp         = new_Kp,
        new_Kd         = new_Kd,
        nom_sr         = nominal['success_rate'],
        under_sr       = under['success_rate'],
        over_sr        = over['success_rate'],
        margin         = min(under['success_rate'], over['success_rate']),
        nom_rms        = nominal['rms_error_deg'],
        pass_label     = 'extension_td300',
    )


def main():
    pool = sample_lhs(N_POOL, seed=POOL_SEED)
    pool['td'] = pool.apply(lambda r: compute_td(r.to_dict()), axis=1)

    high_td = pool[pool['td'] > TD_MIN_EXT].copy()
    print(f"Pool: n={N_POOL}, td>{TD_MIN_EXT}: {len(high_td)} designs "
          f"(td range [{high_td['td'].min():.0f}, {high_td['td'].max():.0f}])")

    if len(high_td) < N_TARGET:
        print(f"WARNING: only {len(high_td)} designs above {TD_MIN_EXT} rad/s^2 in pool. "
              f"Using all of them.")
        selected = high_td
    else:
        # Stratify into 4 sub-bins so the sample covers the td>300 range evenly
        high_td['subbin'] = pd.qcut(high_td['td'], 4, labels=False, duplicates='drop')
        rng = np.random.RandomState(42)
        chosen = []
        for b in sorted(high_td['subbin'].unique()):
            sub = high_td[high_td['subbin'] == b]
            n_take = min(N_TARGET // 4, len(sub))
            chosen.append(sub.sample(n=n_take, random_state=rng))
        selected = pd.concat(chosen).reset_index(drop=True)

    print(f"Selected: n={len(selected)} designs")
    print(f"  td range: [{selected['td'].min():.0f}, {selected['td'].max():.0f}] rad/s^2")
    print(f"  bins: {selected['td'].describe()[['min','25%','50%','75%','max']].round(0).to_dict()}")

    sims_per_design = N_KP * N_KD * len(SEARCH_SEEDS) + 3 * N_FINAL_SEEDS
    print(f"\nSims/design: {int(sims_per_design)}  Total: ~{int(sims_per_design * len(selected))}")
    print("Running...")

    rows = selected.to_dict('records')
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design)(row) for row in rows
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved: {OUT_CSV}")

    # Quick summary
    from scipy.stats import pearsonr
    log_td = np.log(out['td'])
    r, p = pearsonr(log_td, out['over_sr'])
    print(f"\n=== EXTENSION SUMMARY (td > {TD_MIN_EXT}) ===")
    print(f"n={len(out)}, td=[{out['td'].min():.0f}, {out['td'].max():.0f}]")
    print(f"mean over_sr: {out['over_sr'].mean():.3f}  (full dataset mean was 0.906)")
    print(f"r(log_td, over_sr) = {r:+.3f}  p={p:.4f}")

    # Binned means
    bins = [(300, 500), (500, 800), (800, 9999)]
    for lo, hi in bins:
        sub = out[(out['td'] >= lo) & (out['td'] < hi)]
        if len(sub) > 0:
            print(f"  td [{lo}-{min(hi,9999)}]: n={len(sub)}, mean over_sr={sub['over_sr'].mean():.3f}")

    # Pooled re-fit with original dataset
    combined_path = Path('experiments/results/combined_margin_regression_py.csv')
    if combined_path.exists():
        original = pd.read_csv(combined_path)
        if 'pass_label' not in original.columns:
            original['pass_label'] = 'original'
        pooled = pd.concat([original, out], ignore_index=True)
        pooled_path = Path('experiments/results/regression_pooled_py.csv')
        pooled.to_csv(pooled_path, index=False)
        print(f"\nSaved pooled (original n={len(original)} + extension n={len(out)}): {pooled_path}")

        # Re-fit on pooled data
        log_td_p = np.log(pooled['td'])
        log_lat_p = np.log(pooled['latency_steps'])
        y_p = pooled['over_sr'].values
        X_p = np.column_stack([np.ones(len(pooled)), log_td_p, log_lat_p])
        beta_p, *_ = np.linalg.lstsq(X_p, y_p, rcond=None)
        y_hat_p = X_p @ beta_p
        ss_res = float(np.sum((y_p - y_hat_p)**2))
        ss_tot = float(np.sum((y_p - y_p.mean())**2))
        r2_pooled = 1 - ss_res / ss_tot
        print(f"\nPooled OLS (n={len(pooled)}): R^2={r2_pooled:.3f}")
        print(f"  intercept={beta_p[0]:.4f}, log_td={beta_p[1]:.4f}, log_latency={beta_p[2]:.4f}")

        # 5-fold CV
        from sklearn.model_selection import KFold
        from sklearn.linear_model import LinearRegression
        kf = KFold(n_splits=5, shuffle=True, random_state=0)
        X_feat = np.column_stack([log_td_p, log_lat_p])
        cv_r2s = []
        for train_idx, test_idx in kf.split(X_feat):
            reg = LinearRegression().fit(X_feat[train_idx], y_p[train_idx])
            cv_r2s.append(reg.score(X_feat[test_idx], y_p[test_idx]))
        print(f"  5-fold CV R^2: {np.mean(cv_r2s):.3f} +/- {np.std(cv_r2s):.3f}")


if __name__ == '__main__':
    main()
