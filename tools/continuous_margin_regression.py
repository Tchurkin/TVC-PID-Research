"""
tools/continuous_margin_regression.py

Re-frames the central finding as a CONTINUOUS regression problem instead of a
binary FRAGILE/EASY classification, directly addressing two methodological
critiques:

  1. AUC on a ~1.5% base rate is easy to misread as "accuracy" (a trivial
     always-EASY classifier gets ~98.5% accuracy and is useless). Reporting a
     continuous outcome and its R²/correlation sidesteps the base-rate framing
     entirely -- there is no threshold to game.
  2. The EASY/FRAGILE split is, by the project's own boundary experiment and
     the v2 "uncertain" Wilson-CI flag (33.6% of at-risk designs straddle a
     threshold even at 30 seeds), a thresholded view of a continuum, not two
     natural populations. This script tests the continuum claim directly:
     does theta_ddot_max (+ latency) predict a CONTINUOUS robustness margin,
     across a sample stratified to cover the full theta_ddot range (not just
     whatever a uniform LHS happens to draw)?

Method
------
1. Draw a large LHS pool (n=4000) and compute theta_ddot_max (td) for each.
2. Stratify into 10 deciles of td and sample ~13 designs per decile (~130
   total) -- guarantees good coverage of the rare high-authority tail that a
   plain LHS sample would under-represent, while keeping every individual
   design a realistic, jointly-sampled hardware configuration (not a
   cherry-picked single-parameter extreme).
3. For each sampled design: run the same finer joint Kp x Kd search used in
   the final population correction (18 x 7 grid, 3-seed SR/RMS selection),
   then evaluate nominal / under(0.6x) / over(1.4x) success rate with 15
   FRESH seeds each -- a continuous fraction in [0,1], not a binary pass/fail.
4. Regress the continuous over-robustness success rate (the dominant failure
   mode per the final population: ~87%% ceiling-limited) on log(td) and
   log(latency_steps). Report R-squared and correlations -- no threshold,
   no base-rate distortion, no accuracy/AUC ambiguity.

Output: experiments/results/continuous_margin_regression_py.csv
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

KP_MIN, KP_MAX, N_KP = 1.0, 320.0, 18
KD_MIN, KD_MAX, N_KD = 1.0, 64.0, 7
SEARCH_SEEDS  = [1, 2, 3]
N_FINAL_SEEDS = 15
FINAL_SEED_START = 5001

N_POOL       = 4000
N_DECILES    = 10
PER_DECILE   = 13

OUT_CSV = Path('experiments/results/continuous_margin_regression_py.csv')


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
        rocket_id    = row['rocket_id'],
        td           = row['td'],
        latency_steps= row['latency_steps'],
        Iyy          = row['Iyy'],
        motor_scale  = row['motor_scale'],
        max_gimbal_deg = row['max_gimbal_deg'],
        new_Kp       = new_Kp,
        new_Kd       = new_Kd,
        nom_sr       = nominal['success_rate'],
        under_sr     = under['success_rate'],
        over_sr      = over['success_rate'],
        margin       = min(under['success_rate'], over['success_rate']),
        nom_rms      = nominal['rms_error_deg'],
    )


def main():
    pool = sample_lhs(N_POOL, seed=777)
    pool['td'] = pool.apply(lambda r: compute_td(r.to_dict()), axis=1)

    # Stratify by decile of td to guarantee coverage of the rare high-authority tail.
    pool['decile'] = pd.qcut(pool['td'], N_DECILES, labels=False, duplicates='drop')
    rng = np.random.RandomState(123)
    chosen = []
    for d in sorted(pool['decile'].unique()):
        sub = pool[pool['decile'] == d]
        n_take = min(PER_DECILE, len(sub))
        chosen.append(sub.sample(n=n_take, random_state=rng))
    selected = pd.concat(chosen).reset_index(drop=True)

    sims_per_design = N_KP * N_KD * len(SEARCH_SEEDS) + 3 * N_FINAL_SEEDS
    print(f"Continuous margin regression: {len(selected)} designs stratified across "
          f"{N_DECILES} theta_ddot deciles (td range [{selected['td'].min():.1f}, "
          f"{selected['td'].max():.1f}] rad/s^2)")
    print(f"  Sims/design: {int(sims_per_design)}  Total: ~{int(sims_per_design*len(selected))}")

    rows = selected.to_dict('records')
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design)(row) for row in rows
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)

    from scipy.stats import pearsonr, spearmanr
    log_td  = np.log(out['td'])
    log_lat = np.log(out['latency_steps'])

    r_over_td,  _ = pearsonr(log_td, out['over_sr'])
    rho_over_td, _ = spearmanr(out['td'], out['over_sr'])
    r_margin_td, _ = pearsonr(log_td, out['margin'])

    # Two-variable OLS: over_sr ~ log(td) + log(latency)
    X = np.column_stack([np.ones(len(out)), log_td, log_lat])
    y = out['over_sr'].values
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    y_hat = X @ beta
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2_combined = 1 - ss_res / ss_tot

    X1 = np.column_stack([np.ones(len(out)), log_td])
    beta1, *_ = np.linalg.lstsq(X1, y, rcond=None)
    y_hat1 = X1 @ beta1
    r2_td_only = 1 - np.sum((y - y_hat1) ** 2) / ss_tot

    print("\n=== CONTINUOUS REGRESSION RESULTS ===")
    print(f"r(log td, over_sr):        {r_over_td:+.3f}")
    print(f"rho(td, over_sr):          {rho_over_td:+.3f}")
    print(f"r(log td, margin):         {r_margin_td:+.3f}")
    print(f"R^2 (over_sr ~ log td):              {r2_td_only:.3f}")
    print(f"R^2 (over_sr ~ log td + log latency): {r2_combined:.3f}")
    print(f"Coefficients (combined): intercept={beta[0]:.3f} log_td={beta[1]:.3f} log_lat={beta[2]:.3f}")

    print(f"\nSaved: {OUT_CSV}")


if __name__ == '__main__':
    main()
