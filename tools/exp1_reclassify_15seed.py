"""
tools/exp1_reclassify_15seed.py

Full correction pass following the discovery that 43/45 (95.6%) of Exp1 FRAGILE
labels are "borderline" under the 3-seed binary robustness test: SR can only take
values {0, 1/3, 2/3, 1}, and the EASY_ROBUSTNESS threshold (0.80) sits between
2/3 and 1.0. This means the test cannot reliably distinguish a truly-robust design
(true p_success ~0.85-0.95) from a truly-fragile one (true p_success ~0.5-0.7) --
both commonly land on SR=2/3 by chance.

Scope (not a full re-run of n=2400 -- only the at-risk population):
  - ALL FRAGILE (45), MARGINAL (5), INFEASIBLE (3) designs  [53 total]
  - ALL EASY designs with theta_ddot_max > 70 rad/s^2        [196 total]
    (boundary v2 showed even "clean" 3/3 EASY passes can be lucky at high td)
  - Designs below td=70 with clean EASY labels are NOT re-run: at low authority,
    true success probability is high-margin (boundary v2 showed ratio>=43x window
    at td=36) so 3-seed noise is not a practical risk there.

Method: freeze best_Kp/best_Kd from Exp1 (no re-tuning -- isolates classification
noise from gain-selection variance). Re-evaluate nominal/under/over with 15 FRESH
seeds (101-115, disjoint from original 1-3). Reclassify with classify_regime().

Output: experiments/results/exp1_reclassify_15seed_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from sklearn.metrics import roc_auc_score

from design_space import (
    build_plant, build_actuator, build_sensor, build_disturbance,
    build_scenario, classify_regime,
)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate, UNDER_SCALE, OVER_SCALE
from units import FRAGILE_SUCCESS_RATE, ROBUSTNESS_SUCCESS_RATE

CSV     = Path('experiments/results/exp1_regime_index_py.csv')
OUT_CSV = Path('experiments/results/exp1_reclassify_15seed_py.csv')

N_EXTRA_SEEDS = 15
SEED_START    = 101
TD_EASY_CUTOFF = 70.0

F15_T_AVG = 14.4
CU_TO_RAD = (np.pi / 180) * (15 / 12)
L_NOZZLE  = 0.25


def _td(row):
    keff = F15_T_AVG * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']
    return keff * row['max_gimbal_deg'] * 12.0 / 15.0


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _eval_high_seed(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    best_Kp = float(row['best_Kp'])
    best_Kd = float(row['best_Kd'])
    seeds = list(range(SEED_START, SEED_START + N_EXTRA_SEEDS))

    def eval_cond(Kp, Kd):
        pid  = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
        return _aggregate(runs)

    nominal = eval_cond(best_Kp, best_Kd)
    under   = eval_cond(UNDER_SCALE * best_Kp, UNDER_SCALE * best_Kd)
    over    = eval_cond(OVER_SCALE  * best_Kp, OVER_SCALE  * best_Kd)

    n_pass = (
        int(nominal["success_rate"] >= FRAGILE_SUCCESS_RATE)
        + int(under["success_rate"]  >= ROBUSTNESS_SUCCESS_RATE)
        + int(over["success_rate"]   >= ROBUSTNESS_SUCCESS_RATE)
    )
    robustness_new = n_pass / 3.0

    new_label, new_code = classify_regime(
        nominal["success_rate"], nominal["rms_error_deg"],
        nominal["u_cmd_sat_frac"], nominal["slew_sat_frac"],
        nominal["settling_time_s"], nominal["oscillation_score"],
        robustness_new,
    )

    return dict(
        rocket_id            = row['rocket_id'],
        td                   = row['td'],
        original_label       = row['regime_label'],
        original_nom_sr      = row['nominal_success_rate'],
        original_under_sr    = row['under_success_rate'],
        original_over_sr     = row['over_success_rate'],
        new_label             = new_label,
        new_nom_sr_15seed     = nominal["success_rate"],
        new_under_sr_15seed   = under["success_rate"],
        new_over_sr_15seed    = over["success_rate"],
        new_robustness_15seed = robustness_new,
        label_flipped         = int(new_label != row['regime_label']),
        best_Kp               = best_Kp,
        best_Kd                = best_Kd,
    )


def main():
    df = pd.read_csv(CSV)
    df['td'] = df.apply(_td, axis=1)

    at_risk = df[df['regime_label'].isin(['FRAGILE', 'MARGINAL', 'INFEASIBLE'])]
    easy_high_td = df[(df['regime_label'] == 'EASY') & (df['td'] > TD_EASY_CUTOFF)]

    selected = pd.concat([at_risk, easy_high_td]).drop_duplicates(subset=['rocket_id']).reset_index(drop=True)
    print(f"Re-evaluating {len(selected)} designs with {N_EXTRA_SEEDS} fresh seeds:")
    print(f"  at-risk (FRAGILE+MARGINAL+INFEASIBLE): {len(at_risk)}")
    print(f"  EASY with td > {TD_EASY_CUTOFF}: {len(easy_high_td)}")
    print(f"Total sims: {len(selected)} x 3 conditions x {N_EXTRA_SEEDS} seeds = {len(selected)*3*N_EXTRA_SEEDS}")

    rows = selected.to_dict('records')
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_high_seed)(row) for row in rows
    )
    out_df = pd.DataFrame(results)
    out_df.to_csv(OUT_CSV, index=False)

    print("\n" + "="*70)
    print("RECLASSIFICATION RESULTS (15-seed, frozen gains)")
    print("="*70)
    for orig in ['FRAGILE', 'MARGINAL', 'INFEASIBLE', 'EASY']:
        sub = out_df[out_df['original_label'] == orig]
        if len(sub) == 0:
            continue
        n_flip = sub['label_flipped'].sum()
        print(f"\n  Original={orig}: n={len(sub)}  flipped={n_flip} ({100*n_flip/len(sub):.1f}%)")
        if n_flip > 0:
            print(sub[sub['label_flipped']==1]['new_label'].value_counts().to_string())

    # Build CORRECTED population: apply new labels where re-evaluated, keep original elsewhere
    corrected = df.merge(out_df[['rocket_id', 'new_label']], on='rocket_id', how='left')
    corrected['corrected_label'] = corrected['new_label'].fillna(corrected['regime_label'])

    print("\n" + "="*70)
    print("CORRECTED POPULATION COUNTS (n=2400)")
    print("="*70)
    print("Original:")
    print(df['regime_label'].value_counts().to_string())
    print("\nCorrected:")
    print(corrected['corrected_label'].value_counts().to_string())

    # Recompute AUC with corrected labels
    y_orig = (df['regime_label'] == 'FRAGILE').astype(int).values
    y_corr = (corrected['corrected_label'] == 'FRAGILE').astype(int).values
    td_vals = df['td'].values

    if y_corr.sum() >= 5:
        auc_orig = roc_auc_score(y_orig, td_vals)
        auc_corr = roc_auc_score(y_corr, td_vals)
        print(f"\nAUC(theta_ddot predicting FRAGILE), ORIGINAL labels:  {auc_orig:.3f}")
        print(f"AUC(theta_ddot predicting FRAGILE), CORRECTED labels: {auc_corr:.3f}")

    corrected.to_csv('experiments/results/exp1_corrected_population_py.csv', index=False)
    print(f"\nSaved: {OUT_CSV}")
    print(f"Saved: experiments/results/exp1_corrected_population_py.csv")


if __name__ == '__main__':
    main()
