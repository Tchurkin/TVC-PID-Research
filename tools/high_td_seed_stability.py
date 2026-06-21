"""
tools/high_td_seed_stability.py

Tests whether the EASY/FRAGILE convergence observed above theta_ddot_max=100 rad/s^2
(boundary experiment v2) is real physics or an artifact of the 3-seed robustness
test used in Exp1 classification.

Protocol:
  - Select ALL exp1 FRAGILE designs with td > 100 rad/s^2, plus a matched-size
    random sample of EASY designs with td > 100.
  - Re-evaluate nominal / under(0.6x) / over(1.4x) gain conditions with 15 FRESH
    seeds each (101-115; disjoint from the original seeds 1-3 used in Exp1, so
    this is an independent re-measurement, not a re-analysis of the same draws).
  - Gains are NOT re-tuned (best_Kp/best_Kd frozen from Exp1) — this isolates
    classification noise from the robustness test, not gain-selection variance.
  - Reclassify with the same classify_regime() rule and compare to the original
    3-seed label.

If most designs keep their label: the high-td convergence is real (windows
genuinely narrow for both EASY and FRAGILE at high td) — 3-seed test was
already adequate. If many flip: the original 3-seed classification at high td
is unreliable noise, and the "convergence" finding is partly an artifact.
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import (
    build_plant, build_actuator, build_sensor, build_disturbance,
    build_scenario, classify_regime,
)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate, UNDER_SCALE, OVER_SCALE
from units import FRAGILE_SUCCESS_RATE, ROBUSTNESS_SUCCESS_RATE

CSV     = Path('experiments/results/exp1_regime_index_py.csv')
OUT_CSV = Path('experiments/results/high_td_seed_stability_py.csv')

N_EXTRA_SEEDS = 15
SEED_START    = 101  # disjoint from Exp1 seeds (1,2,3)
TD_THRESHOLD  = 100.0

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
        rocket_id           = row['rocket_id'],
        td                  = row['td'],
        original_label      = row['regime_label'],
        original_nom_sr     = row['nominal_success_rate'],
        original_under_sr   = row['under_success_rate'],
        original_over_sr    = row['over_success_rate'],
        new_label            = new_label,
        new_nom_sr_15seed    = nominal["success_rate"],
        new_under_sr_15seed  = under["success_rate"],
        new_over_sr_15seed   = over["success_rate"],
        new_robustness_15seed= robustness_new,
        label_flipped        = int(new_label != row['regime_label']),
        best_Kp              = best_Kp,
        best_Kd              = best_Kd,
    )


def main():
    df = pd.read_csv(CSV)
    df['td'] = df.apply(_td, axis=1)

    high_td = df[(df['td'] > TD_THRESHOLD) & (df['regime_label'].isin(['EASY', 'FRAGILE']))]
    frag = high_td[high_td['regime_label'] == 'FRAGILE']
    easy_pool = high_td[high_td['regime_label'] == 'EASY']

    n_match = min(len(frag), len(easy_pool))
    easy = easy_pool.sample(n=min(len(easy_pool), max(n_match, 30)), random_state=11)

    selected = pd.concat([frag, easy]).reset_index(drop=True)
    print(f"Selected {len(frag)} FRAGILE + {len(easy)} EASY designs with td > {TD_THRESHOLD} rad/s^2")
    print(f"Re-evaluating each with {N_EXTRA_SEEDS} fresh seeds (gains frozen, not re-tuned)...")
    print(f"Total sims: {len(selected)} designs x 3 conditions x {N_EXTRA_SEEDS} seeds = "
          f"{len(selected)*3*N_EXTRA_SEEDS}")

    rows = selected.to_dict('records')
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_high_seed)(row) for row in rows
    )
    out_df = pd.DataFrame(results)
    out_df.to_csv(OUT_CSV, index=False)

    print("\n" + "="*70)
    print("LABEL STABILITY RESULTS (15-seed re-evaluation, frozen gains)")
    print("="*70)
    for orig in ['FRAGILE', 'EASY']:
        sub = out_df[out_df['original_label'] == orig]
        if len(sub) == 0:
            continue
        n_flip = sub['label_flipped'].sum()
        print(f"\n  Original={orig}: n={len(sub)}")
        print(f"    Flipped: {n_flip}/{len(sub)} ({100*n_flip/len(sub):.1f}%)")
        if n_flip > 0:
            print(f"    New label distribution among flips:")
            print(sub[sub['label_flipped']==1]['new_label'].value_counts().to_string())
        print(f"    Mean td: {sub['td'].mean():.1f}")
        print(f"    Original under_sr: mean={sub['original_under_sr'].mean():.3f}  "
              f"15-seed under_sr: mean={sub['new_under_sr_15seed'].mean():.3f}")
        print(f"    Original over_sr:  mean={sub['original_over_sr'].mean():.3f}  "
              f"15-seed over_sr:  mean={sub['new_over_sr_15seed'].mean():.3f}")

    overall_flip_rate = out_df['label_flipped'].mean()
    print(f"\n  OVERALL flip rate: {100*overall_flip_rate:.1f}%")
    print(f"\nInterpretation:")
    if overall_flip_rate < 0.10:
        print("  Low flip rate -> 3-seed Exp1 classification was already stable at high td.")
        print("  The EASY/FRAGILE convergence at td>100 is REAL, not a 3-seed artifact.")
    else:
        print("  High flip rate -> 3-seed Exp1 classification is NOISY at high td.")
        print("  Some fraction of the apparent EASY/FRAGILE convergence is classification noise.")

    print(f"\nSaved: {OUT_CSV}")


if __name__ == '__main__':
    main()
