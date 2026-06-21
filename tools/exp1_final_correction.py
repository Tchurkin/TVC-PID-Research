"""
tools/exp1_final_correction.py

Second-order correction, following gain_search_optimality_audit.py's discovery
that the 15-seed correction (exp1_reclassify_15seed.py) was ITSELF not enough:
re-evaluating the corrected at-risk population's frozen gains with a fresh
batch of seeds flipped 43-100% of labels depending on regime, and a finer
joint Kp x Kd search flipped many further. One INFEASIBLE design (R0475)
showed true success rate ~0.40 over 40 seeds (a stable estimate well above
the 0.35 INFEASIBLE cutoff) despite both the original 3-seed AND the 15-seed
correction landing on INFEASIBLE -- pure sampling variance at small n.

This script does the full fix in one pass for every at-risk design (the
241 designs that are FRAGILE/MARGINAL/INFEASIBLE in the v1-corrected
population, or EASY with theta_ddot_max > 70 rad/s^2):

  1. Re-run gain selection with the FINER JOINT Kp x Kd grid (18 x 7 = 126
     combos, 3-seed SR/RMS selection) instead of trusting the frozen
     autotune_continuous gains, which were shown to sometimes miss the true
     optimum by up to 8.7x in Kp.
  2. Evaluate nominal/under/over robustness with N_FINAL_SEEDS=30 FRESH seeds
     (1001+, disjoint from every seed range used anywhere else in this
     project) using the new gains.
  3. Compute a Wilson 95% CI on each success rate and flag a design
     "uncertain" if its CI straddles the decision threshold relevant to its
     classification (0.35 for the nominal/INFEASIBLE gate, 0.80 for under/over
     robustness) -- i.e. report honestly when 30 seeds still can't resolve it,
     rather than forcing a hard label.
  4. Reclassify with classify_regime() on the point estimate, and recompute
     AUC(theta_ddot_max -> FRAGILE) on the resulting population.

Output: experiments/results/exp1_final_correction_py.csv
        experiments/results/exp1_final_population_py.csv (full n=2400, final labels)
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

CSV     = Path('experiments/results/exp1_corrected_population_py.csv')
OUT_CSV = Path('experiments/results/exp1_final_correction_py.csv')
POP_CSV = Path('experiments/results/exp1_final_population_py.csv')

KP_MIN, KP_MAX, N_KP = 1.0, 320.0, 18
KD_MIN, KD_MAX, N_KD = 1.0, 64.0, 7
SEARCH_SEEDS = [1, 2, 3]
N_FINAL_SEEDS = 30
FINAL_SEED_START = 1001
TD_EASY_CUTOFF = 70.0


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def wilson_ci(successes: int, n: int, z: float = 1.96):
    if n == 0:
        return (0.0, 1.0)
    phat = successes / n
    denom = 1 + z**2 / n
    center = phat + z**2 / (2 * n)
    margin = z * np.sqrt(phat * (1 - phat) / n + z**2 / (4 * n**2))
    return ((center - margin) / denom, (center + margin) / denom)


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
        agg = _aggregate(runs)
        n_succ = int(round(agg["success_rate"] * N_FINAL_SEEDS))
        lo, hi = wilson_ci(n_succ, N_FINAL_SEEDS)
        return agg, lo, hi

    nominal, nom_lo, nom_hi = eval_cond(new_Kp, new_Kd)
    under,   und_lo, und_hi = eval_cond(UNDER_SCALE * new_Kp, UNDER_SCALE * new_Kd)
    over,    ovr_lo, ovr_hi = eval_cond(OVER_SCALE  * new_Kp, OVER_SCALE  * new_Kd)

    n_pass = (
        int(nominal["success_rate"] >= 0.35)
        + int(under["success_rate"]  >= 0.80)
        + int(over["success_rate"]   >= 0.80)
    )
    robustness = n_pass / 3.0

    new_label, new_code = classify_regime(
        nominal["success_rate"], nominal["rms_error_deg"],
        nominal["u_cmd_sat_frac"], nominal["slew_sat_frac"],
        nominal["settling_time_s"], nominal["oscillation_score"],
        robustness,
    )

    nom_uncertain = nom_lo <= 0.35 <= nom_hi
    und_uncertain = und_lo <= 0.80 <= und_hi
    ovr_uncertain = ovr_lo <= 0.80 <= ovr_hi
    uncertain = int(nom_uncertain or und_uncertain or ovr_uncertain)

    return dict(
        rocket_id           = row['rocket_id'],
        td                  = row.get('td', np.nan),
        original_label      = row['corrected_label'],
        new_Kp              = new_Kp,
        new_Kd              = new_Kd,
        orig_Kp             = row['best_Kp'],
        orig_Kd             = row['best_Kd'],
        nom_sr              = nominal["success_rate"],
        nom_ci_lo           = nom_lo,
        nom_ci_hi           = nom_hi,
        under_sr            = under["success_rate"],
        under_ci_lo         = und_lo,
        under_ci_hi         = und_hi,
        over_sr             = over["success_rate"],
        over_ci_lo          = ovr_lo,
        over_ci_hi          = ovr_hi,
        robustness          = robustness,
        final_label         = new_label,
        label_flipped       = int(new_label != row['corrected_label']),
        uncertain           = uncertain,
    )


def main():
    df = pd.read_csv(CSV)

    at_risk = df[df['corrected_label'].isin(['FRAGILE', 'MARGINAL', 'INFEASIBLE'])]
    easy_high_td = df[(df['corrected_label'] == 'EASY') & (df['td'] > TD_EASY_CUTOFF)]
    selected = pd.concat([at_risk, easy_high_td]).drop_duplicates(subset=['rocket_id']).reset_index(drop=True)

    n_combos = N_KP * N_KD
    sims_per_design = n_combos * len(SEARCH_SEEDS) + 3 * N_FINAL_SEEDS
    print(f"Final correction pass on {len(selected)} designs "
          f"(at_risk={len(at_risk)}, easy_high_td={len(easy_high_td)})")
    print(f"  Search: {n_combos} combos x {len(SEARCH_SEEDS)} seeds = {n_combos*len(SEARCH_SEEDS)} sims/design")
    print(f"  Final classification: 3 conditions x {N_FINAL_SEEDS} seeds = {3*N_FINAL_SEEDS} sims/design")
    print(f"  Total: ~{len(selected) * sims_per_design} sims")

    rows = selected.to_dict('records')
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design)(row) for row in rows
    )
    out_df = pd.DataFrame(results)
    out_df.to_csv(OUT_CSV, index=False)

    print("\n" + "="*70)
    print("FINAL CORRECTION RESULTS (finer joint search + 30 fresh seeds + Wilson CI)")
    print("="*70)
    for orig in ['FRAGILE', 'MARGINAL', 'INFEASIBLE', 'EASY']:
        sub = out_df[out_df['original_label'] == orig]
        if len(sub) == 0:
            continue
        n_flip = sub['label_flipped'].sum()
        n_unc  = sub['uncertain'].sum()
        print(f"\n  Original={orig}: n={len(sub)}  flipped={n_flip} ({100*n_flip/len(sub):.1f}%)  "
              f"uncertain={n_unc} ({100*n_unc/len(sub):.1f}%)")
        if n_flip > 0:
            print(sub[sub['label_flipped']==1]['final_label'].value_counts().to_string())

    # Build final population: apply new labels where re-evaluated, keep v1-corrected elsewhere
    final_pop = df.merge(out_df[['rocket_id', 'final_label', 'uncertain']], on='rocket_id', how='left')
    final_pop['final_label'] = final_pop['final_label'].fillna(final_pop['corrected_label'])
    final_pop['uncertain'] = final_pop['uncertain'].fillna(0)

    print("\n" + "="*70)
    print("POPULATION COUNTS (n=2400)")
    print("="*70)
    print("v1-corrected (15-seed, frozen gains):")
    print(df['corrected_label'].value_counts().to_string())
    print("\nv2-final (finer search + 30 seeds):")
    print(final_pop['final_label'].value_counts().to_string())

    y_v1 = (df['corrected_label'] == 'FRAGILE').astype(int).values
    y_v2 = (final_pop['final_label'] == 'FRAGILE').astype(int).values
    td_vals = df['td'].values

    if y_v2.sum() >= 5:
        auc_v1 = roc_auc_score(y_v1, td_vals)
        auc_v2 = roc_auc_score(y_v2, td_vals)
        print(f"\nAUC(theta_ddot predicting FRAGILE), v1-corrected: {auc_v1:.3f}")
        print(f"AUC(theta_ddot predicting FRAGILE), v2-final:      {auc_v2:.3f}")

    n_uncertain_total = int(final_pop['uncertain'].sum())
    print(f"\nDesigns still 'uncertain' after 30 seeds (CI straddles its decision threshold): {n_uncertain_total}")

    final_pop.to_csv(POP_CSV, index=False)
    print(f"\nSaved: {OUT_CSV}")
    print(f"Saved: {POP_CSV}")


if __name__ == '__main__':
    main()
