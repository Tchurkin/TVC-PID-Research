"""
tools/gain_search_optimality_audit.py

Audits whether autotune_continuous (sim/experiment_runner.py) actually finds
near-optimal PID gains, or whether its search procedure has structural blind
spots that could be silently mislabeling designs.

Two concrete concerns motivated this audit:

  1. Kd/Kp are DECOUPLED. autotune_continuous probes Kd in {1,4,16,64} at a
     fixed reference Kp=40, freezes the winner, then searches Kp alone. If the
     true optimal Kd depends on where Kp ends up (plausible for PD control),
     this never finds the joint optimum -- especially for designs whose
     best_Kp lands far from 40.

  2. The Kp SEARCH GRID IS COARSE relative to documented window widths.
     10 coarse log points over [1,320] give a ~2.16x step between points, but
     boundary_experiment_v2 found gain windows as narrow as 1.5-3.3x for some
     high-latency / high-theta_ddot designs. A 2.16x step can straddle or miss
     a 1.5x window entirely.

  3. (Secondary) SR during search is averaged over only 2 seeds -- the same
     class of statistical problem as the 3-seed robustness test that was
     already found to be underpowered (see exp1_reclassify_15seed.py).

METHOD: take the corrected FRAGILE population (n=30) + all MARGINAL (n=2) +
all INFEASIBLE (n=3) + a stratified EASY sample (n=30: top-20 by theta_ddot,
+10 random) -- i.e. the population most likely to be affected by a coarse or
decoupled search. For each design:

  (a) Re-run gain selection with a FINER JOINT Kp x Kd grid (18 x 7 = 126
      combos, 3-seed SR/RMS selection -- same selection rule as the original,
      just joint instead of decoupled and ~5x denser in Kp).
  (b) Evaluate BOTH the ORIGINAL autotune gains and the NEW finer-search gains
      on nominal/under/over robustness with the SAME 7 FRESH seeds (disjoint
      from seeds used during search and from all prior audits) -- this is an
      apples-to-apples comparison: any difference is due to gain choice, not
      seed luck.
  (c) Reclassify both with classify_regime() and report how often the label
      changes -- split into "changed just from fresh-seed re-evaluation of
      the SAME gains" (controls for seed noise) vs. "changed further when
      using the finer-search gains" (isolates the search-optimality effect).

Output: experiments/results/gain_search_optimality_audit_py.csv
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

CSV     = Path('experiments/results/exp1_corrected_population_py.csv')
OUT_CSV = Path('experiments/results/gain_search_optimality_audit_py.csv')

# Joint search grid -- denser than autotune_continuous's decoupled 10+6 Kp
# points and 4 Kd points, and JOINT (every Kp tried with every Kd).
KP_MIN, KP_MAX, N_KP = 1.0, 320.0, 18
KD_MIN, KD_MAX, N_KD = 1.0, 64.0, 7
SEARCH_SEEDS = [1, 2, 3]          # 3-seed SR during search (vs original's 2)
COMPARE_SEEDS = list(range(301, 308))  # 7 fresh seeds, disjoint from everything prior
N_EASY_TOP_TD = 20
N_EASY_RANDOM = 10
RNG_SEED = 42


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _joint_search(plant, act, sen, dis, sc) -> tuple[float, float, float, float]:
    """Finer joint Kp x Kd grid search. Returns (best_Kp, best_Kd, best_sr, best_rms)."""
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
    return best_kp, best_kd, best_sr, best_rms


def _classify_with_gains(plant, act, sen, dis, sc, Kp, Kd, seeds):
    def eval_cond(kp, kd):
        pid = PIDParams(Kp=kp, Kd=kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
        return _aggregate(runs)

    nominal = eval_cond(Kp, Kd)
    under   = eval_cond(UNDER_SCALE * Kp, UNDER_SCALE * Kd)
    over    = eval_cond(OVER_SCALE  * Kp, OVER_SCALE  * Kd)

    n_pass = (
        int(nominal["success_rate"] >= 0.35)
        + int(under["success_rate"]  >= 0.80)
        + int(over["success_rate"]   >= 0.80)
    )
    robustness = n_pass / 3.0
    label, code = classify_regime(
        nominal["success_rate"], nominal["rms_error_deg"],
        nominal["u_cmd_sat_frac"], nominal["slew_sat_frac"],
        nominal["settling_time_s"], nominal["oscillation_score"],
        robustness,
    )
    return label, nominal["success_rate"], under["success_rate"], over["success_rate"], robustness


def _eval_design(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    orig_Kp = float(row['best_Kp'])
    orig_Kd = float(row['best_Kd'])

    # (a) finer joint search
    new_Kp, new_Kd, search_sr, search_rms = _joint_search(plant, act, sen, dis, sc)

    # (b) fresh-seed, apples-to-apples comparison of ORIGINAL gains vs NEW gains
    label_orig_gains, nom_o, under_o, over_o, rob_o = _classify_with_gains(
        plant, act, sen, dis, sc, orig_Kp, orig_Kd, COMPARE_SEEDS)
    label_new_gains, nom_n, under_n, over_n, rob_n = _classify_with_gains(
        plant, act, sen, dis, sc, new_Kp, new_Kd, COMPARE_SEEDS)

    return dict(
        rocket_id          = row['rocket_id'],
        td                 = row.get('td', np.nan),
        original_label     = row['corrected_label'] if 'corrected_label' in row else row['regime_label'],
        orig_Kp             = orig_Kp,
        orig_Kd             = orig_Kd,
        new_Kp              = new_Kp,
        new_Kd              = new_Kd,
        kp_ratio_new_over_orig = new_Kp / orig_Kp if orig_Kp > 0 else np.nan,
        label_orig_gains_freshseed = label_orig_gains,
        nom_sr_orig_gains   = nom_o,
        under_sr_orig_gains = under_o,
        over_sr_orig_gains  = over_o,
        label_new_gains_freshseed = label_new_gains,
        nom_sr_new_gains    = nom_n,
        under_sr_new_gains  = under_n,
        over_sr_new_gains   = over_n,
        flip_from_seed_noise   = int(label_orig_gains != (row['corrected_label'] if 'corrected_label' in row else row['regime_label'])),
        flip_from_search        = int(label_new_gains != label_orig_gains),
        flip_from_original      = int(label_new_gains != (row['corrected_label'] if 'corrected_label' in row else row['regime_label'])),
    )


def main():
    df = pd.read_csv(CSV)
    label_col = 'corrected_label' if 'corrected_label' in df.columns else 'regime_label'

    fragile  = df[df[label_col] == 'FRAGILE']
    marginal = df[df[label_col] == 'MARGINAL']
    infeas   = df[df[label_col] == 'INFEASIBLE']
    easy     = df[df[label_col] == 'EASY'].copy()

    if 'td' not in easy.columns or easy['td'].isna().all():
        F15_T_AVG = 14.4
        CU_TO_RAD = (np.pi / 180) * (15 / 12)
        L_NOZZLE  = 0.25
        def _td(r):
            keff = F15_T_AVG * r['motor_scale'] * CU_TO_RAD * L_NOZZLE / r['Iyy']
            return keff * r['max_gimbal_deg'] * 12.0 / 15.0
        easy['td'] = easy.apply(_td, axis=1)

    easy_top = easy.nlargest(N_EASY_TOP_TD, 'td')
    rng = np.random.RandomState(RNG_SEED)
    remaining = easy.drop(easy_top.index)
    easy_rand = remaining.sample(n=min(N_EASY_RANDOM, len(remaining)), random_state=rng)

    selected = pd.concat([fragile, marginal, infeas, easy_top, easy_rand]).drop_duplicates(subset=['rocket_id']).reset_index(drop=True)

    n_combos = N_KP * N_KD
    print(f"Auditing gain-search optimality on {len(selected)} designs:")
    print(f"  FRAGILE={len(fragile)} MARGINAL={len(marginal)} INFEASIBLE={len(infeas)} "
          f"EASY(top-td)={len(easy_top)} EASY(random)={len(easy_rand)}")
    print(f"  Joint grid: {N_KP} Kp x {N_KD} Kd = {n_combos} combos x {len(SEARCH_SEEDS)} seeds "
          f"= {n_combos*len(SEARCH_SEEDS)} sims/design (search phase)")
    print(f"  Compare phase: 2 gain-sets x 3 conditions x {len(COMPARE_SEEDS)} seeds = "
          f"{2*3*len(COMPARE_SEEDS)} sims/design")
    total_sims = len(selected) * (n_combos*len(SEARCH_SEEDS) + 2*3*len(COMPARE_SEEDS))
    print(f"  Total: ~{total_sims} sims")

    rows = selected.to_dict('records')
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design)(row) for row in rows
    )
    out_df = pd.DataFrame(results)
    out_df.to_csv(OUT_CSV, index=False)

    print("\n" + "="*70)
    print("GAIN SEARCH OPTIMALITY AUDIT RESULTS")
    print("="*70)

    for orig in ['FRAGILE', 'MARGINAL', 'INFEASIBLE', 'EASY']:
        sub = out_df[out_df['original_label'] == orig]
        if len(sub) == 0:
            continue
        n_seed_flip   = sub['flip_from_seed_noise'].sum()
        n_search_flip = sub['flip_from_search'].sum()
        n_total_flip  = sub['flip_from_original'].sum()
        print(f"\n  Original={orig}: n={len(sub)}")
        print(f"    flipped by fresh-seed re-eval alone (noise control): {n_seed_flip} ({100*n_seed_flip/len(sub):.1f}%)")
        print(f"    flipped further by finer/joint search (vs orig-gain fresh-seed label): {n_search_flip} ({100*n_search_flip/len(sub):.1f}%)")
        print(f"    flipped overall vs stored label: {n_total_flip} ({100*n_total_flip/len(sub):.1f}%)")
        kp_ratio = sub['kp_ratio_new_over_orig']
        print(f"    new_Kp/orig_Kp: median={kp_ratio.median():.2f}, "
              f"range=[{kp_ratio.min():.2f}, {kp_ratio.max():.2f}], "
              f"frac with |log ratio|>0.3 (>=2x or <=0.5x): "
              f"{100*((kp_ratio>2)|(kp_ratio<0.5)).mean():.1f}%")

    print(f"\nSaved: {OUT_CSV}")


if __name__ == '__main__':
    main()
