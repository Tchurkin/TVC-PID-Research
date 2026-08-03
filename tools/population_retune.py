"""
tools/population_retune.py  —  re-tune the population out-of-sample and re-test the design screen

HYPOTHESIS
----------
Every gain in `exp1_final_population_py.csv` came from `autotune_grid`, whose docstring reads
"Grid search for (Kp, Kd) that maximises SINGLE-SEED success": it evaluates at seed=1 and keeps
the FIRST grid point that survives that one draw. `_eval_design_exp1` then scores
`nominal_success_rate` on the same seed=1. The gains are therefore fitted to one random draw and
the recorded performance is in-sample. Evidence:

  recorded nominal_success_rate      mean 0.999,  99.7% >= 0.80
  same gains, 15 fresh seeds         mean 0.283,  12.3% >= 0.80
  distinct best_Kd across 2400 designs: 3        (the "first grid point that passed" signature)

The label-correction passes (15-seed reclassify, final correction) re-evaluated LABELS but left
the gains untouched -- best_Kp is 100% identical between them and the final population. So
`final_label` encodes "designs the single-seed search failed on", which is not the same thing as
"designs that are hard to control". Claims 2 (the four-number screen) and 4 (the flight signature)
both use that label as ground truth.

H0: re-tuning properly leaves the labels and the screen's accuracy essentially unchanged
    -> the ground truth was fine and Claims 2 and 4 stand.
H1: labels move substantially and/or the screen's CV AUC collapses on the corrected labels
    -> the screen was partly predicting search luck, and the claim must be restated.

METHOD
------
Per design:
  1. Joint Kp x Kd grid search (18 x 7), objective = mean success rate over SEARCH_SEEDS,
     RMS as tiebreak. Joint, not the decoupled probe-then-freeze used originally.
  2. Evaluate the chosen gains on EVAL_SEEDS -- DISJOINT from the search seeds, so the
     recorded performance is out-of-sample.
  3. Evaluate at 1.4x the chosen gains on the same eval seeds. This is the over-robustness
     probe the paper uses as its continuous margin; label_new = (over_sr < 0.80).
  4. Carry the original gains and label for a paired comparison.

Then re-fit the four-number screen -- log(1/Iyy), log(T*L), log(tau) -- against BOTH the old and
the new labels, 5-fold stratified CV AUC, and report the difference. That number is the answer.

SEEDS: search 330001-330003, eval 330101-330115. Disjoint from all prior experiments.
OUTPUT: experiments/results/population_retune_py.csv

USAGE
-----
  python tools/population_retune.py --n 40          # smoke test
  python tools/population_retune.py                 # full population (~900k sims)
"""

from __future__ import annotations

import argparse
import os
import sys
import warnings
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
warnings.filterwarnings("ignore")

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams
from experiment_runner import _run_one

KP_GRID      = np.geomspace(1.0, 400.0, 18)
KD_GRID      = np.geomspace(0.05, 32.0, 7)
SEARCH_SEEDS = (330001, 330002, 330003)
EVAL_SEEDS   = tuple(range(330101, 330116))     # 15, disjoint from the search
OVER_SCALE   = 1.4
SR_PASS      = 0.80

OUT_CSV     = Path("experiments/results/population_retune_py.csv")
DESIGNS_CSV = Path("experiments/results/exp1_final_population_py.csv")

RESTORE_HINT = f"""
{DESIGNS_CSV} is not in the working tree (deleted by commit bb22d36). Restore with:
    git archive bb22d36^ experiments/results | tar -x -C .
or pass --designs <path>.
"""


def _score(pid, plant, act, sen, dis, sc, seeds) -> tuple[float, float]:
    runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    return (float(np.mean([r["success"] for r in runs])),
            float(np.mean([r["rms_error_deg"] for r in runs])))


def retune(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    # 1. joint search, multi-seed, SR primary with RMS tiebreak
    best = (-1.0, float("inf"))
    best_kp, best_kd = float(KP_GRID[0]), float(KD_GRID[0])
    for kp in KP_GRID:
        for kd in KD_GRID:
            pid = PIDParams(Kp=float(kp), Kd=float(kd), Ki=0.0,
                            u_max=act.u_max, i_lim=act.u_max)
            s = _score(pid, plant, act, sen, dis, sc, SEARCH_SEEDS)
            if s[0] > best[0] or (s[0] == best[0] and s[1] < best[1]):
                best, best_kp, best_kd = s, float(kp), float(kd)

    # 2. out-of-sample evaluation at the chosen gains
    pid_new = PIDParams(Kp=best_kp, Kd=best_kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    sr_new, rms_new = _score(pid_new, plant, act, sen, dis, sc, EVAL_SEEDS)

    # 3. over-robustness probe -> the corrected label
    pid_over = PIDParams(Kp=OVER_SCALE * best_kp, Kd=OVER_SCALE * best_kd, Ki=0.0,
                         u_max=act.u_max, i_lim=act.u_max)
    over_sr, _ = _score(pid_over, plant, act, sen, dis, sc, EVAL_SEEDS)

    # 4. the original single-seed gains, scored on the same fresh seeds
    kp_old = float(row.get("best_Kp", best_kp))
    kd_old = float(row.get("best_Kd", best_kd))
    pid_old = PIDParams(Kp=kp_old, Kd=kd_old, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    sr_old_oos, _ = _score(pid_old, plant, act, sen, dis, sc, EVAL_SEEDS)

    return dict(
        rocket_id=row["rocket_id"], Iyy=row["Iyy"], motor_scale=row["motor_scale"],
        latency_steps=float(row["latency_steps"]), wind_strength=row.get("wind_strength"),
        kp_new=best_kp, kd_new=best_kd, kp_old=kp_old, kd_old=kd_old,
        search_sr=best[0], sr_new_oos=sr_new, rms_new_oos=rms_new,
        sr_old_oos=sr_old_oos,
        sr_old_recorded=row.get("nominal_success_rate"),
        over_sr=over_sr,
        label_new=int(over_sr < SR_PASS),
        label_old=int(str(row.get("final_label", "")) == "FRAGILE"),
    )


def screen_auc(d: pd.DataFrame, label: str) -> tuple[float, float]:
    from sklearn.linear_model import LogisticRegression
    from sklearn.model_selection import StratifiedKFold
    from sklearn.metrics import roc_auc_score
    y = d[label].values
    if y.sum() < 10 or (1 - y).sum() < 10:
        return float("nan"), float("nan")
    X = np.column_stack([np.log(1.0 / d.Iyy), np.log(14.4 * d.motor_scale * 0.25),
                         np.log(d.latency_steps / 200.0)])
    aucs = []
    for seed in range(5):
        sk = StratifiedKFold(5, shuffle=True, random_state=seed)
        for tr, te in sk.split(X, y):
            m = LogisticRegression(max_iter=20000, C=1e6).fit(X[tr], y[tr])
            aucs.append(roc_auc_score(y[te], m.predict_proba(X[te])[:, 1]))
    return float(np.mean(aucs)), float(np.std(aucs))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=0, help="0 = full population")
    ap.add_argument("--designs", type=Path, default=DESIGNS_CSV)
    ap.add_argument("--jobs", type=int, default=-1)
    args = ap.parse_args()

    if not args.designs.exists():
        sys.exit(RESTORE_HINT)
    pop = pd.read_csv(args.designs)
    if args.n:
        pop = pop.sample(min(args.n, len(pop)), random_state=77).reset_index(drop=True)

    per = len(KP_GRID) * len(KD_GRID) * len(SEARCH_SEEDS) + 3 * len(EVAL_SEEDS)
    print("=== population re-tune: joint search, out-of-sample evaluation ===")
    print(f"  designs={len(pop)}  grid={len(KP_GRID)}x{len(KD_GRID)}  "
          f"search seeds={len(SEARCH_SEEDS)}  eval seeds={len(EVAL_SEEDS)}")
    print(f"  ~{len(pop)*per:,} sims")

    out = pd.DataFrame(Parallel(n_jobs=args.jobs, verbose=5)(
        delayed(retune)(r) for r in pop.to_dict("records")))
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(out)} rows -> {OUT_CSV}")

    print("\n=== 1. were the original gains in-sample? ===")
    if out.sr_old_recorded.notna().any():
        print(f"  recorded (Exp1, seed=1, in-sample) : mean {out.sr_old_recorded.mean():.3f}")
    print(f"  same gains, fresh seeds            : mean {out.sr_old_oos.mean():.3f}")
    print(f"  re-tuned gains, fresh seeds        : mean {out.sr_new_oos.mean():.3f}")
    print(f"  designs where re-tuning helped     : {100*(out.sr_new_oos > out.sr_old_oos).mean():.0f}%")

    print("\n=== 2. how much did the labels move? ===")
    ct = pd.crosstab(out.label_old, out.label_new)
    print(ct.to_string())
    print(f"  labels changed: {100*(out.label_old != out.label_new).mean():.1f}%")
    print(f"  old positives: {out.label_old.sum()}   new positives: {out.label_new.sum()}")

    print("\n=== 3. THE ANSWER: does the four-number screen survive the corrected labels? ===")
    for lab, tag in [("label_old", "old label (single-seed gains)"),
                     ("label_new", "new label (out-of-sample)   ")]:
        m, s = screen_auc(out, lab)
        print(f"  {tag}  CV AUC = {m:.4f} +/- {s:.4f}")
    print("\n  If the new-label AUC holds up, Claim 2's ground truth was sound despite the gains.")
    print("  If it collapses, the screen was partly predicting which designs the single-seed")
    print("  search got unlucky on, and the claim must be restated.")


if __name__ == "__main__":
    main()
