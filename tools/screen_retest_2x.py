"""
tools/screen_retest_2x.py  —  re-test the four-number design screen against a probe that actually fires

WHY
---
`tools/population_retune.py` was meant to give Claim 2 an independent ground truth: re-tune each
design with a joint multi-seed search, then label it by whether it survives a gain increase. On the
restored simulator that came back unusable — the 1.4x over-robustness probe fires on only 2.5% of
designs (over_sr median exactly 1.00, 8 positives in 1200), far too few for a cross-validated AUC.

That is itself informative (the population is robust once properly tuned) but it leaves Claim 2
resting on the ORIGINAL label, whose two known weaknesses are a 1.5% base rate and the fact that
29 of its 36 positives are flagged `uncertain` by the project's own Wilson-interval check.

This script re-labels with a harsher probe: SR at 2.0x the re-tuned gains. It reuses the gains
already computed by population_retune.py, so it costs one evaluation per design rather than a full
re-search.

WHAT IT DECIDES
  screen holds on the 2x label  -> Claim 2's ground truth is sound and the screen is real; the
                                   remaining caveat is only the deployment base rate.
  screen collapses              -> the screen tracks the original labelling procedure rather than
                                   controllability, and must be restated.
  2x probe also barely fires    -> no independent test is available from over-robustness at all;
                                   say so plainly rather than reporting the old-label AUC as if
                                   it had been checked.

SEEDS: 350001-350015, disjoint from all prior experiments (retune used 330101-330115).
INPUT : experiments/results/population_retune_py.csv   (needs kp_new / kd_new)
OUTPUT: experiments/results/screen_retest_2x_py.csv
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
from experiment_runner import _run_one, _aggregate

EVAL_SEEDS = tuple(range(350001, 350016))
SCALES     = (1.4, 2.0, 3.0)          # 1.4 reproduces the retune; 2.0 and 3.0 are the harsher probes
SR_PASS    = 0.80

RETUNE_CSV = Path("experiments/results/population_retune_py.csv")
POP_CSV    = Path("experiments/results/exp1_final_population_py.csv")
OUT_CSV    = Path("experiments/results/screen_retest_2x_py.csv")


def probe(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    out = dict(rocket_id=row["rocket_id"], Iyy=row["Iyy"],
               motor_scale=row["motor_scale"], latency_steps=float(row["latency_steps"]),
               kp_new=row["kp_new"], kd_new=row["kd_new"])
    for s in SCALES:
        pid = PIDParams(Kp=float(s * row["kp_new"]), Kd=float(s * row["kd_new"]), Ki=0.0,
                        u_max=act.u_max, i_lim=act.u_max)
        m = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=sd) for sd in EVAL_SEEDS])
        out[f"sr_{s:g}x"] = m["success_rate"]
        out[f"fail_{s:g}x"] = int(m["success_rate"] < SR_PASS)
    return out


def screen_auc(d: pd.DataFrame, label: str):
    from sklearn.linear_model import LogisticRegression
    from sklearn.model_selection import StratifiedKFold
    from sklearn.metrics import roc_auc_score, average_precision_score
    y = d[label].values
    if y.sum() < 10 or (1 - y).sum() < 10:
        return None
    X = np.column_stack([np.log(1.0 / d.Iyy), np.log(14.4 * d.motor_scale * 0.25),
                         np.log(d.latency_steps / 200.0)])
    aucs, aps = [], []
    for seed in range(5):
        for tr, te in StratifiedKFold(5, shuffle=True, random_state=seed).split(X, y):
            m = LogisticRegression(max_iter=20000, C=1e6).fit(X[tr], y[tr])
            p = m.predict_proba(X[te])[:, 1]
            aucs.append(roc_auc_score(y[te], p))
            aps.append(average_precision_score(y[te], p))
    return float(np.mean(aucs)), float(np.std(aucs)), float(np.mean(aps)), float(y.mean())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--retune", type=Path, default=RETUNE_CSV)
    ap.add_argument("--pop", type=Path, default=POP_CSV)
    ap.add_argument("--jobs", type=int, default=-1)
    args = ap.parse_args()
    for p in (args.retune, args.pop):
        if not p.exists():
            sys.exit(f"missing {p}")

    rt = pd.read_csv(args.retune).drop_duplicates("rocket_id")[
        ["rocket_id", "kp_new", "kd_new", "label_old"]]
    pop = pd.read_csv(args.pop)
    d = pop.merge(rt, on="rocket_id", how="inner")
    print(f"=== screen re-test: probing {len(d)} re-tuned designs at {SCALES} ===")
    print(f"  {len(d) * len(SCALES) * len(EVAL_SEEDS):,} sims")

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    res = pd.DataFrame(Parallel(n_jobs=args.jobs, verbose=0)(
        delayed(probe)(r) for r in d.to_dict("records")))
    res = res.merge(rt[["rocket_id", "label_old"]], on="rocket_id")
    res.to_csv(OUT_CSV, index=False)
    print(f"Saved {len(res)} rows -> {OUT_CSV}\n")

    print("=== does each probe actually fire? ===")
    for s in SCALES:
        c = res[f"fail_{s:g}x"]
        print(f"  {s:g}x gains: {c.sum():4d} positives of {len(res)}  ({100*c.mean():5.1f}%)  "
              f"median SR {res[f'sr_{s:g}x'].median():.2f}")
    print(f"  old label: {res.label_old.sum():4d} positives ({100*res.label_old.mean():5.1f}%)")

    print("\n=== four-number screen (1/Iyy, T*L, tau) against each label ===")
    for lab in ["label_old"] + [f"fail_{s:g}x" for s in SCALES]:
        r = screen_auc(res, lab)
        if r is None:
            print(f"  {lab:<12} too few positives to test")
        else:
            m, sd, apv, base = r
            print(f"  {lab:<12} CV AUC = {m:.4f} +/- {sd:.4f}   avg precision {apv:.3f} "
                  f"(no-skill {base:.3f})")
    print("\n  A screen that holds across probe strengths is tracking controllability.")
    print("  One that only works on label_old is tracking the original labelling procedure.")

    print("\n=== agreement between old label and the harsher probes ===")
    for s in SCALES:
        col = f"fail_{s:g}x"
        both = int(((res.label_old == 1) & (res[col] == 1)).sum())
        print(f"  {s:g}x: {both} of {int(res.label_old.sum())} old positives also fail at {s:g}x")


if __name__ == "__main__":
    main()
