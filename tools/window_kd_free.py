"""
tools/window_kd_free.py  —  does the gain-window narrowing survive Kd being free?

THE DECISION THIS MAKES
-----------------------
Six sections of the inherited draft are built on `window_ratio = Kp_ceiling / Kp_floor` as measured
with Kd FROZEN at a single per-design value (window_ratio_v2/v3). The project's own pre-registered
`frozen_kd_artifact_test` (2026-06-22) found that freezing Kd inflates the measured FLOOR by
1.8-2.9x on 8/8 designs, and the follow-ups v4/v5/v5b collapsed the floor law entirely
(keff exponent +1.06 -> +0.21 -> -0.20). But those follow-ups were heavily censored (68-79%) and
left only 13 / 3 / 8 usable designs, so they never settled it.

This run settles it, and the outline depends on the answer:
  window narrowing SURVIVES a free Kd  -> the six sections live, restated with the caveat.
  window narrowing VANISHES            -> they are cut from the 20 pages and the Kd-artifact
                                          methods chapter absorbs them.

DESIGN
------
Paired, on window_ratio_v2's EXACT 120 designs (regenerated from sample_lhs(10000, 8888)), so a
positive control is available. Three arms share one Kp x Kd grid, so pairing is free:

  Kd = 1.0 (REPLICATION)  v2 used opt_Kd = 1.0 for 108 of its 120 designs. This arm is the POSITIVE
                          CONTROL: it must reproduce v2's published window regression before the
                          free-Kd arm means anything.
  FROZEN (per-design)     the Kd that performs best at mid-Kp, then held fixed while Kp sweeps --
                          what a builder who tunes D once and then tunes P actually gets.
  FREE (existential)      at each Kp, does ANY tested Kd pass? The window a builder gets if Kd is
                          allowed to track Kp.

Floor = lowest passing Kp, ceiling = highest passing Kp, window = ceiling/floor, all at SR >= 0.80.

CENSORING IS REPORTED, NOT HIDDEN. v4/v5/v5b failed by censoring; if the free arm censors heavily
here the honest output is "not measurable on this grid", not a regression on the survivors.

PUBLISHED TARGETS (window_ratio_v2, n=82 non-censored):
  log(window) ~ log(keff) + log(latency):  R2 = 0.659, CV = 0.616 +/- 0.080
  coefficients: keff = -1.192, latency = -1.880
  ceiling arm: keff = -0.005 (~0), latency = -0.862   |   floor arm: keff = +1.061, latency = +0.964

SEEDS: 360001-360015 (15, matching v2's count). Disjoint from all prior experiments.
OUTPUT: experiments/results/window_kd_free_py.csv   (checkpointed per chunk, resumes on restart)

USAGE
  python tools/window_kd_free.py --designs <v2_designs.csv> --n 24    # smoke
  python tools/window_kd_free.py --designs <v2_designs.csv>           # all 120
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
from scipy import stats

from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams
from experiment_runner import _run_one

KP_GRID    = np.geomspace(0.1, 800.0, 32)                       # v2's exact sweep
KD_GRID    = np.array([0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0])  # contains v2's 1/2/4/8
KD1_IX     = 3                                                   # index of Kd = 1.0
EVAL_SEEDS = tuple(range(360001, 360016))                        # 15, matching v2
SR_PASS    = 0.80
MID_KP_IX  = len(KP_GRID) // 2

# v2 published, for the positive control
PUB = dict(win_keff=-1.192, win_lat=-1.880, win_r2=0.659,
           ceil_keff=-0.005, ceil_lat=-0.862, floor_keff=+1.061, floor_lat=+0.964)
CONTROL_TOL = 0.45          # |observed - published| allowed on the window exponents

OUT_CSV = Path("experiments/results/window_kd_free_py.csv")


def _edges(mask: np.ndarray):
    """(floor, ceiling, floor_censored, ceil_censored) from a boolean pass-mask over KP_GRID."""
    if not mask.any():
        return np.nan, np.nan, 0, 0
    idx = np.flatnonzero(mask)
    return (float(KP_GRID[idx[0]]), float(KP_GRID[idx[-1]]),
            int(idx[0] == 0), int(idx[-1] == len(KP_GRID) - 1))


def sweep(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    sr = np.zeros((len(KP_GRID), len(KD_GRID)))
    for i, kp in enumerate(KP_GRID):
        for j, kd in enumerate(KD_GRID):
            pid = PIDParams(Kp=float(kp), Kd=float(kd), Ki=0.0,
                            u_max=act.u_max, i_lim=act.u_max)
            runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in EVAL_SEEDS]
            sr[i, j] = float(np.mean([r["success"] for r in runs]))

    out = dict(rocket_id=row["rocket_id"], Iyy=row["Iyy"], motor_scale=row["motor_scale"],
               latency_steps=float(row["latency_steps"]),
               keff=14.4 * float(row["motor_scale"]) * 0.25 / float(row["Iyy"]),
               wind_strength=row.get("wind_strength"))

    ref_j = int(np.argmax(sr[MID_KP_IX]))
    out["ref_kd"] = float(KD_GRID[ref_j])
    for tag, mask in (("kd1",    sr[:, KD1_IX] >= SR_PASS),
                      ("frozen", sr[:, ref_j]  >= SR_PASS),
                      ("free",   sr.max(axis=1) >= SR_PASS)):
        f, c, fc, cc = _edges(mask)
        out[f"floor_{tag}"] = f
        out[f"ceil_{tag}"]  = c
        out[f"window_{tag}"] = (c / f) if (f and f == f and c == c) else np.nan
        out[f"floorcens_{tag}"] = fc
        out[f"ceilcens_{tag}"]  = cc
        out[f"npass_{tag}"] = int(mask.sum())
    return out


def fit(d: pd.DataFrame, col: str, tag: str, cens: list[str]):
    g = d[d[col].notna()]
    for c in cens:
        g = g[g[c] == 0]
    if len(g) < 10:
        print(f"  {tag:<30} n={len(g):<3} too few after censoring — NOT MEASURABLE on this grid")
        return None
    y = np.log(g[col].values)
    X = np.column_stack([np.ones(len(g)), np.log(g.keff.values), np.log(g.latency_steps.values)])
    b, *_ = np.linalg.lstsq(X, y, rcond=None)
    yh = X @ b
    n, k = X.shape
    se = np.sqrt(np.diag(((y - yh) ** 2).sum() / max(n - k, 1) * np.linalg.inv(X.T @ X)))
    r2 = 1 - ((y - yh) ** 2).sum() / max(((y - y.mean()) ** 2).sum(), 1e-12)
    print(f"  {tag:<30} n={len(g):<3} keff {b[1]:+.3f} (SE {se[1]:.3f})  "
          f"lat {b[2]:+.3f} (SE {se[2]:.3f})  R2={r2:.3f}")
    return b[1], b[2], r2, len(g)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--designs", type=Path, required=True)
    ap.add_argument("--n", type=int, default=0)
    ap.add_argument("--jobs", type=int, default=-1)
    args = ap.parse_args()

    pop = pd.read_csv(args.designs)
    if args.n and args.n < len(pop):
        pop = pop.sample(args.n, random_state=7).reset_index(drop=True)

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if OUT_CSV.exists():
        try:
            done = set(pd.read_csv(OUT_CSV).rocket_id)
            print(f"  resuming: {len(done)} already complete")
        except Exception:
            done = set()
    rows = [r for r in pop.to_dict("records") if r["rocket_id"] not in done]
    print(f"=== Kd-free gain-window rerun: {len(rows)} designs of {len(pop)} ===")
    print(f"  {len(rows) * len(KP_GRID) * len(KD_GRID) * len(EVAL_SEEDS):,} sims")

    CHUNK = 20
    for i in range(0, len(rows), CHUNK):
        res = pd.DataFrame(Parallel(n_jobs=args.jobs, verbose=0)(
            delayed(sweep)(r) for r in rows[i:i + CHUNK]))
        res.to_csv(OUT_CSV, mode="a", header=not OUT_CSV.exists(), index=False)
        print(f"  chunk {i//CHUNK + 1}/{-(-len(rows)//CHUNK)}", flush=True)

    d = pd.read_csv(OUT_CSV).drop_duplicates("rocket_id")
    print(f"\nSaved {len(d)} designs -> {OUT_CSV}\n")

    print("=== censoring by arm ===")
    for tag in ("kd1", "frozen", "free"):
        print(f"  {tag:<7} floor {100*d[f'floorcens_{tag}'].mean():5.1f}%   "
              f"ceiling {100*d[f'ceilcens_{tag}'].mean():5.1f}%   "
              f"median window {d[f'window_{tag}'].median():8.1f}x")

    print("\n=== POSITIVE CONTROL: does Kd=1.0 reproduce window_ratio_v2? ===")
    r = fit(d, "window_kd1", "Kd=1.0 window (replication)", ["floorcens_kd1", "ceilcens_kd1"])
    print(f"  {'published v2':<30}      keff {PUB['win_keff']:+.3f}"
          f"          lat {PUB['win_lat']:+.3f}          R2={PUB['win_r2']:.3f}")
    ok = r is not None and abs(r[0] - PUB["win_keff"]) <= CONTROL_TOL \
                       and abs(r[1] - PUB["win_lat"]) <= CONTROL_TOL
    print(f"  CONTROL {'PASSED' if ok else 'FAILED'} (tolerance +/-{CONTROL_TOL} on both exponents)")
    if not ok:
        print("\n  *** Replication failed. The free-Kd arm below is NOT interpretable.")
        print("  *** Report this as 'could not re-measure', not as a null result.")
        return

    print("\n=== THE TEST: does the window still narrow when Kd is free? ===")
    fit(d, "window_frozen", "FROZEN Kd (per-design)", ["floorcens_frozen", "ceilcens_frozen"])
    fit(d, "window_free",   "FREE Kd (existential)",  ["floorcens_free", "ceilcens_free"])
    print("\n  decomposition (which edge moves):")
    fit(d, "ceil_kd1",   "  ceiling, Kd=1.0", ["ceilcens_kd1"])
    fit(d, "ceil_free",  "  ceiling, free",   ["ceilcens_free"])
    fit(d, "floor_kd1",  "  floor,   Kd=1.0", ["floorcens_kd1"])
    fit(d, "floor_free", "  floor,   free",   ["floorcens_free"])

    print("\n=== paired within-design effect of freeing Kd ===")
    g = d[(d.window_kd1.notna()) & (d.window_free.notna())]
    if len(g) >= 8:
        ratio = g.window_free / g.window_kd1
        print(f"  window free/frozen: median {ratio.median():.2f}x  "
              f"(n={len(g)}, {100*(ratio > 1).mean():.0f}% wider)  "
              f"Wilcoxon p={stats.wilcoxon(np.log(g.window_free), np.log(g.window_kd1)).pvalue:.2e}")
    print("\n  VERDICT RULE: if the free-Kd keff/lat exponents collapse toward zero, the six")
    print("  inherited window sections are cut from the 20 pages and the Kd-artifact methods")
    print("  chapter absorbs them. If they hold, those sections live with the caveat stated.")


if __name__ == "__main__":
    main()
