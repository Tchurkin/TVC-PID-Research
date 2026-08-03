"""
tools/ceiling_kd_free.py  —  is the gain ceiling's authority-independence a frozen-Kd artifact?

HYPOTHESIS
----------
Claim 1 of the paper is that the proportional-gain ceiling is set by loop delay and is INDEPENDENT
of rotational authority (keff). Every dataset supporting it measured the ceiling with Kd frozen at
a single per-design value:

    v2      (frozen Kd)  keff coef -0.005  (t = -0.1)   <- supports the claim
    v3      (frozen Kd)  keff coef -0.019  (t = -0.3)   <- supports the claim
    holdout (frozen Kd)  keff coef -0.068  (t = -1.0)   <- supports the claim
    v4      (Kd free)    keff coef -0.516  (t = -3.5)   <- CONTRADICTS the claim
    v5b     (Kd ratios)  keff coef -1.367  (t = -1.8)   <- CONTRADICTS the claim

The protocol choice is perfectly confounded with the conclusion, and the two samples that vary it
are small (n = 25, 8) and heavily censored (79%, 65%), so neither side is decisive.

H0: the ceiling is keff-independent under BOTH protocols -> Claim 1 stands as written.
H1: keff-independence appears only when Kd is frozen -> Claim 1 is a measurement artifact and
    must be withdrawn or restated as "at a fixed Kd, which is what a builder actually flies."

DESIGN — the fix for the previous attempts is PAIRING
----------------------------------------------------
Both protocols are measured on the SAME designs with the SAME seeds in one pass, so the comparison
isolates the protocol and nothing else. The free-Kd arm's grid contains the frozen-Kd arm as a
single column, so pairing costs no extra simulations.

  free-Kd ceiling   : highest Kp at which ANY tested Kd achieves SR >= 0.80  (existential)
  frozen-Kd ceiling : highest Kp at which the ONE reference Kd achieves SR >= 0.80
                      (reference Kd = the ratio that performs best at mid-Kp, i.e. what a
                       per-design tuner would have frozen)

Designs are stratified over a keff x latency grid so the two factors vary independently -- the
whole question is whether they can be separated, and a plain LHS draw makes them collinear.

CENSORING IS A RESULT, NOT A NUISANCE. If freeing Kd removes the ceiling entirely for most
designs, that is the finding: "the ceiling" would then be a property of holding Kd fixed. The
script reports censoring by keff tier, because under a truly keff-independent ceiling the
censoring rate must not depend on keff either.

SEEDS: 320001-320010 (eval). Disjoint from all prior experiments.
OUTPUT: experiments/results/ceiling_kd_free_py.csv   (one row per design)

USAGE
-----
  python tools/ceiling_kd_free.py --n 12          # smoke test
  python tools/ceiling_kd_free.py                 # 160 designs
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

KP_GRID    = np.geomspace(1.0, 2000.0, 22)          # wide: prior runs censored at 500
# ABSOLUTE Kd values, not ratios -- the paper's protocol froze Kd in absolute terms while Kp moved.
# The grid is CENTRED ON THE PAPER'S ACTUAL VALUES and contains them exactly: window_ratio_v2 used
# opt_Kd = 1.0 for 108 of 120 designs, with 2.0/4.0/8.0 for the remainder. A first attempt used
# geomspace(0.05, 64, 8), whose per-design selection centred at ~8.3 -- 8x above the paper's
# regime -- and consequently failed to reproduce the established latency effect at all
# (rho(lat, ceiling) = -0.04 vs the published -0.74). Do not widen this grid without re-checking
# the positive control below.
KD_GRID    = np.array([0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0])
KD1_IX     = 3                                      # index of Kd = 1.0, the paper's actual value
EVAL_SEEDS = tuple(range(320001, 320011))           # 10
SR_PASS    = 0.80
MID_KP_IX  = len(KP_GRID) // 2                      # where the per-design reference Kd is chosen

# POSITIVE CONTROL. Before any conclusion is drawn about keff, the replication arm (Kd fixed at
# 1.0, exactly the paper's protocol) must recover the published latency dependence. If it does
# not, the grid is in the wrong regime and the free-Kd arm's coefficients mean nothing. This is
# the check whose absence invalidated the first run.
CONTROL_RHO_TARGET = -0.74                          # window_ratio_v2: rho(latency, kp_ceiling)
CONTROL_RHO_MAX    = -0.45                          # must be at least this negative to proceed

OUT_CSV     = Path("experiments/results/ceiling_kd_free_py.csv")
DESIGNS_CSV = Path("experiments/results/exp1_final_population_py.csv")

RESTORE_HINT = f"""
{DESIGNS_CSV} is not in the working tree (deleted by commit bb22d36). Restore with:
    git archive bb22d36^ experiments/results | tar -x -C .
or pass --designs <path>.
"""


def sweep_design(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    # sr[i, j] = success rate at (KP_GRID[i], KD_GRID[j])  -- absolute Kd
    sr = np.zeros((len(KP_GRID), len(KD_GRID)))
    for i, kp in enumerate(KP_GRID):
        for j, kd in enumerate(KD_GRID):
            pid = PIDParams(Kp=float(kp), Kd=float(kd), Ki=0.0,
                            u_max=act.u_max, i_lim=act.u_max)
            runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in EVAL_SEEDS]
            sr[i, j] = float(np.mean([r["success"] for r in runs]))

    # --- free-Kd arm: existential over Kd at each Kp ---
    pass_free = sr.max(axis=1) >= SR_PASS
    free_ceiling = float(KP_GRID[pass_free][-1]) if pass_free.any() else np.nan
    free_censored = int(bool(pass_free[-1]))        # still passing at the top of the grid

    # --- frozen-Kd arm: the Kd a per-design tuner would have picked, held fixed ---
    ref_j = int(np.argmax(sr[MID_KP_IX]))
    pass_frozen = sr[:, ref_j] >= SR_PASS
    froz_ceiling = float(KP_GRID[pass_frozen][-1]) if pass_frozen.any() else np.nan
    froz_censored = int(bool(pass_frozen[-1]))

    # --- replication arm: Kd fixed at 1.0 for every design, exactly as window_ratio_v2 did.
    #     This is the positive control; it must recover the published latency dependence. ---
    pass_kd1 = sr[:, KD1_IX] >= SR_PASS
    kd1_ceiling = float(KP_GRID[pass_kd1][-1]) if pass_kd1.any() else np.nan
    kd1_censored = int(bool(pass_kd1[-1]))

    tau = float(row["latency_steps"]) / 200.0
    return dict(
        rocket_id=row["rocket_id"], Iyy=row["Iyy"], motor_scale=row["motor_scale"],
        latency_steps=float(row["latency_steps"]), tau=tau,
        keff=14.4 * float(row["motor_scale"]) * 0.25 / float(row["Iyy"]),
        wind_strength=row.get("wind_strength"), final_label=row.get("final_label"),
        ceiling_free=free_ceiling,   ceil_free_censored=free_censored,
        ceiling_frozen=froz_ceiling, ceil_frozen_censored=froz_censored,
        ceiling_kd1=kd1_ceiling,     ceil_kd1_censored=kd1_censored,
        ref_kd=float(KD_GRID[ref_j]),
        n_pass_free=int(pass_free.sum()), n_pass_frozen=int(pass_frozen.sum()),
        n_pass_kd1=int(pass_kd1.sum()), peak_sr=float(sr.max()),
    )


def fit(g: pd.DataFrame, col: str, tag: str) -> None:
    """log(ceiling) ~ log(keff) + log(tau), with standard errors."""
    d = g[g[col].notna()]
    if len(d) < 8:
        print(f"  {tag:<26} n={len(d):<3} too few to fit")
        return
    y = np.log(d[col].values)
    X = np.column_stack([np.ones(len(d)), np.log(d.keff.values), np.log(d.tau.values)])
    b, *_ = np.linalg.lstsq(X, y, rcond=None)
    yh = X @ b
    n, k = X.shape
    s2 = ((y - yh) ** 2).sum() / max(n - k, 1)
    se = np.sqrt(np.diag(s2 * np.linalg.inv(X.T @ X)))
    r2 = 1 - ((y - yh) ** 2).sum() / max(((y - y.mean()) ** 2).sum(), 1e-12)
    print(f"  {tag:<26} n={len(d):<3} keff {b[1]:+.3f} (SE {se[1]:.3f}, t={b[1]/se[1]:+.1f}, "
          f"95% CI [{b[1]-1.96*se[1]:+.2f},{b[1]+1.96*se[1]:+.2f}])  "
          f"tau {b[2]:+.3f}  R2={r2:.3f}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=160)
    ap.add_argument("--designs", type=Path, default=DESIGNS_CSV)
    ap.add_argument("--jobs", type=int, default=-1)
    args = ap.parse_args()

    if not args.designs.exists():
        sys.exit(RESTORE_HINT)
    pop = pd.read_csv(args.designs)
    pop = pop.assign(keff=14.4 * pop.motor_scale * 0.25 / pop.Iyy)

    # Stratify over keff x latency so the two predictors are not collinear.
    # --n 0 (or >= population size) means "use every design", which is what you want when the
    # designs file is already a curated set. Without this guard, per = max(1, 0 // n_cells) = 1
    # and the run silently collapses to one design per stratification cell.
    if args.n <= 0 or args.n >= len(pop):
        designs = pop.reset_index(drop=True)
    else:
        pop["_kt"] = pd.qcut(np.log(pop.keff), 4, labels=False, duplicates="drop")
        rng = np.random.default_rng(909)
        cells = list(pop.groupby(["_kt", "latency_steps"]))
        per = max(1, args.n // max(len(cells), 1))
        parts = [g.sample(min(per, len(g)), random_state=int(rng.integers(1e9)))
                 for _, g in cells]
        designs = pd.concat(parts).drop(columns="_kt").reset_index(drop=True)

    print("=== ceiling with Kd FREE vs FROZEN (paired, same designs and seeds) ===")
    print(f"  designs={len(designs)}  Kp grid={len(KP_GRID)}  Kd grid={len(KD_GRID)}  "
          f"seeds={len(EVAL_SEEDS)}")
    print(f"  ~{len(designs)*len(KP_GRID)*len(KD_GRID)*len(EVAL_SEEDS):,} sims")

    rows = designs.to_dict("records")
    out = pd.DataFrame(Parallel(n_jobs=args.jobs, verbose=5)(
        delayed(sweep_design)(r) for r in rows))
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(out)} rows -> {OUT_CSV}")

    print("\n=== censoring (a ceiling that vanishes when Kd is freed IS the result) ===")
    print(f"  free-Kd   : {100*out.ceil_free_censored.mean():.0f}% still passing at Kp={KP_GRID[-1]:.0f}")
    print(f"  frozen-Kd : {100*out.ceil_frozen_censored.mean():.0f}%")
    out["keff_tier"] = pd.qcut(np.log(out.keff), 3, labels=["lo", "mid", "hi"])
    print("\n  censoring by keff tier (must be flat if the ceiling is keff-independent):")
    print(out.groupby("keff_tier", observed=True)[["ceil_free_censored", "ceil_frozen_censored"]]
             .mean().round(3).to_string())

    print("\n=== POSITIVE CONTROL: does the Kd=1.0 replication arm recover the published effect? ===")
    ctl = out[(out.ceil_kd1_censored == 0) & out.ceiling_kd1.notna()]
    rho = stats.spearmanr(ctl.latency_steps, ctl.ceiling_kd1)[0] if len(ctl) > 8 else np.nan
    print(f"  rho(latency, ceiling_kd1) = {rho:+.3f}   n={len(ctl)}"
          f"   [published window_ratio_v2: {CONTROL_RHO_TARGET:+.2f}]")
    ok = np.isfinite(rho) and rho <= CONTROL_RHO_MAX
    print(f"  CONTROL {'PASSED' if ok else 'FAILED'}"
          f" (threshold {CONTROL_RHO_MAX:+.2f})")
    if not ok:
        print("\n  *** The replication arm does not reproduce the known latency dependence.")
        print("  *** The grid is in the wrong regime; the keff coefficients below are NOT")
        print("  *** interpretable. Fix the grid before drawing any conclusion about Claim 1.")

    print("\n=== THE TEST: log(ceiling) ~ log(keff) + log(tau) ===")
    g = out[(out.ceil_free_censored == 0) & (out.ceil_frozen_censored == 0)
            & out.ceiling_free.notna() & out.ceiling_frozen.notna()]
    print(f"  (paired subset, both arms uncensored and measurable: n={len(g)})")
    fit(g, "ceiling_kd1",    "Kd=1.0    (replication)")
    fit(g, "ceiling_frozen", "FROZEN Kd (per-design)")
    fit(g, "ceiling_free",   "FREE Kd   (fair protocol)")
    print("\n  Claim 1 asserts keff coef = 0. Interpret ONLY if the control above passed:")
    print("  if the frozen rows are ~0 and the FREE row is not, authority-independence is a")
    print("  property of holding Kd fixed, not of the plant.")
    if len(g) >= 8:
        r = np.log(g.ceiling_free / g.ceiling_frozen)
        print(f"\n  paired ceiling ratio free/frozen: median {np.exp(np.median(r)):.2f}x  "
              f"(fraction > 1: {100*(r > 0).mean():.0f}%)")
        print(f"  Wilcoxon signed-rank: p = {stats.wilcoxon(np.log(g.ceiling_free), np.log(g.ceiling_frozen)).pvalue:.2e}")


if __name__ == "__main__":
    main()
