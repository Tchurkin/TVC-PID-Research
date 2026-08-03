"""
tools/s2r_ic_sweep.py  —  is the still-air overtuning result an artifact of two free choices?

HYPOTHESIS
----------
The published sim-to-real result (a gain tuned in a disturbance-free simulator fails under full
physics, at a rate rising with authority x delay) rests on two parameters that were fixed by fiat:

  1. the initial attitude error used during still-air tuning   (was: theta0_fixed_deg = 10.0)
  2. the top of the gain search grid                           (was: kp_max = 320)

Both plausibly manufacture the effect. With wind, slew, noise and aero all disabled, nothing in
the still-air model penalises a higher gain -- RMS falls monotonically with Kp until the gimbal
limit u_max binds -- so the search runs to the top of whatever grid it is given. A LARGER initial
error makes the tuner smarter (recovery from a big offset saturates u_max, which creates a real
upper bound on useful gain even in still air), so it should SHRINK the measured overtuning. A
LOWER grid ceiling should shrink it too, mechanically.

H0: overtuning rate and its ranking against risk are unchanged across IC and kp_max.
    -> the effect is the missing ceiling signal, and the claim is safe.
H1: the rate tracks IC and/or kp_max.
    -> the effect is partly the experimenter's grid, and the claim must be restated.

METHOD
------
Full factorial: IC in {5, 10, 20, 30} deg  x  kp_max in {100, 320, 1000}  = 12 cells.
Per design per cell:
  1. Tune in FidelityConfig.simple() at the cell's IC, SINGLE-KNOB: Kp swept, Kd tied to Kp by a
     fixed ratio (see KD_RATIO). Single-knob because (a) it is what a naive builder does and
     (b) it removes the decoupled-Kd confound in the original autotune, which picked Kd once at
     a fixed Kp=40 and then froze it while Kp moved.
  2. Fly those gains in FidelityConfig.full() over N_EVAL_SEEDS fresh seeds.
  3. Fly the reference full-physics gains (Exp1 best_Kp/best_Kd) over the same seeds.
Both failure gates are recorded, because the original study conflated them in reporting:
  gate_infeasible : SR < 0.35  (FRAGILE_SUCCESS_RATE, sim/units.py -- what the published
                    9.1% / 61.3% numbers actually used, at 3-seed resolution)
  gate_window     : SR < 0.80  (the usable-gain-window standard used everywhere else in the paper)

WHY 15 SEEDS. The original used 3, so success rate could only take {0, 1/3, 2/3, 1} -- a 33-point
resolution on the quantity the whole claim rests on, and the same mistake the project already
documented for the 3-seed regime classification. 15 seeds gives ~6.7-point resolution.

A STRUCTURAL CHECK RUNS FIRST. On a bare double integrator with all aero disabled, proportional-
only control is undamped, which would make single-knob tuning degenerate in a new way. The script
measures whether the simple model retains any rate damping (Kd=0 decay test) and prints the answer
before sweeping. Read it -- if the model is undamped, KD_RATIO is doing real work and must be
reported as a tuning assumption, not a detail.

SEEDS: eval 310001-310015, tuning objective 310101-310102. Disjoint from all prior experiments.
OUTPUT: experiments/results/s2r_ic_sweep_py.csv  (one row per design x cell)

USAGE
-----
  python tools/s2r_ic_sweep.py                       # 300-design stratified sweep, 12 cells
  python tools/s2r_ic_sweep.py --n 500               # bigger sample
  python tools/s2r_ic_sweep.py --headline --n 0      # full population, canonical cell only
"""

from __future__ import annotations

import argparse
import os
import sys
import warnings
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
warnings.filterwarnings("ignore")

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import FRAGILE_SUCCESS_RATE

# ── Experiment constants ──────────────────────────────────────────────────────
IC_DEGS       = (5.0, 10.0, 20.0, 30.0)     # still-air tuning initial attitude error
KP_MAXES      = (100.0, 320.0, 1000.0)      # top of the still-air gain search
CANONICAL     = (20.0, 320.0)               # cell used for the --headline run
KP_MIN        = 1.0
N_COARSE      = 12
N_REFINE      = 8
KD_RATIO      = 0.05                        # Kd = KD_RATIO * Kp  (single-knob tuning)
EVAL_SEEDS    = tuple(range(310001, 310016))    # 15
TUNE_SEEDS    = (310101, 310102)                # 2, objective only
GATE_WINDOW   = 0.80

OUT_CSV       = Path("experiments/results/s2r_ic_sweep_py.csv")
DESIGNS_CSV   = Path("experiments/results/exp1_final_population_py.csv")

RESTORE_HINT = f"""
{DESIGNS_CSV} is not in the working tree -- the simulation-phase result files were
deleted by commit bb22d36 (repo reorganisation). Restore a read-only snapshot with:

    git archive bb22d36^ experiments/results | tar -x -C .

or point at your own copy with --designs <path>.
"""


# ── Still-air tuning: single knob ─────────────────────────────────────────────
def tune_still_air(plant, act, sen, dis, sc, kp_max: float) -> tuple[float, float]:
    """
    Sweep Kp with Kd = KD_RATIO * Kp. Objective: mean success rate over TUNE_SEEDS,
    RMS as tiebreak -- the same selection rule as the original autotune, so any change
    in the result comes from IC and kp_max, not from a different objective.
    """
    def score(kp: float) -> tuple[float, float]:
        pid = PIDParams(Kp=float(kp), Kd=float(KD_RATIO * kp), Ki=0.0,
                        u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in TUNE_SEEDS]
        return (float(np.mean([r["success"] for r in runs])),
                float(np.mean([r["rms_error_deg"] for r in runs])))

    best_kp, best = KP_MIN, (-1.0, float("inf"))
    for kp in np.geomspace(KP_MIN, kp_max, N_COARSE):
        s = score(kp)
        if s[0] > best[0] or (s[0] == best[0] and s[1] < best[1]):
            best, best_kp = s, float(kp)
    lo, hi = best_kp / 1.5, min(best_kp * 1.5, kp_max)
    for kp in np.geomspace(lo, hi, N_REFINE):
        s = score(kp)
        if s[0] > best[0] or (s[0] == best[0] and s[1] < best[1]):
            best, best_kp = s, float(kp)
    return best_kp, KD_RATIO * best_kp


def _eval(plant, act, sen, dis, sc, pid) -> dict:
    return _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in EVAL_SEEDS])


# ── One (design, cell) ────────────────────────────────────────────────────────
def eval_cell(row: dict, ic_deg: float, kp_max: float) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)

    # Tune: simple physics, deterministic initial offset at the cell's IC.
    sc_tune = replace(build_scenario(), theta0_fixed_deg=float(ic_deg))
    a_s, s_s, d_s, sc_s = apply_fidelity_config(act, sen, dis, sc_tune,
                                                FidelityConfig.simple())
    kp_s, kd_s = tune_still_air(plant, a_s, s_s, d_s, sc_s, kp_max)

    # Fly: full physics, zero initial offset (disturbance-rejection task).
    sc_full = build_scenario(theta0_bias_std=0.0)
    pid_s = PIDParams(Kp=kp_s, Kd=kd_s, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    m_s = _eval(plant, act, sen, dis, sc_full, pid_s)

    kp_f = float(row.get("best_Kp", kp_s))
    kd_f = float(row.get("best_Kd", kd_s))
    pid_f = PIDParams(Kp=kp_f, Kd=kd_f, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    m_f = _eval(plant, act, sen, dis, sc_full, pid_f)

    sr_s, sr_f = m_s["success_rate"], m_f["success_rate"]
    lat = float(row["latency_steps"])
    ceil_theory = 380.0 / lat
    return dict(
        rocket_id=row["rocket_id"], ic_deg=ic_deg, kp_max=kp_max,
        Iyy=row["Iyy"], motor_scale=row["motor_scale"], latency_steps=lat,
        wind_strength=row.get("wind_strength"), final_label=row.get("final_label"),
        kp_simple=kp_s, kd_simple=kd_s, kp_full=kp_f, kd_full=kd_f,
        kp_ceiling_theory=ceil_theory,
        at_grid_top=int(kp_s >= 0.98 * kp_max),      # did the search saturate its own ceiling?
        overtuned=int(kp_s > ceil_theory),
        overtune_severity=kp_s / ceil_theory,
        sr_simple_in_full=sr_s, sr_full_in_full=sr_f, sr_gap=sr_f - sr_s,
        rms_simple_in_full=m_s["rms_error_deg"], rms_full_in_full=m_f["rms_error_deg"],
        # BOTH gates, so the paper never has to be vague about which one it means
        fail_infeasible=int(sr_s < FRAGILE_SUCCESS_RATE),
        fail_window=int(sr_s < GATE_WINDOW),
        ref_fail_infeasible=int(sr_f < FRAGILE_SUCCESS_RATE),
        ref_fail_window=int(sr_f < GATE_WINDOW),
    )


# ── Structural check: is the simple model damped at all? ──────────────────────
def damping_check(row: dict) -> None:
    plant = build_plant(row)
    act, sen, dis = build_actuator(row), build_sensor(row), build_disturbance(row)
    sc = replace(build_scenario(), theta0_fixed_deg=10.0)
    a_s, s_s, d_s, sc_s = apply_fidelity_config(act, sen, dis, sc, FidelityConfig.simple())
    print("\n--- structural check: proportional-only control in the simple model ---")
    for kd in (0.0, KD_RATIO * 40.0):
        pid = PIDParams(Kp=40.0, Kd=kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        r = _run_one(pid, plant, a_s, s_s, d_s, sc_s, seed=TUNE_SEEDS[0])
        print(f"  Kp=40 Kd={kd:5.2f} -> rms={r['rms_error_deg']:7.2f} deg  "
              f"peak={r['peak_error_deg']:7.2f} deg  end={r['end_error_deg']:7.2f} deg")
    print("  If the Kd=0 row does not decay, the simple model has no rate damping and")
    print(f"  KD_RATIO={KD_RATIO} is a load-bearing tuning assumption -- report it as one.\n")


# ── Design selection ──────────────────────────────────────────────────────────
def pick_designs(pop: pd.DataFrame, n: int, seed: int = 4242) -> pd.DataFrame:
    """Stratify by the risk grouping so the sweep spans the range it is meant to test."""
    if n <= 0 or n >= len(pop):
        return pop
    tau = pop["latency_steps"] / 200.0
    keff = 14.4 * pop["motor_scale"] * 0.25 / pop["Iyy"]
    pop = pop.assign(_risk=np.log(keff * tau ** 2))
    q = pd.qcut(pop["_risk"], 10, labels=False, duplicates="drop")
    per = max(1, n // (q.max() + 1))
    rng = np.random.default_rng(seed)
    return (pop.groupby(q, group_keys=False)
               .apply(lambda g: g.sample(min(per, len(g)), random_state=rng.integers(1e9)))
               .drop(columns="_risk").reset_index(drop=True))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=300, help="designs to sample (0 = all)")
    ap.add_argument("--headline", action="store_true", help="canonical cell only")
    ap.add_argument("--designs", type=Path, default=DESIGNS_CSV)
    ap.add_argument("--jobs", type=int, default=-1)
    args = ap.parse_args()

    if not args.designs.exists():
        sys.exit(RESTORE_HINT)
    pop = pd.read_csv(args.designs)
    designs = pick_designs(pop, args.n)
    cells = [CANONICAL] if args.headline else [(ic, km) for ic in IC_DEGS for km in KP_MAXES]

    print(f"=== S2R initial-condition / grid-ceiling sweep ===")
    print(f"  designs={len(designs)}  cells={len(cells)}  eval seeds={len(EVAL_SEEDS)}")
    print(f"  ~{len(designs) * len(cells) * (N_COARSE + N_REFINE) * len(TUNE_SEEDS) + len(designs) * len(cells) * 2 * len(EVAL_SEEDS):,} sims")
    damping_check(designs.iloc[0].to_dict())

    rows = designs.to_dict("records")
    jobs = [delayed(eval_cell)(r, ic, km) for ic, km in cells for r in rows]
    out = pd.DataFrame(Parallel(n_jobs=args.jobs, verbose=5)(jobs))
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved {len(out)} rows -> {OUT_CSV}")

    print("\n=== fail rate by cell (the whole point) ===")
    piv = out.pivot_table(index="ic_deg", columns="kp_max",
                          values=["fail_infeasible", "fail_window", "overtuned", "at_grid_top"])
    print(piv.round(3).to_string())
    print("\n  fail_infeasible : SR < 0.35 (what the published 9.1%/61.3% numbers used)")
    print("  fail_window     : SR < 0.80 (the paper's usable-window standard)")
    print("  at_grid_top     : fraction where the still-air search saturated kp_max.")
    print("                    If this is high and moves with kp_max, the effect size is")
    print("                    partly the grid ceiling and the claim must say so.")


if __name__ == "__main__":
    main()
