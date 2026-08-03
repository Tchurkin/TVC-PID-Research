"""
tools/s2r_replication.py  —  does the published sim-to-real result reproduce, and does it survive
                             a single-knob tuner?

WHY THIS EXISTS
---------------
`tools/s2r_ic_sweep.py` re-ran the sim-to-real experiment on the restored simulator and found
NOTHING: 0 of 12 calibration cells showed a significant risk->failure relationship, pooled
rho(log Pi, fail) = -0.030 (p = 0.074, wrong sign), overall failure 9.6% and flat.

That result is NOT yet evidence against the claim, because the sweep has no positive control and
does not reproduce the published numbers either: at the paper's own gate (SR < 0.35) it gives
2-4% where the paper reports 9.1% below Pi = 300 and 61.3% above. So the sweep's protocol differs
from the original in ways that matter, and a null from an unvalidated protocol says nothing.

This is exactly the failure that cost three void runs on the gain-ceiling experiment: interpreting
a striking result from a protocol that had never been shown to reproduce a known one.

DESIGN — two arms, paired on the same designs and seeds
------------------------------------------------------
ARM A (replication).  The original protocol, reproduced from `_eval_design_s2r_gains` in
  sim/experiment_runner.py at bb22d36^:
      tune   : FidelityConfig.simple(), theta0_fixed_deg = 10.0, autotune_continuous
               (Kp log-search [1, 320], Kd probed once at Kp = 40 over {1,4,16,64} then frozen)
      fly    : FidelityConfig.full(), theta0_bias_std = 0.0, seeds (1, 2, 3)
      gate   : FRAGILE_SUCCESS_RATE = 0.35   (sim/units.py -- the gate the published 9.1%/61.3%
               numbers actually used; at 3 seeds "SR >= 0.35" means 2 of 3)
  This arm is the POSITIVE CONTROL. It must reproduce the published failure-rate-versus-Pi curve
  before anything else in this file is interpretable.

ARM B (single-knob).  The variant from s2r_ic_sweep: Kd tied to Kp by a fixed ratio, evaluated on
  15 fresh seeds. Same designs, same tuning initial condition, so the ONLY differences from Arm A
  are the tuner's gain parameterisation and the seed count.

READING THE OUTCOME
  control fails            -> stop; the replication itself is not faithful, report nothing.
  A reproduces, B does not -> the published effect is contingent on the DECOUPLED-Kd tuner, not on
                              the rocket. Claim 3 becomes a statement about tuning method.
  both reproduce           -> the s2r_ic_sweep null was a protocol artifact; Claim 3 stands.

Pi is computed exactly as tools/pi_s2r_gap_analysis.py did, in native units:
    keff = F15_AVG * motor_scale * CU_TO_RAD * L_NOZZLE / Iyy ,  Pi = keff * latency_steps^2

SEEDS: Arm A uses (1,2,3) deliberately -- reproducing the original requires its seeds.
       Arm B uses 340001-340015, disjoint from all prior experiments.
OUTPUT: experiments/results/s2r_replication_py.csv   (one row per design; checkpointed per chunk)

USAGE
  python tools/s2r_replication.py --n 200      # smoke
  python tools/s2r_replication.py              # full population
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
from experiment_runner import (_run_one, _aggregate, _eval_one_fidelity,
                               autotune_continuous)
from units import FRAGILE_SUCCESS_RATE

F15_AVG   = 14.4
CU_TO_RAD = np.pi / 180 * (15 / 12)
L_NOZZLE  = 0.25

TUNE_IC_DEG = 10.0                      # original used theta0_fixed_deg = 10.0
ARM_A_SEEDS = (1, 2, 3)                 # original eval seeds -- required for replication
ARM_B_SEEDS = tuple(range(340001, 340016))
KD_RATIO    = 0.05                      # Arm B single-knob tie
KP_MIN, KP_MAX = 1.0, 320.0
GATE_WINDOW = 0.80

OUT_CSV     = Path("experiments/results/s2r_replication_py.csv")
DESIGNS_CSV = Path("experiments/results/exp1_final_population_py.csv")

# Published targets (tools/pi_s2r_gap_analysis.py docstring + summary CSV)
PUB_BINS   = [0, 50, 100, 150, 200, 300, 500, 1000]
PUB_FR     = [0.068, 0.093, 0.120, 0.238, 0.313, 0.582, 0.696]
PUB_LO_HI  = (0.091, 0.613)             # Pi < 300 vs Pi >= 300


def _tune_single_knob(plant, act, sen, dis, sc) -> tuple[float, float]:
    """Arm B: sweep Kp with Kd = KD_RATIO * Kp. Same objective rule as autotune_continuous."""
    def score(kp):
        pid = PIDParams(Kp=float(kp), Kd=float(KD_RATIO * kp), Ki=0.0,
                        u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in (1, 2)]
        return (float(np.mean([r["success"] for r in runs])),
                float(np.mean([r["rms_error_deg"] for r in runs])))
    best, best_kp = (-1.0, float("inf")), KP_MIN
    for kp in np.geomspace(KP_MIN, KP_MAX, 10):
        s = score(kp)
        if s[0] > best[0] or (s[0] == best[0] and s[1] < best[1]):
            best, best_kp = s, float(kp)
    for kp in np.geomspace(best_kp / 1.5, min(best_kp * 1.5, KP_MAX), 6):
        s = score(kp)
        if s[0] > best[0] or (s[0] == best[0] and s[1] < best[1]):
            best, best_kp = s, float(kp)
    return best_kp, KD_RATIO * best_kp


def eval_design(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)

    sc_tune = replace(build_scenario(), theta0_fixed_deg=TUNE_IC_DEG)
    a_s, s_s, d_s, sc_s = apply_fidelity_config(act, sen, dis, sc_tune,
                                                FidelityConfig.simple())
    sc_full  = build_scenario(theta0_bias_std=0.0)
    full_cfg = FidelityConfig.full()

    # --- ARM A: original protocol ---
    kp_a, kd_a = autotune_continuous(plant, a_s, s_s, d_s, sc_s,
                                     kp_min=KP_MIN, kp_max=KP_MAX)
    pid_a = PIDParams(Kp=kp_a, Kd=kd_a, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    m_a = _eval_one_fidelity(plant, act, sen, dis, sc_full, pid_a, full_cfg, ARM_A_SEEDS)
    sr_a = m_a["success_rate"]

    # --- ARM B: single-knob, more seeds ---
    kp_b, kd_b = _tune_single_knob(plant, a_s, s_s, d_s, sc_s)
    pid_b = PIDParams(Kp=kp_b, Kd=kd_b, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    m_b = _aggregate([_run_one(pid_b, plant, act, sen, dis, sc_full, seed=s)
                      for s in ARM_B_SEEDS])
    sr_b = m_b["success_rate"]

    keff = F15_AVG * float(row["motor_scale"]) * CU_TO_RAD * L_NOZZLE / float(row["Iyy"])
    lat  = float(row["latency_steps"])
    return dict(
        rocket_id=row["rocket_id"], Iyy=row["Iyy"], motor_scale=row["motor_scale"],
        latency_steps=lat, wind_strength=row.get("wind_strength"),
        keff_native=keff, Pi=keff * lat ** 2,
        kp_a=kp_a, kd_a=kd_a, sr_a=sr_a,
        kp_b=kp_b, kd_b=kd_b, sr_b=sr_b,
        fail_a_gate035=int(sr_a < FRAGILE_SUCCESS_RATE),
        fail_a_gate080=int(sr_a < GATE_WINDOW),
        fail_b_gate035=int(sr_b < FRAGILE_SUCCESS_RATE),
        fail_b_gate080=int(sr_b < GATE_WINDOW),
    )


def report(d: pd.DataFrame) -> None:
    print("\n=== POSITIVE CONTROL: does ARM A reproduce the published Pi curve? ===")
    print("  Pi bin        n     ARM A FR    published    ARM B FR")
    ok_bins = 0
    for i in range(len(PUB_BINS) - 1):
        lo, hi = PUB_BINS[i], PUB_BINS[i + 1]
        g = d[(d.Pi >= lo) & (d.Pi < hi)]
        if len(g) < 10:
            continue
        a, b, p = g.fail_a_gate035.mean(), g.fail_b_gate035.mean(), PUB_FR[i]
        ok = abs(a - p) <= max(0.10, 0.5 * p)
        ok_bins += ok
        print(f"  {lo:5d}-{hi:<5d} {len(g):5d}    {a:6.3f}      {p:6.3f}  {'ok ' if ok else 'OFF'}  {b:6.3f}")
    lo_a = d[d.Pi < 300].fail_a_gate035.mean()
    hi_a = d[d.Pi >= 300].fail_a_gate035.mean()
    lo_b = d[d.Pi < 300].fail_b_gate035.mean()
    hi_b = d[d.Pi >= 300].fail_b_gate035.mean()
    print(f"\n  headline split (gate 0.35):")
    print(f"    ARM A   Pi<300 {lo_a:.3f}  Pi>=300 {hi_a:.3f}   ratio {hi_a/max(lo_a,1e-9):.1f}x")
    print(f"    published        {PUB_LO_HI[0]:.3f}           {PUB_LO_HI[1]:.3f}   ratio {PUB_LO_HI[1]/PUB_LO_HI[0]:.1f}x")
    print(f"    ARM B   Pi<300 {lo_b:.3f}  Pi>=300 {hi_b:.3f}   ratio {hi_b/max(lo_b,1e-9):.1f}x")

    control = abs(lo_a - PUB_LO_HI[0]) <= 0.08 and hi_a >= 0.35
    print(f"\n  CONTROL {'PASSED' if control else 'FAILED'}"
          f"  (needs Pi<300 within 0.08 of {PUB_LO_HI[0]:.3f} and Pi>=300 at least 0.35)")
    if not control:
        print("  *** Arm A does not reproduce the published result. Everything below is")
        print("  *** uninterpretable -- the replication itself is not faithful.")
        return

    from scipy import stats
    print("\n=== if the control passed: does the effect survive the single-knob tuner? ===")
    for arm, col in [("A (original, decoupled Kd)", "fail_a_gate035"),
                     ("B (single-knob Kd=0.05Kp)", "fail_b_gate035")]:
        r, p = stats.spearmanr(np.log(d.Pi), d[col])
        print(f"  rho(log Pi, {arm:<28}) = {r:+.3f}  (p={p:.2g})")
    print("\n  A reproduces and B does not -> the published effect is a property of the")
    print("  decoupled-Kd tuner, not of the rocket. Both reproduce -> Claim 3 stands.")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=0, help="0 = full population")
    ap.add_argument("--designs", type=Path, default=DESIGNS_CSV)
    ap.add_argument("--jobs", type=int, default=-1)
    args = ap.parse_args()
    if not args.designs.exists():
        sys.exit(f"{args.designs} missing — restore with:\n"
                 f"  git archive bb22d36^ experiments/results | tar -x -C .")

    pop = pd.read_csv(args.designs)
    if args.n:
        pop = pop.sample(min(args.n, len(pop)), random_state=31).reset_index(drop=True)

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    done: set = set()
    if OUT_CSV.exists():
        try:
            done = set(pd.read_csv(OUT_CSV).rocket_id)
            print(f"  resuming: {len(done)} already complete")
        except Exception:
            done = set()
    rows = [r for r in pop.to_dict("records") if r["rocket_id"] not in done]
    print(f"=== S2R replication: Arm A (original) vs Arm B (single-knob) ===")
    print(f"  {len(rows)} designs to run of {len(pop)}")

    CHUNK = 200
    for i in range(0, len(rows), CHUNK):
        res = pd.DataFrame(Parallel(n_jobs=args.jobs, verbose=0)(
            delayed(eval_design)(r) for r in rows[i:i + CHUNK]))
        res.to_csv(OUT_CSV, mode="a", header=not OUT_CSV.exists(), index=False)
        print(f"  chunk {i//CHUNK + 1}/{-(-len(rows)//CHUNK)} written", flush=True)

    d = pd.read_csv(OUT_CSV).drop_duplicates("rocket_id")
    print(f"\nSaved {len(d)} unique designs -> {OUT_CSV}")
    report(d)


if __name__ == "__main__":
    main()
