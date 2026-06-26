"""
run_gain_window_sweep.py  (Q-A)
For ~50 designs spanning the authority/Iyy ratio range, sweep Kp over a fine grid
and measure the SR-vs-Kp curve shape.

Goal: measure gain window width (range of Kp where SR >= 0.5) for each design
and check whether window width scales inversely with authority/Iyy ratio.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sim"))

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from fidelity_config import FidelityConfig, apply_fidelity_config
from controller import PIDParams
from experiment_runner import _run_one

OUT_CSV = "experiments/results/gain_window_sweep_py.csv"
N_EVAL_SEEDS = 5
KP_GRID = [1, 2, 3, 5, 8, 12, 20, 30, 40, 60, 80, 120, 160, 240, 320]
KD_FIXED_FACTOR = 0.25   # Kd = best_Kd from Exp1 (use each design's tuned Kd)
N_DESIGNS = 50
N_JOBS = -1


def _sweep_one(row: dict, Kp: float) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    cfg = FidelityConfig.full()
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, cfg)

    # Use design's optimal Kd from Exp1 (or fallback to Kd=8)
    Kd = float(row.get("best_Kd", 8.0))
    pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)

    results = [_run_one(pid, plant, act, sen, dis, sc, seed=s)
               for s in range(1, N_EVAL_SEEDS + 1)]
    sr  = sum(float(r["success"]) for r in results) / N_EVAL_SEEDS
    rms = sum(float(r["rms_error_deg"]) for r in results) / N_EVAL_SEEDS
    pk  = sum(float(r["peak_error_deg"]) for r in results) / N_EVAL_SEEDS

    return {
        "rocket_id": row["rocket_id"],
        "regime_label": row["regime_label"],
        "Kp_sweep": Kp,
        "Kd": Kd,
        "best_Kp_exp1": row["best_Kp"],
        "sr": sr,
        "rms_deg": rms,
        "peak_deg": pk,
        "Iyy": row["Iyy"],
        "authority_inertia_ratio": (row["max_gimbal_deg"] * row["motor_scale"]) / row["Iyy"],
        "static_margin": row["static_margin"],
        "motor_scale": row["motor_scale"],
        "max_gimbal_deg": row["max_gimbal_deg"],
        "wind_strength": row["wind_strength"],
        "servo_slew_deg_s": row["servo_slew_deg_s"],
    }


def select_designs(exp1: pd.DataFrame, n: int = 50) -> pd.DataFrame:
    """Stratified sample across authority/Iyy ratio range."""
    exp1 = exp1.copy()
    exp1["ratio"] = (exp1.max_gimbal_deg * exp1.motor_scale) / exp1.Iyy

    # Always include all FRAGILE designs
    fragile = exp1[exp1.regime_label == "FRAGILE"]

    # Sample EASY designs across ratio quantiles (uniform coverage)
    easy = exp1[exp1.regime_label == "EASY"].copy()
    easy_sorted = easy.sort_values("ratio")
    n_easy = n - len(fragile)
    indices = np.linspace(0, len(easy_sorted) - 1, n_easy, dtype=int)
    easy_sample = easy_sorted.iloc[indices]

    return pd.concat([fragile, easy_sample]).reset_index(drop=True)


if __name__ == "__main__":
    print("Q-A: Gain Window Width vs Authority/Iyy Ratio")
    exp1 = pd.read_csv("experiments/results/exp1_regime_index_py.csv")
    designs = select_designs(exp1, n=N_DESIGNS)
    print(f"  Selected {len(designs)} designs: "
          f"{(designs.regime_label == 'FRAGILE').sum()} FRAGILE, "
          f"{(designs.regime_label == 'EASY').sum()} EASY")
    ratio_range = designs["ratio"] if "ratio" in designs.columns else \
        (designs.max_gimbal_deg * designs.motor_scale) / designs.Iyy
    print(f"  Authority/Iyy ratio range: [{ratio_range.min():.0f}, {ratio_range.max():.0f}]")

    rows = designs.to_dict("records")
    tasks = [(row, Kp) for row in rows for Kp in KP_GRID]
    n_total = len(tasks) * N_EVAL_SEEDS
    print(f"  Tasks: {len(tasks)} ({N_DESIGNS} designs x {len(KP_GRID)} Kp values)")
    print(f"  Sims:  {n_total:,} ({N_EVAL_SEEDS} seeds each)")

    results = Parallel(n_jobs=N_JOBS, verbose=5)(
        delayed(_sweep_one)(row, Kp) for row, Kp in tasks
    )

    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved: {OUT_CSV}  ({len(out)} rows)")
    print(f"  Columns: {list(out.columns)}")

    # Quick summary
    print("\nQuick window analysis:")
    for regime in ["EASY", "FRAGILE"]:
        sub = out[out.regime_label == regime]
        if len(sub) == 0: continue
        # For each design, find Kp window where SR >= 0.5
        windows = []
        for rid, g in sub.groupby("rocket_id"):
            g = g.sort_values("Kp_sweep")
            passing = g[g.sr >= 0.5]
            if len(passing) >= 2:
                windows.append(passing.Kp_sweep.max() / passing.Kp_sweep.min())
        if windows:
            print(f"  {regime}: median Kp window ratio={np.median(windows):.1f}x, "
                  f"n={len(windows)}")
