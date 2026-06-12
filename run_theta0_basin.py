"""
run_theta0_basin.py  (Q-C)
For 30 designs spanning authority/Iyy ratio range, sweep initial tilt angle (theta0)
and find the maximum tilt each design can recover from.

Goal: Does authority/Iyy ratio predict recovery basin (max survivable launch tilt)?
Prediction: high-ratio (over-actuated) designs have smaller recovery basins.

Hardware connection: launch pad alignment requirements — directly testable in flight.
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

OUT_CSV = "experiments/results/theta0_basin_py.csv"
N_EVAL_SEEDS = 5
# Theta0 sweep: 0 to 25 degrees initial tilt
THETA0_GRID = [0.0, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 13.0, 17.0, 21.0, 25.0, 30.0]
N_DESIGNS = 35
N_JOBS = -1


def _eval_theta0(row: dict, theta0_deg: float) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)

    # Use theta0_fixed_deg to set a deterministic initial tilt
    # Plus a small theta0_bias_std to add natural seed variation
    sc = build_scenario(theta0_bias_std=0.0)
    sc.theta0_fixed_deg = theta0_deg

    cfg = FidelityConfig.full()
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, cfg)

    # Use optimal gains from Exp1
    Kp = float(row["best_Kp"])
    Kd = float(row["best_Kd"])
    pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)

    results = [_run_one(pid, plant, act, sen, dis, sc, seed=s)
               for s in range(1, N_EVAL_SEEDS + 1)]
    sr   = sum(float(r["success"]) for r in results) / N_EVAL_SEEDS
    rms  = sum(float(r["rms_error_deg"]) for r in results) / N_EVAL_SEEDS
    peak = sum(float(r["peak_error_deg"]) for r in results) / N_EVAL_SEEDS
    diverged = sum(1 for r in results if float(r["peak_error_deg"]) > 45) / N_EVAL_SEEDS

    return {
        "rocket_id": row["rocket_id"],
        "regime_label": row["regime_label"],
        "theta0_deg": theta0_deg,
        "sr": sr,
        "rms_deg": rms,
        "peak_deg": peak,
        "diverge_rate": diverged,
        "best_Kp": Kp,
        "best_Kd": Kd,
        "Iyy": row["Iyy"],
        "authority_inertia_ratio": (row["max_gimbal_deg"] * row["motor_scale"]) / row["Iyy"],
        "static_margin": row["static_margin"],
        "motor_scale": row["motor_scale"],
        "max_gimbal_deg": row["max_gimbal_deg"],
        "wind_strength": row["wind_strength"],
        "servo_slew_deg_s": row["servo_slew_deg_s"],
        "p_unstable": row["p_unstable"],
    }


def select_designs(exp1: pd.DataFrame, n: int = 35) -> pd.DataFrame:
    """Stratified sample across authority/Iyy ratio range, include all FRAGILE."""
    exp1 = exp1.copy()
    exp1["ratio"] = (exp1.max_gimbal_deg * exp1.motor_scale) / exp1.Iyy

    fragile = exp1[exp1.regime_label == "FRAGILE"]
    easy = exp1[exp1.regime_label == "EASY"].copy().sort_values("ratio")
    n_easy = n - len(fragile)
    indices = np.linspace(0, len(easy) - 1, n_easy, dtype=int)
    easy_sample = easy.iloc[indices]

    return pd.concat([fragile, easy_sample]).reset_index(drop=True)


if __name__ == "__main__":
    print("Q-C: Basin of Attraction (theta0 sweep) vs Authority/Iyy Ratio")
    exp1 = pd.read_csv("experiments/results/exp1_regime_index_py.csv")
    designs = select_designs(exp1, n=N_DESIGNS)
    ratio_col = (designs.max_gimbal_deg * designs.motor_scale) / designs.Iyy
    print(f"  Selected {len(designs)} designs: "
          f"{(designs.regime_label == 'FRAGILE').sum()} FRAGILE, "
          f"{(designs.regime_label == 'EASY').sum()} EASY")
    print(f"  Authority/Iyy ratio range: [{ratio_col.min():.0f}, {ratio_col.max():.0f}]")

    rows = designs.to_dict("records")
    tasks = [(row, t0) for row in rows for t0 in THETA0_GRID]
    n_total = len(tasks) * N_EVAL_SEEDS
    print(f"  Tasks: {len(tasks)} ({N_DESIGNS} designs x {len(THETA0_GRID)} theta0 values)")
    print(f"  Sims:  {n_total:,} ({N_EVAL_SEEDS} seeds each)")

    results = Parallel(n_jobs=N_JOBS, verbose=5)(
        delayed(_eval_theta0)(row, t0) for row, t0 in tasks
    )

    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved: {OUT_CSV}  ({len(out)} rows)")

    # Quick summary: critical theta0 per design
    print("\nQuick recovery basin summary:")
    for regime in ["EASY", "FRAGILE"]:
        sub = out[out.regime_label == regime]
        if len(sub) == 0: continue
        critical_angles = []
        for rid, g in sub.groupby("rocket_id"):
            g = g.sort_values("theta0_deg")
            # Critical angle = last theta0 where SR >= 0.6
            passing = g[g.sr >= 0.6]
            if len(passing) > 0:
                critical_angles.append(passing.theta0_deg.max())
            else:
                critical_angles.append(0.0)
        print(f"  {regime}: median critical theta0 = {np.median(critical_angles):.1f} deg, "
              f"range = [{min(critical_angles):.0f}, {max(critical_angles):.0f}] deg, "
              f"n={len(critical_angles)}")
