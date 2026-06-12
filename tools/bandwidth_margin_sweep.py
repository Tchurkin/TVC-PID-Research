"""
bandwidth_margin_sweep.py

Test whether slight aerodynamic instability provides a maneuverability
sweet spot, and at what servo speed instability becomes beneficial.

Design:
  - REF rocket with static_margin swept from -0.30 (stable) to +0.30 (unstable)
  - Servo slew swept: [60, 120, 200, 400, 800] deg/s (hobby + hypothetical)
  - Task: 10-degree step pitch maneuver (theta_ref = 10 deg)
  - Fidelity: wind + slew + full physics aero; no fault/noise (isolates bandwidth)
  - Autotune per condition (5x5 Kp/Kd grid, 2 seeds), evaluate 5 seeds
  - Also runs theta_ref=0 (attitude hold) as baseline comparison

Null hypothesis: SR monotonically increases toward stable for all servo speeds.
Alternative: local maximum at slight instability (sweet spot) emerges at high slew.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))

import numpy as np
import pandas as pd

from design_space import (
    REF, build_plant, build_actuator, build_sensor, build_disturbance,
    estimate_lambda_aero,
)
from simulator import simulate, ScenarioParams
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config

STATIC_MARGINS = [-0.30, -0.20, -0.10, -0.05, 0.0, 0.05, 0.10, 0.20, 0.30]
SLEW_RATES     = [60, 120, 200, 400, 800]   # deg/s

KP_GRID = [2, 5, 15, 40, 80]
KD_GRID = [1, 2, 8, 16, 32]
TUNE_SEEDS = [1, 2]
EVAL_SEEDS = [1, 2, 3, 4, 5]
MANEUVER_DEGS = [0.0, 10.0]   # 0 = attitude hold baseline, 10 = step maneuver

# Wind + slew + full physical aero; no fault, no gyro noise (isolates bandwidth margin)
FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, sensor_noise=False, latency=False, thrust_var=False,
)


def make_design(static_margin: float, slew: float) -> dict:
    d = dict(REF)
    d["static_margin"] = static_margin
    d["servo_slew_deg_s"] = slew
    d["Cm_alpha"] = -52.0
    d["lambda_aero"] = estimate_lambda_aero(static_margin, -52.0, REF["motor_scale"])
    return d


def run_single(design: dict, Kp: float, Kd: float, theta_ref_deg: float, seed: int):
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = ScenarioParams(t_end=3.0, theta_ref=np.deg2rad(theta_ref_deg))
    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    return simulate(pid, plant, act2, sen2, dis2, sc2, seed=seed)


def autotune(design: dict, theta_ref_deg: float) -> tuple[float, float]:
    best = (KP_GRID[0], KD_GRID[0])
    best_sr  = -1.0
    best_rms = 1e9
    for Kp in KP_GRID:
        for Kd in KD_GRID:
            results = [run_single(design, Kp, Kd, theta_ref_deg, s) for s in TUNE_SEEDS]
            sr  = sum(r.success for r in results) / len(results)
            rms = sum(r.rms_error_deg for r in results) / len(results)
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr, best_rms = sr, rms
                best = (Kp, Kd)
    return best


rows = []
total = len(STATIC_MARGINS) * len(SLEW_RATES) * len(MANEUVER_DEGS)
done  = 0

for task_deg in MANEUVER_DEGS:
    task_label = "hold" if task_deg == 0.0 else f"maneuver_{int(task_deg)}deg"
    for slew in SLEW_RATES:
        for sm in STATIC_MARGINS:
            done += 1
            design = make_design(sm, slew)
            Kp, Kd = autotune(design, task_deg)
            evals  = [run_single(design, Kp, Kd, task_deg, s) for s in EVAL_SEEDS]
            sr       = sum(r.success for r in evals) / len(evals)
            rms      = sum(r.rms_error_deg for r in evals) / len(evals)
            settling = sum(r.settling_time_s for r in evals) / len(evals)
            max_th   = sum(r.max_theta_deg for r in evals) / len(evals)
            print(
                f"[{done:3d}/{total}] task={task_label} slew={slew:3d} "
                f"sm={sm:+.2f} Kp={Kp:2d} Kd={Kd:2d} "
                f"SR={sr:.2f} RMS={rms:.1f}deg settle={settling:.2f}s"
            )
            rows.append({
                "task": task_label,
                "static_margin": sm,
                "servo_slew": slew,
                "best_Kp": Kp,
                "best_Kd": Kd,
                "success_rate": sr,
                "rms_deg": rms,
                "settling_s": settling,
                "mean_max_theta": max_th,
            })

df  = pd.DataFrame(rows)
out = os.path.join(
    os.path.dirname(__file__), "..", "experiments", "results", "bandwidth_margin_sweep_py.csv"
)
df.to_csv(out, index=False)
print(f"\nSaved {len(df)} rows -> {out}")
