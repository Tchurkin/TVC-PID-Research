"""
wind_threshold_sweep.py

Does required Kp show threshold behavior with wind_strength?
Fix REF design, sweep wind_strength from 0 to max, find required Kp at each level.

Hypothesis: There is a sharp threshold — any nonzero wind requires Kp >> 2.
Alternative: Required Kp grows continuously with wind (formula exists).
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))

import numpy as np
import pandas as pd

from design_space import REF, build_plant, build_actuator, build_sensor, build_disturbance
from simulator import simulate, ScenarioParams
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config

WIND_LEVELS = np.concatenate([[0.0], np.logspace(np.log10(0.02), np.log10(0.45), 16)])
KP_GRID = [1, 2, 5, 10, 20, 40, 80, 160]
KD = 8   # fixed mid-range Kd
EVAL_SEEDS = list(range(1, 9))   # 8 seeds for stable SR estimate
SUCCESS_SR_THRESH = 0.75  # SR needed to count as "sufficient"

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, sensor_noise=False, latency=False, thrust_var=False,
)


def run_one(wind: float, Kp: float, seed: int):
    design = dict(REF)
    design["wind_strength"] = wind
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = ScenarioParams(t_end=3.0, theta_ref=0.0)
    pid   = PIDParams(Kp=Kp, Kd=KD, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    r = simulate(pid, plant, act2, sen2, dis2, sc2, seed=seed)
    return r.success, r.rms_error_deg, r.oscillation_score, r.slew_sat_frac


rows = []
total = len(WIND_LEVELS) * len(KP_GRID)
done = 0

for wind in WIND_LEVELS:
    for Kp in KP_GRID:
        done += 1
        results = [run_one(wind, Kp, s) for s in EVAL_SEEDS]
        sr   = sum(r[0] for r in results) / len(results)
        rms  = sum(r[1] for r in results) / len(results)
        osc  = sum(r[2] for r in results) / len(results)
        slew = sum(r[3] for r in results) / len(results)
        rows.append({'wind_strength': round(wind, 4), 'Kp': Kp,
                     'sr': sr, 'rms_deg': rms, 'osc_score': osc, 'slew_sat': slew})

    # Find kp_needed at this wind level
    df_w = pd.DataFrame([r for r in rows if abs(r['wind_strength']-round(wind,4)) < 1e-6])
    kp_needed = None
    for row in df_w.sort_values('Kp').itertuples():
        if row.sr >= SUCCESS_SR_THRESH:
            kp_needed = row.Kp
            break
    print(f"wind={wind:.3f}  kp_needed={kp_needed}  "
          + "  ".join(f"Kp{row.Kp}={row.sr:.2f}" for row in df_w.sort_values('Kp').itertuples()))

df = pd.DataFrame(rows)
out = os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "wind_threshold_sweep_py.csv")
df.to_csv(out, index=False)
print(f"\nSaved {len(df)} rows -> {out}")
