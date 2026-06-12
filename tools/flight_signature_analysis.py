"""
flight_signature_analysis.py

Can a single test flight at Kp=2 detect whether a rocket is FRAGILE?

For all 25 FRAGILE + 25 EASY designs (Q-A sweep sample):
  - Run at Kp=2 in full physics (same as S2R study)
  - Extract per-seed flight signatures: oscillation_score, slew_sat_frac, rms_deg
  - Test if single-flight observables discriminate FRAGILE from EASY
  - Compare single-seed vs 3-seed average detection reliability

Key question: Is the bang-bang oscillation signature detectable in 1 flight?
If AUC(oscillation_score) > AUC(rms), oscillation is a better detector.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))

import numpy as np
import pandas as pd
from sklearn.metrics import roc_auc_score

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from simulator import simulate, ScenarioParams
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config

# Full fidelity matching Exp1 evaluation (wind + noise + latency + slew + physical aero)
# No fault (thrust_var) since we're testing nominal behavior
FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)
KP = 2; KD = 1   # simple-model gains (grid minimum, same as S2R study)
N_SEEDS = 7


def run_design(design_row: dict, seed: int):
    plant = build_plant(design_row)
    act   = build_actuator(design_row)
    sen   = build_sensor(design_row)
    dis   = build_disturbance(design_row)
    sc    = ScenarioParams(t_end=3.0, theta_ref=0.0)
    pid   = PIDParams(Kp=KP, Kd=KD, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    r = simulate(pid, plant, act2, sen2, dis2, sc2, seed=seed)
    return {
        "success": r.success, "rms": r.rms_error_deg,
        "osc": r.oscillation_score, "slew_sat": r.slew_sat_frac,
        "peak": r.peak_error_deg, "max_theta": r.max_theta_deg,
    }


# Load designs from Q-A sweep
qa = pd.read_csv(os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "gain_window_sweep_py.csv"))
exp1 = pd.read_csv(os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "exp1_regime_index_py.csv"))

qa_designs = qa[['rocket_id','regime_label']].drop_duplicates().rename(columns={'regime_label':'qa_regime'})
qa_merged = qa_designs.merge(exp1.drop(columns=['regime_label'], errors='ignore'), on='rocket_id')

rows = []
for _, row in qa_merged.iterrows():
    design_row = row.to_dict()
    label = 1 if row['qa_regime'] == 'FRAGILE' else 0
    seeds_data = [run_design(design_row, s) for s in range(1, N_SEEDS+1)]

    for i, sd in enumerate(seeds_data):
        rows.append({
            "rocket_id": row['rocket_id'], "regime": row['qa_regime'],
            "is_fragile": label, "seed": i+1,
            **{k: sd[k] for k in sd},
        })

    mean_rms = np.mean([s['rms'] for s in seeds_data])
    mean_osc = np.mean([s['osc'] for s in seeds_data])
    mean_sr  = np.mean([s['success'] for s in seeds_data])
    print(f"{row['rocket_id']} {row['qa_regime']:8s}  SR={mean_sr:.2f}  RMS={mean_rms:.1f}  osc={mean_osc:.1f}  slew={np.mean([s['slew_sat'] for s in seeds_data]):.2f}")

df = pd.DataFrame(rows)
out = os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "flight_signature_py.csv")
df.to_csv(out, index=False)

# AUC analysis: single-seed vs multi-seed
print("\n=== AUC: single-seed detection (1 test flight) ===")
single = df[df['seed'] == 1].copy()
for metric in ['rms', 'osc', 'slew_sat', 'peak']:
    auc = roc_auc_score(single['is_fragile'], single[metric])
    print(f"  AUC({metric}, 1 seed) = {auc:.3f}")

print("\n=== AUC: 3-seed average detection ===")
agg3 = df[df['seed'] <= 3].groupby(['rocket_id','regime','is_fragile'])[['rms','osc','slew_sat','peak']].mean().reset_index()
for metric in ['rms', 'osc', 'slew_sat', 'peak']:
    auc = roc_auc_score(agg3['is_fragile'], agg3[metric])
    print(f"  AUC({metric}, 3-seed avg) = {auc:.3f}")

print("\n=== AUC: 7-seed average (ground truth-like) ===")
agg7 = df.groupby(['rocket_id','regime','is_fragile'])[['rms','osc','slew_sat','peak']].mean().reset_index()
for metric in ['rms', 'osc', 'slew_sat', 'peak']:
    auc = roc_auc_score(agg7['is_fragile'], agg7[metric])
    print(f"  AUC({metric}, 7-seed avg) = {auc:.3f}")
print(f"\nSaved {len(df)} rows -> {out}")
