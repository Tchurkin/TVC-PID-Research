"""
kp_window_sweep.py — Fine-grid Kp sweep for 5 representative designs.

Measures kp_floor and kp_ceiling empirically across:
  - 2 EASY designs (low and mid theta_ddot)
  - 3 FRAGILE designs (low, mid, high theta_ddot)

Sweeps Kp from 1 to 320 (log-spaced, 24 values), 5 seeds each.
Extracts floor (SR first reaches 0.7) and ceiling (SR drops below 0.7 after peak).

Output: experiments/results/kp_window_sweep_py.csv
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

import numpy as np
import pandas as pd

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from simulator import simulate, ScenarioParams
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N

L_NOZZLE = 0.25
T_END    = 3.0
N_SEEDS  = 5
KP_GRID  = np.unique(np.round(np.geomspace(1, 320, 24)).astype(int)).tolist()
KD_FIXED = 8.0   # fixed Kd, middle of exp1 grid

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)

df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                              'exp1_regime_index_py.csv'))
df['theta_ddot'] = (F15_AVG_THRUST_N
                    * np.sin(np.deg2rad(df['max_gimbal_deg']))
                    * df['motor_scale'] * L_NOZZLE / df['Iyy'])

easy = df[df.regime_label == 'EASY'].sort_values('theta_ddot')
frag = df[df.regime_label == 'FRAGILE'].sort_values('theta_ddot')

# Pick representative designs
targets = {
    'EASY_low':  easy.iloc[len(easy)//8],
    'EASY_mid':  easy.iloc[len(easy)//2],
    'FRAG_lo':   frag.iloc[frag.theta_ddot.between(60,90).values.argmax()],
    'FRAG_mid':  frag.iloc[frag.theta_ddot.between(110,150).values.argmax()],
    'FRAG_hi':   frag.iloc[-1],
}

# Fallback: pick by position if between() finds nothing
for key in list(targets.keys()):
    if hasattr(targets[key], 'empty') and targets[key].empty:
        if 'FRAG_lo' in key:
            targets[key] = frag.iloc[len(frag)//4]
        elif 'FRAG_mid' in key:
            targets[key] = frag.iloc[len(frag)//2]

print(f"Kp sweep: {len(KP_GRID)} values × {N_SEEDS} seeds × 5 designs")
print(f"Kp grid: {KP_GRID}")
print()
for label, row in targets.items():
    print(f"  {label:10s}: {row.rocket_id}  theta_ddot={row.theta_ddot:.1f}  "
          f"Iyy={row.Iyy:.4f}  best_Kp={row.best_Kp}")
print()

seeds = list(range(1, N_SEEDS + 1))
rows  = []

for label, row in targets.items():
    d     = row.to_dict()
    rid   = d['rocket_id']
    plant = build_plant(d)
    act   = build_actuator(d)
    sen   = build_sensor(d)
    dis   = build_disturbance(d)
    u_max = act.u_max

    print(f"\n--- {label}: {rid}  theta_ddot={d['theta_ddot']:.1f} ---")
    srs, rmss = [], []

    for Kp in KP_GRID:
        sc  = ScenarioParams(t_end=T_END)
        pid = PIDParams(Kp=float(Kp), Kd=KD_FIXED, Ki=0.0, u_max=u_max)
        act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
        results = [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in seeds]
        sr  = float(np.mean([r.success for r in results]))
        rms = float(np.mean([r.rms_error_deg for r in results]))
        srs.append(sr)
        rmss.append(rms)

        rows.append({
            'label':      label,
            'rocket_id':  rid,
            'regime':     d['regime_label'],
            'theta_ddot': d['theta_ddot'],
            'Iyy':        d['Iyy'],
            'Kp':         Kp,
            'sr':         sr,
            'rms_deg':    rms,
        })
        print(f"  Kp={Kp:4.0f}  SR={sr:.2f}  RMS={rms:.1f}°", flush=True)

    # Find floor and ceiling
    srs_arr = np.array(srs)
    kps_arr = np.array(KP_GRID)
    peak_idx = np.argmax(srs_arr)
    peak_sr  = srs_arr[peak_idx]
    peak_kp  = kps_arr[peak_idx]

    # floor = first Kp where SR >= 0.7
    floor_mask = srs_arr >= 0.70
    kp_floor = kps_arr[floor_mask][0] if floor_mask.any() else float('nan')

    # ceiling = last Kp where SR >= 0.7 (before permanent drop)
    kp_ceiling = kps_arr[floor_mask][-1] if floor_mask.any() else float('nan')

    window = kp_ceiling / kp_floor if kp_floor > 0 else float('nan')
    print(f"  SUMMARY: floor={kp_floor}  ceiling={kp_ceiling}  "
          f"window={window:.1f}x  peak_SR={peak_sr:.2f} at Kp={peak_kp}")

out = pd.DataFrame(rows)
out_path = os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                        'kp_window_sweep_py.csv')
out.to_csv(out_path, index=False)

print()
print("=== GAIN WINDOW SUMMARY ===")
print(f"{'Design':12s}  {'regime':8s}  {'td':>7}  {'floor':>7}  {'ceiling':>9}  {'window':>8}")
print("-"*60)
for label, row in targets.items():
    sub = out[out.label == label]
    if len(sub) == 0:
        continue
    srs_arr = sub.sr.values
    kps_arr = sub.Kp.values
    mask = srs_arr >= 0.70
    kp_floor   = kps_arr[mask][0]   if mask.any() else float('nan')
    kp_ceiling = kps_arr[mask][-1]  if mask.any() else float('nan')
    window = kp_ceiling / kp_floor if kp_floor > 0 else float('nan')
    td = row.theta_ddot
    print(f"{label:12s}  {row.regime_label:8s}  {td:7.1f}  {kp_floor:7.0f}  "
          f"{kp_ceiling:9.0f}  {window:8.1f}x")

print(f"\nSaved to {out_path}")
