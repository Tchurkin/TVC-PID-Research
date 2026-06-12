"""
kp_window_sweep_v2.py — Clean gain-window measurement across theta_ddot range.

Fixes from v1:
  - Uses each design's best_Kd (from exp1) instead of a fixed Kd=8.
  - Expands to 12 designs evenly spanning theta_ddot (not just 5).
  - Uses 7 seeds per Kp value for cleaner SR estimates.

Purpose: establish window ∝ theta_ddot^B as a governing relationship.
The window table (320x → 2x) is the main publishable claim from this analysis.

Output: experiments/results/kp_window_sweep_v2_py.csv
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
N_SEEDS  = 7
KP_GRID  = sorted(set(map(int, np.round(np.geomspace(1, 320, 26)))))
SR_GATE  = 0.70   # floor = first Kp with SR >= gate; ceiling = last

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

# ── Select 12 designs spanning theta_ddot ─────────────────────────────────────
# Strategy: 4 EASY (log-spaced) + 4 FRAGILE (log-spaced) + 4 boundary EASY (td 50-100)
easy = df[df.regime_label == 'EASY'].sort_values('theta_ddot')
frag = df[df.regime_label == 'FRAGILE'].sort_values('theta_ddot')
boundary_easy = easy[easy.theta_ddot.between(50, 100)]

easy_pick = easy.iloc[np.linspace(0, len(easy)-1, 4, dtype=int)]
frag_pick = frag.iloc[np.linspace(0, len(frag)-1, 4, dtype=int)]

# 4 boundary EASY (high authority EASY designs that didn't become FRAGILE)
if len(boundary_easy) >= 4:
    be_pick = boundary_easy.iloc[np.linspace(0, len(boundary_easy)-1, 4, dtype=int)]
else:
    be_pick = boundary_easy

designs = pd.concat([easy_pick, be_pick, frag_pick]).drop_duplicates('rocket_id')
designs = designs.sort_values('theta_ddot').reset_index(drop=True)

print(f"Kp window sweep v2: {len(designs)} designs × {len(KP_GRID)} Kp × {N_SEEDS} seeds")
print(f"Using best_Kd from exp1 (per-design). SR gate = {SR_GATE}.")
print(f"Kp grid: {KP_GRID}")
print()
print(f"{'#':>3}  {'rocket_id':>10}  {'regime':>8}  {'td':>7}  {'best_Kp':>8}  {'best_Kd':>8}")
for i, (_, r) in enumerate(designs.iterrows()):
    print(f"{i+1:>3}  {r.rocket_id:>10}  {r.regime_label:>8}  {r.theta_ddot:>7.1f}  "
          f"{r.best_Kp:>8.0f}  {r.best_Kd:>8.0f}")
print()

seeds = list(range(1, N_SEEDS + 1))
rows  = []

for i, (_, row) in enumerate(designs.iterrows()):
    d       = row.to_dict()
    rid     = d['rocket_id']
    Kd_best = float(d['best_Kd'])   # ← use per-design optimal Kd
    td      = d['theta_ddot']

    plant = build_plant(d)
    act   = build_actuator(d)
    sen   = build_sensor(d)
    dis   = build_disturbance(d)
    u_max = act.u_max

    print(f"\n[{i+1}/{len(designs)}] {rid}  regime={d['regime_label']}  "
          f"td={td:.1f}  best_Kd={Kd_best:.0f}")
    srs, rmss = [], []

    for Kp in KP_GRID:
        sc  = ScenarioParams(t_end=T_END)
        pid = PIDParams(Kp=float(Kp), Kd=Kd_best, Ki=0.0, u_max=u_max)
        act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
        results = [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in seeds]
        sr  = float(np.mean([r.success for r in results]))
        rms = float(np.mean([r.rms_error_deg for r in results]))
        slew = float(np.mean([r.slew_sat_frac for r in results]))
        srs.append(sr); rmss.append(rms)

        rows.append({
            'rocket_id':   rid,
            'regime':      d['regime_label'],
            'theta_ddot':  td,
            'Iyy':         d['Iyy'],
            'keff_full':   F15_AVG_THRUST_N * np.pi/180 * 15/12 * d['motor_scale'] * L_NOZZLE / d['Iyy'],
            'best_Kd':     Kd_best,
            'Kp':          Kp,
            'sr':          sr,
            'rms_deg':     rms,
            'slew_sat':    slew,
        })
        print(f"  Kp={Kp:4d}  SR={sr:.2f}  RMS={rms:.1f}°", flush=True)

    # Extract floor/ceiling
    srs_a = np.array(srs)
    kps_a = np.array(KP_GRID)
    mask  = srs_a >= SR_GATE
    fl    = float(kps_a[mask][0])   if mask.any() else np.nan
    ceil  = float(kps_a[mask][-1])  if mask.any() else np.nan
    win   = ceil/fl if fl > 0 else np.nan
    peak_sr = srs_a.max()
    peak_kp = kps_a[srs_a.argmax()]
    print(f"  → floor={fl}  ceiling={ceil}  window={win:.1f}x  peak_SR={peak_sr:.2f} @ Kp={peak_kp}")

out = pd.DataFrame(rows)
out_path = os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                        'kp_window_sweep_v2_py.csv')
out.to_csv(out_path, index=False)

# ── Summary table ─────────────────────────────────────────────────────────────
print()
print("=" * 70)
print("GAIN WINDOW TABLE (governing equation: window ∝ theta_ddot^B)")
print("=" * 70)
print(f"{'Design':>12}  {'regime':>8}  {'td':>7}  {'Kd':>4}  "
      f"{'floor':>7}  {'ceiling':>9}  {'window':>8}  {'peak_SR':>8}")
print("-" * 70)
for rid in out.rocket_id.unique():
    sub  = out[out.rocket_id==rid]
    srs  = sub.sr.values
    kps  = sub.Kp.values
    mask = srs >= SR_GATE
    fl   = float(kps[mask][0])   if mask.any() else np.nan
    ceil = float(kps[mask][-1])  if mask.any() else np.nan
    win  = ceil/fl if fl>0 else np.nan
    td   = sub.theta_ddot.iloc[0]
    reg  = sub.regime.iloc[0]
    kd   = sub.best_Kd.iloc[0]
    psr  = srs.max()
    print(f"{rid:>12}  {reg:>8}  {td:>7.1f}  {kd:>4.0f}  "
          f"{fl:>7.0f}  {ceil:>9.0f}  {win:>8.1f}x  {psr:>8.2f}")

# Regression
ws = []
for rid in out.rocket_id.unique():
    sub = out[out.rocket_id==rid]
    srs, kps = sub.sr.values, sub.Kp.values
    mask = srs >= SR_GATE
    fl   = float(kps[mask][0])   if mask.any() else np.nan
    ceil = float(kps[mask][-1])  if mask.any() else np.nan
    win  = ceil/fl if fl>0 else np.nan
    ws.append({'td': sub.theta_ddot.iloc[0], 'window': win, 'floor': fl,
               'ceiling': ceil, 'regime': sub.regime.iloc[0]})
wdf = pd.DataFrame(ws).dropna()

from scipy.stats import pearsonr
if len(wdf) >= 4:
    B, lA = np.polyfit(np.log(wdf.td), np.log(wdf.window), 1)
    r, p  = pearsonr(np.log(wdf.td), np.log(wdf.window))
    print()
    print(f"Power law fit: window ≈ {np.exp(lA):.0f} × theta_ddot^{B:.2f}  (r={r:.3f})")
    B_f, lAf = np.polyfit(np.log(wdf.td), np.log(wdf.floor), 1)
    print(f"Floor fit:     floor  ≈ {np.exp(lAf):.2f} × theta_ddot^{B_f:.2f}")

print(f"\nSaved to {out_path}")
