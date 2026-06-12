"""
pitch_tracking_pareto.py — Robustness vs Maneuverability Pareto frontier.

For each of 1200 designs evaluate:
  - Attitude-hold robustness   : nom_sr from Exp1 (already computed)
  - Step-tracking agility      : rise_time_s at a 15° step command

Both evaluated with each design's best_Kp from Exp1 (gains tuned for attitude hold,
then applied to the tracking task).  This is the fairest comparison: same controller,
two different performance dimensions.

Also evaluates at Kp=2 to get the "naive deployment" agility baseline.

Output: experiments/results/pitch_tracking_pareto_py.csv
Columns per design:
  rocket_id, regime_label, theta_ddot_max,
  nom_sr (from Exp1),
  rise_time_best, tracking_rms_best, overshoot_best,   <- at best_Kp
  rise_time_kp2,  tracking_rms_kp2,  overshoot_kp2,   <- at Kp=2
  best_Kp, Iyy, motor_scale, max_gimbal_deg, wind_strength
"""

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

import numpy as np
import pandas as pd

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from simulator   import simulate, ScenarioParams
from controller  import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units       import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

# ── Constants ─────────────────────────────────────────────────────────────────
L_NOZZLE   = 0.25        # m, moment arm (assumed fixed as in Exp1)
CU_TO_RAD  = np.pi/180 * REF_MAX_GIMBAL_DEG / REF_U_MAX   # rad/CU

T_END      = 4.0         # s — extra time to see settling
STEP_DEG   = 15.0        # deg — step command magnitude
STEP_TIME  = 1.0         # s   — when the step is commanded
N_SEEDS    = 3           # seeds per condition
SEEDS      = list(range(1, N_SEEDS + 1))

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)

# ── Helper ────────────────────────────────────────────────────────────────────
def run_step(design_dict, Kp, Kd):
    plant = build_plant(design_dict)
    act   = build_actuator(design_dict)
    sen   = build_sensor(design_dict)
    dis   = build_disturbance(design_dict)
    sc    = ScenarioParams(t_end=T_END,
                           theta_step_deg=STEP_DEG,
                           theta_step_time_s=STEP_TIME)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act2.u_max)
    return [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in SEEDS]


def agg(results, attr):
    vals = [getattr(r, attr) for r in results]
    return float(np.mean(vals))


# ── Load Exp1 data ─────────────────────────────────────────────────────────────
df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..',
                              'experiments', 'results', 'exp1_regime_index_py.csv'))

# Compute theta_ddot_max from specs (same formula as CLAUDE.md Finding 2)
df['theta_ddot_max'] = (F15_AVG_THRUST_N
                        * np.sin(np.deg2rad(df['max_gimbal_deg']))
                        * L_NOZZLE
                        * df['motor_scale']
                        / df['Iyy'])

n = len(df)
print(f"Running pitch tracking on {n} designs  ({N_SEEDS} seeds each, 2 gain conditions)")
print(f"Step: {STEP_DEG}° at t={STEP_TIME}s,  t_end={T_END}s")
print(f"Estimated time: ~{n*N_SEEDS*2*0.025:.0f}s  (rough)")
print()

rows = []
t0 = time.time()

for i, (_, row) in enumerate(df.iterrows()):
    d = row.to_dict()
    rid    = d['rocket_id']
    Kp_best = float(d['best_Kp'])
    Kd_best = float(d['best_Kd'])

    # ── Condition 1: best_Kp from Exp1 ────────────────────────────────────────
    res_best = run_step(d, Kp_best, Kd_best)
    rt_best  = agg(res_best, 'rise_time_s')
    trms_best= agg(res_best, 'tracking_rms_deg')
    os_best  = agg(res_best, 'overshoot_deg')

    # ── Condition 2: naive Kp=2 ────────────────────────────────────────────────
    res_kp2  = run_step(d, 2.0, 1.0)
    rt_kp2   = agg(res_kp2, 'rise_time_s')
    trms_kp2 = agg(res_kp2, 'tracking_rms_deg')
    os_kp2   = agg(res_kp2, 'overshoot_deg')

    rows.append({
        'rocket_id':       rid,
        'regime_label':    d['regime_label'],
        'theta_ddot_max':  d['theta_ddot_max'],
        'nom_sr':          float(d['nominal_success_rate']),
        'under_sr':        float(d['under_success_rate']),
        'over_sr':         float(d['over_success_rate']),
        'rise_time_best':  rt_best,
        'tracking_rms_best': trms_best,
        'overshoot_best':  os_best,
        'rise_time_kp2':   rt_kp2,
        'tracking_rms_kp2': trms_kp2,
        'overshoot_kp2':   os_kp2,
        'best_Kp':         Kp_best,
        'best_Kd':         Kd_best,
        'Iyy':             float(d['Iyy']),
        'motor_scale':     float(d['motor_scale']),
        'max_gimbal_deg':  float(d['max_gimbal_deg']),
        'wind_strength':   float(d['wind_strength']),
        'servo_slew_deg_s': float(d['servo_slew_deg_s']),
        'static_margin':   float(d['static_margin']),
    })

    # Progress every 50 designs
    if (i + 1) % 50 == 0:
        elapsed = time.time() - t0
        eta = elapsed / (i + 1) * (n - i - 1)
        print(f"  [{i+1:4d}/{n}]  elapsed={elapsed:.0f}s  ETA={eta:.0f}s  "
              f"last: {rid}  rise_best={rt_best:.3f}s  rise_kp2={rt_kp2:.3f}s")

out = pd.DataFrame(rows)
out_path = os.path.join(os.path.dirname(__file__), '..',
                        'experiments', 'results', 'pitch_tracking_pareto_py.csv')
out.to_csv(out_path, index=False)

total = time.time() - t0
print(f"\nDone in {total:.0f}s.  Saved to {out_path}")
print()

# ── Quick summary ──────────────────────────────────────────────────────────────
print("=== Rise time by regime (best_Kp) ===")
for reg in ['EASY','MARGINAL','FRAGILE']:
    sub = out[out['regime_label']==reg]
    if len(sub)==0: continue
    print(f"  {reg:10s} (n={len(sub):4d}): rise={sub['rise_time_best'].mean():.3f}±{sub['rise_time_best'].std():.3f}s  "
          f"trms={sub['tracking_rms_best'].mean():.1f}°  "
          f"os={sub['overshoot_best'].mean():.1f}°")

print()
print("=== Correlation: theta_ddot_max vs agility/robustness ===")
r_rt  = float(np.corrcoef(out['theta_ddot_max'], out['rise_time_best'])[0,1])
r_sr  = float(np.corrcoef(out['theta_ddot_max'], out['nom_sr'])[0,1])
r_trms= float(np.corrcoef(out['theta_ddot_max'], out['tracking_rms_best'])[0,1])
print(f"  r(theta_ddot_max, rise_time_best)   = {r_rt:.3f}  "
      f"{'[expected: negative]' if r_rt<0 else '[UNEXPECTED positive]'}")
print(f"  r(theta_ddot_max, nom_sr)           = {r_sr:.3f}")
print(f"  r(theta_ddot_max, tracking_rms)     = {r_trms:.3f}")

print()
r_rm  = float(np.corrcoef(out['nom_sr'], out['rise_time_best'])[0,1])
print(f"  r(nom_sr, rise_time_best)           = {r_rm:.3f}  "
      f"[Pareto exists if positive: robust=slow, fragile=fast]")

print()
print("=== FRAGILE vs EASY rise time comparison ===")
frag = out[out['regime_label']=='FRAGILE']
easy = out[out['regime_label']=='EASY']
print(f"  FRAGILE rise_time_best: {frag['rise_time_best'].mean():.3f}±{frag['rise_time_best'].std():.3f}s")
print(f"  EASY    rise_time_best: {easy['rise_time_best'].mean():.3f}±{easy['rise_time_best'].std():.3f}s")
ratio = easy['rise_time_best'].mean() / frag['rise_time_best'].mean()
print(f"  Ratio (EASY/FRAGILE):   {ratio:.2f}x  [>1 means FRAGILE is faster]")
