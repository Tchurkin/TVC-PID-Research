"""
adrc_step_tracking.py — ADRC vs PID step tracking comparison, n=1200.

For each design, compare:
  - PID at best_Kp (attitude-hold-tuned)
  - ADRC at omega_c=5, omega0=25 (same as stress test)

Step: 0° -> 15° at t=1.0s, t_end=4.0s, N_SEEDS seeds.

Key question: Does ADRC's ESO-decoupled architecture achieve better step tracking
than PID's single-Kp formulation, which is forced to use Kp=80 for wind rejection
at the cost of massive step overshoot?

Output: experiments/results/adrc_step_tracking_py.csv
"""

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

import numpy as np
import pandas as pd

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from simulator   import simulate, ScenarioParams
from controller  import PIDParams, ADRCParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units       import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi/180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

T_END     = 4.0
STEP_DEG  = 15.0
STEP_TIME = 1.0
N_SEEDS   = 5
SEEDS     = list(range(1, N_SEEDS + 1))

ADRC_WC   = 5.0   # controller bandwidth (rad/s)
ADRC_W0   = 25.0  # observer bandwidth (rad/s)

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)


def run_step(design_dict, Kp, Kd, adrc_params=None):
    plant = build_plant(design_dict)
    act   = build_actuator(design_dict)
    sen   = build_sensor(design_dict)
    dis   = build_disturbance(design_dict)
    sc    = ScenarioParams(t_end=T_END,
                           theta_step_deg=STEP_DEG,
                           theta_step_time_s=STEP_TIME)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act2.u_max)
    return [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s, adrc=adrc_params)
            for s in SEEDS]


def agg(results, attr):
    return float(np.mean([getattr(r, attr) for r in results]))


df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..',
                              'experiments', 'results', 'exp1_regime_index_py.csv'))

df['theta_ddot_max'] = (F15_AVG_THRUST_N
                        * np.sin(np.deg2rad(df['max_gimbal_deg']))
                        * L_NOZZLE
                        * df['motor_scale']
                        / df['Iyy'])

n = len(df)
print(f"ADRC vs PID step tracking: {n} designs, {N_SEEDS} seeds, "
      f"ADRC wc={ADRC_WC}, w0={ADRC_W0}")
print(f"Step: {STEP_DEG}° at t={STEP_TIME}s,  t_end={T_END}s")
print()

rows = []
t0 = time.time()

for i, (_, row) in enumerate(df.iterrows()):
    d = row.to_dict()
    rid     = d['rocket_id']
    Kp_best = float(d['best_Kp'])
    Kd_best = float(d['best_Kd'])

    # Compute b0 from physics
    b0 = float(d['motor_scale']) * F15_AVG_THRUST_N * CU_TO_RAD * L_NOZZLE / float(d['Iyy'])

    # ── PID at best_Kp ────────────────────────────────────────────────────────
    pid_res = run_step(d, Kp_best, Kd_best, adrc_params=None)

    # ── ADRC ──────────────────────────────────────────────────────────────────
    adrc = ADRCParams(omega_c=ADRC_WC, omega0=ADRC_W0, b0=b0,
                      u_max=build_actuator(d).u_max)
    adr_res = run_step(d, 0.0, 0.0, adrc_params=adrc)

    rows.append({
        'rocket_id':       rid,
        'regime_label':    d['regime_label'],
        'theta_ddot_max':  d['theta_ddot_max'],
        'nom_sr':          float(d['nominal_success_rate']),
        'b0':              b0,
        # PID metrics
        'pid_sr':          np.mean([r.success for r in pid_res]),
        'pid_rise':        agg(pid_res, 'rise_time_s'),
        'pid_trms':        agg(pid_res, 'tracking_rms_deg'),
        'pid_overshoot':   agg(pid_res, 'overshoot_deg'),
        'pid_peak':        15.0 + agg(pid_res, 'overshoot_deg'),
        # ADRC metrics
        'adrc_sr':         np.mean([r.success for r in adr_res]),
        'adrc_rise':       agg(adr_res, 'rise_time_s'),
        'adrc_trms':       agg(adr_res, 'tracking_rms_deg'),
        'adrc_overshoot':  agg(adr_res, 'overshoot_deg'),
        'adrc_peak':       15.0 + agg(adr_res, 'overshoot_deg'),
        # Design params
        'best_Kp':         Kp_best,
        'Iyy':             float(d['Iyy']),
        'motor_scale':     float(d['motor_scale']),
        'max_gimbal_deg':  float(d['max_gimbal_deg']),
        'wind_strength':   float(d['wind_strength']),
    })

    if (i + 1) % 50 == 0:
        elapsed = time.time() - t0
        eta = elapsed / (i + 1) * (n - i - 1)
        print(f"  [{i+1:4d}/{n}]  elapsed={elapsed:.0f}s  ETA={eta:.0f}s  "
              f"last: {rid}  pid_trms={rows[-1]['pid_trms']:.1f}°  "
              f"adrc_trms={rows[-1]['adrc_trms']:.1f}°")

out = pd.DataFrame(rows)
out_path = os.path.join(os.path.dirname(__file__), '..',
                        'experiments', 'results', 'adrc_step_tracking_py.csv')
out.to_csv(out_path, index=False)

total = time.time() - t0
print(f"\nDone in {total:.0f}s.  Saved to {out_path}")
print()

# ── Summary ───────────────────────────────────────────────────────────────────
print("=== PID vs ADRC step tracking (n=1200) ===")
print(f"{'Metric':>25}  {'PID best_Kp':>12}  {'ADRC wc=5':>12}  {'Ratio':>8}")
print("-"*65)
for attr, label in [('_sr','SR'),('_rise','rise (s)'),('_trms','trms (°)'),('_peak','peak angle (°)')]:
    pid_v = out['pid'+attr].mean()
    adr_v = out['adrc'+attr].mean()
    ratio = pid_v / adr_v if adr_v != 0 else float('nan')
    print(f"{label:>25}  {pid_v:>12.3f}  {adr_v:>12.3f}  {ratio:>8.2f}x")

print()
print("=== By regime ===")
for reg in ['EASY','MARGINAL','FRAGILE']:
    sub = out[out['regime_label']==reg]
    if len(sub)==0: continue
    print(f"\n  {reg} (n={len(sub)}):")
    print(f"    PID:  SR={sub.pid_sr.mean():.2f}  rise={sub.pid_rise.mean():.3f}s  "
          f"trms={sub.pid_trms.mean():.1f}°  peak={sub.pid_peak.mean():.0f}°")
    print(f"    ADRC: SR={sub.adrc_sr.mean():.2f}  rise={sub.adrc_rise.mean():.3f}s  "
          f"trms={sub.adrc_trms.mean():.1f}°  peak={sub.adrc_peak.mean():.0f}°")

print()
trms_ratio = out.pid_trms.mean() / out.adrc_trms.mean()
peak_ratio = out.pid_peak.mean() / out.adrc_peak.mean()
speed_ratio = out.adrc_rise.mean() / out.pid_rise.mean()
print(f"SUMMARY: ADRC vs PID at ωc=5")
print(f"  Tracking RMS:   PID {out.pid_trms.mean():.1f}° → ADRC {out.adrc_trms.mean():.1f}°  "
      f"({trms_ratio:.1f}x improvement)")
print(f"  Peak angle:     PID {out.pid_peak.mean():.0f}° → ADRC {out.adrc_peak.mean():.0f}°  "
      f"({peak_ratio:.1f}x improvement)")
print(f"  Rise time:      PID {out.pid_rise.mean():.3f}s → ADRC {out.adrc_rise.mean():.3f}s  "
      f"({speed_ratio:.1f}x slower)")
print(f"  Success rate:   PID {out.pid_sr.mean():.2f} → ADRC {out.adrc_sr.mean():.2f}")
