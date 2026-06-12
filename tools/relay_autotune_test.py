"""
relay_autotune_test.py — Relay feedback autotuning from the bang-bang failure mode.

Theory (Astrom-Hagglund 1984, describing function analysis):
  When a TVC controller saturates its servo (u = +/-u_max), the closed loop is running
  a relay feedback experiment. The limit cycle amplitude A (rad) and relay amplitude
  d (CU) determine the ultimate gain:

    K_u = 4d / (pi * A_theta)   [CU/rad]   (describing function formula)

  Ziegler-Nichols PD tuning from relay:
    Kp_relay = 0.6 * K_u
    Kd_relay = Kp_relay * T_u / 8    (T_u = oscillation period)

  Or for a "no-overshoot" conservative setting:
    Kp_relay = 0.2 * K_u

Claim: a single bad flight at Kp=2 (where bang-bang occurs) contains enough information
to extract the optimal PID gains analytically, with no model knowledge required.

Test procedure:
  1. Run each FRAGILE design at Kp=2 (full physics), capture theta trajectory
  2. Extract amplitude A and period T_u from oscillations (3 seeds -> average)
  3. Compute K_u and Kp_relay
  4. Compare Kp_relay to best_Kp from Exp1 (prediction vs ground truth)
  5. Actually run at Kp_relay and measure SR (does the relay-derived gain work?)
  6. Compare SR(Kp_relay) vs SR(best_Kp) and SR(ADRC)
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
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25
T_END     = 3.0
WARMUP_S  = 0.3   # discard first 0.3s (transient before bang-bang establishes)

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)


def run_and_capture(design, Kp, Kd, seeds):
    """Run simulation; return list of SimResult (has rms, peak, slew_sat) plus theta arrays."""
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = ScenarioParams(t_end=T_END)
    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    return [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in seeds]


def extract_relay_params(results, dt=0.005, u_max=12.0):
    """
    Extract K_u, T_u, and Kp_relay from simulation results.

    Uses theta_true from each result.  Warmup period discarded.
    A = 90th percentile of |theta| (robust to noise, avoids peak inflation)
    T_u = average period from zero crossings of theta
    """
    warmup_steps = int(WARMUP_S / dt)
    As, Ts = [], []

    for r in results:
        theta_d = np.degrees(r.theta_true)[warmup_steps:]
        # Amplitude: use 90th percentile of |theta| for robustness
        A_deg = float(np.percentile(np.abs(theta_d), 90))
        As.append(A_deg)

        # Period: count zero crossings of theta (estimate oscillation frequency)
        signs = np.sign(theta_d)
        signs = signs[signs != 0]
        if len(signs) > 2:
            crossings = np.sum(np.abs(np.diff(signs)) > 0)
            if crossings > 1:
                T_u = 2.0 * (len(theta_d) * dt) / crossings  # full period
                Ts.append(T_u)

    A_mean_deg = float(np.mean(As))
    A_mean_rad = np.deg2rad(A_mean_deg)
    T_u_mean   = float(np.mean(Ts)) if Ts else 0.3  # fallback 0.3s

    K_u     = 4.0 * u_max / (np.pi * A_mean_rad)
    Kp_zn   = 0.6 * K_u   # Ziegler-Nichols aggressive
    Kp_cons = 0.2 * K_u   # conservative (no-overshoot)
    Kd_zn   = Kp_zn * T_u_mean / 8.0

    return {
        'A_deg': A_mean_deg,
        'T_u_s': T_u_mean,
        'K_u':   K_u,
        'Kp_zn': Kp_zn,
        'Kp_cons': Kp_cons,
        'Kd_zn': Kd_zn,
    }


# ── Load FRAGILE designs ───────────────────────────────────────────────────────
df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                              'exp1_regime_index_py.csv'))
fragile = df[df['regime_label'] == 'FRAGILE'].copy()
print(f"FRAGILE n={len(fragile)}")
print()

seeds_probe = [1, 2, 3]   # seeds for Kp=2 probe flight
seeds_eval  = list(range(1, 8))  # 7 seeds for relay-derived gain evaluation

rows = []
for _, row in fragile.iterrows():
    design = row.to_dict()
    rid    = design['rocket_id']
    best_Kp = float(design['best_Kp'])
    best_Kd = float(design['best_Kd'])

    # 1. Probe flight at Kp=2 (the "bad flight")
    plant = build_plant(design)
    u_max_design = build_actuator(design).u_max
    probe_res = run_and_capture(design, Kp=2.0, Kd=1.0, seeds=seeds_probe)

    # 2. Extract relay parameters
    rp = extract_relay_params(probe_res, u_max=u_max_design)

    # 3. Evaluate relay-derived gains (ZN aggressive)
    Kp_relay = rp['Kp_zn']
    Kd_relay = max(rp['Kd_zn'], 1.0)  # floor Kd at 1.0
    relay_res = run_and_capture(design, Kp=Kp_relay, Kd=Kd_relay, seeds=seeds_eval)

    # 4. Evaluate relay-conservative gains
    Kp_cons = rp['Kp_cons']
    cons_res = run_and_capture(design, Kp=Kp_cons, Kd=Kd_relay, seeds=seeds_eval)

    # 5. Evaluate best_Kp from Exp1 (ground truth ceiling)
    best_res = run_and_capture(design, Kp=best_Kp, Kd=best_Kd, seeds=seeds_eval)

    sr_probe  = np.mean([r.success for r in probe_res])
    sr_relay  = np.mean([r.success for r in relay_res])
    sr_cons   = np.mean([r.success for r in cons_res])
    sr_best   = np.mean([r.success for r in best_res])
    rms_relay = np.mean([r.rms_error_deg for r in relay_res])
    rms_best  = np.mean([r.rms_error_deg for r in best_res])

    print(f"{rid}  best_Kp={best_Kp:.0f}  A={rp['A_deg']:.1f}deg  T_u={rp['T_u_s']:.2f}s  "
          f"K_u={rp['K_u']:.1f}  Kp_relay={Kp_relay:.1f}")
    print(f"  SR: probe={sr_probe:.2f}  relay={sr_relay:.2f}  cons={sr_cons:.2f}  best={sr_best:.2f}")
    print(f"  RMS: relay={rms_relay:.1f}deg  best={rms_best:.1f}deg")

    rows.append({
        'rocket_id': rid,
        'best_Kp': best_Kp, 'best_Kd': best_Kd,
        'A_deg': rp['A_deg'], 'T_u_s': rp['T_u_s'],
        'K_u': rp['K_u'], 'Kp_relay': Kp_relay, 'Kp_cons': Kp_cons,
        'sr_probe': sr_probe, 'sr_relay': sr_relay, 'sr_cons': sr_cons,
        'sr_best': sr_best,
        'rms_relay': rms_relay, 'rms_best': rms_best,
    })

res = pd.DataFrame(rows)

print()
print("=== Summary ===")
print(f"Probe (Kp=2):      mean SR = {res['sr_probe'].mean():.3f}")
print(f"Relay ZN:          mean SR = {res['sr_relay'].mean():.3f}  "
      f"(frac=1.0: {(res['sr_relay']==1.0).mean():.1%})")
print(f"Relay conservative: mean SR = {res['sr_cons'].mean():.3f}  "
      f"(frac=1.0: {(res['sr_cons']==1.0).mean():.1%})")
print(f"Best (Exp1 grid):  mean SR = {res['sr_best'].mean():.3f}")

print()
r_ku_bestkp = float(np.corrcoef(res['K_u'], res['best_Kp'])[0,1])
r_relay_best = float(np.corrcoef(res['Kp_relay'], res['best_Kp'])[0,1])
print(f"Correlation: r(K_u, best_Kp)      = {r_ku_bestkp:.3f}")
print(f"Correlation: r(Kp_relay, best_Kp) = {r_relay_best:.3f}")
print(f"Mean Kp_relay = {res['Kp_relay'].mean():.1f}  vs  mean best_Kp = {res['best_Kp'].mean():.1f}")
print(f"Mean ratio Kp_relay/best_Kp = {(res['Kp_relay']/res['best_Kp']).mean():.2f}")

print()
print("Designs where SR(relay) < 0.70:")
low = res[res['sr_relay'] < 0.70]
if len(low) == 0:
    print("  None")
else:
    print(low[['rocket_id','best_Kp','Kp_relay','sr_relay','sr_best']].to_string())

res.to_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                        'relay_autotune_py.csv'), index=False)
print()
print("Saved to experiments/results/relay_autotune_py.csv")
