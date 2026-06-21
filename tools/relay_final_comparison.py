"""
tools/relay_final_comparison.py

Re-derives the K_u (relay-derived gain ceiling) EASY vs FRAGILE comparison on the
FINAL (v2-corrected) n=36 FRAGILE population, superseding relay_easy_comparison.py
(n=25 FRAGILE, stale pre-correction labels, exp1_regime_index_py.csv).

Method unchanged: probe flight at Kp=2, Kd=1, full physics, 5 seeds; extract bang-bang
oscillation amplitude (90th percentile |theta|) and period from zero-crossings; convert
to K_u via the describing-function formula (Astrom-Hagglund 1984): K_u = 4*u_max/(pi*A_rad).

Output: experiments/results/relay_final_comparison_py.csv
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

T_END    = 3.0
WARMUP_S = 0.3
N_PROBE  = 5

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)


def run_probe(design, seeds):
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = ScenarioParams(t_end=T_END)
    pid   = PIDParams(Kp=2.0, Kd=1.0, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    return [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in seeds]


def extract_relay_params(results, u_max, dt=0.005):
    warmup = int(WARMUP_S / dt)
    As, Ts = [], []
    for r in results:
        theta_d = np.degrees(r.theta_true)[warmup:]
        A_deg = float(np.percentile(np.abs(theta_d), 90))
        As.append(A_deg)
        signs = np.sign(theta_d)
        signs = signs[signs != 0]
        if len(signs) > 2:
            crossings = np.sum(np.abs(np.diff(signs)) > 0)
            if crossings > 1:
                T_u = 2.0 * (len(theta_d) * dt) / crossings
                Ts.append(T_u)
    A_mean = float(np.mean(As))
    T_mean = float(np.mean(Ts)) if Ts else 0.5
    K_u    = 4.0 * u_max / (np.pi * np.deg2rad(A_mean))
    return A_mean, T_mean, K_u


df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                              'exp1_final_population_py.csv'))

frag_sel = df[df.final_label == 'FRAGILE'].copy()
n_sample = len(frag_sel)

easy = df[df.final_label == 'EASY'].sort_values('td').reset_index(drop=True)
idx  = np.linspace(0, len(easy) - 1, n_sample, dtype=int)
easy_sel = easy.iloc[idx].copy()

print(f"Running relay probe on {len(easy_sel)} EASY + {len(frag_sel)} FRAGILE designs (final population)")
print(f"Probe: Kp=2, {N_PROBE} seeds, t_end={T_END}s")
print()

seeds = list(range(1, N_PROBE + 1))
rows  = []

for regime, subset in [('EASY', easy_sel), ('FRAGILE', frag_sel)]:
    for _, row in subset.iterrows():
        d      = row.to_dict()
        rid    = d['rocket_id']
        u_max  = build_actuator(d).u_max

        results = run_probe(d, seeds)
        A, T_u, K_u = extract_relay_params(results, u_max=u_max)
        probe_rms  = np.mean([r.rms_error_deg for r in results])
        probe_slew = np.mean([r.slew_sat_frac for r in results])

        print(f"  {regime:7s}  {rid}  td={d['td']:6.1f}  "
              f"A={A:5.1f}deg  K_u={K_u:7.1f}  rms={probe_rms:.1f}  slew={probe_slew:.2f}")

        rows.append({
            'rocket_id':        rid,
            'regime':           regime,
            'theta_ddot':       d['td'],
            'Iyy':              d['Iyy'],
            'motor_scale':      d['motor_scale'],
            'max_gimbal_deg':   d['max_gimbal_deg'],
            'servo_slew_deg_s': d['servo_slew_deg_s'],
            'latency_steps':    d['latency_steps'],
            'A_deg':            A,
            'T_u_s':            T_u,
            'K_u':              K_u,
            'probe_rms':        probe_rms,
            'probe_slew_sat':   probe_slew,
        })

out = pd.DataFrame(rows)
out_path = os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results',
                        'relay_final_comparison_py.csv')
out.to_csv(out_path, index=False)

print()
print("=== SUMMARY (final n=36 FRAGILE population) ===")
for reg in ['EASY', 'FRAGILE']:
    s = out[out.regime == reg]
    print(f"  {reg:7s} (n={len(s)}):  "
          f"K_u={s.K_u.mean():.1f}+-{s.K_u.std():.1f}  median={s.K_u.median():.1f}  "
          f"td={s.theta_ddot.mean():.1f}  rms@Kp2={s.probe_rms.mean():.1f} deg")

from scipy.stats import pearsonr, spearmanr, mannwhitneyu
r_lin, _ = pearsonr(out.theta_ddot, out.K_u)
r_sp, _  = spearmanr(out.theta_ddot, out.K_u)
valid = out[(out.K_u > 0) & (out.theta_ddot > 0)]
r_log, _ = pearsonr(np.log(valid.theta_ddot), np.log(valid.K_u))
B, logA  = np.polyfit(np.log(valid.theta_ddot), np.log(valid.K_u), 1)

frag_ku = out[out.regime == 'FRAGILE'].K_u
easy_ku = out[out.regime == 'EASY'].K_u
u_stat, p_mw = mannwhitneyu(easy_ku, frag_ku, alternative='greater')

print()
print(f"  r(theta_ddot, K_u):       {r_lin:+.3f}")
print(f"  rho(theta_ddot, K_u):     {r_sp:+.3f}")
print(f"  r(log, log):              {r_log:+.3f}")
print(f"  Power law: K_u ~ {np.exp(logA):.0f} x theta_ddot^{B:.2f}")
print(f"  Mann-Whitney (EASY K_u > FRAGILE K_u): p={p_mw:.4g}")
print(f"  Separation ratio (EASY median / FRAGILE median): {easy_ku.median()/frag_ku.median():.2f}x")
print()
print(f"Saved to {out_path}")
