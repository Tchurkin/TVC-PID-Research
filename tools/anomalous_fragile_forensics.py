"""
tools/anomalous_fragile_forensics.py

7-seed forensic evaluation of 3 anomalous low-theta_ddot_max FRAGILE designs:
  R0167: td=13.7, best_Kp=168.6, max_gimbal=2deg, nom_sr=0.667  (real mechanism?)
  R0718: td=18.7, best_Kp=1.0, over_sr=0.667, wind=0.113         (stochastic?)
  R1085: td=18.2, best_Kp=1.0, over_sr=0.667, wind=0.367         (stochastic?)

Questions to answer:
  1. Is each design genuinely FRAGILE (over_sr consistently < 0.80)?
  2. What is SR@best_Kp across 7 seeds?
  3. What is SR@1.4×best_Kp across 7 seeds?
  4. What explains the FRAGILE label? (small gimbal, latency, wind?)
"""

import sys, warnings
import numpy as np
import pandas as pd
from pathlib import Path

sys.path.insert(0, 'sim')
from simulator import simulate, PIDParams
from fidelity_config import FidelityConfig
from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from units import FRAGILE_SUCCESS_RATE, ROBUSTNESS_SUCCESS_RATE

warnings.filterwarnings('ignore')

F15_AVG_N = 14.4
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * 15/12
OVER_SCALE = 1.40

CSV_PATH = 'experiments/results/exp1_regime_index_py.csv'
df = pd.read_csv(CSV_PATH)
df['keff_full'] = F15_AVG_N * df['motor_scale'] * CU_TO_RAD * L_NOZZLE / df['Iyy']
df['u_max']     = df['max_gimbal_deg'] * 12.0 / 15.0
df['td_max']    = df['keff_full'] * df['u_max']


def run_kp(row, kp, n_seeds=7, config=None):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario()
    fc    = config or FidelityConfig.full_physics()
    pid   = PIDParams(Kp=kp, Kd=float(row['best_Kd']), Ki=0.0,
                      u_max=act.u_max, i_lim=act.u_max)
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s, fidelity=fc)
               for s in range(1, n_seeds + 1)]
    sr   = np.mean([r.success for r in results])
    rms  = np.mean([r.rms_error_deg for r in results])
    mmax = np.mean([r.max_theta_deg for r in results])
    return sr, rms, mmax


def print_sep(label=''):
    print(f"\n{'='*60}" + (f" {label}" if label else ''))


# ── Identify the anomalous designs ───────────────────────────────
ANOMALOUS_IDS = ['R0167', 'R0718', 'R1085']
anomalous = df[df['design_id'].isin(ANOMALOUS_IDS)].copy()

if anomalous.empty:
    # Try looking at FRAGILE designs with very low theta_ddot
    df_frag = df[df['regime_label'] == 'FRAGILE'].copy()
    df_frag = df_frag.sort_values('td_max')
    print("FRAGILE sorted by theta_ddot_max (lowest first):")
    print(df_frag[['design_id','td_max','best_Kp','max_gimbal_deg','Iyy',
                   'wind_strength','latency_steps','nominal_success_rate',
                   'over_success_rate','under_success_rate']].head(10).to_string())
    print("\nPicking lowest 3 theta_ddot FRAGILE designs for forensics.")
    anomalous = df_frag.head(3)
    ANOMALOUS_IDS = list(anomalous['design_id'])

print_sep("ANOMALOUS LOW-theta_ddot FRAGILE FORENSICS")
print(f"Target designs: {ANOMALOUS_IDS}")

for _, row in anomalous.iterrows():
    did = row['design_id']
    td  = row['td_max']
    kp  = row['best_Kp']
    kd  = row['best_Kd']
    kp_over = kp * OVER_SCALE

    print_sep(f"{did}  (theta_ddot={td:.1f} rad/s2, Kp={kp:.1f}, Kd={kd:.1f})")
    print(f"  Iyy={row['Iyy']:.4f}  keff={row['keff_full']:.2f}  max_gimbal={row['max_gimbal_deg']:.1f}deg")
    print(f"  wind={row['wind_strength']:.3f}  latency_steps={row['latency_steps']}")
    print(f"  Exp1 results: nom_sr={row['nominal_success_rate']:.3f} "
          f"over_sr={row['over_success_rate']:.3f} under_sr={row['under_success_rate']:.3f}")

    # Q1: 7-seed at best_Kp
    sr_nom, rms_nom, mmax_nom = run_kp(row, kp, n_seeds=7)
    print(f"\n  7-seed @ Kp={kp:.1f} (best_Kp):  SR={sr_nom:.3f}  RMS={rms_nom:.1f}deg  max={mmax_nom:.1f}deg")

    # Q2: 7-seed at 1.4×Kp
    sr_ov, rms_ov, mmax_ov = run_kp(row, kp_over, n_seeds=7)
    print(f"  7-seed @ Kp={kp_over:.1f} (1.4×):   SR={sr_ov:.3f}  RMS={rms_ov:.1f}deg  max={mmax_ov:.1f}deg")

    # Q3: 7-seed at 0.6×Kp
    kp_un = kp * 0.60
    sr_un, rms_un, mmax_un = run_kp(row, kp_un, n_seeds=7)
    print(f"  7-seed @ Kp={kp_un:.1f} (0.6×):   SR={sr_un:.3f}  RMS={rms_un:.1f}deg  max={mmax_un:.1f}deg")

    # Q4: Scan Kp to find floor and ceiling
    print("\n  Kp scan (7-seed each):")
    kp_scan = [1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 80.0, 120.0, 160.0, 200.0, 280.0, 320.0]
    for kp_s in kp_scan:
        sr_s, rms_s, _ = run_kp(row, kp_s, n_seeds=7)
        flag = ' <-- best' if abs(kp_s - kp) < 1.0 else ''
        print(f"    Kp={kp_s:6.1f}: SR={sr_s:.3f}  RMS={rms_s:.1f}deg{flag}")

    # Verdict
    print(f"\n  VERDICT:")
    if sr_ov < ROBUSTNESS_SUCCESS_RATE:
        print(f"    CONFIRMED FRAGILE at 1.4×Kp (SR={sr_ov:.3f} < {ROBUSTNESS_SUCCESS_RATE})")
        if sr_nom < 0.80:
            print(f"    nom_sr={sr_nom:.3f} < 0.80 → ALSO marginally nominal (borderline design)")
        if td < 30:
            print(f"    Mechanism: small max_gimbal ({row['max_gimbal_deg']:.1f}deg) → Kp floor≈ceiling")
    else:
        p_fp = 1 - ROBUSTNESS_SUCCESS_RATE ** 3  # P(any of 3 fails | true SR)
        print(f"    LIKELY STOCHASTIC: 7-seed SR@1.4x={sr_ov:.3f} >= {ROBUSTNESS_SUCCESS_RATE}")
        print(f"    The 3-seed FRAGILE label may be a false positive (P≈{p_fp:.2f} per design)")
        print(f"    Recommend: reclassify as EASY or MARGINAL pending more seeds")

print_sep("SUMMARY RECOMMENDATION")
# Recount: how many of the 3 are confirmed?
print("Run the output above to see per-design verdicts.")
print("If R0718 and R1085 are stochastic false-positives, effective FRAGILE n drops from 19 to 17.")
print("The AUC computation should EXCLUDE confirmed stochastic false-positives.")
print("R0167 (tiny 2deg gimbal, nom_sr=0.667) is likely a genuine new FRAGILE mechanism.")
print("Physical explanation: small gimbal forces Kp=168 for wind rejection,")
print("  but at Kp=168 the design's keff (per-unit sensitivity) causes oscillation -> floor=ceiling.")
