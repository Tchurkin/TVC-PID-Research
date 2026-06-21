"""
tools/adrc_frontier_extension.py — Extend the ADRC performance frontier past Pi=12,648.

The original performance_frontier.py only reached Pi=12,648 (population maximum from
exp1_final_population_py.csv). This left the critical question open: we measured
"ADRC extends PID's frontier 2.4×", but 12,648 is also the MAX Pi in that population,
so ADRC might not have actually failed — we just ran out of designs.

This experiment uses the STRESS TEST population (latency 1-12 steps) which has designs
up to Pi=49,893. It tests a wider ADRC grid:
  omega_c:      {0.5, 1.0, 1.5, 2.0, 3.0, 5.0}
  omega0/omega_c ratio: {5, 8, 12, 20}  (standard=5; ADRC theory allows higher ratios)
  → 6 × 4 = 24 ADRC settings per design (vs 7 settings in original frontier)

Seeds: 9001-9015 (disjoint from all prior experiments).

Design selection: all designs from stress test with Pi ≥ 10,000 (spans the transition
from frontier crossing to extreme territory). Also includes continuity overlap
(5 designs from Pi=9000-12648 range for cross-validation).
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams, ADRCParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

# ── Constants ─────────────────────────────────────────────────────────────────
F15_AVG = 14.4
L_NOZ   = 0.25
CU_RAD  = np.pi / 180 * 15.0 / 12.0
U_MAX   = 12.0

# ── Experiment settings ───────────────────────────────────────────────────────
N_EVAL_SEEDS    = 15
EVAL_SEED_START = 9001   # disjoint from all prior runs
SEARCH_SEEDS    = [1, 2, 3]  # same as final_correction for comparability
SR_PASS         = 0.80

# PID grid — same 18x7 as exp1_final_correction
KP_COARSE = np.logspace(np.log10(1.0), np.log10(320.0), 18)
KD_VALS   = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]

# Extended ADRC grid: omega_c × omega0_ratio
OMEGA_C_GRID    = [0.5, 1.0, 1.5, 2.0, 3.0, 5.0]
OMEGA0_RATIO_GRID = [5, 8, 12, 20]

PI_SELECT_MIN = 9000  # all designs above this Pi value


def keff_full(row):
    return F15_AVG * row['motor_scale'] * CU_RAD * L_NOZ / row['Iyy']


def _build_full_physics(row):
    plant       = build_plant(row)
    actuator    = build_actuator(row)
    sensor      = build_sensor(row)
    disturbance = build_disturbance(row)
    scenario    = build_scenario(theta0_bias_std=0.0)
    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    actuator, sensor, disturbance, scenario = apply_fidelity_config(
        actuator, sensor, disturbance, scenario, fid)
    return plant, actuator, sensor, disturbance, scenario


def eval_sr(row, controller, seeds, is_adrc=False):
    plant, act, sen, dis, sc = _build_full_physics(row)
    if is_adrc:
        pid_dummy = PIDParams(Kp=0.0, Kd=0.0, u_max=U_MAX)
        successes = sum(
            simulate(pid_dummy, plant, act, sen, dis, sc, seed=s, adrc=controller).success
            for s in seeds
        )
    else:
        successes = sum(
            simulate(controller, plant, act, sen, dis, sc, seed=s).success
            for s in seeds
        )
    return successes / len(seeds)


def find_best_pid_gains(row):
    best_sr, best_Kp, best_Kd = 0.0, 10.0, 2.0
    for Kp in KP_COARSE:
        for Kd in KD_VALS:
            pid = PIDParams(Kp=Kp, Kd=Kd, u_max=U_MAX)
            sr  = eval_sr(row, pid, SEARCH_SEEDS, is_adrc=False)
            if sr > best_sr or (sr == best_sr and abs(Kp - 40) < abs(best_Kp - 40)):
                best_sr, best_Kp, best_Kd = sr, Kp, Kd
    return best_Kp, best_Kd, best_sr


def process_design(row_dict):
    keff = keff_full(row_dict)
    td   = keff * (row_dict['max_gimbal_deg'] * 12.0 / 15.0)
    lat  = row_dict['latency_steps']
    Pi   = td * lat ** 2
    rid  = row_dict.get('rocket_id', f"R{row_dict.get('design_id', 'UNK')}")
    lbl  = row_dict.get('regime_label', row_dict.get('final_label', 'UNKNOWN'))
    eval_seeds = list(range(EVAL_SEED_START, EVAL_SEED_START + N_EVAL_SEEDS))

    # PID: find best gains then evaluate
    best_Kp, best_Kd, _ = find_best_pid_gains(row_dict)
    pid_best = PIDParams(Kp=best_Kp, Kd=best_Kd, u_max=U_MAX)
    peak_pid_sr = eval_sr(row_dict, pid_best, eval_seeds, is_adrc=False)

    # ADRC: sweep (omega_c × omega0_ratio) grid, pick best
    best_adrc_sr  = 0.0
    best_wc       = OMEGA_C_GRID[0]
    best_w0_ratio = OMEGA0_RATIO_GRID[0]
    adrc_results  = {}   # keyed by (wc, ratio)

    for wc in OMEGA_C_GRID:
        for ratio in OMEGA0_RATIO_GRID:
            adrc = ADRCParams(
                omega_c=wc,
                omega0=wc * ratio,
                b0=keff,
                u_max=U_MAX,
            )
            sr = eval_sr(row_dict, adrc, eval_seeds, is_adrc=True)
            adrc_results[(wc, ratio)] = sr
            if sr > best_adrc_sr:
                best_adrc_sr  = sr
                best_wc       = wc
                best_w0_ratio = ratio

    # per omega_c: best SR over all omega0 ratios (max-marginalised)
    per_wc = {}
    for wc in OMEGA_C_GRID:
        per_wc[wc] = max(adrc_results[(wc, r)] for r in OMEGA0_RATIO_GRID)
    # best at standard ratio (omega0=5×omega_c)
    adrc_sr_std = max(adrc_results[(wc, 5)] for wc in OMEGA_C_GRID)

    result = {
        'rocket_id':      rid,
        'label':          lbl,
        'td':             td,
        'keff':           keff,
        'latency':        lat,
        'Pi':             Pi,
        'best_Kp':        best_Kp,
        'best_Kd':        best_Kd,
        'peak_pid_sr':    peak_pid_sr,
        'peak_adrc_sr':   best_adrc_sr,
        'adrc_sr_std_ratio': adrc_sr_std,   # omega0=5×omega_c only (matches original frontier)
        'best_wc':        best_wc,
        'best_w0_ratio':  best_w0_ratio,
        'sr_delta':       best_adrc_sr - peak_pid_sr,
    }
    for wc, sr in per_wc.items():
        result[f'adrc_sr_wc{wc}'] = sr

    return result


def main():
    print("Loading stress test population...")
    pop = pd.read_csv(RESULTS / 'exp1_stress_test_py.csv')

    # Compute Pi
    pop['keff_c'] = F15_AVG * pop['motor_scale'] * CU_RAD * L_NOZ / pop['Iyy']
    pop['td_c']   = pop['keff_c'] * (pop['max_gimbal_deg'] * 12.0 / 15.0)
    pop['Pi_c']   = pop['td_c'] * pop['latency_steps'] ** 2

    # Select all designs with Pi >= threshold
    designs = pop[pop['Pi_c'] >= PI_SELECT_MIN].copy().reset_index(drop=True)
    if 'rocket_id' not in designs.columns:
        designs['rocket_id'] = [f"ST{i}" for i in designs.index]

    print(f"Selected designs with Pi >= {PI_SELECT_MIN}: n={len(designs)}")
    print(f"Pi range: {designs['Pi_c'].min():.0f} to {designs['Pi_c'].max():.0f}")
    lbl_col = 'regime_label' if 'regime_label' in designs.columns else 'final_label'
    print(f"Labels: {designs[lbl_col].value_counts().to_dict()}")
    print(f"Latency range: {designs['latency_steps'].min()} - {designs['latency_steps'].max()}")

    n_adrc = len(OMEGA_C_GRID) * len(OMEGA0_RATIO_GRID)
    total = len(designs) * (
        len(KP_COARSE) * len(KD_VALS) * len(SEARCH_SEEDS)  # PID search
        + N_EVAL_SEEDS                                        # PID eval
        + n_adrc * N_EVAL_SEEDS                              # ADRC grid
    )
    print(f"\nADRC grid: {len(OMEGA_C_GRID)} omega_c × {len(OMEGA0_RATIO_GRID)} ratios = {n_adrc} settings")
    print(f"Approx total sims: {total:,}")
    print("Running parallel...\n")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(process_design)(row.to_dict())
        for _, row in designs.iterrows()
    )

    df = pd.DataFrame(results)
    out_path = RESULTS / 'adrc_frontier_extension_py.csv'
    df.to_csv(out_path, index=False)
    print(f"\nSaved {len(df)} rows to {out_path.name}")

    # ── Analysis ──────────────────────────────────────────────────────────────
    print("\n=== ADRC Frontier Extension Results ===")

    def assign_tier(pi):
        if pi < 12648: return '9k-12.6k'
        elif pi < 15000: return '12.6k-15k'
        elif pi < 20000: return '15k-20k'
        elif pi < 30000: return '20k-30k'
        else: return '>30k'

    df['Pi_tier'] = df['Pi'].apply(assign_tier)

    tab = df.groupby('Pi_tier', observed=True).agg(
        n=('peak_pid_sr', 'count'),
        pid_mean=('peak_pid_sr', 'mean'),
        pid_below80=('peak_pid_sr', lambda x: (x < SR_PASS).mean()),
        adrc_std_mean=('adrc_sr_std_ratio', 'mean'),
        adrc_std_below80=('adrc_sr_std_ratio', lambda x: (x < SR_PASS).mean()),
        adrc_ext_mean=('peak_adrc_sr', 'mean'),
        adrc_ext_below80=('peak_adrc_sr', lambda x: (x < SR_PASS).mean()),
    ).round(3)
    print(tab.to_string())

    print("\n=== Designs where extended ADRC succeeds but standard ratio fails ===")
    gap = df[(df['peak_adrc_sr'] >= SR_PASS) & (df['adrc_sr_std_ratio'] < SR_PASS)]
    if len(gap) == 0:
        print("  None — standard omega0/omega_c=5 is sufficient for all designs")
    else:
        print(gap[['rocket_id', 'label', 'td', 'latency', 'Pi',
                   'adrc_sr_std_ratio', 'peak_adrc_sr', 'best_wc', 'best_w0_ratio']].to_string())

    print("\n=== Designs where BOTH PID and extended ADRC fail (sr < 0.80) ===")
    both_fail = df[df['peak_adrc_sr'] < SR_PASS].sort_values('Pi')
    if len(both_fail) == 0:
        print("  None — extended ADRC achieves SR>=0.80 for ALL designs")
    else:
        print(both_fail[['rocket_id', 'label', 'td', 'latency', 'Pi',
                          'peak_pid_sr', 'peak_adrc_sr',
                          'best_wc', 'best_w0_ratio']].to_string())
        print(f"\n  ADRC frontier: Pi < {both_fail['Pi'].min():.0f} → ADRC succeeds")
        print(f"  ADRC frontier: Pi >= {both_fail['Pi'].min():.0f} → ADRC may fail")
        print(f"  vs. PID frontier: Pi >= {df[df['peak_pid_sr'] < SR_PASS]['Pi'].min():.0f}")
        pid_cross  = df[df['peak_pid_sr'] < SR_PASS]['Pi'].min()
        adrc_cross = both_fail['Pi'].min()
        print(f"  ADRC extends frontier {adrc_cross/pid_cross:.1f}x in Pi (extended grid)")

    print("\n=== Spearman correlations ===")
    r_pid, p_pid = stats.spearmanr(df['Pi'], df['peak_pid_sr'])
    r_adr, p_adr = stats.spearmanr(df['Pi'], df['peak_adrc_sr'])
    print(f"rho(Pi, peak_pid_sr):   {r_pid:.3f}  p={p_pid:.2e}")
    print(f"rho(Pi, peak_adrc_sr):  {r_adr:.3f}  p={p_adr:.2e}")

    print("\n=== Best omega0 ratio by design ===")
    ratio_counts = df['best_w0_ratio'].value_counts()
    print(ratio_counts)
    print(f"\nDesigns where omega0/omega_c > 5 was optimal: "
          f"{(df['best_w0_ratio'] > 5).sum()}/{len(df)}")

    print("\nDone.")


if __name__ == '__main__':
    main()
