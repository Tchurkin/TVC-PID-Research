"""
tools/performance_frontier.py — Map the physical performance frontier.

Question: At what Pi = theta_ddot_max * latency^2 does the best achievable SR
          decline below 1.0, then 0.80 — and does ADRC push this frontier further?

This is NOT about window width. It asks: even with OPTIMAL tuning of each
architecture, where does the physics stop you?

Design:
  70 designs, stratified across 7 Pi tiers from the final corrected population:
    Tier 0: Pi < 100      (10 designs)  — universal EASY zone
    Tier 1: Pi 100-300    (10 designs)  — early EASY
    Tier 2: Pi 300-800    (10 designs)  — transition
    Tier 3: Pi 800-2000   (10 designs)  — FRAGILE territory starts
    Tier 4: Pi 2000-5000  (10 designs)  — high-FRAGILE
    Tier 5: Pi 5000-9000  (10 designs)  — INFEASIBLE territory
    Tier 6: Pi > 9000     (10 designs)  — extreme

For each design, run:
  PID  : 18×7 joint Kp x Kd grid (126 combos), 3-seed SR (same as final correction)
         → peak_pid_sr = max SR over all 126 combos (15 eval seeds)
  ADRC : omega_c in {1.5, 2, 3, 5, 7, 10, 15} (7 values), omega0=5*wc, b0=keff
         → peak_adrc_sr = max SR over 7 omega_c (15 eval seeds)

Seeds: 7001-7015 (disjoint from all prior seed ranges).
PID gain search seeds 1-3 (same as exp1_final_correction — enables direct comparison).
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
N_EVAL_SEEDS  = 15
EVAL_SEED_START = 7001    # disjoint from all prior runs
SEARCH_SEEDS    = [1, 2, 3]  # same as final_correction for comparability
SR_PASS         = 0.80

# PID grid — same 18x7 as exp1_final_correction
KP_COARSE = np.logspace(np.log10(1.0), np.log10(320.0), 18)
KD_VALS   = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]

# ADRC grid
OMEGA_C_GRID = [1.5, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0]
OMEGA0_RATIO = 5.0


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


def eval_sr(row, pid_or_adrc, seeds, is_adrc=False):
    """Evaluate SR for a set of seeds with given controller."""
    plant, act, sen, dis, sc = _build_full_physics(row)
    if is_adrc:
        pid_dummy = PIDParams(Kp=0.0, Kd=0.0, u_max=U_MAX)
        successes = sum(
            simulate(pid_dummy, plant, act, sen, dis, sc, seed=s, adrc=pid_or_adrc).success
            for s in seeds
        )
    else:
        successes = sum(
            simulate(pid_or_adrc, plant, act, sen, dis, sc, seed=s).success
            for s in seeds
        )
    return successes / len(seeds)


def find_best_pid_gains(row):
    """
    Joint Kp x Kd grid search (3 seeds) to find best-performing PID gains.
    Returns (best_Kp, best_Kd, best_sr_search).
    """
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
    rid  = row_dict.get('rocket_id', 'unknown')
    lbl  = row_dict.get('final_label', 'UNKNOWN')
    eval_seeds = list(range(EVAL_SEED_START, EVAL_SEED_START + N_EVAL_SEEDS))

    # ── PID: find best gains, then evaluate at those gains ────────────────────
    best_Kp, best_Kd, _ = find_best_pid_gains(row_dict)
    pid_best = PIDParams(Kp=best_Kp, Kd=best_Kd, u_max=U_MAX)
    peak_pid_sr = eval_sr(row_dict, pid_best, eval_seeds, is_adrc=False)

    # ── ADRC: sweep omega_c, pick best ───────────────────────────────────────
    best_adrc_sr = 0.0
    best_wc      = OMEGA_C_GRID[0]
    adrc_results = {}
    for wc in OMEGA_C_GRID:
        adrc = ADRCParams(
            omega_c=wc,
            omega0=wc * OMEGA0_RATIO,
            b0=keff,
            u_max=U_MAX,
        )
        sr = eval_sr(row_dict, adrc, eval_seeds, is_adrc=True)
        adrc_results[wc] = sr
        if sr > best_adrc_sr:
            best_adrc_sr, best_wc = sr, wc

    result = {
        'rocket_id':      rid,
        'final_label':    lbl,
        'td':             td,
        'keff':           keff,
        'latency':        lat,
        'Pi':             Pi,
        'best_Kp':        best_Kp,
        'best_Kd':        best_Kd,
        'peak_pid_sr':    peak_pid_sr,
        'peak_adrc_sr':   best_adrc_sr,
        'best_wc':        best_wc,
        'sr_delta':       best_adrc_sr - peak_pid_sr,
    }
    # add per-wc SR columns
    for wc, sr in adrc_results.items():
        result[f'adrc_sr_wc{wc}'] = sr

    return result


def select_designs(pop_df, n_per_tier=10):
    """Stratify 70 designs across 7 Pi tiers."""
    F15 = 14.4; L = 0.25; CU = np.pi/180*15/12
    pop_df = pop_df.copy()
    pop_df['keff'] = F15 * pop_df['motor_scale'] * CU * L / pop_df['Iyy']
    pop_df['td']   = pop_df['keff'] * (pop_df['max_gimbal_deg'] * 12.0 / 15.0)
    pop_df['Pi']   = pop_df['td'] * pop_df['latency_steps'] ** 2

    tier_bounds = [(0, 100), (100, 300), (300, 800), (800, 2000),
                   (2000, 5000), (5000, 9000), (9000, 1e7)]
    rng = np.random.default_rng(12345)
    selected = []
    for lo, hi in tier_bounds:
        subset = pop_df[(pop_df['Pi'] >= lo) & (pop_df['Pi'] < hi)]
        n_pick = min(n_per_tier, len(subset))
        if n_pick == 0:
            print(f"  WARNING: no designs in Pi [{lo}, {hi})")
            continue
        idx = rng.choice(len(subset), size=n_pick, replace=False)
        chosen = subset.iloc[idx]
        print(f"  Pi [{lo:5.0f}-{hi:6.0f}): n={n_pick}  "
              f"labels: {chosen['final_label'].value_counts().to_dict()}"
              f"  Pi range [{chosen['Pi'].min():.0f},{chosen['Pi'].max():.0f}]")
        selected.append(chosen)
    return pd.concat(selected).reset_index(drop=True)


def main():
    print("Loading population...")
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    print("\nSelecting designs by Pi tier:")
    designs = select_designs(pop, n_per_tier=10)
    print(f"\nTotal designs selected: {len(designs)}")
    print(f"Pi range: {designs['Pi'].min():.0f} to {designs['Pi'].max():.0f}")
    print(f"Regimes: {designs['final_label'].value_counts().to_dict()}")
    total_sims = len(designs) * (len(KP_COARSE) * len(KD_VALS) * len(SEARCH_SEEDS)
                                  + len(OMEGA_C_GRID) * N_EVAL_SEEDS
                                  + N_EVAL_SEEDS)
    print(f"\nApprox total sims: {total_sims:,}")
    print("Running parallel...")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(process_design)(row.to_dict())
        for _, row in designs.iterrows()
    )

    df = pd.DataFrame(results)
    df.to_csv(RESULTS / 'performance_frontier_py.csv', index=False)
    print(f"\nSaved {len(df)} rows to performance_frontier_py.csv")

    # ── Analysis ──────────────────────────────────────────────────────────────
    print("\n=== Performance frontier by Pi tier ===")
    tier_labels = ['<100', '100-300', '300-800', '800-2k', '2k-5k', '5k-9k', '>9k']
    bins = [0, 100, 300, 800, 2000, 5000, 9000, 1e7]
    df['Pi_tier'] = pd.cut(df['Pi'], bins=bins, labels=tier_labels)

    tab = df.groupby('Pi_tier', observed=True).agg(
        n=('peak_pid_sr', 'count'),
        pid_mean=('peak_pid_sr', 'mean'),
        pid_below80=('peak_pid_sr', lambda x: (x < SR_PASS).mean()),
        adrc_mean=('peak_adrc_sr', 'mean'),
        adrc_below80=('peak_adrc_sr', lambda x: (x < SR_PASS).mean()),
        delta_mean=('sr_delta', 'mean'),
    ).round(3)
    print(tab.to_string())

    print("\n=== Designs where ADRC succeeds but PID fails (sr_delta > 0.1) ===")
    adrc_wins = df[(df['peak_adrc_sr'] >= SR_PASS) &
                   (df['peak_pid_sr'] < SR_PASS)].sort_values('Pi')
    if len(adrc_wins) == 0:
        print("  None found")
    else:
        print(adrc_wins[['rocket_id', 'final_label', 'td', 'latency', 'Pi',
                          'peak_pid_sr', 'peak_adrc_sr', 'best_wc']].to_string())

    print("\n=== Designs where BOTH fail (adrc_sr < 0.80) ===")
    both_fail = df[df['peak_adrc_sr'] < SR_PASS].sort_values('Pi')
    if len(both_fail) == 0:
        print("  None — ADRC achieves SR>=0.80 for ALL designs at some wc")
    else:
        print(both_fail[['rocket_id', 'final_label', 'td', 'latency', 'Pi',
                          'peak_pid_sr', 'peak_adrc_sr', 'best_wc']].to_string())

    print("\n=== Spearman correlations ===")
    r_pid, p_pid = stats.spearmanr(df['Pi'], df['peak_pid_sr'])
    r_adr, p_adr = stats.spearmanr(df['Pi'], df['peak_adrc_sr'])
    r_del, p_del = stats.spearmanr(df['Pi'], df['sr_delta'])
    print(f"rho(Pi, peak_pid_sr):   {r_pid:.3f}  p={p_pid:.2e}")
    print(f"rho(Pi, peak_adrc_sr):  {r_adr:.3f}  p={p_adr:.2e}")
    print(f"rho(Pi, sr_delta):      {r_del:.3f}  p={p_del:.2e}  "
          f"[positive = ADRC advantage grows with Pi]")

    # Frontier summary
    print("\n=== Summary ===")
    for pct in [0.90, 0.80, 0.50]:
        pid_cross  = df[df['peak_pid_sr'] < pct]['Pi'].min()
        adrc_cross = df[df['peak_adrc_sr'] < pct]['Pi'].min()
        print(f"First design with SR<{pct:.0%}: PID at Pi={pid_cross:.0f}, "
              f"ADRC at Pi={adrc_cross:.0f}  "
              f"(ADRC extends frontier {adrc_cross/pid_cross:.1f}x in Pi)")

    print("\nDone.")


if __name__ == '__main__':
    main()
