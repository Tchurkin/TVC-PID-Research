"""
tools/adrc_saturation_test.py — Does actuator saturation drive the ADRC vs PID gap?

Carlson (2025, arXiv:2501.11374): for first/second-order linear plants, linear ADRC
(bandwidth-method tuning: omega_c, omega0, b0) is mathematically equivalent to a 2-DOF
PID with set-point weighting and a measurement low-pass filter. Our rocket dynamics are
second-order. So why does ADRC outperform PID?

HYPOTHESIS: The gap lives in the NONLINEAR/SATURATED regime.
  - PID must use high Kp to reject wind disturbances → exceeds gain ceiling → enters
    bang-bang slew saturation → oscillation / failure.
  - ADRC's ESO (z3 term) cancels wind disturbance → controller needs only low omega_c
    → stays BELOW the saturation threshold → linear operation preserved.
  - Without saturation (slew=False), both controllers operate in the linear regime;
    Carlson's equivalence should hold and the gap should narrow substantially.

NOTE — Implementation detail (ESO u_prev wiring):
  In our simulator (simulator.py lines 258-264), the ESO receives u_cmd (the controller
  output, clipped to ±u_max) as its u_prev, NOT u_act (the slew-limited actual deflection
  from step_actuator). The ESO infers actuator saturation INDIRECTLY through the observation
  error (theta_meas - z1), which z3 absorbs. This means the mechanism being tested is:
  "ADRC prevents saturation by requiring lower effective Kp (via ESO disturbance rejection)"
  rather than "ADRC directly compensates through saturation" — both are valid saturation
  mechanisms, but ours is the prevention path, not the through-saturation path.

NOTE — Anti-windup: Ki=0.0 in all experiments (pure PD, no integral term). No windup risk.
  Even if Ki were nonzero, the PID implementation has anti-windup clamping (controller.py L112).

TEST: 2x2 design, 4 conditions per design:
  1. sat_on,  PID  (best Kp/Kd from 18x7 grid, full physics)
  2. sat_on,  ADRC (omega_c=5, omega0=25, b0=keff)
  3. sat_off, PID  (same best gains, slew=False — servo tracks commanded angle perfectly)
  4. sat_off, ADRC (same ADRC settings, slew=False)

KEY METRICS:
  gap_sat    = SR_adrc_sat - SR_pid_sat       (ADRC advantage in real hardware)
  gap_nosat  = SR_adrc_nosat - SR_pid_nosat   (ADRC advantage without saturation)
  gap_closure = (gap_sat - gap_nosat) / gap_sat  (what fraction vanishes w/o saturation)
  slew_sat_frac_pid: fraction of timesteps where servo hits slew limit (PID+sat condition)
  slew_sat_frac_adrc: same for ADRC+sat — expected to be LOWER than PID (the mechanism)

  If hypothesis is correct:
    - gap_sat >> gap_nosat at high Pi (large closure fraction)
    - slew_sat_frac_pid >> slew_sat_frac_adrc (PID saturates; ADRC doesn't need to)
    - slew_sat_frac_pid correlates with Pi and with gap_sat

Design selection: 15 designs from exp1_final_population, stratified by Pi (3 per quintile),
  preferring FRAGILE where available to maximize dynamic range.
Seeds: 10001-10015 (disjoint from all prior experiments).
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

F15_AVG = 14.4
L_NOZ   = 0.25
CU_RAD  = np.pi / 180 * 15.0 / 12.0
U_MAX   = 12.0

N_EVAL_SEEDS    = 15
EVAL_SEED_START = 10001
SEARCH_SEEDS    = [1, 2, 3]
SR_PASS         = 0.80
N_DESIGNS       = 15   # 3 per Pi quintile

KP_COARSE = np.logspace(np.log10(1.0), np.log10(320.0), 18)
KD_VALS   = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]

OMEGA_C_ADRC = 5.0
OMEGA0_ADRC  = 25.0


def keff_full(row):
    return F15_AVG * row['motor_scale'] * CU_RAD * L_NOZ / row['Iyy']


def _build_physics(row, slew_on):
    plant       = build_plant(row)
    actuator    = build_actuator(row)
    sensor      = build_sensor(row)
    disturbance = build_disturbance(row)
    scenario    = build_scenario(theta0_bias_std=0.0)
    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=slew_on, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    actuator, sensor, disturbance, scenario = apply_fidelity_config(
        actuator, sensor, disturbance, scenario, fid)
    return plant, actuator, sensor, disturbance, scenario


def _eval_sr(row, controller, seeds, slew_on, is_adrc):
    """Return (sr, mean_slew_sat_frac)."""
    plant, act, sen, dis, sc = _build_physics(row, slew_on)
    successes = 0
    slew_fracs = []
    if is_adrc:
        dummy = PIDParams(Kp=0.0, Kd=0.0, u_max=U_MAX)
        for s in seeds:
            res = simulate(dummy, plant, act, sen, dis, sc, seed=s, adrc=controller)
            successes += int(res.success)
            slew_fracs.append(res.slew_sat_frac)
    else:
        for s in seeds:
            res = simulate(controller, plant, act, sen, dis, sc, seed=s)
            successes += int(res.success)
            slew_fracs.append(res.slew_sat_frac)
    return successes / len(seeds), float(np.mean(slew_fracs))


def find_best_pid_gains(row):
    best_sr, best_Kp, best_Kd = 0.0, 10.0, 2.0
    for Kp in KP_COARSE:
        for Kd in KD_VALS:
            pid = PIDParams(Kp=Kp, Kd=Kd, u_max=U_MAX)
            sr, _ = _eval_sr(row, pid, SEARCH_SEEDS, slew_on=True, is_adrc=False)
            if sr > best_sr or (sr == best_sr and abs(Kp - 40) < abs(best_Kp - 40)):
                best_sr, best_Kp, best_Kd = sr, Kp, Kd
    return best_Kp, best_Kd


def process_design(row_dict):
    keff = keff_full(row_dict)
    td   = keff * (row_dict['max_gimbal_deg'] * 12.0 / 15.0)
    lat  = int(row_dict['latency_steps'])
    Pi   = td * lat ** 2
    rid  = row_dict.get('rocket_id', 'UNK')
    lbl  = row_dict.get('final_label', 'UNKNOWN')
    eval_seeds = list(range(EVAL_SEED_START, EVAL_SEED_START + N_EVAL_SEEDS))

    # Gain search with saturation ON (real-world tuning condition)
    best_Kp, best_Kd = find_best_pid_gains(row_dict)
    pid  = PIDParams(Kp=best_Kp, Kd=best_Kd, u_max=U_MAX)
    adrc = ADRCParams(omega_c=OMEGA_C_ADRC, omega0=OMEGA0_ADRC, b0=keff, u_max=U_MAX)

    # 4 conditions (returns sr, slew_sat_frac)
    pid_sat,    slew_pid_sat    = _eval_sr(row_dict, pid,  eval_seeds, slew_on=True,  is_adrc=False)
    adrc_sat,   slew_adrc_sat   = _eval_sr(row_dict, adrc, eval_seeds, slew_on=True,  is_adrc=True)
    pid_nosat,  slew_pid_nosat  = _eval_sr(row_dict, pid,  eval_seeds, slew_on=False, is_adrc=False)
    adrc_nosat, slew_adrc_nosat = _eval_sr(row_dict, adrc, eval_seeds, slew_on=False, is_adrc=True)

    gap_sat   = adrc_sat   - pid_sat
    gap_nosat = adrc_nosat - pid_nosat

    return {
        'rocket_id':          rid,
        'label':              lbl,
        'td':                 round(td, 2),
        'keff':               round(keff, 3),
        'latency':            lat,
        'Pi':                 round(Pi, 1),
        'best_Kp':            round(best_Kp, 2),
        'best_Kd':            round(best_Kd, 2),
        # Success rates
        'pid_sat':            round(pid_sat, 4),
        'adrc_sat':           round(adrc_sat, 4),
        'pid_nosat':          round(pid_nosat, 4),
        'adrc_nosat':         round(adrc_nosat, 4),
        # Gap metrics
        'gap_sat':            round(gap_sat, 4),
        'gap_nosat':          round(gap_nosat, 4),
        'gap_explained':      round(gap_sat - gap_nosat, 4),
        'pid_lift':           round(pid_nosat  - pid_sat,  4),
        'adrc_lift':          round(adrc_nosat - adrc_sat, 4),
        # Slew saturation fractions (key mechanism diagnostic)
        'slew_frac_pid_sat':  round(slew_pid_sat, 4),
        'slew_frac_adrc_sat': round(slew_adrc_sat, 4),
        'slew_frac_pid_nosat':round(slew_pid_nosat, 4),
        'slew_diff':          round(slew_pid_sat - slew_adrc_sat, 4),  # PID saturates more
    }


def main():
    print("Loading final population...")
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    pop['keff_c'] = F15_AVG * pop['motor_scale'] * CU_RAD * L_NOZ / pop['Iyy']
    pop['td_c']   = pop['keff_c'] * (pop['max_gimbal_deg'] * 12.0 / 15.0)
    pop['Pi_c']   = pop['td_c'] * pop['latency_steps'] ** 2

    # Design selection: need good Pi dynamic range including high-Pi FRAGILE designs.
    # Pi distribution is very skewed (98% EASY at low Pi), so quintile-based misses FRAGILE.
    # Strategy: take ALL FRAGILE (n=36), then sample EASY across 5 log-spaced Pi bins.
    fragile_all = pop[pop['final_label'] == 'FRAGILE'].copy()
    easy_all    = pop[pop['final_label'] == 'EASY'].copy()

    # Sample EASY: 2 per log-Pi quintile (gives 10 EASY across range)
    easy_sorted = easy_all.sort_values('Pi_c').reset_index(drop=True)
    q_size = len(easy_sorted) // 5
    easy_sel = []
    for i in range(5):
        chunk = easy_sorted.iloc[i * q_size : (i + 1) * q_size]
        easy_sel.append(chunk.sample(n=min(2, len(chunk)), random_state=555))
    easy_selected = pd.concat(easy_sel)

    # Take a balanced subset of FRAGILE (5 across Pi range)
    frag_sorted = fragile_all.sort_values('Pi_c').reset_index(drop=True)
    frag_idx = np.linspace(0, len(frag_sorted) - 1, 5, dtype=int)
    frag_selected = frag_sorted.iloc[frag_idx]

    designs = pd.concat([easy_selected, frag_selected]).reset_index(drop=True)
    if 'rocket_id' not in designs.columns:
        designs['rocket_id'] = designs.index.map(lambda i: f"D{i:04d}")

    print(f"Selected {len(designs)} designs")
    print(f"Pi range: {designs['Pi_c'].min():.0f} to {designs['Pi_c'].max():.0f}")
    print(f"Labels: {designs['final_label'].value_counts().to_dict()}")
    print(f"Latency range: {designs['latency_steps'].min()} - {designs['latency_steps'].max()}")

    n_per = len(KP_COARSE) * len(KD_VALS) * len(SEARCH_SEEDS) + N_EVAL_SEEDS * 4
    print(f"Sims per design: {n_per}  |  Total: {len(designs) * n_per:,}")
    print("Running parallel...\n")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(process_design)(row.to_dict())
        for _, row in designs.iterrows()
    )

    df = pd.DataFrame(results).sort_values('Pi').reset_index(drop=True)
    out = RESULTS / 'adrc_saturation_test_py.csv'
    df.to_csv(out, index=False)
    print(f"\nSaved {len(df)} rows to {out.name}")

    # ── Analysis ─────────────────────────────────────────────────────────────
    print("\n=== Per-design results ===")
    hdr = (f"{'ID':<10} {'Pi':>7} {'Lbl':<10} | "
           f"{'PID+s':>6} {'ADR+s':>6} {'gap_s':>6} | "
           f"{'PID-s':>6} {'ADR-s':>6} {'gap_n':>6} | "
           f"{'expl':>6} | {'slw_PID':>7} {'slw_ADR':>7} {'slw_diff':>8}")
    print(hdr)
    print("-" * len(hdr))
    for _, r in df.iterrows():
        print(f"{r.rocket_id:<10} {r.Pi:>7.0f} {r.label:<10} | "
              f"{r.pid_sat:>6.3f} {r.adrc_sat:>6.3f} {r.gap_sat:>6.3f} | "
              f"{r.pid_nosat:>6.3f} {r.adrc_nosat:>6.3f} {r.gap_nosat:>6.3f} | "
              f"{r.gap_explained:>6.3f} | "
              f"{r.slew_frac_pid_sat:>7.3f} {r.slew_frac_adrc_sat:>7.3f} {r.slew_diff:>8.3f}")

    print("\n=== Summary by Pi tier ===")
    def tier(pi):
        if pi < 1000:   return '1: Pi<1k'
        elif pi < 3000: return '2: 1k-3k'
        elif pi < 6000: return '3: 3k-6k'
        elif pi < 9000: return '4: 6k-9k'
        else:           return '5: >9k'
    df['tier'] = df['Pi'].apply(tier)
    tab = df.groupby('tier').agg(
        n=('Pi','count'),
        pid_sat=('pid_sat','mean'),
        adrc_sat=('adrc_sat','mean'),
        gap_sat=('gap_sat','mean'),
        pid_nosat=('pid_nosat','mean'),
        adrc_nosat=('adrc_nosat','mean'),
        gap_nosat=('gap_nosat','mean'),
        gap_expl=('gap_explained','mean'),
        slew_pid=('slew_frac_pid_sat','mean'),
        slew_adrc=('slew_frac_adrc_sat','mean'),
    ).round(3)
    tier_order = ['1: Pi<1k','2: 1k-3k','3: 3k-6k','4: 6k-9k','5: >9k']
    print(tab.loc[[t for t in tier_order if t in tab.index]].to_string())

    print("\n=== Correlations (Spearman, n=15) ===")
    for col, label in [
        ('gap_sat',           'gap_sat           '),
        ('gap_nosat',         'gap_nosat          '),
        ('gap_explained',     'gap_explained      '),
        ('slew_frac_pid_sat', 'slew_frac_pid_sat  '),
        ('slew_diff',         'slew_diff(PID-ADRC)'),
    ]:
        r, p = stats.spearmanr(df['Pi'], df[col])
        print(f"  rho(Pi, {label}) = {r:+.3f}  p={p:.2e}")

    # Gap closure
    mean_gap_sat   = df['gap_sat'].mean()
    mean_gap_nosat = df['gap_nosat'].mean()
    hi = df[df['Pi'] > 5000]

    print(f"\nOverall: mean gap_sat={mean_gap_sat:.3f}  mean gap_nosat={mean_gap_nosat:.3f}")
    if mean_gap_sat > 0.001:
        print(f"  Gap closure (all designs): {100*(1 - mean_gap_nosat/mean_gap_sat):.1f}%")
    if len(hi) > 0:
        gs, gn = hi['gap_sat'].mean(), hi['gap_nosat'].mean()
        print(f"High-Pi (Pi>5000, n={len(hi)}): gap_sat={gs:.3f}  gap_nosat={gn:.3f}")
        if gs > 0.001:
            print(f"  Gap closure (high-Pi): {100*(1 - gn/gs):.1f}%")

    print(f"\nMechanism check (slew saturation rates):")
    print(f"  Mean slew_frac_pid_sat  = {df['slew_frac_pid_sat'].mean():.3f}")
    print(f"  Mean slew_frac_adrc_sat = {df['slew_frac_adrc_sat'].mean():.3f}")
    print(f"  Mean slew_diff          = {df['slew_diff'].mean():.3f}  (positive = PID saturates more)")
    r_s, p_s = stats.spearmanr(df['slew_frac_pid_sat'], df['gap_sat'])
    print(f"  rho(slew_frac_pid, gap_sat) = {r_s:.3f}  p={p_s:.2e}")

    print("\nDone.")


if __name__ == '__main__':
    main()
