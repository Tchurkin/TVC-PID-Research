"""
tools/observer_universality_test.py  --  Observer universality: does ESO specifically break the
Pi constraint, or does ANY disturbance observer?

Research question:
  PID/LQR/SMC all face rho(Pi, window) ~ -0.75 with no disturbance estimation.
  ADRC with ESO achieves rho ~ -0.325 (performance_frontier).
  The 2x2 saturation test proved ESO prevents saturation = the mechanism for ADRC's advantage.
  But does a SIMPLER disturbance observer (integral PID = classic anti-disturbance) also help?

Architecture comparison on the same 50 designs (from smc_controller_test_py.csv):
  A. PD (Ki=0):   Kp sweep [1..400], Kd=best_Kd.       rho expected: ~ -0.65 to -0.75
  B. PID-I (integral): same Kp sweep + Ki grid.         rho expected: either similar to PD
                                                         or improved if integral helps.
  C. ADRC (ESO):  omega_c sweep, fixed omega0=25, b0=keff.  rho expected: ~ -0.325

If rho(PID-I) ~ rho(PD): integral doesn't break the constraint; ESO is specifically different.
If rho(PID-I) ~ rho(ADRC): integral is sufficient; ESO is not uniquely important.

Physical argument for why integral should NOT help:
  Integral accumulates error OVER TIME to estimate and cancel disturbance.
  In the bang-bang regime (servo saturated), the integral winds up (or anti-windup truncates it).
  Anti-windup prevents the integrator from accumulating enough to cancel the disturbance.
  ESO, by contrast, estimates disturbance from state derivatives BEFORE the servo saturates.

Seeds: 11001-11010 (disjoint from all prior ranges).
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

N_SEEDS    = 10
SEED_START = 11001   # disjoint from all prior experiments
SR_PASS    = 0.80

# Gain sweep for PD and PID-I (same Kp grid for both)
KP_GRID = np.logspace(0, np.log10(400), 12)   # 12 values: 1..400 log-spaced

# Integral gain grid for PID-I (including 0 for PD baseline)
KI_GRID = np.array([0.0, 0.05, 0.2, 1.0, 4.0])  # 5 values; 0 = pure PD

# ADRC omega_c sweep (same as performance_frontier)
WC_GRID = np.array([1.5, 2, 3, 5, 7, 10, 15])

OMEGA0_OVER_WC = 5.0   # standard bandwidth separation


def keff_full(row):
    return F15_AVG * float(row['motor_scale']) * CU_RAD * L_NOZ / float(row['Iyy'])


def build_fidelity(row_dict):
    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    plant       = build_plant(row_dict)
    actuator    = build_actuator(row_dict)
    sensor      = build_sensor(row_dict)
    disturbance = build_disturbance(row_dict)
    scenario    = build_scenario(theta0_bias_std=0.0)
    actuator, sensor, disturbance, scenario = apply_fidelity_config(
        actuator, sensor, disturbance, scenario, fid)
    return plant, actuator, sensor, disturbance, scenario


def run_sr(row_dict, pid=None, adrc=None, seeds=None):
    """Return mean SR over seeds for given PID or ADRC params."""
    plant, actuator, sensor, disturbance, scenario = build_fidelity(row_dict)
    if seeds is None:
        seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    successes = sum(
        simulate(pid if pid else PIDParams(Kp=1.0, Kd=1.0),
                 plant, actuator, sensor, disturbance, scenario,
                 seed=s, adrc=adrc).success
        for s in seeds
    )
    return successes / len(seeds)


def sweep_one_design(row_dict):
    seeds  = list(range(SEED_START, SEED_START + N_SEEDS))
    rid    = row_dict.get('rocket_id', 'unknown')
    keff   = keff_full(row_dict)
    td     = keff * (float(row_dict['max_gimbal_deg']) * 12.0 / 15.0)
    lat    = int(row_dict['latency_steps'])
    Pi     = keff * lat ** 2   # CORRECTED: Pi = keff × tau² (not td × tau²; max_gimbal extraneous)
    best_Kd = max(float(row_dict.get('best_Kd', 1.0)), 0.1)

    # ── A. PD sweep (Ki=0) ────────────────────────────────────────────────────
    pd_results = []
    for Kp in KP_GRID:
        pid = PIDParams(Kp=float(Kp), Kd=best_Kd, Ki=0.0, u_max=U_MAX)
        sr  = run_sr(row_dict, pid=pid, seeds=seeds)
        pd_results.append(sr)
    peak_pd = max(pd_results)
    n_pass_pd = sum(1 for sr in pd_results if sr >= SR_PASS)

    # ── B. PID-I sweep (best Ki per design) ───────────────────────────────────
    pidi_best = 0.0
    pidi_n_pass = 0
    for Ki in KI_GRID:
        for Kp in KP_GRID:
            pid = PIDParams(Kp=float(Kp), Kd=best_Kd, Ki=float(Ki), u_max=U_MAX, i_lim=U_MAX)
            sr  = run_sr(row_dict, pid=pid, seeds=seeds)
            if sr > pidi_best:
                pidi_best = sr
            if sr >= SR_PASS:
                pidi_n_pass += 1
    peak_pidi = pidi_best

    # ── C. ADRC sweep ─────────────────────────────────────────────────────────
    adrc_results = []
    for wc in WC_GRID:
        adrc = ADRCParams(omega_c=float(wc), omega0=OMEGA0_OVER_WC * float(wc),
                          b0=keff, u_max=U_MAX)
        pid_dummy = PIDParams(Kp=1.0, Kd=1.0, u_max=U_MAX)
        sr = run_sr(row_dict, pid=pid_dummy, adrc=adrc, seeds=seeds)
        adrc_results.append(sr)
    peak_adrc = max(adrc_results)
    n_pass_adrc = sum(1 for sr in adrc_results if sr >= SR_PASS)

    return dict(
        rocket_id=rid,
        td=td, keff=keff, latency=lat, Pi=Pi,
        peak_pd=peak_pd, n_pass_pd=n_pass_pd,
        peak_pidi=peak_pidi, n_pass_pidi=pidi_n_pass,
        peak_adrc=peak_adrc, n_pass_adrc=n_pass_adrc,
        final_label=row_dict.get('final_label', 'UNKNOWN'),
    )


def main():
    # ── Load the 50 designs from the LQR/SMC test ─────────────────────────────
    smc = pd.read_csv(RESULTS / 'smc_controller_test_py.csv')
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    merged = smc.merge(pop, on='rocket_id', how='left')
    merged['Pi']     = merged['td_x'] * merged['latency'] ** 2
    merged['td']     = merged['td_x']
    merged['keff']   = merged['keff']
    # final_label from smc file
    if 'final_label_x' in merged.columns:
        merged['final_label'] = merged['final_label_x']

    print(f"Running observer universality test on {len(merged)} designs...")
    print(f"  PD:    {len(KP_GRID)} Kp values x {N_SEEDS} seeds = {len(KP_GRID)*N_SEEDS} sims/design")
    print(f"  PID-I: {len(KP_GRID)*len(KI_GRID)} combos x {N_SEEDS} seeds = {len(KP_GRID)*len(KI_GRID)*N_SEEDS} sims/design")
    print(f"  ADRC:  {len(WC_GRID)} wc values x {N_SEEDS} seeds = {len(WC_GRID)*N_SEEDS} sims/design")
    total = len(merged) * (len(KP_GRID) + len(KP_GRID)*len(KI_GRID) + len(WC_GRID)) * N_SEEDS
    print(f"  Total: ~{total:,} sims")

    rows_dicts = []
    for _, row in merged.iterrows():
        d = row.to_dict()
        rows_dicts.append(d)

    results_list = Parallel(n_jobs=-1, verbose=5)(
        delayed(sweep_one_design)(rd) for rd in rows_dicts
    )

    df = pd.DataFrame(results_list).sort_values('Pi')
    out_path = RESULTS / 'observer_universality_py.csv'
    df.to_csv(out_path, index=False)
    print(f"\nSaved -> {out_path}")

    # ── Compute Spearman rho for each architecture ─────────────────────────────
    log_Pi = np.log(df['Pi'])

    def rho_str(y_col):
        rho, p = stats.spearmanr(log_Pi, df[y_col])
        return f"rho={rho:.3f}, p={p:.2e}"

    print("\n" + "="*60)
    print("OBSERVER UNIVERSALITY RESULTS")
    print("="*60)
    print(f"n = {len(df)} designs, Pi range: {df['Pi'].min():.0f} - {df['Pi'].max():.0f}")
    print()
    print("Spearman rho(log_Pi, peak_SR):")
    print(f"  PD (Ki=0):     {rho_str('peak_pd')}")
    print(f"  PID-I (best):  {rho_str('peak_pidi')}")
    print(f"  ADRC (ESO):    {rho_str('peak_adrc')}")
    print()
    print("Spearman rho(log_Pi, n_pass):")
    print(f"  PD:     {rho_str('n_pass_pd')}")
    print(f"  PID-I:  {rho_str('n_pass_pidi')}")
    print(f"  ADRC:   {rho_str('n_pass_adrc')}")
    print()

    # High-Pi designs comparison
    high_pi = df[df['Pi'] > 3000]
    print(f"High-Pi designs (Pi > 3000, n={len(high_pi)}):")
    for _, r in high_pi.iterrows():
        print(f"  {r['rocket_id']}: Pi={r['Pi']:.0f}  "
              f"PD={r['peak_pd']:.2f}  PID-I={r['peak_pidi']:.2f}  ADRC={r['peak_adrc']:.2f}  "
              f"[{r['final_label']}]")

    print()
    print("INTERPRETATION:")
    rho_pd  = stats.spearmanr(log_Pi, df['peak_pd'])[0]
    rho_pi  = stats.spearmanr(log_Pi, df['peak_pidi'])[0]
    rho_a   = stats.spearmanr(log_Pi, df['peak_adrc'])[0]
    gap_pd_pi = abs(rho_pi - rho_pd)
    if gap_pd_pi < 0.05:
        print("  Integral action: DOES NOT break the Pi constraint (same rho as PD).")
        print("  ESO is specifically what escapes the constraint -- integral control is insufficient.")
        print("  Physical reason: in the saturation regime, integral winds up and anti-windup")
        print("  truncates it, preventing disturbance cancellation.")
    elif rho_pi > rho_pd + 0.1:
        print("  Integral action: PARTIALLY breaks the Pi constraint.")
        print("  ESO still achieves lower rho -- ESO is better but not uniquely special.")
    else:
        print(f"  Integral action marginal improvement: delta_rho = {rho_pi - rho_pd:+.3f}")

    adrc_vs_pd = abs(rho_a - rho_pd)
    if adrc_vs_pd > 0.2:
        print(f"  ADRC-ESO: breaks the constraint (rho {rho_a:.3f} vs PD {rho_pd:.3f}, delta {adrc_vs_pd:.3f})")
    else:
        print(f"  ADRC-ESO advantage smaller than expected: delta = {adrc_vs_pd:.3f}")

    print(f"\nConclusion: 'disturbance estimation' in the ESO sense is {'uniquely' if gap_pd_pi < 0.05 else 'partially'}")
    print("different from the classical integral-action approach to disturbance rejection.")


if __name__ == '__main__':
    main()
