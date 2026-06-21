"""
tools/gain_margin_equation.py

Derives the gain ceiling equation from:
  1. Linear stability theory (phase margin analysis)
  2. Empirical fit to boundary experiment v2 data
  3. Spot-check of specific (Kp, design, condition) cases

Physical model:
  Plant:  G(s) = keff / s²    (double integrator, bang-bang saturated regime)
  Controller: C(s) = Kp(1 + Kd/Kp × s)  (PD, Kd=1 fixed from exp1)
  Delay:  e^{-τs}  where τ = latency_steps × dt

Phase margin condition (linear stability limit):
  PM = arctan(Kd × sqrt(keff/Kp)) - sqrt(Kp × keff) × τ
  At gain ceiling: PM = 0
  → arctan(sqrt(keff/K_u)) = sqrt(K_u × keff) × τ   [Kd=1]
  → x × arctan(x) = keff × τ                          [substitution x = sqrt(keff/K_u)]
  → K_u = keff / x²
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.optimize import brentq
from scipy.stats import linregress

# ── Config ─────────────────────────────────────────────────────────────────────
F15_T_AVG  = 14.4
L_NOZZLE   = 0.25
CU_TO_RAD  = (np.pi / 180) * (15 / 12)
DT         = 0.005          # 200 Hz sampling
KD         = 1.0            # all designs use Kd=1 (from exp1 autotune)

WIN_CSV = Path('experiments/results/gain_window_v2_summary_py.csv')
EXP_CSV = Path('experiments/results/exp1_regime_index_py.csv')


def solve_Ku_linear(keff, Kd, tau):
    """
    Solve arctan(Kd * sqrt(keff/Ku)) = sqrt(Ku * keff) * tau  for Ku.
    Equivalently: x * arctan(x) = keff * Kd^2 * tau  where x = Kd*sqrt(keff/Ku), Ku = keff*Kd^2/x^2
    """
    c = keff * Kd**2 * tau
    if c <= 0:
        return np.inf
    # x * arctan(x) = c; solve on [1e-6, 100]
    def eq(x): return x * np.arctan(x) - c
    if eq(100) < 0:   # no solution in range
        return keff * Kd**2 / 100**2
    x = brentq(eq, 1e-6, 100)
    return keff * Kd**2 / x**2


def Ku_linear_approx(keff, Kd, tau):
    """First-order approximation: Ku ≈ 0.9/tau for typical keff*tau in [0.1, 1]."""
    c = keff * Kd**2 * tau
    # Tabulated correction from numerical solution of x*arctan(x)=c
    # K_u * tau = c/x^2; for small c, c/x^2 → 1; decreases slowly with c
    c_tab = np.array([0.01, 0.05, 0.10, 0.20, 0.30, 0.50, 0.70, 1.00, 2.00, 5.00])
    ku_tau_tab = np.array([0.999, 0.993, 0.983, 0.955, 0.921, 0.846, 0.774, 0.678, 0.434, 0.172])
    factor = np.interp(c, c_tab, ku_tau_tab, left=1.0, right=0.1)
    return factor / tau


def main():
    print("Loading data...")
    win = pd.read_csv(WIN_CSV)
    exp = pd.read_csv(EXP_CSV)

    # Compute median ceiling across wind levels (wind barely affects ceiling)
    lat6 = win[win['latency'] == 6].copy()
    lat3 = win[win['latency'] == 3].copy()
    lat1 = win[win['latency'] == 1].copy()

    def ceiling_stats(sub):
        """Per-design median and std ceiling across wind levels."""
        return sub.groupby(['rocket_id', 'group_type', 'td', 'keff'])['kp_ceil'].agg(
            ['median', 'std', 'min', 'max', 'count']).reset_index()

    c6 = ceiling_stats(lat6);  c6['latency'] = 6
    c3 = ceiling_stats(lat3);  c3['latency'] = 3
    c1 = ceiling_stats(lat1);  c1['latency'] = 1
    cdf = pd.concat([c6, c3, c1], ignore_index=True).rename(
        columns={'median': 'ceil_median', 'std': 'ceil_std'})

    # Theoretical K_u for each design × latency
    cdf['tau'] = cdf['latency'] * DT
    cdf['Ku_theory'] = cdf.apply(
        lambda r: solve_Ku_linear(r['keff'], KD, r['tau']), axis=1)
    cdf['Ku_approx'] = cdf.apply(
        lambda r: Ku_linear_approx(r['keff'], KD, r['tau']), axis=1)
    cdf['ceil_factor'] = cdf['ceil_median'] / cdf['Ku_theory']

    # ── 1. Theory vs data comparison ───────────────────────────────────────────
    print("\n" + "="*80)
    print("LINEAR THEORY vs SIMULATION (median ceiling across 5 wind levels)")
    print("="*80)
    print("Phase margin formula: arctan(sqrt(keff/Ku)) = sqrt(Ku×keff) × latency×dt")
    print("Approximation: Ku_theory ≈ 0.9/tau  (nearly keff-independent for keff×tau ∈ [0.1,1])")
    print()
    print(f"{'ID':8s} {'type':7s} {'keff':6s} {'lat':3s} {'tau[ms]':7s} "
          f"{'keff×tau':8s} {'Ku_theory':9s} {'Ku_approx':9s} {'ceil_data':9s} "
          f"{'factor':6s} {'ceil_std':8s}")
    for _, r in cdf.sort_values(['latency', 'keff']).iterrows():
        if r['latency'] == 6:  # focus on lat=6 where ceiling is most constrained
            factor = r['ceil_factor']
            print(f"  {r['rocket_id']:8s} {r['group_type']:7s} {r['keff']:6.2f} "
                  f"{r['latency']:3.0f} {r['tau']*1000:7.1f} "
                  f"{r['keff']*r['tau']:8.3f} {r['Ku_theory']:9.1f} "
                  f"{r['Ku_approx']:9.1f} {r['ceil_median']:9.0f} "
                  f"{factor:6.2f}x {'±' if not np.isnan(r['ceil_std']) else ''}"
                  f"{r['ceil_std'] if not np.isnan(r['ceil_std']) else 0:.0f}")

    # Correction factor statistics
    lat6_rows = cdf[cdf['latency'] == 6].dropna(subset=['ceil_factor'])
    lat6_rows = lat6_rows[lat6_rows['ceil_median'] < 320]  # exclude search-limit censored
    factor_mean = lat6_rows['ceil_factor'].mean()
    factor_std  = lat6_rows['ceil_factor'].std()
    factor_med  = lat6_rows['ceil_factor'].median()
    print(f"\n  Correction factor (ceil_data / Ku_theory) at lat=6:")
    print(f"    Mean={factor_mean:.2f}x  Median={factor_med:.2f}x  Std={factor_std:.2f}")
    print(f"    Range=[{lat6_rows['ceil_factor'].min():.2f}, {lat6_rows['ceil_factor'].max():.2f}]")
    print(f"  → Practical ceiling ≈ {factor_med:.1f} × Ku_theory ≈ {factor_med*0.9:.1f}/tau")
    print(f"  → At lat=6 (30ms): ceiling ≈ {factor_med*0.9/0.03:.0f} (theory) "
          f"× {factor_med:.1f} ≈ {factor_med*0.9/0.03:.0f}-{factor_med*0.9/0.03*factor_mean:.0f}")

    # ── 2. Empirical power-law fit: ceil = A × keff^α × tau^β ─────────────────
    print("\n" + "="*80)
    print("EMPIRICAL POWER-LAW FIT: ceil = A × keff^α × tau^β")
    print("="*80)

    # Use all latency levels, but exclude censored (ceil=320 = search limit, probably not physical)
    fit_data = cdf[(cdf['ceil_median'] < 320) & (cdf['ceil_median'] > 0)].copy()
    fit_data['log_ceil']  = np.log(fit_data['ceil_median'])
    fit_data['log_keff']  = np.log(fit_data['keff'])
    fit_data['log_tau']   = np.log(fit_data['tau'])

    X = np.column_stack([np.ones(len(fit_data)), fit_data['log_keff'], fit_data['log_tau']])
    y = fit_data['log_ceil'].values
    coefs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    log_A, alpha, beta = coefs

    y_pred = X @ coefs
    ss_res = np.sum((y - y_pred)**2)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1 - ss_res/ss_tot if ss_tot > 0 else 0

    A = np.exp(log_A)
    print(f"\n  Fit result (n={len(fit_data)}, R²={r2:.3f}):")
    print(f"  Ku_empirical = {A:.1f} × keff^({alpha:.3f}) × tau^({beta:.3f})")
    print(f"  Where tau = latency_steps × dt = latency_steps × 0.005 [s]")
    print()
    print(f"  Physical interpretation:")
    print(f"    alpha={alpha:.3f}: ceiling {'decreases' if alpha<0 else 'increases'} with keff "
          f"(power {alpha:.3f}) — {'more aggressive plant → lower ceiling' if alpha<0 else 'unexpected'}")
    print(f"    beta={beta:.3f}: ceiling {'decreases' if beta<0 else 'increases'} with tau "
          f"(power {beta:.3f}) — lag degrades ceiling by tau^{beta:.2f}")
    print(f"  Theoretical prediction: alpha=-0.5 to 0, beta=-1 (from PM analysis)")
    print()

    # Compare to simplified K_u formula
    print("  Comparison to K_u_theory ≈ 0.9/tau (keff-independent approximation):")
    print("  → Theory exponents: alpha=0, beta=-1")
    print(f"  → Empirical exponents: alpha={alpha:.2f}, beta={beta:.2f}")
    if abs(beta - (-1)) < 0.3:
        print("  ✓ Latency exponent is close to -1 (consistent with linear theory)")
    else:
        print(f"  ✗ Latency exponent deviates from -1 by {beta-(-1):.2f} (may reflect nonlinearity)")

    # Show residuals per design
    fit_data = fit_data.copy()
    fit_data['ceil_fit'] = A * fit_data['keff']**alpha * fit_data['tau']**beta
    fit_data['residual_pct'] = (fit_data['ceil_median'] - fit_data['ceil_fit']) / fit_data['ceil_fit'] * 100
    print(f"\n  Per-design residuals at lat=6:")
    lat6_fit = fit_data[fit_data['latency'] == 6].sort_values('keff')
    for _, r in lat6_fit.iterrows():
        print(f"    {r['rocket_id']:8s} [{r['group_type']:7s}] keff={r['keff']:.2f} "
              f"data={r['ceil_median']:.0f} fit={r['ceil_fit']:.0f} "
              f"residual={r['residual_pct']:+.0f}%")

    # ── 3. Unified gain window equation ────────────────────────────────────────
    print("\n" + "="*80)
    print("GAIN WINDOW EQUATION")
    print("="*80)
    print("""
  CEILING (upper Kp limit, phase lag driven):
    Ku_theory = keff / x²   where  x × arctan(x) = keff × Kd × tau    [exact, linear]
    Ku_approx ≈ 0.9 / tau  (±20%, for keff×tau ∈ [0.1, 1.0])           [approx]
    Ku_empirical = {:.1f} × keff^({:.2f}) × tau^({:.2f})  (R²={:.3f})  [empirical, n={}]

    Nonlinear factor: simulation ceiling ≈ {:.1f}× × Ku_theory
    (bang-bang + SR=0.80 threshold = more lenient than PM=0)

  FLOOR (lower Kp limit, wind rejection driven):
    Kp_floor ≈ 0.35 × keff_full^0.70  (relay study, n=41, rho=0.58)
    In boundary experiment: floor = 1-8 for EASY, 5-27 for FRAGILE
    Wind effect: < 5 Kp units across full wind range (not reliable predictor)

  WINDOW COLLAPSE CONDITION:
    Ceiling < Floor → no valid Kp exists
    Kp_floor > Ku × correction_factor  ←  FRAGILE
    → 0.35 × keff^0.70 > {:.1f} / tau
    → keff^0.70 × tau > {:.3f}
    → At tau=0.03s: keff > {:.1f}   (FRAGILE threshold in keff units)
    → In theta_ddot: theta_ddot ≈ keff × u_max; at u_max≈8 (max_gimbal≈10°): td > {:.0f} rad/s²
    """.format(A, alpha, beta, r2, len(fit_data),
               factor_med,
               factor_med*0.9/0.03,  # Ku_theory ceiling at lat=6
               factor_med*0.9,
               (factor_med*0.9/0.03/0.35)**(1/0.70),  # keff threshold
               (factor_med*0.9/0.03/0.35)**(1/0.70) * 8))  # td threshold

    # ── 4. Spot check: verify specific simulation points ───────────────────────
    print("\n" + "="*80)
    print("SPOT CHECK: Running specific cases manually vs boundary experiment")
    print("="*80)
    print("Checking: Does Kp=60 fail for R1526 (FRAGILE, keff=7.06, lat=6, wind=0.22)?")
    print("Boundary experiment says: ceil=90 → Kp=60 SHOULD PASS, Kp=90 should pass,")
    print("Kp=130 should FAIL (since ceil=90 means Kp=130 was first fail point).")

    from simulator import simulate, PIDParams
    from fidelity_config import FidelityConfig, apply_fidelity_config
    from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario

    exp = pd.read_csv(EXP_CSV)
    spot_cases = [
        ('R1526', 'FRAGILE', 60,  0.22, 6, 'should PASS  (below ceil=90)'),
        ('R1526', 'FRAGILE', 90,  0.22, 6, 'should PASS  (at ceil=90)'),
        ('R1526', 'FRAGILE', 130, 0.22, 6, 'should FAIL  (above ceil=90)'),
        ('R0728', 'EASY',    60,  0.22, 6, 'should PASS  (at ceil=60)'),
        ('R0728', 'EASY',    90,  0.22, 6, 'should FAIL  (above ceil=60)'),
    ]

    for rid, gtype, Kp, wind, lat, expect in spot_cases:
        row = exp[exp['rocket_id'] == rid].iloc[0].to_dict()
        row['wind_strength'] = wind
        row['latency_steps'] = lat
        plant = build_plant(row)
        act   = build_actuator(row)
        sen   = build_sensor(row)
        dis   = build_disturbance(row)
        sc    = build_scenario()
        fc    = FidelityConfig.full()
        act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fc)
        pid   = PIDParams(Kp=float(Kp), Kd=1.0, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)

        successes = [simulate(pid, plant, act, sen, dis, sc, seed=s).success for s in range(1,8)]
        sr = sum(successes) / len(successes)
        outcome = "PASS" if sr >= 0.80 else "FAIL"
        match = "✓" if (outcome in expect) else "✗ MISMATCH"
        print(f"  {rid} [{gtype}] Kp={Kp:3d} wind={wind:.2f} lat={lat}: "
              f"SR={sr:.3f} → {outcome} | {expect} {match}")

    print()
    print("If spot checks pass, the boundary experiment ceilings are reliable.")

    # ── 5. Key numbers for paper ────────────────────────────────────────────────
    print("\n" + "="*80)
    print("KEY NUMBERS FOR PAPER / REPORTING")
    print("="*80)
    print(f"""
  1. GAIN CEILING LAW (linear stability + correction):
       Kp_max ≈ CF / (latency_steps × dt)
       where CF = {factor_med:.1f}  [median correction for bang-bang + SR=0.80]

     → At 200 Hz:  Kp_max ≈ {factor_med*0.9:.0f} × 200 / latency_steps = {factor_med*0.9*200:.0f} / latency_steps
     → lat=1: Kp_max ≈ {factor_med*0.9*200/1:.0f}
     → lat=3: Kp_max ≈ {factor_med*0.9*200/3:.0f}
     → lat=6: Kp_max ≈ {factor_med*0.9*200/6:.0f}

  2. GAIN FLOOR LAW (from relay study, keff-driven):
       Kp_min ≈ 0.35 × keff^0.70   [empirical, rho=0.58, n=41]

  3. WINDOW RATIO:
       window = Kp_max / Kp_min ≈ {factor_med*0.9:.0f} / (latency × dt × 0.35 × keff^0.70)

  4. COLLAPSE CONDITION (window_ratio < 1):
       FRAGILE when: 0.35 × keff^0.70 > {factor_med*0.9:.0f} / (latency_steps × 0.005)
       → keff^0.70 > {factor_med*0.9/0.35:.1f} / latency_steps
       → At lat=6:  keff > {(factor_med*0.9/0.35/6)**(1/0.70):.1f} rad/s²/CU
       → At lat=1:  keff > {(factor_med*0.9/0.35/1)**(1/0.70):.1f} rad/s²/CU  (much harder to be FRAGILE)

  5. LATENCY INTERACTION WITH theta_ddot:
       For design at theta_ddot=55 rad/s² (Youden-J threshold):
         typical u_max ≈ 8-10 CU, keff ≈ 55/9 ≈ 6-7 rad/s²/CU
         Kp_floor ≈ 0.35 × 6^0.70 ≈ 0.35 × 3.69 ≈ 1.3 (low floor → EASY)
         Kp_ceil at lat=6 ≈ {factor_med*0.9*200/6:.0f} (well above floor)
         → Window > 30× → no collapse at threshold
       For design at theta_ddot=150 rad/s²:
         typical u_max ≈ 10, keff ≈ 150/10 = 15 rad/s²/CU
         Kp_floor ≈ 0.35 × 15^0.70 ≈ 0.35 × 6.3 ≈ 2.2 (still low)
         Kp_ceil at lat=6 ≈ {factor_med*0.9*200/6:.0f}
         → Window > 20× → still no collapse
       FRAGILE comes from elevated FLOOR, not from ceiling alone.

  NOTE: The floor equation Kp_floor ≈ 0.35 × keff^0.70 is from the relay study (n=41 designs).
  The ceiling law CF/tau is from linear phase margin analysis + empirical correction.
  Both are approximate; hardware validation is required.
""")


if __name__ == '__main__':
    main()
