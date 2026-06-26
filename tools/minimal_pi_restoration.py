#!/usr/bin/env python3
"""
tools/minimal_pi_restoration.py

Physics restoration study: add mechanisms back one at a time to the minimal simulator.

BACKGROUND
----------
minimal_pi_test.py showed:
  fixed wind:  floor ~ keff^(-1.2)   [static rejection regime]
  scaled wind: floor ~ keff^(0.0)    [proportional cancellation]
Neither reproduces full-sim floor ~ keff^(+1.0).

MECHANISM HYPOTHESIS
--------------------
The only way to shift the floor exponent from -1 → +1 without a keff^2 direct disturbance
is an ANGLE-PROPORTIONAL coupling that scales with keff:

    theta_ddot += keff x k_aero x theta           [aerodynamic instability]

In the bang-bang regime this creates self-amplification:
  bang-bang overshoot A_bb ~= keff x S x tau²      (slew-limited)
  aero disturbance d_aero = keff x k x A_bb
                          = keff^2 x k x S x tau²
  Kp_floor ~= d_aero / (keff x S x tau)
           = keff x k x tau                        → floor ∝ keff x latency ✓

The Smith predictor has theta_ddot_model = keff x u (no angle coupling), so its correction
diverges from reality at rate keff x k — explaining the keff=25 Smith collapse.

EXPERIMENTS
-----------
R1: angle-coupling sweep (aerodynamic instability):
    theta_ddot += keff x k_couple x theta
    k_couple in {0, 0.05, 0.10, 0.30, 0.50}
    keff in {5, 12, 25}, latency in {3, 6}
    → identifies threshold k that shifts floor exponent from -1 → +1

R2: direct wind scaling (no angle coupling):
    d_amp = c x keff^n, n in {1.0, 1.5, 2.0}
    keff in {5, 12, 25}, latency = 6
    → tests whether d ∝ keff^2 suffices WITHOUT angle coupling

R3: Smith predictor test at critical coupling from R1
    keff in {5, 12, 25}, latency in {3, 6}
    → tests whether angle coupling also creates Smith collapse

Decision table:
  R1 floor ~ keff^(+1) at some k: angle coupling IS the mechanism → R3 confirms Smith collapse
  R1 floor stays ≤ keff^(0): angle coupling alone insufficient → need dynamic pressure / other
  R2 floor ~ keff^(+1) at n=2: large direct wind scaling sufficient → aerodynamics not needed
  R3 Smith collapses: consistent with aero-coupling hypothesis
"""

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

# ── constants (matching minimal_pi_test.py) ──────────────────────────────
DT            = 0.005
T_END         = 3.0
U_MAX         = 12.0
SLEW_CU_PER_S = 200.0
GUST_TAU      = 1.0
SUCCESS_MAX_DEG = 30.0
SUCCESS_RMS_DEG = 15.0
BASE_WIND_AMP   = 3.0   # rad/s^2 baseline (fixed wind from R2 reference)
EVAL_SEEDS = list(range(60001, 60016))   # 15 seeds, disjoint from all prior


# ── minimal plant + restoration physics ──────────────────────────────────
def simulate_restore(keff, latency_steps, Kp, Kd, d_amp, seed,
                     k_couple=0.0, use_smith=False, keff_model=None):
    """
    Minimal plant with optional aerodynamic restoration:
        theta_ddot = keff * u + d_wind(t) + k_couple * keff * theta

    k_couple > 0: aerodynamic instability (positive coupling → theta grows)
    k_couple = 0: baseline minimal simulator
    keff_model: Smith predictor plant gain (oracle if None → use keff_actual)
    """
    N   = int(T_END / DT)
    rng = np.random.default_rng(seed)
    lat = int(latency_steps)
    keff_m = keff if keff_model is None else keff_model

    # state
    theta, q, u_act = 0.0, 0.0, 0.0

    # sensor delay ring buffer
    theta_buf = np.zeros(lat + 1)
    q_buf     = np.zeros(lat + 1)
    buf_ptr   = 0

    # OU wind
    gust_alpha = np.exp(-DT / GUST_TAU)
    gust_sig   = d_amp * np.sqrt(max(0.0, 1.0 - gust_alpha**2))
    gust = 0.0

    # Smith predictor
    sm_theta, sm_q = 0.0, 0.0
    sm_theta_buf = np.zeros(lat + 1)
    sm_q_buf     = np.zeros(lat + 1)
    sm_ptr = 0

    theta_hist = np.empty(N)
    max_theta  = 0.0

    for k in range(N):
        # OU wind
        gust = gust_alpha * gust + gust_sig * rng.standard_normal()

        # store in delay buffer
        theta_buf[buf_ptr] = theta
        q_buf[buf_ptr]     = q
        del_ptr   = (buf_ptr - lat) % (lat + 1)
        theta_del = theta_buf[del_ptr]
        q_del     = q_buf[del_ptr]

        # Smith predictor correction (delay-compensation only; no k_couple in model)
        if use_smith:
            sm_theta_buf[sm_ptr] = sm_theta
            sm_q_buf[sm_ptr]     = sm_q
            sm_del = (sm_ptr - lat) % (lat + 1)
            theta_ctrl = theta_del + (sm_theta - sm_theta_buf[sm_del])
            q_ctrl     = q_del     + (sm_q     - sm_q_buf[sm_del])
            sm_ptr = (sm_ptr + 1) % (lat + 1)
        else:
            theta_ctrl = theta_del
            q_ctrl     = q_del

        # PD controller
        u_cmd = np.clip(Kp * (-theta_ctrl) - Kd * q_ctrl, -U_MAX, U_MAX)

        # slew-limited actuator
        max_delta = SLEW_CU_PER_S * DT
        u_act = np.clip(u_cmd, u_act - max_delta, u_act + max_delta)

        # plant dynamics (with restoration term)
        aero_coupling = k_couple * keff * theta   # = 0 in baseline
        theta_new = theta + DT * q
        q_new     = q     + DT * (keff * u_act + gust + aero_coupling)
        theta = theta_new
        q     = q_new

        # Smith model update (NO coupling — the key divergence)
        if use_smith:
            sm_q_new     = sm_q     + DT * keff_m * u_act
            sm_theta_new = sm_theta + DT * sm_q
            sm_theta = sm_theta_new
            sm_q     = sm_q_new

        buf_ptr = (buf_ptr + 1) % (lat + 1)

        at = abs(theta)
        if at > max_theta:
            max_theta = at
        if max_theta > np.radians(70.0):
            return 0
        theta_hist[k] = theta

    rms = np.sqrt(np.mean(theta_hist[int(1.0 / DT):]**2))
    if max_theta > np.radians(SUCCESS_MAX_DEG) or rms > np.radians(SUCCESS_RMS_DEG):
        return 0
    return 1


def kp_sweep_restore(keff, latency, Kd, d_amp, k_couple,
                     use_smith=False, keff_model=None,
                     n_kp=24, kp_lo=0.5, kp_hi=500.0):
    kp_vals = np.geomspace(kp_lo, kp_hi, n_kp)
    return [(Kp, np.mean([simulate_restore(keff, latency, Kp, Kd, d_amp, s,
                                            k_couple, use_smith, keff_model)
                          for s in EVAL_SEEDS]))
            for Kp in kp_vals]


def find_window(kp_sr_list, threshold=0.80):
    passing = [kp for kp, sr in kp_sr_list if sr >= threshold]
    if not passing:
        return None, None
    return min(passing), max(passing)


def pick_kd_restore(keff, latency, d_amp, k_couple,
                    search_seeds=(1, 2, 3), Kp_ref=30.0):
    KD_GRID = [0.5, 1.0, 2.0, 4.0]
    best_kd, best_sr = 1.0, -1.0
    for Kd in KD_GRID:
        sr = np.mean([simulate_restore(keff, latency, Kp_ref, Kd, d_amp, s, k_couple)
                      for s in search_seeds])
        if sr > best_sr:
            best_sr = sr
            best_kd = Kd
    return best_kd


def run_combo_r1(keff, latency, k_couple, d_amp):
    """R1 and R3: angle coupling sweep + Smith test."""
    Kd = pick_kd_restore(keff, latency, d_amp, k_couple)
    pid_sweep  = kp_sweep_restore(keff, latency, Kd, d_amp, k_couple, use_smith=False)
    sm_sweep   = kp_sweep_restore(keff, latency, Kd, d_amp, k_couple, use_smith=True, keff_model=keff)
    pid_f, pid_c = find_window(pid_sweep)
    sm_f,  sm_c  = find_window(sm_sweep)
    return dict(
        keff=keff, latency=latency, k_couple=k_couple, Kd=Kd,
        d_amp=d_amp,
        pid_floor=pid_f, pid_ceil=pid_c,
        pid_window=(pid_c / pid_f) if (pid_f and pid_c) else None,
        smith_ceil=sm_c,
        smith_window=(sm_c / sm_f) if (sm_f and sm_c) else None,
        smith_ratio=(sm_c / pid_c) if (sm_c and pid_c) else None,
        pi_keff=keff * latency**2,
    )


def run_combo_r2(keff, latency, d_amp_scale):
    """R2: direct wind scaling d ∝ keff^n."""
    Kd = pick_kd_restore(keff, latency, d_amp_scale, k_couple=0.0)
    pid_sweep = kp_sweep_restore(keff, latency, Kd, d_amp_scale, k_couple=0.0)
    pid_f, pid_c = find_window(pid_sweep)
    return dict(
        keff=keff, latency=latency,
        d_amp=d_amp_scale,
        pid_floor=pid_f, pid_ceil=pid_c,
        pid_window=(pid_c / pid_f) if (pid_f and pid_c) else None,
        pi_keff=keff * latency**2,
    )


# ── main ──────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    KEFF_VALS   = [5.0, 12.0, 25.0]
    LAT_VALS    = [3, 6]
    D_AMP_FIXED = 3.0   # constant angular acceleration disturbance (rad/s^2)

    # R1/R3: angle coupling sweep
    # k_couple = 0 is the baseline (must reproduce minimal-sim result)
    # Positive k: aerodynamic instability (destabilizing, theta grows → more disturbance)
    # Negative k: aerodynamic stability (restoring, should REDUCE floor as control)
    K_COUPLE_VALS = [0.0, 0.05, 0.10, 0.30, 0.50]

    # R2: direct wind scaling d = BASE x keff^n
    # n=1.0: linear (already tested in minimal_pi_test scaled mode)
    # n=1.5: intermediate
    # n=2.0: quadratic (predicts floor ∝ keff^1 via static formula)
    WIND_SCALE_EXP = [1.0, 1.5, 2.0]

    print("=" * 70)
    print("RESTORATION STUDY: adding aerodynamic physics to minimal simulator")
    print("=" * 70)
    print(f"R1/R3: angle coupling k_couple in {K_COUPLE_VALS}")
    print(f"        keff in {KEFF_VALS},  latency in {LAT_VALS}")
    print(f"R2:    wind scaling n in {WIND_SCALE_EXP} (d = 3.0 x keff^n),  latency = 6")
    print()

    # ── R1 + R3: angle coupling ─────────────────────────────────────────
    print("Running R1+R3 (angle coupling sweep + Smith predictor) ...")
    r1_combos = [(keff, lat, kc, D_AMP_FIXED)
                 for keff in KEFF_VALS
                 for lat  in LAT_VALS
                 for kc   in K_COUPLE_VALS]

    r1_rows = Parallel(n_jobs=-1)(
        delayed(run_combo_r1)(keff, lat, kc, D_AMP_FIXED)
        for keff, lat, kc, _ in r1_combos
    )

    # ── R2: wind scaling ─────────────────────────────────────────────────
    print("Running R2 (direct wind magnitude scaling, latency=6) ...")
    r2_combos = [(keff, 6, 3.0 * keff**n)
                 for keff in KEFF_VALS
                 for n    in WIND_SCALE_EXP]

    r2_rows = Parallel(n_jobs=-1)(
        delayed(run_combo_r2)(keff, lat, d_amp)
        for keff, lat, d_amp in r2_combos
    )

    # save
    df_r1 = pd.DataFrame(r1_rows)
    df_r2 = pd.DataFrame(r2_rows)
    df_r1['n_wind'] = 'N/A (fixed wind)'
    df_r2['k_couple'] = 0.0
    df_r2['n_wind'] = [n for _ in KEFF_VALS for n in WIND_SCALE_EXP]

    out_r1 = 'experiments/results/minimal_pi_restoration_r1_py.csv'
    out_r2 = 'experiments/results/minimal_pi_restoration_r2_py.csv'
    df_r1.to_csv(out_r1, index=False)
    df_r2.to_csv(out_r2, index=False)
    print(f"\nSaved R1+R3: {len(df_r1)} rows -> {out_r1}")
    print(f"Saved R2:    {len(df_r2)} rows -> {out_r2}\n")

    # ── Report ─────────────────────────────────────────────────────────
    # R1: floor exponent on keff at each k_couple and latency
    print("=" * 70)
    print("R1: FLOOR EXPONENT ON keff  vs  k_couple  (theory target: +1.0)")
    print("=" * 70)
    for lat in LAT_VALS:
        print(f"\n  latency = {lat}:")
        print(f"  {'k_couple':>10}  {'slope(keff)':>12}  {'evidence'}")
        print(f"  {'-'*50}")
        for kc in K_COUPLE_VALS:
            sub = [r for r in r1_rows if r['latency'] == lat
                   and abs(r['k_couple'] - kc) < 1e-9 and r['pid_floor']]
            if len(sub) >= 2:
                xs = [r['keff'] for r in sub]
                ys = [r['pid_floor'] for r in sub]
                slope = np.polyfit(np.log(xs), np.log(ys), 1)[0]
                direction = ("INCREASING UP" if slope > 0.5
                             else "DECREASING DOWN" if slope < -0.3
                             else "FLAT ~=")
                print(f"  {kc:>10.2f}  {slope:>12.2f}  {direction}")
            else:
                print(f"  {kc:>10.2f}  {'N/A (infeasible)':>12}")

    # R1 raw floor table
    print("\n" + "=" * 70)
    print("R1: FLOOR TABLE  (fixed wind = 3.0 rad/s²)")
    print("=" * 70)
    print(f"{'keff':>5} {'lat':>4} {'k_couple':>9}  {'floor':>8} {'ceil':>7} "
          f"{'window':>8}  {'Sm_ceil':>8} {'Sm_ratio':>9}")
    print("-" * 70)
    for r in sorted(r1_rows, key=lambda x: (x['latency'], x['keff'], x['k_couple'])):
        fl = f"{r['pid_floor']:.2f}"  if r['pid_floor']  else 'INFEAS'
        ce = f"{r['pid_ceil']:.1f}"   if r['pid_ceil']   else 'INFEAS'
        pw = f"{r['pid_window']:.1f}x" if r['pid_window'] else 'N/A'
        sc = f"{r['smith_ceil']:.1f}" if r['smith_ceil'] else 'INFEAS'
        sr = f"{r['smith_ratio']:.2f}" if r['smith_ratio'] else 'N/A'
        print(f"{r['keff']:>5.0f} {r['latency']:>4} {r['k_couple']:>9.2f}  "
              f"{fl:>8} {ce:>7} {pw:>8}  {sc:>8} {sr:>9}")

    # R1: Smith regime shift
    print("\n" + "=" * 70)
    print("R1+R3: SMITH PREDICTOR RATIO vs k_couple  (collapse threshold?)")
    print("       ratio > 2.0 = delay-limited   ratio < 1.0 = Smith fails")
    print("=" * 70)
    for lat in LAT_VALS:
        for keff in KEFF_VALS:
            row_line = f"  keff={keff:.0f} lat={lat}: "
            parts = []
            for kc in K_COUPLE_VALS:
                r = next((x for x in r1_rows
                          if abs(x['keff']-keff)<0.1 and x['latency']==lat
                          and abs(x['k_couple']-kc)<1e-9), None)
                if r and r['smith_ratio']:
                    parts.append(f"k={kc:.2f}→{r['smith_ratio']:.2f}")
                else:
                    parts.append(f"k={kc:.2f}→N/A")
            print(row_line + "  |  ".join(parts))

    # R2: floor exponent with wind scaling
    print("\n" + "=" * 70)
    print("R2: FLOOR vs WIND SCALING EXPONENT n  (d = 3.0 x keff^n, lat=6)")
    print("    Theory: d∝keff^n → static floor∝keff^(n-1)")
    print("=" * 70)
    print(f"{'n':>5}  {'keff':>5}  {'d_amp':>7}  {'floor':>8}  predicted by static formula")
    print("-" * 55)
    for n in WIND_SCALE_EXP:
        sub = [r for r in r2_rows if abs(r['d_amp'] / r['keff']**n - 3.0) < 0.5]
        for r in sorted(sub, key=lambda x: x['keff']):
            fl = f"{r['pid_floor']:.2f}" if r['pid_floor'] else 'INFEAS'
            pred = f"∝keff^{n-1:.1f}" if r['pid_floor'] else ''
            print(f"{n:>5.1f}  {r['keff']:>5.0f}  {r['d_amp']:>7.1f}  {fl:>8}  {pred}")
        # regression
        pts = [(r['keff'], r['pid_floor']) for r in r2_rows
               if abs(r['d_amp']/r['keff']**n - 3.0) < 0.5 and r['pid_floor']]
        if len(pts) >= 2:
            xs, ys = zip(*pts)
            sl = np.polyfit(np.log(xs), np.log(ys), 1)[0]
            print(f"         → measured slope = {sl:.2f}  (expected {n-1:.1f})")
        print()

    # ── DECISION SUMMARY ────────────────────────────────────────────────
    print("=" * 70)
    print("DECISION TABLE: which mechanism creates floor ∝ keff x latency?")
    print("=" * 70)

    # Find k_couple where floor exponent first crosses 0
    print("\nR1 floor exponent vs k_couple at latency=6:")
    for kc in K_COUPLE_VALS:
        sub = [r for r in r1_rows if r['latency'] == 6
               and abs(r['k_couple'] - kc) < 1e-9 and r['pid_floor']]
        if len(sub) >= 2:
            xs = [r['keff'] for r in sub]
            ys = [r['pid_floor'] for r in sub]
            slope = np.polyfit(np.log(xs), np.log(ys), 1)[0]
            mechanism = ""
            if slope > 0.7:
                mechanism = " <- floor ∝ keff^(+1) ACHIEVED"
            elif slope > 0.0:
                mechanism = " <- floor rising with keff"
            print(f"  k_couple={kc:.2f}: floor ~ keff^{slope:.2f}{mechanism}")

    # Check R2 vs R1
    print("\nR2 wind scaling (no angle coupling, latency=6):")
    for n in WIND_SCALE_EXP:
        pts = [(r['keff'], r['pid_floor']) for r in r2_rows
               if abs(r['d_amp']/r['keff']**n - 3.0) < 0.5 and r['pid_floor']]
        if len(pts) >= 2:
            xs, ys = zip(*pts)
            sl = np.polyfit(np.log(xs), np.log(ys), 1)[0]
            print(f"  n={n:.1f}: floor ~ keff^{sl:.2f}  "
                  f"(theory {n-1:.1f} from static formula)")

    print()
    print("CONCLUSION:")
    r1_slopes_lat6 = []
    for kc in K_COUPLE_VALS:
        sub = [r for r in r1_rows if r['latency'] == 6
               and abs(r['k_couple'] - kc) < 1e-9 and r['pid_floor']]
        if len(sub) >= 2:
            xs = [r['keff'] for r in sub]
            ys = [r['pid_floor'] for r in sub]
            sl = np.polyfit(np.log(xs), np.log(ys), 1)[0]
            r1_slopes_lat6.append((kc, sl))

    r2_slope_n2 = None
    pts = [(r['keff'], r['pid_floor']) for r in r2_rows
           if abs(r['d_amp']/r['keff']**2 - 3.0) < 0.5 and r['pid_floor']]
    if len(pts) >= 2:
        xs, ys = zip(*pts)
        r2_slope_n2 = np.polyfit(np.log(xs), np.log(ys), 1)[0]

    angle_coupling_creates_floor = any(sl > 0.7 for _, sl in r1_slopes_lat6)
    wind_scaling_creates_floor   = (r2_slope_n2 is not None and r2_slope_n2 > 0.7)

    if angle_coupling_creates_floor and not wind_scaling_creates_floor:
        print("  MECHANISM: angle-proportional coupling (aerodynamic instability x keff)")
        print("  IMPLICATION: Smith collapse is predicted — see R3 Smith ratios above")
    elif wind_scaling_creates_floor:
        print("  MECHANISM: direct wind magnitude scaling (d ∝ keff^2) is sufficient")
        print("  IMPLICATION: angle coupling is not required; inertia_scale^2 could explain it")
    elif angle_coupling_creates_floor and wind_scaling_creates_floor:
        print("  BOTH mechanisms work independently")
    else:
        print("  NEITHER tested mechanism creates floor ∝ keff^(+1) — more physics needed")
        print("  Candidates: dynamic pressure x angle, nonlinear aero, thrust-aero coupling")
