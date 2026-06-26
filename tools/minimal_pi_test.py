#!/usr/bin/env python3
"""
tools/minimal_pi_test.py

Stripped-down simulator for Pi constraint reproducibility test.

Plant:  theta_ddot = keff * u + d(t)
Slew:   |du/dt| <= S  (CU/s)
Delay:  controller reads theta[k - latency_steps]
Wind:   OU colored noise, angular acceleration disturbance
PD:     u = Kp*(-theta_delayed) - Kd*(q_delayed)
Smith:  delay-compensating predictor using exact keff model, no wind

REMOVED vs full simulator:
  - aerodynamics (static_margin, Cm_alpha, dyn_aero, nonlinear_aero)
  - thrust curve variation
  - CG shift
  - sensor noise
  - backlash
  - deadband
  - mass/inertia variation
  - motor model

Two wind modes:
  fixed  -- d(t) amplitude independent of keff (isolates bang-bang mechanism)
  scaled -- d(t) amplitude proportional to keff (reproduces full-sim behavior)

Comparing floor-keff scaling between modes directly tests Artifact A1
(inertia_scale coupling in the full simulator).
"""

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

# ── simulation constants ────────────────────────────────────────────────────
DT = 0.005          # 200 Hz, matching full simulator
T_END = 3.0
U_MAX = 12.0        # control unit ceiling (matching full sim u_max for max_gimbal≈10)
SLEW_CU_PER_S = 200.0   # servo slew in CU/s (S·tau/u_max ≈ 0.25 at lat=3, 0.5 at lat=6)
GUST_TAU = 1.0      # OU correlation time, seconds
SUCCESS_MAX_DEG = 30.0
SUCCESS_RMS_DEG = 15.0
EVAL_SEEDS = list(range(50001, 50021))   # 20 seeds, disjoint from all prior experiments


# ── minimal plant simulation ───────────────────────────────────────────────
def simulate_minimal(keff, latency_steps, Kp, Kd, d_amp, seed,
                     use_smith=False, keff_model=None):
    """
    Single trajectory. Returns 1 (pass) or 0 (fail).
    d_amp: wind disturbance amplitude in rad/s^2 (already scaled by caller)
    keff_model: Smith predictor plant gain (None -> use keff)
    """
    N = int(T_END / DT)
    rng = np.random.default_rng(seed)

    lat = int(latency_steps)
    keff_m = keff if keff_model is None else keff_model

    # state
    theta = 0.0
    q = 0.0
    u_act = 0.0

    # sensor delay buffers (size lat+1, ring-buffer)
    theta_buf = np.zeros(lat + 1)
    q_buf     = np.zeros(lat + 1)
    buf_ptr   = 0

    # OU wind
    gust_alpha = np.exp(-DT / GUST_TAU)
    gust_sig   = d_amp * np.sqrt(max(0.0, 1.0 - gust_alpha**2))
    gust = 0.0

    # Smith predictor state
    sm_theta = 0.0
    sm_q     = 0.0
    sm_theta_buf = np.zeros(lat + 1)
    sm_q_buf     = np.zeros(lat + 1)
    sm_ptr       = 0

    theta_hist = np.empty(N)
    max_theta  = 0.0

    for k in range(N):
        # --- wind update ---
        gust = gust_alpha * gust + gust_sig * rng.standard_normal()

        # --- store current state in delay buffer ---
        theta_buf[buf_ptr] = theta
        q_buf[buf_ptr]     = q

        # --- read delayed state ---
        del_ptr   = (buf_ptr - lat) % (lat + 1)
        theta_del = theta_buf[del_ptr]
        q_del     = q_buf[del_ptr]

        # --- Smith predictor correction ---
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

        # --- PD controller ---
        u_cmd = Kp * (-theta_ctrl) - Kd * q_ctrl
        u_cmd = np.clip(u_cmd, -U_MAX, U_MAX)

        # --- slew-limited actuator ---
        max_delta = SLEW_CU_PER_S * DT
        u_act = np.clip(u_cmd, u_act - max_delta, u_act + max_delta)

        # --- plant ---
        theta_new = theta + DT * q
        q_new     = q     + DT * (keff * u_act + gust)
        theta = theta_new
        q     = q_new

        # --- Smith model update (disturbance-free) ---
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


# ── Kp sweep helpers ───────────────────────────────────────────────────────
def kp_sweep(keff, latency, Kd, d_amp, use_smith=False, keff_model=None,
             n_kp=24, kp_lo=0.5, kp_hi=500.0):
    """Returns sorted list of (Kp, mean_sr)."""
    kp_vals = np.geomspace(kp_lo, kp_hi, n_kp)
    out = []
    for Kp in kp_vals:
        sr = np.mean([
            simulate_minimal(keff, latency, Kp, Kd, d_amp, s, use_smith, keff_model)
            for s in EVAL_SEEDS
        ])
        out.append((Kp, sr))
    return out


def find_window(kp_sr_list, threshold=0.80):
    """Floor = min passing Kp, ceiling = max passing Kp."""
    passing = [kp for kp, sr in kp_sr_list if sr >= threshold]
    if not passing:
        return None, None
    return min(passing), max(passing)


# ── Kd selection (tiny 4-point grid at mid Kp) ────────────────────────────
def pick_kd(keff, latency, d_amp, search_seeds=(1, 2, 3)):
    """Choose best Kd from [0.5, 1.0, 2.0, 4.0] by SR at Kp=30."""
    KD_GRID = [0.5, 1.0, 2.0, 4.0]
    Kp_ref = 30.0
    best_kd, best_sr = 1.0, -1.0
    for Kd in KD_GRID:
        sr = np.mean([simulate_minimal(keff, latency, Kp_ref, Kd, d_amp, s)
                      for s in search_seeds])
        if sr > best_sr:
            best_sr = sr
            best_kd = Kd
    return best_kd


# ── per-design runner (parallelised) ──────────────────────────────────────
def run_design(keff, latency, d_amp, wind_label):
    """Run one (keff, latency) combination, return result dict."""
    row = dict(keff=keff, latency=latency, wind_mode=wind_label, d_amp=d_amp)

    Kd = pick_kd(keff, latency, d_amp)
    row['Kd'] = Kd

    # PID sweep
    pid_sweep  = kp_sweep(keff, latency, Kd, d_amp, use_smith=False)
    pid_floor, pid_ceil = find_window(pid_sweep)
    row['pid_floor'] = pid_floor
    row['pid_ceil']  = pid_ceil
    row['pid_window'] = (pid_ceil / pid_floor) if (pid_floor and pid_ceil) else None

    # Smith predictor (oracle keff_model = exact)
    smith_sweep = kp_sweep(keff, latency, Kd, d_amp, use_smith=True, keff_model=keff)
    smith_floor, smith_ceil = find_window(smith_sweep)
    row['smith_floor'] = smith_floor
    row['smith_ceil']  = smith_ceil
    row['smith_window'] = (smith_ceil / smith_floor) if (smith_floor and smith_ceil) else None
    row['smith_ceil_ratio'] = (smith_ceil / pid_ceil) if (pid_ceil and smith_ceil) else None

    # Pi
    row['pi_keff'] = keff * latency**2

    return row


# ── main ───────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    KEFF_VALS = [5.0, 8.0, 12.0, 18.0, 25.0]
    LAT_VALS  = [2, 3, 4, 6]

    # Fixed wind: 3.0 rad/s² for ALL keff (isolates bang-bang from wind-scaling artifact)
    # Scaled wind: 0.3 × keff rad/s² (replicates full-sim inertia_scale behavior)
    WIND_FIXED_AMP  = 3.0    # rad/s², constant
    WIND_SCALE_COEF = 0.3    # dimensionless; d_amp = coef × keff

    all_combos = [(keff, lat) for keff in KEFF_VALS for lat in LAT_VALS]
    print(f"Minimal Pi Test  ({len(all_combos)} designs × 2 wind modes)")
    print(f"  keff: {KEFF_VALS}")
    print(f"  lat:  {LAT_VALS}")
    print(f"  slew: {SLEW_CU_PER_S} CU/s  u_max={U_MAX}")
    print(f"  S·tau/u_max at lat=3: {SLEW_CU_PER_S*3*DT/U_MAX:.3f}")
    print(f"  S·tau/u_max at lat=6: {SLEW_CU_PER_S*6*DT/U_MAX:.3f}")
    print()

    # --- wind=FIXED mode ---
    print("Wind mode: FIXED (d_amp independent of keff)")
    rows_fixed = Parallel(n_jobs=-1)(
        delayed(run_design)(keff, lat, WIND_FIXED_AMP, 'fixed')
        for keff, lat in all_combos
    )

    # --- wind=SCALED mode ---
    print("Wind mode: SCALED (d_amp = 0.3 × keff)")
    rows_scaled = Parallel(n_jobs=-1)(
        delayed(run_design)(keff, lat, WIND_SCALE_COEF * keff, 'scaled')
        for keff, lat in all_combos
    )

    all_rows = rows_fixed + rows_scaled
    df = pd.DataFrame(all_rows)
    out = 'experiments/results/minimal_pi_test_py.csv'
    df.to_csv(out, index=False)
    print(f"\nSaved {len(df)} rows -> {out}\n")

    # ── Report ─────────────────────────────────────────────────────────────
    for wind_mode in ['fixed', 'scaled']:
        sub = [r for r in all_rows if r['wind_mode'] == wind_mode]
        print("=" * 68)
        print(f"WIND MODE: {wind_mode.upper()}")
        print("=" * 68)
        print(f"{'keff':>5} {'lat':>4} {'Kd':>4}  {'floor':>7} {'ceil':>7} {'window':>8}  "
              f"{'S_ceil':>7} {'S_ratio':>8}")
        print("-" * 68)
        for r in sorted(sub, key=lambda x: (x['keff'], x['latency'])):
            fl  = f"{r['pid_floor']:.1f}"  if r['pid_floor']  else 'INFEAS'
            ce  = f"{r['pid_ceil']:.1f}"   if r['pid_ceil']   else 'INFEAS'
            pw  = f"{r['pid_window']:.1f}x" if r['pid_window'] else 'N/A'
            sc  = f"{r['smith_ceil']:.1f}" if r['smith_ceil'] else 'INFEAS'
            sr  = f"{r['smith_ceil_ratio']:.2f}" if r['smith_ceil_ratio'] else 'N/A'
            print(f"{r['keff']:>5.0f} {r['latency']:>4}  {r['Kd']:>3.1f}  "
                  f"{fl:>7} {ce:>7} {pw:>8}  {sc:>7} {sr:>8}")
        print()

        # slope analysis
        print(f"  Floor scaling (log-log regression):")
        for lat in LAT_VALS:
            pts = [(r['keff'], r['pid_floor']) for r in sub
                   if r['latency'] == lat and r['pid_floor'] and r['pid_floor'] > 0]
            if len(pts) >= 3:
                xs, ys = zip(*pts)
                slope = np.polyfit(np.log(xs), np.log(ys), 1)[0]
                print(f"    lat={lat}: floor ~ keff^{slope:.2f}  (theory 1.00)")

        print()
        for keff in KEFF_VALS:
            pts = [(r['latency'], r['pid_floor']) for r in sub
                   if abs(r['keff'] - keff) < 0.1 and r['pid_floor'] and r['pid_floor'] > 0]
            if len(pts) >= 3:
                xs, ys = zip(*pts)
                slope = np.polyfit(np.log(xs), np.log(ys), 1)[0]
                print(f"    keff={keff:.0f}: floor ~ lat^{slope:.2f}  (theory 1.00)")

        print()
        print(f"  Window vs Pi:  window ~ Pi^slope  (theory -1.00)")
        pi_pts = [(r['pi_keff'], r['pid_window']) for r in sub
                  if r['pid_window'] and r['pi_keff'] > 0]
        if len(pi_pts) >= 4:
            xs, ys = zip(*pi_pts)
            slope = np.polyfit(np.log(xs), np.log(ys), 1)[0]
            print(f"    slope = {slope:.2f}")
        print()

        # Smith regime diagnosis
        print(f"  Smith ratio diagnosis:")
        print(f"  {'keff':>5} {'lat':>4}  {'ratio':>8}  regime")
        for r in sorted(sub, key=lambda x: (x['keff'], x['latency'])):
            rat = r['smith_ceil_ratio']
            if rat is None:
                regime = 'indeterminate'
            elif rat > 2.0:
                regime = 'delay-limited   (Smith helps)'
            elif rat < 1.3:
                regime = 'saturation-dominated (Smith fails/collapses)'
            else:
                regime = 'mixed'
            rs = f"{rat:.2f}" if rat else 'N/A'
            print(f"  {r['keff']:>5.0f} {r['latency']:>4}  {rs:>8}  {regime}")
        print()

    # ── Q&A summary ────────────────────────────────────────────────────────
    print("=" * 68)
    print("ANSWERS: Does the Pi constraint emerge from the minimal plant?")
    print("=" * 68)

    fixed_sub = [r for r in all_rows if r['wind_mode'] == 'fixed']
    scaled_sub = [r for r in all_rows if r['wind_mode'] == 'scaled']

    # A) Does high-keff ceiling still exist?
    high_keff_fixed = [r for r in fixed_sub if r['keff'] >= 20]
    any_ceil_fixed  = any(r['pid_ceil'] for r in high_keff_fixed)
    print(f"\nA) High-keff ceiling present (fixed wind): {any_ceil_fixed}")
    for r in high_keff_fixed:
        print(f"   keff={r['keff']:.0f} lat={r['latency']}: ceil={r['pid_ceil']}")

    # B) Ceiling slope with latency at mid keff
    mid_keff_rows = [r for r in fixed_sub if abs(r['keff'] - 12.0) < 0.1 and r['pid_ceil']]
    if len(mid_keff_rows) >= 3:
        lats = np.array([r['latency'] for r in mid_keff_rows])
        ceils = np.array([r['pid_ceil'] for r in mid_keff_rows])
        slope_c = np.polyfit(np.log(lats), np.log(ceils), 1)[0]
        print(f"\nB) Ceiling ~ lat^{slope_c:.2f}  (theory -1.00, fixed wind, keff=12)")

    # C/D) Smith regime
    print(f"\nC/D) Smith predictor ratio at keff=25 (saturation-dominated test):")
    smith25 = [r for r in fixed_sub if abs(r['keff'] - 25.0) < 0.1]
    for r in smith25:
        rat = r['smith_ceil_ratio']
        status = 'fails/collapses' if (rat is None or rat < 1.3) else f'raises by {rat:.2f}×'
        print(f"   lat={r['latency']}: Smith {status}")

    # E) Artifact test: is floor ~ keff^1 present WITHOUT wind scaling?
    print(f"\nE) Artifact A1 test (floor ~ keff slope with FIXED wind):")
    for lat in LAT_VALS:
        pts = [(r['keff'], r['pid_floor']) for r in fixed_sub
               if r['latency'] == lat and r['pid_floor']]
        if len(pts) >= 3:
            xs, ys = zip(*pts)
            slope = np.polyfit(np.log(xs), np.log(ys), 1)[0]
            print(f"   lat={lat}: floor ~ keff^{slope:.2f}  "
                  f"({'REAL mechanism' if slope > 0.5 else 'artifact-only'})")

    print(f"\nE) Artifact A1 test (floor ~ keff slope with SCALED wind):")
    for lat in LAT_VALS:
        pts = [(r['keff'], r['pid_floor']) for r in scaled_sub
               if r['latency'] == lat and r['pid_floor']]
        if len(pts) >= 3:
            xs, ys = zip(*pts)
            slope = np.polyfit(np.log(xs), np.log(ys), 1)[0]
            print(f"   lat={lat}: floor ~ keff^{slope:.2f}")

    print()
