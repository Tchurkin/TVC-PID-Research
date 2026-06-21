"""
tools/gain_window_boundary.py

Derives the gain window boundary equation:
    FRAGILE when theta_ddot_max > f*(wind_strength, latency_steps)

Protocol:
1. Select EASY designs at 8 target theta_ddot levels from exp1 (n=2400 run)
2. For each design, override wind_strength (5 levels) and latency_steps (3 levels)
3. Run Kp sweep [15 log-spaced values, 1-320], 5 seeds each, full physics
4. Measure kp_floor, kp_ceiling, window_ratio at each (design, wind, latency)
5. Fit boundary: td_crit = A * wind^alpha * latency^beta

Total: ~8 designs x 5 wind x 3 latency x 15 Kp x 5 seeds = ~9,000 sims
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from simulator import simulate, PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario

# ── Config ────────────────────────────────────────────────────────────────────
N_SEEDS        = 5
SR_THRESH      = 0.80
F15_T_AVG      = 14.4
L_NOZZLE       = 0.25
CU_TO_RAD      = (np.pi / 180) * (15 / 12)

KP_GRID        = np.array([1, 2, 3, 5, 8, 12, 18, 27, 40, 60, 90, 130, 190, 270, 320])
WIND_LEVELS    = [0.05, 0.12, 0.22, 0.32, 0.42]
LATENCY_LEVELS = [1, 3, 6]
TD_TARGETS     = [18, 30, 45, 60, 80, 110, 155, 210]  # rad/s² — dense near 55 boundary

CSV = Path('experiments/results/exp1_regime_index_py.csv')
OUT_RAW = Path('experiments/results/gain_window_boundary_py.csv')
OUT_WIN = Path('experiments/results/gain_window_summary_py.csv')


def _td(row):
    keff = F15_T_AVG * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']
    return keff * row['max_gimbal_deg'] * 12.0 / 15.0


def select_designs(df):
    df = df.copy()
    df['td'] = df.apply(_td, axis=1)
    easy = df[df['regime_label'] == 'EASY']
    selected = []
    for target in TD_TARGETS:
        idx = (easy['td'] - target).abs().idxmin()
        selected.append(easy.loc[[idx]])
    sel = pd.concat(selected).drop_duplicates(subset=['rocket_id']).reset_index(drop=True)
    sel['td'] = sel.apply(_td, axis=1)
    return sel


def run_one(row_dict, wind, latency, Kp, seed):
    d = row_dict.copy()
    d['wind_strength']  = float(wind)
    d['latency_steps']  = int(latency)

    plant = build_plant(d)
    act   = build_actuator(d)
    sen   = build_sensor(d)
    dis   = build_disturbance(d)
    sc    = build_scenario()

    fc = FidelityConfig.full()
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fc)

    Kd  = float(row_dict.get('best_Kd', 8.0))
    pid = PIDParams(Kp=float(Kp), Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    r   = simulate(pid, plant, act, sen, dis, sc, seed=seed)
    return int(r.success)


def main():
    print(f"Loading {CSV}")
    df = pd.read_csv(CSV)
    designs = select_designs(df)

    print(f"Selected {len(designs)} designs:")
    for _, row in designs.iterrows():
        print(f"  {row.get('rocket_id','?'):8s}  td={row['td']:6.1f}  "
              f"Iyy={row['Iyy']:.4f}  gimbal={row['max_gimbal_deg']:.1f}  "
              f"motor={row['motor_scale']:.2f}  Kd={row.get('best_Kd',8):.1f}")

    # Build task list
    tasks = []
    meta  = []
    for _, row in designs.iterrows():
        rd = row.to_dict()
        for wind in WIND_LEVELS:
            for lat in LATENCY_LEVELS:
                for Kp in KP_GRID:
                    for seed in range(1, N_SEEDS + 1):
                        tasks.append((rd, wind, lat, Kp, seed))
                        meta.append({'rocket_id': rd.get('rocket_id','?'),
                                     'td': row['td'], 'Iyy': rd['Iyy'],
                                     'wind': wind, 'latency': lat,
                                     'Kp': int(Kp), 'seed': seed})

    n_tasks = len(tasks)
    print(f"\nRunning {n_tasks} sims "
          f"({len(designs)} designs x {len(WIND_LEVELS)} wind x {len(LATENCY_LEVELS)} latency "
          f"x {len(KP_GRID)} Kp x {N_SEEDS} seeds)...")

    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(run_one)(rd, w, l, kp, s) for rd, w, l, kp, s in tasks
    )

    sim_df = pd.DataFrame([{**m, 'success': s} for m, s in zip(meta, raw)])

    # SR per (design, wind, latency, Kp)
    sr_df = (sim_df.groupby(['rocket_id','td','Iyy','wind','latency','Kp'])
             ['success'].mean().reset_index().rename(columns={'success':'sr'}))
    sr_df.to_csv(OUT_RAW, index=False)
    print(f"\nSaved SR curves: {OUT_RAW}  ({len(sr_df)} rows)")

    # Compute window stats
    win_rows = []
    for (rid, td, Iyy, wind, lat), grp in sr_df.groupby(['rocket_id','td','Iyy','wind','latency']):
        g = grp.sort_values('Kp')
        passing = g[g['sr'] >= SR_THRESH]['Kp']
        if len(passing) == 0:
            kp_fl, kp_ce, ratio, collapsed = np.nan, np.nan, 0.0, True
        else:
            kp_fl  = passing.min()
            kp_ce  = passing.max()
            ratio  = kp_ce / kp_fl
            collapsed = False
        win_rows.append({'rocket_id': rid, 'td': td, 'Iyy': Iyy,
                         'wind': wind, 'latency': lat,
                         'kp_floor': kp_fl, 'kp_ceil': kp_ce,
                         'window_ratio': ratio, 'collapsed': collapsed})

    win_df = pd.DataFrame(win_rows)
    win_df.to_csv(OUT_WIN, index=False)
    print(f"Saved window summary: {OUT_WIN}  ({len(win_df)} rows)")

    # ── Analysis ──────────────────────────────────────────────────────────────
    print("\n=== WINDOW vs THETA_DDOT (wind=0.22, latency=3) ===")
    ref = win_df[(win_df['wind'] == 0.22) & (win_df['latency'] == 3)].sort_values('td')
    for _, r in ref.iterrows():
        status = "COLLAPSED" if r['collapsed'] else f"ratio={r['window_ratio']:.1f}x"
        print(f"  td={r['td']:6.1f}  floor={r['kp_floor']:6.1f}  "
              f"ceil={r['kp_ceil']:6.1f}  {status}")

    print("\n=== FLOOR vs WIND (latency=3, td~55-80) ===")
    mid_td = win_df[(win_df['latency'] == 3) & (win_df['td'].between(40, 90))].sort_values('wind')
    for _, r in mid_td.iterrows():
        print(f"  td={r['td']:5.1f}  wind={r['wind']:.2f}  "
              f"floor={r['kp_floor']:6.1f}  ceil={r['kp_ceil']:6.1f}  ratio={r['window_ratio']:.1f}x")

    print("\n=== CEILING vs LATENCY (wind=0.22, td~55-80) ===")
    mid_td2 = win_df[(win_df['wind'] == 0.22) & (win_df['td'].between(40, 90))].sort_values('latency')
    for _, r in mid_td2.iterrows():
        print(f"  td={r['td']:5.1f}  latency={r['latency']}  "
              f"floor={r['kp_floor']:6.1f}  ceil={r['kp_ceil']:6.1f}  ratio={r['window_ratio']:.1f}x")

    # ── Boundary fitting ──────────────────────────────────────────────────────
    print("\n=== BOUNDARY EQUATION FIT ===")
    # For each (wind, latency), find the td threshold where window collapses
    # Use: last td where window_ratio >= 3 (window just barely viable)
    VIABILITY_RATIO = 3.0
    bnd_rows = []
    for (wind_val, lat_val), grp in win_df.groupby(['wind', 'latency']):
        grp_s = grp.sort_values('td')
        viable = grp_s[grp_s['window_ratio'] >= VIABILITY_RATIO]
        non_v  = grp_s[grp_s['window_ratio'] < VIABILITY_RATIO]
        if len(viable) == 0:
            td_crit = grp_s['td'].min()  # all collapsed
        elif len(non_v) == 0:
            td_crit = grp_s['td'].max() * 1.5  # extrapolate (none collapsed yet)
        else:
            # interpolate between last viable and first non-viable
            td_hi = viable['td'].max()
            td_lo = non_v[non_v['td'] > td_hi]['td'].min() if len(non_v[non_v['td'] > td_hi]) else td_hi * 1.2
            td_crit = (td_hi + td_lo) / 2.0
        bnd_rows.append({'wind': wind_val, 'latency': lat_val, 'td_crit': td_crit})
        print(f"  wind={wind_val:.2f}  lat={lat_val}:  td_crit = {td_crit:.0f} rad/s2")

    bnd = pd.DataFrame(bnd_rows)

    # Power law fit in log space: log(td_crit) = log(A) + alpha*log(wind) + beta*log(latency)
    try:
        valid = bnd[np.isfinite(bnd['td_crit']) & (bnd['td_crit'] > 0)].copy()
        valid['log_td']   = np.log(valid['td_crit'])
        valid['log_wind'] = np.log(valid['wind'])
        valid['log_lat']  = np.log(valid['latency'])

        X = np.column_stack([np.ones(len(valid)), valid['log_wind'], valid['log_lat']])
        y = valid['log_td'].values
        coefs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
        log_A, alpha, beta = coefs
        A = np.exp(log_A)

        # R² for fit quality
        y_pred = X @ coefs
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - y.mean()) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

        print(f"\n  BOUNDARY EQUATION (power law, R2={r2:.3f}):")
        print(f"  td_crit = {A:.1f} * wind^({alpha:.3f}) * latency^({beta:.3f})")
        print(f"\n  Example predictions:")
        for w, l in [(0.05,1), (0.25,3), (0.25,6), (0.45,3), (0.45,6)]:
            tc = A * w**alpha * l**beta
            print(f"    wind={w:.2f}, lat={l}:  td_crit = {tc:.0f} rad/s2")

        print(f"\n  The n=2400 Youden-J threshold of 55 rad/s2 corresponds to:")
        # Solve for the effective (wind, latency) that gives td_crit=55
        # At design-space means: wind~0.25, latency~3.5
        tc_mean = A * 0.25**alpha * 3.5**beta
        print(f"    wind=0.25, latency=3.5 (design-space averages): td_crit = {tc_mean:.0f} rad/s2")
        print(f"    (empirical Youden-J = 55 rad/s2 — matches if R2 is good)")

        print(f"\n  PHYSICAL INTERPRETATION:")
        print(f"    alpha={alpha:.3f}: td_crit {'decreases' if alpha < 0 else 'increases'} with wind "
              f"(stronger wind → {'harder' if alpha < 0 else 'easier'} to be FRAGILE)")
        print(f"    beta={beta:.3f}:  td_crit {'decreases' if beta < 0 else 'increases'} with latency "
              f"(more latency → {'harder' if beta < 0 else 'easier'} to be FRAGILE)")

    except Exception as e:
        print(f"  Fit failed: {e}")

    print("\n=== FLOOR SCALING WITH WIND ===")
    # At fixed latency=3, show how floor scales with wind across td levels
    lat3 = win_df[win_df['latency'] == 3].dropna(subset=['kp_floor'])
    if len(lat3) >= 4:
        try:
            log_floor = np.log(lat3['kp_floor'].clip(lower=0.5))
            log_wind  = np.log(lat3['wind'])
            from scipy.stats import linregress as lr
            sl, ic, r, p, se = lr(log_wind, log_floor)
            print(f"  kp_floor ~ wind^{sl:.2f}  (r={r:.3f}, p={p:.4f}, n={len(lat3)})")
            print(f"  Physical: stronger wind raises gain floor (slope sign: {'+' if sl>0 else '-'})")
        except Exception as e:
            print(f"  Floor regression failed: {e}")

    print("\n=== CEILING SCALING WITH THETA_DDOT ===")
    ref_cond = win_df[(win_df['wind'] == 0.22) & (win_df['latency'] == 3)].dropna(subset=['kp_ceil'])
    if len(ref_cond) >= 4:
        try:
            from scipy.stats import linregress as lr
            log_ceil = np.log(ref_cond['kp_ceil'].clip(lower=1.0))
            log_td   = np.log(ref_cond['td'])
            sl, ic, r, p, se = lr(log_td, log_ceil)
            print(f"  kp_ceil ~ td^{sl:.2f}  (r={r:.3f}, p={p:.4f}, n={len(ref_cond)})")
            print(f"  Physical: higher authority compresses gain ceiling (slope sign: {'+' if sl>0 else '-'})")
        except Exception as e:
            print(f"  Ceiling regression failed: {e}")

    print("\nDone. See gain_window_boundary_py.csv and gain_window_summary_py.csv")


if __name__ == '__main__':
    main()
