"""
tools/gain_window_boundary_v2.py

Corrected boundary experiment: FRAGILE + matched EASY designs.

Key fixes over v1:
  1. Includes 6 FRAGILE designs from exp1 (elevated kp_floor means ceiling meets floor)
  2. 6 matched EASY designs at similar theta_ddot levels for direct comparison
  3. Shows that environment (wind x latency) modulates closure, but FRAGILE designs
     are already at risk while matched EASY designs stay wide.

Protocol:
  - Select 6 FRAGILE from exp1 spanning td = [36, 70, 100, 130, 170, 210] rad/s²
  - Select 6 matched EASY at closest td values
  - Sweep wind [0.05,0.12,0.22,0.32,0.42] x latency [1,3,6] x 15 Kp x 7 seeds
  - Total: 12 designs x 5 wind x 3 lat x 15 Kp x 7 seeds = 18,900 sims (~2 min)
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

# ── Config ─────────────────────────────────────────────────────────────────────
N_SEEDS        = 7
SR_THRESH      = 0.80
F15_T_AVG      = 14.4
L_NOZZLE       = 0.25
CU_TO_RAD      = (np.pi / 180) * (15 / 12)

KP_GRID        = np.array([1, 2, 3, 5, 8, 12, 18, 27, 40, 60, 90, 130, 190, 270, 320])
WIND_LEVELS    = [0.05, 0.12, 0.22, 0.32, 0.42]
LATENCY_LEVELS = [1, 3, 6]

# Target td levels for FRAGILE selection
FRAG_TD_TARGETS = [36, 70, 100, 130, 170, 210]

CSV     = Path('experiments/results/exp1_regime_index_py.csv')
OUT_RAW = Path('experiments/results/gain_window_v2_py.csv')
OUT_WIN = Path('experiments/results/gain_window_v2_summary_py.csv')


def _td(row):
    keff = F15_T_AVG * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']
    return keff * row['max_gimbal_deg'] * 12.0 / 15.0


def _keff(row):
    return F15_T_AVG * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']


def select_designs(df):
    df = df.copy()
    df['td']   = df.apply(_td, axis=1)
    df['keff'] = df.apply(_keff, axis=1)

    frag_df = df[df['regime_label'] == 'FRAGILE']
    easy_df = df[df['regime_label'] == 'EASY']

    fragile_selected = []
    easy_selected    = []

    for target in FRAG_TD_TARGETS:
        # Pick FRAGILE design closest to target td
        idx = (frag_df['td'] - target).abs().idxmin()
        frow = frag_df.loc[[idx]].copy()
        frow['group_type'] = 'FRAGILE'
        frow['td_target']  = target
        fragile_selected.append(frow)

        # Pick EASY design closest to same td
        eidx = (easy_df['td'] - frow['td'].values[0]).abs().idxmin()
        erow = easy_df.loc[[eidx]].copy()
        erow['group_type'] = 'EASY'
        erow['td_target']  = target
        easy_selected.append(erow)

    all_sel = pd.concat(fragile_selected + easy_selected).drop_duplicates(
        subset=['rocket_id']).reset_index(drop=True)
    return all_sel


def run_one(row_dict, wind, latency, Kp, seed):
    d = row_dict.copy()
    d['wind_strength'] = float(wind)
    d['latency_steps'] = int(latency)

    plant = build_plant(d)
    act   = build_actuator(d)
    sen   = build_sensor(d)
    dis   = build_disturbance(d)
    sc    = build_scenario()

    fc = FidelityConfig.full()
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fc)

    Kd  = float(row_dict.get('best_Kd', 1.0))
    pid = PIDParams(Kp=float(Kp), Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    r   = simulate(pid, plant, act, sen, dis, sc, seed=seed)
    return int(r.success)


def compute_window(grp):
    g = grp.sort_values('Kp')
    passing = g[g['sr'] >= SR_THRESH]['Kp']
    if len(passing) == 0:
        return dict(kp_floor=np.nan, kp_ceil=np.nan, window_ratio=0.0, collapsed=True)
    return dict(kp_floor=passing.min(), kp_ceil=passing.max(),
                window_ratio=passing.max() / passing.min(), collapsed=False)


def main():
    print(f"Loading {CSV}")
    df = pd.read_csv(CSV)
    designs = select_designs(df)

    print(f"\nSelected {len(designs)} designs:")
    print(f"{'ID':8s}  {'type':7s}  {'td':6s}  {'keff':6s}  {'Iyy':6s}  "
          f"{'gimbal':6s}  {'motor':5s}  {'lat_nat':7s}  {'wind_nat':8s}  {'bestKp':6s}  {'bestKd':6s}")
    for _, row in designs.sort_values(['td_target', 'group_type']).iterrows():
        keff = F15_T_AVG * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']
        print(f"{row.get('rocket_id','?'):8s}  {row['group_type']:7s}  "
              f"{row['td']:6.1f}  {keff:6.2f}  {row['Iyy']:.4f}  "
              f"{row['max_gimbal_deg']:6.1f}  {row['motor_scale']:5.2f}  "
              f"{row.get('latency_steps','?'):7}  {row.get('wind_strength','?'):8.3f}  "
              f"{row.get('best_Kp','?'):6}  {row.get('best_Kd','?'):6}")

    # Build task list
    tasks, meta = [], []
    for _, row in designs.iterrows():
        rd = row.to_dict()
        for wind in WIND_LEVELS:
            for lat in LATENCY_LEVELS:
                for Kp in KP_GRID:
                    for seed in range(1, N_SEEDS + 1):
                        tasks.append((rd, wind, lat, Kp, seed))
                        meta.append({'rocket_id': rd.get('rocket_id','?'),
                                     'group_type': rd.get('group_type','?'),
                                     'td': row['td'],
                                     'keff': row.get('keff', 0.0),
                                     'Iyy': rd['Iyy'],
                                     'best_Kp_exp1': rd.get('best_Kp', np.nan),
                                     'lat_natural': rd.get('latency_steps', np.nan),
                                     'wind_natural': rd.get('wind_strength', np.nan),
                                     'wind': wind, 'latency': lat,
                                     'Kp': int(Kp), 'seed': seed})

    print(f"\nRunning {len(tasks)} sims "
          f"({len(designs)} designs x {len(WIND_LEVELS)} wind x {len(LATENCY_LEVELS)} lat "
          f"x {len(KP_GRID)} Kp x {N_SEEDS} seeds)...")

    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(run_one)(rd, w, l, kp, s) for rd, w, l, kp, s in tasks
    )

    sim_df = pd.DataFrame([{**m, 'success': s} for m, s in zip(meta, raw)])

    # SR per (design, wind, latency, Kp)
    sr_df = (sim_df.groupby(['rocket_id','group_type','td','keff','Iyy',
                              'best_Kp_exp1','lat_natural','wind_natural',
                              'wind','latency','Kp'])
             ['success'].mean().reset_index().rename(columns={'success':'sr'}))
    sr_df.to_csv(OUT_RAW, index=False)
    print(f"\nSaved SR curves: {OUT_RAW}  ({len(sr_df)} rows)")

    # Compute window per (design, wind, latency)
    win_rows = []
    for keys, grp in sr_df.groupby(['rocket_id','group_type','td','keff','Iyy',
                                     'best_Kp_exp1','lat_natural','wind_natural',
                                     'wind','latency']):
        w = compute_window(grp)
        rdict = dict(zip(['rocket_id','group_type','td','keff','Iyy',
                           'best_Kp_exp1','lat_natural','wind_natural',
                           'wind','latency'], keys))
        rdict.update(w)
        win_rows.append(rdict)

    win_df = pd.DataFrame(win_rows)
    win_df.to_csv(OUT_WIN, index=False)
    print(f"Saved window summary: {OUT_WIN}  ({len(win_df)} rows)")

    # ── Analysis ───────────────────────────────────────────────────────────────
    print("\n" + "="*80)
    print("FRAGILE vs EASY: WINDOW COMPARISON ACROSS CONDITIONS")
    print("="*80)

    frag_sel = designs[designs['group_type'] == 'FRAGILE'].sort_values('td')
    easy_sel = designs[designs['group_type'] == 'EASY'].sort_values('td')

    for _, frow in frag_sel.iterrows():
        frid = frow['rocket_id']
        ftd  = frow['td']
        # find matched EASY
        closest_e = (easy_sel['td'] - ftd).abs().idxmin()
        erid = easy_sel.loc[closest_e, 'rocket_id']

        print(f"\n--- td={ftd:.0f} rad/s² | FRAGILE={frid} (bestKp={frow.get('best_Kp_exp1',frow.get('best_Kp','?'))}) "
              f"vs EASY={erid} (bestKp={easy_sel.loc[closest_e,'best_Kp']}) ---")
        print(f"{'Condition':20s}  {'FRAGILE floor':13s}  {'FRAGILE ceil':12s}  {'F ratio':8s}  "
              f"{'EASY floor':10s}  {'EASY ceil':9s}  {'E ratio':7s}")

        for lat in LATENCY_LEVELS:
            for wind in WIND_LEVELS:
                fw = win_df[(win_df['rocket_id'] == frid) &
                            (win_df['wind'] == wind) &
                            (win_df['latency'] == lat)]
                ew = win_df[(win_df['rocket_id'] == erid) &
                            (win_df['wind'] == wind) &
                            (win_df['latency'] == lat)]
                if len(fw) == 0 or len(ew) == 0:
                    continue
                fr, er = fw.iloc[0], ew.iloc[0]
                f_status = "COLLAPSED" if fr['collapsed'] else f"{fr['window_ratio']:.1f}x"
                e_status = f"{er['window_ratio']:.1f}x"
                f_floor  = "---" if fr['collapsed'] else f"{fr['kp_floor']:.0f}"
                f_ceil   = "---" if fr['collapsed'] else f"{fr['kp_ceil']:.0f}"
                print(f"  lat={lat} wind={wind:.2f}  {'':4s}  "
                      f"{f_floor:13s}  {f_ceil:12s}  {f_status:8s}  "
                      f"{er['kp_floor']:10.0f}  {er['kp_ceil']:9.0f}  {e_status}")

    # Summary: how often does FRAGILE collapse vs EASY?
    print("\n" + "="*80)
    print("COLLAPSE RATE SUMMARY (window_ratio < 3 = near-collapse)")
    print("="*80)
    for gtype in ['FRAGILE', 'EASY']:
        sub = win_df[win_df['group_type'] == gtype]
        n_total    = len(sub)
        n_collapse = (sub['collapsed']).sum()
        n_near     = (sub['window_ratio'] < 3).sum()
        n_tight    = (sub['window_ratio'] < 10).sum()
        print(f"{gtype:7s}: {n_total} conditions | "
              f"collapsed={n_collapse} ({100*n_collapse/n_total:.0f}%) | "
              f"ratio<3 = {n_near} ({100*n_near/n_total:.0f}%) | "
              f"ratio<10 = {n_tight} ({100*n_tight/n_total:.0f}%)")

    # Show worst conditions for EASY designs
    print("\n=== EASY DESIGNS AT WORST-CASE (lat=6, wind≥0.32) ===")
    worst_easy = win_df[(win_df['group_type'] == 'EASY') &
                        (win_df['latency'] == 6) &
                        (win_df['wind'] >= 0.32)].sort_values('window_ratio')
    for _, r in worst_easy.head(10).iterrows():
        status = "COLLAPSED" if r['collapsed'] else f"ratio={r['window_ratio']:.1f}x"
        print(f"  {r['rocket_id']:8s} td={r['td']:6.1f} keff={r['keff']:.2f} "
              f"wind={r['wind']:.2f} lat={r['latency']:.0f} → {status} "
              f"(floor={r['kp_floor']:.0f}, ceil={r['kp_ceil']:.0f})")

    # Show best conditions for FRAGILE designs
    print("\n=== FRAGILE DESIGNS AT BEST-CASE (lat=1, wind=0.05) ===")
    best_frag = win_df[(win_df['group_type'] == 'FRAGILE') &
                       (win_df['latency'] == 1) &
                       (win_df['wind'] == 0.05)].sort_values('window_ratio', ascending=False)
    for _, r in best_frag.iterrows():
        status = "COLLAPSED" if r['collapsed'] else f"ratio={r['window_ratio']:.1f}x"
        print(f"  {r['rocket_id']:8s} td={r['td']:6.1f} keff={r['keff']:.2f} "
              f"wind={r['wind']:.2f} lat={r['latency']:.0f} → {status} "
              f"(floor={r['kp_floor']:.0f}, ceil={r['kp_ceil']:.0f})")

    # keff comparison: FRAGILE vs EASY at same td
    print("\n=== KEFF COMPARISON: FRAGILE vs MATCHED EASY ===")
    print("FRAGILE has higher keff at same theta_ddot → elevated floor → window collapse")
    print(f"{'ID':8s}  {'type':7s}  {'td':6s}  {'keff':6s}  {'Iyy':6s}  {'gimbal':6s}")
    for _, row in designs.sort_values(['td', 'group_type']).iterrows():
        keff = F15_T_AVG * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']
        print(f"  {row.get('rocket_id','?'):8s}  {row['group_type']:7s}  "
              f"{row['td']:6.1f}  {keff:6.2f}  {row['Iyy']:.4f}  {row['max_gimbal_deg']:.1f}°")

    # Ceiling compression by latency: show for all designs at wind=0.22
    print("\n=== CEILING COMPRESSION BY LATENCY (wind=0.22) ===")
    ref = win_df[win_df['wind'] == 0.22].sort_values(['group_type', 'td', 'latency'])
    for _, r in ref.iterrows():
        if r['collapsed']:
            status = f"COLLAPSED (floor={r['kp_floor']:.0f})"
        else:
            status = f"ceil={r['kp_ceil']:.0f}  ratio={r['window_ratio']:.1f}x"
        print(f"  {r['rocket_id']:8s} [{r['group_type']:7s}] td={r['td']:6.1f} "
              f"lat={r['latency']:.0f}  → {status}")

    print("\nDone. Results saved to gain_window_v2_py.csv and gain_window_v2_summary_py.csv")


if __name__ == '__main__':
    main()
