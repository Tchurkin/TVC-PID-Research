"""
tools/frozen_kd_artifact_test.py  (2026-06-22)

RESEARCH QUESTION: Does freezing Kd at the joint-search optimum artificially inflate
the measured kp_floor for high-keff designs?

DESIGN: 8 designs from fsat dataset (4 keff tiers x 2 latency levels).
For each design, at 5 Kp values bracketing the frozen-Kd floor, sweep 9 Kd values
spanning +-3 octaves around opt_Kd. Measure the TRUE floor (minimum Kp for which
ANY tested Kd achieves SR >= 0.80).

Compare:
  frozen_floor = kp_floor from window_ratio_v2 (fixed opt_Kd)
  true_floor   = min Kp where max_Kd SR >= 0.80

Artifact metric:
  floor_shift = (frozen_floor - true_floor) / frozen_floor
  Positive shift = frozen-Kd was too conservative (floor appears higher than reality).

DECISION CRITERIA (pre-specified):
  Negligible: max |floor_shift| < 0.15 for all 8 designs
  Moderate:   2-3 designs with floor_shift 0.15-0.30
  Significant: 3+ designs (keff >= 20) with floor_shift > 0.30

Seeds: 50001-50015 (disjoint from all prior experiments)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy.stats import spearmanr

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario, sample_lhs)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25
N_SEEDS   = 15
SEED0     = 50001
SR_THRESH = 0.80

OUT_CSV = Path('experiments/results/frozen_kd_artifact_py.csv')

# ---- Design selection ----
# 4 keff tiers x 2 lat levels, taken from fsat dataset
# Format: (rocket_id, opt_Kd, frozen_floor, frozen_ceil, keff, lat, Pi, fsat_floor)
TARGET_DESIGNS = {
    # keff~5, lat=3: linear regime (fsat~0.03)
    'R4987':  dict(keff=4.95,  lat=3,  Pi=44.5,   floor=0.24,   ceil=335.25, fsat=0.03),
    # keff~5, lat=6: transitional regime
    'R8400':  dict(keff=4.35,  lat=6,  Pi=156.6,  floor=1.02,   ceil=140.49, fsat=0.12),
    # keff~12, lat=3: transitional
    'R0049':  dict(keff=9.82,  lat=3,  Pi=88.4,   floor=0.76,   ceil=448.00, fsat=0.18),
    # keff~12, lat=6: bang-bang
    'R9127':  dict(keff=12.31, lat=6,  Pi=443.0,  floor=1.82,   ceil=78.68,  fsat=0.63),
    # keff~25, lat=3: bang-bang
    'R2866':  dict(keff=25.64, lat=3,  Pi=230.8,  floor=3.24,   ceil=140.49, fsat=0.55),
    # keff~25, lat=6: deep bang-bang (high floor)
    'R3022':  dict(keff=15.51, lat=6,  Pi=558.3,  floor=3.24,   ceil=58.88,  fsat=0.70),
    # keff~36, lat=3: deep bang-bang
    'R1687':  dict(keff=36.30, lat=3,  Pi=326.7,  floor=7.74,   ceil=187.74, fsat=0.63),
    # keff~22, lat=6: very narrow window
    'R8162':  dict(keff=16.24, lat=6,  Pi=584.6,  floor=4.33,   ceil=58.88,  fsat=0.63),
}


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds):
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['success_rate'])


def _process_design(rocket_id, meta, pool_row):
    frozen_floor = meta['floor']
    frozen_ceil  = meta['ceil']

    # opt_Kd from fsat dataset
    fsat_df = pd.read_csv('experiments/results/fsat_window_test_py.csv')
    fsat_row = fsat_df[fsat_df.rocket_id == rocket_id]
    if len(fsat_row) == 0:
        print(f"  {rocket_id}: not in fsat dataset, skipping")
        return None
    opt_Kd = float(fsat_row.iloc[0]['opt_Kd'])

    plant = build_plant(pool_row)
    act   = build_actuator(pool_row)
    sen   = build_sensor(pool_row)
    dis   = build_disturbance(pool_row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    seeds = list(range(SEED0, SEED0 + N_SEEDS))

    # 5 Kp values bracketing the frozen floor: [0.4x, 0.6x, 1.0x, 1.5x, 2.5x]
    if frozen_floor < 0.5:
        kp_grid = np.array([0.1, 0.25, frozen_floor, frozen_floor * 2, frozen_floor * 4])
    else:
        kp_grid = np.array([
            frozen_floor * 0.35,
            frozen_floor * 0.55,
            frozen_floor * 1.0,
            frozen_floor * 1.6,
            frozen_floor * 2.5,
        ])
    kp_grid = np.clip(kp_grid, 0.05, min(frozen_ceil * 0.9, 400))

    # 9 Kd values: opt_Kd x {1/8, 1/4, 1/2, 1/sqrt(2), 1, sqrt(2), 2, 4, 8}
    kd_multipliers = np.array([0.125, 0.25, 0.5, 0.707, 1.0, 1.414, 2.0, 4.0, 8.0])
    kd_grid = np.clip(opt_Kd * kd_multipliers, 0.01, 64.0)

    rows = []
    for Kp in kp_grid:
        for Kd in kd_grid:
            sr = _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds)
            rows.append(dict(
                rocket_id=rocket_id, keff=meta['keff'], latency=meta['lat'], Pi=meta['Pi'],
                fsat_floor=meta['fsat'], frozen_floor=frozen_floor, frozen_ceil=frozen_ceil,
                opt_Kd=opt_Kd, Kp=float(Kp), Kd=float(Kd), sr=sr,
                kd_mult=float(Kd / opt_Kd),
            ))

    # Compute frozen-Kd SR at each Kp (reference)
    frozen_sr_per_kp = {}
    for r in rows:
        if abs(r['Kd'] - opt_Kd) / opt_Kd < 0.05:  # tolerance for float comparison
            frozen_sr_per_kp[r['Kp']] = r['sr']

    # True floor: minimum Kp where max over Kd grid achieves SR >= SR_THRESH
    df_r = pd.DataFrame(rows)
    max_sr_per_kp = df_r.groupby('Kp')['sr'].max()
    passing_kp = max_sr_per_kp[max_sr_per_kp >= SR_THRESH].index
    true_floor = float(passing_kp.min()) if len(passing_kp) > 0 else float('inf')

    # Best Kd at each Kp
    best_kd_per_kp = df_r.loc[df_r.groupby('Kp')['sr'].idxmax(), ['Kp','Kd','sr']].set_index('Kp')

    floor_shift = (frozen_floor - true_floor) / frozen_floor if frozen_floor > 0 else 0.0

    print(f"  {rocket_id}: keff={meta['keff']:.1f} lat={meta['lat']} Pi={meta['Pi']:.0f}")
    print(f"    frozen_floor={frozen_floor:.3f}  true_floor={true_floor:.3f}  shift={floor_shift:+.3f}")
    print(f"    opt_Kd={opt_Kd:.3f}  fsat={meta['fsat']:.2f}")
    print(f"    SR at frozen_floor with opt_Kd: {frozen_sr_per_kp.get(frozen_floor, 'n/a')}")

    return dict(
        rocket_id=rocket_id, keff=meta['keff'], latency=meta['lat'], Pi=meta['Pi'],
        fsat_floor=meta['fsat'], frozen_floor=frozen_floor, true_floor=true_floor,
        floor_shift=floor_shift, opt_Kd=opt_Kd,
        detail_rows=rows,
    )


def run():
    print("frozen_kd_artifact_test.py")
    print(f"  Seeds: {SEED0} - {SEED0 + N_SEEDS - 1}")
    print(f"  Designs: {len(TARGET_DESIGNS)}")
    print(f"  Kp x Kd grid: 5 x 9 = 45 cells per design")
    print(f"  Total sims: {len(TARGET_DESIGNS) * 45 * N_SEEDS}")

    print("\nLoading design pool (seed=8888)...")
    pool = sample_lhs(10000, seed=8888)
    pool_dict = {r: row for r, row in pool.set_index('rocket_id').iterrows()}

    results = []
    detail_rows_all = []

    for rocket_id, meta in TARGET_DESIGNS.items():
        if rocket_id not in pool_dict:
            print(f"  {rocket_id}: not in pool, skipping")
            continue
        r = _process_design(rocket_id, meta, pool_dict[rocket_id])
        if r is not None:
            detail_rows_all.extend(r.pop('detail_rows'))
            results.append(r)

    df_out = pd.DataFrame(results)
    df_detail = pd.DataFrame(detail_rows_all)

    df_out.to_csv(OUT_CSV, index=False)
    df_detail.to_csv(str(OUT_CSV).replace('.csv', '_detail.csv'), index=False)
    print(f"\nSaved summary to {OUT_CSV}")

    analyze(df_out, df_detail)


def analyze(df=None, df_detail=None):
    if df is None:
        df = pd.read_csv(OUT_CSV)
    if df_detail is None:
        df_detail = pd.read_csv(str(OUT_CSV).replace('.csv', '_detail.csv'))

    print("\n" + "=" * 65)
    print("FROZEN-Kd ARTIFACT ANALYSIS")
    print("=" * 65)
    print()
    print(f"{'Design':<8} {'keff':>6} {'lat':>4} {'Pi':>7} {'fsat':>6} "
          f"{'frozen_floor':>13} {'true_floor':>11} {'shift':>8} {'verdict':>12}")
    print("-" * 90)

    shifts_high_keff = []
    for _, row in df.sort_values('keff').iterrows():
        verdict = ''
        if abs(row.floor_shift) < 0.15:
            verdict = 'NEGLIGIBLE'
        elif abs(row.floor_shift) < 0.30:
            verdict = 'MODERATE'
        else:
            verdict = 'SIGNIFICANT'
        if row.keff >= 20:
            shifts_high_keff.append(row.floor_shift)
        print(f"  {row.rocket_id:<6} {row.keff:>6.1f} {int(row.latency):>4} {row.Pi:>7.0f} "
              f"{row.fsat_floor:>6.2f} {row.frozen_floor:>13.3f} {row.true_floor:>11.3f} "
              f"{row.floor_shift:>+8.3f} {verdict:>12}")

    print()
    print(f"rho(keff, floor_shift) = {spearmanr(df.keff, df.floor_shift).statistic:+.3f}  "
          f"p={spearmanr(df.keff, df.floor_shift).pvalue:.3f}")
    print(f"rho(Pi,   floor_shift) = {spearmanr(df.Pi, df.floor_shift).statistic:+.3f}  "
          f"p={spearmanr(df.Pi, df.floor_shift).pvalue:.3f}")
    print(f"rho(fsat, floor_shift) = {spearmanr(df.fsat_floor, df.floor_shift).statistic:+.3f}  "
          f"p={spearmanr(df.fsat_floor, df.floor_shift).pvalue:.3f}")
    print()

    # Decision rule
    n_sig_high_keff = sum(s > 0.30 for s in shifts_high_keff)
    print("PRE-SPECIFIED DECISION CRITERIA:")
    print(f"  Designs with keff >= 20: {len(shifts_high_keff)}")
    print(f"  Of those with floor_shift > 0.30: {n_sig_high_keff}")
    if max(abs(df.floor_shift)) < 0.15:
        print("  VERDICT: NEGLIGIBLE -- frozen Kd does not bias floor measurement")
    elif n_sig_high_keff >= 3:
        print("  VERDICT: SIGNIFICANT -- frozen Kd inflates floor for high-keff designs; rerun window_ratio with Kd sweep")
    elif n_sig_high_keff >= 1 or df.floor_shift.abs().max() > 0.30:
        print("  VERDICT: MODERATE -- frozen Kd has localized bias; add caveat to floor formula at keff > 20")
    else:
        print("  VERDICT: NEGLIGIBLE -- frozen Kd does not materially bias floor measurement")

    # Show best Kd vs opt_Kd per design at the floor
    print()
    print("Best Kd at the frozen floor Kp (showing whether opt_Kd is actually optimal):")
    print(f"{'Design':<8} {'keff':>6} {'opt_Kd':>8} {'best_Kd_at_floor':>18} {'ratio':>8}")
    print("-" * 55)
    for _, row in df.sort_values('keff').iterrows():
        sub = df_detail[(df_detail.rocket_id == row.rocket_id) &
                        (df_detail.Kp.between(row.frozen_floor * 0.8, row.frozen_floor * 1.25))]
        if len(sub) == 0:
            continue
        best_kd = sub.loc[sub.sr.idxmax(), 'Kd']
        ratio = best_kd / row.opt_Kd
        print(f"  {row.rocket_id:<6} {row.keff:>6.1f} {row.opt_Kd:>8.3f} {best_kd:>18.3f} {ratio:>8.3f}")


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        analyze()
    else:
        run()
