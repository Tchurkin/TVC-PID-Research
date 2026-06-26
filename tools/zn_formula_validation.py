"""
tools/zn_formula_validation.py  (2026-06-23)

VALIDATION OF THE ZERO-CALIBRATION Z-N TUNING FORMULA

DERIVATION (from DIPDT ceiling analysis):
  K_u        = 380 / latency_steps   (empirical ceiling law, keff-independent)
  T_u        = 4 × latency_steps × dt = 4 × lat × 0.005 s   (DIPDT oscillation period)
  Kp_ZN      = 0.6 × K_u = 228 / latency_steps
  Kd_ZN      = Kp × T_u / 8 = (228/lat) × (4×lat×0.005) / 8 = 228 × 0.005 / 2 = 0.570

KEY PREDICTION: Kd_ZN = 0.57 is UNIVERSAL — lat cancels completely.
  A builder needs only one parameter (latency_steps) to compute both Kp and Kd.
  keff, Iyy, motor thrust, gimbal angle are NOT needed.

EXPERIMENT:
  30 designs from exp1_final_population stratified by Pi_keff (6 bins × 5 designs)
  Pi bins: [30,150), [150,350), [350,800), [800,2000), [2000,5000), [5000+)
  Three gain settings per design:
    A. Kp_cons=190/lat,  Kd=0.57  (conservative, 0.5×K_u)
    B. Kp_ZN  =228/lat,  Kd=0.57  (Z-N formula, 0.6×K_u)
    C. Kp_best from exp1_final_correction (optimal, for reference)
  20 full-physics eval seeds (107001-107020, disjoint from all prior)
  Fixed wind=0.25 for comparability; same full-physics fidelity stack.

PREDICTION:
  A and B: SR >= 0.80 for all non-INFEASIBLE designs (Kp < ceiling ≈ 380/lat)
  Gap (C - B) shows cost of using formula vs optimal tuning.
  At Pi > 5000: formula may fail (extreme designs).

FILES OUT:
  experiments/results/zn_formula_validation_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario)
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25
DT        = 0.005

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

N_EVAL      = 20
EVAL_SEED0  = 107001
EVAL_SEEDS  = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))

FIXED_WIND = 0.25

PI_BINS    = [(30, 150), (150, 350), (350, 800),
              (800, 2000), (2000, 5000), (5000, 1e7)]
N_PER_BIN  = 5


def _keff(row):
    return F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']


def _build(row):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    dis.gust_std = FIXED_WIND
    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fid)
    return plant, act, sen, dis, sc


def _run(row, Kp, Kd, seeds):
    plant, act, sen, dis, sc = _build(row)
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    return float(np.mean([r.success for r in results]))


def process_design(row):
    lat      = int(row['latency_steps'])
    keff     = _keff(row)
    Pi       = keff * lat ** 2

    kp_cons  = 190.0 / lat
    kp_zn    = 228.0 / lat
    kd_zn    = 0.57

    sr_cons  = _run(row, kp_cons, kd_zn, EVAL_SEEDS)
    sr_zn    = _run(row, kp_zn,   kd_zn, EVAL_SEEDS)
    # Use the stored best gains as reference (re-evaluated on fresh seeds/wind)
    sr_best  = _run(row, row['best_Kp'], row['best_Kd'], EVAL_SEEDS)

    print(f"  {row['rocket_id']} lat={lat} Pi={Pi:.0f}  "
          f"cons={sr_cons:.2f}  ZN={sr_zn:.2f}  best={sr_best:.2f}  "
          f"Kp_ZN={kp_zn:.1f}  Kp_best={row['best_Kp']:.1f}")

    return dict(
        rocket_id  = row['rocket_id'],
        lat        = lat,
        keff       = round(keff, 3),
        Pi         = round(Pi, 1),
        final_label= row.get('final_label', 'UNKNOWN'),
        kp_cons    = round(kp_cons, 2),
        kp_zn      = round(kp_zn, 2),
        kd_zn      = kd_zn,
        kp_best    = round(row['best_Kp'], 2),
        kd_best    = round(row['best_Kd'], 2),
        sr_cons    = round(sr_cons, 4),
        sr_zn      = round(sr_zn, 4),
        sr_best    = round(sr_best, 4),
        gap_cons   = round(sr_best - sr_cons, 4),
        gap_zn     = round(sr_best - sr_zn, 4),
    )


if __name__ == '__main__':
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')
    pop = pop.dropna(subset=['best_Kp', 'best_Kd', 'final_label'])
    pop['keff_full'] = pop.apply(_keff, axis=1)
    pop['Pi_keff']   = pop['keff_full'] * pop['latency_steps'] ** 2

    # Stratified selection: 5 designs per Pi bin
    selected = []
    rng = np.random.default_rng(107999)
    for lo, hi in PI_BINS:
        pool = pop[(pop['Pi_keff'] >= lo) & (pop['Pi_keff'] < hi)].copy()
        # Prefer diverse latency spread within each bin
        pool = pool.sort_values(['latency_steps', 'Pi_keff'])
        if len(pool) == 0:
            print(f'  Pi [{lo:.0f}, {hi:.0f}): NO DESIGNS')
            continue
        n_take = min(N_PER_BIN, len(pool))
        if n_take < N_PER_BIN:
            step = max(1, len(pool) // n_take)
            idx  = np.arange(0, len(pool), step)[:n_take]
            chosen = pool.iloc[idx]
        else:
            # Evenly spread across the pool
            step = len(pool) // n_take
            idx  = np.arange(0, n_take) * step
            chosen = pool.iloc[idx]
        print(f'  Pi [{lo:>6.0f}, {hi:>8.0f}): {len(chosen)} designs  '
              f'Pi range [{chosen["Pi_keff"].min():.0f}, {chosen["Pi_keff"].max():.0f}]  '
              f'lat {sorted(chosen["latency_steps"].unique().tolist())}')
        selected.append(chosen)

    df_sel = pd.concat(selected).reset_index(drop=True)
    print(f'\nSelected {len(df_sel)} designs total')
    print(f'Eval seeds: {EVAL_SEED0}–{EVAL_SEED0 + N_EVAL - 1}  (n={N_EVAL})')
    print(f'Simulations: {len(df_sel) * 3 * N_EVAL:,} total\n')

    rows = Parallel(n_jobs=-1, verbose=5)(
        delayed(process_design)(df_sel.iloc[i])
        for i in range(len(df_sel))
    )

    out = pd.DataFrame(rows).sort_values('Pi').reset_index(drop=True)
    out.to_csv(RESULTS / 'zn_formula_validation_py.csv', index=False)
    print(f'\nSaved -> {RESULTS / "zn_formula_validation_py.csv"}')

    print('\n' + '='*65)
    print('Z-N FORMULA VALIDATION RESULTS')
    print('='*65)
    print(f'\nFormula: Kp=228/lat, Kd=0.57 (zero calibration; only need lat)')

    n = len(out)
    print(f'\nGlobal (n={n}):')
    print(f'  Conservative (190/lat):  mean SR={out["sr_cons"].mean():.3f}  '
          f'pct >= 0.80: {(out["sr_cons"]>=0.80).mean()*100:.0f}%')
    print(f'  Z-N formula (228/lat):   mean SR={out["sr_zn"].mean():.3f}  '
          f'pct >= 0.80: {(out["sr_zn"]>=0.80).mean()*100:.0f}%')
    print(f'  Best achievable:         mean SR={out["sr_best"].mean():.3f}')
    print(f'  Mean gap (best - ZN):    {out["gap_zn"].mean():.3f}')

    print('\nBy Pi tier:')
    for lo, hi in PI_BINS:
        sub = out[(out['Pi'] >= lo) & (out['Pi'] < hi)]
        if len(sub) == 0:
            continue
        zn_pass = (sub['sr_zn'] >= 0.80).sum()
        print(f'  Pi {lo:>6.0f}–{hi:>8.0f}  n={len(sub)}  '
              f'ZN SR={sub["sr_zn"].mean():.3f}  '
              f'pass={zn_pass}/{len(sub)}  '
              f'best={sub["sr_best"].mean():.3f}  '
              f'gap={sub["gap_zn"].mean():.3f}')

    rho, p = stats.spearmanr(out['Pi'], out['sr_zn'])
    print(f'\nrho(Pi, sr_ZN) = {rho:.3f}  p={p:.2e}')

    failures = out[out['sr_zn'] < 0.80]
    if len(failures) > 0:
        print(f'\nZ-N FAILURES (sr < 0.80):')
        print(failures[['rocket_id', 'lat', 'keff', 'Pi', 'final_label',
                          'kp_zn', 'sr_zn', 'sr_best']].to_string(index=False))
    else:
        print(f'\nZ-N FORMULA: 0 failures. Kp=228/lat, Kd=0.57 achieves SR >= 0.80 universally.')

    kd_compare = out[['kd_zn', 'kd_best']].describe()
    print(f'\nKd comparison:')
    print(f'  Kd_ZN   = 0.57 (formula, universal)')
    print(f'  Kd_best = {out["kd_best"].median():.2f} (population median)  '
          f'range [{out["kd_best"].min():.2f}, {out["kd_best"].max():.2f}]')
