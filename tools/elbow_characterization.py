"""
tools/elbow_characterization.py

Follow-up to continuous_margin_regression.py. That experiment (n=130, td-decile
stratified) found NOT a smooth continuum but a sharp elbow: below td~90 rad/s^2,
over-robustness margin is ~0.98 (essentially perfect, n=122, well-powered) with
only 14%% showing any blip (consistent with 15-seed sampling noise on a near-1.0
design); above td~90, mean margin drops to 0.78 with 87.5%% showing degradation --
but that finding rested on only 8 designs above the elbow. This script adds
targeted resolution exactly where it's needed: ~20 designs per bin across
[40,80), [80,120), [120,180), [180,300), [300,1000) rad/s^2, same finer joint
Kp x Kd gain search + 15-seed continuous over/under/nominal evaluation.

Output: experiments/results/elbow_characterization_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario, sample_lhs
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate, UNDER_SCALE, OVER_SCALE
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

KP_MIN, KP_MAX, N_KP = 1.0, 320.0, 18
KD_MIN, KD_MAX, N_KD = 1.0, 64.0, 7
SEARCH_SEEDS  = [1, 2, 3]
N_FINAL_SEEDS = 15
FINAL_SEED_START = 7001

N_POOL     = 4000
BINS       = [(40, 80), (80, 120), (120, 180), (180, 300), (300, 1000)]
PER_BIN    = 20

OUT_CSV = Path('experiments/results/elbow_characterization_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def compute_td(row: dict) -> float:
    T_avg = F15_AVG_THRUST_N * row['motor_scale']
    keff_full = T_avg * CU_TO_RAD * L_NOZZLE / row['Iyy']
    u_max = row['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff_full * u_max)


def _joint_search(plant, act, sen, dis, sc):
    kp_grid = np.geomspace(KP_MIN, KP_MAX, N_KP)
    kd_grid = np.geomspace(KD_MIN, KD_MAX, N_KD)
    best_kp, best_kd, best_sr, best_rms = float(kp_grid[0]), float(kd_grid[0]), -1.0, float("inf")
    for Kp in kp_grid:
        for Kd in kd_grid:
            pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
            runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in SEARCH_SEEDS]
            agg = _aggregate(runs)
            sr, rms = agg["success_rate"], agg["rms_error_deg"]
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr, best_rms, best_kp, best_kd = sr, rms, float(Kp), float(Kd)
    return best_kp, best_kd


def _eval_design(row: dict) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    new_Kp, new_Kd = _joint_search(plant, act, sen, dis, sc)
    seeds = list(range(FINAL_SEED_START, FINAL_SEED_START + N_FINAL_SEEDS))

    def eval_cond(Kp, Kd):
        pid  = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
        return _aggregate(runs)

    nominal = eval_cond(new_Kp, new_Kd)
    under   = eval_cond(UNDER_SCALE * new_Kp, UNDER_SCALE * new_Kd)
    over    = eval_cond(OVER_SCALE  * new_Kp, OVER_SCALE  * new_Kd)

    return dict(
        rocket_id    = row['rocket_id'],
        td           = row['td'],
        latency_steps= row['latency_steps'],
        Iyy          = row['Iyy'],
        motor_scale  = row['motor_scale'],
        max_gimbal_deg = row['max_gimbal_deg'],
        new_Kp       = new_Kp,
        new_Kd       = new_Kd,
        nom_sr       = nominal['success_rate'],
        under_sr     = under['success_rate'],
        over_sr      = over['success_rate'],
        margin       = min(under['success_rate'], over['success_rate']),
        nom_rms      = nominal['rms_error_deg'],
    )


def main():
    pool = sample_lhs(N_POOL, seed=777)
    pool['td'] = pool.apply(lambda r: compute_td(r.to_dict()), axis=1)

    rng = np.random.RandomState(321)
    chosen = []
    for lo, hi in BINS:
        sub = pool[(pool['td'] >= lo) & (pool['td'] < hi)]
        n_take = min(PER_BIN, len(sub))
        chosen.append(sub.sample(n=n_take, random_state=rng))
    selected = pd.concat(chosen).reset_index(drop=True)

    sims_per_design = N_KP * N_KD * len(SEARCH_SEEDS) + 3 * N_FINAL_SEEDS
    print(f"Elbow characterization: {len(selected)} designs across bins {BINS}")
    print(f"  Total sims: ~{int(sims_per_design*len(selected))}")

    rows = selected.to_dict('records')
    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design)(row) for row in rows
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)

    out_sorted = out.sort_values('td')
    bins_report = [0, 40, 80, 120, 180, 300, 1000]
    out_sorted['bin'] = pd.cut(out_sorted['td'], bins_report)
    g = out_sorted.groupby('bin').agg(
        n=('td', 'size'), mean_td=('td', 'mean'),
        mean_over=('over_sr', 'mean'), frac_below_080=('over_sr', lambda s: (s < 0.80).mean()),
    )
    print("\n=== ELBOW CHARACTERIZATION ===")
    print(g.to_string())
    print(f"\nSaved: {OUT_CSV}")


if __name__ == '__main__':
    main()
