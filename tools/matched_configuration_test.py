"""
tools/matched_configuration_test.py

Directly implements the single most valuable experiment identified by an
external review of this paper: hold every hardware/environment parameter
FIXED except one (Iyy), sweep that one parameter across the study's full
range, and for EACH resulting theta_ddot_max level, sweep Kp finely (fixed
Kd) to trace out a full success-rate-vs-Kp curve. This isolates the causal
variable cleanly -- no two configurations differ in more than Iyy -- which
no prior experiment in this project did (the boundary experiment compared
FRAGILE vs EASY designs that differed in several parameters at once).

This is still a SIMULATED experiment, not a real flight test. It does not
resolve the "all validation traces back to a simulator you built" objection.
What it does resolve is the weaker, but still real, objection that earlier
"FRAGILE vs EASY" comparisons (e.g. the boundary experiment) were comparing
different designs that happened to differ in theta_ddot_max among other
things, rather than a single airframe varied along one axis.

Output: experiments/results/matched_configuration_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# ── Fixed "airframe": everything held constant except Iyy ──────────────────
FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.5, servo_slew_deg_s=120.0,
    max_gimbal_deg=10.0, latency_steps=3, deadband=0.05, backlash=0.10,
    wind_strength=0.20,
)

IYY_VALUES = np.geomspace(0.005, 0.100, 9)   # spans the full design-space range
KP_GRID    = np.geomspace(1.0, 320.0, 25)
KD_FIXED   = 2.0
N_SEEDS    = 10
SEED_START = 11001
SR_THRESH  = 0.80

OUT_CSV = Path('experiments/results/matched_configuration_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def compute_td(Iyy: float) -> float:
    T_avg = F15_AVG_THRUST_N * FIXED['motor_scale']
    keff_full = T_avg * CU_TO_RAD * L_NOZZLE / Iyy
    u_max = FIXED['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff_full * u_max)


def _eval_point(Iyy: float, Kp: float) -> dict:
    row = dict(FIXED)
    row['Iyy'] = Iyy
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    pid = PIDParams(Kp=Kp, Kd=KD_FIXED, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    agg = _aggregate(runs)

    return dict(
        Iyy=Iyy, td=compute_td(Iyy), Kp=Kp,
        sr=agg['success_rate'], rms=agg['rms_error_deg'],
    )


def main():
    tasks = [(iyy, kp) for iyy in IYY_VALUES for kp in KP_GRID]
    print(f"Matched-configuration test: {len(IYY_VALUES)} Iyy levels x {len(KP_GRID)} Kp values "
          f"x {N_SEEDS} seeds = {len(tasks)*N_SEEDS} sims")
    print(f"theta_ddot_max range: [{compute_td(IYY_VALUES.max()):.1f}, {compute_td(IYY_VALUES.min()):.1f}] rad/s^2")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_point)(iyy, kp) for iyy, kp in tasks
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)

    print("\n=== WINDOW WIDTH vs theta_ddot_max (single airframe, only Iyy varied) ===")
    rows = []
    for iyy, sub in out.groupby('Iyy'):
        sub = sub.sort_values('Kp')
        td = sub['td'].iloc[0]
        passing = sub[sub['sr'] >= SR_THRESH]
        if len(passing) == 0:
            floor_kp, ceil_kp, window = float('nan'), float('nan'), 0.0
        else:
            floor_kp, ceil_kp = float(passing['Kp'].min()), float(passing['Kp'].max())
            window = ceil_kp / floor_kp
        rows.append(dict(Iyy=iyy, td=td, floor_kp=floor_kp, ceil_kp=ceil_kp, window_ratio=window))
        print(f"  Iyy={iyy:.4f}  td={td:6.1f} rad/s^2  floor={floor_kp:6.1f}  ceil={ceil_kp:6.1f}  window={window:6.1f}x")

    summary = pd.DataFrame(rows)
    summary.to_csv('experiments/results/matched_configuration_summary_py.csv', index=False)

    from scipy.stats import pearsonr
    valid = summary.dropna()
    r, p = pearsonr(np.log(valid['td']), np.log(valid['window_ratio']))
    print(f"\nr(log td, log window_ratio) = {r:.3f}  p={p:.4f}  (n={len(valid)} configurations)")
    print(f"\nSaved: {OUT_CSV}")
    print("Saved: experiments/results/matched_configuration_summary_py.csv")


if __name__ == '__main__':
    main()
