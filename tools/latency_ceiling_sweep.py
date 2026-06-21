"""
tools/latency_ceiling_sweep.py

Directly measures how the gain CEILING and FLOOR change as a function of
latency_steps alone, holding everything else fixed.

Motivation: the current ceiling formula is Kp_max ≈ 380/latency_steps (empirical).
The DIPDT theoretical prediction gives ceiling ∝ latency^-1.0 to ^-1.1.
The window_ratio regression measures a MIXTURE of td and latency effects.
This experiment ISOLATES the latency exponent by:
  - Fixing 3 reference designs at td ≈ 50, 120, 250 rad/s² (one easy, two at-risk)
  - Varying only latency_steps over {1, 2, 3, 4, 5, 6}
  - Running a 16-point Kp sweep for each (Iyy, latency) combination
  - Fitting log(ceiling) ~ log_latency and log(floor) ~ log_latency separately

This gives clean, unconfounded latency exponent estimates.

Seed isolation: uses seeds 21001-21015 (disjoint from all prior experiments)
Output: experiments/results/latency_ceiling_sweep_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy.stats import pearsonr

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# 3 reference designs at td ≈ 50 / 120 / 250 rad/s²; only Iyy and latency_steps vary.
# F15_AVG_THRUST_N=14.4, motor_scale=1.8: T_avg=25.92N
# CU_TO_RAD=pi/180*15/12=0.02182; u_max=12*(12/15)=9.6 CU
# keff = 25.92*0.02182*0.25/Iyy = 0.1414/Iyy
# td = keff*9.6 = 1.358/Iyy  →  Iyy = 1.358/td

FIXED = dict(
    mass=0.80, static_margin=+0.10, Cm_alpha=-50.0,
    control_effectiveness=8.0, motor_scale=1.8, servo_slew_deg_s=120.0,
    max_gimbal_deg=12.0, deadband=0.05, backlash=0.10,
    wind_strength=0.20,
)

TD_TARGETS = [50, 120, 250]
IYY_VALUES = [round(1.358 / td, 5) for td in TD_TARGETS]   # [0.02716, 0.01132, 0.00543]

LATENCY_VALUES = [1, 2, 3, 4, 5, 6]

# Kp sweep: 16 log-spaced from 0.5 to 480
KP_SWEEP      = np.geomspace(0.5, 480.0, 16)
KD_FIXED      = 2.0                 # fixed Kd (same as matched_config)
N_SEEDS       = 15
SEED_START    = 21001               # disjoint from all prior experiments
SR_THRESH     = 0.80

OUT_CSV = Path('experiments/results/latency_ceiling_sweep_py.csv')


def _physics_cfg() -> FidelityConfig:
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def compute_td(Iyy: float) -> float:
    T_avg = F15_AVG_THRUST_N * FIXED['motor_scale']
    keff  = T_avg * CU_TO_RAD * L_NOZZLE / Iyy
    u_max = FIXED['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff * u_max)


def compute_keff(Iyy: float) -> float:
    T_avg = F15_AVG_THRUST_N * FIXED['motor_scale']
    return float(T_avg * CU_TO_RAD * L_NOZZLE / Iyy)


def _eval_cell(Iyy: float, latency: int) -> dict:
    row = dict(FIXED)
    row['Iyy'] = Iyy
    row['latency_steps'] = latency

    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    sweep_seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    sr_vals = []
    for Kp in KP_SWEEP:
        pid = PIDParams(Kp=float(Kp), Kd=KD_FIXED, Ki=0.0,
                        u_max=act.u_max, i_lim=act.u_max)
        agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s)
                          for s in sweep_seeds])
        sr_vals.append(agg['success_rate'])

    sr_arr  = np.array(sr_vals)
    passing = KP_SWEEP[sr_arr >= SR_THRESH]
    if len(passing) == 0:
        kp_floor = kp_ceiling = window_ratio = float('nan')
        floor_cens = ceil_cens = False
    else:
        kp_floor   = float(passing.min())
        kp_ceiling = float(passing.max())
        window_ratio = kp_ceiling / kp_floor
        floor_cens   = kp_floor   <= KP_SWEEP[0]  * 1.05
        ceil_cens    = kp_ceiling >= KP_SWEEP[-1] * 0.95

    td   = compute_td(Iyy)
    keff = compute_keff(Iyy)
    return dict(
        Iyy            = Iyy,
        td             = td,
        keff           = keff,
        latency_steps  = latency,
        kp_floor       = kp_floor,
        kp_ceiling     = kp_ceiling,
        window_ratio   = window_ratio,
        floor_censored = floor_cens,
        ceil_censored  = ceil_cens,
        sr_curve       = sr_vals,  # for diagnostic inspection
    )


def main():
    cells = [(iyy, lat) for iyy in IYY_VALUES for lat in LATENCY_VALUES]
    n_sims = len(cells) * len(KP_SWEEP) * N_SEEDS
    print(f"Latency ceiling sweep: {len(cells)} cells × {len(KP_SWEEP)} Kp × {N_SEEDS} seeds = {n_sims} sims")
    print(f"  td targets: {TD_TARGETS} rad/s²")
    print(f"  Iyy values: {IYY_VALUES}")
    print(f"  Latency: {LATENCY_VALUES}")
    for Iyy in IYY_VALUES:
        print(f"  td(Iyy={Iyy}) = {compute_td(Iyy):.1f} rad/s²")
    print("Running...")

    results_raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_cell)(iyy, lat) for iyy, lat in cells
    )

    # Drop the sr_curve column before saving (too big for CSV)
    rows = []
    for r in results_raw:
        r2 = dict(r)
        r2.pop('sr_curve', None)
        rows.append(r2)
    out = pd.DataFrame(rows)
    out.to_csv(OUT_CSV, index=False)

    print(f"\n=== LATENCY CEILING SWEEP RESULTS ===")
    for iyy in IYY_VALUES:
        sub = out[out['Iyy'] == iyy].sort_values('latency_steps')
        td = compute_td(iyy)
        keff = compute_keff(iyy)
        print(f"\ntd={td:.0f} rad/s²  (keff={keff:.2f}, Iyy={iyy}):")
        print(f"  {'lat':4} {'floor':8} {'ceiling':9} {'ratio':8} {'fl_cens':8} {'cl_cens':8}")
        for _, r in sub.iterrows():
            print(f"  {r['latency_steps']:4.0f} {r['kp_floor']:8.1f} {r['kp_ceiling']:9.1f} "
                  f"{r['window_ratio']:8.1f}× {str(r['floor_censored']):8} {str(r['ceil_censored']):8}")

    # Fit log(ceiling) ~ log(latency) for each td level
    print("\nCeiling exponent fits (log_ceiling ~ log_latency):")
    print("  Theory predicts exponent ~= -1.0 to -1.1")
    for iyy in IYY_VALUES:
        sub = out[(out['Iyy'] == iyy) & ~out['ceil_censored'] & out['kp_ceiling'].notna()]
        if len(sub) < 3:
            print(f"  td={compute_td(iyy):.0f}: insufficient non-censored points")
            continue
        llat = np.log(sub['latency_steps'].values.astype(float))
        lc   = np.log(sub['kp_ceiling'].values)
        X = np.c_[np.ones(len(sub)), llat]
        beta, *_ = np.linalg.lstsq(X, lc, rcond=None)
        ss_res = np.sum((lc - X@beta)**2)
        ss_tot = np.sum((lc - lc.mean())**2)
        r2 = 1 - ss_res/ss_tot if ss_tot > 0 else 0
        r, _ = pearsonr(llat, lc)
        print(f"  td={compute_td(iyy):4.0f}: exp={beta[1]:+.3f}  R2={r2:.3f}  r={r:+.3f}  "
              f"(intercept={beta[0]:.3f} -> K_u~={np.exp(beta[0]):.1f}/lat^{-beta[1]:.2f})")

    print("\nFloor exponent fits (log_floor ~ log_latency):")
    print("  Theory: floor driven by keff, not latency -> expect near-zero exponent")
    for iyy in IYY_VALUES:
        sub = out[(out['Iyy'] == iyy) & ~out['floor_censored'] & out['kp_floor'].notna()]
        if len(sub) < 3:
            print(f"  td={compute_td(iyy):.0f}: insufficient non-censored points")
            continue
        llat = np.log(sub['latency_steps'].values.astype(float))
        lf   = np.log(sub['kp_floor'].values)
        X = np.c_[np.ones(len(sub)), llat]
        beta, *_ = np.linalg.lstsq(X, lf, rcond=None)
        ss_res = np.sum((lf - X@beta)**2)
        ss_tot = np.sum((lf - lf.mean())**2)
        r2 = 1 - ss_res/ss_tot if ss_tot > 0 else 0
        r, _ = pearsonr(llat, lf)
        print(f"  td={compute_td(iyy):4.0f}: exp={beta[1]:+.3f}  R2={r2:.3f}  r={r:+.3f}")

    print(f"\nSaved: {OUT_CSV}")


if __name__ == '__main__':
    main()
