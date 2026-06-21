"""
tools/exp1_stress_test.py

Stress-test version of Exp1 with three simultaneous changes:

  1. LATENCY EXTENSION: latency_steps in [1, 12] (was [1, 6])
     Represents Arduino/I2C hobbyist setups: 50-100 Hz control loops = 10-20 ms
     per step at 200 Hz sim rate, equivalent to lat=4-8 or more.
     At lat=12: Kp_max ~ 380/12 = 32 (most authority-heavy designs become FRAGILE).

  2. IYY-MASS PHYSICAL FILTER: enforce Iyy >= mass * 0.010
     A uniform rod of 0.5m has Iyy/mass = 0.0208. Iyy < mass*0.010 requires
     mass to be unrealistically concentrated near the CG axis. This removes
     the most physically implausible combinations (e.g., 1.2 kg rocket, Iyy=0.005).

  3. TIGHTER SUCCESS CRITERION: uses band_20 (max_theta < 20 deg) instead of 70 deg
     via the STRICT_CRITERION flag. When True, success = band_20_pass (the existing
     SimResult field), which represents precision attitude hold, not just "didn't fall".

Output: experiments/results/exp1_stress_test_py.csv
Compare against: experiments/results/exp1_regime_index_py.csv (original n=2400)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.stats import qmc
from joblib import Parallel, delayed
from sklearn.metrics import roc_auc_score

from design_space import (
    DESIGN_NAMES, DESIGN_LO, DESIGN_HI,
    build_plant, build_actuator, build_sensor, build_disturbance,
    build_scenario, classify_regime, estimate_lambda_aero, REF,
    estimate_p_unstable,
)
from plant_dynamics import PlantParams
from units import F15_AVG_THRUST_N, FRAGILE_SUCCESS_RATE, ROBUSTNESS_SUCCESS_RATE
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from simulator import simulate, PIDParams as _PIDParams

# Import private helpers from experiment_runner
from experiment_runner import (
    autotune_continuous, _run_one, _aggregate,
    UNDER_SCALE, OVER_SCALE,
)

ROOT       = Path(__file__).resolve().parents[1]
RESULT_DIR = ROOT / "experiments" / "results"
OUT_CSV    = RESULT_DIR / "exp1_stress_test_py.csv"

# ── Stress-test design space bounds ─────────────────────────────────────────
# Only change: latency upper bound raised from 6 → 12
STRESS_LO = DESIGN_LO.copy()
STRESS_HI  = DESIGN_HI.copy()

LAT_IDX = DESIGN_NAMES.index("latency_steps")
STRESS_HI[LAT_IDX] = 12.0   # was 6.0

# Physical Iyy-mass consistency constraint
IYY_IDX  = DESIGN_NAMES.index("Iyy")
MASS_IDX = DESIGN_NAMES.index("mass")
IYY_MASS_RATIO_MIN = 0.010   # Iyy >= mass * 0.010

# Tighter criterion flag
# True  → success = band_20_pass (max_theta < 20 deg)
# False → success = standard (max_theta < 70 deg, etc.)
STRICT_CRITERION = False  # Use standard 30/70-deg criterion to isolate latency effect

N_DESIGNS = 2400
SEED      = 99   # different seed → fresh LHS, not correlated with original


def sample_lhs_stress(n: int, seed: int = 99) -> pd.DataFrame:
    """
    LHS sample with extended latency [1,12] and Iyy-mass physical filter.
    """
    rng_seed = seed
    collected = []
    target = n

    while len(collected) < target:
        sampler = qmc.LatinHypercube(d=len(DESIGN_NAMES), seed=rng_seed)
        batch   = max(target * 3, 1000)   # oversample more — two filters
        X = qmc.scale(sampler.random(batch), STRESS_LO, STRESS_HI)

        # Filter 1: T/W > 1
        T_eff    = F15_AVG_THRUST_N * X[:, DESIGN_NAMES.index("motor_scale")]
        W        = X[:, MASS_IDX] * 9.81
        tw_ok    = T_eff > W

        # Filter 2: Iyy >= mass * IYY_MASS_RATIO_MIN
        iyy_ok   = X[:, IYY_IDX] >= X[:, MASS_IDX] * IYY_MASS_RATIO_MIN

        valid    = tw_ok & iyy_ok
        collected.extend(X[valid])
        rng_seed += 1

    X_out = np.array(collected[:target])
    df = pd.DataFrame(X_out, columns=DESIGN_NAMES)
    df["latency_steps"] = df["latency_steps"].round().astype(int).clip(1, 12)

    df["lambda_aero"] = df.apply(
        lambda r: estimate_lambda_aero(r["static_margin"], r["Cm_alpha"], r["motor_scale"]),
        axis=1,
    )
    df["p_unstable"] = df["lambda_aero"].apply(lambda lam: float(np.sqrt(max(0.0, lam))))
    df["aerodynamically_stable"] = df["static_margin"] < 0.0
    df["tw_ratio"] = (F15_AVG_THRUST_N * df["motor_scale"]) / (df["mass"] * 9.81)
    df["design_id"] = np.arange(1, n + 1)
    df["rocket_id"] = [f"S{i:04d}" for i in range(1, n + 1)]   # "S" for stress-test
    return df


def _success_from_result(r: dict) -> bool:
    """Return success under strict or standard criterion."""
    if STRICT_CRITERION:
        # band_20_pass: max_theta < 20 deg
        return r["band_20_pass"]
    else:
        return r["success"]


def _aggregate_stress(runs: list) -> dict:
    """Like experiment_runner._aggregate but uses strict criterion if enabled."""
    base = _aggregate(runs)
    if STRICT_CRITERION:
        # Override success_rate with band_20_rate
        base["success_rate"] = base["band_20_rate"]
    return base


def _eval_design_stress(row: dict) -> dict:
    """Evaluate one design in the stress-test configuration."""
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    physics_cfg = FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, physics_cfg)

    # Autotune: success criterion as configured
    # For strict mode, autotune still uses standard success (it selects for attitude hold)
    # but the evaluation uses band_20. This is deliberate: autotune finds best gain for
    # "surviving flight", then we ask "does it also meet the 20-deg precision criterion?"
    best_Kp, best_Kd = autotune_continuous(plant, act, sen, dis, sc)

    def eval_nom(Kp, Kd):
        pid  = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in (1, 2, 3)]
        return _aggregate_stress(runs)

    nominal = eval_nom(best_Kp, best_Kd)
    under   = eval_nom(UNDER_SCALE * best_Kp, UNDER_SCALE * best_Kd)
    over    = eval_nom(OVER_SCALE  * best_Kp, OVER_SCALE  * best_Kd)

    n_pass = (
        int(nominal["success_rate"] >= FRAGILE_SUCCESS_RATE)
        + int(under["success_rate"]  >= ROBUSTNESS_SUCCESS_RATE)
        + int(over["success_rate"]   >= ROBUSTNESS_SUCCESS_RATE)
    )
    robustness = n_pass / 3.0

    label, code = classify_regime(
        nominal["success_rate"],
        nominal["rms_error_deg"],
        nominal["u_cmd_sat_frac"],
        nominal["slew_sat_frac"],
        nominal["settling_time_s"],
        nominal["oscillation_score"],
        robustness,
    )

    return dict(
        rocket_id          = row["rocket_id"],
        design_id          = row["design_id"],
        p_unstable         = row["p_unstable"],
        mass               = row["mass"],
        Iyy                = row["Iyy"],
        static_margin      = row["static_margin"],
        Cm_alpha           = row["Cm_alpha"],
        control_effectiveness = row["control_effectiveness"],
        motor_scale        = row["motor_scale"],
        servo_slew_deg_s   = row["servo_slew_deg_s"],
        max_gimbal_deg     = row["max_gimbal_deg"],
        latency_steps      = row["latency_steps"],
        deadband           = row["deadband"],
        backlash           = row["backlash"],
        wind_strength      = row["wind_strength"],
        best_Kp            = best_Kp,
        best_Kd            = best_Kd,
        nominal_success_rate = nominal["success_rate"],
        under_success_rate   = under["success_rate"],
        over_success_rate    = over["success_rate"],
        robustness           = robustness,
        n_success_gain_sets  = n_pass,
        nominal_diverge_rate = nominal["diverge_rate"],
        nominal_band_30_rate = nominal["band_30_rate"],
        nominal_band_20_rate = nominal["band_20_rate"],
        nominal_band_10_rate = nominal["band_10_rate"],
        under_diverge_rate   = under["diverge_rate"],
        over_diverge_rate    = over["diverge_rate"],
        rms_error_deg        = nominal["rms_error_deg"],
        peak_error_deg       = nominal["peak_error_deg"],
        end_error_deg        = nominal["end_error_deg"],
        max_theta_deg        = nominal["max_theta_deg"],
        u_cmd_sat_frac       = nominal["u_cmd_sat_frac"],
        slew_sat_frac        = nominal["slew_sat_frac"],
        settling_time_s      = nominal["settling_time_s"],
        oscillation_score    = nominal["oscillation_score"],
        under_u_sat_frac     = under["u_cmd_sat_frac"],
        over_u_sat_frac      = over["u_cmd_sat_frac"],
        regime_label         = label,
        regime_code          = code,
        strict_criterion     = STRICT_CRITERION,
    )


def main():
    print("="*70)
    print("EXP1 STRESS TEST")
    print(f"  Latency: [1, 12] steps (was [1, 6])")
    print(f"  Iyy filter: Iyy >= mass * {IYY_MASS_RATIO_MIN}")
    print(f"  Success criterion: {'20-deg (band_20)' if STRICT_CRITERION else 'standard (70-deg)'}")
    print(f"  n_designs: {N_DESIGNS}, seed: {SEED}")
    print("="*70)

    print("\nSampling designs...")
    designs = sample_lhs_stress(N_DESIGNS, SEED)

    print(f"Latency distribution:")
    print(designs['latency_steps'].value_counts().sort_index())
    print(f"\nIyy/mass ratio: min={( designs['Iyy']/designs['mass']).min():.4f}  "
          f"median={( designs['Iyy']/designs['mass']).median():.4f}")
    print(f"T/W ratio: min={designs['tw_ratio'].min():.2f}  "
          f"median={designs['tw_ratio'].median():.2f}")

    # Quick theta_ddot preview
    F15 = 14.4; CU_TO_RAD = (np.pi/180)*(15/12); L = 0.25
    td = F15 * designs['motor_scale'] * CU_TO_RAD * L / designs['Iyy'] * designs['max_gimbal_deg'] * 12/15
    print(f"\ntheta_ddot_max preview: min={td.min():.1f}  median={td.median():.1f}  "
          f"max={td.max():.1f} rad/s^2")
    print(f"  Designs with td > 55 rad/s^2 (FRAGILE-flagged): {(td>55).sum()} ({100*(td>55).mean():.1f}%)")

    rows = designs.to_dict("records")
    print(f"\nRunning {len(rows)} designs...")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design_stress)(row) for row in rows
    )

    df = pd.DataFrame(results)
    counts = df["regime_label"].value_counts()
    n = len(df)
    print("\n" + "="*70)
    print("STRESS-TEST REGIME COUNTS")
    print("="*70)
    for r in ['EASY','MARGINAL','FRAGILE','INFEASIBLE']:
        c = counts.get(r, 0)
        print(f"  {r:12s}: {c:5d}  ({100*c/n:.1f}%)")

    print(f"\nOriginal (n=2400, lat[1-6], standard 30-deg criterion):")
    print(f"  EASY=2347 (97.8%), MARGINAL=5 (0.2%), FRAGILE=45 (1.9%), INFEASIBLE=3 (0.1%)")

    # AUC comparison
    F15a = 14.4; CU_TO_RADa = (np.pi/180)*(15/12); La = 0.25
    td_out = F15a * df['motor_scale'] * CU_TO_RADa * La / df['Iyy'] * df['max_gimbal_deg'] * 12/15
    y = (df['regime_label'] == 'FRAGILE').astype(int).values

    if y.sum() >= 5:
        from sklearn.metrics import roc_auc_score
        auc_td   = roc_auc_score(y, td_out)
        auc_comb = roc_auc_score(y, np.log(td_out * df['latency_steps']))
        print(f"\nAUC (theta_ddot predicting FRAGILE): {auc_td:.3f}")
        print(f"AUC (theta_ddot x latency):          {auc_comb:.3f}")
        print(f"(Original: 0.944 / 0.972)")

        # Latency distribution of FRAGILE
        frag = df[df['regime_label'] == 'FRAGILE']
        print(f"\nFRAGILE latency distribution:")
        print(frag['latency_steps'].value_counts().sort_index())
        print(f"FRAGILE theta_ddot: mean={td_out[y==1].mean():.1f}  "
              f"median={np.median(td_out[y==1]):.1f}  min={td_out[y==1].min():.1f}")
        print(f"EASY theta_ddot:    mean={td_out[y==0].mean():.1f}  "
              f"median={np.median(td_out[y==0]):.1f}")

    df.to_csv(OUT_CSV, index=False)
    print(f"\nSaved: {OUT_CSV}  ({len(df)} rows)")
    print("Done.")


if __name__ == '__main__':
    main()
