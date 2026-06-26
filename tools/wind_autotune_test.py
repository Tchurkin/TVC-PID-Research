"""
wind_autotune_test.py  —  Does adding wind to autotune fix the Pi calibration failure?

RESEARCH QUESTION:
  At high Pi (keff x latency^2 > 300), disturbance-free autotune returns Kp_simple >> real ceiling
  (380/latency). This causes 61% false rejection rate. Is the fix simply: add wind to the autotune?

HYPOTHESIS:
  - Wind creates a ceiling signal: at Kp > 380/latency, wind-driven limit cycles degrade SR
  - Wind creates a floor signal: at very low Kp, wind gusts exceed the success threshold
  - With wind, autotune converges near the real physics optimum regardless of Pi
  - Without wind (current): autotune scatters to extremes (320 for high keff, <5 for some designs)

PROTOCOL:
  20 designs selected from exp4_s2r_gains_final_py.csv:
    - 8 overtuned high-Pi (kp_simple >= 200, Pi > 200, false_rejection = 1)
    - 8 undertuned high-Pi (kp_simple <= 10, Pi > 200, false_rejection = 1)
    - 4 control designs (Pi < 100, false_rejection = 0)

  For each design, run autotune_continuous with:
    A. wind=False (simple model — current behavior)
    B. wind=True + latency=True, all other physics OFF (wind-only autotune)
    C. Full physics autotune (all modules ON)

  Evaluate all three Kp choices in full physics (15 seeds).

OUTCOME METRICS:
  - kp_nowind, kp_wind, kp_full
  - SR(kp_nowind), SR(kp_wind), SR(kp_full) in full physics
  - false_rejection_nowind, false_rejection_wind
  - overtuning severity: kp / Kp_ceiling

Seeds: autotune uses seeds 1+2 (internal); eval uses seeds 60001-60015 (all disjoint).
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from dataclasses import asdict

from experiment_runner import (
    _run_one, autotune_continuous,
    build_plant, build_actuator, build_sensor, build_disturbance,
    build_scenario, apply_fidelity_config,
    PIDParams,
)
from fidelity_config import FidelityConfig

# ── Constants ────────────────────────────────────────────────────────────────
F15_AVG    = 14.4
CU_TO_RAD  = np.pi / 180 * (15 / 12)
L_NOZZLE   = 0.25
EVAL_SEEDS = list(range(60001, 60016))  # 15 fresh seeds
SR_THRESH  = 0.80


def compute_keff(row):
    return F15_AVG * row["motor_scale"] * CU_TO_RAD * L_NOZZLE / row["Iyy"]


def eval_gains(kp, kd, plant, base_act, base_sen, base_dis, seeds):
    """Evaluate Kp/Kd in full physics over given seeds. Returns success_rate, mean_rms."""
    pid  = PIDParams(Kp=kp, Kd=kd, Ki=0.0, u_max=base_act.u_max, i_lim=base_act.u_max)
    full_cfg = FidelityConfig.full()
    sc   = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, scenario = apply_fidelity_config(base_act, base_sen, base_dis, sc, full_cfg)
    successes, rms_vals = [], []
    for seed in seeds:
        r = _run_one(pid, plant, act, sen, dis, scenario, seed=seed)
        successes.append(float(r["success"]))
        rms_vals.append(float(r["rms_error_deg"]))
    return np.mean(successes), np.mean(rms_vals)


def run_autotune(row, wind_on: bool, latency_on: bool = False):
    """
    Run autotune_continuous with specified physics.
    wind_on=False → simple model (current behavior)
    wind_on=True  → wind enabled, latency as specified
    """
    plant    = build_plant(row)
    base_act = build_actuator(row)
    base_sen = build_sensor(row)
    base_dis = build_disturbance(row)
    sc       = build_scenario(theta0_fixed_deg=10.0)

    cfg = FidelityConfig(
        wind=wind_on,
        latency=latency_on,
        backlash=False,
        slew=False,
        sensor_noise=False,
        thrust_var=False,
        deadband=False,
        nonlinear_aero=False,
        dyn_aero=False,
        thrust_curve=False,
        cg_shift=False,
    )
    act, sen, dis, scenario = apply_fidelity_config(base_act, base_sen, base_dis, sc, cfg)
    kp, kd = autotune_continuous(plant, act, sen, dis, scenario)
    return kp, kd


def process_design(row):
    plant    = build_plant(row)
    base_act = build_actuator(row)
    base_sen = build_sensor(row)
    base_dis = build_disturbance(row)

    keff = compute_keff(row)
    Pi   = keff * row["latency_steps"] ** 2
    Kp_ceiling = 380.0 / row["latency_steps"]

    # A: no-wind autotune (current behavior)
    kp_nowind, kd_nowind = run_autotune(row, wind_on=False, latency_on=False)

    # B: wind-only autotune (latency OFF — wind adds cost signal without phase lag confusion)
    kp_wind_nolat, kd_wind_nolat = run_autotune(row, wind_on=True, latency_on=False)

    # C: wind + latency autotune (both physical effects present)
    kp_wind_lat, kd_wind_lat = run_autotune(row, wind_on=True, latency_on=True)

    # Evaluate all three in full physics
    sr_nowind, rms_nowind = eval_gains(kp_nowind, kd_nowind, plant, base_act, base_sen, base_dis, EVAL_SEEDS)
    sr_wind_nolat, rms_wind_nolat = eval_gains(kp_wind_nolat, kd_wind_nolat, plant, base_act, base_sen, base_dis, EVAL_SEEDS)
    sr_wind_lat, rms_wind_lat = eval_gains(kp_wind_lat, kd_wind_lat, plant, base_act, base_sen, base_dis, EVAL_SEEDS)

    fr_nowind    = int(sr_nowind < SR_THRESH)
    fr_wind_nolat = int(sr_wind_nolat < SR_THRESH)
    fr_wind_lat  = int(sr_wind_lat < SR_THRESH)

    return dict(
        rocket_id       = row["rocket_id"],
        keff            = round(keff, 4),
        Pi              = round(Pi, 2),
        latency_steps   = int(row["latency_steps"]),
        Kp_ceiling      = round(Kp_ceiling, 1),
        wind_strength   = round(row["wind_strength"], 3),
        # Gains from each autotune mode
        kp_nowind       = kp_nowind,
        kd_nowind       = kd_nowind,
        kp_wind_nolat   = kp_wind_nolat,
        kd_wind_nolat   = kd_wind_nolat,
        kp_wind_lat     = kp_wind_lat,
        kd_wind_lat     = kd_wind_lat,
        # Full-physics SR for each gain choice
        sr_nowind       = round(sr_nowind, 4),
        sr_wind_nolat   = round(sr_wind_nolat, 4),
        sr_wind_lat     = round(sr_wind_lat, 4),
        rms_nowind      = round(rms_nowind, 3),
        rms_wind_nolat  = round(rms_wind_nolat, 3),
        rms_wind_lat    = round(rms_wind_lat, 3),
        # False rejections (full-physics SR < 0.80)
        fr_nowind       = fr_nowind,
        fr_wind_nolat   = fr_wind_nolat,
        fr_wind_lat     = fr_wind_lat,
        # Reference (from exp4 original run)
        kp_simple_orig  = row.get("kp_simple", np.nan),
        kp_full_orig    = row.get("kp_full", np.nan),
        fr_orig         = row.get("false_rejection_v2", np.nan),
        # Overtuning severity
        ot_nowind       = round(kp_nowind / Kp_ceiling, 2),
        ot_wind_nolat   = round(kp_wind_nolat / Kp_ceiling, 2),
        ot_wind_lat     = round(kp_wind_lat / Kp_ceiling, 2),
    )


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="experiments/results/wind_autotune_test_py.csv")
    args = parser.parse_args()

    # Load source data
    s2r = pd.read_csv("experiments/results/exp4_s2r_gains_final_py.csv")
    pop = pd.read_csv("experiments/results/exp1_final_population_py.csv")[["rocket_id", "td", "final_label"]]
    df  = s2r.merge(pop, on="rocket_id", how="left", suffixes=("", "_pop"))
    df["keff"] = F15_AVG * df["motor_scale"] * CU_TO_RAD * L_NOZZLE / df["Iyy"]
    df["Pi"]   = df["keff"] * df["latency_steps"] ** 2

    # Select target designs
    # 8 overtuned high-Pi false rejections (kp_simple >= 200, Pi > 200, FR=1)
    overtuned = df[(df.Pi > 200) & (df.false_rejection_v2 == 1) & (df.kp_simple >= 200)].copy()
    overtuned = overtuned.sort_values("Pi").drop_duplicates("latency_steps").head(4)
    extra_ot  = df[(df.Pi > 200) & (df.false_rejection_v2 == 1) & (df.kp_simple >= 200)].copy()
    extra_ot  = extra_ot.sort_values("Pi", ascending=False).head(4)
    overtuned = pd.concat([overtuned, extra_ot]).drop_duplicates("rocket_id")

    # 6 undertuned high-Pi false rejections (kp_simple <= 10, Pi > 200, FR=1)
    undertuned = df[(df.Pi > 200) & (df.false_rejection_v2 == 1) & (df.kp_simple <= 10)].copy()
    undertuned = undertuned.sort_values("Pi").head(6)

    # 4 control designs (Pi < 100, FR=0)
    controls   = df[(df.Pi < 100) & (df.false_rejection_v2 == 0)].copy()
    controls   = controls.sort_values("Pi", ascending=False).head(4)

    targets = pd.concat([overtuned, undertuned, controls]).drop_duplicates("rocket_id")
    targets["design_type"] = "unknown"
    targets.loc[targets.rocket_id.isin(overtuned.rocket_id), "design_type"] = "overtuned_hi_Pi"
    targets.loc[targets.rocket_id.isin(undertuned.rocket_id), "design_type"] = "undertuned_hi_Pi"
    targets.loc[targets.rocket_id.isin(controls.rocket_id),   "design_type"] = "control_lo_Pi"

    print(f"Running wind autotune test on {len(targets)} designs:")
    print(targets[["rocket_id", "Pi", "keff", "latency_steps", "kp_simple", "kp_full", "design_type"]].to_string(index=False))
    print()

    results = []
    for i, (_, row) in enumerate(targets.iterrows()):
        rtype = targets.loc[_, "design_type"] if _ in targets.index else "?"
        print(f"[{i+1}/{len(targets)}] {row.rocket_id} Pi={row.Pi:.0f} lat={int(row.latency_steps)} kp_simple={row.kp_simple:.0f} ({rtype})")
        res = process_design(row)
        res["design_type"] = rtype
        results.append(res)
        print(f"  nowind: Kp={res['kp_nowind']:.1f} (ceil={res['Kp_ceiling']:.0f}) SR={res['sr_nowind']:.3f}  FR={res['fr_nowind']}")
        print(f"  wind_nolat: Kp={res['kp_wind_nolat']:.1f} SR={res['sr_wind_nolat']:.3f}  FR={res['fr_wind_nolat']}")
        print(f"  wind_lat:   Kp={res['kp_wind_lat']:.1f} SR={res['sr_wind_lat']:.3f}  FR={res['fr_wind_lat']}")

    out_df = pd.DataFrame(results)
    out_df.to_csv(args.out, index=False)
    print(f"\nSaved {len(out_df)} rows to {args.out}")

    # Summary
    print("\n=== SUMMARY ===")
    print(f"{'Group':<25} {'FR nowind':>10} {'FR wind':>10} {'FR wind+lat':>12}")
    for dtype, grp in out_df.groupby("design_type"):
        print(f"{dtype:<25} {grp.fr_nowind.mean():10.3f} {grp.fr_wind_nolat.mean():10.3f} {grp.fr_wind_lat.mean():12.3f}")
    print()
    print("Mean kp_simple / Kp_ceiling (overtuning severity):")
    for dtype, grp in out_df.groupby("design_type"):
        print(f"  {dtype:<25} nowind={grp.ot_nowind.mean():.2f}x  wind={grp.ot_wind_nolat.mean():.2f}x  wind_lat={grp.ot_wind_lat.mean():.2f}x")


if __name__ == "__main__":
    main()
