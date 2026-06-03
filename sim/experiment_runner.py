"""
experiment_runner.py — Exp1 (regime mapping) and Exp4 (fidelity ladder) runners.

Interface contract
──────────────────
  All experiment code accesses physics ONLY through simulator.simulate().
  No experiment code modifies physics parameters directly.

Exp1: Design space regime mapping
──────────────────────────────────
  LHS sample of N designs.  For each design:
    1. Autotune: grid search over 4×4 (Kp, Kd) combinations, single-seed eval.
    2. Evaluate nominal gain set with single-seed (matches MATLAB fast-eval mode).
    3. Evaluate under-gain (0.60×) and over-gain (1.40×) sets.
    4. Robustness = fraction of {nominal, under, over} that pass FRAGILE threshold.
    5. Classify: EASY / FRAGILE / INFEASIBLE.

Exp4: Fidelity ladder
──────────────────────
  Tune ONCE at the reference fidelity level (L5_FULL).
  Evaluate at all 6 fidelity levels (L0 → L5) WITHOUT retuning.
  This answers: "When does a lower-fidelity simulation make the same GO/NOGO
  decision as the full-fidelity reference?"
  MATLAB re-tunes at every level — scientifically incorrect for this question.

Fidelity levels (L0 = simplest, L5 = most realistic)
──────────────────────────────────────────────────────
  L0  BASELINE            ideal actuator, no sensor noise, no gust
  L1  SATURATION          real u_max, still ideal slew
  L2  SLEW_LIMIT          adds servo slew rate
  L3  ACTUATOR_NONLINEAR  adds deadband and backlash
  L4  SENSOR_EFFECTS      adds gyro noise, bias, quantisation, latency
  L5  FULL                adds aero uncertainty (keff drift + gust)

Parallelism: joblib Parallel across designs (outer loop).
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from design_space import (
    sample_lhs, build_plant, build_actuator, build_sensor,
    build_disturbance, build_scenario, classify_regime, REF,
)
from controller import PIDParams
from sensor_model import SensorParams
from actuator_model import ActuatorParams
from disturbance_model import DisturbanceParams
from simulator import ScenarioParams, simulate, SimResult
from units import FRAGILE_SUCCESS_RATE

ROOT       = Path(__file__).resolve().parents[1]
RESULT_DIR = ROOT / "experiments" / "results"
RESULT_DIR.mkdir(parents=True, exist_ok=True)

# ── Tuning grid (matches MATLAB P.tuning block) ───────────────────────────────
KP_GRID     = [10.0, 20.0, 45.0, 80.0]
KD_GRID     = [4.0,  8.0, 16.0, 32.0]
UNDER_SCALE = 0.60
OVER_SCALE  = 1.40


# ── Fidelity level descriptors ────────────────────────────────────────────────

FIDELITY_LEVELS = [
    dict(name="L0_BASELINE",           added_physics="NONE",
         u_scale=4.0, slew_scale=4.0,  deadband=0.0, backlash=0.0,
         latency=1,   sensor_on=False,  aero_on=False),
    dict(name="L1_SATURATION",         added_physics="ACTUATOR_SATURATION",
         u_scale=1.0, slew_scale=4.0,  deadband=0.0, backlash=0.0,
         latency=1,   sensor_on=False,  aero_on=False),
    dict(name="L2_SLEW_LIMIT",         added_physics="SLEW_LIMIT",
         u_scale=1.0, slew_scale=1.0,  deadband=0.0, backlash=0.0,
         latency=1,   sensor_on=False,  aero_on=False),
    dict(name="L3_ACTUATOR_NONLINEAR", added_physics="DEADBAND_BACKLASH",
         u_scale=1.0, slew_scale=1.0,  deadband=None, backlash=None,
         latency=1,   sensor_on=False,  aero_on=False),
    dict(name="L4_SENSOR_EFFECTS",     added_physics="SENSOR_LATENCY_NOISE",
         u_scale=1.0, slew_scale=1.0,  deadband=None, backlash=None,
         latency=None, sensor_on=True,  aero_on=False),
    dict(name="L5_FULL",               added_physics="AERO_UNCERTAINTY_DRIFT",
         u_scale=1.0, slew_scale=1.0,  deadband=None, backlash=None,
         latency=None, sensor_on=True,  aero_on=True),
]


# ── Fidelity builder ──────────────────────────────────────────────────────────

def _apply_fidelity(
    base_act: ActuatorParams,
    base_sen: SensorParams,
    base_dis: DisturbanceParams,
    base_sc:  ScenarioParams,
    design:   dict,
    level:    dict,
) -> tuple[ActuatorParams, SensorParams, DisturbanceParams, ScenarioParams]:
    """Return (actuator, sensor, disturbance, scenario) adjusted for one fidelity level."""
    act = copy.copy(base_act)
    sen = copy.copy(base_sen)
    dis = copy.copy(base_dis)
    sc  = copy.copy(base_sc)

    act.u_max    = base_act.u_max    * level["u_scale"]
    act.slew_max = base_act.slew_max * level["slew_scale"]

    if level["deadband"] is not None:
        act.deadband = level["deadband"]
    if level["backlash"] is not None:
        act.backlash = level["backlash"]

    if level["latency"] is not None:
        sen.sensor_latency_steps = level["latency"]

    if level["sensor_on"]:
        sen.gyro_noise_std    = 0.015
        sen.gyro_bias_init    = 0.010
        sen.gyro_bias_rw      = 0.005
        sen.gyro_quant_lsb    = 2.0 * np.pi / 4000.0
        sen.bias_cal_residual = 0.10
    else:
        sen.gyro_noise_std    = 0.0
        sen.gyro_bias_init    = 0.0
        sen.gyro_bias_rw      = 0.0
        sen.gyro_quant_lsb    = 0.0
        sen.bias_cal_residual = 0.0

    if level["aero_on"]:
        sc.keff_fault_post  = 0.85
        sc.damp_fault_post  = 0.75
        # Fault activates at mid-burn (1.5 s) to model propellant mass loss
        sc.fault_time_s     = 1.5
        dis.gust_std        = max(dis.gust_std, 0.15)
    else:
        sc.keff_fault_post  = 1.0
        sc.damp_fault_post  = 1.0
        sc.fault_time_s     = float("inf")
        dis.gust_std        = 0.0

    return act, sen, dis, sc


# ── Single-run helper ─────────────────────────────────────────────────────────

def _run_one(
    pid:    PIDParams,
    plant,
    act:    ActuatorParams,
    sen:    SensorParams,
    dis:    DisturbanceParams,
    sc:     ScenarioParams,
    seed:   int = 1,
) -> dict:
    """Run one simulation and return a flat metrics dict."""
    r: SimResult = simulate(pid, plant, act, sen, dis, sc, seed=seed)
    return dict(
        success          = r.success,
        rms_error_deg    = r.rms_error_deg,
        peak_error_deg   = r.peak_error_deg,
        end_error_deg    = r.end_error_deg,
        max_theta_deg    = r.max_theta_deg,
        u_cmd_sat_frac   = r.u_cmd_sat_frac,
        slew_sat_frac    = r.slew_sat_frac,
        settling_time_s  = r.settling_time_s,
        oscillation_score= r.oscillation_score,
    )


def _aggregate(runs: list[dict]) -> dict:
    """Aggregate a list of run dicts into mean/max summary."""
    return dict(
        success_rate    = float(np.mean([r["success"] for r in runs])),
        rms_error_deg   = float(np.mean([r["rms_error_deg"] for r in runs])),
        peak_error_deg  = float(np.max([r["peak_error_deg"] for r in runs])),
        end_error_deg   = float(np.mean([r["end_error_deg"] for r in runs])),
        max_theta_deg   = float(np.max([r["max_theta_deg"] for r in runs])),
        u_cmd_sat_frac  = float(np.mean([r["u_cmd_sat_frac"] for r in runs])),
        slew_sat_frac   = float(np.mean([r["slew_sat_frac"] for r in runs])),
        settling_time_s = float(np.mean([r["settling_time_s"] for r in runs])),
        oscillation_score= float(np.mean([r["oscillation_score"] for r in runs])),
    )


# ── Autotuner ────────────────────────────────────────────────────────────────

def autotune_grid(
    plant,
    act:  ActuatorParams,
    sen:  SensorParams,
    dis:  DisturbanceParams,
    sc:   ScenarioParams,
    Kp_grid: list[float] = KP_GRID,
    Kd_grid: list[float] = KD_GRID,
) -> tuple[float, float]:
    """
    Grid search for (Kp, Kd) that maximises single-seed success.
    Returns (best_Kp, best_Kd).
    """
    best_Kp, best_Kd = Kp_grid[0], Kd_grid[0]
    best_sr = -1.0
    for Kp in Kp_grid:
        for Kd in Kd_grid:
            pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
            r = _run_one(pid, plant, act, sen, dis, sc, seed=1)
            sr = 1.0 if r["success"] else 0.0
            if sr > best_sr:
                best_sr = sr
                best_Kp, best_Kd = Kp, Kd
    return best_Kp, best_Kd


# ── Exp1: regime mapping ──────────────────────────────────────────────────────

def _eval_design_exp1(row: dict) -> dict:
    """Evaluate one design for Exp1: tune → nominal/under/over → classify."""
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    best_Kp, best_Kd = autotune_grid(plant, act, sen, dis, sc)

    def eval_gains(Kp, Kd):
        pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        return _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=1)])

    nominal = eval_gains(best_Kp, best_Kd)
    under   = eval_gains(UNDER_SCALE * best_Kp, UNDER_SCALE * best_Kd)
    over    = eval_gains(OVER_SCALE  * best_Kp, OVER_SCALE  * best_Kd)

    n_pass = (
        int(nominal["success_rate"] >= FRAGILE_SUCCESS_RATE)
        + int(under["success_rate"]  >= FRAGILE_SUCCESS_RATE)
        + int(over["success_rate"]   >= FRAGILE_SUCCESS_RATE)
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
        thrust             = row["thrust"],
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
    )


def run_exp1(
    n_designs: int = 1200,
    seed: int = 42,
    n_jobs: int = -1,
    out_dir: Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Experiment 1: map EASY/FRAGILE/INFEASIBLE over the 12-D LHS design space.
    Saves results to out_dir/exp1_regime_index_py.csv.
    """
    print(f"=== EXP1 (Python) n={n_designs} seed={seed} ===")
    designs = sample_lhs(n_designs, seed)
    rows = designs.to_dict("records")

    results = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp1)(row) for row in rows
    )

    df = pd.DataFrame(results)
    counts = df["regime_label"].value_counts()
    print(f"Regime counts:  EASY={counts.get('EASY',0)}  "
          f"FRAGILE={counts.get('FRAGILE',0)}  "
          f"INFEASIBLE={counts.get('INFEASIBLE',0)}")

    out = out_dir / "exp1_regime_index_py.csv"
    df.to_csv(out, index=False)
    print(f"Saved: {out}")
    return df


# ── Exp4: fidelity ladder ─────────────────────────────────────────────────────

def _verdict(success_rate: float) -> str:
    if success_rate >= 0.80:
        return "GO"
    elif success_rate >= 0.35:
        return "MARGINAL"
    return "NOGO"


def _eval_design_exp4(row: dict) -> list[dict]:
    """
    Evaluate one design across all fidelity levels.
    Tune ONCE at L5_FULL, then evaluate at L0–L5 without retuning.
    """
    plant     = build_plant(row)
    base_act  = build_actuator(row)
    base_sen  = build_sensor(row)
    base_dis  = build_disturbance(row)
    base_sc   = build_scenario(theta0_bias_std=0.0)

    # Tune at reference fidelity (L5_FULL)
    ref_level = FIDELITY_LEVELS[-1]
    act_ref, sen_ref, dis_ref, sc_ref = _apply_fidelity(
        base_act, base_sen, base_dis, base_sc, row, ref_level
    )
    best_Kp, best_Kd = autotune_grid(plant, act_ref, sen_ref, dis_ref, sc_ref)
    pid = PIDParams(Kp=best_Kp, Kd=best_Kd, Ki=0.0,
                    u_max=base_act.u_max, i_lim=base_act.u_max)

    # Evaluate at each fidelity level with the L5-tuned gains
    output_rows = []
    decisions   = []
    for level in FIDELITY_LEVELS:
        act_k, sen_k, dis_k, sc_k = _apply_fidelity(
            base_act, base_sen, base_dis, base_sc, row, level
        )
        pid_k = PIDParams(Kp=best_Kp, Kd=best_Kd, Ki=0.0,
                          u_max=act_k.u_max, i_lim=act_k.u_max)
        m = _aggregate([_run_one(pid_k, plant, act_k, sen_k, dis_k, sc_k, seed=1)])
        decisions.append(_verdict(m["success_rate"]))

        output_rows.append(dict(
            rocket_id        = row["rocket_id"],
            regime_label     = row.get("regime_label", "UNKNOWN"),
            fidelity_level   = level["name"],
            added_physics    = level["added_physics"],
            decision         = decisions[-1],
            success_rate     = m["success_rate"],
            rms_error_deg    = m["rms_error_deg"],
            u_cmd_sat_frac   = m["u_cmd_sat_frac"],
            slew_sat_frac    = m["slew_sat_frac"],
            best_Kp          = best_Kp,
            best_Kd          = best_Kd,
            # Pass-through design params for later SHAP analysis
            p_unstable       = row.get("p_unstable"),
            mass             = row.get("mass"),
            Iyy              = row.get("Iyy"),
            static_margin    = row.get("static_margin"),
            Cm_alpha         = row.get("Cm_alpha"),
            thrust           = row.get("thrust"),
            servo_slew_deg_s = row.get("servo_slew_deg_s"),
            max_gimbal_deg   = row.get("max_gimbal_deg"),
            latency_steps    = row.get("latency_steps"),
            deadband         = row.get("deadband"),
            backlash         = row.get("backlash"),
            wind_strength    = row.get("wind_strength"),
        ))

    ref_decision = decisions[-1]
    for i, r in enumerate(output_rows):
        r["reference_decision"] = ref_decision
        r["disagrees_with_ref"] = int(decisions[i] != ref_decision)
        first_match = next(
            (j for j, d in enumerate(decisions) if d == ref_decision),
            len(FIDELITY_LEVELS) - 1
        )
        r["first_correct_fidelity_idx"]  = first_match
        r["first_correct_fidelity_name"] = FIDELITY_LEVELS[first_match]["name"]
        r["dominant_missing_physics"]    = (
            FIDELITY_LEVELS[first_match]["added_physics"]
            if first_match > 0 else "NONE"
        )

    return output_rows


def run_exp4(
    designs_df: Optional[pd.DataFrame] = None,
    n_jobs: int = -1,
    out_dir: Path = RESULT_DIR,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Experiment 4: fidelity ladder study.
    Tune once at L5; evaluate at L0–L5 without retuning.
    Saves:
      exp4_fidelity_decision_trajectories_py.csv  (one row per design × fidelity level)
      exp4_first_correct_fidelity_py.csv          (one row per design, summary)
    """
    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        ml_path = out_dir / "exp1_regime_index.csv"
        path = py_path if py_path.exists() else ml_path
        if not path.exists():
            raise FileNotFoundError(
                "No Exp1 results found.  Run run_exp1() first or pass designs_df."
            )
        designs_df = pd.read_csv(path)

    print(f"=== EXP4 (Python) n={len(designs_df)} designs, tune-once-at-L5 ===")
    rows = designs_df.to_dict("records")

    all_rows = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp4)(row) for row in rows
    )

    traj_rows = [r for per_design in all_rows for r in per_design]
    traj_df   = pd.DataFrame(traj_rows)

    # Summary: one row per design
    summary_rows = []
    for per_design in all_rows:
        r0 = per_design[0]
        summary_rows.append(dict(
            rocket_id                  = r0["rocket_id"],
            regime_label               = r0["regime_label"],
            reference_decision         = r0["reference_decision"],
            first_correct_fidelity_idx = r0["first_correct_fidelity_idx"],
            first_correct_fidelity_name= r0["first_correct_fidelity_name"],
            dominant_missing_physics   = r0["dominant_missing_physics"],
        ))
    summary_df = pd.DataFrame(summary_rows)

    # Print disagreement stats
    for regime in ["EASY", "FRAGILE", "INFEASIBLE"]:
        sub = traj_df[traj_df["regime_label"] == regime]
        if len(sub) == 0:
            continue
        for lvl in ["L0_BASELINE", "L2_SLEW_LIMIT", "L4_SENSOR_EFFECTS"]:
            g = sub[sub["fidelity_level"] == lvl]
            if len(g) > 0:
                dis = g["disagrees_with_ref"].mean()
                print(f"  [{regime}] {lvl} disagrees with L5: {dis:.3f}")

    out_traj = out_dir / "exp4_fidelity_decision_trajectories_py.csv"
    out_sum  = out_dir / "exp4_first_correct_fidelity_py.csv"
    traj_df.to_csv(out_traj, index=False)
    summary_df.to_csv(out_sum, index=False)
    print(f"Saved: {out_traj}")
    print(f"Saved: {out_sum}")
    return traj_df, summary_df


# ── Exp4 ablation: fidelity necessity study ───────────────────────────────────
#
# For each design, run FULL fidelity (all 7 modules on, including thrust_var fault)
# then ablate one module at a time.  Gains frozen at Exp1-tuned values.
#
# Scientific question: which physics module, if omitted from the simulator, most
# changes the engineering decision for each rocket design?
#
# Output schema (one row per design):
#   design params ... | baseline metrics | delta_<metric>_<module> ... | dominant_fidelity
#
# delta sign convention:
#   delta_rms_wind = rms(ablated_wind) - rms(full)
#   Negative = removing wind REDUCES rms (wind was hurting performance)
#   Positive = removing wind WORSENS rms (wind was helping dampen oscillations?)
#   Large |delta| = this module matters for this design.

from fidelity_config import FidelityConfig, apply_fidelity_config, ABLATION_MODULES

# Number of seeds to average over for Exp4 delta estimation.
# More seeds = less RNG noise in delta metrics.  3 is sufficient for identification.
EXP4_N_SEEDS = 3

# Number of seeds for Exp5 gradient estimation (central finite differences).
# Single seed produces noisy gradients.  3 seeds reduces stochastic variance ~1.7x.
# 5 seeds preferred for publication figures; 3 is a good compromise for speed.
EXP5_N_SEEDS_GRAD = 3


def _eval_one_fidelity(
    plant,
    base_act: ActuatorParams,
    base_sen: SensorParams,
    base_dis: DisturbanceParams,
    base_sc:  ScenarioParams,
    pid:      PIDParams,
    cfg:      FidelityConfig,
    seeds:    tuple[int, ...] = (1, 2, 3),
) -> dict:
    """
    Evaluate one (design, fidelity config) pair over multiple seeds.
    Returns averaged scalar metrics dict.
    """
    runs = []
    for s in seeds:
        act, sen, dis, sc = apply_fidelity_config(base_act, base_sen, base_dis, base_sc, cfg)
        pid_k = PIDParams(Kp=pid.Kp, Kd=pid.Kd, Ki=0.0,
                          u_max=base_act.u_max, i_lim=base_act.u_max)
        r: SimResult = simulate(pid_k, plant, act, sen, dis, sc, seed=s)
        runs.append(dict(
            success           = int(r.success),
            rms_error_deg     = r.rms_error_deg,
            end_error_deg     = r.end_error_deg,
            max_theta_deg     = r.max_theta_deg,
            peak_error_deg    = r.peak_error_deg,
            u_cmd_sat_frac    = r.u_cmd_sat_frac,
            slew_sat_frac     = r.slew_sat_frac,
            settling_time_s   = r.settling_time_s,
            oscillation_score = r.oscillation_score,
        ))
    return _aggregate(runs)


def _eval_design_exp4_ablation(row: dict) -> dict:
    """
    Evaluate one design under full fidelity and each single-module ablation.
    Gains frozen at Exp1-tuned best_Kp / best_Kd.
    Returns a flat dict with baseline metrics + per-module deltas.
    """
    if "best_Kp" not in row or "best_Kd" not in row:
        raise ValueError(
            f"Design {row.get('rocket_id','?')} missing best_Kp/best_Kd. "
            "Run Exp1 first."
        )
    Kp = float(row["best_Kp"])
    Kd = float(row["best_Kd"])

    plant    = build_plant(row)
    base_act = build_actuator(row)
    base_sen = build_sensor(row)
    base_dis = build_disturbance(row)
    base_sc  = build_scenario(theta0_bias_std=0.0)

    # Activate thrust_var fault for full fidelity
    base_sc.keff_fault_post = 0.85
    base_sc.damp_fault_post = 0.75
    base_sc.fault_time_s    = 1.5

    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=base_act.u_max, i_lim=base_act.u_max)
    seeds = tuple(range(1, EXP4_N_SEEDS + 1))

    # ── Full fidelity baseline ────────────────────────────────────────────
    full_cfg  = FidelityConfig.full()
    full_m    = _eval_one_fidelity(plant, base_act, base_sen, base_dis, base_sc, pid, full_cfg, seeds)

    out = dict(
        rocket_id    = row["rocket_id"],
        design_id    = row["design_id"],
        regime_label = row.get("regime_label", "UNKNOWN"),
        best_Kp      = Kp,
        best_Kd      = Kd,
    )
    # Pass through design params
    for k in ["p_unstable", "mass", "Iyy", "static_margin", "Cm_alpha",
              "control_effectiveness", "thrust", "servo_slew_deg_s",
              "max_gimbal_deg", "latency_steps", "deadband", "backlash", "wind_strength"]:
        out[k] = row.get(k)

    # Baseline metrics
    out["full_success_rate"] = full_m["success_rate"]
    out["full_rms"]          = full_m["rms_error_deg"]
    out["full_end"]          = full_m["end_error_deg"]
    out["full_max_theta"]    = full_m["max_theta_deg"]
    out["full_peak"]         = full_m["peak_error_deg"]
    out["full_u_sat"]        = full_m["u_cmd_sat_frac"]
    out["full_slew_sat"]     = full_m["slew_sat_frac"]

    # ── Single-module ablations ───────────────────────────────────────────
    delta_rms_by_module: dict[str, float] = {}
    for module in ABLATION_MODULES:
        abl_cfg = FidelityConfig.full().ablate(module)
        abl_m   = _eval_one_fidelity(plant, base_act, base_sen, base_dis, base_sc, pid, abl_cfg, seeds)

        d_rms     = abl_m["rms_error_deg"]   - full_m["rms_error_deg"]
        d_end     = abl_m["end_error_deg"]   - full_m["end_error_deg"]
        d_theta   = abl_m["max_theta_deg"]   - full_m["max_theta_deg"]
        d_success = abl_m["success_rate"]    - full_m["success_rate"]
        # Engineering decision change: does removing this module flip the GO/NOGO?
        full_go   = full_m["success_rate"] >= FRAGILE_SUCCESS_RATE
        abl_go    = abl_m["success_rate"]  >= FRAGILE_SUCCESS_RATE
        decision_flip = int(full_go != abl_go)

        out[f"delta_rms_{module}"]     = d_rms
        out[f"delta_end_{module}"]     = d_end
        out[f"delta_theta_{module}"]   = d_theta
        out[f"delta_success_{module}"] = d_success
        out[f"decision_flip_{module}"] = decision_flip

        delta_rms_by_module[module]    = abs(d_rms)

    # ── Summary: dominant fidelity term ──────────────────────────────────
    out["dominant_fidelity"]    = max(delta_rms_by_module, key=delta_rms_by_module.get)
    out["dominant_delta_rms"]   = delta_rms_by_module[out["dominant_fidelity"]]
    out["n_decision_flips"]     = sum(
        out[f"decision_flip_{m}"] for m in ABLATION_MODULES
    )
    # Fidelity modules that flip the engineering decision (space-separated list)
    out["decision_flip_modules"] = " ".join(
        m for m in ABLATION_MODULES if out[f"decision_flip_{m}"] == 1
    ) or "none"

    return out


def run_exp4_ablation(
    designs_df: Optional[pd.DataFrame] = None,
    n_jobs:     int = -1,
    out_dir:    Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Experiment 4 (ablation): fidelity necessity study.

    For each design: run full fidelity (7 modules on), then ablate each module
    individually.  Compute delta metrics (full minus ablated).

    Scientific question: Which simulator fidelity component is necessary
    for making the correct engineering decision for THIS design?

    Gains frozen at Exp1 best_Kp / best_Kd.

    Saves:
      exp4_ablation_study_py.csv   (one row per design)
    """
    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        if not py_path.exists():
            raise FileNotFoundError(
                "Exp1 results not found.  Run run_exp1() first or pass designs_df."
            )
        designs_df = pd.read_csv(py_path)
        if "best_Kp" not in designs_df.columns:
            raise ValueError(
                "Exp1 CSV missing best_Kp/best_Kd.  Re-run Exp1 with current code."
            )

    print(f"=== EXP4 ABLATION (Python) n={len(designs_df)} designs ===")
    print(f"    Modules: {ABLATION_MODULES}")
    print(f"    Seeds per condition: {EXP4_N_SEEDS}")
    rows = designs_df.to_dict("records")

    results = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp4_ablation)(row) for row in rows
    )

    df = pd.DataFrame(results)

    # Print dominant fidelity distribution
    dom = df["dominant_fidelity"].value_counts()
    print("\n  Dominant fidelity by design:")
    for module, count in dom.items():
        pct = 100 * count / len(df)
        print(f"    {module:<20s} {count:4d}  ({pct:.1f}%)")

    print(f"\n  Designs with >=1 decision flip: "
          f"{(df['n_decision_flips'] > 0).sum()} / {len(df)}")

    out = out_dir / "exp4_ablation_study_py.csv"
    df.to_csv(out, index=False)
    print(f"Saved: {out}")
    return df


# ── Exp4 simple-vs-full paired comparison ────────────────────────────────────
#
# For each design: run SIMPLE fidelity (all modules off, no thrust_var fault)
# and compare decision to Exp1 ground truth (also simple, no fault).
#
# Scientific question: at what rate does simple-model GO/NOGO agree with
# full-fidelity GO/NOGO?  This is the direct measure of simple-model risk.
#
# Output schema (one row per design):
#   design params ... | simple_go | full_go | decision_agrees | regime_label


def _eval_design_exp4simple(row: dict) -> dict:
    """
    Evaluate one design under SIMPLE fidelity and FULL fidelity (with thrust_var fault).
    Gains frozen at Exp1 best_Kp / best_Kd.
    Returns a flat dict with both decisions and whether they agree.
    """
    if "best_Kp" not in row or "best_Kd" not in row:
        raise ValueError(
            f"Design {row.get('rocket_id','?')} missing best_Kp/best_Kd. "
            "Run Exp1 first."
        )
    Kp = float(row["best_Kp"])
    Kd = float(row["best_Kd"])

    plant    = build_plant(row)
    base_act = build_actuator(row)
    base_sen = build_sensor(row)
    base_dis = build_disturbance(row)

    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=base_act.u_max, i_lim=base_act.u_max)
    seeds = tuple(range(1, EXP4_N_SEEDS + 1))

    # Simple fidelity — no fault, all modules off (matches Exp1 evaluation conditions)
    simple_sc  = build_scenario(theta0_bias_std=0.0)
    simple_cfg = FidelityConfig.simple()
    simple_m   = _eval_one_fidelity(plant, base_act, base_sen, base_dis, simple_sc, pid, simple_cfg, seeds)

    # Full fidelity — with thrust_var fault (Exp4 baseline)
    full_sc  = build_scenario(theta0_bias_std=0.0)
    full_sc.keff_fault_post = 0.85
    full_sc.damp_fault_post = 0.75
    full_sc.fault_time_s    = 1.5
    full_cfg = FidelityConfig.full()
    full_m   = _eval_one_fidelity(plant, base_act, base_sen, base_dis, full_sc, pid, full_cfg, seeds)

    simple_go = int(simple_m["success_rate"] >= FRAGILE_SUCCESS_RATE)
    full_go   = int(full_m["success_rate"]   >= FRAGILE_SUCCESS_RATE)
    # Exp1 uses nominal_success_rate (tuned at full, evaluated at nominal wind)
    exp1_sr   = row.get("nominal_success_rate", row.get("success_rate", 0))
    exp1_go   = int(float(exp1_sr) >= FRAGILE_SUCCESS_RATE)

    out = dict(
        rocket_id          = row["rocket_id"],
        design_id          = row["design_id"],
        regime_label       = row.get("regime_label", "UNKNOWN"),
        best_Kp            = Kp,
        best_Kd            = Kd,
        simple_success_rate = simple_m["success_rate"],
        simple_rms         = simple_m["rms_error_deg"],
        simple_go          = simple_go,
        full_success_rate  = full_m["success_rate"],
        full_rms           = full_m["rms_error_deg"],
        full_go            = full_go,
        exp1_go            = exp1_go,
        simple_vs_full_agrees = int(simple_go == full_go),
        simple_vs_exp1_agrees = int(simple_go == exp1_go),
        full_vs_exp1_agrees   = int(full_go   == exp1_go),
    )

    for k in ["p_unstable", "mass", "Iyy", "static_margin", "Cm_alpha",
              "control_effectiveness", "thrust", "servo_slew_deg_s",
              "max_gimbal_deg", "latency_steps", "deadband", "backlash", "wind_strength"]:
        out[k] = row.get(k)

    return out


def run_exp4simple(
    designs_df: Optional[pd.DataFrame] = None,
    n_jobs:     int = -1,
    out_dir:    Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Experiment 4-Simple: paired simple-model vs full-fidelity decision comparison.

    Runs FidelityConfig.simple() and FidelityConfig.full() for all 1200 designs
    with gains frozen at Exp1 values.  Produces the direct answer to:
      "If an engineer uses a simple simulator to decide whether to build this rocket,
       how often will they make the wrong call?"

    Saves:
      exp4_simple_vs_full_py.csv   (one row per design)
    """
    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        if not py_path.exists():
            raise FileNotFoundError(
                "Exp1 results not found.  Run run_exp1() first or pass designs_df."
            )
        designs_df = pd.read_csv(py_path)
        if "best_Kp" not in designs_df.columns:
            raise ValueError(
                "Exp1 CSV missing best_Kp/best_Kd.  Re-run Exp1 with current code."
            )

    print(f"=== EXP4-SIMPLE (Python) n={len(designs_df)} designs ===")
    print(f"    Seeds per condition: {EXP4_N_SEEDS}")
    print(f"    Simple: FidelityConfig.simple() — no fault")
    print(f"    Full:   FidelityConfig.full()   — with thrust_var fault at t=1.5s")
    rows = designs_df.to_dict("records")

    results = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp4simple)(row) for row in rows
    )

    df = pd.DataFrame(results)

    print("\n  Decision agreement by regime:")
    for regime in ["EASY", "FRAGILE", "INFEASIBLE"]:
        sub = df[df["regime_label"] == regime]
        if len(sub) == 0:
            continue
        s_vs_f = sub["simple_vs_full_agrees"].mean()
        s_vs_e = sub["simple_vs_exp1_agrees"].mean()
        f_vs_e = sub["full_vs_exp1_agrees"].mean()
        print(f"  [{regime:10s}] simple==full: {s_vs_f:.3f}  "
              f"simple==exp1: {s_vs_e:.3f}  full==exp1: {f_vs_e:.3f}")

    out = out_dir / "exp4_simple_vs_full_py.csv"
    df.to_csv(out, index=False)
    print(f"Saved: {out}")
    return df


# ── Exp5: performance landscape / evolution atlas ─────────────────────────────
#
# For each design: compute gradient and curvature of performance metrics w.r.t.
# each design parameter (gains frozen at Exp1 values).
#
# Scientific question: In which direction should a designer move this rocket's
# design to most improve performance?  Which dimensions are already saturated?
#
# Output files:
#   exp5_gradient_field_py.csv    one row per design: gradient + curvature + topology
#   exp5_evolution_paths_py.csv   one row per (design, step): path through design space
#   exp5_diminishing_returns_py.csv one row per (param, sweep_point): 1D sweeps

from local_analysis import (
    compute_gradient, compute_curvature, classify_topology,
    best_improvement_direction, compute_evolution_path,
    GRAD_PARAMS_ALL, GRAD_PARAMS_CONTINUOUS,
)


def _eval_design_exp5(row: dict) -> dict:
    """
    Compute gradient field, curvature, and topology for one design.
    Gains frozen at Exp1 best_Kp / best_Kd.
    Primary metric: rms_error_deg.
    """
    if "best_Kp" not in row or "best_Kd" not in row:
        raise ValueError(f"Design {row.get('rocket_id','?')} missing best_Kp/best_Kd.")

    Kp  = float(row["best_Kp"])
    Kd  = float(row["best_Kd"])
    cfg = FidelityConfig.full()

    grad = compute_gradient(row, Kp, Kd, metric="rms_error_deg", cfg=cfg,
                            seeds=tuple(range(1, EXP5_N_SEEDS_GRAD + 1)))
    curv = compute_curvature(row, Kp, Kd, metric="rms_error_deg", cfg=cfg, seed=1)

    best_param, best_dir, best_mag = best_improvement_direction(grad, metric_is_cost=True)

    # Gradient magnitude: range-scaled so all parameters are dimensionless
    from local_analysis import range_scaled_gradient
    scaled     = range_scaled_gradient({p: grad.get(p, 0.0) for p in GRAD_PARAMS_CONTINUOUS})
    g_vec      = np.array([scaled[p] for p in GRAD_PARAMS_CONTINUOUS])
    grad_magnitude = float(np.linalg.norm(g_vec))

    topology = classify_topology(grad, curv)

    out = dict(
        rocket_id       = row["rocket_id"],
        design_id       = row["design_id"],
        regime_label    = row.get("regime_label", "UNKNOWN"),
        best_Kp         = Kp,
        best_Kd         = Kd,
        grad_magnitude  = grad_magnitude,
        topology_class  = topology,
        best_param      = best_param,
        best_direction  = best_dir,
        best_magnitude  = best_mag,
    )
    for k in ["p_unstable", "mass", "Iyy", "static_margin", "Cm_alpha",
              "control_effectiveness", "thrust", "servo_slew_deg_s",
              "max_gimbal_deg", "latency_steps", "deadband", "backlash", "wind_strength"]:
        out[k] = row.get(k)

    for p in GRAD_PARAMS_ALL:
        out[f"grad_rms_{p}"] = grad.get(p, 0.0)
    for p in GRAD_PARAMS_CONTINUOUS:
        out[f"curv_rms_{p}"] = curv.get(p, 0.0)

    return out


def _eval_design_exp5_path(row: dict, n_steps: int = 5) -> list[dict]:
    """Compute evolution path for one design; annotate each step with design_id."""
    Kp  = float(row["best_Kp"])
    Kd  = float(row["best_Kd"])
    cfg = FidelityConfig.full()

    path = compute_evolution_path(row, Kp, Kd, metric="rms_error_deg",
                                   cfg=cfg, n_steps=n_steps, step_fraction=0.10)
    for step in path:
        step["rocket_id"]  = row["rocket_id"]
        step["design_id"]  = row["design_id"]
        step["regime_label"] = row.get("regime_label", "UNKNOWN")
    return path


def run_exp5_landscape(
    designs_df:    Optional[pd.DataFrame] = None,
    n_jobs:        int = -1,
    out_dir:       Path = RESULT_DIR,
    compute_paths: bool = True,
    n_path_steps:  int = 5,
    compute_diminishing: bool = True,
) -> pd.DataFrame:
    """
    Experiment 5: performance landscape / evolution atlas.

    For each design in Exp1:
      - Compute gradient of rms_error_deg w.r.t. all design parameters
      - Compute diagonal curvature (d2 rms / d param^2)
      - Classify topology (plateau / ridge / cliff / bowl)
      - Identify best improvement direction
      - Optionally compute gradient-descent evolution path

    Saves:
      exp5_gradient_field_py.csv       per-design gradient + topology
      exp5_evolution_paths_py.csv      per-design evolution path (if compute_paths)
      exp5_diminishing_returns_py.csv  1D parameter sweeps (if compute_diminishing)
    """
    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        if not py_path.exists():
            raise FileNotFoundError("Exp1 results not found.")
        designs_df = pd.read_csv(py_path)

    print(f"=== EXP5 LANDSCAPE (Python) n={len(designs_df)} designs ===")
    rows = designs_df.to_dict("records")

    # ── Gradient field ───────────────────────────────────────────────────
    print("  Computing gradient field (central differences, full fidelity)...")
    grad_results = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp5)(row) for row in rows
    )
    grad_df = pd.DataFrame(grad_results)

    # Topology distribution
    top_counts = grad_df["topology_class"].value_counts()
    print("\n  Topology distribution:")
    for topo, cnt in top_counts.items():
        print(f"    {topo:<12s} {cnt:4d}  ({100*cnt/len(grad_df):.1f}%)")

    print("\n  Most common improvement direction:")
    bp_counts = grad_df["best_param"].value_counts().head(5)
    for param, cnt in bp_counts.items():
        print(f"    {param:<28s} {cnt:4d}  ({100*cnt/len(grad_df):.1f}%)")

    out_grad = out_dir / "exp5_gradient_field_py.csv"
    grad_df.to_csv(out_grad, index=False)
    print(f"Saved: {out_grad}")

    # ── Retrofit population-level plateau threshold ──────────────────────
    # Reclassify topology using population gradient magnitude percentile.
    g_mags = grad_df["grad_magnitude"].values
    plateau_thresh = float(np.percentile(g_mags, 10))
    print(f"\n  Population plateau threshold (10th pct): {plateau_thresh:.4f}")

    def _reclassify(r_grad, r_curv_cols, threshold):
        g = {p: r_grad.get(f"grad_rms_{p}", 0.0) for p in GRAD_PARAMS_CONTINUOUS}
        c = {p: r_grad.get(f"curv_rms_{p}", 0.0) for p in GRAD_PARAMS_CONTINUOUS}
        return classify_topology(g, c, plateau_pct=threshold, _population_grad_mag=None)

    grad_df["topology_class_pop"] = [
        classify_topology(
            {p: row.get(f"grad_rms_{p}", 0.0) for p in GRAD_PARAMS_CONTINUOUS},
            {p: row.get(f"curv_rms_{p}", 0.0) for p in GRAD_PARAMS_CONTINUOUS},
            plateau_pct=plateau_thresh,
        )
        for row in grad_df.to_dict("records")
    ]
    grad_df.to_csv(out_grad, index=False)

    # ── Evolution paths ──────────────────────────────────────────────────
    if compute_paths:
        print(f"\n  Computing evolution paths ({n_path_steps} steps per design)...")
        path_results = Parallel(n_jobs=n_jobs, verbose=5)(
            delayed(_eval_design_exp5_path)(row, n_path_steps) for row in rows
        )
        path_rows = [step for path in path_results for step in path]
        path_df   = pd.DataFrame(path_rows)
        out_path  = out_dir / "exp5_evolution_paths_py.csv"
        path_df.to_csv(out_path, index=False)
        print(f"Saved: {out_path}  ({len(path_df)} rows)")

    # ── Diminishing returns: 1D parameter sweeps ─────────────────────────
    if compute_diminishing:
        print("\n  Computing diminishing returns (population sweeps, top 5 parameters)...")
        # Rank parameters by RANGE-SCALED gradient (removes unit bias: a raw
        # gradient ranking is dominated by small-unit params like Iyy).
        from local_analysis import range_scaled_gradient
        mean_scaled_grad = {}
        for p in GRAD_PARAMS_CONTINUOUS:
            col = f"grad_rms_{p}"
            if col not in grad_df.columns:
                continue
            scaled = grad_df[col].apply(
                lambda g, pp=p: range_scaled_gradient({pp: g})[pp]
            )
            mean_scaled_grad[p] = float(scaled.abs().mean())
        top_params = sorted(mean_scaled_grad, key=mean_scaled_grad.get, reverse=True)[:5]
        print(f"    Top 5 by mean |range-scaled grad|: {top_params}")

        dr_df  = _compute_diminishing_returns_population(
            designs_df = designs_df,
            params     = top_params,
            n_points   = 40,
            n_designs_sample = 100,
            n_jobs     = n_jobs,
        )
        out_dr = out_dir / "exp5_diminishing_returns_py.csv"
        dr_df.to_csv(out_dr, index=False)
        print(f"Saved: {out_dr}  ({len(dr_df)} rows)")

    return grad_df


def _sweep_one_design(row: dict, param: str, sweep_vals: np.ndarray) -> list[dict]:
    """Sweep one parameter for one design at its own tuned gains; return per-point rms."""
    from local_analysis import _evaluate_design
    from design_space import estimate_p_unstable as _est_p
    Kp  = float(row["best_Kp"])
    Kd  = float(row["best_Kd"])
    cfg = FidelityConfig.full()
    out = []
    for val in sweep_vals:
        d = dict(row)
        d[param] = float(val)
        if param in ("static_margin", "Cm_alpha", "thrust"):
            d["p_unstable"] = _est_p(
                d.get("static_margin", REF["static_margin"]),
                d.get("Cm_alpha",      REF["Cm_alpha"]),
                d.get("thrust",        REF["thrust"]),
            )
        try:
            r = _evaluate_design(d, Kp, Kd, cfg, seed=1)
            out.append((float(val), r.rms_error_deg, int(r.success)))
        except Exception:
            out.append((float(val), float("nan"), 0))
    return out


def _compute_diminishing_returns_population(
    designs_df: pd.DataFrame,
    params: list[str],
    n_points: int = 40,
    n_designs_sample: int = 100,
    n_jobs: int = -1,
) -> pd.DataFrame:
    """
    Population-level diminishing returns.

    For each parameter and each sweep value, override that parameter across a
    sample of designs (each evaluated at its OWN tuned gains), then report the
    median and inter-quartile band of RMS error.

    This is more representative and far smoother than sweeping a single anchor
    rocket: the population band shows how the upgrade pays off across the design
    distribution, and the median curve reveals the diminishing-return knee.

    Returns a tidy DataFrame:
      param, param_val, param_normalised, rms_median, rms_q25, rms_q75, success_rate
    """
    from design_space import DESIGN_LO, DESIGN_HI, DESIGN_NAMES
    lo_map = dict(zip(DESIGN_NAMES, DESIGN_LO))
    hi_map = dict(zip(DESIGN_NAMES, DESIGN_HI))

    # Representative sample: prefer EASY+FRAGILE (where improvement is actionable);
    # fall back to all designs if too few.
    actionable = designs_df[designs_df["regime_label"].isin(["EASY", "FRAGILE"])]
    if len(actionable) < n_designs_sample:
        actionable = designs_df
    sample = actionable.sample(
        min(n_designs_sample, len(actionable)), random_state=42
    ).to_dict("records")

    rows = []
    for param in params:
        sweep_vals = np.linspace(lo_map[param], hi_map[param], n_points)
        # Parallel across designs; each returns a list of (val, rms, success)
        per_design = Parallel(n_jobs=n_jobs)(
            delayed(_sweep_one_design)(row, param, sweep_vals) for row in sample
        )
        # Aggregate per sweep value
        param_range = hi_map[param] - lo_map[param]
        for i, val in enumerate(sweep_vals):
            rms_vals  = np.array([pd_[i][1] for pd_ in per_design], dtype=float)
            succ_vals = np.array([pd_[i][2] for pd_ in per_design], dtype=float)
            rms_vals  = rms_vals[np.isfinite(rms_vals)]
            if len(rms_vals) == 0:
                continue
            rows.append(dict(
                param            = param,
                param_val        = float(val),
                param_normalised = (val - lo_map[param]) / param_range if param_range > 0 else 0.0,
                rms_median       = float(np.median(rms_vals)),
                rms_q25          = float(np.percentile(rms_vals, 25)),
                rms_q75          = float(np.percentile(rms_vals, 75)),
                success_rate     = float(np.mean(succ_vals)),
            ))
    return pd.DataFrame(rows)


def _compute_diminishing_returns(
    ref_design: dict,
    ref_Kp: float,
    ref_Kd: float,
    params: list[str],
    n_points: int = 40,
) -> list[dict]:
    """
    [Legacy] Single-anchor sweep.  Retained for reference; the pipeline now uses
    _compute_diminishing_returns_population() for smoother, representative curves.
    """
    from design_space import DESIGN_LO, DESIGN_HI, DESIGN_NAMES, estimate_p_unstable
    lo_map = dict(zip(DESIGN_NAMES, DESIGN_LO))
    hi_map = dict(zip(DESIGN_NAMES, DESIGN_HI))

    cfg = FidelityConfig.full()
    rows = []
    for param in params:
        sweep_vals = np.linspace(lo_map[param], hi_map[param], n_points)
        for val in sweep_vals:
            d = dict(ref_design)
            d[param] = float(val)
            # Recompute p_unstable if needed
            if param in ("static_margin", "Cm_alpha", "thrust"):
                d["p_unstable"] = estimate_p_unstable(
                    d.get("static_margin", REF["static_margin"]),
                    d.get("Cm_alpha",      REF["Cm_alpha"]),
                    d.get("thrust",        REF["thrust"]),
                )
            try:
                from local_analysis import _evaluate_design
                r = _evaluate_design(d, ref_Kp, ref_Kd, cfg, seed=1)
                rms  = r.rms_error_deg
                succ = int(r.success)
            except Exception:
                rms  = float("nan")
                succ = 0

            param_range = hi_map[param] - lo_map[param]
            rows.append(dict(
                param          = param,
                param_val      = float(val),
                param_normalised = (val - lo_map[param]) / param_range if param_range > 0 else 0.0,
                rms_error_deg  = rms,
                success        = succ,
            ))
    return rows


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    cmd = sys.argv[1] if len(sys.argv) > 1 else "exp1"
    n   = int(sys.argv[2]) if len(sys.argv) > 2 else 1200

    if cmd == "exp1":
        run_exp1(n_designs=n)
    elif cmd == "exp4":
        run_exp4()
    elif cmd == "exp4abl":
        run_exp4_ablation()
    elif cmd == "exp4simple":
        run_exp4simple()
    elif cmd == "exp5":
        run_exp5_landscape()
    elif cmd == "both":
        df = run_exp1(n_designs=n)
        run_exp4(df)
    elif cmd == "all":
        df = run_exp1(n_designs=n)
        run_exp4(df)
        run_exp4_ablation(df)
        run_exp5_landscape(df)
    else:
        print("Usage: python experiment_runner.py [exp1|exp4|exp4abl|exp4simple|exp5|both|all] [n_designs]")
