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
from fidelity_config import FidelityConfig, apply_fidelity_config, ABLATION_MODULES
from simulator import ScenarioParams, simulate, SimResult
from units import FRAGILE_SUCCESS_RATE

ROOT       = Path(__file__).resolve().parents[1]
RESULT_DIR = ROOT / "experiments" / "results"
RESULT_DIR.mkdir(parents=True, exist_ok=True)

# ── Tuning grid ───────────────────────────────────────────────────────────────
# Extended range to cover sub-Kp=5 optima: diagnostics showed 87% of designs
# chose the previous floor (Kp=5), and ~54% of INFEASIBLE designs had their
# under-gains (Kp=3, Kd=1.2) outperform the nominal (Kp=5, Kd=2), proving
# the true optimum was below the grid minimum.
KP_GRID     = [2.0, 5.0, 15.0, 40.0, 80.0]
KD_GRID     = [1.0, 2.0,  8.0, 16.0, 32.0]
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
        diverged         = r.diverged,
        band_30_pass     = r.band_30_pass,
        band_20_pass     = r.band_20_pass,
        band_10_pass     = r.band_10_pass,
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
        success_rate     = float(np.mean([r["success"]      for r in runs])),
        diverge_rate     = float(np.mean([r["diverged"]     for r in runs])),
        band_30_rate     = float(np.mean([r["band_30_pass"] for r in runs])),
        band_20_rate     = float(np.mean([r["band_20_pass"] for r in runs])),
        band_10_rate     = float(np.mean([r["band_10_pass"] for r in runs])),
        rms_error_deg    = float(np.mean([r["rms_error_deg"]    for r in runs])),
        peak_error_deg   = float(np.max( [r["peak_error_deg"]   for r in runs])),
        end_error_deg    = float(np.mean([r["end_error_deg"]    for r in runs])),
        max_theta_deg    = float(np.max( [r["max_theta_deg"]    for r in runs])),
        u_cmd_sat_frac   = float(np.mean([r["u_cmd_sat_frac"]   for r in runs])),
        slew_sat_frac    = float(np.mean([r["slew_sat_frac"]    for r in runs])),
        settling_time_s  = float(np.mean([r["settling_time_s"]  for r in runs])),
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
    Grid search for (Kp, Kd) that best stabilises the design.

    Selection rule (2-seed average for tuning stability):
      1. Primary criterion: highest mean success_rate across seeds (1,2).
      2. Tiebreaker among equal success rates: lowest mean RMS error.
      3. If nothing succeeds: lowest mean RMS regardless of success.

    Two seeds prevent the single-seed artefact where high-gain combos that
    happen to minimise RMS on one specific wind realisation are selected even
    though they diverge on other seeds.  Two seeds also prevent the inverse
    artefact of the old approach (defaulting to the grid minimum the moment
    any combo gives success on seed=1).  Three seeds would be better but
    triples tuning cost; two captures most of the benefit.
    """
    best_Kp, best_Kd = Kp_grid[0], Kd_grid[0]
    best_sr  = -1.0
    best_rms = float("inf")
    for Kp in Kp_grid:
        for Kd in Kd_grid:
            pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
            r1 = _run_one(pid, plant, act, sen, dis, sc, seed=1)
            r2 = _run_one(pid, plant, act, sen, dis, sc, seed=2)
            sr  = 0.5 * (float(r1["success"]) + float(r2["success"]))
            rms = 0.5 * (float(r1["rms_error_deg"]) + float(r2["rms_error_deg"]))
            if sr > best_sr or (sr == best_sr and rms < best_rms):
                best_sr  = sr
                best_rms = rms
                best_Kp, best_Kd = Kp, Kd
    return best_Kp, best_Kd


KI_GRID_PID: list[float] = [0.5, 2.0, 8.0, 20.0]


def autotune_grid_pid(
    plant,
    act:  ActuatorParams,
    sen:  SensorParams,
    dis:  DisturbanceParams,
    sc:   ScenarioParams,
    Kp_grid: list[float] = KP_GRID,
    Kd_grid: list[float] = KD_GRID,
    Ki_grid: list[float] = None,
) -> tuple[float, float, float]:
    """
    Staged PID grid search.  Returns (best_Kp, best_Kd, best_Ki).

    Stage 1: PD search over Kp × Kd (same as autotune_grid).
    Stage 2: Ki search with Kp/Kd fixed.

    The two-stage approach avoids the 4× runtime penalty of a full 3D grid
    while capturing most of the benefit of integral action.  Limitation: the
    optimal Ki may shift the optimal Kp/Kd; treat results as a good approximation.
    """
    if Ki_grid is None:
        Ki_grid = KI_GRID_PID

    # Stage 1: find best Kp, Kd (reuse PD tuner)
    best_Kp, best_Kd = autotune_grid(plant, act, sen, dis, sc, Kp_grid, Kd_grid)

    # Stage 2: search Ki with fixed Kp, Kd
    best_Ki, best_sr = 0.0, -1.0
    for Ki in Ki_grid:
        pid = PIDParams(Kp=best_Kp, Kd=best_Kd, Ki=Ki,
                        u_max=act.u_max, i_lim=act.u_max)
        r = _run_one(pid, plant, act, sen, dis, sc, seed=1)
        sr = 1.0 if r["success"] else 0.0
        if sr > best_sr:
            best_sr = sr
            best_Ki = Ki
    return best_Kp, best_Kd, best_Ki


# ── Exp1: regime mapping ──────────────────────────────────────────────────────

def _eval_design_exp1(row: dict) -> dict:
    """
    Evaluate one design for Exp1: tune → nominal/under/over → classify.

    Uses full-physics config (all structural modules ON, no fault injection)
    so regime labels reflect the TRUE controllability boundary, not the
    simple-mode tuner's approximate proxy.  This resolves the regime validity
    problem where Exp1-INFEASIBLE designs succeeded at 98.6% in full physics.
    """
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    # Full-physics config: all structural modules ON, no fault injection.
    # Matches Exp4C reality config so regime labels are valid in that context.
    physics_cfg = FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, physics_cfg)

    best_Kp, best_Kd = autotune_grid(plant, act, sen, dis, sc)

    def eval_nominal(Kp, Kd):
        pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        # 3 seeds: wind is stochastic — single-seed is too noisy near the EASY/MARGINAL
        # RMS boundary (8 deg).  Under/over checks only need 1 seed since they produce
        # a binary pass/fail for the robustness score.
        runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in (1, 2, 3)]
        return _aggregate(runs)

    def eval_robustness(Kp, Kd):
        pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        return _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=1)])

    nominal = eval_nominal(best_Kp, best_Kd)
    under   = eval_robustness(UNDER_SCALE * best_Kp, UNDER_SCALE * best_Kd)
    over    = eval_robustness(OVER_SCALE  * best_Kp, OVER_SCALE  * best_Kd)

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
        # Progressive band metrics (fraction of seeds passing each threshold)
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
    )


def run_exp1(
    n_designs: int = 1200,
    seed: int = 42,
    n_jobs: int = -1,
    out_dir: Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Experiment 1: map EASY/MARGINAL/FRAGILE/INFEASIBLE over the 12-D LHS design space.

    Regime labels are now computed under full-physics conditions (nonlinear_aero,
    dyn_aero, thrust_curve, cg_shift ON; no fault injection) so that labels
    are valid in the Exp4C evaluation environment.
    Saves results to out_dir/exp1_regime_index_py.csv.
    """
    print(f"=== EXP1 (Python, full-physics regime labels) n={n_designs} seed={seed} ===")
    print(f"    Fidelity: full physics (nonlinear_aero, dyn_aero, thrust_curve, cg_shift ON)")
    print(f"    Gain grid: {len(KP_GRID)}x{len(KD_GRID)} = {len(KP_GRID)*len(KD_GRID)} combos")
    designs = sample_lhs(n_designs, seed)
    rows = designs.to_dict("records")

    results = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp1)(row) for row in rows
    )

    df = pd.DataFrame(results)
    counts = df["regime_label"].value_counts()
    print(f"Regime counts:  EASY={counts.get('EASY',0)}  MARGINAL={counts.get('MARGINAL',0)}  "
          f"FRAGILE={counts.get('FRAGILE',0)}  INFEASIBLE={counts.get('INFEASIBLE',0)}")
    # Divergence summary
    if "nominal_diverge_rate" in df.columns:
        n_diverge = (df["nominal_diverge_rate"] > 0).sum()
        print(f"Any divergence nominally: {n_diverge}/{n_designs} designs")

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
            motor_scale      = row.get("motor_scale"),
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
            diverged          = r.diverged,
            band_30_pass      = r.band_30_pass,
            band_20_pass      = r.band_20_pass,
            band_10_pass      = r.band_10_pass,
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
              "control_effectiveness", "motor_scale", "servo_slew_deg_s",
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
              "control_effectiveness", "motor_scale", "servo_slew_deg_s",
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
              "control_effectiveness", "motor_scale", "servo_slew_deg_s",
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
        if param in ("static_margin", "Cm_alpha", "motor_scale"):
            d["p_unstable"] = _est_p(
                d.get("static_margin", REF["static_margin"]),
                d.get("Cm_alpha",      REF["Cm_alpha"]),
                d.get("motor_scale",   REF["motor_scale"]),
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
    from design_space import DESIGN_LO, DESIGN_HI, DESIGN_NAMES, estimate_p_unstable, estimate_lambda_aero
    lo_map = dict(zip(DESIGN_NAMES, DESIGN_LO))
    hi_map = dict(zip(DESIGN_NAMES, DESIGN_HI))

    cfg = FidelityConfig.full()
    rows = []
    for param in params:
        sweep_vals = np.linspace(lo_map[param], hi_map[param], n_points)
        for val in sweep_vals:
            d = dict(ref_design)
            d[param] = float(val)
            # Recompute aerodynamic terms if needed
            if param in ("static_margin", "Cm_alpha", "motor_scale"):
                sm  = d.get("static_margin", REF["static_margin"])
                cma = d.get("Cm_alpha",      REF["Cm_alpha"])
                ms  = d.get("motor_scale",   REF["motor_scale"])
                d["lambda_aero"] = estimate_lambda_aero(sm, cma, ms)
                d["p_unstable"]  = estimate_p_unstable(sm, cma, ms)
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


# ── Exp5 regime-stratified servo slew payoff sweep ───────────────────────────

def _sweep_slew_one_design(row: dict, slew_vals: np.ndarray, seeds: list[int]) -> list[dict]:
    """
    Sweep servo_slew_deg_s across slew_vals for one design at its own gains.
    Returns per-(design, slew_val) success_rate averaged over seeds.
    """
    from local_analysis import _evaluate_design
    Kp = float(row["best_Kp"])
    Kd = float(row["best_Kd"])
    cfg = FidelityConfig.full()
    out = []
    for val in slew_vals:
        d = dict(row)
        d["servo_slew_deg_s"] = float(val)
        successes = []
        for seed in seeds:
            try:
                r = _evaluate_design(d, Kp, Kd, cfg, seed=seed)
                successes.append(int(r.success))
            except Exception:
                successes.append(0)
        out.append({
            "design_id":    row.get("design_id"),
            "regime_label": row.get("regime_label"),
            "slew_val":     float(val),
            "success_rate": float(np.mean(successes)),
        })
    return out


def run_exp5_slew_stratified(
    designs_df:      Optional[pd.DataFrame] = None,
    n_designs_regime: int = 60,
    n_slew_points:   int = 25,
    seeds:           list[int] = None,
    n_jobs:          int = -1,
    out_dir:         Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Regime-stratified servo slew payoff sweep.

    For each regime (EASY, MARGINAL, FRAGILE, INFEASIBLE), samples
    n_designs_regime designs and sweeps servo_slew_deg_s from 20 to 120 deg/s.
    Reports mean success_rate and quartiles per (regime, slew_val).

    Saves: exp5_slew_stratified_py.csv
    """
    if seeds is None:
        seeds = [1, 2, 3]

    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        if not py_path.exists():
            raise FileNotFoundError("Exp1 results not found.")
        designs_df = pd.read_csv(py_path)

    slew_vals = np.linspace(20.0, 120.0, n_slew_points)
    regime_labels = ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]

    all_rows = []
    for regime in regime_labels:
        sub = designs_df[designs_df["regime_label"] == regime]
        if sub.empty:
            print(f"  {regime}: no designs, skipping")
            continue
        sample = sub.sample(min(n_designs_regime, len(sub)), random_state=42).to_dict("records")
        print(f"  {regime}: sweeping {len(sample)} designs × {n_slew_points} slew points × {len(seeds)} seeds")

        per_design = Parallel(n_jobs=n_jobs)(
            delayed(_sweep_slew_one_design)(row, slew_vals, seeds) for row in sample
        )
        for design_rows in per_design:
            all_rows.extend(design_rows)

    raw_df = pd.DataFrame(all_rows)

    # Aggregate per (regime_label, slew_val)
    agg = (raw_df
           .groupby(["regime_label", "slew_val"])["success_rate"]
           .agg(["mean", "std", lambda x: np.percentile(x, 25), lambda x: np.percentile(x, 75)])
           .reset_index())
    agg.columns = ["regime_label", "slew_val", "success_rate_mean", "success_rate_std",
                   "success_rate_q25", "success_rate_q75"]

    out_path = out_dir / "exp5_slew_stratified_py.csv"
    agg.to_csv(out_path, index=False)
    print(f"\nSaved: {out_path}  ({len(agg)} rows)")
    return agg


# ── Exp4A: 10-module ablation, no fault injection ─────────────────────────────
#
# Experiment design:
#   Baseline = FidelityConfig.full() with thrust_var DISABLED.
#   This is a physically-grounded baseline: all real aerodynamic/actuator/sensor
#   physics are modeled but no fault event is injected.
#
#   Why exclude thrust_var:
#     The other 9 modules model physics always present in flight (wind, sensor
#     noise, servo limits, nonlinear aero, etc.).  thrust_var models a specific
#     motor degradation event that may or may not occur.  Mixing fault injection
#     into the fidelity baseline conflates "accurate physics" with "worst-case
#     scenario."  Exp4A measures physics fidelity requirements only.
#
#   Gains are RE-TUNED at the 10-module baseline for each design.
#   This is important because the full-physics mode (dyn_aero, nonlinear_aero)
#   changes dynamics significantly vs legacy simple mode, so Exp1 gains are
#   suboptimal here.
#
#   Seeds: 5 per condition (baseline + each of 10 ablations = 11 × 5 = 55 runs).
#   Total: 1200 × 55 = 66 000 simulations.
#
#   build_plant() now passes design-specific Cm_alpha_signed, static_margin,
#   Iyy_kgm2, m_kg, max_gimbal_deg to PlantParams, so each design's full-physics
#   aerodynamics are correct (not all evaluated as the reference rocket).
#
# Output: exp4a_ablation_py.csv — one row per design, wide format.
#   Columns: design params | baseline metrics | per-module deltas and flip flags
#   Metadata written to exp4a_ablation_meta.txt alongside the CSV.

EXP4A_MODULES: list[str] = [
    "wind",
    "backlash",
    "slew",
    "latency",
    "sensor_noise",
    "deadband",
    "nonlinear_aero",
    "dyn_aero",
    "thrust_curve",
    "cg_shift",
]
EXP4A_N_SEEDS: int = 5


def _exp4a_baseline_cfg() -> FidelityConfig:
    """10-module baseline for Exp4A (all physics on, no fault injection)."""
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _eval_design_exp4a(row: dict) -> dict:
    """
    Exp4A: 10-module ablation for one design.

    Protocol:
      1. Build design-specific PlantParams (now includes physical aero params).
      2. Tune Kp/Kd at 10-module baseline (seed=1, 4×4 grid).
      3. Evaluate baseline with EXP4A_N_SEEDS seeds.
      4. For each of 10 modules: disable it, evaluate with EXP4A_N_SEEDS seeds.
      5. Compute delta metrics and GO/NOGO flip flags.
    """
    plant    = build_plant(row)
    base_act = build_actuator(row)
    base_sen = build_sensor(row)
    base_dis = build_disturbance(row)
    base_sc  = build_scenario(theta0_bias_std=0.0)
    # No fault on base_sc — Exp4A baseline is fault-free

    baseline_cfg = _exp4a_baseline_cfg()
    seeds = tuple(range(1, EXP4A_N_SEEDS + 1))

    # Tune at 10-module baseline
    act_t, sen_t, dis_t, sc_t = apply_fidelity_config(
        base_act, base_sen, base_dis, base_sc, baseline_cfg
    )
    best_Kp, best_Kd = autotune_grid(plant, act_t, sen_t, dis_t, sc_t)
    pid = PIDParams(Kp=best_Kp, Kd=best_Kd, Ki=0.0,
                    u_max=base_act.u_max, i_lim=base_act.u_max)

    # Evaluate baseline
    base_m = _eval_one_fidelity(
        plant, base_act, base_sen, base_dis, base_sc, pid, baseline_cfg, seeds
    )
    base_go = base_m["success_rate"] >= FRAGILE_SUCCESS_RATE

    out = dict(
        rocket_id    = row["rocket_id"],
        design_id    = row["design_id"],
        regime_label = row.get("regime_label", "UNKNOWN"),
        best_Kp      = best_Kp,
        best_Kd      = best_Kd,
    )
    for k in ["p_unstable", "mass", "Iyy", "static_margin", "Cm_alpha",
              "control_effectiveness", "motor_scale", "servo_slew_deg_s",
              "max_gimbal_deg", "latency_steps", "deadband", "backlash",
              "wind_strength"]:
        out[k] = row.get(k)

    out["baseline_success_rate"] = base_m["success_rate"]
    out["baseline_rms"]          = base_m["rms_error_deg"]
    out["baseline_peak"]         = base_m["peak_error_deg"]
    out["baseline_max_theta"]    = base_m["max_theta_deg"]
    out["baseline_u_sat"]        = base_m["u_cmd_sat_frac"]
    out["baseline_slew_sat"]     = base_m["slew_sat_frac"]
    out["baseline_go"]           = int(base_go)

    # Single-module ablations
    delta_rms_abs: dict[str, float] = {}
    for module in EXP4A_MODULES:
        abl_cfg = baseline_cfg.ablate(module)
        abl_m   = _eval_one_fidelity(
            plant, base_act, base_sen, base_dis, base_sc, pid, abl_cfg, seeds
        )
        abl_go = abl_m["success_rate"] >= FRAGILE_SUCCESS_RATE

        d_success = abl_m["success_rate"]  - base_m["success_rate"]
        d_rms     = abl_m["rms_error_deg"] - base_m["rms_error_deg"]
        d_peak    = abl_m["peak_error_deg"] - base_m["peak_error_deg"]
        d_theta   = abl_m["max_theta_deg"] - base_m["max_theta_deg"]
        flip      = int(base_go != abl_go)

        # Direction: positive d_success = removing module improved outcome
        # (module was making things worse).  "helps" / "hurts" / "" (no flip)
        flip_dir = ("helps" if d_success > 0 else "hurts") if flip else ""

        out[f"abl_success_{module}"]   = abl_m["success_rate"]
        out[f"delta_success_{module}"] = d_success
        out[f"delta_rms_{module}"]     = d_rms
        out[f"delta_peak_{module}"]    = d_peak
        out[f"delta_theta_{module}"]   = d_theta
        out[f"decision_flip_{module}"] = flip
        out[f"flip_dir_{module}"]      = flip_dir

        delta_rms_abs[module] = abs(d_rms)

    # Summary columns
    out["dominant_module"]       = max(delta_rms_abs, key=delta_rms_abs.get)
    out["dominant_delta_rms"]    = delta_rms_abs[out["dominant_module"]]
    out["n_decision_flips"]      = sum(out[f"decision_flip_{m}"] for m in EXP4A_MODULES)
    out["decision_flip_modules"] = (
        " ".join(m for m in EXP4A_MODULES if out[f"decision_flip_{m}"] == 1) or "none"
    )
    # effect_any: fraction of modules that cause any flip (same as n_decision_flips/10)
    out["fidelity_complexity"]   = out["n_decision_flips"]  # alias for clarity

    return out


def run_exp4a_ablation(
    designs_df: Optional[pd.DataFrame] = None,
    n_jobs:     int = -1,
    out_dir:    Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Experiment 4A: 10-module fidelity ablation (no thrust_var fault injection).

    Baseline: FidelityConfig with all 10 physics modules active (thrust_var=False).
    Gains re-tuned at baseline per design (4×4 grid, seed=1).
    Seeds: EXP4A_N_SEEDS per condition.

    Key differences from legacy Exp4 ablation:
      - No fault injection in baseline (thrust_var excluded)
      - Gains re-tuned at new baseline (not frozen from Exp1)
      - build_plant() now passes design-specific physical params (Cm_alpha, Iyy, etc.)
      - 5 seeds vs 3 → finer success_rate resolution for GO/NOGO decisions

    Saves:
      exp4a_ablation_py.csv   (one row per design, wide format)
      exp4a_ablation_meta.txt (experiment metadata)
    """
    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        if not py_path.exists():
            raise FileNotFoundError(
                "Exp1 results not found.  Run run_exp1() first or pass designs_df."
            )
        designs_df = pd.read_csv(py_path)

    n = len(designs_df)
    print(f"=== EXP4A ABLATION  n={n} designs ===")
    print(f"    Modules ({len(EXP4A_MODULES)}): {EXP4A_MODULES}")
    print(f"    Seeds per condition : {EXP4A_N_SEEDS}")
    print(f"    Baseline            : 10-module full physics (thrust_var=False)")
    print(f"    Gains               : re-tuned at baseline per design")
    print(f"    Simulations total   : {n} × {1 + len(EXP4A_MODULES)} conditions "
          f"× {EXP4A_N_SEEDS} seeds + {n} × 16 tune = "
          f"{n * (1 + len(EXP4A_MODULES)) * EXP4A_N_SEEDS + n * 16:,}")
    rows = designs_df.to_dict("records")

    results = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp4a)(row) for row in rows
    )

    df = pd.DataFrame(results)

    # ── Print frequency-of-effect summary ─────────────────────────────────
    print("\n  Frequency of effect by module (% designs with any decision flip):")
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = df[df["regime_label"] == regime]
        if sub.empty:
            continue
        parts = []
        for m in EXP4A_MODULES:
            col = f"decision_flip_{m}"
            if col in sub.columns:
                parts.append(f"{m}={sub[col].mean()*100:.1f}%")
        print(f"  [{regime:10s}] {', '.join(parts)}")

    print("\n  Fidelity complexity (n modules that flip decision) by regime:")
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = df[df["regime_label"] == regime]
        if sub.empty:
            continue
        dist = sub["n_decision_flips"].value_counts().sort_index()
        print(f"  [{regime:10s}] {dict(dist)}")

    print("\n  Dominant module by design count:")
    dom = df["dominant_module"].value_counts()
    for m, cnt in dom.items():
        print(f"    {m:<20s}: {cnt:4d}  ({100*cnt/len(df):.1f}%)")

    # ── Save ──────────────────────────────────────────────────────────────
    out_csv  = out_dir / "exp4a_ablation_py.csv"
    out_meta = out_dir / "exp4a_ablation_meta.txt"
    df.to_csv(out_csv, index=False)

    meta_lines = [
        "Exp4A Ablation — Metadata",
        "=" * 40,
        f"n_designs         : {len(df)}",
        f"n_modules         : {len(EXP4A_MODULES)}",
        f"modules           : {EXP4A_MODULES}",
        f"n_seeds           : {EXP4A_N_SEEDS}",
        f"baseline          : FidelityConfig.full() with thrust_var=False",
        f"gains             : re-tuned at baseline (4x4 grid, seed=1)",
        f"go_nogo_threshold : FRAGILE_SUCCESS_RATE = {FRAGILE_SUCCESS_RATE}",
        f"build_plant       : design-specific Cm_alpha, static_margin, Iyy, mass, max_gimbal",
        f"l_nozzle_m        : 0.25 m (fixed, all designs share 0.5 m airframe)",
        f"aero_model        : Cm_alpha_physical = CN_alpha * static_margin (fixed 2026-06-04)",
        f"cg_bug_fixed      : x_cg_dry = l_nozzle_m (fixed 2026-06-04)",
        "",
        "Regime counts:",
    ]
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        cnt = (df["regime_label"] == regime).sum()
        meta_lines.append(f"  {regime:10s}: {cnt}")
    out_meta.write_text("\n".join(meta_lines))

    print(f"\nSaved: {out_csv}")
    print(f"Saved: {out_meta}")
    return df


# ── Exp4B: frozen-gain 10-module ablation ─────────────────────────────────────
#
# Scientific question:
#   "Given gains tuned in a simple (legacy) simulator, which physical fidelity
#    modules cause decision flips when reality includes full physics?"
#
# This is the engineering-relevant question: an engineer tunes their controller
# in a simplified simulation, then the rocket flies in reality.  Which physical
# effects — absent from their simulator — invalidate their design decision?
#
# Key contrast with Exp4A:
#   Exp4A re-tunes gains at full-physics baseline → measures RESIDUAL sensitivity
#         after the controller has adapted to each module.
#   Exp4B freezes gains from simple-mode tuning → measures TRUE sensitivity
#         to each module given a suboptimal (simple-mode) controller.
#
# Gain policy:
#   Gains sourced from Exp1 best_Kp / best_Kd (tuned at legacy simple-mode
#   conditions: simple plant dynamics, design-specific actuator/sensor/disturbance).
#   Fallback: autotune at FidelityConfig.simple() if Exp1 gains unavailable.
#   Gains are NEVER re-tuned during Exp4B.
#
# Baseline:
#   Same 10-module FidelityConfig as Exp4A (thrust_var=False).
#   The controller is evaluated against full physics with its simple-mode gains.
#
# Interpretation guide (produced in summary output):
#   Module m is "gain-compensated" if:  freq(Exp4B) >> freq(Exp4A)
#     → an optimally-tuned controller can recover from omitting this module
#     → simple-mode tuning is the vulnerability, not the physics itself
#   Module m is "physics-critical" if:  freq(Exp4B) ≈ freq(Exp4A)
#     → even optimal gains cannot compensate for omitting this module
#     → the physics fundamentally changes the outcome
#
# Output: exp4b_frozen_gain_ablation_py.csv + exp4b_frozen_gain_meta.txt

EXP4B_MODULES: list[str] = EXP4A_MODULES   # same 10 modules — direct comparison
EXP4B_N_SEEDS: int = 5


def _eval_design_exp4b(row: dict) -> dict:
    """
    Exp4B: frozen-gain 10-module ablation for one design.

    Protocol:
      1. Load Kp/Kd from Exp1 (simple-mode-tuned gains).
         Fallback: autotune at FidelityConfig.simple() if not available.
      2. Build design-specific PlantParams (physical aero params included).
      3. Evaluate 10-module baseline with EXP4B_N_SEEDS seeds — gains NOT re-tuned.
      4. For each of 10 modules: disable it, evaluate with EXP4B_N_SEEDS seeds.
      5. Compute delta metrics and GO/NOGO flip flags.

    Gains are identical across baseline and all 10 ablation conditions.
    """
    plant    = build_plant(row)
    base_act = build_actuator(row)
    base_sen = build_sensor(row)
    base_dis = build_disturbance(row)
    base_sc  = build_scenario(theta0_bias_std=0.0)
    # No fault injection — Exp4B baseline is fault-free

    seeds = tuple(range(1, EXP4B_N_SEEDS + 1))

    # ── Gain source: Exp1 simple-mode tuning ─────────────────────────────
    gain_source = "exp1"
    try:
        Kp = float(row["best_Kp"])
        Kd = float(row["best_Kd"])
        if not (np.isfinite(Kp) and np.isfinite(Kd)):
            raise ValueError("non-finite gains")
    except (KeyError, TypeError, ValueError):
        # Fallback: tune at FidelityConfig.simple() (all modules OFF)
        simple_cfg = FidelityConfig.simple()
        act_s, sen_s, dis_s, sc_s = apply_fidelity_config(
            base_act, base_sen, base_dis, base_sc, simple_cfg
        )
        Kp, Kd = autotune_grid(plant, act_s, sen_s, dis_s, sc_s)
        gain_source = "simple_tune_fallback"

    pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0,
                    u_max=base_act.u_max, i_lim=base_act.u_max)

    # ── Evaluate 10-module baseline ───────────────────────────────────────
    baseline_cfg = _exp4a_baseline_cfg()   # shared 10-module config
    base_m  = _eval_one_fidelity(
        plant, base_act, base_sen, base_dis, base_sc, pid, baseline_cfg, seeds
    )
    base_go = base_m["success_rate"] >= FRAGILE_SUCCESS_RATE

    out = dict(
        rocket_id    = row["rocket_id"],
        design_id    = row["design_id"],
        regime_label = row.get("regime_label", "UNKNOWN"),
        gain_Kp      = Kp,
        gain_Kd      = Kd,
        gain_source  = gain_source,
    )
    for k in ["p_unstable", "mass", "Iyy", "static_margin", "Cm_alpha",
              "control_effectiveness", "motor_scale", "servo_slew_deg_s",
              "max_gimbal_deg", "latency_steps", "deadband", "backlash",
              "wind_strength"]:
        out[k] = row.get(k)

    out["baseline_success_rate"] = base_m["success_rate"]
    out["baseline_rms"]          = base_m["rms_error_deg"]
    out["baseline_peak"]         = base_m["peak_error_deg"]
    out["baseline_max_theta"]    = base_m["max_theta_deg"]
    out["baseline_u_sat"]        = base_m["u_cmd_sat_frac"]
    out["baseline_go"]           = int(base_go)

    # ── Single-module ablations ───────────────────────────────────────────
    delta_rms_abs: dict[str, float] = {}
    for module in EXP4B_MODULES:
        abl_cfg = baseline_cfg.ablate(module)
        abl_m   = _eval_one_fidelity(
            plant, base_act, base_sen, base_dis, base_sc, pid, abl_cfg, seeds
        )
        abl_go = abl_m["success_rate"] >= FRAGILE_SUCCESS_RATE

        d_success = abl_m["success_rate"]  - base_m["success_rate"]
        d_rms     = abl_m["rms_error_deg"] - base_m["rms_error_deg"]
        d_peak    = abl_m["peak_error_deg"] - base_m["peak_error_deg"]
        d_theta   = abl_m["max_theta_deg"] - base_m["max_theta_deg"]
        flip      = int(base_go != abl_go)
        flip_dir  = ("helps" if d_success > 0 else "hurts") if flip else ""

        out[f"abl_success_{module}"]   = abl_m["success_rate"]
        out[f"delta_success_{module}"] = d_success
        out[f"delta_rms_{module}"]     = d_rms
        out[f"delta_peak_{module}"]    = d_peak
        out[f"delta_theta_{module}"]   = d_theta
        out[f"decision_flip_{module}"] = flip
        out[f"flip_dir_{module}"]      = flip_dir

        delta_rms_abs[module] = abs(d_rms)

    out["dominant_module"]       = max(delta_rms_abs, key=delta_rms_abs.get)
    out["dominant_delta_rms"]    = delta_rms_abs[out["dominant_module"]]
    out["n_decision_flips"]      = sum(out[f"decision_flip_{m}"] for m in EXP4B_MODULES)
    out["decision_flip_modules"] = (
        " ".join(m for m in EXP4B_MODULES if out[f"decision_flip_{m}"] == 1) or "none"
    )
    out["fidelity_complexity"]   = out["n_decision_flips"]

    return out


def run_exp4b_ablation(
    designs_df: Optional[pd.DataFrame] = None,
    exp4a_df:   Optional[pd.DataFrame] = None,
    n_jobs:     int = -1,
    out_dir:    Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Experiment 4B: frozen-gain 10-module ablation.

    Gains are fixed at Exp1 simple-mode values.  Evaluates all 10 modules
    against the 10-module full-physics baseline to measure which modules
    cause decision flips given a suboptimal (simple-mode) controller.

    Optionally compares against Exp4A (re-tuned gains) to separate:
      - Physics sensitivity (module matters regardless of gain quality)
      - Gain compensation (module only matters because gains are suboptimal)

    Saves:
      exp4b_frozen_gain_ablation_py.csv
      exp4b_frozen_gain_meta.txt
    """
    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        if not py_path.exists():
            raise FileNotFoundError("Exp1 results not found.")
        designs_df = pd.read_csv(py_path)
        if "best_Kp" not in designs_df.columns:
            raise ValueError(
                "Exp1 CSV missing best_Kp/best_Kd.  Re-run Exp1 first."
            )

    n = len(designs_df)
    n_sim = n * (1 + len(EXP4B_MODULES)) * EXP4B_N_SEEDS
    print(f"=== EXP4B FROZEN-GAIN ABLATION  n={n} designs ===")
    print(f"    Modules ({len(EXP4B_MODULES)}): {EXP4B_MODULES}")
    print(f"    Seeds per condition  : {EXP4B_N_SEEDS}")
    print(f"    Baseline             : 10-module full physics (thrust_var=False)")
    print(f"    Gains                : FROZEN from Exp1 simple-mode tuning")
    print(f"    Simulations total    : {n_sim:,}  (no tuning runs — gains pre-loaded)")
    rows = designs_df.to_dict("records")

    results = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp4b)(row) for row in rows
    )

    df = pd.DataFrame(results)

    # ── Frequency-of-effect table ─────────────────────────────────────────
    print("\n  Frequency of effect by module × regime (% designs with decision flip):")
    header = f"  {'':12}" + "".join(f"{m[:9]:>10}" for m in EXP4B_MODULES)
    print(header)
    freq_4b: dict[str, dict[str, float]] = {}
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = df[df["regime_label"] == regime]
        if sub.empty:
            continue
        vals = {m: sub[f"decision_flip_{m}"].mean() * 100 for m in EXP4B_MODULES}
        freq_4b[regime] = vals
        row_str = f"  {regime:<12}" + "".join(f"{vals[m]:9.1f}%" for m in EXP4B_MODULES)
        print(row_str)

    # ── Comparison with Exp4A (if available) ─────────────────────────────
    if exp4a_df is None:
        exp4a_path = out_dir / "exp4a_ablation_py.csv"
        if exp4a_path.exists():
            exp4a_df = pd.read_csv(exp4a_path)

    if exp4a_df is not None:
        print("\n  Gain-compensation analysis (Exp4B - Exp4A frequency, % points):")
        print("  Positive = frozen-gain is MORE sensitive => module is gain-compensated")
        print("  Near zero = sensitivity persists under optimal gains => physics-critical")
        print(f"  {'':12}" + "".join(f"{m[:9]:>10}" for m in EXP4B_MODULES))
        for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
            sub_a = exp4a_df[exp4a_df["regime_label"] == regime]
            if sub_a.empty or regime not in freq_4b:
                continue
            diffs = []
            for m in EXP4B_MODULES:
                col = f"decision_flip_{m}"
                fa = sub_a[col].mean() * 100 if col in sub_a.columns else 0.0
                fb = freq_4b[regime].get(m, 0.0)
                diffs.append(fb - fa)
            row_str = f"  {regime:<12}" + "".join(f"{d:+9.1f}%" for d in diffs)
            print(row_str)

        print("\n  Classification (threshold: >=5pp difference):")
        print(f"  {'':20}  {'Physics-critical':>18}  {'Gain-compensated':>18}")
        for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
            sub_a = exp4a_df[exp4a_df["regime_label"] == regime]
            if sub_a.empty or regime not in freq_4b:
                continue
            phys_crit, gain_comp = [], []
            for m in EXP4B_MODULES:
                col = f"decision_flip_{m}"
                fa = sub_a[col].mean() * 100 if col in sub_a.columns else 0.0
                fb = freq_4b[regime].get(m, 0.0)
                diff = fb - fa
                if diff >= 5.0:
                    gain_comp.append(m)
                elif fb >= 5.0:
                    phys_crit.append(m)
            print(f"  {regime:<20}  {', '.join(phys_crit) or 'none':>18}  "
                  f"{', '.join(gain_comp) or 'none':>18}")

    # ── Fidelity complexity ───────────────────────────────────────────────
    print("\n  Fidelity complexity (n modules that flip decision) by regime:")
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = df[df["regime_label"] == regime]
        if sub.empty:
            continue
        pct_zero = (sub["n_decision_flips"] == 0).mean() * 100
        pct_ge1  = (sub["n_decision_flips"] >= 1).mean() * 100
        pct_ge3  = (sub["n_decision_flips"] >= 3).mean() * 100
        mean_f   = sub["n_decision_flips"].mean()
        print(f"  {regime:<10}  zero={pct_zero:.1f}%  1+={pct_ge1:.1f}%  "
              f"3+={pct_ge3:.1f}%  mean={mean_f:.2f}")

    # ── Directional effects ───────────────────────────────────────────────
    print("\n  Directional effects (helps = removing module improves outcome):")
    for m in EXP4B_MODULES:
        helps = (df[f"flip_dir_{m}"] == "helps").sum()
        hurts = (df[f"flip_dir_{m}"] == "hurts").sum()
        total = helps + hurts
        if total > 0:
            print(f"    {m:<20s}: {total:3d} flips  ({helps} helps / {hurts} hurts)")

    # ── Baseline success rates ────────────────────────────────────────────
    print("\n  Baseline success rate at full-physics (frozen gains) by regime:")
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = df[df["regime_label"] == regime]
        if sub.empty:
            continue
        sr  = sub["baseline_success_rate"]
        go  = (sr >= FRAGILE_SUCCESS_RATE).mean() * 100
        print(f"  {regime:<10}  mean_sr={sr.mean():.3f}  GO_rate={go:.1f}%  "
              f"[{sr.min():.2f}, {sr.max():.2f}]")

    # ── Save ──────────────────────────────────────────────────────────────
    out_csv  = out_dir / "exp4b_frozen_gain_ablation_py.csv"
    out_meta = out_dir / "exp4b_frozen_gain_meta.txt"
    df.to_csv(out_csv, index=False)

    gain_sources = df["gain_source"].value_counts().to_dict()
    meta_lines = [
        "Exp4B Frozen-Gain Ablation — Metadata",
        "=" * 40,
        f"n_designs          : {len(df)}",
        f"n_modules          : {len(EXP4B_MODULES)}",
        f"modules            : {EXP4B_MODULES}",
        f"n_seeds            : {EXP4B_N_SEEDS}",
        f"baseline           : 10-module full physics (thrust_var=False)",
        f"gains              : FROZEN from Exp1 simple-mode tuning",
        f"gain_sources       : {gain_sources}",
        f"go_nogo_threshold  : FRAGILE_SUCCESS_RATE = {FRAGILE_SUCCESS_RATE}",
        f"build_plant        : design-specific Cm_alpha, static_margin, Iyy, mass",
        f"l_nozzle_m         : 0.25 m (fixed, all designs share 0.5 m airframe)",
        f"scientific_question: Given simple-mode gains, which modules flip decisions?",
        f"contrast_with      : Exp4A (re-tuned gains) — separates physics sensitivity",
        f"                     from gain compensation",
        "",
        "Regime counts:",
    ]
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        cnt = (df["regime_label"] == regime).sum()
        meta_lines.append(f"  {regime:10s}: {cnt}")
    out_meta.write_text("\n".join(meta_lines))

    print(f"\nSaved: {out_csv}")
    print(f"Saved: {out_meta}")
    return df


# ── Exp4C: tune-in-ablated, test-in-reality ──────────────────────────────────
#
# Scientific question (correct one):
#   "For each physics module X, if an engineer builds a simulator that omits X
#    but includes everything else, how wrong are their performance predictions
#    and engineering decisions?"
#
# This directly models what happens in practice: an engineer tunes their
# controller in a simulator that is missing some physics, then the rocket flies
# in reality (full physics).  The gap between the engineer's prediction and
# the actual outcome is the cost of omitting that module.
#
# Key differences from Exp4A / Exp4B:
#   Exp4A: tune at full → ablate → measure residual sensitivity (answers: does module
#          matter after the controller adapts to it?)
#   Exp4B: tune at simple → ablate from full → measure which modules flip decisions
#          (measures ablation sensitivity given suboptimal gains)
#   Exp4C: for each module X, tune at (full - X) → evaluate at full → measure
#          prediction error (answers: what is the cost of not knowing about X?)
#
# Protocol per module X:
#   1. Build "engineer's sim" = full 10-module physics MINUS module X
#   2. Tune gains in engineer's sim (4x4 grid, seed=1)
#   3. Evaluate those gains in "reality" (full 10-module physics), 5 seeds
#   4. Evaluate those gains in engineer's sim (engineer's prediction), 5 seeds
#   5. Compute: rms_prediction_error = |rms_reality - rms_predicted|
#              rms_bias = rms_reality - rms_predicted (positive = sim too optimistic)
#              decision_error = go_predicted != go_reality
#
# Motor standardization:
#   All designs use F15 motor. thrust_curve=ON → F15 profile. thrust_curve=OFF →
#   constant F15 average (14.4N × motor_scale). Design-space "motor_scale" parameter
#   scales both thrust and motor mass; it replaced the old "thrust" proxy parameter.
#   (simulator.py fix: T_now = F15_AVG_THRUST_N when thrust_curve=OFF)
#
# Output format: LONG (one row per design x module).
#   1200 designs x 11 conditions (10 modules + reference) = 13,200 rows.
#   Reference condition: tune at full reality, evaluate at full reality (rms_bias=0 by def).
#
# Primary metric:  rms_prediction_error (RMS in deg, continuous — no threshold compression)
# Secondary metric: decision_error (GO/NOGO mismatch at FRAGILE_SUCCESS_RATE threshold)
#
# Output: exp4c_prediction_error_py.csv + exp4c_prediction_error_meta.txt

EXP4C_MODULES: list[str] = [
    # Structural physics modules only — wind and sensor_noise are excluded.
    # Wind and sensor noise are stochastic uncertainty parameters that should be
    # characterised via Monte Carlo robustness testing, NOT ablated from the
    # design simulator. Omitting them from the sim doesn't change HOW you tune
    # the controller (gain structure is the same), only the expected-value metric.
    # These 8 modules DO change the closed-loop dynamics and therefore change
    # the optimal gain set — that is the correct fidelity question.
    "backlash",
    "slew",
    "latency",
    "deadband",
    "nonlinear_aero",
    "dyn_aero",
    "thrust_curve",
    "cg_shift",
]
EXP4C_N_SEEDS: int = 5


def _eval_design_exp4c(row: dict, controller_type: str = "PD") -> list[dict]:
    """
    Exp4C worker: tune-in-ablated, test-in-reality for one design.

    controller_type: "PD" (default) or "PID".
      PD  — pure proportional-derivative; Ki=0 everywhere.
      PID — staged PD then Ki search via autotune_grid_pid.

    Returns a list of dicts (one per module + one reference condition).
    Each dict has the same schema so they can be concatenated directly.
    """
    plant    = build_plant(row)
    base_act = build_actuator(row)
    base_sen = build_sensor(row)
    base_dis = build_disturbance(row)
    base_sc  = build_scenario(theta0_bias_std=0.0)

    reality_cfg = _exp4a_baseline_cfg()  # all 10 modules ON, thrust_var=OFF
    seeds   = tuple(range(1, EXP4C_N_SEEDS + 1))
    use_pid = controller_type.upper() == "PID"

    def _tune(act, sen, dis, sc):
        """Tune gains in the given sim config; return (Kp, Kd, Ki)."""
        if use_pid:
            return autotune_grid_pid(plant, act, sen, dis, sc)
        Kp, Kd = autotune_grid(plant, act, sen, dis, sc)
        return Kp, Kd, 0.0

    # Design metadata passed through to every row
    base_info = dict(
        design_id       = row["design_id"],
        rocket_id       = row["rocket_id"],
        regime_label    = row.get("regime_label", "UNKNOWN"),
        controller_type = controller_type,
    )
    for k in ["p_unstable", "mass", "Iyy", "static_margin", "Cm_alpha",
              "control_effectiveness", "motor_scale", "servo_slew_deg_s",
              "max_gimbal_deg", "latency_steps", "deadband", "backlash",
              "wind_strength"]:
        base_info[k] = row.get(k)

    output_rows: list[dict] = []

    # ── Reference: tune at full reality, evaluate at full reality ─────────
    act_r, sen_r, dis_r, sc_r = apply_fidelity_config(
        base_act, base_sen, base_dis, base_sc, reality_cfg
    )
    ref_Kp, ref_Kd, ref_Ki = _tune(act_r, sen_r, dis_r, sc_r)
    ref_pid = PIDParams(Kp=ref_Kp, Kd=ref_Kd, Ki=ref_Ki,
                        u_max=base_act.u_max, i_lim=base_act.u_max)
    ref_m  = _eval_one_fidelity(
        plant, base_act, base_sen, base_dis, base_sc, ref_pid, reality_cfg, seeds
    )
    ref_go = int(ref_m["success_rate"] >= FRAGILE_SUCCESS_RATE)
    output_rows.append({
        **base_info,
        "module_ablated":       "reference",
        "Kp":                   ref_Kp,
        "Kd":                   ref_Kd,
        "Ki":                   ref_Ki,
        "rms_predicted":        ref_m["rms_error_deg"],
        "rms_reality":          ref_m["rms_error_deg"],
        "success_predicted":    ref_m["success_rate"],
        "success_reality":      ref_m["success_rate"],
        "rms_prediction_error": 0.0,
        "rms_bias":             0.0,
        "optimistic":           0,
        "go_predicted":         ref_go,
        "go_reality":           ref_go,
        "decision_error":       0,
    })

    # ── Per-module ablations ───────────────────────────────────────────────
    for module in EXP4C_MODULES:
        ablated_cfg = reality_cfg.ablate(module)

        # Engineer tunes gains in their incomplete simulator
        act_a, sen_a, dis_a, sc_a = apply_fidelity_config(
            base_act, base_sen, base_dis, base_sc, ablated_cfg
        )
        Kp_a, Kd_a, Ki_a = _tune(act_a, sen_a, dis_a, sc_a)
        pid_a = PIDParams(Kp=Kp_a, Kd=Kd_a, Ki=Ki_a,
                          u_max=base_act.u_max, i_lim=base_act.u_max)

        pred_m = _eval_one_fidelity(
            plant, base_act, base_sen, base_dis, base_sc, pid_a, ablated_cfg, seeds
        )
        real_m = _eval_one_fidelity(
            plant, base_act, base_sen, base_dis, base_sc, pid_a, reality_cfg, seeds
        )

        go_pred  = int(pred_m["success_rate"] >= FRAGILE_SUCCESS_RATE)
        go_real  = int(real_m["success_rate"]  >= FRAGILE_SUCCESS_RATE)
        rms_bias = real_m["rms_error_deg"] - pred_m["rms_error_deg"]

        output_rows.append({
            **base_info,
            "module_ablated":       module,
            "Kp":                   Kp_a,
            "Kd":                   Kd_a,
            "Ki":                   Ki_a,
            "rms_predicted":        pred_m["rms_error_deg"],
            "rms_reality":          real_m["rms_error_deg"],
            "success_predicted":    pred_m["success_rate"],
            "success_reality":      real_m["success_rate"],
            "rms_prediction_error": abs(rms_bias),
            "rms_bias":             rms_bias,
            "optimistic":           int(rms_bias > 0),
            "go_predicted":         go_pred,
            "go_reality":           go_real,
            "decision_error":       int(go_pred != go_real),
        })

    return output_rows


def run_exp4c(
    designs_df:      Optional[pd.DataFrame] = None,
    controller_type: str = "PD",
    n_jobs:          int = -1,
    out_dir:         Path = RESULT_DIR,
) -> pd.DataFrame:
    """
    Experiment 4C: tune-in-ablated, test-in-reality fidelity necessity study.

    controller_type: "PD" (default) or "PID".
      Run with both to produce the controller-comparison atlas.
      PD  output: exp4c_prediction_error_pd_py.csv
      PID output: exp4c_prediction_error_pid_py.csv

    For each physics module X:
      - Tune gains (using controller_type) in simulator missing X
      - Evaluate in full-physics reality
      - Measure: RMS prediction error, bias direction, decision error rate

    Output: long format, one row per (design x module).
    Primary metric:   rms_prediction_error = |rms_predicted - rms_reality|
    Secondary metric: decision_error (GO/NOGO mismatch)
    Bias direction:   rms_bias > 0 means sim was too optimistic (reality worse)
    """
    ctype = controller_type.upper()
    if ctype not in ("PD", "PID"):
        raise ValueError(f"controller_type must be 'PD' or 'PID', got '{controller_type}'")

    if designs_df is None:
        py_path = out_dir / "exp1_regime_index_py.csv"
        if not py_path.exists():
            raise FileNotFoundError(
                "Exp1 results not found.  Run run_exp1() first or pass designs_df."
            )
        designs_df = pd.read_csv(py_path)

    n = len(designs_df)
    n_conditions = 1 + len(EXP4C_MODULES)
    tune_evals_per_cond = 16 if ctype == "PD" else 20   # PID: 16 PD + 4 Ki stage
    n_tune  = n * n_conditions * tune_evals_per_cond
    n_eval  = n * (1 + 2 * len(EXP4C_MODULES)) * EXP4C_N_SEEDS
    n_total = n_tune + n_eval

    print(f"=== EXP4C  TUNE-IN-ABLATED, TEST-IN-REALITY  n={n} designs ===")
    print(f"    Controller type          : {ctype}")
    print(f"    Modules ({len(EXP4C_MODULES)}): {EXP4C_MODULES}")
    print(f"    Seeds per eval condition : {EXP4C_N_SEEDS}")
    print(f"    Reality config           : 10-module full physics (thrust_var=False)")
    print(f"    Motor                    : F15 for all designs; constant 14.4N when thrust_curve=OFF")
    print(f"    Output format            : long (one row per design x module)")
    print(f"    Approx simulations       : {n_total:,}  (tune={n_tune:,}  eval={n_eval:,})")
    rows = designs_df.to_dict("records")

    all_lists = Parallel(n_jobs=n_jobs, verbose=5)(
        delayed(_eval_design_exp4c)(row, controller_type=ctype) for row in rows
    )

    flat_rows = [r for per_design in all_lists for r in per_design]
    df = pd.DataFrame(flat_rows)

    # ── Summary printout ──────────────────────────────────────────────────
    abl = df[df["module_ablated"] != "reference"]

    header = f"  {'':14}" + "".join(f"{m[:10]:>11}" for m in EXP4C_MODULES)

    print(f"\n  Mean RMS prediction error (deg) by module x regime:")
    print(header)
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = abl[abl["regime_label"] == regime]
        if sub.empty:
            continue
        vals = [f"{sub[sub['module_ablated']==m]['rms_prediction_error'].mean():10.2f}"
                for m in EXP4C_MODULES]
        print(f"  {regime:<14}" + "".join(vals))

    print(f"\n  Decision error rate (%) by module x regime:")
    print(header)
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = abl[abl["regime_label"] == regime]
        if sub.empty:
            continue
        vals = [f"{sub[sub['module_ablated']==m]['decision_error'].mean()*100:10.1f}%"
                for m in EXP4C_MODULES]
        print(f"  {regime:<14}" + "".join(vals))

    print(f"\n  Optimism rate (% where sim predicts better than reality):")
    print(header)
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = abl[abl["regime_label"] == regime]
        if sub.empty:
            continue
        vals = [f"{sub[sub['module_ablated']==m]['optimistic'].mean()*100:10.1f}%"
                for m in EXP4C_MODULES]
        print(f"  {regime:<14}" + "".join(vals))

    print(f"\n  Mean signed RMS bias (deg; positive = sim too optimistic):")
    print(header)
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = abl[abl["regime_label"] == regime]
        if sub.empty:
            continue
        vals = [f"{sub[sub['module_ablated']==m]['rms_bias'].mean():+10.2f}"
                for m in EXP4C_MODULES]
        print(f"  {regime:<14}" + "".join(vals))

    # Reference-condition oracle summary
    ref = df[df["module_ablated"] == "reference"]
    print(f"\n  Reference (oracle, tuned+evaluated at full reality):")
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        sub = ref[ref["regime_label"] == regime]
        if sub.empty:
            continue
        print(f"  {regime:<10}  mean_rms={sub['rms_reality'].mean():.2f} deg  "
              f"go_rate={sub['go_reality'].mean()*100:.1f}%")

    # ── Save ──────────────────────────────────────────────────────────────
    ctype_tag = ctype.lower()
    out_csv  = out_dir / f"exp4c_prediction_error_{ctype_tag}_py.csv"
    out_meta = out_dir / f"exp4c_prediction_error_{ctype_tag}_meta.txt"
    df.to_csv(out_csv, index=False)

    meta_lines = [
        f"Exp4C Prediction Error Study ({ctype}) — Metadata",
        "=" * 40,
        f"n_designs         : {n}",
        f"controller_type   : {ctype}",
        f"n_modules         : {len(EXP4C_MODULES)}",
        f"modules           : {EXP4C_MODULES}",
        f"n_seeds           : {EXP4C_N_SEEDS}",
        f"reality_config    : _exp4a_baseline_cfg() — 10 modules ON, thrust_var=OFF",
        f"motor             : F15 for all designs; constant 14.4 N when thrust_curve=OFF",
        f"gains             : tuned per module in ablated sim ({ctype} controller)",
        f"go_threshold      : FRAGILE_SUCCESS_RATE = {FRAGILE_SUCCESS_RATE}",
        f"output_format     : long — one row per (design x module), {len(flat_rows):,} rows",
        f"primary_metric    : rms_prediction_error = |rms_predicted - rms_reality|",
        f"secondary_metric  : decision_error = (go_predicted != go_reality)",
        f"rms_bias          : rms_reality - rms_predicted  (+ve = sim too optimistic)",
        f"reference_row     : module_ablated='reference'; tune+eval at full reality",
        f"approx_sims       : {n_total:,}",
        f"scientific_q      : Which modules cause prediction error when omitted from engineer's sim?",
        "",
        "Regime counts:",
    ]
    for regime in ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]:
        cnt = (designs_df.get("regime_label", pd.Series(dtype=str)) == regime).sum()
        meta_lines.append(f"  {regime:10s}: {cnt}")
    out_meta.write_text("\n".join(meta_lines))

    print(f"\nSaved: {out_csv}  ({len(df):,} rows)")
    print(f"Saved: {out_meta}")
    return df


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
    elif cmd == "exp4a":
        run_exp4a_ablation()
    elif cmd == "exp4b":
        run_exp4b_ablation()
    elif cmd == "exp4c":
        run_exp4c(controller_type="PD")
    elif cmd == "exp4c_pid":
        run_exp4c(controller_type="PID")
    elif cmd == "exp4simple":
        run_exp4simple()
    elif cmd == "exp5":
        run_exp5_landscape()
    elif cmd == "exp5slew":
        run_exp5_slew_stratified()
    elif cmd == "both":
        df = run_exp1(n_designs=n)
        run_exp4(df)
    elif cmd == "all":
        df = run_exp1(n_designs=n)
        run_exp4(df)
        run_exp4_ablation(df)
        run_exp5_landscape(df)
    else:
        print("Usage: python experiment_runner.py "
              "[exp1|exp4|exp4abl|exp4a|exp4b|exp4c|exp4simple|exp5|exp5slew|both|all] [n_designs]")
