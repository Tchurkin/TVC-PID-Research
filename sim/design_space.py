"""
design_space.py — Design space sampling, parameter mapping, and regime classification.

Responsibilities
────────────────
  1. estimate_p_unstable()   proxy instability rate from physical design variables
  2. sample_lhs()            Latin Hypercube sample of the 12-D design space
  3. build_*()               convert a design row to simulator parameter structs
  4. classify_regime()       EASY / FRAGILE / INFEASIBLE from evaluation metrics

p_unstable derivation (and known limitations)
─────────────────────────────────────────────
  Physically, the open-loop pitch instability arises from aerodynamic and
  gravitational moments:

      Iyy · θ̈  ≈  [Cm_alpha · q_dyn · S_ref · L_ref] · θ  +  control + dist

  For an aerodynamically unstable rocket (CP forward of CG, Cm_alpha < 0 with
  our convention), the bracketed term is positive, driving unstable growth.
  The natural instability frequency is:
      p  =  sqrt( |Cm_alpha| · q_dyn · S_ref · L_ref / Iyy )

  Dynamic pressure q_dyn ∝ ρV² ∝ thrust (for constant drag coefficient), so
  p ∝ sqrt( |Cm_alpha| · thrust ).  The static_margin calibrates the reference
  geometry to the specific rocket's aerodynamic configuration.

  The implemented formula is a dimensionless proxy calibrated to hobby-TVC data:
      p  =  10 · static_margin · |Cm_alpha| / 52 · sqrt(35 / thrust)
  Constants (10, 52, 35) are calibration values, NOT derived from first principles.
  This is a documented STS vulnerability that should be replaced with a full
  linearised EOM derivation.  See design_space.py TODO.

  Sign convention: static_margin ∈ [0.04, 0.24] calibers; Cm_alpha ∈ [-70, -30].
  Positive static_margin with negative Cm_alpha gives p_unstable > 0 for all
  sampled designs — all rockets in the study are open-loop unstable.

Command unit convention
───────────────────────
  u_max = 12 code units = max_gimbal_deg physical deflection (reference: 15 deg).
  slew_max [code units/s] = slew_deg_s · (π/180) · (12 / max_gimbal_deg)
  See units.py for full explanation and the known (π/180) factor discussion.
"""

from __future__ import annotations

import numpy as np
import pandas as pd
from scipy.stats import qmc

from units import (
    REF_MASS_KG, REF_IYY_KGM2, REF_THRUST_N, REF_KEFF, REF_AERO_DAMP,
    REF_MAX_GIMBAL_DEG, REF_U_MAX,
    EASY_SUCCESS_RATE, EASY_ROBUSTNESS, EASY_U_SAT_FRAC, EASY_SLEW_SAT_FRAC,
    EASY_RMS_DEG, EASY_SETTLING_S, EASY_OSC_SCORE,
    FRAGILE_SUCCESS_RATE, FRAGILE_RMS_DEG,
)
from plant_dynamics import PlantParams
from actuator_model import ActuatorParams
from sensor_model import SensorParams
from disturbance_model import DisturbanceParams
from controller import PIDParams
from simulator import ScenarioParams


# ── Reference rocket ──────────────────────────────────────────────────────────

REF = dict(
    mass=REF_MASS_KG,
    Iyy=REF_IYY_KGM2,
    static_margin=0.10,
    Cm_alpha=-52.0,
    control_effectiveness=REF_KEFF,
    thrust=REF_THRUST_N,
    servo_slew_deg_s=75.0,
    max_gimbal_deg=REF_MAX_GIMBAL_DEG,
    deadband=0.05,
    backlash=0.10,
    latency_steps=3,
    wind_strength=0.15,
)


# ── Design space bounds (match run_exp1_stability_authority_frontier.m) ───────

DESIGN_NAMES = [
    "mass", "Iyy", "static_margin", "Cm_alpha", "control_effectiveness",
    "thrust", "servo_slew_deg_s", "max_gimbal_deg", "latency_steps",
    "deadband", "backlash", "wind_strength",
]
DESIGN_LO = np.array([
    0.90, 0.010,  0.04, -70.0,  5.0, 25.0,  20.0,  8.0, 1.0, 0.00, 0.00, 0.05
])
DESIGN_HI = np.array([
    2.10, 0.040,  0.24, -30.0, 14.0, 50.0, 120.0, 15.0, 6.0, 0.15, 0.25, 0.45
])


# ── p_unstable proxy ──────────────────────────────────────────────────────────

def estimate_p_unstable(
    static_margin: float,
    Cm_alpha: float,
    thrust: float,
) -> float:
    """
    Proxy open-loop instability rate (1/s) from design variables.
    Formula matches MATLAB run_exp1_stability_authority_frontier.m.

    Derivation sketch:
      The linearised pitch EOM for a rocket with CG-CP offset is:
        Iyy * θ̈ ≈ M_aero * θ + M_control + M_disturbance
      where M_aero = Cm_alpha * q_dyn * S_ref * L_ref.
      For the unstable case (Cm_alpha < 0 with our sign convention, positive
      moment for positive θ), the open-loop instability rate is:
        p = sqrt(|M_aero| / Iyy)
          ∝ sqrt(|Cm_alpha| * q_dyn)
          ∝ sqrt(|Cm_alpha| * thrust)   [q_dyn ~ thrust at low speed]
      static_margin calibrates the effective aerodynamic moment arm.
      Iyy is fixed at reference (REF_IYY_KGM2) for this proxy formula.

    Known limitation: constants (10, 52, 35) are calibration values fit to
    hobby-TVC data, not derived from first principles.  The reference values
    (Cm_alpha_ref = -52, thrust_ref = 35 N) set the scale.  static_margin
    scales the aero moment arm.  10 is a dimensionless calibration factor.
    This is a documented STS vulnerability — see CLAUDE.md.
    """
    # p = scale * static_margin * |Cm_alpha| / Cm_alpha_ref * sqrt(thrust_ref / thrust)
    # scale=10, Cm_alpha_ref=52, thrust_ref=35 N — calibrated to match MATLAB reference
    return max(
        0.0,
        10.0 * static_margin * abs(Cm_alpha) / 52.0 * np.sqrt(35.0 / max(1e-6, thrust))
    )


# ── LHS sampler ───────────────────────────────────────────────────────────────

def sample_lhs(n: int, seed: int = 42) -> pd.DataFrame:
    """
    Latin Hypercube sample of the 12-D design space.
    Adds derived columns: p_unstable, design_id, rocket_id.
    """
    sampler = qmc.LatinHypercube(d=len(DESIGN_NAMES), seed=seed)
    X = qmc.scale(sampler.random(n), DESIGN_LO, DESIGN_HI)

    df = pd.DataFrame(X, columns=DESIGN_NAMES)
    df["latency_steps"] = df["latency_steps"].round().astype(int).clip(1, 6)

    df["p_unstable"] = df.apply(
        lambda r: estimate_p_unstable(r["static_margin"], r["Cm_alpha"], r["thrust"]),
        axis=1,
    )
    df["design_id"] = np.arange(1, n + 1)
    df["rocket_id"] = [f"R{i:04d}" for i in range(1, n + 1)]
    return df


# ── Config builders ───────────────────────────────────────────────────────────

def build_plant(design: dict) -> PlantParams:
    """Build PlantParams from a design row dict."""
    mass    = design.get("mass",    REF["mass"])
    Iyy     = design.get("Iyy",     REF["Iyy"])
    thrust  = design.get("thrust",  REF["thrust"])
    sm      = design.get("static_margin", REF["static_margin"])
    Cma     = design.get("Cm_alpha",       REF["Cm_alpha"])
    keff    = design.get("control_effectiveness", REF["control_effectiveness"])
    p_unst  = design.get("p_unstable", estimate_p_unstable(sm, Cma, thrust))

    return PlantParams(
        p_unstable    = float(p_unst),
        control_eff   = float(keff),
        aero_damp     = REF_AERO_DAMP,
        mass_scale    = float(np.clip(mass  / REF_MASS_KG,  0.30, 3.00)),
        inertia_scale = float(np.clip(REF_IYY_KGM2 / max(1e-9, Iyy), 0.20, 5.00)),
        thrust_scale  = float(np.clip(thrust / REF_THRUST_N, 0.20, 5.00)),
        dt            = 0.005,
    )


def build_actuator(design: dict) -> ActuatorParams:
    """Build ActuatorParams from a design row dict."""
    max_gimbal   = max(1e-3, design.get("max_gimbal_deg", REF["max_gimbal_deg"]))
    slew_deg_s   = design.get("servo_slew_deg_s", REF["servo_slew_deg_s"])
    deadband     = design.get("deadband", REF["deadband"])
    backlash     = design.get("backlash",  REF["backlash"])

    u_max    = REF_U_MAX * max_gimbal / REF_MAX_GIMBAL_DEG
    slew_max = slew_deg_s * (np.pi / 180.0) * (REF_U_MAX / max_gimbal)

    return ActuatorParams(
        u_max    = float(u_max),
        slew_max = float(slew_max),
        tau_act  = 0.05,
        deadband = float(deadband),
        backlash = float(backlash),
    )


def build_sensor(design: dict) -> SensorParams:
    """Build SensorParams from a design row dict."""
    return SensorParams(
        gyro_noise_std      = 0.015,
        gyro_bias_init      = 0.010,
        gyro_bias_rw        = 0.005,
        gyro_quant_lsb      = 2.0 * np.pi / 4000.0,
        bias_cal_residual   = 0.10,
        sensor_latency_steps = int(round(design.get("latency_steps", REF["latency_steps"]))),
    )


def build_disturbance(design: dict) -> DisturbanceParams:
    """Build DisturbanceParams from a design row dict."""
    return DisturbanceParams(
        det_amp     = 0.05,
        det_freq_hz = 0.80,
        gust_std    = design.get("wind_strength", REF["wind_strength"]),
        gust_tau    = 0.40,
    )


def build_scenario(theta0_bias_std: float = 0.0) -> ScenarioParams:
    return ScenarioParams(
        t_end             = 3.0,
        theta_ref         = 0.0,
        theta0_bias_std   = theta0_bias_std,
        fault_time_s      = float("inf"),
    )


# ── Regime classification ─────────────────────────────────────────────────────

def classify_regime(
    nominal_success_rate: float,
    nominal_rms_deg: float,
    nominal_u_sat_frac: float,
    nominal_slew_sat_frac: float,
    nominal_settling_s: float,
    nominal_osc_score: float,
    robustness: float,
) -> tuple[str, int]:
    """
    Classify a design as EASY (2), MARGINAL (3), FRAGILE (1), or INFEASIBLE (0).

    Four-class scheme:
      EASY     (2) — robust to all gain conditions AND meets quality thresholds
      MARGINAL (3) — robust to all gain conditions BUT high steady-state RMS;
                     stable and reliable, just a poor tracker (not wind-sensitive)
      FRAGILE  (1) — fails at least one gain condition; genuinely wind-sensitive
      INFEASIBLE(0) — fails even nominal or extreme performance failure

    robustness = fraction of {nominal, under, over} gain sets that pass the
    FRAGILE_SUCCESS_RATE threshold.

    Returns (label, code) where code ∈ {0, 1, 2, 3}.
    """
    easy_q = (
        nominal_success_rate  >= EASY_SUCCESS_RATE
        and nominal_u_sat_frac    <= EASY_U_SAT_FRAC
        and nominal_slew_sat_frac <= EASY_SLEW_SAT_FRAC
        and nominal_rms_deg       <= EASY_RMS_DEG
        and nominal_settling_s    <= EASY_SETTLING_S
        and nominal_osc_score     <= EASY_OSC_SCORE
    )
    frag_q = (
        nominal_success_rate >= FRAGILE_SUCCESS_RATE
        and nominal_rms_deg  <= FRAGILE_RMS_DEG
    )

    if robustness >= EASY_ROBUSTNESS and easy_q:
        return "EASY", 2
    elif robustness >= EASY_ROBUSTNESS and frag_q:
        # Passes all gain conditions (not wind-sensitive) but fails EASY quality gate.
        # High steady-state RMS — a tracking quality limitation, not physical fragility.
        return "MARGINAL", 3
    elif robustness > 0.0 and frag_q:
        return "FRAGILE", 1
    else:
        return "INFEASIBLE", 0
