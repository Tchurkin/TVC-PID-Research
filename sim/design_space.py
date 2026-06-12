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
  slew_max [code units/s] = slew_deg_s × (REF_U_MAX / REF_MAX_GIMBAL_DEG) = slew_deg_s × 0.8
  This is independent of max_gimbal_deg (1 code unit = 1.25 physical degrees, fixed).
  See units.py for the full derivation. Note: the old formula included a π/180 factor
  (WRONG — was fixed 2026-06-05; made servos 57.3× too slow).
"""

from __future__ import annotations

import numpy as np
import pandas as pd
from scipy.stats import qmc

from units import (
    REF_MASS_KG, REF_IYY_KGM2, REF_THRUST_N, REF_KEFF, REF_AERO_DAMP,
    REF_MAX_GIMBAL_DEG, REF_U_MAX, F15_AVG_THRUST_N,
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
    motor_scale=1.0,           # replaces thrust; scale=1 → standard F15 (14.4 N avg)
    servo_slew_deg_s=120.0,
    max_gimbal_deg=REF_MAX_GIMBAL_DEG,
    deadband=0.05,
    backlash=0.10,
    latency_steps=3,
    wind_strength=0.15,
)


# ── Design space bounds (match run_exp1_stability_authority_frontier.m) ───────

DESIGN_NAMES = [
    "mass", "Iyy", "static_margin", "Cm_alpha", "control_effectiveness",
    "motor_scale", "servo_slew_deg_s", "max_gimbal_deg", "latency_steps",
    "deadband", "backlash", "wind_strength",
]
# motor_scale replaces thrust: scales the F15 motor linearly in thrust and mass.
#   0.5 → half-F15 (~7.2 N avg, wet 50 g)   — marginal lift for lightest designs
#   1.0 → standard F15 (14.4 N avg, wet 100 g)
#   3.0 → triple-F15 (43.2 N avg, wet 300 g) — approaching G-class territory
# max_gimbal_deg floor lowered to 2 deg to create genuinely authority-limited designs
# servo_slew_deg_s: [60, 200] deg/s — realistic hobby servo range (40 deg/s is the reliable floor)
# static_margin: [-0.30, +0.30] — signed; negative = stable (fins, CP aft), positive = unstable
# Cm_alpha extended to [-90, -15] — wider aerodynamic instability range
# static_margin is now SIGNED:
#   > 0: CP forward of CG → aerodynamically UNSTABLE (TVC must actively stabilise)
#   = 0: neutrally stable
#   < 0: CP aft of CG   → aerodynamically STABLE   (fins provide passive stability)
# Range [-0.30, +0.30] calibers: covers stable finned rockets to very unstable finless.
# Convention matches Barrowman theory (positive stability margin → CP aft → stable).
DESIGN_LO = np.array([
    0.90, 0.010, -0.30, -90.0,  5.0, 0.50,  60.0,  2.0, 1.0, 0.00, 0.00, 0.05
])
DESIGN_HI = np.array([
    2.10, 0.040,  0.30, -15.0, 14.0, 3.00, 200.0, 15.0, 6.0, 0.15, 0.25, 0.45
])


# ── p_unstable proxy ──────────────────────────────────────────────────────────

def estimate_lambda_aero(
    static_margin: float,
    Cm_alpha: float,
    motor_scale: float = 1.0,
) -> float:
    """
    Signed aerodynamic stiffness (rad/s²/rad) for the simple-mode EOM.

    EOM term: lambda_aero × theta
      lambda_aero > 0: destabilising (CP forward, TVC rocket without fins)
      lambda_aero = 0: neutral
      lambda_aero < 0: restoring  (CP aft, finned stable rocket)

    Magnitude formula (calibrated, not first-principles):
      p_or_wn = 10 × |static_margin| × |Cm_alpha| / 52 × sqrt(35 / T_eff)
    Then:
      unstable (static_margin > 0): lambda_aero = +p²
      stable   (static_margin < 0): lambda_aero = −ω_n²
      neutral  (static_margin = 0): lambda_aero = 0

    Calibration constants (10, 52, 35) are NOT derived from first principles.
    Known STS vulnerability — see CLAUDE.md.  p_unstable proxy also has near-zero
    correlation with regime under full-physics evaluation; Iyy and wind_strength
    are the dominant regime predictors.
    """
    effective_thrust = F15_AVG_THRUST_N * motor_scale
    magnitude = (
        10.0 * abs(static_margin) * abs(Cm_alpha) / 52.0
        * np.sqrt(35.0 / max(1e-6, effective_thrust))
    )
    if static_margin > 0.0:
        return magnitude ** 2    # unstable: positive = diverging
    elif static_margin < 0.0:
        return -(magnitude ** 2) # stable: negative = restoring
    else:
        return 0.0               # neutral


def estimate_p_unstable(
    static_margin: float,
    Cm_alpha: float,
    motor_scale: float = 1.0,
) -> float:
    """Backward-compat alias: instability rate (1/s) ≥ 0. Zero for stable designs."""
    lam = estimate_lambda_aero(static_margin, Cm_alpha, motor_scale)
    return float(np.sqrt(max(0.0, lam)))


# ── LHS sampler ───────────────────────────────────────────────────────────────

def sample_lhs(n: int, seed: int = 42) -> pd.DataFrame:
    """
    Latin Hypercube sample of the 12-D design space.
    Adds derived columns: p_unstable, design_id, rocket_id, tw_ratio.

    Feasibility filter: designs with T/W < 1 (motor thrust insufficient to lift the
    rocket against gravity) are excluded and resampled until n feasible designs are
    obtained.  These designs are physically unrealizable — the simulator assigns them
    near-zero aerodynamic forces, making them appear trivially controllable (EASY)
    even though they cannot launch.
    """
    rng_seed = seed
    collected: list[np.ndarray] = []
    target = n
    # Oversample and filter until we have enough feasible designs.
    while len(collected) < target:
        sampler = qmc.LatinHypercube(d=len(DESIGN_NAMES), seed=rng_seed)
        batch_size = max(target * 2, 500)
        X = qmc.scale(sampler.random(batch_size), DESIGN_LO, DESIGN_HI)
        # T/W feasibility: F15_avg * motor_scale > mass * g
        motor_scale_col = DESIGN_NAMES.index("motor_scale")
        mass_col        = DESIGN_NAMES.index("mass")
        T_eff = F15_AVG_THRUST_N * X[:, motor_scale_col]
        W     = X[:, mass_col] * 9.81
        feasible = T_eff > W
        collected.extend(X[feasible])
        rng_seed += 1  # shift seed on retry so we don't resample identical points

    X_out = np.array(collected[:target])
    df = pd.DataFrame(X_out, columns=DESIGN_NAMES)
    df["latency_steps"] = df["latency_steps"].round().astype(int).clip(1, 6)

    df["lambda_aero"] = df.apply(
        lambda r: estimate_lambda_aero(r["static_margin"], r["Cm_alpha"], r["motor_scale"]),
        axis=1,
    )
    df["p_unstable"] = df["lambda_aero"].apply(lambda lam: float(np.sqrt(max(0.0, lam))))
    df["aerodynamically_stable"] = df["static_margin"] < 0.0
    df["tw_ratio"] = (F15_AVG_THRUST_N * df["motor_scale"]) / (df["mass"] * 9.81)
    df["design_id"] = np.arange(1, n + 1)
    df["rocket_id"] = [f"R{i:04d}" for i in range(1, n + 1)]
    return df


# ── Config builders ───────────────────────────────────────────────────────────

def build_plant(design: dict) -> PlantParams:
    """Build PlantParams from a design row dict.

    Simple-mode fields (p_unstable, scales) are design-specific.
    High-fidelity fields (Cm_alpha_signed, static_margin, Iyy_kgm2, m_kg,
    max_gimbal_deg, motor_scale) are design-specific so full-physics mode
    models the actual design.
    l_nozzle_m is fixed at 0.25 m (all designs share 0.5 m airframe).
    """
    mass        = design.get("mass",    REF["mass"])
    Iyy         = design.get("Iyy",     REF["Iyy"])
    motor_scale = design.get("motor_scale", REF["motor_scale"])
    sm          = design.get("static_margin", REF["static_margin"])
    Cma         = design.get("Cm_alpha",       REF["Cm_alpha"])
    keff        = design.get("control_effectiveness", REF["control_effectiveness"])
    lam_aero    = design.get("lambda_aero", estimate_lambda_aero(sm, Cma, motor_scale))
    p_unst      = float(np.sqrt(max(0.0, lam_aero)))
    mg_deg      = design.get("max_gimbal_deg", REF["max_gimbal_deg"])

    effective_thrust = F15_AVG_THRUST_N * motor_scale

    return PlantParams(
        p_unstable    = float(p_unst),
        lambda_aero   = float(lam_aero),
        control_eff   = float(keff),
        aero_damp     = REF_AERO_DAMP,
        mass_scale    = float(np.clip(mass  / REF_MASS_KG,  0.30, 3.00)),
        inertia_scale = float(np.clip(REF_IYY_KGM2 / max(1e-9, Iyy), 0.20, 5.00)),
        thrust_scale  = float(np.clip(effective_thrust / REF_THRUST_N, 0.20, 5.00)),
        dt            = 0.005,
        # High-fidelity fields — design-specific so full-physics mode is correct
        Cm_alpha_signed = float(Cma),
        static_margin   = float(sm),
        Iyy_kgm2        = float(Iyy),
        m_kg            = float(mass),
        max_gimbal_deg  = float(mg_deg),
        l_nozzle_m      = 0.25,   # fixed: all designs share 0.5 m airframe
        motor_scale     = float(motor_scale),
    )


def build_actuator(design: dict) -> ActuatorParams:
    """Build ActuatorParams from a design row dict."""
    max_gimbal   = max(1e-3, design.get("max_gimbal_deg", REF["max_gimbal_deg"]))
    slew_deg_s   = design.get("servo_slew_deg_s", REF["servo_slew_deg_s"])
    deadband     = design.get("deadband", REF["deadband"])
    backlash     = design.get("backlash",  REF["backlash"])

    u_max    = REF_U_MAX * max_gimbal / REF_MAX_GIMBAL_DEG
    slew_max = slew_deg_s * (REF_U_MAX / REF_MAX_GIMBAL_DEG)  # = slew_deg_s × 12/15

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


def build_scenario(
    theta0_bias_std: float = 0.0,
    theta0_fixed_deg: float = 0.0,
) -> ScenarioParams:
    return ScenarioParams(
        t_end             = 3.0,
        theta_ref         = 0.0,
        theta0_bias_std   = theta0_bias_std,
        theta0_fixed_deg  = theta0_fixed_deg,
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
      EASY       (2) — robust to all gain conditions AND meets success-rate/RMS thresholds
      MARGINAL   (3) — robust to all gain conditions BUT nom_sr < EASY threshold;
                       wind-sensitive (fails some wind seeds) but not gain-sensitive
      FRAGILE    (1) — fails at least one gain robustness condition; gain-sensitive
      INFEASIBLE (0) — cannot be stabilized (fails even nominal evaluation)

    oscillation_score is intentionally excluded from the EASY gate. After the slew-model
    fix, fast-servo designs get high-Kp autotune solutions that produce many zero-crossings
    but low RMS and perfect success rate — classifying them MARGINAL on osc_score alone
    was a tuning artifact, not a physical controllability property.

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
    )
    frag_q = (
        nominal_success_rate >= FRAGILE_SUCCESS_RATE
        and nominal_rms_deg  <= FRAGILE_RMS_DEG
    )

    if robustness >= EASY_ROBUSTNESS and easy_q:
        return "EASY", 2
    elif robustness >= EASY_ROBUSTNESS and frag_q:
        # Passes all gain conditions but fails the EASY success-rate threshold.
        # Wind-sensitive: some wind seeds fail, but not gain-sensitive.
        return "MARGINAL", 3
    elif robustness > 0.0 and frag_q:
        return "FRAGILE", 1
    else:
        return "INFEASIBLE", 0
