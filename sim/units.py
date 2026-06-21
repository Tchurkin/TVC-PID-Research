"""
units.py — Unit conventions and physical constants for the TVC simulator.

Every module in this package adheres to the conventions below.
If you find a violation, it is a bug.

════════════════════════════════════════════════════════════════
INTERNAL UNITS (all simulation state and parameters)
════════════════════════════════════════════════════════════════
  Angle              radians (rad)
  Angular rate       radians per second (rad/s)
  Angular accel      rad/s²
  Time               seconds (s)
  Mass               kilograms (kg)
  Moment of inertia  kg·m²
  Force              N
  Moment             N·m
  Actuator command   code units  (dimensionless, see below)

OUTPUTS (human-readable, converted only at the point of output)
  Angle              degrees (deg)
  Angular rate       deg/s

════════════════════════════════════════════════════════════════
ACTUATOR COMMAND CONVENTION
════════════════════════════════════════════════════════════════
  Command u ∈ [-u_max, +u_max] where u_max = 12 code units by convention.
  Mapping to physical gimbal angle:
      gimbal_deg = u * (max_gimbal_deg / 12.0)
  The reference rocket uses max_gimbal_deg = 15.0, so:
      gimbal_deg = u * 1.25

  Control effectiveness keff [rad/s² / code unit] absorbs the full
  gimbal→angular-acceleration chain:
      keff = (F_thrust × L_tvc / Iyy) × (max_gimbal_deg / 12.0) × (π/180)
  where L_tvc is the moment arm from the nozzle gimbal to the rocket CG.
  At reference conditions, keff = 8.0 rad/s² / code unit.

  Slew rate conversion (physical deg/s → code units/s):
      slew_code = slew_deg_s × (REF_U_MAX / REF_MAX_GIMBAL_DEG)
               = slew_deg_s × (12 / 15)

  Derivation: physical angle per code unit is constant regardless of max_gimbal.
      θ_deg = u × max_gimbal_rad / u_max × (180/π)
            = u × (max_gimbal × π/180) / (12 × max_gimbal/15) × (180/π)
            = u × 15/12    ← independent of max_gimbal

  So: 1 CU = 15/12 = 1.25 degrees (fixed, for all designs).
      slew_code [CU/s] = slew_deg_s [deg/s] / 1.25 [deg/CU]
                       = slew_deg_s × 12/15 = slew_deg_s × 0.8

  A servo at 75 deg/s gives slew_max = 75 × 0.8 = 60 CU/s.
  The previous formula (with π/180) was a unit error that made slew_max
  57.3× too small. All slew-related results prior to this fix are invalid.

════════════════════════════════════════════════════════════════
SUCCESS GATES (all four must be satisfied simultaneously)
════════════════════════════════════════════════════════════════
  Source: default_rocket_params.m, P.analysis block.
  All thresholds are in degrees.

  MAX_THETA_DEG   < 70   — prevent structural / recovery failure
  END_ERROR_DEG   < 15   — end-state accuracy
  RMS_ERROR_DEG   < 15   — tracking quality over full burn
  PEAK_ERROR_DEG  < 50   — peak transient tolerance

════════════════════════════════════════════════════════════════
KNOWN MODELING APPROXIMATIONS (explicit disclosure)
════════════════════════════════════════════════════════════════
  1. damp_eff = aero_damp × sqrt(mass_scale)
     Physically, aerodynamic damping torque / Iyy should scale as 1/Iyy,
     not as sqrt(mass).  This approximation is retained for MATLAB parity
     but is not first-principles.  Flag for future derivation.

  2. estimate_p_unstable() contains calibration constants 10, 52, 35.
     These are not derived from the linearised EOM.  They match hobby-TVC
     data but are a known STS vulnerability.  See design_space.py.

  3. disturbance_eff scales as inertia_scale / mass_scale.
     A rigorous derivation for a wind-torque disturbance would use the full
     aerodynamic model.  The mass scaling here is an approximation.
"""

# ── Reference rocket (basis for all dimensionless scaling) ──────────────────
REF_MASS_KG       = 1.20    # kg
REF_IYY_KGM2      = 0.018   # kg·m²
REF_THRUST_N      = 35.0    # N
REF_KEFF          = 8.0     # rad/s² / code unit
REF_AERO_DAMP     = 0.5     # 1/s
REF_MAX_GIMBAL_DEG = 15.0   # deg
REF_U_MAX         = 12.0    # code units (convention)

# ── Success gates (degrees) ──────────────────────────────────────────────────
SUCCESS_MAX_THETA_DEG  = 70.0
SUCCESS_END_ERROR_DEG  = 15.0
SUCCESS_RMS_ERROR_DEG  = 15.0
SUCCESS_PEAK_ERROR_DEG = 50.0

# ── Regime classification thresholds ────────────────────────────────────────
EASY_SUCCESS_RATE      = 0.80   # nominal success rate threshold for EASY
EASY_ROBUSTNESS        = 1.00   # fraction of gain sets that must pass
EASY_U_SAT_FRAC        = 0.60   # max allowed saturation fraction
EASY_SLEW_SAT_FRAC     = 1.00   # (not binding by default)
EASY_RMS_DEG           = 8.0    # max RMS error for EASY
EASY_SETTLING_S        = 3.0    # max settling time for EASY
EASY_OSC_SCORE         = 3.0    # max zero-crossing count for EASY
FRAGILE_SUCCESS_RATE   = 0.35   # minimum success rate to avoid INFEASIBLE (nominal gate)
FRAGILE_RMS_DEG        = 16.0   # max RMS error for FRAGILE
# Robustness gate for over/under tests: 3-seed mean must reach this threshold at
# ±40% Kp/Kd to be considered "robust".  With 3 binary seeds, SR ∈ {0, 0.333, 0.667, 1.0}.
# Setting 0.80 means only SR=1.0 (all 3 seeds pass) counts as robust — equivalent to
# "any seed failing at ±40% Kp flags as FRAGILE".  More sensitive than the old 1-seed
# test (36% detection at true p_fail=0.14 vs 14% with 1 seed).
ROBUSTNESS_SUCCESS_RATE = 0.80

# ── Divergence and progressive tracking bands (degrees) ─────────────────────
# Primary binary: did the rocket diverge? (theta >= DIVERGE_THETA_DEG)
# Progressive bands: how tight was the attitude control?
# Used to stratify designs and draw "separation lines" in visualisations.
DIVERGE_THETA_DEG  = 90.0   # absolute divergence — rocket out of control
BAND_30_THETA_DEG  = 30.0   # loose band — barely maintained attitude
BAND_20_THETA_DEG  = 20.0   # moderate tracking
BAND_10_THETA_DEG  = 10.0   # tight tracking — good TVC performance

# ── F15 motor reference values ───────────────────────────────────────────────
# Used as the constant-thrust fallback when thrust_curve=OFF.
# All designs share the same physical motor; the design-space "thrust" parameter
# is a proxy for control effectiveness, not actual motor output.
F15_AVG_THRUST_N = 14.4  # average thrust of F15 APCP motor (N), matches MotorParams.F15().T_avg
