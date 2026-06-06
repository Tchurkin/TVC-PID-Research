"""
plant_dynamics.py — Pitch dynamics for a TVC rocket.

Two operating modes controlled by FidelityConfig flags:

  SIMPLE (all new flags off) — original linearised 1-DOF model.
    Identical to the pre-2026-06-04 implementation.  Used as baseline for
    backward compatibility with Exp1/Exp4 results.

  HIGH-FIDELITY (new flags on) — physically derived nonlinear model.
    Removes small-angle approximation.
    Uses dynamic aerodynamics (q_dyn varies with flight speed).
    Optionally uses realistic thrust curve and CG/Iyy evolution.

─────────────────────────────────────────────────────────────────────────────
PHYSICAL EQUATIONS (high-fidelity mode)
─────────────────────────────────────────────────────────────────────────────

Pitch EOM (moment equation about CG, free flight):

    I_yy(t) × θ̈ = M_TVC + M_aero + M_damp

where:

  M_TVC  = T(t) × sin(u_gimbal_rad) × l_nozzle(t)
          ≈ T(t) × u_act × (max_gimbal_rad / u_max) × l_nozzle(t)
          [TVC control moment; positive u_act → nose-right moment]

  M_aero = q_dyn(t) × S_ref × D_ref × Cm_alpha × sin(θ)
          [aerodynamic pitching moment; Cm_alpha > 0 for unstable rocket]
          [sin(θ) replaces θ — removes small-angle approximation]

  M_damp = q_dyn(t) × S_ref × D_ref × Cmq × (θ̇ × D_ref) / (2 × v(t))
          [pitch damping from pitch-rate induced angle of attack distribution]
          [scales as v, not v² — important at low speeds early in flight]

Note on gravity:
  For a FREE-FLYING rocket, gravity acts through the CG and produces ZERO
  pitching moment about the CG.  The "gravitational pendulum" analogy applies
  only to rockets on a fixed pivot (launch rail).  In free flight, gravity
  only affects translation (dv/dt = T/m - g - D/m), not rotation.
  Gravity's effect on pitch dynamics is INDIRECT: it reduces flight speed
  (and hence dynamic pressure and aerodynamic moments) compared to a
  zero-gravity trajectory.

─────────────────────────────────────────────────────────────────────────────
SIMPLE MODE EQUATIONS (legacy compatibility)
─────────────────────────────────────────────────────────────────────────────

    θ̈ = keff_phys × u_act  −  damp_eff × θ̇  +  p² × θ  +  d(t)

  keff_phys  = control_eff × keff_fault × thrust_scale × inertia_scale
  damp_eff   = aero_damp × damp_fault × sqrt(mass_scale)   [approximation]
  p_unstable = design parameter (calibrated proxy for instability rate)

  These are constant throughout the simulation.  Linear in θ.
  Valid for |θ| < ~15 deg.  Used as the simple/baseline fidelity level.

─────────────────────────────────────────────────────────────────────────────
INTEGRATION
─────────────────────────────────────────────────────────────────────────────

  Semi-implicit Euler:
      q_new   = q   + dt × θ̈(θ, q, u_act, t)
      θ_new   = θ   + dt × q_new     ← uses updated q (semi-implicit)

  This is equivalent to the 1-step Adams-Bashforth/Moulton predictor
  for a second-order system.  More stable than forward Euler for
  oscillatory/unstable systems.  First-order accurate in dt.
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from units import REF_KEFF, REF_AERO_DAMP, REF_MAX_GIMBAL_DEG, REF_U_MAX


@dataclass
class PlantParams:
    """
    All plant parameters.

    Simple-mode fields (always used when high-fidelity flags are off):
      p_unstable      proxy instability rate (1/s) — from design_space.estimate_p_unstable()
      control_eff     nominal keff (rad/s² / code unit) — TVC authority at reference conditions
      aero_damp       baseline aerodynamic damping (1/s) — constant approximation
      mass_scale      m / m_ref        (clamped [0.30, 3.00])
      inertia_scale   Iyy_ref / Iyy    (clamped [0.20, 5.00])
      thrust_scale    T / T_ref        (clamped [0.20, 5.00])
      dt              integration timestep (s)

    High-fidelity mode fields (used when new fidelity flags are on):
      Cm_alpha_signed raw design Cm_alpha value (negative for unstable, ∈ [−70, −30])
      static_margin   CP-CG separation in calibers
      max_gimbal_deg  physical gimbal travel limit (deg)
      l_nozzle_m      nozzle-to-CG moment arm (m); derived from rocket length
      Iyy_kgm2        actual moment of inertia (kg·m²) — used directly in full mode
      m_kg            vehicle dry mass (kg)
    """
    # Simple-mode parameters (always required)
    p_unstable:    float               # open-loop instability rate (1/s), ≥ 0 (legacy, kept for compatibility)
    lambda_aero:   float = 0.0        # signed aerodynamic stiffness (rad/s²/rad):
                                       #   > 0: unstable (= p_unstable²), diverging
                                       #   = 0: neutrally stable
                                       #   < 0: stable (= −ω_n²), restoring
    control_eff:   float = REF_KEFF    # nominal keff (rad/s² / code unit)
    aero_damp:     float = REF_AERO_DAMP

    mass_scale:    float = 1.0         # m / m_ref,       clamped [0.30, 3.00]
    inertia_scale: float = 1.0         # Iyy_ref / Iyy,   clamped [0.20, 5.00]
    thrust_scale:  float = 1.0         # T / T_ref,        clamped [0.20, 5.00]

    dt:            float = 0.005       # integration timestep (s)

    # High-fidelity mode parameters (needed for physical aerodynamic model)
    Cm_alpha_signed: float = -52.0     # design space Cm_alpha (negative for unstable)
    static_margin:   float = 0.10      # (x_CG - x_CP) / D_ref (calibers)
    max_gimbal_deg:  float = REF_MAX_GIMBAL_DEG
    l_nozzle_m:      float = 0.25      # moment arm from nozzle to CG (m), ≈ 50% of L
    Iyy_kgm2:        float = 0.018     # actual Iyy (kg·m²)
    m_kg:            float = 1.20      # dry vehicle mass (kg)
    motor_scale:     float = 1.0       # F15 motor scale (1.0 = standard F15)


def simple_effective_params(
    plant: PlantParams,
    keff_fault: float = 1.0,
    damp_fault: float = 1.0,
) -> tuple[float, float]:
    """
    Compute runtime keff_phys and damp_eff for SIMPLE mode.

    Returns
    -------
    (keff_phys, damp_eff) — effective control authority and aerodynamic damping
    """
    keff_phys = (
        plant.control_eff
        * keff_fault
        * plant.thrust_scale
        * plant.inertia_scale
    )
    damp_eff = (
        plant.aero_damp
        * damp_fault
        * np.sqrt(max(0.2, plant.mass_scale))
        # NOTE: sqrt(mass_scale) is an approximation for MATLAB parity.
        # Physically, aero damping should scale as q_dyn × l² / I_yy.
        # See units.py disclosure #1.  This is exact only at mass_scale = 1.
    )
    return keff_phys, damp_eff


def step_plant_simple(
    theta: float,
    q: float,
    u_act: float,
    disturbance: float,
    plant: PlantParams,
    keff_fault: float = 1.0,
    damp_fault: float = 1.0,
) -> tuple[float, float]:
    """
    Simple (legacy) plant step: linearised 1-DOF, constant coefficients.

    Valid for small angles (|theta| < 15 deg).  Identical to the pre-2026-06-04
    implementation.  Used when high-fidelity flags (nonlinear_aero, dyn_aero,
    thrust_curve, cg_shift) are all off.

    EOM: theta_ddot = keff_phys × u_act - damp_eff × q + lambda_aero × theta + d(t)
    where lambda_aero > 0 = unstable (diverging), lambda_aero < 0 = stable (restoring).
    """
    keff_phys, damp_eff = simple_effective_params(plant, keff_fault, damp_fault)

    qdot = (
        keff_phys * u_act
        - damp_eff * q
        + plant.lambda_aero * theta     # signed: + diverges, − restores
        + disturbance
    )

    q_new     = q     + plant.dt * qdot
    theta_new = theta + plant.dt * q_new   # semi-implicit
    return theta_new, q_new


def step_plant_full(
    theta: float,
    q: float,
    u_act: float,
    disturbance: float,
    plant: PlantParams,
    aero_params,           # AeroParams from aero_model.py
    q_dyn: float,          # current dynamic pressure (Pa)
    v: float,              # current flight speed (m/s)
    T_now: float,          # current thrust (N)
    I_yy_now: float,       # current moment of inertia (kg·m²)
    nonlinear_aero: bool = True,
    keff_fault: float = 1.0,
    damp_fault: float = 1.0,
) -> tuple[float, float]:
    """
    Full-physics plant step: nonlinear EOM with dynamic aerodynamics.

    Removes small-angle approximation (uses sin(theta) in aero moment).
    Uses time-varying q_dyn, T, and I_yy.
    Aerodynamic damping scales correctly with q_dyn and v.

    EOM:
        I_yy(t) × theta_ddot = M_TVC + M_aero + M_damp + I_yy × d(t) / keff_phys
        ... + disturbance added directly as angular acceleration

    M_TVC  = T(t) × sin(u_gimbal) × l_nozzle  ← TVC moment
    M_aero = q_dyn × S_ref × D_ref × Cm_alpha × sin(theta)  ← destabilising aero
    M_damp = q_dyn × S_ref × D_ref × Cmq × (q × D_ref / (2v))  ← pitch damping

    Parameters
    ----------
    theta          : pitch angle (rad)
    q              : pitch rate (rad/s)
    u_act          : actuator output (code units) — physical gimbal angle in code units
    disturbance    : external angular acceleration (rad/s²)
    plant          : plant parameters (geometry, gains)
    aero_params    : AeroParams from aero_model.py
    q_dyn          : current dynamic pressure 0.5*rho*v² (Pa)
    v              : current flight speed (m/s)
    T_now          : current thrust (N)
    I_yy_now       : current moment of inertia about CG (kg·m²)
    nonlinear_aero : if True, use sin(theta) in aero moment; else use theta
    keff_fault     : multiplicative fault on control effectiveness
    damp_fault     : multiplicative fault on aerodynamic damping

    Returns
    -------
    (theta_new, q_new)
    """
    from aero_model import aero_pitching_moment, aero_pitch_damping_moment

    # ── TVC control moment ────────────────────────────────────────────────────
    # Convert code units to physical gimbal angle (radians)
    u_max_code = plant.max_gimbal_deg / REF_MAX_GIMBAL_DEG * REF_U_MAX
    gimbal_rad = u_act * (plant.max_gimbal_deg * np.pi / 180.0) / u_max_code
    M_TVC = T_now * keff_fault * np.sin(gimbal_rad) * plant.l_nozzle_m

    # ── Aerodynamic pitching moment ───────────────────────────────────────────
    M_aero = aero_pitching_moment(aero_params, q_dyn, theta, nonlinear=nonlinear_aero)

    # ── Pitch damping moment ──────────────────────────────────────────────────
    M_damp = (aero_pitch_damping_moment(aero_params, q_dyn, q, v)
              * damp_fault)

    # ── Total angular acceleration ────────────────────────────────────────────
    I_safe = max(1e-6, I_yy_now)
    theta_ddot = (M_TVC + M_aero + M_damp) / I_safe + disturbance

    # Semi-implicit Euler integration
    q_new     = q     + plant.dt * theta_ddot
    theta_new = theta + plant.dt * q_new
    return theta_new, q_new


def step_plant(
    theta: float,
    q: float,
    u_act: float,
    disturbance: float,
    plant: PlantParams,
    keff_fault: float = 1.0,
    damp_fault: float = 1.0,
    # High-fidelity optional arguments (all None → simple mode)
    aero_params=None,
    q_dyn: float | None = None,
    v: float | None = None,
    T_now: float | None = None,
    I_yy_now: float | None = None,
    nonlinear_aero: bool = False,
) -> tuple[float, float]:
    """
    Unified plant step dispatcher.

    Routes to step_plant_simple() or step_plant_full() based on whether
    high-fidelity arguments are provided.  Both modes share the same
    semi-implicit Euler integration scheme.

    Simple mode (backward compatible):
        step_plant(theta, q, u_act, d, plant, keff_fault, damp_fault)

    Full mode:
        step_plant(theta, q, u_act, d, plant, keff_fault, damp_fault,
                   aero_params=aero, q_dyn=q_d, v=v_now, T_now=T,
                   I_yy_now=Iyy, nonlinear_aero=True)
    """
    if (aero_params is None
            or q_dyn is None
            or v is None
            or T_now is None
            or I_yy_now is None):
        # Simple mode — use legacy linearised model
        return step_plant_simple(theta, q, u_act, disturbance, plant,
                                  keff_fault, damp_fault)
    else:
        # Full physics mode
        return step_plant_full(theta, q, u_act, disturbance, plant,
                                aero_params, q_dyn, v, T_now, I_yy_now,
                                nonlinear_aero, keff_fault, damp_fault)


# ── Backward-compatible alias (external code calls effective_params) ───────────

def effective_params(
    plant: PlantParams,
    keff_fault: float = 1.0,
    damp_fault: float = 1.0,
) -> tuple[float, float]:
    """Alias for simple_effective_params — backward compatibility."""
    return simple_effective_params(plant, keff_fault, damp_fault)
