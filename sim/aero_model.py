"""
aero_model.py — Physical aerodynamic model for a TVC model rocket.

How aerodynamics actually work for a TVC rocket
─────────────────────────────────────────────────
A rocket flying at angle of attack α (body axis ≠ velocity vector) experiences
aerodynamic forces at every point on the body.  For a smooth body of revolution
without fins, the aerodynamic loads are well described by three quantities:

  1. Normal force N: perpendicular to the velocity vector, acts at the center of
     pressure (CP).  For α > 0 (nose tilted left from velocity), N pushes the nose
     further left — destabilising for an unstable rocket (CP forward of CG).

  2. Axial force (drag) A: parallel to the velocity vector, always opposing motion.
     Does not produce a pitching moment (acts along the thrust axis).

  3. Pitching moment M_aero: moment about the center of mass, produced by N acting
     at CP with lever arm (x_CG − x_CP).

Physical picture of the pitching moment
─────────────────────────────────────────
  Imagine looking at the rocket from the side.  The oncoming air hits the nose and
  body at a small angle α.  The resulting sideways force (normal force) acts at the
  CP, which for a finless rocket is roughly at the 25th-33rd percentile of body
  length from the nose.

  If x_CP < x_CG (CP is forward, closer to nose — aerodynamically unstable):
    A nose-up tilt (α > 0) produces a nose-up normal force at a point FORWARD of
    the CG.  The moment arm is (x_CG − x_CP) in the positive direction.
    This creates a NOSE-UP moment — it makes the tilt worse.  Destabilising.

  If x_CP > x_CG (CP is aft — aerodynamically stable, e.g. finned rocket):
    Same nose-up tilt produces a normal force aft of the CG.
    This creates a NOSE-DOWN restoring moment.  Stabilising.

  For TVC rockets: CP is deliberately kept forward of CG (no large fins) so that
  the rocket NEEDS active control.  Without TVC, it would diverge.  The TVC gimbal
  provides the restoring moment the missing fins would otherwise supply.

Aerodynamic coefficients used
───────────────────────────────
  CN_alpha (per radian):
    Normal force coefficient slope.  The normal force per unit dynamic pressure
    and reference area: N = CN_alpha × α × q_dyn × S_ref.
    For a smooth body of revolution (Barrowman slender body theory):
        CN_alpha ≈ 2.0 / rad  (nose + body, no fins)
    With 3-fin set (trapezoidal, ~4 cm root chord, ~3 cm span):
        CN_alpha ≈ 4.5–6.0 / rad  (fins add ~2.5–4 / rad)
    This study uses finless bodies (TVC provides all stability), so:
        CN_alpha ≈ 2.0 / rad

  Cm_alpha (per radian, about CG):
    Pitching moment coefficient slope.  In this simulator's convention:
        Cm_alpha > 0 → destabilising (CP forward of CG)
        Cm_alpha = CN_alpha × (x_CG − x_CP) / D_ref
    The design space parameter Cm_alpha is in [−70, −30] using the OPPOSITE sign
    convention (negative = unstable).  Internally we use abs(Cm_alpha) for moment
    computation.  The magnitude encodes both the CP-CG separation and the normal
    force coefficient.

  Cmq (per radian, pitch damping):
    Pitch damping moment coefficient.  When the rocket pitches at rate θ̇, the tail
    sweeps through the air creating an effective angle of attack at the tail that
    opposes the rotation.  The resulting moment is:
        M_damp = q_dyn × S_ref × D_ref × Cmq × (θ̇ × D_ref / (2V))
    Note the 1/V factor: at low speed (launch) there is little damping; at high speed
    there is significant damping.  This is physically important because early-flight
    instability is harder to damp than late-flight instability.
    For a finless slender body: Cmq ≈ −10 to −20
    With fins: Cmq ≈ −30 to −60 (fins provide most of the pitch damping)
    This study uses Cmq ≈ −15 for finless TVC rockets.

  CD (drag coefficient, axial):
    Drag reduces acceleration and hence reduces the peak dynamic pressure reached.
    For a smooth body at subsonic speeds: CD ≈ 0.35–0.50
    The drag directly enters the velocity model (translational dynamics).

  CD_alpha (induced drag slope):
    CD increases slightly with α squared: CD_total = CD0 + CD_alpha × α²
    Typically small; CD_alpha ≈ 0.01–0.05 / rad²

Angle of attack convention
───────────────────────────
  For near-vertical flight, α ≈ θ (body angle from vertical).
  More precisely: α = θ − γ where γ is the flight-path angle.
  For the first few seconds of vertical launch, γ ≈ 0, so α ≈ θ.

Nonlinear aerodynamics
───────────────────────
  For small α (< 20 deg = 0.35 rad), the linear model N = CN_alpha × α is accurate.
  For larger α, the normal force saturates and eventually reverses (stall).
  The nonlinear model uses:
      CN(α) = CN_alpha × sin(α) × cos²(α)  [approximate nonlinear body lift]
  This is the Polhamus leading-edge suction analogy adapted for blunt bodies.
  At small α: sin(α) ≈ α, cos²(α) ≈ 1, so this reduces to the linear model.
  At large α: the force saturates below the linear extrapolation.

Reference geometry (default values, replace with measured when available)
──────────────────────────────────────────────────────────────────────────
  These are derived from the design space parameters plus fixed rocket geometry:
    D_ref    = 0.040 m    body diameter (4 cm; 38 mm motor mount + shell)
    S_ref    = π/4 × D_ref² = 1.257e-3 m²
    L_rocket = 0.500 m    total rocket length
    x_CG     = 0.280 m from nose  (56% of length, typical for bottom-heavy TVC)
    x_CP     = static_margin × D_ref + x_CG  (from design parameter)

CFD path
─────────
  When CFD results are available, replace:
    CN_alpha_from_design()  → table lookup from CFD norce force data
    Cmq_from_design()       → table lookup from CFD damping derivative
    CD0 = 0.40              → CFD/wind tunnel drag coefficient
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


# ── Fixed reference geometry ──────────────────────────────────────────────────

D_REF     = 0.040   # m — body diameter (4 cm)
S_REF     = np.pi / 4.0 * D_REF**2   # m² — reference area
L_ROCKET  = 0.500   # m — rocket length (adjust for specific vehicle)
RHO_AIR   = 1.20    # kg/m³ — sea-level air density


@dataclass
class AeroParams:
    """
    Physical aerodynamic parameters for the pitching-moment and drag model.

    All coefficients are dimensionless and referenced to D_ref and S_ref.
    Pitching moments are about the CG.

    convention: CN_alpha > 0, Cm_alpha > 0 means destabilising
    (this is the OPPOSITE of standard aerospace convention where
    Cm_alpha < 0 = stable).  Chosen for clarity in TVC context where
    all sampled designs are unstable by construction.
    """
    CN_alpha: float       # normal force coefficient slope (/ rad)  > 0
    Cm_alpha: float       # pitching moment coefficient slope (/ rad) > 0 for unstable
    Cmq: float            # pitch damping coefficient (/rad); should be < 0
    CD0: float            # zero-alpha drag coefficient
    CD_alpha2: float      # induced drag (alpha²) coefficient (/ rad²)
    D_ref: float = D_REF # reference diameter (m)
    S_ref: float = S_REF # reference area (m²)

    @classmethod
    def from_design(
        cls,
        Cm_alpha_signed: float,  # design space value, negative for unstable
        static_margin_calibers: float,
    ) -> "AeroParams":
        """
        Derive aerodynamic parameters from the design space Cm_alpha and static margin.

        The design space Cm_alpha ∈ [−70, −30] encodes the combined effect of
        normal force slope and CP-CG separation:
            |Cm_alpha_design| ≈ CN_alpha × static_margin (in calibers)

        For a finless body, CN_alpha ≈ 2.0, so:
            static_margin_from_Cm = |Cm_alpha| / CN_alpha
        This is consistent with the design parameter static_margin ∈ [0.04, 0.24].

        Pitch damping Cmq is estimated from slender body theory as
        proportional to −(L/D)² for a cylindrical body.  For L=0.5, D=0.04:
            (L/D) ≈ 12.5 → Cmq ≈ −15 (finless, conservative estimate)
        """
        CN_alpha = 2.0  # finless/finned body (Barrowman, slender body)
        # Signed Cm_alpha:
        #   static_margin_calibers > 0 → CP forward of CG → destabilising (+)
        #   static_margin_calibers = 0 → neutral
        #   static_margin_calibers < 0 → CP aft of CG → restoring (−, stable rocket)
        # This allows aerodynamically stable designs (finned rockets with positive
        # static margin in the traditional sense) to be included in the design space.
        Cm_alpha_physical = CN_alpha * static_margin_calibers

        # Pitch damping: finless body; proportional to body slenderness
        L_D_ratio = L_ROCKET / D_REF
        Cmq = -max(5.0, 0.08 * L_D_ratio**2)   # typically -10 to -20 for finless

        return cls(
            CN_alpha  = CN_alpha,
            Cm_alpha  = Cm_alpha_physical,
            Cmq       = Cmq,
            CD0       = 0.40,     # typical finless body, subsonic
            CD_alpha2 = 0.03,     # small induced drag
        )


def aero_normal_force(
    params: AeroParams,
    q_dyn: float,
    alpha: float,
    nonlinear: bool = False,
) -> float:
    """
    Normal force (N) perpendicular to velocity vector, acting at CP.

    Parameters
    ----------
    params    : aerodynamic parameters
    q_dyn     : dynamic pressure 0.5*rho*v² (Pa)
    alpha     : angle of attack (rad)
    nonlinear : use sin(alpha)*cos^2(alpha) instead of linear alpha

    Returns
    -------
    N (N) — positive when alpha > 0 (nose-up relative to velocity vector)
    """
    if nonlinear:
        # Approximate nonlinear body lift (Polhamus-style)
        alpha_eff = np.sin(alpha) * np.cos(alpha)**2
    else:
        alpha_eff = alpha
    return params.CN_alpha * q_dyn * params.S_ref * alpha_eff


def aero_pitching_moment(
    params: AeroParams,
    q_dyn: float,
    alpha: float,
    nonlinear: bool = False,
) -> float:
    """
    Aerodynamic pitching moment about CG (N·m).

    Positive moment → nose-up (destabilising for nose-up alpha on unstable rocket).

    M_aero = q_dyn × S_ref × D_ref × Cm_alpha × alpha_eff
    where Cm_alpha > 0 means CP is forward of CG (unstable).

    Parameters
    ----------
    params    : aerodynamic parameters
    q_dyn     : dynamic pressure (Pa)
    alpha     : angle of attack (rad)
    nonlinear : use sin(alpha) instead of linear alpha

    Returns
    -------
    M_aero (N·m) — positive = nose-up destabilising moment
    """
    if nonlinear:
        alpha_eff = np.sin(alpha) * np.cos(alpha)**2
    else:
        alpha_eff = alpha
    return params.Cm_alpha * q_dyn * params.S_ref * params.D_ref * alpha_eff


def aero_pitch_damping_moment(
    params: AeroParams,
    q_dyn: float,
    q_rate: float,  # pitch rate theta_dot (rad/s)
    v: float,       # flight speed (m/s)
) -> float:
    """
    Pitch damping moment from rotational motion (N·m).

    When the rocket pitches at rate theta_dot, the tail sweeps through the air
    faster than the nose, creating an asymmetric angle of attack distribution
    that produces a restoring moment proportional to pitch rate.

    M_damp = q_dyn × S_ref × D_ref × Cmq × (theta_dot × D_ref) / (2 × v)

    Note the 1/v factor: damping scales with q_dyn/v ∝ v (not v²).
    At low speed (launch): small damping.  At peak speed: maximum damping.

    Parameters
    ----------
    params   : aerodynamic parameters
    q_dyn    : dynamic pressure (Pa)
    q_rate   : pitch rate theta_dot (rad/s)
    v        : flight speed (m/s); protected against near-zero

    Returns
    -------
    M_damp (N·m) — negative for positive pitch rate (opposing rotation)
    """
    v_safe = max(0.5, v)   # avoid division by zero at launch
    return (q_dyn * params.S_ref * params.D_ref
            * params.Cmq
            * (q_rate * params.D_ref) / (2.0 * v_safe))


def aero_drag(
    params: AeroParams,
    q_dyn: float,
    alpha: float,
) -> float:
    """
    Axial drag force (N).  Acts along velocity vector opposing motion.
    Does not produce a pitching moment.

    F_drag = q_dyn × S_ref × (CD0 + CD_alpha2 × alpha²)
    """
    CD = params.CD0 + params.CD_alpha2 * alpha**2
    return q_dyn * params.S_ref * CD


def dynamic_pressure(v: float, rho: float = RHO_AIR) -> float:
    """q_dyn = 0.5 × rho × v² (Pa)."""
    return 0.5 * rho * v**2


def velocity_step(
    v: float,
    m: float,
    T: float,
    theta: float,
    q_dyn: float,
    aero: AeroParams,
    g: float = 9.81,
    dt: float = 0.005,
) -> float:
    """
    Advance flight speed by one timestep (simplified 1-DOF axial dynamics).

    For near-vertical flight (theta small), this integrates:
        m × dv/dt = T × cos(theta) − F_drag × cos(theta) − m × g × cos(theta)

    The cos(theta) factors account for the projection of thrust and gravity
    onto the velocity direction for a tilted rocket.  For small theta they
    are all ≈ 1.

    This gives time-varying v(t) needed for dynamic pressure q_dyn(t).

    Parameters
    ----------
    v      : current flight speed (m/s)
    m      : current vehicle mass (kg)
    T      : current thrust (N)
    theta  : current pitch angle from vertical (rad)
    q_dyn  : current dynamic pressure (Pa)
    aero   : aerodynamic parameters
    g      : gravitational acceleration (m/s²)
    dt     : timestep (s)

    Returns
    -------
    v_new (m/s)
    """
    cos_theta = np.cos(theta)
    F_drag    = aero_drag(aero, q_dyn, theta)  # use theta as approx alpha for axial
    a_net     = (T * cos_theta - F_drag * cos_theta) / m - g * cos_theta
    return max(0.0, v + dt * a_net)
