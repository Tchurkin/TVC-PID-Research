"""
sim_vsr.vehicle -- mass/inertia/thrust schedules, aerodynamics, planar EOM.

Longitudinal (pitch-plane) flight of a TVC rocket whose static margin can be actively
changed in flight. State is the 6 flight variables; the gimbal deflection and the static
margin are supplied each step by the actuators (sim_vsr.actuators).

Frame: angles from HORIZONTAL. Straight-up launch -> gamma = theta = +90 deg.
  x, z      downrange / altitude            [m]
  V         airspeed                        [m/s]
  gamma     flight-path angle (vel vector)  [rad]
  theta     body pitch attitude             [rad]
  q         body pitch rate                 [rad/s]
  alpha = theta - gamma                     angle of attack [rad]

Pitch moment about CG:
  M_aero = -q_dyn * S * CN_alpha * alpha * (sm * d_ref)   (restoring iff sm>0)
  M_tvc  =  T * sin(delta) * L_nozzle                     (+delta pitches nose up)
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import numpy as np

G = 9.80665
# rad of gimbal per control-unit (matches the main project: pi/180 * 15/12). Used only to
# report Pi = keff_CU * latency^2 on the SAME scale as the project's Pi_crit ~ 275.
CU_TO_RAD = np.pi / 180.0 * 15.0 / 12.0   # 0.021817


@dataclass
class State:
    x: float = 0.0
    z: float = 0.0
    V: float = 0.1
    gamma: float = np.pi / 2     # straight up
    theta: float = np.pi / 2
    q: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.z, self.V, self.gamma, self.theta, self.q], float)

    @staticmethod
    def from_array(a: np.ndarray) -> "State":
        return State(a[0], a[1], a[2], a[3], a[4], a[5])


@dataclass
class CanardParams:
    """Deployable forward canard (fold-out). Generates N_c = q*S_c*CN_c*(alpha+defl) at arm_c
    FORWARD of the CG. The alpha-part is destabilizing (margin modulation when deployed); the
    defl-part is a direct nose-up control moment (raw canard control). Both scale with q ~ V^2."""
    S_c:          float = 0.0018    # m^2 effective area when fully deployed
    CN_c:         float = 3.0       # 1/rad canard normal-force slope
    arm_c:        float = 0.28      # m, canard AC forward of CG (>0)
    defl_max_deg: float = 20.0      # max canard deflection (raw-control authority)

    def dmargin_full(self, p: "VehicleParams") -> float:
        """Static-margin reduction (cal) from full deployment, vs the aft-fin baseline."""
        return (self.S_c * self.CN_c * self.arm_c) / (p.S_ref * p.CN_alpha * p.d_ref)


@dataclass
class VehicleParams:
    # masses / inertia (liftoff -> burnout linearly with propellant fraction)
    m_dry:   float = 0.45      # kg burnout
    m_prop:  float = 0.060     # kg propellant
    Iyy_dry: float = 0.015     # kg m^2 burnout
    Iyy_wet: float = 0.020     # kg m^2 liftoff
    # geometry
    L_body:   float = 0.75     # m
    d_ref:    float = 0.066    # m reference diameter
    L_nozzle: float = 0.35     # m nozzle->CG moment arm
    # aerodynamics
    CN_alpha: float = 8.0      # 1/rad normal-force slope (body+fins)
    CD0:      float = 0.55     # zero-lift drag coeff
    K_ind:    float = 1.2      # induced-drag factor (CD = CD0 + K*CN_alpha*alpha^2)
    rho:      float = 1.20     # kg/m^3
    stall_deg: Optional[float] = None   # fin stall angle; None = linear CN (legacy). e.g. 15.
    stall_model: str = "plateau"        # "plateau" (gentle) or "drop" (realistic post-stall CN loss)
    S_ref:    Optional[float] = None

    def __post_init__(self):
        if self.S_ref is None:
            self.S_ref = np.pi * (self.d_ref / 2.0) ** 2


def mass_props(p: VehicleParams, prop_frac: float):
    """prop_frac in [0,1] = fraction of propellant remaining."""
    prop_frac = float(np.clip(prop_frac, 0.0, 1.0))
    m = p.m_dry + p.m_prop * prop_frac
    Iyy = p.Iyy_dry + (p.Iyy_wet - p.Iyy_dry) * prop_frac
    return m, Iyy


def cn_eff(slope: float, angle: float, stall_deg: Optional[float], stall_model: str = "plateau") -> float:
    """Normal-force coefficient with optional stall. Linear below stall_deg.
      'plateau' : CN saturates flat past stall (gentle, conservative)
      'drop'    : CN loses up to 40% over one stall-angle past stall, then flat (realistic post-stall)
    stall_deg=None -> pure linear (legacy)."""
    if stall_deg is None:
        return slope * angle
    a_s = np.deg2rad(stall_deg); a = abs(angle); sg = np.sign(angle)
    if a <= a_s:
        return slope * angle
    cn_max = slope * a_s
    if stall_model == "drop":
        f = min((a - a_s) / a_s, 1.0)
        return cn_max * sg * (1.0 - 0.4 * f)
    return cn_max * sg


def aero_forces(p: VehicleParams, V: float, alpha: float):
    """Returns (N normal force, D drag, q_dyn)."""
    q_dyn = 0.5 * p.rho * V * V
    CN = cn_eff(p.CN_alpha, alpha, p.stall_deg, p.stall_model)
    N = q_dyn * p.S_ref * CN
    CD = p.CD0 + p.K_ind * p.CN_alpha * alpha * alpha
    D = q_dyn * p.S_ref * CD
    return N, D, q_dyn


def keff_phys(p: VehicleParams, thrust: float, Iyy: float) -> float:
    """Control effectiveness [rad/s^2 per rad gimbal] = T*L_nozzle/Iyy (small-angle)."""
    return thrust * p.L_nozzle / max(Iyy, 1e-6)


def deriv(s: State, gimbal_rad: float, sm_cal: float, thrust: float,
          p: VehicleParams, prop_frac: float, wind_alpha: float = 0.0,
          canard: Optional[CanardParams] = None, deploy_frac: float = 0.0,
          canard_defl_rad: float = 0.0, aft_defl_rad: float = 0.0) -> np.ndarray:
    """Time-derivative of [x,z,V,gamma,theta,q]. wind_alpha = gust-induced effective AoA [rad].

    sm_cal is the BASELINE (canard-retracted) static margin set by the fixed aft fins. If a
    canard is supplied and deploy_frac>0, its alpha-coupling reduces effective stability
    (margin modulation) and its deflection adds a direct control moment (raw canard control).
    """
    m, Iyy = mass_props(p, prop_frac)
    V = max(s.V, 0.2)
    alpha = (s.theta - s.gamma) + wind_alpha
    N, D, q_dyn = aero_forces(p, V, alpha)

    M_aero = -N * (sm_cal * p.d_ref)   # aft fins; N already carries stall saturation
    M_tvc = thrust * np.sin(gimbal_rad) * p.L_nozzle
    N_can = 0.0
    M_can = 0.0
    if canard is not None and deploy_frac > 0.0:
        # forward canard: +force -> nose-up moment; alpha-part destabilizes, defl-part steers
        cn_c = cn_eff(canard.CN_c, alpha + canard_defl_rad, p.stall_deg, p.stall_model)
        N_can = q_dyn * (canard.S_c * deploy_frac) * cn_c
        M_can = N_can * canard.arm_c
    # Steerable AFT fins (optional): an additive aerodynamic control surface at the tail. Its lift
    # saturates with its own deflection (stall on the control angle) -- same authority scale as the
    # canard. Moment-only (translational effect of a small control surface is second-order).
    M_aft = 0.0
    if canard is not None and aft_defl_rad != 0.0:
        cn_aft = cn_eff(canard.CN_c, aft_defl_rad, p.stall_deg, p.stall_model)
        M_aft = q_dyn * canard.S_c * cn_aft * canard.arm_c
    qdot = (M_aero + M_tvc + M_can + M_aft) / Iyy

    Vdot = (thrust * np.cos(alpha) - D) / m - G * np.sin(s.gamma)
    gammadot = (thrust * np.sin(alpha) + N + N_can) / (m * V) - G * np.cos(s.gamma) / V
    xdot = V * np.cos(s.gamma)
    zdot = V * np.sin(s.gamma)
    thetadot = s.q
    return np.array([xdot, zdot, Vdot, gammadot, thetadot, qdot], float)
