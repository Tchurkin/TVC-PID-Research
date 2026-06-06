"""
motor_model.py — Solid rocket motor model for TVC rocket simulator.

Physical quantities modeled
────────────────────────────
  T(t)          Thrust vs. time curve (N).  Parametric shape matched to F-class
                propellant charactersitics; can be replaced with measured data.
  m_prop(t)     Remaining propellant mass (kg).  Integrated from mass flow rate.
  m(t)          Total vehicle mass: m_dry + m_prop(t).
  x_CG(t)       Center of mass position from nose (m).  Shifts forward as propellant burns.
  I_yy(t)       Pitch moment of inertia (kg·m²).  Decreases as propellant burns.

Why CG shift matters
─────────────────────
  As propellant burns, mass is removed from the aft of the rocket (near the nozzle).
  This shifts the CG forward (toward the nose), reducing the CG-CP separation and
  therefore reducing the aerodynamic instability rate p.  For F-class motors, propellant
  mass is ~3-5% of total vehicle mass, so the effect is small (~1-3 mm CG shift).
  For G-class and above with higher propellant fractions (10-20%), the effect is
  significant and can change regime classification during the burn.

F15 motor reference profile
────────────────────────────
  Based on F-class APCP motor characteristics:
    Total impulse    ~43 Ns  (mid-F class)
    Average thrust   ~14.4 N
    Burn time        ~3.0 s
    Peak thrust      ~22 N   (at ignition, ~0.2 s)
    Propellant mass  ~0.028 kg
    Motor casing     ~0.048 kg (dry structure)

  The thrust curve is parametrically generated with four phases:
    1. Rise         t ∈ [0, t_peak]:   linear rise to T_peak
    2. Early burn   t ∈ [t_peak, t1]:  plateau near T_avg
    3. Taper        t ∈ [t1, t_burn]:  linear decay to zero
    4. Coast        t > t_burn:        T = 0

  This captures the ignition spike, plateau, and tail-off characteristic of
  composite propellant motors.  Replace with measured thrust curve data for
  flight-accurate simulation.

Usage
─────
  params = MotorParams.F15()
  state  = MotorState.initial(params)

  for k in range(N):
      T, mdot = motor_thrust_and_mdot(params, t[k])
      state.advance(mdot, dt)
      m_total   = params.m_dry + state.m_prop_remaining
      x_CG_now  = motor_cg_position(params, state.m_prop_remaining)
      I_yy_now  = motor_inertia(params, state.m_prop_remaining, rocket_I_yy_dry)
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class MotorParams:
    """Fixed motor parameters (read-only during simulation)."""
    # Thrust curve shape
    T_peak: float       # peak thrust at ignition (N)
    T_avg: float        # average / plateau thrust (N)
    t_peak: float       # time of peak thrust (s)
    t_plateau_end: float  # time plateau ends and taper begins (s)
    t_burn: float       # burnout time (s)

    # Propellant properties
    m_prop_total: float  # total propellant mass (kg)
    m_casing: float      # dry motor casing mass (kg)

    # Motor geometry (for CG calculation)
    x_motor_fore: float  # forward end of motor from nose (m)  [propellant CG is midway]
    x_motor_aft: float   # aft end of motor from nose (m)

    # Derived
    @property
    def m_motor_total(self) -> float:
        return self.m_prop_total + self.m_casing

    @property
    def x_prop_cg(self) -> float:
        """Propellant CG assumed at motor midpoint."""
        return 0.5 * (self.x_motor_fore + self.x_motor_aft)

    @classmethod
    def F15(cls, rocket_length: float = 0.50, scale: float = 1.0) -> "MotorParams":
        """
        F15-class APCP motor (mid-F, ~43 Ns, 3 s burn).
        Motor occupies aft 15% of rocket length.

        Measured masses: wet ~100 g, dry casing ~40 g, propellant ~60 g.
        The cardboard/clay Estes casing is relatively light compared to
        aluminum-cased APCP motors, giving a ~60% propellant fraction.

        scale: linear scaling factor applied to thrust AND both mass terms.
               scale=1.0 → standard F15.  scale=2.0 → motor with 2× total impulse,
               2× propellant mass, 2× casing mass.  Burn time is unchanged.
        """
        return cls(
            T_peak          = 22.0  * scale,
            T_avg           = 14.4  * scale,
            t_peak          = 0.12,
            t_plateau_end   = 2.4,
            t_burn          = 3.0,
            m_prop_total    = 0.060 * scale,   # 60 g propellant at scale=1 (wet 100g - dry 40g)
            m_casing        = 0.040 * scale,   # 40 g dry casing at scale=1
            x_motor_fore    = rocket_length * 0.85,
            x_motor_aft     = rocket_length * 1.00,
        )

    @classmethod
    def G80(cls, rocket_length: float = 0.55) -> "MotorParams":
        """
        G80-class APCP motor (~120 Ns, 1.5 s burn, higher propellant fraction).
        More significant CG shift than F-class (~8% propellant fraction).
        """
        return cls(
            T_peak          = 110.0,
            T_avg           = 80.0,
            t_peak          = 0.08,
            t_plateau_end   = 1.2,
            t_burn          = 1.5,
            m_prop_total    = 0.060,
            m_casing        = 0.095,
            x_motor_fore    = rocket_length * 0.80,
            x_motor_aft     = rocket_length * 1.00,
        )


@dataclass
class MotorState:
    """Mutable motor state: remaining propellant."""
    m_prop_remaining: float  # kg

    @classmethod
    def initial(cls, params: MotorParams) -> "MotorState":
        return cls(m_prop_remaining=params.m_prop_total)

    def advance(self, mdot: float, dt: float) -> None:
        """Update propellant remaining given mass flow rate mdot (kg/s) and timestep dt."""
        self.m_prop_remaining = max(0.0, self.m_prop_remaining - mdot * dt)


def motor_thrust_and_mdot(params: MotorParams, t: float) -> tuple[float, float]:
    """
    Return (thrust_N, mass_flow_kg_s) at time t.

    Thrust curve: rise → plateau → linear taper → coast.
    Mass flow rate = T(t) / (g0 * Isp) where we derive average Isp from
    total impulse / (m_prop * g0).

    Parameters
    ----------
    params : MotorParams
    t      : current time (s)

    Returns
    -------
    (T, mdot) — thrust in N, mass flow rate in kg/s
    """
    if t <= 0.0 or t >= params.t_burn:
        return 0.0, 0.0

    # Thrust curve
    if t <= params.t_peak:
        # Linear rise from 0 to T_peak
        T = params.T_peak * (t / params.t_peak)
    elif t <= params.t_plateau_end:
        # Plateau at average thrust
        T = params.T_avg
    else:
        # Linear taper from T_avg to 0
        frac = (t - params.t_plateau_end) / (params.t_burn - params.t_plateau_end)
        T = params.T_avg * (1.0 - frac)

    # Average specific impulse from total impulse
    # Isp = total_impulse / (m_prop * g0)
    # total_impulse ≈ T_avg * t_burn (area under curve)
    total_impulse = params.T_avg * params.t_burn  # simple estimate
    Isp = total_impulse / (params.m_prop_total * 9.81)
    mdot = T / (9.81 * Isp)

    return T, mdot


def motor_vehicle_mass(
    m_dry: float,
    motor: MotorParams,
    motor_state: MotorState,
) -> float:
    """Total vehicle mass = dry structure + motor casing + remaining propellant."""
    return m_dry + motor.m_casing + motor_state.m_prop_remaining


def motor_cg_position(
    x_cg_dry: float,
    m_dry_plus_casing: float,
    motor: MotorParams,
    motor_state: MotorState,
) -> float:
    """
    CG position from nose (m), accounting for propellant burn.

    Uses weighted-average of dry rocket CG (includes casing) and propellant CG.

    Parameters
    ----------
    x_cg_dry          : CG of dry rocket including motor casing (m from nose)
    m_dry_plus_casing : mass of dry rocket including motor casing (kg)
    motor             : motor parameters
    motor_state       : current motor state

    Returns
    -------
    x_CG (m from nose) — shifts forward as propellant is consumed
    """
    m_prop = motor_state.m_prop_remaining
    if m_prop <= 0.0:
        return x_cg_dry
    m_total = m_dry_plus_casing + m_prop
    return (m_dry_plus_casing * x_cg_dry + m_prop * motor.x_prop_cg) / m_total


def motor_inertia_correction(
    I_yy_dry: float,
    x_cg_dry: float,
    m_dry_plus_casing: float,
    motor: MotorParams,
    motor_state: MotorState,
    x_cg_total: float,
) -> float:
    """
    Total I_yy about current CG, accounting for propellant contribution.

    Propellant is modeled as a point mass at x_prop_cg (valid for thin-walled
    cylinder approximation; replace with distributed mass for higher accuracy).

    Parameters
    ----------
    I_yy_dry      : moment of inertia of dry rocket about its own CG (kg·m²)
    x_cg_dry      : CG position of dry rocket (m from nose)
    m_dry_plus_casing : dry mass including motor casing (kg)
    motor         : motor params
    motor_state   : current state
    x_cg_total    : current total CG position (m from nose)

    Returns
    -------
    I_yy_total (kg·m²) about current combined CG
    """
    # Parallel-axis correction for dry rocket as CG moves
    d_dry = x_cg_dry - x_cg_total
    I_dry_shifted = I_yy_dry + m_dry_plus_casing * d_dry**2

    # Propellant contribution (point mass at x_prop_cg)
    m_prop = motor_state.m_prop_remaining
    if m_prop <= 0.0:
        return I_dry_shifted
    d_prop = motor.x_prop_cg - x_cg_total
    I_prop = m_prop * d_prop**2

    return I_dry_shifted + I_prop
