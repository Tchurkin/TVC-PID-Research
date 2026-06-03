"""
plant_dynamics.py — Linearised 1-DOF pitch dynamics for a TVC rocket.

Equation of motion (Euler-angle linearisation about vertical):

    θ̈ = keff_eff · u_act  −  damp_eff · θ̇  +  p² · θ  +  d(t)

State:  (θ, θ̇) = (pitch angle, pitch rate)
Input:  u_act  — actuator output (code units, from actuator_model.py)
        d(t)   — external disturbance angular acceleration (rad/s²)

Physical meaning of each term
──────────────────────────────
  keff_eff · u_act   TVC control moment / Iyy.  Positive u_act pitches toward
                      vertical (restoring).  keff_eff scales with thrust (more
                      force → more authority) and inversely with inertia.
  damp_eff · θ̇       Aerodynamic pitch damping.  Always opposes motion.
  p² · θ              Open-loop instability.  p > 0 means the equilibrium
                      θ=0 is unstable without active control.  For a rocket
                      with CP forward of CG plus gravitational pendulum,
                      p ≈ sqrt(|Mα / Iyy|).  See design_space.py for how
                      p is derived from design parameters.
  d(t)                Wind gusts + deterministic sinusoidal disturbance.

Effective parameter scaling
───────────────────────────
  keff_eff = keff × keff_fault_scale × thrust_scale × inertia_scale
      thrust_scale  = T / T_ref          (more thrust → more TVC force)
      inertia_scale = Iyy_ref / Iyy      (more inertia → less angular accel)

  damp_eff = aero_damp × damp_fault_scale × sqrt(max(0.2, mass_scale))
      mass_scale = m / m_ref
      NOTE: sqrt(mass_scale) is a modeling approximation, not derived.
            See units.py disclosure #1.

Integration scheme: semi-implicit Euler
  q_new     = q     + dt × q̇          (forward Euler on rate)
  theta_new = theta + dt × q_new       (uses updated rate — semi-implicit)
This matches the MATLAB reference implementation.
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from units import REF_KEFF, REF_AERO_DAMP


@dataclass
class PlantParams:
    """All plant parameters.  Dimensionless scale factors are relative to REF_* values."""
    p_unstable: float               # open-loop instability rate (1/s), must be ≥ 0
    control_eff: float = REF_KEFF   # nominal keff (rad/s² / code unit)
    aero_damp: float = REF_AERO_DAMP  # baseline aerodynamic damping (1/s)

    mass_scale: float = 1.0         # m / m_ref,       clamped [0.30, 3.00]
    inertia_scale: float = 1.0      # Iyy_ref / Iyy,   clamped [0.20, 5.00]
    thrust_scale: float = 1.0       # T / T_ref,        clamped [0.20, 5.00]

    dt: float = 0.005               # integration timestep (s)


def effective_params(
    plant: PlantParams,
    keff_fault: float = 1.0,
    damp_fault: float = 1.0,
) -> tuple[float, float]:
    """
    Compute runtime keff_eff and damp_eff from plant params and optional fault scales.

    Returns
    -------
    (keff_eff, damp_eff) both in 1/s² (rad/s² per unit argument)
    """
    keff_eff = (
        plant.control_eff
        * keff_fault
        * plant.thrust_scale
        * plant.inertia_scale
    )
    damp_eff = (
        plant.aero_damp
        * damp_fault
        * np.sqrt(max(0.2, plant.mass_scale))
    )
    return keff_eff, damp_eff


def step_plant(
    theta: float,
    q: float,
    u_act: float,
    disturbance: float,
    plant: PlantParams,
    keff_fault: float = 1.0,
    damp_fault: float = 1.0,
) -> tuple[float, float]:
    """
    Advance plant state by one timestep (semi-implicit Euler).

    Parameters
    ----------
    theta       : pitch angle (rad)
    q           : pitch rate (rad/s)
    u_act       : actuator output (code units)
    disturbance : external angular acceleration (rad/s²)
    plant       : plant parameters
    keff_fault  : multiplicative degradation of control effectiveness (1.0 = nominal)
    damp_fault  : multiplicative degradation of aerodynamic damping (1.0 = nominal)

    Returns
    -------
    (theta_new, q_new) — updated state after one timestep
    """
    keff_eff, damp_eff = effective_params(plant, keff_fault, damp_fault)

    qdot = (
        keff_eff * u_act
        - damp_eff * q
        + plant.p_unstable ** 2 * theta
        + disturbance
    )

    q_new = q + plant.dt * qdot
    theta_new = theta + plant.dt * q_new   # semi-implicit: uses updated rate
    return theta_new, q_new
