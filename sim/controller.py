"""
controller.py — PID attitude controller for TVC rocket.

Controller form
───────────────
  u_pid = Kp · e  +  Kd · (de/dt)  +  Ki · ∫e dt

  e = θ_ref − θ_ctrl             (angle error, rad)

  Derivative action is on the MEASUREMENT, not the error:
      de/dt ≈ −(q_ctrl − q_ctrl_prev) / dt
  This avoids derivative kick on reference steps.  The negative sign is
  because q_ctrl = dθ_ctrl/dt, so dθ_ctrl/dt ≈ −d(error)/dt when θ_ref is
  constant.  For a constant reference, measurement derivative = −error derivative.

  Anti-windup: integrator state is clipped to [-i_lim, +i_lim] each step.
  The final output is clipped to [-u_max, +u_max] before being sent to the
  actuator.  No back-calculation anti-windup; simple clamping only.

Units
─────
  θ_ref, θ_ctrl   rad
  q_ctrl          rad/s
  u_pid           code units (before saturation)
  Kp              code units / rad
  Kd              code units / (rad/s)  ≡  code units · s / rad
  Ki              code units / (rad · s)
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class PIDParams:
    Kp: float           # proportional gain (code units / rad)
    Kd: float           # derivative gain (code units / (rad/s))
    Ki: float = 0.0     # integral gain (code units / (rad·s))
    u_max: float = 12.0 # output saturation limit (code units)
    i_lim: float = 12.0 # integrator anti-windup limit (code units)


@dataclass
class PIDState:
    """Mutable PID state.  Initialise once before the loop."""
    i_state: float = 0.0         # integrator accumulator (code units)
    q_ctrl_prev: float = 0.0     # previous rate measurement (rad/s) for derivative


def step_pid(
    state: PIDState,
    params: PIDParams,
    theta_ctrl: float,
    q_ctrl: float,
    theta_ref: float,
    dt: float,
) -> float:
    """
    Compute one PID output.  Mutates `state` in place.

    Parameters
    ----------
    state      : PID state (mutated)
    params     : PID parameters (constant)
    theta_ctrl : angle measurement from sensor (rad)
    q_ctrl     : rate measurement from sensor (rad/s)
    theta_ref  : reference angle command (rad)
    dt         : timestep (s)

    Returns
    -------
    u_cmd (code units) — clipped to ±u_max, ready for actuator
    """
    e = theta_ref - theta_ctrl

    # Derivative on measurement (avoids kick on reference steps)
    de = -(q_ctrl - state.q_ctrl_prev) / dt

    # Integrator with anti-windup clamp
    state.i_state = float(
        np.clip(state.i_state + params.Ki * e * dt, -params.i_lim, params.i_lim)
    )

    u_pid = params.Kp * e + params.Kd * de + state.i_state

    state.q_ctrl_prev = q_ctrl

    return float(np.clip(u_pid, -params.u_max, params.u_max))
