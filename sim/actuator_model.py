"""
actuator_model.py — Servo/gimbal actuator chain for TVC rocket.

Physical chain (applied in order each timestep):
──────────────────────────────────────────────────────────────────────────
  1. FIRST-ORDER LAG      servo motor dynamics; tracks command with time constant τ
  2. SLEW RATE LIMIT      max angular velocity of servo output shaft
  3. DEADBAND             minimum per-step movement below which servo does not move
                          (models stiction / minimum PWM resolution)
  4. BACKLASH             mechanical play in linkage on direction reversal
                          (standard symmetric hysteresis model)
  5. SATURATION           hard mechanical stop at ±u_max (gimbal travel limit)
──────────────────────────────────────────────────────────────────────────

Each nonlinearity is explicitly isolated; they do not share state.

Ordering justification
──────────────────────
  Lag and slew limit are properties of the servo drive and motor (electrical/
  mechanical bandwidth).  Deadband is a stiction effect in the drive train.
  Backlash is a positional hysteresis in the gimbal linkage, downstream of
  the servo shaft.  Saturation is the physical hard stop of the nozzle gimbal.

Backlash model
──────────────
  Standard symmetric hysteresis (Krasnosel'skii–Pokrovskii backlash operator):
      Let half_bl = backlash / 2
      u_out = u_in - half_bl     if u_in > u_out_prev + half_bl  (positive contact)
      u_out = u_in + half_bl     if u_in < u_out_prev - half_bl  (negative contact)
      u_out = u_out_prev         otherwise                        (in the gap)
  This never modifies past state.  The full backlash gap must be traversed on
  direction reversal before the output moves.

Units: all quantities in code units (see units.py for conversion to degrees).
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class ActuatorParams:
    u_max: float        # saturation limit (code units), ±u_max is the gimbal hard stop
    slew_max: float     # slew rate limit (code units/s); float('inf') to disable
    tau_act: float      # first-order lag time constant (s)
    deadband: float     # minimum per-step movement threshold (code units)
    backlash: float     # total backlash gap (code units); half applied each direction


@dataclass
class ActuatorState:
    """All mutable actuator state.  Initialise once before the simulation loop."""
    u_servo: float = 0.0   # servo internal position (after lag + slew, before backlash)
    u_output: float = 0.0  # nozzle gimbal position (after full chain), sent to plant


def step_actuator(
    state: ActuatorState,
    params: ActuatorParams,
    u_cmd: float,
    dt: float,
) -> float:
    """
    Advance actuator state by one timestep.  Mutates `state` in place.

    Parameters
    ----------
    state   : current actuator state (mutated)
    params  : actuator parameters (constant)
    u_cmd   : PID output command (code units), may be outside ±u_max
    dt      : timestep (s)

    Returns
    -------
    u_output : actuator output delivered to plant (code units)
    """
    # ── 1. First-order lag ──────────────────────────────────────────────────
    # Desired rate from lag dynamics (code units/s)
    du_lag = (u_cmd - state.u_servo) / max(1e-9, params.tau_act)

    # ── 2. Slew rate limit ──────────────────────────────────────────────────
    du_slewed = float(np.clip(du_lag, -params.slew_max, params.slew_max))

    # Per-step position change
    delta = du_slewed * dt

    # ── 3. Deadband ─────────────────────────────────────────────────────────
    # Stiction: the servo does not move if the position error from the current
    # servo position to the commanded position is below the breakaway threshold.
    #
    # Physical model: stiction force is proportional to the POSITION ERROR
    # (the spring force demand of the servo trying to reach the command).
    # When |u_cmd - u_servo| < deadband, the stiction force exceeds the
    # spring force and the servo is locked.
    #
    # Result: the servo stops when it gets within (deadband) of u_cmd,
    # creating a persistent steady-state error equal to the deadband.
    # This is the correct physical behavior of a servo with stiction —
    # the steady-state error is exactly why deadband matters for tracking.
    #
    # Note: DO NOT use |delta| here. At max slew (75 deg/s → 60 CU/s → 0.30 CU/step at 200 Hz),
    # |delta| is always smaller than any reasonable deadband, which would
    # permanently freeze the servo. Position error is the correct check.
    if abs(u_cmd - state.u_servo) < params.deadband:
        delta = 0.0

    u_servo_new = state.u_servo + delta

    # ── 4. Backlash (symmetric hysteresis) ──────────────────────────────────
    half_bl = params.backlash / 2.0
    u_out_prev = state.u_output

    if u_servo_new > u_out_prev + half_bl:
        u_out_new = u_servo_new - half_bl      # positive contact
    elif u_servo_new < u_out_prev - half_bl:
        u_out_new = u_servo_new + half_bl      # negative contact
    else:
        u_out_new = u_out_prev                 # in the gap — output stationary

    # ── 5. Saturation ───────────────────────────────────────────────────────
    u_out_sat = float(np.clip(u_out_new, -params.u_max, params.u_max))

    # Clip u_servo to the saturation limits as well.
    # Without this clip, u_servo accumulates past u_max when the output is
    # saturated (integrator windup in the servo state), causing the backlash
    # hysteresis gap to grow artificially on direction reversal after saturation.
    u_servo_new = float(np.clip(u_servo_new, -params.u_max, params.u_max))

    # Update state
    state.u_servo = u_servo_new
    state.u_output = u_out_sat

    return u_out_sat
