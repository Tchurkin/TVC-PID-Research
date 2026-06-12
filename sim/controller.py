"""
controller.py — PID and ADRC attitude controllers for TVC rocket.

PID form
────────
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

PID units
─────────
  θ_ref, θ_ctrl   rad
  q_ctrl          rad/s
  u_pid           code units (before saturation)
  Kp              code units / rad
  Kd              code units / (rad/s)  ≡  code units · s / rad
  Ki              code units / (rad · s)

ADRC form (Active Disturbance Rejection Control)
────────────────────────────────────────────────
  The plant model is:  θ̈ = b0·u + f(t)
  where b0 is the known control gain (≈ keff_full) and f(t) is the "total
  disturbance" lumping wind, aerodynamics, model mismatch, and anything else.

  Extended State Observer (ESO) estimates [θ, θ̇, f] as [z1, z2, z3]:
      ż1 = z2 + 3·ω0·(θ_meas − z1)
      ż2 = b0·u_prev + z3 + 3·ω0²·(θ_meas − z1)
      ż3 = ω0³·(θ_meas − z1)
  where ω0 is the ESO bandwidth (rad/s).

  Control law (after disturbance cancellation):
      u0 = Kp·(θ_ref − z1) − Kd·z2        [PD in state-space form]
      u  = (u0 − z3) / b0                   [disturbance feed-forward]
  where Kp = ωc², Kd = 2·ωc (critical damping, ωc = closed-loop bandwidth).

  Key insight: the PD loop sees an approximately disturbance-free plant because
  z3 cancels f(t) in the control output.  This allows moderate ωc without
  requiring high proportional gain to fight wind — unlike PID which can only
  reject wind by increasing Kp (triggering bang-bang in FRAGILE designs).

ADRC units
──────────
  u0      rad/s²       (intermediate command before b0 normalisation)
  z3      rad/s²       (disturbance estimate)
  u       code units   (after dividing by b0 [rad/s²/CU])
  b0      rad/s²/CU    (≈ keff_full = T·CU_TO_RAD·l_nozzle / Iyy)
  Kp      1/s²         (= ωc²; different units from PID Kp!)
  Kd      1/s          (= 2·ωc)
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


# ── ADRC controller ───────────────────────────────────────────────────────────

@dataclass
class ADRCParams:
    """
    ADRC parameters.  Tuned by two bandwidths and the plant gain estimate.

    omega_c : closed-loop bandwidth (rad/s).  Sets Kp = ωc², Kd = 2·ωc.
              Analogous to PID Kp but in physical units (1/s²).
              Typical range for hobby TVC: 5–50 rad/s.
    omega0  : ESO bandwidth (rad/s).  Controls disturbance estimation speed.
              Rule of thumb: omega0 = 3–10 × omega_c.
              Too low → slow disturbance tracking; too high → noise amplification.
    b0      : estimated plant control gain (rad/s² / code unit).
              Best choice: keff_full = T_avg·CU_TO_RAD·l_nozzle / Iyy.
              Insensitive to 2× errors; degrades at ~5× mismatch.
    u_max   : output saturation (code units), matches actuator.u_max.
    """
    omega_c: float
    omega0:  float
    b0:      float
    u_max:   float = 12.0

    @property
    def Kp(self) -> float:
        """Closed-loop proportional gain (1/s²)."""
        return self.omega_c ** 2

    @property
    def Kd(self) -> float:
        """Closed-loop derivative gain (1/s); critical damping."""
        return 2.0 * self.omega_c


@dataclass
class ADRCState:
    """Mutable ADRC state.  Initialise once before the loop."""
    z1:     float = 0.0   # ESO estimate of θ (rad)
    z2:     float = 0.0   # ESO estimate of θ̇ (rad/s)
    z3:     float = 0.0   # ESO estimate of total disturbance f(t) (rad/s²)
    u_prev: float = 0.0   # previous control command (code units) used in ESO model


def step_adrc(
    state: ADRCState,
    params: ADRCParams,
    theta_ctrl: float,
    q_ctrl: float,    # accepted for interface compatibility; ESO uses theta only
    theta_ref: float,
    dt: float,
) -> float:
    """
    Compute one ADRC output.  Mutates `state` in place.

    The ESO uses only the angle measurement (theta_ctrl); q_ctrl is the sensor
    rate measurement which is accepted for API compatibility but not fed to the
    observer — z2 (rate estimate) is produced internally by the ESO.

    Parameters
    ----------
    state      : ADRC state (mutated: z1, z2, z3, u_prev updated)
    params     : ADRC parameters (constant)
    theta_ctrl : angle measurement from sensor (rad)
    q_ctrl     : rate measurement (rad/s) — not used by ESO, accepted for compatibility
    theta_ref  : reference angle command (rad)
    dt         : timestep (s)

    Returns
    -------
    u_cmd (code units) — clipped to ±u_max, ready for actuator
    """
    w = params.omega0
    e_obs = theta_ctrl - state.z1   # ESO observation error

    # ESO update (Euler forward; uses previous u for model term)
    z1_dot = state.z2 + 3.0 * w * e_obs
    z2_dot = params.b0 * state.u_prev + state.z3 + 3.0 * w ** 2 * e_obs
    z3_dot = w ** 3 * e_obs

    state.z1 += dt * z1_dot
    state.z2 += dt * z2_dot
    state.z3 += dt * z3_dot

    # PD control + disturbance cancellation
    e  = theta_ref - state.z1
    u0 = params.Kp * e - params.Kd * state.z2   # rad/s²
    u  = (u0 - state.z3) / params.b0             # code units

    u_cmd = float(np.clip(u, -params.u_max, params.u_max))
    state.u_prev = u_cmd
    return u_cmd
