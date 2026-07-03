"""
sim_vsr.control -- TVC attitude controller (PID or ADRC) + stability scheduler.

TVCController:
  PID  : gimbal_cmd_deg = Kp*e + Kd*edot + Ki*int(e)   (reactive; faces the Pi constraint)
  ADRC : linear ESO estimates total disturbance (incl. bang-bang energy) and cancels it
         upstream of the gimbal -> servo does not saturate -> escapes the Pi constraint.

StabilityScheduler -- the variable-stability logic. Modes:
  fixed_stable    : hold sm = sm_stable always (conventional rocket; aero fights the maneuver)
  fixed_unstable  : hold sm = sm_unstable always (fast, but never settles)
  switched        : go unstable to fling toward the setpoint, snap stable to catch & settle
                    (hysteresis on attitude error; the central variable-stability idea)
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


# --------------------------------------------------------------------------- TVC

@dataclass
class PIDGains:
    Kp: float = 0.6          # deg gimbal per rad error (small: keff is large)
    Kd: float = 0.10
    Ki: float = 0.0


@dataclass
class ADRCGains:
    omega_c: float = 5.0     # controller bandwidth
    omega0:  float = 25.0    # ESO bandwidth (>= 5*omega_c)
    b0:      float = 10.0    # control gain estimate (~ keff_phys)


class TVCController:
    def __init__(self, kind: str, gains, dt: float):
        self.kind = kind
        self.g = gains
        self.dt = dt
        self._i = 0.0
        # ESO states (z1~theta, z2~rate, z3~disturbance/b0)
        self._z = np.zeros(3)
        self._init = False

    def reset(self):
        self._i = 0.0
        self._z = np.zeros(3)
        self._init = False

    def command(self, theta_meas: float, q_meas: float, theta_ref: float) -> float:
        """Returns gimbal command in DEGREES."""
        if self.kind == "pid":
            g: PIDGains = self.g
            e = theta_ref - theta_meas
            self._i += e * self.dt
            u = g.Kp * e + g.Kd * (-q_meas) + g.Ki * self._i
            return np.rad2deg(u)        # gains are in (rad gimbal)/(rad error); convert to deg
        elif self.kind == "adrc":
            return self._adrc(theta_meas, q_meas, theta_ref)
        else:
            raise ValueError(self.kind)

    def _adrc(self, theta_meas, q_meas, theta_ref):
        g: ADRCGains = self.g
        dt = self.dt
        if not self._init:
            self._z = np.array([theta_meas, q_meas, 0.0])
            self._init = True
        z1, z2, z3 = self._z
        # linear ESO with observer gains from bandwidth omega0
        b1 = 3 * g.omega0
        b2 = 3 * g.omega0 ** 2
        b3 = g.omega0 ** 3
        # last applied control (recompute from law, pre-saturation estimate)
        kp = g.omega_c ** 2
        kd = 2 * g.omega_c
        u0 = kp * (theta_ref - z1) - kd * z2
        u = (u0 - z3) / g.b0
        # propagate observer with the control we are about to issue
        e = theta_meas - z1
        z1d = z2 + b1 * e
        z2d = z3 + g.b0 * u + b2 * e
        z3d = b3 * e
        self._z = np.array([z1 + z1d * dt, z2 + z2d * dt, z3 + z3d * dt])
        return np.rad2deg(u)


# --------------------------------------------------------------------- scheduler

@dataclass
class SchedulerCfg:
    mode: str = "switched"        # fixed_stable | fixed_unstable | switched
    sm_stable: float = 0.8        # cal (stable)
    sm_unstable: float = -0.3     # cal (unstable, fling)
    switch_in_deg: float = 6.0    # go stable when |err| < this (catch zone)
    switch_out_deg: float = 10.0  # go unstable when |err| > this (fling zone) -- hysteresis
    settle_guard: bool = True     # force stable once nearly settled, regardless of hysteresis


class StabilityScheduler:
    def __init__(self, cfg: SchedulerCfg):
        self.cfg = cfg
        self._state = "stable"     # current commanded regime (for hysteresis)

    def reset(self):
        self._state = "stable"

    def command(self, theta_meas: float, q_meas: float, theta_ref: float) -> float:
        c = self.cfg
        if c.mode == "fixed_stable":
            return c.sm_stable
        if c.mode == "fixed_unstable":
            return c.sm_unstable
        # switched, with hysteresis on attitude error
        err_deg = abs(np.rad2deg(theta_ref - theta_meas))
        if self._state == "stable":
            if err_deg > c.switch_out_deg:
                self._state = "unstable"
        else:  # currently unstable (flinging)
            if err_deg < c.switch_in_deg:
                self._state = "stable"
        # settle guard: if very close and slow, force stable to avoid limit-cycling unstable
        if c.settle_guard and err_deg < c.switch_in_deg and abs(np.rad2deg(q_meas)) < 30:
            self._state = "stable"
        return c.sm_stable if self._state == "stable" else c.sm_unstable
