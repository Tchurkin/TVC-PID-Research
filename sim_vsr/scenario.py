"""
sim_vsr.scenario -- mission profile: thrust curve, attitude command schedule, wind.

A Scenario bundles everything that is not the vehicle or the controller: how long to fly,
the motor thrust(t), the commanded pitch schedule theta_ref(t) (the maneuver), and the wind
disturbance. thrust_F_class is a simple Estes-F-style curve; swap in a real .eng curve later.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable, Optional
import numpy as np


def thrust_F_class(t: float, burn: float = 2.2, avg: float = 14.0, peak: float = 26.0) -> float:
    """Crude F-class thrust [N]: brief ignition spike, then a sustained plateau, then tail-off."""
    if t < 0 or t > burn:
        return 0.0
    if t < 0.15:                      # ignition spike
        return peak * (t / 0.15)
    if t < 0.35:                      # decay from peak to plateau
        return peak + (avg - peak) * ((t - 0.15) / 0.20)
    if t < burn - 0.3:                # plateau
        return avg
    return avg * max(0.0, (burn - t) / 0.3)   # tail-off


class WindOU:
    """Ornstein-Uhlenbeck colored wind -> lateral gust velocity [m/s]. Effective AoA = atan(w/V)."""

    def __init__(self, sigma: float = 2.0, tau: float = 0.4, rng: Optional[np.random.Generator] = None):
        self.sigma = sigma
        self.tau = tau
        self.rng = rng or np.random.default_rng(0)
        self.w = 0.0

    def step(self, dt: float) -> float:
        a = np.exp(-dt / self.tau)
        self.w = a * self.w + np.sqrt(max(0.0, 1 - a * a)) * self.sigma * self.rng.standard_normal()
        return self.w


@dataclass
class Scenario:
    t_end: float = 4.0
    dt: float = 0.005                                   # 200 Hz
    launch_speed: float = 15.0                          # rail-exit speed [m/s] (control begins here)
    thrust_fn: Callable[[float], float] = thrust_F_class
    # attitude command schedule: theta_ref(t) in radians-from-horizontal (pi/2 = up)
    maneuver_time: float = 1.0                          # when the pitch-over is commanded
    maneuver_deg: float = 30.0                          # commanded pitch-over from vertical
    maneuver_ramp: float = 0.4                          # s to ramp the setpoint (0 = hard step)
    wind_sigma: float = 2.0
    wind_tau: float = 0.4

    def theta_ref(self, t: float) -> float:
        """Ramped pitch-over: hold vertical, then slew the setpoint by maneuver_deg over maneuver_ramp.

        A real flight computer commands a rate-limited attitude setpoint, not a hard step --
        a step would demand a body rotation the velocity vector cannot follow, driving an
        unphysical transient angle of attack.
        """
        base = np.pi / 2
        if t < self.maneuver_time:
            return base
        if self.maneuver_ramp <= 0:
            return base - np.deg2rad(self.maneuver_deg)
        frac = min(1.0, (t - self.maneuver_time) / self.maneuver_ramp)
        return base - np.deg2rad(self.maneuver_deg) * frac
