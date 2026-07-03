"""
sim_vsr -- Variable-Stability TVC Rocket simulator.

A purpose-built, modular longitudinal-flight simulator for a thrust-vectored model
rocket that can ACTIVELY MODULATE its static margin in flight (morphing surface or
moving mass) to switch between stable and maneuverable configurations.

Modules
-------
  vehicle    : mass/inertia/thrust schedules, aerodynamics, planar EOM derivative
  actuators  : TVC gimbal (slew/deadband/backlash/saturation) + morph (static-margin) actuator
  sensors    : IMU (gyro/accel) with bias, noise, latency
  control    : TVC controller (PID / ADRC), stability scheduler (switching), allocation
  scenario   : mission profile (thrust curve, maneuver schedule, wind)
  simulate   : fixed-step integrator, SimResult, flight metrics

Conventions
-----------
  Planar (longitudinal) flight. Angles from HORIZONTAL: straight-up launch -> gamma=90deg.
  theta = body pitch (deg-from-horizontal), gamma = flight-path angle, alpha = theta-gamma.
  static_margin in CALIBERS (>0 stable: CP behind CG; <0 unstable).
  Gimbal/TVC sign: +delta pitches the nose up (+theta).
"""
from .vehicle import VehicleParams, CanardParams, mass_props, aero_forces, deriv, State
from .actuators import (TVCActuator, TVCParams, MorphActuator, MorphParams,
                        CanardActuator, CanardActParams)
from .sensors import IMU, IMUParams
from .control import TVCController, PIDGains, ADRCGains, StabilityScheduler, SchedulerCfg
from .scenario import Scenario, thrust_F_class, WindOU
from .simulate import simulate, SimResult

__all__ = [
    "VehicleParams", "CanardParams", "mass_props", "aero_forces", "deriv", "State",
    "TVCActuator", "TVCParams", "MorphActuator", "MorphParams",
    "CanardActuator", "CanardActParams",
    "IMU", "IMUParams", "TVCController", "PIDGains", "ADRCGains",
    "StabilityScheduler", "SchedulerCfg",
    "Scenario", "thrust_F_class", "WindOU", "simulate", "SimResult",
]
