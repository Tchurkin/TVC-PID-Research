"""
sim_vsr.actuators -- TVC gimbal + morph (static-margin) actuators.

TVCActuator mirrors the validated sim/actuator_model.py physics: saturation (max gimbal),
slew-rate limit (the servo speed that drives the Pi=keff*tau^2 saturation regime), deadband,
and backlash. slew_sat_frac is the fsat diagnostic used throughout the project.

MorphActuator models the variable-stability mechanism (a morphing surface or moving mass that
shifts CP/CG): a rate-limited, range-bounded static-margin command. rate_cal_s is the binding
hardware parameter -- how fast the airframe can flip between stable and unstable.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class TVCParams:
    u_max_deg:      float = 7.0      # max gimbal deflection (saturation)
    slew_max_deg_s: float = 120.0    # servo slew rate (drives Pi saturation regime)
    deadband_deg:   float = 0.0
    backlash_deg:   float = 0.0


class TVCActuator:
    """Command (deg) -> actual gimbal (rad), with slew/deadband/backlash/saturation."""

    def __init__(self, p: TVCParams):
        self.p = p
        self.pos = 0.0        # commanded shaft angle (deg), pre-backlash
        self.out = 0.0        # output angle through backlash band (deg)
        self._sat = 0
        self._n = 0

    def step(self, cmd_deg: float, dt: float) -> float:
        p = self.p
        self._n += 1
        cmd = float(np.clip(cmd_deg, -p.u_max_deg, p.u_max_deg))
        want = cmd - self.pos
        if abs(want) < p.deadband_deg:
            want = 0.0
        max_step = p.slew_max_deg_s * dt
        if abs(want) > max_step:           # slew-rate saturation
            want = np.sign(want) * max_step
            self._sat += 1
        self.pos += want
        out = self.pos
        if p.backlash_deg > 0.0:           # lost motion within a band
            half = p.backlash_deg / 2.0
            if out > self.out + half:
                self.out = out - half
            elif out < self.out - half:
                self.out = out + half
            out = self.out
        return np.deg2rad(out)

    @property
    def slew_sat_frac(self) -> float:
        return self._sat / max(1, self._n)


@dataclass
class MorphParams:
    sm_min:     float = -0.5     # most-unstable static margin (cal)
    sm_max:     float = 1.0      # most-stable static margin (cal)
    rate_cal_s: float = 15.0     # max |d sm/dt| -- mechanism morph speed (the bottleneck)


class MorphActuator:
    """Static-margin command (cal) -> actual static margin (cal), rate + range limited."""

    def __init__(self, p: MorphParams, sm0: float):
        self.p = p
        self.sm = float(np.clip(sm0, p.sm_min, p.sm_max))
        self._n = 0
        self._rate_lim = 0

    def step(self, sm_cmd: float, dt: float) -> float:
        p = self.p
        self._n += 1
        sm_cmd = float(np.clip(sm_cmd, p.sm_min, p.sm_max))
        want = sm_cmd - self.sm
        max_step = p.rate_cal_s * dt
        if abs(want) > max_step:
            want = np.sign(want) * max_step
            self._rate_lim += 1
        self.sm += want
        return self.sm

    @property
    def rate_limited_frac(self) -> float:
        return self._rate_lim / max(1, self._n)


@dataclass
class CanardActParams:
    deploy_rate_s:   float = 8.0     # 1/s: full fold-out (0->1) takes 1/deploy_rate seconds
    defl_max_deg:    float = 20.0    # max canard deflection
    defl_slew_deg_s: float = 300.0   # canard servo slew rate (faster, smaller surface than TVC)


class CanardActuator:
    """Commands (deploy in [0,1], deflection deg) -> (deploy_frac, deflection rad), rate limited.

    deploy_frac is the fold-out state (0 folded, 1 fully out); deflection is the canard angle
    for raw control. Margin-modulation uses deploy with defl=0; raw control holds deploy=1 and
    drives defl. A single servo can do both, but margin-modulation needs only binary deploy.
    """

    def __init__(self, p: CanardActParams):
        self.p = p
        self.deploy = 0.0
        self.defl = 0.0          # deg
        self._sat = 0
        self._n = 0

    def step(self, deploy_cmd: float, defl_cmd_deg: float, dt: float):
        p = self.p
        self._n += 1
        deploy_cmd = float(np.clip(deploy_cmd, 0.0, 1.0))
        self.deploy += np.clip(deploy_cmd - self.deploy, -p.deploy_rate_s * dt, p.deploy_rate_s * dt)
        defl_cmd = float(np.clip(defl_cmd_deg, -p.defl_max_deg, p.defl_max_deg))
        want = defl_cmd - self.defl
        max_step = p.defl_slew_deg_s * dt
        if abs(want) > max_step:
            want = np.sign(want) * max_step
            self._sat += 1
        self.defl += want
        return self.deploy, np.deg2rad(self.defl)

    @property
    def defl_slew_sat_frac(self) -> float:
        return self._sat / max(1, self._n)
