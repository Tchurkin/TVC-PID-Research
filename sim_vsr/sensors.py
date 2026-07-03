"""
sim_vsr.sensors -- IMU with bias, white noise, and control-loop latency (FIFO).

latency_steps is the tau in Pi = keff * tau^2: the number of control steps between a true
state and the controller seeing it (raw-IMU-read pipeline). A pure integer FIFO is the
bare-metal hobby case; this can be extended to jitter / IIR-smoothed models later.
"""
from __future__ import annotations
from dataclasses import dataclass
from collections import deque
import numpy as np


@dataclass
class IMUParams:
    gyro_noise_dps:  float = 0.5     # white noise std on pitch rate
    gyro_bias_dps:   float = 0.2     # constant bias std (drawn once per flight)
    theta_noise_deg: float = 0.3     # attitude-estimate noise std
    latency_steps:   int = 2         # FIFO delay (controller sees state from L steps ago)


class IMU:
    def __init__(self, p: IMUParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.bias = np.deg2rad(p.gyro_bias_dps) * rng.standard_normal()
        self.buf: deque = deque()

    def measure(self, theta: float, q: float):
        """Returns delayed (theta_meas, q_meas) in rad, rad/s."""
        p = self.p
        q_m = q + self.bias + np.deg2rad(p.gyro_noise_dps) * self.rng.standard_normal()
        th_m = theta + np.deg2rad(p.theta_noise_deg) * self.rng.standard_normal()
        self.buf.append((th_m, q_m))
        if len(self.buf) > p.latency_steps:
            return self.buf.popleft()
        return self.buf[0]
