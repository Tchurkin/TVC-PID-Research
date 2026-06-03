"""
sensor_model.py — IMU / gyro sensor model for TVC rocket.

Models a single-axis MEMS gyro in pure-integration mode (no tilt aid during burn).
The controller receives delayed, noisy, biased, quantized estimates of (θ, θ̇).

Processing chain per timestep
───────────────────────────────────────────────────────────────────────────
  TRUE STATE        → q_true, theta_true
  gyro noise        + white noise (σ_noise) + bias random walk
  quantization      round to nearest gyro LSB
  bias correction   subtract pre-flight bias estimate
  integration       θ_meas += dt × q_corrected  (pure gyro, no tilt correction)
  latency buffer    FIFO delay of `latency_steps` samples
───────────────────────────────────────────────────────────────────────────

Outputs delivered to controller
  q_meas    : rate measurement (rad/s) — delayed by latency_steps
  theta_meas: angle measurement (rad)  — delayed by latency_steps

Noise parameters
  gyro_noise_std   : white noise (rad/s per sqrt step) — added each timestep
  gyro_bias_init   : 1-sigma of initial gyro bias (rad/s); Gaussian draw at t=0
  gyro_bias_rw     : bias random walk coefficient (rad/s/sqrt(s))
  gyro_quant_lsb   : quantisation step size (rad/s); 0 to disable
  bias_cal_s       : unused (pre-flight cal window — bias drawn once at init)
  bias_cal_residual: fraction of initial bias surviving calibration (0.10 = 90% removed)

Latency buffer
  Integer FIFO of depth `latency_steps`.  At each step the current measurement
  enters the front and the controller reads the back (oldest sample).
  latency_steps = 1 → controller sees the measurement from one step ago.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from collections import deque
import numpy as np


@dataclass
class SensorParams:
    gyro_noise_std: float       # white noise sigma (rad/s)
    gyro_bias_init: float       # initial bias 1-sigma (rad/s)
    gyro_bias_rw: float         # bias random walk (rad/s / sqrt(s))
    gyro_quant_lsb: float       # quantisation LSB (rad/s); 0 to disable
    bias_cal_residual: float    # fraction of bias not removed by calibration [0,1]
    sensor_latency_steps: int   # integer sample delay (≥ 1)


@dataclass
class SensorState:
    """Mutable state for the sensor model.  Initialise before the loop."""
    gyro_bias_true: float       # current true bias (rad/s); drawn from N(0, bias_init) at t=0
    bias_estimate: float        # pre-flight estimate applied as offset
    theta_integrated: float     # pure-gyro integrated angle (rad)
    lat_buf_q: deque            # latency FIFO for q_meas
    lat_buf_theta: deque        # latency FIFO for theta_meas


def init_sensor_state(
    params: SensorParams,
    theta0: float,
    rng: np.random.Generator,
) -> SensorState:
    """
    Draw initial sensor state from distributions.  Call once before the sim loop.

    The pre-flight calibration removes (1 - bias_cal_residual) of the true bias.
    The residual fraction survives into flight.
    """
    bias_true = params.gyro_bias_init * rng.standard_normal()
    bias_est = (1.0 - params.bias_cal_residual) * bias_true

    lat = max(1, params.sensor_latency_steps)
    # Fill both FIFOs with the known initial measurement
    return SensorState(
        gyro_bias_true=bias_true,
        bias_estimate=bias_est,
        theta_integrated=theta0,
        lat_buf_q=deque([0.0] * lat, maxlen=lat),
        lat_buf_theta=deque([theta0] * lat, maxlen=lat),
    )


def step_sensor(
    state: SensorState,
    params: SensorParams,
    q_true: float,
    theta_true: float,
    dt: float,
    rng: np.random.Generator,
) -> tuple[float, float]:
    """
    Advance sensor state by one timestep.  Mutates `state` in place.

    Parameters
    ----------
    state      : sensor state (mutated)
    params     : sensor parameters (constant)
    q_true     : true pitch rate (rad/s)
    theta_true : true pitch angle (rad)  — used only for initial state seed
    dt         : timestep (s)
    rng        : random number generator (shared with simulator)

    Returns
    -------
    (q_ctrl, theta_ctrl) — delayed measurements delivered to the controller (rad/s, rad)
    """
    # ── Bias random walk ─────────────────────────────────────────────────────
    state.gyro_bias_true += params.gyro_bias_rw * np.sqrt(dt) * rng.standard_normal()

    # ── Raw gyro output ──────────────────────────────────────────────────────
    q_raw = (
        q_true
        + state.gyro_bias_true
        + params.gyro_noise_std * rng.standard_normal()
    )

    # ── Quantisation ─────────────────────────────────────────────────────────
    if params.gyro_quant_lsb > 0:
        q_raw = params.gyro_quant_lsb * np.round(q_raw / params.gyro_quant_lsb)

    # ── Bias correction ──────────────────────────────────────────────────────
    q_corrected = q_raw - state.bias_estimate

    # ── Pure-gyro angle integration ──────────────────────────────────────────
    # No tilt correction during the burn (no accelerometer aiding).
    state.theta_integrated += dt * q_corrected

    # ── Latency buffer (FIFO) ────────────────────────────────────────────────
    # appendleft pushes new sample to front; the oldest sample is at the right end.
    state.lat_buf_q.appendleft(q_corrected)
    state.lat_buf_theta.appendleft(state.theta_integrated)

    # Controller reads the oldest (most delayed) sample
    q_ctrl = state.lat_buf_q[-1]
    theta_ctrl = state.lat_buf_theta[-1]

    return q_ctrl, theta_ctrl
