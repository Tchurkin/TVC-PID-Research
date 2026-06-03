"""
fidelity_config.py — Independent fidelity module toggle system.

Replaces the cumulative fidelity ladder (L0→L5) with a bitmask-style
FidelityConfig where each physics module is an independent boolean flag.

Why this matters
────────────────
The ladder (L0→L5) adds physics cumulatively and answers:
  "What happens as the simulator becomes more realistic?"

The ablation (start at FULL, disable one module) answers:
  "What does each individual module contribute to the decision?"

These are inverses of each other.  Exp4 needs ablation.
Exp5 uses FULL fidelity (FidelityConfig.full()) with design parameters perturbed.

Module definitions
──────────────────
  wind         Dryden AR(1) gust + deterministic sinusoidal disturbance torque
  backlash     Symmetric hysteresis (KP backlash operator) in gimbal linkage
  slew         Maximum servo angular velocity; binds at 1.05 code_units/s for 75 deg/s
  latency      FIFO sensor pipeline delay (sensor_latency_steps > 1)
  sensor_noise Gyro white noise, bias init, bias random walk, LSB quantisation
  thrust_var   Mid-burn keff and aero_damp degradation fault at t=fault_time_s
  deadband     Servo stiction — position error below threshold produces no motion

Ablation convention
───────────────────
  For each module X: delta_metric_X = metric(FULL minus X) - metric(FULL)
  Sign: negative delta_rms_X means removing X improves RMS (X was hurting performance).
  A large |delta| means X is important for predicting this design's behaviour.

Full fidelity definition for Exp4 ablation
──────────────────────────────────────────
  FULL = all 7 modules on, including thrust_var fault at t=1.5s (mid-burn).
  This is more conservative than the Exp1 single-seed evaluation (which had no fault).
  Using FULL as baseline ensures every ablation is a strict relaxation.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field, fields

import numpy as np

from actuator_model import ActuatorParams
from sensor_model   import SensorParams
from disturbance_model import DisturbanceParams
from simulator      import ScenarioParams


# Ordered list of all ablatable modules — order determines display and output column order.
ABLATION_MODULES: list[str] = [
    "wind",
    "backlash",
    "slew",
    "latency",
    "sensor_noise",
    "thrust_var",
    "deadband",
]


@dataclass
class FidelityConfig:
    """
    Bitmask-style fidelity configuration.  Each flag represents one physics module.
    True = module active (realistic).  False = module removed (idealised).

    Do not modify fields directly — use .ablate() to get a new config.
    """
    wind:         bool = True
    backlash:     bool = True
    slew:         bool = True
    latency:      bool = True
    sensor_noise: bool = True
    thrust_var:   bool = True
    deadband:     bool = True

    # ── Named constructors ─────────────────────────────────────────────────
    @classmethod
    def full(cls) -> "FidelityConfig":
        """All physics modules active.  Highest fidelity."""
        return cls()

    @classmethod
    def simple(cls) -> "FidelityConfig":
        """All physics modules removed.  Idealized baseline (L0-equivalent)."""
        return cls(
            wind=False, backlash=False, slew=False,
            latency=False, sensor_noise=False, thrust_var=False, deadband=False,
        )

    @classmethod
    def from_flags(cls, **flags: bool) -> "FidelityConfig":
        """Create config from keyword flags; unspecified flags default to True."""
        cfg = cls()
        for k, v in flags.items():
            if not hasattr(cfg, k):
                raise ValueError(f"Unknown fidelity module: '{k}'")
            setattr(cfg, k, v)
        return cfg

    # ── Ablation helper ────────────────────────────────────────────────────
    def ablate(self, module: str) -> "FidelityConfig":
        """
        Return a copy of this config with one module disabled.
        Usage: FidelityConfig.full().ablate("backlash")
        """
        if module not in ABLATION_MODULES:
            raise ValueError(
                f"Unknown module '{module}'. Valid: {ABLATION_MODULES}"
            )
        cfg = copy.copy(self)
        setattr(cfg, module, False)
        return cfg

    # ── Introspection ──────────────────────────────────────────────────────
    def active_modules(self) -> list[str]:
        """Return names of all active (True) modules."""
        return [m for m in ABLATION_MODULES if getattr(self, m)]

    def inactive_modules(self) -> list[str]:
        """Return names of all disabled (False) modules."""
        return [m for m in ABLATION_MODULES if not getattr(self, m)]

    def __repr__(self) -> str:
        active = "+".join(self.active_modules()) or "NONE"
        return f"FidelityConfig({active})"


# ── Parameter application ─────────────────────────────────────────────────────

def apply_fidelity_config(
    base_act: ActuatorParams,
    base_sen: SensorParams,
    base_dis: DisturbanceParams,
    base_sc:  ScenarioParams,
    cfg:      FidelityConfig,
) -> tuple[ActuatorParams, SensorParams, DisturbanceParams, ScenarioParams]:
    """
    Return (act, sen, dis, sc) with physics modules toggled per cfg.
    Base objects are NOT mutated.  Returns shallow copies with modified fields.

    Parameters
    ----------
    base_act, base_sen, base_dis, base_sc : parameter objects built from design row
    cfg : FidelityConfig controlling which modules are active

    Returns
    -------
    (act, sen, dis, sc) — modified copies ready for simulate()
    """
    act = copy.copy(base_act)
    sen = copy.copy(base_sen)
    dis = copy.copy(base_dis)
    sc  = copy.copy(base_sc)

    # ── wind ─────────────────────────────────────────────────────────────────
    # Removes both stochastic gust (AR(1)) and deterministic sinusoidal disturbance.
    if not cfg.wind:
        dis.gust_std  = 0.0
        dis.det_amp   = 0.0

    # ── backlash ──────────────────────────────────────────────────────────────
    # Sets the mechanical play gap to zero.  The hysteresis operator becomes
    # a pass-through (u_out = u_servo at all times).
    if not cfg.backlash:
        act.backlash = 0.0

    # ── slew ──────────────────────────────────────────────────────────────────
    # Sets slew_max to a physically unreachable value.  The first-order lag
    # (tau_act = 0.05 s) remains the bandwidth-limiting element.
    if not cfg.slew:
        act.slew_max = 1e9   # code_units/s — effectively unlimited

    # ── latency ───────────────────────────────────────────────────────────────
    # Reduces sensor pipeline delay to 1 step (5 ms) — the minimum causal delay
    # in the simulation loop.  Cannot be set to 0 without restructuring causality.
    if not cfg.latency:
        sen.sensor_latency_steps = 1

    # ── sensor_noise ──────────────────────────────────────────────────────────
    # Removes all gyro error sources.  The sensor becomes a noiseless integrator.
    # Latency is controlled separately (latency flag above).
    if not cfg.sensor_noise:
        sen.gyro_noise_std    = 0.0
        sen.gyro_bias_init    = 0.0
        sen.gyro_bias_rw      = 0.0
        sen.gyro_quant_lsb    = 0.0
        sen.bias_cal_residual = 0.0

    # ── thrust_var ────────────────────────────────────────────────────────────
    # Removes mid-burn fault: keff and aero_damp degrade at fault_time_s (1.5 s).
    # With thrust_var off, the rocket's dynamics are stationary throughout the burn.
    if not cfg.thrust_var:
        sc.keff_fault_post  = 1.0
        sc.damp_fault_post  = 1.0
        sc.fault_time_s     = float("inf")

    # ── deadband ──────────────────────────────────────────────────────────────
    # Removes stiction effect.  Servo responds to arbitrarily small position errors.
    if not cfg.deadband:
        act.deadband = 0.0

    return act, sen, dis, sc
