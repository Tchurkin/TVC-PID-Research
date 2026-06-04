"""
fidelity_config.py — Independent fidelity module toggle system.

Physical fidelity modules (new as of 2026-06-04)
──────────────────────────────────────────────────
  nonlinear_aero  Replace theta with sin(theta) in aerodynamic pitching moment.
                  Removes small-angle approximation; important for |theta| > 20 deg.
                  Effect: slightly more accurate trajectories near 20–60 deg excursions.

  dyn_aero        Dynamic pressure q_dyn varies with flight speed.
                  When on: velocity is integrated alongside pitch dynamics using
                  T(t), drag, and gravity; q_dyn = 0.5*rho*v² changes in real-time.
                  When off: constant q_dyn at a reference value (current simple mode).
                  Effect: early flight has understated aerodynamic moments (low v);
                  late flight has increased aerodynamic moments (high v).

  thrust_curve    Realistic time-varying thrust T(t) from motor_model.py.
                  When on: T follows an F15-class profile (rise, plateau, taper).
                  When off: constant T at design-space 'thrust' parameter value.
                  Effect: early spike then taper changes control authority envelope.

  cg_shift        CG and I_yy evolve as propellant burns.
                  When on: CG moves forward ~1-3 mm over 3 s burn (F-class).
                  When off: fixed CG and I_yy from design parameters.
                  Effect: small for F-class; larger for G/H class motors.
                  Also reduces instability rate p toward end of burn.


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
  deadband     Servo stiction — per-step movement below threshold produces no motion

  nonlinear_aero  sin(theta) in aero moment — removes small-angle approximation
  dyn_aero        time-varying q_dyn from velocity integration (T, drag, gravity)
  thrust_curve    realistic F15 thrust profile vs constant average thrust
  cg_shift        CG and I_yy evolution from propellant burn

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
    # Physical fidelity modules (new 2026-06-04) — ablatable from full()
    "nonlinear_aero",
    "dyn_aero",
    "thrust_curve",
    "cg_shift",
]

# Original 7 modules only — for backward-compatible ablation against legacy_full()
ABLATION_MODULES_LEGACY: list[str] = [
    "wind", "backlash", "slew", "latency", "sensor_noise", "thrust_var", "deadband",
]


@dataclass
class FidelityConfig:
    """
    Bitmask-style fidelity configuration.  Each flag represents one physics module.
    True = module active (realistic).  False = module removed (idealised).

    Do not modify fields directly — use .ablate() to get a new config.
    """
    wind:           bool = True
    backlash:       bool = True
    slew:           bool = True
    latency:        bool = True
    sensor_noise:   bool = True
    thrust_var:     bool = True
    deadband:       bool = True
    # Physical fidelity modules (new 2026-06-04)
    nonlinear_aero: bool = True   # sin(theta) vs theta in aero moment
    dyn_aero:       bool = True   # time-varying q_dyn from velocity integration
    thrust_curve:   bool = True   # realistic T(t) profile vs constant thrust
    cg_shift:       bool = True   # evolving CG/I_yy from propellant burn

    # ── Named constructors ─────────────────────────────────────────────────
    @classmethod
    def full(cls) -> "FidelityConfig":
        """All physics modules active.  Highest fidelity (includes new physical modules)."""
        return cls()

    @classmethod
    def simple(cls) -> "FidelityConfig":
        """All physics modules removed.  Idealized baseline (L0-equivalent)."""
        return cls(
            wind=False, backlash=False, slew=False,
            latency=False, sensor_noise=False, thrust_var=False, deadband=False,
            nonlinear_aero=False, dyn_aero=False, thrust_curve=False, cg_shift=False,
        )

    @classmethod
    def legacy_full(cls) -> "FidelityConfig":
        """
        Full fidelity matching the pre-2026-06-04 simulator (7 original modules).
        New physical modules are OFF — use for comparison with old Exp1/Exp4 results.
        """
        return cls(
            wind=True, backlash=True, slew=True,
            latency=True, sensor_noise=True, thrust_var=True, deadband=True,
            nonlinear_aero=False, dyn_aero=False, thrust_curve=False, cg_shift=False,
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
    # Removes stiction effect.  Servo responds to arbitrarily small per-step movements.
    if not cfg.deadband:
        act.deadband = 0.0

    # ── Physical fidelity flags: stored on ScenarioParams for simulator routing ──
    # These are read by simulator.py to select simple vs. full physics mode.
    # We attach them as dynamic attributes since ScenarioParams is a dataclass;
    # simulator.py uses getattr(scenario, flag, False) to read them safely.
    sc.use_nonlinear_aero = cfg.nonlinear_aero  # sin(theta) in aero moment
    sc.use_dyn_aero       = cfg.dyn_aero        # time-varying q_dyn
    sc.use_thrust_curve   = cfg.thrust_curve     # realistic T(t) profile
    sc.use_cg_shift       = cfg.cg_shift         # CG/I_yy evolution

    return act, sen, dis, sc
