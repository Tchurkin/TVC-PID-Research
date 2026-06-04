"""
disturbance_model.py — External disturbance torques on the TVC rocket.

Two components, always additive:
  1. DETERMINISTIC  sinusoidal torque disturbance (models launch rail vibration
                    or aerodynamic buffet at a fixed frequency)
  2. STOCHASTIC     first-order Dryden-like atmospheric turbulence (AR(1) process)

Both are angular acceleration disturbances [rad/s²] added to the plant dynamics.
They are independent of the controller and plant state.

Stochastic gust model
─────────────────────
  Discrete AR(1) / Ornstein-Uhlenbeck approximation to Dryden turbulence:
      g[k] = α · g[k-1] + σ_step · ε[k],   ε ~ N(0,1) i.i.d.
      α = exp(-dt / τ_gust)
      σ_step = gust_std · sqrt(1 - α²)

  Steady-state variance:  E[g²] = σ_step² / (1 - α²) = gust_std²
  So gust_std is the steady-state RMS gust angular acceleration (rad/s²).

Effective disturbance scaling
──────────────────────────────
  The combined disturbance amplitude is scaled by the plant's inertia and mass
  to model rockets with different moments of inertia and response to wind loads:
      d_eff = d_raw × inertia_scale / max(0.5, mass_scale)
  This is a modeling approximation (see units.py disclosure #3).
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class DisturbanceParams:
    det_amp: float      # sinusoidal disturbance amplitude (rad/s²)
    det_freq_hz: float  # sinusoidal disturbance frequency (Hz)
    gust_std: float     # steady-state RMS of stochastic gust (rad/s²); 0 to disable
    gust_tau: float     # gust correlation time constant (s)


@dataclass
class GustState:
    """Mutable gust state.  One float — the current AR(1) value."""
    g: float = 0.0


def init_gust_state(
    params: DisturbanceParams | None = None,
    rng: np.random.Generator | None = None,
) -> GustState:
    """
    Initialise gust state from stationary distribution.

    The AR(1) gust process has stationary distribution N(0, gust_std).
    Starting at g=0 causes the first ~3τ seconds (≈1.2 s for τ=0.4 s) to
    have understated disturbance, making the simulator systematically optimistic
    in the early phase of flight.

    The correct initialisation draws from the stationary distribution so that
    the simulation begins in a statistically representative gust environment
    from t=0.

    If params or rng are not provided, falls back to g=0 (backward compatible).
    """
    if params is not None and rng is not None and params.gust_std > 0:
        g0 = float(params.gust_std * rng.standard_normal())
    else:
        g0 = 0.0   # backward compatible: caller does not provide params/rng
    return GustState(g=g0)


def step_disturbance(
    gust: GustState,
    params: DisturbanceParams,
    t: float,
    dt: float,
    rng: np.random.Generator,
    inertia_scale: float = 1.0,
    mass_scale: float = 1.0,
) -> float:
    """
    Advance gust state and return the total effective disturbance angular acceleration.
    Mutates `gust` in place.

    Parameters
    ----------
    gust          : current gust state (mutated)
    params        : disturbance parameters (constant)
    t             : current time (s)
    dt            : timestep (s)
    rng           : random number generator (shared with simulator)
    inertia_scale : plant's inertia_scale (Iyy_ref / Iyy) for scaling
    mass_scale    : plant's mass_scale (m / m_ref) for scaling

    Returns
    -------
    d_eff (rad/s²) — disturbance angular acceleration added to qdot
    """
    # ── Update stochastic gust ────────────────────────────────────────────────
    if params.gust_std > 0:
        alpha = np.exp(-dt / max(1e-3, params.gust_tau))
        sigma_step = params.gust_std * np.sqrt(1.0 - alpha ** 2)
        gust.g = alpha * gust.g + sigma_step * rng.standard_normal()
    else:
        gust.g = 0.0

    # ── Deterministic sinusoidal component ────────────────────────────────────
    d_det = params.det_amp * np.sin(2.0 * np.pi * params.det_freq_hz * t)

    # ── Combined and scaled ──────────────────────────────────────────────────
    d_raw = d_det + gust.g
    d_eff = d_raw * inertia_scale / max(0.5, mass_scale)

    return d_eff
