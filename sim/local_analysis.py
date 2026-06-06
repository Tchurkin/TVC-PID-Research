"""
local_analysis.py — Local design-space analysis tools for Exp5.

Provides:
  - Central finite-difference gradient estimation of performance metrics
  - Local curvature (diagonal Hessian) for topology classification
  - Topology classifier (plateau / ridge / cliff / bowl)
  - Gradient-descent evolution path through design space
  - Neighborhood sampler for local LHS around a design point

Scientific framing
──────────────────
Treats the simulator as a black-box function f(x) → performance, where x is
the design vector and performance is a scalar metric (rms_error_deg, etc.).

The gradient ∇f(x) at each design point answers:
  "Which design dimension, if improved, would most reduce this metric?"

The curvature d²f/dx_i² answers:
  "Is this parameter near a diminishing-returns plateau?"

The topology class answers:
  "Is this design stuck on a plateau, near a cliff edge, on a ridge,
   or in a well-conditioned bowl?"

Units and step sizes
────────────────────
  Gradients are in [metric_unit / param_unit], e.g., [deg / (deg/s)] for
  d(rms_deg)/d(servo_slew_deg_s).  Step sizes are 5% of design range for
  continuous parameters; 1 step for integer latency_steps.

  Gradients are NOT normalised within this module.  Use the normalised
  improvement direction (in run_exp5_landscape) for ranking across parameters.

Gain-freezing rationale
────────────────────────
  Gains are frozen at Exp1-tuned (best_Kp, best_Kd).  The local landscape
  measures how sensitive PHYSICAL performance is to design changes, not
  how well the controller adapts.  Re-tuning at each perturbation point
  would measure controller robustness, which is Exp1's job.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

from design_space import (
    DESIGN_NAMES, DESIGN_LO, DESIGN_HI,
    build_plant, build_actuator, build_sensor, build_disturbance, build_scenario,
    estimate_p_unstable, REF,
)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from simulator import simulate, SimResult


# ── Gradient parameter set ────────────────────────────────────────────────────
# All continuous design parameters.  p_unstable is excluded (derived from others).
# latency_steps handled separately (integer arithmetic).

GRAD_PARAMS_CONTINUOUS = [
    "mass", "Iyy", "static_margin", "Cm_alpha",
    "control_effectiveness", "motor_scale",
    "servo_slew_deg_s", "max_gimbal_deg",
    "deadband", "backlash", "wind_strength",
]
GRAD_PARAMS_INTEGER = ["latency_steps"]
GRAD_PARAMS_ALL     = GRAD_PARAMS_CONTINUOUS + GRAD_PARAMS_INTEGER

# Step sizes: 5% of design range for each parameter
_lo = dict(zip(DESIGN_NAMES, DESIGN_LO))
_hi = dict(zip(DESIGN_NAMES, DESIGN_HI))
PARAM_STEP: dict[str, float] = {
    p: 0.05 * (_hi[p] - _lo[p]) for p in DESIGN_NAMES if p in GRAD_PARAMS_CONTINUOUS
}
PARAM_STEP["latency_steps"] = 1.0   # integer: ±1 step

# Performance metrics available for gradient computation
PERF_METRICS = [
    "rms_error_deg",
    "end_error_deg",
    "max_theta_deg",
    "peak_error_deg",
    "u_cmd_sat_frac",
    "slew_sat_frac",
]


# ── Core single-design evaluator ──────────────────────────────────────────────

def _evaluate_design(
    design: dict,
    Kp: float,
    Kd: float,
    cfg: FidelityConfig,
    seed: int = 1,
) -> SimResult:
    """
    Run one simulation for a design row, frozen gains, and fidelity config.
    Returns the raw SimResult.
    """
    plant    = build_plant(design)
    base_act = build_actuator(design)
    base_sen = build_sensor(design)
    base_dis = build_disturbance(design)
    base_sc  = build_scenario(theta0_bias_std=0.0)

    act, sen, dis, sc = apply_fidelity_config(base_act, base_sen, base_dis, base_sc, cfg)
    pid = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=base_act.u_max, i_lim=base_act.u_max)
    return simulate(pid, plant, act, sen, dis, sc, seed=seed)


def _metric_from_result(r: SimResult, metric: str) -> float:
    return float(getattr(r, metric))


def _perturbed_design(base: dict, param: str, delta: float) -> dict:
    """
    Return a copy of the design row with one parameter perturbed by delta.
    Clamps to design bounds.  Recomputes p_unstable if base parameters change.
    """
    d = dict(base)
    raw = float(d.get(param, REF.get(param, 0.0))) + delta
    lo  = float(_lo.get(param, -1e9))
    hi  = float(_hi.get(param, 1e9))
    d[param] = np.clip(raw, lo, hi)

    # Recompute derived variables
    if param in ("static_margin", "Cm_alpha", "motor_scale"):
        d["p_unstable"] = estimate_p_unstable(
            d.get("static_margin", REF["static_margin"]),
            d.get("Cm_alpha",      REF["Cm_alpha"]),
            d.get("motor_scale",   REF["motor_scale"]),
        )
    return d


# ── Gradient estimation (central finite differences) ─────────────────────────

def compute_gradient(
    design: dict,
    Kp: float,
    Kd: float,
    metric: str = "rms_error_deg",
    cfg: FidelityConfig | None = None,
    seeds: tuple[int, ...] = (1,),
) -> dict[str, float]:
    """
    Estimate d(metric)/d(param) for all GRAD_PARAMS via central differences.

    For continuous params: grad_i = [f(x + h_i) - f(x - h_i)] / (2 h_i)
    For latency_steps:     grad_i = [f(x+1) - f(x-1)] / 2  (integer step)

    Parameters
    ----------
    design : design row dict (must contain all GRAD_PARAMS_ALL keys)
    Kp, Kd : frozen PID gains from Exp1
    metric : SimResult attribute to differentiate
    cfg    : FidelityConfig (default: FidelityConfig.full())
    seeds  : tuple of seeds to average over (reduces RNG noise)

    Returns
    -------
    gradient : dict {param_name: float gradient value}
    """
    if cfg is None:
        cfg = FidelityConfig.full()

    def avg_metric(d: dict) -> float:
        return float(np.mean([
            _metric_from_result(_evaluate_design(d, Kp, Kd, cfg, seed=s), metric)
            for s in seeds
        ]))

    gradient: dict[str, float] = {}

    for param in GRAD_PARAMS_CONTINUOUS:
        step = PARAM_STEP[param]
        d_plus  = _perturbed_design(design, param, +step)
        d_minus = _perturbed_design(design, param, -step)

        # Check if boundary clamping made the steps asymmetric
        actual_plus  = float(d_plus[param])  - float(design.get(param, REF.get(param, 0)))
        actual_minus = float(design.get(param, REF.get(param, 0))) - float(d_minus[param])
        denom = actual_plus + actual_minus   # total span, not 2×step if clamped

        if denom < 1e-12:
            # Parameter is pinned at boundary; gradient undefined in this direction
            gradient[param] = 0.0
        else:
            f_plus  = avg_metric(d_plus)
            f_minus = avg_metric(d_minus)
            gradient[param] = (f_plus - f_minus) / denom

    for param in GRAD_PARAMS_INTEGER:
        step = 1
        base_val = int(design.get(param, REF.get(param, 1)))
        d_plus  = dict(design)
        d_minus = dict(design)
        d_plus[param]  = int(np.clip(base_val + step, _lo.get(param, 1), _hi.get(param, 6)))
        d_minus[param] = int(np.clip(base_val - step, _lo.get(param, 1), _hi.get(param, 6)))

        denom = d_plus[param] - d_minus[param]
        if denom == 0:
            gradient[param] = 0.0
        else:
            gradient[param] = (avg_metric(d_plus) - avg_metric(d_minus)) / denom

    return gradient


def compute_curvature(
    design: dict,
    Kp: float,
    Kd: float,
    metric: str = "rms_error_deg",
    cfg: FidelityConfig | None = None,
    seed: int = 1,
) -> dict[str, float]:
    """
    Estimate diagonal Hessian (d²metric / d(param)²) for continuous params.
    Uses second-order central differences: [f(x+h) - 2f(x) + f(x-h)] / h²

    Positive curvature: metric curves upward (diminishing returns from here).
    Negative curvature: metric curves downward (accelerating improvement).
    Near-zero curvature: locally linear — gradient is a reliable predictor.
    """
    if cfg is None:
        cfg = FidelityConfig.full()

    f_center = _metric_from_result(_evaluate_design(design, Kp, Kd, cfg, seed), metric)

    curvature: dict[str, float] = {}
    for param in GRAD_PARAMS_CONTINUOUS:
        step = PARAM_STEP[param]
        d_plus  = _perturbed_design(design, param, +step)
        d_minus = _perturbed_design(design, param, -step)

        actual_h = min(
            abs(float(d_plus[param])  - float(design.get(param, REF.get(param, 0)))),
            abs(float(design.get(param, REF.get(param, 0))) - float(d_minus[param])),
        )
        if actual_h < 1e-12:
            curvature[param] = 0.0
            continue

        f_plus  = _metric_from_result(_evaluate_design(d_plus,  Kp, Kd, cfg, seed), metric)
        f_minus = _metric_from_result(_evaluate_design(d_minus, Kp, Kd, cfg, seed), metric)
        curvature[param] = (f_plus - 2.0 * f_center + f_minus) / (actual_h ** 2)

    return curvature


# ── Topology classification ───────────────────────────────────────────────────

def classify_topology(
    gradient: dict[str, float],
    curvature: dict[str, float],
    plateau_pct: float = 10.0,
    ridge_concentration_thr: float = 0.60,
    cliff_ratio_thr: float = 5.0,
    cliff_abs_min: float = 0.05,
    _population_grad_mag: float | None = None,
) -> str:
    """
    Classify local topology based on gradient and curvature.

    Classes (assuming minimisation of a cost metric like rms_error_deg)
    ──────────────────────────────────────────────────────────────────
      plateau   Gradient magnitude small relative to population.
                Design is already near a local minimum or in a flat basin.
                Marginal improvements available in any direction.

      ridge     One direction dominates (concentration > threshold).
                One parameter is the binding constraint.  Improving it
                gives large gains; others give diminishing returns.

      cliff     One direction is extremely steep relative to others,
                AND the max gradient is absolutely significant (not just
                a noise spike in an otherwise flat region).
                Design is near an instability boundary.  Small worsening
                in one parameter causes large performance degradation.

      bowl      Well-distributed gradient.  Multiple parameters contribute
                roughly equally.  Improvement is available and well-conditioned.

    Parameters
    ----------
    gradient   : from compute_gradient()
    curvature  : from compute_curvature()
    plateau_pct : percentile threshold for "small" gradient (applied if
                  _population_grad_mag is not provided, treated as absolute)
    cliff_abs_min : minimum range-scaled max-gradient for cliff classification.
                    Guards against stochastic noise spikes in near-flat regions
                    being misclassified as cliffs.  0.05 = 5% of design range
                    per full range of the metric.
    _population_grad_mag : if provided, plateau threshold = plateau_pct percentile
                           of this value (pass population distribution).
    """
    params  = [p for p in GRAD_PARAMS_CONTINUOUS if p in gradient]
    # Use range-scaled gradient for all topology checks so unit bias is removed.
    scaled  = range_scaled_gradient({p: gradient[p] for p in params})
    g_vec   = np.array([scaled[p] for p in params])
    g_mag   = float(np.linalg.norm(g_vec))
    g_abs   = np.abs(g_vec)
    g_sum   = g_abs.sum()

    # Plateau threshold
    if _population_grad_mag is not None:
        thresh = float(np.percentile(_population_grad_mag, plateau_pct))
    else:
        thresh = plateau_pct   # treat as absolute threshold

    if g_mag < thresh:
        return "plateau"

    # Ridge: one parameter concentration
    concentration = float(g_abs.max()) / (g_sum + 1e-12)
    if concentration >= ridge_concentration_thr:
        return "ridge"

    # Cliff: one direction extremely steep relative to median,
    # AND the spike must be absolutely significant (not noise in a flat region).
    if g_sum > 1e-12:
        median_abs = float(np.median(g_abs[g_abs > 1e-12])) if (g_abs > 1e-12).any() else 0.0
        if (median_abs > 0
                and float(g_abs.max()) > cliff_ratio_thr * median_abs
                and float(g_abs.max()) > cliff_abs_min):
            return "cliff"

    return "bowl"


def range_scaled_gradient(gradient: dict[str, float]) -> dict[str, float]:
    """
    Scale each gradient component by the parameter's design range so that
    magnitudes are comparable across parameters with different units.

    scaled_grad_i = grad_i × range_i

    Physical interpretation: scaled_grad_i is the expected metric change
    for a move of 100% of the parameter's design range — a dimensionless
    "importance" score.  This removes unit bias (e.g. Iyy in kg·m² vs
    wind_strength dimensionless) from best-direction selection.
    """
    scaled: dict[str, float] = {}
    for p, g in gradient.items():
        r = _hi.get(p, 1.0) - _lo.get(p, 0.0)
        scaled[p] = g * max(r, 1e-12)
    return scaled


def best_improvement_direction(
    gradient: dict[str, float],
    metric_is_cost: bool = True,
) -> tuple[str, str, float]:
    """
    Return (best_param, direction, magnitude) where direction ∈ {'+', '-'}.

    Uses range-scaled gradient so that all parameters are on the same
    dimensionless scale (expected metric change per full-range move).

    If metric_is_cost (True for rms_error, end_error — lower is better):
      Move in the NEGATIVE gradient direction.
      best_param has the largest negative scaled grad.

    Returns
    -------
    best_param  : parameter name
    direction   : '+' means increase this parameter, '-' means decrease it
    magnitude   : |scaled gradient| — expected metric change per full-range move
    """
    scaled = range_scaled_gradient(gradient)
    params = [p for p in GRAD_PARAMS_ALL if p in scaled]
    if not params:
        return ("", "+", 0.0)

    g_vec = np.array([scaled[p] for p in params])

    if metric_is_cost:
        best_idx  = int(np.argmin(g_vec))     # most negative = largest decrease
        direction = "-" if g_vec[best_idx] > 0 else "+"
    else:
        best_idx  = int(np.argmax(g_vec))
        direction = "+" if g_vec[best_idx] > 0 else "-"

    return (params[best_idx], direction, float(abs(g_vec[best_idx])))


# ── Evolution path ────────────────────────────────────────────────────────────

def compute_evolution_path(
    design: dict,
    Kp: float,
    Kd: float,
    metric: str = "rms_error_deg",
    cfg: FidelityConfig | None = None,
    n_steps: int = 5,
    step_fraction: float = 0.10,
    seed: int = 1,
) -> list[dict]:
    """
    Gradient-descent path through design space.

    At each step:
      1. Compute gradient at current position
      2. Normalise by parameter ranges (so all dimensions are comparable)
      3. Move step_fraction × range in the descent direction
      4. Clip to design bounds

    Returns
    -------
    path : list of dicts, one per step, each containing design params + metric value.
           Step 0 = starting design.  Step n_steps = final design.
    """
    if cfg is None:
        cfg = FidelityConfig.full()

    current = {k: design.get(k, REF.get(k)) for k in GRAD_PARAMS_ALL}
    # Include non-gradient params at their fixed values
    full_design = dict(design)

    path = []
    for step_idx in range(n_steps + 1):
        # Rebuild full design from current gradient-param values
        full_design.update(current)
        # Recompute p_unstable
        full_design["p_unstable"] = estimate_p_unstable(
            full_design.get("static_margin", REF["static_margin"]),
            full_design.get("Cm_alpha",      REF["Cm_alpha"]),
            full_design.get("thrust",        REF["thrust"]),
        )

        r     = _evaluate_design(full_design, Kp, Kd, cfg, seed=seed)
        metric_val = _metric_from_result(r, metric)

        path.append(dict(
            step        = step_idx,
            metric      = metric,
            metric_val  = metric_val,
            success     = int(r.success),
            **{k: full_design[k] for k in GRAD_PARAMS_ALL},
        ))

        if step_idx == n_steps:
            break

        # Gradient at current position
        grad = compute_gradient(full_design, Kp, Kd, metric, cfg, seeds=(seed,))

        # Normalise gradient by parameter range (dimensionless direction)
        g_norm = {}
        for p in GRAD_PARAMS_CONTINUOUS:
            if p in grad:
                p_range = _hi[p] - _lo[p]
                g_norm[p] = grad[p] * p_range    # [metric_unit] (range-normalised)

        total = sum(abs(v) for v in g_norm.values()) + 1e-12
        for p in GRAD_PARAMS_CONTINUOUS:
            if p in g_norm:
                descent = -g_norm[p] / total      # normalised descent direction
                step_size = step_fraction * (_hi[p] - _lo[p])
                current[p] = float(np.clip(
                    float(current.get(p, full_design.get(p))) + descent * step_size,
                    _lo[p], _hi[p],
                ))
        # Integer params: move ±1 in descent direction if gradient is large
        for p in GRAD_PARAMS_INTEGER:
            if p in grad and abs(grad[p]) > 0.5:  # threshold: 0.5 deg per step
                direction = -1 if grad[p] > 0 else +1
                current[p] = int(np.clip(
                    int(current.get(p, full_design.get(p))) + direction,
                    _lo.get(p, 1), _hi.get(p, 6),
                ))

    return path


# ── Neighborhood sampler ──────────────────────────────────────────────────────

def sample_neighborhood(
    design: dict,
    n_samples: int = 50,
    radius_frac: float = 0.15,
    seed: int = 42,
) -> pd.DataFrame:
    """
    Latin Hypercube sample within a hyperbox of ±radius_frac×range around design.

    Used for local surrogate fitting or local sensitivity studies.

    Parameters
    ----------
    design      : center design point
    n_samples   : number of neighborhood samples
    radius_frac : half-width of the sampling box as fraction of full design range
    seed        : LHS seed

    Returns
    -------
    DataFrame of n_samples designs in the neighborhood.  Includes p_unstable.
    """
    from scipy.stats import qmc
    from design_space import DESIGN_NAMES as _ALL_NAMES

    rng    = np.random.default_rng(seed)
    lo_nb  = np.zeros(len(_ALL_NAMES))
    hi_nb  = np.zeros(len(_ALL_NAMES))

    center = np.array([float(design.get(k, REF.get(k, 0))) for k in _ALL_NAMES])
    full_lo = np.array([_lo.get(k, 0) for k in _ALL_NAMES])
    full_hi = np.array([_hi.get(k, 1) for k in _ALL_NAMES])
    half    = radius_frac * (full_hi - full_lo)

    lo_nb = np.clip(center - half, full_lo, full_hi)
    hi_nb = np.clip(center + half, full_lo, full_hi)

    sampler = qmc.LatinHypercube(d=len(_ALL_NAMES), seed=seed)
    X       = qmc.scale(sampler.random(n_samples), lo_nb, hi_nb)

    df = pd.DataFrame(X, columns=_ALL_NAMES)
    df["latency_steps"] = df["latency_steps"].round().astype(int).clip(1, 6)
    df["p_unstable"] = df.apply(
        lambda r: estimate_p_unstable(r["static_margin"], r["Cm_alpha"], r["motor_scale"]),
        axis=1,
    )
    df["center_design_id"] = design.get("design_id", -1)
    return df
