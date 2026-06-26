"""
frontier.py — Stability frontier manifold over key design dimensions.

Computes P(regime | parameters) over 2-D and 3-D slices of the design space,
holding all other parameters at their median values from the training set.

The output is a structured dataset (parquet) — no plotting is done here.
Visualisation is deferred to the paper pipeline.

Primary slice: (p_unstable × servo_slew_deg_s) with backlash at low/median/high.
This is the physically motivated frontier: instability rate vs. actuator speed.

Secondary slices computed:
  - (p_unstable × backlash)         at median slew
  - (p_unstable × control_eff)      at median slew and backlash
  - (servo_slew_deg_s × backlash)   at median p_unstable

Grid resolution: N_GRID × N_GRID = 60 × 60 = 3600 points per slice.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from .regime_boundary import _aligned_proba, DESIGN_FEATURES

N_GRID = 60

# Boundary range from DESIGN_LO / DESIGN_HI in design_space.py
RANGES = {
    "p_unstable":          (0.0,  4.0),    # derived variable; 0.04*30/52*sqrt(35/25)~0.45, max ~3.5
    "servo_slew_deg_s":    (20.0, 120.0),
    "backlash":            (0.00, 0.25),
    "control_effectiveness": (5.0, 14.0),
}

# Three-level backlash for parametric study: low / median / high
BACKLASH_LEVELS = [0.05, 0.12, 0.22]


def _median_row(df: pd.DataFrame) -> dict:
    """Return median values of all DESIGN_FEATURES from training set."""
    return {f: float(df[f].median()) for f in DESIGN_FEATURES}


def _grid_predict(
    rf,
    medians: dict,
    x_feat: str,
    y_feat: str,
    x_vals: np.ndarray,
    y_vals: np.ndarray,
    fixed: dict | None = None,
) -> pd.DataFrame:
    """
    Evaluate RF over a 2-D grid of (x_feat, y_feat), all others at medians.

    Returns a DataFrame with columns:
      x_feat, y_feat, p_infeasible, p_fragile, p_easy, p_success, regime_pred
    """
    if fixed is None:
        fixed = {}

    XX, YY = np.meshgrid(x_vals, y_vals, indexing="ij")  # shape (nx, ny)
    n      = XX.size

    # Build feature matrix
    row_template = {**medians, **fixed}
    X = np.array([
        [row_template.get(f, medians[f]) for f in DESIGN_FEATURES]
    ] * n, dtype=float)

    x_idx = DESIGN_FEATURES.index(x_feat)
    y_idx = DESIGN_FEATURES.index(y_feat)
    X[:, x_idx] = XX.ravel()
    X[:, y_idx] = YY.ravel()

    probs = _aligned_proba(rf, X)
    code_pred = np.argmax(probs, axis=1)
    code_map  = {0: "INFEASIBLE", 1: "FRAGILE", 2: "EASY"}

    out = pd.DataFrame({
        x_feat:        XX.ravel(),
        y_feat:        YY.ravel(),
        "p_infeasible": probs[:, 0],
        "p_fragile":    probs[:, 1],
        "p_easy":       probs[:, 2],
        "p_success":    1.0 - probs[:, 0],
        "regime_pred":  [code_map[c] for c in code_pred],
    })
    # Record fixed values so slices are self-describing
    for feat, val in fixed.items():
        out[f"fixed_{feat}"] = val
    out["slice_x_feat"] = x_feat
    out["slice_y_feat"] = y_feat
    return out


def build_frontier_grid(
    model_bundle: dict,
    df: pd.DataFrame,
    n_grid: int = N_GRID,
) -> pd.DataFrame:
    """
    Build the full frontier map dataset.

    Parameters
    ----------
    model_bundle : output of fit_regime_boundary()
    df           : Exp1 DataFrame (used to compute medians)
    n_grid       : grid resolution per axis

    Returns
    -------
    frontier_df : all slices concatenated, self-describing (slice_* columns)
    """
    rf      = model_bundle["random_forest"]
    medians = _median_row(df)

    slices = []

    # ── Primary slice: p_unstable × servo_slew — one panel per backlash level
    p_vals   = np.linspace(*RANGES["p_unstable"],        n_grid)
    slew_vals = np.linspace(*RANGES["servo_slew_deg_s"], n_grid)
    bl_vals   = np.linspace(*RANGES["backlash"],         n_grid)
    keff_vals = np.linspace(*RANGES["control_effectiveness"], n_grid)

    for bl in BACKLASH_LEVELS:
        s = _grid_predict(rf, medians, "p_unstable", "servo_slew_deg_s",
                          p_vals, slew_vals, fixed={"backlash": bl})
        s["slice_name"] = f"p_vs_slew_bl={bl:.2f}"
        slices.append(s)

    # ── p_unstable × backlash at median slew ──────────────────────────────
    s = _grid_predict(rf, medians, "p_unstable", "backlash",
                      p_vals, bl_vals)
    s["slice_name"] = "p_vs_backlash"
    slices.append(s)

    # ── p_unstable × control_effectiveness at median slew, median backlash
    s = _grid_predict(rf, medians, "p_unstable", "control_effectiveness",
                      p_vals, keff_vals)
    s["slice_name"] = "p_vs_keff"
    slices.append(s)

    # ── servo_slew × backlash at median p_unstable ────────────────────────
    s = _grid_predict(rf, medians, "servo_slew_deg_s", "backlash",
                      slew_vals, bl_vals)
    s["slice_name"] = "slew_vs_backlash"
    slices.append(s)

    frontier_df = pd.concat(slices, ignore_index=True)
    # Record median values used (for reproducibility)
    for feat in DESIGN_FEATURES:
        if feat not in frontier_df.columns:
            frontier_df[f"median_{feat}"] = medians[feat]

    return frontier_df
