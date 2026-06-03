"""
analysis/ — Statistical inference layer for TVC stability study.

Modules
───────
  regime_boundary   Classifier (LR + RF) for EASY/FRAGILE/INFEASIBLE; bootstrap CIs
  frontier          P(success | parameters) manifold over design-space slices
  sensitivity       Permutation importance for divergence risk, RMS error, terminal error

All modules are read-only with respect to the simulator.  They consume data
produced by experiment_runner.py and produce artifacts consumed by
run_full_pipeline.py.
"""

from .regime_boundary import fit_regime_boundary, predict_probabilities, load_model, save_model
from .frontier import build_frontier_grid
from .sensitivity import permutation_importance_analysis
