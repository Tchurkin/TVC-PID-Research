"""
sensitivity.py — Permutation importance for TVC design parameters.

Answers: "Which design parameter most affects each outcome metric?"

Method: Permutation feature importance (Breiman 2001, sklearn implementation).
  For each feature:
    1. Randomly shuffle that feature's values in the test set (breaks feature-target association)
    2. Measure the drop in model score
    3. Larger drop = more important feature
  Repeated N_PERM times per feature; report mean ± std.

Targets analysed
────────────────
  A. divergence_risk   P(INFEASIBLE) — the primary safety metric
  B. rms_error_deg     tracking quality — continuous regression target
  C. end_error_deg     terminal accuracy — gyro drift / sensor bias effects

For (A), the RF classifier is used (score = accuracy on regime_code).
For (B) and (C), a separate RF regressor is fit on each continuous target.

Output: sensitivity.json
────────────────────────
  {
    "divergence_risk": {
      "feature_1": {"mean": 0.12, "std": 0.02, "rank": 1},
      ...
    },
    "rms_error_deg": { ... },
    "end_error_deg": { ... },
    "method": "permutation_importance",
    "n_permutations": 30,
    "n_designs": 1200,
    "note": "..."
  }
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor
from sklearn.inspection import permutation_importance
from sklearn.model_selection import train_test_split

from .regime_boundary import DESIGN_FEATURES

N_PERMUTATIONS = 30
TEST_SIZE      = 0.30    # holdout fraction for permutation importance evaluation


def permutation_importance_analysis(
    df: pd.DataFrame,
    model_bundle: dict,
    n_permutations: int = N_PERMUTATIONS,
    seed: int = 42,
) -> dict:
    """
    Compute permutation importance for three targets.

    Parameters
    ----------
    df            : Exp1 DataFrame with DESIGN_FEATURES + outcome columns
    model_bundle  : output of fit_regime_boundary() — RF classifier re-used for (A)
    n_permutations: number of shuffle repeats per feature
    seed          : random seed

    Returns
    -------
    results : nested dict matching the sensitivity.json schema
    """
    X   = df[DESIGN_FEATURES].values.astype(float)
    y_c = df["regime_code"].values.astype(int)         # classification target

    required_regression_targets = ["rms_error_deg", "end_error_deg"]
    for col in required_regression_targets:
        if col not in df.columns:
            raise ValueError(f"DataFrame missing required column: {col}")

    # ── Split for permutation importance (held-out test set) ──────────────
    X_tr, X_te, yc_tr, yc_te = train_test_split(
        X, y_c, test_size=TEST_SIZE, stratify=y_c, random_state=seed
    )
    te_idx = np.where(
        (df.index.isin(df.index[int(len(df) * (1 - TEST_SIZE)):])) | True
    )[0][:len(X_te)]  # Use same test rows for regression targets
    # Simpler: just re-split regression targets with same seed
    rng_split = np.random.default_rng(seed)
    n_te  = len(X_te)
    te_mask = np.zeros(len(df), dtype=bool)
    te_rows = rng_split.choice(len(df), size=n_te, replace=False)
    te_mask[te_rows] = True
    X_te_reg = X[te_mask]

    results: dict[str, dict] = {}

    # ── (A) Divergence risk: classifier, target = is_infeasible (binary) ──
    rf_clf = model_bundle["random_forest"]
    perm_c = permutation_importance(
        rf_clf, X_te, yc_te,
        n_repeats=n_permutations,
        random_state=seed,
        n_jobs=-1,
        scoring="accuracy",
    )
    results["divergence_risk"] = _format_importance(
        DESIGN_FEATURES,
        perm_c.importances_mean,
        perm_c.importances_std,
    )

    # ── (B) RMS error: regression ──────────────────────────────────────────
    y_rms    = df["rms_error_deg"].values
    y_rms_te = y_rms[te_mask]
    rf_rms   = _fit_regressor(X[~te_mask], y_rms[~te_mask], seed)
    perm_rms = permutation_importance(
        rf_rms, X_te_reg, y_rms_te,
        n_repeats=n_permutations,
        random_state=seed,
        n_jobs=-1,
        scoring="r2",
    )
    results["rms_error_deg"] = _format_importance(
        DESIGN_FEATURES,
        perm_rms.importances_mean,
        perm_rms.importances_std,
    )

    # ── (C) Terminal error: regression ────────────────────────────────────
    y_end    = df["end_error_deg"].values
    y_end_te = y_end[te_mask]
    rf_end   = _fit_regressor(X[~te_mask], y_end[~te_mask], seed)
    perm_end = permutation_importance(
        rf_end, X_te_reg, y_end_te,
        n_repeats=n_permutations,
        random_state=seed,
        n_jobs=-1,
        scoring="r2",
    )
    results["end_error_deg"] = _format_importance(
        DESIGN_FEATURES,
        perm_end.importances_mean,
        perm_end.importances_std,
    )

    return {
        **results,
        "method":         "permutation_importance",
        "n_permutations": n_permutations,
        "n_designs":      int(len(df)),
        "test_fraction":  TEST_SIZE,
        "note": (
            "Importance scores are drops in model score (accuracy for divergence_risk, "
            "R2 for rms/end_error) when a feature is randomly shuffled.  "
            "Negative values occur when shuffling accidentally improves OOB score "
            "on the specific test set; treat near-zero values as noise."
        ),
    }


def _fit_regressor(X_tr: np.ndarray, y_tr: np.ndarray, seed: int) -> RandomForestRegressor:
    rf = RandomForestRegressor(
        n_estimators=200,
        max_features="sqrt",
        min_samples_leaf=5,
        random_state=seed,
        n_jobs=-1,
    )
    rf.fit(X_tr, y_tr)
    return rf


def _format_importance(
    features: list[str],
    means: np.ndarray,
    stds: np.ndarray,
) -> dict[str, dict]:
    """Return ranked dict of {feature: {mean, std, rank}}."""
    order = np.argsort(-means)
    out   = {}
    for rank, idx in enumerate(order, start=1):
        out[features[idx]] = {
            "mean": float(means[idx]),
            "std":  float(stds[idx]),
            "rank": rank,
        }
    return out


def save_sensitivity(results: dict, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Saved: {path}")


def print_summary(results: dict) -> None:
    print("\n=== Sensitivity Analysis Summary ===")
    for target in ["divergence_risk", "rms_error_deg", "end_error_deg"]:
        if target not in results:
            continue
        print(f"\n  Target: {target}")
        feat_data = results[target]
        if not isinstance(feat_data, dict) or "method" in feat_data:
            continue
        ranked = sorted(feat_data.items(), key=lambda x: x[1].get("rank", 99))
        for feat, d in ranked[:5]:
            print(f"    #{d['rank']:1d}  {feat:<28s} {d['mean']:+.4f} +/- {d['std']:.4f}")
