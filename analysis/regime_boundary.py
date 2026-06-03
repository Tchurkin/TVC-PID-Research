"""
regime_boundary.py — Probabilistic regime boundary estimation.

Fits two complementary classifiers on Exp1 design features → regime label:
  (A) Logistic regression (multinomial, L2-regularised)
      Interpretable linear boundary in feature space; coefficients are
      directly readable as signed effect sizes after standardisation.

  (B) Random forest (n_estimators=400)
      Non-linear, captures interactions (e.g. backlash × p_unstable).
      Used for P(regime | design) predictions and permutation importance.

Bootstrap uncertainty
─────────────────────
  For each of N_BOOTSTRAP resamples of the training set:
    - Refit the RF (50 trees for speed)
    - Record predicted P(EASY), P(FRAGILE), P(INFEASIBLE) for a reference set
      of designs (the original data points)
  Produces 95% CIs on per-design regime probabilities.

Outputs saved to regime_model.pkl
─────────────────────────────────
  {
    "logistic_regression": fitted LR model (sklearn),
    "scaler":              StandardScaler applied before LR,
    "random_forest":       fitted RF model (sklearn),
    "feature_names":       list[str],
    "rf_importances":      dict feature -> mean importance,
    "rf_importance_ci":    dict feature -> (lo95, hi95) from bootstrap,
    "cv_accuracy_lr":      float (stratified 5-fold),
    "cv_accuracy_rf":      float (stratified 5-fold),
    "class_labels":        ["INFEASIBLE","FRAGILE","EASY"],  (ordered by code 0,1,2)
  }
"""

from __future__ import annotations

import json
import pickle
from pathlib import Path

import numpy as np
import pandas as pd
from joblib import Parallel, delayed
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.preprocessing import StandardScaler

# Import here (not from sim/) so analysis/ has no dependency on the sim package
DESIGN_FEATURES = [
    "p_unstable",
    "control_effectiveness",
    "servo_slew_deg_s",
    "max_gimbal_deg",
    "deadband",
    "backlash",
    "latency_steps",
    "wind_strength",
    "mass",
    "Iyy",
    "thrust",
]

CLASS_LABELS  = ["INFEASIBLE", "FRAGILE", "EASY"]   # ordered by regime_code 0,1,2
CLASS_CODES   = [0, 1, 2]

N_BOOTSTRAP   = 500
SEED_BOOT_BASE = 100


def fit_regime_boundary(
    df: pd.DataFrame,
    n_bootstrap: int = N_BOOTSTRAP,
    seed: int = SEED_BOOT_BASE,
) -> dict:
    """
    Fit LR and RF classifiers on Exp1 data; bootstrap importance CIs.

    Parameters
    ----------
    df          : Exp1 DataFrame (must contain DESIGN_FEATURES + "regime_code")
    n_bootstrap : number of bootstrap resamples for CI estimation
    seed        : base random seed

    Returns
    -------
    model_bundle : dict (see module docstring for contents)
    """
    missing = [f for f in DESIGN_FEATURES if f not in df.columns]
    if missing:
        raise ValueError(f"Missing features in DataFrame: {missing}")
    if "regime_code" not in df.columns:
        raise ValueError("DataFrame must contain 'regime_code' column")

    X = df[DESIGN_FEATURES].values.astype(float)
    y = df["regime_code"].values.astype(int)
    n = len(y)

    # ── Logistic regression (standardised features) ────────────────────────
    scaler   = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    lr = LogisticRegression(
        solver="lbfgs",
        max_iter=2000,
        C=1.0,
        random_state=seed,
    )
    lr.fit(X_scaled, y)

    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=seed)
    cv_acc_lr = cross_val_score(lr, X_scaled, y, cv=cv, scoring="accuracy").mean()

    # ── Random forest ──────────────────────────────────────────────────────
    rf = RandomForestClassifier(
        n_estimators=400,
        max_features="sqrt",
        min_samples_leaf=5,
        random_state=seed,
        n_jobs=-1,
    )
    rf.fit(X, y)
    cv_acc_rf = cross_val_score(rf, X, y, cv=cv, scoring="accuracy").mean()

    # ── Bootstrap: CI on feature importances and per-design probabilities ──
    # Parallelised: each worker fits one bootstrap RF and returns (importances, p_easy, p_infeas).
    def _one_boot(k: int) -> tuple[np.ndarray, float, float]:
        rng_k = np.random.default_rng(seed + k)
        idx   = rng_k.choice(n, size=n, replace=True)
        rf_b  = RandomForestClassifier(
            n_estimators=50,
            max_features="sqrt",
            min_samples_leaf=5,
            random_state=seed + k,
            n_jobs=1,
        )
        rf_b.fit(X[idx], y[idx])
        probs_b  = rf_b.predict_proba(X)
        ci       = {c: i for i, c in enumerate(rf_b.classes_)}
        p_easy   = probs_b[:, ci[2]].mean() if 2 in ci else 0.0
        p_infeas = probs_b[:, ci[0]].mean() if 0 in ci else 0.0
        return rf_b.feature_importances_, p_easy, p_infeas

    boot_results = Parallel(n_jobs=-1, prefer="threads")(
        delayed(_one_boot)(k) for k in range(n_bootstrap)
    )

    boot_importances = np.array([r[0] for r in boot_results])
    boot_p_easy      = np.array([r[1] for r in boot_results])
    boot_p_infeas    = np.array([r[2] for r in boot_results])

    rf_imp_mean = boot_importances.mean(axis=0)
    rf_imp_lo   = np.percentile(boot_importances, 2.5, axis=0)
    rf_imp_hi   = np.percentile(boot_importances, 97.5, axis=0)

    # ── Predict full-dataset probabilities with the final RF ───────────────
    probs = _aligned_proba(rf, X)  # shape (n, 3) ordered [INFEASIBLE, FRAGILE, EASY]

    # Per-design bootstrap CI: reuse the per-bootstrap mean as a coarse CI
    # (individual design CIs would require storing n×n_bootstrap arrays).
    # Report population-level CI on mean P(EASY) and P(INFEASIBLE).
    pop_easy_lo95   = float(np.percentile(boot_p_easy,   2.5))
    pop_easy_hi95   = float(np.percentile(boot_p_easy,  97.5))
    pop_infeas_lo95 = float(np.percentile(boot_p_infeas, 2.5))
    pop_infeas_hi95 = float(np.percentile(boot_p_infeas, 97.5))

    model_bundle = dict(
        logistic_regression  = lr,
        scaler               = scaler,
        random_forest        = rf,
        feature_names        = DESIGN_FEATURES,
        class_labels         = CLASS_LABELS,
        rf_importances       = dict(zip(DESIGN_FEATURES, rf_imp_mean.tolist())),
        rf_importance_ci     = {
            f: (float(rf_imp_lo[i]), float(rf_imp_hi[i]))
            for i, f in enumerate(DESIGN_FEATURES)
        },
        cv_accuracy_lr       = float(cv_acc_lr),
        cv_accuracy_rf       = float(cv_acc_rf),
        population_easy_mean = float(probs[:, 2].mean()),
        population_easy_ci95 = (pop_easy_lo95,   pop_easy_hi95),
        population_infeas_mean = float(probs[:, 0].mean()),
        population_infeas_ci95 = (pop_infeas_lo95, pop_infeas_hi95),
    )
    return model_bundle


def predict_probabilities(
    model_bundle: dict,
    df: pd.DataFrame,
) -> pd.DataFrame:
    """
    Add probabilistic regime predictions to df.  Returns a copy of df with:
      p_infeasible  P(regime = INFEASIBLE)
      p_fragile     P(regime = FRAGILE)
      p_easy        P(regime = EASY)
      p_success     P(regime in {EASY, FRAGILE})  = 1 - p_infeasible
      regime_pred   predicted regime label (argmax)

    Parameters
    ----------
    model_bundle : output of fit_regime_boundary()
    df           : DataFrame containing DESIGN_FEATURES columns
    """
    rf = model_bundle["random_forest"]
    X  = df[model_bundle["feature_names"]].values.astype(float)
    probs = _aligned_proba(rf, X)   # (n, 3) — [INFEASIBLE, FRAGILE, EASY]

    out = df.copy()
    out["p_infeasible"] = probs[:, 0]
    out["p_fragile"]    = probs[:, 1]
    out["p_easy"]       = probs[:, 2]
    out["p_success"]    = 1.0 - probs[:, 0]

    code_pred = np.argmax(probs, axis=1)
    code_to_label = {0: "INFEASIBLE", 1: "FRAGILE", 2: "EASY"}
    out["regime_pred"] = [code_to_label[c] for c in code_pred]

    return out


def _aligned_proba(rf: RandomForestClassifier, X: np.ndarray) -> np.ndarray:
    """
    Return predict_proba output ordered as [P(0), P(1), P(2)] regardless of
    the order rf.classes_ was fit in.  Shape (n_samples, 3).
    """
    raw   = rf.predict_proba(X)           # shape (n, len(rf.classes_))
    order = {c: i for i, c in enumerate(rf.classes_)}
    aligned = np.zeros((len(X), 3), dtype=float)
    for col, code in enumerate([0, 1, 2]):
        if code in order:
            aligned[:, col] = raw[:, order[code]]
    return aligned


def save_model(model_bundle: dict, path: str | Path) -> None:
    """Pickle the model bundle to path."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(model_bundle, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"Saved: {path}")


def load_model(path: str | Path) -> dict:
    """Load a pickled model bundle."""
    with open(path, "rb") as f:
        return pickle.load(f)


def print_summary(model_bundle: dict) -> None:
    """Print a human-readable model summary."""
    print("\n=== Regime Boundary Model Summary ===")
    print(f"  LR 5-fold accuracy : {model_bundle['cv_accuracy_lr']:.3f}")
    print(f"  RF 5-fold accuracy : {model_bundle['cv_accuracy_rf']:.3f}")
    print(f"  Population P(EASY) : {model_bundle['population_easy_mean']:.3f} "
          f"[{model_bundle['population_easy_ci95'][0]:.3f}, "
          f"{model_bundle['population_easy_ci95'][1]:.3f}] 95% CI")
    print(f"  Population P(INFEASIBLE): {model_bundle['population_infeas_mean']:.3f} "
          f"[{model_bundle['population_infeas_ci95'][0]:.3f}, "
          f"{model_bundle['population_infeas_ci95'][1]:.3f}] 95% CI")
    print("\n  RF feature importances (mean +/- 95% CI):")
    imps = model_bundle["rf_importances"]
    cis  = model_bundle["rf_importance_ci"]
    ranked = sorted(imps.items(), key=lambda x: -x[1])
    for feat, imp in ranked:
        lo, hi = cis[feat]
        print(f"    {feat:<28s} {imp:.4f}  [{lo:.4f}, {hi:.4f}]", flush=True)
