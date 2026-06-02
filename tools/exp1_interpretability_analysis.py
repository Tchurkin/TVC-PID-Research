#!/usr/bin/env python3
"""Interpretability study for Exp1 regime prediction.

Computes:
- SHAP global importance
- Permutation importance
- Partial dependence curves + effect strength
- Pairwise interaction strengths (all factors weighted equally)
- Focused backlash-vs-all interaction ranking
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
import shap
from sklearn.ensemble import RandomForestClassifier
from sklearn.inspection import partial_dependence, permutation_importance
from sklearn.metrics import balanced_accuracy_score, classification_report
from sklearn.model_selection import train_test_split


ROOT = Path(__file__).resolve().parents[1]
DATA_PATH = ROOT / "experiments" / "results" / "exp1_regime_index.csv"
OUT_DIR = ROOT / "outputs" / "exp1_interpretability"


def _safe_numeric(df: pd.DataFrame, cols: Iterable[str]) -> None:
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")


def _major_features(df: pd.DataFrame) -> list[str]:
    candidates = [
        "p_unstable",
        "servo_slew_deg_s",
        "max_gimbal_deg",
        "best_u_max_frac",
        "mass",
        "Iyy",
        "static_margin",
        "Cm_alpha",
        "control_effectiveness",
        "thrust",
        "deadband",
        "backlash",
        "latency_steps",
        "wind_strength",
    ]
    return [c for c in candidates if c in df.columns]


def _compute_shap_importance(model: RandomForestClassifier, x_test: pd.DataFrame) -> pd.DataFrame:
    explainer = shap.TreeExplainer(model)
    shap_values = explainer.shap_values(x_test)

    # Common shapes:
    # - list[n_classes] of (n_samples, n_features)
    # - ndarray (n_samples, n_features)
    # - ndarray (n_samples, n_features, n_classes)
    if isinstance(shap_values, list):
        arr = np.stack([np.abs(sv) for sv in shap_values], axis=0)  # (C, N, F)
        mean_abs = arr.mean(axis=(0, 1))
    else:
        arr = np.abs(np.asarray(shap_values))
        if arr.ndim == 2:
            mean_abs = arr.mean(axis=0)
        elif arr.ndim == 3:
            # (N, F, C) expected in some SHAP versions.
            mean_abs = arr.mean(axis=(0, 2))
        else:
            raise ValueError(f"Unexpected SHAP value shape: {arr.shape}")

    out = pd.DataFrame({
        "feature": x_test.columns,
        "mean_abs_shap": mean_abs,
    }).sort_values("mean_abs_shap", ascending=False)
    out["shap_rank"] = np.arange(1, len(out) + 1)
    return out


def _compute_permutation_importance(
    model: RandomForestClassifier, x_test: pd.DataFrame, y_test: pd.Series
) -> pd.DataFrame:
    pi = permutation_importance(
        model,
        x_test,
        y_test,
        n_repeats=6,
        random_state=42,
        scoring="balanced_accuracy",
        n_jobs=1,
    )
    out = pd.DataFrame(
        {
            "feature": x_test.columns,
            "perm_importance_mean": pi.importances_mean,
            "perm_importance_std": pi.importances_std,
        }
    ).sort_values("perm_importance_mean", ascending=False)
    out["perm_rank"] = np.arange(1, len(out) + 1)
    return out


def _compute_pdp(
    model: RandomForestClassifier,
    x_ref: pd.DataFrame,
    classes: np.ndarray,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    long_rows: list[dict] = []
    amp_rows: list[dict] = []

    for feat in x_ref.columns:
        pdp = partial_dependence(
            model,
            x_ref,
            features=[feat],
            grid_resolution=20,
            kind="average",
            response_method="predict_proba",
        )
        grid = pdp["grid_values"][0]
        avg_all = np.asarray(pdp["average"])  # (C, G)

        class_amplitudes = []
        for cidx, cl in enumerate(classes):
            avg = avg_all[cidx].ravel()
            amp = float(np.nanmax(avg) - np.nanmin(avg))
            class_amplitudes.append(amp)

            for g, v in zip(grid, avg):
                long_rows.append(
                    {
                        "feature": feat,
                        "class_label": str(cl),
                        "grid_value": float(g),
                        "partial_dependence": float(v),
                    }
                )

        amp_rows.append(
            {
                "feature": feat,
                "pdp_effect_strength_mean": float(np.mean(class_amplitudes)),
                "pdp_effect_strength_max": float(np.max(class_amplitudes)),
            }
        )

    amp_df = pd.DataFrame(amp_rows).sort_values("pdp_effect_strength_mean", ascending=False)
    amp_df["pdp_rank"] = np.arange(1, len(amp_df) + 1)
    long_df = pd.DataFrame(long_rows)
    return amp_df, long_df


def _compute_pairwise_interactions(
    model: RandomForestClassifier,
    x_ref: pd.DataFrame,
    classes: np.ndarray,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Compute full pairwise interaction strengths with PDP-based H-style metric.

    This treats all factor pairs symmetrically and avoids one-factor-only framing.
    """
    feats = list(x_ref.columns)
    n = len(feats)
    mat = np.zeros((n, n), dtype=float)

    # Cache single-feature PDP arrays for each class.
    single_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for f in feats:
        pdp_f = partial_dependence(
            model,
            x_ref,
            features=[f],
            grid_resolution=8,
            kind="average",
            response_method="predict_proba",
        )
        single_cache[f] = (np.asarray(pdp_f["grid_values"][0]), np.asarray(pdp_f["average"]))

    pair_rows = []
    for i, fi in enumerate(feats):
        print(f"Interaction sweep: feature {i+1}/{n} ({fi})")
        for j in range(i + 1, n):
            fj = feats[j]
            pdp_ij = partial_dependence(
                model,
                x_ref,
                features=[fi, fj],
                grid_resolution=8,
                kind="average",
                response_method="predict_proba",
            )

            avg_ij = np.asarray(pdp_ij["average"])  # (C, Gi, Gj)
            _, avg_i = single_cache[fi]  # (C, Gi)
            _, avg_j = single_cache[fj]  # (C, Gj)

            class_strengths = []
            for cidx, _ in enumerate(classes):
                f_ij = avg_ij[cidx]
                f_i = avg_i[cidx].reshape(-1, 1)
                f_j = avg_j[cidx].reshape(1, -1)
                resid = f_ij - f_i - f_j + np.mean(f_ij)
                denom = np.var(f_ij)
                if denom <= 1e-12:
                    h2 = 0.0
                else:
                    h2 = float(np.var(resid) / denom)
                class_strengths.append(max(0.0, h2))

            strength = float(np.mean(class_strengths))
            mat[i, j] = strength
            mat[j, i] = strength
            pair_rows.append(
                {
                    "feature_a": fi,
                    "feature_b": fj,
                    "interaction_strength_h2": strength,
                }
            )

    np.fill_diagonal(mat, 0.0)
    mat_df = pd.DataFrame(mat, index=feats, columns=feats)
    pair_df = pd.DataFrame(pair_rows).sort_values("interaction_strength_h2", ascending=False)
    pair_df["interaction_rank"] = np.arange(1, len(pair_df) + 1)
    return mat_df, pair_df


def _minmax(s: pd.Series) -> pd.Series:
    lo, hi = float(s.min()), float(s.max())
    if hi - lo <= 1e-12:
        return pd.Series(np.zeros(len(s)), index=s.index)
    return (s - lo) / (hi - lo)


def _composite_rank(
    shap_df: pd.DataFrame,
    perm_df: pd.DataFrame,
    pdp_df: pd.DataFrame,
    interaction_mat: pd.DataFrame,
) -> pd.DataFrame:
    interaction_participation = interaction_mat.apply(lambda r: float(r.sum()), axis=1)
    interaction_participation.name = "interaction_participation"

    merged = (
        shap_df[["feature", "mean_abs_shap"]]
        .merge(perm_df[["feature", "perm_importance_mean"]], on="feature", how="outer")
        .merge(pdp_df[["feature", "pdp_effect_strength_mean"]], on="feature", how="outer")
        .merge(interaction_participation.rename_axis("feature").reset_index(), on="feature", how="outer")
    ).fillna(0.0)

    merged["shap_norm"] = _minmax(merged["mean_abs_shap"])
    merged["perm_norm"] = _minmax(merged["perm_importance_mean"])
    merged["pdp_norm"] = _minmax(merged["pdp_effect_strength_mean"])
    merged["interaction_norm"] = _minmax(merged["interaction_participation"])
    merged["composite_equal_weight"] = (
        merged[["shap_norm", "perm_norm", "pdp_norm", "interaction_norm"]].mean(axis=1)
    )

    out = merged.sort_values("composite_equal_weight", ascending=False)
    out["composite_rank"] = np.arange(1, len(out) + 1)
    return out


def main() -> None:
    if not DATA_PATH.exists():
        raise FileNotFoundError(f"Missing input file: {DATA_PATH}")
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(DATA_PATH)
    features = _major_features(df)
    if "regime_label" not in df.columns:
        raise ValueError("Expected regime_label column not found")

    _safe_numeric(df, features)
    work = df[features + ["regime_label"]].dropna().copy()

    x = work[features].astype(float)
    y = work["regime_label"]

    x_train, x_test, y_train, y_test = train_test_split(
        x,
        y,
        test_size=0.25,
        random_state=42,
        stratify=y,
    )

    model = RandomForestClassifier(
        n_estimators=120,
        random_state=42,
        class_weight="balanced_subsample",
        min_samples_leaf=3,
        n_jobs=1,
    )
    model.fit(x_train, y_train)

    y_pred = model.predict(x_test)
    bal_acc = float(balanced_accuracy_score(y_test, y_pred))
    cls_report = classification_report(y_test, y_pred, output_dict=True)

    print("Computing SHAP importance...")
    shap_df = _compute_shap_importance(model, x_test)
    print("Computing permutation importance...")
    perm_df = _compute_permutation_importance(model, x_test, y_test)
    print("Computing 1D partial dependence...")
    pdp_strength_df, pdp_long_df = _compute_pdp(model, x_test, model.classes_)

    # Sample capped for interaction cost but keeps all factors symmetric.
    x_inter = x_test.sample(n=min(120, len(x_test)), random_state=42)
    print("Computing all-pairs interaction strengths...")
    interaction_mat_df, interaction_pairs_df = _compute_pairwise_interactions(model, x_inter, model.classes_)
    print("Computing equal-weight composite rank...")
    composite_df = _composite_rank(shap_df, perm_df, pdp_strength_df, interaction_mat_df)

    backlash_pairs = interaction_pairs_df[
        (interaction_pairs_df["feature_a"] == "backlash")
        | (interaction_pairs_df["feature_b"] == "backlash")
    ].copy()

    merged = (
        shap_df[["feature", "mean_abs_shap", "shap_rank"]]
        .merge(perm_df[["feature", "perm_importance_mean", "perm_rank"]], on="feature", how="outer")
        .merge(pdp_strength_df[["feature", "pdp_effect_strength_mean", "pdp_rank"]], on="feature", how="outer")
        .merge(composite_df[["feature", "interaction_participation", "composite_equal_weight", "composite_rank"]], on="feature", how="outer")
        .sort_values("composite_rank")
    )

    backlash_row = merged.loc[merged["feature"] == "backlash"].iloc[0].to_dict()
    top_interactions = interaction_pairs_df.head(20)
    top_inter_backlash = backlash_pairs.head(20)

    summary = {
        "rows_used": int(len(work)),
        "feature_count": int(len(features)),
        "classes": [str(c) for c in model.classes_],
        "balanced_accuracy": bal_acc,
        "backlash_ranks": {
            "shap_rank": int(backlash_row.get("shap_rank", -1)),
            "perm_rank": int(backlash_row.get("perm_rank", -1)),
            "pdp_rank": int(backlash_row.get("pdp_rank", -1)),
            "composite_rank": int(backlash_row.get("composite_rank", -1)),
        },
        "top_composite_features": merged.head(10)["feature"].tolist(),
        "top_interaction_pairs": top_interactions[["feature_a", "feature_b", "interaction_strength_h2"]].to_dict(orient="records"),
        "top_backlash_pairs": top_inter_backlash[["feature_a", "feature_b", "interaction_strength_h2"]].to_dict(orient="records"),
        "dominance_interpretation": (
            "Backlash remains dominant after accounting for interactions"
            if int(backlash_row.get("composite_rank", 99)) <= 3
            else "Backlash dominance weakens after accounting for interactions"
        ),
        "classification_report": cls_report,
    }

    shap_df.to_csv(OUT_DIR / "feature_importance_shap.csv", index=False)
    perm_df.to_csv(OUT_DIR / "feature_importance_permutation.csv", index=False)
    pdp_strength_df.to_csv(OUT_DIR / "partial_dependence_effect_strength.csv", index=False)
    pdp_long_df.to_csv(OUT_DIR / "partial_dependence_curves_long.csv", index=False)
    interaction_mat_df.to_csv(OUT_DIR / "interaction_strength_matrix.csv")
    interaction_pairs_df.to_csv(OUT_DIR / "interaction_pairs_ranked.csv", index=False)
    backlash_pairs.to_csv(OUT_DIR / "backlash_pair_interactions.csv", index=False)
    composite_df.to_csv(OUT_DIR / "feature_composite_effect_rank.csv", index=False)
    merged.to_csv(OUT_DIR / "feature_importance_merged_view.csv", index=False)

    with open(OUT_DIR / "interpretability_summary.json", "w", encoding="ascii") as f:
        json.dump(summary, f, indent=2)

    print(f"Saved outputs to: {OUT_DIR}")
    print(f"Balanced accuracy: {bal_acc:.4f}")
    print("Backlash ranks:", summary["backlash_ranks"])
    print("Dominance interpretation:", summary["dominance_interpretation"])


if __name__ == "__main__":
    main()
