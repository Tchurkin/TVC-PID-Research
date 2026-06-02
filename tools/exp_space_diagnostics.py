from __future__ import annotations

import argparse
from itertools import combinations
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.cluster import KMeans
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor
from sklearn.metrics import accuracy_score, balanced_accuracy_score, f1_score, silhouette_score
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.preprocessing import StandardScaler


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "experiments" / "results" / "exp4_first_correct_fidelity.csv"
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "diagnostics"
RANDOM_STATE = 42

PHYSICAL_PARAMS = [
    "mass",
    "Iyy",
    "static_margin",
    "Cm_alpha",
    "control_effectiveness",
    "thrust",
    "deadband",
    "backlash",
    "latency",
    "wind_strength",
    "servo_slew",
    "max_gimbal",
]

EXPECTED_PHYSICS_ORDER = [
    "DEADBAND_BACKLASH",
    "SENSOR_LATENCY_NOISE",
    "AERO_UNCERTAINTY_AND_DRIFT",
    "NONE",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Design space diagnostics for Exp1/Exp4/Exp5 dataset.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Input CSV path")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="Output diagnostics directory")
    return parser.parse_args()


def normalize_0_1(x: pd.Series) -> pd.Series:
    xmin = float(x.min())
    xmax = float(x.max())
    if np.isclose(xmin, xmax):
        return pd.Series(np.zeros(len(x), dtype=float), index=x.index)
    return (x - xmin) / (xmax - xmin)


def compute_performance_score(df: pd.DataFrame) -> pd.Series:
    fidelity_norm = normalize_0_1(df["first_correct_fidelity_level"])
    boundary_negative_penalty = np.abs(np.minimum(df["boundary_distance"].to_numpy(dtype=float), 0.0))
    cm_alpha_proxy = np.abs(df["Cm_alpha"].to_numpy(dtype=float))

    raw = (
        1.5 * (df["reference_decision"].astype(str).str.upper() == "GO").astype(float)
        + 1.0 * fidelity_norm.to_numpy(dtype=float)
        - 1.0 * boundary_negative_penalty
        - 0.5
        * np.log1p(
            df["deadband"].to_numpy(dtype=float)
            + df["backlash"].to_numpy(dtype=float)
            + df["latency"].to_numpy(dtype=float)
        )
        + 0.5 * df["control_effectiveness"].to_numpy(dtype=float)
        + 0.3 * cm_alpha_proxy
    )
    return normalize_0_1(pd.Series(raw, index=df.index, name="performance_raw")).rename("performance_score")


def ensure_schema(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    if "servo_slew" not in df.columns and "nominal_servo_slew_deg_s" in df.columns:
        df["servo_slew"] = pd.to_numeric(df["nominal_servo_slew_deg_s"], errors="coerce")
    if "max_gimbal" not in df.columns and "nominal_max_gimbal_deg" in df.columns:
        df["max_gimbal"] = pd.to_numeric(df["nominal_max_gimbal_deg"], errors="coerce")

    required = [
        "rocket_id",
        "regime_label",
        "dominant_missing_physics",
        "boundary_distance",
        "reference_decision",
        "first_correct_fidelity_level",
        *PHYSICAL_PARAMS,
    ]

    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    for col in ["boundary_distance", "first_correct_fidelity_level", *PHYSICAL_PARAMS]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df["regime_label"] = df["regime_label"].astype(str).str.upper().str.strip()
    df["dominant_missing_physics"] = df["dominant_missing_physics"].astype(str).str.upper().str.strip()

    return df


def prepare_features(df: pd.DataFrame) -> pd.DataFrame:
    x = df[PHYSICAL_PARAMS].copy()
    x = x.fillna(x.median(numeric_only=True))
    return x


def effect_size_cohens_d(a: pd.Series, b: pd.Series) -> float:
    a = pd.to_numeric(a, errors="coerce").dropna()
    b = pd.to_numeric(b, errors="coerce").dropna()
    if len(a) < 2 or len(b) < 2:
        return 0.0
    sa = float(a.std(ddof=1))
    sb = float(b.std(ddof=1))
    pooled = np.sqrt(((len(a) - 1) * sa * sa + (len(b) - 1) * sb * sb) / (len(a) + len(b) - 2))
    if np.isclose(pooled, 0.0):
        return 0.0
    return float((a.mean() - b.mean()) / pooled)


def make_regime_parameter_statistics(df: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    regimes = ["EASY", "FRAGILE", "INFEASIBLE"]
    rows: list[dict[str, float | str]] = []

    for p in PHYSICAL_PARAMS:
        row: dict[str, float | str] = {"parameter": p}
        for r in regimes:
            vals = pd.to_numeric(df.loc[df["regime_label"] == r, p], errors="coerce").dropna()
            row[f"mean_{r}"] = float(vals.mean()) if len(vals) else np.nan
            row[f"median_{r}"] = float(vals.median()) if len(vals) else np.nan
            row[f"std_{r}"] = float(vals.std(ddof=1)) if len(vals) > 1 else 0.0

        row["effect_size_easy_vs_infeasible"] = effect_size_cohens_d(
            df.loc[df["regime_label"] == "EASY", p],
            df.loc[df["regime_label"] == "INFEASIBLE", p],
        )
        rows.append(row)

    out = pd.DataFrame(rows)
    out.to_csv(out_dir / "regime_parameter_statistics.csv", index=False)
    return out


def classifier_cv_with_importance(
    x: pd.DataFrame,
    y: pd.Series,
    label_name: str,
    out_path: Path,
) -> pd.DataFrame:
    min_count = int(y.value_counts().min())
    n_splits = max(2, min(5, min_count))
    cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=RANDOM_STATE)

    clf = RandomForestClassifier(n_estimators=250, random_state=RANDOM_STATE, n_jobs=1)
    pred = cross_val_predict(clf, x, y, cv=cv)

    metrics_df = pd.DataFrame(
        [
            {"section": "metric", "name": "accuracy", "value": float(accuracy_score(y, pred))},
            {
                "section": "metric",
                "name": "balanced_accuracy",
                "value": float(balanced_accuracy_score(y, pred)),
            },
            {"section": "metric", "name": "macro_f1", "value": float(f1_score(y, pred, average="macro"))},
            {"section": "metric", "name": "n_splits", "value": float(n_splits)},
            {"section": "metric", "name": "n_samples", "value": float(len(y))},
            {"section": "metric", "name": "target", "value": label_name},
        ]
    )

    clf.fit(x, y)
    fi = pd.DataFrame(
        {
            "section": "feature_importance",
            "name": x.columns,
            "value": clf.feature_importances_,
        }
    ).sort_values("value", ascending=False)

    out = pd.concat([metrics_df, fi], ignore_index=True)
    out.to_csv(out_path, index=False)
    return out


def make_exp4_overlap_matrix(df: pd.DataFrame, x: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    scaler = StandardScaler()
    x_scaled = pd.DataFrame(scaler.fit_transform(x), columns=x.columns, index=x.index)

    categories = [c for c in EXPECTED_PHYSICS_ORDER if c in set(df["dominant_missing_physics"])]
    if len(categories) < 2:
        out = pd.DataFrame(
            [
                {
                    "category_a": "NA",
                    "category_b": "NA",
                    "centroid_distance": np.nan,
                    "within_spread_a": np.nan,
                    "within_spread_b": np.nan,
                    "overlap_score": np.nan,
                }
            ]
        )
        out.to_csv(out_dir / "exp4_overlap_matrix.csv", index=False)
        return out

    stats: dict[str, dict[str, np.ndarray | float]] = {}
    for c in categories:
        m = df["dominant_missing_physics"] == c
        block = x_scaled.loc[m]
        centroid = block.mean(axis=0).to_numpy(dtype=float)
        dists = np.linalg.norm(block.to_numpy(dtype=float) - centroid, axis=1)
        spread = float(np.mean(dists)) if len(dists) else np.nan
        stats[c] = {"centroid": centroid, "spread": spread}

    rows: list[dict[str, float | str]] = []
    for a, b in combinations(categories, 2):
        ca = stats[a]["centroid"]
        cb = stats[b]["centroid"]
        sa = float(stats[a]["spread"])
        sb = float(stats[b]["spread"])
        dist = float(np.linalg.norm(ca - cb))
        denom = sa + sb + 1e-9
        overlap = float(np.exp(-dist / denom))
        rows.append(
            {
                "category_a": a,
                "category_b": b,
                "centroid_distance": dist,
                "within_spread_a": sa,
                "within_spread_b": sb,
                "overlap_score": overlap,
            }
        )

    out = pd.DataFrame(rows).sort_values("overlap_score", ascending=False)
    out.to_csv(out_dir / "exp4_overlap_matrix.csv", index=False)
    return out


def make_parameter_bottleneck_scores(df: pd.DataFrame, x: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    y = compute_performance_score(df)
    model = RandomForestRegressor(n_estimators=300, random_state=RANDOM_STATE, n_jobs=1)
    model.fit(x, y)

    baseline = x.median(numeric_only=True)
    rows: list[dict[str, float | str | int]] = []

    for p in PHYSICAL_PARAMS:
        p10 = float(x[p].quantile(0.10))
        p90 = float(x[p].quantile(0.90))

        v_low = baseline.copy()
        v_high = baseline.copy()
        v_low[p] = p10
        v_high[p] = p90

        y_low = float(model.predict(pd.DataFrame([v_low]))[0])
        y_high = float(model.predict(pd.DataFrame([v_high]))[0])
        gain = y_high - y_low

        rows.append({"parameter": p, "performance_gain": gain})

    out = pd.DataFrame(rows).sort_values("performance_gain", ascending=False).reset_index(drop=True)
    out["rank"] = np.arange(1, len(out) + 1)
    out.to_csv(out_dir / "parameter_bottleneck_scores.csv", index=False)
    return out


def make_interaction_strengths(df: pd.DataFrame, x: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    y = compute_performance_score(df).to_numpy(dtype=float)
    x_std = pd.DataFrame(StandardScaler().fit_transform(x), columns=x.columns, index=x.index)

    rows: list[dict[str, float | str]] = []
    for a, b in combinations(PHYSICAL_PARAMS, 2):
        xa = x_std[a].to_numpy(dtype=float)
        xb = x_std[b].to_numpy(dtype=float)
        xab = xa * xb

        design = np.column_stack([np.ones(len(xa)), xa, xb, xab])
        coeff, *_ = np.linalg.lstsq(design, y, rcond=None)
        y_hat = design @ coeff
        ss_res = float(np.sum((y - y_hat) ** 2))
        ss_tot = float(np.sum((y - np.mean(y)) ** 2)) + 1e-12
        r2 = 1.0 - ss_res / ss_tot

        # Interaction coefficient magnitude scaled by variability of interaction term.
        strength = float(abs(coeff[3]) * np.std(xab))
        rows.append(
            {
                "parameter_a": a,
                "parameter_b": b,
                "interaction_strength": strength,
                "pair_r2": r2,
            }
        )

    out = pd.DataFrame(rows).sort_values("interaction_strength", ascending=False).reset_index(drop=True)
    out["rank"] = np.arange(1, len(out) + 1)
    out.to_csv(out_dir / "interaction_strengths.csv", index=False)
    return out


def make_cluster_summary(x: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    x_std = StandardScaler().fit_transform(x)

    rows: list[dict[str, float | str | int]] = []

    best_k = None
    best_score = -np.inf
    best_labels = None

    for k in range(2, 11):
        km = KMeans(n_clusters=k, random_state=RANDOM_STATE, n_init=20)
        labels = km.fit_predict(x_std)
        n_unique = int(np.unique(labels).size)
        if n_unique < 2:
            score = np.nan
        else:
            score = float(silhouette_score(x_std, labels))

        rows.append(
            {
                "section": "k_scan",
                "cluster_count": k,
                "silhouette_score": score,
                "best_cluster_count": np.nan,
                "cluster_label": np.nan,
                "cluster_size": np.nan,
                "valid_for_silhouette": int(n_unique >= 2),
            }
        )
        if np.isfinite(score) and score > best_score:
            best_score = score
            best_k = k
            best_labels = labels

    if best_k is None or best_labels is None:
        rows.append(
            {
                "section": "best_model",
                "cluster_count": np.nan,
                "silhouette_score": np.nan,
                "best_cluster_count": np.nan,
                "cluster_label": np.nan,
                "cluster_size": np.nan,
                "valid_for_silhouette": 0,
            }
        )
    else:
        rows.append(
            {
                "section": "best_model",
                "cluster_count": np.nan,
                "silhouette_score": best_score,
                "best_cluster_count": int(best_k),
                "cluster_label": np.nan,
                "cluster_size": np.nan,
                "valid_for_silhouette": 1,
            }
        )

        unique, counts = np.unique(best_labels, return_counts=True)
        for lbl, cnt in zip(unique, counts):
            rows.append(
                {
                    "section": "best_cluster_sizes",
                    "cluster_count": np.nan,
                    "silhouette_score": np.nan,
                    "best_cluster_count": int(best_k),
                    "cluster_label": int(lbl),
                    "cluster_size": int(cnt),
                    "valid_for_silhouette": 1,
                }
            )

    out = pd.DataFrame(rows)
    out.to_csv(out_dir / "cluster_summary.csv", index=False)
    return out


def print_concise_summary(
    regime_stats: pd.DataFrame,
    regime_pred: pd.DataFrame,
    physics_pred: pd.DataFrame,
    bottlenecks: pd.DataFrame,
    interactions: pd.DataFrame,
    cluster_summary: pd.DataFrame,
) -> None:
    regime_bal_acc = float(
        regime_pred.loc[
            (regime_pred["section"] == "metric") & (regime_pred["name"] == "balanced_accuracy"), "value"
        ].iloc[0]
    )
    physics_bal_acc = float(
        physics_pred.loc[
            (physics_pred["section"] == "metric") & (physics_pred["name"] == "balanced_accuracy"), "value"
        ].iloc[0]
    )

    top_sep = regime_stats.reindex(
        regime_stats["effect_size_easy_vs_infeasible"].abs().sort_values(ascending=False).index
    ).head(3)
    top_bottlenecks = bottlenecks.head(3)
    top_interactions = interactions.head(3)

    best_row = cluster_summary.loc[cluster_summary["section"] == "best_model"].iloc[0]
    best_k_val = best_row["best_cluster_count"]
    best_sil = float(best_row["silhouette_score"]) if pd.notna(best_row["silhouette_score"]) else np.nan

    def reality_text(score: float) -> str:
        if score >= 0.7:
            return "strong"
        if score >= 0.55:
            return "moderate"
        return "weak"

    print("\nConcise diagnostic summary")
    print("1) Do regimes appear physically real?")
    print(
        f"   {reality_text(regime_bal_acc)} signal (balanced_accuracy={regime_bal_acc:.3f}); top separators: "
        + ", ".join(top_sep["parameter"].tolist())
    )

    print("2) Do fidelity requirements appear region-dependent?")
    print(
        f"   {reality_text(physics_bal_acc)} signal (balanced_accuracy={physics_bal_acc:.3f}) for dominant_missing_physics prediction."
    )

    print("3) Which parameters dominate performance?")
    print(
        "   "
        + ", ".join(
            [f"{r.parameter} (gain={r.performance_gain:.4f})" for r in top_bottlenecks.itertuples(index=False)]
        )
    )

    print("4) Which interactions dominate performance?")
    print(
        "   "
        + ", ".join(
            [
                f"{r.parameter_a} x {r.parameter_b} (strength={r.interaction_strength:.4f})"
                for r in top_interactions.itertuples(index=False)
            ]
        )
    )

    if pd.isna(best_k_val) or not np.isfinite(best_sil):
        continuity = "mostly continuous"
        best_k_text = "NA"
        best_sil_text = "NA"
    else:
        continuity = "clustered" if best_sil >= 0.35 else "mostly continuous"
        best_k_text = str(int(best_k_val))
        best_sil_text = f"{best_sil:.3f}"
    print("5) Does the design space appear clustered or continuous?")
    print(f"   {continuity} (best_k={best_k_text}, silhouette={best_sil_text}).")


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.input)
    df = ensure_schema(df)
    df = df.dropna(subset=["rocket_id", "regime_label", "dominant_missing_physics"])

    x = prepare_features(df)
    print(f"Loaded dataset with {len(df)} rows and {len(df.columns)} columns")

    print("[1/7] Computing regime parameter statistics...")
    regime_stats = make_regime_parameter_statistics(df, args.output_dir)
    print("[2/7] Computing regime predictability (RF + CV)...")
    regime_pred = classifier_cv_with_importance(x, df["regime_label"], "regime_label", args.output_dir / "regime_predictability.csv")
    print("[3/7] Computing missing-physics predictability (RF + CV)...")
    physics_pred = classifier_cv_with_importance(
        x,
        df["dominant_missing_physics"],
        "dominant_missing_physics",
        args.output_dir / "missing_physics_predictability.csv",
    )

    print("[4/7] Computing Exp4 overlap matrix...")
    make_exp4_overlap_matrix(df, x, args.output_dir)
    print("[5/7] Computing parameter bottleneck scores...")
    bottlenecks = make_parameter_bottleneck_scores(df, x, args.output_dir)
    print("[6/7] Computing interaction strengths...")
    interactions = make_interaction_strengths(df, x, args.output_dir)
    print("[7/7] Computing cluster summary...")
    cluster_summary = make_cluster_summary(x, args.output_dir)

    print(f"Saved diagnostics CSVs to: {args.output_dir}")
    print_concise_summary(regime_stats, regime_pred, physics_pred, bottlenecks, interactions, cluster_summary)


if __name__ == "__main__":
    main()
