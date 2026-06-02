from __future__ import annotations

import argparse
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from sklearn.decomposition import PCA
from sklearn.ensemble import RandomForestRegressor
from sklearn.preprocessing import StandardScaler


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "experiments" / "results" / "exp4_first_correct_fidelity.csv"
DEFAULT_OUTPUT_DIR = ROOT / "outputs"
RANDOM_STATE = 42


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Full Exp1/Exp4/Exp5 analysis and visualization pipeline.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Input CSV path.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="Directory for figure outputs.")
    parser.add_argument(
        "--embedding",
        choices=["pca", "umap", "auto"],
        default="pca",
        help="Embedding method. Use 'pca' for fastest iteration, 'umap' for nonlinear embedding, or 'auto' to try UMAP then fallback.",
    )
    parser.add_argument(
        "--enable-3d-surface",
        action="store_true",
        help="Enable 3D trisurface plot. Disabled by default because triangulation can be slow on some datasets.",
    )
    return parser.parse_args()


def ensure_schema(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    if "servo_slew" not in df.columns and "nominal_servo_slew_deg_s" in df.columns:
        df["servo_slew"] = pd.to_numeric(df["nominal_servo_slew_deg_s"], errors="coerce")
    if "max_gimbal" not in df.columns and "nominal_max_gimbal_deg" in df.columns:
        df["max_gimbal"] = pd.to_numeric(df["nominal_max_gimbal_deg"], errors="coerce")

    required = [
        "rocket_id",
        "regime_label",
        "boundary_distance",
        "reference_decision",
        "first_correct_fidelity_level",
        "dominant_missing_physics",
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

    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    numeric_cols = [
        "boundary_distance",
        "first_correct_fidelity_level",
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
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    return df


def fit_embedding(df: pd.DataFrame, numeric_cols: list[str], embedding_method: str) -> tuple[np.ndarray, str]:
    scaler = StandardScaler()
    X = scaler.fit_transform(df[numeric_cols].fillna(df[numeric_cols].median(numeric_only=True)))

    if embedding_method == "pca":
        reducer = PCA(n_components=2, random_state=RANDOM_STATE)
        emb = reducer.fit_transform(X)
        method = "PCA"
        return emb, method

    if embedding_method == "umap":
        import umap  # type: ignore

        reducer = umap.UMAP(n_components=2, random_state=RANDOM_STATE)
        emb = reducer.fit_transform(X)
        method = "UMAP"
        return emb, method

    # auto mode
    try:
        import umap  # type: ignore

        reducer = umap.UMAP(n_components=2, random_state=RANDOM_STATE)
        emb = reducer.fit_transform(X)
        method = "UMAP"
    except Exception:
        reducer = PCA(n_components=2, random_state=RANDOM_STATE)
        emb = reducer.fit_transform(X)
        method = "PCA (fallback)"

    return emb, method


def normalize_0_1(x: pd.Series) -> pd.Series:
    xmin = float(x.min())
    xmax = float(x.max())
    if np.isclose(xmax, xmin):
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
        - 0.5 * np.log1p(df["deadband"].to_numpy(dtype=float) + df["backlash"].to_numpy(dtype=float) + df["latency"].to_numpy(dtype=float))
        + 0.5 * df["control_effectiveness"].to_numpy(dtype=float)
        + 0.3 * cm_alpha_proxy
    )

    raw_series = pd.Series(raw, index=df.index, name="performance_raw")
    return normalize_0_1(raw_series).rename("performance_score")


def save_exp1_plots(df: pd.DataFrame, emb: np.ndarray, emb_name: str, out_dir: Path) -> None:
    regime_order = ["EASY", "FRAGILE", "INFEASIBLE"]
    regime_palette = {"EASY": "#2a9d8f", "FRAGILE": "#f4a261", "INFEASIBLE": "#e63946"}

    fig, axes = plt.subplots(1, 2, figsize=(14, 6), dpi=160)

    ax0 = axes[0]
    for regime in regime_order:
        m = df["regime_label"].astype(str).str.upper() == regime
        ax0.scatter(emb[m, 0], emb[m, 1], s=28, alpha=0.75, label=regime, color=regime_palette[regime], edgecolors="none")
    ax0.set_title(f"Exp1 Regime Map ({emb_name})")
    ax0.set_xlabel("Embedding Dimension 1")
    ax0.set_ylabel("Embedding Dimension 2")
    ax0.legend(title="regime_label", loc="best")
    ax0.grid(alpha=0.25)

    ax1 = axes[1]
    infeasible = df["regime_label"].astype(str).str.upper() == "INFEASIBLE"
    ax1.scatter(emb[:, 0], emb[:, 1], s=18, alpha=0.15, color="#7f8c8d", label="All")
    ax1.scatter(emb[infeasible, 0], emb[infeasible, 1], s=32, alpha=0.9, color="#e63946", label="INFEASIBLE")
    ax1.set_title("Exp1 Infeasible Overlay")
    ax1.set_xlabel("Embedding Dimension 1")
    ax1.set_ylabel("Embedding Dimension 2")
    ax1.legend(loc="best")
    ax1.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_dir / "exp1_regime_map.png", bbox_inches="tight")
    plt.close(fig)

    fig2, ax2 = plt.subplots(figsize=(8, 6), dpi=160)
    vals = df["boundary_distance"].to_numpy(dtype=float)
    sc = ax2.scatter(emb[:, 0], emb[:, 1], c=vals, cmap="coolwarm", norm=Normalize(vmin=np.nanmin(vals), vmax=np.nanmax(vals)), s=28, alpha=0.85, edgecolors="none")
    ax2.set_title(f"Exp1 Boundary Distance Heatmap ({emb_name})")
    ax2.set_xlabel("Embedding Dimension 1")
    ax2.set_ylabel("Embedding Dimension 2")
    ax2.grid(alpha=0.25)
    cbar = fig2.colorbar(sc, ax=ax2)
    cbar.set_label("boundary_distance")
    fig2.tight_layout()
    fig2.savefig(out_dir / "exp1_boundary_heatmap.png", bbox_inches="tight")
    plt.close(fig2)


def save_exp4_plots(df: pd.DataFrame, emb: np.ndarray, emb_name: str, out_dir: Path) -> None:
    cats = pd.Categorical(df["dominant_missing_physics"].fillna("UNKNOWN").astype(str))
    labels = list(cats.categories)
    cmap = plt.get_cmap("tab20", len(labels))
    color_idx = cats.codes

    fig, ax = plt.subplots(figsize=(9, 7), dpi=160)
    for i, lbl in enumerate(labels):
        m = color_idx == i
        ax.scatter(emb[m, 0], emb[m, 1], s=28, alpha=0.82, color=cmap(i), label=lbl, edgecolors="none")
    ax.set_title(f"Exp4 Missing Physics Clustering ({emb_name})")
    ax.set_xlabel("Embedding Dimension 1")
    ax.set_ylabel("Embedding Dimension 2")
    ax.grid(alpha=0.25)
    ax.legend(title="dominant_missing_physics", fontsize=8, loc="best", ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "exp4_missing_physics_map.png", bbox_inches="tight")
    plt.close(fig)

    regime_order = ["EASY", "FRAGILE", "INFEASIBLE"]
    fig2, axes = plt.subplots(3, 1, figsize=(8, 14), dpi=160, sharex=True, sharey=True)
    for ax, regime in zip(axes, regime_order):
        m = df["regime_label"].astype(str).str.upper() == regime
        ax.scatter(emb[:, 0], emb[:, 1], s=14, alpha=0.08, color="#6c757d", edgecolors="none")
        for i, lbl in enumerate(labels):
            mm = m & (color_idx == i)
            if np.any(mm):
                ax.scatter(emb[mm, 0], emb[mm, 1], s=26, alpha=0.9, color=cmap(i), label=lbl, edgecolors="none")
        ax.set_title(f"Exp4 Regime Overlay: {regime}")
        ax.set_ylabel("Embedding Dimension 2")
        ax.grid(alpha=0.25)

    axes[-1].set_xlabel("Embedding Dimension 1")
    handles, handle_labels = axes[-1].get_legend_handles_labels()
    if handles:
        fig2.legend(handles, handle_labels, title="dominant_missing_physics", loc="lower center", ncol=3, fontsize=8)
    fig2.suptitle(f"Exp4 Regime Facets ({emb_name})", y=0.995)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.98])
    fig2.savefig(out_dir / "exp4_regime_facets.png", bbox_inches="tight")
    plt.close(fig2)


def train_model(df: pd.DataFrame, score: pd.Series) -> tuple[object, pd.DataFrame, np.ndarray, str]:
    feature_cols = [
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
        "boundary_distance",
        "first_correct_fidelity_level",
    ]

    X_num = df[feature_cols].fillna(df[feature_cols].median(numeric_only=True))
    X_cat = pd.get_dummies(
        df[["regime_label", "reference_decision", "dominant_missing_physics"]].fillna("UNKNOWN").astype(str),
        prefix=["regime", "decision", "physics"],
        drop_first=False,
    )
    X = pd.concat([X_num, X_cat], axis=1)
    y = score.to_numpy(dtype=float)

    model_name = "RandomForestRegressor"
    model: object
    try:
        from xgboost import XGBRegressor  # type: ignore

        model = XGBRegressor(
            n_estimators=200,
            max_depth=5,
            learning_rate=0.05,
            subsample=0.9,
            colsample_bytree=0.9,
            objective="reg:squarederror",
            tree_method="hist",
            n_jobs=-1,
            random_state=RANDOM_STATE,
        )
        model_name = "XGBoostRegressor"
    except Exception:
        model = RandomForestRegressor(n_estimators=300, n_jobs=-1, random_state=RANDOM_STATE)

    model.fit(X, y)
    pred = model.predict(X)

    if hasattr(model, "feature_importances_"):
        importances = np.asarray(model.feature_importances_, dtype=float)
    else:
        importances = np.zeros(X.shape[1], dtype=float)

    fi = pd.DataFrame({"feature": X.columns, "importance": importances}).sort_values("importance", ascending=False)
    return model, fi, pred, model_name


def save_exp5_plots(
    df: pd.DataFrame,
    emb: np.ndarray,
    emb_name: str,
    predicted_score: np.ndarray,
    feature_importance: pd.DataFrame,
    out_dir: Path,
    enable_3d_surface: bool,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 7), dpi=160)
    sc = ax.scatter(emb[:, 0], emb[:, 1], c=predicted_score, cmap="viridis", s=30, alpha=0.88, edgecolors="none")
    ax.set_title(f"Exp5 Predicted Performance Landscape ({emb_name})")
    ax.set_xlabel("Embedding Dimension 1")
    ax.set_ylabel("Embedding Dimension 2")
    ax.grid(alpha=0.25)
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("predicted performance_score")
    fig.tight_layout()
    fig.savefig(out_dir / "exp5_performance_map.png", bbox_inches="tight")
    plt.close(fig)

    if enable_3d_surface:
        fig2 = plt.figure(figsize=(10, 7), dpi=160)
        ax2 = fig2.add_subplot(111, projection="3d")
        ax2.plot_trisurf(
            df["static_margin"].to_numpy(dtype=float),
            df["control_effectiveness"].to_numpy(dtype=float),
            predicted_score,
            cmap="viridis",
            alpha=0.9,
            linewidth=0.15,
        )
        ax2.set_title("Exp5 3D Performance Surface")
        ax2.set_xlabel("static_margin")
        ax2.set_ylabel("control_effectiveness")
        ax2.set_zlabel("predicted performance_score")
        fig2.tight_layout()
        fig2.savefig(out_dir / "exp5_surface_3d.png", bbox_inches="tight")
        plt.close(fig2)
    else:
        # Fast fallback: still emit the expected file path without triangulation overhead.
        fig2, ax2 = plt.subplots(figsize=(10, 7), dpi=160)
        hb = ax2.hexbin(
            df["static_margin"].to_numpy(dtype=float),
            df["control_effectiveness"].to_numpy(dtype=float),
            C=predicted_score,
            reduce_C_function=np.mean,
            gridsize=25,
            cmap="viridis",
        )
        ax2.set_title("Exp5 Performance Surface Proxy (3D Disabled)")
        ax2.set_xlabel("static_margin")
        ax2.set_ylabel("control_effectiveness")
        cbar2 = fig2.colorbar(hb, ax=ax2)
        cbar2.set_label("mean predicted performance_score")
        fig2.tight_layout()
        fig2.savefig(out_dir / "exp5_surface_3d.png", bbox_inches="tight")
        plt.close(fig2)

    top10 = feature_importance.head(10).iloc[::-1]
    fig3, ax3 = plt.subplots(figsize=(10, 6), dpi=160)
    ax3.barh(top10["feature"], top10["importance"], color="#1d3557", alpha=0.9)
    ax3.set_title("Exp5 Model Sensitivity (Top 10 Features)")
    ax3.set_xlabel("Feature Importance")
    ax3.set_ylabel("Feature")
    ax3.grid(axis="x", alpha=0.25)
    fig3.tight_layout()
    fig3.savefig(out_dir / "exp5_sensitivity.png", bbox_inches="tight")
    plt.close(fig3)


def save_optional_plotly(df: pd.DataFrame, emb: np.ndarray, predicted_score: np.ndarray, out_dir: Path) -> None:
    try:
        import plotly.express as px  # type: ignore

        tmp = df.copy()
        tmp["emb_1"] = emb[:, 0]
        tmp["emb_2"] = emb[:, 1]
        tmp["predicted_performance_score"] = predicted_score

        fig = px.scatter(
            tmp,
            x="emb_1",
            y="emb_2",
            color="predicted_performance_score",
            color_continuous_scale="Viridis",
            hover_data=["rocket_id", "regime_label", "dominant_missing_physics"],
            title="Exp5 Interactive Predicted Performance Map",
        )
        fig.write_html(str(out_dir / "exp5_performance_map_interactive.html"), include_plotlyjs="cdn")
    except Exception:
        return


def main() -> None:
    args = parse_args()
    t0 = perf_counter()
    out_dir = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    print("[1/7] Loading dataset...")
    t = perf_counter()
    df = pd.read_csv(args.input)
    df = ensure_schema(df)
    df = df.dropna(subset=["rocket_id", "regime_label", "dominant_missing_physics"])
    print(f"  rows={len(df):,}, cols={len(df.columns)}")
    print(f"  load+schema time={perf_counter() - t:.2f}s")

    embedding_features = [
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
        "boundary_distance",
        "first_correct_fidelity_level",
    ]

    print("[2/7] Dataset diagnostics...")
    t = perf_counter()
    dup_count = int(df.duplicated(subset=embedding_features, keep=False).sum())
    unique_count = int(df[embedding_features].drop_duplicates().shape[0])
    print(f"  duplicate numeric rows={dup_count:,}")
    print(f"  unique numeric rows={unique_count:,}")
    print(f"  diagnostics time={perf_counter() - t:.2f}s")

    print(f"[3/7] Starting embedding ({args.embedding})...")
    t = perf_counter()
    emb, emb_name = fit_embedding(df, embedding_features, args.embedding)
    print(f"  embedding method={emb_name}")
    print(f"  embedding time={perf_counter() - t:.2f}s")

    print("[4/7] Generating Exp1/Exp4 plots...")
    t = perf_counter()
    save_exp1_plots(df, emb, emb_name, out_dir)
    save_exp4_plots(df, emb, emb_name, out_dir)
    print(f"  exp1+exp4 plotting time={perf_counter() - t:.2f}s")

    print("[5/7] Computing performance score...")
    t = perf_counter()
    score = compute_performance_score(df)
    df["performance_score"] = score
    print(f"  scoring time={perf_counter() - t:.2f}s")

    print("[6/7] Training model...")
    t = perf_counter()
    _, feature_importance, predicted_score, model_name = train_model(df, score)
    print(f"  model={model_name}")
    print(f"  model training+predict time={perf_counter() - t:.2f}s")

    print(f"[7/7] Generating Exp5 plots (3D={'on' if args.enable_3d_surface else 'off'})...")
    t = perf_counter()
    save_exp5_plots(df, emb, emb_name, predicted_score, feature_importance, out_dir, args.enable_3d_surface)
    save_optional_plotly(df, emb, predicted_score, out_dir)
    print(f"  exp5 plotting time={perf_counter() - t:.2f}s")
    print(f"Total pipeline time={perf_counter() - t0:.2f}s")

    print("Saved output figures to:", out_dir)
    print("Embedding method:", emb_name)
    print("Regression model:", model_name)
    print("\nTop 10 feature importance:")
    print(feature_importance.head(10).to_string(index=False))

    print("\nRegime counts:")
    print(df["regime_label"].value_counts().to_string())

    print("\nDominant missing physics counts:")
    print(df["dominant_missing_physics"].value_counts().to_string())


if __name__ == "__main__":
    main()
