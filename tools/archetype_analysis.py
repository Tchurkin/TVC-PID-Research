"""
archetype_analysis.py
=====================
Local Failure Archetype Analysis for TVC Model Rocket Design Space (Exp1)

Pipeline:
  Step 1: Local SHAP explanation vectors for every design
  Step 2: Frontier improvement paths for FRAGILE / INFEASIBLE designs
  Step 3: Archetype discovery via HDBSCAN + GMM fallback on explanation space
  Step 4: Archetype characterization table
  Step 5: Visualisations (5 figures)

Outputs (all in outputs/archetype_analysis/):
  local_explanations.csv
  design_improvement_paths.csv
  archetype_summary.csv
  fig1_archetype_prevalence.png
  fig2_archetype_transition_paths.png
  fig3_dominant_factors_per_archetype.png
  fig4_example_trajectories.png
  fig5_archetype_regime_map.png
"""

from __future__ import annotations

import warnings
from pathlib import Path
from typing import Dict, List, Tuple

import hdbscan
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd
import shap
from scipy.optimize import minimize
from scipy.spatial.distance import cdist
from sklearn.decomposition import PCA
from sklearn.ensemble import RandomForestClassifier
from sklearn.mixture import GaussianMixture
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn.model_selection import cross_val_score

warnings.filterwarnings("ignore")

# ─────────────────────────── Paths ───────────────────────────
ROOT = Path(__file__).resolve().parents[1]
DATA_FILE = ROOT / "experiments" / "results" / "exp1_regime_index.csv"
OUT_DIR = ROOT / "outputs" / "archetype_analysis"
OUT_DIR.mkdir(parents=True, exist_ok=True)

FEATURES = [
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

REGIME_ORDER = {"EASY": 2, "FRAGILE": 1, "INFEASIBLE": 0}
REGIME_COLOR = {"EASY": "#2ca02c", "FRAGILE": "#ff7f0e", "INFEASIBLE": "#d62728"}
PALETTE = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
    "#aec7e8", "#ffbb78"
]

# ─────────────────────────── Data loading ───────────────────────────

def load_exp1() -> pd.DataFrame:
    df = pd.read_csv(DATA_FILE)
    present = [c for c in FEATURES if c in df.columns]
    missing = [c for c in FEATURES if c not in df.columns]
    if missing:
        print(f"[WARN] Missing feature columns (will skip): {missing}")
    for c in present:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df[present] = df[present].fillna(df[present].median())
    df["regime_code"] = df["regime_label"].map(REGIME_ORDER).fillna(0).astype(int)
    return df, present


# ─────────────────────────── Step 1: Local SHAP vectors ───────────────────────────

def compute_local_shap(df: pd.DataFrame, features: List[str], random_state: int = 42
                        ) -> Tuple[pd.DataFrame, RandomForestClassifier, List[str]]:
    """
    Train a Random Forest on ALL data, then compute SHAP values for every sample.
    Returns a DataFrame where each row = one design, columns = shap_<feature>.
    Also returns the classifier and class ordering.
    """
    X = df[features].values.astype(float)
    y = df["regime_label"].values

    le = LabelEncoder()
    y_enc = le.fit_transform(y)
    classes = list(le.classes_)            # e.g. ['EASY','FRAGILE','INFEASIBLE']

    clf = RandomForestClassifier(
        n_estimators=300, max_depth=None, min_samples_leaf=2,
        random_state=random_state, n_jobs=-1
    )
    clf.fit(X, y_enc)

    # Cross-val sanity check
    cv = cross_val_score(clf, X, y_enc, cv=5, scoring="balanced_accuracy")
    print(f"[INFO] RF CV balanced_accuracy: {cv.mean():.3f} ± {cv.std():.3f}")

    explainer = shap.TreeExplainer(clf)
    shap_vals = explainer.shap_values(X)   # shape varies by SHAP version

    # Normalise to shape (n_samples, n_features, n_classes)
    if isinstance(shap_vals, list):
        # Older SHAP: list of n_classes arrays, each (n_samples, n_features)
        arr = np.stack(shap_vals, axis=-1)          # → (n_samples, n_feat, n_classes)
    else:
        arr = np.asarray(shap_vals)
        if arr.ndim == 3 and arr.shape[2] == len(classes):
            pass   # already (n_samples, n_feat, n_classes)
        elif arr.ndim == 3 and arr.shape[0] == len(classes):
            arr = np.transpose(arr, (1, 2, 0))      # (n_classes, n_samples, n_feat) → (n_samples, n_feat, n_classes)
        else:
            arr = arr[:, :, np.newaxis]             # 2-D binary edge case

    # arr shape is now (n_samples, n_features, n_classes)
    n_samples, n_feat, n_cls = arr.shape

    easy_idx = classes.index("EASY") if "EASY" in classes else 0
    inf_idx  = classes.index("INFEASIBLE") if "INFEASIBLE" in classes else min(2, n_cls - 1)
    frag_idx = classes.index("FRAGILE") if "FRAGILE" in classes else min(1, n_cls - 1)

    # For each sample, local explanation = SHAP for its predicted class
    pred_class_idx = clf.predict(X)                 # encoded predicted class per sample
    local_shap = arr[np.arange(n_samples), :, pred_class_idx]   # (n_samples, n_feat)

    # "toward-EASY" projection: positive = feature pushes THIS design toward EASY
    delta_shap = arr[:, :, easy_idx] - arr[:, :, inf_idx]       # (n_samples, n_feat)

    # Assemble output
    shap_df = pd.DataFrame(
        local_shap,
        columns=[f"shap_{f}" for f in features]
    )
    delta_df = pd.DataFrame(
        delta_shap,
        columns=[f"delta_shap_{f}" for f in features]
    )

    shap_infeasible = pd.DataFrame(
        arr[:, :, inf_idx],
        columns=[f"shap_infeasible_{f}" for f in features]
    )
    shap_fragile = pd.DataFrame(
        arr[:, :, frag_idx],
        columns=[f"shap_fragile_{f}" for f in features]
    )

    meta = df[["rocket_id", "regime_label", "regime_code", "boundary_distance"]].copy().reset_index(drop=True)
    meta["predicted_regime"] = le.inverse_transform(pred_class_idx)
    meta["rf_easy_prob"]  = clf.predict_proba(X)[:, easy_idx]
    meta["rf_infeasible_prob"] = clf.predict_proba(X)[:, inf_idx]

    out = pd.concat([meta, shap_df, delta_df, shap_infeasible, shap_fragile], axis=1)
    out = out.reset_index(drop=True)
    out.insert(0, "design_id", range(1, len(out) + 1))

    return out, clf, classes, arr, explainer, le


# ─────────────────────────── Step 2: Frontier improvement paths ───────────────────────────

def compute_improvement_paths(df: pd.DataFrame, features: List[str],
                               clf: RandomForestClassifier,
                               le: LabelEncoder,
                               random_state: int = 42) -> pd.DataFrame:
    """
    For each FRAGILE / INFEASIBLE design, estimate the minimum directional parameter
    change to cross into the next-better regime.

    Strategy: For each design, identify the features where the delta_shap
    (toward EASY - toward INFEASIBLE) is most negative (i.e., most limiting).
    Then estimate the % increase needed based on the feature's percentile rank
    in EASY designs vs the current design's value.
    """
    X = df[features].values.astype(float)
    scaler = StandardScaler()
    X_std = scaler.fit_transform(X)

    classes = list(le.classes_)
    easy_idx = classes.index("EASY") if "EASY" in classes else 0
    frag_idx = classes.index("FRAGILE") if "FRAGILE" in classes else 1
    inf_idx  = classes.index("INFEASIBLE") if "INFEASIBLE" in classes else 2

    proba = clf.predict_proba(X)  # (n, n_classes)

    # Reference: statistics of EASY designs per feature
    easy_mask = df["regime_label"] == "EASY"
    easy_stats = {}
    for j, f in enumerate(features):
        vals = df.loc[easy_mask, f].dropna()
        easy_stats[f] = {
            "p10": float(np.percentile(vals, 10)),
            "p25": float(np.percentile(vals, 25)),
            "p50": float(np.percentile(vals, 50)),
            "mean": float(vals.mean()),
            "std": float(vals.std()),
        }

    # Features that, when INCREASED, tend to help (higher = better)
    IMPROVE_BY_INCREASE = {"servo_slew_deg_s", "max_gimbal_deg", "control_effectiveness", "thrust"}
    # Features that, when DECREASED, tend to help (lower = better)
    IMPROVE_BY_DECREASE = {"p_unstable", "backlash", "deadband", "latency_steps",
                           "wind_strength", "Cm_alpha"}
    # Others are ambiguous; use sign of delta_shap to decide direction

    records = []
    fail_mask = df["regime_label"].isin(["FRAGILE", "INFEASIBLE"])

    for idx, row in df[fail_mask].iterrows():
        local_x = X[idx].copy()
        current_regime = row["regime_label"]
        target_regime  = "FRAGILE" if current_regime == "INFEASIBLE" else "EASY"
        target_idx = frag_idx if target_regime == "FRAGILE" else easy_idx

        current_prob_target = proba[idx, target_idx]

        # Rank features by their potential to help (|shap toward target|)
        # Use delta_shap: positive = pushes toward EASY
        feat_scores = []
        for j, f in enumerate(features):
            es = easy_stats[f]
            cur_val = float(local_x[j])

            # Direction: should we push up or down?
            if f in IMPROVE_BY_INCREASE:
                direction = +1
                gap_pct = max(0.0, (es["p25"] - cur_val) / (abs(cur_val) + 1e-9)) * 100
            elif f in IMPROVE_BY_DECREASE:
                direction = -1
                gap_pct = max(0.0, (cur_val - es["p75"] if "p75" in es else cur_val - es["p50"])
                              / (abs(cur_val) + 1e-9)) * 100
            else:
                direction = 0
                gap_pct = 0.0

            feat_scores.append((f, direction, gap_pct, es["p25"], cur_val))

        # For each feature, estimate required change as gap to EASY p25 (conservative)
        # Also estimate probability lift if that feature moved to EASY median
        improvements = []
        for f, direction, gap_pct, easy_p25, cur_val in feat_scores:
            j = features.index(f)
            easy_med = easy_stats[f]["p50"]

            # Simulate move to easy median for this feature
            x_sim = X[idx].copy()
            x_sim[j] = easy_med
            sim_prob = clf.predict_proba(x_sim.reshape(1, -1))[0, target_idx]
            prob_lift = float(sim_prob - current_prob_target)

            if abs(easy_med - cur_val) < 1e-12:
                change_pct = 0.0
            else:
                change_pct = (easy_med - cur_val) / (abs(cur_val) + 1e-9) * 100

            improvements.append({
                "feature": f,
                "cur_val": round(cur_val, 6),
                "easy_median": round(easy_med, 6),
                "required_change_pct": round(change_pct, 2),
                "prob_lift_to_target": round(prob_lift, 4),
            })

        # Sort by prob_lift descending
        improvements.sort(key=lambda d: -d["prob_lift_to_target"])

        # Report top 5 improvements
        for rank, imp in enumerate(improvements[:5], 1):
            records.append({
                "rocket_id": row["rocket_id"],
                "current_regime": current_regime,
                "target_regime": target_regime,
                "improvement_rank": rank,
                "limiting_factor": imp["feature"],
                "current_value": imp["cur_val"],
                "easy_median_value": imp["easy_median"],
                "required_change_pct": imp["required_change_pct"],
                "prob_lift_to_target_regime": imp["prob_lift_to_target"],
            })

    return pd.DataFrame(records)


# ─────────────────────────── Step 3 & 4: Archetype discovery ───────────────────────────

def discover_archetypes(local_df: pd.DataFrame, features: List[str],
                         random_state: int = 42) -> Tuple[pd.DataFrame, pd.DataFrame, object, np.ndarray]:
    """
    Cluster the local explanation (delta_shap) vectors to find failure archetypes.
    Uses HDBSCAN as primary, falls back to GMM if too many noise points.

    Returns:
      cluster_assignments: series indexed same as local_df, values = cluster id
      archetype_summary: DataFrame with per-archetype statistics
      clusterer: fitted model
      X_emb: 2D PCA embedding for visualisation
    """
    # Build explanation matrix: delta_shap for each feature
    shap_cols = [f"delta_shap_{f}" for f in features if f"delta_shap_{f}" in local_df.columns]
    X_exp = local_df[shap_cols].values.astype(float)

    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X_exp)

    # Reduce to top-k PCA dims for clustering stability (keep 95% variance)
    pca = PCA(random_state=random_state)
    pca.fit(X_scaled)
    cum_var = np.cumsum(pca.explained_variance_ratio_)
    n_components = max(3, int(np.searchsorted(cum_var, 0.95)) + 1)
    n_components = min(n_components, X_scaled.shape[1])
    pca_k = PCA(n_components=n_components, random_state=random_state)
    X_pca = pca_k.fit_transform(X_scaled)
    print(f"[INFO] Using {n_components} PCA components ({cum_var[n_components-1]*100:.1f}% variance)")

    # 2D embedding for plotting
    pca2 = PCA(n_components=2, random_state=random_state)
    X_emb = pca2.fit_transform(X_scaled)

    # ── HDBSCAN ──
    min_cluster_sz = max(15, len(local_df) // 40)
    clusterer = hdbscan.HDBSCAN(
        min_cluster_size=min_cluster_sz,
        min_samples=5,
        cluster_selection_epsilon=0.3,
        metric="euclidean",
        cluster_selection_method="eom",
    )
    labels_hdb = clusterer.fit_predict(X_pca)

    noise_frac = float((labels_hdb == -1).mean())
    n_clusters_hdb = len(set(labels_hdb) - {-1})
    print(f"[INFO] HDBSCAN: {n_clusters_hdb} clusters, noise fraction={noise_frac:.2f}")

    # If HDBSCAN produces <2 meaningful clusters or >60% noise, fall back to GMM
    if n_clusters_hdb < 2 or noise_frac > 0.60:
        print("[INFO] Falling back to GMM")
        bic_scores = []
        for k in range(2, 9):
            gmm = GaussianMixture(n_components=k, covariance_type="full",
                                   random_state=random_state, n_init=5)
            gmm.fit(X_pca)
            bic_scores.append((k, gmm.bic(X_pca)))
        best_k = min(bic_scores, key=lambda x: x[1])[0]
        print(f"[INFO] GMM best k={best_k} by BIC")
        clusterer = GaussianMixture(n_components=best_k, covariance_type="full",
                                     random_state=random_state, n_init=10)
        clusterer.fit(X_pca)
        labels = clusterer.predict(X_pca)
    else:
        # Re-assign HDBSCAN noise points to nearest cluster centroid
        labels = labels_hdb.copy()
        if noise_frac > 0:
            # Compute cluster centroids
            cluster_ids = sorted(set(labels) - {-1})
            centroids = np.array([X_pca[labels == c].mean(axis=0) for c in cluster_ids])
            noise_idx = np.where(labels == -1)[0]
            dists = cdist(X_pca[noise_idx], centroids)
            nearest = np.argmin(dists, axis=1)
            for ni, ci in zip(noise_idx, nearest):
                labels[ni] = cluster_ids[ci]
            print(f"[INFO] Reassigned {len(noise_idx)} noise points to nearest centroid")

    n_clusters = len(set(labels))
    print(f"[INFO] Final cluster count: {n_clusters}")

    # ── Archetype naming heuristic ──
    archetype_names = _name_archetypes(local_df, features, labels, n_clusters)

    # ── Archetype characterization ──
    rows = []
    for c in sorted(set(labels)):
        mask = labels == c
        sub = local_df[mask]
        regime_dist = sub["regime_label"].value_counts().to_dict()
        dominant_regime = sub["regime_label"].mode()[0]

        # Dominant features: mean |delta_shap|, pick top-3 that are negative (limiting)
        shap_means = {}
        for f in features:
            col = f"delta_shap_{f}"
            if col in sub.columns:
                shap_means[f] = float(sub[col].mean())

        # Limiting factors: most negative delta_shap (pushing AWAY from EASY)
        limiting = sorted(shap_means.items(), key=lambda x: x[1])[:3]  # most negative first
        # Enabling factors: most positive delta_shap
        enabling = sorted(shap_means.items(), key=lambda x: -x[1])[:2]

        # Typical parameter ranges (mean ± std from raw features if present)
        raw_feat_avail = [f for f in features if f in sub.columns]
        param_means = {f: float(sub[f].mean()) for f in raw_feat_avail}
        param_stds  = {f: float(sub[f].std())  for f in raw_feat_avail}

        rows.append({
            "archetype_id":      int(c),
            "archetype_name":    archetype_names[c],
            "n_designs":         int(mask.sum()),
            "prevalence_pct":    round(float(mask.sum()) / len(labels) * 100, 1),
            "dominant_regime":   dominant_regime,
            "pct_easy":          round(regime_dist.get("EASY", 0) / mask.sum() * 100, 1),
            "pct_fragile":       round(regime_dist.get("FRAGILE", 0) / mask.sum() * 100, 1),
            "pct_infeasible":    round(regime_dist.get("INFEASIBLE", 0) / mask.sum() * 100, 1),
            "mean_boundary_distance": round(float(sub["boundary_distance"].mean()), 4),
            "limiting_factor_1": limiting[0][0] if len(limiting) > 0 else "",
            "limiting_factor_1_mean_shap": round(limiting[0][1], 5) if len(limiting) > 0 else 0,
            "limiting_factor_2": limiting[1][0] if len(limiting) > 1 else "",
            "limiting_factor_2_mean_shap": round(limiting[1][1], 5) if len(limiting) > 1 else 0,
            "limiting_factor_3": limiting[2][0] if len(limiting) > 2 else "",
            "limiting_factor_3_mean_shap": round(limiting[2][1], 5) if len(limiting) > 2 else 0,
            "enabling_factor_1": enabling[0][0] if len(enabling) > 0 else "",
            "enabling_factor_1_mean_shap": round(enabling[0][1], 5) if len(enabling) > 0 else 0,
            # Representative example: closest to centroid
            "example_rocket_id": _representative(sub, shap_means, features),
        })

    archetype_df = pd.DataFrame(rows).sort_values("n_designs", ascending=False).reset_index(drop=True)

    # Attach archetype labels back to local_df
    local_df = local_df.copy()
    local_df["archetype_id"] = labels
    local_df["archetype_name"] = [archetype_names[l] for l in labels]

    return local_df, archetype_df, clusterer, X_emb, X_scaled


def _representative(sub: pd.DataFrame, shap_means: dict, features: List[str]) -> str:
    """Return rocket_id of design closest to the cluster's mean explanation vector."""
    if len(sub) == 0:
        return ""
    cols = [f"delta_shap_{f}" for f in features if f"delta_shap_{f}" in sub.columns]
    mean_vec = np.array([shap_means[f] for f in features if f"delta_shap_{f}" in sub.columns])
    mat = sub[cols].values.astype(float)
    dists = np.linalg.norm(mat - mean_vec, axis=1)
    idx = sub.index[np.argmin(dists)]
    return str(sub.loc[idx, "rocket_id"])


def _name_archetypes(local_df: pd.DataFrame, features: List[str],
                      labels: np.ndarray, n_clusters: int) -> Dict[int, str]:
    """
    Assign human-readable names based on dominant limiting (negative delta_shap) factor.
    Names are intentionally engineering-meaningful.
    """
    FACTOR_NAMES = {
        "backlash":             "Precision-limited",
        "deadband":             "Deadband-limited",
        "p_unstable":           "Instability-driven",
        "servo_slew_deg_s":     "Actuator-speed-limited",
        "max_gimbal_deg":       "Authority-limited",
        "control_effectiveness":"Effectiveness-limited",
        "latency_steps":        "Latency-limited",
        "wind_strength":        "Disturbance-driven",
        "mass":                 "Inertia-limited",
        "Iyy":                  "Inertia-limited",
        "thrust":               "Thrust-limited",
        "static_margin":        "Geometry-driven",
        "Cm_alpha":             "Aero-coupled",
        "best_u_max_frac":      "Saturation-limited",
    }
    names = {}
    for c in sorted(set(labels)):
        mask = labels == c
        sub = local_df[mask]
        shap_means = {}
        for f in features:
            col = f"delta_shap_{f}"
            if col in sub.columns:
                shap_means[f] = float(sub[col].mean())
        if not shap_means:
            names[c] = f"Archetype-{c}"
            continue
        top_limiting = sorted(shap_means.items(), key=lambda x: x[1])  # most negative
        # Check if this is a mostly-EASY cluster
        easy_frac = float((sub["regime_label"] == "EASY").mean())
        if easy_frac > 0.70:
            names[c] = "Robust-favorable"
        else:
            primary = top_limiting[0][0]
            secondary = top_limiting[1][0] if len(top_limiting) > 1 else None
            p_name = FACTOR_NAMES.get(primary, primary)
            s_name = FACTOR_NAMES.get(secondary, secondary) if secondary else None
            # Coupled failure if top two are similarly negative
            if (secondary is not None and
                    abs(top_limiting[0][1]) > 0 and
                    abs(top_limiting[1][1] / top_limiting[0][1]) > 0.7):
                if p_name != s_name:
                    names[c] = f"{p_name} + {s_name} (coupled)"
                else:
                    names[c] = p_name
            else:
                names[c] = p_name
    return names


# ─────────────────────────── Step 5: Visualisations ───────────────────────────

def plot_archetype_prevalence(archetype_df: pd.DataFrame):
    """Fig 1: Bar chart of archetype prevalence."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Left: count bar
    ax = axes[0]
    colors = [PALETTE[i % len(PALETTE)] for i in range(len(archetype_df))]
    bars = ax.barh(archetype_df["archetype_name"], archetype_df["prevalence_pct"],
                    color=colors, edgecolor="k", linewidth=0.5)
    ax.set_xlabel("Prevalence (%)", fontsize=11)
    ax.set_title("Failure Archetype Prevalence", fontsize=13, fontweight="bold")
    ax.invert_yaxis()
    for bar, val in zip(bars, archetype_df["n_designs"]):
        ax.text(bar.get_width() + 0.3, bar.get_y() + bar.get_height() / 2,
                f"n={val}", va="center", fontsize=9)

    # Right: stacked bar (regime composition per archetype)
    ax2 = axes[1]
    y = np.arange(len(archetype_df))
    easy = archetype_df["pct_easy"].values
    frag = archetype_df["pct_fragile"].values
    infeas = archetype_df["pct_infeasible"].values
    ax2.barh(y, easy,  color=REGIME_COLOR["EASY"],       label="EASY",       edgecolor="k", lw=0.4)
    ax2.barh(y, frag,  left=easy,                        color=REGIME_COLOR["FRAGILE"],
             label="FRAGILE", edgecolor="k", lw=0.4)
    ax2.barh(y, infeas, left=easy + frag,                color=REGIME_COLOR["INFEASIBLE"],
             label="INFEASIBLE", edgecolor="k", lw=0.4)
    ax2.set_yticks(y)
    ax2.set_yticklabels(archetype_df["archetype_name"], fontsize=8)
    ax2.set_xlabel("Regime composition (%)", fontsize=11)
    ax2.set_title("Regime Composition per Archetype", fontsize=13, fontweight="bold")
    ax2.invert_yaxis()
    ax2.legend(loc="lower right", fontsize=9)

    plt.tight_layout()
    fig.savefig(OUT_DIR / "fig1_archetype_prevalence.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig1_archetype_prevalence.png")


def plot_archetype_transition_paths(local_df: pd.DataFrame, X_emb: np.ndarray,
                                     archetype_df: pd.DataFrame):
    """Fig 2: PCA embedding coloured by archetype, regime shown by marker shape."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    archetype_ids = sorted(local_df["archetype_id"].unique())
    regime_markers = {"EASY": "o", "FRAGILE": "s", "INFEASIBLE": "^"}
    arch_id_to_color = {aid: PALETTE[i % len(PALETTE)] for i, aid in enumerate(archetype_ids)}

    for ax_idx, (ax, color_by) in enumerate(zip(axes, ["regime", "archetype"])):
        for i, row in local_df.iterrows():
            regime = row["regime_label"]
            arch   = row["archetype_id"]
            marker = regime_markers.get(regime, "o")
            color  = REGIME_COLOR[regime] if color_by == "regime" else arch_id_to_color[arch]
            ax.scatter(X_emb[i, 0], X_emb[i, 1],
                        c=color, marker=marker, alpha=0.5, s=18, linewidths=0)

        # Archetype centroids
        for aid in archetype_ids:
            mask = local_df["archetype_id"] == aid
            cx, cy = X_emb[mask, 0].mean(), X_emb[mask, 1].mean()
            name = archetype_df.loc[archetype_df["archetype_id"] == aid, "archetype_name"].values
            label = name[0] if len(name) else str(aid)
            ax.text(cx, cy, label, fontsize=7, ha="center",
                     bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.7, lw=0.5))

        ax.set_xlabel("PCA-1", fontsize=11)
        ax.set_ylabel("PCA-2", fontsize=11)

        if color_by == "regime":
            ax.set_title("Regime Layout in Explanation Space", fontsize=12, fontweight="bold")
            patches = [mpatches.Patch(color=v, label=k) for k, v in REGIME_COLOR.items()]
        else:
            ax.set_title("Archetype Boundaries in Explanation Space", fontsize=12, fontweight="bold")
            patches = [mpatches.Patch(color=arch_id_to_color[aid], label=f"A{aid}") for aid in archetype_ids]
        ax.legend(handles=patches, fontsize=8, loc="upper right")

    plt.tight_layout()
    fig.savefig(OUT_DIR / "fig2_archetype_transition_paths.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig2_archetype_transition_paths.png")


def plot_dominant_factors_per_archetype(local_df: pd.DataFrame, features: List[str],
                                          archetype_df: pd.DataFrame):
    """Fig 3: Heatmap of mean delta_shap per archetype × feature."""
    archetype_ids = list(archetype_df["archetype_id"])
    arch_names    = list(archetype_df["archetype_name"])

    shap_mat = np.zeros((len(archetype_ids), len(features)))
    for i, aid in enumerate(archetype_ids):
        mask = local_df["archetype_id"] == aid
        sub  = local_df[mask]
        for j, f in enumerate(features):
            col = f"delta_shap_{f}"
            if col in sub.columns:
                shap_mat[i, j] = float(sub[col].mean())

    # Shorten feature labels
    FEAT_LABELS = {
        "p_unstable": "p_unstable",
        "servo_slew_deg_s": "slew_rate",
        "max_gimbal_deg": "max_gimbal",
        "best_u_max_frac": "u_max",
        "mass": "mass",
        "Iyy": "Iyy",
        "static_margin": "stab_margin",
        "Cm_alpha": "Cm_alpha",
        "control_effectiveness": "ctrl_eff",
        "thrust": "thrust",
        "deadband": "deadband",
        "backlash": "backlash",
        "latency_steps": "latency",
        "wind_strength": "wind",
    }
    feat_labels = [FEAT_LABELS.get(f, f) for f in features]

    vmax = max(abs(shap_mat).max(), 1e-6)
    fig, ax = plt.subplots(figsize=(max(10, len(features) * 0.7), max(5, len(archetype_ids) * 0.8)))
    im = ax.imshow(shap_mat, cmap="RdYlGn", vmin=-vmax, vmax=vmax, aspect="auto")
    ax.set_xticks(range(len(features)))
    ax.set_xticklabels(feat_labels, rotation=45, ha="right", fontsize=9)
    ax.set_yticks(range(len(archetype_ids)))
    ax.set_yticklabels([f"A{aid}: {n}" for aid, n in zip(archetype_ids, arch_names)], fontsize=9)
    ax.set_title("Mean Δ-SHAP per Archetype × Feature\n(Green = pushes toward EASY, Red = limiting factor)",
                  fontsize=12, fontweight="bold")
    cbar = plt.colorbar(im, ax=ax, shrink=0.8)
    cbar.set_label("Mean Δ-SHAP (EASY − INFEASIBLE)", fontsize=9)

    # Annotate cells
    for i in range(len(archetype_ids)):
        for j in range(len(features)):
            val = shap_mat[i, j]
            if abs(val) > vmax * 0.15:
                ax.text(j, i, f"{val:.3f}", ha="center", va="center",
                         fontsize=7, color="black")

    plt.tight_layout()
    fig.savefig(OUT_DIR / "fig3_dominant_factors_per_archetype.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig3_dominant_factors_per_archetype.png")


def plot_example_trajectories(local_df: pd.DataFrame, X_emb: np.ndarray,
                                features: List[str], archetype_df: pd.DataFrame):
    """
    Fig 4: For each archetype that contains INFEASIBLE designs,
    show the INFEASIBLE → FRAGILE → EASY arrow paths in PCA space,
    illustrating what change is needed.
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    regime_markers = {"EASY": "o", "FRAGILE": "s", "INFEASIBLE": "^"}
    archetype_ids = sorted(local_df["archetype_id"].unique())
    arch_id_to_color = {aid: PALETTE[i % len(PALETTE)] for i, aid in enumerate(archetype_ids)}

    for i, row in local_df.iterrows():
        regime = row["regime_label"]
        arch   = row["archetype_id"]
        marker = regime_markers.get(regime, "o")
        color  = arch_id_to_color[arch]
        alpha  = 0.3 if regime == "EASY" else 0.6
        ax.scatter(X_emb[i, 0], X_emb[i, 1],
                    c=color, marker=marker, alpha=alpha, s=20, linewidths=0)

    # Draw arrows from centroid of INFEASIBLE → FRAGILE → EASY per archetype
    for aid in archetype_ids:
        sub = local_df[local_df["archetype_id"] == aid]
        centroids = {}
        for regime in ["INFEASIBLE", "FRAGILE", "EASY"]:
            rmask = sub["regime_label"] == regime
            if rmask.sum() >= 3:
                ridx = np.where((local_df["archetype_id"] == aid) &
                                (local_df["regime_label"] == regime))[0]
                centroids[regime] = X_emb[ridx].mean(axis=0)

        path = [r for r in ["INFEASIBLE", "FRAGILE", "EASY"] if r in centroids]
        if len(path) >= 2:
            for k in range(len(path) - 1):
                r0, r1 = path[k], path[k + 1]
                x0, y0 = centroids[r0]
                x1, y1 = centroids[r1]
                ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                             arrowprops=dict(arrowstyle="->", color=arch_id_to_color[aid],
                                             lw=2.5, alpha=0.8))

    # Legend
    regime_patches = [mpatches.Patch(color=v, label=k) for k, v in REGIME_COLOR.items()]
    arch_patches = [mpatches.Patch(color=arch_id_to_color[aid],
                                    label=f"A{aid}: {archetype_df.loc[archetype_df['archetype_id']==aid,'archetype_name'].values[0] if len(archetype_df.loc[archetype_df['archetype_id']==aid])>0 else aid}")
                    for aid in archetype_ids[:6]]
    ax.legend(handles=arch_patches + regime_patches, fontsize=7, ncol=2, loc="upper right")
    ax.set_xlabel("PCA-1", fontsize=11)
    ax.set_ylabel("PCA-2", fontsize=11)
    ax.set_title("Transition Trajectories: INFEASIBLE → FRAGILE → EASY per Archetype\n"
                  "(Arrows show direction of improvement in explanation space)",
                  fontsize=12, fontweight="bold")
    plt.tight_layout()
    fig.savefig(OUT_DIR / "fig4_example_trajectories.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig4_example_trajectories.png")


def plot_archetype_regime_map(local_df: pd.DataFrame, features: List[str],
                               archetype_df: pd.DataFrame):
    """
    Fig 5: Archetype occupancy across the regime map.
    Uses the two most physically meaningful axes from Exp1: p_unstable and max_gimbal_deg.
    Scatterplot coloured by archetype, with regime shown as marker.
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle("Archetype Occupancy Across Design Space", fontsize=14, fontweight="bold")

    archetype_ids = sorted(local_df["archetype_id"].unique())
    arch_id_to_color = {aid: PALETTE[i % len(PALETTE)] for i, aid in enumerate(archetype_ids)}
    regime_markers = {"EASY": "o", "FRAGILE": "s", "INFEASIBLE": "^"}

    # Axis pairs to plot
    axis_pairs = [
        ("p_unstable", "max_gimbal_deg"),
        ("backlash", "servo_slew_deg_s"),
        ("mass", "control_effectiveness"),
        ("latency_steps", "wind_strength"),
    ]

    for ax, (xf, yf) in zip(axes.flat, axis_pairs):
        if xf not in local_df.columns or yf not in local_df.columns:
            ax.set_visible(False)
            continue

        for i, row in local_df.iterrows():
            regime = row["regime_label"]
            aid    = row["archetype_id"]
            marker = regime_markers.get(regime, "o")
            color  = arch_id_to_color[aid]
            ax.scatter(row[xf], row[yf],
                        c=color, marker=marker, alpha=0.5, s=18, linewidths=0)

        ax.set_xlabel(xf, fontsize=10)
        ax.set_ylabel(yf, fontsize=10)
        ax.set_title(f"{xf} vs {yf}", fontsize=10)

    # Shared legend bottom-right panel
    ax_leg = axes.flat[3]
    arch_patches  = [mpatches.Patch(color=arch_id_to_color[aid],
                                     label=f"A{aid}: {archetype_df.loc[archetype_df['archetype_id']==aid,'archetype_name'].values[0][:30] if len(archetype_df.loc[archetype_df['archetype_id']==aid])>0 else aid}")
                      for aid in archetype_ids]
    regime_patches = [plt.scatter([], [], marker=v, c="gray", s=40, label=k)
                       for k, v in regime_markers.items()]
    handles = arch_patches + regime_patches
    ax_leg.legend(handles=handles, fontsize=8, loc="center", ncol=1, frameon=True)
    ax_leg.axis("off")
    ax_leg.set_title("Legend", fontsize=10)

    plt.tight_layout()
    fig.savefig(OUT_DIR / "fig5_archetype_regime_map.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[OK] fig5_archetype_regime_map.png")


# ─────────────────────────── Main ───────────────────────────

def main():
    print("=" * 60)
    print("TVC Failure Archetype Analysis Pipeline")
    print("=" * 60)

    # Load data
    df, features = load_exp1()
    print(f"[INFO] Loaded {len(df)} designs, {len(features)} features")
    print(f"[INFO] Regime counts: {df['regime_label'].value_counts().to_dict()}")

    # ── Step 1: Local SHAP ──
    print("\n[STEP 1] Computing local SHAP explanation vectors...")
    local_df, clf, classes, shap_arr, explainer, le = compute_local_shap(df, features)

    # Merge raw feature columns into local_df for downstream use
    for f in features:
        if f in df.columns:
            local_df[f] = df[f].values

    local_df.to_csv(OUT_DIR / "local_explanations.csv", index=False)
    print(f"[OK] local_explanations.csv  ({len(local_df)} rows × {len(local_df.columns)} cols)")

    # ── Step 2: Improvement paths ──
    print("\n[STEP 2] Computing frontier improvement paths...")
    improvement_df = compute_improvement_paths(df, features, clf, le)
    improvement_df.to_csv(OUT_DIR / "design_improvement_paths.csv", index=False)
    print(f"[OK] design_improvement_paths.csv  ({len(improvement_df)} rows)")

    # ── Steps 3 & 4: Archetype discovery ──
    print("\n[STEP 3-4] Discovering failure archetypes...")
    local_df, archetype_df, clusterer, X_emb, X_scaled = discover_archetypes(local_df, features)
    archetype_df.to_csv(OUT_DIR / "archetype_summary.csv", index=False)
    print(f"[OK] archetype_summary.csv  ({len(archetype_df)} archetypes)")

    # Update local_explanations with archetype assignments
    local_df.to_csv(OUT_DIR / "local_explanations.csv", index=False)
    print(f"[OK] local_explanations.csv updated with archetype assignments")

    # Print archetype summary to console
    print("\n── Archetype Summary ──")
    print(archetype_df[["archetype_id", "archetype_name", "n_designs", "prevalence_pct",
                          "dominant_regime", "limiting_factor_1", "limiting_factor_2"]].to_string(index=False))

    # ── Step 5: Figures ──
    print("\n[STEP 5] Generating figures...")
    plot_archetype_prevalence(archetype_df)
    plot_archetype_transition_paths(local_df, X_emb, archetype_df)
    plot_dominant_factors_per_archetype(local_df, features, archetype_df)
    plot_example_trajectories(local_df, X_emb, features, archetype_df)
    plot_archetype_regime_map(local_df, features, archetype_df)

    print("\n" + "=" * 60)
    print("All outputs written to: outputs/archetype_analysis/")
    print("=" * 60)

    # ── Console summary ──
    print("\n── Key Findings ──")
    for _, arow in archetype_df.iterrows():
        print(f"  Archetype {arow['archetype_id']} [{arow['archetype_name']}]  "
              f"n={arow['n_designs']} ({arow['prevalence_pct']}%)  "
              f"dominant={arow['dominant_regime']}  "
              f"top_limit={arow['limiting_factor_1']}")


if __name__ == "__main__":
    main()
