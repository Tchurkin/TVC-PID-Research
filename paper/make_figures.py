"""
make_figures.py — Generate publication-quality PNGs for Exp4 and Exp5.

Run from project root:
    python paper/make_figures.py

Outputs to paper/figures/:
    exp4_fidelity_atlas.png         7-panel fidelity requirement heatmaps
    exp5_terrain_rms.png            4-panel RMS terrain over 2D design slices
    exp5_terrain_success.png        4-panel success probability terrain
    exp5_bottleneck_bars.png        bottleneck classification + best-direction bars
    exp5_diminishing_returns.png    1D performance vs parameter curves with knee detection
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor

ROOT    = Path(__file__).resolve().parent.parent
EXP_DIR = ROOT / "experiments" / "results"
FIG_DIR = ROOT / "paper" / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

sys.path.insert(0, str(ROOT / "sim"))

# ── Style ─────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family":      "sans-serif",
    "font.size":        10,
    "axes.titlesize":   10,
    "axes.labelsize":   9,
    "xtick.labelsize":  8,
    "ytick.labelsize":  8,
    "figure.dpi":       150,
    "axes.spines.top":  False,
    "axes.spines.right":False,
})

DESIGN_FEATURES = [
    "p_unstable", "control_effectiveness", "servo_slew_deg_s", "max_gimbal_deg",
    "deadband", "backlash", "latency_steps", "wind_strength", "mass", "Iyy", "thrust",
]

REGIME_COLORS = {"EASY": "#2ecc71", "FRAGILE": "#f39c12", "INFEASIBLE": "#e74c3c"}

ABLATION_MODULES = ["wind", "backlash", "slew", "latency", "sensor_noise", "thrust_var", "deadband"]
MODULE_LABELS    = {
    "wind":         "Wind / Gusts",
    "backlash":     "Backlash",
    "slew":         "Slew Rate Limit",
    "latency":      "Sensor Latency",
    "sensor_noise": "Sensor Noise",
    "thrust_var":   "Thrust Variation",
    "deadband":     "Deadband",
}

N_GRID = 60


# ── Helper: 2D surrogate grid ─────────────────────────────────────────────────

def _make_grid(
    df: pd.DataFrame,
    x_col: str,
    y_col: str,
    target_col: str,
    n_grid: int = N_GRID,
    rf_n: int = 300,
    seed: int = 42,
    is_classifier: bool = False,
    class_proba_idx: int = 1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Fit RF surrogate on df[DESIGN_FEATURES] → target_col.
    Evaluate over a 2D grid (x_col × y_col) at median values for all others.
    Returns (x_grid_1d, y_grid_1d, z_grid_2d).
    """
    # Drop rows with missing target
    valid = df.dropna(subset=[target_col] + DESIGN_FEATURES)
    X     = valid[DESIGN_FEATURES].values
    y     = valid[target_col].values

    if is_classifier:
        model = RandomForestClassifier(n_estimators=rf_n, random_state=seed, n_jobs=-1)
    else:
        model = RandomForestRegressor(n_estimators=rf_n, random_state=seed, n_jobs=-1)
    model.fit(X, y)

    x_idx = DESIGN_FEATURES.index(x_col)
    y_idx = DESIGN_FEATURES.index(y_col)

    medians = [float(valid[f].median()) for f in DESIGN_FEATURES]
    x_vals  = np.linspace(valid[x_col].quantile(0.02), valid[x_col].quantile(0.98), n_grid)
    y_vals  = np.linspace(valid[y_col].quantile(0.02), valid[y_col].quantile(0.98), n_grid)

    XX, YY = np.meshgrid(x_vals, y_vals)
    n_pts  = XX.size
    X_grid = np.tile(medians, (n_pts, 1))
    X_grid[:, x_idx] = XX.ravel()
    X_grid[:, y_idx] = YY.ravel()

    if is_classifier:
        proba = model.predict_proba(X_grid)
        cls_order = list(model.classes_)
        idx = cls_order.index(class_proba_idx) if class_proba_idx in cls_order else 0
        Z = proba[:, idx].reshape(XX.shape)
    else:
        Z = model.predict(X_grid).reshape(XX.shape)

    return x_vals, y_vals, Z


# ── Figure 1: Exp4 Fidelity Requirement Atlas ─────────────────────────────────

def fig_exp4_fidelity_atlas():
    path = EXP_DIR / "exp4_ablation_study_py.csv"
    if not path.exists():
        print(f"  [SKIP] {path} not found")
        return
    df = pd.read_csv(path)
    print(f"  Exp4 ablation: {len(df)} designs, {df['dominant_fidelity'].value_counts().to_dict()}")

    # Add median-based regime for coloring scatter
    regime_color = df["regime_label"].map(REGIME_COLORS).fillna("#aaaaaa")

    n_mods    = len(ABLATION_MODULES)
    ncols     = 4
    nrows     = (n_mods + ncols - 1) // ncols  # 2 rows of 4
    fig, axes = plt.subplots(nrows, ncols, figsize=(14, 7), constrained_layout=True)
    axes_flat = axes.ravel()

    x_col = "p_unstable"
    y_col = "servo_slew_deg_s"

    for i, module in enumerate(ABLATION_MODULES):
        ax     = axes_flat[i]
        target = f"decision_flip_{module}"

        # Fit RF surrogate: decision_flip → probability
        x_vals, y_vals, Z = _make_grid(df, x_col, y_col, target,
                                        is_classifier=True, class_proba_idx=1)

        im = ax.contourf(x_vals, y_vals, Z, levels=20,
                         cmap="RdYlGn_r", vmin=0, vmax=1)
        ax.contour(x_vals, y_vals, Z, levels=[0.5],
                   colors="white", linewidths=1.2, linestyles="--")

        # Overlay scatter: actual designs colored by regime
        ax.scatter(df[x_col], df[y_col], c=regime_color, s=4, alpha=0.35,
                   linewidths=0, zorder=3)

        ax.set_title(MODULE_LABELS[module], fontweight="bold")
        ax.set_xlabel("p_unstable  (1/s)")
        ax.set_ylabel("Slew  (deg/s)")

        # Decision flip rate annotation
        flip_rate = df[target].mean()
        ax.text(0.97, 0.04, f"{flip_rate:.0%} flip", transform=ax.transAxes,
                ha="right", fontsize=7.5, color="white",
                bbox=dict(fc="black", alpha=0.5, pad=2, boxstyle="round"))

        plt.colorbar(im, ax=ax, label="P(decision changes)")

    # Hide unused axes
    for j in range(n_mods, len(axes_flat)):
        axes_flat[j].set_visible(False)

    # Regime legend
    from matplotlib.lines import Line2D
    handles = [Line2D([0], [0], marker="o", color="w", markerfacecolor=c,
                      markersize=7, label=r)
               for r, c in REGIME_COLORS.items()]
    fig.legend(handles=handles, loc="lower right", ncol=3, fontsize=8,
               title="Regime", title_fontsize=8, framealpha=0.8)

    fig.suptitle(
        "Exp4 Fidelity Requirement Atlas\n"
        "P(removing this module changes engineering decision)   |   "
        "dashed white = 50% boundary   |   dots = sampled rockets",
        fontsize=9.5
    )
    out = FIG_DIR / "exp4_fidelity_atlas.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 2: Exp5 Terrain Maps (RMS error) ──────────────────────────────────

def fig_exp5_terrain():
    exp1 = EXP_DIR / "exp1_regime_index_py.csv"
    if not exp1.exists():
        print(f"  [SKIP] {exp1} not found")
        return
    df = pd.read_csv(exp1)
    print(f"  Exp1: {len(df)} designs for terrain surrogate")

    slices = [
        ("p_unstable",          "servo_slew_deg_s",       "rms_error_deg",    "RMS Error (deg)"),
        ("p_unstable",          "backlash",               "rms_error_deg",    "RMS Error (deg)"),
        ("servo_slew_deg_s",    "control_effectiveness",  "rms_error_deg",    "RMS Error (deg)"),
        ("p_unstable",          "wind_strength",          "rms_error_deg",    "RMS Error (deg)"),
    ]

    fig, axes = plt.subplots(1, 4, figsize=(16, 4.2), constrained_layout=True)

    for ax, (x_col, y_col, target, clabel) in zip(axes, slices):
        # Cap RMS at 30 deg for display clarity (diverged designs otherwise dominate)
        df_plot          = df.copy()
        df_plot[target]  = df_plot[target].clip(upper=30)

        x_vals, y_vals, Z = _make_grid(df_plot, x_col, y_col, target)
        Z = np.clip(Z, 0, 30)

        im = ax.contourf(x_vals, y_vals, Z, levels=20, cmap="RdYlGn_r", vmin=0, vmax=25)
        # Contour lines at regime boundaries
        ax.contour(x_vals, y_vals, Z, levels=[8, 15], colors=["white", "black"],
                   linewidths=[1.0, 1.4], linestyles=["--", "-"])

        ax.set_xlabel(_pretty(x_col))
        ax.set_ylabel(_pretty(y_col))

        title_x = _pretty(x_col)
        title_y = _pretty(y_col)
        ax.set_title(f"{title_x}  ×  {title_y}", fontweight="bold")
        plt.colorbar(im, ax=ax, label=clabel)

    # Annotation on first panel
    axes[0].text(0.04, 0.96, "white dash = EASY/FRAGILE  |  black = FRAGILE/INFEASIBLE",
                 transform=axes[0].transAxes, fontsize=6.5, va="top",
                 bbox=dict(fc="white", alpha=0.7, pad=2, boxstyle="round"))

    fig.suptitle(
        "Exp5 Performance Terrain  —  RMS tracking error (deg) over 2D design slices\n"
        "All other parameters held at population median.  RF surrogate, N=1200.",
        fontsize=9.5
    )
    out = FIG_DIR / "exp5_terrain_rms.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


def fig_exp5_terrain_success():
    exp1 = EXP_DIR / "exp1_regime_index_py.csv"
    if not exp1.exists():
        return
    df = pd.read_csv(exp1)

    # Binary success from nominal_success_rate >= 0.35
    df["success_bin"] = (df["nominal_success_rate"] >= 0.35).astype(int)

    slices = [
        ("p_unstable",       "servo_slew_deg_s",      "success_bin"),
        ("p_unstable",       "backlash",              "success_bin"),
        ("servo_slew_deg_s", "control_effectiveness", "success_bin"),
        ("p_unstable",       "wind_strength",         "success_bin"),
    ]
    slice_labels = [
        "Instability × Slew",
        "Instability × Backlash",
        "Slew × Control Authority",
        "Instability × Wind",
    ]

    fig, axes = plt.subplots(1, 4, figsize=(16, 4.2), constrained_layout=True)

    for ax, (x_col, y_col, target), label in zip(axes, slices, slice_labels):
        x_vals, y_vals, Z = _make_grid(df, x_col, y_col, target,
                                        is_classifier=True, class_proba_idx=1)

        im = ax.contourf(x_vals, y_vals, Z, levels=20, cmap="RdYlGn", vmin=0, vmax=1)
        ax.contour(x_vals, y_vals, Z, levels=[0.35, 0.80],
                   colors=["white", "black"], linewidths=[1.0, 1.4], linestyles=["--", "-"])
        ax.set_xlabel(_pretty(x_col))
        ax.set_ylabel(_pretty(y_col))
        ax.set_title(label, fontweight="bold")
        plt.colorbar(im, ax=ax, label="P(success)")

    axes[0].text(0.04, 0.96, "white dash = FRAGILE threshold  |  black = EASY threshold",
                 transform=axes[0].transAxes, fontsize=6.5, va="top",
                 bbox=dict(fc="white", alpha=0.7, pad=2, boxstyle="round"))

    fig.suptitle(
        "Exp5 Success Probability Terrain  —  P(success) over 2D design slices\n"
        "All other parameters held at population median.  RF surrogate, N=1200.",
        fontsize=9.5
    )
    out = FIG_DIR / "exp5_terrain_success.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 3: Exp5 Bottleneck Classification ──────────────────────────────────

def fig_exp5_bottleneck():
    grad_path = EXP_DIR / "exp5_gradient_field_py.csv"
    if not grad_path.exists():
        print(f"  [SKIP] {grad_path} not found")
        return
    df = pd.read_csv(grad_path)
    print(f"  Exp5 gradient field: {len(df)} designs")

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), constrained_layout=True)

    # ── Panel A: Topology distribution by regime ──────────────────────────
    ax = axes[0]
    topo_col = "topology_class_pop" if "topology_class_pop" in df.columns else "topology_class"
    topo_order  = ["bowl", "ridge", "cliff", "plateau"]
    regime_order = ["EASY", "FRAGILE", "INFEASIBLE"]
    topo_palette = {"bowl": "#3498db", "ridge": "#e67e22", "cliff": "#e74c3c", "plateau": "#95a5a6"}

    bottoms = np.zeros(len(regime_order))
    for topo in topo_order:
        vals = []
        for reg in regime_order:
            sub  = df[df["regime_label"] == reg]
            frac = (sub[topo_col] == topo).mean() if len(sub) > 0 else 0.0
            vals.append(frac)
        ax.bar(regime_order, vals, bottom=bottoms, color=topo_palette[topo],
               label=topo, width=0.6)
        bottoms += np.array(vals)

    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Fraction of designs")
    ax.set_title("Topology Class by Regime", fontweight="bold")
    ax.legend(loc="upper right", fontsize=7.5)
    for i, reg in enumerate(regime_order):
        ax.axhline(0, color="grey", lw=0.5)

    # ── Panel B: Best improvement direction (most common bottleneck) ──────
    ax = axes[1]
    param_label = {
        "servo_slew_deg_s":    "Slew Rate",
        "control_effectiveness": "Ctrl Authority",
        "backlash":            "Backlash",
        "wind_strength":       "Wind",
        "mass":                "Mass",
        "Iyy":                 "Moment Inertia",
        "thrust":              "Thrust",
        "max_gimbal_deg":      "Gimbal Range",
        "deadband":            "Deadband",
        "latency_steps":       "Latency",
        "static_margin":       "Static Margin",
        "Cm_alpha":            "Cm_alpha",
    }
    top_bottlenecks = df.groupby("regime_label")["best_param"].value_counts(normalize=True)

    x_pos   = 0
    bar_data: dict[str, dict[str, float]] = {}
    for reg in regime_order:
        if reg not in top_bottlenecks.index.get_level_values(0):
            continue
        sub = top_bottlenecks[reg].head(5)
        for param, frac in sub.items():
            if param not in bar_data:
                bar_data[param] = {r: 0.0 for r in regime_order}
            bar_data[param][reg] = frac

    params_sorted = sorted(bar_data.keys(),
                           key=lambda p: sum(bar_data[p].values()), reverse=True)[:8]
    x_pos  = np.arange(len(params_sorted))
    width  = 0.25
    colors = {"EASY": "#2ecc71", "FRAGILE": "#f39c12", "INFEASIBLE": "#e74c3c"}
    for j, reg in enumerate(regime_order):
        vals = [bar_data[p][reg] for p in params_sorted]
        ax.bar(x_pos + j * width, vals, width, label=reg, color=colors[reg], alpha=0.85)

    ax.set_xticks(x_pos + width)
    ax.set_xticklabels([param_label.get(p, p) for p in params_sorted],
                       rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("Fraction of designs in regime")
    ax.set_title("Most Frequent Bottleneck Parameter", fontweight="bold")
    ax.legend(fontsize=7.5)

    # ── Panel C: Gradient magnitude by regime (violin-style) ──────────────
    ax = axes[2]
    gmag_col = "grad_magnitude"
    for i, reg in enumerate(regime_order):
        sub = df[df["regime_label"] == reg][gmag_col].dropna()
        if len(sub) == 0:
            continue
        # Box plot
        bp = ax.boxplot(sub.clip(upper=sub.quantile(0.95)), positions=[i],
                        widths=0.5, patch_artist=True, notch=False,
                        medianprops=dict(color="white", linewidth=1.5),
                        boxprops=dict(facecolor=REGIME_COLORS[reg], alpha=0.7),
                        whiskerprops=dict(color="grey"), capprops=dict(color="grey"),
                        flierprops=dict(marker=".", markersize=2, alpha=0.3,
                                        markerfacecolor=REGIME_COLORS[reg]))
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels(regime_order)
    ax.set_ylabel("Gradient magnitude (range-scaled)")
    ax.set_title("Improvement Potential by Regime", fontweight="bold")
    ax.text(0.02, 0.97, "Higher = more improvement available",
            transform=ax.transAxes, fontsize=7.5, va="top", color="grey")

    fig.suptitle("Exp5 Design-Space Topology  —  Bottleneck Classification", fontsize=9.5)
    out = FIG_DIR / "exp5_bottleneck.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 4: Exp5 Diminishing Returns ───────────────────────────────────────

def fig_exp5_diminishing_returns():
    dr_path = EXP_DIR / "exp5_diminishing_returns_py.csv"
    if not dr_path.exists():
        print(f"  [SKIP] {dr_path} not found")
        return
    df = pd.read_csv(dr_path)
    if df.empty:
        print("  [SKIP] diminishing returns CSV empty")
        return

    params = list(df["param"].unique())
    n_params = min(len(params), 5)
    fig, axes = plt.subplots(1, n_params, figsize=(3.3 * n_params, 4), constrained_layout=True)
    if n_params == 1:
        axes = [axes]

    param_labels = {
        "servo_slew_deg_s":    "Servo Slew Rate (deg/s)",
        "control_effectiveness": "Control Effectiveness",
        "backlash":            "Backlash (code units)",
        "wind_strength":       "Wind Strength",
        "mass":                "Vehicle Mass (kg)",
        "Iyy":                 "Moment of Inertia Iyy (kg·m²)",
        "static_margin":       "Static Margin (calibers)",
        "Cm_alpha":            "Cm_alpha (deg)",
        "deadband":            "Deadband (code units)",
        "latency_steps":       "Latency (steps × 5 ms)",
        "thrust":              "Thrust (N)",
    }

    for ax, param in zip(axes, params[:n_params]):
        sub = df[df["param"] == param].sort_values("param_val")
        if sub.empty:
            ax.set_visible(False)
            continue

        x   = sub["param_val"].values
        ymed = sub["rms_median"].values
        ylo  = sub["rms_q25"].values
        yhi  = sub["rms_q75"].values

        # Median curve + IQR band (population spread)
        ax.fill_between(x, ylo, yhi, alpha=0.20, color="#2980b9", label="IQR (25–75%)")
        ax.plot(x, ymed, color="#1f4e79", lw=2.2, zorder=3, label="Median RMS")

        # Knee detection: largest drop in slope magnitude along monotone trend.
        # Use |second derivative| of the median curve, ignoring noisy endpoints.
        if len(ymed) >= 7:
            dy  = np.gradient(ymed, x)
            d2  = np.gradient(dy, x)
            knee_idx = int(np.argmax(np.abs(d2[2:-2]))) + 2
            ax.axvline(x[knee_idx], color="#e67e22", lw=1.4, ls="--", alpha=0.85, zorder=4)
            ax.text(x[knee_idx], yhi.max() * 0.97,
                    f" knee\n {x[knee_idx]:.3g}", fontsize=7, color="#e67e22", va="top")

        # Regime reference lines
        ax.axhline(8,  color="#2ecc71", lw=0.9, ls=":", alpha=0.7, label="EASY (8°)")
        ax.axhline(15, color="#e74c3c", lw=0.9, ls=":", alpha=0.7, label="INFEAS. (15°)")

        ax.set_xlabel(param_labels.get(param, param), fontsize=8)
        ax.set_ylabel("RMS error (deg)" if ax == axes[0] else "")
        ax.set_title(param_labels.get(param, param).split("(")[0].strip(), fontweight="bold")
        ax.set_ylim(bottom=0)
        if ax == axes[0]:
            ax.legend(fontsize=6.2, loc="upper right")

    fig.suptitle(
        "Exp5 Diminishing Returns  —  RMS error vs parameter (population sweep, N=100 designs at own tuned gains)\n"
        "blue = median across population  |  band = inter-quartile range  |  orange dashed = detected knee",
        fontsize=9.5
    )
    out = FIG_DIR / "exp5_diminishing_returns.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 5: Exp5 Evolution Path Overlay ────────────────────────────────────

def fig_exp5_evolution_paths():
    paths_path = EXP_DIR / "exp5_evolution_paths_py.csv"
    exp1_path  = EXP_DIR / "exp1_regime_index_py.csv"
    if not paths_path.exists():
        print(f"  [SKIP] {paths_path} not found")
        return

    paths = pd.read_csv(paths_path)
    exp1  = pd.read_csv(exp1_path) if exp1_path.exists() else None

    # Pick 2 representative designs per regime
    selected = []
    for reg in ["EASY", "FRAGILE", "INFEASIBLE"]:
        sub = paths[paths["regime_label"] == reg]["design_id"].unique()
        if len(sub) == 0:
            continue
        # Pick those with most metric improvement over path
        best_ids = []
        for did in sub[:30]:   # check up to 30 candidates
            p = paths[paths["design_id"] == did].sort_values("step")
            if len(p) < 2:
                continue
            delta = p.iloc[0]["metric_val"] - p.iloc[-1]["metric_val"]
            best_ids.append((delta, did))
        best_ids.sort(reverse=True)
        selected += [did for _, did in best_ids[:2]]

    if not selected:
        print("  [SKIP] no complete evolution paths found")
        return

    # ── Build RMS terrain as backdrop ────────────────────────────────────
    if exp1 is not None:
        df_bg = exp1.copy()
        df_bg["rms_cap"] = df_bg["rms_error_deg"].clip(upper=30)
        x_vals, y_vals, Z = _make_grid(df_bg, "p_unstable", "servo_slew_deg_s",
                                        "rms_cap", n_grid=50)
    else:
        x_vals = y_vals = Z = None

    n_sel = len(selected)
    ncols = min(n_sel, 6)
    nrows = (n_sel + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(3.5 * ncols, 3.8 * nrows),
                              constrained_layout=True)
    if n_sel == 1:
        axes = np.array([[axes]])
    elif nrows == 1:
        axes = axes.reshape(1, -1)

    # Check which columns are available in the paths CSV
    has_p_unstable = "p_unstable" in paths.columns
    # Compute p_unstable from stored design params if missing
    if not has_p_unstable and all(c in paths.columns for c in ["static_margin", "Cm_alpha", "thrust"]):
        sys.path.insert(0, str(ROOT / "sim"))
        from design_space import estimate_p_unstable as _est_p
        paths["p_unstable"] = paths.apply(
            lambda r: _est_p(r["static_margin"], r["Cm_alpha"], r["thrust"]), axis=1
        )
        has_p_unstable = True

    for i, did in enumerate(selected):
        ax  = axes.ravel()[i]
        p   = paths[paths["design_id"] == did].sort_values("step")
        reg = p.iloc[0]["regime_label"]

        # Background terrain
        if Z is not None:
            ax.contourf(x_vals, y_vals, Z, levels=15,
                        cmap="RdYlGn_r", vmin=0, vmax=25, alpha=0.65)
            ax.contour(x_vals, y_vals, Z, levels=[8, 15],
                       colors=["white", "black"], linewidths=[0.7, 1.0],
                       linestyles="--")

        # Evolution path
        px = p["p_unstable"].values if has_p_unstable else p["servo_slew_deg_s"].values
        py = p["servo_slew_deg_s"].values
        ax.plot(px, py, "o-", color="white", lw=2.0, ms=5, zorder=5,
                markerfacecolor="white")
        # Start marker
        ax.plot(px[0], py[0], "D", color="royalblue", ms=9, zorder=6,
                label="Start")
        # End marker
        ax.plot(px[-1], py[-1], "*", color="gold", ms=12, zorder=6,
                label="End")
        # Gradient arrow from start
        if len(px) >= 2:
            dx = px[1] - px[0]
            dy = py[1] - py[0]
            ax.annotate("", xy=(px[0] + dx * 0.6, py[0] + dy * 0.6),
                        xytext=(px[0], py[0]),
                        arrowprops=dict(arrowstyle="->", color="cyan", lw=1.5))

        rms_start = p.iloc[0]["metric_val"]
        rms_end   = p.iloc[-1]["metric_val"]
        ax.set_title(f"D{did}  [{reg}]\n"
                     f"RMS: {rms_start:.1f} → {rms_end:.1f} deg",
                     fontweight="bold", fontsize=8,
                     color=REGIME_COLORS.get(reg, "black"))
        ax.set_xlabel("p_unstable (1/s)" if has_p_unstable else "Slew (deg/s)", fontsize=8)
        ax.set_ylabel("Slew (deg/s)", fontsize=8)
        if i == 0:
            ax.legend(fontsize=7, loc="upper right")

    # Hide unused panels
    for j in range(n_sel, len(axes.ravel())):
        axes.ravel()[j].set_visible(False)

    fig.suptitle(
        "Exp5 Evolution Paths  —  Gradient-descent trajectory through design space\n"
        "Blue diamond = start  |  gold star = end  |  cyan arrow = initial gradient",
        fontsize=9.5
    )
    out = FIG_DIR / "exp5_evolution_paths.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Utility ───────────────────────────────────────────────────────────────────

def _pretty(col: str) -> str:
    m = {
        "p_unstable":           "p_unstable (1/s)",
        "servo_slew_deg_s":     "Servo slew (deg/s)",
        "control_effectiveness":"Ctrl effectiveness",
        "backlash":             "Backlash",
        "wind_strength":        "Wind strength",
        "mass":                 "Mass (kg)",
        "Iyy":                  "Iyy (kg·m²)",
        "thrust":               "Thrust (N)",
        "max_gimbal_deg":       "Gimbal range (deg)",
        "deadband":             "Deadband",
        "latency_steps":        "Latency (steps)",
    }
    return m.get(col, col)


# ── Trajectory figure ─────────────────────────────────────────────────────────

def fig_representative_trajectories():
    """
    Three example rocket simulations — one EASY, one FRAGILE, one INFEASIBLE.
    Shows pitch angle θ(t) and actuator command u(t) over the 3-second flight.

    This is the "show me what the simulation actually does" figure.
    It builds credibility by demonstrating that:
      - EASY rockets track the reference smoothly
      - FRAGILE rockets track but with higher error and more actuator effort
      - INFEASIBLE rockets diverge and crash

    Designs selected: median p_unstable in each regime to be representative.
    Full fidelity simulation, seed=42 for reproducibility.
    """
    from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario, estimate_p_unstable
    from controller import PIDParams
    from fidelity_config import FidelityConfig, apply_fidelity_config
    from simulator import simulate

    exp1_path = EXP_DIR / "exp1_regime_index_py.csv"
    if not exp1_path.exists():
        print("  Skipping trajectory figure: Exp1 results not found.")
        return

    df = pd.read_csv(exp1_path)

    # Pick the design closest to median p_unstable in each regime
    selected = {}
    for regime in ["EASY", "FRAGILE", "INFEASIBLE"]:
        sub = df[df["regime_label"] == regime].copy()
        if sub.empty:
            continue
        med  = sub["p_unstable"].median()
        idx  = (sub["p_unstable"] - med).abs().idxmin()
        selected[regime] = sub.loc[idx].to_dict()

    REGIME_COLORS = {"EASY": "#27ae60", "FRAGILE": "#f39c12", "INFEASIBLE": "#e74c3c"}
    fig, axes = plt.subplots(2, 3, figsize=(14, 7), constrained_layout=True,
                              sharex=True)

    for col, (regime, row) in enumerate(selected.items()):
        plant    = build_plant(row)
        base_act = build_actuator(row)
        base_sen = build_sensor(row)
        base_dis = build_disturbance(row)
        base_sc  = build_scenario(theta0_bias_std=0.0)

        cfg = FidelityConfig.full()
        act, sen, dis, sc = apply_fidelity_config(base_act, base_sen, base_dis, base_sc, cfg)
        pid = PIDParams(Kp=float(row["best_Kp"]), Kd=float(row["best_Kd"]),
                        Ki=0.0, u_max=base_act.u_max, i_lim=base_act.u_max)

        result = simulate(pid, plant, act, sen, dis, sc, seed=42)

        t     = result.t
        theta = result.theta_true     # in degrees (see simulator.py docstring)
        ref   = np.zeros_like(t)      # theta_ref = 0 for all designs (stabilisation task)
        ucmd  = result.u_act          # actuator command in code units
        color = REGIME_COLORS[regime]

        # Top row: attitude θ(t)
        ax = axes[0, col]
        ax.plot(t, ref,   color="gray",  lw=1.2, ls="--", label="Reference", zorder=2)
        ax.plot(t, theta, color=color,   lw=1.8, label="θ(t)", zorder=3)
        ax.axhline(0, color="black", lw=0.5, alpha=0.3)
        ax.set_ylabel("Pitch angle θ (deg)")
        p = float(row["p_unstable"])
        slew = float(row["servo_slew_deg_s"])
        rms  = result.rms_error_deg
        status = "GO" if result.success else "NO-GO"
        status_color = "#27ae60" if result.success else "#e74c3c"
        ax.set_title(
            f"{regime}  (p={p:.1f}/s, slew={slew:.0f} deg/s)\n"
            f"RMS={rms:.1f} deg  |  {status}",
            fontweight="bold", color=color
        )
        ax.text(0.98, 0.02, status, transform=ax.transAxes, ha="right", va="bottom",
                fontsize=11, fontweight="bold", color=status_color,
                bbox=dict(fc="white", alpha=0.85, pad=2, boxstyle="round"))
        ax.set_ylim(-35, 35)
        ax.axhspan(-30, 30, alpha=0.05, color="green", zorder=0)
        ax.axhline(30, color="gray", lw=0.8, ls=":", alpha=0.5)
        ax.axhline(-30, color="gray", lw=0.8, ls=":", alpha=0.5)
        if col == 0:
            ax.legend(fontsize=7.5, loc="upper right")
        ax.grid(alpha=0.2)

        # Bottom row: actuator command u(t)
        ax = axes[1, col]
        u_max = base_act.u_max
        ax.plot(t, ucmd,      color=color,   lw=1.5, zorder=3)
        ax.axhline(+u_max,  color="red",   lw=0.8, ls="--", alpha=0.6, label="±u_max")
        ax.axhline(-u_max,  color="red",   lw=0.8, ls="--", alpha=0.6)
        ax.axhline(0,        color="black", lw=0.5, alpha=0.3)
        ax.set_ylabel("Actuator command (code units)")
        ax.set_xlabel("Time (s)")
        sat_frac = result.u_cmd_sat_frac
        ax.set_title(f"u_sat = {sat_frac:.1%}", fontsize=9)
        ax.set_ylim(-u_max * 1.15, u_max * 1.15)
        if col == 0:
            ax.legend(fontsize=7.5, loc="upper right")
        ax.grid(alpha=0.2)

    fig.suptitle(
        "Representative Simulation Trajectories: EASY / FRAGILE / INFEASIBLE\n"
        "Designs at median p_unstable for each regime.  Full fidelity, seed=42.",
        fontsize=10,
    )
    out = FIG_DIR / "trajectories_by_regime.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Generating figures...")

    print("\n[1/6] Representative trajectories (EASY / FRAGILE / INFEASIBLE)")
    fig_representative_trajectories()

    print("\n[2/6] Exp4 Fidelity Atlas")
    fig_exp4_fidelity_atlas()

    print("\n[3/6] Exp5 Terrain (RMS)")
    fig_exp5_terrain()

    print("\n[4/6] Exp5 Terrain (Success probability)")
    fig_exp5_terrain_success()

    print("\n[5/6] Exp5 Bottleneck classification")
    fig_exp5_bottleneck()

    print("\n[6/6] Exp5 Diminishing returns")
    fig_exp5_diminishing_returns()

    if (EXP_DIR / "exp5_evolution_paths_py.csv").exists():
        print("\n[+] Exp5 Evolution paths")
        fig_exp5_evolution_paths()

    print(f"\nAll figures saved to {FIG_DIR}")
    for f in sorted(FIG_DIR.glob("*.png")):
        print(f"  {f.name}  ({f.stat().st_size // 1024} KB)")
