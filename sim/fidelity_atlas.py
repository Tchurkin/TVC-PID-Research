"""
fidelity_atlas.py — Continuous design-parameter atlas of fidelity sensitivity.

Answers: Given a rocket's physical design parameters, which simulator fidelity
modules are critical for accurate performance prediction?

Methodology
───────────
For each ablated module, a GradientBoostingRegressor is fit to the Exp4C CSV:
    rms_prediction_error = f(design_params)

The fitted model supports:
  - Feature importance     — which parameters predict fidelity sensitivity?
  - 1-D partial dependence — how does sensitivity vary along one parameter?
  - 2-D excerpt            — contour map over two parameters (others marginalised)
  - Parallel coordinates   — all parameters visible simultaneously, colored by error
  - SHAP summary           — signed per-feature effects (requires: pip install shap)
  - Atlas prediction       — for a new rocket, predict which modules are critical

Visualization strategy
──────────────────────
The design space is 12-D. Three complementary views:
  1. Feature importance bar charts  → which parameters matter (rank order)
  2. 1-D PDPs for top features      → functional shape of each dependency
  3. 2-D excerpts (user-chosen pairs) → the "where does X matter?" question
     Example: servo_slew_deg_s × max_gimbal_deg for the slew module
              backlash × p_unstable for the backlash module
              motor_scale × mass for dyn_aero

Usage
─────
  from fidelity_atlas import FidelityAtlas

  # Load and fit (uses exp4c_prediction_error_pd_py.csv by default)
  atlas = FidelityAtlas.from_csv()

  # Feature importance for slew
  atlas.feature_importance("slew")

  # 2-D excerpt: slew sensitivity as a function of servo speed and gimbal limit
  fig = atlas.plot_excerpt_2d("slew", "servo_slew_deg_s", "max_gimbal_deg")
  fig.savefig("atlas_slew_excerpt.png", dpi=150)

  # Full panel figure for one module
  fig = atlas.plot_module_panel("backlash")

  # Predict for a new design
  my_rocket = {"servo_slew_deg_s": 35, "max_gimbal_deg": 8, "motor_scale": 1.2, ...}
  atlas.predict_requirements(my_rocket)

CLI
───
  python sim/fidelity_atlas.py [pd|pid] [out_dir]
  Generates all panel figures and saves them to out_dir/atlas_figures/.
"""

from __future__ import annotations

import sys
import warnings
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from sklearn.ensemble import GradientBoostingRegressor
from sklearn.inspection import partial_dependence

ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = ROOT / "experiments" / "results"
FIGURES_DIR = ROOT / "outputs" / "atlas_figures"


# ── Feature set ───────────────────────────────────────────────────────────────

# All design parameters available in Exp4C rows.
# p_unstable included as a derived summary of aerodynamic instability.
ATLAS_FEATURES: list[str] = [
    "mass",
    "Iyy",
    "static_margin",
    "Cm_alpha",
    "control_effectiveness",
    "motor_scale",
    "servo_slew_deg_s",
    "max_gimbal_deg",
    "latency_steps",
    "deadband",
    "backlash",
    "wind_strength",
    "p_unstable",
]

FEATURE_LABELS: dict[str, str] = {
    "mass":                "Mass (kg)",
    "Iyy":                 "Iyy (kg·m²)",
    "static_margin":       "Static margin (cal.)",
    "Cm_alpha":            "Cm_alpha",
    "control_effectiveness": "Control eff. (rad/s²/cu)",
    "motor_scale":         "Motor scale",
    "servo_slew_deg_s":    "Servo slew (deg/s)",
    "max_gimbal_deg":      "Max gimbal (deg)",
    "latency_steps":       "Latency (steps)",
    "deadband":            "Deadband (cu)",
    "backlash":            "Backlash (cu)",
    "wind_strength":       "Wind strength",
    "p_unstable":          "p_unstable (1/s)",
}

# Default Exp4C module list (8 structural modules)
EXP4C_MODULES: list[str] = [
    "backlash", "slew", "latency", "deadband",
    "nonlinear_aero", "dyn_aero", "thrust_curve", "cg_shift",
]

# Physically motivated 2-D excerpt pairs for each module
DEFAULT_EXCERPT_PAIRS: dict[str, tuple[str, str]] = {
    "slew":         ("servo_slew_deg_s", "max_gimbal_deg"),
    "backlash":     ("backlash",         "p_unstable"),
    "dyn_aero":     ("motor_scale",      "mass"),
    "thrust_curve": ("motor_scale",      "static_margin"),
    "cg_shift":     ("motor_scale",      "Iyy"),
    "latency":      ("latency_steps",    "p_unstable"),
    "deadband":     ("deadband",         "servo_slew_deg_s"),
    "nonlinear_aero": ("static_margin",  "p_unstable"),
}


# ── Atlas class ───────────────────────────────────────────────────────────────

class FidelityAtlas:
    """
    Continuous fidelity-sensitivity atlas: design parameters → prediction error.

    Fits one GradientBoostingRegressor per module on the Exp4C data.
    All analysis and visualization methods operate on these fitted models.
    """

    def __init__(
        self,
        df:           pd.DataFrame,
        features:     list[str] = None,
        modules:      list[str] = None,
        n_estimators: int = 300,
        max_depth:    int = 4,
        learning_rate: float = 0.05,
        random_state: int = 42,
    ):
        self.df       = df[df["module_ablated"] != "reference"].copy()
        self.features = [f for f in (features or ATLAS_FEATURES) if f in df.columns]
        self.modules  = [m for m in (modules  or EXP4C_MODULES)
                         if m in df["module_ablated"].unique()]
        self.models:  dict[str, GradientBoostingRegressor] = {}
        self._n_est   = n_estimators
        self._depth   = max_depth
        self._lr      = learning_rate
        self._rng     = random_state
        self._fit_all()

    # ── Construction ─────────────────────────────────────────────────────────

    @classmethod
    def from_csv(
        cls,
        path: str | Path = None,
        controller_type: str = "pd",
        **kwargs,
    ) -> "FidelityAtlas":
        """
        Load from an Exp4C CSV.  Defaults to the PD result file.

        controller_type: "pd" or "pid" — selects which CSV file to load.
        """
        if path is None:
            ctype = controller_type.lower()
            path = RESULTS_DIR / f"exp4c_prediction_error_{ctype}_py.csv"
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(
                f"Exp4C CSV not found: {path}\n"
                f"Run: python sim/experiment_runner.py exp4c"
            )
        df = pd.read_csv(path)
        return cls(df, **kwargs)

    def _fit_all(self) -> None:
        """Fit one GBM per module.  Warns if a module has no rows."""
        for module in self.modules:
            sub = self.df[self.df["module_ablated"] == module]
            if sub.empty:
                warnings.warn(f"No rows for module '{module}' — skipping.")
                continue
            X = sub[self.features].values
            y = sub["rms_prediction_error"].values
            model = GradientBoostingRegressor(
                n_estimators  = self._n_est,
                max_depth     = self._depth,
                learning_rate = self._lr,
                subsample     = 0.8,
                random_state  = self._rng,
            )
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                model.fit(X, y)
            self.models[module] = model

    # ── Analysis ──────────────────────────────────────────────────────────────

    def feature_importance(self, module: str) -> pd.Series:
        """Feature importances for one module, sorted descending."""
        model = self._get_model(module)
        return (
            pd.Series(model.feature_importances_, index=self.features)
            .sort_values(ascending=False)
        )

    def partial_dependence_1d(
        self,
        module: str,
        feature: str,
        n_grid: int = 60,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        1-D partial dependence of rms_prediction_error on one feature.
        Returns (feature_values, pd_values).
        """
        model    = self._get_model(module)
        feat_idx = self.features.index(feature)
        X        = self._X(module)
        result   = partial_dependence(
            model, X=X, features=[feat_idx],
            grid_resolution=n_grid, kind="average",
        )
        return result["grid_values"][0], result["average"][0]

    def partial_dependence_2d(
        self,
        module: str,
        feature1: str,
        feature2: str,
        n_grid: int = 25,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        2-D partial dependence "excerpt".
        Returns (grid1, grid2, pd_values) where pd_values.shape = (n_grid, n_grid).
        """
        model  = self._get_model(module)
        f1_idx = self.features.index(feature1)
        f2_idx = self.features.index(feature2)
        X      = self._X(module)
        result = partial_dependence(
            model, X=X,
            features=[(f1_idx, f2_idx)],
            grid_resolution=n_grid, kind="average",
        )
        g1, g2 = result["grid_values"]
        return g1, g2, result["average"][0]

    def predict_requirements(
        self,
        design: dict,
        threshold_deg: float = 0.75,
    ) -> pd.Series:
        """
        For a new rocket design, predict which modules are critical.

        Returns a Series {module: predicted_rms_prediction_error_deg},
        sorted descending.  Modules above threshold_deg are 'required'.
        """
        x = np.array([[design.get(f, float("nan")) for f in self.features]])
        scores = {m: float(mdl.predict(x)[0]) for m, mdl in self.models.items()}
        result = pd.Series(scores).sort_values(ascending=False)
        print(f"\nFidelity requirement prediction (threshold = {threshold_deg} deg):")
        for mod, val in result.items():
            flag = "  ← REQUIRED" if val >= threshold_deg else ""
            print(f"  {mod:<20s}: {val:.2f} deg{flag}")
        return result

    # ── Plots ─────────────────────────────────────────────────────────────────

    def plot_feature_importance(
        self,
        modules: list[str] = None,
        top_n:   int = 6,
        figsize: tuple = None,
    ) -> plt.Figure:
        """Side-by-side importance bars for each module."""
        mods   = [m for m in (modules or self.modules) if m in self.models]
        ncols  = len(mods)
        figsize = figsize or (3 * ncols, 4)
        fig, axes = plt.subplots(1, ncols, figsize=figsize)
        if ncols == 1:
            axes = [axes]

        for ax, module in zip(axes, mods):
            imp = self.feature_importance(module).head(top_n)
            labels = [FEATURE_LABELS.get(f, f) for f in imp.index[::-1]]
            ax.barh(labels, imp.values[::-1], color="steelblue", edgecolor="white")
            ax.set_title(module.replace("_", " "), fontsize=9, fontweight="bold")
            ax.set_xlabel("Importance", fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(axis="x", alpha=0.3)

        fig.suptitle("Feature importances: RMS prediction error", fontsize=11, y=1.01)
        plt.tight_layout()
        return fig

    def plot_pdp_1d(
        self,
        module:   str,
        features: list[str] = None,
        n_grid:   int = 60,
        figsize:  tuple = None,
    ) -> plt.Figure:
        """1-D partial dependence curves for a module's top features."""
        if features is None:
            features = self.feature_importance(module).head(4).index.tolist()
        figsize = figsize or (3.5 * len(features), 3)
        fig, axes = plt.subplots(1, len(features), figsize=figsize)
        if len(features) == 1:
            axes = [axes]

        for ax, feat in zip(axes, features):
            x, y = self.partial_dependence_1d(module, feat, n_grid)
            ax.plot(x, y, lw=2, color="steelblue")
            ax.fill_between(x, y, alpha=0.12, color="steelblue")
            ax.axhline(y.mean(), color="gray", lw=0.8, ls="--", alpha=0.6)
            ax.set_xlabel(FEATURE_LABELS.get(feat, feat), fontsize=8)
            ax.set_ylabel("RMS error (deg)", fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.25)

        fig.suptitle(
            f"Partial dependence — {module.replace('_', ' ')} fidelity error",
            fontsize=10,
        )
        plt.tight_layout()
        return fig

    def plot_excerpt_2d(
        self,
        module:   str,
        feature1: str,
        feature2: str,
        n_grid:   int = 30,
        figsize:  tuple = (6, 5),
        cmap:     str = "hot_r",
        scatter_alpha: float = 0.12,
    ) -> plt.Figure:
        """
        2-D partial dependence contour map ('excerpt').

        Contours show predicted rms_prediction_error as feature1 and feature2
        vary across their design-space range.  All other parameters are
        marginalised out over the real data distribution (true PDP, not
        median-fixing).

        Scatter overlay shows actual data points, sized by error magnitude.
        """
        g1, g2, pd_vals = self.partial_dependence_2d(module, feature1, feature2, n_grid)

        fig, ax = plt.subplots(figsize=figsize)

        # Contour fill
        G1, G2 = np.meshgrid(g1, g2, indexing="ij")
        vmin, vmax = pd_vals.min(), pd_vals.max()
        cf = ax.contourf(G1, G2, pd_vals, levels=12, cmap=cmap,
                         vmin=vmin, vmax=vmax)
        cs = ax.contour(G1, G2, pd_vals, levels=8,
                        colors="k", linewidths=0.4, alpha=0.4)
        ax.clabel(cs, inline=True, fontsize=7, fmt="%.1f")
        plt.colorbar(cf, ax=ax, label="RMS prediction error (deg)")

        # Actual data scatter (where the real designs live in this 2-D slice)
        sub = self.df[self.df["module_ablated"] == module]
        if feature1 in sub.columns and feature2 in sub.columns:
            ax.scatter(
                sub[feature1], sub[feature2],
                c=sub["rms_prediction_error"],
                cmap=cmap, vmin=vmin, vmax=vmax,
                s=8, alpha=scatter_alpha, linewidths=0,
            )

        ax.set_xlabel(FEATURE_LABELS.get(feature1, feature1), fontsize=10)
        ax.set_ylabel(FEATURE_LABELS.get(feature2, feature2), fontsize=10)
        ax.set_title(
            f"{module.replace('_', ' ')} — fidelity atlas excerpt\n"
            f"(other parameters marginalised)",
            fontsize=9,
        )
        plt.tight_layout()
        return fig

    def plot_parallel_coordinates(
        self,
        module:   str,
        n_sample: int = 400,
        figsize:  tuple = (15, 4),
        cmap_name: str = "RdYlBu_r",
    ) -> plt.Figure:
        """
        Parallel coordinates: every design is a polyline across all feature axes,
        coloured by rms_prediction_error.  Reveals high-D clustering that PDPs miss.

        High-error designs (red) will cluster in specific feature-value bands —
        those bands identify the design regions where fidelity is critical.
        """
        sub = (
            self.df[self.df["module_ablated"] == module]
            .sample(min(n_sample, len(self.df[self.df["module_ablated"] == module])),
                    random_state=42)
            .copy()
        )

        # Drop integer/constant features for cleaner plot
        plot_feats = [f for f in self.features if f != "latency_steps"]

        # Normalise each feature to [0, 1]
        norm_df = sub[plot_feats].copy()
        feat_mins = norm_df.min()
        feat_ranges = norm_df.max() - feat_mins
        feat_ranges = feat_ranges.replace(0, 1)   # avoid division by zero
        norm_df = (norm_df - feat_mins) / feat_ranges

        err    = sub["rms_prediction_error"].values
        err_n  = (err - err.min()) / max(err.max() - err.min(), 1e-9)
        cmap   = matplotlib.colormaps.get_cmap(cmap_name)

        fig, ax = plt.subplots(figsize=figsize)
        x_pos = np.arange(len(plot_feats))

        for i in range(len(sub)):
            color = cmap(err_n[i])
            alpha = 0.15 + 0.5 * err_n[i]
            ax.plot(x_pos, norm_df.iloc[i][plot_feats].values,
                    color=color, alpha=alpha, lw=0.7)

        ax.set_xticks(x_pos)
        ax.set_xticklabels(
            [FEATURE_LABELS.get(f, f) for f in plot_feats],
            rotation=38, ha="right", fontsize=7,
        )
        ax.set_yticks([0, 0.5, 1.0])
        ax.set_yticklabels(["min", "mid", "max"], fontsize=7)
        ax.set_ylabel("Normalised value", fontsize=9)
        ax.set_title(
            f"{module.replace('_', ' ')} — parallel coordinates\n"
            f"(colour = RMS prediction error, red = high)",
            fontsize=9,
        )
        ax.grid(axis="x", alpha=0.2)

        sm = cm.ScalarMappable(cmap=cmap,
                               norm=plt.Normalize(err.min(), err.max()))
        sm.set_array([])
        plt.colorbar(sm, ax=ax, label="RMS prediction error (deg)", shrink=0.8)
        plt.tight_layout()
        return fig

    def plot_module_panel(
        self,
        module:  str,
        n_grid:  int = 25,
        figsize: tuple = None,
    ) -> plt.Figure:
        """
        Full-panel atlas figure for one module (4 sub-figures):
          [A] Feature importance  [B] 1-D PDP top-3 features  [C] 2-D excerpt
        """
        imp       = self.feature_importance(module)
        top3      = imp.head(3).index.tolist()
        f1, f2    = DEFAULT_EXCERPT_PAIRS.get(module, (top3[0], top3[1] if len(top3) > 1 else top3[0]))
        figsize   = figsize or (16, 4)
        fig       = plt.figure(figsize=figsize)
        gs        = fig.add_gridspec(1, 7, wspace=0.45)

        # [A] Importance
        ax_imp = fig.add_subplot(gs[0, :2])
        top6   = imp.head(6)
        labels = [FEATURE_LABELS.get(f, f) for f in top6.index[::-1]]
        ax_imp.barh(labels, top6.values[::-1], color="steelblue", edgecolor="white")
        ax_imp.set_xlabel("Importance", fontsize=8)
        ax_imp.set_title("Feature importance", fontsize=9)
        ax_imp.tick_params(labelsize=7)
        ax_imp.grid(axis="x", alpha=0.3)

        # [B] 1-D PDPs
        for i, feat in enumerate(top3):
            ax = fig.add_subplot(gs[0, 2 + i])
            x, y = self.partial_dependence_1d(module, feat, n_grid=50)
            ax.plot(x, y, lw=2, color="steelblue")
            ax.fill_between(x, y, alpha=0.12, color="steelblue")
            ax.set_xlabel(FEATURE_LABELS.get(feat, feat), fontsize=7)
            ax.set_ylabel("RMS err (deg)" if i == 0 else "", fontsize=7)
            ax.tick_params(labelsize=6)
            ax.grid(True, alpha=0.25)

        # [C] 2-D excerpt
        ax_2d = fig.add_subplot(gs[0, 5:])
        g1, g2, pv = self.partial_dependence_2d(module, f1, f2, n_grid)
        G1, G2     = np.meshgrid(g1, g2, indexing="ij")
        cf = ax_2d.contourf(G1, G2, pv, levels=10, cmap="hot_r")
        ax_2d.contour(G1, G2, pv, levels=6, colors="k", linewidths=0.4, alpha=0.4)
        plt.colorbar(cf, ax=ax_2d, label="RMS err (deg)", shrink=0.85)
        ax_2d.set_xlabel(FEATURE_LABELS.get(f1, f1), fontsize=7)
        ax_2d.set_ylabel(FEATURE_LABELS.get(f2, f2), fontsize=7)
        ax_2d.tick_params(labelsize=6)
        ax_2d.set_title(f"{f1.replace('_',' ')} × {f2.replace('_',' ')}", fontsize=7)

        fig.suptitle(
            f"Fidelity Atlas — {module.replace('_', ' ')} module",
            fontsize=12, fontweight="bold",
        )
        return fig

    def plot_shap_summary(
        self,
        module:   str,
        n_sample: int = 500,
        figsize:  tuple = (8, 5),
    ) -> plt.Figure:
        """
        SHAP beeswarm summary plot.
        Requires:  pip install shap

        Shows each feature's signed contribution to prediction error across all
        designs.  The best single figure for high-D intuition: direction,
        magnitude, and feature value (color) simultaneously.
        """
        try:
            import shap
        except ImportError:
            raise ImportError(
                "shap is not installed.\n"
                "Install with:  pip install shap\n"
                "or skip this plot and use plot_parallel_coordinates() instead."
            )
        sub = self.df[self.df["module_ablated"] == module]
        X_df = sub[self.features].sample(
            min(n_sample, len(sub)), random_state=42
        )
        model     = self._get_model(module)
        explainer = shap.TreeExplainer(model)
        sv        = explainer.shap_values(X_df.values)

        fig, ax = plt.subplots(figsize=figsize)
        shap.summary_plot(
            sv, X_df.values,
            feature_names=[FEATURE_LABELS.get(f, f) for f in self.features],
            show=False,
            plot_type="dot",
        )
        ax = plt.gca()
        ax.set_title(
            f"SHAP summary — {module.replace('_',' ')} fidelity error",
            fontsize=10,
        )
        plt.tight_layout()
        return plt.gcf()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_model(self, module: str) -> GradientBoostingRegressor:
        if module not in self.models:
            raise KeyError(
                f"No model for module '{module}'. "
                f"Available: {list(self.models.keys())}"
            )
        return self.models[module]

    def _X(self, module: str) -> np.ndarray:
        return self.df[self.df["module_ablated"] == module][self.features].values


# ── Fidelity Requirement Atlas (decision-error, two-stage) ────────────────────

REGIME_FEATURES: list[str] = [
    "Iyy", "wind_strength", "motor_scale", "servo_slew_deg_s",
    "max_gimbal_deg", "static_margin", "mass",
]

REGIME_ORDER:  list[str] = ["EASY", "MARGINAL", "FRAGILE", "INFEASIBLE"]
REGIME_COLORS: dict[str, str] = {
    "EASY":       "#2196F3",
    "MARGINAL":   "#FF9800",
    "FRAGILE":    "#F44336",
    "INFEASIBLE": "#9C27B0",
}

# Decision error rate above this → module is "required"
REQUIRED_THRESHOLD = 0.20


class FidelityRequirementAtlas:
    """
    Two-stage atlas: physical rocket parameters → predicted regime → required modules.

    Stage 1 — RandomForest trained on Exp1 data predicts regime from physical params.
    Stage 2 — Frequency table from Exp4C gives GO/NOGO decision error rate per
              (regime × stability class × module).

    This is the actionable builder tool: given measurements of your rocket,
    it tells you which fidelity modules your simulator must include.

    Usage
    ─────
      fra = FidelityRequirementAtlas.from_csv()

      # Predict for a specific rocket
      fra.predict({"Iyy": 0.018, "wind_strength": 0.28, "motor_scale": 1.2,
                   "servo_slew_deg_s": 45, "max_gimbal_deg": 8,
                   "static_margin": 0.12, "mass": 0.55})

      # Generate the main atlas figures
      fra.plot_heatmap().savefig("atlas_heatmap.png", dpi=150, bbox_inches="tight")
      fra.plot_design_space().savefig("atlas_design_space.png", dpi=150, bbox_inches="tight")
    """

    def __init__(self, exp1_df: pd.DataFrame, exp4c_df: pd.DataFrame):
        self.exp1_df  = exp1_df
        self.abl_df   = exp4c_df[exp4c_df["module_ablated"] != "reference"].copy()
        self.abl_df["is_stable"] = self.abl_df["static_margin"] < 0.0
        self._modules = sorted(self.abl_df["module_ablated"].unique())

        self._clf_features: list[str] = []
        self._clf = self._fit_regime_classifier()
        self._freq = self._build_frequency_table()

    # ── Construction ─────────────────────────────────────────────────────────

    @classmethod
    def from_csv(
        cls,
        exp1_path:       str | Path = None,
        exp4c_path:      str | Path = None,
        controller_type: str = "pd",
    ) -> "FidelityRequirementAtlas":
        if exp1_path is None:
            exp1_path = RESULTS_DIR / "exp1_regime_index_py.csv"
        if exp4c_path is None:
            exp4c_path = RESULTS_DIR / f"exp4c_prediction_error_{controller_type.lower()}_py.csv"
        for p in (exp1_path, exp4c_path):
            if not Path(p).exists():
                raise FileNotFoundError(f"Required CSV not found: {p}")
        return cls(pd.read_csv(exp1_path), pd.read_csv(exp4c_path))

    def _fit_regime_classifier(self):
        from sklearn.ensemble import RandomForestClassifier
        feats = [f for f in REGIME_FEATURES if f in self.exp1_df.columns]
        self._clf_features = feats
        X = self.exp1_df[feats].to_numpy(dtype=float)
        y = self.exp1_df["regime_label"].to_numpy()
        clf = RandomForestClassifier(n_estimators=300, max_depth=None,
                                      random_state=42, n_jobs=-1)
        clf.fit(X, y)
        return clf

    def _build_frequency_table(self) -> pd.DataFrame:
        records = []
        for regime in REGIME_ORDER:
            for stable in [True, False]:
                sub = self.abl_df[
                    (self.abl_df["regime_label"] == regime) &
                    (self.abl_df["is_stable"]    == stable)
                ]
                if sub.empty:
                    continue
                for module in self._modules:
                    m = sub[sub["module_ablated"] == module]
                    if m.empty:
                        continue
                    records.append(dict(
                        regime             = regime,
                        is_stable          = stable,
                        module             = module,
                        decision_error_rate= m["decision_error"].mean(),
                        rms_error_mean     = m["rms_prediction_error"].mean(),
                        n                  = len(m),
                    ))
        return pd.DataFrame(records)

    # ── Prediction ────────────────────────────────────────────────────────────

    def predict_regime(self, rocket_params: dict) -> tuple[str, dict]:
        """Return (predicted_regime, {regime: probability}) from physical params."""
        x = np.array([[rocket_params.get(f, 0.0) for f in self._clf_features]])
        regime = self._clf.predict(x)[0]
        probs  = dict(zip(self._clf.classes_,
                          self._clf.predict_proba(x)[0]))
        return regime, probs

    def predict(
        self,
        rocket_params: dict,
        verbose: bool = True,
    ) -> tuple[str, pd.DataFrame]:
        """
        Full two-stage prediction.

        rocket_params: dict with physical parameters (see REGIME_FEATURES).
          Include 'static_margin' < 0 for stable (finned) designs, > 0 for unstable.

        Returns (predicted_regime, requirement_table).
        """
        is_stable = rocket_params.get("static_margin", 0.0) < 0.0
        regime, probs = self.predict_regime(rocket_params)
        stab_label = "stable (finned)" if is_stable else "unstable (finless)"

        table = self._freq[
            (self._freq["regime"]    == regime) &
            (self._freq["is_stable"] == is_stable)
        ].sort_values("decision_error_rate", ascending=False).reset_index(drop=True)

        required = table.loc[
            table["decision_error_rate"] >= REQUIRED_THRESHOLD, "module"
        ].tolist()

        if verbose:
            w = 58
            print(f"\n{'='*w}")
            print(f"  Fidelity Requirement Atlas")
            print(f"{'='*w}")
            print(f"  Predicted regime : {regime}  ({probs.get(regime,0)*100:.0f}% confidence)")
            print(f"  Stability class  : {stab_label}")
            print(f"\n  Module decision error rates  (>={REQUIRED_THRESHOLD*100:.0f}% -> REQUIRED)")
            print(f"  {'Module':<22} {'Decision err':>14}  {'n':>5}  Status")
            print(f"  {'-'*(w-2)}")
            for _, row in table.iterrows():
                flag = "<-- REQUIRED" if row["decision_error_rate"] >= REQUIRED_THRESHOLD else ""
                print(f"  {row['module']:<22} {row['decision_error_rate']*100:12.1f}%  "
                      f"{int(row['n']):5d}  {flag}")
            print(f"\n  Minimum viable sim must include:  {required or ['none -- simple sim OK']}")
            print(f"{'='*w}\n")

        return regime, table

    def regime_classifier_accuracy(self) -> float:
        """5-fold CV accuracy of the regime classifier (for reporting)."""
        from sklearn.model_selection import cross_val_score
        X = self.exp1_df[self._clf_features].to_numpy(dtype=float)
        y = self.exp1_df["regime_label"].to_numpy()
        scores = cross_val_score(self._clf, X, y, cv=5, scoring="accuracy")
        return float(scores.mean())

    # ── Figures ───────────────────────────────────────────────────────────────

    def plot_heatmap(self, figsize: tuple = (13, 5)) -> plt.Figure:
        """
        Main atlas figure: heatmap of GO/NOGO decision error rate by
        regime × module, side-by-side for stable vs unstable rockets.

        Cell color = fraction of designs where omitting that module causes
        a wrong GO/NOGO engineering decision.
        """
        from matplotlib.colors import LinearSegmentedColormap

        mod_order = ["slew", "backlash", "dyn_aero", "thrust_curve",
                     "latency", "deadband", "cg_shift", "nonlinear_aero"]
        mods = [m for m in mod_order if m in self._modules]

        cmap = LinearSegmentedColormap.from_list(
            "fidelity", ["#f7fbff", "#6baed6", "#08306b"]
        )
        fig, axes = plt.subplots(1, 2, figsize=figsize)

        for ax, (stable, title) in zip(
            axes, [(True, "Stable (finned)"), (False, "Unstable (finless)")]
        ):
            mat   = np.full((len(REGIME_ORDER), len(mods)), np.nan)
            annot = np.empty_like(mat, dtype=object)

            for i, regime in enumerate(REGIME_ORDER):
                for j, module in enumerate(mods):
                    row = self._freq[
                        (self._freq["regime"]    == regime) &
                        (self._freq["is_stable"] == stable) &
                        (self._freq["module"]    == module)
                    ]
                    if not row.empty:
                        v = row["decision_error_rate"].values[0]
                        n = int(row["n"].values[0])
                        mat[i, j]   = v
                        annot[i, j] = f"{v*100:.0f}%\n(n={n})"

            im = ax.imshow(mat, cmap=cmap, vmin=0, vmax=1, aspect="auto")
            for i in range(len(REGIME_ORDER)):
                for j in range(len(mods)):
                    if not np.isnan(mat[i, j]):
                        color = "white" if mat[i, j] > 0.55 else "black"
                        ax.text(j, i, annot[i, j], ha="center", va="center",
                                fontsize=7.5, color=color)

            ax.set_xticks(range(len(mods)))
            ax.set_xticklabels([m.replace("_", "\n") for m in mods], fontsize=8)
            ax.set_yticks(range(len(REGIME_ORDER)))
            ax.set_yticklabels(REGIME_ORDER, fontsize=9, fontweight="bold")
            ax.set_title(title, fontsize=11, fontweight="bold", pad=8)
            plt.colorbar(im, ax=ax, label="Decision error rate", shrink=0.82)

        fig.suptitle(
            "Fidelity Requirement Atlas\n"
            "Decision error rate when module is omitted from engineer's simulator",
            fontsize=11, y=1.02,
        )
        plt.tight_layout()
        return fig

    def plot_design_space(self, figsize: tuple = (11, 4.5)) -> plt.Figure:
        """
        Two-panel design-space figure:
          Left  — (Iyy, wind_strength) scatter colored by regime (Exp1, n=1200)
          Right — same space colored by slew decision error (Exp4C)
        """
        fig, axes = plt.subplots(1, 2, figsize=figsize)

        # Left: controllability map
        ax = axes[0]
        for regime in REGIME_ORDER:
            sub = self.exp1_df[self.exp1_df["regime_label"] == regime]
            ax.scatter(sub["Iyy"], sub["wind_strength"],
                       c=REGIME_COLORS[regime], s=5, alpha=0.55, label=regime, linewidths=0)
        ax.set_xlabel("Iyy (kg·m²)", fontsize=9)
        ax.set_ylabel("Wind strength", fontsize=9)
        ax.set_title("Controllability map (Exp1, n=1200)", fontsize=10, fontweight="bold")
        ax.legend(fontsize=8, markerscale=2.5, framealpha=0.85)
        ax.grid(True, alpha=0.2)

        # Right: slew sensitivity in the same space
        ax = axes[1]
        slew = self.abl_df[self.abl_df["module_ablated"] == "slew"]
        sc = ax.scatter(slew["Iyy"], slew["wind_strength"],
                        c=slew["decision_error"].astype(float),
                        cmap="RdYlGn_r", s=18, alpha=0.8, vmin=0, vmax=1, linewidths=0)
        plt.colorbar(sc, ax=ax, label="Slew causes decision error (1=yes)")
        ax.set_xlabel("Iyy (kg·m²)", fontsize=9)
        ax.set_ylabel("Wind strength", fontsize=9)
        ax.set_title("Slew fidelity sensitivity (Exp4C stratified)", fontsize=10, fontweight="bold")
        ax.grid(True, alpha=0.2)

        plt.tight_layout()
        return fig

    # ── Derived physical parameters ───────────────────────────────────────────

    @staticmethod
    def _add_derived(df: pd.DataFrame) -> pd.DataFrame:
        """
        Add physically interpretable derived columns to any design dataframe.

        Derived parameters
        ──────────────────
        control_authority   — max angular acceleration the controller can produce
                              = control_effectiveness × 12  [rad/s²]
                              (12 = u_max code units; full gimbal deflection)

        effective_wind      — actual disturbance angular acceleration this rocket
                              experiences in its specified wind environment.
                              = wind_strength × (Iyy_ref/Iyy) / max(0.5, m/m_ref)
                              where Iyy_ref=0.018 kg·m², m_ref=1.2 kg
                              [rad/s²]
                              Identical to d_eff formula in disturbance_model.py.

        authority_ratio     — control_authority / effective_wind
                              >  10  : wind is negligible; easy regime expected
                              5 – 10 : wind loads are meaningful; tuning-sensitive
                              <   5  : wind-dominated; FRAGILE/INFEASIBLE risk
                              [dimensionless]

        slew_bandwidth_hz   — approximate maximum servo bandwidth
                              = servo_slew_deg_s / 360   [Hz]
                              (one full revolution equivalent; true bandwidth
                               depends on command amplitude but useful for ranking)

        wind_desc           — Qualitative Beaufort-like label for effective_wind
                              Calibrated to match disturbance model range.
        """
        d = df.copy()
        IYY_REF, M_REF = 0.018, 1.2

        d["control_authority"] = d["control_effectiveness"] * 12.0

        inertia_scale = IYY_REF / d["Iyy"].clip(lower=1e-4)
        mass_scale    = (d["mass"] / M_REF).clip(lower=0.5)
        d["effective_wind"]   = d["wind_strength"] * inertia_scale / mass_scale
        d["authority_ratio"]  = d["control_authority"] / d["effective_wind"].clip(lower=1e-6)
        d["slew_bandwidth_hz"] = d["servo_slew_deg_s"] / 360.0

        def _wind_desc(v):
            if v < 0.20:   return "Calm (<0.20)"
            elif v < 0.35: return "Moderate (0.20-0.35)"
            else:          return "Gusty (>0.35)"
        d["wind_desc"] = d["effective_wind"].apply(_wind_desc)

        return d

    @staticmethod
    def _design_fidelity_tiers(exp4c_df: pd.DataFrame) -> pd.DataFrame:
        """
        Compute per-design fidelity tier from Exp4C decision errors.

        Tier encoding (number of modules beyond slew that are required):
          Tier 0 — no modules required at all (simple sim OK)
          Tier 1 — slew only required
          Tier 2 — slew + 1 other
          Tier 3 — slew + 2 or more others
          (slew-absent tiers are collapsed into the nearest integer count)

        Returns a DataFrame with one row per design:
          design_id, Iyy, wind_strength, servo_slew_deg_s, mass,
          control_effectiveness, max_gimbal_deg, static_margin,
          regime_label, required_modules (frozenset), fidelity_tier (int),
          n_required (int), has_slew (bool)
        """
        abl = exp4c_df[exp4c_df["module_ablated"] != "reference"]
        rows = []
        for did, grp in abl.groupby("design_id"):
            req = set(grp[grp["decision_error"] == 1]["module_ablated"].tolist())
            n   = len(req)
            tier = (
                0 if n == 0 else
                1 if req == {"slew"} else
                2 if n == 2 and "slew" in req else
                3 if n >= 3 and "slew" in req else
                2  # non-slew modules, treat as 2
            )
            meta = grp.iloc[0]
            rows.append(dict(
                design_id         = did,
                regime_label      = meta["regime_label"],
                static_margin     = meta.get("static_margin", 0),
                Iyy               = meta["Iyy"],
                wind_strength     = meta["wind_strength"],
                servo_slew_deg_s  = meta["servo_slew_deg_s"],
                mass              = meta["mass"],
                control_effectiveness = meta["control_effectiveness"],
                max_gimbal_deg    = meta["max_gimbal_deg"],
                motor_scale       = meta.get("motor_scale", 1.0),
                required_modules  = sorted(req),
                n_required        = n,
                has_slew          = "slew" in req,
                fidelity_tier     = tier,
            ))
        return pd.DataFrame(rows)

    def plot_3d_fidelity_space(
        self,
        out_path: str | Path = None,
        open_browser: bool = False,
    ) -> "plotly.graph_objects.Figure":
        """Alias kept for backwards compatibility. Calls plot_3d_full_interactive."""
        return self.plot_3d_full_interactive(out_path=out_path, open_browser=open_browser)

    def plot_3d_full_interactive(
        self,
        out_path: str | Path = None,
        open_browser: bool = False,
    ) -> "plotly.graph_objects.Figure":
        """
        Fully interactive 3D fidelity atlas — standalone HTML.

        Features
        ────────
        Module dropdown  — pick any physics module; points color blue→red by how
                           likely omitting that module causes a wrong GO/NOGO call.
                           (blue = not needed, red = required).
                           Hover text shows ALL 8 modules simultaneously so you
                           can see designs that need aero but not slew, etc.

        Axis dropdowns   — X / Y / Z can be set to any of 12 physical parameters.
                           Defaults: Iyy × Eff.wind × Servo slew.

        Point encoding   — Large solid = Exp4C measured (actual data).
                           Small hollow = atlas-predicted (regime frequency table).

        Parameters available for axes
        ──────────────────────────────
          Iyy · effective_wind · servo_slew_deg_s · authority_ratio
          wind_strength · motor_scale · mass · static_margin
          control_effectiveness · max_gimbal_deg · slew_bandwidth_hz · p_unstable

        Saved as standalone interactive HTML (no server needed).
        """
        import plotly.graph_objects as go

        # ── Constants ─────────────────────────────────────────────────────────
        MODULES = ["slew", "backlash", "latency", "deadband",
                   "nonlinear_aero", "dyn_aero", "thrust_curve", "cg_shift"]
        MODULE_LABELS = {
            "slew":          "Servo slew",
            "backlash":      "Backlash",
            "latency":       "Latency",
            "deadband":      "Deadband",
            "nonlinear_aero":"Nonlinear aero",
            "dyn_aero":      "Dynamic aero",
            "thrust_curve":  "Thrust curve",
            "cg_shift":      "CG shift",
        }
        REGIME_SYMBOLS = {
            "EASY": "circle", "MARGINAL": "square",
            "FRAGILE": "diamond", "INFEASIBLE": "cross",
        }

        PARAMS = [
            ("Iyy",                   "Iyy (kg·m²)"),
            ("wind_strength",         "Wind strength (rad/s²)"),
            ("static_margin",         "Static margin (calibers)"),
            ("motor_scale",           "Motor scale"),
            ("max_gimbal_deg",        "Max gimbal (deg)"),
            ("servo_slew_deg_s",      "Servo slew (deg/s)"),
            ("authority_ratio",       "Authority ratio (ctrl/wind)"),
            ("control_effectiveness", "Control effectiveness (rad/s²/CU)"),
            ("mass",                  "Mass (kg)"),
            ("effective_wind",        "Eff. wind disturbance (rad/s²)"),
        ]

        # ── Step 1: Build unified 1200-row dataset ────────────────────────────
        exp1_ext = self._add_derived(self.exp1_df.copy())

        # Per-module requirement lookup from Exp4C (match by floating-point key)
        def _fkey(r):
            return (round(float(r["Iyy"]),4),
                    round(float(r["wind_strength"]),4),
                    round(float(r["servo_slew_deg_s"]),2))

        exp4c_lookup: dict = {}
        for did, grp in self.abl_df.groupby("design_id"):
            k = _fkey(grp.iloc[0])
            exp4c_lookup[k] = {
                row["module_ablated"]: int(row["decision_error"])
                for _, row in grp.iterrows()
            }

        def _module_probs(row):
            k = _fkey(row)
            if k in exp4c_lookup:
                m = exp4c_lookup[k]
                return {mod: float(m.get(mod, 0)) for mod in MODULES}, True
            is_stable = row["static_margin"] < 0.0
            sub = self._freq[
                (self._freq["regime"]    == row["regime_label"]) &
                (self._freq["is_stable"] == is_stable)
            ]
            probs = {}
            for mod in MODULES:
                r = sub[sub["module"] == mod]
                probs[mod] = float(r["decision_error_rate"].values[0]) if not r.empty else 0.0
            return probs, False

        rec_rows = []
        for _, row in exp1_ext.iterrows():
            probs, measured = _module_probs(row)
            r = row.to_dict()
            for mod in MODULES:
                r[f"req_{mod}"] = probs[mod]
            r["measured"]  = measured
            req_list = [mod for mod in MODULES if probs[mod] >= REQUIRED_THRESHOLD]
            r["required_list"] = ", ".join(req_list) if req_list else "none"
            rec_rows.append(r)

        udf = pd.DataFrame(rec_rows)


        # Single trace — module color swapped via restyle (8x smaller than per-module traces)
        module_color_arrays = {mod: udf[f"req_{mod}"].tolist() for mod in MODULES}
        sym_map = udf["regime_label"].map(REGIME_SYMBOLS).fillna("circle").tolist()
        sizes   = [7 if m else 3.5 for m in udf["measured"].tolist()]
        borders = ["rgba(30,30,30,0.8)" if m else "rgba(150,150,150,0.15)"
                   for m in udf["measured"].tolist()]

        colorscale = [
            [0.00, "#5b9bd5"],   # steel blue — not needed
            [0.18, "#adb5bd"],   # grey — near threshold
            [0.55, "#f4a261"],   # amber
            [1.00, "#c1121f"],   # crimson — required
        ]

        def _hover_text(row):
            req   = row["required_list"]
            wlbl  = ("calm"     if row["effective_wind"] < 0.20 else
                     "light"    if row["effective_wind"] < 0.30 else
                     "moderate" if row["effective_wind"] < 0.40 else "gusty")
            src   = "measured" if row["measured"] else "predicted"
            mods  = " | ".join(
                f"<b>{MODULE_LABELS[m]}</b>" if row[f"req_{m}"] >= REQUIRED_THRESHOLD
                else MODULE_LABELS[m] for m in MODULES
            )
            return (
                f"<b>{row['regime_label']}</b>  "
                f"{'stable' if row['static_margin'] < 0 else 'unstable'}  [{src}]<br>"
                f"Iyy {row['Iyy']:.4f}  Wind {row['effective_wind']:.3f} ({wlbl})  "
                f"Auth {row['authority_ratio']:.1f}x<br>"
                f"Slew {row['servo_slew_deg_s']:.0f} deg/s  "
                f"Motor {row.get('motor_scale', 0):.2f}  Mass {row['mass']:.3f} kg<br>"
                f"<b>Required: {req}</b><br>"
                f"<span style='font-size:10px;color:#555'>(bold=required) {mods}</span>"
            )

        hover_texts = [_hover_text(row) for _, row in udf.iterrows()]

        trace = go.Scatter3d(
            x=udf["Iyy"].tolist(),
            y=udf["wind_strength"].tolist(),
            z=udf["static_margin"].tolist(),
            mode="markers", name="designs", showlegend=False,
            marker=dict(
                size=sizes, symbol=sym_map,
                color=module_color_arrays["slew"],
                colorscale=colorscale, cmin=0, cmax=1, showscale=True,
                colorbar=dict(
                    title=dict(text="Required<br>probability", font=dict(size=11)),
                    tickvals=[0, 0.20, 0.50, 0.80, 1.0],
                    ticktext=["0%<br>not needed", "20%<br>threshold",
                              "50%<br>likely", "80%<br>probable", "100%<br>required"],
                    thickness=16, len=0.50, x=1.01, xpad=8,
                    bgcolor="rgba(255,255,255,0.9)", outlinewidth=0,
                ),
                line=dict(color=borders, width=0.6),
                opacity=0.90,
            ),
            hovertext=hover_texts, hoverinfo="text",
        )

        # Module dropdown — restyle color only (fast, no page reload)
        mod_btns = [dict(
            label=MODULE_LABELS[mod],
            method="update",
            args=[
                {"marker.color": [module_color_arrays[mod]]},
                {"title.text": f"Fidelity Atlas -- {MODULE_LABELS[mod]} (blue=not needed, red=required)"},
            ],
        ) for mod in MODULES]

        def _axis_btns(axis_key, scene_key):
            return [dict(
                label=label, method="update",
                args=[{axis_key: [udf[col].tolist()]},
                      {f"scene.{scene_key}.title.text": label}],
            ) for col, label in PARAMS if col in udf.columns]

        mark_btns = [dict(label="All (no highlight)", method="restyle",
                          args=[{"marker.symbol": [sym_map]}])]
        for rname, rsym in REGIME_SYMBOLS.items():
            hl = [rsym if r == rname else "circle-open" for r in udf["regime_label"].tolist()]
            mark_btns.append(dict(
                label=f"Highlight {rname}", method="restyle",
                args=[{"marker.symbol": [hl]}],
            ))

        R1, R2, LOFF = 0.975, 0.862, 0.030
        layout = go.Layout(
            paper_bgcolor="rgb(250,250,253)",
            title=dict(
                text="Fidelity Requirement Atlas -- Servo slew (blue=not needed, red=required)",
                font=dict(size=13, color="#222"),
                x=0.5, xanchor="center", y=0.985,
            ),
            scene=dict(
                domain=dict(x=[0.0, 1.0], y=[0.0, 0.80]),
                xaxis=dict(title="Iyy (kg·m²)", tickformat=".3f",
                           backgroundcolor="rgb(245,245,250)", gridcolor="#dde",
                           showbackground=True),
                yaxis=dict(
                    title="Wind strength (rad/s²)",
                    tickvals=[0.05, 0.15, 0.25, 0.35, 0.45],
                    ticktext=["0.05 calm", "0.15 light", "0.25 mod.",
                              "0.35 gusty", "0.45 severe"],
                    backgroundcolor="rgb(245,250,245)", gridcolor="#dde",
                    showbackground=True,
                ),
                zaxis=dict(title="Static margin (calibers)",
                           backgroundcolor="rgb(250,245,245)", gridcolor="#dde",
                           showbackground=True),
                camera=dict(eye=dict(x=1.5, y=-1.5, z=0.9)),
            ),
            updatemenus=[
                # Row 1: Module | X axis | Y axis
                dict(buttons=mod_btns, direction="down", showactive=True,
                     x=0.00, y=R1, xanchor="left", yanchor="top",
                     bgcolor="white", bordercolor="#ccc", borderwidth=1,
                     font=dict(size=11, color="#222"), pad=dict(t=3, b=3, l=6, r=6)),
                dict(buttons=_axis_btns("x", "xaxis"), direction="down", showactive=True,
                     x=0.30, y=R1, xanchor="left", yanchor="top",
                     bgcolor="white", bordercolor="#ccc", borderwidth=1,
                     font=dict(size=10, color="#222"), pad=dict(t=3, b=3, l=6, r=6)),
                dict(buttons=_axis_btns("y", "yaxis"), direction="down", showactive=True,
                     x=0.60, y=R1, xanchor="left", yanchor="top",
                     bgcolor="white", bordercolor="#ccc", borderwidth=1,
                     font=dict(size=10, color="#222"), pad=dict(t=3, b=3, l=6, r=6)),
                # Row 2: Z axis | Mark regime
                dict(buttons=_axis_btns("z", "zaxis"), direction="down", showactive=True,
                     x=0.00, y=R2, xanchor="left", yanchor="top",
                     bgcolor="white", bordercolor="#ccc", borderwidth=1,
                     font=dict(size=10, color="#222"), pad=dict(t=3, b=3, l=6, r=6)),
                dict(buttons=mark_btns, direction="down", showactive=True,
                     x=0.50, y=R2, xanchor="left", yanchor="top",
                     bgcolor="white", bordercolor="#ccc", borderwidth=1,
                     font=dict(size=10, color="#222"), pad=dict(t=3, b=3, l=6, r=6)),
            ],
            annotations=[
                dict(text="<b>Module</b>",       x=0.00, y=R1+LOFF,
                     xref="paper", yref="paper", showarrow=False,
                     font=dict(size=10, color="#444"), xanchor="left"),
                dict(text="<b>X axis</b>",       x=0.30, y=R1+LOFF,
                     xref="paper", yref="paper", showarrow=False,
                     font=dict(size=10, color="#444"), xanchor="left"),
                dict(text="<b>Y axis</b>",       x=0.60, y=R1+LOFF,
                     xref="paper", yref="paper", showarrow=False,
                     font=dict(size=10, color="#444"), xanchor="left"),
                dict(text="<b>Z axis</b>",       x=0.00, y=R2+LOFF,
                     xref="paper", yref="paper", showarrow=False,
                     font=dict(size=10, color="#444"), xanchor="left"),
                dict(text="<b>Mark regime</b>",  x=0.50, y=R2+LOFF,
                     xref="paper", yref="paper", showarrow=False,
                     font=dict(size=10, color="#444"), xanchor="left"),
                dict(
                    text=("large = Exp4C measured  .  small = atlas-predicted"
                          "  |  circle=EASY  sq=MARGINAL  diamond=FRAGILE"
                          "  |  NOTE: Exp4C data from prior design space — re-run pending"),
                    x=0.5, y=0.820, xref="paper", yref="paper",
                    showarrow=False, font=dict(size=9, color="#777"), xanchor="center",
                ),
            ],
            margin=dict(l=10, r=10, b=10, t=30),
            width=1120, height=820,
        )

        fig = go.Figure(data=[trace], layout=layout)
        if out_path:
            fig.write_html(str(out_path), include_plotlyjs="cdn",
                           config={"toImageButtonOptions": {"format": "png", "width": 1200,
                                                            "height": 850}})
            print(f"  Saved interactive 3D atlas: {out_path}")
        if open_browser:
            fig.show()
        return fig

    def plot_fidelity_trellis(
        self,
        figsize: tuple = (14, 12),
    ) -> plt.Figure:
        """
        Trellis of 2D scatter plots across key parameter pairs.

        Six panels cover the most important physical relationships:
          (1) Iyy × effective_wind           — the primary controllability boundary
          (2) slew × effective_wind          — slew vs disturbance
          (3) slew × authority_ratio         — actuator speed vs control margin
          (4) authority_ratio × Iyy          — combined predictor vs inertia
          (5) motor_scale × Iyy              — mass/thrust vs inertia
          (6) slew × Iyy                     — the Exp4C slew finding in context

        Each panel: Exp4C measured (large), Exp1 predicted (small).
        Color: fidelity tier (green→red = more physics needed).
        """
        TIER_COLORS_MPL = {0: "#2ecc71", 1: "#f1c40f", 2: "#e67e22", 3: "#e74c3c"}
        TIER_LABELS = {
            0: "None",
            1: "Slew only",
            2: "Slew+1",
            3: "Slew+2+",
        }

        tier_df  = self._design_fidelity_tiers(self.abl_df)
        tier_df  = self._add_derived(tier_df)
        exp1_ext = self._add_derived(self.exp1_df)

        def _predict_tier(row):
            is_stable = row["static_margin"] < 0.0
            sub = self._freq[
                (self._freq["regime"] == row["regime_label"]) &
                (self._freq["is_stable"] == is_stable)
            ]
            if sub.empty:
                return 1
            n = (sub["decision_error_rate"] >= REQUIRED_THRESHOLD).sum()
            return min(n, 3)
        exp1_ext["fidelity_tier"] = exp1_ext.apply(_predict_tier, axis=1)

        PANELS = [
            ("Iyy",            "effective_wind",    "Iyy (kg·m²)",               "Eff. wind disturbance (rad/s²)"),
            ("servo_slew_deg_s","effective_wind",   "Servo slew (deg/s)",         "Eff. wind disturbance (rad/s²)"),
            ("servo_slew_deg_s","authority_ratio",  "Servo slew (deg/s)",         "Authority ratio (ctrl/wind)"),
            ("authority_ratio", "Iyy",              "Authority ratio (ctrl/wind)","Iyy (kg·m²)"),
            ("motor_scale",    "Iyy",               "Motor scale",                "Iyy (kg·m²)"),
            ("servo_slew_deg_s","Iyy",              "Servo slew (deg/s)",         "Iyy (kg·m²)"),
        ]

        fig, axes = plt.subplots(3, 2, figsize=figsize)
        axes = axes.flatten()

        for ax, (xc, yc, xl, yl) in zip(axes, PANELS):
            # Background: Exp1 predicted (small, low alpha)
            for tier in sorted(TIER_COLORS_MPL):
                sub = exp1_ext[exp1_ext["fidelity_tier"] == tier]
                if sub.empty or xc not in sub.columns or yc not in sub.columns:
                    continue
                ax.scatter(sub[xc], sub[yc], c=TIER_COLORS_MPL[tier],
                           s=4, alpha=0.2, linewidths=0, zorder=1)

            # Foreground: Exp4C measured (large, opaque)
            for tier in sorted(TIER_COLORS_MPL):
                sub = tier_df[tier_df["fidelity_tier"] == tier]
                if sub.empty or xc not in sub.columns or yc not in sub.columns:
                    continue
                ax.scatter(sub[xc], sub[yc], c=TIER_COLORS_MPL[tier],
                           s=30, alpha=0.9, linewidths=0.5,
                           edgecolors="white", zorder=2,
                           label=TIER_LABELS[tier])

            # Wind calibration bands for effective_wind axis
            if "effective_wind" in (xc, yc):
                is_y = (yc == "effective_wind")
                for val, lbl in [(0.20, "calm|light"), (0.35, "light|moderate")]:
                    if is_y:
                        ax.axhline(val, ls="--", lw=0.7, color="#999", alpha=0.5)
                        ax.text(ax.get_xlim()[1] if ax.get_xlim()[1] != 1.0 else 1.0,
                                val, lbl, va="bottom", ha="right", fontsize=6, color="#999")
                    else:
                        ax.axvline(val, ls="--", lw=0.7, color="#999", alpha=0.5)

            ax.set_xlabel(xl, fontsize=8)
            ax.set_ylabel(yl, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.15)

        # Shared legend on first panel
        handles = [
            plt.scatter([], [], c=TIER_COLORS_MPL[t], s=40, label=TIER_LABELS[t])
            for t in sorted(TIER_COLORS_MPL)
        ]
        axes[0].legend(handles=handles, title="Fidelity tier", fontsize=7,
                       title_fontsize=7, loc="upper right")

        fig.suptitle(
            "Fidelity Requirement Atlas — Parameter Space Trellis\n"
            "Large dots = measured (Exp4C n=151)  |  Small = atlas prediction (Exp1 n=1200)\n"
            "Color: modules needed in simulator  (green = simple OK, red = full physics)",
            fontsize=10, y=1.01,
        )
        plt.tight_layout()
        return fig

    def plot_authority_ratio_histogram(self, figsize: tuple = (8, 4)) -> plt.Figure:
        """
        Distribution of authority_ratio (control authority / effective wind)
        split by regime. Shows the single best consolidated predictor.
        """
        exp1_ext = self._add_derived(self.exp1_df)

        fig, ax = plt.subplots(figsize=figsize)
        for regime in REGIME_ORDER:
            sub = exp1_ext[exp1_ext["regime_label"] == regime]
            ax.hist(sub["authority_ratio"], bins=40, alpha=0.55,
                    color=REGIME_COLORS[regime], label=regime,
                    density=True, range=(0, 60))

        ax.axvline(5,  ls="--", lw=1.2, color="#c0392b", alpha=0.8,
                   label="Danger zone (<5)")
        ax.axvline(10, ls="--", lw=1.0, color="#e67e22", alpha=0.7,
                   label="Caution zone (<10)")
        ax.set_xlabel(
            "Authority ratio  =  control authority / effective wind disturbance\n"
            "(higher = control is stronger than wind; <5 = FRAGILE/INFEASIBLE risk)",
            fontsize=9,
        )
        ax.set_ylabel("Density", fontsize=9)
        ax.set_title("Authority ratio separates regimes cleanly\n"
                     "Single consolidated predictor combining Iyy, wind, motor, and aero",
                     fontsize=10)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)
        plt.tight_layout()
        return fig

    def plot_regime_classifier_importance(self, figsize: tuple = (6, 3.5)) -> plt.Figure:
        """Bar chart: which physical parameters predict regime?"""
        imp = pd.Series(self._clf.feature_importances_, index=self._clf_features)
        imp = imp.sort_values()
        fig, ax = plt.subplots(figsize=figsize)
        colors = ["#2196F3" if f in ("Iyy", "wind_strength") else "steelblue"
                  for f in imp.index]
        ax.barh([FEATURE_LABELS.get(f, f) for f in imp.index], imp.values,
                color=colors, edgecolor="white")
        ax.set_xlabel("Importance", fontsize=9)
        ax.set_title("Regime classifier: feature importances\n"
                     "(blue = primary predictors per CLAUDE.md findings)", fontsize=9)
        ax.grid(axis="x", alpha=0.3)
        plt.tight_layout()
        return fig


# ── CLI ───────────────────────────────────────────────────────────────────────

def _generate_all_figures(controller_type: str = "pd", out_dir: Path = None) -> None:
    """Generate and save all atlas figures for a given controller type."""
    out_dir = Path(out_dir) if out_dir else FIGURES_DIR / controller_type
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading Exp4C ({controller_type.upper()}) data and fitting models...")
    atlas = FidelityAtlas.from_csv(controller_type=controller_type)
    print(f"  Fitted models for: {list(atlas.models.keys())}")

    # 1. Feature importance for all modules
    print("Generating: feature importance overview...")
    fig = atlas.plot_feature_importance()
    fig.savefig(out_dir / "feature_importance_all.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    for module in atlas.modules:
        if module not in atlas.models:
            continue
        print(f"Generating atlas for: {module}...")

        # 2. Full panel
        fig = atlas.plot_module_panel(module)
        fig.savefig(out_dir / f"panel_{module}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)

        # 3. 1-D PDP for top 4 features
        fig = atlas.plot_pdp_1d(module)
        fig.savefig(out_dir / f"pdp_1d_{module}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)

        # 4. 2-D excerpt (default pair)
        f1, f2 = DEFAULT_EXCERPT_PAIRS.get(
            module,
            tuple(atlas.feature_importance(module).head(2).index.tolist()),
        )
        try:
            fig = atlas.plot_excerpt_2d(module, f1, f2)
            fig.savefig(out_dir / f"excerpt_2d_{module}.png", dpi=150, bbox_inches="tight")
            plt.close(fig)
        except Exception as e:
            print(f"  [WARN] excerpt_2d failed for {module}: {e}")

        # 5. Parallel coordinates
        fig = atlas.plot_parallel_coordinates(module)
        fig.savefig(out_dir / f"parallel_{module}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)

    print(f"\nAll figures saved to: {out_dir}")

    # ── Requirement atlas figures ─────────────────────────────────────────────
    req_dir = out_dir / "requirement_atlas"
    req_dir.mkdir(parents=True, exist_ok=True)

    print("\nBuilding FidelityRequirementAtlas (two-stage, decision-error)...")
    fra = FidelityRequirementAtlas.from_csv(controller_type=controller_type)
    acc = fra.regime_classifier_accuracy()
    print(f"  Regime classifier 5-fold CV accuracy: {acc:.3f}")

    print("Generating: fidelity heatmap (main atlas figure)...")
    fig = fra.plot_heatmap()
    fig.savefig(req_dir / "atlas_heatmap.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    print("Generating: design space / controllability + slew map...")
    fig = fra.plot_design_space()
    fig.savefig(req_dir / "atlas_design_space.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    print("Generating: regime classifier importance...")
    fig = fra.plot_regime_classifier_importance()
    fig.savefig(req_dir / "regime_classifier_importance.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    print("Generating: authority ratio histogram (consolidated predictor)...")
    fig = fra.plot_authority_ratio_histogram()
    fig.savefig(req_dir / "authority_ratio_histogram.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    print("Generating: fidelity trellis (6-panel 2D)...")
    fig = fra.plot_fidelity_trellis()
    fig.savefig(req_dir / "fidelity_trellis.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    print("Generating: interactive 3D fidelity space (HTML)...")
    fra.plot_3d_fidelity_space(out_path=req_dir / "atlas_3d_interactive.html")

    print(f"  Requirement atlas figures saved to: {req_dir}")
    print(f"\nAll figures saved to: {out_dir}")


if __name__ == "__main__":
    ctype   = sys.argv[1] if len(sys.argv) > 1 else "pd"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else None
    _generate_all_figures(controller_type=ctype, out_dir=out_dir)
