"""
fidelity_dominance.py — Regime-stratified fidelity dominance analysis.

Answers:
  "Which simulator fidelity module matters most — and where in the design
   space does each one's importance peak?"

Key distinction from naive global ranking:
  We use |delta_success| (how much does removing this module change the
  GO/NOGO engineering decision?) rather than |delta_rms| (how much does
  it change the tracking error number?).  A module that shifts RMS by
  10 deg is irrelevant if the design is clearly GO either way.

  Fig 6 (metric_agreement) directly compares both framings side-by-side to
  show that metric choice — not just parameter choice — drives the ranking.

Run:
  cd project_root
  python tools/fidelity_dominance.py

Outputs to paper/figures/:
  fidelity_regime_bars.png        — per-regime dominant module distribution
  fidelity_spatial_scatter.png    — all 1200 designs in (p_unstable × slew) space,
                                    coloured by decision-dominant module
  fidelity_handoff.png            — how dominance shifts along p_unstable and slew axes
  fidelity_multisensitivity.png   — how many modules each design needs
  fidelity_complexity_map.png     — n_modules needed, coloured in 2D space
  fidelity_metric_agreement.png   — RMS-dominant vs GO/NOGO-dominant, agreement map
"""

from __future__ import annotations
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap
import numpy as np
import pandas as pd
from scipy.stats import beta as beta_dist

ROOT    = Path(__file__).resolve().parent.parent
EXP_DIR = ROOT / "experiments" / "results"
FIG_DIR = ROOT / "paper" / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "font.family":       "sans-serif",
    "font.size":         10,
    "axes.titlesize":    10,
    "axes.labelsize":    9,
    "xtick.labelsize":   8,
    "ytick.labelsize":   8,
    "figure.dpi":        150,
    "axes.spines.top":   False,
    "axes.spines.right": False,
})

# ── Constants ─────────────────────────────────────────────────────────────────

MODULES = ["wind", "slew", "sensor_noise", "backlash", "thrust_var", "latency", "deadband"]

# Colour palette for modules — chosen to be distinct and print-safe
MODULE_COLORS = {
    "wind":         "#2980b9",   # blue
    "slew":         "#e67e22",   # orange
    "sensor_noise": "#8e44ad",   # purple
    "backlash":     "#27ae60",   # green
    "thrust_var":   "#c0392b",   # red
    "latency":      "#7f8c8d",   # grey
    "deadband":     "#bdc3c7",   # light grey
}
MODULE_LABELS = {
    "wind":         "Wind / Gusts",
    "slew":         "Slew Rate Limit",
    "sensor_noise": "Sensor Noise",
    "backlash":     "Backlash",
    "thrust_var":   "Thrust Variation",
    "latency":      "Sensor Latency",
    "deadband":     "Deadband",
}
REGIME_COLORS  = {"EASY": "#2ecc71", "FRAGILE": "#f39c12", "INFEASIBLE": "#e74c3c"}
REGIME_ORDER   = ["EASY", "FRAGILE", "INFEASIBLE"]


# ── Data loading and preprocessing ───────────────────────────────────────────

def load_data() -> pd.DataFrame:
    """Load Exp4 ablation data and add derived columns."""
    df = pd.read_csv(EXP_DIR / "exp4_ablation_study_py.csv")

    # Decision-dominant module: the one with the largest |delta_success| for this design.
    # This tells us which module most changes the engineering decision (GO/NOGO).
    delta_cols = {m: f"delta_success_{m}" for m in MODULES}
    def _dominant(row):
        scores = {m: abs(row[col]) for m, col in delta_cols.items() if col in row.index}
        return max(scores, key=scores.get)
    df["decision_dominant"] = df.apply(_dominant, axis=1)

    # RMS-dominant module: the one with the largest |delta_rms| (shifts tracking error most).
    # This is what a naive "which module is important?" study would report.
    rms_cols = {m: f"delta_rms_{m}" for m in MODULES}
    def _rms_dominant(row):
        scores = {m: abs(row[col]) for m, col in rms_cols.items() if col in row.index}
        return max(scores, key=scores.get)
    df["rms_dominant"] = df.apply(_rms_dominant, axis=1)

    # Agreement: do both metrics point to the same dominant module?
    df["metrics_agree"] = df["decision_dominant"] == df["rms_dominant"]

    # Count of modules that flip the GO/NOGO decision for this design.
    # A design with n_flips=0 can be simulated with a basic model.
    # A design with n_flips=5 requires many physics layers to model correctly.
    flip_cols = [f"decision_flip_{m}" for m in MODULES if f"decision_flip_{m}" in df.columns]
    df["n_modules_flip"] = df[flip_cols].sum(axis=1)

    return df


# ── Wilson score confidence interval for a proportion ─────────────────────────

def wilson_ci(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """
    Wilson score 95% confidence interval for a proportion.
    More accurate than normal approximation for small samples or extreme proportions.
    Returns (lower, upper) as fractions.
    """
    if n == 0:
        return (0.0, 1.0)
    p    = successes / n
    denom = 1 + z**2 / n
    centre = (p + z**2 / (2 * n)) / denom
    half   = z / denom * np.sqrt(p * (1 - p) / n + z**2 / (4 * n**2))
    return (max(0.0, centre - half), min(1.0, centre + half))


# ── Table 1: Regime-stratified decision-dominant distribution ─────────────────

def table_regime_dominance(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for regime in REGIME_ORDER:
        sub  = df[df["regime_label"] == regime]
        n    = len(sub)
        for module in MODULES:
            k    = (sub["decision_dominant"] == module).sum()
            frac = k / n
            lo, hi = wilson_ci(k, n)
            rows.append(dict(
                regime=regime, module=module,
                n_designs=n, n_dominant=k,
                pct=round(100 * frac, 1),
                ci_lo=round(100 * lo, 1),
                ci_hi=round(100 * hi, 1),
            ))
    return pd.DataFrame(rows)


# ── Table 2: Contextual dominance (near/far frontier, low/high slew) ──────────

def table_context_dominance(df: pd.DataFrame) -> pd.DataFrame:
    p_med   = df["p_unstable"].median()
    slew_med = df["servo_slew_deg_s"].median()

    contexts = {
        "near_frontier (p > median)":    df["p_unstable"]      >= p_med,
        "far_frontier  (p < median)":    df["p_unstable"]      <  p_med,
        "low_slew  (< median)":          df["servo_slew_deg_s"] <  slew_med,
        "high_slew (>= median)":         df["servo_slew_deg_s"] >= slew_med,
    }
    rows = []
    for label, mask in contexts.items():
        sub = df[mask]
        n   = len(sub)
        vc  = sub["decision_dominant"].value_counts()
        top = vc.head(3)
        for module, k in top.items():
            lo, hi = wilson_ci(k, n)
            rows.append(dict(
                context=label, module=module,
                n_context=n, n_dominant=k,
                pct=round(100 * k/n, 1),
                ci_lo=round(100*lo, 1), ci_hi=round(100*hi, 1),
            ))
    return pd.DataFrame(rows)


# ── Table 3: Multi-module sensitivity ─────────────────────────────────────────

def table_multisensitivity(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for regime in REGIME_ORDER:
        sub = df[df["regime_label"] == regime]
        n   = len(sub)
        for k in range(8):
            cnt  = (sub["n_modules_flip"] == k).sum()
            lo, hi = wilson_ci(cnt, n)
            rows.append(dict(
                regime=regime, n_modules_flip=k,
                n_designs=cnt, pct=round(100*cnt/n, 1),
                ci_lo=round(100*lo,1), ci_hi=round(100*hi,1),
            ))
    return pd.DataFrame(rows[rows.index if False else 0:])  # all rows


# ── Figure 1: Decision-dominant module by regime ──────────────────────────────

def fig_regime_bars(df: pd.DataFrame, tbl: pd.DataFrame):
    """
    Three panels (one per regime) showing how often each fidelity module is
    the decision-dominant module, with 95% Wilson confidence intervals.

    Reading this: if a bar is tall, removing that module is what most often
    changes whether you'd fly the rocket or not — for designs in that regime.
    """
    fig, axes = plt.subplots(1, 3, figsize=(13, 5), constrained_layout=True)

    for ax, regime in zip(axes, REGIME_ORDER):
        sub = tbl[tbl["regime"] == regime].sort_values("pct", ascending=False)
        x   = np.arange(len(sub))
        colors = [MODULE_COLORS.get(m, "#aaa") for m in sub["module"]]
        bars = ax.bar(x, sub["pct"], color=colors, width=0.65, zorder=3, alpha=0.88)

        # Wilson 95% CI error bars
        yerr_lo = sub["pct"].values - sub["ci_lo"].values
        yerr_hi = sub["ci_hi"].values - sub["pct"].values
        ax.errorbar(x, sub["pct"], yerr=[yerr_lo, yerr_hi],
                    fmt="none", color="black", capsize=4, lw=1.2, zorder=4)

        ax.set_xticks(x)
        ax.set_xticklabels([MODULE_LABELS[m] for m in sub["module"]],
                           rotation=38, ha="right", fontsize=8)
        ax.set_ylabel("% of designs where this module\nmost changes GO/NOGO decision",
                      fontsize=8)
        n_regime = sub["n_designs"].iloc[0]
        ax.set_title(f"{regime}  (n={n_regime})", fontweight="bold",
                     color=REGIME_COLORS[regime])
        ax.set_ylim(0, 100)
        ax.grid(axis="y", alpha=0.3, lw=0.6)

        # Annotate top bar
        top_row = sub.iloc[0]
        ax.text(x[0], top_row["pct"] + top_row["ci_hi"] - top_row["pct"] + 2,
                f"{top_row['pct']:.0f}%", ha="center", fontsize=8, fontweight="bold")

    fig.suptitle(
        "Decision-Dominant Fidelity Module by Regime\n"
        "Which simulator module most changes the GO/NOGO decision when removed?  "
        "Error bars = 95% Wilson CI.",
        fontsize=9.5,
    )
    out = FIG_DIR / "fidelity_regime_bars.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 2: Spatial scatter — design space coloured by dominant module ──────

def fig_spatial_scatter(df: pd.DataFrame):
    """
    Every rocket as a dot in (p_unstable × servo_slew) space.
    Colour = which fidelity module most changes the decision for that rocket.

    This shows WHERE in the design space each module dominates —
    you can see the handoff between wind domination (high p_unstable)
    and slew/backlash domination (lower instability rates).
    """
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5),
                              constrained_layout=True, sharey=True, sharex=True)

    for ax, regime in zip(axes, REGIME_ORDER):
        sub = df[df["regime_label"] == regime]
        for module in MODULES:
            mask = sub["decision_dominant"] == module
            if mask.sum() == 0:
                continue
            ax.scatter(
                sub.loc[mask, "p_unstable"],
                sub.loc[mask, "servo_slew_deg_s"],
                c=MODULE_COLORS[module], s=18, alpha=0.70,
                label=MODULE_LABELS[module], zorder=3, linewidths=0,
            )
        ax.set_xlabel("p_unstable  (instability rate, 1/s)")
        ax.set_title(f"{regime}", fontweight="bold", color=REGIME_COLORS[regime])

    axes[0].set_ylabel("Servo slew rate  (deg/s)")

    # Shared legend on the right
    handles = [mpatches.Patch(color=MODULE_COLORS[m], label=MODULE_LABELS[m])
               for m in MODULES]
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=8,
               title="Decision-dominant fidelity module", title_fontsize=8,
               bbox_to_anchor=(0.5, -0.08), framealpha=0.9)

    fig.suptitle(
        "Where Each Fidelity Module Dominates — All 1200 Designs\n"
        "Each dot = one rocket design.  Colour = which simulator module "
        "most changes the engineering decision when removed.",
        fontsize=9.5,
    )
    out = FIG_DIR / "fidelity_spatial_scatter.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 3: Dominance handoff along p_unstable and slew axes ───────────────

def fig_handoff(df: pd.DataFrame):
    """
    Two panels showing how the fraction of designs where each module dominates
    changes as we move along the two most important axes.

    This answers: "At what instability rate does wind take over from slew?"
    Think of it like a stacked area chart where the layers are the modules.
    """
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), constrained_layout=True)

    for ax, axis_col, axis_label, n_bins in [
        (axes[0], "p_unstable",       "Instability rate  p_unstable (1/s)",   10),
        (axes[1], "servo_slew_deg_s", "Servo slew rate  (deg/s)",             10),
    ]:
        bins  = pd.qcut(df[axis_col], n_bins, duplicates="drop")
        fracs = (df.groupby(bins)["decision_dominant"]
                   .value_counts(normalize=True)
                   .unstack(fill_value=0))

        # Only keep the 4 most common modules for readability
        top4 = df["decision_dominant"].value_counts().head(4).index.tolist()
        fracs = fracs[[m for m in top4 if m in fracs.columns]]

        # X positions = midpoint of each bin
        bin_mids = [iv.mid for iv in fracs.index]

        bottom = np.zeros(len(fracs))
        for module in top4:
            if module not in fracs.columns:
                continue
            vals = fracs[module].values * 100
            ax.fill_between(bin_mids, bottom, bottom + vals,
                            color=MODULE_COLORS[module], alpha=0.82,
                            label=MODULE_LABELS[module], step="mid")
            bottom += vals

        ax.set_xlabel(axis_label)
        ax.set_ylabel("% of designs in bin")
        ax.set_ylim(0, 100)
        ax.set_xlim(min(bin_mids), max(bin_mids))
        ax.grid(axis="y", alpha=0.3, lw=0.6)

        # Add regime-boundary reference line
        if axis_col == "p_unstable":
            ax.axvline(1.5, color="black", lw=1.2, ls="--", alpha=0.6,
                       label="Approx. regime boundary")
            ax.text(1.52, 95, "← stability boundary", fontsize=7.5, va="top")

    # One shared legend
    handles = [mpatches.Patch(color=MODULE_COLORS[m], label=MODULE_LABELS[m])
               for m in top4]
    handles.append(plt.Line2D([0],[0], color="black", ls="--", lw=1.2, label="Stability boundary"))
    fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=8,
               bbox_to_anchor=(0.5, -0.08), framealpha=0.9)

    fig.suptitle(
        "Fidelity Dominance Handoff Along Key Design Axes\n"
        "Each horizontal slice shows what fraction of rockets in that bin "
        "have each module as their most decision-critical fidelity term.",
        fontsize=9.5,
    )
    out = FIG_DIR / "fidelity_handoff.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 4: Multi-module sensitivity heat map ───────────────────────────────

def fig_multisensitivity(df: pd.DataFrame):
    """
    Two panels:
    Left:  Stacked bar showing how many designs in each regime need 0, 1, 2, 3+ modules.
    Right: Map in (p_unstable × slew) showing n_modules_flip — how many modules you need.

    Interpretation: designs in regions where n_modules_flip is high are 'fidelity-critical' —
    any single simplification you make to your simulator can produce wrong answers.
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)

    # ── Left: stacked bar by regime ───────────────────────────────────────
    ax = axes[0]
    colors_n = ["#2ecc71", "#3498db", "#f39c12", "#e74c3c", "#8e44ad", "#7f8c8d", "#2c3e50", "#1abc9c"]
    max_n    = 7
    bottoms  = np.zeros(len(REGIME_ORDER))

    for k in range(max_n + 1):
        vals = []
        for regime in REGIME_ORDER:
            sub  = df[df["regime_label"] == regime]
            frac = (sub["n_modules_flip"] == k).mean() * 100
            vals.append(frac)
        label = f"{k} module{'s' if k != 1 else ''}" + (" (simple OK)" if k == 0 else "")
        ax.bar(REGIME_ORDER, vals, bottom=bottoms,
               color=colors_n[k], label=label, width=0.55, alpha=0.9)
        bottoms += np.array(vals)

    ax.set_ylabel("% of designs")
    ax.set_title("How Many Fidelity Modules Does Each Design Need?", fontweight="bold")
    ax.set_ylim(0, 105)
    ax.legend(fontsize=7.5, loc="upper right", ncol=1)
    ax.grid(axis="y", alpha=0.3)

    # Annotate "simple OK" fraction for each regime
    for i, regime in enumerate(REGIME_ORDER):
        sub    = df[df["regime_label"] == regime]
        simple = (sub["n_modules_flip"] == 0).mean() * 100
        ax.text(i, simple / 2, f"{simple:.0f}%\nsimple\nOK",
                ha="center", va="center", fontsize=8, fontweight="bold", color="white")

    # ── Right: 2D scatter coloured by n_modules_flip ──────────────────────
    ax = axes[1]
    cmap   = plt.get_cmap("RdYlGn_r", 8)
    sc     = ax.scatter(
        df["p_unstable"], df["servo_slew_deg_s"],
        c=df["n_modules_flip"], cmap=cmap, vmin=0, vmax=7,
        s=16, alpha=0.7, linewidths=0, zorder=3,
    )
    cbar = plt.colorbar(sc, ax=ax, ticks=range(8))
    cbar.set_label("# modules that flip decision")
    cbar.ax.set_yticklabels([f"{k}" for k in range(7)] + ["7"])

    ax.set_xlabel("p_unstable  (instability rate, 1/s)")
    ax.set_ylabel("Servo slew rate  (deg/s)")
    ax.set_title("Fidelity Complexity in Design Space", fontweight="bold")
    ax.text(0.97, 0.97,
            "Green = simple sim OK\nRed = many layers needed",
            transform=ax.transAxes, ha="right", va="top", fontsize=7.5,
            bbox=dict(fc="white", alpha=0.8, pad=2, boxstyle="round"))

    fig.suptitle(
        "Fidelity Complexity: How Many Modules Must Be Modelled?\n"
        "0 = simple simulator is sufficient.  Higher = more physics layers required.",
        fontsize=9.5,
    )
    out = FIG_DIR / "fidelity_multisensitivity.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 5: Wind vs Sensor-noise dominance transition ───────────────────────

def fig_wind_vs_sn(df: pd.DataFrame):
    """
    Wind and sensor_noise are the two most common decision-dominant modules.
    This figure shows exactly WHERE each one dominates, and where they trade off.

    Specifically: a design where wind dominates but not sensor_noise means
    'you need accurate wind modelling but sensor noise doesn't matter much.'
    """
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.8), constrained_layout=True,
                              sharex=True, sharey=True)

    # Compute for each cell: fraction wind-dominant, fraction sn-dominant
    def _plot_dominance_fraction(ax, module, title, color_hi, color_lo="#eeeeee"):
        frac = (df["decision_dominant"] == module).astype(float)
        sc   = ax.scatter(
            df["p_unstable"], df["servo_slew_deg_s"],
            c=frac, cmap=_make_cmap(color_lo, color_hi),
            vmin=0, vmax=1, s=18, alpha=0.85, linewidths=0,
        )
        plt.colorbar(sc, ax=ax, label=f"1 = {MODULE_LABELS[module]} dominant")
        ax.set_xlabel("p_unstable  (1/s)")
        ax.set_title(title, fontweight="bold", color=color_hi)

        # Annotate overall fraction
        overall = frac.mean()
        ax.text(0.04, 0.97, f"Global: {overall:.0%} of designs",
                transform=ax.transAxes, fontsize=8, va="top",
                bbox=dict(fc="white", alpha=0.8, pad=2, boxstyle="round"))

    _plot_dominance_fraction(axes[0], "wind",
                             "Wind dominates\n(1 = wind is #1 for this design)",
                             MODULE_COLORS["wind"])
    _plot_dominance_fraction(axes[1], "sensor_noise",
                             "Sensor noise dominates\n(1 = sensor noise is #1)",
                             MODULE_COLORS["sensor_noise"])
    _plot_dominance_fraction(axes[2], "slew",
                             "Slew rate dominates\n(1 = slew is #1)",
                             MODULE_COLORS["slew"])

    axes[0].set_ylabel("Servo slew rate  (deg/s)")

    fig.suptitle(
        "Spatial Dominance of Top Three Fidelity Modules\n"
        "Dark colour = this module is the primary decision driver for that rocket design.",
        fontsize=9.5,
    )
    out = FIG_DIR / "fidelity_module_dominance_maps.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


def _make_cmap(lo_color: str, hi_color: str):
    """Create a simple two-colour LinearSegmentedColormap."""
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list("", [lo_color, hi_color])


# ── Table 4: Metric agreement by regime ──────────────────────────────────────

def table_metric_agreement(df: pd.DataFrame) -> pd.DataFrame:
    """
    For each regime: how often do RMS-dominant and decision-dominant agree?
    Also shows top module under each metric to reveal the disagreement story.
    """
    rows = []
    for regime in REGIME_ORDER:
        sub = df[df["regime_label"] == regime]
        n   = len(sub)
        agree_k = sub["metrics_agree"].sum()
        lo, hi  = wilson_ci(agree_k, n)

        top_decision = sub["decision_dominant"].value_counts().index[0]
        top_rms      = sub["rms_dominant"].value_counts().index[0]

        rows.append(dict(
            regime=regime, n=n,
            agree_pct=round(100 * agree_k / n, 1),
            ci_lo=round(100 * lo, 1),
            ci_hi=round(100 * hi, 1),
            top_decision=top_decision,
            top_rms=top_rms,
        ))
    return pd.DataFrame(rows)


# ── Figure 6: RMS-dominant vs Decision-dominant — side-by-side + agreement ───

def fig_metric_agreement(df: pd.DataFrame, tbl_agree: pd.DataFrame):
    """
    Three panels:
      Left:  Grouped bars — top module by RMS metric vs top module by GO/NOGO
             decision for each regime.  Shows the ranking reversal.
      Middle: Spatial scatter in (p_unstable × slew) coloured by whether the
              two metrics agree on the dominant module.
      Right:  Stacked bars: for each module, what fraction is RMS-dominant
              vs decision-dominant.  Shows sensor_noise/wind swap.

    Core message: The choice of metric completely changes which simulator
    module you'd prioritise.  Sensor noise looks critical by RMS;
    wind looks critical by engineering decision.
    """
    fig = plt.figure(figsize=(16, 5.5), constrained_layout=True)
    gs  = fig.add_gridspec(1, 3, width_ratios=[1.6, 1.2, 1.6])
    ax_bars, ax_map, ax_swap = (fig.add_subplot(gs[i]) for i in range(3))

    # ── Left: grouped bars per regime, RMS vs Decision ranking ───────────────
    x      = np.arange(len(REGIME_ORDER))
    width  = 0.35
    metric_pairs = [("rms_dominant", "RMS-dominant", "#c0392b"),
                    ("decision_dominant", "Decision-dominant\n(GO/NOGO)", "#2980b9")]

    for i, (col, label, color) in enumerate(metric_pairs):
        # Show the fraction of each regime where the TOP-3 modules appear
        heights = []
        for regime in REGIME_ORDER:
            sub = df[df["regime_label"] == regime]
            top_module = sub[col].value_counts().index[0]
            heights.append(100 * (sub[col] == top_module).mean())

        bars = ax_bars.bar(x + (i - 0.5) * width, heights,
                           width=width, color=color, alpha=0.85, label=label, zorder=3)

    # Label the top module for each column
    for i, (col, _, color) in enumerate(metric_pairs):
        for j, regime in enumerate(REGIME_ORDER):
            sub = df[df["regime_label"] == regime]
            top_module = sub[col].value_counts().index[0]
            pct        = 100 * (sub[col] == top_module).mean()
            ax_bars.text(j + (i - 0.5) * width, pct + 1.2,
                         MODULE_LABELS[top_module].replace(" / ", "\n"),
                         ha="center", va="bottom", fontsize=6.5,
                         fontweight="bold", color=color)

    ax_bars.set_xticks(x)
    ax_bars.set_xticklabels([f"{r}\n(n={tbl_agree.loc[tbl_agree.regime==r,'n'].iloc[0]})"
                              for r in REGIME_ORDER])
    ax_bars.set_ylabel("% of regime where #1 module dominates")
    ax_bars.set_ylim(0, 105)
    ax_bars.set_title("Top Module by Metric\nRed=RMS rank, Blue=Decision rank", fontweight="bold")
    ax_bars.legend(fontsize=7.5, loc="upper right")
    ax_bars.grid(axis="y", alpha=0.3)

    # ── Middle: spatial agreement map ─────────────────────────────────────────
    agree_colors = {True: "#27ae60", False: "#e74c3c"}
    for agree_val, color in agree_colors.items():
        mask = df["metrics_agree"] == agree_val
        lbl  = "Metrics agree" if agree_val else "Metrics disagree"
        ax_map.scatter(
            df.loc[mask, "p_unstable"], df.loc[mask, "servo_slew_deg_s"],
            c=color, s=16, alpha=0.65, linewidths=0, label=lbl, zorder=3,
        )
    agree_overall = df["metrics_agree"].mean()
    ax_map.set_xlabel("p_unstable  (instability rate, 1/s)")
    ax_map.set_ylabel("Servo slew rate  (deg/s)")
    ax_map.set_title("Where Do Both Metrics Agree?\n"
                     "Green = same module either way; Red = depends on metric", fontweight="bold")
    ax_map.legend(fontsize=8, loc="upper right")
    ax_map.text(0.04, 0.04, f"Overall agreement: {agree_overall:.0%}",
                transform=ax_map.transAxes, fontsize=8,
                bbox=dict(fc="white", alpha=0.85, pad=2, boxstyle="round"))
    ax_map.grid(alpha=0.2)

    # ── Right: per-module stacked bars RMS vs Decision prevalence ─────────────
    # Only show the top 5 modules for readability
    top5 = df["decision_dominant"].value_counts().head(5).index.tolist()
    y    = np.arange(len(top5))
    rms_fracs      = [100 * (df["rms_dominant"]      == m).mean() for m in top5]
    decision_fracs = [100 * (df["decision_dominant"] == m).mean() for m in top5]

    ax_swap.barh(y - 0.18, rms_fracs, height=0.32,
                 color="#c0392b", alpha=0.85, label="RMS-dominant")
    ax_swap.barh(y + 0.18, decision_fracs, height=0.32,
                 color="#2980b9", alpha=0.85, label="Decision-dominant")

    for j, (m, rms, dec) in enumerate(zip(top5, rms_fracs, decision_fracs)):
        ax_swap.text(rms + 0.5, j - 0.18, f"{rms:.0f}%", va="center", fontsize=7.5,
                     color="#c0392b", fontweight="bold")
        ax_swap.text(dec + 0.5, j + 0.18, f"{dec:.0f}%", va="center", fontsize=7.5,
                     color="#2980b9", fontweight="bold")

    ax_swap.set_yticks(y)
    ax_swap.set_yticklabels([MODULE_LABELS[m] for m in top5])
    ax_swap.set_xlabel("% of all 1200 designs")
    ax_swap.set_title("Global Ranking: Which Metric You Use\nChanges Which Module Tops the List",
                      fontweight="bold")
    ax_swap.legend(fontsize=8, loc="lower right")
    ax_swap.grid(axis="x", alpha=0.3)

    fig.suptitle(
        "RMS Error vs GO/NOGO Decision: Two Ways to Measure Fidelity Importance\n"
        "The metric choice changes the ranking — sensor noise dominates RMS, "
        "wind dominates engineering decisions.",
        fontsize=9.5,
    )
    out = FIG_DIR / "fidelity_metric_agreement.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Print tables ──────────────────────────────────────────────────────────────

def print_tables(tbl_regime, tbl_context, tbl_multi, tbl_agree):
    print("\n" + "="*65)
    print("TABLE 1: Decision-dominant fidelity module by regime")
    print("         (which module most changes GO/NOGO when removed)")
    print("="*65)
    for regime in REGIME_ORDER:
        sub = tbl_regime[tbl_regime["regime"] == regime].sort_values("pct", ascending=False)
        n   = sub["n_designs"].iloc[0]
        print(f"\n  {regime}  (n={n})")
        print(f"  {'Module':<18s} {'%':>6s}   {'95% CI':>14s}")
        for _, row in sub.head(5).iterrows():
            print(f"  {MODULE_LABELS[row['module']]:<18s} "
                  f"{row['pct']:>5.1f}%   "
                  f"[{row['ci_lo']:.1f}%, {row['ci_hi']:.1f}%]")

    print("\n" + "="*65)
    print("TABLE 2: Decision-dominant module by spatial context")
    print("="*65)
    for ctx in tbl_context["context"].unique():
        sub = tbl_context[tbl_context["context"] == ctx]
        n   = sub["n_context"].iloc[0]
        print(f"\n  {ctx}  (n={n})")
        for _, row in sub.iterrows():
            print(f"  {MODULE_LABELS[row['module']]:<18s} "
                  f"{row['pct']:>5.1f}%  [{row['ci_lo']:.1f}, {row['ci_hi']:.1f}]")

    print("\n" + "="*65)
    print("TABLE 3: Fidelity complexity (n modules needed) by regime")
    print("="*65)
    for regime in REGIME_ORDER:
        sub = tbl_multi[tbl_multi["regime"] == regime]
        n   = sub[sub["n_modules_flip"]==0]["n_designs"].sum() + \
              sub[sub["n_modules_flip"]>0]["n_designs"].sum()
        print(f"\n  {regime}  (n={n})")
        for _, row in sub.iterrows():
            k    = int(row["n_modules_flip"])
            note = " <- simple sim OK" if k == 0 else (" <- needs 1 module" if k == 1 else "")
            print(f"  {k} module(s): {row['pct']:>5.1f}%{note}")

    print("\n" + "="*65)
    print("TABLE 4: Metric agreement — RMS-dominant vs Decision-dominant")
    print("         Does the metric choice change which module tops the list?")
    print("="*65)
    print(f"\n  {'Regime':<12s} {'Agree %':>9s}  {'95% CI':>14s}  "
          f"{'Top (Decision)':>18s}  {'Top (RMS)':>18s}")
    for _, row in tbl_agree.iterrows():
        agree_note = "  <- same!" if row["top_decision"] == row["top_rms"] else "  <- DIFFERENT"
        print(f"  {row['regime']:<12s} {row['agree_pct']:>8.1f}%  "
              f"[{row['ci_lo']:.1f}%, {row['ci_hi']:.1f}%]  "
              f"{MODULE_LABELS[row['top_decision']]:>18s}  "
              f"{MODULE_LABELS[row['top_rms']]:>18s}"
              f"{agree_note}")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Loading data...")
    df = load_data()
    print(f"  {len(df)} designs loaded")

    print("\nComputing tables...")
    tbl_regime  = table_regime_dominance(df)
    tbl_context = table_context_dominance(df)
    tbl_multi   = table_multisensitivity(df)
    tbl_agree   = table_metric_agreement(df)

    print_tables(tbl_regime, tbl_context, tbl_multi, tbl_agree)

    print("\nGenerating figures...")

    print("\n[1/6] Regime-stratified dominance bars")
    fig_regime_bars(df, tbl_regime)

    print("[2/6] Spatial scatter")
    fig_spatial_scatter(df)

    print("[3/6] Dominance handoff along axes")
    fig_handoff(df)

    print("[4/6] Multi-module sensitivity")
    fig_multisensitivity(df)

    print("[5/6] Wind vs sensor-noise vs slew dominance maps")
    fig_wind_vs_sn(df)

    print("[6/6] Metric agreement: RMS vs Decision dominant")
    fig_metric_agreement(df, tbl_agree)

    print(f"\nAll outputs in {FIG_DIR}")
