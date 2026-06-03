"""
exp4a_decision_audit.py — Paired decision-disagreement analysis for Exp4.

Answers the core thesis question:
  "Where does a simplified simulator make WRONG engineering decisions
   compared to a high-fidelity simulator?"

This is distinct from Exp4 ablation, which asks "which module shifts
metrics the most?"  The Exp4A question is about DECISION CORRECTNESS:
  - High-fidelity says GO, simple model says NO-GO  (false pessimism)
  - High-fidelity says NO-GO, simple model says GO  (false optimism — dangerous)

Key concept — the baseline mismatch:
  Exp4 full-fidelity includes a thrust_var fault (keff drops 15% at t=1.5s)
  that was NOT present when Exp1 regime labels were assigned.  This means
  even the "correct answer" (Exp4 full fidelity) may differ from Exp1.
  This audit quantifies that mismatch first, then measures disagreement
  relative to the Exp1 labels as ground truth.

Run:
  cd project_root
  python tools/exp4a_decision_audit.py

Outputs to paper/figures/:
  exp4a_baseline_mismatch.png   — Exp4-full vs Exp1 agreement map
  exp4a_confusion_by_regime.png — confusion matrix for simple vs full model
  exp4a_module_contribution.png — which module, if added back, most often
                                   corrects a wrong simple-model decision

Outputs to experiments/results/:
  exp4a_decision_audit.csv      — per-design decision comparison table
"""

from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
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

MODULES = ["wind", "slew", "sensor_noise", "backlash", "thrust_var", "latency", "deadband"]
MODULE_LABELS = {
    "wind":         "Wind",
    "slew":         "Slew Rate",
    "sensor_noise": "Sensor Noise",
    "backlash":     "Backlash",
    "thrust_var":   "Thrust Variation",
    "latency":      "Latency",
    "deadband":     "Deadband",
}
REGIME_ORDER  = ["EASY", "FRAGILE", "INFEASIBLE"]
REGIME_COLORS = {"EASY": "#2ecc71", "FRAGILE": "#f39c12", "INFEASIBLE": "#e74c3c"}

# GO/NOGO threshold: same as used in Exp4 ablation (FRAGILE_SUCCESS_RATE)
GO_THRESHOLD = 0.50


# ── Data loading ──────────────────────────────────────────────────────────────

def load_data() -> pd.DataFrame:
    """
    Load Exp4 ablation data and add decision-comparison columns.

    Derived columns added:
      exp4_full_go     : GO/NOGO under Exp4 full fidelity (may include thrust_var fault)
      exp1_go          : GO/NOGO from Exp1 evaluation (regime_label-inferred)
      baseline_agrees  : Exp4 full GO == Exp1 GO (checks baseline mismatch magnitude)
      simple_go        : GO/NOGO estimate with ALL fidelity modules removed
                         (inferred: success would improve if removing all modules helps)
      simple_wrong     : simple model would give different answer than Exp1 ground truth
      false_optimism   : simple says GO but Exp1 says INFEASIBLE/FRAGILE failure
      false_pessimism  : simple says NO-GO but Exp1 says GO
    """
    df = pd.read_csv(EXP_DIR / "exp4_ablation_study_py.csv")

    # Exp4 full-fidelity GO/NOGO
    df["exp4_full_go"] = df["full_success_rate"] >= GO_THRESHOLD

    # Exp1 ground truth GO/NOGO: INFEASIBLE designs are NO-GO, others are GO
    # (more nuanced: FRAGILE designs may be borderline — use the full_success_rate
    #  from Exp1 for better ground truth if available, else infer from regime)
    df["exp1_go"] = df["regime_label"].isin(["EASY", "FRAGILE"])

    # Baseline mismatch: does Exp4 full agree with Exp1?
    df["baseline_agrees"] = df["exp4_full_go"] == df["exp1_go"]

    # Simple model estimate: if we remove ALL modules simultaneously.
    # We don't have a direct single-run for the all-modules-off case, but we
    # can estimate it: if every module individually flips GO to NO-GO, then
    # the simple model (no modules) likely GO when the full model is GO, and
    # vice versa when all modules together hurt performance.
    #
    # Best available proxy: success rate without the DOMINANT module.
    # A design that fails only because of wind: simple model would see GO.
    # A design that needs ALL modules to fail: simple would see GO.
    #
    # Conservative estimate: simple_go = True if n_decision_flips == 0
    # (no single module changes the answer, so the simple model agrees).
    # This is an underestimate of simple-model errors.
    df["n_decision_flips"] = df[[f"decision_flip_{m}" for m in MODULES]].sum(axis=1)
    df["simple_agrees_full"] = df["n_decision_flips"] == 0

    # For the counterfactual "simple model" GO/NOGO:
    # If the full model says GO and removing any module keeps it GO (0 flips),
    # then the simple model also says GO → same answer.
    # If the full model says GO but removing modules produces NO-GO flips,
    # we don't know the combined effect (non-additive), but as an approximation:
    # simple model is likely GO (it misses the degradation).
    #
    # Practical: we treat "simple model says GO for any design where exp4_full is GO
    # OR where removing all modules improves success" — this requires a direct run.
    # Mark which designs are candidates for paired disagreement analysis.
    df["needs_paired_run"] = ~df["simple_agrees_full"]

    # False optimism: simple model would say GO when it shouldn't (dangerous)
    # Approximation: designs where exp1_go=False but all individual modules improve
    # performance (removing any single module makes it closer to GO)
    # This means the accumulated effects of all modules together cause failure.
    df["candidate_false_optimism"] = (
        (~df["exp1_go"]) & (df["n_decision_flips"] > 0)
    )

    # False pessimism: simple model would say NO-GO when it should say GO
    # Designs where exp1_go=True but exp4_full_go=False (baseline mismatch)
    df["candidate_false_pessimism"] = (
        df["exp1_go"] & (~df["exp4_full_go"])
    )

    return df


def wilson_ci(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p     = k / n
    denom = 1 + z**2 / n
    c     = (p + z**2 / (2*n)) / denom
    h     = z / denom * np.sqrt(p*(1-p)/n + z**2/(4*n**2))
    return (max(0.0, c - h), min(1.0, c + h))


# ── Table: baseline mismatch by regime ───────────────────────────────────────

def table_baseline_mismatch(df: pd.DataFrame) -> pd.DataFrame:
    """
    Quantifies how much Exp4 full-fidelity GO/NOGO agrees with Exp1 labels.
    This is the "baseline mismatch" caused by the thrust_var fault in Exp4.
    """
    rows = []
    for regime in REGIME_ORDER:
        sub = df[df["regime_label"] == regime]
        n   = len(sub)
        agrees = sub["baseline_agrees"].sum()
        lo, hi = wilson_ci(agrees, n)
        rows.append(dict(
            regime=regime, n=n,
            n_agree=int(agrees),
            agree_pct=round(100 * agrees / n, 1),
            ci_lo=round(100*lo, 1), ci_hi=round(100*hi, 1),
            n_exp4_go=int(sub["exp4_full_go"].sum()),
            n_exp1_go=int(sub["exp1_go"].sum()),
        ))
    return pd.DataFrame(rows)


def table_fidelity_complexity_by_regime(df: pd.DataFrame) -> pd.DataFrame:
    """For each regime: distribution of n_decision_flips (0=simple OK, high=needs many modules)."""
    rows = []
    for regime in REGIME_ORDER:
        sub = df[df["regime_label"] == regime]
        n   = len(sub)
        for k in range(8):
            cnt = (sub["n_decision_flips"] == k).sum()
            lo, hi = wilson_ci(cnt, n)
            rows.append(dict(regime=regime, n_modules=k, n_designs=int(cnt),
                              pct=round(100*cnt/n, 1),
                              ci_lo=round(100*lo,1), ci_hi=round(100*hi,1)))
    return pd.DataFrame(rows)


# ── Figure 1: Baseline mismatch spatial map ───────────────────────────────────

def fig_baseline_mismatch(df: pd.DataFrame, tbl: pd.DataFrame):
    """
    Show WHERE in (p_unstable × slew) space the Exp4 full-fidelity GO/NOGO
    disagrees with Exp1 GO/NOGO.  This is the thrust_var fault contamination map.

    If a judge asks "but does Exp4 even measure what Exp1 measured?",
    this figure answers that question directly.
    """
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)

    # Left: spatial map of agreement
    ax = axes[0]
    colors = {True: "#27ae60", False: "#e74c3c"}
    labels = {True: "Agrees with Exp1", False: "Disagrees (baseline mismatch)"}
    for agree, color in colors.items():
        mask = df["baseline_agrees"] == agree
        ax.scatter(df.loc[mask, "p_unstable"], df.loc[mask, "servo_slew_deg_s"],
                   c=color, s=15, alpha=0.65, linewidths=0, label=labels[agree], zorder=3)
    overall_agree = df["baseline_agrees"].mean()
    ax.set_xlabel("p_unstable  (instability rate, 1/s)")
    ax.set_ylabel("Servo slew rate  (deg/s)")
    ax.set_title("Exp4 Full-Fidelity vs Exp1 GO/NOGO Agreement\n"
                 "(Disagreement = thrust_var fault changes the verdict)", fontweight="bold")
    ax.legend(fontsize=8)
    ax.text(0.04, 0.04, f"Overall: {overall_agree:.1%} agree",
            transform=ax.transAxes, fontsize=9,
            bbox=dict(fc="white", alpha=0.85, pad=2, boxstyle="round"))
    ax.grid(alpha=0.2)

    # Right: per-regime bar chart of agreement rate
    ax = axes[1]
    x  = np.arange(len(REGIME_ORDER))
    agree_pcts  = [tbl.loc[tbl.regime == r, "agree_pct"].iloc[0]  for r in REGIME_ORDER]
    ci_lo_errs  = [tbl.loc[tbl.regime == r, "agree_pct"].iloc[0]  - tbl.loc[tbl.regime == r, "ci_lo"].iloc[0] for r in REGIME_ORDER]
    ci_hi_errs  = [tbl.loc[tbl.regime == r, "ci_hi"].iloc[0]  - tbl.loc[tbl.regime == r, "agree_pct"].iloc[0] for r in REGIME_ORDER]

    bars = ax.bar(x, agree_pcts, color=[REGIME_COLORS[r] for r in REGIME_ORDER],
                  width=0.55, alpha=0.85, zorder=3)
    ax.errorbar(x, agree_pcts, yerr=[ci_lo_errs, ci_hi_errs],
                fmt="none", color="black", capsize=5, lw=1.2, zorder=4)
    for i, (pct, r) in enumerate(zip(agree_pcts, REGIME_ORDER)):
        n = tbl.loc[tbl.regime == r, "n"].iloc[0]
        ax.text(i, pct + 2, f"{pct:.0f}%\n(n={n})", ha="center", fontsize=8.5,
                fontweight="bold")

    ax.axhline(100, color="black", lw=0.8, ls="--", alpha=0.4)
    ax.set_xticks(x)
    ax.set_xticklabels(REGIME_ORDER)
    ax.set_ylabel("% of designs where Exp4-full\nagrees with Exp1 GO/NOGO")
    ax.set_ylim(0, 115)
    ax.set_title("Baseline Agreement by Regime\n"
                 "100% = no contamination from thrust_var fault", fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    fig.suptitle(
        "Exp4 Baseline Mismatch Audit\n"
        "How often does Exp4 full-fidelity agree with the Exp1 ground truth?  "
        "Disagreement = thrust_var fault causes different GO/NOGO.",
        fontsize=9.5,
    )
    out = FIG_DIR / "exp4a_baseline_mismatch.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 2: Module-level decision-flip summary by regime ────────────────────

def fig_decision_flips(df: pd.DataFrame):
    """
    For each regime, show what fraction of designs have each module flipping
    the GO/NOGO decision.  A module that flips decisions frequently in a regime
    is necessary fidelity for designs in that regime.

    This is the "which modules are decision-critical" plot, stratified by regime.
    The x-axis is each module.  The y-axis is % of designs in that regime where
    removing that module changes the GO/NOGO call.
    """
    fig, axes = plt.subplots(1, 3, figsize=(13, 5), constrained_layout=True)

    MODULE_COLORS = {
        "wind": "#2980b9", "slew": "#e67e22", "sensor_noise": "#8e44ad",
        "backlash": "#27ae60", "thrust_var": "#c0392b", "latency": "#7f8c8d",
        "deadband": "#bdc3c7",
    }

    for ax, regime in zip(axes, REGIME_ORDER):
        sub = df[df["regime_label"] == regime]
        n   = len(sub)
        flip_pcts = []
        ci_los, ci_his = [], []
        for module in MODULES:
            col = f"decision_flip_{module}"
            if col not in sub.columns:
                flip_pcts.append(0); ci_los.append(0); ci_his.append(0); continue
            k       = sub[col].sum()
            pct     = 100 * k / n
            lo, hi  = wilson_ci(k, n)
            flip_pcts.append(pct)
            ci_los.append(pct - 100*lo)
            ci_his.append(100*hi - pct)

        x      = np.arange(len(MODULES))
        colors = [MODULE_COLORS[m] for m in MODULES]
        ax.bar(x, flip_pcts, color=colors, width=0.65, alpha=0.85, zorder=3)
        ax.errorbar(x, flip_pcts, yerr=[ci_los, ci_his],
                    fmt="none", color="black", capsize=4, lw=1.2, zorder=4)

        ax.set_xticks(x)
        ax.set_xticklabels([MODULE_LABELS[m] for m in MODULES],
                           rotation=38, ha="right", fontsize=8)
        ax.set_ylabel("% of designs where removing\nthis module flips GO/NOGO")
        ax.set_ylim(0, 100)
        ax.set_title(f"{regime}  (n={n})", fontweight="bold",
                     color=REGIME_COLORS[regime])
        ax.grid(axis="y", alpha=0.3, lw=0.6)

    fig.suptitle(
        "Decision-Critical Fidelity Modules by Regime\n"
        "A bar at X% means: for X% of designs in this regime, removing this module "
        "changes the GO/NOGO engineering verdict.",
        fontsize=9.5,
    )
    out = FIG_DIR / "exp4a_decision_flips.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Figure 3: Fidelity complexity vs regime (how many modules needed) ─────────

def fig_fidelity_complexity(df: pd.DataFrame, tbl_complexity: pd.DataFrame):
    """
    Stacked bars showing distribution of n_decision_flips per regime.

    Key number: what fraction of designs can use a simple (0-module) simulator
    without changing their GO/NOGO verdict?
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)

    colors_n = ["#2ecc71", "#3498db", "#f39c12", "#e74c3c",
                "#8e44ad", "#7f8c8d", "#2c3e50", "#1abc9c"]
    max_k    = int(tbl_complexity["n_modules"].max())

    ax = axes[0]
    bottoms = np.zeros(len(REGIME_ORDER))
    for k in range(max_k + 1):
        vals = []
        for regime in REGIME_ORDER:
            row = tbl_complexity[(tbl_complexity.regime == regime) &
                                  (tbl_complexity.n_modules == k)]
            vals.append(row["pct"].iloc[0] if len(row) else 0.0)
        label = f"{k} module{'s' if k != 1 else ''}" + (" ← simple OK" if k == 0 else "")
        ax.bar(REGIME_ORDER, vals, bottom=bottoms,
               color=colors_n[min(k, len(colors_n)-1)], label=label,
               width=0.55, alpha=0.90)
        bottoms += np.array(vals)

    ax.set_ylabel("% of designs")
    ax.set_ylim(0, 108)
    ax.set_title("Simulator Complexity Needed\nper Regime", fontweight="bold")
    ax.legend(fontsize=7.5, loc="upper right")
    ax.grid(axis="y", alpha=0.3)
    for i, regime in enumerate(REGIME_ORDER):
        sub     = df[df["regime_label"] == regime]
        simple  = (sub["n_decision_flips"] == 0).mean() * 100
        ax.text(i, simple/2, f"{simple:.0f}%\nsimple OK",
                ha="center", va="center", fontsize=8.5, fontweight="bold",
                color="white")

    ax = axes[1]
    cmap = plt.get_cmap("RdYlGn_r", max_k + 1)
    sc   = ax.scatter(df["p_unstable"], df["servo_slew_deg_s"],
                      c=df["n_decision_flips"], cmap=cmap, vmin=0, vmax=max_k,
                      s=16, alpha=0.7, linewidths=0, zorder=3)
    cbar = plt.colorbar(sc, ax=ax, ticks=range(max_k + 1))
    cbar.set_label("# modules that flip decision")
    ax.set_xlabel("p_unstable  (instability rate, 1/s)")
    ax.set_ylabel("Servo slew rate  (deg/s)")
    ax.set_title("Fidelity Complexity Map\nGreen = simple sim OK; Red = many layers needed",
                 fontweight="bold")
    ax.grid(alpha=0.2)

    fig.suptitle(
        "How Many Fidelity Modules Does Each Design Require?\n"
        "n=0: simple simulator gives the same GO/NOGO as high-fidelity.  "
        "n>0: removing that module flips the answer.",
        fontsize=9.5,
    )
    out = FIG_DIR / "exp4a_fidelity_complexity.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out}")


# ── Print tables ──────────────────────────────────────────────────────────────

def print_tables(df, tbl_mismatch, tbl_complexity):
    agree_overall = df["baseline_agrees"].mean()
    print("\n" + "="*65)
    print("Exp4A: Decision-Disagreement Audit")
    print("="*65)

    print(f"\nOverall Exp4-full vs Exp1 GO/NOGO agreement: {agree_overall:.1%}")

    print("\n--- Baseline mismatch by regime ---")
    print(f"  {'Regime':<12s} {'Agree':>8s}  {'95% CI':>14s}  Exp4 GO  Exp1 GO")
    for _, r in tbl_mismatch.iterrows():
        print(f"  {r.regime:<12s} {r.agree_pct:>7.1f}%  "
              f"[{r.ci_lo:.1f}%, {r.ci_hi:.1f}%]  "
              f"{r.n_exp4_go:>7d}  {r.n_exp1_go:>7d}")

    print("\n--- Fidelity complexity (% of designs needing N modules) ---")
    for regime in REGIME_ORDER:
        sub = df[df["regime_label"] == regime]
        n   = len(sub)
        print(f"\n  {regime}  (n={n})")
        for k in range(8):
            cnt = (sub["n_decision_flips"] == k).sum()
            if cnt == 0: continue
            note = "  <- simple simulator OK" if k == 0 else ""
            print(f"    {k} module(s): {100*cnt/n:5.1f}%{note}")

    print("\n--- Per-module decision-flip rates (all 1200 designs) ---")
    print(f"  {'Module':<18s} {'Flip %':>8s}")
    for module in MODULES:
        col  = f"decision_flip_{module}"
        pct  = 100 * df[col].mean()
        print(f"  {MODULE_LABELS[module]:<18s} {pct:>7.1f}%")

    # Note about what Exp4A cannot yet answer
    print("\n--- What this audit does NOT yet answer ---")
    print("  The paired simple-vs-full run (Exp4A proper) requires running")
    print("  FidelityConfig.simple() for all 1200 designs to get actual")
    print("  simple-model GO/NOGO.  This is not in the current dataset.")
    print("  Current analysis uses n_decision_flips as a proxy only.")
    print("  Run: python sim/experiment_runner.py exp4simple")
    print("  to generate the paired data.")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Loading Exp4 ablation data...")
    df = load_data()
    print(f"  {len(df)} designs loaded")

    tbl_mismatch   = table_baseline_mismatch(df)
    tbl_complexity = table_fidelity_complexity_by_regime(df)

    print_tables(df, tbl_mismatch, tbl_complexity)

    print("\nGenerating figures...")
    print("[1/3] Baseline mismatch map")
    fig_baseline_mismatch(df, tbl_mismatch)

    print("[2/3] Per-module decision-flip rates")
    fig_decision_flips(df)

    print("[3/3] Fidelity complexity by regime")
    fig_fidelity_complexity(df, tbl_complexity)

    # Save audit results
    audit_out = EXP_DIR / "exp4a_decision_audit.csv"
    df[[
        "rocket_id", "design_id", "regime_label", "p_unstable", "servo_slew_deg_s",
        "exp4_full_go", "exp1_go", "baseline_agrees",
        "n_decision_flips", "candidate_false_optimism", "candidate_false_pessimism",
        "needs_paired_run",
    ] + [f"decision_flip_{m}" for m in MODULES]].to_csv(audit_out, index=False)
    print(f"\n  Saved: {audit_out}")
    print(f"\nAll outputs in {FIG_DIR}")
