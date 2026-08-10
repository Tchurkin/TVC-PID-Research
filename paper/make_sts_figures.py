"""
paper/make_sts_figures.py — publication figures for the STS report.

Figure numbers follow paper/OUTLINE_STS.md. Fig 5 was cut in the 20-page trim, and Fig 11
(measured bench tau) waits on the engineer's hardware dataset, so this builds 1-4 and 6-10 and 12.

COLOR. Categorical slots are the validated default palette's first three
(#2a78d6 blue, #eb6834 orange, #1baf7a aqua), which is the documented all-pairs-safe
subset for scatter forms. Verified with the skill's validator, light mode, --pairs all:
lightness band PASS, chroma floor PASS, CVD separation PASS (worst all-pairs dE 9.2 deutan),
normal-vision floor PASS (worst 24.0). One WARN: aqua sits at 2.74:1 on the light surface and
therefore requires "relief" -- so aqua is only ever used WITH a direct label or legend text, never
as the sole carrier of identity. Never more than three categorical series; magnitude uses the
blue sequential ramp instead.

Other rules applied: no dual axes anywhere; a legend whenever >= 2 series; text in ink tokens, not
series colors; recessive grid; thin marks.

OUTPUT: paper/figures/fig##_*.png at 300 dpi, sized to a single column (3.4 in) or full
width (7.0 in). PNG rather than vector because several panels carry 2400 scatter points and the
whole submission PDF must stay under 4 MB.

USAGE
  python paper/make_sts_figures.py                # all available
  python paper/make_sts_figures.py --only 3 7 8 9 # the section-6 set plus the title figure
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parent.parent
FIG  = ROOT / "paper" / "figures"
SCRATCH = Path(os.environ.get("STS_SCRATCH", "")) if os.environ.get("STS_SCRATCH") else None

# ---- validated palette ------------------------------------------------------
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED   = "#0b0b0b", "#52514e", "#8a8984"
SEQ = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]

plt.rcParams.update({
    "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8.5,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
    "axes.edgecolor": MUTED, "axes.linewidth": 0.6,
    "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.labelcolor": INK,
    "grid.color": "#e6e5e1", "grid.linewidth": 0.5,
    "figure.dpi": 300, "savefig.dpi": 300, "savefig.bbox": "tight",
    "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
})

COL, FULL = 3.4, 7.0


def _finish(fig, ax_or_axes, name: str):
    for ax in (ax_or_axes if isinstance(ax_or_axes, (list, np.ndarray)) else [ax_or_axes]):
        ax.grid(True, alpha=0.6, zorder=0)
        ax.set_axisbelow(True)
    FIG.mkdir(parents=True, exist_ok=True)
    out = FIG / f"{name}.png"
    fig.savefig(out)
    plt.close(fig)
    print(f"  wrote {out.relative_to(ROOT)}")


def _pop():
    p = SCRATCH / "hist/experiments/results/exp1_final_population_py.csv" if SCRATCH else \
        ROOT / "experiments/results/exp1_final_population_py.csv"
    d = pd.read_csv(p)
    d["tau"] = d.latency_steps / 200.0
    d["keff"] = 14.4 * d.motor_scale * 0.25 / d.Iyy
    d["fail"] = (d.final_label == "FRAGILE")
    return d


# ---- Fig 1: design space coverage -------------------------------------------
def fig1():
    d = _pop()
    fig, ax = plt.subplots(figsize=(COL, 2.6))
    s = ax.scatter(d.Iyy, d.latency_steps + np.random.default_rng(0).uniform(-.28, .28, len(d)),
                   c=np.log10(d.keff), cmap=matplotlib.colors.LinearSegmentedColormap.from_list("b", SEQ),
                   s=3, lw=0, alpha=.75, rasterized=True)
    ax.set_xscale("log")
    ax.set_xlabel("pitch inertia  $I_{yy}$  (kg·m²)")
    ax.set_ylabel("loop delay (control steps)")
    cb = fig.colorbar(s, ax=ax, pad=.02)
    cb.set_label("log₁₀ control authority  $k_{eff}$", fontsize=7)
    cb.ax.tick_params(labelsize=6)
    cb.outline.set_linewidth(0.4)
    _finish(fig, ax, "fig01_design_space")


# ---- Fig 3: the failure map (title figure) -----------------------------------
def fig3():
    d = _pop()
    rng = np.random.default_rng(1)
    jit = rng.uniform(-.28, .28, len(d))
    fig, ax = plt.subplots(figsize=(FULL * 0.62, 3.1))
    ok, bad = d[~d.fail], d[d.fail]
    ax.scatter(ok.Iyy, ok.latency_steps + jit[~d.fail.values], s=4, lw=0,
               color=BLUE, alpha=.30, label=f"tunable  (n={len(ok)})", rasterized=True)
    ax.scatter(bad.Iyy, bad.latency_steps + jit[d.fail.values], s=26, lw=.6,
               facecolor=ORANGE, edgecolor="white", zorder=5,
               label=f"failed to tune  (n={len(bad)})")
    # No cutoff line. The 25th-percentile framing was RETIRED 2026-08-10: the binding failure sits
    # at the 24.79th percentile, so 25 is the smallest round number giving 100% and reads as a
    # threshold chosen to flatter. Annotate the threshold-free statistic instead — it is stronger,
    # carries a p-value, and leaves the reader nothing to measure with a ruler.
    med_f, med_ok = d[d.fail].Iyy.median(), d[~d.fail].Iyy.median()
    ax.axvline(med_f,  color=ORANGE, lw=.9, ls=(0, (4, 3)), zorder=4)
    ax.axvline(med_ok, color=BLUE,   lw=.9, ls=(0, (4, 3)), zorder=4)
    ax.annotate(f"median $I_{{yy}}$   failed {med_f:.4f}  vs  tunable {med_ok:.4f}  (0.19×)\n"
                "a failure has lower $I_{yy}$ than a survivor "
                "93.7% of the time  ($p$ = 9×10$^{-20}$)",
                xy=(0.5, -0.30), xycoords="axes fraction", fontsize=6.2, color=INK2,
                va="top", ha="center")
    ax.set_xscale("log")
    ax.set_xlabel("pitch inertia  $I_{yy}$  (kg·m²)")
    ax.set_ylabel("loop delay (control steps)")
    ax.set_ylim(0.3, 7.4)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.14), ncol=2,
              handletextpad=.5, markerscale=2.2, columnspacing=1.8)
    _finish(fig, ax, "fig03_failure_map")


# ---- Fig 4: performance frontier ---------------------------------------------
def fig4():
    p = SCRATCH / "hist/experiments/results/performance_frontier_py.csv"
    f = pd.read_csv(p)
    pop = _pop()[["rocket_id", "Iyy", "motor_scale", "latency_steps"]]
    m = f.merge(pop, on="rocket_id", how="left").dropna(subset=["Iyy"])
    x = (14.4 * m.motor_scale * 0.25 / m.Iyy) * (m.latency_steps / 200.0) ** 2
    fig, ax = plt.subplots(figsize=(COL, 2.5))
    ax.scatter(x, m.peak_pid_sr, s=14, lw=.5, color=BLUE, edgecolor="white", zorder=3)
    o = np.argsort(x.values)
    k = max(5, len(x) // 6)
    sm = pd.Series(m.peak_pid_sr.values[o]).rolling(k, center=True, min_periods=2).median()
    ax.plot(x.values[o], sm, color=ORANGE, lw=1.6, zorder=4)
    ax.set_xscale("log")
    ax.set_xlabel("authority × delay²   (plotting coordinate)")
    ax.set_ylabel("peak achievable success rate")
    ax.set_ylim(-0.03, 1.05)
    ax.legend(handles=[Line2D([], [], marker="o", ls="", color=BLUE, ms=4, label="design (n=63)"),
                       Line2D([], [], color=ORANGE, lw=1.6, label="rolling median")],
              loc="lower left")
    _finish(fig, ax, "fig04_performance_frontier")


# ---- Fig 6: gain ceiling vs delay --------------------------------------------
def fig6():
    p = SCRATCH / "ceiling_v2_full_RESULT.csv"
    d = pd.read_csv(p)
    d = d[(d.ceil_kd1_censored == 0) & d.ceiling_kd1.notna()]
    fig, ax = plt.subplots(figsize=(COL, 2.6))
    s = ax.scatter(d.tau, d.ceiling_kd1, c=np.log10(d.keff), s=16, lw=.4, edgecolor="white",
                   cmap=matplotlib.colors.LinearSegmentedColormap.from_list("b", SEQ), zorder=3)
    t = np.linspace(d.tau.min(), d.tau.max(), 50)
    CU = np.pi / 180 * (15 / 12)
    ax.plot(t, 0.042 / t / CU, color=INK2, lw=1.2, ls=(0, (5, 3)), zorder=4)
    ax.plot(t, 0.0661 / t / CU, color=ORANGE, lw=1.6, zorder=5)
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("loop delay  τ  (s)")
    ax.set_ylabel("measured gain ceiling")
    cb = fig.colorbar(s, ax=ax, pad=.02); cb.set_label("log₁₀ $k_{eff}$", fontsize=7)
    cb.ax.tick_params(labelsize=6); cb.outline.set_linewidth(0.4)
    ax.legend(handles=[Line2D([], [], color=INK2, lw=1.2, ls=(0, (5, 3)), label="published 0.042/τ"),
                       Line2D([], [], color=ORANGE, lw=1.6, label="recalibrated 0.0661/τ")],
              loc="upper right")
    _finish(fig, ax, "fig06_gain_ceiling")


# ---- Fig 7 & 8: sim-to-real ---------------------------------------------------
def _s2r():
    return pd.read_csv(ROOT / "experiments/results/s2r_replication_py.csv").drop_duplicates("rocket_id")


def fig7():
    d = _s2r()
    bins = [0, 50, 100, 150, 200, 300, 500, 1000]
    mid, rate, n = [], [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        g = d[(d.Pi >= lo) & (d.Pi < hi)]
        if len(g) < 10:
            continue
        mid.append(np.sqrt(max(lo, 1) * hi)); rate.append(g.fail_a_gate035.mean()); n.append(len(g))
    fig, ax = plt.subplots(figsize=(COL, 2.4))
    ax.plot(mid, rate, color=BLUE, lw=1.8, marker="o", ms=5, mec="white", mew=.8, zorder=4)
    for x, y, k in zip(mid, rate, n):
        ax.annotate(f"n={k}", (x, y), textcoords="offset points", xytext=(0, 7),
                    ha="center", fontsize=5.5, color=MUTED)
    ax.set_xscale("log")
    ax.set_xlabel("authority × delay²   (plotting coordinate)")
    ax.set_ylabel("fraction of still-air-tuned gains\nthat fail under full physics")
    ax.set_ylim(-0.03, 0.95)
    _finish(fig, ax, "fig07_s2r_dose_response")


def fig8():
    """THE key figure: same designs, same axis, only what the tuner does to D when P moves.

    NOTE the labelling. This is NOT "sequential vs coupled tuning" -- readers hear "sequential" as
    ordinary manual practice, which is ALTERNATING (adjust P, re-adjust D, repeat) and preserves the
    ratio. The artifact is specifically D held STALE while P sweeps, which is what autotuners and
    sim campaigns do. This project measured no manual tuning (Braxton, domain correction 2026-08-09).
    """
    d = _s2r()
    bins = [0, 50, 100, 150, 200, 300, 500, 1000]
    mid, a, b, n = [], [], [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        g = d[(d.Pi >= lo) & (d.Pi < hi)]
        if len(g) < 10:
            continue
        mid.append(np.sqrt(max(lo, 1) * hi))
        a.append(g.fail_a_gate035.mean()); b.append(g.fail_b_gate035.mean()); n.append(len(g))
    fig, ax = plt.subplots(figsize=(FULL * 0.58, 2.9))
    ax.plot(mid, a, color=ORANGE, lw=2.0, marker="o", ms=5.5, mec="white", mew=.8, zorder=5,
            label="stale D  —  D fixed while P sweeps (autotuner)")
    ax.plot(mid, b, color=BLUE, lw=2.0, marker="s", ms=5, mec="white", mew=.8, zorder=5,
            label="ratio preserved  —  D re-tuned with P")
    ax.fill_between(mid, b, a, color=ORANGE, alpha=.10, zorder=2)
    # Label each endpoint with ITS OWN bin value. An earlier version put the aggregate
    # Pi>=300 rates (70.0% / 1.3%) on these points, which are 78.3% / 4.3% -- two wrong
    # numbers in the paper's key figure. The aggregate belongs to the shaded region, not a point.
    ax.annotate(f"{a[-1]*100:.1f}%", (mid[-1], a[-1]), textcoords="offset points", xytext=(6, 0),
                ha="left", va="center", fontsize=7, color=ORANGE, fontweight="bold")
    ax.annotate(f"{b[-1]*100:.1f}%", (mid[-1], b[-1]), textcoords="offset points", xytext=(6, 0),
                ha="left", va="center", fontsize=7, color=BLUE, fontweight="bold")
    hi = d[d.Pi >= 300]
    ax.axvspan(300, max(mid) * 1.6, color=MUTED, alpha=.07, zorder=1)
    # Report COUNTS, not a 1-decimal rate. The coupled arm is 1 failure in 80 designs, so
    # "1.3%" / "1.2%" are both just rounding artifacts of 1/80 = 1.25% and imply a precision
    # that a single event does not carry. A reader needs the denominator.
    ka, kb, nh = int(hi.fail_a_gate035.sum()), int(hi.fail_b_gate035.sum()), len(hi)
    ax.annotate(f"authority × delay²  ≥ 300\n"
                f"{ka} of {nh} fail   vs   {kb} of {nh}",
                xy=(300 * 1.15, 0.90), fontsize=6.4, color=INK2, va="top", ha="left")
    ax.set_xscale("log")
    ax.set_xlabel("authority × delay²   (plotting coordinate)")
    ax.set_ylabel("fraction of tuned gains that fail\nunder full physics")
    ax.set_ylim(-0.05, 0.95)
    ax.set_xlim(min(mid) * 0.7, max(mid) * 1.75)
    ax.legend(loc="upper left", bbox_to_anchor=(0, 0.86))
    _finish(fig, ax, "fig08_two_tuners")


# ---- Fig 9: the mechanism -----------------------------------------------------
def fig9():
    """Why stale-D tuning fails: D is chosen at one probe gain and then held while P sweeps."""
    fig, ax = plt.subplots(figsize=(COL, 2.6))
    kp = np.geomspace(1, 320, 200)
    ax.plot(kp, 0.05 * kp, color=BLUE, lw=1.8, zorder=4, label="ratio preserved: D re-tuned with P")
    ax.axhline(2.0, color=ORANGE, lw=1.8, zorder=4, label="stale D: frozen at the probe gain")
    ax.axvline(40, color=MUTED, lw=.8, ls=(0, (3, 3)), zorder=3)
    ax.annotate("D chosen here\n(probe gain, P = 40)", (40, 11), fontsize=6.2, color=INK2,
                ha="center", va="top")
    ax.annotate("mismatch grows\nwith the final gain", (300, 6.5), fontsize=6.2, color=ORANGE,
                ha="right", va="center")
    ax.fill_between(kp, np.minimum(0.05 * kp, 2.0), np.maximum(0.05 * kp, 2.0),
                    color=ORANGE, alpha=.09, zorder=2)
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("proportional gain  P  (searched 1 → 320)")
    ax.set_ylabel("derivative gain  D")
    ax.legend(loc="upper left")
    _finish(fig, ax, "fig09_tuning_mechanism")


# ---- Fig 10: flight signature -------------------------------------------------
def fig10():
    p = SCRATCH / "hist/experiments/results/flight_sig_final_py.csv"
    d = pd.read_csv(p)
    m = d.groupby(["rocket_id", "is_fragile"]).agg(rms=("rms", "mean"),
                                                   sat=("slew_sat", "mean")).reset_index()
    fig, axes = plt.subplots(1, 2, figsize=(FULL * 0.72, 2.5))
    for ax, col, lab in ((axes[0], "rms", "attitude RMS (deg)"),
                         (axes[1], "sat", "fraction of burn rate-saturated")):
        for k, (cl, color, name) in enumerate(((0, BLUE, "tunable"), (1, ORANGE, "failed"))):
            v = m[m.is_fragile == cl][col].values
            ax.scatter(np.full(len(v), k) + np.random.default_rng(2).uniform(-.11, .11, len(v)),
                       v, s=13, lw=.4, color=color, edgecolor="white", zorder=3)
            ax.hlines(np.median(v), k - .26, k + .26, color=INK, lw=1.6, zorder=4)
        ax.set_xticks([0, 1]); ax.set_xticklabels(["tunable", "failed"])
        ax.set_ylabel(lab)
        ax.set_xlim(-.5, 1.5)
    axes[0].annotate("13.3° ± 5.2", (1, 13.3), textcoords="offset points", xytext=(14, 0),
                     fontsize=6.5, color=ORANGE, va="center")
    axes[0].annotate("3.8° ± 2.7", (0, 3.8), textcoords="offset points", xytext=(-14, 0),
                     fontsize=6.5, color=BLUE, va="center", ha="right")
    _finish(fig, axes, "fig10_flight_signature")


# ---- Fig 12: the audit --------------------------------------------------------
def fig12():
    """Floor exponent on authority across four measurement protocols."""
    labels = ["v2\nKd frozen", "v3\nKd frozen\nfixed wind", "v4\nKd free", "v5\nKd ratio-\nconstrained"]
    vals   = [1.061, 1.074, 0.206, -0.196]
    ns     = [104, 98, 39, 10]
    fig, ax = plt.subplots(figsize=(COL, 2.5))
    cols = [ORANGE, ORANGE, BLUE, BLUE]
    ax.bar(range(4), vals, color=cols, width=.62, zorder=3, edgecolor="white", lw=.8)
    ax.axhline(0, color=INK2, lw=.8, zorder=4)
    ax.axhline(1.0, color=MUTED, lw=.8, ls=(0, (4, 3)), zorder=4)
    ax.annotate("theory: +1.0", (3.42, 1.0), fontsize=6.2, color=INK2, va="bottom", ha="right")
    for i, (v, k) in enumerate(zip(vals, ns)):
        ax.annotate(f"{v:+.2f}\nn={k}", (i, v), textcoords="offset points",
                    xytext=(0, 6 if v > 0 else -16), ha="center", fontsize=6.2, color=INK2)
    ax.set_xticks(range(4)); ax.set_xticklabels(labels, fontsize=6.2)
    ax.set_ylabel("fitted exponent of the gain floor\non control authority")
    ax.set_ylim(-0.75, 1.45)
    ax.legend(handles=[Line2D([], [], color=ORANGE, lw=6, label="derivative gain frozen"),
                       Line2D([], [], color=BLUE, lw=6, label="derivative gain free")],
              loc="lower left")
    _finish(fig, ax, "fig12_floor_exponent_audit")


ALL = {1: fig1, 3: fig3, 4: fig4, 6: fig6, 7: fig7, 8: fig8, 9: fig9, 10: fig10, 12: fig12}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, nargs="*", default=None)
    a = ap.parse_args()
    want = a.only if a.only else sorted(ALL)
    for k in want:
        if k not in ALL:
            print(f"  fig {k}: not available (Fig 5 cut in the 20-page trim; "
                  f"Fig 11 awaits the bench tau dataset)")
            continue
        try:
            ALL[k]()
        except Exception as e:
            print(f"  fig {k} FAILED: {type(e).__name__}: {e}")


# ---- Fig 13: the retrospective flight-signature test ---------------------------
def fig13():
    """Five archived flights placed on the simulation's own RMS scale.

    Result of the pre-registered retrospective test (paper/RETRO_FLIGHT_SIG_SPEC.md,
    tools/retro_flight_signature.py). Log-x because two real failures land an order of
    magnitude beyond anything the simulated population produces -- which is itself the
    finding: the healthy scale transfers, the failure scale does not.
    """
    sim = pd.read_csv(SCRATCH / "hist/experiments/results/flight_sig_final_py.csv")
    m   = sim.groupby(["rocket_id", "is_fragile"]).agg(rms=("rms", "mean")).reset_index()
    fl  = pd.read_csv(ROOT / "paper" / "retro_flight_signature.csv")

    fig, ax = plt.subplots(figsize=(FULL * 0.78, 2.9))
    rng = np.random.default_rng(4)
    for cl, color, name in ((0, BLUE, "simulated: tunable"), (1, ORANGE, "simulated: failed")):
        v = m[m.is_fragile == cl].rms.values
        y = 0.72 + (0.30 if cl else 0.0) + rng.uniform(-.075, .075, len(v))
        ax.scatter(v, y, s=11, lw=.35, color=color, edgecolor="white", zorder=3, label=name)

    ax.axvspan(5.37, 5.62, color=MUTED, alpha=.30, zorder=1, lw=0)
    ax.axvline(5.62, color=INK, lw=.9, ls=(0, (4, 3)), zorder=4)
    ax.annotate("pre-registered threshold 5.62°\n(band = ±5% reproducibility)",
                (5.9, 1.24), fontsize=6.4, color=INK, ha="left", va="center")

    # dodge: the two healthy flights sit close together on a log axis
    dodge = {"ASC036": (0, -14), "ASC031": (-3, 10),
             "ASC038": (0, 10), "LOG001": (0, -14)}
    for _, r in fl.iterrows():
        good = r["lab"] == 0
        ax.scatter([r["score_rep"]], [0.30], s=52, zorder=6,
                   marker="o" if good else "X",
                   color=BLUE if good else ORANGE, edgecolor="white", lw=.8)
        ax.annotate(r["id"], (r["score_rep"], 0.30), textcoords="offset points",
                    xytext=dodge.get(r["id"], (0, -14)), fontsize=6.3, ha="center",
                    color=BLUE if good else ORANGE)

    ax.set_yticks([0.30, 0.72, 1.02])
    ax.set_yticklabels(["flown\n(n=5)", "simulated\ntunable", "simulated\nfailed"], fontsize=6.8)
    ax.set_ylim(0.02, 1.40)
    ax.set_xscale("log")
    ax.set_xlim(1.2, 260)
    ax.set_xlabel("attitude RMS over the boost (deg, log scale)")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.34), ncol=2, handletextpad=.3,
              columnspacing=1.4)
    _finish(fig, ax, "fig13_retro_flight_test")
