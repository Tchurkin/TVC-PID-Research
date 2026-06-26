"""
Priority figure: slew-saturation fraction (fsat) vs. the authority-delay parameter Pi.

This is the single image that communicates the central result of the paper:
  - A saturation regime transition organized by Pi = keff * tau^2.
  - The transition is a BAND, not a line (delay-model spread + genuine scatter).
  - R0522 (highest authority in the 2,400-design population, lat=1) sits deep in the
    linear regime at Pi=41 with fsat~0 -> authority alone does not cause saturation.
  - ADRC's advantage over optimal PID turns on in the same Pi region.

Data: experiments/results/saturation_regime_map_py.csv (n=29, integer FIFO delay model).
Pi is in NATIVE units in the CSV; physical Pi = native / 872.8 (= CU_TO_RAD / dt^2).

Output: outputs/fsat_vs_pi_regime_map.png and .svg
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "experiments" / "results" / "saturation_regime_map_py.csv"
CSV_LARGE = ROOT / "experiments" / "results" / "saturation_transition_large_py.csv"
OUT_PNG = ROOT / "outputs" / "fsat_vs_pi_regime_map.png"
OUT_SVG = ROOT / "outputs" / "fsat_vs_pi_regime_map.svg"


def load_data():
    """Baseline n=29 map, merged with the larger replication if present (dedup by rocket_id).
    R0522 lives in the baseline, so it is always retained for the highlight."""
    df = pd.read_csv(CSV)
    if CSV_LARGE.exists():
        big = pd.read_csv(CSV_LARGE)
        cols = [c for c in df.columns if c in big.columns]
        df = (pd.concat([df[cols], big[cols]], ignore_index=True)
                .drop_duplicates(subset="rocket_id", keep="first")
                .reset_index(drop=True))
    return df

# native -> physical Pi conversion (CU_TO_RAD / dt^2 = 0.02182 / 0.005^2)
NATIVE_TO_PHYS = 1.0 / 872.8

# Transition band (native Pi). Spans the measured delay-model onset spread
# (bare-metal integer FIFO ~177 to lag_delay ~407; Section 4.4.6) -> honest "band not line".
BAND_LO, BAND_HI = 177.0, 407.0          # native
ADRC_ONSET = 321.0                        # native (performance frontier, ADRC advantage onset)
FSAT_SAT = 0.35                           # saturation diagnostic threshold
FSAT_TRANS = 0.10                         # transitional onset threshold

REGIME_COLORS = {
    "linear":       "#2c7fb8",   # blue
    "transitional": "#d95f0e",   # orange
    "saturation":   "#c0392b",   # red
}
REGIME_LABEL = {
    "linear":       "Linear (fsat < 0.10)",
    "transitional": "Transitional (0.10-0.35)",
    "saturation":   "Saturation (fsat > 0.35)",
}


def main():
    df = load_data()
    n_total = len(df)

    fig, ax = plt.subplots(figsize=(9.2, 6.0))

    # --- shaded vertical zones ---------------------------------------------
    xmin, xmax = 28.0, 1500.0
    ax.axvspan(xmin, BAND_LO, color="#2c7fb8", alpha=0.06, zorder=0)
    ax.axvspan(BAND_LO, BAND_HI, color="#f1c40f", alpha=0.14, zorder=0)
    ax.axvspan(BAND_HI, xmax, color="#c0392b", alpha=0.07, zorder=0)

    # zone labels
    ax.text(np.sqrt(xmin * BAND_LO), 0.79, "LINEAR\nregime",
            ha="center", va="top", fontsize=10, color="#1b4f72", weight="bold", alpha=0.8)
    ax.text(np.sqrt(BAND_LO * BAND_HI), 0.79, "TRANSITION\nband",
            ha="center", va="top", fontsize=10, color="#9a7d0a", weight="bold", alpha=0.9)
    ax.text(np.sqrt(BAND_HI * xmax), 0.79, "SATURATION\nregime",
            ha="center", va="top", fontsize=10, color="#7b241c", weight="bold", alpha=0.8)

    # --- threshold guide lines ---------------------------------------------
    ax.axhline(FSAT_SAT, ls="--", lw=1.1, color="#555555", alpha=0.8, zorder=1)
    ax.text(xmax * 0.96, FSAT_SAT + 0.012, "saturation diagnostic  fsat = 0.35",
            ha="right", va="bottom", fontsize=8.5, color="#555555")

    ax.axvline(ADRC_ONSET, ls=":", lw=1.6, color="#6c3483", alpha=0.9, zorder=1)
    ax.text(ADRC_ONSET * 1.03, 0.015,
            "ADRC advantage\nonset  (Pi = 321)",
            ha="left", va="bottom", fontsize=8.5, color="#6c3483")

    # band edge annotation
    ax.text(BAND_LO, 0.83, "180", ha="center", va="bottom", fontsize=7.5, color="#9a7d0a")
    ax.text(BAND_HI, 0.83, "410", ha="center", va="bottom", fontsize=7.5, color="#9a7d0a")

    # --- scatter by regime --------------------------------------------------
    for regime, sub in df.groupby("regime"):
        ax.scatter(sub["Pi"], sub["fsat"],
                   s=58, c=REGIME_COLORS.get(regime, "#888888"),
                   edgecolors="white", linewidths=0.6, zorder=3,
                   label=REGIME_LABEL.get(regime, regime))

    # --- highlight R0522 (the hero counterexample) -------------------------
    hero = df[df["rocket_id"] == "R0522"].iloc[0]
    ax.scatter([hero["Pi"]], [hero["fsat"]], s=240, marker="*",
               c="#f1c40f", edgecolors="#1b2631", linewidths=1.3, zorder=5)
    ax.annotate(
        "R0522 - highest authority in 2,400 designs\n"
        "(keff = 41 s$^{-2}$, the population max) but lat = 1\n"
        "-> Pi = 41, fsat = 0.018, SR = 1.000\n"
        "Authority alone would predict danger; Pi predicts safe.",
        xy=(hero["Pi"], hero["fsat"]), xytext=(70, 0.40),
        fontsize=8.6, color="#1b2631",
        bbox=dict(boxstyle="round,pad=0.4", fc="#fef9e7", ec="#b7950b", lw=1.0),
        arrowprops=dict(arrowstyle="-|>", color="#b7950b", lw=1.4,
                        connectionstyle="arc3,rad=-0.2"), zorder=6)

    # --- axes ---------------------------------------------------------------
    ax.set_xscale("log")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(-0.02, 0.85)
    ax.set_xlabel(r"$\Pi = k_{\mathrm{eff}} \times \tau^{2}$  =  authority $\times$ delay$^{2}$  (log scale)", fontsize=11)
    ax.set_ylabel("Servo saturation fraction, fsat\n(0 = never rate-limited, 1 = always)", fontsize=10.5)
    ax.set_title("The saturation regime transition\n"
                 "As authority $\\times$ delay$^2$ grows, the servo goes from never-saturated "
                 "(linear: any gain flies)\nto always-saturated (bang-bang: no gain works). "
                 "Only the product matters — see R0522.",
                 fontsize=10.8, weight="bold")

    # secondary top axis in physical units
    secax = ax.secondary_xaxis("top",
                               functions=(lambda x: x * NATIVE_TO_PHYS,
                                          lambda x: x / NATIVE_TO_PHYS))
    secax.set_xlabel(r"$\Pi$ (physical units, $k_{\mathrm{eff}}\,[\mathrm{s}^{-2}] \times \tau^{2}\,[\mathrm{s}^{2}]$)",
                     fontsize=10)

    ax.grid(True, which="both", ls=":", lw=0.5, alpha=0.4)

    # legend (regimes + R0522 marker)
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Line2D([0], [0], marker="*", color="w", markerfacecolor="#f1c40f",
                          markeredgecolor="#1b2631", markersize=15, label="R0522 (global authority max)"))
    ax.legend(handles=handles, loc="lower right", fontsize=8.8, framealpha=0.92)

    fig.text(0.013, 0.012,
             f"n = {n_total} designs, integer-FIFO delay, probe Kp = 0.5x DIPDT ceiling, 20 seeds. "
             "The transition is a band, not a constant: high-$\\Pi$ scatter (e.g. R2106 at $\\Pi$=1147, fsat=0.20) shows fsat is not a step function of $\\Pi$. "
             "Band edges = delay-model onset spread (Section 4.4.6); $\\tau^{2}$ predicted by the blind-spot derivation, consistent with data, not uniquely identified.",
             fontsize=6.6, color="#666666")

    fig.tight_layout(rect=(0, 0.022, 1, 1))
    fig.savefig(OUT_PNG, dpi=200)
    fig.savefig(OUT_SVG)
    print(f"wrote {OUT_PNG}")
    print(f"wrote {OUT_SVG}")
    # quick sanity summary
    print("\nregime counts:", df["regime"].value_counts().to_dict())
    print("R0522:", dict(Pi=hero["Pi"], fsat=round(float(hero["fsat"]), 4),
                         keff=hero["keff"], lat=int(hero["lat"]), regime=hero["regime"]))


if __name__ == "__main__":
    main()
