"""
pi_s2r_gap_analysis.py  —  Pi = keff x lat^2 as a sim-to-real calibration gap predictor

RESEARCH QUESTION:
  Does Pi = keff x latency^2 predict the probability that a disturbance-free autotune returns
  a gain outside the real physics window, causing a false rejection?

KEY FINDING:
  - Pi < 300:  FR = 9.1%  (n=2320, baseline — driven by random calibration noise)
  - Pi >= 300: FR = 61.3% (n=80, systematic failure — overtuning OR near-infeasible)
  - 50% FR threshold: Pi ~= 354 (interpolated from monotone curve)
  - Mechanism: high keff -> autotune climbs to Kp_max=320 in still air (no wind to stop it)
               high lat -> real ceiling = 380/lat drops below 320
               Pi = keff x lat^2 captures both effects simultaneously

Data: experiments/results/exp4_s2r_gains_final_py.csv (n=2400)
      experiments/results/exp1_final_population_py.csv (regime labels)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from scipy import stats

# ── Constants ────────────────────────────────────────────────────────────────
F15_AVG   = 14.4
CU_TO_RAD = np.pi / 180 * (15 / 12)
L_NOZZLE  = 0.25


def load_data():
    df = pd.read_csv("experiments/results/exp4_s2r_gains_final_py.csv")
    pop = pd.read_csv("experiments/results/exp1_final_population_py.csv")[
        ["rocket_id", "td", "final_label"]
    ]
    df = df.merge(pop, on="rocket_id", how="left", suffixes=("", "_pop"))
    df["label"] = df["final_label"]
    df["keff"]  = F15_AVG * df["motor_scale"] * CU_TO_RAD * L_NOZZLE / df["Iyy"]
    df["Pi"]    = df["keff"] * df["latency_steps"] ** 2
    df["Kp_ceiling"] = 380.0 / df["latency_steps"]
    df["overtuned"]  = (df["kp_simple"] > df["Kp_ceiling"]).astype(int)
    df["log_ratio"]  = np.log(df["kp_simple"] / df["kp_full"])
    return df


def fr_vs_pi_table(df):
    """Monotone false-rejection rate table by Pi bin."""
    bins = [0, 50, 100, 150, 200, 300, 500, 1000, 5000]
    rows = []
    for i in range(len(bins) - 1):
        sub = df[(df["Pi"] >= bins[i]) & (df["Pi"] < bins[i + 1])]
        if len(sub) < 10:
            continue
        ot  = sub["overtuned"].mean()
        fr  = sub["false_rejection_v2"].mean()
        ot_sev = sub[sub["overtuned"] == 1]["kp_simple"].median() / sub[sub["overtuned"] == 1]["Kp_ceiling"].median() if ot > 0 else np.nan
        rows.append({
            "Pi_bin":    f"{bins[i]}-{bins[i+1]}",
            "n":         len(sub),
            "FR_pct":    round(fr * 100, 1),
            "overtune_pct": round(ot * 100, 1),
        })
    return pd.DataFrame(rows)


def fr_threshold(df):
    """Interpolate Pi threshold for 50% false rejection."""
    Pi_mids = [25, 75, 125, 175, 250, 400, 750]
    fr_vals = []
    for b in [(0,50),(50,100),(100,150),(150,200),(200,300),(300,500),(500,1000)]:
        sub = df[(df["Pi"] >= b[0]) & (df["Pi"] < b[1])]
        fr_vals.append(sub["false_rejection_v2"].mean())
    for i in range(len(Pi_mids) - 1):
        if fr_vals[i] < 0.50 <= fr_vals[i + 1]:
            x0, x1 = Pi_mids[i], Pi_mids[i + 1]
            y0, y1 = fr_vals[i], fr_vals[i + 1]
            return x0 + (0.50 - y0) / (y1 - y0) * (x1 - x0), Pi_mids, fr_vals
    return None, Pi_mids, fr_vals


def severity_by_pi(df):
    """Overtuning severity (kp_simple / Kp_ceiling) by Pi bin for overtuned designs."""
    bins = [0, 50, 100, 200, 300, 500, 1000]
    rows = []
    ot = df[df["overtuned"] == 1].copy()
    ot["severity"] = ot["kp_simple"] / ot["Kp_ceiling"]
    for i in range(len(bins) - 1):
        sub = ot[(ot["Pi"] >= bins[i]) & (ot["Pi"] < bins[i + 1])]
        if len(sub) < 5:
            continue
        rows.append({
            "Pi_bin":    f"{bins[i]}-{bins[i+1]}",
            "n_ot":      len(sub),
            "sev_median": round(sub["severity"].median(), 1),
            "FR_overtuned": round(sub["false_rejection_v2"].mean() * 100, 0),
        })
    return pd.DataFrame(rows)


def main():
    df = load_data()

    print("=" * 70)
    print("Pi = keff x latency^2 as a sim-to-real calibration gap predictor")
    print("=" * 70)

    # 1. Headline comparison
    below = df[df["Pi"] < 300]
    above = df[df["Pi"] >= 300]
    print(f"\n1. HEADLINE: Pi threshold at 300")
    print(f"   Pi < 300:  FR = {below.false_rejection_v2.mean()*100:.1f}%  (n={len(below)})")
    print(f"   Pi >= 300: FR = {above.false_rejection_v2.mean()*100:.1f}%  (n={len(above)})")
    print(f"   Ratio: {above.false_rejection_v2.mean() / below.false_rejection_v2.mean():.1f}x higher risk above Pi=300")

    # 2. Full monotone table
    print(f"\n2. MONOTONE TABLE: False rejection rate by Pi bin")
    tbl = fr_vs_pi_table(df)
    print(tbl.to_string(index=False))

    # 3. 50% threshold
    pi50, pi_mids, fr_vals = fr_threshold(df)
    if pi50:
        print(f"\n3. 50% FALSE REJECTION THRESHOLD: Pi = {pi50:.0f}")

    # 4. Overtuning severity
    print(f"\n4. OVERTUNING SEVERITY by Pi bin (overtuned designs only)")
    print(f"   (overtuned = kp_simple > 380/latency = real physics ceiling)")
    sev = severity_by_pi(df)
    print(sev.to_string(index=False))

    # 5. Mechanism: does keff predict hitting the search ceiling?
    print(f"\n5. MECHANISM: keff -> autotune reaches Kp_max=320")
    df["at_max"] = (df["kp_simple"] >= 315).astype(int)
    keff_bins = [0, 2, 4, 8, 15, 50]
    for i in range(len(keff_bins) - 1):
        sub = df[(df["keff"] >= keff_bins[i]) & (df["keff"] < keff_bins[i + 1])]
        if len(sub) < 20: continue
        p_max = sub["at_max"].mean() * 100
        print(f"   keff {keff_bins[i]:3.0f}-{keff_bins[i+1]:3.0f}: {p_max:5.1f}% hit Kp=320  (n={len(sub)})")

    # 6. Logistic regression P(FR | Pi)
    from scipy.special import expit
    log_Pi = np.log(df["Pi"] + 1)
    r, p   = stats.spearmanr(log_Pi, df["false_rejection_v2"])
    print(f"\n6. STATISTICAL SIGNIFICANCE")
    print(f"   Spearman rho(log_Pi, false_rejection) = {r:+.3f}, p={p:.1e}")

    # 7. Latency x keff jointly
    print(f"\n7. OVERTUNING RATE by latency x keff (shows Pi captures BOTH)")
    for lat in [2, 3, 4, 5, 6]:
        sub_lat = df[df["latency_steps"] == lat]
        Kp_ceil = 380.0 / lat
        above_ceil = (sub_lat["kp_simple"] > Kp_ceil).mean()
        fr_lat = sub_lat["false_rejection_v2"].mean()
        print(f"   lat={lat} (ceil={Kp_ceil:.0f}): {above_ceil*100:.1f}% above ceil, FR={fr_lat*100:.1f}%  (n={len(sub_lat)})")

    # 8. Key design rule
    print(f"\n8. PRACTICAL DESIGN RULE")
    print(f"   Step 1: Compute keff = T_avg * (pi/180 * 15/12) * L_nozzle / Iyy")
    print(f"   Step 2: Compute Pi = keff * latency_steps^2")
    print(f"   Step 3: If Pi > 350 -> 50%+ probability your disturbance-free tuned Kp is wrong")
    print(f"           Specifically: Kp_simple likely > 380/latency (real wind-resistance ceiling)")
    print(f"           Fix: cap Kp at 380/latency_steps in full-physics validation")
    print(f"           OR:  run with wind in autotune process")
    print(f"           OR:  use ADRC (ceiling-immune architecture)")

    # Save summary stats
    summary_rows = []
    bins = [0, 50, 100, 150, 200, 300, 500, 1000, 5000]
    for i in range(len(bins) - 1):
        sub = df[(df["Pi"] >= bins[i]) & (df["Pi"] < bins[i + 1])]
        if len(sub) < 10: continue
        summary_rows.append({
            "Pi_lo": bins[i], "Pi_hi": bins[i+1], "n": len(sub),
            "fr": sub["false_rejection_v2"].mean(),
            "overtune_rate": sub["overtuned"].mean(),
            "kp_simple_median": sub["kp_simple"].median(),
            "kp_ceiling_median": sub["Kp_ceiling"].median(),
            "kp_full_median": sub["kp_full"].median(),
            "mean_Pi": sub["Pi"].mean(),
        })
    summary_df = pd.DataFrame(summary_rows)
    summary_df.to_csv("experiments/results/pi_s2r_gap_summary_py.csv", index=False)

    # Also save full row-level data with Pi annotation
    out_cols = ["rocket_id", "keff", "Pi", "latency_steps", "Kp_ceiling",
                "kp_simple", "kp_full", "overtuned", "false_rejection_v2",
                "sr_simple_in_full", "sr_full_in_full", "wind_strength", "label"]
    save_cols = [c for c in out_cols if c in df.columns]
    df[save_cols].to_csv("experiments/results/pi_s2r_gap_py.csv", index=False)

    print(f"\nSaved: experiments/results/pi_s2r_gap_summary_py.csv")
    print(f"Saved: experiments/results/pi_s2r_gap_py.csv")


if __name__ == "__main__":
    main()
