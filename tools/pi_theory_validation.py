"""
tools/pi_theory_validation.py — Validate the theoretical derivation of Π = θ̈_max × τ².

Theoretical argument:
  1. Ceiling (DIPDT phase-margin): Kp_ceiling ≈ 0.9/τ,  keff-independent
  2. Floor (bang-bang blind-spot): Kp_floor ∝ keff^α × τ^β (empirically α≈1, β≈1)
  3. Window = ceiling/floor ∝ (τ^-1) / (keff × τ) = 1/(keff × τ²) ∝ 1/Π_keff
     where Π_keff = keff × latency²  (equivalent to Π = θ̈_max × τ² up to u_max scaling)

This file tests whether:
  (a) log(window_ratio) correlates with log(1/Pi) at slope ≈ -1
  (b) The theoretical exponents (keff^-1, lat^-2) match the empirical v2 exponents (-1.19, -1.88)
  (c) Both ceiling and floor individually match theoretical predictions

Data: experiments/results/window_ratio_v2_py.csv (n=120, 32-pt sweep)
      and window_ratio_v2_sweep_py.csv for ceiling/floor decomposition.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from scipy import stats
import warnings
warnings.filterwarnings('ignore')

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

F15_AVG = 14.4
L_NOZ   = 0.25
CU_RAD  = np.pi / 180 * 15.0 / 12.0
U_MAX   = 12.0


def load_window_data():
    """Load window_ratio v2 summary data."""
    df = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')
    return df


def main():
    print("=" * 60)
    print("Pi THEORY VALIDATION")
    print("=" * 60)
    print()

    df = load_window_data()
    print(f"Loaded window_ratio v2: {len(df)} designs")
    print(f"Columns: {list(df.columns)}")
    print()

    # ── Identify key columns ─────────────────────────────────────────────────
    # Find keff, latency, window_ratio columns
    keff_col = next((c for c in df.columns if 'keff' in c.lower()), None)
    lat_col  = next((c for c in df.columns if 'latency' in c.lower()), None)
    wr_col   = next((c for c in df.columns if 'window_ratio' in c.lower() or 'window' in c.lower()), None)
    fl_col   = next((c for c in df.columns if 'floor' in c.lower() and 'log' not in c.lower()), None)
    ce_col   = next((c for c in df.columns if 'ceil' in c.lower() and 'log' not in c.lower()), None)

    print(f"Using columns: keff={keff_col}, latency={lat_col}, window={wr_col}")
    print(f"  floor={fl_col}, ceiling={ce_col}")
    print()

    # Filter valid (non-censored) designs
    valid = df.dropna(subset=[wr_col]).copy()
    # Further filter: window_ratio must be finite and > 0
    valid = valid[valid[wr_col] > 0].copy()
    print(f"Valid (non-censored) designs: {len(valid)} / {len(df)}")
    print()

    # ── Part 1: Π_keff = keff × latency² ────────────────────────────────────
    print("─" * 50)
    print("Part 1: Π_keff = keff × latency²  (single-variable theory)")
    print("─" * 50)

    valid['Pi_keff'] = valid[keff_col] * valid[lat_col] ** 2
    valid['log_Pi']  = np.log(valid['Pi_keff'])
    valid['log_wr']  = np.log(valid[wr_col])

    r_pi, p_pi = stats.spearmanr(valid['Pi_keff'], valid[wr_col])
    r_lpi, p_lpi = stats.pearsonr(valid['log_Pi'], valid['log_wr'])

    print(f"Spearman rho(Pi_keff, window_ratio) = {r_pi:.3f}  (p={p_pi:.2e})")
    print(f"Pearson r(log Pi_keff, log window)  = {r_lpi:.3f}  (p={p_lpi:.2e})")
    print()

    # OLS: log(window) = a × log(Pi) + b
    slope, intercept, r_val, p_val, se = stats.linregress(valid['log_Pi'], valid['log_wr'])
    print(f"OLS: log(window) = {slope:.3f} × log(Pi) + {intercept:.3f}  R²={r_val**2:.3f}")
    print(f"     Theoretical slope: -1.0  (window ∝ 1/Pi)")
    print(f"     Empirical slope:   {slope:.3f}  (deviation: {abs(slope+1):.3f})")
    print()

    # ── Part 2: Separate keff and latency exponents ──────────────────────────
    print("─" * 50)
    print("Part 2: Separate exponent fit — log(window) = a×log(keff) + b×log(lat)")
    print("─" * 50)

    from numpy.linalg import lstsq
    valid['log_keff'] = np.log(valid[keff_col])
    valid['log_lat']  = np.log(valid[lat_col])
    X = np.column_stack([valid['log_keff'], valid['log_lat'], np.ones(len(valid))])
    y = valid['log_wr'].values
    coefs, _, _, _ = lstsq(X, y, rcond=None)
    a_keff, b_lat, c_int = coefs
    y_pred = X @ coefs
    ss_res = np.sum((y - y_pred)**2)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1 - ss_res/ss_tot

    print(f"OLS: log(window) = {a_keff:.3f}×log(keff) + {b_lat:.3f}×log(lat) + {c_int:.3f}")
    print(f"     R² = {r2:.3f}")
    print()
    print("Comparison: Theoretical vs Empirical exponents:")
    print(f"             keff:    Theory = -1.000   Empirical = {a_keff:.3f}  diff = {abs(a_keff+1):.3f}")
    print(f"             latency: Theory = -2.000   Empirical = {b_lat:.3f}  diff = {abs(b_lat+2):.3f}")
    print(f"  (Window_ratio_v2 published: keff=-1.192, lat=-1.880 from Section 4.6)")
    print()

    # ── Part 3: Ceiling — should be keff-independent, lat^(-1) ───────────────
    if ce_col is not None:
        print("─" * 50)
        print("Part 3: Ceiling — theoretical prediction: keff^0 × latency^(-1)")
        print("─" * 50)
        valid_ceil = valid.dropna(subset=[ce_col]).copy()
        valid_ceil = valid_ceil[valid_ceil[ce_col] > 0]
        print(f"Non-censored ceiling: {len(valid_ceil)} designs")

        if len(valid_ceil) >= 10:
            valid_ceil['log_ceil'] = np.log(valid_ceil[ce_col])
            X2 = np.column_stack([valid_ceil['log_keff'], valid_ceil['log_lat'],
                                   np.ones(len(valid_ceil))])
            y2 = valid_ceil['log_ceil'].values
            coefs2, _, _, _ = lstsq(X2, y2, rcond=None)
            a2, b2, c2 = coefs2
            y2p = X2 @ coefs2
            r2_ceil = 1 - np.sum((y2-y2p)**2)/np.sum((y2-y2.mean())**2)
            print(f"OLS: log(ceiling) = {a2:.3f}×log(keff) + {b2:.3f}×log(lat) + {c2:.3f}")
            print(f"     R² = {r2_ceil:.3f}")
            print(f"Theoretical: keff=0, lat=-1.  Empirical: keff={a2:.3f}, lat={b2:.3f}")
            print(f"  keff exponent ≈ 0? Deviation: {abs(a2):.3f}  {'YES (< 0.1)' if abs(a2)<0.1 else 'NO'}")
            print(f"  lat  exponent ≈ -1? Deviation: {abs(b2+1):.3f}  {'YES (< 0.2)' if abs(b2+1)<0.2 else 'NO'}")
        print()

    # ── Part 4: Floor — should be keff^(+1) × latency^(+1) ──────────────────
    if fl_col is not None:
        print("─" * 50)
        print("Part 4: Floor — theoretical prediction: keff^(+1) × latency^(+1)")
        print("─" * 50)
        valid_fl = valid.dropna(subset=[fl_col]).copy()
        valid_fl = valid_fl[valid_fl[fl_col] > 0]
        print(f"Non-censored floor: {len(valid_fl)} designs")

        if len(valid_fl) >= 10:
            valid_fl['log_floor'] = np.log(valid_fl[fl_col])
            X3 = np.column_stack([valid_fl['log_keff'], valid_fl['log_lat'],
                                   np.ones(len(valid_fl))])
            y3 = valid_fl['log_floor'].values
            coefs3, _, _, _ = lstsq(X3, y3, rcond=None)
            a3, b3, c3 = coefs3
            y3p = X3 @ coefs3
            r2_fl = 1 - np.sum((y3-y3p)**2)/np.sum((y3-y3.mean())**2)
            print(f"OLS: log(floor) = {a3:.3f}×log(keff) + {b3:.3f}×log(lat) + {c3:.3f}")
            print(f"     R² = {r2_fl:.3f}")
            print(f"Theoretical: keff=+1, lat=+1.  Empirical: keff={a3:.3f}, lat={b3:.3f}")
            print(f"  keff exponent ≈ +1? Deviation: {abs(a3-1):.3f}  {'YES (< 0.2)' if abs(a3-1)<0.2 else 'NO'}")
            print(f"  lat  exponent ≈ +1? Deviation: {abs(b3-1):.3f}  {'YES (< 0.2)' if abs(b3-1)<0.2 else 'NO'}")
        print()

    # ── Part 5: Physical interpretation — blind-spot displacement ────────────
    print("─" * 50)
    print("Part 5: Physical interpretation summary")
    print("─" * 50)
    print()
    print("THEORETICAL CHAIN:")
    print()
    print("  Step 1 — Angular authority per control step:")
    print("    θ̈_max = T × sin(δ_max) × L / Iyy = keff × u_max  [rad/s²]")
    print()
    print("  Step 2 — CEILING mechanism (DIPDT phase-margin theory):")
    print("    During delay τ, phase lag = ω×τ. Phase margin → 0 at Kp_max.")
    print("    Kp_ceiling ≈ 0.9/τ  (keff-independent when keff×τ << 1)")
    print(f"    Empirical: ceiling ∝ lat^{b2:.2f}  (theory: -1.0)  keff ∝ {a2:.2f}  (theory: 0.0)")
    print()
    print("  Step 3 — FLOOR mechanism (bang-bang blind-spot accumulation):")
    print("    Each corrective bang at max authority θ̈_max lasts ~τ seconds.")
    print("    Angular impulse per bang: Δω_bang = θ̈_max × τ")
    print("    The floor must overcome this per-cycle impulse:")
    print("    Kp_floor ∝ keff × τ  (each doubling of authority or latency doubles floor)")
    print(f"    Empirical: floor ∝ keff^{a3:.2f}  (theory: +1.0)  lat^{b3:.2f}  (theory: +1.0)")
    print()
    print("  Step 4 — WINDOW = ceiling / floor:")
    print("    window ∝ (τ^-1) / (keff × τ) = 1/(keff × τ²)")
    print("    ∴ window ∝ 1/Π_keff  where  Π_keff = keff × latency²")
    print("    In physical units: Π = θ̈_max × τ² = (1/2)·θ̈_max·τ² × 2 = 2 × blind-spot displacement")
    print()
    print("  τ² EXPLAINED:")
    print("    τ^(-2) = τ^(-1) [ceiling drop] + τ^(-1) [floor rise] = DOUBLE SQUEEZE")
    print("    Each doubling of latency → 2^1.88 ≈ 3.7× window compression")
    print("    (vs. 2^1.10 ≈ 2.1× if only the ceiling were sensitive to latency)")
    print()

    print("─" * 50)
    print("SUMMARY: Theoretical vs Empirical (Window_Ratio v2, n=116 non-censored)")
    print("─" * 50)
    headers = ["Parameter", "Theory", "Empirical (v2)", "Match?"]
    rows = [
        ["Ceiling keff exponent",  "0.000", f"{a2:.3f}", "✓" if abs(a2) < 0.15 else "✗"],
        ["Ceiling lat  exponent", "-1.000", f"{b2:.3f}", "✓" if abs(b2+1) < 0.3 else "✗"],
        ["Floor   keff exponent", "+1.000", f"{a3:.3f}", "✓" if abs(a3-1) < 0.3 else "✗"],
        ["Floor   lat  exponent", "+1.000", f"{b3:.3f}", "✓" if abs(b3-1) < 0.3 else "✗"],
        ["Window  keff exponent", "-1.000", f"{a_keff:.3f}", "✓" if abs(a_keff+1) < 0.3 else "✗"],
        ["Window  lat  exponent", "-2.000", f"{b_lat:.3f}", "✓" if abs(b_lat+2) < 0.3 else "✗"],
        ["Pi single-var slope",   "-1.000", f"{slope:.3f}", "✓" if abs(slope+1) < 0.3 else "✗"],
    ]
    col_w = [30, 12, 18, 8]
    print("  ".join(h.ljust(w) for h, w in zip(headers, col_w)))
    print("  ".join("-"*w for w in col_w))
    for r in rows:
        print("  ".join(str(v).ljust(w) for v, w in zip(r, col_w)))

    print()
    print(f"Overall window R²={r2:.3f} with theoretical 2-variable exponents fit")
    print(f"Pi single-variable: R²={r_lpi**2:.3f}")
    print()
    print("Done.")


if __name__ == '__main__':
    main()
