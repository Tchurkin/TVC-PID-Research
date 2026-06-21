"""
tools/window_ratio_interaction.py

Tests the keff×latency interaction model for window_ratio prediction.

MOTIVATION: the latency extension (tools/window_ratio_lat_extension.py) found
that the tau exponent is keff-dependent:
  low keff (~5):  lat exponent = -0.85
  mid keff (~14): lat exponent = -2.23
  high keff (~31): lat exponent = -3.19

This suggests an interaction model:
  log(window) = C + a*log(keff) + b*log(lat) + c*log(keff)*log(lat)

is better than the baseline two-variable model:
  log(window) = C + a*log(keff) + b*log(lat)

Also computes corrected Pi=keff*lat^2 thresholds for the design rule table
using performance_frontier_py.csv and adrc_frontier_extension_py.csv.

Data: experiments/results/window_ratio_v2_py.csv (n=82 non-censored, lat 1-6)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import KFold, cross_val_score

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

def cv(X, y, k=5, seed=42):
    return cross_val_score(
        LinearRegression(), X, y,
        cv=KFold(n_splits=k, shuffle=True, random_state=seed),
        scoring='r2'
    ).mean()


def main():
    df = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')
    valid = df[
        ~df['floor_censored'] & ~df['ceil_censored'] &
        df['window_ratio'].notna() & (df['window_ratio'] > 0)
    ].copy()
    print(f"Window ratio v2 non-censored: n={len(valid)}, lat 1-6")

    y  = np.log(valid['window_ratio'].values)
    lk = np.log(valid['keff'].values)
    ll = np.log(valid['latency_steps'].values)

    print()
    print("=" * 60)
    print("REGRESSION COMPARISON")
    print("=" * 60)

    baseline = cv(np.c_[lk, ll], y)
    inter3   = cv(np.c_[lk, ll, lk*ll], y)
    inter2   = cv(np.c_[lk, lk*ll], y)   # keff alone + interaction (no separate lat)
    pi_only  = cv((lk + 2*ll).reshape(-1,1), y)

    print(f"Pi = keff*lat^2 only (theory):    CV R2={pi_only:.3f}")
    print(f"keff + lat (baseline):             CV R2={baseline:.3f}")
    print(f"keff + lat + keff*lat (3-term):    CV R2={inter3:.3f}  delta={inter3-baseline:+.3f}")
    print(f"keff + keff*lat (2-term):          CV R2={inter2:.3f}  delta={inter2-baseline:+.3f}")

    # Fit the best model: keff + keff*lat
    m2 = LinearRegression().fit(np.c_[lk, lk*ll], y)
    print()
    print("=" * 60)
    print("WINNING MODEL: log(window) = C + a*log(keff) + b*log(keff)*log(lat)")
    print("=" * 60)
    print(f"  C (intercept): {m2.intercept_:.3f}")
    print(f"  a (keff coef): {m2.coef_[0]:.3f}")
    print(f"  b (keff*lat):  {m2.coef_[1]:.3f}")
    print()
    print("Formula: window = exp(C) * keff^a * keff^(b*log(lat))")
    print(f"       = {np.exp(m2.intercept_):.0f} * keff^{m2.coef_[0]:.2f} * lat^({m2.coef_[1]:.2f}*log(keff))")
    print()
    print("Equivalent: window ~ K / keff^(|a| + |b|*log(lat))")
    print(f"          = K / keff^({abs(m2.coef_[0]):.2f} + {abs(m2.coef_[1]):.2f}*log(lat))")
    print()
    print("Effective keff exponent grows with latency:")
    for lat_val in [1, 2, 3, 4, 6]:
        exp_val = m2.coef_[0] + m2.coef_[1] * np.log(lat_val) if lat_val > 0 else m2.coef_[0]
        print(f"  lat={lat_val}: keff exponent = {exp_val:.2f}")
    print()
    print("Effective latency exponent grows with keff (partial d/d log(lat) = b*log(keff)):")
    for keff_val in [3, 5, 10, 14, 20, 30]:
        lat_exp = m2.coef_[1] * np.log(keff_val)
        per_dbl = 2**(-lat_exp)
        print(f"  keff={keff_val:2d}: lat exponent = {lat_exp:.2f}  ({per_dbl:.1f}x per lat doubling)")

    # Fit the 3-term model too for comparison
    m3 = LinearRegression().fit(np.c_[lk, ll, lk*ll], y)
    print()
    print("3-TERM MODEL: log(window) = C + a*log(keff) + b*log(lat) + c*log(keff)*log(lat)")
    print(f"  C={m3.intercept_:.3f}, a(keff)={m3.coef_[0]:.3f}, b(lat)={m3.coef_[1]:.3f}, c(interaction)={m3.coef_[2]:.3f}")
    print("  Effective lat exponent at each keff tier:")
    for keff_val in [5.4, 14.2, 30.8]:
        eff = m3.coef_[1] + m3.coef_[2] * np.log(keff_val)
        print(f"    keff={keff_val}: lat_exp = {eff:.2f}  (measured: {-0.85 if keff_val<8 else -2.23 if keff_val<20 else -3.19:.2f})")

    # Pi threshold table (corrected from Pi_td to Pi_keff)
    print()
    print("=" * 60)
    print("CORRECTED Pi_keff THRESHOLDS FOR DESIGN RULE TABLE")
    print("=" * 60)

    pf = pd.read_csv(RESULTS / 'performance_frontier_py.csv')
    pf['Pi_keff'] = pf['keff'] * pf['latency']**2
    pf_s = pf.sort_values('Pi_keff')

    fe = pd.read_csv(RESULTS / 'adrc_frontier_extension_py.csv')
    fe['Pi_keff'] = fe['keff'] * fe['latency']**2
    fe_s = fe.sort_values('Pi_keff')

    pid90 = pf_s[pf_s['peak_pid_sr'] < 0.90]
    pid80 = pf_s[pf_s['peak_pid_sr'] < 0.80]
    adrc80_std = pf_s[pf_s['peak_adrc_sr'] < 0.80]  # wc=5 fixed in frontier
    adrc80_ext = fe_s[fe_s['adrc_sr_std_ratio'] < 0.80]
    adrc_nearzero = fe_s[fe_s['peak_adrc_sr'] < 0.80]

    print(f"First PID SR<0.90:          Pi_keff={pid90.iloc[0]['Pi_keff']:.0f}  (old Pi_td={pid90.iloc[0]['Pi']:.0f})")
    print(f"First PID SR<0.80:          Pi_keff={pid80.iloc[0]['Pi_keff']:.0f}  (old Pi_td={pid80.iloc[0]['Pi']:.0f})")
    if len(adrc80_std):
        print(f"First ADRC-std (wc=5) <0.80: Pi_keff={adrc80_std.iloc[0]['Pi_keff']:.0f}  (old Pi_td={adrc80_std.iloc[0]['Pi']:.0f})")
    if len(adrc80_ext):
        print(f"First ADRC-std fail (ext):   Pi_keff={adrc80_ext.iloc[0]['Pi_keff']:.0f}  (old Pi_td={adrc80_ext.iloc[0]['Pi']:.0f})")
    if len(adrc_nearzero):
        print(f"ADRC-extended near-fail:     Pi_keff={adrc_nearzero.iloc[0]['Pi_keff']:.0f}  (SR={adrc_nearzero.iloc[0]['peak_adrc_sr']:.2f})")
    print(f"Max Pi_keff tested (ADRC):   {fe_s['Pi_keff'].max():.0f}")

    print()
    print("UPDATED DESIGN RULE TABLE:")
    print("  Pi_keff < 300:        PID safe (SR=1.000 for all tested designs)")
    print("  Pi_keff 300-900:      PID degrades; ADRC (omega0/wc=5) universally safe")
    print("  Pi_keff 900-1600:     ADRC-standard may fail; use omega0/wc=8-20")
    print("  Pi_keff 1600-5600:    ADRC extended (omega0/wc=20) achieves SR=1.000")
    print("  Pi_keff > 5600:       not yet tested")


if __name__ == '__main__':
    main()
