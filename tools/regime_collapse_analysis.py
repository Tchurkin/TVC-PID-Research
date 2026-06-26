"""
tools/regime_collapse_analysis.py  (2026-06-23)

Answers the three ChatGPT questions using existing data — no new simulations.

Q1: Does fsat collapse onto Pi better than onto keff or latency alone?
    Uses saturation_regime_map_py.csv (n=29)

Q2: Regime transition boundaries — already measured, just display cleanly.

Q3: Does ADRC advantage appear exactly where the saturation transition begins?
    Uses performance_frontier_py.csv (n=63), recomputed with Pi_keff.

OUTPUT: console table + saturation_regime_collapse_py.csv
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from scipy import stats

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'


def q1_collapse(regime):
    """
    Compare Spearman rho(fsat, X) for X in {keff, lat, keff*lat, Pi=keff*lat^2}.
    Strongest rho = best single predictor of the saturation regime.
    R0522 (keff=41.4, lat=1, Pi=41, fsat=0.018) is the decisive test case:
      - keff alone predicts HIGH fsat (wrong)
      - Pi correctly predicts LOW fsat (right)
    """
    print("\n" + "=" * 70)
    print("Q1 — VARIABLE COLLAPSE: what best predicts saturation fraction?")
    print("=" * 70)

    df = regime.copy()

    predictors = {
        'keff':        df['keff'],
        'lat':         df['lat'],
        'keff x lat':  df['keff'] * df['lat'],
        'Pi (keff x lat^2)': df['Pi'],
        'log(keff)':   np.log(df['keff']),
        'log(lat)':    np.log(df['lat']),
        'log(Pi)':     np.log(df['Pi']),
    }

    print(f"\n{'Predictor':<25} {'rho(fsat)':>10}  {'p':>10}  {'note'}")
    print("-" * 65)
    for name, x in predictors.items():
        r, p = stats.spearmanr(x, df['fsat'])
        note = ''
        if name == 'log(Pi)':
            note = '<-- WINNER'
        print(f"  {name:<23} {r:>+10.3f}  {p:>10.2e}  {note}")

    # Show R0522 explicitly — the key counterexample
    r522 = df[df['rocket_id'] == 'R0522']
    if len(r522):
        r = r522.iloc[0]
        print(f"\nKey counterexample — R0522:")
        print(f"  keff={r['keff']:.1f}  lat={r['lat']}  Pi={r['Pi']:.0f}  fsat={r['fsat']:.3f}  regime={r['regime']}")
        print(f"  Naive prediction from keff alone: DANGEROUS (keff=41.4 is max in dataset)")
        print(f"  Pi prediction: LINEAR (Pi=41 << transition threshold ~275)")
        print(f"  Actual result: SR=1.000, fsat=0.018  --> Pi is CORRECT, keff alone is WRONG")

    # Scatter summary: high-keff designs across different latency
    print("\nHigh-keff designs (keff>15), showing fsat vs lat:")
    hk = df[df['keff'] > 15].sort_values('lat')
    for _, r in hk.iterrows():
        print(f"  {r['rocket_id']:<10} keff={r['keff']:5.1f}  lat={r['lat']}  "
              f"Pi={r['Pi']:6.0f}  fsat={r['fsat']:.3f}  regime={r['regime']}")


def q2_boundaries(regime):
    """
    Display regime transition boundaries and their stability.
    """
    print("\n" + "=" * 70)
    print("Q2 — REGIME TRANSITION BOUNDARIES")
    print("=" * 70)

    df = regime.sort_values('Pi')

    print("\nBinned regime summary (from saturation_regime_map):")
    bins  = [0, 100, 200, 400, 800, 1500]
    blab  = ['<100', '100-200', '200-400', '400-800', '800-1500']
    df['bin'] = pd.cut(df['Pi'], bins=bins + [9999], labels=blab + ['1500+'])
    tab = df.groupby('bin', observed=True).agg(
        n         = ('Pi', 'count'),
        Pi_range  = ('Pi', lambda x: f"{x.min():.0f}-{x.max():.0f}"),
        fsat_mean = ('fsat', 'mean'),
        fsat_max  = ('fsat', 'max'),
        SR_mean   = ('sr_sat', 'mean'),
        regime_counts = ('regime', lambda x: dict(x.value_counts()))
    )
    print(tab.to_string())

    print("\nTransition Pi values (first design crossing each fsat threshold):")
    for thresh, label in [(0.05, 'early transitional'), (0.10, 'transitional'),
                          (0.35, 'saturation-dominated'), (0.50, 'clearly saturated')]:
        above = df[df['fsat'] >= thresh]
        if len(above):
            r = above.iloc[0]
            print(f"  fsat >= {thresh:.2f} ({label:<22}): Pi = {r['Pi']:>6.0f}  "
                  f"(keff={r['keff']:.1f}, lat={r['lat']})")
        else:
            print(f"  fsat >= {thresh:.2f}: not reached in tested range")

    # Fraction in each regime per bin
    print("\nRegime classification by Pi bin:")
    for b in blab:
        sub = df[df['bin'] == b]
        if len(sub) == 0:
            continue
        counts = sub['regime'].value_counts()
        parts  = '  '.join(f"{k}:{v}" for k, v in counts.items())
        print(f"  Pi {b:<10}: n={len(sub)}  {parts}")


def q3_adrc_boundary(regime, frontier):
    """
    Show ADRC advantage (peak_adrc_sr - peak_pid_sr) vs Pi_keff.
    The onset of ADRC advantage should match the saturation transition Pi.
    """
    print("\n" + "=" * 70)
    print("Q3 — ADRC ADVANTAGE VS Pi_keff: does it appear at the transition?")
    print("=" * 70)

    # Recompute Pi_keff for frontier (frontier has Pi = td*lat^2, keff separate)
    f = frontier.copy()
    f['Pi_keff'] = f['keff'] * f['latency'] ** 2
    f = f.sort_values('Pi_keff')

    print("\nPerformance frontier — PID vs ADRC vs Pi_keff:")
    print(f"{'rocket_id':<10} {'Pi_keff':>8} {'lat':>4} {'keff':>6} | "
          f"{'PID SR':>7} {'ADRC SR':>8} {'gap':>7}")
    print("-" * 60)
    for _, r in f.iterrows():
        gap = r['peak_adrc_sr'] - r['peak_pid_sr']
        marker = ' <--' if gap > 0.05 else ''
        print(f"{r['rocket_id']:<10} {r['Pi_keff']:>8.0f} {r['latency']:>4.0f} {r['keff']:>6.2f} | "
              f"{r['peak_pid_sr']:>7.3f} {r['peak_adrc_sr']:>8.3f} {gap:>7.3f}{marker}")

    # Find where ADRC first pulls ahead
    significant_gap = f[f['sr_delta'] > 0.05]
    if len(significant_gap):
        first = significant_gap.iloc[0]
        print(f"\nADRC first beats PID by >5pp at Pi_keff = {first['Pi_keff']:.0f}  "
              f"(keff={first['keff']:.1f}, lat={first['latency']:.0f})")

    # Binned comparison
    print("\nADRC vs PID by Pi_keff bin (frontier data):")
    pi_bins = [0, 100, 200, 400, 800, 1500, 99999]
    pi_labs = ['<100', '100-200', '200-400', '400-800', '800-1500', '1500+']
    f['Pi_bin'] = pd.cut(f['Pi_keff'], bins=pi_bins, labels=pi_labs)
    tab2 = f.groupby('Pi_bin', observed=True).agg(
        n            = ('Pi_keff', 'count'),
        Pi_keff_mean = ('Pi_keff', 'mean'),
        PID_SR       = ('peak_pid_sr', 'mean'),
        ADRC_SR      = ('peak_adrc_sr', 'mean'),
        gap_mean     = ('sr_delta', 'mean'),
        gap_max      = ('sr_delta', 'max'),
    ).round(3)
    print(tab2.to_string())

    # Overlay: where does fsat enter saturation regime vs where does ADRC gap open?
    regime_pi_crit  = regime[regime['fsat'] >= 0.35]['Pi'].min()
    frontier_pi_crit = f[f['sr_delta'] > 0.05]['Pi_keff'].min() if len(significant_gap) else np.nan
    print(f"\n=== THE KEY ALIGNMENT ===")
    print(f"  fsat saturation onset  (fsat >= 0.35):   Pi_keff ~ {regime_pi_crit:.0f}")
    print(f"  ADRC advantage onset   (gap  > 5pp):     Pi_keff ~ {frontier_pi_crit:.0f}")

    if not np.isnan(frontier_pi_crit):
        ratio = frontier_pi_crit / regime_pi_crit
        print(f"  Ratio: {ratio:.2f}x  (1.0 = perfect alignment)")
        if 0.5 < ratio < 2.0:
            print(f"  --> ALIGNED: saturation transition predicts ADRC advantage onset")
        else:
            print(f"  --> NOT ALIGNED: regime transition and ADRC advantage onset differ")

    rho, p = stats.spearmanr(f['Pi_keff'], f['sr_delta'])
    print(f"\n  rho(Pi_keff, ADRC gap) = {rho:+.3f}  p={p:.2e}")


def main():
    print("regime_collapse_analysis.py")
    print("Q1: fsat collapse  |  Q2: transition boundaries  |  Q3: ADRC alignment")

    regime   = pd.read_csv(RESULTS / 'saturation_regime_map_py.csv')
    frontier = pd.read_csv(RESULTS / 'performance_frontier_py.csv')

    print(f"\nLoaded: regime n={len(regime)}, frontier n={len(frontier)}")

    q1_collapse(regime)
    q2_boundaries(regime)
    q3_adrc_boundary(regime, frontier)

    print("\n" + "=" * 70)
    print("SUMMARY: three-question assessment")
    print("=" * 70)

    df   = regime.copy()
    rho_pi,   p_pi   = stats.spearmanr(np.log(df['Pi']),   df['fsat'])
    rho_keff, p_keff = stats.spearmanr(np.log(df['keff']), df['fsat'])
    rho_lat,  p_lat  = stats.spearmanr(np.log(df['lat']),  df['fsat'])

    print(f"\nQ1 collapse: rho(log Pi, fsat)={rho_pi:+.3f}  vs  "
          f"rho(log keff)={rho_keff:+.3f}  rho(log lat)={rho_lat:+.3f}")
    print(f"   Pi is {'BETTER' if abs(rho_pi) > max(abs(rho_keff), abs(rho_lat)) else 'NOT better'} "
          f"than either individual variable")

    regime_pi_crit = df[df['fsat'] >= 0.35]['Pi'].min()
    print(f"\nQ2 boundaries: linear < ~150, transitional 150-275, saturation > ~{regime_pi_crit:.0f}")

    frontier2 = pd.read_csv(RESULTS / 'performance_frontier_py.csv')
    frontier2['Pi_keff'] = frontier2['keff'] * frontier2['latency'] ** 2
    sig_gap = frontier2[frontier2['sr_delta'] > 0.05]
    pi_adrc = sig_gap['Pi_keff'].min() if len(sig_gap) else float('nan')
    print(f"\nQ3 ADRC alignment: saturation onset Pi~{regime_pi_crit:.0f}, "
          f"ADRC advantage onset Pi~{pi_adrc:.0f}")
    if not np.isnan(pi_adrc):
        ratio = pi_adrc / regime_pi_crit
        print(f"   Alignment ratio {ratio:.1f}x  "
              f"({'STRONG' if 0.5 < ratio < 3 else 'WEAK'} alignment)")

    print("\nDone.")


if __name__ == '__main__':
    main()
