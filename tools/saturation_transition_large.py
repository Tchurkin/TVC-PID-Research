"""
tools/saturation_transition_large.py  (2026-06-24)

PURPOSE
-------
Close reviewer attack-sheet objection #2: "Why is the headline fsat-vs-Pi figure n=29
when the project has 262- and 2400-design datasets?"

This re-runs the EXACT saturation_regime_map probe protocol (Kp_probe = 190/lat, best
constrained Kd, 20 eval seeds 91001-91020, fixed wind 0.25, full physics) on a MUCH
larger stratified sample drawn from the same final population. Same machinery (imported
from saturation_regime_map.process), same seeds -> the original 29 designs reproduce
exactly and the new designs are the independent test of whether rho(log Pi, fsat) holds.

DESIGN SELECTION
----------------
~30 designs per Pi bin for [30,100), [100,200), [200,400), [400,800) (random_state=7),
plus all available designs in [800,1500). The transition ONSET (~Pi 275) sits in the
well-populated [200,400) bin (114 available), so the region that matters scientifically
is heavily sampled. The high-Pi saturated tail (>800) is capped by population availability
(only 5 designs exist) -- but that is the already-clearly-saturated end, not the onset.

OUTPUT: experiments/results/saturation_transition_large_py.csv
Then compares rho(log Pi, fsat), onset, and regime structure to the n=29 baseline.
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats

# Import the EXACT probe protocol used for the n=29 map (guarantees identical method)
from saturation_regime_map import process, _keff, PI_BINS

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

N_PER_BIN  = 30
RNG_STATE  = 7


def select_designs_large(pop):
    pop = pop.copy()
    pop['keff_c']  = pop.apply(_keff, axis=1)
    pop['Pi_keff'] = pop['keff_c'] * pop['latency_steps'] ** 2

    selected = []
    for lo, hi in PI_BINS:
        sub = pop[(pop['Pi_keff'] >= lo) & (pop['Pi_keff'] < hi)].copy()
        if len(sub) == 0:
            print(f"  Pi [{lo:5d},{hi:5d}): NO DESIGNS")
            continue
        n = min(N_PER_BIN, len(sub))
        chosen = sub.sample(n=n, random_state=RNG_STATE) if len(sub) > n else sub
        selected.append(chosen)
        lat_vals = sorted(chosen['latency_steps'].unique())
        print(f"  Pi [{lo:5d},{hi:5d}): {len(sub):4d} avail -> {len(chosen):3d} chosen "
              f"(lat={lat_vals}, Pi={chosen['Pi_keff'].min():.0f}-{chosen['Pi_keff'].max():.0f})")

    return pd.concat(selected).reset_index(drop=True)


def main():
    print("saturation_transition_large.py  (larger-sample replication of the n=29 map)")
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')
    print(f"Loaded population: {len(pop)} designs\n")

    designs = select_designs_large(pop)
    n = len(designs)
    print(f"\nTotal: {n} designs (vs n=29 baseline)")
    n_sims = n * (3 * 5 + 20 * 2)
    print(f"Estimated sims: ~{n_sims:,}\nRunning in parallel...\n")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(process)(row.to_dict()) for _, row in designs.iterrows()
    )
    df = pd.DataFrame(results).sort_values('Pi').reset_index(drop=True)
    out = RESULTS / 'saturation_transition_large_py.csv'
    df.to_csv(out, index=False)
    print(f"\nSaved {len(df)} rows -> {out.name}")

    analyze(df)


def analyze(df=None):
    if df is None:
        df = pd.read_csv(RESULTS / 'saturation_transition_large_py.csv')

    base = pd.read_csv(RESULTS / 'saturation_regime_map_py.csv')

    print("\n" + "=" * 72)
    print(f"LARGER SATURATION-TRANSITION POPULATION  (n={len(df)})  vs baseline (n={len(base)})")
    print("=" * 72)

    def rho(d):
        return stats.spearmanr(np.log(d['Pi'].clip(1)), d['fsat'])

    r_new, p_new = rho(df)
    r_base, p_base = rho(base)
    print(f"\nrho(log Pi, fsat):")
    print(f"  baseline n={len(base):3d}:  rho = {r_base:+.3f}  p = {p_base:.2e}")
    print(f"  larger   n={len(df):3d}:  rho = {r_new:+.3f}  p = {p_new:.2e}")

    # keff-alone and lat-alone for comparison (the 'product collapses it' claim)
    rk, _ = stats.spearmanr(np.log(df['keff']), df['fsat'])
    rl, _ = stats.spearmanr(np.log(df['lat']),  df['fsat'])
    print(f"  larger rho(log keff, fsat) = {rk:+.3f}   rho(log lat, fsat) = {rl:+.3f}")
    print(f"  -> product still beats either factor alone: {'YES' if r_new < min(rk, rl) - 0.02 else 'CHECK'}")

    # Binned means
    bins  = [0, 100, 200, 400, 800, 1500, 7000]
    blabs = ['<100', '100-200', '200-400', '400-800', '800-1500', '1500+']
    df['Pi_bin'] = pd.cut(df['Pi'], bins=bins, labels=blabs)
    tab = df.groupby('Pi_bin', observed=True).agg(
        n=('Pi', 'count'), fsat=('fsat', 'mean'),
        sr_sat=('sr_sat', 'mean'), sr_nosat=('sr_nosat', 'mean')).round(3)
    print("\nBinned (larger sample):")
    print(tab.to_string())

    # Onset (first fsat>=0.35) and transition-region scatter
    ds = df.sort_values('Pi')
    above = ds[ds['fsat'] >= 0.35]
    if len(above):
        print(f"\nsaturation onset (fsat>=0.35): first at Pi = {above.iloc[0]['Pi']:.0f} "
              f"(keff={above.iloc[0]['keff']:.1f}, lat={above.iloc[0]['lat']})")
    # honest band check: range of Pi where linear & saturation overlap
    lin_hi = ds[ds['regime'] == 'linear']['Pi'].max()
    sat_lo = ds[ds['regime'] == 'saturation']['Pi'].min()
    print(f"highest linear-regime Pi = {lin_hi:.0f}; lowest saturation-regime Pi = {sat_lo:.0f} "
          f"(overlap band = real scatter, expected)")

    print("\nRegime counts (larger):", df['regime'].value_counts().to_dict())

    # Causal test still holds?
    print(f"\nCausal test  SR_nosat: mean={df['sr_nosat'].mean():.3f}  "
          f"min={df['sr_nosat'].min():.3f}  (should stay high)")
    print(f"  designs SR_nosat<0.90: {(df['sr_nosat']<0.90).sum()}/{len(df)}")

    print("\nDone.")


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        analyze()
    else:
        main()
