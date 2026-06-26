"""
tools/quad_generalization_extended.py  (2026-06-23)

EXTENDED QUADROTOR GENERALIZATION — stratified Pi sampling.

PROBLEM WITH ORIGINAL (generalization_study.py, n=25 quad):
  Only n=2 designs above Pi=3000 where SR degradation appears.
  LHS with small parameter ranges (F_max 0.05-1.5 N) generates mostly low-Pi designs.
  rho=-0.471 is marginal (p=0.018) due to sparse high-Pi tail.

FIX:
  (1) Use realistic quad hardware ranges (F_max 2-20 N, as in CLAUDE.md spec).
  (2) Stratified Pi sampling: 5 Pi tiers, 10 designs each = 50 total.
      Tier boundaries chosen to bracket where TVC shows degradation,
      scaled up by ~10x to account for quad Pi units being larger.
  (3) Same stripped physics as original: Euler PD + FIFO delay + OU wind.

Pi TIERS (5 × 10 = 50 designs):
  Tier 1: Pi <   1,000   (confident safe zone)
  Tier 2: Pi   1,000 –   5,000
  Tier 3: Pi   5,000 –  20,000
  Tier 4: Pi  20,000 –  80,000
  Tier 5: Pi > 80,000    (extreme authority / long delay)

PHYSICS: Same as generalization_study.py — stripped Euler integrator,
  wind d ∝ keff (aerodynamic coupling, same structure as TVC).
  Kd heuristic: KD_FRAC=0.003/lat (normalized, unchanged from original).
  12 log-spaced Kp points per design (normalized loop-gain sweep).
  15 eval seeds.

FILES OUT:
  experiments/results/quad_gen_extended_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from scipy import stats
from pathlib import Path
from joblib import Parallel, delayed as jdelayed

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

DT      = 0.005
T_END   = 3.0
N_STEPS = int(T_END / DT)
MAX_ANG = np.deg2rad(30.0)

LOOP_GAIN_MIN =    100.0
LOOP_GAIN_MAX = 100000.0   # extended range for extreme-Pi designs
N_KP          = 16         # more Kp points than original (was 12)
KD_FRAC       = 0.003      # same heuristic as original

N_PER_TIER = 10
PI_TIERS   = [(0, 1000), (1000, 5000), (5000, 20000), (20000, 80000), (80000, 1e9)]

N_EVAL   = 15
EVAL_SEEDS = list(range(105001, 105016))


def ou_series(n_steps, tau_ou, sigma, rng):
    alpha = DT / tau_ou
    noise = rng.normal(0, sigma * np.sqrt(2 * alpha), n_steps)
    d = np.zeros(n_steps)
    for i in range(1, n_steps):
        d[i] = d[i - 1] * (1 - alpha) + noise[i]
    return d


def simulate_quad(keff, lat, Kp, Kd, d_sigma, seed):
    rng   = np.random.default_rng(seed)
    dist  = ou_series(N_STEPS, 0.30, d_sigma, rng)
    th0   = float(rng.normal(0, np.deg2rad(2)))
    theta, thetadot = th0, 0.0
    buf_th  = np.zeros(lat + 1)
    buf_thd = np.zeros(lat + 1)
    max_th  = abs(th0)
    for k in range(N_STEPS):
        u = np.clip(-Kp * buf_th[-1] - Kd * buf_thd[-1], -1.0, 1.0)
        thdd = keff * u + dist[k]
        thetadot += thdd * DT
        theta    += thetadot * DT
        if abs(theta) > max_th:
            max_th = abs(theta)
        buf_th  = np.roll(buf_th,  1); buf_th[0]  = theta
        buf_thd = np.roll(buf_thd, 1); buf_thd[0] = thetadot
    return max_th < MAX_ANG


def process_one(design):
    keff     = design['keff']
    lat      = design['lat']
    d_sigma  = design['d_sigma']
    Kd       = KD_FRAC / lat

    loop_gains = np.logspace(np.log10(LOOP_GAIN_MIN),
                              np.log10(LOOP_GAIN_MAX), N_KP)
    Kp_list = loop_gains / (keff * lat)

    best_sr, best_Kp = 0.0, Kp_list[0]
    for Kp in Kp_list:
        srs = [simulate_quad(keff, lat, Kp, Kd, d_sigma, s) for s in EVAL_SEEDS]
        sr  = float(np.mean(srs))
        if sr > best_sr:
            best_sr, best_Kp = sr, Kp

    return dict(
        system   = 'quad_extended',
        lat      = lat,
        keff     = round(keff, 4),
        Pi       = round(design['Pi'], 2),
        d_sigma  = round(d_sigma, 4),
        best_Kp  = round(best_Kp, 4),
        best_sr  = round(best_sr, 4),
    )


def sample_tier(pi_lo, pi_hi, n, master_rng, max_tries=50000):
    """Rejection-sample quad designs within a Pi tier."""
    designs = []
    tried   = 0
    while len(designs) < n and tried < max_tries:
        tried += 1
        arm   = master_rng.uniform(0.05, 0.25)
        F_max = master_rng.uniform(2.0, 20.0)
        Ixx   = master_rng.uniform(0.001, 0.020)
        lat   = int(master_rng.choice([1, 2, 3, 4, 5, 6]))
        gust  = master_rng.uniform(0.10, 0.50)
        keff  = F_max * arm / Ixx
        Pi    = keff * lat ** 2
        if pi_lo <= Pi < pi_hi:
            designs.append(dict(
                lat=lat, keff=keff, Pi=Pi,
                d_sigma=gust * keff * 0.08,
            ))
    if len(designs) < n:
        print(f'  WARNING: only {len(designs)}/{n} for Pi [{pi_lo}, {pi_hi})')
    return designs


if __name__ == '__main__':
    master_rng = np.random.default_rng(106001)

    all_designs = []
    for lo, hi in PI_TIERS:
        tier_designs = sample_tier(lo, hi, N_PER_TIER, master_rng)
        print(f'  Tier Pi [{lo:.0f}, {hi:.0f}): {len(tier_designs)} designs sampled  '
              f'Pi range [{min(d["Pi"] for d in tier_designs):.0f}, '
              f'{max(d["Pi"] for d in tier_designs):.0f}]')
        all_designs.extend(tier_designs)

    print(f'\nTotal: {len(all_designs)} designs, {N_KP} Kp points, {N_EVAL} seeds each')
    print(f'Simulations: {len(all_designs) * N_KP * N_EVAL:,}')

    raw = Parallel(n_jobs=-1, verbose=5)(
        jdelayed(process_one)(d) for d in all_designs
    )
    df = pd.DataFrame(raw).sort_values('Pi').reset_index(drop=True)

    out = RESULTS / 'quad_gen_extended_py.csv'
    df.to_csv(out, index=False)
    print(f'\nSaved {len(df)} rows -> {out}')

    print('\n' + '='*60)
    print('QUAD GENERALIZATION EXTENDED — RESULTS')
    print('='*60)

    rho, p = stats.spearmanr(np.log(df['Pi']), df['best_sr'])
    r_p, p_p = stats.pearsonr(np.log(df['Pi']), df['best_sr'])
    print(f'\nSpearman rho(log Pi, best_SR) = {rho:.3f}  p={p:.2e}')
    print(f'Pearson  r  (log Pi, best_SR) = {r_p:.3f}  p={p_p:.2e}')
    print(f'n = {len(df)}')

    print('\nBinned means by Pi tier:')
    for lo, hi in PI_TIERS:
        sub = df[(df['Pi'] >= lo) & (df['Pi'] < hi)]
        if len(sub) > 0:
            print(f'  Pi {lo:>7,.0f} – {hi:>8,.0f}: '
                  f'mean SR = {sub["best_sr"].mean():.3f}  '
                  f'min = {sub["best_sr"].min():.3f}  '
                  f'n = {len(sub)}')

    print(f'\nComparison:')
    print(f'  Original quad (n=25):  rho = -0.471  p = 1.76e-02')
    print(f'  Extended quad (n={len(df)}): rho = {rho:.3f}  p = {p:.2e}')
    print(f'  TVC reference:         rho = -0.668  p = 2.15e-09')
