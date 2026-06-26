"""
tools/saturation_regime_map.py  (2026-06-22)

PURPOSE
-------
Map the Pi_keff -> saturation regime transition at a FIXED, PRINCIPLED operating point:

    Kp_probe = 190 / latency_steps   (= 0.5 x DIPDT ceiling 380/lat)

For every design, this probe is BELOW the linear-theory ceiling. The only way the
servo saturates at this gain is if the saturation-dominated ceiling has dropped below
Kp_probe -- i.e., the design is in the nonlinear regime.

Measuring fsat (servo saturation fraction) at Kp_probe across a Pi_keff sweep produces
a direct, artifact-free picture of when the control regime transitions from:
  linear DIPDT -> transitional -> saturation-dominated

3 conditions per design:
  A. Full physics, Kp_probe, best constrained Kd:  fsat + SR   [regime mapping]
  B. No saturation (slew=False), same Kp, same Kd: SR only     [causal test]

If saturation is the SOLE cause of SR degradation at high Pi:
  B should give SR_nosat ~1.0 for ALL designs even where SR_sat < 0.5.
  This extends Finding 8 (PID-nosat=1.000, n=15) to a much larger population (n~36).

The gap = SR_nosat - SR_sat is the saturation penalty: how much SR you lose purely from
servo saturation, holding everything else (gains, wind, latency, noise) constant.

DESIGN SELECTION
----------------
36 designs from exp1_final_population_py.csv, stratified by Pi_keff into 6 bins:
  [30-100], [100-200], [200-400], [400-800], [800-1500], [1500+]
6 designs per bin; latency spread preferred within each bin.

WIND: Fixed at 0.25 for all designs (removes wind as a confound on fsat).

SEEDS: eval 91001-91020 (n=20, sigma_p~0.07); Kd search 91021-91023. All disjoint.
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario)
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD  = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE   = 0.25

FIXED_WIND   = 0.25
N_EVAL       = 20
EVAL_SEED0   = 91001
SEARCH_SEEDS = [91021, 91022, 91023]

KD_RATIOS = np.array([0.10, 0.32, 1.00, 3.16])
KD_ZN     = 0.57
KD_CLIP   = (0.01, 32.0)

PI_BINS    = [(30, 100), (100, 200), (200, 400), (400, 800), (800, 1500), (1500, 6000)]
N_PER_BIN  = 6

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'


# ── helpers ──────────────────────────────────────────────────────────────────

def _keff(row):
    return F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']


def _build(row, slew_on):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    dis.gust_std = FIXED_WIND          # fix wind; override LHS value
    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=slew_on, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fid)
    return plant, act, sen, dis, sc


def _run_seeds(row, Kp, Kd, seeds, slew_on):
    """Return (success_rate, mean_slew_sat_frac)."""
    plant, act, sen, dis, sc = _build(row, slew_on)
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    sr   = float(np.mean([r.success for r in results]))
    fsat = float(np.mean([r.slew_sat_frac for r in results]))
    return sr, fsat


def _best_kd(row, Kp_probe):
    """Constrained Kd grid search at Kp_probe with slew ON (3 fast seeds)."""
    kds = np.unique(np.clip(
        list(KD_RATIOS * Kp_probe) + [KD_ZN], KD_CLIP[0], KD_CLIP[1]))
    best_sr, best_kd = -1.0, KD_ZN
    for kd in kds:
        sr, _ = _run_seeds(row, Kp_probe, kd, SEARCH_SEEDS, slew_on=True)
        if sr > best_sr:
            best_sr, best_kd = sr, kd
    return float(best_kd), float(best_sr)


# ── per-design worker ─────────────────────────────────────────────────────────

def process(row_dict):
    keff = _keff(row_dict)
    lat  = int(row_dict['latency_steps'])
    Pi   = keff * lat ** 2                   # Pi_keff (not Pi_td)
    rid  = row_dict.get('rocket_id', 'UNK')
    lbl  = row_dict.get('final_label', 'UNK')

    # Fixed probe: exactly half the DIPDT ceiling, keff-independent
    Kp_probe        = 190.0 / lat
    ceiling_theory  = 380.0 / lat           # linear DIPDT ceiling

    # Kd search at probe point
    best_kd, search_sr = _best_kd(row_dict, Kp_probe)

    # Condition A: full physics
    eval_seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr_sat, fsat = _run_seeds(row_dict, Kp_probe, best_kd, eval_seeds, slew_on=True)

    # Condition B: saturation removed (causal test)
    sr_nosat, _ = _run_seeds(row_dict, Kp_probe, best_kd, eval_seeds, slew_on=False)

    sat_penalty = sr_nosat - sr_sat    # > 0 means saturation is hurting SR
    frac_explained = sat_penalty / (1.0 - sr_sat + 1e-9) if sr_sat < 0.999 else 0.0

    # Regime classification from fsat
    if fsat < 0.10:
        regime = 'linear'
    elif fsat < 0.35:
        regime = 'transitional'
    else:
        regime = 'saturation'

    return dict(
        rocket_id        = rid,
        label            = lbl,
        keff             = round(keff, 3),
        lat              = lat,
        Pi               = round(Pi, 1),         # Pi_keff
        Kp_probe         = round(Kp_probe, 2),
        ceiling_theory   = round(ceiling_theory, 1),
        best_kd          = round(best_kd, 3),
        search_sr        = round(search_sr, 3),
        sr_sat           = round(sr_sat, 4),
        fsat             = round(fsat, 4),
        sr_nosat         = round(sr_nosat, 4),
        sat_penalty      = round(sat_penalty, 4),
        frac_explained   = round(frac_explained, 3),
        regime           = regime,
    )


# ── design selection ──────────────────────────────────────────────────────────

def select_designs(pop):
    pop = pop.copy()
    pop['keff_c']   = pop.apply(_keff, axis=1)
    pop['Pi_keff']  = pop['keff_c'] * pop['latency_steps'] ** 2

    selected = []
    for lo, hi in PI_BINS:
        sub = pop[(pop['Pi_keff'] >= lo) & (pop['Pi_keff'] < hi)].copy()
        n   = min(N_PER_BIN, len(sub))
        if n == 0:
            print(f"  Pi [{lo:5d},{hi:5d}): NO DESIGNS - skipping")
            continue
        # Prefer latency variety: sort by latency, stride-select
        sub_sorted = sub.sort_values('latency_steps').reset_index(drop=True)
        stride  = max(1, len(sub_sorted) // n)
        chosen  = sub_sorted.iloc[::stride].head(n)
        selected.append(chosen)
        lat_vals = sorted(chosen['latency_steps'].unique())
        print(f"  Pi [{lo:5d},{hi:5d}): {len(sub):5d} candidates -> {len(chosen)} chosen "
              f"(lat={lat_vals}, Pi={chosen['Pi_keff'].min():.0f}-{chosen['Pi_keff'].max():.0f})")

    df = pd.concat(selected).reset_index(drop=True)
    return df


# ── analysis ──────────────────────────────────────────────────────────────────

def analyze(df=None):
    if df is None:
        df = pd.read_csv(RESULTS / 'saturation_regime_map_py.csv')

    print("\n" + "=" * 80)
    print("SATURATION REGIME MAP - RESULTS")
    print(f"Probe: Kp = 190/lat (0.50 x DIPDT ceiling)  |  wind=0.25  |  n={len(df)}")
    print("=" * 80)

    # Per-design table
    print(f"\n{'ID':<12} {'Pi':>7} {'lat':>4} {'keff':>6} | "
          f"{'SR_sat':>7} {'fsat':>7} | {'SR_nosat':>8} {'penalty':>8} | regime")
    print("-" * 80)
    for _, r in df.sort_values('Pi').iterrows():
        print(f"{r.rocket_id:<12} {r.Pi:>7.0f} {r.lat:>4} {r.keff:>6.1f} | "
              f"{r.sr_sat:>7.3f} {r.fsat:>7.3f} | {r.sr_nosat:>8.3f} {r.sat_penalty:>8.3f} | {r.regime}")

    # ── binned summary ────────────────────────────────────────────────────────
    print("\n=== Binned regime summary ===")
    bins   = [0, 100, 200, 400, 800, 1500, 7000]
    blabs  = ['<100', '100-200', '200-400', '400-800', '800-1500', '1500+']
    df['Pi_bin'] = pd.cut(df['Pi'], bins=bins, labels=blabs)
    tab = df.groupby('Pi_bin', observed=True).agg(
        n        = ('Pi',       'count'),
        Pi_mean  = ('Pi',       'mean'),
        sr_sat   = ('sr_sat',   'mean'),
        fsat     = ('fsat',     'mean'),
        sr_nosat = ('sr_nosat', 'mean'),
        penalty  = ('sat_penalty', 'mean'),
    ).round(3)
    print(tab.to_string())

    # ── regime transition ────────────────────────────────────────────────────
    print("\n=== Regime transition (fsat thresholds) ===")
    df_s = df.sort_values('Pi')
    for thresh, label in [(0.10, 'transitional onset'), (0.35, 'saturation onset')]:
        above = df_s[df_s['fsat'] >= thresh]
        if len(above):
            print(f"  fsat >= {thresh:.2f} ({label}): first at Pi = {above.iloc[0]['Pi']:.0f}  "
                  f"(keff={above.iloc[0]['keff']:.1f}, lat={above.iloc[0]['lat']})")
        else:
            print(f"  fsat never reached {thresh:.2f} in tested range")

    print("\n=== Regime counts ===")
    print(df['regime'].value_counts().to_string())

    # ── causal test (condition B) ────────────────────────────────────────────
    print("\n=== Causal test: SR_nosat at Kp_probe (should be ~1.0 if saturation is causal) ===")
    print(f"  All designs — SR_nosat: mean={df['sr_nosat'].mean():.3f}  "
          f"min={df['sr_nosat'].min():.3f}  "
          f"median={df['sr_nosat'].median():.3f}")
    print(f"  Designs with SR_nosat < 0.90: {(df['sr_nosat'] < 0.90).sum()} / {len(df)}")

    hi = df[df['Pi'] > 600]
    if len(hi):
        print(f"\n  High-Pi (Pi>600, n={len(hi)}):")
        print(f"    SR_sat   mean = {hi['sr_sat'].mean():.3f}")
        print(f"    SR_nosat mean = {hi['sr_nosat'].mean():.3f}")
        print(f"    fsat     mean = {hi['fsat'].mean():.3f}")
        print(f"    penalty  mean = {hi['sat_penalty'].mean():.3f}")

    # ── correlations ────────────────────────────────────────────────────────
    print("\n=== Spearman correlations with log(Pi_keff) ===")
    log_pi = np.log(df['Pi'].clip(1))
    for col, lbl in [('fsat',        'fsat              '),
                      ('sr_sat',      'SR_sat             '),
                      ('sat_penalty', 'sat_penalty        '),
                      ('sr_nosat',    'SR_nosat           ')]:
        r, p = stats.spearmanr(log_pi, df[col])
        print(f"  rho(log Pi, {lbl}) = {r:+.3f}  p={p:.2e}")

    # ── does fsat predict the penalty better than Pi? ────────────────────────
    r_pi, p_pi = stats.spearmanr(df['Pi'],   df['sat_penalty'])
    r_fs, p_fs = stats.spearmanr(df['fsat'], df['sat_penalty'])
    print(f"\n=== What predicts the saturation penalty? ===")
    print(f"  rho(Pi,   sat_penalty) = {r_pi:+.3f}  p={p_pi:.2e}")
    print(f"  rho(fsat, sat_penalty) = {r_fs:+.3f}  p={p_fs:.2e}")

    # ── SR_nosat distribution for high-Pi ────────────────────────────────────
    print("\n=== SR_nosat by Pi bin (should stay high even where SR_sat collapses) ===")
    cmp = df.groupby('Pi_bin', observed=True)[['sr_sat', 'sr_nosat', 'fsat']].mean().round(3)
    print(cmp.to_string())

    print("\nDone.")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print("saturation_regime_map.py")
    print(f"  Probe Kp = 190/lat  (0.5 x DIPDT ceiling 380/lat)")
    print(f"  Wind: fixed {FIXED_WIND}")
    print(f"  Eval seeds: {EVAL_SEED0}-{EVAL_SEED0 + N_EVAL - 1}  ({N_EVAL} seeds, sigma_p~{1/(2*np.sqrt(N_EVAL)):.2f})")
    print(f"  Kd search seeds: {SEARCH_SEEDS}")

    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')
    print(f"\nLoaded population: {len(pop)} designs")

    print("\nSelecting designs by Pi_keff bin:")
    designs = select_designs(pop)
    n = len(designs)
    print(f"\nTotal: {n} designs")

    # Sim estimate: Kd search (3 seeds × 5 Kd values) + eval (20 seeds × 2 conditions)
    n_sims = n * (len(SEARCH_SEEDS) * (len(KD_RATIOS) + 1) + N_EVAL * 2)
    print(f"Estimated sims: ~{n_sims:,}")
    print("Running in parallel...\n")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(process)(row.to_dict()) for _, row in designs.iterrows()
    )

    df = pd.DataFrame(results).sort_values('Pi').reset_index(drop=True)
    out = RESULTS / 'saturation_regime_map_py.csv'
    df.to_csv(out, index=False)
    print(f"\nSaved {len(df)} rows -> {out.name}")

    analyze(df)


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        analyze()
    else:
        main()
