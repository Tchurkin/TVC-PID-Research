"""
tools/pi_plant_universality_test.py  (2026-06-23)

PLANT-STRUCTURE UNIVERSALITY TEST for the Pi ≈ 275 saturation transition.

RESEARCH QUESTION:
  We've shown Pi_crit ≈ 275 is invariant to wind amplitude and servo speed.
  But is Pi_crit ≈ 275 just a constant of our specific rocket family?

  Skeptic's claim: "The true governing parameter is something else that happens to
  correlate with keff × lat² for your particular designs."

  To answer: vary ONE fundamental plant parameter at a time (×0.5 and ×2).
  Find where fsat = 0.35 for each variant. If it stays in [200, 400] → universal.

VARIANTS TESTED:
  A. max_gimbal × {0.5, 2.0}:
     Changes u_max. keff is UNCHANGED (max_gimbal cancels in keff = T×ms×CU_TO_RAD×l/Iyy).
     PREDICTION: NO effect on Pi_crit. u_max cancels in A_blind/θ_eq = 95 × Pi × dt².
     This tests the theoretical mechanism directly.

  B. Iyy × {0.5, 2.0}:
     Changes keff: keff_eff = keff_orig × (Iyy_orig/Iyy_new).
     Pi_eff = keff_eff × lat².  PREDICTION: Pi_crit still ≈ 275 in Pi_eff space.

  C. motor_scale × {0.5, 2.0}:
     Changes keff: keff_eff = keff_orig × (ms_new/ms_orig).
     Different physical path to keff change vs Iyy.
     PREDICTION: Pi_crit still ≈ 275 in Pi_eff space.

  D. Cm_alpha × {0.5, 2.0}:
     Changes aerodynamic damping (more positive = less stable).
     Cm_alpha is negative in design space [-90, -15]; ×0.5 = less stable, ×2 = more stable.
     PREDICTION: might independently shift Pi_crit (enters through restoring moment, not Pi).
     This is the variant most likely to break universality.

PROTOCOL:
  Same 10 reference designs from servo universality test (Pi 100–1205).
  Kp_probe = 190/lat, constrained Kd search.
  20 eval seeds 93001-93020 (fresh, disjoint from all prior experiments).
  For keff-changing variants: report Pi_eff = keff_eff × lat².

FILES OUT:
  experiments/results/pi_plant_universality_py.csv
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

CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25

# ── design selection (same 10 as servo universality test) ─────────────────────
DESIGNS = [
    'R2229',   # Pi=104,  lat=2, keff=26.1  -- linear, HIGH keff low lat (counterexample)
    'R0200',   # Pi=146,  lat=4, keff=9.2   -- linear, near transition
    'R1119',   # Pi=177,  lat=4, keff=11.1  -- transitional onset
    'R0048',   # Pi=233,  lat=3, keff=25.9  -- transitional
    'R2058',   # Pi=275,  lat=4, keff=17.2  -- SATURATION ONSET (key)
    'R1641',   # Pi=407,  lat=4, keff=25.5  -- saturation
    'R2069',   # Pi=454,  lat=5, keff=18.2  -- saturation
    'R0207',   # Pi=572,  lat=5, keff=22.9  -- saturation
    'R1513',   # Pi=867,  lat=6, keff=24.1  -- deep saturation
    'R2072',   # Pi=1205, lat=6, keff=33.5  -- extreme saturation
]

# Plant variants: (name, parameter, scale_factor)
# 'eff' versions change keff; others don't.
VARIANTS = [
    ('baseline',        None,          1.0),    # no change -- reference
    ('gimbal_half',     'max_gimbal_deg', 0.5), # u_max × 0.5, keff unchanged
    ('gimbal_double',   'max_gimbal_deg', 2.0), # u_max × 2.0, keff unchanged
    ('Iyy_half',        'Iyy',         0.5),    # keff doubles, Pi_eff doubles
    ('Iyy_double',      'Iyy',         2.0),    # keff halves,  Pi_eff halves
    ('motor_half',      'motor_scale', 0.5),    # keff halves,  Pi_eff halves
    ('motor_double',    'motor_scale', 2.0),    # keff doubles, Pi_eff doubles
    ('damping_half',    'Cm_alpha',    0.5),    # less stable (|Cm_alpha| smaller)
    ('damping_double',  'Cm_alpha',    2.0),    # more stable
]

N_EVAL       = 20
EVAL_SEED0   = 93001     # fresh, disjoint from all prior experiments
SEARCH_SEEDS = [93021, 93022, 93023]

FIXED_WIND  = 0.25
FIXED_SLEW  = 120.0      # deg/s -- mid-range hobbyist value

KD_RATIOS = np.array([0.10, 0.32, 1.00, 3.16])
KD_ZN     = 0.57
KD_CLIP   = (0.01, 32.0)

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'


# ── helpers ───────────────────────────────────────────────────────────────────

def _keff(row):
    return F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']


def _apply_variant(row_dict, param, scale):
    """Return a modified copy of row_dict with the named parameter scaled."""
    r = dict(row_dict)
    if param is not None:
        r[param] = float(r[param]) * scale
    return r


def _build(row, slew_deg_s, wind_strength):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    act.slew_max = slew_deg_s * (REF_U_MAX / REF_MAX_GIMBAL_DEG)
    dis.gust_std  = wind_strength

    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fid)
    return plant, act, sen, dis, sc


def _run_seeds(row, Kp, Kd, seeds):
    plant, act, sen, dis, sc = _build(row, FIXED_SLEW, FIXED_WIND)
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    sr   = float(np.mean([r.success for r in results]))
    fsat = float(np.mean([r.slew_sat_frac for r in results]))
    return sr, fsat


def _best_kd(row, Kp_probe):
    kds = np.unique(np.clip(
        list(KD_RATIOS * Kp_probe) + [KD_ZN], KD_CLIP[0], KD_CLIP[1]))
    best_sr, best_kd = -1.0, KD_ZN
    for kd in kds:
        sr, _ = _run_seeds(row, Kp_probe, kd, SEARCH_SEEDS)
        if sr > best_sr:
            best_sr, best_kd = sr, kd
    return float(best_kd), float(best_sr)


# ── per-condition worker ──────────────────────────────────────────────────────

def process_one(row_dict_orig, variant_name, param, scale):
    """Run one (design × plant variant) condition."""
    row = _apply_variant(row_dict_orig, param, scale)

    keff_eff = _keff(row)
    lat      = int(row['latency_steps'])
    Pi_eff   = keff_eff * lat ** 2
    keff_base= _keff(row_dict_orig)
    Pi_base  = keff_base * lat ** 2
    Kp_probe = 190.0 / lat

    best_kd, _ = _best_kd(row, Kp_probe)
    eval_seeds  = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr, fsat    = _run_seeds(row, Kp_probe, best_kd, eval_seeds)

    regime = 'linear' if fsat < 0.10 else ('transitional' if fsat < 0.35 else 'saturation')
    return dict(
        rocket_id    = row_dict_orig.get('rocket_id', 'UNK'),
        variant      = variant_name,
        lat          = lat,
        keff_base    = round(keff_base, 3),
        keff_eff     = round(keff_eff, 3),
        Pi_base      = round(Pi_base, 1),
        Pi_eff       = round(Pi_eff, 1),
        Kp_probe     = round(Kp_probe, 2),
        best_kd      = round(best_kd, 3),
        sr_sat       = round(sr, 4),
        fsat         = round(fsat, 4),
        regime       = regime,
    )


# ── analysis ──────────────────────────────────────────────────────────────────

def analyze(df):
    print("\n" + "=" * 76)
    print("PLANT-STRUCTURE UNIVERSALITY — RESULTS")
    print("=" * 76)

    baseline = df[df['variant'] == 'baseline'].sort_values('Pi_base')

    print(f"\nBaseline fsat vs Pi:")
    print(f"  {'rocket_id':<10} {'Pi_base':>8} {'fsat':>7}  regime")
    for _, r in baseline.iterrows():
        print(f"  {r.rocket_id:<10} {r.Pi_base:>8.0f} {r.fsat:>7.3f}  {r.regime}")

    print("\n--- Per-variant Pi_crit (where fsat first reaches 0.35) ---")
    summary_rows = []
    for vname, grp in df.groupby('variant'):
        grp_s  = grp.sort_values('Pi_eff')
        above  = grp_s[grp_s['fsat'] >= 0.35]
        Pi_crit = above['Pi_eff'].min() if len(above) else np.nan
        n_sat   = (grp_s['fsat'] >= 0.35).sum()
        fsat_mean_mid = grp_s[(grp_s['Pi_eff'] >= 150) & (grp_s['Pi_eff'] <= 500)]['fsat'].mean()
        summary_rows.append(dict(variant=vname, Pi_crit=Pi_crit,
                                 n_saturation=n_sat, fsat_mid_mean=fsat_mean_mid))

    summ = pd.DataFrame(summary_rows).sort_values('Pi_crit')
    print(f"\n  {'variant':<20} {'Pi_crit':>10} {'n_sat':>6} {'fsat_mid':>9}")
    print("  " + "-" * 50)
    baseline_crit = summ.loc[summ['variant'] == 'baseline', 'Pi_crit'].values[0]
    for _, r in summ.iterrows():
        ratio = r.Pi_crit / baseline_crit if (not np.isnan(r.Pi_crit) and baseline_crit > 0) else np.nan
        marker = ''
        if not np.isnan(ratio):
            if ratio < 0.5 or ratio > 2.0:
                marker = '  *** SHIFTED ***'
            elif ratio < 0.7 or ratio > 1.5:
                marker = '  * shifted'
        print(f"  {r.variant:<20} {r.Pi_crit:>10.0f} {r.n_saturation:>6.0f} "
              f"{r.fsat_mid_mean:>9.3f}{marker}")

    print(f"\n  Baseline Pi_crit: {baseline_crit:.0f}")
    valid_crits = summ.dropna(subset=['Pi_crit'])
    if len(valid_crits) > 1:
        print(f"  Range across all variants: [{valid_crits.Pi_crit.min():.0f}, "
              f"{valid_crits.Pi_crit.max():.0f}]")
        spread = valid_crits.Pi_crit.max() / valid_crits.Pi_crit.min()
        print(f"  Max/min ratio: {spread:.2f}×")
        in_range = ((valid_crits.Pi_crit >= 150) & (valid_crits.Pi_crit <= 450)).sum()
        print(f"  Variants with Pi_crit in [150, 450]: {in_range}/{len(valid_crits)}")

    # Detailed pivot: fsat by design × variant
    print("\n--- fsat pivot: Pi_eff ~ 275 designs across variants ---")
    near_crit = df[np.abs(df['Pi_eff'] - 275) < 150].copy()
    if len(near_crit):
        pivot = near_crit.pivot_table(index='rocket_id', columns='variant',
                                       values='fsat', aggfunc='first')
        pivot['Pi_eff'] = near_crit.groupby('rocket_id')['Pi_eff'].mean()
        print(pivot.sort_values('Pi_eff').round(3).to_string())

    # Theory check for max_gimbal variants
    print("\n--- THEORY CHECK: max_gimbal variants (u_max change, keff unchanged) ---")
    gimbal_vars = df[df['variant'].isin(['baseline', 'gimbal_half', 'gimbal_double'])]
    if len(gimbal_vars):
        gp = gimbal_vars.groupby(['rocket_id', 'variant'])['fsat'].first().unstack()
        gp['Pi_base'] = df[df['variant'] == 'baseline'].set_index('rocket_id')['Pi_base']
        print(f"  fsat should be IDENTICAL across all gimbal variants (u_max cancels in theory)")
        print(gp.sort_values('Pi_base').round(3).to_string())

        if 'baseline' in gp and 'gimbal_half' in gp:
            delta_half = (gp['gimbal_half'] - gp['baseline']).abs().mean()
            delta_double = (gp['gimbal_double'] - gp['baseline']).abs().mean() if 'gimbal_double' in gp else np.nan
            print(f"\n  Mean |delta_fsat| gimbal_half vs baseline:   {delta_half:.4f}")
            print(f"  Mean |delta_fsat| gimbal_double vs baseline: {delta_double:.4f}")
            theory_confirmed = delta_half < 0.05 and (np.isnan(delta_double) or delta_double < 0.05)
            print(f"  Theory {'CONFIRMED' if theory_confirmed else 'REJECTED'}: u_max ({'does NOT ' if not theory_confirmed else ''}independently affect fsat)")

    # Spearman rho for each variant
    print("\n--- rho(log Pi_eff, fsat) per variant ---")
    for vname, grp in df.groupby('variant'):
        valid = grp.dropna(subset=['Pi_eff', 'fsat'])
        valid = valid[valid['Pi_eff'] > 0]
        if len(valid) >= 4:
            r, p = stats.spearmanr(np.log(valid['Pi_eff']), valid['fsat'])
            print(f"  {vname:<20}: rho={r:+.3f}  p={p:.2e}  n={len(valid)}")

    print("\nDone.")
    return summ


def main():
    print("pi_plant_universality_test.py -- plant-structure invariance of Pi_crit")
    print(f"Designs: {len(DESIGNS)}  |  Variants: {len(VARIANTS)}  |  "
          f"Seeds: {N_EVAL} eval + 3 search")

    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')
    pop = pop.set_index('rocket_id')

    design_rows = []
    for did in DESIGNS:
        if did not in pop.index:
            print(f"  WARNING: {did} not in population, skipping")
            continue
        r = pop.loc[did].to_dict()
        r['rocket_id'] = did
        design_rows.append(r)

    print(f"Loaded {len(design_rows)} reference designs")

    # Build all (design × variant) tasks
    tasks = []
    for row in design_rows:
        for vname, param, scale in VARIANTS:
            tasks.append((row, vname, param, scale))

    print(f"Total tasks: {len(tasks)}  |  Est. sims: ~{len(tasks) * (N_EVAL + 3 * 5)}")

    print("Running in parallel...")
    results = Parallel(n_jobs=-1, verbose=10)(
        delayed(process_one)(row, vname, param, scale)
        for row, vname, param, scale in tasks
    )

    df = pd.DataFrame(results)
    out_path = RESULTS / 'pi_plant_universality_py.csv'
    df.to_csv(out_path, index=False)
    print(f"\nSaved {len(df)} rows -> pi_plant_universality_py.csv")

    summ = analyze(df)
    summ.to_csv(RESULTS / 'pi_plant_universality_summary_py.csv', index=False)

    return df, summ


if __name__ == '__main__':
    main()
