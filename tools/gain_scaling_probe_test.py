"""
tools/gain_scaling_probe_test.py  (2026-06-23)

GAIN-SCALING ROBUSTNESS TEST for the Pi ~ 275 saturation transition.

RESEARCH QUESTION:
  The saturation regime map used a fixed probe Kp_probe = 190/lat
  (= 0.5 x DIPDT ceiling = 380/lat).  Is Pi_crit ~ 275 a property of the
  PLANT, or an artifact of that specific operating point?

  Weak claim:  "At Kp_probe = 190/lat, saturation begins near Pi = 275."
  Strong claim: "Pi = keff*lat^2 is an intrinsic regime boundary."

  If Pi_crit barely moves across probe gains:
    → the transition is a plant property, not an operating-point artifact.

  If Pi_crit moves dramatically:
    → the threshold is tied to the tuning rule (still an important result,
      but the claim must be scoped: "at recommended operating point").

PROBE GAINS TESTED (all as fractions of DIPDT ceiling = 380/lat):
  0.25x ceiling: Kp_probe = 95/lat
  0.50x ceiling: Kp_probe = 190/lat  (current, reference)
  0.75x ceiling: Kp_probe = 285/lat
  0.25x ceiling: Kp_probe =  47.5/lat  (very conservative)

  We stay below the ceiling to avoid linear phase-margin instability
  contaminating the saturation measurement.

EXPECTED RANGE:
  If Pi_crit varies by < 2x across probe gains → "narrow-band, plant-intrinsic."
  If Pi_crit varies by > 5x across probe gains → "tuning artifact."

DESIGNS: same 10 reference designs from pi_plant_universality_test.py
  (Pi 104 to 1205, covering linear through deep-saturation).

PROTOCOL:
  - For each (design, Kp_probe): find best Kd via 3-seed search, then
    measure fsat with 20 eval seeds.
  - Use BASELINE plant (no parameter changes).
  - Fresh eval seeds: 95001-95020 (disjoint from all prior experiments).
  - Fresh search seeds: 95021-95023.

FILES OUT:
  experiments/results/gain_scaling_probe_py.csv
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

# ── same 10 designs as plant-universality test ────────────────────────────────
DESIGNS = [
    'R2229',   # Pi=104,  lat=2, keff=26.1  -- linear
    'R0200',   # Pi=146,  lat=4, keff=9.2   -- linear, near transition
    'R1119',   # Pi=177,  lat=4, keff=11.1  -- transitional onset
    'R0048',   # Pi=233,  lat=3, keff=25.9  -- transitional/saturation
    'R2058',   # Pi=275,  lat=4, keff=17.2  -- SATURATION ONSET (key)
    'R1641',   # Pi=407,  lat=4, keff=25.5  -- saturation
    'R2069',   # Pi=454,  lat=5, keff=18.2  -- saturation
    'R0207',   # Pi=572,  lat=5, keff=22.9  -- saturation
    'R1513',   # Pi=867,  lat=6, keff=24.1  -- deep saturation
    'R2072',   # Pi=1205, lat=6, keff=33.5  -- extreme saturation
]

# Probe gains as fractions of DIPDT ceiling (380/lat)
# We test four fractions spanning 0.125x to 0.75x ceiling
PROBE_FRACS = [0.125, 0.25, 0.50, 0.75]   # multiplied by (380/lat) = Kp_probe

N_EVAL        = 20
EVAL_SEED0    = 95001     # fresh, disjoint from all prior experiments
SEARCH_SEEDS  = [95021, 95022, 95023]

FIXED_WIND    = 0.25
FIXED_SLEW    = 120.0     # deg/s

KD_RATIOS = np.array([0.05, 0.10, 0.32, 1.00, 3.16])
KD_ZN     = 0.57
KD_CLIP   = (0.01, 32.0)

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
POP_CSV = RESULTS / 'exp1_final_population_py.csv'


# ── helpers ───────────────────────────────────────────────────────────────────

def _keff(row):
    return F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']


def _build(row, slew_deg_s, wind_strength):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    act.slew_max = slew_deg_s * (REF_U_MAX / REF_MAX_GIMBAL_DEG)
    dis.gust_std = wind_strength

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
    """3-seed Kd search around the Z-N formula."""
    kds = np.unique(np.clip(
        list(KD_RATIOS * Kp_probe) + [KD_ZN], KD_CLIP[0], KD_CLIP[1]))
    best_sr, best_kd = -1.0, KD_ZN
    for kd in kds:
        sr, _ = _run_seeds(row, Kp_probe, kd, SEARCH_SEEDS)
        if sr > best_sr:
            best_sr, best_kd = sr, kd
    return float(best_kd)


# ── per-task worker ───────────────────────────────────────────────────────────

def process_one(row_dict, probe_frac):
    """Run one (design, Kp_probe) condition."""
    lat      = int(row_dict['latency_steps'])
    keff     = _keff(row_dict)
    Pi       = keff * lat ** 2
    DIPDT_ceil = 380.0 / lat
    Kp_probe   = probe_frac * DIPDT_ceil

    best_kd     = _best_kd(row_dict, Kp_probe)
    eval_seeds  = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr, fsat    = _run_seeds(row_dict, Kp_probe, best_kd, eval_seeds)

    regime = ('linear'       if fsat < 0.10 else
              'transitional' if fsat < 0.35 else
              'saturation')

    return dict(
        rocket_id   = row_dict.get('rocket_id', 'UNK'),
        lat         = lat,
        keff        = round(keff, 3),
        Pi          = round(Pi, 1),
        DIPDT_ceil  = round(DIPDT_ceil, 1),
        probe_frac  = probe_frac,
        Kp_probe    = round(Kp_probe, 2),
        best_kd     = round(best_kd, 3),
        sr          = round(sr, 4),
        fsat        = round(fsat, 4),
        regime      = regime,
    )


# ── analysis ──────────────────────────────────────────────────────────────────

def analyse(df):
    SAT = 0.35
    print('\n' + '='*65)
    print('GAIN-SCALING ROBUSTNESS TEST')
    print('Question: does Pi_crit move when Kp_probe changes?')
    print('='*65)

    print('\n--- fsat by design and probe fraction ---')
    pivot = df.pivot_table(index='rocket_id', columns='probe_frac',
                           values='fsat', aggfunc='first')
    pivot = pivot.join(df[df['probe_frac']==0.50].set_index('rocket_id')[['Pi','lat']])
    pivot = pivot.sort_values('Pi')
    print(pivot.round(3).to_string())

    print('\n--- Pi_crit by probe_frac (first Pi where fsat >= 0.35) ---')
    for frac in sorted(df['probe_frac'].unique()):
        sub = df[df['probe_frac']==frac].sort_values('Pi')
        sat = sub[sub['fsat'] >= SAT]
        if len(sat) == 0:
            print(f'  Kp_probe = {frac:.3f}x ceiling: NO saturation found')
        else:
            pi_crit  = sat['Pi'].min()
            n_sat    = len(sat)
            ceil_val = sat.iloc[0]['DIPDT_ceil']
            kp_val   = sat.iloc[0]['Kp_probe']
            print(f'  Kp_probe = {frac:.3f}x ceiling ({kp_val:.0f}/lat): '
                  f'Pi_crit = {pi_crit:.0f}   n_sat = {n_sat}')

    crits = []
    for frac in sorted(df['probe_frac'].unique()):
        sub = df[(df['probe_frac']==frac) & (df['fsat'] >= SAT)]
        if len(sub):
            crits.append((frac, sub['Pi'].min()))

    if len(crits) >= 2:
        pi_crit_vals = [c[1] for c in crits]
        rng = max(pi_crit_vals) / min(pi_crit_vals)
        print(f'\nPi_crit range: [{min(pi_crit_vals):.0f}, {max(pi_crit_vals):.0f}]')
        print(f'Max/min ratio: {rng:.2f}x')
        if rng < 2.0:
            verdict = 'NARROW-BAND: transition is plant-intrinsic, not an operating-point artifact.'
        elif rng < 5.0:
            verdict = 'MODERATE shift: transition exists but is partially operating-point dependent.'
        else:
            verdict = 'WIDE shift: transition is predominantly an operating-point artifact.'
        print(f'Verdict: {verdict}')

    print('\n--- Spearman rho(log Pi, fsat) per probe fraction ---')
    for frac in sorted(df['probe_frac'].unique()):
        sub = df[df['probe_frac']==frac]
        rho, p = stats.spearmanr(np.log(sub['Pi']), sub['fsat'])
        print(f'  frac={frac:.3f}:  rho={rho:.3f}  p={p:.2e}  n={len(sub)}')

    print('\n--- Low-Pi control check (Pi < 150 should stay linear at all probe gains) ---')
    low_pi = df[df['Pi'] < 150]
    print(low_pi[['rocket_id','Pi','probe_frac','Kp_probe','fsat','regime']].sort_values(['Pi','probe_frac']).to_string(index=False))


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    # Load the same 10 designs
    pop = pd.read_csv(POP_CSV)
    pop['rocket_id'] = pop.index.map(lambda i: f'R{i:04d}')
    if 'rocket_id' in pd.read_csv(POP_CSV, nrows=1).columns:
        pop = pd.read_csv(POP_CSV)
    selected = pop[pop['rocket_id'].isin(DESIGNS)].copy()
    print(f'Loaded {len(selected)} designs: {sorted(selected["rocket_id"].tolist())}')
    assert len(selected) == len(DESIGNS), f'Missing designs: {set(DESIGNS)-set(selected["rocket_id"])}'

    # Build task list
    tasks = []
    for _, row in selected.iterrows():
        row_dict = row.to_dict()
        for frac in PROBE_FRACS:
            tasks.append((row_dict, frac))

    print(f'\nRunning {len(tasks)} tasks '
          f'({len(DESIGNS)} designs x {len(PROBE_FRACS)} probe fractions)...')

    # Run in parallel
    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(process_one)(rd, fr) for rd, fr in tasks
    )
    df = pd.DataFrame(raw)

    out = RESULTS / 'gain_scaling_probe_py.csv'
    df.to_csv(out, index=False)
    print(f'\nSaved {len(df)} rows -> {out}')

    analyse(df)
