"""
tools/pi_universality_test.py  (2026-06-23)

UNIVERSALITY TEST: Does the Pi saturation threshold hold across:
  A. Different servo speeds   (60-200 deg/s -- full hobbyist hardware range)
  B. Different wind strengths (0.10-0.40 -- benign to stormy)

If Pi ~ 275 is the saturation onset regardless of servo speed => Pi is a genuine
similarity parameter for the hobbyist TVC regime.  If the threshold shifts
proportionally to servo speed => the correct parameter is Pi/servo (or a rescaled form).

Protocol: same as saturation_regime_map.py -- fixed Kp_probe = 190/lat, constrained
Kd search, 20 eval seeds -- but we override servo_slew and/or wind_strength.

PART A -- Servo speed:
  Designs: 10 from regime map spanning Pi 100-1200
  Servo speeds tested: {60, 100, 150, 200} deg/s  (design-space extremes + midpoints)
  Wind: fixed 0.25 (same as original regime map)

PART B -- Wind strength:
  Designs: 6 from the critical Pi 150-600 transition zone
  Wind strengths: {0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40}
  Servo: each design's LHS-sampled value (natural hardware)

Files out:
  experiments/results/pi_universality_servo_py.csv
  experiments/results/pi_universality_wind_py.csv
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

# ── design selection (from saturation_regime_map_py.csv) ──────────────────────
# Part A: broad Pi range, diverse latency
SERVO_DESIGNS = [
    'R2229',   # Pi=104,  lat=2, keff=26.1  -- linear, high keff, low lat (counterexample class)
    'R0200',   # Pi=146,  lat=4, keff=9.2   -- linear, near transition
    'R1119',   # Pi=177,  lat=4, keff=11.1  -- transitional onset
    'R0048',   # Pi=233,  lat=3, keff=25.9  -- transitional, high keff
    'R2058',   # Pi=275,  lat=4, keff=17.2  -- FIRST saturation (key design)
    'R1641',   # Pi=407,  lat=4, keff=25.5  -- clear saturation
    'R2069',   # Pi=454,  lat=5, keff=18.2  -- saturation
    'R0207',   # Pi=572,  lat=5, keff=22.9  -- saturation
    'R1513',   # Pi=867,  lat=6, keff=24.1  -- deep saturation
    'R2072',   # Pi=1205, lat=6, keff=33.5  -- extreme saturation
]
SERVO_SPEEDS = [60.0, 100.0, 150.0, 200.0]   # deg/s

# Part B: transition zone only, broader wind range
WIND_DESIGNS = [
    'R0200',   # Pi=146,  lat=4   -- just below transition
    'R1119',   # Pi=177,  lat=4   -- transitional onset
    'R0048',   # Pi=233,  lat=3   -- transitional
    'R2058',   # Pi=275,  lat=4   -- saturation onset
    'R1641',   # Pi=407,  lat=4   -- saturation
    'R2069',   # Pi=454,  lat=5   -- saturation
]
WIND_LEVELS  = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]

N_EVAL       = 20
EVAL_SEED0   = 92001     # fresh, disjoint from all prior experiments
SEARCH_SEEDS = [92021, 92022, 92023]

KD_RATIOS = np.array([0.10, 0.32, 1.00, 3.16])
KD_ZN     = 0.57
KD_CLIP   = (0.01, 32.0)

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'


# ── helpers ───────────────────────────────────────────────────────────────────

def _keff(row):
    return F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy']


def _build(row, slew_deg_s, wind_strength):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)

    # Override the parameters being varied
    act.slew_max = slew_deg_s * (REF_U_MAX / REF_MAX_GIMBAL_DEG)  # convert deg/s -> CU/s
    dis.gust_std  = wind_strength

    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fid)
    return plant, act, sen, dis, sc


def _run_seeds(row, Kp, Kd, seeds, slew_deg_s, wind_strength):
    plant, act, sen, dis, sc = _build(row, slew_deg_s, wind_strength)
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    sr   = float(np.mean([r.success for r in results]))
    fsat = float(np.mean([r.slew_sat_frac for r in results]))
    return sr, fsat


def _best_kd(row, Kp_probe, slew_deg_s, wind_strength):
    kds = np.unique(np.clip(
        list(KD_RATIOS * Kp_probe) + [KD_ZN], KD_CLIP[0], KD_CLIP[1]))
    best_sr, best_kd = -1.0, KD_ZN
    for kd in kds:
        sr, _ = _run_seeds(row, Kp_probe, kd, SEARCH_SEEDS, slew_deg_s, wind_strength)
        if sr > best_sr:
            best_sr, best_kd = sr, kd
    return float(best_kd), float(best_sr)


# ── per-condition worker ──────────────────────────────────────────────────────

def process_servo(row_dict, slew_deg_s):
    keff = _keff(row_dict)
    lat  = int(row_dict['latency_steps'])
    Pi   = keff * lat ** 2
    Kp_probe = 190.0 / lat

    best_kd, _ = _best_kd(row_dict, Kp_probe, slew_deg_s, wind_strength=0.25)
    eval_seeds  = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr, fsat    = _run_seeds(row_dict, Kp_probe, best_kd, eval_seeds, slew_deg_s, 0.25)

    regime = 'linear' if fsat < 0.10 else ('transitional' if fsat < 0.35 else 'saturation')
    return dict(
        rocket_id   = row_dict.get('rocket_id', 'UNK'),
        keff        = round(keff, 3),
        lat         = lat,
        Pi          = round(Pi, 1),
        slew_deg_s  = slew_deg_s,
        Kp_probe    = round(Kp_probe, 2),
        best_kd     = round(best_kd, 3),
        sr_sat      = round(sr, 4),
        fsat        = round(fsat, 4),
        regime      = regime,
    )


def process_wind(row_dict, wind_strength):
    keff = _keff(row_dict)
    lat  = int(row_dict['latency_steps'])
    Pi   = keff * lat ** 2
    Kp_probe      = 190.0 / lat
    slew_natural  = float(row_dict.get('servo_slew_deg_s', 120.0))

    best_kd, _ = _best_kd(row_dict, Kp_probe, slew_natural, wind_strength)
    eval_seeds  = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr, fsat    = _run_seeds(row_dict, Kp_probe, best_kd, eval_seeds, slew_natural, wind_strength)

    regime = 'linear' if fsat < 0.10 else ('transitional' if fsat < 0.35 else 'saturation')
    return dict(
        rocket_id      = row_dict.get('rocket_id', 'UNK'),
        keff           = round(keff, 3),
        lat            = lat,
        Pi             = round(Pi, 1),
        slew_deg_s     = round(slew_natural, 1),
        wind_strength  = wind_strength,
        Kp_probe       = round(Kp_probe, 2),
        best_kd        = round(best_kd, 3),
        sr_sat         = round(sr, 4),
        fsat           = round(fsat, 4),
        regime         = regime,
    )


# ── analysis ──────────────────────────────────────────────────────────────────

def analyze_servo(df):
    print("\n" + "=" * 72)
    print("PART A -- SERVO SPEED UNIVERSALITY")
    print("Question: does fsat vs Pi shift when servo_slew changes?")
    print("=" * 72)

    for slew in SERVO_SPEEDS:
        sub = df[df['slew_deg_s'] == slew].sort_values('Pi')
        rho, p = stats.spearmanr(np.log(sub['Pi']), sub['fsat'])
        print(f"\n  servo={slew:.0f} deg/s  (n={len(sub)})  "
              f"rho(log Pi, fsat)={rho:+.3f}  p={p:.2e}")
        for _, r in sub.iterrows():
            bar = '#' * int(r['fsat'] * 40)
            print(f"    Pi={r['Pi']:>6.0f}  lat={r['lat']}  fsat={r['fsat']:.3f}  {bar}")

    # Cross-tabulate: for each design, how much does fsat change across servo speeds?
    print("\n  fsat by [rocket_id x servo_slew] (key: stable = Pi is universal):")
    pivot = df.pivot_table(index='rocket_id', columns='slew_deg_s', values='fsat')
    pivot.insert(0, 'Pi', df.groupby('rocket_id')['Pi'].first())
    pivot = pivot.sort_values('Pi')
    print(pivot.round(3).to_string())

    # For each design, compute range of fsat across servo speeds
    rng = df.groupby('rocket_id')['fsat'].agg(['min', 'max', 'std']).round(3)
    rng['Pi'] = df.groupby('rocket_id')['Pi'].first()
    rng = rng.sort_values('Pi')
    mean_std = rng['std'].mean()
    print(f"\n  Mean within-design fsat std across servo speeds: {mean_std:.3f}")
    print(f"  (small => fsat is servo-invariant; large => servo speed matters)")

    # Saturation onset Pi by servo speed
    print(f"\n  Saturation onset (first fsat > 0.35) by servo speed:")
    for slew in SERVO_SPEEDS:
        sub = df[df['slew_deg_s'] == slew].sort_values('Pi')
        above = sub[sub['fsat'] >= 0.35]
        if len(above):
            print(f"    servo={slew:.0f}: Pi_crit ~ {above.iloc[0]['Pi']:.0f}")
        else:
            print(f"    servo={slew:.0f}: no saturation observed")


def analyze_wind(df):
    print("\n" + "=" * 72)
    print("PART B -- WIND STRENGTH SENSITIVITY")
    print("Question: how much does Pi saturation threshold shift with wind?")
    print("=" * 72)

    for wind in WIND_LEVELS:
        sub = df[df['wind_strength'] == wind].sort_values('Pi')
        above = sub[sub['fsat'] >= 0.35]
        threshold = above.iloc[0]['Pi'] if len(above) else float('nan')
        mean_fsat = sub['fsat'].mean()
        print(f"  wind={wind:.2f}:  Pi_crit ~ {threshold:6.0f}  "
              f"(mean fsat={mean_fsat:.3f},  n_sat={len(above)}/{len(sub)})")

    # Pivot
    print("\n  fsat by [rocket_id x wind_strength] (sorted by Pi):")
    pivot = df.pivot_table(index='rocket_id', columns='wind_strength', values='fsat')
    pivot.insert(0, 'Pi', df.groupby('rocket_id')['Pi'].first())
    pivot = pivot.sort_values('Pi')
    print(pivot.round(3).to_string())

    print("\n  Pi_crit vs wind (for paper: shows domain of validity):")
    crits = []
    for wind in WIND_LEVELS:
        sub = df[df['wind_strength'] == wind].sort_values('Pi')
        above = sub[sub['fsat'] >= 0.35]
        crits.append((wind, above.iloc[0]['Pi'] if len(above) else float('nan')))
    for w, p in crits:
        print(f"    wind={w:.2f} -> Pi_crit ~ {p:.0f}")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print("pi_universality_test.py")
    print(f"  Part A: {len(SERVO_DESIGNS)} designs x {len(SERVO_SPEEDS)} servo speeds = "
          f"{len(SERVO_DESIGNS)*len(SERVO_SPEEDS)} conditions")
    print(f"  Part B: {len(WIND_DESIGNS)} designs x {len(WIND_LEVELS)} wind levels = "
          f"{len(WIND_DESIGNS)*len(WIND_LEVELS)} conditions")
    est = (len(SERVO_DESIGNS)*len(SERVO_SPEEDS) + len(WIND_DESIGNS)*len(WIND_LEVELS)) \
          * (len(SEARCH_SEEDS) * (len(KD_RATIOS) + 1) + N_EVAL)
    print(f"  Estimated sims: ~{est:,}")
    print(f"  Eval seeds: {EVAL_SEED0}-{EVAL_SEED0 + N_EVAL - 1}  (fresh, disjoint)")

    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')
    pop_dict = {r['rocket_id']: r.to_dict() for _, r in pop.iterrows()}
    regime_map = pd.read_csv(RESULTS / 'saturation_regime_map_py.csv')

    # Verify all design IDs exist
    all_ids = set(SERVO_DESIGNS + WIND_DESIGNS)
    missing = all_ids - set(pop_dict.keys())
    if missing:
        print(f"  WARNING: {missing} not found in population CSV")

    # ── Part A: Servo ─────────────────────────────────────────────────────────
    print(f"\nPart A: servo universality...")
    servo_jobs = []
    for rid in SERVO_DESIGNS:
        row = pop_dict[rid]
        for slew in SERVO_SPEEDS:
            servo_jobs.append((row, slew))

    print(f"Running {len(servo_jobs)} servo conditions in parallel...")
    servo_results = Parallel(n_jobs=-1, verbose=3)(
        delayed(process_servo)(row, slew) for row, slew in servo_jobs
    )
    df_servo = pd.DataFrame(servo_results).sort_values(['Pi', 'slew_deg_s']).reset_index(drop=True)
    out_servo = RESULTS / 'pi_universality_servo_py.csv'
    df_servo.to_csv(out_servo, index=False)
    print(f"Saved {len(df_servo)} rows -> {out_servo.name}")

    # ── Part B: Wind ──────────────────────────────────────────────────────────
    print(f"\nPart B: wind sensitivity...")
    wind_jobs = []
    for rid in WIND_DESIGNS:
        row = pop_dict[rid]
        for wind in WIND_LEVELS:
            wind_jobs.append((row, wind))

    print(f"Running {len(wind_jobs)} wind conditions in parallel...")
    wind_results = Parallel(n_jobs=-1, verbose=3)(
        delayed(process_wind)(row, wind) for row, wind in wind_jobs
    )
    df_wind = pd.DataFrame(wind_results).sort_values(['Pi', 'wind_strength']).reset_index(drop=True)
    out_wind = RESULTS / 'pi_universality_wind_py.csv'
    df_wind.to_csv(out_wind, index=False)
    print(f"Saved {len(df_wind)} rows -> {out_wind.name}")

    # ── Analysis ───────────────────────────────────────────────────────────────
    analyze_servo(df_servo)
    analyze_wind(df_wind)

    print("\n" + "=" * 72)
    print("UNIVERSALITY VERDICT")
    print("=" * 72)

    # Servo: compute max fsat range per design
    fsat_std_by_design = df_servo.groupby('rocket_id')['fsat'].std()
    servo_invariant = fsat_std_by_design.mean() < 0.08
    print(f"\n  Servo invariance: mean within-design fsat std = "
          f"{fsat_std_by_design.mean():.3f}  "
          f"=> Pi is {'SERVO-INVARIANT' if servo_invariant else 'SERVO-DEPENDENT'}")

    # Servo: onset Pi range
    onsets = []
    for slew in SERVO_SPEEDS:
        sub = df_servo[df_servo['slew_deg_s'] == slew].sort_values('Pi')
        above = sub[sub['fsat'] >= 0.35]
        if len(above):
            onsets.append(above.iloc[0]['Pi'])
    if len(onsets) >= 2:
        onset_range = max(onsets) - min(onsets)
        print(f"  Saturation onset Pi range across servo speeds: "
              f"{min(onsets):.0f} - {max(onsets):.0f}  (range={onset_range:.0f})")

    # Wind: onset sensitivity
    wind_crits = []
    for wind in WIND_LEVELS:
        sub = df_wind[df_wind['wind_strength'] == wind].sort_values('Pi')
        above = sub[sub['fsat'] >= 0.35]
        if len(above):
            wind_crits.append((wind, above.iloc[0]['Pi']))
    if len(wind_crits) >= 2:
        pi_low  = [p for w, p in wind_crits if w <= 0.15]
        pi_high = [p for w, p in wind_crits if w >= 0.35]
        if pi_low and pi_high:
            ratio = np.mean(pi_low) / np.mean(pi_high)
            print(f"  Pi_crit ratio (low wind / high wind): {ratio:.1f}x "
                  f"(Pi is {'wind-sensitive' if ratio > 1.5 else 'wind-stable'})")

    print("\nDone.")


if __name__ == '__main__':
    if '--analyze' in sys.argv:
        df_servo = pd.read_csv(RESULTS / 'pi_universality_servo_py.csv')
        df_wind  = pd.read_csv(RESULTS / 'pi_universality_wind_py.csv')
        analyze_servo(df_servo)
        analyze_wind(df_wind)
    else:
        main()
