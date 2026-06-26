"""
tools/pi_exponent_test.py -- Controlled exponent test for Pi = keff x latency^alpha.

PART 1 -- SMOKING GUN SEARCH:
  Find a population design with high latency + low keff (low Pi) that is EASY.
  R0522 (keff=41.4, lat=1, Pi=41) proves: high authority + low latency = safe.
  The latency-axis complement: find a design with high lat + low keff -> Pi < Pi_crit -> safe.
  Together these two counterexamples confirm the PRODUCT (not either factor alone) matters.

PART 2 -- CONTROLLED EXPONENT TEST:
  4 fixed keff levels x 8 latency levels, all other parameters held constant.
  Measure fsat at Kp_probe = 190/lat (0.5x DIPDT ceiling) using 30 fresh seeds.
  Find lat_crit (interpolated) where fsat crosses 0.35 (saturation onset threshold).
  Pi_crit = keff x lat_crit^alpha; fit alpha minimising coefficient of variation.
  Theory (kinematics: A_blind = 0.5 keff u_max ?^2) predicts alpha = 2.0.

  Prediction:
    keff=3,  lat 1-8: Pi_max=3x64=192 < Pi_crit -> never saturates (lat_crit not found)
    keff=8,  lat_crit ~ 5.9  (sqrt(275/8))
    keff=18, lat_crit ~ 3.9  (sqrt(275/18))
    keff=30, lat_crit ~ 3.0  (sqrt(275/30))
  If alpha=2.0: keff x lat_crit^2 ~ 275 for all three keff levels (constant).

Seeds: 98001-98030 eval (fresh, disjoint from all prior experiments).

Saves:
  experiments/results/pi_exponent_test_py.csv       -- per-cell fsat / SR measurements
  experiments/results/pi_exponent_smoking_gun_py.csv -- high-lat + low-keff population designs
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path

try:
    from joblib import Parallel, delayed
    HAS_JOBLIB = True
except ImportError:
    HAS_JOBLIB = False

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

# --constants --???????????????????????????????????????????????????????????????
CU_TO_RAD = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX   # ~0.02182
L_NOZZLE  = 0.25

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

FIXED_WIND   = 0.25
N_EVAL       = 30
EVAL_SEED0   = 98001          # seeds 98001-98030 (disjoint from all prior experiments)
SEARCH_SEEDS = [98031, 98032, 98033]
SAT_THRESH   = 0.35           # fsat threshold for saturation regime onset

LATENCY_LEVELS = [1, 2, 3, 4, 5, 6, 7, 8]
ALPHA_TEST     = [1.0, 1.5, 2.0, 2.5, 3.0]

# 4 controlled keff levels: (keff_target, motor_scale, Iyy, label)
# keff = F15_AVG x motor_scale x CU_TO_RAD x L_NOZZLE / Iyy = 0.07855 x motor_scale / Iyy
# Iyy = 0.07855 x motor_scale / keff_target
# All designs: mass=0.80 -> T/W = 14.4xmotor_scale / (0.80x9.81)
KEFF_DESIGNS = [
    # (keff_target, motor_scale, Iyy, label)
    # keff=3:  Iyy = 0.07855x1.0/3.0  = 0.02618  T/W = 14.4/(0.8x9.81) = 1.84 ?
    (3.0,  1.0, 0.026183, 'keff_3'),
    # keff=8:  Iyy = 0.07855x1.0/8.0  = 0.009819 T/W = 1.84 ?
    (8.0,  1.0, 0.009819, 'keff_8'),
    # keff=18: Iyy = 0.07855x2.0/18.0 = 0.008728 T/W = 3.67 ?
    (18.0, 2.0, 0.008728, 'keff_18'),
    # keff=30: Iyy = 0.07855x3.0/30.0 = 0.007855 T/W = 5.50 ?
    (30.0, 3.0, 0.007855, 'keff_30'),
]


# --helpers --?????????????????????????????????????????????????????????????????

def _keff(row):
    return float(F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy'])


def _make_row(motor_scale, Iyy, lat):
    """Construct a design dict for the simulator from controlled parameters."""
    return {
        'mass':                  0.80,
        'Iyy':                   float(Iyy),
        'static_margin':         0.10,   # slightly stable (finned) -- aerodynamics help
        'Cm_alpha':             -50.0,
        'control_effectiveness': 8.0,    # keff_simple; unused in full-physics mode
        'motor_scale':           float(motor_scale),
        'servo_slew_deg_s':     120.0,
        'max_gimbal_deg':        10.0,
        'latency_steps':         int(lat),
        'deadband':              0.0,
        'backlash':              0.0,
        'wind_strength':         FIXED_WIND,  # overridden below
    }


def _build(row):
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    dis.gust_std = FIXED_WIND
    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=False,
        latency=True, thrust_var=False, deadband=False,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fid)
    return plant, act, sen, dis, sc


def _run(row, Kp, Kd, seeds):
    plant, act, sen, dis, sc = _build(row)
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    sr   = float(np.mean([r.success for r in results]))
    fsat = float(np.mean([r.slew_sat_frac for r in results]))
    return sr, fsat


def _best_kd(row, Kp_probe):
    """3-point Kd search at the probe Kp to find the best damping ratio."""
    kd_candidates = np.unique(np.clip(
        [0.3 * Kp_probe * 0.005, 0.57, 1.0, 2.0, 4.0],  # ZN-motivated + fallbacks
        0.01, 16.0
    ))
    best_sr, best_kd_val = -1.0, 0.57
    for kd in kd_candidates:
        sr, _ = _run(row, Kp_probe, kd, SEARCH_SEEDS)
        if sr > best_sr:
            best_sr, best_kd_val = sr, float(kd)
    return best_kd_val


# --Part 1: Smoking gun search --??????????????????????????????????????????????

def part1_smoking_gun(pop):
    """
    Search population for R0522 latency-axis equivalent:
      high latency (5 or 6 steps) + low keff -> low Pi -> EASY label.
    R0522 (keff=41.4, lat=1, Pi=41) falsifies "authority alone causes saturation."
    The complement here falsifies "latency alone causes saturation."
    """
    pop = pop.copy()
    pop['keff_c']  = pop.apply(_keff, axis=1)
    pop['Pi_keff'] = pop['keff_c'] * pop['latency_steps'] ** 2

    high_lat = pop[pop['latency_steps'] >= 5].copy()
    easy_high_lat = high_lat[high_lat.get('final_label', high_lat.get('label', 'EASY')) == 'EASY'].copy()
    if len(easy_high_lat) == 0:
        label_col = [c for c in pop.columns if 'label' in c.lower()]
        print(f"  Warning: no 'final_label' column; tried {label_col}. Using all lat>=5.")
        easy_high_lat = high_lat.copy()

    easy_high_lat = easy_high_lat.sort_values('Pi_keff').head(10)

    print("\n-- SMOKING GUN SEARCH: lat >= 5 + low keff (latency-axis complement to R0522) --")
    print("R0522 reference: keff=41.4, lat=1, Pi=41 -> safe  [high authority, low latency]")
    print("Looking for:     keff<<  , lat>=5, Pi<250 -> safe  [low authority, high latency]")
    print()

    cols_want = ['rocket_id', 'keff_c', 'latency_steps', 'Pi_keff', 'motor_scale', 'Iyy']
    cols = [c for c in cols_want if c in easy_high_lat.columns]
    if len(easy_high_lat) > 0:
        print(easy_high_lat[cols].round(3).to_string(index=False))
        best = easy_high_lat.iloc[0]
        print(f"\n* BEST: {best.get('rocket_id','?')} "
              f"keff={best['keff_c']:.2f} lat={int(best['latency_steps'])} "
              f"Pi={best['Pi_keff']:.1f}")
    else:
        print("  No candidates found.")

    return easy_high_lat


# --Part 2: Controlled exponent test --???????????????????????????????????????

def measure_cell(keff_target, motor_scale, Iyy, lat, label):
    row        = _make_row(motor_scale, Iyy, lat)
    keff_act   = _keff(row)
    Kp_probe   = 190.0 / lat
    kd         = _best_kd(row, Kp_probe)
    seeds      = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr, fsat   = _run(row, Kp_probe, kd, seeds)
    return {
        'keff_label':    label,
        'keff_target':   keff_target,
        'keff_actual':   round(keff_act, 3),
        'motor_scale':   motor_scale,
        'Iyy':           Iyy,
        'lat':           lat,
        'Pi':            round(keff_act * lat ** 2, 2),
        'Kp_probe':      round(Kp_probe, 2),
        'best_kd':       round(kd, 4),
        'sr':            round(sr, 4),
        'fsat':          round(fsat, 4),
        'in_saturation': int(fsat >= SAT_THRESH),
    }


def part2_exponent_test():
    tasks = [
        (kt, ms, iy, lat, lab)
        for (kt, ms, iy, lab) in KEFF_DESIGNS
        for lat in LATENCY_LEVELS
    ]
    print(f"\nRunning {len(tasks)} cells (4 keff x 8 lat, 30 seeds each)?")

    if HAS_JOBLIB:
        rows = Parallel(n_jobs=-1, verbose=3)(
            delayed(measure_cell)(*t) for t in tasks
        )
    else:
        rows = []
        for i, t in enumerate(tasks):
            print(f"  {i+1}/{len(tasks)}: keff={t[3]} lat={t[4]}", flush=True)
            rows.append(measure_cell(*t))

    return pd.DataFrame(rows)


# --Alpha fitting --???????????????????????????????????????????????????????????

def fit_exponent(df):
    """Find alpha in Pi = keff x lat^alpha that minimises CV(Pi_crit) across keff levels."""
    print("\n--lat_crit interpolation --")
    lat_crit_rows = []

    for label in ['keff_3', 'keff_8', 'keff_18', 'keff_30']:
        sub  = df[df['keff_label'] == label].sort_values('lat')
        ke   = sub['keff_actual'].iloc[0]
        above = sub[sub['fsat'] >= SAT_THRESH]
        below = sub[sub['fsat'] < SAT_THRESH]

        if len(above) == 0:
            lc = None
            print(f"  {label} (keff={ke:.1f}): NEVER saturates at lat 1-8 -> Pi_max={ke*64:.0f} < Pi_crit")
        elif len(below) == 0:
            lc = 0.5
            print(f"  {label} (keff={ke:.1f}): already saturated at lat=1")
        else:
            lat_hi = above['lat'].min()
            lat_lo_rows = below[below['lat'] < lat_hi]
            if len(lat_lo_rows) == 0:
                lc = float(lat_hi)
            else:
                lat_lo  = lat_lo_rows['lat'].max()
                fsat_lo = sub[sub['lat'] == lat_lo]['fsat'].values[0]
                fsat_hi = sub[sub['lat'] == lat_hi]['fsat'].values[0]
                span    = fsat_hi - fsat_lo
                t       = (SAT_THRESH - fsat_lo) / span if span > 0 else 0.5
                lc      = lat_lo + t * (lat_hi - lat_lo)
            print(f"  {label} (keff={ke:.1f}): lat_crit={lc:.2f}  "
                  f"-> Pi(alpha=2)={ke * lc**2:.0f}")

        lat_crit_rows.append({'label': label, 'keff': ke, 'lat_crit': lc})

    valid = [r for r in lat_crit_rows if r['lat_crit'] is not None]

    print("\n--Alpha fit --")
    alpha_rows = []
    for alpha in ALPHA_TEST:
        pi_vals = [r['keff'] * r['lat_crit'] ** alpha for r in valid]
        mn  = np.mean(pi_vals)
        sd  = np.std(pi_vals)
        cv  = sd / mn if mn > 0 else 9999.0
        alpha_rows.append({'alpha': alpha, 'Pi_crit_mean': round(mn, 1),
                           'Pi_crit_std': round(sd, 1), 'cv': round(cv, 4)})
        print(f"  alpha={alpha:.1f}: Pi_crit={mn:.1f} ? {sd:.1f}  CV={cv:.4f}")

    best = min(alpha_rows, key=lambda x: x['cv'])
    print(f"\n* BEST ALPHA: {best['alpha']}  (CV={best['cv']:.4f})")
    print(f"  Theory predicts alpha = 2.0  (A_blind = 0.5 keff u_max ?^2)")
    if best['alpha'] == 2.0:
        print("  CONFIRMED: exponent matches kinematics derivation.")
    else:
        print(f"  DEVIATION: best={best['alpha']} vs theory 2.0  --  "
              f"{'close' if abs(best['alpha'] - 2.0) <= 0.5 else 'significant'}")

    return pd.DataFrame(lat_crit_rows), pd.DataFrame(alpha_rows)


# --main --????????????????????????????????????????????????????????????????????

def main():
    print("=" * 70)
    print("Pi EXPONENT TEST")
    print("=" * 70)

    # Load population for Part 1
    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    # Part 1: smoking gun search
    print("\nPART 1: SEARCHING FOR R0522 LATENCY-AXIS EQUIVALENT")
    sg = part1_smoking_gun(pop)
    sg.to_csv(RESULTS / 'pi_exponent_smoking_gun_py.csv', index=False)
    print(f"\nSaved: pi_exponent_smoking_gun_py.csv  (n={len(sg)})")

    # Part 2: controlled exponent test
    print("\n\nPART 2: CONTROLLED EXPONENT TEST")
    df = part2_exponent_test()
    df.to_csv(RESULTS / 'pi_exponent_test_py.csv', index=False)

    # fsat pivot table
    print("\n--fsat pivot (keff x latency) --")
    piv = df.pivot_table(index='keff_label', columns='lat', values='fsat').round(3)
    for lbl in ['keff_3', 'keff_8', 'keff_18', 'keff_30']:
        if lbl in piv.index:
            row_str = '  '.join(f'lat{c}={piv.loc[lbl, c]:.3f}'
                                 for c in piv.columns if c in piv.loc[lbl].index)
            print(f"  {lbl}: {row_str}")

    # Fit exponent
    lat_crit_df, alpha_df = fit_exponent(df)

    print("\n--SMOKING GUN CONTROLLED DEMONSTRATION --")
    keff3 = df[df['keff_label'] == 'keff_3']
    if len(keff3) > 0:
        worst_keff3 = keff3.sort_values('lat').iloc[-1]
        print(f"  keff=3.0, lat=8  Pi={worst_keff3['Pi']:.0f}:")
        print(f"    fsat={worst_keff3['fsat']:.3f}  SR={worst_keff3['sr']:.3f}")
        if worst_keff3['fsat'] < SAT_THRESH:
            print("    -> SAFE: high latency alone (lat=8) does NOT cause saturation when keff=3")
            print("    -> Confirms PRODUCT keffxlat^2 matters, not either factor alone")

    print(f"\nSaved: pi_exponent_test_py.csv")


if __name__ == '__main__':
    main()
