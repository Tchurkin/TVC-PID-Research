"""
tools/mpc_controller_test.py -- MPC controller invariance test (v2, fixed).

Research question: Does constrained MPC (finite horizon, explicit |u| <= u_max) break
the Pi constraint the way ADRC does?

Prior results: PID rho ~ -0.75, LQR rho = -0.747, SMC rho = -0.753, ADRC rho = -0.283.
Hypothesis: constrained MPC will rho ~ -0.75 (same as PID/LQR/SMC) because:
  - The Pi constraint arises from WIND-DRIVEN bang-bang saturation (unknown disturbance)
  - MPC can plan around a known u_max constraint but NOT around an unknown disturbance
  - ADRC's ESO subtracts disturbance upstream of servo; MPC has no disturbance model
  - Therefore: constraint knowledge alone does not break the Pi ceiling

Variants:
  A. Unconstrained PD: no clip before slew (pure PD baseline)
  B. Constrained H=1 MPC: clips u to [-u_max, u_max] before slew (= saturated PD)
  C. Constrained H=5 MPC: 5-step receding-horizon QP, projected gradient

FIX vs v1: MPC prediction matrices precomputed once per (keff, kp, kd, H) combination,
           not rebuilt at every timestep. Sequential execution to avoid Windows joblib hang.
           Seeds reduced to 7 (sufficient for rho comparison; n=50 designs gives good power).

Seeds: 12001-12007 (7 per design; disjoint from all prior experiments).
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.stats import spearmanr

from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

# simulation constants
CU_TO_RAD     = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX   # ~0.02182 rad/CU
L_NOZZLE      = 0.25

DT            = 0.005          # 200 Hz
N_STEPS       = 600            # 3.0 s
SUCCESS_RAD   = np.deg2rad(30.0)
SLEW_DEG_TO_CU = REF_U_MAX / REF_MAX_GIMBAL_DEG   # 12/15 = 0.8 CU/deg

SEED_START    = 12001
N_SEEDS       = 7

# Wind calibration: OU process, amplitude proportional to keff (aerodynamic coupling)
WIND_FACTOR   = 0.08
WIND_TAU      = 0.3
WIND_STRENGTH = 0.25

# Kp sweep: 20 log-spaced values
KP_VALUES = np.logspace(0, 2.6, 20)   # [1, 400]
KD_ZN     = 0.57    # Z-N universal Kd

# MPC horizon
H_MPC = 5


def _keff(row):
    return float(F15_AVG_THRUST_N * row['motor_scale'] * CU_TO_RAD * L_NOZZLE / row['Iyy'])


# precomputed MPC matrices (fixed for given keff, kp, kd, H, dt)
def _build_mpc_matrices(keff, kp, kd, H):
    """
    Build prediction matrices for H-step MPC QP.
    Returns: (H_uu, GQP, alpha_gd) where:
      H_uu     -- (H x H) Hessian (fixed per call)
      GQP      -- (H x 2) matrix: g = GQP @ x0 (updated per timestep)
      alpha_gd -- gradient step size = 1.5 / eigmax(H_uu)
    """
    A = np.array([[1.0, DT], [0.0, 1.0]])
    B = np.array([[keff * DT**2 * 0.5], [keff * DT]])
    n, m = 2, 1

    Phi   = np.zeros((n * (H + 1), n))
    Gamma = np.zeros((n * (H + 1), m * H))
    A_pow = np.eye(n)
    for i in range(H + 1):
        Phi[i*n:(i+1)*n, :] = A_pow
        A_pow = A @ A_pow
    for j in range(H):
        A_pow = np.eye(n)
        for i in range(j, H):
            Gamma[(i+1)*n:(i+2)*n, j*m:(j+1)*m] += A_pow @ B
            A_pow = A @ A_pow

    Q     = np.diag([kp, kd])
    Q_big = np.kron(np.eye(H + 1), Q)
    R_big = np.eye(H) * 0.5
    H_uu  = Gamma.T @ Q_big @ Gamma + R_big
    GQP   = Gamma.T @ Q_big @ Phi   # (H x n): g = GQP @ x0

    eig_max = float(np.linalg.eigvalsh(H_uu)[-1])
    alpha_gd = 1.5 / max(eig_max, 1e-6)

    return H_uu, GQP, alpha_gd


def _simulate(keff, lat, u_max, slew_cu_s, kp, kd, constrained, seed,
              mpc_matrices=None):
    """
    Custom physics loop.
    constrained: if True, apply explicit u_max clip or MPC.
    mpc_matrices: (H_uu, GQP, alpha_gd, H) if using H>1 MPC; None for H=1.
    Returns: (success, peak_deg, fsat)
    """
    rng           = np.random.default_rng(seed)
    slew_per_step = slew_cu_s * DT

    alpha_w = DT / WIND_TAU
    sigma_w = WIND_STRENGTH * keff * WIND_FACTOR * np.sqrt(2.0 * alpha_w)

    theta = thetadot = u_actual = wind = 0.0
    buf_th  = np.zeros(max(lat, 1))
    buf_thd = np.zeros(max(lat, 1))

    peak_th = 0.0
    n_sat   = 0

    H_uu = GQP = alpha_gd = None
    H = 1
    if mpc_matrices is not None:
        H_uu, GQP, alpha_gd, H = mpc_matrices

    for _ in range(N_STEPS):
        wind = wind * (1.0 - alpha_w) + rng.standard_normal() * sigma_w

        th_obs  = buf_th[-1]  if lat > 0 else theta
        thd_obs = buf_thd[-1] if lat > 0 else thetadot

        if not constrained:
            u_cmd = -kp * th_obs - kd * thd_obs

        elif H_uu is None:
            # H=1: explicit clip
            u_cmd = float(np.clip(-kp * th_obs - kd * thd_obs, -u_max, u_max))

        else:
            # H>1 MPC: projected gradient with precomputed matrices
            x0 = np.array([th_obs, thd_dot := thd_obs])  # noqa
            g  = GQP @ np.array([th_obs, thd_obs])
            U  = np.zeros(H)
            for _ in range(60):
                U = np.clip(U - alpha_gd * (H_uu @ U + g), -u_max, u_max)
            u_cmd = float(U[0])

        delta    = np.clip(u_cmd - u_actual, -slew_per_step, slew_per_step)
        u_actual = float(np.clip(u_actual + delta, -u_max, u_max))

        if abs(u_actual) >= u_max * 0.95:
            n_sat += 1

        thetaddot = keff * u_actual + wind
        thetadot += thetaddot * DT
        theta    += thetadot * DT

        if abs(theta) > peak_th:
            peak_th = abs(theta)

        buf_th  = np.roll(buf_th, 1);  buf_th[0]  = theta
        buf_thd = np.roll(buf_thd, 1); buf_thd[0] = thetadot

    success = peak_th < SUCCESS_RAD
    return bool(success), float(np.rad2deg(peak_th)), float(n_sat / N_STEPS)


def evaluate_design(row, variant_name, constrained, horizon):
    keff   = float(row['keff_c'])
    lat    = int(row['latency_steps'])
    u_max  = float(row.get('max_gimbal_deg', 10.0)) * (REF_U_MAX / REF_MAX_GIMBAL_DEG)
    slew   = float(row.get('servo_slew_deg_s', 120.0)) * SLEW_DEG_TO_CU

    n_pass = 0
    seeds  = list(range(SEED_START, SEED_START + N_SEEDS))

    for kp in KP_VALUES:
        kd = KD_ZN
        # Precompute MPC matrices once per (kp, kd, horizon) combination
        mpc_mats = None
        if constrained and horizon > 1:
            H_uu, GQP, alpha_gd = _build_mpc_matrices(keff, kp, kd, horizon)
            mpc_mats = (H_uu, GQP, alpha_gd, horizon)

        sr_kp = float(np.mean([
            _simulate(keff, lat, u_max, slew, kp, kd, constrained, s, mpc_mats)[0]
            for s in seeds
        ]))
        if sr_kp >= 0.80:
            n_pass += 1

    return {
        'rocket_id': str(row.get('rocket_id', '?')),
        'keff':      round(keff, 3),
        'lat':       lat,
        'Pi_keff':   round(float(row['Pi_keff']), 1),
        'td':        round(float(row['td']), 1),
        'variant':   variant_name,
        'horizon':   horizon,
        'n_pass':    n_pass,
        'frac_pass': round(n_pass / len(KP_VALUES), 4),
    }


def select_designs(pop):
    pop = pop.copy()
    pop['keff_c']  = pop.apply(_keff, axis=1)
    # td column already present in exp1_final_population_py.csv
    if 'td' not in pop.columns or pop['td'].max() < 1:
        pop['td'] = pop['keff_c'] * pop['max_gimbal_deg'] * (np.pi / 180.0) * (REF_U_MAX / REF_MAX_GIMBAL_DEG)
    pop['Pi_keff'] = pop['keff_c'] * pop['latency_steps'] ** 2

    bins = [(0, 30), (30, 60), (60, 120), (120, 200), (200, 1000)]
    selected = []
    for lo, hi in bins:
        sub = pop[(pop['td'] >= lo) & (pop['td'] < hi)].copy()
        n = min(10, len(sub))
        if n == 0:
            print(f"  bin [{lo},{hi}): empty -- skipping")
            continue
        chosen = sub.sample(n=n, random_state=42)
        selected.append(chosen)
        print(f"  bin [{lo:4d},{hi:4d}): {n} designs, "
              f"Pi={chosen['Pi_keff'].min():.0f}-{chosen['Pi_keff'].max():.0f}")

    return pd.concat(selected).reset_index(drop=True)


def analyze(df):
    print("\n" + "=" * 70)
    print("MPC CONTROLLER INVARIANCE -- RESULTS")
    print("=" * 70)
    print("\nSpearman rho(log Pi_keff, frac_pass) per variant:")
    print("  Compare to: LQR rho=-0.747, SMC rho=-0.753, ADRC rho=-0.283")
    print()

    rho_rows = []
    for variant in df['variant'].unique():
        sub = df[df['variant'] == variant].copy()
        sub = sub[sub['Pi_keff'] > 0]
        rho, pval = spearmanr(np.log10(sub['Pi_keff']), sub['frac_pass'])
        rho_rows.append({'variant': variant, 'n': len(sub),
                         'rho': round(rho, 3), 'p': f'{pval:.2e}'})
        print(f"  {variant:<22}: n={len(sub):2d}  rho={rho:+.3f}  p={pval:.2e}")

    rho_df = pd.DataFrame(rho_rows)

    pi_bins = [(0, 100), (100, 300), (300, 1000), (1000, 5000), (5000, 99999)]
    print(f"\nMedian frac_pass by Pi tier:")
    header = f"  {'Pi tier':<18} " + "  ".join(f"{r['variant'][:14]:>14}" for r in rho_rows)
    print(header)
    for lo, hi in pi_bins:
        sub = df[(df['Pi_keff'] >= lo) & (df['Pi_keff'] < hi)]
        if len(sub) == 0:
            continue
        vals = []
        for r in rho_rows:
            v = sub[sub['variant'] == r['variant']]['frac_pass']
            vals.append(f"{v.median():14.3f}" if len(v) > 0 else f"{'--':>14}")
        print(f"  Pi {lo:5d}-{hi:<6d}     " + "  ".join(vals))

    return rho_df


def main():
    print("=" * 70)
    print("MPC CONTROLLER INVARIANCE TEST (v2 -- precomputed matrices, sequential)")
    print("=" * 70)

    pop     = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    print("\nSelecting 50 designs (5 td bins x 10 each):")
    designs = select_designs(pop)
    print(f"Total: {len(designs)} designs, "
          f"Pi={designs['Pi_keff'].min():.0f}-{designs['Pi_keff'].max():.0f}")

    variants = [
        ('Unconstrained_PD', False, 1),
        ('Constrained_H1',   True,  1),
        ('Constrained_H5',   True,  H_MPC),
    ]

    tasks = [
        (row, vname, const, hor)
        for _, row in designs.iterrows()
        for vname, const, hor in variants
    ]
    print(f"\nRunning {len(tasks)} tasks "
          f"({len(designs)} designs x {len(variants)} variants x {len(KP_VALUES)} Kp "
          f"x {N_SEEDS} seeds -- sequential)")

    results = []
    for i, (r, vn, cn, h) in enumerate(tasks):
        if i % 10 == 0:
            print(f"  {i+1}/{len(tasks)}: {vn} keff={r['keff_c']:.1f} lat={r['latency_steps']} "
                  f"Pi={r['Pi_keff']:.0f}", flush=True)
        results.append(evaluate_design(r, vn, cn, h))

    df = pd.DataFrame(results)
    df.to_csv(RESULTS / 'mpc_controller_test_py.csv', index=False)

    rho_df = analyze(df)
    rho_df.to_csv(RESULTS / 'mpc_rho_summary_py.csv', index=False)

    print("\n--- CONCLUSION ---")
    for _, row in rho_df.iterrows():
        rho  = row['rho']
        name = row['variant']
        if name == 'Unconstrained_PD':
            note = f" (expected ~-0.75; baseline vs PID/LQR/SMC)"
        elif name == 'Constrained_H1':
            note = f" (H=1; hypothesis ~-0.75; clip before slew, no disturbance model)"
        elif name == 'Constrained_H5':
            diff = rho - (-0.283)
            if abs(diff) < 0.10:
                note = f" (near ADRC rho=-0.283 -- look-ahead partially escapes Pi)"
            elif rho < -0.60:
                note = f" (near PID regime; planning without disturbance model fails)"
            else:
                note = f" (intermediate; delta vs ADRC = {diff:+.3f})"
        print(f"  {name:<22} rho={rho:+.3f}{note}")

    print(f"\nSaved: mpc_controller_test_py.csv, mpc_rho_summary_py.csv")


if __name__ == '__main__':
    main()
