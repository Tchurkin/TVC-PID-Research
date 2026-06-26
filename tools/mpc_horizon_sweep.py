"""
tools/mpc_horizon_sweep.py -- MPC planning horizon sweep under full TVC physics.

RESEARCH QUESTION: H=1 MPC is reactive (rho=-0.807); H=5 MPC escapes Pi (rho=-0.052).
What is H_crit -- the minimum planning horizon needed to escape the Pi constraint?
This clarifies the reactive/non-reactive framing and determines computational requirements.

EXPERIMENT:
  Same 50 designs as mpc_full_physics_audit.py (random_state=42, same td bins).
  Horizons tested: H = 1, 2, 3, 4, 5 (H=1 and H=5 serve as cross-checks on prior results).
  Fresh seeds: 202001-202008 (8 per Kp, disjoint from all prior experiments).
  Full TVC physics: wind with inertia_scale, slew+deadband+backlash, sensor latency+noise.
  10 Kp values [1, 400] (slightly fewer than 12 for runtime).
  Outcome: rho(log Pi, frac_pass) per horizon. Plot where rho crosses from ~-0.75 to ~0.

Seeds used:
  mpc_controller_test.py:   12001-12007
  mpc_full_physics_audit.py: 200001-200010
  THIS FILE:                 202001-202008  (fresh, disjoint)
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.stats import spearmanr

from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX, REF_IYY_KGM2
CU_TO_RAD = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from plant_dynamics import step_plant_simple, simple_effective_params
from actuator_model import ActuatorState, step_actuator
from sensor_model import init_sensor_state, step_sensor
from disturbance_model import init_gust_state, step_disturbance
from controller import PIDParams, PIDState, step_pid, ADRCParams, ADRCState, step_adrc

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

DT         = 0.005
N_STEPS    = int(round(3.0 / DT)) + 1   # 3 s
SUCCESS_RAD = np.deg2rad(30.0)

SEED_START = 202001
N_SEEDS    = 8    # 8 seeds; sigma_p ~ 0.18; sufficient to distinguish pass/fail

KP_VALUES  = np.logspace(0, 2.6, 10)  # 10 log-spaced values [1, ~400]
KD_ZN      = 0.57

HORIZONS   = [1, 2, 3, 4, 5]  # H=1 and H=5 serve as cross-checks on prior results

ADRC_OMEGA_C = 5.0
ADRC_OMEGA0  = 25.0


# ── MPC matrix builder ────────────────────────────────────────────────────────

def _build_mpc_matrices(keff, kp, kd, H):
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
    GQP   = Gamma.T @ Q_big @ Phi

    eig_max  = float(np.linalg.eigvalsh(H_uu)[-1])
    alpha_gd = 1.5 / max(eig_max, 1e-6)

    return H_uu, GQP, alpha_gd, H


# ── Single simulation ─────────────────────────────────────────────────────────

def simulate_one(
    plant, actuator, sensor, disturbance,
    seed, kp, kd, u_max,
    horizon,   # 0 = unconstrained PD baseline; -1 = ADRC; 1..N = MPC H=horizon
    keff_phys,
    mpc_mats=None,
):
    rng = np.random.default_rng(seed)
    theta, q = 0.0, 0.0
    theta_ref = 0.0

    act_state   = ActuatorState(u_servo=0.0, u_output=0.0)
    sens_state  = init_sensor_state(sensor, theta, rng)
    gust_state  = init_gust_state(disturbance, rng)
    pid_state   = PIDState(i_state=0.0, q_ctrl_prev=0.0)
    adrc_state  = ADRCState(z1=theta, z2=0.0, z3=0.0, u_prev=0.0)
    adrc_params = ADRCParams(omega_c=ADRC_OMEGA_C, omega0=ADRC_OMEGA0, b0=keff_phys, u_max=u_max)
    pid_params  = PIDParams(Kp=kp, Kd=kd, Ki=0.0, u_max=u_max)

    theta_ctrl, q_ctrl = theta, q
    peak_th = 0.0

    for k in range(1, N_STEPS):
        t = k * DT

        d_eff = step_disturbance(
            gust_state, disturbance, t, DT, rng,
            inertia_scale=plant.inertia_scale,
            mass_scale=plant.mass_scale,
        )

        if horizon == -1:
            u_cmd = step_adrc(adrc_state, adrc_params, theta_ctrl, q_ctrl, theta_ref, DT)
        elif horizon == 0:
            u_cmd = step_pid(pid_state, pid_params, theta_ctrl, q_ctrl, theta_ref, DT)
        elif horizon == 1:
            raw = -kp * theta_ctrl - kd * q_ctrl
            u_cmd = float(np.clip(raw, -u_max, u_max))
        else:
            # H >= 2: MPC receding-horizon QP
            H_uu, GQP, alpha_gd, H = mpc_mats
            g = GQP @ np.array([theta_ctrl, q_ctrl])
            U = np.zeros(H)
            n_iters = max(25, H * 8)   # more iterations for larger H
            for _ in range(n_iters):
                U = np.clip(U - alpha_gd * (H_uu @ U + g), -u_max, u_max)
            u_cmd = float(U[0])

        u_act = step_actuator(act_state, actuator, u_cmd, DT)
        theta, q = step_plant_simple(theta, q, u_act, d_eff, plant)

        if abs(theta) > peak_th:
            peak_th = abs(theta)
        if peak_th > np.deg2rad(60.0):
            break

        q_ctrl, theta_ctrl = step_sensor(sens_state, sensor, q, theta, DT, rng)

    return bool(peak_th < SUCCESS_RAD)


# ── Per-design evaluation ─────────────────────────────────────────────────────

def evaluate_design(row, kp_values, n_seeds):
    design = row.to_dict()
    plant       = build_plant(design)
    actuator    = build_actuator(design)
    sensor      = build_sensor(design)
    disturbance = build_disturbance(design)

    keff_phys, _ = simple_effective_params(plant)
    u_max = actuator.u_max

    results = []
    for kp in kp_values:
        kd = KD_ZN
        # Pre-build MPC matrices for each horizon
        mpc_mats = {}
        for H in HORIZONS:
            if H >= 2:
                mpc_mats[H] = _build_mpc_matrices(keff_phys, kp, kd, H)

        seeds = list(range(SEED_START, SEED_START + n_seeds))

        sr = {}
        # horizon=-1 -> ADRC, horizon=0 -> PD, horizon=1 -> H=1, horizon=H -> H-step MPC
        for horizon in [-1, 0] + HORIZONS:
            label = f'adrc' if horizon == -1 else (f'pd' if horizon == 0 else f'h{horizon}')
            successes = []
            for s in seeds:
                ok = simulate_one(
                    plant, actuator, sensor, disturbance,
                    seed=s, kp=kp, kd=kd, u_max=u_max,
                    horizon=horizon,
                    keff_phys=keff_phys,
                    mpc_mats=(mpc_mats.get(horizon) if horizon >= 2 else None),
                )
                successes.append(ok)
            sr[label] = float(np.mean(successes))

        rec = {
            'rocket_id': str(row.get('rocket_id', '?')),
            'keff':      round(keff_phys, 3),
            'lat':       int(row.get('latency_steps', 3)),
            'Pi_keff':   round(float(row['Pi_keff']), 1),
            'td':        round(float(row.get('td', 0.0)), 1),
            'kp':        round(float(kp), 2),
        }
        rec.update({k: round(v, 4) for k, v in sr.items()})
        results.append(rec)

    return results


# ── Design selection (same as mpc_full_physics_audit) ────────────────────────

def select_designs(pop):
    pop = pop.copy()
    L_NOZZLE = 0.25
    if 'Pi_keff' not in pop.columns:
        pop['keff_c'] = (F15_AVG_THRUST_N * pop['motor_scale']
                         * CU_TO_RAD * L_NOZZLE / pop['Iyy'])
        pop['Pi_keff'] = pop['keff_c'] * pop['latency_steps'] ** 2
    if 'td' not in pop.columns or pop['td'].max() < 1:
        pop['td'] = (F15_AVG_THRUST_N * pop['motor_scale']
                     * CU_TO_RAD * L_NOZZLE / pop['Iyy']
                     * pop['max_gimbal_deg'] * (np.pi / 180.0) * REF_U_MAX / REF_MAX_GIMBAL_DEG)

    bins = [(0, 30), (30, 60), (60, 120), (120, 200), (200, 1000)]
    selected = []
    for lo, hi in bins:
        sub = pop[(pop['td'] >= lo) & (pop['td'] < hi)].copy()
        n = min(10, len(sub))
        if n == 0:
            continue
        chosen = sub.sample(n=n, random_state=42)
        selected.append(chosen)
        print(f"  bin [{lo:4d},{hi:4d}): {n} designs, "
              f"Pi={chosen['Pi_keff'].min():.0f}-{chosen['Pi_keff'].max():.0f}")

    return pd.concat(selected).reset_index(drop=True)


# ── Aggregate and analyze ─────────────────────────────────────────────────────

def aggregate_to_design_level(detail_df):
    records = []
    for rid, grp in detail_df.groupby('rocket_id'):
        n_kp = len(grp)
        row0 = grp.iloc[0]
        rec = {
            'rocket_id': rid,
            'keff':      row0['keff'],
            'lat':       row0['lat'],
            'Pi_keff':   row0['Pi_keff'],
            'td':        row0['td'],
        }
        for col in ['adrc', 'pd'] + [f'h{H}' for H in HORIZONS]:
            if col in grp.columns:
                rec[f'frac_pass_{col}'] = round((grp[col] >= 0.80).sum() / n_kp, 4)
        records.append(rec)
    return pd.DataFrame(records)


def analyze(design_df):
    print("\n" + "=" * 72)
    print("MPC HORIZON SWEEP -- RESULTS")
    print("=" * 72)
    print("Cross-check against prior results:")
    print("  mpc_full_physics_audit: PD rho=-0.701, H1 rho=-0.807, H5 rho=-0.052")
    print()

    rho_rows = []
    cols = [('frac_pass_pd', 'PD_baseline')]
    cols += [(f'frac_pass_h{H}', f'H={H}_MPC') for H in HORIZONS]
    cols += [('frac_pass_adrc', 'ADRC_reference')]

    for col, label in cols:
        if col not in design_df.columns:
            continue
        sub = design_df[design_df['Pi_keff'] > 0].copy()
        rho, pval = spearmanr(np.log10(sub['Pi_keff']), sub[col])
        n_perfect = (sub[col] == 1.0).sum()
        rho_rows.append({
            'variant': label,
            'n': len(sub),
            'rho': round(rho, 3),
            'p': f'{pval:.2e}',
            'n_perfect': int(n_perfect),
            'mean_frac': round(sub[col].mean(), 3),
        })
        sig = '***' if pval < 0.001 else ('**' if pval < 0.01 else ('*' if pval < 0.05 else '(ns)'))
        print(f"  {label:<18}: rho={rho:+.3f} {sig:4s}  n_perfect={n_perfect}/{len(sub)}  "
              f"mean={sub[col].mean():.3f}")

    # Identify H_crit: smallest H where |rho| < 0.30 (non-significant threshold)
    print()
    print("HORIZON TRANSITION SUMMARY:")
    for row in rho_rows:
        if row['variant'].startswith('H='):
            h = int(row['variant'].split('=')[1].split('_')[0])
            status = 'ESCAPES Pi' if abs(row['rho']) < 0.30 else 'Pi-DEPENDENT'
            print(f"  H={h}: rho={row['rho']:+.3f}  {status}")

    h_crit_candidates = [row for row in rho_rows
                         if row['variant'].startswith('H=') and abs(row['rho']) < 0.30]
    if h_crit_candidates:
        h_crit = min(int(r['variant'].split('=')[1].split('_')[0]) for r in h_crit_candidates)
        print(f"\nH_CRIT = {h_crit}: minimum planning horizon to escape Pi constraint")
    else:
        print(f"\nH_CRIT > {max(HORIZONS)}: Pi constraint NOT escaped within tested horizons")

    return pd.DataFrame(rho_rows)


def main():
    print("=" * 72)
    print("MPC HORIZON SWEEP: H = 1,2,3,4,5 vs Pi constraint")
    print("=" * 72)
    print(f"Seeds: {SEED_START}–{SEED_START + N_SEEDS - 1} (fresh, disjoint from all prior)")
    print(f"Horizons: {HORIZONS}")
    print(f"Kp values: {len(KP_VALUES)}, Seeds/Kp: {N_SEEDS}")
    print()

    pop_path = RESULTS / 'exp1_final_population_py.csv'
    pop = pd.read_csv(pop_path)
    print(f"Loaded population: {len(pop)} designs")

    print("\nSelecting 50 designs (same bins as full-physics audit, random_state=42):")
    designs = select_designs(pop)
    print(f"Total: {len(designs)} designs, Pi={designs['Pi_keff'].min():.0f}–{designs['Pi_keff'].max():.0f}")

    n_sims = len(designs) * len(KP_VALUES) * (len(HORIZONS) + 2) * N_SEEDS
    print(f"\nRunning {len(designs)}×{len(KP_VALUES)} Kp × {len(HORIZONS)+2} variants × {N_SEEDS} seeds")
    print(f"= {n_sims:,} simulations (full TVC physics)")
    print("Estimated time: 30-90 minutes")
    print()

    all_rows = []
    for i, (_, row) in enumerate(designs.iterrows()):
        if (i + 1) % 10 == 1:
            print(f"  Design {i+1}/{len(designs)}: {row.get('rocket_id','?')} "
                  f"keff~{row.get('Pi_keff',0)/max(row.get('latency_steps',3)**2,1):.1f} "
                  f"lat={int(row.get('latency_steps',3))} Pi={row.get('Pi_keff',0):.0f}")
        rows = evaluate_design(row, KP_VALUES, N_SEEDS)
        all_rows.extend(rows)

    detail_df = pd.DataFrame(all_rows)
    detail_path = RESULTS / 'mpc_horizon_sweep_detail_py.csv'
    detail_df.to_csv(detail_path, index=False)
    print(f"\nDetail saved: {detail_path.name} ({len(detail_df)} rows)")

    design_df = aggregate_to_design_level(detail_df)
    design_path = RESULTS / 'mpc_horizon_sweep_py.csv'
    design_df.to_csv(design_path, index=False)
    print(f"Design-level saved: {design_path.name} ({len(design_df)} rows)")

    rho_df = analyze(design_df)
    rho_path = RESULTS / 'mpc_horizon_sweep_rho_py.csv'
    rho_df.to_csv(rho_path, index=False)
    print(f"Rho summary saved: {rho_path.name}")

    print("\nDone.")


if __name__ == '__main__':
    main()
