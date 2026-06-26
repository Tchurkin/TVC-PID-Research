"""
tools/mpc_full_physics_audit.py -- MPC audit under full TVC physics.

RESEARCH QUESTION (reviewer-flagged): The original mpc_controller_test.py found that
H=5 MPC achieves frac_pass=1.000 for ALL 50 designs in simplified physics (wind 10-50x
weaker than full TVC aerodynamic coupling). That result is either very important or wrong.
This experiment determines which.

KEY DIFFERENCE FROM ORIGINAL:
  - Full TVC physics: wind scales with plant.inertia_scale (= REF_IYY / Iyy)
    For designs with Iyy=0.005 kg.m^2, inertia_scale=3.6 -> wind amplified 3.6x
    For high-Pi designs (small Iyy + high keff), wind is 10-50x stronger than simplified
  - Full actuator: slew + deadband + backlash (vs slew-only in simplified)
  - Full sensor: latency + gyro noise (vs latency-only in simplified)
  - Full plant: lambda_aero term (aerodynamic stiffness from static_margin)

VARIANTS:
  A. PD baseline:     no u clip before slew (reproduces original baseline)
  B. H=1 MPC:         explicit clip to [-u_max, u_max] before slew
  C. H=5 MPC:         5-step receding-horizon QP (projected gradient, oracle keff, no wind model)
  D. ADRC reference:  omega_c=5, omega0=25, b0=keff_phys (ESO sees all disturbances)

HYPOTHESIS: H=5 MPC will NOT achieve frac_pass=1.000 under full physics because:
  - MPC plans bounded commands to avoid limit-cycle instability (ceiling mechanism)
  - MPC has NO disturbance model -> cannot prevent wind-driven bang-bang (floor mechanism)
  - Under full TVC wind loading, high-Pi designs need Kp >= floor to reject wind
  - MPC with no disturbance model may plan gains that satisfy the bang-bang ceiling
    but fail the wind-rejection floor

PREDICTION IF HYPOTHESIS CORRECT: H=5 MPC rho ~ -0.75 (same as PID/LQR/SMC)
  This would confirm "disturbance estimation (not constraint planning) is uniquely necessary"

PREDICTION IF HYPOTHESIS WRONG: H=5 MPC rho ~ 0 or small (near ADRC)
  This would require revising the architectural claim

Seeds: 200001-200015 (15 per design; all fresh, disjoint from all prior experiments)
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.stats import spearmanr

from units import (
    F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX, REF_IYY_KGM2,
)
CU_TO_RAD = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX  # ~0.02182 rad/CU

# Match original mpc_controller_test.py: peak angle < 30 deg (consistent comparison)
SUCCESS_RAD = np.deg2rad(30.0)
from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from plant_dynamics import step_plant_simple, simple_effective_params
from actuator_model import ActuatorState, step_actuator
from sensor_model import init_sensor_state, step_sensor
from disturbance_model import init_gust_state, step_disturbance
from controller import PIDParams, PIDState, step_pid, ADRCParams, ADRCState, step_adrc

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'

# Simulation constants
DT           = 0.005
N_STEPS      = int(round(3.0 / DT)) + 1   # 3 s at 200 Hz = 601 steps
SUCCESS_RAD  = np.deg2rad(30.0)  # 30 deg — matches original mpc_controller_test.py

SEED_START   = 200001
N_SEEDS      = 10   # 10 seeds per Kp: σ_p ≈ 0.16, sufficient to detect large gaps

# Kp sweep: 12 log-spaced values (was 20 in original; sufficient for frac_pass rho)
KP_VALUES = np.logspace(0, 2.6, 12)   # [1, 400] CU
KD_ZN     = 0.57                       # universal Z-N Kd

# MPC horizon
H_MPC = 5

# ADRC fixed params
ADRC_OMEGA_C  = 5.0
ADRC_OMEGA0   = 25.0


# ── MPC matrix builder (identical to original test) ───────────────────────────

def _build_mpc_matrices(keff, kp, kd, H):
    """
    Build H-step prediction matrices for QP.
    Returns (H_uu, GQP, alpha_gd, H) for use in projected-gradient solve.
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
    GQP   = Gamma.T @ Q_big @ Phi

    eig_max    = float(np.linalg.eigvalsh(H_uu)[-1])
    alpha_gd   = 1.5 / max(eig_max, 1e-6)

    return H_uu, GQP, alpha_gd, H


# ── Full-physics simulation loop with custom controller ───────────────────────

def simulate_one(
    plant, actuator, sensor, disturbance,
    seed, kp, kd, u_max,
    controller_type,         # 'pd', 'h1', 'h5', 'adrc'
    keff_phys,               # plant keff in rad/s^2/CU (for MPC model and ADRC b0)
    mpc_mats=None,           # pre-built MPC matrices (H_uu, GQP, alpha_gd, H)
):
    """
    One simulation under full TVC physics (simple-mode plant):
      - wind scaled by plant.inertia_scale (aerodynamic coupling, full amplitude)
      - actuator with slew + deadband + backlash
      - sensor with latency + gyro noise
      - plant: simple-mode EOM (same as Exp1 evaluation)
      - controller: PD / H=1 clip / H=5 MPC / ADRC

    Returns (success, fsat)
    """
    rng = np.random.default_rng(seed)

    theta    = 0.0
    q        = 0.0
    theta_ref = 0.0

    # Sub-model states
    act_state  = ActuatorState(u_servo=0.0, u_output=0.0)
    sens_state = init_sensor_state(sensor, theta, rng)
    gust_state = init_gust_state(disturbance, rng)

    pid_state  = PIDState(i_state=0.0, q_ctrl_prev=0.0)
    adrc_state = ADRCState(z1=theta, z2=0.0, z3=0.0, u_prev=0.0)
    adrc_params = ADRCParams(
        omega_c=ADRC_OMEGA_C,
        omega0=ADRC_OMEGA0,
        b0=keff_phys,
        u_max=u_max,
    )
    pid_params = PIDParams(Kp=kp, Kd=kd, Ki=0.0, u_max=u_max)

    # PID-mode uses Kp=2 for all designs in ADRC mode (ADRC ignores Kp param)
    # ADRC uses omega_c^2 as Kp internally

    # Delayed sensor output (initial condition)
    theta_ctrl = theta
    q_ctrl     = q

    peak_th = 0.0
    n_sat   = 0

    for k in range(1, N_STEPS):
        t = k * DT

        # 1. Disturbance (full aerodynamic coupling via inertia_scale)
        d_raw = step_disturbance(
            gust_state, disturbance, t, DT, rng,
            inertia_scale=plant.inertia_scale,
            mass_scale=plant.mass_scale,
        )
        d_eff = d_raw  # no bias fault in this audit

        # 2. Controller
        if controller_type == 'adrc':
            u_cmd = step_adrc(adrc_state, adrc_params, theta_ctrl, q_ctrl, theta_ref, DT)

        elif controller_type == 'h5' and mpc_mats is not None:
            # H=5 receding-horizon QP: oracle keff, no disturbance model
            H_uu, GQP, alpha_gd, H = mpc_mats
            g = GQP @ np.array([theta_ctrl, q_ctrl])
            U = np.zeros(H)
            for _ in range(25):  # 25 iterations sufficient for 5x5 with eigenvalue scaling
                U = np.clip(U - alpha_gd * (H_uu @ U + g), -u_max, u_max)
            u_cmd = float(U[0])

        elif controller_type == 'h1':
            # H=1: PD with explicit clip
            raw = -kp * theta_ctrl - kd * q_ctrl
            u_cmd = float(np.clip(raw, -u_max, u_max))

        else:
            # 'pd': unconstrained PD baseline
            u_cmd = step_pid(pid_state, pid_params, theta_ctrl, q_ctrl, theta_ref, DT)

        # 3. Actuator (slew + deadband + backlash)
        u_act = step_actuator(act_state, actuator, u_cmd, DT)

        if abs(u_act) >= u_max * 0.95:
            n_sat += 1

        # 4. Plant (simple mode — same as Exp1 evaluation)
        theta, q = step_plant_simple(theta, q, u_act, d_eff, plant)

        if abs(theta) > peak_th:
            peak_th = abs(theta)

        # Early exit if clearly diverging (saves time)
        if peak_th > np.deg2rad(60.0):
            break

        # 5. Sensor (latency + noise; updates theta_ctrl, q_ctrl)
        q_ctrl, theta_ctrl = step_sensor(sens_state, sensor, q, theta, DT, rng)

    success = peak_th < SUCCESS_RAD
    return bool(success), float(n_sat / N_STEPS)


# ── Per-design evaluation ─────────────────────────────────────────────────────

def evaluate_design(row, kp_values, n_seeds):
    design = row.to_dict()

    plant      = build_plant(design)
    actuator   = build_actuator(design)
    sensor     = build_sensor(design)
    disturbance = build_disturbance(design)

    # Physical keff for MPC model and ADRC b0
    # = what step_plant_simple actually uses as control authority
    keff_phys, _ = simple_effective_params(plant)
    u_max = actuator.u_max

    lat = int(row.get('latency_steps', 3))

    # Pre-build MPC matrices at every (kp, kd) combo
    results = []

    for kp in kp_values:
        kd = KD_ZN
        # Precompute H=5 matrices once per Kp
        mpc_mats_h5 = _build_mpc_matrices(keff_phys, kp, kd, H_MPC)

        seeds = list(range(SEED_START, SEED_START + n_seeds))

        sr = {}
        for ctrl in ('pd', 'h1', 'h5', 'adrc'):
            successes = []
            for s in seeds:
                ok, _ = simulate_one(
                    plant, actuator, sensor, disturbance,
                    seed=s, kp=kp, kd=kd, u_max=u_max,
                    controller_type=ctrl,
                    keff_phys=keff_phys,
                    mpc_mats=(mpc_mats_h5 if ctrl == 'h5' else None),
                )
                successes.append(ok)
            sr[ctrl] = float(np.mean(successes))

        results.append({
            'rocket_id': str(row.get('rocket_id', '?')),
            'keff':      round(keff_phys, 3),
            'lat':       lat,
            'Pi_keff':   round(float(row['Pi_keff']), 1),
            'td':        round(float(row.get('td', 0.0)), 1),
            'kp':        round(float(kp), 2),
            'sr_pd':     round(sr['pd'],   4),
            'sr_h1':     round(sr['h1'],   4),
            'sr_h5':     round(sr['h5'],   4),
            'sr_adrc':   round(sr['adrc'], 4),
        })

    return results


# ── Main ─────────────────────────────────────────────────────────────────────

def select_designs(pop):
    pop = pop.copy()
    # Compute Pi_keff if not present (should be in final population)
    CU_TO_RAD_val = CU_TO_RAD
    L_NOZZLE = 0.25
    if 'Pi_keff' not in pop.columns:
        pop['keff_c'] = (F15_AVG_THRUST_N * pop['motor_scale']
                         * CU_TO_RAD_val * L_NOZZLE / pop['Iyy'])
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
            print(f"  bin [{lo},{hi}): empty -- skipping")
            continue
        chosen = sub.sample(n=n, random_state=42)
        selected.append(chosen)
        print(f"  bin [{lo:4d},{hi:4d}): {n} designs, "
              f"Pi={chosen['Pi_keff'].min():.0f}-{chosen['Pi_keff'].max():.0f}")

    return pd.concat(selected).reset_index(drop=True)


def aggregate_to_design_level(detail_df):
    """Collapse per-Kp rows to per-design frac_pass (fraction of Kp values with SR>=0.80)."""
    records = []
    for rid, grp in detail_df.groupby('rocket_id'):
        n_kp = len(grp)
        row0 = grp.iloc[0]
        records.append({
            'rocket_id': rid,
            'keff':      row0['keff'],
            'lat':       row0['lat'],
            'Pi_keff':   row0['Pi_keff'],
            'td':        row0['td'],
            'frac_pass_pd':   round((grp['sr_pd']   >= 0.80).sum() / n_kp, 4),
            'frac_pass_h1':   round((grp['sr_h1']   >= 0.80).sum() / n_kp, 4),
            'frac_pass_h5':   round((grp['sr_h5']   >= 0.80).sum() / n_kp, 4),
            'frac_pass_adrc': round((grp['sr_adrc'] >= 0.80).sum() / n_kp, 4),
            'best_sr_pd':     round(grp['sr_pd'].max(),   4),
            'best_sr_h1':     round(grp['sr_h1'].max(),   4),
            'best_sr_h5':     round(grp['sr_h5'].max(),   4),
            'best_sr_adrc':   round(grp['sr_adrc'].max(), 4),
        })
    return pd.DataFrame(records)


def analyze(design_df):
    print("\n" + "=" * 72)
    print("MPC FULL-PHYSICS AUDIT -- RESULTS")
    print("=" * 72)
    print("\nSpearman rho(log Pi_keff, frac_pass) per variant:")
    print("  Simplified-physics baselines: PD rho=-0.969, H1 rho=-0.969, H5 rho=NaN")
    print("  Full TVC (other tests): LQR rho=-0.747, SMC rho=-0.753, ADRC rho=-0.283")
    print()

    rho_rows = []
    for col, label in [
        ('frac_pass_pd',   'Unconstrained_PD'),
        ('frac_pass_h1',   'Constrained_H1'),
        ('frac_pass_h5',   'Constrained_H5_MPC'),
        ('frac_pass_adrc', 'ADRC_reference'),
    ]:
        sub = design_df[design_df['Pi_keff'] > 0].copy()
        rho, pval = spearmanr(np.log10(sub['Pi_keff']), sub[col])
        n_perfect = (sub[col] == 1.0).sum()
        rho_rows.append({
            'variant': label, 'n': len(sub),
            'rho': round(rho, 3), 'p': f'{pval:.2e}',
            'n_perfect': n_perfect,
            'mean_frac': round(sub[col].mean(), 3),
        })
        print(f"  {label:<28}: n={len(sub):2d}  rho={rho:+.3f}  p={pval:.2e}  "
              f"n_perfect={n_perfect}/{len(sub)}  mean={sub[col].mean():.3f}")

    print()
    print("Best achievable SR (any Kp) by Pi tier:")
    pi_bins = [(0, 100), (100, 300), (300, 1000), (1000, 5000), (5000, 99999)]
    cols_to_show = [
        ('best_sr_pd',   'PD'),
        ('best_sr_h1',   'H1'),
        ('best_sr_h5',   'H5'),
        ('best_sr_adrc', 'ADRC'),
    ]
    hdr = f"  {'Pi tier':<20} " + "  ".join(f"{l:>8}" for _, l in cols_to_show)
    print(hdr)
    for lo, hi in pi_bins:
        sub = design_df[(design_df['Pi_keff'] >= lo) & (design_df['Pi_keff'] < hi)]
        if len(sub) == 0:
            continue
        vals = "  ".join(f"{sub[c].mean():8.3f}" for c, _ in cols_to_show)
        print(f"  Pi {lo:5d}-{hi:<7d}  n={len(sub):2d}  {vals}")

    # Critical comparison: designs where PD fails (best_sr_pd < 0.80)
    failing = design_df[design_df['best_sr_pd'] < 0.80].copy()
    if len(failing) > 0:
        print(f"\nDesigns where PD fails (best_sr_pd<0.80): n={len(failing)}")
        print(f"  {'rocket_id':<10} {'Pi':>7} {'lat':>4} "
              f"{'PD':>7} {'H1':>7} {'H5':>7} {'ADRC':>7}")
        for _, r in failing.sort_values('Pi_keff', ascending=False).iterrows():
            print(f"  {r['rocket_id']:<10} {r['Pi_keff']:7.0f} {r['lat']:4d} "
                  f"{r['best_sr_pd']:7.3f} {r['best_sr_h1']:7.3f} "
                  f"{r['best_sr_h5']:7.3f} {r['best_sr_adrc']:7.3f}")
    else:
        print("\nNo designs with PD best_sr < 0.80 (unexpectedly high wind tolerance)")

    return pd.DataFrame(rho_rows)


def main():
    import sys
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    print("=" * 72)
    print("MPC FULL-PHYSICS AUDIT (REVIEWER-FLAGGED: is simplified result real or artifact?)")
    print("=" * 72)
    print(f"Seeds: {SEED_START}–{SEED_START + N_SEEDS - 1} ({N_SEEDS} per Kp value)")
    print(f"Kp sweep: {len(KP_VALUES)} values in [{KP_VALUES[0]:.1f}, {KP_VALUES[-1]:.1f}] CU")
    print(f"Physics: FULL (wind with inertia_scale, slew+deadband+backlash, sensor latency+noise)")
    print(f"Hypothesis: H=5 MPC rho ~ -0.75 (same as PID/LQR/SMC) when wind is realistic")
    print()

    pop = pd.read_csv(RESULTS / 'exp1_final_population_py.csv')

    print("\nSelecting 50 designs (5 td bins x 10 each):")
    designs = select_designs(pop)
    print(f"Total: {len(designs)} designs, "
          f"Pi={designs['Pi_keff'].min():.0f}–{designs['Pi_keff'].max():.0f}")

    total_tasks = len(designs) * len(KP_VALUES)
    print(f"\nRunning {total_tasks} design×Kp combos × 4 variants × {N_SEEDS} seeds "
          f"= {total_tasks * 4 * N_SEEDS:,} simulations (sequential)")
    print("This may take 20-60 minutes on a typical CPU.\n")

    all_rows = []
    for i, (_, row) in enumerate(designs.iterrows()):
        if i % 5 == 0:
            print(f"  Design {i+1}/{len(designs)}: {row.get('rocket_id','?')} "
                  f"keff~{row['Pi_keff']/row['latency_steps']**2:.1f} "
                  f"lat={int(row['latency_steps'])} Pi={row['Pi_keff']:.0f}", flush=True)
        rows = evaluate_design(row, KP_VALUES, N_SEEDS)
        all_rows.extend(rows)

    detail_df = pd.DataFrame(all_rows)
    detail_df.to_csv(RESULTS / 'mpc_full_physics_audit_detail_py.csv', index=False)

    design_df = aggregate_to_design_level(detail_df)
    design_df.to_csv(RESULTS / 'mpc_full_physics_audit_py.csv', index=False)

    rho_df = analyze(design_df)
    rho_df.to_csv(RESULTS / 'mpc_full_physics_audit_rho_py.csv', index=False)

    print("\n--- VERDICT ---")
    h5_rho = rho_df.loc[rho_df['variant'] == 'Constrained_H5_MPC', 'rho'].values
    adrc_rho = rho_df.loc[rho_df['variant'] == 'ADRC_reference', 'rho'].values
    if len(h5_rho) > 0 and len(adrc_rho) > 0:
        h5_rho = h5_rho[0]
        adrc_rho = adrc_rho[0]
        if abs(h5_rho) < 0.30:
            print(f"  H5 MPC rho={h5_rho:+.3f}: NEAR ADRC ({adrc_rho:+.3f})")
            print("  -> Constraint planning WITHOUT disturbance model approaches ADRC performance")
            print("  -> CLAIM 'disturbance estimation uniquely necessary' REQUIRES REVISION")
        elif abs(h5_rho) > 0.60:
            print(f"  H5 MPC rho={h5_rho:+.3f}: NEAR PID/LQR/SMC (expected ~-0.75)")
            print("  -> Constraint planning DOES NOT escape the Pi constraint under full wind")
            print("  -> CLAIM 'disturbance estimation uniquely necessary' IS CONFIRMED")
        else:
            print(f"  H5 MPC rho={h5_rho:+.3f}: INTERMEDIATE (ADRC={adrc_rho:+.3f})")
            print("  -> Partial benefit from constraint planning; ESO still distinctly better")

    print(f"\nSaved: mpc_full_physics_audit_detail_py.csv, "
          f"mpc_full_physics_audit_py.csv, mpc_full_physics_audit_rho_py.csv")


if __name__ == '__main__':
    main()
