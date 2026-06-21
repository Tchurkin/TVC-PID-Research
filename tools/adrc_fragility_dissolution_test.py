"""
tools/adrc_fragility_dissolution_test.py

Tests whether FRAGILE is a fundamental hardware limit or a PID-specific artifact.

Protocol: take the final (v2-corrected) n=36 FRAGILE designs plus a td-stratified
n=36 EASY sample, and run BOTH controllers on the *exact* attitude-hold task that
defines FRAGILE (build_scenario(): theta_ref=0, t_end=3s, full physics with wind):

  1. PID at the best gains found by the finer joint Kp x Kd search (30 fresh seeds,
     same as exp1_final_correction.py) -- the best-case PID can do for this hardware.
  2. ADRC at a SINGLE UNIVERSAL setting (omega_c=5, omega0=25, b0=keff_full computed
     from specs) -- NOT tuned per design, NOT searched at all. If this alone closes
     the FRAGILE/EASY SR gap, the danger zone is a property of the PID architecture's
     gain-tuning process, not an unavoidable physical consequence of high authority/
     low inertia hardware.

Output: experiments/results/adrc_dissolution_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams, ADRCParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

ADRC_WC = 5.0
ADRC_W0 = 25.0

N_SEEDS = 20
SEED_START = 3001

POP_CSV  = Path('experiments/results/exp1_final_population_py.csv')
CORR_CSV = Path('experiments/results/exp1_final_correction_py.csv')
OUT_CSV  = Path('experiments/results/adrc_dissolution_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _eval_design(row: dict, kp_pid: float, kd_pid: float) -> dict:
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    seeds = list(range(SEED_START, SEED_START + N_SEEDS))

    pid = PIDParams(Kp=kp_pid, Kd=kd_pid, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    pid_runs = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in seeds]
    pid_sr  = float(np.mean([r.success for r in pid_runs]))
    pid_rms = float(np.mean([r.rms_error_deg for r in pid_runs]))

    b0 = float(row['motor_scale']) * F15_AVG_THRUST_N * CU_TO_RAD * L_NOZZLE / float(row['Iyy'])
    adrc = ADRCParams(omega_c=ADRC_WC, omega0=ADRC_W0, b0=b0, u_max=act.u_max)
    adrc_runs = [simulate(PIDParams(Kp=0.0, Kd=0.0, u_max=act.u_max), plant, act, sen, dis, sc,
                           seed=s, adrc=adrc) for s in seeds]
    adrc_sr  = float(np.mean([r.success for r in adrc_runs]))
    adrc_rms = float(np.mean([r.rms_error_deg for r in adrc_runs]))

    return dict(
        rocket_id = row['rocket_id'],
        regime    = row['final_label'],
        td        = row['td'],
        latency   = row['latency_steps'],
        kp_pid    = kp_pid,
        kd_pid    = kd_pid,
        b0        = b0,
        pid_sr    = pid_sr,
        pid_rms   = pid_rms,
        adrc_sr   = adrc_sr,
        adrc_rms  = adrc_rms,
    )


def main():
    pop  = pd.read_csv(POP_CSV)
    corr = pd.read_csv(CORR_CSV)

    frag = pop[pop['final_label'] == 'FRAGILE'].copy()
    easy = pop[pop['final_label'] == 'EASY'].copy()
    n_sample = len(frag)

    easy_sorted = easy.sort_values('td')
    idx = np.round(np.linspace(0, len(easy_sorted) - 1, n_sample)).astype(int)
    easy_sample = easy_sorted.iloc[idx].copy()

    designs = pd.concat([frag, easy_sample], ignore_index=True)

    # Best-effort PID gains: FRAGILE designs use the finer-search gains from the
    # final correction pass; EASY designs use their existing best_Kp/best_Kd
    # (already established by autotune_continuous across the whole project).
    corr_gains = corr.set_index('rocket_id')[['new_Kp', 'new_Kd']]

    tasks = []
    for _, row in designs.iterrows():
        d = row.to_dict()
        if d['rocket_id'] in corr_gains.index:
            kp = float(corr_gains.loc[d['rocket_id'], 'new_Kp'])
            kd = float(corr_gains.loc[d['rocket_id'], 'new_Kd'])
        else:
            kp = float(d['best_Kp'])
            kd = float(d['best_Kd'])
        tasks.append((d, kp, kd))

    print(f"FRAGILE n={len(frag)}, EASY sample n={n_sample}, total designs={len(designs)}, "
          f"{N_SEEDS} seeds x 2 controllers = {len(designs)*N_SEEDS*2} sims")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design)(d, kp, kd) for d, kp, kd in tasks
    )

    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved: {OUT_CSV}")

    print("\n=== ADRC FRAGILITY DISSOLUTION TEST ===")
    for reg in ['FRAGILE', 'EASY']:
        sub = out[out['regime'] == reg]
        print(f"\n  {reg} (n={len(sub)}):")
        print(f"    PID  (best-effort tuned): SR={sub.pid_sr.mean():.3f}  RMS={sub.pid_rms.mean():.1f} deg")
        print(f"    ADRC (universal, untuned): SR={sub.adrc_sr.mean():.3f}  RMS={sub.adrc_rms.mean():.1f} deg")

    frag = out[out['regime'] == 'FRAGILE']
    easy = out[out['regime'] == 'EASY']
    pid_gap  = easy.pid_sr.mean()  - frag.pid_sr.mean()
    adrc_gap = easy.adrc_sr.mean() - frag.adrc_sr.mean()
    print(f"\nPID  FRAGILE-EASY SR gap:  {pid_gap:.3f}")
    print(f"ADRC FRAGILE-EASY SR gap:  {adrc_gap:.3f}")
    print(f"Gap closed by ADRC: {100*(1 - adrc_gap/pid_gap):.1f}%" if pid_gap > 0 else "N/A")

    n_low_pid_high_adrc = int(((frag.pid_sr < 0.80) & (frag.adrc_sr >= 0.80)).sum())
    print(f"\nFRAGILE designs with PID SR<0.80 but ADRC SR>=0.80: {n_low_pid_high_adrc}/{len(frag)}")


if __name__ == '__main__':
    main()
