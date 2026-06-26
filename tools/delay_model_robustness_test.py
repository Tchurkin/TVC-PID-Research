"""
tools/delay_model_robustness_test.py  (2026-06-23)

DELAY-MODEL ROBUSTNESS TEST for the Pi ~ 275 saturation transition.

RESEARCH QUESTION:
  The Pi_crit derivation assumes PURE INTEGER-STEP DELAY (FIFO buffer in sensor_model.py).
  A controls reviewer will ask: "Is this an artifact of modeling delay as integer samples?"

  Real hardware latency sources:
    - I2C/SPI bus: fairly discrete (integral number of 5 ms frames at 200 Hz) → integer OK
    - Jitter: delay varies ±1 step per cycle due to scheduling variance
    - Filter stage: MCU applies IIR low-pass after sampling → first-order lag
    - Mixed: filter after bus → first-order lag + discrete delay combined

  If Pi_crit stays in [150, 600] (< 4× range) across delay model types → NOT an artifact.
  If Pi_crit jumps by > 5× → the threshold is tied to the FIFO implementation, not physics.

DELAY MODELS TESTED:
  1. integer    : pure FIFO, depth L = sensor_latency_steps (baseline, as-implemented)
  2. jitter_1   : each step, actual read depth = L + round(N(0,1)), clipped [1, 2L-1]
                  Models scheduling jitter of ±1-2 steps.
  3. firstlag   : first-order IIR with tau = L × dt (no FIFO), then 1-step causal delay
                  tau = L*dt → same time constant as integer delay.
                  alpha = dt / (L*dt + dt) = 1/(L+1)
  4. lag_delay  : first-order IIR with tau = (L/2) × dt, then FIFO of depth L/2.
                  Splits "half the delay as filter, half as transport delay."

PHYSICS:
  Uses SIMPLE plant model (semi-implicit Euler, θ̈ = keff × u - damp × q + lam × theta + d)
  with PHYSICAL keff (computed from T, motor_scale, l_nozzle, Iyy — not LHS-sampled).
  The saturation mechanism (bang-bang from wind → slew limit) is present in simple mode.

  Using simple physics for this test is valid: we are comparing delay model types WITHIN
  the same physics setup, not comparing to full-physics results. The question is whether
  delay model type shifts Pi_crit relative to the integer-FIFO baseline.

REFERENCE DESIGNS: same 10 designs as regime-map + plant-universality tests (Pi 104–1205).
EVAL: 20 seeds per (design, delay_model). Kp_probe = 190/lat (same as all prior Pi tests).
SEEDS: 97001-97020 eval, 97021-97023 search (all fresh, disjoint from prior experiments).

FILES OUT:
  experiments/results/delay_model_robustness_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from collections import deque
from joblib import Parallel, delayed as jdelayed
from scipy import stats

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario)
from controller import PIDParams, PIDState, step_pid
from actuator_model import ActuatorState, step_actuator
from disturbance_model import init_gust_state, step_disturbance
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import (
    F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX,
    SUCCESS_MAX_THETA_DEG, SUCCESS_END_ERROR_DEG,
    SUCCESS_RMS_ERROR_DEG, SUCCESS_PEAK_ERROR_DEG,
)

CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25

DESIGNS = [
    'R2229', 'R0200', 'R1119', 'R0048', 'R2058',
    'R1641', 'R2069', 'R0207', 'R1513', 'R2072',
]

DELAY_MODELS = ['integer', 'jitter_1', 'firstlag', 'lag_delay']

N_EVAL       = 20
EVAL_SEED0   = 97001
SEARCH_SEEDS = [97021, 97022, 97023]

FIXED_WIND  = 0.25
FIXED_SLEW  = 120.0       # deg/s
DT          = 0.005       # timestep (200 Hz)
T_END       = 3.0

KD_RATIOS = np.array([0.10, 0.32, 1.00, 3.16])
KD_ZN     = 0.57

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
POP_CSV = RESULTS / 'exp1_final_population_py.csv'


# ── helpers ───────────────────────────────────────────────────────────────────

def _keff_physical(row):
    """Physical keff: T × CU_TO_RAD × l / Iyy (not LHS-sampled control_eff)."""
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


# ── custom simulation with pluggable delay model ──────────────────────────────

def simulate_delay_model(row_dict, Kp, Kd, seed, delay_mode):
    """
    One simulation run with a custom sensor delay model.

    Plant is SIMPLE-mode (linearised, semi-implicit Euler) but uses PHYSICAL keff.
    All other components (actuator, sensor noise/bias/quant, disturbance) are full.
    """
    rng = np.random.default_rng(seed)
    plant, act, sen, dis, sc = _build(row_dict, FIXED_SLEW, FIXED_WIND)

    L         = max(1, sen.sensor_latency_steps)
    keff_phys = _keff_physical(row_dict)
    # Simple-mode derived coefficients (from simple_effective_params logic):
    damp_eff  = plant.aero_damp * np.sqrt(max(0.2, plant.mass_scale))
    lam_aero  = plant.lambda_aero   # signed aero stiffness

    N = int(T_END / DT) + 1
    t = np.arange(N) * DT

    # states
    pid_state  = PIDState(i_state=0.0, q_ctrl_prev=0.0)
    act_state  = ActuatorState(u_servo=0.0, u_output=0.0)
    gust_state = init_gust_state(dis, rng)
    pid_params = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                           u_max=act.u_max, i_lim=act.u_max)

    # sensor bias state
    bias_true = sen.gyro_bias_init * rng.standard_normal()
    bias_est  = (1.0 - sen.bias_cal_residual) * bias_true
    theta_int = 0.0

    # custom delay buffers
    if delay_mode == 'integer':
        buf_q     = deque([0.0] * L, maxlen=L)
        buf_theta = deque([0.0] * L, maxlen=L)
        ema_q = ema_theta = alpha = None   # not used

    elif delay_mode == 'jitter_1':
        buf_len   = max(3, 2 * L + 1)
        buf_q     = deque([0.0] * buf_len, maxlen=buf_len)
        buf_theta = deque([0.0] * buf_len, maxlen=buf_len)
        ema_q = ema_theta = alpha = None

    elif delay_mode == 'firstlag':
        alpha     = DT / (L * DT + DT)    # tau = L*dt; EMA coeff = dt/(tau+dt)
        ema_q     = 0.0
        ema_theta = 0.0
        buf_q     = deque([0.0], maxlen=1)  # 1-step causal delay after EMA
        buf_theta = deque([0.0], maxlen=1)

    elif delay_mode == 'lag_delay':
        half_L    = max(1, L // 2)
        alpha     = DT / (half_L * DT + DT)   # tau = L/2 * dt
        ema_q     = 0.0
        ema_theta = 0.0
        buf_q     = deque([0.0] * half_L, maxlen=half_L)
        buf_theta = deque([0.0] * half_L, maxlen=half_L)

    theta_ctrl = 0.0
    q_ctrl     = 0.0

    theta_arr  = np.zeros(N)
    q_arr      = np.zeros(N)
    u_act_arr  = np.zeros(N)

    for k in range(1, N):

        # 1. disturbance
        d_eff = step_disturbance(gust_state, dis, t[k], DT, rng,
                                 inertia_scale=plant.inertia_scale,
                                 mass_scale=plant.mass_scale)

        # 2. controller
        u_cmd = step_pid(pid_state, pid_params, theta_ctrl, q_ctrl, 0.0, DT)

        # 3. actuator (slew + deadband + backlash + saturation)
        u_act = step_actuator(act_state, act, u_cmd, DT)
        u_act_arr[k] = u_act

        # 4. plant — simple physics with physical keff
        th_prev = theta_arr[k - 1]
        q_prev  = q_arr[k - 1]
        qdot    = keff_phys * u_act - damp_eff * q_prev + lam_aero * th_prev + d_eff
        q_new   = q_prev  + DT * qdot
        th_new  = th_prev + DT * q_new    # semi-implicit Euler
        theta_arr[k] = th_new
        q_arr[k]     = q_new

        # 5. sensor noise chain (same as step_sensor, without the FIFO)
        bias_true += sen.gyro_bias_rw * np.sqrt(DT) * rng.standard_normal()
        q_raw  = q_new + bias_true + sen.gyro_noise_std * rng.standard_normal()
        if sen.gyro_quant_lsb > 0:
            q_raw = sen.gyro_quant_lsb * np.round(q_raw / sen.gyro_quant_lsb)
        q_corr    = q_raw - bias_est
        theta_int += DT * q_corr

        # 6. custom delay buffer
        if delay_mode == 'integer':
            buf_q.appendleft(q_corr)
            buf_theta.appendleft(theta_int)
            q_ctrl     = buf_q[-1]
            theta_ctrl = buf_theta[-1]

        elif delay_mode == 'jitter_1':
            buf_q.appendleft(q_corr)
            buf_theta.appendleft(theta_int)
            jitter = int(np.clip(round(rng.standard_normal()), -(L - 1), L - 1))
            idx    = min(max(0, L - 1 + jitter), len(buf_q) - 1)
            bl     = list(buf_q)
            tl     = list(buf_theta)
            q_ctrl     = bl[idx]
            theta_ctrl = tl[idx]

        elif delay_mode == 'firstlag':
            ema_q     = (1.0 - alpha) * ema_q     + alpha * q_corr
            ema_theta = (1.0 - alpha) * ema_theta + alpha * theta_int
            buf_q.appendleft(ema_q)
            buf_theta.appendleft(ema_theta)
            q_ctrl     = buf_q[-1]
            theta_ctrl = buf_theta[-1]

        elif delay_mode == 'lag_delay':
            ema_q     = (1.0 - alpha) * ema_q     + alpha * q_corr
            ema_theta = (1.0 - alpha) * ema_theta + alpha * theta_int
            buf_q.appendleft(ema_q)
            buf_theta.appendleft(ema_theta)
            q_ctrl     = buf_q[-1]
            theta_ctrl = buf_theta[-1]

    # metrics (match simulator.py definitions)
    slew_rates = np.abs(np.diff(u_act_arr)) / DT
    fsat = float(np.mean(slew_rates >= 0.95 * act.slew_max))

    theta_d = np.degrees(theta_arr)
    max_th  = float(np.max(np.abs(theta_d)))
    end_e   = float(np.abs(theta_d[-1]))
    rms_e   = float(np.sqrt(np.mean(theta_d ** 2)))
    peak_e  = float(np.max(np.abs(theta_d)))

    success = (max_th  < SUCCESS_MAX_THETA_DEG
               and end_e   < SUCCESS_END_ERROR_DEG
               and rms_e   < SUCCESS_RMS_ERROR_DEG
               and peak_e  < SUCCESS_PEAK_ERROR_DEG)
    return float(success), fsat


# ── Kd search and seed evaluator ──────────────────────────────────────────────

def _run_seeds(row, Kp, Kd, seeds, delay_mode):
    srs, fsats = [], []
    for s in seeds:
        sr, fsat = simulate_delay_model(row, Kp, Kd, s, delay_mode)
        srs.append(sr)
        fsats.append(fsat)
    return float(np.mean(srs)), float(np.mean(fsats))


def _best_kd(row, Kp_probe, delay_mode):
    kds = sorted({float(np.clip(kd, 0.01, 32.0))
                  for kd in list(KD_RATIOS * Kp_probe) + [KD_ZN]})
    best_sr, best_kd = -1.0, KD_ZN
    for kd in kds:
        sr, _ = _run_seeds(row, Kp_probe, kd, SEARCH_SEEDS, delay_mode)
        if sr > best_sr:
            best_sr, best_kd = sr, kd
    return float(best_kd)


# ── Per-task worker ───────────────────────────────────────────────────────────

def process_one(row_dict, delay_mode):
    lat      = int(row_dict['latency_steps'])
    keff     = _keff_physical(row_dict)
    Pi       = keff * lat ** 2
    Kp_probe = 190.0 / lat

    best_kd    = _best_kd(row_dict, Kp_probe, delay_mode)
    eval_seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr, fsat   = _run_seeds(row_dict, Kp_probe, best_kd, eval_seeds, delay_mode)

    regime = ('linear'       if fsat < 0.10 else
              'transitional' if fsat < 0.35 else
              'saturation')

    return dict(
        rocket_id  = row_dict.get('rocket_id', 'UNK'),
        delay_mode = delay_mode,
        lat        = lat,
        keff       = round(keff, 3),
        Pi         = round(Pi, 1),
        Kp_probe   = round(Kp_probe, 2),
        best_kd    = round(best_kd, 3),
        sr         = round(sr, 4),
        fsat       = round(fsat, 4),
        regime     = regime,
    )


# ── Analysis ──────────────────────────────────────────────────────────────────

def analyse(df):
    SAT = 0.35
    print('\n' + '='*65)
    print('DELAY-MODEL ROBUSTNESS TEST')
    print('Question: does Pi_crit shift with delay model type?')
    print('='*65)

    # pivot: one row per design, one column per delay model
    pivot = df.pivot_table(index='rocket_id', columns='delay_mode',
                           values='fsat', aggfunc='first')
    if 'integer' in pivot.columns:
        pivot = pivot.join(
            df[df['delay_mode'] == 'integer']
              .set_index('rocket_id')[['Pi', 'lat']]
        )
    pivot = pivot.sort_values('Pi')

    print('\n--- fsat by design and delay model ---')
    cols = ['Pi', 'lat'] + [m for m in DELAY_MODELS if m in pivot.columns]
    print(pivot[[c for c in cols if c in pivot.columns]].round(3).to_string())

    print('\n--- Pi_crit by delay model (first Pi where fsat >= 0.35) ---')
    crits = []
    for mode in DELAY_MODELS:
        sub = df[df['delay_mode'] == mode].sort_values('Pi')
        sat = sub[sub['fsat'] >= SAT]
        if len(sat) == 0:
            print(f'  {mode:<12}: NO saturation  '
                  f'(max fsat = {sub["fsat"].max():.3f})')
        else:
            pi_c = sat['Pi'].min()
            print(f'  {mode:<12}: Pi_crit = {pi_c:.0f}  '
                  f'n_sat={len(sat)}/10  '
                  f'fsat=[{sat["fsat"].min():.3f}, {sat["fsat"].max():.3f}]')
            crits.append((mode, pi_c))

    if len(crits) >= 2:
        pis   = [c[1] for c in crits]
        ratio = max(pis) / min(pis)
        print(f'\nPi_crit range: [{min(pis):.0f}, {max(pis):.0f}]')
        print(f'Max/min ratio: {ratio:.2f}x')
        if ratio < 2.0:
            v = 'ROBUST: Pi_crit is NOT an integer-delay artifact.'
        elif ratio < 5.0:
            v = 'MODERATE: shift exists but within one decade.'
        else:
            v = 'SENSITIVE: Pi_crit strongly depends on delay model type.'
        print(f'Verdict: {v}')

    print('\n--- Spearman rho(log Pi, fsat) per delay model ---')
    for mode in DELAY_MODELS:
        sub = df[df['delay_mode'] == mode].dropna()
        if len(sub) >= 3:
            rho, p = stats.spearmanr(np.log(sub['Pi']), sub['fsat'])
            print(f'  {mode:<12}: rho={rho:.3f}  p={p:.2e}')

    if 'integer' in pivot.columns and 'firstlag' in pivot.columns:
        delta = (pivot['firstlag'] - pivot['integer']).abs()
        print(f'\nMean |fsat(firstlag) - fsat(integer)| = {delta.mean():.4f}')
        print(f'Max  |fsat(firstlag) - fsat(integer)| = {delta.max():.4f}')


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    pop = pd.read_csv(POP_CSV)
    if 'rocket_id' not in pop.columns:
        pop['rocket_id'] = pop.index.map(lambda i: f'R{i:04d}')
    selected = pop[pop['rocket_id'].isin(DESIGNS)].copy()
    print(f'Loaded {len(selected)} designs: {sorted(selected["rocket_id"].tolist())}')

    tasks = [(row.to_dict(), mode)
             for _, row in selected.iterrows()
             for mode in DELAY_MODELS]
    print(f'\nRunning {len(tasks)} tasks '
          f'({len(DESIGNS)} designs x {len(DELAY_MODELS)} delay models)...')

    raw = Parallel(n_jobs=-1, verbose=5)(
        jdelayed(process_one)(rd, dm) for rd, dm in tasks
    )
    df = pd.DataFrame(raw)

    out = RESULTS / 'delay_model_robustness_py.csv'
    df.to_csv(out, index=False)
    print(f'\nSaved {len(df)} rows -> {out}')

    analyse(df)
