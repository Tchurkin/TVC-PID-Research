"""
tools/gyro_filter_alpha_sweep.py  (2026-06-23)

GYRO FILTER ALPHA SWEEP — characterizing the minimum filter strength needed.

BACKGROUND:
  delay_model_robustness_test.py showed that a first-order EMA filter with
  alpha=1/(L+1) (tau=L*dt, same effective lag as the FIFO it replaces) eliminates
  bang-bang saturation entirely (fsat=0.000 for all 10 designs including Pi=1205).
  SR simultaneously improved or held at the probe gain.

  Two gaps remain:
  (A) What MINIMUM alpha is needed? The firstlag test used one specific alpha per design.
  (B) The firstlag model REPLACES the FIFO with an EMA+1step. A builder with existing
      hardware delay would ADD a filter AFTER the FIFO, not replace it. Does that work?

EXPERIMENT:
  Model: integer FIFO (L steps) + EMA(alpha) applied to the delayed rate signal.
  This is the builder-relevant scenario: keep existing latency, add software filter.

  For each design x alpha:
    1. Run FIFO as normal (L-step delay on both theta and q)
    2. Apply EMA to the delayed q BEFORE feeding to PID:
         q_ctrl = (1-alpha)*q_ctrl_prev + alpha*q_fifo
       (theta_ctrl is NOT filtered — only rate signal, since that's what drives saturation
        in the derivative channel. Filtering theta too would hurt angle tracking.)
    3. Measure fsat and SR at Kp_probe = 190/lat.

DESIGNS: 4 high-Pi designs where integer delay shows saturation.
  R2058 (Pi=275, lat=4)   — just above Pi_crit
  R1641 (Pi=407, lat=4)   — moderate saturation
  R1513 (Pi=867, lat=6)   — deep saturation (integer sr=0.25)
  R2072 (Pi=1205, lat=6)  — extreme (integer sr=0.40)

ALPHA VALUES: [1.0, 0.70, 0.50, 0.35, 0.25, 0.18, 0.12, 0.08, 0.05]
  At alpha=1.0: no filtering (should match integer FIFO closely)
  At alpha=1/(L+1): firstlag-equivalent strength (reference point)
  At alpha=0.05: heavy filtering

FILES OUT:
  experiments/results/gyro_filter_alpha_sweep_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from collections import deque
from joblib import Parallel, delayed as jdelayed

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

# Same 4 high-Pi designs from delay_model_robustness_test.py
DESIGNS = ['R2058', 'R1641', 'R1513', 'R2072']

# Alpha=1.0 → no filtering; alpha=1/(L+1) → firstlag-equivalent; alpha=0.05 → heavy
ALPHA_VALUES = [1.0, 0.70, 0.50, 0.35, 0.25, 0.18, 0.12, 0.08, 0.05]

N_EVAL      = 20
EVAL_SEED0  = 99001   # fresh, disjoint from all prior experiments
SEARCH_SEEDS = [99021, 99022, 99023]

FIXED_WIND = 0.25
FIXED_SLEW = 120.0
DT         = 0.005
T_END      = 3.0

KD_RATIOS = np.array([0.10, 0.32, 1.00, 3.16])
KD_ZN     = 0.57

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
POP_CSV = RESULTS / 'exp1_final_population_py.csv'


def _keff_physical(row):
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


def simulate_with_filter(row_dict, Kp, Kd, seed, alpha):
    """
    Integer FIFO + EMA on the rate signal.

    The FIFO provides the full hardware delay (L steps).
    After the FIFO, an EMA filter is applied to q_ctrl only:
      q_ctrl = (1-alpha)*q_ctrl_ema_prev + alpha*q_fifo

    theta_ctrl is not filtered (filtering angle harms tracking directly).
    """
    rng = np.random.default_rng(seed)
    plant, act, sen, dis, sc = _build(row_dict, FIXED_SLEW, FIXED_WIND)

    L = max(1, sen.sensor_latency_steps)
    keff_phys = _keff_physical(row_dict)
    damp_eff  = plant.aero_damp * np.sqrt(max(0.2, plant.mass_scale))
    lam_aero  = plant.lambda_aero

    N = int(T_END / DT) + 1
    t = np.arange(N) * DT

    pid_state  = PIDState(i_state=0.0, q_ctrl_prev=0.0)
    act_state  = ActuatorState(u_servo=0.0, u_output=0.0)
    gust_state = init_gust_state(dis, rng)
    pid_params = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                           u_max=act.u_max, i_lim=act.u_max)

    bias_true = sen.gyro_bias_init * rng.standard_normal()
    bias_est  = (1.0 - sen.bias_cal_residual) * bias_true
    theta_int = 0.0

    # Integer FIFO for both channels
    buf_q     = deque([0.0] * L, maxlen=L)
    buf_theta = deque([0.0] * L, maxlen=L)

    # EMA state for rate channel only
    q_ema = 0.0

    theta_ctrl = 0.0
    q_ctrl     = 0.0

    theta_arr  = np.zeros(N)
    q_arr      = np.zeros(N)
    u_act_arr  = np.zeros(N)

    for k in range(1, N):
        d_eff = step_disturbance(gust_state, dis, t[k], DT, rng,
                                 inertia_scale=plant.inertia_scale,
                                 mass_scale=plant.mass_scale)

        u_cmd = step_pid(pid_state, pid_params, theta_ctrl, q_ctrl, 0.0, DT)

        u_act = step_actuator(act_state, act, u_cmd, DT)
        u_act_arr[k] = u_act

        th_prev = theta_arr[k - 1]
        q_prev  = q_arr[k - 1]
        qdot    = keff_phys * u_act - damp_eff * q_prev + lam_aero * th_prev + d_eff
        q_new   = q_prev  + DT * qdot
        th_new  = th_prev + DT * q_new
        theta_arr[k] = th_new
        q_arr[k]     = q_new

        bias_true += sen.gyro_bias_rw * np.sqrt(DT) * rng.standard_normal()
        q_raw  = q_new + bias_true + sen.gyro_noise_std * rng.standard_normal()
        if sen.gyro_quant_lsb > 0:
            q_raw = sen.gyro_quant_lsb * np.round(q_raw / sen.gyro_quant_lsb)
        q_corr    = q_raw - bias_est
        theta_int += DT * q_corr

        # FIFO delay (same as integer baseline)
        buf_q.appendleft(q_corr)
        buf_theta.appendleft(theta_int)
        q_delayed     = buf_q[-1]
        theta_ctrl = buf_theta[-1]   # theta is NOT additionally filtered

        # EMA on rate signal only (alpha=1.0 → no smoothing, passthrough)
        q_ema  = (1.0 - alpha) * q_ema + alpha * q_delayed
        q_ctrl = q_ema

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


def _run_seeds(row, Kp, Kd, seeds, alpha):
    srs, fsats = [], []
    for s in seeds:
        sr, fsat = simulate_with_filter(row, Kp, Kd, s, alpha)
        srs.append(sr)
        fsats.append(fsat)
    return float(np.mean(srs)), float(np.mean(fsats))


def _best_kd(row, Kp_probe, alpha):
    kds = sorted({float(np.clip(kd, 0.01, 32.0))
                  for kd in list(KD_RATIOS * Kp_probe) + [KD_ZN]})
    best_sr, best_kd = -1.0, KD_ZN
    for kd in kds:
        sr, _ = _run_seeds(row, Kp_probe, kd, SEARCH_SEEDS, alpha)
        if sr > best_sr:
            best_sr, best_kd = sr, kd
    return float(best_kd)


def process_one(row_dict, alpha):
    lat      = int(row_dict['latency_steps'])
    keff     = _keff_physical(row_dict)
    Pi       = keff * lat ** 2
    Kp_probe = 190.0 / lat
    alpha_ref = 1.0 / (lat + 1)   # firstlag-equivalent reference

    best_kd    = _best_kd(row_dict, Kp_probe, alpha)
    eval_seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))
    sr, fsat   = _run_seeds(row_dict, Kp_probe, best_kd, eval_seeds, alpha)

    regime = ('linear'       if fsat < 0.10 else
              'transitional' if fsat < 0.35 else
              'saturation')

    return dict(
        rocket_id  = row_dict.get('rocket_id', 'UNK'),
        alpha      = round(alpha, 4),
        alpha_ref  = round(alpha_ref, 4),   # firstlag-equivalent at this lat
        lat        = lat,
        keff       = round(keff, 3),
        Pi         = round(Pi, 1),
        Kp_probe   = round(Kp_probe, 2),
        best_kd    = round(best_kd, 3),
        sr         = round(sr, 4),
        fsat       = round(fsat, 4),
        regime     = regime,
    )


def analyse(df):
    print('\n' + '='*70)
    print('GYRO FILTER ALPHA SWEEP — FIFO + EMA on rate channel')
    print('='*70)

    print('\n--- fsat by design and alpha (ROWS=design, COLS=alpha) ---')
    pivot_fsat = df.pivot_table(index='rocket_id', columns='alpha',
                                values='fsat', aggfunc='first')
    pi_map = df.drop_duplicates('rocket_id').set_index('rocket_id')['Pi']
    pivot_fsat = pivot_fsat.join(pi_map).sort_values('Pi')
    print(pivot_fsat.round(3).to_string())

    print('\n--- SR by design and alpha ---')
    pivot_sr = df.pivot_table(index='rocket_id', columns='alpha',
                              values='sr', aggfunc='first')
    pivot_sr = pivot_sr.join(pi_map).sort_values('Pi')
    print(pivot_sr.round(3).to_string())

    print('\n--- Critical alpha (first alpha where fsat < 0.35) per design ---')
    for rid, grp in df.groupby('rocket_id'):
        grp = grp.sort_values('alpha', ascending=False)
        Pi_val = grp['Pi'].iloc[0]
        lat_val = grp['lat'].iloc[0]
        alpha_ref = 1.0 / (lat_val + 1)
        sat = grp[grp['fsat'] >= 0.35]
        first_clean = grp[grp['fsat'] < 0.35]
        if len(first_clean) == 0:
            print(f'  {rid} (Pi={Pi_val:.0f}, lat={lat_val}): '
                  f'never exits saturation. Max alpha tried: {grp["alpha"].min():.2f}')
        else:
            alpha_crit = first_clean['alpha'].max()   # highest alpha that exits saturation
            sr_clean = first_clean[first_clean['alpha'] == alpha_crit]['sr'].values[0]
            print(f'  {rid} (Pi={Pi_val:.0f}, lat={lat_val}): '
                  f'alpha_crit={alpha_crit:.2f} '
                  f'(firstlag-equiv={alpha_ref:.2f}) '
                  f'SR@alpha_crit={sr_clean:.3f}')

    print('\n--- SR at alpha=1.0 (no filter) vs firstlag-equivalent alpha ---')
    for rid, grp in df.groupby('rocket_id'):
        grp = grp.sort_values('alpha')
        Pi_val = grp['Pi'].iloc[0]
        lat_val = grp['lat'].iloc[0]
        alpha_ref = 1.0 / (lat_val + 1)
        sr_no_filt = grp[grp['alpha'] == 1.0]['sr'].values
        # find closest alpha to alpha_ref
        grp['alpha_dist'] = (grp['alpha'] - alpha_ref).abs()
        sr_ref = grp.sort_values('alpha_dist').iloc[0]['sr']
        alpha_used = grp.sort_values('alpha_dist').iloc[0]['alpha']
        if len(sr_no_filt) > 0:
            print(f'  {rid} (Pi={Pi_val:.0f}): '
                  f'no filter SR={sr_no_filt[0]:.3f}  '
                  f'alpha={alpha_used:.2f} SR={sr_ref:.3f}')


if __name__ == '__main__':
    pop = pd.read_csv(POP_CSV)
    if 'rocket_id' not in pop.columns:
        pop['rocket_id'] = pop.index.map(lambda i: f'R{i:04d}')
    selected = pop[pop['rocket_id'].isin(DESIGNS)].copy()
    print(f'Loaded {len(selected)} designs: {sorted(selected["rocket_id"].tolist())}')

    tasks = [(row.to_dict(), alpha)
             for _, row in selected.iterrows()
             for alpha in ALPHA_VALUES]
    print(f'\nRunning {len(tasks)} tasks '
          f'({len(DESIGNS)} designs x {len(ALPHA_VALUES)} alpha values)...')
    print('Model: integer FIFO(L) + EMA(alpha) on rate channel only')
    print('(alpha=1.0 ~= no filter; alpha=1/(L+1) = firstlag-equivalent)')

    from joblib import Parallel, delayed as jdelayed
    raw = Parallel(n_jobs=-1, verbose=5)(
        jdelayed(process_one)(rd, a) for rd, a in tasks
    )
    df = pd.DataFrame(raw)

    out = RESULTS / 'gyro_filter_alpha_sweep_py.csv'
    df.to_csv(out, index=False)
    print(f'\nSaved {len(df)} rows -> {out}')

    analyse(df)
