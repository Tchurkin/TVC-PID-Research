"""
tools/flight_sig_rerun_n45.py

Flight signature detection rerun on n=45 FRAGILE labels (n=2400 exp1 run).

Protocol: all 45 FRAGILE + 45 stratified EASY (by theta_ddot quantile) from
exp1_regime_index_py.csv. Run 7 seeds at Kp=2, full physics.

Saves:  experiments/results/flight_sig_updated_py.csv
        (per-seed rows: rocket_id, regime, seed, rms, slew_sat, success, is_fragile)

Outputs AUC, threshold analysis, updated CLAUDE.md numbers.
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from simulator import simulate, PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario

KP = 2.0
KD = 1.0
N_SEEDS = 7
N_EASY_SAMPLE = 45   # match FRAGILE count for balanced AUC

F15_T_AVG, L_NOZZLE = 14.4, 0.25
CU_TO_RAD = (np.pi / 180) * (15 / 12)

CSV = Path('experiments/results/exp1_regime_index_py.csv')
OUT = Path('experiments/results/flight_sig_updated_py.csv')


def _compute_td(df):
    df = df.copy()
    df['keff_full'] = F15_T_AVG * df['motor_scale'] * CU_TO_RAD * L_NOZZLE / df['Iyy']
    df['u_max']     = df['max_gimbal_deg'] * 12.0 / 15.0
    df['td_max']    = df['keff_full'] * df['u_max']
    return df


def run_one(row_dict, seed):
    plant              = build_plant(row_dict)
    act                = build_actuator(row_dict)
    sen                = build_sensor(row_dict)
    dis                = build_disturbance(row_dict)
    sc                 = build_scenario()
    fc                 = FidelityConfig.full()
    act, sen, dis, sc  = apply_fidelity_config(act, sen, dis, sc, fc)
    pid                = PIDParams(Kp=KP, Kd=KD, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    r                  = simulate(pid, plant, act, sen, dis, sc, seed=seed)
    return {
        'rocket_id':  row_dict.get('rocket_id', row_dict.get('design_id', '?')),
        'regime':     row_dict['regime_label'],
        'is_fragile': int(row_dict['regime_label'] == 'FRAGILE'),
        'seed':       seed,
        'rms':        r.rms_error_deg,
        'slew_sat':   r.slew_sat_frac,
        'success':    int(r.success),
        'peak':       r.peak_error_deg,
        'td_max':     row_dict['td_max'],
        'latency':    row_dict['latency_steps'],
    }


def main():
    print(f"Loading {CSV}")
    df = pd.read_csv(CSV)
    df = _compute_td(df)

    frag = df[df['regime_label'] == 'FRAGILE'].copy()
    easy = df[df['regime_label'] == 'EASY'].copy()

    print(f"FRAGILE: n={len(frag)},  EASY pool: n={len(easy)}")

    # Stratified EASY sample: N_EASY_SAMPLE designs spanning the td range
    easy_sorted = easy.sort_values('td_max')
    idx = np.round(np.linspace(0, len(easy_sorted) - 1, N_EASY_SAMPLE)).astype(int)
    easy_sample = easy_sorted.iloc[idx].copy()

    designs = pd.concat([frag, easy_sample], ignore_index=True)
    print(f"Running {len(designs)} designs × {N_SEEDS} seeds = {len(designs) * N_SEEDS} sims")

    tasks = []
    for _, row in designs.iterrows():
        row_dict = row.to_dict()
        for seed in range(1, N_SEEDS + 1):
            tasks.append((row_dict, seed))

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(run_one)(r, s) for r, s in tasks
    )

    res_df = pd.DataFrame(results)
    res_df.to_csv(OUT, index=False)
    print(f"\nSaved: {OUT}  ({len(res_df)} rows)")

    # AUC analysis
    try:
        from sklearn.metrics import roc_auc_score, roc_curve
        per_design = res_df.groupby(['rocket_id', 'regime', 'is_fragile', 'td_max']).agg(
            rms_mean=('rms', 'mean'),
            sr_mean=('success', 'mean'),
        ).reset_index()

        y  = per_design['is_fragile'].values
        s  = per_design['rms_mean'].values
        td = per_design['td_max'].values

        # Bootstrap CI
        rng = np.random.default_rng(0)
        n = len(y)
        boot = [roc_auc_score(y[rng.integers(0,n,n)], s[rng.integers(0,n,n)])
                for _ in range(5000) if len(np.unique(y[rng.integers(0,n,n)])) > 1]
        # Fix: use consistent bootstrap
        boot = []
        for _ in range(5000):
            idx = rng.integers(0, n, n)
            if len(np.unique(y[idx])) > 1:
                boot.append(roc_auc_score(y[idx], s[idx]))
        lo, hi = np.percentile(boot, [2.5, 97.5])

        auc_rms = roc_auc_score(y, s)
        auc_td  = roc_auc_score(y, td)

        print(f"\n=== FLIGHT SIGNATURE DETECTION (n=45 FRAGILE, {N_EASY_SAMPLE} EASY) ===")
        print(f"AUC(RMS, 7-seed):       {auc_rms:.3f} [{lo:.3f}, {hi:.3f}]")
        print(f"AUC(theta_ddot alone):  {auc_td:.3f}")
        print(f"AUC improvement (flight vs spec): {auc_rms - auc_td:+.3f}")

        frag_rms = per_design.loc[per_design['is_fragile']==1, 'rms_mean']
        easy_rms = per_design.loc[per_design['is_fragile']==0, 'rms_mean']
        print(f"\nClass separation at Kp=2 (7-seed means):")
        print(f"  FRAGILE RMS: {frag_rms.mean():.1f} +/- {frag_rms.std():.1f} deg  (n={len(frag_rms)})")
        print(f"  EASY    RMS: {easy_rms.mean():.1f} +/- {easy_rms.std():.1f} deg  (n={len(easy_rms)})")
        print(f"  Ratio: {frag_rms.mean()/easy_rms.mean():.2f}x")

        # Threshold analysis
        fpr, tpr, thresholds = roc_curve(y, s)
        j_scores = tpr - fpr
        j_idx    = np.argmax(j_scores)
        youden_t = thresholds[j_idx]

        print(f"\nThreshold analysis (7-seed mean RMS):")
        for thresh in [6.0, 7.6, 9.0, 11.0, youden_t]:
            pred = (s >= thresh).astype(int)
            tp = int(np.sum((pred==1) & (y==1)))
            fp = int(np.sum((pred==1) & (y==0)))
            fn = int(np.sum((pred==0) & (y==1)))
            tn = int(np.sum((pred==0) & (y==0)))
            prec = tp/(tp+fp) if (tp+fp)>0 else 0.0
            rec  = tp/(tp+fn) if (tp+fn)>0 else 0.0
            f1   = 2*prec*rec/(prec+rec) if (prec+rec)>0 else 0.0
            marker = " ← Youden-J" if abs(thresh - youden_t) < 0.01 else ""
            print(f"  thresh={thresh:.1f}: TP={tp} FP={fp} FN={fn} TN={tn} "
                  f"prec={prec:.2f} rec={rec:.2f} F1={f1:.2f}{marker}")

        # 1-seed AUC
        seed1 = res_df[res_df['seed'] == 1].copy()
        per1  = seed1.groupby(['rocket_id', 'is_fragile']).agg(rms1=('rms','mean')).reset_index()
        auc1  = roc_auc_score(per1['is_fragile'], per1['rms1'])
        print(f"\nAUC(RMS, 1-seed):       {auc1:.3f}")

        print(f"\n=== SUMMARY FOR CLAUDE.md ===")
        print(f"  AUC(7-seed RMS, n=45 FRAGILE, n={N_EASY_SAMPLE} EASY): {auc_rms:.3f} [{lo:.3f}, {hi:.3f}]")
        print(f"  AUC(1-seed RMS): {auc1:.3f}")
        print(f"  Youden-J threshold: {youden_t:.1f} deg (TPR={tpr[j_idx]:.2f}, FPR={fpr[j_idx]:.2f})")
        print(f"  FRAGILE mean RMS: {frag_rms.mean():.1f} +/- {frag_rms.std():.1f} deg")
        print(f"  EASY    mean RMS: {easy_rms.mean():.1f} +/- {easy_rms.std():.1f} deg")

    except ImportError:
        print("sklearn not available — AUC skipped")


if __name__ == '__main__':
    main()
