"""
tools/flight_sig_rerun_final.py

Flight signature detection rerun on the FINAL (v2) corrected population:
EASY=2362, FRAGILE=36, MARGINAL=0, INFEASIBLE=2 (exp1_final_population_py.csv).
Supersedes flight_sig_rerun_n45.py, which used the stale n=45 FRAGILE labels.

Protocol unchanged from the n=45 rerun: all FRAGILE designs + a stratified-by-td
EASY sample of equal size, 7 seeds each at Kp=2/Kd=1, full physics.

Saves:  experiments/results/flight_sig_final_py.csv
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

CSV = Path('experiments/results/exp1_final_population_py.csv')
OUT = Path('experiments/results/flight_sig_final_py.csv')


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
        'rocket_id':  row_dict['rocket_id'],
        'regime':     row_dict['final_label'],
        'is_fragile': int(row_dict['final_label'] == 'FRAGILE'),
        'seed':       seed,
        'rms':        r.rms_error_deg,
        'slew_sat':   r.slew_sat_frac,
        'success':    int(r.success),
        'peak':       r.peak_error_deg,
        'td_max':     row_dict['td'],
        'latency':    row_dict['latency_steps'],
    }


def main():
    print(f"Loading {CSV}")
    df = pd.read_csv(CSV)

    frag = df[df['final_label'] == 'FRAGILE'].copy()
    easy = df[df['final_label'] == 'EASY'].copy()
    n_easy_sample = len(frag)

    print(f"FRAGILE: n={len(frag)},  EASY pool: n={len(easy)}, EASY sample: n={n_easy_sample}")

    easy_sorted = easy.sort_values('td')
    idx = np.round(np.linspace(0, len(easy_sorted) - 1, n_easy_sample)).astype(int)
    easy_sample = easy_sorted.iloc[idx].copy()

    designs = pd.concat([frag, easy_sample], ignore_index=True)
    print(f"Running {len(designs)} designs x {N_SEEDS} seeds = {len(designs) * N_SEEDS} sims")

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

    from sklearn.metrics import roc_auc_score, roc_curve
    per_design = res_df.groupby(['rocket_id', 'regime', 'is_fragile', 'td_max']).agg(
        rms_mean=('rms', 'mean'),
        sr_mean=('success', 'mean'),
    ).reset_index()

    y  = per_design['is_fragile'].values
    s  = per_design['rms_mean'].values
    td = per_design['td_max'].values

    rng = np.random.default_rng(0)
    n = len(y)
    boot = []
    for _ in range(5000):
        idx = rng.integers(0, n, n)
        if len(np.unique(y[idx])) > 1:
            boot.append(roc_auc_score(y[idx], s[idx]))
    lo, hi = np.percentile(boot, [2.5, 97.5])

    auc_rms = roc_auc_score(y, s)
    auc_td  = roc_auc_score(y, td)

    print(f"\n=== FLIGHT SIGNATURE DETECTION (FINAL: n={len(frag)} FRAGILE, n={n_easy_sample} EASY) ===")
    print(f"AUC(RMS, 7-seed):       {auc_rms:.3f} [{lo:.3f}, {hi:.3f}]")
    print(f"AUC(theta_ddot alone):  {auc_td:.3f}")
    print(f"AUC improvement (flight vs spec): {auc_rms - auc_td:+.3f}")

    frag_rms = per_design.loc[per_design['is_fragile']==1, 'rms_mean']
    easy_rms = per_design.loc[per_design['is_fragile']==0, 'rms_mean']
    print(f"\nClass separation at Kp=2 (7-seed means):")
    print(f"  FRAGILE RMS: {frag_rms.mean():.1f} +/- {frag_rms.std():.1f} deg  (n={len(frag_rms)})")
    print(f"  EASY    RMS: {easy_rms.mean():.1f} +/- {easy_rms.std():.1f} deg  (n={len(easy_rms)})")
    print(f"  Ratio: {frag_rms.mean()/easy_rms.mean():.2f}x")

    fpr, tpr, thresholds = roc_curve(y, s)
    j_scores = tpr - fpr
    j_idx    = np.argmax(j_scores)
    youden_t = thresholds[j_idx]

    print(f"\nThreshold analysis (7-seed mean RMS):")
    for thresh in sorted(set([6.0, 7.6, 9.0, 11.0, round(float(youden_t), 1)])):
        pred = (s >= thresh).astype(int)
        tp = int(np.sum((pred==1) & (y==1)))
        fp = int(np.sum((pred==1) & (y==0)))
        fn = int(np.sum((pred==0) & (y==1)))
        tn = int(np.sum((pred==0) & (y==0)))
        prec = tp/(tp+fp) if (tp+fp)>0 else 0.0
        rec  = tp/(tp+fn) if (tp+fn)>0 else 0.0
        f1   = 2*prec*rec/(prec+rec) if (prec+rec)>0 else 0.0
        marker = " <- Youden-J" if abs(thresh - youden_t) < 0.05 else ""
        print(f"  thresh={thresh:.1f}: TP={tp} FP={fp} FN={fn} TN={tn} "
              f"prec={prec:.2f} rec={rec:.2f} F1={f1:.2f}{marker}")

    seed1 = res_df[res_df['seed'] == 1].copy()
    per1  = seed1.groupby(['rocket_id', 'is_fragile']).agg(rms1=('rms','mean')).reset_index()
    auc1  = roc_auc_score(per1['is_fragile'], per1['rms1'])
    print(f"\nAUC(RMS, 1-seed): {auc1:.3f}")

    print(f"\n=== SUMMARY ===")
    print(f"  AUC(7-seed RMS, n={len(frag)} FRAGILE, n={n_easy_sample} EASY): {auc_rms:.3f} [{lo:.3f}, {hi:.3f}]")
    print(f"  AUC(1-seed RMS): {auc1:.3f}")
    print(f"  Youden-J threshold: {youden_t:.1f} deg (TPR={tpr[j_idx]:.2f}, FPR={fpr[j_idx]:.2f})")
    print(f"  FRAGILE mean RMS: {frag_rms.mean():.1f} +/- {frag_rms.std():.1f} deg")
    print(f"  EASY    mean RMS: {easy_rms.mean():.1f} +/- {easy_rms.std():.1f} deg")


if __name__ == '__main__':
    main()
