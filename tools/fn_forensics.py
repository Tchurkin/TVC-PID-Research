"""
tools/fn_forensics.py

6-step forensic investigation: WHY are R0804, R0047, R0680, R0452 FRAGILE
despite theta_ddot_max < 70 rad/s²?

NOT a feature-selection exercise.  Goal = determine mechanism.
"""

import sys, dataclasses, warnings
import numpy as np
import pandas as pd
from pathlib import Path

sys.path.insert(0, 'sim')
from simulator import simulate, PIDParams, PlantParams, ActuatorParams
from fidelity_config import FidelityConfig
from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)

warnings.filterwarnings('ignore')

# ─────────────────────────────────────────────────────────────── constants
F15_AVG_N  = 14.4
L_NOZZLE   = 0.25
CU_TO_RAD  = np.pi / 180 * 15/12  # 0.02182 rad/CU

FN_IDS = ['R0804', 'R0047', 'R0680', 'R0452']
TD_THRESHOLD = 70.0

# ─────────────────────────────────────────────────────────────── load data
df = pd.read_csv('experiments/results/exp1_regime_index_py.csv')
df['keff_full'] = F15_AVG_N * df['motor_scale'] * CU_TO_RAD * L_NOZZLE / df['Iyy']
df['u_max']     = df['max_gimbal_deg'] * 12.0 / 15.0
df['td_max']    = df['keff_full'] * df['u_max']

# ─────────────────────────────────────────────────────────────── helpers
def run_at_kp(row, kp, n_seeds=3):
    """Run full-fidelity simulation at fixed Kp and return mean SR."""
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario()
    pid   = PIDParams(Kp=kp, Kd=float(row['best_Kd']))
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in range(1, n_seeds+1)]
    return np.mean([r.success for r in results])

def run_counterfactual(row, plant_overrides=None, act_overrides=None,
                       kp=None, n_seeds=3):
    """Run with modified plant/actuator params at specified Kp."""
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario()
    if plant_overrides:
        plant = dataclasses.replace(plant, **plant_overrides)
    if act_overrides:
        act = dataclasses.replace(act, **act_overrides)
    if kp is None:
        kp = float(row['best_Kp'])
    pid = PIDParams(Kp=kp, Kd=float(row['best_Kd']))
    results = [simulate(pid, plant, act, sen, dis, sc, seed=s) for s in range(1, n_seeds+1)]
    return np.mean([r.success for r in results])

def keff(motor_scale, Iyy):
    return F15_AVG_N * motor_scale * CU_TO_RAD * L_NOZZLE / Iyy

def td(motor_scale, Iyy, max_gimbal_deg):
    return keff(motor_scale, Iyy) * (max_gimbal_deg * 12/15)

# ─────────────────────────────────────────────────────────────── select FN and neighbors
fn_rows = {rid: df[df['rocket_id']==rid].iloc[0] for rid in FN_IDS}
fn_df   = df[df['rocket_id'].isin(FN_IDS)].set_index('rocket_id')
easy_df = df[df['regime_label']=='EASY']
frag_df = df[df['regime_label']=='FRAGILE']

# ╔══════════════════════════════════════════════════════════════╗
# ║  STEP 1 — FORENSIC COMPARISON                               ║
# ╚══════════════════════════════════════════════════════════════╝
print("=" * 70)
print("STEP 1 — FORENSIC COMPARISON TABLES")
print("=" * 70)

cols_show = ['td_max','keff_full','max_gimbal_deg','motor_scale',
             'Iyy','servo_slew_deg_s','wind_strength',
             'static_margin','best_Kp','nominal_success_rate']

def print_comparison(fn_id, n_neighbors=3):
    fn_row = fn_rows[fn_id]
    fn_td  = fn_row['td_max']
    print(f"\n{'─'*70}")
    print(f"  {fn_id}  td={fn_td:.1f}  keff={fn_row['keff_full']:.2f}"
          f"  max_g={fn_row['max_gimbal_deg']:.1f}°"
          f"  motor={fn_row['motor_scale']:.2f}  Iyy={fn_row['Iyy']:.4f}"
          f"  wind={fn_row['wind_strength']:.3f}  slew={fn_row['servo_slew_deg_s']:.0f}"
          f"  bestKp={fn_row['best_Kp']:.1f}")

    # nearest EASY neighbors in td_max
    easy_sorted = easy_df.copy()
    easy_sorted['_dist'] = (easy_sorted['td_max'] - fn_td).abs()
    near_easy = easy_sorted.nsmallest(n_neighbors, '_dist')

    print(f"\n  [EASY neighbors by td_max]")
    for _, r in near_easy.iterrows():
        print(f"    {r.rocket_id:6s}  td={r.td_max:6.1f}  keff={r.keff_full:5.2f}"
              f"  max_g={r.max_gimbal_deg:5.1f}°"
              f"  motor={r.motor_scale:.2f}  Iyy={r.Iyy:.4f}"
              f"  wind={r.wind_strength:.3f}  slew={r.servo_slew_deg_s:.0f}"
              f"  bestKp={r.best_Kp:.1f}")

    # nearest FRAGILE TP neighbors
    tp_frag = frag_df[~frag_df['rocket_id'].isin(FN_IDS)].copy()
    tp_frag['_dist'] = (tp_frag['td_max'] - fn_td).abs()
    near_frag = tp_frag.nsmallest(n_neighbors, '_dist')

    print(f"\n  [FRAGILE TP neighbors by td_max]")
    for _, r in near_frag.iterrows():
        print(f"    {r.rocket_id:6s}  td={r.td_max:6.1f}  keff={r.keff_full:5.2f}"
              f"  max_g={r.max_gimbal_deg:5.1f}°"
              f"  motor={r.motor_scale:.2f}  Iyy={r.Iyy:.4f}"
              f"  wind={r.wind_strength:.3f}  slew={r.servo_slew_deg_s:.0f}"
              f"  bestKp={r.best_Kp:.1f}")

for fid in FN_IDS:
    print_comparison(fid)

# ╔══════════════════════════════════════════════════════════════╗
# ║  STEP 2 — CEILING MECHANISM AUDIT                           ║
# ╚══════════════════════════════════════════════════════════════╝
print("\n" + "=" * 70)
print("STEP 2 — CEILING MECHANISM AUDIT")
print("Hypothesis: low max_gimbal suppresses θ̈max but NOT small-signal keff")
print("=" * 70)

print(f"\n{'ID':6s}  {'td':6s}  {'keff':5s}  {'u_max':5s}  {'max_g':6s}  "
      f"{'keff/td':6s}  {'td(15deg)':9s}  {'bestKp':6s}  {'ceil_est':8s}")
print("  (keff/td = 1/u_max = proportionality of small-signal vs bang-bang authority)")
print("  (td(15deg) = what td WOULD be at max_gimbal=15, same keff)")

for fid in FN_IDS:
    r = fn_rows[fid]
    ceil_est_lo = r['best_Kp']
    ceil_est_hi = r['best_Kp'] * 2
    td_at_15 = r['keff_full'] * 12.0  # u_max at max_gimbal=15 = 12
    ratio = r['keff_full'] / r['td_max']
    print(f"  {fid}  {r['td_max']:6.1f}  {r['keff_full']:5.2f}  {r['u_max']:5.2f}  "
          f"{r['max_gimbal_deg']:6.2f}  {ratio:.4f}  {td_at_15:9.1f}  "
          f"{r['best_Kp']:6.1f}  ({ceil_est_lo:.0f},{ceil_est_hi:.0f})")

print()
print("  Comparison: EASY designs with similar keff_full (7-9 range):")
easy_similar = easy_df[
    (easy_df['keff_full'] >= 6.5) & (easy_df['keff_full'] <= 10.0)
].copy()
print(f"  n={len(easy_similar)}  "
      f"td: mean={easy_similar.td_max.mean():.1f}  "
      f"bestKp: mean={easy_similar.best_Kp.mean():.1f}  "
      f"max_g: mean={easy_similar.max_gimbal_deg.mean():.1f}°")

# Show what ceiling would be for EASY designs at same keff
# If ceiling ∝ K_u/keff_full, and EASY passes at 2×bestKp, then ceiling > 2×bestKp
# → ceiling_Kp > 2 × bestKp → min ceiling ≈ 2 × bestKp for EASY
# For FN designs: ceiling < 2 × bestKp (over test fails)
# Compute ratio: best_Kp × keff_full (proxy for loop gain at best_Kp)
print()
print("  Loop gain proxy = best_Kp × keff_full:")
for fid in FN_IDS:
    r = fn_rows[fid]
    loop_gain = r['best_Kp'] * r['keff_full']
    print(f"    {fid}: {r['best_Kp']:.1f} × {r['keff_full']:.2f} = {loop_gain:.0f}")
easy_lg = easy_df['best_Kp'] * easy_df['keff_full']
print(f"  EASY:    mean loop gain = {easy_lg.mean():.0f}  [p95={easy_lg.quantile(0.95):.0f}]")
frag_tp = frag_df[~frag_df['rocket_id'].isin(FN_IDS)]
frag_lg = frag_tp['best_Kp'] * frag_tp['keff_full']
print(f"  FRAGILE TP: mean loop gain = {frag_lg.mean():.0f}  [p05={frag_lg.quantile(0.05):.0f}]")

# ╔══════════════════════════════════════════════════════════════╗
# ║  STEP 3 — WINDOW TOPOLOGY (KP SWEEP)                        ║
# ╚══════════════════════════════════════════════════════════════╝
print("\n" + "=" * 70)
print("STEP 3 — WINDOW TOPOLOGY: Kp sweep for each FN design")
print("Running sims — this will take a moment...")
print("=" * 70)

window_results = {}
for fid in FN_IDS:
    r = fn_rows[fid]
    best_kp = r['best_Kp']
    kp_list = sorted(set([
        round(best_kp * f, 1) for f in [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5]
    ]))
    n_seeds = 5
    print(f"\n  {fid} (best_Kp={best_kp:.1f}, td={r['td_max']:.1f}, keff={r['keff_full']:.2f}):")
    sweep = []
    for kp in kp_list:
        sr = run_at_kp(r, kp, n_seeds)
        label = ""
        if abs(kp - best_kp) < 0.5: label = " ← best_Kp"
        if abs(kp - best_kp * 2) < 1.0: label = " ← over test"
        print(f"    Kp={kp:7.1f}  SR={sr:.2f}{label}")
        sweep.append((kp, sr))
    window_results[fid] = sweep

    # Estimate floor and ceiling from sweep
    passing = [(kp, sr) for kp, sr in sweep if sr >= 0.60]
    if passing:
        floor_est = min(kp for kp, sr in passing)
        ceil_est  = max(kp for kp, sr in passing)
        print(f"  → Window estimate: floor≤{floor_est:.1f}, ceiling≥{ceil_est:.1f}")
        print(f"  → Window width proxy: {ceil_est/max(floor_est,1):.1f}× (best estimate)")

# ╔══════════════════════════════════════════════════════════════╗
# ║  STEP 4 — LOCAL DECISION BOUNDARY (KNN)                     ║
# ╚══════════════════════════════════════════════════════════════╝
print("\n" + "=" * 70)
print("STEP 4 — LOCAL DECISION BOUNDARY (20 nearest neighbors)")
print("=" * 70)

# Normalize feature space: use td_max, keff_full, wind, slew, Iyy, max_gimbal
features = ['td_max','keff_full','max_gimbal_deg','motor_scale',
            'Iyy','servo_slew_deg_s','wind_strength']
df_feat = df[df['regime_label'].isin(['EASY','MARGINAL','FRAGILE'])][['rocket_id','regime_label'] + features].copy()
df_feat['is_fragile'] = (df_feat['regime_label']=='FRAGILE').astype(int)

from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
X_all = scaler.fit_transform(df_feat[features])

for fid in FN_IDS:
    fn_idx = df_feat[df_feat['rocket_id']==fid].index
    if len(fn_idx) == 0:
        continue
    fn_pos = df_feat.index.get_loc(fn_idx[0])
    fn_vec = X_all[fn_pos]

    dists = np.linalg.norm(X_all - fn_vec, axis=1)
    sorted_idx = np.argsort(dists)
    neighbors_idx = sorted_idx[1:21]  # skip self
    neighbor_df = df_feat.iloc[neighbors_idx]

    n_easy  = (neighbor_df['regime_label']=='EASY').sum()
    n_frag  = (neighbor_df['regime_label']=='FRAGILE').sum()
    n_marg  = (neighbor_df['regime_label']=='MARGINAL').sum()
    td_neigh = df.loc[neighbor_df.index, 'td_max']

    print(f"\n  {fid} — 20 nearest neighbors in 7D param space:")
    print(f"    EASY={n_easy}, FRAGILE={n_frag}, MARGINAL={n_marg}")
    print(f"    Neighbor td_max: mean={td_neigh.mean():.1f}  "
          f"range=[{td_neigh.min():.1f}, {td_neigh.max():.1f}]")
    frag_neighbors = neighbor_df[neighbor_df['regime_label']=='FRAGILE']
    if len(frag_neighbors) > 0:
        print(f"    FRAGILE neighbors: {list(frag_neighbors['rocket_id'])}")
    # Distance to closest FRAGILE TP
    frag_tp_mask = (df_feat['regime_label']=='FRAGILE') & (~df_feat['rocket_id'].isin(FN_IDS))
    frag_tp_dists = dists[df_feat[frag_tp_mask].index.map(df_feat.index.get_loc)]
    if len(frag_tp_dists) > 0:
        print(f"    Distance to nearest FRAGILE TP: {frag_tp_dists.min():.2f} (vs nearest EASY: {dists[n_easy>0 and sorted_idx[1]]:.2f})")

# ╔══════════════════════════════════════════════════════════════╗
# ║  STEP 5 — COUNTERFACTUAL PERTURBATIONS                      ║
# ╚══════════════════════════════════════════════════════════════╝
print("\n" + "=" * 70)
print("STEP 5 — COUNTERFACTUAL PERTURBATIONS (±20%)")
print("Testing at: Kp = best_Kp (nominal) and Kp = best_Kp × 2 (over test)")
print("Each condition: 3 seeds. SR ≥ 0.67 at both = EASY; SR < 0.67 at 2×Kp = FRAGILE")
print("=" * 70)

# Mechanistic predictions table
print("\n  Parameter change → expected effect on ceiling:")
print("  max_gimbal × 1.2 → td × 1.2,  keff UNCHANGED  → K_u(bang-bang) slightly higher")
print("                                  Prediction: ceiling mainly unchanged (keff dominates)")
print("  motor_scale × 0.8 → td × 0.8, keff × 0.8      → ceiling should rise (1/keff mechanism)")
print("  Iyy × 1.2        → td / 1.2, keff / 1.2        → ceiling should rise (1/keff mechanism)")
print("  servo_slew × 1.2 → NO effect on td or keff      → ceiling unchanged (reject slew mech)")
print()

cf_results = {}
for fid in FN_IDS:
    r = fn_rows[fid]
    best_kp = float(r['best_Kp'])
    best_kd = float(r['best_Kd'])
    over_kp = best_kp * 2.0

    print(f"\n  {fid} (best_Kp={best_kp:.0f}, over_kp={over_kp:.0f}):")
    print(f"    {'Perturbation':<30s}  {'td':5s}  {'keff':5s}  {'SR@bestKp':9s}  {'SR@2xKp':7s}  {'status'}")

    design_rows = []

    # baseline
    sr_nom = run_counterfactual(r, kp=best_kp, n_seeds=3)
    sr_ovr = run_counterfactual(r, kp=over_kp, n_seeds=3)
    status = 'FRAGILE' if sr_ovr < 0.67 else 'EASY'
    td_v = td(r['motor_scale'], r['Iyy'], r['max_gimbal_deg'])
    ke_v = keff(r['motor_scale'], r['Iyy'])
    print(f"    {'ORIGINAL':30s}  {td_v:5.1f}  {ke_v:5.2f}  {sr_nom:.2f}       {sr_ovr:.2f}     {status}")

    for factor in [0.8, 1.2]:
        # max_gimbal perturbation (keff unchanged, u_max changes)
        new_mg = r['max_gimbal_deg'] * factor
        new_umax = new_mg * 12 / 15
        new_slew = float(r['servo_slew_deg_s']) * 12/15 * (1/factor)  # keep physical slew, adjust CU
        # Actually: slew_max in CU/s. If max_gimbal changes, u_max changes, but physical deg/s stays same.
        # slew_max = servo_slew_deg_s × 12/15 (CU/s). It doesn't depend on max_gimbal.
        # So slew_max stays the same. Only u_max changes.
        p_ov = {'max_gimbal_deg': new_mg}
        a_ov = {'u_max': new_umax}
        sr_n = run_counterfactual(r, plant_overrides=p_ov, act_overrides=a_ov, kp=best_kp, n_seeds=3)
        sr_o = run_counterfactual(r, plant_overrides=p_ov, act_overrides=a_ov, kp=over_kp, n_seeds=3)
        status = 'FRAGILE' if sr_o < 0.67 else 'EASY'
        td_v = td(r['motor_scale'], r['Iyy'], new_mg)
        ke_v = keff(r['motor_scale'], r['Iyy'])  # unchanged
        print(f"    {'max_gimbal ×'+str(factor):30s}  {td_v:5.1f}  {ke_v:5.2f}  {sr_n:.2f}       {sr_o:.2f}     {status}")

        # motor_scale perturbation (keff and td both scale with motor_scale)
        new_ms = r['motor_scale'] * factor
        p_ov = {'motor_scale': new_ms}
        sr_n = run_counterfactual(r, plant_overrides=p_ov, kp=best_kp, n_seeds=3)
        sr_o = run_counterfactual(r, plant_overrides=p_ov, kp=over_kp, n_seeds=3)
        status = 'FRAGILE' if sr_o < 0.67 else 'EASY'
        td_v = td(new_ms, r['Iyy'], r['max_gimbal_deg'])
        ke_v = keff(new_ms, r['Iyy'])
        print(f"    {'motor_scale ×'+str(factor):30s}  {td_v:5.1f}  {ke_v:5.2f}  {sr_n:.2f}       {sr_o:.2f}     {status}")

        # Iyy perturbation (keff and td both scale inversely with Iyy)
        new_Iyy = r['Iyy'] * factor
        p_ov = {'Iyy_kgm2': new_Iyy}
        sr_n = run_counterfactual(r, plant_overrides=p_ov, kp=best_kp, n_seeds=3)
        sr_o = run_counterfactual(r, plant_overrides=p_ov, kp=over_kp, n_seeds=3)
        status = 'FRAGILE' if sr_o < 0.67 else 'EASY'
        td_v = td(r['motor_scale'], new_Iyy, r['max_gimbal_deg'])
        ke_v = keff(r['motor_scale'], new_Iyy)
        print(f"    {'Iyy ×'+str(factor):30s}  {td_v:5.1f}  {ke_v:5.2f}  {sr_n:.2f}       {sr_o:.2f}     {status}")

        # servo_slew perturbation (no effect on td or keff)
        new_slew = float(r['servo_slew_deg_s']) * factor
        new_slew_cu = new_slew * 12/15  # convert to CU/s
        a_ov = {'slew_max': new_slew_cu}
        sr_n = run_counterfactual(r, act_overrides=a_ov, kp=best_kp, n_seeds=3)
        sr_o = run_counterfactual(r, act_overrides=a_ov, kp=over_kp, n_seeds=3)
        status = 'FRAGILE' if sr_o < 0.67 else 'EASY'
        td_v = td(r['motor_scale'], r['Iyy'], r['max_gimbal_deg'])  # unchanged
        ke_v = keff(r['motor_scale'], r['Iyy'])  # unchanged
        print(f"    {'servo_slew ×'+str(factor):30s}  {td_v:5.1f}  {ke_v:5.2f}  {sr_n:.2f}       {sr_o:.2f}     {status}")

    cf_results[fid] = True

# ╔══════════════════════════════════════════════════════════════╗
# ║  STEP 6 — SCIENTIFIC CONCLUSION                             ║
# ╚══════════════════════════════════════════════════════════════╝
print("\n" + "=" * 70)
print("STEP 6 — SYNTHESIS (populated after Steps 1-5)")
print("=" * 70)

print("""
Mechanistic key:
  A)  max_gimbal changes (keff unchanged) → tests theta_ddot-as-ceiling-driver
  B)  motor_scale or Iyy changes (both keff and td change) → tests keff-as-ceiling-driver
  C)  servo_slew changes (neither keff nor td change) → tests slew mechanism

If (A) changing max_gimbal ±20% has NO ceiling effect but (B) changing motor_scale/Iyy DOES:
  → keff_full drives the ceiling, NOT theta_ddot_max
  → False negatives are due to small max_gimbal suppressing theta_ddot WITHOUT reducing keff
  → Conclusion (C): false negatives reveal a REFINEMENT of theta_ddot
    (theta_ddot is the proxy; keff_full is the true mechanism variable)

If (A) changing max_gimbal HAS ceiling effect comparable to (B):
  → theta_ddot_max drives the ceiling (total authority, not per-CU sensitivity)
  → False negatives are statistical edge cases near the probabilistic boundary
  → Conclusion (A): false negatives are edge cases

If servo_slew changes move the ceiling:
  → Slew mechanism is real for these designs (contradicts the relay period finding)
  → Would need further investigation (Conclusion D)

Noted: for R0680 and R0452 (td near 70), any mechanism that raises td by 12% would
cross the predictor threshold.  The counterfactual isolates whether the SIMULATION
agrees with the predictor's expected behavior.
""")
print("=" * 70)
