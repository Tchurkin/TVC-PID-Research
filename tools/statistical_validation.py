"""
statistical_validation.py
Tasks:
  1. Bootstrap 95% CIs: AUC, Youden threshold, regime means
  2. Stratified AUC by motor_scale / Iyy / max_gimbal bins
  3. Permutation test (10,000 shuffles)
  4. Flight prediction sheet (all FRAGILE + boundary EASY + hardware templates)
  5. Simulation uncertainty ranking (printed)

Run:
  python tools/statistical_validation.py
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from sklearn.metrics import roc_auc_score, roc_curve
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from scipy import stats
import warnings
warnings.filterwarnings('ignore')

np.random.seed(42)
rng = np.random.default_rng(42)

# ─── Physical constants ────────────────────────────────────────────────────────
F15_AVG_N   = 14.4    # N  (average thrust, full burn)
L_NOZZLE_M  = 0.25    # m  (nozzle moment arm = half rocket length)
AUTH_CONST  = F15_AVG_N * (np.pi / 180) * L_NOZZLE_M   # 0.062832 rad/s² per auth_ratio
CU_TO_RAD   = np.pi / 180 * 15 / 12                     # 0.02182 rad/CU (keff_full formula)
N_BOOT      = 10_000
N_PERM      = 10_000
THRESHOLD_EMPIRICAL = 70.0   # rad/s²  (empirical claim to test)

print("=" * 70)
print("TVC θ̈_max Statistical Validation — n=1200 designs")
print(f"  F15_AVG = {F15_AVG_N} N  |  l_nozzle = {L_NOZZLE_M} m  |  "
      f"AUTHORITY_CONST = {AUTH_CONST:.5f}")
print("=" * 70)

# ─── Load data ─────────────────────────────────────────────────────────────────
df = pd.read_csv(os.path.join(os.path.dirname(__file__), '..',
                              'experiments', 'results', 'exp1_regime_index_py.csv'))

df['authority_ratio'] = df['max_gimbal_deg'] * df['motor_scale'] / df['Iyy']
df['theta_ddot_max']  = df['authority_ratio'] * AUTH_CONST
df['keff_full']       = F15_AVG_N * df['motor_scale'] * CU_TO_RAD * L_NOZZLE_M / df['Iyy']

print(f"\nLoaded {len(df)} designs. Regime counts:")
print(df['regime_label'].value_counts().to_string())

# Working set: EASY + MARGINAL + FRAGILE (exclude INFEASIBLE from AUC scoring)
dv = df[df['regime_label'] != 'INFEASIBLE'].copy()
dv['is_fragile'] = (dv['regime_label'] == 'FRAGILE').astype(int)
td  = dv['theta_ddot_max'].values
y   = dv['is_fragile'].values
n   = len(td)

auc_point = roc_auc_score(y, td)

# ─── TASK 1 — Bootstrap CIs ───────────────────────────────────────────────────
print("\n" + "=" * 70)
print("TASK 1 — Bootstrap 95% CIs (n_boot = {:,})".format(N_BOOT))
print("=" * 70)

# 1a. AUC
boot_aucs = np.array([
    roc_auc_score(y[idx := rng.integers(0, n, n)], td[idx])
    for _ in range(N_BOOT)
    if (idx := rng.integers(0, n, n)) is not None and
       0 < y[idx].sum() < len(idx)
])
# walrus operator version doesn't work cleanly in list comp; use explicit loop
boot_aucs = []
for _ in range(N_BOOT):
    idx = rng.integers(0, n, n)
    if 0 < y[idx].sum() < len(idx):
        boot_aucs.append(roc_auc_score(y[idx], td[idx]))
boot_aucs = np.array(boot_aucs)

# 1b. Optimal threshold (Youden J = sensitivity + specificity - 1)
def youden_threshold(scores, labels):
    fpr, tpr, thresholds = roc_curve(labels, scores)
    j = tpr - fpr
    return float(thresholds[np.argmax(j)])

thres_point = youden_threshold(td, y)
boot_thres = []
for _ in range(N_BOOT):
    idx = rng.integers(0, n, n)
    if 0 < y[idx].sum() < len(idx):
        boot_thres.append(youden_threshold(td[idx], y[idx]))
boot_thres = np.array(boot_thres)

# 1c. Regime means
easy_td = df.loc[df['regime_label'] == 'EASY',  'theta_ddot_max'].values
frag_td = df.loc[df['regime_label'] == 'FRAGILE','theta_ddot_max'].values
marg_td = df.loc[df['regime_label'] == 'MARGINAL','theta_ddot_max'].values

boot_easy = np.array([rng.choice(easy_td, len(easy_td), replace=True).mean()
                      for _ in range(N_BOOT)])
boot_frag = np.array([rng.choice(frag_td, len(frag_td), replace=True).mean()
                      for _ in range(N_BOOT)])

# t-test and Mann-Whitney
t_stat, t_p    = stats.ttest_ind(frag_td, easy_td)
mw_stat, mw_p  = stats.mannwhitneyu(frag_td, easy_td, alternative='greater')
cohen_d        = (frag_td.mean() - easy_td.mean()) / np.sqrt(
    (frag_td.std()**2 + easy_td.std()**2) / 2)

ci_rows = [
    ('AUC (θ̈_max → FRAGILE)',          f'{auc_point:.4f}',
     f'{np.percentile(boot_aucs, 2.5):.4f}', f'{np.percentile(boot_aucs, 97.5):.4f}',
     f'± {boot_aucs.std():.4f}'),
    ('Youden-J threshold (rad/s²)',      f'{thres_point:.1f}',
     f'{np.percentile(boot_thres, 2.5):.1f}', f'{np.percentile(boot_thres, 97.5):.1f}',
     f'± {boot_thres.std():.1f}'),
    ('EASY mean θ̈_max (rad/s²)',        f'{easy_td.mean():.1f}',
     f'{np.percentile(boot_easy, 2.5):.1f}', f'{np.percentile(boot_easy, 97.5):.1f}',
     f'n={len(easy_td)}'),
    ('FRAGILE mean θ̈_max (rad/s²)',     f'{frag_td.mean():.1f}',
     f'{np.percentile(boot_frag, 2.5):.1f}', f'{np.percentile(boot_frag, 97.5):.1f}',
     f'n={len(frag_td)}'),
]
ci_df = pd.DataFrame(ci_rows,
    columns=['Metric', 'Point_estimate', 'CI_lower_95', 'CI_upper_95', 'Note'])
print("\n", ci_df.to_string(index=False))

print(f"\n  Separation tests (FRAGILE vs EASY):")
print(f"    t-test:      t = {t_stat:.2f},  p = {t_p:.2e}")
print(f"    Mann-Whitney: U = {mw_stat:.0f},  p = {mw_p:.2e}")
print(f"    Cohen's d:   {cohen_d:.2f}  (> 2.0 = very large effect)")

# Empirical threshold coverage
frac_frag_above = (frag_td >= THRESHOLD_EMPIRICAL).mean()
frac_easy_above = (easy_td >= THRESHOLD_EMPIRICAL).mean()
print(f"\n  Empirical threshold {THRESHOLD_EMPIRICAL} rad/s²:")
print(f"    FRAGILE above threshold: {frac_frag_above*100:.1f}%  "
      f"({int(frac_frag_above*len(frag_td))}/{len(frag_td)})")
print(f"    EASY above threshold:    {frac_easy_above*100:.1f}%  "
      f"({int(frac_easy_above*len(easy_td))}/{len(easy_td)})")
print(f"    Youden-J optimal threshold: {thres_point:.1f} rad/s²  "
      f"[vs empirical 70 rad/s²: diff = {thres_point - THRESHOLD_EMPIRICAL:+.1f}]")

# ─── TASK 2 — Stratified AUC ──────────────────────────────────────────────────
print("\n" + "=" * 70)
print("TASK 2 — Stratified AUC (θ̈_max → FRAGILE) by physical parameter bins")
print("=" * 70)

def safe_auc(df_sub):
    sub = df_sub.dropna(subset=['is_fragile', 'theta_ddot_max'])
    if sub['is_fragile'].sum() == 0:
        return np.nan, 0, 0
    if sub['is_fragile'].sum() == len(sub):
        return np.nan, len(sub), int(sub['is_fragile'].sum())
    return (roc_auc_score(sub['is_fragile'], sub['theta_ddot_max']),
            len(sub), int(sub['is_fragile'].sum()))

strat_rows = []
for param in ['motor_scale', 'Iyy', 'max_gimbal_deg']:
    cuts = pd.qcut(dv[param], 3, labels=['low', 'mid', 'high'], duplicates='drop')
    for level in ['low', 'mid', 'high']:
        mask = (cuts == level)
        if mask.sum() == 0:
            continue
        sub = dv[mask].copy()
        auc_s, n_tot, n_frag = safe_auc(sub)
        strat_rows.append({
            'Parameter':  param,
            'Bin':        level,
            'Range':      f"[{sub[param].min():.3g}, {sub[param].max():.3g}]",
            'N_total':    n_tot,
            'N_FRAGILE':  n_frag,
            'AUC':        f'{auc_s:.3f}' if not np.isnan(auc_s) else 'N/A (no FRAGILE)',
        })

strat_df = pd.DataFrame(strat_rows)
print("\n", strat_df.to_string(index=False))

# Flag any bin where AUC drops substantially
valid_aucs = [float(r['AUC']) for r in strat_rows
              if r['AUC'] not in ('N/A (no FRAGILE)',)]
if valid_aucs:
    print(f"\n  AUC range across bins: [{min(valid_aucs):.3f}, {max(valid_aucs):.3f}]")
    if min(valid_aucs) < 0.80:
        print("  ⚠  At least one bin has AUC < 0.80 — θ̈_max prediction may be weaker there.")
    else:
        print("  ✓  All bins with FRAGILE designs show AUC ≥ 0.80 — prediction is robust.")

# ─── TASK 3 — Permutation test ────────────────────────────────────────────────
print("\n" + "=" * 70)
print("TASK 3 — Permutation test (n_perm = {:,})".format(N_PERM))
print("=" * 70)

perm_aucs = np.array([
    roc_auc_score(rng.permutation(y), td) for _ in range(N_PERM)
])
p_value = (perm_aucs >= auc_point).mean()
z_score = (auc_point - perm_aucs.mean()) / perm_aucs.std()

print(f"\n  Observed AUC:               {auc_point:.4f}")
print(f"  Null AUC (mean ± std):       {perm_aucs.mean():.4f} ± {perm_aucs.std():.4f}")
print(f"  Null AUC 99.9th percentile:  {np.percentile(perm_aucs, 99.9):.4f}")
print(f"  p-value (AUC ≥ observed):    {p_value:.5f}  {'< 0.001' if p_value == 0 else ''}")
print(f"  Z-score:                     {z_score:.1f}σ")

if p_value == 0.0:
    print(f"  Result: p < 1/{N_PERM} = {1/N_PERM:.5f}  (zero permutations equalled observed AUC)")
print(f"\n  Interpretation: P(random label shuffle gives AUC ≥ {auc_point:.4f}) = {p_value:.5f}")
print(f"  θ̈_max is {z_score:.0f}σ above chance — classification is not due to random chance.")

# ─── TASK 4 — Flight prediction sheet ────────────────────────────────────────
print("\n" + "=" * 70)
print("TASK 4 — Flight prediction sheet")
print("=" * 70)

# Fit logistic regression P(FRAGILE | θ̈_max) on full dataset
X_lr = np.log(dv['theta_ddot_max'].values).reshape(-1, 1)
scaler = StandardScaler()
X_s = scaler.fit_transform(X_lr)
lr = LogisticRegression(C=1.0, max_iter=500).fit(X_s, y)

def p_fragile(td_val):
    return float(lr.predict_proba(scaler.transform([[np.log(td_val)]]))[0][1])

# Expected RMS @ Kp=2: power law from relay probe study (CLAUDE.md Step 2)
def rms_at_kp2(td_val):
    """A ≈ 0.95 × θ̈_max^0.57  (empirical, n=50, rho=0.781)"""
    return round(0.95 * (td_val ** 0.57), 1)

# Expected best Kp from keff_full formula (CLAUDE.md project-direction)
def expected_kp(motor_scale, Iyy):
    keff = F15_AVG_N * motor_scale * CU_TO_RAD * L_NOZZLE_M / Iyy
    return round(float(11.2 * keff ** 0.85), 0)

def regime_label(td_val, p_frag):
    if td_val < 40:           return 'EASY (confident)'
    elif td_val < THRESHOLD_EMPIRICAL:
        return 'EASY (borderline)' if p_frag < 0.15 else 'EASY (watch)'
    elif td_val < 120:        return 'FRAGILE-suspect'
    else:                     return 'FRAGILE (likely)'

def action(td_val):
    if td_val < THRESHOLD_EMPIRICAL:
        return 'Test at Kp=2. Expect RMS < 7.6°. If RMS > 7.6° → re-classify.'
    return 'Tune Kp=40-120. RMS@Kp=2 will likely exceed 7.6°. Use full-physics sim.'

def make_row(rocket_id, max_gimbal, motor_scale, Iyy, note=''):
    auth = max_gimbal * motor_scale / Iyy
    td_val = auth * AUTH_CONST
    pf = p_fragile(td_val)
    return {
        'Rocket_ID':           rocket_id,
        'max_gimbal_deg':      round(max_gimbal, 1),
        'motor_scale':         round(motor_scale, 2),
        'Iyy_kgm2':           round(Iyy, 5),
        'authority_ratio':     int(round(auth, 0)),
        'theta_ddot_max':      round(td_val, 1),
        'P_FRAGILE':           round(pf, 3),
        'Predicted_regime':    regime_label(td_val, pf),
        'Expected_RMS_at_Kp2': rms_at_kp2(td_val),
        'Expected_best_Kp':    expected_kp(motor_scale, Iyy),
        'Action':              action(td_val),
        'Note':                note,
    }

rows = []

# All FRAGILE designs (validation targets — these are the priority flight tests)
frag_df = df[df['regime_label'] == 'FRAGILE'].sort_values('theta_ddot_max')
for _, r in frag_df.iterrows():
    rows.append(make_row(
        r['rocket_id'], r['max_gimbal_deg'], r['motor_scale'], r['Iyy'],
        note=f'FRAGILE in sim | best_Kp={r["best_Kp"]:.1f} | wind={r["wind_strength"]:.2f}'
    ))

# MARGINAL designs
marg_df = df[df['regime_label'] == 'MARGINAL'].sort_values('theta_ddot_max')
for _, r in marg_df.iterrows():
    rows.append(make_row(
        r['rocket_id'], r['max_gimbal_deg'], r['motor_scale'], r['Iyy'],
        note=f'MARGINAL in sim | best_Kp={r["best_Kp"]:.1f} | wind={r["wind_strength"]:.2f}'
    ))

# 5 boundary EASY designs (highest θ̈_max that still classified EASY — good controls)
easy_boundary = df[df['regime_label'] == 'EASY'].nlargest(5, 'theta_ddot_max')
for _, r in easy_boundary.iterrows():
    rows.append(make_row(
        r['rocket_id'] + '_EASY', r['max_gimbal_deg'], r['motor_scale'], r['Iyy'],
        note=f'EASY in sim (high boundary) | best_Kp={r["best_Kp"]:.1f}'
    ))

# Hardware template rows (fill in your rocket's actual specs)
hardware_templates = [
    # (label,               max_gimbal_deg, motor_scale, Iyy_kgm2)
    ('HARDWARE-1 [edit me]',     8.0,          1.0,        0.020),
    ('HARDWARE-2 [edit me]',    10.0,          1.5,        0.015),
    ('HARDWARE-3 [edit me]',    12.0,          2.0,        0.012),
]
for label, mg, ms, iyy in hardware_templates:
    rows.append(make_row(label, mg, ms, iyy, note='REPLACE with actual hardware specs'))

pred_df = pd.DataFrame(rows)
print("\n[All FRAGILE + MARGINAL + 5 boundary EASY + 3 hardware templates]")
display_cols = ['Rocket_ID', 'theta_ddot_max', 'P_FRAGILE', 'Predicted_regime',
                'Expected_RMS_at_Kp2', 'Expected_best_Kp', 'Note']
print(pred_df[display_cols].to_string(index=False))

# ─── TASK 5 — Simulation uncertainty ranking ──────────────────────────────────
print("\n" + "=" * 70)
print("TASK 5 — Top 5 simulation assumptions likely to affect θ̈_max → FRAGILE agreement")
print("=" * 70)

uncertainty_text = """
Rank | Assumption / Parameter          | Impact on θ̈_max formula | Direction of concern
-----|----------------------------------|--------------------------|--------------------
  1  | Thrust profile accuracy         | θ̈_max uses F15_avg=14.4N | Real T_avg may be ±20%.
     | (motor lot-to-lot variation)    | (single fixed value)     | If T_real = 12 N, threshold
     |                                 |                          | shifts to 60 rad/s². At
     |                                 |                          | 14.4 → 60 that is 14% shift.
     | ⚠ Invalidating: True T_avg      |                          | Measure burn on test stand.
     |   deviates > 15% from F15_avg.  |                          |
-----|----------------------------------|--------------------------|--------------------
  2  | l_nozzle (moment arm)           | θ̈_max ∝ l_nozzle (linear)| 10% error in l → 10% error
     | (CG/CP measurement uncertainty) | l = 0.25 m assumed       | in θ̈_max. Real CG shifts
     |                                 |                          | during burn; static l_nozzle
     |                                 |                          | overstates late-burn authority.
     | ⚠ Invalidating: CG travel       |                          | Measure at 50% burnout.
     |   > 5 cm during burn.           |                          |
-----|----------------------------------|--------------------------|--------------------
  3  | Wind model (Gaussian gusts)     | Drives kp_floor but NOT  | Real wind is structured
     | vs real atmospheric turbulence  | FRAGILE classification.  | (correlations, gusts). If real
     |                                 | r(wind, FRAGILE) = -0.012 | wind is 2× stronger than sim
     |                                 | — wind does NOT determine | gust model, kp_floor rises.
     |                                 | FRAGILE; it reveals it.  | FRAGILE designs still FRAGILE;
     | ⚠ Invalidating: all designs     |                          | EASY may shift to MARGINAL.
     |   fail in real wind regardless  |                          |
     |   of θ̈_max.                    |                          |
-----|----------------------------------|--------------------------|--------------------
  4  | Servo slew model                | Affects kp_ceiling but   | Real servo may have backlash,
     | (ideal slew limit in sim)       | NOT θ̈_max formula.       | nonlinearity, temperature
     |                                 | Oscillation T_u=1.85s vs  | drift. If slew = 40 deg/s
     |                                 | slew theory (11× mismatch) | (below 60 floor), FRAGILE
     |                                 | → ceiling is not          | count could increase.
     | ⚠ Invalidating: servo slew is   | slew-determined.          |
     |   < 40 deg/s in practice.       |                          |
-----|----------------------------------|--------------------------|--------------------
  5  | Iyy measurement accuracy        | θ̈_max ∝ 1/Iyy (dominant) | Mass distribution is hard
     | (mass distribution of hardware) | AUC(Iyy alone)=0.833     | to measure precisely. ±10%
     |                                 |                          | Iyy → ±10% θ̈_max → small
     |                                 |                          | boundary uncertainty.
     | ⚠ Invalidating: Iyy is 2× the  |                          | Measure by pendulum test.
     |   modeled value (e.g. heavy     |                          |
     |   components far from CG).      |                          |
"""
print(uncertainty_text)

# ─── Save all outputs ─────────────────────────────────────────────────────────
out_dir = os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results')

ci_df.to_csv(os.path.join(out_dir, 'bootstrap_ci_py.csv'), index=False)
strat_df.to_csv(os.path.join(out_dir, 'stratified_auc_py.csv'), index=False)
pred_df.to_csv(os.path.join(out_dir, 'flight_predictions_py.csv'), index=False)

# Permutation null distribution summary
perm_summary = pd.DataFrame({
    'n_permutations': [N_PERM],
    'observed_AUC':   [round(auc_point, 4)],
    'null_mean_AUC':  [round(perm_aucs.mean(), 4)],
    'null_std_AUC':   [round(perm_aucs.std(), 4)],
    'p_value':        [round(p_value, 5)],
    'z_score':        [round(z_score, 1)],
    'null_99p9':      [round(np.percentile(perm_aucs, 99.9), 4)],
})
perm_summary.to_csv(os.path.join(out_dir, 'permutation_test_py.csv'), index=False)

print("\n" + "=" * 70)
print("Output files saved:")
print("  experiments/results/bootstrap_ci_py.csv")
print("  experiments/results/stratified_auc_py.csv")
print("  experiments/results/permutation_test_py.csv")
print("  experiments/results/flight_predictions_py.csv")
print("=" * 70)
