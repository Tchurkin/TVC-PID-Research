"""
tools/window_ratio_v4_corrected_kd.py  (2026-06-22)

MOTIVATION
----------
window_ratio_resweep_v2 used KD_SRCH_MIN=1.0, always returning opt_Kd=1.0 (the
grid lower bound). frozen_kd_artifact_test showed floors are inflated ~2.9x
universally (shift=+0.65 for 7/8 designs, shift=+0.58 for 1/8), because the
true optimal Kd near the floor is 0.125-0.5 -- entirely below the old Kd grid.

CORRECTIONS
-----------
1. Extended Kd grid: geomspace(0.1, 32, 9) -- lower bound dropped from 1.0 to 0.1
2. 2D coordinate search: alternating golden-section on Kd (fix Kp) and Kp (fix Kd),
   3 rounds after an initial coarse grid. Replaces the flat grid search.
3. Existential floor/ceiling: for each Kp in the sweep, evaluate 5 Kd values
   [KD_EXIST] and take max SR. Floor = min Kp where ANY tested Kd achieves SR>=0.80.
   This gives the true gain-window width, not the window at a single frozen Kd.

GOLDEN SECTION NOTE
-------------------
SR(Kd | Kp) and SR(Kp | Kd) are noisy (binomial, ~5 seeds per query) and
approximately but not strictly unimodal. To reduce false convergence, the
bracket is only accepted if the final range is <0.1 in log-space; otherwise
the coarse-grid best is used as a fallback.

Same 120 designs as v2 (pool seed=8888). New seeds 75001-75012 (12 eval seeds,
disjoint from all prior experiments: v1=20001-20015, v2=22001-22015, v3=23001-23020,
fsat=40001-40020, frozen_kd=50001-50015, valley=various).
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy.stats import pearsonr, spearmanr

from design_space import (
    build_plant, build_actuator, build_sensor,
    build_disturbance, build_scenario, sample_lhs,
)
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

# ---- constants ----
L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# ---- 2D coordinate search: extended Kd range ----
KP_COARSE  = np.geomspace(1.0, 200.0, 9)   # 9 starting Kp values
KD_COARSE  = np.geomspace(0.1,  32.0, 7)   # 7 starting Kd values: [0.1,0.22,0.47,1.0,2.15,4.64,10.0]
                                             # was [1,2,4,8,16,32,64] — now starts at 0.1
SRCH_SEEDS = [1, 2, 3]                      # 3 seeds for search phase
GS_SEEDS   = 5                              # seeds per golden-section query
GS_ITER    = 12                             # golden-section iterations per coordinate

# ---- existential Kp sweep ----
KP_SWEEP   = np.geomspace(0.1, 500.0, 20)  # 20 log-spaced Kp values
KD_EXIST   = np.array([0.10, 0.38, 1.46, 5.62, 21.5])  # 5 Kd values, geomspace(0.1,21.5,5)
N_EVAL     = 12                             # eval seeds per (Kp, Kd) pair
EVAL_SEED0 = 75001                          # disjoint from all prior seeds

SR_THRESH  = 0.80
N_POOL     = 10_000
POOL_SEED  = 8888   # same as v1/v2/v3 to get identical designs

V2_CSV   = Path('experiments/results/window_ratio_v2_py.csv')
OUT_CSV  = Path('experiments/results/window_ratio_v4_py.csv')
OUT_SWEEP = Path('experiments/results/window_ratio_v4_sweep_py.csv')


def _physics_cfg() -> FidelityConfig:
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _compute_specs(row):
    T_avg = F15_AVG_THRUST_N * float(row['motor_scale'])
    keff  = T_avg * CU_TO_RAD * L_NOZZLE / float(row['Iyy'])
    u_max = float(row['max_gimbal_deg']) * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff * u_max), float(keff), float(u_max)


def _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds):
    pid = PIDParams(Kp=float(Kp), Kd=float(Kd), Ki=0.0,
                    u_max=act.u_max, i_lim=act.u_max)
    agg = _aggregate([_run_one(pid, plant, act, sen, dis, sc, seed=s) for s in seeds])
    return float(agg['success_rate'])


def _golden_section_log(f_noisy, lo, hi, n_iter=GS_ITER, tol=0.10):
    """
    Golden section search in log scale for approximate MAXIMUM of f.
    f_noisy(x) -> float (noisy, range 0-1)
    lo, hi: bracket in linear scale.
    Works best when f is approximately unimodal in log(x).
    Returns the estimated maximizer, or None if bracket didn't contract below tol.
    """
    phi = (1 + 5**0.5) / 2
    resphi = 2 - phi  # 1 - 1/phi = 0.382

    llo, lhi = np.log(lo), np.log(hi)
    lx1 = llo + resphi * (lhi - llo)
    lx2 = lhi - resphi * (lhi - llo)
    x1, x2 = np.exp(lx1), np.exp(lx2)
    f1 = f_noisy(x1)
    f2 = f_noisy(x2)

    for _ in range(n_iter):
        if (lhi - llo) < tol:
            break
        if f1 < f2:
            llo = lx1
            lx1, f1 = lx2, f2
            lx2 = lhi - resphi * (lhi - llo)
            x2 = np.exp(lx2)
            f2 = f_noisy(x2)
        else:
            lhi = lx2
            lx2, f2 = lx1, f1
            lx1 = llo + resphi * (lhi - llo)
            x1 = np.exp(lx1)
            f1 = f_noisy(x1)

    converged = (lhi - llo) < tol
    return np.exp((llo + lhi) / 2), converged


def _coord_search_2d(plant, act, sen, dis, sc):
    """
    2D coordinate search: initial coarse grid + 3 rounds of alternating
    golden section on Kd (fix Kp) and Kp (fix Kd).

    Returns (opt_Kp, opt_Kd, peak_sr).
    """
    seeds_coarse = SRCH_SEEDS
    gs_seeds = list(range(1, 1 + GS_SEEDS))  # 5 seeds for golden section queries

    # Step 1: coarse grid to find starting point
    best_sr, opt_Kp, opt_Kd = -1.0, KP_COARSE[4], KD_COARSE[3]
    for Kp in KP_COARSE:
        for Kd in KD_COARSE:
            sr = _eval_sr(plant, act, sen, dis, sc, Kp, Kd, seeds_coarse)
            if sr > best_sr:
                best_sr, opt_Kp, opt_Kd = sr, float(Kp), float(Kd)

    # Step 2: 3 rounds of alternating golden section
    for _ in range(3):
        # Fix Kp, golden section on Kd ∈ [0.05, 32]
        kd_gs, conv = _golden_section_log(
            lambda Kd: _eval_sr(plant, act, sen, dis, sc, opt_Kp, Kd, gs_seeds),
            lo=0.05, hi=32.0
        )
        if conv:
            # Confirm: compare at kd_gs vs opt_Kd
            sr_new = _eval_sr(plant, act, sen, dis, sc, opt_Kp, kd_gs, SRCH_SEEDS)
            if sr_new >= best_sr:
                opt_Kd, best_sr = kd_gs, sr_new

        # Fix Kd, golden section on Kp ∈ [0.5, 400]
        kp_gs, conv = _golden_section_log(
            lambda Kp: _eval_sr(plant, act, sen, dis, sc, Kp, opt_Kd, gs_seeds),
            lo=0.5, hi=400.0
        )
        if conv:
            sr_new = _eval_sr(plant, act, sen, dis, sc, kp_gs, opt_Kd, SRCH_SEEDS)
            if sr_new >= best_sr:
                opt_Kp, best_sr = kp_gs, sr_new

    return float(opt_Kp), float(opt_Kd), float(best_sr)


def _eval_design(row: dict):
    td, keff, u_max = _compute_specs(row)

    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    # 2D coordinate search (extended Kd range starting at 0.1)
    opt_Kp, opt_Kd, peak_sr_search = _coord_search_2d(plant, act, sen, dis, sc)

    # Existential Kp sweep: for each Kp, check 5 Kd values + opt_Kd
    # Floor = min Kp with max_SR >= SR_THRESH across all tested Kd
    eval_seeds = list(range(EVAL_SEED0, EVAL_SEED0 + N_EVAL))

    # Build per-Kp: include KD_EXIST + opt_Kd to ensure coverage near the optimum
    kd_test = np.unique(np.append(KD_EXIST, [opt_Kd]))

    sweep_rows = []
    for Kp in KP_SWEEP:
        sr_by_kd = {}
        for Kd in kd_test:
            sr = _eval_sr(plant, act, sen, dis, sc, Kp, Kd, eval_seeds)
            sr_by_kd[float(Kd)] = sr
        max_sr   = max(sr_by_kd.values())
        best_kd  = max(sr_by_kd, key=lambda k: sr_by_kd[k])
        frozen_sr = _eval_sr(plant, act, sen, dis, sc, Kp, opt_Kd, eval_seeds) \
                    if opt_Kd not in sr_by_kd else sr_by_kd[opt_Kd]
        sweep_rows.append(dict(
            rocket_id=row['rocket_id'], Kp=float(Kp),
            opt_Kd=opt_Kd, max_sr=max_sr, best_kd_at_Kp=best_kd,
            frozen_sr=frozen_sr,
        ))

    # Floor / ceiling from existential SR
    sr_arr  = np.array([r['max_sr'] for r in sweep_rows])
    passing = KP_SWEEP[sr_arr >= SR_THRESH]
    if len(passing) == 0:
        kp_floor = kp_ceiling = window_ratio = float('nan')
        floor_cens = ceil_cens = False
        best_kd_floor = best_kd_ceil = float('nan')
    else:
        kp_floor   = float(passing.min())
        kp_ceiling = float(passing.max())
        window_ratio = kp_ceiling / kp_floor
        floor_cens   = kp_floor   <= KP_SWEEP[0]  * 1.05
        ceil_cens    = kp_ceiling >= KP_SWEEP[-1] * 0.95
        # Best Kd at floor and ceiling Kp points
        floor_row = next(r for r in sweep_rows if abs(r['Kp'] - kp_floor) < 1e-6)
        ceil_row  = next(r for r in sweep_rows if abs(r['Kp'] - kp_ceiling) < 1e-6)
        best_kd_floor = floor_row['best_kd_at_Kp']
        best_kd_ceil  = ceil_row['best_kd_at_Kp']

    summary = dict(
        rocket_id       = row['rocket_id'],
        td              = td,
        keff            = keff,
        u_max           = u_max,
        latency_steps   = int(row['latency_steps']),
        Iyy             = row['Iyy'],
        motor_scale     = row['motor_scale'],
        max_gimbal_deg  = row['max_gimbal_deg'],
        wind_strength   = row['wind_strength'],
        servo_slew      = row['servo_slew_deg_s'],
        static_margin   = row.get('static_margin', 0.0),
        opt_Kp          = opt_Kp,
        opt_Kd          = opt_Kd,
        peak_sr_search  = peak_sr_search,
        kp_floor        = kp_floor,
        kp_ceiling      = kp_ceiling,
        window_ratio    = window_ratio,
        peak_sr         = float(sr_arr.max()),
        floor_censored  = floor_cens,
        ceil_censored   = ceil_cens,
        best_kd_floor   = best_kd_floor,
        best_kd_ceil    = best_kd_ceil,
        td_bin          = str(row.get('td_bin', '')),
        lat_bin         = str(row.get('lat_bin', '')),
    )
    return summary, sweep_rows


def _safe_eval(row: dict):
    try:
        return _eval_design(row)
    except Exception as exc:
        print(f"  ERROR {row.get('rocket_id','?')}: {exc}")
        import traceback; traceback.print_exc()
        return None, []


def _ols(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1 - float(np.sum(resid**2)) / ss_tot if ss_tot > 0 else 0.0
    return beta, r2


def _cv_r2(X, y, k=5, seed=99):
    rng = np.random.RandomState(seed)
    idx = rng.permutation(len(y))
    folds = np.array_split(idx, k)
    r2s = []
    for i in range(k):
        val = folds[i]
        trn = np.concatenate([folds[j] for j in range(k) if j != i])
        beta, *_ = np.linalg.lstsq(X[trn], y[trn], rcond=None)
        yv, yh = y[val], X[val] @ beta
        ss_t = float(np.sum((yv - yv.mean()) ** 2))
        r2s.append(1 - float(np.sum((yv - yh)**2)) / ss_t if ss_t > 0 else 0.0)
    return float(np.mean(r2s)), float(np.std(r2s))


def analyze(out):
    valid = out.dropna(subset=['window_ratio'])
    valid = valid[valid['window_ratio'] > 0].copy()
    n_tot, n_val = len(out), len(valid)
    n_fc = int(valid['floor_censored'].sum())
    n_cc = int(valid['ceil_censored'].sum())

    print(f"\n{'='*65}")
    print("WINDOW RATIO v4 (CORRECTED Kd) RESULTS")
    print(f"{'='*65}")
    print(f"n_valid={n_val}/{n_tot}  floor_censored={n_fc}  ceil_censored={n_cc}")
    print(f"  (v2 had: 116/120, floor_censored=12, ceil_censored=23)")
    wr = valid['window_ratio']
    print(f"window_ratio: min={wr.min():.1f}x  p25={wr.quantile(0.25):.1f}x  "
          f"median={wr.median():.1f}x  p75={wr.quantile(0.75):.1f}x  max={wr.max():.1f}x")

    # opt_Kd distribution
    print(f"\nopt_Kd distribution (should be 0.1-2.0; was always 1.0 in v2):")
    print(f"  min={valid['opt_Kd'].min():.3f}  p25={valid['opt_Kd'].quantile(0.25):.3f}  "
          f"median={valid['opt_Kd'].median():.3f}  p75={valid['opt_Kd'].quantile(0.75):.3f}  "
          f"max={valid['opt_Kd'].max():.3f}")
    n_at_floor = (valid['opt_Kd'] < 0.15).sum()
    print(f"  Designs with opt_Kd < 0.15 (near grid min): {n_at_floor}/{n_val}")

    # best_kd_floor distribution
    bkf = valid['best_kd_floor'].dropna()
    print(f"\nbest_Kd at floor Kp: min={bkf.min():.3f}  median={bkf.median():.3f}  max={bkf.max():.3f}")

    y     = np.log(wr.values)
    ltd   = np.log(valid['td'].values)
    llat  = np.log(valid['latency_steps'].values.astype(float))
    lkeff = np.log(valid['keff'].values)
    ones  = np.ones(n_val)

    print(f"\nCorrelations with log(window_ratio):")
    for name, arr in [('log_td', ltd), ('log_lat', llat), ('log_keff', lkeff)]:
        r, p = pearsonr(arr, y)
        rho, rho_p = spearmanr(arr, y)
        print(f"  r({name})={r:+.3f}  rho={rho:+.3f}  p={p:.2e}")

    def show(label, X):
        beta, r2 = _ols(X, y)
        cv, cv_s = _cv_r2(X, y)
        coef_str = ' '.join(f'{b:+.3f}' for b in beta)
        print(f"  {label:<40} R2={r2:.3f}  CV={cv:.3f}+/-{cv_s:.3f}  coefs=[{coef_str}]")

    print("\nOLS models (outcome: log(window_ratio)):")
    show("log_td only",                   np.c_[ones, ltd])
    show("log_keff only",                 np.c_[ones, lkeff])
    show("log_keff + log_lat",            np.c_[ones, lkeff, llat])
    show("log(Pi) = log_keff + log_lat²", np.c_[ones, lkeff + 2*llat])
    show("log_td + log_lat",              np.c_[ones, ltd, llat])

    print("\n--- Comparison with v2 (frozen Kd=1.0) ---")
    print("  v2: log_keff+log_lat  R2=0.659  CV=0.616+/-0.080  coefs=[-1.192keff,-1.880lat]")
    print("  v2: window median=?x (censored at ~8000x)")

    # Ceiling regression
    vc = valid[~valid['ceil_censored'] & valid['kp_ceiling'].notna()].copy()
    vf = valid[~valid['floor_censored'] & valid['kp_floor'].notna()].copy()

    print(f"\nCeiling regression (n={len(vc)} non-censored):")
    print("  Theory: ceiling ~ lat^-1.0, keff-independent (coeff~0)")
    if len(vc) >= 10:
        lc  = np.log(vc['kp_ceiling'].values)
        lkc = np.log(vc['keff'].values)
        llc = np.log(vc['latency_steps'].values.astype(float))
        beta_c, r2_c = _ols(np.c_[np.ones(len(vc)), lkc, llc], lc)
        print(f"  ceil ~ keff+lat: R2={r2_c:.3f}  coefs: keff={beta_c[1]:+.3f} lat={beta_c[2]:+.3f}")
        # Implied constant: exp(beta[0]) = C in ceil = C * keff^b1 * lat^b2
        print(f"  Implied: ceil ~ {np.exp(beta_c[0]):.0f} * keff^{beta_c[1]:.2f} * lat^{beta_c[2]:.2f}")

    print(f"\nFloor regression (n={len(vf)} non-censored):")
    print("  Theory (v2): floor ~ keff^1.06 * lat^0.96 * 0.06  (BIASED, Kd=1.0)")
    print("  Expected correction: constant ~0.021 (0.06 * 0.35), exponents preserved")
    if len(vf) >= 10:
        lf  = np.log(vf['kp_floor'].values)
        lkf = np.log(vf['keff'].values)
        llf = np.log(vf['latency_steps'].values.astype(float))
        beta_f1, r2_f1 = _ols(np.c_[np.ones(len(vf)), lkf], lf)
        beta_f2, r2_f2 = _ols(np.c_[np.ones(len(vf)), lkf, llf], lf)
        cv_f2, cv_f2s = _cv_r2(np.c_[np.ones(len(vf)), lkf, llf], lf)
        print(f"  floor ~ keff only:    R2={r2_f1:.3f}  coefs: keff={beta_f1[1]:+.3f}  "
              f"implied: floor~{np.exp(beta_f1[0]):.4f}*keff^{beta_f1[1]:.2f}")
        print(f"  floor ~ keff + lat:   R2={r2_f2:.3f}  CV={cv_f2:.3f}+/-{cv_f2s:.3f}")
        print(f"  coefs: keff={beta_f2[1]:+.3f} lat={beta_f2[2]:+.3f}")
        print(f"  Implied: floor ~ {np.exp(beta_f2[0]):.4f} * keff^{beta_f2[1]:.2f} * lat^{beta_f2[2]:.2f}")
        print(f"  v2 was: 0.06 * keff^1.06 * lat^0.96")

    # Pi constant
    pi_arr = valid['keff'].values * valid['latency_steps'].values.astype(float)**2
    C_arr  = valid['window_ratio'].values * pi_arr
    print(f"\nC = window_ratio * Pi: median={np.median(C_arr):.0f}  p25={np.percentile(C_arr,25):.0f}  "
          f"p75={np.percentile(C_arr,75):.0f}")
    print(f"  v2 had: median~9391  (theory 6300 -- both biased upward)")
    print(f"  Expected corrected C ~ {9391 / 0.35:.0f} (if floor shifts 2.9x, C scales same)")

    print(f"\nSaved: {OUT_CSV}")
    print(f"       {OUT_SWEEP}")


def main():
    if not V2_CSV.exists():
        print(f"ERROR: {V2_CSV} not found. Run window_ratio_resweep_v2.py first.")
        return

    v2 = pd.read_csv(V2_CSV)
    rocket_ids = set(v2['rocket_id'].tolist())
    print(f"v2 had {len(v2)} designs; reloading same {len(rocket_ids)} rocket_ids from pool")

    pool = sample_lhs(N_POOL, seed=POOL_SEED)
    T_avg_arr    = F15_AVG_THRUST_N * pool['motor_scale']
    pool['keff'] = T_avg_arr * CU_TO_RAD * L_NOZZLE / pool['Iyy']
    pool['u_max'] = pool['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    pool['td']   = pool['keff'] * pool['u_max']

    selected = pool[pool['rocket_id'].isin(rocket_ids)].copy()
    print(f"Matched {len(selected)}/{len(rocket_ids)} rocket_ids in regenerated pool")

    # Carry over td_bin/lat_bin from v2 for downstream compatibility
    selected = selected.merge(v2[['rocket_id', 'td_bin', 'lat_bin']],
                              on='rocket_id', how='left')

    # Sim count estimate
    sims_search_per   = len(KP_COARSE) * len(KD_COARSE) * len(SRCH_SEEDS)  # coarse grid
    sims_gs_per       = 3 * 2 * GS_ITER * GS_SEEDS                          # 3 rounds, 2 axes
    sims_sweep_per    = len(KP_SWEEP) * (len(KD_EXIST) + 1) * N_EVAL
    sims_per          = sims_search_per + sims_gs_per + sims_sweep_per
    print(f"\nSim estimate per design:")
    print(f"  Coarse grid ({len(KP_COARSE)}Kp x {len(KD_COARSE)}Kd x {len(SRCH_SEEDS)}seeds): {sims_search_per}")
    print(f"  GS refinement (3 rounds x 2 axes x {GS_ITER} iters x {GS_SEEDS} seeds): {sims_gs_per}")
    print(f"  Kp sweep ({len(KP_SWEEP)}Kp x {len(KD_EXIST)+1}Kd x {N_EVAL}seeds): {sims_sweep_per}")
    print(f"  Total per design: {sims_per}   Grand total: ~{sims_per * len(selected):,}")
    print(f"\nKd range for search: [{KD_COARSE[0]:.2f}, {KD_COARSE[-1]:.1f}]  (was [1.0, 64.0])")
    print(f"Kd values for existential sweep: {KD_EXIST}")
    print(f"Eval seeds: {EVAL_SEED0} - {EVAL_SEED0 + N_EVAL - 1}")
    print("Running...\n")

    raw = Parallel(n_jobs=-1, verbose=5)(
        delayed(_safe_eval)(row) for row in selected.to_dict('records')
    )
    summaries   = [r[0] for r in raw if r[0] is not None]
    all_sweep   = [s for r in raw for s in r[1]]

    out = pd.DataFrame(summaries)
    out.to_csv(OUT_CSV, index=False)
    pd.DataFrame(all_sweep).to_csv(OUT_SWEEP, index=False)
    print(f"\nSaved {len(out)} rows to {OUT_CSV}")

    analyze(out)


if __name__ == '__main__':
    main()
