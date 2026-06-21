"""
tools/adrc_ceiling_omega0_sweep.py

Resolves the ω₀-ratio caveat in Section 6.2 of the paper.

The original ADRC bandwidth ceiling sweep (adrc_bandwidth_ceiling.py) fixed
ω₀ = 5×ωc throughout. Three cells at td≈200 rad/s² + latency ∈ {8,10,12}
returned omega_c_max = 0, but the follow-up in Section 6.1 showed that for a
high-latency, high-authority case (td=351, lat=6), lowering ωc to 3 AND raising
ω₀/ωc to ~33 achieved SR=1.00. This script tests the analogous question for the
three remaining zero-ceiling cells: are they genuinely uncontrollable for ADRC,
or just outside the ω₀=5×ωc slice tested before?

Method: same reference design as adrc_bandwidth_ceiling.py (td≈200), same 3
latency values that showed zero ceiling, sweep ωc ∈ {1,2,3,5,7} and
ω₀/ωc ratio ∈ {5,10,20,33} — the latter chosen to include the ~33 ratio that
resolved the lat=6 case in the dissolution follow-up. 10 seeds, full physics.

Also sweeps latency=6 (known to have ceiling=3 at ratio=5) as a sanity check.

Output: experiments/results/adrc_ceiling_omega0_sweep_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario, sample_lhs
from controller import PIDParams, ADRCParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

L_NOZZLE  = 0.25
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX

# Only the zero-ceiling latencies + lat=6 as sanity check
LATENCIES     = [6, 8, 10, 12]
OMEGA_C_GRID  = [1, 2, 3, 5, 7]
OMEGA0_RATIOS = [5, 10, 20, 33]
N_SEEDS       = 10
SEED_START    = 9501   # fresh, disjoint from original sweep (9001-)
TD_TARGET     = 200.0  # only the high-authority reference design

OUT_CSV = Path('experiments/results/adrc_ceiling_omega0_sweep_py.csv')


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def _compute_td(row: dict) -> float:
    T_avg = F15_AVG_THRUST_N * row['motor_scale']
    keff_full = T_avg * CU_TO_RAD * L_NOZZLE / row['Iyy']
    u_max = row['max_gimbal_deg'] * REF_U_MAX / REF_MAX_GIMBAL_DEG
    return float(keff_full * u_max)


def pick_design():
    """Find the reference design closest to td≈200 from the same pool used before."""
    pool = sample_lhs(2000, seed=555)
    pool['td'] = pool.apply(lambda r: _compute_td(r.to_dict()), axis=1)
    idx = (pool['td'] - TD_TARGET).abs().idxmin()
    row = pool.loc[idx].to_dict()
    print(f"Reference design: td={row['td']:.2f} rad/s²  "
          f"Iyy={row['Iyy']:.4f}  motor_scale={row['motor_scale']:.2f}  "
          f"max_gimbal={row['max_gimbal_deg']:.1f}")
    return row


def _eval_cell(design: dict, latency: int, omega_c: float, omega0_ratio: float) -> dict:
    row = dict(design)
    row['latency_steps'] = latency
    plant = build_plant(row)
    act   = build_actuator(row)
    sen   = build_sensor(row)
    dis   = build_disturbance(row)
    sc    = build_scenario(theta0_bias_std=0.0)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

    b0   = float(row['motor_scale']) * F15_AVG_THRUST_N * CU_TO_RAD * L_NOZZLE / float(row['Iyy'])
    adrc = ADRCParams(omega_c=omega_c, omega0=omega0_ratio * omega_c, b0=b0, u_max=act.u_max)
    seeds = list(range(SEED_START, SEED_START + N_SEEDS))
    runs = [simulate(PIDParams(Kp=0.0, Kd=0.0, u_max=act.u_max), plant, act, sen, dis, sc,
                     seed=s, adrc=adrc) for s in seeds]
    sr  = float(np.mean([r.success for r in runs]))
    rms = float(np.mean([r.rms_error_deg for r in runs]))
    return dict(td=row['td'], latency_steps=latency, omega_c=omega_c,
                omega0_ratio=omega0_ratio, omega0=omega0_ratio * omega_c, sr=sr, rms=rms)


def main():
    design = pick_design()

    tasks = [
        (design, lat, wc, ratio)
        for lat   in LATENCIES
        for wc    in OMEGA_C_GRID
        for ratio in OMEGA0_RATIOS
    ]
    total_sims = len(tasks) * N_SEEDS
    print(f"\nGrid: {len(LATENCIES)} latencies x {len(OMEGA_C_GRID)} wc x "
          f"{len(OMEGA0_RATIOS)} ratios = {len(tasks)} cells x {N_SEEDS} seeds = {total_sims} sims")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_cell)(d, lat, wc, ratio) for d, lat, wc, ratio in tasks
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved raw results: {OUT_CSV}")

    # Summary: for each (latency, omega0_ratio), find max wc achieving SR>=0.80
    print("\n=== wc_max achieving SR>=0.80 by (latency, omega0/wc ratio) ===")
    print(f"{'lat':>4}  {'ratio':>6}  {'wc_max':>7}  {'SR@wc_max':>10}  passes?")
    print("-" * 50)
    summary_rows = []
    for lat in LATENCIES:
        for ratio in OMEGA0_RATIOS:
            sub = out[(out['latency_steps'] == lat) & (out['omega0_ratio'] == ratio)]
            passing = sub[sub['sr'] >= 0.80].sort_values('omega_c', ascending=False)
            if len(passing):
                wc_max = float(passing['omega_c'].iloc[0])
                sr_at  = float(passing['sr'].iloc[0])
                status = "PASS"
            else:
                wc_max = 0.0
                sr_at  = float(sub.sort_values('omega_c').iloc[-1]['sr']) if len(sub) else float('nan')
                status = "FAIL (no wc achieves SR>=0.80 in tested grid)"
            print(f"  {lat:>2}  {ratio:>6}  {wc_max:>7.1f}  {sr_at:>10.3f}  {status}")
            summary_rows.append(dict(latency_steps=lat, omega0_ratio=ratio,
                                     omega_c_max=wc_max, sr_at_max=sr_at))

    summary = pd.DataFrame(summary_rows)
    summary_path = 'experiments/results/adrc_ceiling_omega0_summary_py.csv'
    summary.to_csv(summary_path, index=False)
    print(f"\nSaved summary: {summary_path}")

    # Key question: do any of the 3 zero-ceiling cells (lat 8/10/12) become solvable?
    print("\n=== KEY RESULT: zero-ceiling cells (lat in {8,10,12}) ===")
    for lat in [8, 10, 12]:
        sub_lat = summary[summary['latency_steps'] == lat]
        best = sub_lat.loc[sub_lat['omega_c_max'].idxmax()]
        if best['omega_c_max'] > 0:
            print(f"  lat={lat}: SOLVABLE - max wc={best['omega_c_max']:.1f} "
                  f"at omega0/wc={best['omega0_ratio']:.0f} (SR={best['sr_at_max']:.3f})")
        else:
            best_sr = sub_lat.loc[sub_lat['sr_at_max'].idxmax()]
            print(f"  lat={lat}: STILL FAILS in tested grid - "
                  f"best SR={best_sr['sr_at_max']:.3f} at wc grid max, ratio={best_sr['omega0_ratio']:.0f}")

    print("\nConclusion: see above. Compare lat=6 sanity check (should show wc_max=3 at ratio=5).")


if __name__ == '__main__':
    main()
