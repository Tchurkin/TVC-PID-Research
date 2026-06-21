"""
tools/fidelity_cutoff_by_td.py

Answers the question: which fidelity modules matter at which theta_ddot_max level?

For each of 5 td tiers (<40, 40-70, 70-100, 100-150, >150 rad/s^2), take 5
representative designs from the final population (exp1_final_population_py.csv),
use their stored best_Kp (from the corrected autotune), and evaluate SR with:
  (A) Full physics (baseline)
  (B) Wind disabled
  (C) Slew disabled
  (D) Latency forced to 1 step (minimum hardware latency)
  (E) Sensor noise disabled
  (F) Backlash disabled
  (G) Deadband disabled

Delta_SR = SR_ablated - SR_full for each (design, module) pair.
A large DROP in SR (negative delta) when a module is ablated = that module matters.
A near-zero delta = that module is irrelevant at this td level.

Key expected finding: wind ablation should cause large SR drops only for high-td
designs (where the Kp must be tuned for wind rejection). Below td~55, the gain
window is wide enough that any Kp works regardless of wind.

Seeds: 14001-14010 (fresh, disjoint from all prior experiments).
Output: experiments/results/fidelity_cutoff_by_td_py.csv
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path
from joblib import Parallel, delayed

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from experiment_runner import _run_one, _aggregate
from units import F15_AVG_THRUST_N

N_SEEDS    = 10
SEED_START = 14001
N_PER_TIER = 5  # designs per td tier

TD_TIERS = [
    ('td<40',   0,   40),
    ('td40-70', 40,  70),
    ('td70-100',70, 100),
    ('td100-150',100,150),
    ('td>150',  150, 9999),
]

OUT_CSV = Path('experiments/results/fidelity_cutoff_by_td_py.csv')

# Ablation configs: name -> FidelityConfig kwargs (everything else full)
FULL_CFG = dict(
    wind=True, backlash=True, slew=True, latency=True,
    sensor_noise=True, thrust_var=False, deadband=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
)

ABLATIONS = {
    'full':     dict(FULL_CFG),
    'no_wind':  dict(FULL_CFG, wind=False),
    'no_slew':  dict(FULL_CFG, slew=False),
    'no_noise': dict(FULL_CFG, sensor_noise=False),
    'no_backlash': dict(FULL_CFG, backlash=False),
    'no_deadband': dict(FULL_CFG, deadband=False),
}
LATENCY_ABLATION = 'min_latency'  # handled separately (changes design param)


def _eval_design_ablation(row: dict, Kp: float, Kd: float, ablation_name: str,
                           override_latency: int = None) -> dict:
    design = dict(row)
    if override_latency is not None:
        design['latency_steps'] = override_latency

    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = build_scenario(theta0_bias_std=0.0)

    cfg_kwargs = ABLATIONS.get(ablation_name, FULL_CFG)
    cfg = FidelityConfig(**cfg_kwargs)
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, cfg)

    pid  = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    runs = [_run_one(pid, plant, act, sen, dis, sc, seed=s)
            for s in range(SEED_START, SEED_START + N_SEEDS)]
    agg  = _aggregate(runs)
    return dict(
        rocket_id=row.get('rocket_id', '?'),
        td=row['td'], latency_steps=row['latency_steps'],
        best_Kp=Kp, best_Kd=Kd,
        ablation=ablation_name,
        sr=agg['success_rate'], rms=agg['rms_error_deg'],
        final_label=row.get('final_label', '?'),
    )


def select_designs(pop: pd.DataFrame) -> list[dict]:
    selected = []
    for tier_name, lo, hi in TD_TIERS:
        subset = pop[(pop['td'] >= lo) & (pop['td'] < hi)].copy()
        if len(subset) == 0:
            print(f"  WARNING: no designs in tier {tier_name}")
            continue
        # Stratify: take evenly-spaced by td within tier
        subset = subset.sort_values('td').reset_index(drop=True)
        n = min(N_PER_TIER, len(subset))
        indices = np.linspace(0, len(subset) - 1, n).astype(int)
        for i in indices:
            row = subset.iloc[i].to_dict()
            row['_tier'] = tier_name
            selected.append(row)
        print(f"  {tier_name}: {n} designs, td=[{subset['td'].iloc[0]:.1f}, {subset['td'].iloc[-1]:.1f}]")
    return selected


def main():
    pop = pd.read_csv('experiments/results/exp1_final_population_py.csv')
    pop = pop.dropna(subset=['td', 'best_Kp', 'best_Kd'])

    print("Selecting designs by td tier...")
    designs = select_designs(pop)
    print(f"Total designs: {len(designs)}")

    # Build task list: for each design, run all ablations + latency ablation
    tasks = []
    for d in designs:
        Kp = float(d['best_Kp'])
        Kd = float(d['best_Kd'])
        for abl in ABLATIONS.keys():
            tasks.append((d, Kp, Kd, abl, None))
        # Latency ablation: force latency_steps=1
        tasks.append((d, Kp, Kd, LATENCY_ABLATION, 1))

    total_sims = len(tasks) * N_SEEDS
    print(f"\n{len(tasks)} tasks x {N_SEEDS} seeds = {total_sims} sims")
    print("Running...")

    results = Parallel(n_jobs=-1, verbose=5)(
        delayed(_eval_design_ablation)(d, Kp, Kd, abl, lat_override)
        for d, Kp, Kd, abl, lat_override in tasks
    )
    out = pd.DataFrame(results)
    out.to_csv(OUT_CSV, index=False)
    print(f"\nSaved: {OUT_CSV}")

    # Compute delta_SR relative to full-physics baseline
    full_sr = out[out['ablation'] == 'full'].set_index('rocket_id')['sr'].to_dict()
    out['sr_full_baseline'] = out['rocket_id'].map(full_sr)
    out['delta_sr'] = out['sr'] - out['sr_full_baseline']

    # Summary: mean delta_SR by (td_tier, ablation)
    # Assign tier
    def assign_tier(td):
        for name, lo, hi in TD_TIERS:
            if lo <= td < hi:
                return name
        return 'unknown'

    out['td_tier'] = out['td'].apply(assign_tier)

    print("\n=== MEAN DELTA_SR BY (TD_TIER, ABLATION) ===")
    print("Negative = SR drops when module is removed = module MATTERS")
    print("Near zero = module irrelevant at this td level")
    print()

    ablation_order = ['no_wind', 'no_slew', 'min_latency', 'no_noise', 'no_backlash', 'no_deadband']
    tier_order = [t[0] for t in TD_TIERS]

    pivot = out[out['ablation'] != 'full'].groupby(['td_tier', 'ablation'])['delta_sr'].mean().unstack('ablation')
    # Reorder
    pivot = pivot.reindex(index=[t for t in tier_order if t in pivot.index],
                          columns=[c for c in ablation_order if c in pivot.columns])

    print(pivot.round(3).to_string())
    print()

    # Find cutoff: first tier where each module drops SR by > 0.05
    print("=== CUTOFF SUMMARY (where |delta_sr| first exceeds 0.05) ===")
    for abl in ablation_order:
        if abl not in pivot.columns:
            continue
        cutoff_tier = None
        for tier in tier_order:
            if tier not in pivot.index:
                continue
            if abs(pivot.loc[tier, abl]) > 0.05:
                cutoff_tier = tier
                break
        print(f"  {abl:15s}: first significant impact at {cutoff_tier or 'none in tested range'}")


if __name__ == '__main__':
    main()
