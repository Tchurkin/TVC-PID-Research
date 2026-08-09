"""
tools/flight_sig_usat_threshold.py

PREP STEP 2 for the retrospective flight-signature test (paper/RETRO_FLIGHT_SIG_SPEC.md).

WHY THIS EXISTS
---------------
The published flight signature's second feature is `slew_sat_frac` -- the fraction of steps where the
actuator RATE limit binds, measured at the simulator's dt = 0.005 s (200 Hz). The archived flight logs
run at 19-20 Hz. A servo slew limit of a few hundred deg/s simply cannot be resolved from 50 ms
samples, so that feature is NOT MEASURABLE on the flight archive and no amount of care makes it so.

The measurable analog is the POSITION clamp, which the simulator records as `u_cmd_sat_frac` and which
the flight logs give directly (rows at +-MAX_TILT). This script re-runs the exact flight-signature
protocol on the exact same designs and seeds, logging BOTH saturation features, and derives the
Youden-optimal threshold for `u_cmd_sat_frac`.

That threshold is then FROZEN and pre-registered, before any flight saturation number is computed.

POSITIVE CONTROL (methodological rule 1 -- non-negotiable)
----------------------------------------------------------
This rerun must reproduce the published signature statistics before its novel threshold means
anything:
    FRAGILE RMS      13.3 +- 5.2 deg      EASY RMS       3.8 +- 2.7 deg
    FRAGILE slew_sat 0.60                 EASY slew_sat  0.14
If it does not reproduce them, the u_sat threshold is NOT usable and the script says so and exits
nonzero. Three void runs were spent in this project learning to write that check first.

Protocol is copied from tools/flight_sig_rerun_final.py @ bb22d36^ and must not drift: all FRAGILE
designs plus a td-stratified EASY sample of equal size, 7 seeds each, Kp=2 / Kd=1, full physics.
"""

import sys, os, warnings
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from simulator import simulate, PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)

# ── Protocol constants — must match flight_sig_rerun_final.py exactly ─────────
KP, KD  = 2.0, 1.0
N_SEEDS = 7
SEEDS   = list(range(1, N_SEEDS + 1))

# Published values this rerun must reproduce (the positive control)
PUB = {'frag_rms': 13.3, 'easy_rms': 3.8, 'frag_slew': 0.60, 'easy_slew': 0.14}
RMS_TOL, SAT_TOL = 1.5, 0.10       # absolute tolerance on the control comparison

SCRATCH = Path(os.environ.get(
    'TVC_SCRATCH',
    r'C:/Users/braxt/AppData/Local/Temp/claude/c--Users-braxt-VS-Code-Projects-TVC-PID-Research'
    r'/72c919c5-79c9-4d79-81cc-308cb546f3f6/scratchpad'))
POP = SCRATCH / 'sim_recover/experiments/results/exp1_final_population_py.csv'
OUT = SCRATCH / 'flight_sig_usat_py.csv'


def run_one(row, seed):
    plant             = build_plant(row)
    act               = build_actuator(row)
    sen               = build_sensor(row)
    dis               = build_disturbance(row)
    sc                = build_scenario()
    fc                = FidelityConfig.full()
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fc)
    pid               = PIDParams(Kp=KP, Kd=KD, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    r                 = simulate(pid, plant, act, sen, dis, sc, seed=seed)
    return {
        'rocket_id':  row['rocket_id'],
        'is_fragile': int(row['final_label'] == 'FRAGILE'),
        'seed':       seed,
        'rms':        r.rms_error_deg,
        'slew_sat':   r.slew_sat_frac,
        'u_sat':      r.u_cmd_sat_frac,      # ← the feature this script exists to measure
        'success':    int(r.success),
    }


def youden(scores, labels):
    """Return (threshold, J, tp, fn, fp, tn) maximising Youden's J."""
    best = None
    for t in np.unique(scores):
        tp = int(((scores >= t) & (labels == 1)).sum())
        fn = int(((scores <  t) & (labels == 1)).sum())
        fp = int(((scores >= t) & (labels == 0)).sum())
        tn = int(((scores <  t) & (labels == 0)).sum())
        J  = tp / max(tp + fn, 1) + tn / max(tn + fp, 1) - 1
        if best is None or J > best[1]:
            best = (float(t), J, tp, fn, fp, tn)
    return best


def main():
    pop = pd.read_csv(POP)

    frag = pop[pop.final_label == 'FRAGILE'].copy()
    easy = pop[pop.final_label == 'EASY'].copy()

    # td-stratified EASY sample of equal size, deterministic (matches the original protocol)
    easy = easy.sort_values('td').reset_index(drop=True)
    idx  = np.linspace(0, len(easy) - 1, len(frag)).round().astype(int)
    samp = pd.concat([frag, easy.iloc[idx]], ignore_index=True)
    print(f"designs: {len(frag)} FRAGILE + {len(idx)} EASY = {len(samp)}, {N_SEEDS} seeds "
          f"= {len(samp) * N_SEEDS} sims")

    if OUT.exists() and '--reuse' in sys.argv:
        df = pd.read_csv(OUT)
        print(f"reusing {OUT}  ({len(df)} rows)")
    else:
        jobs = [(r, s) for r in samp.to_dict('records') for s in SEEDS]
        res  = Parallel(n_jobs=-1, verbose=1)(delayed(run_one)(r, s) for r, s in jobs)
        df   = pd.DataFrame(res)
        OUT.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(OUT, index=False)
        print(f"wrote {OUT}  ({len(df)} rows)")

    g = df.groupby(['rocket_id', 'is_fragile']).agg(
        rms=('rms', 'mean'), slew=('slew_sat', 'mean'), usat=('u_sat', 'mean')).reset_index()
    F, E = g[g.is_fragile == 1], g[g.is_fragile == 0]

    print("\n-- POSITIVE CONTROL " + "-" * 42)
    obs = {'frag_rms': F.rms.mean(), 'easy_rms': E.rms.mean(),
           'frag_slew': F.slew.mean(), 'easy_slew': E.slew.mean()}
    ok = True
    for k, pub in PUB.items():
        tol = RMS_TOL if 'rms' in k else SAT_TOL
        hit = abs(obs[k] - pub) <= tol
        ok &= hit
        print(f"  {k:11s} published {pub:6.2f}   observed {obs[k]:6.2f}   "
              f"|d|={abs(obs[k]-pub):.2f} <= {tol}  {'PASS' if hit else 'FAIL'}")
    if not ok:
        print("\nCONTROL FAILED -- the rerun does not reproduce the published signature.")
        print("The u_sat threshold derived below is NOT usable. Do not freeze it.")
        return 1
    print("  CONTROL PASSED -- the novel threshold below is interpretable.")

    print("\n-- THRESHOLDS (Youden-optimal on the sim, 7-seed means) " + "-" * 6)
    y = g.is_fragile.values
    for name, col in (('rms', 'rms'), ('slew_sat', 'slew'), ('u_sat  <- FREEZE THIS', 'usat')):
        t, J, tp, fn, fp, tn = youden(g[col].values, y)
        print(f"  {name:22s} thresh={t:.4f}  J={J:.3f}   TP{tp} FN{fn} FP{fp} TN{tn}")

    print(f"\n  u_cmd_sat_frac  FRAGILE {F.usat.mean():.3f} +- {F.usat.std():.3f}   "
          f"EASY {E.usat.mean():.3f} +- {E.usat.std():.3f}")
    print("\nRecord the u_sat threshold in paper/RETRO_FLIGHT_SIG_SPEC.md as T_sat, then it is frozen.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
