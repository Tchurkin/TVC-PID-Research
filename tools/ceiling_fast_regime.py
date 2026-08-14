"""
tools/ceiling_fast_regime.py

DOES THE 1/tau GAIN CEILING LAW HOLD BELOW 5 ms?

WHY THIS EXPERIMENT EXISTS
--------------------------
tau was MEASURED on the flight vehicle 2026-08-13 (paper/TAU_MEASUREMENT.md): ~3.2 ms, against the
0.035 s the project had assumed. But the ceiling law `ceiling ~ 0.0661/tau`, exponent -1.067, was
fitted over tau = 5-30 ms only -- `latency_steps` is clipped to [1,6] at dt = 0.005 s
(design_space.py:201), so the simulator's sampled range starts exactly where the vehicle stops being
covered. Quoting the law at 3.2 ms is an EXTRAPOLATION into the one direction never tested, and it is
a large claim: it predicts 15.7-24.3 against 1.9 at the assumed tau.

This measures it instead of extrapolating.

THE PROBLEM WITH JUST LOWERING dt
---------------------------------
tau = latency_steps * dt, so reaching tau < 5 ms means dt < 0.005. But dt is also the integration
step for the plant, actuator lag, slew limiting and noise. If dt itself shifts the measured ceiling,
any "extension" is uninterpretable -- we would be reading a discretisation artifact as physics. That
is precisely the failure mode this project has been bitten by before.

So the design separates dt from tau:

  ARM A  replication   dt = 0.005   latency_steps 1..6      -> tau  5,10,15,20,25,30 ms
  ARM B  dt control    dt = 0.001   latency_steps 5,10..30  -> tau  5,10,15,20,25,30 ms  (SAME tau)
  ARM C  extension     dt = 0.001   latency_steps 1..4      -> tau  1,2,3,4 ms           (NOVEL)

ARM A must reproduce the published exponent, or the harness is wrong.
ARM B must agree with ARM A at matched tau, or dt is confounded and ARM C means nothing.
Only if BOTH controls pass is ARM C interpretable. Both gates are enforced in code below and the
script refuses to report the extension if either fails.

PROTOCOL
--------
Ceiling is measured exactly as tools/ceiling_kd_free.py does it, at the published Kd = 1.0 -- the
"at a fixed Kd" case, which is the operationally relevant one and the one the published law describes.
Designs are keff-stratified so the keff coefficient stays identifiable, and every design is run at
EVERY tau, which makes the tau exponent a within-design contrast rather than a between-design one.
"""

import sys, os, argparse, dataclasses, warnings
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

# ---- protocol constants, matched to tools/ceiling_kd_free.py -------------------------------
KP_GRID    = np.geomspace(1.0, 2000.0, 22)
KD_FIXED   = 1.0                     # the published protocol's Kd
EVAL_SEEDS = tuple(range(320001, 320011))
SR_PASS    = 0.80
PUB_TAU_EXP = -1.067                 # what ARM A must reproduce
CTRL_TOL_A  = 0.25                   # |armA - published| must be within this
CTRL_TOL_B  = 0.20                   # |armB - armA| must be within this

SCRATCH = Path(os.environ.get(
    'TVC_SCRATCH',
    r'C:/Users/braxt/AppData/Local/Temp/claude/c--Users-braxt-VS-Code-Projects-TVC-PID-Research'
    r'/72c919c5-79c9-4d79-81cc-308cb546f3f6/scratchpad'))
POP = SCRATCH / 'sim_recover/experiments/results/exp1_final_population_py.csv'
OUT = SCRATCH / 'ceiling_fast_regime.csv'


def dt_invariance_check(row):
    """PREFLIGHT. Same tau, two timesteps -- does anything but tau change?

    This must pass before the arms below mean anything. It is separated from ARM A/B because it
    isolates the CAUSE: run with sensor noise on and off, and see which one breaks.
    """
    print("-- PREFLIGHT: is the simulator dt-invariant at matched tau? " + "-"*10)
    print("   same design, tau = 5 ms both ways: dt=0.005 x 1 step  vs  dt=0.001 x 5 steps")
    worst = 0.0
    for noise in (False, True):
        for kp in (5.0, 20.0, 60.0):
            out = []
            for dt, ls in ((0.005, 1), (0.001, 5)):
                r = dict(row); r['latency_steps'] = ls
                plant = dataclasses.replace(build_plant(r), dt=dt)
                act = build_actuator(r); sen = build_sensor(r)
                dis = build_disturbance(r); sc = build_scenario()
                fc  = FidelityConfig.full()
                act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fc)
                if not noise:
                    sen = dataclasses.replace(sen, gyro_noise_std=0.0, gyro_bias_rw=0.0,
                                              gyro_bias_init=0.0)
                runs = [simulate(PIDParams(Kp=kp, Kd=KD_FIXED, Ki=0.0, u_max=act.u_max,
                                           i_lim=act.u_max), plant, act, sen, dis, sc, seed=sd)
                        for sd in EVAL_SEEDS[:5]]
                out.append((float(np.mean([r_.success for r_ in runs])),
                            float(np.mean([r_.rms_error_deg for r_ in runs]))))
            ratio = out[1][1] / max(out[0][1], 1e-9)
            if noise: worst = max(worst, max(ratio, 1/max(ratio,1e-9)))
            print(f"   Kp={kp:5.1f}  noise {'ON ' if noise else 'OFF'}  "
                  f"dt.005 SR{out[0][0]:.1f} rms{out[0][1]:6.2f} | "
                  f"dt.001 SR{out[1][0]:.1f} rms{out[1][1]:6.2f} | ratio {ratio:5.2f}")
    ok = worst < 1.6
    print(f"   worst RMS ratio with noise on: {worst:.2f}  -> "
          f"{'dt-INVARIANT' if ok else 'NOT dt-invariant'}")
    if not ok:
        print()
        print("   *** THE EXPERIMENT IS BLOCKED, AND THIS IS WHY. ***")
        print("   Noise OFF: the two timesteps agree, so the plant, actuator and latency pipeline")
        print("   are all dt-invariant. Noise ON: they do not. The cause is sensor_model.py:117,")
        print("   which adds `gyro_noise_std * randn()` once per STEP with no dt scaling -- its own")
        print("   docstring says 'rad/s per sqrt step'. (The bias random walk on line 111 IS scaled,")
        print("   by sqrt(dt), which is why only the white-noise term misbehaves.)")
        print("   So halving dt does not hold the physical noise fixed, and any ceiling measured at")
        print("   a finer dt differs from the published ones for a reason that is NOT tau.")
        print()
        print("   This does NOT invalidate any published result: every one of them used dt = 0.005")
        print("   throughout, so they are internally consistent. It only means the latency axis")
        print("   cannot be EXTENDED by lowering dt until the noise model is given a physical")
        print("   definition -- which is a modelling decision, not a scale factor.")
    return ok


def measure_ceiling(row, dt, latency_steps):
    """Highest Kp on the grid whose success rate over EVAL_SEEDS reaches SR_PASS, at Kd = 1.0."""
    r = dict(row); r['latency_steps'] = latency_steps
    plant = dataclasses.replace(build_plant(r), dt=dt)
    act   = build_actuator(r)
    sen   = build_sensor(r)
    dis   = build_disturbance(r)
    sc    = build_scenario()
    fc    = FidelityConfig.full()
    act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, fc)

    ok = np.zeros(len(KP_GRID), bool)
    for i, kp in enumerate(KP_GRID):
        pid = PIDParams(Kp=float(kp), Kd=KD_FIXED, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
        s = [simulate(pid, plant, act, sen, dis, sc, seed=sd).success for sd in EVAL_SEEDS]
        ok[i] = float(np.mean(s)) >= SR_PASS
    ceiling  = float(KP_GRID[ok][-1]) if ok.any() else np.nan
    censored = int(bool(ok[-1]))          # still passing at the top of the grid
    return dict(rocket_id=r['rocket_id'], dt=dt, latency_steps=latency_steps,
                tau=latency_steps*dt, ceiling=ceiling, censored=censored,
                keff=14.4*float(r['motor_scale'])*0.25/float(r['Iyy']),
                Iyy=float(r['Iyy']), n_pass=int(ok.sum()))


def fit(d, label):
    """log(ceiling) ~ log(keff) + log(tau). Returns (tau_exp, keff_coef, se_tau, n, R2)."""
    d = d[np.isfinite(d.ceiling) & (d.censored == 0)]
    if len(d) < 6:
        return None
    X = np.column_stack([np.ones(len(d)), np.log(d.keff.values), np.log(d.tau.values)])
    y = np.log(d.ceiling.values)
    b, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ b
    dof   = max(1, len(d) - X.shape[1])
    s2    = float(resid @ resid) / dof
    se    = np.sqrt(np.diag(s2 * np.linalg.inv(X.T @ X)))
    r2    = 1 - float(resid @ resid) / float(((y - y.mean())**2).sum())
    print(f"  {label:<34s} n={len(d):3d}  keff {b[1]:+.3f} (SE {se[1]:.3f})   "
          f"tau {b[2]:+.3f} (SE {se[2]:.3f})   R2={r2:.3f}")
    return b[2], b[1], se[2], len(d), r2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--designs', type=int, default=16)
    ap.add_argument('--reuse', action='store_true')
    ap.add_argument('--smoke', action='store_true', help='4 designs, 2 taus per arm')
    a = ap.parse_args()

    pop = pd.read_csv(POP)
    pop = pop[pop.final_label != 'INFEASIBLE'].copy()
    pop['keff'] = 14.4 * pop.motor_scale * 0.25 / pop.Iyy
    # keff-stratified so the keff coefficient stays identifiable
    pop = pop.sort_values('keff').reset_index(drop=True)
    n = 4 if a.smoke else a.designs
    idx = np.linspace(0, len(pop)-1, n).round().astype(int)
    designs = pop.iloc[idx].to_dict('records')

    if a.smoke:
        arms = [('A_repl', 0.005, [1, 3]), ('B_dtctl', 0.001, [5, 15]), ('C_fast', 0.001, [2, 4])]
    else:
        arms = [('A_repl',  0.005, [1, 2, 3, 4, 5, 6]),
                ('B_dtctl', 0.001, [5, 10, 15, 20, 25, 30]),
                ('C_fast',  0.001, [1, 2, 3, 4])]

    if not dt_invariance_check(designs[len(designs)//2]):
        print()
        print("STOPPING before spending compute on an uninterpretable result.")
        print("Section 5 keeps its extrapolation caveat: the ceiling law is fitted over 5-30 ms")
        print("and the measured 3.2 ms vehicle sits outside it. That stands as the honest position.")
        return 1
    print()

    jobs = [(arm, dt, ls, d) for arm, dt, lss in arms for ls in lss for d in designs]
    print(f"designs={len(designs)}  jobs={len(jobs)}  "
          f"({len(jobs)*len(KP_GRID)*len(EVAL_SEEDS)} sims)")

    if a.reuse and OUT.exists():
        df = pd.read_csv(OUT); print(f"reusing {OUT} ({len(df)} rows)")
    else:
        res = Parallel(n_jobs=-1, verbose=5)(
            delayed(measure_ceiling)(d, dt, ls) for arm, dt, ls, d in jobs)
        df = pd.DataFrame(res)
        df['arm'] = [j[0] for j in jobs]
        OUT.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(OUT, index=False)
        print(f"wrote {OUT} ({len(df)} rows)")

    print("\n-- CENSORING (a censored ceiling is a lower bound, excluded from fits) " + "-"*8)
    for arm in df.arm.unique():
        s = df[df.arm == arm]
        print(f"  {arm:<9s} n={len(s):3d}  censored {int(s.censored.sum()):3d}  "
              f"nan {int(s.ceiling.isna().sum()):3d}")

    print("\n-- FITS " + "-"*66)
    A = fit(df[df.arm == 'A_repl'],  'ARM A  replication dt=0.005')
    B = fit(df[df.arm == 'B_dtctl'], 'ARM B  dt control  dt=0.001')
    C = fit(df[df.arm == 'C_fast'],  'ARM C  fast regime dt=0.001')
    BC = fit(df[df.arm.isin(['B_dtctl', 'C_fast'])], 'ARM B+C  extended range')

    print("\n-- CONTROL GATES " + "-"*57)
    ok = True
    if A is None:
        print("  ARM A did not fit -- STOP."); return 1
    dA = abs(A[0] - PUB_TAU_EXP)
    hitA = dA <= CTRL_TOL_A
    ok &= hitA
    print(f"  1. ARM A vs published {PUB_TAU_EXP:+.3f}: got {A[0]:+.3f}, |d|={dA:.3f} "
          f"<= {CTRL_TOL_A}  {'PASS' if hitA else 'FAIL'}")
    if B is None:
        print("  2. ARM B did not fit -- dt control INCONCLUSIVE."); ok = False
    else:
        dB = abs(B[0] - A[0]); hitB = dB <= CTRL_TOL_B; ok &= hitB
        print(f"  2. ARM B vs ARM A at matched tau: {B[0]:+.3f} vs {A[0]:+.3f}, |d|={dB:.3f} "
              f"<= {CTRL_TOL_B}  {'PASS' if hitB else 'FAIL'}")
        print(f"     (this is what separates dt from tau. If it fails, the finer timestep itself")
        print(f"      moves the ceiling and ARM C cannot be read as physics.)")

    print("\n-- VERDICT " + "-"*63)
    if not ok:
        print("  CONTROLS FAILED. The fast-regime result below is NOT interpretable.")
        if C: print(f"  (withheld: ARM C tau exponent {C[0]:+.3f})")
        print("  Do not put this in the paper.")
        return 1
    print("  Both controls PASSED, so the extension is interpretable.")
    if C is None:
        print("  ARM C did not fit -- too few uncensored points in the fast regime.")
        return 1
    print(f"  tau exponent, fitted range   (5-30 ms): {A[0]:+.3f}")
    print(f"  tau exponent, FAST regime    (1-4 ms):  {C[0]:+.3f}  (SE {C[2]:.3f})")
    if BC: print(f"  tau exponent, extended       (1-30 ms): {BC[0]:+.3f}  (SE {BC[2]:.3f})")
    dC = abs(C[0] - A[0])
    print()
    if dC <= 0.30:
        print(f"  The exponent SURVIVES into the fast regime (|d|={dC:.3f}).")
        print("  -> Section 5 may quote the ceiling prediction for the measured vehicle as")
        print("     validated over an extended range, instead of labelling it an extrapolation.")
    else:
        print(f"  The exponent CHANGES in the fast regime (|d|={dC:.3f}).")
        print("  -> That is the more interesting result: the 1/tau law has a lower bound on its")
        print("     scope. Section 5 must state the fitted range and NOT extrapolate to 3.2 ms.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
