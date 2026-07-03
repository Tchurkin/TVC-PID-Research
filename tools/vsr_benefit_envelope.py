"""
tools/vsr_benefit_envelope.py  (2026-06-26)

Maps WHERE active variable stability (fling-unstable / catch-stable) actually beats a
fixed-stable airframe, using the full sim_vsr architecture. Sweeps TVC authority (gimbal
u_max) x maneuver size/speed, and for each cell compares fixed_stable vs switched vs
fixed_unstable over 10 wind seeds.

HONEST FINDING (this is a NULL-leaning result, not a hype result):
  - Adequate TVC authority      : switched is NO faster (often slower) than fixed_stable.
  - Authority-limited EDGE only  : switched gives a small, real benefit (higher success,
                                   slightly lower overshoot) -- the unstable fling assists a
                                   weak TVC, the stable catch settles it.
  - Beyond the edge (TVC too weak): switched DIVERGES (the fling overshoots, weak TVC cannot
                                   recover). Variable stability is dangerous here, not helpful.
The contribution is the ENVELOPE itself: a quantified map of the narrow region where the
mechanism helps and the adjacent region where it is actively harmful.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from sim_vsr import (VehicleParams, TVCParams, MorphParams, IMUParams, TVCController,
                     PIDGains, StabilityScheduler, SchedulerCfg, Scenario, simulate)


def cell(vp, mode, u_max, man_deg, ramp, morph_rate=15.0, seeds=range(12)):
    rows = []
    for sd in seeds:
        scn = Scenario(t_end=2.0, dt=0.005, launch_speed=15.0, maneuver_time=0.7,
                       maneuver_deg=man_deg, maneuver_ramp=ramp, wind_sigma=1.5)
        tvc_p = TVCParams(u_max_deg=u_max, slew_max_deg_s=120, deadband_deg=0.2, backlash_deg=0.3)
        ctrl = TVCController("pid", PIDGains(Kp=0.3, Kd=0.06, Ki=0.6), scn.dt)
        sched = StabilityScheduler(SchedulerCfg(mode=mode, sm_stable=0.8, sm_unstable=-0.3,
                                                switch_in_deg=5, switch_out_deg=8))
        rows.append(simulate(vp, tvc_p, MorphParams(rate_cal_s=morph_rate),
                             IMUParams(latency_steps=2), ctrl, sched, scn, seed=sd).metrics)

    def mo(k):
        v = [r[k] for r in rows if r.get(k) is not None]
        return np.mean(v) if v else None
    return dict(succ=np.mean([r['success'] for r in rows]),
                div=np.mean([r['diverged'] for r in rows]),
                settle=mo('settle_time'), over=mo('overshoot_deg'), maxA=mo('max_alpha_deg'))


def main():
    vp = VehicleParams()
    grid = [(7, 20, 0.40), (5, 25, 0.30), (3, 30, 0.25), (2.5, 30, 0.20),
            (2, 35, 0.20), (2, 30, 0.15), (1.5, 35, 0.15)]
    print("=== VARIABLE-STABILITY BENEFIT ENVELOPE (full sim_vsr, 12 seeds/cell) ===")
    print("  verdict per row compares switched vs fixed_stable settle/overshoot/success\n")
    print(f"  {'u_max':>5} {'man':>4} {'ramp':>5} | {'mode':14} {'succ':>5} {'div':>5} {'settle':>7} {'over':>6} {'maxA':>6}   verdict")
    for u_max, man, ramp in grid:
        c = {m: cell(vp, m, u_max, man, ramp) for m in ['fixed_stable', 'switched', 'fixed_unstable']}
        # verdict
        fs, sw = c['fixed_stable'], c['switched']
        if sw['succ'] >= fs['succ'] + 0.1 or (sw['succ'] >= 0.9 and fs['succ'] >= 0.9 and
                                              sw['settle'] is not None and fs['settle'] is not None and
                                              sw['settle'] < 0.9 * fs['settle']):
            verdict = "SWITCHED HELPS"
        elif sw['div'] > fs['div'] + 0.1 or (sw['succ'] < fs['succ'] - 0.1):
            verdict = "switched HARMFUL"
        else:
            verdict = "no benefit"
        for i, m in enumerate(['fixed_stable', 'switched', 'fixed_unstable']):
            cc = c[m]
            def f(x, w=7, p=2):
                return ("{:>%d}" % w).format('-' if x is None else f"{x:.{p}f}")
            tag = ("   " + verdict) if i == 1 else ""
            print(f"  {u_max:>5} {man:>4} {ramp:>5} | {m:14} {cc['succ']:5.2f} {cc['div']:5.2f} "
                  f"{f(cc['settle'])} {f(cc['over'],6,1)} {f(cc['maxA'],6,1)}{tag}")
        print()
    print("READ: 'SWITCHED HELPS' should appear only in a narrow authority-limited band; flanked by")
    print("'no benefit' (TVC adequate) above and 'switched HARMFUL' (TVC too weak -> fling diverges) below.")


if __name__ == "__main__":
    main()
