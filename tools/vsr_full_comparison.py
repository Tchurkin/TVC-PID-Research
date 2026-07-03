"""
tools/vsr_full_comparison.py  (2026-06-27)

Full head-to-head of four control mechanisms on ONE airframe with a MATCHED control law
(see tools/vsr_lab.py). Sweeps maneuver size and aerodynamic regime (linear vs stalling),
multi-seed, and reports the metrics that decide the engineering question:

  - success rate, reach time, settle time, overshoot   (performance)
  - peak angle of attack                                (how hard the maneuver pushes)
  - control effort, TVC duty                            (cost / dependence on thrust vectoring)

Headline question this answers: in benign (linear) air the actuators are comparable; the
DECISIVE difference appears under STALL, where deflection surfaces saturate (local angle
alpha+delta exceeds stall) but margin modulation (delta=0) keeps the surface unstalled.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from vsr_lab import run

MODES = ["tvc", "canard", "margin", "hybrid"]
LABEL = {"tvc": "TVC (gimbal)", "canard": "canard deflection",
         "margin": "MARGIN modulation", "hybrid": "hybrid (fling+damp)"}


def cell(mode, man, stall, seeds=range(12)):
    R = [run(mode, man_deg=man, stall_deg=stall, seed=s).metrics for s in seeds]
    def mo(k):
        v = [r[k] for r in R if r.get(k) is not None]
        return np.mean(v) if v else None
    return dict(succ=np.mean([r["success"] for r in R]),
                div=np.mean([r["diverged"] for r in R]),
                reach=mo("reach"), settle=mo("settle"), over=mo("overshoot"),
                aoa=mo("peak_aoa"), eff=mo("effort"), tvc=mo("tvc_duty"))


def fmt(x, p=2):
    return "  -  " if x is None else f"{x:.{p}f}"


def block(man, stall, tag):
    print(f"\n=== {tag}:  {man:.0f}deg maneuver, stall={'OFF' if stall is None else str(stall)+'deg'}, 12 seeds ===")
    print(f"  {'controller':20} {'succ':>5} {'div':>5} {'reach':>6} {'settle':>7} {'over':>6} {'peakAoA':>8} {'effort':>7} {'TVCduty':>8}")
    for mode in MODES:
        c = cell(mode, man, stall)
        print(f"  {LABEL[mode]:20} {c['succ']:5.2f} {c['div']:5.2f} {fmt(c['reach']):>6} {fmt(c['settle']):>7} "
              f"{fmt(c['over'],1):>6} {fmt(c['aoa'],1):>8} {fmt(c['eff'],1):>7} {fmt(c['tvc']):>8}")


def main():
    print("############ VARIABLE-STABILITY ROCKET: FULL CONTROL COMPARISON ############")
    print("One airframe (aft fins + forward canards neutral at half-deploy), matched FL control law.")
    # benign air -- gentle and aggressive maneuvers
    block(30, None, "BENIGN AIR")
    block(45, None, "BENIGN AIR, aggressive")
    # stalling air -- the decisive regime
    block(30, 15.0, "STALLING AIR (fin stall @15deg)")
    block(45, 12.0, "STALLING AIR, aggressive (stall @12deg)")
    print("\nREAD:")
    print(" - Benign air: TVC / canard / margin all comparable; canard deflection often the cleanest.")
    print(" - Stalling air: deflection control degrades (large delta -> canard stalls); MARGIN modulation")
    print("   stays robust (delta=0 keeps the surface unstalled). That regime split is the real result.")
    print(" - hybrid is experimental (fling/catch timing not yet tuned) -- shown for completeness.")


if __name__ == "__main__":
    main()
