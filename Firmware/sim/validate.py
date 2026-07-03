#!/usr/bin/env python3
"""Fast regression guard for the Sysiphus landing SIL. Run after ANY firmware/harness change.

Checks the NOMINAL-plant wind/target envelope still lands soft+on-target+upright (the wind-robustness axis
the sweep covers). It does NOT judge dispersion robustness -- that's montecarlo.py (slower, separate).
Exits non-zero on any regression so it can gate a commit.

Usage:  python validate.py [path\\to\\sim.exe]
"""
import subprocess, csv, io, sys, os, re

_here = os.path.dirname(os.path.abspath(__file__))
SIM = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_here, "sim.exe")
if not os.path.exists(SIM): sys.exit(f"sim.exe not found at {SIM}")

# DISTRIBUTIONAL regression guard (3-D baseline 2026-07-02: 81/100 sweep-pass, vz p50 3.2 / p90 5.4 / max 8.6;
# the ~5-6 m/s tail is one bad sensor-noise draw on the hoverslam knife edge -- known, survivable-on-legs class).
# Per-cell bounds would cry wolf on that known tail, so gate on the DISTRIBUTION: pass-rate + hard worst-case
# caps that a real regression (tumble, lost target, roll runaway) would blow through.
PASS_MIN   = 0.70          # fraction of cells that must meet the full soft+on-target+upright PASS gates
VZ_HARD    = 9.5           # no landing harder than this (a tumble/mistimed burn reads 12-30)
MISS_HARD  = 6.0           # 2-D miss hard cap (m)
TILT_HARD  = 15.0          # total-tilt hard cap (deg)

def run(T, W, K, seed):
    p = subprocess.run([SIM, "--target", str(T), "--wind", str(W), "--kick", str(K),
                        "--gusts", "--seed", str(seed), "--ignjit", "0"], capture_output=True, text=True)
    e = p.stderr
    def g(pat):
        m = re.search(pat, e); return float(m.group(1)) if m else 1e9
    return dict(vz=g(r"touchdown speed : ([\d.]+)"), tilt=abs(g(r"final tilt 3d   : ([\-\d.]+)")),
                miss=abs(g(r"2-D miss ([\-\d.]+)")), passed="PASS" in e)

def bisect_kick(T):
    lo, hi = -8.0, 20.0
    for _ in range(7):
        mid = (lo+hi)/2
        p = subprocess.run([SIM, "--kick", str(mid), "--wind", "0", "--target", str(T),
                            "--nodivert", "--ignjit", "0"], capture_output=True, text=True)
        rows = list(csv.DictReader(io.StringIO(p.stdout)))
        lx = float(rows[-1]["px"]) if rows else 0.0
        lo, hi = (mid, hi) if lx < T else (lo, mid)
    return round((lo+hi)/2, 2)

TARGETS, WINDS, SEEDS = [0, 8, 16, 24], [-4, 0, 4], [1, 2, 3]
res, hard = [], []
print("validating nominal wind/target envelope...")
for T in TARGETS:
    K = bisect_kick(T)
    for W in WINDS:
        for s in SEEDS:
            o = run(T, W, K, s); o.update(T=T, W=W, s=s); res.append(o)
            if o["vz"] > VZ_HARD or o["miss"] > MISS_HARD or o["tilt"] > TILT_HARD: hard.append(o)
    print(f"  target {T:>2}: done")

n = len(res); npass = sum(1 for o in res if o["passed"])
vz = sorted(o["vz"] for o in res)
print(f"\npass {npass}/{n} ({npass/n*100:.0f}%, floor {PASS_MIN*100:.0f}%)  vz p50={vz[n//2]:.2f} p90={vz[int(0.9*n)]:.2f} max={vz[-1]:.2f}")
bad = (npass < PASS_MIN*n) or hard
if bad:
    print("FAIL: regression vs the committed baseline:")
    if npass < PASS_MIN*n: print(f"  pass-rate {npass}/{n} below floor {PASS_MIN*100:.0f}%")
    for o in hard[:15]:
        print(f"  HARD-CAP T={o['T']} W={o['W']} seed={o['s']}: vz={o['vz']:.2f} tilt={o['tilt']:.1f} miss={o['miss']:.2f}")
    sys.exit(1)
print("PASS: within the committed distributional baseline. (Dispersion robustness: run montecarlo.py.)")
