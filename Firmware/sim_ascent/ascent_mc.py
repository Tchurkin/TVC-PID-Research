#!/usr/bin/env python3
"""Monte-Carlo robustness rig for the ascent SIL (runs the EXACT Ascent_TVC.ino)."""
import subprocess, random, statistics, re, sys, os
SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ascent_sim.exe")
N = int(sys.argv[1]) if len(sys.argv) > 1 else 300

def run(args):
    p = subprocess.run([SIM] + [str(a) for a in args], capture_output=True, text=True)
    line = p.stdout.strip()
    d = dict(re.findall(r"(\w+)=\s*(ABORT_TUMBLE|NORECOVERY|MARGINAL|NOFLY|PASS|[\-\d.]+)", line))
    return d, line

def clip(x, a, b): return max(a, min(b, x))
def pct(xs, p): xs = sorted(xs); return xs[min(len(xs)-1, int(p/100*len(xs)))]

print("NOMINAL:", run([])[1])

random.seed(12345)
outcomes = {}; tilts = []; rates = []; coasts = []; rows = []
for i in range(N):
    args = ["--iyy",   round(clip(random.gauss(1, 0.10), 0.80, 1.25), 3),   # inertia measurement error
            "--thrust",round(clip(random.gauss(1, 0.06), 0.85, 1.15), 3),   # F-15 lot variation
            "--larm",  round(clip(random.gauss(1, 0.10), 0.75, 1.30), 3),   # CG->TVC-axis measurement + migration
            "--tipx",  round(random.gauss(0, 4.0), 2), "--tipy", round(random.gauss(0, 4.0), 2),  # rail tip-off (deg/s)
            "--misx",  round(random.gauss(0, 0.3), 3), "--misy", round(random.gauss(0, 0.3), 3),  # thrust misalignment (deg)
            "--gnoise", 0.15,
            "--gbiasx",round(random.gauss(0, 0.3), 3), "--gbiasy",round(random.gauss(0, 0.3), 3), # gyro bias (deg/s)
            "--slew",  round(clip(random.gauss(350, 80), 150, 600), 0),     # servo slew (deg/s)
            "--gustx", round(random.gauss(0, 0.8), 3), "--gusty",round(random.gauss(0, 0.8), 3),  # gust torque (rad/s^2)
            "--seed",  i+1]
    d, l = run(args)
    o = d.get("outcome", "?"); outcomes[o] = outcomes.get(o, 0) + 1
    tilts.append(float(d.get("boostTilt", 999))); rates.append(float(d.get("boostRate", 999)))
    coasts.append(float(d.get("coastTilt", 999)))
    rows.append((float(d.get("boostTilt", 999)), l))
    if (i+1) % 100 == 0: print(f"  {i+1}/{N} ...")

print(f"\n================ ASCENT SIL MONTE CARLO  N={N} ================")
for o, c in sorted(outcomes.items(), key=lambda x: -x[1]):
    print(f"    {o:14s}: {c:4d}  ({c/N*100:5.1f}%)")
print(f"BOOST tilt (deg, THE control test): p50 {statistics.median(tilts):.2f}  p90 {pct(tilts,90):.2f}  p99 {pct(tilts,99):.2f}  max {max(tilts):.2f}")
print(f"BOOST rate (dps):                   p50 {statistics.median(rates):.1f}  p90 {pct(rates,90):.1f}  max {max(rates):.1f}")
print(f"coast tilt (deg, UNCONTROLLED post-burnout -- chute recovers regardless): p50 {statistics.median(coasts):.1f}  p90 {pct(coasts,90):.1f}  max {max(coasts):.1f}")
print("worst 6 by BOOST tilt:")
for t, l in sorted(rows, reverse=True)[:6]: print("   " + l)
