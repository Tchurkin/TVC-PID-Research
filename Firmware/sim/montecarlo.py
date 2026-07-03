#!/usr/bin/env python3
"""Monte Carlo flight-readiness dispersion analysis for the Sysiphus landing firmware.

The target x wind SWEEP holds mass/inertia/CG/thrust/fin/timing at their exact nominal values -- it only
varies wind + gusts. A real flight disperses ALL of them at once (build tolerance, part variation, motor
scatter, loop jitter). This runs the EXACT firmware over N joint random draws and reports the touchdown
distribution + failure rate + the worst cases with their parameters, so "ready to fly?" has a number.

Usage:  python montecarlo.py [N] [path\\to\\sim.exe]
Env:    SCENARIO = broad (default) | realistic (mass weighed tight, motors = lot variation)
        MASSID   = 1 (default) | 0 (pass --nomassid to disable in-flight mass ID)
"""
import subprocess, csv, io, sys, os, random, re, statistics

_here = os.path.dirname(os.path.abspath(__file__))
N   = int(sys.argv[1]) if len(sys.argv) > 1 else 300
SIM = sys.argv[2] if len(sys.argv) > 2 else os.path.join(_here, "sim.exe")
SCENARIO = os.environ.get("SCENARIO", "broad")
MASSID   = os.environ.get("MASSID", "0") == "1"   # in-flight mass ID is OFF by default (see firmware note)
# 1-sigma dispersions per scenario. "realistic": the airframe is WEIGHED and balanced on the ground (mass/CG
# tight, known), so the loose, un-measurable variables are the solid MOTOR lots (ascent + landing thrust).
# "characterized": additionally each motor lot is static-fired and thrust-matched (tw/land tight) -> shows the
# motor-tolerance requirement for a reliable soft landing.
SIG = dict(broad        =dict(mass=0.04, iyy=0.06, cg=0.004, tw=0.04,  land=0.05,  fin=0.12),
           realistic    =dict(mass=0.02, iyy=0.05, cg=0.003, tw=0.04,  land=0.04,  fin=0.10),
           characterized=dict(mass=0.02, iyy=0.05, cg=0.003, tw=0.015, land=0.015, fin=0.10))[SCENARIO]
# 3-D roll-coupling build errors (all scenarios; these are what make TVC leak into roll):
SIG3D = dict(cglat=0.0015,     # 1-sigma LATERAL CG offset per axis (m): thrust axis misses the CG
             cant =2e-5,       # 1-sigma nozzle tangential cant arm (m): constant roll torque under thrust
             finmis=1e-6)      # 1-sigma fin misalignment roll coefficient (m^3)
if not os.path.exists(SIM):
    sys.exit(f"sim.exe not found at {SIM} -- build it first (build_zig.bat).")

# Pass gates (same as the SIL VERDICT): soft, on-target, upright, full profile flown.
VZ_GATE, MISS_GATE, TILT_GATE = 5.0, 5.0, 0.5   # m/s, m, rad

def run(args):
    p = subprocess.run([SIM] + [str(a) for a in args], capture_output=True, text=True)
    e = p.stderr
    def g(pat, d=0.0):
        m = re.search(pat, e); return float(m.group(1)) if m else d
    st = re.search(r"final state     : (\w+)", e)
    o = dict(vz=g(r"touchdown speed : ([\d.]+)"),
             miss=g(r"2-D miss ([\-\d.]+)"),           # planar miss: sqrt((px-T)^2 + py^2)
             tilt=g(r"final tilt 3d   : ([\-\d.]+)"),  # total tilt from the quaternion
             aborted=(st.group(1)=="ABORT" if st else False),   # safety abort -> CHUTE (recovered, not a landing;
                                                                #   the harness doesn't model chute drag, so its
                                                                #   ballistic "impact" number is meaningless)
             apogee=g(r"apogee          : ([\-\d.]+)"),
             td=("TOUCHDOWN" in (re.search(r"final state     : (\w+)", e) or re.match("","")).group(1) if re.search(r"final state     : (\w+)", e) else False),
             passed="PASS" in e)
    rows = list(csv.DictReader(io.StringIO(p.stdout)))
    o["landx"] = float(rows[-1]["px"]) if rows else 0.0
    return o

def bisect_kick(T, W=0):
    lo, hi = -8.0, 20.0
    for _ in range(7):
        mid = (lo + hi) / 2
        o = run(["--kick", mid, "--wind", W, "--target", T, "--nodivert", "--igndelay", 0, "--ignjit", 0])
        lo, hi = (mid, hi) if o["landx"] < T else (lo, mid)
    return round((lo + hi) / 2, 2)

print("calibrating calm-wind kick vs target (pre-flight plan)...")
KT = [0, 8, 16, 24]
KV = [bisect_kick(t) for t in KT]
def kick_for(T):
    if T <= KT[0]: return KV[0]
    if T >= KT[-1]: return KV[-1]
    for i in range(len(KT)-1):
        if KT[i] <= T <= KT[i+1]:
            f = (T-KT[i])/(KT[i+1]-KT[i]); return KV[i] + f*(KV[i+1]-KV[i])
    return KV[-1]

def clip(x, lo, hi): return max(lo, min(hi, x))

# Dispersions (1-sigma). Set to plausible small-rocket build/part tolerances; the point is JOINT stacking.
random.seed(12345)
results, fails = [], []
print(f"running {N} dispersed flights...")
for i in range(N):
    T    = random.uniform(0, 24)
    W    = random.uniform(-5, 5)
    WY   = random.uniform(-4, 4)                       # crosswind (3-D)
    cgx  = clip(random.gauss(0.0, SIG3D["cglat"]), -0.005, 0.005)
    cgy  = clip(random.gauss(0.0, SIG3D["cglat"]), -0.005, 0.005)
    cant = clip(random.gauss(0.0, SIG3D["cant"]), -8e-5, 8e-5)
    fmis = clip(random.gauss(0.0, SIG3D["finmis"]), -4e-6, 4e-6)
    mass = clip(random.gauss(1.0, SIG["mass"]), 0.85, 1.15)   # build/propellant (weighable on the ground)
    iyy  = clip(random.gauss(1.0, SIG["iyy"]),  0.80, 1.25)   # inertia
    cg   = clip(random.gauss(0.0, SIG["cg"]),  -0.012, 0.012) # CG shift (m); nominal static margin ~0.033
    tw   = clip(random.gauss(1.0, SIG["tw"]),   0.85, 1.15)   # ascent motor thrust (lot variation)
    land = clip(random.gauss(1.0, SIG["land"]), 0.82, 1.18)   # landing motor thrust (lot variation, UN-measurable pre-ignition)
    fin  = clip(random.gauss(1.0, SIG["fin"]),  0.65, 1.6)    # fin aero (above the ~0.55 physical cliff)
    args = ["--wind", round(W,2), "--windy", round(WY,2), "--target", round(T,2), "--kick", round(kick_for(T),2),
            "--gusts", "--seed", i+1, "--massmult", round(mass,3), "--iyymult", round(iyy,3), "--cgoff", round(cg,4),
            "--cgoffx", round(cgx,5), "--cgoffy", round(cgy,5), "--rollcant", f"{cant:.6f}", "--finmis", f"{fmis:.7f}",
            "--tw", round(tw,3), "--land", round(land,3), "--finauth", round(fin,3),
            "--igndelay", 25, "--ignjit", 12, "--jitter", 0.003]
    if MASSID: args.append("--massid")
    o = run(args); o.update(T=T, W=W, mass=mass, iyy=iyy, cg=cg, tw=tw, land=land, fin=fin)
    results.append(o)
    if not o["passed"]: fails.append((args, o))
    if (i+1) % 50 == 0: print(f"  {i+1}/{N}  running pass rate {sum(1 for r in results if r['passed'])/(i+1)*100:.1f}%")

def pct(xs, p):
    xs = sorted(xs); k = min(len(xs)-1, int(p/100*len(xs))); return xs[k]
vz  = [r["vz"] for r in results if not r["aborted"]] or [0.0]
am  = [abs(r["miss"]) for r in results if not r["aborted"]] or [0.0]
at  = [abs(r["tilt"]) for r in results if not r["aborted"]] or [0.0]
npass = sum(1 for r in results if r["passed"])
landed = [r for r in results if not r["aborted"]]
nab = N - len(landed)
def upright_ontarget(r): return abs(r["miss"])<MISS_GATE and abs(r["tilt"])<TILT_GATE*57.3
def leg_gate(v): return sum(1 for r in landed if r["vz"]<v and upright_ontarget(r))
print("\n================ MONTE CARLO FLIGHT-READINESS ================")
print(f"scenario={SCENARIO}  massID={'on' if MASSID else 'OFF'}  N={N}   ABORTS(chute)={nab} ({nab/N*100:.1f}%)")
print(f"PASS = {npass}/{N} = {npass/N*100:.1f}%   (gates: vz<{VZ_GATE} m/s, |miss|<{MISS_GATE} m, tilt<{TILT_GATE*57.3:.0f} deg)")
print("survivable rate vs LEG touchdown-speed rating (of ALL flights; aborts recover on the chute, excluded here):")
for v in (5,6,8,10,12): print(f"    legs good for {v:2d} m/s : {leg_gate(v)/N*100:5.1f}%")
print(f"touchdown vz  m/s : p50 {statistics.median(vz):.2f}  p90 {pct(vz,90):.2f}  p99 {pct(vz,99):.2f}  max {max(vz):.2f}")
print(f"|miss|        m   : p50 {statistics.median(am):.2f}  p90 {pct(am,90):.2f}  p99 {pct(am,99):.2f}  max {max(am):.2f}")
print(f"|tilt|        deg : p50 {statistics.median(at):.2f}  p90 {pct(at,90):.2f}  p99 {pct(at,99):.2f}  max {max(at):.2f}")
# failure breakdown by dominant gate
def why(o):
    r=[]
    if o["vz"]>=VZ_GATE: r.append(f"vz={o['vz']:.1f}")
    if abs(o["miss"])>=MISS_GATE: r.append(f"miss={o['miss']:.1f}")
    if abs(o["tilt"])>=TILT_GATE*57.3: r.append(f"tilt={o['tilt']:.0f}")
    if not o.get("td"): r.append("no-TD")
    return ",".join(r) or "?"
print(f"\nFAILURES: {len(fails)}")
for args, o in sorted(fails, key=lambda f: -f[1]['vz'])[:15]:
    print(f"  [{why(o):>16}] T={o['T']:4.1f} W={o['W']:+.1f} mass={o['mass']:.2f} iyy={o['iyy']:.2f} "
          f"cg={o['cg']:+.3f} tw={o['tw']:.2f} land={o['land']:.2f} fin={o['fin']:.2f}")
print("=============================================================")
