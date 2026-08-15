#!/usr/bin/env python3
"""Ascent firmware regression suite — run this before EVERY flash.

    python regression.py            # all checks
    python regression.py --quick    # skip the Monte Carlo

Why this exists: every validation run before 2026-08-04 was ad-hoc and lived in a scratch
directory, so nothing re-ran and nothing could fail loudly. ASC038 then flew on a build the
SIL had passed 8/8 -- because the harness was structurally unable to express the bug (see
DESIGN_LOG 2026-08-04 pm). Each check below is a REGRESSION LOCK on a failure that has
actually happened or was actually found, so it can never quietly come back.

Exit code 0 = all pass. Non-zero = do not flash.

Output style matches sim/validate.py (PASS/FAIL, ANSI colour) per AI_RULES.md §11.
"""
import subprocess, re, sys, os, tempfile, shutil, random, statistics, collections

HERE = os.path.dirname(os.path.abspath(__file__))
SIM  = os.path.join(HERE, "ascent_sim.exe")
G, R, Y, X = "\033[32m", "\033[31m", "\033[33m", "\033[0m"
if os.name == "nt" and not os.environ.get("WT_SESSION"):
    try: os.system("")           # enable ANSI on legacy consoles
    except Exception: G=R=Y=X=""

# the MEASURED airframe (FLIGHT_LOG.md): m 0.927 kg, Iyy 0.0176, L 0.16 m
# Realistic gyro noise is applied to EVERY check. A perfectly noiseless IMU is a simulation
# artifact -- real MEMS always dither at the LSB, and far more under motor vibration. The
# stuck-IMU detector relies on that, so testing without noise would test a vehicle that does
# not exist and would fire the detector on healthy flights.
AIRFRAME = ["--iyy","2.257","--larm","1.143","--mass","0.927","--gnoise","0.15"]
results = []

def run(args, cwd=None):
    wd = cwd or tempfile.mkdtemp(prefix="reg_")
    p = subprocess.run([SIM]+[str(a) for a in args], capture_output=True, text=True, cwd=wd)
    return p.stdout, wd

def run_flight(args, max_boots=4, seedboots=None):
    """Run a flight, RELAUNCHING the simulator on every simulated MCU reset.

    A reset must give the firmware fresh globals, and firmware globals are file-scope statics inside
    the .ino -- they cannot be re-zeroed inside one process. So the simulator snapshots the world
    (physics + EEPROM + clock + RNG), exits 7, and we relaunch it with --statein. The new process
    starts with genuinely fresh .data/.bss, which is what the hardware actually does.
    Returns (final stdout, number of reboots).
    """
    wd = tempfile.mkdtemp(prefix="reg_")
    st = os.path.join(wd, "state.bin")
    base = [SIM] + [str(a) for a in args] + ["--stateout", st]
    extra, boots = [], 0
    try:
        while True:
            p = subprocess.run(base + extra, capture_output=True, text=True, cwd=wd)
            if p.returncode != 7 or boots >= max_boots:
                return p.stdout, boots
            boots += 1
            extra = ["--statein", st]
            if seedboots is not None: extra += ["--seedboots", str(seedboots)]
    finally:
        shutil.rmtree(wd, ignore_errors=True)

def field(out, name, cast=float):
    m = re.search(rf"{name}=\s*([-\d.eE+]+)", out)
    return cast(m.group(1)) if m else None

def outcome(out):
    m = re.search(r"outcome=(\S+)", out)
    return m.group(1) if m else "?"

def ctl_header(wd):
    f = os.path.join(wd, "CTL000.CSV")
    if not os.path.exists(f): return ""
    # Take EVERY '#' line, not the first 14. The fixed window silently truncated the header the moment
    # three plant-model lines were added ahead of the others: every field past the cut read as "missing"
    # and four locks failed with sentinel values (dt_dropped=-1, vbat_min_v=99) rather than real ones.
    # A parser that fails by returning a plausible-looking number is worse than one that throws.
    out = []
    for l in open(f).read().splitlines():
        if l.startswith("#"): out.append(l)
        elif out: break                      # header is contiguous; stop at the first data row
    return " ".join(out)

def check(name, ok, detail=""):
    results.append((name, ok, detail))
    tag = f"{G}PASS{X}" if ok else f"{R}FAIL{X}"
    print(f"  [{tag}] {name}" + (f"\n         {detail}" if detail else ""))

def section(t): print(f"\n{Y}== {t} =={X}")

# ---------------------------------------------------------------- 1. baseline
section("1. Nominal + dispersion battery")
BATTERY = [("nominal", []), ("tipoff", ["--tipx","8","--tipy","-5","--seed","7"]),
           ("misalign", ["--misx","2.0","--misy","-1.5","--seed","11"]),
           ("rollcant", ["--rollcant","0.0008","--cgoffx","0.002","--seed","3"]),
           ("unstable_aero", ["--sm","-0.05","--tipx","4","--seed","5"]),
           ("gust_noise", ["--gustx","6","--gusty","4","--gnoise","3","--seed","13"]),
           ("slowservo", ["--slew","150","--tipx","6","--seed","17"]),
           ("heavy_disp", ["--iyy","1.25","--thrust","0.9","--larm","0.9","--tipx","5","--misx","1.5","--seed","23"])]
bad = [n for n,a in BATTERY if outcome(run(a+["--gnoise","0.15"])[0]) not in ("PASS","MARGINAL")]
check("8-case battery: no aborts", not bad, f"aborted: {bad}" if bad else "")

# ------------------------------------------------- 2. ASC038: loop starvation
section("2. ASC038 regression — SD latency must NOT starve the control loop")
# THE lock. Before the fix, 143 ms of SD write latency (measured on the flight card) froze the
# attitude estimator and the vehicle tumbled to >160 deg. logData() must never touch SD in flight.
for lat in (0, 143, 600):
    out, wd = run(AIRFRAME + ["--misx","1.0","--tipx","4","--sdlatency",lat,"--seed","11"])
    hdr = ctl_header(wd)
    tilt = field(out, "boostTilt")
    dropped = re.search(r"dt_dropped=(\d+)", hdr)
    dropped = int(dropped.group(1)) if dropped else -1
    ok = outcome(out) == "PASS" and tilt < 5.0 and dropped == 0
    check(f"sdlatency {lat} ms -> controlled flight",
          ok, f"outcome={outcome(out)} boostTilt={tilt:.2f} dt_dropped={dropped}")
    shutil.rmtree(wd, ignore_errors=True)

# --------------------------------------------- 3. late ignition (measured lag)
section("3. Late ignition — TVC must still be active when the motor is")
# Measured lag is 462 ms (3 flights) and 920 ms (ASC038) vs an ASSUMED 200 ms. TVC_EXTEND_S covers it.
for lag in (0.0, 0.26, 0.45):
    out, _ = run(AIRFRAME + ["--misx","2.0","--ignlag",lag,"--seed","11"])
    lean = field(out, "endlean")
    check(f"ignlag {lag:.2f}s -> end-of-boost lean stays small", lean is not None and lean < 3.0,
          f"endlean={lean:.2f} deg")

# ----------------------------------------------- 4. actuator-lag gain margin
section("4. Gain robustness vs the UNMEASURED actuator lag")
for tau in (0.03, 0.15, 0.30):
    out, _ = run(AIRFRAME + ["--misx","2.0","--tipx","8","--servotau",tau,"--seed","11"])
    t = field(out, "boostTilt")
    check(f"servotau {tau:.2f}s -> no abort", outcome(out) != "ABORT_TUMBLE", f"boostTilt={t:.2f}")

# -------------------------------------------------- 5. Impulse 2.2 pre-arm gates
section("5. Impulse 2.2 pre-arm gates (BOARD_IMPULSE_22=1 builds only)")
healthy, _ = run(AIRFRAME + ["--seed","11"])
if outcome(healthy) == "PASS":
    for label, args, must_hold in [
            ("flat pack 6.6 V",        ["--vbat","6.6"],    True),
            ("tired pack 7.1 V",       ["--vbat","7.1"],    True),
            ("healthy pack 8.0 V",     ["--vbat","8.0"],    False),
            ("chute igniter OPEN",     ["--pyroopen","4"],  True),
            ("legs igniter OPEN",      ["--pyroopen","1"],  False)]:
        out, _ = run(AIRFRAME + args + ["--seed","11"])
        held = outcome(out) == "NORECOVERY"     # gate refused to ignite -> never flew
        check(f"{label} -> {'held on pad' if must_hold else 'allowed to fly'}",
              held == must_hold, f"outcome={outcome(out)}")
    # The ASC036/ASC038 scenario: a pack that legitimately PASSES the pad gate, then sags under servo
    # stall in flight, and has that sag RECORDED. The earlier version of this test used 7.5 V / 0.6 ohm,
    # which the pad gate now (correctly) refuses to launch -- so it was asserting against a flight that
    # never happened. A gate that rejects a bad pack is the right behaviour; it just means this lock
    # needs a pack good enough to fly, which is also the harder and more useful case for the recorder.
    out, wd = run(AIRFRAME + ["--vbat","8.0","--rint","0.45","--stallA","2.0","--misx","3.0","--seed","11"])
    m = re.search(r"vbat_min_v=([\d.]+)", ctl_header(wd))
    vmin = float(m.group(1)) if m else 99
    check("in-flight sag is recorded (vbat_min_v below open-circuit)",
          outcome(out) == "PASS" and vmin < 7.90,
          f"outcome={outcome(out)} vbat_min_v={vmin:.2f} V vs 8.00 V open-circuit")
    shutil.rmtree(wd, ignore_errors=True)
else:
    check("2.2 gates", False, "healthy-pack case did not PASS; is BOARD_IMPULSE_22 set?")

# ------------------------------------------- 5b. fault management / degradation ladder
section("5b. Degradation ladder — must engage on faults, stay SILENT on healthy flights")
# Gyro noise is on deliberately: real hardware always dithers, and the stuck-IMU detector relies on
# that. An early draft of the detector fired on healthy flights, dropped the vehicle to rate-only
# control and turned a PASS into a tumble -- a fault handler that misfires is worse than none, so the
# false-positive locks below matter MORE than the true-positive ones.
NOISE = []   # noise is already in AIRFRAME
def health(out_wd):
    h = ctl_header(out_wd)
    lvl = re.search(r"health_level=(\d)", h)
    flt = re.search(r"faults=0x([0-9A-Fa-f]+)", h)
    return (int(lvl.group(1)) if lvl else -1, int(flt.group(1),16) if flt else -1)

for label, extra in [("nominal", []), ("2 deg misalign", ["--misx","2.0"]),
                     ("tip-off + gust", ["--tipx","8","--gustx","6"]),
                     ("slow servo", ["--slew","150","--misx","1.5"]),
                     ("SD card 143 ms", ["--sdlatency","143"])]:
    out, wd = run(AIRFRAME + NOISE + extra + ["--seed","11"])
    lvl, flt = health(wd)
    check(f"healthy flight ({label}) -> NO fault raised", lvl == 0 and flt == 0,
          f"health_level={lvl} faults=0x{flt:03X}" if lvl else "")
    shutil.rmtree(wd, ignore_errors=True)

# Losing BOTH inertial sources is not survivable as a mission and the lock must not pretend otherwise.
# There is no third attitude reference, so control stops (see F_IMU_BOTH_DEAD in the firmware) and an
# aerodynamically-unstable airframe without control tumbles -- that is physics, not a firmware defect.
# What IS in the firmware's control is whether the airframe comes home: both the tilt abort and the rate
# abort read the IMU, so a dead IMU silences both, and before the blind abort this case hit the ground
# with the chute packed. So the lock asserts the LEVEL OF SAFETY that survives, not a mission success.
for t_fault in (0.5, 1.5, 2.5):
    out, wd = run(AIRFRAME + NOISE + ["--imustuck",t_fault,"--seed","11"])
    lvl, flt = health(wd)
    p4t = float(re.search(r"p4t=\s*(-?[\d.]+)", out).group(1))
    # sim clock includes pad time, so compare against the healthy flight's own launch reference
    fired = p4t > 0
    check(f"IMU wedges at t={t_fault}s -> detected, control stopped, chute OUT",
          lvl == 3 and (flt & 0x8000) and (flt & 0x008) and fired,
          f"health_level={lvl} faults=0x{flt:04X} chute_fired={'yes at t=%.2f' % p4t if fired else 'NO'}")
    shutil.rmtree(wd, ignore_errors=True)

# The blind abort must be PROMPT -- a chute that comes out after the vehicle has already arrived is not
# a recovery system. Measured 0.39 s (detection dwell + BLIND_ABORT_MS); allow headroom, but not seconds.
base_out, _ = run(AIRFRAME + NOISE + ["--seed","11"])
for t_fault in (0.5, 1.5, 2.5):
    out, wd = run(AIRFRAME + NOISE + ["--imustuck",t_fault,"--seed","11"])
    p4t = float(re.search(r"p4t=\s*(-?[\d.]+)", out).group(1))
    # both runs share the same pad sequence, so the healthy run's deploy time anchors the clock:
    # healthy deploys at apogee (~5.7 s after launch), and launch is at a fixed offset in both.
    ref = float(re.search(r"p4t=\s*(-?[\d.]+)", base_out).group(1))
    lag = p4t - (ref - 5.66) - t_fault      # (ref - 5.66) == launch instant on the sim clock
    check(f"IMU wedge at t={t_fault}s -> chute within 1.0 s of the failure",
          0 < lag < 1.0, f"chute {lag:.2f} s after the sensors died")
    shutil.rmtree(wd, ignore_errors=True)

# ------------------------------------------- 5d. loop-health governor (variable-rate clock)
section("5d. Loop-health governor — sheds work when the loop runs late")
# Only testable since 2026-08-08, when the IMU shim stopped charging a hard-coded 5 ms per read.
# Until then the loop period was CONSTANT in simulation and none of this code had ever executed.
# --loopjitterflight degrades ONLY after ignition, so it is invisible to the pad sensor-rate gate.
# Using --loopjitter here instead would be caught on the pad and the vehicle would never fly, which
# is correct behaviour but tests the gate rather than the governor.
for label, extra, expect_shed in [
        ("healthy 3.5 ms + 2 ms jitter", ["--loopbase","3.5","--loopjitter","2"],       False),
        ("degrades in flight: 0-60 ms",  ["--loopbase","3.5","--loopjitterflight","60"], True)]:
    out, wd = run(AIRFRAME + extra + ["--misx","1.0","--seed","11"])
    h = ctl_header(wd)
    baro = re.search(r"baro_interval_ms=(\d+)", h)
    baro = int(baro.group(1)) if baro else -1
    lvl, flt = health(wd)
    shed = baro > 25
    ok = (shed == expect_shed) and outcome(out) in ("PASS","MARGINAL")
    check(f"{label} -> {'sheds baro' if expect_shed else 'sheds nothing'}", ok,
          f"outcome={outcome(out)} baro_interval={baro}ms health={lvl}")
    shutil.rmtree(wd, ignore_errors=True)

# a sick bus must be caught ON THE PAD, not by silently stretching the countdown
out, _ = run(AIRFRAME + ["--imuslow","40","--seed","11"])
check("sick sensor bus -> refuses to arm (sensor-rate gate)", outcome(out) == "NORECOVERY",
      f"outcome={outcome(out)}")

# ------------------------------------------- 5e. control-sign statistic (DIAGNOSTIC, not an abort)
section("5e. Control-sign statistic — logged, must NOT act")
# Deliberately NOT an abort: measured 24/120 false positives without a saturation gate and 0/12 true
# positives with one. Wiring sense is not detectable in flight; the bench test is the only authority.
# This lock exists to stop anyone promoting it to an action without redoing that measurement.
F_INVERTED = 0x10000
bad_fire = 0
for extra in ([], ["--gustx","12","--gusty","9","--tipx","10"], ["--miswire","1","--tipx","4"]):
    _, wd = run(AIRFRAME + extra + ["--seed","11"])
    _, flt = health(wd)
    if flt & F_INVERTED: bad_fire += 1
    shutil.rmtree(wd, ignore_errors=True)
check("sign statistic never triggers a fault (diagnostic only)", bad_fire == 0,
      f"{bad_fire} case(s) raised F_INVERTED_CTRL -- it must stay non-acting")

# ------------------------------------------- 5c. people-safety interlocks
section("5c. Safety interlocks — pyros must never fire when they must not")
# The 2026-07-20 session has a recorded no-ignition run (MTR000) in which the firmware executed the
# ENTIRE sequence -- countdown, burn window, coast, recovery -- while sitting on the pad. Under the
# old code that ends with the parachute and leg charges firing at ground level next to whoever is
# standing there. This is a people-safety lock, not a vehicle one.
out, wd = run(AIRFRAME + ["--thrust","0.02","--seed","11"])
lvl, flt = health(wd)
check("no ignition -> pyros INHIBITED (never leaves the pad)",
      outcome(out) == "NORECOVERY" and (flt & 0x1000),
      f"outcome={outcome(out)} faults=0x{flt:03X} (bit12=F_NO_LIFTOFF)")
shutil.rmtree(wd, ignore_errors=True)
# and the converse: a real flight must still deploy
out, _ = run(AIRFRAME + ["--seed","11"])
check("normal flight -> still deploys", outcome(out) == "PASS", f"outcome={outcome(out)}")

# ------------------------------------------------- 6. brownout-resume interlock
section("6. Brownout resume must NOT fire from a stale EEPROM record on the ground")
# p4t is an ABSOLUTE sim-clock reading, so it moves whenever anything changes the pad sequence --
# handling and clearing a stale record costs ~0.4 s before launch, which is real but harmless. Comparing
# it for equality would fail on a benign pad-time shift while telling us nothing about the flight, so it
# is excluded here. What must be identical is everything the flight actually does, INCLUDING the state
# the vehicle was in when the chute fired (p4alt/p4vz) -- that is what a spurious resume would corrupt.
def flight_fields(s):
    return re.sub(r"p4t=\s*-?[\d.]+", "", s)

ref, _ = run(AIRFRAME + ["--misx","2.0","--seed","11"])
for ph in (1, 2):
    out, _ = run(AIRFRAME + ["--misx","2.0","--seed","11","--seedphase",ph])
    same = flight_fields(out) == flight_fields(ref)
    check(f"stale EEPROM phase={ph} -> flight identical to clean run", same,
          "" if same else "spurious resume path taken")
    # and the pad-time shift must stay a pad-time shift: never large enough to eat into the flight
    dt_pad = field(out,"p4t") - field(ref,"p4t")
    check(f"stale EEPROM phase={ph} -> costs only pad time (<1 s)", 0 <= dt_pad < 1.0,
          f"deploy shifted {dt_pad:+.2f} s on the absolute clock")

# ------------------------------------- 6b. Mid-flight MCU reset (brownout / watchdog recovery)
section("6b. Mid-flight MCU reset — the board reboots in the air and must still recover the vehicle")
# Until 2026-08-09 this whole feature was untested: the harness set a brownout flag and never acted on
# it, so setup() ran exactly once per flight and resumeAfterReset() had NEVER executed in flight
# conditions. Section 6 above only covers the negative case (a stale record on the GROUND must not
# resume). These are the positive cases, and the first run of them found a firmware bug that lost the
# vehicle outright -- see the burnout-window lock below.
for t, phase, must_pass in [(1.0,"boost",False), (2.0,"boost",False), (3.0,"boost",False),
                            (3.6,"burnout",True), (4.5,"coast",True), (5.5,"apogee",True),
                            (6.5,"descent",True)]:
    out, boots = run_flight(AIRFRAME + ["--resetat",t,"--seed","11"])
    p4t = field(out,"p4t")
    fired = p4t is not None and p4t > 0
    # The chute must ALWAYS come out. Whether the MISSION survives depends on when the reset hit:
    # resumeAfterReset() deliberately never restarts TVC, so a reset during boost means the rest of
    # the burn is uncontrolled and the vehicle tumbles -- the airframe still comes home, which is the
    # trade. After burnout there is no control left to lose, so those must still be a clean flight.
    ok = fired and boots == 1 and (outcome(out) == "PASS" if must_pass else True)
    check(f"reset at t={t}s ({phase}) -> reboots once, chute OUT" + (" , flight still PASSes" if must_pass else ""),
          ok, f"outcome={outcome(out)} boots={boots} chute={'yes at %.1f m' % field(out,'p4alt') if fired else 'NEVER'}")

# THE bug this section found. looksAirborne() took min/max across a 400 ms window and asked whether the
# WHOLE window was free-fall or the WHOLE window was thrust. A window straddling burnout is neither, so
# both tests failed and the vehicle -- at 37 m climbing at 19.7 m/s -- was declared to be on the ground.
# The chute never came out. It was the only reset time in the entire flight that lost the vehicle, and
# it sat exactly at burnout: peak servo current, and the likeliest moment to brown out in the first place.
out, boots = run_flight(AIRFRAME + ["--resetat","3.6","--seed","11"])
check("burnout-window reset is still detected as AIRBORNE (per-sample, not window min/max)",
      outcome(out) == "PASS" and field(out,"p4t") > 0,
      f"outcome={outcome(out)} chute={'at %.1f m' % field(out,'p4alt') if field(out,'p4t')>0 else 'NEVER'}")

# The SECOND blind spot in looksAirborne(), found 2026-08-13 -- by SWEEPING --resetat in 50 ms steps
# instead of probing a handful of round numbers. Specific force does not jump from thrust to free-fall,
# it sweeps: across the F15 tail-off it runs 1.53 g -> 0.48 g and passes through 1.003 g at t=3.05 s,
# so for ~250 ms it sits inside the detector's own +/-0.45 g dead band while the vehicle is at 35-40 m
# climbing at 20 m/s. Every reset in t=[2.85, 3.25] was declared "on the ground" and the chute never
# came out. The fix reads barometric altitude CHANGE across the same window; no accelerometer
# threshold can close it, because at that instant a flying rocket and a resting one genuinely produce
# the same specific force.
# The lesson is the sampling, not the bug: 1.0/2.0/3.0 straddled a 410 ms hole for months without
# landing in it, and t=3.0 only caught it once the rebuild changed the mass. Sweep, don't sample.
# (A 100 ms sweep over the whole flight, 0.2-8.0 s, was clean at the time of the fix; this keeps the
# fine sweep on the tail-off, which is the only place the crossing is physically forced.)
holes, t = [], 2.60
while t <= 3.86:
    out, boots = run_flight(AIRFRAME + ["--resetat", f"{t:.2f}", "--seed", "11"])
    p4t = field(out, "p4t")
    if not (p4t is not None and p4t > 0 and boots == 1): holes.append(f"{t:.2f}")
    t += 0.05
check("motor tail-off: EVERY reset through the 1 g crossing still deploys (50 ms sweep)",
      not holes, f"chute NEVER fired for --resetat {', '.join(holes)}")

# The chute must never be fired into the thrust column, whenever the reset lands.
for t in (1.0, 2.0, 3.0):
    out, _ = run_flight(AIRFRAME + ["--resetat",t,"--seed","11"])
    check(f"reset at t={t}s -> chute NOT fired under thrust", field(out,"p4vz") < 0,
          f"deployed at vz={field(out,'p4vz'):+.1f} m/s (must be descending)")

# Reboot-loop guard: a flight that has already rebooted RESUME_MAX_BOOTS times must stop trying rather
# than cycle reboot -> deploy -> reboot on the pyros forever. Failing safe here means NOT deploying.
for b, should_deploy in [(0,True), (2,True), (3,False), (4,False)]:
    out, _ = run_flight(AIRFRAME + ["--resetat","5.0","--seed","11"], seedboots=b)
    fired = field(out,"p4t") is not None and field(out,"p4t") > 0
    check(f"boots={b} -> {'resumes' if should_deploy else 'gives up (RESUME_MAX_BOOTS=3)'}",
          fired == should_deploy, f"outcome={outcome(out)} chute={'fired' if fired else 'not fired'}")

# ------------------------------------------- 6c. Bus hang vs firmware lockup (two different faults)
section("6c. Hangs — a wedged BUS and a wedged FIRMWARE need different defences")
# FAILURE_MODES D2 used to claim a stuck I2C bus wedges the loop forever and that Teensy's timeout API
# was unused. Checked against WireIMXRT.cpp (Teensyduino 1.59): the driver already bounds every transfer
# (CLOCK_STRETCH_TIMEOUT 15 ms in hardware, 50 ms generic bail-out, returning error 4). The real defect
# was ours -- imuReadSource() DISCARDED getEvent()'s return, so a timed-out read was published as a live
# sample and every downstream protection keyed off that bool was silently disabled.
for t in (1.0, 2.0, 3.0, 5.0):
    out, boots = run_flight(AIRFRAME + ["--i2chang",t,"--seed","11"])
    p4t = field(out,"p4t")
    check(f"I2C bus hang at t={t}s -> failed read, ladder responds, chute OUT",
          p4t is not None and p4t > 0 and boots == 0,
          f"outcome={outcome(out)} reboots={boots} "
          f"chute={'at %.1f m' % field(out,'p4alt') if p4t>0 else 'NEVER'}")

# A firmware lockup is a different animal: nothing returns, so nothing feeds WDOG1. The watchdog is the
# only defence, and it did not exist in simulation until 2026-08-09 (wdtFeed/wdtStart were no-ops
# off-target). It is also why the watchdog is now armed EARLY on an in-flight reboot: the first run of
# this test reset once, hung again inside setup()'s IMU probe -- which runs BEFORE the old wdtStart() --
# and sat there unprotected until impact with the chute packed.
for t, dur in [(2.0, 1.0), (2.0, 3.0)]:
    out, boots = run_flight(AIRFRAME + ["--firmwarehang",t,"--firmwarehangdur",dur,"--seed","11"])
    p4t = field(out,"p4t")
    check(f"transient {dur}s lockup at t={t}s -> watchdog resets, flight RECOVERS",
          outcome(out) == "PASS" and p4t > 0 and boots >= 1,
          f"outcome={outcome(out)} reboots={boots} chute={'at %.1f m' % field(out,'p4alt') if p4t>0 else 'NEVER'}")

# Honest negative lock: a lockup that recurs identically on every boot cannot be recovered, because no
# parachute can be deployed by code that cannot run. Locked so nobody later reads the watchdog as a
# guarantee it is not -- and so that if this ever starts PASSING, someone checks WHY.
out, boots = run_flight(AIRFRAME + ["--firmwarehang","2.0","--seed","11"])
check("PERMANENT lockup is unrecoverable (documented limit, not a regression)",
      (field(out,"p4t") or -1) < 0, f"outcome={outcome(out)} reboots={boots}")

# ------------------------------------------- 6d. Plant model / torque-commanded TVC
section("6d. Plant model — keff is identified in flight, not assumed")
# The harness only gained burn-varying mass properties on 2026-08-09; before that MASS/IYY/L_ARM were
# constant for the whole flight and 60 g of F15 propellant never left the aft end, so none of this
# could be tested at all.
for th in ("0.85", "1.00", "1.15"):
    out, _ = run(AIRFRAME + ["--thrust",th,"--seed","11"])
    kt, ke = field(out,"keff_true"), field(out,"keff_est")
    err = abs(ke-kt)/kt if kt else 9
    check(f"thrust x{th}: identified keff within 3% of truth at mid-burn", err < 0.03,
          f"true={kt:.1f} est={ke:.1f} ({100*(ke-kt)/kt:+.1f}%)")

# At the design point the scheduled gains must reduce EXACTLY to the hand-computed P_GAIN/D_GAIN --
# otherwise this is a new controller wearing the old one's validation.
out, _ = run(AIRFRAME + ["--seed","11"])
base, _ = run(AIRFRAME + ["--seed","11","--fixedgain"])
check("torque-commanded reduces to the flown gains on the nominal airframe",
      abs(field(out,"boostTilt") - field(base,"boostTilt")) < 0.35,
      f"torque={field(out,'boostTilt'):.2f} fixed={field(base,'boostTilt'):.2f} deg boost tilt")

# THE SCHEDULE MUST STAY ONE-SIDED. Full normalisation was measured WORSE (139/140 airframes, PASS
# 100% -> 94.3%) because zeta rises with keff, so the thrust spike hands the loop free damping.
# This lock fails if anyone re-introduces a schedule that gives that margin back.
out, _ = run(AIRFRAME + ["--tipx","6","--tipy","6","--seed","11"])
base, _ = run(AIRFRAME + ["--tipx","6","--tipy","6","--seed","11","--fixedgain"])
check("schedule never surrenders margin at HIGH keff (must not exceed fixed-gain tilt)",
      field(out,"boostTilt") <= field(base,"boostTilt") + 0.35,
      f"torque={field(out,'boostTilt'):.2f} vs fixed={field(base,'boostTilt'):.2f}")

# The motivating case: a rebuilt airframe. Typing the new mass properties in must recover the response
# that hand-rescaling the gains would have given -- that is the entire point of deriving them.
#
# REWRITTEN 2026-08-14. The old form asserted `told < stale - 0.5` on a bare --tipx 5 run. That is
# arithmetically unsatisfiable: the scenario is so benign that stale itself is only 0.38 deg, so it
# demanded a boost tilt below -0.12 deg. NO firmware change could ever pass it, and it had been
# reading as a firmware defect. Two things were wrong with it, and both are worth not repeating:
#   - an ABSOLUTE threshold on a metric whose scale is set by the airframe and the disturbance, and
#   - a scenario with no disturbance worth rejecting, so nothing could distinguish the two arms.
# Now run under thrust misalignment -- section 7b establishes that misalignment, not gain choice, is
# the dominant driver of what a flight does -- and scored RELATIVE, over several seeds so one lucky
# draw cannot decide it. This is not a threshold tuned until it went green: entering the numbers is
# worth ~20% here, and the bar is set at 10%.
heavy = ["--mass","0.927","--iyy","3.40","--larm","1.30","--tipx","5",
         "--misx","2.0","--gnoise","0.15"]
gains = []
for sd in (11, 12, 13, 14, 15):
    stale, _ = run(heavy + ["--fixedgain",    "--seed", sd])
    told,  _ = run(heavy + ["--tellfirmware", "--seed", sd])
    gains.append((field(stale,"boostTilt"), field(told,"boostTilt")))
med = statistics.median((t - s) / s for s, t in gains)
check("heavier rebuild: entering the numbers beats flying stale gains (5 seeds, >=10%)",
      med < -0.10,
      f"median {100*med:+.1f}% boost tilt vs stale "
      f"(stale {statistics.median(s for s,_ in gains):.2f} -> told {statistics.median(t for _,t in gains):.2f} deg)")

# THE OTHER HALF OF THE SAME KNOB, and it points the opposite way. Added 2026-08-14.
# The one-sided lock above varies --tipx to reach "HIGH keff", but tip-off does not set keff at all:
# keff = T*L/Iyy. So nothing in this suite had ever driven keff high through the parameter that
# actually controls it, and the schedule's protection was never tested where it matters.
# Driving it properly (Iyy DOWN -> keff UP, i.e. a LIGHTER rebuild) shows the clamp doing its job --
# sched = constrain(keffNominal/keffEst, 1.0, ..) is inert above nominal, so the in-flight schedule
# never gives margin back. But --tellfirmware does not go through the schedule: it rewrites DRY_*,
# which raises keffNominal itself, and pEff = TVC_WN^2/keffNominal then FALLS. That is exactly the
# full normalisation the schedule comment says was measured worse on 139 of 140 airframes -- entering
# through the nominal instead of through the schedule, where the one-sided clamp cannot protect it.
# Measured (40 seeds, 1 deg/axis misalign): worst-case boost tilt 2.16 -> 2.52 -> 3.12 -> 4.50 deg as
# Iyy falls 2.257 -> 1.90 -> 1.50 -> 1.00, monotone, and told was worse on 40 of 40 seeds at each.
# Every arm still PASSed 40/40, so this is MARGIN, not safety -- and that is what is locked here.
# It is deliberately NOT asserted that telling the firmware helps on a light airframe. It does not.
for iyy, worst_allowed in (("1.90", 3.2), ("1.50", 3.8)):
    light = ["--mass","0.927","--iyy",iyy,"--larm","1.143","--misx","1.0","--misy","1.0",
             "--tipx","3","--gnoise","0.15"]
    bad = []
    for sd in (101, 102, 103, 104, 105):
        told, _ = run(light + ["--tellfirmware", "--seed", sd])
        if outcome(told) != "PASS" or field(told,"boostTilt") > worst_allowed:
            bad.append(f"seed {sd}: {outcome(told)} tilt {field(told,'boostTilt'):.2f}")
    check(f"lighter rebuild (Iyy x{iyy}): typing the numbers in costs margin but never the flight",
          not bad, "; ".join(bad))

# ----------------------------------------------------------- 7. Monte Carlo
#
# WHY THIS SECTION IS SPLIT (rewritten 2026-08-08)
#
# The old version ran ONE Monte Carlo at sigma=1.5 deg/axis thrust misalignment and locked
# "pass rate >= 90%". That lock was measuring the wrong thing. Misalignment sigma is a statement
# about how well the MOTOR MOUNT WAS BUILT, not about the firmware -- so a badly-aligned build made
# the suite print DO NOT FLASH at code that cannot possibly fix it, and (worse) a well-aligned build
# would hide a genuine firmware regression behind the easy margin.
#
# Measured from real flight data (mean commanded TVC over the burn = the misalignment being trimmed):
#     ASC036  1.12 deg   flew clean, gimbal never saturated
#     ASC037  >5.2 deg   railed for 85% of the burn -- authority-limited, exactly this failure mode
# and the sim's own sensitivity, 600 flights per row:
#     sigma 0.50 -> 100.0% PASS      sigma 1.00 -> 94.8%      sigma 1.50 -> 83.8%   sigma 2.00 -> 75.0%
#
# So: gate the FIRMWARE on a well-built airframe, where any failure must be the code's fault, and
# report the build-quality curve as information rather than as a pass/fail on the software.
def mc(sigma, n, seed):
    rnd = random.Random(seed); out_c = collections.Counter(); tilts = []
    cl = lambda x,a,b: max(a, min(b, x))
    for i in range(n):
        a = ["--mass","0.927",
             "--iyy",  round(cl(rnd.gauss(2.257,0.23),1.6,3.0),3),
             "--larm", round(cl(rnd.gauss(1.143,0.11),0.85,1.45),3),
             "--thrust",round(cl(rnd.gauss(1,0.06),0.85,1.15),3),
             "--tipx", round(rnd.gauss(0,4.0),2), "--tipy", round(rnd.gauss(0,4.0),2),
             "--misx", round(rnd.gauss(0,sigma),3), "--misy", round(rnd.gauss(0,sigma),3),
             "--sm",   round(rnd.gauss(0,0.006),4), "--gnoise","0.15",
             "--slew", round(cl(rnd.gauss(250,70),100,500),0),
             "--gustx",round(rnd.gauss(0,0.8),3), "--gusty",round(rnd.gauss(0,0.8),3),
             "--ignlag",round(cl(rnd.gauss(0.262,0.058),0.1,0.45),3), "--seed", i+1]
        o, _ = run(a); out_c[outcome(o)] += 1; tilts.append(field(o,"boostTilt"))
    return out_c, tilts

if "--quick" not in sys.argv:
    section("7. Monte Carlo — FIRMWARE lock on a well-built airframe (N=120, misalign sigma=0.5 deg)")
    # Everything else is still fully dispersed: inertia, moment arm, thrust, tip-off, gusts, servo
    # slew, static margin, ignition lag. Only the motor alignment is held to a standard the build can
    # actually meet. At this sigma the vehicle has authority margin, so a tumble here means the CODE
    # broke -- which is precisely what a firmware regression suite should be sensitive to.
    out_c, tilts = mc(0.5, 120, 777)
    npass = out_c["PASS"]; nabort = out_c["ABORT_TUMBLE"]
    detail = "  ".join(f"{k} {v} ({100*v/120:.1f}%)" for k,v in out_c.most_common())
    check("well-built airframe: pass rate >= 98%", npass/120 >= 0.98, detail)
    check("well-built airframe: abort rate <= 1%", nabort/120 <= 0.01,
          f"boost tilt p50 {statistics.median(tilts):.2f} p90 {sorted(tilts)[108]:.2f}")

    section("7b. Build-quality sensitivity — NOT a firmware gate, this is the alignment spec")
    print("     Motor alignment is the dominant driver of mission success and no gain setting")
    print("     substitutes for it (I_GAIN 0.2 -> 0.8 buys under 1 point; alignment buys 25).")
    print(f"     {'sigma/axis':>11}  {'typical':>8}  {'PASS':>7}  {'ABORT':>7}")
    curve = {}
    for sg in (0.5, 1.0, 1.5, 2.0):
        oc, _ = mc(sg, 60, 777)
        p, ab = 100.0*oc["PASS"]/60, 100.0*oc["ABORT_TUMBLE"]/60
        curve[sg] = p
        flag = "   <- ASC037 was here" if sg >= 2.0 else ("   <- ASC036 was here" if sg == 1.0 else "")
        print(f"     {sg:10.2f}  {1.253*sg:7.2f}d  {p:6.1f}%  {ab:6.1f}%{flag}")
    # A lock that only fires if the ORDERING breaks -- i.e. if better alignment stopped helping,
    # which would mean the misalignment path in the sim or the firmware trim had regressed.
    check("mission success is monotonic in alignment quality",
          curve[0.5] >= curve[1.0] >= curve[1.5] >= curve[2.0],
          "  ".join(f"s{k}={v:.0f}%" for k,v in sorted(curve.items())))

# ---------------------------------------------------------------- summary
nfail = sum(1 for _,ok,_ in results if not ok)
print(f"\n{'='*64}")
print(f"  {len(results)-nfail} passed, {nfail} failed")
if nfail:
    print(f"  {R}DO NOT FLASH{X} — failing: " + ", ".join(n for n,ok,_ in results if not ok))
else:
    print(f"  {G}All regression locks hold.{X}")
print(f"{'='*64}")
sys.exit(1 if nfail else 0)
