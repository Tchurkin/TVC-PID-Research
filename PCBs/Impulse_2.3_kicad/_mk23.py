# Create Impulse 2.3 from Impulse 2.2.
# 2.3 exists for ONE root cause: Q21, the pyro arm P-FET, was an AO3401A in SOT-23 carrying the
# full firing current of every pyro channel. A 900-1000 ms pulse into a ~1 ohm initiator is ~7-8 A;
# at ~60 mOhm that is 1.2-2.4 W in a package whose transient thermal impedance is ~130 C/W at one
# second, i.e. 150-300 C of junction rise. It failed on the bench 2026-08-12, exactly as
# DESIGN_LOG 2026-07-23 "REMAINING MANUAL STEPS" item (3) predicted it would.
#
# This script only does the mechanical fork (copy + rename + reference rewrite). The electrical
# changes live in _fix_pyro_arm.py.
import os, re, shutil, sys

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Impulse_2.2_kicad"))
DST = os.path.dirname(os.path.abspath(__file__))
OLD, NEW = "Impulse_2.2", "Impulse_2.3"

PROJECT_FILES = ["kicad_pcb", "kicad_sch", "kicad_pro", "kicad_prl"]
VERBATIM = ["Impulse22.kicad_sym", "fp-lib-table", "sym-lib-table"]


def must(c, m):
    if not c:
        print("FAIL:", m)
        sys.exit(1)


must(os.path.isdir(SRC), "source project not found: " + SRC)

# Refuse to run against a KiCad session that has 2.2 open -- a save from that session would be
# written from stale in-memory state and we would fork the wrong bytes.
locks = [f for f in os.listdir(SRC) if f.startswith("~") and f.endswith(".lck")]
if locks and "--ignore-locks" not in sys.argv:
    print("REFUSING: KiCad has %s open (%s)." % (OLD, ", ".join(locks)))
    print("Close it (or pass --ignore-locks if you are certain the on-disk files are current).")
    sys.exit(2)

for ext in PROJECT_FILES:
    s = os.path.join(SRC, "%s.%s" % (OLD, ext))
    d = os.path.join(DST, "%s.%s" % (NEW, ext))
    must(os.path.isfile(s), "missing " + s)
    t = open(s, encoding="utf-8", newline="").read()
    crlf = "\r\n" in t
    t = t.replace("\r\n", "\n")
    n_before = t.count(OLD)
    t = t.replace(OLD, NEW)
    # The custom library nickname "Impulse22" is deliberately NOT renamed: it is one shared
    # project library across board revisions, and renaming it would touch every footprint ref.
    must(OLD not in t, "residual %s reference in %s" % (OLD, ext))
    open(d, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
    print("%-10s copied, rewrote %3d references" % (ext, n_before))

for f in VERBATIM:
    shutil.copy2(os.path.join(SRC, f), os.path.join(DST, f))
    print("%-10s copied verbatim" % f)

pretty_s = os.path.join(SRC, "Impulse22.pretty")
pretty_d = os.path.join(DST, "Impulse22.pretty")
if os.path.isdir(pretty_d):
    shutil.rmtree(pretty_d)
shutil.copytree(pretty_s, pretty_d)
print("Impulse22.pretty copied (%d footprints)" % len([f for f in os.listdir(pretty_d) if f.endswith(".kicad_mod")]))

# 3D models are referenced through ${KICAD10_3DMODEL_DIR}; confirm nothing points at a path that
# would break, then leave the 20 MB 3dmodels/ tree where it is (2.2's copy) rather than duplicate it.
pcb = open(os.path.join(DST, "%s.kicad_pcb" % NEW), encoding="utf-8").read()
models = sorted(set(re.findall(r'\(model "([^"]+)"', pcb)))
bad = [m for m in models if not m.startswith("${KICAD10_3DMODEL_DIR}") and not m.startswith("${KIPRJMOD}")]
print("3D models: %d refs, %d not env-var relative" % (len(models), len(bad)))
for m in bad:
    print("   NOTE non-portable model path:", m)
print("\nImpulse 2.3 forked from 2.2. Electrical changes: run _fix_pyro_arm.py")
