# Copper rework around the new SOIC-8 Q21 (see _fix_pyro_arm.py for the why).
#
# Phases, so the clearance check runs against the board as it will actually be:
#   --cut    delete the stubs that pointed at the old SOT-23 pads
#   --check  verify every planned addition against pcbnew-dumped geometry (re-dump between phases)
#   --add    lay the new copper
#
# Gate detour rationale: on a SOIC-8 the gate is pin 4, the SOUTHERN-most left pad, below all three
# source pads -- but west of Q21 the gate lane runs NORTH of the source lane. That order inversion
# forces exactly one crossing. F.Cu cannot take it (the C15/R67/R68 corridors are 0.7 mm) and a via
# cannot sit just south of the rail either, because In2 carries a board-wide horizontal signal bus
# (CS_IMU 51.564, SENS_DET 52.374, INT_IMU 53.183, MISO 54.038) whose 0.81 mm gaps are too tight for
# a 0.6 mm via plus clearance. So PYRO_G drops to In1, runs south clear of the whole bus, and
# returns in the empty F.Cu region below Q21.
import hashlib, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
PRO = os.path.join(HERE, "Impulse_2.3.kicad_pro")
GEOM = os.path.join(HERE, "geom.json")

W_RAIL = 1.0     # approach runs: capped by R67's 0.8 mm pad, not by choice
W_BOND = 1.2     # along the SOIC-8 pad columns; 1.4+ collides with the gate pad's clearance
W_SIG = 0.25
VIA_S, VIA_D = 0.6, 0.3
# Extra rail cross-section comes from copper ZONES (_pour_pyro_arm.py), not from fatter tracks:
# on a 1.27 mm pad pitch a 2 mm track cannot clear the gate pad, and zones self-clear other nets.

# stubs left pointing at the old SOT-23 pads
CUTS = [
    ("PYRO_G",   "F.Cu", 166.870, 49.034, 163.551, 49.034),
    ("7.4V_RAW", "F.Cu", 166.681, 50.745, 163.490, 50.745),
    ("7.4V_RAW", "F.Cu", 166.870, 50.934, 166.681, 50.745),
    ("7.4V",     "F.Cu", 169.496, 49.233, 168.745, 49.984),
    ("7.4V",     "F.Cu", 169.496, 48.841, 169.496, 49.233),
]

# (net, layer, x1, y1, x2, y2, width, label)
#
# Gate route. Going north-then-east on F.Cu was tried and is genuinely impossible: the corridor
# between the source-pad column and the 1 mm 7.4V diagonal feeding P1O closes to nothing (the two
# constraints cross at x=166.70 vs x=166.82). So PYRO_G drops to In1 immediately east of R67, runs
# under the source pads, and comes back up at the one via-legal pocket in this whole area -- the
# empty region under the SOIC-8 body between the two pad rows.
ADDS = [
    ("PYRO_G",   "F.Cu",   163.490, 49.095, 163.300, 49.200, W_SIG,  "R67.2 -> via down"),
    ("PYRO_G",   "In1.Cu", 163.300, 49.200, 167.500, 50.000, W_SIG,  "In1 under the source pads"),
    ("PYRO_G",   "F.Cu",   167.500, 50.000, 167.500, 52.200, W_SIG,  "south, under the SOIC-8 body"),
    ("PYRO_G",   "F.Cu",   167.500, 52.200, 165.420, 51.889, W_SIG,  "west into Q21.4 gate"),
    ("7.4V_RAW", "F.Cu",   163.490, 50.745, 165.420, 50.619, W_RAIL, "R67.1 -> Q21.3 source"),
    ("7.4V_RAW", "F.Cu",   165.420, 50.619, 165.420, 48.079, W_BOND, "bond source pads 3-2-1"),
    ("7.4V",     "F.Cu",   170.370, 51.889, 170.370, 48.079, W_BOND, "bond drain pads 5-6-7-8"),
    ("7.4V",     "F.Cu",   170.370, 49.349, 169.496, 48.841, W_RAIL, "drain column -> C10/P1O junction"),
]
VIAS = [("PYRO_G", 163.300, 49.200), ("PYRO_G", 167.500, 50.000)]


def u(tag):
    h = hashlib.md5(("pyroarm23r/" + tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


def load():
    t = open(PCB, encoding="utf-8", newline="").read()
    return t.replace("\r\n", "\n"), "\r\n" in t


def save(t, crlf):
    open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)


def near(a, b, tol=0.002):
    return abs(a - b) <= tol


def do_cut():
    t, crlf = load()
    pat = re.compile(
        r'\t\(segment\s*\n\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)'
        r'\s*\n\s*\(width ([\d.]+)\)\s*\n\s*\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\)'
        r'\s*\n\s*\(uuid "[^"]+"\)\s*\n\t\)\n')
    removed = []
    out, pos = [], 0
    for m in pat.finditer(t):
        x1, y1, x2, y2 = map(float, m.group(1, 2, 3, 4))
        lay, net = m.group(6), m.group(7)
        hit = None
        for c in CUTS:
            if c[0] == net and c[1] == lay and (
                (near(x1, c[2]) and near(y1, c[3]) and near(x2, c[4]) and near(y2, c[5])) or
                (near(x1, c[4]) and near(y1, c[5]) and near(x2, c[2]) and near(y2, c[3]))):
                hit = c
                break
        if hit:
            out.append(t[pos:m.start()])
            pos = m.end()
            removed.append(hit)
    out.append(t[pos:])
    t = "".join(out)
    for c in CUTS:
        mark = "removed" if c in removed else "NOT FOUND"
        print("  cut %-9s %-6s (%.3f,%.3f)->(%.3f,%.3f)  %s" % (c[0], c[1], c[2], c[3], c[4], c[5], mark))
    if len(removed) != len(CUTS):
        print("FAIL: %d of %d stubs matched" % (len(removed), len(CUTS)))
        sys.exit(1)
    save(t, crlf)
    print("cut: %d stub segments removed" % len(removed))


def do_check():
    sys.path.insert(0, HERE)
    import _geom
    b = _geom.Board(GEOM, PRO)
    ok = True
    print("via checks (a through via must clear every layer):")
    for net, x, y in VIAS:
        good = b.via_ok(net, x, y, VIA_S, report=True)
        print("   %-9s (%.3f, %.3f)  %s" % (net, x, y, "OK" if good else "BLOCKED"))
        ok &= good
    print("segment checks:")
    for net, lay, x1, y1, x2, y2, w, label in ADDS:
        good = b.seg_ok(net, lay, x1, y1, x2, y2, w, report=True)
        print("   %-9s %-7s w=%-4s %-34s %s" % (net, lay, w, label, "OK" if good else "BLOCKED"))
        ok &= good
    print("\nALL CLEAR" if ok else "\nSOME ITEMS BLOCKED -- do not --add until resolved")
    sys.exit(0 if ok else 1)


def do_add():
    t, crlf = load()
    anchor = t.rindex("\n\t(segment")
    end = t.index("\n\t)", anchor) + 3
    blocks = []
    for net, lay, x1, y1, x2, y2, w, label in ADDS:
        blocks.append(
            '\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n\t\t(layer "%s")\n'
            '\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
            % (x1, y1, x2, y2, w, lay, net, u("%s/%s/%s,%s-%s,%s" % (net, lay, x1, y1, x2, y2))))
    for net, x, y in VIAS:
        blocks.append(
            '\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n\t\t(layers "F.Cu" "B.Cu")\n'
            '\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
            % (x, y, VIA_S, VIA_D, net, u("via/%s/%s,%s" % (net, x, y))))
    t = t[:end] + "".join(blocks) + t[end:]
    save(t, crlf)
    print("add: %d segments + %d vias" % (len(ADDS), len(VIAS)))


if __name__ == "__main__":
    if "--cut" in sys.argv:
        do_cut()
    elif "--check" in sys.argv:
        do_check()
    elif "--add" in sys.argv:
        do_add()
    else:
        print(__doc__ or "use --cut | --check | --add")
