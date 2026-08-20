# Re-route the pyro section around the upsized 5x6 FETs.
#   --cut     delete copper left over from the old (small) footprints, plus 2 stale vias
#   --check   clearance-check every planned addition against pcbnew-dumped geometry
#   --add     lay the new copper
# Re-dump geom.json between phases so --check sees the board as it will actually be.
#
# Notes on the routing choices:
# * Drains need NO trace: at rot 270 each tab overlaps its connector's pin-1 pad directly, same net.
# * Gates approach each FET from the EAST, because pad 5 (gate) is the eastern-most lead and pads
#   6/7/8 (GND) sit west of it -- coming from the west would cross the source pads.
# * Q21's gate has to cross the 7.4V_RAW source rail (same order-inversion as before). It crosses on
#   B.Cu between two vias placed either side of Q21's tab, both north of In2's CS_IMU trace (51.564)
#   which is the southern via fence in this area.
# * The SENS_DET via lands inside Q21's source pad, so it slides west along its own In2 trace to
#   164.4 -- still inside the 0.81 mm window between CS_IMU (51.564) and INT_IMU (53.183).
import hashlib, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
PRO = os.path.join(HERE, "Impulse_2.3.kicad_pro")
GEOM = os.path.join(HERE, "geom.json")

CUT_TRACKS = [
    ("PYRO1", "B.Cu", 167.767, 46.383, 161.349, 52.801),
    ("PYRO2", "B.Cu", 158.369, 46.387, 158.369, 50.888),
    ("PYRO3", "B.Cu", 151.429, 48.901, 148.971, 46.443),
    ("PYRO4", "B.Cu", 139.593, 46.507, 139.573, 46.507),
    ("PYRO4", "B.Cu", 146.555, 53.468, 139.593, 46.507),
    ("PYRO_G", "F.Cu", 167.500, 52.200, 165.420, 51.889),
    ("PYRO_G", "F.Cu", 167.500, 50.000, 167.500, 52.200),
    ("PYRO_G", "F.Cu", 163.490, 49.095, 163.300, 49.200),
    ("PYRO_G", "In1.Cu", 163.300, 49.200, 167.500, 50.000),
    ("7.4V_RAW", "F.Cu", 163.490, 50.745, 165.420, 50.619),
    ("7.4V_RAW", "F.Cu", 165.420, 50.619, 165.420, 48.079),
    ("7.4V", "F.Cu", 170.370, 51.889, 170.370, 48.079),
    ("SENS_DET", "In1.Cu", 167.939, 53.063, 168.628, 52.374),
    ("7.4V", "F.Cu", 170.370, 49.349, 169.496, 48.841),
]
CUT_VIAS = [("PYRO_G", 163.300, 49.200), ("PYRO_G", 167.500, 50.000),
            ("SENS_DET", 168.628, 52.374)]

W_SIG, W_RAIL = 0.25, 1.2
ADDS = [
    ("PYRO1", "B.Cu", 161.349, 52.801, 169.700, 52.801, W_SIG, "PYRO1 east"),
    ("PYRO1", "B.Cu", 169.700, 52.801, 169.700, 48.225, W_SIG, "PYRO1 north"),
    ("PYRO1", "B.Cu", 169.700, 48.225, 169.047, 48.225, W_SIG, "PYRO1 -> Q16.5"),
    ("PYRO3", "B.Cu", 151.429, 48.901, 150.251, 48.225, W_SIG, "PYRO3 -> Q19.5"),
    ("PYRO4", "B.Cu", 146.555, 53.468, 145.600, 50.800, W_SIG, "PYRO4 dodge SENS_DET via"),
    ("PYRO4", "B.Cu", 145.600, 50.800, 140.853, 48.225, W_SIG, "PYRO4 -> Q18.5"),
    ("PYRO_G", "F.Cu", 163.490, 49.095, 164.700, 49.500, W_SIG, "R67.2 -> via down"),
    ("PYRO_G", "B.Cu", 164.700, 49.500, 171.000, 50.800, W_SIG, "B.Cu under the tab"),
    ("PYRO_G", "F.Cu", 171.000, 50.800, 171.000, 52.709, W_SIG, "east of the tab, south"),
    ("PYRO_G", "F.Cu", 171.000, 52.709, 170.050, 52.709, W_SIG, "-> Q21.4 gate"),
    ("7.4V_RAW", "F.Cu", 163.490, 50.745, 165.890, 52.709, W_RAIL, "R67.1 -> Q21 source"),
    ("7.4V_RAW", "F.Cu", 165.890, 52.709, 168.430, 52.709, W_RAIL, "bond Q21 source pads"),
    ("SENS_DET", "In1.Cu", 167.939, 53.063, 164.400, 52.374, W_SIG, "SENS_DET to relocated via"),
]
ADD_VIAS = [("PYRO_G", 164.700, 49.500), ("PYRO_G", 171.000, 50.800),
            ("SENS_DET", 164.400, 52.374)]
VIA_S, VIA_D = 0.6, 0.3


def u(tag):
    h = hashlib.md5(("upsizeroute/" + tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


def load():
    t = open(PCB, encoding="utf-8", newline="").read()
    return t.replace("\r\n", "\n"), "\r\n" in t


def save(t, crlf):
    open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)


def near(a, b, tol=0.003):
    return abs(a - b) <= tol


def do_cut():
    t, crlf = load()
    pat = re.compile(
        r'\t\(segment\s*\n\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)'
        r'\s*\n\s*\(width ([\d.]+)\)\s*\n\s*\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\)'
        r'\s*\n\s*\(uuid "[^"]+"\)\s*\n\t\)\n')
    got, out, pos = [], [], 0
    for m in pat.finditer(t):
        x1, y1, x2, y2 = map(float, m.group(1, 2, 3, 4))
        lay, net = m.group(6), m.group(7)
        for c in CUT_TRACKS:
            if c[0] == net and c[1] == lay and (
                (near(x1, c[2]) and near(y1, c[3]) and near(x2, c[4]) and near(y2, c[5])) or
                (near(x1, c[4]) and near(y1, c[5]) and near(x2, c[2]) and near(y2, c[3]))):
                out.append(t[pos:m.start()])
                pos = m.end()
                got.append(c)
                break
    out.append(t[pos:])
    t = "".join(out)
    for c in CUT_TRACKS:
        print("  cut %-10s %-7s (%.3f,%.3f)->(%.3f,%.3f) %s"
              % (c[0], c[1], c[2], c[3], c[4], c[5], "ok" if c in got else "NOT FOUND"))
    vpat = re.compile(
        r'\t\(via\s*\n\s*\(at ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(size [\d.]+\)\s*\n\s*\(drill [\d.]+\)'
        r'\s*\n\s*\(layers "[^"]+" "[^"]+"\)\s*\n\s*\(net "([^"]+)"\)\s*\n\s*\(uuid "[^"]+"\)\s*\n\t\)\n')
    gotv, out, pos = [], [], 0
    for m in vpat.finditer(t):
        x, y, net = float(m.group(1)), float(m.group(2)), m.group(3)
        for c in CUT_VIAS:
            if c[0] == net and near(x, c[1]) and near(y, c[2]):
                out.append(t[pos:m.start()])
                pos = m.end()
                gotv.append(c)
                break
    out.append(t[pos:])
    t = "".join(out)
    for c in CUT_VIAS:
        print("  cut via %-9s (%.3f,%.3f) %s" % (c[0], c[1], c[2], "ok" if c in gotv else "NOT FOUND"))
    if len(got) != len(CUT_TRACKS) or len(gotv) != len(CUT_VIAS):
        print("FAIL: %d/%d tracks, %d/%d vias" % (len(got), len(CUT_TRACKS), len(gotv), len(CUT_VIAS)))
        sys.exit(1)
    save(t, crlf)
    print("cut: %d tracks, %d vias" % (len(got), len(gotv)))


def do_check():
    sys.path.insert(0, HERE)
    import _geom
    b = _geom.Board(GEOM, PRO)
    ok = True
    for net, x, y in ADD_VIAS:
        good = b.via_ok(net, x, y, VIA_S, report=True)
        print("   via %-9s (%.3f,%.3f) %s" % (net, x, y, "OK" if good else "BLOCKED"))
        ok &= good
    for net, lay, x1, y1, x2, y2, w, lab in ADDS:
        good = b.seg_ok(net, lay, x1, y1, x2, y2, w, report=True)
        print("   %-9s %-7s w=%-5s %-28s %s" % (net, lay, w, lab, "OK" if good else "BLOCKED"))
        ok &= good
    print("\nALL CLEAR" if ok else "\nBLOCKED -- resolve before --add")
    sys.exit(0 if ok else 1)


def do_add():
    t, crlf = load()
    anchor = t.rindex("\n\t(segment")
    end = t.index("\n\t)", anchor) + 3
    blocks = []
    for net, lay, x1, y1, x2, y2, w, lab in ADDS:
        blocks.append('\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n'
                      '\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                      % (x1, y1, x2, y2, w, lay, net,
                         u("%s/%s/%s,%s-%s,%s" % (net, lay, x1, y1, x2, y2))))
    for net, x, y in ADD_VIAS:
        blocks.append('\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n'
                      '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                      % (x, y, VIA_S, VIA_D, net, u("via/%s/%s,%s" % (net, x, y))))
    save(t[:end] + "".join(blocks) + t[end:], crlf)
    print("add: %d segments + %d vias" % (len(ADDS), len(ADD_VIAS)))


if __name__ == "__main__":
    {"--cut": do_cut, "--check": do_check, "--add": do_add}[sys.argv[1]]()
