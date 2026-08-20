# Fix the conflicts DRC found BETWEEN the newly added tracks.
#
# The --check pass validated each new segment against the board as it was BEFORE the batch, so
# additions could not see each other. Three collided:
#   * PYRO1's x=169.7 riser crossed PYRO_G's B.Cu leg  -> tracks_crossing (a real short)
#   * the relocated SENS_DET via sat on the new 7.4V_RAW rail -> shorting_items
#   * that same via was 0.002 mm from PYRO1's y=52.801 run -> clearance
# Fix: SENS_DET via moves further west to 162.5, and PYRO1 runs south of it at y=53.3 then rises
# at x=172.0, east of PYRO_G's via at (171.0, 50.8).
#
# This pass checks every replacement against the CURRENT board, additions included.
import hashlib, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
PRO = os.path.join(HERE, "Impulse_2.3.kicad_pro")
GEOM = os.path.join(HERE, "geom.json")
W = 0.25
VIA_S, VIA_D = 0.6, 0.3

CUT_TRACKS = [
    ("PYRO1", "B.Cu", 161.349, 52.801, 169.700, 52.801),
    ("PYRO1", "B.Cu", 169.700, 52.801, 169.700, 48.225),
    ("PYRO1", "B.Cu", 169.700, 48.225, 169.047, 48.225),
    ("SENS_DET", "In1.Cu", 167.939, 53.063, 164.400, 52.374),
]
CUT_VIAS = [("SENS_DET", 164.400, 52.374)]
ADDS = [
    ("PYRO1", "B.Cu", 161.349, 52.801, 162.500, 53.300, W, "PYRO1 south of the SENS_DET via"),
    ("PYRO1", "B.Cu", 162.500, 53.300, 172.000, 53.300, W, "PYRO1 east"),
    ("PYRO1", "B.Cu", 172.000, 53.300, 172.000, 48.225, W, "PYRO1 riser, east of PYRO_G's via"),
    ("PYRO1", "B.Cu", 172.000, 48.225, 169.047, 48.225, W, "PYRO1 -> Q16.5"),
    ("SENS_DET", "In1.Cu", 167.939, 53.063, 162.500, 52.374, W, "SENS_DET to relocated via"),
]
ADD_VIAS = [("SENS_DET", 162.500, 52.374)]


def u(tag):
    h = hashlib.md5(("fixconf/" + tag).encode()).hexdigest()
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
    save("".join(out), crlf)
    print("cut %d/%d tracks, %d/%d vias" % (len(got), len(CUT_TRACKS), len(gotv), len(CUT_VIAS)))
    if len(got) != len(CUT_TRACKS) or len(gotv) != len(CUT_VIAS):
        sys.exit(1)


def do_check():
    sys.path.insert(0, HERE)
    import _geom
    b = _geom.Board(GEOM, PRO)
    ok = True
    # fold the planned additions into the model so they are checked against EACH OTHER too
    pend_t = [dict(net=n, layer=l, x1=a, y1=c, x2=d, y2=e, w=w) for n, l, a, c, d, e, w, _ in ADDS]
    pend_v = [dict(net=n, x=x, y=y, size=VIA_S, d=VIA_D) for n, x, y in ADD_VIAS]
    for i, (net, x, y) in enumerate(ADD_VIAS):
        b.tracks = b.tracks + pend_t
        b.vias = b.vias + [v for j, v in enumerate(pend_v) if j != i]
        good = b.via_ok(net, x, y, VIA_S, report=True)
        print("   via %-9s (%.3f,%.3f) %s" % (net, x, y, "OK" if good else "BLOCKED"))
        ok &= good
        b.tracks = b.tracks[:-len(pend_t)]
        b.vias = b.vias[:len(b.vias) - (len(pend_v) - 1)]
    base_t, base_v = list(b.tracks), list(b.vias)
    for i, (net, lay, x1, y1, x2, y2, w, lab) in enumerate(ADDS):
        b.tracks = base_t + [t for j, t in enumerate(pend_t) if j != i]
        b.vias = base_v + pend_v
        good = b.seg_ok(net, lay, x1, y1, x2, y2, w, report=True)
        print("   %-9s %-7s %-38s %s" % (net, lay, lab, "OK" if good else "BLOCKED"))
        ok &= good
    b.tracks, b.vias = base_t, base_v
    print("\nALL CLEAR" if ok else "\nBLOCKED")
    sys.exit(0 if ok else 1)


def do_add():
    t, crlf = load()
    anchor = t.rindex("\n\t(segment")
    end = t.index("\n\t)", anchor) + 3
    blocks = []
    for net, lay, x1, y1, x2, y2, w, lab in ADDS:
        blocks.append('\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n'
                      '\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                      % (x1, y1, x2, y2, w, lay, net, u("%s%s%s%s%s%s" % (net, lay, x1, y1, x2, y2))))
    for net, x, y in ADD_VIAS:
        blocks.append('\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n'
                      '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                      % (x, y, VIA_S, VIA_D, net, u("via%s%s%s" % (net, x, y))))
    save(t[:end] + "".join(blocks) + t[end:], crlf)
    print("add: %d segments + %d vias" % (len(ADDS), len(ADD_VIAS)))


if __name__ == "__main__":
    {"--cut": do_cut, "--check": do_check, "--add": do_add}[sys.argv[1]]()
