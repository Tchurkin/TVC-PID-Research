# Copper rework for Q20 (main battery switch) after the AON6403 swap.
#
# Placement: (175.500, 79.500) rot 0 on B.Cu -- EAST of the buzzer. Q20 could not stay in its old
# pocket: the window between the BUZZER's through-hole pads (bottom edge 78.44) and R66 (top edge
# 82.96) is 4.52 mm and a DFN5x6 needs 4.96 + clearance. East is not a compromise, it is where the
# net already goes: VBAT leaves Q20 EASTWARD to C39, then via -> In2 -> north-west to R54/U2/U3.
# The drain tab (x 174.4..179.0) now sits directly on that path.
#
# THE ONE REAL TRAP, caught by the clearance checker and worth remembering: the old
# (169.332,79.780)->(176.504,79.780) VBAT segment ran from the OLD drain pad eastward, and the new
# SOURCE pads land right on top of it -- a VBAT/VBAT_RAW short. The placement scan missed it because
# it treated VBAT as "Q20's own net" and skipped it; VBAT is the DRAIN's net, and the source pads
# must still clear it. That whole segment is now obsolete (nothing else on VBAT touches its west
# end), so it is deleted rather than re-routed.
#
# The gate needs a via jog for the same reason Q21's did: MAIN_G comes from R66 in the south, but
# the gate pad is on the far side of the source-pad column, so it crosses the source lane. It is a
# ~67 uA signal; In1 is clear underneath.
import hashlib, os, re, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
PRO = os.path.join(HERE, "Impulse_2.3.kicad_pro")
GEOM = os.path.join(HERE, "geom.json")
VIA_S, VIA_D = 0.6, 0.3

CUTS = [
    # obsolete: ran from the old SOT-23 drain pad east; the tab now covers this path, and the new
    # source pads sit on top of it (VBAT vs VBAT_RAW short)
    ("VBAT",   "B.Cu", 169.332, 79.780, 176.504, 79.780),
    # old gate stubs to the SOT-23 pad
    ("MAIN_G", "B.Cu", 170.282, 81.655, 170.157, 81.780),
    ("MAIN_G", "B.Cu", 170.157, 81.780, 170.157, 83.436),
]

ADDS = [
    ("MAIN_G",   "B.Cu",   170.157, 83.436, 171.200, 83.000, 0.25, "R66 -> via down"),
    ("MAIN_G",   "In1.Cu", 171.200, 83.000, 173.500, 77.000, 0.25, "In1 under the VBAT/VBAT_RAW lanes"),
    ("MAIN_G",   "B.Cu",   173.500, 77.000, 172.775, 77.595, 0.25, "via up -> Q20.4 gate"),
    ("VBAT_RAW", "B.Cu",   168.382, 81.655, 172.775, 81.405, 1.2,  "MI lane -> Q20.1 source"),
    ("VBAT_RAW", "B.Cu",   172.775, 81.405, 172.775, 78.865, 1.2,  "bond source pads 1-2-3"),
]
VIAS = [("MAIN_G", 171.200, 83.000), ("MAIN_G", 173.500, 77.000)]


def u(tag):
    h = hashlib.md5(("q20route/" + tag).encode()).hexdigest()
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
    if not os.path.exists(PCB + ".pre_q20"):
        shutil.copy2(PCB, PCB + ".pre_q20")
    pat = re.compile(
        r'\t\(segment\s*\n\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)'
        r'\s*\n\s*\(width ([\d.]+)\)\s*\n\s*\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\)'
        r'\s*\n\s*\(uuid "[^"]+"\)\s*\n\t\)\n')
    removed, out, pos = [], [], 0
    for m in pat.finditer(t):
        x1, y1, x2, y2 = map(float, m.group(1, 2, 3, 4))
        lay, net = m.group(6), m.group(7)
        for c in CUTS:
            if c[0] == net and c[1] == lay and (
                (near(x1, c[2]) and near(y1, c[3]) and near(x2, c[4]) and near(y2, c[5])) or
                (near(x1, c[4]) and near(y1, c[5]) and near(x2, c[2]) and near(y2, c[3]))):
                out.append(t[pos:m.start()])
                pos = m.end()
                removed.append(c)
                break
    out.append(t[pos:])
    t = "".join(out)
    for c in CUTS:
        print("  cut %-9s %-6s (%.3f,%.3f)->(%.3f,%.3f)  %s"
              % (c[0], c[1], c[2], c[3], c[4], c[5], "removed" if c in removed else "NOT FOUND"))
    if len(removed) != len(CUTS):
        print("FAIL: %d of %d matched" % (len(removed), len(CUTS)))
        sys.exit(1)
    save(t, crlf)
    print("cut: %d segments removed" % len(removed))


def do_check():
    sys.path.insert(0, HERE)
    import _geom
    b = _geom.Board(GEOM, PRO)
    ok = True
    for net, x, y in VIAS:
        g = b.via_ok(net, x, y, VIA_S, report=True)
        print("   via %-9s (%.3f, %.3f)  %s" % (net, x, y, "OK" if g else "BLOCKED"))
        ok &= g
    for net, lay, x1, y1, x2, y2, w, label in ADDS:
        g = b.seg_ok(net, lay, x1, y1, x2, y2, w, report=True)
        print("   %-9s %-7s w=%-4s %-36s %s" % (net, lay, w, label, "OK" if g else "BLOCKED"))
        ok &= g
    print("\nALL CLEAR" if ok else "\nBLOCKED -- do not --add")
    sys.exit(0 if ok else 1)


def do_add():
    t, crlf = load()
    anchor = t.rindex("\n\t(segment")
    end = t.index("\n\t)", anchor) + 3
    blocks = []
    for net, lay, x1, y1, x2, y2, w, label in ADDS:
        blocks.append('\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n'
                      '\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                      % (x1, y1, x2, y2, w, lay, net,
                         u("%s/%s/%s,%s-%s,%s" % (net, lay, x1, y1, x2, y2))))
    for net, x, y in VIAS:
        blocks.append('\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n'
                      '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                      % (x, y, VIA_S, VIA_D, net, u("via/%s/%s,%s" % (net, x, y))))
    t = t[:end] + "".join(blocks) + t[end:]
    save(t, crlf)
    print("add: %d segments + %d vias" % (len(ADDS), len(VIAS)))


if __name__ == "__main__":
    {"--cut": do_cut, "--check": do_check, "--add": do_add}.get(
        sys.argv[1] if len(sys.argv) > 1 else "", lambda: print("use --cut | --check | --add"))()
