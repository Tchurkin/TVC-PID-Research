# Widen the pyro rail and drains to whatever each segment's neighbours actually allow.
#
# WHY: DESIGN_LOG 2026-07-23 flagged "the 1.0 mm pyro rail trace necks (sized for brief e-match
# pulses)" as a remaining manual step, and it was never done. At ~8 A for a 900 ms pulse a
# 1.0 mm / 1 oz trace heats ~235 C adiabatically (0.035 mm^2 -> J = 2.3e8 A/m^2; rho*J^2 / cv
# = 261 K/s). Conduction into the pours pulls the real figure well below that, but it is not a
# margin worth flying. 2.5 mm cuts the power density ~6x.
#
# TWO RULES LEARNED THE HARD WAY (the first version of this script violated both, and DRC caught
# it as a P3O/7.4V short on In2 plus a trace 0.383 mm from the board edge):
#   1. Check against LIVE geometry. Sizing every segment against the pre-pass board lets two
#      neighbours both widen into each other.
#   2. Check the BOARD EDGE. The outline is a circle; a widened trace near it silently breaks the
#      0.5 mm copper-to-edge rule.
# And never narrow below the original width -- that would be a regression against 2.2, which is
# what happened when a "fix the overshoot" pass was allowed to shrink the main source rail to 0.8.
import math, os, re, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _geom

PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
GEOM = os.path.join(HERE, "geom.json")
PRO = os.path.join(HERE, "Impulse_2.3.kicad_pro")

NETS = {"7.4V", "7.4V_RAW",
        "Net-(P1O-Pin_1)", "Net-(P2O-Pin_1)", "Net-(P3O-Pin_1)", "Net-(P4O-Pin_1)"}
TRY = [2.5, 2.0, 1.6, 1.4, 1.2]
EDGE_CLEAR = 0.5


def board_circle(txt):
    m = re.search(r'\(gr_circle\s*\n\s*\(center ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)', txt)
    if m:
        cx, cy, ex, ey = map(float, m.groups())
        return cx, cy, math.hypot(ex - cx, ey - cy)
    return 152.7, 62.9, 35.0


def same(a, b):
    return abs(a - b) < 1e-4


def edge_ok(cx, cy, r, x1, y1, x2, y2, w):
    lim = r - EDGE_CLEAR - w / 2.0
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
    return all(math.hypot(x1 + (x2 - x1) * i / n - cx, y1 + (y2 - y1) * i / n - cy) <= lim
               for i in range(n + 1))


def main():
    txt = open(PCB, encoding="utf-8", newline="").read()
    crlf = "\r\n" in txt
    t_ = txt.replace("\r\n", "\n")
    cx, cy, r = board_circle(t_)
    print("outline: centre (%.2f, %.2f) r=%.2f, edge clearance %.2f mm" % (cx, cy, r, EDGE_CLEAR))

    b = _geom.Board(GEOM, PRO)
    targets = [t for t in b.tracks if t["net"] in NETS]
    # Widen the shortest segments last: long runs set the constraints, stubs fill in around them.
    order = sorted(targets, key=lambda t: -math.hypot(t["x2"] - t["x1"], t["y2"] - t["y1"]))
    print("pyro-rail segments: %d" % len(targets))

    plan = {}
    for t in order:
        keep = b.tracks
        b.tracks = [o for o in keep if o is not t]           # live list: prior widenings included
        best = None
        for w in TRY:
            if w <= t["w"] + 1e-9:
                break
            if (b.seg_ok(t["net"], t["layer"], t["x1"], t["y1"], t["x2"], t["y2"], w)
                    and edge_ok(cx, cy, r, t["x1"], t["y1"], t["x2"], t["y2"], w)):
                best = w
                break
        b.tracks = keep
        if best:
            t["w"] = best                                    # commit so later segments see it
            plan[id(t)] = (t, best)

    if not plan:
        print("nothing to widen")
        return
    if not os.path.exists(PCB + ".pre_widen"):
        shutil.copy2(PCB, PCB + ".pre_widen")

    pat = re.compile(
        r'(\t\(segment\s*\n\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)'
        r'\s*\n\s*\(width )([\d.]+)(\)\s*\n\s*\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\))')
    done = [0]

    def repl(m):
        x1, y1, x2, y2 = float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))
        lay, net, oldw = m.group(8), m.group(9), float(m.group(6))
        for t, w in plan.values():
            if (t["net"] == net and t["layer"] == lay and w > oldw
                    and same(t["x1"], x1) and same(t["y1"], y1)
                    and same(t["x2"], x2) and same(t["y2"], y2)):
                done[0] += 1
                return m.group(1) + ("%g" % w) + m.group(7)
        return m.group(0)

    t_ = pat.sub(repl, t_)
    open(PCB, "w", encoding="utf-8", newline="").write(t_.replace("\n", "\r\n") if crlf else t_)
    print("widened %d of %d segments (%d rewritten)" % (len(plan), len(targets), done[0]))
    from collections import Counter
    c = Counter((t["net"], w) for t, w in plan.values())
    for (net, w), n in sorted(c.items()):
        print("   %-16s -> %.1f mm  x%d" % (net, w, n))


if __name__ == "__main__":
    main()
