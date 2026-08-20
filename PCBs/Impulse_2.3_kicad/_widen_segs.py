# Widen specific existing segments, verified against LIVE geometry (excluding the segment itself).
#
# Separate from _size_by_current.py because that one plans whole nets; this one is surgical, for the
# individual legs the 2026-08-19 audit found undersized once inner-layer copper was accounted for.
import os, re, shutil, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _geom
from _size_by_current import edge_ok

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")


def widen(targets, backup=None):
    """targets: list of (net, layer, x1, y1, x2, y2, ladder) -- ladder widest first."""
    b = _geom.Board(os.path.join(HERE, "geom.json"), os.path.join(HERE, "Impulse_2.3.kicad_pro"))
    keep = b.tracks
    plan = {}
    for (net, lay, x1, y1, x2, y2, ladder) in targets:
        me = [t for t in keep if t["net"] == net and t["layer"] == lay
              and abs(t["x1"] - x1) < 1e-3 and abs(t["y1"] - y1) < 1e-3
              and abs(t["x2"] - x2) < 1e-3 and abs(t["y2"] - y2) < 1e-3]
        if not me:
            print("  NOT FOUND %s %s (%.3f,%.3f)->(%.3f,%.3f)" % (net, lay, x1, y1, x2, y2))
            continue
        cur = me[0]["w"]
        b.tracks = [o for o in keep if o is not me[0]]
        got = None
        for w in ladder:
            if w <= cur:
                break
            if b.seg_ok(net, lay, x1, y1, x2, y2, w) and edge_ok(x1, y1, x2, y2, w):
                got = w
                break
        b.tracks = keep
        print("  %-18s %-7s %.2f -> %s" % (net, lay, cur,
              ("%.2f" % got) if got else "no room, LEFT AS-IS"))
        if got:
            plan[(net, lay, round(x1, 3), round(y1, 3), round(x2, 3), round(y2, 3))] = got
    if not plan:
        return 0
    if backup and not os.path.exists(PCB + backup):
        shutil.copy2(PCB, PCB + backup)
    t = open(PCB, encoding="utf-8", newline="").read()
    crlf = "\r\n" in t
    t = t.replace("\r\n", "\n")
    pat = re.compile(
        r'(\t\(segment\s*\n\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)'
        r'\s*\n\s*\(width )([\d.]+)(\)\s*\n\s*\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\))')
    n = [0]

    def repl(m):
        k = (m.group(9), m.group(8), round(float(m.group(2)), 3), round(float(m.group(3)), 3),
             round(float(m.group(4)), 3), round(float(m.group(5)), 3))
        if k in plan:
            n[0] += 1
            return m.group(1) + ("%g" % plan[k]) + m.group(7)
        return m.group(0)

    t = pat.sub(repl, t)
    open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
    print("widened %d segments" % n[0])
    return n[0]


if __name__ == "__main__":
    L = [3.4, 3.0, 2.8, 2.5, 2.2, 2.0, 1.8, 1.6, 1.4]
    print("=== widening the firing legs the audit found undersized ===")
    widen([
        ("7.4V_RAW", "F.Cu", 164.000, 47.200, 164.236, 48.079, L),
        ("7.4V_RAW", "F.Cu", 164.236, 48.079, 164.236, 50.619, L),
        ("Net-(P4O-Pin_1)", "B.Cu", 138.598, 43.586, 138.598, 48.467, L),
    ], ".pre_widen2")
