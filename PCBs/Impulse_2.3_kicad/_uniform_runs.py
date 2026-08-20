# Give every contiguous power run ONE width.
#
# WHY: maximising each segment individually is what produced the ragged look -- a single 15 mm run
# carrying 3.40 / 2.80 / 3.00 / 2.80 / 2.50 / 2.80 / 3.00 mm, with segments as short as 0.15 mm.
# Those steps are A* corner artifacts (each 45-degree corner clears marginally less, so it accepted
# a marginally different maximum), not design intent. A reader cannot tell which changes mean
# something, which is the real cost.
#
# RULE: split each net into runs of segments joined end-to-end at degree-2 nodes (a degree-3+ node
# is a genuine branch, where a width change IS meaningful). Give every segment in a run the same
# width: the widest that EVERY segment in that run can carry. Width then changes only at branches
# and at real constrictions between runs.
#
# Interspersed constrictions are why a two-width compromise looks worse than one width: the
# narrow spots are scattered along the run, so any two-value scheme alternates repeatedly.
import math, os, re, shutil, sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _geom
from _size_by_current import edge_ok

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
# net -> (cap, FLOOR). The floor is the width the CURRENT requires; a run that cannot reach it
# uniformly is left ragged and reported, never narrowed to fit. Without this the rule is actively
# dangerous: taking the minimum across a run dropped a 47 mm VBAT run from 1.2 mm to 0.40 mm,
# i.e. below its 1.0 mm requirement, purely to make the picture tidy.
NETS = {"7.4V": (3.4, 2.5), "7.4V_RAW": (3.4, 2.5), "Net-(P1O-Pin_1)": (3.4, 2.5),
        "Net-(P2O-Pin_1)": (3.4, 2.5), "Net-(P3O-Pin_1)": (3.4, 2.5),
        "Net-(P4O-Pin_1)": (3.4, 2.0), "VBAT": (1.6, 1.2), "VBAT_RAW": (1.6, 1.2)}
LADDER = [3.4, 3.0, 2.8, 2.5, 2.0, 1.6, 1.4, 1.2, 1.0, 0.8, 0.6, 0.4, 0.3]
SIGNAL = 0.35          # anything at or below this is a sense/gate stub -- leave it alone


def runs_of(segs):
    node = defaultdict(list)
    for s in segs:
        node[(s["layer"], round(s["x1"], 3), round(s["y1"], 3))].append(s)
        node[(s["layer"], round(s["x2"], 3), round(s["y2"], 3))].append(s)
    adj = defaultdict(list)
    for k, v in node.items():
        if len(v) == 2:                    # degree-2 = mid-run; 3+ = branch, keep runs separate
            a, c = v
            adj[id(a)].append(c)
            adj[id(c)].append(a)
    byid = {id(s): s for s in segs}
    seen, out = set(), []
    for s in segs:
        if id(s) in seen:
            continue
        stack, grp = [s], []
        seen.add(id(s))
        while stack:
            cur = stack.pop()
            grp.append(cur)
            for nb in adj.get(id(cur), []):
                if id(nb) not in seen:
                    seen.add(id(nb))
                    stack.append(nb)
        out.append(grp)
    return out


def main():
    b = _geom.Board(os.path.join(HERE, "geom.json"), os.path.join(HERE, "Impulse_2.3.kicad_pro"))
    if not os.path.exists(PCB + ".pre_uniform"):
        shutil.copy2(PCB, PCB + ".pre_uniform")
    keep = b.tracks
    plan = {}
    print("%-18s %-5s %-8s %-9s %s" % ("net", "segs", "length", "was", "-> uniform"))
    for net, (cap, floor) in NETS.items():
        segs = [t for t in b.tracks if t["net"] == net and t["w"] > SIGNAL]
        for grp in runs_of(segs):
            if len(grp) < 2:
                continue
            widest = None
            for w in LADDER:
                if w > cap:
                    continue
                good = True
                for s in grp:
                    b.tracks = [o for o in keep if o is not s]
                    ok = (b.seg_ok(net, s["layer"], s["x1"], s["y1"], s["x2"], s["y2"], w)
                          and edge_ok(s["x1"], s["y1"], s["x2"], s["y2"], w))
                    b.tracks = keep
                    if not ok:
                        good = False
                        break
                if good:
                    widest = w
                    break
            if widest is None or widest < floor - 1e-6:
                was = sorted({round(s["w"], 2) for s in grp})
                if len(was) > 1:
                    print("%-18s %-5d %-8.2f %-9s -> LEFT AS-IS (uniform would be %s, below the %.1f mm floor)"
                          % (net, len(grp), sum(math.hypot(s["x2"]-s["x1"], s["y2"]-s["y1"]) for s in grp),
                             ",".join("%g" % w for w in was),
                             ("%.2f" % widest) if widest else "unroutable", floor))
                continue
            was = sorted({round(s["w"], 2) for s in grp})
            if len(was) == 1 and abs(was[0] - widest) < 1e-6:
                continue
            L = sum(math.hypot(s["x2"] - s["x1"], s["y2"] - s["y1"]) for s in grp)
            print("%-18s %-5d %-8.2f %-9s -> %.2f mm" % (net, len(grp), L,
                  ",".join("%g" % w for w in was), widest))
            for s in grp:
                plan[(net, s["layer"], s["x1"], s["y1"], s["x2"], s["y2"])] = widest

    t = open(PCB, encoding="utf-8", newline="").read()
    crlf = "\r\n" in t
    t = t.replace("\r\n", "\n")
    pat = re.compile(
        r'(\t\(segment\s*\n\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)'
        r'\s*\n\s*\(width )([\d.]+)(\)\s*\n\s*\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\))')
    n = [0]

    def repl(m):
        x1, y1, x2, y2 = map(float, m.group(2, 3, 4, 5))
        lay, net = m.group(8), m.group(9)
        for (nt, ly, a, bb, c, d), w in plan.items():
            if nt == net and ly == lay and abs(a - x1) < 1e-4 and abs(bb - y1) < 1e-4 \
                    and abs(c - x2) < 1e-4 and abs(d - y2) < 1e-4:
                if abs(w - float(m.group(6))) > 1e-6:
                    n[0] += 1
                return m.group(1) + ("%g" % w) + m.group(7)
        return m.group(0)

    t = pat.sub(repl, t)
    open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
    print("\n%d segments re-sized so each run carries one width" % n[0])


if __name__ == "__main__":
    main()
