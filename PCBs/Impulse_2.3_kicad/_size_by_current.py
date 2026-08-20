# Size every power-net segment for the current it actually carries, uniformly.
#
# Replaces the old "widen each segment as far as its neighbours allow" approach, which produced
# ragged nets (1.0/1.2/1.4/1.6/2.0/2.5 mm along one run -- so the narrowest link silently set the
# rating) and inflated ~0.6 mA continuity-sense stubs to 2.5 mm.
#
# Targets come from _current_budget.py:
#   18 A pulse, <=100 C adiabatic over the 900 ms firmware timeout  -> 3.44 mm ideal, 2.5 mm floor
#   2.4-2.8 A continuous, IPC-2221 external 1 oz, 10 C rise         -> 1.0-1.25 mm, use 1.2 mm
#   sense / gate / decoupling stubs (uA-mA)                          -> 0.3 mm
#
# Pass 1 narrows the stubs (which frees space), pass 2 widens the cores into it.
import json, math, os, re, shutil, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _geom
from _classify_nets import classify, POWER_PADS

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
CX, CY, RAD, EDGE = 152.7, 62.9, 35.0, 0.5

STUB_W = 0.3
CORE_TARGET = {            # (ideal, floor)
    "7.4V": (3.4, 2.5), "7.4V_RAW": (3.4, 2.5),
    "Net-(P1O-Pin_1)": (3.4, 2.5), "Net-(P2O-Pin_1)": (3.4, 2.5),
    "Net-(P3O-Pin_1)": (3.4, 2.5), "Net-(P4O-Pin_1)": (3.4, 2.5),
    "VBAT": (1.6, 1.2), "VBAT_RAW": (1.6, 1.2),
}
LADDER = [3.4, 3.0, 2.5, 2.0, 1.6, 1.4, 1.2, 1.0, 0.8, 0.6, 0.5, 0.4, 0.3]


def edge_ok(x1, y1, x2, y2, w):
    lim = RAD - EDGE - w / 2.0
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.1) + 1)
    return all(math.hypot(x1 + (x2 - x1) * i / n - CX, y1 + (y2 - y1) * i / n - CY) <= lim
               for i in range(n + 1))


def same(a, b):
    return abs(a - b) < 1e-4


def apply_widths(plan):
    t = open(PCB, encoding="utf-8", newline="").read()
    crlf = "\r\n" in t
    t = t.replace("\r\n", "\n")
    pat = re.compile(
        r'(\t\(segment\s*\n\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)'
        r'\s*\n\s*\(width )([\d.]+)(\)\s*\n\s*\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\))')
    n = [0]

    def repl(m):
        x1, y1, x2, y2 = float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))
        lay, net = m.group(8), m.group(9)
        for (nt, ly, a, bb, c, d), w in plan.items():
            if nt == net and ly == lay and same(a, x1) and same(bb, y1) and same(c, x2) and same(d, y2):
                if abs(w - float(m.group(6))) > 1e-6:
                    n[0] += 1
                    return m.group(1) + ("%g" % w) + m.group(7)
        return m.group(0)

    t = pat.sub(repl, t)
    open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
    return n[0]


def run():
    b = _geom.Board(os.path.join(HERE, "geom.json"), os.path.join(HERE, "Impulse_2.3.kicad_pro"))
    if not os.path.exists(PCB + ".pre_sizing"):
        shutil.copy2(PCB, PCB + ".pre_sizing")

    plan = {}
    report = []
    # ---- pass 1: stubs down to signal width ----
    for net in POWER_PADS:
        core, stub, _ = classify(b, net)
        for s in stub:
            plan[(net, s["layer"], s["x1"], s["y1"], s["x2"], s["y2"])] = STUB_W
        report.append((net, len(core), len(stub)))
    nchg = apply_widths(plan)
    print("pass 1 -- stubs to %.1f mm: %d segments narrowed" % (STUB_W, nchg))
    for net, nc, ns in report:
        print("   %-18s core %2d seg, stub %2d seg" % (net, nc, ns))
    return plan


if __name__ == "__main__":
    run()
