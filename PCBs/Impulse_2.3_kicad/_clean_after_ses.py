# Post-SES cleanup, by TEXT SURGERY (pcbnew Remove loops corrupt this board -- documented).
#   1. delete freerouting's GND tracks/vias: the 4 GND pours serve that net, and stray GND traces
#      fence later hand-routing
#   2. delete degenerate <0.01 mm segments, which show up as phantom shorts
import os, re, sys

PCB = sys.argv[1] if len(sys.argv) > 1 else "Impulse_2.3.kicad_pcb"
t = open(PCB, encoding="utf-8", newline="").read()
crlf = "\r\n" in t
t = t.replace("\r\n", "\n")


def bal(s, i):
    d = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            d += 1
        elif s[j] == ")":
            d -= 1
            if d == 0:
                return j + 1


def drop(text, pred, opener):
    out, pos, n = [], 0, 0
    for m in re.finditer(r"^\t\(%s\b" % opener, text, re.M):
        if m.start() < pos:
            continue
        e = bal(text, m.start() + 1)
        blk = text[m.start():e]
        if pred(blk):
            while e < len(text) and text[e] in "\r\n":
                e += 1
            out.append(text[pos:m.start()])
            pos = e
            n += 1
    out.append(text[pos:])
    return "".join(out), n


def is_gnd(blk):
    return '(net "GND")' in blk


def is_degenerate(blk):
    m = re.search(r"\(start ([-\d.]+) ([-\d.]+)\)\s*\n\s*\(end ([-\d.]+) ([-\d.]+)\)", blk)
    if not m:
        return False
    x1, y1, x2, y2 = map(float, m.groups())
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 < 0.01


t, n_gnd_s = drop(t, is_gnd, "segment")
t, n_gnd_v = drop(t, is_gnd, "via")
t, n_deg = drop(t, is_degenerate, "segment")
open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
print("removed %d GND segments, %d GND vias, %d degenerate stubs" % (n_gnd_s, n_gnd_v, n_deg))
print("remaining: %d segments, %d vias" % (len(re.findall(r"^\t\(segment", t, re.M)),
                                           len(re.findall(r"^\t\(via", t, re.M))))
