#!/usr/bin/env python3
# Rebuild component silkscreens after the manual spacing-out:
#  - delete ALL scattered board-level silk (it no longer tracks the moved parts)
#  - for every component, generate a clean body-outline rectangle (pad extents + 0.3mm) as
#    footprint-level fp_line, with a pin-1 chamfer where a pad "1" exists, so silk moves with the part.
#  Skips mounting holes (H*) and J_SENSOR (they already have correct footprint silk).
import re, math, uuid
PCB = "Impulse_2.1.kicad_pcb"
M = 0.3            # outline margin beyond pad edges
p = open(PCB, encoding="utf-8").read()
def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
def nid(): return str(uuid.uuid4())
def g(v): return ('%.4f' % v).rstrip('0').rstrip('.')
SKIP = lambda r: r.startswith('H') or r == 'J_SENSOR'

# --- collect footprint blocks: ref, silk layer, pad extents (local), pad-1 pos ---
fps = []
for m in re.finditer(r'\(footprint ', p):
    fs = m.start(); fe = bend(p, fs); b = p[fs:fe]
    rr = re.search(r'\(property "Reference" "([^"]+)"', b)
    lay = re.search(r'\(layer "([BF])\.Cu"\)', b)
    if not rr: continue
    silk = "%s.SilkS" % (lay.group(1) if lay else "F")
    xs = []; ys = []; p1 = None
    for pm in re.finditer(r'\(pad ', b):
        pb = b[pm.start():bend(b, pm.start())]
        nm = re.match(r'\(pad "([^"]*)"', pb).group(1)
        at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', pb)
        sz = re.search(r'\(size ([\-\d.]+) ([\-\d.]+)\)', pb)
        if not at: continue
        px, py = float(at.group(1)), float(at.group(2))
        h = (max(float(sz.group(1)), float(sz.group(2))) / 2) if sz else 0.8
        xs += [px - h, px + h]; ys += [py - h, py + h]
        if nm == "1": p1 = (px, py)
    fps.append(dict(ref=rr.group(1), fs=fs, fe=fe, block=b, silk=silk,
                    box=(min(xs), min(ys), max(xs), max(ys)) if xs else None, p1=p1))

def rect_lines(box, silk, p1):
    x0, y0, x1, y1 = box[0]-M, box[1]-M, box[2]+M, box[3]+M
    c = min(0.8, (x1-x0)/3, (y1-y0)/3)              # chamfer size
    # choose corner nearest pad 1 (default top-left)
    corner = "TL"
    if p1:
        cs = {"TL": (x0, y0), "TR": (x1, y0), "BL": (x0, y1), "BR": (x1, y1)}
        corner = min(cs, key=lambda k: math.hypot(p1[0]-cs[k][0], p1[1]-cs[k][1]))
    # rectangle edges with one chamfered corner
    if corner == "TL":  pts = [(x0+c, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0+c), (x0+c, y0)]
    elif corner == "TR": pts = [(x0, y0), (x1-c, y0), (x1, y0+c), (x1, y1), (x0, y1), (x0, y0)]
    elif corner == "BR": pts = [(x0, y0), (x1, y0), (x1, y1-c), (x1-c, y1), (x0, y1), (x0, y0)]
    else:                pts = [(x0, y0), (x1, y0), (x1, y1), (x0+c, y1), (x0, y1-c), (x0, y0)]
    out = ""
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        out += ('\t\t(fp_line (start %s %s) (end %s %s) (stroke (width 0.12) (type solid)) '
                '(layer "%s") (uuid "%s"))\n') % (g(ax), g(ay), g(bx), g(by), silk, nid())
    return out

# --- pass 1: rewrite footprint blocks (strip old fp_line silk, add outline) high->low ---
edits = []
n_out = 0
for f in fps:
    if SKIP(f['ref']) or not f['box']: continue
    b = f['block']
    b = re.sub(r'\t*\(fp_line\b[^\n]*"[BF]\.SilkS"[^\n]*\)\n', '', b)   # drop existing 1-line silk
    ins = len(b) - 1                                                     # before block's closing ')'
    while b[ins] != ')': ins -= 1
    b = b[:ins] + rect_lines(f['box'], f['silk'], f['p1']) + b[ins:]
    edits.append((f['fs'], f['fe'], b)); n_out += 1
for fs, fe, nb in sorted(edits, key=lambda e: -e[0]):
    p = p[:fs] + nb + p[fe:]

# --- pass 2: delete every board-level silk gr_line ---
spans = []
for m in re.finditer(r'\(gr_line\b', p):
    e = bend(p, m.start())
    if re.search(r'\(layer "[BF]\.SilkS"\)', p[m.start():e]): spans.append((m.start(), e))
removed = len(spans)
for s, e in sorted(spans, reverse=True):
    q = e
    while q < len(p) and p[q] in ' \t\n': q += 1     # eat trailing whitespace/newline
    p = p[:s] + p[q:]

open(PCB, "w", encoding="utf-8").write(p)
print("generated outlines for %d components; deleted %d board-level silk lines" % (n_out, removed))
