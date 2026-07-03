#!/usr/bin/env python3
# The EasyEDA import left every component's silkscreen OUTLINE as board-level gr_line graphics, so
# moving a footprint doesn't move its silk. This moves each silk line INTO the footprint it belongs
# to (as fp_line, in the footprint's local frame) so parts + silk move together.
# Assignment = footprint with the nearest pad to the line (holes excluded); lines >6mm from every
# part stay board-level (true board decoration).
import re, math, uuid
PCB = "Impulse_2.1.kicad_pcb"
THR = 6.0
p = open(PCB, encoding="utf-8").read()
def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
def nid(): return str(uuid.uuid4())

# --- parse footprints: ref, origin, angle(rad), abs pad positions, block span ---
FP = []
for m in re.finditer(r'\(footprint ', p):
    fs = m.start(); fe = bend(p, fs); b = p[fs:fe]
    rr = re.search(r'\(property "Reference" "([^"]+)"', b)
    at = re.search(r'\n\s*\(at ([\-\d.]+) ([\-\d.]+)(?: ([\-\d.]+))?\)', b)
    if not rr or not at: continue
    ox, oy = float(at.group(1)), float(at.group(2)); ang = float(at.group(3) or 0)
    a = math.radians(ang); pads = []
    for pm in re.finditer(r'\(pad ', b):
        pb = b[pm.start():bend(b, pm.start())]
        pat = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', pb)
        if pat:
            px, py = float(pat.group(1)), float(pat.group(2))
            pads.append((ox + px*math.cos(a) - py*math.sin(a), oy + px*math.sin(a) + py*math.cos(a)))
    FP.append(dict(ref=rr.group(1), ox=ox, oy=oy, ang=ang, pads=pads, fs=fs, fe=fe))
byref = {f['ref']: f for f in FP}

def nearest(pts):
    best = None; bd = THR
    for f in FP:
        if f['ref'].startswith('H') or not f['pads']: continue
        for (px, py) in f['pads']:
            for (qx, qy) in pts:
                dd = math.hypot(qx-px, qy-py)
                if dd < bd: bd = dd; best = f['ref']
    return best

def to_local(f, x, y):                       # abs -> footprint local (unrotated) frame
    dx, dy = x - f['ox'], y - f['oy']; t = math.radians(-f['ang'])
    return (dx*math.cos(t) - dy*math.sin(t), dx*math.sin(t) + dy*math.cos(t))
def g(v): return ('%.4f' % v).rstrip('0').rstrip('.')

# --- classify each board-level silk gr_line ---
inserts = {}          # ref -> [fp_line text]
deletions = []        # (start, end) spans to remove
moved = 0
for m in re.finditer(r'\(gr_line\b', p):
    s = m.start(); e = bend(p, s); blk = p[s:e]
    lm = re.search(r'\(layer "([BF]\.SilkS)"', blk)
    if not lm: continue                       # only silk
    coord = re.search(r'\(start ([\-\d.]+) ([\-\d.]+)\)\s*\(end ([\-\d.]+) ([\-\d.]+)\)', blk)
    x1, y1, x2, y2 = map(float, coord.groups())
    w = re.search(r'\(width ([\-\d.]+)\)', blk)
    width = w.group(1) if w else "0.12"
    ref = nearest([(x1, y1), (x2, y2), ((x1+x2)/2, (y1+y2)/2)])
    if not ref: continue                      # orphan -> keep board-level
    f = byref[ref]
    lx1, ly1 = to_local(f, x1, y1); lx2, ly2 = to_local(f, x2, y2)
    fpl = ('    (fp_line (start %s %s) (end %s %s) (stroke (width %s) (type solid)) '
           '(layer "%s") (uuid "%s"))\n') % (g(lx1), g(ly1), g(lx2), g(ly2), width, lm.group(1), nid())
    inserts.setdefault(ref, []).append(fpl)
    deletions.append((s, e))
    moved += 1

# --- build edit list (insertions at each footprint's closing paren + deletions), apply high->low ---
edits = []
for ref, lines in inserts.items():
    f = byref[ref]
    edits.append((f['fe'] - 1, 'ins', "".join(lines)))     # before footprint's final ')'
for (s, e) in deletions:
    edits.append((s, 'del', e))
edits.sort(key=lambda x: x[0], reverse=True)
# guard: no insertion position lies inside a deletion span
for pos, kind, arg in edits:
    if kind == 'ins':
        assert not any(s <= pos < e for (s, e) in deletions), "insert inside deletion!"
for pos, kind, arg in edits:
    if kind == 'ins':
        p = p[:pos] + arg + p[pos:]
    else:
        p = p[:pos] + p[arg:]

open(PCB, "w", encoding="utf-8").write(p)
print("moved %d silk lines into footprints; %d footprints gained silk" % (moved, len(inserts)))
top = sorted(inserts.items(), key=lambda kv: -len(kv[1]))[:8]
print("top:", [(k, len(v)) for k, v in top])
print("U1 silk lines attached:", len(inserts.get('U1', [])))
