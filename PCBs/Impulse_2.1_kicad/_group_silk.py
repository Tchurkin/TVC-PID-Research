#!/usr/bin/env python3
# Group each component footprint with its surrounding board-level silk lines into a KiCad group,
# so they move together. Non-destructive: silk lines are untouched, just bundled by membership.
# Assignment = component with the nearest pad (holes/SERVO2-4/J_SENSOR skipped: they carry their
# own footprint silk). Silk >10mm from every part is left ungrouped (stranded outlines).
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

# --- footprints: ref, uuid, abs pad positions ---
FP = []
for m in re.finditer(r'\(footprint ', p):
    b = p[m.start():bend(p, m.start())]
    rr = re.search(r'\(property "Reference" "([^"]+)"', b)
    fu = re.search(r'\(uuid "([0-9a-f\-]{36})"\)', b)          # first uuid = footprint uuid
    at = re.search(r'\n\s*\(at ([\-\d.]+) ([\-\d.]+)(?: ([\-\d.]+))?\)', b)
    if not rr or not fu or not at: continue
    ox, oy = float(at.group(1)), float(at.group(2)); a = math.radians(float(at.group(3) or 0))
    pads = []
    for pm in re.finditer(r'\(pad ', b):
        pb = b[pm.start():bend(b, pm.start())]
        pat = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', pb)
        if pat:
            px, py = float(pat.group(1)), float(pat.group(2))
            pads.append((ox + px*math.cos(a) - py*math.sin(a), oy + px*math.sin(a) + py*math.cos(a)))
    FP.append(dict(ref=rr.group(1), uuid=fu.group(1), pads=pads))
SKIP = lambda r: r.startswith('H') or r in ('SERVO2', 'SERVO3', 'SERVO4', 'J_SENSOR')

def nearest(x1, y1, x2, y2):
    best = None; bd = THR
    for f in FP:
        if SKIP(f['ref']): continue
        for (px, py) in f['pads']:
            for (qx, qy) in ((x1, y1), (x2, y2), ((x1+x2)/2, (y1+y2)/2)):
                dd = math.hypot(qx-px, qy-py)
                if dd < bd: bd = dd; best = f['ref']
    return best

# --- assign each silk gr_line uuid to a component ---
members = {}   # ref -> [silk uuids]
orphan = 0
for m in re.finditer(r'\(gr_line\b', p):
    e = bend(p, m.start()); blk = p[m.start():e]
    if not re.search(r'\(layer "[BF]\.SilkS"\)', blk): continue
    co = re.search(r'\(start ([\-\d.]+) ([\-\d.]+)\)\s*\(end ([\-\d.]+) ([\-\d.]+)\)', blk)
    lu = re.search(r'\(uuid "([0-9a-f\-]{36})"\)', blk)
    ref = nearest(*map(float, co.groups()))
    if ref: members.setdefault(ref, []).append(lu.group(1))
    else: orphan += 1

# --- build (group ...) blocks: footprint uuid + its silk line uuids ---
byref = {f['ref']: f for f in FP}
blocks = ""
grouped = 0
for ref, uuids in sorted(members.items()):
    ids = [byref[ref]['uuid']] + uuids
    body = "".join('\t\t\t"%s"\n' % u for u in ids)
    blocks += ('\t(group "%s"\n\t\t(uuid "%s")\n\t\t(members\n%s\t\t)\n\t)\n'
               % (ref, nid(), body))
    grouped += 1
idx = p.rstrip().rfind(")")
p = p[:idx] + blocks + p[idx:]
open(PCB, "w", encoding="utf-8").write(p)
print("created %d component groups; %d silk lines left ungrouped (stranded >%.0fmm)" % (grouped, orphan, THR))
print("components with NO silk to group:", sorted(f['ref'] for f in FP if not SKIP(f['ref']) and f['ref'] not in members))
PY