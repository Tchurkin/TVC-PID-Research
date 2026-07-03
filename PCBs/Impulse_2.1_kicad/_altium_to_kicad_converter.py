#!/usr/bin/env python3
# Altium ASCII (Protel v5.01) PCB -> KiCad .kicad_pcb (unrouted: placement + nets + pads + silk + outline)
import re, uuid, sys, math
SRC = sys.argv[1]
OUT = sys.argv[2]
MIL = 0.0254  # mil -> mm

def parse(path):
    data = open(path, encoding="utf-8", errors="replace").read()
    recs = []
    for p in data.split("|RECORD=")[1:]:
        d = {}
        for f in ("RECORD=" + p).rstrip("\r\n").split("|"):
            if "=" in f:
                k, v = f.split("=", 1); d[k] = v
        recs.append(d)
    return recs

def mil(v):
    if v is None: return None
    try: return float(str(v).replace("mil", "").strip())
    except: return None

recs = parse(SRC)
from collections import defaultdict
bt = defaultdict(list)
for r in recs: bt[r.get("RECORD")].append(r)

# --- nets: id -> name ---
nets = {}
for r in bt["Net"]:
    if "ID" in r: nets[int(r["ID"])] = r.get("NAME", "")
maxid = max(nets) if nets else -1

# --- coordinate transform (Y flip), computed over components+pads+board verts ---
xs, ys = [], []
for r in bt["Component"] + bt["Pad"]:
    if mil(r.get("X")) is not None: xs.append(mil(r["X"])); ys.append(mil(r["Y"]))
# board outline verts
board = bt["Board"][0] if bt["Board"] else {}
nv = int(board.get("MAINCONTOURVERTEXCOUNT", "0") or 0)
verts = []
for i in range(nv):
    vx, vy = mil(board.get(f"VX{i}")), mil(board.get(f"VY{i}"))
    if vx is not None: verts.append((vx, vy)); xs.append(vx); ys.append(vy)
X0 = min(xs); Y1 = max(ys)
MARGIN = 10.0  # mm
def T(ax, ay):
    return ((ax - X0) * MIL + MARGIN, (Y1 - ay) * MIL + MARGIN)  # Y flip

def uid(): return str(uuid.uuid4())

# --- build footprints from components + their pads ---
pads_by_comp = defaultdict(list)
for p in bt["Pad"]:
    pads_by_comp[p.get("COMPONENT")].append(p)

def pad_layers(layer, hole):
    if layer == "TOP": return '"F.Cu" "F.Paste" "F.Mask"'
    if layer == "BOTTOM": return '"B.Cu" "B.Paste" "B.Mask"'
    return '"*.Cu" "*.Mask"'  # MULTILAYER

def pad_shape(shape, sx, sy):
    s = (shape or "").upper()
    if s == "RECTANGLE": return "rect"
    if s == "ROUND": return "circle" if abs(sx - sy) < 1e-6 else "oval"
    if s == "OCTAGONAL": return "roundrect"
    return "rect"

fp_blocks = []
net_pad_count = defaultdict(int)
for comp in bt["Component"]:
    cid = comp.get("ID")
    cx, cy = mil(comp.get("X")), mil(comp.get("Y"))
    if cx is None: continue
    fx, fy = T(cx, cy)
    desig = comp.get("SOURCEDESIGNATOR", "REF")
    patt = comp.get("PATTERN", "FP").replace('"', "")
    lay = "B.Cu" if comp.get("LAYER") == "BOTTOM" else "F.Cu"
    lines = []
    lines.append(f'  (footprint "converted:{patt}"')
    lines.append(f'    (layer "{lay}")')
    lines.append(f'    (uuid "{uid()}")')
    lines.append(f'    (at {fx:.4f} {fy:.4f} 0)')
    lines.append(f'    (property "Reference" "{desig}" (at 0 0 0) (layer "F.SilkS") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))')
    lines.append(f'    (property "Value" "{patt}" (at 0 1 0) (layer "F.Fab") (hide yes) (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))')
    for p in pads_by_comp.get(cid, []):
        px, py = mil(p.get("X")), mil(p.get("Y"))
        if px is None: continue
        gx, gy = T(px, py)
        dx, dy = gx - fx, gy - fy
        sx = (mil(p.get("XSIZE")) or 10) * MIL
        sy = (mil(p.get("YSIZE")) or 10) * MIL
        hole = mil(p.get("HOLESIZE")) or 0
        ptype = "thru_hole" if (p.get("LAYER") == "MULTILAYER" or hole > 0) else "smd"
        shp = pad_shape(p.get("SHAPE"), sx, sy)
        name = p.get("NAME", "1").replace('"', "")
        prot = mil(p.get("ROTATION")) or 0.0
        rot = (-prot) % 360
        netclause = ""
        if "NET" in p:
            nid = int(p["NET"])
            if nid in nets:
                kn = nid + 1
                netclause = f' (net {kn} "{nets[nid]}")'
                net_pad_count[nets[nid]] += 1
        rr = f" {rot:.0f}" if rot else ""
        drill = f" (drill {hole*MIL:.3f})" if (ptype == "thru_hole" and hole > 0) else ""
        rrx = ' (roundrect_rratio 0.25)' if shp == "roundrect" else ""
        lines.append(f'    (pad "{name}" {ptype} {shp} (at {dx:.4f} {dy:.4f}{rr}) (size {sx:.4f} {sy:.4f}){drill} (layers {pad_layers(p.get("LAYER"), hole)}){rrx} (uuid "{uid()}"){netclause})')
    lines.append('  )')
    fp_blocks.append("\n".join(lines))

# --- silk lines from overlay tracks ---
def gr_line(x1, y1, x2, y2, layer, w):
    a = T(x1, y1); b = T(x2, y2)
    return f'  (gr_line (start {a[0]:.4f} {a[1]:.4f}) (end {b[0]:.4f} {b[1]:.4f}) (stroke (width {max(w,0.05):.3f}) (type solid)) (layer "{layer}") (uuid "{uid()}"))'

silk = []
for t in bt["Track"]:
    L = t.get("LAYER")
    lay = {"TOPOVERLAY": "F.SilkS", "BOTTOMOVERLAY": "B.SilkS"}.get(L)
    if not lay: continue
    x1, y1, x2, y2 = mil(t.get("X1")), mil(t.get("Y1")), mil(t.get("X2")), mil(t.get("Y2"))
    if None in (x1, y1, x2, y2): continue
    silk.append(gr_line(x1, y1, x2, y2, lay, (mil(t.get("WIDTH")) or 5) * MIL))

# --- board outline on Edge.Cuts: real shape is on KEEPOUT (lines + arcs), NOT the Board canvas box ---
def gr_arc(cx, cy, R, sa, ea, layer, w):
    ma = (sa + ea) / 2.0
    def pt(ang):
        r = math.radians(ang); return (cx + R*math.cos(r), cy + R*math.sin(r))
    s = T(*pt(sa)); m = T(*pt(ma)); e = T(*pt(ea))
    return (f'  (gr_arc (start {s[0]:.4f} {s[1]:.4f}) (mid {m[0]:.4f} {m[1]:.4f}) '
            f'(end {e[0]:.4f} {e[1]:.4f}) (stroke (width {max(w,0.05):.3f}) (type solid)) '
            f'(layer "{layer}") (uuid "{uid()}"))')

edge = []
for t in bt["Track"]:
    if t.get("LAYER") == "KEEPOUT":
        x1, y1, x2, y2 = mil(t.get("X1")), mil(t.get("Y1")), mil(t.get("X2")), mil(t.get("Y2"))
        if None not in (x1, y1, x2, y2):
            edge.append(gr_line(x1, y1, x2, y2, "Edge.Cuts", 0.1))
for a in bt["Arc"]:
    if a.get("LAYER") == "KEEPOUT":
        cx, cy, R = mil(a.get("LOCATION.X")), mil(a.get("LOCATION.Y")), mil(a.get("RADIUS"))
        sa, ea = mil(a.get("STARTANGLE")), mil(a.get("ENDANGLE"))
        if None not in (cx, cy, R, sa, ea):
            edge.append(gr_arc(cx, cy, R, sa, ea, "Edge.Cuts", 0.1))

# --- net declarations ---
net_decls = ['  (net 0 "")']
for nid in range(0, maxid + 1):
    if nid in nets:
        net_decls.append(f'  (net {nid+1} "{nets[nid]}")')

LAYERS = '''  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (32 "B.Adhes" user "B.Adhesive")
    (33 "F.Adhes" user "F.Adhesive")
    (34 "B.Paste" user)
    (35 "F.Paste" user)
    (36 "B.SilkS" user "B.Silkscreen")
    (37 "F.SilkS" user "F.Silkscreen")
    (38 "B.Mask" user)
    (39 "F.Mask" user)
    (40 "Dwgs.User" user "User.Drawings")
    (41 "Cmts.User" user "User.Comments")
    (44 "Edge.Cuts" user)
    (45 "Margin" user)
    (46 "B.CrtYd" user "B.Courtyard")
    (47 "F.CrtYd" user "F.Courtyard")
    (48 "B.Fab" user)
    (49 "F.Fab" user)
  )'''

with open(OUT, "w", encoding="utf-8") as f:
    f.write('(kicad_pcb\n')
    f.write('  (version 20240108)\n')
    f.write('  (generator "altium_ascii_convert")\n')
    f.write('  (generator_version "8.0")\n')
    f.write('  (general (thickness 1.6))\n')
    f.write('  (paper "A4")\n')
    f.write(LAYERS + "\n")
    f.write('  (setup (pad_to_mask_clearance 0))\n')
    f.write("\n".join(net_decls) + "\n")
    f.write("\n".join(fp_blocks) + "\n")
    f.write("\n".join(edge) + "\n")
    f.write("\n".join(silk) + "\n")
    f.write(')\n')

print(f"components(footprints): {len(fp_blocks)}")
print(f"nets declared: {len([n for n in range(maxid+1) if n in nets])} (+net0)")
print(f"silk lines: {len(silk)}   edge lines: {len(edge)}")
print("top nets by pad count:", sorted(net_pad_count.items(), key=lambda x:-x[1])[:6])
print(f"wrote {OUT}")
