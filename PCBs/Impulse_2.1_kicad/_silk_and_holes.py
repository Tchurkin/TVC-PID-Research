#!/usr/bin/env python3
# 1) add F.SilkS body outlines (chamfered pin-1 rectangle) to SERVO2/3/4 (they had none)
# 2) add 4x M3 (3.2mm NPTH) mounting holes on a 60mm bolt circle at the disc's diagonal corners
import re, uuid, math
PCB = "Impulse_2.1.kicad_pcb"
p = open(PCB, encoding="utf-8").read()
def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
def nid(): return str(uuid.uuid4())
def line(x1, y1, x2, y2):
    return ('    (fp_line (start %g %g) (end %g %g) (stroke (width 0.12) (type solid)) '
            '(layer "F.SilkS") (uuid "%s"))\n') % (x1, y1, x2, y2, nid())

# --- silk outline relative to a vertical 3-pin header origin (pads at y=-2.54/0/+2.54) ---
# 2.5 x 7.62 body rectangle, top-left corner chamfered = pin-1 marker (pin1 is the signal pad @ -y)
def silk():
    return (line(-0.65, -3.81, 1.25, -3.81) +     # top (chamfer start)
            line(1.25, -3.81, 1.25, 3.81) +        # right
            line(1.25, 3.81, -1.25, 3.81) +        # bottom
            line(-1.25, 3.81, -1.25, -3.21) +      # left (chamfer start)
            line(-1.25, -3.21, -0.65, -3.81))      # pin-1 chamfer

added = []
for ref in ["SERVO2", "SERVO3", "SERVO4"]:
    m = re.search(r'\(property "Reference" "%s"' % ref, p)
    fs = p.rfind('(footprint ', 0, m.start()); fe = bend(p, fs)
    assert '(fp_line' not in p[fs:fe], "%s already has silk" % ref
    ins = fe - 1                                    # before footprint's closing ')'
    p = p[:ins] + silk() + p[ins:]
    added.append(ref)

# --- 4x M3 mounting holes ---
cx, cy, R = 51.3446, 51.5644, 35.3888
BR = 30.0                                           # bolt-circle radius (60mm dia)
def hole(ref, x, y):
    return ('  (footprint "MountingHole:MountingHole_3.2mm_M3"\n'
            '    (layer "F.Cu")\n    (uuid "%s")\n    (at %.4f %.4f)\n'
            '    (property "Reference" "%s" (at 0 -3.7 0) (layer "F.SilkS") (uuid "%s")'
            ' (effects (font (size 1 1) (thickness 0.15))))\n'
            '    (property "Value" "MountingHole_3.2mm_M3" (at 0 3.7 0) (layer "F.Fab") (hide yes)'
            ' (uuid "%s") (effects (font (size 1 1) (thickness 0.15))))\n'
            '    (attr exclude_from_pos_files exclude_from_bom allow_missing_courtyard)\n'
            '    (fp_circle (center 0 0) (end 2.5 0) (stroke (width 0.15) (type solid)) (fill no)'
            ' (layer "F.SilkS") (uuid "%s"))\n'
            '    (pad "" np_thru_hole circle (at 0 0) (size 3.2 3.2) (drill 3.2)'
            ' (layers "*.Cu" "*.Mask") (uuid "%s"))\n  )\n'
            ) % (nid(), x, y, ref, nid(), nid(), nid(), nid())
holes = ""
pos = {}
for i, ang in enumerate([45, 135, 225, 315], 1):
    x = cx + BR * math.cos(math.radians(ang)); y = cy + BR * math.sin(math.radians(ang))
    pos["H%d" % i] = (round(x, 2), round(y, 2))
    holes += hole("H%d" % i, x, y)
idx = p.rstrip().rfind(")")
p = p[:idx] + holes + p[idx:]

open(PCB, "w", encoding="utf-8").write(p)
print("added silk outlines to:", added)
print("added M3 mounting holes (bolt-circle r=%.0fmm):" % BR, pos)
