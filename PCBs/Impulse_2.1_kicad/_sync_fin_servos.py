#!/usr/bin/env python3
# Schematic side of the fin-servo change, kept in sync with the PCB:
#   - rename Y1 connector + SERVOZ labels -> SERVO1
#   - add SERVO2/3/4 connectors (self-contained: signal label + 5V-DIRTY label + GND symbol)
#   - add Teensy-side global labels on pins 28/29/25
#   - link the PCB SERVO1/2/3/4 footprints to their schematic symbols via (path ...)
import re, sys, uuid
SCH, PCB = "Impulse_2.1.kicad_sch", "Impulse_2.1.kicad_pcb"
SHEET = "61f4f9be-db17-4eed-b67c-dfd3ce4de527"
s = open(SCH, encoding="utf-8").read()

def bend(text, start):            # index just past the sexpr starting at 'start'
    d = 0
    for j in range(start, len(text)):
        if text[j] == '(': d += 1
        elif text[j] == ')':
            d -= 1
            if d == 0: return j + 1
    raise ValueError
def nid(): return str(uuid.uuid4())
def f(v):
    return ('%.4f' % v).rstrip('0').rstrip('.')
def translate(block, dx, dy):
    block = re.sub(r'\(at (-?\d+\.?\d*) (-?\d+\.?\d*)( \d+)?\)',
                   lambda m: '(at %s %s%s)' % (f(float(m.group(1))+dx), f(float(m.group(2))+dy), m.group(3) or ''),
                   block)
    block = re.sub(r'\(xy (-?\d+\.?\d*) (-?\d+\.?\d*)\)',
                   lambda m: '(xy %s %s)' % (f(float(m.group(1))+dx), f(float(m.group(2))+dy)),
                   block)
    return block
def fresh_uuids(block):
    ids = []
    def r(m):
        u = nid(); ids.append(u); return '(uuid "%s")' % u
    return re.sub(r'\(uuid "[0-9a-f\-]{36}"\)', r, block), ids[0]

# ---- templates pulled from the existing Y connector group -------------------
def grab(anchor, back='(symbol'):
    i = s.find(anchor); st = s.rfind(back, 0, i); return s[st:bend(s, st)]
CONN = grab('2435c17b-2b83-4977-a815-988c0e4a355e')                       # Y connector symbol
GND  = grab('(lib_id "ProDoc_P1_-easyedapro:Ground-GND")\n\t\t(at 414.02 234.95')
assert '"Y"' in CONN and CONN.count('(uuid') == 4

def glabel(name, x, y, angle, justify):
    return ('\t(global_label "%s"\n\t\t(shape input)\n\t\t(at %s %s %d)\n'
            '\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.5748 1.5748)\n\t\t\t)\n\t\t\t(justify %s)\n\t\t)\n'
            '\t\t(uuid "%s")\n'
            '\t\t(property "Intersheetrefs" "${INTERSHEET_REFS}"\n\t\t\t(at %s %s 0)\n\t\t\t(hide yes)\n'
            '\t\t\t(show_name no)\n\t\t\t(do_not_autoplace no)\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n'
            ) % (name, f(x), f(y), angle, justify, nid(), f(x), f(y))
def wire(x1, y1, x2, y2):
    return ('\t(wire\n\t\t(pts\n\t\t\t(xy %s %s) (xy %s %s)\n\t\t)\n'
            '\t\t(stroke\n\t\t\t(width 0)\n\t\t\t(type default)\n\t\t)\n\t\t(uuid "%s")\n\t)\n'
            ) % (f(x1), f(y1), f(x2), f(y2), nid())

# ---- 1. rename SERVOZ -> SERVO1 (both global labels) and the Y1 connector ----
assert s.count('(global_label "SERVOZ"') == 2
s = s.replace('(global_label "SERVOZ"', '(global_label "SERVO1"')
assert s.count('(property "Reference" "Y1"') == 1 and s.count('(reference "Y1")') == 1
s = s.replace('(property "Reference" "Y1"', '(property "Reference" "SERVO1"')
s = s.replace('(reference "Y1")', '(reference "SERVO1")')

# ---- 2. build SERVO2/3/4 connector groups in the empty right column ----------
COLX = 469.1                       # existing column is 419.1; +50mm
ROWS = {"SERVO2": 207.01, "SERVO3": 232.41, "SERVO4": 257.81}
conn_uuid = {}
new = []
for ref, yc in ROWS.items():
    dy = yc - 232.41               # Y template center
    c = translate(CONN, 50.0, dy)
    c, cu = fresh_uuids(c)
    c = c.replace('(property "Reference" "Y"', '(property "Reference" "%s"' % ref, 1)
    c = c.replace('(reference "Y")', '(reference "%s")' % ref, 1)
    conn_uuid[ref] = cu
    g = translate(GND, 50.0, dy); g, _ = fresh_uuids(g)
    px = COLX - 5.08               # pin x = 464.02
    new += [c, g,
            wire(px, yc - 2.54, 447.51, yc - 2.54), glabel(ref, 447.51, yc - 2.54, 180, "right"),
            wire(px, yc, 451.32, yc), glabel("5V-DIRTY", 451.32, yc, 180, "right")]

# ---- 3. Teensy-side labels on the free PWM pins ------------------------------
new += [glabel("SERVO2", 302.26, 269.24, 0, "left"),    # pin 28 (right edge)
        glabel("SERVO3", 302.26, 271.78, 0, "left"),    # pin 29 (right edge)
        glabel("SERVO4", 246.38, 266.70, 180, "right")] # pin 25 (left edge)

block = "".join(new)
pos = s.rfind('\t(embedded_fonts no)')   # LAST one = top-level (others live inside lib_symbols)
assert pos != -1 and s[pos:].strip() == '(embedded_fonts no)\n)', "unexpected file tail"
s = s[:pos] + block + s[pos:]
open(SCH, "w", encoding="utf-8").write(s)

# ---- 4. link PCB footprints to their schematic symbols ----------------------
p = open(PCB, encoding="utf-8").read()
link = {"SERVO1": "a4a68360-0e21-48d5-a956-7c4c6b304fdc", "SERVO2": conn_uuid["SERVO2"],
        "SERVO3": conn_uuid["SERVO3"], "SERVO4": conn_uuid["SERVO4"]}
for ref, su in link.items():
    fi = p.find('(footprint ')
    while fi != -1:
        blk_end = bend(p, fi)
        if '(property "Reference" "%s"' % ref in p[fi:blk_end]:
            atm = re.search(r'\n([ \t]*)\(at [\-\d. ]+\)\n', p[fi:blk_end])
            ins = fi + atm.end()
            assert '(path "' not in p[fi:ins], "%s already has a path" % ref
            p = p[:ins] + '%s(path "/%s/%s")\n' % (atm.group(1), SHEET, su) + p[ins:]
            break
        fi = p.find('(footprint ', blk_end)
    else:
        raise SystemExit("footprint %s not found" % ref)
open(PCB, "w", encoding="utf-8").write(p)

print("renamed Y1/SERVOZ -> SERVO1")
print("added SERVO2/3/4 connectors at x=%.1f rows %s" % (COLX, list(ROWS.values())))
print("connector uuids:", conn_uuid)
print("linked PCB footprints -> schematic:", {k: v[:8] for k, v in link.items()})
