#!/usr/bin/env python3
# Power-section redesign (schematic):
#  OUT: LM2594 circuit (U4,L4,C22,C38,R8,R9,D10), DROK module (U8), OR-diodes D8/D9, and the
#       jumpers LM2594_enable1 / logic_power_reg1 / servo_power_reg1 / DROK_exclusive1 / power_tap1
#  IN:  U2 = AP63205WU 5V/2A sync buck (logic -> 5V-CLEAN)
#       U3 = AP63305   5V/3A sync buck, same family/pinout (servo -> SERVO_EN1 jumper -> 5V-DIRTY)
#       C10 = 220uF bulk on the pyro 7.4V rail; PWR_FLAGs on the source rails
#  Kept: SW1 master switch, PYRO_ON1 arming jumper, C39 input bulk, C33/C43 rail caps.
import re, uuid, math, json

SCH = "Impulse_2.2.kicad_sch"
SYMD = "C:/Program Files/KiCad/10.0/share/kicad/symbols"
DEAD = ["U4", "L4", "C22", "C38", "R8", "R9", "D8", "D9", "D10", "U8",
        "LM2594_enable1", "logic_power_reg1", "servo_power_reg1", "DROK_exclusive1", "power_tap1"]

def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
    raise ValueError
def nid(): return str(uuid.uuid4())
def g(v): return ('%.2f' % v).rstrip('0').rstrip('.')
def rp(x, y): return (round(x, 2), round(y, 2))
def mat_for(ang, mirror):
    a = math.radians(ang)
    R = [[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]   # CCW on screen, y-down
    F = [[1, 0], [0, -1]]
    M = [[R[0][0]*F[0][0], R[0][1]*F[1][1]], [R[1][0]*F[0][0], R[1][1]*F[1][1]]]
    if mirror == 'y': M = [[-M[0][0], -M[0][1]], [M[1][0], M[1][1]]]
    if mirror == 'x': M = [[M[0][0], M[0][1]], [-M[1][0], -M[1][1]]]
    return M
def xf(M, px, py): return (M[0][0]*px + M[0][1]*py, M[1][0]*px + M[1][1]*py)

s = open(SCH, encoding="utf-8").read()
li = s.find('(lib_symbols'); le = bend(s, li)

# ---------- embedded lib pins ----------
def parse_pins(blk):
    out = []
    for pm in re.finditer(r'\(pin\s+\w+\s+\w+', blk):
        pb = blk[pm.start():bend(blk, pm.start())]
        at = re.search(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)(?:\s+([\-\d.]+))?\)', pb)
        num = re.search(r'\(number\s+"([^"]*)"', pb)
        out.append((num.group(1), float(at.group(1)), float(at.group(2)), float(at.group(3) or 0)))
    return out
libpins = {}
libs = s[li:le]
for m in re.finditer(r'\(symbol "([^"]+:[^"]+)"', libs):
    blk = libs[m.start():bend(libs, m.start())]
    ps = parse_pins(blk)
    if ps: libpins[m.group(1)] = ps

# ---------- instances ----------
inst = []
body_start = le
for m in re.finditer(r'\(lib_id "([^"]+)"\)', s[le:]):
    st = s.rfind('(symbol', 0, le + m.start()); en = bend(s, st)
    blk = s[st:en]
    rr = re.search(r'\(property "Reference" "([^"]+)"', blk)
    at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+) (\d+)\)', blk)
    mir = re.search(r'\(mirror (\w+)\)', blk)
    inst.append(dict(ref=rr.group(1) if rr else "?", lib=m.group(1), st=st, en=en,
                     x=float(at.group(1)), y=float(at.group(2)), ang=int(at.group(3)),
                     mirror=mir.group(1) if mir else None))
def pins_abs(it):
    M = mat_for(it['ang'], it['mirror'])
    out = {}
    for (num, px, py, pa) in libpins.get(it['lib'], []):
        dx, dy = xf(M, px, py)
        out[num] = rp(it['x'] + dx, it['y'] + dy)
    return out
dead_pins = set()
for it in inst:
    if it['ref'] in DEAD: dead_pins |= set(pins_abs(it).values())

# ---------- wires / junctions / labels / gnd symbols ----------
def blocks(tok):
    return [(le + m.start(), bend(s, le + m.start())) for m in re.finditer(r'\(%s\b' % tok, s[le:])]
wires = []
for st, en in blocks("wire"):
    xy = re.findall(r'\(xy ([\-\d.]+) ([\-\d.]+)\)', s[st:en])
    wires.append(dict(st=st, en=en, a=rp(float(xy[0][0]), float(xy[0][1])),
                      b=rp(float(xy[1][0]), float(xy[1][1]))))
juncs = []
for st, en in blocks("junction"):
    at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)\)', s[st:en])
    juncs.append(dict(st=st, en=en, p=rp(float(at.group(1)), float(at.group(2)))))
glabels = []
for st, en in blocks("global_label"):
    at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', s[st:en])
    nm = re.match(r'\(global_label "([^"]*)"', s[st:en])
    glabels.append(dict(st=st, en=en, p=rp(float(at.group(1)), float(at.group(2))), name=nm.group(1)))
nocons = []
for st, en in blocks("no_connect"):
    at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)\)', s[st:en])
    nocons.append(dict(st=st, en=en, p=rp(float(at.group(1)), float(at.group(2)))))

# union-find over wires (endpoints + junction points + T-joints)
parent = {}
def find(x):
    parent.setdefault(x, x)
    while parent[x] != x:
        parent[x] = parent[parent[x]]; x = parent[x]
    return x
def union(a, b): parent[find(a)] = find(b)
def on_seg(q, a, b, tol=0.01):
    if not (min(a[0], b[0]) - tol <= q[0] <= max(a[0], b[0]) + tol and
            min(a[1], b[1]) - tol <= q[1] <= max(a[1], b[1]) + tol): return False
    return abs((b[0]-a[0])*(q[1]-a[1]) - (b[1]-a[1])*(q[0]-a[0])) <= tol*(abs(b[0]-a[0])+abs(b[1]-a[1])+tol)
for w in wires: union(w['a'], w['b'])
pts = set()
for w in wires: pts |= {w['a'], w['b']}
for j in juncs: pts.add(j['p'])
for q in pts:
    for w in wires:
        if q not in (w['a'], w['b']) and on_seg(q, w['a'], w['b']): union(q, w['a'])
dead_roots = set(find(p) for p in dead_pins if p in parent)

# ---------- deletions ----------
spans = []
for it in inst:
    if it['ref'] in DEAD: spans.append((it['st'], it['en']))
kill_wire_pts = set()
for w in wires:
    if find(w['a']) in dead_roots:
        spans.append((w['st'], w['en'])); kill_wire_pts |= {w['a'], w['b']}
for j in juncs:
    if find(j['p']) in dead_roots: spans.append((j['st'], j['en']))
for gl in glabels:
    if find(gl['p']) in dead_roots or gl['p'] in dead_pins: spans.append((gl['st'], gl['en']))
for nc in nocons:
    if nc['p'] in dead_pins: spans.append((nc['st'], nc['en']))
# GND power symbols sitting on dead wiring or dead pins
gnd_del = 0
for it in inst:
    if it['lib'] == 'power:GND' and (find(rp(it['x'], it['y'])) in dead_roots or rp(it['x'], it['y']) in dead_pins):
        spans.append((it['st'], it['en'])); gnd_del += 1
for st, en in sorted(set(spans), reverse=True):
    q = en
    while q < len(s) and s[q] in ' \t\n': q += 1
    s = s[:st] + s[q:]
print("deleted: %d symbols, %d GND syms, %d wires+junctions+labels total spans"
      % (len(DEAD), gnd_del, len(set(spans))))

# ---------- embed AP63205WU (flattened) + ensure PWR_FLAG ----------
def raw_symbol(libfile, name):
    t = open("%s/%s.kicad_sym" % (SYMD, libfile), encoding="utf-8").read()
    m = re.search(r'\(symbol "%s"' % re.escape(name), t)
    return t[m.start():bend(t, m.start())]
li = s.find('(lib_symbols'); le = bend(s, li)
add_defs = ""
if 'Regulator_Switching:AP63205WU' not in s:
    blk = raw_symbol("Regulator_Switching", "AP63205WU")
    ext = re.search(r'\(extends "([^"]+)"\)', blk)
    if ext:
        blk = raw_symbol("Regulator_Switching", ext.group(1))
        blk = blk.replace('(symbol "%s_' % ext.group(1), '(symbol "AP63205WU_')
        blk = blk.replace('(symbol "%s"' % ext.group(1), '(symbol "AP63205WU"', 1)
    libpins['Regulator_Switching:AP63205WU'] = parse_pins(blk)
    blk = blk.replace('(symbol "AP63205WU"', '(symbol "Regulator_Switching:AP63205WU"', 1)
    add_defs += "    " + blk.strip() + "\n"
if 'power:PWR_FLAG' not in s:
    blk = raw_symbol("power", "PWR_FLAG")
    libpins['power:PWR_FLAG'] = parse_pins(blk)
    blk = blk.replace('(symbol "PWR_FLAG"', '(symbol "power:PWR_FLAG"', 1)
    add_defs += "    " + blk.strip() + "\n"
if add_defs:
    s = s[:le-1] + add_defs + s[le-1:]

# ---------- emit helpers ----------
NEW = ""
uu_of = {}
def wire(a, b):
    global NEW
    NEW += ('  (wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) (uuid %s))\n'
            % (g(a[0]), g(a[1]), g(b[0]), g(b[1]), nid()))
def junc(p):
    global NEW
    NEW += '  (junction (at %s %s) (diameter 0) (color 0 0 0 0) (uuid %s))\n' % (g(p[0]), g(p[1]), nid())
def label(name, p, ang, just):
    global NEW
    NEW += ('  (global_label "%s" (shape input) (at %s %s %d) '
            '(effects (font (size 1.27 1.27)) (justify %s))\n    (uuid %s)\n'
            '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) (hide yes) '
            '(effects (font (size 1.27 1.27))))\n  )\n'
            % (name, g(p[0]), g(p[1]), ang, just, nid(), g(p[0]), g(p[1])))
def sym(lib, ref, val, fp, x, y, ang=0, hideval=False):
    global NEW
    u = nid(); uu_of[ref] = u
    pinsrc = libpins[lib]
    NEW += ('  (symbol (lib_id "%s") (at %s %s %d) (unit 1) (exclude_from_sim no) (in_bom %s) '
            '(on_board yes) (dnp no)\n    (uuid %s)\n'
            '    (property "Reference" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) %s))\n'
            '    (property "Value" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) %s))\n'
            '    (property "Footprint" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
            % (lib, g(x), g(y), ang, "no" if ref.startswith('#') else "yes", u,
               ref, g(x+2), g(y-4), "(hide yes)" if ref.startswith('#') else "(justify left bottom)",
               val, g(x+2), g(y+4), "(hide yes)" if hideval else "(justify left top)",
               fp, g(x), g(y)))
    for (num, _px, _py, _pa) in pinsrc:
        NEW += '    (pin "%s" (uuid %s))\n' % (num, nid())
    NEW += ('    (instances (project "Impulse_2.2" (path "/%s" (reference "%s") (unit 1))))\n  )\n'
            % ("11111111-2222-3333-4444-000000000001", ref))
    M = mat_for(ang, None)
    return {num: rp(x + xf(M, px, py)[0], y + xf(M, px, py)[1]) for (num, px, py, _pa) in pinsrc}
def gnd(p):
    global NEW
    u = nid()
    NEW += ('  (symbol (lib_id "power:GND") (at %s %s 0) (unit 1) (exclude_from_sim no) (in_bom no) '
            '(on_board yes) (dnp no)\n    (uuid %s)\n'
            '    (property "Reference" "#PWR?" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
            '    (property "Value" "GND" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
            '    (pin "1" (uuid %s))\n'
            '    (instances (project "Impulse_2.2" (path "/%s" (reference "#PWR?") (unit 1))))\n  )\n'
            % (g(p[0]), g(p[1]), u, g(p[0]), g(p[1]-3), g(p[0]), g(p[1]+3), nid(),
               "11111111-2222-3333-4444-000000000001"))
def flag(net, p):
    global NEW
    u = nid()
    NEW += ('  (symbol (lib_id "power:PWR_FLAG") (at %s %s 0) (unit 1) (exclude_from_sim no) '
            '(in_bom no) (on_board yes) (dnp no)\n    (uuid %s)\n'
            '    (property "Reference" "#FLG?" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
            '    (property "Value" "PWR_FLAG" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
            '    (pin "1" (uuid %s))\n'
            '    (instances (project "Impulse_2.2" (path "/%s" (reference "#FLG?") (unit 1))))\n  )\n'
            % (g(p[0]), g(p[1]), u, g(p[0]), g(p[1]-4), g(p[0]), g(p[1]-6), nid(),
               "11111111-2222-3333-4444-000000000001"))

APS = 'Regulator_Switching:AP63205WU'
CAP = 'Device:C'; LSY = 'Device:L'; CPOL = 'Device:C_Polarized'; HDR = 'Connector_Generic:Conn_01x02'
FPC1206 = 'Capacitor_SMD:C_1206_3216Metric'; FPC0603 = 'Capacitor_SMD:C_0603_1608Metric'
FPL = 'Inductor_SMD:L_Bourns_SRN6045TA'; FPTSOT = 'Package_TO_SOT_SMD:TSOT-23-6'

def buck(uref, uval, lref, lval, cinr, bstr, co1, co2, X, Y):
    up = sym(APS, uref, uval, FPTSOT, X, Y)
    vinl = (X - 25, up['3'][1])
    wire(up['3'], vinl)                                   # IN left
    wire(up['2'], (vinl[0] + 5, up['2'][1])); wire((vinl[0] + 5, up['2'][1]), (vinl[0] + 5, vinl[1]))
    junc((vinl[0] + 5, vinl[1]))                          # EN tied to VIN
    cin = sym(CAP, cinr, "10uF 25V", FPC1206, X - 20, Y + 1.27)
    wire(cin['1'], (cin['1'][0], vinl[1])); junc((cin['1'][0], vinl[1]))
    gnd(cin['2'])
    label("VBAT", vinl, 180, "right")
    # SW -> L -> OUT
    lp = sym(LSY, lref, lval, FPL, X + 35, up['5'][1], 90)
    wire(up['5'], lp['1'])
    out_end = (X + 55, up['5'][1])
    wire(lp['2'], out_end)
    # BST cap above the SW line
    cb = sym(CAP, bstr, "100nF", FPC0603, X + 13, up['5'][1] - 3.81)
    wire(cb['2'], (cb['2'][0], up['5'][1])); junc((cb['2'][0], up['5'][1]))
    wire(up['6'], (X + 28, up['6'][1])); wire((X + 28, up['6'][1]), (X + 28, cb['1'][1]))
    wire((X + 28, cb['1'][1]), cb['1'])
    # FB sense to OUT
    wire(up['1'], (X + 41, up['1'][1])); wire((X + 41, up['1'][1]), (X + 41, up['5'][1]))
    junc((X + 41, up['5'][1]))
    # output caps
    c1 = sym(CAP, co1, "22uF 16V", FPC1206, X + 45, Y + 1.27)
    c2 = sym(CAP, co2, "22uF 16V", FPC1206, X + 51, Y + 1.27)
    for c in (c1, c2):
        wire(c['1'], (c['1'][0], up['5'][1])); junc((c['1'][0], up['5'][1])); gnd(c['2'])
    gnd(up['4'])
    return out_end

# logic buck
o1 = buck("U2", "AP63205WU-7", "L1", "6.8uH", "C1", "C3", "C5", "C6", 150, 85)
label("5V-CLEAN", o1, 0, "left"); flag("5V-CLEAN", o1)
# servo buck + SERVO_EN jumper
o2 = buck("U3", "AP63305WU-7", "L2", "4.7uH", "C2", "C4", "C7", "C8", 150, 152)
sj = sym(HDR, "SERVO_EN1", "servo disable", 'Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical',
         o2[0] + 7, o2[1])
wire(o2, sj['1'])
wire(sj['2'], (sj['2'][0] - 4, sj['2'][1]))
label("5V-DIRTY", (sj['2'][0] - 4, sj['2'][1]), 180, "right"); flag("5V-DIRTY", (sj['2'][0] - 4, sj['2'][1]))
# C39 input bulk rewire (kept at its position)
c39 = [it for it in inst if it['ref'] == 'C39'][0]
c39p = pins_abs(c39)
wire(c39p['1'], (c39p['1'][0], c39p['1'][1] - 3))
label("VBAT", (c39p['1'][0], c39p['1'][1] - 3), 90, "left")
# ensure C39 pin2 GND (its old symbol may have died with the wiring)
gnd(c39p['2'])
# SW1 output stub -> VBAT
sw1 = [it for it in inst if it['ref'] == 'SW1'][0]
swp = pins_abs(sw1)
stub = (swp['2'][0] + 7, swp['2'][1])
wire(swp['2'], stub)
label("VBAT", stub, 0, "left"); flag("VBAT", stub)
# pyro bulk cap where power_tap1 was + 7.4V flag + GND flag
c10 = sym(CPOL, "C10", "220uF 25V", 'Capacitor_SMD:CP_Elec_8x10', 303.53, 135.89)
wire(c10['1'], (c10['1'][0], c10['1'][1] - 3))
label("7.4V", (c10['1'][0], c10['1'][1] - 3), 90, "left")
flag("7.4V", (c10['1'][0], c10['1'][1] - 3))
gnd(c10['2']); flag("GND", c10['2'])

idx = s.rfind('(sheet_instances')
s = s[:idx] + NEW + s[idx:]
open(SCH, "w", encoding="utf-8").write(s)
json.dump(uu_of, open("_new_uuids.json", "w"), indent=0)
print("added: U2/U3 bucks + L1/L2 + C1-C8, SERVO_EN1, C10 pyro bulk, VBAT labels, 4 PWR_FLAGs + GND flag")
