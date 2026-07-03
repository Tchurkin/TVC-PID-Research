#!/usr/bin/env python3
# Impulse 2.2 schematic, layout-preserving + NET-AWARE:
# reproduce the user's Impulse 2.1 sheet (boxes, titles, wires, junctions, labels) with native
# symbols. Old wiring is kept verbatim; nets are propagated through it (union-find seeded from
# the old PCB netlist), each symbol's orientation matrix is chosen so pins land on the RIGHT
# nets, symbols are nudged off foreign wires, and stub wires bridge remaining pin offsets.
import re, json, uuid

OLD = "../Impulse_2.1_kicad/Impulse_2.1.kicad_sch"
INV = "../Impulse_2.1_kicad/_inventory.json"
PCB = "Impulse_2.2.kicad_pcb"
OUT = "Impulse_2.2.kicad_sch"
SYMD = "C:/Program Files/KiCad/10.0/share/kicad/symbols"
ROOT = "11111111-2222-3333-4444-000000000001"
sym_uuids = json.load(open("_sym_uuids.json"))

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

REF_RENAME = {"X": "X1", "Y": "Y1", "Out": "OUT1", "PYRO": "PYRO1", "PYRO_ON": "PYRO_ON1",
              "BUTTON": "BUTTON1", "BUZZER": "BUZZER1", "LED": "LED1",
              "MAIN": "BAT_MAIN1", "MAIN1": "AUX_3V3_1",
              "DROK_exclusive": "DROK_exclusive1", "logic_power_reg": "logic_power_reg1",
              "power_tap": "power_tap1", "servo_power_reg": "servo_power_reg1",
              "J_SENSOR": "J_SENSOR1", "P1O": "P1O1", "P2O": "P2O1", "P3O": "P3O1", "P4O": "P4O1"}
NET_RENAME = {"$1N50225": "VBAT_MAIN", "$1N50240": "VBAT_SW", "$1N49822": "SERVO_5V_IN",
              "$1N52582": "LOGIC_5V_IN", "$1N33870": "BUCK_SW", "$1N52591": "BUCK_5V",
              "$1N33864": "BUCK_FB", "$1N50177": "PYRO_ARM",
              "$1N14307": "RGB_B", "$1N14222": "RGB_G", "$1N14265": "RGB_R",
              "$1N42243": "PYRO1_OUT", "$1N42245": "PYRO1_LED",
              "$1N42393": "PYRO2_OUT", "$1N42395": "PYRO2_LED",
              "$1N56715": "PYRO3_OUT", "$1N42562": "PYRO3_LED",
              "$1N56609": "PYRO4_OUT", "$1N56605": "PYRO4_LED",
              "$1N14383": "BUTTON"}
def newnet(n): return NET_RENAME.get(n, n) if n else None

def padmap_for(oldref):
    if oldref == "U1": return "TEENSY"
    if oldref in ("P1", "P2", "P3", "P4"): return {"1": "2", "2": "1"}
    if oldref == "LED": return {"1": "1", "2": "2", "3": "4", "4": "3"}
    if oldref == "BUTTON": return {"1": "1", "2": None, "3": "2", "4": "2"}
    return None
def tmap_teensy(n):
    if n in ("VIN", "GND1"): return n
    if n == "GND3": return "GND2"
    if n == "GND6": return "GND3"
    if n == "3V3-1": return "3V3_1"
    if n == "3V3-3": return "3V3_2"
    m = re.match(r'(\d+)', n)
    return m.group(1) if m else None

# ---------------- new PCB parts + OLD netlist (by old ref/pad) ----------------
p = open(PCB, encoding="utf-8").read()
parts = {}
for m in re.finditer(r'\(footprint "([^"]+)"', p):
    b = p[m.start():bend(p, m.start())]
    ref = re.search(r'\(property "Reference" "([^"]+)"', b).group(1)
    val = re.search(r'\(property "Value" "([^"]+)"', b).group(1)
    pads = {}
    for pm in re.finditer(r'\(pad\s+"([^"]*)"', b):
        pb = b[pm.start():bend(b, pm.start())]
        nm = re.search(r'\(net "([^"]*)"\)', pb)
        pads.setdefault(pm.group(1), nm.group(1) if nm else None)
    parts[ref] = dict(fp=m.group(1), val=val, pads=pads)
inv = {f['ref']: f for f in json.load(open(INV))}
oldnet = {}   # (oldref, oldpad) -> NEW net name
for r, f in inv.items():
    for pd in f['pads']:
        if pd['net']: oldnet[(r, pd['n'])] = newnet(pd['net'])

# ---------------- symbol selection ----------------
def symbol_for(ref):
    fp = parts[ref]['fp']
    if ref == "U1": return ("CUSTOM", "Impulse22:TEENSY41")
    if ref == "U4": return ("Regulator_Switching", "LM2594M-ADJ")
    if ref.startswith("Q"): return ("Transistor_FET", "IRLML6244")
    if ref == "LED1": return ("Device", "LED_ABRG")
    if ref in ("P1", "P2", "P3", "P4"): return ("Device", "LED")
    if ref == "BUZZER1": return ("Device", "Buzzer")
    if ref == "BUTTON1": return ("Switch", "SW_Push")
    if ref == "SW1": return ("Switch", "SW_SPST")
    if ref in ("H1", "H2", "H3", "H4"): return ("Mechanical", "MountingHole")
    if ref in ("C38", "C39", "C43"): return ("Device", "C_Polarized")
    if ref in ("C22", "C33"): return ("Device", "C")
    if ref.startswith("R") and ref[1:].isdigit(): return ("Device", "R")
    if ref == "L4": return ("Device", "L")
    if ref in ("D8", "D9", "D10"): return ("Device", "D_Schottky")
    if "PinHeader_1x02" in fp: return ("Connector_Generic", "Conn_01x02")
    if "PinHeader_1x03" in fp: return ("Connector_Generic", "Conn_01x03")
    if "PinHeader_1x05" in fp: return ("Connector_Generic", "Conn_01x05")
    if "PinHeader_1x06" in fp: return ("Connector_Generic", "Conn_01x06")
    if "PinHeader_1x08" in fp: return ("Connector_Generic", "Conn_01x08")
    if "PinHeader_2x05" in fp: return ("Connector_Generic", "Conn_02x05_Odd_Even")
    raise SystemExit("no symbol for %s (%s)" % (ref, fp))

# ---------------- native symbol embedding ----------------
_libcache = {}
def libtext(lib):
    if lib not in _libcache:
        _libcache[lib] = open("%s/%s.kicad_sym" % (SYMD, lib), encoding="utf-8").read()
    return _libcache[lib]
def raw_symbol(lib, name):
    t = libtext(lib)
    m = re.search(r'\(symbol "%s"' % re.escape(name), t)
    assert m, "%s missing in %s" % (name, lib)
    return t[m.start():bend(t, m.start())]
embedded = {}
pins_of = {}
def parse_pins(blk):
    pins = []
    for pm in re.finditer(r'\(pin\s+\w+\s+\w+', blk):
        pb = blk[pm.start():bend(blk, pm.start())]
        at = re.search(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)(?:\s+([\-\d.]+))?\)', pb)
        num = re.search(r'\(number\s+"([^"]*)"', pb)
        pins.append((num.group(1), float(at.group(1)), float(at.group(2)), float(at.group(3) or 0)))
    return pins
def embed(lib, name):
    key = "%s:%s" % (lib, name)
    if key in embedded: return key
    blk = raw_symbol(lib, name)
    ext = re.search(r'\(extends "([^"]+)"\)', blk)
    if ext:
        parent = ext.group(1)
        blk = raw_symbol(lib, parent)
        blk = blk.replace('(symbol "%s_' % parent, '(symbol "%s_' % name)
        blk = blk.replace('(symbol "%s"' % parent, '(symbol "%s"' % name, 1)
    pins_of[key] = parse_pins(blk)
    blk = blk.replace('(symbol "%s"' % name, '(symbol "%s"' % key, 1)
    embedded[key] = blk
    return key

# ---------------- parse OLD schematic ----------------
s = open(OLD, encoding="utf-8").read()
li = s.find('(lib_symbols'); le = bend(s, li)
oldlibs = s[li:le]; body = s[le:]
paper = re.search(r'\(paper [^\n]*\)', s).group(0)
oldpins = {}
for m in re.finditer(r'\t\(symbol "([^"]+)"\n', oldlibs):
    nm = m.group(1)
    blk = oldlibs[m.start():bend(oldlibs, m.start())]
    short = nm.split(':')[-1]
    ps = []
    for sm in re.finditer(r'\(symbol "%s_\d+_\d+"' % re.escape(short), blk):
        ps.extend(parse_pins(blk[sm.start():bend(blk, sm.start())]))
    if ps: oldpins[nm] = ps
oldsym = []
for m in re.finditer(r'\n\t\(symbol\n', body):
    st = m.start() + 1
    blk = body[st:bend(body, st)]
    lib = re.search(r'\(lib_id "([^"]+)"\)', blk)
    at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+) (\d+)\)', blk)
    mir = re.search(r'\(mirror (\w+)\)', blk)
    ref = re.search(r'\(property "Reference" "([^"]+)"\s*\n\s*\(at ([\-\d.]+) ([\-\d.]+)', blk)
    if not lib or not at: continue
    oldsym.append(dict(ref=ref.group(1) if ref else "?", lib=lib.group(1),
                       x=float(at.group(1)), y=float(at.group(2)), ang=int(at.group(3)),
                       mirror=mir.group(1) if mir else None,
                       rx=float(ref.group(2)) if ref else 0, ry=float(ref.group(3)) if ref else 0))
def blocks(tok):
    return [body[m.start():bend(body, m.start())] for m in re.finditer(r'\(%s\b' % tok, body)]
wires = blocks("wire")
junctions = blocks("junction")
glabels = blocks("global_label")
texts = blocks("text")
rects = blocks("rectangle")

segs = []          # ((x1,y1),(x2,y2))
for w in wires:
    xy = re.findall(r'\(xy ([\-\d.]+) ([\-\d.]+)\)', w)
    for a, b2 in zip(xy, xy[1:]):
        segs.append((rp(float(a[0]), float(a[1])), rp(float(b2[0]), float(b2[1]))))
label_at = []      # (pt, name)
for gl in glabels:
    nm = re.match(r'\(global_label "([^"]*)"', gl).group(1)
    at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)', gl)
    label_at.append((rp(float(at.group(1)), float(at.group(2))), nm))
gnd_pts = set(rp(sy['x'], sy['y']) for sy in oldsym if "Ground-GND" in sy['lib'])
attach = set()
for a, b2 in segs: attach |= {a, b2}
attach |= set(pt for pt, _ in label_at) | gnd_pts
attach_hard = set(attach)      # real furniture (wires/labels/GND) — never self-pins
pin_anchor = {}                # pt -> set of refs whose OLD pin sits there (contact joins)

# ---------------- old-net propagation (union-find over wiring) ----------------
parent = {}
def find(x):
    parent.setdefault(x, x)
    while parent[x] != x:
        parent[x] = parent[parent[x]]; x = parent[x]
    return x
def union(a, b): parent[find(a)] = find(b)
for a, b2 in segs: union(a, b2)
for pt, nm in label_at: union(pt, ("LBL", nm))
for pt in gnd_pts: union(pt, ("LBL", "GND"))
# on-segment touches within old wiring: wire END touching another segment's middle
def on_seg(q, a, b, tol=0.01):
    (x, y), (x1, y1), (x2, y2) = q, a, b
    if not (min(x1, x2) - tol <= x <= max(x1, x2) + tol and min(y1, y2) - tol <= y <= max(y1, y2) + tol):
        return False
    return abs((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)) <= tol * (abs(x2 - x1) + abs(y2 - y1) + tol)
for q in list(attach):
    for a, b2 in segs:
        if q != a and q != b2 and on_seg(q, a, b2): union(q, a)

# transform matrices
MATS = [(1,0,0,1),(0,1,-1,0),(-1,0,0,-1),(0,-1,1,0),(-1,0,0,1),(1,0,0,-1),(0,1,1,0),(0,-1,-1,0)]
def apply(mat, px, py): return (mat[0]*px + mat[1]*py, mat[2]*px + mat[3]*py)

# seed component nets from θ=0/no-mirror instances (transform known: (1,0,0,-1)? calibrated below)
# calibrate the base matrix on the biggest class first (geometric hits)
def cls(sy): return (sy['ang'], sy['mirror'])
base = None; besthits = -1
for mat in MATS:
    hits = 0
    for sy in oldsym:
        if cls(sy) != (0, None) or sy['lib'] not in oldpins: continue
        for (num, px, py, pa) in oldpins[sy['lib']]:
            dx, dy = apply(mat, px, py)
            if rp(sy['x'] + dx, sy['y'] + dy) in attach: hits += 1
    if hits > besthits: besthits, base = hits, mat
print("base matrix for (0,None):", base, "hits:", besthits)
comp_net = {}
def learn(pt, netname):
    r = find(pt)
    if r in comp_net and comp_net[r] != netname:
        print("  NET CONFLICT at", pt, comp_net[r], "vs", netname)
    comp_net[r] = netname
for (pt, nm) in label_at: learn(pt, nm)          # labels carry their own (new==old) names
for pt in gnd_pts: learn(pt, "GND")
for sy in oldsym:
    if cls(sy) != (0, None) or sy['lib'] not in oldpins or sy['ref'] not in inv: continue
    for (num, px, py, pa) in oldpins[sy['lib']]:
        dx, dy = apply(base, px, py)
        pt = rp(sy['x'] + dx, sy['y'] + dy)
        nn = oldnet.get((sy['ref'], num))
        if nn:
            attach.add(pt)   # pins are contact points too (EasyEDA wires-less pin-on-pin joins)
            pin_anchor.setdefault(pt, set()).add(sy['ref'])
            learn(pt, nn)
def net_at(pt):
    r = find(pt)
    return comp_net.get(r)
def seg_net(a): return net_at(a)

# ---------------- per-instance matrix: iterative net-agreement resolution ----------------
# geometric per-class fallback (as prior / tie-break)
class_mat = {}
for c in set(cls(sy) for sy in oldsym):
    besth = (-1, base)
    for mat in MATS:
        hits = 0
        for sy in oldsym:
            if cls(sy) != c or sy['lib'] not in oldpins: continue
            for (num, px, py, pa) in oldpins[sy['lib']]:
                dx, dy = apply(mat, px, py)
                if rp(sy['x'] + dx, sy['y'] + dy) in attach: hits += 1
        if hits > besth[0]: besth = (hits, mat)
    class_mat[c] = besth[1]

def net_score(sy, mat):
    score = hits = 0
    for (num, px, py, pa) in oldpins[sy['lib']]:
        nn = oldnet.get((sy['ref'], num))
        if not nn: continue
        dx, dy = apply(mat, px, py)
        pt = rp(sy['x'] + dx, sy['y'] + dy)
        pn = net_at(pt) if pt in attach else None
        if pn == nn: score += 2; hits += 1
        elif pn is not None: score -= 3
    return score, hits

chosen_mat = {}
work = [sy for sy in oldsym if sy['ref'] != "?" and 'Ground-GND' not in sy['lib']
        and 'Drawing' not in sy['lib'] and sy['lib'] in oldpins and sy['ref'] in inv]
for _round in range(6):
    progress = False
    for sy in work:
        if id(sy) in chosen_mat: continue
        # group matrices by the pin-position map they induce (y-flip of a horizontal 2-pin
        # part is invisible); require a strict win between DISTINCT maps only
        sig = {}
        for mat in MATS:
            key2 = tuple(sorted((num, rp(sy['x'] + apply(mat, px, py)[0],
                                          sy['y'] + apply(mat, px, py)[1]))
                                for (num, px, py, pa) in oldpins[sy['lib']]))
            pref = 2 if mat == class_mat.get(cls(sy)) else (1 if mat == base else 0)
            if key2 not in sig or pref > sig[key2][0]:
                sig[key2] = (pref, mat)
        scored = sorted((net_score(sy, mat)[0], pref, mat) for (pref, mat) in sig.values())
        top = scored[-1]
        if top[0] > 0 and (len(scored) == 1 or top[0] > scored[-2][0]):
            chosen_mat[id(sy)] = top[2]
            for (num, px, py, pa) in oldpins[sy['lib']]:   # learn this symbol's pin nets
                nn = oldnet.get((sy['ref'], num))
                dx, dy = apply(top[2], px, py)
                pt = rp(sy['x'] + dx, sy['y'] + dy)
                if nn:
                    attach.add(pt)                          # pin-contact anchor
                    pin_anchor.setdefault(pt, set()).add(sy['ref'])
                    if net_at(pt) is None: learn(pt, nn)
            progress = True
    if not progress: break
unresolved = [sy['ref'] for sy in work if id(sy) not in chosen_mat]
print("matrix resolved: %d/%d; class-fallback for: %s" %
      (len(chosen_mat), len(work), unresolved))
def pick_matrix(sy):
    return chosen_mat.get(id(sy), class_mat.get(cls(sy), base))

# custom TEENSY41 symbol with the OLD pin geometry (zero stubs) - must exist before emit
def teensy_symbol():
    src = oldpins["ProDoc_P1_-easyedapro:TEENSY41"]
    pins = []
    t = ('    (symbol "Impulse22:TEENSY41"\n      (pin_names (offset 1.016)) (exclude_from_sim no) '
         '(in_bom yes) (on_board yes)\n'
         '      (property "Reference" "U" (at 0 46 0) (effects (font (size 1.27 1.27))))\n'
         '      (property "Value" "Teensy 4.1" (at 0 -46 0) (effects (font (size 1.27 1.27))))\n'
         '      (symbol "TEENSY41_1_1"\n'
         '        (rectangle (start -22.86 44.45) (end 22.86 -44.45) '
         '(stroke (width 0.254) (type solid)) (fill (type background)))\n')
    for (num, px, py, pa) in src:
        nn = tmap_teensy(num)
        if nn is None or any(pn[0] == nn for pn in pins): continue
        pins.append((nn, px, py, pa))
        t += ('        (pin bidirectional line (at %s %s %d) (length 5.08)\n'
              '          (name "%s" (effects (font (size 1.27 1.27))))\n'
              '          (number "%s" (effects (font (size 1.27 1.27)))))\n'
              % (g(px), g(py), int(pa), nn, nn))
    t += "      )\n    )\n"
    pins_of["Impulse22:TEENSY41"] = pins
    embedded["Impulse22:TEENSY41"] = t
teensy_symbol()

def mat2x2_mul(A, B):
    return (A[0]*B[0]+A[1]*B[2], A[0]*B[1]+A[1]*B[3], A[2]*B[0]+A[3]*B[2], A[2]*B[1]+A[3]*B[3])
def mat2x2_inv(M):
    det = M[0]*M[3] - M[1]*M[2]
    return (M[3]//det if det else M[3], -M[1]//det if det else -M[1],
            -M[2]//det if det else -M[2], M[0]//det if det else M[0])

# ---------------- emit ----------------
out_sym = out_wire = out_misc = ""
stub_count = nc_count = nudge_count = 0
placed = set()
newpin_pts = {}    # pt -> net (placed new pins, for collision checks)
pin_nets = []      # (pt, net, ref, pinnum) for final connectivity verification
def conflicts(pts_nets):
    for pt, nn in pts_nets:
        pn = net_at(pt) if pt in attach else None
        if pn is not None and pn != nn: return True
        if pt in newpin_pts and newpin_pts[pt] != nn: return True
        for a, b2 in segs:
            if pt != a and pt != b2 and on_seg(pt, a, b2) and seg_net(a) not in (None, nn):
                return True
    return False
NUDGE = [(0,0),(1.27,0),(-1.27,0),(0,1.27),(0,-1.27),(2.54,0),(-2.54,0),(0,2.54),(0,-2.54),
         (1.27,1.27),(-1.27,-1.27),(1.27,-1.27),(-1.27,1.27),(3.81,0),(-3.81,0),(0,3.81),(0,-3.81)]

def stub_ok(a, b2, nn):
    for pt in list(attach) + list(newpin_pts):
        if pt in (a, b2): continue
        if on_seg(pt, a, b2):
            other = net_at(pt) if pt in attach else newpin_pts.get(pt)
            if other is not None and other != nn: return False
    return True

for sy in oldsym:
    oref = sy['ref']
    if oref == "?" or 'Ground-GND' in sy['lib'] or 'Drawing' in sy['lib']: continue
    ref = REF_RENAME.get(oref, oref)
    if ref not in parts: continue
    placed.add(ref)
    lib, name = symbol_for(ref)
    key = "Impulse22:TEENSY41" if lib == "CUSTOM" else embed(lib, name)
    mp = padmap_for(oref)
    mat = pick_matrix(sy)
    def to_new(onum):
        if mp == "TEENSY": return tmap_teensy(onum)
        if mp is None: return onum
        return mp.get(onum)
    oldabs = {}
    for (num, px, py, pa) in oldpins.get(sy['lib'], []):
        dx, dy = apply(mat, px, py)
        oldabs[num] = rp(sy['x'] + dx, sy['y'] + dy)
    # choose nudge so new pins don't touch foreign nets
    npins = pins_of[key]
    chosen = (0, 0)
    for off in NUDGE:
        trial = []
        for (num, px, py, pa) in npins:
            dx, dy = apply(mat, px, py)
            nn = parts[ref]['pads'].get(num)
            trial.append((rp(sy['x'] + off[0] + dx, sy['y'] + off[1] + dy), nn or "__nc__"))
        # NC pins must not touch anything either
        bad = False
        for pt, nn in trial:
            if nn == "__nc__":
                if (pt in attach and net_at(pt) is not None) or pt in newpin_pts: bad = True; break
                if any(on_seg(pt, a, b2) and pt not in (a, b2) for a, b2 in segs): bad = True; break
        if not bad and not conflicts([(pt, nn) for pt, nn in trial if nn != "__nc__"]):
            chosen = off; break
    else:
        print("  no clean nudge for", ref, "- keeping origin")
    if chosen != (0, 0): nudge_count += 1
    ox, oy = sy['x'] + chosen[0], sy['y'] + chosen[1]
    newabs = {}
    for (num, px, py, pa) in npins:
        dx, dy = apply(mat, px, py)
        newabs[num] = rp(ox + dx, oy + dy)
        nn = parts[ref]['pads'].get(num)
        if nn:
            newpin_pts[newabs[num]] = nn
            pin_nets.append((newabs[num], nn, ref, num))
    # orientation attrs: eeschema pin offset = Mmirror . Rvis(theta) . diag(1,-1) . (px,py);
    # solve that composition for each of the 8 signed permutations:
    MAT2KICAD = {(1,0,0,-1): (0, None), (0,-1,-1,0): (90, None), (-1,0,0,1): (180, None),
                 (0,1,1,0): (270, None), (-1,0,0,-1): (0, 'y'), (1,0,0,1): (0, 'x'),
                 (0,1,-1,0): (90, 'y'), (0,-1,1,0): (90, 'x')}
    ang, mirr = MAT2KICAD[mat]
    mirror = ("\n\t\t(mirror %s)" % mirr) if mirr else ""
    out_sym += ('  (symbol (lib_id "%s") (at %s %s %d)%s (unit 1) (exclude_from_sim no) '
                '(in_bom %s) (on_board yes) (dnp no)\n    (uuid %s)\n'
                % (key, g(ox), g(oy), ang, mirror,
                   "no" if ref.startswith("H") else "yes", sym_uuids[ref]))
    out_sym += ('    (property "Reference" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) '
                '(justify left bottom)))\n'
                '    (property "Value" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) '
                '(justify left top)))\n'
                '    (property "Footprint" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
                % (ref, g(sy['rx']), g(sy['ry']), parts[ref]['val'].replace('"', ''),
                   g(sy['rx']), g(sy['ry'] + 2.6), parts[ref]['fp'], g(ox), g(oy)))
    for (num, _px, _py, _pa) in npins:
        out_sym += '    (pin "%s" (uuid %s))\n' % (num, nid())
    out_sym += ('    (instances (project "Impulse_2.2" (path "/%s" (reference "%s") (unit 1))))\n  )\n'
                % (ROOT, ref))
    # stubs (target must be real furniture or ANOTHER symbol's pin contact, not our own)
    def is_attached(opt2):
        return opt2 in attach_hard or bool(pin_anchor.get(opt2, set()) - {oref})
    for onum, opt in oldabs.items():
        nn2 = to_new(onum)
        if nn2 is None or nn2 not in newabs: continue
        if not is_attached(opt): continue
        npt = newabs[nn2]
        if npt == opt: continue
        netname = parts[ref]['pads'].get(nn2) or "?"
        if stub_ok(npt, opt, netname):
            out_wire += ('  (wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) '
                         '(uuid %s))\n' % (g(npt[0]), g(npt[1]), g(opt[0]), g(opt[1]), nid()))
        else:  # L-route fallbacks
            m1 = rp(npt[0], opt[1]); m2 = rp(opt[0], npt[1])
            for mid in (m1, m2):
                if stub_ok(npt, mid, netname) and stub_ok(mid, opt, netname):
                    for a, b2 in ((npt, mid), (mid, opt)):
                        out_wire += ('  (wire (pts (xy %s %s) (xy %s %s)) '
                                     '(stroke (width 0) (type default)) (uuid %s))\n'
                                     % (g(a[0]), g(a[1]), g(b2[0]), g(b2[1]), nid()))
                    break
            else:
                print("  STUB CONFLICT %s pin %s (diagonal emitted anyway)" % (ref, nn2))
                out_wire += ('  (wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) '
                             '(uuid %s))\n' % (g(npt[0]), g(npt[1]), g(opt[0]), g(opt[1]), nid()))
        stub_count += 1
    # no_connects + safety labels for pins whose old drawing had no attachment
    for (num, _px, _py, _pa) in npins:
        netname = parts[ref]['pads'].get(num)
        pt = newabs[num]
        if netname is None:
            if not ref.startswith("H"):
                out_misc += '  (no_connect (at %s %s) (uuid %s))\n' % (g(pt[0]), g(pt[1]), nid())
                nc_count += 1
        else:
            had = any(to_new(onum) == num and is_attached(oldabs[onum]) for onum in oldabs)
            if not had:
                out_misc += ('  (global_label "%s" (shape input) (at %s %s 0) '
                             '(effects (font (size 1.27 1.27)) (justify left))\n    (uuid %s)\n'
                             '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) '
                             '(hide yes) (effects (font (size 1.27 1.27))))\n  )\n'
                             % (netname, g(pt[0]), g(pt[1]), nid(), g(pt[0]), g(pt[1])))

# ---------------- PCB-only parts strip ----------------
missing = set(parts) - placed
hx = 30
LBL = {0: (180, "right"), 180: (0, "left"), 90: (270, "left"), 270: (90, "left")}
for ref in sorted(missing):
    lib, name = symbol_for(ref)
    key = embed(lib, name)
    out_sym += ('  (symbol (lib_id "%s") (at %s 480 0) (unit 1) (exclude_from_sim no) '
                '(in_bom %s) (on_board yes) (dnp no)\n    (uuid %s)\n'
                '    (property "Reference" "%s" (at %s 473 0) (effects (font (size 1.27 1.27))))\n'
                '    (property "Value" "%s" (at %s 488 0) (effects (font (size 1.27 1.27))))\n'
                '    (property "Footprint" "%s" (at %s 480 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
                % (key, g(hx), "no" if ref.startswith("H") else "yes", sym_uuids[ref],
                   ref, g(hx), parts[ref]['val'].replace('"', ''), g(hx), parts[ref]['fp'], g(hx)))
    for (num, _px, _py, _pa) in pins_of[key]:
        out_sym += '    (pin "%s" (uuid %s))\n' % (num, nid())
    out_sym += ('    (instances (project "Impulse_2.2" (path "/%s" (reference "%s") (unit 1))))\n  )\n'
                % (ROOT, ref))
    for (num, px, py, pa) in pins_of[key]:
        ax, ay = hx + px, 480 - py
        netname = parts[ref]['pads'].get(num)
        if netname: pin_nets.append((rp(ax, ay), netname, ref, num))
        if netname:
            la, ju = LBL[int(pa) % 360]
            out_misc += ('  (global_label "%s" (shape input) (at %s %s %d) '
                         '(effects (font (size 1.27 1.27)) (justify %s))\n    (uuid %s)\n'
                         '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) (hide yes) '
                         '(effects (font (size 1.27 1.27))))\n  )\n'
                         % (netname, g(ax), g(ay), la, ju, nid(), g(ax), g(ay)))
        elif not ref.startswith("H"):
            out_misc += '  (no_connect (at %s %s) (uuid %s))\n' % (g(ax), g(ay), nid())
    hx += 30

# ---------------- furniture ----------------
for w in wires: out_wire += "  " + w + "\n"
for j in junctions: out_misc += "  " + j + "\n"
for r in rects: out_misc += "  " + r + "\n"
for t in texts: out_misc += "  " + t + "\n"
for gl in glabels: out_misc += "  " + gl + "\n"
embed("power", "GND")
for i, pt in enumerate(sorted(gnd_pts)):
    out_sym += ('  (symbol (lib_id "power:GND") (at %s %s 0) (unit 1) (exclude_from_sim no) '
                '(in_bom no) (on_board yes) (dnp no)\n    (uuid %s)\n'
                '    (property "Reference" "#PWR0%03d" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
                '    (property "Value" "GND" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
                '    (pin "1" (uuid %s))\n'
                '    (instances (project "Impulse_2.2" (path "/%s" (reference "#PWR0%03d") (unit 1))))\n  )\n'
                % (g(pt[0]), g(pt[1]), nid(), i + 1, g(pt[0]), g(pt[1] - 4),
                   g(pt[0]), g(pt[1] + 4), nid(), ROOT, i + 1))

# net-name labels for renamed wire-only nets: anchor on an actual NEW pin of that net
# (guaranteed reachable via its stub), so the label can't miss the wiring by a sub-grid hair.
net_pin_pts = {}
for pt, nn, ref, num in pin_nets:
    net_pin_pts.setdefault(nn, pt)
lbl_done = set()
for old, new in NET_RENAME.items():
    if new == "BUTTON": continue
    pt = net_pin_pts.get(new)
    if pt:
        out_misc += ('  (global_label "%s" (shape input) (at %s %s 0) '
                     '(effects (font (size 1.27 1.27)) (justify left))\n    (uuid %s)\n'
                     '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) (hide yes) '
                     '(effects (font (size 1.27 1.27))))\n  )\n'
                     % (new, g(pt[0]), g(pt[1]), nid(), g(pt[0]), g(pt[1])))
        lbl_done.add(new)
print("net-name labels placed:", len(lbl_done), "of", len(set(NET_RENAME.values())) - 1)

# PWR_FLAGs
embed("power", "PWR_FLAG")
pf = pins_of["power:PWR_FLAG"][0]
PWR = ["GND", "3.3V", "5V-CLEAN", "5V-DIRTY", "7.4V", "VBAT_MAIN", "VBAT_SW",
       "BUCK_5V", "LOGIC_5V_IN", "SERVO_5V_IN"]
for i, netname in enumerate(PWR):
    ix, iy = 40 + i * 25, 505
    out_sym += ('  (symbol (lib_id "power:PWR_FLAG") (at %s %s 0) (unit 1) (exclude_from_sim no) '
                '(in_bom no) (on_board yes) (dnp no)\n    (uuid %s)\n'
                '    (property "Reference" "#FLG%02d" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
                '    (property "Value" "PWR_FLAG" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
                '    (pin "1" (uuid %s))\n'
                '    (instances (project "Impulse_2.2" (path "/%s" (reference "#FLG%02d") (unit 1))))\n  )\n'
                % (g(ix), g(iy), nid(), i + 1, g(ix), g(iy - 4), g(ix), g(iy - 6), nid(), ROOT, i + 1))
    out_misc += ('  (global_label "%s" (shape input) (at %s %s 270) '
                 '(effects (font (size 1.27 1.27)) (justify left))\n    (uuid %s)\n'
                 '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) (hide yes) '
                 '(effects (font (size 1.27 1.27))))\n  )\n'
                 % (netname, g(ix + pf[1]), g(iy - pf[2]), nid(), g(ix), g(iy)))

# ---------------- closed-loop connectivity verification ----------------
# model KiCad's connectivity on the EMITTED geometry; any pin that can't reach a label/GND
# naming its net gets a safety label; wire-end-on-wire-middle T-joints get junction dots.
all_segs = list(segs)
for m in re.finditer(r'\(wire \(pts \(xy ([\-\d.]+) ([\-\d.]+)\) \(xy ([\-\d.]+) ([\-\d.]+)\)\)',
                     out_wire):
    all_segs.append((rp(float(m.group(1)), float(m.group(2))),
                     rp(float(m.group(3)), float(m.group(4)))))
all_lbl = list(label_at) and [(pt, nm) for pt, nm in label_at]
for m in re.finditer(r'\(global_label "([^"]+)" \(shape input\) \(at ([\-\d.]+) ([\-\d.]+)',
                     out_misc):
    all_lbl.append((rp(float(m.group(2)), float(m.group(3))), m.group(1)))
vparent = {}
def vfind(x):
    vparent.setdefault(x, x)
    while vparent[x] != x:
        vparent[x] = vparent[vparent[x]]; x = vparent[x]
    return x
def vunion(a, b): vparent[vfind(a)] = vfind(b)
nodes = set()
for a, b2 in all_segs: vunion(a, b2); nodes |= {a, b2}
for pt, nm in all_lbl: vunion(pt, ("N", nm)); nodes.add(pt)
for pt in gnd_pts: vunion(pt, ("N", "GND")); nodes.add(pt)
for pt, nn, ref, num in pin_nets: nodes.add(pt)
old_junc = set(junc_pts) if 'junc_pts' in dir() else set()
old_junc = set()
for j in junctions:
    at = re.search(r'\(at ([\-\d.]+) ([\-\d.]+)\)', j)
    old_junc.add(rp(float(at.group(1)), float(at.group(2))))
new_junc = set()
for q in nodes:
    for a, b2 in all_segs:
        if q != a and q != b2 and on_seg(q, a, b2):
            vunion(q, a)
            if q not in old_junc: new_junc.add(q)
fixed = 0
for pt, nn, ref, num in pin_nets:
    names = set()
    r0 = vfind(pt)
    # collect the component's label names
    for pt2, nm in all_lbl:
        if vfind(pt2) == r0: names.add(nm)
    if vfind(("N", "GND")) == r0: names.add("GND")
    if nn not in names:
        out_misc += ('  (global_label "%s" (shape input) (at %s %s 0) '
                     '(effects (font (size 1.27 1.27)) (justify left))\n    (uuid %s)\n'
                     '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) (hide yes) '
                     '(effects (font (size 1.27 1.27))))\n  )\n'
                     % (nn, g(pt[0]), g(pt[1]), nid(), g(pt[0]), g(pt[1])))
        fixed += 1
for q in sorted(new_junc):
    out_misc += ('  (junction (at %s %s) (diameter 0) (color 0 0 0 0) (uuid %s))\n'
                 % (g(q[0]), g(q[1]), nid()))
print("verification: %d pins needed safety labels; %d T-joint junctions added" % (fixed, len(new_junc)))

sch = ('(kicad_sch\n  (version 20231120)\n  (generator "eeschema")\n  (generator_version "8.0")\n'
       '  (uuid %s)\n  %s\n  (lib_symbols\n%s  )\n%s%s%s'
       '  (sheet_instances (path "/" (page "1")))\n)\n'
       % (ROOT, paper, "".join("    " + e.strip() + "\n" for e in embedded.values()),
          out_sym, out_wire, out_misc))
open(OUT, "w", encoding="utf-8").write(sch)
json.dump([[pt[0], pt[1], nn, ref, num] for pt, nn, ref, num in pin_nets],
          open("_pin_nets.json", "w"))
print("wrote %s: symbols=%d stubs=%d nudges=%d no_connects=%d" %
      (OUT, len(placed) + len(missing), stub_count, nudge_count, nc_count))
