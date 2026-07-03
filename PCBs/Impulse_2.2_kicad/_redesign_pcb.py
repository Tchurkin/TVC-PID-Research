#!/usr/bin/env python3
# PCB side of the power redesign: remove the 15 old regulator parts, add the two buck clusters
# (back side, where the old regs were), the pyro bulk cap and SERVO_EN jumper (front), with nets
# from the new schematic netlist and (path) links to the new symbols.
import re, json, uuid, math

PCB = "Impulse_2.2.kicad_pcb"; NL = "_netlist_new.net"
FPD = "C:/Program Files/KiCad/10.0/share/kicad/footprints"
uu = json.load(open("_new_uuids.json"))
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
def g(v): return ('%.4f' % v).rstrip('0').rstrip('.')

# nets per (ref,pin) from schematic netlist
n = open(NL, encoding="utf-8").read()
pin_net = {}
for m in re.finditer(r'\(net\s+\(code "[^"]*"\)\s+\(name "([^"]+)"\)', n):
    blk = n[m.start():bend(n, m.start())]
    for a, b in re.findall(r'\(node\s+\(ref "([^"]+)"\)\s+\(pin "([^"]+)"\)', blk):
        if not m.group(1).startswith("unconnected-"): pin_net[(a, b)] = m.group(1)

p = open(PCB, encoding="utf-8").read()
# ---- delete old footprints ----
for ref in DEAD:
    m = re.search(r'\(property "Reference" "%s"' % re.escape(ref), p)
    assert m, ref
    fs = p.rfind('(footprint', 0, m.start()); fe = bend(p, fs); q = fe
    while q < len(p) and p[q] in ' \t\n': q += 1
    p = p[:fs] + p[q:]
# ---- rename surviving old net ----
p = p.replace('"Net-(SW1-B)"', '"VBAT"')

# ---- footprint loader + back-side flip (same approach as _gen_pcb.py) ----
_cache = {}
def flip_mod(body):
    def negx(m): return '(%s %s %s' % (m.group(1), g(-float(m.group(2))), m.group(3))
    body = re.sub(r'\((at|start|end|center|mid|xy)\s+([\-\d.]+)\s+([\-\d.]+)', negx, body)
    def negang(m): return '(at %s %s %s)' % (m.group(1), m.group(2), g((-float(m.group(3))) % 360))
    body = re.sub(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)\s+([\-\d.]+)\)', negang, body)
    for a, b in [("F.Cu", "B.Cu"), ("F.Mask", "B.Mask"), ("F.Paste", "B.Paste"),
                 ("F.SilkS", "B.SilkS"), ("F.Fab", "B.Fab"), ("F.CrtYd", "B.CrtYd")]:
        body = body.replace('"%s"' % a, '"@@T@@"').replace('"%s"' % b, '"%s"' % a).replace('"@@T@@"', '"%s"' % b)
    return body
def load_mod(lib, name, back):
    key = (lib, name, back)
    if key in _cache: return _cache[key]
    t = open("%s/%s.pretty/%s.kicad_mod" % (FPD, lib, name), encoding="utf-8").read()
    i = t.find('(footprint'); body = t[i:bend(t, i)]
    body = re.sub(r'\(version [^)]*\)\s*|\(generator[^)]*\)\s*|\(generator_version [^)]*\)\s*', '', body)
    if back: body = flip_mod(body)
    _cache[key] = body
    return body

def add_fp(ref, val, lib, name, x, y, rot, back):
    global p
    body = load_mod(lib, name, back)
    body = re.sub(r'^\(footprint\s+"[^"]*"', '', body, count=1).rstrip()[:-1]
    body = re.sub(r'\(property\s+"Reference"\s+"[^"]*"', '(property "Reference" "%s"' % ref, body, count=1)
    body = re.sub(r'\(property\s+"Value"\s+"[^"]*"', '(property "Value" "%s"' % val, body, count=1)
    out = []; last = 0
    for m in re.finditer(r'\(pad\s+"([^"]*)"', body):
        ps = m.start(); pe = bend(body, ps); pb = body[ps:pe]
        if rot:
            am = re.search(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)(?:\s+([\-\d.]+))?\)', pb)
            pa = (float(am.group(3)) if am.group(3) else 0) + rot
            pb = pb[:am.start()] + '(at %s %s %s)' % (am.group(1), am.group(2), g(pa % 360)) + pb[am.end():]
        netname = pin_net.get((ref, m.group(1)))
        if netname and 'np_thru_hole' not in pb:
            pb = pb[:-1] + ' (net "%s")' % netname + ')'
        out.append(body[last:ps]); out.append(pb); last = pe
    out.append(body[last:])
    body = "".join(out)
    hdr = ('\t(footprint "%s:%s"\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t\t(at %s %s%s)\n'
           '\t\t(path "/%s")\n' % (lib, name, "B.Cu" if back else "F.Cu", nid(), g(x), g(y),
                                   (" " + g(rot)) if rot else "", uu[ref]))
    idx = p.rstrip().rfind(')')
    p = p[:idx] + hdr + body + "\n\t)\n" + p[idx:]

# ---- placements ----
# logic buck: old U4 area (back, ~40.8,49.7); servo buck: old U8 area (back, ~171.5,22.9)
TS = "Package_TO_SOT_SMD"; CS = "Capacitor_SMD"; IS_ = "Inductor_SMD"; CH = "Connector_PinHeader_2.54mm"
add_fp("U2", "AP63205WU-7", TS, "TSOT-23-6", 40.8, 49.7, 0, True)
add_fp("L1", "6.8uH", IS_, "L_Bourns_SRN6045TA", 47.5, 49.7, 0, True)
add_fp("C1", "10uF 25V", CS, "C_1206_3216Metric", 36.0, 47.5, 90, True)
add_fp("C3", "100nF", CS, "C_0603_1608Metric", 40.8, 45.6, 0, True)
add_fp("C5", "22uF 16V", CS, "C_1206_3216Metric", 52.5, 47.0, 90, True)
add_fp("C6", "22uF 16V", CS, "C_1206_3216Metric", 55.0, 47.0, 90, True)
add_fp("U3", "AP63305WU-7", TS, "TSOT-23-6", 171.5, 22.9, 0, True)
add_fp("L2", "4.7uH", IS_, "L_Bourns_SRN6045TA", 178.2, 22.9, 0, True)
add_fp("C2", "10uF 25V", CS, "C_1206_3216Metric", 166.7, 20.7, 90, True)
add_fp("C4", "100nF", CS, "C_0603_1608Metric", 171.5, 18.8, 0, True)
add_fp("C7", "22uF 16V", CS, "C_1206_3216Metric", 183.2, 20.2, 90, True)
add_fp("C8", "22uF 16V", CS, "C_1206_3216Metric", 185.7, 20.2, 90, True)
add_fp("C10", "220uF 25V", CS, "CP_Elec_8x10", 70.0, 76.5, 0, False)      # near pyro outputs
add_fp("SERVO_EN1", "servo disable", CH, "PinHeader_1x02_P2.54mm_Vertical", 160.0, 64.0, 0, False)
open(PCB, "w", encoding="utf-8").write(p)
print("PCB: removed %d, added 14 footprints" % len(DEAD))
