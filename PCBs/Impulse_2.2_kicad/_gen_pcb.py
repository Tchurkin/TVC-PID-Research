#!/usr/bin/env python3
# Impulse 2.2: regenerate the PCB from the EasyEDA-imported Impulse 2.1, using NATIVE KiCad
# footprints. Wiring (netlist) is copied exactly (with readable net renames + the BUTTON pad3/4
# merge, which the switch internally bonds). Placement copied; per-part rotation chosen to best
# reproduce the old pad positions. Custom clean footprints only for TEENSY41 and the 9mm buzzer.
import re, math, json, uuid

SRC = "../Impulse_2.1_kicad/_inventory.json"
OLD = "../Impulse_2.1_kicad/Impulse_2.1.kicad_pcb"
OUT = "Impulse_2.2.kicad_pcb"
FPD = "C:/Program Files/KiCad/10.0/share/kicad/footprints"
ROOT_SHEET = "11111111-2222-3333-4444-000000000001"   # fixed root uuid shared with the schematic
SYM_UUID_FILE = "_sym_uuids.json"                      # ref -> schematic symbol uuid (shared)

def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
    raise ValueError("unbalanced")
def nid(): return str(uuid.uuid4())
def g(v): return ('%.4f' % v).rstrip('0').rstrip('.')
def rot(x, y, deg):                       # KiCad board frame: y down, +angle = CCW on screen
    a = math.radians(deg)
    return (x*math.cos(a) + y*math.sin(a), -x*math.sin(a) + y*math.cos(a))

inv = {f['ref']: f for f in json.load(open(SRC))}

# ---------------- net renames + merges ----------------
RENAME = {"$1N50225": "VBAT_MAIN", "$1N50240": "VBAT_SW", "$1N49822": "SERVO_5V_IN",
          "$1N52582": "LOGIC_5V_IN", "$1N33870": "BUCK_SW", "$1N52591": "BUCK_5V",
          "$1N33864": "BUCK_FB", "$1N50177": "PYRO_ARM",
          "$1N14307": "RGB_B", "$1N14222": "RGB_G", "$1N14265": "RGB_R",
          "$1N42243": "PYRO1_OUT", "$1N42245": "PYRO1_LED",
          "$1N42393": "PYRO2_OUT", "$1N42395": "PYRO2_LED",
          "$1N56715": "PYRO3_OUT", "$1N42562": "PYRO3_LED",
          "$1N56609": "PYRO4_OUT", "$1N56605": "PYRO4_LED",
          "$1N14383": "BUTTON"}                        # merge: switch pins 3/4 internally bonded
def net(n): return RENAME.get(n, n) if n else None

# refs must end in a digit for KiCad annotation; also disambiguate MAIN vs MAIN1
REF_RENAME = {"X": "X1", "Y": "Y1", "Out": "OUT1", "PYRO": "PYRO1", "PYRO_ON": "PYRO_ON1",
              "BUTTON": "BUTTON1", "BUZZER": "BUZZER1", "LED": "LED1",
              "MAIN": "BAT_MAIN1", "MAIN1": "AUX_3V3_1",
              "DROK_exclusive": "DROK_exclusive1", "logic_power_reg": "logic_power_reg1",
              "power_tap": "power_tap1", "servo_power_reg": "servo_power_reg1",
              "J_SENSOR": "J_SENSOR1", "P1O": "P1O1", "P2O": "P2O1", "P3O": "P3O1", "P4O": "P4O1"}
def newref(r): return REF_RENAME.get(r, r)

# ---------------- values ----------------
VALUES = {"R23": "10k", "R37": "10k", "R43": "10k", "R44": "10k", "R45": "10k", "R47": "10k",
          "R48": "10k", "R49": "10k", "R50": "10k", "R38": "220", "R39": "220", "R40": "220",
          "R9": "1k", "R8": "3.05k", "C22": "4.7nF", "C33": "100nF", "C38": "120uF",
          "C39": "68uF", "C43": "470uF", "L4": "68uH", "D8": "SS34", "D9": "SS34", "D10": "SS34",
          "Q13": "IRLML6344", "Q14": "IRLML6344", "Q15": "IRLML6344", "Q16": "IRLML6344",
          "U4": "LM2594M-ADJ", "U1": "Teensy 4.1", "U6": "MPU-6050 Module", "U7": "BMP280 Module",
          "U8": "DROK Buck 3A", "LED": "XL-3528RGBW-HM", "BUZZER": "Buzzer 2.7kHz",
          "BUTTON": "TS665CJ", "SW1": "DSWB01LHGET", "P1": "LED red", "P2": "LED red",
          "P3": "LED red", "P4": "LED red", "J_SENSOR": "Sensor board 2x5"}
def value(ref, f):
    if ref in VALUES: return VALUES[ref]
    if f['fp'].endswith("HDR-TH_2P-P2.54-V-M"): return "Conn_01x02"
    if f['fp'].endswith("HDR-TH_3P-P2.54-V-M"): return "Conn_01x03"
    if "MountingHole" in f['fp']: return "MountingHole_3.2mm_M3"
    return f['val']

# ---------------- footprint mapping ----------------
IDENT = None
def fam(f):
    fp = f['fp'].split(':')[-1]
    if fp == "R0603": return ("Resistor_SMD", "R_0603_1608Metric", IDENT)
    if fp == "R0402": return ("Resistor_SMD", "R_0402_1005Metric", IDENT)
    if fp == "C0402": return ("Capacitor_SMD", "C_0402_1005Metric", IDENT)
    if fp == "C0603": return ("Capacitor_SMD", "C_0603_1608Metric", IDENT)
    if fp.startswith("CAP-SMD_BD6.3"): return ("Capacitor_SMD", "CP_Elec_6.3x7.7", IDENT)
    if fp == "HDR-TH_2P-P2.54-V-M": return ("Connector_PinHeader_2.54mm", "PinHeader_1x02_P2.54mm_Vertical", IDENT)
    if fp == "HDR-TH_3P-P2.54-V-M": return ("Connector_PinHeader_2.54mm", "PinHeader_1x03_P2.54mm_Vertical", IDENT)
    if fp == "PinHeader_2x05_P2.54mm_Vertical": return ("Connector_PinHeader_2.54mm", "PinHeader_2x05_P2.54mm_Vertical", IDENT)
    if fp.startswith("SMA_"): return ("Diode_SMD", "D_SMA", IDENT)
    if fp.startswith("SOT-23-3"): return ("Package_TO_SOT_SMD", "SOT-23", IDENT)
    if fp.startswith("SOP-8"): return ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm", IDENT)
    if fp == "LED0805-R-RD": return ("LED_SMD", "LED_0805_2012Metric", {"1": "2", "2": "1"})  # old1=A -> KiCad pad2=A
    if fp.startswith("LED-SMD_4P"): return ("LED_SMD", "LED_RGB_Wuerth-PLCC4_3.2x2.8mm_150141M173100",
                                            {"1": "1", "2": "2", "3": "4", "4": "3"})          # position match
    if fp == "IND-SMD_L5.8-W5.2": return ("Inductor_SMD", "L_Bourns_SRN6045TA", IDENT)
    if fp.startswith("SW-TH_4P"): return ("Button_Switch_THT", "SW_PUSH_6mm", {"1": "1", "2": "1", "3": "2", "4": "2"})
    if fp == "SW-TH_DSWB01LHGET": return ("Button_Switch_THT", "SW_DIP_SPSTx01_Slide_6.7x4.1mm_W7.62mm_P2.54mm_LowProfile", IDENT)
    if fp == "MPU-6050 MODULE": return ("Connector_PinHeader_2.54mm", "PinHeader_1x08_P2.54mm_Vertical", IDENT)
    if fp == "BMP280": return ("Connector_PinHeader_2.54mm", "PinHeader_1x06_P2.54mm_Vertical", IDENT)
    if fp == "BUCK CONVERTER-DROK": return ("Connector_PinHeader_2.54mm", "PinHeader_1x05_P2.54mm_Vertical", IDENT)
    if fp.startswith("MountingHole"): return ("MountingHole", "MountingHole_3.2mm_M3", IDENT)
    if fp == "TEENSY41": return ("Impulse22", "TEENSY41", "TEENSY")
    if fp.startswith("BUZ-TH"): return ("Impulse22", "Buzzer_D9.0mm_P4.00mm", IDENT)
    raise SystemExit("no mapping for footprint %s" % fp)

# Teensy pad rename old->new
def teensy_padmap(oldpads):
    mp = {}
    for pd in oldpads:
        n = pd['n']
        if n == "VIN": mp[n] = "VIN"
        elif n == "GND1": mp[n] = "GND1"
        elif n == "GND3": mp[n] = "GND2"
        elif n == "GND6": mp[n] = "GND3"
        elif n == "3V3-1": mp[n] = "3V3_1"
        elif n == "3V3-3": mp[n] = "3V3_2"
        else:
            m = re.match(r'(\d+)', n)
            assert m, "unexpected teensy pad %s" % n
            mp[n] = m.group(1)
    return mp

# ---------------- custom footprints (clean, project lib) ----------------
def teensy_fp_body(oldpads, padnets):
    mp = teensy_padmap(oldpads)
    s = ""
    for pd in oldpads:
        nn = mp[pd['n']]; x, y = pd['at'][0], pd['at'][1]
        shape = "rect" if nn == "GND1" else "circle"
        nete = (' (net "%s")' % padnets[nn]) if padnets.get(nn) else ""
        s += ('\t\t(pad "%s" thru_hole %s (at %s %s) (size 1.6 1.6) (drill 1.02) '
              '(layers "*.Cu" "*.Mask")%s (uuid "%s"))\n' % (nn, shape, g(x), g(y), nete, nid()))
    # outline: silk + courtyard + fab
    x0, x1, y0, y1 = -31.75, 29.21, -8.89, 8.89
    for (lay, w, off) in [("F.SilkS", 0.12, 0.11), ("F.CrtYd", 0.05, 0.25), ("F.Fab", 0.1, 0.0)]:
        a, b, c, d = x0-off, x1+off, y0-off, y1+off
        for (p1, p2) in [((a, c), (b, c)), ((b, c), (b, d)), ((b, d), (a, d)), ((a, d), (a, c))]:
            s += ('\t\t(fp_line (start %s %s) (end %s %s) (stroke (width %s) (type solid)) '
                  '(layer "%s") (uuid "%s"))\n' % (g(p1[0]), g(p1[1]), g(p2[0]), g(p2[1]), w, lay, nid()))
    s += ('\t\t(fp_circle (center -30.48 10.5) (end -30.08 10.5) (stroke (width 0.2) (type solid)) '
          '(fill no) (layer "F.SilkS") (uuid "%s"))\n' % nid())   # pin-1 (GND1) dot
    return s

def buzzer_fp_body(padnets):
    s = ""
    for (nn, x, shape) in [("1", 2.0, "rect"), ("2", -2.0, "circle")]:
        nete = (' (net "%s")' % padnets[nn]) if padnets.get(nn) else ""
        s += ('\t\t(pad "%s" thru_hole %s (at %s 0) (size 1.8 1.8) (drill 1.02) '
              '(layers "*.Cu" "*.Mask")%s (uuid "%s"))\n' % (nn, shape, g(x), nete, nid()))
    s += ('\t\t(fp_circle (center 0 0) (end 4.5 0) (stroke (width 0.12) (type solid)) (fill no) '
          '(layer "F.SilkS") (uuid "%s"))\n' % nid())
    s += ('\t\t(fp_circle (center 0 0) (end 4.75 0) (stroke (width 0.05) (type solid)) (fill no) '
          '(layer "F.CrtYd") (uuid "%s"))\n' % nid())
    s += ('\t\t(fp_text user "+" (at 3.6 -2.2) (layer "F.SilkS") (uuid "%s") '
          '(effects (font (size 1 1) (thickness 0.15))))\n' % nid())
    return s

# ---------------- native footprint loader ----------------
_cache = {}
def load_mod(lib, name, back=False):
    key = (lib, name, back)
    if key in _cache: return _cache[key]
    t = open("%s/%s.pretty/%s.kicad_mod" % (FPD, lib, name), encoding="utf-8").read()
    # body = everything inside (footprint "name" ... ) minus header tokens we replace
    i = t.find('(footprint'); j = bend(t, i)
    body = t[i:j]
    # strip the outer name line pieces: version/generator lines
    body = re.sub(r'\(version [^)]*\)\s*', '', body)
    body = re.sub(r'\(generator [^)]*\)\s*', '', body)
    body = re.sub(r'\(generator_version [^)]*\)\s*', '', body)
    if back:
        body = flip_mod(body)
    _cache[key] = body
    return body

def flip_mod(body):
    """Mirror a front-side footprint body to the back, the way KiCad's Flip does:
    negate local x, negate angles, swap F.* <-> B.* layers."""
    def negx(m):
        return '(%s %s %s' % (m.group(1), g(-float(m.group(2))), m.group(3))
    body = re.sub(r'\((at|start|end|center|mid|xy)\s+([\-\d.]+)\s+([\-\d.]+)', negx, body)
    def negang(m):
        return '(at %s %s %s)' % (m.group(1), m.group(2), g((-float(m.group(3))) % 360))
    body = re.sub(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)\s+([\-\d.]+)\)', negang, body)
    for a, b in [("F.Cu", "B.Cu"), ("F.Mask", "B.Mask"), ("F.Paste", "B.Paste"),
                 ("F.SilkS", "B.SilkS"), ("F.Fab", "B.Fab"), ("F.CrtYd", "B.CrtYd")]:
        body = body.replace('"%s"' % a, '"@@TMP@@"').replace('"%s"' % b, '"%s"' % a).replace('"@@TMP@@"', '"%s"' % b)
    return body

def mod_pads(body):
    out = []
    for m in re.finditer(r'\(pad\s+"([^"]*)"', body):
        pb = body[m.start():bend(body, m.start())]
        at = re.search(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)(?:\s+([\-\d.]+))?\)', pb)
        out.append((m.group(1), float(at.group(1)), float(at.group(2)), m.start(), bend(body, m.start())))
    return out

# ---------------- placement solver ----------------
def place(f, lib, name, padmap):
    """returns (origin_x, origin_y, rotation, {newpad->net}, worst_err)"""
    oldabs = {}
    for pd in f['pads']:
        ax, ay = rot(pd['at'][0], pd['at'][1], f['at'][2])
        oldabs[pd['n']] = (f['at'][0]+ax, f['at'][1]+ay, net(pd['net']))
    if padmap == "TEENSY":
        mp = teensy_padmap(f['pads'])
    elif padmap is None:
        mp = {pd['n']: pd['n'] for pd in f['pads']}
    else:
        mp = padmap
    # nets per new pad number
    padnets = {}
    for old, newn in mp.items():
        nv = oldabs[old][2]
        if nv: padnets.setdefault(newn, nv)
    if lib == "Impulse22":   # customs keep exact geometry: origin/rot = old
        return f['at'][0], f['at'][1], f['at'][2], padnets, 0.0
    body = load_mod(lib, name, back=(f['layer'] != 'F.Cu'))
    npads = mod_pads(body)
    # centroids over mapped pads
    tgt = [(oldabs[o][0], oldabs[o][1], mp[o]) for o in mp]
    best = None
    for th in (0, 90, 180, 270):
        newpos = {}
        for (nn, x, y, _, _) in npads:
            newpos.setdefault(nn, []).append(rot(x, y, th))
        ctg = (sum(t[0] for t in tgt)/len(tgt), sum(t[1] for t in tgt)/len(tgt))
        allnp = [v for vs in newpos.values() for v in vs]
        cnp = (sum(v[0] for v in allnp)/len(allnp), sum(v[1] for v in allnp)/len(allnp))
        ox, oy = ctg[0]-cnp[0], ctg[1]-cnp[1]
        err = 0.0; wmax = 0.0
        for (txp, typ, nn) in tgt:
            cands = newpos.get(nn)
            if not cands: err += 99; continue
            dd = min(math.hypot(txp-(ox+cx), typ-(oy+cy)) for cx, cy in cands)
            err += dd; wmax = max(wmax, dd)
        if best is None or err < best[0]:
            best = (err, th, ox, oy, wmax)
    _, th, ox, oy, wmax = best
    return ox, oy, th, padnets, wmax

# ---------------- build footprints ----------------
sym_uuids = json.load(open(SYM_UUID_FILE)) if __import__('os').path.exists(SYM_UUID_FILE) else {}
def sym_uuid(ref):
    if ref not in sym_uuids: sym_uuids[ref] = nid()
    return sym_uuids[ref]

fps_out = []
report = []
for oldref, f in sorted(inv.items()):
    ref = newref(oldref)
    lib, name, padmap = fam(f)
    back = (f['layer'] != 'F.Cu')
    ox, oy, th, padnets, wmax = place(f, lib, name, padmap)
    fpid = "%s:%s" % (lib, name)
    L = lambda front: front.replace("F.", "B.") if back else front
    hdr = ('\t(footprint "%s"\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t\t(at %s %s%s)\n'
           '\t\t(path "/%s")\n'
           % (fpid, L("F.Cu"), nid(), g(ox), g(oy), (" " + g(th)) if th else "", sym_uuid(ref)))
    props = ('\t\t(property "Reference" "%s" (at 0 -2.5 %s) (layer "%s") (uuid "%s") '
             '(effects (font (size 1 1) (thickness 0.15))%s))\n'
             '\t\t(property "Value" "%s" (at 0 2.5 %s) (layer "%s") (uuid "%s") '
             '(effects (font (size 1 1) (thickness 0.15))%s))\n'
             % (ref, g(th), L("F.SilkS"), nid(), " (justify mirror)" if back else "",
                value(oldref, f), g(th), L("F.Fab"), nid(), " (justify mirror)" if back else ""))
    if lib == "Impulse22":
        body = teensy_fp_body(f['pads'], padnets) if name == "TEENSY41" else buzzer_fp_body(padnets)
        attr = "\t\t(attr through_hole)\n"
        fps_out.append(hdr + props + attr + body + "\t)\n")
    else:
        body = load_mod(lib, name, back=back)
        # replace footprint name with lib-qualified
        body = re.sub(r'^\(footprint\s+"[^"]*"', '', body, count=1)
        body = body.rstrip()[:-1]  # drop trailing ')'
        # substitute REF**/value properties
        body = re.sub(r'\(property\s+"Reference"\s+"[^"]*"', '(property "Reference" "%s"' % ref, body, count=1)
        body = re.sub(r'\(property\s+"Value"\s+"[^"]*"', '(property "Value" "%s"' % value(ref, f), body, count=1)
        # rotate pad angles + inject nets
        out = []; last = 0
        for m in re.finditer(r'\(pad\s+"([^"]*)"', body):
            s0 = m.start(); e0 = bend(body, s0); pb = body[s0:e0]
            nn = m.group(1)
            if th:
                am = re.search(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)(?:\s+([\-\d.]+))?\)', pb)
                pa = (float(am.group(3)) if am.group(3) else 0) + th
                pb = pb[:am.start()] + '(at %s %s %s)' % (am.group(1), am.group(2), g(pa % 360)) + pb[am.end():]
            if padnets.get(nn) and 'np_thru_hole' not in pb:
                pb = pb[:-1] + ' (net "%s")' % padnets[nn] + ')'
            out.append(body[last:s0]); out.append(pb); last = e0
        out.append(body[last:])
        body = "".join(out)
        # give every element a uuid? KiCad tolerates missing; add for pads only via existing uuids in libs (none). fine.
        fps_out.append(hdr + props + body + "\n\t)\n")
    report.append((ref, fpid, th, round(wmax, 2)))

# ---------------- assemble board ----------------
old = open(OLD, encoding="utf-8").read()
i = old.find('(general'); setup_start = old.find('(setup')
layers_start = old.find('(layers'); layers_end = bend(old, layers_start)
setup_end = bend(old, setup_start)
general = old[i:bend(old, i)]
layers = old[layers_start:layers_end]
setup = old[setup_start:setup_end]
paper = re.search(r'\(paper "[^"]*"\)', old).group(0)

board = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n\t(generator_version "10.0")\n'
         '\t%s\n\t%s\n\t%s\n\t%s\n' % (general, paper, layers, setup))
board += "".join(fps_out)
board += ('\t(gr_circle (center 51.3446 51.5644) (end 86.7334 51.5644) (stroke (width 0.1) '
          '(type solid)) (fill no) (layer "Edge.Cuts") (uuid "%s"))\n' % nid())
board += ('\t(embedded_fonts no)\n)\n')
open(OUT, "w", encoding="utf-8").write(board)
json.dump(sym_uuids, open(SYM_UUID_FILE, "w"), indent=0)

print("wrote %s: %d footprints" % (OUT, len(fps_out)))
print("\nplacement report (ref, footprint, rot, worst pad offset mm):")
for r in report:
    flag = "  <-- CHECK" if r[3] > 1.5 else ""
    print("  %-18s %-55s rot=%-3s err=%.2f%s" % (*r, flag))
