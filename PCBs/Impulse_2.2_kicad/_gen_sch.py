#!/usr/bin/env python3
# Impulse 2.2 schematic: native KiCad symbols, one global label per connected pin (net names taken
# from the generated PCB, so sch == pcb by construction), no_connect on unused pins, PWR_FLAGs.
# Also writes the .kicad_pro, lib tables, and the Impulse22 custom footprint/symbol libraries.
import re, json, uuid, os

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

# ---------- read back the PCB: ref -> pads/nets/footprint/value ----------
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

# ---------- symbol selection ----------
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

# ---------- native symbol extraction (resolving 'extends') ----------
_libcache = {}
def libtext(lib):
    if lib not in _libcache:
        fn = "power" if lib == "power" else lib
        _libcache[lib] = open("%s/%s.kicad_sym" % (SYMD, fn), encoding="utf-8").read()
    return _libcache[lib]
def raw_symbol(lib, name):
    t = libtext(lib)
    m = re.search(r'\(symbol "%s"' % re.escape(name), t)
    assert m, "symbol %s missing in %s" % (name, lib)
    return t[m.start():bend(t, m.start())]

embedded = {}    # "Lib:Name" -> block text
pins_of = {}     # "Lib:Name" -> [(number, name, x, y, ang)]
def embed(lib, name):
    key = "%s:%s" % (lib, name)
    if key in embedded: return key
    blk = raw_symbol(lib, name)
    ext = re.search(r'\(extends "([^"]+)"\)', blk)
    src = blk
    if ext:
        # FLATTEN: use the parent's full definition (graphics+pins) under this symbol's name;
        # embedded derived symbols would need a resolvable parent, flattening avoids that.
        parent = ext.group(1)
        src = raw_symbol(lib, parent)
        src = src.replace('(symbol "%s_' % parent, '(symbol "%s_' % name)
        src = src.replace('(symbol "%s"' % parent, '(symbol "%s"' % name, 1)
        # keep the derived symbol's own Value/Reference names by leaving parent props; instance
        # properties override anyway.
        blk = src
    pins = []
    for pm in re.finditer(r'\(pin\s+\w+\s+\w+', blk):
        pb = blk[pm.start():bend(blk, pm.start())]
        at = re.search(r'\(at\s+([\-\d.]+)\s+([\-\d.]+)(?:\s+([\-\d.]+))?\)', pb)
        nm = re.search(r'\(name\s+"([^"]*)"', pb)
        num = re.search(r'\(number\s+"([^"]*)"', pb)
        pins.append((num.group(1), nm.group(1), float(at.group(1)), float(at.group(2)),
                     float(at.group(3) or 0)))
    pins_of[key] = pins
    src = src.replace('(symbol "%s"' % name, '(symbol "%s"' % key, 1)
    embedded[key] = src
    return key

# ---------- custom TEENSY41 symbol ----------
def teensy_symbol():
    left = ["GND1"] + [str(i) for i in range(13)] + ["3V3_1"] + [str(i) for i in range(24, 33)]
    right = ["VIN", "GND3", "3V3_2"] + [str(i) for i in range(23, 12, -1)] + ["GND2"] + \
            [str(i) for i in range(41, 32, -1)]
    assert len(left) == 24 and len(right) == 24
    pins = []
    s = ('    (symbol "Impulse22:TEENSY41"\n      (pin_names (offset 1.016)) (exclude_from_sim no) '
         '(in_bom yes) (on_board yes)\n'
         '      (property "Reference" "U" (at 0 33 0) (effects (font (size 1.27 1.27))))\n'
         '      (property "Value" "Teensy 4.1" (at 0 -33 0) (effects (font (size 1.27 1.27))))\n'
         '      (symbol "TEENSY41_1_1"\n'
         '        (rectangle (start -12.7 31.75) (end 12.7 -31.75) '
         '(stroke (width 0.254) (type solid)) (fill (type background)))\n')
    def pin(num, x, y, ang):
        pins.append((num, num, x, y, ang))
        return ('        (pin bidirectional line (at %s %s %d) (length 5.08)\n'
                '          (name "%s" (effects (font (size 1.27 1.27))))\n'
                '          (number "%s" (effects (font (size 1.27 1.27)))))\n'
                % (g(x), g(y), ang, num, num))
    for i, n in enumerate(left):
        s += pin(n, -17.78, 29.21 - i * 2.54, 0)
    for i, n in enumerate(right):
        s += pin(n, 17.78, 29.21 - i * 2.54, 180)
    s += "      )\n    )\n"
    pins_of["Impulse22:TEENSY41"] = pins
    embedded["Impulse22:TEENSY41"] = s
    return s

teensy_symbol()

# ---------- sanity checks on pin-number -> function ----------
embed("Device", "LED_ABRG")
abrg = {n: nm for n, nm, *_ in pins_of["Device:LED_ABRG"]}
assert "A" in abrg.get("1", "") and "B" in abrg.get("2", "") and "R" in abrg.get("3", "") \
       and "G" in abrg.get("4", ""), "LED_ABRG pin order unexpected: %s" % abrg
embed("Transistor_FET", "IRLML6244")
fet = {n: nm for n, nm, *_ in pins_of["Transistor_FET:IRLML6244"]}
assert fet == {"1": "G", "2": "S", "3": "D"}, "FET pins: %s" % fet
embed("Regulator_Switching", "LM2594M-ADJ")
reg = {n: nm for n, nm, *_ in pins_of["Regulator_Switching:LM2594M-ADJ"]}
print("LM2594M-ADJ pins:", reg)
embed("Device", "Buzzer")
buz = {n: nm for n, nm, *_ in pins_of["Device:Buzzer"]}
assert buz.get("1") == "+" and buz.get("2") == "-", "Buzzer pins: %s" % buz

# ---------- layout ----------
ORDER = ["SW1", "BAT_MAIN1", "U4", "L4", "D10", "C38",
         "C22", "R8", "R9", "LM2594_enable1", "logic_power_reg1", "D8",
         "C33", "C39", "U8", "servo_power_reg1", "D9", "C43",
         "DROK_exclusive1", "power_tap1", "PYRO_ON1", "PYRO1", "P1+P2", "P3+P4",
         "Q16", "R50", "R37", "P1", "P1O1", "AUX_3V3_1",
         "Q15", "R49", "R43", "P2", "P2O1", "OUT1",
         "Q14", "R48", "R44", "P3", "P3O1", "BUTTON1",
         "Q13", "R47", "R45", "P4", "P4O1", "R23",
         "X1", "Y1", "SERVO1", "SERVO2", "SERVO3", "SERVO4",
         "J_SENSOR1", "U6", "U7", "BUZZER1", "LED1", "R38",
         "R39", "R40", "H1", "H2", "H3", "H4"]
assert set(ORDER) | {"U1"} == set(parts), (set(parts) - set(ORDER) - {"U1"}, set(ORDER) - set(parts))
POS = {}
for i, ref in enumerate(ORDER):
    col, row = divmod(i, 6)
    POS[ref] = (40 + col * 42, 50 + row * 55)
POS["U1"] = (510, 200)

# ---------- emit ----------
body = ""
labels = ""
def glabel(netname, x, y, ang, just):
    return ('  (global_label "%s" (shape input) (at %s %s %d) '
            '(effects (font (size 1.27 1.27)) (justify %s))\n    (uuid %s)\n'
            '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) (hide yes) '
            '(effects (font (size 1.27 1.27))))\n  )\n'
            % (netname, g(x), g(y), ang, just, nid(), g(x), g(y)))
LBL = {0: (180, "right"), 180: (0, "left"), 90: (270, "left"), 270: (90, "left")}

for ref in list(ORDER) + ["U1"]:
    pt = parts[ref]
    lib, name = symbol_for(ref)
    key = "Impulse22:TEENSY41" if lib == "CUSTOM" else embed(lib, name)
    ix, iy = POS[ref]
    pins = pins_of[key]
    body += ('  (symbol (lib_id "%s") (at %s %s 0) (unit 1) (exclude_from_sim no) (in_bom %s) '
             '(on_board yes) (dnp no)\n    (uuid %s)\n'
             % (key, g(ix), g(iy), "no" if ref.startswith("H") else "yes", sym_uuids[ref]))
    body += ('    (property "Reference" "%s" (at %s %s 0) (effects (font (size 1.27 1.27))))\n'
             '    (property "Value" "%s" (at %s %s 0) (effects (font (size 1.27 1.27))))\n'
             '    (property "Footprint" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
             % (ref, g(ix + 3), g(iy - 6), pt['val'].replace('"', ''), g(ix + 3), g(iy + 6),
                pt['fp'], g(ix), g(iy)))
    for (num, _nm, px, py, ang) in pins:
        body += '    (pin "%s" (uuid %s))\n' % (num, nid())
    body += ('    (instances (project "Impulse_2.2" (path "/%s" (reference "%s") (unit 1))))\n  )\n'
             % (ROOT, ref))
    seen = set()
    for (num, _nm, px, py, ang) in pins:
        if num in seen: continue
        seen.add(num)
        ax, ay = ix + px, iy - py
        netname = pt['pads'].get(num)
        if netname:
            la, ju = LBL[int(ang) % 360]
            labels += glabel(netname, ax, ay, la, ju)
        elif not ref.startswith("H"):
            labels += '  (no_connect (at %s %s) (uuid %s))\n' % (g(ax), g(ay), nid())

# PWR_FLAGs on power nets
embed("power", "PWR_FLAG")
pfpins = pins_of["power:PWR_FLAG"]
PWR = ["GND", "3.3V", "5V-CLEAN", "5V-DIRTY", "7.4V", "VBAT_MAIN", "VBAT_SW",
       "BUCK_5V", "LOGIC_5V_IN", "SERVO_5V_IN"]
for i, netname in enumerate(PWR):
    ix, iy = 60 + i * 30, 400
    u = nid()
    body += ('  (symbol (lib_id "power:PWR_FLAG") (at %s %s 0) (unit 1) (exclude_from_sim no) '
             '(in_bom no) (on_board yes) (dnp no)\n    (uuid %s)\n'
             '    (property "Reference" "#FLG%02d" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
             '    (property "Value" "PWR_FLAG" (at %s %s 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
             % (g(ix), g(iy), u, i + 1, g(ix), g(iy - 4), g(ix), g(iy - 6)))
    for (num, _nm, px, py, ang) in pfpins:
        body += '    (pin "%s" (uuid %s))\n' % (num, nid())
    body += ('    (instances (project "Impulse_2.2" (path "/%s" (reference "#FLG%02d") (unit 1))))\n  )\n'
             % (ROOT, i + 1))
    (num, _nm, px, py, ang) = pfpins[0]
    labels += glabel(netname, ix + px, iy - py, 270, "left")

sch = ('(kicad_sch\n  (version 20231120)\n  (generator "eeschema")\n  (generator_version "8.0")\n'
       '  (uuid %s)\n  (paper "A2")\n  (lib_symbols\n%s  )\n%s%s  (sheet_instances (path "/" (page "1")))\n)\n'
       % (ROOT, "".join("    " + e.strip() + "\n" for e in embedded.values()), body, labels))
open(OUT, "w", encoding="utf-8").write(sch)
print("wrote %s: %d symbols, %d embedded defs" % (OUT, len(parts) + len(PWR), len(embedded)))

# ---------- project + lib tables + custom libs ----------
pro = open("../Sensor_Board_kicad/Sensor_Board.kicad_pro", encoding="utf-8").read()
open("Impulse_2.2.kicad_pro", "w", encoding="utf-8").write(pro.replace("Sensor_Board", "Impulse_2.2"))
open("sym-lib-table", "w", encoding="utf-8").write(
    '(sym_lib_table\n  (version 7)\n  (lib (name "Impulse22")(type "KiCad")'
    '(uri "${KIPRJMOD}/Impulse22.kicad_sym")(options "")(descr "Impulse project symbols"))\n)\n')
open("fp-lib-table", "w", encoding="utf-8").write(
    '(fp_lib_table\n  (version 7)\n  (lib (name "Impulse22")(type "KiCad")'
    '(uri "${KIPRJMOD}/Impulse22.pretty")(options "")(descr "Impulse project footprints"))\n)\n')
open("Impulse22.kicad_sym", "w", encoding="utf-8").write(
    '(kicad_symbol_lib (version 20231120) (generator "impulse_gen")\n' +
    embedded["Impulse22:TEENSY41"].replace('Impulse22:TEENSY41', 'TEENSY41') + ')\n')
# custom footprints -> .pretty (strip nets/path, zero position)
for ref, fname in [("U1", "TEENSY41"), ("BUZZER", "Buzzer_D9.0mm_P4.00mm")]:
    m = re.search(r'\(footprint "Impulse22:%s"' % re.escape(fname), p)
    blk = p[m.start():bend(p, m.start())]
    blk = re.sub(r'\s*\(net "[^"]*"\)', '', blk)
    blk = re.sub(r'\s*\(path "[^"]*"\)', '', blk)
    blk = re.sub(r'\(at [\-\d. ]+\)', '(at 0 0)', blk, count=1)
    blk = blk.replace('(footprint "Impulse22:%s"' % fname, '(footprint "%s"' % fname, 1)
    open("Impulse22.pretty/%s.kicad_mod" % fname, "w", encoding="utf-8").write(blk + "\n")
print("wrote project, lib tables, Impulse22 custom libs")
