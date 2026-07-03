#!/usr/bin/env python3
# Add the external-sensor-board 2x5 header (J_SENSOR) to the SCHEMATIC, matching the PCB footprint:
#   pinout 1:3.3V 2:GND 3:SCK 4:MOSI 5:MISO 6:CS_IMU 7:INT_IMU 8:SDA 9:SCL 10:GND
# Places it at the "External Sensor Board" spot, wires each pin to a global label, adds the
# Teensy-side labels for the 5 new SPI/sensor nets (pins 27/26/1/0/2), and links the PCB footprint.
import re, uuid
SCH, PCB = "Impulse_2.1.kicad_sch", "Impulse_2.1.kicad_pcb"
SHEET = "61f4f9be-db17-4eed-b67c-dfd3ce4de527"
s = open(SCH, encoding="utf-8").read()
def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
def nid(): return str(uuid.uuid4())
def fnum(v): return ('%.4f' % v).rstrip('0').rstrip('.')
def glabel(name, x, y, angle, justify):
    return ('\t(global_label "%s"\n\t\t(shape input)\n\t\t(at %s %s %d)\n'
            '\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.5748 1.5748)\n\t\t\t)\n\t\t\t(justify %s)\n\t\t)\n'
            '\t\t(uuid "%s")\n'
            '\t\t(property "Intersheetrefs" "${INTERSHEET_REFS}"\n\t\t\t(at %s %s 0)\n\t\t\t(hide yes)\n'
            '\t\t\t(show_name no)\n\t\t\t(do_not_autoplace no)\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n'
            ) % (name, fnum(x), fnum(y), angle, justify, nid(), fnum(x), fnum(y))
def wire(x1, y1, x2, y2):
    return ('\t(wire\n\t\t(pts\n\t\t\t(xy %s %s) (xy %s %s)\n\t\t)\n'
            '\t\t(stroke\n\t\t\t(width 0)\n\t\t\t(type default)\n\t\t)\n\t\t(uuid "%s")\n\t)\n'
            ) % (fnum(x1), fnum(y1), fnum(x2), fnum(y2), nid())

# ---- 1. new lib_symbol: 2x5 header -----------------------------------------
LIBNAME = "ProDoc_P1_-easyedapro:Conn_02x05_Sensor"
assert LIBNAME not in s, "lib symbol already present"
def libpin(num, x, y, ang):
    return ('\t\t\t(pin unspecified line\n\t\t\t\t(at %s %s %d)\n\t\t\t\t(length 2.54)\n'
            '\t\t\t\t(name "%s"\n\t\t\t\t\t(effects\n\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t(size 1.27 1.27)\n'
            '\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t)\n'
            '\t\t\t\t(number "%s"\n\t\t\t\t\t(effects\n\t\t\t\t\t\t(font\n\t\t\t\t\t\t\t(size 1.27 1.27)\n'
            '\t\t\t\t\t\t)\n\t\t\t\t\t)\n\t\t\t\t)\n\t\t\t)\n') % (fnum(x), fnum(y), ang, num, num)
LEFT  = [("1", 5.08), ("3", 2.54), ("5", 0.0), ("7", -2.54), ("9", -5.08)]   # odd, left
RIGHT = [("2", 5.08), ("4", 2.54), ("6", 0.0), ("8", -2.54), ("10", -5.08)]  # even, right
pins = "".join(libpin(n, -5.08, y, 0) for n, y in LEFT) + \
       "".join(libpin(n,  5.08, y, 180) for n, y in RIGHT)
def prop(k, v, x, y, hide, jl=False):
    h = "\t\t\t(hide yes)\n" if hide else ""
    j = "\t\t\t\t(justify left bottom)\n" if jl else ""
    return ('\t\t(property "%s" "%s"\n\t\t\t(at %s %s 0)\n%s'
            '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n%s\t\t\t)\n\t\t)\n'
            ) % (k, v, fnum(x), fnum(y), h, j)
LIBSYM = ('\t(symbol "%s"\n\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n'
          % LIBNAME
          + prop("Reference", "J", 0, 8.89, False)
          + prop("Value", "Conn_02x05", 0, -8.89, False)
          + prop("Footprint", "PinHeader_2x05_P2.54mm_Vertical", 0, 0, True)
          + prop("Datasheet", "", 0, 0, True)
          + prop("Description", "External sensor board 2x5 header", 0, 0, True)
          + '\t\t(symbol "Conn_02x05_Sensor_1_0"\n'
          + '\t\t\t(rectangle\n\t\t\t\t(start -2.54 6.35)\n\t\t\t\t(end 2.54 -6.35)\n'
            '\t\t\t\t(stroke\n\t\t\t\t\t(width 0.254)\n\t\t\t\t\t(type solid)\n\t\t\t\t)\n'
            '\t\t\t\t(fill\n\t\t\t\t\t(type none)\n\t\t\t\t)\n\t\t\t)\n'
          + pins + '\t\t)\n\t)\n')
li = s.find('(lib_symbols'); le = bend(s, li)
s = s[:le - 1] + LIBSYM + s[le - 1:]

# ---- 2. J_SENSOR instance at the External Sensor Board spot ------------------
OX, OY = 63.5, 425.0
SYMU = nid()
def iprop(k, v, x, y, hide, jlb=False):
    h = "\t\t\t(hide yes)\n" if hide else ""
    fx = "\t\t\t\t(justify left bottom)\n" if jlb else ""
    sz = "1.2446 1.2446" if not hide else "1.27 1.27"
    return ('\t\t(property "%s" "%s"\n\t\t\t(at %s %s 0)\n%s'
            '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size %s)\n\t\t\t\t)\n%s\t\t\t)\n\t\t)\n'
            ) % (k, v, fnum(x), fnum(y), h, sz, fx)
inst = ('\t(symbol\n\t\t(lib_id "%s")\n\t\t(at %s %s 0)\n\t\t(unit 1)\n\t\t(body_style 0)\n'
        '\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n\t\t(in_pos_files yes)\n\t\t(dnp no)\n'
        '\t\t(uuid "%s")\n' % (LIBNAME, fnum(OX), fnum(OY), SYMU)
        + iprop("Reference", "J_SENSOR", OX + 7.5, OY - 8.89, False, True)
        + iprop("Value", "Conn_02x05", OX + 7.5, OY + 9.0, False, True)
        + iprop("Footprint", "PinHeader_2x05_P2.54mm_Vertical", OX, OY, True)
        + iprop("Datasheet", "", OX, OY, True)
        + iprop("Description", "", OX, OY, True)
        + "".join('\t\t(pin "%s"\n\t\t\t(uuid "%s")\n\t\t)\n' % (n, nid())
                  for n, _ in LEFT + RIGHT)
        + '\t\t(instances\n\t\t\t(project "ProDoc_P1_2026-06-30"\n\t\t\t\t(path "/%s"\n'
          '\t\t\t\t\t(reference "J_SENSOR")\n\t\t\t\t\t(unit 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n' % SHEET)

# pin nets (matches PCB J_SENSOR pads)
NET = {"1": "3.3V", "2": "GND", "3": "SCK", "4": "MOSI", "5": "MISO",
       "6": "CS_IMU", "7": "INT_IMU", "8": "SDA", "9": "SCL", "10": "GND"}
extra = inst
for n, y in LEFT:                       # left tips at OX-5.08, wire out to OX-15, label
    ty = OY - y
    extra += wire(OX - 5.08, ty, OX - 15.08, ty) + glabel(NET[n], OX - 15.08, ty, 180, "right")
for n, y in RIGHT:                      # right tips at OX+5.08, wire out to OX+15
    ty = OY - y
    extra += wire(OX + 5.08, ty, OX + 15.08, ty) + glabel(NET[n], OX + 15.08, ty, 0, "left")

# ---- 3. Teensy-side labels for the 5 new SPI/sensor nets --------------------
extra += glabel("SCK", 246.38, 271.78, 180, "right")     # pin 27 (left)
extra += glabel("MOSI", 246.38, 269.24, 180, "right")    # pin 26 (left)
extra += glabel("MISO", 302.26, 256.54, 0, "left")       # pin 1  (right)
extra += glabel("CS_IMU", 302.26, 254.00, 0, "left")     # pin 0  (right)
extra += glabel("INT_IMU", 302.26, 304.80, 0, "left")    # pin 2  (right)

pos = s.rfind('\t(embedded_fonts no)')
assert pos != -1 and s[pos:].strip() == '(embedded_fonts no)\n)', "unexpected tail"
s = s[:pos] + extra + s[pos:]
open(SCH, "w", encoding="utf-8").write(s)

# ---- 4. link the PCB J_SENSOR footprint to the schematic symbol -------------
p = open(PCB, encoding="utf-8").read()
fi = p.find('(footprint ')
while fi != -1:
    e = bend(p, fi)
    if '(property "Reference" "J_SENSOR"' in p[fi:e]:
        atm = re.search(r'\n([ \t]*)\(at [\-\d. ]+\)\n', p[fi:e])
        ins = fi + atm.end()
        assert '(path "' not in p[fi:ins]
        p = p[:ins] + '%s(path "/%s/%s")\n' % (atm.group(1), SHEET, SYMU) + p[ins:]
        break
    fi = p.find('(footprint ', e)
else:
    raise SystemExit("J_SENSOR footprint not found in PCB")
open(PCB, "w", encoding="utf-8").write(p)
print("added lib_symbol %s" % LIBNAME)
print("placed J_SENSOR at (%.1f,%.1f), uuid %s" % (OX, OY, SYMU))
print("wired pins:", NET)
print("added Teensy-side labels: SCK/MOSI/MISO/CS_IMU/INT_IMU")
print("linked PCB J_SENSOR -> schematic symbol")
