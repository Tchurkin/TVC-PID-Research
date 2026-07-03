#!/usr/bin/env python3
# Generate a KiCad sensor-board PCB scaffold: real KiCad-lib footprints, placed on a round
# grommet-mounted board, with the netlist baked into pads (route from ratsnest).
import re, uuid, os

KF = "C:/Program Files/KiCad/10.0/share/kicad/footprints"
FP = {
    "IMU":   f"{KF}/Package_LGA.pretty/LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y.kicad_mod",
    "BARO":  f"{KF}/Package_LGA.pretty/Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering.kicad_mod",
    "CONN":  f"{KF}/Connector_PinHeader_2.54mm.pretty/PinHeader_2x05_P2.54mm_Vertical.kicad_mod",
    "C0402": f"{KF}/Capacitor_SMD.pretty/C_0402_1005Metric.kicad_mod",
    "C0805": f"{KF}/Capacitor_SMD.pretty/C_0805_2012Metric.kicad_mod",
    "R0402": f"{KF}/Resistor_SMD.pretty/R_0402_1005Metric.kicad_mod",
    "MHOLE": f"{KF}/MountingHole.pretty/MountingHole_3.2mm_M3.kicad_mod",
}
NETS = ["", "GND", "+3V3", "+3V3_RAW", "SCK", "MOSI", "MISO", "CS_IMU", "INT_IMU", "SDA", "SCL"]
NN = {n: i for i, n in enumerate(NETS)}
LIBQ = {"IMU": "Package_LGA", "BARO": "Package_LGA", "CONN": "Connector_PinHeader_2.54mm",
        "C0402": "Capacitor_SMD", "C0805": "Capacitor_SMD", "R0402": "Resistor_SMD", "MHOLE": "MountingHole"}

def read_fp(key):
    t = open(FP[key], encoding="utf-8").read().strip()
    return re.sub(r'(\(footprint\s+")', r'\g<1>' + LIBQ[key] + ':', t, count=1)  # lib-qualify FPID

def pad_blocks(text):
    out = []
    for m in re.finditer(r'\(pad\b', text):
        i = m.start(); d = 0; j = i
        while j < len(text):
            c = text[j]
            if c == '(': d += 1
            elif c == ')':
                d -= 1
                if d == 0: break
            j += 1
        out.append((i, j + 1))
    return out

def place(key, ref, value, x, y, rot, padnet):
    t = read_fp(key)
    # set footprint placement right after the FIRST (layer "...") (= footprint layer, before pads).
    lm = re.search(r'\(layer\s+"[^"]+"\)', t)
    pos = lm.end()
    atstr = '\n\t(at %.3f %.3f %d)' % (x, y, rot)
    if re.match(r'\s*\(at [0-9.\- ]+\)', t[pos:pos + 60]):      # existing fp-level (at) -> replace
        t = t[:pos] + re.sub(r'\s*\(at [0-9.\- ]+\)', atstr, t[pos:], count=1)
    else:                                                        # none -> insert
        t = t[:pos] + atstr + t[pos:]
    # reference + value
    t = re.sub(r'(\(property "Reference" ")[^"]*(")', r'\g<1>%s\g<2>' % ref, t, count=1)
    t = re.sub(r'(\(property "Value" ")[^"]*(")', r'\g<1>%s\g<2>' % value, t, count=1)
    # add uuid to footprint if missing
    if not re.search(r'\(footprint[^\n]*\n\s*\(', t):
        pass
    # inject nets into pads (by pad number)
    blocks = pad_blocks(t)
    for s, e in sorted(blocks, key=lambda b: -b[0]):
        blk = t[s:e]
        pm = re.match(r'\(pad\s+"([^"]+)"', blk)
        if not pm: continue
        num = pm.group(1)
        if num in padnet and '(net ' not in blk:
            code = padnet[num]
            newblk = blk[:-1] + ' (net %d "%s"))' % (code, NETS[code])
            t = t[:s] + newblk + t[e:]
    return "  " + t.replace("\n", "\n  ")

# ---- component instances: (key, ref, value, x, y, rot, {pad#: netcode}) ----
comps = [
 # tight functional cluster (center ~25,24): ICs in a row, decoupling caps above,
 # power row (ferrite+bulk) below, CS pull-up beside the IMU.
 ("IMU","U1","ICM-42688-P",23,23.5,0, {"1":NN["INT_IMU"],"5":NN["GND"],"9":NN["CS_IMU"],
    "10":NN["SCK"],"11":NN["MISO"],"12":NN["MOSI"],"13":NN["+3V3"],"14":NN["+3V3"]}),
 ("BARO","U2","DPS310",28,23.5,0, {"1":NN["GND"],"2":NN["+3V3"],"3":NN["SDA"],"4":NN["SCL"],
    "5":NN["GND"],"6":NN["+3V3"],"7":NN["GND"],"8":NN["+3V3"]}),
 ("C0402","C1","100nF",22.5,20.5,0, {"1":NN["+3V3"],"2":NN["GND"]}),
 ("C0402","C2","10nF",25,20.5,0, {"1":NN["+3V3"],"2":NN["GND"]}),
 ("C0402","C3","100nF",27.5,20.5,0, {"1":NN["+3V3"],"2":NN["GND"]}),
 ("C0402","C4","100nF",30,20.5,0, {"1":NN["+3V3"],"2":NN["GND"]}),
 ("R0402","R1","10k",20,23.5,90, {"1":NN["CS_IMU"],"2":NN["+3V3"]}),
 ("R0402","FB1","FerriteBead",23,27,0, {"1":NN["+3V3_RAW"],"2":NN["+3V3"]}),
 ("C0805","C5","10uF",27.5,27,0, {"1":NN["+3V3"],"2":NN["GND"]}),
 ("CONN","J1","Header 2x5 to main",25,37,90, {"1":NN["+3V3_RAW"],"2":NN["GND"],"3":NN["SCK"],
    "4":NN["MOSI"],"5":NN["MISO"],"6":NN["CS_IMU"],"7":NN["INT_IMU"],"8":NN["SDA"],
    "9":NN["SCL"],"10":NN["GND"]}),
 # 4 mounting holes in a SQUARE (M3, for grommets), center (25,25)
 ("MHOLE","H1","M3",14.5,14.5,0, {}),
 ("MHOLE","H2","M3",35.5,14.5,0, {}),
 ("MHOLE","H3","M3",35.5,35.5,0, {}),
 ("MHOLE","H4","M3",14.5,35.5,0, {}),
]

fps = [place(*c) for c in comps]

# board outline circle + net decls
CX, CY, R = 25.0, 25.0, 18.0
netdecls = "\n".join('  (net %d "%s")' % (i, n) for i, n in enumerate(NETS))
circle = ('  (gr_circle (center %.3f %.3f) (end %.3f %.3f) (stroke (width 0.1) (type solid)) '
          '(fill no) (layer "Edge.Cuts") (uuid "%s"))' % (CX, CY, CX + R, CY, uuid.uuid4()))

LAYERS = '''  (layers
    (0 "F.Cu" signal) (31 "B.Cu" signal)
    (34 "B.Paste" user) (35 "F.Paste" user)
    (36 "B.SilkS" user) (37 "F.SilkS" user)
    (38 "B.Mask" user) (39 "F.Mask" user)
    (40 "Dwgs.User" user) (44 "Edge.Cuts" user)
    (46 "B.CrtYd" user) (47 "F.CrtYd" user)
    (48 "B.Fab" user) (49 "F.Fab" user)
  )'''

OUT = "sensor_board.kicad_pcb"
with open(OUT, "w", encoding="utf-8") as f:
    f.write('(kicad_pcb\n  (version 20240108)\n  (generator "sensor_board_gen")\n')
    f.write('  (general (thickness 1.6))\n  (paper "A4")\n')
    f.write(LAYERS + "\n  (setup (pad_to_mask_clearance 0))\n")
    f.write(netdecls + "\n")
    f.write("\n".join(fps) + "\n")
    f.write(circle + "\n)\n")
print("wrote", OUT, "with", len(comps), "components,", len(NETS), "nets")
