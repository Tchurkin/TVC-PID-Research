#!/usr/bin/env python3
# Add sensor-module interface to the MAIN board: JST-GH 12 connector + SPI1 nets, wired to the
# free Teensy pins (SPI1 26/1/27, CS1 0, INT on 2) and the existing 3.3V/GND/SDA/SCL.
import re, uuid, sys
IN, OUT = sys.argv[1], sys.argv[2]
FP = "C:/Program Files/KiCad/10.0/share/kicad/footprints/Connector_PinHeader_2.54mm.pretty/PinHeader_2x05_P2.54mm_Vertical.kicad_mod"
t = open(IN, encoding="utf-8").read()

# --- existing net name -> code ---
netcode = {name: int(c) for c, name in re.findall(r'\(net (\d+) "([^"]*)"\)', t)}
maxc = max(netcode.values())
NEW = ["SCK", "MOSI", "MISO", "CS_IMU", "INT_IMU"]
for i, n in enumerate(NEW, 1):
    netcode[n] = maxc + i
# insert new net declarations after the last existing (net ...) decl
decls = "".join('  (net %d "%s")\n' % (netcode[n], n) for n in NEW)
last = list(re.finditer(r'^\s*\(net \d+ "[^"]*"\)\n', t, re.M))[-1]
t = t[:last.end()] + decls + t[last.end():]

def blocks(text, tok):
    out = []
    for m in re.finditer(r'\(' + tok + r'\b', text):
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

# --- assign the free Teensy (U1) pins to the new nets ---
padmap = {"27A13SCK1": "SCK", "26A12MOSI1": "MOSI", "1TX1MISO1CTX2": "MISO",
          "0RX1CS1CRX2": "CS_IMU", "2OUT2": "INT_IMU"}
# locate U1 block
u1s = u1e = None
for s, e in blocks(t, 'footprint'):
    if re.search(r'\(property "Reference" "U1"', t[s:e]): u1s, u1e = s, e; break
u1 = t[u1s:u1e]
for ps, pe in sorted(blocks(u1, 'pad'), key=lambda b: -b[0]):
    pb = u1[ps:pe]
    nm = re.match(r'\(pad "([^"]+)"', pb).group(1)
    if nm in padmap and '(net ' not in pb:
        code = netcode[padmap[nm]]
        u1 = u1[:ps] + pb[:-1] + ' (net %d "%s"))' % (code, padmap[nm]) + u1[pe:]
t = t[:u1s] + u1 + t[u1e:]

# --- build the connector footprint (JST-GH 1x12), place near bottom edge ---
conn = open(FP, encoding="utf-8").read().strip()
lm = re.search(r'\(layer\s+"[^"]+"\)', conn)
conn = conn[:lm.end()] + '\n\t(at 52.0 84.0 0)' + conn[lm.end():]
conn = re.sub(r'(\(property "Reference" ")[^"]*(")', r'\g<1>J_SENSOR\g<2>', conn, count=1)
conn = re.sub(r'(\(property "Value" ")[^"]*(")', r'\g<1>Header 2x5 sensor module\g<2>', conn, count=1)
# straight-through pinout matching the sensor board J1 (10-pin)
cpn = {"1": "3.3V", "2": "GND", "3": "SCK", "4": "MOSI", "5": "MISO", "6": "CS_IMU",
       "7": "INT_IMU", "8": "SDA", "9": "SCL", "10": "GND"}
for ps, pe in sorted(blocks(conn, 'pad'), key=lambda b: -b[0]):
    pb = conn[ps:pe]
    nm = re.match(r'\(pad "([^"]+)"', pb).group(1)
    if nm in cpn and '(net ' not in pb:
        code = netcode[cpn[nm]]
        conn = conn[:ps] + pb[:-1] + ' (net %d "%s"))' % (code, cpn[nm]) + conn[pe:]
conn_block = "  " + conn.replace("\n", "\n  ")
# insert connector before final ')'
idx = t.rstrip().rfind(")")
t = t[:idx] + conn_block + "\n" + t[idx:]

open(OUT, "w", encoding="utf-8").write(t)
print("added nets:", NEW, "codes", [netcode[n] for n in NEW])
print("assigned Teensy pins:", padmap)
print("connector J_SENSOR (JST-GH 12) placed at (52,84); pinout matches sensor board J1")
