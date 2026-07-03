#!/usr/bin/env python3
# Make forward-annotation (Update PCB from Schematic) run clean on the EasyEDA-imported design:
#  PCB:  - rename the 48 U1/TEENSY41 footprint pads to match the symbol pin numbers
#          (footprint pads had punctuation stripped: "0RX1CS1CRX2" -> symbol "0-RX1/CS1/CRX2")
#        - give J_SENSOR footprint a proper Library:Name id
#  SCH:  - prune the 19 Teensy symbol pins that this 48-pad header footprint doesn't break out
#          (14 unused USB/SD/PROGRAM/etc + 5 REDUNDANT power/gnd whose nets reach U1 via other pads)
#        - remove their now-dangling GND flags (GND2/4/5) and labels (+5V=5V-CLEAN, 3V3-2=3.3V)
#        - mark the A4 drawing frame not-on-board so it isn't annotated as component "1"
#        - match J_SENSOR symbol Footprint field to the PCB
import re
BS = chr(92)
SCH, PCB = "Impulse_2.1.kicad_sch", "Impulse_2.1.kicad_pcb"

def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
def strip(x):
    for ch in ('-', '/', '_', BS): x = x.replace(ch, '')
    return x

# ================= PCB =================
p = open(PCB, encoding="utf-8").read()
s = open(SCH, encoding="utf-8").read()
# symbol pin numbers (canonical names)
li = s.find('(symbol "TEENSY41_1_0"'); sblk0 = s[li:bend(s, li)]
snums = re.findall(r'\(number "([^"]+)"', sblk0)
strip2num = {strip(n): n for n in snums}

# --- rename U1 footprint pads ---
k = p.find('"Reference" "U1"'); ust = p.rfind('(footprint', 0, k); uend = bend(p, ust)
ublk = p[ust:uend]
renamed = 0
def rename_pad(m):
    global renamed
    name = m.group(1)
    if name in strip2num and strip2num[name] != name:
        renamed += 1
        return '(pad "%s"' % strip2num[name]
    return m.group(0)
ublk_new = re.sub(r'\(pad "([^"]+)"', rename_pad, ublk)
newpads = re.findall(r'\(pad "([^"]+)"', ublk_new)
assert len(newpads) == 48 and all(pd in snums for pd in newpads) and len(set(newpads)) == 48, \
    "pad->pin bijection broken after rename"
assert renamed == 44, "expected 44 renames (4 already matched), got %d" % renamed
p = p[:ust] + ublk_new + p[uend:]

# --- J_SENSOR footprint proper library id ---
assert p.count('(footprint "PinHeader_2x05_P2.54mm_Vertical"') == 1
p = p.replace('(footprint "PinHeader_2x05_P2.54mm_Vertical"',
              '(footprint "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical"', 1)
open(PCB, "w", encoding="utf-8").write(p)

# ================= SCHEMATIC =================
REMOVE = ['+5V', '3V3-2', 'D+', 'D-', 'DM', 'DP', 'G', 'GND2', 'GND4', 'GND5',
          'LED', 'ON/OFF', 'PROGRAM', 'R+', 'R-', 'T+', 'T-', 'VBAT', 'VUSB']

# --- (1) prune pins from the TEENSY41_1_0 lib symbol ---
li = s.find('(symbol "TEENSY41_1_0"'); le = bend(s, li)
lib = s[li:le]
removed = 0
for num in REMOVE:
    # each pin block ends with (number "NUM" ... ); find the (pin ... that contains it
    pat = re.compile(r'\(pin\b')
    idx = 0; found = False
    for m in pat.finditer(lib):
        b = lib[m.start():bend(lib, m.start())]
        nm = re.search(r'\(number "([^"]+)"', b)
        if nm and nm.group(1) == num:
            lib = lib[:m.start()] + lib[m.start() + len(b):]
            removed += 1; found = True; break
    assert found, "lib pin %s not found" % num
# tidy blank lines
lib = re.sub(r'\n[ \t]*\n', '\n', lib)
assert removed == 19
s = s[:li] + lib + s[le:]

# --- (2) remove the matching (pin "NUM" (uuid..)) entries from the U1 instance ---
k = s.find('(lib_id "ProDoc_P1_-easyedapro:TEENSY41")'); ist = s.rfind('(symbol', 0, k); ie = bend(s, ist)
inst = s[ist:ie]
for num in REMOVE:
    inst2 = re.sub(r'\t*\(pin "%s"\s*\n\s*\(uuid "[0-9a-f\-]+"\)\s*\n\s*\)\n' % re.escape(num), '', inst, count=1)
    assert inst2 != inst, "instance pin %s not found" % num
    inst = inst2
s = s[:ist] + inst + s[ie:]

# --- helper: remove the enclosing block of type `tok` whose main (at x y ..) matches ---
def remove_block_at(text, tok, x, y, libmatch=None):
    for m in re.finditer(r'\(' + tok + r'\b', text):
        b = text[m.start():bend(text, m.start())]
        if libmatch and libmatch not in b: continue
        am = re.search(r'\(at (-?\d+\.?\d*) (-?\d+\.?\d*)', b)
        if am and abs(float(am.group(1)) - x) < 0.05 and abs(float(am.group(2)) - y) < 0.05:
            return text[:m.start()] + text[m.start() + len(b):], True
    return text, False

# --- (3) remove dangling GND flags for GND2/4/5 ---
for (x, y) in [(269.24, 320.04), (274.32, 320.04), (276.86, 320.04)]:
    s, ok = remove_block_at(s, 'symbol', x, y, libmatch='Ground-GND')
    assert ok, "GND flag @(%s,%s) not found" % (x, y)

# --- (4) remove dangling labels for +5V (5V-CLEAN) and 3V3-2 (3.3V) ---
for tok, x, y in [('global_label', 302.26, 233.68), ('label', 302.26, 233.68),
                  ('global_label', 284.48, 228.6), ('label', 284.48, 228.6)]:
    s, _ = remove_block_at(s, tok, x, y)
# verify both target points are now clear of labels
for x, y in [(302.26, 233.68), (284.48, 228.6)]:
    assert not re.search(r'\((?:global_label|label) "[^"]+"\s*\n\s*(?:\(shape \w+\)\s*\n\s*)?\(at %s %s' %
                         (x, y), s), "label @(%s,%s) still present" % (x, y)

# --- (5) A4 drawing frame: not on board / not in BOM (stop it annotating as "1") ---
k = s.find('(lib_id "ProDoc_P1_-easyedapro:Drawing-Symbol_A4")'); fst = s.rfind('(symbol', 0, k); fe = bend(s, fst)
fr = s[fst:fe]
fr = fr.replace('(in_bom yes)', '(in_bom no)', 1).replace('(on_board yes)', '(on_board no)', 1)
s = s[:fst] + fr + s[fe:]

# --- (6) J_SENSOR symbol Footprint field -> full library id ---
s = s.replace('"Footprint" "PinHeader_2x05_P2.54mm_Vertical"',
              '"Footprint" "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical"')
open(SCH, "w", encoding="utf-8").write(s)
print("PCB: renamed %d U1 pads, fixed J_SENSOR footprint id" % renamed)
print("SCH: pruned %d Teensy pins + instance entries, removed 3 GND flags + 2 labels" % removed)
print("SCH: A4 frame set off-board; J_SENSOR footprint field fixed")
