#!/usr/bin/env python3
# Add the four fin servos to the MAIN (Impulse) board:
#   - rename existing SERVOZ header -> SERVO1 (fin servo 1)
#   - add SERVO2, SERVO3, SERVO4 (new 3-pin headers: signal / 5V-DIRTY / GND)
#   - wire the three new signals to free Teensy 4.1 PWM pins 28, 29, 25
# PCB-only, same approach as _add_sensor_interface.py (schematic is left untouched).
import re, sys, uuid

IN, OUT = sys.argv[1], sys.argv[2]
t = open(IN, encoding="utf-8").read()

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

# --- 1. rename SERVOZ -> SERVO1 (net string everywhere) + connector Reference "Z" -> "SERVO1"
assert t.count('"SERVOZ"') >= 2, "expected SERVOZ net decl + pad refs"
t = t.replace('"SERVOZ"', '"SERVO1"')
assert '(property "Reference" "Z" (at' in t, "Z servo connector ref not found"
t = t.replace('(property "Reference" "Z" (at', '(property "Reference" "SERVO1" (at', 1)

# --- 2. new nets after the last existing (net N ...) decl
netcode = {name: int(c) for c, name in re.findall(r'\(net (\d+) "([^"]*)"\)', t)}
maxc = max(netcode.values())
NEW = ["SERVO2", "SERVO3", "SERVO4"]
for i, n in enumerate(NEW, 1):
    netcode[n] = maxc + i
decls = "".join('  (net %d "%s")\n' % (netcode[n], n) for n in NEW)
last = list(re.finditer(r'^\s*\(net \d+ "[^"]*"\)\n', t, re.M))[-1]
t = t[:last.end()] + decls + t[last.end():]

# --- 3. assign free Teensy (U1) PWM pins to the new servo signals
padmap = {"28RX7": "SERVO2", "29TX7": "SERVO3", "25A11RX6SDA2": "SERVO4"}
u1s = u1e = None
for s, e in blocks(t, 'footprint'):
    if re.search(r'\(property "Reference" "U1"', t[s:e]): u1s, u1e = s, e; break
u1 = t[u1s:u1e]
for ps, pe in sorted(blocks(u1, 'pad'), key=lambda b: -b[0]):
    pb = u1[ps:pe]
    nm = re.match(r'\(pad "([^"]+)"', pb).group(1)
    if nm in padmap:
        assert '(net ' not in pb, "pad %s already has a net!" % nm
        code = netcode[padmap[nm]]
        u1 = u1[:ps] + pb[:-1] + ' (net %d "%s"))' % (code, padmap[nm]) + u1[pe:]
t = t[:u1s] + u1 + t[u1e:]

# --- 4. clone the existing "X" 3-pin servo header 3x for SERVO2/3/4
XU = "de21b1f2-1a5a-4d07-b9b9-13f6ebdf0926"           # uuid of the "X" footprint
xs = xe = None
for s, e in blocks(t, 'footprint'):
    if XU in t[s:e]: xs, xe = s, e; break
template = t[xs:xe]

# placement: horizontal row in the open area near (68.5, 46.5), pitch 3.302 like X/Y/Z
POS = {"SERVO2": 65.20, "SERVO3": 68.50, "SERVO4": 71.80}
YROW = 46.50
new_fps = []
for net in NEW:
    b = template
    b = re.sub(r'\(uuid "[0-9a-f\-]{36}"\)',
               lambda m: '(uuid "%s")' % uuid.uuid4(), b)   # fresh uuids (fp + pads)
    b = re.sub(r'\(path "[^"]*"\)\s*\n?', '', b)             # drop schematic path
    b = re.sub(r'\(at [\-\d.]+ [\-\d.]+ 0\)',
               '(at %.4f %.4f 0)' % (POS[net], YROW), b, count=1)
    b = b.replace('(property "Reference" "X"',
                  '(property "Reference" "%s"' % net, 1)
    b = b.replace('(net 6 "SERVOX")', '(net %d "%s")' % (netcode[net], net), 1)  # pad1 signal
    new_fps.append(b.rstrip())

block = "\n".join("  " + f.replace("\n", "\n  ") for f in new_fps)
idx = t.rstrip().rfind(")")
t = t[:idx] + block + "\n" + t[idx:]

open(OUT, "w", encoding="utf-8").write(t)
print("renamed SERVOZ -> SERVO1 (net + connector ref)")
print("added nets:", {n: netcode[n] for n in NEW})
print("Teensy PWM pins:", padmap)
print("placed SERVO2/3/4 headers at y=%.1f, x=%s" % (YROW, list(POS.values())))
