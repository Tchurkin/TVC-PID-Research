#!/usr/bin/env python3
# Ground-truth patch: use KiCad's exported netlist to find any pin whose net didn't propagate
# through the wiring (sub-grid bridge misses), and drop a global label directly on that pin.
# Idempotent-ish: re-run generator first, then this.
import re, json, uuid, subprocess, sys
SCH = "Impulse_2.2.kicad_sch"; PCB = "Impulse_2.2.kicad_pcb"
CLI = "C:/Program Files/KiCad/10.0/bin/kicad-cli.exe"
def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
def g(v): return ('%.2f' % v).rstrip('0').rstrip('.')

subprocess.run([CLI, "sch", "export", "netlist", "--output", "_netlist.net", SCH],
               check=True, capture_output=True)
n = open("_netlist.net", encoding="utf-8").read()
sch = set()
for m in re.finditer(r'\(net\s+\(code "[^"]*"\)\s+\(name "([^"]+)"\)', n):
    blk = n[m.start():bend(n, m.start())]
    for nm in re.finditer(r'\(node\s+\(ref "([^"]+)"\)\s+\(pin "([^"]+)"\)', blk):
        sch.add((m.group(1), nm.group(1), nm.group(2)))
p = open(PCB, encoding="utf-8").read()
pcb = set()
for m in re.finditer(r'\(footprint "([^"]+)"', p):
    b = p[m.start():bend(p, m.start())]
    ref = re.search(r'\(property "Reference" "([^"]+)"', b).group(1)
    for pm in re.finditer(r'\(pad\s+"([^"]*)"', b):
        pb = b[pm.start():bend(b, pm.start())]
        net = re.search(r'\(net "([^"]*)"\)', pb)
        if net: pcb.add((net.group(1), ref, pm.group(1)))
miss = pcb - sch
if not miss:
    print("nothing to patch: netlist already identical")
    sys.exit(0)
pinpts = {(nn, ref, num): (x, y) for x, y, nn, ref, num in json.load(open("_pin_nets.json"))}
s = open(SCH, encoding="utf-8").read()
add = ""
patched = 0
for (net, ref, num) in sorted(miss):
    pt = pinpts.get((net, ref, num))
    if not pt:
        print("  !! no pin coord for", (net, ref, num)); continue
    add += ('  (global_label "%s" (shape input) (at %s %s 0) '
            '(effects (font (size 1.27 1.27)) (justify left))\n    (uuid %s)\n'
            '    (property "Intersheetrefs" "${INTERSHEET_REFS}" (at %s %s 0) (hide yes) '
            '(effects (font (size 1.27 1.27))))\n  )\n'
            % (net, g(pt[0]), g(pt[1]), uuid.uuid4(), g(pt[0]), g(pt[1])))
    patched += 1
idx = s.rfind('(sheet_instances')
s = s[:idx] + add + s[idx:]
open(SCH, "w", encoding="utf-8").write(s)
print("patched %d pins with a label at the pin itself:" % patched)
for e in sorted(miss): print("   ", e)
