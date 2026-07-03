#!/usr/bin/env python3
# Link PCB footprints to schematic symbols by reference -> write KiCad UUID path + matching FPID.
import re, sys
SCH, PCB_IN, PCB_OUT = sys.argv[1], sys.argv[2], sys.argv[3]
sch = open(SCH, encoding="utf-8").read()
pcb = open(PCB_IN, encoding="utf-8").read()

def blocks(text, token):
    out = []
    for m in re.finditer(r'\(' + token + r'\b', text):
        i = m.start(); d = 0; j = i
        while j < len(text):
            c = text[j]
            if c == '(': d += 1
            elif c == ')':
                d -= 1
                if d == 0: break
            j += 1
        out.append((i, j + 1, text[i:j + 1]))
    return out

# --- schematic: reference -> (symbol_uuid, footprint_field, sheet_path) ---
symmap = {}
for s, e, b in blocks(sch, 'symbol'):
    if '(instances' not in b:           # only placed instances, not lib defs
        continue
    pathm = re.search(r'\(path "([^"]+)"\s*\(reference "([^"]+)"', b)
    if not pathm:
        continue
    sheetpath, ref = pathm.group(1), pathm.group(2)
    if ref in ('', '?') or ref.startswith('#'):
        continue
    uu = re.search(r'\(uuid "?([0-9a-fA-F-]{36})"?\)', b)   # first uuid = symbol uuid
    fpf = re.search(r'\(property "Footprint" "([^"]*)"', b)
    if uu:
        symmap[ref] = (uu.group(1), fpf.group(1) if fpf else None, sheetpath)

# --- edit PCB footprints ---
linked = 0; unlinked = []
parts = blocks(pcb, 'footprint')
new = pcb
# process from end to start so indices stay valid
for s, e, b in sorted(parts, key=lambda x: -x[0]):
    rm = re.search(r'\(property "Reference" "([^"]+)"', b)
    if not rm:
        continue
    ref = rm.group(1)
    if ref not in symmap:
        unlinked.append(ref); continue
    symuuid, fpf, sheetpath = symmap[ref]
    fppath = sheetpath.rstrip('/') + '/' + symuuid
    nb = b
    # (a) set FPID to match schematic Footprint field (prevents swap on update)
    if fpf:
        nb = re.sub(r'^\(footprint "[^"]*"', f'(footprint "{fpf}"', nb, count=1)
    # (b) insert path after the footprint's own (at ...) line, if not already present
    if '(path "' not in nb:
        nb = re.sub(r'(\n\s*\(at [^\n]*\)\n)', r'\1    (path "' + fppath + '")\n', nb, count=1)
    new = new[:s] + nb + new[e:]
    linked += 1

open(PCB_OUT, "w", encoding="utf-8").write(new)
print(f"schematic placed symbols: {len(symmap)}")
print(f"footprints linked: {linked}")
print(f"footprints with NO matching schematic symbol ({len(unlinked)}): {sorted(set(unlinked))}")
# sanity: show U1 result
mm = re.search(r'\(footprint "[^"]*TEENSY[^"]*".*?\(path "([^"]+)"', new, re.S)
print("U1 footprint path ->", mm.group(1) if mm else "NONE")
