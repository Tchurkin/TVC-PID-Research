#!/usr/bin/env python3
# Build a project footprint library (.pretty) from the PCB's embedded footprints,
# normalize all FPIDs to that lib, and write fp-lib-table. Clears lib_footprint_issues.
import re, sys, os, uuid
PCB_IN, PCB_OUT, PRETTY, FPLIB = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
LIBNICK = "ProDoc_P1_-easyedapro"
t = open(PCB_IN, encoding="utf-8").read()
os.makedirs(PRETTY, exist_ok=True)

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

made = set()
for s, e, b in blocks(t, 'footprint'):
    fpid = re.match(r'\(footprint "([^"]+)"', b).group(1)
    name = fpid.split(":", 1)[-1]
    if name in made:
        continue
    made.add(name)
    body = b
    body = re.sub(r'^\(footprint "[^"]+"',
                  f'(footprint "{name}"\n\t(version 20240108)\n\t(generator "altium_convert")', body, count=1)
    body = re.sub(r'\n\s+\(at [^\n]*\)', '', body, count=1)          # drop footprint placement
    body = re.sub(r'\n\s+\(path "[^"]*"\)', '', body)                # drop schematic path
    body = re.sub(r'\s*\(net \d+ "[^"]*"\)', '', body)               # drop pad nets (lib fp has none)
    body = re.sub(r'(\(property "Reference" ")[^"]*(")', r'\1REF**\2', body, count=1)
    fname = re.sub(r'[^A-Za-z0-9_.\-]', '_', name) + ".kicad_mod"
    open(os.path.join(PRETTY, fname), "w", encoding="utf-8").write(body + "\n")

# normalize every FPID prefix to the one lib so all footprints resolve
t = re.sub(r'\(footprint "[^":]+:', f'(footprint "{LIBNICK}:', t)
open(PCB_OUT, "w", encoding="utf-8").write(t)

open(FPLIB, "w", encoding="utf-8").write(
    '(fp_lib_table\n\t(version 7)\n'
    f'\t(lib (name "{LIBNICK}")(type "KiCad")(uri "${{KIPRJMOD}}/{LIBNICK}.pretty")(options "")(descr ""))\n)\n')

print(f"footprints written to .pretty: {len(made)}")
print(f"fp-lib-table -> lib '{LIBNICK}'")
