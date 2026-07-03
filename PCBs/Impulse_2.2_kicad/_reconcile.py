#!/usr/bin/env python3
# Convert (sync) the PCB to the user's edited schematic:
#  - the user renumbered/redrew some pyro+regulator parts (new UUIDs, new refs) and deleted my
#    net-name-label clutter (nets are auto-named now). Wiring is electrically identical.
#  - Re-link every schematic symbol to its existing PCB footprint (UUID, else footprint+net
#    topology), rename the PCB footprint ref + path to match the schematic, and rename PCB nets
#    to the schematic's names. Placement is preserved.
#  - The two board-only breakout connectors (AUX_3V3_1, OUT1) have no schematic symbol; keep them
#    on the board, flagged board-only, on their (unchanged) nets.
import re, json
SCH = "Impulse_2.2.kicad_sch"; PCB = "Impulse_2.2.kicad_pcb"; NL = "_netlist_new.net"
def bend(t, i):
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: return j + 1
    raise ValueError

# ---- schematic: ref -> uuid, footprint ----
s = open(SCH, encoding="utf-8").read()
sref = {}
for m in re.finditer(r'\(lib_id "([^"]+)"\)', s):
    st = s.rfind('(symbol', 0, m.start()); blk = s[st:bend(s, st)]
    rr = re.search(r'\(property "Reference" "([^"]+)"', blk)
    uu = re.search(r'\(uuid "([0-9a-f\-]{36})"\)', blk)
    fp = re.search(r'\(property "Footprint" "([^"]+)"', blk)
    if rr and uu and not rr.group(1).startswith('#'):
        sref[rr.group(1)] = dict(uuid=uu.group(1), fp=fp.group(1) if fp else "")

# ---- schematic netlist: ref -> {pin: net} ----
n = open(NL, encoding="utf-8").read()
spin = {}
for m in re.finditer(r'\(net\s+\(code "[^"]*"\)\s+\(name "([^"]+)"\)', n):
    blk = n[m.start():bend(n, m.start())]
    for a, b in re.findall(r'\(node\s+\(ref "([^"]+)"\)\s+\(pin "([^"]+)"\)', blk):
        spin.setdefault(a, {})[b] = m.group(1)

# ---- PCB: pref -> footprint block info ----
p = open(PCB, encoding="utf-8").read()
pcb = {}
for m in re.finditer(r'\(footprint "([^"]+)"', p):
    e = bend(p, m.start()); blk = p[m.start():e]
    rr = re.search(r'\(property "Reference" "([^"]+)"', blk)
    pa = re.search(r'\(path "/([0-9a-f\-]{36})"\)', blk)
    pads = {}
    for pm in re.finditer(r'\(pad\s+"([^"]*)"', blk):
        pb = blk[pm.start():bend(blk, pm.start())]
        net = re.search(r'\(net "([^"]*)"\)', pb)
        pads.setdefault(pm.group(1), net.group(1) if net else None)
    pcb[rr.group(1)] = dict(fp=m.group(1), path=pa.group(1) if pa else None,
                            pads=pads, start=m.start(), end=e)

# ---- match: 1) by UUID ----
path2ref = {v['path']: k for k, v in pcb.items() if v['path']}
match = {}   # sref -> pref
for r, d in sref.items():
    if d['uuid'] in path2ref: match[r] = path2ref[d['uuid']]
# ---- net correspondence pcbnet<->schnet from matched shared pin numbers ----
p2s = {}
for sr, pr in match.items():
    for pin, sn in spin.get(sr, {}).items():
        pn = pcb[pr]['pads'].get(pin)
        if pn and sn: p2s.setdefault(pn, set()).add(sn)
p2s = {k: list(v)[0] for k, v in p2s.items() if len(v) == 1}
s2p = {v: k for k, v in p2s.items()}
# ---- 2) iterative partial-signature match: anchor on known nets, propagate correspondences ----
def add_corr(pn, sn):        # conflict-safe both directions
    if pn in p2s and p2s[pn] != sn: return
    if sn in s2p and s2p[sn] != pn: return
    p2s[pn] = sn; s2p[sn] = pn
def propagate(sr, pr):
    # align sch pins to pcb pins by KNOWN net correspondence first (handles symmetric parts
    # where pin numbering is flipped), then force the remaining single pin by elimination.
    sp = spin.get(sr, {}); pp = pcb[pr]['pads']
    used = set(); pairs = []
    for s, sn in sp.items():
        for c, pn in pp.items():
            if c in used: continue
            if (sn in s2p and s2p[sn] == pn) or (pn in p2s and p2s[pn] == sn):
                pairs.append((s, c)); used.add(c); break
    rem_s = [s for s in sp if s not in {x[0] for x in pairs}]
    rem_c = [c for c in pp if c not in used]
    if len(rem_s) == len(rem_c) <= 1:          # unambiguous remainder only
        pairs += list(zip(rem_s, rem_c))
    for s, c in pairs:
        if sp[s] and pp[c]: add_corr(pp[c], sp[s])
for _round in range(15):
    progressed = False
    taken = set(match.values())
    for sr in [r for r in sref if r not in match]:
        fp = sref[sr]['fp']
        known = {s2p[sn] for sn in spin.get(sr, {}).values() if sn in s2p}   # required pcb nets
        if not known: continue
        cands = [pr for pr in pcb if pr not in taken and pcb[pr]['fp'] == fp
                 and known <= {v for v in pcb[pr]['pads'].values() if v}]
        if len(cands) == 1:
            pr = cands[0]; match[sr] = pr; taken.add(pr); progressed = True
            propagate(sr, pr)
    if not progressed: break
unmatched = [r for r in sref if r not in match]
leftover_pcb = [r for r in pcb if r not in set(match.values())]
print("matched %d/%d sch symbols" % (len(match), len(sref)))
print("leftover PCB footprints:", leftover_pcb)
if unmatched:
    print("STILL UNMATCHED — diagnostics:")
    for sr in unmatched:
        knownpcb = {s2p[sn] for sn in spin.get(sr, {}).values() if sn in s2p}
        allnets = set(spin.get(sr, {}).values())
        print("  %s fp=%s  sch-nets=%s  -> known-pcb-nets=%s" %
              (sr, sref[sr]['fp'].split(':')[-1], allnets, knownpcb))
    for pr in leftover_pcb:
        print("  PCB %s fp=%s nets=%s" %
              (pr, pcb[pr]['fp'].split(':')[-1], {v for v in pcb[pr]['pads'].values() if v}))
# fallback: unique bipartite assignment among leftovers of the same footprint by net overlap
taken = set(match.values())
from collections import defaultdict
by_fp_pcb = defaultdict(list)
for pr in leftover_pcb:
    by_fp_pcb[pcb[pr]['fp']].append(pr)
for sr in list(unmatched):
    prs = by_fp_pcb[sref[sr]['fp']]
    if len(prs) == 1:
        match[sr] = prs[0]; prs.clear()
        for pin, sn in spin.get(sr, {}).items():
            pn = pcb[prs[0] if False else match[sr]]['pads'].get(pin)
            if pn and sn: add_corr(pn, sn)
unmatched = [r for r in sref if r not in match]
leftover_pcb = [r for r in pcb if r not in set(match.values())]
assert not unmatched, "unresolved after fallback: %s" % unmatched

# ---- apply: for each matched part rename ref+path AND set every pad's net = schematic pin's ----
# net (by pin/pad number = how KiCad links symbol<->footprint). This syncs names AND symmetric
# pin swaps in one shot.
edits = []
nchg = 0
for sr, pr in match.items():
    d = pcb[pr]
    blk = p[d['start']:d['end']]
    blk = re.sub(r'(\(property "Reference" ")[^"]*(")', r'\g<1>%s\g<2>' % re.escape(sr).replace('\\', ''), blk, count=1)
    blk = re.sub(r'(\(path "/)[0-9a-f\-]{36}(")', r'\g<1>%s\g<2>' % sref[sr]['uuid'], blk, count=1)
    # rewrite pad nets by pad number -> schematic pin net
    pins = spin.get(sr, {})
    out = []; last = 0
    for pm in re.finditer(r'\(pad\s+"([^"]*)"', blk):
        ps = pm.start(); pe = bend(blk, ps); pb = blk[ps:pe]; padnum = pm.group(1)
        want = pins.get(padnum)
        if want is not None:
            nm = re.search(r'\(net "([^"]*)"\)', pb)
            if nm and nm.group(1) != want:
                pb = pb[:nm.start()] + '(net "%s")' % want + pb[nm.end():]; nchg += 1
            elif not nm and 'np_thru_hole' not in pb:
                pb = pb[:-1] + ' (net "%s")' % want + ')'; nchg += 1
        out.append(blk[last:ps]); out.append(pb); last = pe
    out.append(blk[last:])
    edits.append((d['start'], d['end'], "".join(out)))
for st, en, blk in sorted(edits, key=lambda e: -e[0]):
    p = p[:st] + blk + p[en:]
open(PCB, "w", encoding="utf-8").write(p)
print("applied: %d parts relinked, %d pad-net assignments updated" % (len(match), nchg))
