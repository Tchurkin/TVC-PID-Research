# Guard against collateral footprint movement from a pcbnew LoadBoard/Remove/Add/SaveBoard cycle.
#
# WHY THIS EXISTS: running _place_pyro_fets.py for Q20 on an already-upsized board moved NINE other
# footprints as well as Q20 -- Q16-Q19 dragged 3 mm south (straight back onto the CS_IMU/INT_IMU via
# field the y=45.5 placement existed to avoid), Q21's rotation reset 90 -> 0, and PI/C15/R67/R68
# shifted west. Result: 15 unconnected and 19 shorts on a board that had been clean. The board had
# to be restored from a snapshot.
#
# This is the documented pcbnew hazard on this project ("b.Remove() in a loop corrupts subsequent
# Get* iteration ... the corrupted-Get* gotcha extends to what SaveBoard serializes"), and the
# lesson is that a pcbnew save is NOT a safe no-op for the rest of the board. So: snapshot every
# footprint's (at x y rot) + layer BEFORE, and afterwards put back anything that moved except the
# one part being placed.
#
#   python _guard_placement.py snapshot <pcb> <json>
#   python _guard_placement.py repair   <pcb> <json> <ref-to-exclude>
import json, re, sys


def bal(s, i):
    d = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            d += 1
        elif s[j] == ")":
            d -= 1
            if d == 0:
                return j + 1


def scan(text):
    out = {}
    for m in re.finditer(r'\(footprint "', text):
        e = bal(text, m.start())
        blk = text[m.start():e]
        r = re.search(r'\(property "Reference" "([^"]+)"', blk)
        at = re.search(r"\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", blk)
        lay = re.search(r'\(layer "([^"]+)"\)', blk)
        if r and at:
            out[r.group(1)] = [at.group(1), at.group(2), at.group(3) or "0",
                               lay.group(1) if lay else "?", m.start(), e]
    return out


mode, pcb = sys.argv[1], sys.argv[2]
text = open(pcb, encoding="utf-8", newline="").read()
crlf = "\r\n" in text
t = text.replace("\r\n", "\n")

if mode == "snapshot":
    snap = {k: v[:4] for k, v in scan(t).items()}
    json.dump(snap, open(sys.argv[3], "w"), indent=0)
    print("snapshot: %d footprints -> %s" % (len(snap), sys.argv[3]))
    sys.exit(0)

if mode == "repair":
    ref_keep = sys.argv[4]
    snap = json.load(open(sys.argv[3]))
    cur = scan(t)
    moved = [r for r in sorted(set(snap) & set(cur))
             if r != ref_keep and [cur[r][0], cur[r][1], cur[r][2], cur[r][3]] != snap[r]]
    if not moved:
        print("repair: nothing moved -- placement was clean")
        sys.exit(0)
    # rewrite from the end so earlier offsets stay valid
    for r in sorted(moved, key=lambda r: -cur[r][4]):
        x, y, rot, lay, s, e = cur[r]
        ox, oy, orot, olay = snap[r]
        blk = t[s:e]
        newat = "(at %s %s)" % (ox, oy) if orot in ("0", "0.0") else "(at %s %s %s)" % (ox, oy, orot)
        blk2 = re.sub(r"\(at [-\d.]+ [-\d.]+(?: [-\d.]+)?\)", newat, blk, count=1)
        if olay != lay:
            print("   WARNING %s changed layer %s -> %s; not auto-repaired" % (r, olay, lay))
        t = t[:s] + blk2 + t[e:]
        print("   restored %-8s (%s,%s r=%s) <- was moved to (%s,%s r=%s)" % (r, ox, oy, orot, x, y, rot))
    open(pcb, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
    print("repair: %d footprints put back" % len(moved))
    sys.exit(0)

print("use: snapshot <pcb> <json> | repair <pcb> <json> <ref>")
sys.exit(1)
