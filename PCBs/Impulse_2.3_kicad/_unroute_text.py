# Unroute the board by TEXT SURGERY, never a pcbnew Remove() loop.
#
# WHY THE METHOD MATTERS: this project has already been bitten -- `_unroute_all.py`'s
# b.Remove()-loop followed by SaveBoard silently DROPPED all four GND zone definitions, because the
# corrupted-Get* iteration gotcha extends to what SaveBoard serializes. Zone outlines are
# board-outline pours and placement-independent, so they are worth keeping across a re-layout;
# losing them means transplanting them back from a backup by hand.
#
# What this removes:  every (segment ...) and (via ...), plus the (filled_polygon ...) inside zones
#                     so the board doesn't display stale copper over a moved layout.
# What this KEEPS:    all 99 footprints and their placement, the 4 zone OUTLINES, board outline,
#                     silk, netclasses, stackup.
#
# Usage: python _unroute_text.py [--dry-run]
import os, re, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
DRY = "--dry-run" in sys.argv


def bal(s, i):
    d = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            d += 1
        elif s[j] == ")":
            d -= 1
            if d == 0:
                return j + 1
    raise ValueError("unbalanced at %d" % i)


def strip_blocks(text, opener, indent="\t"):
    """Remove every top-level (opener ...) block that starts at the given indent."""
    out, pos, n = [], 0, 0
    pat = re.compile(r"^%s\(%s\b" % (re.escape(indent), re.escape(opener)), re.M)
    for m in pat.finditer(text):
        if m.start() < pos:
            continue
        e = bal(text, m.start() + len(indent))
        while e < len(text) and text[e] in "\r\n":
            e += 1
        out.append(text[pos:m.start()])
        pos = e
        n += 1
    out.append(text[pos:])
    return "".join(out), n


t = open(PCB, encoding="utf-8", newline="").read()
crlf = "\r\n" in t
t = t.replace("\r\n", "\n")

before = {k: len(re.findall(r"^\t\(%s\b" % k, t, re.M)) for k in ("segment", "via", "zone", "footprint")}

t, n_seg = strip_blocks(t, "segment")
t, n_via = strip_blocks(t, "via")

# zone fills live nested inside each zone block; drop the polygons, keep the outline
n_fill = 0
while True:
    m = re.search(r"^\t+\(filled_polygon\b", t, re.M)
    if not m:
        break
    ind = len(m.group(0)) - len("(filled_polygon")
    e = bal(t, m.start() + ind)
    while e < len(t) and t[e] in "\r\n":
        e += 1
    t = t[:m.start()] + t[e:]
    n_fill += 1

after = {k: len(re.findall(r"^\t\(%s\b" % k, t, re.M)) for k in ("segment", "via", "zone", "footprint")}

print("removed: %d segments, %d vias, %d zone fill polygons" % (n_seg, n_via, n_fill))
print("  %-10s %8s %8s" % ("item", "before", "after"))
for k in ("segment", "via", "zone", "footprint"):
    flag = ""
    if k in ("zone", "footprint") and before[k] != after[k]:
        flag = "   <-- LOST, ABORT"
    print("  %-10s %8d %8d%s" % (k, before[k], after[k], flag))

if after["zone"] != before["zone"] or after["footprint"] != before["footprint"]:
    print("FAIL: zones or footprints were lost -- not writing")
    sys.exit(1)
if after["segment"] or after["via"]:
    print("FAIL: residual routing left behind")
    sys.exit(1)

if DRY:
    print("\n--dry-run: nothing written")
    sys.exit(0)

bak = PCB + ".routed_2026-08-18"
if not os.path.exists(bak):
    shutil.copy2(PCB, bak)
    print("\nbackup of the fully-routed, DRC-clean board: %s" % os.path.basename(bak))
open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
print("unrouted. Placement, zone outlines, silk, netclasses and stackup all intact.")
