# Patch the fab package for Q20's AON6403 upgrade.
#
# Patches rather than regenerates, for the same reason the rest of this project does: the CPL
# carries hand-converged JLC rotations that KiCad cannot reproduce. Only Q20's row changes.
#
# Rotation convention, verified against the rows already in this CPL:
#   bottom-side parts  CPL = KiCad + 180   (Q16-Q19 KiCad 270 -> CPL 90 ; old Q20 KiCad 90 -> 270)
#   top-side parts     CPL = KiCad         (Q21 KiCad 90 -> CPL 90)
# Q20 is now KiCad rot 0 on B.Cu, so CPL = 180. Still UNVERIFIED against JLC's preview, exactly like
# the other four custom-footprint FETs -- a 180 error on the main battery switch is a dead board.
#
# BOM: Q20 joins Q21's AON6403 line. AO3401A disappears from the board entirely.
import csv, os, sys, zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
FAB = os.path.join(HERE, "fab")
Q20_X, Q20_Y, Q20_ROT = "175.7000", "-79.5000", "180"
LCSC, FPNAME = "C2760089", "AON6403_DFN5x6 (DFN-8 5x6)"

# ---- CPL ----
p = os.path.join(FAB, "Impulse_2.3_CPL.csv")
rows = list(csv.reader(open(p, newline="")))
hit = 0
for r in rows:
    if r and r[0] == "Q20":
        assert r[3] == "Bottom", "Q20 should be bottom-side, got %s" % r[3]
        print("CPL: Q20 %s,%s rot=%s -> %s,%s rot=%s" % (r[1], r[2], r[4], Q20_X, Q20_Y, Q20_ROT))
        r[1], r[2], r[4] = Q20_X, Q20_Y, Q20_ROT
        hit += 1
assert hit == 1, "expected one Q20 CPL row, got %d" % hit
csv.writer(open(p, "w", newline="")).writerows(rows)

# ---- BOM ----
p = os.path.join(FAB, "Impulse_2.3_BOM.csv")
rows = list(csv.reader(open(p, newline="")))
out, dropped, merged = [], 0, 0
for r in rows:
    if r and r[0] == "AO3401A":
        assert r[1] == "Q20", "unexpected AO3401A designators: %s" % r[1]
        dropped += 1
        continue                     # Q20 is no longer an AO3401A
    if r and r[0] == "AON6403":
        assert r[1] == "Q21", "unexpected AON6403 designators: %s" % r[1]
        r[1] = "Q20,Q21"             # same part, one line, one feeder
        merged += 1
    out.append(r)
assert dropped == 1 and merged == 1, "BOM patch did not apply cleanly (%d/%d)" % (dropped, merged)
csv.writer(open(p, "w", newline="")).writerows(out)
print("BOM: AO3401A row removed; AON6403 now covers Q20,Q21 (%s)" % LCSC)

# ---- gerber zip (board changed) ----
gdir = os.path.join(FAB, "gerbers")
files = sorted(f for f in os.listdir(gdir) if not f.endswith(".zip"))
z = os.path.join(FAB, "Impulse_2.3_gerbers.zip")
with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as zf:
    for f in files:
        zf.write(os.path.join(gdir, f), f)
print("gerbers: rezipped %d files" % len(files))
