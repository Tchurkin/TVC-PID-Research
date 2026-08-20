# Regenerate the JLC CPL after a re-placement WITHOUT discarding the hand-converged rotations.
#
# 2.2/2.3's CPL rotations cost a session to establish and are NOT a single rule: bottom-side parts
# generally take +180 vs kicad-cli, but electrolytic zeros are PER-PART (C39 took the class +180,
# C10/C43 the opposite -- different suppliers) and THT headers needed -90. Regenerating from scratch
# throws that away; keeping the old file is wrong because 27 footprints have moved.
#
# So preserve each part's OFFSET rather than its rotation:
#     offset      = old_CPL_rot - old_kicad_rot     (mod 360, per part)
#     new_CPL_rot = new_kicad_rot + offset          (mod 360)
# A part that didn't move reproduces its old row exactly (a self-check we assert on); a part that
# moved keeps its calibration and picks up its new position/rotation.
import csv, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
T = os.environ.get("TEMP", "/tmp")
CPL = os.path.join(HERE, "fab", "Impulse_2.3_CPL.csv")


def load_pos(p):
    d = {}
    for r in csv.reader(open(p, newline="")):
        if not r or r[0] == "Ref":
            continue
        d[r[0]] = (float(r[3]), float(r[4]), float(r[5]), r[6])
    return d


old = load_pos(os.path.join(T, "pos_old.csv"))
new = load_pos(os.path.join(T, "pos_new.csv"))
rows = list(csv.reader(open(CPL, newline="")))
hdr, body = rows[0], rows[1:]

out, moved, same, problems = [hdr], 0, 0, []
for r in body:
    ref = r[0]
    if ref not in old or ref not in new:
        problems.append("%s: no placement data" % ref)
        out.append(r)
        continue
    oy_rot = old[ref][2]
    ny_rot = new[ref][2]
    offset = (float(r[4]) - oy_rot) % 360
    nx, ny = new[ref][0], new[ref][1]
    nrot = (ny_rot + offset) % 360
    side = "Bottom" if new[ref][3].lower().startswith("b") else "Top"
    if side != r[3]:
        problems.append("%s: side changed %s -> %s" % (ref, r[3], side))
    row = [ref, "%.4f" % nx, "%.4f" % ny, side, "%g" % nrot]
    if (abs(nx - float(r[1])) < 1e-4 and abs(ny - float(r[2])) < 1e-4
            and abs(((nrot - float(r[4])) % 360)) < 1e-6):
        same += 1
    else:
        moved += 1
    out.append(row)

csv.writer(open(CPL, "w", newline="")).writerows(out)
print("CPL regenerated: %d rows unchanged, %d updated" % (same, moved))
for p in problems:
    print("   PROBLEM:", p)
if problems:
    sys.exit(1)
