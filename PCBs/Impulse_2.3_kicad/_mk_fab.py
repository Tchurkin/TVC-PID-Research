# Build the 2.3 fab package.
#
# The BOM and CPL are DERIVED from 2.2's, not regenerated from scratch, because 2.2's CPL carries
# hand-converged JLC rotations that cost a whole session to establish (bottom-side parts +180
# wholesale, per-part electrolytic zeros, THT headers -90) and that KiCad cannot reproduce. A
# kicad-cli placement diff confirms exactly ONE row differs between the two boards -- Q21 -- so
# copying 2.2's calibration and patching that single row is both safe and lossless.
#
# Q21's rotation is deliberately NOT inherited: 2.2's value (180) was a SOT-23-family JLC
# calibration, and the part is now SOIC-8. It is emitted as 0 (KiCad-native, pin 1 top-left) and
# flagged for JLC-preview confirmation, which this project's own notes call the only ground truth.
import csv, os, shutil, sys, zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "Impulse_2.2_kicad", "fab")
DST = os.path.join(HERE, "fab")

Q21_X, Q21_Y, Q21_ROT = "167.8950", "-49.9840", "0"
AO4407A_LCSC = "C16072"          # AOS genuine; JLCPCB assembly library = Extended
AO4407A_FP = "SOIC-8_3.9x4.9mm_P1.27mm"

os.makedirs(DST, exist_ok=True)

# ---- CPL ----
rows = list(csv.reader(open(os.path.join(SRC, "Impulse_2.2_CPL.csv"), newline="")))
hit = 0
for r in rows:
    if r and r[0] == "Q21":
        assert r[1:] == ["167.8080", "-49.9840", "Top", "180"], "unexpected 2.2 Q21 CPL row: %s" % r
        r[1], r[2], r[4] = Q21_X, Q21_Y, Q21_ROT
        hit += 1
assert hit == 1, "expected exactly one Q21 CPL row, got %d" % hit
with open(os.path.join(DST, "Impulse_2.3_CPL.csv"), "w", newline="") as f:
    csv.writer(f).writerows(rows)
print("CPL: Q21 -> x=%s rot=%s (was 167.8080 / 180 for the SOT-23)" % (Q21_X, Q21_ROT))

# ---- BOM ----
rows = list(csv.reader(open(os.path.join(SRC, "Impulse_2.2_BOM.csv"), newline="")))
out, hit = [], 0
for r in rows:
    if r and r[0] == "AO3401A":
        assert r[1] == "Q20,Q21", "unexpected 2.2 AO3401A designators: %s" % r[1]
        out.append(["AO3401A", "Q20", r[2], r[3] if len(r) > 3 else ""])
        out.append(["AO4407A", "Q21", AO4407A_FP, AO4407A_LCSC])
        hit += 1
    else:
        out.append(r)
assert hit == 1, "expected exactly one AO3401A BOM row, got %d" % hit
with open(os.path.join(DST, "Impulse_2.3_BOM.csv"), "w", newline="") as f:
    csv.writer(f).writerows(out)
print("BOM: AO3401A row split -> Q20 stays AO3401A, Q21 = AO4407A (%s)" % AO4407A_LCSC)

# ---- hand-solder list (unchanged: Q21 is machine-placed either way) ----
shutil.copy2(os.path.join(SRC, "HAND_SOLDER.txt"), os.path.join(DST, "HAND_SOLDER.txt"))

# ---- gerber zip ----
gdir = os.path.join(DST, "gerbers")
files = sorted(f for f in os.listdir(gdir) if not f.endswith(".zip"))
zpath = os.path.join(DST, "Impulse_2.3_gerbers.zip")
with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
    for f in files:
        z.write(os.path.join(gdir, f), f)
print("gerbers: %d files -> %s" % (len(files), os.path.basename(zpath)))

open(os.path.join(DST, "ORDER_NOTES.txt"), "w").write("""Impulse 2.3 -- notes before ordering
====================================

WHY 2.3 EXISTS
  Q21, the pyro rail ARM switch, was an AO3401A in SOT-23 carrying the firing current of every
  pyro channel. It died on the bench 2026-08-12 on a full-length (~1 s) pulse. 2.3 replaces it
  with an AO4407A in SOIC-8 and widens the pyro rail/drain copper.

VERIFY BEFORE YOU ORDER
  1. Q21 ROTATION IS UNVERIFIED. 2.2's CPL used 180 for Q21, but that was a SOT-23-family JLC
     calibration and the part is now SOIC-8. This CPL emits 0 (KiCad-native, pin 1 = top-left).
     CHECK IT IN JLC'S PREVIEW against the board's silk pin-1 tick before paying. Every other row
     is byte-identical to the 2.2 order that was placed on 2026-08-01.
  2. Q21 = AO4407A, LCSC C16072 (Alpha & Omega), JLCPCB assembly library = Extended (feeder fee).
     If it is out of stock, JLC also lists AO4407A as C3011194 (TECH PUBLIC) and C5155211 (JSMSEMI).
     Any substitute MUST keep: P-channel, Vds >= -20 V, RDS(on) specified at VGS = -4.5 V or -6 V
     (the gate sits at -6.7 V from the R67/R68 divider, -7.6 V on a full pack), and a package with
     RthJA <= ~50 C/W. Do NOT substitute back into SOT-23 -- that is the failure being fixed.
  3. Same 4-layer stackup as 2.2: JLC04161H-7628, 1 oz outer.
  4. Delivery inspection, unchanged from 2.2: ICM/DPS310 pin-1 dots vs silk before first power-up,
     R67 present (pyro-arm pull-up -- safety), electrolytic stripes opposite the silk "+".

KNOWN, ACCEPTED DRC DEVIATIONS
  22 courtyard overlaps (2.2 had 20). The two new ones are Q21 vs R67 and Q21 vs C10: the SOIC-8's
  courtyard is generous, but actual PAD-to-PAD clearance is 0.48 mm on both sides and body-to-body
  is over 2 mm, which is comfortably inside JLC's assembly minimum. 14 pth_inside_courtyard are the
  pre-existing deliberate under-Teensy placement.
""")
print("wrote ORDER_NOTES.txt")
