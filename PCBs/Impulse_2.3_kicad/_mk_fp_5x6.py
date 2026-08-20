# Generate the two 5x6 power-MOSFET footprints the upsized pyro path needs, both derived from
# KiCad's PQFN-8-EP_6x5mm_P1.27mm_Generic land pattern -- whose own description names exactly the
# two packages we are using: "Texas Instrument SON 5 x 6 mm (Q5A), Alpha and Omega DFN 5 x 6".
#
# The two parts are the same physical package but number their pins differently:
#   AON6403      (P-ch): leads = 1,2,3 SOURCE + 4 GATE ; tab = DRAIN (pins 5-8)
#   CSD17301Q5A  (N-ch): tab  = DRAIN (pins 1-4)       ; leads = 5 GATE + 6,7,8 SOURCE
# The generic footprint already numbers leads 1-4 and the tab 5, which IS the AOS map, so the AOS
# variant is a straight copy. The TI variant needs the geometry rotated 180 deg in its OWN frame
# (tab to the left, leads to the right, gate at bottom-right) and the pads renumbered.
#
# Rotating in the footprint's local frame rather than baking a 180 deg board rotation matters:
# it keeps pin 1 where the manufacturer's datasheet puts it, so the JLC CPL rotation stays a normal
# 0 and we do not repeat the SOT-23 rotation confusion.
import os, re, sys

SRC = r"C:/Program Files/KiCad/10.0/share/kicad/footprints/Package_DFN_QFN.pretty/PQFN-8-EP_6x5mm_P1.27mm_Generic.kicad_mod"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Impulse22.pretty")

src = open(SRC, encoding="utf-8").read()


def bal(s, i):
    d = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            d += 1
        elif s[j] == ")":
            d -= 1
            if d == 0:
                return j + 1


def rot180_coords(text):
    """Negate x,y in (at x y [rot]), (start ..), (end ..), (center ..), (xy ..) tokens."""
    def neg(v):
        f = -float(v)
        return ("%g" % f) if f != 0 else "0"

    def at_sub(m):
        rot = m.group(3)
        if rot is None:
            return "(at %s %s)" % (neg(m.group(1)), neg(m.group(2)))
        # a pad's own rotation is unchanged by rotating the whole body 180 for a 2-fold symmetric pad
        return "(at %s %s %s)" % (neg(m.group(1)), neg(m.group(2)), rot)

    text = re.sub(r"\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", at_sub, text)
    for tag in ("start", "end", "center", "mid", "xy"):
        text = re.sub(r"\(%s ([-\d.]+) ([-\d.]+)\)" % tag,
                      lambda m, t=tag: "(%s %s %s)" % (t, neg(m.group(1)), neg(m.group(2))), text)
    return text


def build(name, descr, tags, renumber, rotate):
    t = src
    # rename the footprint
    t = re.sub(r'\(footprint "[^"]+"', '(footprint "%s"' % name, t, count=1)
    t = re.sub(r'\(descr "[^"]*"', '(descr "%s"' % descr, t, count=1)
    t = re.sub(r'\(tags "[^"]*"', '(tags "%s"' % tags, t, count=1)
    if rotate:
        t = rot180_coords(t)
    if renumber:
        # do it in one pass via a placeholder so 1->5 and 5->1 cannot collide
        t = re.sub(r'\(pad "([1-9])"', lambda m: '(pad "@%s@"' % renumber[m.group(1)], t)
        t = re.sub(r'\(pad "@(\w+)@"', lambda m: '(pad "%s"' % m.group(1), t)
    return t


def extents(t):
    xs, ys = [], []
    for m in re.finditer(r'\(pad "([1-9])"[\s\S]{0,200}?\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)'
                         r'\s*\n\s*\(size ([\d.]+) ([\d.]+)\)', t):
        x, y = float(m.group(2)), float(m.group(3))
        rot = float(m.group(4) or 0)
        w, h = float(m.group(5)), float(m.group(6))
        if abs(rot % 180 - 90) < 1:
            w, h = h, w
        xs += [x - w / 2, x + w / 2]
        ys += [y - h / 2, y + h / 2]
    # the tab is a custom pad: fold in its polygon points too
    for m in re.finditer(r'\(pad "[1-9]" smd custom[\s\S]*?\(primitives([\s\S]*?)\n\t\t\)', t):
        for pm in re.finditer(r"\(xy ([-\d.]+) ([-\d.]+)\)", m.group(1)):
            xs.append(float(pm.group(1)))
            ys.append(float(pm.group(2)))
    return min(xs), max(xs), min(ys), max(ys)


os.makedirs(OUT, exist_ok=True)
jobs = [
    ("AON6403_DFN5x6",
     "AOS DFN 5x6 power MOSFET land pattern (from KiCad PQFN-8-EP_6x5mm_P1.27mm_Generic). "
     "Pads 1,2,3=SOURCE 4=GATE 5=DRAIN tab. For AON6403 P-channel, LCSC C2760089.",
     "DFN5x6 PQFN power mosfet AOS", None, False),
    ("TI_SON5x6_Q5A",
     "TI SON 5x6 (Q5A) power MOSFET land pattern (from KiCad PQFN-8-EP_6x5mm_P1.27mm_Generic, "
     "rotated 180 in-frame). Pad 1=DRAIN tab, 5=GATE, 6,7,8=SOURCE. For CSD17301Q5A, LCSC C129940.",
     "SON5x6 Q5A power mosfet TI", {"1": "5", "2": "6", "3": "7", "4": "8", "5": "1"}, True),
]
for name, descr, tags, renum, rot in jobs:
    t = build(name, descr, tags, renum, rot)
    p = os.path.join(OUT, name + ".kicad_mod")
    open(p, "w", encoding="utf-8").write(t)
    x0, x1, y0, y1 = extents(t)
    nums = re.findall(r'\(pad "([1-9])"', t)
    print("%-18s pads=%-14s copper extent x[%.3f..%.3f] y[%.3f..%.3f]  (%.2f x %.2f mm)"
          % (name, ",".join(nums), x0, x1, y0, y1, x1 - x0, y1 - y0))
print("\nwrote to", OUT)
