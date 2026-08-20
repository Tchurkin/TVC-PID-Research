# Impulse 2.3 root-cause fix: pyro ARM switch Q21, AO3401A/SOT-23 -> AO4407A/SOIC-8.
#
# WHY (bench failure 2026-08-12, and DESIGN_LOG 2026-07-23 predicted it):
#   Q21 is the series high-side arm switch for the WHOLE pyro rail -- every channel's firing current
#   flows through this one part. A 900-1000 ms pulse into a ~1 ohm initiator is ~7-8 A. In an
#   AO3401A that is 60 mOhm (>80 mOhm hot) => 4-5 W in a SOT-23 whose transient thermal impedance is
#   ~130 C/W at one second: several hundred C of junction rise. It died on the first full-length
#   pulse. The firmware's burst-fire workaround (30 ms on / 150 ms off) cannot be the whole answer,
#   because the LEGS channel drives a NICHROME BAND CUTTER that must stay hot for 900 ms continuous
#   -- bursting it defeats the cut. So the fix has to be in hardware.
#
# PART: AO4407A (AOS, verified against the Rev 11.1 datasheet, not from memory):
#   -30 V, -12 A, RDS(on) <= 17 mOhm @ VGS = -6 V (we sit at -6.7 V from the R67/R68 divider,
#   -7.6 V at a full 8.4 V pack), VGS rating +/-25 V, SOIC-8, RthJA 40 C/W max for t <= 10 s.
#   Worst case 8 A: 8^2 * 0.023 (hot) = 1.5 W * 40 = ~59 C rise, vs several hundred before.
#   LCSC C16072 (AOS); also in the JLCPCB assembly library as C3011194 / C5155211.
#
# SYMBOL: KiCad's Transistor_FET:IRF7404 is a single P-channel SO-8 whose pin ENDPOINTS are
#   identical to AO3401A's (G=(-5.08,0), S=(2.54,-5.08), D=(2.54,5.08)) and whose numbering is the
#   standard SO-8 power-FET map (1,2,3=S  4=G  5,6,7,8=D) that AO4407A uses. Cloning its body as
#   "AO4407A" therefore preserves the schematic wiring BY CONSTRUCTION -- same trick as the proven
#   CSD17313Q2 swap in _swap_pyro_fets.py -- and lets us use the stock SOIC-8 footprint with no
#   custom part and no duplicate-pad-number games.
#
# ROUTING NOTE (why the gate takes a detour): west of Q21 the gate lane sits NORTH of the source
# lane, but on a SOIC-8 the gate is pin 4 -- the SOUTHERN-most left pad, below all three source
# pads. That inverts the order, so exactly one crossing is topologically forced. It cannot be done
# on F.Cu (the 0.7 mm corridors between C15/R67/R68 are too narrow) and it cannot cross just south
# of the rail either, because In2 carries a board-wide horizontal signal bus (CS_IMU y=51.564,
# SENS_DET 52.374, INT_IMU 53.183, MISO 54.038) whose 0.81 mm gaps cannot take a via. So PYRO_G
# drops to In1, runs south past the whole bus, and comes back up in the empty F.Cu area below Q21.
# It is a 67 uA signal; the detour costs nothing and keeps the arm net well clear of the rail.
import hashlib, os, re, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCH = os.path.join(HERE, "Impulse_2.3.kicad_sch")
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
PRO = os.path.join(HERE, "Impulse_2.3.kicad_pro")
SYMLIB = r"C:/Program Files/KiCad/10.0/share/kicad/symbols/Transistor_FET.kicad_sym"
FPLIB = r"C:/Program Files/KiCad/10.0/share/kicad/footprints/Package_SO.pretty/SOIC-8_3.9x4.9mm_P1.27mm.kicad_mod"

FP_ID = "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm"
Q21_UUID_SCH = "6b3c6dcd-5867-00c8-11bd-41549ad6f177"
QX, QY = 167.895, 49.984          # re-centred in the R67..C10 window (0.48 mm either side)
PAD_NET = {"1": "7.4V_RAW", "2": "7.4V_RAW", "3": "7.4V_RAW",
           "4": "PYRO_G",
           "5": "7.4V", "6": "7.4V", "7": "7.4V", "8": "7.4V"}


def must(c, m):
    if not c:
        print("FAIL:", m)
        sys.exit(1)


def u(tag):
    h = hashlib.md5(("pyroarm23/" + tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


def load(p):
    t = open(p, encoding="utf-8", newline="").read()
    return t.replace("\r\n", "\n"), "\r\n" in t


def save(p, t, crlf):
    open(p, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)


def bal(text, start):
    d = 0
    for j in range(start, len(text)):
        if text[j] == "(":
            d += 1
        elif text[j] == ")":
            d -= 1
            if d == 0:
                return j + 1


for f in (SCH, PCB):
    if not os.path.exists(f + ".pre_arm"):
        shutil.copy2(f, f + ".pre_arm")

# ============================== SCHEMATIC ==============================
t, crlf = load(SCH)

if '(symbol "Transistor_FET:AO4407A"' not in t:
    lib = open(SYMLIB, encoding="utf-8").read()
    m = re.search(r'\n\t\(symbol "IRF7404"', lib)
    must(m, "IRF7404 not in Transistor_FET lib")
    body = lib[m.start() + 2:bal(lib, m.start() + 2)]
    body = body.replace('(symbol "IRF7404"', '(symbol "Transistor_FET:AO4407A"', 1)
    body = body.replace('(symbol "IRF7404_0_1"', '(symbol "AO4407A_0_1"', 1)
    body = body.replace('(symbol "IRF7404_1_1"', '(symbol "AO4407A_1_1"', 1)
    for k, v in (("Value", "AO4407A"),
                 ("Footprint", FP_ID),
                 ("Datasheet", "https://www.aosmd.com/res/data_sheets/AO4407A.pdf"),
                 ("Description", "-12A Id, -30V Vds, P-Channel MOSFET, 17mOhm Ron @ Vgs=-6V, SO-8")):
        body = re.sub(r'\(property "%s" "[^"]*"' % k, '(property "%s" "%s"' % (k, v), body, count=1)
    body = "\n".join("\t" + ln if ln else ln for ln in body.split("\n"))
    anchor = '\t\t(symbol "power:GND"'
    must(anchor in t, "power:GND anchor for lib_symbols insert")
    t = t.replace(anchor, body + "\n" + anchor, 1)
    print("sch: embedded Transistor_FET:AO4407A (cloned from IRF7404)")
else:
    print("sch: AO4407A lib symbol already present")

up = t.find('(uuid "%s")' % Q21_UUID_SCH)
must(up != -1, "Q21 symbol uuid not found in schematic")
sp = t.rfind("\n\t(symbol\n", 0, up) + 2
en = bal(t, sp)
blk = t[sp:en]
must('(property "Reference" "Q21"' in blk, "Q21 reference not in located block")
if 'Transistor_FET:AO3401A' in blk:
    nb = blk.replace('(lib_id "Transistor_FET:AO3401A")', '(lib_id "Transistor_FET:AO4407A")', 1)
    nb = nb.replace('(property "Value" "AO3401A"', '(property "Value" "AO4407A"', 1)
    nb = nb.replace('(property "Footprint" "Package_TO_SOT_SMD:SOT-23"',
                    '(property "Footprint" "%s"' % FP_ID, 1)
    pins8 = "".join('\t\t(pin "%d"\n\t\t\t(uuid "%s")\n\t\t)\n' % (n, u("Q21/pin%d" % n)) for n in range(1, 9))
    m = re.search(r'\t\t\(pin "1"\n[\s\S]*?\n(\t\t\(instances)', nb)
    must(m, "Q21 3-pin block not found")
    nb = nb[:m.start()] + pins8 + m.group(1) + nb[m.end():]
    must(nb.count('(pin "') == 8, "Q21 pin count wrong after swap: %d" % nb.count('(pin "'))
    t = t[:sp] + nb + t[en:]
    print("sch: Q21 -> AO4407A / %s, 3 pins -> 8" % FP_ID)
else:
    must('Transistor_FET:AO4407A' in blk, "Q21 lib_id is neither AO3401A nor AO4407A")
    print("sch: Q21 already swapped")

save(SCH, t, crlf)

# ================================= PCB =================================
t, crlf = load(PCB)

# ---- 1. replace the Q21 footprint block with a SOIC-8 -------------------------------------
fpsrc = open(FPLIB, encoding="utf-8").read()
gm = re.search(r'\(footprint "SOIC-8_3\.9x4\.9mm_P1\.27mm"', fpsrc)
must(gm, "stock SOIC-8 footprint not found")
graphics = []
pads = []
i = fpsrc.find("(", gm.end())
# walk the top-level children of the library footprint
depth_start = fpsrc.index("(", gm.start())
end = bal(fpsrc, depth_start)
j = gm.end()
while j < end:
    k = fpsrc.find("(", j)
    if k == -1 or k >= end:
        break
    e2 = bal(fpsrc, k)
    if e2 is None or e2 > end:
        break
    child = fpsrc[k:e2]
    tag = child[1:].split()[0].split("\n")[0]
    if tag in ("fp_line", "fp_poly", "fp_circle", "fp_arc", "fp_rect", "fp_text"):
        graphics.append(child)
    elif tag == "pad":
        pads.append(child)
    j = e2
must(len(pads) == 8, "expected 8 SOIC-8 pads, got %d" % len(pads))

def reindent(block, tabs):
    lines = block.split("\n")
    out = [lines[0]]
    for ln in lines[1:]:
        out.append("\t" * tabs + ln.lstrip("\t") if ln.strip() else ln)
    return ("\t" * tabs) + "\n".join(out)

for m in re.finditer(r'\(footprint "', t):
    e = bal(t, m.start())
    blk = t[m.start():e]
    if '(property "Reference" "Q21"' in blk:
        q_start, q_end, q_blk = m.start(), e, blk
        break
else:
    must(False, "Q21 footprint not found on the board")

if 'SOIC-8' in q_blk:
    print("pcb: Q21 already SOIC-8 -- skipping footprint replace")
else:
    fp_uuid = re.search(r'\(uuid "([^"]+)"\)', q_blk).group(1)
    path = re.search(r'\(path "([^"]*)"\)', q_blk).group(1)
    ref_prop = re.search(r'(\(property "Reference" "Q21"[\s\S]*?\n\t\t\))', q_blk).group(1)
    val_prop = re.search(r'(\(property "Value" "[^"]*"[\s\S]*?\n\t\t\))', q_blk).group(1)
    val_prop = re.sub(r'\(property "Value" "[^"]*"', '(property "Value" "AO4407A"', val_prop, count=1)

    parts = ['\t(footprint "%s"' % FP_ID,
             '\t\t(layer "F.Cu")',
             '\t\t(uuid "%s")' % fp_uuid,
             '\t\t(at %s %s)' % (QX, QY),
             '\t\t(descr "SOIC, 8 Pin (3.9x4.9mm body, 1.27mm pitch)")',
             '\t\t(tags "SOIC SO")',
             "\t\t" + ref_prop,
             "\t\t" + val_prop,
             '\t\t(property "Datasheet" "https://www.aosmd.com/res/data_sheets/AO4407A.pdf"\n'
             '\t\t\t(at 0 0 0)\n\t\t\t(layer "F.Fab")\n\t\t\t(hide yes)\n'
             '\t\t\t(uuid "%s")\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n'
             '\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)' % u("Q21/dsprop"),
             '\t\t(property "Description" "P-channel MOSFET, pyro rail ARM switch"\n'
             '\t\t\t(at 0 0 0)\n\t\t\t(layer "F.Fab")\n\t\t\t(hide yes)\n'
             '\t\t\t(uuid "%s")\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n'
             '\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)' % u("Q21/descprop"),
             '\t\t(path "%s")' % path,
             '\t\t(sheetname "/")',
             '\t\t(sheetfile "Impulse_2.3.kicad_sch")',
             '\t\t(attr smd)']
    for g in graphics:
        gg = g
        gu = re.search(r'\(uuid "([^"]+)"\)', gg)
        if gu:
            gg = gg.replace(gu.group(1), u("Q21/g%d" % graphics.index(g)), 1)
        else:
            gg = gg[:-1].rstrip() + '\n\t\t\t(uuid "%s")\n\t\t)' % u("Q21/g%d" % graphics.index(g))
        parts.append(reindent(gg, 2))
    for p in pads:
        num = re.search(r'\(pad "(\d+)"', p).group(1)
        net = PAD_NET[num]
        pp = p.rstrip()
        must(pp.endswith(")"), "pad block malformed")
        pp = pp[:-1].rstrip() + '\n\t\t\t(net "%s")\n\t\t\t(uuid "%s")\n\t\t)' % (net, u("Q21/pad%s" % num))
        parts.append(reindent(pp, 2))
    parts.append('\t\t(embedded_fonts no)')
    parts.append('\t\t(model "${KICAD10_3DMODEL_DIR}/Package_SO.3dshapes/SOIC-8_3.9x4.9mm_P1.27mm.step"\n'
                 '\t\t\t(offset\n\t\t\t\t(xyz 0 0 0)\n\t\t\t)\n'
                 '\t\t\t(scale\n\t\t\t\t(xyz 1 1 1)\n\t\t\t)\n'
                 '\t\t\t(rotate\n\t\t\t\t(xyz 0 0 0)\n\t\t\t)\n\t\t)')
    parts.append("\t)")
    newblk = "\n".join(parts)
    t = t[:q_start - 1] + newblk + t[q_end:]
    print("pcb: Q21 footprint -> SOIC-8 at (%s, %s), 8 pads netted" % (QX, QY))

save(PCB, t, crlf)
print("\nstage 1 done (symbol + footprint). Copper rework is in _reroute_pyro_arm.py")
