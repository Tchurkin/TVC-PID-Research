# Upsize the whole pyro switching path for a 0.3 ohm initiator (~14-20 A), not the 2-4 A the board
# was designed around.
#
#   Q21  arm   AO4407A  SOIC-8      -> AON6403     DFN 5x6   LCSC C2760089
#   Q16-Q19    CSD17313Q2 SON 2x2   -> CSD17301Q5A SON 5x6   LCSC C129940
#
# WHY: at 0.3 ohm the loop runs 14-20 A. Held for the firmware's 900 ms that is 11.5 W in a
# CSD17313Q2 (package rated 3 W) and was 16.5 W in the original SOT-23 AO3401A, which is what
# destroyed it. AON6403 is 4.3 mOhm max at VGS=-4.5V (we drive -6.7 V) -> 2.2 W, survives the pulse
# INDEFINITELY. CSD17301Q5A is 2.9 mOhm GUARANTEED AT VGS=3V -- that 3 V row is the hard constraint,
# because the Teensy drives these gates directly with no driver -- -> ~2.1 W, Tj ~48 C, which MATCHES
# the arm FET rather than leaving the firing side as the weak half (an interim CSD17302Q5A at
# 9.5 mOhm would have run 5.3 W / Tj ~84 C). It also carries 414 mJ EAS vs 61 mJ, which is the
# margin that absorbs the inductive kick when the nichrome opens ~20 A.
#
# Symbol pin numbering differs between the two packages and both differ from what they replace:
#   AON6403       leads 1,2,3=SOURCE 4=GATE ; tab 5=DRAIN
#   CSD17301Q5A   tab 1=DRAIN ; leads 5=GATE 6,7,8=SOURCE
# Connectivity is preserved regardless, because every candidate symbol places G/S/D at the SAME
# endpoints (G=(-5.08,0), S=(2.54,-5.08), D=(2.54,5.08)) -- the same property that made the earlier
# AO3401A->AO4407A and IRLML6244->CSD17313Q2 swaps safe. Only pin NUMBERS change.
import hashlib, os, re, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCH = os.path.join(HERE, "Impulse_2.3.kicad_sch")

NEW = {
    "AON6403": dict(
        src="Transistor_FET:AO4407A",
        value="AON6403",
        fp="Impulse22:AON6403_DFN5x6",
        ds="https://www.aosmd.com/res/datasheets/AON6403.pdf",
        desc="-85A Id, -30V Vds, P-Channel MOSFET, 4.3mOhm Ron @ Vgs=-4.5V, DFN5x6",
        pins=[("S", "1"), ("S", "2"), ("S", "3"), ("G", "4"), ("D", "5")],
    ),
    "CSD17301Q5A": dict(
        src="Transistor_FET:CSD17313Q2",
        value="CSD17301Q5A",
        fp="Impulse22:TI_SON5x6_Q5A",
        ds="https://www.ti.com/lit/gpn/CSD17301Q5A",
        desc="100A Id, 30V Vds, N-Channel NexFET, 2.9mOhm Ron @ Vgs=3V, 414mJ EAS, SON5x6",
        pins=[("D", "1"), ("G", "5"), ("S", "6"), ("S", "7"), ("S", "8")],
    ),
}
# Q20 (MAIN battery switch) joins the AON6403 list: same part as Q21, so it shares the feeder and
# adds no BOM line. It is the single point of failure for the whole avionics stack -- the Teensy
# that fires the chute is powered through it -- and as an AO3401A a sustained multi-servo stall sat
# it at ~0.43 W in SOT-23, Tj ~135 C CONTINUOUSLY. At 4.3 mOhm that becomes ~0.03 W.
# Note its instance is still AO3401A (it never went through the AO4407A step Q21 did); the lib_id
# rewrite below is source-agnostic, so that is fine.
INSTANCES = [("Q20", "6c77b041-c619-e4c7-1fc6-de87bcbc2dff", "AON6403"),
             ("Q21", "6b3c6dcd-5867-00c8-11bd-41549ad6f177", "AON6403"),
             ("Q16", "2c53e37c-246a-4b81-8821-3788ebd2e8f6", "CSD17301Q5A"),
             ("Q17", "698b7dc0-4267-4751-b8ae-e69c2a47eeed", "CSD17301Q5A"),
             ("Q18", "2bbc76a6-08e5-4dbf-ba64-5953d019f3db", "CSD17301Q5A"),
             ("Q19", "e5250c35-b0e4-4ae9-8d0a-7674e96668ef", "CSD17301Q5A")]


def must(c, m):
    if not c:
        print("FAIL:", m)
        sys.exit(1)


def u(tag):
    h = hashlib.md5(("upsize23/" + tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


def bal(s, i):
    d = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            d += 1
        elif s[j] == ")":
            d -= 1
            if d == 0:
                return j + 1


t = open(SCH, encoding="utf-8", newline="").read()
crlf = "\r\n" in t
t = t.replace("\r\n", "\n")
if not os.path.exists(SCH + ".pre_upsize"):
    shutil.copy2(SCH, SCH + ".pre_upsize")

# ---------- 1. embed the two new library symbols ----------
for name, spec in NEW.items():
    if '(symbol "Transistor_FET:%s"' % name in t:
        print("sch: %s lib symbol already present" % name)
        continue
    m = re.search(r'\t\t\(symbol "%s"' % re.escape(spec["src"]), t)
    must(m, "source lib symbol %s not embedded" % spec["src"])
    body = t[m.start() + 2:bal(t, m.start() + 2)]

    # harvest one pin block per terminal letter, from the *_1_1 sub-symbol
    pin_blocks = {}
    for pm in re.finditer(r"\t\t\t\(pin \w+ \w+\n", body):
        blk = body[pm.start() + 3:bal(body, pm.start() + 3)]
        nm = re.search(r'\(name "([^"]*)"', blk)
        if nm and nm.group(1) not in pin_blocks:
            pin_blocks[nm.group(1)] = blk
    for need in {p[0] for p in spec["pins"]}:
        must(need in pin_blocks, "%s: no source pin block named %s" % (name, need))

    newpins = []
    for term, num in spec["pins"]:
        b = pin_blocks[term]
        b = re.sub(r'\(number "[^"]*"', '(number "%s"' % num, b, count=1)
        b = re.sub(r'\(uuid "[^"]*"\)', '(uuid "%s")' % u("%s/pin%s" % (name, num)), b, count=1)
        newpins.append("\t\t\t" + b)
    # replace the whole pin list inside the body
    first = re.search(r"\t\t\t\(pin \w+ \w+\n", body)
    last_end = 0
    for pm in re.finditer(r"\t\t\t\(pin \w+ \w+\n", body):
        last_end = bal(body, pm.start() + 3)
    body = body[:first.start()] + "\n".join(newpins) + "\n" + body[last_end + 1:]

    old_lib = spec["src"].split(":", 1)[1]
    body = body.replace('(symbol "%s"' % spec["src"], '(symbol "Transistor_FET:%s"' % name, 1)
    body = body.replace('(symbol "%s_0_1"' % old_lib, '(symbol "%s_0_1"' % name, 1)
    body = body.replace('(symbol "%s_1_1"' % old_lib, '(symbol "%s_1_1"' % name, 1)
    for k, v in (("Value", spec["value"]), ("Footprint", spec["fp"]),
                 ("Datasheet", spec["ds"]), ("Description", spec["desc"])):
        body = re.sub(r'\(property "%s" "[^"]*"' % k, '(property "%s" "%s"' % (k, v), body, count=1)
    anchor = '\t\t(symbol "power:GND"'
    must(anchor in t, "power:GND anchor")
    t = t.replace(anchor, body + "\n" + anchor, 1)
    print("sch: embedded Transistor_FET:%s (pins %s)"
          % (name, ",".join(n for _, n in spec["pins"])))

# ---------- 2. retarget the instances ----------
for ref, uuid, name in INSTANCES:
    spec = NEW[name]
    up = t.find('(uuid "%s")' % uuid)
    must(up != -1, "%s symbol uuid not found" % ref)
    sp = t.rfind("\n\t(symbol\n", 0, up) + 2
    en = bal(t, sp)
    blk = t[sp:en]
    must('(property "Reference" "%s"' % ref in blk, "%s reference mismatch" % ref)
    if '(lib_id "Transistor_FET:%s")' % name in blk:
        print("sch: %s already %s" % (ref, name))
        continue
    nb = re.sub(r'\(lib_id "Transistor_FET:[^"]+"\)',
                '(lib_id "Transistor_FET:%s")' % name, blk, count=1)
    nb = re.sub(r'\(property "Value" "[^"]*"',
                '(property "Value" "%s"' % spec["value"], nb, count=1)
    nb = re.sub(r'\(property "Footprint" "[^"]*"',
                '(property "Footprint" "%s"' % spec["fp"], nb, count=1)
    pins = "".join('\t\t(pin "%s"\n\t\t\t(uuid "%s")\n\t\t)\n' % (n, u("%s/ipin%s" % (ref, n)))
                   for _, n in spec["pins"])
    m = re.search(r'\t\t\(pin "[^"]+"\n[\s\S]*?\n(\t\t\(instances)', nb)
    must(m, "%s instance pin list not found" % ref)
    nb = nb[:m.start()] + pins + m.group(1) + nb[m.end():]
    must(nb.count('(pin "') == len(spec["pins"]),
         "%s pin count %d != %d" % (ref, nb.count('(pin "'), len(spec["pins"])))
    t = t[:sp] + nb + t[en:]
    print("sch: %s -> %s / %s (pins %s)"
          % (ref, spec["value"], spec["fp"], ",".join(n for _, n in spec["pins"])))

open(SCH, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
print("\nschematic saved")
