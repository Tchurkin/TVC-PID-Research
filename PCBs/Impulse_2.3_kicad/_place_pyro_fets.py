# Swap the five pyro FET footprints on the PCB to the 5x6 packages and place them.
#
# Uses pcbnew (not text surgery) because these footprints are flipped onto B.Cu and carry a custom
# polygon tab pad; predicting the mirrored geometry by hand is exactly the class of error that has
# already bitten this rework twice. ONE footprint per process invocation -- repeated
# FootprintLoad()/Duplicate() in a single interpreter start returning untyped SwigPyObjects.
#
# Usage: python _place_pyro_fets.py Q16
import os, sys, shutil, gc
import pcbnew

gc.disable()
HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
LIB = os.path.join(HERE, "Impulse22.pretty")

# ref -> (footprint, x, y, rotation, {pad -> net})
# Q16-Q19 shift SOUTH to y=47.2 so the 4.7 mm-tall body clears the P#O through-hole pads
# (bottom edge 44.336). Q17/Q16 also shift EAST: the PI battery header (THT, x 155.5-157.2,
# y 49.3-51.0) sits exactly where Q17 would otherwise land.
PLACE = {
    # rot 270 puts the DRAIN TAB north -- directly overlapping the connector's pin-1 pad, same net,
    # so the firing current gets a zero-length connection -- and the GATE south-east, facing the
    # PYROn traces that come up from the Teensy. Q17 shifts east off its connector because the PI
    # battery header (THT, x 153.0-157.2) occupies its natural spot.
    "Q18": ("TI_SON5x6_Q5A", 138.598, 45.500, 270, {"1": "Net-(P4O-Pin_1)", "5": "PYRO4"}),
    "Q19": ("TI_SON5x6_Q5A", 147.996, 45.500, 270, {"1": "Net-(P3O-Pin_1)", "5": "PYRO3"}),
    "Q17": ("TI_SON5x6_Q5A", 160.000, 45.500, 270, {"1": "Net-(P2O-Pin_1)", "5": "PYRO2"}),
    "Q16": ("TI_SON5x6_Q5A", 166.792, 45.500, 270, {"1": "Net-(P1O-Pin_1)", "5": "PYRO1"}),
    # Q21 rotates 90 deg: leads (source+gate) face SOUTH into the empty F.Cu area, tab (drain)
    # faces NORTH toward P1O. That buys ~1.4 mm side clearance vs the 0.53 mm an unrotated part
    # would leave between R67 and C10.
    "Q21": ("AON6403_DFN5x6", 167.795, 49.984, 90, {"4": "PYRO_G", "5": "7.4V"}),
    # Q20, the MAIN battery switch, gets the same AON6403 as Q21 -- same part number, so it shares
    # Q21's feeder and adds no BOM line. It is the single point of failure for the whole avionics
    # stack (the Teensy that fires the chute is powered through it), and as an AO3401A a sustained
    # multi-servo stall put it at ~0.43 W in SOT-23 => Tj ~135 C CONTINUOUSLY. At 4.3 mOhm it is
    # ~0.03 W instead.
    # It cannot stay in its old pocket: the window between the BUZZER's through-hole pads (bottom
    # edge 78.44) and R66 (top edge 82.96) is 4.52 mm and this package needs 4.96 + clearance.
    # Moving EAST of the buzzer costs nothing -- the VBAT drain track already runs that way to the
    # bucks, so the drain gets shorter and only VBAT_RAW lengthens.
    "Q20": ("AON6403_DFN5x6", 175.700, 79.500, 0, {"4": "MAIN_G", "5": "VBAT"}),
}
SRC_NET = {"Q16": "GND", "Q17": "GND", "Q18": "GND", "Q19": "GND", "Q21": "7.4V_RAW",
           "Q20": "VBAT_RAW"}
SRC_PADS = {"Q16": ["6", "7", "8"], "Q17": ["6", "7", "8"], "Q18": ["6", "7", "8"],
            "Q19": ["6", "7", "8"], "Q21": ["1", "2", "3"], "Q20": ["1", "2", "3"]}


def must(c, m):
    if not c:
        print("FAIL:", m)
        sys.exit(1)


ref = sys.argv[1]
must(ref in PLACE, "unknown ref " + ref)
fpname, x, y, rot, netmap = PLACE[ref]
# probe mode: python _place_pyro_fets.py Q16 --probe <rot> [<x> <y>]  -> report geometry, save nothing
PROBE = "--probe" in sys.argv
if PROBE:
    i = sys.argv.index("--probe")
    rot = float(sys.argv[i + 1])
    if len(sys.argv) > i + 3:
        x, y = float(sys.argv[i + 2]), float(sys.argv[i + 3])
netmap = dict(netmap)
for p in SRC_PADS[ref]:
    netmap[p] = SRC_NET[ref]

if not os.path.exists(PCB + ".pre_upsize"):
    shutil.copy2(PCB, PCB + ".pre_upsize")

b = pcbnew.LoadBoard(PCB)
old = b.FindFootprintByReference(ref)
must(old, "footprint %s not on board" % ref)
if old.GetFPIDAsString() == "Impulse22:" + fpname and not PROBE and "--force" not in sys.argv:
    print("%s already %s -- skip" % (ref, fpname))
    sys.exit(0)
flip = old.IsFlipped()
kpath = old.GetPath()
b.Remove(old)

plug = pcbnew.PCB_IO_MGR.FindPlugin(pcbnew.PCB_IO_MGR.GuessPluginTypeFromLibPath(LIB))
fp = plug.FootprintLoad(LIB, fpname, False)
must(hasattr(fp, "SetReference"), "%s loaded untyped" % fpname)
fp.SetFPIDAsString("Impulse22:" + fpname)
b.Add(fp)
fp.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
if flip:
    fp.Flip(fp.GetPosition(), True)
fp.SetOrientationDegrees(rot)
fp.SetReference(ref)
fp.SetValue("CSD17301Q5A" if fpname.startswith("TI_") else "AON6403")
fp.SetPath(kpath)

for num, netname in netmap.items():
    net = b.FindNet(netname)
    must(net is not None and net.GetNetCode() > 0, "net missing: " + netname)
    hit = 0
    for pd in fp.Pads():
        if pd.GetNumber() == num:
            pd.SetNet(net)
            hit += 1
    must(hit, "%s: no pad numbered %s" % (ref, num))
for pd in fp.Pads():
    if pd.GetNumber():
        must(pd.GetNet().GetNetCode() > 0, "%s pad %s unnetted" % (ref, pd.GetNumber()))

if not PROBE:
    pcbnew.SaveBoard(PCB, b)
print("%s -> %s at (%.3f, %.3f) rot=%d %s" % (ref, fpname, x, y, rot, "B.Cu" if flip else "F.Cu"))
for pd in fp.Pads():
    if pd.GetNumber():
        bb = pd.GetBoundingBox()
        print("   pad %-2s %-16s x[%7.3f..%7.3f] y[%7.3f..%7.3f]"
              % (pd.GetNumber(), pd.GetNetname(), bb.GetLeft() / 1e6, bb.GetRight() / 1e6,
                 bb.GetTop() / 1e6, bb.GetBottom() / 1e6))
