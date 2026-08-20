# Refill the copper pours after the pyro-arm rework.
# Run with KiCad's interpreter:
#   "C:/Program Files/KiCad/10.0/bin/python.exe" _refill_zones.py
# New tracks/vias do not update the stored zone fills, so DRC reports the pours overlapping the new
# copper at 0.000 mm until they are re-poured. Nothing is Remove()d here -- a b.Remove() loop
# followed by SaveBoard silently dropped all four GND zone definitions once before on this project.
import os, shutil, sys, pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")

bak = PCB + ".pre_fill"
if not os.path.exists(bak):
    shutil.copy2(PCB, bak)

b = pcbnew.LoadBoard(PCB)
zones = b.Zones()
print("zones on board: %d" % len(zones))
for z in zones:
    print("   net=%-10s layers=%s priority=%s" % (
        z.GetNetname(), [b.GetLayerName(l) for l in z.GetLayerSet().Seq()], z.GetAssignedPriority()))

filler = pcbnew.ZONE_FILLER(b)
ok = filler.Fill(zones)
print("fill returned:", ok)
pcbnew.SaveBoard(PCB, b)

b2 = pcbnew.LoadBoard(PCB)
print("after save: %d zones still present" % len(b2.Zones()))
if len(b2.Zones()) != len(zones):
    print("FAIL: zone count changed on save -- restore from", bak)
    sys.exit(1)
print("zones refilled")
