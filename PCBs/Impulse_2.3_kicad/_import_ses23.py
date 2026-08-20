# Import the freerouting SES back into the board.
# Run with KiCad's interpreter, ALONE in its own process: ImportSpecctraSES mistypes every later
# b.GetFootprints()/Zones()/FindNet() call to an untyped SwigPyObject, so any cleanup must happen
# in a separate fresh-load process afterwards.
import os, shutil, sys, pcbnew
pcb, ses = sys.argv[1], sys.argv[2]
bak = pcb + ".pre_ses"
if not os.path.exists(bak):
    shutil.copy2(pcb, bak)
b = pcbnew.LoadBoard(pcb)
ok = pcbnew.ImportSpecctraSES(b, ses)
pcbnew.SaveBoard(pcb, b)
print("ImportSpecctraSES ->", ok)
