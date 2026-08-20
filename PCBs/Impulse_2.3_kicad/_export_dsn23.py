# Export the board to Specctra DSN for freerouting.
# Run with KiCad's interpreter.
import sys, pcbnew
pcb, out = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(pcb)
ok = pcbnew.ExportSpecctraDSN(b, out)
print("ExportSpecctraDSN ->", ok, out)
