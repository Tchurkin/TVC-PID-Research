# Ask pcbnew for a footprint's real pad extents (the tab is a custom pad with its own rotation,
# so hand-computing its polygon bounds gets it wrong).
# Usage (ONE footprint per process -- repeated FootprintLoad returns untyped SwigPyObjects):
#   "C:/Program Files/KiCad/10.0/bin/python.exe" _fp_extent.py Impulse22.pretty AON6403_DFN5x6
import os, sys, pcbnew

lib, name = sys.argv[1], sys.argv[2]
lib = os.path.join(os.path.dirname(os.path.abspath(__file__)), lib)
plug = pcbnew.PCB_IO_MGR.FindPlugin(pcbnew.PCB_IO_MGR.GuessPluginTypeFromLibPath(lib))
fp = plug.FootprintLoad(lib, name, False)
MM = 1e-6
xs, ys = [], []
print("%s:" % name)
for p in fp.Pads():
    bb = p.GetBoundingBox()
    x0, y0 = bb.GetLeft() * MM, bb.GetTop() * MM
    x1, y1 = bb.GetRight() * MM, bb.GetBottom() * MM
    if p.GetNumber():
        xs += [x0, x1]
        ys += [y0, y1]
        print("   pad %-3s x[%7.3f..%7.3f] y[%7.3f..%7.3f]  (%.2f x %.2f)"
              % (p.GetNumber(), x0, x1, y0, y1, x1 - x0, y1 - y0))
print("   COPPER EXTENT x[%.3f..%.3f] y[%.3f..%.3f]  ->  %.2f x %.2f mm"
      % (min(xs), max(xs), min(ys), max(ys), max(xs) - min(xs), max(ys) - min(ys)))
