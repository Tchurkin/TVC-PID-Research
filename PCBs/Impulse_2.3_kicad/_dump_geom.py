# Dump pad / track / via geometry from pcbnew itself, so the hand-routing checker never has to
# guess KiCad's footprint rotation-and-flip convention (getting that sign wrong silently inverts
# which pad is which net, which is exactly the kind of error that ends up as a short on a real board).
# Run with KiCad's interpreter:
#   "C:/Program Files/KiCad/10.0/bin/python.exe" _dump_geom.py Impulse_2.3.kicad_pcb geom.json
import json, sys, pcbnew

pcb, out = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(pcb)
MM = 1e-6

data = {"pads": [], "tracks": [], "vias": [], "layers": {}}
for i in range(pcbnew.PCB_LAYER_ID_COUNT):
    n = b.GetLayerName(i)
    if n:
        data["layers"][str(i)] = n

for fp in b.GetFootprints():
    ref = fp.GetReference()
    for p in fp.Pads():
        pos = p.GetPosition()
        sz = p.GetSize()
        lset = [b.GetLayerName(l) for l in p.GetLayerSet().Seq()]
        data["pads"].append({
            "ref": ref, "num": p.GetNumber(), "net": p.GetNetname(),
            "x": pos.x * MM, "y": pos.y * MM,
            "w": sz.x * MM, "h": sz.y * MM,
            "ang": p.GetOrientationDegrees(),
            "shape": int(p.GetShape()),
            "layers": [l for l in lset if l.endswith(".Cu")],
        })

for tr in b.GetTracks():
    if isinstance(tr, pcbnew.PCB_VIA):
        pos = tr.GetPosition()
        data["vias"].append({"net": tr.GetNetname(), "x": pos.x * MM, "y": pos.y * MM,
                             "d": tr.GetDrillValue() * MM,
                             "size": tr.GetWidth(pcbnew.F_Cu) * MM if hasattr(tr, "GetWidth") else 0.6})
    else:
        s, e = tr.GetStart(), tr.GetEnd()
        data["tracks"].append({"net": tr.GetNetname(), "layer": b.GetLayerName(tr.GetLayer()),
                               "x1": s.x * MM, "y1": s.y * MM, "x2": e.x * MM, "y2": e.y * MM,
                               "w": tr.GetWidth() * MM})

json.dump(data, open(out, "w"), indent=0)
print("pads=%d tracks=%d vias=%d -> %s" % (len(data["pads"]), len(data["tracks"]), len(data["vias"]), out))
