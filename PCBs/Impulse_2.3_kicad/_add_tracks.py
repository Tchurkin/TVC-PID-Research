# Append verified tracks to the board by TEXT surgery.
#
# Every track is clearance-checked against live geometry BEFORE it is written, and refused if it
# fails. Text surgery rather than pcbnew because LoadBoard/Remove/Add/SaveBoard is not a safe no-op
# on this board (it silently displaced Q16-Q19 once already).
import hashlib, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _geom
from _size_by_current import edge_ok

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")


def uid(tag):
    h = hashlib.md5(("addtrk23/" + tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


def add(tracks, tag, backup=None):
    """tracks: list of (net, layer, x1, y1, x2, y2, w). Returns count written."""
    b = _geom.Board(os.path.join(HERE, "geom.json"), os.path.join(HERE, "Impulse_2.3.kicad_pro"))
    good = []
    for (net, lay, x1, y1, x2, y2, w) in tracks:
        ok = b.seg_ok(net, lay, x1, y1, x2, y2, w, report=True)
        eok = edge_ok(x1, y1, x2, y2, w)
        print("  %-18s %-7s w=%.2f (%.3f,%.3f)->(%.3f,%.3f)  clr=%s edge=%s"
              % (net, lay, w, x1, y1, x2, y2, "OK" if ok else "FAIL", "OK" if eok else "FAIL"))
        if ok and eok:
            good.append((net, lay, x1, y1, x2, y2, w))
            b.tracks.append({"net": net, "layer": lay, "x1": x1, "y1": y1,
                             "x2": x2, "y2": y2, "w": w})
    if not good:
        print("nothing to write")
        return 0
    if backup and not os.path.exists(PCB + backup):
        import shutil
        shutil.copy2(PCB, PCB + backup)
    t = open(PCB, encoding="utf-8", newline="").read()
    crlf = "\r\n" in t
    t = t.replace("\r\n", "\n")
    anchor = t.rindex("\n\t(segment")
    end = t.index("\n\t)", anchor) + 3
    blocks = []
    for i, (net, lay, x1, y1, x2, y2, w) in enumerate(good):
        blocks.append('\n\t(segment\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(width %g)\n'
                      '\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                      % (x1, y1, x2, y2, w, lay, net, uid("%s/%d" % (tag, i))))
    t = t[:end] + "".join(blocks) + t[end:]
    open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
    print("wrote %d/%d tracks" % (len(good), len(tracks)))
    return len(good)
