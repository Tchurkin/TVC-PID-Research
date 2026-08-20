# Clearance checker for hand-routing Impulse 2.3.
#
# Geometry comes from _dump_geom.py (pcbnew itself) rather than from parsing the s-expression, so
# footprint rotation/flip conventions are never guessed. That matters: an earlier hand-rolled parser
# got the rotation SIGN wrong and reported R67's two pads with their nets swapped -- the kind of
# error that turns into a real short on a real board.
#
# Clearances come from the .kicad_pro netclass patterns; a pair of nets needs max(clearance) and
# same-net pairs need nothing. Pads are treated as ORIENTED RECTANGLES, never as bounding circles:
# modelling a pad as a circle of radius max(w,h)/2 was the bug that wasted hours on the 2.2 routing
# (it makes big pads falsely block everything nearby).
import json, math

LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]


class Board:
    def __init__(self, geom_json, pro):
        d = json.load(open(geom_json))
        self.pads = d["pads"]
        self.tracks = d["tracks"]
        self.vias = d["vias"]
        p = json.load(open(pro))
        ns = p.get("net_settings", {})
        self.cls_clear = {c["name"]: c.get("clearance", 0.2) for c in ns.get("classes", [])}
        self.net_cls = {q["pattern"]: q["netclass"] for q in ns.get("netclass_patterns", [])}
        self.default_clear = self.cls_clear.get("Default", 0.2)

    def clearance(self, net):
        return self.cls_clear.get(self.net_cls.get(net, "Default"), self.default_clear)

    def pair(self, a, b):
        return 0.0 if a == b else max(self.clearance(a), self.clearance(b))

    @staticmethod
    def _pt_seg(px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        if L2 == 0:
            return math.hypot(px - x1, py - y1)
        u = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
        return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))

    @staticmethod
    def _pt_rect(px, py, cx, cy, w, h, ang):
        dx, dy = px - cx, py - cy
        r = math.radians(-ang)
        lx = dx * math.cos(r) - dy * math.sin(r)
        ly = dx * math.sin(r) + dy * math.cos(r)
        return math.hypot(max(abs(lx) - w / 2, 0.0), max(abs(ly) - h / 2, 0.0))

    @staticmethod
    def _seg_seg(ax, ay, bx, by, cx, cy, dx_, dy_, step=0.05):
        n = max(2, int(math.hypot(bx - ax, by - ay) / step) + 1)
        best = 1e9
        for i in range(n + 1):
            px = ax + (bx - ax) * i / n
            py = ay + (by - ay) * i / n
            best = min(best, Board._pt_seg(px, py, cx, cy, dx_, dy_))
        return best

    def via_ok(self, net, x, y, size=0.6, report=False):
        """A through via spans every copper layer, so check against ALL of them."""
        r = size / 2.0
        bad = []
        for p in self.pads:
            need = self.pair(net, p["net"])
            if need == 0:
                continue
            d = self._pt_rect(x, y, p["x"], p["y"], p["w"], p["h"], p["ang"]) - r
            if d < need:
                bad.append(("pad %s.%s/%s [%s]" % (p["ref"], p["num"], p["net"] or "-", ",".join(p["layers"])), d, need))
        for t in self.tracks:
            need = self.pair(net, t["net"])
            if need == 0:
                continue
            d = self._pt_seg(x, y, t["x1"], t["y1"], t["x2"], t["y2"]) - r - t["w"] / 2
            if d < need:
                bad.append(("track %s on %s" % (t["net"], t["layer"]), d, need))
        for v in self.vias:
            need = self.pair(net, v["net"])
            if need == 0:
                continue
            d = math.hypot(x - v["x"], y - v["y"]) - r - v["size"] / 2
            if d < need:
                bad.append(("via %s" % v["net"], d, need))
        if report:
            for w, d, need in sorted(bad, key=lambda z: z[1]):
                print("        X %-46s gap %+.3f  need %.3f" % (w, d, need))
        return not bad

    def seg_ok(self, net, layer, x1, y1, x2, y2, w, report=False):
        """Same-layer tracks, plus pads/vias that exist on this layer (or all layers)."""
        hw = w / 2.0
        bad = []
        for p in self.pads:
            need = self.pair(net, p["net"])
            if need == 0 or layer not in p["layers"]:
                continue
            n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
            best = 1e9
            for i in range(n + 1):
                px = x1 + (x2 - x1) * i / n
                py = y1 + (y2 - y1) * i / n
                best = min(best, self._pt_rect(px, py, p["x"], p["y"], p["w"], p["h"], p["ang"]))
            d = best - hw
            if d < need:
                bad.append(("pad %s.%s/%s" % (p["ref"], p["num"], p["net"] or "-"), d, need))
        for t in self.tracks:
            need = self.pair(net, t["net"])
            if need == 0 or t["layer"] != layer:
                continue
            d = self._seg_seg(x1, y1, x2, y2, t["x1"], t["y1"], t["x2"], t["y2"]) - hw - t["w"] / 2
            if d < need:
                bad.append(("track %s on %s" % (t["net"], t["layer"]), d, need))
        for v in self.vias:
            need = self.pair(net, v["net"])
            if need == 0:
                continue
            d = self._seg_seg(x1, y1, x2, y2, v["x"], v["y"], v["x"], v["y"]) - hw - v["size"] / 2
            if d < need:
                bad.append(("via %s" % v["net"], d, need))
        if report:
            for wd, d, need in sorted(bad, key=lambda z: z[1]):
                print("        X %-46s gap %+.3f  need %.3f" % (wd, d, need))
        return not bad
