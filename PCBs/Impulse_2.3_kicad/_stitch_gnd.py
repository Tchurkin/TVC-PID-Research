# Bond the GND pour islands with stitching vias -- POUR-AWARE.
#
# A naive grid stitch fails badly here: `via_ok` proves a via clears other nets, but says nothing
# about whether GND copper actually EXISTS at that spot. Blindly gridding the board produced 16
# dangling vias sitting in voids and 10 copper-edge violations near the rim, and took DRC from
# 4 unconnected to 19. So: read the filled polygons back out of the board, and only place a via
# where it lands inside real pour on at least two layers, well clear of the board edge.
import hashlib, math, os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _geom

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
SIZE, DRILL = 0.8, 0.4
CX, CY, RAD = 152.7, 62.9, 35.0
EDGE_MARGIN = 1.2
PITCH = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0


def bal(s, i):
    d = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            d += 1
        elif s[j] == ")":
            d -= 1
            if d == 0:
                return j + 1


def islands(text):
    """-> list of (layer, [(x,y), ...]) filled polygons"""
    out = []
    for m in re.finditer(r"^\t\(zone\b", text, re.M):
        e = bal(text, m.start() + 1)
        z = text[m.start():e]
        for fm in re.finditer(r"\(filled_polygon\b", z):
            fe = bal(z, fm.start())
            fp = z[fm.start():fe]
            lay = re.search(r'\(layer "([^"]+)"\)', fp)
            pts = [(float(a), float(b)) for a, b in re.findall(r"\(xy ([-\d.]+) ([-\d.]+)\)", fp)]
            if lay and len(pts) > 2:
                out.append((lay.group(1), pts))
    return out


def inside(pt, poly):
    x, y = pt
    n = len(poly)
    c = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-15) + xi):
            c = not c
        j = i
    return c


def dist_to_edge(pt, poly):
    x, y = pt
    best = 1e18
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + u * dx), y - (ay + u * dy)))
    return best


def _run():
    text = open(PCB, encoding="utf-8", newline="").read().replace("\r\n", "\n")
    polys = islands(text)
    print("filled islands found: %d" % len(polys))
    by_layer = {}
    for lay, pts in polys:
        by_layer.setdefault(lay, []).append(pts)
    for lay, ps in sorted(by_layer.items()):
        print("   %-8s %d island(s)" % (lay, len(ps)))
    
    b = _geom.Board(os.path.join(HERE, "geom.json"), os.path.join(HERE, "Impulse_2.3.kicad_pro"))
    need = SIZE / 2.0 + 0.15          # via copper must sit inside the pour with a little margin
    
    placed = []
    y = CY - RAD
    while y <= CY + RAD:
        x = CX - RAD
        while x <= CX + RAD:
            if math.hypot(x - CX, y - CY) <= RAD - EDGE_MARGIN:
                hits = [lay for lay, pts in polys if inside((x, y), pts) and dist_to_edge((x, y), pts) > need]
                if len(set(hits)) >= 2 and b.via_ok("GND", x, y, SIZE):
                    placed.append((round(x, 3), round(y, 3)))
                    b.vias.append({"net": "GND", "x": x, "y": y, "size": SIZE, "d": DRILL})
            x += PITCH
        y += PITCH
    
    
    def u(tag):
        h = hashlib.md5(("stitch23/" + tag).encode()).hexdigest()
        return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])
    
    
    crlf = "\r\n" in open(PCB, encoding="utf-8", newline="").read()
    a = text.rindex("\n\t(via") if "\n\t(via" in text else text.rindex("\n\t(segment")
    end = text.index("\n\t)", a) + 3
    blocks = ['\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n\t\t(layers "F.Cu" "B.Cu")\n'
              '\t\t(net "GND")\n\t\t(uuid "%s")\n\t)' % (x, y, SIZE, DRILL, u("%s,%s" % (x, y)))
              for x, y in placed]
    text = text[:end] + "".join(blocks) + text[end:]
    open(PCB, "w", encoding="utf-8", newline="").write(text.replace("\n", "\r\n") if crlf else text)
    print("placed %d pour-aware GND stitching vias (%.1f mm grid)" % (len(placed), PITCH))


if __name__ == "__main__":
    _run()
