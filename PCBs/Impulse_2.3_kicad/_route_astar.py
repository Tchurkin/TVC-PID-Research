# Grid A* router for connections freerouting leaves behind.
#
# Rasterises every obstacle into a per-layer occupancy grid ONCE, then searches. Checking each
# candidate segment against ~770 tracks individually is far too slow for a 36 mm route -- a
# one-waypoint brute-force search over this board did not finish in two minutes.
#
# Lessons from this project's earlier hand-routing, baked in:
#   * pads are ORIENTED RECTANGLES, never bounding circles (a circle of radius max(w,h)/2 falsely
#     blocks everything near a large pad -- that bug cost hours)
#   * a via is a THROUGH via, so it must clear copper on all four layers
#   * simplify COLLINEAR-ONLY; line-of-sight simplification cuts corners across copper
#
# Usage: python _route_astar.py <net> <x1> <y1> <x2> <y2> [width] [via_size]
import heapq, json, math, sys
import numpy as np
import _geom

LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
GRID = 0.15
CX, CY, RAD = 152.7, 62.9, 35.0


def build(net, w, vs):
    b = _geom.Board("geom.json", "Impulse_2.3.kicad_pro")
    hw = w / 2.0
    x0, y0 = 116.0, 26.0
    x1, y1 = 190.0, 100.0
    nx = int((x1 - x0) / GRID) + 1
    ny = int((y1 - y0) / GRID) + 1
    occ = np.zeros((len(LAYERS), nx, ny), dtype=bool)
    viablk = np.zeros((nx, ny), dtype=bool)
    gx = x0 + np.arange(nx) * GRID
    gy = y0 + np.arange(ny) * GRID
    GX, GY = np.meshgrid(gx, gy, indexing="ij")

    def disc(arr, cx, cy, r):
        i0 = max(0, int((cx - r - x0) / GRID)); i1 = min(nx - 1, int((cx + r - x0) / GRID) + 1)
        j0 = max(0, int((cy - r - y0) / GRID)); j1 = min(ny - 1, int((cy + r - y0) / GRID) + 1)
        if i0 > i1 or j0 > j1:
            return
        d = (GX[i0:i1 + 1, j0:j1 + 1] - cx) ** 2 + (GY[i0:i1 + 1, j0:j1 + 1] - cy) ** 2
        arr[i0:i1 + 1, j0:j1 + 1] |= d <= r * r

    def rect(arr, cx, cy, rw, rh, ang, pad):
        rr = math.hypot(rw, rh) / 2 + pad
        i0 = max(0, int((cx - rr - x0) / GRID)); i1 = min(nx - 1, int((cx + rr - x0) / GRID) + 1)
        j0 = max(0, int((cy - rr - y0) / GRID)); j1 = min(ny - 1, int((cy + rr - y0) / GRID) + 1)
        if i0 > i1 or j0 > j1:
            return
        dx = GX[i0:i1 + 1, j0:j1 + 1] - cx
        dy = GY[i0:i1 + 1, j0:j1 + 1] - cy
        a = math.radians(-ang)
        lx = dx * math.cos(a) - dy * math.sin(a)
        ly = dx * math.sin(a) + dy * math.cos(a)
        ox = np.maximum(np.abs(lx) - rw / 2.0, 0.0)
        oy = np.maximum(np.abs(ly) - rh / 2.0, 0.0)
        arr[i0:i1 + 1, j0:j1 + 1] |= (ox * ox + oy * oy) <= pad * pad

    def seg(arr, ax, ay, bx, by, r):
        n = max(1, int(math.hypot(bx - ax, by - ay) / (GRID * 0.7)))
        for k in range(n + 1):
            disc(arr, ax + (bx - ax) * k / n, ay + (by - ay) * k / n, r)

    for p in b.pads:
        if p["net"] == net:
            continue
        clr = b.pair(net, p["net"] or "!unnamed")
        for li, L in enumerate(LAYERS):
            if L in p["layers"]:
                rect(occ[li], p["x"], p["y"], p["w"], p["h"], p["ang"], clr + hw)
        rect(viablk, p["x"], p["y"], p["w"], p["h"], p["ang"], clr + vs / 2.0)
    for t in b.tracks:
        if t["net"] == net or t["layer"] not in LAYERS:
            continue
        clr = b.pair(net, t["net"])
        li = LAYERS.index(t["layer"])
        seg(occ[li], t["x1"], t["y1"], t["x2"], t["y2"], t["w"] / 2.0 + clr + hw)
        seg(viablk, t["x1"], t["y1"], t["x2"], t["y2"], t["w"] / 2.0 + clr + vs / 2.0)
    for v in b.vias:
        if v["net"] == net:
            continue
        clr = b.pair(net, v["net"])
        for li in range(len(LAYERS)):
            disc(occ[li], v["x"], v["y"], v["size"] / 2.0 + clr + hw)
        disc(viablk, v["x"], v["y"], v["size"] / 2.0 + clr + vs / 2.0)

    outside = ((GX - CX) ** 2 + (GY - CY) ** 2) > (RAD - 0.5 - hw) ** 2
    for li in range(len(LAYERS)):
        occ[li] |= outside
    viablk |= ((GX - CX) ** 2 + (GY - CY) ** 2) > (RAD - 0.5 - vs / 2.0) ** 2
    viablk |= occ.any(axis=0)
    return b, occ, viablk, x0, y0, nx, ny


def route(net, a, bpt, w=0.3, vs=0.6):
    b, occ, viablk, x0, y0, nx, ny = build(net, w, vs)

    def cell(p):
        return (int(round((p[0] - x0) / GRID)), int(round((p[1] - y0) / GRID)))

    def endlayers(p):
        got = None
        for pd in b.pads:
            if pd["net"] != net:
                continue
            if (abs(pd["x"] - p[0]) - pd["w"] / 2 < 0.7) and (abs(pd["y"] - p[1]) - pd["h"] / 2 < 0.7):
                got = [LAYERS.index(L) for L in pd["layers"] if L in LAYERS]
        return got or list(range(len(LAYERS)))

    s, g = cell(a), cell(bpt)
    starts = [(l, s[0], s[1]) for l in endlayers(a)]
    goals = set((l, g[0], g[1]) for l in endlayers(bpt))
    dirs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    VIA = 10.0
    dist, prev, pq = {}, {}, []
    for st in starts:
        dist[st] = 0.0
        heapq.heappush(pq, (0.0, st, None))
    hit = None
    while pq:
        f, cur, par = heapq.heappop(pq)
        if cur in prev:
            continue
        prev[cur] = par
        if cur in goals:
            hit = cur
            break
        l, i, j = cur
        d0 = dist[cur]
        for di, dj, c in dirs:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny) or occ[l, ni, nj]:
                continue
            nd = d0 + c
            k = (l, ni, nj)
            if nd < dist.get(k, 1e18):
                dist[k] = nd
                heapq.heappush(pq, (nd + 0.95 * math.hypot(ni - g[0], nj - g[1]), k, cur))
        if not viablk[i, j]:
            for nl in range(len(LAYERS)):
                if nl == l:
                    continue
                k = (nl, i, j)
                nd = d0 + VIA
                if nd < dist.get(k, 1e18):
                    dist[k] = nd
                    heapq.heappush(pq, (nd + 0.95 * math.hypot(i - g[0], j - g[1]), k, cur))
    if hit is None:
        print("NO PATH")
        return None
    path = []
    c = hit
    while c is not None:
        path.append(c)
        c = prev[c]
    path.reverse()
    runs, cur = [], [path[0]]
    for p in path[1:]:
        if p[0] != cur[-1][0]:
            runs.append(cur)
            cur = [p]
        else:
            cur.append(p)
    runs.append(cur)
    segs, vias = [], []
    for r in runs:
        pts = [(x0 + i * GRID, y0 + j * GRID) for _, i, j in r]
        simp = [pts[0]]
        for k in range(1, len(pts) - 1):
            ax, ay = simp[-1]
            bx, by = pts[k]
            cx2, cy2 = pts[k + 1]
            if abs((bx - ax) * (cy2 - ay) - (by - ay) * (cx2 - ax)) > 1e-9:
                simp.append(pts[k])
        simp.append(pts[-1])
        for k in range(len(simp) - 1):
            segs.append((LAYERS[r[0][0]], round(simp[k][0], 3), round(simp[k][1], 3),
                         round(simp[k + 1][0], 3), round(simp[k + 1][1], 3)))
    for k in range(len(runs) - 1):
        _, i, j = runs[k][-1]
        vias.append((round(x0 + i * GRID, 3), round(y0 + j * GRID, 3)))
    return segs, vias


if __name__ == "__main__":
    net = sys.argv[1]
    a = (float(sys.argv[2]), float(sys.argv[3]))
    bp = (float(sys.argv[4]), float(sys.argv[5]))
    w = float(sys.argv[6]) if len(sys.argv) > 6 else 0.3
    res = route(net, a, bp, w)
    if res:
        segs, vias = res
        print("PATH: %d segments, %d vias" % (len(segs), len(vias)))
        json.dump({"net": net, "width": w, "segs": segs, "vias": vias},
                  open("astar_path.json", "w"), indent=1)
        for s in segs:
            print("   %-7s (%8.3f,%7.3f) -> (%8.3f,%7.3f)" % s)
        for v in vias:
            print("   via (%8.3f,%7.3f)" % v)
