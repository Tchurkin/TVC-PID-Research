#!/usr/bin/env python3
# Replace messy/invalid Edge.Cuts with a single clean gr_circle fitted (robustly) to the
# perimeter points. Board is a round rocket-tube PCB, so a fitted circle is the true shape.
import re, sys, uuid, math
IN, OUT = sys.argv[1], sys.argv[2]
t = open(IN, encoding="utf-8").read()

pts = []
for m in re.finditer(r'\(gr_line \(start ([\d.\-]+) ([\d.\-]+)\) \(end ([\d.\-]+) ([\d.\-]+)\)[^\n]*?"Edge\.Cuts"[^\n]*\)', t):
    a, b, c, d = map(float, m.groups()); pts += [(a, b), (c, d)]
for m in re.finditer(r'\(gr_arc \(start ([\d.\-]+) ([\d.\-]+)\) \(mid ([\d.\-]+) ([\d.\-]+)\) \(end ([\d.\-]+) ([\d.\-]+)\)[^\n]*?"Edge\.Cuts"[^\n]*\)', t):
    g = list(map(float, m.groups())); pts += [(g[0], g[1]), (g[2], g[3]), (g[4], g[5])]
print("Edge.Cuts points:", len(pts))

def solve3(M, v):  # Cramer's rule 3x3
    def det(m):
        return (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
              - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
              + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))
    D = det(M)
    out = []
    for i in range(3):
        Mi = [row[:] for row in M]
        for r in range(3): Mi[r][i] = v[r]
        out.append(det(Mi) / D)
    return out

def fit_circle(P):
    n = len(P)
    Sx = sum(p[0] for p in P); Sy = sum(p[1] for p in P)
    Sxx = sum(p[0]**2 for p in P); Syy = sum(p[1]**2 for p in P); Sxy = sum(p[0]*p[1] for p in P)
    Sxz = sum(p[0]*(p[0]**2+p[1]**2) for p in P)
    Syz = sum(p[1]*(p[0]**2+p[1]**2) for p in P)
    Sz  = sum((p[0]**2+p[1]**2) for p in P)
    M = [[Sxx, Sxy, Sx], [Sxy, Syy, Sy], [Sx, Sy, n]]
    A, B, C = solve3(M, [Sxz, Syz, Sz])
    a, b = A/2, B/2
    R = math.sqrt(max(0.0, C + a*a + b*b))
    return a, b, R

inl = pts[:]
for it in range(6):
    a, b, R = fit_circle(inl)
    res = [(abs(math.hypot(x-a, y-b) - R), (x, y)) for (x, y) in pts]
    inl = [p for d, p in res if d < 0.10 * R + 0.5]   # keep points near the circle
rms = math.sqrt(sum((math.hypot(x-a, y-b)-R)**2 for x, y in inl)/len(inl))
# enlarge R to enclose ALL copper pads with 0.5mm edge clearance (footprints are at angle 0,
# so pad global = footprint(at) + pad(at)). Prevents copper-edge-clearance violations.
CLEAR = 0.6
need = R
for fb in re.finditer(r'\(footprint\b', t):
    i = fb.start(); d = 0; j = i
    while j < len(t):
        if t[j] == '(': d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0: break
        j += 1
    blk = t[i:j+1]
    fa = re.search(r'\n\s+\(at ([\d.\-]+) ([\d.\-]+)', blk)
    if not fa: continue
    fx, fy = float(fa.group(1)), float(fa.group(2))
    for pm in re.finditer(r'\(pad "[^"]*" \S+ \S+ \(at ([\d.\-]+) ([\d.\-]+)[^)]*\) \(size ([\d.\-]+) ([\d.\-]+)\)', blk):
        px, py, sx, sy = map(float, pm.groups())
        reach = math.hypot(fx+px-a, fy+py-b) + max(sx, sy)/2 + CLEAR
        if reach > need: need = reach
R = need
print(f"circle center=({a:.2f},{b:.2f}) R(fit)~{rms:.2f}rms -> R(enclose copper)={R:.2f}mm  inliers={len(inl)}/{len(pts)}")

# remove existing Edge.Cuts gr_line/gr_arc
kept = [L for L in t.split("\n")
        if not ('"Edge.Cuts"' in L and (L.strip().startswith("(gr_line") or L.strip().startswith("(gr_arc")))]
t2 = "\n".join(kept)

circle = (f'  (gr_circle (center {a:.4f} {b:.4f}) (end {a+R:.4f} {b:.4f}) '
          f'(stroke (width 0.1) (type solid)) (fill no) (layer "Edge.Cuts") (uuid "{uuid.uuid4()}"))')
idx = t2.rstrip().rfind(")")
t2 = t2[:idx] + circle + "\n" + t2[idx:]
open(OUT, "w", encoding="utf-8").write(t2)
print("wrote single gr_circle outline")
