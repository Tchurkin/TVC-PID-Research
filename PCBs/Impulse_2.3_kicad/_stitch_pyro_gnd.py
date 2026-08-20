# Bond each pyro FET's source (GND) pads to the other three GND planes.
#
# WHY: the four firing FETs return the full ~18 A through their source pads into the B.Cu pour, and
# the 2026-08-19 audit found the B.Cu return pinches to ~0.30 mm beside each FET. There were ZERO
# GND stitching vias within 6 mm of any of the four -- so the entire return was one layer of 1 oz
# copper through whatever the pour happened to leave. Vias here put F.Cu, In1 and In2 in parallel
# with it, which is the cheapest large improvement available and moves no footprint.
#
# Pour-aware, like _stitch_gnd.py: a via is only placed where real GND copper exists on >=2 layers,
# because via_ok proves clearance but says nothing about whether copper is there.
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _geom
import _stitch_gnd as S          # reuse islands()/inside()/dist_to_edge()
import _add_tracks               # only for the uuid helper

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
SIZE, DRILL = 0.8, 0.4

# source-pad rows of the four firing FETs (from geom): three GND pads each, on B.Cu
FETS = {
    "Q16": [(164.887, 51.192), (166.157, 51.192), (167.427, 51.192)],
    "Q17": [(155.489, 51.192), (156.759, 51.192), (158.029, 51.192)],
    "Q19": [(146.091, 51.192), (147.361, 51.192), (148.631, 51.192)],
    "Q18": [(137.963, 51.192), (139.233, 51.192), (140.503, 51.192)],
}


def main():
    text = open(PCB, encoding="utf-8", newline="").read().replace("\r\n", "\n")
    polys = S.islands(text)
    b = _geom.Board(os.path.join(HERE, "geom.json"), os.path.join(HERE, "Impulse_2.3.kicad_pro"))
    need = SIZE / 2.0 + 0.15
    placed = []
    for ref, pads in sorted(FETS.items()):
        got = 0
        for (px, py) in pads:
            # try just south, then north, of each source pad, stepping outward
            # nearest-first radial scan: space beside these FETs is tight, so take
            # whatever legal spot is closest rather than a fixed offset
            cands = []
            for r in [0.4 * k for k in range(3, 14)]:
                for a in range(0, 360, 15):
                    cands.append((px + r * math.cos(math.radians(a)),
                                  py + r * math.sin(math.radians(a))))
            for (x, y) in cands:
                hits = {lay for lay, pts in polys
                        if S.inside((x, y), pts) and S.dist_to_edge((x, y), pts) > need}
                if len(hits) >= 2 and b.via_ok("GND", x, y, SIZE):
                    placed.append((round(x, 3), round(y, 3)))
                    b.vias.append({"net": "GND", "x": x, "y": y, "size": SIZE, "d": DRILL})
                    got += 1
                    break
        print("  %s: %d/%d source pads stitched" % (ref, got, len(pads)))
    if not placed:
        print("nothing placed")
        return
    crlf = "\r\n" in open(PCB, encoding="utf-8", newline="").read()
    if not os.path.exists(PCB + ".pre_pyrognd"):
        import shutil
        shutil.copy2(PCB, PCB + ".pre_pyrognd")
    a = text.rindex("\n\t(via") if "\n\t(via" in text else text.rindex("\n\t(segment")
    end = text.index("\n\t)", a) + 3
    blocks = ['\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n\t\t(layers "F.Cu" "B.Cu")\n'
              '\t\t(net "GND")\n\t\t(uuid "%s")\n\t)'
              % (x, y, SIZE, DRILL, _add_tracks.uid("pyrognd/%s,%s" % (x, y)))
              for x, y in placed]
    text = text[:end] + "".join(blocks) + text[end:]
    open(PCB, "w", encoding="utf-8", newline="").write(text.replace("\n", "\r\n") if crlf else text)
    print("placed %d pyro GND stitching vias" % len(placed))


if __name__ == "__main__":
    main()
