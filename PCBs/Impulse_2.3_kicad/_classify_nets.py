# Classify every segment of the power nets as CURRENT-CARRYING or STUB, so each can be sized for
# what it actually carries instead of being lumped together.
#
# WHY: a per-segment widening pass treats a net as one thing. It left the pyro drain nets with
# widths of 1.0/1.2/1.4/1.6/2.0/2.5 mm scattered along them -- ragged, which hides bottlenecks --
# and it inflated the continuity-SENSE branches (~0.6 mA) to 2.5 mm, wasting board space and
# blocking routing. Both are artifacts, not design.
#
# METHOD: build the net's graph, then iteratively prune degree-1 nodes that are not POWER pads.
# What survives is the core that actually carries current between the power terminals; everything
# pruned is a stub feeding a sense resistor, gate resistor or decoupling cap.
import json, math, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _geom

HERE = os.path.dirname(os.path.abspath(__file__))

# net -> pads that genuinely carry the current (everything else on the net is a stub)
POWER_PADS = {
    "7.4V":            [("P1O", "2"), ("P2O", "2"), ("P3O", "2"), ("P4O", "2"), ("Q21", "5"), ("C10", "1")],
    "7.4V_RAW":        [("PI", "1"), ("Q21", "1"), ("Q21", "2"), ("Q21", "3")],
    "Net-(P1O-Pin_1)": [("P1O", "1"), ("Q16", "1")],
    "Net-(P2O-Pin_1)": [("P2O", "1"), ("Q17", "1")],
    "Net-(P3O-Pin_1)": [("P3O", "1"), ("Q19", "1")],
    "Net-(P4O-Pin_1)": [("P4O", "1"), ("Q18", "1")],
    "VBAT":            [("U2", "2"), ("U2", "3"), ("U3", "3"), ("C2", "1"), ("C9", "1"), ("C39", "1"), ("Q20", "5")],
    "VBAT_RAW":        [("MI", "1"), ("Q20", "1"), ("Q20", "2"), ("Q20", "3")],
}
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]


def key(x, y, l):
    return (round(x, 3), round(y, 3), l)


def classify(b, net):
    segs = [t for t in b.tracks if t["net"] == net]
    vias = [v for v in b.vias if v["net"] == net]
    pads = {(p["ref"], p["num"]): p for p in b.pads if p["net"] == net}
    edges = []            # (node_a, node_b, seg_or_None)
    for t in segs:
        edges.append((key(t["x1"], t["y1"], t["layer"]), key(t["x2"], t["y2"], t["layer"]), t))
    for v in vias:
        for i in range(len(LAYERS) - 1):
            edges.append((key(v["x"], v["y"], LAYERS[i]), key(v["x"], v["y"], LAYERS[i + 1]), None))
    # anchor nodes: any graph node lying on a power pad
    anchors = set()
    nodes = set()
    for a, c, _ in edges:
        nodes.add(a); nodes.add(c)
    for pk in POWER_PADS.get(net, []):
        p = pads.get(pk)
        if not p:
            continue
        for n in nodes:
            x, y, l = n
            if l in p["layers"] and abs(x - p["x"]) <= p["w"] / 2 + 0.4 and abs(y - p["y"]) <= p["h"] / 2 + 0.4:
                anchors.add(n)
    # iteratively prune degree-1 non-anchor nodes
    live = set(range(len(edges)))
    while True:
        deg = {}
        for i in live:
            a, c, _ = edges[i]
            deg[a] = deg.get(a, 0) + 1
            deg[c] = deg.get(c, 0) + 1
        drop = set()
        for i in live:
            a, c, _ = edges[i]
            for n in (a, c):
                if deg.get(n, 0) == 1 and n not in anchors:
                    drop.add(i)
        if not drop:
            break
        live -= drop
    core = [edges[i][2] for i in live if edges[i][2] is not None]
    stub = [edges[i][2] for i in range(len(edges)) if i not in live and edges[i][2] is not None]
    return core, stub, len(anchors)


if __name__ == "__main__":
    b = _geom.Board(os.path.join(HERE, "geom.json"), os.path.join(HERE, "Impulse_2.3.kicad_pro"))
    out = {}
    print("%-18s %-6s %-28s %-28s" % ("net", "anchor", "CURRENT-CARRYING core", "STUBS (sense/gate/decoupling)"))
    for net in POWER_PADS:
        core, stub, na = classify(b, net)
        def summ(ss):
            if not ss:
                return "-"
            ws = sorted({round(s["w"], 2) for s in ss})
            L = sum(math.hypot(s["x2"] - s["x1"], s["y2"] - s["y1"]) for s in ss)
            return "%d seg, %.1f mm, w=%s" % (len(ss), L, ",".join("%g" % w for w in ws))
        print("%-18s %-6d %-28s %-28s" % (net, na, summ(core), summ(stub)))
        out[net] = {
            "core": [[s["layer"], s["x1"], s["y1"], s["x2"], s["y2"], s["w"]] for s in core],
            "stub": [[s["layer"], s["x1"], s["y1"], s["x2"], s["y2"], s["w"]] for s in stub],
        }
    json.dump(out, open(os.path.join(HERE, "net_classes.json"), "w"), indent=1)
    print("\nwrote net_classes.json")
