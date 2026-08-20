# Add the hand-routed connections freerouting could not place.
#
# WHY IT COULDN'T: both are low-current nets carrying a POWER netclass width. VBAT is MAIN_PWR
# (1.2 mm) but U3 pin 5 is the buck's ENABLE pin -- microamps -- and 1.2 mm will not thread a
# SOT-23-6 pin field. Net-(P4O-Pin_1) is PYRO_DRAIN (1.0 mm) but the R42 leg is the continuity
# SENSE branch (~0.6 mA); the firing half of that net is already routed FET-tab -> connector.
# Freerouting never necks below the class width, so it reported both as unroutable. Hand-routing
# them at signal width is correct, not a compromise.
import hashlib, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")

# (net, layer, x1, y1, x2, y2, width, label)
SEGS = [
    ("VBAT", "F.Cu", 162.480, 65.478, 161.430, 65.478, 0.4, "U3.5 EN west out of the pin field"),
    ("VBAT", "F.Cu", 161.430, 65.478, 161.430, 65.780, 0.4, "up to via A"),
    ("VBAT", "B.Cu", 161.430, 65.780, 164.000, 63.180, 0.4, "B.Cu hop under the buck"),
    ("VBAT", "F.Cu", 164.000, 63.180, 164.754, 64.528, 0.4, "via B into U3.3 VIN"),
]
VIAS = [("VBAT", 161.430, 65.780, 0.5, 0.3), ("VBAT", 164.000, 63.180, 0.5, 0.3)]


def u(tag):
    h = hashlib.md5(("straggler23/" + tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[:8], h[8:12], h[12:16], h[16:20], h[20:32])


t = open(PCB, encoding="utf-8", newline="").read()
crlf = "\r\n" in t
t = t.replace("\r\n", "\n")
anchor = t.rindex("\n\t(segment")
end = t.index("\n\t)", anchor) + 3
blocks = []
for net, lay, x1, y1, x2, y2, w, lab in SEGS:
    blocks.append('\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n\t\t(layer "%s")\n'
                  '\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
                  % (x1, y1, x2, y2, w, lay, net, u("%s%s%s%s%s" % (net, lay, x1, y1, x2))))
for net, x, y, size, drill in VIAS:
    blocks.append('\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n\t\t(layers "F.Cu" "B.Cu")\n'
                  '\t\t(net "%s")\n\t\t(uuid "%s")\n\t)' % (x, y, size, drill, net, u("via%s%s%s" % (net, x, y))))
t = t[:end] + "".join(blocks) + t[end:]
open(PCB, "w", encoding="utf-8", newline="").write(t.replace("\n", "\r\n") if crlf else t)
print("added %d segments + %d vias" % (len(SEGS), len(VIAS)))
for net, lay, x1, y1, x2, y2, w, lab in SEGS:
    print("   %-6s %-7s w=%.1f  %s" % (net, lay, w, lab))
