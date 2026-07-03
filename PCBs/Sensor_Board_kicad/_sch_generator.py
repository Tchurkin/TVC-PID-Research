#!/usr/bin/env python3
# Generate a KiCad schematic for the sensor board. Custom box symbols + global-label
# connectivity (each pin gets a global label = its net; same net name -> connected).
import uuid, sys
OUT = "Sensor_Board.kicad_sch"
ROOT = str(uuid.uuid4())
def U(): return str(uuid.uuid4())
HW = 6.35          # box half-width
LEN = 3.81         # pin length
PITCH = 2.54

# component: ref, value, [(num, name, net, side)]  side in {L,R}
COMPS = [
 ("U1","ICM-42688-P",[
   ("10","SCLK","SCK","L"),("12","SDI","MOSI","L"),("11","SDO","MISO","L"),("9","CS","CS_IMU","L"),
   ("14","VDD","+3V3","R"),("13","VDDIO","+3V3","R"),("5","GND","GND","R"),("1","INT1","INT_IMU","R")], 90,70),
 ("U2","DPS310",[
   ("4","SCL","SCL","L"),("3","SDA","SDA","L"),("2","CSB","+3V3","L"),("5","SDO_ADDR","GND","L"),
   ("8","VDD","+3V3","R"),("6","VDDIO","+3V3","R"),("1","GND","GND","R"),("7","GND2","GND","R")],150,70),
 ("J1","Header-2x5",[
   ("1","3V3","+3V3_RAW","R"),("2","GND","GND","R"),("3","SCK","SCK","R"),("4","MOSI","MOSI","R"),
   ("5","MISO","MISO","R"),("6","CS","CS_IMU","R"),("7","INT","INT_IMU","R"),("8","SDA","SDA","R"),
   ("9","SCL","SCL","R"),("10","GND","GND","R")],40,60),
 ("FB1","FerriteBead",[("1","1","+3V3_RAW","L"),("2","2","+3V3","R")],90,120),
 ("R1","10k",[("1","1","CS_IMU","L"),("2","2","+3V3","R")],120,120),
 ("C1","100nF",[("1","1","+3V3","L"),("2","2","GND","R")],150,120),
 ("C2","10nF",[("1","1","+3V3","L"),("2","2","GND","R")],180,120),
 ("C3","100nF",[("1","1","+3V3","L"),("2","2","GND","R")],150,140),
 ("C4","100nF",[("1","1","+3V3","L"),("2","2","GND","R")],180,140),
 ("C5","10uF",[("1","1","+3V3","L"),("2","2","GND","R")],120,140),
]

def build_symbol(ref, value, pins):
    lib = "sensor:%s" % ref
    L = [p for p in pins if p[3]=="L"]; R = [p for p in pins if p[3]=="R"]
    n = max(len(L), len(R)); halfh = (n*PITCH)/2 + PITCH
    s = []
    s.append('    (symbol "%s"' % lib)
    s.append('      (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)')
    s.append('      (property "Reference" "%s" (at 0 %.2f 0) (effects (font (size 1.27 1.27))))' % (ref[0], halfh+2))
    s.append('      (property "Value" "%s" (at 0 %.2f 0) (effects (font (size 1.27 1.27))))' % (value, -halfh-2))
    s.append('      (symbol "%s_1_1"' % ref)
    s.append('        (rectangle (start %.2f %.2f) (end %.2f %.2f) (stroke (width 0.254) (type default)) (fill (type background)))' % (-HW, halfh, HW, -halfh))
    def emit(lst, side):
        out=[]
        for i,(num,name,net,sd) in enumerate(lst):
            y = halfh - PITCH - i*PITCH
            if side=="L":
                x=-(HW+LEN); ang=0            # points right (toward body); free end at x
            else:
                x=(HW+LEN); ang=180
            out.append('        (pin passive line (at %.2f %.2f %d) (length %.2f) (name "%s" (effects (font (size 1.0 1.0)))) (number "%s" (effects (font (size 1.0 1.0)))))' % (x,y,ang,LEN,name,num))
        return out
    s += emit(L,"L"); s += emit(R,"R")
    s.append('      )')
    s.append('    )')
    # return also pin endpoint local coords for label placement
    ep={}
    for i,(num,name,net,sd) in enumerate(L):
        ep[num]=(-(HW+LEN), halfh - PITCH - i*PITCH)
    for i,(num,name,net,sd) in enumerate(R):
        ep[num]=((HW+LEN), halfh - PITCH - i*PITCH)
    return "\n".join(s), ep

FPIDS={"U1":"Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y",
 "U2":"Package_LGA:Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering",
 "J1":"Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical",
 "FB1":"Resistor_SMD:R_0402_1005Metric","R1":"Resistor_SMD:R_0402_1005Metric",
 "C1":"Capacitor_SMD:C_0402_1005Metric","C2":"Capacitor_SMD:C_0402_1005Metric",
 "C3":"Capacitor_SMD:C_0402_1005Metric","C4":"Capacitor_SMD:C_0402_1005Metric",
 "C5":"Capacitor_SMD:C_0805_2012Metric"}
libsyms=[]; instances=[]; labels=[]; endpoints={}
for ref,value,pins,ix,iy in COMPS:
    ix=round(ix/2.54)*2.54; iy=round(iy/2.54)*2.54   # snap to 0.1" grid
    sym,ep = build_symbol(ref,value,pins)
    libsyms.append(sym); endpoints[ref]=ep
    # instance
    pinuuids="".join('\n        (pin "%s" (uuid %s))'%(p[0],U()) for p in pins)
    instances.append(
      '  (symbol (lib_id "sensor:%s") (at %.2f %.2f 0) (unit 1) (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)\n'
      '    (uuid %s)\n'
      '    (property "Reference" "%s" (at %.2f %.2f 0) (effects (font (size 1.27 1.27))))\n'
      '    (property "Value" "%s" (at %.2f %.2f 0) (effects (font (size 1.27 1.27))))\n'
      '    (property "Footprint" "%s" (at %.2f %.2f 0) (effects (font (size 1.27 1.27)) (hide yes)))%s\n'
      '    (instances (project "Sensor_Board" (path "/%s" (reference "%s") (unit 1))))\n  )'
      %(ref,ix,iy,U(),ref,ix+HW+2,iy-2,value,ix+HW+2,iy+2,FPIDS.get(ref,""),ix+HW+2,iy+4,pinuuids,ROOT,ref))
    # global labels at each pin endpoint (schematic abs = ix+lx, iy-ly)
    for num,name,net,side in pins:
        lx,ly=ep[num]; ax=ix+lx; ay=iy-ly
        just = "right" if side=="L" else "left"
        # label anchored at pin free end
        labels.append('  (global_label "%s" (shape passive) (at %.2f %.2f %d) (effects (font (size 1.27 1.27)) (justify %s)) (uuid %s))'
                       %(net,ax,ay,(180 if side=="L" else 0),just,U()))

with open(OUT,"w",encoding="utf-8") as f:
    f.write('(kicad_sch\n  (version 20231120)\n  (generator "eeschema")\n  (generator_version "8.0")\n  (uuid %s)\n  (paper "A3")\n'%ROOT)
    f.write('  (lib_symbols\n'+"\n".join(libsyms)+'\n  )\n')
    f.write("\n".join(instances)+"\n")
    f.write("\n".join(labels)+"\n")
    f.write('  (sheet_instances (path "/" (page "1")))\n)\n')
print("wrote",OUT,"with",len(COMPS),"symbols,",len(labels),"pin-labels")
# standalone symbol lib (bare symbol names) + sym-lib-table so lib_id "sensor:X" resolves
standalone=[s.replace('(symbol "sensor:','(symbol "') for s in libsyms]
with open("sensor.kicad_sym","w",encoding="utf-8") as f:
    f.write('(kicad_symbol_lib\n  (version 20231120)\n  (generator "custom")\n')
    f.write("\n".join(standalone)+"\n)\n")
with open("sym-lib-table","w",encoding="utf-8") as f:
    f.write('(sym_lib_table\n  (version 7)\n  (lib (name "sensor")(type "KiCad")(uri "${KIPRJMOD}/sensor.kicad_sym")(options "")(descr ""))\n)\n')
print("wrote sensor.kicad_sym + sym-lib-table")
