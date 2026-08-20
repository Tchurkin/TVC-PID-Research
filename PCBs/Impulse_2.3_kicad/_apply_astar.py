import hashlib, json, os, sys
HERE=os.path.dirname(os.path.abspath(__file__))
PCB=os.path.join(HERE,"Impulse_2.3.kicad_pcb")
d=json.load(open(os.path.join(HERE,"astar_path.json")))
net,w=d["net"],d["width"]
def u(tag):
    h=hashlib.md5(("astar23/"+tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s"%(h[:8],h[8:12],h[12:16],h[16:20],h[20:32])
t=open(PCB,encoding="utf-8",newline="").read(); crlf="\r\n" in t; t=t.replace("\r\n","\n")
a=t.rindex("\n\t(segment"); end=t.index("\n\t)",a)+3
blocks=[]
for i,(lay,x1,y1,x2,y2) in enumerate(d["segs"]):
    blocks.append('\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'%(x1,y1,x2,y2,w,lay,net,u("%s/s%d"%(net,i))))
for i,(x,y) in enumerate(d["vias"]):
    blocks.append('\n\t(via\n\t\t(at %s %s)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'%(x,y,net,u("%s/v%d"%(net,i))))
t=t[:end]+"".join(blocks)+t[end:]
open(PCB,"w",encoding="utf-8",newline="").write(t.replace("\n","\r\n") if crlf else t)
print("applied %d segments + %d vias for %s"%(len(d["segs"]),len(d["vias"]),net))
