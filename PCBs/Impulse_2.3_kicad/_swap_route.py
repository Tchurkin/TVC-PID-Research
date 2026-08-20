# Replace a set of segments with an A* route (astar_path.json), at uniform width.
import hashlib, json, os, re, sys
HERE=os.path.dirname(os.path.abspath(__file__)); PCB=os.path.join(HERE,"Impulse_2.3.kicad_pcb")
d=json.load(open(os.path.join(HERE,"astar_path.json")))
net,w=d["net"],d["width"]
# segments to delete: net,layer,x1,y1,x2,y2 (order-insensitive) from argv pairs file
dele=json.load(open(os.path.join(HERE,sys.argv[1])))
def bal(s,i):
    dd=0
    for j in range(i,len(s)):
        if s[j]=='(': dd+=1
        elif s[j]==')':
            dd-=1
            if dd==0: return j+1
t=open(PCB,encoding="utf-8",newline="").read(); crlf="\r\n" in t; t=t.replace("\r\n","\n")
out,pos,n=[],0,0
for m in re.finditer(r'^\t\(segment\b',t,re.M):
    if m.start()<pos: continue
    e=bal(t,m.start()+1); blk=t[m.start():e]
    s=re.search(r'\(start ([-\d.]+) ([-\d.]+)\)',blk); en=re.search(r'\(end ([-\d.]+) ([-\d.]+)\)',blk)
    nt=re.search(r'\(net "([^"]+)"\)',blk); ly=re.search(r'\(layer "([^"]+)"\)',blk)
    if s and en and nt and ly:
        a=(round(float(s.group(1)),3),round(float(s.group(2)),3)); c=(round(float(en.group(1)),3),round(float(en.group(2)),3))
        for dn,dl,x1,y1,x2,y2 in dele:
            if nt.group(1)==dn and ly.group(1)==dl and {a,c}=={(round(x1,3),round(y1,3)),(round(x2,3),round(y2,3))}:
                while e<len(t) and t[e] in "\r\n": e+=1
                out.append(t[pos:m.start()]); pos=e; n+=1; break
out.append(t[pos:]); t="".join(out)
def u(tag):
    h=hashlib.md5(("swaproute23/"+tag).encode()).hexdigest()
    return "%s-%s-%s-%s-%s"%(h[:8],h[8:12],h[12:16],h[16:20],h[20:32])
a=t.rindex("\n\t(segment"); end=t.index("\n\t)",a)+3
blocks=['\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
        %(x1,y1,x2,y2,w,lay,net,u("%s%d"%(net,i))) for i,(lay,x1,y1,x2,y2) in enumerate(d["segs"])]
t=t[:end]+"".join(blocks)+t[end:]
open(PCB,"w",encoding="utf-8",newline="").write(t.replace("\n","\r\n") if crlf else t)
print("removed %d old segments, added %d at %.2f mm uniform"%(n,len(d["segs"]),w))
