#!/usr/bin/env python3
"""Sweep the firmware across a target x wind grid and build ONE interactive HTML.

Runs the EXACT firmware (sim.exe) at every grid point, then embeds all the flights in a
self-contained page with TARGET and WIND sliders, a PASS/FAIL heatmap, an outcome readout,
and the animated replay -- so you can scrub the parameter space and watch where it breaks.

Usage:  python sweep_html.py [path\\to\\sim.exe] [out.html]
"""
import subprocess, csv, io, json, sys, os

_here = os.path.dirname(os.path.abspath(__file__))
# Default to the sim.exe next to THIS script as an absolute path. (Windows subprocess does NOT search the CWD,
# so a bare "sim.exe" raises FileNotFoundError -> every flight fails; an absolute path avoids that.)
SIM = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_here, "sim.exe")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(_here, "sweep.html")
if not os.path.exists(SIM):
    sys.exit(f"sim.exe not found at {SIM}\nBuild it first (build_zig.bat) or pass the path: python sweep_html.py <sim.exe>")
TARGETS = [0, 8, 16, 24]          # m downrange
WINDS   = [-4, -2, 0, 2, 4]       # m/s (+ = tailwind toward +x)
SEEDS   = ["1", "2", "3", "4", "5"]   # gust realizations per cell (median shown, spread reported)

def run(args):
    p = subprocess.run([SIM] + [str(a) for a in args], capture_output=True, text=True)
    rows = list(csv.DictReader(io.StringIO(p.stdout)))
    o = {"miss": 0.0, "vz": 0.0, "tilt": 0.0, "state": "?", "pass": False, "apogee": 0.0}
    for ln in p.stderr.splitlines():
        try:
            if "touchdown x" in ln:   o["miss"] = float(ln.split("miss")[1].strip().rstrip(")"))
            elif "touchdown speed" in ln: o["vz"] = float(ln.split(":")[1].split("m/s")[0])
            elif "final tilt" in ln:  o["tilt"] = float(ln.split(":")[1].split("deg")[0])
            elif "final state" in ln: o["state"] = ln.split(":")[1].strip()
            elif "apogee" in ln:      o["apogee"] = float(ln.split(":")[1].split("m")[0])
            elif "VERDICT" in ln:     o["pass"] = ("PASS" in ln)
        except Exception:
            pass
    return rows, o

def land_x(rows):
    return float(rows[-1]["px"]) if rows else 0.0

# The vehicle now aims itself: one CALM-wind kick per target (the pre-flight plan), then wind-adaptive ascent
# guidance steers the boost for whatever wind is actually present, and the descent lands PURE RETROGRADE (no
# divert). So we bisect the kick ONCE per target at W=0 and fly every wind with that same kick + gusts.
def bisect_kick(T, W=0):
    lo, hi = -8.0, 20.0
    for _ in range(7):
        mid = (lo + hi) / 2
        rows, _o = run(["--kick", mid, "--wind", W, "--nodivert"])
        (lo, hi) = (mid, hi) if land_x(rows) < T else (lo, mid)
    return round((lo + hi) / 2, 2)

# This sweep isolates the WIND/TARGET robustness axis: it holds the build/motor/timing dispersions at NOMINAL
# and pins ignition SCATTER to zero (--ignjit 0). Robustness to those dispersions is a SEPARATE question,
# measured by montecarlo.py (which stacks mass/inertia/CG/thrust/fin/timing/jitter jointly). Do not read a green
# heatmap here as "flight-ready" -- it means "the GNC handles wind+gusts", not "it survives a dispersed build".
DISP_OFF = ["--ignjit", "0"]    # no ignition scatter: nominal-plant wind sweep (dispersions -> montecarlo.py)
print("running grid (wind-adaptive ascent + gusts; nominal plant -- dispersions live in montecarlo.py)...")
grid = []
for T in TARGETS:
    r = []
    K = bisect_kick(T)          # ONE calm-wind kick per target; guidance adapts it in flight
    for W in WINDS:
        # Run several gust seeds (gusts are random -- a single fixed seed makes headwind LOOK systematically bad).
        # Display the MEDIAN-|miss| flight (representative), and report the mean/worst spread so the tail is honest.
        trials = []
        for seed in SEEDS:
            rows, o = run(["--kick", K, "--target", T, "--wind", W, "--gusts", "--seed", seed] + DISP_OFF)
            trials.append((rows, o))
        trials.sort(key=lambda t: abs(t[1]["miss"]))
        rows, o = trials[len(trials)//2]                       # median flight for animation
        misses = [t[1]["miss"] for t in trials]
        o["missMean"] = sum(misses)/len(misses)
        o["missWorst"] = max(misses, key=abs)
        o["passN"] = sum(1 for t in trials if t[1]["pass"])
        o["nSeed"] = len(trials)
        F = []
        for i, row in enumerate(rows):
            # keep every logged frame (20 ms sim) -> 50 fps at real-time playback (was every 3rd -> ~17 fps)
            g = lambda k: float(row[k])
            F.append([round(g("t"),2), row["state"], round(g("px"),2), round(g("pz"),1),
                      round(g("th_deg"),1), round(g("thrust"),0), round(g("estX"),2), round(g("estZ"),1),
                      round(g("thEst_deg"),1), round(g("uTVC"),2), round(g("deploy"),2), round(g("keff"),1),
                      round(g("vz"),1), int(float(row["legs"])), int(float(row["chute"])),
                      round(g("py"),2), round(g("Qw"),4), round(g("Qx"),4), round(g("Qy"),4), round(g("Qz"),4),
                      round(g("rollDef"),2), round(g("uTVC2"),2), round(g("estY"),2)])
        r.append({"f": F, "o": o, "T": T})
        print(f"  T={T:>3} W={W:>3}: pass {o['passN']}/{o['nSeed']}  miss median={o['miss']:6.2f} mean={o['missMean']:6.2f} worst={o['missWorst']:6.2f}  tilt={o['tilt']:6.1f}")
    grid.append(r)

data = json.dumps(grid, separators=(",", ":"))
allf = [fr for row in grid for run_ in row for fr in run_["f"]]
apogee = max((fr[3] for fr in allf), default=10)
xmin = min((min(fr[2], fr[6]) for fr in allf), default=0)
xmax = max((max(fr[2], fr[6]) for fr in allf), default=0)
ymax = max((abs(fr[15]) for fr in allf), default=1)

HTML = r"""<!doctype html><html><head><meta charset="utf-8"><title>Firmware weakness explorer</title>
<style>
 body{margin:0;background:#0d1117;color:#c9d1d9;font-family:Segoe UI,system-ui,sans-serif}
 #wrap{display:flex;gap:14px;padding:14px}#side{width:320px;font-size:13px}
 canvas{background:linear-gradient(#0b1a2b,#0d1117);border:1px solid #30363d;border-radius:8px}
 h2{margin:.2em 0;font-size:16px;color:#58a6ff} .row{margin:5px 0} b{color:#8b949e}
 button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;padding:5px 10px;cursor:pointer}
 input[type=range]{width:100%}
 table{border-collapse:collapse;margin-top:6px} td,th{border:1px solid #30363d;padding:3px 6px;text-align:center;font-size:12px}
 td.cell{cursor:pointer} .tag{display:inline-block;padding:1px 8px;border-radius:10px;font-weight:600;color:#fff}
</style></head><body>
<div id="wrap">
 <canvas id="c" width="820" height="640"></canvas>
 <div id="side">
  <h2>Firmware weakness explorer <span style="font-size:11px;color:#8b949e">3-D &middot; drag to orbit</span></h2>
  <div class="row">Each cell/slider position is a full run of the EXACT firmware.</div>
  <div class="row">TARGET <input type="range" id="ti" min="0" max="__TN__" value="0"> <span id="tv"></span> m</div>
  <div class="row">WIND <input type="range" id="wi" min="0" max="__WN__" value="__W0__"> <span id="wv"></span> m/s</div>
  <div class="row"><button id="play">&#10073;&#10073; Pause</button> <button id="restart">&#9658; Restart</button>
     speed <input type="range" id="spd" min="20" max="400" value="100" style="width:90px"><span id="spdv">1.0x</span></div>
  <hr style="border-color:#30363d">
  <div class="row">outcome <span class="tag" id="verdict"></span></div>
  <div class="row">t <b id="t"></b>s &middot; phase <b id="ph"></b> &middot; alt <b id="alt"></b> m</div>
  <div class="row">downrange <b id="dr"></b> m &middot; crossrange <b id="cr"></b> m &middot; roll <b id="rl"></b>&deg;</div>
  <div class="row">miss <b id="miss"></b> m &middot; touchdown vz <b id="tvz"></b> m/s &middot; tilt <b id="tilt"></b>&deg;</div>
  <div class="row">apogee <b id="apo"></b> m &middot; <button id="topv" style="padding:2px 6px;font-size:11px">top view</button> <button id="sidev" style="padding:2px 6px;font-size:11px">side view</button></div>
  <hr style="border-color:#30363d">
  <div class="row"><b>PASS/FAIL map</b> (rows=target, cols=wind; click a cell). green=soft on-target, red=fail.</div>
  <div class="row" style="color:#d29922;font-size:12px">WIND/TARGET robustness only (nominal build+motor). Robustness to build/motor/timing dispersion is separate &mdash; see montecarlo.py.</div>
  <table id="map"></table>
  <div class="row" style="color:#8b949e;margin-top:8px">3-D render of the TRUE rocket (attitude from the logged quaternion). Gray marker = EKF position belief. The 4 blue margin fins fold out with deploy (folded flat=stable, 90&deg; out=destabilizing; a differential fold does roll); the 3 green legs swing out at ignition; the gimballed exhaust shows the TVC deflection. Drag the scene to orbit; wheel to zoom. Playback 1.0x = real time.</div>
 </div>
</div>
<script>
const G=__DATA__, TARGETS=__TARGETS__, WINDS=__WINDS__, APO=__APO__, XMIN=__XMIN__, XMAX=__XMAX__, YMAX=__YMAX__;
const cv=document.getElementById('c'),cx=cv.getContext('2d'),W=cv.width,H=cv.height;
const PHc={BOOST:'#f0883e',COAST:'#58a6ff',BURN:'#f85149',TD:'#3fb950',ABORT:'#f85149',PAD:'#8b949e',ARM:'#8b949e',CD:'#d29922'};
// ================= tiny 3-D engine (orbit camera + painter's algorithm) =================
// World axes: X = downrange, Y = crossrange, Z = up. Body +Z = the rocket's nose.
let az=-1.05, el=0.30, zoom=1.0;                      // orbit azimuth/elevation (rad) + zoom
const zTop=Math.max(APO*1.15,10);
let WSC=(H-150)/zTop;                                 // world metres -> px (updated by zoom)
const CEN=[Math.max(XMAX,8)*0.45, 0, zTop*0.42];      // look-at centre (world)
const RPX=30;                                         // rocket drawn at a fixed pixel size (schematic scale)
function cam(){const ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);
  return {R:[-sa,ca,0], U:[-se*ca,-se*sa,ce], F:[ce*ca,ce*sa,se]};}   // screen-right, screen-up, toward-camera
function dot(a,b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
function nrm(v){const m=Math.hypot(v[0],v[1],v[2])||1;return [v[0]/m,v[1]/m,v[2]/m];}
function qrot(q,v){const w=q[0],x=q[1],y=q[2],z=q[3];   // rotate a body vector to world by quaternion
  return [(1-2*(y*y+z*z))*v[0]+2*(x*y-w*z)*v[1]+2*(x*z+w*y)*v[2],
          2*(x*y+w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-w*x)*v[2],
          2*(x*z-w*y)*v[0]+2*(y*z+w*x)*v[1]+(1-2*(x*x+y*y))*v[2]];}
function projW(p){const b=cam(),dx=[p[0]-CEN[0],p[1]-CEN[1],p[2]-CEN[2]];   // world point -> [sx,sy,depth]
  return [W/2+WSC*zoom*dot(dx,b.R), H/2-WSC*zoom*dot(dx,b.U), dot(dx,b.F)];}
function projB(q,vb){const b=cam(),vw=qrot(q,vb);return [RPX*dot(vw,b.R), -RPX*dot(vw,b.U), dot(vw,b.F)];}  // body vec -> screen offset(px)+depth
function shade(hex,br){const n=parseInt(hex.slice(1),16),r=(n>>16)&255,g=(n>>8)&255,b=n&255,f=Math.max(0.35,Math.min(1,br));
  return 'rgb('+(r*f|0)+','+(g*f|0)+','+(b*f|0)+')';}
// build all shaded faces of the rocket at world position `pos`, attitude quaternion `q`
function rocketFaces(pos,q,dep,rdf,thr,uT,uT2,legF){
  const ps=projW(pos), F=cam().F, faces=[];
  const V=vb=>{const o=projB(q,vb);return [ps[0]+o[0], ps[1]+o[1], o[2]];};   // vertex -> screen [x,y,localDepth]
  const face=(vs,col,br,strk)=>{let d=0;for(const v of vs)d+=v[2];faces.push({p:vs,d:d/vs.length,c:col,b:br==null?1:br,s:strk});};
  const rB=0.16, bot=-1.0, top=0.85, tip=1.25, NS=14;
  const facing=a=>{const nw=qrot(q,[Math.cos(a),Math.sin(a),0]);return 0.55+0.45*dot(nw,F);};  // body-surface lighting
  for(let k=0;k<NS;k++){const a0=k/NS*2*Math.PI,a1=(k+1)/NS*2*Math.PI;   // body tube
    face([V([rB*Math.cos(a0),rB*Math.sin(a0),bot]),V([rB*Math.cos(a1),rB*Math.sin(a1),bot]),
          V([rB*Math.cos(a1),rB*Math.sin(a1),top]),V([rB*Math.cos(a0),rB*Math.sin(a0),top])],'#dfe7ef',facing((a0+a1)/2));}
  for(let k=0;k<NS;k++){const a0=k/NS*2*Math.PI,a1=(k+1)/NS*2*Math.PI;   // nose cone
    face([V([rB*Math.cos(a0),rB*Math.sin(a0),top]),V([rB*Math.cos(a1),rB*Math.sin(a1),top]),V([0,0,tip])],'#f85149',facing((a0+a1)/2));}
  const finZ=0.42, span=0.75, chord=0.34;             // 4 fold-out margin fins (canards near the nose)
  for(let f=0;f<4;f++){const phi=f*Math.PI/2, rad=[Math.cos(phi),Math.sin(phi),0], tan=[-Math.sin(phi),Math.cos(phi),0];
    let fa=(dep + rdf*(f%2?-1:1)*0.5)*Math.PI/2; fa=Math.max(0,Math.min(Math.PI/2,fa));   // differential fold -> roll
    const fold=[Math.sin(fa)*rad[0],Math.sin(fa)*rad[1],-Math.cos(fa)];   // fa0: along body (-Z); fa90: straight out
    const h=[rB*rad[0],rB*rad[1],finZ];
    const c0=V(h), c1=V([h[0],h[1],h[2]+chord]),
          c2=V([h[0]+span*fold[0],h[1]+span*fold[1],h[2]+chord+span*fold[2]]), c3=V([h[0]+span*fold[0],h[1]+span*fold[1],h[2]+span*fold[2]]);
    face([c0,c1,c2,c3],'#4d9dff',0.6+0.4*Math.abs(dot(qrot(q,tan),F)),'#2f6fb0');}  // lit by the fin's flat-face normal (~tangent)
  if(legF>0.01){for(let f=0;f<3;f++){const phi=f*2*Math.PI/3+0.5, rad=[Math.cos(phi),Math.sin(phi),0], tan=[-Math.sin(phi),Math.cos(phi),0];
    const oa=legF*1.15, dir=[Math.sin(oa)*rad[0],Math.sin(oa)*rad[1],-Math.cos(oa)], h=[rB*rad[0],rB*rad[1],bot+0.05], wgt=0.05;   // 3 landing legs
    const knee=[h[0]+0.62*dir[0],h[1]+0.62*dir[1],h[2]+0.62*dir[2]];
    face([V([h[0]+wgt*tan[0],h[1]+wgt*tan[1],h[2]]),V([h[0]-wgt*tan[0],h[1]-wgt*tan[1],h[2]]),
          V([knee[0]-wgt*tan[0],knee[1]-wgt*tan[1],knee[2]]),V([knee[0]+wgt*tan[0],knee[1]+wgt*tan[1],knee[2]])],'#3fb950',0.9);
    face([V([knee[0],knee[1],knee[2]]),V([knee[0]+0.14*rad[0],knee[1]+0.14*rad[1],knee[2]-0.02]),V([knee[0]+0.14*rad[0]+0.02*tan[0],knee[1]+0.14*rad[1],knee[2]+0.03])],'#2f9e42',0.9);}}
  if(thr>0.5){const dir=nrm([(uT||0)*0.03,(uT2||0)*0.03,-1]), fl=0.55+Math.min(thr,30)*0.02;   // gimballed exhaust flame
    const tipf=[dir[0]*fl,dir[1]*fl,bot+dir[2]*fl];
    for(let k=0;k<8;k++){const a0=k/8*2*Math.PI,a1=(k+1)/8*2*Math.PI,rr=0.11;
      face([V([rr*Math.cos(a0),rr*Math.sin(a0),bot]),V([rr*Math.cos(a1),rr*Math.sin(a1),bot]),V(tipf)],k%2?'#ffd23f':'#ff8c42',1.0);}}
  faces.sort((a,b)=>a.d-b.d);   // painter: far (small toward-camera depth) first
  for(const f of faces){cx.beginPath();cx.moveTo(f.p[0][0],f.p[0][1]);for(let j=1;j<f.p.length;j++)cx.lineTo(f.p[j][0],f.p[j][1]);cx.closePath();
    cx.fillStyle=shade(f.c,f.b);cx.fill();if(f.s){cx.strokeStyle=f.s;cx.lineWidth=1;cx.stroke();}}
  return ps;
}
let ti=0, wi=WINDS.indexOf(0)<0?0:WINDS.indexOf(0), i=0, playing=true, speed=1.0, last=performance.now(), acc=0, trail=[];
function cur(){return G[ti][wi];}
function gline(a,b,col,dash){const A=projW(a),B=projW(b);cx.strokeStyle=col;if(dash)cx.setLineDash(dash);
  cx.beginPath();cx.moveTo(A[0],A[1]);cx.lineTo(B[0],B[1]);cx.stroke();if(dash)cx.setLineDash([]);}
function draw(){
  const r=cur(), F=r.f; if(i>=F.length)i=0; const f=F[i];
  cx.clearRect(0,0,W,H);
  // ground grid on z=0 (world), spanning the flight
  const gx0=-4, gx1=Math.max(XMAX,r.T)+4, gyext=Math.max(YMAX,3)+2;
  cx.lineWidth=1;
  for(let gx=Math.ceil(gx0/5)*5; gx<=gx1; gx+=5) gline([gx,-gyext,0],[gx,gyext,0],'#1b2330');
  for(let gyv=-Math.floor(gyext/5)*5; gyv<=gyext; gyv+=5) gline([gx0,gyv,0],[gx1,gyv,0],'#1b2330');
  // pad (0,0) + target marker
  gline([-1,0,0],[1,0,0],'#8b949e'); gline([0,-1,0],[0,1,0],'#8b949e');
  const TR=1.2; cx.strokeStyle='#3fb950';cx.lineWidth=2;cx.beginPath();
  for(let k=0;k<=20;k++){const a=k/20*2*Math.PI,P=projW([r.T+TR*Math.cos(a),TR*Math.sin(a),0]);k?cx.lineTo(P[0],P[1]):cx.moveTo(P[0],P[1]);}
  cx.stroke();cx.lineWidth=1;
  const tp=projW([r.T,0,0]);cx.fillStyle='#3fb950';cx.font='11px monospace';cx.fillText('target '+r.T+'m',tp[0]+6,tp[1]-4);
  // trail (true path)
  cx.strokeStyle='#1f6feb';cx.lineWidth=1.6;cx.beginPath();
  for(let k=0;k<trail.length;k++){const P=projW(trail[k]);k?cx.lineTo(P[0],P[1]):cx.moveTo(P[0],P[1]);}cx.stroke();cx.lineWidth=1;
  // legs/chute latch (pyro pin high ~0.9 s) + first-fire index for the deploy swing animation
  let legs=false, chute=false, legFire=-1; for(let k=0;k<=i;k++){ if(F[k][13]){legs=true; if(legFire<0)legFire=k;} if(F[k][14]) chute=true; }
  const legF = legFire<0?0:Math.max(0,Math.min(1,(i-legFire)/25));   // ~0.5 s swing-out
  // EKF position belief marker (gray)
  const gp=projW([f[6],f[22],f[7]]);cx.strokeStyle='#8b949e';cx.setLineDash([3,3]);
  cx.beginPath();cx.arc(gp[0],gp[1],5,0,2*Math.PI);cx.stroke();cx.setLineDash([]);
  // THE ROCKET (true attitude from quaternion)
  const ps=rocketFaces([f[2],f[15],f[3]], [f[16],f[17],f[18],f[19]], f[10], f[20], f[5], f[9], f[21], legF);
  if(legs){cx.fillStyle='#3fb950';cx.font='12px sans-serif';cx.fillText('LEGS',ps[0]+22,ps[1]);}
  if(chute){cx.fillStyle='#d29922';cx.fillText('CHUTE',ps[0]+22,ps[1]-14);}
  document.getElementById('t').textContent=f[0].toFixed(2);
  const ph=document.getElementById('ph');ph.textContent=f[1];
  document.getElementById('alt').textContent=f[3].toFixed(1);
  document.getElementById('dr').textContent=f[2].toFixed(1);
  document.getElementById('cr').textContent=f[15].toFixed(2);
  // roll (true twist) — recover from the quaternion
  document.getElementById('rl').textContent=(2*Math.atan2(f[19],f[16])*180/Math.PI).toFixed(0);
}
function refresh(){
  const o=cur().o;
  document.getElementById('tv').textContent=TARGETS[ti];
  document.getElementById('wv').textContent=WINDS[wi];
  const v=document.getElementById('verdict');const allpass=o.passN===o.nSeed;v.textContent=(o.passN)+'/'+(o.nSeed)+' seeds pass';v.style.background=allpass?'#238636':'#da3633';
  document.getElementById('miss').textContent=o.miss.toFixed(2)+' (median; mean '+o.missMean.toFixed(2)+', worst '+o.missWorst.toFixed(2)+')';
  document.getElementById('tvz').textContent=o.vz.toFixed(2);
  document.getElementById('tilt').textContent=o.tilt.toFixed(1);
  document.getElementById('apo').textContent=o.apogee.toFixed(0);
  document.getElementById('ti').value=ti; document.getElementById('wi').value=wi;
  i=0; trail=[]; drawMap();
}
function drawMap(){
  let h='<tr><th>t\\w</th>'; for(const w of WINDS)h+='<th>'+w+'</th>'; h+='</tr>';
  for(let t=0;t<TARGETS.length;t++){ h+='<tr><th>'+TARGETS[t]+'</th>';
    for(let w=0;w<WINDS.length;w++){const o=G[t][w].o;const allp=o.passN===o.nSeed;const c=allp?'#196c2e':(o.passN>0?'#8a6d1a':'#8b1a1a');const sel=(t===ti&&w===wi)?'outline:2px solid #58a6ff;':'';
      h+='<td class="cell" style="background:'+c+';'+sel+'" data-t="'+t+'" data-w="'+w+'" title="pass '+o.passN+'/'+o.nSeed+' | miss mean '+o.missMean.toFixed(1)+' worst '+o.missWorst.toFixed(1)+'">'+(allp?'&#10003;':o.passN)+'</td>';}
    h+='</tr>'; }
  const m=document.getElementById('map'); m.innerHTML=h;
  m.querySelectorAll('td.cell').forEach(td=>td.onclick=()=>{ti=+td.dataset.t;wi=+td.dataset.w;refresh();});
}
const FRAME_MS=20;   // each stored frame = 20 ms of SIM time (50 fps) -> 1x = real time
function loop(now){const dt=now-last;last=now;const F=cur().f;
  if(playing){acc+=dt*speed;while(acc>FRAME_MS){i=(i+1)%F.length;if(i===0)trail=[];trail.push([F[i][2],F[i][15],F[i][3]]);if(trail.length>400)trail.shift();acc-=FRAME_MS;}}
  draw();requestAnimationFrame(loop);}
document.getElementById('ti').oninput=function(){ti=+this.value;refresh();};
document.getElementById('wi').oninput=function(){wi=+this.value;refresh();};
document.getElementById('spd').oninput=function(){speed=this.value/100;document.getElementById('spdv').textContent=speed.toFixed(1)+'x';};
document.getElementById('play').onclick=function(){playing=!playing;this.innerHTML=playing?'&#10073;&#10073; Pause':'&#9658; Play';};
document.getElementById('restart').onclick=()=>{i=0;trail=[];};
document.getElementById('topv').onclick=()=>{az=-1.05;el=1.45;};      // ~top-down (see cross-range spread)
document.getElementById('sidev').onclick=()=>{az=-1.5708;el=0.02;};   // downrange side profile
// ---- orbit: drag to rotate, wheel to zoom ----
let drag=false,mx=0,my=0;
cv.addEventListener('mousedown',e=>{drag=true;mx=e.clientX;my=e.clientY;});
window.addEventListener('mouseup',()=>drag=false);
window.addEventListener('mousemove',e=>{if(!drag)return;az-=(e.clientX-mx)*0.008;el+=(e.clientY-my)*0.008;
  el=Math.max(-0.2,Math.min(1.55,el));mx=e.clientX;my=e.clientY;});
cv.addEventListener('wheel',e=>{e.preventDefault();zoom*=e.deltaY<0?1.1:0.9;zoom=Math.max(0.3,Math.min(4,zoom));},{passive:false});
refresh(); requestAnimationFrame(loop);
</script></body></html>"""
w0 = WINDS.index(0) if 0 in WINDS else 0
HTML = (HTML.replace("__DATA__", data).replace("__TARGETS__", json.dumps(TARGETS)).replace("__WINDS__", json.dumps(WINDS))
            .replace("__APO__", f"{apogee:.1f}").replace("__XMIN__", f"{xmin:.2f}").replace("__XMAX__", f"{xmax:.2f}")
            .replace("__YMAX__", f"{ymax:.2f}")
            .replace("__TN__", str(len(TARGETS)-1)).replace("__WN__", str(len(WINDS)-1)).replace("__W0__", str(w0)))
with open(OUT, "w", encoding="utf-8") as f:
    f.write(HTML)
print(f"wrote {OUT}")
