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
 html,body{margin:0;height:100%;background:#0d1117;color:#c9d1d9;font-family:Segoe UI,system-ui,sans-serif}
 #wrap{display:flex;gap:10px;padding:10px;height:100vh;box-sizing:border-box}
 #view{flex:1;display:flex;flex-direction:column;min-width:0;position:relative}
 canvas#c{flex:1;width:100%;min-height:0;background:linear-gradient(#0b1a2b,#0d1117);border:1px solid #30363d;border-radius:8px}
 canvas#tl{height:34px;width:100%;margin-top:6px;background:#0b1017;border:1px solid #30363d;border-radius:6px;cursor:pointer}
 #side{width:335px;overflow:auto;font-size:13px}
 h2{margin:.2em 0;font-size:16px;color:#58a6ff} .row{margin:5px 0} b{color:#8b949e}
 button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;padding:5px 10px;cursor:pointer}
 button.cam,button.chip,button.pre{font-size:11px;padding:3px 7px;margin:2px 3px 0 0}
 button.on,button.chip.on{background:#1f6feb;border-color:#388bfd;color:#fff}
 button:focus-visible,input:focus-visible{outline:2px solid #58a6ff}
 input[type=range]{width:100%;vertical-align:middle}
 table{border-collapse:collapse;margin-top:6px} td,th{border:1px solid #30363d;padding:3px 6px;text-align:center;font-size:12px}
 td.cell{cursor:pointer} .tag{display:inline-block;padding:1px 8px;border-radius:10px;font-weight:600;color:#fff}
 #wrap.collapsed #side{display:none}
 #help{position:absolute;inset:0;background:rgba(1,4,9,.9);display:none;padding:22px 26px;overflow:auto;border-radius:8px;font-size:13px;line-height:1.6}
 #help.show{display:block} #help h3{color:#58a6ff;margin:.4em 0} #help code{color:#e3b341}
</style></head><body>
<div id="wrap">
 <div id="view">
  <canvas id="c"></canvas>
  <canvas id="tl"></canvas>
  <div id="help">
   <h3>Controls</h3>
   <p><b>Mouse</b> &mdash; drag = orbit (Free/Follow) &middot; wheel = zoom (zoom-to-cursor in Free) &middot; shift/right-drag = pan (Free) &middot; drag the timeline to scrub.</p>
   <h3>Cameras</h3>
   <p><code>1</code> Free orbit &middot; <code>2</code> Follow (centre on rocket) &middot; <code>3</code> Chase (ride behind the flight path) &middot; <code>4</code> Onboard (nose FPV) &middot; <code>5</code> reset camera. <code>f</code> toggles Follow.</p>
   <h3>Transport</h3>
   <p><code>Space</code> play/pause &middot; <code>&larr;/&rarr;</code> step frame (<code>Shift</code> = x10) &middot; <code>Home/End</code> first/last &middot; <code>r</code> restart &middot; <code>[ ]</code> speed &middot; <code>+/-</code> zoom.</p>
   <h3>Layers</h3>
   <p><code>g</code> grid &middot; <code>t</code> trail &middot; <code>e</code> EKF belief &middot; <code>h</code> HUD &middot; <code>m</code> minimap. Chips in the panel toggle every overlay.</p>
   <h3>Legend</h3>
   <p>Trail coloured by phase (BOOST orange, COAST blue, BURN red, TD green). Gray = EKF belief. Green ring = target. Amber arrow = velocity vector.</p>
   <p style="color:#8b949e">Press <code>?</code> or Esc to close.</p>
  </div>
 </div>
 <div id="side">
  <h2>Firmware weakness explorer</h2>
  <div class="row" style="color:#8b949e;font-size:12px">Each cell/slider position is a full run of the EXACT firmware.</div>
  <div class="row">TARGET <input type="range" id="ti" min="0" max="__TN__" value="0"> <span id="tv"></span> m</div>
  <div class="row">WIND <input type="range" id="wi" min="0" max="__WN__" value="__W0__"> <span id="wv"></span> m/s</div>
  <div class="row"><button id="play">&#10073;&#10073; Pause</button> <button id="restart">&#9658; Restart</button>
     speed <input type="range" id="spd" min="20" max="400" value="100" style="width:78px"><span id="spdv">1.0x</span></div>
  <div class="row"><b>Camera</b>&nbsp;
    <button class="cam" id="m_free">Free</button><button class="cam" id="m_follow">Follow</button><button class="cam" id="m_chase">Chase</button><button class="cam" id="m_onboard">Onboard</button></div>
  <div class="row">
    <button class="pre" id="p_top">Top</button><button class="pre" id="p_side">Side</button><button class="pre" id="p_iso">Iso</button><button class="pre" id="p_frame">Frame&nbsp;all</button><button class="pre" id="p_reset">Reset&nbsp;cam</button></div>
  <div class="row" style="font-size:12px">onboard&nbsp; <button class="chip" id="t_roll">level horizon</button><button class="chip" id="t_retro">retro look</button></div>
  <div class="row"><b>Layers</b>&nbsp;
    <button class="chip" data-l="grid">Grid</button><button class="chip" data-l="trail">Trail</button><button class="chip" data-l="shadow">Shadow</button><button class="chip" data-l="ekf">EKF</button><button class="chip" data-l="vel">Velocity</button>
    <button class="chip" data-l="hud">HUD</button><button class="chip" data-l="att">Attitude</button><button class="chip" data-l="gauge">Gauge</button><button class="chip" data-l="map">Minimap</button></div>
  <div class="row">
    <button id="fs">Fullscreen</button><button id="collapse">Hide panel</button>
    <button class="chip" id="t_loop">loop</button><button class="chip" id="t_auto">auto-pause</button><button id="helpb">? Help</button></div>
  <hr style="border-color:#30363d">
  <div class="row">outcome <span class="tag" id="verdict"></span></div>
  <div class="row">t <b id="t"></b>s &middot; phase <b id="ph"></b> &middot; alt <b id="alt"></b> m</div>
  <div class="row">downrange <b id="dr"></b> m &middot; crossrange <b id="cr"></b> m &middot; roll <b id="rl"></b>&deg;</div>
  <div class="row">miss <b id="miss"></b> m &middot; touchdown vz <b id="tvz"></b> m/s &middot; tilt <b id="tilt"></b>&deg;</div>
  <div class="row">apogee <b id="apo"></b> m</div>
  <hr style="border-color:#30363d">
  <div class="row"><b>PASS/FAIL map</b> (rows=target, cols=wind; click a cell). green=soft on-target, red=fail.</div>
  <div class="row" style="color:#d29922;font-size:12px">WIND/TARGET robustness only (nominal build+motor). Build/motor/timing dispersion is separate &mdash; see montecarlo.py.</div>
  <table id="map"></table>
  <div class="row" style="color:#8b949e;font-size:11px;margin-top:8px">Drag=orbit &middot; wheel=zoom &middot; shift/right-drag=pan (Free) &middot; drag the timeline to scrub. Keys: Space, &larr;&rarr;, Home/End, 1-4 cameras, f, r, [ ], ?.</div>
 </div>
</div>
<script>
const G=__DATA__, TARGETS=__TARGETS__, WINDS=__WINDS__, APO=__APO__, XMIN=__XMIN__, XMAX=__XMAX__, YMAX=__YMAX__;
const cv=document.getElementById('c'),cx=cv.getContext('2d');
const tl=document.getElementById('tl'),tlx=tl.getContext('2d');
let W=820,H=620,DPR=1,TLW=820,TLH=34;
const PHc={BOOST:'#f0883e',COAST:'#58a6ff',BURN:'#f85149',TD:'#3fb950',ABORT:'#f85149',PAD:'#8b949e',ARM:'#8b949e',CD:'#d29922'};
const RM=!!(window.matchMedia&&window.matchMedia('(prefers-reduced-motion:reduce)').matches);
// ================= math =================
function dot(a,b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
function nrm(v){const m=Math.hypot(v[0],v[1],v[2])||1;return [v[0]/m,v[1]/m,v[2]/m];}
function cross(a,b){return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function clamp(x,a,b){return x<a?a:(x>b?b:x);}
function qrot(q,v){const w=q[0],x=q[1],y=q[2],z=q[3];   // rotate a body vector to world by quaternion
  return [(1-2*(y*y+z*z))*v[0]+2*(x*y-w*z)*v[1]+2*(x*z+w*y)*v[2],
          2*(x*y+w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-w*x)*v[2],
          2*(x*z-w*y)*v[0]+2*(y*z+w*x)*v[1]+(1-2*(x*x+y*y))*v[2]];}
// ================= scene / camera state =================
// World axes: X = downrange, Y = crossrange, Z = up. Body +Z = the rocket's nose.
const zTop=Math.max(APO*1.15,10);
let WSC=(H-150)/zTop;                                 // world metres -> px (recomputed on resize)
const CEN0=[Math.max(XMAX,8)*0.45, 0, zTop*0.42];     // home look-at
let CEN=CEN0.slice(), CENt=CEN0.slice();              // live + target look-at
let CAMD=zTop*2.0;                                    // eye distance from CEN along +Fv (set per mode) -> PERSPECTIVE
const NEAR=0.03;                                      // near plane (m): projW clamp + gclip cull share it (small: onboard cam sits ~cm from the tube)
const R_AX=0.30, R_RAD=0.194;                         // model-unit -> metres (anisotropic): TRUE physical scale, rocket ~0.675 m long x 0.062 m dia
const SUN=nrm([-0.35,0.5,0.85]);                      // fixed world sun (toward-light) for stable shading
let az=-1.05, el=0.30, zoom=1.0;                      // orbit azimuth/elevation (rad) + zoom
let azT=az, elT=el, zoomT=1.0;                        // eased targets (smooth view changes)
let camMode='free', headingAz=az, rollStab=true, retroLook=false;
let Rv=[0,1,0], Uv=[0,0,1], Fv=[1,0,0];               // cached camera basis (screen-right / up / toward-camera)
function orbitBasis(){const ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);
  Rv[0]=-sa;Rv[1]=ca;Rv[2]=0; Uv[0]=-se*ca;Uv[1]=-se*sa;Uv[2]=ce; Fv[0]=ce*ca;Fv[1]=ce*sa;Fv[2]=se;}
orbitBasis();
// ================= projection (reads cached basis + CEN + CAMD; eye = CEN + CAMD*Fv, looks along -Fv) =================
function projW(p){const dx0=p[0]-CEN[0],dx1=p[1]-CEN[1],dx2=p[2]-CEN[2];   // -> [sx,sy,vd(depth-in-front),perspFactor]
  const vd=CAMD-(dx0*Fv[0]+dx1*Fv[1]+dx2*Fv[2]); const dp=vd<NEAR?NEAR:vd; const pf=CAMD/dp, sc=WSC*zoom*pf;
  return [W/2+sc*(dx0*Rv[0]+dx1*Rv[1]+dx2*Rv[2]), H/2-sc*(dx0*Uv[0]+dx1*Uv[1]+dx2*Uv[2]), vd, pf];}
function vdOf(p){return CAMD-((p[0]-CEN[0])*Fv[0]+(p[1]-CEN[1])*Fv[1]+(p[2]-CEN[2])*Fv[2]);}
function clipSeg(a,b){let ax=a,bx=b,vA=vdOf(a),vB=vdOf(b);   // near-plane clip a world segment (behind-eye smear guard)
  if(vA<NEAR&&vB<NEAR)return null;
  if(vA<NEAR){const t=(NEAR-vA)/(vB-vA);ax=[a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*(b[2]-a[2])];}
  else if(vB<NEAR){const t=(NEAR-vB)/(vA-vB);bx=[b[0]+t*(a[0]-b[0]),b[1]+t*(a[1]-b[1]),b[2]+t*(a[2]-b[2])];}
  return [projW(ax),projW(bx)];}
function gclip(a,b,col,dash,wd){const s=clipSeg(a,b);if(!s)return;cx.strokeStyle=col;if(wd)cx.lineWidth=wd;if(dash)cx.setLineDash(dash);
  cx.beginPath();cx.moveTo(s[0][0],s[0][1]);cx.lineTo(s[1][0],s[1][1]);cx.stroke();if(dash)cx.setLineDash([]);if(wd)cx.lineWidth=1;}
function shade(hex,br){const n=parseInt(hex.slice(1),16),r=(n>>16)&255,g=(n>>8)&255,b=n&255,f=Math.max(0.30,Math.min(1,br));
  return 'rgb('+(r*f|0)+','+(g*f|0)+','+(b*f|0)+')';}
function rr(x,y,w,h,r){cx.beginPath();cx.moveTo(x+r,y);cx.arcTo(x+w,y,x+w,y+h,r);cx.arcTo(x+w,y+h,x,y+h,r);cx.arcTo(x,y+h,x,y,r);cx.arcTo(x,y,x+w,y,r);cx.closePath();}
// ================= the rocket (true attitude from quaternion; fixed-sun shading + exhaust glow) =================
function rocketFaces(pos,q,dep,rdf,thr,uT,uT2,legF){
  const ps=projW(pos), faces=[];
  // TRUE physical scale: model units * [R_RAD,R_RAD,R_AX] -> metres -> world -> perspective, so the rocket
  // scales with zoom exactly like the world. Each vertex is near-clipped (behind-eye -> null -> face() drops it).
  const V=vb=>{const vw=qrot(q,[vb[0]*R_RAD,vb[1]*R_RAD,vb[2]*R_AX]),wp=[pos[0]+vw[0],pos[1]+vw[1],pos[2]+vw[2]],vd=vdOf(wp);
    if(vd<NEAR)return null;const P=projW(wp);return [P[0],P[1],-vd];};
  const face=(vs,col,br,strk)=>{for(const v of vs)if(!v)return;let d=0;for(const v of vs)d+=v[2];faces.push({p:vs,d:d/vs.length,c:col,b:br==null?1:br,s:strk});};
  const rB=0.16, bot=-1.0, top=0.85, tip=1.25, NS=14;
  const lit=a=>{const nw=qrot(q,[Math.cos(a),Math.sin(a),0]);return 0.5+0.5*Math.max(0,dot(nw,SUN));};
  for(let k=0;k<NS;k++){const a0=k/NS*2*Math.PI,a1=(k+1)/NS*2*Math.PI;   // body tube
    face([V([rB*Math.cos(a0),rB*Math.sin(a0),bot]),V([rB*Math.cos(a1),rB*Math.sin(a1),bot]),
          V([rB*Math.cos(a1),rB*Math.sin(a1),top]),V([rB*Math.cos(a0),rB*Math.sin(a0),top])],'#dfe7ef',lit((a0+a1)/2));}
  for(let k=0;k<NS;k++){const a0=k/NS*2*Math.PI,a1=(k+1)/NS*2*Math.PI;   // nose cone
    face([V([rB*Math.cos(a0),rB*Math.sin(a0),top]),V([rB*Math.cos(a1),rB*Math.sin(a1),top]),V([0,0,tip])],'#f85149',lit((a0+a1)/2));}
  const finZ=0.42, span=0.75, chord=0.34;             // 4 fold-out margin fins (canards near the nose)
  for(let f=0;f<4;f++){const phi=f*Math.PI/2, rad=[Math.cos(phi),Math.sin(phi),0], tan=[-Math.sin(phi),Math.cos(phi),0];
    let fa=(dep + rdf*(f%2?-1:1)*0.5)*Math.PI/2; fa=Math.max(0,Math.min(Math.PI/2,fa));   // differential fold -> roll
    const fold=[Math.sin(fa)*rad[0],Math.sin(fa)*rad[1],-Math.cos(fa)];
    const h=[rB*rad[0],rB*rad[1],finZ];
    const c0=V(h), c1=V([h[0],h[1],h[2]+chord]),
          c2=V([h[0]+span*fold[0],h[1]+span*fold[1],h[2]+chord+span*fold[2]]), c3=V([h[0]+span*fold[0],h[1]+span*fold[1],h[2]+span*fold[2]]);
    face([c0,c1,c2,c3],'#4d9dff',0.55+0.45*Math.abs(dot(qrot(q,tan),SUN)),'#2f6fb0');}
  if(legF>0.01){for(let f=0;f<4;f++){const phi=f*Math.PI/2+Math.PI/4, rad=[Math.cos(phi),Math.sin(phi),0], tan=[-Math.sin(phi),Math.cos(phi),0];   // 4 legs, 45deg off the fins
    const a=0.35+legF*(2.62-0.35), dir=[Math.sin(a)*rad[0],Math.sin(a)*rad[1],Math.cos(a)], h=[rB*rad[0],rB*rad[1],bot+0.15], wgt=0.05, LL=0.62;   // fold DOWN: stowed up(a small)->deployed down+out(a large), a from +Z
    const knee=[h[0]+LL*dir[0],h[1]+LL*dir[1],h[2]+LL*dir[2]];
    face([V([h[0]+wgt*tan[0],h[1]+wgt*tan[1],h[2]]),V([h[0]-wgt*tan[0],h[1]-wgt*tan[1],h[2]]),
          V([knee[0]-wgt*tan[0],knee[1]-wgt*tan[1],knee[2]]),V([knee[0]+wgt*tan[0],knee[1]+wgt*tan[1],knee[2]])],'#3fb950',0.9);
    face([V([knee[0],knee[1],knee[2]]),V([knee[0]+0.14*rad[0],knee[1]+0.14*rad[1],knee[2]-0.02]),V([knee[0]+0.14*rad[0]+0.02*tan[0],knee[1]+0.14*rad[1],knee[2]+0.03])],'#2f9e42',0.9);}}
  if(thr>0.5){const nvw=qrot(q,[0,0,bot*R_AX]),nw=[pos[0]+nvw[0],pos[1]+nvw[1],pos[2]+nvw[2]];   // exhaust: radial glow (world-scale) + flame polys
    if(vdOf(nw)>=NEAR){const NP=projW(nw),gl=Math.max(3,Math.min(300,(0.05+Math.min(thr,30)*0.008)*WSC*zoom*NP[3]));   // px per metre = WSC*zoom*pf
      const rg=cx.createRadialGradient(NP[0],NP[1],0,NP[0],NP[1],gl);rg.addColorStop(0,'rgba(255,185,80,0.55)');rg.addColorStop(1,'rgba(255,140,40,0)');
      cx.fillStyle=rg;cx.beginPath();cx.arc(NP[0],NP[1],gl,0,2*Math.PI);cx.fill();}
    const dir=nrm([(uT||0)*0.03,(uT2||0)*0.03,-1]), fl=0.55+Math.min(thr,30)*0.02, tipf=[dir[0]*fl,dir[1]*fl,bot+dir[2]*fl];
    for(let k=0;k<8;k++){const a0=k/8*2*Math.PI,a1=(k+1)/8*2*Math.PI,rrn=0.11;
      face([V([rrn*Math.cos(a0),rrn*Math.sin(a0),bot]),V([rrn*Math.cos(a1),rrn*Math.sin(a1),bot]),V(tipf)],k%2?'#ffd23f':'#ff8c42',1.0);}}
  faces.sort((a,b)=>a.d-b.d);   // painter: far (small toward-camera depth) first
  for(const f of faces){cx.beginPath();cx.moveTo(f.p[0][0],f.p[0][1]);for(let j=1;j<f.p.length;j++)cx.lineTo(f.p[j][0],f.p[j][1]);cx.closePath();
    cx.fillStyle=shade(f.c,f.b);cx.fill();if(f.s){cx.strokeStyle=f.s;cx.lineWidth=1;cx.stroke();}}
  return ps;
}
// ================= replay state =================
let ti=0, wi=(WINDS.indexOf(0)<0?0:WINDS.indexOf(0)), i=0, playing=true, speed=1.0;
let trail=[], trailE=[], runs=[], prevPh='', loopMode=true, autopause=false;
let last=performance.now(), acc=0, lastCam=performance.now();
const FRAME_MS=20;   // each stored frame = 20 ms of SIM time (50 fps) -> 1x = real time
const LYR={grid:1,trail:1,shadow:1,target:1,ekf:0,vel:1,hud:1,att:1,gauge:1,map:1};
function cur(){return G[ti][wi];}
function frameVel(F,k){const D=Math.min(5,k);if(D<1)return [0,0,0];const a=F[k],b=F[k-D];let dts=a[0]-b[0];if(dts<=0)dts=D*FRAME_MS/1000;
  return [(a[2]-b[2])/dts,(a[15]-b[15])/dts,(a[3]-b[3])/dts];}   // world velocity from finite-diff (robust to pause)
// ---- camera update (one place fills Rv/Uv/Fv + CEN + CAMD for the active mode) ----
function updateCamera(f,velW,dt){
  const kk=RM?1:(1-Math.exp(-dt/0.18));
  const pos=[f[2],f[15],f[3]], q=[f[16],f[17],f[18],f[19]];
  if(camMode==='onboard'){
    CAMD=zTop*0.8;                                       // sets the onboard FOV (focal = WSC*zoom*CAMD ~ 80deg)
    const bz=qrot(q,[0,0,1]), bx=qrot(q,[1,0,0]);       // body nose (+Z) and mount side (+X)
    const mountR=0.08, mountZ=0.10;                      // camera on the SIDE of the ~3.1 cm-radius tube: ~5 cm standoff, 10 cm up (m)
    const eye=[pos[0]+bx[0]*mountR+bz[0]*mountZ, pos[1]+bx[1]*mountR+bz[1]*mountZ, pos[2]+bx[2]*mountR+bz[2]*mountZ];
    // look DOWN the body (-Z) tilted slightly INWARD (-X, toward the tube) so the body wall + legs + ground fill the frame;
    // 'retro look' instead aims down the velocity vector (watch the pad rise during the burn)
    const sp=Math.hypot(velW[0],velW[1],velW[2]);
    const viewDir=(retroLook&&sp>0.8)?nrm([-velW[0],-velW[1],-velW[2]])
                                     :nrm([-bz[0]-0.3*bx[0],-bz[1]-0.3*bx[1],-bz[2]-0.3*bx[2]]);
    Fv[0]=-viewDir[0];Fv[1]=-viewDir[1];Fv[2]=-viewDir[2];
    let upH=rollStab?bz:bx;                              // screen-up toward the nose (world-up is degenerate looking straight down)
    let d=upH[0]*Fv[0]+upH[1]*Fv[1]+upH[2]*Fv[2];
    if(Math.abs(d)>0.985){upH=rollStab?bx:bz;d=upH[0]*Fv[0]+upH[1]*Fv[1]+upH[2]*Fv[2];}   // gimbal guard: pick the OTHER body axis (bx perp bz)
    const u=nrm([upH[0]-d*Fv[0],upH[1]-d*Fv[1],upH[2]-d*Fv[2]]);Uv[0]=u[0];Uv[1]=u[1];Uv[2]=u[2];
    const rrv=cross(Uv,Fv);Rv[0]=rrv[0];Rv[1]=rrv[1];Rv[2]=rrv[2];                        // R = U x F handedness
    CEN[0]=eye[0]+CAMD*viewDir[0];CEN[1]=eye[1]+CAMD*viewDir[1];CEN[2]=eye[2]+CAMD*viewDir[2]; // so projW eye = CEN+CAMD*Fv = eye
    zoom+=(zoomT-zoom)*kk; return;
  }
  if(camMode==='chase'){CAMD=zTop*0.9;const vh=Math.hypot(velW[0],velW[1]);if(vh>0.5)headingAz=Math.atan2(velW[1],velW[0]);azT=headingAz+Math.PI;elT=0.34;}
  else{CAMD=zTop*2.0;}
  let da=azT-az;da=Math.atan2(Math.sin(da),Math.cos(da));az+=da*kk;   // ease az the short way
  el+=(elT-el)*kk;zoom+=(zoomT-zoom)*kk;el=clamp(el,-0.25,1.55);orbitBasis();
  if(camMode==='follow'||camMode==='chase'){CENt[0]=pos[0];CENt[1]=pos[1];CENt[2]=pos[2];}
  CEN[0]+=(CENt[0]-CEN[0])*kk;CEN[1]+=(CENt[1]-CEN[1])*kk;CEN[2]+=(CENt[2]-CEN[2])*kk;
}
// ---- overlays ----
function drawTrail(arr,fade){if(arr.length<2)return;cx.lineWidth=2;
  for(let k=1;k<arr.length;k++){const s=clipSeg(arr[k-1],arr[k]);if(!s)continue;
    cx.strokeStyle=arr[k][3]||'#1f6feb';cx.globalAlpha=fade?clamp(0.15+0.85*k/arr.length,0.1,1):0.55;
    cx.beginPath();cx.moveTo(s[0][0],s[0][1]);cx.lineTo(s[1][0],s[1][1]);cx.stroke();}
  cx.globalAlpha=1;cx.lineWidth=1;}
function drawFPV(f,r,velW,pos){
  cx.strokeStyle='rgba(130,205,130,0.55)';cx.lineWidth=1;cx.beginPath();
  cx.moveTo(W/2-15,H/2);cx.lineTo(W/2-5,H/2);cx.moveTo(W/2+5,H/2);cx.lineTo(W/2+15,H/2);
  cx.moveTo(W/2,H/2-15);cx.lineTo(W/2,H/2-5);cx.moveTo(W/2,H/2+5);cx.lineTo(W/2,H/2+15);cx.stroke();
  const sp=Math.hypot(velW[0],velW[1],velW[2]);
  if(sp>0.5){const vh=nrm(velW),fp=[pos[0]+vh[0]*8,pos[1]+vh[1]*8,pos[2]+vh[2]*8];
    if(vdOf(fp)>=NEAR){const P=projW(fp);cx.strokeStyle='#e3b341';cx.beginPath();cx.arc(P[0],P[1],6,0,2*Math.PI);
      cx.moveTo(P[0]-11,P[1]);cx.lineTo(P[0]-6,P[1]);cx.moveTo(P[0]+6,P[1]);cx.lineTo(P[0]+11,P[1]);cx.moveTo(P[0],P[1]-11);cx.lineTo(P[0],P[1]-6);cx.stroke();}}
  if(vdOf([r.T,0,0])>=NEAR){const T=projW([r.T,0,0]);cx.fillStyle='#3fb950';cx.beginPath();
    cx.moveTo(T[0],T[1]-6);cx.lineTo(T[0]+6,T[1]);cx.lineTo(T[0],T[1]+6);cx.lineTo(T[0]-6,T[1]);cx.closePath();cx.stroke();}
}
function drawHUD(f,r){
  const roll=2*Math.atan2(f[19],f[16])*180/Math.PI, miss=Math.hypot(f[2]-r.T,f[15]);
  const lines=[['alt',f[3].toFixed(1)+' m'],['vz',f[12].toFixed(1)+' m/s'],['tilt',f[4].toFixed(1)+'°'],['roll',roll.toFixed(0)+'°'],
    ['downrange',f[2].toFixed(1)+' m'],['crossrange',f[15].toFixed(2)+' m'],['thrust',f[5].toFixed(0)+' N'],['keff',f[11].toFixed(1)],['miss',miss.toFixed(2)+' m']];
  const x=10,y=10,w=182,h=34+lines.length*15;
  cx.fillStyle='rgba(13,17,23,0.62)';rr(x,y,w,h,6);cx.fill();
  cx.font='12px monospace';cx.fillStyle=PHc[f[1]]||'#c9d1d9';cx.fillRect(x+8,y+8,10,10);
  cx.fillStyle='#e6edf3';cx.fillText(f[0].toFixed(2)+'s  '+f[1],x+24,y+17);
  let yy=y+34;for(const ln of lines){cx.fillStyle='#8b949e';cx.fillText(ln[0],x+8,yy);cx.fillStyle='#e6edf3';cx.fillText(ln[1],x+96,yy);yy+=15;}
}
function drawAttitude(f){
  const R=50,X=W-R-16,Y=R+16,q=[f[16],f[17],f[18],f[19]],noseW=qrot(q,[0,0,1]);
  const tilt=f[4],lean=Math.atan2(noseW[1],noseW[0]),roll=2*Math.atan2(f[19],f[16]);
  cx.save();cx.beginPath();cx.arc(X,Y,R,0,2*Math.PI);cx.fillStyle='rgba(13,17,23,0.66)';cx.fill();cx.clip();
  cx.translate(X,Y);cx.rotate(-roll);const pit=clamp(tilt/45,0,1)*R*0.8;
  cx.fillStyle='rgba(60,110,180,0.30)';cx.fillRect(-R,-R,2*R,R+pit);
  cx.strokeStyle='#9fb4d0';cx.lineWidth=1.5;cx.beginPath();cx.moveTo(-R,pit);cx.lineTo(R,pit);cx.stroke();cx.restore();
  cx.save();cx.strokeStyle='#30363d';cx.lineWidth=1;cx.beginPath();cx.arc(X,Y,R,0,2*Math.PI);cx.stroke();
  cx.strokeStyle='#c9d1d9';cx.beginPath();cx.moveTo(X-9,Y);cx.lineTo(X-3,Y);cx.moveTo(X+3,Y);cx.lineTo(X+9,Y);cx.stroke();
  const rp=clamp(tilt/30,0,1)*(R-8),px=X+Math.cos(lean)*rp,py=Y-Math.sin(lean)*rp;
  cx.fillStyle=tilt>15?'#f85149':'#3fb950';cx.beginPath();cx.arc(px,py,4,0,2*Math.PI);cx.fill();
  cx.fillStyle='#8b949e';cx.font='10px monospace';cx.textAlign='center';cx.fillText('tilt '+tilt.toFixed(0)+'° roll '+(roll*180/Math.PI).toFixed(0)+'°',X,Y+R+12);cx.textAlign='left';cx.restore();
}
function drawGauge(f){
  const x=12,y=H-100,w=150,h=88;cx.fillStyle='rgba(13,17,23,0.62)';rr(x,y,w,h,6);cx.fill();
  cx.font='10px monospace';cx.fillStyle='#8b949e';cx.fillText('thrust',x+6,y+13);cx.fillText('TVC',x+58,y+13);cx.fillText('fin',x+118,y+13);
  const th=clamp(f[5]/30,0,1);cx.fillStyle='#30363d';cx.fillRect(x+8,y+18,14,62);cx.fillStyle='#f0883e';cx.fillRect(x+8,y+18+62*(1-th),14,62*th);
  cx.strokeStyle='#f85149';cx.lineWidth=1;cx.beginPath();cx.moveTo(x+6,y+18+62*(1-28/30));cx.lineTo(x+24,y+18+62*(1-28/30));cx.stroke();
  const gx=x+66,gy=y+50,gr=22;cx.strokeStyle='#30363d';cx.strokeRect(gx-gr,gy-gr,2*gr,2*gr);
  cx.strokeStyle='#233049';cx.beginPath();cx.moveTo(gx-gr,gy);cx.lineTo(gx+gr,gy);cx.moveTo(gx,gy-gr);cx.lineTo(gx,gy+gr);cx.stroke();
  const GL=6,ddx=clamp(f[9]/GL,-1,1)*gr,ddy=clamp(f[21]/GL,-1,1)*gr,sat=Math.abs(f[9])>=GL||Math.abs(f[21])>=GL;
  cx.fillStyle=sat?'#f85149':'#58a6ff';cx.beginPath();cx.arc(gx+ddx,gy-ddy,4,0,2*Math.PI);cx.fill();
  const fx=x+120,fy=y+72,fr=20;cx.strokeStyle='#30363d';cx.beginPath();cx.arc(fx,fy,fr,-Math.PI,-Math.PI/2);cx.stroke();
  const dp=clamp(f[10],0,1),ang=-Math.PI+dp*Math.PI/2;cx.strokeStyle='#4d9dff';cx.lineWidth=2;cx.beginPath();cx.moveTo(fx,fy);cx.lineTo(fx+fr*Math.cos(ang),fy+fr*Math.sin(ang));cx.stroke();cx.lineWidth=1;
}
function drawMinimap(f,r){
  const w=150,h=134,x=W-w-14,y=H-h-14;cx.fillStyle='rgba(13,17,23,0.72)';rr(x,y,w,h,6);cx.fill();
  cx.strokeStyle='#30363d';cx.strokeRect(x+6,y+18,w-12,h-26);
  cx.fillStyle='#8b949e';cx.font='9px monospace';cx.fillText('top-down  down↑ cross→',x+8,y+13);
  const rangeX=Math.max(XMAX,r.T)+6, rangeY=Math.max(YMAX,4)+2;
  const sc=Math.min((w-24)/(2*rangeY),(h-34)/rangeX), padX=x+w*0.5, padY=y+h-16;
  const mp=(px,py)=>[padX+py*sc, padY-px*sc];
  const tp=mp(r.T,0);cx.strokeStyle='#3fb950';cx.lineWidth=1;cx.beginPath();cx.arc(tp[0],tp[1],Math.max(3,1.2*sc),0,2*Math.PI);cx.stroke();
  const pp=mp(0,0);cx.fillStyle='#8b949e';cx.fillRect(pp[0]-2,pp[1]-2,4,4);
  cx.strokeStyle='#1f6feb';cx.beginPath();for(let k=0;k<trail.length;k++){const P=mp(trail[k][0],trail[k][1]);k?cx.lineTo(P[0],P[1]):cx.moveTo(P[0],P[1]);}cx.stroke();
  const ep=mp(f[6],f[22]);cx.strokeStyle='#8b949e';cx.beginPath();cx.arc(ep[0],ep[1],3,0,2*Math.PI);cx.stroke();
  const rpp=mp(f[2],f[15]);cx.fillStyle='#f85149';cx.beginPath();cx.arc(rpp[0],rpp[1],3.5,0,2*Math.PI);cx.fill();
}
function drawModeLabel(){const t='CAM: '+camMode.toUpperCase()+(camMode==='onboard'?(rollStab?' (level)':' (body-roll)')+(retroLook?' retro':''):'');
  cx.font='11px monospace';const w=cx.measureText(t).width+14;cx.fillStyle='rgba(13,17,23,0.5)';rr(W/2-w/2,8,w,18,4);cx.fill();
  cx.fillStyle='#58a6ff';cx.textAlign='center';cx.fillText(t,W/2,21);cx.textAlign='left';}
function setT(id,v){const e=document.getElementById(id);if(e)e.textContent=v;}
function updateReadouts(f){setT('t',f[0].toFixed(2));setT('ph',f[1]);setT('alt',f[3].toFixed(1));setT('dr',f[2].toFixed(1));
  setT('cr',f[15].toFixed(2));setT('rl',(2*Math.atan2(f[19],f[16])*180/Math.PI).toFixed(0));}
// ================= main draw =================
function draw(){
  const cnow=performance.now();let cdt=(cnow-lastCam)/1000;lastCam=cnow;if(cdt>0.05)cdt=0.05;
  const r=cur(),F=r.f;if(i>=F.length)i=F.length-1;if(i<0)i=0;const f=F[i],pos=[f[2],f[15],f[3]];
  const velW=frameVel(F,i);
  updateCamera(f,velW,cdt);
  cx.clearRect(0,0,W,H);
  if(LYR.grid){const gx0=-4,gx1=Math.max(XMAX,r.T)+4,gyext=Math.max(YMAX,3)+2;
    for(let gx=Math.ceil(gx0/5)*5;gx<=gx1;gx+=5)gclip([gx,-gyext,0],[gx,gyext,0],'#22314a');
    for(let gyv=-Math.floor(gyext/5)*5;gyv<=gyext;gyv+=5)gclip([gx0,gyv,0],[gx1,gyv,0],'#22314a');}
  if(LYR.target){gclip([-1,0,0],[1,0,0],'#8b949e');gclip([0,-1,0],[0,1,0],'#8b949e');
    const TR=1.2;cx.strokeStyle='#3fb950';cx.lineWidth=2;cx.beginPath();let started=false;
    for(let k=0;k<=24;k++){const a=k/24*2*Math.PI,pt=[r.T+TR*Math.cos(a),TR*Math.sin(a),0];
      if(vdOf(pt)<NEAR){started=false;continue;}const P=projW(pt);if(!started){cx.moveTo(P[0],P[1]);started=true;}else cx.lineTo(P[0],P[1]);}
    cx.stroke();cx.lineWidth=1;
    if(vdOf([r.T,0,0])>=NEAR){const tp=projW([r.T,0,0]);cx.fillStyle='#3fb950';cx.font='11px monospace';cx.fillText('target '+r.T+'m',tp[0]+6,tp[1]-4);}}
  if(LYR.shadow&&camMode!=='onboard'){const gp0=[f[2],f[15],0];
    if(vdOf(gp0)>=NEAR){const Gp=projW(gp0),rx=Math.max(2,Math.min(400,0.22*WSC*zoom*Gp[3])),ry=Math.max(1,rx*(0.25+0.75*Math.abs(Math.sin(el))));
      cx.save();cx.globalAlpha=clamp(0.4/(1+f[3]*0.04),0.06,0.4);cx.fillStyle='#000';cx.beginPath();cx.ellipse(Gp[0],Gp[1],rx,ry,0,0,2*Math.PI);cx.fill();cx.restore();}
    gclip([f[2],f[15],f[3]],[f[2],f[15],0],'rgba(180,200,230,0.25)',[4,4]);}
  if(LYR.trail)drawTrail(trail,true);
  if(LYR.ekf){drawTrail(trailE,false);gclip(pos,[f[6],f[22],f[7]],'#8b949e',[2,2]);
    if(vdOf([f[6],f[22],f[7]])>=NEAR){const gpx=projW([f[6],f[22],f[7]]);cx.strokeStyle='#8b949e';cx.setLineDash([3,3]);cx.beginPath();cx.arc(gpx[0],gpx[1],5,0,2*Math.PI);cx.stroke();cx.setLineDash([]);}}
  if(LYR.vel){const sp=Math.hypot(velW[0],velW[1],velW[2]);if(sp>0.4){const L=clamp(sp*0.2,0.6,Math.max(3,zTop*0.12)),vh=nrm(velW);
    gclip(pos,[pos[0]+vh[0]*L,pos[1]+vh[1]*L,pos[2]+vh[2]*L],'#e3b341',null,2);}}
  {
    let legs=false,chute=false,legFire=-1;for(let k=0;k<=i;k++){if(F[k][13]){legs=true;if(legFire<0)legFire=k;}if(F[k][14])chute=true;}
    const legF=legFire<0?0:Math.max(0,Math.min(1,(i-legFire)/25));
    const ps=(camMode==='onboard'||vdOf(pos)>=NEAR)?rocketFaces(pos,[f[16],f[17],f[18],f[19]],f[10],f[20],f[5],f[9],f[21],legF):null;   // true-scale rocket; skip if center behind eye (Free pan)
    if(ps&&camMode!=='onboard'){
      if(legs){cx.fillStyle='#3fb950';cx.font='12px sans-serif';cx.fillText('LEGS',ps[0]+22,ps[1]);}
      if(chute){cx.fillStyle='#d29922';cx.fillText('CHUTE',ps[0]+22,ps[1]-14);}
    }
  }
  if(camMode==='onboard')drawFPV(f,r,velW,pos);
  if(LYR.hud)drawHUD(f,r);
  if(LYR.att)drawAttitude(f);
  if(LYR.gauge)drawGauge(f);
  if(LYR.map)drawMinimap(f,r);
  drawModeLabel();
  drawTimeline();
  updateReadouts(f);
}
// ================= timeline (separate canvas -> its own pointer handlers) =================
function computeRuns(F){const rns=[];let s=0;for(let k=1;k<=F.length;k++){if(k===F.length||F[k][1]!==F[s][1]){rns.push({i0:s,i1:k-1,ph:F[s][1]});s=k;}}return rns;}
function drawTimeline(){const F=cur().f,N=F.length;tlx.clearRect(0,0,TLW,TLH);
  for(const rn of runs){const x0=rn.i0/N*TLW,x1=(rn.i1+1)/N*TLW;tlx.fillStyle=PHc[rn.ph]||'#30363d';tlx.fillRect(x0,3,Math.max(1,x1-x0),TLH-12);}
  const px=i/N*TLW;tlx.strokeStyle='#fff';tlx.lineWidth=2;tlx.beginPath();tlx.moveTo(px,0);tlx.lineTo(px,TLH);tlx.stroke();
  tlx.fillStyle='#c9d1d9';tlx.font='10px monospace';tlx.fillText(F[i][0].toFixed(2)+' / '+F[N-1][0].toFixed(2)+'s  ['+F[i][1]+']',5,TLH-2);}
function rebuildTrail(){trail=[];trailE=[];const F=cur().f,s=Math.max(0,i-400);
  for(let k=s;k<=i&&k<F.length;k++){trail.push([F[k][2],F[k][15],F[k][3],PHc[F[k][1]]||'#1f6feb']);trailE.push([F[k][6],F[k][22],F[k][7],'#8b949e']);}}
let scrubbing=false;
function tlSeek(e){const rc=tl.getBoundingClientRect(),N=cur().f.length;i=clamp(Math.round((e.clientX-rc.left)/rc.width*N),0,N-1);playing=false;syncPlay();rebuildTrail();}
tl.addEventListener('pointerdown',e=>{scrubbing=true;tl.setPointerCapture(e.pointerId);tlSeek(e);});
tl.addEventListener('pointermove',e=>{if(scrubbing)tlSeek(e);});
tl.addEventListener('pointerup',()=>{scrubbing=false;});
// ================= panel / heatmap =================
function refresh(){const o=cur().o;
  setT('tv',TARGETS[ti]);setT('wv',WINDS[wi]);
  const v=document.getElementById('verdict'),allpass=o.passN===o.nSeed;v.textContent=o.passN+'/'+o.nSeed+' seeds pass';v.style.background=allpass?'#238636':'#da3633';
  setT('miss',o.miss.toFixed(2)+' (median; mean '+o.missMean.toFixed(2)+', worst '+o.missWorst.toFixed(2)+')');
  setT('tvz',o.vz.toFixed(2));setT('tilt',o.tilt.toFixed(1));setT('apo',o.apogee.toFixed(0));
  document.getElementById('ti').value=ti;document.getElementById('wi').value=wi;
  i=0;runs=computeRuns(cur().f);rebuildTrail();drawMap();}
function drawMap(){let h='<tr><th>t\\w</th>';for(const w of WINDS)h+='<th>'+w+'</th>';h+='</tr>';
  for(let t=0;t<TARGETS.length;t++){h+='<tr><th>'+TARGETS[t]+'</th>';
    for(let w=0;w<WINDS.length;w++){const o=G[t][w].o,allp=o.passN===o.nSeed,c=allp?'#196c2e':(o.passN>0?'#8a6d1a':'#8b1a1a'),sel=(t===ti&&w===wi)?'outline:2px solid #58a6ff;':'';
      h+='<td class="cell" style="background:'+c+';'+sel+'" data-t="'+t+'" data-w="'+w+'" title="pass '+o.passN+'/'+o.nSeed+' | miss mean '+o.missMean.toFixed(1)+' worst '+o.missWorst.toFixed(1)+'">'+(allp?'&#10003;':o.passN)+'</td>';}
    h+='</tr>';}
  const m=document.getElementById('map');m.innerHTML=h;
  m.querySelectorAll('td.cell').forEach(td=>td.onclick=()=>{ti=+td.dataset.t;wi=+td.dataset.w;refresh();});}
// ================= playback loop =================
function syncPlay(){const b=document.getElementById('play');if(b)b.innerHTML=playing?'&#10073;&#10073; Pause':'&#9658; Play';}
function loop(now){const dt=now-last;last=now;const F=cur().f;
  if(playing){acc+=dt*speed;let guard=0;while(acc>FRAME_MS&&guard++<2000){
    i++;if(i>=F.length){if(loopMode){i=0;trail=[];trailE=[];}else{i=F.length-1;playing=false;syncPlay();acc=0;break;}}
    const ph=F[i][1];
    if(autopause&&(ph==='TD'||ph==='ABORT')&&prevPh!=='TD'&&prevPh!=='ABORT'){playing=false;syncPlay();acc=0;prevPh=ph;break;}   // stop ON the transition
    prevPh=ph;
    trail.push([F[i][2],F[i][15],F[i][3],PHc[ph]||'#1f6feb']);if(trail.length>400)trail.shift();
    trailE.push([F[i][6],F[i][22],F[i][7],'#8b949e']);if(trailE.length>400)trailE.shift();
    acc-=FRAME_MS;}}
  draw();requestAnimationFrame(loop);}
// ================= camera controls =================
function hlMode(){['free','follow','chase','onboard'].forEach(m=>{const b=document.getElementById('m_'+m);if(b)b.classList.toggle('on',camMode===m);});}
function setMode(m){const fz=Math.min(30,Math.max(2,(H*0.33)/(WSC*0.675)));   // zoom that frames the true-scale (~0.68 m) rocket to ~1/3 canvas
  if(m==='free'){CENt=CEN0.slice();zoomT=1.0;}if(m==='follow')zoomT=fz;if(m==='chase')zoomT=fz*0.6;if(m==='onboard')zoomT=1.0;camMode=m;hlMode();}
function preset(a,e,z){camMode='free';hlMode();azT=a;elT=e;if(z)zoomT=z;}
function resetCam(){camMode='free';hlMode();CENt=CEN0.slice();azT=-1.05;elT=0.30;zoomT=1.0;}
function frameAll(){const F=cur().f;let a=[1e9,1e9,1e9],b=[-1e9,-1e9,-1e9];
  for(const fr of F){const p=[fr[2],fr[15],fr[3]];for(let j=0;j<3;j++){a[j]=Math.min(a[j],p[j]);b[j]=Math.max(b[j],p[j]);}}
  camMode='free';hlMode();CENt=[(a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2];CEN=CENt.slice();
  const ext=Math.max(b[0]-a[0],b[1]-a[1],b[2]-a[2],4);zoomT=clamp((zTop*0.9)/ext,0.3,3);}
// ================= input wiring =================
document.getElementById('ti').oninput=function(){ti=+this.value;refresh();};
document.getElementById('wi').oninput=function(){wi=+this.value;refresh();};
document.getElementById('spd').oninput=function(){speed=this.value/100;setT('spdv',speed.toFixed(1)+'x');};
document.getElementById('play').onclick=function(){playing=!playing;if(playing&&!loopMode&&i>=cur().f.length-1){i=0;rebuildTrail();}syncPlay();};
document.getElementById('restart').onclick=()=>{i=0;rebuildTrail();};
document.getElementById('m_free').onclick=()=>setMode('free');
document.getElementById('m_follow').onclick=()=>setMode('follow');
document.getElementById('m_chase').onclick=()=>setMode('chase');
document.getElementById('m_onboard').onclick=()=>setMode('onboard');
document.getElementById('p_top').onclick=()=>preset(-1.05,1.45);
document.getElementById('p_side').onclick=()=>preset(-1.5708,0.05);
document.getElementById('p_iso').onclick=()=>preset(-1.05,0.32,1.0);
document.getElementById('p_frame').onclick=frameAll;
document.getElementById('p_reset').onclick=resetCam;
const rollBtn=document.getElementById('t_roll');rollBtn.classList.toggle('on',rollStab);rollBtn.onclick=()=>{rollStab=!rollStab;rollBtn.classList.toggle('on',rollStab);};
const retroBtn=document.getElementById('t_retro');retroBtn.onclick=()=>{retroLook=!retroLook;retroBtn.classList.toggle('on',retroLook);};
const loopBtn=document.getElementById('t_loop');loopBtn.classList.toggle('on',loopMode);loopBtn.onclick=()=>{loopMode=!loopMode;loopBtn.classList.toggle('on',loopMode);};
const autoBtn=document.getElementById('t_auto');autoBtn.onclick=()=>{autopause=!autopause;autoBtn.classList.toggle('on',autopause);};
document.querySelectorAll('.chip[data-l]').forEach(b=>{const k=b.dataset.l;if(LYR[k])b.classList.add('on');b.onclick=()=>{LYR[k]=LYR[k]?0:1;b.classList.toggle('on',!!LYR[k]);};});
document.getElementById('fs').onclick=()=>{const v=document.getElementById('view');if(!document.fullscreenElement){if(v.requestFullscreen)v.requestFullscreen();}else document.exitFullscreen();};
document.addEventListener('fullscreenchange',()=>setTimeout(resize,60));
document.getElementById('collapse').onclick=()=>{document.getElementById('wrap').classList.toggle('collapsed');setTimeout(resize,60);};
function toggleHelp(){document.getElementById('help').classList.toggle('show');}
document.getElementById('helpb').onclick=toggleHelp;
// ---- orbit / pan / zoom on the main canvas ----
let drag=false,mx=0,my=0,panning=false;
cv.addEventListener('mousedown',e=>{drag=true;panning=(e.button===2||e.shiftKey);mx=e.clientX;my=e.clientY;});
window.addEventListener('mouseup',()=>{drag=false;panning=false;});
window.addEventListener('mousemove',e=>{if(!drag)return;const dx=e.clientX-mx,dy=e.clientY-my;mx=e.clientX;my=e.clientY;
  if(camMode==='free'&&panning){const a=dx/(WSC*zoom),b=dy/(WSC*zoom);for(let j=0;j<3;j++)CENt[j]-=a*Rv[j]-b*Uv[j];return;}   // screen-down = -Uv: grab-pan both axes
  if(camMode==='free'||camMode==='follow'){az-=dx*0.008;el+=dy*0.008;el=clamp(el,-0.25,1.55);azT=az;elT=el;}});
cv.addEventListener('contextmenu',e=>e.preventDefault());
cv.addEventListener('wheel',e=>{e.preventDefault();const k=e.deltaY<0?1.1:0.9;
  if(camMode==='free'){const sx=e.offsetX-W/2,sy=e.offsetY-H/2,a=sx/(WSC*zoom),b=-sy/(WSC*zoom);
    zoom=clamp(zoom*k,0.2,30);zoomT=zoom;const a2=sx/(WSC*zoom),b2=-sy/(WSC*zoom);
    for(let j=0;j<3;j++)CENt[j]+=(a-a2)*Rv[j]+(b-b2)*Uv[j];CEN=CENt.slice();}
  else zoomT=clamp(zoomT*k,0.2,30);},{passive:false});
// ---- keyboard ----
window.addEventListener('keydown',e=>{const ae=document.activeElement;if(ae&&/INPUT|SELECT|TEXTAREA/.test(ae.tagName))return;
  const F=cur().f;let handled=true;
  switch(e.key){
    case ' ':playing=!playing;if(playing&&!loopMode&&i>=F.length-1){i=0;rebuildTrail();}syncPlay();break;
    case 'ArrowLeft':playing=false;syncPlay();i=Math.max(0,i-(e.shiftKey?10:1));rebuildTrail();break;
    case 'ArrowRight':playing=false;syncPlay();i=Math.min(F.length-1,i+(e.shiftKey?10:1));rebuildTrail();break;
    case 'Home':playing=false;syncPlay();i=0;rebuildTrail();break;
    case 'End':playing=false;syncPlay();i=F.length-1;rebuildTrail();break;
    case 'r':case 'R':i=0;rebuildTrail();break;
    case '[':speed=Math.max(0.2,speed-0.2);document.getElementById('spd').value=speed*100;setT('spdv',speed.toFixed(1)+'x');break;
    case ']':speed=Math.min(4,speed+0.2);document.getElementById('spd').value=speed*100;setT('spdv',speed.toFixed(1)+'x');break;
    case '+':case '=':zoomT=clamp(zoomT*1.1,0.2,30);break;
    case '-':case '_':zoomT=clamp(zoomT*0.9,0.2,30);break;
    case '1':setMode('free');break;
    case '2':setMode('follow');break;
    case '3':setMode('chase');break;
    case '4':setMode('onboard');break;
    case '5':resetCam();break;
    case 'f':case 'F':setMode(camMode==='follow'?'free':'follow');break;
    case 'g':case 'G':LYR.grid^=1;syncChip('grid');break;
    case 't':case 'T':LYR.trail^=1;syncChip('trail');break;
    case 'e':case 'E':LYR.ekf^=1;syncChip('ekf');break;
    case 'h':case 'H':LYR.hud^=1;syncChip('hud');break;
    case 'm':case 'M':LYR.map^=1;syncChip('map');break;
    case '?':toggleHelp();break;
    case 'Escape':document.getElementById('help').classList.remove('show');break;
    default:handled=false;}
  if(handled)e.preventDefault();});
function syncChip(k){const b=document.querySelector('.chip[data-l="'+k+'"]');if(b)b.classList.toggle('on',!!LYR[k]);}
// ---- responsive canvas + hi-DPI ----
function resize(){DPR=Math.min(window.devicePixelRatio||1,2);
  const cw=Math.max(200,cv.clientWidth),ch=Math.max(200,cv.clientHeight);
  cv.width=Math.round(cw*DPR);cv.height=Math.round(ch*DPR);cx.setTransform(DPR,0,0,DPR,0,0);
  W=cw;H=ch;WSC=Math.max(1,(H-150)/zTop);
  const tw=Math.max(200,tl.clientWidth);tl.width=Math.round(tw*DPR);tl.height=Math.round(34*DPR);tlx.setTransform(DPR,0,0,DPR,0,0);TLW=tw;TLH=34;}
new ResizeObserver(resize).observe(document.getElementById('view'));
window.addEventListener('resize',resize);
// ---- go ----
resize();hlMode();syncPlay();refresh();requestAnimationFrame(loop);
</script></body></html>"""
w0 = WINDS.index(0) if 0 in WINDS else 0
HTML = (HTML.replace("__DATA__", data).replace("__TARGETS__", json.dumps(TARGETS)).replace("__WINDS__", json.dumps(WINDS))
            .replace("__APO__", f"{apogee:.1f}").replace("__XMIN__", f"{xmin:.2f}").replace("__XMAX__", f"{xmax:.2f}")
            .replace("__YMAX__", f"{ymax:.2f}")
            .replace("__TN__", str(len(TARGETS)-1)).replace("__WN__", str(len(WINDS)-1)).replace("__W0__", str(w0)))
with open(OUT, "w", encoding="utf-8") as f:
    f.write(HTML)
print(f"wrote {OUT}")
