#!/usr/bin/env python3
"""Turn a SIL flight CSV into a self-contained animated HTML (like the landing sims).

Usage:  python make_flight_html.py [flight.csv] [flight.html]
Shows the rocket flying the profile: true trajectory + attitude, thrust flame,
the firmware's EKF estimate (ghost), phase, and a live HUD. Open the HTML in a browser.
"""
import csv, json, sys

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "flight.csv"
    out = sys.argv[2] if len(sys.argv) > 2 else "flight.html"
    F = []
    with open(src, newline="") as f:
        for r in csv.DictReader(f):
            g = lambda k: float(r[k])
            F.append([g("t"), r["state"], g("px"), g("pz"), g("th_deg"), g("thrust"),
                      g("estX"), g("estZ"), g("thEst_deg"), g("uTVC"), g("deploy"),
                      g("keff"), g("vz"), int(float(r["legs"])), int(float(r["chute"])),
                      float(r.get("py",0) or 0), float(r.get("roll_deg",0) or 0),
                      float(r.get("rollrate_dps",0) or 0), float(r.get("rollDef",0) or 0)])
    if not F:
        print("empty CSV"); return
    apogee = max(fr[3] for fr in F)
    xmin = min(min(fr[2] for fr in F), min(fr[6] for fr in F), 0.0)
    xmax = max(max(fr[2] for fr in F), max(fr[6] for fr in F), 0.0)
    data = json.dumps(F, separators=(",", ":"))

    html = """<!doctype html><html><head><meta charset="utf-8"><title>SIL flight replay</title>
<style>
 body{margin:0;background:#0d1117;color:#c9d1d9;font-family:Segoe UI,system-ui,sans-serif}
 #wrap{display:flex;gap:14px;padding:14px}
 canvas{background:linear-gradient(#0b1a2b,#0d1117);border:1px solid #30363d;border-radius:8px}
 #side{width:290px;font-size:13px}
 h2{margin:.2em 0;font-size:16px;color:#58a6ff}
 .row{margin:3px 0} b{color:#8b949e}
 button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;padding:5px 10px;cursor:pointer}
 input[type=range]{width:100%}
 .tag{display:inline-block;padding:1px 7px;border-radius:10px;background:#1f6feb;color:#fff;font-weight:600}
</style></head><body>
<div id="wrap">
 <canvas id="c" width="820" height="620"></canvas>
 <div id="side">
  <h2>SIL flight replay</h2>
  <div class="row">Runs the EXACT firmware in the software-in-the-loop sim.</div>
  <div class="row"><button id="play">&#10073;&#10073; Pause</button> <button id="restart">&#9658; Restart</button></div>
  <div class="row">speed <input type="range" id="spd" min="10" max="400" value="100"> <span id="spdv">1.0x</span></div>
  <div class="row">time <input type="range" id="scrub" min="0" max="100" value="0"></div>
  <hr style="border-color:#30363d">
  <div class="row">t = <b id="t"></b> s &middot; phase <span class="tag" id="ph"></span></div>
  <div class="row">altitude <b id="alt"></b> m (apogee %APO% m)</div>
  <div class="row">vert vel <b id="vz"></b> m/s</div>
  <div class="row">downrange x <b id="x"></b> m (target 0)</div>
  <div class="row">tilt (true) <b id="th"></b>&deg; &middot; (EKF) <b id="the"></b>&deg;</div>
  <div class="row">ROLL <b id="roll"></b>&deg; @ <b id="rollr"></b>&deg;/s &middot; crossrange y <b id="py"></b> m</div>
  <div class="row">EKF pos error <b id="err"></b> m</div>
  <div class="row">TVC cmd <b id="u"></b> &middot; margin <b id="dep"></b> &middot; roll-fin <b id="rdf"></b></div>
  <div class="row">keff (RLS) <b id="keff"></b> &middot; thrust <b id="thr"></b> N</div>
  <hr style="border-color:#30363d">
  <div class="row" style="color:#8b949e">solid = true rocket; dashed ghost = firmware's EKF belief (sensor lags simulated + lead-compensated, so it should hug the rocket). Blue margin fins fold out with deploy (folded=stable, out=destabilizing; neutral fold migrates with the CG). Legs/chute flash on deploy.</div>
 </div>
</div>
<script>
const F=%DATA%, XMIN=%XMIN%, XMAX=%XMAX%, APO=%APO%;
const cv=document.getElementById('c'),cx=cv.getContext('2d'),W=cv.width,H=cv.height;
const PAD=40, gy=H-46;
const zTop=Math.max(APO*1.1,5), xspan=Math.max(XMAX-XMIN,8)*1.15, xmid=(XMIN+XMAX)/2;
const sy=(gy-30)/zTop, sx=(W-2*PAD)/xspan;
const S=Math.min(sy,sx*1);                   // world m -> px (shared vertical scale)
function X(x){return W/2+(x-xmid)*sx;}
function Y(z){return gy - z*S;}
const PHc={BOOST:'#f0883e',COAST:'#58a6ff',BURN:'#f85149',TD:'#3fb950',ABORT:'#f85149',PAD:'#8b949e',ARM:'#8b949e',CD:'#d29922'};
let i=0, playing=true, speed=1, last=performance.now(), acc=0;
const FPS=1000/50;
function rocket(x,z,thDeg,thrust,ghost,dep){
  const px=X(x),py=Y(z),a=thDeg*Math.PI/180;   // tilt from vertical, +toward +x
  cx.save();cx.translate(px,py);cx.rotate(a);
  const L=26,Wd=7;
  if(ghost){cx.setLineDash([4,3]);cx.strokeStyle='#8b949e';cx.lineWidth=1.5;cx.strokeRect(-Wd/2,-L,Wd,L);cx.setLineDash([]);cx.restore();return;}
  if(thrust>0.5){cx.fillStyle='#ffd23f';const fl=6+thrust*0.7;cx.beginPath();cx.moveTo(-Wd/2+1,0);cx.lineTo(Wd/2-1,0);cx.lineTo(0,fl);cx.closePath();cx.fill();}
  // fold-out margin fins (forward), drawn BEFORE the body so folded fins hide behind it:
  // deploy=0 -> folded flat down along the body (full stable); deploy=1 -> 90deg out (full unstable)
  const dp=Math.max(0,Math.min(1,dep==null?0.5:dep)), fna=dp*Math.PI/2, fnl=10;
  cx.strokeStyle='#58a6ff';cx.lineWidth=2.6;
  for(const s of [-1,1]){const hx=s*(Wd/2-1.5);
    cx.beginPath();cx.moveTo(hx,-L+4);cx.lineTo(hx+s*fnl*Math.sin(fna),-L+4+fnl*Math.cos(fna));cx.stroke();}
  cx.lineWidth=1;
  cx.fillStyle='#e6edf3';cx.fillRect(-Wd/2,-L,Wd,L);            // body (nose up = -y)
  cx.fillStyle='#f85149';cx.beginPath();cx.moveTo(-Wd/2,-L);cx.lineTo(Wd/2,-L);cx.lineTo(0,-L-8);cx.closePath();cx.fill(); // nose
  cx.restore();
}
let trail=[];
function draw(){
  const f=F[i];
  cx.clearRect(0,0,W,H);
  // ground + target
  cx.strokeStyle='#30363d';cx.beginPath();cx.moveTo(0,gy);cx.lineTo(W,gy);cx.stroke();
  cx.fillStyle='#3fb950';cx.fillRect(X(0)-14,gy,28,3);
  cx.fillStyle='#8b949e';cx.font='11px monospace';cx.fillText('pad',X(0)-8,gy+16);
  // altitude gridlines
  cx.strokeStyle='#161b22';cx.fillStyle='#484f58';
  for(let z=0;z<=zTop;z+= (zTop>120?50:(zTop>40?20:10))){cx.beginPath();cx.moveTo(0,Y(z));cx.lineTo(W,Y(z));cx.stroke();cx.fillText(z+'m',4,Y(z)-2);}
  // trail
  cx.strokeStyle='#1f6feb';cx.lineWidth=1.5;cx.beginPath();
  for(let k=0;k<trail.length;k++){const p=trail[k];k?cx.lineTo(X(p[0]),Y(p[1])):cx.moveTo(X(p[0]),Y(p[1]));}cx.stroke();
  rocket(f[6],f[7],f[8],0,true);     // EKF ghost
  rocket(f[2],f[3],f[4],f[5],false,f[10]); // true rocket
  // deploy flashes
  if(f[13]){cx.fillStyle='#3fb950';cx.font='13px sans-serif';cx.fillText('LEGS',X(f[2])+16,Y(f[3]));}
  if(f[14]){cx.fillStyle='#d29922';cx.fillText('CHUTE',X(f[2])+16,Y(f[3])-14);}
  // HUD
  const err=Math.hypot(f[6]-f[2],f[7]-f[3]);
  document.getElementById('t').textContent=f[0].toFixed(2);
  const ph=document.getElementById('ph');ph.textContent=f[1];ph.style.background=PHc[f[1]]||'#1f6feb';
  document.getElementById('alt').textContent=f[3].toFixed(1);
  document.getElementById('vz').textContent=f[12].toFixed(1);
  document.getElementById('x').textContent=f[2].toFixed(2);
  document.getElementById('th').textContent=f[4].toFixed(1);
  document.getElementById('the').textContent=f[8].toFixed(1);
  document.getElementById('err').textContent=err.toFixed(2);
  document.getElementById('roll').textContent=f[16].toFixed(1);
  document.getElementById('rollr').textContent=f[17].toFixed(0);
  document.getElementById('py').textContent=f[15].toFixed(2);
  document.getElementById('rdf').textContent=f[18].toFixed(2);
  document.getElementById('u').textContent=f[9].toFixed(1);
  document.getElementById('dep').textContent=f[10].toFixed(2);
  document.getElementById('keff').textContent=f[11].toFixed(1);
  document.getElementById('thr').textContent=f[5].toFixed(0);
  document.getElementById('scrub').value=(i/(F.length-1)*100).toFixed(1);
}
function loop(now){
  const dt=now-last;last=now;
  if(playing){acc+=dt*speed;while(acc>FPS){i=(i+1)%F.length;if(i===0)trail=[];trail.push([F[i][2],F[i][3]]);if(trail.length>400)trail.shift();acc-=FPS;}}
  draw();requestAnimationFrame(loop);
}
document.getElementById('play').onclick=function(){playing=!playing;this.innerHTML=playing?'&#10073;&#10073; Pause':'&#9658; Play';};
document.getElementById('restart').onclick=()=>{i=0;trail=[];};
document.getElementById('spd').oninput=function(){speed=this.value/100;document.getElementById('spdv').textContent=speed.toFixed(1)+'x';};
document.getElementById('scrub').oninput=function(){i=Math.round(this.value/100*(F.length-1));trail=[];};
requestAnimationFrame(loop);
</script></body></html>"""
    html = (html.replace("%DATA%", data).replace("%XMIN%", f"{xmin:.3f}")
                .replace("%XMAX%", f"{xmax:.3f}").replace("%APO%", f"{apogee:.1f}"))
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"wrote {out}  ({len(F)} frames, apogee {apogee:.1f} m)")

if __name__ == "__main__":
    main()
