"""
tools/vsr_animate.py  (2026-06-27)

Generates a self-contained HTML animation (outputs/vsr_animation.html) showing three rockets
flying the SAME pitch-over maneuver side by side, so you can VISUALLY confirm each control
mechanism is functioning:

  TVC      : the nozzle/flame at the tail gimbals
  CANARD   : the forward fins deflect (rotate)
  MARGIN   : the forward fins EXTEND and RETRACT (length changes); deflection stays 0

Each panel shows the rocket body (rotating to its pitch attitude), fixed aft fins, the moving
control surface, the velocity vector (so you can see angle of attack = body minus velocity),
the commanded target attitude (dashed), and a live readout of the control variable. Open the
HTML in any browser; play/pause/scrub/speed controls included. No dependencies.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from vsr_lab import run

DECIM = 3                      # keep every 3rd sim step (~0.015 s/frame)
MODES = ["tvc", "canard", "margin"]
TITLE = {"tvc": "THRUST VECTORING", "canard": "CANARD DEFLECTION", "margin": "MARGIN MODULATION"}
SUB = {"tvc": "nozzle gimbals", "canard": "forward fins deflect", "margin": "forward fins extend / retract"}


def pack(mode):
    from sim_vsr.scenario import thrust_F_class
    tr = run(mode, man_deg=30.0, ramp=0.10, stall_deg=None, seed=0, t_end=2.4)
    idx = range(0, len(tr.t), DECIM)
    frames = []
    for k in idx:
        frames.append(dict(
            t=round(float(tr.t[k]), 3),
            th=round(float(np.rad2deg(tr.theta[k])), 2),     # body pitch from horizontal
            ga=round(float(np.rad2deg(tr.gamma[k])), 2),     # flight-path angle
            ref=round(float(np.rad2deg(tr.ref[k])), 2),      # commanded attitude
            gim=round(float(tr.gimbal_deg[k]), 2),
            dep=round(float(tr.deploy[k]), 3),
            cd=round(float(tr.canard_deg[k]), 2),
            mg=round(float(tr.margin[k]), 3),
            thr=1 if thrust_F_class(tr.t[k]) > 0.5 else 0,
        ))
    m = tr.metrics
    return dict(mode=mode, title=TITLE[mode], sub=SUB[mode], frames=frames,
                settle=m.get("settle"), success=bool(m.get("success")),
                overshoot=m.get("overshoot"), peak_aoa=m.get("peak_aoa"))


def main():
    data = [pack(m) for m in MODES]
    nframes = min(len(d["frames"]) for d in data)
    for d in data:
        d["frames"] = d["frames"][:nframes]
    payload = json.dumps(dict(panels=data, n=nframes), separators=(",", ":"))

    html = HTML_TEMPLATE.replace("/*DATA*/", payload)
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "vsr_animation.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
    print("wrote", path, f"({nframes} frames x {len(data)} panels)")
    for d in data:
        print(f"  {d['mode']:8} success={d['success']} settle={d['settle']} overshoot={d['overshoot']} peakAoA={d['peak_aoa']}")


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Variable-Stability Rocket - Control Comparison</title>
<style>
 body{background:#0d1117;color:#e6edf3;font-family:Segoe UI,Arial,sans-serif;margin:0;padding:16px}
 h1{font-size:19px;margin:0 0 2px} .note{color:#8b949e;font-size:13px;margin-bottom:10px}
 #row{display:flex;gap:14px;flex-wrap:wrap}
 .panel{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:8px}
 .ptitle{font-size:14px;font-weight:600;margin:2px 4px} .psub{font-size:11px;color:#8b949e;margin:0 4px 4px}
 canvas{background:linear-gradient(#0b1d3a,#0d1117);border-radius:6px;display:block}
 .read{font-size:11px;color:#9ad;margin:4px 4px 0;font-family:Consolas,monospace;white-space:pre}
 #ctl{margin-top:14px;display:flex;align-items:center;gap:12px;flex-wrap:wrap}
 button{background:#238636;color:#fff;border:0;border-radius:6px;padding:7px 16px;font-size:14px;cursor:pointer}
 button.sec{background:#30363d} input[type=range]{width:420px} label{font-size:13px;color:#8b949e}
 .lg{font-size:12px;color:#8b949e;margin-top:10px;max-width:1100px;line-height:1.5}
 b.g{color:#7ee787} b.o{color:#ffa657} b.b{color:#79c0ff}
</style></head><body>
<h1>Variable-Stability Rocket &mdash; control-mechanism comparison</h1>
<div class="note">Same airframe, same maneuver (30&deg; pitch-over), same control law. Watch the control surface on each.
Body = <b class="b">blue</b>, velocity vector = <b class="o">orange</b> (gap to body = angle of attack), commanded attitude = dashed grey.</div>
<div id="row"></div>
<div id="ctl">
  <button id="play">&#10073;&#10073; Pause</button>
  <button id="restart" class="sec">&#8635; Restart</button>
  <label>time <input type="range" id="scrub" min="0" max="100" value="0"></label>
  <label>speed <input type="range" id="speed" min="25" max="200" value="100" style="width:120px"> <span id="spd">1.0x</span></label>
  <span id="clock" style="font-family:Consolas;color:#9ad"></span>
</div>
<div class="lg" id="legend"></div>
<script>
const D = /*DATA*/;
const W=360,H=520, CX=180, CY=300;       // panel canvas + rocket pivot (CG)
const PXM=11;                            // px per metre for velocity-vector scale
// build panels
const row=document.getElementById('row'); const panels=[];
D.panels.forEach(p=>{
  const d=document.createElement('div'); d.className='panel';
  d.innerHTML=`<div class="ptitle">${p.title}</div><div class="psub">${p.sub}</div>`;
  const c=document.createElement('canvas'); c.width=W;c.height=H; d.appendChild(c);
  const r=document.createElement('div'); r.className='read'; d.appendChild(r);
  row.appendChild(d); panels.push({p,ctx:c.getContext('2d'),read:r});
});

function rot(ax,ay,deg){const r=deg*Math.PI/180;return [ax*Math.cos(r)-ay*Math.sin(r), ax*Math.sin(r)+ay*Math.cos(r)];}
// body frame: x_b along nose direction, y_b to the rocket's "left". theta_h measured from horizontal,
// screen: up is -y. nose direction on screen = (cos th, -sin th).
function bodyPt(th, along, perp){
  const dx=Math.cos(th*Math.PI/180), dy=-Math.sin(th*Math.PI/180);   // nose dir (screen)
  const px=-dy, py=dx;                                               // left-perp (screen)
  return [CX+along*dx+perp*px, CY+along*dy+perp*py];
}
function line(ctx,a,b,w,col){ctx.strokeStyle=col;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();}

function drawRocket(ctx, mode, fr){
  ctx.clearRect(0,0,W,H);
  // ground ref + commanded attitude (dashed) through CG
  const refdx=Math.cos(fr.ref*Math.PI/180), refdy=-Math.sin(fr.ref*Math.PI/180);
  ctx.strokeStyle='#586069';ctx.setLineDash([6,6]);ctx.lineWidth=1.5;ctx.beginPath();
  ctx.moveTo(CX-refdx*120,CY-refdy*120);ctx.lineTo(CX+refdx*150,CY+refdy*150);ctx.stroke();ctx.setLineDash([]);
  // velocity vector (orange) from CG -- shows flight-path direction (gamma)
  const gdx=Math.cos(fr.ga*Math.PI/180), gdy=-Math.sin(fr.ga*Math.PI/180);
  line(ctx,[CX,CY],[CX+gdx*95,CY+gdy*95],3,'#ffa657');
  // arrowhead
  const ah1=rot(gdx,gdy,150),ah2=rot(gdx,gdy,-150);
  line(ctx,[CX+gdx*95,CY+gdy*95],[CX+gdx*95+ah1[0]*12,CY+gdy*95+ah1[1]*12],3,'#ffa657');
  line(ctx,[CX+gdx*95,CY+gdy*95],[CX+gdx*95+ah2[0]*12,CY+gdy*95+ah2[1]*12],3,'#ffa657');

  const th=fr.th;
  const nose=bodyPt(th,82,0), tail=bodyPt(th,-70,0);
  // body
  line(ctx,tail,nose,9,'#79c0ff');
  // nose cone tip
  ctx.fillStyle='#a5d6ff';ctx.beginPath();const n2=bodyPt(th,96,0);
  const nl=bodyPt(th,78,6),nr=bodyPt(th,78,-6);ctx.moveTo(n2[0],n2[1]);ctx.lineTo(nl[0],nl[1]);ctx.lineTo(nr[0],nr[1]);ctx.closePath();ctx.fill();
  // fixed aft fins (two), swept back
  [1,-1].forEach(sgn=>{ const root=bodyPt(th,-58,sgn*5), tip=bodyPt(th,-78,sgn*26); line(ctx,root,tip,5,'#6e7681'); line(ctx,bodyPt(th,-70,sgn*5),tip,5,'#6e7681');});

  // --- control surface per mode ---
  if(mode==='tvc'){
     // gimbaled nozzle: thrust dir = aft (-nose) rotated by gimbal
     const baseAlong=-70;
     const gdir=th+180+fr.gim;                 // aft direction + gimbal
     const ndx=Math.cos(gdir*Math.PI/180), ndy=-Math.sin(gdir*Math.PI/180);
     const noz0=bodyPt(th,-70,0), noz1=[noz0[0]+ndx*16,noz0[1]+ndy*16];
     line(ctx,noz0,noz1,7,'#d2a8ff');
     if(fr.thr){ // flame
       ctx.fillStyle='rgba(255,150,60,0.85)';ctx.beginPath();
       const f1=[noz1[0]+ndx*6+(-ndy)*6,noz1[1]+ndy*6+(ndx)*6];
       const f2=[noz1[0]+ndx*6-(-ndy)*6,noz1[1]+ndy*6-(ndx)*6];
       const ftip=[noz1[0]+ndx*40,noz1[1]+ndy*40];
       ctx.moveTo(f1[0],f1[1]);ctx.lineTo(ftip[0],ftip[1]);ctx.lineTo(f2[0],f2[1]);ctx.closePath();ctx.fill();
     }
  } else {
     // forward canards near nose. canard mode: rotate by deflection. margin mode: length=deploy.
     const at=58;
     let len = (mode==='margin') ? (4+30*fr.dep) : 26;   // extend/retract vs fixed
     let defl = (mode==='canard') ? fr.cd : 0;
     [1,-1].forEach(sgn=>{
       const root=bodyPt(th,at,sgn*5);
       // canard extends sideways (perp) then we rotate the surface line by deflection about root
       const baseTip=bodyPt(th,at,sgn*(5+len));
       // apply deflection: rotate tip about root in screen plane by sgn*defl
       const vx=baseTip[0]-root[0], vy=baseTip[1]-root[1];
       const rr=rot(vx,vy, defl);   // deflection rotates the fin
       const tip=[root[0]+rr[0],root[1]+rr[1]];
       const col = (mode==='margin')? (fr.mg<-0.02?'#ff7b72':(fr.mg>0.02?'#7ee787':'#e3b341')) : '#d2a8ff';
       line(ctx,root,tip,6,col);
     });
  }
  // CG dot
  ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(CX,CY,3,0,7);ctx.fill();
}

function readout(mode,fr){
  const aoa=(fr.th-fr.ga).toFixed(1);
  let cv;
  if(mode==='tvc') cv=`gimbal   ${fr.gim>=0?' ':''}${fr.gim.toFixed(1)}°`;
  else if(mode==='canard') cv=`canard   ${fr.cd>=0?' ':''}${fr.cd.toFixed(1)}°`;
  else cv=`deploy   ${(fr.dep*100).toFixed(0)}%   margin ${fr.mg>=0?'+':''}${fr.mg.toFixed(2)} cal`;
  return `t=${fr.t.toFixed(2)}s   pitch=${fr.th.toFixed(0)}°  AoA=${aoa}°\n${cv}`;
}

// animation engine
let frame=0, playing=true, speed=1.0;
const N=D.n, scrub=document.getElementById('scrub'), clock=document.getElementById('clock');
scrub.max=N-1;
function render(){
  panels.forEach(pn=>{ const fr=pn.p.frames[frame]; drawRocket(pn.ctx,pn.p.mode,fr); pn.read.textContent=readout(pn.p.mode,fr);});
  scrub.value=frame; clock.textContent=`frame ${frame}/${N-1}`;
}
let acc=0,last=performance.now();
function loop(now){ const dt=now-last; last=now;
  if(playing){ acc+=dt*speed; while(acc>15){ frame=(frame+1)%N; acc-=15; } render(); }
  requestAnimationFrame(loop);
}
document.getElementById('play').onclick=function(){playing=!playing;this.innerHTML=playing?'&#10073;&#10073; Pause':'&#9658; Play';};
document.getElementById('restart').onclick=()=>{frame=0;render();};
scrub.oninput=function(){frame=+this.value;playing=false;document.getElementById('play').innerHTML='&#9658; Play';render();};
const spd=document.getElementById('spd');
document.getElementById('speed').oninput=function(){speed=this.value/100;spd.textContent=speed.toFixed(1)+'x';};
// legend with results
let lg='<b>What you are watching:</b> each rocket holds vertical, then is commanded 30&deg; over. ';
lg+='<b class="b">Blue</b>=body, <b class="o">orange</b>=velocity (body-to-orange gap = angle of attack), dashed=target.<br>';
D.panels.forEach(p=>{ lg+=`<b>${p.title}</b>: ${p.success?'<b class="g">reached &amp; held</b>':'<b class="o">did not settle</b>'} `
  +`settle ${p.settle!=null?p.settle.toFixed(2)+'s':'-'}, overshoot ${p.overshoot!=null?p.overshoot.toFixed(1)+'°':'-'}, peak AoA ${p.peak_aoa}°. &nbsp; `;});
lg+='<br><i>MARGIN panel: fin color = green stable / yellow neutral / red unstable; watch the fins extend (destabilize, &ldquo;fling&rdquo;) then retract (restabilize, &ldquo;catch&rdquo;).</i>';
document.getElementById('legend').innerHTML=lg;
render(); requestAnimationFrame(loop);
</script></body></html>"""


if __name__ == "__main__":
    main()
