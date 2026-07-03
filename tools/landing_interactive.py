"""
tools/landing_interactive.py  (2026-06-29)

Writes outputs/landing_interactive.html -- an INTERACTIVE landing webapp. The full 3-DOF physics + the LIVE
guidance (gravity-turn ascent, margin-mod flip to engine-down, live suicide-burn ignition, retrograde landing
burn) are ported to JavaScript, so the browser RE-COMPUTES the whole trajectory whenever you move a slider.
Nothing is pre-baked -- you are watching/stress-testing the live calculations.

Controls:
  TARGET  -- downrange landing point. The rocket auto-aims (a quick in-browser bisection picks the gravity-turn
             kick that lands on the target), then flies there.
  THRUST  -- T/W ratio (flat constant thrust). A vertical hoverslam works at ANY T/W>1 (you ignite partway down
             at h_ign = apogee/(T/W), not after a full free-fall). What T/W buys is the DOWNRANGE soft-landing
             envelope: more thrust margin = more ability to ALSO kill horizontal velocity while still arresting
             the fall. ~1.7 lands soft to ~60 m; ~2.0 soft everywhere; below ~1.5 the far targets land hard.

Live ignition math (re-evaluated every step from the current state, so the rocket can course-adapt):
  vertical suicide burn: ignite when altitude <= vz^2 / (2*(T/m - g)).
  Coast lineup flies PROGRADE during ascent (body along velocity, alpha~0 -> clean parabolic coast with no aero
  deceleration), then flips to retrograde (engine-down) only after apogee where dynamic pressure is low. Holding
  engine-down during the high-speed ascent (the old behavior) put the body broadside to a 30 m/s airflow and bled
  off the horizontal velocity -- the trajectory looked non-ballistic. vz floors avoid the apogee velocity swing.

Also fixes ground-clipping: the rocket glyph rests its tail on the ground (it no longer sinks to the CG).
"""
import os

HTML = r"""<!doctype html><html><head><meta charset="utf-8"><title>Interactive TVC Landing -- live guidance</title>
<style>
 body{background:#0d1117;color:#c9d1d9;font:14px Segoe UI,system-ui,sans-serif;margin:0;padding:14px}
 h1{font-size:18px;margin:0 0 6px} .note{color:#8b949e;font-size:12px;margin:0 0 10px;max-width:940px}
 #wrap{display:flex;gap:14px;flex-wrap:wrap} canvas{background:#0a0e14;border:1px solid #30363d;border-radius:6px}
 #side{min-width:240px;font-size:13px;line-height:1.55}
 #ctl{margin:10px 0;display:flex;gap:22px;align-items:center;flex-wrap:wrap}
 button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;padding:6px 12px;cursor:pointer}
 button:hover{background:#30363d} label{color:#8b949e}
 .bar{display:inline-block;width:120px;height:8px;background:#161b22;border-radius:4px;position:relative;vertical-align:middle;margin-left:6px}
 .fill{position:absolute;top:0;height:8px;background:#1f6feb;border-radius:4px} b{color:#c9d1d9} .big{font-size:15px}
</style></head><body>
<h1>Interactive TVC Landing &mdash; live guidance (computed in your browser)</h1>
<div class="note">Drag the sliders &mdash; every change <b>re-runs the full sim live</b> (no pre-baked trajectory).
<b>TARGET</b>: the rocket auto-aims (a search picks the gravity-turn kick) and flies there with state-based guidance:
gravity-turn ascent &rarr; margin-mod canards flip it engine-down &rarr; <b>live suicide-burn ignition</b>
(<i>ignite when altitude &le; vz&sup2;/(2&middot;(T/m &minus; g))</i>) &rarr; retrograde burn.
<b>THRUST</b> (T/W): a vertical hoverslam works at <b>any T/W&gt;1</b> (ignite partway down at apogee/(T/W)). More T/W just widens the DOWNRANGE soft-landing envelope &mdash; ~1.7 soft to ~60&nbsp;m, ~2.0 soft everywhere, &lt;1.5 far targets land hard. No launch rail, no drag (ideal).</div>
<div id="ctl">
 <label>TARGET downrange <input type="range" id="tgt" min="0" max="80" value="40" step="1" style="width:230px"> <span id="tgtv" class="big" style="color:#e3b341">40 m</span></label>
 <label>THRUST (T/W) <input type="range" id="tw" min="1.2" max="3.0" value="2.0" step="0.05" style="width:150px"> <span id="twv" class="big" style="color:#7ee787">2.00</span></label>
 <label>WIND (m/s) <input type="range" id="wind" min="-5" max="5" value="0" step="0.5" style="width:150px"> <span id="windv" class="big" style="color:#79c0ff">0.0</span></label>
 <button id="ctrl">CONTROLLER: ADRC</button> <button id="sens">SENSORS: EKF (IMU+baro)</button> <button id="gps">GPS: on (~0.6 m)</button> <button id="cal">CALIB: on</button> <button id="gust">&#127788; new gust</button>
 <button id="play">&#10073;&#10073; Pause</button> <button id="restart">&#8635; Restart</button>
 <label>speed <input type="range" id="speed" min="20" max="400" value="100" style="width:100px"> <span id="spd">1.0x</span></label>
</div>
<div id="wrap"><canvas id="cv" width="760" height="540"></canvas><div id="side"></div></div>
<script>
// ---------------- PHYSICS (ported from tools/landing_sim3dof.py) ----------------
const G=9.80665, RHO=1.20, DT=0.003;     // DT a touch coarser than the 1ms sim for snappy in-browser re-runs
const P={m:0.9, Iyy:0.045, L_nozzle:0.30, d_ref:0.076, CN_total:9.6, CD0:0.5,
         C_mq:12.0, sm_max:1.0, delta_max:5*Math.PI/180};
const CD_BROADSIDE=1.4;   // broadside (alpha=90) drag coefficient
const CLMAX=1.5;          // STALL: normal-force coeff caps here (real surfaces stall ~CL 1.2-1.6)
const VE=1000, DSLEW=1.5;   // effective exhaust velocity (mass-loss), TVC gimbal slew (rad/s)
// MOTOR + MASS REALISM: F-15-style thrust CURVE (spike to 1.85x, then plateau), scaled so the PLATEAU equals the
// T/W slider x launch weight. As propellant burns: mass falls (VE), and because the propellant sits aft, the CG
// migrates FORWARD -> the nozzle moment-arm L_nozzle GROWS and Iyy SHRINKS through the flight. Launch mass
// M_DRY+MPROP = 0.90 kg; launch Iyy~0.050, L_nozzle~0.30 (match the old constants), drifting to 0.043 / 0.358 empty.
const M_DRY=0.68, MPROP=0.22;        // dry structure + propellant (kg)
const L_DRY=0.358, L_PROP=0.12;      // CG-from-nozzle (m): empty-structure CG, and propellant CG (aft, near nozzle)
const IYY_DRY=0.043, RG2=0.0318;     // Iyy(mprop) = IYY_DRY + mprop*RG2  (prop's radius-of-gyration^2 about CG)
const TBURN=3.45;                    // motor burn time (s), F-15 class
function thrustShape(tau){ if(tau<0||tau>TBURN)return 0; if(tau<0.15)return 1.85*tau/0.15; if(tau<0.50)return 1.85-0.85*((tau-0.15)/0.35); return 1.0; }
function massProps(mr){ if(mr<0)mr=0; const m=M_DRY+mr; return [m, IYY_DRY+mr*RG2, (M_DRY*L_DRY+mr*L_PROP)/m]; }   // [mass, Iyy, L_nozzle]
// The 4 fins (33mm chord x 92mm span) are the margin-control surface, a TWO-SIDED actuator: deploy=0.5 -> neutral
// (CP=CG); <0.5 -> nose-first stable; >0.5 -> tail-first stable, EQUAL margin (+-sm_max cal) either side, so the
// canards both DRIVE and BRAKE the retrograde flip (neutral body flips cleanly). CONTROL IS ACCELERATION-COMMANDED:
// every controller outputs a desired angular accel and inverts it through the CURRENT thrust*L_nozzle/Iyy (TVC) or
// q_dyn (canards), so it exerts the SAME angular accel regardless of the thrust curve, mass, CG or Iyy migration.
P.S_ref=Math.PI*(P.d_ref/2)**2;
let CT=2.0*P.m*G;                          // flat constant thrust (T/W slider x launch weight)
let Z_TD=0;                                // touchdown CG altitude = rocket half-length (tail on ground). Set in recompute
// to the glyph's half-length in metres so the rocket starts AND lands tail-on-ground -- the burn nulls vz right at Z_TD,
// so the full soft slowdown happens IN VIEW and the (oversized) glyph never freezes/clips at the ground. See recompute().
const ASC=2.0, LAND=3.45;                  // ascent / landing burn (s); ASC short so apogee ~ downrange (visible arc)
const wrap=a=>((a+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI;
const clip=(v,a,b)=>v<a?a:(v>b?b:v);

function aero(vx,vz,th,q,deploy){   // returns [Fx,Fz,M]; kept for reference (hot path uses derivInto below)
 const V=Math.hypot(vx,vz); if(V<0.05) return [0,0,0];
 const gv=Math.atan2(vx,vz), al=wrap(th-gv), qd=0.5*RHO*V*V, sa=Math.sin(al);
 const sc=Math.sign(sa)*Math.min(Math.abs(sa), CLMAX/P.CN_total), N=qd*P.S_ref*P.CN_total*sc;  // total normal force (stall-capped)
 let Fx=N*Math.cos(gv), Fz=N*(-Math.sin(gv));
 const CD=P.CD0*Math.cos(al)**2 + CD_BROADSIDE*sa*sa, D=qd*P.S_ref*CD; Fx+=-D*vx/V; Fz+=-D*vz/V;  // axial + broadside
 const smEff=P.sm_max*(1-2*deploy);   // TWO-SIDED margin: deploy 0.5=neutral, <0.5 nose-first stable, >0.5 tail-first
 let M=-N*smEff*P.d_ref - P.C_mq*qd*P.S_ref*(P.d_ref**2)*q/Math.max(V,0.5);  // pitching moment + pitch damping
 return [Fx,Fz,M];
}
// ALLOCATION-FREE derivative: writes [vx,vz,ax,az,q,qdd] into `out` (aero inlined). The whole-flight auto-aim runs
// thousands of sims; avoiding per-step array allocation (was 7 .map/aero arrays per RK4 step) is a ~6x speedup.
function derivInto(s,thrust,delta,deploy,m,Iyy,Lnoz,windvx,out){
 const th=s[4], q=s[5], vx=s[2], vz=s[3];
 const Fxt=thrust*Math.sin(th+delta), Fzt=thrust*Math.cos(th+delta), Mtvc=thrust*Math.sin(delta)*Lnoz;
 let Fxa=0, Fza=0, Ma=0; const avx=vx-windvx, V=Math.hypot(avx,vz);   // AIR-relative velocity (wind shifts the airflow)
 if(V>=0.05){ const gv=Math.atan2(avx,vz), al=wrap(th-gv), qd=0.5*RHO*V*V, sa=Math.sin(al);
  const sc=Math.sign(sa)*Math.min(Math.abs(sa), CLMAX/P.CN_total), N=qd*P.S_ref*P.CN_total*sc;
  Fxa=N*Math.cos(gv); Fza=N*(-Math.sin(gv));
  const CD=P.CD0*Math.cos(al)**2 + CD_BROADSIDE*sa*sa, D=qd*P.S_ref*CD; Fxa+=-D*avx/V; Fza+=-D*vz/V;  // drag opposes AIR-relative motion
  const smEff=P.sm_max*(1-2*deploy); Ma=-N*smEff*P.d_ref - P.C_mq*qd*P.S_ref*(P.d_ref**2)*q/Math.max(V,0.5); }
 out[0]=vx; out[1]=vz; out[2]=(Fxt+Fxa)/m; out[3]=(Fzt+Fza)/m-G; out[4]=q; out[5]=(Mtvc+Ma)/Iyy;
}
const _k1=new Array(6),_k2=new Array(6),_k3=new Array(6),_k4=new Array(6),_tmp=new Array(6);  // RK4 scratch (reused)
function step(s,thrust,delta,deploy,m,Iyy,Lnoz,windvx){
 derivInto(s,thrust,delta,deploy,m,Iyy,Lnoz,windvx,_k1);
 for(let i=0;i<6;i++)_tmp[i]=s[i]+0.5*DT*_k1[i]; derivInto(_tmp,thrust,delta,deploy,m,Iyy,Lnoz,windvx,_k2);
 for(let i=0;i<6;i++)_tmp[i]=s[i]+0.5*DT*_k2[i]; derivInto(_tmp,thrust,delta,deploy,m,Iyy,Lnoz,windvx,_k3);
 for(let i=0;i<6;i++)_tmp[i]=s[i]+DT*_k3[i];     derivInto(_tmp,thrust,delta,deploy,m,Iyy,Lnoz,windvx,_k4);
 const r=new Array(6); for(let i=0;i<6;i++)r[i]=s[i]+DT/6*(_k1[i]+2*_k2[i]+2*_k3[i]+_k4[i]); return r;
}
// WIND = PREVAILING (steady, deterministic -> the auto-aim/search see this and aim into it) + STOCHASTIC gusts
// (an Ornstein-Uhlenbeck process added ONLY on the actually-flown trajectory; the closed-loop divert corrects them).
let windStrength=0, useADRC=true, useRealSensors=true, useGPS=true, useCalib=true;   // wind; ADRC; realistic IMU+baro; GPS; pad calibration (UI toggles)
// GPS (consumer u-blox class): SLOW (few Hz) + NOISY absolute fix = per-flight correlated BIAS (multipath/iono, ~const
// over a flight, does NOT average out) + per-fix random noise; VELOCITY is good (Doppler). Loosely fused with the IMU:
// GPS bounds the drift, IMU fills between fixes. RTK (cm) would be ~50x tighter -- see GPS_* constants.
const GPS_HZ=8, GPS_BIAS_STD=0.8, GPS_NOISE=0.6, GPS_VEL_NOISE=0.07, KP_GPS=0.12, KV_GPS=0.15;  // "reasonable" u-blox M8/M9+SBAS ~1 m CEP
// KP_GPS/KV_GPS retained for reference; the live estimator is now the EKF below (covariance-weighted gains).
// ---- loosely-coupled INS/GPS KALMAN FILTER: one per axis, state = [position, velocity, accel-bias] ----
// Strapdown-integrates the accelerometer (predict) and fuses GPS + baro (update). Estimating the accel-bias is what
// removes the drift; covariance-weighted gains self-tune (low when confident -> smooth, unlike the old fixed gain).
function m3mul(A,B){const C=[0,0,0,0,0,0,0,0,0];for(let i=0;i<3;i++)for(let j=0;j<3;j++){let s=0;for(let k=0;k<3;k++)s+=A[i*3+k]*B[k*3+j];C[i*3+j]=s;}return C;}
function m3T(A){return [A[0],A[3],A[6],A[1],A[4],A[7],A[2],A[5],A[8]];}
function kfPredict(st,P,a,dt,Q){ st[0]+=st[1]*dt; st[1]+=(a-st[2])*dt;   // pos+=vel·dt; vel+=(accel-bias)·dt; bias=const
  const F=[1,dt,0, 0,1,-dt, 0,0,1], Pn=m3mul(m3mul(F,P),m3T(F)); for(let i=0;i<9;i++)P[i]=Pn[i]+Q[i]; }   // P=FPFᵀ+Q
function kfUpdate(st,P,hi,m,R){   // scalar update of state element hi (0=pos, 1=vel); H = unit vector e_hi
  const PHt=[P[hi],P[3+hi],P[6+hi]], S=PHt[hi]+R, K=[PHt[0]/S,PHt[1]/S,PHt[2]/S], innov=m-st[hi];
  st[0]+=K[0]*innov; st[1]+=K[1]*innov; st[2]+=K[2]*innov;                // x += K·(meas - Hx)
  const IKH=[1,0,0,0,1,0,0,0,1]; IKH[hi]-=K[0]; IKH[3+hi]-=K[1]; IKH[6+hi]-=K[2];   // I - K·Hᵀ
  const Pn=m3mul(IKH,P); for(let i=0;i<9;i++)P[i]=Pn[i]; }
function windAt(t){ return windStrength; }   // prevailing component (steady); gusts added per-step in the flown pass
function gaussian(){ return Math.sqrt(-2*Math.log(1-Math.random()))*Math.cos(2*Math.PI*Math.random()); }
// REALISTIC SENSORS (hobby IMU+baro, calibrated). The controller flies on FUSED ESTIMATES, not truth: gyro -> attitude
// (small bias drift, no absolute ref), baro -> altitude (noisy but ABSOLUTE -> bounded), accelerometer -> horizontal
// velocity/position by integration (NO GPS -> DRIFTS, the precision-limiting error). True state still drives physics.
// MPU6050-class IMU (cheap, calibrated). KEY: a rocket has NO gravity reference in flight (accel reads thrust during
// burns, ~0 in freefall) -> attitude is GYRO-ONLY and drifts the whole flight. And the MPU6050's notable accel bias
// double-integrates into SEVERAL METRES of horizontal position drift -- the dominant precision error.
const GYRO_BIAS_STD=0.005, GYRO_NOISE=0.0012,   // rad/s: per-flight gyro in-run bias (~0.29 deg/s), white rate noise
      ACCEL_BIAS_STD=0.10, ACCEL_NOISE=0.05,    // m/s^2: per-flight accel bias (~10 mg) + noise/vibration -> pos DRIFT
      ACCEL_MISALIGN=0.015,                     // rad (~0.9 deg): accel cross-axis misalignment -> axial thrust leaks into the lateral
      VIB_FACTOR=8,                             // motor burn shakes the IMU: accel noise is ~8x higher DURING thrust -> integration drift
      BARO_NOISE=0.30, VZ_NOISE=0.30;           // axis at HIGH g (a static 1-g pad cal can't remove it) -> residual drift even calibrated
const ADRC_WC=6, ADRC_W0=30;                 // ADRC controller / ESO bandwidths (rad/s); b0=1 since control = angular accel
// NOMINAL (gust-free) descent reference: the flown divert corrects only the DEVIATION from this, so it never chases the
// whole approach (which overshoots) -- it nudges out the stochastic gust drift and lands on the pad, engine-down.
let NOMZ=[], NOMX=[], NOMVX=[];
function nomAt(z){ const n=NOMZ.length; if(!n) return [0,0];
  if(z>=NOMZ[0]) return [NOMX[0],NOMVX[0]]; if(z<=NOMZ[n-1]) return [NOMX[n-1],NOMVX[n-1]];
  for(let i=0;i<n-1;i++) if(z<=NOMZ[i] && z>=NOMZ[i+1]){ const f=(NOMZ[i]-z)/((NOMZ[i]-NOMZ[i+1])||1);
    return [NOMX[i]+f*(NOMX[i+1]-NOMX[i]), NOMVX[i]+f*(NOMVX[i+1]-NOMVX[i])]; }
  return [NOMX[n-1],NOMVX[n-1]]; }

// ---------------- LIVE GUIDANCE (grav-turn ascent + margin flip + vertical suicide burn + retro landing) ----------
function runSim(kick_deg, ignOff, rec, gusts=false, divert=true){   // gusts: stochastic wind (flown pass); divert: closed-loop targeting
 let s=[0,Z_TD,0,0,0,0], t=0, phase='boost', tIgn=0, flipped=false, cut=false, thRet=0, apogee=0, deploy=0.5, hIgn=0,
     retroHold=0, mp=0, delta=0, wTurb=0, z1=0, z2=0, z3=0, zx1=0, zx2=0, zx3=0, txInit=false, thCoastRef=0;  // z*=attitude ESO; zx*=transl ESO
 const KAb=49,KDb=10, KAr=110,KDr=20, kpc=30,kdc=25, kickT=0.6,kickD=0.4, dmax=P.delta_max;  // boost/retro/canard PD accel gains
 const wc=ADRC_WC, b1=3*ADRC_W0, b2=3*ADRC_W0*ADRC_W0, b3=ADRC_W0*ADRC_W0*ADRC_W0;            // ADRC attitude controller/ESO gains
 const KPX=4.0, KDX=4.5, STILT=Math.sin(40*Math.PI/180), STILT2=Math.sin(24*Math.PI/180), TAU_W=0.8, SIG_W=0.35*Math.abs(windStrength);
 const wcx=1.6, bx1=3*6, bx2=3*36, bx3=216;   // TRANSLATIONAL ADRC on the gust DEVIATION: LOW bw (wcx=1.6, ESO w0x=6) so it
 // corrects the slow drift SMOOTHLY instead of lurching at the noisy position estimate (the big ±5° leans looked jerky).
 const realSense=(gusts&&useRealSensors);   // realistic IMU+baro only on the flown pass (search/aim stay exact -> deterministic)
 const gyroBias=realSense?GYRO_BIAS_STD*gaussian():0, accelBias=realSense?ACCEL_BIAS_STD*gaussian():0, accelBiasZ=realSense?ACCEL_BIAS_STD*gaussian():0;
 // PRE-LAUNCH PAD CALIBRATION: estimate each bias from ~1.8 s of stationary samples (mean = true bias + noise/sqrt(N)) and
 // subtract it -> only the small RESIDUAL is left in flight. Every real flight controller does this; it's the dominant lever.
 const calG=GYRO_NOISE/Math.sqrt(600), calA=ACCEL_NOISE/Math.sqrt(600);   // 1-sigma residual after a 600-sample pad calibration
 const gyroBiasEst=(realSense&&useCalib)?gyroBias+calG*gaussian():0, gyroBiasRes=gyroBias-gyroBiasEst;   // post-cal residual gyro bias
 const accelMis=realSense?ACCEL_MISALIGN*gaussian():0;   // per-flight accel misalignment: axial SF leaks into lateral; pad cal sees only its g-part
 const accelBiasEst=(realSense&&useCalib)?accelBias+G*accelMis+calA*gaussian():0, accelBiasZEst=(realSense&&useCalib)?accelBiasZ+calA*gaussian():0;
 const gpsBias=(realSense&&useGPS)?GPS_BIAS_STD*gaussian():0, gpsBiasZ=(realSense&&useGPS)?GPS_BIAS_STD*gaussian():0;   // per-flight GPS biases
 let th_err=0, nextGps=0, vxB=0, vzB=0;   // attitude drift; next GPS-fix time; velocity-before-step (for the accelerometer)
 const st_h=[0,0,0], P_h=[0.04,0,0, 0,0.04,0, 0,0,0.04];   // EKF horizontal: state [x,vx,accel-bias]; the bias state absorbs the gravity-projection leak
 const st_v=[Z_TD,0,0], P_v=[0.04,0,0, 0,0.04,0, 0,0,0.04];   // EKF vertical: state [z,vz,accel-bias]
 const Qh=[1e-7,0,0, 0,(ACCEL_NOISE*DT)*(ACCEL_NOISE*DT),0, 0,0,1e-6], Qv=Qh.slice();   // process noise (pos / vel / bias-walk: larger to track the leak)
 const frames=[]; let i=0;
 while(t<26 && !(flipped && s[1]<=Z_TD)){   // land when DESCENDING CG reaches Z_TD (tail touches ground)
  // SENSOR FUSION: the controller sees ESTIMATES (gyro/baro/accel), not truth. Truth s[] still drives the physics,
  // rendering and the ACTUAL touchdown. Horizontal position xE drifts (accel double-integration, no GPS) -> this is the
  // realistic precision limit. Attitude drifts slowly (gyro bias); altitude/vert-vel are baro-bounded.
  // The controller flies on the EKF ESTIMATE (updated at the END of the previous step); truth s[] drives the physics.
  let xE, th, q, vx, vz, z;
  if(realSense){ xE=st_h[0]; vx=st_h[1]; z=st_v[0]; vz=st_v[1]; th=s[4]+th_err; q=s[5]+GYRO_NOISE*gaussian(); }
  else { xE=s[0]; th=s[4]; q=s[5]; vx=s[2]; vz=s[3]; z=s[1]; }
  const mr=(MPROP-mp)>0?(MPROP-mp):0, mass=M_DRY+mr, Iyy=IYY_DRY+mr*RG2, Lnoz=(M_DRY*L_DRY+mr*L_PROP)/mass;  // CURRENT mass/Iyy/L (migrate)
  if(gusts){ wTurb += (-wTurb/TAU_W)*DT + SIG_W*Math.sqrt(DT)*gaussian(); }   // stochastic gust (flown pass only)
  const windvx = windAt(t) + wTurb;                                          // prevailing + gust
  let thrust=0, dcmd=0, dep_t=deploy, thRef=0, act='none', pdP=0, pdD=0;
  if(s[3]<-0.3) flipped=true;   // APOGEE/descent detection from the baro altitude TREND (robust; a per-step noisy vz would false-trigger on the pad)
  // suicide-burn ignition (decided BEFORE dispatch, no 1-step delay). a_net uses PLATEAU thrust at the held tilt;
  // only cos(tilt) is vertical -> ignite higher on tilted descents. bestIgnOff refines ignOff to null vz at Z_TD.
  if(phase==='coast'){ const V=Math.hypot(vx,vz), al=wrap(th-Math.atan2(vx,vz)), qd=0.5*RHO*V*V,
       CD=P.CD0*Math.cos(al)**2+CD_BROADSIDE*Math.sin(al)**2, Dacc=qd*P.S_ref*CD/mass,
       an=Math.max(Math.cos(wrap(retroHold)),0.4)*CT/mass-G+Dacc, hs0=an>0.2?vz*vz/(2*an):1e9;
     if(flipped&&vz<-2.0&&z-Z_TD<=hs0+ignOff){phase='burn';tIgn=t;hIgn=z;} }
  // ---- GUIDANCE (outer loop): sets the desired body angle thRef + which actuator drives it ----
  if(phase==='boost'){
   thrust=(t<ASC)?CT*thrustShape(t):0; const Vb=Math.hypot(vx,vz);   // ascent: F-15 thrust CURVE; gravity-turn after the kick
   if(t<kickT) thRef=0; else if(t<kickT+kickD) thRef=kick_deg*Math.PI/180; else thRef=(Vb>2.0)?Math.atan2(vx,vz):kick_deg*Math.PI/180;
   act='tvc'; pdP=KAb; pdD=KDb; dep_t=0.5;
   if(t>=ASC){ phase='coast';   // AT BURNOUT: predict the landing-burn retrograde angle and hold it through coast.
     const apoP=z+vz*vz/(2*G), TW=CT/(mass*G), hIgnP=apoP/TW, vzIgn=-Math.sqrt(Math.max(0,2*G*(apoP-hIgnP)));
     retroHold=wrap(Math.atan2(vx, vzIgn)+Math.PI); thCoastRef=th; }   // start the flip reference at the CURRENT angle
  } else if(phase==='coast'){
   const remC=wrap(retroHold - thCoastRef); thCoastRef += clip(remC*2.5, -1.6, 1.6)*DT;   // DECELERATING approach (rate ~ remaining
   thRef=thCoastRef; thRet=retroHold; act='canard'; pdP=kpc; pdD=kdc;   // angle, capped) -> body slows INTO retrograde, no momentum overshoot
  } else { // burn
   const tau=t-tIgn; if(vz>=0) cut=true; thrust=(cut||tau>=LAND)?0:CT*thrustShape(tau); const sp=Math.hypot(vx,vz);
   // BASE guidance = RETROGRADE (kills velocity, lands ~on target via the kick aim), leveling to engine-down for the
   // last 3.5 m. This is identical to the ballistic search/nominal, so the search & flown trajectories AGREE.
   let thBase; if(z-Z_TD<3.5) thBase=0; else if(sp>3.0) thBase=wrap(Math.atan2(vx,vz)+Math.PI); else thBase=0;
   thRef=thBase;
   // DIVERT (flown pass): a SMALL correction tilt that drives the DEVIATION from the NOMINAL gust-free path to zero,
   // so the stochastic gusts don't push the landing off the pad. It corrects ONLY the gust drift (small) -- it does
   // NOT chase the whole approach (that overshoots). The translational ADRC ESO estimates & cancels the gust force.
   if(divert && thrust>1){ const nn=nomAt(z), dx=xE-nn[0], dvx=vx-nn[1];   // ESTIMATED deviation from nominal (xE drifts -> the miss)
    // Divert up high; below 3.5 m keep correcting ONLY while still meaningfully off-course (|dx|>0.8 m). Tiny near-
    // ground deviations don't make it fidget/wobble over the pad (clean vertical touchdown), but a LARGE drift (short
    // target -> high apogee -> long coast -> big gust drift) is still chased down to the deck -> on-target, accepting
    // some tilt. Prioritise hitting the pad when far off; prioritise vertical when already close.
    if(z-Z_TD>3.5 || Math.abs(dx)>1.5){   // near the ground, don't chase deviations SMALLER than the sensor accuracy
      // (~1 m GPS noise) -- that's just tilting to correct sensor error -> tip-over. Level instead; the ~1 m miss is
      // irreducible with these sensors anyway. Still chase a genuinely LARGE drift (>1.5 m).
     if(!txInit){ zx1=dx; zx2=dvx; zx3=0; txInit=true; }
     const ah = useADRC ? (-wcx*wcx*zx1 - 2*wcx*zx2 - zx3) : (-KPX*dx - KDX*dvx);   // drive deviation -> 0 (+cancel gust force zx3)
     const corr = Math.asin(clip(mass*ah/thrust, -STILT2, STILT2));
     thRef = thBase + corr;
     if(useADRC){ const axR=thrust*Math.sin(corr)/mass, e=dx-zx1; zx1+=DT*(zx2+bx1*e); zx2+=DT*(zx3+axR+bx2*e); zx3+=DT*(bx3*e); } } }
   thRet=thRef; act='tvc'; pdP=KAr; pdD=KDr; dep_t=0.5;
  }
  // ---- ATTITUDE (inner loop): command ANGULAR ACCEL to track thRef. ADRC = ESO estimates & cancels the disturbance
  // (wind torque, model error) via z3; PD = plain. b0=1 because the control IS angular accel (theta_ddot = alphaDes + dist).
  let alphaDes, Nc=0;
  if(useADRC) alphaDes = wc*wc*wrap(thRef - z1) - 2*wc*z2 - z3;   // ADRC: track + cancel estimated disturbance z3
  else        alphaDes = pdP*wrap(thRef - wrap(th)) + pdD*(0 - q);
  // INVERT through the CURRENT plant -> exert exactly alphaDes regardless of thrust curve / mass / CG / Iyy.
  if(act==='tvc'){ const sd=(thrust>1)? (Iyy*alphaDes)/(thrust*Lnoz) : 0; dcmd=clip(Math.asin(clip(sd,-1,1)),-dmax,dmax); }
  else if(act==='canard'){
     // SENSOR-BASED (the controller is NOT fed the wind): the IMU accelerometer measures the LATERAL specific force
     // the airframe feels = the aero side-force N. During coast (thrust off) that is a direct, wind-AGNOSTIC read --
     // the rocket senses the FORCE the wind produces, never the wind VELOCITY (windvx enters ONLY here, as the physics
     // the accelerometer measures). That sensed force gives the canard the correct sign & authority in any wind.
     // [Modeled as an ideal/noise-free IMU; a real accel adds noise -- TODO realistic sensor model.]
     const av=s[2]-windvx, V=Math.hypot(av,s[3]), gv=Math.atan2(av,s[3]), sa=Math.sin(wrap(s[4]-gv)),   // accel reads the TRUE
       sc=Math.sign(sa)*Math.min(Math.abs(sa),CLMAX/P.CN_total), N_imu=0.5*RHO*V*V*P.S_ref*P.CN_total*sc;  // lateral force (direct)
     const M_des=Iyy*alphaDes, den=N_imu*P.d_ref; Nc=N_imu;
     if(Math.abs(den)>2e-3) dep_t=clip(0.5 + M_des/(2*P.sm_max*den), 0, 1); else dep_t=deploy; }   // two-sided margin SET
  delta+=clip(dcmd-delta, -DSLEW*DT, DSLEW*DT);                 // TVC gimbal servo slew limit
  const ddmax=20.0*DT; if(phase==='boost') deploy=dep_t; else deploy+=clip(dep_t-deploy,-ddmax,ddmax);  // fast canard servo
  // ESO update with the ACTUAL realized angular accel (anti-windup): when the gimbal/canard saturates, the ESO must
  // NOT assume the full command was applied, or z3 winds up and over-rotates the body. uA = realized control accel.
  if(useADRC){ const uA=(act==='tvc')?((thrust>1)?thrust*Math.sin(delta)*Lnoz/Iyy:0):(act==='canard'?-Nc*smEff(deploy)*P.d_ref/Iyy:0);
     const e=wrap(th - z1); z1+=DT*(z2 + b1*e); z2+=DT*(z3 + uA + b2*e); z3+=DT*(b3*e); }
  if(thrust>0 && mp<MPROP){ mp=Math.min(MPROP, mp+thrust/VE*DT); }   // propellant consumed -> mass / CG / Iyy migrate
  vxB=s[2]; vzB=s[3];   // velocity BEFORE the step -> realized world accel = (after - before)/DT = what the accelerometer reads
  s=step(s,thrust,delta,deploy,mass,Iyy,Lnoz,windvx);
  if(phase==='boost' && s[1]<Z_TD){ s[1]=Z_TD; s[3]=Math.max(0,s[3]); s[0]=0; s[2]=0; }     // sit on pad (tail on ground = CG at Z_TD)
  if(realSense){   // ---- EKF: strapdown-propagate (body-frame accelerometer rotated by the ESTIMATED attitude) + fuse baro/GPS ----
    th_err += (gyroBiasRes + GYRO_NOISE*gaussian())*DT;   // calibrated gyro: only the small residual bias + white noise drift attitude
    // The accelerometer measures specific force in the BODY frame (pad calibration already subtracted). The strapdown rotates it
    // to world using the ESTIMATED attitude (s[4]+th_err) -> if th_err!=0, gravity/thrust leaks into the horizontal channel.
    // THIS is the dominant real INS drift, and it ties attitude error -> position error (so calibrating the gyro helps position too).
    const sfx=(s[2]-vxB)/DT, sfz=(s[3]-vzB)/DT+G, ct=Math.cos(s[4]), stt=Math.sin(s[4]);
    const sfUpT=stt*sfx + ct*sfz, sfLatT=ct*sfx - stt*sfz;   // TRUE body-frame specific force (up / lateral)
    const sfUp = sfUpT + (accelBiasZ-accelBiasZEst) + ACCEL_NOISE*gaussian();               // body up-axis (measured)
    const sfLat= sfLatT + sfUpT*accelMis + (accelBias-accelBiasEst) + ACCEL_NOISE*gaussian();// lateral + AXIAL LEAK (misalignment); pad cal removed only its g-part
    const ce=Math.cos(s[4]+th_err), se=Math.sin(s[4]+th_err);
    kfPredict(st_h,P_h, sfUp*se + sfLat*ce,     DT, Qh);   // est world accel x (gravity/thrust leak through th_err)
    kfPredict(st_v,P_v, sfUp*ce - sfLat*se - G, DT, Qv);   // est world accel z
    kfUpdate(st_v,P_v, 0, s[1] + BARO_NOISE*gaussian(), BARO_NOISE*BARO_NOISE);         // baro altitude (every step)
    if(useGPS && t>=nextGps){ nextGps += 1/GPS_HZ;                                      // GPS fix (8 Hz): pos + vel, both axes
      kfUpdate(st_h,P_h, 0, s[0]+gpsBias  + GPS_NOISE*gaussian(),     GPS_NOISE*GPS_NOISE);
      kfUpdate(st_h,P_h, 1, s[2]          + GPS_VEL_NOISE*gaussian(), GPS_VEL_NOISE*GPS_VEL_NOISE);
      kfUpdate(st_v,P_v, 0, s[1]+gpsBiasZ + GPS_NOISE*gaussian(),     GPS_NOISE*GPS_NOISE);
      kfUpdate(st_v,P_v, 1, s[3]          + GPS_VEL_NOISE*gaussian(), GPS_VEL_NOISE*GPS_VEL_NOISE); }
  }
  apogee=Math.max(apogee,s[1]);
  if(rec && (i%5===0)) frames.push({x:s[0],z:Math.max(s[1],0),th:s[4],gim:delta*180/Math.PI,dep:deploy,
     marg:smEff(deploy),thr:thrust>0.5?1:0,ph:phase,vx:s[2],vz:s[3],
     m:mass,tf:CT>0?thrust/CT:0,Iyy:Iyy,Lnoz:Lnoz,wind:windvx,dist:useADRC?z3:0,xe:xE});   // +wind, z3, ESTIMATED x (drift)
  i++; t+=DT;
 }
 return {frames, x:s[0], vz:s[3], vx:s[2], th:s[4]*180/Math.PI, apogee, hIgn};
}
function smEff(d){ return P.sm_max*(1-2*d); }   // two-sided margin (cal): d=0.5 neutral, <0.5 stable, >0.5 tail-first
function findKick(target, ignOff){
 // Aim BALLISTICALLY (divert OFF) so x is monotonic in kick -> clean bisection AND the divert keeps full +-authority
 // for the gusts. (With the divert on, x~=target for many kicks -> degenerate, no-margin aim that fails in gusts.)
 let lo=-30, hi=60; if(target<=runSim(lo,ignOff,false,false,false).x) return lo; if(target>=runSim(hi,ignOff,false,false,false).x) return hi;   // lo<0 leans UPWIND
 for(let k=0;k<16;k++){ const m=0.5*(lo+hi); (runSim(m,ignOff,false,false,false).x<target)?lo=m:hi=m; } return 0.5*(lo+hi);
}
// The soft-landing optimum is a knife-edge: the ideal ignition altitude is ~0.1 m wide, and the online h_stop formula
// can't reliably hit it (the descent moves ~0.06 m per sim step). So we SEARCH the ignition-altitude offset offline
// (re-running the deterministic sim) to drive touchdown |vz| to zero -- the same idea as the auto-aim, but for softness.
// touchdown vz(ignOff) is cleanly UNIMODAL (single sharp peak), so golden-section maximises vz (= minimises |vz|)
// robustly and cheaply -- a coarse grid would alias across the ~0.05 m-wide optimum and miss it.
function bestIgnOff(kick, div){   // div=false: ballistic (paired with findKick); div=true: tune ignition for the DIVERT burn (soft)
 let a=-9, b=2; const g=0.6180339887;
 let c=b-g*(b-a), d=a+g*(b-a), fc=runSim(kick,c,false,false,div).vz, fd=runSim(kick,d,false,false,div).vz;
 for(let i=0;i<18;i++){ if(fc>fd){b=d;d=c;fd=fc;c=b-g*(b-a);fc=runSim(kick,c,false,false,div).vz;}
                        else    {a=c;c=d;fc=fd;d=a+g*(b-a);fd=runSim(kick,d,false,false,div).vz;} }
 return (fc>fd)?c:d;
}

// ---------------- RENDER ----------------
const cv=document.getElementById('cv'), ctx=cv.getContext('2d'), W=cv.width, Hh=cv.height;
const PAD=46, GY=Hh-34, BL=30, BW=6;
let DATA=null, target=40, twRatio=2.0, fr=0, playing=true, speed=1.0, acc=0, last=performance.now(), dirty=true, sx,sy,zmax,xmin,xmax;
let pendingSince=0; const DEBOUNCE_MS=130;   // the full auto-aim+soft-landing solve is heavy; run it once AFTER the
// slider stops moving (not on every drag pixel) so dragging stays smooth. The target marker tracks live meanwhile.
function recompute(){
 CT=twRatio*P.m*G;
 // SIZE the rocket (Z_TD = glyph half-length in metres) from the view scale, so the glyph lands tail-on-ground and the
 // burn nulls vz right at Z_TD (full slowdown shown, no clamp/loiter). Estimate apogee (vertical) -> view ppm -> Z_TD.
 Z_TD=0; const apoEst=runSim(0,0,false,false,false).apogee;   // ballistic vertical apogee (cheap; divert/gusts off)
 const ppmEst=Math.min((W-2*PAD)/(Math.max(target,4)+6), (GY-22)/(1.08*(apoEst+1)));
 Z_TD=BL/ppmEst;
 let k=findKick(target,0), ignOff=0;  // 1) iterate the BALLISTIC kick + ignition (clean monotonic aim to the pad)
 for(let it=0;it<2;it++){ ignOff=bestIgnOff(k,false); k=findKick(target,ignOff); }
 const nom=runSim(k,ignOff,true,false,false);   // 2) NOMINAL gust-free reference (retrograde, divert off) -- store its descent
 NOMZ=[]; NOMX=[]; NOMVX=[]; for(const f of nom.frames){ if(f.vz<0){ NOMZ.push(f.z); NOMX.push(f.x); NOMVX.push(f.vx); } }
 const r=runSim(k,ignOff,true,true,true);  // 3) FLOWN pass: gusts=true + divert=true (tracks the nominal -> on the pad in gusts)
 DATA={...r, target};
 const xs=r.frames.map(f=>f.x), zs=r.frames.map(f=>f.z);
 xmin=Math.min(-2,...xs); xmax=Math.max(4,...xs,target+4); zmax=Math.max(5,Math.max(...zs)*1.08);
 // EQUAL ASPECT: same pixels-per-metre on both axes so the trajectory has true geometry (a 45deg arc looks 45deg)
 const xr=xmax-xmin, ppm=Math.min((W-2*PAD)/xr, (GY-22)/zmax), xoff=PAD+((W-2*PAD)-xr*ppm)/2;
 sx=x=>xoff+(x-xmin)*ppm; sy=z=>GY-z*ppm;
 fr=0; acc=0;
}
function L(a,b,w,c){ctx.strokeStyle=c;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();}
const phaseCol={boost:'#ffa657',coast:'#7ee787',burn:'#ff7b72'};
function draw(i){
 const F=DATA.frames, f=F[i]; ctx.clearRect(0,0,W,Hh);
 L([0,GY],[W,GY],2,'#30363d');
 ctx.fillStyle='#1f6feb';ctx.fillRect(sx(0)-16,GY,32,5);
 const tx=sx(target);   // LIVE target marker (tracks the slider during drag, before the trajectory recomputes)
 ctx.fillStyle='#e3b341';ctx.fillRect(tx-15,GY,30,5);
 ctx.strokeStyle='#e3b341';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(tx,GY);ctx.lineTo(tx,GY-26);ctx.stroke();
 ctx.fillStyle='#e3b341';ctx.beginPath();ctx.moveTo(tx,GY-26);ctx.lineTo(tx+13,GY-21);ctx.lineTo(tx,GY-16);ctx.closePath();ctx.fill();
 ctx.fillStyle='#586069';ctx.font='10px Consolas';
 for(let zz=0;zz<=zmax;zz+=Math.ceil(zmax/6)){ctx.fillText(zz+'m',2,sy(zz));L([26,sy(zz)],[W,sy(zz)],0.5,'#161b22');}
 ctx.strokeStyle='#243b53';ctx.lineWidth=2;ctx.beginPath();
 for(let k=0;k<=i;k++){const p=[sx(F[k].x),sy(F[k].z)];k?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]);}ctx.stroke();
 // rocket glyph -- SHIFTED up by half-length so the tail rests on the ground (no CG clipping)
 const th=f.th, nd=[Math.sin(th),-Math.cos(th)], pd=[-nd[1],nd[0]];
 const cx=sx(f.x), cy=Math.min(sy(f.z), GY-BL);   // CG sits ON the trajectory in flight; only lifts to rest the tail on the ground near touchdown
 const Pt=(al,pe)=>[cx+nd[0]*al+pd[0]*pe, cy+nd[1]*al+pd[1]*pe];
 const tail=Pt(-BL,0), nose=Pt(BL,0);
 L(tail,nose,2*BW,'#79c0ff');
 ctx.fillStyle='#a5d6ff';ctx.beginPath();const nt=Pt(BL+11,0),nl=Pt(BL,BW),nr=Pt(BL,-BW);
 ctx.moveTo(nt[0],nt[1]);ctx.lineTo(nl[0],nl[1]);ctx.lineTo(nr[0],nr[1]);ctx.closePath();ctx.fill();
 [1,-1].forEach(s=>{L(Pt(-BL+4,s*BW),Pt(-BL-4,s*(BW+11)),4,'#6e7681');});
 const clen=4+20*f.dep;
 [1,-1].forEach(s=>{L(Pt(BL-11,s*BW),Pt(BL-11,s*(BW+clen)),5,'#7ee787');});
 const gdir=th+Math.PI+f.gim*Math.PI/180, gd=[Math.sin(gdir),-Math.cos(gdir)];
 const noz1=[tail[0]+gd[0]*11,tail[1]+gd[1]*11]; L(tail,noz1,6,'#d2a8ff');
 if(f.thr){ctx.fillStyle='rgba(255,150,60,0.9)';ctx.beginPath();
   const a=[noz1[0]+gd[0]*4-gd[1]*5,noz1[1]+gd[1]*4+gd[0]*5],b=[noz1[0]+gd[0]*4+gd[1]*5,noz1[1]+gd[1]*4-gd[0]*5],tp=[noz1[0]+gd[0]*30,noz1[1]+gd[1]*30];
   ctx.moveTo(a[0],a[1]);ctx.lineTo(tp[0],tp[1]);ctx.lineTo(b[0],b[1]);ctx.closePath();ctx.fill();}
 const cg=Pt(0,0), cp=Pt(-f.marg*7,0);
 ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(cg[0],cg[1],3.5,0,7);ctx.fill();
 ctx.fillStyle='#ff33aa';ctx.beginPath();ctx.arc(cp[0],cp[1],3,0,7);ctx.fill();
 ctx.fillStyle=phaseCol[f.ph];ctx.font='bold 16px Segoe UI';ctx.fillText(f.ph.toUpperCase(),W-110,28);
 if(Math.abs(windStrength)>0.05){ const wy=20, wcx=W/2, dir=Math.sign(windStrength), ln=10+Math.abs(windStrength)*7;  // wind arrow
   ctx.strokeStyle='#79c0ff';ctx.lineWidth=2.5;ctx.beginPath();ctx.moveTo(wcx-dir*ln,wy);ctx.lineTo(wcx+dir*ln,wy);ctx.stroke();
   ctx.beginPath();ctx.moveTo(wcx+dir*ln,wy);ctx.lineTo(wcx+dir*ln-dir*7,wy-5);ctx.lineTo(wcx+dir*ln-dir*7,wy+5);ctx.closePath();ctx.fillStyle='#79c0ff';ctx.fill();
   ctx.font='11px Consolas';ctx.fillText('wind '+Math.abs(windStrength).toFixed(1)+' m/s',wcx-28,wy-9); }
}
function bar(v,lo,hi){const c=clip((v-lo)/(hi-lo),0,1);return `<div class="bar"><div class="fill" style="left:${c<0.5?c*100:50}%;width:${Math.abs(c-0.5)*100}%"></div></div>`;}
function side(i){const F=DATA.frames,f=F[i],m=DATA, soft=Math.abs(m.vz)<3.0 && Math.abs(m.th)<20;
 document.getElementById('side').innerHTML=
  `<div style="color:${phaseCol[f.ph]};font-weight:bold">${f.ph.toUpperCase()}</div>`+
  `t=${(i*5*DT).toFixed(2)} s<br>alt=${f.z.toFixed(1)} m  vz=${f.vz.toFixed(1)}<br>downrange=${f.x.toFixed(1)} m  vx=${f.vx.toFixed(1)}<br>`+
  `pitch=${(f.th*180/Math.PI).toFixed(1)}&deg; (0=engine-down)<br><br>`+
  `<b>TVC gimbal</b> ${f.gim.toFixed(1)}&deg;`+bar(f.gim,-6,6)+
  `<b>canard deploy</b> ${(f.dep*100).toFixed(0)}%`+bar(f.dep,0,1)+
  `<b>static margin</b> ${f.marg>=0?'+':''}${f.marg.toFixed(2)} cal`+bar(f.marg,-1.2,1.2)+
  `<b>thrust</b> ${(f.tf*100).toFixed(0)}% of plateau (curve)`+bar(f.tf,0,1.85)+
  `<b>mass</b> ${f.m.toFixed(3)} kg (${((1-(f.m-M_DRY)/MPROP)*100).toFixed(0)}% propellant burned)`+bar(f.m,M_DRY,M_DRY+MPROP)+
  `<b>Iyy</b> ${f.Iyy.toFixed(4)} &middot; <b>L<sub>nozzle</sub></b> ${f.Lnoz.toFixed(3)} m (CG migrates)<br>`+
  `<b>wind</b> ${f.wind.toFixed(1)} m/s (prevailing+gust)${useADRC?` &middot; <b>ADRC est&nbsp;dist</b> ${f.dist.toFixed(1)}`:''}`+`<br><br>`+
  `<div style="color:#8b949e">T/W=${twRatio.toFixed(2)} &middot; wind ${windStrength.toFixed(1)} m/s &middot; <b style="color:${useADRC?'#d2a8ff':'#8b949e'}">${useADRC?'ADRC':'PD'}</b> &middot; <b style="color:${useRealSensors?'#ffa657':'#8b949e'}">${useRealSensors?'EKF: IMU+baro'+(useGPS?'+GPS':''):'ideal sensors'}</b> &middot; apogee ${m.apogee.toFixed(1)} m<br>`+
  (useRealSensors?`<b>pos&#8209;estimate error</b> ${Math.abs(F[F.length-1].xe-F[F.length-1].x).toFixed(1)} m (${useGPS?'EKF&#8209;fused IMU+baro+GPS, ~0.6 m':'no GPS &rarr; horizontal unobservable, drifts'}; sets the precision floor)<br>`:'')+
  `<b>TOUCHDOWN</b> x=${m.x.toFixed(1)} m (target ${m.target}, miss ${Math.abs(m.x-m.target).toFixed(1)} m)<br>`+
  `vz=${m.vz.toFixed(2)} m/s, vx=${m.vx.toFixed(2)}, pitch=${m.th.toFixed(1)}&deg;<br>`+
  `${Math.abs(m.x-m.target)<2.0?'<b style="color:#7ee787">ON TARGET</b>':'<b style="color:#ffa657">off target</b>'}${soft?' &middot; <span style="color:#7ee787">soft</span>':' &middot; <span style="color:#ffa657">firm (wind)</span>'}</div>`;
}
const FMS=5*DT*1000;
function loop(now){const dt=now-last;last=now;
 if(dirty && now-pendingSince>=DEBOUNCE_MS){ recompute(); dirty=false; }   // debounced heavy solve
 if(playing&&DATA){acc+=dt*speed; while(acc>FMS){fr=(fr+1)%DATA.frames.length;acc-=FMS;}}   // sim already ends at touchdown (z=Z_TD)
 if(DATA){draw(fr); side(fr);}
 if(dirty){ ctx.fillStyle='#e3b341'; ctx.font='bold 13px Segoe UI'; ctx.fillText('↻ recomputing…', W/2-44, 28); }
 requestAnimationFrame(loop);}
function mark(){dirty=true; pendingSince=performance.now();}   // (re)start the debounce timer
document.getElementById('tgt').oninput=function(){target=+this.value;document.getElementById('tgtv').textContent=target+' m';mark();};
document.getElementById('tw').oninput=function(){twRatio=+this.value;document.getElementById('twv').textContent=twRatio.toFixed(2);mark();};
document.getElementById('play').onclick=function(){playing=!playing;this.innerHTML=playing?'&#10073;&#10073; Pause':'&#9658; Play';};
document.getElementById('restart').onclick=()=>{fr=0;acc=0;};
document.getElementById('speed').oninput=function(){speed=this.value/100;document.getElementById('spd').textContent=speed.toFixed(1)+'x';};
document.getElementById('wind').oninput=function(){windStrength=+this.value;document.getElementById('windv').textContent=windStrength.toFixed(1);mark();};
document.getElementById('ctrl').onclick=function(){useADRC=!useADRC;this.textContent='CONTROLLER: '+(useADRC?'ADRC':'PD');mark();};
document.getElementById('sens').onclick=function(){useRealSensors=!useRealSensors;this.textContent='SENSORS: '+(useRealSensors?'EKF (IMU+baro)':'ideal');mark();};
document.getElementById('gps').onclick=function(){useGPS=!useGPS;this.textContent='GPS: '+(useGPS?'on (~0.6 m)':'off');mark();};
document.getElementById('cal').onclick=function(){useCalib=!useCalib;this.textContent='CALIB: '+(useCalib?'on':'OFF (no pad cal)');mark();};
document.getElementById('gust').onclick=()=>mark();   // re-roll the stochastic gust (recompute draws fresh randoms)
requestAnimationFrame(loop);
</script></body></html>"""

def main():
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "landing_interactive.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(HTML)
    print("wrote", path, "(%d bytes)" % len(HTML))


if __name__ == "__main__":
    main()
