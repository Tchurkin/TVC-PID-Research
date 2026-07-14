#!/usr/bin/env python3
"""
Closed-loop validator for the Ascent_TVC quaternion estimator.

Ports the EXACT physics of Firmware/sim_ascent/ascent_sim.cpp (quaternion rigid body with
(Iyy-Izz)*w*w gyroscopic coupling, roll-torque sources, servo lag+slew, F-15 thrust) and runs
the firmware's control law with EITHER estimator:
  - 'naive' : gyro_x += ang_vel_x*dt   (the old code -- treats body rates as inertial tilt)
  - 'quat'  : the new roll-aware quaternion estimator I just put in Ascent_TVC.ino

Question under test: under a roll source, does the naive estimate diverge from truth (roll-coupling
failure) while the quaternion tracks truth and keeps the loop controlled? And in the NO-roll case,
do they agree (so the pole-placed gains are unchanged)?
"""
import math

RAD2DEG = 57.2957795; DEG2RAD = math.pi/180.0
# ---- physics constants (verbatim from ascent_sim.cpp) ----
G=9.80665; SERVO_MULT=5.0; IGN_DELAY=0.2; BURN=3.45
IYY0=0.0078; L_ARM0=0.14; MASS=0.818; IZZ0=4.5e-4
S_REF=0.003; CD=0.6; RHO=1.20; CN_AERO=2.0
C_DAMP=0.003; C_DAMP_Z=2.0e-5; SERVO_TAU=0.03
# ---- firmware control constants (Ascent_TVC.ino) ----
P_GAIN=0.249; D_GAIN=0.062; I_GAIN=0.20; MAX_TILT=5.0; ANGVEL_ALPHA=0.9
SERVO_SIGN=+1   # current firmware (matches the flipped linkage); SIL: +1 == restoring

def thrustAt(tau):
    if tau<0 or tau>BURN: return 0.0
    if tau<0.15: return 26.0*tau/0.15
    if tau<0.50: return 26.0-(26.0-14.0)*(tau-0.15)/0.35
    return 14.0

def qRotB2W(Q,bx,by,bz):
    Qw,Qx,Qy,Qz=Q
    t2=Qw*Qx;t3=Qw*Qy;t4=Qw*Qz;t5=-Qx*Qx;t6=Qx*Qy;t7=Qx*Qz;t8=-Qy*Qy;t9=Qy*Qz;t10=-Qz*Qz
    wx=2*((t8+t10)*bx+(t6-t4)*by+(t3+t7)*bz)+bx
    wy=2*((t4+t6)*bx+(t5+t10)*by+(t9-t2)*bz)+by
    wz=2*((t7-t3)*bx+(t2+t9)*by+(t5+t8)*bz)+bz
    return wx,wy,wz
def qRotW2B(Q,wx,wy,wz):
    Qw,Qx,Qy,Qz=Q
    t2=Qw*Qx;t3=Qw*Qy;t4=Qw*Qz;t5=-Qx*Qx;t6=Qx*Qy;t7=Qx*Qz;t8=-Qy*Qy;t9=Qy*Qz;t10=-Qz*Qz
    bx=2*((t8+t10)*wx+(t6+t4)*wy+(t7-t3)*wz)+wx
    by=2*((t6-t4)*wx+(t5+t10)*wy+(t2+t9)*wz)+wy
    bz=2*((t3+t7)*wx+(t9-t2)*wy+(t5+t8)*wz)+wz
    return bx,by,bz
def qnorm(Q):
    n=math.sqrt(sum(c*c for c in Q)); return tuple(c/n for c in Q) if n>1e-12 else (1,0,0,0)

# ---- firmware estimator (mirrors Ascent_TVC.ino exactly) ----
class Estimator:
    def __init__(self,kind):
        self.kind=kind
        self.gx=0.0; self.gy=0.0; self.gz=0.0            # tilt X/Y (deg), roll (deg)
        self.avx=0.0; self.avy=0.0                        # filtered body rates (deg/s)
        self.q=(1.0,0.0,0.0,0.0)                          # quaternion (quat mode)
    def from_accel(self,ax,ay,az):
        n=math.sqrt(ax*ax+ay*ay+az*az)
        if n<1e-6: self.q=(1,0,0,0); return
        ax,ay,az=ax/n,ay/n,az/n
        qw=1.0+az; qx=ay; qy=-ax; qz=0.0
        if qw<1e-6: qw,qx,qy,qz=0,1,0,0
        self.q=qnorm((qw,qx,qy,qz))
    def propagate(self,gxd,gyd,gzd,dt):
        qw,qx,qy,qz=self.q; wx,wy,wz=gxd*DEG2RAD,gyd*DEG2RAD,gzd*DEG2RAD
        dw=0.5*(-qx*wx-qy*wy-qz*wz); dx=0.5*(qw*wx+qy*wz-qz*wy)
        dy=0.5*(qw*wy-qx*wz+qz*wx); dz=0.5*(qw*wz+qx*wy-qy*wx)
        self.q=qnorm((qw+dw*dt,qx+dx*dt,qy+dy*dt,qz+dz*dt))
    def up_in_body(self):
        qw,qx,qy,qz=self.q
        return (2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy))
    def update(self,imu_gx,imu_gy,imu_gz,ax,ay,az,inflight,dt):
        # ang_vel filtering (both modes): raw_avx=imu_gx, raw_avy=-imu_gy
        self.avx=ANGVEL_ALPHA*imu_gx+(1-ANGVEL_ALPHA)*self.avx
        self.avy=ANGVEL_ALPHA*(-imu_gy)+(1-ANGVEL_ALPHA)*self.avy
        if self.kind=='naive':
            if inflight and 0<dt<0.1: self.gx+=self.avx*dt; self.gy+=self.avy*dt
            else:
                self.gx= math.atan2(ay,az)*RAD2DEG; self.gy=-math.atan2(ax,az)*RAD2DEG
        else:  # quaternion
            if inflight:
                if 0<dt<0.1: self.propagate(imu_gx,-imu_gy,imu_gz,dt)
            else: self.from_accel(ax,ay,az)
            ubx,uby,ubz=self.up_in_body()
            self.gx= math.atan2(uby,ubz)*RAD2DEG; self.gy=-math.atan2(ubx,ubz)*RAD2DEG
        if 0<dt<0.1: self.gz+=imu_gz*dt

def run(kind, rollcant=0.0, cgoffx=0.0, cgoffy=0.0, misx=0.0, misy=0.0, sm=0.0, tipx=0.0, tipy=0.0):
    Q=(1.0,0.0,0.0,0.0); wbx=wby=wbz=0.0; alt=0.0; vz=0.0
    servoXa=90.0; servoYa=90.0; cmdX=90.0; cmdY=90.0
    iyy=IYY0; izz=IZZ0; L=L_ARM0
    est=Estimator(kind)
    iTermX=iTermY=0.0
    lifted=False; t=0.0
    H=0.001; ctrl_dt=0.005
    max_true=0.0; max_err=0.0; aborted=False; boost_end_true=0.0; max_roll=0.0
    est_at_maxtrue=0.0
    nsteps=int((IGN_DELAY+BURN+0.2)/ctrl_dt)
    for k in range(nsteps):
        T_ctrl=thrustAt(t-IGN_DELAY)
        boost = T_ctrl>0.5
        # ---- advance physics 5 x 1ms with the current servo command ----
        for _ in range(5):
            T=thrustAt(t-IGN_DELAY)
            if not lifted:
                if T>MASS*G: lifted=True; wbx=tipx*DEG2RAD; wby=tipy*DEG2RAD
            else:
                mx=400.0*H  # slew (use nominal 400 dps unloaded; MC would vary)
                aLag=min(H/SERVO_TAU,1.0)
                sX=aLag*(cmdX-servoXa); sX=max(-mx,min(mx,sX)); servoXa+=sX
                sY=aLag*(cmdY-servoYa); sY=max(-mx,min(mx,sY)); servoYa+=sY
                tvcX=(servoXa-90.0)/SERVO_MULT*DEG2RAD; tvcY=(servoYa-90.0)/SERVO_MULT*DEG2RAD
                qdyn=0.5*RHO*vz*vz
                Mx=-T*L*tvcX; My=-T*L*tvcY
                Mx+= T*L*(misx*DEG2RAD)-cgoffy*T; My+= T*L*(misy*DEG2RAD)+cgoffx*T
                # aero normal-force moment (world-referenced crossflow -> rotates in body under roll)
                vbx,vby,vbz=qRotW2B(Q,0,0,vz); Vs=max(math.sqrt(vbx*vbx+vby*vby+vbz*vbz),0.5)
                kAero=qdyn*S_REF*CN_AERO/Vs; Mx+= sm*kAero*(-vby); My+= sm*kAero*(vbx)
                Mx+= -C_DAMP*wbx; My+= -C_DAMP*wby
                Mz= T*rollcant + T*(cgoffx*tvcY-cgoffy*tvcX) + qdyn*0.0 - C_DAMP_Z*wbz
                dwx=(Mx+(iyy-izz)*wby*wbz)/iyy; dwy=(My+(izz-iyy)*wbx*wbz)/iyy; dwz=Mz/izz
                wbx+=dwx*H; wby+=dwy*H; wbz+=dwz*H
                Qw,Qx,Qy,Qz=Q
                dQw=0.5*(-Qx*wbx-Qy*wby-Qz*wbz); dQx=0.5*(Qw*wbx+Qy*wbz-Qz*wby)
                dQy=0.5*(Qw*wby-Qx*wbz+Qz*wbx); dQz=0.5*(Qw*wbz+Qx*wby-Qy*wbx)
                Q=qnorm((Qw+dQw*H,Qx+dQx*H,Qy+dQy*H,Qz+dQz*H))
                zbx,zby,zbz=qRotB2W(Q,0,0,1)
                az=T*zbz/MASS-G-0.5*RHO*S_REF*CD*vz*abs(vz)/MASS
                vz+=az*H; alt+=vz*H
                if alt<0: alt=0.0; vz=max(0.0,vz)
            t+=H
        # ---- synth sensors (body frame; gyro Y inverted at the sensor, as the SIL/hardware do) ----
        imu_gx=wbx*RAD2DEG; imu_gy=-wby*RAD2DEG; imu_gz=wbz*RAD2DEG
        azw=(T_ctrl*qRotB2W(Q,0,0,1)[2]/MASS-G-0.5*RHO*S_REF*CD*vz*abs(vz)/MASS) if lifted else 0.0
        fbx,fby,fbz=qRotW2B(Q,0,0,azw+G)
        ax,ay,az=fbx,fby,fbz
        # ---- firmware estimate + control ----
        est.update(imu_gx,imu_gy,imu_gz,ax,ay,az,inflight=lifted,dt=ctrl_dt)
        idt=ctrl_dt
        rawX=P_GAIN*est.gx+iTermX+D_GAIN*est.avx; rawY=P_GAIN*est.gy+iTermY+D_GAIN*est.avy
        if abs(rawX)<MAX_TILT: iTermX=max(-MAX_TILT,min(MAX_TILT,iTermX+I_GAIN*est.gx*idt))
        if abs(rawY)<MAX_TILT: iTermY=max(-MAX_TILT,min(MAX_TILT,iTermY+I_GAIN*est.gy*idt))
        tiltX=max(-MAX_TILT,min(MAX_TILT,P_GAIN*est.gx+iTermX+D_GAIN*est.avx))*SERVO_MULT
        tiltY=max(-MAX_TILT,min(MAX_TILT,P_GAIN*est.gy+iTermY+D_GAIN*est.avy))*SERVO_MULT
        cmdX=SERVO_SIGN*tiltX+90.0; cmdY=SERVO_SIGN*tiltY+90.0
        # ---- metrics (TRUE tilt/roll from physics Q) ----
        zbz=qRotB2W(Q,0,0,1)[2]; true_tilt=math.acos(max(-1,min(1,zbz)))*RAD2DEG
        roll_true=abs(2*math.atan2(Q[3],Q[0])*RAD2DEG)
        est_tilt=math.sqrt(est.gx*est.gx+est.gy*est.gy)
        if boost:
            if true_tilt>max_true: max_true=true_tilt; est_at_maxtrue=est_tilt
            if abs(est_tilt-true_tilt)>max_err: max_err=abs(est_tilt-true_tilt)
            boost_end_true=true_tilt
            if roll_true>max_roll: max_roll=roll_true
            # firmware emergency: |gyro_x|>45 or |gyro_y|>45 (ESTIMATED) during powered flight
            if abs(est.gx)>45 or abs(est.gy)>45: aborted=True
    outcome = "ABORT" if aborted else ("PASS" if max_true<10 else ("MARGINAL" if max_true<44 else "TUMBLE"))
    return dict(outcome=outcome, max_true=max_true, boost_end=boost_end_true,
                est_err=max_err, roll=max_roll, est_at_peak=est_at_maxtrue)

def show(title, **kw):
    print(f"\n### {title}")
    for kind in ('naive','quat'):
        r=run(kind,**kw)
        print(f"  {kind:6s}: {r['outcome']:8s}  trueTilt_max={r['max_true']:6.1f}  boostEnd={r['boost_end']:5.1f}  "
              f"estERR_max={r['est_err']:6.1f}  roll_max={r['roll']:5.0f}  (est@peak={r['est_at_peak']:.1f})")

print("="*96)
print("CLOSED-LOOP: naive body-integration  vs  new quaternion estimator   (SERVO_SIGN=+1, restoring)")
print("outcome PASS<10deg  MARGINAL<44  TUMBLE>=44  ABORT=firmware est-tilt tripped 45deg emergency")
print("="*96)
show("A. NOMINAL (no roll source) -- must agree + PASS (gains preserved)")
show("B. Mild roll (nozzle cant 1.5e-5)", rollcant=1.5e-5)
show("C. ASC007-like (cant 3e-5 + CG offset 3mm + 2deg misalign)", rollcant=3e-5, cgoffx=0.003, misx=2.0)
show("D. Strong roll (cant 5e-5 + CG offset 4mm both + 2deg misalign both)",
     rollcant=5e-5, cgoffx=0.004, cgoffy=0.004, misx=2.0, misy=2.0)
show("E. Unstable airframe (neg static margin) + roll", rollcant=3e-5, cgoffx=0.003, sm=-0.005, misx=1.5)
show("F. Saturating lean (5deg misalign) + strong roll", rollcant=5e-5, misx=5.0, misy=3.0)
show("G. Big lean (6deg mis + 7mm CG offset) + strong roll", rollcant=6e-5, cgoffx=0.007, cgoffy=0.005, misx=6.0, misy=4.0)
show("H. Big lean + fast roll + tip-off", rollcant=8e-5, cgoffx=0.006, misx=5.0, misy=5.0, tipx=6.0, tipy=4.0)

print("\n" + "="*96)
print("SWEEP: steady lean (misalign X, deg) vs max TRUE tilt, with a strong roll source (cant 6e-5, CG 5mm)")
print("  -> the disturbance where each estimator loses control. quaternion should tolerate a larger lean.")
print("="*96)
print(f"  {'misalign':>8s} | {'naive true':>10s} {'naive out':>9s} | {'quat true':>10s} {'quat out':>9s}")
for mis in (2,3,4,5,6,7,8):
    rn=run('naive',rollcant=6e-5,cgoffx=0.005,cgoffy=0.004,misx=mis,misy=mis*0.6)
    rq=run('quat', rollcant=6e-5,cgoffx=0.005,cgoffy=0.004,misx=mis,misy=mis*0.6)
    print(f"  {mis:8.1f} | {rn['max_true']:10.1f} {rn['outcome']:>9s} | {rq['max_true']:10.1f} {rq['outcome']:>9s}")
