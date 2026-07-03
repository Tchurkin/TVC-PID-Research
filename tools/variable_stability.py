"""
tools/variable_stability.py  (2026-06-25)

Tests the user's idea: MODULATE aerodynamic static stability in flight to control a
pitch maneuver -- go UNSTABLE to accelerate toward the setpoint (F-16-like agility),
then STABLE to settle. Compares three modes for a commanded pitch-over to theta_sp:
  (A) fixed STABLE        (conventional: TVC fights aero restoring -> slow but clean)
  (B) fixed UNSTABLE      (TVC + amplifying aero -> fast but hard to settle)
  (C) SWITCHED            (unstable while far from setpoint, stable when near)

Model (planar, powered cruise): aero acts on ANGLE OF ATTACK alpha = theta - gamma.
  Iyy*theta_ddot = M_tvc + M_aero,   M_aero = -k_aero * alpha   (k_aero>0 stable)
  gamma_dot = q*S*CN_alpha*alpha / (m*V)     (normal force turns the velocity vector)
  TVC: PID on (theta - theta_sp), bounded by theta_ddot_max (TVC authority).
Stability is modulated by flipping the SIGN/size of k_aero (via a moving CG / moving CP
mechanism). |k_aero| is bounded by a realizable static-margin swing.
"""
import numpy as np

IYY=0.015; M=0.55; V=60.0; S=np.pi*(0.05/2)**2; CN=8.0; RHO=1.20; DREF=0.05
Q=0.5*RHO*V*V
DT=0.002; T_END=3.0
THETA_SP=np.deg2rad(30.0)

def k_aero(sm_cal):  # N*m per rad of alpha
    return Q*S*CN*(sm_cal*DREF)

def run(mode, tvc_ddot_max=30.0, sm_mag=1.0, switch_deg=6.0, kp=60.0, kd=10.0,
        sm_rate=np.inf):
    theta=0.0; w=0.0; gamma=0.0
    sm=+sm_mag                                   # actual (rate-limited) static margin state
    t_reach=None; settled_since=None; overshoot=0.0; max_alpha=0.0; diverged=False
    n=int(T_END/DT)
    for k in range(n):
        err=theta-THETA_SP
        alpha=theta-gamma
        max_alpha=max(max_alpha,abs(np.rad2deg(alpha)))
        # --- commanded stability mode ---
        if mode=='stable':      sm_cmd=+sm_mag
        elif mode=='unstable':  sm_cmd=-sm_mag
        else:                   # switched: unstable when far, stable when near
            sm_cmd = -sm_mag if abs(np.rad2deg(err))>switch_deg else +sm_mag
        # --- rate-limit the morphing mechanism (servo moving a surface / mass) ---
        dsm=np.clip(sm_cmd-sm, -sm_rate*DT, sm_rate*DT); sm+=dsm
        ka=k_aero(sm)
        # --- TVC control (PID on attitude error), authority-bounded ---
        u=np.clip(-(kp*err+kd*w), -tvc_ddot_max, tvc_ddot_max)   # rad/s^2
        M_tvc=IYY*u
        M_aero=-ka*alpha
        w+=((M_tvc+M_aero)/IYY)*DT
        theta+=w*DT
        gamma+=(Q*S*CN*alpha/(M*V))*DT
        if abs(np.rad2deg(theta))>120: diverged=True; break
        # metrics
        if t_reach is None and abs(np.rad2deg(theta-THETA_SP))<2.0: t_reach=k*DT
        if theta>THETA_SP: overshoot=max(overshoot,np.rad2deg(theta-THETA_SP))
        if abs(np.rad2deg(theta-THETA_SP))<2.0:
            if settled_since is None: settled_since=k*DT
        else:
            settled_since=None
    settle_time=settled_since if settled_since is not None else None
    return dict(mode=mode, t_reach=t_reach, settle_t=settle_time, overshoot=round(overshoot,1),
                max_alpha=round(max_alpha,1), diverged=diverged)

def main():
    print("=== VARIABLE-STABILITY MANEUVER: pitch-over to 30 deg ===")
    print(f"  V={V}m/s q={Q:.0f}Pa  aero stiffness at SM=1cal: k_aero={k_aero(1.0):.2f} Nm/rad")
    print(f"  (aero angular accel at alpha=10deg: {np.rad2deg(k_aero(1.0)*np.deg2rad(10)/IYY):.0f} rad/s^2)\n")
    for tvc in [15.0, 30.0, 60.0]:
        print(f"  --- TVC authority = {tvc:.0f} rad/s^2 ---")
        for mode in ['stable','unstable','switched']:
            r=run(mode, tvc_ddot_max=tvc)
            tr = f"{r['t_reach']:.2f}s" if r['t_reach'] else "NEVER"
            st = f"{r['settle_t']:.2f}s" if r['settle_t'] else "no"
            print(f"     {mode:9}: reach2deg={tr:>7}  settled@={st:>6}  overshoot={r['overshoot']:5.1f}deg  "
                  f"max_alpha={r['max_alpha']:5.1f}deg  diverged={r['diverged']}")
        print()
    print("=== FEASIBILITY: switched mode with FINITE morphing speed (sm_rate cal/s) ===")
    print("  switching +1<->-1 cal = 2 cal swing; rate R cal/s -> takes 2/R seconds to flip.\n")
    for R,label in [(np.inf,'instant'),(40,'0.05s flip (very fast servo)'),
                    (20,'0.10s flip'),(10,'0.20s flip'),(5,'0.40s flip (slow)')]:
        r=run('switched', tvc_ddot_max=30.0, sm_rate=R)
        tr=f"{r['t_reach']:.2f}s" if r['t_reach'] else "NEVER"
        print(f"  sm_rate={str(R):>5} ({label:28}): reach={tr:>7} overshoot={r['overshoot']:5.1f}deg "
              f"max_alpha={r['max_alpha']:5.1f}deg diverged={r['diverged']}")
    print("\nKEY: switched only works on hardware if the morphing mechanism can flip stability")
    print("fast enough. If it diverges at realistic servo speeds -> mechanism is the bottleneck.")

if __name__=='__main__': main()
