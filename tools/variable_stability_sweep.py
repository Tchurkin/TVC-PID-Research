"""
tools/variable_stability_sweep.py  (2026-06-25)

Maps the FEASIBLE OPERATING REGION for switched variable-stability pitch maneuvers:
    (how-unstable you fling)  x  (how-fast the mechanism morphs)  x  (TVC backstop authority)
Classifies each combo as:
    SUCCESS    reaches 30deg, settles, overshoot < 15 deg, no divergence  (and beats fixed-stable)
    MARGINAL   reaches but overshoot 15-40 deg
    DIVERGE    crashes (the danger zone)
    NO-BENEFIT never reaches in 3 s (no faster than just flying stable)

Goal: find a window that is BOTH effective (clearly beats fixed-stable) AND survivable at
realistic servo morph speeds, away from the divergence ridge.
"""
import numpy as np

IYY=0.015; M=0.55; V=60.0; S=np.pi*(0.05/2)**2; CN=8.0; RHO=1.20; DREF=0.05
Q=0.5*RHO*V*V; DT=0.002; T_END=3.0; THETA_SP=np.deg2rad(30.0)
STABLE_SM=0.5; SWITCH_DEG=6.0; KP=60.0; KD=10.0

def k_aero(sm): return Q*S*CN*(sm*DREF)

def run(unstable_sm, sm_rate, tvc, mode='switched'):
    theta=0.0; w=0.0; gamma=0.0; sm=STABLE_SM
    t_reach=None; overshoot=0.0; diverged=False; settled=False
    for k in range(int(T_END/DT)):
        err=theta-THETA_SP; alpha=theta-gamma
        if mode=='stable': sm_cmd=STABLE_SM
        else: sm_cmd = unstable_sm if abs(np.rad2deg(err))>SWITCH_DEG else STABLE_SM
        sm += np.clip(sm_cmd-sm, -sm_rate*DT, sm_rate*DT)
        u=np.clip(-(KP*err+KD*w), -tvc, tvc)
        w += ((IYY*u - k_aero(sm)*alpha)/IYY)*DT
        theta += w*DT
        gamma += (Q*S*CN*alpha/(M*V))*DT
        if abs(np.rad2deg(theta))>120: diverged=True; break
        if t_reach is None and abs(np.rad2deg(theta-THETA_SP))<2.0: t_reach=k*DT
        if theta>THETA_SP: overshoot=max(overshoot,np.rad2deg(theta-THETA_SP))
        settled = abs(np.rad2deg(theta-THETA_SP))<2.0
    return dict(t_reach=t_reach, overshoot=round(overshoot,1), diverged=diverged, settled=settled)

def classify(r, stable_reach):
    if r['diverged']: return 'DIVERGE'
    if r['t_reach'] is None: return 'NO-BENEFIT'
    if r['overshoot']>40: return 'DIVERGE'
    if r['overshoot']>15: return 'MARGINAL'
    # success only if it actually beats flying stable (faster, or stable never reaches)
    if stable_reach is None or r['t_reach'] < 0.8*stable_reach: return 'SUCCESS'
    return 'NO-BENEFIT'

def main():
    UNSTABLE=[-0.1,-0.2,-0.35,-0.5,-0.8]
    RATES=[5,10,20,40,80]    # cal/s ; flip of |stable-unstable| takes (swing/rate) s
    TVC=[15,30,60,100]
    sym={'SUCCESS':' S ','MARGINAL':' m ','DIVERGE':' X ','NO-BENEFIT':' . '}
    print("=== VARIABLE-STABILITY FEASIBLE-REGION SWEEP ===")
    print("  S=success(beats stable, clean)  m=marginal  X=diverge/crash  .=no-benefit")
    print(f"  stable cruise SM=+{STABLE_SM} cal; maneuver to 30deg; rows=unstable_sm, cols=morph rate (cal/s)\n")
    for tvc in TVC:
        st=run(0,80,tvc,mode='stable'); stable_reach=st['t_reach']
        sr = f"{stable_reach:.2f}s" if stable_reach else "NEVER"
        print(f"  --- TVC backstop = {tvc} rad/s^2   (fixed-stable reaches 30deg in: {sr}) ---")
        print("      morph:   " + "  ".join(f"{r:>4}" for r in RATES) + "   (cal/s; flip time for 0.8cal swing)")
        print("               " + "  ".join(f"{800/r/10:>4.0f}" for r in RATES) + "   (ms)")
        for us in UNSTABLE:
            cells=[]
            for rate in RATES:
                r=run(us,rate,tvc); cells.append(sym[classify(r,stable_reach)])
            print(f"   us={us:+.2f}cal: " + " ".join(cells))
        print()
    print("READ: look for a block of 'S' at REALISTIC morph rates (10-20 cal/s ~ 40-80ms flip) and")
    print("buildable TVC. A wide S-block = robust project; S only at extreme rates = knife-edge.")

if __name__=='__main__': main()
