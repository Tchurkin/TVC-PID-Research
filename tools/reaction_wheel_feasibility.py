"""
tools/reaction_wheel_feasibility.py  (2026-06-25)

FEASIBILITY of reaction-wheel attitude control on a MODEL ROCKET, focused on the COAST
phase (where TVC is dead because there is no thrust to vector).

Reaction wheel: a motor spins a flywheel (inertia I_w). Accelerating the wheel applies an
equal-and-opposite torque to the rocket: tau_rocket = -I_w * wheel_accel.
TWO hard limits:
  1. TORQUE:    |tau| <= tau_max   (motor stall torque)  -> sets angular-accel authority
  2. MOMENTUM:  |I_w * omega_wheel| <= H_max = I_w*omega_max  (wheel SATURATION)
     A sustained disturbance spins the wheel up until it saturates, after which the wheel
     can no longer help. This momentum budget is the binding constraint for reaction wheels.
Unlike moving-mass/TVC, authority is AIRSPEED-INDEPENDENT -> it works in coast and at apogee.

1-DOF pitch, near-vertical. Tests: (a) coast attitude HOLD against gusts; (b) a commanded
SLEW (point to a target) and whether the wheel saturates; (c) the momentum budget.
"""
import numpy as np

# ---- rocket ----
IYY     = 0.015     # kg m^2
D_REF   = 0.05; S_REF = np.pi*(D_REF/2)**2; RHO = 1.20
DT      = 0.005

# ---- reaction wheel options (small / medium / large for a model rocket) ----
WHEELS = {
    'small  (20g, 3cm, small motor)':  dict(I_w=1.0e-5, tau_max=0.010, omega_max=1500),
    'medium (40g, 4cm, decent BLDC)':  dict(I_w=4.5e-5, tau_max=0.030, omega_max=1500),
    'large  (80g, 5cm, strong BLDC)':  dict(I_w=1.2e-4, tau_max=0.060, omega_max=1800),
}

def coast_q(v): return 0.5*RHO*v*v

def sim_hold(wheel, v_coast=15.0, sm_cal=1.0, wind_amp=0.20, seed=0, kp=0.6, kd=0.15):
    """Coast attitude HOLD against wind gusts at a representative coast airspeed."""
    rng=np.random.default_rng(seed)
    Iw=wheel['I_w']; tmax=wheel['tau_max']; wmax=wheel['omega_max']; Hmax=Iw*wmax
    theta=np.deg2rad(5.0); q_rate=0.0; w_wheel=0.0; wind=0.0
    q=coast_q(v_coast); k_aero=q*S_REF*8.0*(sm_cal*D_REF)   # >0 stable
    maxabs=0.0; sat=0; n=0; Hmax_used=0.0
    for k in range(int(4.0/DT)):                # 4 s of coast
        wind=0.95*wind+0.32*wind_amp*rng.standard_normal()
        M_wind=q*S_REF*8.0*(D_REF*0.5)*wind
        # PID -> desired reaction torque
        tau_des=-(kp*theta+kd*q_rate)*IYY
        tau=np.clip(tau_des,-tmax,tmax)
        # wheel accel = -tau/Iw ; check saturation (can't exceed Hmax in that direction)
        w_new=w_wheel + (-tau/Iw)*DT
        if abs(w_new)>wmax:                     # saturated -> wheel can't absorb more
            w_new=np.clip(w_new,-wmax,wmax); tau=-Iw*(w_new-w_wheel)/DT; sat+=1
        w_wheel=w_new
        Hmax_used=max(Hmax_used, abs(Iw*w_wheel))
        # rocket dynamics
        Mtot=-k_aero*theta + tau + M_wind
        q_rate+=(Mtot/IYY)*DT; theta+=q_rate*DT
        maxabs=max(maxabs,abs(np.rad2deg(theta))); n+=1
    return dict(max_dev=round(maxabs,1), sat_frac=round(sat/n,2),
                H_used_frac=round(Hmax_used/Hmax,2), success=maxabs<20.0)

def sim_slew(wheel, target_deg=20.0, v_coast=15.0):
    """Command a 20-deg pointing slew in coast; does the wheel have the momentum?"""
    Iw=wheel['I_w']; tmax=wheel['tau_max']; wmax=wheel['omega_max']; Hmax=Iw*wmax
    theta=0.0; q_rate=0.0; w_wheel=0.0; tgt=np.deg2rad(target_deg)
    q=coast_q(v_coast); k_aero=q*S_REF*8.0*(1.0*D_REF)
    reached=False; t_reach=None; saturated=False
    for k in range(int(4.0/DT)):
        e=tgt-theta
        tau_des=-(0.8*(theta-tgt)+0.3*q_rate)*IYY
        tau=np.clip(tau_des,-tmax,tmax)
        w_new=w_wheel+(-tau/Iw)*DT
        if abs(w_new)>wmax: w_new=np.clip(w_new,-wmax,wmax); tau=-Iw*(w_new-w_wheel)/DT; saturated=True
        w_wheel=w_new
        Mtot=-k_aero*theta+tau; q_rate+=(Mtot/IYY)*DT; theta+=q_rate*DT
        if not reached and abs(np.rad2deg(theta)-target_deg)<2.0: reached=True; t_reach=k*DT
    return dict(reached=reached, t_reach=t_reach, saturated=saturated,
                H_budget_slews=round(Hmax/(IYY*1.0),1))   # how many 1 rad/s slews the wheel can store

def main():
    print("=== REACTION-WHEEL FEASIBILITY (model rocket, COAST phase) ===")
    print(f"  Iyy={IYY} kg m^2.  Reaction wheel works airspeed-independent (unlike TVC/moving-mass).\n")
    for name,w in WHEELS.items():
        Hmax=w['I_w']*w['omega_max']; auth=w['tau_max']/IYY
        print(f"  {name}")
        print(f"     torque authority = {auth:.2f} rad/s^2   |   momentum budget H_max = {Hmax:.3f} N m s "
              f"(= {Hmax/(IYY*1.0):.1f} full 1-rad/s rocket slews)")
        # coast hold
        res=[sim_hold(w,seed=s) for s in range(8)]
        print(f"     COAST HOLD vs gusts: success={np.mean([r['success'] for r in res]):.2f}  "
              f"max_dev={np.mean([r['max_dev'] for r in res]):.1f}deg  wheel_sat_frac={np.mean([r['sat_frac'] for r in res]):.2f}  "
              f"H_used={np.mean([r['H_used_frac'] for r in res]):.2f}")
        # slew
        sl=sim_slew(w)
        print(f"     COAST SLEW 20deg: reached={sl['reached']} t={sl['t_reach']}s saturated={sl['saturated']}\n")
    print("VERDICT: reaction wheel has airspeed-independent authority (works in coast), but")
    print("(1) torque authority is modest (motor-limited) and (2) momentum SATURATES under")
    print("sustained load. Feasible for GENTLE coast attitude hold/pointing; not for aggressive control.")

if __name__=='__main__': main()
