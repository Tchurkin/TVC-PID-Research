"""
tools/maneuver_regime_pilot.py  (2026-06-25)

DOMAIN-KNOWLEDGE PROBE (not a novelty claim): the whole project is attitude-HOLD
(theta_ref=0), where saturation is DISTURBANCE-driven and governed by Pi=keff*tau^2.
Real guided TVC rockets MANEUVER (pitch-over). There, saturation is COMMAND-driven.

QUESTION: is maneuver capability governed by the SAME Pi, or by a DISTINCT parameter
(authority theta_ddot_max and servo slew, i.e. agility) -- which would mean Pi is only
the HOLD parameter and the project's headline is incomplete for guided flight?

DESIGN: pick designs spanning keff (authority) x latency so keff and Pi DIVERGE
(high keff + low latency = high agility but low Pi). For each design, with one sensible
latency-adapted gain (ZN: Kp=228/lat, Kd=0.57), run:
  HOLD task     : theta_ref=0, wind on   -> hold_sr  (Pi-governed, known)
  MANEUVER task : step to 30 deg at t=1s  -> man_sr, rise_time, overshoot
Then ask what predicts maneuver capability: Pi? theta_ddot_max? servo_slew? latency?

If a design that is BAD at holding (high Pi) is GOOD at maneuvering (high agility), the
hold/maneuver constraints are DISTINCT -> a real, testable domain finding (a tradeoff,
and a separate maneuver design rule the domain lacks). If they track together, no finding.
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')
import numpy as np, pandas as pd
from pathlib import Path
from scipy import stats
from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU = np.pi/180*REF_MAX_GIMBAL_DEG/REF_U_MAX; L_NOZ = 0.25
ROOT = Path(__file__).resolve().parents[1]; RES = ROOT/'experiments'/'results'
FULL = dict(wind=True,sensor_noise=True,slew=True,backlash=True,latency=True,
            thrust_var=False,deadband=True,nonlinear_aero=True,dyn_aero=True,
            thrust_curve=True,cg_shift=True)
STEP_DEG = 30.0

def keff_of(r): return F15_AVG_THRUST_N*r['motor_scale']*CU*L_NOZ/r['Iyy']
def umax_of(r): return r['max_gimbal_deg']*12.0/15.0   # CU
def thddot_of(r): return keff_of(r)*umax_of(r)          # rad/s^2 (theta_ddot_max)

def build(r, step_deg, wind=0.25):
    plant=build_plant(r); act=build_actuator(r); sen=build_sensor(r)
    dis=build_disturbance(r); sc=build_scenario(theta0_bias_std=0.0)
    dis.gust_std=wind
    sc.theta_step_deg=float(step_deg); sc.theta_step_time_s=1.0; sc.t_end=4.0
    act,sen,dis,sc=apply_fidelity_config(act,sen,dis,sc,FidelityConfig(**FULL))
    return plant,act,sen,dis,sc

def run(r, Kp, Kd, step_deg, seeds):
    plant,act,sen,dis,sc=build(r,step_deg)
    pid=PIDParams(Kp=float(Kp),Kd=float(Kd),Ki=0.0,u_max=act.u_max,i_lim=act.u_max)
    res=[simulate(pid,plant,act,sen,dis,sc,seed=s) for s in seeds]
    sr=float(np.mean([x.success for x in res]))
    rise=float(np.median([getattr(x,'rise_time',np.nan) for x in res]))
    maxth=float(np.median([x.max_theta_deg for x in res]))
    return sr, rise, maxth

def main():
    pop=pd.read_csv(RES/'exp1_final_population_py.csv')
    pop['keff']=pop.apply(keff_of,axis=1)
    pop['thddot']=pop.apply(thddot_of,axis=1)
    pop['Pi']=pop['keff']*pop['latency_steps']**2
    # pick designs spanning keff x latency so keff and Pi DIVERGE
    picks=[]
    for klo,khi in [(3,7),(10,16),(22,40)]:          # low / mid / high authority
        for llo,lhi in [(1,2),(3,4),(5,6)]:          # low / mid / high latency
            sub=pop[(pop.keff>=klo)&(pop.keff<khi)&(pop.latency_steps>=llo)&(pop.latency_steps<=lhi)]
            if len(sub): picks.append(sub.sample(1,random_state=3).iloc[0])
    seeds=list(range(130001,130009))
    rows=[]
    print(f"{'id':7} {'keff':>5} {'lat':>3} {'thddot':>7} {'Pi':>6} | {'hold_sr':>7} | {'man_sr':>6} {'rise_s':>6} {'maxdeg':>6}")
    print('-'*78)
    for r in picks:
        lat=int(r['latency_steps']); Kp=228.0/lat; Kd=0.57
        hold_sr,_,_ = run(r, Kp, Kd, 0.0, seeds)            # HOLD
        man_sr,rise,maxth = run(r, Kp, Kd, STEP_DEG, seeds) # MANEUVER (step 30 deg)
        rows.append(dict(rocket_id=r.get('rocket_id','?'),keff=round(r['keff'],1),lat=lat,
                         thddot=round(r['thddot'],1),Pi=round(r['Pi'],0),
                         hold_sr=round(hold_sr,2),man_sr=round(man_sr,2),
                         rise_s=round(rise,2),max_deg=round(maxth,1)))
        print(f"{r.get('rocket_id','?'):7} {r['keff']:5.1f} {lat:>3} {r['thddot']:7.1f} {r['Pi']:6.0f} | "
              f"{hold_sr:7.2f} | {man_sr:6.2f} {rise:6.2f} {maxth:6.1f}")
    df=pd.DataFrame(rows); df.to_csv(RES/'maneuver_regime_pilot_py.csv',index=False)
    print('\nsaved -> maneuver_regime_pilot_py.csv')
    # what predicts each capability?
    def sp(x,y):
        x=np.asarray(x,float); y=np.asarray(y,float); m=np.isfinite(x)&np.isfinite(y)
        return stats.spearmanr(x[m],y[m])[0] if m.sum()>3 else np.nan
    lp=np.log(df.Pi.clip(1)); lt=np.log(df.thddot.clip(1)); ll=np.log(df.lat)
    print('\n=== what predicts HOLD success ===')
    print(f"  rho(logPi, hold_sr)    = {sp(lp,df.hold_sr):+.2f}   rho(log_thddot,hold)={sp(lt,df.hold_sr):+.2f}")
    print('=== what predicts MANEUVER success ===')
    print(f"  rho(logPi, man_sr)     = {sp(lp,df.man_sr):+.2f}   rho(log_thddot,man) ={sp(lt,df.man_sr):+.2f}   rho(log_lat,man)={sp(ll,df.man_sr):+.2f}")
    print('=== what predicts MANEUVER speed (rise time; lower=faster) ===')
    print(f"  rho(log_thddot, rise)  = {sp(lt,df.rise_s):+.2f}   rho(logPi,rise)={sp(lp,df.rise_s):+.2f}")
    print('\nKEY: if hold tracks Pi (neg) but maneuver tracks thddot (pos) and NOT Pi,')
    print('then hold and maneuver are DISTINCT regimes -> Pi is only the hold parameter.')

if __name__=='__main__': main()
