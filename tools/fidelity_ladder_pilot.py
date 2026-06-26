"""
tools/fidelity_ladder_pilot.py  (2026-06-25)

SCOPING PILOT for a possible derivative paper: "minimum simulator fidelity required
to make the correct TVC gain decision, as a function of Pi."

Question: if you tune a gain in a CHEAP simulator (fewer physics modules) and fly it in
FULL physics, at what Pi does each missing module start producing a wrong gain?

Method: for each design, tune Kp at each rung of a fidelity ladder (cheapest -> fullest),
then evaluate that tuned gain in FULL physics. The cheapest rung whose tuned gain still
succeeds in full physics (SR >= 0.80) = the minimum required fidelity for that design.

Ladder (cumulative):
  L0 linear      : no disturbances, no nonlinearities (ideal double-integrator + thrust)
  L1 +latency    : add control-loop delay
  L2 +slew       : add servo slew-rate saturation (the key nonlinearity)
  L3 +wind       : add wind disturbance
  L4 +aero       : add aerodynamic coupling (nonlinear_aero + dyn_aero)
  L5 full        : everything (noise, backlash, deadband, thrust_curve, cg_shift)
Ground truth = L5.

This is a PILOT (6 designs spanning Pi) to preview whether min-required-fidelity rises
with Pi cleanly enough to justify a full run.
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')
import numpy as np, pandas as pd
from pathlib import Path
from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU = np.pi/180*REF_MAX_GIMBAL_DEG/REF_U_MAX; L_NOZ = 0.25
ROOT = Path(__file__).resolve().parents[1]; RES = ROOT/'experiments'/'results'

def keff_of(r): return F15_AVG_THRUST_N*r['motor_scale']*CU*L_NOZ/r['Iyy']

# fidelity rungs (cumulative)
LADDER = [
 ('L0_linear', dict(wind=False,sensor_noise=False,slew=False,backlash=False,latency=False,thrust_var=False,deadband=False,nonlinear_aero=False,dyn_aero=False,thrust_curve=False,cg_shift=False)),
 ('L1_+latency', dict(wind=False,sensor_noise=False,slew=False,backlash=False,latency=True, thrust_var=False,deadband=False,nonlinear_aero=False,dyn_aero=False,thrust_curve=False,cg_shift=False)),
 ('L2_+slew',   dict(wind=False,sensor_noise=False,slew=True, backlash=False,latency=True, thrust_var=False,deadband=False,nonlinear_aero=False,dyn_aero=False,thrust_curve=False,cg_shift=False)),
 ('L3_+wind',   dict(wind=True, sensor_noise=False,slew=True, backlash=False,latency=True, thrust_var=False,deadband=False,nonlinear_aero=False,dyn_aero=False,thrust_curve=False,cg_shift=False)),
 ('L4_+aero',   dict(wind=True, sensor_noise=False,slew=True, backlash=False,latency=True, thrust_var=False,deadband=False,nonlinear_aero=True, dyn_aero=True, thrust_curve=False,cg_shift=False)),
 ('L5_full',    dict(wind=True, sensor_noise=True, slew=True, backlash=True, latency=True, thrust_var=False,deadband=True, nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True)),
]
FULL = LADDER[-1][1]
KP_GRID = np.geomspace(3, 320, 9); KD_SET = [0.57, 2.0]

def build(r, fid_kw, wind=0.25):
    plant=build_plant(r); act=build_actuator(r); sen=build_sensor(r)
    dis=build_disturbance(r); sc=build_scenario(theta0_bias_std=0.0)
    dis.gust_std=wind
    fid=FidelityConfig(**fid_kw)
    return apply_fidelity_config(act,sen,dis,sc,fid)+(plant,)

def sr_at(r, Kp, Kd, fid_kw, seeds):
    act,sen,dis,sc,plant=build(r,fid_kw)
    pid=PIDParams(Kp=float(Kp),Kd=float(Kd),Ki=0.0,u_max=act.u_max,i_lim=act.u_max)
    return float(np.mean([simulate(pid,plant,act,sen,dis,sc,seed=s).success for s in seeds]))

def tune(r, fid_kw, seeds=(1,2,3)):
    best=(-1,None,None)
    for Kp in KP_GRID:
        for Kd in KD_SET:
            sr=sr_at(r,Kp,Kd,fid_kw,seeds)
            if sr>best[0]: best=(sr,Kp,Kd)
    return best[1],best[2]

def main():
    pop=pd.read_csv(RES/'exp1_final_population_py.csv')
    pop['keff']=pop.apply(keff_of,axis=1); pop['Pi']=pop['keff']*pop['latency_steps']**2
    # pick ~6 designs spanning Pi
    targets=[40,110,210,400,800,1150]; picks=[]
    for t in targets:
        i=(pop['Pi']-t).abs().idxmin(); picks.append(pop.loc[i])
    eval_seeds=list(range(120001,120011))
    rows=[]
    print(f"{'design':8} {'Pi':>6} {'lat':>3} | min-required-fidelity (cheapest rung whose tuned gain survives FULL)")
    print('-'*92)
    for r in picks:
        rid=r.get('rocket_id','?'); Pi=r['Pi']; lat=int(r['latency_steps'])
        srs={}; min_rung=None
        for name,fk in LADDER:
            Kp,Kd=tune(r,fk)
            sr_full=sr_at(r,Kp,Kd,FULL,eval_seeds)
            srs[name]=(round(sr_full,2),round(Kp,1))
            if min_rung is None and sr_full>=0.80: min_rung=name
        rows.append(dict(rocket_id=rid,Pi=round(Pi,0),lat=lat,min_fidelity=min_rung,
                         **{k:v[0] for k,v in srs.items()}))
        ladder_str=' '.join('%s=%.2f'%(n.split('_')[0],srs[n][0]) for n,_ in LADDER)
        print("%-8s %6.0f %3d | min=%-11s | %s"%(rid,Pi,lat,str(min_rung),ladder_str))
    df=pd.DataFrame(rows); df.to_csv(RES/'fidelity_ladder_pilot_py.csv',index=False)
    print('\nsaved -> fidelity_ladder_pilot_py.csv')
    print('\nINTERPRETATION: min_fidelity should climb the ladder as Pi rises if the claim holds')
    print('(low Pi: L0/L1 linear sim already picks a working gain; high Pi: need +slew/+wind/+aero).')

if __name__=='__main__': main()
