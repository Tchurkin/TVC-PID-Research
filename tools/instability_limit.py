"""
tools/instability_limit.py  (2026-06-25)

STS-DIRECTION FEASIBILITY PROBE (not a novelty claim): the most visceral hardware demo
is "fly a rocket that is aerodynamically UNSTABLE and would tumble instantly without
active control." Question: is there a PREDICTABLE instability limit (how negative can
static margin go and still be stabilized by TVC) that scales with control authority, and
does ADRC extend it beyond PID? If yes, the demo is "predict the limit, then fly to it"
(a Kevin-Shen-style predicted-and-measured result), with a benefit (active control
extends the flyable design envelope).

static_margin > 0 = stable (CG ahead of CP); < 0 = UNSTABLE (diverges without control).
Sweep static_margin from +0.2 to -0.6 for several authority levels; PID (best-of-grid)
and ADRC; measure attitude-hold SR (full physics). Find most-negative SM with SR>=0.80.
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')
import numpy as np, pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams, ADRCParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU = np.pi/180*REF_MAX_GIMBAL_DEG/REF_U_MAX; L = 0.25
ROOT = Path(__file__).resolve().parents[1]; RES = ROOT/'experiments'/'results'
FULL = dict(wind=True, sensor_noise=True, slew=True, backlash=True, latency=True,
            thrust_var=False, deadband=True, nonlinear_aero=True, dyn_aero=True,
            thrust_curve=True, cg_shift=True)
SM_SWEEP = [0.20, 0.10, 0.0, -0.10, -0.20, -0.30, -0.40, -0.50, -0.60]
KP_GRID = np.geomspace(3, 200, 10)
EVAL_SEEDS = list(range(150001, 150011))

def keff_of(r): return F15_AVG_THRUST_N*r['motor_scale']*CU*L/r['Iyy']

def _build(row, sm):
    r = dict(row); r['static_margin'] = float(sm)
    plant=build_plant(r); act=build_actuator(r); sen=build_sensor(r)
    dis=build_disturbance(r); sc=build_scenario(theta0_bias_std=0.0); dis.gust_std=0.25
    return apply_fidelity_config(act,sen,dis,sc,FidelityConfig(**FULL))+(plant,)

def sr_pid(row, sm, Kp, Kd, seeds):
    act,sen,dis,sc,plant=_build(row,sm)
    pid=PIDParams(Kp=float(Kp),Kd=float(Kd),Ki=0.0,u_max=act.u_max,i_lim=act.u_max)
    return float(np.mean([simulate(pid,plant,act,sen,dis,sc,seed=s).success for s in seeds]))

def sr_adrc(row, sm, keff, seeds, wc=5.0):
    act,sen,dis,sc,plant=_build(row,sm)
    adrc=ADRCParams(omega_c=wc, omega0=5*wc, b0=keff, u_max=act.u_max)
    dummy=PIDParams(Kp=0,Kd=0,Ki=0,u_max=act.u_max,i_lim=act.u_max)
    return float(np.mean([simulate(dummy,plant,act,sen,dis,sc,seed=s,adrc=adrc).success for s in seeds]))

def process(row):
    keff=keff_of(row); lat=int(row['latency_steps']); rid=row.get('rocket_id','?')
    out=[]
    for sm in SM_SWEEP:
        # PID best of grid
        best=-1.0
        for Kp in KP_GRID:
            s=sr_pid(row,sm,Kp,0.57,EVAL_SEEDS[:5])
            if s>best: best=s
        pid_sr=best
        adrc_sr=sr_adrc(row,sm,keff,EVAL_SEEDS)
        out.append((sm,round(pid_sr,2),round(adrc_sr,2)))
    # find limits (most-negative SM with SR>=0.8)
    pid_lim=min([sm for sm,p,a in out if p>=0.8], default=0.5)
    adrc_lim=min([sm for sm,p,a in out if a>=0.8], default=0.5)
    return dict(rocket_id=rid, keff=round(keff,1), lat=lat,
                pid_limit_SM=pid_lim, adrc_limit_SM=adrc_lim,
                sweep={sm:(p,a) for sm,p,a in out})

def main():
    pop=pd.read_csv(RES/'exp1_final_population_py.csv'); pop['keff']=pop.apply(keff_of,axis=1)
    # 3 authority levels, low latency (so authority, not delay, sets the limit)
    picks=[]
    for klo,khi in [(4,7),(12,18),(25,40)]:
        sub=pop[(pop.keff>=klo)&(pop.keff<khi)&(pop.latency_steps<=2)]
        if len(sub): picks.append(sub.sample(1,random_state=4).iloc[0])
    print(f"instability limit probe: {len(picks)} authority levels, static_margin {SM_SWEEP[0]}..{SM_SWEEP[-1]}")
    rows=Parallel(n_jobs=-1,verbose=5)(delayed(process)(r.to_dict()) for r in picks)
    print("\n%-7s %6s | %-12s %-12s | sweep (SM: PID/ADRC SR)" % ('id','keff','PID_limit_SM','ADRC_limit_SM'))
    print('-'*100)
    for r in rows:
        sw=' '.join('%.2f:%.1f/%.1f'%(sm,p,a) for sm,(p,a) in r['sweep'].items())
        print('%-7s %6.1f | %-12.2f %-12.2f | %s'%(r['rocket_id'],r['keff'],r['pid_limit_SM'],r['adrc_limit_SM'],sw))
    print("\nINTERPRETATION: more-negative limit = can stabilize a MORE unstable rocket.")
    print("If limit gets more negative with keff (authority) -> predictable instability envelope.")
    print("If ADRC_limit < PID_limit (more negative) -> ADRC extends the flyable instability envelope.")
    pd.DataFrame([{k:v for k,v in r.items() if k!='sweep'} for r in rows]).to_csv(
        RES/'instability_limit_py.csv', index=False)

if __name__=='__main__': main()
