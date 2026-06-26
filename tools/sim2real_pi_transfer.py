"""
tools/sim2real_pi_transfer.py  (2026-06-25)

NOVELTY PROBE (sprint): Is Π = keff×τ² a controller-AGNOSTIC predictor of the
sim-to-real transfer gap for actuator-rate-limited systems?

Motivation: the RL sim-to-real literature identifies two dominant transfer-killers —
(1) actuator/slew saturation (policies learn bang-bang in idealized sim, fail in reality)
and (2) actuator delay. That is EXACTLY Π = keff×τ² (authority/saturation × delay).
The field characterizes this empirically (domain randomization); it lacks a spec-computable
a-priori predictor. The project's pi_s2r showed it for PID GAIN CALIBRATION. Here we test
whether the SAME Π-driven transfer failure appears for a controller designed by a completely
different principle (LQR / optimal control), which would show the failure is a property of
Π, not of any one tuning method.

PROTOCOL: for designs spanning Π:
  NAIVE sim = full physics EXCEPT slew limit and latency idealized away (slew_max=inf, latency=0)
              — the common "I modeled the rocket but assumed an ideal fast servo + no loop delay".
  REAL sim  = full physics (slew + latency on).
  Two controllers, tuned/designed using the NAIVE model only, then DEPLOYED in REAL:
    PID : grid Kp×Kd, pick best NAIVE SR.
    LQR : DARE gains from the idealized linear plant (no slew/latency by construction);
          Q/R swept, pick best NAIVE SR.
  Transfer gap = SR_naive − SR_real.  Question: does it collapse onto Π for BOTH?
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')
import numpy as np, pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats
from scipy.linalg import solve_discrete_are
from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU = np.pi/180*REF_MAX_GIMBAL_DEG/REF_U_MAX; L = 0.25; DT = 0.005
ROOT = Path(__file__).resolve().parents[1]; RES = ROOT/'experiments'/'results'

def keff_of(r): return F15_AVG_THRUST_N*r['motor_scale']*CU*L/r['Iyy']

NAIVE = dict(wind=True, sensor_noise=True, slew=False, backlash=True, latency=False,
             thrust_var=False, deadband=True, nonlinear_aero=True, dyn_aero=True,
             thrust_curve=True, cg_shift=True)   # idealized actuator: no slew, no delay
REAL  = dict(wind=True, sensor_noise=True, slew=True,  backlash=True, latency=True,
             thrust_var=False, deadband=True, nonlinear_aero=True, dyn_aero=True,
             thrust_curve=True, cg_shift=True)

KP_GRID = np.geomspace(2, 320, 12); KD_GRID = [0.57, 2.0]
QR_GRID = np.geomspace(1e-2, 1e4, 10)

def _sr(row, Kp, Kd, fid_kw, seeds):
    plant=build_plant(row); act=build_actuator(row); sen=build_sensor(row)
    dis=build_disturbance(row); sc=build_scenario(theta0_bias_std=0.0); dis.gust_std=0.25
    act,sen,dis,sc=apply_fidelity_config(act,sen,dis,sc,FidelityConfig(**fid_kw))
    pid=PIDParams(Kp=float(Kp),Kd=float(Kd),Ki=0.0,u_max=act.u_max,i_lim=act.u_max)
    return float(np.mean([simulate(pid,plant,act,sen,dis,sc,seed=s).success for s in seeds]))

def lqr_gains(keff, qr):
    A=np.array([[1.0,DT],[0.0,1.0]]); B=np.array([[0.5*keff*DT**2],[keff*DT]])
    Q=np.diag([qr,1.0]); R=np.array([[1.0]])
    try:
        P=solve_discrete_are(A,B,Q,R); K=np.linalg.inv(R+B.T@P@B)@(B.T@P@A)
        return float(K[0,0]), float(K[0,1])
    except Exception:
        return None,None

def process(r):
    keff=keff_of(r); lat=int(r['latency_steps']); Pi=keff*lat**2
    tune_seeds=list(range(140001,140006)); eval_seeds=list(range(140101,140111))
    # PID: grid, best in NAIVE
    best=(-1,None,None)
    for Kp in KP_GRID:
        for Kd in KD_GRID:
            s=_sr(r,Kp,Kd,NAIVE,tune_seeds)
            if s>best[0]: best=(s,Kp,Kd)
    pid_naive=_sr(r,best[1],best[2],NAIVE,eval_seeds)
    pid_real =_sr(r,best[1],best[2],REAL, eval_seeds)
    # LQR: DARE gains (idealized linear plant), Q/R picked by best NAIVE SR
    lbest=(-1,None,None)
    for qr in QR_GRID:
        kp,kd=lqr_gains(keff,qr)
        if kp is None or kp<=0: continue
        s=_sr(r,kp,kd,NAIVE,tune_seeds)
        if s>lbest[0]: lbest=(s,kp,kd)
    if lbest[1] is None: lqr_naive=lqr_real=np.nan; lqr_kp=np.nan
    else:
        lqr_naive=_sr(r,lbest[1],lbest[2],NAIVE,eval_seeds)
        lqr_real =_sr(r,lbest[1],lbest[2],REAL, eval_seeds)
        lqr_kp=lbest[1]
    return dict(rocket_id=r.get('rocket_id','?'), keff=round(keff,2), lat=lat, Pi=round(Pi,0),
                pid_kp=round(best[1],1), pid_naive=round(pid_naive,3), pid_real=round(pid_real,3),
                pid_gap=round(pid_naive-pid_real,3),
                lqr_kp=round(lqr_kp,1) if lqr_kp==lqr_kp else np.nan,
                lqr_naive=round(lqr_naive,3), lqr_real=round(lqr_real,3),
                lqr_gap=round(lqr_naive-lqr_real,3))

def main():
    pop=pd.read_csv(RES/'exp1_final_population_py.csv'); pop['keff']=pop.apply(keff_of,axis=1)
    pop['Pi']=pop['keff']*pop['latency_steps']**2
    # ~40 designs stratified across Pi
    picks=[]
    for lo,hi in [(0,60),(60,120),(120,200),(200,300),(300,500),(500,1300)]:
        sub=pop[(pop.Pi>=lo)&(pop.Pi<hi)]
        if len(sub): picks.append(sub.sample(min(7,len(sub)),random_state=5))
    designs=pd.concat(picks).reset_index(drop=True)
    print(f"sim2real Pi transfer: {len(designs)} designs, NAIVE(no slew/latency)->REAL(full)")
    rows=Parallel(n_jobs=-1,verbose=5)(delayed(process)(r.to_dict()) for _,r in designs.iterrows())
    df=pd.DataFrame(rows).sort_values('Pi').reset_index(drop=True)
    df.to_csv(RES/'sim2real_pi_transfer_py.csv',index=False)
    analyze(df)

def analyze(df=None):
    if df is None: df=pd.read_csv(RES/'sim2real_pi_transfer_py.csv')
    print(f"\n{'='*70}\nsim-to-real transfer gap vs Pi  (n={len(df)})\n{'='*70}")
    lp=np.log(df.Pi.clip(1))
    for c in ['pid','lqr']:
        g=df[f'{c}_gap']; m=np.isfinite(g)
        rho,p=stats.spearmanr(lp[m],g[m])
        print(f"  {c.upper():4}: rho(logPi, transfer_gap)={rho:+.3f} p={p:.1e}  "
              f"mean_naive={df[f'{c}_naive'].mean():.2f} mean_real={df[f'{c}_real'].mean():.2f}")
    print("\nBinned mean transfer gap (SR_naive - SR_real):")
    df['b']=pd.cut(df.Pi,[0,120,300,1300],labels=['Pi<120','120-300','>300'])
    print(df.groupby('b',observed=True)[['pid_gap','lqr_gap','pid_real','lqr_real']].mean().round(3).to_string())
    print("\nKEY: if BOTH controllers' real-SR drops with Pi (gap rises), the sim-to-real")
    print("transfer failure is Pi-driven and controller-agnostic, not a PID-tuning artifact.")

if __name__=='__main__':
    analyze() if '--analyze' in sys.argv else main()
