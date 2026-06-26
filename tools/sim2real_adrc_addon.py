"""
tools/sim2real_adrc_addon.py  (2026-06-25)

Stress-test add-on to sim2real_pi_transfer: does ADRC (anticipatory / observer-based,
b0 from specs, NOT sim-optimized) transfer well across Pi, unlike the reactive PID/LQR?

If ADRC's transfer gap stays small and flat vs Pi while PID/LQR gaps rise, the story is:
  reactive controllers tuned in an idealized sim over-fit the missing actuator limits and
  fail to transfer above Pi_crit; an anticipatory controller that does not over-fit transfers.
This ties the sim-to-real transfer finding to the project's reactive-vs-anticipatory spine.
"""
import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')
import numpy as np, pandas as pd
from pathlib import Path
from joblib import Parallel, delayed
from scipy import stats
from design_space import (build_plant, build_actuator, build_sensor,
                          build_disturbance, build_scenario)
from controller import PIDParams, ADRCParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU = np.pi/180*REF_MAX_GIMBAL_DEG/REF_U_MAX; L = 0.25
ROOT = Path(__file__).resolve().parents[1]; RES = ROOT/'experiments'/'results'
def keff_of(r): return F15_AVG_THRUST_N*r['motor_scale']*CU*L/r['Iyy']

NAIVE = dict(wind=True, sensor_noise=True, slew=False, backlash=True, latency=False,
             thrust_var=False, deadband=True, nonlinear_aero=True, dyn_aero=True,
             thrust_curve=True, cg_shift=True)
REAL  = dict(wind=True, sensor_noise=True, slew=True,  backlash=True, latency=True,
             thrust_var=False, deadband=True, nonlinear_aero=True, dyn_aero=True,
             thrust_curve=True, cg_shift=True)
OMEGA_C, OMEGA0 = 5.0, 25.0

def _sr_adrc(row, fid_kw, seeds):
    plant=build_plant(row); act=build_actuator(row); sen=build_sensor(row)
    dis=build_disturbance(row); sc=build_scenario(theta0_bias_std=0.0); dis.gust_std=0.25
    act,sen,dis,sc=apply_fidelity_config(act,sen,dis,sc,FidelityConfig(**fid_kw))
    adrc=ADRCParams(omega_c=OMEGA_C, omega0=OMEGA0, b0=keff_of(row))
    dummy=PIDParams(Kp=0.0,Kd=0.0,Ki=0.0,u_max=act.u_max,i_lim=act.u_max)
    return float(np.mean([simulate(dummy,plant,act,sen,dis,sc,seed=s,adrc=adrc).success for s in seeds]))

def process(r):
    eval_seeds=list(range(140101,140111))
    n=_sr_adrc(r,NAIVE,eval_seeds); rl=_sr_adrc(r,REAL,eval_seeds)
    return dict(rocket_id=r.get('rocket_id','?'), adrc_naive=round(n,3),
                adrc_real=round(rl,3), adrc_gap=round(n-rl,3))

def main():
    base=pd.read_csv(RES/'sim2real_pi_transfer_py.csv')   # the 42 designs already run
    pop=pd.read_csv(RES/'exp1_final_population_py.csv')
    designs=pop[pop['rocket_id'].isin(base['rocket_id'])].copy()
    print(f"ADRC add-on on the same {len(designs)} designs")
    rows=Parallel(n_jobs=-1,verbose=5)(delayed(process)(r.to_dict()) for _,r in designs.iterrows())
    adf=pd.DataFrame(rows)
    df=base.merge(adf,on='rocket_id',how='left').sort_values('Pi').reset_index(drop=True)
    df.to_csv(RES/'sim2real_pi_transfer_py.csv',index=False)
    analyze(df)

def analyze(df=None):
    if df is None: df=pd.read_csv(RES/'sim2real_pi_transfer_py.csv')
    print(f"\n{'='*72}\nsim-to-real transfer: reactive (PID,LQR) vs anticipatory (ADRC)  n={len(df)}\n{'='*72}")
    lp=np.log(df.Pi.clip(1))
    for c in ['pid','lqr','adrc']:
        g=df[f'{c}_gap']; m=np.isfinite(g)
        rho,p=stats.spearmanr(lp[m],g[m])
        print(f"  {c.upper():4}: rho(logPi,gap)={rho:+.3f} p={p:.1e}  mean_real={df[f'{c}_real'].mean():.2f}  mean_gap={g[m].mean():.3f}")
    df['b']=pd.cut(df.Pi,[0,120,300,1300],labels=['Pi<120','120-300','>300'])
    print("\nMean REAL-physics SR by Pi tier (higher=transfers better):")
    print(df.groupby('b',observed=True)[['pid_real','lqr_real','adrc_real']].mean().round(3).to_string())
    print("\nMean transfer GAP (SR_naive - SR_real) by Pi tier (lower=transfers better):")
    print(df.groupby('b',observed=True)[['pid_gap','lqr_gap','adrc_gap']].mean().round(3).to_string())

if __name__=='__main__':
    analyze() if '--analyze' in sys.argv else main()
