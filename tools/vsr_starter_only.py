"""
tools/vsr_starter_only.py  (2026-06-27)

Tests the user's actual architecture (which the earlier 'complement' test got wrong):
  margin modulation is the PRIMARY controller -- it does the fling (deploy>0.5, unstable),
  the catch (deploy<0.5, stable), and the hold (deploy~0.5, neutral). The supplemental
  actuator (TVC) only provides a brief STARTER kick to create the initial angle of attack at
  alpha~0 (where margin is singular), then gets out of the way.

Question: how SMALL can the starter authority be while the maneuver still succeeds? If it can
shrink toward ~0, the user's claim "supplement authority -> 0" holds.

Supplement modes:
  starter  : TVC active ONLY in [man_t, man_t+0.2] (the kick), then OFF; margin does fling+catch+hold
  backstop : TVC active whenever |alpha|<floor (starter AND fine-hold)         [for comparison]
  none     : no supplement at all                                              [does margin self-start?]

Also sweeps the realistic post-stall model ('drop') vs gentle ('plateau').
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from sim_vsr import VehicleParams, CanardParams, TVCParams, TVCActuator, CanardActParams, CanardActuator, IMUParams, IMU, State, deriv
from sim_vsr.vehicle import mass_props
from sim_vsr.scenario import thrust_F_class, WindOU

KP, KD = 36.0, 8.0


def run(supp_mode, u_max, man=35.0, ramp=0.08, stall_deg=None, stall_model="plateau", seed=0,
        t_end=2.4, dt=0.005):
    vp = VehicleParams(stall_deg=stall_deg, stall_model=stall_model)
    can = CanardParams(S_c=0.0028, CN_c=3.0, arm_c=0.28, defl_max_deg=20.0)
    DM = can.dmargin_full(vp); SM_AFT = 0.5 * DM
    rng = np.random.default_rng(seed); man_t = 0.8; target = np.pi / 2 - np.deg2rad(man)
    tvc = TVCActuator(TVCParams(u_max_deg=max(u_max, 1e-3), slew_max_deg_s=400, deadband_deg=0.1, backlash_deg=0.2))
    canact = CanardActuator(CanardActParams(defl_max_deg=can.defl_max_deg, deploy_rate_s=8.0, defl_slew_deg_s=500))
    imu = IMU(IMUParams(latency_steps=2), rng); wind = WindOU(1.5, 0.4, rng)
    s = State(V=22.0); n = int(t_end / dt)
    total = max(sum(thrust_F_class(k * dt) for k in range(n)) * dt, 1e-6); cum = 0.0
    af = np.deg2rad(3.0); i_t = 0.0; th_h = []; post = 0; supp_eff = 0.0
    for k in range(n):
        t = k * dt; thrust = thrust_F_class(t); pf = max(0.0, 1.0 - cum / total); cum += thrust * dt
        m, Iyy = mass_props(vp, pf); q = 0.5 * vp.rho * s.V * s.V
        w = wind.step(dt) if t > man_t else 0.0; wa = np.arctan2(w, max(s.V, 2.0))
        th_m, q_m = imu.measure(s.theta, s.q)
        ref = (np.pi / 2) if t < man_t else (np.pi / 2 - np.deg2rad(man) * min(1.0, (t - man_t) / ramp))
        e = ref - th_m; ae = (s.theta - s.gamma) + np.deg2rad(0.5) * rng.standard_normal()
        M_des = Iyy * (KP * e + KD * (-q_m))

        # margin (PRIMARY): two-sided FL via deploy
        deploy = 0.5
        per = q * can.S_c * can.CN_c * ae * can.arm_c
        if abs(ae) > af and abs(per) > 1e-6:
            deploy = float(np.clip(0.5 + M_des / per, 0, 1))

        # supplement (TVC): starter / backstop / none
        gim_cmd = 0.0
        supp_on = ((supp_mode == "starter" and man_t <= t < man_t + 0.2) or
                   (supp_mode == "backstop" and abs(ae) <= af))
        if supp_on and thrust > 1.0:
            gim_cmd = np.rad2deg(2.0 * e + 0.4 * (-q_m))   # gimbal PID (deg); clipped to u_max by actuator

        g = tvc.step(gim_cmd, dt); dep, _ = canact.step(deploy, 0.0, dt)
        a0 = s.to_array()
        f = lambda arr: deriv(State.from_array(arr), g, SM_AFT, thrust, vp, pf, wa,
                              canard=can, deploy_frac=dep, canard_defl_rad=0.0)
        k1 = f(a0); k2 = f(a0 + .5 * dt * k1); k3 = f(a0 + .5 * dt * k2); k4 = f(a0 + dt * k3)
        s = State.from_array(a0 + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
        th_h.append(s.theta)
        if t >= man_t:
            post += 1; supp_eff += abs(np.rad2deg(g)) * dt
        if not np.isfinite(s.theta) or abs(np.rad2deg(s.theta - ref)) > 120 or abs(np.rad2deg(s.theta - s.gamma)) > 80:
            return dict(success=False, settle=None, supp_eff=None)
    th = np.rad2deg(np.array(th_h)); tgt = np.rad2deg(ref); tt = np.arange(len(th)) * dt
    pm = tt >= man_t; tp = tt[pm]; thp = th[pm]; near = np.abs(thp - tgt) < 3
    settle = None
    if near.any():
        outs = np.where(~near)[0]
        settle = float(tp[outs[-1] + 1] - man_t) if (len(outs) and outs[-1] + 1 < len(tp)) else float(tp[np.argmax(near)] - man_t)
    rms = float(np.sqrt(np.mean((thp[tp >= tp[-1] - 0.5] - tgt) ** 2)))
    return dict(success=bool(near.any() and rms < 5), settle=settle, supp_eff=round(supp_eff, 3))


def agg(supp_mode, u_max, stall_deg, stall_model, seeds=range(12)):
    R = [run(supp_mode, u_max, stall_deg=stall_deg, stall_model=stall_model, seed=s) for s in seeds]
    def mo(k):
        v = [r[k] for r in R if r.get(k) is not None]; return np.mean(v) if v else None
    return dict(succ=np.mean([r["success"] for r in R]), settle=mo("settle"), eff=mo("supp_eff"))


def f(x, p=2): return "  -  " if x is None else f"{x:.{p}f}"


def main():
    print("######### MARGIN AS PRIMARY: how small can the STARTER supplement be? #########")
    print("35deg maneuver, 12 seeds. supplement = TVC starter kick only (then OFF); margin does fling+catch+hold.\n")
    for stall_deg, sm in [(None, "plateau"), (12.0, "plateau"), (12.0, "drop")]:
        tag = "linear (no stall)" if stall_deg is None else f"stall {stall_deg}deg / {sm}"
        print(f"=== {tag} ===")
        print(f"  {'supplement':22} {'u_max':>6} {'succ':>5} {'settle':>7} {'supp_effort':>11}")
        # starter-only at shrinking authority
        for um in [2.5, 1.0, 0.5, 0.25, 0.0]:
            a = agg("starter", um, stall_deg, sm)
            print(f"  {'starter-only':22} {um:6.2f} {a['succ']:5.2f} {f(a['settle']):>7} {f(a['eff'],3):>11}")
        # comparison: none, and backstop
        an = agg("none", 0.0, stall_deg, sm)
        print(f"  {'NONE (margin self-start)':22} {0.0:6.2f} {an['succ']:5.2f} {f(an['settle']):>7} {f(an['eff'],3):>11}")
        ab = agg("backstop", 2.5, stall_deg, sm)
        print(f"  {'backstop (start+hold)':22} {2.5:6.2f} {ab['succ']:5.2f} {f(ab['settle']):>7} {f(ab['eff'],3):>11}")
        print()
    print("READ: if 'starter-only' keeps succeeding as u_max shrinks toward 0, the supplement authority")
    print("requirement really does -> ~0 (your claim). 'NONE' tests whether margin can even self-start.")


if __name__ == "__main__":
    main()
