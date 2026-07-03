"""
tools/vsr_canard_comparison.py  (2026-06-26)

Answers the user's question: with fixed aft fins + DEPLOYABLE FORWARD CANARDS, is
  RAW CANARD CONTROL (actively deflect the canard to steer)
better or worse than
  MARGIN MODULATION (deploy/retract the canard only to change stability; TVC steers)?
...and how do both compare to plain TVC, across powered vs coast and adequate vs weak TVC?

Four modes (same airframe: aft fins give baseline +0.8 cal, canard is forward/deployable):
  tvc_only    : canard folded; PID gimbal controls attitude               (conventional)
  margin_mod  : canard DEPLOY/RETRACT (defl=0) flings unstable/catches stable; PID gimbal steers
  canard_only : TVC OFF (gimbal=0); canard fully deployed; PID controls canard DEFLECTION
  canard_tvc  : both active (gimbal + canard deflection share the attitude loop)

Physics is the real canard model in sim_vsr.vehicle.deriv: N_c = q*S_c*CN_c*(alpha+defl) at a
forward arm -> alpha-part destabilizes (margin), defl-part steers (control). Canard authority
scales with q ~ V^2 (weak slow, strong fast); TVC scales with thrust (dead in coast).
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from sim_vsr import (VehicleParams, CanardParams, TVCParams, TVCActuator,
                     CanardActParams, CanardActuator, IMUParams, IMU,
                     Scenario, State, deriv)
from sim_vsr.vehicle import mass_props, aero_forces, keff_phys, CU_TO_RAD
from sim_vsr.scenario import WindOU

SM_BASE = 0.8                 # aft-fin baseline static margin (cal), canard retracted
Q_REF = 540.0                 # reference dynamic pressure for canard gain scheduling (V~30 m/s)


def run(mode, vp, canard, u_max, man_deg, ramp, launch_speed, maneuver_time, t_end,
        Kp_t=0.3, Kd_t=0.06, Ki_t=0.6, Kp_c=1.8, Kd_c=0.36, q_sched=True,
        wind_sigma=1.5, seed=0, dt=0.005):
    rng = np.random.default_rng(seed)
    scn = Scenario(t_end=t_end, dt=dt, launch_speed=launch_speed, maneuver_time=maneuver_time,
                   maneuver_deg=man_deg, maneuver_ramp=ramp, wind_sigma=wind_sigma)
    tvc = TVCActuator(TVCParams(u_max_deg=u_max, slew_max_deg_s=120, deadband_deg=0.2, backlash_deg=0.3))
    can = CanardActuator(CanardActParams(defl_max_deg=canard.defl_max_deg, deploy_rate_s=8.0,
                                         defl_slew_deg_s=300.0))
    imu = IMU(IMUParams(latency_steps=2), rng)
    wind = WindOU(wind_sigma, 0.4, rng)

    s = State(V=launch_speed)
    n = int(t_end / dt)
    from sim_vsr.scenario import thrust_F_class
    total_imp = max(sum(thrust_F_class(k * dt) for k in range(n)) * dt, 1e-6)
    cum_imp = 0.0
    i_term = 0.0
    deploy_state = "stable"

    TH = np.zeros(n); REF = np.zeros(n); AL = np.zeros(n)
    diverged = False
    keff_man = np.nan
    for k in range(n):
        t = k * dt
        thrust = thrust_F_class(t)
        prop_frac = max(0.0, 1.0 - cum_imp / total_imp); cum_imp += thrust * dt
        w = wind.step(dt); wind_alpha = np.arctan2(w, max(s.V, 2.0))
        th_m, q_m = imu.measure(s.theta, s.q)
        ref = scn.theta_ref(t)
        e = ref - th_m
        q_dyn = 0.5 * vp.rho * s.V * s.V

        # --- TVC channel ---
        if mode in ("tvc_only", "margin_mod", "canard_tvc"):
            i_term += e * dt
            gimbal_cmd_deg = np.rad2deg(Kp_t * e + Kd_t * (-q_m) + Ki_t * i_term)
        else:
            gimbal_cmd_deg = 0.0

        # --- canard channel ---
        deploy_cmd, defl_cmd_deg = 0.0, 0.0
        if mode == "margin_mod":
            err_deg = abs(np.rad2deg(e))
            if deploy_state == "stable" and err_deg > 8:
                deploy_state = "deployed"
            elif deploy_state == "deployed" and err_deg < 5:
                deploy_state = "stable"
            deploy_cmd = 1.0 if deploy_state == "deployed" else 0.0
        elif mode in ("canard_only", "canard_tvc"):
            deploy_cmd = 1.0
            sched = (Q_REF / max(q_dyn, 50.0)) if q_sched else 1.0
            sched = min(sched, 8.0)                          # cap boost at very low q
            defl_cmd_deg = np.rad2deg(Kp_c * sched * e + Kd_c * sched * (-q_m))

        gimbal_rad = tvc.step(gimbal_cmd_deg, dt)
        deploy_frac, canard_defl_rad = can.step(deploy_cmd, defl_cmd_deg, dt)

        if abs(t - maneuver_time) < dt:
            _, Iyy = mass_props(vp, prop_frac)
            keff_man = keff_phys(vp, max(thrust, 1e-3), Iyy)

        a0 = s.to_array()
        f = lambda arr: deriv(State.from_array(arr), gimbal_rad, SM_BASE, thrust, vp, prop_frac,
                              wind_alpha, canard=canard, deploy_frac=deploy_frac,
                              canard_defl_rad=canard_defl_rad)
        k1 = f(a0); k2 = f(a0 + 0.5 * dt * k1); k3 = f(a0 + 0.5 * dt * k2); k4 = f(a0 + dt * k3)
        s = State.from_array(a0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4))
        TH[k] = s.theta; REF[k] = ref; AL[k] = s.theta - s.gamma
        if (not np.isfinite(s.theta)) or abs(np.rad2deg(s.theta - ref)) > 120 or abs(np.rad2deg(AL[k])) > 75:
            diverged = True; TH = TH[:k+1]; REF = REF[:k+1]; AL = AL[:k+1]; break

    return _metrics(np.arange(len(TH)) * dt, TH, REF, AL, maneuver_time, diverged,
                    tvc.slew_sat_frac, can.defl_slew_sat_frac, keff_man)


def _metrics(T, TH, REF, AL, man_t, diverged, tvc_sat, can_sat, keff):
    deg = np.rad2deg
    out = dict(diverged=diverged, tvc_sat=round(float(tvc_sat), 3), can_sat=round(float(can_sat), 3),
               max_alpha=round(float(np.max(np.abs(deg(AL)))), 1) if len(AL) else None,
               Pi=round(float(keff * CU_TO_RAD * 4), 1) if np.isfinite(keff) else None)
    if diverged or len(T) < 5:
        out.update(reached=False, settle=None, overshoot=None, rms=None, success=False); return out
    err = deg(TH - REF); post = T >= man_t; tp = T[post]; ep = err[post]
    if len(tp) < 3:
        out.update(reached=False, settle=None, overshoot=None, rms=None, success=False); return out
    within = np.abs(ep) < 2.0
    if within.any():
        lo = np.where(~within)[0]
        settle = float(tp[lo[-1] + 1] - man_t) if (len(lo) and lo[-1] + 1 < len(tp)) else float(tp[np.argmax(within)] - man_t)
    else:
        settle = None
    over = max(0.0, float(np.max(-ep))) if len(ep) else None       # pitch-over reduces theta
    tail = tp >= (tp[-1] - 0.5); rms = float(np.sqrt(np.mean(ep[tail]**2))) if tail.any() else None
    success = bool(within.any() and rms is not None and rms < 5.0 and not diverged)
    out.update(reached=bool(within.any()), settle=round(settle, 3) if settle is not None else None,
               overshoot=round(over, 1) if over is not None else None,
               rms=round(rms, 2) if rms is not None else None, success=success)
    return out


def cellavg(mode, vp, canard, seeds=range(12), **kw):
    rows = [run(mode, vp, canard, seed=sd, **kw) for sd in seeds]
    def mo(k):
        v = [r[k] for r in rows if r.get(k) is not None]
        return np.mean(v) if v else None
    return dict(succ=np.mean([r["success"] for r in rows]), div=np.mean([r["diverged"] for r in rows]),
                settle=mo("settle"), over=mo("overshoot"), maxA=mo("max_alpha"),
                tvc_sat=mo("tvc_sat"), can_sat=mo("can_sat"))


def main():
    vp = VehicleParams()
    canard = CanardParams(S_c=0.0018, CN_c=3.0, arm_c=0.28, defl_max_deg=20.0)
    dm = canard.dmargin_full(vp)
    print("=== CANARD CONTROL vs MARGIN MODULATION vs TVC (full canard physics, 12 seeds) ===")
    print(f"  airframe: aft fins baseline SM=+{SM_BASE} cal; canard full deploy reduces SM by "
          f"{dm:.2f} cal -> deployed SM = +{SM_BASE-dm:.2f} cal")
    print(f"  canard: S_c={canard.S_c*1e4:.0f}cm^2 CN={canard.CN_c} arm={canard.arm_c}m defl_max={canard.defl_max_deg}deg\n")

    regimes = [
        ("POWERED, adequate TVC", dict(u_max=7, man_deg=20, ramp=0.4, launch_speed=15, maneuver_time=0.8, t_end=2.0)),
        ("POWERED, WEAK TVC (3deg)", dict(u_max=3, man_deg=25, ramp=0.3, launch_speed=15, maneuver_time=0.8, t_end=2.0)),
        ("HIGH-SPEED, weak TVC", dict(u_max=3, man_deg=25, ramp=0.3, launch_speed=45, maneuver_time=0.5, t_end=1.8)),
        ("COAST (thrust=0, TVC dead)", dict(u_max=7, man_deg=20, ramp=0.4, launch_speed=15, maneuver_time=2.6, t_end=3.6)),
    ]
    modes = ["tvc_only", "margin_mod", "canard_only", "canard_tvc"]
    for label, kw in regimes:
        print(f"--- {label} ---")
        print(f"  {'mode':12} {'succ':>5} {'div':>5} {'settle':>7} {'over':>6} {'maxA':>6} {'tvcSat':>7} {'canSat':>7}")
        for mode in modes:
            c = cellavg(mode, vp, canard, **kw)
            def f(x, w=7, p=2): return ("{:>%d}" % w).format("-" if x is None else f"{x:.{p}f}")
            print(f"  {mode:12} {c['succ']:5.2f} {c['div']:5.2f} {f(c['settle'])} "
                  f"{f(c['over'],6,1)} {f(c['maxA'],6,1)} {f(c['tvc_sat'],7,3)} {f(c['can_sat'],7,3)}")
        print()
    print("READ: coast row is decisive -- tvc_only should FAIL (no thrust to vector) while canard")
    print("modes still work. Compare margin_mod vs canard_only/canard_tvc for the powered rows.")


if __name__ == "__main__":
    main()
