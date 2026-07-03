"""
tools/vsr_complement.py  (2026-06-27)

Rigorously tests the REFRAMED thesis: margin modulation is an authority-OFFLOADING COMPLEMENT.
It cannot start the angle of attack (singular at alpha=0), so a PRIMARY actuator (TVC gimbal or
steerable aft fins) handles the alpha~0 starter, the residual, and damping; margin modulation
deploys to carry the BULK of the rotation aerodynamically once alpha is established.

Control allocation each step (matched FL law a_des = Kp*e + Kd*(-q), M_des = Iyy*a_des):
  if |alpha| > floor:  margin provides as much of M_des as deploy in [0,1] allows
                       residual = M_des - M_margin_actual  -> PRIMARY provides residual
  else (alpha~0):      margin idle (neutral);  PRIMARY provides all of M_des   (the "starter")

Two questions, both falsifiable:
  Q1 FASTER?         at matched primary authority, does +margin reach/settle quicker?
  Q2 LESS ACTUATION? at matched authority, does +margin use less primary effort & lower peak?
  Q3 LESS AUTHORITY? what is the MINIMUM primary authority for success, with vs without margin?

Primaries: 'tvc' (gimbal, needs thrust)  and  'aft' (steerable aft fins, all-aero, works in coast).
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from sim_vsr import VehicleParams, CanardParams, TVCParams, TVCActuator, CanardActParams, CanardActuator, IMUParams, IMU, State, deriv
from sim_vsr.vehicle import mass_props
from sim_vsr.scenario import thrust_F_class, WindOU

KP, KD = 36.0, 8.0


def run(primary, use_margin, prim_auth, man=35.0, ramp=0.08, stall_deg=None, seed=0,
        t_end=2.4, dt=0.005, coast=False):
    vp = VehicleParams(stall_deg=stall_deg)
    can = CanardParams(S_c=0.0028, CN_c=3.0, arm_c=0.28, defl_max_deg=20.0)
    DM = can.dmargin_full(vp); SM_AFT = 0.5 * DM
    rng = np.random.default_rng(seed); man_t = 0.8; target = np.pi / 2 - np.deg2rad(man)
    u_max = prim_auth if primary == "tvc" else 7.0
    tvc = TVCActuator(TVCParams(u_max_deg=u_max, slew_max_deg_s=400, deadband_deg=0.2, backlash_deg=0.3))
    canact = CanardActuator(CanardActParams(defl_max_deg=can.defl_max_deg, deploy_rate_s=8.0, defl_slew_deg_s=500))
    imu = IMU(IMUParams(latency_steps=2), rng); wind = WindOU(1.5, 0.4, rng)
    aft_max = np.deg2rad(prim_auth) if primary == "aft" else np.deg2rad(20.0)
    aft = 0.0; aft_slew = np.deg2rad(500) * dt
    s = State(V=22.0); n = int(t_end / dt)
    total = max(sum(thrust_F_class(k * dt) for k in range(n)) * dt, 1e-6); cum = 0.0; af = np.deg2rad(3.0)
    th_h = []; post = 0; prim_eff = 0.0; peak_prim = 0.0; per_aft = None
    for k in range(n):
        t = k * dt
        thrust = 0.0 if coast else thrust_F_class(t)
        # if coast: give a brief boost so it has airspeed, then thrust=0 during maneuver
        if coast and t < 0.6:
            thrust = thrust_F_class(t)
        pf = max(0.0, 1.0 - cum / total); cum += thrust * dt
        m, Iyy = mass_props(vp, pf); q = 0.5 * vp.rho * s.V * s.V
        w = wind.step(dt) if t > man_t else 0.0; wa = np.arctan2(w, max(s.V, 2.0))
        th_m, q_m = imu.measure(s.theta, s.q)
        ref = (np.pi / 2) if t < man_t else (np.pi / 2 - np.deg2rad(man) * min(1.0, (t - man_t) / ramp))
        e = ref - th_m; ae = (s.theta - s.gamma) + np.deg2rad(0.5) * rng.standard_normal()
        M_des = Iyy * (KP * e + KD * (-q_m))

        # --- margin allocation (SMOOTH alpha-weighted blend: phase margin in with alpha so the
        #     1/alpha handoff does not blow up / ring near the target) ---
        deploy = 0.5; M_margin = 0.0
        if use_margin:
            w = min(abs(ae) / np.deg2rad(8.0), 1.0)      # 0 at alpha~0 -> 1 at |alpha|>=8deg
            per = q * can.S_c * can.CN_c * ae * can.arm_c
            if abs(per) > 1e-6:
                deploy = float(np.clip(0.5 + (M_des * w) / per, 0, 1))
                M_margin = q * can.S_c * can.CN_c * can.arm_c * (deploy - 0.5) * ae
        residual = M_des - M_margin

        # --- primary provides residual ---
        gim_cmd = 0.0; aft_cmd = 0.0
        if primary == "tvc":
            if thrust > 1.0:
                gim_cmd = np.rad2deg(np.arcsin(np.clip(residual / (thrust * vp.L_nozzle), -1, 1)))
            else:                       # no thrust -> gimbal useless; margin/aft must carry (coast)
                gim_cmd = 0.0
        else:  # aft fins
            per_aft = q * can.S_c * can.CN_c * can.arm_c
            aft_cmd = residual / per_aft if per_aft > 1e-9 else 0.0   # rad (linear inversion)

        g = tvc.step(gim_cmd, dt)
        dep, _ = canact.step(deploy, 0.0, dt)
        # aft fin slew + clip
        aft_des = float(np.clip(aft_cmd, -aft_max, aft_max))
        aft += np.clip(aft_des - aft, -aft_slew, aft_slew)

        a0 = s.to_array()
        f = lambda arr: deriv(State.from_array(arr), g, SM_AFT, thrust, vp, pf, wa,
                              canard=can, deploy_frac=dep, canard_defl_rad=0.0, aft_defl_rad=aft)
        k1 = f(a0); k2 = f(a0 + .5 * dt * k1); k3 = f(a0 + .5 * dt * k2); k4 = f(a0 + dt * k3)
        s = State.from_array(a0 + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
        th_h.append(s.theta)
        if t >= man_t:
            post += 1
            if primary == "tvc":
                prim_eff += abs(np.rad2deg(g)) * dt; peak_prim = max(peak_prim, abs(np.rad2deg(g)))
            else:
                prim_eff += abs(np.rad2deg(aft)) * dt; peak_prim = max(peak_prim, abs(np.rad2deg(aft)))
        if not np.isfinite(s.theta) or abs(np.rad2deg(s.theta - ref)) > 120 or abs(np.rad2deg(s.theta - s.gamma)) > 80:
            return dict(success=False, diverged=True, reach=None, settle=None, eff=None, peak=None)
    th = np.rad2deg(np.array(th_h)); tgt = np.rad2deg(ref); tt = np.arange(len(th)) * dt
    pm = tt >= man_t; tp = tt[pm]; thp = th[pm]; near = np.abs(thp - tgt) < 3
    reach = float(tp[np.argmax(near)] - man_t) if near.any() else None
    settle = None
    if near.any():
        outs = np.where(~near)[0]
        settle = float(tp[outs[-1] + 1] - man_t) if (len(outs) and outs[-1] + 1 < len(tp)) else reach
    rms = float(np.sqrt(np.mean((thp[tp >= tp[-1] - 0.5] - tgt) ** 2)))
    return dict(success=bool(near.any() and rms < 5), diverged=False, reach=reach, settle=settle,
                eff=round(prim_eff, 2), peak=round(peak_prim, 1))


def agg(primary, use_margin, auth, stall, coast=False, seeds=range(12)):
    R = [run(primary, use_margin, auth, stall_deg=stall, seed=s, coast=coast) for s in seeds]
    def mo(k):
        v = [r[k] for r in R if r.get(k) is not None]; return np.mean(v) if v else None
    return dict(succ=np.mean([r["success"] for r in R]), reach=mo("reach"), settle=mo("settle"),
                eff=mo("eff"), peak=mo("peak"))


def fmt(x, p=2): return "  -  " if x is None else f"{x:.{p}f}"


def q12(primary, auth, stall, tag):
    print(f"\n--- {tag}: primary={primary.upper()}, authority={auth}{'deg' if primary=='aft' else 'deg gimbal'}, "
          f"stall={'OFF' if stall is None else str(stall)} ---")
    print(f"  {'config':16} {'succ':>5} {'reach':>6} {'settle':>7} {'prim_effort':>12} {'peak_prim':>10}")
    for um, lab in [(False, primary + " alone"), (True, primary + " + MARGIN")]:
        a = agg(primary, um, auth, stall)
        print(f"  {lab:16} {a['succ']:5.2f} {fmt(a['reach']):>6} {fmt(a['settle']):>7} {fmt(a['eff'],2):>12} {fmt(a['peak'],1):>10}")


def min_authority(primary, stall):
    levels = [1, 1.5, 2, 3, 5, 7] if primary == "tvc" else [2, 3, 4, 6, 10, 15]
    out = {}
    for um in (False, True):
        best = None
        for L in levels:
            if agg(primary, um, L, stall)["succ"] >= 0.9:
                best = L; break
        out[um] = best
    return out


def main():
    print("########## MARGIN MODULATION AS AN AUTHORITY-OFFLOADING COMPLEMENT ##########")
    print("35deg maneuver, 12 seeds. Q1 faster? Q2 less effort/peak? Q3 less authority needed?\n")
    print("==================  Q1 & Q2: matched authority  ==================")
    q12("tvc", 3.0, None, "benign air")
    q12("tvc", 3.0, 12.0, "stalling air")
    q12("aft", 6.0, 12.0, "stalling air (all-aero, your aft+forward combo)")

    print("\n==================  Q3: MINIMUM authority for success (>=90%)  ==================")
    for primary, stall, tag in [("tvc", None, "TVC, benign"), ("tvc", 12.0, "TVC, stall"),
                                ("aft", 12.0, "aft fins, stall")]:
        r = min_authority(primary, stall)
        a = r[False]; b = r[True]
        unit = "deg gimbal" if primary == "tvc" else "deg aft-defl"
        print(f"  {tag:18}:  alone needs {a if a else '>max'} {unit}   |   +MARGIN needs {b if b else '>max'} {unit}"
              + (f"   -> {a/b:.1f}x less authority" if (a and b and b < a) else "   -> no reduction"))
    print("\nREAD: a YES on Q1/Q2/Q3 supports 'margin modulation offloads bulk control to aerodynamics'.")


if __name__ == "__main__":
    main()
