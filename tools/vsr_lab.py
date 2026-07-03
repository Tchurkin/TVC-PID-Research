"""
tools/vsr_lab.py  (2026-06-27)

Shared simulation lab for the variable-stability-rocket control comparison. Runs ONE airframe
and ONE maneuver under four control mechanisms, with a MATCHED feedback-linearizing control law
(same desired angular-acceleration a_des = Kp*e + Kd*(-q)) so the comparison isolates the ACTUATOR,
not the tuning:

  tvc     : thrust vectoring          -- gimbal = asin(M_des / (T*L))             (needs thrust)
  canard  : canard deflection control -- delta  = M_des / (q*S_c*deploy*CN_c*arm) (deploy fixed=0.5 neutral)
  margin  : static-margin modulation  -- deploy = 0.5 + M_des/(q*S_c*CN_c*alpha*arm)  (delta=0; TVC backstop at alpha~0)
  hybrid  : margin fling + canard damp -- deploy modulated to fling unstable, delta = canard control

Airframe is held identical: static aft fins (SM_aft) + forward canards that CANCEL them at HALF
deploy -> deploy=0.5 is NEUTRAL, deploy=0 is stable (+SM_aft), deploy=1 is unstable (-SM_aft).

run() returns a Trace with full time series (for animation) and a metrics dict (for comparison).
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim_vsr import (VehicleParams, CanardParams, TVCParams, TVCActuator,
                     CanardActParams, CanardActuator, IMUParams, IMU, State, deriv)
from sim_vsr.vehicle import mass_props
from sim_vsr.scenario import thrust_F_class, WindOU

KP, KD = 36.0, 8.0          # desired-accel gains (omega_n=6, zeta=0.67) -- SAME for all controllers
QREF = 540.0


@dataclass
class Trace:
    t: np.ndarray
    x: np.ndarray; z: np.ndarray; V: np.ndarray
    theta: np.ndarray; gamma: np.ndarray; alpha: np.ndarray; ref: np.ndarray
    gimbal_deg: np.ndarray            # TVC nozzle angle
    deploy: np.ndarray                # canard deploy fraction [0,1]  (0.5=neutral)
    canard_deg: np.ndarray            # canard deflection angle
    margin: np.ndarray                # net static margin [cal]
    metrics: dict = field(default_factory=dict)


def run(mode, man_deg=30.0, ramp=0.08, stall_deg=None, stall_model="plateau", seed=0,
        t_end=2.4, dt=0.005, wind_sigma=1.5, man_t=0.8) -> Trace:
    vp = VehicleParams(stall_deg=stall_deg, stall_model=stall_model)
    can = CanardParams(S_c=0.0028, CN_c=3.0, arm_c=0.28, defl_max_deg=20.0)
    DM = can.dmargin_full(vp)
    SM_AFT = 0.5 * DM                      # half-deploy cancels -> neutral
    rng = np.random.default_rng(seed)
    target = np.pi / 2 - np.deg2rad(man_deg)
    tvc = TVCActuator(TVCParams(u_max_deg=(7.0 if mode == "tvc" else 2.5),
                                slew_max_deg_s=300, deadband_deg=0.2, backlash_deg=0.3))
    canact = CanardActuator(CanardActParams(defl_max_deg=can.defl_max_deg,
                                            deploy_rate_s=8.0, defl_slew_deg_s=400))
    imu = IMU(IMUParams(latency_steps=2), rng)
    wind = WindOU(wind_sigma, 0.4, rng)
    s = State(V=22.0)
    n = int(t_end / dt)
    total_imp = max(sum(thrust_F_class(k * dt) for k in range(n)) * dt, 1e-6)
    cum = 0.0; i_t = 0.0; dep_state = "catch"; af = np.deg2rad(3.0)

    T = np.zeros(n); X = np.zeros(n); Z = np.zeros(n); VV = np.zeros(n)
    TH = np.zeros(n); GA = np.zeros(n); AL = np.zeros(n); REF = np.zeros(n)
    GD = np.zeros(n); DEP = np.zeros(n); CD = np.zeros(n); MG = np.zeros(n)
    tvc_steps = 0; eff = 0.0

    def ref_at(t):
        if t < man_t:
            return np.pi / 2
        f = min(1.0, (t - man_t) / ramp)
        return np.pi / 2 - np.deg2rad(man_deg) * f

    diverged = False
    for k in range(n):
        t = k * dt
        thrust = thrust_F_class(t)
        pf = max(0.0, 1.0 - cum / total_imp); cum += thrust * dt
        m, Iyy = mass_props(vp, pf); q = 0.5 * vp.rho * s.V * s.V
        w = wind.step(dt) if t > man_t else 0.0
        wa = np.arctan2(w, max(s.V, 2.0))
        th_m, q_m = imu.measure(s.theta, s.q)
        ref = ref_at(t); e = ref - th_m
        ae = (s.theta - s.gamma) + np.deg2rad(0.5) * rng.standard_normal()
        a_des = KP * e + KD * (-q_m); M_des = Iyy * a_des

        gimbal_cmd_deg = 0.0; deploy_cmd = 0.5; defl_cmd_deg = 0.0
        if mode == "tvc":
            if thrust > 1.0:
                gimbal_cmd_deg = np.rad2deg(np.arcsin(np.clip(M_des / (thrust * vp.L_nozzle), -1, 1)))
            deploy_cmd = 0.5
        elif mode == "canard":
            deploy_cmd = 0.5
            per = q * can.S_c * 0.5 * can.CN_c * can.arm_c
            if per > 1e-9:
                defl_cmd_deg = np.rad2deg(M_des / per)
        elif mode == "margin":
            defl_cmd_deg = 0.0
            per = q * can.S_c * can.CN_c * ae * can.arm_c
            if abs(ae) > af and abs(per) > 1e-6:
                deploy_cmd = float(np.clip(0.5 + M_des / per, 0, 1))
            else:                               # TVC backstop near alpha=0
                i_t += e * dt
                gimbal_cmd_deg = np.rad2deg(0.3 * e + 0.06 * (-q_m))
                deploy_cmd = 0.5
        elif mode == "hybrid":                  # fling unstable, damp with canard deflection
            ed = abs(np.rad2deg(e))
            if dep_state == "catch" and ed > 8: dep_state = "fling"
            elif dep_state == "fling" and ed < 5: dep_state = "catch"
            deploy_cmd = 1.0 if dep_state == "fling" else 0.0
            sched = min(QREF / max(q, 50), 8.0)
            defl_cmd_deg = np.rad2deg(0.8 * sched * e + 0.15 * sched * (-q_m))

        g = tvc.step(gimbal_cmd_deg, dt)
        dep, dfl = canact.step(deploy_cmd, defl_cmd_deg, dt)
        margin = SM_AFT - DM * dep              # net static margin [cal]

        a0 = s.to_array()
        f = lambda arr: deriv(State.from_array(arr), g, SM_AFT, thrust, vp, pf, wa,
                              canard=can, deploy_frac=dep, canard_defl_rad=dfl)
        k1 = f(a0); k2 = f(a0 + .5 * dt * k1); k3 = f(a0 + .5 * dt * k2); k4 = f(a0 + dt * k3)
        s = State.from_array(a0 + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

        T[k] = t; X[k] = s.x; Z[k] = s.z; VV[k] = s.V
        TH[k] = s.theta; GA[k] = s.gamma; AL[k] = s.theta - s.gamma; REF[k] = ref
        GD[k] = np.rad2deg(g); DEP[k] = dep; CD[k] = np.rad2deg(dfl); MG[k] = margin
        if t >= man_t:
            if abs(GD[k]) > 0.5: tvc_steps += 1
            eff += (abs(GD[k]) + abs(CD[k])) * dt
        if (not np.isfinite(s.theta)) or abs(np.rad2deg(s.theta - ref)) > 120 or abs(np.rad2deg(AL[k])) > 80:
            diverged = True
            sl = slice(0, k + 1)
            T, X, Z, VV, TH, GA, AL, REF, GD, DEP, CD, MG = (a[sl] for a in
                (T, X, Z, VV, TH, GA, AL, REF, GD, DEP, CD, MG))
            break

    met = _metrics(T, TH, REF, AL, GD, man_t, diverged, mode)
    met["tvc_duty"] = tvc_steps / max(1, np.sum(T >= man_t))
    met["effort"] = round(float(eff), 1)
    return Trace(T, X, Z, VV, TH, GA, AL, REF, GD, DEP, CD, MG, met)


def _metrics(T, TH, REF, AL, GD, man_t, diverged, mode):
    deg = np.rad2deg
    m = dict(mode=mode, diverged=bool(diverged),
             peak_aoa=round(float(np.max(np.abs(deg(AL)))), 1) if len(AL) else None)
    if diverged or len(T) < 5:
        m.update(success=False, reach=None, settle=None, overshoot=None); return m
    err = deg(TH - REF); post = T >= man_t; tp = T[post]; ep = err[post]
    tgt0 = deg(REF[-1])
    th_post = deg(TH[post]); near = np.abs(th_post - tgt0) < 3
    reach = float(tp[np.argmax(near)] - man_t) if near.any() else None
    if near.any():
        outs = np.where(~near)[0]
        settle = float(tp[outs[-1] + 1] - man_t) if (len(outs) and outs[-1] + 1 < len(tp)) else reach
    else:
        settle = None
    over = float(np.max(tgt0 * 0 + (deg(REF[-1]) - th_post)))   # pitch-over reduces theta
    over = max(0.0, over) if len(th_post) else None
    tail = tp >= (tp[-1] - 0.5)
    rms = float(np.sqrt(np.mean((th_post[tail] - tgt0) ** 2))) if tail.any() else None
    m.update(success=bool(near.any() and rms is not None and rms < 5.0 and not diverged),
             reach=round(reach, 3) if reach is not None else None,
             settle=round(settle, 3) if settle is not None else None,
             overshoot=round(over, 1) if over is not None else None)
    return m


if __name__ == "__main__":
    for mode in ["tvc", "canard", "margin", "hybrid"]:
        tr = run(mode, seed=0)
        print(mode, tr.metrics)
