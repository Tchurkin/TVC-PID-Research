"""
sim_vsr.simulate -- fixed-step RK4 integrator + flight metrics.

Controller, scheduler, and actuators run once per dt (zero-order hold); the continuous plant
is integrated with RK4 between control updates. This mirrors a real digital flight computer:
discrete control at the loop rate, continuous physics in between.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from .vehicle import VehicleParams, State, deriv, mass_props, keff_phys, CU_TO_RAD
from .actuators import TVCActuator, TVCParams, MorphActuator, MorphParams
from .sensors import IMU, IMUParams
from .control import TVCController, StabilityScheduler
from .scenario import Scenario, WindOU


@dataclass
class SimResult:
    t: np.ndarray
    theta: np.ndarray
    theta_ref: np.ndarray
    gamma: np.ndarray
    alpha: np.ndarray
    q: np.ndarray
    gimbal_deg: np.ndarray
    sm: np.ndarray
    z: np.ndarray
    V: np.ndarray
    metrics: dict = field(default_factory=dict)


def _total_impulse(scn: Scenario) -> float:
    ts = np.arange(0.0, scn.t_end, scn.dt)
    return float(np.sum([scn.thrust_fn(t) for t in ts]) * scn.dt)


def simulate(vp: VehicleParams, tvc_params: TVCParams, morph_params: MorphParams,
             imu_params: IMUParams, controller: TVCController,
             scheduler: StabilityScheduler, scn: Scenario,
             seed: int = 0, sm0: Optional[float] = None) -> SimResult:
    rng = np.random.default_rng(seed)
    dt = scn.dt
    n = int(scn.t_end / dt)

    tvc = TVCActuator(tvc_params)
    if sm0 is None:
        sm0 = scheduler.cfg.sm_stable
    morph = MorphActuator(morph_params, sm0)
    imu = IMU(imu_params, rng)
    wind = WindOU(scn.wind_sigma, scn.wind_tau, rng)
    controller.reset()
    scheduler.reset()

    s = State(V=scn.launch_speed)        # straight up, at rail-exit speed (control begins here)
    total_imp = max(_total_impulse(scn), 1e-6)
    cum_imp = 0.0

    T = np.zeros(n); TH = np.zeros(n); REF = np.zeros(n); GA = np.zeros(n)
    AL = np.zeros(n); Q = np.zeros(n); GD = np.zeros(n); SM = np.zeros(n)
    Z = np.zeros(n); VV = np.zeros(n)

    diverged = False
    keff_at_maneuver = np.nan
    for k in range(n):
        t = k * dt
        thrust = scn.thrust_fn(t)
        prop_frac = max(0.0, 1.0 - cum_imp / total_imp)
        cum_imp += thrust * dt

        w = wind.step(dt)
        wind_alpha = np.arctan2(w, max(s.V, 2.0))   # floor V so a gust can't make a singular AoA

        th_m, q_m = imu.measure(s.theta, s.q)
        ref = scn.theta_ref(t)
        gimbal_cmd_deg = controller.command(th_m, q_m, ref)
        sm_cmd = scheduler.command(th_m, q_m, ref)

        gimbal_rad = tvc.step(gimbal_cmd_deg, dt)
        sm = morph.step(sm_cmd, dt)

        if abs(t - scn.maneuver_time) < dt:
            _, Iyy = mass_props(vp, prop_frac)
            keff_at_maneuver = keff_phys(vp, max(thrust, 1e-3), Iyy)

        # RK4 on the continuous plant (gimbal/sm/thrust/wind held over the step)
        a0 = s.to_array()
        f = lambda arr: deriv(State.from_array(arr), gimbal_rad, sm, thrust, vp, prop_frac, wind_alpha)
        k1 = f(a0); k2 = f(a0 + 0.5 * dt * k1)
        k3 = f(a0 + 0.5 * dt * k2); k4 = f(a0 + dt * k3)
        a1 = a0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        s = State.from_array(a1)

        alpha = s.theta - s.gamma
        T[k] = t; TH[k] = s.theta; REF[k] = ref; GA[k] = s.gamma
        AL[k] = alpha; Q[k] = s.q; GD[k] = np.rad2deg(gimbal_rad); SM[k] = sm
        Z[k] = s.z; VV[k] = s.V

        if (not np.isfinite(s.theta)) or abs(np.rad2deg(s.theta - ref)) > 120 or abs(np.rad2deg(alpha)) > 75:
            diverged = True
            T = T[:k + 1]; TH = TH[:k + 1]; REF = REF[:k + 1]; GA = GA[:k + 1]
            AL = AL[:k + 1]; Q = Q[:k + 1]; GD = GD[:k + 1]; SM = SM[:k + 1]
            Z = Z[:k + 1]; VV = VV[:k + 1]
            break

    metrics = _metrics(T, TH, REF, AL, scn, diverged, tvc.slew_sat_frac,
                       morph.rate_limited_frac, keff_at_maneuver, imu_params.latency_steps)
    return SimResult(T, TH, REF, GA, AL, Q, GD, SM, Z, VV, metrics)


def _metrics(T, TH, REF, AL, scn, diverged, slew_sat_frac, morph_rate_frac,
             keff, latency_steps) -> dict:
    deg = np.rad2deg
    # keff passed in is in rad/s^2 per rad-gimbal; convert to per-CU so Pi is comparable to
    # the project's Pi_crit ~ 275 (which uses keff in rad/s^2/CU).
    keff_cu = keff * CU_TO_RAD if np.isfinite(keff) else np.nan
    pi_cu = round(float(keff_cu * (latency_steps ** 2)), 1) if np.isfinite(keff_cu) else None
    m = dict(diverged=bool(diverged), slew_sat_frac=round(float(slew_sat_frac), 3),
             morph_rate_limited_frac=round(float(morph_rate_frac), 3),
             keff_phys_rad=round(float(keff), 1) if np.isfinite(keff) else None,
             keff_cu=round(float(keff_cu), 2) if np.isfinite(keff_cu) else None,
             latency_steps=int(latency_steps), max_alpha_deg=round(float(np.max(np.abs(deg(AL)))), 1))
    if diverged or len(T) < 5:
        m.update(reached=False, t_reach=None, settle_time=None, overshoot_deg=None,
                 rms_err_deg=None, success=False)
        m["Pi"] = pi_cu
        return m

    err = deg(TH - REF)
    post = T >= scn.maneuver_time
    tp = T[post]; ep = err[post]
    if len(tp) < 3:
        m.update(reached=False, t_reach=None, settle_time=None, overshoot_deg=None,
                 rms_err_deg=None, success=False)
        m["Pi"] = pi_cu
        return m

    within = np.abs(ep) < 2.0
    t_reach = float(tp[np.argmax(within)] - scn.maneuver_time) if within.any() else None
    # settle time = last time it left the 2deg band, relative to maneuver
    if within.any():
        last_out = np.where(~within)[0]
        settle = float(tp[last_out[-1] + 1] - scn.maneuver_time) if (len(last_out) and last_out[-1] + 1 < len(tp)) else t_reach
    else:
        settle = None
    sign = -1.0 if scn.maneuver_deg > 0 else 1.0     # pitch-over reduces theta
    overshoot = float(np.max(sign * ep)) if len(ep) else None
    overshoot = max(0.0, overshoot) if overshoot is not None else None
    # steady RMS over the last 0.5 s
    tail = tp >= (tp[-1] - 0.5)
    rms = float(np.sqrt(np.mean(ep[tail] ** 2))) if tail.any() else None
    success = bool(within.any() and (rms is not None and rms < 5.0) and not diverged)

    m.update(reached=bool(within.any()), t_reach=round(t_reach, 3) if t_reach is not None else None,
             settle_time=round(settle, 3) if settle is not None else None,
             overshoot_deg=round(overshoot, 1) if overshoot is not None else None,
             rms_err_deg=round(rms, 2) if rms is not None else None, success=success)
    m["Pi"] = pi_cu
    return m
