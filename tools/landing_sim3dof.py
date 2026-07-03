"""
tools/landing_sim3dof.py  (2026-06-28)  --  3-DOF LONGITUDINAL landing sim (Layer 1 of the 6-DOF build)

Cartesian planar dynamics (x downrange, z up, theta = pitch from VERTICAL: 0 = nose-up/engine-down).
Avoids the V/gamma singularity at hover that makes sim_vsr unsuitable for the landing burn.

Captures: TVC (gimbal delta, |delta|<=delta_max), aero (normal force + drag + CP/CG moment with stall),
deployable canards (CP shift for engine-down descent stability + margin modulation), gravity, wind.
Phases driven externally (boost thrust / coast / landing thrust). Body-tilt throttle = command a tilt so
vertical thrust = T*cos(theta).  NOTE Layer 1: PD attitude control (ADRC + sensor/Kalman models = layer 1.5);
roll/yaw/3-D divert = layer 2.

⚠️ This file LEADS WITH SANITY CHECKS (ballistic projectile, energy conservation, weathercock, canard
descent stability). Do NOT trust landing numbers until the checks pass. Given several buggy sims this
session, validation is the point of this layer.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

G = 9.80665
RHO = 1.20
DT = 0.001


@dataclass
class P:
    m: float = 1.0
    Iyy: float = 0.05
    L_nozzle: float = 0.30      # CG -> nozzle (m)
    d_ref: float = 0.066
    S_ref: float = None
    CN_alpha: float = 8.0       # body+aft-fin normal-force slope (1/rad)
    CD0: float = 0.6
    K_induced: float = 1.2      # induced-drag factor (set 0 to test pure energy conservation)
    C_mq: float = 10.0          # pitch-damping coefficient (rate damping; set 0 for undamped test)
    sm_cal: float = 0.5         # static margin (calibers, +): CP behind CG by sm*d  (aft fins)
    # canard (deployable, forward): adds N forward of CG; deploy_frac in [0,1]
    canard_NA: float = 0.0      # effective (S_c*CN_c) when fully deployed (m^2/rad)
    canard_arm: float = 0.28    # forward of CG (m)
    delta_max: float = np.deg2rad(5.0)

    def __post_init__(self):
        if self.S_ref is None:
            self.S_ref = np.pi * (self.d_ref / 2) ** 2


def _wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def aero_moment_and_forces(p, vx, vz, th, q, deploy, wind_vx=0.0):
    """All-angle planar aero. theta = pitch from VERTICAL; gamma_v = velocity angle from vertical.
    alpha = th - gamma_v (wrapped). CN = CN_alpha*sin(alpha) (valid for all alpha incl. tail-first ~180deg).
    Lift is perpendicular to velocity and RESTORING (turns velocity toward the body axis). Aft fins give a
    restoring moment about CG; deployed canards give a forward (destabilizing-nose-first / stabilizing-tail-
    first) moment -> with a big enough canard, the engine-down (alpha~180) orientation becomes stable."""
    rvx, rvz = vx - wind_vx, vz
    V = np.hypot(rvx, rvz)
    if V < 0.05:
        return 0.0, 0.0, 0.0
    gamma_v = np.arctan2(rvx, rvz)               # velocity direction, angle from +z (vertical)
    alpha = _wrap_pi(th - gamma_v)
    qdyn = 0.5 * RHO * V * V
    sa = np.sin(alpha)
    N_aft = qdyn * p.S_ref * p.CN_alpha * sa
    N_can = qdyn * (p.canard_NA * deploy) * sa
    N = N_aft + N_can
    # lift perpendicular to velocity; +N (alpha>0) must INCREASE gamma_v (turn velocity toward body) = restoring
    lift_dir = (np.cos(gamma_v), -np.sin(gamma_v))
    Fx = N * lift_dir[0]
    Fz = N * lift_dir[1]
    # drag along -velocity
    CD = p.CD0 + p.K_induced * p.CN_alpha * sa * sa
    D = qdyn * p.S_ref * CD
    Fx += -D * (rvx / V)
    Fz += -D * (rvz / V)
    # moments about CG: aft fins restoring (toward alpha=0 nose-first); canard forward (toward alpha=180)
    M = -N_aft * (p.sm_cal * p.d_ref) + N_can * p.canard_arm
    # aerodynamic pitch damping (opposes pitch rate, scales with dynamic pressure)
    M -= p.C_mq * qdyn * p.S_ref * (p.d_ref ** 2) * q / max(V, 0.5)
    return Fx, Fz, M


def deriv(s, thrust, delta, p, deploy, wind_vx=0.0):
    x, z, vx, vz, th, q = s
    # thrust deflected by gimbal delta from body axis (about y)
    Fx_t, Fz_t = thrust * np.sin(th + delta), thrust * np.cos(th + delta)
    M_tvc = thrust * np.sin(delta) * p.L_nozzle      # +delta -> +pitch moment
    Fx_a, Fz_a, M_a = aero_moment_and_forces(p, vx, vz, th, q, deploy, wind_vx)
    ax = (Fx_t + Fx_a) / p.m
    az = (Fz_t + Fz_a) / p.m - G
    qd = (M_tvc + M_a) / p.Iyy
    return np.array([vx, vz, ax, az, q, qd])


def step(s, thrust, delta, p, deploy, wind_vx=0.0):
    k1 = deriv(s, thrust, delta, p, deploy, wind_vx)
    k2 = deriv(s + 0.5*DT*k1, thrust, delta, p, deploy, wind_vx)
    k3 = deriv(s + 0.5*DT*k2, thrust, delta, p, deploy, wind_vx)
    k4 = deriv(s + DT*k3, thrust, delta, p, deploy, wind_vx)
    return s + DT/6*(k1 + 2*k2 + 2*k3 + k4)


# ----------------------------- SANITY CHECKS -----------------------------
def check_ballistic():
    """No thrust, no aero (CN=0,CD0=0), straight up v0 -> apogee must equal v0^2/2g, symmetric."""
    p = P(CN_alpha=0.0, CD0=0.0, canard_NA=0.0)
    v0 = 30.0
    s = np.array([0, 0, 0.0, v0, 0.0, 0.0])
    apo = 0.0; t = 0.0
    while s[1] >= 0 and t < 20:
        s = step(s, 0.0, 0.0, p, 0.0); t += DT
        apo = max(apo, s[1])
    expect = v0*v0/(2*G)
    return apo, expect, abs(apo-expect)/expect


def check_energy_coast():
    """No thrust, no drag (CD0=0, K_induced=0), no damping (C_mq=0), aero lift on: KE+PE conserved."""
    p = P(CD0=0.0, K_induced=0.0, C_mq=0.0)
    s = np.array([0, 50.0, 5.0, 5.0, np.deg2rad(3), 0.0])
    def E(s): return 0.5*p.m*(s[2]**2+s[3]**2) + p.m*G*s[1]
    E0 = E(s); t = 0.0
    while s[1] > 0 and t < 5:
        s = step(s, 0.0, 0.0, p, 0.0); t += DT
    return E0, E(s), abs(E(s)-E0)/E0


def check_weathercock():
    """Stable rocket (sm>0, no canard), launched with 10deg alpha, no control. Track ANGLE OF ATTACK
    (alignment with velocity), not theta. With pitch damping, alpha should DECAY toward 0 (the rocket
    aligns with its velocity = correctly statically+dynamically stable). theta itself rotates with the
    flight path (nose-up climb -> nose-down descent), which is correct weathercocking, not divergence."""
    # clean dynamic-stability test: steady DESCENT in the stable (nose-down) orientation, perturb alpha 10deg,
    # check it DECAYS (damped oscillation) rather than tumbling. Avoids the apogee velocity-reversal artifact.
    p = P(sm_cal=1.0, canard_NA=0.0)
    s = np.array([0, 80.0, 0.0, -20.0, np.pi + np.deg2rad(10), 0.0])   # nose-down + 10deg, falling
    t = 0.0; max_a = 0.0; alpha = 0.0
    while s[1] > 0 and t < 6:
        s = step(s, 0.0, 0.0, p, 0.0); t += DT
        gamma_v = np.arctan2(s[2], s[3]); alpha = np.rad2deg(_wrap_pi(s[4] - gamma_v))
        if t > 0.1:
            max_a = max(max_a, abs(alpha))
    return alpha, max_a    # final|alpha| should decay toward 0; max|alpha| bounded (~<=12)


def check_canard_descent():
    """Descent (falling), nose-up start (th~0, engine-down). WITHOUT canard a stable aft-fin rocket flips
    nose-DOWN (th->180); WITH enough canard deploy, CP moves forward -> nose-up (engine-down) becomes stable."""
    res = {}
    for deploy, tag in ((0.0, "no_canard"), (1.0, "canard_deployed")):
        # big canard so CP can move ahead of CG: canard_NA*arm > S*CN_alpha*sm*d
        p = P(sm_cal=0.8, canard_NA=0.02, canard_arm=0.30)
        s = np.array([0, 40.0, 0.0, -1.0, np.deg2rad(8), 0.0])   # falling, 8deg off nose-up
        t = 0.0
        while s[1] > 0 and t < 6:
            s = step(s, 0.0, 0.0, p, deploy); t += DT
        res[tag] = np.rad2deg(s[4])   # final theta: ~0 = stayed engine-down (good); ~180 = flipped nose-down
    return res


def f15(t):
    if t < 0 or t > 3.45:
        return 0.0
    if t < 0.15:
        return 25.0 * t / 0.15
    if t < 0.50:
        return 25.0 - (25.0 - 13.5) * ((t - 0.15) / 0.35)
    return 13.5


def simulate_flight(p, wind_vx=0.0, gust_t=None, gust_v=0.0, deploy_canard=True,
                    coast_control="passive", kp=0.6, kd=0.12, th0_deg=2.0, kpc=6.0, kdc=1.5,
                    alpha_bias_deg=0.0, alpha_noise_deg=0.0, theta_noise_deg=0.0, gyro_noise_dps=0.0,
                    canard_rate=99.0, gyro_bias_dps=0.0, latency_steps=0, seed=0):
    """Full boost->coast flight. Boost: TVC PD holds vertical (theta=0), canards RETRACTED.
    Coast (after burnout): TVC off. coast_control:
      'passive' -> canards deploy to a FIXED fraction (aero stability; tips into wind at apogee)
      'margin'  -> FEEDBACK-LINEARIZED margin control: modulate canard deploy so the aero pitch moment =
                   Iyy*(kpc*(0-theta)+kdc*(0-q)), i.e. actively hold vertical using margin modulation. Authority
                   ~ q_dyn*sin(alpha) (wind provides it); saturates (deploy clipped [0,1]) when too weak.
    Returns diagnostics incl. whether it ARRIVES engine-down at the ignition altitude."""
    BURN = 3.45
    s = np.array([0, 0, 0.0, 5.0, np.deg2rad(th0_deg), 0.0])    # rail exit: 5 m/s up, 2deg perturbation
    rng = np.random.default_rng(seed)
    t = 0.0; deploy = 0.0; apogee = 0.0
    th_hat = s[4]                                  # onboard attitude estimate (integrated gyro; DRIFTS w/ bias)
    cmd_hist = []                                  # (delta, dep_t) FIFO for control LATENCY
    th_boost_max = 0.0; th_apo = None; q_apo = None; reached_apo = False
    th_ign = None; a_ign = None; ign_alt = 8.0; sat_frac = 0.0; n_coast = 0
    while s[1] >= -0.05 and t < 30:
        thrust = f15(t)
        # onboard estimates: attitude integrated from BIASED gyro (drifts); rate measured w/ bias+noise
        th_hat += (s[5] + np.deg2rad(gyro_bias_dps)) * DT
        q_hat = s[5] + np.deg2rad(gyro_bias_dps) + np.deg2rad(gyro_noise_dps) * rng.standard_normal()
        th_use = th_hat + np.deg2rad(theta_noise_deg) * rng.standard_normal()
        if t < BURN:
            e = 0.0 - th_use
            delta = np.clip(kp * e + kd * (0.0 - q_hat), -p.delta_max, p.delta_max)
            dep_t = 0.0
            th_boost_max = max(th_boost_max, abs(np.rad2deg(s[4])))
        else:
            delta = 0.0
            if coast_control == "margin" and deploy_canard:
                # feedback-linearized margin control using ESTIMATED (biased+noisy+drifting) states
                wnow = wind_vx + (gust_v if (gust_t is not None and gust_t <= t < gust_t + 0.3) else 0.0)
                rvx, rvz = s[2] - wnow, s[3]
                V = np.hypot(rvx, rvz); gv = np.arctan2(rvx, rvz)
                alpha = _wrap_pi(s[4] - gv)
                a_hat = alpha + np.deg2rad(alpha_bias_deg) + np.deg2rad(alpha_noise_deg) * rng.standard_normal()
                sa = np.sin(a_hat)
                qd = 0.5 * RHO * V * V
                M_des = p.Iyy * (kpc * (0.0 - th_use) + kdc * (0.0 - q_hat))
                denom = qd * sa
                if abs(denom) > 1e-4:
                    dep_raw = (M_des / denom + p.S_ref * p.CN_alpha * p.sm_cal * p.d_ref) / (p.canard_NA * p.canard_arm)
                else:
                    dep_raw = 0.5                                  # no authority (alpha~0 / V~0): neutral-ish
                dep_t = float(np.clip(dep_raw, 0.0, 1.0))
                if t > BURN + 0.3:
                    n_coast += 1
                    if dep_raw < 0.0 or dep_raw > 1.0:
                        sat_frac += 1                              # controller wanted more than canards allow
            else:
                dep_t = 1.0 if deploy_canard else 0.0
        # control LATENCY: actuator gets a command delayed by latency_steps
        cmd_hist.append((delta, dep_t))
        if len(cmd_hist) > latency_steps:
            delta_a, dep_a = cmd_hist.pop(0)
        else:
            delta_a, dep_a = 0.0, deploy               # not enough history yet -> hold
        # canard deploy actuator: RATE-LIMITED in margin mode (real servo), ramped otherwise
        if coast_control == "margin" and t >= BURN:
            dmax = canard_rate * DT
            deploy += float(np.clip(dep_a - deploy, -dmax, dmax))
        else:
            deploy += (dep_a - deploy) * min(1.0, DT / 0.3)
        w = wind_vx + (gust_v if (gust_t is not None and gust_t <= t < gust_t + 0.3) else 0.0)
        s = step(s, thrust, delta_a, p, deploy, w)
        if s[1] > apogee:
            apogee = s[1]
        if (not reached_apo) and s[3] < 0 and t > BURN:        # just passed apogee (descending)
            reached_apo = True; th_apo = np.rad2deg(s[4]); q_apo = np.rad2deg(s[5])
        if reached_apo and th_ign is None and s[1] <= ign_alt: # crossed ignition altitude on descent
            gv = np.arctan2(s[2], s[3]); th_ign = np.rad2deg(s[4]); a_ign = np.rad2deg(_wrap_pi(s[4] - gv))
        t += DT
    return dict(apogee=apogee, th_boost_max=th_boost_max, th_apo=th_apo, q_apo=q_apo,
                th_ign=th_ign, alpha_ign=a_ign, sat_frac=sat_frac / max(n_coast, 1))


def flight_test():
    print("\n" + "-" * 74)
    print("  FULL FLIGHT (boost TVC-hold -> burnout -> canard-coast): APOGEE-CAPTURE CRUX")
    print("  Q: does it ARRIVE engine-down (theta~0) + low-rate at the ~8 m ignition altitude?")
    print("-" * 74)
    # engine-down-stable descent config: canard_NA*arm > S*CN_alpha*sm*d
    base = dict(m=1.0, Iyy=0.05, sm_cal=0.5, canard_NA=0.015, canard_arm=0.28, CN_alpha=8.0)
    cases = [
        ("calm, canards ON",      dict(wind_vx=0.0, gust_t=None, deploy_canard=True)),
        ("calm, canards OFF",     dict(wind_vx=0.0, gust_t=None, deploy_canard=False)),
        ("3 m/s wind, canards ON", dict(wind_vx=3.0, gust_t=None, deploy_canard=True)),
        ("apogee gust 4m/s, ON",  dict(wind_vx=0.0, gust_t=None, gust_v=4.0, deploy_canard=True)),
    ]
    def f(x): return f"{x:7.1f}" if x is not None else "   -   "
    def run(name, ctrl, **kw):
        p = P(**base)
        if kw.get('gust_v', 0.0) > 0:
            kw['gust_t'] = 3.6
        d = simulate_flight(p, coast_control=ctrl, **kw)
        print(f"  {name:28} {d['apogee']:6.1f} {f(d['th_apo']):>7} {f(d['q_apo']):>7} "
              f"{f(d['th_ign']):>7} {f(d['alpha_ign']):>7} {d['sat_frac']*100:5.0f}%")

    print(f"  {'case':28} {'apogee':>6} {'th@apo':>7} {'q@apo':>7} {'th@ign':>7} {'a@ign':>7} {'sat':>6}")
    print("  -- PASSIVE canard stability (fixed deploy): --")
    run("calm",              "passive", wind_vx=0.0)
    run("3 m/s wind",        "passive", wind_vx=3.0)
    run("apogee gust 4 m/s", "passive", wind_vx=0.0, gust_v=4.0)
    print("  -- ACTIVE MARGIN CONTROL in coast (feedback-linearized deploy holds vertical): --")
    run("calm",              "margin",  wind_vx=0.0)
    run("3 m/s wind",        "margin",  wind_vx=3.0)
    run("apogee gust 4 m/s", "margin",  wind_vx=0.0, gust_v=4.0)
    run("6 m/s wind",        "margin",  wind_vx=6.0)
    print("  th@ign ~0 (engine-down) + |a@ign|~180 (tail-first) = ARRIVES CONTROLLABLE.")
    print("  sat = % of coast where margin controller wanted MORE authority than canards allow (saturated).")
    print("  KEY: does ACTIVE MARGIN CONTROL hold engine-down in WIND where PASSIVE tips over?")


def _predict_td_vz(z0, vz0, m, CdA, tilt=0.0):
    """1-D vertical forward sim of the landing burn (F15 up, gravity, drag) from (z0, vz0<0).
    Returns touchdown vertical velocity. Used by the suicide-burn ignition scan."""
    z, vz, t = z0, vz0, 0.0
    while z > 0 and t < 12:
        F = f15(t) * np.cos(tilt)
        a = F / m - G - 0.5 * RHO * CdA * vz * abs(vz) / m
        vz += a * DT; z += vz * DT; t += DT
    return vz


def _best_ign_alt(apogee, m, CdA):
    """Scan ignition altitudes; pick the one whose burn best nulls touchdown vz (optimal suicide-burn)."""
    best_z, best = apogee * 0.5, -1e9
    for zc in np.linspace(apogee * 0.97, 0.5, 200):
        vzc = -np.sqrt(max(2 * G * (apogee - zc), 0.0))
        vp = _predict_td_vz(zc, vzc, m, CdA)
        score = -abs(vp) if vp <= 0.3 else -10 - vp
        if score > -abs(best):
            best = vp; best_z = zc
    return best_z


def simulate_full_landing(p, wind_vx=0.0, use_throttle=False, alpha_bias_deg=0.0, gyro_bias_dps=0.3,
                          canard_rate=2.5, seed=0, CdA=0.01, th0_deg=2.0, ascent_burn_s=3.45,
                          kp=0.6, kd=0.12, kpc=6.0, kdc=1.5, tilt_max_deg=30.0):
    """END-TO-END: boost (TVC hold) -> coast (MARGIN control holds engine-down) -> suicide-burn ignition
    -> landing burn (TVC attitude + optional body-tilt throttle) -> touchdown. Returns landing metrics."""
    BURN = 3.45
    s = np.array([0, 0, 0.0, 5.0, np.deg2rad(th0_deg), 0.0])
    rng = np.random.default_rng(seed)
    t = 0.0; deploy = 0.0; apogee = 0.0; th_hat = s[4]
    phase = "boost"; best_z = None; t_ign = None
    while s[1] >= -0.02 and t < 40:
        th_hat += (s[5] + np.deg2rad(gyro_bias_dps)) * DT
        q_hat = s[5] + np.deg2rad(gyro_bias_dps)
        thrust = 0.0; delta = 0.0; dep_t = deploy
        if phase == "boost":
            thrust = f15(t) if t < ascent_burn_s else 0.0   # ascent_burn_s<3.45 = smaller ascent motor (lower apogee)
            delta = np.clip(kp * (0 - th_hat) + kd * (0 - q_hat), -p.delta_max, p.delta_max)
            dep_t = 0.0
            if t >= ascent_burn_s:
                phase = "coast"
        elif phase == "coast":
            thrust = 0.0
            if apogee == 0.0 or s[1] > apogee - 0.5:
                pass
            if s[3] < 0 and best_z is None:                        # just past apogee -> plan ignition
                best_z = _best_ign_alt(apogee, p.m, CdA)
            # MARGIN control holds engine-down (theta=0)
            wnow = wind_vx
            rvx, rvz = s[2] - wnow, s[3]; V = np.hypot(rvx, rvz); gv = np.arctan2(rvx, rvz)
            alpha = _wrap_pi(s[4] - gv); a_hat = alpha + np.deg2rad(alpha_bias_deg)
            sa = np.sin(a_hat); qd = 0.5 * RHO * V * V
            M_des = p.Iyy * (kpc * (0 - th_hat) + kdc * (0 - q_hat)); denom = qd * sa
            dep_t = float(np.clip((M_des / denom + p.S_ref * p.CN_alpha * p.sm_cal * p.d_ref) /
                                  (p.canard_NA * p.canard_arm), 0, 1)) if abs(denom) > 1e-4 else 0.5
            if best_z is not None and s[1] <= best_z:
                phase = "burn"; t_ign = t
        elif phase == "burn":
            tb = t - t_ign
            thrust = f15(tb)
            th_cmd = 0.0
            if use_throttle and s[3] < 0 and s[1] > 0.1:
                a_req = s[3] * s[3] / (2 * s[1])                    # vertical decel to null at ground
                m = p.m
                c = (a_req + G) * m / max(thrust, 1.0)             # cos(tilt) for that vertical decel
                th_cmd = np.clip(np.arccos(np.clip(c, 0.0, 1.0)), 0, np.deg2rad(tilt_max_deg))
            delta = np.clip(kp * 3 * (th_cmd - th_hat) + kd * 3 * (0 - q_hat), -p.delta_max, p.delta_max)
            dep_t = deploy                                          # leave canards as-is during burn
        # canard actuator (rate-limited)
        dmax = canard_rate * DT
        deploy += float(np.clip(dep_t - deploy, -dmax, dmax)) if phase != "boost" else (dep_t - deploy)
        s = step(s, thrust, delta, p, deploy, wind_vx)
        apogee = max(apogee, s[1])
        t += DT
    vz_td, vx_td, th_td = s[3], s[2], np.rad2deg(_wrap_pi(s[4]))
    landed = abs(vz_td) < 2.5 and abs(s[2]) < 3.0 and abs(th_td) < 20
    return dict(apogee=apogee, ign_alt=best_z, vz_td=vz_td, vx_td=vx_td, th_td=th_td, landed=landed)


def full_landing_test():
    print("\n" + "-" * 78)
    print("  END-TO-END LANDING: boost -> margin-coast -> suicide-burn -> TVC burn -> touchdown")
    print("-" * 78)
    base = dict(m=1.0, Iyy=0.05, sm_cal=0.5, canard_NA=0.015, canard_arm=0.28, CN_alpha=8.0)
    def show(label, **kw):
        p = P(**base)
        d = simulate_full_landing(p, **kw)
        ia = f"{d['ign_alt']:.1f}" if d['ign_alt'] else "-"
        print(f"  {label:30} apo={d['apogee']:5.1f} ign={ia:>5} vz_td={d['vz_td']:6.1f} "
              f"vx_td={d['vx_td']:6.1f} th_td={d['th_td']:6.1f}  {'LANDED' if d['landed'] else 'crash'}")
    print(f"  (LANDED = |vz|<2.5 m/s AND |vx|<3 AND |theta|<20deg; vz/vx m/s, theta deg)")
    show("calm, no throttle",          wind_vx=0.0, use_throttle=False)
    show("calm, body-tilt throttle",   wind_vx=0.0, use_throttle=True)
    show("3 m/s wind, throttle",       wind_vx=3.0, use_throttle=True)
    print("  -- MATCHED-APOGEE sweep (reduce ascent impulse -> lower apogee -> F15 can null it): --")
    for ab in (3.45, 2.0, 1.5, 1.2, 1.0):
        p = P(**base)
        d = simulate_full_landing(p, wind_vx=0.0, use_throttle=False, ascent_burn_s=ab)
        ia = f"{d['ign_alt']:.1f}" if d['ign_alt'] else "-"
        print(f"  ascent_burn={ab:.2f}s: apo={d['apogee']:5.1f} ign={ia:>5} vz_td={d['vz_td']:6.1f} "
              f"th_td={d['th_td']:5.1f}  {'LANDED' if d['landed'] else 'crash'}")
    print("  -- best matched apogee + 3 m/s wind (the real test): --")
    for ab in (1.5, 1.2):
        p = P(**base)
        d = simulate_full_landing(p, wind_vx=3.0, use_throttle=False, ascent_burn_s=ab)
        print(f"  ascent_burn={ab:.2f}s + wind: apo={d['apogee']:5.1f} vz_td={d['vz_td']:6.1f} "
              f"th_td={d['th_td']:5.1f} vx_td={d['vx_td']:5.1f}  {'LANDED' if d['landed'] else 'crash'}")
    print("  NOTE: full F15 boost overshoots (62m) -> F15 can't null 35 m/s. MATCH the apogee to the motor")
    print("        (smaller ascent motor / heavier / drag) so descent v ~ F15 removable ~14 m/s. The attitude")
    print("        (th_td~0 engine-down) is solved throughout; vertical = matched-apogee problem, now shown.")


def realism_test():
    print("\n" + "-" * 74)
    print("  REALISM: margin control in 3 m/s wind with alpha-ESTIMATION error + sensor noise +")
    print("  canard servo RATE LIMIT. Q: how accurate must alpha_hat be to still arrive engine-down?")
    print("-" * 74)
    base = dict(m=1.0, Iyy=0.05, sm_cal=0.5, canard_NA=0.015, canard_arm=0.28, CN_alpha=8.0)
    # realistic sensor/actuator: canard full-deploy in ~0.4s (rate 2.5/s), alpha noise 1deg, gyro 2dps, theta 1deg
    NS = 6
    def avg(label, **kw):
        ths = []; sats = []
        for sd in range(NS):
            p = P(**base)
            d = simulate_flight(p, coast_control="margin", wind_vx=3.0, theta_noise_deg=1.0,
                                gyro_noise_dps=2.0, canard_rate=2.5, seed=sd, **kw)
            ti = d['th_ign']
            ths.append(abs(ti) if ti is not None else 180.0); sats.append(d['sat_frac'])
        m = np.mean(ths); w = np.max(ths)
        ok = "OK" if w < 20 else ("marginal" if w < 40 else "FAIL")
        print(f"  {label:30} {m:8.1f}d {w:10.1f}d {np.mean(sats)*100:5.0f}%  {ok}")
    print(f"  {'config (3 m/s wind, real sensors)':30} {'mean|th@ign|':>9} {'worst':>11} {'sat%':>6}")
    print("  -- (A) alpha-estimation BIAS sweep (gyro_bias=0, latency=0): --")
    for ab in (0, 2, 5, 8):
        avg(f"alpha_bias={ab}deg", alpha_bias_deg=ab, alpha_noise_deg=1.0)
    print("  -- (B) GYRO BIAS -> theta-drift sweep (alpha_bias=2deg, latency=0): --")
    for gb in (0.0, 0.5, 1.0, 2.0, 4.0):
        avg(f"gyro_bias={gb}dps (~{gb*6:.0f}deg drift)", alpha_bias_deg=2.0, alpha_noise_deg=1.0, gyro_bias_dps=gb)
    print("  -- (C) control LATENCY sweep (alpha_bias=2deg, gyro_bias=0.5dps): --")
    for ls in (0, 10, 20, 40):
        avg(f"latency={ls} steps ({ls*5}ms)", alpha_bias_deg=2.0, alpha_noise_deg=1.0,
            gyro_bias_dps=0.5, latency_steps=ls)
    print("  READ: (A) alpha tolerance ~5deg (you can do ~1.5). (B) theta-drift from gyro bias is the SETPOINT")
    print("        error -> watch where worst crosses 20deg. (C) latency: does the coast loop stay stable?")
    print("        Realistic: BMI088 gyro bias <~0.1dps (calibrated) -> <1deg drift; loop latency ~5-20ms.")


def main():
    print("=" * 74)
    print("  3-DOF LONGITUDINAL LANDING SIM -- LAYER 1 VALIDATION (sanity checks first)")
    print("=" * 74)

    apo, exp, err = check_ballistic()
    print(f"\n[1] BALLISTIC: apogee={apo:.2f} m, expected v0^2/2g={exp:.2f} m, error={err*100:.2f}%  "
          f"{'PASS' if err < 0.01 else 'FAIL'}")

    E0, E1, eerr = check_energy_coast()
    print(f"[2] ENERGY (coast, no drag): E0={E0:.1f} J -> E={E1:.1f} J, drift={eerr*100:.3f}%  "
          f"{'PASS' if eerr < 0.01 else 'FAIL'}")

    fa, ma = check_weathercock()
    print(f"[3] WEATHERCOCK (stable, no control): final|alpha|={abs(fa):.1f} deg, max|alpha|={ma:.1f} deg  "
          f"{'PASS (aligns w/ velocity, alpha decays)' if abs(fa) < 5 and ma < 20 else 'FAIL'}")

    cd = check_canard_descent()
    print(f"[4] CANARD DESCENT STABILITY (nose-up=engine-down=good):")
    print(f"      no_canard:        final theta = {cd['no_canard']:.1f} deg  "
          f"({'flipped nose-down' if abs(cd['no_canard'])>90 else 'stayed engine-down'})")
    print(f"      canard_deployed:  final theta = {cd['canard_deployed']:.1f} deg  "
          f"({'flipped nose-down' if abs(cd['canard_deployed'])>90 else 'stayed engine-down'})")
    print(f"      -> canard should KEEP it engine-down where no-canard FLIPS it. "
          f"{'PASS' if abs(cd['canard_deployed'])<abs(cd['no_canard']) else 'CHECK'}")

    print("\n  READ: if [1] and [2] PASS, the core dynamics/integrator are correct. [3]/[4] validate the")
    print("        aero moment signs (stability). Only THEN do we add boost/coast/land phases, ADRC,")
    print("        suicide-burn guidance, body-tilt throttle, and sensors. This is Layer 1 of the 6-DOF build.")
    flight_test()
    realism_test()
    full_landing_test()


if __name__ == "__main__":
    main()
