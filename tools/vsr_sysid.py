r"""
tools/vsr_sysid.py  (2026-06-28)  --  IDENTIFIABILITY KILL-TEST + MODEL-MISMATCH STRESS

THE QUESTION (make-or-break for the cheap-diagnostic-tool direction):
  From ONE powered flight + CHEAP sensors (MPU-6050-class IMU + BMP280-class baro), can we recover
      Iyy      (pitch moment of inertia)
      CN_alpha (normal-force slope)
      sm       (static margin, calibers)
  with CIs tight enough to be useful -- AND does it survive when the TRUE physics differs from the
  estimator's simple linear model (stall, TVC side force, sensor latency, servo backlash, geometry
  error)? The first pass passed in the FAVORABLE case; this version turns on the real-world errors.

IDENTIFIABILITY PATH (estimator's assumed model -- deliberately SIMPLE):
  Eq1 (moment; TVC moment is a KNOWN input):
      T*sin(delta)*L  =  Iyy*qdot  +  (q_dyn*S*d)*(CN_alpha*sm)*alpha  +  C_damp*q
  Eq2 (lateral force):   m*a_lat  =  q_dyn*S*CN_alpha*alpha   -> CN_alpha ;  sm = Pm/CN.
  alpha is reconstructed by strapdown dead-reckoning (IMU integrated, baro bounds vertical drift).

KEY PRIOR FINDING (favorable case): static margin is the best-determined parameter because the
  alpha-estimation error is COMMON to Pm (Eq1) and CN (Eq2) and CANCELS in sm = Pm/CN
  (corr(logPm,logCN)=+0.97). This run tests whether that cancellation SURVIVES model mismatch.

MISMATCH KNOBS (truth-side error the naive estimator does NOT model):
  stall      : truth uses post-stall CN loss ('drop'); estimator assumes linear aero
  side_force : truth adds T*sin(delta) translational side force; estimator ignores it
  latency    : IMU FIFO delay (steps); estimator does not compensate
  servo+cmd  : truth servo has deadband/backlash; estimator logs the COMMANDED gimbal, not achieved
  geom       : estimator uses L_nozzle off by +5% (CG/arm uncertainty -- dominant geometry error)

Run:  python tools/vsr_sysid.py
"""
from __future__ import annotations
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np

from sim_vsr import VehicleParams, TVCParams, TVCActuator, IMUParams, IMU, State, deriv
from sim_vsr.vehicle import mass_props, aero_forces, G

from sim_vsr.scenario import thrust_F_class, WindOU

try:
    from scipy.signal import savgol_filter
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False


# residual sensor noise tiers (per-sample at 200 Hz), AFTER on-pad bias calibration
TIERS = {
    "ideal":     dict(gyro_n=0.001, gyro_b=0.000, acc_n=0.001, acc_b=0.000, baro_n=0.01),
    "good":      dict(gyro_n=0.05,  gyro_b=0.02,  acc_n=0.03,  acc_b=0.01,  baro_n=0.3),
    "realistic": dict(gyro_n=0.15,  gyro_b=0.10,  acc_n=0.08,  acc_b=0.03,  baro_n=0.8),
    "poor":      dict(gyro_n=0.50,  gyro_b=0.50,  acc_n=0.30,  acc_b=0.10,  baro_n=2.0),
}

DT = 0.005
T_END = 2.1
SM_TRUE = 1.2          # cal
CN_TRUE = 8.0          # /rad


def _clean_cfg():
    return dict(stall_deg=None, stall_model="plateau", side_force=False,
               deadband=0.0, backlash=0.0, latency=0,
               est_use_cmd=False, est_L_scale=1.0, est_side_force=False,
               est_cd=False, cg_migrate=False)


def deriv_truth(s, g, sm, thrust, vp, pf, wa, side_force):
    """sim_vsr deriv (moment-only TVC) plus an optional TVC translational side force F=T*sin(g)
    perpendicular to the body axis (real rockets have it; the base sim omits it)."""
    d = deriv(s, g, sm, thrust, vp, pf, wa, canard=None, deploy_frac=0.0)
    if side_force:
        m, _ = mass_props(vp, pf); V = max(s.V, 0.2)
        alpha = (s.theta - s.gamma) + wa
        F = thrust * np.sin(g)
        d[3] += F * np.cos(alpha) / (m * V)     # gammadot  (perp to velocity)
        d[2] += -F * np.sin(alpha) / m          # Vdot      (along velocity, small)
    return d


def gen_truth(seed=0, excite=True, wind_sigma=1.0, cfg=None):
    cfg = cfg or _clean_cfg()
    vp = VehicleParams(stall_deg=cfg["stall_deg"], stall_model=cfg["stall_model"])
    rng = np.random.default_rng(seed)
    tvc = TVCActuator(TVCParams(u_max_deg=6.0, slew_max_deg_s=600,
                                deadband_deg=cfg["deadband"], backlash_deg=cfg["backlash"]))
    wind = WindOU(wind_sigma, 0.5, rng)
    s = State(V=22.0)
    t_end_run = cfg.get("t_end", T_END)
    coast = cfg.get("coast", False)
    n = int(t_end_run / DT)
    total_imp = max(sum(thrust_F_class(k * DT) for k in range(n)) * DT, 1e-6)
    cum = 0.0
    keys = ("t", "V", "gamma", "theta", "q", "qdot", "delta_ach", "delta_cmd", "T",
            "qdyn", "alpha", "ax_sf", "lat_sf", "z", "m", "Iyy", "sm_true", "L_true")
    A = {k: np.zeros(n) for k in keys}
    f_hz = np.array([0.8, 1.7, 3.1, 5.3]); amp = np.array([2.2, 1.6, 1.1, 0.7])
    u_max = np.deg2rad(6.0)
    L0 = vp.L_nozzle                       # pre-flight (builder-measured) moment arm

    for k in range(n):
        t = k * DT
        thrust = thrust_F_class(t)
        pf = max(0.0, 1.0 - cum / total_imp); cum += thrust * DT
        m, Iyy = mass_props(vp, pf)
        qd = 0.5 * vp.rho * s.V * s.V
        w = wind.step(DT); wa = np.arctan2(w, max(s.V, 2.0))

        ref_th = np.pi / 2 + np.deg2rad(6.0) * np.sin(2 * np.pi * 0.9 * t) if excite else np.pi / 2
        e = ref_th - s.theta
        hold = np.rad2deg(0.18 * e + 0.04 * (-s.q))
        probe = float(np.sum(amp * np.sin(2 * np.pi * f_hz * t))) if excite else 0.0
        gcmd = hold + probe
        if coast and thrust < 1.0:                              # burnout: TVC off -> free oscillation
            gcmd = 0.0
        kick = cfg.get("coast_kick", 0.0)                       # deliberate burnout doublet to ring up
        if coast and kick > 0.0 and 2.00 <= t < 2.14:           # still thrust here (burn ends ~2.2s)
            gcmd = kick if t < 2.07 else -kick
        g = tvc.step(gcmd, DT)                                  # achieved (rad)
        gcmd_log = float(np.clip(np.deg2rad(gcmd), -u_max, u_max))   # firmware-logged command

        # CG MIGRATION: as propellant burns (pf:1->0), CG moves forward -> moment arm L grows and
        # static margin grows. Off by default (current sim holds CG fixed; only m,Iyy vary).
        burned = 1.0 - pf
        migrate = cfg.get("cg_migrate", False)
        sm_t = SM_TRUE * (1.0 + 0.33 * burned) if migrate else SM_TRUE
        L_t = L0 * (1.0 + 0.12 * burned) if migrate else L0
        vp.L_nozzle = L_t                  # deriv reads p.L_nozzle for M_tvc

        alpha = (s.theta - s.gamma) + wa
        N, Drag, _ = aero_forces(vp, max(s.V, 0.2), alpha)
        d = deriv_truth(s, g, sm_t, thrust, vp, pf, wa, cfg["side_force"])
        Vdot, gammadot, qdot = d[2], d[3], d[5]
        aIx = Vdot * np.cos(s.gamma) - s.V * np.sin(s.gamma) * gammadot
        aIz = Vdot * np.sin(s.gamma) + s.V * np.cos(s.gamma) * gammadot
        fIx, fIz = aIx, aIz + G
        ax = fIx * np.cos(s.theta) + fIz * np.sin(s.theta)
        lat = fIx * (-np.sin(s.theta)) + fIz * np.cos(s.theta)

        A["t"][k] = t; A["V"][k] = s.V; A["gamma"][k] = s.gamma; A["theta"][k] = s.theta
        A["q"][k] = s.q; A["qdot"][k] = qdot; A["delta_ach"][k] = g; A["delta_cmd"][k] = gcmd_log
        A["T"][k] = thrust; A["qdyn"][k] = qd; A["alpha"][k] = alpha
        A["ax_sf"][k] = ax; A["lat_sf"][k] = lat; A["z"][k] = s.z; A["m"][k] = m; A["Iyy"][k] = Iyy
        A["sm_true"][k] = sm_t; A["L_true"][k] = L_t

        arr = s.to_array()
        f = lambda a: deriv_truth(State.from_array(a), g, sm_t, thrust, vp, pf, wa, cfg["side_force"])
        k1 = f(arr); k2 = f(arr + .5*DT*k1); k3 = f(arr + .5*DT*k2); k4 = f(arr + DT*k3)
        s = State.from_array(arr + DT/6*(k1 + 2*k2 + 2*k3 + k4))

    consts = dict(S=vp.S_ref, d=vp.d_ref, L=L0, rho=vp.rho, Iyy_true=A["Iyy"])
    return A, consts


def _shift(a, L):
    if L <= 0:
        return a
    out = np.empty_like(a); out[:L] = a[0]; out[L:] = a[:-L]; return out


def synth_sensors(A, tier, seed, latency=0):
    p = TIERS[tier]; rng = np.random.default_rng(10_000 + seed); n = len(A["t"])
    gb = np.deg2rad(p["gyro_b"]) * rng.standard_normal()
    axb = p["acc_b"] * rng.standard_normal(); latb = p["acc_b"] * rng.standard_normal()
    gyro = A["q"] + gb + np.deg2rad(p["gyro_n"]) * rng.standard_normal(n)
    ax = A["ax_sf"] + axb + p["acc_n"] * rng.standard_normal(n)
    lat = A["lat_sf"] + latb + p["acc_n"] * rng.standard_normal(n)
    baro = A["z"] + p["baro_n"] * rng.standard_normal(n)
    # IMU FIFO latency (estimator does NOT compensate)
    gyro = _shift(gyro, latency); ax = _shift(ax, latency); lat = _shift(lat, latency)
    return dict(gyro=gyro, ax=ax, lat=lat, baro=baro)


def estimate_nav(S, A, consts, use_cf=False, cf_gain=0.8):
    n = len(A["t"])
    theta = np.zeros(n); Vx = np.zeros(n); Vz = np.zeros(n); z = np.zeros(n)
    theta[0] = A["theta"][0]
    V0 = A["V"][0]; Vx[0] = V0 * np.cos(A["gamma"][0]); Vz[0] = V0 * np.sin(A["gamma"][0])
    Kp_b, Kv_b = 1.2, 0.6
    for k in range(1, n):
        theta[k] = theta[k-1] + S["gyro"][k-1] * DT
        c, s_ = np.cos(theta[k-1]), np.sin(theta[k-1])
        fIx = S["ax"][k-1] * c - S["lat"][k-1] * s_
        fIz = S["ax"][k-1] * s_ + S["lat"][k-1] * c
        Vx[k] = Vx[k-1] + fIx * DT
        Vz[k] = Vz[k-1] + (fIz - G) * DT
        z[k] = z[k-1] + Vz[k-1] * DT
        res = S["baro"][k] - z[k]
        z[k] += Kp_b * res * DT; Vz[k] += Kv_b * res
        if use_cf and Vx[k] > 1.0:
            # complementary filter: slowly pull theta_hat toward the flight-path angle (assumes
            # mean AoA ~ 0). SLOW gain -> corrects DC gyro/dead-reckoning drift, preserves the AC
            # oscillation/weave (and thus the within-window dynamics the margin fit needs).
            gamma_k = np.arctan2(Vz[k], Vx[k])
            theta[k] += cf_gain * DT * (gamma_k - theta[k])
    V = np.sqrt(Vx**2 + Vz**2); gamma = np.arctan2(Vz, Vx)
    qdyn = 0.5 * consts["rho"] * V * V
    if _HAVE_SCIPY and n > 13:
        qdot = savgol_filter(S["gyro"], 13, 3, deriv=1, delta=DT)
    else:
        sm = np.convolve(S["gyro"], np.ones(7)/7, mode="same"); qdot = np.gradient(sm, DT)
    return dict(theta=theta, V=V, gamma=gamma, alpha=theta - gamma, qdyn=qdyn, qdot=qdot,
                q=S["gyro"], ax=S["ax"])


def regress(nav, A, consts, win, delta, geomL=1.0, est_side=False, est_cd=False):
    S, d, L = consts["S"], consts["d"], consts["L"] * geomL
    sl = win
    qdot = nav["qdot"][sl]; alpha = nav["alpha"][sl]; q = nav["q"][sl]
    qdyn = nav["qdyn"][sl]; dlt = delta[sl]; T = A["T"][sl]; m = A["m"][sl]
    y1 = T * np.sin(dlt) * L
    X1 = np.column_stack([qdot, (qdyn * S * d) * alpha, q])
    b1, *_ = np.linalg.lstsq(X1, y1, rcond=None)
    Iyy_hat, Pm_hat, Cdamp = b1
    a_lat = A["_lat_meas"][sl] - (T * np.sin(dlt) / m if est_side else 0.0)
    y2 = m * a_lat
    CD_hat = np.nan
    if est_cd:
        # estimate drag from the AXIAL channel: D = T - m*a_ax ; CD = D/(q*S). Subtract the drag
        # projection (q*S*CD*sin a) from the lateral channel so the lift slope is clean CN.
        D = T - m * nav["ax"][sl]
        with np.errstate(divide="ignore", invalid="ignore"):
            CD_step = np.where(qdyn * S > 1e-9, D / (qdyn * S), 0.0)
        CD_step = np.clip(CD_step, 0.0, 3.0)
        CD_hat = float(np.median(CD_step))
        y2 = y2 - qdyn * S * CD_step * np.sin(alpha)
        X2 = (qdyn * S * alpha * np.cos(alpha)).reshape(-1, 1)
    else:
        X2 = (qdyn * S * alpha).reshape(-1, 1)
    CN_hat = float(np.linalg.lstsq(X2, y2, rcond=None)[0][0])
    sm_hat = Pm_hat / CN_hat if abs(CN_hat) > 1e-6 else np.nan
    return dict(Iyy=Iyy_hat, Pm=Pm_hat, CN=CN_hat, sm=sm_hat, CD=CD_hat, cond1=np.linalg.cond(X1))


def run_case(tier, cfg, trials=40, excite=True, oracle=False, wind_sigma=1.0):
    out = {k: [] for k in ("Iyy", "CN", "sm", "cond1", "aoa_rms")}
    rng_geom = np.random.default_rng(777)
    for tr in range(trials):
        A, consts = gen_truth(seed=tr, excite=excite, wind_sigma=wind_sigma, cfg=cfg)
        S = synth_sensors(A, tier, tr, latency=cfg["latency"])
        if oracle:
            A["_lat_meas"] = A["lat_sf"]
            nav = dict(theta=A["theta"], V=A["V"], gamma=A["gamma"], alpha=A["alpha"],
                       qdyn=A["qdyn"], qdot=A["qdot"], q=A["q"], ax=A["ax_sf"])
        else:
            A["_lat_meas"] = S["lat"]
            nav = estimate_nav(S, A, consts, use_cf=cfg.get("use_cf", False))
        win = slice(int(0.15 / DT), int(1.9 / DT))
        delta = A["delta_cmd"] if cfg["est_use_cmd"] else A["delta_ach"]
        # geometry-L error: fixed scale, or random sign per flight (worst-case stacking test)
        gL = cfg["est_L_scale"]
        if cfg.get("est_L_random", False):
            gL = float(rng_geom.choice([0.90, 0.95, 1.05, 1.10]))
        r = regress(nav, A, consts, win, delta, geomL=gL,
                    est_side=cfg.get("est_side_force", False), est_cd=cfg.get("est_cd", False))
        Iyy_ref = float(np.mean(consts["Iyy_true"][win]))
        out["Iyy"].append(r["Iyy"] / Iyy_ref)
        out["CN"].append(r["CN"] / CN_TRUE)
        out["sm"].append(r["sm"])            # absolute cal
        out["cond1"].append(r["cond1"])
        # angle-of-attack reconstruction accuracy (deg RMS over window), nav vs truth
        out["aoa_rms"].append(float(np.sqrt(np.mean(
            (np.rad2deg(nav["alpha"][win] - A["alpha"][win])) ** 2))))
    return out


def track_margin_cg(tier, cfg, trials=30, win_s=0.5, step_s=0.25, oracle=False):
    """Sliding-window margin(t): does the tool recover a MOVING CG (rising margin) over the burn?
    oracle=True feeds TRUE states (no dead-reckoning drift) but estimator still uses fixed L0 ->
    isolates the geometry-coupling effect from the nav-drift effect.
    Returns window-center times, true margin(t), estimated margin(t) (median over trials), thrust(t)."""
    centers, t_true, t_est = None, None, None
    est_runs = []
    for tr in range(trials):
        A, consts = gen_truth(seed=tr, excite=True, cfg=cfg)
        S = synth_sensors(A, tier, tr, latency=cfg["latency"])
        if oracle:
            A["_lat_meas"] = A["lat_sf"]
            nav = dict(theta=A["theta"], V=A["V"], gamma=A["gamma"], alpha=A["alpha"],
                       qdyn=A["qdyn"], qdot=A["qdot"], q=A["q"], ax=A["ax_sf"])
        else:
            A["_lat_meas"] = S["lat"]
            nav = estimate_nav(S, A, consts, use_cf=cfg.get("use_cf", False))
        delta = A["delta_cmd"] if cfg["est_use_cmd"] else A["delta_ach"]
        n = len(A["t"]); wlen = int(win_s / DT); wstep = int(step_s / DT)
        starts = range(int(0.15 / DT), n - wlen, wstep)
        cs, tt, te, thr = [], [], [], []
        for s0 in starts:
            sl = slice(s0, s0 + wlen)
            r = regress(nav, A, consts, sl, delta, geomL=cfg["est_L_scale"],
                        est_side=cfg.get("est_side_force", False), est_cd=cfg.get("est_cd", False))
            cs.append((s0 + wlen / 2) * DT)
            tt.append(float(np.mean(A["sm_true"][sl])))
            te.append(r["sm"])
            thr.append(float(np.mean(A["T"][sl])))
        centers = np.array(cs); t_true = np.array(tt); est_runs.append(te); thrw = np.array(thr)
    t_est = np.nanmedian(np.array(est_runs, float), axis=0)
    return centers, t_true, t_est, thrw


def passive_margin(nav, consts, coast_slice, CN_est, Iyy_est):
    """Margin from the COAST oscillation FREQUENCY (a drift-IMMUNE AC measurement -- a slow sensor
    bias does not change an oscillation period). Free-pitch natural frequency:
        omega_n^2 = q_dyn*S*CN_alpha*sm*d / Iyy   ->   Pm = CN*sm = omega_n^2 * Iyy / (q_dyn*S*d)
    Fits a damped sine to the DETRENDED coast gyro; q_dyn from mean coast V."""
    S, d = consts["S"], consts["d"]
    sl = coast_slice
    y = np.asarray(nav["q"][sl], float)
    t = np.arange(len(y)) * DT
    if len(y) < 20:
        return dict(omega=np.nan, Pm=np.nan, sm=np.nan, f_hz=np.nan)
    y = y - np.polyval(np.polyfit(t, y, 1), t)          # remove drift+mean -> frequency is bias-immune
    if np.std(y) < 1e-7:
        return dict(omega=np.nan, Pm=np.nan, sm=np.nan, f_hz=np.nan)
    Y = np.abs(np.fft.rfft(y * np.hanning(len(y)))); freqs = np.fft.rfftfreq(len(y), DT)
    band = (freqs > 0.5) & (freqs < 15)
    w0 = 2 * np.pi * (freqs[band][np.argmax(Y[band])] if band.any() else 2.0)
    def model(tt, a, b, g, w): return np.exp(-g * tt) * (a * np.cos(w * tt) + b * np.sin(w * tt))
    w = w0
    try:
        from scipy.optimize import curve_fit
        p, _ = curve_fit(model, t, y, p0=[np.std(y), np.std(y), 1.0, w0], maxfev=8000)
        if 2 * np.pi * 0.4 < abs(p[3]) < 2 * np.pi * 18:
            w = abs(p[3])
    except Exception:
        pass
    qd = float(np.mean(nav["qdyn"][sl]))
    Pm = w * w * Iyy_est / max(qd * S * d, 1e-9)
    sm = Pm / CN_est if abs(CN_est) > 1e-6 else np.nan
    return dict(omega=w, Pm=Pm, sm=sm, f_hz=w / (2 * np.pi))


def pct(a, p):
    a = np.asarray(a); a = a[np.isfinite(a)]; return float(np.percentile(a, p))


def med(a):
    a = np.asarray(a); a = a[np.isfinite(a)]; return float(np.median(a))


def main():
    print("=" * 84)
    print("  MODEL-MISMATCH STRESS: cheap IMU+baro sys-ID with real-world errors (tier=realistic)")
    print("=" * 84)
    print(f"  True: Iyy~0.0165, CN_alpha=8.0, static margin=1.20 cal.  40 seeds/case.")
    print(f"  Iyy/CN as ratio est/true (1.00=perfect). MARGIN in absolute cal (true 1.20).")
    print(f"  Estimator is NAIVE: linear aero, no latency comp, nominal geometry unless noted.\n")

    cases = []
    c = _clean_cfg(); cases.append(("baseline (clean)", c))
    c = _clean_cfg(); c["stall_deg"] = 8.0; c["stall_model"] = "drop"; cases.append(("+stall (drop, 8deg)", c))
    c = _clean_cfg(); c["side_force"] = True; cases.append(("+TVC side force (naive)", c))
    c = _clean_cfg(); c["side_force"] = True; c["est_side_force"] = True
    cases.append(("+side force (est corrects)", c))
    c = _clean_cfg(); c["latency"] = 2; cases.append(("+IMU latency (2 steps)", c))
    c = _clean_cfg(); c["deadband"] = 0.2; c["backlash"] = 0.3; c["est_use_cmd"] = True
    cases.append(("+servo & cmd-gimbal", c))
    c = _clean_cfg(); c["est_L_scale"] = 1.05; cases.append(("+geometry L +5%", c))
    c = dict(stall_deg=8.0, stall_model="drop", side_force=True, deadband=0.2, backlash=0.3,
             latency=2, est_use_cmd=True, est_L_scale=1.05, est_side_force=True)
    cases.append(("ALL (est corrects side)", c))
    c = dict(stall_deg=8.0, stall_model="drop", side_force=True, deadband=0.2, backlash=0.3,
             latency=2, est_use_cmd=True, est_L_scale=1.05, est_side_force=False)
    cases.append(("ALL (naive side)", c))

    hdr = f"  {'case':24} {'Iyy ratio':>17} {'CN ratio':>17} {'margin [cal]':>22}"
    print("-" * 84); print(hdr); print("-" * 84)
    for name, cfg in cases:
        o = run_case("realistic", cfg, trials=40)
        iyy = f"{med(o['Iyy']):.2f}[{pct(o['Iyy'],5):.2f},{pct(o['Iyy'],95):.2f}]"
        cn = f"{med(o['CN']):.2f}[{pct(o['CN'],5):.2f},{pct(o['CN'],95):.2f}]"
        smc = f"{med(o['sm']):.2f} [{pct(o['sm'],5):.2f},{pct(o['sm'],95):.2f}]"
        half = 0.5 * (pct(o['sm'], 95) - pct(o['sm'], 5))
        useful = abs(med(o['sm']) - 1.2) <= 0.2 and half <= 0.2
        flag = "  OK" if useful else "  <-- BUSTS bar"
        print(f"  {name:24} {iyy:>17} {cn:>17} {smc:>22}{flag}")
    print("-" * 84)
    print("  USEFULNESS BAR: |median margin - 1.20| <= 0.2 cal  AND  90%-band half-width <= 0.2 cal.")
    print("  READ: if 'ALL combined' keeps margin inside the bar, the favorable-case pass was REAL")
    print("        and the tool survives model mismatch. If a single row busts it, that's the killer.\n")

    # ---- GEOMETRY-SIGN ROBUSTNESS: is the 0.07cal 'ALL combined' real or lucky offset? ----
    print("-" * 84)
    print("  GEOMETRY-SIGN ROBUSTNESS (realistic noise; side force ON + est-corrects).")
    print("  Does margin survive L error of BOTH signs, and random per-flight? (no lucky offset)")
    print("-" * 84)
    print(f"  {'L scale':>10} {'margin [cal]':>24} {'err':>6}")
    for L in (0.90, 0.95, 1.00, 1.05, 1.10):
        c = dict(stall_deg=8.0, stall_model="drop", side_force=True, deadband=0.2, backlash=0.3,
                 latency=2, est_use_cmd=True, est_L_scale=L, est_side_force=True, est_L_random=False)
        o = run_case("realistic", c, trials=40)
        print(f"  {L:>10.2f} {med(o['sm']):.2f} [{pct(o['sm'],5):.2f},{pct(o['sm'],95):.2f}]"
              f"{'':6} {abs(med(o['sm'])-1.2):.2f}")
    c = dict(stall_deg=8.0, stall_model="drop", side_force=True, deadband=0.2, backlash=0.3,
             latency=2, est_use_cmd=True, est_L_scale=1.0, est_side_force=True, est_L_random=True)
    o = run_case("realistic", c, trials=60)
    print(f"  {'random+-':>10} {med(o['sm']):.2f} [{pct(o['sm'],5):.2f},{pct(o['sm'],95):.2f}]"
          f"{'':6} {abs(med(o['sm'])-1.2):.2f}  <- worst-case stacking")
    print("  READ: if all rows stay within ~0.2-0.3 cal, the 0.07 was NOT a lucky offset -- margin")
    print("        error tracks |L error| linearly (~1:1) and there is no catastrophic cancellation loss.\n")

    # ---- ANGLE-OF-ATTACK reconstruction accuracy (the 'for fun' output) ----
    print("-" * 84)
    print("  ANGLE-OF-ATTACK reconstruction accuracy (nav alpha_hat vs truth, deg RMS over window)")
    print("-" * 84)
    cc = _clean_cfg()
    for tier in ("ideal", "good", "realistic", "poor"):
        o = run_case(tier, cc, trials=40)
        print(f"  {tier:10}  alpha RMS error = {med(o['aoa_rms']):.2f} deg "
              f"[{pct(o['aoa_rms'],5):.2f}, {pct(o['aoa_rms'],95):.2f}]")
    print("  NOTE: alpha_hat is the WORST output (it is what makes CN_alpha noisy). Static margin is")
    print("        accurate DESPITE poor alpha because the alpha error cancels in sm = Pm/CN.\n")

    # ---- FLOOR FIX: estimate CD from the axial channel -> removes the systematic -0.12 cal floor ----
    print("-" * 84)
    print("  FLOOR FIX: co-estimate drag CD from the axial accelerometer (removes the -0.12cal floor)")
    print("  Why +10%L looked 'best': two errors cancelled. Fix the model so margin is honest at L=1.0")
    print("-" * 84)
    print(f"  {'L scale':>9} {'naive margin':>16} {'+CD-corrected':>16} {'CD est':>10} (true CD=0.55)")
    for L in (0.90, 1.00, 1.10):
        cn_ = _clean_cfg(); cn_["est_L_scale"] = L
        cc_ = _clean_cfg(); cc_["est_L_scale"] = L; cc_["est_cd"] = True
        on = run_case("realistic", cn_, trials=40)
        oc_full = run_case("realistic", cc_, trials=40)
        # grab CD est separately (run_case doesn't return CD; quick recompute on one rep)
        A, consts = gen_truth(seed=0, cfg=cc_); S = synth_sensors(A, "realistic", 0); A["_lat_meas"] = S["lat"]
        nav = estimate_nav(S, A, consts); win = slice(int(0.15/DT), int(1.9/DT))
        rcd = regress(nav, A, consts, win, A["delta_ach"], geomL=L, est_cd=True)
        print(f"  {L:>9.2f} {med(on['sm']):.2f} [{pct(on['sm'],5):.2f},{pct(on['sm'],95):.2f}]"
              f"  {med(oc_full['sm']):.2f} [{pct(oc_full['sm'],5):.2f},{pct(oc_full['sm'],95):.2f}]"
              f"  {rcd['CD']:>8.2f}")
    print("  READ: with CD correction, margin at L=1.0 should sit near the TRUE 1.20 (floor gone), and")
    print("        geometry error then shows HONESTLY & symmetrically (no lucky cancellation). CD~0.55")
    print("        is a free bonus -> drives apogee prediction.\n")

    # ---- MOVING CG: can the tool track a CG that migrates forward during the burn? ----
    print("-" * 84)
    print("  MOVING CG: sliding-window margin(t) when the CG migrates forward over the burn")
    print("  (current base sim holds CG FIXED; this turns on margin 1.20->~1.60 + arm growth)")
    print("-" * 84)
    cfg = _clean_cfg(); cfg["cg_migrate"] = True; cfg["est_cd"] = True
    centers, t_true, t_est_s, thrw = track_margin_cg("realistic", cfg, trials=30, oracle=False)
    _,     _,      t_est_o, _      = track_margin_cg("realistic", cfg, trials=30, oracle=True)
    # analytic L-coupling-only prediction: fixed-L0 estimator -> margin_est = margin_true / (1+0.12*burned)
    burned = (t_true / SM_TRUE - 1.0) / 0.33           # invert sm_true = 1.2*(1+0.33*burned)
    t_pred_Lc = t_true / (1.0 + 0.12 * burned)
    print(f"  {'t [s]':>7} {'thrust':>7} {'TRUE':>6} {'L-couple':>9} {'oracle':>7} {'sensor':>7}")
    for c, th, tt, tp, to, ts in zip(centers, thrw, t_true, t_pred_Lc, t_est_o, t_est_s):
        print(f"  {c:>7.2f} {th:>7.1f} {tt:>6.2f} {tp:>9.2f} {to:>7.2f} {ts:>7.2f}")
    g = np.isfinite(t_est_o) & np.isfinite(t_est_s)
    sl = lambda y: np.polyfit(centers[g], np.asarray(y)[g], 1)[0]
    print(f"  d(margin)/dt:  TRUE={sl(t_true):+.2f}   L-couple-pred={sl(t_pred_Lc):+.2f}   "
          f"ORACLE={sl(t_est_o):+.2f}   SENSOR={sl(t_est_s):+.2f}  cal/s")
    print("  DECISIVE READ:")
    print("   - thrust ~constant (my 'thrust-fade' story was WRONG).")
    print("   - if ORACLE slope ~ L-couple-pred (+, attenuated) and SENSOR slope ~0 -> DRIFT is the killer,")
    print("     geometry-coupling only attenuates. (My 'L-coupling dominant' claim would then ALSO be wrong.)")
    print("   - if ORACLE slope ~0 too -> geometry-coupling is the killer (fixed-L0 masks the rise).")


if __name__ == "__main__":
    main()
