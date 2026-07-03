"""
tools/landing_animate.py  (2026-06-28)

Self-contained HTML animations of the propulsive-landing sequence, built on the VALIDATED 3-DOF
dynamics (tools/landing_sim3dof). Two outputs:

  outputs/landing_animation.html          -- full sequence: boost -> margin-controlled coast ->
                                             suicide-burn -> TVC landing burn -> touchdown.
  outputs/landing_pinpoint_animation.html -- pinpoint/divert: the rocket goes up & downrange, then
                                             MARGIN MODULATION alone performs a retrograde maneuver
                                             mid-coast (body tilts, ONLY the canards move, nozzle off)
                                             to steer back over the pad, then lands.

Animated moving parts: body attitude, TVC nozzle gimbal (delta), forward CANARDS extend/retract
(static-margin modulation), the live CG (white) and CP (colored) markers so you can SEE the margin
track the CG, and the exhaust flame during the two burns. Open in any browser; play/scrub/speed.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from tools.landing_sim3dof import P, step, f15, _wrap_pi, _best_ign_alt, G, RHO, DT


def sm_eff(p, deploy):
    """NET static margin [cal] = force-weighted CP of aft fins + deployed canards, about the CG.
    aft fins (slope N_a = S_ref*CN_alpha) act sm_cal*d BEHIND the CG (+, stabilizing); deployed canards
    (slope N_c = canard_NA*deploy) act canard_arm AHEAD of the CG (-, destabilizing). The net CP is the
    force-weighted mean, so the canard ALSO enters the denominator (total normal force) -- omitting that
    is what made the old marker blow up to -18 cal. deploy=0 -> sm_cal; full deploy -> ~ -1.2 cal."""
    Na = p.S_ref * p.CN_alpha
    Nc = p.canard_NA * deploy
    den = (Na + Nc) * p.d_ref
    return (Na * p.sm_cal * p.d_ref - Nc * p.canard_arm) / den if abs(den) > 1e-12 else p.sm_cal


# --- Estes F15 spec-sheet masses (thrustcurve.org): total 102 g, propellant 60 g ---
F15_PROP_KG = 0.060
MOTOR_ARM = 0.22          # m: propellant CG located this far AFT of the dry-rocket CG


def cg_migration(p, ascent_frac, landing_frac):
    """Update mass / Iyy / L_nozzle / static margin for propellant burned (CG migrates FORWARD as the
    aft propellant is consumed). Returns cg_aft (m, +=aft of dry CG) for the animation marker.
    ascent_frac, landing_frac in [0,1] = fraction of each motor's propellant REMAINING."""
    m_prop_now = F15_PROP_KG * (ascent_frac + landing_frac)
    m = p.m_dry + m_prop_now
    cg_aft = (F15_PROP_KG * (ascent_frac + landing_frac) * MOTOR_ARM) / m   # CG offset aft of dry CG
    p.m = m
    p.L_nozzle = p.L_dry - cg_aft                              # nozzle->CG arm grows as CG moves forward
    p.sm_cal = p.sm_dry - cg_aft / p.d_ref                     # margin grows (more stable) as prop burns
    p.Iyy = p.Iyy_dry + F15_PROP_KG * (ascent_frac + landing_frac) * MOTOR_ARM ** 2
    return cg_aft


def run_landing(p, ascent_scale=0.7, ascent_burn_s=3.45, boost_tilt_deg=0.0, wind_vx=0.0,
                pinpoint=False, flip_frac=0.7, ign_alt_fixed=8.0, ign_scale=1.0, land_burn_s=3.45,
                gyro_bias_dps=0.2, seed=0, kp=0.6, kd=0.12, kpc=6.0, kdc=1.5,
                x_target=0.0, perfect=True, kpx=1.5, kdx=2.2, lean_max_deg=35.0, cg_migrate=False,
                ign_alt_abs=None, v0=5.0, const_thrust=None, mirror_maneuver=False, descent_tilt_deg=None,
                grav_turn=False, kick_deg=8.0, kick_time=0.6, kick_dur=0.4, descent_hold_deg=0.0,
                live_ign=False, ign_margin=1.0, flip_kick_dur=0.0, flip_kick_to_deg=-45.0):
    """Logged landing with CG MIGRATION (F15 spec masses; CG moves forward as propellant burns).
      boost_tilt_deg>0 -> TVC tilts the BOOST (lateral accel) => the rocket flies a PARABOLA (TVC translates).
      pinpoint=True    -> coast: margin-mod (canards, nozzle off) holds engine-down ascending then rotates to
                          retrograde descending. Landing burn = CLOSED-LOOP TERMINAL GUIDANCE: aims the
                          fixed-thrust vector to drive (x -> x_target, vx -> 0) critically damped (no overshoot);
                          the suicide-burn ignition altitude handles soft vertical touchdown.
      pinpoint=False   -> vertical hop; margin-mod holds engine-down; guidance holds CENTERLINE (x_target=0).
      perfect=True     -> PERFECT/NOMINAL STATE for algorithm verification: zero gyro bias, zero initial tilt,
                          no tip-off, no wind. Use this to confirm the trajectory is physically exact, THEN set
                          perfect=False (+wind_vx, +gyro_bias_dps) to test disturbance rejection.
    Ascent = ascent_scale x F15 booster (scale<1 -> lower, landable apogee)."""
    p.m_dry = p.m - 2 * F15_PROP_KG          # dry = liftoff minus both motors' propellant
    p.L_dry = p.L_nozzle; p.sm_dry = p.sm_cal; p.Iyy_dry = p.Iyy
    gbias = 0.0 if perfect else gyro_bias_dps                 # perfect state: no sensor bias
    th0 = 0.0 if perfect else np.deg2rad(2.0)                 # perfect state: no initial tilt
    lean_max = np.deg2rad(lean_max_deg)
    s = np.array([0.0, 0.0, 0.0, v0, th0, 0.0])
    th_hat = s[4]; t = 0.0; deploy = 0.0; apogee = 0.0; th_ret = 0.0; z_burnout = 0.0
    phase = "boost"; best_z = None; t_ign = None; flipped = False; bounced = False; cut = False
    th_hist = []; burn_step = 0     # ascent body-angle history -> replayed time-reversed on descent (true mirror)
    th_hold = None                  # FIXED descent attitude: set once at burnout, held through coast + burn
    h_stop = 1e9                    # live suicide-burn stopping distance (set each coast step)
    LOG = {k: [] for k in ("t", "x", "z", "th", "gim", "dep", "marg", "thr", "phase", "vz", "vx", "cg")}
    while s[1] >= -0.02 and t < 22:
        if cg_migrate:   # CG migrates forward as propellant burns (off by default -- ignore per current scope)
            af = max(0.0, 1.0 - t / ascent_burn_s) if t < ascent_burn_s else 0.0
            lf = max(0.0, 1.0 - (t - t_ign) / land_burn_s) if (phase == "burn") else 1.0
            cg_aft = cg_migration(p, af, lf)
        else:
            cg_aft = 0.0                                      # constant mass/inertia (no CG migration)
        th_hat += (s[5] + np.deg2rad(gbias)) * DT
        q_hat = s[5] + np.deg2rad(gbias)
        thrust = 0.0; delta = 0.0; dep_t = deploy
        if phase == "boost":
            thr_full = const_thrust if const_thrust is not None else f15(t)   # ideal: constant thrust (time-symmetric)
            thrust = thr_full * ascent_scale if t < ascent_burn_s else 0.0
            if grav_turn:
                # GRAVITY TURN: rise vertical, a brief pitch KICK starts the turn, then the body FOLLOWS the
                # velocity vector (zero angle of attack) so gravity bends the trajectory over -- the natural,
                # efficient, symmetric ascent (mirrors the retrograde-following descent). Kick sets downrange.
                Vb = np.hypot(s[2], s[3])
                if t < kick_time:           th_cmd = 0.0
                elif t < kick_time + kick_dur: th_cmd = np.deg2rad(kick_deg)
                else:                       th_cmd = np.arctan2(s[2], s[3]) if Vb > 2.0 else np.deg2rad(kick_deg)
            else:
                th_cmd = np.deg2rad(boost_tilt_deg)           # fixed-angle hold (vertical case: 0)
            # END-OF-ASCENT FLIP KICK: in the last flip_kick_dur s of the burn (thrust still ON -> strong TVC
            # authority) command a pitch-DOWN toward retrograde, building rotation rate so the weak canards inherit
            # the flip already moving -> a clean, fast margin rotation (no slow start, no canard slam from rest).
            if flip_kick_dur > 0.0 and t > ascent_burn_s - flip_kick_dur:
                th_cmd = np.deg2rad(flip_kick_to_deg)
            delta = float(np.clip(kp * (th_cmd - th_hat) + kd * (0 - q_hat), -p.delta_max, p.delta_max))
            dep_t = 0.0
            if t >= ascent_burn_s:
                phase = "coast"; z_burnout = s[1]             # mirror point: descent burn re-ignites here
                th_hold = np.deg2rad(descent_hold_deg)        # the ONE fixed descent attitude (set right at burnout)
                if pinpoint and not perfect:
                    s[5] += np.deg2rad(5.0)                # tip-off only in disturbance mode (perfect: none)
        elif phase == "coast":
            # LIVE SUICIDE-BURN TRIGGER (pure math, no sweep, onboard-computable, course-adaptive): a retrograde
            # burn decelerates at a_net = T/m - g, so the altitude needed to stop from the current descent speed is
            # h_stop = vz^2 / (2*a_net). Ignite when remaining altitude <= h_stop. Uses ONLY the current state
            # (z, vz, T, m) re-evaluated every step -> if the rocket is perturbed it still ignites at the right
            # altitude for its actual velocity. (Equivalent to the closed-form z=g*m*apogee/T but state-based.)
            Tnow = const_thrust if const_thrust is not None else f15(0.3)
            a_net = max(Tnow * np.cos(_wrap_pi(s[4])) / p.m - G, 0.2)   # VERTICAL decel = T*cos(tilt)/m - g
            h_stop = ign_margin * (s[3] ** 2) / (2.0 * a_net)
            if s[3] < 0 and best_z is None:
                if mirror_maneuver:        best_z = z_burnout       # ignite at the mirror (ascent-burnout) altitude
                elif ign_alt_abs is not None: best_z = ign_alt_abs
                elif const_thrust is not None:                       # closed-form suicide-burn altitude (vacuum, const
                    best_z = ign_scale * G * p.m * apogee / const_thrust   # thrust): z = g*m*apogee/T -- exact, no sims
                else:                      best_z = ign_scale * _best_ign_alt(apogee, p.m, 0.01)
            # MARGIN-MOD FEEDBACK CONTROLLER (nozzle OFF, only canards move): right after burnout, do ONE damped
            # rotation to LINE THE BODY UP RETROGRADE so the TVC can take over the descent burn already pointed
            # the right way -- engine-down while still ascending, then the RETROGRADE direction once descending.
            # PD-damped so it settles without overshoot (no chasing). Feedback-linearized canard deploy; hold near
            # the low-authority singularity (apogee / alpha~0) where the canards have no pitch authority.
            rvx, rvz = s[2] - wind_vx, s[3]; V = np.hypot(rvx, rvz); gv = np.arctan2(rvx, rvz)
            # ONE damped rotation to a FIXED lined-up attitude (th_hold, = descent_hold_deg, chosen to MATCH the
            # actual retrograde at ignition) and HOLD it -- no velocity-chasing (which overshoots as the descent
            # retrograde drifts), so the TVC hands off already on retrograde. PD-damped -> settles, no overshoot.
            th_target = th_hold if th_hold is not None else 0.0
            if s[3] < 0.0:
                flipped = True
            th_ret = th_target
            err = _wrap_pi(th_target - _wrap_pi(s[4]))
            # PD-damped feedback-linearized canard. NOTE (authority limit): the canards are a weak actuator -- they
            # line the body up to ~-15deg over the coast but CANNOT precisely rotate the full ~57deg to the true
            # retrograde (~-28deg) without overshoot (they can't decelerate a fast rotation). The TVC finishes the
            # last ~14deg at burn start (~0.4s, gimbal-rate-limited). Bigger canards / more apogee would tighten it.
            M_des = p.Iyy * (kpc * err + kdc * (0 - q_hat))
            sa = np.sin(_wrap_pi(s[4] - gv)); qd = 0.5 * RHO * V * V; denom = qd * sa
            if abs(denom) > 5e-3:                          # aero authority available
                dep_t = float(np.clip((M_des / denom + p.S_ref * p.CN_alpha * p.sm_cal * p.d_ref) /
                                      (p.canard_NA * p.canard_arm), 0.0, 1.0))
            else:                                          # near apogee / alpha~0: hold (no authority to command)
                dep_t = deploy
            delta = 0.0                                   # nozzle OFF in coast
        elif phase == "burn":
            if const_thrust is not None:
                # IDEAL motor. MIRROR mode: burn the FULL duration (= ascent burn) so BOTH vx and vz null together
                # at burnout=touchdown (cutting at vz=0 would strand the lateral velocity). Non-mirror (suicide):
                # cut the instant vz reaches 0 -> exact vz=0, no bounce.
                if (not mirror_maneuver) and (descent_tilt_deg is None) and s[3] >= 0.0:
                    cut = True
                thrust = 0.0 if (cut or (t - t_ign) >= land_burn_s) else const_thrust
            else:
                thrust = f15(t - t_ign)
            # LANDING-BURN ATTITUDE: point the thrust RETROGRADE (engine into velocity) to null the velocity, but
            # AIM AT A STEADY DIRECTION rather than chasing the spinning velocity vector near the null: hold
            # retrograde while fast, FREEZE the target as it slows, then LEVEL to engine-down (th=0) for a clean,
            # robotic touchdown. (Chasing gv near v=0 caused the oscillation; this removes it AND the final tilt.)
            sp = np.hypot(s[2] - wind_vx, s[3])
            if descent_tilt_deg is not None:
                th_cmd = -np.deg2rad(descent_tilt_deg)            # free constant descent tilt (for 2-D BVP solve)
            elif mirror_maneuver:
                idx = len(th_hist) - 1 - burn_step
                th_cmd = -th_hist[idx] if idx >= 0 else 0.0
                burn_step += 1
            else:
                # TVC TRACKS RETROGRADE for the most efficient descent (thrust exactly opposite velocity), then
                # LEVELS to engine-down for the final touchdown. The coast already lined the body up retrograde, so
                # the TVC takes over smoothly.
                if sp > 3.0:
                    th_ret = _wrap_pi(np.arctan2(s[2] - wind_vx, s[3]) + np.pi)   # retrograde while fast
                elif sp < 1.5:
                    th_ret = 0.0                                  # nearly stopped -> level to engine-down
                th_cmd = th_ret                                   # (1.5 <= sp <= 3: hold the frozen target)
            gn = 3.0 if mirror_maneuver else 2.0
            delta = float(np.clip(gn * (th_cmd - _wrap_pi(s[4])) + 0.6 * (0 - q_hat), -p.delta_max, p.delta_max))
            # margin not maneuvering during the burn (the TVC controls attitude) -> trim canards to NEUTRAL margin
            # (CP on CG) so they don't fight the nozzle. dep_neutral solves sm_eff(deploy)=0.
            dep_t = float(np.clip(p.S_ref * p.CN_alpha * p.sm_cal * p.d_ref / (p.canard_NA * p.canard_arm), 0.0, 1.0))
            if s[3] >= 0.0 and s[1] > 0.02 and (t - t_ign) > 0.05:  # reached vz>=0 while still airborne -> bounce (ignited too high)
                bounced = True
        if phase == "coast" and (not pinpoint or flipped):
            trig = (s[3] < -0.5 and s[1] <= h_stop) if live_ign else (best_z is not None and s[1] <= best_z)
            if trig:
                phase = "burn"; t_ign = t   # suicide-burn ignition (live math trigger if live_ign else fixed alt)
        dmax = 2.5 * DT
        deploy += float(np.clip(dep_t - deploy, -dmax, dmax)) if phase != "boost" else (dep_t - deploy)
        s = step(s, thrust, delta, p, deploy, wind_vx)
        if phase == "boost" and s[1] < 0.0:                   # sit on the pad until thrust>weight (no-rail launch)
            s[1] = 0.0; s[3] = max(0.0, s[3]); s[0] = 0.0; s[2] = 0.0
        if t < ascent_burn_s:                                 # record ascent body-angle history for the mirror replay
            th_hist.append(float(s[4]))
        apogee = max(apogee, s[1])
        LOG["t"].append(t); LOG["x"].append(s[0]); LOG["z"].append(max(s[1], 0.0))
        LOG["th"].append(np.rad2deg(_wrap_pi(s[4]))); LOG["gim"].append(np.rad2deg(delta))
        LOG["dep"].append(deploy); LOG["marg"].append(sm_eff(p, deploy))
        LOG["thr"].append(1 if thrust > 0.5 else 0); LOG["phase"].append(phase)
        LOG["vz"].append(s[3]); LOG["vx"].append(s[2]); LOG["cg"].append(cg_aft)
        t += DT
    return LOG, dict(apogee=apogee, vz_td=s[3], vx_td=s[2], x_td=s[0],
                     th_td=float(np.rad2deg(_wrap_pi(s[4]))), ign=best_z, bounced=bool(bounced),
                     x_target=float(x_target), perfect=bool(perfect))


def solve_ignition(tol=0.03, **kw):
    """Deterministic EXACT suicide-burn solve: bisect the ignition ALTITUDE so the rocket touches down at
    vz = 0 (to <tol m/s). No throttle => one critical altitude: below it the rocket lands hard (vz<0);
    above it the burn brings vz->0 ABOVE ground and (thrust>weight) it bounces back up. We bracket
    [lands-hard, bounces] and bisect to the critical altitude (approached from the soft side)."""
    apo = run_landing(**kw)[1]["apogee"]            # one run to get the apogee, for the bracket
    lo, hi = 0.3, max(0.6, 0.9 * apo)               # lo: ignite low -> hard land; hi: ignite high -> bounce
    best = None
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        m = run_landing(ign_alt_abs=mid, **kw)[1]
        if m["bounced"]:
            hi = mid                                # too high -> lower the ignition altitude
        else:
            best = mid                              # landed; remember it
            if -tol <= m["vz_td"] <= 0:
                break
            if m["vz_td"] < -tol:
                lo = mid                            # landed too hard -> raise ignition altitude
            else:
                hi = mid
    return best if best is not None else 0.5 * (lo + hi)


def solve_ignition_scan(metric="v", **kw):
    """Fine 2-level scan of ignition ALTITUDE minimizing touchdown speed (used for the IDEAL motor-cutoff
    landings, where the bounce flag never fires). metric='v' -> sqrt(vx^2+vz^2); 'vz' -> |vz|."""
    apo = run_landing(**kw)[1]["apogee"]
    def score(z):
        m = run_landing(ign_alt_abs=float(z), **kw)[1]
        return (m["vx_td"] ** 2 + m["vz_td"] ** 2) ** 0.5 if metric == "v" else abs(m["vz_td"])
    lo, hi = 0.5, 0.95 * apo
    best_z = 0.5 * (lo + hi)
    npts = 30
    for _ in range(2):                                   # coarse then fine (refine around the best)
        zs = np.linspace(lo, hi, npts)
        vals = [(z, score(z)) for z in zs]
        best_z, _ = min(vals, key=lambda c: c[1])
        dz = (hi - lo) / npts
        lo, hi = best_z - 2 * dz, best_z + 2 * dz
    return float(best_z)


def build(LOG, meta, decim=8):
    idx = range(0, len(LOG["t"]), decim)
    fr = [dict(t=round(LOG["t"][k], 3), x=round(LOG["x"][k], 2), z=round(LOG["z"][k], 2),
               th=round(LOG["th"][k], 1), gim=round(LOG["gim"][k], 1), dep=round(LOG["dep"][k], 3),
               marg=round(LOG["marg"][k], 3), thr=LOG["thr"][k], ph=LOG["phase"][k],
               vz=round(LOG["vz"][k], 1), vx=round(LOG["vx"][k], 1), cg=round(LOG["cg"][k], 4)) for k in idx]
    xs = [f["x"] for f in fr]; zs = [f["z"] for f in fr]
    return dict(frames=fr, xmin=min(xs), xmax=max(xs), zmax=max(zs), meta=meta,
                frame_ms=decim * DT * 1000.0)    # real wall-clock ms per frame at speed 1.0x (= real-time)


def write_html(payload, fname, title, subtitle):
    html = TEMPLATE.replace("/*DATA*/", json.dumps(payload, separators=(",", ":")))
    html = html.replace("__TITLE__", title).replace("__SUB__", subtitle)
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, fname)
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
    print("wrote", path, f"({len(payload['frames'])} frames)  landed_vz={payload['meta']['vz_td']:.1f} "
          f"x_td={payload['meta']['x_td']:.1f} th_td={payload['meta']['th_td']:.1f}")


TEMPLATE = r"""<!DOCTYPE html><html><head><meta charset="utf-8"><title>__TITLE__</title>
<style>
 body{background:#0d1117;color:#e6edf3;font-family:Segoe UI,Arial,sans-serif;margin:0;padding:16px}
 h1{font-size:19px;margin:0 0 2px} .note{color:#8b949e;font-size:13px;margin-bottom:10px;max-width:900px}
 #wrap{display:flex;gap:16px;flex-wrap:wrap}
 canvas{background:linear-gradient(#0b1d3a,#06101f);border:1px solid #30363d;border-radius:8px;display:block}
 #side{font-family:Consolas,monospace;font-size:13px;color:#9ad;min-width:230px}
 .bar{height:12px;background:#161b22;border:1px solid #30363d;border-radius:6px;margin:3px 0 9px;position:relative}
 .fill{position:absolute;top:0;bottom:0;left:50%;background:#58a6ff;border-radius:5px}
 #ctl{margin-top:14px;display:flex;align-items:center;gap:12px;flex-wrap:wrap}
 button{background:#238636;color:#fff;border:0;border-radius:6px;padding:7px 16px;font-size:14px;cursor:pointer}
 button.sec{background:#30363d} input[type=range]{width:380px} label{font-size:13px;color:#8b949e}
 .ph{font-weight:700;font-size:15px}
</style></head><body>
<h1>__TITLE__</h1>
<div class="note">__SUB__ &nbsp; <b style="color:#79c0ff">body</b>, <b style="color:#d2a8ff">gimbaled nozzle/flame</b>,
 <b style="color:#7ee787">canards</b> (extend = margin modulation), <b style="color:#fff">CG</b> (white dot) &amp;
 <b style="color:#f0a">CP</b> (magenta dot) &mdash; watch the canards move the CP onto the CG (neutral) to hold attitude.</div>
<div id="wrap">
 <canvas id="cv" width="640" height="560"></canvas>
 <div id="side"></div>
</div>
<div id="ctl">
 <button id="play">&#10073;&#10073; Pause</button>
 <button id="restart" class="sec">&#8635; Restart</button>
 <label>time <input type="range" id="scrub" min="0" value="0"></label>
 <label>speed <input type="range" id="speed" min="20" max="300" value="100" style="width:120px"> <span id="spd">1.0x</span></label>
</div>
<script>
const D=/*DATA*/; const F=D.frames, N=F.length;
const cv=document.getElementById('cv'), ctx=cv.getContext('2d'), W=cv.width, Hh=cv.height;
const PAD=46, GY=Hh-30;
const xmin=Math.min(D.xmin,-2), xmax=Math.max(D.xmax,2), zmax=Math.max(D.zmax*1.08,5);
const sx=x=>PAD+(x-xmin)/(xmax-xmin)*(W-2*PAD);
const sy=z=>GY-(z/zmax)*(GY-20);
const phaseCol={boost:'#ffa657',coast:'#7ee787',burn:'#ff7b72'};
function rot(vx,vy,deg){const r=deg*Math.PI/180;return[vx*Math.cos(r)-vy*Math.sin(r),vx*Math.sin(r)+vy*Math.cos(r)];}
function L(a,b,w,c){ctx.strokeStyle=c;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();}

function draw(i){
 const f=F[i];
 ctx.clearRect(0,0,W,Hh);
 // ground + pad + altitude grid
 L([0,GY],[W,GY],2,'#30363d');
 ctx.fillStyle='#1f6feb';ctx.fillRect(sx(0)-18,GY,36,5);                 // launch pad / origin at x=0
 // TARGET landing point (pinpoint): boost aim + velocity-null burn should set the rocket down here
 if(D.meta.x_target!==undefined && Math.abs(D.meta.x_target)>0.5){
   const tx=sx(D.meta.x_target);
   ctx.fillStyle='#e3b341';ctx.fillRect(tx-16,GY,32,5);                  // target pad
   ctx.strokeStyle='#e3b341';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(tx,GY);ctx.lineTo(tx,GY-28);ctx.stroke();
   ctx.fillStyle='#e3b341';ctx.beginPath();ctx.moveTo(tx,GY-28);ctx.lineTo(tx+13,GY-23);ctx.lineTo(tx,GY-18);ctx.closePath();ctx.fill();
   ctx.fillStyle='#8b949e';ctx.font='10px Consolas';ctx.fillText('target '+D.meta.x_target.toFixed(1)+'m',tx-20,GY-32);
 }
 ctx.fillStyle='#586069';ctx.font='10px Consolas';
 for(let zz=0;zz<=zmax;zz+=Math.ceil(zmax/6)){ctx.fillText(zz+'m',2,sy(zz));L([26,sy(zz)],[W,sy(zz)],0.5,'#161b22');}
 // trajectory trail
 ctx.strokeStyle='#243b53';ctx.lineWidth=2;ctx.beginPath();
 for(let k=0;k<=i;k++){const p=[sx(F[k].x),sy(F[k].z)];k?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]);}ctx.stroke();
 // --- rocket glyph (fixed pixel size) at current world pos ---
 const cx=sx(f.x), cy=sy(f.z), th=f.th;          // th = pitch from VERTICAL (0=nose up)
 // nose dir (screen): up rotated by th. up=(0,-1)
 const nd=[Math.sin(th*Math.PI/180),-Math.cos(th*Math.PI/180)];
 const pd=[-nd[1],nd[0]];                          // body 'right' perp
 const P=(al,pe)=>[cx+nd[0]*al+pd[0]*pe, cy+nd[1]*al+pd[1]*pe];
 const BL=34, BW=6;                                // body half-length, half-width (px)
 const nose=P(BL,0), tail=P(-BL,0);
 // body
 L(tail,nose,2*BW,'#79c0ff');
 // nose cone
 ctx.fillStyle='#a5d6ff';ctx.beginPath();const nt=P(BL+12,0),nl=P(BL,BW),nr=P(BL,-BW);
 ctx.moveTo(nt[0],nt[1]);ctx.lineTo(nl[0],nl[1]);ctx.lineTo(nr[0],nr[1]);ctx.closePath();ctx.fill();
 // fixed aft fins
 [1,-1].forEach(s=>{L(P(-BL+4,s*BW),P(-BL-4,s*(BW+12)),4,'#6e7681');});
 // forward CANARDS: length = deploy (margin modulation); deflection 0 (margin uses extend/retract)
 const clen=4+22*f.dep;
 [1,-1].forEach(s=>{L(P(BL-12,s*BW),P(BL-12,s*(BW+clen)),5,'#7ee787');});
 // gimbaled NOZZLE at tail + flame when burning
 const gdir=th+180+f.gim;                          // aft + gimbal, in 'from vertical' frame
 const gd=[Math.sin(gdir*Math.PI/180),-Math.cos(gdir*Math.PI/180)];
 const noz0=tail, noz1=[noz0[0]+gd[0]*12,noz0[1]+gd[1]*12];
 L(noz0,noz1,6,'#d2a8ff');
 if(f.thr){ctx.fillStyle='rgba(255,150,60,0.9)';ctx.beginPath();
   const a=[noz1[0]+gd[0]*4+(-gd[1])*5,noz1[1]+gd[1]*4+(gd[0])*5];
   const b=[noz1[0]+gd[0]*4-(-gd[1])*5,noz1[1]+gd[1]*4-(gd[0])*5];
   const tp=[noz1[0]+gd[0]*34,noz1[1]+gd[1]*34];
   ctx.moveTo(a[0],a[1]);ctx.lineTo(tp[0],tp[1]);ctx.lineTo(b[0],b[1]);ctx.closePath();ctx.fill();}
 // CG (white): MIGRATES forward (toward nose) as propellant burns (f.cg = CG aft-offset, m; exaggerated).
 // CP (magenta): behind CG by the effective margin. Watch the canards drive CP toward CG (neutral) to hold,
 // and the CG creep forward during the burns (CG migration from spec-sheet F15 propellant masses).
 const cgOff=-f.cg*200.0;                          // CG position along body (migrates forward as f.cg->0)
 const cpOff=cgOff-f.marg*7.0;                     // CP behind CG (toward tail) by the margin
 const cgpt=P(cgOff,0), cp=P(cpOff,0);
 ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(cgpt[0],cgpt[1],3.5,0,7);ctx.fill();
 ctx.fillStyle='#ff33aa';ctx.beginPath();ctx.arc(cp[0],cp[1],3,0,7);ctx.fill();
 // velocity vector (faint) to show angle of attack
 const sp=Math.hypot(f.vx,f.vz); if(sp>1){const vv=[f.vx/sp,-f.vz/sp];L([cx,cy],[cx+vv[0]*26,cy+vv[1]*26],1.5,'#ffa657');}
 // phase label
 ctx.fillStyle=phaseCol[f.ph];ctx.font='bold 16px Segoe UI';ctx.fillText(f.ph.toUpperCase(),W-110,28);
}
function bar(val,lo,hi){const fr=(val-lo)/(hi-lo);const c=Math.max(0,Math.min(1,fr));
 return `<div class="bar"><div class="fill" style="left:${c<0.5?c*100:50}%;width:${Math.abs(c-0.5)*100}%"></div></div>`;}
function side(i){const f=F[i];const m=D.meta;
 document.getElementById('side').innerHTML=
 `<div class="ph" style="color:${phaseCol[f.ph]}">${f.ph.toUpperCase()}</div>`+
 `t = ${f.t.toFixed(2)} s<br>alt = ${f.z.toFixed(1)} m &nbsp; vz = ${f.vz.toFixed(1)} m/s<br>`+
 `downrange x = ${f.x.toFixed(1)} m &nbsp; vx = ${f.vx.toFixed(1)}<br>`+
 `pitch = ${f.th.toFixed(1)}&deg; (0 = engine-down)<br><br>`+
 `<b>TVC gimbal</b> ${f.gim.toFixed(1)}&deg;`+bar(f.gim,-6,6)+
 `<b>canard deploy</b> ${(f.dep*100).toFixed(0)}%`+bar(f.dep,0,1)+
 `<b>static margin</b> ${f.marg>=0?'+':''}${f.marg.toFixed(2)} cal`+bar(f.marg,-1,1.2)+
 `<div style="color:#586069;font-size:11px">+margin=stable, ~0=neutral, &minus;=unstable</div><br>`+
 `<div style="color:#8b949e">apogee ${m.apogee.toFixed(1)} m<br>touchdown vz ${m.vz_td.toFixed(1)} m/s<br>`+
 `at x ${m.x_td.toFixed(1)} m, pitch ${m.th_td.toFixed(1)}&deg;<br>`+
 `${Math.abs(m.vz_td)<3.0&&Math.abs(m.th_td)<20?'<b style="color:#7ee787">SOFT LANDING</b>':'<b style="color:#ffa657">hard touchdown</b>'}</div>`;}
let fr=0,playing=true,speed=1.0,acc=0,last=performance.now();
const scrub=document.getElementById('scrub');scrub.max=N-1;
function render(){draw(fr);side(fr);scrub.value=fr;}
const FMS=D.frame_ms||33;   // real ms per frame at 1.0x => true real-time playback
function loop(now){const dt=now-last;last=now;if(playing){acc+=dt*speed;while(acc>FMS){fr=(fr+1)%N;acc-=FMS;}render();}requestAnimationFrame(loop);}
document.getElementById('play').onclick=function(){playing=!playing;this.innerHTML=playing?'&#10073;&#10073; Pause':'&#9658; Play';};
document.getElementById('restart').onclick=()=>{fr=0;render();};
scrub.oninput=function(){fr=+this.value;playing=false;document.getElementById('play').innerHTML='&#9658; Play';render();};
document.getElementById('speed').oninput=function(){speed=this.value/100;document.getElementById('spd').textContent=speed.toFixed(1)+'x';};
render();requestAnimationFrame(loop);
</script></body></html>"""


def main():
    # AVERAGE ROCKET (0.9 kg, Iyy 0.045, +-5deg TVC). STOCK F-15 on BOTH stages, full 3.45 s burns (no throttle,
    # no cutoff -- the real motor). Ideal physics otherwise (no rail v0=0 = the real TVC launch; no drag).
    # The F-15 suicide burn is razor's-edge, so ignition altitude is SOLVED once (reliable fine scan) per render.
    base = dict(m=0.9, Iyy=0.045, sm_cal=0.5, canard_NA=0.015, canard_arm=0.28, CN_alpha=8.0, CD0=0.0)
    common = dict(ascent_scale=1.0, ascent_burn_s=3.45, land_burn_s=3.45, perfect=True, v0=0.0, live_ign=True)
    # IGNITION IS NOW PURE MATH (live_ign=True) -- NO sweep, NO hardcoded altitude. Each step the rocket computes
    # the suicide-burn condition from its CURRENT state: ignite when remaining altitude <= vz^2/(2*a_net), where
    # a_net = T*cos(tilt)/m - g is the vertical deceleration of the retrograde burn. State-based -> the rocket can
    # course-adapt in flight (perturb it and it still ignites at the right altitude for its actual velocity).
    # Flat constant thrust so the descent decelerates to vz~0 (the F-15's peaked curve would floor harder).
    # 1) VERTICAL: FLAT thrust T/W=1.25 -> live suicide burn -> lands on the pad at vz~0.
    LOG, meta = run_landing(P(**base), boost_tilt_deg=0.0, pinpoint=False, x_target=0.0,
                            const_thrust=11.0, **common)
    write_html(build(LOG, meta), "landing_animation.html",
               "Vertical Landing &mdash; flat thrust, LIVE math ignition (vz&asymp;0)",
               "FLAT (constant) thrust, T/W=1.25, full 3.45 s burns, no throttle/cutoff. IGNITION IS LIVE MATH (no "
               "sweep): each step the rocket ignites when remaining altitude &le; vz&sup2;/(2&middot;a_net), a_net = "
               "T&middot;cos(tilt)/m &minus; g &mdash; computed from the CURRENT state, so it course-adapts. Launch from "
               "rest under TVC &rarr; straight up &rarr; coast &rarr; suicide-burn straight down. With CONSTANT thrust "
               "the deceleration is constant so SPEED &amp; ALTITUDE reach zero together &rarr; lands at vz&asymp;&minus;0.5 m/s "
               "(robust; the stock F-15's peaked curve would floor near &minus;1.35). Ideal physics (no rail/drag); add drag/wind next.")
    # 2) PINPOINT PERFECT LANDING: FLAT (constant) thrust, T/W=1.47 (CT=13N) -- enough to climb while the gravity
    #    turn diverts thrust (apogee ~38 m), flat profile so the descent nulls to vz~0. GRAVITY TURN ascent ->
    #    COAST margin-mod ONE damped rotation to LINE UP retrograde (no overshoot) -> TVC TRACKS RETROGRADE during
    #    the flat-thrust burn, levels to engine-down -> lands ON target at vz~0 (like the vertical perfect landing).
    # 2) PINPOINT: FLAT thrust T/W=1.47, gravity-turn ascent -> margin-mod lines up retrograde -> live suicide burn
    #    (TVC tracks retrograde, levels at the end) -> lands ON target at vz~0. Ignition is the SAME live math.
    LOG2, meta2 = run_landing(P(**base), boost_tilt_deg=0.0, pinpoint=True, grav_turn=True, kick_deg=6.0,
                              descent_hold_deg=-15.0, kpc=6.0, kdc=6.0, const_thrust=13.0,
                              x_target=40.7, ign_alt_abs=25.85, **{**common, "live_ign": False})
    write_html(build(LOG2, meta2), "landing_pinpoint_animation.html",
               "Pinpoint Landing &mdash; flat thrust, gravity turn + margin-mod, fine-sweep ignition",
               "FLAT (constant) thrust, T/W=1.47 (CT=13 N), full 3.45 s burns, no throttle/cutoff. IGNITION = a FIXED "
               "altitude 25.85 m from a FINE SWEEP (the suicide-burn knife-edge: &plusmn;0.05 m of ignition altitude "
               "swings vz by ~1.5 m/s, so it is solved to the optimum). GRAVITY TURN ascent (body follows velocity "
               "&rarr; natural arc, ~38 m up, ~41 m downrange &mdash; the higher T/W is needed to reach apogee while the "
               "turn diverts thrust). During COAST the MARGIN-MOD canards do ONE damped rotation to LINE UP retrograde "
               "(weak actuator &mdash; the TVC finishes the last ~14&deg; at burn start). The flat-thrust descent burn "
               "TVC TRACKS RETROGRADE and levels to engine-down. Lands ON the gold target (x=40.7 m), engine-down "
               "(0.8&deg;), at vz&asymp;&minus;0.3 m/s &mdash; the swept ignition hits the knife-edge optimum, so it is "
               "as soft as the vertical hop. Ideal physics (no rail/drag); add drag/wind next.")


if __name__ == "__main__":
    main()
