"""
tools/landing_sim.py  (2026-06-28)  --  PROPULSIVE-LANDING HOVERSLAM SIM (1-D vertical core)

Question this answers: can a FIXED-THRUST solid (F15) land a TVC rocket via a predictive suicide-burn,
and does TVC-as-throttle (cosine modulation, vertical thrust = F*cos(tilt)) rescue the landing under
realistic landing-motor thrust-curve VARIABILITY (the thing that makes a non-throttleable solid brutal)?

Model: 1-D vertical (the hoverslam is fundamentally a vertical-velocity-nulling problem). Attitude is
abstracted -- we ASSUME the attitude controller can achieve the commanded tilt for cosine-throttle, and
flag that the coast/ignition attitude problem is separate (see notes). Mass drops as propellant burns.

Phases: BOOST (ascent motor) -> COAST (no thrust, ballistic fall) -> LANDING BURN (ignite landing motor
at the predictive trigger; cosine-throttle tracks a constant-deceleration-to-zero-at-ground profile).

Key outputs:
  - apogee & average ascent T/W vs MASS (the ascent-vs-landing tradeoff)
  - descent velocity at ignition
  - touchdown velocity  (a) predictive ignition, NO throttle  (b) WITH cosine-throttle
  - sensitivity to +-10% landing-motor thrust-curve variability (ignition timed on the NOMINAL curve)
  - the hoverslam ignition-altitude WINDOW (range giving a survivable touchdown)

"Landed" = |touchdown velocity| < LEG_TOL (legs absorb the residual).
"""
from __future__ import annotations
import numpy as np

G = 9.80665
RHO = 1.20
LEG_TOL = 2.5          # m/s touchdown the landing legs can absorb
DT = 0.002

# --- Estes F15 thrust curve approximation (total impulse ~48 Ns, burn ~3.45 s, prop mass ~0.060 kg) ---
def f15_thrust(t, scale=1.0):
    if t < 0 or t > 3.45:
        return 0.0
    if t < 0.15:
        f = 25.0 * (t / 0.15)          # rise to ~25 N
    elif t < 0.50:
        f = 25.0 - (25.0 - 13.5) * ((t - 0.15) / 0.35)   # drop to plateau
    else:
        f = 13.5                       # plateau to burnout
    return f * scale

F15_BURN = 3.45
F15_PROP = 0.060       # kg propellant


def mass_at(m_dry, t_since_ign, burning, motor_prop=F15_PROP, burn=F15_BURN):
    """Mass dropping linearly as propellant burns during an active burn."""
    if not burning:
        return m_dry
    frac = min(max(t_since_ign / burn, 0.0), 1.0)
    return m_dry + motor_prop * (1.0 - frac)


def drag(v, CdA):
    return 0.5 * RHO * CdA * v * abs(v)


def simulate_ascent(m_dry, CdA, thrust_scale=1.0):
    """Boost on one F15 from rest to apogee. m_dry excludes ascent-motor propellant (added as it burns).
    Sits on the pad until thrust exceeds weight (the F15 curve starts at 0 N)."""
    h, v, t = 0.0, 0.0, 0.0
    apogee = 0.0; twmax = 0.0; tw_sum = 0.0; n = 0; launched = False
    while t < 30:
        burning = t < F15_BURN
        F = f15_thrust(t, thrust_scale) if burning else 0.0
        m = m_dry + (F15_PROP * (1.0 - min(t / F15_BURN, 1.0)) if burning else 0.0)
        if burning and F > 0:
            tw = F / (m * G); twmax = max(twmax, tw); tw_sum += tw; n += 1
        if not launched:
            if F > m * G:
                launched = True
            else:
                t += DT
                if not burning:      # motor done and never lifted -> won't fly
                    break
                continue
        a = (F - drag(v, CdA)) / m - G
        v += a * DT; h += v * DT; t += DT
        apogee = max(apogee, h)
        if v < 0:                    # apogee reached
            break
    return apogee, (tw_sum / max(n, 1)), twmax


def predict_touchdown_v(h0, v0, m_dry, thrust_scale, tilt=0.0, CdA=0.01):
    """Forward-sim the FULL trajectory (burn, then freefall after burnout if it climbed) from
    (h0, v0<0) at fixed tilt; return velocity at ground contact. Lets the rocket rise and fall back
    naturally (no early return) so the suicide-burn trigger sees the true touchdown velocity."""
    h, v, t = h0, v0, 0.0
    while h > 0 and t < 20:
        burning = t < F15_BURN
        F = f15_thrust(t, thrust_scale) if burning else 0.0
        m = mass_at(m_dry, t, burning)
        a = (F * np.cos(tilt) - drag(v, CdA)) / m - G
        v += a * DT; h += v * DT; t += DT
    return v


def landing_burn(h_ign, v_ign, m_dry, thrust_scale, use_throttle, CdA=0.01):
    """Run the actual landing burn from ignition. cosine-throttle tracks constant-decel-to-zero profile."""
    h, v, t = h_ign, v_ign, 0.0
    tilt = 0.0
    while h > 0 and t < 10:
        burning = t < F15_BURN
        F = f15_thrust(t, thrust_scale) if burning else 0.0
        m = mass_at(m_dry, t, burning)
        if use_throttle and burning and h > 0.05 and v < 0:
            a_req = v * v / (2.0 * h)                 # decel needed to reach 0 at ground
            a_avail = F / m - G
            if a_avail > 0.1:
                c = (a_req + G) * m / F               # cos(tilt) to make net decel = a_req
                c = min(max(c, 0.0), 1.0)
                tilt = np.arccos(c)
                tilt = min(tilt, np.deg2rad(45.0))    # authority limit
            else:
                tilt = 0.0
        else:
            tilt = 0.0
        a = (F * np.cos(tilt) - drag(v, CdA)) / m - G
        v += a * DT; h += v * DT; t += DT
        if v > 3 and not burning:      # over-decelerated, motor dead -> falls again, no recovery
            return _freefall_touchdown(h, v, CdA, m)
    return v


def _freefall_touchdown(h, v, CdA, m):
    t = 0.0
    while h > 0 and t < 10:
        a = -drag(v, CdA) / m - G
        v += a * DT; h += v * DT; t += DT
    return v


def best_ignition(apogee, m_dry, nom_scale, CdA):
    """SCAN candidate ignition altitudes; return the one whose nominal-thrust burn gives the touchdown
    velocity CLOSEST to zero (the optimal suicide-burn point). Also returns the best achievable v_pred:
    if best_vpred is still very negative, the motor physically cannot soft-land this apogee (no throttle)."""
    best_h, best_vpred = apogee * 0.5, -1e9
    for h_ig in np.linspace(apogee * 0.98, 0.5, 250):
        v_ig = -np.sqrt(max(2 * G * (apogee - h_ig), 0.0))
        vpred = predict_touchdown_v(h_ig, v_ig, m_dry, nom_scale, 0.0, CdA)
        # we want |vpred| minimal; prefer the least-negative (a slightly-positive vpred means it would
        # climb back -> also bad, so score by -|vpred| and disallow positive (climb-back) solutions)
        score = -abs(vpred) if vpred <= 0.3 else -10 - vpred
        if score > -abs(best_vpred):
            best_vpred = vpred; best_h = h_ig
    return best_h, best_vpred


def descend_and_land(apogee, m_dry, use_throttle, ign_bias=0.0, true_scale=1.0, nom_scale=1.0, CdA=0.01):
    """Find the optimal ignition altitude (scan on the NOMINAL curve), then run the real burn on the
    TRUE curve from there. ign_bias shifts the chosen ignition altitude (+ = higher/earlier)."""
    h_ig, best_vpred = best_ignition(apogee, m_dry, nom_scale, CdA)
    h_ig = min(max(h_ig + ign_bias, 0.5), apogee * 0.99)
    v_ig = -np.sqrt(max(2 * G * (apogee - h_ig), 0.0))
    v_td = landing_burn(h_ig, v_ig, m_dry, true_scale, use_throttle, CdA)
    return v_td, v_ig, h_ig


def main():
    print("=" * 78)
    print("  PROPULSIVE-LANDING HOVERSLAM SIM (F15, 1-D vertical, cosine-throttle)")
    print("=" * 78)
    print(f"  Landed = |touchdown v| < {LEG_TOL} m/s.  Ascent & landing both on F15 (~15N avg).")
    print(f"  m_dry below = airframe+avionics+BOTH motor casings (excl. propellant).\n")
    CdA = 0.01    # ~ pi*(0.033)^2 * Cd0(0.6) ~ 0.002... use 0.01 for a draggy small rocket; tweak w/ real Cd

    # --- MASS SWEEP: ascent apogee/TW (one F15) + landing result (second F15) ---
    print("-" * 78)
    print("  MASS SWEEP  (ascent on F15 #1, land on F15 #2; predictive ignition)")
    print("-" * 78)
    print(f"  {'m_dry':>6} {'apogee':>7} {'TWavg':>6} {'v@ign':>7} {'td_noThr':>9} {'td_Throttle':>11}")
    for m_dry in (0.8, 1.0, 1.2, 1.5):
        apo, twavg, twmax = simulate_ascent(m_dry, CdA)
        if apo < 2.0 or twmax < 1.2:
            print(f"  {m_dry:6.1f}  apogee={apo:4.1f}m (peak T/W={twmax:.2f}) -> too heavy for F15, won't hop")
            continue
        td_no, vign, hign = descend_and_land(apo, m_dry, use_throttle=False, true_scale=1.0, nom_scale=1.0, CdA=CdA)
        td_thr, _, _      = descend_and_land(apo, m_dry, use_throttle=True,  true_scale=1.0, nom_scale=1.0, CdA=CdA)
        print(f"  {m_dry:6.1f} {apo:7.1f} {twavg:6.2f} {vign:7.1f} {td_no:9.1f} {td_thr:11.1f}")
    print("  (v@ign, td_* are m/s; negative = descending. |td|<2.5 = landed.)\n")

    # --- THRUST-VARIABILITY: ignition timed on NOMINAL curve, real motor +-10% ---
    print("-" * 78)
    print("  THRUST-CURVE VARIABILITY (m_dry=1.2 kg; ignition timed on NOMINAL, real motor varies)")
    print("  -> does cosine-throttle rescue a fixed-solid landing when the motor isn't exactly nominal?")
    print("-" * 78)
    m_dry = 1.2; apo, twavg, _ = simulate_ascent(m_dry, CdA)
    print(f"  apogee {apo:.1f} m, ascent T/W {twavg:.2f}")
    print(f"  {'true motor':>11} {'td_noThrottle':>14} {'td_Throttle':>12}")
    for ts, lab in ((0.90, "-10%"), (0.95, "-5%"), (1.00, "nominal"), (1.05, "+5%"), (1.10, "+10%")):
        td_no, _, _  = descend_and_land(apo, m_dry, use_throttle=False, true_scale=ts, nom_scale=1.0, CdA=CdA)
        td_thr, _, _ = descend_and_land(apo, m_dry, use_throttle=True,  true_scale=ts, nom_scale=1.0, CdA=CdA)
        f1 = "LAND" if abs(td_no) < LEG_TOL else "crash"
        f2 = "LAND" if abs(td_thr) < LEG_TOL else "crash"
        print(f"  {lab:>11} {td_no:8.1f} {f1:>5} {td_thr:7.1f} {f2:>5}")
    print()

    # --- IGNITION-TIMING WINDOW: how many meters of ignition-altitude error survive? ---
    print("-" * 78)
    print("  IGNITION-ALTITUDE WINDOW (m_dry=1.2, nominal motor; ignite N m early(+)/late(-))")
    print("-" * 78)
    print(f"  {'bias(m)':>8} {'td_noThrottle':>14} {'td_Throttle':>12}")
    for bias in (3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0):
        td_no, _, _  = descend_and_land(apo, m_dry, use_throttle=False, ign_bias=bias, true_scale=1.0, nom_scale=1.0, CdA=CdA)
        td_thr, _, _ = descend_and_land(apo, m_dry, use_throttle=True,  ign_bias=bias, true_scale=1.0, nom_scale=1.0, CdA=CdA)
        f1 = "LAND" if abs(td_no) < LEG_TOL else "crash"
        f2 = "LAND" if abs(td_thr) < LEG_TOL else "crash"
        print(f"  {bias:8.1f} {td_no:8.1f} {f1:>5} {td_thr:7.1f} {f2:>5}")
    print()
    print("  READ: if cosine-throttle turns 'crash' into 'LAND' across +-10% thrust and a wider ignition")
    print("        window, it is the enabling technique for a non-throttleable solid landing.")
    print("  CAVEAT: 1-D vertical only. Assumes attitude control delivers the commanded tilt and that the")
    print("          vehicle is engine-down/low-rate at ignition -- the SEPARATE, unsolved coast-attitude")
    print("          problem. Replace f15_thrust() with your STATIC-FIRE-MEASURED curve before trusting numbers.")


if __name__ == "__main__":
    main()
