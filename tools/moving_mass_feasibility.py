"""
tools/moving_mass_feasibility.py  (2026-06-25)

FEASIBILITY of moving-mass (CG-shift) attitude control on a MODEL ROCKET.

Mechanism modeled (lateral-offset, the most TVC-like / direct-torque scheme):
  An internal mass m_move slides laterally by y (|y|<=y_max), shifting the vehicle CG
  laterally by  delta_cg = (m_move / M_total) * y.
  The axial forces (thrust T, drag D) act along the body axis; with the CG offset by
  delta_cg they produce a CONTROL TORQUE about the CG:
        M_control = (T + D) * delta_cg
  Aerodynamic restoring (or destabilizing) moment:  M_aero = -k_aero(t) * theta
        k_aero(t) = q(t) * S * CN_alpha * (static_margin_cal * D_ref)   [>0 stable]
  Wind enters as a gust angle-of-attack disturbance moment.
  The mass actuator is RATE-LIMITED (a servo/leadscrew moves the mass at v_mass max).

Honest scope: 1-DOF pitch, near-vertical (alpha ~ theta), lumped aero. This is a
feasibility-authority assessment, not a high-fidelity 6-DOF sim. The point is to size
the control authority over the flight (burn vs coast) and test closed-loop holdability.
"""
import numpy as np

# ---- realistic model-rocket parameters ----
M_TOTAL = 0.55        # kg total
IYY     = 0.015       # kg m^2 pitch inertia
M_MOVE  = 0.15        # kg moving mass (~27% of total; e.g. the LiPo battery)
Y_MAX   = 0.030       # m max lateral travel (mechanism allows a bit past the 5cm body radius)
V_MASS  = 0.25        # m/s max mass slew rate (servo + leadscrew)
DELTA_CG_MAX = (M_MOVE / M_TOTAL) * Y_MAX   # m, max CG lateral offset

D_REF   = 0.05        # m body diameter
S_REF   = np.pi*(D_REF/2)**2   # m^2 reference area
CD      = 0.55        # axial drag coeff
RHO     = 1.20        # kg/m^3
G       = 9.81

T_BURN  = 12.0        # N average thrust (F-class)
BURN_S  = 3.0         # s burn time
T_END   = 8.0
DT      = 0.005

def thrust(t):  return T_BURN if t < BURN_S else 0.0

def flight_profile():
    """Integrate axial velocity to get v(t), q(t), D(t) over the flight."""
    n = int(T_END/DT); v = 0.0; out = []
    for k in range(n):
        t = k*DT
        D = 0.5*RHO*v*v*S_REF*CD
        vdot = (thrust(t) - D)/M_TOTAL - G          # near-vertical
        v = max(0.0, v + vdot*DT)
        q = 0.5*RHO*v*v
        out.append((t, v, q, D))
    return out

def authority_profile(prof):
    """theta_ddot_max(t) = (T+D)*delta_cg_max / Iyy."""
    return [( t, ((thrust(t)+D)*DELTA_CG_MAX)/IYY ) for (t,v,q,D) in prof]

def simulate(static_margin_cal, wind_amp=0.20, seed=0, kp=8.0, kd=2.0,
             control_during_coast=True, mass_control=True):
    """Closed-loop pitch hold (theta_ref=0). Returns dict of metrics."""
    rng = np.random.default_rng(seed)
    prof = flight_profile()
    theta = np.deg2rad(3.0)   # small initial perturbation
    q_rate = 0.0; y = 0.0     # mass position
    wind_state = 0.0
    max_abs = 0.0; coast_max = 0.0; sat_frac = 0; nsteps = 0
    for (t, v, q, D) in prof:
        powered = t < BURN_S
        # aero stiffness (N*m per rad);  >0 restoring (stable), <0 destabilizing (unstable)
        k_aero = q * S_REF * 8.0 * (static_margin_cal * D_REF)   # CN_alpha~8 finned
        # colored wind gust -> disturbance angle of attack -> moment
        wind_state = 0.95*wind_state + 0.32*wind_amp*rng.standard_normal()
        M_wind = q * S_REF * 8.0 * (D_REF*0.5) * wind_state
        # ---- control: PID on theta -> desired CG offset -> mass position ----
        if mass_control and (powered or control_during_coast):
            axial = thrust(t) + D
            # desired control moment from PID
            M_des = -(kp*theta + kd*q_rate)
            # required CG offset (avoid div by ~0 axial force near apogee)
            delta_des = M_des/axial if axial > 0.05 else 0.0
            y_des = np.clip(delta_des*(M_TOTAL/M_MOVE), -Y_MAX, Y_MAX)
            # rate-limit the mass motion
            dy = np.clip(y_des - y, -V_MASS*DT, V_MASS*DT); y += dy
            if abs(abs(y)-Y_MAX) < 1e-6: sat_frac += 1
            delta_cg = (M_MOVE/M_TOTAL)*y
            M_ctrl = axial*delta_cg
        else:
            M_ctrl = 0.0
        # ---- pitch dynamics ----
        M_total = -k_aero*theta + M_ctrl + M_wind
        q_rate += (M_total/IYY)*DT
        theta  += q_rate*DT
        max_abs = max(max_abs, abs(np.rad2deg(theta)))
        if not powered: coast_max = max(coast_max, abs(np.rad2deg(theta)))
        nsteps += 1
    success = max_abs < 30.0    # stayed within 30 deg (didn't tumble)
    return dict(success=success, max_dev_deg=round(max_abs,1),
                coast_max_deg=round(coast_max,1), sat_frac=round(sat_frac/nsteps,2))

def main():
    prof = flight_profile()
    auth = authority_profile(prof)
    print("=== MOVING-MASS CONTROL FEASIBILITY (model rocket) ===")
    print(f"  M={M_TOTAL}kg Iyy={IYY} m_move={M_MOVE}kg y_max={Y_MAX*100:.0f}cm  -> delta_cg_max={DELTA_CG_MAX*1000:.1f}mm")
    print(f"  TVC reference authority (project): theta_ddot_max ~ 20-200 rad/s^2\n")
    print("  Control authority theta_ddot_max over flight:")
    for tt in [0.1, 0.5, 1.0, 2.0, 2.9, 3.1, 4.0, 5.0, 6.0, 7.0]:
        a = min(auth, key=lambda z: abs(z[0]-tt)); v = min(prof, key=lambda z: abs(z[0]-tt))[1]
        phase = 'BURN ' if tt < BURN_S else 'COAST'
        print(f"    t={tt:4.1f}s {phase} v={v:5.1f}m/s  theta_ddot_max={a[1]:6.2f} rad/s^2")
    peak = max(a for _,a in auth)
    print(f"\n  Peak moving-mass authority: {peak:.1f} rad/s^2  (~{200/max(peak,0.1):.0f}x weaker than strong TVC, ~{20/max(peak,0.1):.1f}x vs weak TVC)")

    print("\n=== Closed-loop attitude hold (stable airframe, SM=+1.0 cal), 8 seeds ===")
    res = [simulate(1.0, seed=s) for s in range(8)]
    sr = np.mean([r['success'] for r in res])
    print(f"  success={sr:.2f}  mean max_dev={np.mean([r['max_dev_deg'] for r in res]):.1f}deg  "
          f"mean coast_max={np.mean([r['coast_max_deg'] for r in res]):.1f}deg  sat={np.mean([r['sat_frac'] for r in res]):.2f}")

    print("\n=== Coast-only control benefit: hold during coast vs no coast control ===")
    on  = [simulate(1.0, seed=s, control_during_coast=True)  for s in range(8)]
    off = [simulate(1.0, seed=s, control_during_coast=False) for s in range(8)]
    print(f"  coast control ON : coast_max={np.mean([r['coast_max_deg'] for r in on]):.1f}deg  success={np.mean([r['success'] for r in on]):.2f}")
    print(f"  coast control OFF: coast_max={np.mean([r['coast_max_deg'] for r in off]):.1f}deg  success={np.mean([r['success'] for r in off]):.2f}")

    print("\n=== Max controllable instability (sweep static margin negative) ===")
    for sm in [0.5, 0.0, -0.25, -0.5, -1.0, -1.5]:
        r = [simulate(sm, seed=s) for s in range(5)]
        print(f"  SM={sm:+.2f} cal: success={np.mean([x['success'] for x in r]):.2f}  max_dev={np.mean([x['max_dev_deg'] for x in r]):.1f}deg")
    print("\nVERDICT printed above: compare authority to TVC, check stable-hold success,")
    print("coast benefit (ON vs OFF), and how unstable it can stabilize.")

if __name__ == '__main__':
    main()
