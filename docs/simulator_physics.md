# TVC Rocket Simulator — Physics Reference

This document is the authoritative reference for the physical model implemented in `sim/`. It is intended for use across multiple AI assistant sessions to maintain consistent physical reasoning.

---

## 1. What the Simulator Models

A single-axis (1-DOF pitch) TVC model rocket during a 3-second powered burn, flying nearly vertically from a launch rail. The controller is a pure PD controller (Ki=0) acting on the pitch angle error. The rocket is aerodynamically unstable by design — it requires active TVC to remain upright.

The pitch angle θ is measured from vertical. θ=0 is the upright reference. Positive θ is nose-right (or nose-tilted from vertical). The control objective is θ → 0.

---

## 2. Gravity — What It Does and Does Not Do

**Gravity does NOT produce a pitching moment about the CG in free flight.**

For a free-flying rocket (not on a pivot), gravity acts through the center of mass. By definition, the moment of a force about the point through which it acts is zero. Therefore:

- Gravity enters only the **translational** equation: `m * dv/dt = T*cos(θ) - D - m*g`
- Gravity does NOT appear in the **rotational** equation (pitching moment about CG)
- The `p² * θ` term in the plant EOM is entirely aerodynamic, not gravitational
- Old documentation that mentions "gravitational pendulum" is incorrect for free flight

**Indirect effect of gravity on pitch dynamics:** Gravity reduces flight speed (and hence dynamic pressure q_dyn), which in turn reduces the aerodynamic pitching moment and pitch damping. This is captured when the `dyn_aero` module is active.

---

## 3. Aerodynamics in Detail

### 3.1 Angle of Attack

For a rocket flying at angle θ from vertical, the oncoming airflow (assumed vertical) hits the rocket body at angle of attack α. For near-vertical flight:

```
α ≈ θ − γ ≈ θ       (γ = flight path angle ≈ 0 for vertical launch)
```

### 3.2 Normal Force

The most important aerodynamic force for TVC rockets. When the rocket tilts at angle of attack α, the airflow creates a normal force N perpendicular to the velocity vector, acting at the **center of pressure (CP)**:

```
N = CN_alpha × q_dyn × S_ref × α
```

where:
- `CN_alpha` ≈ 2.0 / rad for a finless body of revolution (Barrowman slender body theory)
- `q_dyn = 0.5 × ρ × v²` = dynamic pressure (Pa), varies with flight speed
- `S_ref = π/4 × D²` = reference area (cross-sectional area)
- `α` = angle of attack (rad)

### 3.3 Pitching Moment About CG

The normal force N acts at the CP, creating a moment about the CG:

```
M_aero = N × (x_CG − x_CP) = CN_alpha × q_dyn × S_ref × α × (x_CG − x_CP)
```

Equivalently, using the Cm_alpha coefficient:
```
M_aero = Cm_alpha × q_dyn × S_ref × D_ref × α
```
where `Cm_alpha = CN_alpha × (x_CG − x_CP) / D_ref`.

**Sign convention used in this simulator:**
- `Cm_alpha > 0` means **unstable** (CP forward of CG, x_CP < x_CG)
- When α > 0 (nose up), M_aero > 0 (also nose-up) → destabilising

**Sign convention in the design space parameter `Cm_alpha`:**
- Sampled in range [-70, -30] — **negative values represent unstable rockets**
- This is opposite to the convention above: `Cm_alpha_physical = abs(Cm_alpha_design)`
- The design space uses aeronautics convention (Cm_alpha < 0 = stable)
- Internal physics uses `abs(Cm_alpha_design)` as the destabilising coefficient

**Nonlinear correction:** For large angles, replace α with sin(α):
```
M_aero = Cm_alpha × q_dyn × S_ref × D_ref × sin(θ)
```
At 30°: sin(30°) = 0.50 vs 30° in rad = 0.524 → 5% error (significant for transients)
At 50°: sin(50°) = 0.766 vs 0.873 rad → 12% error

### 3.4 Pitch Damping

When the rocket pitches at rate θ̇, the tail moves perpendicular to the flight path faster than the nose. This creates an asymmetric angle-of-attack distribution along the body: the tail sees a higher effective angle of attack than the nose. The resulting aerodynamic force opposes the rotation.

```
M_damp = q_dyn × S_ref × D_ref × Cmq × (θ̇ × D_ref) / (2 × v)
```

**Key physical insight:** Damping scales as `q_dyn/v ∝ v` (not v²). At launch (v small), there is little aerodynamic damping. As the rocket accelerates, damping grows. This means early-flight instability is harder to damp than late-flight instability.

Typical values:
- Finless body: `Cmq ≈ -10 to -20`
- With fins: `Cmq ≈ -30 to -60`
- This simulator uses `Cmq ≈ -15` for finless TVC rockets

The existing simple model uses `damp_eff × θ̇` (constant damping), which is equivalent to assuming constant flight speed. The `dyn_aero` module replaces this with the physically correct velocity-dependent form.

### 3.5 Drag

```
F_drag = q_dyn × S_ref × (CD0 + CD_alpha2 × α²)
```

- `CD0 ≈ 0.40` for a smooth finless body (subsonic)
- `CD_alpha2 ≈ 0.03 / rad²` (induced drag, small)
- Drag affects translation (dv/dt) but NOT the pitching moment

### 3.6 Dynamic Pressure History

For a near-vertical launch:
```
dv/dt = T(t)/m(t) - CD × S_ref × q_dyn / m(t) - g
q_dyn(t) = 0.5 × ρ × v(t)²
```

For an F15 motor (average 14.4 N, 3 s burn, total mass 1.2 kg):
- At liftoff: v ≈ 0.01 m/s, q_dyn ≈ 0 Pa → aerodynamic forces negligible
- At t=1s: v ≈ 10 m/s, q_dyn ≈ 60 Pa
- At t=3s (burnout): v ≈ 25-30 m/s, q_dyn ≈ 375-540 Pa
- At constant q_dyn reference (simple mode): q_dyn ≈ 540 Pa (representative of average)

The `dyn_aero` module integrates this velocity evolution and uses the correct time-varying q_dyn.

---

## 4. TVC Control Moment

```
M_TVC = T(t) × sin(δ) × l_nozzle
```

where:
- `T(t)` = current thrust (N)
- `δ` = gimbal deflection angle (rad), commanded by controller
- `l_nozzle` = distance from nozzle to CG (m)

**In code units:**
```
δ = u_act × (max_gimbal_deg × π/180) / u_max_code
M_TVC = T × sin(δ) × l_nozzle
```

**Simple mode approximation:** The effective control authority `keff_phys` absorbs all of T, l_nozzle, gimbal range, and Iyy into one constant:
```
keff_phys = keff_design × thrust_scale × inertia_scale
```
This is constant throughout the simulation. The `thrust_curve` and `cg_shift` modules make T(t) and l_nozzle(t) time-varying.

---

## 5. Complete Pitch EOM (Full Physics Mode)

```
I_yy(t) × θ̈ = M_TVC + M_aero + M_damp + I_yy × d(t)

M_TVC  = T(t) × keff_fault × sin(u_act × gimbal_rad/u_max) × l_nozzle(t)
M_aero = Cm_alpha × q_dyn(t) × S_ref × D_ref × sin(θ)   [nonlinear]
M_damp = q_dyn(t) × S_ref × D_ref × Cmq × (θ̇ × D_ref) / (2 × v(t))
d(t)   = disturbance angular acceleration (rad/s²)
```

Integration: semi-implicit Euler at dt = 0.005 s.

---

## 6. Motor Model (motor_model.py)

### F15-class thrust curve shape (parametric)
- Rise: linear 0 → T_peak (0 to t_peak ≈ 0.12 s)
- Plateau: T_avg for t ∈ [t_peak, t_plateau_end ≈ 2.4 s]
- Taper: linear T_avg → 0 for t ∈ [t_plateau_end, t_burn ≈ 3.0 s]

### Propellant burn
```
dm_prop/dt = -T(t) / (g0 × Isp)
Isp = total_impulse / (m_prop × g0)
```
Mass flow is integrated each timestep. CG moves forward as aft propellant is consumed.

### CG evolution
```
x_CG(t) = (m_dry × x_CG_dry + m_prop(t) × x_prop_cg) / m_total(t)
```
For F-class motors: total CG shift ≈ 1-3 mm over the full burn. Small but physically correct.

### I_yy evolution
Using parallel axis theorem: I_yy(t) decreases as propellant mass is removed from aft.

---

## 7. Simple Mode vs. Full Physics Mode

| Feature | Simple (legacy) | Full (new modules) |
|---|---|---|
| Aerodynamic moment | p² × θ (linearised, constant) | Cm_alpha × q_dyn(t) × S_ref × D × sin(θ) |
| Aerodynamic damping | damp_eff × θ̇ (constant) | q_dyn/v dependent pitch damping |
| Thrust | Constant (thrust_scale × T_ref) | Realistic T(t) profile from motor_model |
| Mass/CG/I_yy | Fixed | Evolving from propellant burn |
| Dynamic pressure | Constant reference value | Integrated from velocity equation |

**FidelityConfig flags:**
- `nonlinear_aero`: sin(θ) vs θ in aerodynamic moment
- `dyn_aero`: time-varying q_dyn from velocity integration
- `thrust_curve`: realistic T(t) profile
- `cg_shift`: evolving CG, I_yy, total mass
- `FidelityConfig.legacy_full()`: original 7 modules, new physics OFF (for Exp1/Exp4 compatibility)
- `FidelityConfig.full()`: all 11 modules ON (most physically accurate)

---

## 8. Known Physical Approximations (Disclosed)

1. **sqrt(mass_scale) damping** (simple mode only): `damp_eff ∝ sqrt(m/m_ref)`. Not first-principles. Retained for MATLAB parity. Replaced by physical Cmq model in full mode.

2. **p_unstable calibration constants** (simple mode only): `p = 10 × static_margin × |Cm_alpha| / 52 × sqrt(35/thrust)`. Three constants (10, 52, 35) calibrated to data, not derived. This is the most significant physical approximation in the simple model. In full mode, the instability rate is computed directly from `q_dyn × S_ref × D × Cm_alpha / I_yy`.

3. **1-DOF (pitch only)**: Roll, yaw, and translational-rotational coupling are absent. Valid for near-vertical flight with no initial yaw perturbation. Cross-axis coupling is a future fidelity module.

4. **Constant air density**: ρ = 1.20 kg/m³ (sea level). Negligible error for the first few hundred meters of altitude.

5. **Propellant CG at motor midpoint**: More accurate would be distributed mass model (propellant density × volume element). Point-mass approximation introduces ~5% error in CG position.

6. **Sensor noise per-step amplitude**: `gyro_noise_std` is not scaled to dt, creating a latent units bug. Does not affect results at fixed dt = 0.005 s. Correction: scale noise as `N(0, gyro_noise_std / sqrt(dt))` for variable dt.

7. **Gust initialization**: Fixed at t=0 from stationary distribution N(0, gust_std). Previous implementation initialized at g=0, understating disturbances for first ~1.2 s. Fixed in current version.

---

## 9. CFD Integration Path

When CFD data is available, replace the following in `aero_model.py`:
- `CN_alpha_from_design()` → table lookup from CFD normal force vs AoA
- `Cmq_from_design()` → table lookup from CFD pitch damping derivative vs Re
- `CD0 = 0.40` → measured CD at zero AoA
- `CD_alpha2 = 0.03` → measured induced drag coefficient

CFD should output, at minimum:
- CN(α) at α = 0, 5, 10, 15, 20, 30, 45 deg (for nonlinear table)
- Cmq at representative Reynolds numbers (one value is sufficient for model rockets)
- CD(α) at the same AoA range
- CP position as function of Mach number and AoA

---

## 10. Design Space Parameter Mapping

| Design parameter | Physical meaning | Used in |
|---|---|---|
| `mass` | Vehicle wet mass (kg) | mass_scale, velocity eq |
| `Iyy` | Pitch moment of inertia (kg·m²) | inertia_scale, rotational EOM |
| `static_margin` | (x_CG - x_CP)/D_ref (calibers) | p_unstable proxy, Cm_alpha derivation |
| `Cm_alpha` | Pitching moment coefficient slope (negative = unstable) | Aerodynamic moment magnitude |
| `control_effectiveness` | keff: TVC angular authority (rad/s² / code unit) | M_TVC equivalent in simple mode |
| `thrust` | Average thrust (N) | thrust_scale, velocity equation |
| `servo_slew_deg_s` | Max servo angular velocity (deg/s) | Actuator slew limit |
| `max_gimbal_deg` | Physical gimbal travel limit (deg) | Gimbal angle conversion |
| `latency_steps` | Sensor pipeline delay (samples × 5 ms) | FIFO depth |
| `deadband` | Actuator stiction threshold (code units) | Per-step movement check |
| `backlash` | Total gimbal play gap (code units) | KP hysteresis operator |
| `wind_strength` | Gust steady-state RMS (rad/s²) | AR(1) disturbance amplitude |
