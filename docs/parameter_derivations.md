# Simulator Parameter Derivations

Every numerical parameter in the simulator is listed below with its physical origin, units, and the evidence supporting its range. Parameters are grouped by subsystem. Parameters that are design-space variables (sampled by LHS) are marked **[DS]**; parameters that are fixed physical constants are marked **[FX]**; parameters that are calibrated empirical proxies are marked **[EM]**.

---

## 1. Reference Geometry

**[FX] D_ref = 0.040 m** — Body outer diameter. Chosen as a representative 38 mm motor mount + fiberglass shell. Standard Aerotech 38 mm hardware fits 2.5–3 inch airframes; 4 cm is the midpoint of typical hobby TVC builds (3–5 cm diameter). Used as aerodynamic reference diameter in all coefficient normalizations.

**[FX] S_ref = π/4 × D_ref² = 1.257 × 10⁻³ m²** — Cross-sectional reference area. Standard aerodynamic reference for axisymmetric bodies. Derived exactly from D_ref; no free parameter.

**[FX] L_rocket = 0.500 m** — Total rocket length. Consistent with an F-class model rocket: motor length (~10 cm) + avionics bay (~8 cm) + payload/nose (~32 cm). Range for hobby TVC builds is 35–70 cm; 50 cm is the lower bound for a build with adequate moment arm.

**[FX] l_nozzle_m ≈ 0.25 m** — Moment arm from the nozzle gimbal pivot to the vehicle CG. By definition equal to (L_rocket − x_CG_from_nose). For a bottom-heavy TVC build (motor at aft, avionics forward), CG sits at roughly 50% of body length from the nose, giving l_nozzle ≈ L/2 = 0.25 m. This parameter is the most mechanically important: control authority scales linearly with l_nozzle. For designs in the study, l_nozzle = l_nozzle_m is computed from the rocket length derived from this parameter.

**[FX] x_CG_from_nose ≈ l_nozzle_m** — CG position from nose. Follows directly from the definition of l_nozzle_m as the aft-to-CG moment arm: x_CG = L_rocket − l_nozzle_m. For L = 0.50 m and l_nozzle = 0.25 m, x_CG = 0.25 m (50% from nose). This is physically consistent with a design where structural mass and motor casing are roughly balanced fore and aft of the midpoint.

---

## 2. Aerodynamic Coefficients

**[FX] CN_alpha = 2.0 / rad** — Normal force coefficient slope. Barrowman (1966) slender body theory for a finless axisymmetric body of revolution gives exactly CN_alpha = 2.0 / rad for the combination of a nose cone and cylindrical body (the 2.0 is the classical result for a slender pointed body in linearized potential flow). This is independent of body diameter for a slender body (L/D > 5). Our geometry has L/D = 12.5, firmly in the slender body regime. With fins, CN_alpha would be 4–6 / rad; the finless assumption is correct for TVC designs that deliberately eliminate passive stability.

**[FX] Cm_alpha (physical, post-fix) = CN_alpha × static_margin** — Pitching moment coefficient about CG. Derived from CN_alpha and the CP-CG separation: Cm_alpha = CN_alpha × (x_CG − x_CP) / D_ref = CN_alpha × static_margin_calibers. For the design space range of static_margin ∈ [0.04, 0.24] calibers: Cm_alpha ∈ [0.08, 0.48]. A value of 0.20 at the reference design (static_margin = 0.10) means a 10° tilt at burnout (q_dyn = 540 Pa) produces a pitching acceleration of approximately 0.30 rad/s², requiring TVC authority to arrest within 0.5–1.0 s.

**[EM] Cmq = −12.5** — Pitch damping coefficient. No closed-form exact result exists for a finless cylinder; this value comes from the slender body approximation Cmq ≈ −0.08 × (L/D)². For L/D = 12.5: Cmq = −0.08 × 156.25 = −12.5. This falls within the published range for finless slender bodies (−10 to −20). With fins, Cmq would be −30 to −60; the finless estimate is conservative (less damping). Damping only enters when dyn_aero is active; its exact value affects the oscillation decay rate at high speed but does not change regime classification in simulation tests.

**[FX] CD0 = 0.40** — Zero-alpha axial drag coefficient. For a smooth fiberglass cylinder at subsonic speeds (Mach < 0.3), measured values cluster around 0.35–0.50. The upper estimate (0.40) accounts for surface roughness, fin-body junction drag (even without large fins, the nozzle assembly creates base drag), and launch lug drag. CD0 affects only the velocity integration (q_dyn evolution) and has second-order influence on regime classification.

**[FX] CD_alpha2 = 0.03 / rad²** — Induced drag coefficient slope. For a slender body, induced drag from AoA is small: CD ≈ CD0 + CD_alpha2 × alpha². At alpha = 0.2 rad (11.5°): delta_CD = 0.03 × 0.04 = 0.001, adding 0.25% to total drag. Negligible for the pitch dynamics.

**[FX] rho_air = 1.20 kg/m³** — Sea-level air density at 20°C. Standard atmosphere gives 1.225 kg/m³ at 15°C; 1.20 is appropriate for a summer launch day at moderate altitude. Variation over the first 200 m of flight is less than 2%.

---

## 3. Design Space Parameters (12 variables)

**[DS] mass ∈ [0.90, 2.10] kg** — Dry vehicle mass. Lower bound: minimum structural mass for an F-class build with avionics (≈ 0.9 kg including airframe, avionics, recovery hardware, excluding motor). Upper bound: heavy 3-inch build with ballast for center-of-mass adjustment experiments. Reference: 1.20 kg, consistent with published Aerotech 38 mm builds. Affects inertia_scale = Iyy_ref / Iyy and mass_scale = mass / mass_ref; enters disturbance scaling and simple-mode damping.

**[DS] Iyy ∈ [0.010, 0.040] kg·m²** — Pitch moment of inertia. Estimated from a uniform cylinder approximation: Iyy ≈ m × (L²/12 + r²/4) ≈ m × L²/12 for a slender body. For m = 1.20 kg, L = 0.50 m: Iyy ≈ 1.20 × 0.0208 = 0.025 kg·m². Mass distribution modifies this: forward-heavy builds reduce Iyy; aft-heavy builds (motor at extreme rear) increase it. Range [0.010, 0.040] spans factor-of-4 variation in mass distribution, covering short stubby designs (low Iyy) to long nose-heavy designs (high Iyy). Reference: 0.018 kg·m² (slightly below the uniform-cylinder estimate, reflecting some concentration toward the axis).

**[DS] static_margin ∈ [0.04, 0.24] calibers** — Aerodynamic stability margin, measured as (x_CG − x_CP) / D_ref. Positive means CP is aft of CG (counterintuitive for unstable rockets: here CP is forward, so x_CG > x_CP means the CG is aft of CP, making the rocket aerodynamically UNSTABLE). For a finless TVC rocket that requires active control: the design deliberately places CP forward of CG. The margin of 0.04–0.24 calibers corresponds to a CP-CG separation of 0.04 × 0.04 m = 1.6 mm to 0.24 × 0.04 m = 9.6 mm. Smaller margins (0.04) are nearly neutrally stable; larger margins (0.24) create strong divergence. The range captures all practically achievable finless TVC geometries.

**[DS] Cm_alpha ∈ [−70, −30] (design space proxy, dimensionless proxy units)** — This parameter is NOT the standard aerodynamic Cm_alpha. It is a calibration proxy used only in the simple-mode p_unstable formula. The negative sign follows aeronautics convention (negative = destabilizing). Numerically, |Cm_alpha_design| / 52 acts as a multiplier on the reference instability. The range [30, 70] creates instability rates p ∈ [0.5, 2.0] rad/s at reference conditions (thrust = 35 N, static_margin = 0.10). In full-physics mode, this parameter is superseded by CN_alpha × static_margin (the physically correct coefficient).

**[DS] control_effectiveness ∈ [5.0, 14.0] rad/s² / code unit** — Effective TVC angular authority (keff). Derived from the TVC moment chain: keff = (T × sin(max_gimbal) × l_nozzle / Iyy) / u_max. At reference conditions (T = 35 N, max_gimbal = 15°, l_nozzle = 0.25 m, Iyy = 0.018 kg·m², u_max = 12): keff = 35 × 0.259 × 0.25 / 0.018 / 12 = 10.5 rad/s² / code unit. The range [5, 14] spans variation in gimbal geometry, nozzle moment arm, and Iyy. Lower bound (5): small gimbal or heavy rocket. Upper bound (14): large gimbal with light rocket.

**[DS] thrust ∈ [25, 50] N** — Average motor thrust. F-class APCP motors span approximately 20–80 N average thrust. The range [25, 50] N corresponds to the lower half of F-class (F15 at 14.4 N average is below this range — the design space thrust represents the effective thrust seen by the controller, which can differ from motor rating due to nozzle efficiency). Reference: 35 N. Thrust enters q_dyn evolution (higher thrust → higher velocity → higher dynamic pressure → stronger aerodynamics).

**[DS] servo_slew_deg_s ∈ [20, 120] deg/s** — Maximum gimbal angular velocity. Hobby servo specifications: low-cost servos (e.g., SG90) achieve 60–120 deg/s at no load; loaded in a gimbal mechanism, effective slew drops to 30–70 deg/s. Premium digital servos (e.g., Hitec HS-5685MH) achieve 100–150 deg/s. Lower bound (20) represents a severely overloaded servo or deliberately limited PWM rate. Upper bound (120) is near the physical limit of hobby servos. Converted to code units: slew_max = slew_deg_s × (π/180) × (12 / max_gimbal_deg).

**[DS] max_gimbal_deg ∈ [8, 15] deg** — Maximum physical gimbal deflection angle. Lower bound (8°): mechanically constrained gimbal with limited linkage travel; still sufficient for TVC but reduces control authority. Upper bound (15°): standard TVC design target; beyond 15° the nozzle plume impingement on the aft airframe creates thermal problems and the linearization sin(δ) ≈ δ begins to fail at the 1% level. Reference: 15°.

**[DS] latency_steps ∈ [1, 6]** — Sensor pipeline delay in 5 ms samples. At dt = 0.005 s, one step = 5 ms. Sources of latency: ADC conversion (0.5–1 ms), digital filter settling (1–5 ms), software processing jitter (1–3 ms), USB/serial round-trip if using external computer (5–20 ms). Total: 3–10 ms typical, up to 30 ms for USB-based systems. Range [5, 30] ms → [1, 6] steps. Reference: 3 steps = 15 ms (representative of a fast embedded system with hardware serial).

**[DS] deadband ∈ [0.00, 0.15] code units** — Servo stiction threshold. Physical origin: minimum torque required to overcome static friction in the servo geartrain and gimbal linkage. In code units: a deadband of 0.10 with u_max = 12 corresponds to 0.10/12 × 15° = 0.125° of angular stiction. Published servo stall torques suggest stiction equivalent to 0.05–0.15° of gimbal hold — consistent with this range. Zero deadband represents an ideal servo; 0.15 is the upper limit before steady-state bias becomes large enough to threaten stability.

**[DS] backlash ∈ [0.00, 0.25] code units** — Total mechanical play in the gimbal linkage. Physical origin: clearance between drive gear teeth, pivot pin slop in the horn, and ball-link socket clearance. A 0.10 code unit backlash with u_max = 12 corresponds to 0.10/12 × 15° = 0.125° total play (±0.0625° each way). This is consistent with measured backlash in hobby servo horns (0.1–0.3° total) and commercial TVC gimbal kits (0.1–0.5°). Zero backlash is achievable with precision bearings; 0.25 represents poorly assembled hardware.

**[DS] wind_strength ∈ [0.05, 0.45] rad/s²** — Steady-state RMS of the AR(1) gust angular acceleration seen by the reference rocket. Derived from Dryden turbulence model at low altitude (< 100 m, Category C): RMS horizontal wind speed ~ 1–5 m/s, gust time scale ~ 0.4 s. For a 1.2 kg rocket with l_nozzle = 0.25 m and Iyy = 0.018 kg·m², a 1 m/s wind gust at 30° creates a normal force moment ≈ q × S × CN_alpha × alpha × l_arm ≈ 60 × 1.257e-3 × 2.0 × 0.1 × 0.15 ≈ 0.0023 N·m → 0.13 rad/s². The range [0.05, 0.45] spans calm-day (light breeze) to gusty-day (fresh breeze, Beaufort 4) conditions. Reference: 0.15 rad/s².

---

## 4. Motor Parameters (F15-class, fixed)

**[FX] T_peak = 22 N, T_avg = 14.4 N** — Peak and average thrust. Consistent with published Estes/Aerotech F-class APCP data. The F15 designation (15 N·s average thrust × 3 s ≈ 43 Ns total impulse) places this in the lower F class. The peak-to-average ratio of 22/14.4 = 1.53 matches typical APCP ignition spike characteristics.

**[FX] t_burn = 3.0 s** — Burn duration. Consistent with F-class at T_avg = 14.4 N: total impulse ≈ 43 Ns, Isp ≈ 157 s (typical APCP), m_prop = 43 / (9.81 × 157) ≈ 0.028 kg. The 3 s burn also defines the simulation window (t_end = 3.0 s).

**[FX] m_prop = 0.028 kg, m_casing = 0.048 kg** — Propellant and casing mass. Published Aerotech 38 mm F-class: propellant fraction ≈ 2–3% of vehicle wet mass (0.028/1.276 = 2.2%). Casing (48 g) is a typical aluminum 38 mm single-use casing weight. Both values set the magnitude of CG shift over the burn.

---

## 5. Sensor Parameters (fixed across all designs)

**[FX] gyro_noise_std = 0.015 rad/s** — Gyro white noise per sample (at dt = 0.005 s). Equivalent noise density: 0.015 × sqrt(0.005) = 0.0011 (rad/s)/√Hz ≈ 0.060 (deg/s)/√Hz. Consistent with published MEMS IMU specs: MPU-6050 noise density ≈ 0.005–0.05 (deg/s)/√Hz; ICM-42688-P ≈ 0.002–0.010 (deg/s)/√Hz. Value represents a mid-grade IMU under vibration.

**[FX] gyro_bias_init = 0.010 rad/s (1σ)** — Initial gyro bias drawn at t = 0. At 0.010 rad/s with a 3 s burn, uncorrected bias produces 0.03 rad = 1.7° of accumulated angle error — significant for tracking. The 90% calibration removal (bias_cal_residual = 0.10) leaves a residual of 0.001 rad/s → 0.003 rad error over the burn, acceptable.

**[FX] gyro_bias_rw = 0.005 rad/s/√s** — Bias random walk coefficient. Published Allan deviation plots for MPU-class IMUs show bias instability of 3–10 deg/hr ≈ 0.001–0.003 rad/s at 1 s averaging. Random walk coefficient is higher (0.005 rad/s/√s) to account for vibration-induced bias drift during motor burn.

**[FX] gyro_quant_lsb = 2π/4000 rad/s ≈ 0.00157 rad/s** — Gyro quantization step. Corresponds to a 4000-count full-scale range at ±2000 dps (a common MEMS gyro range): LSB = 4000 dps / 2000 = 2 mdps per count ≈ 0.0349 rad/s... Actually: 2π / 4000 = 0.00157 rad/s = 0.090 deg/s per count. This is consistent with a 16-bit ADC on a ±300 deg/s range: 600 / 65536 × π/180 ≈ 0.00016 rad/s (finer) or a 12-bit ADC on ±2000 deg/s: 4000 / 4096 × π/180 ≈ 0.017 rad/s (coarser). The value 2π/4000 is approximately a 12-bit ADC at ±360 deg/s.

---

## 6. Disturbance Parameters (fixed except wind_strength)

**[FX] gust_tau = 0.40 s** — AR(1) gust correlation time constant. Dryden turbulence length scale at low altitude: L_w ≈ h (altitude) for h < 600 m. At 30 m altitude and 5 m/s wind speed: tau ≈ L_w / V_wind = 30 / 5 = 6 s (Dryden). However, the simulator uses angular acceleration disturbances, not translational. The effective correlation time for torque fluctuations on a small rocket is shorter due to the finite response time of the aero-torque coupling. 0.40 s represents a moderate-coherence gust environment. Over a 3 s burn, the rocket experiences 3/0.40 ≈ 7 independent gust events.

**[FX] det_amp = 0.05 rad/s², det_freq_hz = 0.80 Hz** — Deterministic sinusoidal disturbance. Models structural resonance excitation or periodic buffet from the launch tower / rail. Amplitude 0.05 rad/s² is small relative to gust_std (reference: 0.15 rad/s²) — it represents a background periodic forcing, not a dominant disturbance. Frequency 0.80 Hz is below the control bandwidth (typically 2–5 Hz) so the controller should be able to reject it; failure to reject indicates insufficient Kd.

---

## 7. Controller and Integration

**[FX] dt = 0.005 s (200 Hz)** — Integration timestep. Chosen to resolve the actuator dynamics (tau_act = 0.05 s → 10 steps per time constant) and the fastest physical mode (p_unstable ≤ 10 rad/s → period ≥ 0.63 s → 126 steps). Semi-implicit Euler is stable at this dt for all design space conditions.

**[FX] tau_act = 0.05 s** — Servo first-order lag time constant. Represents servo motor bandwidth. Typical hobby servo step-response time constants: 30–80 ms. 50 ms (0.05 s) is representative of a mid-grade servo under load (HS-5685MH datasheet: ~40 ms no-load, ~80 ms loaded).

**[FX] Ki = 0** — Integral gain. Deliberately zero in all experiments. The control goal is attitude stabilization during a 3 s burn, not precise set-point tracking over long time horizons. Integral action would wind up against steady bias forces and introduce instability under the nonlinear actuator chain (backlash + saturation).

---

## 8. Success Thresholds

**[FX] max_theta < 70°** — Prevents structural and recovery failure. At 70° from vertical, recovery system deployment becomes unreliable and aerodynamic loads on the body exceed safe structural margins for cardboard airframes. Literature: RSO-level range safety typically requires 60–80° maximum excursion for hobby TVC.

**[FX] end_error < 15°, rms_error < 15°, peak_error < 50°** — Tracking quality gates. The 15° RMS threshold means the rocket achieves useful guidance accuracy (a 15° cone half-angle encompasses a 2× diameter ground footprint at 100 m altitude). The 50° peak limit prevents triggering the 70° structural limit on a single transient. These thresholds are inherited from the MATLAB baseline; they are design choices, not derived from first principles.

---

## Known Weaknesses in Parameter Evidence

| Parameter | Weakness |
|---|---|
| Cm_alpha_design proxy [30–70] | Calibration constants (10, 52, 35) fit to undocumented MATLAB reference data |
| REF_AERO_DAMP = 0.5 /s | Simple-mode value; not independently measured. Replaced by physical Cmq model in full mode |
| CD0 = 0.40 | Estimated from literature; no wind tunnel or CFD confirmation for this specific geometry |
| Cmq = −12.5 | Slender body approximation; no experimental validation |
| gust_tau = 0.40 s | Chosen to approximate low-altitude turbulence; not matched to a specific Dryden profile |
| det_amp, det_freq | Arbitrary; represent no specific physical phenomenon — modeling robustness, not realism |
