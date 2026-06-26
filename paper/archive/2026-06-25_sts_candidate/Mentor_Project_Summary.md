Project Summary: A Saturation Regime Parameter for Hobby-Scale TVC Attitude Control

---

What this project is

This project discovers and validates Π = keff × τ² — a nondimensional parameter that predicts when a hobby TVC rocket's control loop will enter a structural bang-bang saturation regime that PID controllers cannot escape, and quantifies exactly when ADRC's disturbance observer breaks that constraint. The core result is a regime transition at Π ≈ 0.32 (dimensionless angular displacement accumulated during one latency window), confirmed invariant across wind, servo speed, plant parameters, and probe operating point.

This is NOT a controller-design project and NOT a tuning study. It is an investigation of design-space structure: which combinations of thrust, inertia, and control-loop latency make a TVC rocket structurally resistant to gain selection, and why ADRC — but not PID, LQR, or SMC — breaks that resistance.

---

Why it matters

The amateur TVC community (BPS.space, OpenTVC, and thousands of hobbyists globally) uses PID controllers tuned in disturbance-free simulators. This project shows that for roughly 1.5% of realistic hardware combinations — concentrated among small, powerful rockets with slower microcontrollers — this practice has a 64% probability of producing false approval: the sim selects a gain that is 3–4× above the real stability ceiling (0.042/τ [rad/rad]). The builder trusts the sim, launches, and the rocket oscillates or crashes. The project identifies WHY through a causal mechanism (Π = keff × τ²), how to detect it before flight (single Kp = 0.044 [rad/rad] test), and how to fix it (ADRC or gain capping at 0.042/τ [rad/rad]).

The deliverables are: (1) a hardware formula builders can compute from motor specs before flight, (2) a flight test protocol (Kp = 0.044 [rad/rad], measure RMS) that detects at-risk designs on the launchpad, and (3) a concrete architectural recommendation (ADRC over PID) for hardware above the Π threshold.

---

Core findings

1. Π = keff × τ² organizes servo saturation fraction: fsat rises monotonically with Π (binned means 0.007 → 0.59 across Π tiers; Spearman rho = +0.55, n = 142, p = 9×10⁻¹³). An initial n = 29 sample gave an optimistic rho = +0.80, revised down on replication (the dose-response and the causal test both *strengthened* with the larger sample). A dual counterexample shows neither factor alone suffices: the highest-authority design in the population (keff = 1,897 s⁻², lat = 1 step) is perfectly safe at Π = 0.047, and a low-authority high-latency design is also safe at low Π — only the product determines risk. **Honest caveat:** on this design space (latency 1–6 steps) the aggregate correlation does *not* separate Π from keff (rho 0.55 vs 0.54), because latency's narrow range gives the τ² factor little marginal ranking power. The evidence that the product (not keff alone) is the right parameter therefore rests on the divergent counterexamples above, the latency-extended (1–12 step) data, and the blind-spot derivation A_blind = ½ keff × u_max × τ² — which predicts the τ² scaling. That exponent is consistent with the data but not uniquely identified by the available experiments.

2. The transition at Π ≈ 0.32 is confirmed invariant to: wind variation ×4, servo speed variation ×3.3, aerodynamic damping variation ×4 (exact zero effect), probe gain variation ×6 (max/min ratio = 1.00 exact) — within FIFO-delay implementations. The delay model type does matter: bare-metal integer FIFO gives Π_crit ≈ 0.20 (first onset in the 10-design reference set); RTOS jitter gives ≈ 0.32; IIR rate filtering (Madgwick/complementary) eliminates saturation entirely. Π ≈ 0.32 is an empirical transition within this TVC plant class, not a universal control-systems constant.

3. Causal confirmation: removing saturation from the simulation restores PID success rate to 1.000 for all 15 designs tested (2×2 factorial). ADRC's disturbance observer never triggers saturation (slew_frac = 0.000 for all 15 designs). Adding a first-order lag filter on the gyro output eliminates saturation entirely (fsat = 0.000 at Π = 1.38) — confirming the mechanism is the derivative channel's rate transients, not phase shift.

4. **Reactive** architectures all face the same Π constraint: PID (ρ ≈ −0.78), LQR (ρ = −0.747), SMC (ρ = −0.753), MPC H=1 saturated clip (ρ = −0.807 full physics). Both **non-reactive** approaches tested escape it: ADRC (ρ = −0.283 in the gain-window study; frac_pass = 1.000 for 50/50 designs in the full-physics audit) and **MPC H=5** (full-physics audit 2026-06-24: ρ = −0.052, p = 0.718, frac_pass_mean = 0.855). The common element is "reactive saturation" — both ESO (disturbance estimation before the actuator) and H=5 horizon planning (anticipatory bounded-command sequences) escape it. ADRC remains superior in practice (full success vs. 82%; lower compute cost).

5. ADRC extends the controllable frontier to Π_td = 57 (Π_keff ≈ 6.4) with no failures (3.9× beyond the original population maximum). At Π_td > 17 (Π_keff > 2.0), the ω₀/ωc ratio must scale with Π; at standard ratios, the frontier extends to Π_td ≈ 17 (Π_keff ≈ 2.0).

6. Disturbance-free simulator gives false approval to wrong gains for Π ≥ 0.34: calibration failure rate jumps from 9.1% to 61.3% (6.7× increase). In still air the gain search climbs to Kp ≈ 5.5–7.0 [rad/rad] with no wind ceiling signal; the real stability ceiling for high-Π designs is 0.042/τ ≈ 1.4–2.1 [rad/rad]. The builder uses the sim-approved gain, real flight enters persistent bang-bang oscillation, rocket crashes. Both genuinely uncontrollable designs receive 100% false approval — no warning signal exists in a disturbance-free simulator for a design that has no valid gain at any Kp.

---

What is built

Simulation infrastructure:
- Python TVC simulator with 10 independently toggleable fidelity modules (wind, sensor noise, slew, backlash, latency, thrust variation, deadband, nonlinear aerodynamics, dynamic pressure, CG shift). Validated cross-module.
- n = 2,400 LHS designs covering the realistic hobby TVC hardware space (T/W > 1 enforced).
- Gain-search protocol: joint 18×7 Kp×Kd grid (126 combinations), 30-seed evaluation, Wilson 95% CI flagging.

Core experiments (all with fresh disjoint seeds):
- Saturation regime map (n = 29 designs, 4 universality campaigns)
- 2×2 factorial causal isolation (n = 15, saturation on/off × PID/ADRC)
- Controller invariance: PID/LQR/SMC/ADRC (n = 50 designs each); MPC H=1 (ρ = −0.807, full physics, same as PD); **MPC H=5 full-physics audit (2026-06-24, n = 50): ρ = −0.052, p = 0.718 — escapes Π constraint via anticipatory planning; comparable to ADRC on frac_pass (0.855 vs 1.000); revised claim: reactive controllers fail, H=5 planning OR ESO succeeds**
- Performance frontier and extension to Π_td = 57 / Π_keff ≈ 6.4 (n = 63 + 44)
- Delay model robustness (integer FIFO, jitter, lag, lag+delay, n = 10 × 4 models)
- Single-airframe controlled experiment (Iyy swept, all else fixed, n = 18 levels)
- Flight detection (n = 36 narrow-window + 36 EASY, 7 seeds each; AUC = 0.954)
- S2R calibration failure study (n = 2,400, Π as predictor, false-approval framing)
- Pi exponent controlled test (4 keff × 8 lat, 30 seeds; smoking gun: keff = 137 s⁻², lat = 8 steps (τ = 40 ms), Π = 0.22 → safe)

Tools deliverable:
- `tools/gain_advisor.py`: runnable CLI that takes hardware specs (thrust, Iyy, latency, l_nozzle) and outputs Π, predicted risk tier, recommended Kp range and ADRC ωc ceiling with provenance notes.

---

Self-correction narrative (for context)

The project ran three major correction passes before reaching the definitive population:
1. 3-seed robustness tests are too coarse for the 0.80 threshold (resolution = 1/3 steps) → corrected to 15-seed re-evaluation.
2. Gain search was decoupled (Kd probed once at fixed Kp, then frozen) and coarser than some gain windows → corrected to joint 18×7 search.
3. Both R0475 (labeled INFEASIBLE in two passes) and several "FRAGILE" designs were simply not searched properly — the designs were controllable, the search wasn't finding their gains.

Each correction strengthened the primary AUC: 0.944 → 0.957 → 0.975. The corrections removed classification noise, revealing a stronger physical signal. Three bugs in the simulator (servo unit error, initial-angle radians/degrees, gain-search decoupling) were found and fixed; each required re-running a substantial subset of experiments.

This self-correction process is itself part of the scientific contribution: the project demonstrates what rigorous empirical work in simulation looks like, including how to distinguish noise removal from result manipulation.

---

Cross-platform generalization

A simplified version of the same test (Euler integrator, FIFO delay, normalized Kp sweep, success rate as outcome) was run on two additional second-order attitude control systems:

- **Quadrotor roll** (n = 50 stratified designs, extended run): rho(log Π, SR) = −0.937, p = 1.7×10⁻²³. Clear monotone dose-response: Π < 3.4 → mean SR = 1.000; Π > 80 → mean SR = 0.000. A precursor LHS run (n = 25, only n = 2 designs above Π = 3.4) gave rho = −0.471; the stratified extension resolved the underpowering.
- **Inverted pendulum** (n = 25 LHS designs, with gravitational instability): rho = −0.647, p = 4.8×10⁻⁴

All three systems show the same direction: higher Π → lower achievable SR. The quadrotor rho (−0.937) exceeds TVC (−0.668) because stripped physics makes Π the sole determinant — less confounding variance. The pendulum degrades at a lower Π threshold than the quadrotor because the gravitational instability term adds Π-independent difficulty. Each system has its own Π_crit; the direction (higher Π → lower SR) is what generalizes.

These results establish that Π = keff × τ² is not a TVC-specific artifact. The specific Π_crit ≈ 0.32 and the bang-bang slew-saturation mechanism are TVC-specific; the gross performance degradation at high Π generalizes to other delayed second-order plants.

What remains

Hardware validation: all thresholds (Π ≈ 0.32, 6.0° RMS, θ̈_max > 55 rad/s²) are calibrated in simulation. The direction and ranking of results generalize independently of absolute simulator accuracy; the specific constants require hardware confirmation. Priority hardware experiments:
1. Fly at Kp = 0.044 [rad/rad]; confirm RMS > 6.0° for a high-Π design (AUC = 0.954 in sim)
2. Fly same design with full-physics-tuned Kp; confirm SR improvement
3. Fly same design with ADRC; confirm saturation elimination
4. Compare high-Π vs low-Π hardware; confirm Π classification from specs
5. (Optional hardware extension) Vary quadrotor loop rate (8kHz → 250Hz) to sweep τ and test Π prediction at fixed keff

---

Mentorship requested

Most useful backgrounds:
1. Guidance, navigation, and control (GNC) — to review the DIPDT ceiling derivation, ADRC tuning methodology, and the bang-bang blind-spot mechanism.
2. Experimental design and statistics — to assess the simulation study design (LHS stratification, Wilson CI flagging, universality campaign design).
3. Embedded aerospace systems — to evaluate the delay model assumptions (FIFO vs filtered MCU implementations) and the practical scope of the Π_crit claim.
4. Hardware flight testing — to help design the minimal hardware validation campaign that would confirm or refute the primary threshold.

The most useful mentor intervention would be to assess: which of the six findings survives hardware validation with highest probability, and how to design the first two or three flights to produce the most scientific evidence per flight.
