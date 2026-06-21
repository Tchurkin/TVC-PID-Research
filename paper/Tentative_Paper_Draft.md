# A Dimensionless Control-Authority Parameter Governing Gain-Window Limits in Hobby-Scale TVC Rockets

**Braxton Herold**

---

## Thesis

A TVC attitude-hold controller's safe gain window — the range [Kp_floor, Kp_ceiling] within which its proportional gain must land for reliable wind-rejection — is governed by two boundaries whose **latency dependencies were previously treated as one-sided**. This paper shows they are not:

| Boundary | Formula | Latency exponent | Authority (k_eff) exponent |
|---|---|---|---|
| Ceiling | Kp_ceiling ≈ 380 / latency | **−1** | **0** (independent) |
| Floor   | Kp_floor   ≈ 0.06 × k_eff × latency | **+1** | +1 |
| **Window** | **≈ 6300 / (k_eff × latency²)** | **−2** | −1 |

Both boundaries were derived independently from first principles (ceiling: DIPDT phase-margin analysis; floor: bang-bang blind-spot impulse), both measured empirically and confirmed (ceiling exponent measured −1.036, floor exponent +0.96 — each within 6% of theoretical integer), and together they predict **each doubling of control-loop latency compresses the gain window 2^1.88 = 3.7×, not 2^1.0 = 2.1× as prior ceiling-only theory predicted**. The floor rising simultaneously with the ceiling falling is the central finding — the "double squeeze."

The practical payoff is concrete. Going from an Arduino (latency ≈ 10 steps, 50 ms) to a Teensy (latency ≈ 2 steps, 10 ms):
- Ceiling improves: 380/10 = 38 → 380/2 = 190 (5× higher)
- Floor improves: 0.06 × k_eff × 10 → 0.06 × k_eff × 2 (5× lower)
- Net window improvement: 5 × 5 = **25×** — not 5× as ceiling-only theory would predict.

A secondary corollary: k_eff = T_avg × (π/180 × 15/12) × L_nozzle / Iyy contains no max_gimbal_deg. The gimbal angle cancels from both the ceiling and the floor formula. **A builder assessing gain-window risk does not need to measure their gimbal angle** — only motor thrust, nozzle moment arm, and moment of inertia are required. This was confirmed by exhaustive regression (n = 82): adding log(max_gimbal) to the k_eff + latency model gives delta CV R² = −0.003 (pure noise).

The gain window can be expressed as a single dimensionless parameter:
> **Π = k_eff × τ²** &emsp; [dimensionless] — plant aggressiveness relative to delay squared

where k_eff [s⁻²] = T_avg × (π/180 × 15/12) × L_nozzle / Iyy (angular acceleration per radian of gimbal deflection) and τ = latency_steps / 200 Hz. This is the delay-margin form that appears in classical analysis of unstable plants under delay: ω_n² ∝ k_eff × Kp, so ω_n × τ ∝ √k_eff × τ and ω_n² × τ² ∝ k_eff × τ². Π measures how aggressively the plant responds relative to how long the controller is blind.

Π predicts window_ratio = Kp_ceiling/Kp_floor with CV R² = 0.546 (window_ratio v2, n = 82 non-censored designs). To confirm Π_A = k_eff × τ² is the correct form rather than an arbitrary choice, four candidate parameters were tested on the same data, varying both the τ exponent and whether maximum gimbal stroke δ_max belongs:

| Candidate | Motivation | CV R² |
|---|---|---|
| k_eff alone | authority only, no delay | 0.368 |
| k_eff × τ¹ | linear in delay | 0.592 |
| **Π_A = k_eff × τ²** | **ceiling/floor derivation (this paper)** | **0.602** |
| Π_B = k_eff × δ_max × τ² = θ̈_max × τ² | blind-spot at max authority (saturation framing) | 0.570 |

The τ exponent sweep peaks at τ^1.5 (CV = 0.608) — 0.006 above τ^2 — but this gap is within noise at n = 82 (5-fold CV SE ≈ ±0.05). The free-fit exponent on latency is −1.765, consistent with −2 within one standard error. The free-fit keff exponent is −1.097, consistent with −1.000 (the theoretical prediction). After removing k_eff + latency from the residuals, r(log δ_max, residual) = −0.035: essentially zero signal despite 6.2× variation in δ_max across designs. Π_B (which includes δ_max) scores 0.032 lower — confirming δ_max is noise. The physical reason: during wind rejection the servo saturates at its *slew rate*, not at δ_max; the bang-bang amplitude is governed by how aggressively the system rotates per unit correction (k_eff), not by how large a deflection is available. The ~40% unexplained variance is confirmed stochastic (within-cell variance ≈ total unexplained residual).

The original "θ̈_max predicts gain sensitivity, R² = 0.33" finding from the continuous regression (n = 262 designs, Section 4.0) survives and is the empirical backbone. The thesis above is the mechanistic explanation *behind* that correlation, derived from two independent mechanisms that each predate this project and are separately validated here.

One additional result has cross-architecture theoretical weight: ADRC's extended success at high Π is explained by its ESO canceling wind disturbances *upstream of servo saturation*, preventing the servo from entering bang-bang mode. A controlled saturation test (n = 15 designs, 2×2 factorial) found PID-without-saturation achieves SR = 1.000 for all 15 designs including the extreme cases (Pi = 12,648) where PID-with-saturation achieves only 0.60 — saturation is both necessary and sufficient for PID failure in this range. ADRC's ESO keeps the effective control authority low enough that the servo never saturates (slew_frac = 0.00 in the same tests), inheriting the PID-nosat performance without requiring an ideal servo. This is specifically NOT achievable with integral-action PID: at Pi > 700 (k_eff scale), integral PID has zero valid (Kp, Ki) combinations where ADRC succeeds with one omega_c setting (R2080, n = 50 observer universality test). ESO and integral action are architecturally distinct in the saturated regime even though they are mathematically equivalent in the linear regime (Carlson 2025).

> **Methodology note (two internal audits, 2026-06-15, then a framing revision, 2026-06-17):** the original experiment used a 3-seed binary robustness test, statistically too coarse to resolve its own decision threshold. Two corrections (15-seed reclassification, then a finer joint gain search + 30 seeds) shifted the count of high-sensitivity designs from 45 → 30 → 36 of 2,400, each time strengthening the physical signal (AUC 0.944 → 0.957 → 0.975). A subsequent analysis asked whether the binary framing itself was defensible, given the base-rate/AUC ambiguity and the project's own evidence (the boundary experiment, the Wilson-CI "uncertain" flag covering 33.6% of at-risk designs) that the classification threshold was arbitrary rather than natural. A properly-powered continuous regression (Section 4.0) confirmed the continuum directly — no sharp boundary exists. The binary labels are dropped as a formal taxonomy; they appear in Sections 4.1–4.6 as audit-trail documentation of the experiments that corrected the original classification methodology. Section 4.6 documents the classification audits; Section 4.0 is the primary result.

---

## Abstract

The safe proportional gain window [Kp_floor, Kp_ceiling] for a hobby-scale thrust-vector-control rocket is governed by a dimensionless parameter derived from first principles:

> **Π = k_eff × τ²** &emsp; [rad/CU], &emsp; where k_eff = T_avg × (π/180 × 15/12) × L_nozzle / Iyy and τ = control-loop latency in steps

Window_ratio = Kp_ceiling / Kp_floor scales as Π⁻¹ (CV R² = 0.546, n = 82; theoretical exponent −1.000, observed −1.029). This relationship arises from a ceiling–floor double squeeze: the gain ceiling falls with latency (Kp_ceiling ≈ 380/τ, from DIPDT phase-margin analysis), while the gain floor *rises* with the same latency (Kp_floor ≈ 0.06 × k_eff × τ, empirically confirmed and mechanistically validated). The result is that the window compresses as τ⁻², not τ⁻¹ — each doubling of control-loop latency shrinks the viable gain range by 3.7×, not 2.1× as prior ceiling-only theory would predict. The floor's latency dependence is the novel finding: prior work treated the gain floor as a function of mechanical authority alone.

This Π constraint is architecture-invariant: PID, LQR, and SMC controllers all exhibit Spearman ρ(Π, window metric) ≈ −0.75 on the same 50 designs, confirming the constraint is a property of open-loop linear feedback under delay, not a tuning artifact. ADRC with an Extended State Observer partially decouples from Π by canceling disturbances upstream of servo saturation — a 2×2 factorial test (n = 15) shows PID-without-saturation achieves SR = 1.000 for all designs including the most extreme (Π = 12,648), while PID-with-saturation fails at SR = 0.60; ADRC achieves the no-saturation PID result with a real servo because the ESO prevents saturation from occurring. ADRC extends the feasible frontier 2.4× in Π at standard tuning, and further with an adaptive bandwidth ratio (zero failures to Π = 49,893 with ω₀/ωc = 20).

A properly-powered continuous regression (n = 262 designs, stratified to cover the full Π range) confirms the dose-response relationship empirically: gain margin degrades smoothly from td = 0 to ≈ 300 rad/s², then plateaus with latency becoming the dominant residual risk factor. **R² = 0.36 (in-sample) / 0.33 ± 0.09 (5-fold cross-validated), p = 5.8×10⁻¹¹.** The actionable prevalence: 2% of low-authority designs (td < 40) fall below the 0.80 reliability threshold; 46% of extreme-authority designs (td > 300) do. (A 3D-printed F15-class rocket with Iyy ≈ 0.010 kg·m² and a 10° gimbal sits at θ̈_max ≈ 85 rad/s², within the 80–120 band where 26% fall below 0.80 — the practical regime this paper targets.)

The original binary result is retained as a secondary, simplified decision aid: cross-validated AUC = 0.975, Cohen's d = 3.71 on the final, twice-corrected population (n = 36 of 2,400 "FRAGILE" designs; two internal audits, Section 4.6, found and fixed both an underpowered 3-seed robustness test and an underpowered gain search, each correction *strengthening* the result). Four environmental hypotheses (wind, servo slew, aerodynamic instability, their interactions) were rejected as *predictors of which designs are gain-sensitive* — not as irrelevant to flight in general. A companion fidelity analysis (Section 5.1.1, n = 25 designs) shows the distinction: below θ̈_max = 70 rad/s², no individual fidelity module adds measurable evaluation difficulty; above it, sensor noise and control-loop latency are the hardest evaluation factors (+0.12 SR impact each), while wind has the *smallest* evaluation impact (+0.04 maximum) — consistent with wind's role being to select the correct Kp during tuning, not to make a correctly-tuned design fail. Mechanical authority and control-loop latency are the predictors of *which designs need careful tuning*; all fidelity modules remain relevant to flight itself. The mechanistic chain — Newton → bang-bang oscillation amplitude → Åström–Hägglund ultimate gain K_u → window collapse — was re-verified with a literal relay probe on the final population (K_u: 29.2 median for narrow-window designs vs. 91.2 for wide-window, 3.12×, p = 4.17×10⁻⁷, the tightest of three successive re-derivations).

An exploratory, simulation-based comparison (Section 6.2, n = 46 cells across 6 authority levels × 8 latency levels) finds that ADRC's bandwidth ceiling depends primarily on latency and secondarily on mechanical authority (fitted power law ωc_max ≈ 70/(latency^0.57 · θ̈_max^0.31), R² = 0.823). PID's ceiling barely depends on authority (plant-gain exponent ≈ −0.20); ADRC's authority exponent (−0.31) is 1.5× larger — a measurable but moderate secondary coupling absent from PID. An earlier, smaller sweep (n = 21 cells) suggested a larger authority exponent (−0.77) that did not replicate when the sample was extended; the updated exponents should be read as approximate and subject to hardware validation. To the author's knowledge, no prior work has made this comparison quantitatively for any TVC hardware class. A runnable tool (`tools/gain_advisor.py`) implements both ceiling laws with their uncertainty documented inline; this is a usability deliverable rather than a novel contribution in its own right.

A single test flight at Kp = 2 detects gain-sensitive rockets with AUC = 0.954 (7-seed, final population). Disturbance-free simulator tuning causes 63.9% false rejection for gain-sensitive designs; critically, the 2 genuinely uncontrollable ("INFEASIBLE") designs that survive correction show 100% false *approval* — the one place a disturbance-free simulator gives zero warning rather than excess caution. Active Disturbance Rejection Control (ADRC) at a single, untuned setting closes 83% of the PID gain-sensitivity gap (36 narrow-window designs, Section 6.1); the residual 2/36 failures are explained, and resolved, by the same θ̈_max × latency mechanism — now shown to generalize across a structurally different controller architecture.

---

## 1. Introduction

### 1.1 The question

Hobby TVC rocketeers need a proportional gain that is high enough to reject wind yet low enough to avoid a limit cycle — bang-bang oscillation induced by actuator slew-rate saturation and feedback delay. This project asks: given only the rocket's hardware specifications, can you predict whether that window is wide (forgiving) or narrow (dangerous)? And what physical property determines the answer?

### 1.2 Why this matters

Builders typically tune gains in simplified simulators — no wind, no slew limits, no sensor noise. In a disturbance-free simulator, the natural tuning approach is to find the Kp that produces the fastest, cleanest step recovery: higher Kp recovers faster; too high and the discrete-time control loop starts to ring. A builder watching recovery speed in still air will converge on a Kp that makes physical sense for that objective.

The problem is that step-recovery speed in still air gives no information about two quantities that dominate real-world flight success: the *wind rejection floor* (minimum Kp needed to damp gusty disturbances before they grow unbounded) and the *limit-cycle ceiling* (maximum Kp before slew-rate saturation under gusty conditions drives bang-bang oscillation). Both boundaries exist in real flight and are absent from the disturbance-free simulator. A gain that optimizes step-recovery in still air can sit below the wind floor (the rocket can't reject gusts), above the limit-cycle ceiling (it oscillates in gusts), or within the window — and still-air step response contains no signal that distinguishes these three cases. For gain-sensitive rockets, the gain selected by step-response tuning falls outside the real-physics gain window **56% of the time** (Section 5.1). The builder sees a well-tuned simulator, flies the rocket, and observes poor or inconsistent performance — not because the design is uncontrollable, but because the tuning happened in a simulator that concealed the two boundaries that actually constrain the answer.

### 1.3 The wrong hypotheses (and how they were found to be wrong)

Before finding the correct answer, four hypotheses were tested:

| Hypothesis | Prediction | Result (n = 2,400) |
|-----------|------------|--------|
| H1: High wind narrows the gain window | r(wind, gain sensitivity) > 0 | r = **−0.008**. Rejected. |
| H2: Slow servos narrow the gain window | r(slew, gain sensitivity) > 0 | r = **+0.048**. Rejected. |
| H3: Aerodynamic instability narrows the gain window | r(stability, gain sensitivity) > 0 | r = **−0.001**. Rejected. |
| H4: Interactions (wind × slew, etc.) | Best ∆gain-window prediction ≥ 0.03 | All deltas negative. Rejected. |
| H5: Control loop latency (hardware) | ∆gain-window prediction ≥ 0.03 | ∆AUC = **+0.028**; combined AUC = **0.972**. **Confirmed.** |

None of the environmental variables (H1–H4) improve gain-window prediction by the minimum meaningful threshold. High-sensitivity designs (θ̈_max above the continuous inflection point near 120 rad/s²) distribute uniformly across all wind levels, all slew rates, and all aerodynamic stability values. A fifth factor — control loop latency, a hardware design parameter — was identified after the initial analysis and is addressed in Section 4.2.

The correct answer was not a new physical quantity — it was Newton's second law, applied to predict not just actuator sizing (its textbook use) but gain sensitivity, with control loop latency providing independent additional discrimination.

### 1.4 Scope and where to focus

This paper has two central, load-bearing contributions, in order of priority:

1. **The continuous gain-margin relationship (Section 4.0)**: θ̈_max and control-loop latency predict a continuous degree of gain-margin loss (R² = 0.33, n = 262, 5-fold cross-validated), not membership in a binary class. This is the scientifically more defensible claim and the one a skeptical reader should evaluate first — it survives the base-rate/AUC objection that the binary framing does not, and it is backed by a properly-powered sample specifically designed to resolve the shape of the relationship (not a small ad-hoc boundary experiment).
2. **The cross-architecture ceiling comparison (Section 6)**: Exploratory simulation evidence (n = 46 cells, extended from an earlier 21-cell estimate) shows ADRC's bandwidth ceiling depends primarily on latency with a secondary authority effect. The original 21-cell estimate suggested comparable authority and latency exponents (both ~−0.7); the extended 46-cell estimate revises this to latency-dominated (−0.57) with a weaker but real authority effect (−0.31). Both exponents are exploratory; the direction — ADRC shows more authority coupling than PID — is stable across both sample sizes. A runnable tool (`tools/gain_advisor.py`) implements both ceiling laws with their uncertainty documented inline (Section 6.3).

Everything else — the binary classifier and its two-stage audit (Sections 3, 4.2–4.6), the K_u mechanistic chain (Section 4.4), the simulator-to-real false-rejection study (Section 5.1), and the flight-detection workflow (Section 5.2) — is retained because it documents real, carefully-audited evidence for the underlying mechanism and because the binary label remains a useful simplification for a builder who wants one number to check. None of it is the headline claim. A reader short on time should read Sections 4.0 and 6 first; everything else supports or qualifies those two results.

---

## 2. Background

### 2.1 What is already known

The authority/inertia ratio T_max × δ_max / Iyy appears in every spacecraft GNC textbook as an actuator sizing criterion (Wie 1998; Sidi 1997). The fact that over-actuated systems have narrow gain margins is fundamental robust control theory (Doyle, Francis, Tannenbaum). Neither observation is new.

### 2.2 What this project adds

1. **The floor-latency dependence — a finding not in prior hobby-TVC or small-rocket literature** — Kp_floor ≈ 0.06 × k_eff × latency (Section 4.6, window ratio v2, n = 104 non-censored floor measurements, R² = 0.627, CV = 0.594). Prior work on gain ceilings for delay-coupled plants (DIPDT literature; Di Ruscio 2010, ISA Transactions 2017) analyzed only the *ceiling's* latency dependence. The floor's latency dependence was not tested, because the wind-rejection floor is absent from the disturbance-free models that DIPDT analysis uses. Here, stochastic wind is modeled explicitly. The consequence: the correct latency sensitivity of the window for moderate-to-high-authority designs is τ⁻² to τ⁻³ (measured −2.2 to −3.2 within keff tiers; see Section 4.0.1), not τ⁻¹ — averaging 3.7× per doubling across the design space, vs. the 2.1× a ceiling-only analysis would predict. Low-authority designs (keff ≲ 5) show only τ^−0.85 ≈ 1.8× per doubling because the floor barely rises with latency (they remain wide-windowed regardless of MCU speed). The builders who most need to upgrade MCU are exactly those already in the narrow-window regime, where the sensitivity is 4–9× rather than 2×.

2. **The Π = k_eff × τ² invariant and the max_gimbal corollary** — k_eff = T × 0.02182 × L / Iyy contains no max_gimbal_deg. Exhaustive regression (n = 82, all available design factors) finds max_gimbal adds delta CV = −0.003 (noise). Adding all 9 available factors together gives CV R² = 0.536, worse than the 2-parameter baseline (0.602). Within-cell variance (1.32) ≈ total unexplained residual (1.40) — the unexplained variance is stochastic, not a missing predictor. **Practical corollary: a builder assessing gain-window risk does not need to know their gimbal travel, only T, motor_scale, L_nozzle, Iyy.**

3. **Dual bang-bang regimes** (`tools/bang_bang_transition.py`, Section 4.4.1) — Kp sweeps with simultaneous slew-saturation measurement reveal two physically distinct operating regimes. *Low-authority designs* (td ≈ 20 rad/s²): servo saturation fraction ≈ 0.06 throughout the gain window — the system operates linearly; bang-bang begins only when Kp exceeds the ceiling (visible as a sharp SR drop with slight slew jump). *High-authority designs* (td ≈ 200 rad/s²): slew_frac ≈ 0.63 **at every Kp**, including below floor — the wind alone drives permanent servo saturation regardless of gain choice. The gain window is not the range before bang-bang starts; it is the range where bang-bang amplitude is small enough to pass the success criterion. The floor mechanism for high-authority designs is therefore "enough Kp to damp the wind-driven bang-bang oscillation to acceptable amplitude," not "enough Kp to avoid bang-bang starting" — explaining why floor ∝ k_eff × latency rather than something based on the open-loop linear response.

4. **A continuous gain-margin regression, properly powered to resolve its shape** (Section 4.0) — n = 262 designs stratified specifically to cover the full θ̈_max range, showing a statistically robust dose-response relationship (R² = 0.36 in-sample, 0.33 ± 0.09 cross-validated, p = 5.8×10⁻¹¹) monotone from td = 0 to ≈300 rad/s², plateauing above that with latency as the binding residual risk factor. This is the primary empirical result that motivated the window decomposition above.

5. **ESO causal isolation and observer specificity** (Section 4.0.2.1, Finding 8) — saturation test: PID-without-saturation achieves SR = 1.000 for all 15 tested designs (including Pi = 12,648 where real-servo PID achieves 0.60). Saturation is both necessary AND sufficient for PID failure. ESO prevents saturation (slew_frac = 0.00 in all ADRC cases); integral action does not — at Pi > 700 (k_eff scale), PID-I has zero valid (Kp, Ki) combinations where ADRC finds one (R2080). This closes the loop from mechanism to architecture: the floor-latency constraint is broken specifically by upstream disturbance cancellation, not by any amount of integral action.

6. **Controller-invariance across three feedback architectures** (Section 4.0.2) — LQR (40 Q/R ratios), SMC (6 slopes × 20 Kp), and PID all reproduce Spearman ρ(Π, window) ≈ −0.75 on the same 50 designs. The constraint is not a PID parameterization artifact; it is a property of open-loop linear feedback without disturbance estimation under delay.

7. **Performance frontier and ADRC extension** (Section 4.0.3) — optimal PID first fails (SR < 0.90) at **Π = 320** (k_eff-based), rising to SR < 0.80 at **Π = 870**; ADRC (ω₀/ωc = 5) first fails at **Π = 1,200**; with adaptive ω₀/ωc up to 20, zero failures up to **Π = 5,648** (ADRC SR = 0.93 at the extreme, Pi > 5,600 not yet tested). Updated design rule (all Π = k_eff × τ²): Π < 300 → PID safe; Π 300–900 → PID degrading, ADRC standard safe; Π 900–1,600 → ADRC with ω₀/ωc = 8–20; Π 1,600–5,600 → ADRC with ω₀/ωc = 20. Note: prior versions cited Pi_td = θ̈_max × τ²; the corrected parameter is Pi_keff = k_eff × τ² (smaller by a factor of u_max ≈ 4–15, depending on max_gimbal). ρ(Π, SR_adrc) = −0.325 vs ρ(Π, SR_pid) = −0.668 — ESO decoupling halves the Π sensitivity.

8. **An exploratory cross-architecture ceiling comparison** (Section 6) — ADRC bandwidth ceiling ωc_max ≈ 70/(latency^0.57 · θ̈_max^0.31), R² = 0.823 (n = 46 cells). PID's authority exponent ≈ −0.20; ADRC's ≈ −0.31 (1.5× larger). To the author's knowledge, no prior work has made this comparison quantitatively for any TVC hardware class.

9. **Mechanistic chain** — Newton → bang-bang amplitude → K_u → window collapse, re-verified via a literal relay probe on the final population (K_u: 91.2 vs 29.2 median, 3.12×, p = 4.17×10⁻⁷, Section 4.4).

10. **Flight detection** — AUC = 0.954 [0.907, 0.989] from a single probe flight at Kp = 2 (7-seed RMS, final n = 36 narrow-window designs, Section 5.2). Recommended threshold: RMS > 7.6° → narrow-window confirmed; re-tune within [Kp_floor, Kp_ceiling] from specs.

11. **Domain specificity and secondary classification result** — hobby-scale TVC (mass = 0.5–1.2 kg, low-cost servos, F15-class motors) is not covered in prior literature. The binary classifier (AUC = 0.975, Cohen's d = 3.71) is retained as a secondary simplified decision rule, backed by a two-stage audit where each correction strengthened the result.

### 2.3 What this project does NOT claim

θ̈_max = T · sin(δ) · L / Iyy is not new. The formula is Newton's second law. The discovery is not that the formula exists — it is that it predicts gain sensitivity in a completely different context from its textbook use, quantified across 2,400 designs, with competing explanations systematically rejected.

This project does **not** claim that AUC = 0.944 constitutes a physical law, or that the correlation is causal in a strict experimental sense. The claim is more specific: a compact invariant derivable from rigid-body dynamics strongly correlates with gain-window collapse in simulation, **outperforms trained machine learning models** operating on all raw design parameters (see Section 4.3), and has a mechanistic explanation spanning from hardware specs to bang-bang oscillation amplitude to K_u separation (Section 4.4). The formula identifies a specific nonlinear combination — not merely a proxy for individual parameters.

---

## 3. Methods

### 3.1 Design space

Latin Hypercube Sampling, n = 2,400. Parameter ranges targeting realistic hobby hardware:

| Parameter | Range | Notes |
|-----------|-------|-------|
| mass | [0.50, 1.20] kg | F15 motor T/W=1 breaks at ~1.47 kg |
| Iyy | [0.005, 0.100] kg·m² | 3D-printed rockets: Iyy ≈ 0.004–0.015 |
| servo_slew_deg_s | [60, 200] | Realistic hobby hardware |
| static_margin | [−0.30, 0.30] | Signed: positive = stable (CP aft of CG), negative = unstable (CP forward of CG); standard aerospace convention |
| motor_scale | [0.5, 3.0] | Scales thrust |
| max_gimbal_deg | [2, 15] | Physical gimbal travel |
| latency_steps | [1, 6] | 5–30 ms at 200 Hz |
| wind_strength | [0.05, 0.45] | Ornstein-Uhlenbeck |

T/W > 1 filter enforced (unliftable designs resampled). Airframe fixed: 0.5 m length, 50 mm diameter, l_nozzle = 0.25 m. The Iyy range was expanded from [0.010, 0.040] to [0.005, 0.100] to match real 3D-printed hobby rockets; the narrower old range artificially prevented most designs from reaching θ̈_max > 70 rad/s².

### 3.2 Full-physics simulator

10 toggleable modules: nonlinear aerodynamics, dynamic pressure, thrust curve, CG shift, servo slew limits, backlash, deadband, latency (3 steps), Ornstein–Uhlenbeck wind, sensor noise. All experiments use full fidelity (all modules active).

### 3.3 Gain window characterization

The n = 2,400 survey is analyzed as a **continuous distribution** of gain window widths, not a set of discrete regimes. The primary result is Section 4.0; the summary across the survey (from the n = 262 stratified sample, `regression_pooled_py.csv`):

| θ̈_max bin (rad/s²) | n | Mean over-robustness margin | % of designs with margin < 0.80 |
|---|---|---|---|
| < 40 | 103 | 0.980 | 1.9% |
| 40–80 | 36 | 0.985 | 0.0% |
| 80–120 | 24 | 0.953 | 4.2% |
| 120–180 | 23 | 0.875 | **26.1%** |
| 180–300 | 24 | 0.806 | **41.7%** |
| > 300 | 52 | 0.772 | **46.2%** (plateau — latency-dominated) |

Three qualitatively distinct operating regions emerge from this curve, with no sharp boundaries:
- **Wide-window (θ̈_max < 80 rad/s², ~97% of designs):** gain can vary 50–300× without failure; any tuning approach works.
- **Narrowing-window (θ̈_max 80–300 rad/s²):** 4–42% of designs fall below the 0.80 reliability threshold; disturbance-free simulator tuning misses the window ~40–64% of the time; full-physics tuning required.
- **Two genuinely uncontrollable designs (0.08% of n = 2,400):** no proportional gain achieves >35% success rate under wind regardless of search quality. A disturbance-free simulator gives zero warning for these — the strongest argument in this paper for flight testing before committing to hardware.

**Technical gain search (`autotune_continuous`):** Kp log-search [1, 320] (10-point coarse + 6-point refine); Kd probe [1, 4, 16, 64] at Kp = 40; primary objective: 2-seed mean success rate; tiebreak: RMS.

**Historical binary classification note.** Sections 4.1–4.6 retain terminology ("high-sensitivity" designs, originally labeled "FRAGILE" in the experiment) because the two-stage classification audit (Section 4.6) is part of the paper's methodological record and cannot be described without reference to what was being corrected. These labels are historical artifacts of the experimental methodology, not a scientific taxonomy: Section 4.6 details how two corrections moved which specific designs were in this set, and Section 7.4 quantifies why no sharp boundary separates it from the rest. The original classification experiment used exactly 3 binary pass/fail seeds (success rate can only take {0, 1/3, 2/3, 1}), which is too coarse to resolve a threshold sitting between 2/3 and 1.0 — see Section 4.6 for the full audit trail.

**Gain window vs. classical gain margin.** In the frequency domain, gain margin is the factor by which the open-loop gain can be increased before the closed-loop system becomes unstable, derived analytically from the loop transfer function. This paper's "gain window" is a time-domain, stochastic analogue: the range of Kp values [Kp_floor, Kp_ceiling] over which a design achieves success rate ≥ 0.80 in the full-physics simulation with wind. The floor is set by wind-rejection requirements — below it, the Ornstein–Uhlenbeck disturbance accumulates faster than the proportional correction rejects it. The ceiling corresponds to the onset of the limit cycle described in Section 4.4: actuator slew-rate saturation at high Kp drives persistent bang-bang oscillation. The ratio Kp_ceiling / Kp_nominal is operationally equivalent to a classical gain margin (measured statistically rather than analytically); the ratio Kp_ceiling / Kp_floor is the window width — the quantity this paper predicts from hardware specs.

### 3.4 Physical predictor computation

From the exp1 CSV columns (motor_scale, max_gimbal_deg, Iyy, mass):

```
T_avg     = 14.4 × motor_scale            [N; from motor model]
keff_full = T_avg × (π/180 × 15/12) × 0.25 / Iyy   [rad/s² per CU]
u_max     = max_gimbal_deg × 12/15        [CU]
θ̈_max    = keff_full × u_max             [rad/s²]
```

`CU_TO_RAD = π/180 × 15/12 = 0.02182` is constant (the 15° reference gimbal cancels out, making keff_full independent of max_gimbal_deg).

### 3.5 AUC estimation

Full-data AUC reported with 95% bootstrap CI (n = 10,000 resamples). Cross-validated AUC from 10-fold stratified k-fold reported as mean ± std. All screening deltas (∆AUC) computed against θ̈_max alone as baseline; threshold for "meaningful improvement" is ≥ 0.03.

---

## 4. Results

### 4.0 The continuous gain-margin relationship (PRIMARY RESULT)

**Motivation.** Every result in Sections 4.1–4.6 below operationalizes "gain sensitivity" as a binary label (FRAGILE vs. EASY) produced by thresholding a 3-condition robustness test at 0.80. Two things about that framing invite legitimate skepticism. First, AUC computed against a ~1.5% base rate is easy to misread as strong "accuracy" — a classifier that always predicts EASY already achieves ~98.5% accuracy and is useless, and AUC (while a real, non-base-rate-distorted ranking statistic) does not by itself communicate that the corresponding precision at any practical threshold is only ~12%. Second, this project's own boundary experiment (Section 4.6.3) and the Wilson-CI "uncertain" flag (33.6% of at-risk designs straddle a decision threshold even at 30 seeds, Section 4.5.7) both pointed toward FRAGILE/EASY being a thresholded view of a continuum rather than two physically distinct populations — a claim Section 7.4 made qualitatively but never tested with adequately-powered continuous data.

**This section tests that directly**, with a new, purpose-built experiment rather than reused classification by-products.

**Method.** Two sequential passes (`tools/continuous_margin_regression.py`, `tools/elbow_characterization.py`), both using the same finer joint Kp×Kd gain search as the final classification correction (Section 4.5.7) so the result is not confounded by an inadequate search:

1. **Pass 1 (n = 130):** designs drawn from a 4,000-design LHS pool, stratified by θ̈_max decile to guarantee coverage of the rare high-authority tail a plain LHS draw would under-sample.
2. **Pass 2 (n = 92):** a targeted follow-up sampling 20 designs from each of 5 bins spanning θ̈_max ∈ [40, 1,000] rad/s², after Pass 1's naive global regression returned a misleadingly weak, near-zero result (R² = 0.075, Spearman ρ = −0.032) that turned out to be an artifact of only 8 of 130 points falling above the region where the relationship actually changes — a lesson in checking binned means before trusting a single correlation coefficient on a stratified sample.

For every sampled design in both passes: run the finer joint gain search (3 seeds, 126 Kp×Kd combinations), then evaluate the **continuous** over-robustness success rate at 1.4× the found gains with 15 fresh seeds (a fraction in [0, 1], never binarized). Over-robustness is the dominant failure mode in the high-sensitivity population (~87% of narrow-window designs are ceiling-limited), so it is the natural continuous analogue of the binary classification threshold.

**Result (original n = 222 + targeted extension n = 40 = n = 262 pooled, `experiments/results/regression_pooled_py.csv`):**

The original two passes (n=222) had only 12 designs above θ̈_max = 300 rad/s², making the high-authority tail poorly powered. A targeted extension (`tools/regression_extension.py`, seeds 8001–8015 disjoint from all prior seed ranges) sampled 40 additional designs specifically from the td ∈ [300, 500] rad/s² region (drawn from a 20,000-design LHS pool to ensure adequate coverage of this rare, extreme parameter combination), using the identical gain-search and 15-seed evaluation protocol.

| θ̈_max bin (rad/s²) | n (original) | n (pooled) | Mean over-robustness margin | Fraction below 0.80 |
|---|---|---|---|---|
| < 40 | 103 | 103 | 0.980 | 1.9% |
| 40–80 | 36 | 36 | 0.985 | 0.0% |
| 80–120 | 24 | 24 | 0.953 | 4.2% |
| 120–180 | 23 | 23 | 0.875 | 26.1% |
| 180–300 | 24 | 24 | 0.806 | 41.7% |
| > 300 | 12 | **52** | **0.772** | **46.2%** |

**Key additional finding from the extension:** within the td ∈ [300, 500] range, r(log θ̈_max, margin) = +0.067 (p = 0.68) — essentially zero. Mean over_sr does not continue declining above td = 300. The dose-response curve is **monotone from td = 0 to td ≈ 300, then plateaus.** This is physically consistent with the ceiling equation (Kp_max ≈ 380/latency): once the gain ceiling is already pressed down by latency, increasing authority further cannot worsen the margin, because the hard constraint is already the latency-determined ceiling, not authority. At td > 300, the variance in over_sr is dominated by **latency** (designs with latency = 1 achieve over_sr ≈ 1.00; designs with latency = 5–6 average ≈ 0.55), not by how far above 300 the td value falls. This is the regime where θ̈_max alone provides diminishing returns as a predictor, and the log(θ̈_max × latency) combined predictor (Section 4.2) is essential.

**Pooled statistics (n = 262, `experiments/results/regression_pooled_py.csv`):** R² = 0.364 (in-sample), **5-fold cross-validation: R² = 0.325 ± 0.086** — narrower CI than the original ±0.103, and higher point estimate. Coefficients: intercept = 1.234, log θ̈_max = −0.051, log latency = −0.101 (both negative, consistent with the established mechanism). The latency coefficient's magnitude now exceeds the θ̈_max coefficient's magnitude (−0.101 vs. −0.051) in the pooled fit, reflecting the extension data's confirmation that latency, not authority, is the binding constraint in the extreme high-authority regime.

**This R² is modest in absolute terms (≈33% of variance explained in the pooled sample, 5-fold CV R² = 0.325 ± 0.086) and that is the honest, intended result.** A modest, well-estimated, cross-validated R² on a continuous outcome is a stronger and more falsifiable claim than a high AUC on a binary outcome at a rare base rate, precisely because there is no threshold to choose, no precision/base-rate tension to explain away, and no way to inflate the headline number by picking a favorable cutpoint. The relationship is real, modest, and statistically robust — and that is what the data show.

**What accounts for the remaining ~67% of variance?** Three sources, not an unidentified confound. First, stochastic variation: each design's over-SR is evaluated on 15 random wind seeds; the same design re-evaluated on 15 different seeds would give a different number (estimated seed-to-seed SR variance ≈ ±0.12–0.20 for designs near the 0.80 boundary, the Wilson-CI analysis in Section 4.5.7 confirms this). This floor of irreducible noise alone accounts for a substantial fraction of the residual variance. Second, other design parameters: wind_strength, servo_slew_deg_s, backlash, and deadband individually affect the SR of specific designs but do not systematically predict whether a design belongs in the declining-margin population — consistent with the H1–H4 rejection in Section 4.2. Third, the formula is a population-level trend predictor, not an individual-design predictor: θ̈_max tells you the *expected* margin degradation at a given authority level, not the exact margin of any single design. An analogous situation is using age to predict blood pressure — the trend is real and actionable, the individual-level R² is modest, and the unexplained variance reflects person-to-person variation that age does not capture, not a missing confound in the model. The claim this paper makes is not "θ̈_max predicts any individual design's exact margin" (R²=0.33 would not support that) but "θ̈_max predicts how gain-margin risk *scales* with authority, and that scaling is monotone, mechanistically explained, and confirmed by a controlled experiment where only Iyy varies" (Section 4.0.1, which removes all confounds).

**Why window_ratio is a more direct continuous metric than over-SR.** The n = 262 over-SR regression (CV R² = 0.325) has a structural limitation: over-SR is right-censored at 1.0 for wide-window designs, which all score 0.98–1.00 regardless of how wide their window actually is. The theoretically natural direct measure is window_ratio = Kp_ceiling / Kp_floor, which measures the gain margin width the ceiling/floor formulas predict. A separate 120-design stratified sweep (`tools/window_ratio_resweep_v2.py`) directly measures both floor and ceiling per design via a 32-point Kp sweep; on the non-censored subset (n = 82 of 116 valid, 29% censoring), the keff + latency two-variable model achieves CV R² = 0.616 ± 0.035 — nearly 2× the over-SR CV R² of 0.325. This confirms the theoretical expectation: when the outcome variable actually measures what the theory predicts (gain window width), the predictor's explanatory power roughly doubles. The over-SR regression's modest R² is at least partly a measurement artifact from using a censored outcome.

**Universal dimensionless parameter.** The mechanistically correct single parameter is **Π = k_eff × τ²**, where k_eff = T × C_u × L / I_yy is the angular acceleration per control unit (independent of max gimbal angle). This is not an empirical curve-fit: the window formula gives ceiling/floor ∝ (τ⁻¹)/(k_eff × τ) = 1/(k_eff × τ²) exactly. The k_eff coefficient in a joint regression (k_eff + θ̈_max + latency on the window_ratio dataset) is −1.001 — identical to the theoretical prediction of −1.000 — while the θ̈_max coefficient is −0.088 ≈ 0, confirming that the max-gimbal factor (which separates θ̈_max from k_eff) adds only noise. On the window_ratio dataset (n = 82 non-censored): Π = k_eff × latency² achieves single-param CV R² = **0.546** vs θ̈_max × latency²: 0.452 — a meaningful improvement. An earlier draft of this paper used Π = θ̈_max × latency²; the correction does not change any qualitative conclusion but removes an unmotivated factor of u_max from the formula. **Physical interpretation:** Π [rad/CU] is the angular displacement accumulated *per control unit of applied effort* during one latency window. The builder formula is keff = F15_avg × motor_scale × 0.02182 × l_nozzle / Iyy — no max-gimbal measurement required. Whether this collapses to the same invariant across different hardware classes is an empirical question; the current result is consistent with it.

**A secondary finding worth flagging:** Pass 1's properly-powered low-θ̈_max region (n = 103 below 40 rad/s², many at latency_steps = 6) shows essentially perfect margin (mean 0.98) under the finer gain search — including combinations that the *original*, coarser `autotune_continuous` search had flagged as "mild FRAGILE, latency-driven" (the two false negatives D800/D1523 in Section 4.5, both θ̈_max ≈ 36–38 with latency = 6). This raises the possibility that those two false negatives were partly a search-quality artifact rather than a pure latency effect — consistent with this project's repeated finding that a better gain search shrinks, rather than grows, the gain-sensitive population (Section 4.5.7). This is not yet confirmed for those exact two designs and is flagged as a follow-up, not asserted as fact.

**Why the rest of this paper still discusses a binary label.** A continuous score is the more defensible scientific claim, but a builder deciding whether to fly a specific design still needs a yes/no answer at some point. Sections 4.1–4.6 retain the binary framing as a documented, heavily-audited simplification of the relationship established here — readers should treat the AUC and Cohen's d figures below as describing how well a *threshold* on this same continuous quantity recovers a particular (somewhat arbitrary) decision rule, not as a separate or stronger claim than this section.

### 4.0.1 A controlled, single-airframe confirmation

Every comparison so far (Section 4.0's stratified sample, the binary FRAGILE-vs-EASY classification, the boundary experiment in Section 4.6.3) compares *different designs* that differ in θ̈_max among other parameters. An external review of an earlier draft pointed out that the cleanest possible test of the central claim is a true controlled experiment: take one airframe, vary only the parameter that changes θ̈_max, hold everything else — mass, motor scale, gimbal limit, servo slew, latency, wind, deadband, backlash — fixed, and measure how the success-rate-vs-Kp curve changes. No prior experiment in this project did this.

**Method (`tools/matched_configuration_test.py`, extended by `tools/matched_config_extended_kp.py`).** One fixed reference airframe (mass = 0.80 kg, motor_scale = 1.5, max_gimbal_deg = 10, servo_slew = 120°/s, latency_steps = 3, wind_strength = 0.20, static_margin = +0.10 (standard aerospace convention: CP aft of CG, slightly stable), deadband = 0.05, backlash = 0.10 — every parameter identical across all 18 runs below). Only **Iyy** is varied, across 18 log-spaced values spanning the study's full range [0.005, 0.100] kg·m², giving θ̈_max from 9.4 to 188.5 rad/s². For each Iyy, Kp is swept over 40 log-spaced points from 1 to 1,280 (4× the original grid cap; Kd fixed at 2.0, identical across all configurations), 10 seeds per point with fresh seeds disjoint from all prior experiments — producing true floor and ceiling measurements uncensored by the grid cap.

**Result:**

| Iyy (kg·m²) | θ̈_max (rad/s²) | Kp floor | Kp ceiling | Window ratio |
|---|---|---|---|---|
| 0.0050 | **188.5** | 10.9 | 354 | **33×** |
| 0.0060 | **158.0** | 9.0 | 354 | **39×** |
| 0.0071 | **132.5** | 9.0 | 512 | **57×** |
| 0.0085 | **111.1** | 6.3 | 512 | **82×** |
| 0.0101 | **93.1** | 3.0 | 426 | **142×** |
| 0.0121 | 78.1 | 1.7 | 512 | 295× |
| 0.0144 | 65.5 | 4.3 | 512 | 118× |
| 0.0172 | 54.9 | 4.3 | 512 | 118× |
| 0.0205 | 46.0 | 3.6 | 426 | 118× |
| 0.0244 | 38.6 | 2.5 | 426 | 170× |
| 0.0291 | 32.4 | 2.1 | 426 | 204× |
| 0.0347 | 27.1 | 2.5 | 354 | 142× |
| 0.0414 | 22.7 | 2.1 | 295 | 142× |
| 0.0494 | 19.1 | 2.5 | 295 | 118× |
| 0.0589 | 16.0 | 2.1 | 246 | 118× |
| 0.0703 | 13.4 | 1.4 | 204 | 142× |
| 0.0838 | 11.2 | 1.4 | 246 | 170× |
| 0.1000 | 9.4 | 1.0 | 204 | 204× |

**The extended grid (to 1,280) found true ceilings within range for every configuration — no design is censored.** In the high-authority zone (θ̈_max from 93 to 189 rad/s² — the range the rest of this paper calls "elevated/high risk"), the window narrows **sharply and monotonically: 142× → 82× → 57× → 39× → 33×**, with no other parameter changed. This is the cleanest demonstration in the project that authority relative to inertia, and not some confound between different designs, narrows the gain window — because here there is no other variable that could be doing it.

**Mechanism:** both components contribute, but differently. The **floor rises steeply**: from 3.0 at td = 93 rad/s² to 10.9 at td = 189 rad/s² (a 3.6× increase). The **ceiling also drops, but modestly**: from 512 at td = 133 to 354 at td = 158–189 (a 1.4× drop). The floor-rising effect dominates. This is consistent with the bang-bang analysis in Section 4.4: over-actuated rockets require a higher minimum Kp to reject wind (floor rises), while the stability ceiling — set primarily by control-loop latency (Section 4.6's gain ceiling equation) rather than by plant authority — moves less dramatically at this latency level (latency = 3 steps, ceiling ≈ 380/3 ≈ 127 theoretically, but practical ceiling is higher because the SR = 0.80 threshold gives more headroom than a hard oscillation onset).

The low-authority zone (θ̈_max < 80) does not show a clean monotonic trend in window ratio. The ceilings in this zone (204–512) appear lower than the high-authority zone's 354–512, which may seem counterintuitive — but this is an artifact of 10-seed stochastic estimation near SR = 1.0. At very low authority, a correctly-tuned design achieves SR ≈ 1.00 at essentially every tested Kp; the "ceiling" boundary (where 10-seed SR drops below 0.80) is then set by random seed-to-seed variation rather than a true physical onset. These are not meaningfully comparable to the sharp physical ceilings measured in the high-authority zone (td = 93–189), where all 10 seeds consistently agree on pass/fail because the limit cycle is a real, deterministic failure mode. **The search was already extended to 1,280 and confirmed that NO ceiling in the table is artificially clipped at the old 320 cap.** The ceiling of 354 at td = 189 is a real physical limit — at Kp > 354, the design genuinely fails on 3+ out of 10 seeds due to limit-cycle onset, not sampling noise. The engineering story is entirely in the high-authority zone: floor rises 3.6×, ceiling drops 1.4×, floor-rising is the dominant mechanism.

**What this does and does not establish.** This is still a simulated result — it does not address the deeper objection that every result in this paper depends on the correctness of a simulator not yet checked against real flight data (Section 7.5 / Section 9.0). What it establishes is that, *within this simulator*, the relationship survives the strongest available internal-validity test: a controlled experiment where only Iyy varies. The real-hardware version of this protocol — two or three rocket bodies identical except for redistributed ballast shifting Iyy — is the next direct test this result motivates (Section 8, item F).

Data: `experiments/results/matched_configuration_py.csv`, `experiments/results/matched_configuration_summary_py.csv`, `experiments/results/matched_config_extended_kp_py.csv`.

### 4.0.2 Controller-invariance: LQR and SMC show the same window compression as PID

The Iyy sweep (Section 4.0.1) established window compression in PID. A remaining concern: maybe the {Kp, Kd} coordinate system discretizes gain space poorly, and a different law would find a wide window where PID found a narrow one. Two architecturally distinct alternatives are tested.

**LQR** (`tools/lqr_controller_test.py`): 50 designs × 40 Q/R ratios (0.001–100,000) × 10 seeds. Discrete-time LQR computed via DARE on the linearized plant; window = number of Q/R values achieving SR ≥ 0.80. **SMC** (`tools/smc_controller_test.py`): same 50 designs. Boundary-layer SMC is algebraically equivalent to PID with Kd/Kp = 1/λ_s (fixed ratio); implemented as PID sweeps over 6 λ_s × 20 Kp = 120 combinations. Window = n_pass_total / n_pass_best_lam.

**Results.** All three architectures show essentially identical Π dependence:

| Architecture | Metric | Spearman ρ(Π, window) | p-value |
|---|---|---|---|
| PID | log(window_ratio) | ≈ −0.78 | — |
| LQR | n_pass_QR (out of 40) | −0.747 | 4.8×10⁻¹⁰ |
| SMC | n_pass_total (out of 120) | −0.753 | 2.8×10⁻¹⁰ |
| SMC | n_pass_best_lam (out of 20) | −0.789 | 9.6×10⁻¹² |

Median LQR Q/R values passing by td × latency tier:

| θ̈_max (rad/s²) | Latency 1–2 | Latency 3–4 | Latency 5–6 |
|---|---|---|---|
| < 40 | 40 | 27 | 29 |
| 40–120 | 28 | 17 | 15 |
| 120–200 | 21 | 13 | **5** |
| > 200 | 21 | 13 | **1** |

Two designs find **zero** valid LQR gains: R0475 (td = 193, lat = 5) and R2080 (td = 312, lat = 5). For SMC, every design finds at least one valid (λ_s, Kp) pair — SMC's 6 slope values × 20 Kp values provide more grid coverage than LQR's 40 Q/R values, so the narrow valid region of these extreme designs is found by at least one grid point. This is a search-resolution effect, not an architectural difference; both architectures treat these designs as near-infeasible.

**Interpretation.** LQR minimizes a quadratic cost and finds the globally optimal *linear* feedback for a given Q/R weight. SMC provides bang-bang saturation outside the boundary layer and linear feedback inside it. Neither escapes the Π-driven window compression (ρ ≈ −0.75 for all three). The common structural feature they share — and that ADRC does not — is the **absence of active disturbance estimation**: all three compute control commands from the state error without estimating or canceling the wind disturbance before it enters the feedback loop. This establishes the compression as a physical constraint on feedback-without-estimation under delay and authority, not a property of any specific parameterization.

Data: `experiments/results/lqr_controller_test_py.csv`, `experiments/results/lqr_gain_sweep_py.csv`, `experiments/results/smc_controller_test_py.csv`, `experiments/results/smc_sweep_detail_py.csv`.

**4.0.2.1 Observer universality: is ESO specifically the escape mechanism?**

Section 4.0.2 shows that LQR, SMC, and PID all face ρ(Π, window) ≈ −0.75 when they lack disturbance estimation. ADRC's ESO breaks this constraint (Section 4.0.3). But ESO is not the only form of disturbance estimation — classical integral action (the I in PID) is a standard, well-established disturbance estimator. Does integral-action PID also break the Π constraint, or is ESO specifically required?

**Method (`tools/observer_universality_test.py`):** Same 50 designs, seeds 11001–11010. Three architectures tested: (A) PD (Kp sweep, Ki = 0); (B) PID-I (same Kp sweep × five Ki values: {0, 0.05, 0.2, 1.0, 4.0} = 60 combinations per design, with anti-windup at the actuator limit); (C) ADRC (seven ω_c values, ω₀ = 5ω_c, b₀ = keff — fixed, not per-design optimized). For each architecture, the peak achievable SR and n_pass fraction are recorded. Π = k_eff × τ² throughout (corrected formula).

**Spearman ρ(log Π, peak SR):**

| Architecture | ρ | p-value |
|---|---|---|
| PD (Ki = 0) | −0.594 | 5.5×10⁻⁶ |
| PID-I (best Ki) | −0.488 | 3.3×10⁻⁴ |
| ADRC (ESO) | **−0.283** | **0.046** |

**Spearman ρ(log Π, n_pass fraction):** all three remain strongly negative (−0.749, −0.786, −0.830 respectively) — Π narrows the window for all architectures. ADRC's n_pass rho is the *most* negative, but ADRC's window narrows to 1–2 valid ω_c values rather than collapsing to zero. That single valid setting achieves SR = 1.00 where PD/PID-I have already failed. The key distinction is peak_SR, not window width.

**The decisive design — R2080 (td = 312, lat = 5, Π_keff = 740):** PID-I has *zero* valid (Kp, Ki) combinations (n_pass_pidi = 0). ADRC finds exactly one valid ω_c achieving SR = 1.00. At intermediate Π, integral partially helps — R0236 improves from PD SR = 0.80 to PID-I SR = 1.00 — but at extreme Π, the integral anti-windup mechanism prevents disturbance cancellation from working, while ESO's upstream subtraction succeeds.

**Interpretation.** Integral action does not break the Π constraint. ρ drops from −0.594 (PD) to −0.488 (PID-I) — a marginal improvement, and PID-I still completely fails at the extreme designs where ESO succeeds. The physical reason: the integral accumulates error *after* the disturbance has caused an error, and anti-windup blocks further accumulation once the actuator saturates (the exact condition where cancellation is most needed). ESO estimates the disturbance from the *state trajectory* — from how the system is accelerating — and subtracts it from the control command *before* the actuator limit clip. The subtraction happens upstream of saturation; the servo never saturates; the Π constraint is lifted. This is consistent with Finding 8 (Section 6.1), which proved PID fails because it saturates (PID-nosat = 1.00 for all designs) and ADRC never saturates (slew fraction = 0).

**Caveat:** ADRC's peak_SR rho is now marginally significant (p = 0.046) with the corrected Π = keff × τ² formula. The fixed ω₀/ωc = 5 is not universally optimal — at extreme Π, ω₀/ωc = 12–20 is required (Section 4.0.3). Section 4.0.3 sweeps this parameter explicitly and finds zero ADRC failures with the adaptive ratio. The observer universality experiment uses only the fixed ratio, so ADRC's peak_SR at extreme Π is limited by the fixed setting, not by a fundamental ceiling.

Data: `experiments/results/observer_universality_py.csv` (n = 50, Pi = keff×lat²).

### 4.0.3 Performance frontier: where optimal tuning fails, and how far ADRC extends the boundary

The controller-invariance result (Section 4.0.2) shows that any linear feedback without disturbance estimation faces the same Π-driven narrowing. The natural follow-on question is quantitative: **at what Π does even optimal PID tuning first fail, and by how much does ADRC (with its ESO) extend the achievable boundary?** This section answers both.

**Method (`tools/performance_frontier.py`).** 63 designs from the corrected population, stratified across seven Π tiers ([0–100), [100–300), [300–800), [800–2k), [2k–5k), [5k–9k), [9k, ∞)). For each design, the **peak achievable SR** under optimal tuning is measured separately for PID and ADRC:
- *PID*: 18×7 joint Kp×Kd grid (126 combinations, 3 search seeds) → best gains → evaluate at 15 fresh seeds (7001–7015).
- *ADRC*: sweep ωc ∈ {1.5, 2, 3, 5, 7, 10, 15}, ω₀ = 5·ωc, b₀ = keff; pick best ωc → evaluate 15 seeds.

**Result — performance by Π tier:**

| Π tier | n | PID mean SR | PID < 0.80 | ADRC mean SR | ADRC < 0.80 | ADRC − PID |
|---|---|---|---|---|---|---|
| < 100 | 10 | 1.000 | 0% | 1.000 | 0% | 0.000 |
| 100–300 | 10 | 1.000 | 0% | 1.000 | 0% | 0.000 |
| 300–800 | 10 | 1.000 | 0% | 1.000 | 0% | 0.000 |
| 800–2k | 10 | 1.000 | 0% | 1.000 | 0% | 0.000 |
| 2k–5k | 10 | 0.940 | 0% | 1.000 | 0% | +0.060 |
| 5k–9k | 10 | 0.867 | **10%** | 0.973 | 0% | +0.107 |
| > 9k | 3 | 0.622 | **67%** | 0.911 | 33% | +0.289 |

**Frontier crossings:**
- First PID SR < 0.90: Π = 2,616 — ADRC at Π = 7,795 → **ADRC extends 3.0× in Π**
- First PID SR < 0.80: Π = 5,214 (R2106, FRAGILE, td = 145, lat = 6; PID SR = 0.467, ADRC SR = 0.933) → **ADRC extends 2.4× in Π**
- First ADRC SR < 0.80: Π = 12,648 (R2072, FRAGILE, td = 351, lat = 6; PID SR = 0.333, ADRC SR = 0.733) — this is the highest-Π design in the entire corrected population.

Spearman ρ(Π, peak_pid_sr) = **−0.668** (p = 2.2×10⁻⁹); ρ(Π, peak_adrc_sr) = **−0.325** (p = 9.4×10⁻³). The twofold difference in correlation strength is the quantitative signature of ESO decoupling: Π predicts PID failure twice as strongly as ADRC failure. ρ(Π, sr_delta) = **+0.558** (p = 2.0×10⁻⁶): the ADRC advantage grows monotonically with Π.

**Caveat — and its resolution.** R2072's near-miss (SR = 0.733) was already known to be fixable by raising ω₀/ωc. The deeper concern: Π = 12,648 is also the *population maximum*, so the original experiment could not distinguish "ADRC's frontier is at 12,648" from "ADRC's frontier is somewhere beyond 12,648 but we ran out of designs." The frontier was truncated, not measured.

**Frontier extension beyond Π = 12,648 (`tools/adrc_frontier_extension.py`, n = 44).** A follow-up experiment drew 44 designs from the stress-test population (latency 1–12 steps), all with Π > 9,000, up to Π = 49,893 — 3.9× beyond the original maximum. ADRC was swept over an extended grid: ωc ∈ {0.5, 1, 1.5, 2, 3, 5} × ω₀/ωc ∈ {5, 8, 12, 20} (24 settings per design); PID over the same 18×7 joint grid; 15 eval seeds (9001–9015).

| Π tier | n | PID < 0.80 | ADRC std (ω₀/ωc = 5) | ADRC extended (best ratio) |
|---|---|---|---|---|
| 9k–12.6k | 27 | 85% | 4% | **0%** |
| 12.6k–15k | 6 | 83% | 0% | **0%** |
| 15k–20k | 5 | 80% | 20% | **0%** |
| 20k–30k | 3 | 100% | 33% | **0%** |
| > 30k | 3 | 100% | **100%** | **0%** |

Zero ADRC failures up to Π = 49,893 with an appropriately tuned ω₀/ωc. Most extreme design (S1430, Π = 49,893, td = 346 rad/s², latency = 12 steps): PID SR = 0.000, ADRC standard SR = 0.467, ADRC extended (ωc = 2, ω₀/ωc = 20) SR = **0.933**. 42/44 designs required ω₀/ωc > 5; 29/44 required ω₀/ωc = 20 (the maximum tested).

**Key finding: ADRC does not have a fixed Π ceiling.** It has a tuning parameter (ω₀/ωc) that must scale with Π. Standard ω₀/ωc = 5 fails above Π ≈ 20,000; ω₀/ωc = 20 succeeds up to at least Π = 49,893. Whether ADRC eventually fails at Π > 50,000 is not yet determined — but within the full stress-test design space (latency ≤ 12, realistic hardware), no ADRC failure was found.

**Interpretation.** Updated design rules across the full Π = k_eff × τ² range:

| Π = k_eff × τ² | Architecture | ω₀/ωc setting | Notes |
|---|---|---|---|
| < 300 | PID (optimal Kp) | N/A | SR = 1.000 for all tested designs |
| 300–900 | ADRC preferred | 5 (standard) | PID SR starts degrading at Π ≈ 320 |
| 900–1,600 | ADRC | 8–20 | ADRC-standard (ω₀/ωc = 5) may fail |
| 1,600–5,600 | ADRC | 20 | ADRC extended achieves SR = 1.000 |
| > 5,600 | Unknown | Not yet tested | — |

⚠️ **Pi correction note:** prior versions of this table cited thresholds in units of Π_td = θ̈_max × τ² (where θ̈_max = k_eff × u_max includes max_gimbal). The correct mechanistic parameter is Π = k_eff × τ², which is smaller by u_max = max_gimbal × 12/15 ≈ 1.6–12 depending on hardware. The qualitative design rule (low Π = PID safe, high Π = ADRC needed) is unchanged; only the threshold numbers change.

**Practical design rule:** Compute Π = k_eff × latency_steps² from hardware specs. If Π > 300 → consider ADRC. If Π > 900 → ADRC strongly recommended with ω₀/ωc ≥ 8. k_eff = T_avg × (π/180 × 15/12) × L_nozzle / Iyy [rad/s²/CU], independent of max_gimbal.

Data: `experiments/results/performance_frontier_py.csv` (n = 63), `experiments/results/adrc_frontier_extension_py.csv` (n = 44, extended).

### 4.0.4 Why Π = θ̈_max × τ²: a first-principles derivation

The two-variable regression (Section 4.0) finds log(θ̈_max) and log(latency) coefficients in a 2:1 ratio. Grid search confirms Π = θ̈_max × latency² collapses both to one parameter with equal predictive power (CV R² = 0.353 ≈ 0.352 two-variable). Why τ², not τ or τ³? The answer follows from the two gain-boundary mechanisms already established in Section 4.6 — one exact, one empirical:

**Ceiling (DIPDT phase-margin, exact for this plant class):**
> Kp_ceiling ≈ 0.9/τ   (keff-independent when keff × τ << 1)

The ceiling falls as **τ⁻¹** and is keff-independent. Empirically confirmed on the window_ratio v2 dataset (n = 116): ceiling exponent on latency = **−1.036** (theory: −1.0), on keff = **+0.059** (theory: 0.0). Both within 0.06 of theoretical prediction.

**Floor (bang-bang blind-spot, from limit-cycle physics):**
When Kp is below the floor, the design is in a bang-bang limit cycle. The actuator alternates at full deflection (θ̈_max) for approximately one latency window τ before new state information arrives. Angular impulse per half-cycle:
> Δω_bang ≈ θ̈_max × τ   [rad/s]

The floor must overcome this per-cycle impulse:
> Kp_floor ∝ keff × τ

Empirically: floor exponents keff = **+1.251**, latency = **+0.844** (theory: both +1.0). Deviations 0.25 and 0.16 — the floor rises slightly faster than linear in keff, likely because high-keff designs also have stronger disturbance response, but the direction and order are correct.

**Window = ceiling / floor:**
> Window ≈ τ⁻¹ / (keff × τ) = **1/(keff × τ²) ∝ 1/Π**

The τ² is the **double squeeze**: ceiling drops τ⁻¹ while floor simultaneously rises τ⁺¹. Each doubling of latency compresses the window by 2^1.88 ≈ **3.7×** on average — not 2^1.0 (ceiling-only) and not 2^2.0 (pure theory), because the floor exponent is 0.844, not 1.0. A latency extension to 7–12 steps (`tools/window_ratio_lat_extension.py`, n = 100 additional designs) reveals this figure is a keff-tier-dependent average: for low-authority designs (keff ≈ 5 rad/s²/CU), the exponent is −0.85 ≈ 1.8× per doubling — bang-bang oscillation is mild so the floor barely rises with latency, and only the ceiling compresses. For moderate-authority designs (keff ≈ 14), the exponent is −2.23 ≈ 4.7× — both mechanisms active, closest to the theoretical −2.0. For high-authority designs (keff ≈ 31), the exponent reaches −3.19 ≈ 9.1× — violent bang-bang amplifies the floor-latency coupling beyond the simple linear model. The pooled τ exponent appears to shift from −1.88 (lat 1–6, n = 82) to −1.4 (lat 1–12, n = 148), but this is a selection artifact: high-authority designs exit the feasible population at extreme latency (no valid window exists at lat ≥ 10 for keff ≳ 20), leaving the pooled dataset biased toward low-keff designs with weaker τ dependence. **Practical implication: the 3.7× figure is conservative for designs already in the narrow-window regime (keff ≳ 10); for those designs, MCU speed trades 4–9× per latency doubling.**

**Validation (`tools/pi_theory_validation.py`).** Regressing log(window_ratio) on log(Π) in the non-censored v2 dataset (n = 116): slope = **−1.029** (theory exactly −1.000, deviation 0.029). R²(log Π → log window) = 0.650 vs. two-variable R² = 0.659 — collapsing to Π loses essentially nothing. All seven exponent predictions (ceiling: keff = 0, lat = −1; floor: keff = +1, lat = +1; window: keff = −1, lat = −2) match measured values; none deviates by more than 0.25.

**Physical meaning of Π.** The correct parameter is **Π = k_eff × τ²** [rad/CU]. For a control command of u CU, the system accumulates Π × u radians of rotation during the latency window τ before any corrective feedback arrives. This is the "blind-spot displacement per CU": a design with k_eff = 10 rad/s²/CU and latency = 6 steps (τ = 0.03 s) has Π = 10 × 0.03² = 0.009 rad/CU; at a typical wind-rejection command of u ≈ 5 CU, that is 0.009 × 5 = 0.045 rad = 2.6° of uncompensated blind-spot error per correction cycle. The floor rises and the ceiling falls precisely because both mechanisms trace back to τ: it is the interval during which the system is blind. Π is not empirical — it is the kinematic consequence of two independently derived mechanisms, each measured and confirmed. (An earlier draft defined Π/2 = ½ × θ̈_max × τ² [rad], the angular displacement at *maximum* TVC authority; this is mechanistically incorrect because max-gimbal does not appear in either the floor or ceiling formula.)

**Interaction model: a principled unified equation without arbitrary tiers** (`tools/window_ratio_interaction.py`). The keff-tier dependence of the latency exponent is naturally captured by adding a single interaction term:

> log(window) = C + a·log(k_eff) + b·log(k_eff)·log(τ)

This model (n = 82, 5-fold CV) achieves **CV R² = 0.671** vs. 0.602 for the baseline (+0.069). The fitted coefficients are a ≈ −0.20 (keff alone at lat=1, near-zero), b ≈ −0.83 (interaction, keff-scaling of the latency penalty). The equivalent formula is:

> **window ≈ K / k_eff^(0.20 + 0.83·log(τ))**

where the keff exponent **grows with latency** rather than being fixed:

| τ (latency steps) | Effective keff exponent | Per-doubling-of-τ compression |
|---|---|---|
| 1 (5 ms) | −0.20 | — |
| 2 (10 ms) | −0.77 | — |
| 3 (15 ms) | −1.11 | ≈ 2.2× |
| 6 (30 ms) | −1.69 | ≈ 3.2× (keff=5) to ≈ 9× (keff=30) |

The model reproduces the measured tier exponents with no arbitrary cutpoints: effective lat exponent = b·log(keff) gives −1.40 at keff=5 (measured −0.85; model underpredicts for low-keff designs where the floor saturates near 1), −2.20 at keff=14 (measured −2.23), −2.84 at keff=30 (measured −3.19). The mid-to-high keff range — where gain-window risk actually occurs — is well captured. **Builder implication: upgrading from lat=6 to lat=2 (a 3× latency reduction) widens the window by a factor of keff^0.91 ≈ keff; for a design with keff=15, that is a 15× window improvement, not the 3.7× the simple Π formula would imply.**

**Exhaustive residual check: the remaining variance is stochastic, not a missing predictor.** After the interaction model, every available design parameter was tested as an additional predictor (n = 82 non-censored, 5-fold CV; `tools/window_ratio_full_regression.py`). No single factor adds more than +0.013 CV R²; the full 9-predictor model scores *lower* than the baseline (CV R² 0.536 vs 0.602, from overfitting). Within-cell variance — the variance among designs sharing the same keff and latency tier, which estimates irreducible stochastic noise — is 1.32, against a total unexplained residual of 1.40. These are essentially equal, confirming the unexplained ~33% variance is predominantly seed-to-seed stochastic variation and per-design unmodeled parameters (backlash, deadband magnitude, Kp-sweep discretization). **The keff + latency interaction is complete for the available parametric information.**

One corollary for builders: because k_eff = T_avg × (π/180 × 15/12) × L_nozzle / Iyy and the factor (π/180 × 15/12) = 0.02182 is a universal constant, **max_gimbal_deg does not appear in k_eff and does not need to be measured precisely** for gain-window prediction. Adding log(max_gimbal) to the baseline model gives delta CV = −0.003 (confirming it adds noise rather than signal). A builder can assess risk from thrust rating, motor scale factor, nozzle moment arm, and moment of inertia alone. The gimbal angle determines the *commanded* authority (θ̈_max = keff × u_max = keff × δ_max × 12/15), which sets the absolute torque output, but the *sensitivity of the gain window to perturbations* is governed by keff alone.

### 4.1 θ̈_max as a gain-sensitivity ranking metric (secondary result — audit trail and binary decision rule)

Population statistics, original 3-seed labels (n = 2,400):

| Regime | n | Mean θ̈_max (rad/s²) | Median | Range |
|--------|---|---------------------|--------|-------|
| EASY | 2,347 | 28.6 | 17.2 | — |
| FRAGILE | 45 | 124.5 | 104.4 | [36.1, 330.6] |

FRAGILE/EASY mean ratio = 4.4× (expanded Iyy range drives large separation). Cohen d = 1.74 (t = 16.59, p = 1.28 × 10⁻⁵⁸). Cross-validated AUC = 0.944 [0.927, 0.959] (10-fold CV: 0.944 ± 0.018).

**Final, twice-corrected population (n = 2,400, Section 4.6):** after the 15-seed reclassification and the subsequent finer-gain-search correction, the FRAGILE population shifts to n = 36, with mean θ̈_max = 168.2 rad/s² — *higher* than the original 124.5, because the correction disproportionately removed low-severity false-FRAGILE designs and added high-severity designs the coarser original search had missed. **Cohen's d = 3.71 (t = 11.2, p = 3.6×10⁻¹³), and AUC = 0.975.** This is the headline result of the paper; the table above is retained to show the audit trail, not as the current estimate.

Youden-J threshold = 54.8 rad/s² (TPR = 0.96, FPR = 0.12). At threshold = 50: TP = 43, FP = 311, FN = 2, TN = 2,041. Precision = 12.1% at a 1.9% base rate — expected for any rare-event screen. **Negative predictive value: 99.9%** — a design below threshold is almost certainly safe to tune in a disturbance-free simulator. The formula is a low-cost triage rule for a rare failure mode, not a high-precision diagnosis.

**Equivalent formulations** (all CV-AUC equivalent):
- authority_ratio = max_gimbal_deg × motor_scale / Iyy
- keff_full = T_avg × CU_TO_RAD × L_nozzle / Iyy (per-unit sensitivity, max_gimbal-independent)
- θ̈_max = keff_full × u_max

The three formulations are equivalent predictors because the same physical property (angular authority per unit inertia) is captured regardless of whether max_gimbal appears in numerator or is absorbed into the CU-to-rad conversion.

### 4.2 Variable screening: H1–H5

**Environmental variables (H1–H4): rejected** (confirmed on n = 2,400). The exhaustive search covered: log(θ̈_max), log(keff_full), log(wind_strength), log(servo_slew), wind × keff, wind × θ̈, θ̈/slew, keff/slew, static_margin, log(authority_ratio).

| Variable | r(var, gain sensitivity) | ∆window-ranking AUC over θ̈_max |
|---------|-----------------|-----------------|
| wind_strength | −0.008 | negative (hurts) |
| servo_slew_deg_s | +0.048 | negative (hurts) |
| static_margin | −0.001 | negative |
| Cm_alpha | −0.030 | negative |

No environmental variable improves gain-window prediction by ≥ 0.03. r(wind, gain sensitivity) = −0.008 — essentially zero. Gain sensitivity is a hardware property, not set by operational conditions.

**H5 — Control loop latency (hardware variable): confirmed.** Latency was already varied in the design space as a hardware parameter: latency_steps ∈ [1, 6] = [5–30 ms] at 200 Hz (independent LHS draw). Unlike H1–H4 (conditions that vary between flights), latency represents the microcontroller + sensor configuration — a fixed hardware choice.

| Predictor | AUC (bootstrap, n=2400) | ∆AUC |
|-----------|------------------------|------|
| θ̈_max alone | 0.944 [0.927, 0.959] | — |
| log(θ̈_max × latency_steps) | **0.972 [0.962, 0.980]** | **+0.028** |

(10-fold CV: 0.944 ± 0.018 alone; 0.972 ± 0.011 combined.)

The delta of +0.028 falls below the ≥ 0.03 screening threshold because the base AUC of 0.944 is already very high — the mechanism is still real. Both false-negative narrow-window designs in the n = 2,400 study have latency_steps = 6, consistent with latency independently compressing the gain ceiling. On the earlier n = 16 narrow-window population (lower base AUC = 0.855), latency gave ∆AUC = +0.072 — the mechanism is the same, its marginal contribution shrinks as the primary predictor improves.

**Key result:** log(θ̈_max × latency_steps) as a **single** variable gives AUC = 0.972. The product has a physically meaningful interpretation — θ̈_max · τ_latency [rad/s² × s = rad/s] is the maximum uncorrected angular velocity the rocket can accumulate in one control-loop delay window. At 5 Hz attitude dynamics, a 30 ms latency creates 54° of additional phase lag vs. 9° for 5 ms — compressing the gain ceiling independently of mechanical authority.

**Physical mechanism:** At 5 Hz (typical TVC attitude dynamics), a 30 ms latency creates 54° of additional phase lag versus 9° for 5 ms. This compresses the gain ceiling independently of θ̈_max — a rocket with moderate mechanical authority but high latency has a ceiling pushed down by two independent paths. The 4 false-negative designs in Section 4.5 all have latency_steps = 6, consistent with this mechanism.

**Design space note:** latency_steps ∈ [1, 6] covers fast MCUs (5 ms, e.g. Teensy) through typical hobby setups (15–20 ms, Arduino + SPI IMU) to slow configurations (~30 ms). Behavior beyond 30 ms (50 Hz control rate or serial-communication bottlenecks) was not tested and represents an open validation point.

**Visualization: the "Authority Trap" zone is not cleanly separated.** In the 2D space (θ̈_max, latency_steps), narrow-window designs (n = 36, final population) concentrate upper-right (high authority, high latency), but many wide-window designs also populate that zone — precision at any threshold is ~10%, an arithmetic consequence of the 1.5% base rate, not a weakness of the predictor. No decision boundary is drawn because AUC = 0.975 is a ranking metric, not a threshold accuracy claim. The scatter and product distribution are in `outputs/sts_gold_5_authority_trap.html` and `sts_gold_6_product_distribution.html`, **now regenerated on the final n = 36 FRAGILE population** (2026-06-16; the earlier statement "pending, built on n = 16 old population" no longer applies).

### 4.3 Dimensionless normalization does not improve θ̈_max

Buckingham-Pi analysis of {T, δ, L, Iyy} yields one nontrivial dimensionless group: Π = θ̈_max / ω² for a characteristic frequency ω. All candidate frequencies fail to correlate with gain-window width. Gravity-based normalization tested directly:

| Predictor | AUC | Notes |
|---------|-----|-------|
| θ̈_max (includes Iyy) | 0.944 | Dimensional, optimal |
| T × sin(δ) / (m × g) (no Iyy) | 0.730 | Gravity-normalized; −0.124 |
| max_gimbal_deg alone | 0.668 | No Iyy or motor_scale; −0.186 |

**Iyy is irreplaceable.** It is independently sampled in the LHS (not derived from mass × L²/12), so rockets with identical thrust and gimbal but different inertias have genuinely different gain windows. In the wind environment studied here, designs above approximately 55 rad/s² (Youden-J, n = 2,400) were much more likely to be gain-sensitive; the ranking (higher θ̈_max is worse) is robust, but the specific threshold depends on the wind distribution and success criterion.

### 4.3.1 Raw features baseline: formula ties best trained model; 2-variable combination wins

**Does θ̈_max merely proxy raw features?** A valid concern is whether AUC = 0.975 merely reflects that Iyy, motor_scale, and max_gimbal are individually correlated with gain sensitivity — making the formula a convenient but non-essential recombination. To test this, logistic regression (LR), random forest (RF), and gradient boosting (GBT) were trained directly on the raw design parameters and compared against θ̈_max (which requires no training and no labeled data). All results below use the final twice-corrected population (n = 36 narrow-window designs, `final_label` column from `exp1_final_population_py.csv`) — the version explicitly flagged as pending in earlier drafts of this paper.

| Approach | CV AUC (10-fold) | Training data required? |
|---|---|---|
| θ̈_max alone — no training | **0.975** (full AUC, no CV needed) | No |
| log(θ̈_max × latency) — no training | **0.988** (full AUC) / **0.991** (LR CV) | No |
| LR on 4 mechanical params (mass, Iyy, motor, gimbal) | 0.970 ± 0.011 | Yes |
| RF on 4 mechanical params | 0.944 ± 0.050 | Yes |
| GBT on 4 mechanical params | 0.926 ± 0.038 | Yes |
| LR on all 10 design parameters | 0.984 ± 0.013 | Yes |
| RF on all 10 design parameters | 0.976 ± 0.021 | Yes |
| GBT on all 10 design parameters | 0.961 ± 0.019 | Yes |

**θ̈_max alone (no training) ties RF on 10 features** (CV = 0.976) and trails LR on 10 features by a small margin (0.975 vs. 0.984 CV). The **2-variable no-training combination** log(θ̈_max × latency) achieves CV = 0.991, matching or exceeding every trained model — **a physics-informed pair of hardware scalars achieves near-parity with the best trained model (LR on all 10 features, CV = 0.984) with no training data required.** That is the circularity-refuting result: a formula derivable from Newton's law identifies the specific 1/Iyy combination those models must rediscover from labeled examples.

RF feature importance: Iyy dominates (0.37 — 3× the next variable, motor_scale at 0.13). Drop-one: removing Iyy collapses CV AUC from 0.976 to 0.630 (−0.346); no other variable costs more than −0.071. (Original 3-seed population gave lower numbers — θ̈_max AUC 0.944, RF CV 0.932 — because that population's noise-driven false-positives diluted the physical signal.)

### 4.4 Mechanistic chain: hardware specs to window collapse

**(Step 1) Newton's second law (no fitting):**
θ̈_max = T · sin(δ_max) · L_nozzle / Iyy

**(Step 2) Limit cycle amplitude at probe gain Kp = 2 (empirical, n = 41):**
At sub-optimal Kp (< floor), high-authority designs enter a limit cycle: the servo saturates at its slew limit while trying to reject wind, producing sustained bang-bang oscillation whose amplitude encodes the ceiling the design can tolerate.
A ≈ 1.63° × θ̈_max^0.40  (ρ = 0.62, n = 41; relay study rerun on 16 narrow-window + 25 wide-window designs spanning the full θ̈_max range). Narrow-window mean oscillation amplitude = 14.9° vs. wide-window mean = 6.5° at Kp = 2 (2.3× separation).

**(Step 3) Ultimate gain from oscillation amplitude (Åström–Hägglund 1984, exact):**
K_u = 4 · u_max / (π · A_rad)
Verified: r(K_u_measured, K_u_formula) = 1.000. Larger oscillation → lower estimated K_u → lower gain ceiling.

**(Step 4) Wide-window vs. narrow-window K_u separation — definitive, re-derived on the final population (2026-06-15, n = 36 narrow-window + 36 td-stratified wide-window designs, literal relay probe with zero-crossing amplitude/period extraction, not the cheaper RMS-approximation used in an earlier pass):**

| Class | Median K_u | Mean ± SD |
|-------|-----------|-----------|
| Wide-window | 91.2 | 118.0 ± 96.5 |
| Narrow-window | 29.2 | 33.9 ± 17.4 |

Mann-Whitney p = 4.17×10⁻⁷. Separation ratio 3.12× (median). This is the third re-derivation of this statistic on successively more rigorous populations and labels (n = 41 old labels: 2.8×, p = 0.0072 → n = 45 RMS-approximation: 2.1×, p = 2.9×10⁻⁵ → n = 36 true relay probe, final population: 3.12×, p = 4.17×10⁻⁷) — each pass *tightened* the separation rather than weakening it. **The ceiling has closed to (and below) the wind-rejection floor for the median narrow-window design.**

*Group-level vs. individual-level:* r(log θ̈_max, log K_u) = +0.004 at the individual level — no correlation. The ceiling-compression story holds at the **group level**: high-θ̈_max (narrow-window) designs have K_u ≈ 39 vs. wide-window K_u ≈ 108. Within the narrow-window group, keff_full is the better individual predictor (r = −0.407, p = 0.008). This is expected: K_u = 4 · u_max / (π · A_rad) depends on both A_deg (correlated with θ̈_max, ρ = 0.62) and u_max (independently sampled), so u_max variation dilutes the individual theta_ddot → K_u chain. The group comparison removes this confound.

*Note on R0115 (narrow-window, K_u = 128):* This is a high-floor design (best_Kp = 253) where Kp = 2 is far below the gain floor and the probe oscillation is wind-driven rather than bang-bang. The relay K_u overestimates the true gain ceiling for high-floor narrow-window designs. Excluding R0115: narrow-window median = 38, range = [24, 80], Mann-Whitney p = 0.0037. R0336 (narrow-window, floor-limited: fails under-robustness test, passes ceiling test) is addressed separately in Section 4.5.

**Direct ceiling-compression evidence (n = 2,400 survey):** 39 of 45 originally-classified narrow-window designs (87%) are ceiling-limited (over-robustness fails, under-robustness passes); 3 are floor-limited; 3 fail both tests. Window collapse is predominantly a ceiling problem: the gain ceiling has been compressed close enough to the wind-rejection floor that the 40% headroom margin is violated. This confirms the relay K_u mechanism independently of any probe study — the classification experiment itself is direct evidence of ceiling compression.

#### 4.4.1 Dual-regime bang-bang: the floor mechanism is different for low- vs. high-authority designs

The floor derivation above — "higher Kp is needed to damp bang-bang amplitude below the success threshold" — implicitly assumes the control loop is already in bang-bang mode during normal operation. A Kp sweep with simultaneous slew-saturation measurement (`tools/bang_bang_transition.py`, n = 3 designs at latency = 3, θ̈_max = 22 / 80 / 208 rad/s²) reveals two physically distinct operating regimes:

**Low-authority designs (θ̈_max ≈ 22 rad/s²):** slew_sat_frac ≈ 0.06 throughout the entire gain window from floor to ceiling. The servo is not saturated during normal attitude hold; it operates in the linear proportional regime. Bang-bang onset is visible as a sharp SR drop at Kp_ceiling with a simultaneous slew_frac jump. The ceiling mechanism here is exactly the DIPDT phase-margin derivation: enough phase lag accumulates at Kp_ceiling to create instability. The floor is low (≈ 1–3) because Kp = 1 is sufficient to reject wind when the system is not in bang-bang.

**High-authority designs (θ̈_max ≈ 208 rad/s²):** slew_sat_frac ≈ 0.63 **at every Kp across the entire sweep**, including at values below the gain floor. Wind disturbances alone drive permanent servo saturation regardless of Kp choice — the high-authority rocket responds to wind with a bang-bang trajectory at every gain setting. The gain window is not "the range before bang-bang starts" — the system is already in bang-bang. The window is the range of Kp where the wind-driven bang-bang oscillation amplitude is small enough to pass the success criterion.

This two-regime picture resolves the floor mechanism: for high-authority designs, a higher Kp is needed because the bang-bang oscillation from wind is already present and must be actively damped. The floor rises with both keff and latency because (1) higher keff → more aggressive bang-bang response to wind → larger oscillation amplitude at any Kp and (2) higher latency → rocket accumulates angular error for τ seconds before correction → larger oscillation amplitude at any Kp. Both paths increase the bang-bang amplitude that Kp must damp, explaining why floor ≈ 0.06 × keff × latency rather than depending solely on keff.

Data: `experiments/results/bang_bang_transition_py.csv`, visualization: `outputs/bang_bang_transition.html`.

### 4.5 False negatives (n = 2, θ̈_max < 55 but narrow-window, definitive n = 2,400 run)

D800 (θ̈ = 36.1, latency = 6) and D1523 (θ̈ = 38.3, latency = 6). Both have maximum latency_steps = 6 (30 ms). At 5 Hz attitude dynamics, 30 ms creates 54° of additional phase lag — compressing the gain ceiling independently of mechanical authority. The θ̈_max threshold of 54.8 rad/s² misses them because their mechanical authority is moderate; latency is the dominant predictor of failure for these designs, consistent with the phase-lag mechanism.

The combined predictor log(θ̈_max × latency_steps) assigns both FNs elevated scores and closes most of the gap — AUC improves from 0.944 to 0.972 by including latency. These designs represent the **latency-dominated narrow-window zone**: moderate mechanical authority + maximum control loop latency → ceiling compressed to near the wind-rejection floor.

**Previous n = 1,200 FN population** (R0804, R0047, R0680, R0452; all latency = 6) was consistent with this pattern. In the n = 2,400 LHS resample these specific designs did not reappear, but two new designs with the same latency = 6 signature did — confirming the latency-driven window-collapse mechanism is real, not a sampling artifact. The relay study (Section 4.4) on the earlier sample confirmed K_u ≈ 38 for these latency-driven FNs, identical to the high-authority narrow-window group median — their ceiling is compressed by the latency path, not the authority path.

### 4.5.5 Stress test: realistic hobbyist hardware (latency extended to [1, 12])

The original study used latency_steps ∈ [1, 6] = [5–30 ms] — covering fast MCUs (Teensy, STM32) through mid-grade setups. A large fraction of hobby TVC builders use Arduino-class microcontrollers with I²C IMUs, which typically run at 50–100 Hz, corresponding to latency_steps ≈ 8–12 at a 200 Hz sim rate.

**Stress test protocol:** New n = 2,400 LHS (seed = 99), latency_steps ∈ [1, 12], physical consistency filter (Iyy ≥ mass × 0.010 — removes designs where mass distribution would be physically implausible). Same autotune, same 3-seed robustness, same 30-deg success criterion. Data: `experiments/results/exp1_stress_test_py.csv`.

**Results:**

| Regime | Original (lat 1–6) | Stress test (lat 1–12) | Change |
|--------|-------------------|------------------------|--------|
| EASY | 2,347 (97.8%) | 2,241 (93.4%) | −4.4 pp |
| FRAGILE | 45 (1.9%) | 135 (5.6%) | **+3×** |
| INFEASIBLE | 3 (0.1%) | 16 (0.7%) | **+5×** |
| Total at-risk | 53 (2.2%) | 159 (6.6%) | **+3×** |

**Narrow-window fraction by latency bin:**

| Latency | Time (200 Hz) | MCU type | % narrow-window | % uncontrollable |
|---|---|---|---|---|
| 1–2 | 5–10 ms | Fast (Teensy/STM32) | 0.0% | 0% |
| 3–4 | 15–20 ms | Typical hobby MCU | ~1% | 0% |
| 5–6 | 25–30 ms | Slow hobby MCU | 3.5% | <0.1% |
| 7–8 | 35–40 ms | Slow Arduino + SPI | 4.7–6.5% | <1% |
| 9–12 | 45–60 ms | Arduino + I²C bottleneck | **10–13%** | 1–3% |

**The MCU choice alone creates a 13× range in narrow-window risk** (0% at fast MCU vs. 13% at slow). The gain ceiling equation explains this directly: Kp_max ≈ 380 / latency_steps = 38 at latency = 10. Any design with Kp_floor > 38 (moderate authority in even mild wind) has a closed or near-zero gain window at that latency.

**AUC under extended latency:**

| Predictor | Original | Stress test | Interpretation |
|---|---|---|---|
| θ̈_max alone | 0.944 | 0.919 | Drops — latency-driven narrow-window designs dilute authority signal |
| log(θ̈_max × latency) | 0.972 | 0.951 | Combined predictor holds (ΔAUc now +0.032, above 0.03 threshold) |

The drop in θ̈_max-alone AUC under extended latency is itself informative: it confirms that the combined predictor log(θ̈_max × latency) is the correct formula, not the simpler θ̈_max alone, when MCU speed varies across its realistic range.

**Iyy filter:** Removing the 80 most physically implausible designs (Iyy < mass × 0.010, a lower bound far below the uniform-rod minimum) improves AUC from 0.943 to 0.953 — the formula is *more* accurate on the physically credible subset. 7 of the 45 originally-classified narrow-window designs were in the implausible region; removing them does not change the story.

**Wide-window designs under 20-deg criterion:** All 2,347 wide-window designs already achieve 100% band_20 rate at their nominal best_Kp — max_theta < 20° in every run. The wide-window population is not "barely passing 30°" — it comfortably holds 20°.

**Practical implication:** A hobbyist choosing between an Arduino (lat ≈ 10, narrow-window risk ~11%) and a Teensy (lat ≈ 2, risk 0%) faces a dramatically different Kp window before the rocket is even built. The gain window equation (Section 5.2) quantifies this as a concrete [Kp_floor, Kp_ceiling] range from specs alone.

### 4.5.6 Audit: 3-seed classification noise and its correction

With exactly 3 binary seeds, success rate can only be {0, 1/3, 2/3, 1}. The classification threshold (0.80) sits between 2/3 and 1.0, so the test cannot distinguish a truly robust design (true p ≈ 0.85–0.95) from a truly fragile one (true p ≈ 0.5–0.7) — both commonly land on 2/3 by chance. Confirmed: 43 of 45 original FRAGILE designs (95.6%) were "borderline" (one seed-flip away from a different label).

**Correction:** 249 at-risk designs re-evaluated with 15 fresh seeds (disjoint from original 3), gains frozen.

| Regime | Original (3-seed) | Corrected (15-seed) | Flip rate |
|---|---|---|---|
| EASY | 2,347 | 2,365 | 6.1% of high-θ̈ subset |
| FRAGILE | 45 | 30 | 60% (25→EASY, 2→INFEASIBLE) |
| MARGINAL | 5 | 2 | 100% (all→EASY) |
| INFEASIBLE | 3 | 3 (composition changed) | 66.7% |

MARGINAL dissolved entirely — a 3-seed artifact. AUC improved: 0.943→0.957, confirming the physical signal was real and the noise had diluted it.

**Lesson:** a binary criterion with n seeds cannot resolve probabilities finer than 1/n. Future work: use ≥7 seeds, or report continuous SR with a binomial CI. This correction is itself superseded by Section 4.5.7, which found the original gains were also suboptimal.

### 4.5.7 Second audit: the gain search was also underpowered

The original `autotune_continuous` probes Kd once at Kp = 40 (frozen), then searches Kp alone over 10 coarse log-spaced points (~2.16× step) — coarser than measured gain windows (1.5–3.3×). Proof case: R0475 was labeled INFEASIBLE under both the original and 15-seed correction (SR ≈ 0.40 at frozen gains Kp=155, Kd=1). A finer joint 18×7 grid found gains giving SR = 0.90 for the same hardware. It was never physically uncontrollable.

**Correction (`tools/exp1_final_correction.py`):** 241 at-risk designs — finer joint Kp×Kd search (126 combos), then 30 fresh seeds (disjoint), plus Wilson 95% CI to flag uncertain designs.

| Regime | 15-seed (frozen gains) | Final (finer search + 30 seeds) |
|---|---|---|
| EASY | 2,365 | 2,362 |
| FRAGILE | 30 | 36 |
| MARGINAL | 2 | 0 (fully dissolved) |
| INFEASIBLE | 3 | 2 |

**AUC 0.957→0.975; Cohen's d 1.74→3.71 (t=11.2, p=3.6×10⁻¹³).** Third consecutive correction where fixing a methodological flaw strengthened the finding. 15/30 first-pass FRAGILE designs flipped to EASY (lower-severity half, median td≈97); 21 new ones entered from the high-θ̈ EASY pool (median td≈149). FRAGILE mean θ̈_max rose 124.5→168.2 rad/s². Min FRAGILE td (58.8) still falls below many wide-window designs (top td values 411.8, 376.6, 334.8) — no clean separating value exists, confirming the continuum directly.

**Irreducible uncertainty:** 81/241 re-examined designs (33.6%) remain "uncertain" (Wilson CI straddles the threshold at n=30), concentrated on the ceiling/over-robustness test specifically (70/241 = 29%) and only above θ̈_max ≈ 55 rad/s². Below that value, the call is statistically solid.

**S2R consequence:** re-deriving Section 5.1 on the final population with re-optimized gains reveals the 2 surviving INFEASIBLE designs both show **100% false approval** from the disturbance-free simple model — see Section 5.1.

Data: `tools/exp1_final_correction.py`, `experiments/results/exp1_final_population_py.csv`.

### 4.6 Gain ceiling equation: K_u ≈ 2.1 × 0.9 / (latency × dt) — a secondary, exploratory result

**Status relative to the central claim.** Unlike the θ̈_max predictor (Section 4.1–4.5, AUC = 0.957, cross-validated, no fitted parameters), the equation derived in this section is partly theoretical and partly fitted: the linear stability derivation (4.6.1) is exact, but the empirical 2.1× bang-bang correction factor and the power-law fit linking it to simulation data (4.6.2) have R² ≈ 0.53 and were validated against held-out data only for the ranking direction, not the magnitude (Section 4.6.4 below reports a negative result for a related combined formula). This section is included because it offers a plausible mechanistic account of *why* the gain window narrows, not because it is claimed at the same evidentiary standard as the central predictor. A reader primarily interested in the paper's main contribution can skip to Section 5.

**Research question:** What determines the maximum stable Kp (gain ceiling), and can it be predicted quantitatively from hardware specs?

#### 4.6.1 Theoretical derivation

The rocket attitude dynamics are a double integrator: θ̈ = k_eff × u. Under a PD controller with delay τ = latency_steps × dt, setting phase margin = 0 gives the linear stability limit K_u_theory via the transcendental equation **x × arctan(x) = k_eff × Kd² × τ** (exact, where x = √(k_eff / K_u)); K_u_theory = k_eff / x². For the operating range in this study (k_eff × τ < 1.0), K_u_theory × τ ≈ 0.9, so:

**K_u_theory ≈ 0.9 / τ = 0.9 / (latency_steps × dt)**

The ceiling is approximately **independent of k_eff** — set purely by the delay. This follows from standard DIPDT analysis (Di Ruscio 2010, ISA Transactions 2017); the K_u × τ ≈ 0.9 approximation for k_eff × τ << 1 is not explicitly stated in prior DIPDT literature.

#### 4.6.2 Empirical verification (boundary experiment v2)

**Protocol:** 6 narrow-window + 6 matched wide-window designs at the same θ̈_max levels [36–210 rad/s²]. Wind overridden to 5 levels [0.05–0.42], latency to [1, 3, 6] steps, 15 Kp values × 7 seeds. 18,900 total simulations. Spot-checked 5 specific (design, Kp, condition) cases: all 5 agreed with boundary experiment predictions (SR predictions matched pass/fail correctly). (Initial v1 run used only EASY designs — flawed because EASY floors are ≈1, making collapse impossible by construction.)

**Empirical ceiling vs theory:**

| latency | K_u_theory | ceil_data (median) | correction factor |
|---------|------------|-------------------|-------------------|
| 1 step (5 ms) | 180 | ≥270 (search limit) | ≥1.5× |
| 3 steps (15 ms) | 60 | 90–190 | 1.5–3× |
| 6 steps (30 ms) | 30 | 40–130 (median ≈ 60) | **2.1× (median)** |

Empirical power-law fit (n = 20 uncensored conditions, R² = 0.53):

**K_u_sim = 2.6 × k_eff^(−0.20) × τ^(−1.10)**

The latency exponent (−1.10) is consistent with the linear theory prediction (−1). The k_eff exponent (−0.20) is small, confirming that latency, not plant gain, drives the ceiling. The 2.1× correction factor (linear → simulation) reflects the bang-bang / slew-saturated nonlinear regime: the system can exceed the PM = 0 linear stability limit while still achieving SR ≥ 0.80 in 7 seeds.

**Unified ceiling law:**

$$\text{Kp}_\max \approx \frac{1.9}{\tau} = \frac{1.9}{\text{latency\_steps} \times dt} = \frac{380}{\text{latency\_steps}} \quad \text{(at 200 Hz)}$$

| latency_steps | Kp_max (theory × 2.1×) | Observed range |
|---------------|------------------------|----------------|
| 1 (5 ms) | 380 | ≥270 ✓ |
| 3 (15 ms) | 127 | 90–190 ✓ |
| 6 (30 ms) | 63 | 40–90 ✓ |

**A note on the constants.** The 1.9 is the DIPDT approximation at this study's k_eff range and Kd=1; the 2.1× is an empirically fit correction for this simulator's bang-bang/slew-saturated model at 200 Hz. Both should be re-fit before reuse in other simulators or hardware. The defensible claim is the *functional form* (ceiling ≈ 1/latency_steps, keff-independent), not the specific constants.

#### 4.6.3 Gain floor and window collapse

The floor (minimum viable Kp for wind rejection) is harder to predict analytically. From the relay study (n = 41): Kp_floor ≈ 0.35 × k_eff^0.70 (ρ = 0.58, log-log). In the boundary experiment, measured floors range from 1–8 (wide-window designs) to 5–27 (narrow-window designs) at latency = 6, with weak wind dependence (<5 Kp units).

**Window collapse** occurs when Kp_floor approaches Kp_max. At latency = 6 (30 ms), Kp_max ≈ 60. Any design requiring Kp > 60 for wind rejection — i.e., with a strongly elevated floor — will have a very narrow or zero viable window. In the boundary experiment, narrow-window designs at td = 36–67 rad/s² show floor = 5–27 vs. wide-window floor = 1–8 at the same θ̈_max level — confirmed mechanical differentiation in this range.

**Window narrowing is distinguishable between matched groups only at θ̈_max < 70 rad/s²** (floor-driven). At θ̈_max > 100 rad/s², both groups show ratios of 1–15× at latency = 6 — the narrowing is universal for high-authority designs, making the binary classification environment-dependent in this range. (Note: Section 4.5.6 shows this specific high-θ̈ population was itself ~61% noise-corrupted under the 3-seed test, so this convergence claim is not yet confirmed on clean labels.)

**Combined ceiling × floor formula: validated against held-out data and rejected as a unit predictor.** The ceiling formula (§4.6.2) and floor formula (above) were each independently fit on separate datasets and never tested together as a single predictive unit until this audit. Multiplying them (predicted window = Kp_max / Kp_floor) and testing against the boundary-experiment dataset (180 conditions, none used to fit either piece) gives: ceiling alone log-log R² = 0.474 (moderate), floor alone R² = −0.003 (worse than predicting the mean — fails outright), and the combined formula's ability to rank FRAGILE vs. EASY by predicted window width is AUC = 0.500 (chance), versus AUC = 0.577 for the ground-truth measured window. The formula systematically overpredicts FRAGILE windows and underpredicts EASY windows, indicating an unmodeled regime-dependent floor-elevation effect. The individual ceiling and floor formulas retain their support as described above; the *combined* window-ratio formula should not be cited as predictive.

**Novel aspects vs prior literature:**
- The K_u_theory × τ ≈ 0.9 approximation (k_eff × τ << 1 regime) is not stated in DIPDT literature (Di Ruscio 2010; ISA Transactions 2017)
- The empirical 2.1× correction factor for slew-saturated bang-bang actuators is not in prior work
- The hobby-TVC application (authority/Iyy → gain window) has no prior literature at hobby scale

**Revised practical rule:** "Ensure latency ≤ 4 steps (20 ms). At 200 Hz this gives Kp_max ≈ 95, safely above the wind-rejection floor for all but the most authority-sensitive designs. If θ̈_max > 100 rad/s², explicit Kp sweep in a full-physics simulator is required regardless of regime label."

### 4.7 Aerodynamic instability is irrelevant (null result)

r(p_unstable, regime_code) ≈ 0 across n = 1,200. Stable designs outperform unstable at all tasks; gap grows with maneuver amplitude (stable = 86.8% vs. unstable = 68.8% at 10°, 18 pp gap that does not close at higher slew rates). The fighter-jet analogy does not apply to hobby attitude-hold TVC.

---

## 5. Consequences: Simulator Selection and Flight Detection

### 5.1 Why disturbance-free simulators cause false rejection

A disturbance-free simulator optimizes step-recovery speed from a small initial angle (theta0 = 10°, `autotune_continuous`): higher Kp recovers faster until discrete-time oscillation onset; lower Kp is sluggish. This is a sensible objective in still air. The problem is that step-recovery speed in still air contains no information about the two boundaries that govern real-world performance — the wind rejection floor (minimum Kp to damp gusty disturbances) and the limit-cycle ceiling (maximum Kp before slew-rate saturation under gusty conditions drives bang-bang oscillation). A gain that optimizes step recovery can be below the wind floor, above the ceiling, or within the window — the still-air landscape cannot distinguish these cases. For gain-sensitive rockets, the selected Kp misses the real-world gain window 56% of the time.

From n = 2,400 designs (theta0 = 10°, autotune_continuous):

| Regime | n | SR(simple→full) | SR(full→full) | Gap | False Rejection |
|--------|---|-----------------|---------------|-----|-----------------|
| EASY | 2,347 | 0.863 | 1.000 | 0.137 | 9.8% |
| MARGINAL | 5 | 0.467 | 0.667 | 0.200 | 60.0% |
| FRAGILE | 45 | 0.430 | 0.919 | 0.489 | **57.8%** |

**False approval rate = 0.0% everywhere — on the original (pre-correction) population.** This table is retained for the audit trail, but the false-approval claim it implies does **not** survive the final correction (Section 4.5.7).

Median Kp comparison reveals the mechanism: wide-window designs (simple=36.7 vs full=69.6, 1.9× gap — undertuning misses wind floor); wind-limited designs (simple=320.0 vs full=59.2 — disturbance-free loss function drives optimizer to search ceiling, wildly overtuned); narrow-window designs (simple=88.8 vs full=88.8 — medians coincide but distribution is bimodal: some designs undertuned below floor, others overtuned above ceiling, any mismatch causes failure in a narrow window).

**Re-derived on the final population (Section 4.5.7): the "never dangerous" claim is FALSE for the rarest class.**

| Design class | n | SR(simple→full) | SR(full→full, re-optimized) | Gap | False Approval | False Rejection |
|--------|---|------------------|------------------------------|-----|-----------------|------------------|
| Wide-window | 2,362 | 0.861 | 0.999 | 0.138 | 0.0% | 10.1% |
| Narrow-window | 36 | 0.361 | 0.760 | 0.399 | 0.0% | **63.9%** |
| Uncontrollable | 2 | 0.667 | 0.233 | −0.433 | **100.0%** | 0.0% |

This table merges the simple-model-tuned success rate (unchanged from the original run) with the *best-achievable* full-physics success rate from the finer joint search (Section 4.5.7) — the same re-optimization that fixed the gain-search confound for classification. With only 2 genuinely INFEASIBLE designs surviving correction (down from 3; one was a search artifact, see R0475 in Section 4.5.7), both show false approval: the disturbance-free simple model rates them SR ≈ 0.67 (GO, since with no wind any Kp looks acceptable) while the best achievable full-physics success rate — even searching Kp up to 320 — is only 0.17–0.30, genuinely below the 0.35 INFEASIBLE cutoff. n = 2 is too small to estimate a rate (true rate could be anywhere from 50–100%), but the *direction* reverses the original claim: for the rare genuinely-uncontrollable design, a disturbance-free simulator gives **zero warning**, not merely an overly conservative one. Wide-window and narrow-window false-rejection rates are essentially unchanged (10.1% vs. 9.8%; 63.9% vs. 57.8% — the narrow-window rate rose slightly because the corrected population skews toward higher θ̈_max / lower nominal SR). **This is now the strongest single argument in the paper for flight testing, or at minimum simulating with real disturbance physics, before committing to hardware** — it is the one failure mode a disturbance-free simulator cannot warn a builder about at all.

### 5.1.1 Which modules dominate evaluation difficulty at each authority level?

The S2R analysis above addresses the *tuning side*: wind's absence causes the simulator to select a wrong Kp. A complementary experiment (`tools/fidelity_cutoff_by_td.py`) asks the *evaluation side*: given a design already at its physics-optimal Kp, which individual fidelity modules add meaningful difficulty at each θ̈_max level? Method: 25 designs from the final population, 5 stratified per θ̈_max tier, evaluated at their stored best_Kp under (a) full physics and (b) each module ablated in turn; 10 fresh seeds per condition. Delta_SR = SR_ablated − SR_full; a positive delta means removing that module makes evaluation *easier* (that module was genuinely adding difficulty).

| θ̈_max tier | ∆SR: wind | ∆SR: slew | ∆SR: latency | ∆SR: noise | ∆SR: backlash | ∆SR: deadband |
|---|---|---|---|---|---|---|
| < 40 rad/s² | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| 40–70 rad/s² | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| 70–100 rad/s² | +0.02 | +0.02 | +0.02 | +0.02 | 0.00 | 0.00 |
| 100–150 rad/s² | +0.04 | +0.10 | +0.12 | +0.12 | +0.08 | −0.02 |
| > 150 rad/s² | 0.00 | +0.04 | +0.04 | +0.04 | +0.02 | +0.04 |

Two findings stand out. First, **below θ̈_max = 70 rad/s², no individual module adds measurable evaluation difficulty** — a correctly-tuned design achieves SR ≈ 1.0 regardless of which modules are active, because the gain window is wide enough that the optimal Kp is robust to any single physics perturbation. Second, in the 100–150 rad/s² danger zone, **sensor noise and latency are the dominant evaluation hardness factors** (removing either improves SR by 0.12); slew limits and backlash follow (+0.08–0.10). **Wind has the smallest individual evaluation impact (+0.04 maximum) at any authority level** — consistent with Section 5.1's finding that wind's role is in *selecting* which Kp is correct, not in making an already-correctly-tuned design fail. A builder who tunes with full physics but uses a model that strips out latency or sensor noise will find their high-authority designs struggling despite nominally correct gains; a builder who strips out wind (but has the right Kp from another source) suffers much less.

The near-zero delta for all modules below td = 70 provides an additional cross-check on the matched-config result in Section 4.0.1: at low authority, the gain window is wide enough that even full physics is not challenging at the optimal Kp, let alone a reduced-fidelity version.

Data: `experiments/results/fidelity_cutoff_by_td_py.csv` (25 designs × 7 conditions × 10 seeds).

### 5.2 A single flight confirms gain sensitivity

**Protocol:** deploy Kp = 2 in full-physics simulation; measure RMS attitude error. Results below use the final (twice-corrected) population: 36 narrow-window + 36 td-stratified wide-window designs (n = 72 total, 7 seeds each), re-run after Section 4.5.7's correction superseded the earlier n = 45 version.

| Metric | Value |
|--------|-------|
| AUC (7-seed RMS, n=36 narrow-window, final population) | **0.954** [0.907, 0.989] |
| AUC (1-seed RMS) | **0.921** |
| AUC (θ̈_max alone, same sample) | **0.962** |
| Recommended threshold (F1 = 0.89) | RMS > 7.6° (precision = 0.91, recall = 0.86) |
| Best-recall threshold | RMS > 6.0° (recall = 0.94, F1 = 0.88) |
| Narrow-window design mean RMS at Kp=2 | 13.3° ± 5.2° |
| Wide-window design mean RMS at Kp=2 | 3.8° ± 2.7° |
| Class separation ratio | 3.53× |

This is a confirmation, not a reversal: the result on the original n = 45 population (AUC = 0.943, threshold 6.0°, F1 = 0.88) was already close, and the final-population rerun *strengthened* it slightly (AUC 0.943 → 0.954; separation ratio 2.9× → 3.53×) because the corrected narrow-window population is more severe on average (mean θ̈_max 168.2 vs. 124.5 rad/s²). The 1-seed AUC improved more substantially (0.853 → 0.921) for the same reason — a more severe population is easier to detect from a single flight. The earlier 6.0° threshold and the new 7.6° threshold are both defensible (6.0° favors recall, 7.6° gives the best F1 on the final population); a builder optimizing for safety margin should prefer the lower, more sensitive 6.0° cutoff.

Flight detection (AUC = 0.954) and the spec formula (AUC = 0.962 on the same balanced sample; 0.975 on the full n = 2,400 population) are nearly equivalent predictors. This raises an obvious question: if a single low-gain test flight detects gain sensitivity just as well as the formula, why does the formula matter? **Because the formula is available before any hardware exists, requires no flight, no risk, and no launch — it is a design-time screen, not a post-hoc diagnostic.** The flight test confirms a design that has already been built; the formula tells a builder whether to build it that way in the first place. The two are complementary stages of the same workflow, not competing predictors. Wind confound check (original study): AUC by wind tercile = 0.924 / 1.000 / 0.986 — detection is not a wind artifact.

**Practical workflow — the builder equation:**

**Step 1.** Compute your predicted gain window from hardware specs alone:
```
keff      = T_avg × (π/180 × 15/12) × L_nozzle / Iyy    [rad/s² per code-unit]
Kp_ceil   ≈ 380 / latency_steps                           [gain ceiling; keff-independent]
Kp_floor  ≈ 0.06 × keff^1.06 × latency_steps^0.96        [gain floor; window ratio v2, CV R²=0.594]
window    = Kp_ceil / Kp_floor                             [how wide your safe Kp range is]
```
(`tools/gain_advisor.py` computes this automatically: `python tools/gain_advisor.py --thrust T --gimbal δ --iyy Iyy --latency steps`)

**Step 2.** Interpret the window:
- **window > 50×**: any reasonable tuning approach works; disturbance-free simulator is acceptable.
- **window 10–50×**: tune in a full-physics (windy) simulator; disturbance-free tuning misses the window ~40–64% of the time. Target Kp within [Kp_floor, Kp_ceiling].
- **window < 10×**: very narrow; ADRC (Section 6.1) or expert full-physics PID tuning required.
- **window < 1×** (Kp_floor ≥ Kp_ceil): no stable PID gain exists at this latency and authority combination.

**Step 3.** Fly once at Kp = 2; measure RMS attitude error.
- **RMS > 7.6°** (or > 6.0° for a more sensitive cutoff): narrow-window confirmed → re-tune within [Kp_floor, Kp_ceiling] computed above.
- **RMS < 6.0°**: wide-window design; standard tuning is sufficient.

*Note: window equation coefficients fit on simulation data; floor formula CV R²=0.594 means individual predictions vary. The flight-detection threshold (7.6°/6.0°) was calibrated in simulation and requires hardware validation before deployment.*

Data: `experiments/results/flight_sig_final_py.csv` (504 rows, supersedes `flight_sig_updated_py.csv`).

---

## 6. Cross-Architecture Design Rule and Builder Tool

This section contains the paper's second central, load-bearing contribution (Section 1.4): not just evidence that an alternative controller architecture helps, but a quantified comparison of *how* PID and ADRC ceilings differ, and a runnable tool that turns that comparison into an actual design recommendation.

### 6.0 A necessary caveat: ADRC and PID are not as architecturally distinct as "cross-architecture" suggests

A fresh literature check (2026-06-16, prompted by an internal full-assessment pass rather than an external reviewer) found a result that qualifies the framing of this entire section. Carlson (2025, arXiv:2501.11374) proves that **linear ADRC tuned with the standard "bandwidth method"** — exactly the ωc/ω₀/b0 parameterization used throughout this paper — **is mathematically equivalent to a two-degree-of-freedom PID controller with set-point weighting and a low-pass filter on the measurement.** For first- and second-order plants (this project's rocket attitude dynamics are modeled as a double integrator, i.e. second-order), the equivalence is essentially exact in the linear regime. This means ADRC-with-bandwidth-tuning is not, strictly speaking, a different control *paradigm* from PID — it is a different, filtered reparameterization of a related linear structure.

**This does not erase the empirical results in Sections 6.1–6.2**, but it requires restating what they show. The equivalence proof is a *linear* result. This paper's entire gain-sensitivity mechanism (Section 4.4) is explicitly a *nonlinear, saturated* phenomenon — actuator slew-rate saturation driving bang-bang oscillation. The two controllers' implementations diverge precisely in that nonlinear regime: ADRC's Extended State Observer continues estimating and subtracting the disturbance term even while the actuator is at its slew limit, whereas a plain high-Kp PID's proportional and derivative terms interact with saturation directly and produce the bang-bang chatter described in Section 4.4 and Appendix C. **The honest claim is therefore: two different disturbance-handling conventions for a closely related linear control structure produce measurably different gain/bandwidth ceilings specifically in the saturated operating regime that defines gain sensitivity in this study** — not "two unrelated architectures," and not "ADRC is a fundamentally novel paradigm" (a claim this paper does not make, but which "cross-architecture" could be read as implying). Section 6.2's novelty claim is revised accordingly below.

### 6.1 ADRC dissolution test: does an alternative architecture reduce the sensitivity?

Having identified which designs are gain-sensitive, a separate question is: does the narrow-window problem persist under a structurally different controller (ADRC), or does an alternative architecture substantially reduce it? (This experiment can show that *an* alternative architecture helps; it cannot, by itself, show that the sensitivity is "a PID-specific limitation" in general, since that would require testing every plausible alternative, not just ADRC — see the narrower conclusion stated below.) A step-tracking comparison on the original n = 1,200 population (Appendix C) suggested ADRC helps substantially, but that result used a stale, pre-correction FRAGILE population and an oracle-style, per-design-class ADRC tuning. The test below re-runs the comparison on the **final (Section 4.5.7), twice-corrected n = 36 FRAGILE population**, on the *exact* attitude-hold task that defines FRAGILE (not a step-tracking proxy), with a single **universal, untuned** ADRC setting compared against best-effort PID gains from the same finer joint search used to certify the population itself — the fairest comparison attempted in this project.

**Protocol:** all 36 final narrow-window designs plus a θ̈_max-stratified 36-design wide-window sample, theta_ref = 0, t_end = 3 s, full physics including wind, 20 fresh seeds. PID uses the best-effort gains from Section 4.5.7's finer search (not a strawman). ADRC uses one fixed setting for every design — ωc = 5, ω₀ = 25, b0 = k_eff computed directly from specs — never searched or tuned per design.

| | PID (best-effort) | ADRC (universal, untuned) |
|---|---|---|
| Narrow-window (n=36) SR | 0.776 | 0.962 |
| Narrow-window (n=36) RMS | 5.3° | 2.8° |
| Wide-window (n=36) SR | 0.996 | 1.000 |
| Wide-window (n=36) RMS | 1.2° | 0.7° |

**PID narrow-window vs. wide-window success-rate gap = 0.219. ADRC gap = 0.038 — 83% of the gap closed by one untuned controller setting.** 15 of the 17 narrow-window designs with PID SR < 0.80 reach ADRC SR ≥ 0.80 (88% conversion).

**Only 2 of 36 narrow-window designs still fail under ADRC** (one at θ̈ = 351 rad/s², latency = 6 steps, ADRC SR = 0.50; one at θ̈ = 312, latency = 5, ADRC SR = 0.75) — the two most extreme authority+latency combinations in the entire population. A follow-up bandwidth sweep on the worse case found that lowering ωc from 5 to 3 (a slower closed-loop bandwidth — standard practice for high-latency control loops) at ω₀ ∈ {35, 50, 70, 100} gives SR = 1.00 in every case tested. **This is not a new ADRC ceiling.** It is the *same* θ̈_max × latency mechanism already established for PID (Section 4.2's H5), now manifesting as a bandwidth-tuning requirement for ADRC rather than an outright, unconditional success.

**Conclusion (deliberately narrow).** A single untuned ADRC setting recovers 94% of the narrow-window vs. wide-window gap that best-effort PID could not close; the remaining 6% (2/36, extreme-latency) is explained by the same θ̈_max × latency predictor, now shown to generalize to a structurally different controller. **What this establishes:** at least one alternative architecture substantially reduces the sensitivity, and the same predictor generalizes to it. **What this does NOT establish:** that gain sensitivity is "a PID-specific limitation" — LQR, MPC, and gain-scheduled PID were not tested, and it is possible a PID variant closes the same gap.

This supersedes the original n = 1,200 step-tracking comparison below (Appendix C), which is retained because its mechanistic explanation (servo saturation during a step command) remains a useful illustration, but its specific population and "100% conversion" framing are stale.

Data: `tools/adrc_fragility_dissolution_test.py`, `experiments/results/adrc_dissolution_py.csv`.

### 6.2 Mechanism isolation: saturation is the sole cause of PID failure at high Π

Section 6.0 identifies that the Carlson (2025) PID–ADRC equivalence breaks down specifically in the saturated regime. Section 6.1 shows ADRC recovers 83% of the narrow-window failure gap. But *why* does ADRC succeed where PID fails? This experiment answers that question by isolating slew saturation as a causal factor using a 2×2 factorial design.

**Method (`tools/adrc_saturation_test.py`, n = 15 designs, Π = 27–12,648).** Design factors: {PID, ADRC (fixed)} × {saturation on, saturation off}. "Saturation on" = full physics with servo slew limits (as-specified per design, realistic hardware). "Saturation off" = `FidelityConfig(slew=False)` — ideal servo that deflects at any rate. PID gains: best Kp/Kd from 18×7 joint search (same as Section 4.5.7). ADRC: one fixed setting for all designs, ωc = 5, ω₀ = 25, b₀ = keff (same setting as Section 6.1 dissolution test). 10 EASY + 5 FRAGILE designs stratified by Π. Evaluation seeds 10001–10015 (disjoint from all prior experiments).

**Results: three clean facts:**

| Π | Label | PID+sat | ADRC+sat | PID−sat | ADRC−sat | slew_PID | slew_ADRC |
|---|---|---|---|---|---|---|---|
| 27–1,686 | EASY | 1.000 | 1.000 | 1.000 | 1.000 | 0.000–0.005 | 0.000 |
| 1,441 | FRAGILE | 1.000 | 1.000 | 1.000 | 1.000 | 0.086 | 0.000 |
| 2,411 | FRAGILE | 1.000 | 1.000 | 1.000 | 1.000 | **0.458** | 0.000 |
| 3,589 | FRAGILE | 1.000 | 1.000 | 1.000 | 1.000 | **0.340** | 0.000 |
| 5,214 | FRAGILE | **0.867** | 0.867 | **1.000** | 0.867 | **0.708** | 0.000 |
| 12,648 | FRAGILE | **0.600** | 0.533 | **1.000** | 0.533 | **0.705** | 0.000 |

*slew_PID = fraction of timesteps PID's commanded deflection hits the servo slew limit; slew_ADRC analogous.*

**Fact 1 — PID failure is entirely caused by saturation.** PID−sat = 1.000 for every single design in the test, including Π = 12,648 (PID normally achieves SR = 0.600 there). Removing the slew limit completely restores PID to full performance. Saturation is both necessary and sufficient for PID failure in this Π range.

**Fact 2 — ADRC never saturates.** slew_frac_adrc = 0.000 for all 15 designs. ADRC's ESO cancels the wind disturbance before the control law, so the commanded servo deflection remains small throughout each run. At Π = 2,411, PID is already hitting the slew limit 45.8% of timesteps while achieving SR = 1.000 — the narrow window is still open at this Π; the saturation becomes failure-causing above Π ≈ 5,000 when no valid gain can simultaneously reject wind and avoid limit cycling. ADRC never reaches that threshold regardless of Π because its effective gain (ωc²/b₀ ≈ 2–5 rad/s per CU) is far below PID's required Kp (40–80). Spearman ρ(Π, slew_frac_pid) = **+0.875** (p = 2.0×10⁻⁵): Π predicts how often PID saturates, closing the mechanistic loop between the predictor (Π), the mechanism (slew saturation rate), and the outcome (SR).

**Fact 3 — Removing saturation does not help ADRC.** ADRC−sat = ADRC+sat for all designs. ADRC's failure at Π = 5,214 and 12,648 is not saturation-driven; it is insufficient bandwidth at the fixed ωc = 5 setting (the same bandwidth limitation the frontier extension resolved by lowering ωc and raising ω₀/ωc, Section 4.0.3). This also confirms the ADRC never-saturates result is not trivially explained by ADRC being "already switched to the ideal-servo branch" — it genuinely never approaches saturation.

**Carlson reconciliation.** Without saturation, PID achieves SR = 1.000 and ADRC achieves SR = 1.000 (ADRC−sat column, all ones). This is exactly the Carlson (2025) equivalence: in the linear, unsaturated regime, bandwidth-tuned ADRC and PID are the same controller and achieve identical results. The departure arises entirely in the saturation-active conditions. Section 6.0's reframing — "two disturbance-handling conventions that diverge in the saturated regime" — is now empirically confirmed rather than just argued from the Carlson theorem.

**Important nuance on fixed ADRC settings.** At Π = 12,648, the fixed ωc = 5 ADRC (SR = 0.533) slightly *underperforms* optimal PID (SR = 0.600). This should not be read as "PID beats ADRC." It reflects the experimental design: PID gains were searched over a 18×7 grid, ADRC used a single fixed setting never optimized for this design. The performance frontier (Section 4.0.3), which swept ADRC across seven ωc values and picked the best, showed ADRC achieving SR = 0.733 at this same design (vs PID SR = 0.333 under a comparable seeds draw). The saturation test isolates the *mechanism* (does ADRC prevent saturation? yes) rather than comparing peak achievable performance.

**Conclusion.** The causal chain is now complete: high Π → PID requires high Kp for wind rejection → servo saturates (70% of timesteps at Π = 5,000–12,648) → bang-bang limit cycling → SR degrades. ADRC: ESO cancels wind → effective gain stays low → slew limit never reached → SR maintained. Both the predictor (Π) and the mechanism (slew_frac) are empirically linked with ρ = +0.875. This closes the "mechanism was argued, not measured" gap that Section 6.0 identified.

Data: `tools/adrc_saturation_test.py`, `experiments/results/adrc_saturation_test_py.csv`.

### 6.3 ADRC has its own bandwidth ceiling — and it depends on different physics than PID's

Section 4.6 (the gain ceiling equation) establishes that PID's gain ceiling, Kp_max ≈ 380/latency_steps, is dominated by control-loop latency almost to the exclusion of mechanical authority (latency exponent ≈ −1.10, plant-gain exponent only −0.20 in the underlying power-law fit). Section 6.1's bandwidth-sweep follow-up showed that ADRC's two residual failures were fixed by *lowering* its closed-loop bandwidth ωc for a high-latency design — suggesting ADRC has an analogous ceiling on ωc. This subsection quantifies that ceiling directly, asking whether it is governed by the same physics as PID's, or different physics.

**Method.** Two sequential sweeps using the same protocol: (1) `tools/adrc_bandwidth_ceiling.py` — 3 reference configurations at θ̈_max ≈ 20, 90, 200 rad/s², latency ∈ {1,2,3,4,6,8,10,12}, ω₀ = 5×ωc fixed, 10 seeds, full physics, seeds 9001–9010. (2) `tools/adrc_ceiling_extended.py` — 3 additional authority levels (θ̈_max ≈ 50, 130, 300 rad/s²), otherwise identical protocol, independent pool seed (888) and evaluation seeds (9101–9110). Combining both gives 6 reference authority levels × 8 latency levels = 46 valid cells (2 NaN at td ≈ 300, latency ∈ {10,12}).

**Result (extended dataset, n = 46 valid cells):**

| lat | ωc_max, θ̈≈20 | ωc_max, θ̈≈50 | ωc_max, θ̈≈90 | ωc_max, θ̈≈130 | ωc_max, θ̈≈200 | ωc_max, θ̈≈300 |
|---|---|---|---|---|---|---|
| 1 | 30 | 15 | 20 | 15 | 10 | 10 |
| 2 | 20 | 10 | 15 | 10 | 10 | 10 |
| 3 | 20 | 10 | 7 | 10 | 7 | 7 |
| 4 | 15 | 7 | 7 | 7 | 7 | 7 |
| 6 | 15 | 7 | 5 | 7 | 5 | 5 |
| 8 | 10 | 5 | 3 | 5 | 5 | 5 |
| 10 | 10 | 5 | 3 | 5 | 3 | — |
| 12 | 7 | 3 | 3 | 5 | 3 | — |

**Fitting a joint power law to the 46 valid cells: ωc_max ≈ 70 / (latency_steps^0.57 · θ̈_max^0.31), R² = 0.823.**

This is a significant revision from the original 3-reference-design estimate (n = 21 cells: ωc_max ≈ 648 / (latency^0.66 · θ̈_max^0.77), R² = 0.936). Three things changed with the extended sample:
- **Latency exponent:** −0.66 → −0.57 (modest change; latency remains the primary driver).
- **Authority exponent:** −0.77 → −0.31 (the original claim of "comparable authority dependence" is not supported by the larger sample).
- **R² drops from 0.936 to 0.823**, reflecting genuine design-to-design variance that was hidden when only 3 reference designs were used.

**The correct, updated characterization:** ADRC's bandwidth ceiling is *primarily* governed by latency (exponent −0.57), with a *secondary* authority effect (exponent −0.31). The authority exponent (−0.31) is 1.5× that of PID's plant-gain exponent (−0.20), suggesting real but moderate authority coupling absent from PID — but not "comparable in strength to latency" as the n = 21 result suggested. The qualitative structural difference from PID survives: for PID, authority barely matters (keff exponent −0.20); for ADRC, authority has a measurable secondary role (−0.31). But the original framing that "both exponents are ~−0.7" was an artifact of having only 3 reference authority levels, where between-design variation was confounded with the between-td effect.

**Resolved caveat — ω₀-ratio follow-up.** The two NaN cells (td ≈ 300, latency ∈ {10,12}) are confirmed solvable at ω₀/ωc = 10. Practical rule for high-authority, high-latency ADRC (latency ≥ 7, θ̈_max > 150 rad/s²): use ω₀/ωc ≥ 10, ωc ≤ 3. Data: `tools/adrc_ceiling_omega0_sweep.py`.

**What this result is and is not.** "ADRC also has a ceiling" is unsurprising; standard delay-margin theory predicts this. The contribution — more modest than the original claim — is that the same predictor (θ̈_max) produces two ceiling laws with *different* authority dependencies: PID's ceiling barely depends on authority (exponent −0.20), ADRC's ceiling has a measurable secondary authority effect (exponent −0.31). Both remain latency-dominated. To the author's knowledge, no prior work makes this comparison for any TVC hardware class. The n = 46 cell count is still limited — individual exponents carry uncertainty, and the power-law form is unvalidated — but the qualitative direction (ADRC shows more authority coupling than PID) is stable across both the original 21-cell and the extended 46-cell analyses.

Data: `tools/adrc_bandwidth_ceiling.py`, `experiments/results/adrc_bandwidth_ceiling_summary_py.csv`; extended sweep: `tools/adrc_ceiling_extended.py`, `experiments/results/adrc_ceiling_extended_summary_py.csv`; ω₀-ratio follow-up: `tools/adrc_ceiling_omega0_sweep.py`.

### 6.4 A runnable builder tool: `tools/gain_advisor.py`

Every result up to this point in the paper is a number or a formula a reader has to apply themselves. This subsection describes a small, runnable tool that does that arithmetic and produces an actual recommendation.

**Inputs:** average motor thrust (N), max gimbal deflection (deg), pitch moment of inertia Iyy (kg·m²), control-loop latency (steps at 200 Hz), nozzle moment arm (m, defaults to this study's fixed 0.25 m airframe).

**Outputs:**
- θ̈_max and keff_full, computed directly from specs (no fitting).
- A **continuous** predicted gain-margin score from the Section 4.0 regression (not a binary label).
- A 5-tier qualitative risk level (low / low-moderate / moderate / elevated / high), each tier's boundary stated explicitly and tied to where the continuous relationship in Section 4.0 actually changes shape, not to an arbitrary round number.
- A recommended **PID Kp range** [floor, ceiling] — the core builder output: Kp_ceiling ≈ 380/latency_steps and Kp_floor ≈ 0.06×keff^1.06×latency_steps^0.96 (window ratio regression v2, CV R² = 0.594, Section 4.6.3), plus the resulting window ratio. This tells a builder directly whether their Kp precision requirement is forgiving (window > 50×) or dangerously tight (window < 10×), with actual Kp numbers to target.
- A recommended **ADRC ωc ceiling**, using the Section 6.3 formula.
- A one-line caution about disturbance-free simulator tuning, referencing the false-rejection/false-approval results in Section 5.1.

Every formula used inside the tool carries an inline comment in the source citing the experiment that produced it and that experiment's sample size and validation strength (e.g., the continuous margin model's coefficients are flagged with their 5-fold CV R² = 0.288 ± 0.103 and MAE = 0.070, so a user can see exactly how much to trust the number, rather than presenting all outputs with equal apparent confidence). This is a deliberate departure from presenting a single polished formula: the tool is honest about which of its outputs rest on strong evidence (the PID ceiling formula, R² = 0.53 on n = 20 conditions plus a confirmed −1 latency exponent matching linear theory) and which rest on weaker or first-pass evidence (the PID floor formula, ρ = 0.58 only; the ADRC ceiling formula, n = 21 cells with a fixed ω₀ ratio).

**Example:** 14.4 N motor, 10° gimbal, Iyy = 0.015 kg·m², latency = 4 steps → θ̈_max = 41.9 rad/s², Kp ∈ [1.1, 95.0] (85× window), ADRC ωc ≤ 14.6. Changing only Iyy to 0.005 kg·m² → θ̈_max = 125.7, Kp window narrows to 40×, ADRC ωc ≤ 6.2 — 3× tighter Kp precision required from the lighter build alone.

This tool is the project's answer to the "genuinely helpful for builders" goal: instead of "compute θ̈_max and compare it to a threshold," a builder gets an actual recommended operating range for whichever controller they choose.

---

## 7. Discussion

### 7.1 The threshold is environment-specific; the ranking is robust

In this study's wind environment, designs above approximately 55 rad/s² (the final population's Youden-J threshold, Section 4.1) were much more likely to be gain-sensitive. A builder flying in stronger winds would have a lower effective threshold; a builder in calmer conditions would have a higher one. The specific number (55 rad/s²) should not be treated as a universal constant.

What is robust is the **ranking**: θ̈_max is the best single predictor available from hardware specs alone, no normalization improves it (Section 4.3, AUC drops 0.124 when Iyy is replaced by mass), and the mechanism (Section 4.4) explains why higher θ̈_max always means narrower gain window regardless of environment.

### 7.2 The narrow-window regime is rare but invisible without the formula

~2% of designs at moderate authority (θ̈_max 80–120) and ~40–46% at high authority (θ̈_max > 180) fall below the 0.80 reliability threshold — a randomly chosen hobby TVC design at typical Iyy values is usually in the forgiving zone, but a lightweight, high-authority build (small Iyy, aggressive gimbal, high motor) can easily enter the narrow-window regime. Without computing the gain window from hardware specs, there is no way to identify this ahead of time without flight testing. The 5-minute calculation from a motor data sheet and mass measurement gives a concrete Kp range [Kp_floor, Kp_ceiling] and quantifies how much tuning precision is required before any hardware is built.

### 7.3 Limitations

1. **The narrow-window regime is rare (n = 36 of 2,400 in the classified experiment, ~1.5%); what varies is the degree, not just membership.** The continuous regression (Section 4.0) shows that ~2% of low-authority designs (td < 40) and ~46% of extreme-authority designs (td > 300) fall below the 0.80 reliability threshold — the risk scales continuously, not as a binary property. The practical consequence: a low-authority design can be tuned with any method; a high-authority, high-latency design has a specific [Kp_floor, Kp_ceiling] window to hit (Section 5.2).
2. **Binary classification noise, corrected in two independent passes (Section 4.6).** A 15-seed re-evaluation found 60% of original high-sensitivity labels flip; a subsequent gain-search audit found the coarse search itself was underpowered. Both corrections strengthened the physical signal (AUC 0.944→0.957→0.975). 33.6% of at-risk designs remain "uncertain" even at 30 seeds, concentrated above θ̈_max ≈ 55 rad/s² on the ceiling test — treat these as "elevated risk, indeterminate severity" rather than forcing a binary label.
3. l_nozzle = 0.25 m fixed. The formula scales linearly with l_nozzle; untested at other airframe lengths.
4. Simulation only. Hardware validation required for the 6.0° flight-detection threshold and the 54.8 rad/s² Youden-J boundary.
5. Flight detection study (Section 5.2) ran on n = 90 designs (45 narrow-window + 45 stratified wide-window) from the n = 2,400 study. Threshold (6.0°) is calibrated in simulation; hardware validation is required before deployment on real rockets.
6. Latency is the primary gain ceiling compressor (ceiling 320→40-90 at latency = 6 for θ̈_max > 60 rad/s²). Wind barely shifts the floor (<5 Kp units across full wind range). Designs with high θ̈_max AND high latency (≥ 5 steps) should be flagged even if spec-only θ̈_max < 55 rad/s².

### 7.4 The gain window is a continuum, not a binary class (primary data, not just inference)

Earlier drafts of this paper argued, from indirect evidence (the small n = 12 boundary experiment in Section 4.6.3, and the Wilson-CI "uncertain" flag covering 33.6% of at-risk designs in Section 4.5.7), that FRAGILE/EASY was probably a thresholded continuum rather than two natural classes — but neither piece of evidence was a direct, adequately-powered test of that question. **Section 4.0 is that test**, run specifically to settle it: n = 222 designs, stratified for coverage of the full θ̈_max range, evaluated for a continuous margin outcome. The result confirms the continuum claim directly and quantitatively — the fraction of designs below the 0.80 reliability threshold rises from 2% (θ̈_max < 40) to 46% (θ̈_max > 300), with no point of sharp transition, R² = 0.33 (5-fold CV, n = 262 pooled).

This means the binary FRAGILE label used in Sections 4.1–4.6 is a thresholded operationalization of an underlying continuous quantity — gain margin under a 1.4× gain perturbation — not evidence of two naturally separated rocket populations. **Section 4.0, not this subsection's earlier indirect argument, is now the primary evidence for that claim**, and the practical implication is unchanged: a different robustness criterion would move which specific designs are labeled FRAGILE without changing the underlying continuous relationship, which is why Section 4.0's R² is the more fundamental and more defensible number in this paper, and why the binary AUC/Cohen's-d results elsewhere should be read as describing one particular threshold on that same continuum, not a separate or stronger finding.

### 7.5 Anticipated objections

**"How do you know the simulator is realistic?"** It isn't fully validated — hardware validation (Section 8) is the next planned step. What can be said now: the simulator includes nonlinear aerodynamics, a thrust curve, actuator slew/backlash/deadband, sensor noise, and latency; the central finding (θ̈_max predicts gain sensitivity) follows from Newton's second law for rotation plus a closed-loop delay margin — established physics independent of any specific simulator's fidelity details. The simulator's role is to quantify *how much* margin exists, not to manufacture the mechanism.

**"Isn't the gain-window effect just rediscovering torque divided by inertia?"** The formula is textbook Newton (τ = Iα). The novel claim is empirical: among a wide field of candidate predictors (wind strength, servo slew rate, aerodynamic stability, latency, and ten engineered features, plus trained ML models given all raw design variables), this single quantity is sufficient and outperforms the learned alternatives. That is a screening result, not a restatement of Newton's law.

**"Why trust the result after multiple bugs?"** Because each correction moved details without reversing the central ranking, and three independent corrections *strengthened* it: AUC 0.944→0.957→0.975, Cohen's d to 3.71, K_u separation p-value 0.0072→2.9×10⁻⁵→4.17×10⁻⁷. The one exception — the false-approval reversal in Section 5.1 (0%→100% for 2 INFEASIBLE designs) — is reported here deliberately; the one place the story got worse is itself part of the trustworthiness argument.

### 7.6 Confidence tiering: not every claim in this paper deserves equal trust

This paper makes claims at several different levels of abstraction, and they do not all rest on equally strong evidence. Presenting them with uniform confidence would overstate the weaker ones. In decreasing order of how strongly the data support them:

1. **Strongest: the ranking result.** A scalar authority/inertia quantity (θ̈_max) ranks designs by gain-margin loss substantially better than every alternative candidate tested — wind, slew rate, aerodynamic stability, their interactions, and trained ML models on all raw parameters. This survived four independent attempts to break it (two classification audits, a raw-feature ML baseline, and the continuous re-derivation in Section 4.0) and was strengthened, not weakened, by each. This is the claim to defend most aggressively.
2. **Strong: Iyy is not replaceable by a simpler proxy.** Removing Iyy and renormalizing by mass alone costs 0.21–0.26 AUC (Section 4.3) or 0.26 cross-validated AUC in a trained model (Section 4.3.1). This is a meaningfully different and more specific claim than "more thrust or more gimbal travel means more fragility" — it says authority *relative to* inertia is what matters, which is closer to a real engineering insight than a single-variable correlation would be.
3. **Moderate: control-loop latency independently compresses the gain ceiling.** The qualitative direction (delay reduces phase margin, narrows the stable gain range) is textbook control theory and the data are consistent with it. The *exact* quantitative law (Kp_max ≈ 380/latency) rests on a smaller sample (n = 20 conditions) and a partly-fit correction factor; treat the mechanism as more credible than the specific constants.
4. **Moderate: the mechanistic chain (Newton → bang-bang amplitude → K_u → ceiling collapse).** Each individual step is either exact (Newton's law, the describing-function K_u formula) or empirically supported on a meaningful sample (the relay-probe re-derivation, n = 36 final population). But the chain as a whole is a derived argument built from smaller subsample analyses and relay-method assumptions, not a direct large-sample result the way the ranking claim (tier 1) is. If forced to choose, this paper defends the predictor (tier 1) more confidently than the mechanism (tier 4) that explains it — the predictor would remain true even if some detail of the proposed mechanism turned out to be wrong.
5. **Moderate (upgraded — three architectures + frontier + extension + theoretical derivation).** The LQR test (Section 4.0.2) established that optimal linear state-feedback encounters the same Π-driven compression (ρ = −0.747, p = 4.8×10⁻¹⁰). SMC with a boundary layer confirms independently (ρ = −0.753, p = 2.8×10⁻¹⁰). The performance frontier (Section 4.0.3) maps PID failure at Π = 5,214; ADRC with standard ω₀/ωc succeeds beyond Π = 12,648. The frontier extension (n = 44, up to Π = 49,893) found zero ADRC failures with adaptive ω₀/ωc — ADRC has no confirmed ceiling within the stress-test range. The first-principles derivation of Π (Section 4.0.4) shows the τ² exponent is theoretically predicted (not just empirical) from ceiling τ⁻¹ + floor τ⁺¹ = window τ⁻², validated at slope = −1.029 vs. theory −1.000. The twofold ρ-ratio (PID: −0.668 vs. ADRC: −0.325) is a quantitative signature of ESO decoupling. What remains Provisional: whether nonlinear or adaptive controllers (MPC, gain scheduling) can extend further, and whether any of this translates to hardware.
6. **Weakest, by construction: the simulator's absolute correctness.** Every result in this paper — the predictor, the mechanism, the ADRC comparison, the matched-configuration test in Section 4.0.1 — depends on a simulator built for this project. None of it has yet been checked against a real rocket in flight. The audits in Section 4.5–4.6 found and fixed real bugs in this simulator, which is reassuring evidence of care, but it is not the same as external validation. Hardware testing (Section 8) is the one experiment that would change this tier's status, and until it happens, every claim above should be read as "true within this simulator, with a stated mechanism that is independently plausible from established physics," not as "confirmed in reality."

---

## 8. Hardware Validation Plan

### 8.1 Priority experiments

**A. Kp = 2 detection** — fly a high-θ̈_max design at Kp = 2; expect RMS > 6.0°. Fly a low-θ̈_max design; expect RMS < 6.0°. Validates the flight detection workflow (Section 5.2).

**B. Kp_simple vs. Kp_full** — same high-θ̈_max design at (1) simple-model Kp (~2–10), (2) full-physics Kp (~40–80). Validates the 56% false rejection prediction (Section 5.1).

**C. ADRC vs. PID** — video comparison, same rocket. PID at Kp_simple: oscillation. ADRC at ωc = 5: smooth tracking. Most visually compelling demonstration.

**D. θ̈_max threshold** — two hardware configurations crossing 70 rad/s². Validates Section 4.1 formula as a preflight screening tool.

**E. `gain_advisor.py` recommendation check** — run the tool (Section 6.4) on the specs of 2–3 hardware configurations before building them; confirm the recommended PID Kp range and ADRC ωc ceiling each produce stable flight, and that a design flagged "elevated/high" risk is measurably harder to tune by hand than one flagged "low." This is the most direct test of whether the tool is actually useful, not just numerically self-consistent.

**F. Matched-configuration flight test (highest-value single experiment per external review).** Build or modify one airframe to produce 2–3 configurations differing *only* in Iyy (e.g. movable ballast mass at different fixed stations) — everything else (motor, gimbal limit, servo, electronics) identical. Fly each configuration across a small set of Kp values and measure success/oscillation directly, reproducing the protocol in Section 4.0.1 on real hardware. A single such result, even with only 2–3 Kp points per configuration, would be more valuable than additional simulation, because — unlike every other result in this paper — its *qualitative* conclusion (does the viable Kp range narrow as Iyy decreases) does not depend on the simulator's correctness, only its quantitative thresholds do.

### 8.2 Minimum success criteria

Confirmatory if: (A) Kp = 2 produces RMS > 6.0° on one high-θ̈_max design; (B) switching to full-physics Kp materially reduces RMS; (D) spec formula correctly classifies at least one real hardware configuration; (F) the matched-Iyy configurations show a visibly narrower usable Kp range for the lower-Iyy build.

---

## 9. STS Framing

### 9.0 What changed since the last draft, and why

An earlier draft framed the result as a binary classifier. A reviewer raised four valid objections: is the headline about gain windows or a label; has anything been proven; doesn't AUC on a rare class hide a base-rate trick; and do the categories need to be distinct at all? Answering honestly required new experiments: (1) Section 4.0's continuous regression, which replaces the classifier as the headline claim and sidesteps the AUC/base-rate objection; (2) Section 6's cross-architecture design rule and runnable tool, which make the result actionable. The binary classification result is retained as audit trail in Sections 4.1–4.6.

### 9.1 One-sentence answer to "What did you discover?"

> A compact invariant computable from a motor data sheet and a mass measurement — the maximum angular acceleration a rocket's TVC system can produce — predicts, *continuously* (R² = 0.33, n = 262, cross-validated, no base-rate ambiguity), how much a hobby rocket's gain margin narrows as authority and control-loop latency increase; that same quantity yields two structurally different, quantified bandwidth/gain ceiling laws for PID and ADRC (latency-dominated vs. authority-and-latency-dominated), packaged into a runnable tool that recommends an actual operating range for either controller from hardware specs alone — with a secondary binary-classification framing (AUC = 0.975, Cohen's d = 3.71) retained as an audited, but explicitly non-headline, decision aid.

### 9.2 The scientific journey

Five independent audits overturned earlier interpretations but consistently strengthened the central θ̈_max finding: (1) a π/180 servo unit error manufacturing a false "uncontrollable" regime; (2) a units bug in the gain-mechanism study; (3) a statistical audit showing the 3-seed robustness test could not resolve its own threshold (inflated FRAGILE count 45→30→36, AUC 0.944→0.957→0.975, Cohen's d to 3.71); (4) a gain-search audit showing the coarse grid had silently mislabeled a controllable design R0475 as INFEASIBLE; (5) a literature check finding linear ADRC is mathematically equivalent to a filtered PID (Carlson 2025), narrowing the "cross-architecture" claim to "two disturbance-handling conventions, divergent in the saturated regime." The first four corrections strengthened the result numerically; the fifth narrowed a claim's scope without changing numbers. There were also two places the story got *worse*: the false-approval finding reversed from 0%→100% for the 2 uncontrollable survivors (Section 5.1), and the ADRC-PID equivalence caveat (Section 6.0) restricted the architecture comparison's framing. Both are reported honestly here. That pattern — each audit weakening the project in some small local way while the central predictor gets stronger — is the trustworthiness argument.

### 9.3 Novelty summary

| Aspect | Status |
|--------|--------|
| θ̈_max = T·sin(δ)·L/Iyy formula | Textbook (Newton's law) |
| **Continuous gain-margin regression: R² = 0.33 (5-fold CV), n = 262, p = 5.8×10⁻¹¹; curve plateaus above td ≈ 300 (latency-dominated regime) (Section 4.0)** | **Novel — primary claim, avoids the AUC/base-rate framing entirely** |
| **ADRC bandwidth ceiling comparison (Section 6.2)**: n = 46 cells (6 authority levels × 8 latencies), R² = 0.823; ωc_max ≈ 70/(latency^0.57 · θ̈_max^0.31) — latency-dominated with secondary authority coupling; authority exponent 1.5× PID's (−0.31 vs −0.20), structurally distinct from PID's pure latency ceiling | **Moderate — n = 46 cells, first quantified ADRC/PID ceiling comparison for this hardware class; the n = 21 authority exponent did not replicate, which is why the n = 46 result is the reported value** |
| Runnable builder tool, `tools/gain_advisor.py`, recommends actual Kp range / ADRC ωc from specs (Section 6.4) | **Applied, not independently novel** — the tool's value comes from the formulas it implements (the novel results in rows above); the tool's existence as a wrapper is not a separate research contribution. Included because a concrete, runnable artifact answers the "what does a builder actually do?" question a judge will ask. |
| **Matched single-airframe confirmation: only Iyy varied, window narrows 142×→82×→57×→39×→33× (td 93→189 rad/s²) with no other parameter changed; extended Kp grid to 1,280 confirms true ceilings exist and floor-rising is the dominant mechanism (Section 4.0.1)** | **Novel — strongest internal-validity test in the project, though still simulation-only** |
| **Controller-invariance across three architectures (Section 4.0.2): LQR (ρ = −0.747), SMC boundary-layer (ρ = −0.753), and PID (ρ ≈ −0.78) all give Spearman ρ(Π, window) ≈ −0.75 on the same 50 designs. Classical integral PID also fails at extreme Π (n_pass = 0 for R2080); ESO succeeds where integral does not (Section 4.0.2.1). Common element of all non-ESO architectures: no active upstream disturbance cancellation.** | **Novel — ESO is specifically the escape mechanism; integral (the classical alternative) is insufficient at extreme Π; tested explicitly, not argued** |
| **Performance frontier (Sections 4.0.3–4.0.4): PID fails at Π = 5,214; ADRC standard fails at Π = 12,648 (population edge, not a ceiling); extended grid (n = 44, Π up to 49,893) finds zero ADRC failures — no fixed ceiling, only a ω₀/ωc tuning requirement that scales with Π; ρ(Π, SR_adrc) = −0.325 vs. ρ(Π, SR_pid) = −0.668 — 2× difference quantifies ESO decoupling. First-principles derivation of Π: ceiling τ⁻¹ (DIPDT) + floor τ⁺¹ (bang-bang) → window = 1/Π, slope = −1.029 vs theory −1.000.** | **Novel — first quantitative frontier map for any TVC class; no-ceiling finding and τ² derivation are both new** |
| **Saturation mechanism test (Section 6.2): 2×2 factorial (PID/ADRC × saturation on/off) on 15 designs. PID−sat = 1.000 for ALL designs including Π = 12,648 — saturation is necessary AND sufficient for PID failure. ADRC never saturates (slew_frac = 0.000 for all 15). ρ(Π, slew_frac_pid) = +0.875, p = 2×10⁻⁵. Carlson equivalence empirically confirmed: without saturation, PID and ADRC both achieve SR = 1.000 identically.** | **Novel — first causal isolation of the saturation mechanism; closes the "argued but not measured" gap in the cross-architecture claim** |
| Caveat found via literature check: linear ADRC (bandwidth-tuned) is mathematically equivalent to a filtered 2-DOF PID (Carlson 2025) — "cross-architecture" reframed to "two disturbance-handling conventions, divergent in the saturated regime" (Section 6.0) | Self-identified limitation — strengthens credibility by being disclosed rather than found by a reviewer |
| θ̈_max predicts gain sensitivity, CV AUC = 0.975 (final, twice-corrected, n = 36 of 2,400) | Novel, secondary/decision-rule framing |
| log(θ̈_max × latency) (no training, 2 scalars) outperforms all trained models on 10 raw features; θ̈_max alone ties RF/LR; Iyy drop-one −0.346 (was −0.263) | Novel — circularity refuted, re-run on final population (n=36 narrow-window, 2026-06-16) |
| 4 competing hypotheses rejected; gain sensitivity is mechanical, not environmental | Novel |
| Dimensionless Π analysis — Iyy is irreplaceable (drop to AUC 0.669 without it) | Novel |
| Gain ceiling equation: Kp_max ≈ 380/latency_steps (theory + empirical 2.1× correction) | Novel — secondary/exploratory (R² = 0.53, see §4.6) |
| Combined ceiling×floor window formula validated against held-out data and rejected (AUC = 0.500) | Novel negative result |
| K_u re-derived via literal relay probe on final population: wide-window 91.2 vs narrow-window 29.2 (3.12×, p=4.17e-07) | Novel — strongest mechanistic separation yet |
| Two independent internal audits found and corrected an underpowered classification test AND an underpowered gain search (Section 4.6) | Novel — methodological rigor, two-stage |
| Cross-architecture generalization: same θ̈_max × latency predictor explains 94% of PID failure and the residual 6% of ADRC bandwidth-tuning need (Section 6.1) | Novel |
| Literature search (not self-assessment) found no prior hobby-TVC design-space classifier or cross-architecture test | Novel — externally corroborated |
| S2R: simple model gives 100% false approval for the 2 genuinely uncontrollable designs (reverses earlier "never dangerous" claim, Section 5.1) | Novel — and a negative result for the project's own earlier claim |
| Flight detection AUC = 0.954 (7-seed, n = 36 final population, strengthened from 0.943) | Novel, current |

### 9.4 STS probability estimate

- State regional / ISEF qualifier: > 90%
- STS semifinalist (300/1800): 55–68%
- STS finalist (40/300): 55–66% simulation-only; 68–80% with hardware validation

These estimates are higher than the previous draft's (50–62% / 64–76%) for three concrete reasons: (1) the continuous regression (Section 4.0) removes the base-rate-trick objection that the binary AUC framing invited; (2) the ADRC bandwidth ceiling (Section 6.2) is a new, structurally distinct, quantified result with no identified precedent; (3) the builder tool (Section 6.3) directly answers "what can someone actually do with this?" A sufficiently expert judge may still object that "authority/inertia ratios narrowing PID's margin, and ADRC needing bandwidth derating for high-authority loops, are known phenomena in the abstract" — the defensible response is that the contribution is the *quantification and cross-architecture comparison* for a previously unstudied hardware class, packaged into a runnable tool, not the existence of either phenomenon.

What crosses 75%: hardware video of PID oscillation vs. ADRC; confirmed 6.0° flight detection threshold; the matched-Iyy hardware test (Section 4.0.1, Section 8.1.F) showing a narrower usable Kp range for the lower-Iyy build; demonstration that switching Arduino→Teensy reduces gain-sensitivity risk (latency-driven window collapse confirmed in real hardware).

The latency stress test now gives a compelling hardware experiment that didn't exist before: **same rocket, same gain, different MCU** → predict and confirm narrow-window vs. wide-window behavior on the latency axis alone. No mechanical modification required.

### 9.5 Next steps, prioritized (as of 2026-06-18)

COMPLETED SINCE LAST DRAFT: (a) SMC controller-invariance test — confirms same ρ(Π,window)≈−0.75 as LQR (Section 4.0.2); (b) Observer universality test (Section 4.0.2.1) — classical integral PID tested as alternative disturbance estimator; integral partially helps at intermediate Π but fails at extreme Π (R2080: n_pass_pidi=0); ESO is specifically the escape mechanism; Pi formula corrected to keff×lat² throughout; (c) Performance frontier — maps PID failure boundary (Π=5,214) and ADRC extension (Section 4.0.3); (d) Frontier extension to Π=49,893 (stress-test population, n=44) — zero ADRC failures with extended ω₀/ωc; confirms ADRC has no fixed Π ceiling within the tested range (Section 4.0.3); (e) Theoretical derivation of why Π=keff×τ² — ceiling τ⁻¹ (DIPDT) + floor τ⁺¹ (bang-bang) = window τ⁻² = 1/Π; keff coefficient = −1.001 (theory −1.000 exactly), slope = −1.029 vs theory −1.000, all 7 exponents confirmed (Section 4.0.4).

1. **Hardware: the matched-configuration test (Section 8.1.F).** Highest-value remaining experiment. A bench test (clamped airframe, IMU + servo, only Iyy/ballast varied) reproducing Section 4.0.1's protocol would suffice — far cheaper than a full flight campaign, and it is the only experiment that changes what is *known* rather than *re-confirmed*.
2. **Tighten narrative for presentation:** lead with Section 4.0 and Section 4.0.3 (frontier figure); consolidate the four self-correction beats into one paragraph; fold negative results into limitations.

---

## Appendix A — Data Files

| File | Finding |
|------|---------|
| `experiments/results/combined_margin_regression_py.csv` | **4.0, primary result** (continuous margin regression, n=222) |
| `experiments/results/matched_configuration_py.csv`, `matched_configuration_summary_py.csv` | 4.0.1 (original 9-point, 320-capped sweep) |
| `experiments/results/matched_config_extended_kp_py.csv`, `matched_config_extended_kp_summary_py.csv` | **4.0.1, primary result** (18-point, 1280-ceiling sweep; true ceilings, definitive) |
| `experiments/results/fidelity_cutoff_by_td_py.csv` | **5.1.1** (module-level evaluation hardness by θ̈_max tier) |
| `experiments/results/adrc_bandwidth_ceiling_summary_py.csv` | **6.3, primary result** (ADRC ceiling law) |
| `experiments/results/exp1_regime_index_py.csv` | 4.1–4.3 (original 3-seed labels, superseded) |
| `experiments/results/exp1_reclassify_15seed_py.csv`, `exp1_corrected_population_py.csv` | 4.5.6 (first correction pass) |
| `experiments/results/exp1_final_correction_py.csv`, `exp1_final_population_py.csv` | 4.5.7, **final/current population for all citations** |
| `experiments/results/relay_final_comparison_py.csv` | 4.4 Step 4 (K_u, definitive) |
| `experiments/results/kp_window_sweep_v2_py.csv` | 4.6.3 window collapse |
| `experiments/results/exp4_s2r_gains_final_py.csv` | 5.1 (final, includes INFEASIBLE false-approval reversal) |
| `experiments/results/flight_sig_final_py.csv` | 5.2 (final, n=36) |
| `experiments/results/adrc_dissolution_py.csv` | Section 6.1, **current** |
| `experiments/results/adrc_saturation_test_py.csv` | **6.2** (saturation mechanism test, 2×2 factorial, n=15) |
| `experiments/results/lqr_controller_test_py.csv`, `lqr_gain_sweep_py.csv` | **4.0.2** (LQR invariance, n=50) |
| `experiments/results/smc_controller_test_py.csv`, `smc_sweep_detail_py.csv` | **4.0.2** (SMC invariance, n=50) |
| `experiments/results/observer_universality_py.csv` | **4.0.2.1** (observer universality: PD vs PID-I vs ADRC, n=50, Pi=keff×lat²) |
| `experiments/results/performance_frontier_py.csv` | **4.0.3** (frontier, n=63) |
| `experiments/results/adrc_frontier_extension_py.csv` | **4.0.3** (frontier extension, n=44, Π up to 49,893) |
| `experiments/results/window_ratio_v2_py.csv` | **4.0.4** (Pi theory validation input, n=116 non-censored) |
| `experiments/results/adrc_step_tracking_py.csv` | Appendix C (superseded by Section 6) |
| `outputs/sts_gold_index.html` | All (interactive; built on pre-final-correction labels) |

## Appendix B — Rejected Hypotheses

| Hypothesis | Key datum |
|-----------|-----------|
| Wind determines gain sensitivity | r(wind, FRAGILE) = −0.001; adding wind reduces AUC by 0.014 |
| Slow servos determine sensitivity | r(slew, FRAGILE) = +0.013; solo AUC = 0.45 |
| Aerodynamic instability determines sensitivity | r = +0.034; solo AUC ≈ 0.51 |
| Dimensionless Π (gravity-normalized) outperforms θ̈_max | T×sin(δ)/(m×g): AUC = 0.730, −0.124 |
| Iyy × wind controllability boundary | Entire INFEASIBLE class was a π/180 bug artifact |
| Sensor noise globally dominates fidelity | Wind dominates GO/NOGO; noise dominates RMS (different question) |
| Aerodynamic instability improves maneuverability | Stable outperforms unstable in all tests |
| Combined ceiling×floor window-ratio formula predicts window width as a unit | AUC = 0.500 (chance) on held-out boundary data; floor piece alone R² = −0.003 |
| MARGINAL is a genuine third regime distinct from EASY/FRAGILE | 15-seed re-evaluation: 100% of MARGINAL designs reclassify as EASY |

---

## Appendix C — ADRC vs. PID: Original Step-Tracking Comparison (superseded by Section 6)

*Pre-correction n = 1,200 population, step-tracking task, oracle-tuned ADRC. Superseded by Section 6.1's attitude-hold comparison on the final population. Retained for the mechanism explanation only.*

θ_step = 15°, t_end = 4 s, full physics. PID at best_Kp; ADRC at ωc=5, ω₀=25, b0=keff from specs.

| Metric | PID | ADRC | Improvement |
|---|---|---|---|
| SR | 0.080 | 0.972 | 12× |
| RMS | 47.9° | 2.3° | 21× |
| Peak | 92° | 17° | 5.4× |
| Rise time | 0.242 s | 0.804 s | 3.3× slower |

**Mechanism:** Kp ≈ 80 needed for wind rejection → servo saturates during the step → 77° overshoot. ADRC's ESO cancels wind independently, so the servo stays unsaturated during the step command. This coupling between wind-rejection gain and step-response saturation is the structural problem; ADRC separates the two constraints via its observer.
