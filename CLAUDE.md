# TVC Rocket Research – Claude Context

## Project Purpose

This project investigates how physical rocket properties determine:

1. The boundary between controllable and uncontrollable TVC rockets.
2. The simulator fidelity required to make correct engineering design decisions.
3. The robustness–maneuverability tradeoff in low-cost TVC rockets.

This is NOT a PID tuning project.

The primary scientific goal is understanding design-space structure and simulator fidelity requirements.

---

# Research Philosophy

Act as a skeptical reviewer.

Do NOT optimize for exciting conclusions.

Do NOT defend previous work automatically.

Always:

* inspect data before drawing conclusions
* quantify uncertainty
* search for alternative explanations
* identify circular reasoning
* identify confounding variables
* distinguish visualization from evidence
* distinguish correlation from causation

Prefer falsification over confirmation.

If a conclusion is weak, say so.

---

# Current Thesis Direction

Current working thesis (updated 2026-06-05):

A TVC rocket's controllability boundary is determined primarily by its rotational inertia (Iyy)
relative to its wind-environment loading, NOT by aerodynamic instability (p_unstable).
Rockets with small Iyy in high-wind environments require high-bandwidth controllers,
which require accurate slew and latency modeling for correct design decisions.

The simulator fidelity requirements depend on where a design sits relative to this
Iyy × wind_strength controllability boundary.

IMPORTANT: The p_unstable proxy has near-zero correlation with regime under full-physics
evaluation. Do NOT claim that aerodynamic instability is the primary controllability predictor.

Correct claim: Iyy and wind_strength jointly determine regime; p_unstable is a secondary factor.

The project is moving away from global rankings and toward regime-dependent fidelity requirements.

---

# Experiment Status

## Exp1 – Regime Mapping

Status: Major revision complete (2026-06-05). All prior Exp1 results are SUPERSEDED.

Methodology (current):
* 1200 LHS designs with T/W > 1 feasibility filter (no longer includes physically unliftable designs)
* Full-physics evaluation: nonlinear_aero + dyn_aero + thrust_curve + cg_shift ON
* Gain search: 5×5 grid KP=[2,5,15,40,80], KD=[1,2,8,16,32] with 2-seed autotune
* Autotune objective: highest mean success rate across seeds 1+2; tiebreak by minimum RMS
* Nominal evaluation: 3-seed average (wind is stochastic; single-seed is unreliable)
* Under/over robustness: single seed=1 (binary pass/fail sufficient)

Design space (current):
* servo_slew_deg_s: [5, 120], static_margin: [-0.30, 0.30], Cm_alpha: [-90, -15]
* motor_scale: [0.5, 3.0], max_gimbal_deg: [2, 15]

Main outputs (4-class scheme, counts as of 2026-06-05):

* EASY (n=850, 71%): nom_sr=1.00, robustness=1.00 — works in all conditions
* MARGINAL (n=241, 20%): nom_sr=0.72, robustness=1.00 — WIND-SENSITIVE (not gain-sensitive)
  — REINTERPRETATION: MARGINAL is NOT about high RMS; it is about wind-environment sensitivity.
  — MARGINAL designs have 27% higher wind_strength than EASY (mean 0.303 vs 0.223).
  — They pass all gain robustness checks but fail ~28% of random wind realizations.
* FRAGILE (n=68, 6%): nom_sr=0.76, robustness=0.56 — gain-sensitive (fails some gain conditions)
* INFEASIBLE (n=41, 3%): nom_sr=0.30, robustness=0.23 — cannot be stabilized

RELIABLE BOUNDARY: FRAGILE+INFEASIBLE = 109 (9.1%) is stable across methodology variants.
SOFT BOUNDARY: EASY vs MARGINAL split is autotune-sensitive (varies ~350 designs across runs).

Regime evolution (INFEASIBLE count as methodology improved):
* Old simple-mode: INFEASIBLE=235 (19.6%) — mostly tuner artifacts
* Full-phys 4x4: INFEASIBLE=74 — grid floor and T/W contamination removed
* Full-phys 5x5 2-seed: INFEASIBLE=41 (3.4%) — current best; 34% residual paradox (near-boundary noise)

Key findings (2026-06-05):

FINDING 1: p_unstable has near-zero correlation with regime (r≈0 for all class comparisons).
Do NOT cite p_unstable as the primary controllability predictor.

FINDING 2: Primary physical predictors of regime are:
  — wind_strength (design parameter): strongest predictor; INFEASIBLE designs have 44% higher wind
  — Iyy: second strongest; INFEASIBLE Iyy=0.014 vs rest=0.025 (44% lower)
  — motor_scale: moderate predictor; higher thrust → higher q_dyn → stronger wind loading
  p_unstable: negligible (r≈0 across all regime comparisons)

FINDING 3: Aerodynamic instability does NOT make rockets more wind-resistant or maneuverable.
The opposite is true: unstable aerodynamics amplify wind disturbances. The TVC fights both
instability and wind simultaneously rather than having aerodynamics partially absorb the wind.

FINDING 4: The controllability boundary lives in (Iyy, wind_strength) space, not p_unstable space.
Small Iyy + strong wind = fast dynamics overwhelmed by disturbances = INFEASIBLE.

Confidence: MEDIUM-HIGH (regime counts are autotune-sensitive near EASY/MARGINAL boundary;
FRAGILE+INFEASIBLE total is stable; physical predictor findings are robust across runs).

---

## Exp4 – Fidelity Ablation

Status: Complete (N=1200, 3 seeds per condition)

Goal:

Determine which simulator fidelity terms change engineering conclusions.

Modules:

* wind
* sensor_noise
* slew
* backlash
* latency
* thrust_var
* deadband

Key finding — use frequency-of-effect (NOT tie-broken dominance percentages):

Old "Wind is dominant X%" figures are replaced. 63% of designs had tied dominance,
making those percentages artifacts of Python dict ordering. New metric: % of designs
where each module causes any decision flip (effect_any), split by direction.

Frequency-of-effect by regime (2026-06-03, 4-class scheme):
* EASY:      Slew 62.7% any (33% helps / 30% hurts), Sensor Noise 61.9%, Wind 55.3%
* MARGINAL:  Sensor Noise 87.0%, Slew 77.3%, Wind 73.7%
* FRAGILE:   Sensor Noise 89.3%, Wind 75.7%, Slew 75.7%
* INFEASIBLE:Sensor Noise 100%, Wind 96.2%, Slew 72.3%

Critical finding on Slew bidirectionality:
* In EASY: 33% of designs — removing slew HELPS (slew was limiting); 30% removing slew HURTS
* Designs where removing slew hurts: higher original servo_slew (85.4) + lower p_unstable (1.265)
* This is a gain-confound artifact: aggressive gains tuned with fast actuator become unstable
  when slew is removed. Must be disclosed as a limitation.

Metric comparison finding (4-class):
* EASY/MARGINAL/FRAGILE: RMS metric and decision metric disagree on top module
* Sensor noise tops the RMS ranking; wind + sensor noise top the GO/NOGO ranking
* INFEASIBLE 85.1% agreement (both metrics agree: wind dominates)

Fidelity complexity (4-class):
* EASY: 47% need zero fidelity modules (simple simulator OK)
* MARGINAL: 27% need zero modules
* FRAGILE: 13% need zero modules
* INFEASIBLE: 0% — all need 2-3 modules

Simple-model decision error rates (exp4simple paired run, 2026-06-03):
* INFEASIBLE: simple model has 99.6% FALSE APPROVAL rate
* MARGINAL: (new — not yet computed separately; was pooled with FRAGILE)
* FRAGILE: simple vs full agreement = 50.3% (coin flip — but this was OLD pooled FRAGILE)
* EASY: simple vs full agreement = 78.3% — simple model mostly correct but 21.7% wrong
* There were ZERO false rejections in any regime (simple model is always optimistic)
NOTE: The old FRAGILE agreement (50.3%) mixes MARGINAL and FRAGILE. Recompute per new class.

Key open issue:

Exp4 baseline mismatch: Exp4 full-fidelity includes thrust_var fault (keff drops 15% at 1.5s)
but Exp1 regime labels were computed without this fault. Caveat added to all Exp4 figure captions.
This affects all Exp4 claims — disclosed in figures, must be addressed before final publication.

Current direction:

Build a Fidelity Requirement Atlas:

Given a rocket's physical properties, determine which fidelity terms are necessary to obtain correct design conclusions.

Confidence: MEDIUM-HIGH (finding is robust; baseline mismatch remains a methodological concern)

---

## Exp5 – Design-Space Topology

Status: Partial — topology analysis removed; stratified slew payoff is new lead result

Current outputs:

* gradient field (central finite differences, 3-seed average, range-scaled)
  — Exp5 CSV now has both grad_rms_* (raw, physical units) and grad_scaled_* (range-scaled)
  — Do NOT compare raw grad_rms_* columns across parameters (unit bias)
* evolution paths (5 gradient-descent steps per design, 7200 rows)
* diminishing returns curves (population-level, top 5 parameters)
* NEW: regime-stratified servo slew payoff curves (exp5_slew_stratified_py.csv)

Topology analysis REMOVED from paper figures (2026-06-03):
* Cliff/bowl classification is uniformly distributed across regimes (~15-17% cliff in all)
* Does NOT correlate with regime boundaries or stability proximity
* Not validated by seed-variance test
* Do not cite topology results in paper

Strongest surviving result:

Regime-stratified servo slew payoff (exp5_slew_payoff_stratified.png):
* EASY: plateaus at ~0.60 success rate around 40-50 deg/s
* MARGINAL/FRAGILE: plateau below EASY level (~0.48-0.52) — cannot servo-slew to EASY
* INFEASIBLE: no plateau — keeps slowly improving even at 120 deg/s but never useful
* Key engineering message: beyond ~60 deg/s, slew is not the binding constraint

Known issues:

* Stratified payoff uses full fidelity (thrust_var fault included) — success rates are
  lower than Exp1 values; relative regime ordering is informative, absolute levels are not
* 3-seed gradients: Iyy at 13.9% best_param may still be noise
* INFEASIBLE gradient analysis removed (numerically unreliable for diverging trajectories)
* Gains frozen from Exp1; not re-tuned per slew value — bidirectionality still present

Confidence:

* Stratified slew payoff: MEDIUM-HIGH (clear qualitative ordering across regimes)
* Diminishing returns: MEDIUM-HIGH
* Terrain maps: MEDIUM
* Gradient bottlenecks: LOW (do not cite without multi-seed validation)
* Evolution paths: LOW (do not cite without multi-seed validation)

---

# Known Methodological Concerns

These issues must be remembered during future analysis.

## Exp4 Baseline Mismatch — QUANTIFIED

Exp4 full-fidelity conditions differ from Exp1 evaluation conditions because
Exp4 includes a thrust_var fault (keff drops 15% at t=1.5s) that was NOT present
during Exp1 regime labeling.

Quantified impact (from Exp4A audit, 2026-06-03):
- EASY: 78.3% GO/NOGO agreement  [74.4%, 81.7%]
- FRAGILE: 50.3% agreement  [45.8%, 54.8%] — effectively a coin flip
- INFEASIBLE: 99.6% agreement (thrust fault doesn't change uncontrollable verdict)

Interpretation: The baseline mismatch primarily affects FRAGILE designs.
Any fidelity claim involving FRAGILE designs must caveat this.

Next step: Run exp4simple (FidelityConfig.simple() for all 1200 designs) to get
actual paired simple-vs-full decision comparison.

---

## Decision Dominance Tie Risk

Decision dominance based on delta_success may contain many ties.

Before accepting dominance rankings:

* audit tie frequency
* audit tie-breaking behavior
* verify conclusions are stable

---

## Gradient Reliability

Current Exp5 gradients use 3-seed averaging (improved from single-seed).
3 seeds reduces stochastic variance ~1.7× but is not fully converged.

Potential future fixes:

* 5+ seed gradients (preferred for publication)
* local response surfaces
* surrogate-based derivatives

Topology classification has been hardened with absolute gradient guard
(cliff_abs_min=0.05) to prevent noise-spike misclassification.

---

## Regime Circularity

Be careful when a later analysis "discovers" the importance of variables already used in regime construction.

Always check for circular reasoning.

## Autotune Methodology Sensitivity (2026-06-05)

The EASY/MARGINAL split is highly sensitive to the autotune method used:
* Single-seed: 87% of designs defaulted to grid minimum (Kp=5) — gain floor artifact
* RMS tiebreaker only: over-preferred high gains → EASY=494, MARGINAL=592
* 2-seed success-rate primary, RMS tiebreaker: EASY=850, MARGINAL=241

The EASY/MARGINAL boundary shifts by ~350 designs between autotune variants.
FRAGILE+INFEASIBLE total is stable across all variants (~100-110 designs).

Practical guidance: Report EASY+MARGINAL as "controllable" (91%) and FRAGILE+INFEASIBLE
as "at-risk" (9%). Do not make precise claims about exact MARGINAL vs EASY counts
until a more robust autotune (5+ seed averaging) is implemented.

## Gain Grid Design (2026-06-05)

Current grid: KP=[2,5,15,40,80], KD=[1,2,8,16,32] (5×5 = 25 combinations)
Autotune: 2-seed average success rate primary, then RMS tiebreaker, then best-effort RMS.

Known limitation: 14/41 INFEASIBLE designs still show under_sr > nom_sr (34%).
These are near-boundary designs where the true optimal gains lie between grid points.
Continuous optimization (Bayesian or gradient-based) would resolve this but is out of scope.

## T/W Filter (2026-06-05)

Design space now enforces T/W > 1 in sample_lhs() — iteratively resamples until all
designs have motor thrust exceeding rocket weight. Without this filter, 20.7% of designs
were physically unliftable but appeared EASY in simulation (near-zero aerodynamic forces).

## dyn_aero Reference Pressure (2026-06-05)

When dyn_aero=OFF, the constant reference q_dyn is now per-design:
  q = 0.5 × 1.20 × v_mid²  where  v_mid = max(0.5, (T_eff - m×g)/m) × (t_end/2)
This ranges from ~0.3 Pa (low motor_scale + heavy) to ~1900 Pa (high motor_scale + light).
Old value: hardcoded 540 Pa for all designs — inflated aerodynamic forces by 10-100× for
most designs, making dyn_aero ablation results partly a calibration artifact.

---

# Rejected or Unsupported Claims

Do not present these as established findings.

* Sensor noise globally dominates simulator fidelity requirements.
  (Disproven by delta_success analysis — wind dominates GO/NOGO decisions.)
* Infeasible rockets are primarily aerodynamic-limited.
  (DISPROVEN 2026-06-05: INFEASIBLE is driven by high wind_strength + low Iyy, not high p_unstable.)
* p_unstable is the primary predictor of controllability.
  (DISPROVEN 2026-06-05: p_unstable has near-zero correlation with regime under full-physics eval.
   Iyy and wind_strength are the dominant predictors.)
* Aerodynamic instability improves wind resistance or maneuverability for TVC attitude-hold.
  (DISPROVEN: instability amplifies wind disturbances; stable aerodynamics help absorb wind.
   The fighter-jet analogy does not apply to attitude-hold TVC rockets.)
* slew × latency interaction is super-additive.
  (DISPROVEN: mean interaction ratio = 0.55 across 50 tests — strongly sub-additive.
   Both ablations push gains in the same direction; combined effect saturates at single-ablation level.)
* Gradient bottlenecks are validated design levers.
* Evolution paths represent physically validated improvement trajectories.
* Topology classes have been proven reproducible.

These remain hypotheses or open questions.

---

# High-Priority Open Questions

1. Which fidelity terms matter in each regime?
   — Partially answered: wind dominates GO/NOGO; sensor_noise dominates RMS.

2. Where do fidelity handoffs occur?
   — Partially answered: spatial maps in fidelity_dominance.py figures.

3. Can fidelity requirements be predicted from physical rocket properties?
   — Open: needs Fidelity Requirement Atlas (atlas-style classifier).

4. Does the stability frontier also predict simulator complexity requirements?
   — Partial: INFEASIBLE needs 2-3 modules always; EASY can often use simple sim.

5. Which Exp5 outputs survive multi-seed validation?
   — Partially answered: 3-seed run complete. servo_slew dropped from 42% → 25% (noise
     confirmed). Iyy enters top-3 (13.9%) but may still be noise — needs 5-seed validation.

6. Which simulator findings survive real flight testing?
   — Not started: requires hardware bench data + 6-12 flights.

7. What is the actual simple-model vs full-model decision disagreement rate?
   — ANSWERED (exp4simple, 2026-06-03): INFEASIBLE false approval rate = 99.6%;
     FRAGILE agreement = 50.3% (coin flip); EASY agreement = 78.3%.
     Simple model is always optimistic — zero false rejections across all regimes.

---

# Flight Validation Plan

Purpose:

Attempt to falsify simulator predictions.

Priority:

1. Regime boundary validation
2. Servo slew diminishing returns
3. Sensor fidelity effects
4. Wind sensitivity

Do not assume simulator correctness.

Flight data has higher evidential value than simulation results.

---

# Response Style

Explain concepts clearly.

Assume the researcher is still learning advanced experimental design and statistics.

When identifying flaws:

* explain why they matter
* explain their practical impact
* suggest possible fixes

Avoid unnecessary jargon.

Teach while critiquing.
