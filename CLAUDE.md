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

Current working thesis:

The physical regime of a TVC rocket determines both:

* whether the rocket is controllable
* which simulator fidelity terms are required to make correct engineering decisions

Different regions of design space may require different simulator fidelity.

The project is moving away from global rankings and toward regime-dependent fidelity requirements.

---

# Experiment Status

## Exp1 – Regime Mapping

Status: Strong (4-class scheme as of 2026-06-03)

Designs:

* 1200 Latin Hypercube samples

Main outputs (4-class scheme, counts as of 2026-06-03):

* EASY (n=488): robust to all gain conditions, meets quality thresholds
* MARGINAL (n=300): robust to all gain conditions, high steady-state RMS (>8 deg)
  — passes all wind/gain tests; fails only the EASY RMS quality threshold
  — NOT wind-sensitive; tracking quality is the limiting factor
* FRAGILE (n=177): fails at least one gain condition; genuinely wind-sensitive
* INFEASIBLE (n=235): fails even nominal evaluation

CRITICAL: Old FRAGILE (n=477) was split into MARGINAL + FRAGILE.
62.9% of old "FRAGILE" designs (MARGINAL) are robust with robustness=1.0.
Any claim that references old FRAGILE regime is now split between MARGINAL and FRAGILE.

Important findings:

* clear stability frontier
* strong p_unstable interaction
* strong servo slew interaction
* meaningful robustness differences between regimes

Confidence: HIGH

Treat Exp1 as the most reliable foundation of the project.

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

---

# Rejected or Unsupported Claims

Do not present these as established findings.

* Sensor noise globally dominates simulator fidelity requirements.
  (Disproven by delta_success analysis — wind dominates GO/NOGO decisions.
   Sensor noise dominates RMS shifts only, which is the wrong metric for engineering decisions.)
* Infeasible rockets are primarily aerodynamic-limited.
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
