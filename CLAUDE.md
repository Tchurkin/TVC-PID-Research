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

Status: Strong

Designs:

* 1200 Latin Hypercube samples

Main outputs:

* EASY regime
* FRAGILE regime
* INFEASIBLE regime

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

Key finding (use delta_success, not delta_rms):

Wind is the decision-dominant module in all three regimes:
* EASY: Wind 55.3%, Slew 24.8%, Sensor noise 7.0%
* FRAGILE: Wind 50.1%, Slew 20.8%, Sensor noise 20.3%
* INFEASIBLE: Wind 72.8%, Sensor noise 22.1%

Metric comparison finding:

RMS-dominant and decision-dominant modules agree only 22.5% (EASY), 45.9% (FRAGILE), 85.1% (INFEASIBLE) of the time.
Sensor noise tops the RMS ranking; wind tops the GO/NOGO ranking.
The choice of metric determines the ranking.

Fidelity complexity:
* 47% of EASY designs need zero fidelity modules (simple simulator OK)
* 0% of INFEASIBLE designs can use a simple simulator (all need 2-3 modules)

Simple-model decision error rates (exp4simple paired run, 2026-06-03):
* INFEASIBLE: simple model has 99.6% FALSE APPROVAL rate — virtually every physically
  unstable design would be wrongly approved by a simple simulator
* FRAGILE: simple vs full agreement = 50.3% (coin flip) — simple model is useless here
* EASY: simple vs full agreement = 78.3% — simple model mostly correct but 21.7% wrong
* There were ZERO false rejections in any regime (simple model is always optimistic)

Key open issue:

Exp4 baseline mismatch: Exp4 full-fidelity includes thrust_var fault (keff drops 15% at 1.5s)
but Exp1 regime labels were computed without this fault. Only 71.3% of designs have the same
GO/NOGO between Exp4 full and Exp1. This must be addressed before final claims.

Current direction:

Build a Fidelity Requirement Atlas:

Given a rocket's physical properties, determine which fidelity terms are necessary to obtain correct design conclusions.

Confidence: MEDIUM-HIGH (finding is robust; baseline mismatch remains a methodological concern)

---

## Exp5 – Design-Space Topology

Status: Complete (N=1200), results exploratory — require multi-seed validation

Current outputs:

* gradient field (central finite differences, range-scaled)
* topology classification: bowl 84.8%, cliff 15.0%, ridge 0.2%
* evolution paths (5 gradient-descent steps per design, 7200 rows)
* diminishing returns curves (population-level, top 5 parameters)

Strongest surviving result:

servo slew diminishing returns (population median, EASY+FRAGILE designs)

Known issues:

* single-seed gradients — stochastic noise contaminates finite differences
* gradient blow-up near stability boundaries (cliff designs)
* topology reproducibility not yet demonstrated across seeds
* unit bias was present in early runs; fixed by range-scaling gradients

Confidence:

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
