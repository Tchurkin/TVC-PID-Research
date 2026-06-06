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

Current working thesis (updated 2026-06-06):

⚠️ MAJOR REVISION: The slew formula had a π/180 unit error (servos modeled 57.3× too slow).
After fix: INFEASIBLE=0. The Iyy × wind_strength controllability boundary is DEAD — it was
entirely a slew artifact. All prior INFEASIBLE findings, Exp4 fidelity results, and Exp5
slew payoff curves are INVALID and must not be cited.

NEW THESIS: The servo slew rate is the single dominant hardware constraint for hobby TVC.
The minimum required slew is ~40 deg/s for reliable attitude hold and maneuvers up to 15°,
PROVIDED gains are co-optimized with the servo. Below ~15 deg/s, even attitude hold fails.

The MARGINAL regime (3.6% of designs) is entirely a slow-servo cluster:
— 77% of MARGINAL designs have servo_slew < 20 deg/s; median = 12.5 deg/s
— Mechanism: slew limit reached frequently (slew_sat_frac=0.23), authority never saturated
— Optimal gain for slow-servo TVC is LOW (Kp=2); high gains exceed servo bandwidth

The S2R fidelity story has INVERTED:
— Old story: simple model falsely approves INFEASIBLE designs (99.6%)
— New story: simple model picks Kp=2 for all designs; for FRAGILE this is too low →
  15.4% false REJECTIONS; for MARGINAL it's accidentally optimal → 24% better SR

IMPORTANT: p_unstable still has near-zero correlation with regime. The "p_unstable is negligible"
finding survives. The Iyy and wind_strength findings do NOT survive.

The project is studying: servo slew threshold as a function of task complexity,
and gain co-design as the mechanism linking servo hardware to controller performance.

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

⚠️ ALL COUNTS BELOW ARE SUPERSEDED. See thesis update above.
Old counts (pre-slew-fix, invalid): EASY=850, MARGINAL=241, FRAGILE=68, INFEASIBLE=41.

CURRENT counts (post-slew-fix, from exp1_regime_index_py.csv, 2026-06-06):
* EASY     (n=1144, 95.3%): all gain conditions pass, nom_sr ≥ 0.80
* MARGINAL (n=43,   3.6%):  all gain conditions pass, nom_sr = 0.667 — SERVO-SLEW LIMITED
  — 77% have servo_slew < 20 deg/s (median 12.5 deg/s)
  — slew_sat_frac = 0.228, u_cmd_sat_frac ≈ 0 (servo limit, not gimbal limit)
  — optimal gain is Kp=2 (gentle); high gains worsen performance with slow servo
* FRAGILE  (n=13,   1.1%):  fails some gain robustness conditions
  — simple model always picks Kp=2; full physics needs Kp=40-80
  — 15.4% get falsely REJECTED by simple-model gains (too gentle for disturbance rejection)
* INFEASIBLE (n=0,  0.0%): gone — was entirely slew artifact

Key findings (2026-06-06):

FINDING 1: p_unstable still has near-zero correlation with regime (r≈0). CONFIRMED.

FINDING 2 (REVISED): servo_slew_deg_s is the ONLY predictor of regime difficulty (r=-0.189).
  Iyy: r≈-0.045, wind_strength: r≈+0.009, motor_scale: r≈+0.042 — all negligible.
  The Iyy × wind finding was a slew artifact. Do NOT cite it.

FINDING 3 (SURVIVES): Aerodynamic instability does not help maneuverability.
  Stable designs outperform unstable on all tasks. Gap grows with maneuver amplitude.
  At 10° maneuver, stable=86.8% vs unstable=68.8% at slew=120 (18pp gap, does not close).

FINDING 4 (NEW): Servo slew threshold is ~40 deg/s for reliable TVC with co-optimized gains.
  Below 20 deg/s: even attitude hold is unreliable (50-90% success).
  At 40 deg/s: hold and maneuvers to 15° are fully reliable (100%) when gains are co-designed.
  The gain co-design insight: optimal Kp for slow-servo (Kp=2) ≠ optimal for fast-servo (Kp=40-80).

Confidence: HIGH for FINDING 1 (p_unstable negligible), MEDIUM-HIGH for FINDING 4 (slew threshold).
FINDING 2 (servo_slew only predictor): MEDIUM (based on current 1200-design LHS; r=-0.189 is weak).

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

⚠️ ALL EXP4 RESULTS BELOW ARE INVALID — based on pre-slew-fix simulator and INFEASIBLE=235 labels.
Do NOT cite frequency-of-effect rates, false-approval rates, or module dominance rankings.

NEW S2R findings (2026-06-06, from targeted tests on post-fix regime labels):

Simple model ALWAYS selects Kp=2 regardless of design (no wind/noise → gentle gains sufficient).
Full physics needs Kp=5-80 depending on disturbance environment.

| Regime | n tested | Agreement | False Approval | False Rejection | SR(simple-gains) | SR(full-gains) |
|---|---|---|---|---|---|---|
| EASY | 20 | 100% | 0% | 0% | 0.960 | 1.000 |
| MARGINAL | 43 | 100% | 0% | 0% | 0.907 | 0.730 |
| FRAGILE | 13 | 84.6% | 0% | 15.4% | 0.631 | 0.862 |

Key new insight: simple model is NOT dangerous (0% false approvals).
It HELPS MARGINAL designs (accidentally picks the right gentle gains for slow servos).
It HURTS FRAGILE designs (too gentle for disturbance rejection → 15.4% false rejections).

The old "99.6% false approval" story is DEAD (INFEASIBLE=0).
The old "simple model always optimistic" finding is DEAD for FRAGILE.
The old slew bidirectionality finding is a slew-bug artifact.

Priority: Re-run full Exp4C with corrected slew and new regime labels before any submission.

Confidence: LOW for any specific Exp4 percentages (all need re-run)

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

⚠️ ALL EXP5 SLEW PAYOFF RESULTS ARE INVALID — based on pre-slew-fix simulator.
The regime-stratified slew payoff curves (EASY/MARGINAL/FRAGILE/INFEASIBLE) used the buggy
slew formula. With INFEASIBLE=0 and MARGINAL redefined as slow-servo, these curves are void.

Surviving result: Exp5 gradient/topology analysis was already removed from paper figures (2026-06-03).
There are no surviving Exp5 results ready for publication.

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
