# A Bench-Calibrated Preflight Tool for Amateur TVC Rockets

## Working Title
Replacing "tune until it flies" with measure, verify, tune: a free, open-source preflight workflow that tells small-rocket TVC builders whether their rocket is likely to stabilize before they launch it, and what gains to start with.

## Mission Statement
Give amateur and student TVC builders a quantitative preflight check and bench-calibrated starting gains, so deciding whether to fly stops being a guess.

## Abstract
The hobbyist thrust-vector-control rocket community is growing quickly, driven by propulsive-landing demonstrations and educational kits. The dominant tuning practice in that community is still essentially "copy a published PID, fly, crash, retune, repeat." This is unsafe and unscientific. The actuator that flies is almost always slower, softer, and more nonlinear than the actuator the designer assumed, and the consequences of that gap are largest in exactly the configurations builders most want to fly: statically unstable airframes with limited servo authority. This project builds and validates a preflight workflow that bench-measures the real actuator, propagates that measurement into a realistic flight model, classifies the rocket as GO / MARGINAL / NOGO, and recommends starting PD gains. The contribution is not a new control law; it is a measurement-to-decision pipeline that does not currently exist in the amateur TVC space, plus an empirical scaling law and a regime-specific sim-to-real ablation that tell builders which parts of the reality gap actually matter where.

---

## 1. Problem and Motivation
A small TVC rocket fails for boring reasons before it fails for interesting reasons. The textbook failure is "my controller was not robust enough." The real failure is "the servo I bolted in could not move fast enough under load, my airframe was more unstable than I guessed, and the gains I copied off GitHub were tuned for a different rocket." That cluster of small mistakes is invisible until ignition.

The central question of this project is therefore practical, not theoretical:

*Can a hobbyist measure a few bench-accessible quantities before flight, use them to decide whether to fly and how to tune, and demonstrably reduce the gap between "works in simulation" and "works on the launch pad"?*

This reframes the project away from a generic adaptive-control study and into a tool-building question with a clear pass/fail criterion: does the proposed preflight workflow make better decisions than current hobbyist practice on the same hardware?

---

## 2. Gap and Positioning
### 2.1 What is already known
1. Robust, adaptive, and fault-tolerant control are mature research areas.
2. Launch vehicles and aircraft already use extensive envelope protection, scheduling, and conservative validation.
3. Sim-to-real mismatch is a standard problem in controls and robotics.

### 2.2 What is missing in this niche
1. Amateur TVC rockets rarely have a reproducible preflight validator tied to measured actuator behavior.
2. Public hobby projects usually publish controller gains, not measurement-grounded reasons those gains should transfer to another rocket.
3. Bench-measured slew loss, endpoint compression, deadband, and backlash are rarely carried all the way into a flight-readiness decision.
4. Small-rocket sim-to-real work often treats "realism" as a vague collection of noise terms instead of asking which factors actually matter in which regime.

### 2.3 Positioning of this project
This project is not primarily a new-controller paper. It is a practical sim-to-real paper with a controls core:
1. Measure the real actuator under rocket-relevant conditions.
2. Determine which nonidealities materially shift the boundary.
3. Use that information to issue a preflight stabilizability verdict and recommended starting gains.
4. Validate the tool on held-out hardware configurations.

Adaptive or fault-recovery controllers remain in scope only as secondary demonstrations if real hardware later shows a clear need.

---

## 3. Core Claims
### Claim A: The actuator reality gap changes decisions
The difference between assumed and measured actuator behavior is large enough to change whether a given rocket should be considered flyable and how it should be tuned.

### Claim B: A bench-calibrated preflight tool can beat nominal-assumption tuning
A workflow that uses measured actuator behavior should produce better GO/MARGINAL/NOGO decisions and better starting gains than nominal tuning or copied open-source gains.

### Claim C: Sim-to-real mismatch is regime-dependent, not uniformly distributed
Different operating regimes care about different realism blocks. In particular, boundary and actuator-limited regimes are dominated much more by actuator nonlinearity than by small sensor noise terms.

### Claim D: Adaptive recovery is secondary, not primary
If adaptive or fail-aware control is retained, it should be framed as an extension for induced-fault or severe-stress cases after the preflight validator/autotuner is established.

---

## 4. Method Overview
### 4.1 Bench measurement layer
The hardware path measures actuator behavior using analog-feedback servo data and bench scripts already present in the workspace:
1. Firmware/feedback_servo_calibration.ino
2. tools/process_gimbal_bench_test.m
3. tools/process_gimbal_bench_serial.m
4. tools/bench_to_validator.m
5. tools/bench_to_autotune.m

The measured quantities of interest are:
1. Loaded slew rate.
2. Effective travel or endpoint utilization.
3. Deadband and backlash proxies where available.
4. Repeatability across voltage, temperature, mounting condition, and commanded span.

### 4.2 Flight-model layer
The realistic simulator already includes the main uncertainty channels needed for this project:
1. Sensor noise, bias, and quantization.
2. Sensor latency.
3. Servo deadband and backlash.
4. Gust disturbance.
5. Optional effectiveness drift.

Bench measurements are converted into this model instead of leaving the actuator as a nominal placeholder.

### 4.3 Decision layer
The preflight decision path is now:
1. Measure the actuator.
2. Estimate rocket instability demand and control effectiveness.
3. Run the validator to classify the regime.
4. Run measured-plant PD autotuning.
5. Report GO/MARGINAL/NOGO plus starting gains and diagnostic margins.

### 4.4 Sim-to-real interpretation layer
The new regime-aware experiment path identifies which realism blocks actually matter:
1. Ideal tuning on a simplified plant.
2. One-factor-at-a-time perturbation.
3. Combined realistic perturbation.
4. Compare success drop by regime.

This prevents the paper from claiming that every realism term matters equally.

---

## 5. Current Status
### 5.1 Completed simulation-side assets
1. A calibrated validator mapping measured slew and travel into GO/MARGINAL/NOGO.
2. Empirical scaling-law extraction showing required slew rises approximately quadratically with instability demand.
3. Open-source firmware audit showing copied hobby gains do not survive the statically unstable regime.
4. Steel-man PD retuning showing that the current validator classification is stronger than its original LQR gain recommendation.
5. A new bench-to-autotune bridge in tools/bench_to_autotune.m.
6. A new regime-aware factor screen in experiments/run_s2r_factor_screen.m.

### 5.2 What the workspace now centers on
The primary workspace path is no longer "find the fanciest controller."
It is now:
1. Measure actuator reality.
2. Propagate that reality through the simulator.
3. Predict whether the rocket is stabilizable.
4. Recommend starting gains.
5. Validate against held-out hardware cases.

### 5.3 What is still missing
1. Real summer bench datasets across multiple actuator conditions.
2. Held-out hardware validation of the new autotune recommendations.
3. A minimal-factor sim-to-real correction model backed by real data.
4. Optional secondary adaptive-fault demo, if time and hardware capacity remain.

---

## 6. Locked Simulation Findings
### 6.1 Scaling law
The current simulator supports an empirical relation of the form

slew_min(p) = 0.316 * p^1.99

with fitted exponent near 2. This is useful, but it should be framed as an empirical small-rocket boundary law, not new physics.

### 6.2 Validator quality
On held-out simulated calibration cases, the current regime classifier is already strong:
1. GO precision: 91%.
2. NOGO precision: 100%.
3. MARGINAL behaves as an intermediate zone rather than random noise.

### 6.3 Gain recommendation correction
The old conclusion that the validator's built-in LQR recommendation was enough does not survive steel-man comparison. Well-tuned PD beats the current LQR recommendation on the realistic plant in the working cells. That is why the workspace needed a measured-data-to-PD autotune bridge.

### 6.4 Regime-aware realism screen
The new factor screen already supports a cleaner sim-to-real story:
1. In a low-demand regime, single realism blocks cause almost no success loss, while the full realistic stack reduces success from 1.00 to 0.83.
2. In a boundary regime, actuator nonlinearity alone causes the largest drop, reducing success from 1.00 to 0.50, while latency, sensor noise, gust, and modest effectiveness drift each cause little or no isolated drop.
3. In an actuator-limited regime, the system is already marginal even in idealized tuning, and actuator nonlinearity pushes it to failure.

This is the right kind of claim: not "everything matters," but "which factors matter depends strongly on the regime, and actuator nonlinearity dominates where the project actually becomes safety-critical."

### 6.5 Discovery-fishing conclusion
Additional MATLAB-only novelty hunting is now a weak use of time. The simulator appears to have already yielded its main structural findings. The strongest remaining novelty comes from real measured hardware data and from proving a bench-calibrated preflight method works.

---

## 7. Experimental Plan
### 7.1 Primary experimental arc
1. Bench-characterize the servo and linkage under rocket-relevant conditions.
2. Build a measured library of slew, travel, deadband, and repeatability.
3. Estimate or bound airframe instability demand for candidate rockets.
4. Run the validator/autotuner before each major hardware test.
5. Compare predicted verdict and recommended gains against actual test outcomes.

### 7.2 Minimum dataset needed for the main claim
1. At least two actuators or actuator states with meaningfully different measured behavior.
2. At least one rocket configuration near the boundary, not just trivially easy flights.
3. Repeated tests under matched conditions so the paper can discuss variability.
4. Held-out cases not used to build the gain lookup.

### 7.3 Secondary branch
If summer hardware testing goes well, add one secondary branch:
1. Induced actuator degradation or constrained-travel fault.
2. Compare fixed preflight-tuned PD against an adaptive or fail-aware recovery path.
3. Keep this as an extension, not the main thesis.

---

## 8. Summer Plan
### June
1. Finalize the bench procedure and make it repeatable enough that the same servo tested twice gives nearly the same extracted metrics.
2. Collect actuator datasets for unloaded, installed, low-voltage, and warm or repeated-run conditions.
3. Build a small actuator reality-gap table: datasheet slew versus measured loaded slew, plus endpoint utilization.
4. Start estimating rocket-side instability demand for the real hardware configurations you plan to fly.

### July
1. Use the measured bench data to populate the new bench_to_autotune workflow.
2. Build the first held-out comparison set: nominal tuning versus measured-data autotuning.
3. Run the regime-aware factor screen against the measured cases to decide which realism blocks must stay in the reduced model.
4. Freeze the minimal preflight input set you will actually ask a builder to supply.

### August
1. Run repeated ground and flight-adjacent tests with the measured-data recommendations.
2. Check whether the predicted GO/MARGINAL/NOGO verdict matches observed behavior.
3. Quantify whether the recommended gains beat copied hobby gains or nominal-assumption gains.
4. Decide whether there is enough evidence to support a secondary adaptive-fault demonstration.

### September
1. Lock the main claim.
2. Run only the missing confirmation tests, not new idea fishing.
3. Build the final artifact table and figures.

### October to submission
1. Write the paper around the measured workflow and held-out results.
2. Keep the final claim narrow, empirical, and defensible.
3. Include negative results and failure cases directly.

---

## 9. Evaluation Plan and Success Criteria
### 9.1 Primary metrics
1. Preflight verdict accuracy on held-out test conditions.
2. Improvement in starting-gain quality relative to copied firmware gains and nominal-assumption tuning.
3. Reduction in unstable or clearly poor launches during the hardware campaign.
4. Generalization across multiple actuator conditions and at least one nontrivial rocket regime.

### 9.2 Strong evidence threshold
The project becomes strong if it can show all of the following:
1. The measured actuator differs materially from the assumed actuator.
2. That difference changes the predicted regime or gain recommendation.
3. The bench-calibrated recommendation matches reality better than the naive one.
4. The regime-aware reduced realism model explains most of the sim-to-real shift without requiring an unwieldy full-physics model.

---

## 10. STS Framing
The strongest STS version of this project is not:
1. "I made another rocket controller."
2. "I compared a few servos."
3. "I found a cool MATLAB curve."

The strongest STS version is:

I built and validated a preflight method for small TVC rockets that uses measured actuator behavior to decide whether the rocket is likely to stabilize and how it should be tuned, and I showed which parts of the sim-to-real gap actually matter near the stability boundary.

That is practical, safety-relevant, technically deep, and broad enough to interest mentors outside the exact hobby niche.

---

## 11. Claim Boundaries and Risk Controls
1. Do not claim new control theory.
2. Do not claim universal flight safety.
3. Keep adaptive recovery secondary unless hardware data clearly justifies promoting it.
4. Treat synthetic data as placeholder evidence only until the summer hardware campaign is complete.
5. Publish failure cases, false positives, and false negatives alongside successes.

---

## One-Sentence Thesis
This project builds and validates a free, open-source preflight workflow for amateur thrust-vector-control rockets that replaces "tune until it flies" with bench-measured stabilizability prediction and starting-gain recommendation, giving hobbyist builders a quantitative way to decide whether their rocket can safely fly before they light the motor.

---

## Appendix: New Evidence Artifacts (May 2026)
The following artifacts support the claims in Section 3 and are available under `experiments/results/`:

1. **`s2r_ablation.csv` + `graphs/preflight_workflow.png`** — leave-one-out ablation from a canonical realistic baseline. Shows that, in the actuator-limited regime, removing actuator nonlinearity alone recovers ~17 percentage points of success rate; in the boundary regime, sensor noise and effectiveness drift dominate. This replaces the earlier one-at-a-time factor screen, which had inconsistent profile definitions.
2. **`p_estimation_error.csv`** — sweep of designer-assumed instability vs true instability. At `p_true = 10` with the assumption matching reality, a controller tuned on a basic sim achieves 0% on the realistic plant; a controller tuned on the realistic sim achieves 50%. Several cells show Δ = +0.40 to +0.50 in favor of the bench-calibrated workflow.
3. **`basic_vs_better_sim.csv`** — same plant, two different design simulators. The basic-sim controller picks over-aggressive gains that fail on the realistic plant (LOW_DEMAND 0.75 vs 1.00; BOUNDARY 0.58 vs 0.83). This is the cleanest simulator-side demonstration that the modelling step changes outcomes, not just the gain numbers.
4. **`current_vs_proposed_practice.csv` + `graphs/current_vs_proposed_practice.png`** — audited open-source firmware shipped gains vs validator-recommended gains on the same plant cells. The validator's value in this comparison is gain replacement, not GO/NOGO triage at the firmware's assumed actuator envelope.
5. **`graphs/phase_diagram_reference_vehicles.png`** — the validator phase diagram in physical units (deg/s vs rad/s) with hobbyist-class reference vehicles overlaid so a builder familiar with the propulsive-landing community can locate themselves on the diagram. Reference positions are public-video estimates, not measurements from those projects.
6. **`graphs/preflight_workflow.png`** — one-page workflow diagram suitable for the front page of the paper and the dashboard.
