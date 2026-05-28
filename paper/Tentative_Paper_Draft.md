# Fail-Aware, Safety-Bounded TVC for Small Rockets

## Working Title
A hardware-validated control framework that identifies common TVC degradation modes and adapts safely within measured operating boundaries.

## Mission Statement
Make small-scale TVC rockets safer and more robust by detecting and adapting to the most common control degradation modes in real time.

## Abstract
Large launch-vehicle guidance and control methods handle actuator limits, loss of control effectiveness, and safety constraints, but operational systems are usually conservative, tightly certified, and supported by expensive validation infrastructure. Publicly documented large-rocket practice appears to rely far more on robust design, scheduling, and envelope protection than on openly described online adaptive primary ascent control. Small model rockets face the same classes of failures with tighter hardware limits, noisier sensors, and stronger sim-to-real mismatch. This project translates failure-aware control ideas to small-scale TVC by combining online estimation of control effectiveness and actuator rate-envelope health, with safety-bounded adaptation and explicit failure-boundary mapping. The contribution is not a claim of inventing adaptive control theory; it is a reproducible, hardware-grounded framework that quantifies where uncertainty materially reduces margin, where adaptation helps, where it fails, and why.

---

## 1. Problem and Motivation
Small TVC rockets commonly experience control degradation from one or more of the following:
1. Reduced effective control authority (thrust or geometry effectiveness drift).
2. Actuator rate/position limitations under load, thermal effects, voltage sag, or wear.
3. Sensor and structural artifacts that contaminate estimator signals.

Conventional fixed-gain controllers can perform well in nominal conditions but lose margin under these degradations. The research question is therefore:

Can a small-rocket TVC controller detect degradation mode online and adapt safely without leaving a verified operating envelope?

---

## 2. Prior Methods and Gap
### 2.1 What is already known
1. Fault-tolerant and adaptive control for launch vehicles and aircraft is well established.
2. Saturation handling (anti-windup, hedging, scheduling, constrained commands) is standard practice.
3. Robust and adaptive methods are widely studied in theory and high-end systems.
4. Publicly documented operational launch vehicles appear to use conservative, certifiable control architectures more often than openly described online adaptive primary flight control.

### 2.2 What is missing at small scale
1. Few student-scale systems provide hardware-validated dual-fault identification with clear arbitration.
2. Most small-rocket demonstrations emphasize nominal tracking, not quantified failure boundaries.
3. Sim-to-real trust is often weak because sensing and actuation limits are under-modeled.
4. Measured actuator degradation and control-effectiveness uncertainty are rarely propagated all the way through baseline comparison, safety analysis, and flight evidence.

### 2.3 Why small TVC is the right niche
1. Small rockets have larger relative uncertainty, weaker actuators, noisier sensing, and thinner validation margins than large launch systems.
2. They are a practical place to test uncertainty-aware control because experiments can be repeated at low cost and with direct hardware access.
3. This does not try to replace certified large-launch control practice; it develops a rigorous small-scale testbed for questions that are expensive or risky to study at full scale.
4. The niche matters because improved handling of non-ideal actuation and authority variation can directly improve safety and mission success in amateur and educational TVC.

### 2.4 Positioning of this work
This project contributes a small-scale, reproducible translation of fail-aware control principles with explicit boundary evidence, rather than a claim of first-in-field control theory invention.

---

## 3. Contribution Claims
This draft is centered on three claims.

### Claim A: Dual degradation awareness
The controller distinguishes two practically important degradation classes:
1. Control-effectiveness loss (authority reduction).
2. Actuator rate-envelope degradation (slew-limited behavior).

### Claim B: Safety-bounded adaptation
Adaptation is gated and shaped by safety logic so that performance gains do not come at the cost of unstable or unsafe transients.

### Claim C: Boundary-level evidence
Performance is evaluated as a map over disturbance and degradation conditions, not just isolated point demos.

### Claim D: Measured uncertainty matters
Real measured slew degradation and bounded control-effectiveness variation are large enough to change the stable or acceptable operating region of fixed controllers.

---

## 4. Method Overview
### 4.1 Control architecture
1. Nominal state-feedback baseline for attitude/rate stabilization.
2. Online effectiveness estimation (authority channel).
3. Online actuator-envelope monitoring (rate channel).
4. Arbitration logic when one degradation mode can contaminate inference of the other.
5. Safety shields (command rate limiting, attitude guardrails, abort paths).

### 4.2 Sensing strategy
1. Primary state sensing from IMU and actuator feedback.
2. Hardware path now includes analog-feedback servos for direct actuator-state measurement.
3. Sensor quality and structural effects are treated as explicit uncertainty sources, not ignored assumptions.

### 4.3 Validation strategy
1. Baseline comparisons under identical scenarios.
2. Ablation tests to isolate mechanism value.
3. Monte Carlo/stress sweeps for robustness.
4. Hardware-in-the-loop and bench campaigns before flight claims.
5. Explicit injection of measured actuator degradation and bounded control-effectiveness uncertainty into simulation before launch conclusions are made.

---

## 5. Current Status (May 2026)
### Completed
1. Core simulation framework with baseline controllers and stress sweeps.
2. Initial joint adaptive logic and safety-path firmware integration.
3. Early evidence that degradation-aware adaptation can reduce post-fault error in relevant regimes.
4. Bench diagnostics for direct actuator feedback, endpoint utilization, and slew-envelope characterization.

### In progress
1. Transition from fragile magnetic angle path to more robust direct actuator feedback path.
2. Hardware calibration and uncertainty characterization for new feedback servos.
3. Updated realism campaign to ensure conclusions survive real sensing/actuation constraints.

### Next critical milestone
Complete hardware-backed degradation experiments with repeatability and uncertainty bars.

---

## 6. Experimental Path
The experimental path is intentionally staged.

### 6.1 Stage 1: Measure uncertainty at hardware level
1. Quantify actuator slew degradation and endpoint loss under unloaded, installed, static-fire, and reduced-voltage conditions.
2. Bound plausible control-effectiveness variation using motor thrust tolerance, geometry, mass-property uncertainty, and sensitivity analysis.

### 6.2 Stage 2: Map control consequences
1. Inject measured slew degradation into simulation and compare fixed baselines against slew-aware control.
2. Inject bounded control-effectiveness variation into simulation and compare fixed baselines against effectiveness-aware control.
3. Evaluate the joint case when both uncertainty classes are present.

### 6.3 Stage 3: Validate on hardware
1. Bench and static-fire validation of the actuator degradation story.
2. Flight tests only after hardware-backed envelopes and safety thresholds are established.

---

## 7. Evaluation Plan and Success Criteria
### 7.1 Primary metrics
1. Post-fault RMS attitude error.
2. Peak excursion and recovery time.
3. Pass/fail fraction under defined safety thresholds.
4. False-alarm and missed-detection rates for degradation arbitration.
5. Robustness under disturbance/noise/model mismatch sweeps.
6. Stable operating-region size under measured uncertainty.

### 7.2 Baselines
1. Fixed PID or fixed state-feedback baseline.
2. Slew-unaware baseline under measured actuator degradation.
3. Effectiveness-unaware baseline under bounded control-effectiveness variation.
4. Single-channel adaptive baselines.
5. Joint-aware controller with safety gating.

### 7.3 Minimum evidence for strong claim
1. Improvement over at least two strong baselines under matched conditions.
2. Ablation showing which module causes which gain.
3. Failure-boundary map showing where the method is valid and where it is not.
4. Hardware repeatability across seeds/runs with explicit uncertainty.
5. Evidence that measured non-idealities materially shrink baseline-safe regimes.

---

## 8. Real-World Relevance
If validated, the framework enables:
1. Safer low-cost TVC for student and educational launch platforms.
2. Better preflight decision-making via quantified control envelopes.
3. Transferable methodology for constrained embedded control systems beyond rockets.

The near-term impact is practical and methodological: making uncertainty-aware, fail-aware control deployable and trustworthy on resource-limited hardware where fixed tuning is often brittle.

### 8.1 Potential users and practical adoption
The likely adoption path is a focused technical market rather than mass consumer scale:
1. Amateur and educational TVC builders that currently rely on fixed-gain PID tuning.
2. University rocketry and controls labs that need reproducible uncertainty-testing workflows.
3. Open-source flight-control communities seeking safer actuator- and envelope-aware tuning methods.
4. Related embedded actuator-control systems where cheap sensing and non-ideal actuation create similar uncertainty.

The practical product of this research is not only a controller, but a validated workflow: measure uncertainty, propagate it through simulation, compare baselines, and publish safe/unsafe operating boundaries.

---

## 9. Limitations and Risk Controls
### Known limitations
1. Small-scale mechanical and sensing artifacts can masquerade as control faults.
2. Identifiability weakens in low-excitation regimes.
3. Extreme disturbances can force conservative behavior and reduce nominal performance.
4. Large-launch operational control architectures are partly proprietary, so positioning must rely on public evidence rather than speculation.

### Risk controls
1. Keep claim scope bounded to tested envelopes.
2. Use confidence-gated adaptation and explicit abort/safety logic.
3. Publish failure modes and non-working regimes alongside wins.

---

## 10. STS Framing
The strongest STS framing is:
1. Not "new controller buzzword."
2. A rigorous, hardware-validated answer to a safety-critical question:
   Can small rockets detect and respond to common control degradation before instability, and does that materially expand the usable safe envelope relative to fixed controllers?

Finalist competitiveness will depend on evidence depth and clarity, especially:
1. Controlled baseline comparisons.
2. Reproducible artifacts.
3. Honest boundaries and limitations.
4. A clearly staged experimental story from measured uncertainty to control consequence to launch evidence.

---

## 11. Why Joint-Aware Control Is Necessary
Using only one adaptive channel can be misleading in coupled uncertainty regimes.

### 11.1 Why keff-only can fail under slew degradation
1. When actuators are rate-limited, commanded motion is not realized.
2. A keff-only estimator can misinterpret rate-limited behavior as authority loss.
3. This can drive incorrect adaptation or inflated command demand while the actuator is already constrained.

### 11.2 Why slew-only can fail under authority variation
1. True control-effectiveness loss can look like sluggish response even with healthy actuator rate.
2. Slew-only logic may preserve actuator assumptions while missing real authority drift.
3. This yields biased control action and reduced robustness under thrust or geometry variation.

### 11.3 Joint-aware hypothesis to be tested
1. Joint-aware arbitration should outperform one-sided adaptation specifically in combined uncertainty regimes.
2. The gain must be demonstrated through boundary maps, not isolated demos.
3. If joint-aware logic does not materially expand safe envelope over single-channel baselines, complexity is not justified and claims must be reduced.

---

## 12. Immediate Work Plan (June to November 2026)
1. June: finalize feedback-servo integration and build unloaded, installed, static-fire, and reduced-voltage actuator degradation datasets.
2. July: propagate measured slew degradation through baseline and adaptive simulation campaigns.
3. August: bound control-effectiveness variation and run effectiveness-aware versus fixed-baseline comparisons.
4. September: complete joint uncertainty maps, hardware validation lock, and claim freeze.
5. October: report writing, artifact packaging, and application assembly.
6. Early November: submit before deadline with all recommendations and artifacts complete.

---

## One-Sentence Thesis
This project translates fail-aware launch-vehicle control ideas to small rockets by measuring real actuator and authority uncertainty, mapping how those uncertainties shrink fixed-controller margins, and testing whether safety-bounded adaptation can recover usable operating envelope.
