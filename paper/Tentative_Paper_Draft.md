# Quantifying Stability Boundaries for Amateur TVC Rockets

## Working Title
From guess-and-check PID tuning to quantified stability: mapping easy, fragile, and infeasible regimes in amateur thrust-vector-control rockets.

## Mission Statement
Give amateur and student TVC builders a quantitative answer before they fly: is this rocket easy, fragile, or fundamentally infeasible, and which hardware parameters actually matter most for moving it into the workable region?

## Abstract
The hobbyist thrust-vector-control rocket community is growing quickly, but the dominant design workflow is still qualitative: copy someone else's PID gains, fly, crash, retune, repeat. That workflow hides two different failure modes. Some rockets are fundamentally under-actuated for their instability and no PID tuning will save them; others are stabilizable in principle but fragile because actuator-envelope degradation, slew loss, deadband, and nonlinearity are ignored. This project asks three linked questions: where is the boundary between easy, fragile, and infeasible amateur TVC configurations, which hardware and modelling factors most strongly shift that boundary, and can a bench-calibrated workflow recover challenging maneuver cells that amateur practice misses? The contribution is not new control theory. The contribution is a quantitative stability map for amateur TVC rockets, an evidence-based ranking of which builder choices matter most for stability, and a bench-calibrated workflow that translates those findings into preflight verdicts and starting gains. New maneuver evidence uses a commanded pitch program (0 to 20-25 deg and back) across instability and wind sweeps: in hobby-nominal conditions, ordinary best-tuned PID is often sufficient except at the edge; in sloppy-hardware middle-regime cells, measured-plant tuning (PID or PCH depending on regime) materially outperforms naive and clean-sim tuning; and in hard-limit cells, most configurations remain infeasible. Bench and launch validation remain the decisive test of whether these simulation-side claims transfer to real hardware.

---

## 1. Problem and Motivation
A small TVC rocket usually fails for boring reasons before it fails for interesting ones. The textbook failure story is "my controller was not robust enough." The more common real story is "my servo could not move fast enough under load, my airframe was more unstable than I thought, and the gains I copied from another project were tuned for a different envelope." That cluster of mistakes is invisible until ignition, and the hobby community often collapses all of them into the single label of "bad PID tuning."

That is the exigence for this project. Amateur TVC rocketry is growing quickly, but it is still unusually hand-wavy for such a safety-sensitive control problem. Builders need to know what actually determines stability so they can spend money and engineering effort on the things that matter: mechanical advantage, loaded slew rate, travel, linkage stiffness, and only then fine controller tuning.

The central question of this project is practical, but it is no longer merely a tool question:

*Where is the boundary between impossible, fragile, and recoverable amateur TVC rockets, can that boundary be measured well enough to guide hardware design choices, and can a measured-workflow controller selection recover maneuvering cells that amateur tuning misses without over-claiming impossible cells?*

This reframes the project away from a generic adaptive-control study and away from a pure tool paper. The stronger scientific question is whether the community is using the wrong mental model of failure, and whether that mistaken model can be replaced with a quantitative regime map plus a deployable low-compute mitigation where mitigation is actually possible.

---

## 2. Gap and Positioning
### 2.1 What is already known
1. Robust, adaptive, and fault-tolerant control are mature research areas.
2. Launch vehicles and aircraft already use extensive envelope protection, scheduling, and conservative validation.
3. Sim-to-real mismatch is a standard problem in controls and robotics.

### 2.2 What is missing in this niche
1. Amateur TVC rockets do not have a quantitative map separating hardware-impossible configurations from hardware-feasible-but-fragile ones.
2. Public hobby projects usually publish controller gains, not measurement-grounded reasons those gains should transfer to another rocket.
3. Bench-measured slew loss, endpoint compression, deadband, and backlash are rarely carried all the way into either a flight-readiness decision or a controller-design decision.
4. Small-rocket sim-to-real work often treats "realism" as a vague collection of noise terms instead of asking which factors actually dominate the stability boundary.
5. The niche lacks a lightweight, hobby-deployable bridge between ordinary PID practice and more constraint-aware control ideas.

### 2.3 Positioning of this project
This project is best positioned as an empirical cartography paper with a lightweight implementation branch:
1. Quantify the stability boundary in the hobby-relevant space spanned by instability demand, actuator slew, and available travel.
2. Identify which aspects of the real actuator and linkage dominate that boundary.
3. Use those findings to tell builders what to prioritize in hardware design and what can be left secondary.
4. Test whether a very lightweight actuator-aware PID hardening layer can recover part of the fragile middle regime below the hard hardware wall.
5. Validate the map and the controller recommendation against held-out bench and launch evidence.

The preflight validator remains important, but it is now the translation mechanism from the research findings to builder practice, not the main intellectual claim by itself.

---

## 3. Core Claims
### Claim A: Amateur TVC stability has three practically distinct regimes
The amateur TVC design space is not well described by a single "stable/unstable" label. It is better described by at least three regimes:
1. Easy: ordinary fixed-gain control works.
2. Fragile but recoverable: naive PID often fails, but the hardware is not fundamentally impossible.
3. Fundamentally infeasible: no realistic fixed-gain tuning rescues the configuration.

### Claim B: Actuator-envelope reality dominates the boundary more than hobby practice assumes
Loaded slew rate, available travel, mechanical advantage, and actuator nonlinearity shift the boundary far more than the community's usual tuning-centric narrative suggests. This is why builders should often prioritize slew and geometry over small improvements in sensor quality or minute controller tweaks.

### Claim C: Sim-to-real mismatch is regime-dependent, not uniformly distributed
Different operating regimes care about different realism blocks. Near the boundary and in actuator-limited cases, actuator nonlinearity and envelope loss dominate. In easier regimes, those same effects matter much less.

### Claim D: The main recoverable-region story is hardware-aware tuning and classification, not a new controller
In the tested hobby-relevant nominal slew regime, best-tuned ordinary PID already covers the workable cells. Any lightweight actuator-aware PID hardening claim must therefore be explicitly secondary and bounded to severe-collapse appendix cases rather than treated as the central research contribution.

### Claim D2: The workflow advantage is regime selection, not universal controller superiority
The strongest claim is not "PCH beats PID" or "PID beats PCH" globally. The strongest claim is that measured-workflow selection of architecture and gains (PID where sufficient, PCH where constraints dominate, NOGO where infeasible) expands the successful maneuver envelope versus naive nominal-gain and clean-sim tuning baselines.

### Claim E: Bench-calibrated preflight is the translation path, not the whole thesis
Once the boundary and middle regime are quantified, a bench-calibrated preflight workflow becomes the natural way to apply those findings to real builders: issue a regime verdict, recommend priorities, and suggest starting gains or the lightweight hardening option where appropriate.

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
5. Effective linkage geometry or mechanical-advantage consequences where measurable.

### 4.2 Flight-model layer
The realistic simulator already includes the main uncertainty channels needed for this project:
1. Sensor noise, bias, and quantization.
2. Sensor latency.
3. Servo deadband and backlash.
4. Gust disturbance.
5. Optional effectiveness drift.

Bench measurements are converted into this model instead of leaving the actuator as a nominal placeholder.

### 4.3 Regime-mapping layer
The main analysis path is now a regime map, not just an autotune pipeline:
1. Sweep instability demand, loaded slew, and travel.
2. Identify success and failure boundaries.
3. Separate impossible cells from fragile cells where retuning or controller hardening still helps.
4. Extract builder-facing tradeoffs such as whether additional slew matters more than higher positional accuracy.
5. Add maneuver-tracking sweeps (commanded pitch programs) so "works" means more than hovering near vertical.

### 4.4 Optional controller-hardening appendix
The controller-hardening branch is now an appendix path rather than the main story:
1. Estimate delivered actuator slew online.
2. Detect sustained rate-limited operation.
3. Preserve derivative damping while avoiding the worst command pile-up behavior of naive PID.
4. Keep the compute and firmware footprint close to ordinary hobby PID.

This branch is only retained where it survives explicit severe-collapse stress tests. It is not claimed to be necessary in the current hobby-nominal regime.

### 4.5 Decision layer
The preflight decision path is now:
1. Measure the actuator.
2. Estimate rocket instability demand and control effectiveness.
3. Run the validator to classify the rocket as easy, fragile, or infeasible.
4. Run measured-plant PD autotuning where the hardware is workable.
5. In fragile cells, compare at least one constraint-aware option (e.g., PCH) against measured-plant PID and pick by empirical success, not theory preference.
6. Report GO/MARGINAL/NOGO plus starting gains, regime label, and diagnostic margins.

### 4.6 Sim-to-real interpretation layer
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
6. A regime-aware factor screen showing actuator nonlinearity dominates near the stability boundary.
7. A hobby-range controller sweep showing that best-tuned ordinary PID already stabilizes all tested nominal 45-90 deg/s loaded-slew cells, so controller novelty is not the main claim.
8. A legacy severe-collapse stress probe where a slew-aware PID branch can still be studied as a bounded appendix result.
9. A new pitch-program head-to-head sweep (`run_pitch_program_head_to_head.m`) showing that hobby-nominal maneuver cells are mostly easy except at high-instability edge cases.
10. A new pitch-program stress sweep (`run_pitch_program_stress_sweep.m`) showing workflow-level rescue in sloppy middle-regime cells and persistent infeasibility in hard-limit cells.

### 5.2 What the workspace now centers on
The primary workspace path is no longer "find the fanciest controller."
It is now:
1. Measure actuator reality.
2. Propagate that reality through the simulator.
3. Identify whether the rocket is easy, fragile, or fundamentally infeasible.
4. Recommend hardware priorities and starting gains.
5. Optionally test a lightweight actuator-aware controller patch only in explicit severe-collapse appendix cells.
6. Validate the map and the controller recommendation against held-out hardware cases.

### 5.3 What is still missing
1. Real summer bench datasets across multiple actuator conditions.
2. Held-out hardware and launch validation of the regime map.
3. If the controller appendix is retained, a severe-collapse validation set showing exactly where it helps and where it does not.
4. A minimal-factor sim-to-real correction model backed by real data.
5. Optional secondary adaptive-fault demo only if it adds clear evidence beyond the lighter PID path.

---

## 6. Locked Simulation Findings
### 6.1 Scaling law
The current simulator supports an empirical relation of the form

slew_min(p) = 0.316 * p^1.99

with fitted exponent near 2. This is useful, but it should be framed as an empirical small-rocket boundary law, not new physics.

### 6.2 Regime split
The strongest simulation-side story is not simply that some rockets are impossible. It is that the design space splits into:
1. Cells where ordinary PID is fine.
2. Cells where naive PID fails but the hardware is still recoverable.
3. Cells where no realistic fixed-gain path works and redesign is required.

### 6.3 Validator quality
On held-out simulated calibration cases, the current regime classifier is already strong:
1. GO precision: 91%.
2. NOGO precision: 100%.
3. MARGINAL behaves as an intermediate zone rather than random noise.

### 6.4 Gain recommendation correction
The old conclusion that the validator's built-in LQR recommendation was enough does not survive steel-man comparison. Well-tuned PD beats the current LQR recommendation on the realistic plant in the working cells. That is why the workspace needed a measured-data-to-PD autotune bridge.

### 6.5 Regime-aware realism screen
The new factor screen already supports a cleaner sim-to-real story:
1. In a low-demand regime, single realism blocks cause almost no success loss, while the full realistic stack reduces success from 1.00 to 0.83.
2. In a boundary regime, actuator nonlinearity alone causes the largest drop, reducing success from 1.00 to 0.50, while latency, sensor noise, gust, and modest effectiveness drift each cause little or no isolated drop.
3. In an actuator-limited regime, the system is already marginal even in idealized tuning, and actuator nonlinearity pushes it to failure.

This is the right kind of claim: not "everything matters," but "which factors matter depends strongly on the regime, and actuator nonlinearity dominates where the project actually becomes safety-critical."

### 6.6 Lightweight controller hardening result
The new PID_SLEW_AWARE branch is promising but bounded. In the current realistic slew-degradation probe, it strongly outperforms naive PID across the easy and moderate-instability cells and remains competitive where ordinary PID collapses. However, it is not a universal controller and should not be framed that way. Its honest contribution is a lightweight hardening path for part of the fragile middle regime, not a replacement for the stability map.

### 6.7 Maneuver-envelope result (new)
The new commanded pitch-program campaigns sharpen the central claim:
1. In hobby-nominal actuator conditions (slew=60 code units/s, u_max=12), most cells are easy; however, at p=12 and gust=0.9, nominal amateur PID gives 0% success while bench-tuned PCH gives 100%, demonstrating a real edge-of-envelope rescue cell.
2. In a sloppy-TVC profile (slew=30, u_max=10, deadband=0.15, backlash=0.30), middle-regime recovery appears clearly: for p=6 and gust=0.9, amateur nominal PID and clean-retuned PID both fail (0%), while bench-tuned PID and bench-tuned PCH both reach 100%; for p=8 and gust=0.9, bench-tuned PID reaches 75% while both amateur baselines remain at 0%.
3. In a hard-limit profile (slew=20, u_max=8, deadband=0.20, backlash=0.40), most cells remain infeasible regardless of controller choice, which supports the GO/MARGINAL/NOGO boundary framing and prevents over-claiming.

This is the key correction to keep: the maneuver result supports a workflow that selects architecture by regime, not a single globally best controller.

### 6.8 Discovery-fishing conclusion
Additional MATLAB-only novelty hunting is now a weak use of time. The strongest remaining novelty comes from real measured hardware data, launch validation of the regime split, and an honest demonstration that the lightweight controller path helps where claimed and fails where redesign is required.

---

## 7. Experimental Plan
### 7.1 Primary experimental arc
1. Bench-characterize the servo and linkage under rocket-relevant conditions.
2. Build a measured library of slew, travel, deadband, and repeatability.
3. Estimate or bound airframe instability demand for candidate rockets.
4. Use those measurements to place each hardware configuration in the easy / fragile / infeasible map.
5. Run the validator/autotuner before each major hardware test.
6. Compare predicted verdict and recommended gains against actual test outcomes.
7. In fragile cells, compare naive and clean-sim baselines against measured-workflow-selected control (PID and/or PCH).

### 7.2 Minimum dataset needed for the main claim
1. At least two actuators or actuator states with meaningfully different measured behavior.
2. At least one hardware configuration in each of the three claimed regions: easy, fragile, and infeasible.
3. At least one fragile configuration where measured-workflow selection materially outperforms naive and clean-sim tuning.
4. Repeated tests under matched conditions so the paper can discuss variability.
5. Held-out cases not used to build the gain lookup.

### 7.3 Secondary branch
If summer hardware testing goes well, the secondary branch should stay narrow:
1. Induced actuator degradation or constrained-travel fault.
2. Compare fixed preflight-tuned PD against the lightweight hardening layer.
3. Only compare heavier adaptive methods if they add a clearly different conclusion.
4. Keep this as an extension, not the main thesis.

---

## 8. Summer Plan
### June
1. Finalize the bench procedure and make it repeatable enough that the same servo tested twice gives nearly the same extracted metrics.
2. Collect actuator datasets for unloaded, installed, low-voltage, and warm or repeated-run conditions.
3. Build a small actuator reality-gap table: datasheet slew versus measured loaded slew, plus endpoint utilization and effective linkage geometry.
4. Start estimating rocket-side instability demand for the real hardware configurations you plan to fly.
5. Choose at least one candidate configuration intentionally near the fragile middle regime.

### July
1. Use the measured bench data to populate the new bench_to_autotune workflow.
2. Build the first held-out comparison set: nominal tuning versus measured-data autotuning.
3. Run the regime-aware factor screen against the measured cases to decide which realism blocks must stay in the reduced model.
4. Run the lightweight controller against the fragile cases to determine whether the middle regime is real on hardware.
5. Freeze the minimal preflight input set you will actually ask a builder to supply.

### August
1. Run repeated ground and flight-adjacent tests with the measured-data recommendations.
2. Check whether the predicted regime label and GO/MARGINAL/NOGO verdict match observed behavior.
3. Quantify whether the recommended gains beat copied hobby gains or nominal-assumption gains.
4. In fragile cells, quantify whether the lightweight controller materially improves robustness.
5. Decide whether there is enough evidence to support a secondary adaptive-fault demonstration.

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
1. Regime-classification accuracy on held-out test conditions.
2. Preflight verdict accuracy on held-out test conditions.
3. Improvement in starting-gain quality relative to copied firmware gains and nominal-assumption tuning.
4. Improvement of measured-workflow selected control (PID and/or PCH) over naive and clean-sim baselines in fragile hardware-feasible maneuver cells.
5. Reduction in unstable or clearly poor launches during the hardware campaign.
6. Generalization across multiple actuator conditions and at least one nontrivial rocket regime.
7. Pitch-program tracking metrics (RMS error, peak error, end-point error) in addition to binary stability.

### 9.2 Strong evidence threshold
The project becomes strong if it can show all of the following:
1. The measured actuator differs materially from the assumed actuator.
2. That difference changes the predicted regime or gain recommendation.
3. At least one real hardware configuration falls in a fragile middle regime where naive and clean-sim tuning fail or are marginal but measured-workflow tuning materially improves behavior.
4. At least one real hardware configuration is shown to be fundamentally infeasible despite retuning, supporting the hard boundary claim.
5. The bench-calibrated recommendation matches reality better than the naive one.
6. The regime-aware reduced realism model explains most of the sim-to-real shift without requiring an unwieldy full-physics model.

---

## 10. STS Framing
The strongest STS version of this project is not:
1. "I made another rocket controller."
2. "I compared a few servos."
3. "I found a cool MATLAB curve."
4. "I built a tool for hobbyists."

The strongest STS version is:

I quantitatively mapped when amateur TVC rockets are impossible, fragile, or recoverable; showed that hobby practice often confuses those regimes; identified which builder-facing hardware parameters most strongly control stability; and demonstrated on challenging pitch-program maneuvers that a bench-calibrated workflow can recover middle-regime cells that amateur and clean-sim tuning miss, without pretending to beat the hard physics wall.

That is safety-relevant, technically deep, empirically grounded, and broad enough to interest judges outside the exact hobby niche because it changes how builders should reason about design, tuning, and hardware tradeoffs.

---

## 11. Claim Boundaries and Risk Controls
1. Do not claim new control theory.
2. Do not claim that the boundary itself is philosophically surprising; claim that its location, shape, and practical implications for amateur TVC are newly quantified.
3. Do not claim the lightweight controller is universal.
4. Keep adaptive recovery secondary unless hardware data clearly justifies promoting it.
5. Treat synthetic data as placeholder evidence only until the summer hardware campaign is complete.
6. Publish failure cases, false positives, false negatives, and no-help controller cases alongside successes.

---

## One-Sentence Thesis
This project turns amateur TVC rocketry from guess-and-check tuning into a quantified stability problem by mapping which rockets are impossible, fragile, or recoverable, identifying the hardware parameters that actually control that boundary, and using a bench-calibrated workflow to select viable gains and controller architecture for recoverable maneuvering regimes.

---

## Appendix: New Evidence Artifacts (May 2026)
The following artifacts support the claims in Section 3 and are available under `experiments/results/`:

1. **`s2r_ablation.csv` + `graphs/preflight_workflow.png`** — leave-one-out ablation from a canonical realistic baseline. Shows that, in the actuator-limited regime, removing actuator nonlinearity alone recovers ~17 percentage points of success rate; in the boundary regime, sensor noise and effectiveness drift dominate. This replaces the earlier one-at-a-time factor screen, which had inconsistent profile definitions.
2. **`p_estimation_error.csv`** — sweep of designer-assumed instability vs true instability. At `p_true = 10` with the assumption matching reality, a controller tuned on a basic sim achieves 0% on the realistic plant; a controller tuned on the realistic sim achieves 50%. Several cells show Δ = +0.40 to +0.50 in favor of the bench-calibrated workflow.
3. **`basic_vs_better_sim.csv`** — same plant, two different design simulators. The basic-sim controller picks over-aggressive gains that fail on the realistic plant (LOW_DEMAND 0.75 vs 1.00; BOUNDARY 0.58 vs 0.83). This is the cleanest simulator-side demonstration that the modelling step changes outcomes, not just the gain numbers.
4. **`current_vs_proposed_practice.csv` + `graphs/current_vs_proposed_practice.png`** — audited open-source firmware shipped gains vs validator-recommended gains on the same plant cells. The validator's value in this comparison is gain replacement, not GO/NOGO triage at the firmware's assumed actuator envelope.
5. **`graphs/phase_diagram_reference_vehicles.png`** — the validator phase diagram in physical units (deg/s vs rad/s) with hobbyist-class reference vehicles overlaid so a builder familiar with the propulsive-landing community can locate themselves on the diagram. Reference positions are public-video estimates, not measurements from those projects.
6. **`graphs/preflight_workflow.png`** — one-page workflow diagram suitable for the front page of the paper and the dashboard.
7. **`pid_slew_probe.csv`** — realistic slew-degradation probe sweeping `p ∈ {0,4,6,8}` and four PD gain pairs. Supports the bounded controller claim: the lightweight slew-aware PID strongly improves the moderate-instability regime and remains a clearly bounded, non-universal method rather than a new general controller.
8. **`pitch_program_summary.csv` + `pitch_program_head_to_head.csv` + `graphs/pitch_program_success_heatmap.png`** — first commanded-maneuver campaign (0 to 20 deg to 0) showing hobby-nominal maneuver cells are mostly easy, which constrains over-claims.
9. **`pitch_program_stress_summary.csv` + `pitch_program_stress_trials.csv` + `graphs/pitch_program_stress_bars.png` + `graphs/pitch_program_workflow_delta.png`** — stress campaign across hobby-nominal, sloppy-TVC, and hard-limit profiles. Shows workflow-level rescue in fragile middle cells and persistent infeasibility in hard-limit cells.
