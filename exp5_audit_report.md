# Exp5 Audit Report

## Major Findings
- Envelope metric is quantized in 5 deg bins and hard-capped at 30 deg.
- Thresholds are computed only from envelope_cmd_deg, so crossings can be dominated by quantization and plateaus.
- Bottleneck labeling order can classify saturated points as diminishing_returns.
- Some non-monotonic trends are physically plausible instability effects, but several headline thresholds are analysis artifacts.

## 1) Physical Plausibility Validation
- Kp increase reducing envelope in easy/fragile regimes is physically plausible when overshoot and saturation rise sharply.
- Gimbal increase reducing envelope is suspicious for a pure authority sweep; here the sweep also scales control_effectiveness, effectively changing loop gain and confounding interpretation.
- Hard regime remains mostly infeasible (frequent zero-envelope outcomes), consistent with severe control limits in those baseline settings.

### Suspicious Results (Concrete Examples)
- easy_regime max_gimbal_deg=2: envelope=30.0, success_like=0.000, overshoot=0.0%, act_sat=0.002, slew_sat=0.469, label=diminishing_returns
- easy_regime Kp=2: envelope=30.0, success_like=0.000, overshoot=0.0%, act_sat=0.002, slew_sat=0.997, label=diminishing_returns
- fragile_regime max_gimbal_deg=2: envelope=30.0, success_like=0.000, overshoot=0.0%, act_sat=0.002, slew_sat=0.748, label=diminishing_returns
- fragile_regime servo_slew_deg_s=120: envelope=30.0, success_like=0.000, overshoot=8.7%, act_sat=0.312, slew_sat=0.995, label=diminishing_returns
- hard_regime Kp=45: envelope=0.0, success_like=0.000, overshoot=48.3%, act_sat=0.957, slew_sat=1.000, label=authority_limited

## 2) Maneuverability Envelope Metric Audit
- Computation path in Exp5 runner:
  - Evaluate command grid: [5, 10, 15, 20, 25, 30] deg.
  - For each command, pass if all gates hold: success_rate >= 0.50, rms_error_deg <= 15, peak_error_deg <= 35.
  - envelope_cmd_deg = largest passing command; else 0 if none pass.
- What increases envelope: higher command bins crossing all three gates.
- What decreases envelope: instability/saturation that causes RMS or peak-error gate failures at previously passing bins.
- Ceiling behavior: yes, fixed ceiling at 30 deg.
- Instability sensitivity: high; envelope can collapse to zero while other metrics still vary.
- Threshold dominance risk: high, because threshold logic uses envelope only.

## 3) Bottleneck Classifier Audit
- Label assignment order:
  - diminishing_returns if envelope_cmd_ratio >= 0.95 and bounded_window_norm >= 0.95
  - slew_limited if slew_sat_frac >= 0.55 and >= actuator_sat_frac + 0.05
  - authority_limited if actuator_sat_frac >= 0.55
  - controller_limited if sweeping Kp/Kd or overshoot >= 15 or bounded window < 0.90
  - mixed_limited otherwise

### Clearly Incorrect/Questionable Labels
- easy_regime Kp=2: label=diminishing_returns, slew_sat=0.997, act_sat=0.002, env=30.0
- easy_regime max_gimbal_deg=2: label=diminishing_returns, slew_sat=0.469, act_sat=0.002, env=30.0
- easy_regime max_gimbal_deg=4: label=diminishing_returns, slew_sat=0.847, act_sat=0.002, env=30.0
- easy_regime max_gimbal_deg=6: label=diminishing_returns, slew_sat=0.989, act_sat=0.002, env=30.0
- easy_regime max_gimbal_deg=8: label=diminishing_returns, slew_sat=0.995, act_sat=0.002, env=30.0
- fragile_regime max_gimbal_deg=2: label=diminishing_returns, slew_sat=0.748, act_sat=0.002, env=30.0
- fragile_regime servo_slew_deg_s=120: label=diminishing_returns, slew_sat=0.995, act_sat=0.312, env=30.0

### Recommended Classifier Logic
- Check instability and saturation first; only then allow diminishing_returns classification.
- Require low saturation and low slope over at least 2-3 adjacent parameter points for diminishing_returns.
- Add an explicit unstable_or_diverging class for high overshoot/high RMS collapse cases.

## 4) Diagnostic Plots
Per-regime diagnostics (A-H coverage, thresholds, smoothing, raw points):
- ![](experiments/results/audit/graphs/exp5_audit_diagnostics_easy_regime.png)
- ![](experiments/results/audit/graphs/exp5_audit_diagnostics_fragile_regime.png)
- ![](experiments/results/audit/graphs/exp5_audit_diagnostics_hard_regime.png)
Quantization and plateau diagnostic:
- ![](experiments/results/audit/graphs/exp5_audit_envelope_quantization.png)

## 5) Hidden Saturation Effects
### Ceiling/Quantization Summary
| case | parameter | frac_at_ceiling | unique_levels | min_env | max_env | quant_step |
|---|---|---:|---:|---:|---:|---:|
| easy_regime | servo_slew_deg_s | 0.000 | 5 | 0.0 | 25.0 | 5.0 |
| easy_regime | max_gimbal_deg | 0.500 | 3 | 15.0 | 30.0 | 5.0 |
| easy_regime | Kp | 0.111 | 5 | 0.0 | 30.0 | 5.0 |
| easy_regime | Kd | 0.000 | 5 | 0.0 | 25.0 | 5.0 |
| fragile_regime | servo_slew_deg_s | 0.111 | 6 | 0.0 | 30.0 | 5.0 |
| fragile_regime | max_gimbal_deg | 0.125 | 6 | 0.0 | 30.0 | 5.0 |
| fragile_regime | Kp | 0.000 | 3 | 0.0 | 15.0 | 5.0 |
| fragile_regime | Kd | 0.000 | 3 | 0.0 | 15.0 | 5.0 |
| hard_regime | servo_slew_deg_s | 0.000 | 1 | 0.0 | 0.0 | 0.0 |
| hard_regime | max_gimbal_deg | 0.000 | 2 | 0.0 | 5.0 | 5.0 |
| hard_regime | Kp | 0.000 | 1 | 0.0 | 0.0 | 0.0 |
| hard_regime | Kd | 0.000 | 1 | 0.0 | 0.0 | 0.0 |

### Collapsed Envelope with Hidden Metric Variation
| case | parameter | envelope | points | param span | success span | overshoot span | rms span |
|---|---|---:|---:|---|---:|---:|---:|
| easy_regime | servo_slew_deg_s | 0.0 | 4 | 10-30 | 0.000 | 101.1 | 6.4 |
| easy_regime | max_gimbal_deg | 30.0 | 4 | 2-8 | 0.167 | 4.7 | 0.8 |
| easy_regime | Kp | 0.0 | 3 | 30-60 | 0.000 | 42.1 | 25.9 |
| easy_regime | Kp | 25.0 | 2 | 5-8 | 0.000 | 29.2 | 1.3 |
| easy_regime | Kd | 0.0 | 3 | 0-4 | 0.000 | 75.5 | 33.7 |
| fragile_regime | servo_slew_deg_s | 0.0 | 3 | 10-20 | 0.000 | 47.6 | 11.8 |
| fragile_regime | max_gimbal_deg | 0.0 | 2 | 15-18 | 0.000 | 85.7 | 3.5 |
| fragile_regime | Kp | 0.0 | 3 | 30-60 | 0.000 | 44.0 | 20.5 |
| fragile_regime | Kp | 10.0 | 2 | 15-20 | 0.000 | 66.3 | 2.7 |
| fragile_regime | Kd | 0.0 | 3 | 0-4 | 0.000 | 75.7 | 45.3 |
| fragile_regime | Kd | 10.0 | 3 | 6-12 | 0.000 | 58.7 | 29.3 |
| hard_regime | servo_slew_deg_s | 0.0 | 9 | 10-120 | 0.000 | 98.3 | 0.0 |

## 6) True Engineering Conclusions
### PHYSICALLY SUPPORTED
- High Kp can degrade maneuverability envelope by driving overshoot and actuator saturation.
- Kd usually improves envelope up to a plateau in easy/fragile regimes.
- Hard regime is predominantly limited, with low achievable envelope under tested baselines.

### LIKELY ANALYSIS ARTIFACTS
- Early diminishing-return thresholds at lowest tested values for gimbal/Kp in easy/fragile regimes.
- Thresholds reported at extreme sweep endpoints when max envelope is zero (procedural, not physical).
- Bottleneck claims based only on classifier labels without support from overshoot/saturation/RMS trends.

## 7) Recommended Fixes
- Refine envelope search resolution to 1-2 deg, or estimate pass/fail crossing continuously.
- Revise threshold logic to use local slope and robustness, not first crossing only.
- Reorder and harden bottleneck classifier with instability-first rules.
- Decouple authority sweep from control_effectiveness scaling for physically clean gimbal conclusions.
- Add uncertainty bands (seed distribution) rather than only mean metrics.

## Trustworthiness for Paper Use
- Use Exp5 qualitatively for instability and saturation patterns.
- Do not use current diminishing-return thresholds or bottleneck boundary claims as final paper evidence without metric/classifier fixes.

---

# Scientific Interpretation Audit — Exp4 and Exp5 (June 2026)

This section reviews every metric currently produced by Exp4 (`run_exp34_regime_importance.m`) and both Exp5 paths (legacy threshold discovery + new design sensitivity) from a scientific interpretation standpoint. Each metric is assessed on four axes: physical meaning, real-flight observability, defensibility as an STS claim, and attack surface. A minimal recommended metric set for paper figures is provided at the end.

---

## Exp4 — Regime-Conditioned Factor Attribution

Exp4 answers: *which hardware/environment non-idealities matter most for flight success, and does the answer change by regime?*

The ablation method is leave-one-out: tune PD on full-realistic baseline, freeze gains, then remove one non-ideality at a time and re-evaluate.

### Metric-by-Metric Assessment

#### `success_recovery` (ablated_rate − full_rate)

**Physical meaning:** If this non-ideality did not exist in flight, the success rate would change by this amount. Positive means the factor hurts performance. For backlash in the FRAGILE regime, median = +0.333 — meaning roughly one-third of borderline flights fail because of mechanical backlash.

**Flight observability:** Not directly. You cannot turn off backlash mid-flight. You could observe it indirectly with paired hardware tests (stock gimbal vs anti-backlash gimbal, same gains). That paired-hardware test is the natural next step for hardware validation.

**STS claim:** **STRONG.** "Backlash causes a +33 percentage-point drop in success rate in the fragile regime, and a +67 pp drop in the infeasible regime" is a concrete, auditable, regime-specific attribution. It directly supports the claim that mechanical precision matters more than sensor precision for this class of rocket.

**Attack surface:** (1) The ablation is simulation-only; a real flight won't "remove" backlash cleanly. (2) The baseline is a fixed (tuned-for-full-realistic) PD controller; if you re-tune after removing backlash you might get a different improvement. Response: this is the correct framing — it answers "what does backlash cost a builder who has already done their best tuning," which is the practically relevant question.

#### `rms_recovery_deg` (full_rms − ablated_rms)

**Physical meaning:** How much does steady-state tracking error improve when this non-ideality is removed? Positive means the factor increases tracking noise/error.

**Flight observability:** **YES, cleanly.** Attitude RMS is directly readable from an onboard IMU (e.g., MPU6050 at 100 Hz over the flight segment). This is the most flight-testable metric in the entire Exp4 output.

**STS claim:** Strong secondary support. "Backlash adds ~9 deg of RMS tracking error in the fragile regime" translates directly to "servo quality affects how straight this rocket flies."

**Attack surface:** Low, if the simulation dynamics match the real rocket. Main vulnerability: a 3-second simulation window may not represent the full powered flight duration where disturbances compound.

#### `s2r_reduction` (actuator stress-to-range composite)

**Physical meaning:** The `s2r` metric equals `mean(u_sat_frac + slew_sat_frac) / 2` — a composite of how often the actuator hits its position limit vs. its slew-rate limit. `s2r_reduction` is how much this stress drops when a factor is removed.

**Flight observability:** **WEAK.** Position saturation is partially observable (servo stalling, buzzing). Slew saturation requires knowing real-time actuator velocity, which amateur firmware does not log. The composite of the two has no established physical meaning.

**STS claim:** **WEAK.** No published control paper uses `s2r` as a named metric. A reviewer will ask what the formula is and why it is weighted equally. Even if the answer is defensible, the conversation detracts from the main story.

**Recommendation: Drop from paper figures.** Report `u_sat_frac` and `slew_sat_frac` separately if actuator stress is mentioned at all, but neither is the primary claim.

#### `importance_score` and `factor_importance_pct`

**Physical meaning:** A weighted composite score that combines `success_recovery`, `rms_recovery`, and `s2r_reduction` after normalizing each by the baseline value. The percentage is a within-cell normalization of the composite.

**Flight observability:** None. This is a scoring artifact, not a physical quantity.

**STS claim:** **VULNERABLE.** The current data shows backlash at 82–93% of "importance" across all regimes. That sounds authoritative, but the specific percentage changes if the formula weights change. A hostile reviewer will ask: "Why those weights? Did you tune the formula after seeing the results?" If you cannot show that the formula was pre-registered before running the experiment, you will lose that exchange.

**Critical risk:** The backlash-dominates conclusion is fully supported by `success_recovery` alone (rank-1 in 100% of cells regardless of formula). The composite score adds no information and introduces formula-dependence vulnerability.

**Recommendation: Remove from paper figures entirely.** The conclusion is stronger without it — "backlash ranks first in every sampled cell in all three regimes" based on `success_recovery` is harder to attack than any percentage.

#### `full_peak_deg` / `factor_peak_deg` and `full_end_deg` / `factor_end_deg`

**Physical meaning:** Peak tracking error (worst-case instantaneous excursion) and terminal error at end of simulation window.

**Flight observability:** Peak error is IMU-observable. Terminal error is semi-observable but depends on simulation duration choice.

**STS claim:** Secondary. Peak error and RMS error are correlated in this simulation; both being high means the controller is struggling. Including both in a figure does not add distinct information.

**Recommendation:** Keep `peak_err_deg` as one secondary column in the full data table for completeness, but use `rms_recovery_deg` as the primary error metric in paper figures. Drop `end_error_deg` — it adds nothing beyond the success gate.

#### `full_u_sat_frac` / `factor_u_sat_frac` and `full_slew_sat_frac` / `factor_slew_sat_frac`

**Physical meaning:** Fraction of the simulation window when the actuator was hitting its position or slew limit.

**Flight observability:** Slew saturation is not logged in any of the three audited amateur firmwares. Position saturation might manifest as servo stall current draw, but this is not standard instrumentation.

**STS claim:** Useful context for the "authority wall" framing (cells where saturation is high regardless of which factor you ablate suggest a hardware-limited regime). But the individual numbers are not the headline.

**Recommendation:** Retain as diagnostic columns in the data file. Do not plot in paper figures unless making the specific point that an infeasible cell has u_sat near 1.0 regardless of tuning.

#### The `latency` factor — zero importance across all regimes

**Observed result:** `median_success_increase = 0`, `median_rms_recovery_deg = 0` for latency in all three regimes. This means removing the 2-step latency from the simulation produces no measurable improvement.

**Physical meaning concern:** In a 50 Hz simulation with 2-step latency, that is 40 ms. For an unstable rocket with p = 8–10 rad/s, 40 ms corresponds to ~0.3 rad of phase lag at the natural frequency, which should matter. The zero result may reflect that the `latency_n = 2` toggle resets to 0 in the "ideal" profile, but the dynamics at tested p values are dominated by backlash so strongly that latency effects are masked.

**Disclosure requirement:** If Exp4 is cited in the paper, the latency = 0 result must be explicitly disclosed and explained, not silently omitted. The most honest explanation: at the tested stress levels, mechanical backlash is so dominant that the marginal contribution of sensor latency is undetectable in the sim. This framing is honest and actually strengthens the backlash story.

**Risk if omitted:** A reviewer who knows digital control will notice latency is in the factor table with zero importance and will assume either a modeling flaw or selective reporting.

---

## Exp5 (Legacy) — Threshold Discovery

The legacy path (`run_exp5_threshold_discovery_legacy.m`) sweeps each design parameter from a low floor to a high ceiling within a fixed regime and asks: *at what value does the controller first work, work well, and stop improving?*

### Metric-by-Metric Assessment

#### `failure_threshold` — minimum value for any success

**Physical meaning:** The absolute floor — below this parameter value, zero pitch commands in the command grid succeed. For example, "servo slew must be above 45 deg/s in the easy regime before any maneuver is possible."

**Flight observability:** **YES, directly.** Servo slew is measurable with `feedback_servo_calibration.ino` before launch. This is the closest thing to a "spec sheet for your rocket" that the experiment produces.

**STS claim:** **STRONG.** "Below [X] deg/s servo slew, no gain tuning can save a p = [Y] rocket" is a falsifiable, benchable, clean claim. It also directly justifies the validator tool.

**Attack surface:** Low. The threshold is a simulation-derived bound; its value depends on the disturbance/initial condition model. Disclose this explicitly.

#### `minimum_practical_threshold` — value for useful (≥50%) success

**Physical meaning:** The "minimum viable spec" that makes the rocket flyable with the recommended gains.

**Flight observability:** Yes — this is what bench measurement + validator output together produce. The builder measures slew, looks up the threshold, and gets a GO/NOGO verdict.

**STS claim:** **STRONGEST single actionable metric.** The entire validator tool is operationally this threshold: "your measured slew is 68 deg/s; the practical threshold for p = 8 is 65 deg/s; verdict GO."

**Attack surface:** The 50% success gate is arbitrary. Justification: 50% is a standard binary classifier threshold, and the paper's framing is about distinguishing feasible from infeasible hardware, not optimizing performance. Pre-register this threshold choice.

#### `diminishing_return_threshold`

**Physical meaning:** Where further improvement saturates. Beyond this slew rate, adding a faster servo does not help.

**Flight observability:** Yes, indirectly.

**STS claim:** **WEAK.** In the current output, the DR threshold often equals the practical threshold or the maximum swept value, meaning the sweep didn't resolve a distinct plateau. The column adds apparent precision without real information.

**Recommendation:** Drop from paper figures. The practical threshold is the actionable spec; the DR threshold is a secondary planning tool at best.

#### `max_envelope_cmd_deg`

**Physical meaning:** The largest pitch command amplitude (degrees) for which at least 50% of simulation seeds succeed. This is the maneuverability ceiling — "this airframe can execute up to X-degree pitch kicks."

**Flight observability:** YES. You can command a known pitch program in flight and measure whether the attitude error remains bounded. This is the most mission-relevant metric in the Exp5 suite.

**STS claim:** Strong. "A properly tuned p = 6 rocket with 60 deg/s slew can execute 25-degree pitch programs; the same rocket with 30 deg/s slew can only execute 5-degree programs" is a concrete, demonstrable claim.

**Attack surface:** The 5-degree grid resolution makes this metric quantized; differences of one grid step (5 deg) are not statistically distinguishable. Acknowledge this openly. The remedy is to either use a finer grid or report as a range.

#### `envelope_at_practical_cmd_deg` and `envelope_at_dr_cmd_deg`

**Physical meaning:** These re-report the command envelope at two specific parameter thresholds.

**Redundancy analysis:** In almost all rows, `envelope_at_practical_cmd_deg == max_envelope_cmd_deg` because by the time the parameter reaches the practical threshold, the system is operating well. The DR version similarly converges.

**Recommendation:** Remove both from paper figures. They are output-table bookkeeping, not independent findings.

#### `bounded_window_norm_at_practical`

**Physical meaning:** Normalized duration over which the attitude error stays below the 30-degree hard threshold. A value of 1.0 means the system never crashed during the window; 0.5 means it crashed halfway through.

**Flight observability:** Observable from IMU but the 30-degree threshold is not a standard spec — it is the simulation fail gate.

**STS claim:** **WEAK.** This metric mostly tracks with success rate (success = bounded_window_norm ~= 1.0; failure = lower values). It does not add independent information.

**Recommendation:** Drop from paper figures. Its correlation with success_rate means it adds no separate claim.

#### `overshoot_pct_at_practical`

**Physical meaning:** Percentage overshoot of the attitude response at the practical threshold. Standard second-order step-response metric.

**Flight observability:** **YES, cleanly.** Overshoot is directly readable from IMU attitude log. It is also a universally understood control metric that any judge will recognize.

**STS claim:** Strong complement to success rate. "At the minimum practical servo spec, overshoot is [X]%, consistent with a damped closed-loop response" tells the story of how well the controller behaves, not just whether it barely passes.

**Recommendation:** Keep in figures, but only for the passing regime comparisons (easy and fragile). Hard-regime overshoot values are meaningless because the system is in failure mode.

#### `failure_bottleneck` / `practical_bottleneck`

**Physical meaning:** What is the binding constraint — actuator position limit (authority) or actuator slew limit (rate)? These are categorical diagnostic labels.

**Flight observability:** Diagnosable from bench testing. If you reduce Kp until the servo stops stalling, it was authority-limited. If the servo never hits the stop but still oscillates, it is likely slew-limited.

**STS claim:** **STRONG.** "In the fragile regime, failure is slew-limited in 3 of 4 parameter sweeps, meaning buying a higher-torque servo does not help — you need a faster one" is exactly the kind of actionable engineering insight that distinguishes this paper from a generic simulation study.

**Attack surface:** The classifier logic has known issues (documented in the software audit above: priority ordering can incorrectly label high-slew-saturation cases as diminishing_returns). The `practical_bottleneck` label at the practical threshold is more defensible than the `failure_bottleneck` label at the lowest swept value.

**Recommendation:** Use `practical_bottleneck` as the primary reported label. Fix the classifier before citing it in figures.

#### `dr_bottleneck`

Same concerns as `diminishing_return_threshold`. Drop from paper figures.

---

## Exp5 (New) — Design Sensitivity

The new path (`run_exp5_design_sensitivity.m`) computes local (±10%) gradients of maneuverability with respect to four parameters (`max_gimbal_deg`, `servo_slew_deg_s`, `Kp`, `Kd`) at representative cells from each regime.

### Metric-by-Metric Assessment

#### `d_rms_err_deg` (gradient of RMS tracking error)

**Physical meaning:** A 10% increase in this parameter changes RMS attitude error by this many degrees. Negative means the parameter helps (increasing it reduces error).

**Flight observability:** **YES.** RMS tracking error is IMU-measurable. The gradient is the most directly hardware-actionable of the sensitivity metrics.

**STS claim:** Strong. "Increasing servo slew by 10% from baseline reduces RMS tracking error by 1.2 degrees in the fragile regime" is a concise, reportable engineering result.

**Attack surface:** Local linear gradients are evaluated at one nominal operating point. The relationship may be nonlinear away from that point (especially near regime transitions). Disclose that these are first-order local estimates.

#### `d_envelope_cmd_deg` (gradient of command envelope)

**Physical meaning:** How much the maneuverability ceiling shifts per 10% parameter change.

**Flight observability:** Indirect — envelope is not directly logged; you'd need a planned maneuver campaign.

**STS claim:** Moderate. Useful to show which parameter controls maneuverability most, but the 5-degree quantization of `envelope_cmd_deg` makes gradients noisy.

**Attack surface:** Quantization noise dominates the gradient when the underlying metric moves by less than one 5-degree grid step. A gradient of ±5 could mean the envelope genuinely shifted or that the parameter crossed a 5-degree quantization boundary.

**Recommendation:** Reduce the grid resolution to 2–3 degrees before trusting these gradients, or replace with the continuous `success_like_rate` gradient. As-is, use only for directional claims (sign), not magnitude.

#### `d_success_like_rate` (gradient of pass fraction)

**Physical meaning:** How much the fraction of passing command amplitudes changes per 10% parameter change.

**Flight observability:** Indirect — requires a programmed maneuver campaign.

**STS claim:** Moderate. The pass-fraction gradient is less vulnerable to quantization than the envelope gradient because it averages over six command levels.

**Terminology vulnerability:** "Success-like rate" is not a standard term. It means "fraction of tested command amplitudes for which mean success ≥ 50%." Rename to **pass fraction** or **maneuverability pass rate** in any paper use.

#### `gradient_direction` (categorical label)

**Physical meaning:** A classification of the gradient vector into categories like `improve_maneuverability`, `improve_stability`, `tradeoff_positive`, `stability_cost`, `neutral`.

**STS claim:** **TOO FRAGILE.** The classification logic requires all three of `d_success`, `d_env`, and `d_rms` to point in specific directions simultaneously. Slight noise in any one component flips the label. The same underlying physics could produce "improve_maneuverability" in one seed and "tradeoff_positive" in another.

**Recommendation:** Drop from paper figures. Report the underlying numeric gradients directly. Readers can determine the direction from the numbers; the categorical label adds interpretation without transparency.

#### `base_envelope_cmd_deg`, `base_success_like_rate`, `base_rms_err_deg`

**Physical meaning:** Baseline (unperturbed) values at the representative cell. These are the starting point for the sensitivity analysis.

**STS claim:** These should be reported as context in any sensitivity figure (the "where you start" before showing the gradient). They are not standalone claims.

---

## Pitch Program Stress Sweep (`pitch_program_stress_summary.csv`)

This dataset compares AMATEUR_NOMINAL_PID, CLEAN_RETUNED_PID, BENCH_TUNED_PID, and BENCH_TUNED_PCH across p, gust, and hardware profiles (HOBBY_NOMINAL, SLOPPY_TVC, HARD_LIMIT).

This is currently the **most publication-ready dataset in the project.** Every metric in it has a direct physical meaning and clear experimental structure.

#### `deploy_success_rate` vs `design_success_rate`

**Physical meaning:** `design_success_rate` = success rate in the ideal simulation used for tuning. `deploy_success_rate` = success rate in the full-realistic simulation with the same gains. The gap is the sim-to-real penalty.

**STS claim:** **STRONGEST.** "The amateur firmware achieves 100% success in its design environment and 100% in the nominal real environment; it drops to 0% when the TVC hardware degrades by SLOPPY_TVC conditions at p ≥ 8." This is directly analogous to how real rocket failures occur.

#### `rms_error_deg`, `peak_error_deg`

**Physical meaning:** Standard tracking quality metrics. IMU-observable.

**STS claim:** Strong secondary support. "The bench-tuned controller reduces RMS by X% vs the amateur controller at the same hardware conditions."

#### `u_cmd_sat_frac`, `slew_sat_frac`

Same analysis as Exp4. Contextual diagnostic, not paper figure material.

---

## Critical Disclosure Requirements

These findings must be explicitly stated in the paper or supplementary material, or they become vulnerabilities:

1. **Latency = 0 importance.** Every reviewer with digital-control background will notice this. The honest explanation strengthens the backlash claim; the omission of an explanation weakens the whole table.

2. **Simulation-only baseline.** All success rates come from simulation. The paper should state explicitly that real-flight validation (the Firmware/feedback_servo_calibration.ino bench test) has been conducted for actuator characterization, but closed-loop flight data has not been collected. The sim-to-real connection is through measured bench data feeding into the simulator, not through flight telemetry.

3. **`importance_pct` formula dependence.** If any reviewer asks you to derive the importance percentage analytically from the individual metrics, the answer should be transparent. If you cannot explain the formula in one sentence without referring to code, don't put the percentage in a figure.

4. **Envelope quantization.** The 5-degree grid for `envelope_cmd_deg` means all claims about command envelope threshold differences smaller than 5 degrees are not statistically supported. This affects both the threshold discovery and sensitivity gradient claims.

5. **PD re-tuning was done on full-realistic sim.** The ablation result (backlash matters most) is conditional on the PD being tuned for full realism. A controller tuned for ideal sim would show a different pattern. This framing is correct for the paper's argument but must be stated.

---

## Recommended Minimal Metric Set for Strongest Paper Figures

### Exp4 figure (regime × factor attribution heatmap)

| Metric | Role |
|---|---|
| `success_recovery` | Primary — headline importance claim |
| `rms_recovery_deg` | Secondary — observable quality correlation |
| `frac_cells_success_positive` | Robustness — shows effect is consistent, not one cell |

**Drop:** `importance_score`, `factor_importance_pct`, `s2r_reduction`, `full_peak_deg`, `full_end_deg`, `u_sat_frac`, `slew_sat_frac`

The Exp4 paper figure should be a 3-regime × 7-factor heatmap of `success_recovery` with `rms_recovery_deg` as annotation. The backlash dominance pattern reads immediately without any composite score.

### Legacy Exp5 figure (design threshold spec sheet)

| Metric | Role |
|---|---|
| `minimum_practical_threshold` | Primary — the actionable builder spec |
| `practical_bottleneck` | Secondary — "buy faster or bigger servo?" |
| `overshoot_pct_at_practical` | Tertiary — response quality at minimum viable spec |

**Drop:** `diminishing_return_threshold`, `failure_threshold` (show only in supplementary), `bounded_window_norm`, `envelope_at_practical_cmd_deg`, `envelope_at_dr_cmd_deg`, `slew_sat_frac_at_practical`, `actuator_sat_frac_at_practical`

The Exp5 legacy figure should be a table: rows are regimes, columns are parameters, cells contain the minimum practical threshold value with a colored bottleneck indicator.

### New Exp5 design sensitivity figure

| Metric | Role |
|---|---|
| `d_rms_err_deg` | Primary — most observable gradient |
| sign of `d_success_like_rate` | Direction indicator only |

**Drop:** `gradient_direction` category labels, `d_envelope_cmd_deg` (quantization noise), rename `success_like_rate` → `pass_fraction` before any paper use

The sensitivity figure should be a 3-regime × 4-parameter bar chart of `d_rms_err_deg`, signed positive = improvement. This is the most defensible single-number gradient metric.

### Pitch stress sweep figure (the strongest standalone figure)

| Metric | Role |
|---|---|
| `deploy_success_rate` | Primary outcome — did the controller work in realistic hardware? |
| `rms_error_deg` | Tracking quality |
| Method × p_unstable × profile grouping | The experimental structure |

This should be the lead figure in any STS presentation of the paper. It is the only figure that directly compares the proposed workflow (bench-tuned) against status quo (amateur nominal) with observable metrics on a realistic hardware model.

---

## Summary Vulnerability Table

| Metric | Keep? | Risk Level | Reason |
|---|---|---|---|
| `success_recovery` | YES | LOW | Physically interpretable, method is standard ablation |
| `rms_recovery_deg` | YES | LOW | Standard metric, IMU-observable |
| `frac_cells_success_positive` | YES | LOW | Robustness check, transparent |
| `importance_score` | NO | HIGH | Arbitrary composite, formula-dependent percentage |
| `factor_importance_pct` | NO | HIGH | Same; the % number will be questioned |
| `s2r_reduction` | NO | MEDIUM | Non-standard metric, redundant with individual sat fracs |
| `bounded_window_norm` | NO | MEDIUM | Mostly tracks success_rate, no independent claim |
| `diminishing_return_threshold` | NO | MEDIUM | Often equals practical threshold; adds noise |
| `gradient_direction` labels | NO | HIGH | Categorical from noisy tri-variate rule; flips easily |
| `d_envelope_cmd_deg` | CAUTION | MEDIUM | Quantization noise at 5-deg grid; use sign only |
| `d_rms_err_deg` | YES | LOW | Observable gradient, standard metric |
| `minimum_practical_threshold` | YES | LOW | Core builder-facing spec |
| `practical_bottleneck` | YES | MEDIUM | Fix classifier first; then strong claim |
| `overshoot_pct_at_practical` | YES | LOW | Universal metric, IMU-observable |
| `deploy_success_rate` | YES | LOW | Cleanest outcome metric |
| `latency=0` result | DISCLOSE | HIGH | Omission is a disclosure risk |