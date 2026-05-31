# STS Claim-to-Artifact Matrix

This file maps each major research claim to the exact evidence required to defend it in STS judging.

## Claim 1
**Statement:** Actuator-envelope degradation is the dominant practical failure mode in this small-rocket TVC stack, and actuator-envelope-aware control is the primary robust mitigation.

**Required artifacts:**
1. Matched sweeps showing performance under isolated slew degradation vs isolated authority-loss variation.
2. Controller comparison table showing where fixed, slew-aware, and effectiveness-aware paths each win.
3. Boundary statement identifying where claims hold and where they do not.
4. Run and archive authority-loss comparison script outputs for final lock.

**Primary files:**
- `ModelRocket_Adaptive_TVC/outputs/data/robustness_sweep.csv`
- `ModelRocket_Adaptive_TVC/outputs/graphs/robustness_sweep.png`
- `ModelRocket_Adaptive_TVC/tools/quick_authority_sweep_compare.m`

**Pass criteria:**
1. Slew-aware controller outperforms fixed baseline in the measured slew-degradation envelope.
2. Effectiveness-aware gains, if any, are reported as conditional (stress-regime only) unless they are broad and repeatable.

---

## Claim 2
**Statement:** Safety-bounded adaptation improves post-fault control while respecting defined safety limits.

**Required artifacts:**
1. Post-fault RMS and peak comparisons versus fixed and adaptive baselines.
2. Safety-threshold pass/fail fraction across disturbance bins.
3. Telemetry showing guardrail activation and bounded behavior.

**Primary files:**
- `ModelRocket_Adaptive_TVC/outputs/data/robustness_sweep.csv`
- `ModelRocket_Adaptive_TVC/outputs/data/joint_safety_cert_demo.csv`
- `ModelRocket_Adaptive_TVC/outputs/graphs/robustness_sweep.png`
- `ModelRocket_Adaptive_TVC/outputs/graphs/joint_safety_cert_demo.png`

**Pass criteria:**
1. Beat at least 2 strong baselines in post-fault RMS at target fault levels.
2. Safety pass fraction >= 0.8 in declared operating envelope.

---

## Claim 3
**Statement:** The method is robust to realistic sensing and actuator imperfections on small hardware.

**Required artifacts:**
1. Realism Monte Carlo with sensor noise, quantization, bias drift, and actuator nonlinearities.
2. Sensitivity plots vs noise and disturbance amplitudes.
3. Hardware bench calibration report with repeatability.

**Primary files:**
- `ModelRocket_Adaptive_TVC/outputs/data/realism_montecarlo.csv`
- `ModelRocket_Adaptive_TVC/outputs/graphs/realism_headline.png`
- `ModelRocket_Adaptive_TVC/outputs/graphs/realism_sensitivity.png`
- `data/bench/gimbal_bench_test_template.csv`

**Pass criteria:**
1. Maintain bounded control under representative noise/disturbance set.
2. No catastrophic divergence in declared safe envelope.

---

## Claim 4
**Statement:** Results are reproducible and transferable to real small-rocket workflows.

**Required artifacts:**
1. Re-runnable scripts with fixed seed controls.
2. Firmware + simulation parameter snapshots.
3. Launch-validation bundle from real logs.

**Primary files:**
- `ModelRocket_Adaptive_TVC/tools/generate_all_demos.m`
- `ModelRocket_Adaptive_TVC/tools/build_launch_validation_bundle.m`
- `ModelRocket_Adaptive_TVC/outputs/flight_validation/launch_validation_summary.csv`
- `Firmware/SisyphusCode.cpp`

**Pass criteria:**
1. Independent rerun reproduces headline plots within tolerance.
2. Flight-log pipeline executes without manual data editing.

---

## Claim 5
**Statement:** This work enables safer, practical adoption of fail-aware TVC in amateur/educational systems.

**Required artifacts:**
1. Deployment-oriented setup docs and calibration workflow.
2. Explicit safe-operating envelope and no-claim regions.
3. Open-source release package and usage steps.

**Primary files:**
- `README.md`
- `Firmware/README.md`
- `paper/Tentative_Paper_Draft.md`
- `STS_2027_EXECUTION_ROADMAP.md`

**Pass criteria:**
1. Documentation sufficient for a new user to run bench calibration and simulation bundle.
2. Claims explicitly bounded by validated regime.

---

## Judge-Facing Rule
For every major claim in the paper/application, include:
1. One quantitative table.
2. One visual artifact.
3. One limitation/boundary statement.

If any claim lacks this 3-part evidence, downgrade it to exploratory status.
