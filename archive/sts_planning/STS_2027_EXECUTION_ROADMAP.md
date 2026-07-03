# STS 2027 Execution Roadmap

## Program Anchor
- Application opens: June 1, 2026.
- Target submission window: first week of November 2026.
- Internal hard deadline: submit 48+ hours early.

## Finalist Objective
Deliver a rigorous, hardware-backed answer to:

Can small-rocket TVC detect and respond to common degradation modes before instability while staying inside a verified safety envelope?

## Success Conditions
1. Outperform at least two strong baselines under matched faults/disturbances.
2. Show ablation-level causality (which module creates which gain).
3. Publish operating boundary maps with uncertainty/repeatability.
4. Demonstrate hardware evidence consistent with simulation claims.
5. Lock a single primary narrative (slew-aware default path) with conditional secondary claims only.

## Monthly Plan
### June (Hardware and Calibration Lock)
1. Integrate analog-feedback servos in firmware.
2. Run bench calibration and monotonicity checks.
3. Finalize telemetry schema for actuator-state and safety flags.
4. Produce calibration report with uncertainty bounds.

### July (Controlled Baseline Campaign)
1. Run fixed and adaptive baselines under matched scenarios.
2. Execute ablation variants.
3. Capture pass/fail rates and post-fault metrics.
4. Freeze baseline test protocol and lock slew-aware as primary if results remain consistent.

### August (Boundary Mapping)
1. Sweep disturbance, mismatch, and degradation axes.
2. Build failure-boundary maps and confidence intervals.
3. Quantify false alarms and missed detections.
4. Identify no-claim regions explicitly, especially where effectiveness-aware channels are only conditionally helpful.

### September (Hardware Validation Lock)
1. Re-run key scenarios on hardware with repeated seeds/runs.
2. Check sim-to-hardware consistency for core claims.
3. Freeze claim language and figures.
4. Build reproducibility package for external users.

### October (Paper and Application Assembly)
1. Finalize report narrative and contribution boundaries.
2. Prepare STS application responses and supporting docs.
3. Complete recommendation logistics and compliance checks.
4. Conduct final artifact audit.

### Early November (Submission)
1. Submit final package before the hard deadline.
2. Keep last 48 hours for QA only.

## Claim Guardrails
1. Do not claim invention of adaptive control theory.
2. Claim validated small-scale translation + safety-bounded deployment.
3. Separate confirmed results from exploratory findings.
4. Report failures and boundary limits alongside wins.

## Deliverables Checklist
1. Reproducible scripts and config snapshots.
2. Hardware log dataset with metadata.
3. Baseline comparison tables/plots.
4. Ablation and failure-boundary artifacts.
5. Clear "safe operating envelope" summary.

## Recommended Launch Campaign (Ideal Count)

Ideal total launches for finalist-grade evidence: **10 to 12 flights**.

Suggested allocation:
1. **2 flights**: integration checkout (sensor, logging, abort logic, ignition sequence).
2. **3 flights**: nominal repeatability runs (same setup, different seeds/wind conditions).
3. **3 flights**: controlled degradation/fault emulation runs.
4. **2 flights**: boundary confirmation runs near pass/fail edge.
5. **+2 optional reserve flights**: anomaly reruns or tie-break evidence.

Minimum acceptable campaign for credible claims: **6 flights**
(1 checkout, 2 nominal, 2 degraded, 1 boundary).

Launch data quality rules:
1. Every launch must produce synchronized controller, actuator, and state logs.
2. Any failed telemetry launch does not count toward evidence targets.
3. Use pre-defined pass/fail thresholds before reviewing outcomes.
