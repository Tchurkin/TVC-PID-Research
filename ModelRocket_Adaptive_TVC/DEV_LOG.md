# ModelRocket Adaptive TVC - Developer Log

Last updated: 2026-05-13

This document is the chronological engineering journal for the project.
It records architecture decisions, major code changes, validation runs,
regressions, fixes, and current open work.

## 0) Project Origin and Core Goal

The project started as a simulation-first testbed for low-cost model-rocket
thrust vector control (TVC), with a specific objective:

- build a controller that remains stable under actuator-envelope faults,
  noisy MEMS sensing, and realistic disturbance conditions,
- while outperforming standard fixed and adaptive baselines.

From the beginning, the high-priority failure mode was not just motor
authority change (`keff`) but servo slew degradation during flight.

## 1) Early Controller Stack and Baselines

Initial/early controller lineup:

- `FIXED_LQR`
- `ADAPTIVE_KEFF_LQR` (RLS-based single-parameter adaptation)
- `SIGMA_MRAC` (leakage-stabilized adaptive baseline)
- `PCH_LQR` (pseudo-control hedging baseline)
- `JOINT_ADAPTIVE` (proposed)

Key direction was to keep baselines fair and strong so comparisons are
credible:

- `SIGMA_MRAC` included proper sigma-modification leakage (not a strawman).
- `PCH_LQR` modeled the standard practitioner hedge strategy with assumed
  nominal envelope.

## 2) Plant and Scenario Framing

Core scenarios were formalized:

- `NOMINAL`
- `HIGH_KEFF_FAULT` (motor authority drop)
- `SLEW_DEGRADATION` (servo rate envelope collapse)

This clarified the main claim:

- single-parameter adaptation can help in `keff` faults,
- but only joint estimation/gating can reliably survive major slew-envelope
  faults.

## 3) Realism Layer Added

A realistic simulation path was added with:

- MEMS gyro white noise,
- bias random walk,
- quantization,
- pure gyro integration (no tilt aid under thrust),
- sensor latency,
- servo deadband and backlash,
- actuator position noise,
- Dryden-style gust shaping,
- optional drift terms.

This moved the project from idealized control demos to hardware-relevant
flight-computer behavior.

## 4) JOINT Controller Maturation

`JOINT_ADAPTIVE` evolved into a two-estimator architecture:

- estimator A: `keff_est`,
- estimator B: `slew_est`.

Core mechanisms included:

- saturation-aware gating,
- `keff` freeze arbitration when slew health is degraded,
- bandwidth-matching gain scaling using identified slew envelope,
- richer telemetry for diagnosis.

Observed result pattern became stable across many runs:

- severe divergence for fixed/single-adaptive controllers in major
  slew-degradation cases,
- bounded recovery for `JOINT_ADAPTIVE`.

## 5) Analysis Pipeline Expansion

The tools pipeline expanded to generate publication-ready artifacts:

- headline slew demo,
- motor-authority demo,
- tip-over envelope,
- robustness sweep,
- 2-DOF coupling demo,
- realism headline and sensitivity,
- JOINT ablation,
- JOINT failure boundary and cause map,
- safety-cert comparison.

Master entry point:

- `tools/generate_all_demos.m`

## 6) Failure Boundary and Cause Taxonomy

Boundary analysis matured beyond pass/fail into cause attribution.
Cause codes were expanded and used in grid outputs:

- `0` pass
- `1` false saturation trigger
- `2` late detection
- `3` confidence collapse / contamination
- `4` authority exhausted
- `5` safety-shield dominated

This enabled targeted tuning rather than blind parameter changes.

## 7) Safety-Certified JOINT Extension (Major Pivot)

A safety-certified extension was integrated into `JOINT_ADAPTIVE`:

- confidence-aware adaptation weighting,
- disturbance-decoupled gate input,
- runtime safety shield (command slew shaping + attitude guard),
- additional diagnostics for confidence, disturbance proxy, and shield use.

### 7.1 Regression Found

Initial aggressive default activation caused major performance regression:

- pass collapse in safety-cert comparison,
- boundary contraction in fault maps,
- large RMS growth.

### 7.2 Corrective Decision

The architecture was kept, but defaults were moved to non-regressive mode:

- safety mechanisms remain implemented and observable,
- aggressive behavior only enabled via explicit variant tuning.

This preserved baseline stability and avoided hidden regressions.

## 8) Workspace-Wide Alignment Pass

A full workspace alignment pass synchronized:

- source controller logic,
- simulators,
- boundary scripts,
- safety-cert demo,
- top-level demo runner,
- README framing,
- paper draft sections/tables.

At this point, all edited files passed static error checks and main scripts
executed successfully.

## 9) Latest Validation Snapshot (2026-05-13)

Full pipeline was executed end-to-end (`generate_all_demos`).

Headline indicators from latest runs:

- Slew degradation (clean sim):
  - `JOINT_ADAPTIVE` RMS post-fault about `2.06 deg`
  - fixed/single-adaptive baselines about `265-298 deg`
- Realism headline:
  - `JOINT_ADAPTIVE` RMS post-fault about `3.67 deg`
  - baselines about `305-387 deg`

Failure boundary summary (`pass >= 0.8` max amplitude by gust sigma):

- 0.10 -> 4.00
- 0.20 -> 5.50
- 0.30 -> 5.50
- 0.40 -> 5.00
- 0.50 -> 5.50
- 0.65 -> 5.50
- 0.80 -> 5.00
- 1.00 -> 4.50

## 10) Safety-Cert Tuning Upgrade (2026-05-13)

New tuning script added:

- `tools/generate_joint_safety_tuning_sweep.m`

Method:

- coordinate sweep over key safety-cert parameters,
- objective balances pass rate, RMS, false-sat, late detection,
- viability guardrail keeps pass and RMS near legacy quality.

Recommended safety-cert parameters selected from sweep:

- `gate_disturb_gain = 0.00`
- `conf_min = 0.70`
- `conf_resid_gain = 1.00`
- `conf_floor_blend = 0.80`
- `safety_cmd_slew_frac = 8.0`
- `theta_guard_rad = 80 deg`

Applied to:

- `tools/generate_joint_safety_cert_demo.m`

Post-update safety-cert demo result:

- Legacy JOINT:
  - pass `1.00`
  - RMS post `4.28 deg`
  - false-sat `33.33%`
- Safety-Cert JOINT (tuned):
  - pass `1.00`
  - RMS post `4.41 deg`
  - false-sat `16.67%`

Interpretation:

- false-trigger behavior improved materially,
- pass rate preserved,
- RMS stayed close to legacy.

## 11) Documentation State

Current key docs:

- `README.md` for project framing and run commands,
- `paper/Tentative_Paper_Draft.md` for manuscript narrative/results,
- this file (`DEV_LOG.md`) as chronological engineering journal.

## 12) Open Work and Next Priorities

Primary remaining engineering goals:

- reduce RMS gap between tuned safety-cert and legacy while keeping lower
  false-sat rate,
- extend tuning objective to include boundary-map area retention,
- run larger-seed confirmation for selected safety-cert settings,
- freeze a candidate parameter set for hardware transition testing.

## 13) Artifact Index (Current)

Main graph outputs:

- `outputs/graphs/slew_degradation_demo.png`
- `outputs/graphs/keff_spike_demo.png`
- `outputs/graphs/tipover_envelope_demo.png`
- `outputs/graphs/robustness_sweep.png`
- `outputs/graphs/two_dof_demo.png`
- `outputs/graphs/realism_headline.png`
- `outputs/graphs/realism_sensitivity.png`
- `outputs/graphs/joint_ablation_demo.png`
- `outputs/graphs/joint_failure_boundary.png`
- `outputs/graphs/joint_safety_cert_demo.png`

Main data outputs:

- `outputs/data/robustness_sweep.csv`
- `outputs/data/realism_montecarlo.csv`
- `outputs/data/joint_ablation.csv`
- `outputs/data/joint_failure_boundary_raw.csv`
- `outputs/data/joint_failure_boundary_grid.csv`
- `outputs/data/joint_safety_cert_demo.csv`
- `outputs/data/joint_safety_tuning_sweep.csv`
- `outputs/data/joint_safety_tuning_best.csv`
