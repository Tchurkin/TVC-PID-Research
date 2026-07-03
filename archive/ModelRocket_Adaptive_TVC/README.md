# ModelRocket Adaptive TVC

This project implements a research-to-hardware workflow for
**fail-aware, safety-bounded TVC control** on small rockets.

Mission:
- Translate modern adaptive/robust control ideas into a small-rocket TVC stack
	that is practical, reproducible, and safer for real users.

Core technical direction:
- Actuator-envelope-aware control (slew-aware) as the primary adaptive path.
- Control-effectiveness-aware adaptation (`keff`) as a conditional extension for severe authority-loss stress cases.
- Confidence-aware adaptation under noisy sensing.
- Disturbance-decoupled fault gating to reduce false commits.
- Runtime safety shields and explicit operating-boundary validation.

Baselines included:
- `FIXED_LQR`
- `ADAPTIVE_KEFF_LQR` (bare RLS)
- `SLEW_ADAPTIVE`
- `SIGMA_MRAC`
- `PCH_LQR`
- `JOINT_ADAPTIVE`

Primary framing:
- Not "inventing adaptive control theory".
- Contribution is a validated small-scale integration with clear boundaries,
  reproducible artifacts, and deployment-oriented safety logic.
- Current evidence emphasis: slew-aware control is the practical default;
	effectiveness-aware channels are retained as bounded, testable extensions.

## Entry Points

Run complete artifact generation from MATLAB:

```matlab
cd ModelRocket_Adaptive_TVC/tools
generate_all_demos
```

Run safety-cert tuning sweep:

```matlab
cd ModelRocket_Adaptive_TVC/tools
generate_joint_safety_tuning_sweep
```

Run post-flight launch validation bundle:

```matlab
cd ModelRocket_Adaptive_TVC/tools
run_launch_validation_pipeline
```

Run focused sensitivity sweeps used for claim lock:

```matlab
cd ModelRocket_Adaptive_TVC/tools
quick_slew_drop_sweep
quick_authority_sweep_compare
```

Developer journal / project history:

- `DEV_LOG.md`

## Primary Outputs

Graphs:
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

Data:
- `outputs/data/robustness_sweep.csv`
- `outputs/data/slew_drop_sweep_10_to_50.csv`
- `outputs/data/realism_montecarlo.csv`
- `outputs/data/joint_ablation.csv`
- `outputs/data/joint_failure_boundary_raw.csv`
- `outputs/data/joint_failure_boundary_grid.csv`
- `outputs/data/joint_safety_cert_demo.csv`
- `outputs/data/joint_safety_tuning_sweep.csv`
- `outputs/data/joint_safety_tuning_best.csv`

## Simulation Claim Lock (May 2026 snapshot)

From current generated artifacts:
1. High-stress slew-degradation regime: slew-aware and joint-adaptive are dominant (~3.4 deg mean post-fault RMS) while fixed/keff-only/PCH can diverge strongly.
2. Authority-loss regime under current setup: fixed and slew-aware remain strong, while keff-only and PCH are conditional and can degrade under heavy disturbance.
3. Moderate standalone slew degradation (10%-50%) at tested disturbance did not separate controllers much; this defines a low-stress region where adaptation may be unnecessary.
4. Core research claim is therefore boundary-aware controller selection under measured uncertainty, not universal superiority of one controller.

Launch validation artifacts:
- `outputs/flight_validation/launch_validation_summary.csv`
- `outputs/flight_validation/launch_validation_report.md`
- `outputs/flight_validation/graphs/*_overview.png`

Flight log input location:
- `data/flight_logs/` (copy raw Teensy CSV files here)

## STS-Ready Evidence Stack

For finalist-level competitiveness, prioritize these artifacts:
1. Baseline comparisons under matched disturbance/fault conditions.
2. Ablation evidence showing mechanism-level causality.
3. Failure-boundary maps with uncertainty/repeatability.
4. Hardware-backed validation (not simulation-only claims).
