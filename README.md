# Quantifying Stability Boundaries for Amateur TVC Rockets (STS 2027)

This repository builds and validates a free, open-source **stability-mapping and preflight workflow** for small thrust-vector-control rockets. The goal is to replace hobbyist "tune until it flies" practice with a quantitative answer to a harder question: is a candidate rocket easy, fragile, or fundamentally infeasible given its real actuator and airframe?

## Mission
Give amateur and student TVC builders a quantitative preflight answer before they fly: is this rocket stabilizable at all, what hardware parameters matter most, and what starting gains are justified by the measured actuator rather than copied from another project.

## Core Contribution
This project does **not** claim new control theory. It contributes a *measurement-to-decision pipeline* and a quantitative stability map that do not currently exist in the amateur TVC community:
1. Bench measurement of loaded actuator slew, travel, deadband, and backlash.
2. Realistic flight model with documented sensor and actuator nonidealities.
3. Empirically calibrated easy / fragile / infeasible regime map plus GO / MARGINAL / NOGO classifier.
4. Bench-calibrated PD autotuner that beats copied or over-clean-sim gains in the workable region.
5. Regime-specific sim-to-real ablation that tells builders which parts of the reality gap actually matter where.
6. An explicitly secondary severe-collapse controller appendix, kept only where it survives honest stress tests.

## Headline Results (simulation-side, May 2026)
- **Scaling law:** required loaded slew rate ≈ `0.32 · p^2` (R² = 0.75), matching inverted-pendulum theory.
- **Firmware audit:** three publicly deployed open-source TVC firmwares all fail predictably on statically unstable airframes with their shipped PID gains.
- **Validator calibration** on held-out random configs: GO precision 91%, NOGO precision 100%.
- **Measured-aware tuning:** measured-plant PD beats naive-sim or copied-gain baselines in several hard cells.
- **Regime-aware realism screen:** actuator nonlinearity dominates the actuator-limited regime; sensor noise and effectiveness drift dominate near the boundary.
- **Hobby-range controller check:** in `experiments/results/pid_slew_regime_summary.csv`, a best-tuned sweep over nominal loaded slew 45-90 deg/s, `p = 4, 6, 8`, and retained slew scales 1.00-0.25 yields `48/48 BOTH_FINE` for PID and PID_SLEW_AWARE.
- **Severe-collapse appendix:** in the explicit legacy stress probe `experiments/results/pid_slew_probe.csv`, the lightweight slew-aware PID branch can still rescue some low-slew collapse cells, but that is no longer the main claim.

Current nominal sim stack: 75 deg/s loaded gimbal slew. The old ~15 deg/s stack is retained only in explicit severe-collapse controller stress scripts.

## Workspace Map
- `experiments/` — main evidence-generation scripts (validator, scaling law, ablations, comparisons) and `experiments/validator/recommend_envelope.m`.
- `experiments/run_pid_slew_regime_map.m` — honest hobby-range check for whether the lightweight controller matters once the nominal actuator is realistic.
- `experiments/results/` — CSVs and PNGs for every claim; `experiments/results/graphs/preflight_workflow.png` is the headline figure.
- `tools/bench_to_validator.m`, `tools/bench_to_autotune.m`, `tools/process_gimbal_bench_test.m` — bench-CSV → validator pipeline.
- `Firmware/feedback_servo_calibration.ino` — bench-rig firmware (analog-feedback servo data).
- `ModelRocket_Adaptive_TVC/src/simulate_case_realistic.m` — realistic flight simulator.
- `paper/Tentative_Paper_Draft.md`, `paper/Mentor_Project_Summary.md` — current paper framing.
- `data/bench/` — bench-test templates and (eventually) real measurements.

Older directories (`Direction_B_RC_ADRC_PathFollow/`, `supporting/Direction_C_Companion/`) are exploratory branches from earlier directions; they are retained for reference but are not on the current primary path.

## Quick Start
### Regenerate the headline figures
```powershell
cd experiments
& "C:/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "generate_preflight_workflow_figure; generate_phase_diagram_reference_vehicles; run_current_vs_proposed_practice; generate_preflight_story_figures;"
```

### Run the bench-to-autotune bridge on a bench CSV
```matlab
addpath(genpath('tools'));
addpath(genpath('ModelRocket_Adaptive_TVC/src'));
addpath('experiments/validator');
result = bench_to_autotune('data/bench/gimbal_bench_test_template.csv');
disp(result);
```

### Rebuild all simulator-side sweeps (slow)
```powershell
cd experiments
& "C:/Program Files/MATLAB/R2026a/bin/matlab.exe" -batch "run('run_s2r_ablation.m'); run('run_p_estimation_error.m'); run('run_basic_vs_better_sim.m');"
```

## Direction Anchor Files
- `paper/Tentative_Paper_Draft.md` — the paper draft (builder-voice abstract + intro).
- `paper/Mentor_Project_Summary.md` — short mentor-facing summary.
- `experiments/HEADLINE_FINAL.md` — locked simulator-side findings.
- `experiments/results/graphs/preflight_workflow.png` — one-page workflow diagram.
- `experiments/results/graphs/phase_diagram_reference_vehicles.png` — phase diagram with hobbyist reference dots.

## STS Framing
This is positioned as a *quantitative stability-boundary and preflight workflow paper for an underserved community*, not as a new-controller paper. The right STS comparison set is past finalists who built useful, reproducible, community-facing engineering artifacts and validated them with real measurements — not past finalists who claimed new physics or new control theory. The lightweight controller branch is now explicitly appendix-level unless future hardware data proves otherwise.
