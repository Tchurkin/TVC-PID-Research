# A Bench-Calibrated Preflight Tool for Amateur TVC Rockets (STS 2027)

This repository builds and validates a free, open-source **preflight workflow** for small thrust-vector-control rockets. The workflow replaces hobbyist "tune until it flies" practice with a quantitative pipeline: bench-measure the real servo, propagate that measurement through a realistic flight model, classify the rocket as **GO / MARGINAL / NOGO**, and recommend starting PD gains.

## Mission
Give amateur and student TVC builders a quantitative preflight check and bench-calibrated starting gains, so deciding whether to fly stops being a guess.

## Core Contribution
This project does **not** claim new control theory. It contributes a *measurement-to-decision pipeline* that does not currently exist in the amateur TVC community:
1. Bench measurement of loaded actuator slew, travel, deadband, and backlash.
2. Realistic flight model with documented sensor and actuator nonidealities.
3. Empirically calibrated GO / MARGINAL / NOGO classifier.
4. Bench-calibrated PD autotuner that beats designs from over-clean basic sims.
5. Regime-specific sim-to-real ablation that tells builders which parts of the reality gap actually matter where.

## Headline Results (simulation-side, May 2026)
- **Validator calibration** on held-out random configs: GO precision 91%, NOGO precision 100%.
- **Scaling law:** required loaded slew rate ≈ `0.32 · p^2` (R² = 0.75), matching inverted-pendulum theory.
- **Basic-sim vs better-sim tuning, deployed on realistic plant:** LOW_DEMAND 0.75 → 1.00, BOUNDARY 0.58 → 0.83.
- **Wrong-`p`-assumption sweep:** measured-aware PD beats naive-sim PD by +0.40 to +0.50 success rate in several hard cells.
- **Leave-one-out ablation:** actuator nonlinearity dominates the actuator-limited regime; sensor noise and effectiveness drift dominate near the boundary.
- **Firmware audit:** three publicly deployed open-source TVC firmwares all fail predictably on statically unstable airframes with their shipped PID gains.

## Workspace Map
- `experiments/` — main evidence-generation scripts (validator, scaling law, ablations, comparisons) and `experiments/validator/recommend_envelope.m`.
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
This is positioned as a *novel engineering tool for an underserved community*, not as a discovery paper. The right STS comparison set is past finalists who built useful, reproducible, community-facing engineering artifacts and validated them with real measurements — not past finalists who claimed new physics or new control theory.
