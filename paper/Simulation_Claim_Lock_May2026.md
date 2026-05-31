# Simulation Claim Lock (May 2026)

## Purpose
This note freezes the simulation-side claim before full hardware lock.
It is intentionally conservative and tied to generated artifacts.

## Primary claim
The main contribution is not universal controller superiority.
The contribution is a measured-uncertainty workflow that maps where controller rankings change and where adaptation is or is not justified.

## Evidence summary
Source artifacts:
- ModelRocket_Adaptive_TVC/outputs/data/robustness_sweep.csv
- ModelRocket_Adaptive_TVC/outputs/data/slew_drop_sweep_10_to_50.csv
- ModelRocket_Adaptive_TVC/outputs/data/realism_montecarlo.csv

### A) High-stress slew-degradation regime
Mean post-fault RMS (deg):
1. Fixed LQR: 211.687
2. keff-only: 293.257
3. slew-only: 3.442
4. PCH: 221.867
5. Joint Adaptive: 3.421

Interpretation:
1. Explicit actuator-envelope awareness is strongly beneficial in this stress regime.
2. Naive effectiveness-only adaptation can be actively harmful when slew-limited dynamics dominate.

### B) Authority-drop regime (current disturbance setup)
Mean post-fault RMS (deg):
1. Fixed LQR: 3.737
2. keff-only: 110.725
3. slew-only: 4.101
4. PCH: 92.403
5. Joint Adaptive: 8.167

Interpretation:
1. Fixed and slew-aware remain strong in this tested setup.
2. Effectiveness-aware channels are conditional and can degrade under heavy disturbance/tuning mismatch.

### C) Moderate standalone slew degradation (10% to 50%)
Mean RMS over tested drops (deg):
1. Fixed LQR: 2.427
2. keff-only: 2.407
3. slew-aware: 2.427
4. Joint Adaptive: 2.412

Interpretation:
1. At the tested disturbance level, this regime is not strongly rate-limited.
2. Adaptation is not always necessary; this is a useful negative result.

## Locked narrative for paper/application
1. Slew-aware control is the primary deployable path where actuator-rate limits are the dominant stressor.
2. Effectiveness-aware adaptation remains a secondary, conditional extension for severe combined-stress regions.
3. The scientific output is a boundary map and controller-selection rule under measured uncertainty.

## Not claimed
1. No claim of new adaptive control theory.
2. No claim that any one controller is best in all regimes.
3. No claim that simulation-only evidence is sufficient for final deployment decisions.

## Hardware closure required
1. Validate at least one boundary transition with bench/static-fire data.
2. Confirm that simulation ranking direction matches hardware trend at key stress points.
3. Publish no-claim regions and failure modes with the same visibility as wins.
