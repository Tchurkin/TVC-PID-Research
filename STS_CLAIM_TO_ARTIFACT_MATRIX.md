# STS Claim-to-Artifact Matrix

This file maps each major research claim to the exact evidence required to defend it in STS judging.

## Claim 1
**Statement:** Amateur TVC rockets split into easy, fragile-but-recoverable, and fundamentally infeasible regimes, and that regime boundary can be quantitatively mapped.

**Required artifacts:**
1. Stability map over instability demand, loaded slew, and travel.
2. Scaling-law fit or equivalent boundary summary.
3. Held-out validation showing that at least one infeasible configuration stays infeasible despite retuning.
4. Boundary statement identifying where claims hold and where they do not.

**Primary files:**
- `experiments/results/validator_calibration.csv`
- `experiments/results/scaling_law_fit.csv`
- `experiments/results/scaling_law_slew_vs_p.csv`
- `experiments/results/graphs/phase_diagram_reference_vehicles.png`

**Pass criteria:**
1. The map separates at least one easy, one fragile, and one infeasible configuration.
2. The infeasible regime is shown to resist ordinary retuning.

---

## Claim 2
**Statement:** Builder-facing hardware parameters such as loaded slew, travel, and mechanical advantage matter more near the boundary than hobby practice usually assumes.

**Required artifacts:**
1. Regime-aware realism screen or ablation showing actuator-envelope effects dominate near the boundary.
2. Bench measurements showing real actuator performance differs materially from nominal assumptions.
3. A builder-facing tradeoff output, such as servo or geometry prioritization, that follows from the map.

**Primary files:**
- `experiments/results/s2r_ablation.csv`
- `experiments/results/basic_vs_better_sim.csv`
- `experiments/results/servo_pareto_frontier.csv`
- `data/bench/`

**Pass criteria:**
1. Changing actuator-envelope assumptions changes the predicted regime or outcome.
2. The paper can make at least one concrete builder recommendation backed by quantitative evidence.

---

## Claim 3
**Statement:** Any lightweight actuator-aware PID hardening claim is explicitly secondary and bounded: best-tuned ordinary PID already covers the tested hobby-relevant nominal regime, so PID_SLEW_AWARE can only be defended as a severe-collapse appendix result.

**Required artifacts:**
1. Hobby-range regime sweep showing whether best-tuned ordinary PID already handles the nominal actuator band.
2. Severe-collapse stress sweep showing whether the controller helps at all once the plant is pushed well below the nominal envelope.
3. Boundary statement explicitly excluding both the infeasible region and the main hobby-nominal regime if ordinary PID already works there.

**Primary files:**
- `ModelRocket_Adaptive_TVC/src/pid_slew_aware_layer.m`
- `experiments/run_pid_slew_regime_map.m`
- `experiments/results/pid_slew_regime_summary.csv`
- `experiments/run_pid_slew_probe.m`
- `experiments/results/pid_slew_probe.csv`

**Pass criteria:**
1. The paper states clearly that best-tuned PID already handles the tested 45-90 deg/s nominal band if that remains true.
2. Any retained controller benefit is shown only in explicit severe-collapse cells and is not marketed as the finalist-level main result.

---

## Claim 4
**Statement:** The bench-to-decision workflow transfers the simulation findings into real small-rocket build and flight decisions.

**Required artifacts:**
1. Bench ingestion path from real actuator data into the validator/autotune workflow.
2. Held-out hardware or launch-adjacent validation showing verdict accuracy.
3. Launch-validation bundle from real logs.

**Primary files:**
- `tools/bench_to_validator.m`
- `tools/bench_to_autotune.m`
- `ModelRocket_Adaptive_TVC/outputs/flight_validation/launch_validation_summary.csv`
- `Firmware/feedback_servo_calibration.ino`

**Pass criteria:**
1. Bench-measured actuator values materially improve verdicts or gains versus naive assumptions.
2. Flight-log pipeline executes without manual data editing.

---

## Claim 5
**Statement:** This work enables a more rigorous and safer amateur TVC design culture by replacing guess-and-check tuning with quantified stability reasoning.

**Required artifacts:**
1. Deployment-oriented setup docs and calibration workflow.
2. Explicit safe-operating envelope, fragile region, and no-claim regions.
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
