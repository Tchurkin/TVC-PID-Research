# Tentative Paper Draft

## Title
When Does TVC Simulation Stop Being Decision-Useful?

*Subtitle: Experimentally Grounded Sim-to-Real Mismatch Decomposition for Low-Cost TVC Rockets*

## Abstract
This project investigates a model-validity question, not a controller race: **which physical mismatch sources dominate the Sim-to-Real (S2R) gap in low-cost TVC rockets, and what minimum model fidelity is needed before simulation remains decision-useful for control design?**

A modular MATLAB pipeline replays launch telemetry and compares measured pitch response against an ODE45-based pitch-plane variable-mass model with second-order actuator dynamics, delay, aerodynamic lookup tables, and sensor effects. The residual is defined as:

$$
\epsilon(t) = \theta_{real}(t) - \theta_{sim}(t)
$$

Two complementary sensitivity analyses are applied: (1) One-factor-at-a-time (OFAT) Monte Carlo decomposition isolates individual mismatch contributions; (2) Morris elementary effects screening captures non-linear interaction effects ignored by OFAT alone. Sources are categorised into **Tier 1** (actuator delay, servo slew, deadband, sensor noise — likely dominant) and **Tier 2** (aerodynamic, inertia, thrust misalignment, wind — likely secondary).

Critically, the pipeline measures the *incremental value* of each fidelity addition — not merely total model accuracy — by computing stability misclassification rate as sources are enabled one-at-a-time. A diagnostic torque decomposition ($\tau_{TVC}$, $\tau_{aero}$, $\tau_{gust}$) is recorded at every timestep for mechanistic interpretation.

The central contribution is a reproducible methodology for determining the minimum simulation fidelity required to preserve design-relevant conclusions about TVC stability.

## 1. Research Question
### Primary question
Which physical effects dominate S2R breakdown in low-cost TVC rockets, and what minimum fidelity is necessary for simulation to remain decision-useful for controller design?

### What this project is NOT
- This is **not** a controller comparison. PID vs ADRC is secondary analysis only.
- This is **not** a claim that "more fidelity always reduces error" (trivially true).
- This is **not** a high-fidelity CFD or 6DOF rigid-body study. Experimental grounding, not simulation realism, is the novelty.

### Core deliverable
A quantified answer: *"These specific hardware effects, at these magnitude ranges, cross a threshold where simplified simulation no longer preserves stability design conclusions."*

### Secondary question
Does ADRC reduce sensitivity to specific mismatch classes compared with PID? (Measured as $\Delta_{sensitivity} = S_{ADRC} - S_{PID}$ per source. Weak finding: "ADRC is better." Strong finding: "ADRC tolerates delay mismatch better than PID by X%.")

## 2. Modeling Framework
### 2.1 Plant and integration
- Pitch-plane dynamic model with variable mass and moment of inertia through burnout transition ($dI/dt$ explicitly tracked).
- Numerical integration: ODE45. No fixed-step Euler.
- States: $[\theta, q, v, h, \delta_{act}, \dot{\delta}_{act}]$.
- All physical constants stored in central `cfg` struct — no magic numbers.

### 2.2 Actuator model (Non-linear, not simplified lag)
Actuator command-to-deflection dynamics:

$$
G(s)=\frac{\omega_n^2}{s^2+2\zeta\omega_n s+\omega_n^2}e^{-\tau s}
$$

with hard enforcement of:
- transport delay $\tau$ (primary S2R source),
- slew-rate saturation at $\dot{\delta}_{max}$ deg/s (likely second-largest source),
- deadband nonlinearity at $\delta_{db}$ deg.

### 2.3 Aerodynamics
- Reynolds-number-dependent coefficients via $C_d(\alpha, Re)$ and $C_m(\alpha, Re)$ lookup tables.
- Pitch damping torque $\tau_{damp} = q_{dyn} S L C_{mq} q L / (2V)$ explicitly separated from restoring torque.

### 2.4 Diagnostic torque logging
Every simulation run stores per-timestep torque components:
- $\tau_{TVC}(t)$ — control authority from gimbal deflection
- $\tau_{aero,restoring}(t)$ — aerodynamic restoring moment
- $\tau_{aero,damp}(t)$ — aerodynamic pitch damping
- $\tau_{gust}(t)$ — wind disturbance torque

This diagnostic struct (`run.diag`) enables mechanistic explanation of *why* the derivative changed at any timestep.

### 2.5 FidelityConfig toggle
A boolean struct `FidelityConfig` enables or disables each mismatch source independently:
```matlab
FidelityConfig.use_actuator_delay = true;   % toggle off → nominal delay
FidelityConfig.use_servo_slew     = false;  % toggle off → infinite slew
```
This is the mechanism for incremental fidelity analysis.

### 2.6 Controllers
- PID baseline with $K_P$, $K_D$ tuned to nominal plant.
- ADRC with LESO (Linear Extended State Observer) for disturbance estimation.
- Controller comparison is supporting analysis; primary result is model validity.

## 3. Data and System Identification Plan
### 3.1 Telemetry processing
- Ingest flight CSV with `interp1` resampling to uniform 100 Hz clock.
- Identify T-0 (liftoff) event marker to synchronise ODE45 simulation start.
- Traces: $\theta(t)$, $q(t)$, gimbal command $\delta_{cmd}(t)$.
- Methodology must remain **fixed** once real data is introduced — no per-launch tuning.

### 3.2 Bench SysID
Use gimbal bench test CSV to estimate $\omega_n$, $\zeta$, $\tau$ via `fminsearchbnd` minimisation of step-response residual.

### 3.3 Motor characterisation
- Import .ENG/.RSE thrust curves via `import_motor_thrust_curve`.
- Monte Carlo $\pm 5\%$ scaling to represent lot-to-lot variation.

## 4. Decomposition Methodology
### 4.1 Residual metrics
For each replayed launch:
- $RMSE(\epsilon)$ — primary magnitude metric
- Maximum absolute residual
- Trend correlation $\rho(\theta_{real}, \theta_{sim})$
- Stability-classification agreement flag

**Decision-usefulness** is a hard binary criterion:
```matlab
sim_useful = stability_match ...
           && rmse_deg < threshold_rmse ...
           && corrcoef_val > threshold_corr;
```
Thresholds are configurable in `s2r_build_config` and logged to `usefulness_thresholds.csv`.

### 4.2 One-factor-at-a-time (OFAT) Monte Carlo
Large Monte Carlo sweeps perturbing one source at a time. Produces intuitive per-source sensitivity estimates. Limitation: non-linear interactions are ignored (addressed by Morris screening).

### 4.3 Morris elementary effects screening (interaction analysis)
Morris screening over all 8 parameters simultaneously using $r$ trajectories of length $k+1$:

$$
EE_i = \frac{y(x + \Delta e_i) - y(x)}{\Delta}
$$

Outputs $\mu^*_i = \langle |EE_i| \rangle$ and $\sigma_i = \text{std}(EE_i)$.
- High $\mu^*$, low $\sigma$: linear, dominant effect.
- High $\mu^*$, high $\sigma$: interaction-driven or non-linear effect.

This directly addresses the scientific weakness of OFAT-only analysis.

### 4.4 Incremental fidelity analysis (most important scientific output)
Sources are added to the simulation model one-at-a-time in Tier-1-first canonical order. At each level $L$, stability misclassification rate is computed across all launches and MC replicates:

| Level | Sources active | Stability misclassification |
|-------|---------------|-----------------------------|
| 0     | (none — ideal sim) | ~baseline% |
| 1     | + actuator_delay | ... |
| 2     | + servo_slew | ... |
| ... | ... | ... |
| 8     | all sources | ~floor% |

The "cliff" in this curve identifies where minimum-necessary fidelity lies.

### 4.5 Mismatch tier structure

**Tier 1 — Likely dominant** (hardware interface effects):
- Actuator delay
- Servo slew rate saturation
- Deadband
- Sensor noise

**Tier 2 — Likely secondary** (plant model uncertainty):
- Aerodynamic coefficient mismatch
- Inertia mismatch
- Thrust misalignment
- Wind disturbance torque

### 4.6 ADRC vs PID delta-sensitivity
$$
\Delta S_i = S_{ADRC,i} - S_{PID,i}
$$
where $S_{controller,i}$ is the OFAT impact score for source $i$. Negative $\Delta S$ means ADRC is less sensitive to that source.

## 5. Decision-Usefulness Criteria
Simulation is treated as decision-useful if it preserves:
1. Stability classification (binary agree/disagree).
2. Trend direction of response ($\rho > 0.7$).
3. Residual RMSE below configurable threshold.

Outputs: `usefulness_metrics.csv`, `usefulness_thresholds.csv`.

## 6. Figures and Tables for Submission

**Priority figures** (in order of scientific importance):
1. **Incremental fidelity curve** — stability misclassification rate vs sources added.
2. **Real vs simulated overlay** — $\theta(t)$ and $q(t)$ for representative flights.
3. **Mismatch sensitivity ranking** — OFAT impact scores with tier labels.
4. **Morris $\mu^*$-$\sigma$ chart** — interaction detection.
5. **PID vs ADRC $\Delta_{sensitivity}$ bar chart** — per-source controller sensitivity comparison.
6. **Fidelity threshold chart** — minimum magnitude threshold per source for 80% agreement.
7. **PCA stress map** — optional if it produces clean agreement/divergence separation.

**Required tables:**
1. Replay metrics by launch (`sim_vs_real_replay_metrics.csv`).
2. Decomposition summary by source (`decomposition_summary.csv`).
3. Fidelity thresholds (`fidelity_thresholds.csv`).
4. Morris screening results (`morris_screening.csv`).
5. Delta-sensitivity PID vs ADRC (`delta_sensitivity_pid_vs_adrc.csv`).
6. Incremental fidelity progression (`incremental_fidelity.csv`).
7. SysID fit results from bench test.

## 7. Claims Boundaries
- No universal PID/ADRC superiority claim.
- No claim that burnout alone explains failures.
- No claim that CFD-level fidelity is required for decision-useful simulation.
- Every claim must map to a generated CSV/figure artifact.
- Controller conclusion must be framed as $\Delta_{sensitivity}$, not "ADRC is better."

## 8. Auto-Generated Central Conclusion Format
The pipeline outputs a `central_conclusion.txt` with the following structure:

> *"Including [top_source_1] and [top_source_2] dynamics reduced stability misclassification from [baseline]% (ideal simulation) to [tier1]% (Tier-1 sources active), while all higher-order fidelity additions combined provided only [tier2_improvement] pp further improvement (full model: [full]%). This establishes actuator hardware effects as the minimum necessary fidelity layer for decision-useful TVC simulation."*

## 9. Current Pipeline Status
Fully implemented pipeline produces:
- ODE45-based replay with variable mass/inertia and 2nd-order actuator.
- OFAT Monte Carlo decomposition with controller tracking.
- Morris elementary effects screening with $\mu^*$-$\sigma$ output.
- Incremental fidelity analysis (9-level curve with misclassification rate).
- PID vs ADRC delta-sensitivity comparison.
- Diagnostic torque decomposition logged per timestep.
- Sensitivity ranking with Tier 1/Tier 2 labels.
- Auto-generated central conclusion string.
- PCA stress map and fidelity threshold estimates.

**Next step to reach finalist-level evidence:** replace synthetic telemetry with measured launch data and repeat the exact pipeline without changing any analysis definitions or thresholds.


## Title
When Does TVC Simulation Stop Being Decision-Useful?

*Subtitle: Sim-to-Real Mismatch Decomposition for Low-Cost TVC Model Rockets*

## Abstract
This project investigates a model-validity question, not a controller race: **which physical mismatch sources dominate the Sim-to-Real (S2R) gap in low-cost TVC rockets, and what minimum model fidelity is needed before simulation remains useful for control design decisions?**

A modular MATLAB pipeline replays launch telemetry and compares measured pitch response against an ODE45-based pitch-plane variable-mass model with second-order actuator dynamics, delay, aerodynamic lookup tables, and sensor effects. The residual is defined as:

$$
\epsilon(t) = \theta_{real}(t) - \theta_{sim}(t)
$$

One-factor-at-a-time Monte Carlo decomposition is used to isolate mismatch classes: actuator delay, slew limit, deadband, sensor noise, inertia mismatch, aerodynamic coefficient mismatch, thrust misalignment, and wind torque disturbance. The pipeline outputs residual error metrics, stability agreement, sensitivity rankings, PCA stress maps, and per-source fidelity thresholds.

The core contribution is a reproducible methodology for determining when simulation preserves design-relevant conclusions and when additional model fidelity is required.

## 1. Research Question
### Primary question
Which physical effects dominate S2R breakdown in low-cost TVC rockets, and what minimum fidelity is necessary for simulation to remain decision-useful for controller design?

### Secondary question
Does ADRC reduce sensitivity to specific mismatch classes compared with PID without claiming universal superiority?

## 2. Modeling Framework
### 2.1 Plant and integration
- Pitch-plane dynamic model with variable mass/inertia through burnout transition.
- Numerical integration uses ODE45 (no simplified Euler core integrator).
- States include pitch angle, pitch rate, actuator states, velocity, and altitude.

### 2.2 Actuator model
Actuator command-to-deflection dynamics are modeled as:

$$
G(s)=\frac{\omega_n^2}{s^2+2\zeta\omega_n s+\omega_n^2}e^{-\tau s}
$$

with:
- transport delay $\tau$,
- slew-rate saturation,
- deadband nonlinearity.

### 2.3 Aerodynamics
- Reynolds-number-dependent drag and moment coefficients via lookup tables $C_d(\alpha, Re)$ and $C_m(\alpha, Re)$.
- Coefficients are evaluated at runtime from angle of attack proxy and velocity-derived Reynolds number.

### 2.4 Controllers
- PID baseline.
- ADRC with LESO disturbance estimate.
- Controller comparison is supporting analysis only; the main result is model validity/decomposition.

## 3. Data and System Identification Plan
### 3.1 Telemetry processing
- Flight traces: $\theta(t)$, $q(t)$, and gimbal command.
- Event markers: disturbance time, burnout, apogee.
- Replay simulation under matched initial conditions.

### 3.2 Bench SysID
Use gimbal bench test data (commanded vs measured deflection) to estimate:
- actuator natural frequency $\omega_n$,
- damping ratio $\zeta$,
- effective delay $\tau$.

### 3.3 Motor characterization
- Import .ENG/.RSE thrust curves.
- Apply Monte Carlo scaling ($\pm 5\%$ nominal) to represent lot-to-lot and environmental variation.

## 4. Decomposition Methodology
### 4.1 Residual metrics
For each replayed launch, compute:
- RMSE of $\epsilon(t)$,
- maximum absolute residual,
- trend correlation between simulated and measured response,
- stability-classification agreement.

### 4.2 One-factor-at-a-time Monte Carlo
Run large Monte Carlo sweeps where only one mismatch source is perturbed per trial. This isolates causal contribution to S2R breakdown.

### 4.3 Outputs for scientific claims
- `decomposition_samples.csv`
- `decomposition_summary.csv`
- `sensitivity_ranking.csv`
- `stress_map_points.csv`
- `fidelity_thresholds.csv`

## 5. Decision-Usefulness Criteria
Simulation is treated as decision-useful if it preserves:
1. Stability classification.
2. Trend direction of response.
3. Residual error below application-defined threshold.

This replaces “best controller wins” framing with “when can simulation be trusted?” framing.

## 6. Figures and Tables for Submission
Required figures:
1. Replay overlay (real vs sim) for representative flights.
2. Sim-to-real error decomposition by mismatch source.
3. Sensitivity ranking bar chart.
4. PCA stress map showing agreement/divergence regimes.
5. Fidelity-threshold chart per mismatch source.

Required tables:
1. Replay metrics by launch.
2. Decomposition summary by source.
3. Fidelity thresholds.
4. SysID fit results for actuator model.

## 7. Claims Boundaries
- No universal PID/ADRC superiority claim.
- No claim that burnout alone explains failures.
- No claim that CFD-level fidelity is required for decision-useful simulation.
- Every claim must map to generated CSV/figure artifacts.

## 8. Current Pipeline Status
Implemented pipeline now produces:
- ODE45-based replay simulation outputs.
- One-factor mismatch decomposition outputs.
- Sensitivity rankings and PCA stress mapping.
- Fidelity-threshold estimates.

Next step to reach finalist-level evidence: replace placeholder telemetry with measured launch data and repeat the exact pipeline without changing the analysis definitions.
