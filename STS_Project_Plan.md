# Regeneron STS Project Plan (Rough Execution Roadmap)

## Project Title
Regime-Dependent Limits of Simulation Fidelity in Thrust Vector Controlled Rockets: Experimental Identification of Stability Boundaries Under Delay and Aerodynamic Uncertainty

## Objective
Identify where simulation prediction failure transitions from aerodynamic-dominated to delay-dominated behavior, using a code-only 6-DOF MATLAB model and 15 instrumented flight launches.

## Step-by-Step Plan

1. Define fixed baseline configuration
- Freeze controller gains, geometry, inertia, and avionics stack.
- Record exact firmware, filtering pipeline, and actuator model assumptions.
- Create a single baseline config file and do not change it mid-campaign.

2. Define uncertainty factors and sweep bounds
- Actuator delay: choose 3-5 realistic latency settings.
- Sensor/filter delay: choose 3-5 settings from low to high lag.
- Aerodynamic damping scale: vary around nominal (e.g., 0.7-1.3).
- Thrust variability scale: include expected motor-to-motor variation.

3. Build simulation experiment matrix in MATLAB code
- Generate full-factorial or Latin-hypercube design table.
- Run each case through the same post-processing code.
- Save outputs as residual metrics and stability labels.

4. Pre-register analysis metrics
- Trajectory residuals (angle/rate RMS and peak errors).
- Phase-lag divergence between predicted and measured states.
- Stability criteria (bounded attitude/rate over defined time horizon).

5. Design 15-flight campaign by regime
- 5 launches in low-rate regime.
- 5 launches in intermediate regime.
- 5 launches in high-rate regime.
- Keep hardware changes minimal between launches.

6. Instrument and validate data capture
- Log IMU at high rate (consistent sample period).
- Log commanded vs actual actuator position with synchronized timestamps.
- Verify all channels before each launch with bench test traces.

7. Execute launches with controlled test notes
- For every launch, record wind estimate, mass, CG, and launch condition.
- Tag each launch by intended regime and observed regime.
- Stop and troubleshoot if timestamp quality degrades.

8. Preprocess and align data
- Convert all logs to shared units and coordinate conventions.
- Time-align simulation and hardware streams.
- Apply one consistent filtering/post-processing method.

9. Run ablation comparisons
- Delay-only model perturbation.
- Aero-only model perturbation.
- Combined delay+aero perturbation.
- Compare each against measured data to isolate contribution.

10. Estimate transition boundary
- Quantify where delay contribution overtakes aerodynamic contribution.
- Fit transition boundary in terms of angular-rate regime and uncertainty level.
- Use confidence intervals (bootstrap or repeated-sample estimates).

11. Validate predictive usefulness by regime
- Report where simplified models remain predictive.
- Report where high-fidelity modeling is required.
- Convert findings into actionable controller-design guidance.

12. Prepare STS deliverables
- Final abstract aligned to measured results only.
- 3-5 core figures (residual map, phase divergence, boundary transition plot).
- Methods appendix with reproducibility table and assumptions.

## Suggested 10-Week Timeline

1. Weeks 1-2: Baseline freeze, sweep design, logging validation.
2. Weeks 3-4: Initial code-only simulation campaign and metric pipeline.
3. Weeks 5-7: Flight campaign (15 launches) and quality checks.
4. Weeks 8-9: Ablation, boundary estimation, uncertainty quantification.
5. Week 10: Writing, figure polish, and STS packaging.

## Minimum Data Schema Per Flight
- `flight_id`
- `timestamp_s`
- `theta_deg`, `omega_deg_s`
- `act_cmd_deg`, `act_meas_deg`
- `imu_latency_ms` (if estimated/measured)
- `wind_est_m_s`
- `mass_kg`, `cg_from_nose_m`
- `stability_label`

## Risk Controls
- Keep one untouched backup controller configuration.
- Add go/no-go criteria for weather and telemetry quality.
- Use repeated launches in each regime for statistical confidence.
- Avoid over-claiming before confidence intervals are computed.
