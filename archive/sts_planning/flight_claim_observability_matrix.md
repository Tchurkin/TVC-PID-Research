# Flight Claim Observability Matrix

This matrix defines the minimum flight data needed to test each scientific claim in the TVC thesis pipeline.

## Logging Baseline
- Clock sync: all channels timestamped by a single monotonic clock.
- Required log rate: >= 200 Hz for attitude, rates, and actuator channels.
- Minimum mission metadata: rocket_id, firmware hash, controller type, gains, weather estimate, propellant state.

## Claim Matrix

| Claim ID | Scientific claim | Required flight measurements | Required sensors | Required logging rate | Required calibration procedures | Required post-processing |
|---|---|---|---|---|---|---|
| C1 | Regime structure exists (EASY/FRAGILE/INFEASIBLE) and predicts controllability class | theta(t), q(t), commanded gimbal, actual gimbal, ignition-to-cutoff timeline, success/failure label | IMU gyro + attitude estimator, actuator feedback (pot/encoder), flight computer event logger | 200 Hz state+actuator, 50 Hz events | IMU bias calibration, servo center and travel calibration, actuator timing calibration | trial-level success metrics, regime-assignment replay, boundary consistency check across repeated flights |
| C2 | Regime map is not a grid artifact and generalizes locally | Same as C1 over near-boundary neighboring configurations | Same as C1 + measured servo slew bench value loaded before flight | 200 Hz | Same as C1 + preflight bench slew check | local interpolation residuals, nearest-boundary distance vs observed outcome, holdout-cell confusion table |
| C3 | Dominant failure physics differ by regime | theta/q residual growth, u_cmd-u_act residuals, saturation fraction, latency proxy, wind proxy | IMU, actuator feedback, optional pitot/baro for disturbance context | 200 Hz | Sensor timestamp skew check, actuator feedback linearity check | error-signature extraction (saturation frequency, divergence time, oscillation growth), regime-conditioned attribution |
| C4 | Minimum simulator fidelity level needed for correct design decision depends on regime | Per-flight decision under each fidelity level (sim replay) + observed flight decision | Same as C1 plus reliable actuator feedback | 200 Hz | Same as C1 + replay channel completeness validation | fidelity decision trajectory L0->L5, first_correct_fidelity_level, dominant_missing_physics |
| C5 | Exp5 design lever recommendations improve performance in regime-conditioned way | Before/after modification paired flights: success rate, RMS error, maneuver envelope surrogate | Same as C1; hardware change metadata (servo/gearing/deadband treatment) | 200 Hz | Recalibrate servo after each hardware change, verify unchanged controller code hash unless intentional | paired-delta analysis, lever effect size with CI, tradeoff score validation |
| C6 | Simulator can estimate physical parameters from flight data (digital twin validity) | u_cmd, u_act, theta, q, reference profile, timing | IMU + actuator feedback mandatory | 200 Hz | actuator feedback scale calibration, IMU rate-scale check | parameter ID (slew, deadband, backlash, tau_act, keff, aero_damp), replay error metrics |
| C7 | Validator GO/MARGINAL/NOGO has predictive value in real flights | Preflight predicted verdict + postflight outcome and margin metrics | Same as C1 and preflight bench ingestion | 200 Hz | bench parser validation, predictor version lock | confusion matrix, GO precision/NOGO precision, calibration curves |

## Sensor and Hardware Minimum Set
- Mandatory:
  - 6-DOF IMU (gyro channels required).
  - Actuator feedback (potentiometer or encoder) for true u_act.
  - Flight computer log with controller internals: u_cmd, gains, mode flags.
- Recommended:
  - Barometer for event segmentation.
  - Optional wind proxy (pitot or external weather station) to contextualize disturbance load.

## Preflight Calibration Checklist
1. IMU gyro bias capture with stationary dwell >= 5 s.
2. Actuator center, endpoint, and polarity verification.
3. Actuator slew bench test at flight battery state.
4. Feedback linearity fit (command vs measured position).
5. End-to-end timestamp latency sanity test (command step response).

## Postflight Processing Checklist
1. Synchronize and resample logs to a common timeline.
2. Compute standard outcomes: success_rate surrogate, RMS error, peak error, saturation fractions.
3. Run parameter identification (`estimate_parameters.m`).
4. Run replay (`replay_flight.m`) and residual audit (`compare_prediction_vs_reality.m`).
5. Update fidelity ladder decision agreement and Exp5 lever deltas.
6. Record claim status as confirmed/refuted/ambiguous with explicit criteria.
