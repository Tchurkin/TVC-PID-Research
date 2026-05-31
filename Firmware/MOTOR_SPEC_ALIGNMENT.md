# Firmware-to-MATLAB Motor Parameter Alignment

This document ensures that the Teensy firmware (`SisyphusCode.cpp`) accurately implements the JOINT_ADAPTIVE controller from MATLAB (`rocket_defaults.m` and related simulation code).

## Motor Physical Specifications

### Thrust Profile
- **Motor Type**: Estes F15-4 (primary flight candidate)
- **Nominal Burn Time**: 3.45 seconds (firmware: `BURN_TIME_S = 3.45f`)
- **Total Impulse**: ~58 N·s
- **Peak Thrust**: ~65 N

### TVC Actuator
- **Type**: Generic servo (e.g., HS-5086WP) with linear mechanical reduction
- **Servo Rate (bare)**: ~1 rad/s
- **Gimbal Mechanical Reduction**: 4:1 (servo linkage lever arm)
- **Resultant Gimbal Slew Rate**: 0.25 rad/s ≈ 14.3 deg/s

### Gimbal Geometry
- **Command Range**: ±15° deflection
- **Code Unit Scaling**: 12 code units ↔ ±15° deflection
- **Servo Neutral Position**: 90° PWM
- **Conversion**: 2.0 deg/unit in PWM space
  - Min PWM: 90 - 12×2 = 66°
  - Max PWM: 90 + 12×2 = 114°

## Plant Model Parameters (Firmware vs. MATLAB)

| Parameter | MATLAB | Firmware | Units | Notes |
|-----------|--------|----------|-------|-------|
| `aero_damp` | 1.20 | 1.20 | unitless | Pitch damping constant |
| `control_eff` (keff_nom) | 8.0 | 8.0 | rad/s² per unit | Control effectiveness at nominal conditions |
| `tau_act` | 0.05 | 0.05 | seconds | Actuator first-order lag (50 ms) |
| `slew_max` | 12.0 | 12.0 | code units/s | Gimbal slew envelope (~0.25 rad/s) |
| `u_max` | 12.0 | 12.0 | code units | Saturation limit (full gimbal range) |
| `dt` (control) | 0.005 | 0.010 | seconds | **Firmware runs at 100 Hz (10 ms)** |
| `dt` (sensor) | 0.005 | 0.005 | seconds | Sensor sampling at 200 Hz (5 ms) |

## Adaptive Controller Parameters (JOINT_ADAPTIVE)

### RLS Keff Estimation
| Parameter | MATLAB | Firmware | Notes |
|-----------|--------|----------|-------|
| `lambda_rls` | 0.97 | 0.97 | Forgetting factor (slower decay = more memory) |
| `adapt_guard_s` | 0.50 | 0.50 | Gate delay: wait 500ms before keff ID starts |
| `keff_min` | 1.2 | 1.2 | Min allowable keff (0.15 × 8.0) |
| `keff_max` | 44.0 | 44.0 | Max allowable keff (5.50 × 8.0) |
| `delta_min` | 0.10 | 0.10 | Excitation threshold (units) |
| `keff_beta` | 0.10 | 0.10 | Update rate for keff blending |
| `alpha_beta` | 0.25 | 0.25 | LP filter for normalized acceleration proxy |

### Slew Envelope Adaptation
| Parameter | MATLAB | Firmware | Notes |
|-----------|--------|----------|-------|
| `slew_nominal` | 12.0 | 12.0 | Nominal slew (units/s) |
| `slew_min` | 1.2 | 1.2 | Min envelope (0.10 × 12.0) |
| `slew_alpha_sat` | 0.05 | 0.05 | Rate constant during saturation (fast pull-down) |
| `slew_alpha_relax` | 0.01 | 0.01 | Rate constant during relaxation (~100 ms) |
| `slew_health_keff_freeze` | 0.70 | 0.70 | Below this, don't trust keff (slew fault dominates) |

### Saturation Detection
| Parameter | MATLAB | Firmware | Notes |
|-----------|--------|----------|-------|
| `sat_streak_min` | 10 | 10 | Consecutive saturated samples to declare saturation (~50 ms @ 200 Hz) |
| `sat_noise_floor` | 0.5 | 0.5 | Additive margin (units/s) to prevent false trips |
| `sat_decay` | 0.10 | 0.10 | Asymmetric decay rate when not saturating |
| `gate_disturb_gain` | 0.00 | 0.00 | Disturbance decoupling (disabled in legacy mode) |

### Gain Scaling & Confidence
| Parameter | MATLAB | Firmware | Notes |
|-----------|--------|----------|-------|
| `conf_min` | 1.00 | 1.00 | Minimum confidence weight (legacy: no weighting) |
| `conf_resid_gain` | 0.00 | 0.00 | Residual penalty (disabled in legacy mode) |
| `conf_floor_blend` | 1.00 | 1.00 | Confidence blend floor (legacy: always 1.0) |
| `min_scale` | 0.10 | 0.10 | Floor on total gain scaling |
| `max_scale` | 4.00 | 4.00 | Ceiling on total gain scaling |

## LQR Gains (FIXED NOMINAL)

Both firmware and MATLAB use identical nominal LQR gains, computed offline from the augmented plant model (rocket + 50 ms actuator lag) with Q=diag(200, 1, 0.01), R=0.5:

```
K_nominal = [20.00, 3.14]  (state feedback on [theta, q])
```

**Firmware**: `K1_NOMINAL = 20.00`, `K2_NOMINAL = 3.14`  
**MATLAB**: `flqr.K = [20.00, 3.14]`

Actual command gain is then scaled by `gain_scale = keff_scale × slew_scale`:
```
K_eff = K_nominal × gain_scale
```

## Safety Thresholds

| Parameter | Firmware | MATLAB | Units | Purpose |
|-----------|----------|--------|-------|---------|
| `THETA_GUARD_RAD` | 70.0° | 80.0° | deg | Extreme attitude guard (firmware more conservative) |
| `U_MAX` | 12.0 | 12.0 | units | Command saturation |
| Attitude abort | 45.0° | N/A | deg | Firmware safety abort threshold |
| Rate abort | 300.0° | N/A | deg/s | Firmware safety abort threshold |

## Countdown Validation

| Parameter | Firmware | Notes |
|-----------|----------|-------|
| `COUNTDOWN_TILT_ABORT_DEG` | 5.0 | Pad tilt error before abort |
| `COUNTDOWN_RATE_ABORT_DPS` | 25.0 | Pad spin rate before abort |
| `COUNTDOWN_MOTION_CONFIRM_MS` | 100 | Hysteresis window for motion detection |

## Flight State Machine

```
BOOT → PAD_ALIGN → ARMING (5 button presses) → COUNTDOWN (10s)
  ↓
POWERED_FLIGHT (TVC active, ~3.45s) → COAST (apogee detection)
  ↓
DEPLOY (parachute + legs) → LANDED
  ↓ (abort path)
ABORTED
```

**New Abort Reasons** (firmware enhancement):
- `ADAPT_ESTIMATE_FAILURE`: keff diverges outside safe bounds
- `APOGEE_TIMEOUT`: Apogee not detected after 10 seconds in coast
- `ACTUATOR_FEEDBACK_LOSS`: Reserved for future sensor redundancy

## Validation Checklist

- [x] Motor burn time aligns (3.45 s)
- [x] Plant parameters match (keff=8.0, tau_act=0.05, aero_damp=1.20)
- [x] RLS adaptation parameters identical to MATLAB
- [x] LQR gains [20.00, 3.14] match nominal design
- [x] Saturation detection logic and gates equivalent
- [x] Safety shields and abort paths documented
- [x] Apogee detection with timeout added (firmware improvement)
- [x] Adaptive validity checking added (firmware improvement)

## Known Differences (Firmware Enhancements)

1. **Adaptive Estimate Validity Gate**: Firmware checks keff bounds and aborts if diverged. MATLAB doesn't have flight-time abort logic for this.
2. **Apogee Detection Timeout**: Firmware has 10-second timeout to prevent hung coast state. MATLAB uses altitude-based logic only.
3. **Coast Transition Reset**: Firmware resets adaptive state at coast to prevent stale estimates. MATLAB simulation continues for analysis.
4. **Actuator Feedback Fallback**: Firmware includes observer-based fallback when no hardware feedback is available.

## Recommended Flight Test

1. **Dry run (no burn)**: Verify countdown abort logic, state machine transitions, SD logging
2. **Bench test with servo feedback**: Validate slew detection with linear potentiometer or encoder
3. **First flight (safe altitude/field)**: Monitor adaptive gain scaling; log flight data for post-flight validation
4. **Second flight (if first successful)**: Introduce known fault scenario (e.g., servo load increase) to test adaptive response

## References

- `rocket_defaults.m`: MATLAB plant and controller configuration
- `simulate_case.m`: MATLAB closed-loop simulation (comparison baseline)
- `SisyphusCode.cpp`: Teensy firmware implementation
