# Teensy Flight Firmware — Fail-Aware JOINT_ADAPTIVE TVC

This folder contains research flight firmware for the Teensy-based rocket PCB.
It implements the JOINT_ADAPTIVE controller structure from the MATLAB research codebase,
with explicit safety guardrails and feedback-mode options for real hardware.

## File

- `SisyphusCode.cpp`: JOINT_ADAPTIVE flight firmware with online keff + slew estimation, bandwidth-matching gain scaling, safety shields, and full diagnostics logging

## What It Does

**Control Law:**
- Online recursive-least-squares (RLS) estimation of effective control gain (`keff`)
- Online servo slew envelope detection when real actuator feedback is available
- Bandwidth-matching gain scaling: `K_eff = K_nominal × (keff_nom / keff_est) × (slew_est / slew_nom)`
- Fixed 2-state LQR state feedback on attitude and rate
- Safety shield 1: Command slew-rate limiter  
- Safety shield 2: Attitude guard (prevent commanding further away when tilted beyond guard angle)
- Countdown tilt/rate abort and manual countdown abort
- Explicit actuator observer fallback when no feedback tap is configured

**Flight Sequence:**
- Pad-angle alignment indicator (blue/red/cyan feedback)
- 5-press rapid-fire arming (with beep feedback)
- 10-second countdown with movement/spin abort checks
- Button press during countdown triggers manual abort
- Ignition pyro trigger and powered-flight TVC active
- Online adaptation continues during powered flight
- Apogee detection and parachute/legs deployment
- Safety aborts on sensor failure, attitude/rate limits
- SD logging of all adaptation diagnostics for post-flight analysis

## Logged Channels (CSV)

Full telemetry row at ~20 Hz during flight:

```
TimeMs,State,AbortReason,ThetaRad,QRad_s,UAct,UCmd,
AltM,VertVelMps,HighAltM,
KeffEst,SlewEst,GainScale,KeffTheta,KeffQ,
DemandRate,DemandRateDecoupled,AbsDuObs,Confidence,
Saturating,SatStreak,ActFeedback,ShieldSlew,ShieldAttitude,ComputeUs
```

The controller channels remain comparable to MATLAB, but the flight log now also records abort reason and whether a real actuator feedback measurement was available on that sample.

## Parameters (Compile-Time Tuning)

Edit these in `SisyphusCode.cpp` to sweep experiment space:

**Adaptation Parameters:**
- `LAMBDA_RLS`: RLS exponential forgetting (default 0.97)
- `ADAPT_GUARD_S`: seconds before keff adaptation enables (default 0.50)
- `CONF_MIN`, `CONF_RESID_GAIN`, `CONF_FLOOR_BLEND`: confidence weighting knobs
- `GATE_DISTURB_GAIN`: disturbance decoupling gate (default 0, opt-in)

**Actuator Feedback / Calibration:**
- `SERVO_X_FB_MODE`: feedback source selector (`OBSERVER`, `ANALOG_ADC`, `AS5600_I2C`)
- `SERVO_X_FB_PIN`: analog feedback pin for true actuator position (`-1` = fallback observer only)
- `SERVO_X_FB_ADC_MIN`, `SERVO_X_FB_ADC_MAX`: ADC calibration endpoints
- `SERVO_X_FB_DEG_MIN`, `SERVO_X_FB_DEG_MAX`: angle calibration endpoints
- `AS5600_ZERO_RAW`: AS5600 12-bit raw code at gimbal mechanical neutral
- `AS5600_DIR`: sign convention (`+1` or `-1`) so measured feedback direction matches control law
- `AS5600_SENSOR_DEG_PER_GIMBAL_DEG`: gear ratio scale (1.0 = direct on gimbal axis)
- `AS5600_BENCH_PRINT`: serial AS5600 telemetry while not in flight (`true` for bench, `false` for launch)
- `TAU_ACT_ASSUMED`, `SLEW_NOM`: should be updated from real bench data

Recommended path:
- Primary: `ANALOG_ADC` with direct feedback servos (least mechanical complexity).
- Secondary/experimental: `AS5600_I2C` for magnetic angle sensing studies.
- Last resort: `OBSERVER` for dry-run and early integration only.

**Safety Shield Knobs:**
- `SAFETY_CMD_SLEW_FRAC`: command rate limiter scale (default 10.0, large = disabled)
- `THETA_GUARD_RAD`: attitude guard threshold (default 70°)

**LQR Gain Baseline:**
- `K1_NOMINAL`, `K2_NOMINAL`: nominal state feedback gains (fixed from MATLAB; do not change)

**Saturation Detection:**
- `SAT_STREAK_MIN`: consecutive saturated samples to commit (default 10, ~50ms)
- `SAT_NOISE_FLOOR`: additive margin on saturation gate (default 0.5 units/s)

## Pin Map (Preserved from Working Hardware)

- `BUTTON`: 14 (armament/reset button)
- `BUZZER`: 13 (audio feedback)
- `RLED/GLED/BLED`: 6/7/8 (status LEDs, active-low)
- `P1`: 9 (landing legs pyro)
- `P2`: 10 (spare pyro)
- `P3`: 11 (ascent motor ignition pyro)
- `P4`: 12 (parachute pyro)
- `SERVO_X_PIN`: 4 (gimbal X servo)
- `SERVO_Y_PIN`: 3 (gimbal Y servo, currently disabled)
- `SD_CS`: BUILTIN_SDCARD (microSD logger)

## AS5600 Wiring (I2C Mode, Optional)

When `SERVO_X_FB_MODE = AS5600_I2C`, wire AS5600 to Teensy `Wire` bus:

- AS5600 `VCC` -> Teensy `3.3V`
- AS5600 `GND` -> Teensy `GND`
- AS5600 `SDA` -> Teensy `SDA` (`Wire`, typically pin 18 on Teensy 4.1)
- AS5600 `SCL` -> Teensy `SCL` (`Wire`, typically pin 19 on Teensy 4.1)

Notes:
- Most AS5600 breakouts already include I2C pull-ups. If your board does not, add ~4.7k pull-ups to 3.3V on SDA/SCL.
- Use a diametric magnet centered on the AS5600 package; off-axis mounting causes angle ripple.
- Keep magnet-to-sensor gap in the vendor-recommended range (typically around 1-3 mm depending on magnet strength).

## AS5600 Bench Calibration / Test (Optional)

1. Set `SERVO_X_FB_MODE = AS5600_I2C` and `AS5600_BENCH_PRINT = true`.
2. Upload firmware and open serial monitor at 115200 baud.
3. Hold gimbal at mechanical neutral and note printed `raw=` value.
4. Set `AS5600_ZERO_RAW` to that neutral raw code.
5. Move gimbal + and - manually:
   - If `u_act` sign is reversed, set `AS5600_DIR = -1`.
   - If encoder is geared, set `AS5600_SENSOR_DEG_PER_GIMBAL_DEG` to sensor_deg/gimbal_deg ratio.
6. Verify small deflections produce symmetric `u_act` values around 0 and sensible `gimbal_deg`.
7. Before flight, set `AS5600_BENCH_PRINT = false` to reduce serial load.

## Pre-Flight Checklist

1. **Bench Static Tests (INERT LOADS ONLY):**
   - Confirm each pyro channel pulses correctly (oscilloscope or multimeter)
   - Verify servo response to command and neutral positioning
   - If using analog actuator feedback, verify ADC counts map correctly to commanded gimbal motion
   - Check IMU orientation (output should show tilting when physically rotated)
   - Verify SD card write via terminal logging

2. **Actuator Bench Fit:**
   - Capture a real `gimbal_bench_test.csv` under `data/bench/`
   - Run `tools/process_gimbal_bench_test.m` to estimate slew rate, delay, deadband, and hysteresis
   - Update `TAU_ACT_ASSUMED`, `SLEW_NOM`, and the feedback calibration constants from measured data
   - If using AS5600, confirm no `ACTUATOR_FEEDBACK_LOSS` abort appears during actuator sweeps
   - If using analog-feedback servos, verify monotonic ADC-to-deflection calibration with no dead zones

3. **Calibration:**
   - Confirm BMP280 address and pressure baseline (set `SEA_LEVEL_HPA`)
   - Gyro offset capture happens at startup (hold rocket level for 1 second)

4. **Arming & Countdown Test (No Propellant):**
   - Pad alignment mode lights up correctly
   - Button presses register and beep
   - Countdown manual abort works
   - Countdown movement/rate abort works without false positives
   - Apogee event triggers normally

## Flight-Ready Parameters

Verify these before each flight:

- `BURN_TIME_S`: motor burn duration (default 3.45 s for Estes F15)
- `COUNTDOWN_S`: preflight countdown duration (default 10 s)
- `SEA_LEVEL_HPA`: pressure at launch site (default 1013.25 hPa)

## Final Firmware Settings (Current Default Build)

`SisyphusCode.cpp` is currently configured for immediate feedback-servo flight testing:
1. `SERVO_X_FB_MODE = ANALOG_ADC`
2. `SERVO_X_FB_PIN = A2`
3. `analogReadResolution(12)` enabled in `setup()`
4. `SERVO_X_FB_ADC_MAX = 4095`
5. `AS5600_BENCH_PRINT = false`

Before launch, you only need to calibrate these four constants:
1. `SERVO_X_FB_ADC_MIN`
2. `SERVO_X_FB_ADC_MAX`
3. `SERVO_X_FB_DEG_MIN`
4. `SERVO_X_FB_DEG_MAX`

If your servo feedback pin outputs >3.3V, add a resistor divider before Teensy analog input.

## Firmware Validation Strategy

### Stage 1: Bench Static (No Pyros)
- Sensors calibration (IMU gyro offset, BMP zero)
- Serial console logs state transitions
- Pad-align and arming flow verified

### Stage 2: Inert Pyro Test
- Each pyro channel pulses separately (bench with dummy load)
- Timing and current draw verified
- Countdown manual abort path tested

### Stage 3: Dry Run (No Propellant)
- Full countdown-to-coast sequence without ignition
- Adaptive estimator state logged (CSV)
- Verify countdown manual abort and movement abort behave as expected
- Verify safety thresholds don't trigger spuriously

### Stage 4: Static Fire (Tethered)
- Single test with actual motor and pyros armed
- Compare logged adaptation data to MATLAB/controller expectations
- Check servo response timing and saturation detection
- If `ActFeedback=0`, treat slew-estimation results as observer-limited and do not claim validated slew identification

### Stage 5: Flight
- Use tuned parameters from simulation validation
- Log always enabled
- Post-flight CSV analysis against sim baseline

## Hardware Readiness Notes

- For rigorous flight validation of slew-aware adaptation, use real actuator feedback (`ANALOG_ADC` preferred for deployment simplicity).
- `AS5600_I2C` is useful for comparative sensing experiments but can be more mechanically sensitive to alignment/flex artifacts.
- `OBSERVER` mode is acceptable for bench, dry run, and early tethered testing, but it is not sufficient evidence for in-flight slew-envelope identification claims.
- The current `BURN_TIME_S = 3.45 s` matches the Estes F15 long-burn motor family and is a reasonable default for initial testing.

## STS Direction Alignment

This firmware supports the paper's core claim:
1. Detect common degradation modes in small TVC systems.
2. Adapt safely within measured actuator limits.
3. Publish repeatable evidence of where adaptation helps and where it does not.

## Compilation & Upload

## Feedback Servo Wiring And Calibration Quick Start

Use `feedback_servo_calibration.ino` for bench calibration before flight firmware upload.

Wiring for each analog-feedback servo:
1. Servo red (power) -> external regulated 5V rail (not Teensy 3.3V)
2. Servo black/brown (ground) -> external rail ground
3. Servo signal (usually white/orange/yellow) -> Teensy PWM output
4. Servo feedback output wire -> Teensy analog input (3.3V-safe only)

Pin map options:
1. Teensy flight map (matches `SisyphusCode.cpp`):
   - X command: pin 4
   - Y command: pin 3
   - X feedback: A2
   - Y feedback: A3
2. Arduino bench map (default in `feedback_servo_calibration.ino`):
   - X command: pin 9
   - Y command: pin 10
   - X feedback: A0
   - Y feedback: A1

Important:
1. Tie Teensy ground and servo power ground together.
2. Do not power servos from the Teensy board.
3. If feedback output exceeds 3.3V, add a voltage divider before A2/A3.
4. On Arduino UNO/Nano bench tests, analog input is 0-5V and reports 0-1023.

Calibration steps:
1. Upload `feedback_servo_calibration.ino`.
2. Open Serial Monitor at 115200 and record CSV output.
3. Note raw feedback at your minimum and maximum allowed gimbal angles.
4. Set these constants in `SisyphusCode.cpp`:
   - `SERVO_X_FB_ADC_MIN`, `SERVO_X_FB_ADC_MAX`
   - `SERVO_X_FB_DEG_MIN`, `SERVO_X_FB_DEG_MAX`
5. Re-upload `SisyphusCode.cpp` and verify `ActFeedback` stays `1` in logs.

If you calibrated on Arduino and then move to Teensy:
1. Repeat a short endpoint calibration on Teensy (recommended), or
2. Convert ADC endpoints approximately from 10-bit to 12-bit by multiplying by 4.

**With Arduino IDE:**
1. Open `SisyphusCode.cpp` in Arduino IDE
2. Select `Teensy 4.1` board (or your variant)
3. Verify libraries installed (`Sketch > Include Library > Manage Libraries`):
   - `MPU6050_tockn`
   - `Adafruit BMP280`
   - `PWMServo`
4. Upload (`Sketch > Upload` or Ctrl+U)

**With PlatformIO:**
```bash
platformio run -t upload
```

**Required Libraries:**
- `MPU6050_tockn` (gyro + accel)
- `Adafruit BMP280` (barometer)
- `PWMServo` (servo control)
- `SD`, `SPI`, `Wire` (built-in)

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| No serial output | Baud rate mismatch | Set 115200 in terminal |
| Sensors not found | I2C address wrong | Check BMP280 0x76 vs 0x77; try both |
| Pyros won't fire | Polarity reversed | Swap pin wiring (HIGH arms, LOW fires) |
| Adaptation not converging | Saturation detection too strict | Reduce `SAT_STREAK_MIN` |
| Servo jittering | Actuator lag model wrong | Adjust `TAU_ACT_ASSUMED` |
| Unexpected aborts | Safety thresholds triggered | Loosen `ABORT_TILT_DEG`, `ABORT_RATE_DPS` |

## References

- **MATLAB Codebase:** `ModelRocket_Adaptive_TVC/src/lqr_layer_joint_adaptive.m`
- **Parameter Source:** `ModelRocket_Adaptive_TVC/src/rocket_defaults.m`
- **Published Baseline:** `DEV_LOG.md`, `paper/Tentative_Paper_Draft.md`
