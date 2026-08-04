/*
  feedback_servo_calibration.ino

  Bench tool for analog-feedback TVC servos.
  - Sweeps one or two servos across a safe range.
  - Reads analog feedback outputs.
  - Prints CSV lines for quick endpoint calibration.

  Tested pin defaults are aligned with SisyphusCode.cpp:
    Servo X command pin: 4
    Servo Y command pin: 3
    Servo X feedback pin: A2
    Servo Y feedback pin: A3
*/

#include <Arduino.h>

#if defined(CORE_TEENSY)
#include <PWMServo.h>
using ServoDriver = PWMServo;
#else
#include <Servo.h>
using ServoDriver = Servo;
#endif

ServoDriver servoX;
ServoDriver servoY;

// Command pins:
// - Teensy defaults match flight firmware.
// - Arduino defaults are chosen for quick bench bring-up.
#if defined(CORE_TEENSY)
constexpr int SERVO_X_PIN = 4;
constexpr int SERVO_Y_PIN = 3;
#else
constexpr int SERVO_X_PIN = 9;
constexpr int SERVO_Y_PIN = 10;
#endif

// Feedback pins
#if defined(CORE_TEENSY)
constexpr int FB_X_PIN = A2;
constexpr int FB_Y_PIN = A3;
#else
constexpr int FB_X_PIN = A0;
constexpr int FB_Y_PIN = A1;
#endif

// Safe sweep range for TVC linkage bench testing.
// Define the test range in gimbal angle and convert to servo angle using
// the linkage mechanical advantage (servo_deg / gimbal_deg).
constexpr int SERVO_NEUTRAL_DEG = 90;
constexpr float LINKAGE_SERVO_DEG_PER_GIMBAL_DEG = 4.5f;   // physical linkage, bench-corrected 2026-08-04
// (was 4.0, never measured; a 5.5 measurement on 2026-08-03 was itself a mis-measurement)
constexpr float GIMBAL_SWEEP_HALF_SPAN_DEG = 5.0f; // requested +/-5 deg gimbal
constexpr int SERVO_SWEEP_HALF_SPAN_DEG =
  (int)(LINKAGE_SERVO_DEG_PER_GIMBAL_DEG * GIMBAL_SWEEP_HALF_SPAN_DEG + 0.5f);
constexpr int SWEEP_MIN_DEG = SERVO_NEUTRAL_DEG - SERVO_SWEEP_HALF_SPAN_DEG;
constexpr int SWEEP_MAX_DEG = SERVO_NEUTRAL_DEG + SERVO_SWEEP_HALF_SPAN_DEG;
constexpr int SWEEP_STEP_DEG = 1;
constexpr int SETTLE_MS = 50;
constexpr int LOOP_PAUSE_MS = 600;

// Temporary install mode: keep servo parked at neutral for horn/linkage setup.
// Set to false to resume sweep diagnostics.
constexpr bool HOLD_NEUTRAL_FOR_INSTALL = true;

// Binary-step mode is best for visualizing delay/lag in Serial Plotter.
// true  -> alternate directly between SWEEP_MIN_DEG and SWEEP_MAX_DEG
// false -> classic ramp sweep
constexpr bool BINARY_STEP_MODE = true;
constexpr int BINARY_HOLD_MS = 1200;
constexpr int BINARY_SAMPLE_MS = 10;

// Binary step endpoints use the same gimbal-equivalent sweep range for
// consistent slew diagnostics and linkage-safe travel.
constexpr int STEP_LOW_DEG = SWEEP_MIN_DEG;
constexpr int STEP_HIGH_DEG = SWEEP_MAX_DEG;

// Command style:
// - true: explicit pulse widths (best for checking real endpoint travel)
// - false: angle writes via Servo.write(deg)
constexpr bool USE_MICROSECOND_COMMANDS = true;
constexpr int CMD_MIN_US = 1000;
constexpr int CMD_MAX_US = 2000;

// Set false if only one feedback servo is installed
constexpr bool USE_SERVO_Y = true;

// Output mode:
// - true: Serial Plotter friendly (two lines: commanded_deg, actual_deg_est)
// - false: CSV table for offline calibration
constexpr bool SERIAL_PLOTTER_MODE = true;
constexpr bool PLOT_RAW_FEEDBACK = true;
constexpr float SLEW_SAMPLE_GATE_DEG_PER_S = 1.0f;
constexpr int RECOMMEND_EVERY_TRANSITIONS = 10;
constexpr float RECOMMEND_SAFETY_MARGIN = 0.95f;
constexpr bool PRINT_RECOMMENDATION_IN_PLOTTER = false;

float fbXMinSeen = 1e9f;
float fbXMaxSeen = -1e9f;
float prevActualDegEst = SERVO_NEUTRAL_DEG;
uint32_t prevSampleMs = 0;
float slewDegPerSecLp = 0.0f;
float prevFbX = 0.0f;
float slewRawCountsPerSecLp = 0.0f;

float transitionSlewAbsSum = 0.0f;
int transitionSlewCount = 0;
float transitionAvgSlew = 0.0f;
int transitionDir = 0; // +1 up, -1 down

float rollingUpSlewSum = 0.0f;
int rollingUpCount = 0;
float rollingDownSlewSum = 0.0f;
int rollingDownCount = 0;

float rollingUpAvg = 0.0f;
float rollingDownAvg = 0.0f;

float rollingUpNetSlewSum = 0.0f;
int rollingUpNetCount = 0;
float rollingDownNetSlewSum = 0.0f;
int rollingDownNetCount = 0;
float rollingUpNetAvg = 0.0f;
float rollingDownNetAvg = 0.0f;

uint32_t transitionStartMs = 0;
float transitionStartDeg = SERVO_NEUTRAL_DEG;
float transitionLastDeg = SERVO_NEUTRAL_DEG;
float transitionNetSlew = 0.0f;
float endpointUtilPct = 0.0f;
float endpointUtilSumPct = 0.0f;
int endpointUtilCount = 0;
float endpointUtilAvgPct = 0.0f;
int totalTransitions = 0;

int lastCommandDeg = SERVO_NEUTRAL_DEG;

float estimateRangeMinDeg() {
  return BINARY_STEP_MODE ? (float)STEP_LOW_DEG : (float)SWEEP_MIN_DEG;
}

float estimateRangeMaxDeg() {
  return BINARY_STEP_MODE ? (float)STEP_HIGH_DEG : (float)SWEEP_MAX_DEG;
}

void startTransition(int targetDeg) {
  if (targetDeg == lastCommandDeg) return;
  transitionDir = (targetDeg > lastCommandDeg) ? 1 : -1;
  transitionSlewAbsSum = 0.0f;
  transitionSlewCount = 0;
  transitionAvgSlew = 0.0f;
  transitionNetSlew = 0.0f;
  transitionStartMs = millis();
  transitionStartDeg = prevActualDegEst;
  transitionLastDeg = prevActualDegEst;
  lastCommandDeg = targetDeg;
}

void finalizeTransitionStats() {
  if (transitionSlewCount <= 0 || transitionDir == 0) return;

  transitionAvgSlew = transitionSlewAbsSum / (float)transitionSlewCount;

  uint32_t elapsedMs = millis() - transitionStartMs;
  float elapsedS = (float)elapsedMs / 1000.0f;
  if (elapsedS > 1e-4f) {
    transitionNetSlew = fabsf(transitionLastDeg - transitionStartDeg) / elapsedS;
  }

  float fullSpan = max(1.0f, (float)(STEP_HIGH_DEG - STEP_LOW_DEG));
  endpointUtilPct = 100.0f * fabsf(transitionLastDeg - transitionStartDeg) / fullSpan;
  endpointUtilSumPct += endpointUtilPct;
  endpointUtilCount++;
  endpointUtilAvgPct = endpointUtilSumPct / (float)endpointUtilCount;
  totalTransitions++;

  if (transitionDir > 0) {
    rollingUpSlewSum += transitionAvgSlew;
    rollingUpCount++;
    rollingUpAvg = rollingUpSlewSum / (float)rollingUpCount;

    rollingUpNetSlewSum += transitionNetSlew;
    rollingUpNetCount++;
    rollingUpNetAvg = rollingUpNetSlewSum / (float)rollingUpNetCount;
  } else {
    rollingDownSlewSum += transitionAvgSlew;
    rollingDownCount++;
    rollingDownAvg = rollingDownSlewSum / (float)rollingDownCount;

    rollingDownNetSlewSum += transitionNetSlew;
    rollingDownNetCount++;
    rollingDownNetAvg = rollingDownNetSlewSum / (float)rollingDownNetCount;
  }
}

void printRecommendationIfDue() {
  if (endpointUtilCount <= 0) return;
  if ((totalTransitions % RECOMMEND_EVERY_TRANSITIONS) != 0) return;
  if (SERIAL_PLOTTER_MODE && !PRINT_RECOMMENDATION_IN_PLOTTER) return;

  float utilFrac = constrain(endpointUtilAvgPct / 100.0f, 0.10f, 1.00f);
  float center = 0.5f * ((float)STEP_LOW_DEG + (float)STEP_HIGH_DEG);
  float halfSpan = 0.5f * ((float)STEP_HIGH_DEG - (float)STEP_LOW_DEG);
  float safeHalfSpan = halfSpan * utilFrac * RECOMMEND_SAFETY_MARGIN;

  int recLow = (int)(center - safeHalfSpan + 0.5f);
  int recHigh = (int)(center + safeHalfSpan + 0.5f);

  Serial.print("recommendation,transitions,");
  Serial.print(totalTransitions);
  Serial.print(",endpoint_util_avg_pct,");
  Serial.print(endpointUtilAvgPct, 2);
  Serial.print(",safe_cmd_low_deg,");
  Serial.print(recLow);
  Serial.print(",safe_cmd_high_deg,");
  Serial.println(recHigh);
}

void printHeader() {
  if (SERIAL_PLOTTER_MODE) {
    if (PLOT_RAW_FEEDBACK) {
      Serial.println("commanded_deg,actual_deg_est,slew_deg_per_s,slew_raw_counts_per_s,transition_avg_slew_deg_s,transition_net_slew_deg_s,rolling_up_net_slew_deg_s,rolling_down_net_slew_deg_s,endpoint_util_pct,fb_x_raw");
    } else {
      Serial.println("commanded_deg,actual_deg_est,slew_deg_per_s,slew_raw_counts_per_s,transition_avg_slew_deg_s,transition_net_slew_deg_s,rolling_up_net_slew_deg_s,rolling_down_net_slew_deg_s,endpoint_util_pct");
    }
  } else {
    Serial.println("mode,servo_deg,actual_deg_est,slew_deg_per_s,slew_raw_counts_per_s,transition_avg_slew_deg_s,transition_net_slew_deg_s,rolling_up_net_slew_deg_s,rolling_down_net_slew_deg_s,endpoint_util_pct,fb_x_raw,fb_y_raw,time_ms");
  }
}

int readAveraged(int pin, int samples = 8) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return (int)(sum / samples);
}

void sampleAndPrint(const char* mode, int deg) {
  int fbX = readAveraged(FB_X_PIN);
  int fbY = USE_SERVO_Y ? readAveraged(FB_Y_PIN) : -1;

  if ((float)fbX < fbXMinSeen) fbXMinSeen = (float)fbX;
  if ((float)fbX > fbXMaxSeen) fbXMaxSeen = (float)fbX;

  float estMinDeg = estimateRangeMinDeg();
  float estMaxDeg = estimateRangeMaxDeg();
  float span = max(1.0f, fbXMaxSeen - fbXMinSeen);
  float actualDegEst = estMinDeg + ((float)fbX - fbXMinSeen) * (estMaxDeg - estMinDeg) / span;
  actualDegEst = constrain(actualDegEst, estMinDeg, estMaxDeg);

  uint32_t nowMs = millis();
  float dt = (prevSampleMs > 0) ? ((float)(nowMs - prevSampleMs) / 1000.0f) : 0.0f;
  float slewInst = 0.0f;
  float slewRawInst = 0.0f;
  if (dt > 1e-4f) {
    slewInst = (actualDegEst - prevActualDegEst) / dt;
    slewRawInst = ((float)fbX - prevFbX) / dt;
  }

  if (fabsf(slewInst) > SLEW_SAMPLE_GATE_DEG_PER_S) {
    transitionSlewAbsSum += fabsf(slewInst);
    transitionSlewCount++;
  }

  // Light low-pass so the slew trace is readable in Serial Plotter.
  slewDegPerSecLp = 0.75f * slewDegPerSecLp + 0.25f * slewInst;
  slewRawCountsPerSecLp = 0.75f * slewRawCountsPerSecLp + 0.25f * slewRawInst;

  // Keep transition metrics live so they respond during motion, not only after finalize.
  if (transitionSlewCount > 0) {
    transitionAvgSlew = transitionSlewAbsSum / (float)transitionSlewCount;
  }
  if (transitionStartMs > 0) {
    float elapsedLive = (float)(nowMs - transitionStartMs) / 1000.0f;
    if (elapsedLive > 1e-4f) {
      transitionNetSlew = fabsf(actualDegEst - transitionStartDeg) / elapsedLive;
    }
  }

  prevActualDegEst = actualDegEst;
  prevSampleMs = nowMs;
  prevFbX = (float)fbX;
  transitionLastDeg = actualDegEst;

  if (SERIAL_PLOTTER_MODE) {
    Serial.print((float)deg, 3);
    Serial.print(',');
    Serial.print(actualDegEst, 3);
    Serial.print(',');
    Serial.print(slewDegPerSecLp, 3);
    Serial.print(',');
    Serial.print(slewRawCountsPerSecLp, 3);
    Serial.print(',');
    Serial.print(transitionAvgSlew, 3);
    Serial.print(',');
    Serial.print(transitionNetSlew, 3);
    Serial.print(',');
    Serial.print(rollingUpNetAvg, 3);
    Serial.print(',');
    Serial.print(rollingDownNetAvg, 3);
    Serial.print(',');
    Serial.print(endpointUtilPct, 2);
    if (PLOT_RAW_FEEDBACK) {
      Serial.print(',');
      Serial.println(fbX);
    } else {
      Serial.println();
    }
    return;
  }

  Serial.print(mode);
  Serial.print(',');
  Serial.print(deg);
  Serial.print(',');
  Serial.print(actualDegEst, 3);
  Serial.print(',');
  Serial.print(slewDegPerSecLp, 3);
  Serial.print(',');
  Serial.print(slewRawCountsPerSecLp, 3);
  Serial.print(',');
  Serial.print(transitionAvgSlew, 3);
  Serial.print(',');
  Serial.print(transitionNetSlew, 3);
  Serial.print(',');
  Serial.print(rollingUpNetAvg, 3);
  Serial.print(',');
  Serial.print(rollingDownNetAvg, 3);
  Serial.print(',');
  Serial.print(endpointUtilPct, 2);
  Serial.print(',');
  Serial.print(fbX);
  Serial.print(',');
  Serial.print(fbY);
  Serial.print(',');
  Serial.println(millis());
}

void commandServos(int deg) {
  if (USE_MICROSECOND_COMMANDS) {
    int us = map(deg, 0, 180, CMD_MIN_US, CMD_MAX_US);
    servoX.writeMicroseconds(us);
    if (USE_SERVO_Y) {
      servoY.writeMicroseconds(us);
    }
    return;
  }

  servoX.write(deg);
  if (USE_SERVO_Y) {
    servoY.write(deg);
  }
}

void sweepUp() {
  for (int deg = SWEEP_MIN_DEG; deg <= SWEEP_MAX_DEG; deg += SWEEP_STEP_DEG) {
    commandServos(deg);
    delay(SETTLE_MS);
    sampleAndPrint("up", deg);
  }
}

void sweepDown() {
  for (int deg = SWEEP_MAX_DEG; deg >= SWEEP_MIN_DEG; deg -= SWEEP_STEP_DEG) {
    commandServos(deg);
    delay(SETTLE_MS);
    sampleAndPrint("down", deg);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  // Teensy 4.1 analog input can be set to 12-bit.
  // Arduino UNO/Nano default to 10-bit (0..1023).
#if defined(CORE_TEENSY)
  analogReadResolution(12);
#endif

  pinMode(FB_X_PIN, INPUT);
  pinMode(FB_Y_PIN, INPUT);

  servoX.attach(SERVO_X_PIN);
  if (USE_SERVO_Y) {
    servoY.attach(SERVO_Y_PIN);
  }

  commandServos(SERVO_NEUTRAL_DEG);
  delay(600);

  printHeader();
  sampleAndPrint("neutral", SERVO_NEUTRAL_DEG);
}

void loop() {
  commandServos(SERVO_NEUTRAL_DEG);
  delay(200);
}
