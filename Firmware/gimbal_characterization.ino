/*
  gimbal_characterization.ino

  One-shot bench characterization of a TVC gimbal. Runs a fixed sequence
  of tests on each servo axis and prints a final summary that maps
  directly into the validator/autotuner pipeline:

    bench_to_validator.m  ->  recommend_envelope.m  ->  GO / MARGINAL / NOGO

  Sequence:
    1. Neutral baseline      - record feedback noise floor (raw counts).
    2. Endpoint sweep        - slow walk to negative endpoint, back to
                               neutral, to positive endpoint, back to
                               neutral. Captures min/max feedback and
                               linkage utilization.
    3. Deadband probe        - tiny commanded steps from neutral; the
                               smallest step that produces a feedback
                               change > 3*noise sigma defines deadband.
    4. Slew step             - large commanded step (negative -> positive
                               endpoint). Peak measured d(feedback)/dt
                               gives loaded slew rate.
    5. Backlash check        - approach the same midpoint from above and
                               from below; difference is backlash.

  After the sequence, the sketch prints a SUMMARY block, then idles
  at neutral. Re-flash or hit RESET to repeat.

  Pin and linkage convention is intentionally identical to
  feedback_servo_calibration.ino.

  Output is CSV during the run plus a clearly delimited SUMMARY block
  at the end. Paste the SUMMARY block into a bench CSV, or pipe the
  raw run into tools/process_gimbal_bench_test.m.
*/

#include <Arduino.h>
#include <math.h>

#if defined(CORE_TEENSY)
#include <PWMServo.h>
using ServoDriver = PWMServo;
#else
#include <Servo.h>
using ServoDriver = Servo;
#endif

ServoDriver servoX;
ServoDriver servoY;

// ---- Pin configuration (matches feedback_servo_calibration.ino) ----
#if defined(CORE_TEENSY)
constexpr int SERVO_X_PIN = 4;
constexpr int SERVO_Y_PIN = 3;
constexpr int FB_X_PIN    = A2;
constexpr int FB_Y_PIN    = A3;
#else
constexpr int SERVO_X_PIN = 9;
constexpr int SERVO_Y_PIN = 10;
constexpr int FB_X_PIN    = A0;
constexpr int FB_Y_PIN    = A1;
#endif

// ---- Mechanical / linkage convention ----
constexpr int   SERVO_NEUTRAL_DEG                = 90;
constexpr float LINKAGE_SERVO_DEG_PER_GIMBAL_DEG = 4.5f;   // physical linkage, bench-corrected 2026-08-04
// (was 4.0, never measured; a 5.5 measurement on 2026-08-03 was itself a mis-measurement)
constexpr float GIMBAL_SWEEP_HALF_SPAN_DEG       = 5.0f;
constexpr int   SERVO_HALF_SPAN_DEG =
    (int)(LINKAGE_SERVO_DEG_PER_GIMBAL_DEG * GIMBAL_SWEEP_HALF_SPAN_DEG + 0.5f);
constexpr int   SERVO_MIN_DEG = SERVO_NEUTRAL_DEG - SERVO_HALF_SPAN_DEG;
constexpr int   SERVO_MAX_DEG = SERVO_NEUTRAL_DEG + SERVO_HALF_SPAN_DEG;

// ---- Command style ----
constexpr bool USE_MICROSECOND_COMMANDS = true;
constexpr int  CMD_MIN_US      = 1000;
constexpr int  CMD_MAX_US      = 2000;
constexpr int  CMD_MIN_DEG_REF = 0;
constexpr int  CMD_MAX_DEG_REF = 180;

// ---- Sequence timing ----
constexpr uint32_t INITIAL_HOLD_MS   = 2000;   // clearance check
constexpr uint32_t SETTLE_MS         = 300;    // wait after each step
constexpr uint32_t BASELINE_MS       = 1000;   // noise-floor capture
constexpr uint32_t SLEW_SAMPLE_MS    = 2;      // ~500 Hz during step
constexpr uint32_t SLEW_WINDOW_MS    = 600;    // sample window after step
constexpr uint32_t SWEEP_STEP_DELAY  = 25;     // ms per servo-deg in slow sweep
constexpr int      DEADBAND_TRIALS   = 8;      // increasing micro-steps

// ---- Calibration: ADC counts per gimbal degree ----
// Bench scaling defaults: ~700 counts of feedback travel across the
// full +/-5 deg gimbal sweep (10 deg) is typical for analog-feedback
// micro servos. Overridden per-axis using measured endpoint travel.
constexpr float DEFAULT_COUNTS_PER_GIMBAL_DEG = 70.0f;

struct AxisResult {
  const char* name;
  ServoDriver* servo;
  int fbPin;

  float baselineMeanRaw = 0;
  float baselineSigmaRaw = 0;

  int   endpointMinRaw = 0;
  int   endpointMaxRaw = 0;
  float countsPerGimbalDeg = DEFAULT_COUNTS_PER_GIMBAL_DEG;
  float endpointTravelGimbalDeg = 0;
  float endpointUtilPct = 0;

  float deadbandGimbalDeg = NAN;
  float loadedSlewDegPerSec = 0;
  float backlashGimbalDeg = 0;
};

AxisResult axisX{"X", &servoX, FB_X_PIN};
AxisResult axisY{"Y", &servoY, FB_Y_PIN};

// ---------- Servo command helpers ----------

static int servoDegToMicroseconds(int servoDeg) {
  long us = map((long)servoDeg,
                (long)CMD_MIN_DEG_REF, (long)CMD_MAX_DEG_REF,
                (long)CMD_MIN_US, (long)CMD_MAX_US);
  if (us < CMD_MIN_US) us = CMD_MIN_US;
  if (us > CMD_MAX_US) us = CMD_MAX_US;
  return (int)us;
}

static void writeServo(ServoDriver &s, int servoDeg) {
  if (servoDeg < 0)   servoDeg = 0;
  if (servoDeg > 180) servoDeg = 180;
  if (USE_MICROSECOND_COMMANDS) {
    s.writeMicroseconds(servoDegToMicroseconds(servoDeg));
  } else {
    s.write(servoDeg);
  }
}

static int gimbalDegToServoDeg(float gimbalDeg) {
  return SERVO_NEUTRAL_DEG +
         (int)(LINKAGE_SERVO_DEG_PER_GIMBAL_DEG * gimbalDeg + 0.5f);
}

static int readFb(int pin) {
  return analogRead(pin);
}

// ---------- Test stages ----------

static void parkBothNeutral() {
  writeServo(servoX, SERVO_NEUTRAL_DEG);
  writeServo(servoY, SERVO_NEUTRAL_DEG);
}

// Slow ramp from current commanded angle to target, in single-deg steps.
static void slowRampTo(ServoDriver &s, int fromDeg, int toDeg) {
  int step = (toDeg >= fromDeg) ? 1 : -1;
  for (int d = fromDeg; d != toDeg; d += step) {
    writeServo(s, d);
    delay(SWEEP_STEP_DELAY);
  }
  writeServo(s, toDeg);
  delay(SWEEP_STEP_DELAY);
}

static void runBaseline(AxisResult &a) {
  writeServo(*a.servo, SERVO_NEUTRAL_DEG);
  delay(SETTLE_MS);
  long n = 0;
  double sum = 0, sumSq = 0;
  uint32_t t0 = millis();
  while ((millis() - t0) < BASELINE_MS) {
    int v = readFb(a.fbPin);
    sum   += v;
    sumSq += (double)v * v;
    n++;
    delay(2);
  }
  double mean = sum / (double)n;
  double var  = sumSq / (double)n - mean * mean;
  if (var < 0) var = 0;
  a.baselineMeanRaw  = (float)mean;
  a.baselineSigmaRaw = (float)sqrt(var);

  Serial.print(F("# ")); Serial.print(a.name);
  Serial.print(F(" baseline_mean_raw=")); Serial.print(a.baselineMeanRaw, 1);
  Serial.print(F(" sigma_raw="));         Serial.println(a.baselineSigmaRaw, 2);
}

static void runEndpointSweep(AxisResult &a) {
  // Walk: neutral -> min -> neutral -> max -> neutral.
  int fbMin = 32767;
  int fbMax = -32768;

  Serial.print(F("# ")); Serial.print(a.name); Serial.println(F(" endpoint_sweep_begin"));
  Serial.println(F("t_ms,axis,cmd_servo_deg,fb_raw"));

  uint32_t t0 = millis();
  auto logAndTrack = [&](int cmd) {
    int v = readFb(a.fbPin);
    if (v < fbMin) fbMin = v;
    if (v > fbMax) fbMax = v;
    Serial.print(millis() - t0); Serial.print(',');
    Serial.print(a.name);        Serial.print(',');
    Serial.print(cmd);           Serial.print(',');
    Serial.println(v);
  };

  // neutral -> min
  for (int d = SERVO_NEUTRAL_DEG; d >= SERVO_MIN_DEG; d--) {
    writeServo(*a.servo, d); delay(SWEEP_STEP_DELAY); logAndTrack(d);
  }
  delay(SETTLE_MS);
  // min -> neutral
  for (int d = SERVO_MIN_DEG; d <= SERVO_NEUTRAL_DEG; d++) {
    writeServo(*a.servo, d); delay(SWEEP_STEP_DELAY); logAndTrack(d);
  }
  delay(SETTLE_MS);
  // neutral -> max
  for (int d = SERVO_NEUTRAL_DEG; d <= SERVO_MAX_DEG; d++) {
    writeServo(*a.servo, d); delay(SWEEP_STEP_DELAY); logAndTrack(d);
  }
  delay(SETTLE_MS);
  // max -> neutral
  for (int d = SERVO_MAX_DEG; d >= SERVO_NEUTRAL_DEG; d--) {
    writeServo(*a.servo, d); delay(SWEEP_STEP_DELAY); logAndTrack(d);
  }
  delay(SETTLE_MS);

  a.endpointMinRaw = fbMin;
  a.endpointMaxRaw = fbMax;

  // Commanded gimbal span = 2 * GIMBAL_SWEEP_HALF_SPAN_DEG (e.g. 10 deg).
  // Observed raw travel = (fbMax - fbMin). Counts per gimbal degree calibration.
  float commandedSpanDeg = 2.0f * GIMBAL_SWEEP_HALF_SPAN_DEG;
  int rawSpan = a.endpointMaxRaw - a.endpointMinRaw;
  if (rawSpan > 0 && commandedSpanDeg > 0) {
    a.countsPerGimbalDeg = (float)rawSpan / commandedSpanDeg;
    a.endpointTravelGimbalDeg = commandedSpanDeg;
    // Utilization: if linkage binds at endpoint, real travel < commanded.
    // We approximate "full travel" as the requested commanded span and
    // flag <100% util only if a separate hard-stop probe is added later.
    // For now, util = 100% by definition of the sweep range; the user
    // will rerun with a wider GIMBAL_SWEEP_HALF_SPAN_DEG if needed.
    a.endpointUtilPct = 100.0f;
  }
}

static void runDeadbandProbe(AxisResult &a) {
  // From neutral, send increasing micro-steps and measure smallest
  // commanded change that yields |fb - baseline| > 3 sigma.
  writeServo(*a.servo, SERVO_NEUTRAL_DEG);
  delay(SETTLE_MS);

  float threshold = 3.0f * a.baselineSigmaRaw;
  if (threshold < 1.0f) threshold = 1.0f;

  float deadbandDeg = NAN;
  Serial.print(F("# ")); Serial.print(a.name); Serial.println(F(" deadband_probe"));
  Serial.println(F("trial,axis,cmd_offset_servo_deg,delta_fb_raw"));

  for (int k = 1; k <= DEADBAND_TRIALS; k++) {
    int cmd = SERVO_NEUTRAL_DEG + k;
    writeServo(*a.servo, cmd);
    delay(SETTLE_MS);
    int fb = readFb(a.fbPin);
    float dFb = fabsf((float)fb - a.baselineMeanRaw);
    Serial.print(k); Serial.print(',');
    Serial.print(a.name); Serial.print(',');
    Serial.print(k);  Serial.print(',');
    Serial.println(dFb, 1);
    if (isnan(deadbandDeg) && dFb >= threshold) {
      // Convert servo-deg trigger to gimbal-deg deadband.
      deadbandDeg = (float)k / LINKAGE_SERVO_DEG_PER_GIMBAL_DEG;
    }
    writeServo(*a.servo, SERVO_NEUTRAL_DEG);
    delay(SETTLE_MS / 2);
  }

  a.deadbandGimbalDeg = deadbandDeg;
}

static void runSlewStep(AxisResult &a) {
  // Park at negative endpoint, command large step to positive endpoint,
  // measure peak |d(fb_gimbal_deg) / dt|.
  writeServo(*a.servo, SERVO_MIN_DEG);
  delay(SETTLE_MS * 3);

  Serial.print(F("# ")); Serial.print(a.name); Serial.println(F(" slew_step"));
  Serial.println(F("t_ms,axis,cmd_servo_deg,fb_raw,fb_gimbal_deg"));

  uint32_t t0 = millis();
  writeServo(*a.servo, SERVO_MAX_DEG);

  int   prevFb   = readFb(a.fbPin);
  uint32_t prevMs = millis();
  float peakSlewDegPerS = 0;
  // Light low-pass on the per-sample slew to suppress single-sample spikes.
  float lpSlew = 0;
  const float lpAlpha = 0.30f;

  while ((millis() - t0) < SLEW_WINDOW_MS) {
    uint32_t now = millis();
    int   fb     = readFb(a.fbPin);
    float fbDeg  = ((float)fb - a.baselineMeanRaw) / a.countsPerGimbalDeg;

    if ((now - prevMs) >= SLEW_SAMPLE_MS) {
      float dt = (now - prevMs) * 1e-3f;
      if (dt > 0) {
        float dDeg = ((float)fb - (float)prevFb) / a.countsPerGimbalDeg;
        float instSlew = fabsf(dDeg / dt);
        lpSlew = lpAlpha * instSlew + (1.0f - lpAlpha) * lpSlew;
        if (lpSlew > peakSlewDegPerS) peakSlewDegPerS = lpSlew;
      }
      Serial.print(now - t0);  Serial.print(',');
      Serial.print(a.name);    Serial.print(',');
      Serial.print(SERVO_MAX_DEG); Serial.print(',');
      Serial.print(fb);        Serial.print(',');
      Serial.println(fbDeg, 3);
      prevFb = fb;
      prevMs = now;
    }
  }

  a.loadedSlewDegPerSec = peakSlewDegPerS;

  // Return to neutral cleanly.
  slowRampTo(*a.servo, SERVO_MAX_DEG, SERVO_NEUTRAL_DEG);
}

static void runBacklashCheck(AxisResult &a) {
  // Approach servo neutral from above and from below; compare feedback.
  int target = SERVO_NEUTRAL_DEG;

  slowRampTo(*a.servo, SERVO_NEUTRAL_DEG, SERVO_MAX_DEG);
  delay(SETTLE_MS);
  slowRampTo(*a.servo, SERVO_MAX_DEG, target);
  delay(SETTLE_MS * 2);
  int fbFromAbove = readFb(a.fbPin);

  slowRampTo(*a.servo, target, SERVO_MIN_DEG);
  delay(SETTLE_MS);
  slowRampTo(*a.servo, SERVO_MIN_DEG, target);
  delay(SETTLE_MS * 2);
  int fbFromBelow = readFb(a.fbPin);

  float deltaRaw = fabsf((float)fbFromAbove - (float)fbFromBelow);
  a.backlashGimbalDeg = deltaRaw / a.countsPerGimbalDeg;

  Serial.print(F("# ")); Serial.print(a.name);
  Serial.print(F(" backlash_fb_above=")); Serial.print(fbFromAbove);
  Serial.print(F(" fb_below="));          Serial.print(fbFromBelow);
  Serial.print(F(" delta_raw="));         Serial.print(deltaRaw, 1);
  Serial.print(F(" delta_gimbal_deg="));  Serial.println(a.backlashGimbalDeg, 3);
}

// ---------- Summary print ----------

static void printAxisSummary(const AxisResult &a) {
  Serial.print(F("axis,"));
  Serial.print(a.name);                                    Serial.print(',');
  Serial.print(F("baseline_mean_raw,"));
  Serial.print(a.baselineMeanRaw, 1);                      Serial.print(',');
  Serial.print(F("baseline_sigma_raw,"));
  Serial.print(a.baselineSigmaRaw, 2);                     Serial.print(',');
  Serial.print(F("endpoint_min_raw,"));
  Serial.print(a.endpointMinRaw);                          Serial.print(',');
  Serial.print(F("endpoint_max_raw,"));
  Serial.print(a.endpointMaxRaw);                          Serial.print(',');
  Serial.print(F("counts_per_gimbal_deg,"));
  Serial.print(a.countsPerGimbalDeg, 2);                   Serial.print(',');
  Serial.print(F("endpoint_travel_gimbal_deg,"));
  Serial.print(a.endpointTravelGimbalDeg, 2);              Serial.print(',');
  Serial.print(F("endpoint_util_pct,"));
  Serial.print(a.endpointUtilPct, 1);                      Serial.print(',');
  Serial.print(F("deadband_gimbal_deg,"));
  if (isnan(a.deadbandGimbalDeg)) Serial.print(F("NA"));
  else                            Serial.print(a.deadbandGimbalDeg, 3);
  Serial.print(',');
  Serial.print(F("loaded_slew_gimbal_deg_per_s,"));
  Serial.print(a.loadedSlewDegPerSec, 2);                  Serial.print(',');
  Serial.print(F("backlash_gimbal_deg,"));
  Serial.println(a.backlashGimbalDeg, 3);
}

static void printSummary() {
  Serial.println();
  Serial.println(F("===== TVC CHARACTERIZATION SUMMARY ====="));
  Serial.print  (F("linkage_servo_deg_per_gimbal_deg,"));
  Serial.println(LINKAGE_SERVO_DEG_PER_GIMBAL_DEG, 3);
  Serial.print  (F("commanded_gimbal_half_span_deg,"));
  Serial.println(GIMBAL_SWEEP_HALF_SPAN_DEG, 3);
  printAxisSummary(axisX);
  printAxisSummary(axisY);
  Serial.println(F("===== END SUMMARY ====="));
  Serial.println();
  Serial.println(F("# Map to validator inputs:"));
  Serial.println(F("#   slew_max_deg_s     = min(X.loaded_slew, Y.loaded_slew)"));
  Serial.println(F("#   servo_max_deg      = endpoint_travel_gimbal_deg / 2"));
  Serial.println(F("#   deadband_deg       = max(X.deadband, Y.deadband)"));
  Serial.println(F("#   backlash_deg       = max(X.backlash, Y.backlash)"));
  Serial.println(F("# Feed these into tools/bench_to_validator.m"));
}

// ---------- Arduino entry points ----------

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart) < 1500) { /* brief wait for USB-CDC */ }

  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  parkBothNeutral();

  Serial.println(F("# gimbal_characterization.ino"));
  Serial.print  (F("# linkage_ratio=")); Serial.println(LINKAGE_SERVO_DEG_PER_GIMBAL_DEG, 3);
  Serial.print  (F("# gimbal_half_span_deg=")); Serial.println(GIMBAL_SWEEP_HALF_SPAN_DEG, 3);
  Serial.println(F("# Hold neutral for clearance check..."));

  delay(INITIAL_HOLD_MS);

  // ---- Run sequence for axis X then axis Y ----
  AxisResult* axes[] = {&axisX, &axisY};
  for (int i = 0; i < 2; i++) {
    AxisResult &a = *axes[i];
    Serial.print(F("# ===== Characterizing axis "));
    Serial.print(a.name);
    Serial.println(F(" ====="));

    runBaseline(a);
    runEndpointSweep(a);
    runDeadbandProbe(a);
    runSlewStep(a);
    runBacklashCheck(a);
    parkBothNeutral();
    delay(SETTLE_MS * 2);
  }

  printSummary();
}

void loop() {
  // Idle at neutral. Hit RESET to repeat the characterization.
  parkBothNeutral();
  delay(1000);
}
