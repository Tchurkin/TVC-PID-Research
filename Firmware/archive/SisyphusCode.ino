/*
  SisyphusCode.cpp
  Teensy-based TVC flight firmware implementing JOINT_ADAPTIVE controller.
  
  This firmware follows the JOINT_ADAPTIVE controller structure from MATLAB
  (rocket_defaults.m), with explicit hardware fallbacks where direct actuator
  feedback is not available yet.
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ==================== JOINT_ADAPTIVE PARAMETERS ====================
// All parameters from rocket_defaults.m JOINT_ADAPTIVE struct

constexpr float DT_S                 = 0.005f;   // 200 Hz sensor loop
constexpr float CONTROL_HZ           = 100.0f;   // 100 Hz control rate

// Plant model (used by adaptive estimator)
constexpr float AERO_DAMP            = 1.20f;
constexpr float KEFF_NOM             = 8.0f;     // nominal control effectiveness
constexpr float SLEW_NOM             = 12.0f;    // nominal servo rate (units/s)
constexpr float TAU_ACT_ASSUMED      = 0.05f;    // assumed actuator lag (50 ms)

// Fixed LQR gains
constexpr float K1_NOMINAL           = 20.00f;   // theta feedback gain
constexpr float K2_NOMINAL           = 3.14f;    // q feedback gain
constexpr float U_MAX                = 12.0f;    // command saturation

// ---- Attitude control-law selection ----
// CONTROL_LQR  = existing flight-proven adaptive-LQR state feedback. DEFAULT (no behavior change).
// CONTROL_ADRC = ADRC ported from the validated landing sim (ESO + online keff as b0).
//                BENCH-TEST on hardware before flying it; then set CONTROL_MODE = CONTROL_ADRC.
#define CONTROL_LQR  0
#define CONTROL_ADRC 1
#define CONTROL_MODE CONTROL_LQR
constexpr float ADRC_WC = 6.0f;                  // ADRC controller bandwidth (rad/s) -- from the validated sim
constexpr float ADRC_W0 = 30.0f;                 // ESO bandwidth (rad/s) = 5*wc
constexpr float DT_CTRL = 1.0f / CONTROL_HZ;     // true control-loop period (s) for the ESO integration

// Adaptation parameters (from jad struct in rocket_defaults.m)
constexpr float LAMBDA_RLS           = 0.97f;
constexpr float ADAPT_GUARD_S        = 0.50f;
constexpr float KEFF_MIN             = 0.15f * KEFF_NOM;
constexpr float KEFF_MAX             = 5.50f * KEFF_NOM;
constexpr float DELTA_MIN            = 0.10f;
constexpr float KEFF_BETA            = 0.10f;
constexpr float ALPHA_BETA           = 0.25f;
constexpr float MIN_SCALE            = 0.10f;
constexpr float MAX_SCALE            = 4.00f;

// Slew adaptation
constexpr float SLEW_MIN             = 0.10f * SLEW_NOM;
constexpr float SLEW_ALPHA_SAT       = 0.05f;
constexpr float SLEW_ALPHA_RELAX     = 0.01f;
constexpr float SLEW_SCALE_MIN       = 0.05f;
constexpr float SLEW_HEALTH_KEFF_FREEZE = 0.70f;

// Saturation detection
constexpr int   SAT_STREAK_MIN       = 10;
constexpr float SAT_NOISE_FLOOR      = 0.5f;
constexpr float SAT_DECAY            = 0.10f;
constexpr float GATE_DISTURB_GAIN    = 0.00f;
constexpr float DISTURB_ALPHA        = 0.03f;
constexpr float CONF_MIN             = 1.00f;
constexpr float CONF_RESID_GAIN      = 0.00f;
constexpr float CONF_FLOOR_BLEND     = 1.00f;

// Safety shields
constexpr float SAFETY_CMD_SLEW_FRAC = 10.0f;
constexpr float THETA_GUARD_RAD      = 70.0f * (M_PI / 180.0f);

// Flight parameters
constexpr float BURN_TIME_S          = 3.45f;   // Estes F15 nominal burn time
constexpr float COUNTDOWN_S          = 10.0f;
constexpr float SEA_LEVEL_HPA        = 1013.25f;
constexpr uint32_t SENSOR_PERIOD_MS  = (uint32_t)(DT_S * 1000.0f + 0.5f);
constexpr uint32_t CONTROL_PERIOD_MS = (uint32_t)(1000.0f / CONTROL_HZ + 0.5f);

// Countdown safety
constexpr float COUNTDOWN_TILT_ABORT_DEG = 5.0f;
constexpr float COUNTDOWN_RATE_ABORT_DPS = 25.0f;
constexpr uint32_t COUNTDOWN_MOTION_CONFIRM_MS = 100;

// Servo command and optional analog feedback calibration.
// Set SERVO_X_FB_PIN to an analog pin wired to a real position sensor or
// feedback servo tap for flight-valid slew identification. Leave at -1 to
// fall back to a nominal actuator observer for bench and dry-run work.
constexpr float SERVO_CMD_SCALE_DEG_PER_UNIT = 2.0f;
constexpr int SERVO_NEUTRAL_DEG = 90;
constexpr float SERVO_MIN_DEG = SERVO_NEUTRAL_DEG - U_MAX * SERVO_CMD_SCALE_DEG_PER_UNIT;
constexpr float SERVO_MAX_DEG = SERVO_NEUTRAL_DEG + U_MAX * SERVO_CMD_SCALE_DEG_PER_UNIT;

enum class FeedbackMode : uint8_t {
  OBSERVER,
  ANALOG_ADC,
  AS5600_I2C
};

// Feedback mode selection.
// - OBSERVER: no direct measurement, uses first-order actuator observer fallback
// - ANALOG_ADC: reads analog feedback pin calibration
// - AS5600_I2C: reads AS5600 absolute magnetic angle over I2C
constexpr FeedbackMode SERVO_X_FB_MODE = FeedbackMode::ANALOG_ADC;

// Teensy 4.1 analog input for feedback-servo position tap.
constexpr int SERVO_X_FB_PIN = A2;
constexpr float SERVO_X_FB_ADC_MIN = 0.0f;
constexpr float SERVO_X_FB_ADC_MAX = 4095.0f;
constexpr float SERVO_X_FB_DEG_MIN = SERVO_MIN_DEG;
constexpr float SERVO_X_FB_DEG_MAX = SERVO_MAX_DEG;

// AS5600 (0x36) raw angle register map.
// Set AS5600_ZERO_RAW at mechanical neutral and AS5600_DIR to match sign.
constexpr uint8_t AS5600_I2C_ADDR = 0x36;
constexpr uint8_t AS5600_RAW_ANGLE_HI_REG = 0x0C;
constexpr uint8_t AS5600_RAW_ANGLE_LO_REG = 0x0D;
constexpr uint16_t AS5600_ZERO_RAW = 0;
constexpr int AS5600_DIR = 1;
constexpr float AS5600_SENSOR_DEG_PER_GIMBAL_DEG = 1.0f;
constexpr bool AS5600_BENCH_PRINT = false;
constexpr uint32_t AS5600_BENCH_PRINT_MS = 200;

// ==================== PINS ====================
constexpr int BUTTON = 14;
constexpr int BUZZER = 13;
constexpr int P1 = 9;      // Landing legs pyro
constexpr int P2 = 10;     // Spare pyro
constexpr int P3 = 11;     // Ascent motor ignition pyro
constexpr int P4 = 12;     // Parachute pyro
constexpr int RLED = 6;
constexpr int GLED = 7;
constexpr int BLED = 8;
constexpr int SD_CS = BUILTIN_SDCARD;
constexpr int SERVO_X_PIN = 4;
constexpr int SERVO_Y_PIN = 3;

// ==================== STATE MACHINE ====================
enum class FlightState : uint8_t {
  BOOT, PAD_ALIGN, ARMING, COUNTDOWN, POWERED_FLIGHT, COAST, DEPLOY, LANDED, ABORTED
};

enum class AbortReason : uint8_t {
  NONE, SENSOR_FAILURE, COUNTDOWN_MOVED, ATTITUDE_LIMIT, RATE_LIMIT, MANUAL_ABORT,
  ADAPT_ESTIMATE_FAILURE, APOGEE_TIMEOUT, ACTUATOR_FEEDBACK_LOSS
};

FlightState state = FlightState::BOOT;
AbortReason abortReason = AbortReason::NONE;

// ==================== ADAPTIVE STATE ====================
struct AdaptiveState {
  float t = 0.0f;
  float keff_est = KEFF_NOM;
  float alpha_lp = 0.0f;
  float S_uu = 1.0f;
  float S_uy = KEFF_NOM;
  float slew_est = SLEW_NOM;
  float sat_streak = 0.0f;
  float du_obs_lp = 0.0f;
  float demand_lp = 0.0f;
  float disturb_lp = 0.0f;
  float resid_lp = 0.0f;
  float z1 = 0.0f, z2 = 0.0f, z3 = 0.0f;   // ADRC ESO states: attitude / rate / total-disturbance estimates
};

struct Diagnostics {
  float keff_est = KEFF_NOM;
  float slew_est = SLEW_NOM;
  float gain_scale = 1.0f;
  float K_eff_theta = K1_NOMINAL;
  float K_eff_q = K2_NOMINAL;
  bool saturating = false;
  float sat_streak = 0.0f;
  float demand_rate = 0.0f;
  float demand_rate_decoupled = 0.0f;
  float du_obs_lp = 0.0f;
  float confidence = 1.0f;
  bool actuator_feedback = false;
  bool shield_slew = false;
  bool shield_attitude = false;
};

struct Telemetry {
  float theta_rad = 0.0f;
  float q_meas = 0.0f;
  float u_act_meas = 0.0f;
  float altitude = 0.0f;
  float initialAlt = 0.0f;
  float highestAlt = 0.0f;
  float vertVel = 0.0f;
  float u_cmd = 0.0f;
  uint32_t computeUs = 0;
};

struct PyroChannel {
  int pin;
  bool active;
  uint32_t startMs;
  uint32_t pulseMs;
};

AdaptiveState adapt;
Diagnostics diag;
Telemetry tm, tm_prev;

// ==================== HARDWARE ====================
PWMServo servoX, servoY;
MPU6050 mpu6050(Wire);
Adafruit_BMP280 bmp;
File logFile;
char logFilename[20];

PyroChannel pyros[] = {
  {P1, false, 0, 900},
  {P2, false, 0, 900},
  {P3, false, 0, 900},
  {P4, false, 0, 900}
};

// ==================== RUNTIME STATE ====================
bool inFlight = false;
float padAngleX = 0.0f, padAngleY = 0.0f;
float gyroRefX = 0.0f, gyroRefY = 0.0f;
float u_cmd_prev = 0.0f, u_act_prev = 0.0f;

uint32_t stateStartMs = 0, ignitionMs = 0;
uint32_t lastSensorMs = 0, lastControlMs = 0, lastLogMs = 0;
uint32_t lastButtonEdgeMs = 0, lastArmPressMs = 0;
int armPressCount = 0;
bool sensorsHealthy = false;
int countdownLastRemaining = -1;
uint32_t countdownMotionStartMs = 0;
uint32_t actuatorFeedbackLossStartMs = 0;

// Apogee and coast detection
bool apogee_detected = false;
float coast_peak_alt = 0.0f;
uint32_t coast_start_ms = 0;
float adaptive_est_valid = 1.0f;  // gate flag for estimate sanity (0 = invalid, 1 = valid)

uint16_t as5600LastRaw = 0;
float as5600LastDeg = 0.0f;
bool as5600LastOk = false;
uint32_t as5600LastBenchPrintMs = 0;

// ==================== UTILITY FUNCTIONS ====================
void neutralServos();

const char* stateName(FlightState s) {
  const char* names[] = {"BOOT", "PAD_ALIGN", "ARMING", "COUNTDOWN", "POWERED", "COAST", "DEPLOY", "LANDED", "ABORTED"};
  return (int)s < 9 ? names[(int)s] : "UNKNOWN";
}

const char* abortName(AbortReason r) {
  const char* names[] = {"NONE", "SENSOR_FAILURE", "COUNTDOWN_MOVED", "ATTITUDE_LIMIT", "RATE_LIMIT", "MANUAL_ABORT",
                         "ADAPT_ESTIMATE_FAILURE", "APOGEE_TIMEOUT", "ACTUATOR_FEEDBACK_LOSS"};
  return (int)r < 9 ? names[(int)r] : "UNKNOWN";
}

void setState(FlightState next) {
  if (state == next) return;
  state = next;
  stateStartMs = millis();
  Serial.print(F("STATE -> "));
  Serial.println(stateName(state));
}

void LED(bool r, bool g, bool b) {
  digitalWrite(RLED, !r);
  digitalWrite(GLED, !g);
  digitalWrite(BLED, !b);
}

void writeServoCommand(float u_cmd) {
  float servoXDeg = constrain(SERVO_NEUTRAL_DEG - u_cmd * SERVO_CMD_SCALE_DEG_PER_UNIT,
                              SERVO_MIN_DEG,
                              SERVO_MAX_DEG);
  servoX.write((int)(servoXDeg + 0.5f));
  servoY.write(SERVO_NEUTRAL_DEG);
}

void resetAdaptiveState() {
  adapt = AdaptiveState{};
  diag = Diagnostics{};
  tm.u_cmd = 0.0f;
  if (SERVO_X_FB_MODE == FeedbackMode::OBSERVER) {
    tm.u_act_meas = 0.0f;
  }
  u_cmd_prev = 0.0f;
  u_act_prev = tm.u_act_meas;
}

static float wrapAngleDeg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

bool readAS5600Raw(uint16_t &raw) {
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(AS5600_RAW_ANGLE_HI_REG);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t readCount = Wire.requestFrom((int)AS5600_I2C_ADDR, 2);
  if (readCount != 2) {
    return false;
  }

  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = (uint16_t)(((uint16_t)hi << 8) | lo) & 0x0FFF;
  return true;
}

bool readAS5600Feedback(float &u_act_code_units) {
  uint16_t raw = 0;
  if (!readAS5600Raw(raw)) {
    as5600LastOk = false;
    return false;
  }

  float rawDeg = (360.0f * (float)raw) / 4096.0f;
  float zeroDeg = (360.0f * (float)(AS5600_ZERO_RAW & 0x0FFF)) / 4096.0f;
  float sensorDeltaDeg = wrapAngleDeg(rawDeg - zeroDeg) * (float)AS5600_DIR;
  float gimbalDeg = sensorDeltaDeg / max(1e-6f, AS5600_SENSOR_DEG_PER_GIMBAL_DEG);
  float servoDeg = SERVO_NEUTRAL_DEG + gimbalDeg;

  u_act_code_units = constrain((SERVO_NEUTRAL_DEG - servoDeg) / SERVO_CMD_SCALE_DEG_PER_UNIT,
                               -U_MAX,
                               U_MAX);
  as5600LastRaw = raw;
  as5600LastDeg = gimbalDeg;
  as5600LastOk = true;
  return true;
}

bool readServoFeedback(float &u_act_code_units) {
  if (SERVO_X_FB_MODE == FeedbackMode::AS5600_I2C) {
    return readAS5600Feedback(u_act_code_units);
  }

  if (SERVO_X_FB_MODE != FeedbackMode::ANALOG_ADC || SERVO_X_FB_PIN < 0) {
    return false;
  }

  float raw = (float)analogRead(SERVO_X_FB_PIN);
  float rawClamped = constrain(raw, SERVO_X_FB_ADC_MIN, SERVO_X_FB_ADC_MAX);
  float rawSpan = max(1.0f, SERVO_X_FB_ADC_MAX - SERVO_X_FB_ADC_MIN);
  float servoDeg = SERVO_X_FB_DEG_MIN
                 + (rawClamped - SERVO_X_FB_ADC_MIN) * (SERVO_X_FB_DEG_MAX - SERVO_X_FB_DEG_MIN) / rawSpan;

  u_act_code_units = constrain((SERVO_NEUTRAL_DEG - servoDeg) / SERVO_CMD_SCALE_DEG_PER_UNIT,
                               -U_MAX,
                               U_MAX);
  return true;
}

void updateActuatorMeasurement(float dt) {
  float measuredU = 0.0f;
  if (readServoFeedback(measuredU)) {
    tm.u_act_meas = measuredU;
    diag.actuator_feedback = true;
    return;
  }

  // Fallback observer for hardware without a feedback tap. This keeps the
  // controller dynamics honest for bench testing, but real slew-fault
  // identification requires a measured actuator state.
  float du_cmd = (tm.u_cmd - tm.u_act_meas) / max(1e-6f, TAU_ACT_ASSUMED);
  float du = constrain(du_cmd, -SLEW_NOM, SLEW_NOM);
  tm.u_act_meas = constrain(tm.u_act_meas + dt * du, -U_MAX, U_MAX);
  diag.actuator_feedback = false;
}

void abortFlight(AbortReason reason, const __FlashStringHelper* detail = nullptr) {
  if (state == FlightState::ABORTED) return;

  abortReason = reason;
  Serial.print(F("ABORT -> "));
  Serial.print(abortName(reason));
  if (detail != nullptr) {
    Serial.print(F(" | "));
    Serial.print(detail);
  }
  Serial.println();

  neutralServos();
  setState(FlightState::ABORTED);
}

void beep(uint16_t freq, uint16_t durMs) {
  tone(BUZZER, freq);
  delay(durMs);
  noTone(BUZZER);
}

bool buttonPressedEdge() {
  bool raw = (digitalRead(BUTTON) == HIGH);
  uint32_t now = millis();
  if (raw && (now - lastButtonEdgeMs > 40)) {
    lastButtonEdgeMs = now;
    while (digitalRead(BUTTON) == HIGH) {
      for (auto &ch : pyros) {
        if (ch.active && (now - ch.startMs >= ch.pulseMs)) {
          digitalWrite(ch.pin, LOW);
          ch.active = false;
        }
      }
      now = millis();
      delay(1);
    }
    return true;
  }
  return false;
}

void updatePyros() {
  uint32_t now = millis();
  for (auto &ch : pyros) {
    if (ch.active && (now - ch.startMs >= ch.pulseMs)) {
      digitalWrite(ch.pin, LOW);
      ch.active = false;
    }
  }
}

void triggerPyro(int pin, uint32_t pulseMs = 900) {
  for (auto &ch : pyros) {
    if (ch.pin == pin && !ch.active) {
      digitalWrite(pin, HIGH);
      ch.active = true;
      ch.startMs = millis();
      ch.pulseMs = pulseMs;
      return;
    }
  }
}

void neutralServos() {
  writeServoCommand(0.0f);
  tm.u_cmd = 0.0f;
  if (SERVO_X_FB_MODE == FeedbackMode::OBSERVER) {
    tm.u_act_meas = 0.0f;
  }
  u_cmd_prev = 0.0f;
  u_act_prev = tm.u_act_meas;
}

// ==================== SENSORS ====================
bool initSensors() {
  Wire.begin();
  Wire.setClock(400000);

  bool bmpOk = bmp.begin(0x76) || bmp.begin(0x77);
  if (!bmpOk) {
    Serial.println(F("BMP280 not found"));
    return false;
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  return true;
}

void capturePadReference() {
  mpu6050.update();
  padAngleX = mpu6050.getAccAngleX() * (M_PI / 180.0f);
  padAngleY = -mpu6050.getAccAngleY() * (M_PI / 180.0f);
  gyroRefX = mpu6050.getGyroAngleX() * (M_PI / 180.0f);
  gyroRefY = mpu6050.getGyroAngleY() * (M_PI / 180.0f);
}

void updateSensors() {
  static uint32_t prevVelMs = 0;
  static float prevAlt = 0.0f;

  tm_prev = tm;
  mpu6050.update();

  // Attitude estimation
  if (inFlight) {
    float gyroAngleX = mpu6050.getGyroAngleX() * (M_PI / 180.0f);
    float gyroAngleY = mpu6050.getGyroAngleY() * (M_PI / 180.0f);
    tm.theta_rad = padAngleX + (gyroAngleX - gyroRefX);
  } else {
    tm.theta_rad = mpu6050.getAccAngleX() * (M_PI / 180.0f);
  }

  // Angular rate
  float angvelXDeg = mpu6050.getGyroX();
  tm.q_meas = angvelXDeg * (M_PI / 180.0f);

  // Altitude
  float altNow = bmp.readAltitude(SEA_LEVEL_HPA) - tm.initialAlt;
  tm.altitude = altNow;
  if (altNow > tm.highestAlt) tm.highestAlt = altNow;

  // Vertical velocity (low-pass filtered)
  uint32_t now = millis();
  if (prevVelMs == 0) {
    prevVelMs = now;
    prevAlt = altNow;
  }
  uint32_t elapsed = now - prevVelMs;
  if (elapsed >= 50) {
    float rawVel = (altNow - prevAlt) / (elapsed / 1000.0f);
    tm.vertVel = 0.2f * rawVel + 0.8f * tm.vertVel;
    prevAlt = altNow;
    prevVelMs = now;
  }

  sensorsHealthy = isfinite(tm.theta_rad) && isfinite(tm.q_meas) && isfinite(tm.altitude);
}

// ==================== JOINT ADAPTIVE CONTROLLER ====================

void runJointAdaptive() {
  uint32_t t0 = micros();

  // Advance adaptation time
  adapt.t += DT_S;

  // --- Observe actuator rate (low-pass filtered) ---
  float du_obs_inst = (tm.u_act_meas - u_act_prev) / max(1e-6f, DT_S);
  float lp_alpha = 0.30f;
  adapt.du_obs_lp = (1.0f - lp_alpha) * adapt.du_obs_lp + lp_alpha * fabsf(du_obs_inst);
  float abs_du_obs = adapt.du_obs_lp;

  // Demand rate (magnitude)
  float demand_rate_inst = fabsf(u_cmd_prev - u_act_prev) / max(1e-3f, TAU_ACT_ASSUMED);
  adapt.demand_lp = (1.0f - lp_alpha) * adapt.demand_lp + lp_alpha * demand_rate_inst;
  float demand_rate = adapt.demand_lp;

  // Disturbance-decoupled gate
  float demand_rate_decoupled = max(0.0f, demand_rate - GATE_DISTURB_GAIN * fabsf(adapt.disturb_lp));
  diag.demand_rate = demand_rate;
  diag.demand_rate_decoupled = demand_rate_decoupled;

  // Saturation detection
  float sat_min_motion = SLEW_NOM * 0.10f;
  bool canObserveActuator = diag.actuator_feedback;
  bool saturating_inst = canObserveActuator
                      && (demand_rate_decoupled > 1.20f * abs_du_obs + SAT_NOISE_FLOOR)
                      && (abs_du_obs > sat_min_motion);

  if (saturating_inst) {
    adapt.sat_streak += 1.0f;
  } else {
    adapt.sat_streak = max(0.0f, adapt.sat_streak - SAT_DECAY);
  }
  bool saturating = (adapt.sat_streak >= SAT_STREAK_MIN);
  diag.saturating = saturating;
  diag.sat_streak = adapt.sat_streak;
  diag.du_obs_lp = abs_du_obs;

  // --- Estimator (1): keff via RLS, gated by NO saturation ---
  float qdot_meas = (tm.q_meas - tm_prev.q_meas) / max(1e-6f, DT_S);
  float alpha_proxy = qdot_meas + AERO_DAMP * tm.q_meas;
  adapt.alpha_lp = (1.0f - ALPHA_BETA) * adapt.alpha_lp + ALPHA_BETA * alpha_proxy;

  float resid_inst = adapt.alpha_lp - adapt.keff_est * tm.u_act_meas;
  adapt.resid_lp = (1.0f - DISTURB_ALPHA) * adapt.resid_lp + DISTURB_ALPHA * fabsf(resid_inst);
  adapt.disturb_lp = (1.0f - DISTURB_ALPHA) * adapt.disturb_lp + DISTURB_ALPHA * resid_inst;

  // Confidence (0..1): low residual + good excitation
  float excitation_conf = min(1.0f, fabsf(tm.u_act_meas) / max(DELTA_MIN, 1e-3f));
  float resid_scale = max(1e-3f, fabsf(adapt.alpha_lp) + fabsf(KEFF_NOM * tm.u_act_meas));
  float resid_norm = adapt.resid_lp / resid_scale;
  float resid_conf = 1.0f / (1.0f + CONF_RESID_GAIN * resid_norm);
  float adapt_conf = max(CONF_MIN, min(1.0f, excitation_conf * resid_conf));
  diag.confidence = adapt_conf;

  bool keff_gate = (adapt.t >= ADAPT_GUARD_S) && (fabsf(tm.u_act_meas) >= DELTA_MIN) && (!saturating);

  if (keff_gate) {
    float weighted_u2 = (adapt_conf * adapt_conf) * tm.u_act_meas * tm.u_act_meas;
    float weighted_uy = (adapt_conf * adapt_conf) * tm.u_act_meas * adapt.alpha_lp;
    adapt.S_uu = LAMBDA_RLS * adapt.S_uu + weighted_u2;
    adapt.S_uy = LAMBDA_RLS * adapt.S_uy + weighted_uy;
    float raw_keff = adapt.S_uy / max(1e-6f, adapt.S_uu);
    raw_keff = min(KEFF_MAX, max(KEFF_MIN, raw_keff));
    float beta_eff = KEFF_BETA * (CONF_FLOOR_BLEND + (1.0f - CONF_FLOOR_BLEND) * adapt_conf);
    adapt.keff_est = (1.0f - beta_eff) * adapt.keff_est + beta_eff * raw_keff;
  }

  // --- Estimator (2): slew envelope ---
  if (canObserveActuator && saturating) {
    float new_se = (1.0f - SLEW_ALPHA_SAT) * abs_du_obs + SLEW_ALPHA_SAT * adapt.slew_est;
    adapt.slew_est = min(adapt.slew_est, new_se);
  } else if (canObserveActuator) {
    adapt.slew_est = (1.0f - SLEW_ALPHA_RELAX) * adapt.slew_est + SLEW_ALPHA_RELAX * SLEW_NOM;
  } else {
    adapt.slew_est = SLEW_NOM;
  }
  adapt.slew_est = max(SLEW_MIN, min(SLEW_NOM, adapt.slew_est));

  // --- Bandwidth-matching gain scaling ---
  float slew_health = adapt.slew_est / SLEW_NOM;
  float keff_scale = 1.0f;
  if (slew_health >= SLEW_HEALTH_KEFF_FREEZE) {
    keff_scale = KEFF_NOM / max(1e-6f, adapt.keff_est);
    keff_scale = min(MAX_SCALE, max(MIN_SCALE, keff_scale));
  }

  float slew_scale = max(SLEW_SCALE_MIN, min(1.0f, slew_health));
  float gain_scale = keff_scale * slew_scale;

  diag.keff_est = adapt.keff_est;
  diag.slew_est = adapt.slew_est;
  diag.gain_scale = gain_scale;
  diag.K_eff_theta = K1_NOMINAL * gain_scale;
  diag.K_eff_q = K2_NOMINAL * gain_scale;

  // --- Control law: adaptive-LQR (default) or ADRC (ported from the validated sim) ---
#if CONTROL_MODE == CONTROL_ADRC
  // ADRC: a 3rd-order ESO estimates rate (z2) and total disturbance (z3) from the attitude estimate; b0 = the online
  // keff estimate, so the firmware's adaptive identification feeds the ADRC inversion. thRef = 0 (regulate to vertical).
  float b0 = min(KEFF_MAX, max(KEFF_MIN, adapt.keff_est));
  float uA = b0 * tm.u_act_meas;                                          // realized angular accel (anti-windup: measured actuator)
  float e_obs = tm.theta_rad - adapt.z1;
  adapt.z1 += DT_CTRL * (adapt.z2 + 3.0f * ADRC_W0 * e_obs);
  adapt.z2 += DT_CTRL * (adapt.z3 + uA + 3.0f * ADRC_W0 * ADRC_W0 * e_obs);
  adapt.z3 += DT_CTRL * (ADRC_W0 * ADRC_W0 * ADRC_W0 * e_obs);
  float alphaDes = ADRC_WC * ADRC_WC * (0.0f - adapt.z1) - 2.0f * ADRC_WC * adapt.z2 - adapt.z3;
  float u_cmd_raw = alphaDes / b0;                                        // invert: command units = desired angular accel / keff
  diag.K_eff_theta = ADRC_WC * ADRC_WC / b0;                             // effective gains, for log comparability
  diag.K_eff_q = 2.0f * ADRC_WC / b0;
#else
  // --- State feedback: u_cmd = -K_eff * [theta; q] ---
  float u_cmd_raw = -(diag.K_eff_theta * tm.theta_rad + diag.K_eff_q * tm.q_meas);
#endif

  // --- Safety shields ---
  diag.shield_slew = false;
  diag.shield_attitude = false;

  // Command slew limiter
  float max_cmd_step = SAFETY_CMD_SLEW_FRAC * adapt.slew_est * DT_S;
  float u_cmd_slew_limited = constrain(u_cmd_raw, u_cmd_prev - max_cmd_step, u_cmd_prev + max_cmd_step);
  if (fabsf(u_cmd_slew_limited - u_cmd_raw) > 1e-9f) {
    diag.shield_slew = true;
  }

  // Attitude guard
  if (fabsf(tm.theta_rad) > THETA_GUARD_RAD && (u_cmd_slew_limited * tm.theta_rad) > 0.0f) {
    u_cmd_slew_limited = 0.0f;
    diag.shield_attitude = true;
  }

  float u_cmd = constrain(u_cmd_slew_limited, -U_MAX, U_MAX);

  tm.u_cmd = u_cmd;
  u_cmd_prev = u_cmd;
  u_act_prev = tm.u_act_meas;

  // Servo output
  writeServoCommand(u_cmd);

  tm.computeUs = micros() - t0;

  // Update adaptive estimate validity for next safety check
  checkAdaptiveEstimateValidity();
}

// ==================== SAFETY ====================
void checkAdaptiveEstimateValidity() {
  // Ensure keff estimate is within safe bounds
  // Out-of-bounds keff indicates model mismatch or sensor fault
  if (adapt.keff_est < KEFF_MIN || adapt.keff_est > KEFF_MAX) {
    adaptive_est_valid = 0.0f;
  } else {
    adaptive_est_valid = 1.0f;
  }
}

void safetyChecks() {
  if (!sensorsHealthy) {
    abortFlight(AbortReason::SENSOR_FAILURE, F("sensor health invalid"));
    return;
  }

  if (SERVO_X_FB_MODE != FeedbackMode::OBSERVER && inFlight) {
    if (!diag.actuator_feedback) {
      if (actuatorFeedbackLossStartMs == 0) {
        actuatorFeedbackLossStartMs = millis();
      } else if (millis() - actuatorFeedbackLossStartMs > 200) {
        abortFlight(AbortReason::ACTUATOR_FEEDBACK_LOSS, F("actuator feedback lost"));
        return;
      }
    } else {
      actuatorFeedbackLossStartMs = 0;
    }
  }

  // Check if adaptive estimates have diverged
  if (adaptive_est_valid < 0.5f) {
    abortFlight(AbortReason::ADAPT_ESTIMATE_FAILURE, F("keff estimate out of bounds"));
    return;
  }

  float theta_deg = tm.theta_rad * 180.0f / M_PI;
  if (fabsf(theta_deg) > 45.0f) {
    abortFlight(AbortReason::ATTITUDE_LIMIT, F("attitude limit"));
    return;
  }

  float rate_dps = tm.q_meas * 180.0f / M_PI;
  if (fabsf(rate_dps) > 300.0f) {
    abortFlight(AbortReason::RATE_LIMIT, F("rate limit"));
    return;
  }
}

void apogeeDetectionCheck() {
  // Track peak altitude during coast; detect apogee via descent + timeout
  if (tm.altitude > coast_peak_alt) {
    coast_peak_alt = tm.altitude;
  }

  // Declare apogee if:
  // 1) Altitude is 1+ meter below peak, AND
  // 2) Vertical velocity is negative (descending), AND
  // 3) At least 2 seconds have elapsed since coast start
  uint32_t coast_elapsed = millis() - coast_start_ms;
  if (!apogee_detected &&
      (coast_peak_alt - tm.altitude > 1.0f) &&
      (tm.vertVel < -0.5f) &&
      (coast_elapsed > 2000)) {
    apogee_detected = true;
  }

  // Timeout: if coast has been running > 10 seconds without apogee detection, abort
  if (!apogee_detected && coast_elapsed > 10000) {
    abortFlight(AbortReason::APOGEE_TIMEOUT, F("apogee not detected after 10s"));
  }
}

// ==================== LOGGING ====================
void createUniqueLogFile() {
  int idx = 0;
  do {
    snprintf(logFilename, sizeof(logFilename), "RES%03d.CSV", idx++);
  } while (SD.exists(logFilename) && idx < 1000);

  logFile = SD.open(logFilename, FILE_WRITE);
  if (!logFile) {
    Serial.println(F("LOG: failed to create file"));
    return;
  }

  Serial.print(F("LOG: "));
  Serial.println(logFilename);

  // CSV header matching MATLAB output
  logFile.println(F("TimeMs,State,AbortReason,"
                    "ThetaRad,QRad_s,UAct,UCmd,"
                    "AltM,VertVelMps,HighAltM,"
                    "KeffEst,SlewEst,GainScale,KeffTheta,KeffQ,"
                    "DemandRate,DemandRateDecoupled,AbsDuObs,Confidence,"
                    "Saturating,SatStreak,ActFeedback,"
                    "ShieldSlew,ShieldAttitude,ComputeUs"));
  logFile.flush();
}

void logDataRow() {
  if (!logFile) return;

  char row[360];
  int n = snprintf(row, sizeof(row),
    "%lu,%s,%s,"
    "%.6f,%.6f,%.6f,%.6f,"
    "%.3f,%.3f,%.3f,"
    "%.6f,%.6f,%.4f,%.4f,%.4f,"
    "%.6f,%.6f,%.6f,%.4f,"
    "%d,%.2f,%d,"
    "%d,%d,%lu\n",
    millis(), stateName(state), abortName(abortReason),
    tm.theta_rad, tm.q_meas, tm.u_act_meas, tm.u_cmd,
    tm.altitude, tm.vertVel, tm.highestAlt,
    diag.keff_est, diag.slew_est, diag.gain_scale, diag.K_eff_theta, diag.K_eff_q,
    diag.demand_rate, diag.demand_rate_decoupled, diag.du_obs_lp, diag.confidence,
    (int)diag.saturating, diag.sat_streak, (int)diag.actuator_feedback,
    (int)diag.shield_slew, (int)diag.shield_attitude,
    tm.computeUs);

  if (n > 0 && n < (int)sizeof(row)) {
    logFile.write((const uint8_t*)row, (size_t)n);
  }
}

// ==================== STATE HANDLERS ====================

void handlePadAlign() {
  float tilt = tm.theta_rad * 180.0f / M_PI;

  if (tilt < -1.0f) LED(false, false, true);
  else if (tilt > 1.0f) LED(true, false, false);
  else LED(false, true, true);

  if (buttonPressedEdge()) {
    LED(false, false, false);
    setState(FlightState::ARMING);
    Serial.println(F("ARMING: press button 5 times"));
  }
}

void handleArming() {
  LED(true, true, true);

  if (buttonPressedEdge()) {
    uint32_t now = millis();
    if (now - lastArmPressMs <= 300) {
      armPressCount++;
    } else {
      armPressCount = 1;
    }
    lastArmPressMs = now;

    beep(380 + 40 * armPressCount, 70);
    Serial.print(F("ARM presses: "));
    Serial.println(armPressCount);

    if (armPressCount >= 5) {
      capturePadReference();
      resetAdaptiveState();
      countdownLastRemaining = -1;
      countdownMotionStartMs = 0;
      setState(FlightState::COUNTDOWN);
      armPressCount = 0;
      createUniqueLogFile();
    }
  }
}

void handleCountdown() {
  if (buttonPressedEdge()) {
    abortFlight(AbortReason::MANUAL_ABORT, F("countdown manual abort"));
    return;
  }

  float tiltErrorDeg = fabsf(tm.theta_rad - padAngleX) * 180.0f / M_PI;
  float rateDps = fabsf(tm.q_meas) * 180.0f / M_PI;
  if ((tiltErrorDeg > COUNTDOWN_TILT_ABORT_DEG) || (rateDps > COUNTDOWN_RATE_ABORT_DPS)) {
    if (countdownMotionStartMs == 0) {
      countdownMotionStartMs = millis();
    } else if (millis() - countdownMotionStartMs >= COUNTDOWN_MOTION_CONFIRM_MS) {
      abortFlight(AbortReason::COUNTDOWN_MOVED, F("countdown tilt/rate abort"));
      return;
    }
  } else {
    countdownMotionStartMs = 0;
  }

  uint32_t elapsed = millis() - stateStartMs;
  int remaining = (int)COUNTDOWN_S - (int)(elapsed / 1000UL);

  if (remaining != countdownLastRemaining && remaining >= 0) {
    countdownLastRemaining = remaining;
    Serial.print(F("T-"));
    Serial.println(remaining);

    if (remaining > 3) {
      LED(true, false, false);
      beep(440, 120);
    } else {
      LED(true, false, true);
      beep(880, 120);
    }
    LED(false, false, false);
  }

  if (elapsed >= COUNTDOWN_S * 1000UL) {
    // Launch sequence
    tm.initialAlt = bmp.readAltitude(SEA_LEVEL_HPA);
    tm.highestAlt = 0.0f;
    tm.vertVel = 0.0f;

    resetAdaptiveState();
    capturePadReference();
    countdownLastRemaining = -1;
    countdownMotionStartMs = 0;

    triggerPyro(P3);
    ignitionMs = millis();
    inFlight = true;

    setState(FlightState::POWERED_FLIGHT);
  }
}

void handlePoweredFlight() {
  LED(true, true, false);
  safetyChecks();
  checkAdaptiveEstimateValidity();

  if (millis() - ignitionMs >= (uint32_t)(BURN_TIME_S * 1000.0f)) {
    // Transition to coast: reset adaptive state and prepare apogee detection
    apogee_detected = false;
    coast_peak_alt = tm.altitude;
    coast_start_ms = millis();
    
    resetAdaptiveState();
    neutralServos();
    setState(FlightState::COAST);
  }
}

void handleCoast() {
  LED(false, true, false);
  safetyChecks();
  apogeeDetectionCheck();

  // Transition to deploy when apogee is detected and altitude drops below peak
  if (apogee_detected && (tm.altitude < tm.highestAlt - 1.0f) && (millis() - ignitionMs > 1500)) {
    setState(FlightState::DEPLOY);
  }
}

void handleDeploy() {
  LED(false, true, true);
  triggerPyro(P4);
  triggerPyro(P1);
  setState(FlightState::LANDED);
}

void handleLanded() {
  inFlight = false;
  neutralServos();
  LED(true, true, true);

  static uint32_t lastBeep = 0;
  if (millis() - lastBeep > 2000) {
    lastBeep = millis();
    beep(523, 80);
    beep(392, 80);
  }

  if (buttonPressedEdge()) {
    if (logFile) {
      logFile.flush();
      logFile.close();
    }
    resetAdaptiveState();
    abortReason = AbortReason::NONE;
    setState(FlightState::PAD_ALIGN);
  }
}

void handleAborted() {
  inFlight = false;
  neutralServos();
  LED(true, false, false);

  static uint32_t lastBeep = 0;
  if (millis() - lastBeep > 250) {
    lastBeep = millis();
    beep(700, 80);
  }

  if (buttonPressedEdge()) {
    if (logFile) {
      logFile.flush();
      logFile.close();
    }
    resetAdaptiveState();
    abortReason = AbortReason::NONE;
    setState(FlightState::PAD_ALIGN);
  }
}

// ==================== SETUP/LOOP ====================

void setup() {
  analogReadResolution(12);

  pinMode(BUTTON, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  pinMode(P1, OUTPUT);
  pinMode(P2, OUTPUT);
  pinMode(P3, OUTPUT);
  pinMode(P4, OUTPUT);
  if (SERVO_X_FB_MODE == FeedbackMode::ANALOG_ADC && SERVO_X_FB_PIN >= 0) {
    pinMode(SERVO_X_FB_PIN, INPUT);
  }

  digitalWrite(P1, LOW);
  digitalWrite(P2, LOW);
  digitalWrite(P3, LOW);
  digitalWrite(P4, LOW);

  LED(false, false, false);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== SISYPHUS (JOINT_ADAPTIVE) ==="));

  bool sensorInitOk = initSensors();
  bool sdOk = SD.begin(SD_CS);

  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  neutralServos();

  if (!sensorInitOk) {
    Serial.println(F("Init failed: sensors"));
    LED(true, false, false);
    while (true) {
      updatePyros();
      beep(700, 120);
      delay(200);
    }
  }

  if (!sdOk) {
    Serial.println(F("Warning: SD init failed"));
  }

  if (SERVO_X_FB_MODE == FeedbackMode::OBSERVER) {
    Serial.println(F("ACT: observer-only actuator estimate enabled (no direct feedback)"));
  } else if (SERVO_X_FB_MODE == FeedbackMode::ANALOG_ADC) {
    Serial.println(F("ACT: analog servo feedback enabled; verify ADC and degree calibration before flight"));
  } else {
    Serial.println(F("ACT: AS5600 I2C feedback enabled; verify neutral zero and direction before flight"));
  }

  beep(523, 70);
  beep(659, 70);
  beep(784, 120);

  lastSensorMs = millis();
  lastControlMs = millis();
  lastLogMs = millis();

  setState(FlightState::PAD_ALIGN);
}

void loop() {
  uint32_t now = millis();

  // Sensor scheduler (200 Hz)
  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    updateSensors();
    updateActuatorMeasurement(DT_S);
  }

  updatePyros();

  // State machine
  switch (state) {
    case FlightState::PAD_ALIGN:
      handlePadAlign();
      break;

    case FlightState::ARMING:
      handleArming();
      break;

    case FlightState::COUNTDOWN:
      handleCountdown();
      break;

    case FlightState::POWERED_FLIGHT: {
      // Control scheduler (100 Hz)
      if (now - lastControlMs >= CONTROL_PERIOD_MS) {
        lastControlMs = now;
        runJointAdaptive();
      }
      handlePoweredFlight();
      break;
    }

    case FlightState::COAST:
      handleCoast();
      break;

    case FlightState::DEPLOY:
      handleDeploy();
      break;

    case FlightState::LANDED:
      handleLanded();
      break;

    case FlightState::ABORTED:
      handleAborted();
      break;

    default:
      setState(FlightState::PAD_ALIGN);
      break;
  }

  // Logging scheduler (20 Hz during flight)
  if (inFlight && (now - lastLogMs >= 50)) {
    lastLogMs = now;
    logDataRow();
  }

  if (AS5600_BENCH_PRINT && SERVO_X_FB_MODE == FeedbackMode::AS5600_I2C && !inFlight && (now - as5600LastBenchPrintMs >= AS5600_BENCH_PRINT_MS)) {
    as5600LastBenchPrintMs = now;
    float uMeasured = 0.0f;
    bool ok = readAS5600Feedback(uMeasured);
    Serial.print(F("AS5600: ok="));
    Serial.print(ok ? 1 : 0);
    Serial.print(F(" raw="));
    Serial.print(as5600LastRaw);
    Serial.print(F(" gimbal_deg="));
    Serial.print(as5600LastDeg, 2);
    Serial.print(F(" u_act="));
    Serial.println(uMeasured, 3);
  }
}
