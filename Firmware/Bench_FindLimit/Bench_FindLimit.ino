/*
  Bench_FindLimit.ino — MEASURE the gimbal's true mechanical travel, per axis, per direction.
  ==========================================================================================
  Bench_ThrowCheck answers "is MAX_TILT safe, yes/no". This answers the better question:
  *how far does the gimbal actually go before it binds?* Knowing the real number lets MAX_TILT be
  set with a stated margin instead of a guess, and it is the last unmeasured item in the actuator
  chain (linkage ratio and sign are now measured; travel is not).

  HOW IT WORKS
  Steps out in 0.25 deg increments from a known-safe 3.50 deg to 6.00 deg. At each level it holds
  each of X+, X-, Y+, Y- for HOLD_MS with a CONSTANT command, re-sent at the flight loop rate, and
  returns to neutral between. A healthy servo is SILENT while holding. The first level where you
  hear a sustained buzz/growl is that axis' mechanical limit.

  *** WATCH FOR ASYMMETRY. *** X+ and X- need not bind at the same angle, and X need not match Y.
  The binding limit is the SMALLEST of the four. Note each one separately -- an asymmetric limit
  means the horn is indexed off-centre, which is worth fixing mechanically rather than designing
  around, because it wastes travel on one side while the other side has spare.

  READING IT
    - Run with the Serial monitor open at 115200 if you can: it prints the exact gimbal degrees and
      servo degrees for every hold, so you can write down the number rather than estimate it.
    - Without serial it still runs; the deflection grows visibly step by step and it pauses longer
      between levels. Cut power at the first sustained buzz.
    - Deliberately NO buzzer/tone anywhere: the whole measurement is listening to the servos.

  SAFETY
    - MOTOR OUT. Pyro leads DISCONNECTED. This sketch never touches a pyro pin, but do it anyway.
    - CHARGED pack. A flat pack makes a healthy servo stall and will fake a bind (that happened on
      2026-08-03). If a bench supply with a current readout is available, use it -- a stalled servo
      pulls ~1-1.5 A and a hunting one ~100-200 mA, which is a far cleaner signal than sound.
    - Each hold is short and returns to neutral, so a bind cannot cook the servo while you react.
      Cut power at the first buzz; do not let it sit loaded.
    - It STOPS at END_DEG and parks at neutral. It does not loop, so it cannot creep past the limit
      unattended.

  AFTER
    Set Ascent_TVC.ino MAX_TILT = (smallest of the four limits) - 0.5 deg of margin, and set
    Bench_ThrowCheck to the same value to confirm. Record the four numbers in FLIGHT_LOG.md.
*/
#include <PWMServo.h>

// ---- MUST MATCH Ascent_TVC.ino ----
constexpr int   SERVO_X_PIN = 4, SERVO_Y_PIN = 3;
constexpr float XTUNE = -11.0f, YTUNE = +5.0f;   // MEASURED 2026-08-06 rebuild -- must match Ascent_TVC.ino
constexpr int   SERVO_X_SIGN = -1, SERVO_Y_SIGN = -1;
constexpr float SERVO_X_MULT = 4.5f, SERVO_Y_MULT = 4.5f;   // = the physical linkage, so TVC deg == gimbal deg

// ---- sweep definition ----
constexpr float START_DEG = 3.50f;   // known-safe starting deflection
constexpr float END_DEG   = 6.00f;   // hard ceiling -- never commands past this
constexpr float STEP_DEG  = 0.25f;   // resolution of the answer
constexpr int   STEP_DELAY_MS = 3;   // flight loop rate, so the 1-deg command staircase blurs as it does in flight
constexpr int   RAMP_STEPS    = 250;
constexpr int   HOLD_MS       = 900;  // the listening window at each level
constexpr int   NEUTRAL_MS    = 500;
constexpr int   LEVEL_PAUSE_MS= 1400; // longer pause between LEVELS so they are countable without serial

PWMServo servoX, servoY;

void writeServos(float sx, float sy){          // identical in form to the flight firmware
  servoX.write((int)constrain(SERVO_X_SIGN*sx + 90 + XTUNE, 0, 180));
  servoY.write((int)constrain(SERVO_Y_SIGN*sy + 90 + YTUNE, 0, 180));
}
int pulseUs(int deg){ return 544 + (int)((long)deg * (2400 - 544) / 180); }

// hold one axis/direction at `gimbalDeg` and listen
void probe(const char* label, bool isX, float gimbalDeg){
  float mult = isX ? SERVO_X_MULT : SERVO_Y_MULT;
  int   sign = isX ? SERVO_X_SIGN : SERVO_Y_SIGN;
  float servoOff = gimbalDeg * mult;
  int   cmd = (int)constrain(sign*servoOff + 90 + (isX?XTUNE:YTUNE), 0, 180);
  Serial.print(F("    ")); Serial.print(label);
  Serial.print(F("  gimbal ")); Serial.print(gimbalDeg,2);
  Serial.print(F(" deg -> servo offset ")); Serial.print(servoOff,2);
  Serial.print(F(" -> write(")); Serial.print(cmd);
  Serial.print(F(") = ")); Serial.print(pulseUs(cmd)); Serial.println(F(" us   [listen: silence = OK]"));

  for(int i=1;i<=RAMP_STEPS;i++){                       // ease out
    float t = servoOff*(float)i/RAMP_STEPS;
    if(isX) writeServos(t,0); else writeServos(0,t);
    delay(STEP_DELAY_MS);
  }
  for(unsigned long t0=millis(); millis()-t0 < HOLD_MS; ){   // CONSTANT command -- the measurement
    if(isX) writeServos(servoOff,0); else writeServos(0,servoOff);
    delay(STEP_DELAY_MS);
  }
  for(int i=RAMP_STEPS;i>=0;i--){                        // ease back, never leave it loaded
    float t = servoOff*(float)i/RAMP_STEPS;
    if(isX) writeServos(t,0); else writeServos(0,t);
    delay(STEP_DELAY_MS);
  }
  writeServos(0,0);
  delay(NEUTRAL_MS);
}

bool done = false;

void setup(){
  servoX.attach(SERVO_X_PIN); servoY.attach(SERVO_Y_PIN);   // default 544-2400 us, exactly as the flight code
  writeServos(0,0);
  Serial.begin(115200);
  unsigned long t0=millis(); while(!Serial && millis()-t0<1500);   // never blocks: runs with or without a monitor
  Serial.println();
  Serial.println(F("=== Bench_FindLimit: measuring true gimbal travel ==="));
  Serial.print(F("linkage ")); Serial.print(SERVO_X_MULT,2); Serial.println(F(":1  (commanded deg == gimbal deg)"));
  Serial.print(F("sweeping ")); Serial.print(START_DEG,2); Serial.print(F(" -> "));
  Serial.print(END_DEG,2); Serial.print(F(" deg in ")); Serial.print(STEP_DEG,2); Serial.println(F(" deg steps"));
  Serial.println(F("MOTOR OUT? PYROS DISCONNECTED? CHARGED PACK? Gimbal centred at neutral?"));
  Serial.println(F("Silence during each HOLD = fine. First sustained buzz = that axis' limit. CUT POWER THEN."));
  Serial.println(F("Starting in 4 s."));
  delay(4000);
}

void loop(){
  if(done){ writeServos(0,0); delay(500); return; }        // parked; will not creep past the limit unattended
  for(float g = START_DEG; g <= END_DEG + 1e-4f; g += STEP_DEG){
    Serial.print(F("  ---- level ")); Serial.print(g,2); Serial.println(F(" deg gimbal ----"));
    probe("X+", true,   g);
    probe("X-", true,  -g);
    probe("Y+", false,  g);
    probe("Y-", false, -g);
    delay(LEVEL_PAUSE_MS);
  }
  writeServos(0,0);
  Serial.println(F("=== reached the ceiling without stopping ==="));
  Serial.print(F("If nothing buzzed, travel exceeds ")); Serial.print(END_DEG,2);
  Serial.println(F(" deg on all four -- set MAX_TILT to 5.0 and keep the margin."));
  Serial.println(F("Parked at neutral. Power off."));
  done = true;
}
