/*
  Bench_ThrowCheck.ino — does the gimbal reach FULL flight deflection without binding?
  ====================================================================================
  Answers exactly one question: when Ascent_TVC.ino commands its maximum TVC angle, does either
  axis hit a mechanical stop and stall? A stalled hobby servo pulls ~1-1.5 A, which on this vehicle
  is enough to sag the rail and brown out the flight computer -- the leading suspect for ASC036's
  mid-flight logging loss, and for the reboot-stall seen on the bench 2026-08-03.

  *** WHY NOT gimbal_characterization.ino / feedback_servo_calibration.ino / Bench_Latency.ino ***
  They under-test the throw and would give a false pass. NOTE (2026-08-04): reason 1 below is now
  VOID -- the linkage was corrected to 1:4.5 and those sketches were updated from 4.0 to 4.5, so
  their "5 deg gimbal" sweep now commands the same +-22.5 servo deg the flight firmware does.
  Reason 2 is INDEPENDENT of the linkage and still stands on its own, so this sketch is still the
  one to trust for a throw check.
    1. [VOID since 2026-08-04] They carried LINKAGE_SERVO_DEG_PER_GIMBAL_DEG = 4.0 against a
       believed-5.5 linkage, so their "5 deg gimbal" sweep commanded +-20 servo deg rather than
       the flight's throw. Both numbers have since been corrected to 4.5 and now agree.
    2. They use USE_MICROSECOND_COMMANDS with a 1000-2000 us map over 0-180 deg. PWMServo -- which
       the flight firmware actually uses -- defaults to 544-2400 us (PWMServo.h:90). Same "degrees",
       different pulse. Flight neutral = 1472 us and full throw = 1755 us (+283); that sketch reaches
       only 1611 us (+111).
  This sketch instead uses PWMServo with the default attach, the same pins, and the same constants as
  the flight firmware, so the pulse train the servo sees is bit-for-bit what it will see in flight.

  SAFETY BEFORE RUNNING
    - MOTOR OUT. Pyro leads DISCONNECTED. This sketch never touches a pyro pin, but do it anyway.
    - Use a CHARGED pack (or a bench supply with a current display -- the current is the real answer).
    - WATCH and LISTEN. A bind is audible: buzzing/growling instead of a clean move-and-stop.
    - It holds each extreme only briefly and returns to neutral, so a bind cannot cook the servo while
      you decide. If you hear buzzing, cut power -- do not let it sit there.

  READING THE RESULT
    Clean  -> servo moves, stops, goes quiet at the limit. Both axes, both directions. You are good.
    Bind   -> buzzing/growling at or before the limit, or the horn visibly stops short and strains.
              FIX (revised 2026-08-04): do NOT lower SERVO_*_MULT -- it now equals the physical
              1:4.5 linkage, so dropping it would desync commanded from true gimbal degrees and
              silently corrupt every logged TVC angle. Lower MAX_TILT instead: it costs the same
              authority but keeps the units 1:1. History: 5.5 buzzed (6.1 deg of gimbal), 5.0
              buzzed (5.56 deg), 4.5 sits exactly on the 5 deg stop.
              *** BUT DO NOT KEEP WALKING IT DOWN WITHOUT CHECKING WHY. *** Every step costs real
              control authority, and authority is what lost ASC007. At MULT 4.5 the SIL aborts at
              4.0 deg of thrust misalignment -- ⚠ STALE, that SIL run used SERVO_MULT = 5.5 and must
              be recomputed now the harness is at 4.5; true authority is +-5.00 deg, not 4.09, so the
              real abort threshold is HIGHER than 4.0. ASC007's measured misalignment was 4.28 deg. Before
              dropping further, confirm the noise is a genuine STALL and not normal servo hunting:
              a stalled servo draws ~1-1.5 A and gets hot within seconds; a hunting one draws
              ~100-200 mA and stays cool. If it is a real stall, the right fix is MECHANICAL --
              open up the gimbal's travel or re-index the horn -- not more MULT reduction.
              Do NOT fly a binding axis.
    Also check NEUTRAL: if the gimbal is not centred at 90 deg, a horn is mis-indexed by a spline --
    that alone can stall a servo at rest and is worth fixing mechanically before trusting either MULT.
*/
#include <PWMServo.h>

// ---- MUST MATCH Ascent_TVC.ino EXACTLY. If you change them there, change them here. ----
constexpr int   SERVO_X_PIN = 4, SERVO_Y_PIN = 3;
constexpr float XTUNE = -11.0f, YTUNE = +5.0f;   // MEASURED 2026-08-06 rebuild -- must match Ascent_TVC.ino
constexpr int   SERVO_X_SIGN = -1, SERVO_Y_SIGN = -1;      // bench-set 2026-08-03
constexpr float SERVO_X_MULT = 4.5f, SERVO_Y_MULT = 4.5f;  // MUST MATCH Ascent_TVC.ino.
// CORRECTED 2026-08-04: this now EQUALS the physical linkage. The 1:5.5 measured 2026-08-03 was a
// mis-measurement; the true ratio is 1:4.5 (Braxton). So +-22.5 servo deg is +-5.00 deg of ACTUAL
// gimbal -- NOT the 4.09 deg this comment used to claim. The earlier buzzing is now explained:
// commanding 5.5 against a true 4.5 linkage drove 6.1 deg of gimbal, and 5.0 drove 5.56 deg, both
// past a +-5 deg stop. 4.5 lands exactly ON the 5 deg stop, so re-run to confirm it is silent --
// there is no margin left for mechanical tolerance.
constexpr float MAX_TILT     = 5.0f;   // TVC deg, the firmware's clamp -- MUST track Ascent_TVC.ino.
// Raised 4.5 -> 5.0 on 2026-08-04 after Bench_FindLimit MEASURED the travel: 5.00 deg held silent on all
// four (X+/X-/Y+/Y-), 5.25 deg buzzed. So the mechanical limit is in [5.00, 5.25) and MAX_TILT 5.0 is
// validated by direct test rather than assumed. Note the (int) truncation is asymmetric -- MAX_TILT 5.0
// writes 112 (+dir, 4.89 deg gimbal) and 67 (-dir, 5.11 deg) -- and Bench_FindLimit uses the identical
// expression, so it exercised exactly these two integers. There is no untested margin between them.

// Command the servo at the FLIGHT loop rate. PWMServo::write() takes INTEGER degrees only (there is no
// writeMicroseconds in this library), so every command is quantised to 1 servo deg = ~10.3 us = ~0.18 deg
// of TVC at the 4.5:1 linkage (1/4.5 = 0.222 deg per servo deg). The flight firmware carries exactly the same quantisation. That staircase is
// audible as a series of ticks if you step slowly -- so step at the rate the flight loop actually uses
// (~1-3 ms) and the steps blur into continuous motion, as they will in flight. An earlier version of this
// sketch stepped every 45 ms and sounded rough for that reason alone; that was a test artifact, not a fault.
constexpr int  STEP_DELAY_MS = 3;     // flight loop is ~1-3 ms; this makes the command stream flight-like
constexpr int  RAMP_STEPS    = 400;   // ~1.2 s of travel at 3 ms/step
constexpr int  HOLD_MS       = 1200;  // dwell at the extreme -- THE listening window (see below)
constexpr int  SETTLE_MS     = 700;

PWMServo servoX, servoY;

// the flight firmware's writeServos(), verbatim in form
void writeServos(float sx, float sy){
  servoX.write((int)constrain(SERVO_X_SIGN*sx + 90 + XTUNE, 0, 180));
  servoY.write((int)constrain(SERVO_Y_SIGN*sy + 90 + YTUNE, 0, 180));
}
// what PWMServo's default attach(pin) actually emits, so the log is checkable against a scope
int pulseUs(int deg){ return 544 + (int)((long)deg * (2400 - 544) / 180); }

void rampAxis(const char* name, bool isX, float tvcTarget){
  float mult = isX ? SERVO_X_MULT : SERVO_Y_MULT;
  int   sign = isX ? SERVO_X_SIGN : SERVO_Y_SIGN;
  int   endServoDeg = (int)constrain(sign*(tvcTarget*mult) + 90 + (isX?XTUNE:YTUNE), 0, 180);
  Serial.print("  "); Serial.print(name);
  Serial.print(" -> TVC "); Serial.print(tvcTarget,2);
  Serial.print(" deg = servo offset "); Serial.print(tvcTarget*mult,2);
  Serial.print(" deg -> write("); Serial.print(endServoDeg);
  Serial.print(") = "); Serial.print(pulseUs(endServoDeg)); Serial.println(" us");

  const int N = RAMP_STEPS;
  for(int i=1;i<=N;i++){
    float f = (float)i/N, t = tvcTarget*f*mult;
    if(isX) writeServos(t, 0); else writeServos(0, t);
    delay(STEP_DELAY_MS);
  }
  // ---- THE measurement. The command is now CONSTANT at the flight limit, and it is re-sent every
  // STEP_DELAY_MS exactly as the flight loop does. A healthy servo goes SILENT here. Any sustained
  // buzz/growl/hunt with a fixed command means it is fighting a mechanical stop and drawing stall
  // current -- that is the failure this whole test exists to find. Noise while MOVING is the 1-deg
  // command staircase and is benign; noise while HOLDING is not.
  Serial.println(F("      HOLDING at the limit -- should be SILENT. Buzzing here = BIND."));
  for(unsigned long t0=millis(); millis()-t0 < HOLD_MS; ){
    if(isX) writeServos(tvcTarget*mult, 0); else writeServos(0, tvcTarget*mult);
    delay(STEP_DELAY_MS);
  }
  for(int i=N;i>=0;i--){                // back out, so nothing is left loaded against a stop
    float f = (float)i/N, t = tvcTarget*f*mult;
    if(isX) writeServos(t, 0); else writeServos(0, t);
    delay(STEP_DELAY_MS);
  }
  writeServos(0,0);
  Serial.println(F("      back at neutral -- should also be silent here."));
  delay(SETTLE_MS);
}

void setup(){
  servoX.attach(SERVO_X_PIN); servoY.attach(SERVO_Y_PIN);   // DEFAULT 544-2400 us, exactly as the flight code
  writeServos(0,0);
  Serial.begin(115200);
  // No blocking wait and no key press: this runs STANDALONE on a battery with no laptop attached, which is
  // how it gets used at the field. Serial output still happens if a monitor is connected -- it is just never
  // waited on. Deliberately no buzzer/tone either: the whole test is listening to the SERVOS.
  unsigned long t0=millis(); while(!Serial && millis()-t0<1500);
  Serial.println();
  Serial.println(F("=== Bench_ThrowCheck: full flight-deflection bind test (free-running) ==="));
  Serial.print(F("MAX_TILT ")); Serial.print(MAX_TILT,2);
  Serial.print(F(" TVC deg x MULT ")); Serial.print(SERVO_X_MULT,2);
  Serial.print(F(" = +-")); Serial.print(MAX_TILT*SERVO_X_MULT,1); Serial.println(F(" servo deg"));
  Serial.print(F("neutral write(90) = ")); Serial.print(pulseUs(90)); Serial.println(F(" us"));
  Serial.println(F("MOTOR OUT? PYROS DISCONNECTED? Gimbal CENTRED at neutral?"));
  Serial.println(F("Starting in 3 s, then cycling X+ X- Y+ Y- forever. Power off to stop."));
  delay(3000);                      // hands and ears clear before it moves
}

void loop(){
  Serial.println(F("\n-- X axis --"));
  rampAxis("X +", true,  MAX_TILT);
  rampAxis("X -", true, -MAX_TILT);
  Serial.println(F("-- Y axis --"));
  rampAxis("Y +", false,  MAX_TILT);
  rampAxis("Y -", false, -MAX_TILT);
  Serial.println(F("Cycle complete, both axes, both directions, at FULL flight throw."));
  Serial.println(F("Silent during every HOLD = pass. Repeating in 2 s (power off to stop)."));
  delay(2000);
}
