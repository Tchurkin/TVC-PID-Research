/*
  Ascent_TVC.cpp — dedicated pure-TVC ASCENT-TEST firmware (Teensy 4.1)
  =====================================================================
  Mission: vertical thrust-vectored boost on ONE motor, then PARACHUTE recovery
  at apogee. NO landing burn, NO margin fins. LANDING LEGS deploy at apogee so the
  touchdown loads the legs, not the TVC gimbal/nozzle.

  Derived from Ascent_Test.cpp (the previously-flown ascent controller): same simple
  per-axis PD on tilt + rate, +/-5 deg TVC, accel tilt reference on the pad. ATTITUDE IN
  FLIGHT IS NOW A ROLL-AWARE QUATERNION (2026-07-13) -- was naive body-rate integration,
  which inverted its own correction near ~180 deg roll and drove the ASC007/ASC031 roll-
  coupling divergence. Still a per-axis PD (not the full ADRC stack in Sysiphus_Landing).

  Vehicle (MEASURED 2026-08-03, POST-ASC036 REBUILD -- ASC036 destroyed the previous airframe):
    mass 0.927 kg  |  Iyy 0.0176 kg*m^2 (bifilar)  |  CG 0.16 m forward of the TVC pivot  |  linkage 4.5:1
    (linkage CORRECTED 2026-08-04: the 5.5:1 measured 2026-08-03 was a mis-measurement.)
    Datum convention: the TVC rotation point. So the CG position and the control moment arm L are the SAME
    number by construction -- 0.16 m. Config: legs folded, ascent. [CONFIRM] motor loaded when weighed.
  The PREVIOUS airframe (flew ASC007/031/036) was: Iyy 0.0078, mass 0.818 kg, L 0.14 m, gimbal 5:1. Those
  numbers describe a DIFFERENT, lighter, longer rocket and must not be reused.
    keff = T*L/Iyy = 14.34*0.16/0.0176 = 130 rad/s^2 per rad TVC  (was 257 on the old airframe -- Iyy went
    up 2.26x but L went up 1.14x, so keff halved rather than dropping 2.3x).
  GAINS: POLE-PLACED (P=0.249, D=0.062) for keff = T*L/IYY = 257, target zeta=1.0 / wn=8 rad/s;
  closed-loop SIL-validated (0.6 s settle, 0% overshoot, robust across the burn + a slow servo).

  ⚠ THE GAINS ARE STILL SIZED FOR THE OLD AIRFRAME (flagged 2026-08-04, researcher). They were
  pole-placed against keff = 257; the REBUILT vehicle is keff = 130. Holding P=0.249 / D=0.062 at
  keff = 130 gives wn = sqrt(0.249*130) = 5.7 rad/s (not 8, i.e. 29% slower) and zeta =
  0.062*130/(2*5.7) = 0.71 (not 1.0, so it now overshoots ~4%). To restore the design target at the
  measured airframe: P = wn^2/keff = 64/130 = 0.492, D = 2*zeta*wn/keff = 16/130 = 0.123.
  NOTE both scale by the SAME factor (1.98x), so the P/D RATIO is unchanged -- which is the safe way
  to retune: the sim work found that changing P without changing D by the same factor is what makes
  gains fail (failure 70% vs 1.3% at high authority-delay). Do NOT apply this without a SIL run and
  the bench pitch-the-nose check; it is flagged here, not adopted.
  This is MORE active than the old flown 0.08/0.08 (which was overdamped) -- if it looks TOO hot
  on the bench, reduce P_GAIN first. Validation = this design + heritage + the BENCH CHECKS below.

  *** CRITICAL PRE-FLIGHT BENCH CHECK -- DO NOT SKIP ***
    Power on and leave it in the pre-arm state (accel tilt reference live). Physically
    pitch the NOSE one way: the gimbal MUST deflect so its thrust line pushes the nose
    BACK upright -- on BOTH axes. If an axis drives the wrong way, flip that axis'
    SERVO_*_SIGN below (or swap the linkage). A wrong sign = guaranteed divergence at
    launch. Then confirm the servos CENTER the nozzle straight at 90 deg and the
    +/-range does not bind. Verify the pin map and servo pins match YOUR wiring.
*/
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PWMServo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <EEPROM.h>

// -- Types --------------------------------------------------------------------
// Defined before ALL functions: the Arduino IDE auto-generates function prototypes and HOISTS them above the
// first function, so a prototype taking 'KF&' (kfInit/kfPredict/kfUpdate) would otherwise reference KF before
// it is declared -> "'KF' was not declared in this scope". (The SIL builds .ino as plain C++ with no hoisting,
// so it did not catch this -- verify in the Arduino IDE too.)
struct KF{ float s[3]; float P[9]; };   // vertical Kalman: state s=[pos,vel,accel-bias], covariance P (3x3)

// Brownout-persistent flight state (EEPROM). Declared HERE, at the top, for the same Arduino-IDE
// prototype-hoisting reason as KF above. Holds only what CANNOT be re-derived after an in-flight reboot:
// the pad gyro-bias calibration (it takes 1.2 s of stillness to measure -- impossible in flight) and the
// ground barometric reference (without it, "altitude" is meaningless). Everything else is re-read live.
struct Persist{
  uint32_t magic;                 // validity marker
  uint8_t  phase;                 // 0=idle 1=boost 2=coast 3=chute deployed
  uint16_t boots;                 // reboots seen inside ONE flight -- caps a brownout->deploy->brownout loop
  float    gbx,gby,gbz;           // rad/s, pad gyro bias
  float    groundAlt;             // m, pad barometric reference
  uint32_t sum;                   // integrity check over the fields above
};

// -- Pins (verify against your wiring) ----------------------------------------
constexpr int BUTTON = 14;
constexpr int BUZZER = 13;
constexpr int P1     = 9;    // LANDING LEGS deploy (melt wire) -- protects the TVC on touchdown
constexpr int P2     = 10;   // not connected
constexpr int P3     = 11;   // ASCENT motor ignition
constexpr int P4     = 12;   // PARACHUTE melt wire (primary recovery)
constexpr int RLED   = 6, GLED = 7, BLED = 8;
constexpr int SD_CS  = BUILTIN_SDCARD;
constexpr int SERVO_X_PIN = 4, SERVO_Y_PIN = 3;

// -- TVC / control (PD; identical form to the flown Ascent_Test) ---------------
constexpr float XTUNE = 0, YTUNE = 0;         // servo neutral trim (deg) -- set so 90+trim points the nozzle straight
constexpr int   SERVO_X_SIGN = -1;            // *** SET FROM THE BENCH, 2026-08-03: with +1 BOTH axes drove the nose the WRONG
constexpr int   SERVO_Y_SIGN = -1;            // way (pitch-the-nose test), so both were flipped to -1 and the test re-run. ***
                                              // The bench test is the ONLY authority for these two constants. Everything below is
                                              // context for why, not a reason to override it.
                                              //
                                              // ⚠ UNEXPLAINED, RESOLVE BEFORE TRUSTING THE AIRFRAME: flight ASC036 (2026-07-20)
                                              // flew +1/+1 on THIS SAME Adafruit_MPU6050 driver and held attitude cleanly (boost
                                              // tilt 8.7 deg, ZERO gimbal saturation). Same code, same driver, opposite bench
                                              // result today => something MECHANICAL changed after ASC036 -- linkage rebuilt to its
                                              // original sense, IMU remounted/rotated, or a servo re-horned/swapped. If the airframe
                                              // was rebuilt, ASC036's measured 0.66 deg thrust misalignment does NOT carry over, and
                                              // that number is what separated ASC036's clean flight from ASC007's abort. Re-check
                                              // alignment, mass, Iyy and the CG->TVC arm before flying. See FLIGHT_LOG.md.
                                              //
                                              // The previous comment here justified +1 by citing flight ASC031. That justification
                                              // was WRONG and has been removed: ASC031 flew the pre-2026-07-07-pm firmware, proven
                                              // twice from its own log (axial railed at exactly 2.000 g = the old tockn driver's
                                              // +-2 g FSR, and its TVC command is pure PD to +-0.1 deg, i.e. no integral trim).
                                              // A flight flown on a DIFFERENT gyro driver cannot validate this build's sign.
                                              //
                                              // AFTER ANY RE-FLASH: re-run the bench pitch-the-nose test on BOTH axes -- the correct
                                              // sign depends on BOTH the linkage AND the IMU driver's gyro convention, and a reflash
                                              // can change the driver. Never fly a reflash without re-confirming the gimbal corrects
                                              // the nose back toward upright. A wrong sign is positive feedback = guaranteed divergence.
// PD POLE-PLACED on the measured plant (theta_ddot = keff*delta; keff = T*L/IYY = 14.34*0.14/0.0078 ~= 257 rad/s^2
// per rad TVC, L = CG->TVC-axis = 0.14 m MEASURED). Target zeta=1.0 (critically damped, no overshoot), wn=8 rad/s:
//   Kp = wn^2/keff = 64/257 = 0.249 ;  Kd = 2*zeta*wn/keff = 16/257 = 0.062
// SIL-validated: 0.6 s settle, 0% overshoot, peak servo ~15 deg (X, 8:1), robust across the F-15 burn (keff 215-520
// -> zeta 0.9-1.4) and a slow 150 deg/s servo. The (P*theta + D*rate) sum IS the TVC angle in DEG (capped at MAX_TILT);
// each axis' SERVO_*_MULT converts it to that axis' servo deg, so common P/D gives matched dynamics on both axes.
// Previously-flown fallback (zeta~3.3, overdamped/sluggish): 0.08 / 0.08.  BENCH-TEST before flight.
// *** RE-POLED 2026-08-04 onto the MEASURED plant. *** The old 0.249/0.062 were pole-placed for the DESTROYED
// airframe's keff=257. The rebuilt vehicle measures keff = T*L/Iyy = 14.34*0.16/0.0176 = 130, so those gains
// delivered wn=5.70 / zeta=0.71 -- sluggish and under-damped on THIS rocket. Scaling both by 257/130 = 1.97
// restores the original, intended design point: wn=8.0 rad/s, zeta=1.00 (critically damped, no overshoot).
// Kd/Kp is preserved at 0.25 (0.249 -> 0.251), which is the thing that matters: the researcher's 2026-08-03
// replication (n=2400) showed that moving Kd independently of Kp is what manufactures the failure effect.
// WHY THIS IS THE SAFER CHOICE, not the more aggressive one -- steady lean = misalignment/P_GAIN, so DOUBLING
// P HALVES the lean (4.02 -> 2.04 deg per deg of misalignment) and hands that authority back. Lean eating the
// gimbal is precisely what aborted ASC007. SIL on the measured airframe, N=120 Monte Carlo including the
// measured 262 ms excess ignition lag:  PASS 80.8% -> 97.5%,  MARGINAL 18.3% -> 1.7%,  boost tilt p50
// 6.80 -> 3.53 deg, p90 12.69 -> 6.44 deg. Verified robust to actuator lag out to 0.30 s (10x the assumed
// SERVO_TAU), to loaded slew down to 70 deg/s, and to gyro noise to 8 dps (53x nominal).
float P_GAIN = 0.491f, D_GAIN = 0.123f;        // NOT constexpr: the ascent SIL overrides these via extern (getenv
                                               // PGAIN/DGAIN) to adjudicate gains against the roll-coupling failure.
                                               // Flight defaults are unchanged; on hardware they behave as before.
float I_GAIN = 0.20f;                          // integral trim (anti-windup): winds out a CONSTANT thrust-misalignment
                                               // (the ASC007 mechanism) so the vehicle flies VERTICAL instead of holding a
                                               // lean near the 45 deg abort. SIL-tuned/overridable (getenv IGAIN). Does NOT
                                               // add authority -- cancelling a 4 deg misalignment still costs 4 deg of gimbal,
                                               // so ALIGN MECHANICALLY too. Set I_GAIN=0 to disable (pure PD).
float iTermX = 0, iTermY = 0;                  // integral accumulators (TVC deg), reset at launch, clamped +-MAX_TILT
constexpr float SERVO_X_MULT = 4.5f, SERVO_Y_MULT = 4.5f;   // servo deg per TVC deg, as COMMANDED by this firmware.
// THIS NOW MATCHES THE PHYSICAL LINKAGE. Corrected 2026-08-04 (Braxton): the 1:5.5 measured on the bench
// 2026-08-03 was a MIS-MEASUREMENT; the real ratio is 1:4.5 on both axes. The value here was already 4.5 --
// it had been walked down 5.5 -> 5.0 -> 4.5 because the gimbal hit its mechanical stop and buzzed at full
// throw -- so the constant does not change, but the REASON does, and so does every number derived from it.
// (The buzzing is now explained: at a commanded 5.5 against a true 4.5 linkage, MAX_TILT drove 27.5/4.5 =
// 6.1 deg of gimbal, past a +-5 deg stop.)
//
// THREE CONSEQUENCES, all revised:
//   1. AUTHORITY IS BETTER THAN WE THOUGHT. Commanding MAX_TILT (5 deg) writes 22.5 servo deg, which the true
//      4.5:1 linkage turns into 22.5/4.5 = 5.00 deg of ACTUAL gimbal. True authority is +-5.00 deg, not the
//      +-4.09 deg previously believed -- a 22% increase.
//   2. UNITS ARE NOW 1:1. Commanded TVC deg == true gimbal deg, so ASC###.CSV TVCx/TVCy and the CTL###.CSV
//      pre/post columns are TRUE degrees and need no correction. The 1.222x offline correction previously
//      required is WITHDRAWN, and keff fits are no longer 22% high. CAUTION for older logs: actual gimbal =
//      logged TVC deg x (SERVO_MULT_at_the_time / 4.5), so flights logged while MULT was 5.0 read 1.111x low
//      and 5.5 read 1.222x low. The CTL header records servo_mult_x/y, so each file is self-correcting.
//   3. ⚠ MISALIGNMENT BUDGET -- still the thing that can lose the vehicle, but it now closes. A thrust
//      misalignment must be cancelled by the gimbal 1:1, straight off the authority above. At +-5.00 deg
//      against ASC007's measured 4.28 deg misalignment there is 0.72 deg left for control -- tight but
//      positive. Under the old (wrong) 4.09 deg figure the budget was NEGATIVE, i.e. that vehicle could not
//      have cancelled its own misalignment. Still boresight the nozzle before flying; the flight measures it
//      for you, since the steady integral-trim value IS the misalignment (ASC036 read 0.66 deg that way).
//
// ⚠ REMAINING RISK -- commanded == physical means MAX_TILT now drives the gimbal to EXACTLY 5.00 deg, and the
// airframe's documented travel is +-5 deg. There is no margin against mechanical tolerance or overshoot, and
// a servo stalled against a stop pulls ~1-1.5 A and can brown out the flight computer (the leading suspect
// for ASC036's mid-flight logging loss). If the buzzing recurs on the bench at full throw, reduce MAX_TILT
// rather than SERVO_*_MULT -- that keeps commanded and physical in agreement so the logs stay 1:1.
//
// The SIL (ascent_sim.cpp SERVO_MULT) was updated to 4.5 to match; there is no longer a command/linkage
// mismatch for the harness to model.
// Sets servo THROW, not loop gain (that is P_GAIN, on the TVC angle).
// The bench sketches (Bench_Latency / gimbal_characterization / feedback_servo_calibration) were updated from
// 4.0 to 4.5 at the same time, so all four files now agree. Any gimbal-deg figure produced by those sketches
// BEFORE 2026-08-04 is stale.
constexpr float MAX_TILT = 4.5f;              // deg, TVC command limit BEFORE the linkage multiply.
// LOWERED 5.0 -> 4.5 on 2026-08-04 to keep the servo OFF its mechanical stop. With SERVO_*_MULT = 4.5
// matching the physical linkage, a command of MAX_TILT writes MAX_TILT*4.5 servo deg, so:
//     MAX_TILT 5.0 -> 22.5 servo deg -> 5.00 deg gimbal == EXACTLY the stop, zero tolerance margin
//     MAX_TILT 4.5 -> 20.25 servo deg -> 4.50 deg gimbal == 0.5 deg INSIDE the stop
// That 0.5 deg matters more than it looks. SIL, tip-off + gust on the measured airframe: the controller
// commands past its clamp for ~20% of the burn. At MAX_TILT 5.0 that is 20% of the burn with the servo
// PINNED ON THE STOP drawing ~1-1.5 A stall current -- the exact rail sag suspected of browning out the
// flight computer on ASC036. At 4.5 the clamp holds the servo 0.5 deg short of the stop, so it saturates
// electrically but never mechanically: same control behaviour, zero stall exposure.
// COST: none that shows up. The abort threshold against thrust misalignment is UNCHANGED at ~5 deg,
// because the re-poled P_GAIN keeps the vehicle upright well enough to offset the smaller clamp
// (MARGINAL at 4.5 deg misalign, ABORT at 5.0 deg -- identical to the old 0.249/MAX_TILT 5.0 pairing).
// If the gimbal's true travel is later opened up mechanically, raise this back toward 5.0.

// -- Motor / flight constants --------------------------------------------------
constexpr float AV_THRUST     = 14.34f;       // N, average ascent thrust (F-15) -- reference / apogee-timing check
constexpr float BURN_TIME     = 3.45f;        // s, ascent motor burn (F-15) -- burnout at IGN_DELAY+BURN_TIME
constexpr float IGN_DELAY     = 0.2f;         // s, ASSUMED ignition lag -- never yet measured on this vehicle
constexpr float TVC_EXTEND_S  = 1.0f;         // s, keep VECTORING this long past the nominal burnout.
// WHY: the TVC window is a fixed 3.65 s from the ignition COMMAND, but the real ignition lag is unmeasured
// (the only MTR###.CSV on record is from a no-ignition run, so IGN_DELAY=0.2 is an assumption). If the motor
// lights late, or its tail-off runs long, thrust is STILL ON when the gimbal centres -- and a centred gimbal
// on a vehicle whose thrust axis does not pass exactly through the CG applies an uncontrolled trim torque
// T*L*misalign, which is precisely the ASC007 divergence mechanism, now with NO controller opposing it.
// COST IF THE MOTOR IS ALREADY OUT: none. Control torque is T*L*delta, so with T=0 the gimbal has zero
// authority -- it cannot help or hurt. That asymmetry (helps when thrust persists, does nothing when it
// does not) is why a fixed extension is preferred to trying to detect burnout and stop early: a detector
// that mis-fires would surrender control WHILE thrust is live, which is the failure this exists to prevent.
// Measure the real lag from MTR###.CSV (ignition_lag_ms) and shorten this once it is known.
constexpr float ROCKET_WEIGHT = 0.927f;       // kg all-up mass at liftoff [MEASURED 2026-08-03, post-ASC036 rebuild;
                                              // was 0.818 for the airframe ASC036 destroyed]. LOGGING ONLY -- it is
                                              // written to the MTR###.CSV header and used nowhere in control, but it
                                              // is the m in the offline thrust reconstruction T = a*m(t) + drag, so a
                                              // stale value silently biases the measured thrust curve by the same 13%.
                                              // [CONFIRM] whether this was weighed with the motor loaded.
constexpr float G             = 9.81f;
constexpr float SEA_LEVEL_HPA = 1013.25f;

// -- Recovery / safety ---------------------------------------------------------
constexpr float EMERGENCY_TILT   = 45.0f;     // deg: |tilt| beyond this DURING BURN -> abort + chute
constexpr float APOGEE_DROP_M    = 1.0f;      // m below peak to call apogee (baro)
constexpr float BACKUP_AFTER_BURNOUT_S = 4.0f;  // baro-fail backup: deploy chute this long AFTER BURNOUT.
// Kinematics check (F-15 + 818 g): apogee is ~6.2 s after launch = ~2.5 s after burnout, so 4 s after burnout fires
// ~1.5 s PAST apogee (descending, low speed -- safe). PRIMARY deploy is the baro at real apogee (~6.6 s); this timer
// is only the sensor-failure failsafe. If you change motor/mass, re-verify apogee stays < BACKUP_AFTER_BURNOUT_S
// after burnout (else the backup would fire during ascent). At burnout+4 s the vehicle is descending, NOT at t=4 s
// from launch (that would be 53 m up ascending at 22 m/s).

// -- IMU (Adafruit_MPU6050) ----------------------------------------------------
// FLIGHT ASC007 (2026-07-07) railed AccelZ at +/-2 g (true axial ~3.2 g at the F-15 thrust peak). Switched the
// driver MPU6050_tockn -> Adafruit_MPU6050 so setup() sets the FSR to +/-16 g / +/-2000 dps with the library
// AUTO-SCALING correctly (no manual rescale factor on the control path). getEvent() returns accel in m/s^2 and
// gyro in rad/s; readIMU() caches them in the firmware's units (gyro deg/s bias-removed, accel m/s^2). Gyro bias
// is measured on the pad (calibrateGyroBias). BENCH after the swap: re-run the SERVO_*_SIGN pitch-the-nose check,
// and confirm the logged roll (GyroZ) tracks a known hand-rotation 1:1 (an axis/scale error hides from the sign check).
constexpr float RAD2DEG = 57.2957795f;
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;    // rad/s, measured at rest on the pad, subtracted on read
float imu_gx=0, imu_gy=0, imu_gz=0;             // cached gyro rate (deg/s, bias-removed)
float imu_ax=0, imu_ay=0, imu_az=0;             // cached accel (m/s^2)

// -- Sensor smoothing ----------------------------------------------------------
constexpr float ANGVEL_ALPHA = 0.9f, VEL_ALPHA = 0.2f;

// -- Hardware ------------------------------------------------------------------
PWMServo servoX, servoY;
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
File logFile; char logFilename[20];

// -- Flight state --------------------------------------------------------------
float altitude, initial_alt, highest_alt, vert_vel;
float gyro_x, gyro_y, gyro_z, ang_vel_x, ang_vel_y;
float accel_x, accel_y, accel_z, tiltX, tiltY;
bool  poweredFlight = false;   // true only during motor burn -> gates emergency deploy
bool  inFlight      = false;   // true launch..apogee -> gates the angle source
unsigned long launchTime = 0;
// Declared up here (not down in the brownout section) because dumpControlLog() writes all three into the
// CTL###.CSV header, and it is defined earlier in the file than the brownout-recovery block.
Persist  persist;              // EEPROM-backed flight state -- see the brownout section for how it is used
uint32_t resetCause  = 0;      // SRC_SRSR latched at boot: WHY the last reset happened (0 = not captured)
uint16_t sdFailCount = 0;      // logData() SD-open failures -- diagnoses "the log stopped mid-flight"

// -- In-flight motor characterization ("static fire in the sky") ---------------------------------------
// High-rate axial specific force (imu_az ~ (thrust - drag)/mass, m/s^2) + ignition lag, buffered in RAM
// during the burn and dumped to MTR###.CSV at recovery. Lets you (1) compare the FLOWN motor's thrust
// curve + ignition lag against the bench static fire, and (2) validate the landing firmware's in-flight
// sys-ID (which identifies exactly this T/m scale) on real motors. Reconstruct thrust offline:
//   T(t) = imu_az(t)*m(t) + drag,   m(t) = dry + prop*(1 - t/BURN_TIME).
constexpr int   MOTOR_BUF_N    = 1000;         // 100 Hz -> ~10 s of headroom; 1000*8 B = 8 KB RAM
constexpr float THRUST_ONSET_G = 1.5f;         // g; axial accel above this = motor lit / lifting off
uint32_t motorT[MOTOR_BUF_N];                  // ms since the ignition COMMAND (P3)
float    motorAz[MOTOR_BUF_N];                 // axial specific force (m/s^2) at that time
int      motorN = 0;                           // samples captured
uint32_t ignCmdMs = 0, thrustOnsetMs = 0;      // P3-command time / first-thrust time (millis)
void captureMotor(){                           // called each burn cycle: detect thrust onset + sample at 100 Hz
  if(thrustOnsetMs==0 && imu_az > THRUST_ONSET_G*G) thrustOnsetMs = millis();   // onset: full-rate for precision
  static uint32_t lastCap=0;
  if(millis()-lastCap < 10) return;            // ~100 Hz cap on the buffered curve
  lastCap=millis();
  if(motorN < MOTOR_BUF_N){ motorT[motorN]=millis()-ignCmdMs; motorAz[motorN]=imu_az; motorN++; }
}
void dumpMotorLog(){                           // write the buffered thrust curve + summary once, at recovery
  static bool done=false; if(done) return; done=true;   // called from BOTH the recovery path and emergency(); the
                                                        // abort path never returns, but the guard makes that a
                                                        // property of this function rather than of its callers.
  char fn[20]; int idx=0; do{ sprintf(fn,"MTR%03d.CSV",idx++); } while(SD.exists(fn)&&idx<1000);
  File f=SD.open(fn,FILE_WRITE); if(!f){ Serial.println(F("MTR log failed")); return; }
  uint32_t lag = (thrustOnsetMs>ignCmdMs)? thrustOnsetMs-ignCmdMs : 0;
  float peak=0; double sum=0; int nb=0;
  for(int i=0;i<motorN;i++){ if(motorAz[i]>peak)peak=motorAz[i];
    if(motorAz[i]>THRUST_ONSET_G*G){ sum+=motorAz[i]; nb++; } }
  char b[64];   // format with snprintf then print a plain string (portable: Teensy File AND the SIL shim)
  snprintf(b,sizeof(b),"# ignition_lag_ms=%lu",(unsigned long)lag);              f.println(b);
  snprintf(b,sizeof(b),"# peak_axial_mps2=%.2f",peak);                           f.println(b);
  snprintf(b,sizeof(b),"# mean_burn_axial_mps2=%.2f",nb?(float)(sum/nb):0.0f);   f.println(b);
  snprintf(b,sizeof(b),"# rocket_mass_kg=%.3f",ROCKET_WEIGHT);                    f.println(b);
  f.println("t_since_ign_cmd_ms,axial_accel_mps2");
  for(int i=0;i<motorN;i++){ snprintf(b,sizeof(b),"%lu,%.3f",(unsigned long)motorT[i],motorAz[i]); f.println(b); }
  f.close();
  snprintf(b,sizeof(b),"MOTOR -> %s  ign_lag_ms=%lu  peak_ax=%.2f  n=%d",fn,(unsigned long)lag,peak,motorN);
  Serial.println(b);
}

// -- High-rate CONTROL-LOOP capture -> CTL###.CSV ------------------------------------------------------
// The SD flight log (logData) writes one row per 50 ms = 20 Hz. That cannot resolve a servo limit cycle,
// cannot measure what fraction of the burn the gimbal spends against its +/-MAX_TILT stop, and cannot see
// the loop's own transport delay -- all three are single-digit-millisecond phenomena and 20 Hz aliases them
// away. This buffers the CONTROL loop in RAM at CTL_HZ during POWERED FLIGHT ONLY, then dumps CTL###.CSV at
// recovery. Same mechanism, naming and write pattern as captureMotor()/dumpMotorLog() above, which flew.
//
// WHY EACH FIELD (they are not redundant):
//   dt_us         loop period of THAT iteration. Control-loop transport delay is currently an ASSUMPTION in
//                 the sim and has never been measured on the vehicle. Logging it makes it an observable.
//                 micros() not millis(): the loop runs ~1-3 ms, so ms resolution would quantize it to 1-3 counts.
//   raw AND filt  ANGVEL_ALPHA (the gyro EMA) is itself an experimental variable. Logging the pre-EMA raw rate
//                 beside the filtered one lets alpha be re-evaluated OFFLINE instead of spending another flight.
//   pre AND post  post = constrain(pre, +-MAX_TILT). Their DIFFERENCE is gimbal saturation; either alone hides it.
//                 (iTerm is not stored -- it is recoverable exactly as pre - P_GAIN*tilt - D_GAIN*rateFilt.)
//
// RAM BUDGET (Teensy 4.1, 512 KB RAM1): 2000 samples x 46 B = 92 KB in struct-of-arrays, matching the
// motorT[]/motorAz[] style (+8 KB if CTL_SERVO_FEEDBACK is on, +0.5 KB for the loop-period histogram).
// 2000 samples at 500 Hz = 4.0 s > the 3.65 s powered phase (IGN_DELAY+BURN_TIME), so the burn never truncates.
#define CTL_CAPTURE        1        // 1 = capture ON (flight default). 0 = arrays not allocated and nothing is
                                    //     stored, but the loop-period statistics below still run -- flashing 0
                                    //     then 1 is the A/B that proves the capture did not lengthen the loop
                                    //     it exists to measure. See the bench procedure in DESIGN_LOG.md.
#define CTL_SERVO_FEEDBACK 0        // 1 = also log analog servo position on A2/A3. DEFAULT OFF: the flight
                                    //     vehicle is NOT wired for it, and analogRead() on a floating pin logs
                                    //     noise while costing real loop time. Turn on only after wiring the pots.
constexpr int      CTL_BUF_N     = 2500;   // 5.0 s at 500 Hz -- must cover IGN_DELAY+BURN_TIME+TVC_EXTEND_S
                                           // (4.65 s), so raised from 2000 when the TVC extension was added
constexpr uint32_t CTL_PERIOD_US = 2000;   // 500 Hz. If the loop is SLOWER than this the capture degrades to one
                                           // sample per iteration -- the gate can only skip work, never add any.
#if CTL_CAPTURE
uint32_t ctlT[CTL_BUF_N];                              // micros() at the sample (raw/monotonic; see the header's ign_cmd_ms)
uint16_t ctlDt[CTL_BUF_N];                             // that iteration's loop period (us), saturated at 65535 (>65 ms stall)
float ctlTiltX[CTL_BUF_N],  ctlTiltY[CTL_BUF_N];       // quaternion tilt fed to the P term (deg)
float ctlRfX[CTL_BUF_N],    ctlRfY[CTL_BUF_N];         // FILTERED body rate used by the D term (deg/s)
float ctlRrX[CTL_BUF_N],    ctlRrY[CTL_BUF_N];         // RAW bias-removed body rate, pre-EMA (deg/s)
float ctlPreX[CTL_BUF_N],   ctlPreY[CTL_BUF_N];        // TVC command BEFORE constrain() (TVC deg)
float ctlPostX[CTL_BUF_N],  ctlPostY[CTL_BUF_N];       // TVC command AFTER constrain() (TVC deg)
#if CTL_SERVO_FEEDBACK
int16_t ctlFbX[CTL_BUF_N],  ctlFbY[CTL_BUF_N];         // raw ADC counts on A2/A3
#endif
#endif
int ctlN = 0;                                          // samples captured

// Loop-period statistics over the powered-flight control loop. Compiled in REGARDLESS of CTL_CAPTURE (it costs
// ~0.5 KB and 4 integer ops per iteration) so the CTL_CAPTURE=0 and =1 builds measure the loop the SAME way and
// the difference between their medians is attributable to the capture and to nothing else.
constexpr int LOOP_HIST_N = 128;               // 100 us bins -> 0..12.8 ms; anything slower lands in the last bin
uint32_t loopHist[LOOP_HIST_N];
uint32_t loopUsMin = 0xFFFFFFFFu, loopUsMax = 0, loopIters = 0;
uint64_t loopUsSum = 0;
uint32_t ctlPrevUs = 0, ctlLastCapUs = 0;      // previous iteration timestamp / last stored sample (rate gate)
uint32_t ctlCostCyc = 0;                       // self-cost of captureControl(), in CTL_CYC() units

// Self-cost of the instrumentation, MEASURED rather than assumed -- the one risk that matters here is that
// capturing at 500 Hz lengthens the loop, which would inflate the very transport delay this exists to measure.
// On Teensy 4.x use the DWT cycle counter (a single register read, ~1.67 ns resolution at 600 MHz); micros()
// has 1 us resolution, which is coarser than the quantity being measured. The host SIL falls back to micros().
#if defined(__IMXRT1062__)
  #define CTL_CYC()      ARM_DWT_CYCCNT
  #define CTL_CYC_PER_US (F_CPU_ACTUAL/1000000u)
#else
  #define CTL_CYC()      micros()
  #define CTL_CYC_PER_US 1u
#endif

// Called at the END of every powered-flight control iteration, after writeServos(), so dt_us spans
// command-to-command = a true control period. Takes plain floats (no struct) so the Arduino IDE's hoisted
// prototypes cannot trip over an undeclared type -- the KF gotcha documented at the top of this file.
void captureControl(float preX, float preY){
  uint32_t c0 = CTL_CYC();
  uint32_t now = micros();
  uint32_t dt  = ctlPrevUs ? (uint32_t)(now - ctlPrevUs) : 0;   // unsigned subtraction is wrap-safe at the micros() rollover
  ctlPrevUs = now;
  if(dt){                                      // distribution over EVERY iteration, sampled or not -- the rate gate
    if(dt < loopUsMin) loopUsMin = dt;         // below skips samples, so per-sample dt alone would undercount
    if(dt > loopUsMax) loopUsMax = dt;
    loopUsSum += dt; loopIters++;
    uint32_t b = dt/100u; if(b >= (uint32_t)LOOP_HIST_N) b = LOOP_HIST_N-1; loopHist[b]++;
  }
#if CTL_CAPTURE
  if(ctlN < CTL_BUF_N && (uint32_t)(now - ctlLastCapUs) >= CTL_PERIOD_US){
    ctlLastCapUs = now;
    int i = ctlN;
    ctlT[i]=now;  ctlDt[i] = (dt>65535u) ? 65535u : (uint16_t)dt;
    ctlTiltX[i]=gyro_x;     ctlTiltY[i]=gyro_y;
    ctlRfX[i]=ang_vel_x;    ctlRfY[i]=ang_vel_y;
    ctlRrX[i]=imu_gx;       ctlRrY[i]=-imu_gy;              // SAME sign convention sensors() feeds into the EMA
    ctlPreX[i]=preX;        ctlPreY[i]=preY;
    ctlPostX[i]=tiltX/SERVO_X_MULT;  ctlPostY[i]=tiltY/SERVO_Y_MULT;   // servo deg -> TVC deg, as logData() does
#if CTL_SERVO_FEEDBACK
    ctlFbX[i]=(int16_t)analogRead(A2);  ctlFbY[i]=(int16_t)analogRead(A3);
#endif
    ctlN++;
  }
#endif
  ctlCostCyc += (uint32_t)(CTL_CYC() - c0);
}

void updatePyros();   // fwd decl: the dump below ticks the pyro timer so a long SD write cannot hold a melt wire on
void dumpControlLog(){
  static bool done=false; if(done) return; done=true;       // recovery OR emergency, whichever is reached first
  char fn[20]; int idx=0; do{ sprintf(fn,"CTL%03d.CSV",idx++); } while(SD.exists(fn)&&idx<1000);
  File f=SD.open(fn,FILE_WRITE); if(!f){ Serial.println(F("CTL log failed")); return; }
  // median from the 100 us histogram (bin midpoint, +-50 us); min/max/mean below are exact to 1 us
  uint32_t half=loopIters/2, acc=0; int mb=0;
  for(int i=0;i<LOOP_HIST_N;i++){ acc+=loopHist[i]; if(acc>half){ mb=i; break; } }
  uint32_t med  = loopIters ? (uint32_t)mb*100u + 50u : 0;
  uint32_t mean = loopIters ? (uint32_t)(loopUsSum/loopIters) : 0;
  uint32_t mn   = loopIters ? loopUsMin : 0;
  uint32_t costUs = ctlCostCyc / (CTL_CYC_PER_US ? CTL_CYC_PER_US : 1u);
  int nsat=0;
#if CTL_CAPTURE
  for(int i=0;i<ctlN;i++) if(fabsf(ctlPreX[i])>MAX_TILT || fabsf(ctlPreY[i])>MAX_TILT) nsat++;   // |pre|>limit = the
#endif                                                                                          // gimbal was on its stop
  char b[128];   // format with snprintf then print a plain string (portable: Teensy File AND the SIL shim)
  snprintf(b,sizeof(b),"# ctl_capture=%d servo_feedback=%d capture_hz=%.0f buf_n=%d samples=%d",
           CTL_CAPTURE,CTL_SERVO_FEEDBACK,1.0e6f/(float)CTL_PERIOD_US,CTL_BUF_N,ctlN);            f.println(b);
  snprintf(b,sizeof(b),"# loop_us_min=%lu loop_us_median=%lu loop_us_mean=%lu loop_us_max=%lu loop_iters=%lu",
           (unsigned long)mn,(unsigned long)med,(unsigned long)mean,(unsigned long)loopUsMax,
           (unsigned long)loopIters);                                                             f.println(b);
  // ACCEPTANCE CHECK: capture_overhead is what the instrumentation added to each loop. Compare it against
  // loop_us_median -- if it is a few percent, the measured transport delay is the vehicle's, not the logger's.
  snprintf(b,sizeof(b),"# capture_overhead_us_total=%lu capture_overhead_us_per_iter=%.3f",
           (unsigned long)costUs, loopIters? (float)costUs/(float)loopIters : 0.0f);              f.println(b);
  snprintf(b,sizeof(b),"# saturated_samples=%d saturated_frac=%.4f",
           nsat, ctlN? (float)nsat/(float)ctlN : 0.0f);                                           f.println(b);
  snprintf(b,sizeof(b),"# p_gain=%.4f d_gain=%.4f i_gain=%.4f max_tilt_deg=%.2f angvel_alpha=%.3f",
           P_GAIN,D_GAIN,I_GAIN,MAX_TILT,ANGVEL_ALPHA);                                           f.println(b);
  snprintf(b,sizeof(b),"# servo_mult_x=%.2f servo_mult_y=%.2f ign_cmd_ms=%lu launch_ms=%lu",
           SERVO_X_MULT,SERVO_Y_MULT,(unsigned long)ignCmdMs,(unsigned long)launchTime);          f.println(b);
  // sd_open_failures>0 with loop_iters covering the full burn = the board lived and only SD writes died.
  // reset_cause!=0 on a flight log = the board rebooted; decode against SRC_SRSR_* in imxrt.h.
  snprintf(b,sizeof(b),"# sd_open_failures=%u reset_cause=0x%08lX resume_boots=%u",
           (unsigned)sdFailCount,(unsigned long)resetCause,(unsigned)persist.boots);              f.println(b);
#if CTL_SERVO_FEEDBACK
  f.println("t_us,dt_us,tiltX_deg,tiltY_deg,rateFx_dps,rateFy_dps,rateRx_dps,rateRy_dps,preX_tvcdeg,preY_tvcdeg,postX_tvcdeg,postY_tvcdeg,fbX_adc,fbY_adc");
#else
  f.println("t_us,dt_us,tiltX_deg,tiltY_deg,rateFx_dps,rateFy_dps,rateRx_dps,rateRy_dps,preX_tvcdeg,preY_tvcdeg,postX_tvcdeg,postY_tvcdeg");
#endif
#if CTL_CAPTURE
  for(int i=0;i<ctlN;i++){
    if((i & 63)==0) updatePyros();   // a 2000-row SD write can outlast the 1000 ms pyro pulse; tick the timer so a
                                     // melt wire is never held on longer than intended just because we are logging
    snprintf(b,sizeof(b),"%lu,%u,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f",
      (unsigned long)ctlT[i],(unsigned)ctlDt[i],ctlTiltX[i],ctlTiltY[i],ctlRfX[i],ctlRfY[i],
      ctlRrX[i],ctlRrY[i],ctlPreX[i],ctlPreY[i],ctlPostX[i],ctlPostY[i]);
#if CTL_SERVO_FEEDBACK
    char b2[24]; snprintf(b2,sizeof(b2),",%d,%d",(int)ctlFbX[i],(int)ctlFbY[i]); strncat(b,b2,sizeof(b)-strlen(b)-1);
#endif
    f.println(b);
  }
#endif
  f.close();
  snprintf(b,sizeof(b),"CTL -> %s  n=%d  loop_us min/med/max=%lu/%lu/%lu  overhead_us/iter=%.3f  sat=%.1f%%",
    fn,ctlN,(unsigned long)mn,(unsigned long)med,(unsigned long)loopUsMax,
    loopIters? (float)costUs/(float)loopIters : 0.0f, ctlN? 100.0f*nsat/ctlN : 0.0f);
  Serial.println(b);
}

// -- PASSIVE vertical Kalman (baro + IMU fusion) -------------------------------------------------------
// Verbatim port of the Sysiphus_Landing kfZ (3-state: position, velocity, accel-bias; strapdown predict on
// the world-vertical accel, baro update with velocity lag-compensation). Runs PASSIVE here: estZ/estVz are
// LOGGED ONLY, never used for control -- so an ascent flight validates the hoverslam's altitude/velocity
// estimator on real sensor data (vibration, baro transients) against the crude finite-difference vert_vel,
// with ZERO risk to the flown PD controller. NOTE: ascent regime only -- a clean result here does NOT
// validate the landing (descent / near-ground baro / retrograde) regime. (struct KF is defined at the top,
// before all functions, so the Arduino IDE's hoisted prototypes can see the type; kfZ/estZ/estVz live here.)
KF kfZ; float estZ=0, estVz=0;
constexpr float ACCEL_NOISE=0.05f, BARO_NOISE=0.30f, Q_BIAS=1e-7f, BARO_LAG_S=0.040f;
static void m3mul(const float*A,const float*B,float*C){
  for(int i=0;i<3;i++)for(int j=0;j<3;j++){ float s=0; for(int k=0;k<3;k++)s+=A[i*3+k]*B[k*3+j]; C[i*3+j]=s; } }
void kfInit(KF&k,float p0){ k.s[0]=p0;k.s[1]=0;k.s[2]=0; for(int i=0;i<9;i++)k.P[i]=0; k.P[0]=0.04f; k.P[4]=0.04f; k.P[8]=0.01f; }
void kfPredict(KF&k,float a,float dt){             // dt = ACTUAL elapsed step (sensor rate), not a fixed nominal
  k.s[0]+=k.s[1]*dt; k.s[1]+=(a-k.s[2])*dt;
  const float F[9]={1,dt,0,0,1,-dt,0,0,1}, Ft[9]={1,0,0,dt,1,0,0,-dt,1};
  float FP[9],Pn[9]; m3mul(F,k.P,FP); m3mul(FP,Ft,Pn); for(int i=0;i<9;i++)k.P[i]=Pn[i];
  k.P[0]+=1e-7f; k.P[4]+=(ACCEL_NOISE*dt)*(ACCEL_NOISE*dt); k.P[8]+=Q_BIAS;
}
void kfUpdate(KF&k,int hi,float meas,float R){
  float PHt[3]={k.P[hi],k.P[3+hi],k.P[6+hi]}, S=PHt[hi]+R, K[3]={PHt[0]/S,PHt[1]/S,PHt[2]/S}, in=meas-k.s[hi];
  k.s[0]+=K[0]*in; k.s[1]+=K[1]*in; k.s[2]+=K[2]*in;
  float IKH[9]={1,0,0,0,1,0,0,0,1}; IKH[hi]-=K[0]; IKH[3+hi]-=K[1]; IKH[6+hi]-=K[2];
  float Pn[9]; m3mul(IKH,k.P,Pn); for(int i=0;i<9;i++)k.P[i]=Pn[i];
}

// forward declarations (Arduino auto-prototypes; declared explicitly so it also builds in plain C++)
void sensors(); void emergency(); void TVC();

// -- Pyro ----------------------------------------------------------------------
struct PyroChannel { int pin; bool active; unsigned long startTime; };
PyroChannel pyros[] = { {P1,false,0}, {P2,false,0}, {P3,false,0}, {P4,false,0} };
void triggerPyro(int pin){ for(auto&c:pyros) if(c.pin==pin&&!c.active){ digitalWrite(pin,HIGH); c.active=true; c.startTime=millis(); } }
void updatePyros(){ for(auto&c:pyros) if(c.active&&millis()-c.startTime>=1000){ digitalWrite(c.pin,LOW); c.active=false; } }

// -- Terminal-state button exit ------------------------------------------------
// Every "done" state (abort, recovered, sensor-init failure) used to be a hard while(true). The only way
// out was a power cycle, which in the field means unplugging a flight battery from a vehicle that may
// still have live pyro channels -- the worst moment to be handling it. A button press now leaves those
// loops and restarts the flight computer into its normal ARM state.
// SAFETY, in this order and for these reasons:
//   1. The press is CONSUMED (wait for release) so buttonCount() cannot mistake it for arming press #1.
//      Bounded at 5 s so a stuck or shorted button degrades to "carry on beeping", never to a silent hang.
//   2. Every pyro is then driven LOW and disarmed, so a press can NEVER leave a melt wire energized --
//      the one thing this must not do is strand a channel HIGH.
//   3. Re-arming still requires the full 5 rapid presses + 30 s countdown, so one press cannot ignite
//      anything. No interlock is weakened; only the escape from a dead-end loop is added.
bool exitOnButton(){
  if(digitalRead(BUTTON)!=HIGH) return false;
  unsigned long t0=millis();
  while(digitalRead(BUTTON)==HIGH && millis()-t0<5000) updatePyros();
  for(auto&c:pyros){ digitalWrite(c.pin,LOW); c.active=false; }
  return true;
}
void neutralServos(); void LED(bool,bool,bool); void beep(int,int);   // fwd decls: defined below in Utils, so this
                                                                     // block also builds as plain C++ in the SIL
// Announced, retryable halt for a failed sensor init. n blinks + beeps = which sensor (2 = BMP280, 3 = MPU6050),
// repeating forever so it is diagnosable on the pad without a laptop. The button restarts (retries init); it
// NEVER continues into flight -- no IMU/baro means no attitude and no apogee detect.
void resetFlightComputer();
void sensorFaultHalt(int n){
  while(true){
    for(int i=0;i<n;i++){ LED(true,false,false); beep(880,120); LED(false,false,false); delay(180); }
    unsigned long t0=millis();
    while(millis()-t0<1200) if(exitOnButton()) resetFlightComputer();
  }
}
// Full restart rather than a return. emergency() is called three frames deep (loop -> TVC -> sensors), so
// simply returning would drop back INTO the ascent loop, which would then coast to its backup timer and
// re-fire pyros that had already fired. A reset guarantees no stale flight state and no re-fire, and lands
// in setup() -> loop() = ARM, which is exactly the state you want after walking back with the rocket.
void resetFlightComputer(){
  neutralServos();
  for(auto&c:pyros){ digitalWrite(c.pin,LOW); c.active=false; }   // belt-and-braces: never reset with a channel HIGH
#if defined(__IMXRT1062__)
  SCB_AIRCR = 0x05FA0004;              // ARM Application Interrupt & Reset Control -- system reset request (Teensy 4.x)
#endif
  while(true) updatePyros();           // unreachable on Teensy; the SIL/host fallback keeps the old terminal behaviour
}

// -- Utils ---------------------------------------------------------------------
void LED(bool r,bool g,bool b){ digitalWrite(RLED,!r); digitalWrite(GLED,!g); digitalWrite(BLED,!b); }
void beep(int f,int d){ tone(BUZZER,f); delay(d); noTone(BUZZER); }
void writeServos(float sx,float sy){                    // clamp to a valid servo range for safety
  servoX.write((int)constrain(SERVO_X_SIGN*sx + 90 + XTUNE, 0, 180));
  servoY.write((int)constrain(SERVO_Y_SIGN*sy + 90 + YTUNE, 0, 180));
}
void neutralServos(){ servoX.write((int)(90+XTUNE)); servoY.write((int)(90+YTUNE)); }

// -- Burnout servo centering ramp ----------------------------------------------
// At burnout the gimbal can be tens of servo-degrees off neutral. Stepping straight to centre drives BOTH
// loaded servos at full rate simultaneously -- the largest current transient of the flight, on the same
// rail that feeds the flight computer -- and shock-loads the linkage while the nozzle still has residual
// pressure. Easing there over BURNOUT_CENTER_MS costs nothing: thrust is gone, so the gimbal has nothing
// to vector and there is no control authority to preserve. Smoothstep (not linear) so the motion starts
// and ends at zero rate -- no jerk at either end. Peak rate is ~1.5x the average, i.e. ~41 deg/s from a
// full 27.5 deg deflection, far below the servo's own slew limit, so it is the servo's gentlest motion.
constexpr unsigned long BURNOUT_CENTER_MS = 1000;
bool burnoutRampActive=false; unsigned long burnoutT0=0; float burnoutSx=0, burnoutSy=0;
void centerServosRamp(){
  if(!burnoutRampActive){ neutralServos(); return; }
  unsigned long el = millis()-burnoutT0;
  if(el >= BURNOUT_CENTER_MS){ burnoutRampActive=false; tiltX=tiltY=0; neutralServos(); return; }
  float f = (float)el/(float)BURNOUT_CENTER_MS;
  float k = 1.0f - (f*f*(3.0f-2.0f*f));            // smoothstep from 1 -> 0
  tiltX = burnoutSx*k; tiltY = burnoutSy*k;        // keep tiltX/tiltY truthful: logData() logs them as the TVC command
  writeServos(tiltX, tiltY);
}

// -- Brownout / reset survival -------------------------------------------------------------------------
// GOAL (the one that matters): a power glitch in flight must not cost the parachute. A reboot at apogee
// should still recover the vehicle.
//
// *** WHY THIS DOES NOT RESUME TVC -- read before "improving" it. ***
// Attitude is NOT recoverable after an in-flight reboot, and that is physics, not a design choice. The
// quaternion is dead-reckoned from the pad; a reboot loses it, and the gap is unobserved. It cannot be
// re-levelled from the accelerometer either, because levelling needs gravity ALONE as the reference:
//   - under thrust the accel reads thrust+gravity (~3 g along the body axis) -- no gravity reference;
//   - in coast the vehicle is in near free-fall, so the accel reads ~0 -- no reference at all.
// A strapdown IMU cannot recover WORLD-referenced attitude without an external reference, and there is
// none on this vehicle. Resuming TVC on a stale attitude would command corrections against an orientation
// the vehicle no longer has -- actively driving a tumble, strictly worse than coasting. So: recovery only.
// This never touches P3 (motor ignition) and never re-enters the control loop.
constexpr uint32_t PERSIST_MAGIC   = 0x54564332;   // 'TVC2'
constexpr int      PERSIST_ADDR    = 0;
constexpr uint16_t RESUME_MAX_BOOTS = 3;           // give up after this many reboots in one flight
constexpr unsigned long RESUME_MAX_WAIT_MS = 8000; // hard deploy timeout once a resume is armed
void readIMU();                                    // fwd decl: defined below, so this block also builds as plain C++
uint32_t persistSum(const Persist&p){               // cheap integrity check: a half-written record must not be trusted
  const uint8_t* b=(const uint8_t*)&p; uint32_t h=2166136261u;
  for(size_t i=0;i<sizeof(Persist)-sizeof(uint32_t);i++){ h^=b[i]; h*=16777619u; }
  return h;
}
void savePersist(){ persist.magic=PERSIST_MAGIC; persist.sum=persistSum(persist); EEPROM.put(PERSIST_ADDR,persist); }
void loadPersist(){ EEPROM.get(PERSIST_ADDR,persist);
  if(persist.magic!=PERSIST_MAGIC || persist.sum!=persistSum(persist)){ persist=Persist(); persist.phase=0; } }
void setPhase(uint8_t ph){ persist.phase=ph; savePersist(); }   // called at phase EDGES only -- never in the control
                                                                // loop (a Teensy EEPROM write is flash-backed and can
                                                                // stall for milliseconds; that is fine in coast, and
                                                                // it is never done while TVC is active)

// Is the vehicle sitting in someone's hand / on the pad, or is it actually flying? This is the interlock that
// stops a stale EEPROM record from firing a pyro during handling, so it is deliberately strict: it demands
// POSITIVE evidence of flight, and anything ambiguous reads as "on the ground".
//   free-fall / coast : |a| collapses to ~0-0.3 g   (drag only)
//   under thrust      : |a| ~3 g
//   tumbling          : body rate >> anything hand-handling produces
//   being carried     : |a| ~1 g and rates are low  -> NOT flight -> no deploy
bool looksAirborne(){
  float minA=1e9f, maxA=-1e9f, maxW=0;
  unsigned long t0=millis();
  while(millis()-t0<400){
    readIMU();
    float am=sqrtf(imu_ax*imu_ax+imu_ay*imu_ay+imu_az*imu_az);
    float wm=sqrtf(imu_gx*imu_gx+imu_gy*imu_gy+imu_gz*imu_gz);
    if(am<minA)minA=am; if(am>maxA)maxA=am; if(wm>maxW)maxW=wm;
    delay(5);
  }
  bool notOneG = (maxA < 0.55f*G) || (minA > 2.0f*G);   // free-fall or thrust -- neither survives being held
  bool tumbling = (maxW > 90.0f);                        // deg/s, well above hand movement
  return notOneG || tumbling;
}

// Armed only when a reboot is detected mid-flight AND the sensors independently agree the vehicle is flying.
void resumeAfterReset(){
  if(persist.phase==0 || persist.phase>=3) return;          // nothing in progress, or the chute is already out
  if(persist.boots>=RESUME_MAX_BOOTS){ setPhase(0); return; }// reboot loop: stop trying, fail safe to ARM
  gyroBiasX=persist.gbx; gyroBiasY=persist.gby; gyroBiasZ=persist.gbz;   // restore what cannot be re-measured
  initial_alt=persist.groundAlt;
  persist.boots++; savePersist();
  if(!looksAirborne()){ setPhase(0); persist.boots=0; savePersist(); return; }   // on the ground -> normal ARM

  Serial.println(F("RESUME: reboot detected IN FLIGHT -- parachute watchdog armed (no TVC, see note in source)"));
  LED(true,false,true);
  highest_alt=0;
  unsigned long t0=millis();
  while(true){
    readIMU();
    altitude = bmp.readAltitude(SEA_LEVEL_HPA) - initial_alt;
    if(altitude>highest_alt) highest_alt=altitude;
    bool thrusting = imu_az > THRUST_ONSET_G*G;             // NEVER deploy under thrust -- that destroys the chute
    if(!thrusting && (altitude < highest_alt-APOGEE_DROP_M || millis()-t0>RESUME_MAX_WAIT_MS)) break;
    updatePyros(); delay(10);
  }
  triggerPyro(P4);                    // parachute
  triggerPyro(P1);                    // legs
  setPhase(3);
  dumpControlLog();                   // whatever survived in RAM is gone after a reboot, but the loop stats are not
  Serial.println(F("RESUME: chute + legs deployed"));
  LED(false,true,true);
  while(!exitOnButton()){ beep(523,600); beep(392,600); updatePyros(); }
  setPhase(0); persist.boots=0; savePersist();
  LED(false,false,false);
}

// -- SD logging ----------------------------------------------------------------
void createUniqueLogFile(){
  int idx=0; do{ sprintf(logFilename,"ASC%03d.CSV",idx++); } while(SD.exists(logFilename)&&idx<1000);
  logFile=SD.open(logFilename,FILE_WRITE);
  if(logFile){ Serial.print(F("Logging to: ")); Serial.println(logFilename);
    logFile.println(F("Time(ms),Altitude(m),VertVel(m/s),GyroX,GyroY,GyroZ,AngVelX,AngVelY,AccelX,AccelY,AccelZ,TVCx(deg),TVCy(deg),Phase,estZ(m),estVz(m/s)"));
    logFile.close();
  } else Serial.println(F("Failed to create log file!"));
}
void logData(){
  logFile=SD.open(logFilename,FILE_WRITE);
  if(!logFile){ if(sdFailCount<65535) sdFailCount++; return; }   // count, don't retry: a retry inside the control
                                                                 // loop would lengthen exactly the stall that
                                                                 // caused the failure. The count + the CTL log's
                                                                 // loop_iters together separate "SD died" from
                                                                 // "the whole board died" (the ASC036 question).
  char buf[160];
  // TVCx/TVCy = the COMMANDED TVC angle in deg (tilt/ MULT recovers it from the servo-offset value); Phase 0=pad 1=boost 2=coast 3=descent
  int phase = poweredFlight ? 1 : (inFlight ? 2 : (highest_alt>2.0f ? 3 : 0));
  snprintf(buf,sizeof(buf),"%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f",
    millis(),altitude,vert_vel,gyro_x,gyro_y,gyro_z,ang_vel_x,ang_vel_y,accel_x,accel_y,accel_z,
    tiltX/SERVO_X_MULT, tiltY/SERVO_Y_MULT, phase, estZ, estVz);
  logFile.println(buf); logFile.close();
}

// -- IMU read / calibrate (Adafruit_MPU6050) -----------------------------------
void readIMU(){                                   // one getEvent -> cache in the firmware's units (replaces mpu.update())
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  imu_gx = (g.gyro.x - gyroBiasX) * RAD2DEG;       // deg/s, bias-removed
  imu_gy = (g.gyro.y - gyroBiasY) * RAD2DEG;
  imu_gz = (g.gyro.z - gyroBiasZ) * RAD2DEG;
  imu_ax = a.acceleration.x; imu_ay = a.acceleration.y; imu_az = a.acceleration.z;   // m/s^2
}
void calibrateGyroBias(){                          // average N samples at rest -> rad/s bias (replaces calcGyroOffsets)
  const int N=600; double sx=0,sy=0,sz=0;
  for(int i=0;i<N;i++){ sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t); sx+=g.gyro.x; sy+=g.gyro.y; sz+=g.gyro.z; delay(2); }
  gyroBiasX=(float)(sx/N); gyroBiasY=(float)(sy/N); gyroBiasZ=(float)(sz/N);
}

// -- Quaternion attitude estimator (roll-aware) --------------------------------
// q = (qw,qx,qy,qz), body->world. REPLACES the old naive body-rate integration (gyro_x += ang_vel_x*dt),
// which treated body pitch/yaw RATES as if they were inertial tilt ANGLES -- valid only at small roll. Once
// the vehicle rolled ~180 deg (ASC007/ASC031: uncontrolled roll to ~190 deg) the body X/Y axes had swapped,
// so the naive "tilt" was wrong and the PD correction inverted -> roll-coupling divergence. The quaternion
// tracks full 3D orientation, so the tilt decomposition below stays correct at ANY roll.
// Body frame = the COMPENSATED sensor frame: rates (imu_gx, -imu_gy, imu_gz) -- the -imu_gy un-inverts the
// MPU Y exactly as the control channels already do, giving a consistent right-handed frame with the accel.
float qw=1.0f, qx=0.0f, qy=0.0f, qz=0.0f;
constexpr float DEG2RAD = 0.017453292519943295f;
void quatNormalize(){ float n=sqrtf(qw*qw+qx*qx+qy*qy+qz*qz); if(n>1e-9f){ qw/=n; qx/=n; qy/=n; qz/=n; } }
void quatFromAccel(){                               // shortest-arc: rotate body-up (accel dir) onto world +Z.
  float ax=imu_ax, ay=imu_ay, az=imu_az, n=sqrtf(ax*ax+ay*ay+az*az);   // ONLY valid at rest (accel == gravity)
  if(n<1e-6f){ qw=1; qx=qy=qz=0; return; }
  ax/=n; ay/=n; az/=n;                              // measured "up" in the body frame (unit)
  qw=1.0f+az; qx=ay; qy=-ax; qz=0.0f;               // q = (1+u.z, u x zhat), u=(ax,ay,az), zhat=(0,0,1)
  if(qw<1e-6f){ qw=0; qx=1; qy=0; qz=0; }           // pointing straight down (degenerate): 180 deg about body X
  quatNormalize();
}
void quatPropagate(float gxd,float gyd,float gzd,float dt){   // integrate body rates (deg/s) one step
  float wx=gxd*DEG2RAD, wy=gyd*DEG2RAD, wz=gzd*DEG2RAD;       // Qdot = 0.5 * Q (x) (0, wb)  (Hamilton)
  float dw=0.5f*(-qx*wx - qy*wy - qz*wz);
  float dx=0.5f*( qw*wx + qy*wz - qz*wy);
  float dy=0.5f*( qw*wy - qx*wz + qz*wx);
  float dz=0.5f*( qw*wz + qx*wy - qy*wx);
  qw+=dw*dt; qx+=dx*dt; qy+=dy*dt; qz+=dz*dt; quatNormalize();
}
void quatUpInBody(float&ux,float&uy,float&uz){      // world +Z expressed in the body frame = R(q)^T*(0,0,1)
  ux = 2.0f*(qx*qz - qw*qy);
  uy = 2.0f*(qy*qz + qw*qx);
  uz = 1.0f - 2.0f*(qx*qx + qy*qy);
}

// -- Sensors (one attitude source at a time; identical policy to the flown code)
//    on the pad: accel levels the quaternion (gravity is a reliable reference)
//    in flight : quaternion propagated by the gyro (accel reads thrust+gravity, unusable as a reference)
void sensors(){
  static unsigned long prevTime=millis(), angleTime=millis(); static float prev_alt=0;
  readIMU();
  float raw_avx =  imu_gx;    // deg/s, bias-removed, Adafruit auto-scaled (no FSR fix factor)
  float raw_avy = -imu_gy;
  ang_vel_x = ANGVEL_ALPHA*raw_avx + (1-ANGVEL_ALPHA)*ang_vel_x;
  ang_vel_y = ANGVEL_ALPHA*raw_avy + (1-ANGVEL_ALPHA)*ang_vel_y;

  unsigned long angleNow=millis(); float dt=(angleNow-angleTime)/1000.0f; angleTime=angleNow;
  // --- QUATERNION attitude (roll-aware) -- replaces the old naive body-rate integration whose tilt estimate
  //     inverted near ~180 deg roll. On the PAD (!inFlight): continuously re-level q to gravity (accel is a
  //     valid reference at rest). In FLIGHT: PURE gyro propagation -- under thrust the accel reads thrust+
  //     gravity and is NOT a gravity reference, so NO accel correction (gyro drift over a ~3.5 s boost is
  //     small). tilt = body +Z (thrust axis) vs vertical, decomposed with the SAME sign convention as the old
  //     pad atan2 (gyro_x==atan2(ay,az), gyro_y==-atan2(ax,az) when leveled) so the pole-placed PD gains are UNCHANGED.
  if(inFlight){ if(dt>0 && dt<0.1f) quatPropagate(imu_gx, -imu_gy, imu_gz, dt); }
  else quatFromAccel();
  float ubx,uby,ubz; quatUpInBody(ubx,uby,ubz);     // world-up in the body frame
  gyro_x =  atan2f(uby, ubz)*RAD2DEG;               // tilt in the firmware's X control channel (deg)
  gyro_y = -atan2f(ubx, ubz)*RAD2DEG;               // tilt in the Y control channel (deg)
  // roll: integrate the Z rate ourselves (logged, not controlled), from the pad-cal reset (gyro_z=0 -> on-pad roll ~0)
  if(dt>0 && dt<0.1f) gyro_z += imu_gz*dt;
  accel_x=imu_ax; accel_y=imu_ay; accel_z=imu_az;   // m/s^2 (already scaled)

  altitude = bmp.readAltitude(SEA_LEVEL_HPA) - initial_alt;
  if(altitude>highest_alt) highest_alt=altitude;

  // PASSIVE vertical Kalman (logged, NOT used for control). aWz = world-vertical accel = (body specific force
  // rotated up) - g; (ubx,uby,ubz) IS the strapdown rotation's 3rd row, so reuse it. Baro update lead-compensated.
  if(dt>0 && dt<0.1f){
    float aWz = ubx*imu_ax + uby*imu_ay + ubz*imu_az - G;
    kfPredict(kfZ, aWz, dt);
    kfUpdate(kfZ, 0, altitude + estVz*BARO_LAG_S, BARO_NOISE*BARO_NOISE);
    estZ=kfZ.s[0]; estVz=kfZ.s[1];
  }

  unsigned long now=millis(), elapsed=now-prevTime;
  if(elapsed>=50){
    float raw_vel=(altitude-prev_alt)/(elapsed/1000.0f);
    vert_vel = VEL_ALPHA*raw_vel + (1-VEL_ALPHA)*vert_vel;
    prev_alt=altitude; prevTime=now; logData();
  }
  updatePyros();
  emergency();
}

// -- Emergency: only during the burn, tumble past EMERGENCY_TILT -> chute -------
void emergency(){
  if(!poweredFlight) return;
  if(fabsf(gyro_x)<=EMERGENCY_TILT && fabsf(gyro_y)<=EMERGENCY_TILT) return;
  neutralServos();
  Serial.println(F("EMERGENCY"));
  LED(true,false,false);
  triggerPyro(P4);                 // parachute first
  triggerPyro(P1);                 // then legs -- protect the TVC on the (tumbling) recovery landing
  poweredFlight=false; inFlight=false;
  setPhase(3);                     // chute out: a reboot after this must not re-arm the resume watchdog
  // Both pyros have ALREADY fired -- this only delays the beeping, and the dumps tick updatePyros() internally so
  // the 1000 ms melt-wire pulse is unaffected. An abort is precisely the flight whose control trace you need
  // (ASC007 aborted at 43 deg with the gimbal saturated); without this the buffers die in RAM on every failure.
  dumpMotorLog(); dumpControlLog();
  // Beep until the button is pressed, then RESTART into ARM (see exitOnButton/resetFlightComputer).
  // Previously a press only moved to an inner while(true) -- it silenced the buzzer but the computer was
  // still stuck, so recovering the vehicle meant unplugging a battery from a possibly-live pyro bank.
  while(!exitOnButton()){ updatePyros(); beep(500,50); delay(50); }
  Serial.println(F("EMERGENCY cleared by button -- restarting"));
  LED(false,false,false);
  resetFlightComputer();
}

// -- Arm: 5 rapid button presses ----------------------------------------------
bool buttonCount(){
  constexpr int PRESS_WINDOW=300, PRESSES_REQD=5;
  static unsigned long lastPress=0; static int pressCount=0;
  if(digitalRead(BUTTON)==HIGH){
    if(millis()-lastPress<=PRESS_WINDOW){ pressCount++; LED(false,true,false);
      tone(BUZZER,311.13f*pow(2.0f,pressCount/12.0f));
      while(digitalRead(BUTTON)==HIGH){} noTone(BUZZER); LED(false,false,false);
    } else pressCount=1;
    lastPress=millis();
  }
  return pressCount>PRESSES_REQD;
}

// -- Countdown + pad calibration + pre-launch sanity check ---------------------
bool countdown(){
  constexpr int DURATION=30;
  neutralServos();
  createUniqueLogFile();
  for(int i=DURATION;i>0;i--){
    Serial.println(i);
    if(i>5){ LED(true,false,false); beep(440,200); LED(false,false,false); delay(800); }
    else if(i>3){ LED(true,false,false); tone(BUZZER,440); delay(1000); }
    else if(i==3){ LED(true,false,true); tone(BUZZER,880);
      calibrateGyroBias();
      initial_alt=bmp.readAltitude(SEA_LEVEL_HPA);
      Serial.print(F("  baseline alt: ")); Serial.println(initial_alt);
      noTone(BUZZER); LED(false,false,false); }
  }
  // let the loop settle, then LEVEL the attitude quaternion to the pad (accel gravity) and zero the roll log +
  // rate filters + integral accumulators for launch. gyro_x/gyro_y are now DERIVED from the quaternion each
  // sensors() call, so the quaternion -- not a hard-zeroed angle -- carries the initial attitude: launch starts
  // from the TRUE measured pad tilt, not an assumed vertical.
  unsigned long flush=millis(); while(millis()-flush<1000) readIMU();
  quatFromAccel(); gyro_z=ang_vel_x=ang_vel_y=0; iTermX=iTermY=0; kfInit(kfZ,0); estZ=estVz=0;

  // pre-launch sanity: bad tilt (>45) or drift (>10 dps) -> abort, no ignition
  for(int i=0;i<10;i++){ readIMU(); delay(20); }
  // accel-atan2 tilt (gravity ratio) + bias-removed rate drift
  float cx= atan2f(imu_ay,imu_az)*180.0f/M_PI;
  float cy=-atan2f(imu_ax,imu_az)*180.0f/M_PI;
  float cavx=imu_gx, cavy=imu_gy;
  if(fabsf(cx)>45||fabsf(cy)>45||fabsf(cavx)>10||fabsf(cavy)>10){
    Serial.print(F("ABORT -- tilt X=")); Serial.print(cx); Serial.print(F(" Y=")); Serial.print(cy);
    Serial.print(F("  rate avX=")); Serial.print(cavx); Serial.print(F(" avY=")); Serial.println(cavy);
    LED(true,false,false); for(int i=0;i<10;i++){ beep(880,100); delay(100); } LED(false,false,false);
    return false;
  }
  Serial.println(F("LAUNCH"));
  // Persist the two things a post-brownout reboot cannot re-derive: the pad gyro-bias calibration (needs
  // 1.2 s of stillness) and the ground barometric reference. Written HERE, before ignition, so the record
  // is committed while the vehicle is still stationary and an EEPROM stall costs nothing.
  persist.gbx=gyroBiasX; persist.gby=gyroBiasY; persist.gbz=gyroBiasZ;
  persist.groundAlt=initial_alt; persist.boots=0; setPhase(1);
  ignCmdMs=millis();               // stamp the ignition COMMAND time -> the in-flight motor ignition-lag measurement
  triggerPyro(P3);
  return true;
}

// -- TVC: per-axis PID to vertical (setpoint 0) -------------------------------
// PD (pole-placed) + an integral TRIM with conditional-integration anti-windup. The I term winds out a CONSTANT
// thrust-misalignment/CG-offset (the ASC007 trim that held a ~16 deg lean) so the vehicle flies VERTICAL, off the
// 45 deg abort. Anti-windup: freeze the integrator whenever the pre-clamp command is saturated (a REAL tumble
// saturates -> the I term can't wind up and make it worse), and clamp the accumulator to +-MAX_TILT.
void TVC(){
  sensors();
  static unsigned long itPrev=millis();
  unsigned long itNow=millis(); float idt=(itNow-itPrev)/1000.0f; itPrev=itNow;
  if(idt<0 || idt>0.1f) idt=0;                                  // guard: first call / loop stall
  float rawX = P_GAIN*gyro_x + iTermX + D_GAIN*ang_vel_x;
  float rawY = P_GAIN*gyro_y + iTermY + D_GAIN*ang_vel_y;
  if(fabsf(rawX) < MAX_TILT){ iTermX += I_GAIN*gyro_x*idt; iTermX = constrain(iTermX,-MAX_TILT,MAX_TILT); }
  if(fabsf(rawY) < MAX_TILT){ iTermY += I_GAIN*gyro_y*idt; iTermY = constrain(iTermY,-MAX_TILT,MAX_TILT); }
  // cmdX/cmdY are the PRE-CLAMP command -- the exact argument constrain() already received (note it uses the
  // JUST-UPDATED iTerm, unlike rawX/rawY above, which use the pre-update value for the anti-windup test). Hoisting
  // it into a local changes nothing numerically -- same expression, same order of operations -- but it makes the
  // pre-clamp value observable, and it evaluates the sum ONCE instead of the 2-3 times the constrain() MACRO
  // textually substituted it. Post-clamp minus pre-clamp is how gimbal saturation is detected offline.
  float cmdX = P_GAIN*gyro_x + iTermX + D_GAIN*ang_vel_x;
  float cmdY = P_GAIN*gyro_y + iTermY + D_GAIN*ang_vel_y;
  tiltX = constrain(cmdX, -MAX_TILT, MAX_TILT) * SERVO_X_MULT;
  tiltY = constrain(cmdY, -MAX_TILT, MAX_TILT) * SERVO_Y_MULT;
  writeServos(tiltX, tiltY);
  captureControl(cmdX, cmdY);      // AFTER the servo write: dt_us then spans command-to-command = a true control period
}

// -- Setup ---------------------------------------------------------------------
void setup(){
  pinMode(BUTTON,INPUT); pinMode(BUZZER,OUTPUT);
  pinMode(RLED,OUTPUT); pinMode(GLED,OUTPUT); pinMode(BLED,OUTPUT);
  pinMode(P1,OUTPUT); pinMode(P2,OUTPUT); pinMode(P3,OUTPUT); pinMode(P4,OUTPUT);
  digitalWrite(P1,LOW); digitalWrite(P2,LOW); digitalWrite(P3,LOW); digitalWrite(P4,LOW);
  servoX.attach(SERVO_X_PIN); servoY.attach(SERVO_Y_PIN);
#if CTL_SERVO_FEEDBACK
  analogReadResolution(12);        // match Bench_Latency.ino / feedback_servo_calibration.ino (A2/A3, 12-bit) so
  analogReadAveraging(1);          // in-flight feedback counts are directly comparable to the bench calibration
#endif
  neutralServos();
  LED(false,false,false);
  Serial.begin(115200);
  while(!Serial && millis()<3000);   // wait up to 3 s for the USB monitor so the setup prints (MPU range) are
                                     // visible on the bench; times out -> on the pad with NO monitor it proceeds
                                     // after 3 s (never hangs). Pure startup timing; no control/safety effect.
  Wire.begin();
  // Sensor-init failure used to be a silent while(true) -- on the pad with no laptop that is indistinguishable
  // from a dead board. Now it is ANNOUNCED (red LED + buzzer) and the button RETRIES via a restart. The button
  // deliberately does NOT continue past a failed init: flying without an IMU or baro is not a survivable option.
  if(!bmp.begin(0x76)){ Serial.println(F("BMP280 not found")); sensorFaultHalt(2); }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_1);
  if(!mpu.begin()){ Serial.println(F("MPU6050 not found")); sensorFaultHalt(3); }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);   // ASC007 railed at +/-2 g; +/-16 g clears the ~3.2 g axial peak
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);        // headroom for roll/tumble; control rates stay well under 2000 dps
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);      // DLPF: reject motor vibration without lagging the loop
  Serial.print(F("MPU6050 range (want 3/3 = 16g/2000dps): acc=")); Serial.print((int)mpu.getAccelerometerRange());
  Serial.print(F(" gyro=")); Serial.println((int)mpu.getGyroRange());
  SD.begin(SD_CS);

  // WHY the last reset happened. This is the diagnostic ASC036 needed and did not have: its log stopped
  // mid-flight and nothing recorded whether the board browned out, was watchdog-reset, or kept running
  // while only the SD writes failed. Bit meanings are in imxrt.h (SRC_SRSR_*); the raw word is logged so
  // it can be decoded offline. Read-and-clear: the register is sticky across resets until written back.
#if defined(__IMXRT1062__)
  resetCause = SRC_SRSR; SRC_SRSR = resetCause;
#endif
  Serial.print(F("reset cause (SRC_SRSR) = 0x")); Serial.print((int)resetCause); Serial.print(F("  ="));
  if(resetCause & (1<<0)) Serial.print(F(" POWER-ON-or-BROWNOUT"));   // the LVD brownout path resets through POR,
                                                                     // so this bit ALONE cannot separate the two --
                                                                     // use persist.phase below: a POR while phase!=0
                                                                     // means power was lost MID-FLIGHT = brownout.
  if(resetCause & (1<<1)) Serial.print(F(" CPU-LOCKUP-or-SOFTWARE-RESET"));  // expected after an abort restart
  if(resetCause & (1<<3)) Serial.print(F(" RESET-PIN"));
  if(resetCause & (1<<4)) Serial.print(F(" WATCHDOG"));
  if(resetCause & (1<<7)) Serial.print(F(" WATCHDOG3"));
  if(resetCause & (1<<8)) Serial.print(F(" OVER-TEMPERATURE"));
  if(resetCause & ((1<<2)|(1<<5)|(1<<6))) Serial.print(F(" JTAG/CSU"));
  Serial.println();

  loadPersist();
  Serial.print(F("persist: phase=")); Serial.print((int)persist.phase);
  Serial.print(F(" boots=")); Serial.println((int)persist.boots);
  // THE brownout discriminator: a power-on reset is normal on the bench, but a power-on reset while the
  // saved phase says we were flying means the rail collapsed in flight. Printed loudly because this is the
  // open ASC036 question (its log stopped mid-flight with no recorded reason).
  if(persist.phase!=0 && (resetCause & (1<<0)))
    Serial.println(F("*** BROWNOUT: power was lost MID-FLIGHT (POR with an in-flight saved phase) ***"));
  resumeAfterReset();     // if we rebooted mid-flight AND the sensors agree we are flying: protect the chute.
                          // Returns immediately in every normal power-on. Never touches P3, never resumes TVC.
}

// -- Main loop -----------------------------------------------------------------
void loop(){
  // ARM
  LED(true,true,true); delay(500); LED(false,false,false);
  while(!buttonCount()) delay(50);
  delay(500);
  if(!countdown()){                       // pad abort -> wait for button, restart arm
    LED(true,false,false);
    while(!exitOnButton()) delay(50);      // exitOnButton (not a bare digitalRead) so the press is CONSUMED --
    LED(false,false,false);                // otherwise buttonCount() sees the same press as arming press #1
    return;
  }

  // ASCENT -- TVC during burn, neutral + coast after burnout, until apogee (or hard timeout)
  launchTime=millis();
  LED(true,true,false);
  poweredFlight=true; inFlight=true;
  // backup chute timer (baro-fail failsafe): BACKUP_AFTER_BURNOUT_S after burnout. Kinematics (F-15+818 g) put
  // apogee ~2.5 s after burnout, so this fires ~1.5 s into descent -- past apogee, low speed. See the note above.
  float _backupS = IGN_DELAY + BURN_TIME + BACKUP_AFTER_BURNOUT_S;
  unsigned long backupMs=(unsigned long)(_backupS*1000.0f);
  Serial.print(F("backup chute @ s after launch: ")); Serial.println(_backupS,2);
  bool apogee=false;
  while(!apogee){
    unsigned long sinceLaunch  = millis()-launchTime;
    unsigned long nominalBurnoutMs = (unsigned long)((BURN_TIME+IGN_DELAY)*1000.0f);
    unsigned long tvcEndMs         = nominalBurnoutMs + (unsigned long)(TVC_EXTEND_S*1000.0f);
    if(sinceLaunch < tvcEndMs){
      // Past the NOMINAL burnout we keep vectoring, but the emergency-abort gate now follows the
      // ACCELEROMETER rather than the clock: poweredFlight stays true only while thrust is genuinely
      // still there. Without this, extending the window would also extend the >45 deg chute trigger into
      // a normal thrust-free coast -- where large tilt is EXPECTED (MC coast-tilt p90 is ~77 deg) -- and
      // would deploy the chute at ~30 m/s for no reason. Coast reads ~0 g and thrust ~3 g, so the two are
      // cleanly separated; falling below the threshold merely closes the abort window slightly early,
      // which is the safe direction.
      if(sinceLaunch >= nominalBurnoutMs) poweredFlight = (imu_az > THRUST_ONSET_G*G);
      TVC();
      captureMotor();              // in-flight motor characterization: sample axial thrust/mass + detect onset
    } else {
      if(poweredFlight){                   // FIRST coast iteration == the burnout transition: latch the ramp start
        poweredFlight=false;               // burnout: no thrust -> no control authority; emergency deploy off
        burnoutRampActive=true; burnoutT0=millis(); burnoutSx=tiltX; burnoutSy=tiltY;
        setPhase(2);                       // coast. Safe place for the EEPROM write: control is already over, so a
                                           // multi-ms flash stall here cannot affect anything.
      }
      centerServosRamp();                  // ease to neutral over ~1 s, then hold neutral (see the note at its definition)
      sensors();
    }
    // apogee: baro fell APOGEE_DROP_M below the peak AND descending -- the vert_vel<0 guard stops a boost baro
    // glitch from deploying the chute during powered ascent. OR the hard time backup (baro-failure safety).
    if(altitude < highest_alt - APOGEE_DROP_M && vert_vel < 0.0f) apogee=true;
    if(millis()-launchTime >= backupMs){ apogee=true; Serial.println(F("APOGEE (time backup)")); }
  }
  poweredFlight=false; inFlight=false;

  // RECOVERY
  neutralServos();
  beep(659,100); beep(523,100); beep(659,100);
  LED(false,true,true);
  triggerPyro(P4);                         // PARACHUTE (primary recovery)
  triggerPyro(P1);                         // LANDING LEGS -- deploy now so touchdown loads the legs, not the TVC
  setPhase(3);                             // chute is out: a later reboot must NOT re-run the resume watchdog
  dumpMotorLog();                          // write the flown motor's thrust curve + ignition lag -> MTR###.CSV
  dumpControlLog();                        // write the high-rate control trace + loop-period stats -> CTL###.CSV

  // RECOVERED -- locator beep until the button is pressed, then back to ARM (no power cycle needed)
  LED(true,true,true);
  Serial.println(F("RECOVERED"));
  while(!exitOnButton()){ beep(523,600); beep(392,600); updatePyros(); }
  setPhase(0); persist.boots=0; savePersist();   // flight over: clear the record so it can never arm a resume later
  LED(false,false,false);
  Serial.println(F("RECOVERED cleared by button -- returning to ARM"));
  return;      // loop() is re-entered by the core = ARM state. exitOnButton() has already disarmed every
               // pyro, and re-arming still needs 5 rapid presses + the 30 s countdown before P3 can fire.
}
