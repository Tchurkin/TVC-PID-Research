/*
  Ascent_TVC.cpp â€” dedicated pure-TVC ASCENT-TEST firmware (Teensy 4.1)
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

  âš  THE GAINS ARE STILL SIZED FOR THE OLD AIRFRAME (flagged 2026-08-04, researcher). They were
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
  float    gbx,gby,gbz;           // rad/s, pad gyro bias -- MPU-6050 (backup source)
  float    groundAlt;             // m, pad barometric reference
  uint32_t sum;                   // integrity check over the fields above
};

// Bench gyro-bias cache. DELIBERATELY a separate EEPROM record rather than extra fields in Persist:
// Persist is flight-critical brownout state that resumeAfterReset() keys on, and writing it from the
// calibration path perturbed the reboot-resume behaviour (regression 6b "boots=2 -> resumes" failed).
// A convenience cache for bench work has no business sharing a record with parachute recovery state.
struct BenchCal{
  uint32_t magic;
  float    gbx,gby,gbz;           // rad/s, MPU-6050
  float    ibx,iby,ibz;           // dps,   ICM-42688-P, firmware axes
  uint32_t sum;
};
constexpr int      BENCHCAL_ADDR  = 200;          // clear of Persist at the bottom of EEPROM
constexpr uint32_t BENCHCAL_MAGIC = 0xB1A5CA10;

// -- BOARD REVISION -----------------------------------------------------------
// 0 = Impulse 2.0/2.1 (the board that flew ASC007..ASC038)
// 1 = Impulse 2.2 (ordered 2026-08-01, PCBA). Adds rail-voltage sensing, pyro continuity sensing,
//     a sensor-cable detect, GPS and extra servo channels. Pin map extracted directly from
//     PCBs/Impulse_2.2_kicad/Impulse_2.2.kicad_pcb (the netlist is the ground truth, not the silk).
#define BOARD_IMPULSE_22  1

// ============================================================================================
// GROUND TEST MODE  --  1 = bench, 0 = FLIGHT
// ============================================================================================
// Set to 1 to exercise the full ARM -> countdown -> abort -> recovery sequence on the bench without
// a flight-ready pyro bank. The pre-arm gates (rails, parachute continuity, sensor rate, pad tilt)
// become LOUD WARNINGS instead of aborts, so a bench setup with no igniters fitted, no 7.4 V pyro
// pack and the airframe lying on its side can still run the sequence end to end.
//
// *** IT ALSO HARD-INHIBITS EVERY PYRO CHANNEL. THAT IS THE POINT, NOT A SIDE EFFECT. ***
// The obvious hazard of a "skip the safety checks" flag is that it survives to launch day. This one
// cannot usefully do so: with GROUND_TEST=1 triggerPyro() refuses every channel, so the motor cannot
// light and no recovery charge can fire either. The inhibit is at the DRIVER, not at the call sites.
//
// ⚠ THE FIRST VERSION OF THIS GOT IT WRONG, AND THE BENCH CAUGHT IT (CTL003, 2026-08-15).
// It inhibited only the motor channel and reasoned that with no ignition, liftoffConfirmed could never go true, so
// the existing liftoff interlock would hold the chute and leg charges. But liftoffConfirmed latches
// on a TRANSIENT above THRUST_ONSET_G (1.5 g) sampled at ~11.7 kHz, and ordinary bench handling
// produces that. It latched, the firmware took the normal recovery path, and both melt wires were
// energised on a bench. Reasoning that a safety property "follows from an existing mechanism" is
// exactly the step that needs testing rather than trusting -- the interlock guards "the motor never
// lit", which is a different proposition from "the vehicle never flew".
//
// Three further things make it hard to forget:
//   1. setup() prints a banner and sounds a distinctive triple chirp at every boot;
//   2. every bypassed gate says so on the console and flashes red+blue rather than the abort alarm;
//   3. regression.py REFUSES TO RUN with it set, and tells you to clear it. The flight gate is the
//      last thing between a build and the pad, so that is where the reminder belongs.
#define GROUND_TEST 1
// Runtime mirror of the build flag, for the same reason TVC_TORQUE_CMD is a variable: it lets the
// SIL exercise the inhibit inside a FLIGHT build (--groundtest), so "no pyro can fire in ground-test
// mode" is a gate-locked property instead of an argument. The build flag is still what the gate
// greps for, and in a flight build nothing ever sets this true.
bool    groundTestInhibit = (GROUND_TEST != 0);
uint8_t pyroAttemptMask   = 0;   // bits 0..3 = P1..P4 (ascent,descent,legs,chute) requested while inhibited. Doubles as the
                                 // positive control: it proves the firmware DID reach the code that
                                 // would have fired, so a passing test means the inhibit stopped it
                                 // rather than the path never being taken.

// ---- GT_PYRO_LIVE: let the RECOVERY channels actually fire during a ground test ----------------
// Set to 1 to bench-test the real fire path end to end -- FET switching, the 1000 ms pulse, and the
// post-fire continuity readback going CONT -> open. That last one is a genuine flight diagnostic
// (FAILURE_MODES B3: it separates "the firmware never fired" from "the firmware fired and the
// hardware did not respond") and it can only be exercised by actually burning something.
//
// *** THE MOTOR CHANNELS ARE NEVER RELEASED BY THIS. ***
// A ground test does not involve lighting a motor. That is enforced here rather than left as an
// intention, because "I won't light any motors" is a promise and a pin exclusion is a property. So the
// worst this flag can do is fire a recovery channel you chose to connect.
//
// USE A DUMMY LOAD FIRST -- a 10 ohm 1 W resistor, or an LED with a 470 ohm series resistor, across
// the channel. It proves the FET switches and the pulse is the right length while igniting nothing.
// Fit a real melt wire only after the dummy load has passed. Firmware/Impulse22_PyroTest does the
// same job for ONE chosen channel with a hold-to-fire interlock; prefer it if you only want to prove
// the hardware. Use this one when you specifically want the FLIGHT firmware's own sequencing.
#define GT_PYRO_LIVE 1
bool gtPyroLive = (GT_PYRO_LIVE != 0);   // runtime mirror so the SIL can lock the motor exclusion

// -- Pins (verify against your wiring) ----------------------------------------
// *** SERVO PIN CROSSING -- READ BEFORE "FIXING" THIS. ***
// The board netlist calls pin 3 SERVOX and pin 4 SERVOY. This firmware has always used
// SERVO_X_PIN=4 / SERVO_Y_PIN=3, i.e. the firmware's X channel drives the net named SERVOY.
// Impulse 2.2's netlist is verified IDENTICAL to 2.1, which is the board that flew ASC007-ASC038,
// so this crossing is what every successful flight used. It is consistent because the mechanical
// build was made to match it. DO NOT "correct" it to agree with the net names -- that swaps the
// control axes and is guaranteed loss of control. The bench pitch-the-nose test is the only
// authority, and it must be re-run on the new board before flight (new harness, new connectors).
constexpr int BUTTON = 14;
constexpr int BUZZER = 13;
// -- PYRO CHANNELS ------------------------------------------------------------
// PHYSICAL channels. These are board pins and do not move.
constexpr int P1     = 9;
constexpr int P2     = 10;
constexpr int P3     = 11;
constexpr int P4     = 12;
// ROLES. *** REASSIGNED 2026-08-15 (Braxton): ascent P1, descent P2, legs P3, chute P4. ***
// Previously: legs P1, n/c P2, ascent P3, chute P4 -- so ASCENT AND LEGS SWAPPED.
//
// ALWAYS FIRE BY ROLE, NEVER BY CHANNEL NUMBER. Every call site below uses these names, so a future
// rewire is this block and nothing else. That is not tidiness: the previous mapping had the
// ground-test motor inhibit written as `pin != P3`, and under the new mapping P3 is the LEGS -- so a
// straight renumber would have inhibited the legs and RELEASED THE ASCENT MOTOR on a bench test.
// A channel number cannot express "the thing that must never fire indoors"; a role can.
constexpr int PYRO_ASCENT  = P1;   // ASCENT motor ignition
constexpr int PYRO_DESCENT = P2;   // DESCENT/landing motor ignition -- NEWLY LIVE, was "not connected"
constexpr int PYRO_LEGS    = P3;   // LANDING LEGS deploy (melt wire) -- protects the TVC on touchdown
constexpr int PYRO_CHUTE   = P4;   // PARACHUTE melt wire (primary recovery)
// THIS AIRFRAME / NEXT FLIGHT uses P1 (ascent) and P4 (chute) ONLY. P2 and P3 are wired but unused:
// there is no landing burn and no leg deploy on this flight. Nothing below fires PYRO_DESCENT, and
// PYRO_LEGS is fired only on the recovery path, which is harmless with no melt wire fitted.
constexpr int RLED   = 6, GLED = 7, BLED = 8;
// Pack-status LEDs, readable on the pad with no laptop. Unlike the RGB (common anode on 5V-CLEAN),
// these are ordinary discretes: pin -> 330R -> LED -> GND, so they are ACTIVE HIGH.
// -- SENSOR BOARD, SPI1 --------------------------------------------------------
// Declared HERE with the rest of the pin map rather than beside the drivers 600 lines below.
// Chip selects living next to the code that toggles them is how you end up with two ideas about
// which pin is which.
constexpr int PIN_CS_IMU  = 0;    // ICM-42688-P
constexpr int PIN_CS_BARO = 15;   // DPS310
constexpr int PIN_MISO1   = 1, PIN_MOSI1 = 26, PIN_SCK1 = 27;

// -- SIGNAL POLARITY -- every one of these has bitten this project ------------
//   SENS_DET (32)   ACTIVE LOW.  Sensor board ties J1 pad 12 to GND; LED4+R64 pull up from
//                                5V-CLEAN. CONNECTED = LOW. Reading it as active-high halted the
//                                board on every boot with the harness correctly plugged in.
//   RGB LED1 (6/7/8) COMMON ANODE on 5V-CLEAN -> a channel lights when its pin is pulled LOW, and
//                                "off" must be HIGH-Z: driven HIGH leaves 1.7 V across the red die,
//                                right at its knee, and it glows.
//   PACK LEDs (30/31) ACTIVE HIGH. Ordinary discretes: pin -> 330R -> LED -> GND.
//   BUTTON (14)     ACTIVE HIGH, external 10k pulldown (R23). Do NOT use INPUT_PULLUP.
//   PYRO gates      ACTIVE HIGH, external 10k pulldowns (R50-R53) hold them off while the MCU boots.
constexpr int PIN_MAIN_LED = 30;   // LED2, green  -- main pack (VBAT)
constexpr int PIN_PYRO_LED = 31;   // LED3, yellow -- pyro pack (7.4 V)
constexpr int SD_CS  = BUILTIN_SDCARD;
constexpr int SERVO_X_PIN = 4, SERVO_Y_PIN = 3;   // crossed vs the net names ON PURPOSE -- see the note above

#if BOARD_IMPULSE_22
// ---- Impulse 2.2 additions. All pins taken from the PCB netlist, not the silkscreen. ----
// Rail voltage sensing. Resistor values read out of the board file; the ADC is 3.3 V full scale.
//   VBAT_SENSE (21): R54 10k / R55 4.7k  -> pin sees VBAT * 4.7/14.7 = /3.128   (max readable 10.3 V)
//   SERVO_V_SENSE(22): R56 10k / R57 10k -> pin sees 5V-DIRTY / 2.0             (max readable  6.6 V)
//   PYRO_V_SENSE (23): R58 100k / R59 47k-> pin sees 7.4V rail * 47/147 = /3.128(max readable 10.3 V)
// A 2S LiPo (6.4-8.4 V) lands at 2.05-2.69 V on VBAT_SENSE, comfortably inside range.
constexpr int PIN_VBAT_SENSE = 21, PIN_SERVO_V_SENSE = 22, PIN_PYRO_V_SENSE = 23;
constexpr float VBAT_DIV = 3.128f, SERVO_V_DIV = 2.0f, PYRO_V_DIV = 3.128f;
// Pyro continuity. Each channel: 10k from the pyro OUTPUT node to PYROn_SENSE, and a red LED from
// PYROn_SENSE to GND. With the FET off and an igniter/melt-wire CONNECTED, current flows through the
// wire and the LED clamps the sense node near its forward drop (~1.6-2.0 V). With the wire OPEN or
// missing, no current flows and the node sits near 0 V. So:
//     PYROn_SENSE HIGH -> igniter present and continuous
//     PYROn_SENSE LOW  -> open circuit: not connected, broken, or ALREADY BURNED THROUGH
// That last case makes this a post-fire confirmation as well as a pre-arm check -- ASC038 had no way
// to tell whether the melt wire ever burned, which is exactly the unknown in its recovery timeline.
constexpr int PIN_PYRO_SENSE[4] = {38, 39, 40, 41};       // indexed P1..P4 = ascent, descent, legs, chute
// RAISED 0.9 -> 1.5 V on 2026-08-15: 0.9 V was giving FALSE POSITIVES on the bench, i.e. reporting
// an igniter present on an open channel. An open node is only pulled to 0 V by leakage through the
// LED, so it floats higher than the design assumed, and 0.9 V sat inside that float.
//
// ERROR DIRECTION, and why raising it is the safe way to be wrong. A false POSITIVE is the dangerous
// failure: the board believes the P4 melt wire is good, arms, flies, and there is no parachute --
// which is exactly the unknown ASC038 could not resolve. A false NEGATIVE only refuses to arm on the
// pad. Given a choice, this threshold should fail toward "refuse", so it belongs nearer the top of
// the gap rather than the middle.
//
// ! MARGIN WARNING: the LED forward drop is ~1.6-2.0 V, so 1.5 V leaves only ~0.1 V against the LOW
// end of that range. If a channel starts reading open when an igniter IS fitted, this is why, and the
// fix is to measure rather than nudge -- printPyroStatus() prints all four channels in volts.
// Read a KNOWN-OPEN and a KNOWN-CONNECTED channel, then put this constant midway between the two
// observed levels. That is a five-minute bench measurement and it ends the guessing.
constexpr float PYRO_CONT_V = 1.5f;                        // V at the pin above which we call it continuous
constexpr int PIN_SENS_DET = 32;                           // sensor-board ribbon present (J_SENSOR1 pin 12)
constexpr float ADC_VREF = 3.3f;
constexpr int   ADC_BITS = 12, ADC_MAX = 4095;
// Pre-arm limits. A 2S LiPo at 7.4 V nominal; 7.0 V loaded is a tired pack and 6.6 V is nearly flat.
// The bench session of 2026-08-03 found a flat pack stalling a servo and browning out the board, and a
// flat pack is the leading suspect for ASC036/ASC038 -- so this refuses to ARM rather than warning.
constexpr float VBAT_ARM_MIN_V  = 7.20f;
constexpr float SERVO_V_MIN_V   = 4.50f;   // 5 V rail, loaded
constexpr float PYRO_V_MIN_V    = 6.80f;   // must be able to heat a melt wire
float vbatV=0, servoV=0, pyroV=0;          // latest readings, logged every cycle
float vbatMinV=99, servoMinV=99, pyroMinV=99;   // in-flight MINIMA -- the brownout evidence ASC038 lacked
#endif

// -- TVC / control (PD; identical form to the flown Ascent_Test) ---------------
// Servo neutral trim, in COMMAND degrees. write(90 + TUNE) must point the nozzle straight.
// MEASURED on the rebuilt airframe 2026-08-06: X centres at 1360 us, Y at 1520 us.
// CONVERSION -- the trap here is that PWMServo's neutral is NOT the classic 1500 us. attach(pin)
// defaults to a 544-2400 us range over 0-180 deg (PWMServo.h:90), so write(90) emits 1472 us and the
// scale is 10.3111 us per command degree. Working in a 1000-2000 us head model puts Y off by ~4.7 deg.
//   X: (1360-1472)/10.3111 = -10.86  -> -11  ->  write(79) = 1358.6 us  (1.4 us from target)
//   Y: (1520-1472)/10.3111 =  +4.66  ->  +5  ->  write(95) = 1523.6 us  (3.6 us from target)
// Rounded to integers on purpose: write() takes an int, so +4.66 would truncate to write(94) = 1513 us,
// which is FURTHER from 1520 than write(95). Trim resolution is one command degree = 10.3 us =
// 0.222 deg of gimbal at the 4.5:1 linkage -- finer than that needs a linkage change, not a constant.
constexpr float XTUNE = 0.0f, YTUNE = 0.0f;
constexpr int   SERVO_X_SIGN = -1;
// WHOLE-CHANNEL inversion applied ONLY when the primary (ICM-42688-P) is the active source.
// Bench 2026-08-13: with the ICM, the entire X channel -- P and D together -- drove the nose the
// wrong way. It cannot be fixed with ICM_SGN_*, because those are per SENSOR axis and X's P comes
// from accel Y while its D comes from gyro X, so either would fix one term and break a Y term.
// It must NOT be fixed by flipping SERVO_X_SIGN either: doing that turned the simulated flight into
// a tumble, because the SIL models the MPU-6050 frame that SERVO_X_SIGN was verified against.
// So: correct the ICM relative to the MPU, and leave the MPU/SIL path exactly as proven.
// ================= EVERY SIGN CONVENTION, IN ONE PLACE =======================================
// Three separate knobs act at three different points, and confusing them cost a full bench session.
// The trap is that SENSOR axes cross over into CONTROL channels:
//
//     control channel X  <-  P from accel Y    D from gyro X
//     control channel Y  <-  P from accel X    D from gyro Y
//
// so no single per-axis constant can fix one channel -- it always fixes one term and breaks another.
//   ICM_MAP_* / ICM_SGN_*   per SENSOR axis, applied in icm42688Read(). Flips accel and gyro
//                           together, which is what keeps the frame self-consistent for the
//                           quaternion. Use for a rotated/mirrored sensor mounting.
//   ICM_CHAN_*_SIGN         per CONTROL channel, ICM only, applied in writeServos(). Flips P and D
//                           together on one channel. Use when a whole channel drives the wrong way.
//   SERVO_*_SIGN            per control channel, ALL sources. Verified 2026-08-03 against the
//                           MPU-6050 -- changing it breaks the SIL, which models that frame.
constexpr int   ICM_AXES_VERIFIED = 1;                    // <<< SET after the bench check
constexpr int   ICM_MAP_X = 0, ICM_MAP_Y = 1, ICM_MAP_Z = 2;   // raw axis feeding firmware X/Y/Z
// X flipped from the bench, 2026-08-13: with +1 the gimbal drove the nose the WRONG way on the X
// channel. Corrected HERE rather than at SERVO_X_SIGN because SERVO_X_SIGN was verified 2026-08-03
// against the MPU-6050 with these same servos and this same linkage, neither of which has changed --
// the only new variable is the IMU. Flipping the sensor axis also flips accel and gyro together, so
// the frame stays self-consistent and the LOGGED tilt sign is right; flipping the servo instead
// would have fixed the loop while leaving every flight log recording X backwards.
constexpr float ICM_SGN_X = +1.0f, ICM_SGN_Y = +1.0f, ICM_SGN_Z = +1.0f;
// ---- ICM UI FILTER BANDWIDTH (GYRO_ACCEL_CONFIG0, register 0x52) ----------------------------
// Found 2026-08-15: icm42688Begin() never wrote this register, so the UI filter bandwidth -- which
// is a DIRECT contributor to loop delay tau, the quantity the whole gain design is indexed on -- was
// left at whatever the part powers up with. Not wrong, but set by accident, and a tau measurement
// against an accidental configuration is a measurement of an accident.
// DELIBERATELY NOT CHANGED. 0xFF means "leave the power-on default alone" and is the flight setting:
// altering the filter changes both the noise the D term sees and the delay, neither of which this
// project's simulator can express, and doing that between a tau measurement and a launch would
// invalidate the measurement it is meant to inform. What IS added is a READBACK, printed at boot and
// written to the CTL header, so every flight and every bench run records the filter it actually ran.
// TO CHANGE IT LATER: set this to the register value you want (bits [7:4] accel, [3:0] gyro), re-run
// Bench_Tau_ICM to get the new tau, and re-run the gate. Do not change it and fly the same day.
constexpr uint8_t ICM_UI_FILT_CFG = 0xFF;    // 0xFF = do not write; any other value is written to 0x52
uint8_t icmUiFiltReadback = 0x00;            // what 0x52 actually reads back after configuration
constexpr int   ICM_CHAN_X_SIGN = -1;
constexpr int   ICM_CHAN_Y_SIGN = +1;            // *** SET FROM THE BENCH, 2026-08-03: with +1 BOTH axes drove the nose the WRONG
constexpr int   SERVO_Y_SIGN = -1;            // way (pitch-the-nose test), so both were flipped to -1 and the test re-run. ***
                                              // The bench test is the ONLY authority for these two constants. Everything below is
                                              // context for why, not a reason to override it.
                                              //
                                              // âš  UNEXPLAINED, RESOLVE BEFORE TRUSTING THE AIRFRAME: flight ASC036 (2026-07-20)
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

// ============================================================================================
// TORQUE-COMMANDED TVC â€” the plant is re-identified in flight instead of assumed constant
// ============================================================================================
// THE PROBLEM. Everything above pole-places ONE keff. But keff = T*L/Iyy and none of the three are
// constant: the F15 spikes to ~26 N then settles to ~14 N (the comment at P_GAIN already concedes
// "keff 215-520 -> zeta 0.9-1.4"), and 60 g of propellant -- 6.5% of the vehicle -- leaves the aft
// end, moving the CG forward and dropping Iyy. So the loop is ~36% faster and noticeably
// under-damped (zeta ~0.73) exactly at ignition, which is when tip-off happens and when damping
// matters most. It is also why the gains go stale every time the airframe is rebuilt.
//
// THE FIX. P_GAIN and D_GAIN were never independent numbers -- they are wn^2/keff and 2*zeta*wn/keff
// evaluated once, at keff = 130. Do that division LIVE against the keff the vehicle actually has and
// the closed-loop poles stop moving: the PD law asks for an angular ACCELERATION, and the gimbal
// angle needed to deliver it is alpha/keff. Same design point, held for the whole burn.
//
// WHAT IS MEASURED vs ASSUMED. Thrust is not assumed from a curve -- it is MEASURED, because the
// accelerometer already reads specific force along the body axis: under thrust f_z = (T - D)/m, so
// T = m*a_z + D. That is the dominant term (T swings ~85%, mass properties only ~11%), and measuring
// it means a motor that underperforms, or lights late, is handled without being told.
//
// WHAT YOU MUST ENTER. Dry properties + the propellant, from which the wet state is derived; the
// firmware prints BOTH at boot so the derived wet numbers can be checked against a bifilar swing.
// >>> The three DRY_* values below are BACK-DERIVED from the measured WET values (0.927 kg, L 0.16 m,
// >>> Iyy 0.0176) by removing 60 g of F15 propellant at PROP_ARM_M. They are therefore only as good as
// >>> PROP_ARM_M. Measuring the DRY vehicle directly (spent casing fitted) is strictly better -- do
// >>> that and overwrite these three, then check the printed wet numbers against the bifilar result.
float DRY_MASS_KG  = 0.867f;   // NOT constexpr: the SIL overrides these (--drymass/--dryl/--dryiyy) to   // kg,      all-up mass with an EMPTY motor casing fitted
float DRY_L_M      = 0.1683f;  // prove that entering a rebuilt airframe's numbers really does re-derive  // m,       DRY CG -> gimbal pivot
float DRY_IYY      = 0.01666f; // the gains. On hardware they are edited here and behave as constants. // kg*m^2,  pitch/yaw inertia about the DRY CG
// Propellant, as a body at PROP_ARM_M forward of the gimbal pivot. Estes F15: 60 g, 49.6 N*s.
constexpr float PROP_MASS_KG = 0.060f;   // kg
constexpr float PROP_ARM_M   = 0.040f;   // m, gimbal pivot -> propellant centroid.  <<< MEASURE THIS
constexpr float PROP_LEN_M   = 0.070f;   // m, grain length   ) its own inertia is ~0.2% of Iyy, so
constexpr float PROP_DIA_M   = 0.024f;   // m, grain diameter ) these two barely matter
constexpr float MOTOR_IMPULSE_NS = 49.6f;// N*s total impulse -- sets how burn fraction is scored
constexpr float PROP_IOWN = PROP_MASS_KG*(3.0f*(PROP_DIA_M*0.5f)*(PROP_DIA_M*0.5f)
                                          + PROP_LEN_M*PROP_LEN_M)/12.0f;   // solid cylinder, transverse

// THE ACTUAL DESIGN PARAMETERS. These, not P_GAIN/D_GAIN, are what you tune now: they are the
// closed-loop response you want, and they do not change when the airframe does.
// NOT constexpr so the SIL can sweep them (--wn / --zeta).
float TVC_WN   = 8.0f;                   // rad/s, closed-loop natural frequency
float TVC_ZETA = 1.0f;                   // 1.0 = critically damped, no overshoot
// >>> THESE ARE NOT AT THE OPTIMUM, AND THE SWEEP SAYS SO. <<<
// Full (wn, zeta) grid, 70 dispersed airframes per cell, servo lag dispersed:
//        z=0.707   z=0.85    z=1.00    z=1.20    z=1.40      (PASS % / boost-tilt p90 deg)
//   wn= 6  54.3%    61.4%     67.1%     72.9%     75.7%
//   wn= 8  90.0%    92.9%     92.9% <-- flying    94.3%
//   wn=10  92.9%    94.3%     94.3%     95.7%     95.7%
//   wn=12  94.3%    95.7%     95.7%     95.7%     95.7%
//   wn=14  95.7%    95.7%     95.7%     95.7%     95.7%
// wn=12 / zeta=1.2 gives 95.7% and p90 boost tilt 3.91 deg against 8.01 deg here -- and it HOLDS at
// every servo lag tested, 0.015 s through 0.160 s (4.5x the assumed tau), with no crossover.
//
// WHY THERE IS NO CEILING, AND WHY THAT IS NOT A CONTRADICTION: this project's own gain-ceiling
// result was measured with Kd FROZEN while Kp was pushed -- sequential tuning. Raising wn and zeta
// together holds Kd/Kp fixed, which is coupled tuning, and the 2026-08-03 replication (n=2400) is
// precisely the finding that the ceiling is an artifact of moving them independently. The firmware
// and the research agree here, which is a real cross-check rather than a coincidence.
//
// NOT CHANGED YET, DELIBERATELY. The simulator cannot express servo backlash or deadband (see
// FAILURE_MODES "known physics simplifications"), and backlash-driven limit cycles are exactly what
// bites at raised gain. Servo-angle quantisation IS modelled (PWMServo takes integer degrees, and the
// harness quantises identically), so that much is covered -- but not the mechanical slop.
// TO ADOPT: set 12.0 / 1.2 here, then bench-verify BEFORE flight -- pitch the nose and confirm the
// gimbal still settles without buzzing or hunting at the new gain. If it hunts on the bench, the
// linkage slop is the limit and this stays at 8.0 / 1.0.
bool TVC_TORQUE_CMD = true;              // false = fall back to the fixed P_GAIN/D_GAIN of every prior
                                         // flight. A VARIABLE, not a #define, so the SIL can A/B both
                                         // control laws in one binary (--fixedgain) and the regression
                                         // suite can lock the legacy path instead of letting it rot.

// Bounds on the scheduled gain. A gimbal command is alpha/keff, so a keff estimate that collapses
// toward zero -- burnout, a bad accelerometer, a NaN -- would demand an infinite deflection. Outside
// this band the estimate is REJECTED and the last good value held, which is why keffEst is seeded
// with the nominal rather than zero.
constexpr float KEFF_MIN = 40.0f;        // rad/s^2 per rad; below this the motor is effectively out
constexpr float KEFF_MAX = 600.0f;
constexpr float GAIN_SCHED_MIN = 0.35f;  // hard clamp on keffNominal/keffEst, belt and braces
constexpr float GAIN_SCHED_MAX = 2.50f;
constexpr float THRUST_TAU_S = 0.10f;    // s, low-pass on the thrust estimate. The accelerometer is
                                         // buried in motor vibration; thrust itself moves on ~0.2 s.
constexpr float RHO_AIR   = 1.20f;       // kg/m^3   ) drag correction to T = m*a_z + D. Small (~7% at
constexpr float S_REF_M2  = 0.003f;      // m^2      ) 30 m/s) but systematic, and it grows through the
constexpr float CD_AXIAL  = 0.60f;       //          ) burn exactly where the gain matters most.

// ---- F15 REFERENCE THRUST CURVE + IN-FLIGHT SYSTEM ID ----------------------------------------
// Two ways to know the thrust, and neither is sufficient alone:
//   the CURVE knows the SHAPE       -- it anticipates the 26 N spike instead of lagging it through a
//                                      filter, and it covers the first ~100 ms before any estimate
//                                      has settled. But it does not know THIS motor.
//   the ACCELEROMETER knows the TRUTH -- T = m*a_z + D, so a motor that runs hot, cold, or dies is
//                                      seen. But it is buried in motor vibration and must be filtered,
//                                      which costs exactly the responsiveness the spike needs.
// So: take the shape from the curve and IDENTIFY THE AMPLITUDE IN FLIGHT. thrustScale is the system-ID
// output -- one well-conditioned scalar, heavily filterable without blurring the curve's fast edges,
// and logged so a flight tells you "this motor ran 6% hot" as a number.
//
// Time base is measured from THRUST ONSET, not from the ignition command: measured lag is 462 +- 58 ms
// across three flights and was 920 ms on ASC038, so indexing the curve off the command would smear it
// by more than the spike is wide.
//
// The table is hand-digitised from the published Estes F15 curve (49.6 N*s, 14.34 N avg, 3.45 s) and
// integrates to 49.65 N*s / 14.39 N avg. It is a PRIOR, not truth -- after a flight, MTR###.CSV holds
// the curve this vehicle actually flew and can be used to refine it.
constexpr int   F15_N = 14;
constexpr float F15_T[F15_N] = {0.00f,0.05f,0.10f,0.15f,0.20f,0.30f,0.40f,0.50f,
                                1.00f,1.50f,2.00f,2.50f,3.00f,3.45f};
constexpr float F15_F[F15_N] = {0.0f, 12.0f,22.0f,25.8f,23.5f,19.0f,16.5f,15.4f,
                                15.0f,14.6f,14.2f,13.9f,13.5f, 0.0f};

float thrustRefN(float tau){                    // N at tau seconds after thrust onset
  if(tau <= 0 || tau >= F15_T[F15_N-1]) return 0.0f;
  for(int i=1;i<F15_N;i++){
    if(tau <= F15_T[i]){
      float f = (tau - F15_T[i-1]) / (F15_T[i] - F15_T[i-1]);
      return F15_F[i-1] + f*(F15_F[i] - F15_F[i-1]);
    }
  }
  return 0.0f;
}
constexpr float THRUST_SCALE_TAU_S = 0.30f;     // s, adaptation rate of the identified amplitude.
                                                // Fast enough to track a genuinely odd burn out of the
                                                // curve's shape, slow enough to ignore vibration.
constexpr float THRUST_SCALE_MIN = 0.40f;       // a motor this far off the curve is a different failure;
constexpr float THRUST_SCALE_MAX = 1.60f;       // poweredFlight (axial g) is what stops TVC, not this.

// Live plant estimate (logged, so a flight can be checked against this model afterwards).
float plantMass = DRY_MASS_KG + PROP_MASS_KG;   // kg
float plantL    = 0;                            // m, CG -> gimbal pivot
float plantIyy  = 0;                            // kg*m^2
float plantThrustN = 0;                         // N, curve shape * identified amplitude
float thrustMeasN  = 0;                         // N, raw accelerometer-derived (logged for comparison)
float thrustScale  = 1.0f;                      // THE system-ID output: this motor vs the F15 curve
float keffEst   = 130.0f;                       // rad/s^2 per rad -- seeded at the nominal design point
float keffNominal = 130.0f;                     // computed at boot from the numbers above
float burnFrac  = 0;                            // 0 = full grain, 1 = spent
float impulseNs = 0;                            // N*s delivered so far
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
//   3. âš  MISALIGNMENT BUDGET -- still the thing that can lose the vehicle, but it now closes. A thrust
//      misalignment must be cancelled by the gimbal 1:1, straight off the authority above. At +-5.00 deg
//      against ASC007's measured 4.28 deg misalignment there is 0.72 deg left for control -- tight but
//      positive. Under the old (wrong) 4.09 deg figure the budget was NEGATIVE, i.e. that vehicle could not
//      have cancelled its own misalignment. Still boresight the nozzle before flying; the flight measures it
//      for you, since the steady integral-trim value IS the misalignment (ASC036 read 0.66 deg that way).
//
// âš  REMAINING RISK -- commanded == physical means MAX_TILT now drives the gimbal to EXACTLY 5.00 deg, and the
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
constexpr float MAX_TILT = 5.0f;              // deg, TVC command limit BEFORE the linkage multiply.
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
// Forward-declared on purpose. The Arduino IDE hoists auto-generated prototypes so it would build
// without this, but the SIL compiles the .ino as plain C++ with no hoisting -- and altitude is read
// (line ~1150) well before the sensor-board drivers that define this are declared.
float baroReadAltitude(float seaLevelHpa);

// -- Recovery / safety ---------------------------------------------------------
constexpr float EMERGENCY_TILT   = 45.0f;     // deg: |tilt| beyond this DURING BURN -> abort + chute
// SECOND, ATTITUDE-INDEPENDENT ABORT -- added 2026-08-04 after ASC038.
// The tilt abort above reads gyro_x/gyro_y, which come from the quaternion estimator. On ASC038 the
// estimator froze (see sensors()) and the tilt abort was therefore BLIND: the vehicle tumbled past
// 160 deg at 248 deg/s and the abort could not fire, because as far as it could tell the tilt was
// 2.4 deg. ANY abort that depends on the estimator dies with the estimator. This one reads the RAW
// bias-removed gyro directly -- no quaternion, no EMA, no dt integration -- so it survives an
// estimator failure, a frozen loop, or a bad attitude initialisation.
// Threshold: the SIL Monte Carlo puts boost body rate at p90 ~53 deg/s and max ~103 deg/s on healthy
// flights, ASC007 peaked at 145, ASC038 reached 248. 150 deg/s sustained for 300 ms is comfortably
// above any healthy flight (including a hard rail tip-off, which is brief) and well below a real
// tumble. The dwell requirement is what keeps a transient from tripping it.
constexpr float EMERGENCY_RATE_DPS = 150.0f;  // deg/s, |pitch/yaw body rate|, RAW
constexpr unsigned long EMERGENCY_RATE_MS = 300;   // must persist this long before it fires
// BLIND ABORT -- added 2026-08-08. Both aborts above read the IMU, so losing every inertial source
// silences BOTH of them: measured with --imustuck at t=0.5 s, the vehicle reached 179.8 deg and
// 800 deg/s with ZERO aborts raised, then hit the ground with the chute still packed. A detector
// that shares its only sensor with the failure it is meant to catch is not a detector.
// We cannot see the tumble, so we assume it. Control is already inhibited by then (gimbal neutral,
// integrators zeroed) and the airframe is aerodynamically unstable without control, so the outcome
// is not in doubt -- only whether the chute is out when it arrives. Losing the mission beats losing
// the airframe. The dwell rejects a momentary double-bus glitch; it is short because at a 0.5 s
// failure the vehicle only reaches ~7 m, so there is no time to spend waiting for burnout.
constexpr unsigned long BLIND_ABORT_MS = 250;      // no trustworthy IMU for this long -> deploy
constexpr float APOGEE_DROP_M    = 1.0f;      // m below peak to call apogee (baro)
// PRIMARY DEPLOY TIMER -- added 2026-08-04 after ASC038's canopy appeared ~1.5 m above the ground.
// Deploy this long after TVC ends (TVC ends at IGN_DELAY+BURN_TIME+TVC_EXTEND_S = 4.65 s), whether or
// not the barometer has called apogee. Whichever fires FIRST wins, so this can only ever make
// deployment earlier, never later. It removes the flight's dependence on a baro/velocity trigger that
// was 0.87 s late on ASC038 (VEL_ALPHA=0.2 EMA lag + the 1 m APOGEE_DROP_M).
// WHY +1.0 s AND NOT AT TVC CUTOFF: at 4.65 s the vehicle is still CLIMBING in most cases -- SIL across
// motor/ignition dispersions gives worst-case +19.3 m/s at 4.65 s vs +8.6 m/s at 5.65 s. Deploying into
// a fast climb risks the canopy. Going later still (6.65 s) puts every case in descent, but the
// weak-motor case apogees at 4.85 s / 32 m and would then burn 16 m of the altitude the melt wire needs.
// 5.65 s is the balance, and it is also exactly when the burnout servo ramp completes.
constexpr float DEPLOY_AFTER_TVC_S = 1.0f;
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
unsigned long rateHighSince = 0;   // ms when the raw body rate first exceeded EMERGENCY_RATE_DPS (0 = not high)
unsigned long blindSince    = 0;   // ms when the last inertial source died (0 = we can still see)

// ================= FAULT MANAGEMENT / DEGRADATION LADDER =====================================
// Design intent (2026-08-04, after ASC038): a subsystem failure should cost a LEVEL OF SAFETY,
// not the vehicle. The flight computer detects its own faults, sheds the least important work
// first, falls back to simpler and more proven behaviour, and records how far down the ladder it
// went. A flight that lands with healthLevel=1 flew fine and told you something broke; a flight
// that lands at 3 is a near miss you can learn from. Nothing here is allowed to make a healthy
// flight worse -- every detector requires SUSTAINED evidence, and every response is conservative.
//
//   0 NOMINAL   full quaternion attitude, PD+I, full logging
//   1 REDUCED   non-critical work shed (baro rate cut, logging cut). Control UNAFFECTED.
//   2 FALLBACK  attitude no longer trusted -> rate damping only, and deploy early. Cannot hold
//               attitude (there is no second attitude source on this vehicle -- under thrust the
//               accel reads thrust, in coast it reads free-fall), so this mode buys a calmer
//               descent into recovery, not a controlled flight. Being honest about that matters.
//   3 SURVIVE   control not trustworthy at all -> gimbal neutral, protect the parachute.
enum { HEALTH_NOMINAL=0, HEALTH_REDUCED=1, HEALTH_FALLBACK=2, HEALTH_SURVIVE=3 };
uint8_t  healthLevel = HEALTH_NOMINAL;
uint32_t faultFlags  = 0;
#define F_LOOP_SLOW     (1u<<0)   // loop period exceeded DT_WARN_S repeatedly
#define F_BARO_SHED     (1u<<1)   // baro polling rate cut to buy loop time
#define F_LOG_SHED      (1u<<2)   // flight logging cut, then stopped
#define F_IMU_STUCK     (1u<<3)   // IMU returned identical samples -> bus wedged or device dead
#define F_IMU_SLOW      (1u<<4)   // a single IMU read took absurdly long (I2C stretching/hang)
#define F_ATT_STALE     (1u<<5)   // attitude not tracking the measured body rates
#define F_OSCILLATION   (1u<<6)   // command ringing against the stops
#define F_GAIN_BACKOFF  (1u<<7)   // gains halved in response to F_OSCILLATION
#define F_RAIL_SAG      (1u<<8)   // a monitored rail went below its in-flight floor
#define F_RATE_ABORT    (1u<<9)   // estimator-independent abort fired
// Latching: flags never clear and the level never drops. The point is the FLIGHT RECORD -- you want
// to know the worst state reached, not the state at touchdown. Recovering a level mid-flight would
// also mean re-trusting a subsystem that has already lied once, which is not a good trade.
// ---- HARDWARE WATCHDOG -----------------------------------------------------------------------
// The last line of defence, and the only thing that can rescue a genuinely hung loop -- an I2C
// device holding SDA low blocks inside the Adafruit driver forever and no software check can escape
// it (FAILURE_MODES.md D2). On a watchdog reset the EEPROM record survives, resumeAfterReset() sees
// an in-flight phase, corroborates with the sensors and deploys the parachute. That turns "the board
// hung and the rocket augered in" into "the board reset and it came down under canopy".
// Timeout is DELIBERATELY generous (4 s): every normal path here is well under 1 s except the log
// dumps, and a watchdog that trips on healthy code is a reset loop on a flight computer. WDE cannot
// be cleared once set, so this is a commitment -- feed it in every loop that can run longer than a
// few hundred ms. Set WATCHDOG_ENABLE 0 to disable if it ever misbehaves on the bench.
#define WATCHDOG_ENABLE 1
#define WATCHDOG_SECONDS 4
// The SIL branches below are why the watchdog is testable at all. Before 2026-08-09 both of these were
// no-ops off-target, so in simulation the watchdog did not exist: a hung I2C bus simply spun forever
// and the hang -> WDT reset -> resumeAfterReset -> chute chain -- the entire reason the watchdog is
// enabled -- had never once been exercised. The harness now implements the timer.
inline void wdtFeed(){
#if WATCHDOG_ENABLE && defined(__IMXRT1062__)
  WDOG1_WSR = 0x5555; WDOG1_WSR = 0xAAAA;    // the required two-word service sequence
#elif WATCHDOG_ENABLE
  { extern void sim_wdtFeed(); sim_wdtFeed(); }
#endif
}
inline void wdtStart(){
#if WATCHDOG_ENABLE && defined(__IMXRT1062__)
  // WT field is (timeout/0.5 s) - 1. SRS and WDA left set so we do not drive the external reset pins.
  uint16_t wt = (uint16_t)((WATCHDOG_SECONDS * 2) - 1);
  WDOG1_WCR = WDOG_WCR_WDE | WDOG_WCR_WT(wt) | WDOG_WCR_SRS | WDOG_WCR_WDA | WDOG_WCR_WDBG;
#elif WATCHDOG_ENABLE
  { extern void sim_wdtStart(double seconds); sim_wdtStart((double)WATCHDOG_SECONDS); }
#endif
}
void raiseFault(uint32_t bit, uint8_t minLevel){
  faultFlags |= bit;
  if(minLevel > healthLevel){
    healthLevel = minLevel;
    Serial.print(F("HEALTH -> level ")); Serial.print((int)healthLevel);
    Serial.print(F("  faults=0x")); Serial.println((int)faultFlags);
  }
}
// --- detector tuning. Every one needs SUSTAINED evidence; a single bad sample changes nothing. ---
constexpr int  SLOW_LOOPS_TO_SHED   = 12;    // consecutive slow iterations before shedding work
constexpr unsigned long IMU_SLOW_US = 8000;  // a single IMU read this long = the bus is misbehaving
constexpr unsigned long SENSOR_READ_MAX_US = 6000;  // pad gate: mean IMU read cost above this refuses to arm.
                                        // A healthy MPU6050 read over 400 kHz I2C is a few hundred us; even
                                        // at 100 kHz it is ~2 ms. 6 ms means the bus is sick, and the whole
                                        // control design assumes a ~3.5 ms loop, so flying it is not an option.
constexpr int  IMU_STUCK_N          = 25;    // identical consecutive samples = wedged bus / dead device
// Attitude-staleness, RATE-vs-RATE so the loop period cancels. See the detector in sensors() for why
// the previous per-sample form broke: it compared a per-iteration movement against a fixed 0.05 deg,
// which is only meaningful at one loop speed, and the loop got 40x faster when the SPI IMU came up.
constexpr float ATT_STALE_WINDOW_S   = 0.15f; // integrate over this long, then judge (was 40 samples --
                                              // 140 ms at the old loop rate, 3.4 ms at the new one)
constexpr float ATT_STALE_MIN_ROT_DEG = 4.0f; // ...but only judge windows with real rotation in them.
                                              // 0.15 s at the 25 dps the old gate used = 3.75 deg.
constexpr float ATT_STALE_TRACK_FRAC = 0.25f; // stale if the estimate showed under this fraction of the
                                              // rotation the gyro reported. ASC038 showed ~0 (frozen);
                                              // a healthy estimate tracks at ~1.0, so 0.25 sits far
                                              // from both. Gyro noise cannot fake tracking -- noise
                                              // ADDS movement, so it can only push this number UP.
float attStaleExpDeg = 0, attStaleGotDeg = 0, attStaleWinS = 0;
constexpr int  OSC_FLIPS            = 10;    // command sign reversals within OSC_WINDOW_MS...
constexpr unsigned long OSC_WINDOW_MS = 1000;// ...at large amplitude = ringing
constexpr float OSC_AMP_FRAC        = 0.60f; // "large" = this fraction of MAX_TILT
int      slowLoopRun=0, imuStuckRun=0;   // (attStaleRun retired 2026-08-15 -- the staleness detector
                                         //  is time-windowed now, not a consecutive-sample counter)
// --- redundant inertial sources (Impulse 2.2 carries two, on independent buses) ---
// Declared here rather than beside readIMU() because dumpControlLog() reports them and is defined
// earlier in the file. See the block above readIMU() for the architecture and why the SPI primary
// driver is deliberately stubbed to fail.
#define SENSOR_BOARD_ENABLE 1     // 1 = attempt the SPI primary (auto-fails over until the driver exists)
#define ICM42688_DRIVER_PRESENT 1 // driver implemented + hardware-validated 2026-08-12; axis SIGNS still unverified
enum { SRC_PRIMARY=0, SRC_BACKUP=1, SRC_COUNT=2 };
const char* const SRC_NAME[SRC_COUNT] = {"PRIMARY(SPI ICM42688)", "BACKUP(I2C MPU6050)"};
uint8_t  imuActive = SRC_BACKUP;                  // resolved in setup()
bool     imuOk[SRC_COUNT] = {true, true};

// ================= GPS + PASSIVE HORIZONTAL NAVIGATION =======================================
// Declared HERE with the other health globals for exactly the reason imuActive is: dumpControlLog()
// reports them and is defined ~250 lines EARLIER in this file than the driver that maintains them.
//
// *** EVERY QUANTITY IN THIS SUBSYSTEM IS PASSIVE. *** Nothing derived from GPS reaches a servo, a
// pyro, an abort, the apogee decision or the plant estimator. It is computed, logged, and otherwise
// ignored. That is the whole design: the landing firmware's guidance needs a fused GPS/IMU/baro
// horizontal state, that fusion has never run anywhere but the simulator, and an ascent test is the
// cheapest place to learn how it behaves on real hardware -- boost vibration, ~3 g axial, fix
// dropout and reacquisition -- with a control loop that is structurally unable to be affected by
// the answer. Identical pattern and identical justification to the passive vertical Kalman (kfZ).
//
// HARDWARE: ATGM336H on Serial4 (RX4 = pin 16), PPS on pin 20, from the Impulse 2.2 netlist and
// confirmed working by Impulse22_SensorCheck (fix at 10 sats, HDOP 1.2). NOT Serial1: Serial1 is
// pins 0/1, which on this board are CS_IMU and MISO1 -- it would fight the sensor board for the SPI
// chip select. (Sysiphus_Landing.ino still has `#define GPS_SERIAL Serial1` and has this collision.)
#define GPS_ENABLE 1
#define GPS_SERIAL Serial4
constexpr uint32_t GPS_BAUD    = 9600;
constexpr int      PIN_GPS_PPS = 20;
// Hard cap on bytes consumed per pumpGPS() call. At 9600 baud a byte arrives every ~1.04 ms and the
// control loop runs every ~3.5 ms, so ~4 bytes accumulate per iteration -- 96 drains any realistic
// backlog many times over while making the work per call ABSOLUTELY bounded. An unbounded
// `while(available())` on a babbling or shorted RX line is an unbounded stall in the control loop,
// which is the ASC038 failure mode with a different cause. Bound it by construction, not by trust.
constexpr int GPS_MAX_BYTES_PER_CALL = 96;
// Datum quality gate, carried over verbatim from Impulse22_SensorCheck. Do NOT take the first fix:
// the first fix is the worst fix (typically 4 satellites at HDOP ~10, tens of metres out) and every
// relative reading afterwards inherits that error as a fixed offset. The bench session measured GPS
// vertical drifting 28 m from exactly this mistake.
constexpr int   DATUM_MIN_SATS = 6;
constexpr float DATUM_MAX_HDOP = 2.5f;

// ---- GPS state ----
double   gpsLat=0, gpsLon=0;             // deg, latest fix
double   gpsLat0=0, gpsLon0=0;           // deg, datum (captured once, gated on quality)
float    gpsMsl=0, gpsMsl0=0;            // m, GGA orthometric height / datum
float    gpsN=0, gpsE=0, gpsU=0;         // m from the datum, local tangent plane
int      gpsFixQual=0, gpsSats=0;        // GGA field 6 / field 7
float    gpsHdop=0;
bool     gpsHasFix=false, gpsOriginSet=false;
// gpsFixLost counts LOSSES (falling edges) and gpsReacquires counts recoveries (rising edges). They
// were briefly one counter, which double-counted every dropout: a single 2 s outage read as "2 fixes
// lost". A diagnostic that reports twice the truth is worse than no diagnostic.
uint32_t gpsSentences=0, gpsCsumErr=0, gpsFixUpdates=0, gpsFixLost=0, gpsReacquires=0;
uint32_t gpsLastFixMs=0;                 // millis() of the last GGA carrying a position
int      gpsSatsMin=99, gpsSatsMax=0;
float    gpsHdopMin=99.9f, gpsHdopMax=0.0f;
bool     gpsFixAtLiftoff=false;
volatile uint32_t gpsPpsCount=0;         // PPS edges -- proves the receiver is alive even if NMEA framing breaks

// ---- Passive horizontal state ----
// estN/estE/estVn/estVe : the FUSED GPS+IMU estimate, in the GPS north/east frame. This is the
//                         landing firmware's kfX/kfY, running here with nothing depending on it.
// insX/insY/insVx/insVy : pure IMU dead reckoning in the FIRMWARE's world frame, no GPS at all. Kept
//                         separately because it is the only way to measure inertial drift against an
//                         independent reference, and because the heading solver below needs an
//                         unfused inertial track to compare with.
float estN=0, estE=0, estVn=0, estVe=0;
float insX=0, insY=0, insVx=0, insVy=0;
// GPS position at the moment the INS track was zeroed. The two tracks must share an origin before
// they can be compared: insX/insY start at zero at liftoff, while gpsN/gpsE are measured from a
// datum captured minutes earlier on the pad, and the several metres of receiver scatter between
// those two moments is a fixed offset that would otherwise be read as a real displacement. On a
// near-vertical ascent the total horizontal travel is only ~5-10 m, so a 2 m offset is not a
// rounding error -- it is a third of the signal the heading solve has to work with.
float gpsN0=0, gpsE0=0;
float navMaxHorizM=0;                    // largest |horizontal| excursion the fused estimate reported
uint32_t navGpsUpdates=0;                // GPS position updates actually fused
// ================= PAD HEADING, AND THE HANDEDNESS THAT GOES WITH IT =========================
// The firmware's world frame is levelled to gravity by quatFromAccel() but its AZIMUTH is arbitrary:
// there is no magnetometer, so world +X is wherever the rocket's body +X happened to point on the
// pad. GPS measures north/east. Converting between them needs TWO facts, and only the first is
// obvious.
//
// FACT 1 -- the angle. psi = bearing of world +X, clockwise from true north.
//
// FACT 2 -- THE HANDEDNESS, and this is a trap. (N, E, U) is a LEFT-handed triad: N x E = -U. Check
// it on a globe -- at lat 0 lon 0 in ECEF, N=(0,0,1), E=(0,1,0), U=(1,0,0), and (0,0,1)x(0,1,0) =
// (-1,0,0) = -U. (This is why the standard aerospace frame is NED, not NEU.) The firmware's world
// frame, by contrast, is the image of the body frame under a proper rotation with +Z up, so it is
// RIGHT-handed. A right-handed frame does not map onto a left-handed one by a rotation, and treating
// it as though it does mirrors every horizontal result:
//
//     world +X in (N,E) = ( cos psi,  sin psi)
//     world +Y in (N,E) = ( sin psi, -cos psi)     <-- NOT (-sin psi, cos psi)
//
// So at psi = 0, world +X is NORTH and world +Y is WEST (check: North x West = Up, right-handed).
// The first version of this code used the rotation-only form, and the SIL did not catch it because
// the harness had been given the same wrong convention -- a self-consistency test proving only that
// two copies of one mistake agree. The regression suite now asserts the sign relation directly.
//
// >>> THE SOLVE IS NOT TRUSTED BY DEFAULT, AND THE MEASUREMENT SAYS IT SHOULD NOT BE. <<<
// Sweep of 25 seeds, realistic 1 Hz / 2 m receiver, near-vertical ascent (~6 m of horizontal travel):
//        ideal accelerometer   locks 17/25, median heading error 17.2 deg
//        10 mg lateral bias    locks 17/25, median 13.0 deg
//        30 mg lateral bias    locks 19/25, median 22.1 deg, worst 61 deg
//        30 mg + a 10 Hz receiver          median 27.2 deg  <-- MORE DATA IS WORSE
// More measurements making it worse is the signature of a SYSTEMATIC error, not a noisy one: the
// limit is inertial drift, not the receiver. A near-vertical flight simply does not travel far enough
// sideways for a displacement-matching fit to work, and the accelerometer bias that drives the drift
// is never calibrated (the pad routine calibrates GYRO bias only; quatFromAccel absorbs accel bias
// into ATTITUDE, where ~3 g of thrust re-emits it as a much larger false lateral acceleration).
//
// THEREFORE: NAV_HDG_FROM_PAD = true means the filter uses the ENTERED heading and the in-flight
// solve runs alongside as a passive, logged EXPERIMENT scored against it. That is a positive control
// (see CLAUDE.md methodological rule 1) rather than a dependency: the flight's horizontal record
// stays sound whatever the solve does, and the solve's own accuracy becomes a measured result.
// NOT constexpr, for the same reason TVC_TORQUE_CMD is not: the SIL A/Bs both modes in one binary
// (--selfalign), so the legacy self-aligning path stays exercised instead of rotting.
bool  NAV_HDG_FROM_PAD = true;                // true = trust GPS_PAD_HEADING_DEG; solve is diagnostic only
// >>> SET THIS FROM THE PAD ORIENTATION. Bearing of world +X, degrees clockwise from TRUE north. <<<
// World +X is, to first order, the horizontal direction the body +X axis points. To find which
// physical direction that is WITHOUT reasoning about sensor axes: power the board, tilt the nose, and
// read the tilt channels. Tilting the nose toward world +X drives tiltY POSITIVE with tiltX ~0
// (gyro_y = -atan2(ubx,ubz), so a nose-lean toward +X gives +tiltY). Point the nose that way, take
// its compass bearing, correct for magnetic declination, and enter it here. It is a property of the
// AIRFRAME, so it only has to be measured once -- then set the rocket on the pad in that orientation.
// NOT constexpr: the SIL sets it via extern (--padheading) to simulate the orientation having been
// entered. On hardware it is edited here and behaves as a constant.
float GPS_PAD_HEADING_DEG = 0.0f;
constexpr float NAV_HDG_MIN_DISP_M  = 5.0f;   // INS+GPS must both have moved this far before the fit locks
float    navHeadingRad = GPS_PAD_HEADING_DEG*0.017453292519943295f;   // what the FILTER uses
float    navHeadingSolvedRad = 0;             // what the in-flight fit found (diagnostic)
bool     navHeadingLocked = false;
double   navHdgA=0, navHdgB=0;           // Procrustes accumulators (see navUpdate)
float    navInsGpsResidM=0;              // |INS track - GPS track| after alignment: the drift number

// Body-frame Y rate. The `-imu_gy` that the control path, the quaternion and the control log all
// applied was an MPU-6050 axis convention, NOT a universal one -- applied to the ICM-42688-P it
// inverts D on the Y channel. It cannot be folded into ICM_SGN_Y either, because that constant also
// scales accel Y, and accel Y feeds P on the *X* channel (tiltX = atan2(ay,az)). The sensor axes
// cross over into the control channels, so one per-axis sign cannot fix one channel:
//     channel X  <-  P from accel Y,  D from gyro X
//     channel Y  <-  P from accel X,  D from gyro Y
// Bench 2026-08-13 pinned all four independently: X-P right, X-D inverted, Y both inverted.
inline float rateYBody(){ return (imuActive == SRC_PRIMARY) ? imu_gy : -imu_gy; }
int      imuStuckRunSrc[SRC_COUNT] = {0, 0};
uint32_t imuFailoverCount = 0;
#define F_IMU_FAILOVER  (1u<<14)  // switched inertial source; control UNAFFECTED
#define F_IMU_BOTH_DEAD (1u<<15)  // no trustworthy inertial source remains
#define F_INVERTED_CTRL (1u<<16)  // commanded gimbal and resulting angular accel have the SAME sign
// ---- CONTROL-SIGN (POSITIVE-FEEDBACK) DETECTOR ------------------------------------------------
// The one failure class I have repeatedly called "bench-only" -- and it is not. The physics is
// unambiguous: the control torque is Mx = -T*L*delta, so angular ACCELERATION must oppose the
// commanded gimbal angle. If cmd and d(rate)/dt share a sign, the loop is positive feedback and the
// vehicle WILL diverge. This catches a wrong SERVO_*_SIGN, a swapped X/Y wiring, a linkage rebuilt to
// the opposite sense, or an estimator whose output has inverted -- all of which have bitten this
// project -- without caring which one it is.
// Requires thrust (no torque without it), a meaningful command, and sustained agreement, so ordinary
// gust-driven disagreement cannot trip it.
constexpr float SIGN_MIN_CMD_DEG = 1.0f;   // only judge when the loop is actually commanding something
// Thresholds set from MEASURED separation, not intuition (SIL, 2026-08-08):
//   correct wiring, nominal / slow servo / unstable aero : bad_frac 0.43 - 0.50
//   correct wiring, HEAVY GUST                           : bad_frac 0.759   <- worst healthy case
//   inverted wiring (X, Y, or both)                      : bad_frac 1.000
// The first draft used 0.75, which the heavy-gust case EXCEEDS -- it would have neutralised the
// gimbal on a healthy vehicle in wind, causing the crash it exists to prevent. 0.90 sits clear of
// the worst healthy case and far below the inverted signature. Counting is CUMULATIVE, not windowed,
// so the fraction converges on the true rate instead of wandering into a threshold on a bad patch.
constexpr int   SIGN_MIN_SAMPLES = 50;     // inverted wiring reaches ~50 qualifying samples before
                                           // the tilt abort ends the flight, so this is the usable budget
constexpr float SIGN_BAD_FRAC    = 0.90f;
int signGood=0, signBad=0;
#define F_NAN_ATT       (1u<<10)  // attitude quaternion went non-finite
#define F_SAT_SENSOR    (1u<<11)  // gyro or accel pinned at its full-scale limit
#define F_NO_LIFTOFF    (1u<<12)  // sequence completed without the vehicle ever leaving the pad
#define F_WDT_RESET     (1u<<13)  // came up from a watchdog reset
// Liftoff confirmation. NOTHING may fire a pyro until this is true. The 2026-07-20 session has a
// recorded no-ignition run (MTR000: 343 samples, axial flat at the at-rest value, zero samples above
// the thrust threshold) where the firmware ran the ENTIRE sequence -- countdown, burn window, coast,
// recovery -- while sitting on the pad. With the current code that ends in the parachute and leg
// charges firing at ground level next to whoever is standing there. That is the worst hazard in this
// firmware and it is a people-safety issue, not a vehicle one.
bool liftoffConfirmed = false;
constexpr float LIFTOFF_ALT_M = 3.0f;        // baro says we are clearly off the pad
// Sensor full-scale limits (set in setup(): +-16 g, +-2000 dps). A reading pinned at the rail is not
// a measurement, it is a clip -- and an attitude integrated from clipped rates is wrong in a way that
// looks perfectly plausible. ASC007 flew with a railed +-2 g accelerometer for exactly this reason.
constexpr float GYRO_FS_DPS = 2000.0f, ACCEL_FS_MPS2 = 16.0f*9.81f;
constexpr float SAT_FRAC = 0.98f;            // within 2% of the rail counts as saturated
uint32_t baroIntervalMs = 25;                // 40 Hz nominal; stretched when the loop is starved
uint32_t logIntervalMs  = 50;                // 20 Hz nominal; stretched, then stopped
bool     loggingOn = true;
float    gainScale = 1.0f;                   // 1.0 nominal; halved on sustained oscillation
bool     attitudeTrusted = true;             // false -> rate damping only (HEALTH_FALLBACK)
bool     controlInhibited = false;           // true -> gimbal held at TRUE neutral, integrators zeroed.
                                             // Used when commanding is worse than not commanding, i.e.
                                             // confirmed positive feedback. Distinct from gainScale,
                                             // which only scales kP/kD and cannot silence the I term.
// Loop-health counters, reported in the CTL###.CSV header. dtLong = iterations slower than DT_WARN_S but
// still integrated; dtDropped = iterations so slow they were discarded. On ASC038 the equivalent of
// dtDropped would have been 29 of 170 -- and nothing in any log said so. Now it is impossible to miss.
constexpr float DT_WARN_S   = 0.010f;   // 10 ms. LOWERED from 20 ms on 2026-08-08: with a variable-rate
                                        // clock in the harness (finally) it showed a 20 ms loop -- 6x
                                        // slower than nominal -- shedding NOTHING, because 20 ms only just
                                        // reached the old threshold. Shed early and shed cheap: 10 ms is
                                        // still ~3x the healthy 3.5 ms period, so healthy flights never
                                        // trip it, but a degrading loop starts dropping ballast sooner.
constexpr float DT_SANITY_S = 0.500f;   // 500 ms: beyond this the sample is garbage (stall / millis rollover)
uint32_t dtLongCount = 0, dtDroppedCount = 0;

// -- In-flight motor characterization ("static fire in the sky") ---------------------------------------
// High-rate axial specific force (imu_az ~ (thrust - drag)/mass, m/s^2) + ignition lag, buffered in RAM
// during the burn and dumped to MTR###.CSV at recovery. Lets you (1) compare the FLOWN motor's thrust
// curve + ignition lag against the bench static fire, and (2) validate the landing firmware's in-flight
// sys-ID (which identifies exactly this T/m scale) on real motors. Reconstruct thrust offline:
//   T(t) = imu_az(t)*m(t) + drag,   m(t) = dry + prop*(1 - t/BURN_TIME).
constexpr int   MOTOR_BUF_N    = 1000;         // 100 Hz -> ~10 s of headroom; 1000*8 B = 8 KB RAM
constexpr float THRUST_ONSET_G = 1.5f;         // g; axial accel above this = motor lit / lifting off
uint32_t motorT[MOTOR_BUF_N];                  // ms since the ignition COMMAND (PYRO_ASCENT)
float    motorAz[MOTOR_BUF_N];                 // axial specific force (m/s^2) at that time
int      motorN = 0;                           // samples captured
uint32_t ignCmdMs = 0, thrustOnsetMs = 0;      // ignition-command time / first-thrust time (millis)
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
uint32_t ctlDt[CTL_BUF_N];                             // that iteration's loop period (us). WIDENED uint16->uint32 after
                                                       // ASC038: it saturated at 65535 and destroyed the evidence on 17%
                                                       // of iterations, exactly the ones that mattered. The header's
                                                       // min/mean/max were uint32 and survived; the per-sample column
                                                       // did not. Never store a diagnostic in a type that can clip.
float ctlTiltX[CTL_BUF_N],  ctlTiltY[CTL_BUF_N];       // quaternion tilt fed to the P term (deg)
float ctlRfX[CTL_BUF_N],    ctlRfY[CTL_BUF_N];         // FILTERED body rate used by the D term (deg/s)
float ctlRrX[CTL_BUF_N],    ctlRrY[CTL_BUF_N];         // RAW bias-removed body rate, pre-EMA (deg/s)
float ctlPreX[CTL_BUF_N],   ctlPreY[CTL_BUF_N];        // TVC command BEFORE constrain() (TVC deg)
float ctlPostX[CTL_BUF_N],  ctlPostY[CTL_BUF_N];       // TVC command AFTER constrain() (TVC deg)
float ctlKeff[CTL_BUF_N],   ctlThrust[CTL_BUF_N];      // scheduled plant: keff (1/s^2) and thrust (N)
#if CTL_SERVO_FEEDBACK
int16_t ctlFbX[CTL_BUF_N],  ctlFbY[CTL_BUF_N];         // raw ADC counts on A2/A3
#endif
#endif
int ctlN = 0;                                          // samples captured

// Loop-period statistics over the powered-flight control loop. Compiled in REGARDLESS of CTL_CAPTURE (it costs
// ~0.5 KB and 4 integer ops per iteration) so the CTL_CAPTURE=0 and =1 builds measure the loop the SAME way and
// the difference between their medians is attributable to the capture and to nothing else.
// Loop-period histogram. BIN WIDTH REDUCED 100 us -> 10 us on 2026-08-15, after CTL001 reported
// loop_us_median=50 alongside loop_us_min=84 -- a median BELOW the minimum, which is the giveaway.
// With 100 us bins an 85 us loop lands entirely in bin 0 and the "median" was just that bin's
// midpoint, i.e. a constant. The bins were sized for the 3450 us I2C loop; the SPI sensor board made
// the loop 40x faster and the diagnostic lost all resolution at exactly the moment it got interesting.
// 256 bins x 10 us covers 0..2.56 ms, which spans a healthy loop with room to spare; anything slower
// lands in the top bin and is caught by loop_us_max and the dt counters, which are exact.
constexpr int LOOP_HIST_N  = 256;
constexpr uint32_t LOOP_HIST_BIN_US = 10;
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
    uint32_t b = dt/LOOP_HIST_BIN_US; if(b >= (uint32_t)LOOP_HIST_N) b = LOOP_HIST_N-1; loopHist[b]++;

    // --- LOOP-HEALTH GOVERNOR ---------------------------------------------------------------
    // ASC038's lesson stated as a rule: the control loop is the only thing that MUST run on time.
    // Everything else is discretionary and gets shed, in order of least value first, the moment
    // the loop starts running late. The firmware should have noticed and reduced its own logging
    // rather than starving the estimator until the attitude froze.
    // Shed order:  baro polling (25 -> 100 ms)  ->  flight log (50 -> 200 ms)  ->  flight log off.
    // Control, the CTL capture (0.3 us) and the recovery path are never shed.
    if(dt > (uint32_t)(DT_WARN_S*1e6f)){
      if(++slowLoopRun >= SLOW_LOOPS_TO_SHED){
        slowLoopRun = 0;
        raiseFault(F_LOOP_SLOW, HEALTH_REDUCED);
        if(baroIntervalMs < 100){ baroIntervalMs = 100; raiseFault(F_BARO_SHED, HEALTH_REDUCED); }
        else if(logIntervalMs < 200){ logIntervalMs = 200; raiseFault(F_LOG_SHED, HEALTH_REDUCED); }
        else if(loggingOn){ loggingOn = false; raiseFault(F_LOG_SHED, HEALTH_REDUCED); }
      }
    } else slowLoopRun = 0;
  }
#if CTL_CAPTURE
  if(ctlN < CTL_BUF_N && (uint32_t)(now - ctlLastCapUs) >= CTL_PERIOD_US){
    ctlLastCapUs = now;
    int i = ctlN;
    ctlT[i]=now;  ctlDt[i] = dt;
    ctlTiltX[i]=gyro_x;     ctlTiltY[i]=gyro_y;
    ctlRfX[i]=ang_vel_x;    ctlRfY[i]=ang_vel_y;
    ctlRrX[i]=imu_gx;       ctlRrY[i]=rateYBody();              // SAME sign convention sensors() feeds into the EMA
    ctlPreX[i]=preX;        ctlPreY[i]=preY;
    ctlPostX[i]=tiltX/SERVO_X_MULT;  ctlPostY[i]=tiltY/SERVO_Y_MULT;   // servo deg -> TVC deg, as logData() does
    // The scheduled plant, per sample. Without this the gain the loop actually flew is unrecoverable
    // from the log -- pre/post only show the command, not the keff it was divided by.
    ctlKeff[i]=keffEst;     ctlThrust[i]=plantThrustN;
#if CTL_SERVO_FEEDBACK
    ctlFbX[i]=(int16_t)analogRead(A2);  ctlFbY[i]=(int16_t)analogRead(A3);
#endif
    ctlN++;
  }
#endif
  ctlCostCyc += (uint32_t)(CTL_CYC() - c0);
}

void updatePyros();   // fwd decl: the dump below ticks the pyro timer so a long SD write cannot hold a melt wire on
#if BOARD_IMPULSE_22
float readRail(int pin, float divider);   // fwd decl, same reason as pyroContinuous below: the CTL
                                // header now logs raw pyro sense volts and readRail is defined with
                                // the other 2.2 helpers, ~700 lines further down. The Arduino IDE
                                // hoists auto-generated prototypes so it builds there regardless --
                                // the SIL, which compiles the .ino as plain C++, is what catches it.
bool pyroContinuous(int idx);   // fwd decl: defined with the other 2.2 helpers below; the CTL header
                                // reports post-fire continuity, so the dump needs it before its definition
#endif
void dumpControlLog(){
  static bool done=false; if(done) return; done=true;       // recovery OR emergency, whichever is reached first
  char fn[20]; int idx=0; do{ sprintf(fn,"CTL%03d.CSV",idx++); } while(SD.exists(fn)&&idx<1000);
  File f=SD.open(fn,FILE_WRITE); if(!f){ Serial.println(F("CTL log failed")); return; }
  // median from the 100 us histogram (bin midpoint, +-50 us); min/max/mean below are exact to 1 us
  uint32_t half=loopIters/2, acc=0; int mb=0;
  for(int i=0;i<LOOP_HIST_N;i++){ acc+=loopHist[i]; if(acc>half){ mb=i; break; } }
  uint32_t med  = loopIters ? (uint32_t)mb*LOOP_HIST_BIN_US + LOOP_HIST_BIN_US/2u : 0;
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
  // THE SYSTEM-ID RESULT. thrust_scale is what this motor actually did against the F15 reference
  // curve: 1.00 = on the curve, 1.08 = ran 8% hot. keff_final/keff_nominal say how far the plant
  // moved from the design point, i.e. how much of a correction the scheduling was actually making.
  snprintf(b,sizeof(b),"# torque_cmd=%d wn=%.2f zeta=%.2f keff_nom=%.1f keff_final=%.1f",
           (int)TVC_TORQUE_CMD,TVC_WN,TVC_ZETA,keffNominal,keffEst);                             f.println(b);
  snprintf(b,sizeof(b),"# thrust_scale=%.3f burn_frac=%.3f impulse_Ns=%.1f motor_Ns=%.1f",
           thrustScale,burnFrac,impulseNs,MOTOR_IMPULSE_NS);                                      f.println(b);
  snprintf(b,sizeof(b),"# plant_final: m=%.3f L=%.4f Iyy=%.5f  dry: m=%.3f L=%.4f Iyy=%.5f",
           plantMass,plantL,plantIyy,DRY_MASS_KG,DRY_L_M,DRY_IYY);                                f.println(b);
  // sd_open_failures>0 with loop_iters covering the full burn = the board lived and only SD writes died.
  // reset_cause!=0 on a flight log = the board rebooted; decode against SRC_SRSR_* in imxrt.h.
  snprintf(b,sizeof(b),"# sd_open_failures=%u reset_cause=0x%08lX resume_boots=%u",
           (unsigned)sdFailCount,(unsigned long)resetCause,(unsigned)persist.boots);              f.println(b);
  // LOOP HEALTH. dt_dropped > 0 means the attitude estimator lost samples -- treat any nonzero value as
  // a failed flight for control purposes, regardless of how the vehicle looked. This is the line that
  // would have made ASC038's root cause obvious on the first read.
  // THE HEALTH RECORD. How far down the degradation ladder the flight went, and which detectors
  // tripped. health_level=0 with faults=0 is a clean flight; anything else tells you what broke and
  // how much safety margin it cost -- which is the whole point of the ladder.
  snprintf(b,sizeof(b),"# health_level=%u faults=0x%03lX gain_scale=%.2f attitude_trusted=%d",
           (unsigned)healthLevel,(unsigned long)faultFlags,gainScale,(int)attitudeTrusted);   f.println(b);
  snprintf(b,sizeof(b),"# faultbits: 0=loopslow 1=baroshed 2=logshed 3=imustuck 4=imuslow "
                       "5=attstale 6=oscillation 7=gainbackoff 8=railsag 9=rateabort");       f.println(b);
  snprintf(b,sizeof(b),"# shed_state: baro_interval_ms=%lu log_interval_ms=%lu logging_on=%d",
           (unsigned long)baroIntervalMs,(unsigned long)logIntervalMs,(int)loggingOn);        f.println(b);
  // Which inertial source flew, and whether it ever switched. imu_failovers>0 with health_level=1
  // is the good outcome: a sensor died and the flight carried on under full attitude control.
  // Control-sign evidence. sign_bad/sign_tot near 1.0 means the loop was fighting itself: commanded
  // gimbal and resulting angular acceleration shared a sign, i.e. positive feedback. Logged even when
  // the detector did not reach its threshold, because the RATIO is diagnostic on its own -- a tumble
  // with sign_bad_frac ~0.9 is a wiring fault, the same tumble at ~0.5 is a disturbance it lost to.
  snprintf(b,sizeof(b),"# sign_good=%d sign_bad=%d sign_bad_frac=%.3f (threshold %.2f over %d samples)",
           signGood,signBad,(signGood+signBad)?(float)signBad/(float)(signGood+signBad):0.0f,
           SIGN_BAD_FRAC,SIGN_MIN_SAMPLES);                                                       f.println(b);
  snprintf(b,sizeof(b),"# imu_active=%d imu_failovers=%lu imu_primary_ok=%d imu_backup_ok=%d",
           (int)imuActive,(unsigned long)imuFailoverCount,(int)imuOk[SRC_PRIMARY],(int)imuOk[SRC_BACKUP]);
                                                                                              f.println(b);
  snprintf(b,sizeof(b),"# dt_long=%lu dt_dropped=%lu  (warn>%.0fms drop>%.0fms)",
           (unsigned long)dtLongCount,(unsigned long)dtDroppedCount,
           DT_WARN_S*1000.0f, DT_SANITY_S*1000.0f);                                               f.println(b);
#if GPS_ENABLE
  // ---- GPS + PASSIVE HORIZONTAL NAV. Instrumentation only; none of it touched control. ----
  // THE QUESTION THIS FLIGHT EXISTS TO ANSWER, in four numbers:
  //   gps_fix_lost   how many times the receiver dropped a fix. Boost is the hostile part -- ~3 g
  //                  axial and motor vibration -- and a receiver that cannot hold lock through it
  //                  cannot time a hoverslam. 0 is the result that matters.
  //   gps_updates_used  fixes actually FUSED (quality-gated). Fewer than sentences_with_fix means
  //                  HDOP degraded past GPS_HDOP_MAX_USE, which is itself the interesting finding.
  //   heading_locked whether the pad-azimuth alignment solved from flight data alone. The landing
  //                  guidance needs this and has never had to find it without being told.
  //   ins_gps_resid  metres between the pure inertial track and GPS after alignment = the dead
  //                  reckoning drift budget, measured instead of assumed.
  snprintf(b,sizeof(b),"# gps_enabled=1 sentences=%lu csum_err=%lu fix_updates=%lu fix_lost=%lu reacquires=%lu pps=%lu",
           (unsigned long)gpsSentences,(unsigned long)gpsCsumErr,(unsigned long)gpsFixUpdates,
           (unsigned long)gpsFixLost,(unsigned long)gpsReacquires,(unsigned long)gpsPpsCount);      f.println(b);
  snprintf(b,sizeof(b),"# gps_fix_at_liftoff=%d fix_qual_final=%d sats_min=%d sats_max=%d hdop_min=%.1f hdop_max=%.1f",
           (int)gpsFixAtLiftoff,gpsFixQual,(gpsSatsMin==99?0:gpsSatsMin),gpsSatsMax,
           (gpsHdopMin>99.0f?0.0f:gpsHdopMin),gpsHdopMax);                                         f.println(b);
  snprintf(b,sizeof(b),"# gps_datum_set=%d lat=%.7f lon=%.7f msl=%.1f",
           (int)gpsOriginSet,gpsLat0,gpsLon0,gpsMsl0);                                             f.println(b);
  // heading_used is what the filter flew. heading_solved is the in-flight fit, which with
  // NAV_HDG_FROM_PAD is a scored EXPERIMENT and not an input -- their difference is its error.
  snprintf(b,sizeof(b),"# nav_gps_updates=%lu heading_used_deg=%.1f heading_solved_deg=%.1f heading_err_deg=%.1f heading_locked=%d from_pad=%d",
           (unsigned long)navGpsUpdates,navHeadingRad*RAD2DEG,navHeadingSolvedRad*RAD2DEG,
           navHeadingLocked ? (navHeadingSolvedRad-navHeadingRad)*RAD2DEG : 0.0f,
           (int)navHeadingLocked,(int)NAV_HDG_FROM_PAD);                                           f.println(b);
  snprintf(b,sizeof(b),"# max_horiz_m=%.1f ins_gps_resid_m=%.1f",
           navMaxHorizM,navInsGpsResidM);                                                          f.println(b);
  snprintf(b,sizeof(b),"# nav_final: estN=%.1f estE=%.1f estVn=%.1f estVe=%.1f insX=%.1f insY=%.1f gpsN=%.1f gpsE=%.1f",
           estN,estE,estVn,estVe,insX,insY,gpsN,gpsE);                                             f.println(b);
#else
  f.println("# gps_enabled=0");
#endif
  // The ICM's UI filter bandwidth is a DIRECT contributor to loop delay tau, and until this readback
  // existed it was whatever the part powers up with -- set by accident. Recorded per flight so a tau
  // measurement can be attributed to a known filter configuration. See ICM_UI_FILT_CFG.
  snprintf(b,sizeof(b),"# icm_uifilt_cfg=0x%02X icm_uifilt_written=0x%02X (GYRO_ACCEL_CONFIG0 @0x52; 0xFF written = left at power-on default)",
           (unsigned)icmUiFiltReadback,(unsigned)ICM_UI_FILT_CFG);                                 f.println(b);
#if BOARD_IMPULSE_22
  // RAIL MINIMA over the whole flight, sampled at loop rate. This is the measurement ASC036 and ASC038
  // both needed and neither had: a brownout is a millisecond-scale sag and is invisible at 20 Hz.
  // If vbat_min drops near 6 V while servo_min collapses, that is the servo-stall brownout, confirmed.
  snprintf(b,sizeof(b),"# vbat_min_v=%.2f servo_min_v=%.2f pyro_min_v=%.2f  (arm gates %.2f/%.2f/%.2f)",
           vbatMinV,servoMinV,pyroMinV,VBAT_ARM_MIN_V,SERVO_V_MIN_V,PYRO_V_MIN_V);                f.println(b);
  // Post-fire continuity: a melt wire that burned through reads OPEN. "fired=1 cont=1" means the pyro
  // was commanded but the wire is still intact -- i.e. it never got hot enough. That distinguishes
  // "the firmware did not fire" from "the firmware fired and the hardware did not respond".
  snprintf(b,sizeof(b),"# postfire_continuity P1ascent=%d P2descent=%d P3legs=%d P4chute=%d  (1 = still continuous = wire did NOT burn)",
           (int)pyroContinuous(0), (int)pyroContinuous(1),
           (int)pyroContinuous(2), (int)pyroContinuous(3));                                       f.println(b);
  // Raw sense volts alongside, because the boolean above hides a marginal joint: a channel at 0.95 V
  // and one at 1.85 V both read "1" against the 0.90 V threshold, and only one of them is healthy.
  snprintf(b,sizeof(b),"# pyro_sense_v P1ascent=%.2f P2descent=%.2f P3legs=%.2f P4chute=%.2f  (threshold %.2f)",
           readRail(PIN_PYRO_SENSE[0],1.0f), readRail(PIN_PYRO_SENSE[1],1.0f),
           readRail(PIN_PYRO_SENSE[2],1.0f), readRail(PIN_PYRO_SENSE[3],1.0f), PYRO_CONT_V);
                                                                                                  f.println(b);
  snprintf(b,sizeof(b),"# ground_test=%d gt_pyro_live=%d pyro_attempt_mask=0x%X (bits P1ascent,P2descent,P3legs,P4chute = requested but refused)",
           (int)groundTestInhibit,(int)gtPyroLive,(unsigned)pyroAttemptMask);                      f.println(b);
#endif
#if CTL_SERVO_FEEDBACK
  f.println("t_us,dt_us,tiltX_deg,tiltY_deg,rateFx_dps,rateFy_dps,rateRx_dps,rateRy_dps,preX_tvcdeg,preY_tvcdeg,postX_tvcdeg,postY_tvcdeg,keff,thrust_N,fbX_adc,fbY_adc");
#else
  f.println("t_us,dt_us,tiltX_deg,tiltY_deg,rateFx_dps,rateFy_dps,rateRx_dps,rateRy_dps,preX_tvcdeg,preY_tvcdeg,postX_tvcdeg,postY_tvcdeg,keff,thrust_N");
#endif
#if CTL_CAPTURE
  for(int i=0;i<ctlN;i++){
    if((i & 63)==0){ updatePyros(); wdtFeed(); }  // a 2000-row SD write can outlast the 1000 ms pyro pulse; tick the timer so a
                                     // melt wire is never held on longer than intended just because we are logging
    snprintf(b,sizeof(b),"%lu,%lu,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.1f,%.2f",
      (unsigned long)ctlT[i],(unsigned long)ctlDt[i],ctlTiltX[i],ctlTiltY[i],ctlRfX[i],ctlRfY[i],
      ctlRrX[i],ctlRrY[i],ctlPreX[i],ctlPreY[i],ctlPostX[i],ctlPostY[i],ctlKeff[i],ctlThrust[i]);
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
void triggerPyro(int pin){
  if(groundTestInhibit){
  // *** EVERY CHANNEL, AT THE DRIVER. Corrected 2026-08-15 after CTL003. ***
  // The first version of GROUND_TEST inhibited only P3 and argued that the liftoff interlock would
  // hold P1/P4, because with no ignition liftoffConfirmed could never go true. That was WRONG, and
  // the bench proved it: liftoffConfirmed latches on a TRANSIENT above THRUST_ONSET_G (1.5 g),
  // sampled at ~11.7 kHz, and ordinary handling produces that. On CTL003 it latched (faults=0x000,
  // i.e. F_NO_LIFTOFF never raised), the firmware ran the normal recovery path, and the parachute
  // and leg melt wires were energised on a bench -- postfire continuity went P1=1/P4=1 on the
  // previous run to P1=0/P4=0 on that one.
  // An interlock against "the motor never lit" is not an interlock against "someone picked the
  // rocket up". The inhibit belongs at the single choke point every pyro call passes through, where
  // no call site -- present or future -- can fail to inherit it.
    // GT_PYRO_LIVE releases the RECOVERY channels only. BOTH MOTOR channels are excluded
    // unconditionally: a ground test never lights a motor, and that is a property of this line
    // rather than a promise made elsewhere. Note the exclusion is on the PIN, so it holds no matter
    // which call site asked.
    // *** 2026-08-15: written by ROLE, not number. This used to read `pin != P3` when P3 was the
    // motor. P3 is now the LEGS, so that line would have inhibited the legs and RELEASED THE ASCENT
    // MOTOR on a bench. PYRO_DESCENT is added because it is newly live and is also a motor. ***
    const bool allowed = gtPyroLive && (pin != PYRO_ASCENT) && (pin != PYRO_DESCENT);
    if(!allowed){
      if(pin==P1) pyroAttemptMask |= 0x1; else if(pin==P2) pyroAttemptMask |= 0x2;
      else if(pin==P3) pyroAttemptMask |= 0x4; else if(pin==P4) pyroAttemptMask |= 0x8;
      Serial.print(F("GROUND TEST: pyro INHIBITED, pin ")); Serial.println(pin);
      return;
    }
    Serial.print(F("GROUND TEST: pyro LIVE (recovery channel) -- FIRING pin ")); Serial.println(pin);
  }
  for(auto&c:pyros) if(c.pin==pin&&!c.active){ digitalWrite(pin,HIGH); c.active=true; c.startTime=millis(); }
}
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
void saveBenchCal(); bool loadBenchCal();   // fwd: calibrateGyroBias() writes the cache, bench mode reads it
#if BOARD_IMPULSE_22
void sampleRails();     // fwd: the pack lights are driven from here, and the halt below wants them live
#endif
void sensorFaultHalt(int n){
  while(true){
    for(int i=0;i<n;i++){ LED(true,false,false); beep(880,120); LED(false,false,false); delay(180); }
    unsigned long t0=millis();
    while(millis()-t0<1200){
      wdtFeed();
#if BOARD_IMPULSE_22
      // Keep the pack lights updating while we sit here. A flat or missing battery is one of the
      // commonest reasons to be in this loop, and previously the halt went dark on exactly the two
      // indicators that would tell you which one -- so the board just flashed red and said nothing.
      sampleRails();
#endif
      if(exitOnButton()) resetFlightComputer();
    }
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
  while(true){ wdtFeed(); updatePyros(); }   // unreachable on Teensy; the SIL/host fallback keeps the old terminal behaviour
}

#if BOARD_IMPULSE_22
// -- Impulse 2.2 rail monitoring + pyro continuity -----------------------------------------------
// These are the two capabilities the previous board did not have, and they map directly onto the two
// unexplained failures on record: ASC036/ASC038's suspected brownout (no voltage was ever recorded)
// and ASC038's recovery, where nothing could confirm whether the melt wire actually burned through.
float readRail(int pin, float divider){
  int a = analogRead(pin);                       // 12-bit, set in setup()
  return (a * ADC_VREF / (float)ADC_MAX) * divider;
}
void updateBatteryLeds();                        // defined just below; fwd-declared for the SIL, which
                                                 // compiles the .ino as plain C++ with no hoisted prototypes
void sampleRails(){                              // cheap: 3 ADC reads, no blocking. Called from sensors().
  vbatV  = readRail(PIN_VBAT_SENSE,     VBAT_DIV);
  servoV = readRail(PIN_SERVO_V_SENSE,  SERVO_V_DIV);
  pyroV  = readRail(PIN_PYRO_V_SENSE,   PYRO_V_DIV);
  if(vbatV  < vbatMinV)  vbatMinV  = vbatV;      // the MINIMA are the point: a sag lasting a few ms is
  if(servoV < servoMinV) servoMinV = servoV;     // invisible in a 20 Hz log but is exactly what browns
  if(pyroV  < pyroMinV)  pyroMinV  = pyroV;      // out an MCU. Sampled at full loop rate, reported at recovery.
  // In-flight rail floors. Below these the hardware is misbehaving; record it as a degraded flight so
  // it shows up in the health record instead of only as an unexplained anomaly after the fact.
  if(vbatV < 6.60f || servoV < 4.20f) raiseFault(F_RAIL_SAG, HEALTH_REDUCED);
  updateBatteryLeds();
}

// Pack status at a glance, no laptop required -- the state you actually want on the pad.
//   solid       healthy, will arm
//   slow blink  low: above the in-flight sag floor but below the arm minimum, so it will NOT arm
//   fast blink  critical: below the sag floor, or a pyro pack too weak to heat a match
//   dark        rail absent
// Dark on the PYRO light is the normal disarmed state, not a fault -- that pack is meant to be the
// last thing connected. Dark on MAIN would mean the board is running on USB alone.
void updateBatteryLeds(){
  const uint32_t t = millis();

  // SELF TEST: both hard ON for the first 2.5 s of uptime, bypassing every threshold. If they do
  // not light during this window they are never going to, and the fault is the LED, the pin or the
  // solder joint -- not the logic. Lit here but dark afterwards means the rails or the thresholds.
  //
  // Deliberately NON-BLOCKING. The first version of this was a delay()-based animation in setup(),
  // which cost 1.9 s -- and the SIL's physics clock keeps running through delay(), so the simulated
  // vehicle aged 1.9 s before the firmware was ready. That shifted launch detection and broke five
  // brownout-resume timing locks (regression 6). A boot animation is not free on this vehicle.
  // Timed from the FIRST CALL, not from boot. Timing it from millis() meant setup() had already
  // spent longer than the window before anything sampled the rails -- up to 3 s waiting for the USB
  // monitor, plus 1.2 s of gyro calibration -- so the self test had always expired unseen.
  static uint32_t firstCall = 0;
  if(firstCall == 0) firstCall = t ? t : 1;
  if(t - firstCall < 2500){
    digitalWrite(PIN_MAIN_LED, HIGH);
    digitalWrite(PIN_PYRO_LED, HIGH);
    return;
  }

  const bool slow = (t % 1000) < 500;
  const bool fast = (t %  200) < 100;

  bool m;
  if     (vbatV < 1.00f)          m = false;
  else if(vbatV < 6.60f)          m = fast;      // same floor that raises F_RAIL_SAG above
  else if(vbatV < VBAT_ARM_MIN_V) m = slow;
  else                            m = true;

  bool p;
  if     (pyroV < 1.00f)          p = false;     // disarmed -- correct, not a fault
  else if(pyroV < PYRO_V_MIN_V)   p = fast;
  else                            p = true;

  digitalWrite(PIN_MAIN_LED, m);
  digitalWrite(PIN_PYRO_LED, p);

  // NO periodic Serial here. This function runs from the countdown and the flight loop, and the SIL
  // regression parses stdout -- a diagnostic line every 2 s injects noise into the stream the test
  // harness reads. The rails are already reported by the pre-arm gate and logged every cycle.
}
bool pyroContinuous(int idx){                    // idx 0..3 -> P1..P4
  return readRail(PIN_PYRO_SENSE[idx], 1.0f) > PYRO_CONT_V;
}

// All four channels, with the RAW SENSE VOLTAGE beside each verdict.
// The voltage is the point. As a bare boolean a channel sitting at 1.55 V reads identically to one
// at 1.95 V -- both "CONT" -- but the first is a hair above PYRO_CONT_V and could drop open under
// vibration, while the second is a solid connection sitting on the LED clamp. Only two channels
// were ever reported, and only as booleans; the other two were never shown at all.
// (Example levels updated 2026-08-15 with the threshold; they only illustrate if they straddle it.)
//
// NOTHING IS ENERGISED TO MEASURE THIS. The sense circuit works with the FET OFF: ~0.6 mA flows
// through the initiator via the 10k pull-up and is clamped by the indicator LED, far below any
// e-match no-fire current. So this is safe to call continuously, and it cannot fire anything.
// To deliberately FIRE a channel on the bench, use Firmware/Impulse22_PyroTest -- it is built for
// exactly that, with a dummy-load procedure and a hold-to-fire interlock.
void printPyroStatus(){
  static const char* const PYRO_NAME[4] = {"P1ascnt", "P2dscnt", "P3legs", "P4chut"};
  Serial.print(F("pyro cont:"));
  for(int i=0;i<4;i++){
    Serial.print(F("  ")); Serial.print(PYRO_NAME[i]); Serial.print(F("="));
    Serial.print(pyroContinuous(i) ? F("CONT") : F("open"));
    Serial.print(F(" ")); Serial.print(readRail(PIN_PYRO_SENSE[i], 1.0f), 2); Serial.print(F("V"));
  }
  Serial.print(F("   [thresh ")); Serial.print(PYRO_CONT_V, 2);
  Serial.print(F("V]  rails VBAT ")); Serial.print(vbatV, 2);
  Serial.print(F(" SERVO ")); Serial.print(servoV, 2);
  Serial.print(F(" PYRO ")); Serial.print(pyroV, 2); Serial.println(F(" V"));
}
// ACTIVE LOW -- this was inverted until 2026-08-12 and it halted the board on every boot with the
// harness correctly plugged in. The detect is not a logic output: the main board runs 5V-CLEAN
// through R64 (470R) and LED4 (blue) into this pin, and the sensor board ties J1 pad 12 to GND. So
// CONNECTED pulls the pin down (and lights LED4); DISCONNECTED leaves it floating up to ~2 V through
// the LED string, which reads HIGH. Getting this backwards meant haveBoard=false with the cable in,
// so the ICM was never tried, and with no MPU-6050 fitted that is "no inertial source" -> halt.
bool sensorCablePresent(){ return digitalRead(PIN_SENS_DET) == LOW; }
#endif

// -- Utils ---------------------------------------------------------------------
// LED1 is a COMMON-ANODE RGB tied to 5V-CLEAN, so a channel lights when its pin is pulled LOW.
// "Off" cannot be OUTPUT HIGH: that parks the pin at 3.3 V, leaving 5.00 - 3.3 = 1.70 V across the
// die -- right at the red emitter's conduction knee, so red glows faintly instead of going dark.
// Green and blue have Vf ~3 V and are unaffected, which is why only red does it.
// Releasing the pin to high-Z instead lets the node float up until the LED stops conducting
// (bench-measured 2026-08-12: pin 6 floats to 3.6 V, 7 to 3.1 V, 8 to 2.7 V), dropping red to
// ~1.4 V -- below the knee, genuinely off.
// The pin then sits at VDD+0.3 with the ESD clamp passing sub-microamps. That is the absolute
// maximum rather than inside it; it is what this hardware forces, and Impulse 2.3 should run the
// anode off 3.3 V instead. The state cache keeps this from churning pinMode every control cycle.
void LED(bool r,bool g,bool b){
  static int8_t lr=-1, lg=-1, lb=-1;
  if((int8_t)r != lr){ lr=r; if(r){ pinMode(RLED,OUTPUT); digitalWrite(RLED,LOW); } else pinMode(RLED,INPUT); }
  if((int8_t)g != lg){ lg=g; if(g){ pinMode(GLED,OUTPUT); digitalWrite(GLED,LOW); } else pinMode(GLED,INPUT); }
  if((int8_t)b != lb){ lb=b; if(b){ pinMode(BLED,OUTPUT); digitalWrite(BLED,LOW); } else pinMode(BLED,INPUT); }
}
void beep(int f,int d){ tone(BUZZER,f); delay(d); noTone(BUZZER); }
// Recorded so bench mode can print what was actually COMMANDED, not what it assumes was commanded.
// Verifying a sign by watching a servo is easy to fool yourself with; reading the number is not.
float lastCmdX = 0, lastCmdY = 0;        // TVC degrees, before sign and trim
int   lastServoX = 90, lastServoY = 90;  // the angle actually written
void writeServos(float sx,float sy){                    // clamp to a valid servo range for safety
  if(imuActive == SRC_PRIMARY){ sx *= ICM_CHAN_X_SIGN; sy *= ICM_CHAN_Y_SIGN; }   // see ICM_CHAN_*
  lastCmdX = sx; lastCmdY = sy;
  lastServoX = (int)constrain(SERVO_X_SIGN*sx + 90 + XTUNE, 0, 180);
  lastServoY = (int)constrain(SERVO_Y_SIGN*sy + 90 + YTUNE, 0, 180);
  servoX.write(lastServoX);
  servoY.write(lastServoY);
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
// This never touches PYRO_ASCENT (motor ignition) and never re-enters the control loop.
constexpr uint32_t PERSIST_MAGIC   = 0x54564332;   // 'TVC2'
constexpr int      PERSIST_ADDR    = 0;
constexpr uint16_t RESUME_MAX_BOOTS = 3;           // give up after this many reboots in one flight
constexpr unsigned long RESUME_MAX_WAIT_MS = 8000; // hard deploy timeout once a resume is armed
// Barometric vertical motion over looksAirborne()'s 400 ms window. 2 m in 400 ms is 5 m/s sustained.
// Bench noise on the fitted DPS310 is 0.16 m peak-to-peak over minutes (Impulse22_SensorCheck,
// 2026-08-13), so this is >12x the sensor floor, and no one carries a rocket up at 5 m/s for 400 ms.
// Real flight is nowhere near the threshold: 20 m/s at the blind spot below puts 8 m in the window.
constexpr float RESUME_ALT_MOVE_M = 2.0f;
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
  // PER-SAMPLE, not window min/max. The original took maxA/minA across the whole 400 ms window and
  // asked "was the ENTIRE window free-fall (maxA < 0.55 g)" or "the ENTIRE window thrust (minA > 2 g)".
  // Any window STRADDLING burnout contains both, so maxA is too high for the free-fall test and minA
  // too low for the thrust test and BOTH fail. Measured in the SIL (--resetat 3.6, burnout at 3.65 s):
  // the vehicle was at 37 m climbing at 19.7 m/s and this returned false, so resumeAfterReset() decided
  // it was on the ground and the chute never came out -- the only reset time in the whole flight that
  // lost the vehicle. That blind spot sat exactly at burnout, which is peak servo current and the most
  // likely moment to brown out in the first place.
  // Judging each sample on its own removes the hole entirely: thrust (~2.1 g) and free-fall (~0.1 g)
  // BOTH deviate from 1 g, so a mixed window becomes a strong positive instead of a false negative.
  //
  // THAT IS TRUE AT THE ENDPOINTS AND FALSE IN BETWEEN, which left a second blind spot (found
  // 2026-08-13, SIL sweep of --resetat in 50 ms steps). Specific force does not JUMP from thrust to
  // free-fall, it sweeps: across the F15 tail-off it runs 1.53 g -> 0.48 g and passes through
  // 1.003 g at t=3.05 s. For ~250 ms it sits inside this test's own +/-0.45 g dead band while the
  // vehicle is at 35-40 m climbing at 20 m/s. Every reset in t=[2.85, 3.25] was declared "on the
  // ground" and the chute never came out -- a 410 ms hole, and the failure is silent.
  //
  // No accelerometer threshold can close this, because at that instant a flying rocket and a resting
  // one genuinely produce the same specific force. The evidence has to come from somewhere else, so
  // the barometer is sampled across the same window.
  //
  // CHANGE in altitude, never absolute AGL. Absolute would mean trusting persist.groundAlt, and a
  // stale record from a previous flight at a lower field would then read as hundreds of metres of
  // "altitude" the moment the board powers on -- firing a pyro in your hands. A difference taken
  // inside one 400 ms window cannot be fooled by a bad datum, only by real vertical motion.
  int n=0, offOneG=0; float maxW=0;
  float altMin=1e9f, altMax=-1e9f;
  unsigned long t0=millis(), tBaro=0;
  while(millis()-t0<400){ wdtFeed();
    readIMU();
    float am=sqrtf(imu_ax*imu_ax+imu_ay*imu_ay+imu_az*imu_az);
    float wm=sqrtf(imu_gx*imu_gx+imu_gy*imu_gy+imu_gz*imu_gz);
    if(fabsf(am - G) > 0.45f*G) offOneG++;               // thrust OR free-fall, either way not resting
    n++;
    if(wm>maxW)maxW=wm;
    // Every 50 ms, not every pass: the DPS310 is configured for 32 Hz, so a faster poll just re-reads
    // the same conversion and spends loop time to learn nothing.
    if(millis()-tBaro >= 50){
      tBaro = millis();
      float a = baroReadAltitude(SEA_LEVEL_HPA);          // returns 0.0 with no baro fitted, so the
      if(a<altMin) altMin=a;                              // term below is simply dead in that case
      if(a>altMax) altMax=a;
    }
    delay(5);
  }
  // SUSTAINED, at 60% of the window -- deliberately stricter than a bare majority. A vehicle resting in
  // someone's hand reads 1 g and scores ~0%, so this cannot be tripped by holding the rocket; it needs
  // most of a continuous 400 ms to be off 1 g. This is the interlock that stops a stale EEPROM record
  // firing a pyro on the bench, so it is a check that must stay hard to satisfy, not an easy one.
  bool sustained = (n>0) && (offOneG*5 > n*3);
  bool tumbling  = (maxW > 90.0f);                       // deg/s, well above hand movement
  // Purely additive: on the ground this is 0.0 m and the interlock behaves exactly as before, so the
  // stale-EEPROM-must-not-deploy locks in section 6 are unaffected.
  bool climbing  = (altMax > altMin) && ((altMax-altMin) > RESUME_ALT_MOVE_M);
  return sustained || tumbling || climbing;
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
    altitude = baroReadAltitude(SEA_LEVEL_HPA) - initial_alt;
    if(altitude>highest_alt) highest_alt=altitude;
    bool thrusting = imu_az > THRUST_ONSET_G*G;             // NEVER deploy under thrust -- that destroys the chute
    if(!thrusting && (altitude < highest_alt-APOGEE_DROP_M || millis()-t0>RESUME_MAX_WAIT_MS)) break;
    wdtFeed(); updatePyros(); delay(10);
  }
  triggerPyro(PYRO_CHUTE);            // parachute
  triggerPyro(PYRO_LEGS);             // legs
  setPhase(3);
  dumpControlLog();                   // whatever survived in RAM is gone after a reboot, but the loop stats are not
  Serial.println(F("RESUME: chute + legs deployed"));
  LED(false,true,true);
  while(!exitOnButton()){ wdtFeed(); beep(523,600); beep(392,600); updatePyros(); }
  setPhase(0); persist.boots=0; savePersist();
  LED(false,false,false);
}

// -- SD logging ----------------------------------------------------------------
void createUniqueLogFile(){
  int idx=0; do{ sprintf(logFilename,"ASC%03d.CSV",idx++); } while(SD.exists(logFilename)&&idx<1000);
  logFile=SD.open(logFilename,FILE_WRITE);
  if(logFile){ Serial.print(F("Logging to: ")); Serial.println(logFilename);
    logFile.println(F("Time(ms),Altitude(m),VertVel(m/s),GyroX,GyroY,GyroZ,AngVelX,AngVelY,AccelX,AccelY,AccelZ,TVCx(deg),TVCy(deg),Phase,estZ(m),estVz(m/s),"
                      "gpsFix,gpsSats,gpsN(m),gpsE(m),gpsU(m),estN(m),estE(m),estVn(m/s),estVe(m/s),insX(m),insY(m),hdgSolved(deg)"));
    logFile.close();
  } else Serial.println(F("Failed to create log file!"));
}
// *** THE ASC038 FIX: NO SD ACCESS IN THE CONTROL LOOP. ***
// logData() used to do SD.open + println + close every 50 ms. On the flight card that cost a MEASURED
// mean of 143 ms (max 179 ms) per call -- 40x the healthy 3.5 ms loop period. The control loop was
// blocked inside the SD library for 88% of the burn, dt blew past the estimator's guard, the attitude
// froze, and the vehicle tumbled uncontrolled. The SD card is not the problem; calling it from the
// control loop is. Rows now go to RAM (a few hundred ns) and the whole file is written once, at
// recovery, exactly as MTR###/CTL### already did.
// RAM: 400 rows x ~104 B = ~42 KB (62 B of vehicle state + 42 B of GPS/nav). 400 rows at 20 Hz =
// 20 s, far beyond the ~8 s flight. Teensy 4.1 has 512 KB of RAM1 and the CTL buffers are the large
// consumer at ~92 KB, so this is comfortable.
constexpr int FLOG_BUF_N = 400;
uint32_t flogT[FLOG_BUF_N];
float    flogAlt[FLOG_BUF_N],  flogVz[FLOG_BUF_N];
float    flogGx[FLOG_BUF_N],   flogGy[FLOG_BUF_N],  flogGz[FLOG_BUF_N];
float    flogAvx[FLOG_BUF_N],  flogAvy[FLOG_BUF_N];
float    flogAx[FLOG_BUF_N],   flogAy[FLOG_BUF_N],  flogAz[FLOG_BUF_N];
float    flogTvx[FLOG_BUF_N],  flogTvy[FLOG_BUF_N];
float    flogEstZ[FLOG_BUF_N], flogEstVz[FLOG_BUF_N];
uint8_t  flogPhase[FLOG_BUF_N];
// GPS + passive horizontal nav. Raw measurement (gpsN/E/U) is stored ALONGSIDE the fused estimate
// (estN/estE/estVn/estVe) and the unfused inertial track (insX/insY) rather than instead of them:
// with all three plus the heading the entire fusion can be re-derived offline, so a filter tuning
// choice made today does not permanently destroy the flight's information content.
uint8_t  flogGpsFix[FLOG_BUF_N], flogGpsSats[FLOG_BUF_N];
float    flogGpsN[FLOG_BUF_N], flogGpsE[FLOG_BUF_N], flogGpsU[FLOG_BUF_N];
float    flogEstN[FLOG_BUF_N], flogEstE[FLOG_BUF_N];
float    flogEstVn[FLOG_BUF_N], flogEstVe[FLOG_BUF_N];
float    flogInsX[FLOG_BUF_N], flogInsY[FLOG_BUF_N], flogHdg[FLOG_BUF_N];
int      flogN = 0;

void logData(){                       // RAM only. Must stay allocation-free and SD-free.
  if(flogN >= FLOG_BUF_N) return;
  int i = flogN;
  flogT[i]=millis(); flogAlt[i]=altitude; flogVz[i]=vert_vel;
  flogGx[i]=gyro_x;  flogGy[i]=gyro_y;    flogGz[i]=gyro_z;
  flogAvx[i]=ang_vel_x; flogAvy[i]=ang_vel_y;
  flogAx[i]=accel_x; flogAy[i]=accel_y;   flogAz[i]=accel_z;
  flogTvx[i]=tiltX/SERVO_X_MULT; flogTvy[i]=tiltY/SERVO_Y_MULT;   // the COMMANDED TVC angle, deg
  flogEstZ[i]=estZ; flogEstVz[i]=estVz;
  flogPhase[i]= poweredFlight ? 1 : (inFlight ? 2 : (highest_alt>2.0f ? 3 : 0));
  flogGpsFix[i]=(uint8_t)gpsFixQual; flogGpsSats[i]=(uint8_t)gpsSats;
  flogGpsN[i]=gpsN; flogGpsE[i]=gpsE; flogGpsU[i]=gpsU;
  flogEstN[i]=estN; flogEstE[i]=estE; flogEstVn[i]=estVn; flogEstVe[i]=estVe;
  flogInsX[i]=insX; flogInsY[i]=insY; flogHdg[i]=navHeadingSolvedRad*RAD2DEG;   // the SOLVE, per row:
  // it evolves through the flight and watching it converge (or not) is the experiment. The heading
  // the filter actually used is constant and lives in the CTL header as heading_used_deg.
  flogN++;
}

// Writes every buffered row not yet on the card, then leaves the file closed. Called twice: once at
// BURNOUT (so the boost record survives a later reset) and once at RECOVERY (for the coast rows).
// Resumable rather than one-shot, so the second call only writes what the first did not.
int flogWritten = 0;
void dumpFlightLog(){
  if(flogWritten >= flogN) return;                 // nothing new to persist
  File f=SD.open(logFilename,FILE_WRITE);
  if(!f){ sdFailCount++; Serial.println(F("ASC log failed")); return; }
  char b[288];   // widened from 160 when the 12 GPS/nav columns were added -- a truncated row is a
                 // silently corrupt row, and snprintf truncates without complaining
  for(int i=flogWritten;i<flogN;i++){
    if((i & 63)==0){ updatePyros(); wdtFeed(); }   // same reason as the CTL dump: a long write must not hold a melt wire on
    snprintf(b,sizeof(b),"%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f"
                         ",%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f",
      (unsigned long)flogT[i],flogAlt[i],flogVz[i],flogGx[i],flogGy[i],flogGz[i],
      flogAvx[i],flogAvy[i],flogAx[i],flogAy[i],flogAz[i],
      flogTvx[i],flogTvy[i],(int)flogPhase[i],flogEstZ[i],flogEstVz[i],
      (int)flogGpsFix[i],(int)flogGpsSats[i],flogGpsN[i],flogGpsE[i],flogGpsU[i],
      flogEstN[i],flogEstE[i],flogEstVn[i],flogEstVe[i],flogInsX[i],flogInsY[i],flogHdg[i]);
    f.println(b);
  }
  f.close();
  snprintf(b,sizeof(b),"ASC -> %s  rows %d..%d of %d%s",logFilename,flogWritten,flogN-1,flogN,
           flogN>=FLOG_BUF_N ? "  *** BUFFER FULL, LOG TRUNCATED ***" : "");
  Serial.println(b);
  flogWritten = flogN;
}

// ============================================================================================
// SENSOR-BOARD SPI DRIVERS (ICM-42688-P + DPS310), added 2026-08-12 after bench bring-up.
// Both live on SPI1: MOSI 26, MISO 1, SCK 27. CS_IMU 0, CS_BARO 15. Validated on hardware with
// Impulse22_Bringup: ICM gave |A| = 1.007 g and <1 dps of gyro bias at rest, DPS310 held
// +/-0.01 hPa (~0.08 m). Written here as raw register access on purpose -- there is no library
// dependency to break, and every failure path is visible.
//
// *** AXIS SIGNS ARE NOT VERIFIED FOR THIS DEVICE. ***
// SERVO_X_SIGN / SERVO_Y_SIGN were bench-set on 2026-08-03 against the MPU-6050. The ICM is a
// different part on a different PCB in a different orientation, so NONE of that carries over
// (FAILURE_MODES.md D4). A wrong sign is not a degraded flight, it is guaranteed divergence.
// Re-run the pitch-the-nose check, set the map below, then set ICM_AXES_VERIFIED to 1.

constexpr float ICM_G0 = 9.80665f;
constexpr float ICM_ACC_LSB_PER_G   = 2048.0f;            // +/-16 g
constexpr float ICM_GYR_LSB_PER_DPS = 16.4f;              // +/-2000 dps
float icmBiasX = 0, icmBiasY = 0, icmBiasZ = 0;           // dps, in FIRMWARE axes (post-remap)

static SPISettings ICM_SPI_CFG(8000000, MSBFIRST, SPI_MODE0);
static SPISettings DPS_SPI_CFG(4000000, MSBFIRST, SPI_MODE0);

static uint8_t spi1Read(int cs, SPISettings s, uint8_t reg){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg|0x80); uint8_t v=SPI1.transfer(0);
  digitalWrite(cs,HIGH); SPI1.endTransaction(); return v;
}
static void spi1Burst(int cs, SPISettings s, uint8_t reg, uint8_t*buf, uint8_t n){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg|0x80); for(uint8_t i=0;i<n;i++) buf[i]=SPI1.transfer(0);
  digitalWrite(cs,HIGH); SPI1.endTransaction();
}
static void spi1Write(int cs, SPISettings s, uint8_t reg, uint8_t val){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg&0x7F); SPI1.transfer(val);
  digitalWrite(cs,HIGH); SPI1.endTransaction();
}
static int32_t sxt(uint32_t v, uint8_t bits){
  if(v & (1UL<<(bits-1))) v |= (0xFFFFFFFFUL<<bits);
  return (int32_t)v;
}

bool icm42688Begin(){
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x11, 0x01); delay(5);          // DEVICE_CONFIG soft reset
  if(spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x75) != 0x47) return false;  // WHO_AM_I
  uint8_t itf = spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x4C);
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x4C, (itf&0xFC)|0x03);         // lock to SPI, disable I2C
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x4F, 0x06);                    // GYRO  +/-2000 dps, 1 kHz
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x50, 0x06);                    // ACCEL +/-16 g,     1 kHz
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x4E, 0x0F);                    // both low-noise
  // UI filter bandwidth. Written ONLY if ICM_UI_FILT_CFG asks for it (0xFF = leave the power-on
  // default); always read back, so the configuration that produced a given tau is on the record
  // instead of being inferred. See the note at ICM_UI_FILT_CFG.
  if(ICM_UI_FILT_CFG != 0xFF) spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x52, ICM_UI_FILT_CFG);
  icmUiFiltReadback = spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x52);
  delay(50);                                                          // gyro start-up
  return true;
}

// Contract matches imuReadSource(): gyro deg/s with bias removed, accel m/s^2.
bool icm42688Read(float&gx,float&gy,float&gz,float&ax,float&ay,float&az){
  uint8_t b[14];
  spi1Burst(PIN_CS_IMU, ICM_SPI_CFG, 0x1D, b, 14);      // TEMP + ACCEL xyz + GYRO xyz
  int16_t ra[3] = { (int16_t)((b[2]<<8)|b[3]),  (int16_t)((b[4]<<8)|b[5]),  (int16_t)((b[6]<<8)|b[7])  };
  int16_t rg[3] = { (int16_t)((b[8]<<8)|b[9]),  (int16_t)((b[10]<<8)|b[11]),(int16_t)((b[12]<<8)|b[13])};
  // Reject the three ways this bus lies rather than errors. 0x8000 on every channel is the ICM's
  // documented no-data sentinel: the part reset and is sitting in standby with PWR_MGMT0 cleared,
  // and WHO_AM_I still answers 0x47 in that state, so an ID check will NOT catch it (seen on the
  // bench 2026-08-12). All-zero is MISO stuck low, all-ones is MISO floating. Any of them must
  // read as a failed sample so the failover above can act on it.
  if(ra[0]==-32768 && ra[1]==-32768 && ra[2]==-32768) return false;
  if((ra[0]|ra[1]|ra[2]|rg[0]|rg[1]|rg[2]) == 0)      return false;
  if(ra[0]==-1 && ra[1]==-1 && ra[2]==-1)             return false;

  float a3[3], g3[3];
  for(int i=0;i<3;i++){ a3[i]=ra[i]/ICM_ACC_LSB_PER_G*ICM_G0; g3[i]=rg[i]/ICM_GYR_LSB_PER_DPS; }
  ax = ICM_SGN_X*a3[ICM_MAP_X];  ay = ICM_SGN_Y*a3[ICM_MAP_Y];  az = ICM_SGN_Z*a3[ICM_MAP_Z];
  gx = ICM_SGN_X*g3[ICM_MAP_X] - icmBiasX;
  gy = ICM_SGN_Y*g3[ICM_MAP_Y] - icmBiasY;
  gz = ICM_SGN_Z*g3[ICM_MAP_Z] - icmBiasZ;
  return true;
}

// -- DPS310 barometer (SPI1, CS 15) --------------------------------------------
// Primary altitude source. The BMP280 backup (U6/U7 on the main board) is frequently not fitted,
// so baroReadAltitude() below uses whichever is actually present instead of halting on the BMP.
struct { int32_t c0,c1,c00,c10,c01,c11,c20,c21,c30; } dpsC;
bool  dpsPresent = false;
constexpr float DPS_KP = 7864320.0f;   // 8x  oversampling
constexpr float DPS_KT =  524288.0f;   // 1x  oversampling

bool dps310Begin(){
  spi1Write(PIN_CS_BARO, DPS_SPI_CFG, 0x0C, 0x89); delay(50);        // soft reset
  if(spi1Read(PIN_CS_BARO, DPS_SPI_CFG, 0x0D) != 0x10) return false; // PROD_ID
  uint32_t t0=millis();
  while(!(spi1Read(PIN_CS_BARO, DPS_SPI_CFG, 0x08) & 0xC0)){          // SENSOR_RDY | COEF_RDY
    if(millis()-t0 > 500) return false; delay(5);
  }
  // Infineon errata: re-trigger the OTP read after reset or some lots return wrong temp coefficients.
  spi1Write(PIN_CS_BARO,DPS_SPI_CFG,0x0E,0xA5); spi1Write(PIN_CS_BARO,DPS_SPI_CFG,0x0F,0x96);
  spi1Write(PIN_CS_BARO,DPS_SPI_CFG,0x62,0x02);
  spi1Write(PIN_CS_BARO,DPS_SPI_CFG,0x0E,0x00); spi1Write(PIN_CS_BARO,DPS_SPI_CFG,0x0F,0x00);

  uint8_t c[18]; spi1Burst(PIN_CS_BARO, DPS_SPI_CFG, 0x10, c, 18);
  dpsC.c0 =sxt(((uint32_t)c[0]<<4)|(c[1]>>4),12);
  dpsC.c1 =sxt((((uint32_t)c[1]&0x0F)<<8)|c[2],12);
  dpsC.c00=sxt(((uint32_t)c[3]<<12)|((uint32_t)c[4]<<4)|(c[5]>>4),20);
  dpsC.c10=sxt((((uint32_t)c[5]&0x0F)<<16)|((uint32_t)c[6]<<8)|c[7],20);
  dpsC.c01=sxt(((uint32_t)c[8]<<8)|c[9],16);   dpsC.c11=sxt(((uint32_t)c[10]<<8)|c[11],16);
  dpsC.c20=sxt(((uint32_t)c[12]<<8)|c[13],16); dpsC.c21=sxt(((uint32_t)c[14]<<8)|c[15],16);
  dpsC.c30=sxt(((uint32_t)c[16]<<8)|c[17],16);

  uint8_t srce = spi1Read(PIN_CS_BARO, DPS_SPI_CFG, 0x28) & 0x80;     // TMP_EXT must match the trim source
  spi1Write(PIN_CS_BARO, DPS_SPI_CFG, 0x06, 0x53);                    // PRS 32 Hz, 8x
  spi1Write(PIN_CS_BARO, DPS_SPI_CFG, 0x07, srce|0x50);               // TMP 32 Hz, 1x
  spi1Write(PIN_CS_BARO, DPS_SPI_CFG, 0x09, 0x00);                    // no shift needed at <=8x
  spi1Write(PIN_CS_BARO, DPS_SPI_CFG, 0x08, 0x07);                    // continuous pressure + temp
  delay(50);
  return true;
}

float dps310AltitudeM(float seaLevelHpa){
  uint8_t b[6]; spi1Burst(PIN_CS_BARO, DPS_SPI_CFG, 0x00, b, 6);
  float psc = sxt(((uint32_t)b[0]<<16)|((uint32_t)b[1]<<8)|b[2],24)/DPS_KP;
  float tsc = sxt(((uint32_t)b[3]<<16)|((uint32_t)b[4]<<8)|b[5],24)/DPS_KT;
  float pa  = dpsC.c00 + psc*(dpsC.c10 + psc*(dpsC.c20 + psc*dpsC.c30))
            + tsc*dpsC.c01 + tsc*psc*(dpsC.c11 + psc*dpsC.c21);
  return 44330.0f*(1.0f - powf((pa/100.0f)/seaLevelHpa, 1.0f/5.255f));
}

// Single altitude entry point. DPS310 if it came up, else the BMP280. Both absent is caught in
// setup() -- by here at least one is known good.
bool baroBmpPresent = false;
float baroReadAltitude(float seaLevelHpa){
  if(dpsPresent)     return dps310AltitudeM(seaLevelHpa);
  if(baroBmpPresent) return bmp.readAltitude(seaLevelHpa);
  return 0.0f;
}

// -- IMU read / calibrate (Adafruit_MPU6050) -----------------------------------
// ================= REDUNDANT IMU SOURCES + AUTOMATIC FAILOVER ================================
// Impulse 2.2 carries TWO independent inertial sources on TWO independent buses:
//   PRIMARY  ICM-42688-P on the sensor board, SPI  (CS_IMU 0, MISO 1, MOSI 26, SCK 27, INT 2)
//   BACKUP   MPU-6050 (U6) on the main board, I2C (SDA 18, SCL 19)
// Because the buses are separate, a wedged I2C bus or a dead device on one CANNOT take the other
// down. So losing the primary is a REDUCED-level event -- switch source, keep full quaternion
// attitude control, flight unaffected -- NOT a fallback to rate damping. Only losing BOTH costs
// attitude. This is the difference between "a sensor died and the flight continued" and "a sensor
// died and we glided into recovery", and it is the whole reason for carrying two.
//
// *** THE SPI PRIMARY DRIVER IS NOT WRITTEN. *** That is deliberate: it cannot be bench-verified
// without the board in hand, and an unverified driver in the control loop is exactly the class of
// risk that has already cost two airframes. It is stubbed to FAIL, which is safe by construction --
// the failover below catches it on the first read, switches to the proven MPU-6050 path, and records
// F_IMU_FAILOVER in the flight log. Flying today therefore runs on the backup with the redundancy
// logic exercised and logged. When the ICM-42688 driver is written AND bench-verified against a
// known rotation, implement imuReadPrimary() and the vehicle gains true redundancy with no other change.
// (SRC_*/imuActive/imuOk are declared with the other health globals near the top so the CTL dump,
//  which is defined earlier in the file, can report them.)
// Reads one source into the caller's floats in the FIRMWARE's units (gyro deg/s bias-removed,
// accel m/s^2). Returns false if the source did not produce a fresh, plausible sample.
bool imuReadSource(uint8_t src, float&gx, float&gy, float&gz, float&ax, float&ay, float&az){
  if(src == SRC_PRIMARY){
#if SENSOR_BOARD_ENABLE && defined(ICM42688_DRIVER_PRESENT)
    // Driver written and hardware-validated 2026-08-12. The axis SIGNS are still unverified --
    // see ICM_AXES_VERIFIED above; the startup banner shouts about it until the bench check is done.
    return icm42688Read(gx,gy,gz,ax,ay,az);
#else
    return false;                 // driver not written -> always fails over. Safe by construction.
#endif
  }
  sensors_event_t a, g, t;
  unsigned long t0 = micros();
  // The return value used to be DISCARDED, so a bus that NAKed or timed out still counted as a good
  // sample and the stale event struct was published as live data. Every downstream protection --
  // failover, the stuck detector, both-dead -- keys off this bool, so throwing it away disabled all of
  // them at once. A failed read must be reported as a failed read.
  if(!mpu.getEvent(&a, &g, &t)) return false;
  if(micros()-t0 > IMU_SLOW_US) raiseFault(F_IMU_SLOW, HEALTH_REDUCED);
  gx=(g.gyro.x-gyroBiasX)*RAD2DEG; gy=(g.gyro.y-gyroBiasY)*RAD2DEG; gz=(g.gyro.z-gyroBiasZ)*RAD2DEG;
  ax=a.acceleration.x; ay=a.acceleration.y; az=a.acceleration.z;
  return true;
}

void readIMU(){
  float ngx,ngy,ngz,nax,nay,naz;
  bool ok = imuReadSource(imuActive, ngx,ngy,ngz,nax,nay,naz);

  // --- STUCK DETECTION, per source -------------------------------------------------------------
  // A wedged bus / dead device returns the SAME bytes forever. Gating is deliberately strict because
  // an over-eager version is WORSE THAN NOTHING: an early draft fired on healthy flights, dropped the
  // vehicle to rate-only control and turned a PASS into a tumble in the SIL. All must hold:
  //   (a) POWERED FLIGHT only -- a burn guarantees vibration, so a live IMU cannot sit perfectly still.
  //       At rest on the pad, identical samples are normal and mean nothing.
  //   (b) ALL SIX axes bit-identical -- one axis parking on an LSB boundary is ordinary.
  //   (c) sustained for IMU_STUCK_N samples (~90 ms at loop rate).
  if(ok && poweredFlight && ngx==imu_gx && ngy==imu_gy && ngz==imu_gz &&
     nax==imu_ax && nay==imu_ay && naz==imu_az){
    if(++imuStuckRunSrc[imuActive] >= IMU_STUCK_N) ok = false;
  } else imuStuckRunSrc[imuActive] = 0;

  // --- FAILOVER ---------------------------------------------------------------------------------
  // The two sources are on independent buses, so the healthy one is genuinely unaffected by whatever
  // killed the other. Switching costs a REDUCED level and nothing else: full quaternion attitude,
  // full PD+I, flight continues. That is the point of carrying two.
  if(!ok){
    imuOk[imuActive] = false;
    uint8_t other = imuActive ^ 1;
    if(imuOk[other]){
      Serial.print(F("IMU FAILOVER: ")); Serial.print(SRC_NAME[imuActive]);
      Serial.print(F(" -> ")); Serial.println(SRC_NAME[other]);
      imuActive = other; imuFailoverCount++;
      imuStuckRunSrc[imuActive] = 0;
      raiseFault(F_IMU_FAILOVER, HEALTH_REDUCED);      // control UNAFFECTED
      ok = imuReadSource(imuActive, ngx,ngy,ngz,nax,nay,naz);
    }
  }
  if(!ok){
    // Both sources gone. Only now is attitude unrecoverable -- there is no third reference, and the
    // accelerometer cannot substitute (under thrust it reads thrust, in coast it reads free-fall).
    // Both sources gone. STOP ACTUATING ENTIRELY -- do not fall back to rate damping.
    // Rate damping needs a LIVE rate. With a frozen sensor the D term sees a constant non-zero rate
    // and commands a CONSTANT gimbal deflection, which is a constant torque: it does not damp the
    // vehicle, it spins it up. Measured in the SIL when this case first ran: 179.9 deg of tilt and
    // 620 deg/s, i.e. the "safe" fallback was actively flying the rocket into a tumble.
    // With no trustworthy sensor the only honest command is no command.
    raiseFault(F_IMU_BOTH_DEAD|F_IMU_STUCK, HEALTH_SURVIVE);
    attitudeTrusted = false;
    controlInhibited = true;                           // gimbal to true neutral, integrators zeroed
    return;                                            // keep the last good values; do not integrate garbage
  }
  imu_gx=ngx; imu_gy=ngy; imu_gz=ngz;
  imu_ax=nax; imu_ay=nay; imu_az=naz;                  // m/s^2
}
// Averages N samples at rest. BOTH sources are calibrated in the same pass, each in its own units
// (MPU rad/s, ICM dps in firmware axes). Calibrating only the active one would mean a mid-flight
// failover switched to a source carrying an uncorrected bias -- which the quaternion integrates
// straight into a drifting attitude, i.e. the failover would "work" and still lose the vehicle.
// Failed reads are excluded from their own average rather than counted as zero.
void cdPollButton();                        // fwd decl: defined with the countdown, ~800 lines below.
                                            // The 1.2 s loop here runs at T-3 and used to be totally
                                            // deaf to the abort button -- the single longest blind
                                            // window in the whole countdown, and the last moment you
                                            // would actually want to scrub.
void calibrateGyroBias(){
  const int N=600;                          // 1.2 s of sampling -- must feed the watchdog inside
  double sx=0,sy=0,sz=0;  int nm=0;         // MPU-6050, rad/s
  double ix=0,iy=0,iz=0;  int ni=0;         // ICM-42688-P, dps
  icmBiasX=icmBiasY=icmBiasZ=0;             // zero first: icm42688Read() subtracts these, and we want raw
  for(int i=0;i<N;i++){
    wdtFeed();
    cdPollButton();      // costs one digitalRead; advances no clock, so calibration timing is unmoved
    {
      // UNCONDITIONAL. In the SIL this call IS the clock: getEvent() steps sim_advance (physics +
      // time), so gating it on imuOk[] silently deletes 1.2 s of simulated flight whenever that flag
      // is false, shifting every downstream event. Guard the ACCUMULATION, never the call.
      sensors_event_t a,g,t;
      bool ok = mpu.getEvent(&a,&g,&t);
      if(ok && imuOk[SRC_BACKUP]){ sx+=g.gyro.x; sy+=g.gyro.y; sz+=g.gyro.z; nm++; }
    }
    if(imuOk[SRC_PRIMARY]){
      float gx,gy,gz,ax,ay,az;
      if(icm42688Read(gx,gy,gz,ax,ay,az)){ ix+=gx; iy+=gy; iz+=gz; ni++; }
    }
    delay(2);
  }
  if(nm>0){ gyroBiasX=(float)(sx/nm); gyroBiasY=(float)(sy/nm); gyroBiasZ=(float)(sz/nm); }
  if(ni>0){ icmBiasX =(float)(ix/ni); icmBiasY =(float)(iy/ni); icmBiasZ =(float)(iz/ni); }
  saveBenchCal();   // cache for bench use; flight always re-measures via countdown()
  Serial.print(F("gyro bias  MPU(dps) ")); Serial.print(gyroBiasX*RAD2DEG,3); Serial.print(' ');
  Serial.print(gyroBiasY*RAD2DEG,3); Serial.print(' '); Serial.print(gyroBiasZ*RAD2DEG,3);
  Serial.print(F("   ICM(dps) ")); Serial.print(icmBiasX,3); Serial.print(' ');
  Serial.print(icmBiasY,3); Serial.print(' '); Serial.print(icmBiasZ,3);
  Serial.print(F("   [n=")); Serial.print(nm); Serial.print('/'); Serial.print(ni); Serial.println(']');
}

// -- Quaternion attitude estimator (roll-aware) --------------------------------
// q = (qw,qx,qy,qz), body->world. REPLACES the old naive body-rate integration (gyro_x += ang_vel_x*dt),
// which treated body pitch/yaw RATES as if they were inertial tilt ANGLES -- valid only at small roll. Once
// the vehicle rolled ~180 deg (ASC007/ASC031: uncontrolled roll to ~190 deg) the body X/Y axes had swapped,
// so the naive "tilt" was wrong and the PD correction inverted -> roll-coupling divergence. The quaternion
// tracks full 3D orientation, so the tilt decomposition below stays correct at ANY roll.
// Body frame = the COMPENSATED sensor frame: rates (imu_gx, rateYBody(), imu_gz). rateYBody() is
// per-source: it un-inverts Y for the MPU-6050 and passes it straight through for the ICM-42688-P,
// whose axes do not need it. It used to be a hardcoded -imu_gy -- see the note at its definition.
// [historical] the -imu_gy un-inverts the
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

// ============================================================================================
// GPS DRIVER + PASSIVE HORIZONTAL NAVIGATOR
// ============================================================================================
// Placed here, after the quaternion, because navUpdate() needs the attitude to rotate body specific
// force into the world frame; the globals it maintains are declared ~800 lines up with the other
// health state so dumpControlLog() and logData() can both report them.
//
// The NMEA parser is a straight port of Impulse22_SensorCheck, which has a hardware fix on this
// board -- it is proven code moving into the flight build, not new code. Two deliberate reductions:
// GSV is not parsed (satellite-by-satellite detail is a bench diagnostic; GGA already carries the
// fix quality, satellite count and HDOP that matter in flight) and nothing blocks or retries.
#if GPS_ENABLE
char     gpsNmea[84]; uint8_t gpsNmeaLen=0;
// WGS84 meridional / prime-vertical radii AT THE DATUM. Computed once when the datum is captured:
// they vary by under a part in 10^6 over any distance a rocket covers, so recomputing per fix buys
// nothing. This beats the usual flat 111320 m/deg by correcting for flattening and latitude both.
double   gpsRm=6367449.0, gpsRn=6378137.0, gpsCosLat0=1.0;
void gpsComputeRadii(double latRad){
  const double a=6378137.0, f=1.0/298.257223563, e2=f*(2.0-f);
  double s=sin(latRad), w=1.0-e2*s*s;
  gpsRn = a/sqrt(w);
  gpsRm = a*(1.0-e2)/(w*sqrt(w));
  gpsCosLat0 = cos(latRad);      // hoisted out of gpsProject(): that runs per fix, inside the
                                 // control loop's call chain, and the datum latitude never changes
}
static double gpsNmeaDeg(const char*s){          // ddmm.mmmm -> decimal degrees
  if(!*s) return 0;
  double v=atof(s);
  int deg=(int)(v/100.0);
  return deg + (v - deg*100.0)/60.0;
}
static const char* gpsField(const char*s, int n){       // pointer to comma-field n (0-based)
  int f=0; while(*s && f<n){ if(*s==',') f++; s++; }
  return s;
}
static bool gpsCsumOk(const char*s){             // s starts at '$'
  const char* star = strchr(s,'*');
  if(!star || strlen(star)<3) return false;
  uint8_t sum=0;
  for(const char*p=s+1; p<star; p++) sum ^= (uint8_t)*p;
  return sum == (uint8_t)strtol(star+1,nullptr,16);
}
void gpsSetDatum(){
  gpsLat0=gpsLat; gpsLon0=gpsLon; gpsMsl0=gpsMsl;
  gpsComputeRadii(gpsLat0*M_PI/180.0);
  gpsOriginSet=true;
  gpsN=gpsE=0; gpsU=0;
}
void gpsProject(){
  if(!gpsOriginSet || !gpsHasFix) return;
  const double dLat=(gpsLat-gpsLat0)*M_PI/180.0, dLon=(gpsLon-gpsLon0)*M_PI/180.0;
  gpsN = (float)(gpsRm*dLat);
  gpsE = (float)(gpsRn*gpsCosLat0*dLon);
  gpsU = gpsMsl - gpsMsl0;
}
void gpsParseGGA(const char*s){
  const char* q = gpsField(s,6);  if(*q) gpsFixQual = atoi(q);
  const char* n = gpsField(s,7);  if(*n) gpsSats    = atoi(n);
  const char* h = gpsField(s,8);  if(*h) gpsHdop    = atof(h);
  const char* la= gpsField(s,2);  const char* ns = gpsField(s,3);
  const char* lo= gpsField(s,4);  const char* ew = gpsField(s,5);
  const char* al= gpsField(s,9);  if(*al) gpsMsl   = atof(al);
  if(gpsFixQual>0 && *la && *lo){
    double lat=gpsNmeaDeg(la), lon=gpsNmeaDeg(lo);
    if(*ns=='S') lat=-lat;
    if(*ew=='W') lon=-lon;
    gpsLat=lat; gpsLon=lon;
    if(!gpsHasFix && gpsFixUpdates) gpsReacquires++;   // had one, lost it, and it is back
    gpsHasFix=true; gpsFixUpdates++; gpsLastFixMs=millis();
    if(gpsSats<gpsSatsMin) gpsSatsMin=gpsSats;
    if(gpsSats>gpsSatsMax) gpsSatsMax=gpsSats;
    if(gpsHdop>0){ if(gpsHdop<gpsHdopMin) gpsHdopMin=gpsHdop; if(gpsHdop>gpsHdopMax) gpsHdopMax=gpsHdop; }
    // Datum quality gate -- see DATUM_MIN_SATS. Only ever captured on the ground: once liftoff is
    // confirmed the vehicle is moving, and a datum captured mid-flight would silently redefine the
    // origin partway through the record.
    if(!gpsOriginSet && !liftoffConfirmed && gpsSats>=DATUM_MIN_SATS && gpsHdop>0 && gpsHdop<=DATUM_MAX_HDOP){
      gpsSetDatum();
      Serial.print(F("GPS datum set: ")); Serial.print(gpsSats);
      Serial.print(F(" sats, hdop ")); Serial.println(gpsHdop,1);
    }
    gpsProject();
  } else {
    if(gpsHasFix) gpsFixLost++;
    gpsHasFix=false;
  }
}
// Drains at most GPS_MAX_BYTES_PER_CALL bytes. Bounded by construction -- see the constant.
void pumpGPS(){
  int budget = GPS_MAX_BYTES_PER_CALL;
  while(budget-- > 0 && GPS_SERIAL.available()){
    char c = (char)GPS_SERIAL.read();
    if(c=='$'){ gpsNmeaLen=0; gpsNmea[gpsNmeaLen++]=c; continue; }
    if(gpsNmeaLen==0) continue;
    if(c=='\r' || c=='\n'){
      gpsNmea[gpsNmeaLen]=0; gpsSentences++;
      if(gpsNmeaLen>9 && gpsCsumOk(gpsNmea)){
        if(strstr(gpsNmea,"GGA")) gpsParseGGA(gpsNmea);
      } else if(gpsNmeaLen>9) gpsCsumErr++;
      gpsNmeaLen=0; continue;
    }
    if(gpsNmeaLen < sizeof(gpsNmea)-1) gpsNmea[gpsNmeaLen++]=c;
  }
}
void gpsPpsISR(){ gpsPpsCount++; }
#else
inline void pumpGPS(){}
#endif

// ---- Passive horizontal Kalman (GPS + IMU), north/east ---------------------------------------
// Two independent per-axis filters of the SAME 3-state form as kfZ (position, velocity,
// accel-bias), so this reuses kfPredict/kfUpdate rather than carrying a second implementation that
// could drift out of agreement with the one the landing firmware ports.
KF kfN, kfE;
// Base horizontal position noise, SCALED BY HDOP at fusion. A consumer receiver sitting still
// scatters several metres and the scatter tracks HDOP directly, so a fixed R would over-trust a
// degraded fix -- which is exactly the condition a boost transient produces. 2.0 m at HDOP 1.
constexpr float GPS_POS_NOISE = 2.0f;
constexpr float GPS_HDOP_MAX_USE = 8.0f;   // above this the fix is not fused at all, only logged
// NMEA fix latency: a sentence reports where the vehicle WAS this long ago (receiver solve + 9600
// baud serial). Lead-compensated at fusion exactly as kfZ does for baro transport lag, so estN/estE
// track the present rather than trailing by v*latency. At 30 m/s that is ~6 m -- not a detail.
constexpr float GPS_LAT_S = 0.20f;
uint32_t navLastFixUsed = 0;               // gpsFixUpdates value at the last fusion (edge detect)

void navReset(){
  kfInit(kfN,0.0f); kfInit(kfE,0.0f);
  estN=estE=estVn=estVe=0;
  insX=insY=insVx=insVy=0;
  gpsN0=gpsN; gpsE0=gpsE;              // shared origin for the INS-vs-GPS comparison; see gpsN0
  navHdgA=navHdgB=0; navHeadingLocked=false; navHeadingSolvedRad=0;
  navHeadingRad = GPS_PAD_HEADING_DEG*0.017453292519943295f;
  navMaxHorizM=0; navGpsUpdates=0; navInsGpsResidM=0;
  navLastFixUsed = gpsFixUpdates;
}

// One step of the passive horizontal navigator. dt is the SAME clamped dt the attitude integrator
// and kfZ use, so a starved loop degrades all three identically rather than in three different ways.
void navUpdate(float dt){
  if(!(dt > 0.0f) || dt > DT_SANITY_S) return;
  // World-horizontal specific force. The world frame's THIRD row is what kfZ already uses via
  // quatUpInBody(); these are the first two rows of the same body->world rotation, so the three
  // axes are guaranteed mutually consistent. Gravity is purely vertical in the world frame and
  // therefore does not appear in the horizontal components at all -- no g subtraction here.
  const float wxx = 1.0f - 2.0f*(qy*qy + qz*qz), wxy = 2.0f*(qx*qy - qw*qz), wxz = 2.0f*(qx*qz + qw*qy);
  const float wyx = 2.0f*(qx*qy + qw*qz), wyy = 1.0f - 2.0f*(qx*qx + qz*qz), wyz = 2.0f*(qy*qz - qw*qx);
  float aWx = wxx*imu_ax + wxy*imu_ay + wxz*imu_az;      // m/s^2, firmware world +X
  float aWy = wyx*imu_ax + wyy*imu_ay + wyz*imu_az;      // m/s^2, firmware world +Y
  if(!isfinite(aWx) || !isfinite(aWy)) return;           // a poisoned quaternion must not poison the track

  // --- pure INS dead reckoning, firmware world frame, NO GPS ---
  insVx += aWx*dt; insVy += aWy*dt;
  insX  += insVx*dt; insY += insVy*dt;

  // --- fused filters, GPS north/east frame ---
  // (N,E) = ( wx*cos + wy*sin ,  wx*sin - wy*cos ). The second sign is the LEFT-handedness of NEU --
  // see the derivation at GPS_PAD_HEADING_DEG. A rotation here would mirror every horizontal result.
  const float ch=cosf(navHeadingRad), sh=sinf(navHeadingRad);
  kfPredict(kfN, aWx*ch + aWy*sh, dt);
  kfPredict(kfE, aWx*sh - aWy*ch, dt);

#if GPS_ENABLE
  // Fuse only on a genuinely NEW fix. GGA repeats at 1 Hz while this runs at ~300 Hz; updating on
  // every call would fuse the same measurement hundreds of times and collapse the covariance onto a
  // single noisy sample, which reads as a confident and wrong estimate.
  if(gpsOriginSet && gpsHasFix && gpsFixUpdates != navLastFixUsed &&
     gpsHdop > 0 && gpsHdop <= GPS_HDOP_MAX_USE){
    navLastFixUsed = gpsFixUpdates;
    const float r = GPS_POS_NOISE*gpsHdop; const float R = r*r;
    kfUpdate(kfN, 0, gpsN + estVn*GPS_LAT_S, R);
    kfUpdate(kfE, 0, gpsE + estVe*GPS_LAT_S, R);
    navGpsUpdates++;

    // --- pad-heading solve: least-squares fit of the INS track onto the GPS track ---------------
    // Minimising SUM |M(psi)*ins - gps|^2 over the LEFT-handed map above gives
    //     psi = atan2(B, A),  A = SUM(ix*gn - iy*ge),  B = SUM(ix*ge + iy*gn)
    // Accumulated over the whole flight rather than fitted per sample, so noise averages out instead
    // of making the heading jitter. Both tracks share an origin (the liftoff instant) -- see gpsN0.
    // DIAGNOSTIC BY DEFAULT: with NAV_HDG_FROM_PAD the result is recorded and NOT fed to the filter,
    // because measured across 25 seeds it is 13-27 deg wrong on a near-vertical ascent.
    const float dN = gpsN - gpsN0, dE = gpsE - gpsE0;
    navHdgA += (double)insX*(double)dN - (double)insY*(double)dE;
    navHdgB += (double)insX*(double)dE + (double)insY*(double)dN;
    const float insDisp = sqrtf(insX*insX + insY*insY);
    const float gpsDisp = sqrtf(dN*dN + dE*dE);
    if(insDisp > NAV_HDG_MIN_DISP_M && gpsDisp > NAV_HDG_MIN_DISP_M &&
       (navHdgA != 0.0 || navHdgB != 0.0)){
      float psi = atan2f((float)navHdgB, (float)navHdgA);
      if(isfinite(psi)){
        navHeadingSolvedRad = psi;
        navHeadingLocked = true;
        if(!NAV_HDG_FROM_PAD) navHeadingRad = psi;   // legacy self-alignment mode
      }
    }
    // Residual after alignment: how far the inertial track has drifted from the GPS track. This is
    // the number the landing firmware's dead-reckoning budget has to be sized against, and it has
    // never been measured on this vehicle. Scored against the heading the FILTER used, so with a
    // known pad heading it is a clean drift measurement instead of drift plus alignment error.
    const float rx = insX*ch + insY*sh, ry = insX*sh - insY*ch;
    navInsGpsResidM = sqrtf((rx-dN)*(rx-dN) + (ry-dE)*(ry-dE));
  }
#endif
  estN=kfN.s[0]; estVn=kfN.s[1];
  estE=kfE.s[0]; estVe=kfE.s[1];
  float h = sqrtf(estN*estN + estE*estE);
  if(isfinite(h) && h > navMaxHorizM) navMaxHorizM = h;
}

// -- Sensors (one attitude source at a time; identical policy to the flown code)
//    on the pad: accel levels the quaternion (gravity is a reliable reference)
//    in flight : quaternion propagated by the gyro (accel reads thrust+gravity, unusable as a reference)
// Re-identify the plant. Called once per control step from sensors(), with the SAME clamped dt the
// attitude integrator uses, so a starved loop cannot corrupt the estimate differently from the estimator.
void plantUpdate(float dt){
  // ---- mass properties at the current burn fraction ----------------------------------------
  // Exact at both endpoints (full grain / spent) and correct in between by construction: the
  // propellant is treated as a body at PROP_ARM_M whose mass shrinks, and everything is composed
  // about the GIMBAL PIVOT -- a fixed point -- then referred back to the moving CG at the end. Doing
  // it about the CG directly would need the CG before it is known.
  float mp   = PROP_MASS_KG * (1.0f - burnFrac);
  plantMass  = DRY_MASS_KG + mp;
  plantL     = (DRY_MASS_KG*DRY_L_M + mp*PROP_ARM_M) / plantMass;      // CG -> pivot, moves FORWARD
  float iPiv = (DRY_IYY + DRY_MASS_KG*DRY_L_M*DRY_L_M)                  // dry, about the pivot
             + (PROP_IOWN*(1.0f-burnFrac) + mp*PROP_ARM_M*PROP_ARM_M);  // propellant, about the pivot
  plantIyy   = iPiv - plantMass*plantL*plantL;                          // parallel axis, back to the CG
  if(!(plantIyy > 1e-5f)) plantIyy = DRY_IYY;                           // geometry typo guard

  // ---- thrust: measured amplitude on the curve's shape --------------------------------------
  float tau = (thrustOnsetMs && poweredFlight) ? (millis()-thrustOnsetMs)*0.001f : -1.0f;
  float ref = thrustRefN(tau);
  // Specific force along body Z is (T - D)/m in free flight -- gravity does NOT appear in it. On the
  // pad the rail carries the weight and a_z reads +1 g with no thrust at all, so this is gated on
  // liftoff: without that gate the estimator would read ~8.5 N of thrust from a vehicle sitting still.
  if(liftoffConfirmed && poweredFlight){
    float drag = 0.5f*RHO_AIR*S_REF_M2*CD_AXIAL*estVz*estVz;
    if(!(drag >= 0.0f) || drag > 5.0f) drag = 0.0f;        // NaN / runaway estimator guard
    thrustMeasN = plantMass*imu_az + drag;
    if(!(thrustMeasN > 0.0f)) thrustMeasN = 0.0f;
    // Identify the amplitude only where the curve is WELL CONDITIONED. Two separate hazards:
    //   ref -> 0 at both ends makes the ratio meaningless (0/0);
    //   the tail-off falls so steeply that the 0.30 s filter lags it, and the lag reads as a low scale.
    // Measured: tracking is within 0.5% of truth through the whole sustain, then degrades to 0.81 in
    // the last 0.3 s purely from that lag. Freezing the scale before the tail keeps the identified
    // number meaning what it says -- and by then the burn is over, so there is nothing left to schedule.
    if(ref > 8.0f && burnFrac < 0.92f && dt > 0.0f){
      float inst = thrustMeasN / ref;
      float a = dt/(THRUST_SCALE_TAU_S + dt);
      thrustScale += a*(inst - thrustScale);
      thrustScale = constrain(thrustScale, THRUST_SCALE_MIN, THRUST_SCALE_MAX);
    }
  }
  plantThrustN = (ref > 0.0f) ? thrustScale*ref : 0.0f;

  // ---- burn fraction, from delivered impulse -------------------------------------------------
  // Impulse rather than elapsed time: it is self-correcting against a motor that runs hot or cold,
  // and it does not care that ignition lag is 462 +- 58 ms.
  if(plantThrustN > 0.5f && dt > 0.0f) impulseNs += plantThrustN*dt;
  burnFrac = constrain(impulseNs/MOTOR_IMPULSE_NS, 0.0f, 1.0f);

  // ---- keff, with the estimate REJECTED rather than clamped when it is out of family ----------
  // Holding the last good value is the safe failure: the gimbal command is alpha/keff, so accepting a
  // collapsing keff would demand ever-larger deflections at exactly the moment the vehicle has least
  // authority to deliver them.
  float k = plantThrustN*plantL/plantIyy;
  if(k >= KEFF_MIN && k <= KEFF_MAX) keffEst = k;
}

void sensors(){
  wdtFeed();      // fed once per control iteration -- the primary heartbeat
  static unsigned long prevTime=millis(), angleTime=millis(); static float prev_alt=0;
  readIMU();
#if BOARD_IMPULSE_22
  sampleRails();      // 3 ADC reads at full loop rate -- catches the millisecond sags a 20 Hz log misses
#endif
  pumpGPS();          // bounded RX drain -- see GPS_MAX_BYTES_PER_CALL. Typical cost is 4 bytes of
                      // ring-buffer read; a GGA parse (~1 Hz) is tens of microseconds. Never blocks.
  float raw_avx =  imu_gx;    // deg/s, bias-removed, Adafruit auto-scaled (no FSR fix factor)
  float raw_avy = rateYBody();
  ang_vel_x = ANGVEL_ALPHA*raw_avx + (1-ANGVEL_ALPHA)*ang_vel_x;
  ang_vel_y = ANGVEL_ALPHA*raw_avy + (1-ANGVEL_ALPHA)*ang_vel_y;

  unsigned long angleNow=millis(); float dt=(angleNow-angleTime)/1000.0f; angleTime=angleNow;
  // --- QUATERNION attitude (roll-aware) -- replaces the old naive body-rate integration whose tilt estimate
  //     inverted near ~180 deg roll. On the PAD (!inFlight): continuously re-level q to gravity (accel is a
  //     valid reference at rest). In FLIGHT: PURE gyro propagation -- under thrust the accel reads thrust+
  //     gravity and is NOT a gravity reference, so NO accel correction (gyro drift over a ~3.5 s boost is
  //     small). tilt = body +Z (thrust axis) vs vertical, decomposed with the SAME sign convention as the old
  //     pad atan2 (gyro_x==atan2(ay,az), gyro_y==-atan2(ax,az) when leveled) so the pole-placed PD gains are UNCHANGED.
  // *** dt HANDLING -- REWRITTEN 2026-08-04 AFTER FLIGHT ASC038. READ BEFORE CHANGING. ***
  // This line used to be `if(dt>0 && dt<0.1f) quatPropagate(...)`. On ASC038 the SD writes in
  // logData() cost a measured mean 143 ms each, so 29 of 170 control iterations exceeded 100 ms --
  // only 17% of iterations but 88% of the elapsed burn. Every one of those SKIPPED propagation, the
  // quaternion stopped integrating, and the tilt estimate froze at -0.10/-2.43 deg for the entire
  // flight while the vehicle actually tumbled past 160 deg at 248 deg/s. The P term went dead, the
  // 45 deg abort (which reads gyro_x/gyro_y) was blinded, and the rocket was never under control.
  // SKIPPING IS THE WORST POSSIBLE RESPONSE to a long dt: it silently substitutes "no rotation
  // happened" for "I do not know", with no indication anywhere. Now:
  //   - a long-but-real dt is INTEGRATED, so the estimate degrades gracefully instead of freezing;
  //   - only an absurd dt (>DT_SANITY_S, i.e. a stall or a millis() rollover) is dropped;
  //   - both cases are COUNTED and written to the CTL###.CSV header, so a starved loop can never
  //     again be invisible in the data.
  if(inFlight){
    if(dt > DT_SANITY_S){ dtDroppedCount++; }                 // genuinely broken -- integrating it would be worse
    else if(dt > 0){
      if(dt > DT_WARN_S) dtLongCount++;                       // real but concerning: recorded, still integrated
      quatPropagate(imu_gx, rateYBody(), imu_gz, dt);
      gyro_z += imu_gz*dt;                                    // roll, same policy
    }
  }
  else quatFromAccel();
  float ubx,uby,ubz; quatUpInBody(ubx,uby,ubz);     // world-up in the body frame
  float prevGx = gyro_x, prevGy = gyro_y;
  gyro_x =  atan2f(uby, ubz)*RAD2DEG;               // tilt in the firmware's X control channel (deg)
  gyro_y = -atan2f(ubx, ubz)*RAD2DEG;               // tilt in the Y control channel (deg)

  // --- ATTITUDE-STALENESS DETECTOR ------------------------------------------------------------
  // The exact ASC038 signature, made detectable: the gyro says the body is turning, but the tilt
  // estimate is not moving. On that flight the estimate sat bit-identical at -0.10/-2.43 deg for
  // the whole burn while the real body rate reached 248 deg/s, and NOTHING noticed. This is a
  // cross-check between two independent quantities the firmware already has, so it catches a frozen
  // estimator whatever the cause -- starved loop, wedged bus, or a bug not yet written.
  //
  // *** REWRITTEN 2026-08-15 AFTER A GROUND TEST FIRED IT ON A PERFECTLY HEALTHY ESTIMATE. ***
  // The old test was PER SAMPLE: rate > 25 dps AND tilt moved < 0.05 deg since the last iteration.
  // Movement per iteration is rate * loop period, so that 0.05 silently encoded a loop period -- and
  // the loop period just changed by 40x. Moving to the ICM-42688-P on SPI took an iteration from
  // ~3450 us to ~85 us, and the detector inverted:
  //     loop 3.5 ms  -> fires only below 14 dps, which the 25 dps gate already excludes -> never
  //     loop  85 us  -> fires below 588 dps, i.e. on essentially ANY real rotation
  // On CTL001 (ground test, board rotated by hand at 35-45 dps) it declared a tilt estimate that was
  // tracking perfectly to be stale, cleared attitudeTrusted, and dropped the vehicle to rate damping.
  // Boost body rate is p90 ~53 dps, so in flight this would have fired on every launch.
  //
  // The fix is to compare quantities that are both RATES, so the loop period cancels instead of
  // being baked into a constant. Movement and the rotation that should have caused it are integrated
  // over a fixed TIME window and compared as a ratio. Nothing here changes with the loop rate, which
  // is the property the old version lacked and the whole reason it broke.
  if(inFlight){
    float rawRate = sqrtf(imu_gx*imu_gx + imu_gy*imu_gy);
    float moved   = fabsf(gyro_x-prevGx) + fabsf(gyro_y-prevGy);
    // dt is CLAMPED, not gated. Gating on `dt <= DT_SANITY_S` would have stopped accumulating in
    // precisely the case this detector exists for: an absurd dt makes quatPropagate skip, so the
    // estimate freezes while the gyro keeps reporting rotation -- the ASC038 signature exactly. A
    // detector that goes blind at the same moment its failure arrives is not a detector, which is
    // the lesson C2b already recorded about the aborts. Clamping bounds the expectation without
    // creating that hole.
    if(dt > 0){
      float dtc = (dt > DT_SANITY_S) ? DT_SANITY_S : dt;
      attStaleExpDeg += rawRate*dtc;     // rotation the gyro says happened
      attStaleGotDeg += moved;           // rotation the attitude estimate actually showed
      attStaleWinS   += dtc;
    }
    if(attStaleWinS >= ATT_STALE_WINDOW_S){
      // Only judge a window with enough rotation in it to be judged. Below this the vehicle is
      // essentially still and the ratio is noise divided by noise.
      if(attStaleExpDeg >= ATT_STALE_MIN_ROT_DEG &&
         attStaleGotDeg < ATT_STALE_TRACK_FRAC*attStaleExpDeg && attitudeTrusted){
        // Do NOT keep steering on an attitude that is demonstrably not tracking reality. There is
        // no second attitude source on this vehicle, so fall back to rate damping and get the
        // parachute out early rather than pretending to fly.
        attitudeTrusted = false;
        raiseFault(F_ATT_STALE, HEALTH_FALLBACK);
      }
      attStaleExpDeg = attStaleGotDeg = 0; attStaleWinS = 0;
    }
  } else { attStaleExpDeg = attStaleGotDeg = 0; attStaleWinS = 0; }
  // --- NaN GUARD ------------------------------------------------------------------------------
  // A single non-finite value poisons the quaternion permanently and every downstream number becomes
  // NaN: comparisons silently go false, so the 45 deg abort would never fire and nothing would look
  // wrong in the log except zeros. Cheap to check, catastrophic to miss.
  if(!isfinite(gyro_x) || !isfinite(gyro_y) || !isfinite(qw) || !isfinite(qx) || !isfinite(qy) || !isfinite(qz)){
    qw=1; qx=qy=qz=0; gyro_x=gyro_y=0;
    attitudeTrusted = false;                       // re-levelling mid-flight is impossible; fall back
    raiseFault(F_NAN_ATT, HEALTH_FALLBACK);
  }
  // --- SENSOR SATURATION ----------------------------------------------------------------------
  // A reading pinned at full scale is a clip, not a measurement, and rates integrated from clipped
  // gyro data drift in a way that looks entirely plausible. Flag it so the log says so.
  if(fabsf(imu_gx) > SAT_FRAC*GYRO_FS_DPS || fabsf(imu_gy) > SAT_FRAC*GYRO_FS_DPS ||
     fabsf(imu_gz) > SAT_FRAC*GYRO_FS_DPS || fabsf(imu_az) > SAT_FRAC*ACCEL_FS_MPS2)
    raiseFault(F_SAT_SENSOR, HEALTH_REDUCED);

  // --- LIFTOFF CONFIRMATION -------------------------------------------------------------------
  // Two independent pieces of evidence, either sufficient: the barometer says we are clearly off the
  // pad, or the accelerometer saw real thrust. Latched -- once true it stays true, because the
  // recovery interlock must not be defeated by a momentary sensor dropout late in the flight.
  if(!liftoffConfirmed && (altitude > LIFTOFF_ALT_M || imu_az > THRUST_ONSET_G*G)){
    liftoffConfirmed = true;
    // Zero the PASSIVE horizontal navigator at the liftoff edge. On the pad the vehicle is not
    // moving, so every metre of GPS scatter and every milli-g of accelerometer bias is winding the
    // filters up against a truth of exactly zero -- the accel-bias states in particular would enter
    // flight already committed to absorbing pad noise. Starting the track at liftoff makes "distance
    // travelled" mean what it says. Purely a logging concern; nothing here gates control.
    gpsFixAtLiftoff = gpsHasFix;
    navReset();
  }

  // (roll is now integrated inside the attitude block above, under the same dt policy)
  accel_x=imu_ax; accel_y=imu_ay; accel_z=imu_az;   // m/s^2 (already scaled)

  // Baro is an I2C transaction and is the largest sheddable cost in this loop. It does not need to
  // run at control rate -- apogee detection is a ~10 Hz problem. Polled at baroIntervalMs, which the
  // loop-health governor stretches to 100 ms if the loop starts running late.
  {
    static unsigned long baroPrev = 0;
    unsigned long bnow = millis();
    if(bnow - baroPrev >= baroIntervalMs){
      baroPrev = bnow;
      altitude = baroReadAltitude(SEA_LEVEL_HPA) - initial_alt;
      if(altitude>highest_alt) highest_alt=altitude;
    }
  }

  // PASSIVE vertical Kalman (logged, NOT used for control). aWz = world-vertical accel = (body specific force
  // rotated up) - g; (ubx,uby,ubz) IS the strapdown rotation's 3rd row, so reuse it. Baro update lead-compensated.
  // Same dt policy as the attitude block. On ASC038 this froze too -- estZ/estVz sat at -0.18/-0.42
  // for the whole flight -- which is why "just use the Kalman for apogee" would NOT have helped.
  if(dt > 0 && dt <= DT_SANITY_S){
    float aWz = ubx*imu_ax + uby*imu_ay + ubz*imu_az - G;
    kfPredict(kfZ, aWz, dt);
    kfUpdate(kfZ, 0, altitude + estVz*BARO_LAG_S, BARO_NOISE*BARO_NOISE);
    estZ=kfZ.s[0]; estVz=kfZ.s[1];
  }

  // PASSIVE horizontal navigator (GPS + IMU), on the same dt policy and for the same reason as the
  // vertical filter above: logged, never used for control. See the block at its definition.
  navUpdate(dt);

  // Re-identify the plant on the same clamped dt. Placed AFTER the Kalman because the drag term in the
  // thrust estimate reads estVz, and one step of staleness there is worth less than the clarity of
  // having exactly one place where dt is decided.
  if(dt > 0 && dt <= DT_SANITY_S) plantUpdate(dt);

  unsigned long now=millis(), elapsed=now-prevTime;
  if(elapsed>=logIntervalMs && loggingOn){   // rate is shed by the loop-health governor if needed
    float raw_vel=(altitude-prev_alt)/(elapsed/1000.0f);
    vert_vel = VEL_ALPHA*raw_vel + (1-VEL_ALPHA)*vert_vel;
    prev_alt=altitude; prevTime=now; logData();
  }
  updatePyros();
  emergency();
}

// -- Emergency: only during the burn, tumble past EMERGENCY_TILT -> chute -------
void emergency(){
  if(!poweredFlight){ rateHighSince=0; blindSince=0; return; }
  // (1) TILT abort -- the original. Depends on the quaternion estimator.
  bool tiltAbort = (fabsf(gyro_x)>EMERGENCY_TILT || fabsf(gyro_y)>EMERGENCY_TILT);
  // (2) RATE abort -- attitude-independent, straight off the raw bias-removed gyro. See the constants.
  //     Survives a frozen/failed estimator, which is exactly the case that blinded ASC038.
  float rawRate = sqrtf(imu_gx*imu_gx + imu_gy*imu_gy);
  bool rateAbort = false;
  if(rawRate > EMERGENCY_RATE_DPS){
    if(rateHighSince==0) rateHighSince = millis();                    // start the dwell timer
    else if(millis()-rateHighSince >= EMERGENCY_RATE_MS) rateAbort = true;
  } else rateHighSince = 0;                                           // must be SUSTAINED, not a transient
  if(rateAbort) raiseFault(F_RATE_ABORT, HEALTH_SURVIVE);
  // (3) BLIND abort -- sensor-independent, because there is no sensor left. See BLIND_ABORT_MS.
  //     Keyed on the fault bit rather than controlInhibited so a future use of that flag for a
  //     recoverable condition can never silently arm a parachute.
  bool blindAbort = false;
  if(faultFlags & F_IMU_BOTH_DEAD){
    if(blindSince==0) blindSince = millis();
    else if(millis()-blindSince >= BLIND_ABORT_MS) blindAbort = true;
  } else blindSince = 0;                                              // a failover recovered us
  if(!tiltAbort && !rateAbort && !blindAbort) return;
  Serial.print(F("ABORT cause: "));
  Serial.println(tiltAbort ? F("TILT")
                : rateAbort ? F("BODY RATE (estimator-independent)")
                            : F("NO INERTIAL SOURCE (blind -- tumble assumed)"));
  neutralServos();
  Serial.println(F("EMERGENCY"));
  LED(true,false,false);
  triggerPyro(PYRO_CHUTE);         // parachute first
  triggerPyro(PYRO_LEGS);          // then legs -- protect the TVC on the (tumbling) recovery landing
  poweredFlight=false; inFlight=false;
  setPhase(3);                     // chute out: a reboot after this must not re-arm the resume watchdog
  // Both pyros have ALREADY fired -- this only delays the beeping, and the dumps tick updatePyros() internally so
  // the 1000 ms melt-wire pulse is unaffected. An abort is precisely the flight whose control trace you need
  // (ASC007 aborted at 43 deg with the gimbal saturated); without this the buffers die in RAM on every failure.
  dumpFlightLog(); dumpMotorLog(); dumpControlLog();
  // Beep until the button is pressed, then RESTART into ARM (see exitOnButton/resetFlightComputer).
  // Previously a press only moved to an inner while(true) -- it silenced the buzzer but the computer was
  // still stuck, so recovering the vehicle meant unplugging a battery from a possibly-live pyro bank.
  while(!exitOnButton()){ wdtFeed(); updatePyros(); beep(500,50); delay(50); }
  Serial.println(F("EMERGENCY cleared by button -- restarting"));
  LED(false,false,false);
  resetFlightComputer();
}

// -- Arm: 5 rapid button presses ----------------------------------------------
// ============================================================================================
// BENCH TVC MODE -- entered by HOLDING the button at ARM. Runs the real flight control path so the
// SERVO_*_SIGN / ICM_MAP_* verification is done against the code that actually flies, not a
// re-implementation that could be wrong in a different way.
//
// Safe by construction rather than by flag:
//   * poweredFlight stays false, so emergency() returns at its first line and hand-tilting the
//     airframe cannot trip a tilt or rate abort.
//   * every pyro gate is driven LOW on EVERY iteration, unconditionally. Nothing in this loop
//     calls triggerPyro(), and even a stray state change cannot energise a channel.
//   * no countdown, no ignition, no sequence, no log files.
// Exits on a button press, straight back to ARM.
constexpr unsigned long BENCH_HOLD_MS = 2000;   // hold this long at ARM to enter
bool benchMode = false;

// Bench gyro-bias cache. The countdown's 1.2 s pad calibration remains authoritative and overwrites
// this every launch; the cache only spares you re-running that calibration for every bench session.
// Flight NEVER substitutes it for the pad measurement -- countdown() calls calibrateGyroBias()
// unconditionally, and that is what writes this in the first place.
static uint32_t benchCalSum(const BenchCal&c){
  const uint8_t* p=(const uint8_t*)&c; uint32_t s=0;
  for(size_t i=0;i<sizeof(BenchCal)-sizeof(uint32_t);i++) s = s*31u + p[i];
  return s;
}
void saveBenchCal(){
  BenchCal c;
  c.magic=BENCHCAL_MAGIC;
  c.gbx=gyroBiasX; c.gby=gyroBiasY; c.gbz=gyroBiasZ;
  c.ibx=icmBiasX;  c.iby=icmBiasY;  c.ibz=icmBiasZ;
  c.sum=benchCalSum(c);
  EEPROM.put(BENCHCAL_ADDR,c);
}
bool loadBenchCal(){
  BenchCal c; EEPROM.get(BENCHCAL_ADDR,c);
  if(c.magic!=BENCHCAL_MAGIC || c.sum!=benchCalSum(c)) return false;
  gyroBiasX=c.gbx; gyroBiasY=c.gby; gyroBiasZ=c.gbz;
  icmBiasX =c.ibx; icmBiasY =c.iby; icmBiasZ =c.ibz;
  return true;
}

// 0 = nothing, 1 = short press (released before the threshold), 2 = long hold.
// Waits for release in both cases, so the caller counts presses itself.
int armGesture(){
  if(digitalRead(BUTTON) != HIGH) return 0;
  unsigned long t0 = millis();
  bool ramping = false;
  while(digitalRead(BUTTON) == HIGH){
    wdtFeed();
    unsigned long held = millis() - t0;

    // Past ~300 ms this is no longer a press, it is a hold. Say so continuously rather than
    // leaving the operator guessing whether anything is happening: the tone sweeps upward and the
    // blue flash accelerates, both tracking progress toward the threshold.
    if(held > 300){
      ramping = true;
      tone(BUZZER, 300 + (int)(1100.0f * held / BENCH_HOLD_MS));
      unsigned long period = 400 - (300 * held) / BENCH_HOLD_MS;   // 400 ms -> 100 ms
      if(period < 60) period = 60;
      LED(false, false, ((held / period) % 2) == 0);
    }

    if(held >= BENCH_HOLD_MS){
      noTone(BUZZER);
      LED(false, true, true);                                      // cyan = bench
      beep(1200, 90); beep(1600, 160);                             // distinct two-tone confirm
      while(digitalRead(BUTTON) == HIGH) wdtFeed();                // consume the rest of the hold
      return 2;
    }
  }
  if(ramping){                       // released mid-ramp: cancel, and do NOT count it as an arm press
    noTone(BUZZER); LED(false,false,false);
    beep(500,60); beep(380,90);      // falling two-tone = aborted
    return 0;
  }
  return 1;
}

void benchTVC(){
  benchMode = true;
  Serial.println(F("\n================= BENCH TVC MODE ================="));
  Serial.println(F("Real control path. Pyros hard-inhibited. No ignition."));
  Serial.println(F("Tilt the airframe: the gimbal must deflect to OPPOSE the tilt."));
  Serial.println(F("  wrong axis responds -> fix ICM_MAP_X/Y/Z"));
  Serial.println(F("  right axis, wrong way -> flip ICM_SGN_X / SERVO_X_SIGN"));
  Serial.println(F("Press the button to exit.\n"));

  for(auto&c:pyros){ digitalWrite(c.pin,LOW); c.active=false; }
  if(loadBenchCal()){
    Serial.print(F("gyro bias from EEPROM cache -- ICM(dps) "));
    Serial.print(icmBiasX,3); Serial.print(' '); Serial.print(icmBiasY,3); Serial.print(' ');
    Serial.println(icmBiasZ,3);
  } else {
    Serial.println(F("no cached bias -- measuring now, hold the airframe STILL for 1.2 s"));
    calibrateGyroBias();
  }
  poweredFlight=false; inFlight=false; controlInhibited=false;
  keffEst = keffNominal;              // no thrust to identify from; keep the gain schedule inert
  quatFromAccel();                    // level to however the airframe is sitting right now
  iTermX=iTermY=0;

  uint32_t lastPrint=0;
  while(!exitOnButton()){
    wdtFeed();
    for(auto&c:pyros) digitalWrite(c.pin,LOW);   // every iteration, unconditionally
    // attitudeTrusted and controlInhibited are ONE-WAY latches in flight: readIMU() sets both when
    // no inertial source answers, and nothing ever clears them. With the MPU-6050 backup unfitted,
    // a single dropped SPI read therefore disables the P term for the rest of the session and
    // leaves rate damping only -- which is precisely what the bench showed (servo tracking rate,
    // ignoring a 47 deg tilt). Latching is right for flight; for bench work it just means one
    // glitch silently invalidates everything after it, so clear them here every iteration.
    attitudeTrusted = true; controlInhibited = false;
    TVC();                                       // the real thing: sensors() + control + servos
    // Kill the integrator every iteration. On a bench the gimbal cannot actually rotate the
    // airframe, so ANY residual tilt error integrates without limit until it clamps -- the servo
    // then sits at an offset that has nothing to do with the current attitude and everything to do
    // with how long you have been holding it. For sign verification you want pure P+D on the
    // measured tilt, which is what this leaves.
    iTermX = 0; iTermY = 0;
    LED(false,true,true);                        // cyan = bench
    uint32_t now=millis();
    if(now-lastPrint>=200){
      lastPrint=now;
      Serial.print(F("tiltX ")); Serial.print(gyro_x,2);
      Serial.print(F("  tiltY ")); Serial.print(gyro_y,2);
      Serial.print(F("  rate ")); Serial.print(imu_gx,1); Serial.print('/'); Serial.print(imu_gy,1);
      Serial.print(F("   servoX ")); Serial.print(lastServoX);
      Serial.print(F("  servoY ")); Serial.print(lastServoY);
      // Print the flags that can silently delete the P term. Without these, "P does nothing" is
      // indistinguishable from "P is small", and that cost a whole bench session to work out.
      Serial.print(F("   att=")); Serial.print(attitudeTrusted?F("ok"):F("UNTRUSTED"));
      Serial.print(F(" inh=")); Serial.print(controlInhibited?F("YES"):F("no"));
      Serial.print(F(" src=")); Serial.print(imuActive==SRC_PRIMARY?F("ICM"):F("MPU"));
      Serial.print(F(" flt=")); Serial.println((int)faultFlags);   // int: the SIL shim has no HEX and no ulong overload
    }
  }
  neutralServos();
  for(auto&c:pyros){ digitalWrite(c.pin,LOW); c.active=false; }
  LED(false,false,false);
  benchMode=false;
  Serial.println(F("bench mode exit -> ARM\n"));
}

bool buttonCount(){
  constexpr int PRESS_WINDOW=500, PRESSES_REQD=5;
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
// *** ABORT SAMPLING -- REWRITTEN 2026-08-15 AFTER A BENCH REPORT. READ BEFORE CHANGING. ***
// The button used to be read at exactly ONE INSTANT per countdown tick, between the blocking beeps.
// A tick is `beep(...,200)` + `delay(800)`, and neither samples the button, so for ~1000 ms out of
// every 1000 ms the press was invisible. The abort therefore only worked if the button happened to
// be held down at the single moment it was sampled -- which on the bench feels exactly like "you
// have to hold it, or press it at the right time". It was not a debounce problem; the press was
// never seen at all.
//
// The previous comment here defended that as a deliberate trade ("a 5 ms polling loop replacing
// delay(800) measurably changed the countdown's duration and walked an injected reset across the
// burnout boundary in regression 6b"). The timing concern was real. The conclusion was not: the fix
// is to poll WITHOUT changing the elapsed time, not to stop polling.
//
// cdWait() below sleeps in 5 ms slices that sum to EXACTLY the requested duration, so the countdown
// takes the same time to the millisecond and every downstream flight timing is unmoved. The press is
// LATCHED, so it cannot be missed no matter when in the tick it happens.
//   deadtime  the last ARM press must not cancel the countdown it just started. The latch is
//             actively CLEARED throughout the deadtime rather than merely ignored, so a press that
//             is still being released as the countdown begins cannot scrub it a moment later.
//   debounce  30 ms of CONTINUOUS press. A plain debounce, not a hold -- a human tap is 50-150 ms,
//             so one deliberate press always scrubs the launch. It rejects contact bounce and the
//             SIL's alternating single-sample button model (which returns HIGH/LOW on successive
//             reads and so can never accumulate 30 ms), which would otherwise cancel every
//             simulated countdown.
constexpr unsigned long COUNTDOWN_ABORT_DEADTIME_MS = 1000;
constexpr unsigned long COUNTDOWN_ABORT_CONFIRM_MS  = 30;
constexpr unsigned long CD_SLICE_MS = 5;        // poll granularity inside the countdown's waits
// The countdown ticks once a second. Landing the FIRST tick exactly 1 s after the last arm press
// makes arming flow into the countdown as one rhythm instead of stopping and restarting.
// Measured FROM THE PRESS, not from countdown() entry, so the SD file creation in between is
// ABSORBED by this wait rather than added to it -- otherwise the first beep would drift by however
// long the card took, which is variable and card-dependent.
constexpr unsigned long ARM_TO_FIRST_BEEP_MS = 1000;
unsigned long armReleaseMs = 0;                 // millis() of the fifth arm press's release

bool          cdAbortLatched = false;
unsigned long cdBtnDownSince = 0;               // 0 = button is up
unsigned long cdStartMs      = 0;               // set by countdown(); gates the deadtime

// Sampled from every wait in the countdown, and from calibrateGyroBias() -- which blocks for 1.2 s
// at T-3 and was a completely deaf window before this.
void cdPollButton(){
  if(digitalRead(BUTTON) == HIGH){
    if(cdBtnDownSince == 0) cdBtnDownSince = millis();
    else if(millis() - cdBtnDownSince >= COUNTDOWN_ABORT_CONFIRM_MS) cdAbortLatched = true;
  } else cdBtnDownSince = 0;                    // released before the debounce -- bounce, not a press
  // Deadtime: discard anything latched, INCLUDING a still-held arm press. Once the deadtime passes,
  // a button that is genuinely still down re-latches on the next poll, which is the right behaviour.
  if(millis() - cdStartMs <= COUNTDOWN_ABORT_DEADTIME_MS) cdAbortLatched = false;
}

// Waits out the press that triggered an abort, so releasing it does not immediately read as arm
// press #1. BOUNDED, for the reason exitOnButton() gives: an unbounded wait on a stuck or shorted
// button hangs forever *while feeding the watchdog*, which is the one hang the watchdog cannot
// rescue. This was `while(digitalRead(BUTTON)==HIGH) wdtFeed();` -- no timeout at all.
// The delay(1) is what makes the bound REAL rather than nominal: a spin that never advances millis()
// can never reach its own timeout. (It also makes the abort testable in the SIL, where nothing else
// in the loop moves the clock -- the first version of this deadlocked the harness outright.)
void cdConsumePress(){
  unsigned long t0 = millis();
  while(digitalRead(BUTTON)==HIGH && millis()-t0 < 5000){ wdtFeed(); delay(1); }
}

// Waits `ms`, polling throughout. Returns true if an abort latched, and returns EARLY in that case
// only -- an aborted countdown has no downstream timing for anything to depend on. On the non-abort
// path the slices sum to exactly `ms`, which is the property regression 6b needs.
bool cdWait(unsigned long ms){
  unsigned long done = 0;
  while(done < ms){
    unsigned long step = (ms - done < CD_SLICE_MS) ? (ms - done) : CD_SLICE_MS;
    delay(step);
    done += step;
    wdtFeed();
    cdPollButton();
#if BOARD_IMPULSE_22
    sampleRails();     // pack lights stay live for the whole countdown, not one instant per second
#endif
    pumpGPS();         // and the receiver keeps converging through it
    if(cdAbortLatched) return true;
  }
  return false;
}

// The countdown's beep pitch, and the note the ARM press sequence resolves onto. Shared so the two
// cannot drift apart: the arm tones are derived DOWNWARD from this, so the final press lands exactly
// on it. Previously the presses ran 311.13*2^(n/12), which put the fifth at 415.30 Hz -- a semitone
// under the countdown, so the sequence sounded like it stopped one step short of arriving.
constexpr int ARM_TONE_TOP_HZ = 440;   // A4
// Now just reads the latch that cdPollButton() maintains. Kept as a function (rather than reading
// cdAbortLatched directly at the call sites) so the deadtime rule lives in exactly one place, and
// polled once more here to catch a press made during a stretch that did not go through cdWait().
bool countdownAborted(unsigned long cdStart){
  (void)cdStart;                 // deadtime is keyed on cdStartMs, set by countdown()
  cdPollButton();
  return cdAbortLatched;
}

// Shared tail for every pre-arm gate. Returns true if the caller must abort.
// In flight builds that is always -- the alarm sounds and the countdown ends. Under GROUND_TEST the
// gate still SAYS it failed (the diagnostics above each call site are unchanged and still print),
// but the countdown continues, with a visually distinct cue so a bypass can never be mistaken for a
// clean pad. Ignition is inhibited separately, at the bottom of countdown().
bool padGateFail(){
#if GROUND_TEST
  Serial.println(F("   >>> GROUND TEST: gate BYPASSED, continuing. MOTOR CANNOT LIGHT. <<<"));
  LED(true,false,true);                                     // magenta = bypassed, not the red alarm
  for(int i=0;i<3;i++){ wdtFeed(); beep(1500,60); delay(60); }
  LED(false,false,false);
  return false;
#else
  LED(true,false,false);
  for(int i=0;i<10;i++){ wdtFeed(); beep(880,100); delay(100); }
  LED(false,false,false);
  return true;
#endif
}

bool countdown(){
  constexpr int DURATION=30;
  neutralServos();
  createUniqueLogFile();
  // Pace the FIRST tick to land 1 s after the last arm press, so the five arm chirps and the
  // countdown are one continuous rhythm. Everything above (servo neutral, SD file creation) happens
  // inside this second rather than before it, so a slow card cannot push the first beep late.
  while(armReleaseMs && millis() - armReleaseMs < ARM_TO_FIRST_BEEP_MS){ wdtFeed(); delay(1); }
  const unsigned long cdStart = millis();
  cdStartMs = cdStart; cdAbortLatched = false; cdBtnDownSince = 0;   // arm the abort latch
  for(int i=DURATION;i>0;i--){
    wdtFeed();
#if BOARD_IMPULSE_22
    // Keep the pack lights live THROUGH the countdown. They used to be driven only by the pre-arm
    // gate below, i.e. after all 30 seconds had already elapsed -- so the two indicators that tell
    // you whether to abort stayed dark for the entire window in which aborting is still free.
    sampleRails();
#endif
    pumpGPS();      // keep acquiring through the countdown -- the datum gate may not have been
                    // satisfied during ARM, and these are the last 30 s of stillness it will get
    Serial.println(i);
    // Same tones, same durations, same 1000 ms per tick -- but the waits now sample the button
    // throughout instead of leaving it unread. noTone() is explicit because cdWait() can return
    // early on an abort, and a buzzer left screaming into the abort beeps is a bad cue.
    if(i>5){
      LED(true,false,false); tone(BUZZER,ARM_TONE_TOP_HZ);
      bool ab = cdWait(200);
      noTone(BUZZER); LED(false,false,false);
      if(!ab) ab = cdWait(800);
    }
    else if(i>3){
      LED(true,false,false); tone(BUZZER,ARM_TONE_TOP_HZ);
      cdWait(1000);
      noTone(BUZZER);
    }
    if(countdownAborted(cdStart)){
      LED(false,false,false);
      noTone(BUZZER);
      cdConsumePress();                                    // consume the press (bounded -- see below)
      Serial.print(F("\n*** COUNTDOWN ABORTED by button at T-")); Serial.print(i); Serial.println(F(" ***"));
      neutralServos();
      for(auto&c:pyros){ digitalWrite(c.pin,LOW); c.active=false; }
      LED(true,false,false);
      beep(700,150); beep(500,150); beep(350,350);         // falling three-tone = aborted
      LED(false,false,false);
      return false;                                        // loop() takes this back to ARM
    }
    else if(i==3){ LED(true,false,true); tone(BUZZER,880);
      calibrateGyroBias();
      initial_alt=baroReadAltitude(SEA_LEVEL_HPA);
      Serial.print(F("  baseline alt: ")); Serial.println(initial_alt);
      noTone(BUZZER); LED(false,false,false); }
#if GPS_ENABLE
    // GPS state, once per second, on the console. REPORTED, NEVER GATED: the GPS is passive
    // instrumentation and must not be able to scrub a launch. "datum NO" simply means the horizontal
    // record will be absolute-referenced rather than relative -- the flight is unaffected.
    if(i<=5 || (i%5)==0){
      Serial.print(F("  GPS: fix=")); Serial.print(gpsFixQual);
      Serial.print(F(" sats=")); Serial.print(gpsSats);
      Serial.print(F(" hdop=")); Serial.print(gpsHdop,1);
      Serial.print(F(" datum=")); Serial.print(gpsOriginSet?F("YES"):F("no"));
      Serial.print(F(" nmea=")); Serial.print((int)gpsSentences);
      Serial.print(F(" pps=")); Serial.println((int)gpsPpsCount);
    }
#endif
  }
  // let the loop settle, then LEVEL the attitude quaternion to the pad (accel gravity) and zero the roll log +
  // rate filters + integral accumulators for launch. gyro_x/gyro_y are now DERIVED from the quaternion each
  // sensors() call, so the quaternion -- not a hard-zeroed angle -- carries the initial attitude: launch starts
  // from the TRUE measured pad tilt, not an assumed vertical.
  unsigned long flush=millis(); while(millis()-flush<1000){ wdtFeed(); cdPollButton(); readIMU(); }
  // LAST CHANCE TO SCRUB. This 1 s settle plus the gyro calibration above were the two longest
  // stretches of the countdown with no way to abort, and they sit at T-3 to T-0 -- the moment you
  // are most likely to see something wrong and least able to do anything about it. cdPollButton()
  // adds one digitalRead per iteration and advances no clock, so the settle is unchanged.
  if(countdownAborted(cdStart)){
    noTone(BUZZER);
    cdConsumePress();
    Serial.println(F("\n*** COUNTDOWN ABORTED by button at T-0 (pre-ignition) ***"));
    neutralServos();
    for(auto&c:pyros){ digitalWrite(c.pin,LOW); c.active=false; }
    LED(true,false,false);
    beep(700,150); beep(500,150); beep(350,350);
    LED(false,false,false);
    return false;
  }
  quatFromAccel(); gyro_z=ang_vel_x=ang_vel_y=0; iTermX=iTermY=0; kfInit(kfZ,0); estZ=estVz=0;
  navReset();      // same reason the vertical filter is re-initialised here: the attitude has just been
                   // re-levelled, so any horizontal track integrated against the PREVIOUS quaternion is
                   // referenced to a frame that no longer exists. (navReset also runs at the liftoff edge.)

#if BOARD_IMPULSE_22
  // ---- Impulse 2.2 PRE-ARM GATES. These refuse to ignite; they never merely warn. ----
  sampleRails();
  Serial.print(F("rails: VBAT ")); Serial.print(vbatV,2);
  Serial.print(F(" V  SERVO ")); Serial.print(servoV,2);
  Serial.print(F(" V  PYRO ")); Serial.print(pyroV,2); Serial.println(F(" V"));
  bool railsBad = (vbatV < VBAT_ARM_MIN_V) || (servoV < SERVO_V_MIN_V) || (pyroV < PYRO_V_MIN_V);
  // ALL FOUR channels are reported (with sense volts) by printPyroStatus(); only the CHUTE is GATED.
  // Roles updated 2026-08-15 -- gating is unchanged because the chute is still P4, but the other two
  // names moved and the reasoning below is per ROLE, not per channel number:
  //   PYRO_CHUTE  (P4) GATED     -- an open igniter here means the vehicle is unrecoverable, so
  //                                 there is no reason to leave the pad.
  //   PYRO_LEGS   (P3) reported, NOT gated -- legs are not survival critical and a false abort is
  //                                 worse than a hard landing. Not fitted at all on this flight.
  //   PYRO_ASCENT (P1) reported, NOT gated -- a motor igniter reads open in some wiring schemes, so
  //                                 gating it would scrub good launches.
  //   PYRO_DESCENT(P2) reported, NOT gated -- unused on this flight.
  bool p4Open = !pyroContinuous(3);
  printPyroStatus();
  if(railsBad || p4Open){
    Serial.println(F("ABORT -- pre-arm gate failed (see rails/continuity above). No ignition."));
    if(vbatV < VBAT_ARM_MIN_V) Serial.println(F("   battery below arm minimum -- CHARGE IT. A flat pack"
                                                " stalls a servo and browns out the flight computer."));
    if(p4Open)                 Serial.println(F("   PARACHUTE igniter reads OPEN -- the vehicle would not"
                                                " be recoverable. Check the melt wire and its connector."));
    if(padGateFail()) return false;
  }
#endif

  // ---- SENSOR-RATE GATE. The control loop cannot be faster than the sensor it waits on. ----
  // Found 2026-08-08 once the harness could model a slow bus: at 45 ms per IMU read the countdown's
  // 600-sample gyro calibration takes 27 SECONDS and the vehicle simply never launches, with nothing
  // said. A sick I2C bus must ABORT LOUDLY on the pad, not silently stretch the countdown -- and it
  // must certainly not fly, because the whole control design assumes a ~3.5 ms loop.
  {
    unsigned long t0 = micros();
    const int N = 50;
    for(int i=0;i<N;i++){ wdtFeed(); readIMU(); }
    unsigned long perRead = (micros()-t0)/N;
    Serial.print(F("sensor read: ")); Serial.print((int)perRead); Serial.println(F(" us/sample"));
    if(perRead > SENSOR_READ_MAX_US){
      Serial.print(F("ABORT -- sensor bus too slow (")); Serial.print((int)perRead);
      Serial.print(F(" us > ")); Serial.print((int)SENSOR_READ_MAX_US);
      Serial.println(F(" us). Control loop would be starved. Check I2C wiring/pull-ups."));
      if(padGateFail()) return false;
    }
  }

  // pre-launch sanity: bad tilt (>45) or drift (>10 dps) -> abort, no ignition
  for(int i=0;i<10;i++){ readIMU(); delay(20); }
  // accel-atan2 tilt (gravity ratio) + bias-removed rate drift
  float cx= atan2f(imu_ay,imu_az)*180.0f/M_PI;
  float cy=-atan2f(imu_ax,imu_az)*180.0f/M_PI;
  float cavx=imu_gx, cavy=imu_gy;
  if(fabsf(cx)>45||fabsf(cy)>45||fabsf(cavx)>10||fabsf(cavy)>10){
    Serial.print(F("ABORT -- tilt X=")); Serial.print(cx); Serial.print(F(" Y=")); Serial.print(cy);
    Serial.print(F("  rate avX=")); Serial.print(cavx); Serial.print(F(" avY=")); Serial.println(cavy);
    if(padGateFail()) return false;
  }
  Serial.println(F("LAUNCH"));
  // Persist the two things a post-brownout reboot cannot re-derive: the pad gyro-bias calibration (needs
  // 1.2 s of stillness) and the ground barometric reference. Written HERE, before ignition, so the record
  // is committed while the vehicle is still stationary and an EEPROM stall costs nothing.
  persist.gbx=gyroBiasX; persist.gby=gyroBiasY; persist.gbz=gyroBiasZ;
  persist.groundAlt=initial_alt; persist.boots=0; setPhase(1);
  ignCmdMs=millis();               // stamp the ignition COMMAND time -> the in-flight motor ignition-lag measurement
#if GROUND_TEST
  // THE INHIBIT. Everything above ran exactly as it will on the pad -- gyro calibration, baro
  // baseline, EEPROM commit, the abort window -- so the bench exercises the real sequence. Only the
  // last line is withheld. And because the motor never lights, liftoffConfirmed stays false, so the
  // existing liftoff interlock in loop() then refuses to fire the chute and leg charges as well:
  // the entire pyro bank is inert in this mode without any new code deciding that.
  Serial.println(F("*** GROUND TEST: IGNITION INHIBITED -- ascent motor not fired. ***"));
  Serial.println(F("    The sequence continues so you can watch TVC, logging and recovery run."));
  Serial.println(F("    Set GROUND_TEST 0 at the top of this file before flying."));
  for(int i=0;i<3;i++){ LED(true,false,true); beep(1500,60); LED(false,false,false); delay(60); }
#else
  triggerPyro(PYRO_ASCENT);
#endif
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
  // ---- TORQUE-COMMANDED GAINS ---------------------------------------------------------------
  // The PD law asks for an angular ACCELERATION (deg/s^2); the gimbal angle that delivers it is
  // alpha/keff. So the effective angle gains are wn^2/keff and 2*zeta*wn/keff -- exactly the formula
  // P_GAIN and D_GAIN were computed from by hand, now evaluated against the CURRENT keff instead of
  // a keff the vehicle only has for one instant of one flight.
  // The I term is deliberately NOT divided by keff: it cancels thrust MISALIGNMENT, and a misalignment
  // of mu degrees needs mu degrees of opposing gimbal whatever the thrust is. It is an angle, not a
  // torque, and scaling it would make the trim wander with the burn.
  float pEff, dEff;
  if(TVC_TORQUE_CMD){
    // THE SCHEDULE IS ONE-SIDED, AND THE MEASUREMENT SAYS IT MUST BE.
    // Full normalisation -- holding wn and zeta constant by dividing out every change in keff -- was
    // built first and MEASURED WORSE: paired A/B over 140 dispersed airframes, torque-commanded gave
    // HIGHER boost tilt on 139 of 140 and dropped PASS from 100% to 94.3%.
    // The reason is that zeta = sqrt(keff)*D_GAIN/(2*sqrt(P_GAIN)), so zeta RISES with keff. The 26 N
    // ignition spike therefore hands the loop a faster AND better-damped response for free, exactly
    // when the rail tip-off needs rejecting. Normalising that away spends margin to hit a number.
    // So: schedule only in the direction that RESTORES the design response, never in the direction
    // that surrenders margin the plant is already giving. sched >= 1 boosts the gains when keff falls
    // below nominal -- a weak motor, a heavy build, the tail of the burn -- and is otherwise inert.
    // Loop gain keff*pEff never exceeds the design value, so this cannot climb toward the gain ceiling.
    float sched = constrain(keffNominal/keffEst, 1.0f, GAIN_SCHED_MAX);
    pEff = (TVC_WN*TVC_WN)/keffNominal * sched;
    dEff = (2.0f*TVC_ZETA*TVC_WN)/keffNominal * sched;
  } else {
    pEff = P_GAIN; dEff = D_GAIN;              // legacy: the fixed gains every prior flight used
  }
  float rawX = pEff*gyro_x + iTermX + dEff*ang_vel_x;
  float rawY = pEff*gyro_y + iTermY + dEff*ang_vel_y;
  if(fabsf(rawX) < MAX_TILT){ iTermX += I_GAIN*gyro_x*idt; iTermX = constrain(iTermX,-MAX_TILT,MAX_TILT); }
  if(fabsf(rawY) < MAX_TILT){ iTermY += I_GAIN*gyro_y*idt; iTermY = constrain(iTermY,-MAX_TILT,MAX_TILT); }
  // cmdX/cmdY are the PRE-CLAMP command -- the exact argument constrain() already received (note it uses the
  // JUST-UPDATED iTerm, unlike rawX/rawY above, which use the pre-update value for the anti-windup test). Hoisting
  // it into a local changes nothing numerically -- same expression, same order of operations -- but it makes the
  // pre-clamp value observable, and it evaluates the sum ONCE instead of the 2-3 times the constrain() MACRO
  // textually substituted it. Post-clamp minus pre-clamp is how gimbal saturation is detected offline.
  // --- DEGRADED CONTROL MODES ----------------------------------------------------------------
  // gainScale: halved once on sustained oscillation. P and D are scaled TOGETHER so Kd/Kp is
  // preserved -- the researcher's n=2400 replication showed that moving Kd independently of Kp is
  // what manufactures the failure effect, so a fallback that broke the ratio would be self-defeating.
  // attitudeTrusted=false: the attitude estimate has been caught not tracking reality, so the P and
  // I terms are dropped and only rate damping is commanded. That cannot HOLD attitude (this vehicle
  // has no second attitude source) but it calms a tumble and keeps the gimbal from acting on a lie.
  float kP = pEff*gainScale, kD = dEff*gainScale;
  float cmdX, cmdY;
  if(attitudeTrusted){
    cmdX = kP*gyro_x + iTermX + kD*ang_vel_x;
    cmdY = kP*gyro_y + iTermY + kD*ang_vel_y;
  } else {
    cmdX = kD*ang_vel_x;                          // rate damping only -- proven, less capable, honest
    cmdY = kD*ang_vel_y;
  }
  // --- OSCILLATION DETECTOR -------------------------------------------------------------------
  // Ringing shows up as the command reversing sign repeatedly at large amplitude. On ASC038 the
  // command alternated between the +-5 deg stops because the D term was acting alone on huge rates.
  // Requires BOTH high reversal count AND large amplitude, so ordinary small corrections never trip it.
  {
    static bool  lastSign = false;
    static int   flips = 0;
    static unsigned long winStart = 0;
    unsigned long now = millis();
    if(winStart == 0) winStart = now;
    bool sign = (cmdX >= 0);
    if(sign != lastSign && fabsf(cmdX) > OSC_AMP_FRAC*MAX_TILT) flips++;
    lastSign = sign;
    if(now - winStart >= OSC_WINDOW_MS){
      if(flips >= OSC_FLIPS){
        if(gainScale > 0.9f){                     // first response: back the gains off, keep flying
          gainScale = 0.5f;
          raiseFault(F_OSCILLATION|F_GAIN_BACKOFF, HEALTH_REDUCED);
        } else if(attitudeTrusted){               // still ringing at half gain: stop trusting attitude
          attitudeTrusted = false;
          raiseFault(F_OSCILLATION, HEALTH_FALLBACK);
        }
      }
      flips = 0; winStart = now;
    }
  }
  // --- CONTROL-SIGN CHECK (see the detector notes at SIGN_MIN_CMD_DEG) -------------------------
  // Restoring control satisfies cmd * d(rate)/dt < 0. Judged only under thrust, only when the loop is
  // commanding meaningfully, and only on sustained evidence.
  {
    static float prevAvX = 0; static unsigned long prevUs = 0;
    unsigned long nowUs = micros();
    // ONLY judge while the gimbal is UNSATURATED. This is the difference between a working detector
    // and a dangerous one: when the controller is pinned at its limit and losing to a disturbance,
    // command and angular acceleration share a sign for a completely innocent reason -- the correction
    // is right and merely too small. A first version without this gate produced 24 false positives in
    // 120 healthy flights, i.e. it would have neutralised the gimbal on a healthy vehicle in wind.
    if(prevUs && poweredFlight && fabsf(cmdX) > SIGN_MIN_CMD_DEG && fabsf(cmdX) < MAX_TILT*0.95f){
      float ddt = (nowUs - prevUs) * 1e-6f;
      if(ddt > 1e-4f && ddt < 0.1f){
        float angAccel = (ang_vel_x - prevAvX) / ddt;      // deg/s^2
        if(fabsf(angAccel) > 5.0f){                        // ignore near-zero accel: no information in it
          if(cmdX * angAccel > 0) signBad++; else signGood++;
        }
      }
    }
    prevAvX = ang_vel_x; prevUs = nowUs;
    // *** DIAGNOSTIC ONLY -- THIS DOES NOT ACT. TESTED AND REJECTED AS AN ABORT, 2026-08-08. ***
    // The idea was sound (restoring control requires cmd * angular-accel < 0) but it cannot separate
    // "my correction is backwards" from "my correction is correct and too small". Measured in the SIL:
    //   - without a saturation gate: 24 FALSE POSITIVES in 120 healthy flights. Correctly-wired
    //     vehicles losing to a gust reach bad_frac = 1.000, because a saturated controller that is
    //     being overpowered also shows cmd and accel sharing a sign.
    //   - with an unsaturated-only gate: 0 false positives, but also 0 TRUE positives -- inverted
    //     wiring diverges so fast that the command saturates before enough clean samples exist.
    // Both settings are useless, in opposite directions. Wiring sense is therefore NOT detectable in
    // flight on this vehicle, and THE BENCH PITCH-THE-NOSE TEST REMAINS THE ONLY TRUSTWORTHY CHECK.
    // It must never be skipped. Shipping this as an abort would have manufactured exactly the false
    // confidence that cost ASC038.
    // The counters are still logged: post-flight, sign_bad_frac read TOGETHER WITH saturated_frac is
    // informative -- high bad_frac with LOW saturation points at wiring, high bad_frac with HIGH
    // saturation points at an authority shortfall. That distinction is cheap to keep and costs nothing.
  }
  // controlInhibited is the hard stop: the gimbal is commanded to true neutral and STAYS there.
  // gainScale cannot do this job -- it only scales kP/kD, and iTermX/iTermY bypass it entirely, so
  // gainScale=0 would still leave the integral trim driving the nozzle. (It is also already applied
  // to kP/kD above; applying it again here would square it.)
  if(controlInhibited){ cmdX = 0; cmdY = 0; iTermX = 0; iTermY = 0; }
  tiltX = constrain(cmdX, -MAX_TILT, MAX_TILT) * SERVO_X_MULT;
  tiltY = constrain(cmdY, -MAX_TILT, MAX_TILT) * SERVO_Y_MULT;
  writeServos(tiltX, tiltY);
  captureControl(cmdX, cmdY);      // AFTER the servo write: dt_us then spans command-to-command = a true control period
}

// -- Setup ---------------------------------------------------------------------
void setup(){
  pinMode(BUTTON,INPUT); pinMode(BUZZER,OUTPUT);
  pinMode(RLED,OUTPUT); pinMode(GLED,OUTPUT); pinMode(BLED,OUTPUT);
  pinMode(PIN_MAIN_LED,OUTPUT); digitalWrite(PIN_MAIN_LED,LOW);   // active HIGH, start dark
  pinMode(PIN_PYRO_LED,OUTPUT); digitalWrite(PIN_PYRO_LED,LOW);
  // (the pack-LED self test is NON-BLOCKING and lives in updateBatteryLeds() -- see the note there)
  // All four driven LOW before anything else can run. Order is physical (P1..P4); the roles are
  // PYRO_ASCENT / PYRO_DESCENT / PYRO_LEGS / PYRO_CHUTE respectively.
  pinMode(P1,OUTPUT); pinMode(P2,OUTPUT); pinMode(P3,OUTPUT); pinMode(P4,OUTPUT);
  digitalWrite(P1,LOW); digitalWrite(P2,LOW); digitalWrite(P3,LOW); digitalWrite(P4,LOW);
#if BOARD_IMPULSE_22
  pinMode(PIN_SENS_DET, INPUT);
  analogReadResolution(ADC_BITS);     // 12-bit for the rail/continuity dividers
#endif
  servoX.attach(SERVO_X_PIN); servoY.attach(SERVO_Y_PIN);
#if CTL_SERVO_FEEDBACK
  analogReadResolution(12);        // match Bench_Latency.ino / feedback_servo_calibration.ino (A2/A3, 12-bit) so
  analogReadAveraging(1);          // in-flight feedback counts are directly comparable to the bench calibration
#endif
  neutralServos();
  LED(false,false,false);
  Serial.begin(115200);
  // Wait for the USB monitor, but only briefly. `Serial` is true the instant the host has the port
  // open, so with a monitor already running this costs ZERO -- the timeout is only ever paid when
  // there is no monitor, i.e. ON THE PAD, every single boot. It was 3000 ms, which bought nothing
  // there but three seconds of a dead-looking board. If you want the boot prints on the bench, open
  // the monitor and tap the Teensy reset button.
  while(!Serial && millis()<250);
#if GROUND_TEST
  // Announced at EVERY boot, before anything else can scroll it away, and with a sound as well as
  // text -- on the pad there is no laptop, and "I forgot which build is flashed" is precisely the
  // failure this flag has to be unable to cause.
  Serial.println(F("\n*********************************************************"));
  Serial.println(F("*  GROUND TEST BUILD -- THIS FIRMWARE CANNOT LAUNCH.     *"));
  Serial.println(F("*  Pre-arm gates are BYPASSED (warnings only) and the    *"));
  Serial.println(F("*  the ascent igniter is HARD-INHIBITED. No liftoff means *"));
  Serial.println(F("*  the chute and leg charges are interlocked off too.    *"));
  Serial.println(F("*  Set GROUND_TEST 0 to fly. regression.py will refuse   *"));
  Serial.println(F("*  to run until you do.                                  *"));
  Serial.println(F("*********************************************************\n"));
  for(int i=0;i<3;i++){ LED(true,false,true); beep(1500,70); LED(false,false,false); delay(70); }
#endif
  Wire.begin();
  // FAILURE_MODES D2 said a stuck bus wedges the control loop forever and that Teensy's timeout API was
  // unused. Checked against the library source (WireIMXRT.cpp, Teensyduino 1.59): NOT SO -- the driver
  // already bounds every transaction. CLOCK_STRETCH_TIMEOUT is 15 ms in hardware (MCFGR3 PINLOW), and
  // the transfer loops bail at 50 ms, returning 4 = "clock stretched too long or generic timeout". So a
  // latched bus costs ~50 ms per read, not infinity. There is no setWireTimeout() on this core to call.
  // 50 ms is still 10x a control period, which is what the dt clamp and the loop-health governor exist
  // for. The real defect was in OUR code: imuReadSource() discarded getEvent()'s return value, so a
  // timed-out read was published as a live sample. That is fixed there, not here.
  // Sensor-init failure used to be a silent while(true) -- on the pad with no laptop that is indistinguishable
  // from a dead board. Now it is ANNOUNCED (red LED + buzzer) and the button RETRIES via a restart. The button
  // deliberately does NOT continue past a failed init: flying without an IMU or baro is not a survivable option.
  // SPI1 for the sensor board. Explicit pin calls even though these are the Teensy 4.1 defaults --
  // this is the bus the primary IMU and the primary baro both live on, and it should be obvious.
  SPI1.setMOSI(26); SPI1.setMISO(1); SPI1.setSCK(27);
  pinMode(PIN_CS_IMU, OUTPUT);  digitalWrite(PIN_CS_IMU, HIGH);
  pinMode(PIN_CS_BARO,OUTPUT);  digitalWrite(PIN_CS_BARO,HIGH);
  SPI1.begin();

  // BARO: DPS310 primary, BMP280 backup. This used to halt outright if the BMP280 was missing --
  // wrong now that the BMP is the redundant one and is frequently not fitted (it was not fitted at
  // all during the 2026-08-12 bring-up). Halt only if BOTH are gone, since altitude drives apogee
  // detection and the chute.
  dpsPresent = dps310Begin();
  baroBmpPresent = bmp.begin(0x76) || bmp.begin(0x77);
  if(baroBmpPresent)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_1);
  Serial.print(F("BARO: DPS310 ")); Serial.print(dpsPresent?F("ok"):F("absent"));
  Serial.print(F("   BMP280 "));    Serial.print(baroBmpPresent?F("ok"):F("absent"));
  Serial.print(F("  -> using "));   Serial.println(dpsPresent?F("DPS310"):(baroBmpPresent?F("BMP280"):F("NOTHING")));
  if(!dpsPresent && !baroBmpPresent){ Serial.println(F("NO BAROMETER -- refusing to arm.")); sensorFaultHalt(2); }

  // IMU primary. Failure here is not fatal: imuReadSource() reports it and the failover picks the
  // backup, exactly as before the driver existed.
  bool icmOk = icm42688Begin();
  Serial.print(F("ICM-42688-P: ")); Serial.print(icmOk?F("ok"):F("NOT FOUND"));
  Serial.print(F("   UI filter cfg (0x52) = 0x")); Serial.print((int)icmUiFiltReadback);
  Serial.println(ICM_UI_FILT_CFG==0xFF ? F("  [power-on default, not written]") : F("  [written by firmware]"));

#if GPS_ENABLE
  // GPS. Serial4 = RX4 on pin 16 per the Impulse 2.2 netlist -- NOT Serial1, whose pins 0/1 are
  // CS_IMU and MISO1 on this board. Purely passive instrumentation: a GPS that never answers costs
  // nothing but empty columns, so there is no gate, no halt and no fault raised here.
  pinMode(PIN_GPS_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), gpsPpsISR, RISING);
  GPS_SERIAL.begin(GPS_BAUD);
  // Enlarge the RX ring. The stock 64-byte buffer overflows after ~66 ms of not being drained at
  // 9600 baud, and there are blocking stretches on the pad (gyro calibration, the countdown's
  // beeps, SD dumps) far longer than that. 512 B covers ~530 ms, so a full 1 Hz NMEA burst survives
  // any of them and the parser sees whole sentences rather than shredded ones.
  static uint8_t gpsRxBuf[512];
  GPS_SERIAL.addMemoryForRead(gpsRxBuf, sizeof(gpsRxBuf));
  navReset();
  Serial.print(F("GPS: Serial4 @ ")); Serial.print((int)GPS_BAUD);
  Serial.println(F(" baud, PPS on pin 20 (PASSIVE -- logged, never used for control)"));
#endif

  if(!mpu.begin()){
    // No longer fatal on its own -- the MPU is the BACKUP now. Only "no inertial source at all",
    // checked further down, halts.
    Serial.println(F("MPU6050 not found (backup absent -- single inertial source this flight)"));
  } else {
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);   // ASC007 railed at +/-2 g; +/-16 g clears the ~3.2 g axial peak
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);        // headroom for roll/tumble; control rates stay well under 2000 dps
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);      // DLPF: reject motor vibration without lagging the loop
  Serial.print(F("MPU6050 range (want 3/3 = 16g/2000dps): acc=")); Serial.print((int)mpu.getAccelerometerRange());
  Serial.print(F(" gyro=")); Serial.println((int)mpu.getGyroRange());
  }
  SD.begin(SD_CS);

  // WHY the last reset happened. This is the diagnostic ASC036 needed and did not have: its log stopped
  // mid-flight and nothing recorded whether the board browned out, was watchdog-reset, or kept running
  // while only the SD writes failed. Bit meanings are in imxrt.h (SRC_SRSR_*); the raw word is logged so
  // it can be decoded offline. Read-and-clear: the register is sticky across resets until written back.
#if defined(__IMXRT1062__)
  resetCause = SRC_SRSR; SRC_SRSR = resetCause;
#else
  // SIL: there is no SRC_SRSR off-target, so the harness injects the word. Without this the whole
  // brownout branch below was unreachable in simulation (resetCause stayed 0 forever) and the
  // "*** BROWNOUT: power was lost MID-FLIGHT ***" discriminator had never once been evaluated.
  { extern uint32_t g_resetCause; resetCause = g_resetCause; }
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

  // ---- PLANT MODEL: derive the wet state and the nominal design point ----------------------
  // Printed because DRY_* are back-derived from a bifilar swing and PROP_ARM_M is a ruler measurement.
  // If the WET numbers below do not match what was actually weighed and swung, the model is wrong and
  // every scheduled gain is wrong with it -- so it goes on the console where it cannot be missed.
  burnFrac = 0; impulseNs = 0; thrustScale = 1.0f;
  plantUpdate(0.0f);                                  // dt=0: mass properties only, no integration
  keffNominal = (MOTOR_IMPULSE_NS/BURN_TIME) * plantL / plantIyy;   // at AVERAGE thrust, full grain
  keffEst = keffNominal;
  Serial.print(F("plant WET (full grain): m=")); Serial.print(plantMass,3);
  Serial.print(F(" kg  L=")); Serial.print(plantL,4);
  Serial.print(F(" m  Iyy=")); Serial.print(plantIyy,5);
  Serial.print(F("  keff@avgT=")); Serial.println(keffNominal,1);
  Serial.print(F("plant DRY (spent):      m=")); Serial.print(DRY_MASS_KG,3);
  Serial.print(F(" kg  L=")); Serial.print(DRY_L_M,4);
  Serial.print(F(" m  Iyy=")); Serial.println(DRY_IYY,5);
  Serial.print(F("TVC: ")); Serial.print(TVC_TORQUE_CMD ? F("TORQUE-COMMANDED  wn=") : F("FIXED-GAIN  wn="));
  Serial.print(TVC_WN,2); Serial.print(F(" rad/s  zeta=")); Serial.print(TVC_ZETA,2);
  Serial.print(F("  -> P=")); Serial.print((TVC_WN*TVC_WN)/keffNominal,4);
  Serial.print(F(" D=")); Serial.println((2.0f*TVC_ZETA*TVC_WN)/keffNominal,4);

  loadPersist();
  Serial.print(F("persist: phase=")); Serial.print((int)persist.phase);
  Serial.print(F(" boots=")); Serial.println((int)persist.boots);
  // ARM THE WATCHDOG EARLY, BUT ONLY WHEN WE REBOOTED IN FLIGHT.
  // It is normally started at the very END of setup() so a slow sensor probe can never trip it, and on
  // the pad that is right -- there is a human present and an indefinite button wait ahead. In flight it
  // is wrong: measured with --firmwarehang, the board took a watchdog reset, came back up, hit the same
  // fault inside the IMU probe below (which runs BEFORE the old wdtStart()) and hung there unprotected
  // until impact, chute still packed. Anything that can hang setup() twice will hang it every time, so
  // the reboot needs a watchdog covering init itself. Gated on phase!=0 so pad behaviour is unchanged,
  // and RESUME_MAX_BOOTS still caps how many times this may cycle.
  if(persist.phase != 0){
    wdtStart();
    Serial.println(F("watchdog armed EARLY (in-flight reboot: init is not a safe place to hang)"));
  }
  // THE brownout discriminator: a power-on reset is normal on the bench, but a power-on reset while the
  // saved phase says we were flying means the rail collapsed in flight. Printed loudly because this is the
  // open ASC036 question (its log stopped mid-flight with no recorded reason).
  if(persist.phase!=0 && (resetCause & (1<<0)))
    Serial.println(F("*** BROWNOUT: power was lost MID-FLIGHT (POR with an in-flight saved phase) ***"));
  // Pick the starting inertial source. SENS_DET tells us whether the sensor-board ribbon is even
  // plugged in -- if it is not, there is no point trying the SPI primary and failing over on the pad.
  {
    bool haveBoard = false;
#if BOARD_IMPULSE_22 && SENSOR_BOARD_ENABLE
    haveBoard = sensorCablePresent();
#endif
    float gx,gy,gz,ax,ay,az;
    imuOk[SRC_PRIMARY] = haveBoard && imuReadSource(SRC_PRIMARY,gx,gy,gz,ax,ay,az);
    imuOk[SRC_BACKUP]  = imuReadSource(SRC_BACKUP ,gx,gy,gz,ax,ay,az);
    imuActive = imuOk[SRC_PRIMARY] ? SRC_PRIMARY : SRC_BACKUP;
    Serial.print(F("IMU: sensor board ")); Serial.print(haveBoard?F("present"):F("ABSENT"));
    Serial.print(F("  primary ")); Serial.print(imuOk[SRC_PRIMARY]?F("ok"):F("unavailable"));
    Serial.print(F("  backup "));  Serial.print(imuOk[SRC_BACKUP] ?F("ok"):F("FAILED"));
    Serial.print(F("  -> flying on ")); Serial.println(SRC_NAME[imuActive]);
    // The one thing that will silently destroy this vehicle. SERVO_*_SIGN was verified against the
    // MPU-6050's axes on 2026-08-03; the ICM is a different part on a different board in a different
    // orientation, so that verification does not transfer. Flying the primary with the wrong sign is
    // not degraded control, it is a positive feedback loop.
    if(imuActive==SRC_PRIMARY && !ICM_AXES_VERIFIED){
      Serial.println(F("\n*********************************************************"));
      Serial.println(F("*  ICM-42688-P AXIS SIGNS ARE NOT BENCH-VERIFIED.       *"));
      Serial.println(F("*  Control signs were set against the MPU-6050 and do   *"));
      Serial.println(F("*  NOT carry over. DO NOT FLY until the pitch-the-nose  *"));
      Serial.println(F("*  check is re-run and ICM_AXES_VERIFIED is set to 1.   *"));
      Serial.println(F("*********************************************************\n"));
      for(int i=0;i<3;i++){ LED(true,false,false); beep(400,120); LED(false,false,false); delay(120); }
    }
    // Only a FAULT if the board is physically present and its IMU nevertheless failed. Flying with
    // no sensor board fitted is a deliberate configuration (and is the state until the ICM-42688
    // driver is written and bench-verified), not a malfunction -- calling it one every flight would
    // dilute the health record until nobody reads it.
    if(haveBoard && !imuOk[SRC_PRIMARY] && imuOk[SRC_BACKUP])
      raiseFault(F_IMU_FAILOVER, HEALTH_REDUCED);
    else if(!haveBoard)
      Serial.println(F("   (no sensor board fitted -- single inertial source, NO redundancy this flight)"));
    if(!imuOk[SRC_BACKUP] && !imuOk[SRC_PRIMARY]){
      Serial.println(F("NO INERTIAL SOURCE -- refusing to arm."));
      sensorFaultHalt(3);
    }
  }
  if(resetCause & (1<<4)) raiseFault(F_WDT_RESET, HEALTH_REDUCED);   // came up from a watchdog reset
  resumeAfterReset();     // if we rebooted mid-flight AND the sensors agree we are flying: protect the chute.
                          // Returns immediately in every normal power-on. Never fires a motor, never resumes TVC.
  // Watchdog LAST, after every blocking init is done, so a slow sensor probe can never trip it.
  // From here on the control loop and every long-running loop feed it; a genuine hang resets the
  // board, and the EEPROM record + resumeAfterReset() above then get the parachute out.
  wdtStart();
  Serial.print(F("watchdog: ")); Serial.print(WATCHDOG_ENABLE ? WATCHDOG_SECONDS : 0);
  Serial.println(F(" s (0 = disabled)"));
}

// -- Main loop -----------------------------------------------------------------
void loop(){
  // ARM. A brief white flash = "computer is up and in ARM", which is the one thing you cannot tell
  // from a dark board on a pad with no laptop. It was 500 ms; 120 ms is just as visible and this
  // runs every time the vehicle returns to ARM, not once per power-up.
  LED(true,true,true); delay(120); LED(false,false,false);
  // ARM accepts two gestures: 5 rapid presses -> countdown, or a 2 s HOLD -> bench TVC.
  // This counts presses itself rather than calling buttonCount(), because buttonCount() blocks in
  // its own wait-for-release loop from the second press onward and would swallow the hold before it
  // ever reached the threshold. Same window and same press count, so arming feels identical.
  // ARM: 5 rapid presses -> countdown, or a 2 s HOLD -> bench TVC.
  //
  // NON-BLOCKING BY CONSTRUCTION. The previous version drove the LED and buzzer as side effects from
  // inside a blocking armGesture(), so any path that returned early left them latched on -- the
  // stuck-buzzer-after-a-cancelled-hold bug. Here the button is sampled ONCE per iteration, all
  // decisions come from timestamps, and the outputs are re-asserted UNCONDITIONALLY at the bottom of
  // every pass. "Stuck on" is no longer a reachable state rather than a bug that was patched out.
  {
    constexpr uint32_t ARM_WINDOW_MS = 300, ARM_DEBOUNCE_MS = 15, PRESS_CHIRP_MS = 90;
    constexpr uint32_t ARM_TAP_MAX_MS = 300;   // longer than this = an attempted hold, not a tap
    constexpr int      ARM_PRESSES_REQD = 5;
    int      presses = 0;
    uint32_t lastRelease = 0, pressStart = 0, lastEdge = 0;
    bool     down = false, holdFired = false, armed = false, goBench = false;

    while(!armed && !goBench){
      wdtFeed();
#if BOARD_IMPULSE_22
      sampleRails();               // pack lights live while waiting -- the longest-lived pad state
#endif
      // Live continuity while you sit at ARM, so you can plug, unplug and re-crimp a channel and
      // watch it change. This is the whole bench workflow for the sense circuit, and it needs no
      // pyro to be energised -- see printPyroStatus().
      { static uint32_t lastPyroPrint = 0;
        if(millis() - lastPyroPrint >= 2000){ lastPyroPrint = millis(); printPyroStatus(); } }
      pumpGPS();                   // ARM is where the receiver actually acquires: it is the longest
                                   // time the vehicle sits still, and the datum gate needs a
                                   // converged fix (6 sats, HDOP <= 2.5) BEFORE liftoff or the whole
                                   // horizontal record is referenced to a bad origin.
      const uint32_t now = millis();
      const bool raw = (digitalRead(BUTTON) == HIGH);

      if(raw != down && now - lastEdge >= ARM_DEBOUNCE_MS){
        lastEdge = now;
        down = raw;
        if(down){ pressStart = now; holdFired = false; }
        else if(!holdFired && now - pressStart < ARM_TAP_MAX_MS){        // a TAP, not a released hold
          // Anything longer than a tap was an attempted hold. Releasing it cancels; it must not also
          // register as an arm press, which is what made backing out of bench mode start arming.
          presses = (now - lastRelease <= ARM_WINDOW_MS) ? presses+1 : 1;
          lastRelease = now;
          Serial.print(F("arm press ")); Serial.print(presses);
          Serial.print('/'); Serial.println(ARM_PRESSES_REQD);
          if(presses >= ARM_PRESSES_REQD) armed = true;
        }
      }

      if(down && !holdFired && now - pressStart >= BENCH_HOLD_MS){ holdFired = true; goBench = true; }

      // ---- OUTPUTS: recomputed from scratch and re-asserted every pass ----
      bool r=false,g=false,b=false; int hz=0;
      if(down && now - pressStart > 300){                    // holding: rising tone, quickening flash
        const uint32_t held = now - pressStart;
        hz = 300 + (int)(1100.0f * held / BENCH_HOLD_MS);
        uint32_t period = 400 - (300*held)/BENCH_HOLD_MS; if(period < 60) period = 60;
        b = ((held / period) % 2) == 0;
      } else if(presses > 0 && now - lastRelease < PRESS_CHIRP_MS){     // press acknowledgement
        g  = true;
        hz = (int)(ARM_TONE_TOP_HZ * powf(2.0f, (presses - ARM_PRESSES_REQD)/12.0f));
      }
      LED(r,g,b);
      if(hz) tone(BUZZER,(unsigned int)hz); else noTone(BUZZER);
      delay(5);
    }
    if(goBench){
      noTone(BUZZER); LED(false,false,false);
      while(digitalRead(BUTTON)==HIGH) wdtFeed();            // consume the rest of the hold
      beep(1200,90); beep(1600,160);
      benchTVC();
      return;
    }
    // ARMED. Let the fifth press's chirp run PRESS_CHIRP_MS and no longer, so all five presses are
    // identical. `armed` is set inside the edge handler above, so without this the while() condition
    // ends the loop on the next 5 ms pass and the last press is 18x SHORTER than the four before it.
    while(millis() - lastRelease < PRESS_CHIRP_MS){ wdtFeed(); delay(1); }
    armReleaseMs = lastRelease;   // countdown() paces its FIRST beep off this -- see ARM_TO_FIRST_BEEP_MS
  }
  noTone(BUZZER); LED(false,false,false);
  // Straight into the countdown. There used to be a bare delay(500) here doing nothing: the servos
  // are already neutral and countdown() re-neutralises them anyway, so it was pure dead air between
  // the last press and the first tick.
  if(!countdown()){
    // countdown() fails for two very different reasons, and they do not deserve the same response.
    //
    // YOU scrubbed it (cdAbortLatched -- the button). Go straight back to ARM. The press has already
    // been consumed by cdConsumePress(), the falling three-tone has already told you it worked, and
    // the servos and pyros are already safe. Demanding a SECOND press to clear a scrub you just
    // performed yourself communicates nothing you do not already know.
    //
    // THE FIRMWARE refused to arm (flat pack, open chute igniter, sick sensor bus, vehicle leaning
    // on the pad). Nobody asked for that, and each of those paths has already sounded a 2 s alarm.
    // It must NOT be possible to wander off from a flat battery or a dead parachute igniter because
    // the board quietly went back to ARM on its own -- so hold solid red until a human acknowledges.
    // exitOnButton() (not a bare digitalRead) so that press is CONSUMED and cannot be miscounted as
    // arming press #1.
    if(cdAbortLatched){
      LED(false,false,false);
      return;                              // straight back to ARM
    }
    LED(true,false,false);
    while(!exitOnButton()){ wdtFeed(); delay(50); }
    LED(false,false,false);
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
        // FLUSH THE FLIGHT LOG TO SD NOW. Rows are buffered in RAM during boost so the control loop
        // never touches the card (the ASC038 fix) -- but that means a reset before recovery loses the
        // whole flight. Burnout is the first moment control no longer matters, so the boost data goes
        // to the card here, ~1 s before deploy. If a brownout or watchdog reset happens after this
        // point the boost record survives; if it happens during boost it is still lost, which is the
        // deliberate price of never stalling the loop while the motor is lit.
        dumpFlightLog();
      }
      centerServosRamp();                  // ease to neutral over ~1 s, then hold neutral (see the note at its definition)
      sensors();
    }
    // apogee: baro fell APOGEE_DROP_M below the peak AND descending -- the vert_vel<0 guard stops a boost baro
    // glitch from deploying the chute during powered ascent. OR the hard time backup (baro-failure safety).
    if(altitude < highest_alt - APOGEE_DROP_M && vert_vel < 0.0f) apogee=true;
    // PRIMARY DEPLOY TIMER: fires DEPLOY_AFTER_TVC_S after TVC ends, regardless of the barometer.
    // Whichever trigger comes first wins, so this can only ever make deployment EARLIER, never later.
    // On ASC038 the baro/velocity path was 0.87 s late and the canopy did not appear until ~1.5 m;
    // this bounds that delay with something that cannot be starved or filtered.
    // THRUST INHIBIT on the timed deploy. The timer fires at a FIXED time from the ignition command,
    // but the ignition lag is variable and has measured as high as 920 ms -- so on a very late light
    // the motor could still be burning when the timer expires, and deploying a canopy into a live
    // motor destroys it. The barometric trigger never had this problem (it needs descent); the timer
    // I added does, so it must carry the same interlock the brownout-resume path already uses.
    bool thrusting = (imu_az > THRUST_ONSET_G*G);
    if(!thrusting && sinceLaunch >= tvcEndMs + (unsigned long)(DEPLOY_AFTER_TVC_S*1000.0f)){
      apogee=true; Serial.println(F("DEPLOY (TVC-stop timer)"));
    }
    // HEALTH_FALLBACK or worse means the attitude estimate is not trusted and the vehicle is only
    // being rate-damped. It cannot hold attitude, so there is nothing to gain by staying in powered
    // flight -- end the boost phase and get to recovery as soon as thrust is gone. This is the
    // "degrade, do not crash" response: a short flight under canopy beats a tumble.
    if(healthLevel >= HEALTH_FALLBACK && !poweredFlight){
      apogee=true; Serial.println(F("DEPLOY (health fallback -- attitude not trusted)"));
    }
    if(millis()-launchTime >= backupMs){ apogee=true; Serial.println(F("APOGEE (time backup)")); }
  }
  poweredFlight=false; inFlight=false;

  // RECOVERY -- PYROS FIRST. The three beeps used to run BEFORE this, and beep() is a blocking
  // delay(): 300 ms of dead time while the vehicle fell ~3 m. On ASC038 the deploy chain was already
  // 0.87 s late from the velocity filter, so every avoidable millisecond mattered -- the canopy did
  // not appear until ~1.5 m above the ground. Nothing may sit between apogee detection and the pyro.
  neutralServos();
  LED(false,true,true);
  // *** LIFTOFF INTERLOCK -- PEOPLE SAFETY, NOT VEHICLE SAFETY. ***
  // If the vehicle never left the pad (motor never lit, or lit and failed to lift), firing the
  // parachute and leg charges at ground level throws pyro next to whoever is standing there. The
  // 2026-07-20 session has a recorded no-ignition run where this whole sequence executed on the pad.
  // No liftoff evidence -> NO PYROS, loud alarm, hold until a human intervenes.
  if(!liftoffConfirmed){
    raiseFault(F_NO_LIFTOFF, HEALTH_SURVIVE);
    Serial.println(F("*** NO LIFTOFF DETECTED -- PYROS INHIBITED. Vehicle is on the pad. ***"));
    Serial.println(F("    Approach only after disarming and disconnecting the battery."));
    dumpFlightLog(); dumpMotorLog(); dumpControlLog();
    LED(true,false,false);
    while(!exitOnButton()){ wdtFeed(); updatePyros(); beep(880,120); delay(120); beep(660,120); delay(400); }
    setPhase(0); persist.boots=0; savePersist();
    LED(false,false,false);
    return;                                // back to ARM without ever energising a pyro channel
  }
  triggerPyro(PYRO_CHUTE);                 // PARACHUTE (primary recovery)
  triggerPyro(PYRO_LEGS);                  // LANDING LEGS -- deploy now so touchdown loads the legs, not the TVC
  setPhase(3);                             // chute is out: a later reboot must NOT re-run the resume watchdog
  beep(659,100); beep(523,100); beep(659,100);   // moved AFTER the pyros: audible cue, costs no altitude
  dumpFlightLog();                         // write the RAM-buffered 20 Hz flight log -> ASC###.CSV
  dumpMotorLog();                          // write the flown motor's thrust curve + ignition lag -> MTR###.CSV
  dumpControlLog();                        // write the high-rate control trace + loop-period stats -> CTL###.CSV

  // RECOVERED -- locator beep until the button is pressed, then back to ARM (no power cycle needed)
  LED(true,true,true);
  Serial.println(F("RECOVERED"));
  while(!exitOnButton()){ wdtFeed(); beep(523,600); beep(392,600); updatePyros(); }
  setPhase(0); persist.boots=0; savePersist();   // flight over: clear the record so it can never arm a resume later
  LED(false,false,false);
  Serial.println(F("RECOVERED cleared by button -- returning to ARM"));
  return;      // loop() is re-entered by the core = ARM state. exitOnButton() has already disarmed every
               // pyro, and re-arming still needs 5 rapid presses + the 30 s countdown before ignition.
}



