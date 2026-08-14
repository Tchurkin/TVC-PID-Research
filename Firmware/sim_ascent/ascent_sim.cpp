/*
  ascent_sim.cpp â€” Software-In-the-Loop harness for the EXACT Ascent_TVC.ino flight code.
  =======================================================================================
  Runs setup()/loop() unmodified against a ROLL-AWARE 6DOF attitude + vertical-trajectory
  physics model (quaternion attitude; body rates X=pitch, Y=yaw, Z=ROLL).

  WHY THE REWRITE (2026-07-07): the previous harness modeled only two DECOUPLED tilt
  integrators (thX, thY) with gyroZ hardcoded 0 -- no roll axis, no gyroscopic coupling, no
  roll-torque source. That is exactly the failure mode that killed flight ASC007: an
  uncontrolled roll built to ~113 dps / ~172 deg, and once the roll was large AND the +/-5 deg
  TVC was saturated holding a steady lean, the pitch/yaw diverged (gyroscopic p-q-r coupling
  the firmware's decoupled PD ignores) and the vehicle aborted (chute) at ~52 deg tilt. The old
  SIL was structurally blind to it. This harness adds:
    - full quaternion attitude + Euler rigid-body eqs WITH the (Iyy-Izz)*w*w gyroscopic terms,
    - roll inertia Izz (~17x smaller than Iyy) so small roll torques wind up fast,
    - roll-torque SOURCES: nozzle tangential cant (--rollcant), CG lateral offset (--cgoffx/y)
      which ALSO trims pitch/yaw AND leaks every TVC correction into roll, aero fin roll (--finmis),
    - the firmware's NAIVE gyro integration is fed the true body rates, so its tilt estimate
      diverges from truth under roll exactly as the real code does.
  A good build (no roll source) still flies straight -> PASS: the control torque is directly
  restoring, so the bench-verified firmware stabilizes. Dispersions via argv = Monte-Carlo rig.
  Build: build_zig.bat   (nominal: ascent_sim.exe   MC: python ascent_mc.py 300)
*/
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <csetjmp>

// ---- Arduino core globals (declared extern in Arduino.h) ----
uint64_t g_micros = 0;
int      g_pin[64] = {0};
int      g_servoDeg[64];
double   g_imu_gyroX=0, g_imu_gyroY=0, g_imu_gyroZ=0;
double   g_imu_accX=0, g_imu_accY=0, g_imu_accZ=1.0;
double   g_baro_alt = 0;
// SD write latency, charged to the SIMULATED clock by shims/SD.h. Zero reproduces the OLD, blind
// harness. ASC038 measured a mean 143 ms per logData() open+write+close on the flight card, which
// starved the control loop past the dt<0.1f guard in sensors() and froze the attitude estimator.
// With this at 0 that failure was structurally impossible to simulate. Set via --sdlatency.
double g_sdOpenLatencyMs=0, g_sdCloseLatencyMs=0, g_sdTotalStallMs=0;
long   g_sdOpenCount=0;
bool   g_sdStallEnabled=true;
// ---- LOOP TIMING MODEL (2026-08-08) ---------------------------------------------------------
// The IMU shim charges these to the simulated clock on every control iteration. Defaults reproduce
// the historical fixed 5 ms loop so old results stay comparable; --loopjitter/--imuslow make the
// loop period VARIABLE, which is what the loop-health governor, the dt clamp and the dt-guard
// logic all key off. Without this the harness had a constant loop period and could not express
// ASC038's failure at all.
double g_imuBaseMs = 5.0;      // --loopbase   nominal cost of one sensor read + control step
double g_imuSlowMs = 0.0;      // --imuslow    extra blocking ms per read (I2C clock-stretching / sick bus)
double g_loopJitterMs = 0.0;   // --loopjitter uniform [0,J) ms added per iteration
// --loopjitterflight: jitter that only appears AFTER ignition. The pad sensor-rate gate rejects a bus
// that is already slow when armed, so anything slow enough to trip the in-flight governor via
// --loopjitter would be caught on the pad and never fly. Degradation that APPEARS in flight (thermal,
// vibration, bus contention) is the case the governor actually exists for, and this is how to model it.
double g_loopJitterFlightMs = 0.0;
bool   g_inFlightNow = false;
// --miswire: model a vehicle wired/built with the control sense INVERTED (wrong SERVO_*_SIGN, a
// linkage rebuilt to the opposite sense, or swapped X/Y). The firmware cannot see any of these
// directly -- it only sees that its corrections make things worse -- so this is how the in-flight
// positive-feedback detector gets tested. 1 = invert X, 2 = invert Y, 3 = invert both.
int d_misWire = 0;
// ---- Impulse 2.2 electrical model (added 2026-08-04) ----------------------------------------
// The 2.2 board can measure VBAT, the 5 V servo rail and the 7.4 V pyro rail, and can test pyro
// continuity. Those map onto the two unexplained failures on record -- the suspected ASC036/ASC038
// brownout, and ASC038's unconfirmed melt-wire burn -- so the harness has to be able to EXPRESS
// them, or the new pre-arm gates are untested code. Pack model: V = Voc - I*Rint, with servo
// current rising with commanded slew and going to stall current when a servo is driven into its
// mechanical stop. A tired 2S pack is Rint ~0.3-0.6 ohm; a healthy one ~0.06-0.10.
double d_vbatOC=8.00;      // --vbat    pack open-circuit volts (2S: 8.4 full, 7.4 nominal, 6.6 flat)
double d_rInt=0.08;        // --rint    pack internal resistance, ohms
double d_stallA=0.0;       // --stallA  EXTRA amps drawn by a servo fighting its stop (0 = none)
double d_brownV=0.0;       // --brownv  volts below which the MCU is considered browned out (0 = never)
// ---- MID-FLIGHT MCU RESET ------------------------------------------------------------------------
// Until 2026-08-09 the harness set g_brownedOut and did nothing with it: the comment said "MCU would
// reset here" and no reset ever happened. setup() ran exactly ONCE per run, so resumeAfterReset() --
// the entire brownout/watchdog recovery feature -- had never executed in flight conditions. The only
// coverage was --seedphase, which pre-loads a stale record on the GROUND and checks the firmware does
// NOT resume: the negative case. The positive case (rail collapses at apogee -> board reboots ->
// chute still comes out) was the thing the feature exists for and was completely untested.
//
// A faithful reset needs FRESH firmware globals, and firmware globals are file-scope statics inside the
// .ino -- there is no way to re-zero them inside one process. So a reset here SNAPSHOTS the world
// (physics + EEPROM + clock + RNG), exits with code 7, and the caller relaunches with --statein. The new
// process gets genuinely fresh .data/.bss, which is exactly what the hardware does. run_flight() in
// regression.py drives the relaunch loop.
double   d_resetAt = -1;   // --resetat  s after ignition to force an MCU reset (-1 = never)
uint32_t g_resetCause = 0; // SRC_SRSR word handed to the NEXT boot; the firmware reads this in the SIL
int      g_bootCount  = 0; // how many times this flight has already rebooted (reported, and a loop guard)
static const char* d_stateIn  = nullptr;
static const char* d_stateOut = nullptr;
static bool g_resetPending = false;   // latch: one reset per process, else the snapshot could recurse
static int  g_injectedResetUsed = 0;  // carried in the snapshot; see SimState::injectedResetUsed
static uint32_t g_nextResetCause = 1; // SRC_SRSR word the NEXT boot will see: 1 = POR/brownout, 16 = WDT
// ---- hardware watchdog (WDOG1) -------------------------------------------------------------------
// The firmware's wdtFeed()/wdtStart() were no-ops off-target, so the watchdog did not exist in
// simulation. It is the last line of defence behind a hung I2C bus (FAILURE_MODES D2, still unguarded
// in firmware), and its whole value is the chain hang -> reset -> resumeAfterReset -> chute. None of
// that had ever run. Modelled here as a deadline the firmware must keep pushing forward.
static bool   g_wdtArmed = false;
static double g_wdtTimeout = 4.0;     // s, set from the firmware's own WATCHDOG_SECONDS
static double g_wdtLastFeed = 0;
void sim_wdtStart(double seconds){ g_wdtArmed=true; g_wdtTimeout=seconds; g_wdtLastFeed=g_micros/1e6; }
void sim_wdtFeed(){ g_wdtLastFeed = g_micros/1e6; }
double d_i2cHangAt = -1;              // --i2chang      s after ignition when the IMU bus stops responding
double d_fwHangAt  = -1;
double d_fwHangDur = 0;               // --firmwarehangdur  s the lockup lasts (0 = permanent)              // --firmwarehang s after ignition when the FIRMWARE itself locks up
int    d_pyroOpen=-1;
double d_imuStuckAt=-1;    // --imustuck  s after ignition when the IMU starts returning frozen samples
                           //             (models a wedged I2C bus / dead device -- FAILURE_MODES D2)
double d_imuSlowMs=0;      // --imuslow   ms of extra blocking time per IMU read (bus clock-stretching)      // --pyroopen  1..4 -> that channel reads OPEN (broken/missing igniter)
double g_vbat=8.0, g_v5=5.0, g_v74=8.0;   // live rail volts, read by the analogRead shim
bool   g_brownedOut=false;   // cleared at recovery so the log dumps are not charged (see shims/SD.h)
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
HardwareSerial Serial(false);      // set true to echo firmware Serial to stderr (--verbose)
HardwareSerial Serial1(false);
TwoWire  Wire;
SPIClass SPI;
SPIClass SPI1;   // sensor-board bus; shim returns 0x00 so the ICM/DPS read as absent
SDClass  SD;
EEPROMClass EEPROM;                // brownout-persistence backing store (RAM here; blank at every run start)

// ---- firmware symbols we override to sweep gains (Ascent_TVC.ino: plain float, not constexpr) ----
extern float P_GAIN, D_GAIN, I_GAIN;
extern float TVC_WN, TVC_ZETA; extern bool TVC_TORQUE_CMD;
extern float DRY_MASS_KG, DRY_L_M, DRY_IYY;   // what the FIRMWARE believes the airframe is         // torque-commanded design point (--wn / --zeta)
extern float keffEst, keffNominal, plantThrustN, thrustScale, burnFrac, plantL, plantIyy, plantMass;
extern float gyro_x, gyro_y, gyro_z;   // the firmware's OWN estimate: quaternion tilt X/Y (deg) + roll Z (deg)
extern float estZ, estVz;              // the firmware's PASSIVE vertical Kalman (baro+IMU) -- vs true alt/vz
extern float iTermX, iTermY;           // the firmware's integral-trim accumulators (TVC deg)

// ---- vehicle (matches Ascent_TVC.ino MEASURED 2026-07-06) ----
static const int    SERVO_X_PIN=4, SERVO_Y_PIN=3, P1=9, P3=11, P4=12, BUTTON=14;
static const double G=9.80665, SERVO_MULT=4.5, IGN_DELAY=0.2, BURN=3.45;   // physical linkage, servo deg per gimbal deg.
// 4.5 CORRECTED 2026-08-04: the 2026-08-03 bench value of 5.5 was a mis-measurement (Braxton).
// This now EQUALS Ascent_TVC.ino's SERVO_X/Y_MULT, so commanded TVC deg == true gimbal deg and the
// harness no longer models a command/linkage mismatch. Do not re-introduce one without a bench re-measure.
// PHYSICAL LINKAGE SENSE: which way the gimbal swings for a servo command above 90 deg. This used to be an
// UNNAMED assumption baked into the torque sign below (+1), and that made the harness silently unable to
// model the vehicle after a linkage rebuild -- flipping the firmware's SERVO_*_SIGN alone produced fake
// positive feedback and aborted every dispersed case. It is now an explicit property of the hardware.
//   +1 = pre-2026-08-03 build (firmware flew SERVO_*_SIGN=+1, e.g. ASC036)
//   -1 = current build. Set from the 2026-08-03 bench pitch-the-nose test, which found BOTH axes reversed.
// NOTE this is CALIBRATED to the bench observation, so a PASS here is a CONSISTENCY check, not independent
// confirmation of the sign. The bench test is the only authority for the sign; see Ascent_TVC.ino.
static const double LINKAGE_SENSE = -1.0;
static double IYY=0.0078, L_ARM=0.14, MASS=0.818;   // baseline = the PRE-ASC036 airframe; scale to the current
                                                     // vehicle with --iyy/--larm/--mass (see FLIGHT_LOG.md)
static const double IZZ0=4.5e-4;                 // roll inertia (slender body ~0.5*m*r^2, r~33mm) -- ~17x < IYY
static const double S_REF=0.003, CD=0.6, RHO=1.20, CN_AERO=2.0;
static const double C_DAMP=0.003, C_DAMP_Z=2.0e-5;   // aero rate damping (pitch/yaw, roll) -- weak over a 2 s boost
static double SERVO_TAU=0.03;   // s, servo first-order lag. NOT constexpr and sweepable via --servotau:
                                // this is the single largest UNMEASURED parameter in the model (it was a
                                // plausible hobby-servo default, never measured on this vehicle), and it is
                                // exactly what limits how far the gains can be raised. Bench_Latency.ino
                                // measures it for real; until then, sweep it and require margin.

// ---- dispersions / error sources (set from argv) ----
static double d_iyy=1, d_izz=1, d_thrust=1, d_L=1, d_tipX=0, d_tipY=0, d_misX=0, d_misY=0, d_sm=0;
static double d_gnoise=0, d_gbiasX=0, d_gbiasY=0, d_gbiasZ=0, d_servoSlew=400, d_gustX=0, d_gustY=0, d_gustHz=0.7;
static double d_rollCant=0, d_cgOffX=0, d_cgOffY=0, d_finMis=0;   // <-- NEW: roll-torque sources
// EXTRA ignition lag beyond the firmware's assumed IGN_DELAY=0.2 s. This is the scenario that motivated the
// post-burnout TVC extension: the firmware ends TVC a fixed 3.65 s after the ignition COMMAND, so if the motor
// lights late the motor is STILL BURNING when the gimbal centres -- and a centred gimbal on a vehicle whose
// thrust axis never passes exactly through the CG applies an uncontrolled trim torque. The real lag has never
// been measured on this vehicle (the only MTR###.CSV on record is from a no-ignition run), so it is a free
// parameter and must be swept, not assumed.
static double d_ignLag=0;
static unsigned d_seed=1;
static bool verbose=false, trace=false;

// F-15 thrust (N) vs time since ignition; avg ~14.5 N, 3.45 s
// Estes F15 thrust curve, 14 points, 49.65 N*s / 14.39 N avg over 3.45 s. Replaces a 3-segment
// approximation (ramp 26 N / decay / flat 14 N) that was close in impulse but wrong in shape -- and
// shape is what a gain schedule tracking keff = T*L/Iyy actually rides on.
// This is the harness's TRUTH. The firmware carries its own copy as a PRIOR and identifies the
// amplitude in flight, so --thrust injects a motor that is off-curve and thrustScale must find it.
static const int    F15N = 14;
static const double F15TT[F15N] = {0.00,0.05,0.10,0.15,0.20,0.30,0.40,0.50,1.00,1.50,2.00,2.50,3.00,3.45};
static const double F15FF[F15N] = {0.0 ,12.0,22.0,25.8,23.5,19.0,16.5,15.4,15.0,14.6,14.2,13.9,13.5, 0.0};
static double thrustAt(double tau){
  if(tau<=0 || tau>=F15TT[F15N-1]) return 0.0;
  for(int i=1;i<F15N;i++)
    if(tau<=F15TT[i]){
      double f=(tau-F15TT[i-1])/(F15TT[i]-F15TT[i-1]);
      return F15FF[i-1]+f*(F15FF[i]-F15FF[i-1]);
    }
  return 0.0;
}
// ---- PROPELLANT BURN-OFF -------------------------------------------------------------------------
// The harness used to hold MASS, IYY and L_ARM constant for the whole flight, so 60 g of F15 propellant
// -- 6.5% of the vehicle -- never left the aft end. That made the model structurally unable to express
// the very drift the firmware's plant estimator now corrects, and a PASS would have proved nothing
// about it. Composed about the GIMBAL PIVOT (a fixed point) then referred to the moving CG, which is
// the same construction the firmware uses, from the same physical numbers.
static const double PROP_M = 0.060, PROP_ARM = 0.040, MOTOR_NS = 49.6;
static const double PROP_IOWN = PROP_M*(3.0*0.012*0.012 + 0.070*0.070)/12.0;
static double keffMidEst=0, keffMidTrue=0, tscaleMid=0; static bool keffMidDone=false;  // sampled at burnFrac 0.5:
                                  // the end-of-flight values are uninformative (thrust is gone), so the
                                  // comparison that matters is taken mid-burn, where the loop is flying.
static double impulseTrue = 0;                    // N*s delivered so far (true, not estimated)
static double curMass=0, curL=0, curIyy=0;        // live mass properties, set by updateMassProps()
static void updateMassProps(){
  double wetM = MASS, wetL = L_ARM*d_L, wetI = IYY*d_iyy;      // as configured = FULL grain
  double dryM = wetM - PROP_M;
  double dryL = (wetM*wetL - PROP_M*PROP_ARM)/dryM;
  double iPivWet = wetI + wetM*wetL*wetL;
  double iPivDry = iPivWet - (PROP_IOWN + PROP_M*PROP_ARM*PROP_ARM);
  double f  = impulseTrue/MOTOR_NS; if(f<0) f=0; if(f>1) f=1;
  double mp = PROP_M*(1.0-f);
  curMass = dryM + mp;
  curL    = (dryM*dryL + mp*PROP_ARM)/curMass;
  curIyy  = (iPivDry + PROP_IOWN*(1.0-f) + mp*PROP_ARM*PROP_ARM) - curMass*curL*curL;
  if(!(curIyy > 1e-5)) curIyy = wetI;
}

// ---- physics state (quaternion attitude, body rates rad/s) ----
static double Qw=1,Qx=0,Qy=0,Qz=0;         // body->world attitude
static double wbx=0,wby=0,wbz=0;           // body rates: X pitch, Y yaw, Z ROLL
static double alt=0, vz=0;
static bool ignited=false, lifted=false; static double ignT=-1;
static double servoXa=90, servoYa=90;      // slew-limited actual servo (deg)
static bool p4Fired=false, p1Fired=false; static double p4Tilt=0,p4Alt=0,p4Vz=0; static bool p4AtDescent=false;
static double p4T=-1;   // sim time the chute fired -- the run ends a grace period AFTER this, not immediately,
                        // so the firmware's recovery beeps and its three log dumps can complete. The beeps are
                        // blocking delay()s and now run AFTER the pyro (a 2026-08-04 firmware fix), so an
                        // immediate longjmp truncated the run before dumpControlLog() ever executed.
static double apogee=0, maxTilt=0, maxRate=0, mtBoost=0, mrBoost=0;   // *Boost = while thrust ON (control has authority)
static double boostEndTilt=0;                                        // tiltTRUE at the LAST thrust-on sample (~burnout) =
                                                                    //   the STEADY-STATE lean the integral trim winds down
static double maxRollAng=0, maxRollRate=0;                           // roll diagnostics (deg, dps)
static uint32_t rngState=1;
double sim_frand(){ rngState=rngState*1664525u+1013904223u; return ((rngState>>8)&0xFFFF)/65536.0; }
static double urand(){ rngState=rngState*1664525u+1013904223u; return ((rngState>>8)&0xFFFF)/32768.0-1.0; } // [-1,1)
static jmp_buf endJmp;

// ---- reset snapshot ------------------------------------------------------------------------------
// Everything a real MCU reset does NOT destroy: the vehicle keeps flying, the emulated EEPROM keeps its
// contents, and wall-clock keeps running. Everything else (all firmware RAM) is deliberately lost by
// virtue of relaunching the process. Stat accumulators ride along so the final report covers the whole
// flight rather than just the last boot. POD only -- this is fwrite/fread'd verbatim.
struct SimState {
  uint32_t magic;                                  // guards against a stale/foreign snapshot
  uint64_t micros;
  double Qw,Qx,Qy,Qz, wbx,wby,wbz, alt, vz;
  double servoXa, servoYa, ignT;
  int    ignited, lifted, p4Fired, p1Fired, p4AtDescent;
  double p4Tilt, p4Alt, p4Vz, p4T;
  double apogee, maxTilt, maxRate, mtBoost, mrBoost, boostEndTilt, maxRollAng, maxRollRate;
  double vbat, v74, v5;
  uint32_t rngState, resetCause;
  int    bootCount;
  int    injectedResetUsed;   // --resetat is ONE-SHOT: without this the reset condition is still true
                              // on the next boot (same ignT, later clock) and the flight reboots forever.
                              // A --brownv reset is deliberately NOT latched -- a rail that keeps
                              // collapsing SHOULD keep resetting, which is what RESUME_MAX_BOOTS is for.
  unsigned char eeprom[4284];                      // matches the EEPROM shim's cell[]
};
static const uint32_t STATE_MAGIC = 0x52535431;    // "RST1"

static void stateSave(const char* path){
  SimState s{}; s.magic=STATE_MAGIC; s.micros=g_micros;
  s.Qw=Qw; s.Qx=Qx; s.Qy=Qy; s.Qz=Qz; s.wbx=wbx; s.wby=wby; s.wbz=wbz; s.alt=alt; s.vz=vz;
  s.servoXa=servoXa; s.servoYa=servoYa; s.ignT=ignT;
  s.ignited=ignited; s.lifted=lifted; s.p4Fired=p4Fired; s.p1Fired=p1Fired; s.p4AtDescent=p4AtDescent;
  s.p4Tilt=p4Tilt; s.p4Alt=p4Alt; s.p4Vz=p4Vz; s.p4T=p4T;
  s.apogee=apogee; s.maxTilt=maxTilt; s.maxRate=maxRate; s.mtBoost=mtBoost; s.mrBoost=mrBoost;
  s.boostEndTilt=boostEndTilt; s.maxRollAng=maxRollAng; s.maxRollRate=maxRollRate;
  s.vbat=g_vbat; s.v74=g_v74; s.v5=g_v5;
  s.rngState=rngState; s.bootCount=g_bootCount+1;
  s.injectedResetUsed = (d_resetAt>=0) ? 1 : g_injectedResetUsed;
  // Bit 0 = POR, which is the path an LVD brownout resets through -- the firmware separates brownout
  // from a bench power-on using persist.phase, not this bit, so handing it POR is the honest emulation.
  s.resetCause = g_nextResetCause;
  for(int i=0;i<(int)sizeof(s.eeprom);i++) s.eeprom[i] = EEPROM.read(i);
  FILE* f=fopen(path,"wb"); if(!f) return; fwrite(&s,sizeof(s),1,f); fclose(f);
}

static bool stateLoad(const char* path){
  FILE* f=fopen(path,"rb"); if(!f) return false;
  SimState s{}; size_t n=fread(&s,sizeof(s),1,f); fclose(f);
  if(n!=1 || s.magic!=STATE_MAGIC) return false;
  g_micros=s.micros;
  Qw=s.Qw; Qx=s.Qx; Qy=s.Qy; Qz=s.Qz; wbx=s.wbx; wby=s.wby; wbz=s.wbz; alt=s.alt; vz=s.vz;
  servoXa=s.servoXa; servoYa=s.servoYa; ignT=s.ignT;
  ignited=s.ignited; lifted=s.lifted; p4Fired=s.p4Fired; p1Fired=s.p1Fired; p4AtDescent=s.p4AtDescent;
  p4Tilt=s.p4Tilt; p4Alt=s.p4Alt; p4Vz=s.p4Vz; p4T=s.p4T;
  apogee=s.apogee; maxTilt=s.maxTilt; maxRate=s.maxRate; mtBoost=s.mtBoost; mrBoost=s.mrBoost;
  boostEndTilt=s.boostEndTilt; maxRollAng=s.maxRollAng; maxRollRate=s.maxRollRate;
  g_vbat=s.vbat; g_v74=s.v74; g_v5=s.v5;
  rngState=s.rngState; g_resetCause=s.resetCause; g_bootCount=s.bootCount;
  g_injectedResetUsed=s.injectedResetUsed;
  for(int i=0;i<(int)sizeof(s.eeprom);i++) EEPROM.write(i, s.eeprom[i]);
  return true;
}

static void qNorm(){ double n=sqrt(Qw*Qw+Qx*Qx+Qy*Qy+Qz*Qz); if(n>1e-12){Qw/=n;Qx/=n;Qy/=n;Qz/=n;} }
static void qRotB2W(double bx,double by,double bz,double&wx,double&wy,double&wz){   // v_world = R(Q)*v_body
  double t2=Qw*Qx,t3=Qw*Qy,t4=Qw*Qz,t5=-Qx*Qx,t6=Qx*Qy,t7=Qx*Qz,t8=-Qy*Qy,t9=Qy*Qz,t10=-Qz*Qz;
  wx=2*((t8+t10)*bx+(t6-t4)*by+(t3+t7)*bz)+bx;
  wy=2*((t4+t6)*bx+(t5+t10)*by+(t9-t2)*bz)+by;
  wz=2*((t7-t3)*bx+(t2+t9)*by+(t5+t8)*bz)+bz;
}
static void qRotW2B(double wx,double wy,double wz,double&bx,double&by,double&bz){   // v_body = R(Q)^T*v_world
  double t2=Qw*Qx,t3=Qw*Qy,t4=Qw*Qz,t5=-Qx*Qx,t6=Qx*Qy,t7=Qx*Qz,t8=-Qy*Qy,t9=Qy*Qz,t10=-Qz*Qz;
  bx=2*((t8+t10)*wx+(t6+t4)*wy+(t7-t3)*wz)+wx;
  by=2*((t6-t4)*wx+(t5+t10)*wy+(t2+t9)*wz)+wy;
  bz=2*((t3+t7)*wx+(t9-t2)*wy+(t5+t8)*wz)+wz;
}

// Pack model: V = Voc - I*Rint. Quiescent load is the Teensy+sensors+buzzer; servo current rises
// with how hard the gimbal is being driven, and jumps by d_stallA if a servo is against its stop.
static void updateRails(){
  double cmdX=g_servoDeg[SERVO_X_PIN], cmdY=g_servoDeg[SERVO_Y_PIN];
  double errX=fabs(cmdX-servoXa), errY=fabs(cmdY-servoYa);      // how far the servo still has to move
  double iServo = 0.10 + 0.020*(errX+errY);                     // A, idle -> moving
  bool atStopX = fabs(cmdX-90.0) > 22.4, atStopY = fabs(cmdY-90.0) > 22.4;   // ~5 deg gimbal at 4.5:1
  if(atStopX) iServo += d_stallA;
  if(atStopY) iServo += d_stallA;
  double I = 0.25 + iServo;                                     // + quiescent
  g_vbat = d_vbatOC - I*d_rInt;
  if(g_vbat < 0) g_vbat = 0;
  g_v74  = g_vbat;                                              // pyro rail is the raw pack
  g_v5   = (g_vbat > 5.6) ? 5.00 : (g_vbat - 0.55);             // buck regulates until dropout
  if(g_v5 < 0) g_v5 = 0;
  // NOT a latch. It used to set true and never clear, which was harmless while nothing read it, but
  // once it drove a real reset it meant the first sag rebooted the flight FOREVER: every relaunched
  // process saw the stale flag and reset again at the same instant, making no progress. The rail is a
  // live quantity -- when the servos stop stalling it recovers, and only a rail that is STILL collapsed
  // should reset the board again. A genuinely repeating collapse is what RESUME_MAX_BOOTS exists for.
  g_brownedOut = (d_brownV > 0 && g_vbat < d_brownV);
}

// Exported for the IMU shim: ignited/ignT are file-static, so the check lives here rather than
// leaking two more globals across the shim boundary.
bool sim_i2cHung(){ return d_i2cHangAt >= 0 && ignited && (g_micros/1e6 - ignT) > d_i2cHangAt; }
// --firmwarehangdur bounds the lockup. A fault that recurs identically on EVERY boot is unrecoverable
// by construction -- no parachute can be deployed by code that cannot run -- so the interesting case is
// the transient one (a corrupted variable, a race, a glitched peripheral), where the reboot clears it.
// That is what the watchdog actually buys, and it is only visible if transience can be modelled.
bool sim_firmwareHung(){
  if(d_fwHangAt < 0 || !ignited) return false;
  double dt = g_micros/1e6 - ignT;
  if(dt <= d_fwHangAt) return false;
  return d_fwHangDur <= 0 || dt < (d_fwHangAt + d_fwHangDur);
}

static void synthSensors(){
  updateRails();
  // TRUE tilt (angle of body +Z from world up) and roll (twist about body Z), from the quaternion
  double zbx,zby,zbz; qRotB2W(0,0,1, zbx,zby,zbz);
  double tiltTrue = acos(zbz>1?1:(zbz<-1?-1:zbz))*180.0/M_PI;
  double rollTrue = 2.0*atan2(Qz,Qw)*180.0/M_PI;
  if(tiltTrue>maxTilt) maxTilt=tiltTrue;
  double rm=sqrt(wbx*wbx+wby*wby)*180.0/M_PI;
  if(rm>maxRate) maxRate=rm;
  if(fabs(rollTrue)>maxRollAng) maxRollAng=fabs(rollTrue);
  if(fabs(wbz)*180.0/M_PI>maxRollRate) maxRollRate=fabs(wbz)*180.0/M_PI;
  double T = (ignited)? thrustAt(g_micros/1e6-ignT-IGN_DELAY-d_ignLag)*d_thrust : 0.0;   // d_ignLag: motor lights LATE
  if(T>0.5){ if(tiltTrue>mtBoost)mtBoost=tiltTrue; if(rm>mrBoost)mrBoost=rm; boostEndTilt=tiltTrue; }
  // gyro: TRUE body rates (deg/s). Y inverted to match the firmware's -getGyroY(); Z now REAL (was hardcoded 0).
  // +/-2000 dps clamp models the reconfigured FSR (ASC007 railed the old +/-500 dps only in a full tumble).
  // Injected IMU fault: after d_imuStuckAt the device returns bit-identical samples forever, which
  // is what a wedged I2C bus or a dead MEMS looks like from the firmware's side.
  if(d_imuStuckAt>=0 && ignited && (g_micros/1e6-ignT) > d_imuStuckAt) return;   // leave g_imu_* frozen
  double gx =  wbx*180.0/M_PI + d_gbiasX + d_gnoise*urand();
  double gy = -wby*180.0/M_PI + d_gbiasY + d_gnoise*urand();
  double gz =  wbz*180.0/M_PI + d_gbiasZ + d_gnoise*urand();
  auto clampd=[](double v,double lim){ return v>lim?lim:(v<-lim?-lim:v); };
  g_imu_gyroX=clampd(gx,2000.0); g_imu_gyroY=clampd(gy,2000.0); g_imu_gyroZ=clampd(gz,2000.0);
  // accel = body-frame specific force f = R^T*(a_world + (0,0,G)). Vertical-only translation -> a_world=(0,0,az).
  double azw = lifted ? (T/curMass - G - 0.5*RHO*S_REF*CD*vz*fabs(vz)/curMass) : 0.0;   // LIVE mass: this IS what the firmware's estimator reads back
  double fbx,fby,fbz; qRotW2B(0,0, azw+G, fbx,fby,fbz);
  g_imu_accX=clampd(fbx/G,16.0); g_imu_accY=clampd(fby/G,16.0); g_imu_accZ=clampd(fbz/G,16.0);
  g_baro_alt = alt + 0.15*urand();
}

void sim_advance(unsigned long ms){
  if(!ignited && g_pin[P3]!=1){                    // on-pad phase (pre-ignition): nothing moves -> fast-path
    g_micros += (uint64_t)ms*1000ULL;
    updateRails();   // MUST run here too: the 2.2 pre-arm gates read the rails while still on the pad,
                     // and this fast-path skips synthSensors(). Without this the firmware saw a stale
                     // initial 8.0 V and the low-battery gate could never fire in simulation.
    g_imu_gyroX=d_gbiasX; g_imu_gyroY=d_gbiasY; g_imu_gyroZ=d_gbiasZ;   // pad: at rest + residual gyro bias
    g_imu_accX=0; g_imu_accY=0; g_imu_accZ=1.0; g_baro_alt=0;
    if(g_micros/1e6>60.0) longjmp(endJmp,2);
    return;
  }
  const double H=0.001;
  double rem=ms/1000.0;
  while(rem>1e-9){
    double dt = rem>H?H:rem; rem-=dt;
    g_micros += (uint64_t)(dt*1e6+0.5);
    double now=g_micros/1e6;
    if(!ignited && g_pin[P3]==1){ ignited=true; ignT=now; g_inFlightNow=true; }
    if(g_pin[P4]==1 && !p4Fired){ p4Fired=true; g_sdStallEnabled=false; double zx,zy,zz; qRotB2W(0,0,1,zx,zy,zz);
      p4Tilt=acos(zz>1?1:(zz<-1?-1:zz))*180.0/M_PI; p4Alt=alt; p4Vz=vz; p4AtDescent=(vz<0); p4T=now; }
    if(g_pin[P1]==1) p1Fired=true;

    double T=0; if(ignited){ double tau=now-ignT-IGN_DELAY-d_ignLag; if(tau>=0) T=thrustAt(tau)*d_thrust; }  // d_ignLag shifts the
    // whole burn later, so the motor can still be running when the firmware's fixed TVC window closes
    if(!lifted){                                   // rail-constrained: no motion until thrust beats weight
      if(T>curMass*G){ lifted=true; wbx=d_tipX*M_PI/180; wby=d_tipY*M_PI/180; }   // rail-exit tip-off RATE
    } else {
      // --- realized servos: first-order lag (SERVO_TAU) THEN slew-rate clamp (matches sim_main; the ascent flight
      //     ran real servos with both -- the old harness modeled only slew, so its loop was artificially glassy) ---
      double cmdX=g_servoDeg[SERVO_X_PIN], cmdY=g_servoDeg[SERVO_Y_PIN], mx=d_servoSlew*dt;
      double aLag=fmin(dt/SERVO_TAU,1.0);
      double sX=aLag*(cmdX-servoXa); if(sX>mx)sX=mx; if(sX<-mx)sX=-mx; servoXa+=sX;
      double sY=aLag*(cmdY-servoYa); if(sY>mx)sY=mx; if(sY<-mx)sY=-mx; servoYa+=sY;
      // servo deg -> physical gimbal rad. LINKAGE_SENSE carries which way the linkage actually swings, so the
      // firmware's SERVO_*_SIGN and the hardware's linkage are modelled as the two INDEPENDENT things they are.
      double tvcX=LINKAGE_SENSE*(servoXa-90.0)/SERVO_MULT*M_PI/180.0;   // rad, X-plane gimbal (firmware's gyro_x channel)
      double tvcY=LINKAGE_SENSE*(servoYa-90.0)/SERVO_MULT*M_PI/180.0;   // rad, Y-plane gimbal
      if(d_misWire & 1) tvcX = -tvcX;    // built with the X sense reversed
      if(d_misWire & 2) tvcY = -tvcY;    // built with the Y sense reversed
      updateMassProps();
      double iyy=curIyy, izz=IZZ0*d_izz, L=curL;   // LIVE: propellant burns off, CG moves fwd, Iyy drops
      double qdyn=0.5*RHO*vz*vz;
      double gustX=d_gustX*sin(2*M_PI*d_gustHz*now), gustY=d_gustY*cos(2*M_PI*d_gustHz*now*1.3);
      // --- body torques ---
      // CONTROL (restoring, bench-verified sign): +tvc on an axis drives that axis' rate DOWN.
      double Mx = -T*L*tvcX;
      double My = -T*L*tvcY;
      // TRIM: thrust misalignment (deg) + CG lateral offset (thrust misses the CG: Mx=-cgy*T, My=+cgx*T).
      //   The CG-offset trim scales with T, so the lean GROWS as thrust builds -- as ASC007 showed.
      Mx += T*L*(d_misX*M_PI/180.0) - d_cgOffY*T;
      My += T*L*(d_misY*M_PI/180.0) + d_cgOffX*T;
      // gust disturbance torque (as angular accel -> torque)
      Mx += iyy*gustX; My += iyy*gustY;
      // AERO normal-force moment from crossflow. Velocity is ~vertical (world +z), so the crossflow (and thus the
      // aoa) is set by the TILT -- and it is WORLD-referenced: as the body ROLLS, this moment's body-frame direction
      // rotates, so the firmware's naive body-frame integrator chases a rotating disturbance and its correction
      // inverts near 180 deg roll -> the ASC007 roll-coupling divergence. d_sm = static margin (m): + stable (CP aft
      // of CG, weathercocks toward the flight path), - UNSTABLE (finless CP fwd of CG, grows the aoa).
      { double vbx,vby,vbz; qRotW2B(0,0,vz, vbx,vby,vbz);
        double Vs=fmax(sqrt(vbx*vbx+vby*vby+vbz*vbz),0.5);
        double kAero=qdyn*S_REF*CN_AERO/Vs;
        Mx += d_sm*kAero*(-vby); My += d_sm*kAero*( vbx); }
      // rate damping
      Mx += -C_DAMP*wbx; My += -C_DAMP*wby;
      // ROLL torque: nozzle tangential cant (constant under thrust) + CG-offset TVC leak (every TVC
      //   correction leaks roll: Mz=T*(cgx*db - cgy*da)) + aero fin/misalignment roll. Izz is tiny -> winds up.
      double Mz = T*d_rollCant + T*(d_cgOffX*tvcY - d_cgOffY*tvcX) + qdyn*d_finMis - C_DAMP_Z*wbz;
      // --- Euler rigid-body eqs (Ix=Iy=iyy) WITH gyroscopic coupling: this is what diverges under roll ---
      double dwx = (Mx + (iyy-izz)*wby*wbz)/iyy;
      double dwy = (My + (izz-iyy)*wbx*wbz)/iyy;
      double dwz =  Mz/izz;
      wbx+=dwx*dt; wby+=dwy*dt; wbz+=dwz*dt;
      // --- quaternion kinematics: Qdot = 0.5 * Q (x) (0,wb) ---
      double dQw=0.5*(-Qx*wbx-Qy*wby-Qz*wbz), dQx=0.5*(Qw*wbx+Qy*wbz-Qz*wby),
             dQy=0.5*(Qw*wby-Qx*wbz+Qz*wbx), dQz=0.5*(Qw*wbz+Qx*wby-Qy*wbx);
      Qw+=dQw*dt; Qx+=dQx*dt; Qy+=dQy*dt; Qz+=dQz*dt; qNorm();
      // --- vertical translation (axial thrust component along world up ~ cos(tilt)) ---
      double zbx,zby,zbz; qRotB2W(0,0,1,zbx,zby,zbz);
      double az = T*zbz/curMass - G - 0.5*RHO*S_REF*CD*vz*fabs(vz)/curMass;
      vz+=az*dt; alt+=vz*dt; if(alt<0){alt=0; if(vz<0)vz=0;}
    }
    if(!keffMidDone && burnFrac >= 0.5f){ keffMidDone=true; keffMidEst=keffEst; keffMidTrue=T*curL/curIyy; tscaleMid=thrustScale; }
    if(T>0) impulseTrue += T*dt;      // true delivered impulse -> drives the propellant burn-off
    if(alt>apogee) apogee=alt;
    synthSensors();
    if(trace && ((g_micros/1000)%50==0)){ double zx,zy,zz; qRotB2W(0,0,1,zx,zy,zz);
      double tiltTrue=acos(zz>1?1:(zz<-1?-1:zz))*180/M_PI, rollTrue=2*atan2(Qz,Qw)*180/M_PI;
      double estTilt=sqrt((double)gyro_x*gyro_x+(double)gyro_y*gyro_y);
      fprintf(stderr,"t=%.2f T=%.1f | tiltTRUE=%.1f tiltEST=%.1f | altTRUE=%.1f altEST=%.1f | vzTRUE=%.2f vzEST=%.2f | roll=%.0f\n",
        now,T, tiltTrue,estTilt, alt,(double)estZ, vz,(double)estVz, rollTrue); }
    // ---- MCU RESET (brownout / injected) ----
    // Checked BEFORE the end conditions so a reset during the post-deploy grace period still counts.
    // Not armed until the vehicle is off the pad: a pad brownout is a pre-arm gate problem (section 5),
    // and resumeAfterReset() is explicitly required to fall through to a normal flight on the ground.
    bool wdtExpired = g_wdtArmed && (now - g_wdtLastFeed) > g_wdtTimeout;
    if(d_stateOut && lifted && !g_resetPending){
      bool due = (d_resetAt>=0 && !g_injectedResetUsed && ignited && (now-ignT) >= d_resetAt)
                 || g_brownedOut || wdtExpired;
      if(due){
        g_resetPending = true;
        // Bit 4 = WATCHDOG, bit 0 = POR (the path an LVD brownout takes). The firmware raises
        // F_WDT_RESET off bit 4, so handing it the right cause is what makes that branch testable.
        g_nextResetCause = wdtExpired ? (1u<<4) : 1u;
        stateSave(d_stateOut);
        fprintf(stderr,"[MCU RESET (%s) at t=%.2f s (alt %.1f m, vz %+.1f m/s) -- relaunching]\n",
                wdtExpired?"WATCHDOG":"brownout/injected", now, alt, vz);
        longjmp(endJmp,7);
      }
    }
    // A hang with the watchdog somehow disabled would spin here forever; make that loud rather than
    // letting the suite hang, since "the test never finished" reads as "the test never ran".
    if(wdtExpired && !d_stateOut){
      fprintf(stderr,"[WATCHDOG expired at t=%.2f s but no --stateout: cannot model the reset]\n", now);
      longjmp(endJmp,2);
    }
    // ---- end conditions ----
    if(p4Fired && p4T>=0 && (now - p4T > 3.0)) longjmp(endJmp,1);   // recovery deployed + grace for the log dumps
    if(now>60.0) longjmp(endJmp,2);                                     // hard timeout
  }
}

// ---- Arduino core ----
unsigned long millis(){ return (unsigned long)(g_micros/1000ULL); }
unsigned long micros(){ return (unsigned long)g_micros; }
void delay(unsigned long ms){ sim_advance(ms); }
void delayMicroseconds(unsigned int us){ sim_advance((us+999)/1000); }
void pinMode(int,int){}
void digitalWrite(int p,int v){ if(p>=0&&p<64)g_pin[p]=v;
  // Stop charging SD latency the INSTANT the chute pyro is commanded. Waiting for the next
  // sim_advance() is too late: the first dump write would charge a stall, sim_advance() would
  // hit the recovery end-condition, and longjmp out mid-dump -- losing the CTL###.CSV we want.
  if(p==P4 && v==1) g_sdStallEnabled=false; }
void analogReadResolution(int){}
// Impulse 2.2 sense pins. 12-bit counts over a 3.3 V reference, matching the firmware's
// analogReadResolution(12). Divider ratios are the ones read out of the board file.
int  analogRead(int pin){
  auto counts=[&](double vAtPin){ double c=vAtPin/3.3*4095.0; return (int)(c<0?0:(c>4095?4095:c)); };
  switch(pin){
    case 21: return counts(g_vbat/3.128);        // VBAT_SENSE   R54 10k / R55 4.7k
    case 22: return counts(g_v5  /2.000);        // SERVO_V_SENSE R56 10k / R57 10k
    case 23: return counts(g_v74 /3.128);        // PYRO_V_SENSE  R58 100k / R59 47k
    // Pyro continuity: LED clamps the node near ~1.8 V when the igniter is intact, ~0 V when open.
    // A channel also reads OPEN once it has FIRED (the melt wire burns through) -- that is what makes
    // this a post-fire confirmation, not just a pre-arm check.
    case 38: return counts((d_pyroOpen==1 || g_pin[P1]==1) ? 0.05 : 1.80);
    case 39: return counts((d_pyroOpen==2)               ? 0.05 : 1.80);
    case 40: return counts((d_pyroOpen==3 || g_pin[P3]==1)? 0.05 : 1.80);
    case 41: return counts((d_pyroOpen==4 || g_pin[P4]==1)? 0.05 : 1.80);
    default: return 0;
  }
}
void tone(int,unsigned int){}
void tone(int,unsigned int,unsigned long){}   // duration form -- non-blocking on hardware
void noTone(int){}
// button auto-arm: deliver press pulses in the first 3 s (arming window); LOW afterwards so the firmware's
// recovery loop `while(digitalRead(BUTTON)==LOW)` enters and lets the longjmp fire.
static bool btnHeld=false;
// SENS_DET (Teensy 32) is ACTIVE LOW on Impulse 2.2: the sensor board ties J1 pad 12 to GND, so
// CABLE PRESENT pulls this pin down and ABSENT floats it high through LED4/R64.
// The SIL cannot model the ICM-42688-P -- the SPI shim returns 0x00, so the part reads as dead --
// which means the honest state to report here is "no sensor board fitted", i.e. HIGH. Leaving it
// at the default 0 would claim the board is present-but-broken and raise F_IMU_FAILOVER on every
// healthy flight, which the degradation-ladder regression would (rightly) fail.
static const int SIM_PIN_SENS_DET = 32;
int digitalRead(int p){
  if(p==BUTTON){
    if(g_micros > 3000000ULL) return 0;
    if(btnHeld){ btnHeld=false; return 0; }
    btnHeld=true; return 1;
  }
  if(p==SIM_PIN_SENS_DET) return 1;      // no sensor board in simulation -- see note above
  return (p>=0&&p<64)?g_pin[p]:0;
}

void setup(); void loop();

static double getarg(int argc,char**argv,const char*k,double def){
  for(int i=1;i<argc-1;i++) if(!strcmp(argv[i],k)) return atof(argv[i+1]);
  return def;
}
int main(int argc,char**argv){
  d_iyy=getarg(argc,argv,"--iyy",1); d_izz=getarg(argc,argv,"--izz",1);
  d_thrust=getarg(argc,argv,"--thrust",1); d_L=getarg(argc,argv,"--larm",1);
  d_tipX=getarg(argc,argv,"--tipx",0); d_tipY=getarg(argc,argv,"--tipy",0);
  d_misX=getarg(argc,argv,"--misx",0); d_misY=getarg(argc,argv,"--misy",0);
  d_sm=getarg(argc,argv,"--sm",0);                                               // m: static margin (+ stable, - unstable)
  d_gnoise=getarg(argc,argv,"--gnoise",0);
  d_gbiasX=getarg(argc,argv,"--gbiasx",0); d_gbiasY=getarg(argc,argv,"--gbiasy",0); d_gbiasZ=getarg(argc,argv,"--gbiasz",0);
  d_servoSlew=getarg(argc,argv,"--slew",400); d_gustX=getarg(argc,argv,"--gustx",0); d_gustY=getarg(argc,argv,"--gusty",0);
  d_rollCant=getarg(argc,argv,"--rollcant",0);                                   // m: nozzle tangential cant arm
  d_cgOffX=getarg(argc,argv,"--cgoffx",0); d_cgOffY=getarg(argc,argv,"--cgoffy",0);  // m: CG lateral offset
  d_finMis=getarg(argc,argv,"--finmis",0);
  d_vbatOC=getarg(argc,argv,"--vbat",d_vbatOC);
  d_rInt  =getarg(argc,argv,"--rint",d_rInt);
  d_stallA=getarg(argc,argv,"--stallA",d_stallA);
  d_brownV=getarg(argc,argv,"--brownv",d_brownV);
  d_pyroOpen=(int)getarg(argc,argv,"--pyroopen",-1);
  d_imuStuckAt=getarg(argc,argv,"--imustuck",-1);
  g_imuBaseMs   =getarg(argc,argv,"--loopbase",g_imuBaseMs);
  g_imuSlowMs   =getarg(argc,argv,"--imuslow",0);      // now ACTUALLY consumed by the IMU shim
  g_loopJitterMs=getarg(argc,argv,"--loopjitter",0);
  g_loopJitterFlightMs=getarg(argc,argv,"--loopjitterflight",0);
  d_misWire=(int)getarg(argc,argv,"--miswire",0);
  { double sdl=getarg(argc,argv,"--sdlatency",0);   // ms per logData() open+write+close. ASC038 card: 143
    g_sdOpenLatencyMs=sdl*0.5; g_sdCloseLatencyMs=sdl*0.5; }
  SERVO_TAU=getarg(argc,argv,"--servotau",SERVO_TAU);   // s, actuator lag -- the gain-limiting unknown
  MASS=getarg(argc,argv,"--mass",MASS);        // kg, all-up liftoff mass -- affects trajectory/apogee (and hence the
                                               // backup-chute timing check), not the rotational control test
  d_ignLag=getarg(argc,argv,"--ignlag",0);      // s, EXTRA ignition lag beyond the assumed 0.2 s                                       // aero roll trim coeff
  d_seed=(unsigned)getarg(argc,argv,"--seed",1); rngState=d_seed?d_seed:1;
  for(int i=1;i<argc;i++){ if(!strcmp(argv[i],"--verbose")){verbose=true;Serial=HardwareSerial(true);} if(!strcmp(argv[i],"--trace"))trace=true; }
  // gain sweep (adjudicate the ASC007 hot gains): env PGAIN/DGAIN or --pgain/--dgain override the firmware defaults
  double pg=getarg(argc,argv,"--pgain",-1), dg=getarg(argc,argv,"--dgain",-1), ig=getarg(argc,argv,"--igain",-1);
  double wn=getarg(argc,argv,"--wn",-1), zt=getarg(argc,argv,"--zeta",-1);
  // --tellfirmware: derive the firmware's DRY_* from the TRUE dispersed airframe, i.e. simulate the
  // user having measured this rocket and typed the numbers in. Without it the firmware keeps believing
  // the nominal airframe, which is exactly the stale-gain situation after every rebuild.
  bool tellFw=false; for(int i=1;i<argc;i++) if(!strcmp(argv[i],"--tellfirmware")) tellFw=true;
  bool fixedGain=false;
  for(int i=1;i<argc;i++) if(!strcmp(argv[i],"--fixedgain")) fixedGain=true;
  if(getenv("PGAIN")) pg=atof(getenv("PGAIN"));
  if(getenv("DGAIN")) dg=atof(getenv("DGAIN"));
  if(getenv("IGAIN")) ig=atof(getenv("IGAIN"));

  // --seedphase N : pre-load the EEPROM brownout record as if the board had just rebooted mid-flight with
  // phase N (1=boost, 2=coast). SAFETY TEST: the vehicle here is sitting still on the pad, so looksAirborne()
  // must reject it and the firmware must fall through to a NORMAL flight -- a stale record must never fire a
  // pyro in someone's hands. Expect the outcome to be identical to the un-seeded run.
  // --seedboots N: pre-load persist.boots. RESUME_MAX_BOOTS caps how many times one flight may reboot
  // and still try to resume; seeding the counter is the only practical way to reach that cap, because
  // a rail sick enough to reboot repeatedly in the physics model is a rail that cannot fly at all.
  int seedBoots=(int)getarg(argc,argv,"--seedboots",0);
  int seedPhase=(int)getarg(argc,argv,"--seedphase",0);
  if(seedPhase>0){
    struct SeedRec{ uint32_t magic; uint8_t phase; uint16_t boots; float gbx,gby,gbz; float groundAlt; uint32_t sum; } r{};
    r.magic=0x54564332; r.phase=(uint8_t)seedPhase; r.boots=(uint16_t)seedBoots;
    r.gbx=r.gby=r.gbz=0; r.groundAlt=0;
    const uint8_t* b=(const uint8_t*)&r; uint32_t h=2166136261u;          // same FNV-1a the firmware uses
    for(size_t i=0;i<sizeof(SeedRec)-sizeof(uint32_t);i++){ h^=b[i]; h*=16777619u; }
    r.sum=h; EEPROM.put(0,r);
    fprintf(stderr,"[seeded EEPROM brownout record: phase=%d]\n",seedPhase);
  }

  updateMassProps();       // seed curMass/curL/curIyy before ANY physics or sensor synthesis runs
  if(tellFw){              // must precede setup(): that is where keffNominal is computed
    double wetM=MASS, wetL=L_ARM*d_L, wetI=IYY*d_iyy;
    double dryM=wetM-PROP_M;
    DRY_MASS_KG=(float)dryM;
    DRY_L_M    =(float)((wetM*wetL - PROP_M*PROP_ARM)/dryM);
    DRY_IYY    =(float)((wetI + wetM*wetL*wetL - (PROP_IOWN + PROP_M*PROP_ARM*PROP_ARM))
                        - dryM*DRY_L_M*DRY_L_M);
  }
  g_vbat=d_vbatOC; g_v74=d_vbatOC; g_v5=(d_vbatOC>5.6)?5.00:(d_vbatOC-0.55);  // seed before setup()
  d_resetAt   = getarg(argc,argv,"--resetat",-1);
  d_i2cHangAt = getarg(argc,argv,"--i2chang",-1);
  d_fwHangAt  = getarg(argc,argv,"--firmwarehang",-1);
  d_fwHangDur = getarg(argc,argv,"--firmwarehangdur",0);
  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"--stateout") && i+1<argc) d_stateOut=argv[++i];
    else if(!strcmp(argv[i],"--statein") && i+1<argc) d_stateIn=argv[++i];
  }
  // Restore BEFORE setup(): the firmware reads EEPROM and the reset cause during setup(), and the
  // physics must already be mid-flight when resumeAfterReset() asks looksAirborne().
  if(d_stateIn && !stateLoad(d_stateIn)){ fprintf(stderr,"[statein: unreadable snapshot]\n"); return 9; }
  // With --statein, --seedboots patches ONLY the boots field of the record the snapshot carried, leaving
  // phase / gyro bias / ground altitude as the flight actually left them. That is what makes the
  // RESUME_MAX_BOOTS cap reachable: it is checked before looksAirborne(), so it cannot be exercised from
  // the pad, and a rail sick enough to reboot three times for real is one that never leaves the ground.
  if(d_stateIn && seedBoots>0){
    struct SeedRec{ uint32_t magic; uint8_t phase; uint16_t boots; float gbx,gby,gbz; float groundAlt; uint32_t sum; } r{};
    EEPROM.get(0,r);
    r.boots=(uint16_t)seedBoots;
    const uint8_t* b=(const uint8_t*)&r; uint32_t h=2166136261u;
    for(size_t i=0;i<sizeof(SeedRec)-sizeof(uint32_t);i++){ h^=b[i]; h*=16777619u; }
    r.sum=h; EEPROM.put(0,r);
    fprintf(stderr,"[patched EEPROM boots=%d (phase=%d)]\n",seedBoots,(int)r.phase);
  }

  int why = setjmp(endJmp);
  if(why==0){ setup();
    if(pg>=0) P_GAIN=(float)pg; if(dg>=0) D_GAIN=(float)dg; if(ig>=0) I_GAIN=(float)ig;
    if(wn>=0) TVC_WN=(float)wn;  if(zt>=0) TVC_ZETA=(float)zt;
    if(fixedGain) TVC_TORQUE_CMD=false;      // A/B the legacy fixed-gain law in the same binary
    loop(); why=3; }
  if(why==7){                    // MCU reset: the snapshot is written, the caller relaunches with --statein
    printf("outcome=MCU_RESET   reset_t=%.2f alt=%6.1f vz=%6.1f boot=%d\n",
           g_micros/1e6, alt, vz, g_bootCount+1);
    return 7;
  }

  // classify on the BOOST-phase TRUE tilt (the control test)
  const char* outcome;
  if(!p4Fired || why==2)                 outcome="NORECOVERY";
  else if(mtBoost>=44.0)                 outcome="ABORT_TUMBLE";   // boost tilt hit the firmware's 45 deg emergency
  else if(apogee<3.0)                    outcome="NOFLY";
  else if(mtBoost<10.0)                  outcome="PASS";
  else                                   outcome="MARGINAL";
  // p4t/p4alt/p4vz are reported because "did the chute come out" is not the same question as "did it
  // come out in time" -- ASC038's canopy appeared ~1.5 m above the ground, which p4descent alone calls
  // a success. p4t=-1 means the chute NEVER fired, which is the failure worth shouting about.
  printf("outcome=%-12s boostTilt=%6.2f boostRate=%6.1f endlean=%6.2f coastTilt=%6.2f apogee=%6.1f p4descent=%d "
         "p4t=%6.2f p4alt=%6.1f p4vz=%6.1f "
         "keff_true=%6.1f keff_est=%6.1f tscale=%.3f burnfrac=%.3f "
         "roll=%6.0f rollrate=%6.0f "
         "iyy=%.2f izz=%.2f thrust=%.2f larm=%.2f rollcant=%.5f cgoffx=%.4f cgoffy=%.4f misx=%.2f misy=%.2f sm=%.4f "
         "slew=%.0f pgain=%.3f dgain=%.3f igain=%.3f seed=%u\n",
         outcome,mtBoost,mrBoost,boostEndTilt,maxTilt,apogee,(int)p4AtDescent,
         p4Fired?p4T:-1.0, p4Fired?p4Alt:-1.0, p4Fired?p4Vz:0.0,
         keffMidTrue, keffMidEst,
         tscaleMid, (double)burnFrac,
         maxRollAng,maxRollRate,
         d_iyy,d_izz,d_thrust,d_L,d_rollCant,d_cgOffX,d_cgOffY,d_misX,d_misY,d_sm,
         d_servoSlew,(double)P_GAIN,(double)D_GAIN,(double)I_GAIN,d_seed);
  return 0;
}








