/*
  Sysiphus_Landing.cpp  —  Propulsive-landing GNC flight firmware (Teensy 4.x)
  =============================================================================
  NEW-ROCKET build. Faithful port of the validated landing simulator
  (tools/landing_interactive.py): pad calibration, per-axis INS/GPS Kalman
  filter, acceleration-commanded ADRC, the sim's TWO-ACTUATOR control scheme
  (TVC gimbal + forward MARGIN FINS), causal suicide-burn ignition, and the
  full landing sequence.

  ACTUATORS (exactly the sim's two control channels, single control plane = pitch):
    * TVC gimbal servo  — active under thrust (BOOST, LANDING_BURN)
    * Margin-fin servo  — active during unpowered COAST (modulates static margin /
                          CP to make an aerodynamic control moment) -> this is what
                          lets the rocket reorient to RETROGRADE during coast, the
                          way the sim does. One servo => one control plane.

  FLIGHT ARCHITECTURE (matches the sim):
    PAD_ALIGN -> ARMING -> COUNTDOWN
    -> BOOST        : ascent motor (P_ASCENT); TVC ADRC holds the gravity-turn/kick
    -> COAST        : unpowered; MARGIN-FIN ADRC slews attitude to RETROGRADE and
                      holds it (aero authority grows with airspeed)
    -> LANDING_BURN : causal suicide burn (P_LAND) so the EKF descent velocity is
                      nulled at the ground; TVC ADRC holds retrograde + diverts to
                      target; deploy legs (P_LEGS)
    -> TOUCHDOWN
    ABORT (any phase) -> deploy chute (P_CHUTE) + neutral actuators

  WHY THE EKF MATTERS: the hoverslam is timed off descent velocity, and a low-FSR
  accelerometer rails during the burn. The KF fuses the barometer (+GPS) so the
  vertical-velocity estimate stays good through the burn. Set the IMU to +/-16 g.

  ====================== READ BEFORE FLYING ======================
  * NOT compile-tested here and NEVER hardware-tested. Compile, then dry-run/bench
    test EVERY phase before propellant. Faithful port of sim logic, not proven code.
  * Single CONTROL PLANE (pitch), exactly like the 2D sim. A real free-flight rocket
    also needs the YAW plane: duplicate the ESO/inversion on gyroY + a 2nd TVC axis,
    and rely on aero stability for yaw during coast. Flagged, not implemented.
  * // <<< SET  = fill from bench/CFD/sim for YOUR rocket.   // <<< MISSION = from the sim run.
  * Positive ASCENT static margin is a prerequisite the code can't fix.
  ================================================================
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ============================================================================
//  CONFIG
// ============================================================================
constexpr float G          = 9.80665f;
constexpr float RHO        = 1.20f;             // air density (kg/m^3)
constexpr float DT_S       = 0.005f;            // 200 Hz sensor loop
constexpr float CONTROL_HZ = 100.0f;            // 100 Hz control + EKF
constexpr float DT_CTRL    = 1.0f / CONTROL_HZ;
constexpr uint32_t SENSOR_PERIOD_MS  = (uint32_t)(DT_S*1000.0f+0.5f);
constexpr uint32_t CONTROL_PERIOD_MS = (uint32_t)(1000.0f/CONTROL_HZ+0.5f);

// ---- Rocket ----
constexpr float IYY        = 0.012f;            // lateral (pitch=yaw) inertia (kg m^2)  [SIL default rocket] // <<< SET
constexpr float IZZ_ROLL   = 1.8e-4f;           // ROLL inertia (kg m^2): slender body ~m*r^2/2, ~65x SMALLER than
                                                // IYY -- which is why tiny roll torques (TVC/CG misalignment)
                                                // wind up fast. Roll divergence is a first-class failure mode. // <<< SET
constexpr float KEFF_NOM   = 8.0f;              // TVC effectiveness (rad/s^2 per cmd unit), same both gimbal axes // <<< SET
constexpr float U_MAX      = 12.0f;             // TVC command saturation (= full gimbal)
constexpr float TVC_DEG_PER_UNIT = 2.0f;        // servo deg per command unit
constexpr int   TVC_NEUTRAL_DEG  = 90;

// ---- Margin fins (aero control authority for coast) ----
// FOLD-OUT canard pair on one servo: deploy=0 folded flat (max nose-first stability) -> 1 fully out 90 deg
// (max destabilization). A symmetric fold can only SHIFT THE CP: its moment is aoa-proportional (zero at
// aoa 0/180, sign follows aoa) and referenced to the MIGRATING CG -- so the stability-neutral deploy is NOT
// a fixed 50%: it moves as propellant burns (CG forward) and legs deploy (CG aft). The firmware schedules it
// from the known mass state (depNeutralCoast) instead of assuming a fixed neutral.
constexpr float S_REF      = 0.0030f;           // body reference area (m^2)         // <<< SET (CFD)
constexpr float S_FIN      = 0.0009f;           // fin pair area, fully deployed (m^2) // <<< SET (CFD)
constexpr float CN_FIN     = 2.0f;              // fin normal-force slope             // <<< SET (CFD)
constexpr float L_FIN0     = 0.30f;             // fin CP forward of NOMINAL CG (m)   // <<< SET (CFD)
constexpr float CN_BODY    = 2.0f;              // bare-body normal-force slope       // <<< SET (CFD)
constexpr float L_BODYCP0  = 0.033f;            // body CP aft of NOMINAL CG (m): near-neutral airframe // <<< SET
constexpr float CG_PROP_M  = 0.009f;            // CG forward shift per motor's (aft) propellant burnt (m) // <<< SET
constexpr float CG_LEGS_M  = 0.030f;            // CG aft shift when the legs deploy (m)                   // <<< SET
constexpr float DRAG_CD    = 0.6f;             // body axial drag coeff on S_REF (CFD) - used by ignition + fin-authority // <<< SET
constexpr int   MARGIN_NEUTRAL_DEG = 90;        // servo angle at deploy=0.5
constexpr float MARGIN_DEG_RANGE   = 35.0f;     // +/- deg of servo travel about that (deploy 0..1)
// ---- Roll control (differential margin-fin fold) ----
// The fold-out pair doubles as the ROLL actuator: a DIFFERENTIAL fold turns crossflow into roll torque,
// authority ~ q*S_FIN*CN_FIN*(vlat/V)*R_FIN -- proportional to sin(aoa), so it's strong exactly at HIGH aoa
// (reorientation transients, wind) and ~zero pointing along the flow. Needed because TVC cannot command roll,
// yet a lateral CG/thrust misalignment makes every TVC deflection LEAK roll torque (M_z = T*(cgy*da - cgx*db))
// into a roll inertia ~65x smaller than pitch -- classic TVC roll divergence.
constexpr float R_FIN_M    = 0.030f;            // fin center radial offset from the roll axis (m)    // <<< SET
constexpr int   ROLL_NEUTRAL_DEG = 90;          // differential servo neutral
constexpr float ROLL_DEG_RANGE   = 20.0f;       // +/- deg differential at rollDef +-0.5
constexpr float WC_ROLL = 5.0f, W0_ROLL = 20.0f;// roll-rate ESO bandwidth (rate loop only; roll ANGLE is free)
constexpr float B0_ROLL_MIN = 3.0f;             // rad/s^2/unit below which the fins can't bite -> hold 0

// ---- Motors ----
constexpr float ASCENT_BURN_S     = 3.45f;      // ascent motor burn (s)
constexpr float LAND_THRUST_AVG_N = 20.0f;      // landing-motor avg thrust during decel (N) [SIL default] // <<< SET
constexpr float MASS_LAND_KG      = 0.80f;      // NOMINAL all-up mass at landing burn (kg) [SIL default]  // <<< SET
constexpr float LAND_PROP_KG      = 0.03f;      // landing-motor propellant burnt (kg): mass drops this much over the burn // <<< SET
constexpr float ASC_PROP_KG       = 0.03f;      // ascent-motor propellant (kg): used by the in-flight mass estimator // <<< SET
constexpr float IGN_LEAD_S        = 0.025f;     // KNOWN mean pyro->thrust dead time (s), from static fire. The motor
                                                // doesn't light instantly; ignitionReached fires this much EARLIER
                                                // (projects the free-fall during spin-up) so the burn BEGINS where the
                                                // predictor wants. Uncompensated, a 25 ms delay = ~0.6 m late at 24 m/s
                                                // = under-brake (SIL: vz 3.8->6.2). Scatter around it is irreducible. // <<< SET
// Landing-motor thrust CURVE (from static-fire / datasheet). A solid motor cannot be throttled or cut, so the
// hoverslam is timed off the ACTUAL curve (a flat-average braking-distance model over- or under-brakes because the
// curve is front-loaded). Shape mirrors the SIL motor: ramp to peak by 0.15 s, peak->plateau by 0.50 s, plateau.
constexpr float LAND_BURN_S=2.0f, LAND_PLATEAU_N=18.0f, LAND_PEAK_N=28.0f;   // <<< SET (your motor)
// Ascent-motor thrust CURVE (same datasheet form), used by the wind-adaptive ascent guidance to forward-predict
// where a given boost aim will land under the estimated wind, and steer the kick to hit the target.  // <<< SET
constexpr float ASC_BURN_S=3.45f, ASC_PLATEAU_N=12.0f, ASC_PEAK_N=22.0f;

// ---- ADRC (from the validated sim) ----
// ADRC bandwidth MUST be matched to the SERVO SLEW RATE: the controller can't command the gimbal faster than the
// servo can move. At the 150 deg/s servo the tight 6/30 flies cleanly. A SLOW ~60 deg/s servo would need these
// detuned to ~2/8 (flies but tracks the wind-guidance poorly -> several-m misses); the SIL `WC`/`W0` env knobs
// sweep this. Threshold: 6/30 needs >=~100 deg/s or it slew-saturates and tumbles in the margin-fin coast.  // <<< SET
float ADRC_WC = 6.0f, ADRC_W0 = 30.0f;   // TVC phases (non-const: SIL can sweep via WC/W0 env)
// COAST bandwidth is separate and LOW: the margin fin delivers ~1-2 rad/s^2 (vs TVC's ~100) through a
// ~0.5 s-travel servo. Driving it at WC=6 demands ~20x the available authority -> the command lives on the
// 0/1 rails and CHATTERS as the error/noise flips sign; the real servo lowpasses that chatter to ~neutral
// and the vehicle coasts on pure momentum (the headwind capture-divergence). Bandwidth matched to actuator.
constexpr float WC_COAST = 2.0f, W0_COAST = 10.0f;
constexpr float KEFF_MIN = 1.2f, KEFF_MAX = 44.0f;
// Actuator observers must reflect the REAL servo slew (~150 deg/s), or the ESO anti-windup believes commands
// it never got: TVC 150/2 deg-per-unit = 75 u/s; margin 150/(2*35 deg full range) = ~2.1 fraction/s.
constexpr float SERVO_SLEW_DPS = 150.0f;                                            // <<< SET (your servo)

// ---- RLS keff identification (TVC b0) ----
constexpr float AERO_DAMP=1.20f, LAMBDA_RLS=0.97f, ADAPT_GUARD_S=0.50f, DELTA_MIN=0.10f, KEFF_BETA=0.10f;

// ---- Sensors / EKF noise (tune to your parts) ----                               // <<< SET
constexpr float ACCEL_NOISE=0.05f, BARO_NOISE=0.30f, GPS_POS_NOISE=0.8f, GPS_HZ=8.0f, Q_BIAS=1e-7f;

// ---- Suicide burn / touchdown ----
constexpr float DESCENT_VZ_MIN=1.0f, TD_ALT_M=0.15f, TD_VZ_M=0.6f;
// Baro transport lag (s): a BMP280 reading represents the altitude ~this long ago (first-order conversion
// lag ~30 ms + half the 50 Hz output-hold ~10 ms). At 20+ m/s descent that is ~1 m of stale altitude. The
// EKF LEAD-COMPENSATES the MEASUREMENT at fusion (meas + estVz*BARO_LAG_S) so estZ itself tracks the true
// present altitude -- every consumer (hoverslam ignition, TD gate, telemetry) then sees a current estimate.
// Don't inflate this "to be safe": overcompensating reads LOW by v*excess -> the hoverslam ignites early ->
// arrests above the deck -> the un-cuttable solid relaunches (hop) -> harder impact than igniting late.  // <<< SET
constexpr float BARO_LAG_S=0.040f;
// GPS fix latency (s): an NMEA fix reports where the vehicle WAS ~this long ago (receiver solve + serial).
// Lead-compensated the same way at fusion (gpsX + estVx*GPS_LAT_S) so estX doesn't trail by vx*latency.  // <<< SET
constexpr float GPS_LAT_S=0.20f;
// ---- Retrograde attitude command shaping (coast + burn) ----
// Cap how far off vertical we ever command, and blend the retro target in by DESCENT SPEED so
// the near-horizontal, ill-conditioned atan2 at apogee (V->0) can never step-command a large tilt
// that saturates the weak coast fin and tumbles the vehicle.
constexpr float RETRO_MAX_RAD=0.61f;                 // ~35 deg tilt cap
// retrograde target (world tilt X/Y components, rad), speed-blended and cone-limited; 0 while ascending/near apogee.
static inline void retroTargetXY(float vx,float vy,float vz,float&tx,float&ty){
  tx=0; ty=0;
  if(vz>=-0.5f) return;                               // ascending or at apogee -> hold vertical
  float V=sqrtf(vx*vx+vy*vy+vz*vz);
  float rx=atan2f(-vx,-vz), ry=atan2f(-vy,-vz);
  float m=sqrtf(rx*rx+ry*ry);
  if(m>RETRO_MAX_RAD){ rx*=RETRO_MAX_RAD/m; ry*=RETRO_MAX_RAD/m; }   // cone cap on the TOTAL tilt
  float wspd=constrain((V-2.0f)/4.0f,0.0f,1.0f);      // 0 below 2 m/s, full above 6 m/s
  tx=rx*wspd; ty=ry*wspd;
}

// ---- Mission profile (generate from the sim for your rocket+target) ----          // <<< MISSION
constexpr float KICK_DEG = 0.0f;                // ascent gravity-turn kick (0 = vertical)
constexpr int   NOM_N = 0;                      // nominal descent x(z) table size (0 = divert off)
const float NOM_Z[1]={0}, NOM_X[1]={0};         // altitude knots (descending) / target x

// ---- Calibration / countdown / safety ----
constexpr int   CAL_SAMPLES=600;
constexpr float COUNTDOWN_S=10.0f, SEA_LEVEL_HPA=1013.25f;                          // <<< SET pressure
constexpr float ABORT_TILT_DEG=80.0f, ABORT_RATE_DPS=400.0f;                        // wide: a flip is normal here
constexpr float ABORT_ROLL_DPS=720.0f;          // roll-rate abort: past this the fins can't catch it and the
                                                // lateral control frame is spinning faster than the loop tracks
constexpr float CD_TILT_ABORT_DEG=5.0f, CD_RATE_ABORT_DPS=25.0f;

// ---- GPS (NMEA over Serial1) ----
#define GPS_SERIAL Serial1
constexpr uint32_t GPS_BAUD=9600; constexpr bool USE_GPS=true;                      // false until wired // <<< SET

// ---- Pins (new board; set to your layout) ----                                   // <<< SET
constexpr int BUTTON=14, BUZZER=13, RLED=6, GLED=7, BLED=8;
constexpr int P_LEGS=9, P_LAND=10, P_ASCENT=11, P_CHUTE=12;
constexpr int SERVO_TVC_PIN=4, SERVO_TVC2_PIN=5, SERVO_MARGIN_PIN=3, SERVO_ROLL_PIN=2, SD_CS=BUILTIN_SDCARD;

// ============================================================================
//  STATE
// ============================================================================
enum class St:uint8_t { BOOT,PAD_ALIGN,ARMING,COUNTDOWN,BOOST,COAST,LANDING_BURN,TOUCHDOWN,ABORTED };
enum class Ab:uint8_t { NONE,SENSOR,CD_MOVED,TILT,RATE,MANUAL,APOGEE_TO };
enum class Act:uint8_t { NONE,TVC,MARGIN };
St state=St::BOOT; Ab abortReason=Ab::NONE;

PWMServo servoTVC, servoTVC2, servoMargin, servoRoll;
MPU6050 mpu(Wire); Adafruit_BMP280 bmp; File logFile; char logName[20];

// ---- ATTITUDE: full 3-axis quaternion (body->world), gyro-integrated, pad-calibrated. Body frame: +Z = nose,
// X/Y lateral. This is what makes control ROLL-CORRECT: the vehicle may roll freely (TVC can't stop it) and the
// lateral control axes rotate with it -- world-frame tilt control mapped through the measured roll stays valid.
float qw=1, qx=0, qy=0, qz=0;                   // attitude quaternion
float gxr=0, gyr=0, gzr=0;                      // bias-corrected body rates (rad/s): X pitch, Y yaw, Z ROLL
float tiltX=0, tiltY=0, rollTwist=0;            // body-axis tilt components (world) + twist (roll) about the body axis
float th=0, gBias=0, q=0;                       // legacy: th = tiltX, q = gxr (logs/harness externs)
float gBiasY=0, gBiasZ=0;
float aBiasX=0, aBiasY=0, aBiasZ=0;             // body accel biases (lateral X/Y, axial)
float sfBx=0, sfBy=0, sfBz=0;                   // latest body specific force
float sfBxLP=0, sfByLP=0;                      // low-passed SIGNED lateral aero force per body axis
                                               // (coast: m*sf_lat = q*S*CD*sin(aoa)*direction -> wind-agnostic SIGNED
                                               //  aoa-vector proxy: margin-fin authority magnitude AND direction)
// ADRC ESOs [attitude, rate, disturbance]: one per WORLD tilt axis + a 2-state roll-RATE ESO
struct ESO{ float z1,z2,z3; };
ESO esoX={0,0,0}, esoY={0,0,0};                 // lateral attitude (world tilt X / Y components)
ESO esoR={0,0,0};                               // roll: z1=rate, z2=disturbance (z3 unused)
float u_tvc=0,  u_tvc_act=0;                    // TVC X-plane command / realized (units)
float u_tvc2=0, u_tvc2_act=0;                   // TVC Y-plane command / realized
float deploy=0.0f, deploy_act=0.0f;            // margin fold-out fraction / realized (0 = stowed)
float rollDef=0.0f, rollDef_act=0.0f;          // differential fin fold (+-0.5) / realized
// RLS keff (TVC b0; keff is the same for both gimbal axes -- identified on the X channel's excitation)
float keff_est=KEFF_NOM, S_uu=1, S_uy=KEFF_NOM, alpha_lp=0, adaptT=0;
float massScale=1.0f;                           // true all-up mass / nominal, identified from boost thrust/accel (see ctrlBoost)
// per-axis KF [pos,vel,accel-bias]
struct KF{ float s[3]; float P[9]; }; KF kfZ,kfX,kfY;
float baroAlt0=0, lastAccZ=-G, lastAccX=0, lastAccY=0;
float windEst=0, windEstY=0; bool windLocked=false;   // in-flight wind estimate (m/s, +x/+y) / frozen at ignition
float estZ=0,estVz=0,estX=0,estVx=0,estY=0,estVy=0;
float thRetro=0, thRetroY=0;                    // retrograde attitude target, world tilt X/Y components
// Mission (runtime-settable so a test harness / config can vary the shot without recompiling)
float g_kickDeg = KICK_DEG;                     // ascent gravity-turn kick (deg) -> nominal downrange aim (pre-flight)
float g_kickCmd = KICK_DEG;                     // LIVE commanded kick (deg, world X): wind-adaptive guidance steers it
float g_kickCmdY = 0.0f;                        // LIVE crossrange aim (deg, world Y): nulls the sensed CROSSwind drift
float g_targetX = 0.0f;                         // target downrange (m); 0 = land on pad
// -- LEGACY position-divert (UNUSED). The terminal descent is now PURE RETROGRADE and downrange accuracy is
// handled by wind-adaptive ASCENT guidance (see ctrlBoost), so divertOffset()/nomInterp()/the nominal table are
// no longer called. Retained (compiles, inert) in case a position-divert is ever wanted again; safe to delete.
constexpr float WCX=1.6f, W0X=6.0f, STILT2=0.259f;   // (legacy divert ESO bw / tilt cap)
int   g_nomN=0; float g_nomZ[48], g_nomX[48], g_nomVx[48];   // (legacy) nominal descent trajectory
float zx1=0,zx2=0,zx3=0; bool txInit=false;                  // (legacy) translational ESO state
bool  g_divertOn=true;   // GUIDANCE gate: false (via --nodivert) = fly the fixed kick ballistic, used for the
                         //   pre-flight calm-kick bisection; true = wind-adaptive ascent guidance active
// In-flight boost mass ID for the hoverslam. DEFAULT OFF: boost accel gives only thrust/MASS (the two aren't
// separable), so an ascent-motor lot deviation gets mis-attributed to mass and POISONS the hoverslam. Since the
// airframe is WEIGHED on the ground (set MASS_LAND_KG), identifying mass in flight trades a solved problem for an
// unsolvable one -- SIL Monte Carlo: 32% soft-pass OFF vs 10% ON under realistic motor dispersion. Enable only if
// mass is genuinely unknown pre-flight (--massid in the SIL).
bool  g_massIdOn=false;
// GPS
double gpsLat=0,gpsLon=0,lat0=0,lon0=0; bool gpsFix=false,gpsOrigin=false;
float gpsX=0, gpsY=0; char nmea[84]; uint8_t nmeaIdx=0;
// runtime
bool inFlight=false;
uint32_t stateMs=0,ignMs=0,lastSensMs=0,lastCtrlMs=0,lastLogMs=0,lastBtnMs=0,lastArmMs=0;
int armCount=0; bool sensorsOk=false; int cdLast=-1; uint32_t cdMoveMs=0;
float coastPeak=0; uint32_t coastMs=0; bool apogee=false; bool legsFired=false;

struct Pyro{ int pin; bool on; uint32_t t0,dur; };
Pyro pyros[]={{P_LEGS,false,0,900},{P_LAND,false,0,900},{P_ASCENT,false,0,900},{P_CHUTE,false,0,900}};

// ============================================================================
//  UTILITIES
// ============================================================================
void LED(bool r,bool g,bool b){ digitalWrite(RLED,!r); digitalWrite(GLED,!g); digitalWrite(BLED,!b); }
void beep(uint16_t f,uint16_t d){ tone(BUZZER,f); delay(d); noTone(BUZZER); }
const char* stName(St s){ const char* n[]={"BOOT","PAD","ARM","CD","BOOST","COAST","BURN","TD","ABORT"}; return n[(int)s]; }
const char* abName(Ab a){ const char* n[]={"NONE","SENSOR","CD_MOVED","TILT","RATE","MANUAL","APOGEE_TO"}; return n[(int)a]; }
void setState(St n){ if(state==n)return; state=n; stateMs=millis(); Serial.print(F("STATE -> ")); Serial.println(stName(n)); }
static float wrapPi(float a){ while(a>M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; }

void updatePyros(){ uint32_t now=millis(); for(auto&c:pyros) if(c.on&&now-c.t0>=c.dur){ digitalWrite(c.pin,LOW); c.on=false; } }
void firePyro(int pin,uint32_t dur=900){ for(auto&c:pyros) if(c.pin==pin&&!c.on){ digitalWrite(pin,HIGH); c.on=true; c.t0=millis(); c.dur=dur; return; } }

void writeTVCServo(PWMServo& s, float u){
  float d=constrain(TVC_NEUTRAL_DEG-u*TVC_DEG_PER_UNIT,
                    TVC_NEUTRAL_DEG-U_MAX*TVC_DEG_PER_UNIT, TVC_NEUTRAL_DEG+U_MAX*TVC_DEG_PER_UNIT);
  s.write((int)(d+0.5f));
}
void writeTVC(float ua,float ub){ writeTVCServo(servoTVC,ua); writeTVCServo(servoTVC2,ub); }
void writeMargin(float dep){ servoMargin.write((int)(MARGIN_NEUTRAL_DEG+(dep-0.5f)*2.0f*MARGIN_DEG_RANGE+0.5f)); }
void writeRoll(float rd){ servoRoll.write((int)(ROLL_NEUTRAL_DEG+constrain(rd,-0.5f,0.5f)*2.0f*ROLL_DEG_RANGE+0.5f)); }
void neutralActuators(){ writeTVC(0,0); writeMargin(0.0f); writeRoll(0.0f);
  u_tvc=0; u_tvc2=0; deploy=0.0f; rollDef=0.0f; }   // fins STOWED (folded = max static stability)

bool buttonEdge(){
  if(digitalRead(BUTTON)==HIGH && millis()-lastBtnMs>40){ lastBtnMs=millis();
    while(digitalRead(BUTTON)==HIGH){ updatePyros(); delay(1); } return true; }
  return false;
}

// ============================================================================
//  3x3 KALMAN FILTER (state [pos,vel,accel-bias]) — ported from the sim
// ============================================================================
static void m3mul(const float*A,const float*B,float*C){
  for(int i=0;i<3;i++)for(int j=0;j<3;j++){ float s=0; for(int k=0;k<3;k++)s+=A[i*3+k]*B[k*3+j]; C[i*3+j]=s; } }
// accel-bias variance TIGHT (pad cal removed it) so the KF won't re-estimate it and absorb the boost
// thrust transient -- that wind-up was corrupting the coast/descent velocity estimate.
void kfInit(KF&k,float p0){ k.s[0]=p0;k.s[1]=0;k.s[2]=0; for(int i=0;i<9;i++)k.P[i]=0; k.P[0]=0.04f; k.P[4]=0.04f; k.P[8]=0.01f; }
void kfPredict(KF&k,float a,float dt){      // dt = ACTUAL elapsed step (ekfStep runs at the sensor rate, not CONTROL_HZ)
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

// ============================================================================
//  GPS (minimal NMEA GGA -> local east-meters from pad origin)
// ============================================================================
double nmeaDeg(const char* f,char h){ double v=atof(f); int d=(int)(v/100); double m=v-d*100,dd=d+m/60.0; if(h=='S'||h=='W')dd=-dd; return dd; }
void parseGGA(char* s){
  char* tok[8]; int n=0;
  for(char* p=s; *p&&n<8; p++) if(*p==','){ *p=0; tok[n++]=p+1; }
  if(n<6) return;
  if(atoi(tok[5])<1){ gpsFix=false; return; }
  double lat=nmeaDeg(tok[1],tok[2][0]), lon=nmeaDeg(tok[3],tok[4][0]);
  gpsLat=lat; gpsLon=lon; gpsFix=true;
  if(!gpsOrigin){ lat0=lat; lon0=lon; gpsOrigin=true; }
  gpsX=(float)((lon-lon0)*cos(lat0*M_PI/180.0)*111320.0);     // east (m), world X (downrange)
  gpsY=(float)((lat-lat0)*111320.0);                          // north (m), world Y (crossrange)
}
void serviceGPS(){
  if(!USE_GPS) return;
  while(GPS_SERIAL.available()){ char c=GPS_SERIAL.read();
    if(c=='$'){ nmeaIdx=0; nmea[nmeaIdx++]='$'; }
    else if(nmeaIdx>0 && nmeaIdx<sizeof(nmea)-1){
      if(c=='\r'||c=='\n'){ nmea[nmeaIdx]=0; if(strstr(nmea,"GGA")) parseGGA(nmea); nmeaIdx=0; }
      else nmea[nmeaIdx++]=c; }
  }
}

// ============================================================================
//  SENSORS + PAD CALIBRATION
// ============================================================================
bool initSensors(){
  Wire.begin(); Wire.setClock(400000);
  if(!(bmp.begin(0x76)||bmp.begin(0x77))){ Serial.println(F("BMP280 not found")); return false; }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_1);
  mpu.begin();
  // Set the accelerometer to +/-16 g so the landing burn does not rail it.   // <<< SET (FSR register 0x1C)
  mpu.calcGyroOffsets(true);
  return true;
}
void padCalibrate(){
  double g0=0,g1=0,g2=0,ax=0,ay=0,az=0;
  for(int i=0;i<CAL_SAMPLES;i++){ mpu.update();
    g0+=mpu.getGyroX(); g1+=mpu.getGyroY(); g2+=mpu.getGyroZ();
    ax+=mpu.getAccX();  ay+=mpu.getAccY();  az+=mpu.getAccZ(); delay(3); }
  gBias =(float)(g0/CAL_SAMPLES)*(M_PI/180.0f);
  gBiasY=(float)(g1/CAL_SAMPLES)*(M_PI/180.0f);
  gBiasZ=(float)(g2/CAL_SAMPLES)*(M_PI/180.0f);
  aBiasX=(float)(ax/CAL_SAMPLES)*G;
  aBiasY=(float)(ay/CAL_SAMPLES)*G;
  aBiasZ=(float)(az/CAL_SAMPLES)*G-G;            // axial reads +1g on the pad
  qw=1; qx=0; qy=0; qz=0; th=0; tiltX=0; tiltY=0; rollTwist=0;   // pad = vertical, roll reference zero
  baroAlt0=bmp.readAltitude(SEA_LEVEL_HPA);
  Serial.print(F("PAD CAL gBias ")); Serial.print(gBias,4); Serial.print(F(" aBias ")); Serial.print(aBiasX,3); Serial.print(','); Serial.println(aBiasZ,3);
}
void readIMU(float& sfx,float& sfz,float dt){
  mpu.update();
  gxr=mpu.getGyroX()*(M_PI/180.0f)-gBias;
  gyr=mpu.getGyroY()*(M_PI/180.0f)-gBiasY;
  gzr=mpu.getGyroZ()*(M_PI/180.0f)-gBiasZ;
  if(inFlight){
    // quaternion kinematics on the MEASURED elapsed dt (loop-stall robust): qdot = 0.5*q (x) (0, w_body)
    float dqw=0.5f*(-qx*gxr-qy*gyr-qz*gzr), dqx=0.5f*( qw*gxr+qy*gzr-qz*gyr),
          dqy=0.5f*( qw*gyr-qx*gzr+qz*gxr), dqz=0.5f*( qw*gzr+qx*gyr-qy*gxr);
    qw+=dqw*dt; qx+=dqx*dt; qy+=dqy*dt; qz+=dqz*dt;
    float n=sqrtf(qw*qw+qx*qx+qy*qy+qz*qz); if(n>1e-6f){qw/=n;qx/=n;qy/=n;qz/=n;}
  }
  // derive: body nose axis in world (tilt components, roll-invariant) + twist (roll) about the body axis
  float zbx=2*(qx*qz+qw*qy), zby=2*(qy*qz-qw*qx), zbz=1-2*(qx*qx+qy*qy);
  tiltX=atan2f(zbx,zbz); tiltY=atan2f(zby,zbz);
  rollTwist=2.0f*atan2f(qz,qw);                   // swing-twist decomposition: rotation about the body axis
  th=tiltX; q=gxr;                                // legacy exports
  sfx=mpu.getAccX()*G-aBiasX; sfz=mpu.getAccZ()*G-aBiasZ;
  sfBx=sfx; sfBy=mpu.getAccY()*G-aBiasY; sfBz=sfz;
  sfBxLP += 0.15f*(sfBx-sfBxLP);                 // low-pass the SIGNED lateral aero force per axis
  sfByLP += 0.15f*(sfBy-sfByLP);                 //   (noisy near apogee)
}

// ============================================================================
//  EKF (strapdown predict + baro/GPS update) — single horizontal plane (x) + vertical (z)
// ============================================================================
void updateWind();
void ekfStep(float sfx,float sfz,float dt){
  // rotate the body specific force to world with the FULL quaternion (roll-correct strapdown)
  float bx=sfBx, by=sfBy, bz=sfBz;
  float r11=1-2*(qy*qy+qz*qz), r12=2*(qx*qy-qw*qz), r13=2*(qx*qz+qw*qy);
  float r21=2*(qx*qy+qw*qz), r22=1-2*(qx*qx+qz*qz), r23=2*(qy*qz-qw*qx);
  float r31=2*(qx*qz-qw*qy), r32=2*(qy*qz+qw*qx), r33=1-2*(qx*qx+qy*qy);
  float aWx = r11*bx+r12*by+r13*bz;
  float aWy = r21*bx+r22*by+r23*bz;
  float aWz = r31*bx+r32*by+r33*bz - G;           // gravity removed
  lastAccZ=aWz; lastAccX=aWx; lastAccY=aWy;
  kfPredict(kfZ,aWz,dt); kfPredict(kfX,aWx,dt); kfPredict(kfY,aWy,dt);
                                                  // integrate on MEASURED elapsed dt, not a fixed nominal --
                                                  // a stalled loop (SD/I2C) makes the real step > nominal; a fixed
                                                  // dt would silently lose time and mistime the hoverslam.
  // Lead-compensate the SENSOR transport lags at fusion: the baro reading is the altitude BARO_LAG_S ago and a
  // GPS fix is the position GPS_LAT_S ago, so fusing them raw drags the estimate ~v*lag behind truth (at 24 m/s
  // descent estZ read ~1.1 m high; at a few m/s cross-range estX trailed ~0.5-1 m). Propagate each measurement
  // to "now" with the KF's own velocity (accel-integrated, essentially lag-free) before the update. On the pad
  // v~0 so this is a no-op; in flight the estimate tracks the true present state.
  kfUpdate(kfZ,0,bmp.readAltitude(SEA_LEVEL_HPA)-baroAlt0 + estVz*BARO_LAG_S,BARO_NOISE*BARO_NOISE);
  if(USE_GPS && gpsFix && gpsOrigin){ static uint32_t lu=0;
    if(millis()-lu>=(uint32_t)(1000.0f/GPS_HZ)){ lu=millis();
      kfUpdate(kfX,0,gpsX + estVx*GPS_LAT_S,GPS_POS_NOISE*GPS_POS_NOISE);
      kfUpdate(kfY,0,gpsY + estVy*GPS_LAT_S,GPS_POS_NOISE*GPS_POS_NOISE); } }
  estZ=kfZ.s[0]; estVz=kfZ.s[1]; estX=kfX.s[0]; estVx=kfX.s[1]; estY=kfY.s[0]; estVy=kfY.s[1];
  if(estZ>coastPeak) coastPeak=estZ;
  updateWind();
}

// In-flight horizontal WIND estimate. During UNPOWERED descent the only horizontal force is aero drag along
// the AIR-relative velocity, so the sensed world horizontal accel is  aWx = -kd*Vair*(estVx - w), with
// kd = 0.5*rho*S_REF*CD/m and Vair the air-relative speed (descent-dominated, ~|estVz|). Invert for the wind w
// and slowly low-pass it (the per-sample drag signal ~ the accel-noise floor, but 200 Hz over the ~2 s coast
// averages it down). Frozen at ignition (windLocked). Telemetry/diagnostic: the position divert already crabs
// toward the nominal for whatever wind is actually present, so this is logged (post-flight wind truth, feed-forward
// research) rather than fed into control -- attempts to use it for gating or coast feed-forward didn't pay off
// (the drift is coast-accumulated and coast has no horizontal authority; see DESIGN_LOG).
void updateWind(){
  if(windLocked) return;
  const float kd = 0.5f*RHO*S_REF*DRAG_CD/MASS_LAND_KG;
  float wInstX, wInstY;
  if(state==St::BOOST && estVz>6.0f){
    // POWERED ascent: aW_lat = thrust*tilt/m - kd*V*(estV_lat-w) per world axis. Infer thrust from the VERTICAL
    // accel balance (incl. vertical drag so the lateral residual isn't biased); the commanded lateral accel is
    // thrust/m * (nose-axis tilt component), the rest of the sensed lateral accel is drag -> invert for wind.
    float ctot=cosf(tiltX)*cosf(tiltY);              // ~cos(total tilt) for small angles
    float thrustOverM = (lastAccZ + G + kd*estVz*estVz)/max(0.5f,ctot);
    float vax = (thrustOverM*sinf(tiltX) - lastAccX)/(kd*max(estVz,5.0f));
    float vay = (thrustOverM*sinf(tiltY) - lastAccY)/(kd*max(estVz,5.0f));
    wInstX = estVx - vax; wInstY = estVy - vay;
  } else if(estVz < -6.0f){
    // UNPOWERED descent: only horizontal force is drag along v_air -> aW_lat = -kd*Vair*(estV_lat-w).
    float Vair = sqrtf((estVx-windEst)*(estVx-windEst)+(estVy-windEstY)*(estVy-windEstY)+estVz*estVz);
    wInstX = estVx + lastAccX/(kd*max(Vair,1.0f));
    wInstY = estVy + lastAccY/(kd*max(Vair,1.0f));
  } else return;                                     // near apogee / low speed -> no clean signal
  windEst  += 0.02f*(constrain(wInstX,-15.0f,15.0f)-windEst);    // slow LP toward the steady wind
  windEstY += 0.02f*(constrain(wInstY,-15.0f,15.0f)-windEstY);
}

// ============================================================================
//  RLS keff (TVC b0)
// ============================================================================
void updateKeff(){
  static float qp=0; adaptT+=DT_CTRL;
  // the u_tvc (X) channel torques about body +Y, so its rate is gyr -- NOT gxr (3-D axis pairing)
  float qdot=(gyr-qp)/DT_CTRL; qp=gyr;
  alpha_lp=0.75f*alpha_lp+0.25f*(qdot+AERO_DAMP*gyr);
  // FREEZE keff during the landing burn: b0 is reset to KEFF_NOM at ignition (~= true legs-out plateau
  // effectiveness). The RLS excitation here is tiny (small retrograde/divert u) and the AERO_DAMP term is
  // far larger than the real low-speed aero damping, so continued adaptation drives keff to a ~2x-low value
  // -> the ADRC over-commands TVC -> the terminal attitude oscillation. Hold b0 fixed through the burn.
  if(state==St::LANDING_BURN) return;
  if(adaptT>=ADAPT_GUARD_S && fabsf(u_tvc_act)>=DELTA_MIN){
    S_uu=LAMBDA_RLS*S_uu+u_tvc_act*u_tvc_act; S_uy=LAMBDA_RLS*S_uy+u_tvc_act*alpha_lp;
    float raw=constrain(S_uy/max(1e-6f,S_uu),KEFF_MIN,KEFF_MAX);
    keff_est=(1.0f-KEFF_BETA)*keff_est+KEFF_BETA*raw;
  }
}

// ============================================================================
//  ADRC (single plane). Active actuator + its effectiveness b0 depend on phase.
//  Returns the desired angular acceleration; the caller inverts to the actuator.
// ============================================================================
// Per-axis 3rd-order ESO + PD law. `meas` = the axis's WORLD tilt component (roll-invariant); `thRef` = the
// target tilt; `uA` = the REALIZED angular accel of `meas` (anti-windup). The output is mapped to the co-located
// body gimbal by the caller (w2b). NOTE: keep the (guidance-steered) target as the REFERENCE, never folded into
// `meas` -- a moving reference folded into the measurement looks like a disturbance and winds z3 up to divergence
// (learned the hard way attempting a body-frame-error variant of this loop, 2026-07-03).
float adrcAlphaDes(ESO& o,float meas,float thRef,float uA,float wc=0,float w0=0){
  if(wc<=0){ wc=ADRC_WC; w0=ADRC_W0; }            // default: the TVC-phase bandwidth
  float e=meas-o.z1;
  o.z1+=DT_CTRL*(o.z2+3.0f*w0*e);
  o.z2+=DT_CTRL*(o.z3+uA+3.0f*w0*w0*e);
  o.z3+=DT_CTRL*(w0*w0*w0*e);
  return wc*wc*wrapPi(thRef-o.z1) - 2.0f*wc*o.z2 - o.z3;
}
// WORLD-tilt <-> BODY-lateral mapping through the measured roll (swing-twist). The IMU and TVC gimbal are bolted
// to the same airframe, so the gimbal frame == the gyro frame == the body frame; this rotation IS that co-frame
// relationship, using the roll the co-located gyros measure. The lateral ESOs run in the roll-invariant WORLD
// tilt frame and the command is mapped into the current body/gimbal frame here (w2b). A roll-ignorant controller
// would skip this and diverge -- at 90 deg roll the pitch channel would command yaw; at 180 the sign flips.
static inline void w2b(float wx_,float wy_,float&bx,float&by){
  float c=cosf(rollTwist), s=sinf(rollTwist); bx=c*wx_+s*wy_; by=-s*wx_+c*wy_; }
static inline void b2w(float bx,float by,float&wx_,float&wy_){
  float c=cosf(rollTwist), s=sinf(rollTwist); wx_=c*bx-s*by; wy_=s*bx+c*by; }

// Margin-fin aero authority (rad/s^2 per unit deploy change), SIGNED. A fold-out pair's moment per deploy is
// dM/ddep = -q*sin(aoa)*S_FIN*CN_FIN*l_fin -- aoa-mediated: zero pointing exactly along/against the airflow,
// sign follows aoa. The SENSED lateral aero specific force gives exactly that gain, wind-agnostically (the
// old |sf| magnitude gauge kept the wind-agnostic trick but threw away the sign the fold-out plant needs):
// m*sfx = lateral aero force ~ +q*S_REF*CD*sin(aoa)  =>  dM/ddep = sfx*m*S_FIN*CN_FIN*l_fin/(S_REF*CD).
// Neutral-stability deploy, CG-scheduled: by coast the ascent propellant is burnt -> CG is CG_PROP_M forward
// of nominal -> the body arm is LONGER and the fin arm SHORTER than at launch -> dep* is NOT a fixed value.
float depNeutralCoast(){
  float xcg=CG_PROP_M;                          // coast mass state: ascent propellant burnt, legs stowed
  float lb=L_BODYCP0+xcg, lf=L_FIN0-xcg;
  return constrain((S_REF*CN_BODY*lb)/(S_FIN*CN_FIN*lf),0.0f,1.0f);
}
// 3-D: the fin moment per unit deploy is a VECTOR fixed by the crossflow direction (the fold-out pair can only
// push the nose within the plane containing the body axis and the airflow). In TILT-CHANNEL coordinates (the
// axes the ESOs control: tiltX is driven by torque about body +Y, tiltY by torque about body -X) the authority
// vector reduces to simply +k*(sfx, sfy): the sensed lateral aero force IS the achievable tilt-accel direction,
// signed -- the clean 3-D generalization of the 2-D scalar b0 = k*sfBxLP. Wind-agnostic, magnitude AND direction.
void marginAuthorityVec(float&bx,float&by){     // rad/s^2 per unit deploy, BODY tilt-channel axes
  float lf=L_FIN0-CG_PROP_M;
  float k=(MASS_LAND_KG*S_FIN*CN_FIN*lf)/(IYY*S_REF*DRAG_CD);
  bx=k*sfBxLP; by=k*sfByLP;
}
// ROLL authority of the DIFFERENTIAL fold: dM_z/drollDef ~ q*S_FIN*CN_FIN*(vlat/V)*R_FIN*2 -- again recovered
// wind-agnostically from |sf_lat| (~ q*S*CD*vlat/V / m). Proportional to sin(aoa): the fins can only fight roll
// when there IS crossflow. That's physics, not a controller choice -- at aoa~0 (early boost) roll is UNCONTROLLED
// and only bounded by keeping the TVC/CG misalignment small (build quality) until aero authority arrives.
float rollAuthority(){                          // rad/s^2 per unit rollDef (>=0)
  float sfLat=sqrtf(sfBxLP*sfBxLP+sfByLP*sfByLP);
  return sfLat*(MASS_LAND_KG*S_FIN*CN_FIN*R_FIN_M*2.0f)/(IZZ_ROLL*S_REF*DRAG_CD);
}
// Roll-rate ESO + rate law (roll ANGLE is free -- the quaternion handles orientation; we only keep the RATE
// bounded so the lateral loops track and the vehicle doesn't wind up). 2-state ESO: z1=rate, z2=disturbance.
void ctrlRoll(){
  float b0r=rollAuthority();
  float uA=b0r*rollDef_act;
  float e=gzr-esoR.z1;
  esoR.z1+=DT_CTRL*(esoR.z2+uA+2.0f*W0_ROLL*e);
  esoR.z2+=DT_CTRL*(W0_ROLL*W0_ROLL*e);
  if(b0r<B0_ROLL_MIN){ rollDef=0.0f; }           // no crossflow to bite -> stow the differential, ride it out
  else {
    float alphaR=-WC_ROLL*esoR.z1-esoR.z2;       // drive roll RATE to zero + cancel the identified roll torque
    rollDef=constrain(alphaR/b0r,-0.5f,0.5f);
  }
  rollDef_act+=constrain((rollDef-rollDef_act)/0.05f,
                         -SERVO_SLEW_DPS/(2.0f*ROLL_DEG_RANGE),SERVO_SLEW_DPS/(2.0f*ROLL_DEG_RANGE))*DT_CTRL;
  writeRoll(rollDef);
}
// NOTE (2026-07-02): the coast loop below ALREADY does angular-acceleration ("torque") commanding via authority
// inversion (deploy = depN + alphaDes/b0), exactly parallel to the TVC loops' u = alphaDes/keff. Mechanism: the
// net aero moment is -q*sin(aoa)*(S_REF*CN_BODY*l_b - dep*S_FIN*CN_FIN*l_f). At the NEUTRAL fold dep* (~0.48 here)
// the bracket is zero -> torque-free at any attitude. Fold BELOW dep* -> net CP aft of CG -> weathercocks nose-
// first; fold ABOVE dep* -> net CP forward of CG -> weathercocks TAIL-first (engine-down = retrograde). The ADRC
// picks how far past/below dep* to fold moment-by-moment: it's WEATHERCOCK-STEERING, modulated by feedback.
// TVC additionally RLS-identifies keff online; a coast analog was BUILT, TESTED, REMOVED. (1) coast RLS is
// confounded -- the fin regulates against a body weathercock of comparable size, so with good tracking net accel
// ~0 while deploy!=0 and the deploy-vs-accel slope reads ~0 (TVC's RLS works because powered flight has
// independent excitation and a smaller relative disturbance). (2) The SIL fin-authority sweep (--finauth) showed
// the ESO already makes this robust across true/modeled fin ratio 0.7-2.0x. It fails below ~0.55x, but that is a
// PHYSICAL limit: the retrograde slew is driven by the NET tail-first restoring at full deploy = (fin_full - body).
// Nominal fin_full is 2.08x the body weathercock -> net 1.08x body (comfortable). At 0.5x, fin_full ~= body -> net
// ~0 -> no authority to force/hold engine-down -> tumbles even in dead-calm air. No b0 identification manufactures
// that authority; the lever is fin SIZING so fin_full stays well above the body weathercock (net >~0.15x body).

// Divert: nudge the retrograde target toward the nominal x(z) (limited authority).
static float nomInterp(float z, const float* a){   // interp a[] at altitude z over descending g_nomZ
  if(g_nomN<2) return 0;
  if(z>=g_nomZ[0]) return a[0];
  if(z<=g_nomZ[g_nomN-1]) return a[g_nomN-1];
  for(int i=0;i<g_nomN-1;i++) if(z<=g_nomZ[i] && z>=g_nomZ[i+1]){ float f=(g_nomZ[i]-z)/max(1e-3f,(g_nomZ[i]-g_nomZ[i+1])); return a[i]+f*(a[i+1]-a[i]); }
  return a[g_nomN-1];
}
// Divert: null the DEVIATION from the nominal trajectory via a translational ADRC, convert the desired
// horizontal accel to a thrust tilt. thrustAcc = thrust/mass (available lateral authority during the burn).
float divertOffset(float thrustAcc){
  if(!g_divertOn) return 0.0f;               // nominal-generation / kick-bisection: fly pure ballistic
  float dx, dvx;
  if(g_nomN>=2){ dx=estX-nomInterp(estZ,g_nomX); dvx=estVx-nomInterp(estZ,g_nomVx); }
  else { dx=estX-g_targetX; dvx=estVx; }            // fallback: regulate straight to target (PD via the ESO)
  if(!txInit){ zx1=dx; zx2=dvx; zx3=0; txInit=true; }
  float ah = -WCX*WCX*zx1 - 2.0f*WCX*zx2 - zx3;      // desired horizontal accel to drive the deviation to 0
  float corr = asinf(constrain(ah/max(1.0f,thrustAcc), -STILT2, STILT2));
  float axR = thrustAcc*sinf(corr), e = dx - zx1;    // ESO update with the realized horizontal accel (anti-windup)
  zx1 += DT_CTRL*(zx2 + 3.0f*W0X*e);
  zx2 += DT_CTRL*(zx3 + axR + 3.0f*W0X*W0X*e);
  zx3 += DT_CTRL*(W0X*W0X*W0X*e);
  // Ramp the divert OUT from ~10 m down to ~4 m so the rocket finishes correcting position while it's still
  // high, then flies PURELY retrograde->vertical for the last ~4 m -- no leaning-for-target near the ground
  // (that terminal lean, right as legs deploy, is what induces the wobble). Land safe first, hit the mark second.
  // Fade the divert out HIGH (full to 16 m, gone by 8 m). Because the wind is estimated during coast and locked
  // at ignition, the correction can be made up high where there is still altitude for retrograde to re-null the
  // cross-range velocity it builds and recover vertical before touchdown -- so the corrective lean doesn't become
  // a terminal wobble. (Fading it out low, ~4 m, recovers more accuracy but lands still leaning ~7 deg.)
  return corr * constrain((estZ - 8.0f) / 8.0f, 0.0f, 1.0f);
}

// ---- WIND-ADAPTIVE ASCENT GUIDANCE ---------------------------------------------------------------
// Ascent-motor thrust at burn-time tau (mirror of the datasheet curve; same form as landThrustAt).
static inline float ascentThrustAt(float tau){
  if(tau<0.0f||tau>ASC_BURN_S) return 0.0f;
  if(tau<0.15f) return ASC_PEAK_N*tau/0.15f;
  if(tau<0.50f) return ASC_PEAK_N-(ASC_PEAK_N-ASC_PLATEAU_N)*(tau-0.15f)/0.35f;
  return ASC_PLATEAU_N;
}
constexpr float GUIDE_IGN_ALT = 22.0f;   // altitude the hoverslam nominally ignites at (predictor's descent stop)
// Forward-predict the LANDING downrange for a candidate boost aim `kickRad`, from the current state, under the
// estimated wind: fly the REMAINING boost (thrust along the aimed attitude) then ballistic coast/descent (drag +
// wind) down to the ignition altitude; add the small cross-range the retrograde burn coasts out (~0.5*vx_ign,
// measured). Pure axial-drag airframe (matches the plant): attitude affects translation only while thrusting.
static void predictLandXY(float kickRadX, float kickRadY, float tauNow, float&lx, float&ly){
  float px=estX, vx=estVx, py_=estY, vy_=estVy, pz=estZ, vz=estVz, tau=tauNow; const float dt=0.05f;
  const float kd=0.5f*RHO*S_REF*DRAG_CD/MASS_LAND_KG, m=MASS_LAND_KG;
  for(int n=0; n<400; n++){
    float thr = (tau<ASC_BURN_S)? ascentThrustAt(tau) : 0.0f;
    float vax=vx-windEst, vay=vy_-windEstY, V=sqrtf(vax*vax+vay*vay+vz*vz)+1e-3f;
    float ax = thr*sinf(kickRadX)/m - kd*V*vax;
    float ay = thr*sinf(kickRadY)/m - kd*V*vay;
    float az = thr*cosf(kickRadX)*cosf(kickRadY)/m - kd*V*vz - G;
    vx+=ax*dt; vy_+=ay*dt; vz+=az*dt; px+=vx*dt; py_+=vy_*dt; pz+=vz*dt; tau+=dt;
    if(tau>ASC_BURN_S && vz<0.0f && pz<=GUIDE_IGN_ALT) break;   // reached hoverslam ignition on the way down
  }
  lx = px + 0.5f*vx;                        // + cross-range the retrograde burn coasts out before it nulls v_lat
  ly = py_ + 0.5f*vy_;
}
// ============================================================================
//  CONTROL TICKS (100 Hz)
// ============================================================================
void ctrlBoost(){                                  // TVC, hold gravity-turn/kick + wind-adaptive downrange guidance
  updateKeff(); float b0=constrain(keff_est,KEFF_MIN,KEFF_MAX);
  // Steer the boost aim so the PREDICTED landing (under the wind estimated so far) hits the target -- correcting
  // the wind-induced coast drift UP FRONT where thrust has authority, so the descent can land pure-retrograde
  // (smooth) with no terminal divert. Wait for the wind estimate to settle (~1 s) and freeze the aim just before
  // burnout (attitude then settles to the final kick). Only active when a target is set (g_targetX != 0).
  float tau=(millis()-ignMs)/1000.0f;
  // IN-FLIGHT MASS IDENTIFICATION: the hoverslam is feed-forward and otherwise commits to a NOMINAL mass, so a
  // real +-8% build/propellant dispersion busts the soft-landing gate (SIL: vz 3.5 -> 8-12). During boost the
  // body-axial specific force sfBz ~= thrust/m (accelerometer reads the thrust accel; axial drag is small), and
  // the ascent thrust is the KNOWN curve -> m = thrust/sfBz. Scale the whole vehicle mass by the measured/nominal
  // ratio (assumed ~constant across the flight) and feed it to the hoverslam. Mid-boost only (clean strong thrust).
  if(g_massIdOn && tau>0.6f && tau<ASC_BURN_S-0.3f && sfBz>4.0f){
    float f=constrain(tau/ASC_BURN_S,0.0f,1.0f);
    float mNom=MASS_LAND_KG + ASC_PROP_KG*(1.0f-f);          // nominal boost mass at this burn fraction
    float V=sqrtf(estVx*estVx+estVz*estVz);
    float drag=0.5f*RHO*V*V*S_REF*DRAG_CD;                   // axial drag: sfBz=(thrust-drag)/m, so subtract it or m biases +3-5%
    float sInst=constrain(((ascentThrustAt(tau)-drag)/sfBz)/mNom, 0.7f, 1.4f);
    massScale += 0.03f*(sInst-massScale);                    // slow LP toward the measured mass ratio
  }
  if(g_divertOn && tau>1.0f && tau<ASC_BURN_S-0.3f){   // g_divertOn=false only during the pre-flight calm-kick bisection
    float lx,ly; predictLandXY(g_kickCmd*M_PI/180.0f, g_kickCmdY*M_PI/180.0f, tau, lx, ly);
    g_kickCmd  = constrain(g_kickCmd  + 0.15f*(g_targetX-lx), -8.0f, 15.0f);   // aim up/downwind (X)
    g_kickCmdY = constrain(g_kickCmdY + 0.15f*(0.0f    -ly), -8.0f,  8.0f);    // null the CROSSwind drift (Y)
  }
  // Lateral ESOs run in the roll-invariant WORLD tilt frame; the realized accel (anti-windup) and the commanded
  // accel are mapped through the measured roll to the co-located body gimbals (w2b/b2w). See w2b note.
  float uAwx,uAwy; b2w(b0*u_tvc_act, b0*u_tvc2_act, uAwx, uAwy);
  float aXw=adrcAlphaDes(esoX, tiltX, g_kickCmd *M_PI/180.0f, uAwx);
  float aYw=adrcAlphaDes(esoY, tiltY, g_kickCmdY*M_PI/180.0f, uAwy);
  float aXb,aYb; w2b(aXw,aYw,aXb,aYb);
  u_tvc =constrain(aXb/b0,-U_MAX,U_MAX);
  u_tvc2=constrain(aYb/b0,-U_MAX,U_MAX);
  u_tvc_act +=constrain((u_tvc -u_tvc_act )/0.05f,
                       -SERVO_SLEW_DPS/TVC_DEG_PER_UNIT,SERVO_SLEW_DPS/TVC_DEG_PER_UNIT)*DT_CTRL;   // actuator observers
  u_tvc2_act+=constrain((u_tvc2-u_tvc2_act)/0.05f,
                       -SERVO_SLEW_DPS/TVC_DEG_PER_UNIT,SERVO_SLEW_DPS/TVC_DEG_PER_UNIT)*DT_CTRL;
  writeTVC(u_tvc,u_tvc2); writeMargin(0.0f);       // fins folded under thrust: max static stability
  ctrlRoll();                                      // fins CAN still bite differentially if crossflow exists
}
// Coast control = MARGIN MODULATION (matches the fold-out plant). The key property of the fold-out pair:
// at the CG-scheduled NEUTRAL deploy (dep*) the net aero moment is ZERO AT ANY ATTITUDE -- the vehicle is
// torque-free and just holds its attitude (rate-damped). So the policy is TRIM-RIDE: sit at dep* and let the
// ADRC modulate deploy around it through the SIGNED aoa-mediated authority. Holding an attitude far off the
// airflow (ground-retrograde in a strong headwind) costs NOTHING here -- unlike a stability bias, which
// would let the flow CAPTURE the vehicle onto the wind-tilted aero equilibrium (that capture, then a slow
// sin(aoa)-limited escape arriving at ignition with huge rate, was the headwind failure mode). Where the
// authority is unreadable (aoa ~ 0/180 or apogee airspeed: sensed side force below B0_COAST_MIN) just hold
// dep*: no disturbance to fight there is exactly what neutral gives.
constexpr float B0_COAST_MIN=0.5f;               // rad/s^2/unit below which we hold neutral trim
void ctrlCoast(){                                  // MARGIN FINS, slew to & hold retrograde
  // Speed-blended + cone-limited 3-D retro target: near apogee atan2 is ill-conditioned; hold vertical until
  // there's real descent speed, then ease in <=35 deg total.
  retroTargetXY(estVx, estVy, estVz, thRetro, thRetroY);
  float depN=depNeutralCoast();
  // ANGULAR-ACCELERATION ("torque") COMMANDING, same shape as the TVC loops -- but the single fold servo's
  // moment is a VECTOR fixed by the crossflow direction (the pair can only push the nose within the plane
  // containing the body axis and the airflow). Least-squares-project the desired accel vector onto that
  // achievable direction; the out-of-plane component is weakly relevant for retro tracking (it's the azimuth
  // AROUND the flow axis) and the ESOs absorb it. See marginAuthorityVec() re: why no RLS self-ID of b0.
  float bvx,bvy; marginAuthorityVec(bvx,bvy);      // BODY-frame authority vector (signed by the sensed aoa)
  float bmag2=bvx*bvx+bvy*bvy;
  float dAct=deploy_act-depN;
  float uAwx,uAwy; b2w(bvx*dAct, bvy*dAct, uAwx, uAwy);   // realized world accels for anti-windup
  float aXw=adrcAlphaDes(esoX, tiltX, thRetro,  uAwx, WC_COAST, W0_COAST);
  float aYw=adrcAlphaDes(esoY, tiltY, thRetroY, uAwy, WC_COAST, W0_COAST);
  if(bmag2<B0_COAST_MIN*B0_COAST_MIN){
    deploy = depN;                                 // torque-free trim: hold attitude, damp rates
  } else {
    float aXb,aYb; w2b(aXw,aYw,aXb,aYb);
    deploy=constrain(depN+(aXb*bvx+aYb*bvy)/bmag2,0.0f,1.0f);   // project the desired accel onto the fin authority vector
  }
  deploy_act+=constrain((deploy-deploy_act)/0.05f,
                        -SERVO_SLEW_DPS/(2.0f*MARGIN_DEG_RANGE),SERVO_SLEW_DPS/(2.0f*MARGIN_DEG_RANGE))*DT_CTRL;
  u_tvc=0; u_tvc2=0;                               // (logged) TVC is neutral during coast
  writeMargin(deploy); writeTVC(0,0);
  ctrlRoll();                                      // differential fold: roll-rate damping (authority ~ sin aoa)
}
void ctrlBurn(){                                   // TVC, hold retrograde + null velocity + divert
  updateKeff(); float b0=constrain(keff_est,KEFF_MIN,KEFF_MAX);
  // Retrograde blended to vertical by SPEED (not altitude): point opposite the velocity while there's
  // velocity to kill, smoothly -> vertical as the burn nulls it. This also drives atan2(-vx,-vz) toward 0
  // exactly when it becomes ill-conditioned (V->0), which was the source of the terminal attitude swing.
  retroTargetXY(estVx, estVy, estVz, thRetro, thRetroY);   // speed-blended, cone-limited (shared with coast)
  // TERMINAL DESCENT = PURE RETROGRADE (3-D). Point anti-velocity, speed-blended to vertical -> nulls descent
  // AND both cross-range velocities together for a smooth upright touchdown. NO position divert here (leaning
  // for the target near the deck is what caused the terminal wobble); downrange/crossrange accuracy is handled
  // UP FRONT by the wind-adaptive ascent guidance in BOTH axes.
  float uAwx,uAwy; b2w(b0*u_tvc_act, b0*u_tvc2_act, uAwx, uAwy);
  float aXw=adrcAlphaDes(esoX, tiltX, thRetro,  uAwx);
  float aYw=adrcAlphaDes(esoY, tiltY, thRetroY, uAwy);
  float aXb,aYb; w2b(aXw,aYw,aXb,aYb);
  u_tvc =constrain(aXb/b0,-U_MAX,U_MAX);
  u_tvc2=constrain(aYb/b0,-U_MAX,U_MAX);
  u_tvc_act +=constrain((u_tvc -u_tvc_act )/0.05f,
                       -SERVO_SLEW_DPS/TVC_DEG_PER_UNIT,SERVO_SLEW_DPS/TVC_DEG_PER_UNIT)*DT_CTRL;   // slew-matched
  u_tvc2_act+=constrain((u_tvc2-u_tvc2_act)/0.05f,
                       -SERVO_SLEW_DPS/TVC_DEG_PER_UNIT,SERVO_SLEW_DPS/TVC_DEG_PER_UNIT)*DT_CTRL;
  writeTVC(u_tvc,u_tvc2); writeMargin(0.0f);       // fins folded under thrust
  ctrlRoll();
}
void resetESO(){ esoX={tiltX,0,0}; esoY={tiltY,0,0}; esoR={gzr,0,0}; txInit=false; }   // seed ESOs at the current world tilt

// Landing-motor thrust at burn-time tau (mirror of the datasheet curve used to size the hoverslam).
static inline float landThrustAt(float tau){
  if(tau<0.0f||tau>LAND_BURN_S) return 0.0f;
  if(tau<0.15f) return LAND_PEAK_N*tau/0.15f;
  if(tau<0.50f) return LAND_PEAK_N-(LAND_PEAK_N-LAND_PLATEAU_N)*(tau-0.15f)/0.35f;
  return LAND_PLATEAU_N;
}
// Forward-simulate "ignite NOW" from (z,v<0) under gravity + the KNOWN thrust curve; return the altitude at which
// the descent is arrested (vz reaches 0). >0 => over-brake (stops above ground -> a solid motor then lifts back
// off); <=0 => the ground is reached while still descending (under-brake / hard). Purely causal: current state +
// motor datasheet, no offline replay. cosT accounts for a tilted (retrograde/divert) burn stealing vertical thrust.
static float predictStopAlt(float z,float v){
  float V=sqrtf(estVx*estVx+estVy*estVy+estVz*estVz);
  float cosT=(V>0.5f)? constrain(fabsf(estVz)/V, cosf(RETRO_MAX_RAD), 1.0f) : 1.0f;   // vertical fraction of thrust
  float vzz=v, zz=z, tau=0.0f; const float dt=0.01f;
  while(tau<LAND_BURN_S && vzz<0.0f){
    // mass drops as the landing motor burns its propellant -> decel rises through the burn; model it so the
    // ignition timing stays right (a constant-mass predictor over-brakes late in the burn -> hop/hard). massScale
    // = the in-flight-identified true/nominal mass ratio (boost thrust/accel), so a dispersed build times right.
    float m=(MASS_LAND_KG - LAND_PROP_KG*(tau/LAND_BURN_S))*massScale;
    float aDrag=0.5f*RHO*vzz*vzz*S_REF*DRAG_CD/m;                 // opposes descent (vzz<0 -> +up), real braking
    float a=cosT*landThrustAt(tau)/m - G + aDrag;                 // net vertical accel (thrust + drag - gravity)
    vzz+=a*dt; zz+=vzz*dt; tau+=dt;
    if(zz<=0.0f) return zz;                             // hit ground still descending (under-brake)
  }
  return zz;   // arrested at altitude zz (>=0 over-brake margin)
}
// Ignite as LATE as possible while a full burn still arrests v at ~ground: the predicted stop-altitude falls
// monotonically as we descend (a burn-now over-brakes high up early, reaches the deck only near the crossover).
// Ignite when it crosses 0 -- the last instant a full burn still just reaches the ground as v nulls.
// Ignition setpoint: aim the predicted arrest slightly BELOW the deck. The cost of timing error is
// asymmetric for a solid motor: arriving ~1-2 m/s hot just uses the legs, but arresting ABOVE the deck
// leaves un-cuttable thrust (T/W>1) that relaunches the vehicle -> ~5+ m/s free-fall impact. The margin
// absorbs predictor/plant mismatch (gusts, tilt profile, thrust tolerance) on the safe side of that edge.
constexpr float STOP_MARGIN_M=0.65f;                                                // <<< SET
bool ignitionReached(){
  // EKF already compensates SENSOR lag at fusion (estZ/estVz are the true present state -- don't re-lead that).
  // But the MOTOR has a dead time (IGN_LEAD_S) before thrust builds, during which the vehicle FREE-FALLS. Project
  // the state forward by that dead time and test the burn FROM there, so the burn BEGINS where the predictor wants
  // (fires IGN_LEAD_S earlier). Without this a 25 ms motor delay ignites ~0.6 m late at 24 m/s -> under-brake.
  if(estVz>-DESCENT_VZ_MIN) return false;
  float zc = estZ + estVz*IGN_LEAD_S - 0.5f*G*IGN_LEAD_S*IGN_LEAD_S;   // free-fall during spin-up dead time
  float vc = estVz - G*IGN_LEAD_S;
  // Margin applied to the START altitude (not the return value: predictStopAlt returns at the first ground
  // crossing, so its value saturates at ~-vz*dt and a negative threshold would never trigger). "Ignite only
  // once a burn started STOP_MARGIN_M higher would just reach the deck" == arrives slightly hot, not airborne.
  return predictStopAlt(zc+STOP_MARGIN_M,vc) <= 0.0f;
}

// ============================================================================
//  SAFETY
// ============================================================================
void abortFlight(Ab r){ if(state==St::ABORTED)return; abortReason=r; Serial.print(F("ABORT -> ")); Serial.println(abName(r));
  neutralActuators(); firePyro(P_CHUTE); setState(St::ABORTED); }
void safety(){
  if(!sensorsOk){ abortFlight(Ab::SENSOR); return; }
  float zbz=1-2*(qx*qx+qy*qy);                                          // cos(total tilt) from the quaternion
  if(acosf(constrain(zbz,-1.0f,1.0f))*180.0f/M_PI>ABORT_TILT_DEG){ abortFlight(Ab::TILT); return; }
  if(max(fabsf(gxr),fabsf(gyr))*180.0f/M_PI>ABORT_RATE_DPS){ abortFlight(Ab::RATE); return; }
  if(fabsf(gzr)*180.0f/M_PI>ABORT_ROLL_DPS){ abortFlight(Ab::RATE); return; }   // roll ran away (fins couldn't catch it)
}

// ============================================================================
//  LOGGING
// ============================================================================
void openLog(){
  int i=0; do{ snprintf(logName,sizeof(logName),"LND%03d.CSV",i++);}while(SD.exists(logName)&&i<1000);
  logFile=SD.open(logName,FILE_WRITE);
  if(logFile){ Serial.print(F("LOG ")); Serial.println(logName);
    logFile.println(F("ms,state,abort,tiltX,tiltY,roll,gx,gy,gz,uTVC,uTVC2,deploy,rollDef,keff,estZ,estVz,estX,estVx,estY,estVy,thRetro,aWz,gpsFix,gpsX,gpsY")); logFile.flush(); }
}
void logRow(){
  if(!logFile)return; char r[300];
  int n=snprintf(r,sizeof(r),"%lu,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f,%d,%.2f,%.2f\n",
    millis(),stName(state),abName(abortReason),tiltX,tiltY,rollTwist,gxr,gyr,gzr,u_tvc,u_tvc2,deploy,rollDef,
    keff_est,estZ,estVz,estX,estVx,estY,estVy,thRetro,lastAccZ,(int)gpsFix,gpsX,gpsY);
  if(n>0&&n<(int)sizeof(r)) logFile.write((const uint8_t*)r,(size_t)n);
}

// ============================================================================
//  STATE HANDLERS
// ============================================================================
void hPad(){ float t=th*180.0f/M_PI; if(t<-1)LED(0,0,1); else if(t>1)LED(1,0,0); else LED(0,1,1);
  if(buttonEdge()){ setState(St::ARMING); Serial.println(F("ARM: press 5x")); } }
void hArm(){ LED(1,1,1);
  if(buttonEdge()){ uint32_t now=millis(); armCount=(now-lastArmMs<=300)?armCount+1:1; lastArmMs=now; beep(380+40*armCount,70);
    if(armCount>=5){ Serial.println(F("CALIBRATING - hold still")); LED(1,0,1); padCalibrate();
      kfInit(kfZ,0); kfInit(kfX,0); kfInit(kfY,0); openLog(); cdLast=-1; cdMoveMs=0; armCount=0; setState(St::COUNTDOWN); } } }
void hCd(){
  if(buttonEdge()){ abortReason=Ab::MANUAL; neutralActuators(); setState(St::ABORTED); return; }
  if(max(fabsf(tiltX),fabsf(tiltY))*180.0f/M_PI>CD_TILT_ABORT_DEG ||
     max(max(fabsf(gxr),fabsf(gyr)),fabsf(gzr))*180.0f/M_PI>CD_RATE_ABORT_DPS){
    if(cdMoveMs==0)cdMoveMs=millis(); else if(millis()-cdMoveMs>=100){ abortReason=Ab::CD_MOVED; setState(St::ABORTED); return; } } else cdMoveMs=0;
  uint32_t el=millis()-stateMs; int rem=(int)COUNTDOWN_S-(int)(el/1000UL);
  if(rem!=cdLast&&rem>=0){ cdLast=rem; Serial.print(F("T-")); Serial.println(rem); beep(rem>3?440:880,100); }
  if(el>=COUNTDOWN_S*1000UL){ baroAlt0=bmp.readAltitude(SEA_LEVEL_HPA); coastPeak=0; apogee=false;
    firePyro(P_ASCENT); ignMs=millis(); inFlight=true; adaptT=0; g_kickCmd=g_kickDeg; g_kickCmdY=0; resetESO(); setState(St::BOOST); } }
void hBoost(){ LED(1,1,0); safety();
  if(millis()-ignMs>=(uint32_t)(ASCENT_BURN_S*1000.0f)){ coastMs=millis(); apogee=false; resetESO(); setState(St::COAST); } }
void hCoast(){ LED(0,1,0); safety();
  uint32_t el=millis()-coastMs;
  if(!apogee && coastPeak-estZ>1.0f && estVz<-0.5f && el>1000) apogee=true;        // past apogee, now descending under control
  if(ignitionReached()){
    firePyro(P_LAND);
    firePyro(P_LEGS); legsFired=true;   // deploy legs AT ignition: legs shift the CG/inertia (~25% keff drop);
    // doing it once at burn-start keeps the plant CONSTANT for the whole controlled descent (RLS identifies it
    // from the start with ~1.5 s to settle), instead of a 25% plant jump ~0.5 s before touchdown that the
    // controller can't track in time -> the near-ground wobble. Legs are aft/trailing in the engine-down burn.
    windLocked=true;                    // freeze the coast wind estimate for use through the burn
    keff_est=KEFF_NOM;                  // reset b0: coast leaves keff at a stale/low value -> 3x over-actuation
    // and a violent uTVC/rate oscillation for the first ~0.15 s of the burn until RLS re-converges. Start at the
    // landing-plant nominal instead; RLS refines it.
    ignMs=millis(); adaptT=0; resetESO(); setState(St::LANDING_BURN); return;
  }
  if(el>25000){ abortFlight(Ab::APOGEE_TO); return; } }
void hBurn(){ LED(1,0,1); safety();
  // Do NOT declare touchdown while the solid is still thrusting (T/W>1): if velocity nulls a few cm above
  // the deck, the motor relaunches the vehicle whether or not we call it "landed" -- and cutting control
  // there surrenders TVC while the motor is still shoving. Worse in 3-D: with a lateral CG offset the residual
  // thrust is an UNOPPOSED constant torque (~T*cgoff/IYY), which spun up a rate that the hop then integrated
  // into ~55 deg of tilt. Keep TVC until the burn is fully over (declaring TD 0.3 s late ON the deck is
  // harmless -- contact physics has already frozen the vehicle; declaring it early in the air is not).
  bool burnDone = millis()-ignMs >= (uint32_t)(1050.0f*LAND_BURN_S);   // burn + 5% margin fully elapsed
  if(burnDone && (estZ<TD_ALT_M || (estZ<0.5f && estVz>-TD_VZ_M))){ neutralActuators(); setState(St::TOUCHDOWN); } }
void hTd(){ inFlight=false; neutralActuators(); LED(1,1,1);
  static uint32_t lb=0; if(millis()-lb>2000){ lb=millis(); beep(523,80); beep(392,80); }
  if(buttonEdge()){ if(logFile){logFile.flush();logFile.close();} abortReason=Ab::NONE; setState(St::PAD_ALIGN); } }
void hAbort(){ inFlight=false; neutralActuators(); LED(1,0,0);
  static uint32_t lb=0; if(millis()-lb>250){ lb=millis(); beep(700,80); }
  if(buttonEdge()){ if(logFile){logFile.flush();logFile.close();} abortReason=Ab::NONE; setState(St::PAD_ALIGN); } }

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup(){
  analogReadResolution(12);
  pinMode(BUTTON,INPUT); pinMode(BUZZER,OUTPUT); pinMode(RLED,OUTPUT); pinMode(GLED,OUTPUT); pinMode(BLED,OUTPUT);
  pinMode(P_LEGS,OUTPUT); pinMode(P_LAND,OUTPUT); pinMode(P_ASCENT,OUTPUT); pinMode(P_CHUTE,OUTPUT);
  digitalWrite(P_LEGS,LOW); digitalWrite(P_LAND,LOW); digitalWrite(P_ASCENT,LOW); digitalWrite(P_CHUTE,LOW);
  LED(0,0,0); Serial.begin(115200); delay(200); Serial.println(F("\n=== SYSIPHUS LANDING GNC (TVC + MARGIN FINS) ==="));
  if(USE_GPS) GPS_SERIAL.begin(GPS_BAUD);
  bool sOk=initSensors(); bool sd=SD.begin(SD_CS);
  servoTVC.attach(SERVO_TVC_PIN); servoTVC2.attach(SERVO_TVC2_PIN);
  servoMargin.attach(SERVO_MARGIN_PIN); servoRoll.attach(SERVO_ROLL_PIN); neutralActuators();
  if(!sOk){ Serial.println(F("SENSOR INIT FAIL")); LED(1,0,0); while(1){ beep(700,120); delay(200);} }
  if(!sd) Serial.println(F("WARN: SD init failed"));
  beep(523,70); beep(659,70); beep(784,120);
  lastSensMs=lastCtrlMs=lastLogMs=millis(); setState(St::PAD_ALIGN);
}
void loop(){
  uint32_t now=millis(); serviceGPS();
  if(now-lastSensMs>=SENSOR_PERIOD_MS){ float dt=constrain((now-lastSensMs)*0.001f,0.001f,0.05f); lastSensMs=now;
    float sfx,sfz; readIMU(sfx,sfz,dt);           // dt = MEASURED elapsed so a loop stall doesn't corrupt the integrators
    sensorsOk=isfinite(th); if(inFlight) ekfStep(sfx,sfz,dt); }
  updatePyros();
  switch(state){
    case St::PAD_ALIGN: hPad(); break;
    case St::ARMING:    hArm(); break;
    case St::COUNTDOWN: hCd();  break;
    case St::BOOST:        if(now-lastCtrlMs>=CONTROL_PERIOD_MS){ lastCtrlMs=now; ctrlBoost(); } hBoost(); break;
    case St::COAST:        if(now-lastCtrlMs>=CONTROL_PERIOD_MS){ lastCtrlMs=now; ctrlCoast(); } hCoast(); break;
    case St::LANDING_BURN: if(now-lastCtrlMs>=CONTROL_PERIOD_MS){ lastCtrlMs=now; ctrlBurn();  } hBurn();  break;
    case St::TOUCHDOWN: hTd();  break;
    case St::ABORTED:   hAbort();break;
    default: setState(St::PAD_ALIGN); break;
  }
  if(inFlight && now-lastLogMs>=50){ lastLogMs=now; logRow(); }
}
