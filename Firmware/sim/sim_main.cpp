/*
  sim_main.cpp — Software-In-the-Loop (SIL) harness for Sysiphus_Landing.cpp
  ==========================================================================
  Runs the EXACT flight firmware (../Sysiphus_Landing.cpp) on the host:
   - defines the Arduino/Teensy core the shims declare (clock, pins, Serial)
   - simulates a DEFAULT ROCKET (2D: pitch plane + vertical), feeds synthetic
     IMU/baro/GPS into the firmware, captures its servo + pyro outputs, and
     advances the physics with them
   - drives the state machine automatically (button presses) and logs a CSV of
     TRUTH vs the firmware's own estimates so you can see exactly how the code flies

  Build (needs a host C++ compiler):  see README.md   (e.g. g++ -std=c++17 ...)
  Output: CSV to stdout (redirect to a file); a SUMMARY to stderr.

  The default-rocket constants below MUST stay consistent with the firmware's
  config block (IYY, MASS_LAND_KG, LAND_THRUST_AVG_N, the aero params, pins).
*/
#include "shims/Wire.h"
#include "shims/SPI.h"
#include "shims/SD.h"

// ============================================================================
//  Simulator globals (declared extern in Arduino.h)
// ============================================================================
uint64_t g_micros = 0;
int      g_pin[64] = {0};
int      g_servoDeg[64];
double   g_imu_gyroX = 0, g_imu_gyroY = 0, g_imu_gyroZ = 0;
double   g_imu_accX = 0, g_imu_accY = 0, g_imu_accZ = 0;
double   g_baro_alt = 0;
HardwareSerial Serial(true);    // echo firmware status messages to stderr
HardwareSerial Serial1(false);  // GPS port (RX only)
TwoWire  Wire;
SPIClass SPI;
SDClass  SD;

// ============================================================================
//  Arduino core implementation
// ============================================================================
unsigned long millis(){ return (unsigned long)(g_micros / 1000ULL); }
unsigned long micros(){ return (unsigned long)(g_micros); }
void pinMode(int,int){}
void digitalWrite(int p,int v){ if(p>=0&&p<64) g_pin[p]=v; }
int  digitalRead(int p){ return (p>=0&&p<64)? g_pin[p] : LOW; }
int  analogRead(int){ return 0; }
void analogReadResolution(int){}
void tone(int,unsigned int){}
void noTone(int){}
void delayMicroseconds(unsigned int us){ sim_advance((us+999)/1000); }
void delay(unsigned long ms){ sim_advance(ms); }

// ============================================================================
//  Firmware symbols (from ../Sysiphus_Landing.cpp)
// ============================================================================
void setup(); void loop();
enum class St:uint8_t { BOOT,PAD_ALIGN,ARMING,COUNTDOWN,BOOST,COAST,LANDING_BURN,TOUCHDOWN,ABORTED };
extern St state;
extern bool inFlight;
extern float th, q, estZ, estVz, estX, estVx, keff_est, u_tvc, deploy, thRetro, windEst;
extern float estY, estVy, u_tvc2, rollDef, rollTwist, windEstY;   // 3-axis additions
extern float g_kickDeg, g_targetX;   // firmware mission params (set from argv before the flight)
extern float ADRC_WC, ADRC_W0;       // firmware ADRC bandwidth (SIL sweep for slow-servo retune)
extern int g_nomN; extern float g_nomZ[], g_nomX[], g_nomVx[];   // firmware nominal trajectory (loaded via --nomin)
extern bool g_divertOn;                                          // --nodivert => pure ballistic (nominal generation)
extern bool g_massIdOn;                                          // --nomassid => disable in-flight boost mass ID
static const char* nomOutPath=nullptr; static const char* nomInPath=nullptr;
static double nrZ[48], nrX[48], nrVx[48]; static int nrN=0;      // recorded descent trajectory (for --nomout)
static const char* stN(St s){ const char* n[]={"BOOT","PAD","ARM","CD","BOOST","COAST","BURN","TD","ABORT"}; return n[(int)s]; }

// ============================================================================
//  DEFAULT ROCKET — keep consistent with the firmware config block
// ============================================================================
// Mirror of the firmware's pin / servo mapping (so we read the right channels)
static const int SERVO_TVC_PIN=4, SERVO_TVC2_PIN=5, SERVO_MARGIN_PIN=3, SERVO_ROLL_PIN=2;
static const int P_ASCENT=11, P_LAND=10, P_LEGS=9, P_CHUTE=12, BTN=14;
static const double TVC_NEUTRAL=90, TVC_DEG_PER_UNIT=2.0;
static const double MARGIN_NEUTRAL=90, MARGIN_DEG_RANGE=35.0;
static const double ROLL_NEUTRAL=90,  ROLL_DEG_RANGE=20.0;   // differential-fin servo: +-20 deg at rollDef +-0.5
static const double CU_TO_RAD = (15.0/12.0)*(M_PI/180.0);   // gimbal rad per command unit

// Physics / aero (match firmware IYY + aero params)
static const double G=9.80665, RHO=1.20;
static const double MASS=0.80;            // kg -- NOMINAL all-up mass at the landing burn (== firmware MASS_LAND_KG)
static const double IYY=0.012;            // kg m^2 lateral (pitch=yaw, == firmware IYY, at MASS)
static const double IZZ0=1.8e-4;          // kg m^2 ROLL inertia (slender body: ~m*r^2/2) -- ~65x smaller than IYY,
                                          //   which is WHY tiny roll torques diverge fast (the TVC roll problem)
static const double LEG_IZZ_FACTOR=1.6;   // legs swing out radially -> roll inertia jumps
static const double R_FIN=0.030;          // fin center radial offset from the roll axis (m): differential-fold arm
static const double L_NOZZLE=0.30;        // m
// Propellant depletion (REALISM): mass/CG/inertia change as the motors burn. Split so that at landing-burn
// START mass == MASS (the firmware's assumption); during the landing burn the true mass then drops BELOW it,
// testing the hoverslam's robustness to the mismatch it doesn't model.                                 // <<< SET
static const double LAND_PROP=0.03;       // landing-motor propellant (kg): mass 0.80 -> 0.76 over the burn
static const double ASC_PROP =0.03;       // ascent-motor propellant (kg): liftoff mass = MASS+ASC_PROP = 0.90
static const double MASS_DRY =MASS-LAND_PROP;                 // burnt-out mass (0.76)
static const double S_REF=0.003, CD=0.6, C_DAMP=0.003;        // == firmware aero (near-neutral airframe)
// MARGIN FINS = fold-out canards (REALISM, replaces the old signed-direct-moment abstraction): a symmetric
// fold-out pair on ONE servo can only shift the CP -- its moment is aoa-PROPORTIONAL (zero at aoa 0/180,
// sign follows aoa) and referenced to the MIGRATING CG. deploy 0 = folded flat (full nose-first stability),
// 1 = out 90 deg (max destabilization). Neutral stability is dep* = S_REF*CN_BODY*l_body/(S_FIN*CN_FIN*l_fin),
// which MIGRATES as the CG does (propellant burns aft->CG moves forward; legs swing CG aft).
static const double S_FIN=0.0009, CN_FIN=2.0, L_FIN0=0.30;    // fin pair fully out; fin CP fwd of nominal CG
static const double CN_BODY=2.0,  L_BODYCP0=0.033;            // bare-body CP 3.3 cm aft of nominal CG
static const double CG_PROP_M=0.009;       // CG moves this far FORWARD per motor's (aft) propellant burnt
static const double CG_LEGS_M=0.030;       // legs deploy -> CG aft by this (also shortens the TVC arm)
static double WIND=2.0;                    // m/s steady wind at the reference altitude, +x (downrange)  -- runtime: --wind
static double WINDY=0.0;                   // m/s steady CROSSwind, +y  -- runtime: --windy (3-D: guidance must null it)
static double wTurb=0, wTurbY=0; static bool GUSTS=false; static const double TAU_W=0.8;   // OU gusts per axis (--gusts)
// Wind SHEAR (REALISM): wind grows with altitude (atmospheric boundary layer, power law). WIND is the speed at
// WIND_ZREF; aloft it's stronger, near the deck weaker. Stresses the wind estimator/guidance (which assume it's
// uniform) and means the descent sees a changing wind.                                                 // <<< SET
static const double SHEAR_ALPHA=0.14, WIND_ZREF=10.0;
static double windAt(double z){ return WIND*pow(fmax(z,0.3)/WIND_ZREF, SHEAR_ALPHA); }
static double TW_SCALE=1.0;                // ascent-motor thrust scale     -- runtime: --tw
static double LAND_SCALE=1.0;              // landing-motor thrust scale    -- runtime: --land (firmware still assumes nominal -> tests hoverslam robustness)
static double FIN_AUTH=1.0;                // TRUE margin-fin effectiveness vs the firmware's a-priori model -- runtime: --finauth
                                           //   (firmware keeps nominal S_FIN*CN_FIN*l_fin -> tests the coast torque-commander's robustness to fin-model error)
// ---- FLIGHT-READINESS dispersions & un-modeled failure modes (the Monte Carlo runner samples these) ----
// Each is a TRUE-plant perturbation the firmware does NOT know about, so a run tests robustness to build
// tolerance / part variation / real-world effects the 5-seed nominal sweep holds fixed.                 // <<< SET
static double MASS_MULT=1.0;     // --massmult : true all-up mass vs the firmware's assumed MASS_LAND_KG
static double IYY_MULT =1.0;     // --iyymult  : true pitch inertia vs the firmware's assumed IYY
static double CG_OFF   =0.0;     // --cgoff  m : build-tolerance CG shift (+ = forward = MORE static margin). Flight 1 died on static margin.
static double IGN_DELAY_MS=25.0; // --igndelay : mean pyro->thrust motor delay (spin-up). Real; the hoverslam is razor-edge sensitive.
static double IGN_JIT_MS  =12.0; // --ignjit   : 1-sigma scatter on that delay (can't be lead-compensated -> pure dispersion)
static double ACC_FSR_G   =16.0; // --accfsr   : accelerometer full-scale (g). Flight 1 RAILED at 2 g -> lost attitude.
static double GYRO_FSR_DPS=2000.0;//--gyrofsr  : gyro full-scale (deg/s). Flight 1 RAILED at 500 dps.
static double LOOP_JIT    =0.0;  // --jitter   : per-ms probability of a control-loop stall (SD write / I2C hang)
static double LOOP_STALL_MS=6.0; //             stall duration when it hits -> the firmware's millis() gates batch/skip (latency = the Pi killer)
// ---- 3-D roll-coupling error sources (the classic TVC roll-divergence mechanisms) ----
static double CG_OFF_X=0.0;      // --cgoffx m : CG laterally off the geometric (thrust) axis, body X. Thrust then
static double CG_OFF_Y=0.0;      // --cgoffy m :   misses the CG -> constant pitch/yaw trim moment AND, once the
                                 //   gimbal deflects, a ROLL torque M_z = T*(cgy*da - cgx*db): every TVC correction
                                 //   leaks into roll, and IZZ is ~65x smaller than IYY -> winds up fast.
static double ROLL_CANT=0.0;     // --rollcant m : nozzle tangential cant arm (m): constant roll torque = T*this
static double FIN_MIS=0.0;       // --finmis m^3 : fin misalignment roll coefficient: M_z += qdyn*this (aero roll trim)
// Ascent motor (T/W~1.7) and landing motor (T/W~4.5; aNet must match firmware's LAND/MASS_LAND)
static const double ASC_PLATEAU=12, ASC_PEAK=22, ASC_BURN=3.45;
static const double LAND_PLATEAU=18, LAND_PEAK=28, LAND_BURN=2.0;   // sized so decel nulls vz ~as it burns out at the ground
// Simulated sensor errors (so pad calibration has something to remove)
static const double SIM_GBIAS=0.010, SIM_GBIASY=-0.006, SIM_GBIASZ=0.008;          // rad/s per gyro axis
static const double SIM_ABIASX=0.20, SIM_ABIASY=-0.12, SIM_ABIASZ=0.15;            // m/s^2 per accel axis
static const double GYRO_N=0.002, ACC_N=0.05, BARO_N=0.30;              // noise sigmas
static const double BARO_REF=100.0;       // site elevation (firmware subtracts its own baseline)
static const double LAT0=32.7157, LON0=-117.1611;  // San Diego pad origin
static const double GPS_HZ=8.0;

// ============================================================================
//  Physics state — full 3-D 6DOF (body frame: +Z = nose/axial, X/Y lateral)
// ============================================================================
static double px=0, py=0, pz=0, vx=0, vy=0, vz=0;       // true position / velocity (world; Z up, X downrange)
static double Qw=1, Qx=0, Qy=0, Qz=0;                   // attitude quaternion, body -> world
static double wbx=0, wby=0, wbz=0;                      // body angular rates (rad/s): X pitch, Y yaw, Z ROLL
static double th_t=0, q_t=0;                            // legacy pitch-plane tilt / rate (derived, for logs)
static double lastAx=0, lastAy=0, lastAz=0;             // world non-grav accel (for sensor synthesis)
static double sfbX=0, sfbY=0, sfbZ=0;                   // body-frame specific force (for the IMU mock)
static double maxRollDps=0;                             // worst |roll rate| seen (summary diagnostic)
// quaternion helpers
static void qNorm(){ double n=sqrt(Qw*Qw+Qx*Qx+Qy*Qy+Qz*Qz); if(n>1e-12){Qw/=n;Qx/=n;Qy/=n;Qz/=n;} }
static void qRotB2W(double bx,double by,double bz,double&wx_,double&wy_,double&wz_){   // v_world = R(Q)*v_body
  double t2=Qw*Qx,t3=Qw*Qy,t4=Qw*Qz,t5=-Qx*Qx,t6=Qx*Qy,t7=Qx*Qz,t8=-Qy*Qy,t9=Qy*Qz,t10=-Qz*Qz;
  wx_=2*((t8+t10)*bx+(t6-t4)*by+(t3+t7)*bz)+bx;
  wy_=2*((t4+t6)*bx+(t5+t10)*by+(t9-t2)*bz)+by;
  wz_=2*((t7-t3)*bx+(t2+t9)*by+(t5+t8)*bz)+bz;
}
static void qRotW2B(double wx_,double wy_,double wz_,double&bx,double&by,double&bz){   // v_body = R(Q)^T*v_world
  double t2=Qw*Qx,t3=Qw*Qy,t4=Qw*Qz,t5=-Qx*Qx,t6=Qx*Qy,t7=Qx*Qz,t8=-Qy*Qy,t9=Qy*Qz,t10=-Qz*Qz;
  bx=2*((t8+t10)*wx_+(t6+t4)*wy_+(t7-t3)*wz_)+wx_;
  by=2*((t6-t4)*wx_+(t5+t10)*wy_+(t2+t9)*wz_)+wy_;
  bz=2*((t3+t7)*wx_+(t9-t2)*wy_+(t5+t8)*wz_)+wz_;
}
// --- Servo actuator dynamics (REALISM): rate-limited first-order for all four channels
// (TVC X, TVC Y, margin fold, roll differential).  // <<< SET
static double tvcRealDeg=TVC_NEUTRAL, tvc2RealDeg=TVC_NEUTRAL, marRealDeg=MARGIN_NEUTRAL, rollRealDeg=ROLL_NEUTRAL;
static const double SERVO_SLEW=150.0;    // deg/s max slew  // <<< SET (your servo)
static const double SERVO_TAU=0.03;      // s first-order lag
static void updateServos(double dt){
  double SLEW = getenv("SLEW")?atof(getenv("SLEW")):SERVO_SLEW;   // SIL sweep of servo speed
  double a=fmin(dt/SERVO_TAU,1.0);       // first-order approach
  const int pins[4]={SERVO_TVC_PIN,SERVO_TVC2_PIN,SERVO_MARGIN_PIN,SERVO_ROLL_PIN};
  double* real[4]={&tvcRealDeg,&tvc2RealDeg,&marRealDeg,&rollRealDeg};
  for(int k=0;k<4;k++){
    double step = a*(g_servoDeg[pins[k]]-*real[k]);
    double lim  = SLEW*dt;
    if(step> lim) step= lim; if(step<-lim) step=-lim;   // slew-rate clamp
    *real[k] += step;
  }
}
static bool   ascentLit=false, landLit=false, lifted=false, landed=false, legsOut=false;
static const double LEG_IYY_FACTOR=1.20;   // legs deploy -> +20% pitch inertia (CG shift itself is CG_LEGS_M)
static const double LEG_H=0.08;            // deployed legs contact the ground this far below the body reference (m).
                                           // Without this, contact is a knife-edge at exactly pz=0: a burn that nulls
                                           // v 4 cm up "hovers" and the un-cuttable motor relaunches it -- an artifact
                                           // of point-contact physics, not a real failure mode legs exist to absorb.
static double ascentT0=0, landT0=0;
static double maxPz=0, impactVz=0;
static bool   sawBoost=false, sawCoast=false, sawBurn=false;

static double frand(){ return (double)rand()/(double)RAND_MAX; }
static double gn(double s){ double u1=fmax(1e-12,frand()),u2=frand(); return s*sqrt(-2*log(u1))*cos(2*M_PI*u2); }

static double thrustCurve(double tau,double burn,double plateau,double peak){
  if(tau<0||tau>burn) return 0;
  if(tau<0.15) return peak*tau/0.15;
  if(tau<0.50) return peak-(peak-plateau)*(tau-0.15)/0.35;
  return plateau;
}
static double ascentThrust(){ return ascentLit? TW_SCALE*thrustCurve(g_micros/1e6-ascentT0,ASC_BURN,ASC_PLATEAU,ASC_PEAK):0; }
static double landThrust(){ return landLit? LAND_SCALE*thrustCurve(g_micros/1e6-landT0,LAND_BURN,LAND_PLATEAU,LAND_PEAK):0; }

// Solid motors don't light instantly: a real pyro + grain spin-up takes ~tens of ms with unit-to-unit scatter.
// Model it -> the motor lights IGN_DELAY_MS(+scatter) after the firmware raises the pin, so the burn starts lower
// than the firmware's causal predictor expects. The firmware lead-compensates the MEAN (IGN_LEAD_S); the SCATTER
// is irreducible dispersion the stop-margin must absorb.
static double ascReqT=-1, ascDelay=0, landReqT=-1, landDelay=0;
static void servicePyros(){
  double t=g_micros/1e6;
  if(g_pin[P_ASCENT]==HIGH && ascReqT<0){ ascReqT=t; ascDelay=fmax(0.0,(IGN_DELAY_MS+gn(IGN_JIT_MS))/1000.0); }
  if(ascReqT>=0 && !ascentLit && t>=ascReqT+ascDelay){ ascentLit=true; ascentT0=t; }
  if(g_pin[P_LAND]==HIGH && landReqT<0){ landReqT=t; landDelay=fmax(0.0,(IGN_DELAY_MS+gn(IGN_JIT_MS))/1000.0); }
  if(landReqT>=0 && !landLit && t>=landReqT+landDelay){ landLit=true; landT0=t; }
  if(g_pin[P_LEGS]==HIGH) legsOut=true;    // legs deploy -> CG/inertia shift (applied in stepPhysics)
}

static void stepPhysics(double dt){
  if(landed) return;   // frozen on the ground: don't re-integrate (gravity would re-cross pz<=0 every
                       // step and OVERWRITE impactVz with a ~-0.01 m/s re-impact, masking a hard landing)
  updateServos(dt);    // realized (slewed/lagged) servo angles, not the instant command
  // actuator channels (realized): TVC gimbal deflections in the body X-Z and Y-Z planes, margin fold, roll diff
  double uA  = (TVC_NEUTRAL - tvcRealDeg ) / TVC_DEG_PER_UNIT;
  double uB  = (TVC_NEUTRAL - tvc2RealDeg) / TVC_DEG_PER_UNIT;
  double dep = 0.5 + (marRealDeg -MARGIN_NEUTRAL)/(2.0*MARGIN_DEG_RANGE);
  double rDf =       (rollRealDeg-ROLL_NEUTRAL  )/(2.0*ROLL_DEG_RANGE);   // differential fin fold, +-0.5
  double thrust = ascentThrust() + landThrust();
  // Current mass/CG/inertia from propellant burnt so far (linear over each burn).
  double tnow=g_micros/1e6;
  double ascFrac  = ascentLit? fmin(fmax((tnow-ascentT0)/ASC_BURN ,0.0),1.0):0.0;
  double landFrac = landLit  ? fmin(fmax((tnow-landT0)  /LAND_BURN,0.0),1.0):0.0;
  double mass = (MASS_DRY + ASC_PROP*(1.0-ascFrac) + LAND_PROP*(1.0-landFrac))*MASS_MULT;   // MASS_MULT: build dispersion
  if(!lifted){
    if(ascentThrust() > mass*G){ lifted=true; }       // liftoff once thrust beats weight
    else { lastAx=0; lastAy=0; lastAz=0; sfbX=0; sfbY=0; sfbZ=G; return; }   // held vertical (reads +1g axial)
  }
  // wind vector (steady sheared + OU gusts, per axis)
  double wLx = windAt(pz), wLy = (WINDY!=0.0)? WINDY*pow(fmax(pz,0.3)/WIND_ZREF,SHEAR_ALPHA) : 0.0;
  if(GUSTS){ double sx=0.35*fabs(wLx), sy=0.35*fmax(fabs(wLy),0.5*fabs(wLx));
    wTurb  += (-wTurb /TAU_W)*dt + sx*sqrt(2.0/TAU_W)*sqrt(dt)*gn(1.0);
    wTurbY += (-wTurbY/TAU_W)*dt + sy*sqrt(2.0/TAU_W)*sqrt(dt)*gn(1.0); }
  // air-relative velocity, world -> body
  double vaw_x=vx-(wLx+wTurb), vaw_y=vy-(wLy+wTurbY), vaw_z=vz;
  double vbx,vby,vbz; qRotW2B(vaw_x,vaw_y,vaw_z,vbx,vby,vbz);
  double V=sqrt(vbx*vbx+vby*vby+vbz*vbz), Vs=fmax(V,0.1), qdyn=0.5*RHO*V*V;
  double vlat=sqrt(vbx*vbx+vby*vby);                    // body-frame crossflow magnitude (~V*sin(aoa))
  // Explicit MIGRATING CG, axial (+ = forward of nominal): propellant burns aft -> forward; legs -> aft.
  double xcg  = CG_PROP_M*(ascFrac+landFrac) - (legsOut? CG_LEGS_M:0.0) + CG_OFF;
  double lnoz = L_NOZZLE + xcg;
  double iyy  = IYY*(mass/(MASS*MASS_MULT))*IYY_MULT;
  double izz  = IZZ0*(mass/(MASS*MASS_MULT));
  if(legsOut){ iyy*=LEG_IYY_FACTOR; izz*=LEG_IZZ_FACTOR; }
  // THRUST in body frame (2-axis gimbal), applied at the nozzle. The nozzle sits on the GEOMETRIC axis; the CG
  // is laterally offset (CG_OFF_X/Y build error), so r_nozzle-from-CG = (-cgx,-cgy,-lnoz) and M = r x F gives
  //   M_x = T*( lnoz*db - cgy*Fz~ ),  M_y = T*( cgx - lnoz*da ),  M_z = T*( cgy*da - cgx*db )   <- TVC->ROLL leak
  // gimbal sign convention: positive command tilts the THRUST toward -x/-y (nozzle toward +x/+y), so the
  // moment about the CG rotates the NOSE toward +x/+y -- matching the firmware's keff>0 channel convention.
  double sa=sin(uA*CU_TO_RAD), sb=sin(uB*CU_TO_RAD), cz=sqrt(fmax(0.0,1.0-sa*sa-sb*sb));
  double Ftb_x=-thrust*sa, Ftb_y=-thrust*sb, Ftb_z=thrust*cz;
  double rnx=-CG_OFF_X, rny=-CG_OFF_Y, rnz=-lnoz;
  double Mtx = rny*Ftb_z - rnz*Ftb_y;
  double Mty = rnz*Ftb_x - rnx*Ftb_z;
  double Mtz = rnx*Ftb_y - rny*Ftb_x + thrust*ROLL_CANT;   // + nozzle tangential cant = constant roll torque under thrust
  // AERO: axial drag along -v_air; margin-modulation normal forces (body CP aft, fin CP fwd) from the crossflow.
  double Fdb_x=-qdyn*S_REF*CD*(vbx/Vs), Fdb_y=-qdyn*S_REF*CD*(vby/Vs), Fdb_z=-qdyn*S_REF*CD*(vbz/Vs);
  double l_b = L_BODYCP0 + xcg, l_f = L_FIN0 - xcg;
  double kB = qdyn*S_REF*CN_BODY/Vs;                     // body normal-force gain (per crossflow velocity)
  double kF = qdyn*S_FIN*CN_FIN*dep*FIN_AUTH/Vs;         // fin normal-force gain (scaled by fold-out)
  // body CP at z=-l_b: M += (0,0,-l_b) x F_body ; fin CP at z=+l_f: M += (0,0,+l_f) x F_fin. Net:
  // restoring toward nose-into-flow when the body term dominates, destabilizing (tail-first-seeking) when the
  // deployed fins do -- the same margin-modulation physics as the 2-D plant, now on the full crossflow vector.
  double Max = (l_b*kB - l_f*kF)*(-vby);
  double May = (l_b*kB - l_f*kF)*( vbx);
  // FIN ROLL (the user's mechanism): differential fold turns crossflow into roll torque, authority ~ sin(aoa) --
  // strong exactly at high aoa; near-zero pointing along the flow. + fin misalignment aero roll trim.
  double Mfz = qdyn*S_FIN*CN_FIN*(vlat/Vs)*R_FIN*rDf*FIN_AUTH*2.0 + qdyn*FIN_MIS;
  // damping
  double Mdx=-C_DAMP*wbx, Mdy=-C_DAMP*wby, Mdz=-2.0e-5*wbz;
  // Euler rigid-body equations (Ix=Iy=iyy, Iz=izz): gyroscopic roll-pitch coupling INCLUDED -- with spin wbz it
  // cross-couples the lateral axes (inertial roll coupling, the other half of why roll divergence kills TVC birds)
  double dwx = (Mtx+Max+Mdx + (iyy-izz)*wby*wbz)/iyy;
  double dwy = (Mty+May+Mdy + (izz-iyy)*wbx*wbz)/iyy;
  double dwz = (Mtz+Mfz+Mdz)/izz;
  wbx+=dwx*dt; wby+=dwy*dt; wbz+=dwz*dt;
  if(fabs(wbz)*180.0/M_PI>maxRollDps) maxRollDps=fabs(wbz)*180.0/M_PI;
  // quaternion kinematics: Qdot = 0.5*Q (x) (0,wb)
  double dQw=0.5*(-Qx*wbx-Qy*wby-Qz*wbz), dQx=0.5*(Qw*wbx+Qy*wbz-Qz*wby),
         dQy=0.5*(Qw*wby-Qx*wbz+Qz*wbx), dQz=0.5*(Qw*wbz+Qx*wby-Qy*wbx);
  Qw+=dQw*dt; Qx+=dQx*dt; Qy+=dQy*dt; Qz+=dQz*dt; qNorm();
  // translate: F_world = R*(thrust_body + drag_body) + gravity
  double Fbx=Ftb_x+Fdb_x, Fby=Ftb_y+Fdb_y, Fbz=Ftb_z+Fdb_z;
  double Fwx,Fwy,Fwz; qRotB2W(Fbx,Fby,Fbz,Fwx,Fwy,Fwz);
  double ax=Fwx/mass, ay=Fwy/mass, az=Fwz/mass-G;
  vx+=ax*dt; vy+=ay*dt; vz+=az*dt; px+=vx*dt; py+=vy*dt; pz+=vz*dt;
  lastAx=ax; lastAy=ay; lastAz=az;
  sfbX=Fbx/mass; sfbY=Fby/mass; sfbZ=Fbz/mass;          // body specific force for the IMU mock
  // legacy pitch-plane diagnostics (body axis tilt toward +x, and body-X rate)
  double zbx,zby,zbz; qRotB2W(0,0,1,zbx,zby,zbz);
  th_t=atan2(zbx,zbz); q_t=wbx;
  if(pz>maxPz) maxPz=pz;
  double gndH = legsOut? LEG_H : 0.0;        // deployed legs touch first
  if(pz<=gndH && vz<0){ impactVz=vz; pz=gndH; vx=vy=vz=0; wbx=wby=wbz=0; landed=true; lastAz=0; sfbZ=G; }
}

// Sensor timing/latency (REALISM): the old model handed the firmware perfect, instantaneous, full-rate sensors.
// Real parts add lag + slower rates + drift, which is what actually degrades the EKF.                  // <<< SET
static const double BARO_TAU=0.03, BARO_HZ=50.0;   // BMP280 first-order lag + output rate
static const double GYRO_RW=2.0e-5;                // gyro bias random-walk per ms (rad/s)
static const double GPS_LAT_S=0.20;                // GPS fix latency (s)
static double gbiasDriftX=0, gbiasDriftY=0, gbiasDriftZ=0;
static void updateSensors(){
  // body-frame specific force comes straight from the physics (sfbX/Y/Z), computed with the true quaternion.
  gbiasDriftX += gn(GYRO_RW); gbiasDriftY += gn(GYRO_RW); gbiasDriftZ += gn(GYRO_RW);
  // Finite sensor full-scale (FSR): a real IMU CLIPS beyond its range and the firmware then integrates a lie.
  // Flight 1 railed accel at 2 g and gyro at 500 dps and lost attitude; the recommended config is 16 g / 2000 dps.
  double gX = (wbx + SIM_GBIAS  + gbiasDriftX + gn(GYRO_N))*180.0/M_PI;
  double gY = (wby + SIM_GBIASY + gbiasDriftY + gn(GYRO_N))*180.0/M_PI;
  double gZ = (wbz + SIM_GBIASZ + gbiasDriftZ + gn(GYRO_N))*180.0/M_PI;
  double aX = (sfbX + SIM_ABIASX + gn(ACC_N))/G;
  double aY = (sfbY + SIM_ABIASY + gn(ACC_N))/G;
  double aZ = (sfbZ + SIM_ABIASZ + gn(ACC_N))/G;
  g_imu_gyroX = fmax(-GYRO_FSR_DPS, fmin(GYRO_FSR_DPS, gX));
  g_imu_gyroY = fmax(-GYRO_FSR_DPS, fmin(GYRO_FSR_DPS, gY));
  g_imu_gyroZ = fmax(-GYRO_FSR_DPS, fmin(GYRO_FSR_DPS, gZ));
  g_imu_accX  = fmax(-ACC_FSR_G,   fmin(ACC_FSR_G,   aX));
  g_imu_accY  = fmax(-ACC_FSR_G,   fmin(ACC_FSR_G,   aY));
  g_imu_accZ  = fmax(-ACC_FSR_G,   fmin(ACC_FSR_G,   aZ));
  // Barometer: first-order lag toward true altitude, sampled/held at BARO_HZ (not the 1 kHz truth).
  static double baroFilt=BARO_REF; static uint64_t lastBaro=0;
  baroFilt += fmin(0.001/BARO_TAU,1.0)*((pz+BARO_REF)-baroFilt);
  if(g_micros-lastBaro >= (uint64_t)(1e6/BARO_HZ)){ lastBaro=g_micros; g_baro_alt = baroFilt + gn(BARO_N); }
}

static double pxHist[256], pyHist[256]; static int pxHistIdx=0;   // 1 ms samples for GPS latency
static void feedGPS(){
  static uint64_t last=0;
  if(g_micros-last < (uint64_t)(1e6/GPS_HZ)) return;
  last=g_micros;
  int back=(int)(GPS_LAT_S*1000.0);                  // report the position from GPS_LAT_S ago (fix latency)
  int idx=(pxHistIdx-back-1+256*4)&255;
  double pxDelayed = pxHist[idx], pyDelayed = pyHist[idx];
  double lon = LON0 + pxDelayed/(cos(LAT0*M_PI/180.0)*111320.0);
  double lat = LAT0 + pyDelayed/111320.0;            // crossrange -> latitude (3-D: firmware fuses x AND y)
  double aLat=fabs(lat), aLon=fabs(lon);
  int latD=(int)aLat; double latM=(aLat-latD)*60.0;
  int lonD=(int)aLon; double lonM=(aLon-lonD)*60.0;
  char s[110];
  snprintf(s,sizeof(s),"$GPGGA,120000.00,%02d%07.4f,%c,%03d%07.4f,%c,1,08,0.9,%.1f,M,0,M,,*00\r\n",
           latD,latM,(lat>=0?'N':'S'), lonD,lonM,(lon>=0?'E':'W'), pz+BARO_REF);
  Serial1.feed(s);
}

// Automatic button presses (sim seconds): 1 to arm-mode, then 5 rapid to start countdown.
static void serviceButton(){
  static const double T[] = {0.5, 1.0,1.25,1.5,1.75,2.0};
  static int idx=0; static uint64_t releaseUs=0;
  double t=g_micros/1e6;
  if(idx<6 && t>=T[idx]){ g_pin[BTN]=HIGH; releaseUs=g_micros+40000; idx++; }
  if(g_pin[BTN]==HIGH && g_micros>=releaseUs) g_pin[BTN]=LOW;
}

static void logRow(){
  // legacy 2-D columns first (existing tools keep working), 3-D columns appended after windTrue
  printf("%.3f,%d,%s,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.2f,%d,%d,%.3f,%.3f,"
         "%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f\n",
    g_micros/1e6,(int)state,stN(state),
    px,pz,vx,vz, th_t*180.0/M_PI, q_t*180.0/M_PI,
    estX,estZ,estVx,estVz, th*180.0/M_PI, u_tvc, deploy, keff_est,
    ascentThrust()+landThrust(), (int)(g_pin[P_LEGS]), (int)(g_pin[P_CHUTE]),
    windEst, windAt(pz)+wTurb,
    py, vy, estY, rollTwist*180.0/M_PI, wbz*180.0/M_PI, u_tvc2, rollDef, windEstY,
    2.0*atan2(Qz,Qw)*180.0/M_PI,     // TRUE twist: est-vs-true roll comparison
    Qw,Qx,Qy,Qz,                     // TRUE attitude quaternion (for the 3-D renderer)
    // ACTUAL servo-limited fin positions (the fins the physics actually uses) -> the viewer shows the real
    // smooth fin, not the chattering raw command (deploy/rollDef). marRealDeg/rollRealDeg are slew+lag limited.
    fmax(0.0,fmin(1.0, 0.5+(marRealDeg-MARGIN_NEUTRAL)/(2.0*MARGIN_DEG_RANGE))),
    (rollRealDeg-ROLL_NEUTRAL)/(2.0*ROLL_DEG_RANGE));
}

void sim_advance(unsigned long ms){
  for(unsigned long k=0;k<ms;k++){
    servicePyros();
    stepPhysics(0.001);
    g_micros += 1000;
    pxHist[pxHistIdx]=px; pyHist[pxHistIdx]=py; pxHistIdx=(pxHistIdx+1)&255;   // GPS latency history
    updateSensors();
    feedGPS();
    serviceButton();
    if(state==St::BOOST) sawBoost=true;
    if(state==St::COAST) sawCoast=true;
    if(state==St::LANDING_BURN) sawBurn=true;
    if(lifted && vz<0 && nrN<48 && (nrN==0 || nrZ[nrN-1]-pz>1.5)){ nrZ[nrN]=pz; nrX[nrN]=px; nrVx[nrN]=vx; nrN++; }  // record descent (nominal)
    static uint64_t lastLog=0;   // log from liftoff through landing (sawBoost stays true post-TD so legs/chute frames record)
    if(sawBoost && g_micros-lastLog >= 20000){ lastLog=g_micros; logRow(); }
  }
}

int main(int argc, char** argv){
  unsigned seed=1;
  for(int i=1;i<argc;i++){                   // runtime knobs for weakness-hunting
    if(!strcmp(argv[i],"--seed")  && i+1<argc) seed=(unsigned)atoi(argv[++i]);
    else if(!strcmp(argv[i],"--wind")  && i+1<argc) WIND=atof(argv[++i]);
    else if(!strcmp(argv[i],"--tw")    && i+1<argc) TW_SCALE=atof(argv[++i]);
    else if(!strcmp(argv[i],"--land")  && i+1<argc) LAND_SCALE=atof(argv[++i]);
    else if(!strcmp(argv[i],"--finauth")&&i+1<argc) FIN_AUTH=atof(argv[++i]);
    else if(!strcmp(argv[i],"--massmult")&&i+1<argc) MASS_MULT=atof(argv[++i]);
    else if(!strcmp(argv[i],"--iyymult")&&i+1<argc) IYY_MULT=atof(argv[++i]);
    else if(!strcmp(argv[i],"--cgoff")  &&i+1<argc) CG_OFF=atof(argv[++i]);
    else if(!strcmp(argv[i],"--igndelay")&&i+1<argc) IGN_DELAY_MS=atof(argv[++i]);
    else if(!strcmp(argv[i],"--ignjit") &&i+1<argc) IGN_JIT_MS=atof(argv[++i]);
    else if(!strcmp(argv[i],"--accfsr") &&i+1<argc) ACC_FSR_G=atof(argv[++i]);
    else if(!strcmp(argv[i],"--gyrofsr")&&i+1<argc) GYRO_FSR_DPS=atof(argv[++i]);
    else if(!strcmp(argv[i],"--jitter") &&i+1<argc) LOOP_JIT=atof(argv[++i]);
    else if(!strcmp(argv[i],"--windy")  &&i+1<argc) WINDY=atof(argv[++i]);
    else if(!strcmp(argv[i],"--cgoffx") &&i+1<argc) CG_OFF_X=atof(argv[++i]);
    else if(!strcmp(argv[i],"--cgoffy") &&i+1<argc) CG_OFF_Y=atof(argv[++i]);
    else if(!strcmp(argv[i],"--rollcant")&&i+1<argc) ROLL_CANT=atof(argv[++i]);
    else if(!strcmp(argv[i],"--finmis") &&i+1<argc) FIN_MIS=atof(argv[++i]);
    else if(!strcmp(argv[i],"--kick")  && i+1<argc) g_kickDeg=atof(argv[++i]);
    else if(!strcmp(argv[i],"--target")&& i+1<argc) g_targetX=atof(argv[++i]);
    else if(!strcmp(argv[i],"--nomout")&& i+1<argc) nomOutPath=argv[++i];
    else if(!strcmp(argv[i],"--nomin") && i+1<argc) nomInPath=argv[++i];
    else if(!strcmp(argv[i],"--nodivert")) g_divertOn=false;
    else if(!strcmp(argv[i],"--nomassid")) g_massIdOn=false;
    else if(!strcmp(argv[i],"--massid"))   g_massIdOn=true;
    else if(!strcmp(argv[i],"--gusts")) GUSTS=true;
  }
  if(nomInPath){ FILE* f=fopen(nomInPath,"r"); if(f){ int n=0; double z,x,v;   // load nominal descent into the firmware
    while(n<48 && fscanf(f,"%lf %lf %lf",&z,&x,&v)==3){ g_nomZ[n]=(float)z; g_nomX[n]=(float)x; g_nomVx[n]=(float)v; n++; }
    g_nomN=n; fclose(f); } }
  if(getenv("WC")) ADRC_WC=atof(getenv("WC"));
  if(getenv("W0")) ADRC_W0=atof(getenv("W0"));
  srand(seed);
  for(int i=0;i<64;i++) g_servoDeg[i]=90;   // neutral before the firmware writes
  updateSensors();                          // pad readings ready for the first loop()
  setup();                                  // firmware setup()
  printf("t,st,state,px,pz,vx,vz,th_deg,q_dps,estX,estZ,estVx,estVz,thEst_deg,uTVC,deploy,keff,thrust,legs,chute,windEst,windTrue,"
         "py,vy,estY,roll_deg,rollrate_dps,uTVC2,rollDef,windEstY,rollTrue_deg,Qw,Qx,Qy,Qz,depReal,rollDefReal\n");

  const uint64_t TMAX = 90ULL*1000000ULL;
  uint64_t termAt = 0;
  while(g_micros < TMAX){
    loop();
    sim_advance(1);
    // Control-loop stall injection: the physical world keeps advancing (sensors update) but the firmware's
    // loop() is NOT called for LOOP_STALL_MS -> its millis()-gated control/EKF batch and resume late, exactly
    // like a blocking SD write or an I2C hang. Latency is what compresses the gain ceiling (the Pi thesis).
    if(LOOP_JIT>0.0 && frand()<LOOP_JIT){
      unsigned long stall=(unsigned long)fmax(1.0, LOOP_STALL_MS*(0.5+frand()));
      sim_advance(stall);
    }
    if((state==St::TOUCHDOWN || state==St::ABORTED)){
      if(termAt==0) termAt=g_micros;
      // Run until the vehicle has TRULY contacted the ground (physics `landed`) so a solid-motor HOP -- null vz
      // above the deck, the un-cuttable motor lifts back up, burn out, free-fall -- is captured as the real impact,
      // not cut off mid-hop (which used to read a false vz~0). Hard cap so a persistent hop can't run forever.
      if(landed || g_micros-termAt > 6000000ULL) break;
    }
  }

  // ---- summary to stderr ----
  fprintf(stderr,"\n================ SIL SUMMARY ================\n");
  fprintf(stderr,"final state     : %s\n", stN(state));
  fprintf(stderr,"phases reached  : BOOST=%d COAST=%d LANDING_BURN=%d\n", sawBoost,sawCoast,sawBurn);
  fprintf(stderr,"apogee          : %.1f m\n", maxPz);
  double miss2d = sqrt((px-g_targetX)*(px-g_targetX)+py*py);
  double zbx,zby,zbz; qRotB2W(0,0,1,zbx,zby,zbz);
  double tiltTot = acos(fmax(-1.0,fmin(1.0,zbz)));
  fprintf(stderr,"touchdown x     : %.2f m   (target %.1f, miss %.2f)\n", px, g_targetX, px-g_targetX);
  fprintf(stderr,"touchdown y     : %.2f m   (2-D miss %.2f)\n", py, miss2d);
  fprintf(stderr,"touchdown speed : %.2f m/s (impact vz)\n", fabs(impactVz));
  fprintf(stderr,"final tilt      : %.1f deg (true)\n", th_t*180.0/M_PI);
  fprintf(stderr,"final tilt 3d   : %.1f deg   max |roll rate| %.0f dps\n", tiltTot*180.0/M_PI, maxRollDps);
  fprintf(stderr,"est vs true @ TD: estZ=%.2f (true %.2f)  estX=%.2f (true %.2f)  estY=%.2f (true %.2f)\n", estZ,pz,estX,px,estY,py);
  fprintf(stderr,"keff (RLS)      : %.2f\n", keff_est);
  bool ok = (state==St::TOUCHDOWN) && sawBoost && sawCoast && sawBurn && fabs(impactVz)<5.0 && miss2d<5.0
            && tiltTot<0.5 && maxRollDps<540.0;   // 3-D gates: planar 2-D miss, TOTAL tilt, roll never ran away
  fprintf(stderr,"VERDICT         : %s\n", ok? "PASS (flew the full profile, soft+on-target)" : "see trace (did not meet all pass gates)");
  fprintf(stderr,"=============================================\n");
  if(nomOutPath){ FILE* f=fopen(nomOutPath,"w");   // write the recorded gust-free descent as the nominal
    if(f){ for(int i=0;i<nrN;i++) fprintf(f,"%.3f %.3f %.3f\n",nrZ[i],nrX[i],nrVx[i]); fclose(f); } }
  return 0;
}
