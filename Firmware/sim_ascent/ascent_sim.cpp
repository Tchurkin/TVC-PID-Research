/*
  ascent_sim.cpp — Software-In-the-Loop harness for the EXACT Ascent_TVC.ino flight code.
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
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
HardwareSerial Serial(false);      // set true to echo firmware Serial to stderr (--verbose)
HardwareSerial Serial1(false);
TwoWire  Wire;
SPIClass SPI;
SDClass  SD;
EEPROMClass EEPROM;                // brownout-persistence backing store (RAM here; blank at every run start)

// ---- firmware symbols we override to sweep gains (Ascent_TVC.ino: plain float, not constexpr) ----
extern float P_GAIN, D_GAIN, I_GAIN;
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
static double thrustAt(double tau){
  if(tau<0 || tau>BURN) return 0.0;
  if(tau<0.15) return 26.0*tau/0.15;
  if(tau<0.50) return 26.0-(26.0-14.0)*(tau-0.15)/0.35;
  return 14.0;
}

// ---- physics state (quaternion attitude, body rates rad/s) ----
static double Qw=1,Qx=0,Qy=0,Qz=0;         // body->world attitude
static double wbx=0,wby=0,wbz=0;           // body rates: X pitch, Y yaw, Z ROLL
static double alt=0, vz=0;
static bool ignited=false, lifted=false; static double ignT=-1;
static double servoXa=90, servoYa=90;      // slew-limited actual servo (deg)
static bool p4Fired=false, p1Fired=false; static double p4Tilt=0,p4Alt=0,p4Vz=0; static bool p4AtDescent=false;
static double apogee=0, maxTilt=0, maxRate=0, mtBoost=0, mrBoost=0;   // *Boost = while thrust ON (control has authority)
static double boostEndTilt=0;                                        // tiltTRUE at the LAST thrust-on sample (~burnout) =
                                                                    //   the STEADY-STATE lean the integral trim winds down
static double maxRollAng=0, maxRollRate=0;                           // roll diagnostics (deg, dps)
static uint32_t rngState=1;
static double urand(){ rngState=rngState*1664525u+1013904223u; return ((rngState>>8)&0xFFFF)/32768.0-1.0; } // [-1,1)
static jmp_buf endJmp;

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

static void synthSensors(){
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
  double gx =  wbx*180.0/M_PI + d_gbiasX + d_gnoise*urand();
  double gy = -wby*180.0/M_PI + d_gbiasY + d_gnoise*urand();
  double gz =  wbz*180.0/M_PI + d_gbiasZ + d_gnoise*urand();
  auto clampd=[](double v,double lim){ return v>lim?lim:(v<-lim?-lim:v); };
  g_imu_gyroX=clampd(gx,2000.0); g_imu_gyroY=clampd(gy,2000.0); g_imu_gyroZ=clampd(gz,2000.0);
  // accel = body-frame specific force f = R^T*(a_world + (0,0,G)). Vertical-only translation -> a_world=(0,0,az).
  double azw = lifted ? (T/((MASS+0)) - G - 0.5*RHO*S_REF*CD*vz*fabs(vz)/MASS) : 0.0;
  double fbx,fby,fbz; qRotW2B(0,0, azw+G, fbx,fby,fbz);
  g_imu_accX=clampd(fbx/G,16.0); g_imu_accY=clampd(fby/G,16.0); g_imu_accZ=clampd(fbz/G,16.0);
  g_baro_alt = alt + 0.15*urand();
}

void sim_advance(unsigned long ms){
  if(!ignited && g_pin[P3]!=1){                    // on-pad phase (pre-ignition): nothing moves -> fast-path
    g_micros += (uint64_t)ms*1000ULL;
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
    if(!ignited && g_pin[P3]==1){ ignited=true; ignT=now; }
    if(g_pin[P4]==1 && !p4Fired){ p4Fired=true; double zx,zy,zz; qRotB2W(0,0,1,zx,zy,zz);
      p4Tilt=acos(zz>1?1:(zz<-1?-1:zz))*180.0/M_PI; p4Alt=alt; p4Vz=vz; p4AtDescent=(vz<0); }
    if(g_pin[P1]==1) p1Fired=true;

    double T=0; if(ignited){ double tau=now-ignT-IGN_DELAY-d_ignLag; if(tau>=0) T=thrustAt(tau)*d_thrust; }  // d_ignLag shifts the
    // whole burn later, so the motor can still be running when the firmware's fixed TVC window closes
    if(!lifted){                                   // rail-constrained: no motion until thrust beats weight
      if(T>MASS*G){ lifted=true; wbx=d_tipX*M_PI/180; wby=d_tipY*M_PI/180; }   // rail-exit tip-off RATE
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
      double iyy=IYY*d_iyy, izz=IZZ0*d_izz, L=L_ARM*d_L;
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
      double az = T*zbz/MASS - G - 0.5*RHO*S_REF*CD*vz*fabs(vz)/MASS;
      vz+=az*dt; alt+=vz*dt; if(alt<0){alt=0; if(vz<0)vz=0;}
    }
    if(alt>apogee) apogee=alt;
    synthSensors();
    if(trace && ((g_micros/1000)%50==0)){ double zx,zy,zz; qRotB2W(0,0,1,zx,zy,zz);
      double tiltTrue=acos(zz>1?1:(zz<-1?-1:zz))*180/M_PI, rollTrue=2*atan2(Qz,Qw)*180/M_PI;
      double estTilt=sqrt((double)gyro_x*gyro_x+(double)gyro_y*gyro_y);
      fprintf(stderr,"t=%.2f T=%.1f | tiltTRUE=%.1f tiltEST=%.1f | altTRUE=%.1f altEST=%.1f | vzTRUE=%.2f vzEST=%.2f | roll=%.0f\n",
        now,T, tiltTrue,estTilt, alt,(double)estZ, vz,(double)estVz, rollTrue); }
    // ---- end conditions ----
    if(p4Fired && (now - (ignited?ignT:0) > 0.2)) longjmp(endJmp,1);   // recovery deployed -> done
    if(now>60.0) longjmp(endJmp,2);                                     // hard timeout
  }
}

// ---- Arduino core ----
unsigned long millis(){ return (unsigned long)(g_micros/1000ULL); }
unsigned long micros(){ return (unsigned long)g_micros; }
void delay(unsigned long ms){ sim_advance(ms); }
void delayMicroseconds(unsigned int us){ sim_advance((us+999)/1000); }
void pinMode(int,int){}
void digitalWrite(int p,int v){ if(p>=0&&p<64)g_pin[p]=v; }
void analogReadResolution(int){}
int  analogRead(int){ return 0; }
void tone(int,unsigned int){}
void noTone(int){}
// button auto-arm: deliver press pulses in the first 3 s (arming window); LOW afterwards so the firmware's
// recovery loop `while(digitalRead(BUTTON)==LOW)` enters and lets the longjmp fire.
static bool btnHeld=false;
int digitalRead(int p){
  if(p==BUTTON){
    if(g_micros > 3000000ULL) return 0;
    if(btnHeld){ btnHeld=false; return 0; }
    btnHeld=true; return 1;
  }
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
  SERVO_TAU=getarg(argc,argv,"--servotau",SERVO_TAU);   // s, actuator lag -- the gain-limiting unknown
  MASS=getarg(argc,argv,"--mass",MASS);        // kg, all-up liftoff mass -- affects trajectory/apogee (and hence the
                                               // backup-chute timing check), not the rotational control test
  d_ignLag=getarg(argc,argv,"--ignlag",0);      // s, EXTRA ignition lag beyond the assumed 0.2 s                                       // aero roll trim coeff
  d_seed=(unsigned)getarg(argc,argv,"--seed",1); rngState=d_seed?d_seed:1;
  for(int i=1;i<argc;i++){ if(!strcmp(argv[i],"--verbose")){verbose=true;Serial=HardwareSerial(true);} if(!strcmp(argv[i],"--trace"))trace=true; }
  // gain sweep (adjudicate the ASC007 hot gains): env PGAIN/DGAIN or --pgain/--dgain override the firmware defaults
  double pg=getarg(argc,argv,"--pgain",-1), dg=getarg(argc,argv,"--dgain",-1), ig=getarg(argc,argv,"--igain",-1);
  if(getenv("PGAIN")) pg=atof(getenv("PGAIN"));
  if(getenv("DGAIN")) dg=atof(getenv("DGAIN"));
  if(getenv("IGAIN")) ig=atof(getenv("IGAIN"));

  // --seedphase N : pre-load the EEPROM brownout record as if the board had just rebooted mid-flight with
  // phase N (1=boost, 2=coast). SAFETY TEST: the vehicle here is sitting still on the pad, so looksAirborne()
  // must reject it and the firmware must fall through to a NORMAL flight -- a stale record must never fire a
  // pyro in someone's hands. Expect the outcome to be identical to the un-seeded run.
  int seedPhase=(int)getarg(argc,argv,"--seedphase",0);
  if(seedPhase>0){
    struct SeedRec{ uint32_t magic; uint8_t phase; uint16_t boots; float gbx,gby,gbz; float groundAlt; uint32_t sum; } r{};
    r.magic=0x54564332; r.phase=(uint8_t)seedPhase; r.boots=0;
    r.gbx=r.gby=r.gbz=0; r.groundAlt=0;
    const uint8_t* b=(const uint8_t*)&r; uint32_t h=2166136261u;          // same FNV-1a the firmware uses
    for(size_t i=0;i<sizeof(SeedRec)-sizeof(uint32_t);i++){ h^=b[i]; h*=16777619u; }
    r.sum=h; EEPROM.put(0,r);
    fprintf(stderr,"[seeded EEPROM brownout record: phase=%d]\n",seedPhase);
  }

  int why = setjmp(endJmp);
  if(why==0){ setup(); if(pg>=0) P_GAIN=(float)pg; if(dg>=0) D_GAIN=(float)dg; if(ig>=0) I_GAIN=(float)ig; loop(); why=3; }

  // classify on the BOOST-phase TRUE tilt (the control test)
  const char* outcome;
  if(!p4Fired || why==2)                 outcome="NORECOVERY";
  else if(mtBoost>=44.0)                 outcome="ABORT_TUMBLE";   // boost tilt hit the firmware's 45 deg emergency
  else if(apogee<3.0)                    outcome="NOFLY";
  else if(mtBoost<10.0)                  outcome="PASS";
  else                                   outcome="MARGINAL";
  printf("outcome=%-12s boostTilt=%6.2f boostRate=%6.1f endlean=%6.2f coastTilt=%6.2f apogee=%6.1f p4descent=%d "
         "roll=%6.0f rollrate=%6.0f "
         "iyy=%.2f izz=%.2f thrust=%.2f larm=%.2f rollcant=%.5f cgoffx=%.4f cgoffy=%.4f misx=%.2f misy=%.2f sm=%.4f "
         "slew=%.0f pgain=%.3f dgain=%.3f igain=%.3f seed=%u\n",
         outcome,mtBoost,mrBoost,boostEndTilt,maxTilt,apogee,(int)p4AtDescent,
         maxRollAng,maxRollRate,
         d_iyy,d_izz,d_thrust,d_L,d_rollCant,d_cgOffX,d_cgOffY,d_misX,d_misY,d_sm,
         d_servoSlew,(double)P_GAIN,(double)D_GAIN,(double)I_GAIN,d_seed);
  return 0;
}
