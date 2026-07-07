/*
  ascent_sim.cpp — Software-In-the-Loop harness for the EXACT Ascent_TVC.ino flight code.
  Runs setup()/loop() unmodified against a 2-axis attitude + vertical-trajectory physics model:
   - feeds synthetic IMU (gyro deg/s, Y inverted to match the firmware's -getGyroY) + baro,
   - captures the servo writes -> gimbal -> restoring torque (sign matches the BENCH-verified
     SERVO_SIGN=+1, so a correct firmware stabilizes; the SIL then tests dynamics + robustness),
   - auto-arms (button queue) and runs a full flight: pad -> boost(TVC) -> coast -> apogee -> recovery.
  Dispersions via argv make it a Monte-Carlo robustness rig.  Build: see build.sh in this dir.
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
HardwareSerial Serial(false);      // set true to echo firmware Serial to stderr (--verbose)
HardwareSerial Serial1(false);
TwoWire  Wire;
SPIClass SPI;
SDClass  SD;

// ---- vehicle (matches Ascent_TVC.ino) ----
static const int    SERVO_X_PIN=4, SERVO_Y_PIN=3, P1=9, P3=11, P4=12, BUTTON=14;
static const double G=9.80665, SERVO_MULT=5.0, IGN_DELAY=0.2, BURN=3.45;
static double IYY=0.0078, L_ARM=0.14, MASS=0.818;
static const double S_REF=0.003, CD=0.6, RHO=1.20;

// ---- dispersions (set from argv) ----
static double d_iyy=1, d_thrust=1, d_L=1, d_tipX=0, d_tipY=0, d_misX=0, d_misY=0;
static double d_gnoise=0, d_gbiasX=0, d_gbiasY=0, d_servoSlew=400, d_gustX=0, d_gustY=0, d_gustHz=0.7;
static unsigned d_seed=1;
static bool verbose=false, trace=false;

// F-15 thrust (N) vs time since ignition; avg ~14.5 N, 3.45 s
static double thrustAt(double tau){
  if(tau<0 || tau>BURN) return 0.0;
  if(tau<0.15) return 26.0*tau/0.15;
  if(tau<0.50) return 26.0-(26.0-14.0)*(tau-0.15)/0.35;
  return 14.0;
}

// ---- physics state ----
static double thX=0,thY=0, rX=0,rY=0;     // tilt (rad), rate (rad/s)
static double alt=0, vz=0;
static bool ignited=false, lifted=false; static double ignT=-1;
static double servoXa=90, servoYa=90;      // slew-limited actual servo (deg)
static bool p4Fired=false, p1Fired=false; static double p4Tilt=0,p4Alt=0,p4Vz=0; static bool p4AtDescent=false;
static double apogee=0; static double maxTilt=0, maxRate=0, mtBoost=0, mrBoost=0;   // *Boost = while thrust is ON (control has authority)
static uint32_t rngState=1;
static double urand(){ rngState=rngState*1664525u+1013904223u; return ((rngState>>8)&0xFFFF)/32768.0-1.0; } // [-1,1)

static jmp_buf endJmp;

void sim_advance(unsigned long ms){
  if(!ignited && g_pin[P3]!=1){                    // on-pad phase (pre-ignition): nothing moves -> fast-path
    g_micros += (uint64_t)ms*1000ULL;
    g_imu_gyroX=d_gbiasX; g_imu_gyroY=d_gbiasY; g_imu_gyroZ=0;   // pad: rest + residual gyro bias (for the pre-launch check)
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
    if(g_pin[P4]==1 && !p4Fired){ p4Fired=true; p4Tilt=sqrt(thX*thX+thY*thY)*180/M_PI; p4Alt=alt; p4Vz=vz; p4AtDescent=(vz<0); }
    if(g_pin[P1]==1) p1Fired=true;

    double T=0; if(ignited){ double tau=now-ignT-IGN_DELAY; if(tau>=0) T=thrustAt(tau)*d_thrust; }
    if(!lifted){                                   // rail-constrained: no motion until thrust beats weight
      if(T>MASS*G){ lifted=true; rX=d_tipX*M_PI/180; rY=d_tipY*M_PI/180; }   // tip-off rate at rail exit
    } else {
      // servo actual (slew-limited toward the firmware's command)
      double cmdX=g_servoDeg[SERVO_X_PIN], cmdY=g_servoDeg[SERVO_Y_PIN], mx=d_servoSlew*dt;
      servoXa += (cmdX-servoXa> mx)? mx : (cmdX-servoXa<-mx? -mx : cmdX-servoXa);
      servoYa += (cmdY-servoYa> mx)? mx : (cmdY-servoYa<-mx? -mx : cmdY-servoYa);
      // gimbal -> restoring torque.  keff = T*L/IYY.  servo>90 (firmware's +tilt response) must reduce theta.
      double keff = T*(L_ARM*d_L)/(IYY*d_iyy);
      double link = 1.0/SERVO_MULT;                                    // TVC deg per servo deg
      double gustX = d_gustX*sin(2*M_PI*d_gustHz*now), gustY=d_gustY*cos(2*M_PI*d_gustHz*now*1.3);
      double aX = -keff*((servoXa-90)*link*M_PI/180) + keff*(d_misX*M_PI/180) + gustX;
      double aY = -keff*((servoYa-90)*link*M_PI/180) + keff*(d_misY*M_PI/180) + gustY;
      rX+=aX*dt; rY+=aY*dt; thX+=rX*dt; thY+=rY*dt;
      double v=fabs(vz), az = T/MASS - G - 0.5*RHO*S_REF*CD*vz*v/MASS;
      vz+=az*dt; alt+=vz*dt; if(alt<0){alt=0; if(vz<0)vz=0;}
    }
    if(alt>apogee) apogee=alt;
    double tm=sqrt(thX*thX+thY*thY)*180/M_PI, rm=sqrt(rX*rX+rY*rY)*180/M_PI;
    if(tm>maxTilt)maxTilt=tm; if(rm>maxRate)maxRate=rm;
    if(T>0.5){ if(tm>mtBoost)mtBoost=tm; if(rm>mrBoost)mrBoost=rm; }   // boost-phase (control-active) peaks
    // ---- synthesize sensors ----  gyro deg/s (Y INVERTED to match -getGyroY); accel = gravity projection
    g_imu_gyroX =  rX*180/M_PI + d_gbiasX + d_gnoise*urand();
    g_imu_gyroY = -(rY*180/M_PI) + d_gbiasY + d_gnoise*urand();
    g_imu_gyroZ = 0;
    g_imu_accX = -sin(thY); g_imu_accY = sin(thX); g_imu_accZ = cos(sqrt(thX*thX+thY*thY));
    g_baro_alt = alt + 0.15*urand();
    if(trace && ((g_micros/1000)%50==0)) fprintf(stderr,"t=%.2f alt=%.1f tilt=%.2f srvX=%.1f srvY=%.1f T=%.1f\n",now,alt,tm,servoXa,servoYa,T);
    // ---- end conditions ----
    if(p4Fired && (now - (ignited?ignT:0) > 0.2)) longjmp(endJmp,1);   // recovery deployed -> done
    if(now>60.0) longjmp(endJmp,2);                                     // hard timeout (past 30 s countdown + flight)
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
// button auto-arm: deliver press pulses (HIGH once, then release) ONLY in the first 3 s (the arming window);
// LOW afterwards so the firmware's recovery loop `while(digitalRead(BUTTON)==LOW)` enters and lets the longjmp fire.
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
  d_iyy=getarg(argc,argv,"--iyy",1); d_thrust=getarg(argc,argv,"--thrust",1); d_L=getarg(argc,argv,"--larm",1);
  d_tipX=getarg(argc,argv,"--tipx",0); d_tipY=getarg(argc,argv,"--tipy",0);
  d_misX=getarg(argc,argv,"--misx",0); d_misY=getarg(argc,argv,"--misy",0);
  d_gnoise=getarg(argc,argv,"--gnoise",0); d_gbiasX=getarg(argc,argv,"--gbiasx",0); d_gbiasY=getarg(argc,argv,"--gbiasy",0);
  d_servoSlew=getarg(argc,argv,"--slew",400); d_gustX=getarg(argc,argv,"--gustx",0); d_gustY=getarg(argc,argv,"--gusty",0);
  d_seed=(unsigned)getarg(argc,argv,"--seed",1); rngState=d_seed?d_seed:1;
  for(int i=1;i<argc;i++){ if(!strcmp(argv[i],"--verbose")){verbose=true;Serial=HardwareSerial(true);} if(!strcmp(argv[i],"--trace"))trace=true; }

  int why = setjmp(endJmp);
  if(why==0){ setup(); loop(); why=3; }   // why=3 = loop() returned on its own (shouldn't happen)

  // classify
  const char* outcome;                                            // classify on the BOOST-phase tilt (control test)
  if(!p4Fired || why==2)                 outcome="NORECOVERY";
  else if(mtBoost>=44.0)                 outcome="ABORT_TUMBLE";   // boost tilt hit the firmware's 45 deg emergency -> control lost
  else if(apogee<3.0)                    outcome="NOFLY";
  else if(mtBoost<10.0)                  outcome="PASS";
  else                                   outcome="MARGINAL";
  printf("outcome=%-12s boostTilt=%6.2f boostRate=%6.1f coastTilt=%6.2f apogee=%6.1f p4descent=%d "
         "iyy=%.2f thrust=%.2f larm=%.2f tipx=%.1f tipy=%.1f misx=%.2f misy=%.2f slew=%.0f gust=%.1f seed=%u\n",
         outcome,mtBoost,mrBoost,maxTilt,apogee,(int)p4AtDescent,d_iyy,d_thrust,d_L,d_tipX,d_tipY,d_misX,d_misY,
         d_servoSlew,sqrt(d_gustX*d_gustX+d_gustY*d_gustY),d_seed);
  return 0;
}
