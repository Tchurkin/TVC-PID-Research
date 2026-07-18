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

  Vehicle (MEASURED 2026-07-06): IYY = 0.0078 kg*m^2 (wet, one motor), mass = 0.818 kg,
  CG->TVC-axis L = 0.14 m, F-15 motor, gimbal 5:1 both axes.
  GAINS: POLE-PLACED (P=0.249, D=0.062) for keff = T*L/IYY = 257, target zeta=1.0 / wn=8 rad/s;
  closed-loop SIL-validated (0.6 s settle, 0% overshoot, robust across the burn + a slow servo).
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

// -- Types --------------------------------------------------------------------
// Defined before ALL functions: the Arduino IDE auto-generates function prototypes and HOISTS them above the
// first function, so a prototype taking 'KF&' (kfInit/kfPredict/kfUpdate) would otherwise reference KF before
// it is declared -> "'KF' was not declared in this scope". (The SIL builds .ino as plain C++ with no hoisting,
// so it did not catch this -- verify in the Arduino IDE too.)
struct KF{ float s[3]; float P[9]; };   // vertical Kalman: state s=[pos,vel,accel-bias], covariance P (3x3)

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
constexpr int   SERVO_X_SIGN = +1;            // Sign is CORRECT via a MECHANICAL LINKAGE FLIP done 2026-07-12 (computer was
constexpr int   SERVO_Y_SIGN = +1;            // down, couldn't reflash). Flight ASC031 flew THIS code sign (+1) + the flipped
                                              // linkage and held attitude (tilt bounded ~11 deg). *** DO NOT flip these in code ***
                                              // -- the linkage already inverts the sense; a code flip DOUBLE-inverts -> positive
                                              // feedback -> divergence (that was ASC007: +1 + the ORIGINAL linkage -> 43 deg tilt).
                                              // If the linkage is ever rebuilt to its original sense, THEN set these to -1.
                                              // AFTER ANY RE-FLASH: re-run the bench pitch-the-nose test -- the correct code sign
                                              // depends on BOTH the linkage AND the IMU driver's gyro convention, and a reflash can
                                              // change the driver. Never fly a reflash without re-confirming the gimbal corrects.
// PD POLE-PLACED on the measured plant (theta_ddot = keff*delta; keff = T*L/IYY = 14.34*0.14/0.0078 ~= 257 rad/s^2
// per rad TVC, L = CG->TVC-axis = 0.14 m MEASURED). Target zeta=1.0 (critically damped, no overshoot), wn=8 rad/s:
//   Kp = wn^2/keff = 64/257 = 0.249 ;  Kd = 2*zeta*wn/keff = 16/257 = 0.062
// SIL-validated: 0.6 s settle, 0% overshoot, peak servo ~15 deg (X, 8:1), robust across the F-15 burn (keff 215-520
// -> zeta 0.9-1.4) and a slow 150 deg/s servo. The (P*theta + D*rate) sum IS the TVC angle in DEG (capped at MAX_TILT);
// each axis' SERVO_*_MULT converts it to that axis' servo deg, so common P/D gives matched dynamics on both axes.
// Previously-flown fallback (zeta~3.3, overdamped/sluggish): 0.08 / 0.08.  BENCH-TEST before flight.
float P_GAIN = 0.249f, D_GAIN = 0.062f;        // NOT constexpr: the ascent SIL overrides these via extern (getenv
                                               // PGAIN/DGAIN) to adjudicate gains against the roll-coupling failure.
                                               // Flight defaults are unchanged; on hardware they behave as before.
float I_GAIN = 0.20f;                          // integral trim (anti-windup): winds out a CONSTANT thrust-misalignment
                                               // (the ASC007 mechanism) so the vehicle flies VERTICAL instead of holding a
                                               // lean near the 45 deg abort. SIL-tuned/overridable (getenv IGAIN). Does NOT
                                               // add authority -- cancelling a 4 deg misalignment still costs 4 deg of gimbal,
                                               // so ALIGN MECHANICALLY too. Set I_GAIN=0 to disable (pure PD).
float iTermX = 0, iTermY = 0;                  // integral accumulators (TVC deg), reset at launch, clamped +-MAX_TILT
constexpr float SERVO_X_MULT = 5.0f, SERVO_Y_MULT = 5.0f;   // servo deg per TVC deg = 5:1 both axes (5 deg TVC = 25 deg servo).
                                                            // Must match your real gimbal linkage; sets servo THROW, not the loop
                                                            // gain (that's P_GAIN on the TVC angle). Bench-check equal throw both axes.
constexpr float MAX_TILT = 5.0f;              // deg, TVC command limit BEFORE the linkage multiply

// -- Motor / flight constants --------------------------------------------------
constexpr float AV_THRUST     = 14.34f;       // N, average ascent thrust (F-15) -- reference / apogee-timing check
constexpr float BURN_TIME     = 3.45f;        // s, ascent motor burn (F-15) -- burnout at IGN_DELAY+BURN_TIME
constexpr float IGN_DELAY     = 0.2f;         // s, ignition lag
constexpr float ROCKET_WEIGHT = 0.818f;       // kg all-up mass at liftoff   [MEASURED 2026-07-06]
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

// -- Utils ---------------------------------------------------------------------
void LED(bool r,bool g,bool b){ digitalWrite(RLED,!r); digitalWrite(GLED,!g); digitalWrite(BLED,!b); }
void beep(int f,int d){ tone(BUZZER,f); delay(d); noTone(BUZZER); }
void writeServos(float sx,float sy){                    // clamp to a valid servo range for safety
  servoX.write((int)constrain(SERVO_X_SIGN*sx + 90 + XTUNE, 0, 180));
  servoY.write((int)constrain(SERVO_Y_SIGN*sy + 90 + YTUNE, 0, 180));
}
void neutralServos(){ servoX.write((int)(90+XTUNE)); servoY.write((int)(90+YTUNE)); }

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
  logFile=SD.open(logFilename,FILE_WRITE); if(!logFile) return;
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
  while(true){ updatePyros(); beep(500,50); delay(50);
    if(digitalRead(BUTTON)==HIGH) while(true) updatePyros(); }
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
  tiltX = constrain(P_GAIN*gyro_x + iTermX + D_GAIN*ang_vel_x, -MAX_TILT, MAX_TILT) * SERVO_X_MULT;
  tiltY = constrain(P_GAIN*gyro_y + iTermY + D_GAIN*ang_vel_y, -MAX_TILT, MAX_TILT) * SERVO_Y_MULT;
  writeServos(tiltX, tiltY);
}

// -- Setup ---------------------------------------------------------------------
void setup(){
  pinMode(BUTTON,INPUT); pinMode(BUZZER,OUTPUT);
  pinMode(RLED,OUTPUT); pinMode(GLED,OUTPUT); pinMode(BLED,OUTPUT);
  pinMode(P1,OUTPUT); pinMode(P2,OUTPUT); pinMode(P3,OUTPUT); pinMode(P4,OUTPUT);
  digitalWrite(P1,LOW); digitalWrite(P2,LOW); digitalWrite(P3,LOW); digitalWrite(P4,LOW);
  servoX.attach(SERVO_X_PIN); servoY.attach(SERVO_Y_PIN);
  neutralServos();
  LED(false,false,false);
  Serial.begin(115200);
  while(!Serial && millis()<3000);   // wait up to 3 s for the USB monitor so the setup prints (MPU range) are
                                     // visible on the bench; times out -> on the pad with NO monitor it proceeds
                                     // after 3 s (never hangs). Pure startup timing; no control/safety effect.
  Wire.begin();
  if(!bmp.begin(0x76)){ Serial.println(F("BMP280 not found")); while(true); }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_1);
  if(!mpu.begin()){ Serial.println(F("MPU6050 not found")); while(true); }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);   // ASC007 railed at +/-2 g; +/-16 g clears the ~3.2 g axial peak
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);        // headroom for roll/tumble; control rates stay well under 2000 dps
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);      // DLPF: reject motor vibration without lagging the loop
  Serial.print(F("MPU6050 range (want 3/3 = 16g/2000dps): acc=")); Serial.print((int)mpu.getAccelerometerRange());
  Serial.print(F(" gyro=")); Serial.println((int)mpu.getGyroRange());
  SD.begin(SD_CS);
}

// -- Main loop -----------------------------------------------------------------
void loop(){
  // ARM
  LED(true,true,true); delay(500); LED(false,false,false);
  while(!buttonCount()) delay(50);
  delay(500);
  if(!countdown()){                       // pad abort -> wait for button, restart arm
    LED(true,false,false);
    while(digitalRead(BUTTON)==LOW) delay(50);
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
    if(millis()-launchTime < (unsigned long)((BURN_TIME+IGN_DELAY)*1000.0f)){
      TVC();
      captureMotor();              // in-flight motor characterization: sample axial thrust/mass + detect onset
    } else {
      poweredFlight=false;                 // burnout: no thrust -> no control authority; emergency deploy off
      neutralServos();
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
  dumpMotorLog();                          // write the flown motor's thrust curve + ignition lag -> MTR###.CSV

  // RECOVERED
  LED(true,true,true);
  Serial.println(F("RECOVERED"));
  while(digitalRead(BUTTON)==LOW){ beep(523,600); beep(392,600); updatePyros(); }
  LED(false,false,false);
  while(true) updatePyros();
}
