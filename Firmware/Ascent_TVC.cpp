/*
  Ascent_TVC.cpp — dedicated pure-TVC ASCENT-TEST firmware (Teensy 4.1)
  =====================================================================
  Mission: vertical thrust-vectored boost on ONE motor, then PARACHUTE recovery
  at apogee. NO landing burn, NO margin fins. LANDING LEGS deploy at apogee so the
  touchdown loads the legs, not the TVC gimbal/nozzle.

  Derived from Ascent_Test.cpp (the previously-flown ascent controller): same simple
  per-axis PD on gyro angle + rate, +/-5 deg TVC, accel tilt reference on the pad and
  gyro integration in flight -- NOT the ADRC/quaternion stack in Sysiphus_Landing.cpp.

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
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

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
constexpr int   SERVO_X_SIGN = -1;            // <<< BENCH-SET: sign so the gimbal corrects the tilt (was -tiltX)
constexpr int   SERVO_Y_SIGN = -1;            // <<< BENCH-SET
// PD POLE-PLACED on the measured plant (theta_ddot = keff*delta; keff = T*L/IYY = 14.34*0.14/0.0078 ~= 257 rad/s^2
// per rad TVC, L = CG->TVC-axis = 0.14 m MEASURED). Target zeta=1.0 (critically damped, no overshoot), wn=8 rad/s:
//   Kp = wn^2/keff = 64/257 = 0.249 ;  Kd = 2*zeta*wn/keff = 16/257 = 0.062
// SIL-validated: 0.6 s settle, 0% overshoot, peak servo ~15 deg (X, 8:1), robust across the F-15 burn (keff 215-520
// -> zeta 0.9-1.4) and a slow 150 deg/s servo. The (P*theta + D*rate) sum IS the TVC angle in DEG (capped at MAX_TILT);
// each axis' SERVO_*_MULT converts it to that axis' servo deg, so common P/D gives matched dynamics on both axes.
// Previously-flown fallback (zeta~3.3, overdamped/sluggish): 0.08 / 0.08.  BENCH-TEST before flight.
constexpr float P_GAIN = 0.249f, D_GAIN = 0.062f;
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

// -- Sensor smoothing ----------------------------------------------------------
constexpr float ANGVEL_ALPHA = 0.9f, VEL_ALPHA = 0.2f;

// -- Hardware ------------------------------------------------------------------
PWMServo servoX, servoY;
MPU6050  mpu6050(Wire);
Adafruit_BMP280 bmp;
File logFile; char logFilename[20];

// -- Flight state --------------------------------------------------------------
float altitude, initial_alt, highest_alt, vert_vel;
float gyro_x, gyro_y, gyro_z, ang_vel_x, ang_vel_y;
float accel_x, accel_y, accel_z, tiltX, tiltY;
bool  poweredFlight = false;   // true only during motor burn -> gates emergency deploy
bool  inFlight      = false;   // true launch..apogee -> gates the angle source
unsigned long launchTime = 0;

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
    logFile.println(F("Time(ms),Altitude(m),VertVel(m/s),GyroX,GyroY,GyroZ,AngVelX,AngVelY,AccelX,AccelY,AccelZ,ServoX,ServoY"));
    logFile.close();
  } else Serial.println(F("Failed to create log file!"));
}
void logData(){
  logFile=SD.open(logFilename,FILE_WRITE); if(!logFile) return;
  char buf[128];
  snprintf(buf,sizeof(buf),"%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
    millis(),altitude,vert_vel,gyro_x,gyro_y,gyro_z,ang_vel_x,ang_vel_y,accel_x,accel_y,accel_z,
    constrain(tiltX,-MAX_TILT,MAX_TILT),constrain(tiltY,-MAX_TILT,MAX_TILT));
  logFile.println(buf); logFile.close();
}

// -- Sensors (one attitude source at a time; identical policy to the flown code)
//    on the pad: accel atan2 (gravity is a reliable reference)
//    in flight : gyro integration (accel reads thrust+gravity, unusable)
void sensors(){
  static unsigned long prevTime=millis(), angleTime=millis(); static float prev_alt=0;
  mpu6050.update();
  float raw_avx =  mpu6050.getGyroX();
  float raw_avy = -mpu6050.getGyroY();
  ang_vel_x = ANGVEL_ALPHA*raw_avx + (1-ANGVEL_ALPHA)*ang_vel_x;
  ang_vel_y = ANGVEL_ALPHA*raw_avy + (1-ANGVEL_ALPHA)*ang_vel_y;

  unsigned long angleNow=millis(); float dt=(angleNow-angleTime)/1000.0f; angleTime=angleNow;
  if(inFlight && dt>0 && dt<0.1f){ gyro_x += ang_vel_x*dt; gyro_y += ang_vel_y*dt; }
  else if(!inFlight){
    gyro_x =  atan2f(mpu6050.getAccY(), mpu6050.getAccZ())*180.0f/M_PI;
    gyro_y = -atan2f(mpu6050.getAccX(), mpu6050.getAccZ())*180.0f/M_PI;
  }
  gyro_z = mpu6050.getAngleZ();
  accel_x=mpu6050.getAccX()*G; accel_y=mpu6050.getAccY()*G; accel_z=mpu6050.getAccZ()*G;

  altitude = bmp.readAltitude(SEA_LEVEL_HPA) - initial_alt;
  if(altitude>highest_alt) highest_alt=altitude;

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
      mpu6050.calcGyroOffsets(true,0,0);
      initial_alt=bmp.readAltitude(SEA_LEVEL_HPA);
      Serial.print(F("  baseline alt: ")); Serial.println(initial_alt);
      noTone(BUZZER); LED(false,false,false); }
  }
  // calcGyroOffsets fixes rate bias but not accumulated angle -> spin update() 1 s then zero the state
  unsigned long flush=millis(); while(millis()-flush<1000) mpu6050.update();
  gyro_x=gyro_y=gyro_z=ang_vel_x=ang_vel_y=0;

  // pre-launch sanity: bad tilt (>45) or drift (>10 dps) -> abort, no ignition
  for(int i=0;i<10;i++){ mpu6050.update(); delay(20); }
  float cx=mpu6050.getAngleX(), cy=mpu6050.getAngleY(), cavx=mpu6050.getGyroX(), cavy=mpu6050.getGyroY();
  if(fabsf(cx)>45||fabsf(cy)>45||fabsf(cavx)>10||fabsf(cavy)>10){
    Serial.print(F("ABORT -- tilt X=")); Serial.print(cx); Serial.print(F(" Y=")); Serial.print(cy);
    Serial.print(F("  rate avX=")); Serial.print(cavx); Serial.print(F(" avY=")); Serial.println(cavy);
    LED(true,false,false); for(int i=0;i<10;i++){ beep(880,100); delay(100); } LED(false,false,false);
    return false;
  }
  Serial.println(F("LAUNCH"));
  triggerPyro(P3);
  return true;
}

// -- TVC: per-axis PD to vertical (setpoint 0) --------------------------------
void TVC(){
  sensors();
  tiltX = constrain(P_GAIN*gyro_x + D_GAIN*ang_vel_x, -MAX_TILT, MAX_TILT) * SERVO_X_MULT;
  tiltY = constrain(P_GAIN*gyro_y + D_GAIN*ang_vel_y, -MAX_TILT, MAX_TILT) * SERVO_Y_MULT;
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
  Serial.begin(115200); Wire.begin();
  if(!bmp.begin(0x76)){ Serial.println(F("BMP280 not found")); while(true); }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_1);
  mpu6050.begin();
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

  // RECOVERED
  LED(true,true,true);
  Serial.println(F("RECOVERED"));
  while(digitalRead(BUTTON)==LOW){ beep(523,600); beep(392,600); updatePyros(); }
  LED(false,false,false);
  while(true) updatePyros();
}
