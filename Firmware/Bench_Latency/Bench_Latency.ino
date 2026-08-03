/*
  Bench_Latency.ino — actuator transport-delay characterization for the TVC gimbal
  ==================================================================================
  PURPOSE: measure the ACTUATOR half of the control-loop delay tau, which is the single
  unmeasured quantity that every prediction in the gain-window study is indexed on.

  The simulator assumes SERVO_TAU = 0.030 s (ascent_sim.cpp:61). That number was never
  measured on this hardware -- it was a plausible hobby-servo default. Since the risk
  parameter goes as tau^2, a 2x error in tau is a 4x error in the prediction, so a flight
  flown before this measurement produces a data point that cannot be placed on any axis.

  WHAT THIS MEASURES (per axis, per step size, per direction):
    t_dead   [ms]      command issued -> feedback first moves (transport delay)
    slew     [deg/s]   peak d(position)/dt during a large, rate-limited step
    tau_lag  [ms]      first-order lag, from a SMALL step that never hits the rate limit
    settle   [ms]      time to stay inside a 2% band
    tau_act  [ms]      = t_dead + tau_lag  <- the number to put in Pi = keff * tau^2

  WHY THE PHASE RANDOMIZATION MATTERS (the part a naive step test gets wrong):
    PWMServo updates the servo on a ~50 Hz frame. A command issued just after a frame
    boundary is not physically applied until the next one -- so the true transport delay
    is not a constant, it is roughly UNIFORM over one frame period (~0-20 ms) plus the
    servo's own electronics delay. Stepping at a fixed instant every trial samples ONE
    phase and reports a confident, wrong number. This sketch inserts a random 0-25 ms
    delay before each step so the reported dead time is a DISTRIBUTION. Report the mean
    for tau, and the max when you want the conservative case.

  *** CALIBRATION WARNING -- RESOLVE BEFORE TRUSTING ANY OUTPUT ***
    This file and the other two bench sketches assume LINKAGE_SERVO_DEG_PER_GIMBAL_DEG
    = 4.0, but Ascent_TVC.ino flies SERVO_X_MULT = SERVO_Y_MULT = 5.0. They disagree.
    Everything reported in GIMBAL degrees scales by this ratio, so a 25% linkage error
    becomes a ~56% error in Pi. Measure the real ratio (command a known servo deflection,
    measure the nozzle angle with a protractor or photo) and set it here AND in the
    flight firmware before using any of these numbers.

  HARDWARE: same wiring as feedback_servo_calibration.ino / gimbal_characterization.ino.
    Servo X cmd = pin 4, Y cmd = pin 3; feedback X = A2, Y = A3.
    NO MOTOR. Bench only. The gimbal must be free to move through its full travel.

  USAGE (Serial @ 115200): boot -> holds neutral for the clearance check -> type 'g' to
    run. Emits a per-trial CSV block, then a SUMMARY block. Type 'r' to re-run.
*/
#include <Arduino.h>
#include <math.h>

#if defined(CORE_TEENSY)
#include <PWMServo.h>
using ServoDriver = PWMServo;
#else
#include <Servo.h>
using ServoDriver = Servo;
#endif

// ---- Pins (identical to feedback_servo_calibration.ino) ----------------------
#if defined(CORE_TEENSY)
constexpr int SERVO_X_PIN = 4, SERVO_Y_PIN = 3;
constexpr int FB_X_PIN = A2,   FB_Y_PIN = A3;
#else
constexpr int SERVO_X_PIN = 9, SERVO_Y_PIN = 10;
constexpr int FB_X_PIN = A0,   FB_Y_PIN = A1;
#endif

// ---- Linkage / travel (SEE CALIBRATION WARNING ABOVE) -----------------------
constexpr int   SERVO_NEUTRAL_DEG = 90;
constexpr float LINKAGE_SERVO_DEG_PER_GIMBAL_DEG = 4.0f;   // <<< flight firmware says 5.0
constexpr float GIMBAL_HALF_SPAN_DEG = 5.0f;               // matches MAX_TILT in flight code
constexpr int   SERVO_HALF_SPAN_DEG =
    (int)(LINKAGE_SERVO_DEG_PER_GIMBAL_DEG * GIMBAL_HALF_SPAN_DEG + 0.5f);

// ---- Vehicle constants, for the on-bench Pi readout -------------------------
// keff = T_avg * L / Iyy, all MEASURED (Ascent_TVC.ino header, 2026-07-06).
constexpr float T_AVG_N   = 14.34f;    // N,      F-15 average thrust
constexpr float L_CG_M    = 0.14f;     // m,      CG -> TVC pivot
constexpr float IYY_KGM2  = 0.0078f;   // kg*m^2, wet, one motor
constexpr float KEFF      = T_AVG_N * L_CG_M / IYY_KGM2;   // ~257 s^-2

// ---- Capture ----------------------------------------------------------------
constexpr int      CAP_N          = 900;   // samples per trial
constexpr uint32_t CAP_PERIOD_US  = 500;   // 2 kHz -> 450 ms window
constexpr int      PRE_N          = 100;   // pre-step samples -> noise floor
constexpr int      N_REPS         = 12;    // reps per (axis, size, direction)
constexpr uint32_t PHASE_MAX_US   = 25000; // randomized step phase vs the ~50 Hz frame
constexpr float    DETECT_SIGMA   = 5.0f;  // dead-time trigger, in baseline sigmas
constexpr int      DETECT_RUN     = 3;     // consecutive samples required (rejects spikes)
constexpr float    SETTLE_BAND    = 0.02f; // 2% of step travel

// Small step must NOT hit the rate limit (isolates lag); large step must (gives slew).
constexpr int SMALL_STEP_SERVO_DEG = 2;
constexpr int LARGE_STEP_SERVO_DEG = SERVO_HALF_SPAN_DEG;

ServoDriver servoX, servoY;
uint32_t capT[CAP_N];
int16_t  capV[CAP_N];

struct Stat { float n=0, mean=0, m2=0, mn=1e9f, mx=-1e9f;
  void add(float x){ n++; float d=x-mean; mean+=d/n; m2+=d*(x-mean); if(x<mn)mn=x; if(x>mx)mx=x; }
  float sd() const { return n>1? sqrtf(m2/(n-1)) : 0.0f; } };

struct Axis { const char* name; ServoDriver* servo; int fbPin;
              Stat deadSmall, deadLarge, lag, slewCounts, settle; float countsPerServoDeg=0; };
Axis axX{"X", &servoX, FB_X_PIN};
Axis axY{"Y", &servoY, FB_Y_PIN};

// ---- helpers ----------------------------------------------------------------
static void writeServo(ServoDriver& s, int deg){
  s.write(constrain(deg, SERVO_NEUTRAL_DEG - SERVO_HALF_SPAN_DEG,
                         SERVO_NEUTRAL_DEG + SERVO_HALF_SPAN_DEG));
}

// Capture a step: settle at 'from', randomize phase, command 'to', log at CAP_PERIOD_US.
// Returns the index at which the command was issued (== PRE_N).
static int captureStep(Axis& a, int from, int to){
  writeServo(*a.servo, from);
  delay(400);                                   // settle + let any lag decay
  uint32_t next = micros();
  for(int i=0;i<PRE_N;i++){                     // pre-step baseline
    while((int32_t)(micros()-next) < 0) {}
    capT[i]=micros(); capV[i]=analogRead(a.fbPin); next += CAP_PERIOD_US;
  }
  // Randomize where the command lands inside the ~50 Hz PWM frame. Without this the
  // measured dead time is one arbitrary phase of a variable delay, reported as a constant.
  delayMicroseconds(random(0, (long)PHASE_MAX_US));
  uint32_t tCmd = micros();
  writeServo(*a.servo, to);
  next = micros();
  for(int i=PRE_N;i<CAP_N;i++){
    while((int32_t)(micros()-next) < 0) {}
    capT[i]=micros(); capV[i]=analogRead(a.fbPin); next += CAP_PERIOD_US;
  }
  capT[PRE_N-1] = tCmd;                         // stash command instant (see analyse())
  return PRE_N;
}

// Analyse one captured step. Writes results into the Stat accumulators.
// isLarge selects which dead-time bucket and whether slew/lag are meaningful.
static void analyse(Axis& a, bool isLarge, int stepServoDeg, bool verbose){
  // baseline mean/sigma from the pre-step window (skip the last, it holds tCmd)
  double s=0, s2=0; int n=PRE_N-1;
  for(int i=0;i<n;i++){ s+=capV[i]; s2+=(double)capV[i]*capV[i]; }
  float base = s/n;
  float sig  = sqrtf(fmaxf((float)(s2/n - (double)base*base), 1e-6f));
  uint32_t tCmd = capT[PRE_N-1];

  float finalV = 0; { double f=0; int k=0;
    for(int i=CAP_N-60;i<CAP_N;i++){ f+=capV[i]; k++; } finalV=f/k; }
  float travel = finalV - base;
  if (fabsf(travel) < DETECT_SIGMA*sig){        // servo did not move -> bad wiring/binding
    if(verbose) Serial.println(F("# WARN: no feedback motion detected -- check wiring/binding"));
    return;
  }
  a.countsPerServoDeg = fabsf(travel) / (float)stepServoDeg;

  // --- dead time: first sustained excursion past DETECT_SIGMA ---
  int iMove=-1, run=0;
  for(int i=PRE_N;i<CAP_N;i++){
    if (fabsf(capV[i]-base) > DETECT_SIGMA*sig){ if(++run>=DETECT_RUN){ iMove=i-DETECT_RUN+1; break; } }
    else run=0;
  }
  if(iMove<0) return;
  float tDead = (capT[iMove]-tCmd)/1000.0f;     // ms
  if(isLarge) a.deadLarge.add(tDead); else a.deadSmall.add(tDead);

  // --- slew: peak slope over a 5-sample window, only meaningful for the large step ---
  if(isLarge){
    float best=0;
    for(int i=iMove;i<CAP_N-5;i++){
      float d = fabsf((float)capV[i+5]-capV[i]);
      float dt = (capT[i+5]-capT[i])/1e6f;
      if(dt>0 && d/dt > best) best = d/dt;      // counts per second
    }
    a.slewCounts.add(best);
  }

  // --- first-order lag: only from the SMALL step, which never rate-limits ---
  if(!isLarge){
    float target = base + 0.632f*travel;
    for(int i=iMove;i<CAP_N;i++){
      if((travel>0 && capV[i]>=target)||(travel<0 && capV[i]<=target)){
        a.lag.add((capT[i]-capT[iMove])/1000.0f); break; }
    }
  }

  // --- settling: last exit from the 2% band ---
  float band = fabsf(travel)*SETTLE_BAND;
  int iSet=CAP_N-1;
  for(int i=CAP_N-1;i>=iMove;i--){ if(fabsf(capV[i]-finalV)>band){ iSet=i; break; } }
  a.settle.add((capT[iSet]-tCmd)/1000.0f);

  if(verbose){
    Serial.print(a.name); Serial.print(',');
    Serial.print(isLarge?"large":"small"); Serial.print(',');
    Serial.print(stepServoDeg); Serial.print(',');
    Serial.print(tDead,2); Serial.print(',');
    Serial.print(sig,2); Serial.print(',');
    Serial.println(travel,1);
  }
}

static void runAxis(Axis& a){
  const int lo = SERVO_NEUTRAL_DEG - LARGE_STEP_SERVO_DEG;
  const int hi = SERVO_NEUTRAL_DEG + LARGE_STEP_SERVO_DEG;
  for(int r=0;r<N_REPS;r++){
    // large, both directions (asymmetry is real on geared servos under linkage preload)
    captureStep(a, lo, hi); analyse(a, true,  2*LARGE_STEP_SERVO_DEG, true);
    captureStep(a, hi, lo); analyse(a, true,  2*LARGE_STEP_SERVO_DEG, true);
    // small, both directions
    captureStep(a, SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG+SMALL_STEP_SERVO_DEG);
    analyse(a, false, SMALL_STEP_SERVO_DEG, true);
    captureStep(a, SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG-SMALL_STEP_SERVO_DEG);
    analyse(a, false, SMALL_STEP_SERVO_DEG, true);
  }
  writeServo(*a.servo, SERVO_NEUTRAL_DEG);
}

static void report(Axis& a){
  float cps = a.countsPerServoDeg;                         // counts per servo deg
  float slewServo = cps>0 ? a.slewCounts.mean/cps : NAN;   // servo deg/s
  float slewGimb  = slewServo / LINKAGE_SERVO_DEG_PER_GIMBAL_DEG;
  float tauAct    = a.deadSmall.mean + a.lag.mean;         // ms
  Serial.print(F("AXIS ")); Serial.println(a.name);
  Serial.print(F("  t_dead small  [ms]  mean=")); Serial.print(a.deadSmall.mean,2);
  Serial.print(F("  sd=")); Serial.print(a.deadSmall.sd(),2);
  Serial.print(F("  min=")); Serial.print(a.deadSmall.mn,2);
  Serial.print(F("  max=")); Serial.println(a.deadSmall.mx,2);
  Serial.print(F("  t_dead large  [ms]  mean=")); Serial.print(a.deadLarge.mean,2);
  Serial.print(F("  sd=")); Serial.println(a.deadLarge.sd(),2);
  Serial.print(F("  tau_lag       [ms]  mean=")); Serial.print(a.lag.mean,2);
  Serial.print(F("  sd=")); Serial.println(a.lag.sd(),2);
  Serial.print(F("  settle 2%     [ms]  mean=")); Serial.println(a.settle.mean,2);
  Serial.print(F("  slew  [servo deg/s] ")); Serial.print(slewServo,1);
  Serial.print(F("   [gimbal deg/s] "));     Serial.println(slewGimb,1);
  Serial.print(F("  tau_act = dead+lag [ms] = ")); Serial.println(tauAct,2);
  Serial.print(F("  -> Pi_actuator_only = keff*tau^2 = "));
  Serial.println(KEFF * (tauAct/1000.0f) * (tauAct/1000.0f), 3);
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<4000);
#if defined(CORE_TEENSY)
  analogReadResolution(12);
  analogReadAveraging(1);          // speed over smoothing: we want the true noise floor
#endif
  randomSeed(micros());
  servoX.attach(SERVO_X_PIN); servoY.attach(SERVO_Y_PIN);
  writeServo(servoX, SERVO_NEUTRAL_DEG); writeServo(servoY, SERVO_NEUTRAL_DEG);
  Serial.println(F("# Bench_Latency -- actuator transport delay"));
  Serial.println(F("# NO MOTOR. Confirm the gimbal is free through full travel."));
  Serial.print  (F("# keff = T*L/Iyy = ")); Serial.print(KEFF,1); Serial.println(F(" s^-2"));
  Serial.print  (F("# linkage servo-deg per gimbal-deg = "));
  Serial.print(LINKAGE_SERVO_DEG_PER_GIMBAL_DEG,1);
  Serial.println(F("   <-- flight firmware uses 5.0. RESOLVE THIS."));
  Serial.println(F("# holding neutral. send 'g' to run, 'r' to re-run."));
}

void loop(){
  if(!Serial.available()) return;
  char c = Serial.read();
  if(c!='g' && c!='r') return;
  axX = Axis{"X",&servoX,FB_X_PIN}; axY = Axis{"Y",&servoY,FB_Y_PIN};
  Serial.println(F("axis,size,step_servo_deg,t_dead_ms,fb_sigma_counts,travel_counts"));
  runAxis(axX); runAxis(axY);
  Serial.println(F("==================== SUMMARY ===================="));
  report(axX); report(axY);
  Serial.println(F("NOTE: Pi above uses the ACTUATOR delay only. Total loop tau also includes"));
  Serial.println(F("  the sensor/compute path (MPU6050 DLPF group delay + loop period). Add that"));
  Serial.println(F("  from the flight firmware's loop-period log before computing the vehicle Pi."));
  Serial.println(F("================================================"));
}
