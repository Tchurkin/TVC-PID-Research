/*
  Bench_Tau_BNO055.ino — measure the SENSOR+COMPUTE loop delay tau on a BNO055 vehicle
  =====================================================================================
  UPLOAD IT, FOLLOW THE ONE PROMPT, PASTE THE SERIAL OUTPUT. No servos, no motor, no
  extra hardware, no wiring changes. Bench only.

  WHY THIS EXISTS, AND WHY IT IS NOT Bench_Latency
  ------------------------------------------------
  Bench_Latency measures the ACTUATOR half (servo dead time + slew) and needs pot-tapped
  servos. That is NOT the tau the research is indexed on. In the simulator:

      tools/ceiling_kd_free.py:136   tau = latency_steps / 200.0
      sim/design_space.py:285        sensor_latency_steps = latency_steps

  so the tau in `ceiling ~ 0.0661/tau` is the SENSOR->CONTROLLER transport delay. The
  actuator's own lag is a separate parameter (tau_act = 0.05 s) held FIXED across all
  2,400 designs, so it is not what the ceiling law varies. This sketch measures the
  quantity the paper actually needs.

  THE BNO055 PROBLEM
  ------------------
  An MPU6050 is a raw sensor: its delay is a datasheet DLPF group delay of a few ms. The
  BNO055 is NOT that. It runs Bosch's sensor-fusion firmware on an internal Cortex-M0 and
  outputs a FILTERED attitude estimate at 100 Hz. That fusion introduces a delay which:
    - is the dominant term in tau on this vehicle,
    - is not published in the datasheet, and
    - cannot be looked up. It has to be measured.

  HOW IT IS MEASURED WITH ONE SENSOR AND NO REFERENCE
  ---------------------------------------------------
  The BNO055 exposes BOTH the fused Euler estimate AND its own raw gyro. The raw gyro is
  nearly un-delayed; the fused estimate carries the whole fusion pipeline. Sample both in
  the same loop while the board is waved by hand, differentiate the Euler angles, and
  cross-correlate d(Euler)/dt against the raw gyro. The lag at peak correlation IS the
  fusion delay. Both channels are sampled at the same instants, so everything common to
  them (I2C time, loop jitter) cancels and what is left is the fusion pipeline.

  Sub-sample resolution comes from parabolic interpolation on the correlation peak, so the
  answer is not quantised to the 2.5 ms sample period.

  WHAT IT REPORTS
  ---------------
    t_i2c        [us] cost of each read type, so the loop budget is known
    T_out        [ms] true fused-output update period (nominally 10 ms = 100 Hz)
    t_fusion     [ms] THE measurement -- fused estimate lag behind raw gyro
    T_loop       [ms] flight-representative loop period, min/median/max
    tau_total    [ms] mean and worst case, with the arithmetic shown

  PRECISION TARGET: +-10% on tau_total. Do not chase +-2%; tau enters the ceiling law
  linearly and that model's own residual spread is [0.35x, 2.03x].
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ---- capture geometry -------------------------------------------------------
constexpr uint32_t SAMPLE_US   = 2500;   // 400 Hz. Must be well above the 100 Hz fused rate.
constexpr int      NCAP        = 4000;   // 10.0 s of waving
constexpr int      MAXLAG      = 40;     // 40 * 2.5 ms = 100 ms of lag searched
constexpr int      NI2C_REPS   = 200;    // reps for the I2C cost measurement
constexpr uint32_t OUTPER_MS   = 3000;   // how long to watch for fused-output changes
constexpr int      NLOOP_REPS  = 2000;   // reps for the loop-period measurement

// Teensy: put the capture in RAM2 so it never competes with the stack.
#if defined(__IMXRT1062__)
#define BIGBUF DMAMEM
#else
#define BIGBUF
#endif
BIGBUF uint32_t capT[NCAP];
BIGBUF float    capG[NCAP][3];      // raw gyro, deg/s
BIGBUF float    capE[NCAP][3];      // fused Euler, deg  (heading, roll, pitch)

// Constructed AFTER detection, so the right bus and address are known first.
Adafruit_BNO055* bno = nullptr;
TwoWire* i2cBus = nullptr;
uint8_t  bnoAddr = 0;
const char* busName = "?";
uint32_t i2cHz = 100000;

// ---- helpers ----------------------------------------------------------------
static float medianOf(uint32_t* v, int n){
  // insertion sort is fine at n<=2000 on a 600 MHz core and avoids qsort's comparator
  for(int i=1;i<n;i++){ uint32_t k=v[i]; int j=i-1; while(j>=0&&v[j]>k){v[j+1]=v[j];j--;} v[j+1]=k; }
  return (n&1)? (float)v[n/2] : 0.5f*((float)v[n/2-1]+(float)v[n/2]);
}

// Unwrap an angle sequence in place (heading wraps 0..360; roll/pitch can too under waving).
static void unwrap(float* a, int n){
  float off = 0, prev = a[0];
  for(int i=1;i<n;i++){
    float d = a[i] - prev; prev = a[i];
    if(d >  180.0f) off -= 360.0f;
    if(d < -180.0f) off += 360.0f;
    a[i] += off;
  }
}

// Read the BNO055 CHIP_ID (reg 0x00). A real BNO055 answers 0xA0. This distinguishes it
// from anything else that happens to ACK on 0x28/0x29.
static bool bnoChipIdOk(TwoWire& w, uint8_t addr){
  w.beginTransmission(addr); w.write((uint8_t)0x00);
  if(w.endTransmission() != 0) return false;
  if(w.requestFrom((int)addr, 1) != 1) return false;
  return w.read() == 0xA0;
}

static void scanBus(TwoWire& w, const char* name, uint32_t hz){
  w.begin(); w.setClock(hz);
  Serial.print(F("  ")); Serial.print(name); Serial.print(F(" @ "));
  Serial.print(hz/1000); Serial.print(F(" kHz: "));
  int found=0;
  for(uint8_t a=0x08; a<0x78; a++){
    w.beginTransmission(a);
    if(w.endTransmission()==0){
      Serial.print(F("0x")); if(a<16) Serial.print('0'); Serial.print(a,HEX); Serial.print(' ');
      found++;
    }
  }
  if(!found) Serial.print(F("(nothing)"));
  Serial.println();
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis() < 4000);

  Serial.println(F("# ============================================================"));
  Serial.println(F("# Bench_Tau_BNO055 -- sensor+compute loop delay"));
  Serial.println(F("# BENCH ONLY. No motor. Servos may be disconnected."));
  Serial.println(F("# ============================================================"));

  // The BNO055 needs ~650 ms from power-on before it answers. A Teensy soft-reset does NOT
  // power-cycle the sensor, but a fresh upload can, so wait unconditionally.
  Serial.println(F("# waiting 1000 ms for BNO055 boot..."));
  delay(1000);

  // Try every bus, and 100 kHz FIRST. The BNO055 stretches the clock and is widely reported
  // to enumerate unreliably at 400 kHz -- the flight firmware runs Wire at the 100 kHz default.
  TwoWire* busses[] = {
    &Wire,
#if defined(__IMXRT1062__)
    &Wire1, &Wire2,
#endif
  };
  const char* names[] = {
    "Wire  (SDA18/SCL19)",
#if defined(__IMXRT1062__)
    "Wire1 (SDA17/SCL16)", "Wire2 (SDA25/SCL24)",
#endif
  };
  const int NBUS = sizeof(busses)/sizeof(busses[0]);
  const uint32_t rates[] = {100000, 400000};

  for(int r=0; r<2 && !i2cBus; r++){
    for(int b=0; b<NBUS && !i2cBus; b++){
      busses[b]->begin();
      busses[b]->setClock(rates[r]);
      for(int k=0; k<2; k++){
        uint8_t a = k ? 0x29 : 0x28;
        if(bnoChipIdOk(*busses[b], a)){
          i2cBus=busses[b]; bnoAddr=a; busName=names[b]; i2cHz=rates[r];
          break;
        }
      }
    }
  }

  if(!i2cBus){
    Serial.println(F("FATAL: no BNO055 (CHIP_ID 0xA0) on any bus at 0x28 or 0x29."));
    Serial.println(F("Full scan of every bus, so you can see what IS connected:"));
    for(int r=0;r<2;r++) for(int b=0;b<NBUS;b++) scanBus(*busses[b], names[b], rates[r]);
    Serial.println(F(""));
    Serial.println(F("Checklist:"));
    Serial.println(F("  1. VIN to 3.3V (or 5V if your breakout has a regulator), GND common."));
    Serial.println(F("  2. SDA/SCL not swapped. Teensy 4.1 Wire = SDA 18, SCL 19."));
    Serial.println(F("  3. Pull-ups present -- most breakouts have them, bare chips do not."));
    Serial.println(F("  4. ADR pin: low = 0x28, high = 0x29."));
    Serial.println(F("  5. If an address ACKs above but CHIP_ID was wrong, it is not a BNO055."));
    Serial.println(F("Paste this whole block and I can tell you which one it is."));
    while(1) delay(1000);
  }

  Serial.print(F("# BNO055 found: ")); Serial.print(busName);
  Serial.print(F("  addr 0x")); Serial.print(bnoAddr,HEX);
  Serial.print(F("  @ ")); Serial.print(i2cHz/1000); Serial.println(F(" kHz"));

  bno = new Adafruit_BNO055(55, bnoAddr, i2cBus);
  if(!bno->begin(OPERATION_MODE_NDOF)){
    Serial.println(F("FATAL: CHIP_ID matched but begin() failed -- sensor present, not responding."));
    while(1) delay(1000);
  }
  delay(1000);
  bno->setExtCrystalUse(true);
  delay(500);

  uint8_t sysCal=0, gyroCal=0, accCal=0, magCal=0;
  bno->getCalibration(&sysCal, &gyroCal, &accCal, &magCal);
  Serial.print(F("# calibration sys/gyro/acc/mag = "));
  Serial.print(sysCal); Serial.print('/'); Serial.print(gyroCal); Serial.print('/');
  Serial.print(accCal); Serial.print('/'); Serial.println(magCal);
  Serial.println(F("#   gyro>=3 is what matters here. If gyro is 0, leave the board still ~5 s."));
  Serial.println();
}

// ---- Phase 1: I2C read cost -------------------------------------------------
static void phaseI2C(){
  Serial.println(F("-- PHASE 1: I2C read cost ---------------------------------"));
  static uint32_t d1[NI2C_REPS], d2[NI2C_REPS], d3[NI2C_REPS];
  for(int i=0;i<NI2C_REPS;i++){
    uint32_t a=micros(); volatile imu::Vector<3> g = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    uint32_t b=micros(); volatile imu::Vector<3> e = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
    uint32_t c=micros(); volatile imu::Quaternion q = bno->getQuat();
    uint32_t d=micros();
    (void)g;(void)e;(void)q;
    d1[i]=b-a; d2[i]=c-b; d3[i]=d-c;
  }
  Serial.print(F("  getVector(GYRO)  median us = ")); Serial.println(medianOf(d1,NI2C_REPS),1);
  Serial.print(F("  getVector(EULER) median us = ")); Serial.println(medianOf(d2,NI2C_REPS),1);
  Serial.print(F("  getQuat()        median us = ")); Serial.println(medianOf(d3,NI2C_REPS),1);
  Serial.println();
}

// ---- Phase 2: true fused output period --------------------------------------
static float phaseOutputPeriod(){
  Serial.println(F("-- PHASE 2: fused-output update period ---------------------"));
  Serial.println(F("   (hold the board STILL is fine; we watch the LSB change)"));
  imu::Vector<3> prev = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
  uint32_t tPrev = micros(), t0 = millis();
  uint32_t nChg = 0; double sum = 0; uint32_t mn = 0xFFFFFFFF, mx = 0;
  while(millis() - t0 < OUTPER_MS){
    imu::Vector<3> e = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
    if(e.x()!=prev.x() || e.y()!=prev.y() || e.z()!=prev.z()){
      uint32_t now = micros(), dt = now - tPrev;
      if(nChg){ sum += dt; if(dt<mn)mn=dt; if(dt>mx)mx=dt; }
      nChg++; tPrev = now; prev = e;
    }
  }
  float per = nChg>1 ? (float)(sum/(nChg-1))/1000.0f : NAN;
  Serial.print(F("  changes seen        = ")); Serial.println(nChg);
  Serial.print(F("  update period  [ms] = ")); Serial.print(per,3);
  Serial.print(F("   min ")); Serial.print(mn/1000.0f,3);
  Serial.print(F("   max ")); Serial.println(mx/1000.0f,3);
  Serial.print(F("  -> effective rate [Hz] = ")); Serial.println(per>0?1000.0f/per:0.0f,1);
  Serial.println(F("  (BNO055 NDOF fusion is nominally 100 Hz = 10.000 ms)"));
  Serial.println();
  return per;
}

// ---- Phase 3: the wave test -- fusion delay ---------------------------------
static float phaseFusionDelay(float* outCorr, int* outBestPair){
  Serial.println(F("-- PHASE 3: fusion delay (THE measurement) -----------------"));
  Serial.println(F("   >>> WAVE THE BOARD BY HAND, briskly, back and forth, for 10 s. <<<"));
  Serial.println(F("   Rotate it -- do not translate it. A 1-2 Hz wave of +-30 deg is ideal."));
  Serial.println(F("   Any axis; the analysis picks the best-excited pair automatically."));
  for(int i=3;i>0;i--){ Serial.print(F("   starting in ")); Serial.println(i); delay(1000); }
  Serial.println(F("   GO -- wave now."));

  uint32_t next = micros();
  for(int i=0;i<NCAP;i++){
    while((int32_t)(micros()-next) < 0) {}
    capT[i] = micros();
    imu::Vector<3> g = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  // deg/s
    imu::Vector<3> e = bno->getVector(Adafruit_BNO055::VECTOR_EULER);      // deg
    capG[i][0]=g.x(); capG[i][1]=g.y(); capG[i][2]=g.z();
    capE[i][0]=e.x(); capE[i][1]=e.y(); capE[i][2]=e.z();
    next += SAMPLE_US;
  }
  Serial.println(F("   capture done."));

  // Unwrap each Euler channel, then differentiate onto the same grid.
  static float dE[NCAP][3];
  for(int c=0;c<3;c++){
    static float tmp[NCAP];
    for(int i=0;i<NCAP;i++) tmp[i]=capE[i][c];
    unwrap(tmp, NCAP);
    dE[0][c]=0;
    for(int i=1;i<NCAP;i++){
      float dt=(capT[i]-capT[i-1])/1e6f;
      dE[i][c] = dt>0 ? (tmp[i]-tmp[i-1])/dt : 0;
    }
  }
  // Remove means so correlation is on the AC part only.
  static float gm[3], em[3];
  for(int c=0;c<3;c++){
    double sg=0, se=0;
    for(int i=1;i<NCAP;i++){ sg+=capG[i][c]; se+=dE[i][c]; }
    gm[c]=sg/(NCAP-1); em[c]=se/(NCAP-1);
  }

  // Report how much motion each channel actually saw -- a flat channel cannot give a lag.
  Serial.println(F("   excitation (rms, deg/s):"));
  for(int c=0;c<3;c++){
    double sg=0, se=0;
    for(int i=1;i<NCAP;i++){ float a=capG[i][c]-gm[c], b=dE[i][c]-em[c]; sg+=a*a; se+=b*b; }
    Serial.print(F("     axis ")); Serial.print(c);
    Serial.print(F("  gyro ")); Serial.print(sqrt(sg/(NCAP-1)),2);
    Serial.print(F("   dEuler ")); Serial.println(sqrt(se/(NCAP-1)),2);
  }

  // Cross-correlate every (gyro axis, dEuler axis) pair over lag 0..MAXLAG.
  // dEuler LAGS gyro, so we compare gyro[i] against dEuler[i+L] and look for L>0.
  float bestR=-2; int bestL=0, bestPair=0;
  static float corrBest[MAXLAG+1];
  for(int gc=0; gc<3; gc++) for(int ec=0; ec<3; ec++){
    static float corr[MAXLAG+1];
    for(int L=0;L<=MAXLAG;L++){
      double num=0, dg=0, de=0;
      for(int i=1;i<NCAP-MAXLAG;i++){
        float a = capG[i][gc]-gm[gc];
        float b = dE[i+L][ec]-em[ec];
        num += (double)a*b; dg += (double)a*a; de += (double)b*b;
      }
      corr[L] = (dg>0&&de>0) ? (float)(num/sqrt(dg*de)) : 0.0f;
    }
    for(int L=0;L<=MAXLAG;L++){
      if(fabsf(corr[L]) > fabsf(bestR)){
        bestR=corr[L]; bestL=L; bestPair=gc*3+ec;
        for(int k=0;k<=MAXLAG;k++) corrBest[k]=corr[k];
      }
    }
  }

  // Parabolic interpolation about the peak -> sub-sample lag.
  float lagSamples = bestL;
  if(bestL>0 && bestL<MAXLAG){
    float y0=fabsf(corrBest[bestL-1]), y1=fabsf(corrBest[bestL]), y2=fabsf(corrBest[bestL+1]);
    float den = (y0 - 2*y1 + y2);
    if(fabsf(den) > 1e-9f) lagSamples = bestL + 0.5f*(y0-y2)/den;
  }
  float lagMs = lagSamples * (SAMPLE_US/1000.0f);

  Serial.print(F("   best pair: gyro axis ")); Serial.print(bestPair/3);
  Serial.print(F(" vs dEuler axis "));        Serial.print(bestPair%3);
  Serial.print(F("   peak r = "));            Serial.println(bestR,4);
  Serial.print(F("   peak lag = "));          Serial.print(lagSamples,2);
  Serial.print(F(" samples = "));             Serial.print(lagMs,2); Serial.println(F(" ms"));
  Serial.println(F("   correlation vs lag (paste this too -- it shows whether the peak is clean):"));
  Serial.println(F("   lag_samples,lag_ms,r"));
  for(int L=0;L<=MAXLAG;L++){
    Serial.print(F("   ")); Serial.print(L); Serial.print(',');
    Serial.print(L*(SAMPLE_US/1000.0f),2); Serial.print(',');
    Serial.println(corrBest[L],4);
  }
  Serial.println();
  *outCorr = bestR; *outBestPair = bestPair;
  return lagMs;
}

// ---- Phase 4: flight-representative loop period -----------------------------
static float phaseLoopPeriod(float* mn, float* mx){
  Serial.println(F("-- PHASE 4: loop period (flight-representative) ------------"));
  static uint32_t d[NLOOP_REPS];
  for(int i=0;i<NLOOP_REPS;i++){
    uint32_t a=micros();
    volatile imu::Vector<3> e = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
    volatile imu::Vector<3> g = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    (void)e;(void)g;
    volatile float u = 0;                        // stand-in for the PD + trim arithmetic
    for(int k=0;k<40;k++) u += sqrtf((float)k)*0.001f;
    d[i]=micros()-a;
  }
  uint32_t lo=0xFFFFFFFF, hi=0;
  for(int i=0;i<NLOOP_REPS;i++){ if(d[i]<lo)lo=d[i]; if(d[i]>hi)hi=d[i]; }
  float med = medianOf(d, NLOOP_REPS);
  Serial.print(F("  loop period [ms]  min ")); Serial.print(lo/1000.0f,3);
  Serial.print(F("   median "));               Serial.print(med/1000.0f,3);
  Serial.print(F("   max "));                  Serial.println(hi/1000.0f,3);
  Serial.println(F("  (sensor reads + representative control arithmetic; no SD, no servo write)"));
  Serial.println();
  *mn=lo/1000.0f; *mx=hi/1000.0f;
  return med/1000.0f;
}

void loop(){
  static bool done=false;
  if(done){ delay(1000); return; }
  done = true;

  phaseI2C();
  float Tout = phaseOutputPeriod();
  float r=0; int pair=0;
  float tFus = phaseFusionDelay(&r, &pair);
  float lmn, lmx;
  float Tloop = phaseLoopPeriod(&lmn, &lmx);

  Serial.println(F("==================== SUMMARY ===================="));
  Serial.print(F("fusion delay          [ms] = ")); Serial.print(tFus,2);
  Serial.print(F("   (peak r = ")); Serial.print(r,3); Serial.println(F(")"));
  Serial.print(F("fused output period   [ms] = ")); Serial.println(Tout,3);
  Serial.print(F("loop period median    [ms] = ")); Serial.println(Tloop,3);
  Serial.println();
  Serial.println(F("tau_sensor+compute, mean case:"));
  Serial.println(F("   fusion delay  +  half the output period  +  half the loop period"));
  float tauMean = tFus + 0.5f*Tout + 0.5f*Tloop;
  Serial.print(F("   = ")); Serial.print(tFus,2); Serial.print(F(" + ")); Serial.print(0.5f*Tout,2);
  Serial.print(F(" + ")); Serial.print(0.5f*Tloop,2);
  Serial.print(F(" = ")); Serial.print(tauMean,2); Serial.print(F(" ms = "));
  Serial.print(tauMean/1000.0f,4); Serial.println(F(" s"));
  Serial.println(F("tau_sensor+compute, worst case (full periods):"));
  float tauMax = tFus + Tout + lmx;
  Serial.print(F("   = ")); Serial.print(tauMax,2); Serial.print(F(" ms = "));
  Serial.print(tauMax/1000.0f,4); Serial.println(F(" s"));
  Serial.println();
  Serial.print(F("For reference the paper currently ASSUMES tau = 0.035 s. Ratio (mean) = "));
  Serial.println((tauMean/1000.0f)/0.035f,2);
  Serial.println(F("NOTE: this is the sensor+compute transport delay -- the quantity the"));
  Serial.println(F("  gain-ceiling law is indexed on. The ACTUATOR lag is a separate parameter"));
  Serial.println(F("  the simulator holds fixed; it is NOT added here on purpose."));
  Serial.println(F("================================================"));
  Serial.println(F("# Paste everything from PHASE 1 down."));
}
