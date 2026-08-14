/*
  Bench_Tau_ICM.ino — measure the SENSOR+COMPUTE loop delay tau on the vehicle as it is
  =====================================================================================
  UPLOAD IT. IT RUNS BY ITSELF. PASTE THE SERIAL OUTPUT. No motor, no servos, no wiring
  changes, no waving. Bench only.

  WHICH SENSOR THIS TARGETS, AND WHY
  ----------------------------------
  Ascent_TVC.ino contains NO BNO055 -- checked, the only "bno" in that file is a local
  variable `bnow` in the baro scheduler. Its inertial sources are:
      PRIMARY  ICM-42688-P  on SPI1  (CS 0, MISO 1, MOSI 26, SCK 27)
      BACKUP   MPU-6050     on I2C   (SDA 18, SCL 19)  -- not currently populated
  So the attitude that flies comes from the ICM over SPI. This sketch measures THAT path,
  using the flight firmware's own register configuration and burst read, so the number it
  reports is the number the vehicle actually has.

  WHY THIS IS MUCH SIMPLER THAN THE BNO055 CASE
  ---------------------------------------------
  A BNO055 is a fusion sensor: it runs Bosch's filter on an internal M0 and its delay is a
  black box that has to be measured by cross-correlation. The ICM-42688-P is a RAW sensor,
  and Ascent_TVC does the attitude integration itself (quatPropagate, at loop rate). So
  there is no hidden pipeline. tau decomposes into three terms, all of them knowable:

      tau = UI-filter group delay  +  sampling age  +  loop period

  and this sketch measures or reads back every one of them.

  WHAT TAU IS, PRECISELY
  ----------------------
      tools/ceiling_kd_free.py:136   tau = latency_steps / 200.0
      sim/design_space.py:285        sensor_latency_steps = latency_steps
  so the tau in `ceiling ~ 0.0661/tau` is the SENSOR->CONTROLLER transport delay. The
  actuator lag is a separate simulator parameter (tau_act = 0.05 s) held FIXED across all
  2,400 designs, so it is deliberately NOT added here.

  PRECISION TARGET: +-10%. tau enters the ceiling law linearly and that model's own
  residual spread is [0.35x, 2.03x], so do not chase +-2%.
*/
#include <SPI.h>
#include <math.h>

// ---- pins and SPI settings: copied from Ascent_TVC.ino, must not drift ----------------
constexpr int PIN_CS_IMU = 0;
static SPISettings ICM_SPI_CFG(8000000, MSBFIRST, SPI_MODE3);
constexpr float ICM_ACC_LSB_PER_G   = 2048.0f;    // +/-16 g
constexpr float ICM_GYR_LSB_PER_DPS = 16.4f;      // +/-2000 dps

constexpr int NREPS_SPI  = 500;
constexpr int NREPS_LOOP = 2000;
constexpr uint32_t ODR_WATCH_MS = 2000;

// ---- flight firmware's SPI helpers, verbatim -----------------------------------------
static void spi1Write(int cs, SPISettings s, uint8_t reg, uint8_t val){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg&0x7F); SPI1.transfer(val);
  digitalWrite(cs,HIGH); SPI1.endTransaction();
}
static uint8_t spi1Read(int cs, SPISettings s, uint8_t reg){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg|0x80); uint8_t v = SPI1.transfer(0x00);
  digitalWrite(cs,HIGH); SPI1.endTransaction(); return v;
}
static void spi1Burst(int cs, SPISettings s, uint8_t reg, uint8_t*buf, uint8_t n){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg|0x80);
  for(uint8_t i=0;i<n;i++) buf[i]=SPI1.transfer(0x00);
  digitalWrite(cs,HIGH); SPI1.endTransaction();
}

static float medianOf(uint32_t* v, int n){
  for(int i=1;i<n;i++){ uint32_t k=v[i]; int j=i-1; while(j>=0&&v[j]>k){v[j+1]=v[j];j--;} v[j+1]=k; }
  return (n&1)? (float)v[n/2] : 0.5f*((float)v[n/2-1]+(float)v[n/2]);
}

// ODR code -> Hz, from the ICM-42688-P datasheet (GYRO/ACCEL_CONFIG0 bits 3:0)
static float odrHz(uint8_t code){
  switch(code & 0x0F){
    case 0x01: return 32000; case 0x02: return 16000; case 0x03: return 8000;
    case 0x04: return 4000;  case 0x05: return 2000;  case 0x06: return 1000;
    case 0x07: return 200;   case 0x08: return 100;   case 0x09: return 50;
    case 0x0A: return 25;    case 0x0B: return 12.5;  case 0x0F: return 500;
    default: return NAN;
  }
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<4000);
  Serial.println(F("# ============================================================"));
  Serial.println(F("# Bench_Tau_ICM -- sensor+compute loop delay, ICM-42688-P on SPI1"));
  Serial.println(F("# BENCH ONLY. No motor. This is the sensor Ascent_TVC actually flies."));
  Serial.println(F("# ============================================================"));

  SPI1.setMOSI(26); SPI1.setMISO(1); SPI1.setSCK(27);
  pinMode(PIN_CS_IMU, OUTPUT); digitalWrite(PIN_CS_IMU, HIGH);
  SPI1.begin();

  // Exactly Ascent_TVC's icm42688Begin()
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x11, 0x01); delay(5);
  uint8_t who = spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x75);
  Serial.print(F("WHO_AM_I = 0x")); Serial.print(who,HEX);
  Serial.println(who==0x47 ? F("  (0x47 = ICM-42688-P, correct)") : F("  *** EXPECTED 0x47 ***"));
  if(who != 0x47){
    Serial.println(F("FATAL: ICM-42688-P not answering on SPI1."));
    Serial.println(F("  Check CS=0, MISO=1, MOSI=26, SCK=27, and that the sensor board is seated."));
    while(1) delay(1000);
  }
  uint8_t itf = spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x4C);
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x4C, (itf&0xFC)|0x03);
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x4F, 0x06);   // gyro  +/-2000 dps, 1 kHz
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x50, 0x06);   // accel +/-16 g,     1 kHz
  spi1Write(PIN_CS_IMU, ICM_SPI_CFG, 0x4E, 0x0F);   // both low-noise
  delay(50);
  Serial.println();
}

void loop(){
  static bool done=false;
  if(done){ delay(1000); return; }
  done = true;

  // ---- Phase 1: read back the ACTUAL config -------------------------------------------
  Serial.println(F("-- PHASE 1: configuration as the flight firmware leaves it -"));
  uint8_t g0 = spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x4F);
  uint8_t a0 = spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x50);
  uint8_t gaf= spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x52);   // GYRO_ACCEL_CONFIG0 (UI filter BW)
  uint8_t pwr= spi1Read(PIN_CS_IMU, ICM_SPI_CFG, 0x4E);
  Serial.print(F("  GYRO_CONFIG0  (0x4F) = 0x")); Serial.print(g0,HEX);
  Serial.print(F("   ODR = ")); Serial.print(odrHz(g0),0); Serial.println(F(" Hz"));
  Serial.print(F("  ACCEL_CONFIG0 (0x50) = 0x")); Serial.print(a0,HEX);
  Serial.print(F("   ODR = ")); Serial.print(odrHz(a0),0); Serial.println(F(" Hz"));
  Serial.print(F("  PWR_MGMT0     (0x4E) = 0x")); Serial.println(pwr,HEX);
  Serial.print(F("  GYRO_ACCEL_CONFIG0 (0x52) = 0x")); Serial.println(gaf,HEX);
  Serial.print(F("     GYRO_UI_FILT_BW  = ")); Serial.println(gaf & 0x0F);
  Serial.print(F("     ACCEL_UI_FILT_BW = ")); Serial.println((gaf>>4) & 0x0F);
  Serial.println(F("  NOTE: Ascent_TVC never writes 0x52, so this is the power-on default."));
  Serial.println(F("  Look up the group delay for this BW code at this ODR in the ICM-42688-P"));
  Serial.println(F("  datasheet (UI filter section). That is the only term not measured below."));
  Serial.println();

  // ---- Phase 2: SPI burst read cost ----------------------------------------------------
  Serial.println(F("-- PHASE 2: SPI read cost ----------------------------------"));
  static uint32_t d[NREPS_SPI];
  uint8_t b[14];
  for(int i=0;i<NREPS_SPI;i++){
    uint32_t t0=micros();
    spi1Burst(PIN_CS_IMU, ICM_SPI_CFG, 0x1D, b, 14);   // TEMP + ACCEL + GYRO, as it flies
    d[i]=micros()-t0;
  }
  uint32_t lo=0xFFFFFFFF, hi=0; for(int i=0;i<NREPS_SPI;i++){ if(d[i]<lo)lo=d[i]; if(d[i]>hi)hi=d[i]; }
  float spiMed = medianOf(d, NREPS_SPI);
  Serial.print(F("  14-byte burst [us]  min ")); Serial.print(lo);
  Serial.print(F("   median ")); Serial.print(spiMed,1);
  Serial.print(F("   max ")); Serial.println(hi);
  Serial.println();

  // ---- Phase 3: true sample update period ----------------------------------------------
  // Poll far faster than the ODR and time the intervals at which the sample words CHANGE.
  // That measures the real ODR as configured, not the nominal one.
  Serial.println(F("-- PHASE 3: actual sample update period --------------------"));
  int16_t pg = 0; bool have=false;
  uint32_t tPrev=0, nChg=0; double sum=0; uint32_t mn=0xFFFFFFFF, mx=0;
  uint32_t t0=millis();
  while(millis()-t0 < ODR_WATCH_MS){
    spi1Burst(PIN_CS_IMU, ICM_SPI_CFG, 0x1D, b, 14);
    int16_t gz = (int16_t)((b[12]<<8)|b[13]);
    if(!have){ pg=gz; tPrev=micros(); have=true; continue; }
    if(gz != pg){
      uint32_t now=micros(), dt=now-tPrev;
      if(nChg){ sum+=dt; if(dt<mn)mn=dt; if(dt>mx)mx=dt; }
      nChg++; tPrev=now; pg=gz;
    }
  }
  float per = nChg>1 ? (float)(sum/(nChg-1))/1000.0f : NAN;
  Serial.print(F("  changes seen         = ")); Serial.println(nChg);
  Serial.print(F("  update period  [ms]  = ")); Serial.print(per,3);
  Serial.print(F("   min ")); Serial.print(mn/1000.0f,3);
  Serial.print(F("   max ")); Serial.println(mx/1000.0f,3);
  Serial.print(F("  -> effective rate [Hz] = ")); Serial.println(per>0?1000.0f/per:0.0f,1);
  Serial.println(F("  (should be ~1.000 ms = 1000 Hz; the read is faster than the ODR, so"));
  Serial.println(F("   the sample the loop gets is up to one ODR period old.)"));
  Serial.println();

  // ---- Phase 4: flight-representative loop period ---------------------------------------
  Serial.println(F("-- PHASE 4: loop period ------------------------------------"));
  static uint32_t dl[NREPS_LOOP];
  for(int i=0;i<NREPS_LOOP;i++){
    uint32_t a=micros();
    spi1Burst(PIN_CS_IMU, ICM_SPI_CFG, 0x1D, b, 14);
    // stand-in for quatPropagate + tilt decomposition + PD + trim, at the flown cost
    volatile float q0=1,q1=0,q2=0,q3=0, u=0;
    for(int k=0;k<25;k++){
      float wx=0.01f*k, wy=0.002f*k, wz=0.003f*k, dt=0.0035f;
      float dw=0.5f*(-q1*wx-q2*wy-q3*wz), dx=0.5f*(q0*wx+q2*wz-q3*wy);
      float dy=0.5f*(q0*wy-q1*wz+q3*wx), dz=0.5f*(q0*wz+q1*wy-q2*wx);
      q0+=dw*dt; q1+=dx*dt; q2+=dy*dt; q3+=dz*dt;
      float n=sqrtf(q0*q0+q1*q1+q2*q2+q3*q3); q0/=n;q1/=n;q2/=n;q3/=n;
      u += atan2f(q2,q3);
    }
    dl[i]=micros()-a;
  }
  uint32_t llo=0xFFFFFFFF, lhi=0; for(int i=0;i<NREPS_LOOP;i++){ if(dl[i]<llo)llo=dl[i]; if(dl[i]>lhi)lhi=dl[i]; }
  float loopMed = medianOf(dl, NREPS_LOOP);
  Serial.print(F("  loop period [ms]  min ")); Serial.print(llo/1000.0f,3);
  Serial.print(F("   median ")); Serial.print(loopMed/1000.0f,3);
  Serial.print(F("   max ")); Serial.println(lhi/1000.0f,3);
  Serial.println(F("  (SPI read + quaternion propagate + tilt + PD; no SD, no servo write)"));
  Serial.println();

  // ---- Summary --------------------------------------------------------------------------
  float tSpi  = spiMed/1000.0f;
  float tSamp = 0.5f*per;                 // mean age of the sample the loop reads
  float tLoop = 0.5f*(loopMed/1000.0f);   // mean age of the command within one loop
  float tauMeasured = tSamp + tLoop;
  Serial.println(F("==================== SUMMARY ===================="));
  Serial.print(F("SPI burst read       [ms] = ")); Serial.println(tSpi,3);
  Serial.print(F("sample update period [ms] = ")); Serial.println(per,3);
  Serial.print(F("loop period median   [ms] = ")); Serial.println(loopMed/1000.0f,3);
  Serial.println();
  Serial.println(F("tau_sensor+compute, MEASURED terms (mean case):"));
  Serial.print(F("   half the sample period  ")); Serial.print(tSamp,3);
  Serial.print(F("  +  half the loop period  ")); Serial.print(tLoop,3);
  Serial.print(F("  =  ")); Serial.print(tauMeasured,3); Serial.println(F(" ms"));
  Serial.println(F("PLUS the ICM UI-filter group delay for the BW code printed in PHASE 1,"));
  Serial.println(F("which is a datasheet lookup, not something this sketch can measure."));
  Serial.println();
  Serial.print(F("Worst case (full periods): "));
  Serial.print(per + lhi/1000.0f, 3); Serial.println(F(" ms + filter delay"));
  Serial.println();
  Serial.println(F("The paper currently ASSUMES tau = 0.035 s = 35.0 ms."));
  Serial.println(F("NOTE: sensor+compute only. The ACTUATOR lag is a separate simulator"));
  Serial.println(F("  parameter held fixed at 0.05 s and is deliberately NOT summed here."));
  Serial.println(F("================================================"));
  Serial.println(F("# Paste everything from PHASE 1 down."));
}
