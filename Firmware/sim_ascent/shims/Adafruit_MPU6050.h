// Adafruit_MPU6050.h — ASCENT-SIL shim. getEvent() steps the harness (sim_advance -> clock + physics, like
// tockn's update()) and returns the mock IMU with the library's REAL behavior: accel in m/s^2, gyro in rad/s.
// The sim already clamps g_imu_* at the selected FSR, and Adafruit auto-scales on hardware, so there is NO manual
// rescale factor here -- that is the whole point of the driver swap (kills the old ACC_FIX/GYRO_FIX patch).
#pragma once
#include "Arduino.h"
#include "Adafruit_Sensor.h"
enum mpu6050_accel_range_t { MPU6050_RANGE_2_G=0, MPU6050_RANGE_4_G, MPU6050_RANGE_8_G, MPU6050_RANGE_16_G };
enum mpu6050_gyro_range_t  { MPU6050_RANGE_250_DEG=0, MPU6050_RANGE_500_DEG, MPU6050_RANGE_1000_DEG, MPU6050_RANGE_2000_DEG };
enum mpu6050_bandwidth_t   { MPU6050_BAND_260_HZ=0, MPU6050_BAND_184_HZ, MPU6050_BAND_94_HZ,
                             MPU6050_BAND_44_HZ, MPU6050_BAND_21_HZ, MPU6050_BAND_10_HZ, MPU6050_BAND_5_HZ };
class Adafruit_MPU6050 {
  int accR = MPU6050_RANGE_16_G, gyR = MPU6050_RANGE_2000_DEG;
public:
  bool begin(unsigned char = 0x68, void* = 0, int = 0){ return true; }
  void setAccelerometerRange(int r){ accR = r; }
  void setGyroRange(int r){ gyR = r; }
  void setFilterBandwidth(int){}
  int  getAccelerometerRange(){ return accR; }
  int  getGyroRange(){ return gyR; }
  bool getEvent(sensors_event_t* a, sensors_event_t* g, sensors_event_t* t){
    // ---- LOOP TIMING (rewritten 2026-08-08) ----------------------------------------------------
    // This used to be a hard-coded sim_advance(5) -- every control iteration cost exactly 5 ms, forever.
    // That single line is the reason the harness could not express ASC038: with a constant loop period
    // the dt guard in sensors() could never trip, the loop-health governor could never shed work, and
    // the dt clamp could never engage. A fixed clock does not just mis-model timing, it makes an entire
    // CLASS of failure structurally invisible, and a PASS then means nothing about that class.
    // Now the per-iteration cost is: a base I2C read time, plus optional injected stall (--imuslow),
    // plus optional random jitter (--loopjitter). Defaults reproduce the old 5 ms behaviour exactly so
    // existing results stay comparable.
    extern double g_imuBaseMs, g_imuSlowMs, g_loopJitterMs, g_loopJitterFlightMs;
    extern bool   g_inFlightNow;
    extern double sim_frand();                         // deterministic, seeded from --seed
    // ---- I2C BUS HANG (--i2chang) ----
    // FAILURE_MODES D2: Teensy's Wire has no timeout set, so a bus that latches SDA low blocks the
    // control loop FOREVER. --imustuck models a wedged SENSOR (the call returns, with stale data);
    // this models a wedged BUS, where the call never returns at all. They are different failures and
    // only this one reaches the watchdog. Modelled by burning simulated time without ever producing a
    // sample: sim_advance() longjmps out when the watchdog deadline passes, which is exactly what the
    // hardware does. The bound is a safety net for a build with WATCHDOG_ENABLE 0 -- see sim_advance.
    // --i2chang: the bus stops responding. With Wire.setWireTimeout() the call now COSTS the timeout
    // and returns FAILURE rather than blocking, so this models the firmware as it actually is. The
    // failed read then drives failover -> both-dead -> blind abort, which is the designed response.
    extern bool sim_i2cHung(); extern bool sim_firmwareHung();
    if(sim_i2cHung()){ sim_advance(50); return false; }         // 50 ms = WireIMXRT's own bail-out
    // --firmwarehang: a genuine lockup (infinite loop, not a bus fault). Nothing returns and nothing
    // feeds WDOG1, which is precisely what the watchdog exists for. sim_advance() longjmps out when
    // the deadline passes; the bound is only a safety net for a WATCHDOG_ENABLE 0 build.
    if(sim_firmwareHung()){ for(int guard=0; guard<20000; guard++) sim_advance(10); }
    double cost = g_imuBaseMs + g_imuSlowMs;
    if(g_loopJitterMs > 0) cost += g_loopJitterMs * sim_frand();
    // in-flight-only degradation: invisible to the pad gate, which is the case the governor is for
    if(g_inFlightNow && g_loopJitterFlightMs > 0) cost += g_loopJitterFlightMs * sim_frand();
    sim_advance((unsigned long)(cost + 0.5));
    const double D2R = M_PI/180.0;
    g->gyro.x=(float)(g_imu_gyroX*D2R); g->gyro.y=(float)(g_imu_gyroY*D2R); g->gyro.z=(float)(g_imu_gyroZ*D2R);   // rad/s
    a->acceleration.x=(float)(g_imu_accX*9.80665);     // m/s^2 (g_imu_acc* are in g)
    a->acceleration.y=(float)(g_imu_accY*9.80665);
    a->acceleration.z=(float)(g_imu_accZ*9.80665);
    if(t) t->temperature = 25.0f;
    return true;
  }
};
