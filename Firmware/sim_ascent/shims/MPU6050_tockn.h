// MPU6050_tockn.h — ASCENT-SIL shim. update() steps the harness (clock + physics) so the
// firmware's tight TVC loop (which has no delay()) actually advances sim time. Getters return
// the harness-synthesized IMU (g_imu_*), set by sim_advance().
#pragma once
#include "Arduino.h"
class MPU6050 {
public:
  MPU6050(TwoWire&){}
  void begin(){}
  void calcGyroOffsets(bool, unsigned=0, unsigned=0){}
  void update(){ sim_advance(5); }                 // 5 ms control step -> advance clock + physics
  float getGyroX(){ return (float)g_imu_gyroX; }   // deg/s
  float getGyroY(){ return (float)g_imu_gyroY; }
  float getGyroZ(){ return (float)g_imu_gyroZ; }
  float getAccX(){ return (float)g_imu_accX; }      // g
  float getAccY(){ return (float)g_imu_accY; }
  float getAccZ(){ return (float)g_imu_accZ; }
  float getAngleX(){ return atan2f((float)g_imu_accY,(float)g_imu_accZ)*180.0f/(float)M_PI; }  // deg (accel tilt)
  float getAngleY(){ return atan2f((float)g_imu_accX,(float)g_imu_accZ)*180.0f/(float)M_PI; }
  float getAngleZ(){ return 0.0f; }
  float getAccAngleX(){ return getAngleX(); }
  float getAccAngleY(){ return getAngleY(); }
  float getGyroAngleX(){ return 0.0f; }
  float getGyroAngleY(){ return 0.0f; }
};
