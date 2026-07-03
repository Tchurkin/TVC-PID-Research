// MPU6050_tockn.h — host shim. Returns synthetic IMU values the harness computes from
// the true rocket state (set in g_imu_*). Matches the methods the firmware calls.
#pragma once
#include "Arduino.h"
#include "Wire.h"
class MPU6050 {
public:
  MPU6050(TwoWire&){}
  void begin(){}
  void calcGyroOffsets(bool){}     // real bias removal is done by the firmware's padCalibrate()
  void update(){}
  float getGyroX(){ return (float)g_imu_gyroX; }   // deg/s, body X (pitch plane)
  float getGyroY(){ return (float)g_imu_gyroY; }   // deg/s, body Y (yaw plane)
  float getGyroZ(){ return (float)g_imu_gyroZ; }   // deg/s, body Z (roll)
  float getAccX(){ return (float)g_imu_accX; }      // g, body lateral X
  float getAccY(){ return (float)g_imu_accY; }      // g, body lateral Y
  float getAccZ(){ return (float)g_imu_accZ; }      // g, body axial
  float getAccAngleX(){ return 0.0f; }
  float getAccAngleY(){ return 0.0f; }
  float getGyroAngleX(){ return 0.0f; }
  float getGyroAngleY(){ return 0.0f; }
};
