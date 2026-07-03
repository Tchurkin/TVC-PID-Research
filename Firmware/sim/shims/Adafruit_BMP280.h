// Adafruit_BMP280.h — host shim. readAltitude() returns the harness-synthesized barometer.
#pragma once
#include "Arduino.h"
class Adafruit_BMP280 {
public:
  enum sensor_mode { MODE_NORMAL=3 };
  enum sensor_sampling { SAMPLING_X2=2, SAMPLING_X16=5 };
  enum sensor_filter { FILTER_X16=4 };
  enum standby_duration { STANDBY_MS_1=0 };
  Adafruit_BMP280(){}
  bool begin(uint8_t=0x77){ return true; }
  void setSampling(sensor_mode, sensor_sampling, sensor_sampling, sensor_filter, standby_duration){}
  float readAltitude(float){ return (float)g_baro_alt; }
};
