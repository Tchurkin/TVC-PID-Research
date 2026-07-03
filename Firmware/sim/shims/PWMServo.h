// PWMServo.h — host shim. write(deg) is captured into g_servoDeg[pin] for the physics model.
#pragma once
#include "Arduino.h"
class PWMServo {
  int pin = -1;
public:
  uint8_t attach(int p){ pin = p; return 1; }
  void write(int deg){ if(pin>=0 && pin<64) g_servoDeg[pin] = deg; }
  void write(double deg){ write((int)(deg+0.5)); }
};
