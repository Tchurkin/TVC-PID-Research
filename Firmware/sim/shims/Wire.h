// Wire.h — host shim (I2C). The mock sensors don't use the bus; this just satisfies the API.
#pragma once
#include "Arduino.h"
class TwoWire {
public:
  void begin(){}
  void begin(int){}
  void setClock(uint32_t){}
  void beginTransmission(uint8_t){}
  size_t write(uint8_t){ return 1; }
  uint8_t endTransmission(bool=true){ return 0; }
  uint8_t requestFrom(int,int){ return 0; }
  int available(){ return 0; }
  int read(){ return 0; }
};
extern TwoWire Wire;
