// EEPROM.h — host shim. Backs the firmware's brownout-persistence with a plain RAM array.
// The SIL runs one process per flight, so this does NOT persist across runs -- which is the correct
// default: every SIL run starts from a blank record, i.e. the normal power-on path. That keeps the
// harness testing the FLIGHT path rather than accidentally testing the resume path.
// To exercise the resume logic, a test can call EEPROM.put() directly before invoking setup().
#pragma once
#include "Arduino.h"
#include <string.h>

class EEPROMClass {
  unsigned char cell[4284] = {0};        // Teensy 4.1 emulated-EEPROM size
public:
  unsigned char read(int a){ return (a>=0 && a<(int)sizeof(cell)) ? cell[a] : 0; }
  void write(int a, unsigned char v){ if(a>=0 && a<(int)sizeof(cell)) cell[a]=v; }
  template<class T> T& get(int a, T& v){
    if(a>=0 && a+(int)sizeof(T)<=(int)sizeof(cell)) memcpy(&v, cell+a, sizeof(T));
    return v;
  }
  template<class T> const T& put(int a, const T& v){
    if(a>=0 && a+(int)sizeof(T)<=(int)sizeof(cell)) memcpy(cell+a, &v, sizeof(T));
    return v;
  }
  unsigned length(){ return sizeof(cell); }
};
extern EEPROMClass EEPROM;
