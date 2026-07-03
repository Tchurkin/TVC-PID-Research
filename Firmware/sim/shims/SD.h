// SD.h — host shim. The firmware's own SD log is a no-op here; the harness writes the
// authoritative flight CSV (truth + firmware estimates) to stdout instead.
#pragma once
#include "Arduino.h"
class File {
  bool valid;
public:
  File(bool v=false):valid(v){}
  explicit operator bool() const { return valid; }
  size_t println(const char*){ return 0; }
  size_t print(const char*){ return 0; }
  size_t write(const uint8_t*, size_t n){ return n; }
  void flush(){}
  void close(){ valid=false; }
};
class SDClass {
public:
  bool begin(int){ return true; }
  bool exists(const char*){ return false; }   // always "new" -> firmware picks LND000
  File open(const char*, int){ return File(true); }
};
extern SDClass SD;
