// SD.h -- ASCENT-SIL shim that WRITES THE REAL FLIGHT CSV to the host filesystem.
// Unlike the shared ../sim/shims/SD.h (a no-op), this runs the firmware's ACTUAL logging
// path (createUniqueLogFile -> logData -> snprintf -> phase) end-to-end so we can inspect
// the exact bytes the flight computer would write to the SD card.
//   Output path : $ASCENT_SIL_LOG if set, else the firmware's chosen name (ASC###.CSV) in CWD.
//   First open of the process truncates + writes the header; later opens append -- matching
//   the Teensy SD's FILE_WRITE (append) semantics that logData() relies on.
// Takes precedence over the shared shim because the ascent build passes -I shims first.
#pragma once
#include "Arduino.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

inline const char* _sil_logpath(const char* name){
  const char* env = getenv("ASCENT_SIL_LOG");
  return (env && *env) ? env : name;
}

class File {
  FILE* f;
public:
  File(FILE* fp=nullptr): f(fp){}
  explicit operator bool() const { return f != nullptr; }
  size_t println(const char* s){ if(!f) return 0; int n=fprintf(f,"%s\n",s); return n>0?(size_t)n:0; }
  size_t print(const char* s){ if(!f||!s) return 0; return fputs(s,f)>=0?strlen(s):0; }
  size_t write(const uint8_t* b, size_t n){ return f? fwrite(b,1,n,f):0; }
  void flush(){ if(f) fflush(f); }
  void close(){ if(f){ fclose(f); f=nullptr; } }
};

class SDClass {
  bool firstOpen = true;
public:
  bool begin(int){ return true; }
  bool exists(const char*){ return false; }   // SIL: always "fresh" so createUniqueLogFile picks ASC000
  File open(const char* name, int){
    FILE* fp = fopen(_sil_logpath(name), firstOpen ? "w" : "a");  // first open truncates+header, rest append
    if(fp) firstOpen = false;
    return File(fp);
  }
};
extern SDClass SD;
