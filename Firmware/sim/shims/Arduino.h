// Arduino.h — host shim for the SIL (software-in-the-loop) firmware test.
// Force-included into every translation unit (-include). Provides the Arduino/Teensy
// core API the firmware uses, backed by simulator globals defined in sim_main.cpp.
#pragma once
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define A2 16
#define A3 17
#define BUILTIN_SDCARD 254
#define FILE_WRITE 1
#define F(x) (x)
typedef uint8_t byte;
typedef bool boolean;

// Arduino min/max/constrain macros — defined AFTER the C headers above so they
// don't clobber any standard-library template uses inside those headers.
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))

// ---- simulator shared state (defined in sim_main.cpp) ----
extern uint64_t g_micros;          // simulated clock (microseconds)
extern int      g_pin[64];         // digital pin states (pyros, button, LEDs)
extern int      g_servoDeg[64];    // last servo angle written, by pin
extern double   g_imu_gyroX, g_imu_gyroY, g_imu_gyroZ;   // mock IMU rates (deg/s): body X (pitch), Y (yaw), Z (roll)
extern double   g_imu_accX, g_imu_accY, g_imu_accZ;      // mock IMU specific force (g): body X, Y lateral, Z axial
extern double   g_baro_alt;        // mock barometer altitude (m)

// ---- Arduino core (defined in sim_main.cpp) ----
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int val);
int  digitalRead(int pin);
int  analogRead(int pin);
void analogReadResolution(int bits);
inline void analogReadAveraging(int){}   // Teensy-only ADC setting; no-op here so EVERY firmware config (incl.
                                         // Ascent_TVC's optional CTL_SERVO_FEEDBACK path) still builds in the SIL
void tone(int pin, unsigned int freq);
void tone(int pin, unsigned int freq, unsigned long dur);   // duration form: non-blocking on hardware
void noTone(int pin);
void sim_advance(unsigned long ms);   // step the harness clock + physics (also called by delay())

// ---- HardwareSerial (USB Serial + Serial1 for GPS) ----
class HardwareSerial {
  char rb[512]; int rh=0, rt=0;   // RX ring buffer (for GPS NMEA injected by the harness)
  bool echo;                       // mirror writes to stderr (firmware status messages)
public:
  HardwareSerial(bool e=false):echo(e){}
  explicit operator bool() const { return true; }   // mock the Teensy USB "monitor connected" flag: always true
                                                     // in the SIL so `while(!Serial && millis()<t)` waits are skipped
  void begin(unsigned long){}
  void feed(const char* s){ while(*s){ int n=(rt+1)&511; if(n!=rh){ rb[rt]=*s; rt=n; } s++; } }
  int  available(){ return (rt-rh+512)&511; }
  int  read(){ if(rh==rt) return -1; unsigned char c=rb[rh]; rh=(rh+1)&511; return (int)c; }
  void print(const char* s){ if(echo) fputs(s,stderr); }
  void print(int v){ if(echo) fprintf(stderr,"%d",v); }
  void print(double v,int d=2){ if(echo) fprintf(stderr,"%.*f",d,v); }
  void println(const char* s){ if(echo) fprintf(stderr,"%s\n",s); }
  void println(int v){ if(echo) fprintf(stderr,"%d\n",v); }
  void println(double v,int d=2){ if(echo) fprintf(stderr,"%.*f\n",d,v); }
  void println(){ if(echo) fputc('\n',stderr); }
};
extern HardwareSerial Serial;
extern HardwareSerial Serial1;
