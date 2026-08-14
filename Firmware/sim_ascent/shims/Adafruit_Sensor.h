// Adafruit_Sensor.h — minimal SIL shim: just the sensors_event_t fields Ascent_TVC.ino reads.
// The real struct is a tagged union (accel OR gyro per event); the firmware fills separate accel/gyro
// events via getEvent(&a,&g,&t) and reads a.acceleration.* and g.gyro.*, so plain members suffice here.
#pragma once
typedef struct { float x, y, z; } sim_svec3;
typedef struct {
  sim_svec3 acceleration;   // m/s^2
  sim_svec3 gyro;           // rad/s
  float     temperature;    // deg C
} sensors_event_t;
