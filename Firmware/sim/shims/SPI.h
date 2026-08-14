// SPI.h — host shim. SD pulls in SPI; Ascent_TVC also drives SPI1 for the sensor board
// (ICM-42688-P on CS 0, DPS310 on CS 15).
//
// transfer() deliberately returns 0x00. That makes the sensor board read as ABSENT in the SIL:
// WHO_AM_I comes back 0x00 instead of 0x47 and PROD_ID 0x00 instead of 0x10, so icm42688Begin()
// and dps310Begin() both fail, icm42688Read() trips its all-zero guard, and the firmware falls
// through to the MPU-6050 + BMP280 path the simulation actually models. Existing SIL results are
// therefore unchanged by the driver landing, and the IMU failover path gets exercised every run.
// If the sim is ever taught to model the ICM, give this a real backing store instead.
#pragma once
#include "Arduino.h"

#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef SPI_MODE0
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3
#endif

class SPISettings {
public:
  SPISettings() {}
  SPISettings(uint32_t /*clock*/, uint8_t /*bitOrder*/, uint8_t /*dataMode*/) {}
};

class SPIClass {
public:
  void    begin() {}
  void    end() {}
  void    setMOSI(int) {}
  void    setMISO(int) {}
  void    setSCK(int) {}
  void    beginTransaction(SPISettings) {}
  void    endTransaction() {}
  uint8_t transfer(uint8_t) { return 0x00; }   // sensor board reads as absent — see header note
};

extern SPIClass SPI;
extern SPIClass SPI1;
