// SPI.h — host shim (unused by the firmware logic; SD pulls it in).
#pragma once
#include "Arduino.h"
class SPIClass { public: void begin(){} };
extern SPIClass SPI;
