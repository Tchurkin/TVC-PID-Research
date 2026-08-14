/*
  Impulse22_Bringup.ino — first-power-on sensor bring-up for Impulse 2.2
  ======================================================================
  Read-only. Drives NOTHING except the RGB LED and buzzer. Pyro gates are
  forced LOW in the first lines of setup() and never touched again. No servo
  object is created, so no servo pin is ever driven.

  Pin map verified pad-by-pad against PCBs/Impulse_2.2_kicad/Impulse_2.2.kicad_pcb
  (2026-08-12). It is NOT the same as the old Sysiphus sketches: those use
  Serial1 for GPS (= pins 0/1 = CS_IMU/MISO here) and servo pin 2 (= INT_IMU).

  SENSORS
    Sensor board, SPI1 @ MOSI 26 / MISO 1 / SCK 27:
      ICM-42688-P  CS = pin 0    WHO_AM_I 0x75 -> 0x47
      DPS310       CS = pin 15   PROD_ID  0x0D -> 0x10
    Main board, I2C (Wire) @ SDA 18 / SCL 19:
      MPU6050      0x68          WHO_AM_I 0x75 -> 0x68
      BMP280       0x76 or 0x77  CHIPID   0xD0 -> 0x58
    GPS ATGM336H on Serial4 (RX4 = 16), PPS on pin 20.

  PHASES — press the button to advance, hold >1 s to jump back to phase 0.
    0  INVENTORY   white   one-shot bus scan + ID check + rail voltages
    1  RAILS       yellow  VBAT / 5V-DIRTY / 7.4V via the on-board dividers
    2  ICM-42688   blue    primary IMU: accel g, gyro dps, temp
    3  DPS310      cyan    primary baro: hPa, degC, altitude m
    4  MPU6050     green   backup IMU
    5  BMP280      purple  backup baro
    6  GPS         red     NMEA sentence count, fix, sats, PPS edges
    7  CROSS-CHECK white   both IMUs and both baros side by side

  Phase 7 is the one that catches wiring mistakes a single-sensor readout
  cannot: if ICM and MPU disagree on which axis gravity is on, you have an
  orientation or harness problem, not a bad chip.
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// ── Impulse 2.2 pin map ──────────────────────────────────────────────────────
constexpr int PIN_CS_IMU    = 0;    // SPI1 CS  -> ICM-42688-P
constexpr int PIN_INT_IMU   = 2;    // ICM-42688-P INT1
constexpr int PIN_RLED      = 6;    // RGB LED1 is COMMON ANODE to 5V-CLEAN:
constexpr int PIN_GLED      = 7;    //   drive LOW = on, HIGH = off
constexpr int PIN_BLED      = 8;
constexpr int PIN_PYRO[4]   = {9, 10, 11, 12};
constexpr int PIN_BUZZER    = 13;   // gate of Q22, active HIGH
constexpr int PIN_BUTTON    = 14;   // 10k pulldown R23, pressed = HIGH
constexpr int PIN_CS_BARO   = 15;   // SPI1 CS  -> DPS310
constexpr int PIN_GPS_PPS   = 20;
constexpr int PIN_VBAT_SENSE     = 21;
constexpr int PIN_SERVO_V_SENSE  = 22;
constexpr int PIN_PYRO_V_SENSE   = 23;
constexpr int PIN_MOSI1     = 26;
constexpr int PIN_SCK1      = 27;
constexpr int PIN_MISO1     = 1;
constexpr int PIN_SENS_DET  = 32;   // LOW = sensor-board harness present

#define GPS_SERIAL Serial4          // NOT Serial1 — pins 0/1 are CS_IMU/MISO
constexpr uint32_t GPS_BAUD = 9600;

// Divider ratios from the schematic: Vrail = Vpin / ratio
constexpr float VBAT_RATIO  = 4.7f  / (10.0f + 4.7f);    // R54 10k / R55 4.7k
constexpr float SERVO_RATIO = 10.0f / (10.0f + 10.0f);   // R56 10k / R57 10k
constexpr float PYRO_RATIO  = 47.0f / (100.0f + 47.0f);  // R58 100k / R59 47k
constexpr float ADC_VREF    = 3.30f;
constexpr int   ADC_BITS    = 12;

constexpr float SEA_LEVEL_HPA = 1013.25f;

// ── SPI settings ─────────────────────────────────────────────────────────────
// Conservative for bring-up. ICM tolerates 24 MHz, DPS310 10 MHz.
SPISettings ICM_SPI(8000000, MSBFIRST, SPI_MODE0);
SPISettings DPS_SPI(4000000, MSBFIRST, SPI_MODE0);

// ── ICM-42688-P registers (bank 0) ───────────────────────────────────────────
constexpr uint8_t ICM_DEVICE_CONFIG = 0x11;
constexpr uint8_t ICM_INTF_CONFIG0  = 0x4C;
constexpr uint8_t ICM_PWR_MGMT0     = 0x4E;
constexpr uint8_t ICM_GYRO_CONFIG0  = 0x4F;
constexpr uint8_t ICM_ACCEL_CONFIG0 = 0x50;
constexpr uint8_t ICM_TEMP_DATA1    = 0x1D;   // burst start: 14 bytes to 0x2A
constexpr uint8_t ICM_WHO_AM_I      = 0x75;
constexpr uint8_t ICM_WHOAMI_EXPECT = 0x47;

// ±16 g and ±2000 dps => these LSB scalings
constexpr float ICM_ACC_LSB_PER_G   = 2048.0f;
constexpr float ICM_GYR_LSB_PER_DPS = 16.4f;

// ── DPS310 registers ─────────────────────────────────────────────────────────
constexpr uint8_t DPS_PSR_B2   = 0x00;
constexpr uint8_t DPS_PRS_CFG  = 0x06;
constexpr uint8_t DPS_TMP_CFG  = 0x07;
constexpr uint8_t DPS_MEAS_CFG = 0x08;
constexpr uint8_t DPS_CFG_REG  = 0x09;
constexpr uint8_t DPS_RESET    = 0x0C;
constexpr uint8_t DPS_PROD_ID  = 0x0D;
constexpr uint8_t DPS_COEF     = 0x10;
constexpr uint8_t DPS_COEF_SRCE= 0x28;
constexpr uint8_t DPS_ID_EXPECT= 0x10;

// Oversampling scale factors (datasheet Table "Compensation Scale Factors")
constexpr float DPS_KP = 7864320.0f;   // 8x  oversampling  (PM_PRC = 0011)
constexpr float DPS_KT =  524288.0f;   // 1x  oversampling  (TMP_PRC = 0000)

struct DpsCoef {
  int32_t c0, c1, c00, c10;
  int32_t c01, c11, c20, c21, c30;
} dpsCoef;

// ── Device presence flags, filled by the inventory phase ─────────────────────
bool haveICM = false, haveDPS = false, haveMPU = false, haveBMP = false;
uint8_t bmpAddr = 0x00;

// Filled in by a *Begin() on failure, printed by the caller after OK/FAIL so the
// verdict and its explanation don't interleave.
char diag[80];

Adafruit_BMP280 bmp;

volatile uint32_t ppsCount = 0;
void ppsISR() { ppsCount++; }

bool     icmStale   = false;   // ICM returned its no-data sentinel
uint32_t brownouts  = 0;       // VBAT dropped below UVLO and came back
bool     railWasUp  = false;
bool     railSeenUp = false;

// ─────────────────────────────────────────────────────────────────────────────
//  LED / buzzer / button
// ─────────────────────────────────────────────────────────────────────────────
void LED(bool r, bool g, bool b) {
  digitalWrite(PIN_RLED, !r);   // common anode -> inverted
  digitalWrite(PIN_GLED, !g);
  digitalWrite(PIN_BLED, !b);
}
void beep(int freq, int dur) { tone(PIN_BUZZER, freq); delay(dur); noTone(PIN_BUZZER); }

// Returns 0 = not pressed, 1 = short press, 2 = long press (>1 s)
int readButton() {
  if (digitalRead(PIN_BUTTON) != HIGH) return 0;
  delay(30);                                        // debounce
  if (digitalRead(PIN_BUTTON) != HIGH) return 0;
  uint32_t t0 = millis();
  while (digitalRead(PIN_BUTTON) == HIGH) {
    if (millis() - t0 > 1000) {                     // long press: confirm + wait for release
      beep(330, 200);
      while (digitalRead(PIN_BUTTON) == HIGH) delay(10);
      return 2;
    }
    delay(10);
  }
  return 1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SPI1 primitives
// ─────────────────────────────────────────────────────────────────────────────
uint8_t spiReadReg(int cs, SPISettings s, uint8_t reg) {
  SPI1.beginTransaction(s);
  digitalWrite(cs, LOW);
  SPI1.transfer(reg | 0x80);                        // MSB set = read
  uint8_t v = SPI1.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI1.endTransaction();
  return v;
}

void spiReadBurst(int cs, SPISettings s, uint8_t reg, uint8_t* buf, uint8_t n) {
  SPI1.beginTransaction(s);
  digitalWrite(cs, LOW);
  SPI1.transfer(reg | 0x80);
  for (uint8_t i = 0; i < n; i++) buf[i] = SPI1.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI1.endTransaction();
}

void spiWriteReg(int cs, SPISettings s, uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(s);
  digitalWrite(cs, LOW);
  SPI1.transfer(reg & 0x7F);                        // MSB clear = write
  SPI1.transfer(val);
  digitalWrite(cs, HIGH);
  SPI1.endTransaction();
}

static int32_t signExtend(uint32_t v, uint8_t bits) {
  if (v & (1UL << (bits - 1))) v |= (0xFFFFFFFFUL << bits);
  return (int32_t)v;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ICM-42688-P
// ─────────────────────────────────────────────────────────────────────────────
bool icmBegin() {
  spiWriteReg(PIN_CS_IMU, ICM_SPI, ICM_DEVICE_CONFIG, 0x01);   // soft reset
  delay(5);

  uint8_t who = spiReadReg(PIN_CS_IMU, ICM_SPI, ICM_WHO_AM_I);
  if (who != ICM_WHOAMI_EXPECT) {
    snprintf(diag, sizeof diag, "WHO_AM_I = 0x%02X, expected 0x47%s", who,
             who == 0xFF ? "  (MISO floating / not connected?)"
           : who == 0x00 ? "  (MISO stuck low, or board unpowered?)" : "");
    return false;
  }

  // Pin 9 of the LGA is unused here; lock the interface to SPI so a floating
  // I2C pad can never put the part into I2C mode mid-flight.
  uint8_t intf = spiReadReg(PIN_CS_IMU, ICM_SPI, ICM_INTF_CONFIG0);
  spiWriteReg(PIN_CS_IMU, ICM_SPI, ICM_INTF_CONFIG0, (intf & 0xFC) | 0x03);  // UI_SIFS_CFG = disable I2C

  spiWriteReg(PIN_CS_IMU, ICM_SPI, ICM_GYRO_CONFIG0,  (0x00 << 5) | 0x06);   // ±2000 dps, 1 kHz
  spiWriteReg(PIN_CS_IMU, ICM_SPI, ICM_ACCEL_CONFIG0, (0x00 << 5) | 0x06);   // ±16 g,     1 kHz
  spiWriteReg(PIN_CS_IMU, ICM_SPI, ICM_PWR_MGMT0, 0x0F);                     // gyro + accel low-noise
  delay(50);                                                                 // gyro start-up
  return true;
}

// ax/ay/az in g, gx/gy/gz in dps, temp in degC
void icmRead(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& tempC) {
  uint8_t b[14];
  spiReadBurst(PIN_CS_IMU, ICM_SPI, ICM_TEMP_DATA1, b, 14);
  int16_t t  = (int16_t)((b[0]  << 8) | b[1]);
  int16_t rx = (int16_t)((b[2]  << 8) | b[3]);
  int16_t ry = (int16_t)((b[4]  << 8) | b[5]);
  int16_t rz = (int16_t)((b[6]  << 8) | b[7]);
  int16_t px = (int16_t)((b[8]  << 8) | b[9]);
  int16_t py = (int16_t)((b[10] << 8) | b[11]);
  int16_t pz = (int16_t)((b[12] << 8) | b[13]);
  // 0x8000 in every channel is the ICM's "no data" sentinel — it means the part
  // is in standby with PWR_MGMT0 cleared, i.e. it reset and lost its config.
  // WHO_AM_I still answers 0x47 in that state, so an ID check will not catch it.
  icmStale = (rx == -32768 && ry == -32768 && rz == -32768);
  ax = rx / ICM_ACC_LSB_PER_G;  ay = ry / ICM_ACC_LSB_PER_G;  az = rz / ICM_ACC_LSB_PER_G;
  gx = px / ICM_GYR_LSB_PER_DPS; gy = py / ICM_GYR_LSB_PER_DPS; gz = pz / ICM_GYR_LSB_PER_DPS;
  tempC = (t / 132.48f) + 25.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  DPS310
// ─────────────────────────────────────────────────────────────────────────────
bool dpsBegin() {
  spiWriteReg(PIN_CS_BARO, DPS_SPI, DPS_RESET, 0x89);          // soft reset
  delay(50);

  uint8_t id = spiReadReg(PIN_CS_BARO, DPS_SPI, DPS_PROD_ID);
  if (id != DPS_ID_EXPECT) {
    snprintf(diag, sizeof diag, "PROD_ID = 0x%02X, expected 0x10%s", id,
             (id == 0xFF || id == 0x00) ? "  (check CS_BARO, harness pin 5)" : "");
    return false;
  }

  // Wait for the trim/coefficients to load out of OTP.
  uint32_t t0 = millis();
  while (!(spiReadReg(PIN_CS_BARO, DPS_SPI, DPS_MEAS_CFG) & 0xC0)) {   // SENSOR_RDY | COEF_RDY
    if (millis() - t0 > 500) {
      snprintf(diag, sizeof diag, "timeout waiting for SENSOR_RDY|COEF_RDY");
      return false;
    }
    delay(5);
  }

  // Infineon errata workaround: the OTP read must be re-triggered once after
  // reset or the temperature coefficients come back wrong on some lots.
  spiWriteReg(PIN_CS_BARO, DPS_SPI, 0x0E, 0xA5);
  spiWriteReg(PIN_CS_BARO, DPS_SPI, 0x0F, 0x96);
  spiWriteReg(PIN_CS_BARO, DPS_SPI, 0x62, 0x02);
  spiWriteReg(PIN_CS_BARO, DPS_SPI, 0x0E, 0x00);
  spiWriteReg(PIN_CS_BARO, DPS_SPI, 0x0F, 0x00);

  uint8_t c[18];
  spiReadBurst(PIN_CS_BARO, DPS_SPI, DPS_COEF, c, 18);
  dpsCoef.c0  = signExtend(((uint32_t)c[0] << 4) | (c[1] >> 4), 12);
  dpsCoef.c1  = signExtend((((uint32_t)c[1] & 0x0F) << 8) | c[2], 12);
  dpsCoef.c00 = signExtend(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (c[5] >> 4), 20);
  dpsCoef.c10 = signExtend((((uint32_t)c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | c[7], 20);
  dpsCoef.c01 = signExtend(((uint32_t)c[8]  << 8) | c[9],  16);
  dpsCoef.c11 = signExtend(((uint32_t)c[10] << 8) | c[11], 16);
  dpsCoef.c20 = signExtend(((uint32_t)c[12] << 8) | c[13], 16);
  dpsCoef.c21 = signExtend(((uint32_t)c[14] << 8) | c[15], 16);
  dpsCoef.c30 = signExtend(((uint32_t)c[16] << 8) | c[17], 16);

  // TMP_EXT must match the source the coefficients were trimmed against.
  uint8_t srce = spiReadReg(PIN_CS_BARO, DPS_SPI, DPS_COEF_SRCE) & 0x80;
  spiWriteReg(PIN_CS_BARO, DPS_SPI, DPS_PRS_CFG, 0x53);          // 32 Hz, 8x oversample
  spiWriteReg(PIN_CS_BARO, DPS_SPI, DPS_TMP_CFG, srce | 0x50);   // 32 Hz, 1x oversample
  spiWriteReg(PIN_CS_BARO, DPS_SPI, DPS_CFG_REG, 0x00);          // no shift needed at <=8x
  spiWriteReg(PIN_CS_BARO, DPS_SPI, DPS_MEAS_CFG, 0x07);         // continuous pressure + temp
  delay(50);
  return true;
}

void dpsRead(float& hPa, float& tempC, float& altM) {
  uint8_t b[6];
  spiReadBurst(PIN_CS_BARO, DPS_SPI, DPS_PSR_B2, b, 6);
  int32_t praw = signExtend(((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2], 24);
  int32_t traw = signExtend(((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5], 24);

  float psc = praw / DPS_KP;
  float tsc = traw / DPS_KT;

  float pcomp = dpsCoef.c00
              + psc * (dpsCoef.c10 + psc * (dpsCoef.c20 + psc * dpsCoef.c30))
              + tsc * dpsCoef.c01
              + tsc * psc * (dpsCoef.c11 + psc * dpsCoef.c21);

  tempC = dpsCoef.c0 * 0.5f + dpsCoef.c1 * tsc;
  hPa   = pcomp / 100.0f;
  altM  = 44330.0f * (1.0f - powf(hPa / SEA_LEVEL_HPA, 1.0f / 5.255f));
}

// ─────────────────────────────────────────────────────────────────────────────
//  MPU6050 (raw I2C — no library, so a library problem can't mask a wiring one)
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint8_t MPU_ADDR = 0x68;

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool mpuReadBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool mpuBegin() {
  uint8_t who;
  if (!mpuReadBytes(0x75, &who, 1)) {
    snprintf(diag, sizeof diag, "no ACK — module absent, or 3.3V rail dead");
    return false;
  }
  if (who != 0x68) {
    snprintf(diag, sizeof diag, "WHO_AM_I = 0x%02X, expected 0x68", who);
    return false;
  }
  mpuWrite(0x6B, 0x00);          // wake from sleep
  delay(10);
  mpuWrite(0x1B, 0x18);          // gyro  ±2000 dps
  mpuWrite(0x1C, 0x18);          // accel ±16 g
  mpuWrite(0x1A, 0x03);          // DLPF 44 Hz
  return true;
}

void mpuRead(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& tempC) {
  uint8_t b[14];
  if (!mpuReadBytes(0x3B, b, 14)) { ax = ay = az = gx = gy = gz = tempC = NAN; return; }
  int16_t rax = (b[0]  << 8) | b[1];
  int16_t ray = (b[2]  << 8) | b[3];
  int16_t raz = (b[4]  << 8) | b[5];
  int16_t rt  = (b[6]  << 8) | b[7];
  int16_t rgx = (b[8]  << 8) | b[9];
  int16_t rgy = (b[10] << 8) | b[11];
  int16_t rgz = (b[12] << 8) | b[13];
  ax = rax / 2048.0f;  ay = ray / 2048.0f;  az = raz / 2048.0f;   // ±16 g
  gx = rgx / 16.4f;    gy = rgy / 16.4f;    gz = rgz / 16.4f;     // ±2000 dps
  tempC = rt / 340.0f + 36.53f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Rails
// ─────────────────────────────────────────────────────────────────────────────
float readRail(int pin, float ratio) {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogRead(pin);
  float counts = acc / 16.0f;
  return (counts / ((1 << ADC_BITS) - 1)) * ADC_VREF / ratio;
}

// A brownout resets the sensors and clears their configuration, but leaves
// WHO_AM_I answering — so they look present while streaming sentinel values.
// Detect the recovery edge and re-run the init that the glitch threw away.
void checkBrownout() {
  bool up = (readRail(PIN_VBAT_SENSE, VBAT_RATIO) >= 6.0f);
  if (up && !railWasUp && railSeenUp) {
    brownouts++;
    Serial.print(F("  [VBAT dropped and recovered — brownout #"));
    Serial.print(brownouts); Serial.println(F("; re-initialising sensors]"));
    if (haveICM) icmBegin();
    if (haveDPS) dpsBegin();
  }
  if (up) railSeenUp = true;
  railWasUp = up;
}

void printRails() {
  Serial.print(F("  VBAT      ")); Serial.print(readRail(PIN_VBAT_SENSE, VBAT_RATIO), 2);
  Serial.print(F(" V   |  5V-DIRTY ")); Serial.print(readRail(PIN_SERVO_V_SENSE, SERVO_RATIO), 2);
  Serial.print(F(" V   |  7.4V pyro ")); Serial.print(readRail(PIN_PYRO_V_SENSE, PYRO_RATIO), 2);
  Serial.print(F(" V"));
  if (brownouts) { Serial.print(F("   [brownouts: ")); Serial.print(brownouts); Serial.print(F("]")); }
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
//  GPS
// ─────────────────────────────────────────────────────────────────────────────
char nmea[100];
uint8_t nmeaLen = 0;
uint32_t nmeaCount = 0;
int gpsFixQuality = -1, gpsSats = -1;
char lastGGA[100] = {0};
char lastAny[100] = {0};                 // any completed sentence, not just GGA
uint32_t gpsRawBytes = 0;                // counts bytes even if framing is wrong
const __FlashStringHelper* rx4State = 0; // captured once, before Serial4 claims pin 16

const uint32_t BAUDS[] = {9600, 4800, 19200, 38400, 57600, 115200};
constexpr int NBAUD = sizeof(BAUDS) / sizeof(BAUDS[0]);

// Three-state test: is anything actually driving this line, or is it floating?
// A powered UART transmitter idles HIGH, so a connected GPS holds pin 16 high
// against an internal pulldown. Floating means nothing is driving it at all.
const __FlashStringHelper* lineState(int pin) {
  // Check for live traffic FIRST. A transmitting UART spends most of its time
  // low, so a static pullup/pulldown probe reads it as "held low" and reports a
  // dead line when the module is in fact talking.
  pinMode(pin, INPUT);
  bool sawHigh = false, sawLow = false;
  uint32_t t0 = millis();
  while (millis() - t0 < 30) {
    if (digitalRead(pin)) sawHigh = true; else sawLow = true;
    if (sawHigh && sawLow) return F("TOGGLING     (live traffic — module is transmitting)");
  }
  pinMode(pin, INPUT_PULLDOWN); delayMicroseconds(500);
  bool withPD = digitalRead(pin);
  pinMode(pin, INPUT_PULLUP);   delayMicroseconds(500);
  bool withPU = digitalRead(pin);
  pinMode(pin, INPUT);
  if (withPD && withPU)   return F("driven HIGH  (UART idle — transmitter is alive)");
  if (!withPD && !withPU) return F("driven LOW   (held low — TX/RX swap, or module in reset)");
  return F("FLOATING     (nothing driving it — module unseated or unpowered)");
}

// Sweep the common NMEA bauds looking for ANY traffic. Zero bytes everywhere
// means the problem is electrical, not a baud-rate mismatch.
void gpsBaudScan() {
  // Serial4 must release pin 16 first — with the UART attached, its internal
  // keeper reads back as "driven HIGH" no matter what is on the wire. That made
  // the first version of this test report a live transmitter onto a dead line.
  GPS_SERIAL.end();
  delay(2);
  Serial.print(F("  RX4 (pin 16) line state: "));
  Serial.println(lineState(16));
  Serial.println(F("  Scanning bauds for traffic..."));
  uint32_t best = 0; int bestN = 0;
  for (int i = 0; i < NBAUD; i++) {
    GPS_SERIAL.end();
    GPS_SERIAL.begin(BAUDS[i]);
    delay(60);
    while (GPS_SERIAL.available()) GPS_SERIAL.read();      // flush junk from the switch
    uint32_t t0 = millis(); int n = 0, printable = 0;
    while (millis() - t0 < 1200) {
      while (GPS_SERIAL.available()) {
        char c = GPS_SERIAL.read(); n++;
        if (c == '$' || c == '\r' || c == '\n' || (c >= 32 && c < 127)) printable++;
      }
    }
    Serial.print(F("    ")); Serial.print(BAUDS[i]);
    Serial.print(F("\t")); Serial.print(n); Serial.print(F(" bytes, "));
    Serial.print(printable); Serial.println(F(" printable"));
    if (printable > bestN) { bestN = printable; best = BAUDS[i]; }
  }
  GPS_SERIAL.end();
  GPS_SERIAL.begin(best ? best : GPS_BAUD);
  Serial.print(F("  -> listening at ")); Serial.println(best ? best : GPS_BAUD);
  if (!bestN) {
    Serial.println(F("  ZERO bytes at every baud. That is electrical, not configuration:"));
    Serial.println(F("    - module actually seated in J_GPS?"));
    Serial.println(F("    - module's own pin labels vs the J_GPS silk (pinouts vary by vendor)"));
    Serial.println(F("    - TXD/RXD swapped at J_GPS or in the harness"));
    Serial.println(F("    - harness pin 6 continuity: sensor J1.6 -> main J_SENSOR1.6 -> Teensy 16"));
  }
}

void pumpGPS() {
  while (GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    gpsRawBytes++;
    if (c == '$') { nmeaLen = 0; nmea[nmeaLen++] = c; continue; }
    if (nmeaLen == 0) continue;
    if (c == '\r' || c == '\n') {
      nmea[nmeaLen] = 0;
      nmeaCount++;
      memcpy(lastAny, nmea, nmeaLen);
      lastAny[nmeaLen] = 0;
      if (nmeaLen > 6 && strstr(nmea, "GGA")) {
        memcpy(lastGGA, nmea, nmeaLen);
        lastGGA[nmeaLen] = 0;
        // field 6 = fix quality, field 7 = satellites
        int field = 0; const char* p = nmea;
        while (*p && field < 6) { if (*p == ',') field++; p++; }
        if (*p) gpsFixQuality = atoi(p);
        while (*p && field < 7) { if (*p == ',') field++; p++; }
        if (*p) gpsSats = atoi(p);
      }
      nmeaLen = 0;
      continue;
    }
    if (nmeaLen < sizeof(nmea) - 1) nmea[nmeaLen++] = c;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Inventory
// ─────────────────────────────────────────────────────────────────────────────
void i2cScan() {
  Serial.println(F("  I2C scan (SDA 18 / SCL 19):"));
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("    0x")); Serial.print(a, HEX);
      if (a == 0x68) Serial.print(F("  <- MPU6050"));
      if (a == 0x76 || a == 0x77) Serial.print(F("  <- BMP280"));
      Serial.println();
      found++;
    }
  }
  if (!found) Serial.println(F("    (nothing responded — check 3.3V and the module headers)"));
}

void inventory() {
  Serial.println(F("\n================ IMPULSE 2.2 INVENTORY ================"));

  printRails();

  // Both bucks are off below their UVLO (AP63205 3.8 V, TPS565208 4.5 V). When
  // that happens 5V-CLEAN and 3.3V are dead, and ANY sensor that still answers
  // is running on leakage through its own SPI pins' ESD clamps — those chips
  // work down to 1.7 V, so they will happily return good-looking data that
  // proves nothing about the board.
  float vbat  = readRail(PIN_VBAT_SENSE, VBAT_RATIO);
  float vserv = readRail(PIN_SERVO_V_SENSE, SERVO_RATIO);
  bool powered = (vbat >= 6.0f);

  // Every rail number above is measured against the Teensy's own 3.3 V rail as
  // the ADC reference. If that rail sags, ALL THREE readings scale together and
  // stay in correct proportion — which looks like "the battery drooped" when it
  // is really the microcontroller browning out. 5V-DIRTY is the giveaway: the
  // TPS565208 regulates to 4.93 V for any VIN above ~6 V, so if VBAT is healthy
  // and 5V-DIRTY still reads low, either the buck is damaged or the ADC is lying.
  if (powered && (vserv < 4.70f || vserv > 5.15f)) {
    Serial.println(F("\n  !! 5V-DIRTY reads out of regulation while VBAT looks OK."));
    Serial.println(F("     Three causes, and this ADC cannot tell them apart:"));
    Serial.println(F("       1. U3 genuinely out of regulation"));
    Serial.println(F("       2. Teensy 3.3 V (the ADC reference) sagging — but then"));
    Serial.println(F("          VBAT and 7.4V scale by the same factor, so check those"));
    Serial.println(F("       3. leakage on the SERVO_V_SENSE node (pin 22 / R56 / R57),"));
    Serial.println(F("          which drags this reading alone and leaves the others right"));
    Serial.println(F("     On THIS board (2026-08-12) it was #3: meter said 4.95 V while"));
    Serial.println(F("     the ADC said 3.60 V. Verify with a meter before believing this."));
  }
  if (!powered) {
    Serial.println(F("\n  *****************************************************"));
    Serial.println(F("  *  VBAT < 6 V — THE BOARD IS NOT ON BATTERY POWER.   *"));
    Serial.println(F("  *  Both bucks are below UVLO, so 5V-CLEAN and 3.3V   *"));
    Serial.println(F("  *  are dead. Any SPI sensor that answers below is    *"));
    Serial.println(F("  *  phantom-powered through its own pins and does     *"));
    Serial.println(F("  *  NOT constitute a passing test.                    *"));
    Serial.println(F("  *  Check: battery on MI, SW1 ON. Then re-run.        *"));
    Serial.println(F("  *****************************************************"));
  }

  Serial.print(F("\n  SENS_DET (pin 32): "));
  if (!powered) {
    Serial.println(F("unreadable — pull-up comes from 5V-CLEAN, which is dead"));
  } else {
    Serial.println(digitalRead(PIN_SENS_DET) == LOW
                   ? F("LOW  -> sensor-board harness CONNECTED")
                   : F("HIGH -> harness NOT detected"));
  }

  Serial.println();
  i2cScan();
  Serial.println();

  Serial.println(F("  Sensor board (SPI1):"));
  Serial.print(F("    ICM-42688-P (CS 0)  ... "));
  diag[0] = 0; haveICM = icmBegin();
  Serial.println(haveICM ? F("OK") : F("FAIL"));
  if (diag[0]) { Serial.print(F("        ")); Serial.println(diag); }

  Serial.print(F("    DPS310      (CS 15) ... "));
  diag[0] = 0; haveDPS = dpsBegin();
  Serial.println(haveDPS ? F("OK") : F("FAIL"));
  if (diag[0]) { Serial.print(F("        ")); Serial.println(diag); }

  Serial.println(F("  Main board (I2C):"));
  Serial.print(F("    MPU6050  ... "));
  diag[0] = 0; haveMPU = mpuBegin();
  Serial.println(haveMPU ? F("OK") : F("FAIL"));
  if (diag[0]) { Serial.print(F("        ")); Serial.println(diag); }

  Serial.print(F("    BMP280   ... "));
  if (bmp.begin(0x76))      { haveBMP = true; bmpAddr = 0x76; }
  else if (bmp.begin(0x77)) { haveBMP = true; bmpAddr = 0x77; }
  if (haveBMP) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1);
    Serial.print(F("OK at 0x")); Serial.println(bmpAddr, HEX);
  } else {
    Serial.println(F("FAIL"));
  }

  // A powered ATGM336H emits NMEA within a second or two of boot, with empty
  // fields until it gets a fix. Silence is never "still acquiring".
  Serial.print(F("  GPS on Serial4 ... "));
  uint32_t t0 = millis(), before = nmeaCount;
  while (millis() - t0 < 2500) pumpGPS();
  if (nmeaCount > before) {
    Serial.print(nmeaCount - before); Serial.println(F(" NMEA sentences in 2.5 s"));
  } else {
    Serial.println(F("SILENT"));
    Serial.println(powered
      ? F("        A powered module talks immediately, fix or no fix.")
      : F("        Expected while unpowered — the module needs ~30 mA."));
    Serial.print(F("        RX4 at boot was: "));
    Serial.println(rx4State ? rx4State : F("(not sampled)"));
    Serial.println(F("        Go to phase 6 for a full baud scan."));
  }

  int ok = haveICM + haveDPS + haveMPU + haveBMP;
  Serial.print(F("\n  RESULT: ")); Serial.print(ok); Serial.println(F("/4 sensors responding"));
  Serial.println(F("=======================================================\n"));

  if (ok == 4) { beep(523, 80); beep(659, 80); beep(784, 150); }
  else         { beep(220, 400); }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Phases
// ─────────────────────────────────────────────────────────────────────────────
constexpr int NUM_PHASES = 8;
int phase = 0;

const bool PHASE_LED[NUM_PHASES][3] = {
  {1,1,1},  // 0 inventory   white
  {1,1,0},  // 1 rails       yellow
  {0,0,1},  // 2 ICM         blue
  {0,1,1},  // 3 DPS310      cyan
  {0,1,0},  // 4 MPU6050     green
  {1,0,1},  // 5 BMP280      purple
  {1,0,0},  // 6 GPS         red
  {1,1,1},  // 7 cross-check white
};

const char* PHASE_NAME[NUM_PHASES] = {
  "0 INVENTORY", "1 RAILS", "2 ICM-42688-P (primary IMU)", "3 DPS310 (primary baro)",
  "4 MPU6050 (backup IMU)", "5 BMP280 (backup baro)", "6 GPS", "7 CROSS-CHECK",
};

void enterPhase(int p) {
  LED(PHASE_LED[p][0], PHASE_LED[p][1], PHASE_LED[p][2]);
  Serial.print(F("\n──── ")); Serial.print(PHASE_NAME[p]); Serial.println(F(" ────"));
  if (p == 0) inventory();
  if (p == 6) gpsBaudScan();       // ~8 s, runs once on entry
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  // Pyros OFF first, before anything else can glitch a gate.
  for (int i = 0; i < 4; i++) { pinMode(PIN_PYRO[i], OUTPUT); digitalWrite(PIN_PYRO[i], LOW); }

  pinMode(PIN_RLED, OUTPUT); pinMode(PIN_GLED, OUTPUT); pinMode(PIN_BLED, OUTPUT);
  LED(false, false, false);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);          // external 10k pulldown, do NOT use INPUT_PULLUP
  pinMode(PIN_SENS_DET, INPUT);        // driven by the LED4 string, no pullup
  pinMode(PIN_INT_IMU, INPUT);
  pinMode(PIN_GPS_PPS, INPUT);
  pinMode(PIN_CS_IMU, OUTPUT);  digitalWrite(PIN_CS_IMU, HIGH);
  pinMode(PIN_CS_BARO, OUTPUT); digitalWrite(PIN_CS_BARO, HIGH);

  analogReadResolution(ADC_BITS);
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsISR, RISING);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }    // wait for the monitor, but don't hang without one

  SPI1.setMOSI(PIN_MOSI1); SPI1.setMISO(PIN_MISO1); SPI1.setSCK(PIN_SCK1);
  SPI1.begin();
  Wire.begin();
  Wire.setClock(400000);
  rx4State = lineState(16);        // must happen BEFORE Serial4 takes the pin
  GPS_SERIAL.begin(GPS_BAUD);

  Serial.println(F("\n\n╔═══════════════════════════════════════════════╗"));
  Serial.println(F("║   IMPULSE 2.2 — SENSOR BRING-UP (read-only)   ║"));
  Serial.println(F("╚═══════════════════════════════════════════════╝"));
  Serial.println(F("Short press = next phase.  Long press (>1 s) = back to inventory."));

  enterPhase(0);
  phase = 0;
}

void loop() {
  pumpGPS();                                        // keep the GPS buffer drained in every phase

  int btn = readButton();
  if (btn == 2)      { phase = 0;                     beep(880, 120); enterPhase(phase); }
  else if (btn == 1) { phase = (phase + 1) % NUM_PHASES; beep(880, 120); enterPhase(phase); }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 250) return;
  lastPrint = millis();

  checkBrownout();

  float ax, ay, az, gx, gy, gz, tc;

  switch (phase) {

    case 0:   // inventory already printed once on entry; idle here
      break;

    case 1:
      printRails();
      break;

    case 2:
      if (!haveICM) { Serial.println(F("  ICM not present")); break; }
      icmRead(ax, ay, az, gx, gy, gz, tc);
      if (icmStale) {
        Serial.println(F("  ALL CHANNELS 0x8000 — part is in standby, config was lost."));
        Serial.println(F("  It reset (power glitch) after init. Re-initialising..."));
        haveICM = icmBegin();
        break;
      }
      Serial.print(F("  A ")); Serial.print(ax, 3); Serial.print(' '); Serial.print(ay, 3);
      Serial.print(' '); Serial.print(az, 3); Serial.print(F(" g   G "));
      Serial.print(gx, 2); Serial.print(' '); Serial.print(gy, 2); Serial.print(' ');
      Serial.print(gz, 2); Serial.print(F(" dps   T "));
      Serial.print(tc, 1); Serial.println(F(" C"));
      break;

    case 3: {
      if (!haveDPS) { Serial.println(F("  DPS310 not present")); break; }
      float hPa, altM;
      dpsRead(hPa, tc, altM);
      Serial.print(F("  ")); Serial.print(hPa, 3); Serial.print(F(" hPa   "));
      Serial.print(tc, 2); Serial.print(F(" C   alt "));
      Serial.print(altM, 2); Serial.println(F(" m"));
      break;
    }

    case 4:
      if (!haveMPU) { Serial.println(F("  MPU6050 not present")); break; }
      mpuRead(ax, ay, az, gx, gy, gz, tc);
      Serial.print(F("  A ")); Serial.print(ax, 3); Serial.print(' '); Serial.print(ay, 3);
      Serial.print(' '); Serial.print(az, 3); Serial.print(F(" g   G "));
      Serial.print(gx, 2); Serial.print(' '); Serial.print(gy, 2); Serial.print(' ');
      Serial.print(gz, 2); Serial.print(F(" dps   T "));
      Serial.print(tc, 1); Serial.println(F(" C"));
      break;

    case 5:
      if (!haveBMP) { Serial.println(F("  BMP280 not present")); break; }
      Serial.print(F("  ")); Serial.print(bmp.readPressure() / 100.0f, 3);
      Serial.print(F(" hPa   ")); Serial.print(bmp.readTemperature(), 2);
      Serial.print(F(" C   alt ")); Serial.print(bmp.readAltitude(SEA_LEVEL_HPA), 2);
      Serial.println(F(" m"));
      break;

    case 6:
      Serial.print(F("  raw bytes ")); Serial.print(gpsRawBytes);
      Serial.print(F("   NMEA ")); Serial.print(nmeaCount);
      Serial.print(F("   fix ")); Serial.print(gpsFixQuality);
      Serial.print(F("   sats ")); Serial.print(gpsSats);
      Serial.print(F("   PPS ")); Serial.println(ppsCount);
      if (lastAny[0]) { Serial.print(F("    ")); Serial.println(lastAny); }
      break;

    case 7: {   // side-by-side — the phase that catches harness and orientation errors
      float iax = NAN, iay = NAN, iaz = NAN, igx = NAN, igy = NAN, igz = NAN, itc;
      float max_ = NAN, may = NAN, maz = NAN, mgx = NAN, mgy = NAN, mgz = NAN, mtc;
      if (haveICM) icmRead(iax, iay, iaz, igx, igy, igz, itc);
      if (haveMPU) mpuRead(max_, may, maz, mgx, mgy, mgz, mtc);
      Serial.print(F("  ICM A ")); Serial.print(iax, 2); Serial.print(' ');
      Serial.print(iay, 2); Serial.print(' '); Serial.print(iaz, 2);
      Serial.print(F("  |  MPU A ")); Serial.print(max_, 2); Serial.print(' ');
      Serial.print(may, 2); Serial.print(' '); Serial.print(maz, 2);
      Serial.print(F("   ||A|| ")); Serial.print(sqrtf(iax*iax + iay*iay + iaz*iaz), 3);
      Serial.print(F(" / ")); Serial.println(sqrtf(max_*max_ + may*may + maz*maz), 3);

      if (haveDPS && haveBMP) {
        float hPa, altM;
        dpsRead(hPa, tc, altM);
        float bh = bmp.readPressure() / 100.0f;
        Serial.print(F("  DPS ")); Serial.print(hPa, 3);
        Serial.print(F(" hPa  |  BMP ")); Serial.print(bh, 3);
        Serial.print(F(" hPa  |  delta ")); Serial.print(hPa - bh, 3);
        Serial.println(F(" hPa"));
      }
      break;
    }
  }
}
