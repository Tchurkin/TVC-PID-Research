/*
  Impulse22_SensorCheck.ino — IMU + barometer + RELATIVE GPS, in metres.
  =====================================================================
  Read-only. Drives nothing but the status LEDs. Pyro gates are forced LOW in the
  first line of setup() and never touched again; no servo object is constructed.

  Everything is reported in the units you actually care about on a range:
    IMU    ICM-42688-P (SPI1, CS 0)    accel g + |A|, gyro dps, die temp
    BARO   DPS310      (SPI1, CS 15)   hPa, temp, and altitude AGL relative to a
                                       captured ground pressure -- not a sea-level guess
    GPS    ATGM336H    (Serial4)       North / East / Up in METRES from the first fix

  BUTTON  short press re-zeroes both origins (ground pressure AND the GPS datum),
          so you can set it down, zero, then walk a known distance and check it.

  PACK LEDS, so this is usable outdoors with no laptop:
    green  (LED2, pin 30)  both SPI sensors answering
    yellow (LED3, pin 31)  GPS has a position fix

  GPS PROJECTION
    Local tangent plane about the first fix, using the WGS84 meridional and
    prime-vertical radii evaluated at that latitude:
        north = M(lat0) * dlat_rad
        east  = N(lat0) * cos(lat0) * dlon_rad
    Good to well under a metre over the few-km scale a rocket ever sees, and it
    beats the usual flat 111320 m/deg by correcting for both flattening and
    latitude. Up comes from the GGA MSL altitude, also relative to the datum.

    Expect several metres of scatter on a consumer GPS even sitting still, and
    Up is always noisier than North/East -- that is the receiver, not this code.
    The barometer is the altitude source worth trusting; GPS Up is a sanity check.
*/

#include <SPI.h>
#include <math.h>

// ── Impulse 2.2 pin map ──────────────────────────────────────────────────────
constexpr int PIN_CS_IMU  = 0,  PIN_CS_BARO = 15;
constexpr int PIN_MISO1   = 1,  PIN_MOSI1   = 26, PIN_SCK1 = 27;
constexpr int PIN_PYRO[4] = {9, 10, 11, 12};
constexpr int PIN_RLED    = 6,  PIN_GLED    = 7,  PIN_BLED = 8;
constexpr int PIN_BUZZER  = 13, PIN_BUTTON  = 14;
constexpr int PIN_GPS_PPS = 20;
constexpr int PIN_MAIN_LED = 30, PIN_PYRO_LED = 31;   // active HIGH
#define GPS_SERIAL Serial4                            // NOT Serial1: pins 0/1 are CS_IMU/MISO
constexpr uint32_t GPS_BAUD = 9600;

SPISettings ICM_CFG(8000000, MSBFIRST, SPI_MODE0);
SPISettings DPS_CFG(4000000, MSBFIRST, SPI_MODE0);

constexpr float G0 = 9.80665f;
constexpr float ACC_LSB_PER_G = 2048.0f;    // ±16 g
constexpr float GYR_LSB_PER_DPS = 16.4f;    // ±2000 dps

bool haveICM = false, haveDPS = false;

// ── SPI1 helpers ─────────────────────────────────────────────────────────────
static uint8_t rd(int cs, SPISettings s, uint8_t reg){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg|0x80); uint8_t v=SPI1.transfer(0);
  digitalWrite(cs,HIGH); SPI1.endTransaction(); return v;
}
static void rdN(int cs, SPISettings s, uint8_t reg, uint8_t*b, uint8_t n){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg|0x80); for(uint8_t i=0;i<n;i++) b[i]=SPI1.transfer(0);
  digitalWrite(cs,HIGH); SPI1.endTransaction();
}
static void wr(int cs, SPISettings s, uint8_t reg, uint8_t v){
  SPI1.beginTransaction(s); digitalWrite(cs,LOW);
  SPI1.transfer(reg&0x7F); SPI1.transfer(v);
  digitalWrite(cs,HIGH); SPI1.endTransaction();
}
static int32_t sxt(uint32_t v, uint8_t bits){
  if(v & (1UL<<(bits-1))) v |= (0xFFFFFFFFUL<<bits);
  return (int32_t)v;
}

// ── ICM-42688-P ──────────────────────────────────────────────────────────────
bool icmBegin(){
  wr(PIN_CS_IMU,ICM_CFG,0x11,0x01); delay(5);            // soft reset
  if(rd(PIN_CS_IMU,ICM_CFG,0x75) != 0x47) return false;  // WHO_AM_I
  uint8_t itf = rd(PIN_CS_IMU,ICM_CFG,0x4C);
  wr(PIN_CS_IMU,ICM_CFG,0x4C,(itf&0xFC)|0x03);           // lock to SPI
  wr(PIN_CS_IMU,ICM_CFG,0x4F,0x06);                      // gyro  ±2000 dps, 1 kHz
  wr(PIN_CS_IMU,ICM_CFG,0x50,0x06);                      // accel ±16 g,     1 kHz
  wr(PIN_CS_IMU,ICM_CFG,0x4E,0x0F);                      // both low-noise
  delay(50);
  return true;
}
bool icmRead(float&ax,float&ay,float&az,float&gx,float&gy,float&gz,float&tc){
  uint8_t b[14]; rdN(PIN_CS_IMU,ICM_CFG,0x1D,b,14);
  int16_t t =(int16_t)((b[0]<<8)|b[1]);
  int16_t rx=(int16_t)((b[2]<<8)|b[3]),  ry=(int16_t)((b[4]<<8)|b[5]),  rz=(int16_t)((b[6]<<8)|b[7]);
  int16_t px=(int16_t)((b[8]<<8)|b[9]),  py=(int16_t)((b[10]<<8)|b[11]),pz=(int16_t)((b[12]<<8)|b[13]);
  // 0x8000 on every channel is the "no data" sentinel -- the part reset into standby and lost its
  // config. WHO_AM_I still answers 0x47 in that state, so an ID check will not catch it.
  if(rx==-32768 && ry==-32768 && rz==-32768) return false;
  if((rx|ry|rz|px|py|pz)==0) return false;               // MISO stuck low
  if(rx==-1 && ry==-1 && rz==-1) return false;           // MISO floating
  ax=rx/ACC_LSB_PER_G; ay=ry/ACC_LSB_PER_G; az=rz/ACC_LSB_PER_G;
  gx=px/GYR_LSB_PER_DPS; gy=py/GYR_LSB_PER_DPS; gz=pz/GYR_LSB_PER_DPS;
  tc=(t/132.48f)+25.0f;
  return true;
}

// ── DPS310 ───────────────────────────────────────────────────────────────────
struct { int32_t c0,c1,c00,c10,c01,c11,c20,c21,c30; } C;
constexpr float KP = 7864320.0f, KT = 524288.0f;         // 8x / 1x oversampling
bool dpsBegin(){
  wr(PIN_CS_BARO,DPS_CFG,0x0C,0x89); delay(50);          // soft reset
  if(rd(PIN_CS_BARO,DPS_CFG,0x0D) != 0x10) return false; // PROD_ID
  uint32_t t0=millis();
  while(!(rd(PIN_CS_BARO,DPS_CFG,0x08)&0xC0)){ if(millis()-t0>500) return false; delay(5); }
  // Infineon errata: re-trigger the OTP read or some lots return wrong temp coefficients.
  wr(PIN_CS_BARO,DPS_CFG,0x0E,0xA5); wr(PIN_CS_BARO,DPS_CFG,0x0F,0x96);
  wr(PIN_CS_BARO,DPS_CFG,0x62,0x02);
  wr(PIN_CS_BARO,DPS_CFG,0x0E,0x00); wr(PIN_CS_BARO,DPS_CFG,0x0F,0x00);
  uint8_t c[18]; rdN(PIN_CS_BARO,DPS_CFG,0x10,c,18);
  C.c0 =sxt(((uint32_t)c[0]<<4)|(c[1]>>4),12);
  C.c1 =sxt((((uint32_t)c[1]&0x0F)<<8)|c[2],12);
  C.c00=sxt(((uint32_t)c[3]<<12)|((uint32_t)c[4]<<4)|(c[5]>>4),20);
  C.c10=sxt((((uint32_t)c[5]&0x0F)<<16)|((uint32_t)c[6]<<8)|c[7],20);
  C.c01=sxt(((uint32_t)c[8]<<8)|c[9],16);   C.c11=sxt(((uint32_t)c[10]<<8)|c[11],16);
  C.c20=sxt(((uint32_t)c[12]<<8)|c[13],16); C.c21=sxt(((uint32_t)c[14]<<8)|c[15],16);
  C.c30=sxt(((uint32_t)c[16]<<8)|c[17],16);
  uint8_t srce = rd(PIN_CS_BARO,DPS_CFG,0x28)&0x80;
  wr(PIN_CS_BARO,DPS_CFG,0x06,0x53);                     // pressure 32 Hz, 8x
  wr(PIN_CS_BARO,DPS_CFG,0x07,srce|0x50);                // temp     32 Hz, 1x
  wr(PIN_CS_BARO,DPS_CFG,0x09,0x00);
  wr(PIN_CS_BARO,DPS_CFG,0x08,0x07);                     // continuous
  delay(50);
  return true;
}
void dpsRead(float&pa, float&tc){
  uint8_t b[6]; rdN(PIN_CS_BARO,DPS_CFG,0x00,b,6);
  float psc = sxt(((uint32_t)b[0]<<16)|((uint32_t)b[1]<<8)|b[2],24)/KP;
  float tsc = sxt(((uint32_t)b[3]<<16)|((uint32_t)b[4]<<8)|b[5],24)/KT;
  pa = C.c00 + psc*(C.c10 + psc*(C.c20 + psc*C.c30)) + tsc*C.c01 + tsc*psc*(C.c11 + psc*C.c21);
  tc = C.c0*0.5f + C.c1*tsc;
}

// ── GPS: NMEA -> local tangent plane in metres ───────────────────────────────
char     nmea[100]; uint8_t nlen=0;
uint32_t nmeaCount=0;
int      gpsFix=0, gpsSats=0;
float    gpsHdop=0, gpsMsl=0;
double   gpsLat=0, gpsLon=0;
bool     gpsHasFix=false, originSet=false;
double   lat0=0, lon0=0; float msl0=0;
double   gpsN=0, gpsE=0; float gpsU=0;
volatile uint32_t ppsCount=0;
void ppsISR(){ ppsCount++; }

// WGS84 radii at the datum latitude. Computed ONCE when the origin is captured -- they vary by
// under a part in 10^6 over any distance a rocket covers, so recomputing per fix buys nothing.
double Rm=6367449.0, Rn=6378137.0;
void computeRadii(double latRad){
  const double a=6378137.0, f=1.0/298.257223563, e2=f*(2.0-f);
  double s=sin(latRad), w=1.0-e2*s*s;
  Rn = a/sqrt(w);
  Rm = a*(1.0-e2)/(w*sqrt(w));
}
// ddmm.mmmm -> decimal degrees
static double nmeaDeg(const char*s, bool isLon){
  if(!*s) return 0;
  double v=atof(s);
  int deg=(int)(v/100.0);
  return deg + (v - deg*100.0)/60.0;
}
static const char* field(const char*s, int n){        // pointer to comma-field n (0-based)
  int f=0; while(*s && f<n){ if(*s==',') f++; s++; }
  return s;
}
static bool nmeaChecksumOk(const char*s){             // s starts at '$'
  const char* star = strchr(s,'*');
  if(!star || strlen(star)<3) return false;
  uint8_t sum=0;
  for(const char*p=s+1; p<star; p++) sum ^= (uint8_t)*p;
  return sum == (uint8_t)strtol(star+1,nullptr,16);
}
void setOrigin(){
  if(!gpsHasFix) return;
  lat0=gpsLat; lon0=gpsLon; msl0=gpsMsl;
  computeRadii(lat0*M_PI/180.0);
  originSet=true;
  gpsN=gpsE=0; gpsU=0;
}
void projectFix(){
  if(!originSet || !gpsHasFix) return;
  const double dLat=(gpsLat-lat0)*M_PI/180.0, dLon=(gpsLon-lon0)*M_PI/180.0;
  gpsN = Rm*dLat;
  gpsE = Rn*cos(lat0*M_PI/180.0)*dLon;
  gpsU = gpsMsl - msl0;
}
void parseGGA(const char*s){
  const char* q = field(s,6);  if(*q) gpsFix  = atoi(q);
  const char* n = field(s,7);  if(*n) gpsSats = atoi(n);
  const char* h = field(s,8);  if(*h) gpsHdop = atof(h);
  const char* la= field(s,2);  const char* ns = field(s,3);
  const char* lo= field(s,4);  const char* ew = field(s,5);
  const char* al= field(s,9);  if(*al) gpsMsl = atof(al);
  if(gpsFix>0 && *la && *lo){
    double lat=nmeaDeg(la,false), lon=nmeaDeg(lo,true);
    if(*ns=='S') lat=-lat;
    if(*ew=='W') lon=-lon;
    gpsLat=lat; gpsLon=lon; gpsHasFix=true;
    if(!originSet) setOrigin();            // first good fix becomes the datum
    projectFix();
  } else gpsHasFix=false;
}
void pumpGPS(){
  while(GPS_SERIAL.available()){
    char c=GPS_SERIAL.read();
    if(c=='$'){ nlen=0; nmea[nlen++]=c; continue; }
    if(nlen==0) continue;
    if(c=='\r'||c=='\n'){
      nmea[nlen]=0; nmeaCount++;
      if(nlen>9 && strstr(nmea,"GGA") && nmeaChecksumOk(nmea)) parseGGA(nmea);
      nlen=0; continue;
    }
    if(nlen<sizeof(nmea)-1) nmea[nlen++]=c;
  }
}

// ── LEDs ─────────────────────────────────────────────────────────────────────
// RGB is common-anode on 5V-CLEAN. "Off" must be high-Z, not driven HIGH: at 3.3 V the red die
// still sees 1.7 V, right at its knee, and glows faintly.
void LED(bool r,bool g,bool b){
  if(r){pinMode(PIN_RLED,OUTPUT);digitalWrite(PIN_RLED,LOW);}else pinMode(PIN_RLED,INPUT);
  if(g){pinMode(PIN_GLED,OUTPUT);digitalWrite(PIN_GLED,LOW);}else pinMode(PIN_GLED,INPUT);
  if(b){pinMode(PIN_BLED,OUTPUT);digitalWrite(PIN_BLED,LOW);}else pinMode(PIN_BLED,INPUT);
}
void beep(int f,int d){ tone(PIN_BUZZER,f); delay(d); noTone(PIN_BUZZER); }

float groundPa = 0;
void zeroBaro(){ float pa,tc; dpsRead(pa,tc); groundPa = pa; }
float altAGL(float pa){                                  // relative to the captured ground pressure
  if(groundPa <= 0) return 0;
  return 44330.0f*(1.0f - powf(pa/groundPa, 1.0f/5.255f));
}

void setup(){
  for(int i=0;i<4;i++){ pinMode(PIN_PYRO[i],OUTPUT); digitalWrite(PIN_PYRO[i],LOW); }
  pinMode(PIN_RLED,OUTPUT); pinMode(PIN_GLED,OUTPUT); pinMode(PIN_BLED,OUTPUT); LED(false,false,false);
  pinMode(PIN_MAIN_LED,OUTPUT); digitalWrite(PIN_MAIN_LED,LOW);
  pinMode(PIN_PYRO_LED,OUTPUT); digitalWrite(PIN_PYRO_LED,LOW);
  pinMode(PIN_BUZZER,OUTPUT);
  pinMode(PIN_BUTTON,INPUT);                             // external 10k pulldown
  pinMode(PIN_GPS_PPS,INPUT);
  pinMode(PIN_CS_IMU,OUTPUT);  digitalWrite(PIN_CS_IMU,HIGH);
  pinMode(PIN_CS_BARO,OUTPUT); digitalWrite(PIN_CS_BARO,HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsISR, RISING);

  Serial.begin(115200);
  uint32_t t0=millis(); while(!Serial && millis()-t0<3000){}

  SPI1.setMOSI(PIN_MOSI1); SPI1.setMISO(PIN_MISO1); SPI1.setSCK(PIN_SCK1); SPI1.begin();
  GPS_SERIAL.begin(GPS_BAUD);

  Serial.println(F("\n=== IMPULSE 2.2 SENSOR CHECK ==="));
  Serial.print(F("ICM-42688-P ... ")); haveICM=icmBegin(); Serial.println(haveICM?F("ok"):F("FAIL"));
  Serial.print(F("DPS310      ... ")); haveDPS=dpsBegin(); Serial.println(haveDPS?F("ok"):F("FAIL"));
  if(haveDPS){ delay(200); zeroBaro(); }
  Serial.println(F("GPS: first fix becomes the datum. Button = re-zero baro AND GPS origin.\n"));
  if(haveICM&&haveDPS){ beep(880,80); beep(1320,120); } else beep(220,400);
}

void loop(){
  pumpGPS();

  if(digitalRead(PIN_BUTTON)==HIGH){                     // re-zero both datums
    delay(30);
    if(digitalRead(PIN_BUTTON)==HIGH){
      while(digitalRead(PIN_BUTTON)==HIGH) pumpGPS();
      if(haveDPS) zeroBaro();
      originSet=false; setOrigin();
      Serial.println(F(">>> re-zeroed: baro ground pressure and GPS datum"));
      beep(1500,120);
    }
  }

  static uint32_t last=0;
  if(millis()-last < 250) return;
  last = millis();

  digitalWrite(PIN_MAIN_LED, haveICM && haveDPS);        // green  = SPI sensors alive
  digitalWrite(PIN_PYRO_LED, gpsHasFix);                 // yellow = GPS fix

  if(haveICM){
    float ax,ay,az,gx,gy,gz,tc;
    if(icmRead(ax,ay,az,gx,gy,gz,tc)){
      Serial.print(F("A ")); Serial.print(ax,3); Serial.print(' '); Serial.print(ay,3);
      Serial.print(' '); Serial.print(az,3);
      Serial.print(F("  |A| ")); Serial.print(sqrtf(ax*ax+ay*ay+az*az),3);
      Serial.print(F("   G ")); Serial.print(gx,2); Serial.print(' '); Serial.print(gy,2);
      Serial.print(' '); Serial.print(gz,2);
      Serial.print(F("   T ")); Serial.print(tc,1); Serial.println('C');
    } else {
      Serial.println(F("A  -- ICM returned its no-data sentinel (reset into standby); re-initialising"));
      haveICM = icmBegin();
    }
  }

  if(haveDPS){
    float pa,tc; dpsRead(pa,tc);
    Serial.print(F("BARO ")); Serial.print(pa/100.0f,3); Serial.print(F(" hPa  "));
    Serial.print(tc,2); Serial.print(F("C   AGL "));
    Serial.print(altAGL(pa),2); Serial.println(F(" m"));
  }

  Serial.print(F("GPS  fix ")); Serial.print(gpsFix);
  Serial.print(F(" sats ")); Serial.print(gpsSats);
  Serial.print(F(" hdop ")); Serial.print(gpsHdop,1);
  Serial.print(F("  nmea ")); Serial.print(nmeaCount);
  Serial.print(F(" pps ")); Serial.print(ppsCount);
  if(originSet && gpsHasFix){
    Serial.print(F("   N ")); Serial.print(gpsN,2);
    Serial.print(F("  E ")); Serial.print(gpsE,2);
    Serial.print(F("  U ")); Serial.print(gpsU,2);
    Serial.print(F(" m   dist ")); Serial.print(sqrt(gpsN*gpsN+gpsE*gpsE),2);
    Serial.println(F(" m"));
  } else {
    Serial.println(gpsHasFix ? F("   (no datum yet)") : F("   NO FIX"));
  }
  Serial.println();
}
