/*
  StaticFire.ino — motor thrust-curve logger for the F15 characterization static fire.
  ============================================================================================
  Reads a 5 kg bending-beam load cell via HX711, auto-detects the burn, records (time, thrust)
  at max rate into RAM, then prints: peak / average / burn time / TOTAL IMPULSE, the full CSV,
  and a downsampled table you paste straight into the SIL's F15_T/F15_F arrays (sim_main.cpp +
  Sysiphus_Landing.ino) so the landing predictions run on YOUR measured motor.

  WHY THIS MATTERS: characterizing the motor (its curve AND its sharp tail-off) is the #1 lever
  for the no-throttle landing -- it takes leg-10-survivable from ~80% (lot variation) to ~88%.

  HARDWARE: HX711 DOUT/SCK to the pins below; load cell to the HX711 (E+/E-/A+/A-). To get 80 SPS
  (vs 10), the HX711 module's RATE pin (pin 15) must be tied HIGH -- many cheap boards hardwire it
  to GND and need a trace cut / pad bridge. At 10 SPS you still get total impulse fine but only ~34
  points over the 3.4 s burn (coarse tail-off); 80 SPS (~272 pts) resolves the tail-off cliff.
  LIBRARY: install "HX711 Arduino Library" by Bogdan Necula (Library Manager).

  USAGE (Serial @ 115200):
    - Boot: it tares (zero) automatically -- make sure NO thrust/weight change during boot.
    - Type 'c' to calibrate: follow the prompt (place a KNOWN weight, enter its grams).
    - Type 't' to re-tare.  Type 'f' to arm capture (or it auto-arms and auto-triggers on thrust).
    - FIRE: it records automatically when thrust exceeds START_N, stops after thrust < STOP_N for
      STOP_MS, then dumps everything. Copy the SIL block into the F15 tables.
  SAFETY: this is the LOGGER, not an igniter. Ignite the motor by your own remote/electrical
  method, from a safe standoff, with your Designated Supervisor present. Shield the cell from the plume.
*/
#include <HX711.h>

// ---- config (SET THESE) ------------------------------------------------------
constexpr int   HX_DOUT = 2, HX_SCK = 3;        // <<< your wiring
float CAL_FACTOR = 420.0f;                       // <<< set from 'c' calibration (raw counts per gram). Placeholder.
constexpr float START_N = 2.0f;                  // N: thrust above this arms/starts recording
constexpr float STOP_N  = 1.0f;                  // N: thrust below this (for STOP_MS) ends the burn
constexpr uint32_t STOP_MS = 400;                // ms of sub-STOP_N before we call the burn over
constexpr int   MAXN = 1500;                     // sample buffer (1500 @ 80 SPS ~= 18 s, plenty)
constexpr int   SIL_PTS = 23;                    // points in the paste-ready SIL table

HX711 scale;
uint32_t t_ms[MAXN]; float f_N[MAXN]; int n = 0;
enum { IDLE, RECORDING } state = IDLE;
uint32_t belowSince = 0, t0 = 0;

float readN(){ return scale.get_units(1) * 9.80665f / 1000.0f; }   // grams-force -> N (1 sample = max rate)

void calibrate(){
  Serial.println(F("CAL: remove all load, then send any char..."));
  while(!Serial.available()){} while(Serial.available()) Serial.read();
  scale.tare(20);
  Serial.println(F("Place a KNOWN weight on the stand (in the THRUST direction), type its mass in GRAMS + Enter:"));
  while(!Serial.available()){}
  float grams = Serial.parseFloat(); while(Serial.available()) Serial.read();
  long raw = scale.get_value(20);                 // averaged raw counts for the known load
  if(grams>1){ CAL_FACTOR = raw / grams; scale.set_scale(CAL_FACTOR);
    Serial.print(F("CAL_FACTOR = ")); Serial.print(CAL_FACTOR,3);
    Serial.println(F("  <-- put this in the config #define so you don't recal each boot")); }
  else Serial.println(F("bad mass, ignored"));
  scale.tare(20); Serial.println(F("tared. ready."));
}

void dump(){
  if(n < 3){ Serial.println(F("(no burn captured)")); state=IDLE; n=0; return; }
  // peak, total impulse (trapezoidal), burn time, average
  float peak=0, imp=0; for(int i=0;i<n;i++) if(f_N[i]>peak) peak=f_N[i];
  for(int i=1;i<n;i++){ float dt=(t_ms[i]-t_ms[i-1])/1000.0f; imp += 0.5f*(f_N[i]+f_N[i-1])*dt; }
  float burn = (t_ms[n-1]-t_ms[0])/1000.0f, avg = burn>0? imp/burn : 0;
  Serial.println(F("\n================ STATIC FIRE RESULT ================"));
  Serial.print(F("peak    : ")); Serial.print(peak,2);  Serial.println(F(" N"));
  Serial.print(F("average : ")); Serial.print(avg,2);   Serial.println(F(" N"));
  Serial.print(F("burn    : ")); Serial.print(burn,3);  Serial.println(F(" s"));
  Serial.print(F("IMPULSE : ")); Serial.print(imp,2);   Serial.println(F(" N.s  (F15 datasheet ~49.6)"));
  Serial.print(F("samples : ")); Serial.print(n); Serial.print(F("  (rate ~")); Serial.print(n/max(burn,0.001f),0); Serial.println(F(" Hz)"));
  Serial.println(F("\n--- full curve (time_ms,thrust_N) ---"));
  for(int i=0;i<n;i++){ Serial.print(t_ms[i]); Serial.print(','); Serial.println(f_N[i],3); }
  // downsampled paste-ready SIL table (time normalized to seconds from ignition)
  Serial.println(F("\n--- PASTE into the SIL (sim_main.cpp + Sysiphus_Landing.ino F15_T/F15_F) ---"));
  Serial.print(F("F15_T[] = {"));
  for(int k=0;k<SIL_PTS;k++){ int i=(int)((long)k*(n-1)/(SIL_PTS-1)); Serial.print((t_ms[i]-t_ms[0])/1000.0f,3); Serial.print(k<SIL_PTS-1?",":"};\n"); }
  Serial.print(F("F15_F[] = {"));
  for(int k=0;k<SIL_PTS;k++){ int i=(int)((long)k*(n-1)/(SIL_PTS-1)); Serial.print(f_N[i],2); Serial.print(k<SIL_PTS-1?",":"};\n"); }
  Serial.println(F("(remember to set F15_N = 23 and re-run montecarlo.py to update the landing numbers)"));
  Serial.println(F("====================================================\n"));
  state=IDLE; n=0;
}

void setup(){
  Serial.begin(115200); while(!Serial && millis()<2000){}
  scale.begin(HX_DOUT, HX_SCK); scale.set_scale(CAL_FACTOR);
  Serial.println(F("Static-fire logger. taring (hold still)...")); delay(500); scale.tare(20);
  Serial.println(F("READY. 'c'=calibrate  't'=tare  'f'=arm.  Auto-triggers on thrust > START_N."));
}

void loop(){
  if(Serial.available()){ char c=Serial.read();
    if(c=='c') calibrate();
    else if(c=='t'){ scale.tare(20); Serial.println(F("tared")); }
    else if(c=='f'){ Serial.println(F("armed - fire when ready")); } }

  float N = readN();
  if(state==IDLE){
    static uint32_t lastPrint=0;
    if(millis()-lastPrint>250){ lastPrint=millis(); Serial.print(F("idle  thrust=")); Serial.print(N,2); Serial.println(F(" N")); }
    if(N > START_N){ state=RECORDING; n=0; t0=millis(); Serial.println(F(">>> BURN DETECTED - recording")); }
  }
  if(state==RECORDING){
    if(n<MAXN){ t_ms[n]=millis()-t0; f_N[n]=N; n++; }
    if(N < STOP_N){ if(belowSince==0) belowSince=millis(); else if(millis()-belowSince>STOP_MS){ belowSince=0; dump(); } }
    else belowSince=0;
    if(n>=MAXN){ Serial.println(F("(buffer full)")); dump(); }
  }
}
