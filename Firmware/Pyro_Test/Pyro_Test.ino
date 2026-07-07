/*
  Pyro_Test.ino — BUTTON-ONLY bench pyro test (no serial monitor needed)
  ======================================================================
  TAP the button to cycle which channel is selected; HOLD to fire it.
    TAP  (< 0.4 s)  -> cycle selection:  NONE -> LEGS -> CHUTE -> IGNITER -> NONE ...
    HOLD            -> FIRE the selected channel  (legs/chute 2.5 s, IGNITER 4 s)
    release during the rising warning tone = CANCEL.
  After a fire, selection returns to NONE (tap to select again).

  Channels (match Ascent_TVC.ino):  P1 = 9 LEGS ,  P4 = 12 CHUTE ,  P3 = 11 ASCENT IGNITER.

  LED:   BLUE = none/idle · GREEN = LEGS · CYAN = CHUTE · RED = IGNITER selected
         fast RED flash + rising tone = holding to fire · GREEN double-blink = fired.
  BEEPS on select:  1 = LEGS ,  2 = CHUTE ,  3 = IGNITER.

  *** SAFETY ***
   - Legs (P1) / chute (P4) are melt-wire releases: safe to fire, re-set afterward.
   - IGNITER (P3) *** LIGHTS THE MOTOR ***. Test it with the igniter DISCONNECTED or into
     a dummy load (flashbulb / LED+resistor) unless doing a static fire. It needs a longer
     4 s hold on purpose. Check igniter/melt-wire CONTINUITY with a multimeter first.
   - A single tap only SELECTS; nothing fires without a sustained hold past the warning tone.
*/
#include <Arduino.h>

constexpr int BUTTON=14, BUZZER=13, RLED=6, GLED=7, BLED=8;
constexpr int P1=9, P3=11, P4=12;                    // legs, igniter, chute
constexpr unsigned long PULSE_MS       = 1000;       // fire pulse (matches the flight firmware)
constexpr unsigned long TAP_MS         = 400;        // press shorter than this = a TAP (cycle)
constexpr unsigned long FIRE_HOLD_MS   = 2500;       // hold to fire legs / chute
constexpr unsigned long IGNITER_HOLD_MS= 4000;       // hold to fire the igniter (longer on purpose)
constexpr unsigned long DEBOUNCE_MS    = 25;

// selection: 0=none, 1=LEGS(P1), 2=CHUTE(P4), 3=IGNITER(P3)
int sel = 0;
int selPin(int s){ return s==1?P1 : s==2?P4 : s==3?P3 : -1; }
unsigned long selHold(int s){ return s==3?IGNITER_HOLD_MS:FIRE_HOLD_MS; }

void LED(bool r,bool g,bool b){ digitalWrite(RLED,!r); digitalWrite(GLED,!g); digitalWrite(BLED,!b); }
void beep(int f,int d){ tone(BUZZER,f); delay(d); noTone(BUZZER); }
void setLED(int s){ if(s==1)LED(0,1,0); else if(s==2)LED(0,1,1); else if(s==3)LED(1,0,0); else LED(0,0,1); }
void announce(int s){                                // LED + beep count on (re)select
  setLED(s);
  if(s==0){ beep(500,150); return; }
  for(int i=0;i<s;i++){ beep(s==3?1900:1300,90); delay(110); }
}
void fire(int pin){
  LED(1,0,0);
  digitalWrite(pin,HIGH); tone(BUZZER,2600);         // channel ON + solid tone
  delay(PULSE_MS);
  digitalWrite(pin,LOW); noTone(BUZZER);             // channel OFF
  for(int i=0;i<2;i++){ LED(0,1,0); beep(1600,120); LED(0,0,0); delay(120); }   // "fired" confirmation
}

void setup(){
  pinMode(BUTTON,INPUT); pinMode(BUZZER,OUTPUT);
  pinMode(RLED,OUTPUT); pinMode(GLED,OUTPUT); pinMode(BLED,OUTPUT);
  pinMode(P1,OUTPUT); pinMode(P3,OUTPUT); pinMode(P4,OUTPUT);
  digitalWrite(P1,LOW); digitalWrite(P3,LOW); digitalWrite(P4,LOW);   // all channels OFF
  setLED(0); beep(800,120);                          // ready
}

void loop(){
  // ---- debounced button edge detection ----
  static bool pressed=false, lastRaw=false; static unsigned long lastChange=0;
  bool raw = (digitalRead(BUTTON)==HIGH);
  if(raw!=lastRaw){ lastRaw=raw; lastChange=millis(); }
  bool db = (millis()-lastChange>DEBOUNCE_MS) ? raw : pressed;

  static unsigned long pressStart=0; static bool fired=false;

  if(db && !pressed){ pressStart=millis(); fired=false; }          // press
  else if(db && pressed){                                          // held
    unsigned long held = millis()-pressStart;
    if(sel!=0 && !fired){
      unsigned long fh = selHold(sel);
      if(held>=TAP_MS){                                            // past tap window -> arming to fire
        tone(BUZZER, constrain(400 + (int)((held-TAP_MS)*1600/(fh-TAP_MS)), 400, 2000));  // rising tone
        LED(((held/90)%2), 0, 0);                                  // fast red flash = FIRE IMMINENT
      }
      if(held>=fh){ noTone(BUZZER); fire(selPin(sel)); fired=true; sel=0; announce(0); }
    }
  }
  else if(!db && pressed){                                         // release
    unsigned long held = millis()-pressStart;
    noTone(BUZZER);
    if(!fired){
      if(held<TAP_MS){ sel=(sel+1)%4; announce(sel); }             // TAP -> cycle selection
      else setLED(sel);                                            // released during warning -> CANCEL
    }
  }
  pressed = db;
}
