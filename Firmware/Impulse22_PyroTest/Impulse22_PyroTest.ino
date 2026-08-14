/*
  Impulse22_PyroTest.ino — single-channel pyro bench test for Impulse 2.2
  ======================================================================
  Fires ONE deliberately-chosen pyro channel, with continuity verification
  before and after. Defaults to channel 2.

  *** THIS IGNITES THINGS. READ BEFORE UPLOADING. ***

  Do the first run into a DUMMY LOAD, not a live e-match. A 10 ohm 1 W
  resistor, or an LED with a 470 ohm series resistor, across P2O pins 1-2
  proves the FET switches and the timing is right while igniting nothing.
  Only fit a live initiator once the dummy load has passed.

  SAFETY PROPERTIES BUILT IN
    - Pyro gates are driven LOW in the first statement of setup(), before
      anything else can run. They are never touched again except by fire().
    - ONLY the channel selected by PYRO_CHANNEL below can ever be driven.
      The other three are held low for the entire life of the program. This
      is deliberate: it makes it impossible to fire the motor igniter by
      miscounting button presses.
    - Nothing fires without a sustained physical button hold through a
      rising warning tone. Releasing early cancels. A serial byte cannot
      fire anything; there is no serial command path to fire().
    - The pulse is a bounded delay with no loop around it.
    - Refuses to arm if the 7.4 V pyro rail is absent.

  CONTROLS
    Short press ......... report status (rail voltage + continuity)
    Hold ARM_HOLD_MS .... fire, after a rising warning tone
    Release early ....... cancel

  CONTINUITY
    The sense circuit puts ~0.6 mA through the initiator (7.4 V rail, R41
    10k, red LED clamp) — far below any e-match no-fire current. Sense pin
    reads ~1.8 V with a load present, ~0 V open. After a successful fire
    the initiator goes open, so the sense reading dropping to ~0 V is your
    confirmation that it actually went.
*/

#include <Arduino.h>

// ── WHICH CHANNEL. Change this one number to test a different channel. ──────
constexpr int PYRO_CHANNEL = 2;          // 1..4

// ── Impulse 2.2 pin map (verified against Impulse_2.2.kicad_pcb) ────────────
constexpr int PIN_PYRO[4]       = {9, 10, 11, 12};    // gates, via 10k pulldowns R50-R53
constexpr int PIN_PYRO_SENSE[4] = {38, 39, 40, 41};   // continuity, via 10k R37/R41/R42/R43
constexpr int PIN_PYRO_V_SENSE  = 23;                 // 7.4V rail, R58 100k / R59 47k
constexpr int PIN_RLED = 6, PIN_GLED = 7, PIN_BLED = 8;
constexpr int PIN_BUZZER = 13, PIN_BUTTON = 14;

constexpr float PYRO_RATIO = 47.0f / (100.0f + 47.0f);
constexpr float ADC_VREF   = 3.30f;
constexpr int   ADC_BITS   = 12;

// ── Timing ──────────────────────────────────────────────────────────────────
// Fired as short bursts, not one long pulse. Q21 (AO3401A, SOT-23) passes the
// full firing current: ~4.9 A into a 1.5 ohm match, ~7 A into a 1 ohm one, so
// 1.2-2.4 W in a package whose transient thermal impedance is ~130 C/W at one
// second. A 1000 ms pulse puts the junction 150-300 C over ambient and destroys
// it — which is what killed the first one. At 20-30 ms it is ~40-80 C and fine,
// and an e-match at 5 A goes in single-digit milliseconds anyway.
//
// Between bursts the channel is off, so the continuity sense works again and we
// can stop the moment the initiator opens. The gap also lets Q21 cool.
constexpr uint32_t BURST_MS      = 30;     // on-time per attempt
constexpr uint32_t BURST_GAP_MS  = 150;    // off-time between attempts
constexpr int      MAX_BURSTS    = 6;      // give up after this many
constexpr uint32_t SETTLE_MS     = 15;     // let the sense node recover after turn-off
constexpr uint32_t ARM_HOLD_MS   = 3000;   // hold this long, through the tone, to fire
constexpr uint32_t DEBOUNCE_MS   = 30;
constexpr float    RAIL_MIN_V    = 6.0f;   // below this, refuse to arm
// Load present clamps at the red LED's ~1.7 V; open sits near 0 once senseV()
// discharges the node. 1.0 V splits them with margin on both sides.
constexpr float    CONT_THRESH_V = 1.0f;

const int CH = PYRO_CHANNEL - 1;           // zero-based

void LED(bool r, bool g, bool b) {         // LED1 is common-anode: LOW = on
  digitalWrite(PIN_RLED, !r); digitalWrite(PIN_GLED, !g); digitalWrite(PIN_BLED, !b);
}
void beep(int f, int d) { tone(PIN_BUZZER, f); delay(d); noTone(PIN_BUZZER); }

float railV() {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogRead(PIN_PYRO_V_SENSE);
  return (acc / 16.0f) / ((1 << ADC_BITS) - 1) * ADC_VREF / PYRO_RATIO;
}

// An OPEN channel leaves this node with no source and nothing pulling it down:
// the drain floats and the LED is a high impedance below its forward voltage.
// A floating pin does not read zero — the ADC's sample-and-hold retains charge
// from the previously converted channel, so an open channel came back ~1.1 V
// (a fraction of the 2.6 V rail reading taken just before) and sailed past the
// threshold. Every "initiator present" was a false positive.
//
// Fix: drive the node to ground, release it, and see whether it comes back. With
// a load present it recharges through R41 in well under a microsecond and clamps
// at the LED's ~1.7 V. Open, it stays where we left it.
float senseV(int ch) {
  const int p = PIN_PYRO_SENSE[ch];
  pinMode(p, OUTPUT); digitalWrite(p, LOW);   // discharge (worst case 0.8 mA through R41)
  delayMicroseconds(200);
  pinMode(p, INPUT);                          // release
  delayMicroseconds(500);                     // recharge window if a load is there
  analogRead(p);                              // discard — flushes the S&H
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogRead(p);
  return (acc / 16.0f) / ((1 << ADC_BITS) - 1) * ADC_VREF;
}

bool continuity(int ch) { return senseV(ch) > CONT_THRESH_V; }

void report() {
  float v = railV();
  Serial.print(F("\n  7.4V pyro rail : ")); Serial.print(v, 2); Serial.println(F(" V"));
  Serial.print(F("  CH")); Serial.print(PYRO_CHANNEL);
  Serial.print(F(" sense      : ")); Serial.print(senseV(CH), 2); Serial.print(F(" V"));
  // The sense circuit is fed FROM the 7.4 V rail through the initiator. With the
  // rail down there is no source, so the node floats and any reading is noise —
  // it must not be reported as continuity.
  if (v < RAIL_MIN_V) {
    Serial.println(F("  -> INDETERMINATE (rail is down; node is floating)"));
    Serial.println(F("  !! rail below 6 V — PI pack connected? SW2 on? Cannot fire."));
  } else {
    Serial.println(continuity(CH) ? F("  -> LOAD PRESENT") : F("  -> OPEN (no initiator)"));
  }
}

// The only place a pyro gate is ever driven high.
void fire() {
  bool before = continuity(CH);

  Serial.print(F("\n*** FIRING CH")); Serial.print(PYRO_CHANNEL);
  Serial.print(F(" — up to ")); Serial.print(MAX_BURSTS);
  Serial.print(F(" x ")); Serial.print(BURST_MS); Serial.println(F(" ms bursts ***"));

  LED(true, false, false);
  tone(PIN_BUZZER, 2600);

  int bursts = 0;
  bool railDied = false;
  for (bursts = 1; bursts <= MAX_BURSTS; bursts++) {
    digitalWrite(PIN_PYRO[CH], HIGH);
    delay(BURST_MS);                      // bounded, no inner loop
    digitalWrite(PIN_PYRO[CH], LOW);
    delay(SETTLE_MS);

    if (railV() < RAIL_MIN_V) { railDied = true; break; }   // fault — stop now
    if (!continuity(CH))      { break; }                    // it went — stop now
    if (bursts < MAX_BURSTS)  delay(BURST_GAP_MS);          // let Q21 cool
  }

  digitalWrite(PIN_PYRO[CH], LOW);        // belt and braces
  noTone(PIN_BUZZER);
  LED(false, false, false);

  Serial.print(F("  bursts delivered : ")); Serial.println(bursts);

  delay(250);
  bool after   = continuity(CH);
  float railAfter = railV();
  (void)railDied;

  Serial.print(F("  continuity before: ")); Serial.println(before ? F("present") : F("open"));
  Serial.print(F("  continuity after : ")); Serial.println(after  ? F("present") : F("open"));
  Serial.print(F("  rail after       : ")); Serial.print(railAfter, 2); Serial.println(F(" V"));

  // Continuity going open only proves the initiator fired if the rail SURVIVED.
  // If the rail collapsed, the sense node lost its source and reads open either
  // way — the two outcomes are indistinguishable, and saying "it fired" would be
  // a guess dressed up as a measurement.
  if (railAfter < RAIL_MIN_V) {
    Serial.println(F("  -> RAIL COLLAPSED DURING THE FIRE."));
    Serial.println(F("     Cannot tell whether the initiator went — with no rail the"));
    Serial.println(F("     sense node floats and reads open regardless."));
    Serial.println(F("     Meter 7.4V_RAW at PI vs 7.4V at Q21's drain:"));
    Serial.println(F("       RAW 8 V, drain 0 V -> Q21 failed"));
    Serial.println(F("       RAW 0 V            -> pack, protection trip, or SW2"));
    LED(true, false, false); beep(300, 700); LED(false, false, false);
  } else if (before && !after) {
    Serial.println(F("  -> WENT OPEN with the rail intact. The initiator fired."));
    LED(false, true, false); beep(1200, 100); beep(1600, 200); LED(false, false, false);
  } else if (before && after) {
    Serial.println(F("  -> STILL CONTINUOUS. Did not fire, or this is a dummy load"));
    Serial.println(F("     (a resistor stays continuous — that is a PASS for the FET)."));
    beep(700, 400);
  } else {
    Serial.println(F("  -> was already open before firing; nothing was connected."));
    beep(400, 500);
  }
}

void setup() {
  // Gates low first, before anything else in the program runs.
  for (int i = 0; i < 4; i++) { pinMode(PIN_PYRO[i], OUTPUT); digitalWrite(PIN_PYRO[i], LOW); }

  pinMode(PIN_RLED, OUTPUT); pinMode(PIN_GLED, OUTPUT); pinMode(PIN_BLED, OUTPUT);
  LED(false, false, false);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);            // external 10k pulldown R23
  analogReadResolution(ADC_BITS);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }

  Serial.println(F("\n╔════════════════════════════════════════════╗"));
  Serial.print  (F("║   IMPULSE 2.2 PYRO TEST — CHANNEL "));
  Serial.print(PYRO_CHANNEL);
  Serial.println(F("        ║"));
  Serial.println(F("╚════════════════════════════════════════════╝"));
  Serial.println(F("Only this channel can be driven. The other three are held low."));
  Serial.println(F("Short press = status.   Hold 3 s through the tone = FIRE."));
  report();
  LED(false, false, true);               // blue = idle
}

void loop() {
  if (digitalRead(PIN_BUTTON) != HIGH) return;
  delay(DEBOUNCE_MS);
  if (digitalRead(PIN_BUTTON) != HIGH) return;

  uint32_t t0 = millis();
  bool warned = false;

  while (digitalRead(PIN_BUTTON) == HIGH) {
    uint32_t held = millis() - t0;

    if (held > 400) {                    // past a tap: start the warning
      if (!warned) {
        warned = true;
        Serial.print(F("\nHold to fire CH")); Serial.print(PYRO_CHANNEL);
        Serial.println(F("... release to cancel"));
      }
      // rising tone + fast red flash tracks progress toward the fire point
      int f = 800 + (int)(1400.0f * held / ARM_HOLD_MS);
      tone(PIN_BUZZER, f);
      LED((held / 100) % 2, false, false);
    }

    if (held >= ARM_HOLD_MS) {
      noTone(PIN_BUZZER);
      if (railV() < RAIL_MIN_V) {
        Serial.println(F("\nABORT: pyro rail below 6 V. Check PI pack and SW2."));
        beep(300, 600);
      } else {
        fire();
      }
      while (digitalRead(PIN_BUTTON) == HIGH) delay(10);   // wait for release
      LED(false, false, true);
      return;
    }
    delay(10);
  }

  // released before the hold completed
  noTone(PIN_BUZZER);
  if (warned) { Serial.println(F("cancelled")); beep(500, 150); }
  else        { report(); beep(1000, 80); }
  LED(false, false, true);
}
