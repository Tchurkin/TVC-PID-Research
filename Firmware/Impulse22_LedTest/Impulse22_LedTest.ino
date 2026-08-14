/*
  Impulse22_LedTest.ino — dumb, unconditional drive of the two pack-status LEDs.
  ==============================================================================
  No thresholds, no sensors, no timing window. It just drives the pins so you can
  look at the board and put a meter on it at your leisure.

    LED2  green   pin 30  MAIN_LED   pin -> R62 330R -> LED anode, cathode to GND  (ACTIVE HIGH)
    LED3  yellow  pin 31  PYRO_LED   pin -> R63 330R -> LED anode, cathode to GND  (ACTIVE HIGH)

  Cycle, 1.5 s per step:  both off -> green only -> yellow only -> both on -> repeat.
  The RGB (LED1) mirrors the step so you can confirm the sketch is actually running
  even if the pack LEDs are dead: blue = off, green = green step, red = yellow step,
  white = both.

  WHAT THE RESULT MEANS
    Pack LEDs light                -> hardware is fine, the problem was in the flight firmware.
    Dark, but pin 30/31 pulses 3.3 V -> the pin drives; the LED is missing, backwards or dead.
    Dark, and the pin stays at 0 V   -> the Teensy pin or its socket joint is not connecting.

  Meter points, during the "green only" step:
    Teensy pin 30 ............. 3.3 V
    far end of R62 (LED anode)  ~1.3-2.0 V if a green LED is fitted and conducting
                                ~3.3 V if the LED is absent or open (no current, no drop)
*/

constexpr int PIN_MAIN_LED = 30;   // LED2 green
constexpr int PIN_PYRO_LED = 31;   // LED3 yellow
constexpr int RLED = 6, GLED = 7, BLED = 8;   // RGB LED1, common anode on 5V-CLEAN = active LOW

// Off must be high-Z, not OUTPUT HIGH: the anode sits at 5.0 V, so driving the pin to 3.3 V leaves
// 1.7 V across the die -- right at the red emitter's knee, which makes it glow faintly.
void LED(bool r, bool g, bool b) {
  if (r) { pinMode(RLED, OUTPUT); digitalWrite(RLED, LOW); } else pinMode(RLED, INPUT);
  if (g) { pinMode(GLED, OUTPUT); digitalWrite(GLED, LOW); } else pinMode(GLED, INPUT);
  if (b) { pinMode(BLED, OUTPUT); digitalWrite(BLED, LOW); } else pinMode(BLED, INPUT);
}

void setup() {
  pinMode(PIN_MAIN_LED, OUTPUT); digitalWrite(PIN_MAIN_LED, LOW);
  pinMode(PIN_PYRO_LED, OUTPUT); digitalWrite(PIN_PYRO_LED, LOW);
  pinMode(RLED, OUTPUT); pinMode(GLED, OUTPUT); pinMode(BLED, OUTPUT);
  LED(false, false, false);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }
  Serial.println(F("\nImpulse 2.2 pack-LED test — pins 30 (green LED2) and 31 (yellow LED3)"));
  Serial.println(F("Both are ACTIVE HIGH. 1.5 s per step. RGB mirrors the step.\n"));
}

void loop() {
  struct Step { bool main_; bool pyro; const char* name; bool r, g, b; };
  static const Step steps[] = {
    { false, false, "both OFF          ", false, false, true  },
    { true,  false, "GREEN  (pin 30) on", false, true,  false },
    { false, true,  "YELLOW (pin 31) on", true,  false, false },
    { true,  true,  "both ON           ", true,  true,  true  },
  };
  static int i = 0;

  const Step& s = steps[i];
  digitalWrite(PIN_MAIN_LED, s.main_);
  digitalWrite(PIN_PYRO_LED, s.pyro);
  LED(s.r, s.g, s.b);

  Serial.print(F("  ")); Serial.print(s.name);
  Serial.print(F("   pin30=")); Serial.print(s.main_ ? F("HIGH") : F("LOW "));
  Serial.print(F("  pin31=")); Serial.println(s.pyro ? F("HIGH") : F("LOW "));

  i = (i + 1) % 4;
  delay(1500);
}
