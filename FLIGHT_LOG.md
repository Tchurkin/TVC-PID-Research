# FLIGHT_LOG.md — Flight-by-flight record

> **Purpose.** One row per flight attempt, with the airframe it flew, the firmware it flew, what
> happened, and a link to the raw data. Companion machine-readable copy: [FLIGHT_LOG.csv](FLIGHT_LOG.csv).
>
> **Newest at the TOP.** Append-only, like `DESIGN_LOG.md`.
>
> **Provenance rule.** Every number in the "measured" columns is computed FROM THE LOG FILE, not from
> memory or from prose. Anything not supported by data is marked `[CONFIRM]` — that means *unfilled*,
> not *none*. Do not delete a `[CONFIRM]`; fill it or explain why it cannot be filled.
>
> **This is a lab record, not report prose.** Per `AI_LOG.md`, any of this that reaches the Research
> Report must be re-written by Braxton in his own words. The tables here are data.

---

## Airframe reference (the "Ascent_TVC" vehicle)

MEASURED 2026-07-06, cited in `Firmware/Ascent_TVC/Ascent_TVC.ino`:

| Property | Symbol | Value | How obtained |
|---|---|---|---|
| All-up mass at liftoff | m | 0.818 kg | measured 2026-07-06 |
| Pitch/yaw moment of inertia | Iyy | 0.0078 kg·m² | measured 2026-07-06 |
| Roll moment of inertia | Izz | ~4.5e-4 kg·m² | **estimated** (slender-body 0.5·m·r², r≈33 mm) — `ascent_sim.cpp:58`. NOT measured. |
| CG → TVC pivot (moment arm) | L | 0.14 m | measured 2026-07-06 |
| Control effectiveness | keff = T·L/Iyy | ≈257 rad/s² per rad | derived from the three above + T=14.34 N |
| Gimbal linkage ratio | — | 5:1 both axes | servo deg per TVC deg |
| TVC command limit | MAX_TILT | ±5.0° | firmware clamp |
| Motor | — | Estes F15 | avg thrust 14.34 N, burn 3.45 s |

⚠ **The table above describes the PRE-ASC036 airframe, which was DESTROYED.** ASC036 (2026-07-20) wrecked
the vehicle and it was rebuilt — shorter, and heavier (927 g vs 818 g, **+13%**). The rebuild is also what
flipped the TVC linkage sense found on the bench 2026-08-03. Every row carries its own airframe columns
rather than pointing at this table; re-measure and record per flight.

### MEASURED AIRFRAME — post-ASC036 rebuild, 2026-08-03 (this is the vehicle that flies)

**Datum convention: the TVC rotation point.** CG position and the control moment arm L are therefore the
same number by construction. Use this datum for every future measurement so the two can never disagree.

| Property | Symbol | Measured | Previous airframe | note |
|---|---|---|---|---|
| All-up mass | m | **0.927 kg** | 0.818 kg | +13%. `[CONFIRM]` motor loaded when weighed |
| Pitch/yaw inertia | Iyy | **0.0176 kg·m²** | 0.0078 | bifilar, below; ×2.26 |
| CG fwd of TVC pivot = moment arm | L | **0.16 m** | 0.14 m | ×1.14 |
| Linkage ratio | `SERVO_*_MULT` | **4.5:1** both axes | 5.0 assumed | corrected 2026-08-04; commanded = physical, units 1:1 |
| Gimbal mechanical travel | — | **[5.00°, 5.25°)** | — | MEASURED 2026-08-04: 5.00 silent all 4 dirs, 5.25 buzzed |
| Servo sign | — | **−1 / −1** | +1 / +1 | bench pitch-the-nose; rebuild flipped the linkage |
| Control effectiveness | keff = T·L/Iyy | **130 rad/s² per rad** | 257 | **halved** |
| Ignition lag | — | **462 ± 58 ms** | 200 assumed | from 3 flight logs, ±50–100 ms |

Config: **927 g, legs folded, ascent configuration.**

**Linkage note.** The 1:5.5 measured on 2026-08-03 was a mis-measurement; the true ratio is **1:4.5**
(corrected 2026-08-04). `SERVO_*_MULT` now equals the physical linkage, so **commanded TVC degrees are true
gimbal degrees — no log correction is needed.** The earlier bench buzzing is explained: commanding 5.5
against a true 4.5 linkage drove 6.11° of gimbal and 5.0 drove 5.56°, both past the ±5.0° hard stop.

### Gains RE-POLED onto the measured plant, 2026-08-04

`P_GAIN 0.249 → 0.491`, `D_GAIN 0.062 → 0.123`, `MAX_TILT 5.0 → 4.5`.

The old gains were pole-placed for the destroyed airframe's keff = 257. On the rebuilt vehicle (keff = 130)
they delivered ωₙ = 5.70 / ζ = 0.71 — sluggish and under-damped. Scaling both by 257/130 = 1.97 restores the
*original intended* design point, ωₙ = 8.0 / ζ = 1.00, and preserves Kd/Kp at 0.25.

**This is the safer direction, not the more aggressive one.** Steady lean = misalignment/P_GAIN, so doubling
P halves the lean (4.02° → 2.04° per degree of misalignment) and hands that authority back — lean eating the
gimbal is exactly what aborted ASC007.

**`MAX_TILT` stayed at 5.0 — after the travel was MEASURED rather than assumed.** It was briefly dropped to
4.5 for stop clearance while the gimbal's true travel was unknown. `Bench_FindLimit.ino` (0.25° staircase,
constant-command holds at flight loop rate) then measured it: **5.00° silent on all four of X+/X−/Y+/Y−,
5.25° buzzed**, so the limit lies in [5.00°, 5.25°) and 5.0 is validated by direct test.

The validation is exact, not approximate: the `(int)` truncation in `writeServos()` is **asymmetric** —
`MAX_TILT 5.0` writes **112** (+dir, 4.89° gimbal) and **67** (−dir, 5.11° gimbal), not mirror images.
`Bench_FindLimit` uses the identical expression, so its 5.00° level commanded exactly those two integers.
There is no untested margin hiding between the bench test and the flight command.

| | old (P .249, MT 5.0) | **flying (P .491, MT 5.0)** |
|---|---|---|
| Monte Carlo PASS (N=150) | 80.8% | **98.7%** |
| MARGINAL | 18.3% | **0.7%** |
| ABORT | 0.8% | 0.7% |
| boost tilt p50 / p90 | 6.80° / 12.69° | **3.20° / 6.20°** |
| end lean p50 | — | 1.13° |
| misalignment tolerance | MARGINAL from 3° | **PASS through 4.5°** |

Robustness verified: actuator lag to **0.30 s** (10× the assumed `SERVO_TAU`), loaded slew down to **70°/s**,
gyro noise to **8 dps** (53× nominal). Above ~0.25 s lag or below ~60°/s slew both tunings degrade together.

**keff halved rather than dropping 2.26×, because L rose 14% and partly offsets Iyy.** With the flown gains
(P=0.249, D=0.062, unchanged) the closed loop moves ωₙ 8.01 → **5.70 rad/s** and ζ 1.00 → **0.71** — slower,
mildly overshooting, still comfortably damped. Gains deliberately not re-poled before the flight; if they
ever are, scale P and D **by the same factor** (see `DESIGN_LOG.md` and the Kd-decoupling result).

**Monte Carlo on the measured airframe** (N=150; Iyy ±10%, L ±10%, m=0.927, plus the measured 262 ms excess
ignition lag): **80.0% PASS / 19.3% MARGINAL / 0.7% ABORT**; boost tilt p50 6.44°, p90 12.88°; end lean p50
1.40°. Compare the same rig on the *assumed* airframe: 89.3 / 10.3 / 0.3. **The real vehicle has materially
less margin than the sim previously implied** — flyable, but tip-off and gusts bite harder. Predicted apogee
also drops ~85 m → ~58 m on the heavier vehicle; chute still deploys during descent in every case checked.

Three runs, parallel strings, m = 0.927 kg:

| run | b (half-separation) | L | 30 osc | T | Iyy | ω_torsion/ω_swing |
|---|---|---|---|---|---|---|
| A | 0.13 m | 24.5″ | 46.6 s | 1.5533 s | 0.01509 | **0.94 — on resonance** |
| B | 0.10 m | 24.5″ | 53.2 s | 1.7733 s | 0.01164 | 0.73 — marginal |
| C | 0.20 m | 30.5″ | 36.5 s | 1.2167 s | **0.01761** | 1.45 — clear |

**Best estimate: Iyy ≈ 0.0176 kg·m², = 2.26× the 0.0078 previously assumed.**

The three runs spread 51%, which by the usual rule means the measurement is bad — but the spread is
*explained*, not random. The bifilar torsional mode and the simple-pendulum swing mode are degenerate at
`b = 1/√(m/I)` = **13.8 cm/side**; note that this ratio is `b·√(m/I)` and so is **independent of string
length** — longer strings do not separate the modes. Run A sat 6% from that degeneracy and is unusable;
run B is close to it; **run C is the only run clear of it and is the one to trust.**

Not yet confirmed. Two-minute check: b = 0.28 m (56 cm apart), L = 30.5″ → predicted **26.1 s** for 30
oscillations. That b gives a mode ratio of 2.03, well clear. If it reads ~26 s, 0.0176 is confirmed.

**Consequence if confirmed:** keff = T·L/Iyy falls 257 → 114 s⁻². With the flown gains (P=0.249, D=0.062)
the closed loop goes ωₙ 8.00 → 5.32 rad/s and ζ 1.00 → 0.66 — slower, mildly overshooting, still damped.
SIL across the range: no aborts; worst case is MARGINAL 14.8° boost tilt in a tip-off + gust case against
a 45° threshold. **The steady lean is essentially unchanged (0.16° → 0.28°)** because lean =
misalignment/P_GAIN and does not depend on keff — so the trim-vs-authority mechanism that aborted ASC007 is
insensitive to this. It did not block the 2026-08-04 flight.

**The independent check is the flight itself.** The 500 Hz `CTL###.CSV` gives tilt, body rate and commanded
TVC angle; differentiating rate against commanded gimbal angle fits keff directly, and with thrust from
`MTR###.CSV` and the measured arm L that yields `Iyy = T·L/keff` with no string involved. That also settles
the linkage-ratio question. Prefer it over the pendulum.

**Where 0.0078 came from is now an open question.** It was measured once, on the destroyed airframe, and
`[CONFIRM]` by what method — if it was the same bifilar rig, it may have hit the same degeneracy.

---

## Summary

| Flight | Date | Type | Outcome | Apogee | Max boost tilt | TVC saturated | Data |
|---|---|---|---|---|---|---|---|
| ASC036 | 2026-07-20 | TVC ascent + chute | **PARTIAL** — best control, log died at burnout | ≥21.8 m (log ends) | 8.7° | 0.0% | [CSV](Rocket%20data/ASC036.CSV) |
| *(unnamed)* | 2026-07-20 | no-ignition run | **NO IGNITION** | — | — | — | [MTR000](Rocket%20data/MTR000.CSV) |
| *ASC037* | `[CONFIRM]` | **ground test, NO MOTOR** | n/a — never left the ground | 0.78 m | 7.6° (hand motion) | 56.5% | [CSV](Rocket%20data/ASC037.CSV) |
| ASC031 | 2026-07-12 | TVC ascent + chute | **SUCCESS** (boost controlled) | 45.7 m | 11.2° | 0.0% | [CSV](Rocket%20data/ASC031.CSV) |
| ASC007 | 2026-07-07 | TVC ascent + chute | **FAILURE** — self-aborted mid-boost | 12.8 m | 52.6° | 17.4% | [CSV](Rocket%20data/ASC007.CSV) |
| LOG001 | ≤2026-06-28 | TVC ascent (pre-Ascent_TVC fw) | **FAILURE** — diverged, gimbal pinned | 33.6 m | 38.7° | 88.6% | [CSV](Rocket%20data/LOG001.CSV) |
| RES005 | ≤2026-04-19 | passive response test, TVC off | `[CONFIRM]` | 46.2 m | n/a (TVC off) | n/a | [CSV](Rocket%20data/RES005.CSV) |

Dates are the **SD-card copy-off timestamp** (the flight computer has no RTC, so every CSV's internal
mtime is 2018). For ASC007 the copy date matches the `DESIGN_LOG.md` entry, and for ASC031 it matches
the linkage-flip date cited in the firmware header — so the copy date is a good proxy, but it is an
**upper bound**: a flight could have been flown earlier and copied later.

---

## ASC038 — 2026-08-04 — TVC ascent + chute — **FAILURE (uncontrolled), full data return**

- **Conditions:** wind 3.1 m/s. Card copied off 2026-08-04 16:29.
- **Outcome:** the vehicle **tumbled**. It reached only 26.8 m (ASC031, controlled, reached 45.7 m on the
  lighter airframe) and recovered under canopy. **The controller was blind for the entire burn and the 45°
  abort could not fire.** Files: [ASC038](Rocket%20data/ASC038.CSV), [CTL000](Rocket%20data/CTL000.CSV),
  [MTR001](Rocket%20data/MTR001.CSV).
- **ROOT CAUSE — SD write latency starved the control loop, which silently froze the attitude estimator.**
  The chain, every link measured from this flight's own logs:
  1. `logData()` opens, writes and closes `ASC###.CSV` every 50 ms. On this card each write cost a
     **mean 143 ms, max 179 ms**.
  2. So **29 of 170 control iterations took ≥100 ms** — only 17% of iterations, but **88% of the burn's
     elapsed time**. Median iteration was a healthy 3.49 ms; the mean was 27.7 ms.
  3. `sensors()` propagates attitude only `if(dt>0 && dt<0.1f)`. Every one of those 29 long iterations
     **skipped `quatPropagate()`**, so the quaternion stopped integrating almost immediately.
  4. The tilt estimate froze at **tiltX −0.10°, tiltY −2.43°** and stayed *bit-identical* for the rest of
     the flight while the real body rates reached **248 °/s**.
  5. With P·tilt frozen, **98% of the TVC command was the D term** — rate damping with no attitude
     reference. Pre-clamp command hit **30.25°** against a 5° limit; 14% of samples saturated.
  6. **The 45° emergency abort reads `gyro_x`/`gyro_y`.** They were frozen at ~2.4°, so the abort was
     blind and could not fire. This is the most serious finding.
- **True attitude (replay).** Integrating the firmware's *own logged raw rates* with the `dt<0.1` guard
  removed puts the vehicle past **45° at t = 2.21 s** and at **≥161°** by t=3.4 s. Roll rate is not in the
  CTL schema, so that is a **lower bound**, and the 143 ms sampling gaps make it approximate — but the
  direction is unambiguous and is corroborated by the low apogee.
- **This is not caused by the 2026-08-04 retune.** The capture's own measured overhead was
  **0.288 µs/iteration** — 0.008% of a 3.49 ms loop — and the stalls are SD writes, which predate every
  change made this week. The gains, signs, linkage and MAX_TILT all behaved as designed; they were simply
  fed a dead attitude.
- **Motor:** `ignition_lag_ms=920` — but treat with suspicion, onset detection runs in the same starved
  loop. Peak axial 29.16 m/s² (2.97 g), mean burn 17.70 m/s², burn ended ~3.74 s after the ignition command.
- **REQUIRED BEFORE THE NEXT FLIGHT** (in priority order):
  1. **Get SD writes out of the control loop.** Either hold the file open and flush periodically, or buffer
     ASC rows in RAM and dump at recovery exactly as `MTR###`/`CTL###` already do. This is the fix; the rest
     are defences.
  2. **Never let `sensors()` silently skip propagation.** Skipping is the worst option — it freezes the
     estimate with no indication. Clamp `dt` instead, and count the events so the log shows it happened.
  3. **Add an attitude-independent abort.** A sustained body-rate threshold would have fired here at ~2.2 s.
     Any abort that depends on the estimator dies with the estimator.
  4. **Widen `dt_us` in the CTL capture to `uint32`.** It saturated at 65535 and lost the real stall
     lengths; only the header's uint32 min/mean/max survived. My error — noted at the time as covering a
     ">65 ms stall" that "should be pathological". It was 17% of iterations.
- **What worked:** every piece of new instrumentation. The 500 Hz capture, the loop-period statistics, the
  raw-vs-filtered rate channels and the pre/post-clamp channels together produced a complete, unambiguous
  root cause from a single flight. The abort path, the burnout ramp and the recovery all executed; all three
  logs were written.

## ASC037 — date `[CONFIRM]` — **GROUND TEST, NO MOTOR** — added 2026-08-09

Found on the card during the retrospective flight-signature work; it had never been logged. **This is
not a flight and not a static fire.** Evidence, all computed from `Rocket data/ASC037.CSV`:

| quantity | ASC037 | a real flight (ASC036) | reading |
|---|---|---|---|
| acceleration **magnitude**, median | **8.89 m/s²** | 11.96 | 1 g — nothing is accelerating it |
| samples above 1.5 g | **1 of 134** (t = 0.78 s) | many | a single knock, not a burn |
| altitude max | **0.78 m** | 21.8 m | never left the ground |
| vertical velocity max | **0.74 m/s** | 13.7 m/s | ditto |
| body rates | up to 76 °/s, roll drifting to 73° | — | **someone was moving it by hand** |
| TVC saturated | 56.5% of the "boost" window | 0.0% | controller winding up against no response |

So: the full arm → ignition-command → TVC → phase-advance sequence ran, on a vehicle with no motor,
being handled. 134 rows, 6.96 s, 19.2 Hz, Phase advancing 1 → 2.

`[CONFIRM]` **what this run was for** — a deliberate handling/response rehearsal, or a dry run that was
never written up? The data cannot distinguish, and only Braxton knows.

⚠ **Do not treat the peak `AccelZ` of 22.24 m/s² as thrust.** It is one sample. This log was briefly
mis-identified as a static fire on exactly that basis. The correct test is the acceleration
*magnitude*: a clamped **or** hand-held vehicle reads ~1 g no matter what the motor is doing, because a
static-fire stand reacts the thrust into the ground.

**It also produced a real result.** As a control for the flight-signature test it scores **7.6° RMS**,
above the 5.62° failure threshold — i.e. *hand motion alone trips the signature*. That is the
specificity limit of the diagnostic and it is now recorded in §7 of the paper.

## ASC036 — 2026-07-20 — TVC ascent + chute — PARTIAL

- **Outcome:** best-controlled boost to date, but **the log stops at burnout**. 68 rows, all `Phase=1`,
  last row t=+3.49 s into a 3.65 s powered window, vehicle at 21.8 m still climbing at 13.1 m/s. There
  are **no coast, apogee or descent rows**, and **no `MTR###.CSV` was written** (the motor-log dump only
  runs on the recovery path). Both facts point the same way: **the flight computer stopped logging
  while still airborne under thrust.**
- **This is the top open item before the next flight.** Candidate causes, none yet eliminated:
  brown-out under servo current, SD write failure, battery/connector disconnect under thrust or
  vibration, or a structural/mounting failure. `[CONFIRM]` — what did the vehicle look like on recovery?
  Was the chute out? Did it come down under canopy? (**The airframe was destroyed on this flight and
  rebuilt**, confirmed 2026-08-03 — which is also what flipped the TVC linkage sense.)
- **Leading hypothesis as of 2026-08-03: brownout under servo load.** On the bench that day, with a pack
  that turned out to be flat, the vehicle showed (a) a servo driving to its stop and stalling on reboot,
  and (b) the countdown halting part-way with the status LED *floating* — a floating LED means the pin is
  high-Z, i.e. the MCU has stopped driving it, which is a reset/brownout signature, not a hang (a hung
  MCU keeps its pins driven). A stalled servo draws roughly 1–1.5 A, which is exactly the sag that would
  do it, and ASC036's log stops during the burn when servo activity is highest.
  **This is NOT established for ASC036.** No voltage was recorded on that flight, the board has no
  battery sense, and it was a different session two weeks earlier. Treat it as the leading candidate.
- ⚠ **Causality is not yet settled, and it matters:** did a flat battery cause the servo to stall, or did
  a servo binding against a mechanical stop drain the battery and brown out the board? A bind draws stall
  current *regardless of battery health*, so "it was just a low battery" is the comfortable answer and the
  dangerous one. **Decisive test:** charge the pack fully, power up, and watch. If a servo still drives to
  a stop with a fresh pack, the fault is mechanical (most likely servo-horn indexing after the rebuild —
  if 90° sits near the linkage's limit, any startup transient pushes it in). Confirm with a current
  measurement at idle: a centred, free gimbal should draw tens of mA, not hundreds.
- **Now diagnosable if it recurs.** Firmware from 2026-08-03 latches `SRC_SRSR` at boot and prints
  `*** BROWNOUT: power was lost MID-FLIGHT ***` when a power-on reset coincides with an in-flight saved
  EEPROM phase — the two together are the discriminator, because on i.MX RT the low-voltage-detect
  brownout path resets through POR and so is indistinguishable from a normal power-up on its own.
  `reset_cause` and `sd_open_failures` are also written into the `CTL###.CSV` header.
- **Airframe:** m=0.818 kg, Iyy=0.0078 kg·m², L=0.14 m, gimbal 5:1, MAX_TILT ±5°, motor F15. `[CONFIRM]`
  whether the airframe was rebuilt/re-measured after ASC031.
- **Firmware (inferred, two independent measurements agree):** the post-2026-07-17 build.
  Evidence: (1) log schema includes `estZ(m),estVz(m/s)` → the passive Kalman commit `5c3c58e`
  (2026-07-17); (2) peak axial accel **27.39 m/s² = 2.79 g with no clipping** → the ±16 g FSR fix was
  live; (3) the TVC command carries a steady offset the pure-PD law does not explain (median residual
  `TVC − (P·tilt + D·rate)` = −0.23° X, −0.62° Y) → **the integral trim was active**.
- **Gains:** P=0.249, D=0.062, I=0.20.
- **Measured:** max boost tilt 8.7° (|X| 4.0°, |Y| 8.2°); roll 0 → 74.6° (uncontrolled, as expected —
  2-axis TVC has no roll authority); **0.0% of boost rows at the ±5° gimbal limit** — the controller
  never ran out of authority; log rate 19.2 Hz.
- **Bonus result:** the passive vertical Kalman tracked the barometer to **median |estZ − baro| = 0.14 m,
  max 0.62 m** over the boost. That is the landing firmware's altitude estimator validated on real
  ascent sensor data. Caveat from `Ascent_TVC.ino`: this is the **ascent regime only** — it does not
  validate the descent / near-ground / retrograde regime the hoverslam actually needs.
- **Files:** [ASC036.CSV](Rocket%20data/ASC036.CSV), [thrust fit](Rocket%20data/ASC036_thrust.png),
  [mass fit](Rocket%20data/ASC036_massfit.png)

## *(flight ID not preserved)* — 2026-07-20 — no-ignition run — NO IGNITION

- **Outcome:** the flight computer ran a complete arm → countdown → ignition-command → burn-window →
  coast → recovery sequence, but **the motor produced no thrust**. Axial specific force sits flat at
  ~8.8 m/s² (the vehicle's at-rest reading) for the entire 3.64 s window; **0 of 343 samples exceed the
  1.5 g thrust-onset threshold**; the header records `ignition_lag_ms=0` and `mean_burn_axial=0.00`.
- Pulled off the card one minute after ASC036 (16:51 vs 16:50), so it is from the same session. It is
  **not** ASC036's motor log — ASC036 never reached the recovery path that writes `MTR###.CSV`.
- `[CONFIRM]` — was this a deliberate dry/no-motor arming rehearsal, or a real ignition failure? The
  data cannot distinguish them. The matching `ASC###.CSV` was not copied off the card.
- **Files:** [MTR000.CSV](Rocket%20data/MTR000.CSV)

## ASC031 — 2026-07-12 — TVC ascent + chute — SUCCESS (boost controlled)

- **Outcome:** first controlled boost. Tilt bounded at 11.2°, apogee 45.7 m, chute recovery, **0.0% of
  boost rows at the gimbal limit** (peak command 3.23° X / 2.40° Y against a ±5° limit — roughly 35%
  authority margin). Roll ran away uncontrolled to 193° again; the boost tolerated it.
- **Airframe:** as the reference table. `[CONFIRM]` post-ASC007 rebuild state.
- **Firmware (inferred — and this CORRECTS an assumption written in the firmware):** ASC031 flew the
  **pre-2026-07-07-pm** build, *not* the current one. Two independent measurements:
  (1) peak axial accel **railed at exactly 19.620 m/s² = 2.000 g on 6 samples** → the IMU was still on
  the ±2 g full-scale range, i.e. the `MPU6050_tockn` → `Adafruit_MPU6050` swap was not in effect;
  (2) the TVC command is explained by pure PD to within noise (median residual +0.005° X, −0.018° Y,
  sd ≈0.1°) → **no integral trim**, which shipped in the same 2026-07-07-pm change.
- ⚠ **Consequence — read before flying.** `Ascent_TVC.ino:57-62` cites ASC031 as the flight that
  validated `SERVO_X_SIGN`/`SERVO_Y_SIGN = +1` against the flipped linkage. The same comment block warns
  that the correct sign depends on **both** the linkage **and** the IMU driver's gyro convention. Since
  ASC031 flew a *different driver*, it is **not** valid evidence for the current build's sign. **ASC036
  is** — it flew the current driver and held attitude with margin. This does not change the code; it
  changes what the code's own comment may claim. `[CONFIRM]` with Braxton, then correct the comment.
- **Gains:** P=0.249, D=0.062, I=0 (see above).
- **Measured:** apogee 45.69 m; boost max |tiltX| 11.1°, |tiltY| 7.7°, total 11.2°; boost lean grew to
  ~10° in X and stayed there (the signature of a trim with no integrator); roll 8.3 → 193.4°;
  125 rows over 6.31 s at 20.0 Hz; boost 71 rows, coast 54 rows.
- **Files:** [ASC031.CSV](Rocket%20data/ASC031.CSV)

## ASC007 — 2026-07-07 — TVC ascent + chute — FAILURE (self-aborted)

- **Outcome:** first flight of `Ascent_TVC`. Self-aborted and deployed the chute ~2.3 s into the boost
  at **52.6° total tilt, 12.8 m**, having tripped the 45° emergency threshold.
- **Root cause (from `DESIGN_LOG.md` 2026-07-07, after the roll-coupling hypothesis was falsified):**
  **trim vs authority.** An effective thrust-misalignment / CG-offset of ~4° drove a steady lean of
  misalignment/P_GAIN ≈ 16°, which ate the ±5° TVC authority; the vehicle then had nothing left for the
  disturbance. Roll was a *symptom*, not the cause.
- **Airframe:** as the reference table (measured the day before, 2026-07-06).
- **Firmware:** the original build. Confirmed by data: axial accel **railed at exactly 19.620 m/s² =
  2.000 g on 9 of 46 samples** (true axial was ~3.2 g — the log is clipped), and the TVC command is
  pure PD to within noise (median residual −0.005° X, −0.011° Y) → **no integral trim**.
- **Gains:** P=0.249, D=0.062, I=0.
- **Measured:** 46 rows over 2.31 s at 19.2 Hz; max |tiltX| 29.9°, |tiltY| 43.3°, total 52.6°;
  roll −8.0 → −180.4°; **17.4% of boost rows at the ±5° gimbal limit**; apogee 12.75 m.
- **Files:** [ASC007.CSV](Rocket%20data/ASC007.CSV), [analysis](Rocket%20data/ASC007_analysis.png)

## LOG001 — ≤2026-06-28 — TVC ascent (pre-`Ascent_TVC` firmware) — FAILURE (diverged)

- **Outcome:** diverged. Total tilt reached 38.7°, roll swung 297°, and the gimbal was **pinned at its
  ±5° limit for 88.6% of the log** — near-total loss of control authority for most of the flight.
- **Firmware:** predates `Ascent_TVC`. Log schema is `...,ServoX,ServoY` with **no `Phase` column**
  (the `Phase` column arrived in commit `0ddd51a`, 2026-07-07) — this is the older
  `Sysiphus_Landing`-lineage ascent code. Axial accel railed at 2.000 g.
- **Airframe:** `[CONFIRM]` — this predates the 2026-07-06 mass/inertia measurement, so the reference
  table does **not** apply. Mass, Iyy and L for this vehicle are unknown.
- **Gains:** `[CONFIRM]` (the 0.249/0.062 pole-placed pair postdates this flight).
- **Measured:** 70 rows over 3.56 s at 19.2 Hz; apogee 33.59 m; max |tiltX| 27.4°, |tiltY| 35.5°.
- **Caveat:** in this schema `ServoX`/`ServoY` record the **post-clamp** value, so 88.6% is the fraction
  of rows *at* the limit — the log cannot show how much command was being *thrown away*. That blind spot
  is exactly what the new `CTL###.CSV` pre-clamp channel fixes (see `DESIGN_LOG.md` 2026-08-02).
- **Files:** [LOG001.CSV](Rocket%20data/LOG001.CSV)

## RES005 — ≤2026-04-19 — passive response test, TVC disabled — `[CONFIRM]`

- **Purpose (inferred from schema):** a controller-response / latency experiment. The log carries
  `DelayMS`, `SlewDPS`, `Saturated` and `ComputeTime_us` columns, and `ServoX`/`ServoY` are **0.00 for
  every row** — the gimbal was commanded to neutral throughout, so this was a **passive** flight.
- **Outcome:** `[CONFIRM]`. Apogee 46.20 m. The logged tilt estimate winds up to 605°/556°, which is the
  naive body-rate integrator diverging under roll, not a physical attitude.
- **The one durable number here:** `ComputeTime_us` = **min 690, median 691, max 4160 µs**. This is the
  only on-vehicle loop-timing measurement that exists. ⚠ The firmware that produced it is **not in the
  repository**, so the exact definition of `ComputeTime_us` (whole-loop period vs. compute-only,
  excluding I²C waits) **cannot be verified**. Treat it as an order-of-magnitude anchor only — sub-ms to
  a few ms — not as a calibrated loop period. The `CTL###.CSV` `dt_us` channel added 2026-08-02
  supersedes it with a defined, in-firmware measurement.
- **Airframe / gains:** `[CONFIRM]`.
- **Files:** [RES005.CSV](Rocket%20data/RES005.CSV)

---

## Cross-flight analysis — why ASC007 aborted (offline estimator replay, 2026-08-02)

The project carries two conflicting accounts of ASC007: `DESIGN_LOG.md` (2026-07-07) **falsified**
roll-coupling and reframed the cause as *trim vs authority*; the `Ascent_TVC.ino` header (2026-07-13)
re-asserts that the naive estimator "inverted its own correction near ~180° roll and drove the
divergence". No DESIGN_LOG entry supersedes the falsification. The flight logs can partly settle it.

**Method.** ASC007 and ASC031 logged the filtered body rates and the integrated roll, so both the naive
integrator and the roll-aware quaternion can be re-run offline on the *same* rate history.
**Validated first:** re-integrating the naive law reproduces what the firmware actually logged to
**0.29° (ASC007) / 0.43° (ASC031) median**, so the replay is faithful. *The quaternion replay is not
ground truth either* — both are dead-reckoned from the same 20 Hz filtered rates with no attitude truth
reference. It shows what the two estimators **disagree about**, not what the vehicle actually did.

**Result 1 — the inversion is real.** On ASC007 the estimates separate as roll builds (at |roll|=159°:
naive 25.0° tilt, roll-aware 8.3°) and the **X channel disagrees in sign on 69% of rows above 120° roll**.
This explains the one thing DESIGN_LOG conceded it could not reproduce — "hold ~16° for 1.4 s *then*
diverge at 1.9 s". A constant trim gives a sharp hold-or-tumble threshold, not a delayed trigger; an
estimator that is correct at low roll and inverts past ~120° gives exactly that. First saturation was at
t=1.95 s, |roll|=148.9°; **0 of 8 saturated rows occur below 140° roll.**

**Result 2 — but inversion is not sufficient. ASC031 is the control experiment.** Same naive estimator,
rolled *further* (185.8° vs 172.6°), same inversion present (29% X sign disagreement) — **and it
survived with 0% saturation.** What differed was authority margin:

| Flight | implied thrust misalignment | roll excursion | TVC saturated | outcome |
|---|---|---|---|---|
| ASC007 | **4.28°** (lean × P_GAIN) | 172.6° | 17.4% | ABORT |
| ASC031 | **2.52°** (lean × P_GAIN) | 185.8° | 0.0% | survived, held a ~10° lean |
| ASC036 | **0.66°** (from the I-term) | 74.6° | 0.0% | clean |

**Synthesis.** Both mechanisms are real and they multiply. Inversion + no margin → abort (ASC007);
inversion + margin → survives with a lean (ASC031); good alignment + roll-aware estimator + low roll →
clean (ASC036). Neither existing account is complete, and the alignment improved monotonically across
all three flights (4.28° → 2.52° → 0.66°), which is mechanical work, not firmware.

**⚠ ASC007 may have aborted on a phantom.** The abort fires on `|GyroX|>45 or |GyroY|>45`. The naive
estimate peaked at **46.7° on Y** — barely over. The roll-aware replay of the same flight peaks at
**33.7°**. The vehicle may never have reached the abort threshold at all. `[CONFIRM]` — not provable
from this data, but it is the most likely reading and it is testable (below).

**ASC036 does NOT prove the quaternion fix.** It only reached **74.6°** roll — it never entered the
regime where the naive estimator inverts, so it did not exercise the fix. It also changed three things
at once: IMU driver + FSR, integral trim, and the quaternion estimator.

**How to actually settle it (one flight):** the quaternion fix is only testable at large roll. A flight
that reaches **>140° roll with good alignment** discriminates — if it holds attitude where ASC007
diverged, the estimator was the deciding factor. Do **not** deliberately induce roll to get there; roll
has been an uncommanded 75–195° on every flight so far, so the case will arrive on its own. The new
`CTL###.CSV` capture makes that flight far more conclusive: pre- vs post-clamp shows exactly how much
authority was being demanded versus delivered, at 500 Hz instead of 20.

---

## Cross-flight analysis — ignition lag is ~2.3× the assumed value (2026-08-03)

**Method.** `logData()` first fires ~50 ms after `launchTime`, and `launchTime` is set immediately after
`triggerPyro(P3)` — so row 0 of every `ASC###.CSV` is essentially the ignition **command**. Lag = time of
the first row showing thrust, minus row 0. Resolution is the 20 Hz log rate, and the ~50 ms first-row
offset is **not** removed, so these are **upper bounds, good to about ±50–100 ms**.

| Flight | measured lag | peak axial | note |
|---|---|---|---|
| ASC007 | 406 ms | 19.62 (clipped at ±2 g) | log ends during burn |
| ASC031 | 458 ms | 19.62 (clipped at ±2 g) | |
| ASC036 | 522 ms | 27.39 m/s² (unclipped) | textbook F15 shape: flat 8.7 → peak 27.4 → decay |

**mean 462 ms, sd 58 ms, range 406–522 ms. The firmware assumes `IGN_DELAY = 200 ms`.**

**Consequence.** The TVC window is a fixed `IGN_DELAY + BURN_TIME` = 3.65 s from the ignition command, but
real burnout is at `real_lag + 3.45 s` ≈ 3.91 s. So **the gimbal used to centre while the motor was still
burning, for up to 322 ms** — with a centred gimbal on a vehicle whose thrust axis does not pass exactly
through the CG, that is an uncontrolled `T·L·misalign` torque with nothing opposing it. `TVC_EXTEND_S = 1.0 s`
(added 2026-08-03) covers the worst measured case with **3.1× margin**. SIL, 2.0° misalignment: at 0.4 s of
excess lag the old behaviour ends the boost with a 3.30° lean, at 0.6 s with 12.60°, at 0.8 s with 31.63° —
the extension holds 0.16° in every case, and costs nothing at zero lag.

**`IGN_DELAY` was deliberately NOT changed to the measured value.** It also feeds the backup-chute timer
(`IGN_DELAY + BURN_TIME + BACKUP_AFTER_BURNOUT_S`). Leaving it at 0.2 s makes that timer fire ~260 ms
*earlier* relative to true burnout — the safe direction, still comfortably past apogee — whereas raising it
delays the failsafe on the strength of a ±50–100 ms estimate from three flights. Set it from
`ignition_lag_ms` in the next `MTR###.CSV`, which is measured at **loop rate (~1–3 ms)**, not 20 Hz:
`captureMotor()` tests the thrust-onset threshold every iteration, and only the buffered curve is gated at 100 Hz.

**Open question this raises for the landing.** The lag varies by **116 ms across three flights**. For a
hoverslam igniting at a computed altitude while descending at ~20 m/s, ±116 ms is **±2.3 m of altitude
error** — a first-order error source for the landing burn, not a detail. Note also that lag is a property of
the **igniter, pyro channel and battery state** as much as the motor, and the descent motor lights from a
different channel after seconds of servo draw, so the ascent measurement may not transfer directly.

---

## How to log the next flight

1. **Before the flight** — fill the airframe columns *by measurement*, not by copying the last row.
   Mass on a scale; moment arm CG→TVC pivot with a ruler after a balance test; Iyy by bifilar/pendulum
   swing if it was rebuilt. If you did not re-measure, write `unchanged since <date>` — that is a claim
   you can defend; a copied number is not.
2. **Record the firmware** — `git rev-parse --short HEAD` plus any `#define` you changed
   (`CTL_CAPTURE`, `CTL_SERVO_FEEDBACK`) and the P/D/I gains actually flashed.
3. **After recovery** — copy **all four** files off the card straight away: `ASC###.CSV` (20 Hz flight),
   `CTL###.CSV` (500 Hz control + loop timing), `MTR###.CSV` (motor curve), and note the SD copy date.
   The card has no RTC; the copy timestamp is your only date evidence.
4. **Fill the measured columns from the data**, not from impression. `Firmware/analyze_flight.py`
   plots `ASC###.CSV`; the `CTL###.CSV` header block already reports loop min/median/max, capture
   overhead, and saturated fraction.
5. **Outcome vocabulary** — keep it comparable across rows:
   `SUCCESS` (flew the intended profile, controlled), `PARTIAL` (flew, but a stated objective or the
   data return failed), `FAILURE` (lost control / aborted / destroyed), `NO IGNITION`, `SCRUB` (never
   left the pad by choice).
6. **Add the row to `FLIGHT_LOG.csv` too** — that is the file analysis scripts read.
