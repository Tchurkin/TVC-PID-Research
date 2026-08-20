# DESIGN_LOG.md — Engineering Decision Record

> Append-only. **Add new entries at the TOP** (reverse chronological). Never edit or delete a prior
> entry — if a decision is reversed, add a new entry that supersedes it and link back. Each entry:
> Decision / Reasoning / Alternatives / Tradeoffs / Risks / Date.

This log is seeded (2026-06-30) from decisions already evident in `CLAUDE.md` and `memory/`; those
files remain authoritative for the full detail. New decisions go here going forward.

## 2026-08-17 — Impulse 2.3: Q20 main battery switch → AON6403; the nichrome self-clears; P1/P2 field failure

- **Decision — Q20 upgraded.** The main battery switch becomes the **same AON6403** as Q21
  (LCSC **C2760089**), moved to **(175.700, 79.500) rot 0 on B.Cu**, east of the buzzer.
  **AO3401A now disappears from the board entirely**; the whole switching path is two part numbers.
  Q20 shares Q21's feeder, so this adds **no BOM line and no extra fee**.

- **Reasoning.** Q20 is a **single point of failure for the entire avionics stack** — the Teensy that
  fires the chute is powered through it. As an AO3401A a sustained multi-servo stall put it at
  ~2.4 A → **0.43 W in a SOT-23 → Tj ≈ 135 °C, *continuously***, not for a 900 ms pulse. At 4.3 mΩ
  that becomes ~0.03 W. `FAILURE_MODES.md` A2 already names servo-stall pack sag as the
  ASC036/ASC038 prime suspect, so the stall condition is real rather than hypothetical.

- **Why it moved east.** Its old pocket has only **4.52 mm** between the BUZZER's through-hole pads
  (bottom edge 78.44) and R66 (top edge 82.96); a DFN5x6 needs 4.96 plus clearance. Moving east
  costs nothing electrically because **VBAT already leaves Q20 eastward** — to C39, then via In2
  around to R54/U2/U3 — so the drain tab now sits directly on the path it already took. Only
  VBAT_RAW lengthens (~4 mm of 1.2 mm trace, ≈2 mΩ). The gate takes a two-via In1 jog for the same
  reason Q21's did: it is on the far side of the source-pad column. It is a ~67 µA signal.

- **Correction to 2026-07-23 — the nichrome DOES clear.** That entry called the hot wire "a
  non-clearing resistive load held ON for seconds". Braxton reports the opposite: the wire **melts
  through and breaks its own continuity**, so FET conduction time is the *melt time*, not the
  commanded 900 ms. This does not change what 2.3 needs — it is sized for the pessimistic case
  (never clears, full 900 ms at 20 A) — so self-clearing is pure extra margin. It does mean the
  firmware's 900 ms is a **timeout, not a duration**, and post-fire continuity already detects the
  clear.

- **Field evidence — P1/P2 failed on the 2.2 board (2026-08-16).** P1 reads **always low**, P2
  **always high**. Always-low is the signature of a firing FET **shorted drain-to-source**: the
  sense node is clamped to ground regardless of the initiator. ⚠ **That channel would fire the
  instant the rail comes up, with no gate command** — a fire-on-arm hazard. Test with the pyro
  battery disconnected: P#O pin 1 → GND should read ≥10 kΩ (the R37/R41 sense path); near 0 Ω is a
  shorted FET. This **corroborates the 0.3 Ω analysis**: at 14 A the CSD17313Q2 took 11.5 W in a
  3 W package, and `Impulse22_PyroTest.ino` defaults to `PYRO_CHANNEL = 2` — the damaged channels
  are the ones that saw current. The firing FETs were not collateral damage; they were the same
  failure one step downstream.

- **Sensor interface — deliberately NOT reverted.** 2.3 renames ribbon pin 8 `GPS_PPS` → `CS_MAG`
  for the v2 sensor board. Braxton is flying the v1 board and asked whether that needed undoing:
  **it does not.** The nets have identical membership (`J_SENSOR1` pin 8 ↔ Teensy pin 20), the silk
  carries no pin-8 label, and the change is a pure CAD rename — so the same copper serves **both**
  sensor boards and only firmware decides what pin 20 does. The one real caveat is **pin direction**:
  the v1 board's GPS *drives* PPS into pin 20, so firmware must keep it an input (it does —
  `PIN_GPS_PPS = 20`). Mag firmware driving it as a chip-select with the v1 board fitted would be
  output-against-output.

- **Process failure worth recording.** The board was found in a **broken, unverified state** at the
  start of this session: 15 unconnected and 136 DRC errors, with Q16–Q19 displaced ~3 mm south onto
  the CS_IMU/INT_IMU via field the y=45.5 placement exists to avoid, Q21's rotation reset 90 → 0,
  and PI/C15/R67/R68 shifted. The last verified-clean artifact was `drc_mag_main.json` (2026-08-15).
  Recovery: restore `Impulse_2.3.kicad_pcb.pre_mag`, re-apply the mag rename, redo Q20. The corrupt
  state is kept as `Impulse_2.3.kicad_pcb.corrupt_20260817`. **Lesson:** a pcbnew
  `LoadBoard/Remove/Add/SaveBoard` cycle is not a safe no-op for the rest of the board, so
  `_guard_placement.py` now snapshots every footprint's `(at x y rot)` before such a run and puts
  back anything that moved. Re-running the placement under the guard reported *nothing moved*,
  which is how the corruption was pinned on the earlier unverified edit rather than on this session.

- **Verification.** ERC 0 errors. DRC vs the restored baseline: **0 unconnected, 0 schematic parity,
  0 clearance, 0 shorts** — courtyard 21 and pth_inside_courtyard 18, *identical* to the pre-Q20
  baseline, so Q20 added **zero** new DRC items. Netlist: 77 nets in 2.2 and 2.3, none added or
  removed, **zero membership changes outside the six FETs**. Q20 reads source = pins 1,2,3, gate =
  pin 4, drain = tab pin 5 — the AON6403 map.

- **Tradeoffs / risks.** (1) **Q20's CPL rotation is unverified**, like the other five: CPL emits
  **180** (KiCad 0 + this board's +180 bottom-side convention, re-derived from the CPL's own rows).
  A 180° error on the main battery switch is a dead board — check JLC's preview. (2) One more via
  pair on a gate net. (3) The 0.3 Ω initiator figure is still a single unzeroed multimeter reading,
  and every thermal number scales with it.

- **Decision — firing FETs CSD17302Q5A → CSD17301Q5A** (LCSC **C129940**), so the two halves of the
  switching path are *matched* rather than leaving the pyro side as the weak one. Braxton's ask was
  that nothing bottleneck: the two groups carry **identical** current (they are in series in one
  loop), so the question is only which runs closer to its limit, and at 20 A / 900 ms it was
  **2.4 W / Tj ≈ 44 °C** in the AON6403 arm switches against **5.3 W / Tj ≈ 84 °C** in the
  CSD17302Q5A firing FETs — a 2.2× mismatch. The CSD17301Q5A is **2.9 mΩ guaranteed at V_GS = 3 V**
  (vs 9.5) → **~2.1 W / Tj ≈ 48 °C**, matching the arm side to within 15%. An independent calculation
  of "what R_DS(on) would equalise the 900 ms rise" returned ≤3.1 mΩ; the part is 2.9.

  **Cost: zero layout.** Identical SON 5×6 footprint and — verified, not assumed — an identical pin
  map (`1 D 2 D 3 D 4 D  5 G  6 S 7 S 8 S`, confirmed from TI's own datasheet text for both parts).
  It *replaces* the CSD17302Q5A, so the feeder count is unchanged. Netlist after the swap: 77 nets,
  none added or removed, zero membership changes outside the six FETs; DRC and ERC byte-identical to
  before (0 unconnected, 0 parity, 0 clearance, 0 shorts).

  **The sleeper benefit is avalanche energy: E_AS 414 mJ vs 61 mJ, 6.8×.** When the nichrome melts
  and opens ~20 A into the harness inductance, that energy has to go somewhere, and this is the
  rating that absorbs it. Given the wire self-clears by design (above), that event happens on *every*
  firing, not as a fault case.

  **Why stop here rather than go bigger.** "As big as needed to never fail" is now met on every axis:
  thermal Tj ≈ 48 °C against a 150 °C limit; pulse current 181 A against 20 A (9×); avalanche 414 mJ
  against the ~0.2 mJ the harness actually stores (2000×). Going lower in R_DS(on) would buy nothing
  measurable — the FETs are **~4 % of a 469 mΩ loop**, so the initiator and pack internal resistance
  set the current — and it would re-create the mismatch in the opposite direction by making the arm
  FETs the weak half.

- **Date:** 2026-08-17.

## 2026-08-15 — Sensor Board v2: MMC5983MA magnetometer added; ribbon pin 8 GPS_PPS → CS_MAG

- **Decision.** New board revision `PCBs/Sensor_Board_v2_kicad/`, forked from the v1 sensor board.
  Adds **U3 = MMC5983MA** (3-axis magnetometer, ±8 G, 0.4 mG RMS, LGA-16 3×3, LCSC **C404329**,
  JLC assembly library) on the existing SPI bus, plus C9 (10 µF, the SET/RESET coil reservoir),
  C10 and C11 (100 nF decoupling).

- **The binding constraint was the ribbon, not the board.** All 12 conductors were allocated. An SPI
  magnetometer shares SCK/MOSI/MISO but needs its own chip select, so **pin 8 was repurposed from
  GPS_PPS to CS_MAG**. `SENSOR_BOARD_DESIGN.md` already recorded PPS as *"Optional … not needed for
  the EKF position aiding"*, and on Impulse 2.3 pad 8 already lands on Teensy pin 20 — so the change
  costs **no copper on the main board at all**, only a net rename plus firmware. Rejected: widening
  the ribbon to 14 pins (would mean re-laying out J_SENSOR1 on a board that is verified and about to
  be ordered) and putting the magnetometer on the main board (magnetically the worst place on the
  vehicle — it is next to the pyro FETs now carrying 14–17 A).

- **Pin map verified, not recalled.** Taken from the MMC5983MA Rev A pin-description block and
  cross-checked against SparkFun's production Eagle library for the same part; they agree exactly.
  This mattered: **KiCad's stock `Package_LGA:LGA-16_3x3mm_P0.5mm` is the wrong land pattern** — its
  own `descr` cites the MMC5**8**83MA and it places pad 1 on the LEFT edge, while the MMC5**9**83MA
  numbers counter-clockwise from the TOP-RIGHT. Same 16 pads, rotated 90°, and using it would have
  silently swapped SPI pins onto power pins. A correct footprint was generated from SparkFun's
  shipping geometry into `sensorv2.pretty/`.

- **CAP is its own net**, with only the 10 µF cap on it — not tied to the rail. The datasheet states
  VDD and CAP are shorted *inside* the device, and CAP feeds the SET/RESET degauss coil. That coil
  is the reason this part was chosen over the cheaper LIS3MDL: it can clear residual magnetisation
  after a large field exposure, which on this vehicle means after a pyro firing.

- **Placement obeys the datasheet's hard rule** — *"Do not route current carrying traces under the
  sensor or on the other side of the PCB opposite the device."* U3 sits ~16 mm from both current
  sources (the ribbon and the GPS module), the furthest clear site on a 35.8 mm board, and **no track
  runs under its footprint on any layer**. In1 is untouched (dedicated GND plane).

- **Verification.** Sensor v2: ERC **0 violations**, DRC **0 errors, 0 unconnected, 0 parity** — only
  6 cosmetic silk warnings above the v1 baseline. Netlist diff vs v1: `CS_MAG`/`MAG_CAP` added,
  `GPS_PPS` removed, **no membership change to any pre-existing net**. Impulse 2.3 after the rename:
  77 nets, only GPS_PPS → CS_MAG, **zero other changes**, DRC and ERC counts identical to before.

- **Honest limits, both recorded in the v2 spec.** (1) **The magnetic axis orientation is not
  established** — the datasheet's axes figure has no extractable text, so unlike the ICM triad it was
  not derived. U3 is placed at rotation 0 so its frame is axis-aligned with the IMU and the
  correction can only be a 90° multiple plus signs, but it **must be settled on the bench** before
  any heading output is trusted. (2) **Off-board interference dominates and layout cannot fix it**:
  a 15 A pyro pulse is ~1.2× Earth's field at 50 mm, servos ~0.24×, and the **steel motor casing is a
  soft-iron distorter that changes as the grain burns**. That, not the wiring, is the dominant
  heading error on a solid-motor rocket.

- **Firmware consequences.** GPS PPS is gone (J_GPS pin 5 is a no-connect); Teensy pin 20 becomes a
  GPIO chip select. The magnetometer is **polled** — there is no spare conductor for its INT.

- **Date:** 2026-08-15.

## 2026-08-13 — Impulse 2.3: the WHOLE pyro switching path upsized, after the initiator measured 0.3 Ω

**Supersedes the sizing in the 2026-08-12 entry below.** That entry replaced Q21 with an AO4407A
sized for the ~7–8 A the firmware comments assumed (a 1 Ω initiator). Braxton then measured the
initiator at **~0.3 Ω**. That changes the design point completely and the AO4407A choice with it.

- **The real load.** 0.3 Ω initiator + ~0.05 Ω harness + FETs + pack internal resistance puts the
  firing loop at **12–17 A**, not 2–4 A. The 2026-07-23 entry's instruction — *"keep the nichrome
  sized so the 900 ms cut current stays ~2–4 A"* — is not what is actually being flown. At 14 A,
  held for the firmware's 900 ms:
  | part | P | ΔT over 900 ms | verdict |
  |---|---|---|---|
  | AO3401A SOT-23 (the part that died) | 16.5 W | ~2100 °C | obliterated — fully explains the failure |
  | CSD17313Q2 SON 2×2 (Q16–Q19) | 11.5 W | ~254 °C | **would also fail** on a sustained pulse |
  | AO4407A SOIC-8 (yesterday's fix) | 4.5 W | ~121 °C | Tj ≈ 146 °C vs a 150 °C limit — borderline |

- **Decision.** Braxton's call was to upsize the silicon rather than trade margin for empty board
  area. Both new parts verified against their datasheets and confirmed in JLCPCB's assembly library
  (Extended, same tier as the CSD17313Q2 they replace):
  - **Q21 → AON6403**, DFN 5×6, LCSC **C2760089**. −30 V, −85 A, **R<sub>DS(on)</sub> ≤ 4.3 mΩ at
    V<sub>GS</sub> = −4.5 V** (the R67/R68 divider gives −6.7 V, so the spec row applies).
    At 20 A: **2.2 W**, and even at *steady state* Tj ≈ 115 °C — it survives the pulse indefinitely.
  - **Q16–Q19 → CSD17302Q5A**, SON 5×6, LCSC **C553151**. 30 V, 87 A,
    **R<sub>DS(on)</sub> = 9.5 mΩ guaranteed at V<sub>GS</sub> = 3 V** — the binding constraint,
    because the Teensy drives these gates directly with no driver. At 20 A: **5.3 W, Tj ≈ 85 °C**.
  Total on-resistance in the firing loop drops **84 mΩ → 19 mΩ**. Going lower would be pointless:
  at 0.3 Ω the FETs are only ~6 % of the loop, so a 2.7 mΩ part moves the current 17.9 → 18.4 A.

- **Rejected.** AO4407A for the firing FETs — it is **P-channel**, and Q16–Q19 are N-channel
  low-side switches with their sources at GND; it would never turn on. Separately its lowest
  guaranteed R<sub>DS(on)</sub> row is −6 V, and the Teensy drives 3.3 V. This is exactly the trap
  the 2026-07-23 entry documented.

- **Layout consequences.** The firing FETs went from a ~2×2 mm package to 6.80 × 4.70 mm with a
  4.6 × 4.7 mm drain tab. Placed at **rot 270** so each tab sits directly over its connector's pin-1
  pad — same net, so the firing current gets a **zero-length connection instead of a trace** — and
  the gate lands south-east facing the PYROn traces. They shifted north to y = 45.5 to clear a via
  field (CS_IMU, INT_IMU, GPS_RX, PYRO_SW) that the lead rows had landed on. Q21 rotated 90° with
  its tab north, which also improved its side clearance from 0.48 mm to ~1.4 mm.

- **A consequence worth recording:** because each drain tab now overlaps its connector pad directly,
  **every remaining track on the `Net-(PnO-Pin_1)` nets is continuity-sense only (~0.6 mA)**. The
  2026-08-12 widening pass had fattened those to 2.5 mm, which bought nothing and actively blocked
  routing — they are back to 0.3 mm, and that is what finally freed the corridor for PYRO2's gate.

- **Verification.** ERC **0 errors**. DRC vs the 2.2 baseline: **0 clearance, 0 shorts, 0 unconnected,
  0 schematic-parity**. Netlist diff vs 2.2: 77 nets both, none added or removed, and **zero
  membership changes outside the five swapped FETs**. New DRC items are 1 courtyard overlap and 4
  `pth_inside_courtyard` — the latter deliberate, being the tab-over-connector-pad overlaps.

- **Method note (cost real time, twice).** Clearance-checking planned copper against the board *as it
  was before the batch* lets additions collide with **each other**: it produced a real PYRO1↔PYRO_G
  short and a via sitting on a rail. Check every addition against the pending set too. Same lesson
  as the widening pass. Also: a via scan is worth more than arithmetic — In2 carries a board-wide
  horizontal signal bus whose 0.81 mm gaps fit no via, which invalidates whole classes of route.

- **Tradeoffs / risks.** (1) **Rotations for all five FETs are unverified** — these are new custom
  footprints, so none of 2.2's JLC rotation calibration carries over; `fab/ORDER_NOTES.txt` flags
  all five for JLC-preview confirmation. (2) Two Extended parts (two feeder fees). (3) **Q20 is
  still an AO3401A** and is a single point of failure for the whole avionics stack.

- **Firmware, unchanged but implicated.** The wire dissipates ~59 W at 14 A, so it cuts far faster
  than 900 ms. The silicon now survives that, but holding the channel on after the cut still sags
  the pack to ~6.2–6.8 V. Measure the cut time and shorten the pulse; keep the burst-fire logic as
  defence in depth.

- **Date:** 2026-08-13.

## 2026-08-12 — Impulse 2.3: pyro ARM FET Q21 AO3401A/SOT-23 → AO4407A/SOIC-8 after a bench failure

- **Decision.** New board revision `PCBs/Impulse_2.3_kicad/`, forked from the ordered 2.2 files.
  Q21 (the high-side ARM switch through which **every** pyro channel's firing current flows) becomes
  an **AO4407A in SOIC-8**, LCSC **C16072** (JLC assembly library: Extended). The pyro rail and drain
  copper is widened from 1.0 mm to 1.2–2.5 mm wherever neighbours allow (42 of 112 segments).
  Everything else on the board is unchanged — a kicad-cli placement diff shows exactly **one** row
  differing from 2.2.

- **The failure.** Q21 died on the bench 2026-08-12 during a single-channel pyro test. This was
  **predicted by this log** on 2026-07-23, in the "REMAINING MANUAL STEPS" of the Q16–Q19 entry, item
  (3): *"the sustained ~2–4 A legs current also flows through the arm P-FET Q21 (AO3401A, ~0.5–0.9 W
  in SOT-23 at 4 A for 3 s — borderline) … consider Q21 margin."* The step was never done. The real
  load was worse than that estimate: ~4.9 A into a 1.5 Ω initiator, ~7 A into a 1 Ω one, and at
  60 mΩ (>80 mΩ hot) that is several watts in a package whose transient thermal impedance is
  ~130 °C/W at one second — 150–300 °C of junction rise on a single full-length pulse.

- **Reasoning — why hardware, not just firmware.** `Impulse22_PyroTest.ino` already mitigates this in
  software by bursting (30 ms on / 150 ms off). That is a good pad-test guard but it **cannot be the
  whole answer**, because the LEGS channel drives a **nichrome band cutter** that is bench-verified to
  cut at **900 ms continuous** — bursting it lets the wire cool between pulses and defeats the cut.
  The arm switch has to survive a sustained pulse in hardware.

- **Part choice, checked against the datasheet rather than from memory** (AOS AO4407A Rev 11.1):
  −30 V, −12 A, **RDS(on) ≤ 17 mΩ at VGS = −6 V**, VGS rating ±25 V, RθJA **40 °C/W max for t ≤ 10 s**.
  The gate sits at **−6.7 V** (R67 100 k / R68 10 k divider off a 7.4 V pack, −7.6 V at 8.4 V), so the
  −6 V spec row is the one that applies and it is guaranteed, not extrapolated. Worst case 8 A:
  8² × 0.023 Ω (hot) = 1.5 W × 40 °C/W ≈ **59 °C rise**, against several hundred before — roughly an
  **11× reduction in junction rise**. The nichrome case (4 A, 900 ms) is 0.37 W ≈ 15 °C, trivial.
  A gate-to-source short fails **safe** for a P-FET (VGS = 0 → off), which is the right direction for
  the one net pair the new layout brings close together.

- **Alternatives.** (a) **AON7407 / AON7403** (DFN 3×3, exposed pad) — thermally better still, and
  AON7407 is 9.5 mΩ; rejected because SOIC-8 already has ~2.5× the needed margin, needs no thermal via
  array, and AON7407 is only a 20 V part. (b) **Firmware burst-fire alone** — rejected above; it is
  retained as defence in depth, not as the fix. (c) **Paralleled AO3401As** — still SOT-23 thermals.
  (d) **Leaving the arm switch out of the firing path** — rejected: a series high-side switch that
  physically breaks the rail is the safety property worth having.

- **Symbol/footprint mechanics.** KiCad's `Transistor_FET:IRF7404` is a single P-channel SO-8 whose pin
  **endpoints are identical** to AO3401A's (G = (−5.08,0), S = (2.54,−5.08), D = (2.54,5.08)) and whose
  numbering is the standard SO-8 power map (1,2,3 = S; 4 = G; 5–8 = D) that AO4407A uses. Cloning its
  body as `AO4407A` preserves the schematic wiring **by construction** — the same trick as the 2026-07-23
  CSD17313Q2 swap — so no custom symbol or footprint was needed and the stock
  `Package_SO:SOIC-8_3.9x4.9mm_P1.27mm` is used.

- **Routing note worth keeping.** On a SOIC-8 the gate is pin 4, the *southern-most* left pad, below all
  three source pads — but west of Q21 the gate lane runs *north* of the source lane. That order
  inversion makes **exactly one crossing topologically unavoidable**, and it could not be taken either
  on F.Cu (the C15/R67/R68 corridors are 0.7 mm) or with a via just south of the rail, because **In2
  carries a board-wide horizontal signal bus** (CS_IMU 51.564, SENS_DET 52.374, INT_IMU 53.183, MISO
  54.038) whose 0.81 mm gaps cannot take a 0.6 mm via plus clearance. PYRO_G therefore drops to In1 at
  R67, runs under the source pads, and returns through the one via-legal pocket in the whole area — the
  empty region **under the SOIC-8 body**, between the two pad rows.

- **Rail widening — the other half of the 2026-07-23 flagged item.** A 1.0 mm / 1 oz trace at ~8 A for
  900 ms heats ~235 °C adiabatically (ρJ²/cᵥ = 261 K/s); 2.5 mm cuts that ~6×. The widening pass also
  found and fixed **four 0.3 mm necks on the pyro drain nets** — though note these turned out to be on
  the continuity-**sense** branches (to R37/R41/R42/R43), not the firing path, so they were never the
  hazard they first looked like. Two rules were learned by having DRC reject the first attempt: size
  against **live** geometry (or two neighbours both widen into each other — it produced a real
  P3O/7.4V short on In2), and check the **board edge** (a trace landed 0.383 mm from the 35 mm-radius
  outline against a 0.5 mm rule). Widening is never allowed to narrow a segment below its 2.2 width.

- **Verification.** ERC 0 errors (64 warnings vs 2.2's 63; the extra is `lib_symbol_issues` because
  AO4407A is not a stock KiCad library part). DRC vs the 2.2 baseline: **0 unconnected, 0 schematic
  parity**, no clearance / short / hole / mask-bridge / edge errors. The only new items are **2
  courtyard overlaps** (Q21 vs R67, Q21 vs C10) — accepted: actual pad-to-pad clearance is **0.48 mm**
  on both sides and body-to-body is over 2 mm.

- **Tradeoffs / risks.** (1) **Q21's CPL rotation is unverified** — 2.2's value of 180 was a SOT-23-family
  JLC calibration and the part is now SOIC-8; the CPL emits 0 and `fab/ORDER_NOTES.txt` flags it for
  JLC-preview confirmation, which this project's notes call the only ground truth. (2) AO4407A is a JLC
  **Extended** part (feeder fee), same tier as the CSD17313Q2 already on the board. (3) One via now sits
  **under the Q21 package body** — standard practice, but it is there. (4) The board was forked from the
  **on-disk** 2.2, which is the state that was ordered on 2026-08-01 and that failed; a KiCad session had
  2.2 open at the time, so any unsaved edits in that session are not in 2.3.

- **NOT done, and recommended — Q20.** Q20 is the *same* AO3401A/SOT-23 on the main battery, feeding
  both bucks (U2 5 V/2 A, U3 5 V/3 A → servos). Worst case is a sustained multi-servo stall: ~2.4 A
  → ~0.43 W in SOT-23 → **~110 °C rise, i.e. Tj ≈ 135 °C** with essentially no margin, and unlike Q21's
  case that condition is *continuous*, not a 900 ms pulse. Normal servo load is ~0.6 A and utterly
  benign, so this is a margin concern rather than a demonstrated failure — but `FAILURE_MODES.md` A2
  already names servo-stall pack sag as the ASC036/ASC038 prime suspect, so the condition is real.
  Upgrading Q20 to the same AO4407A would cost **no extra JLC feeder fee** (same part number), but it
  needs re-placement: Q20 sits on B.Cu with R66 2.54 mm away and the BUZZER's through-hole pads 3.9 mm
  off, so a SOIC-8 does not drop in. Left as a decision for Braxton.

- **Date:** 2026-08-12.

## 2026-08-09 (evening) — LQR and ADRC evaluated and declined; the real headroom is in the gains already present

### Decision — do not adopt LQR

**Decision.** No LQR. The plant is a double integrator per axis (θ̈ = keff·δ), and solving the Riccati
equation for it with Q = diag(q,0), R = r gives `k1 = sqrt(q/r)`, `k2 = sqrt(2·k1/keff)`, hence
`wn = sqrt(keff·k1)` and **`zeta = sqrt(2)/2 = 0.707 identically`**, for every q and r. LQR *is* PD on
this plant, with zeta locked and only wn free — a strictly smaller design space than the pole placement
already in use.

**Measured, not asserted.** A full (wn, zeta) sweep — 70 dispersed airframes per cell, servo lag
dispersed — shows the zeta = 0.707 column is the WORST at every wn: 90.0% vs 94.3% at wn=8, and 54.3%
vs 75.7% at wn=6. LQR cannot reach the better cells because it cannot leave that column.

**What would change this.** LQR earns its keep when the state is richer than the plant's — e.g. a
3-state design including actuator lag, which pole-placed PD cannot account for. That is worth revisiting
**after `Bench_Latency` measures tau** (open question 3). Designing an "optimal" controller around an
unmeasured parameter is the failure mode this project's own methodology rules exist to prevent.

### Decision — do not adopt ADRC now

**Decision.** No ADRC before the landing attempt.

**Reasoning.** ADRC's claim is rejecting a lumped "total disturbance" so plant uncertainty stops
mattering. Here the disturbances are enumerable and already handled: thrust misalignment is the dominant
one, the integral trim already cancels it, and it is **authority-limited, not control-limited** — a 4°
misalignment costs 4° of a ±5° gimbal no matter what the controller is (2026-08-08 entry). Plant
uncertainty in keff was measured to be worth under a point of pass rate. ADRC would also relocate the
tuning rather than remove it: observer bandwidth becomes the knob, and it is limited by the same sensor
noise and actuator lag that limit the PD gains.

**What would change this.** Evidence that a disturbance the I-term cannot see is limiting the vehicle —
a real, unmodelled, time-varying torque. Nothing measured so far looks like that.

### The finding that matters more than either

The current design (wn=8, zeta=1.0) is **not optimal**. wn=12 / zeta=1.2 gives 95.7% PASS and boost-tilt
p90 **3.91° vs 8.01°**, and it holds at every servo lag from 0.015 s to 0.160 s (4.5× the assumed tau)
with no crossover.

**This cross-validates the research.** The project's gain-ceiling result was measured with Kd FROZEN
while Kp was pushed — sequential tuning. Raising wn and zeta together holds Kd/Kp fixed, i.e. coupled
tuning, and the central 2026-08-03 finding (n=2400) is exactly that the ceiling is an artifact of moving
them independently. The firmware sweep and the paper's headline agree, independently.

**Not adopted in this commit, deliberately.** The simulator cannot express servo backlash or deadband,
and backlash-driven limit cycles are the classic high-gain failure. Servo-angle quantisation *is*
modelled (PWMServo takes integer degrees and the harness quantises identically), so that part is
covered; mechanical slop is not. Recorded at `TVC_WN` with the adoption procedure: set 12.0/1.2, then
bench-verify that the gimbal settles without hunting before flying it.

**Date.** 2026-08-09

---

## 2026-08-09 (pm) — Torque-commanded TVC: gains derived from mass properties, and why the schedule is ONE-SIDED

### Decision — derive the gains, identify the thrust, but do not normalise keff

**Decision.** `P_GAIN`/`D_GAIN` are no longer the design parameters. `TVC_WN` (8.0 rad/s) and `TVC_ZETA`
(1.0) are, and the angle gains are computed as `wn²/keff` and `2·zeta·wn/keff` from a live plant model:
mass, CG and Iyy propagated through propellant burn-off, and thrust **measured** from the accelerometer
(`T = m·a_z + D`) using the F15 curve for shape and an in-flight identified amplitude (`thrustScale`).

**Reasoning.** `P_GAIN`/`D_GAIN` were never independent numbers — they are that same formula evaluated
once, by hand, at keff = 130. Hard-coding the result is what makes the gains go stale on every rebuild
(ASC036 destroyed the airframe; keff went 257 → 130 and the flight gains stayed poled for the dead
rocket until someone noticed).

**Measured.** Identified keff is within **0.6% of truth** across ±15% motor variation (3% lock). On the
nominal airframe the new law reduces to the flown gains exactly (boost tilt 4.90 vs 4.89). On a heavier
rebuild, entering the numbers gives **3.82° vs 5.00°** boost tilt against stale gains — and matches
hand-rescaling to 2 decimals (3.76 vs 3.75 in the wider sweep), which is the whole point: it does the
arithmetic correctly, every time.

**The refutation, and what it changed.** Full normalisation — holding wn and zeta constant by dividing
out every change in keff, which is what "uniform control through the burn" literally means — was built
first and **measured worse**: paired A/B over 140 dispersed airframes gave HIGHER boost tilt on **139 of
140** and dropped PASS from **100% to 94.3%**. The reason is that `zeta = sqrt(keff)·D_GAIN/(2·sqrt(P_GAIN))`,
so zeta RISES with keff: the 26 N ignition spike hands the loop a faster AND better-damped response for
free, exactly when the rail tip-off needs rejecting. Normalising it away spends real margin to hit a
number. The comment at `P_GAIN` ("keff 215-520 -> zeta 0.9-1.4") had said so all along.

So the schedule is now **one-sided**: `sched = constrain(keffNominal/keffEst, 1.0, 2.5)`. It boosts gains
when keff falls below nominal (weak motor, heavy build, tail of the burn) and is inert when keff is
above it. Loop gain never exceeds the design value, so this cannot climb toward the gain ceiling.

**Also measured, and worth stating plainly:** the in-flight schedule is worth ~nothing against motor
variation alone (±15% thrust moves boost tilt by less than the run-to-run noise). Essentially all the
value is in the **boot-time derivation** from mass properties, not in the in-flight adaptation. And on a
*lighter* rebuild stale gains beat correctly-retuned ones (4.81 vs 6.42) for the same zeta reason — so
"the rocket got lighter" is the safe direction to be wrong in, and "heavier" is the dangerous one.

**Alternatives.** Keep hand-computed gains (rejected: this is the stale-gain failure mode, and it is a
recurring one). Full normalisation (rejected on the measurement above). Schedule on elapsed time rather
than measured impulse (rejected: ignition lag is 462 ± 58 ms and was 920 ms on ASC038).

**Tradeoffs.** More flight code in the control path, and it now depends on the accelerometer being sane —
mitigated by rejecting rather than clamping out-of-band keff, holding the last good value, and the
`GAIN_SCHED` clamp on top. `TVC_TORQUE_CMD` is a runtime bool so the legacy law can be flown or A/B'd.

**Risks.** `PROP_ARM_M` (gimbal pivot → propellant centroid) is a **ruler measurement that has not been
taken** — the DRY_* values are back-derived from the measured wet numbers using it. The boot print shows
the derived wet state so it can be checked against the bifilar result. RAM1 rose 176.7 → 196.8 kB
(196 kB still free). Never flown.

**Date.** 2026-08-09

---

## 2026-08-09 — Simulating a real MCU reset, and the three bugs it found

### Decision — a reset relaunches the process

**Decision.** `ascent_sim.cpp` gained `--resetat` / `--statein` / `--stateout`. On a reset it snapshots the
world (physics + EEPROM + clock + RNG), exits 7, and `run_flight()` in `regression.py` relaunches it.

**Reasoning.** The harness had set `g_brownedOut` and done nothing with it — the comment literally said
"MCU would reset here" and no reset ever happened. `setup()` ran exactly once per flight, so
`resumeAfterReset()`, the whole brownout/watchdog recovery feature, **had never executed in flight
conditions**. The only coverage was `--seedphase`, which is the *negative* case (a stale record on the
ground must not resume). A faithful reset needs fresh firmware globals, and those are file-scope statics
inside the `.ino` — unreachable from inside one process. A new process gets genuinely fresh `.data`/`.bss`,
which is exactly what the hardware does.

**Three bugs found on first run.**

1. **`looksAirborne()` was blind at burnout.** It took min/max across a 400 ms window and asked whether
   the *whole* window was free-fall (`maxA < 0.55 g`) or the *whole* window was thrust (`minA > 2 g`).
   A window straddling burnout is neither, so both failed. Measured: the vehicle at **37 m climbing at
   19.7 m/s** was declared to be on the ground and the chute never came out — the only reset time in the
   entire flight that lost the vehicle, sitting exactly at burnout, which is peak servo current and the
   likeliest moment to brown out. Now **per-sample**: thrust (~2.1 g) and free-fall (~0.1 g) both deviate
   from 1 g, so a mixed window becomes a strong positive. Threshold 60% of the window — deliberately
   stricter than a bare majority, because this is the interlock that stops a stale record firing a pyro
   in someone's hands, and a vehicle resting in a hand scores ~0%.

2. **`imuReadSource()` discarded `getEvent()`'s return value.** A timed-out or NAKed read was published
   as a live sample. Every downstream protection — failover, stuck detection, both-dead, and therefore
   the blind abort — keys off that bool, so throwing it away disabled all of them at once.

3. **The watchdog did not cover `setup()`.** It is started last, deliberately, so a slow sensor probe
   cannot trip it. But a board that reboots into a persistent fault hangs in the IMU probe *before*
   `wdtStart()`, unprotected, until impact. Now armed early when `persist.phase != 0` — in-flight reboot
   only, pad behaviour unchanged.

**Correction to a prior claim.** FAILURE_MODES D2 said a stuck I²C bus wedges the loop forever and that
Teensy's timeout API was unused. Checked against `WireIMXRT.cpp` (Teensyduino 1.59): **false.** The driver
already bounds every transfer — `CLOCK_STRETCH_TIMEOUT` 15 ms in hardware plus a 50 ms generic bail-out
returning error 4 — and there is no `setWireTimeout()` on this core. I briefly added such a call; it does
not compile. The hang was always bounded; the defect was bug 2 above.

**Alternatives.** Re-zero globals in-process (impossible for `.ino` statics); `fork()` (not available on
Windows); ignore the feature (it is the one the vehicle's survival depends on after a rail collapse).

**Tradeoffs.** Tests that involve a reset must go through `run_flight()`, not `run()`. The snapshot must
be kept in step with the physics state — a field forgotten there is silently lost across the reset.

**Risks.** The early watchdog introduces a new reset path during in-flight init. Bounded by
`RESUME_MAX_BOOTS = 3` and gated on `phase != 0`. Untested on hardware.

**Open, deliberately not changed.** `WATCHDOG_SECONDS = 4` exceeds the ~3.4 s fall from apogee, so a
lockup near apogee deploys at 0.0 m. Shortening it only helps if every blocking path — the SD log dumps
above all — feeds the watchdog, which is unverified; a spurious reset mid-dump would be a regression.
Recorded as FAILURE_MODES D6.

**Date.** 2026-08-09

---

## 2026-08-08 — Blind abort for total inertial loss; the Monte Carlo gate was measuring build quality, not firmware

### Decision 1 — a third abort path that does not read the IMU

**Decision.** Added `BLIND_ABORT_MS = 250`: if `F_IMU_BOTH_DEAD` stays latched for 250 ms during
powered flight, deploy recovery immediately, without waiting for a tilt or rate trigger.

**Reasoning.** The two existing aborts both read the IMU — the tilt abort uses the quaternion estimate,
the rate abort uses raw `imu_gx/imu_gy`. Losing every inertial source therefore silences *both*.
Measured with `--imustuck` at t=0.5 s: the vehicle reached **179.8° tilt and 800 °/s with zero aborts
raised**, and hit the ground with the chute packed. A detector that shares its only sensor with the
failure it is meant to catch is not a detector. Since we cannot see the tumble we assume it: control
is already inhibited by that point (gimbal neutral, integrators zeroed) and the airframe is
aerodynamically unstable without control, so the outcome is not in question — only whether the canopy
is out when it arrives.

**Measured after the change.** Chute fires 0.39 s after the sensors die in all three injection times
(t=0.5/1.5/2.5 s), at +5.9/+11.1/+16.1 m/s — still climbing, which is the correct trade. Zero false
positives across 480 dispersed healthy flights (a spurious fire deploys mid-boost and fails the flight;
the σ=0.5 Monte Carlo is 120/120 PASS).

**Alternatives.** (a) Wait for burnout and use the normal timed deploy — rejected: at a 0.5 s failure
the vehicle only reaches ~7 m, so there is no time to spend. (b) Key on `controlInhibited` — rejected:
keying on the fault bit means a future use of that flag for a *recoverable* condition can never
silently arm a parachute. (c) Accept the loss — rejected; this is precisely the "failures should cost
levels of safety, not the vehicle" requirement.

**Tradeoffs.** Deploys under thrust at up to ~16 m/s. Dynamic pressure there is ~150 Pa, low for this
canopy, and the alternative is an unrecoverable airframe.

**Risks.** A transient double-bus dropout longer than 250 ms would deploy a chute on an otherwise
healthy flight. Mitigated by the dwell and by the fact that `F_IMU_BOTH_DEAD` is only latched after
both `imuReadSource` calls have already failed. Not yet demonstrated on hardware.

### Decision 2 — split the Monte Carlo gate from the alignment spec

**Decision.** Section 7 of `regression.py` now gates the firmware at **σ=0.5°/axis** thrust
misalignment (lock: ≥98% PASS, ≤1% abort) and reports the σ=0.5/1.0/1.5/2.0 curve as *information*,
not as a pass/fail on the software.

**Reasoning.** The old single lock ("pass rate ≥ 90%" at σ=1.5°) was measuring the wrong thing.
Misalignment σ is a statement about how well the motor mount was built, so a badly-aligned build made
the suite print DO NOT FLASH at code that cannot fix it, while a well-aligned build would hide a real
firmware regression behind the easy margin. Splitting them means a failure in section 7 is
unambiguously the code's fault.

**Evidence.** Misalignment measured directly from flight logs (mean commanded TVC over the burn is the
misalignment being trimmed out): **ASC036 = 1.12°**, flew clean, gimbal never saturated; **ASC037 ≥
5.2°**, railed for 85% of the burn. The sim's sensitivity, 600 flights per row, brackets both:

| σ/axis | typical &#124;mis&#124; | PASS | ABORT |
|---|---|---|---|
| 0.25° | 0.31° | 100.0% | 0.0% |
| 0.50° | 0.63° | 100.0% | 0.0% |
| 1.00° | 1.25° | 94.8% | 2.0% |
| 1.50° | 1.88° | 83.8% | 8.2% |
| 2.00° | 2.51° | 75.0% | 18.2% |

**Alternatives.** Lower the threshold to 80% so the suite passes — rejected as calibrating the test to
the defect. Raise gimbal authority instead — rejected: the bench found 5.00° works and 5.25° binds, so
±5° is a hard mechanical limit.

**Why software cannot substitute.** Swept `I_GAIN` 0.0 → 2.5 over 600 flights: 0.20 → 0.80 buys **+0.8
points** fleet-wide and the >4° band is pinned at 50% regardless. The steady-state gimbal command
equals the misalignment whatever the gain, and biasing the servo centre to absorb 2° would need 7° of
throw the linkage does not have. **This failure mode is mechanical by construction.**

**Risks.** σ=0.5° is an *assumed* build standard, not yet a measured one — only ASC036 (1.12°) has a
trustworthy per-flight number. If real builds sit at 1.5°, fleet success is ~84% and the firmware gate
will still read 100%, which is correct but easy to misread as "the rocket is fine". The 7b table is
printed on every run to keep that visible.

**Date.** 2026-08-08

### Decision 3 — sim reports deploy time/altitude/velocity

**Decision.** `ascent_sim.cpp` now prints `p4t`, `p4alt`, `p4vz`; `p4t=-1` means the chute never fired.

**Reasoning.** "Did the chute come out" is not the same question as "did it come out in time" — ASC038's
canopy appeared ~1.5 m above the ground, which the old `p4descent` flag alone scores as a success. The
blind-abort work was undiagnosable without it. Consequence: the stale-EEPROM lock now excludes `p4t`
from its equality test, because it is an absolute sim-clock reading and a benign 0.40 s pad-time shift
(the cost of handling and clearing the record) is not a flight difference. `p4alt`/`p4vz` are compared
instead, since those are what a spurious resume would actually corrupt.

**Date.** 2026-08-08

---

## 2026-08-04 (pm) — ASC038 post-flight: SD writes out of the control loop, dt never silently skipped, estimator-independent abort — AND the SIL blind spot that let this fly

### The validation failure comes first, because it is the more important defect

**The SIL passed this build 8/8 and I reported it flight-ready. The vehicle then tumbled.** The harness
could not have caught it: its clock is synthetic — `Adafruit_MPU6050::getEvent()` calls `sim_advance(5)`,
so every loop iteration advanced exactly 5 ms — and the SD shim wrote to the host filesystem at **zero
simulated cost**. In simulation `dt` was always 5 ms, never exceeded the `dt<0.1f` guard in `sensors()`,
and `quatPropagate()` was therefore *always* called. The failure mode was **structurally impossible to
express**, so a PASS carried no information about it.

This is the same class of error as the pre-2026-07-07 harness that hardcoded `gyroZ=0` and was
"structurally blind to roll" — the blind spot that made ASC007's SIL validation worthless. I knew the
clock was synthetic and had written it down (when reporting that the SIL cannot measure loop period), but
treated it as a **measurement limitation** rather than a **validation blind spot**. Those are different
claims and conflating them is what produced a false ready-to-fly.

**Rule adopted:** before reporting any build flight-ready, state explicitly which failure modes the harness
is *structurally incapable* of expressing. A PASS is only evidence about modes the model can represent.

**Harness fix (`ascent_sim.cpp`, `shims/SD.h`):** SD open/close now charge simulated time via
`--sdlatency` (ms per `logData()` open+write+close), so the physics keeps running while the flight computer
is blocked in the SD library — as it really is. Regression, same airframe, latency the only variable:

| `--sdlatency` | outcome | boost tilt |
|---|---|---|
| 0 ms (the old, blind harness) | PASS | 1.51° |
| 60 ms | MARGINAL | 16.93° |
| **143 ms (measured on the flight card)** | **ABORT_TUMBLE** | **97.5°** |

The 143 ms run also reproduces the flight's signature: `loop_us_mean` 118 ms, attitude frozen, apogee
21.3 m (flight: mean 27.7 ms, frozen, 26.8 m). **The harness can now express the bug that flew.**

### Firmware fixes

- **SD access removed from the control loop** (the root cause). `logData()` used to `SD.open` + `println` +
  `close` every 50 ms; on the flight card that measured a **mean 143 ms, max 179 ms** — 40× the healthy
  3.5 ms loop. Rows now go to a RAM buffer (`FLOG_BUF_N=400`, ~25 kB) and the file is written once at
  recovery, exactly as `MTR###`/`CTL###` already did. Verified: with the fix the outcome is **identical
  (PASS 1.51°) at 0, 143, 300 and 600 ms** of card latency, with `dt_dropped=0` and the attitude live.
- **`dt` is never silently skipped again.** The old `if(dt>0 && dt<0.1f) quatPropagate(...)` substituted
  "no rotation happened" for "I don't know", with no indication anywhere. Now a long-but-real `dt` is
  **integrated** (graceful degradation), only an absurd `dt > DT_SANITY_S` (0.5 s) is dropped, and both
  cases are **counted and written to the CTL header** as `dt_long` / `dt_dropped`. Any nonzero
  `dt_dropped` should be treated as a failed flight for control purposes. The vertical Kalman got the same
  treatment — it froze identically on ASC038, which is why "use `estVz` for apogee" would *not* have helped.
- **Second, estimator-independent abort.** The 45° tilt abort reads `gyro_x`/`gyro_y`, so when the
  estimator froze it was blind: 248 °/s of real tumble read as 2.4° of tilt and it could not fire. Added
  `EMERGENCY_RATE_DPS=150` sustained `EMERGENCY_RATE_MS=300`, read straight off the **raw bias-removed
  gyro** — no quaternion, no EMA, no `dt` integration. Validated: **0 false positives in 150 healthy MC
  flights** (boost rate p90 27 °/s, max 46 °/s — 5.5× margin), fires on misalignment and roll-cant tumbles,
  and correctly does *not* fire on a hard 25 °/s tip-off.
- **Recovery beeps moved after the pyros.** `beep()` is a blocking `delay()`; three of them ran *before*
  `triggerPyro(P4)`, costing 300 ms (~3 m of altitude) at the worst possible moment. On ASC038 the deploy
  was already 0.87 s late from the `VEL_ALPHA=0.2` velocity filter and the canopy did not appear until
  ~1.5 m. Nothing may now sit between apogee detection and the pyro.
- **`ctlDt` widened `uint16` → `uint32`.** It saturated at 65535 µs and destroyed the per-sample evidence
  on exactly the 17% of iterations that mattered; only the header's uint32 min/mean/max survived. My error,
  documented at the time as covering a ">65 ms stall" that "should be pathological".

### Verification
Real Teensy 4.1 compile clean at `--warnings all --clean` (RAM1 174,592 B, 218 kB free — the +25 kB buffer).
SIL 8/8 PASS with numbers unchanged from before the fixes. Monte Carlo N=150 on the measured airframe:
**98.7% PASS / 0.7% MARGINAL / 0.7% ABORT**, boost tilt p50 3.20°, unchanged — the fixes cost nothing in
nominal performance. ASC log output verified: 16 columns, correct header, phases 1 and 2 present.

### Still open
`APOGEE_DROP_M` + the `VEL_ALPHA=0.2` EMA together cost 0.87 s of deploy delay. With the loop no longer
starved, `estVz` is live and is the better velocity source — but changing the apogee trigger is a separate,
testable change and was **not** bundled into this fix. The nichrome melt + canopy release time is
unmeasured and is the remaining unknown in the deploy chain; it should be bench-timed.
**Date:** 2026-08-04.

---

## 2026-08-04 — Ascent gains RE-POLED onto the measured plant; MAX_TILT lowered to clear the gimbal stop; linkage corrected 5.5 → 4.5

- **Decision.** `P_GAIN 0.249 → 0.491`, `D_GAIN 0.062 → 0.123`, `MAX_TILT 5.0 → 4.5`. Linkage constant
  corrected to **4.5** (the 5.5 measured 2026-08-03 was a mis-measurement), so commanded TVC degrees are now
  true gimbal degrees and no log rescaling is needed. Taken on Braxton's instruction to prioritise vehicle
  safety over approaching interesting regimes, now that the project is optimising for landing over research.
- **Reasoning — the retune is the CONSERVATIVE choice, which is not obvious.** The old gains were pole-placed
  for the destroyed airframe's keff = 257; the rebuilt vehicle measures keff = T·L/Iyy = 14.34·0.16/0.0176 =
  **130**, so they delivered ωₙ 5.70 / ζ 0.71 — sluggish and under-damped. Scaling both by 257/130 = 1.97
  restores the *original intended* ωₙ 8.0 / ζ 1.00 and preserves Kd/Kp at 0.25 (the researcher's 2026-08-03
  n=2400 replication: moving Kd independently of Kp is what manufactures the failure effect). Crucially,
  **steady lean = misalignment/P_GAIN**, so doubling P halves the lean (4.02° → 2.04° per degree of
  misalignment) and hands authority back — lean eating the gimbal is the ASC007 abort mechanism.
- **MAX_TILT 4.5 is about the servo, not the controller.** With `SERVO_*_MULT = 4.5` matching the physical
  linkage, `MAX_TILT 5.0` commands exactly the ±5.0° mechanical stop — zero tolerance margin. Measured in
  SIL via the new CTL instrumentation: in a tip-off + gust case the controller commands past its clamp for
  **~20% of the burn**, which at MAX_TILT 5.0 is 20% of the burn with the servo pinned on its stop drawing
  ~1–1.5 A — the exact rail sag suspected of browning out ASC036. At 4.5 the clamp holds 0.5° short of the
  stop: same control saturation, **zero mechanical stall exposure**.
- **Evidence.** Monte Carlo, N=120, measured airframe + the measured 262 ms excess ignition lag:
  **PASS 80.8% → 97.5%, MARGINAL 18.3% → 1.7%**, ABORT unchanged 0.8%; boost tilt p50 6.80° → 3.53°, p90
  12.69° → 6.44°. Misalignment tolerance improves (MARGINAL from 3° → PASS through 4°) while the abort
  threshold is **unchanged at ~5°** — the smaller clamp costs nothing because the stiffer loop keeps the
  vehicle upright enough to offset it. 8-case battery 8/8 PASS with every tilt lower than before.
- **Falsification attempts that the retune survived.** (a) *Delay* — the main theoretical risk of raising
  gains. Added `--servotau` to the harness (it was a fixed, unmeasured 0.03 s). The retuned gains are better
  at **every** lag out to 0.30 s, 10× the assumed value; no crossover. (b) *Slew* — better down to 70°/s;
  only below ~60°/s (a dying servo) do the higher gains lose. (c) *Gyro noise* — flat to 8 dps, 53× nominal,
  so the doubled D term does not amplify noise into the loop. (d) *Stop exposure* — measured, not assumed,
  and found essentially identical between gain sets (saturation is set by the disturbance, not the gain).
- **Alternatives considered.** Keep the flown gains (rejected: 18.3% MARGINAL on the real plant, and it
  leaves the vehicle under-damped on a rocket it was never tuned for). 1.4× intermediate (better than
  current, strictly worse than 2×). Lower `SERVO_*_MULT` instead of `MAX_TILT` to gain stop clearance
  (rejected: it desyncs commanded from true gimbal degrees and silently corrupts every logged TVC angle —
  Braxton's own call, and correct). Re-poling to a lower ωₙ (rejected: gives up the lean reduction that is
  the whole point).
- **Tradeoffs / risks.** These gains have **never flown**; the evidence is a harness now calibrated to
  measured mass, inertia, moment arm, linkage and ignition lag, but still a model. `SERVO_TAU` remains
  unmeasured — the robustness sweep is the mitigation, and `Bench_Latency.ino` should measure it before the
  next tuning decision. The vehicle will visibly react faster than on previous flights; that is intended.
  If it ever needs retuning again, **scale P and D by the same factor.**
- **Verification.** Real Teensy 4.1 compile clean at `--warnings all` (RAM1 145,184 B; 248 kB free). SIL
  8/8 PASS. Abort path still writes all three logs. Stale-EEPROM interlock still byte-identical to a clean
  run (no spurious deploy). CTL header reports the new gains and `max_tilt_deg=4.50`.
- **Date:** 2026-08-04.

---

## 2026-08-03 — Pre-flight build: TVC signs flipped from the bench, linkage ratio measured (5.5), burnout ramp, button-exit from terminal states, EEPROM brownout recovery

- **SERVO_*_SIGN +1 → −1 (BOTH axes), set from the bench pitch-the-nose test.** The bench is the only
  authority for these constants. The prior comment forbade flipping them and justified `+1` by citing
  flight ASC031 — that justification was removed as false (ASC031 flew a different IMU driver, see the
  2026-08-02 entry). **⚠ Unexplained and flagged in-code:** ASC036 (2026-07-20) flew `+1` on *this same*
  Adafruit driver and held attitude cleanly, so a **mechanical** change happened after ASC036 (linkage
  rebuilt, IMU remounted, or a servo re-horned). If the airframe was rebuilt, ASC036's measured 0.66°
  misalignment does not carry over — and that number is what separated its clean flight from ASC007's abort.
- **The SIL could not model the flip, and that was a latent defect worth more than the flip itself.**
  Flipping the firmware sign alone turned **all 8** dispersion cases into `ABORT_TUMBLE`. Cause: the
  harness had the old linkage sense baked in as an *unnamed* assumption inside `Mx = -T*L*tvcX`, so the
  firmware sign and the hardware linkage were not separable — any linkage rebuild silently invalidated
  every SIL result for the vehicle. Now an explicit `LINKAGE_SENSE` constant (`ascent_sim.cpp`). With
  `LINKAGE_SENSE=-1` + firmware `-1` the flips cancel and all 8 return to PASS. **Note this is CALIBRATED
  to the bench observation — a SIL PASS here is a consistency check, not independent confirmation.**
- **SERVO_*_MULT 5.0 → 5.5, MEASURED on the bench.** Resolves the long-standing 4.0-vs-5.0 discrepancy;
  the true value is *neither*. With 5.0 the firmware under-drove the gimbal ~9% (commanding 5° TVC wrote
  25 servo°, which a real 5.5:1 linkage turns into 4.55° of gimbal), so the pole-placed design was running
  ~9% below its intended keff. Bench sketches still carry `LINKAGE_SERVO_DEG_PER_GIMBAL_DEG = 4.0`, now
  known wrong — flagged in-code. **Revert both to 5.0 if the ±27.5° throw binds**; 5.0 is merely soft, not unsafe.
- **Burnout servo ramp (smoothstep, 1 s).** Was a hard `neutralServos()` step: both loaded servos slammed
  to centre simultaneously — the largest current transient of the flight, on the rail feeding the flight
  computer — while the nozzle still had residual pressure. Control authority is already gone at burnout, so
  easing costs nothing. Verified in SIL: TVC eases 1.82° → 0.00° over 1000 ms with the expected slow-fast-slow shape.
- **Button now exits every terminal `while(true)`** (abort, recovered, pad-abort, sensor-init fault). The
  only previous escape was a power cycle — i.e. unplugging a battery from a vehicle with possibly-live pyro
  channels. `exitOnButton()` consumes the press (so `buttonCount()` cannot see it as arming press #1, bounded
  at 5 s against a stuck button) and hard-disarms every pyro before returning. Re-arming still needs the full
  5-press + 30 s countdown, so **no interlock is weakened**. `emergency()` restarts via `SCB_AIRCR` rather than
  returning, because returning would drop back *into* the ascent loop and re-fire already-fired pyros.
  Sensor-init failure is now announced (2 blinks = BMP280, 3 = MPU6050) and the button retries — it can
  **never** continue into flight without a sensor.
- **EEPROM brownout recovery — recovery only, deliberately NOT TVC.** Persists the pad gyro-bias calibration
  and ground baro reference (the only two things a reboot cannot re-derive) plus a phase marker, written at
  phase EDGES only — never inside the control loop, since a Teensy EEPROM write is flash-backed and can stall
  for milliseconds. On boot, if the record says in-flight, the vehicle must *independently* corroborate flight
  before anything fires.
  - **Why not resume TVC:** attitude is not recoverable after an in-flight reboot, and that is physics, not a
    design choice. The quaternion is dead-reckoned from the pad and the reboot gap is unobserved; it cannot be
    re-levelled from the accelerometer because levelling needs gravity *alone* — under thrust the accel reads
    thrust+gravity (~3 g axial) and in coast the vehicle is in near free-fall (accel ≈ 0). A strapdown IMU
    cannot recover world-referenced attitude without an external reference and this vehicle has none.
    Resuming on a stale attitude would command corrections against an orientation the vehicle no longer has —
    actively driving a tumble, strictly worse than coasting. The chute is what saves the rocket.
  - **Anti-false-fire interlock** (the real risk is a pyro firing during handling): requires positive evidence
    of flight — free-fall/thrust (|a| < 0.55 g or > 2 g) or tumbling (> 90 °/s). Being carried reads ~1 g at low
    rate and is rejected. Plus: never deploys under thrust, a 3-reboot cap, and the record is cleared at
    deploy and at RECOVERED. **Tested in SIL** via a new `--seedphase` harness flag: a stale in-flight record
    with the vehicle at rest on the pad produces a byte-identical normal flight — no spurious deploy.
- **Diagnostics for the ASC036 mystery:** `SRC_SRSR` reset cause latched at boot and `sd_open_failures`
  counted (counted, not retried — a retry inside the control loop would lengthen the very stall that caused
  the failure). Both go in the `CTL###.CSV` header. Together with `loop_iters` these separate "the board
  rebooted" from "the board lived and only SD writes died" — the question ASC036 could not answer.
- **Validation.** Real Teensy 4.1 compile clean (RAM1 121,152 B; 272,064 free). Monte Carlo N=300:
  **89.3% PASS / 10.3% MARGINAL / 0.3% ABORT** — statistically identical to the pre-change 89.5/10.2/0.2, so
  robustness is preserved. End-of-boost lean p50 0.57°. Abort path emits all three logs.
- **Risks / accepted.** Ramp, button-exit and brownout recovery are **flown untested** — accepted by Braxton
  for this flight, on the basis that each fails toward "does nothing" rather than "does something wrong".
  The brownout recovery is a **mitigation, not a fix**: ASC036's root cause is still unknown, and if the
  power rail cannot sustain the servos, a reboot-and-deploy may simply brown out again.
- **Date:** 2026-08-03.

---

## 2026-08-02 — ASC007 diagnosis RECONCILED: estimator inversion is real but not sufficient; it multiplies with the trim/authority margin (partially supersedes 2026-07-07)

- **Context.** Two accounts of ASC007 coexisted in this repo: the 2026-07-07 entry below **falsified**
  roll-coupling and reframed the cause as *trim vs authority* (roll = symptom); the `Ascent_TVC.ino`
  header, written 2026-07-13, re-asserts that the naive integrator "inverted its own correction near
  ~180° roll and drove the divergence". No entry superseded the falsification, so the code and the
  decision record contradicted each other. This is now application-critical as well — the personal
  statement describes the estimator mechanism.
- **Method (falsification-first, no new flight).** ASC007 and ASC031 logged the filtered body rates and
  the integrated roll, so both estimators can be re-run offline on the *same* rate history. Validated
  before use: re-integrating the naive law reproduces the firmware's actually-logged output to **0.29°
  (ASC007) / 0.43° (ASC031) median**. **The quaternion replay is NOT ground truth** — both are
  dead-reckoned from the same 20 Hz filtered rates with no attitude reference. It shows what the two
  estimators *disagree about*, not what the vehicle did.
- **Finding 1 — the inversion is REAL** (the 07-07 falsification was too strong). On ASC007 the estimates
  separate as roll builds (|roll|=159°: naive 25.0° vs roll-aware 8.3°) and the **X channel disagrees in
  SIGN on 69% of rows above 120° roll**. This supplies the trigger the 07-07 entry explicitly admitted it
  could not reproduce — "hold ~16° for 1.4 s **then** diverge at 1.9 s". A constant trim gives a sharp
  hold-or-tumble threshold; an estimator that is correct at low roll and inverts past ~120° gives a
  *delayed* divergence. First saturation: t=1.95 s at |roll|=148.9°; **0 of 8 saturated rows below 140°**.
- **Finding 2 — but inversion is NOT sufficient; ASC031 is the control experiment.** Same naive
  estimator, rolled **further** (185.8° vs 172.6°), same inversion present (29% X sign disagreement) —
  and **survived with 0% saturation**. The discriminator is authority margin. Implied thrust
  misalignment, measured from the logs: **ASC007 4.28° → ASC031 2.52° → ASC036 0.66°** (first two as
  lean×P_GAIN; ASC036 from the I-term, which absorbs it directly).
- **Reconciliation.** Both mechanisms are real and they **multiply**: inversion + no margin → abort;
  inversion + margin → survives holding a lean; good alignment + roll-aware estimator + low roll → clean.
  Neither prior account is complete. Note also that alignment improved monotonically across all three
  flights — that is mechanical work, and it confounds any single-flight attribution to firmware.
- **Correction to a plausible reading of ASC036.** ASC036 does **not** demonstrate the quaternion fix: it
  peaked at **74.6° roll**, never entering the regime where the naive estimator inverts, and it changed
  three things at once (IMU driver + FSR, integral trim, quaternion). It is consistent with the fix; it
  is not evidence for it.
- **Open, and flagged rather than asserted.** ASC007's abort fires on `|GyroX|>45 or |GyroY|>45`; the
  naive estimate peaked at **46.7° (Y)** — barely over — while the roll-aware replay of the same flight
  peaks at **33.7°**. **The vehicle may never have reached the abort threshold.** Not provable from this
  data. `[CONFIRM]`.
- **Decision.** No code change. The gains, the estimator and the 45° threshold all stay as they are —
  this is a documentation and interpretation correction, and the current firmware (quaternion + I-trim)
  is the right build under *either* account. The discriminating test is a flight reaching **>140° roll
  with good alignment**; roll has been an uncommanded 75–195° on every flight, so it will arrive without
  being provoked. Do not induce roll deliberately.
- **Tradeoffs / risks.** The replay's authority is limited by 20 Hz filtered rates and a differentiated
  roll signal; it is strong on *sign disagreement* (a geometric effect) and weak on *absolute tilt*.
  Anyone citing the 33.7° figure must carry that caveat. **Files:** `FLIGHT_LOG.md` (full tables).
- **Date:** 2026-08-02.

---

## 2026-08-02 — Ascent_TVC: 500 Hz in-RAM control-loop capture (CTL###.CSV) + FLIGHT_LOG.md/.csv

- **Decision.** Add a second in-RAM high-rate buffer to `Ascent_TVC.ino`, mirroring the flight-proven
  `captureMotor()`/`dumpMotorLog()` pattern, capturing the CONTROL loop at 500 Hz during powered flight
  only and dumping `CTL###.CSV` at recovery. Per sample: `micros()` timestamp, **that iteration's loop
  period in µs**, tilt X/Y, **filtered** body rates (the D-term input), **raw** pre-EMA body rates,
  TVC command **pre-clamp**, and TVC command **post-clamp**. Optional A2/A3 servo feedback behind
  `#define CTL_SERVO_FEEDBACK`, default OFF. Also created `FLIGHT_LOG.md` + `FLIGHT_LOG.csv`.
- **Reasoning.** The existing SD log is one row per 50 ms (20 Hz). Three quantities the research
  depends on are single-digit-millisecond phenomena that 20 Hz aliases away: a servo limit cycle, the
  fraction of the burn spent rate-saturated, and the control loop's own transport delay. Specifically:
  - *Loop period* is the one term in Π = keff·τ² that has never been measured on this vehicle. It was
    an assumption (`SERVO_TAU=0.030` in `ascent_sim.cpp:61`, a plausible default). Since risk goes as
    τ², a 2× error in τ is a 4× error in the prediction. `Bench_Latency.ino` measures the *actuator*
    half of τ; `dt_us` measures the *compute* half. Together they close it.
  - *Raw AND filtered rates* — `ANGVEL_ALPHA` is itself an experimental variable. Logging both lets α
    be re-evaluated offline instead of spending a flight on it.
  - *Pre AND post clamp* — post = `constrain(pre, ±MAX_TILT)`; their difference IS saturation. Every
    log the project has ever recorded stores only the post-clamp value, so it can show *that* the
    gimbal was on its stop but never *how much* command was being discarded. LOG001 sat at the limit
    for 88.6% of its flight and the log cannot say whether it was asking for 5.1° or 30°.
- **Instrumentation-only guarantee, and how it was tested.** No gain, limit, sign, threshold, pyro or
  recovery logic was changed. The only edit inside the control law hoists the `constrain()` argument
  into a local `cmdX`/`cmdY` — same expression, same order of operations — which as a side effect
  evaluates the sum **once** instead of the 2–3 textual substitutions the `constrain()` **macro** was
  producing. Verified by paired SIL: baseline vs instrumented over **400 identical Monte-Carlo
  dispersion vectors → 0 outcome-line differences**, on a population that was 89.5% PASS / 10.2%
  MARGINAL / 0.2% ABORT (i.e. not degenerate — divergence had opportunity to appear), plus an 8-case
  battery and 3 edge cases, all byte-identical.
- **The risk that mattered: does capturing at 500 Hz lengthen the loop it exists to measure?**
  Evidence gathered, and its limits stated honestly:
  - The SIL **cannot** answer this — its clock is synthetic (`Adafruit_MPU6050.h` shim advances
    exactly 5 ms per `getEvent()`), so SIL `dt_us` is 5000 µs by construction. Reported as such.
  - **Static upper bound from the real Teensy binary:** `captureControl` compiles to **157
    instructions** and `micros()` to 31, both in **ITCM** (zero wait state), and every CTL buffer lands
    in **DTCM** (zero wait state) — so instruction count ≈ cycle count. ≤188 instructions + ~26 cycles
    for two `vdiv.f32` ⇒ **≲0.4 µs per iteration at 600 MHz**. This is a bound, not a measurement.
  - The loop is **I²C-bound**: `Wire.begin()` is left at the 100 kHz default, and one `getEvent()` +
    one `readAltitude()` is ~20 bytes of I²C ⇒ ~1–2 ms. So the predicted overhead is ~0.02–0.04%,
    roughly three orders of magnitude inside the ±15% acceptance threshold. **`Wire.setClock(400000)`
    was deliberately NOT added** — it would speed the loop up and confound the very baseline being
    established. Note it for later.
  - **The firmware now measures its own overhead.** `capture_overhead_us_per_iter` (DWT cycle counter
    on Teensy, `micros()` fallback on host) and `loop_us_min/median/mean/max` are written into the
    `CTL###.CSV` header, so one real run reports the true number instead of this estimate.
  - `#define CTL_CAPTURE 0` keeps the loop-period statistics but allocates nothing and stores nothing,
    so a 0→1 recompile is a paired A/B on identical hardware if the self-measurement is ever doubted.
- **Alternatives considered.** (a) *Log at 20 Hz and interpolate* — rejected, aliasing is not
  recoverable. (b) *Stream to SD in flight* — rejected, SD writes are milliseconds and would inject
  exactly the jitter being measured. (c) *Store iTerm as a 13th field* — rejected as redundant: it is
  recoverable exactly as `pre − P_GAIN·tilt − D_GAIN·rateFilt`, saving 16 kB. (d) *`millis()`
  timestamps* — rejected, 1 ms resolution on a 1–3 ms loop is 1–3 counts. (e) *DMAMEM/OCRAM for the
  buffers* — rejected, DTCM is faster and there is ample room.
- **Tradeoffs.** RAM1 variables 27,424 → **120,992 B (+93,568 B)**, leaving **304,992 B** free for
  locals; FLASH code +2,944 B. Median loop period resolves to ±50 µs (100 µs histogram bins); min/max/
  mean are exact to 1 µs. 2000 samples at 500 Hz = 4.0 s covers the 3.65 s powered phase with margin,
  and the buffer-full guard was tested by building at `CTL_BUF_N=100` (stopped cleanly at 100).
- **Also changed (deliberate, flagged).** The abort path now dumps both logs. `emergency()` previously
  ended in `while(true)` with both RAM buffers still in memory — so **every aborted flight destroyed its
  own control trace**, which is precisely the flight you need it from. Verified in SIL: on an
  `ABORT_TUMBLE` case the baseline binary produced only `ASC000.CSV` while the instrumented one produced
  `ASC000` + `MTR000` + `CTL000`, the last showing the controller demanding **27.96° of TVC against a
  5.00° stop, 42.9% of the burn saturated**. The dumps run *after* both pyros fire (no deploy delay) and
  tick `updatePyros()` every 64 rows so a long SD write cannot hold a melt wire past its 1000 ms pulse —
  strictly safer than the pre-existing `dumpMotorLog()` call on the recovery path, which had no such tick.
- **Risks.** (1) The Arduino IDE hoists auto-generated prototypes, which broke `struct KF` in commit
  `4613c7b`; `captureControl(float,float)` takes no struct, and the **real `arduino-cli` Teensy 4.1
  compile was run and passes clean with `--warnings all`** — this is no longer a theoretical check.
  (2) `CTL_SERVO_FEEDBACK` reads A2/A3, which are **not wired on the flight vehicle**; it is default OFF
  and `analogRead` on a floating pin logs noise. (3) The 2000-row dump is the longest SD write the
  firmware performs; mitigated by the `updatePyros()` tick. (4) **Reflash consequence:** per
  `Ascent_TVC.ino:63-65`, the correct `SERVO_*_SIGN` depends on both the linkage and the IMU driver's
  gyro convention, so the bench pitch-the-nose check MUST be re-run before flight. Surfaced to the user.
- **FLIGHT_LOG.md / .csv.** Six flight attempts reconstructed **from the log files themselves** (schema,
  FSR clipping, and a pure-PD residual test), not from prose. Two findings fell out that were not
  previously recorded: **(a)** ASC036's log stops at burnout with no coast/apogee/descent rows and no
  MTR file — the flight computer stopped logging while airborne under thrust; root cause unknown and
  this is the top pre-flight open item. **(b)** ASC031 flew the **pre-2026-07-07-pm** firmware, proven
  twice over — axial railed at exactly 2.000 g (±2 g FSR, i.e. the `tockn` driver) and the TVC command
  is pure PD to within ±0.1° (no integral trim). `Ascent_TVC.ino:57-62` cites ASC031 as validating
  `SERVO_*_SIGN=+1` for the current build; since that same comment states the correct sign depends on
  the driver's gyro convention, **ASC031 is not valid evidence for the current build — ASC036 is.** The
  code is unchanged; what its comment may claim is not. Flagged `[CONFIRM]` rather than edited.
- **Date:** 2026-08-02.

---

## 2026-08-01 — Impulse 2.2 + Sensor Board ordered at JLCPCB (PCBA both)

- **Decision:** Both boards ordered with assembly: Impulse 2.2 (4-layer JLC04161H-7628, 1oz outer/0.5oz inner, 36 BOM lines incl. THT wave-soldering of button/switches/buzzer/servo headers/Micro-Fits) and Sensor Board (converted to 4-layer same stackup, 11 BOM lines incl. genuine ICM-42688-P C1850418 and DPS310 C130156 bare LGAs).
- **Reasoning:** Sensor board went 4-layer after the 2-layer version proved unroutable-in-practice (IMU GND pads unreachable by pour vs. signals dropped by autorouter — a real dead-board bug caught by pad-cluster connectivity verification, not DRC counts). In1.Cu is a dedicated GND plane. 1oz outer verified adequate for Impulse via per-net per-layer current audit (sustained paths ≥1.2mm outer; inner runs transient-only).
- **Alternatives:** 2-layer sensor with hand-patching (diverged), 2oz copper (unnecessary cost), splitting orders (unnecessary once 4-layer routed clean).
- **Tradeoffs:** Teensy sockets NOT assembled (JLC lacks 1x24 female; butted 1x6 render showed edge overlap) — hand-solder Sullins/PJRC 1x24s with the socket-on-Teensy self-jig. Hand-solder list: Teensy(+sockets), PI/MI pigtails, U6/U7 modules, GPS module on sensor board.
- **Risks:** Part orientations converged via JLC preview against board silk/pad ground truth (per-part library zeros are unknowable offline); residual risk pinned on U2/DPS310 via order remark (pin-1 dot = TOP-LEFT, matching silk) after JLC's own library showed a 180° self-contradiction. Delivery inspection required: both LGA pin-1 dots vs silk dots BEFORE first power-up; R67 (pyro-arm pull-up) present; electrolytic stripes opposite silk "+". MEMS parts are the only practically non-reworkable orientation failures (heat-gun rework covers everything else).
- **Date:** 2026-08-01.


---

## 2026-07-27 — Impulse 2.2: CS_IMU pull-up added; pyro-gate series R considered & rejected

**Decision:** From a full two-board design review, added **R81 (10k) — CS_IMU pull-up to 3.3V**
(`CS_IMU`↔`3.3V`). CS_BARO already had a main-board pull-up (R70); CS_IMU did not — it relied solely on
the sensor board's R1 across the harness, so it floats at the Teensy if the ribbon is unseated (or during
the sensor-board power-up window). R81 restores symmetry and safe deselect.

**Considered and REJECTED — pyro-gate series resistors (R82–R85, 47Ω).** Briefly added, then removed.
Reasoning: the four pyro FETs already have gate pull-downs (R50–R53, 10k) — that is the fire-on-boot
safety and it is sufficient. Series gate resistors only limit GPIO gate-charge inrush and damp ringing,
which matter for FETs switching at high frequency millions of times (motor/buck), **not** for an igniter
FET that fires once per flight; they also slightly slow turn-on (undesirable for a pyro FET). Net benefit
here is marginal, so not worth the extra parts on a dense, hand-routed board. Reverted via `*.pre_gatefix`.

**NOT done — pyro rail bulk cap:** review flagged "no bulk on firing rail," but **C10 (220µF 25V) is
already on the switched `7.4V` rail** at the FET bank (it's a `Device:CP`, missed by a `Device:C`-only
scan). 220µF is adequate for ms e-match pulses; no cap added. Keep C10 near the FET bank when routing.

**Also corrected:** the "EN over-voltage" flag on U3 (TPS565208) was a FALSE ALARM — abs-max VIN,EN =
19V (DS verified), EN-to-VBAT is in spec. And `SENSOR_BOARD_DESIGN.md` GPS pin note fixed (pads 6/7/8,
not 11/12/13).

**Verification:** ERC unchanged from baseline (no new violations); schematic-parity 0; DRC +2 vs
baseline = R81's courtyard/silk overlap only (0 shorts, 0 mask bridges).
**Tradeoffs/Risks:** R81 is **staged at the bottom edge and needs final placement near the CS_IMU run**
during layout. Board remains unrouted. Backups: `*.pre_gatefix`. **Date:** 2026-07-27.

## 2026-07-25 — Impulse 2.2 fully rerouted on new placement; bottom kept minimal (motor-flame)
- **Context:** user unrouted the board, rearranged, and imposed a new constraint — **keep B.Cu (bottom)
  as bare as possible** because the motor exhaust can scorch bottom copper. Reserving the bottom entirely
  is infeasible at this density (3 signal layers → 76 unroutable). Compromise: route on 4 layers, then
  migrate B.Cu traces up to inner layers wherever geometry allows.
- **Key placement finding (localized the whole routing problem):** the *only* thing blocking a clean route
  was one over-dense pocket — the status-LED array (LED2/3/4 + R62/63/64) stacked against the bottom edge
  (each LED sitting on its resistor at the same XY) + D2/D3. freerouting shorted LED nets there and 8 nets
  couldn't escape. **User's fix: moved R62/63/64 to the back side (under the LEDs) + spread D2/D3.** That
  unstacking cleared it — freerouting went from 8 unrouted + 6 shorts/mask-bridges to **1 unrouted, ZERO
  DRC**.
- **B.Cu minimization is placement-bound:** with 35 bottom-side components (their pads on B.Cu), most
  bottom copper is *required* to feed them; the trace-migration pass (`_minimize_bcu.py`, guarded to never
  move a trace feeding a bottom-only pad) only moves a handful. Final B.Cu = 157 segments, mostly required.
- **Result: 811 tracks, 115 vias, 93 comps (58 top / 35 bottom), 0 unconnected.** GND poured on 4 layers,
  50 main-pour stitching vias, isolated pyro-cluster GND island connected. **5 residual DRC (all in the
  pyro cluster):** 3 GND vias inside the conservative 0.25mm rule (still > JLC 0.127mm fab min) + 2 cosmetic
  starved-thermals — trivial GUI nudges. Backup: `Impulse_2.2.kicad_pcb.routed_clean`.
- **Reusable tooling built:** general A* maze straggler router with via-escape at boxed endpoints
  (`_route_stragglers_maze.py`), B.Cu migration (`_minimize_bcu.py`), GND zone rebuild/stitch/island-bridge
  (`_make_gnd_zones.py`/`_stitch_gnd.py`/`_bridge_cluster.py`/`_fix_gnd_cluster.py`). Gotcha: pad zone
  connection setter is `SetLocalZoneConnection` (not `SetZoneConnection`), and it fails silently mid-script.
- **Date:** 2026-07-25.

---

## 2026-07-24 — Impulse 2.2 last-3 stragglers routed (GND-as-plane) + GND fully restored
- **Context:** freerouting plateaued at 3 unrouted nets regardless of layers/placement/optimization
  (VBAT U3→C2, Net-(U3-SW) U3→C4, PYRO_SW SW2→R68 56mm cross-board). User's key insight: *remove GND from
  routing, route the rest, add GND back as a plane*. Confirmed GND was never the blocker — with GND excluded
  (`-inc GND`, all 4 layers free) freerouting STILL left the exact same 3 (identical score).
- **Root-cause of the "impossible" stragglers (the real find):** a home-grown geometric router kept saying
  "no path" because it modelled every pad as a **circle of radius max(w,h)/2** — the L2 inductor pad became a
  2.55mm-radius circle that falsely blocked everything within 3mm; buck pads inflated the same way. Switching
  to **oriented-rectangle** pad clearance (transform segment into pad frame, distance to AABB, sampled)
  routed U3-SW on F.Cu and VBAT instantly. freerouting also fails these because it won't neck below class
  width (VBAT 1.5mm / U3-SW 1.0mm don't fit the buck); the geometric router uses sensible narrower widths on
  these short hops (`_route2.py`).
- **VBAT needed via-escape:** a via *at* U3.2 clips the In2 LED trace 0.42mm away, so the via is offset off
  the pad and reached by a short F.Cu stub. **PYRO_SW (56mm)** needed a real A* maze router on In1 (the GND
  pour is not an obstacle — it re-clears around any trace on refill), with a **line-of-sight-free collinear
  simplify** and a generous grid clearance (0.42mm) so the path keeps clear of In1 signals near SERVOX
  (`_route_pyro.py`).
- **GND restored (`_stitch_gnd.py` + `_bridge_gnd_pads.py`):** poured 4 layers, 44 main-pour stitching vias
  (disk fully inside the *main* island on all 4 layers so slivers auto-remove), then bridged 20/21 stranded
  SMD GND pads (local F.Cu pour was an isolated island) with a via into an inner-layer main pour + short
  stub — also good practice (a GND via by each decoupling cap). Bridge clearance 0.28mm (>GND class 0.25).
- **Cleaned freerouting's own mess:** deleted 17 degenerate <0.01mm stub segments (fixed a real SDA↔3.3V
  short at (159,67.3)).
- **Final state:** 729 tracks, 117 vias, **1 unrouted net** (C6.2 GND pad, boxed in the Teensy center — a
  15-second GUI via). 8 error-DRC left, ALL pre-existing freerouting: 5 SDA↔3.3V clearance (close, no short,
  at 159/67) + 3 copper-edge on Net-(P1O-Pin_1) (pyro drain near the circular edge). Every agent-introduced
  violation was resolved. Backup: `Impulse_2.2.kicad_pcb.stragglers_routed`.
- **Remaining GUI finish (all localized, safe in the editor, risky in script):** add 1 GND via by C6;
  nudge SDA or 3.3V apart at (159,67); pull the P1O pyro trace off the board edge; then Fill All Zones.
- **Date:** 2026-07-24.

---

## 2026-07-23 — Impulse 2.2 autorouted with current-based 2oz trace widths + GND planes
- **Decision (`_apply_netclasses.py`, freerouting 2.2.4; current-analysis workflow):** Route the board with
  per-net trace widths sized to each net's worst-case current on 2oz copper, GND as a plane, via freerouting.
- **Netclasses (IPC-2221, 2oz/70µm, 10°C rise, generous per "as wide as space allows"):** PYRO_RAIL 2.5mm
  (7.4V/7.4V_RAW, 4A/ch), PYRO_DRAIN 2.3mm (Net-(PnO-Pin_1), 2-4A+inrush), MAIN_PWR 1.5mm (VBAT, 4A),
  SERVO_PWR 1.0mm (5V-DIRTY, 3A=U3 limit), RAIL_5V 0.6mm, RAIL_3V3 0.4mm, SIGNAL 0.25mm (66 nets, thin to
  fit the Teensy), GND = poured plane both layers (not a routed width; ~7A/15A-peak return).
- **Stackup:** set F.Cu/B.Cu to 0.070mm (2oz) — the file was 1oz. **MUST order 2oz outer copper from the
  fab** or every power width is ~2× undersized.
- **Toolchain:** freerouting 2.2.4 needs **Java 25** (class 69.0 — the "runs on 21" advice was wrong; JRE 21
  = class 65 → UnsupportedClassVersionError). Portable Temurin JRE 25 + the bare jar, run from a SPACELESS
  dir (freerouting's arg parser splits the DSN path on spaces). Pipeline: pcbnew ExportSpecctraDSN → `java
  -jar freerouting-2.2.4.jar -de board.dsn -do board.ses -mp 100 -mt 0 -da -inc GND` → ImportSpecctraSES →
  ZONE_FILLER. GND excluded from the router (-inc GND) and poured instead.
- **Result (two passes):** PASS 1 at the wide widths above → 192/209 routed but the whole high-current
  backbone (17 conns: VBAT ×5, pyro rails/drains, servo) would NOT fit at full width on 2 layers.
  **Insight (user): the wide traces were dragging out to µA loads** — continuity LEDs (R37 on the 2.3mm
  pyro-drain net), the armed LED (R63 on 2.5mm 7.4V), sense taps — because a netclass sets ONE width per
  whole net. PASS 2: right-sized power to ~current (PYRO 1.2, MAIN 1.2, SERVO 0.9mm — all still ≥10°C-rise
  requirement; a 4A firing pull over a short run only needs ~1mm on 2oz), re-routed → **~621 tracks / 47
  vias, ~20 non-GND unrouted (VBAT, 7.4V, pyro drains, servos + a few congested signals) + ~14 GND-plane
  fragments** needing stitching vias. GND poured both layers. Grid stitching-via attempt REVERTED (crude
  placement clashed with holes/edge → use KiCad's Route→Add Stitching Vias tool instead). Other DRC
  (courtyards 33, lib-mismatch 15, silk 16, pth-inside-courtyard 14) all PRE-EXISTING baseline.
- **Conclusion: this board is too dense to 100% auto-route on 2 layers.** ~90% routes with correct
  current-based widths + GND planes; completing it needs hand-routing the ~20 congestion stragglers +
  proper GND stitching, OR moving to 4 layers (dedicated power/GND planes = clean full auto-route).

## 2026-07-24 — Impulse 2.2 converted to 4 layers (resolves the 2-layer congestion)
- **Decision (user):** go 4-layer. `SetCopperLayerCount(4)` → F.Cu (signal), **In1.Cu (reserved GND plane)**,
  In2.Cu (signal/power), B.Cu (signal). Reserve In1.Cu by editing the exported DSN's In1.Cu layer
  `(type signal)`→`(type power)` so freerouting won't route signals on it (routes on F.Cu/In2.Cu/B.Cu).
- **Result:** freerouting **5 unrouted** (from ~17-20 on 2 layers): 2 pyro drains, VBAT, 5V-DIRTY, U3-SW.
  Poured GND on all 4 layers (In1.Cu = solid ~3013mm² plane) + **34 collision-checked stitching vias**
  (inside all 4 GND fills, cleared of every hole/pad/trace; 4 that clipped clearance were auto-removed by
  matching DRC coords). **Final: 7 unconnected** (5 power stragglers + 2 GND pads), **0 clearance/short**
  violations; remaining DRC (courtyards, lib-mismatch, silk, pth-inside-courtyard) all PRE-EXISTING baseline.
  Widths kept right-sized (PYRO/MAIN 1.2, SERVO 0.9mm); outer copper 2oz.
- **Refinements (2026-07-24):** (a) freerouting re-run WITH optimization (`-mt 4 -oit 1.0`, ripup-reroute)
  + power right-sized to the exact 10°C requirement (PYRO/MAIN 1.0, SERVO 0.8mm) → down to ~5 real
  stragglers. (b) **LED/continuity/sense taps narrowed to 0.25mm** via a graph-walk from each low-current
  resistor (R37/41/42/43 continuity, R63 armed-LED, R58 sense) that STOPS at the FET/connector pad, so the
  firing paths stay 1.0mm (verified: Q16/17/18 drains still 1.0mm at pad). Math: continuity 0.65mA and LED
  19mA both need <0.01mm on 2oz, so 0.25mm is 80-1200x margin — the taps were only wide because a netclass
  is one-width-per-net. 77 tap segments narrowed.
- **FINAL (all-4-layer route, GND poured not reserved):** routing on ALL 4 layers (In1.Cu NOT reserved) +
  optimization + GND-included got freerouting to **2 unrouted of 153** (151 routed). Both are switching-
  regulator LOOP traces wedged in the tight buck clusters: **VBAT C9↔U2 (buck input loop)** and **Net-(U3-SW)
  U3↔C4 (buck switch node)**. 3 scripted direct-route attempts (In1.Cu straight, per-layer, 0.15mm-clearance)
  all produced shorts — the buck areas are physically SATURATED (U3-SW neighbors C2/C4/U3; VBAT neighbors
  C3/C9/U2). **This is a PLACEMENT-density limit, not a layer limit** — 4 layers is plenty everywhere else.
  These 2 are also exactly the loops you route deliberately for a buck (perf-critical), so autorouting them
  arbitrarily would be worse. Genuine 100% needs nudging a buck passive to open a channel (placement change).
- **State: ~99% routed, 709 tracks, 4-layer GND planes (circular, solid pad connect), LED/continuity/sense
  taps at 0.25mm (firing paths 1.0mm), 0 shorts/clearance/crossing.** 6 cosmetic zone-island artifacts in the
  unconnected count (self-referencing edge points, not pad gaps). Route saved in `Impulse_2.2.ses`.
- **Fab:** finalize the 4-layer physical stackup in Board Setup (outer=2oz set; confirm inner In1/In2) and
  **order 4-layer with 2oz outer copper**. Backups: `Impulse_2.2.kicad_{pcb,pro}.pre_4layer`. **Date:** 2026-07-24.
- **Key finding — freerouting cannot "maximize" width:** it routes at fixed netclass widths and never
  grows-to-fit or necks below class width; "as wide as space allows" = generous fixed widths + manual local
  widening. On 2 layers, full-width power + signals do not co-fit; completing the backbone needs either
  narrower (still-adequate: MAIN_PWR 1.0 / PYRO 1.5mm = 20°C rise) widths, hand-routing, or 4 layers.
- **Risks / open:** power backbone unrouted (board not yet manufacturable); 2oz order dependency; autoroute
  is machine-quality not hand-optimized. Pre-op backups: `Impulse_2.2.kicad_{pcb,sch,pro}.pre_route`.
  **Date:** 2026-07-23.

## 2026-07-23 — Pyro firing FETs Q16-Q19: IRLML6344 (SOT-23) → TI CSD17313Q2 (SON-6), all 4 channels
- **Decision (Impulse 2.2, `_swap_pyro_fets.py` + `_swap_pcb_fet.py`; research+adversarial-verify workflows):**
  Swap all four pyro firing FETs from **IRLML6344 (SOT-23)** to **TI CSD17313Q2** (LCSC **C2863837**,
  `Package_SON:Texas_DQK`, SON-6 exposed-pad, 30 V). Firmware pulse widths UNCHANGED: all channels stay
  900 ms — the LEGS nichrome band cutter is **bench-verified to cut at 900 ms** (an initial 3 s change was
  reverted per that test). Note 900 ms of ~2–4 A is still a sustained load vs an e-match's ms, so the
  exposed-pad FET upgrade still applies, just with margin.
- **Reasoning:** The LEGS channel does not fire an e-match — it heats a **nichrome hot-wire band cutter**
  to release the landing legs. Unlike an e-match (bridgewire opens in ~1–10 ms → the FET sees a negligible
  pulse), a hot-wire is a **non-clearing resistive load held ON for seconds** (~2–4 A for the whole cut).
  That moves the binding constraint from pulse-SOA (which SOT-23 aces) to **sustained thermal dissipation**
  (which a bare SOT-23 does not). CSD17313Q2: RDS(on) **42 mΩ max @ VGS = 3 V** — the only exposed-pad
  power FET on the JLC/LCSC catalog whose on-resistance is *datasheet-guaranteed at ≤3.3 V* (critical: the
  Teensy drives the gate directly, no gate driver), VGS(th) 1.8 V max, RθJC 7.4 °C/W. All four channels
  made identical so ANY can drive the nichrome.
- **Alternatives:** (a) Keep IRLML6344 + size the wire to 2–4 Ω + copper pour — electrically sufficient for
  a *controlled* wire, rejected because the user wanted margin for any channel/any wire. (b) Lower-Rds
  CSD17577Q3A — better thermally but only spec'd ≥4.5 V, so "on by physics, not datasheet" at 3.3 V.
  (c) **Every other JLC exposed-pad "logic-level" power FET (AON7426, SiS412DN, NTMFS4C302N, NCE3065Q…) is
  a trap: VGS(th) max 2.2–2.5 V, RDS(on) spec'd only at 4.5 V → can sit half-on at a 3.3 V gate, the
  catastrophic failure mode for a pyro switch.** (d) AO3400A drop-in (JLC Basic, genuinely logic-level) —
  the no-footprint-change option, but SOT-23 thermal, no exposed pad.
- **Verified:** old IRLML6244 symbol G/S/D pin endpoints are IDENTICAL to the CSD16301Q2/CSD17313Q2 symbol
  (G=(-5.08,0), S=(2.54,-5.08), D=(2.54,5.08)) → schematic wiring preserved by construction. Pad map on
  Texas_DQK: gate=pad 3, source=pads 4/7, drain=pads 1/2/5/6/8 + exposed pad. Datasheet 3 V RDS(on) row
  confirmed against `csd17313q2.pdf` (the "optimized for 5 V" bullet is marketing).
- **Tradeoffs / status:** CSD17313Q2 is a JLC **Extended** part (~$3 feeder fee, ~500 pcs stock — verify at
  order time). **The board is NOT yet manufacturable:** the swap is electrically correct (schematic ERC:
  66→60 msgs, the swap *removed* 6 warnings; the 1 pre-existing error is an unrelated #FLG013/#FLG015
  PWR_FLAG conflict; PCB pad→net verified by pcbnew introspection) but the larger Texas_DQK footprints
  overlap the old SOT-23 track stubs → **28/29 new DRC violations (15+ drain↔GND shorts, 40 fine-pitch
  solder-mask bridges, 6 clearance) are localized to the 4-FET row and need manual re-routing.** No copper
  zones exist on the board (0 zones), so a refill won't help — the old FET track stubs must be deleted and
  the new pads routed.
- **REMAINING MANUAL STEPS (in KiCad, mid-layout):** (1) delete the orphaned pyro-FET track stubs and route
  the new pads; (2) pour/route generous **drain copper + thermal vias** under each exposed pad (drain net,
  per-channel); (3) the sustained ~2–4 A legs current also flows through the **arm P-FET Q21 (AO3401A,
  ~0.5–0.9 W in SOT-23 at 4 A for 3 s — borderline)** and the **1.0 mm pyro rail trace necks** (sized for
  "brief e-match pulses"): widen the legs-carrying rail/drain traces to ≥1.5–2 mm and consider Q21 margin;
  (4) keep the nichrome sized so the 900 ms cut current stays ~2–4 A (matches the bench-verified cutter).
- **Risks:** Re-route is manual and unverified until done; leadless SON-6 is machine-place only (fine — JLC
  assembly). Pre-swap backups kept: `Impulse_2.2.kicad_{sch,pcb}.pre_pyrofet`. **Date:** 2026-07-23.

## 2026-07-23 — Soft power switching on BOTH batteries, buzzer driver, CSB strap (research-verified)
- **Decision (all on Impulse 2.2, `_add_power_fets.py`; 10-agent research+adversarial-verify workflow):**
  1. **Q20/Q21 = AO3401A high-side P-FETs** switch the main and pyro batteries. Gate network per
     rail: 100k gate-source pull-up (default OFF — safe at battery hot-plug), 10k series to a slide
     switch to GND (~85µA through the switch, curing the 25mA SW1 rating problem), 100nF G-S.
     SW1 = main PWR; **SW2 (new) = pyro ARM** — the pyro rail finally has an arm switch instead of
     "plugging the battery is arming". AO3401A chosen over IRLML6402 after adversarial datasheet
     review: −3.2A@70°C continuous (servo stall), IDM −27A (e-match pulses), Vgs ±12V ≥ the 7.6V
     max applied by the 100k/10k divider. Gate-DRAIN Miller soft-start was REJECTED: it momentarily
     energizes both rails at battery hot-plug (verifier-caught pyro hazard).
  2. **Q22 = IRLML6344 low-side buzzer driver** from 5V-CLEAN with SS14 flyback (D4) + 10k gate
     pulldown (R69). Buzzer BOM: **Soberton GT-0905A passive magnetic transducer** (9mm/4.0mm pitch
     — drops into the existing footprint; 85dB@10cm; 2730Hz resonance ≈ the firmware's tones; 30Ω,
     needs the driver — direct pin-13 drive would overstress the Teensy pad). Verified terminology:
     ACTIVE buzzers cannot change pitch; firmware melodies (tone()) require this passive part.
  3. **U7 (BMP280 module) CSB strapped to 3.3V** (was floating): Bosch ds sec 5.1 — any transient
     low on CSB latches SPI mode until POR; hard strap removes the failure mode. SDO-to-GND (0x76)
     confirmed correct and protects the ascent firmware, which probes 0x76 ONLY.
- **Research also confirmed:** GY-521 fine as-is (onboard 4.7k AD0 pulldown → 0x68 = firmware
  default; onboard 2.2k pullups stack with R60/R61 to ~1.5k → OK at the firmware's 400kHz).
  Firmware pre-flight to-dos found by audit: landing firmware still uses **Serial1 for GPS (board
  is Serial4)** and its servo pins (2=roll) collide with INT_IMU on this board's pin map — remap
  before flying 2.2; ascent sensor-init failure is a silent hang (no beep) vs landing's alarm.
- **Verification:** ERC 64 (baseline + 2 known lib-mismatch), DRC 92 with zero non-baseline items,
  schematic-parity 0. New copper: PI→Q21.S / Q21.D→C10 re-splice (1mm necks). Preview
  `_preview_power_switching.png`. BOM adds: 2× AO3401A, 1× IRLML6344, SS14, GT-0905A, 2×100k,
  3×10k, 2×100n, 1× DSWB01LHGET. **Date:** 2026-07-23.

## 2026-07-22 — Pre-fab design review of BOTH boards (netlist-level, against datasheets)
- **Verified sound:** AP63205 (5V/2A → Teensy via SS14) & AP63305 (5V/3A → servos) per datasheet —
  pinout, EN, BST 100nF, L (6.8µH/4.7µH within 2.2-10µH rec.), 22µF×2 out-caps, bulk (68µF VBAT,
  470µF servo, 220µF pyro). All three ADC dividers land ≤2.7V at 8.4V (Teensy 3.3V ADC safe). Pyro
  chain: 10k gate pulldowns (FETs off at boot), IRLML6344 logic-level @3.3V, ~0.55mA continuity
  current (≫ safety margin below no-fire). I²C pullups 4.7k on main only, per plan. Teensy pin map
  valid: SPI1 (1/26/27), Serial4 (16/17, TX/RX correctly crossed), PWM on all 7 servo pins, A7-A9
  senses. Sensor board: ICM-42688 + DPS310 pinouts re-checked (match datasheets), decoupling per
  spec, ferrite-isolated 3V3, CS pull-up. Both boards: ERC/parity clean; ERC/DRC baselines triaged —
  no electrical items, all cosmetic/documented classes.
- **Findings:** (1) BLOCKER — both boards UNROUTED (153/42 airwires): route + GND zones + copper DRC
  before ordering. (2) SW1 (DSWB01LHGET) rated 24V/25mA but carries the full main-battery current
  incl. the 3A servo buck — replace with a high-current switch or switch-drives-P-FET arrangement.
  (3) BUZZER1 is direct on Teensy pin 13 — part MUST be a passive piezo element (~2mA), not an
  active/magnetic buzzer. (4) Dual-servo stall can transiently exceed AP63305's 3A limit — 5V-DIRTY
  droops but Teensy is isolated on its own regulator (good architecture); bench-test stall. (5) No
  reverse-polarity protection (accepted 2026-07-20) — procedural multimeter check stands.
  **Date:** 2026-07-22.

## 2026-07-22 — Silkscreen cleanup: only essentials stay; small-part refs moved to fab layer
- **Decision:** Moved all small-part reference designators (R*, C*, D*, L*, Q*, U1-U3, H1-H4, LED1;
  50 refs on Impulse 2.2, 16 on Sensor Board) from silkscreen to the fab layer. Silkscreen keeps the
  operational essentials only: connector names (PI/MI/J_SENSOR1/servo X-D/pyro P#O, J1/J_GPS),
  SW1/BUTTON1/BUZZER1, P1-P4 check-light labels, M/P/S status labels (user-shortened from
  MAIN/PYRO/SENS), +/- polarity marks, and the U6/U7 module drawings. Preview `_preview_clean_silk.png`.
- **Also this session (user hand-edits, reconciled):** LED status circuits were redrawn/repositioned
  in eeschema with wires/pin-to-pin connections replacing the generated labels — PCB pad nets renamed
  to the schematic's names (Net-(LED2-A), Net-(D2-K), etc.); on the sensor board a `LED1_A` global
  label was added at the R2/LED1 pin junction (parity checker can't resolve bare pin-on-pin contact).
- **Result:** Impulse DRC violations 162 → 78 (silk noise gone), ERC 62 (baseline), parity 0.
  Sensor: ERC 0, DRC 3 (baseline), parity 0. Refs remain available in fab-layer plots for assembly
  documentation. **Date:** 2026-07-22.

## 2026-07-21 — Pre-launch status LEDs: battery-charge indicators + sensor-cable detect
- **Decision:** Extend the existing P1-P4 red pyro-continuity LED row on Impulse 2.2 with three
  indicators (`_add_status_leds.py`, preview `_preview_status_leds.png`):
  - **MAIN** (green LED2): VBAT → R62 330Ω → LED → D2 BZT52C5V6 5.6V zener → GND. Turn-on at
    ~7.7V = ~50% SoC on a 2S LiPo (3.85 V/cell); ~2 mA at 8.4 V (100%). Brightness ≈ charge.
  - **PYRO** (yellow LED3): same circuit on the 7.4V pyro rail (R63/D3).
  - **SENS** (blue LED4): 3.3V → R64 470Ω → LED → `SENS_DET` = J_SENSOR1 pad 12. Pad 12 (was the
    2nd GND) is grounded on the SENSOR side only, so the LED lights iff the JST cable is seated.
  Sensor board gets LED1 (blue, +3V3_RAW → R2 470Ω → GND): board-is-powered indicator.
- **Reasoning:** Zener-in-series is the simplest analog "off at 50%, full at 100%" indicator —
  Vz(5.6) + Vf(≈2.1) ≈ the 2S 50%-SoC voltage; current (and brightness) then scales ~linearly with
  voltage over the top half of the charge range. Caveat: zener knee is soft, so expect a faint glow
  slightly below threshold. Cable-detect uses the ground-loop trick (like SD card-detect pins),
  trading the redundant 2nd ground wire for a true connection check.
- **Colors:** red = pyro continuity (existing), yellow = pyro battery, green = main battery,
  blue = sensor board power + cable detect. Blues on 3.3V run at low margin (Vf 2.7-3.0) — softer
  glow, 470Ω chosen accordingly.
- **Risks:** Continuity/battery LEDs draw ~2 mA each whenever their battery is connected. Verified:
  ERC/parity at baseline both boards; DRC below pre-change count (new refs on F.Fab). **Date:** 2026-07-21.

## 2026-07-21 — Sensor interface connectors go VERTICAL (BM12B-GHS-TBT) on both boards
- **Decision:** Swap J_SENSOR1 (Impulse 2.2) and J1 (Sensor Board) from side-entry JST GH
  (SM12B-GHS-TB) to top-entry vertical (BM12B-GHS-TBT). Same series/pitch/latch — the GHR-12V-S
  harness is unchanged; pin order (see entry below) and pin-1 direction preserved; done via pcbnew
  API footprint replacement carrying position/rotation/nets/sheet-link.
- **Reasoning:** The sensor module stacks above the main board, so a straight-up cable with no
  90° bends beats side exit; cable tension under vibration then pulls along the mating axis, the
  direction the GH latch is designed to resist.
- **Tradeoffs:** Vertical body is slightly taller; pad rows sit ~3-4 mm from the old positions
  (unrouted boards, no copper to move). BOM: order BM12B-GHS-TBT instead of SM12B-GHS-TB.
- **Risks:** None found — ERC/DRC/parity identical to baseline on both projects after the swap.
  **Date:** 2026-07-21.

## 2026-07-21 — Sensor interface re-pinned on BOTH boards (J_SENSOR1 ↔ J1, still straight-through)
- **Decision:** New 12-pin order: `1 3V3(+3V3_RAW), 2 GND, 3 SCK, 4 MOSI, 5 CS_BARO, 6 GPS_TX,
  7 GPS_RX, 8 GPS_PPS, 9 INT_IMU, 10 MISO, 11 CS_IMU, 12 GND` — applied identically to Impulse 2.2
  `J_SENSOR1` and Sensor Board `J1` (sch + pcb, `_repin_sensor_if.py`), harness stays pad N ↔ pad N.
  Power kept on the edge pair (pins 1-2) per normal convention; only signals 3-11 are order-optimized.
- **Reasoning:** Old order made the connector's middle pads (CS_IMU/MISO/INT_IMU) reach the Teensy's
  far-east pins while GPS pads reached mid-board — a self-crossing fan-out. New order sorts pads by
  Teensy destination west→east: zero crossings on Impulse; on the sensor board GPS_TX/RX/PPS now land
  nearly straight under J_GPS (SPI picks up 1-2 easy crossings on a bare 2-layer board). Cable keeps
  GND beside SCK and a GND at the far end.
- **Alternatives:** Keep order and eat the crossings (both boards are unrouted, so re-pinning is free
  now and impossible after boards are ordered).
- **Risks:** Any already-built harness/boards from the OLD pinout would mis-wire power onto signal
  pins — but nothing has been fabbed with the 12-pin GH interface yet. Verified: ERC/DRC/parity at
  baseline on both projects (Impulse 62/0 parity; Sensor 0 ERC/0 parity). **Date:** 2026-07-21.

## 2026-07-21 — Impulse 2.2: backup sensor modules (U6 MPU6050 / U7 BMP280) live on the BOTTOM as movable units
- **Decision:** U6 (GY-521/MPU6050 socket) and U7 (GY-BMP280 socket) were flipped to B.Cu (user, in
  pcbnew). Each footprint now carries its full module drawing INSIDE the footprint on B.SilkS
  (`_add_bottom_silk.py`, preview `_preview_bottom_sensors.png`): physical module outline (GY-521
  21.2×16.4 mm, GY-BMP280 15×11.5 mm, bodies extending away from each other), per-hole pin labels
  (U6: VCC/GND/SCL/SDA/XDA/XCL/AD0/INT; U7: VCC/GND/SCL/SDA/CSB/SDO; VCC = square pad), and the
  module name + ref inside the outline. Everything is footprint-local, so selecting U6 or U7 in
  pcbnew moves/rotates the entire module drawing as one unit.
- **Reasoning:** Footprint-internal graphics beat a KiCad group: they survive move, rotate, and
  further flips, and the outline shows the real board area each module will occupy under the board.
  Labels are mirrored to read from the assembly-side view and sit INSIDE the module outline (body
  side of the pin row) — read them before plugging the module in, since the module covers them.
- **Tradeoffs / current state:** At today's positions the outlines still cross the board edge and
  some Teensy/pyro through-holes (a handful of position-dependent silk DRC warnings) — expected,
  since the whole point is to drag the modules to their final spots; warnings clear on placement.
- **Risks:** If a module ends up chip-DOWN instead of chip-up, sensors work electrically but the
  MPU6050's axes flip vs firmware sign conventions — bench-verify axis signs after mounting.
  **Date:** 2026-07-21.

## 2026-07-20 — REVERTED: reverse-polarity P-FETs removed (supersedes entry below)
- **Decision:** Remove the Q20/Q21 IRLML6402 reverse-polarity FETs added earlier today; files restored to
  the pre-change state. Kept: `+`/`-` polarity silkscreen on BOTH battery inputs (`PI` and `MI`) and their
  nudged reference labels.
- **Reasoning:** User decision — connectors are soldered once and checked at the bench, so careful assembly
  plus the silkscreen marks are judged sufficient; no per-flight plugging error to guard against on the
  board side.
- **Tradeoffs:** The hazard identified below still exists if a battery pigtail is ever soldered reversed:
  a reversed PYRO battery can push current through the e-matches via the pyro NFET body diodes. Mitigation
  is now procedural (multimeter check of pigtail polarity against the silkscreen before first power-up).
- **Risks:** Assembly-error dependent. ERC/DRC/parity verified back at baseline (62/165/0). **Date:** 2026-07-20.

## 2026-07-20 — Impulse 2.2: reverse-polarity protection P-FETs on BOTH LiPo inputs (REVERTED, see above)
- **Decision:** Add high-side P-channel MOSFETs (IRLML6402, SOT-23, -20V/-3.7A/65mΩ) in series with both
  battery inputs: `Q20` on `PI` (pyro battery: new net `7.4V_RAW` → FET → `7.4V` rail) and `Q21` after `SW1`
  on the main battery (`VBAT_RAW` → FET → `VBAT`). Drain to battery, source to load, gate to GND — the
  classic zero-drop reverse-battery circuit (body diode conducts at plug-in, channel then shorts it out).
- **Reasoning:** Neither input had any protection. Worse than dead regulators: a REVERSED pyro battery
  drives current through the e-matches via the pyro NFETs' (Q16-Q19) body diodes — a plausible
  accidental-ignition path. 2S Vgs (-8.4V) is within the 6402's ±12V rating; ~50mΩ drops only ~0.1V at 2A.
- **Alternatives:** Series Schottky (0.3-0.4V loss, heat at servo current); ideal-diode controller IC
  (overkill); fuse+TVS crowbar (sacrificial). P-FET is the standard low-cost choice.
- **Tradeoffs:** Two more SOT-23s; pyro input copper re-spliced through Q20 (1.0mm necks in the 2.3mm bus,
  fine for brief e-match pulses); Q21 unrouted like the rest of the VBAT path (board is mid-layout).
- **Risks:** None found: ERC/DRC/parity all net-zero vs baseline (62/165/0). Implemented by
  `_add_revpol.py` (text surgery, deterministic UUIDs). Previews: `_preview_batt_input.png`,
  `_preview_batt_main_input.png`. **Date:** 2026-07-20.

## 2026-07-20 — Impulse 2.2 battery input stays THT pads + RCY pigtail (no board-mount JST)
- **Decision:** Keep `PI` (the 7.4V battery input) as the existing 1x02 through-hole footprint; connect the
  LiPo by soldering a male JST-RCY pigtail into the pads. Added `+`/`-` silkscreen polarity marks above/below
  the pads (`_preview_batt_input.png`).
- **Reasoning:** The "red JST" on 2S LiPos is the JST RCY series, which is **wire-to-wire only** — JST makes
  no PCB-mount mate, and KiCad ships no RCY footprint. A soldered male pigtail is the standard hobby solution
  and mates directly with the battery.
- **Alternatives:** Third-party board-mount "RCY" headers (uncontrolled dimensions/sourcing); switch to a
  genuine board-mount family like JST-XH (battery lead wouldn't mate without re-terminating).
- **Tradeoffs:** Pigtail wires need strain relief; no keyed board-mount connector.
- **Risks:** Reverse-polarity soldering of the pigtail — mitigated by the new silkscreen marks (square pad +
  `+` = red wire). **Date:** 2026-07-20.

## 2026-07-11 — PCB gap analysis for the LANDING: margin-fin servo is REQUIRED, GPS is a ~10-14 pt penalty (not a blocker), quantified in the SIL (new --nogps/--nomargin knobs)
- **Context.** Current flight PCB (the one that flew ASC007): TVC×2 servos + 4 pyro + MPU6050 + BMP280, **no GPS, no
  margin/roll servo channels**. The landing firmware wants 4 servos (TVC×2, `servoMargin`, `servoRoll`) + GPS. Question:
  does the board block a landing, and is a redesign needed? Added SIL knobs `--nogps` (skip the GPS feed -> firmware's
  `gpsFix` never sets -> accel-only x/y) and `--nomargin` (force the coast margin fin STOWED, `dep=0` -> no coast attitude
  control) + `NOGPS`/`NOMARGIN` env pass-through in `montecarlo.py`, to measure each gap instead of guessing.
- **Findings (N=150, on-target F15 landing).**
  - **Margin-fin servo = HARD BLOCKER.** `--nomargin`: the vehicle TUMBLES to 162-170° during the unpowered coast/descent
    and ABORTS (chute) — ~0% landings. Physics: with TVC off after burnout there is no other attitude control, so a
    stowed fin weathercocks the vehicle nose-first (engine UP) → the landing burn would fire the wrong way. The margin
    fins ARE the coast/descent attitude controller. The user designed the margin *module* but the PCB can't drive its
    servo — that's the real block.
  - **GPS = meaningful degradation, NOT a blocker.** `--nogps` still lands upright, but legs-10 survivable drops
    **80%→68%** (realistic) / **88%→74%** (characterized), and the miss tail grows (p90 ~7 m vs ~6). Accel-only x/y
    dead-reckoning drifts → more off-target + tippier touchdowns. A first CALM landing could work without GPS; on-target
    + wind performance wants it.
  - Roll servo = secondary (roll is a symptom; not tested as a blocker).
- **Verdict — targeted PCB addition, NOT a from-scratch respin.** The board runs a Teensy 4.x (plenty of spare PWM pins +
  UARTs), so the missing channels are just spare pins + power: add the **margin servo (required)** + **roll servo
  (recommended)** + a **GPS UART (strongly recommended, +10-14 pts)**. A minimal board revision (2 servo headers + a GPS
  connector) or a soldered proto add-on suffices — flying wires on a Gs-pulling, hard-landing vehicle are a reliability
  risk, so make them proper connectors. Current board is fine to finish the ASCENT + the static fire.
- **Files:** `Firmware/sim/sim_main.cpp` (--nogps/--nomargin), `Firmware/sim/montecarlo.py` (env pass-through).

## 2026-07-08 (pm) — REAL thrustcurve.org F15 curve in the SIL + firmware; NO-THROTTLE landing VALIDATED against real hobbyist landers (Kapoor/JRD, Project Horizon)
- **Context.** User is flying a same-batch F15 landing burn and wants to AVOID throttling. Two things this entry settles:
  the SIL's motor model (was a parametric ramp/plateau approximation, then briefly a guessed 0.4 s tail-off ramp), and
  whether a no-throttle solid landing is real (deep-research pass).
- **Real curve.** Replaced the parametric `thrustCurve`/`landThrustAt`/`ascentThrustAt` with the **Estes F15 RASP.ENG
  table from thrustcurve.org** (`F15_T`/`F15_F`, 23 pts; impulse ~49.6 Ns, peak 26.75 N @0.386 s, burn ~3.40 s) in BOTH
  the SIL truth (`sim/sim_main.cpp`) and the firmware's assumed model (`Sysiphus_Landing.ino`). **Key data fact: the
  tail-off is a ~0.05 s CLIFF (13 N→7.3→0), NOT a ramp** — my previous 0.4 s tail-off (which had lifted characterized
  legs-10 to 92%) was optimistic; removed. `MOTOR_TAILOFF_S`/`TAILOFF_S` deleted; `*_PLATEAU_N`/`*_PEAK_N` now vestigial labels.
- **Results (N=150).** Nominal **0.34 m/s** SOFT (perfect timing → motor cuts at the deck). Dispersed: legs-10 m/s
  survivable **80%** (lot motor) → **88%** (characterized ±1.5%); soft <5 m/s 38%→61%; hop 27%→11%; vz p90 10.1→7.8.
  So the real curve lands between the instant-cutoff (87%) and the fake-tailoff (92%) — **~88% legs-10 with a characterized
  motor is the honest number.** Dispersion (a hot/cold motor mistiming against the sharp cutoff) is the hop driver, not
  the tail-off shape.
- **No-throttle VALIDATED (deep research, 17 claims verified 3-0; note: synthesis step + ~26 agents hit a session limit,
  so this is the verified-claims set, not a merged report).** Besides BPS.space: **Aryan Kapoor / JRD Propulsion** (high-
  school student, Montgomery HS; first successful vertical landing **2024-07-05**, ~3 yr effort) landed a SOLID-motor
  rocket with **NO throttle** — TWO stacked solids (ascent + descent) in a 3D-printed gimbal, **±7° TVC** for attitude
  only, and beat the hop with **shock-absorbing legs (repurposed syringe + rubber bands)**; the touchdown was HARD and it
  **BOUNCED (landed upright twice)**. **Project Horizon** ("Eagle", published 2023-11-29) is a second non-BPS solid-motor
  propulsive landing. BPS.space is the one that DID fake a throttle (TVC-oscillation, then mechanical ceramic pincers).
- **Converged conclusion.** A no-throttle solid propulsive landing IS real and repeatable-ish, and the method is exactly
  what the SIL concluded: **TVC for attitude + a characterized motor for timing + shock-absorbing/crushable legs that
  absorb a HARD, possibly bouncy touchdown** — not a feather-soft landing. The user's architecture (two stacked F15s +
  TVC gimbal, ±10° on one axis, no throttle) matches Kapoor's. **Levers: characterize the motor (80→88% legs-10) + good
  energy-absorbing legs (~10 m/s, anti-bounce) + iterate (they took ~3 yr; lean on the SIL to compress that).**
- **Files:** `Firmware/sim/sim_main.cpp`, `Firmware/Sysiphus_Landing.ino`. Sources: thrustcurve.org (F15),
  jrdpropulsion.com, popsci.com (Kapoor), youtube @Project-Horizon, hackaday.com (BPS).

## 2026-07-08 — Landing SIL + firmware re-parameterized for the REAL Estes F15 (same-batch, both burns); honest F15-landing Monte Carlo
- **Context.** User locked **same-batch Estes F15 for both ascent and landing** (see [[project-isef-goal]] / ISEF_SCHEDULE).
  The landing SIL + firmware were configured for a *fictional* high-T/W short-burn landing motor (LAND 28 N peak /
  18 N plat / 2.0 s, T/W≈4.5). Re-parameterized BOTH the firmware's assumed model and the SIL truth to the F15
  (26 N peak / 14 N plat / **3.45 s** burn, T/W≈1.5) and the landing mass 0.80 → **0.95 kg** (two motors + legs +
  spent ascent casing). So the landing predictions are finally for the motor actually being flown.
- **Changes.** Firmware `<<< SET` block (`Sysiphus_Landing.ino`): `LAND_THRUST_AVG_N` 20→14, `MASS_LAND_KG` 0.80→0.95,
  `LAND_BURN_S/PLATEAU_N/PEAK_N` 2.0/18/28 → 3.45/14/26, `ASC_*` 12/22 → 14/26. SIL mirror (`sim/sim_main.cpp`):
  `MASS` 0.80→0.95, `LAND_*` 18/28/2.0 → 14/26/3.45, `ASC_*` 12/22 → 14/26.
- **Results.** **Nominal PASSES: 3.29 m/s, upright (3°), apogee 50.5 m** — the hoverslam survives the long weak burn
  (confirms the user's matched-impulse symmetry: the F15 arrests ~what it added). MC (N=150, mass weighed):
  - **realistic** (motor = lot variation ±4–5%): soft<5 **31%**, legs-8 m/s **61%**, legs-10 m/s **75%**; vz p50 6.0 /
    p90 10.1 / p99 16.3; aborts 2%.
  - **characterized** (motor static-fired/matched ±1.5%): soft<5 **49%**, legs-8 m/s **75%**, legs-10 m/s **79%**;
    vz p50 4.5 / p90 8.0 / p99 10.2; aborts 2.7%.
- **Findings.** (1) F15 landing is a **viable crushable-legs lander (rate legs ~10 m/s), NOT soft** — roughly comparable
  to the earlier idealized-motor baseline (legs-10 was 82%). (2) **Characterizing the motor is a big, quantified lever**
  (legs-8 survivability 61→75%, soft 31→49%) → **validates the static-fire + in-flight sys-ID plan.** (3) **The F15's
  long 3.45 s burn makes ATTITUDE-through-the-burn the binding constraint** (predicted): the far-target + crosswind
  corners lose retrograde attitude and tumble (~150–170° tilt, no clean TD) during the long burn — the F15's Achilles
  heel, and the abort does NOT always catch it (a real firmware robustness gap to fix: tilt-abort during LANDING_BURN).
- **Caveats / follow-ups.** Masses are estimates — WEIGH the real two-F15 vehicle and update `MASS_LAND_KG`/`MASS`.
  `validate.py` (nominal regression guard) needs **re-baselining** for the F15 config (it was tuned for the old motor).
  `KEFF_NOM=8` flies nominal but may want re-tuning for the F15 keff. The corner-tumble abort gap is unfixed.
- **Verdict.** F15 landing STANDS as the honest same-batch config: **characterize the motor + rate legs ~10 m/s.**
  **Files:** `Firmware/Sysiphus_Landing.ino` (`<<< SET` block), `Firmware/sim/sim_main.cpp`.

## 2026-07-07 (pm) — Ascent firmware: IMU driver MPU6050_tockn → Adafruit_MPU6050 (kills the FSR-rescale hack) + PID integral trim with anti-windup (auto-cancels the ASC007 lean). SUPERSEDES the ACC_FIX/GYRO_FIX approach in the entry below.
- **Context.** Two follow-ups to the ASC007 diagnosis (entry below). (1) The morning's FSR fix kept MPU6050_tockn and
  patched its hardcoded ±2 g/±500 dps scale with `ACC_FIX`/`GYRO_FIX` read-multipliers — an *un-bench-testable* scale
  factor on the control + abort path (the static pitch-the-nose test can't catch a scale error). (2) The root cause is
  a constant thrust-misalignment lean; a PD loop shows it as a steady lean = misalignment/`P_GAIN` that parks the
  vehicle near the 45° abort.
- **Change 1 — IMU driver swap (`Ascent_TVC.ino`).** `MPU6050_tockn` → **`Adafruit_MPU6050`** (same Adafruit ecosystem
  as the BMP280). `setAccelerometerRange(16 G)` / `setGyroRange(2000 DEG)` / `setFilterBandwidth(94 Hz)` — the library
  **auto-scales correctly**, so `ACC_FIX`/`GYRO_FIX` and the manual `0x1B`/`0x1C` register writes are **deleted**.
  A thin `readIMU()` caches `getEvent()` into the firmware's units (gyro deg/s bias-removed, accel m/s²);
  `calibrateGyroBias()` (average 600 at rest) replaces tockn's `calcGyroOffsets`. The firmware already does its own
  attitude fusion (accel-atan2 pad / gyro-integration flight), so it only needed clean rates+accel — a clean fit.
  Requires the **Adafruit_MPU6050 + Adafruit_Unified_Sensor** libraries (Library Manager). Signs unchanged (both
  drivers read the same raw registers) but BENCH-re-verify the pitch-the-nose sign check + a dynamic roll check.
- **Change 2 — integral trim + anti-windup (`TVC()`).** PD → PID: an integral `iTermX/Y` winds out the CONSTANT
  misalignment so the vehicle flies VERTICAL instead of holding a lean. **Conditional-integration anti-windup**
  (freeze the integrator whenever the pre-clamp command is saturated — a REAL tumble saturates, so the I term can't
  wind up and worsen it) + accumulator clamp ±`MAX_TILT`, reset at launch. `I_GAIN=0.20` (SIL-tuned: winds a 4° trim's
  16° lean down to ~0.7° in ~2 s, no overshoot, stable to slew 150 dps; SIL-overridable via getenv `IGAIN`, set 0 to
  disable). **It does NOT add authority** — cancelling a 4° misalignment still costs 4° of gimbal — so it complements,
  not replaces, the mechanical alignment fix.
- **SIL result (N=200, align σ=2°, roll/trim-aware harness).** Integral OFF→ON: **PASS 56.5% → 72.5%** (+16 pts),
  end-of-boost lean p50 **8.8° → 4.2°** / p90 17.1° → 10.2°. **ABORT rate unchanged (4.0%)** — those are early
  high-misalignment (4–6°) transients the integrator can't pre-empt; anti-windup makes the abort cases byte-identical
  (confirmed). So the integral is a robustness win for the marginal cases and safe on the tumble cases, exactly as
  intended.
- **SIL infra.** New shims `sim_ascent/shims/{Adafruit_MPU6050.h,Adafruit_Sensor.h}` (model the true auto-scaled
  values — no rescale); `ascent_sim.cpp` externs `I_GAIN`/`iTermX/Y`, adds `--igain`/getenv `IGAIN` and an
  `endlean` (end-of-boost tiltTRUE) metric so the MC can see the steady-state lean the integral fixes; `ascent_mc.py`
  reports it. Removed the now-unused `MPU6050_tockn.h` shim. Build clean (zig), nominal PASS.
- **Decision.** Ship both: Adafruit driver (verifiable-by-construction FSR, kills the un-testable patch) + integral
  trim (flies vertical under residual misalignment). Keep hot P/D gains. Mechanical alignment (entry below) remains the
  fundamental fix for authority headroom. **Files:** `Firmware/Ascent_TVC/Ascent_TVC.ino`,
  `Firmware/sim_ascent/{ascent_sim.cpp,ascent_mc.py,shims/Adafruit_MPU6050.h,shims/Adafruit_Sensor.h}`.

## 2026-07-07 — ASC007 (first Ascent_TVC flight) aborted: diagnosis REFRAMED from "roll-coupling" to "trim vs authority" (roll is a symptom); IMU FSR fixed; ascent SIL rebuilt roll-aware
- **Context.** First flight of `Ascent_TVC.ino`. It self-aborted (chute) ~1.9 s into a 3.65 s boost at ~52° tilt,
  ~13 m. Log `Rocket data/ASC007.CSV` (46 rows). First-look readout (from the log alone): an uncontrolled roll
  (GyroZ 0→−172°, ~113 dps peak; TVC has no roll authority) built the whole flight, a steady ~16° pitch/yaw lean
  sat with TVCy pinned near +5° (little margin), and both axes ran away when roll passed ~140° → hypothesized
  **naive-integration / gyroscopic roll-coupling**. The old ascent SIL (`sim_ascent/ascent_sim.cpp`) was 2 DECOUPLED
  tilt integrators with `gyroZ` hardcoded 0 — structurally blind to roll — so it had "validated" the gains falsely.
- **Firmware change (#3, `Ascent_TVC.ino`).** IMU FSR ±2 g/±500 dps (tockn default) → **±16 g/±2000 dps** (ASC007
  railed AccelZ at 2 g; true axial ~3.2 g). tockn's getters keep the ±2 g/±500 dps divisors, so added `ACC_FIX=8`,
  `GYRO_FIX=4` re-scale on read; **roll now self-integrated** from the scaled Z rate (was `getAngleZ()`, which bakes
  in the old scale); pre-launch check switched to FSR-invariant accel-atan2; register readback printed. Accel path
  is ratio-based (atan2) so control is unaffected; the FSR change is control-transparent + fixes the rail. `P_GAIN`/
  `D_GAIN` made plain `float` (not constexpr) so the SIL overrides them (env `PGAIN`/`DGAIN`) — flight defaults
  unchanged. **NOTE:** the gyro-scale path is NOT covered by the static pitch-the-nose bench test — verify with a
  DYNAMIC rotate-by-known-angle check before flight.
- **SIL rebuild (#4, `sim_ascent/ascent_sim.cpp`).** Replaced the toy physics with **quaternion 6DOF**: roll axis
  (Izz≈Iyy/17), Euler eqs WITH the (Iyy−Izz)·ω·ω gyroscopic terms, roll-torque SOURCES (`--rollcant` nozzle cant,
  `--cgoffx/y` CG-offset trim+TVC-roll-leak, `--finmis`), a WORLD-referenced aero normal-force moment (`--sm`
  static margin), servo first-order lag + slew, and the firmware's own estimate externed so the trace shows
  est-vs-true. Shim mirrors the FSR under-read (÷8/÷4). Nominal (good build) flies straight → PASS (control signs
  validated). `ascent_mc.py` now samples realistic misalignment (`MIS_SIGMA` deg), roll cant, static margin, and a
  LOADED-gimbal servo slew.
- **Finding — the roll-coupling hypothesis is FALSIFIED; two more corrections.** (1) **Roll is a SYMPTOM, not the
  cause**: the boost tolerates 884 dps roll (PASS); a body-fixed lean holds rock-steady as roll climbs past 180°
  (est tracks truth). (2) The cause is **trim vs authority**: an effective thrust-misalignment/CG-offset drives a
  steady lean = misalignment/`P_GAIN` (≈4°/° here), which eats the ±5° TVC. ASC007's ~16° lean ⇒ **~4° effective
  misalignment** — right at the ~5° tumble edge. (3) **Softening the gains is CONTRAINDICATED** (reverses the
  first-look hedge): soft 0.08/0.08 droops **3× more** under trim (50° lean at 4° misalignment; MC p99 lean 40°
  vs the hot gains' 14.5°) and tumbles at a SMALLER misalignment. MC: hot gains 93% PASS / 0% abort at align σ=1°,
  falling to 56%/4% abort at σ=2° — and every abort is a high-misalignment (4–6.5°), low-roll draw.
- **Decision.** KEEP the hot pole-placed gains (0.249/0.062) — they are the more trim-robust choice. Primary fix is
  HARDWARE: tighten nozzle/CG-to-thrust-axis alignment to ≤~1° (σ=1° → 93% robust), and use a fast/low-load gimbal
  servo (at 4° misalignment, slew 100 dps tumbles, ≥200 holds). Secondary: more TVC authority (raise `MAX_TILT` if
  the gimbal allows) raises the saturation ceiling. Find/kill the roll source too (nozzle cant / build asymmetry)
  since it's real, but it is not the abort driver.
- **Reproduction caveat (honest).** A CONSTANT trim reproduces ASC007's boost lean/TVC/roll magnitudes but gives a
  sharp hold-vs-immediate-tumble threshold, not the observed "hold ~16° for 1.4 s then diverge at 1.9 s." The
  final-second trigger (marginal-airframe q-creep, a gust, or the loaded servo momentarily slipping) is bracketed
  (holds at 4°, tumbles at ~4.7°) but not pinned — a fidelity limit, not a diagnosis gap.
- **Alternatives considered.** Adding roll control (rejected: 2-axis TVC has no roll authority without CG offset, and
  roll isn't the cause); quaternion/derotation attitude in the ascent firmware (deferred: unnecessary — body-frame
  control is self-consistent for body-fixed disturbances, the SIL shows). **Files:** `Firmware/Ascent_TVC/Ascent_TVC.ino`,
  `Firmware/sim_ascent/{ascent_sim.cpp,ascent_mc.py,shims/MPU6050_tockn.h}`, `Rocket data/ASC007_analysis.png`.

## 2026-07-05 — Estimator-tail diagnosis: the velocity estimate is NOT the bottleneck (falsifies the "gravity-leakage → velocity bias" hypothesis)
- **Context.** The 07-03 baseline blamed the vz tail on "seed-2 gyro random walk → ~2° attitude drift → gravity
  leakage → velocity-estimate bias → hoverslam arrests ~0.5 m high." TODO_AI P1 scoped GPS velocity fusion (RMC)
  + accel attitude aiding to fix it. **Before building either, I MEASURED the est-vs-true velocity error at
  ignition** (the harness logs both true and estimated state) — falsify before confirm.
- **Finding (measured; validate grid + the seed-2 cluster).** The velocity estimate at hoverslam ignition is
  EXCELLENT everywhere: **estVz error −0.04..−0.10 m/s, estVx error +0.06..+0.10 m/s** — identical in the WORST
  cell (T=24 W=−4, vz=8.69), the MEDIAN cell (vz=2.89), and the entire seed-2 5.3–5.6 cluster. There is no
  velocity bias to fix. GPS velocity fusion would correct ~0.1 m/s → **it cannot move the tail. Premise falsified.**
- **What the tail actually is.** (1) WORST cell (8.69) = the far-target + headwind corner igniting at ~50° attitude
  (true roll +42.8°, tiltX −49.6°): the thrust vertical fraction is cos(~50°)≈0.64 while `predictStopAlt`'s `cosT`
  floors at cos(RETRO_MAX_RAD=35°)=0.82 → the ignition predictor is mildly optimistic → under-brake, compounded by
  reorient time inside the ~2 s burn. This is the known far+headwind authority wall (TODO_AI). (2) seed-2 cluster
  (5.3–5.6, across many cells) = the irreducible solid-motor knife-edge: a gust near ignition perturbs the
  pre-committed, un-cuttable, un-throttleable burn; the vehicle KNOWS its velocity but cannot re-plan. Not estimation.
- **There IS a small ~2° ATTITUDE estimate error** (estTilt vs true tiltX) from gyro drift — but it costs only a
  small pointing miss during the burn (misses stay <5 m, passing), NOT the headline vz. Attitude aiding would buy
  a tighter pad, not a softer landing.
- **Redirect.** De-scope GPS velocity fusion / attitude aiding as the *tail* fix — they target a non-problem for
  this plant + SIL. The real vz levers are unchanged and hardware-side: crushable ~10 m/s legs + a characterized
  landing motor. Firmware candidates that DO target the measured mechanism (both delicate, confirm first, low
  yield): a tilt-aware `cosT` in `predictStopAlt` for the far corner (1 cell), or bigger margin fins / boostback
  for the far+headwind authority wall. **Caveat:** if the FLIGHT IMU drifts materially worse than the SIL gyro
  model, revisit velocity fusion as insurance — but that's a hardware-driven decision, not today's bottleneck.
- **Files:** none (measurement only). Diagnostic script kept in the session scratchpad; TODO_AI P1 reframed.

## 2026-07-05 — Coast margin-fin controller: damped-LS fin inversion + servo-bandwidth low-pass (replaces the on/off authority gate)
- **Change (`ctrlCoast`, firmware).** Replaced the discontinuous authority gate (`if bmag2<B0_COAST_MIN² hold
  neutral, else project (a·b)/bmag2`) with a DAMPED least-squares fin inversion `deploy = dep* +
  (a·b)/(bmag2 + LAMBDA_COAST)` (LAMBDA_COAST=2.0), followed by a low-pass `deploy += DEPLOY_LP_BETA·(cmd−deploy)`
  (β=0.09, τ≈0.11 s @100 Hz).
- **Why.** (1) The hard gate switched at B0_COAST_MIN and could limit-cycle as the sensed authority crossed the
  threshold; damped-LS is continuous and, as aero authority → 0, the numerator (a·b) → 0 too, so the command eases
  to the neutral trim dep* with no switch. (2) The margin servo is SLOW (~0.5 s full travel); the accel-noise-driven
  raw command chatters faster than the fin can move, so the servo lowpasses it toward neutral = lost authority.
  Matching the command to the fin's real bandwidth gives a smooth, efficient fin that tracks the slew, not the noise.
- **Validation.** validate.py **27/36 (75%, floor 70%), vz p50 2.89 (was 3.2) / p90 5.47 / max 8.69** — no
  regression, p50 slightly better (smoother coast fin). MC realistic N=150: soft 21%, legs-8 63%, legs-10 82% —
  unchanged from the 07-03 baseline, as expected (the coast fin is not a touchdown-vz lever; see the diagnosis above).
- **Viewer.** Harness logs two new columns depReal/rollDefReal (the ACTUAL servo-limited fin positions) so the 3-D
  sweep viewer shows the real smooth fin instead of the raw chattering command.
- **Files:** `Firmware/Sysiphus_Landing.cpp` (ctrlCoast), `Firmware/sim/sim_main.cpp` (log cols),
  `Firmware/sim/sweep_html.py` + `sweep.html` (viewer).

## 2026-07-03 (pm) — 3-D sweep RENDERER; body-frame control refactor attempted + reverted (co-located IMU/TVC)
- **3-D render of the sweep animation (user request).** Replaced the 2-D side-view canvas in `sweep_html.py`
  with a self-contained (no external libs) 3-D renderer: orbit camera (drag/wheel + top/side buttons),
  painter's-algorithm shaded rocket posed by the LOGGED TRUE QUATERNION (added Qw..Qz to the CSV), the 4 margin
  fins as fold-out canards (common fold = deploy, differential fold = roll from `rollDef`), 3 landing legs that
  swing out at ignition, a gimballed exhaust following the TVC command, ground grid, target ring, EKF-belief
  marker, crossrange/roll HUD. Headless-smoke-tested (node, stubbed canvas) across all 10,098 frames of the grid:
  0 NaN coords, 0 exceptions, 390k faces. `make_flight_html.py` unchanged (still the 2-D single-flight view).
- **Body-frame control refactor: attempted, REVERTED (kept the validated derotation form).** User note: the IMU
  is bolted to the TVC so each gimbal axis stays aligned with its gyro axis -> the honest controller is body-frame-
  native (per-axis, co-located gyro+gimbal), no roll derotation. Tried it two ways; BOTH diverged and I reverted:
  * v1 folded the (guidance-steered) target INTO the ESO measurement as a body pointing error `ex=target-nose`.
    A time-varying reference inside the measurement looks like a disturbance -> the ESO's z3 winds up -> divergence.
    Real lesson (kept as a code note): NEVER fold a moving reference into the ESO measurement; it must stay the REF.
  * v2 fixed that (ESO tracks the roll-derotated nose lean, target stays the reference) and boost then tracked
    perfectly (miss 0.76 m), but it still tumbled -- a residual instability (a mirror `uT2=-uT` signature at
    roll~0) I could not root-cause quickly despite the loop being algebraically identical to the working form at
    small roll. Reverted to the derotation controller (validated 75% distributional baseline, worst vz 8.6).
  * **Why the revert is correct, not a cop-out:** the derotation IS the co-located-frame mapping -- it rotates the
    command by the roll the co-located gyros measure (the quaternion is built from them), so the hardware wiring is
    fully consistent with it. Body-native is an equivalent, arguably cleaner form worth finishing later (it needs
    careful sign/transient validation), but shipping a divergent loop would be worse than a validated equivalent.
- **Files:** `Firmware/sim/sim_main.cpp` (Qw..Qz log cols), `Firmware/sim/sweep_html.py` (3-D renderer),
  `Firmware/Sysiphus_Landing.cpp` (control reverted to derotation + a "don't fold a moving ref into meas" note).

## 2026-07-03 — FULL 3-D BUILD: quaternion GNC, 2-axis TVC, roll control via differential margin fins, CG-misalignment roll coupling
The architectural gap (2-D pitch-plane only) is CLOSED. Firmware + harness are now genuinely 3-D; the classic
TVC roll-divergence mechanisms are modeled and handled. (User: "do the 3d... margin mod fins for roll control at
high aoa... add cg alignment error so TVC can accidentally induce roll.")
- **Harness (sim_main.cpp) → 6DOF:** quaternion attitude, body rates (p,q,r), Euler rigid-body equations WITH
  gyroscopic roll-lateral coupling (Iz≈IYY/65 -> spin cross-couples the lateral axes: inertial roll coupling);
  2-axis TVC gimbal applied at the nozzle with LATERAL CG OFFSET (--cgoffx/--cgoffy): M_roll = T*(cgy*da - cgx*db)
  -- every TVC correction leaks roll torque, exactly the classic mechanism; nozzle tangential cant (--rollcant,
  constant roll torque under thrust); fin misalignment (--finmis, aero roll trim); CROSSwind (--windy) with its
  own gusts; margin-modulation aero as a full crossflow VECTOR (body CP aft / fin CP fwd, same physics as 2-D);
  differential-fin roll torque ~ q*S_FIN*CN_FIN*(vlat/V)*R_FIN*rollDef (authority ~ sin(aoa): strong at HIGH aoa,
  zero along the flow); 3-axis IMU w/ per-axis bias+random-walk+FSR; GPS x AND y; landed-latch, legs, LEG_H as before.
- **Firmware → 3-axis:** quaternion strapdown attitude (measured-dt, loop-stall robust); tilt decomposed into
  world tiltX/tiltY (roll-invariant) + swing-twist roll angle; per-tilt-channel ADRC ESOs with the actuation
  DEROTATED through the measured roll (at 90 deg roll the gimbal axes swap; at 180 the sign flips -- a
  roll-ignorant controller diverges instantly, this one keeps tracking while rolling, verified at 200 dps);
  2-axis TVC (u_tvc/u_tvc2, slew-matched observers); 3-channel EKF (kfY + GPS lat fusion, lead-compensated);
  2-axis wind estimate + 2-axis wind-adaptive boost guidance (crosswind nulled: y-miss <=2.1 m at +-4 m/s
  crosswind); 3-D retrograde target (total-tilt cone cap); margin coast = same trim-ride torque-commander with
  the authority now a VECTOR from the signed lateral specific force -- in tilt-channel coordinates it is simply
  +k*(sfx,sfy), the clean 3-D generalization of the 2-D scalar (least-squares projection of the desired accel
  onto the crossflow-achievable direction); ROLL-RATE ESO driving the DIFFERENTIAL fin fold (roll angle is free;
  only the rate is bounded); roll-rate abort at 720 dps.
- **Sign/pairing traps found by the planar regression (document for the 3-D-firmware future):** (1) gimbal sign:
  deflecting thrust +x pushes the TAIL +x -> nose -x; the plant needs F_lat=-T*sin(d) for the firmware's keff>0.
  (2) axis pairing: torque about body +Y drives tilt-X, torque about -X drives tilt-Y -- keff RLS must regress on
  gyroY for the X-channel, and the margin authority vector is +k*(sfx,sfy) only in tilt-channel coords.
  (3) TD gate: declaring touchdown while the solid still thrusts surrenders TVC to an UNOPPOSED CG-offset torque
  (~T*cgoff/IYY) that the hop integrates into ~55 deg tilt -> TD now requires the full burn elapsed (+5%).
- **Roll-divergence results:** cgoff 2-4 mm: roll held to 7-42 dps, tilt <3 deg, PASS. Realistic nozzle cant
  (5e-5 m arm): roll builds to ~170-200 dps during BOOST (fins can't bite at aoa~0 -- physics, not tuning), the
  differential fins catch it in coast when crossflow develops; lands fine. Extreme cant (4e-4): correctly ABORTS
  at 4400 dps. Combined diagonal-wind+cgoff+cant: 0.8-1.4 m/s touchdowns, <=52 dps.
- **Roll-twist subtlety (verified, not a bug):** a planar-looking big swing accumulates tens of degrees of
  REAL geometric twist (rotation sequence non-commutativity) with ~zero roll RATE; firmware twist tracked truth
  within 1.5 deg through a 46 deg excursion -- which is exactly why the derotated mapping is load-bearing.
- **Honest 3-D baseline (nominal build, wind grid x 5 seeds, incl. 25 ms motor dead-time):** 81/100 sweep-PASS,
  vz p50 3.2 / p90 5.4 / max 8.6. The tail is ONE bad 3-axis gyro-random-walk draw (seed 2: ~2 deg attitude
  drift -> gravity leakage -> velocity-estimate bias -> knife-edge arrests ~0.5 m high, 5-6 m/s hops + ~4 m
  misses, at EVERY wind incl. calm) + one capture-swing outlier (8.6). Tried: STOP_MARGIN 0.5->0.65->0.8 (see-saws
  the distribution; kept 0.65) and Q_BIAS loosening (no effect: 8 Hz position-only GPS can't identify 3-axis
  leakage). The REAL fix for the tail is sensor/estimator hardware-class work (accel gravity-vector attitude
  aiding when unpowered, GPS velocity fusion), logged in TODO_AI. Note: WITHOUT dead-time seed 2 is WORSE (6.7-7.1
  -- the IGN_LEAD_S comp then fires early into an instant-light motor); the system is tuned for the honest plant.
- **3-D Monte Carlo (realistic scenario + lateral-CG/cant/finmis dispersions + crosswind):** soft<5 21%, legs
  8 m/s 63%, legs 10 m/s 82%; ~2-5% safety ABORTS (chute; roll-cant tail trips the 720 dps gate -- counted
  separately now, the harness has no chute drag model so their "impact" is meaningless). vs 2-D: soft 33%->21%,
  legs-10 87%->82% -- 3-D honesty costs ~5-12 points; the crushable-legs + characterized-motor conclusion stands.
- **validate.py reworked to a DISTRIBUTIONAL guard** (pass-rate floor 70% + hard caps vz<9.5/miss<6/tilt<15) --
  per-cell bounds would cry wolf on the known seed-2 tail; a real regression (tumble, roll runaway, lost target)
  blows the caps. Committed baseline above. make_flight_html.py: roll/rollrate/crossrange/roll-fin HUD readouts.
- **Files:** `Firmware/Sysiphus_Landing.cpp` (3-axis rewrite), `Firmware/sim/sim_main.cpp` (6DOF plant + knobs),
  `Firmware/sim/shims/{Arduino,MPU6050_tockn}.h` (3-axis IMU), `montecarlo.py` (3-D dispersions + abort
  accounting), `validate.py` (distributional), `make_flight_html.py` (3-D HUD).

## 2026-07-02 (pm) — FLIGHT-READINESS: Monte Carlo dispersion analysis. Verdict: NOT ready; solid-motor soft-landing is fundamentally dispersion-limited
- **Question (user):** is the sim fully built out / the firmware ready to fly? **Answer: no, on both counts.** The
  100/100 sweep measured robustness to WIND ONLY; it held mass/inertia/CG/thrust/timing at exact nominal.
- **Built the missing tools + realism** (Firmware/sim): sensor FSR saturation (`--accfsr/--gyrofsr`; flight 1
  railed 2 g/500 dps), ignition dead-time + scatter (`--igndelay/--ignjit`), control-loop stall/jitter
  (`--jitter`), and mass/inertia/CG/fin dispersions (`--massmult/--iyymult/--cgoff/--finauth`). New
  `montecarlo.py` (joint dispersion MC + leg-rating survivability) and `validate.py` (fast nominal regression
  guard, 36/36). The sweep now pins `--ignjit 0` to isolate the wind axis and warns that green != flight-ready.
- **Real firmware gaps found + FIXED** (each was invisible at nominal):
  * **Ignition motor dead-time:** the hoverslam ignited on the present state, but the solid takes ~25 ms to build
    thrust -> ignites ~0.6 m late at 24 m/s -> under-brake (vz 3.5->6.2). Fix: `ignitionReached` projects the
    free-fall over `IGN_LEAD_S` and tests the burn from there. (Scatter around the mean stays irreducible.)
  * **Fixed-dt integrators:** EKF + attitude used a constant `DT_S`; a loop stall (SD/I2C) then silently loses
    time and mistimes the hoverslam (jitter 2%/ms: vz 3.5->8.9, tilt->-8.6). Fix: integrate on MEASURED elapsed
    dt -> jitter becomes benign scatter (vz 3.5, tilt -3.5). This is the Pi=keff*tau^2 latency lesson, in firmware.
  * **Sensor FSR** now modeled (16 g/2000 dps nominal; clips like the real part).
- **In-flight mass ID: built, then DEFAULTED OFF** (`g_massIdOn=false`, `--massid` to enable). Boost accel gives
  only thrust/MASS (not separable), so an ascent-motor lot deviation is mis-attributed to mass and POISONS the
  hoverslam. Since the airframe is WEIGHED on the ground, identifying mass in flight trades a solved problem for
  an unsolvable one. MC (realistic): 33% soft OFF vs 10% ON. Off wins.
- **THE FUNDAMENTAL FINDING (quantified):** the controlling quantity for the hoverslam is the LANDING motor's
  thrust/mass, and it is UNOBSERVABLE before the solid commits (can't measure landing thrust pre-ignition; boost
  conflates mass with ascent thrust; and a solid can't throttle/cut to correct mid-burn). So a solid-motor
  hoverslam cannot be made reliably SOFT across realistic motor dispersion. MC pass rates (mass-ID off,
  wind +-5, targets 0-24, gusts, all dispersions stacked, N=150 each):
    | gate / build            | soft <5 m/s | legs 8 m/s | legs 10 m/s | legs 12 m/s | p50 vz |
    | realistic (+-4% motors) |     33%     |    70%     |     87%     |     93%     |  6.0   |
    | characterized (+-1.5%)  |     41%     |    91%     |     98%     |     99%     |  5.4   |
  Single-axis: landing-thrust +-4% -> 6-8 m/s; ascent-thrust +-4% -> 7-8 m/s; mass +-10% -> handled by weighing.
- **VERDICT + the actionable path (hardware, not firmware):** near-zero touchdown is NOT reachable with a solid
  (confirms + quantifies the long-standing "throttleable motor" ceiling). But the vehicle IS a viable
  CONTROLLED-DESCENT-TO-CRUSHABLE-LEGS lander: **static-fire-characterize the landing motor lot (+-1.5%) + legs
  rated ~8-10 m/s stroke -> ~91-98% land survivable, upright, on-target.** Priority levers: (1) crushable/energy-
  absorbing legs sized for ~10 m/s [cheapest, biggest], (2) characterized/thrust-matched landing motor, (3) for
  true soft, a throttleable/extinguishable motor. SEPARATELY, the firmware is still 2-D (pitch only): the YAW
  plane + roll handling are unimplemented and MUST be added before any free flight (the header flags this).
- **Files:** `Firmware/Sysiphus_Landing.cpp` (IGN_LEAD_S projection, measured-dt EKF/IMU, mass-ID + default-off,
  ASC_PROP_KG); `Firmware/sim/sim_main.cpp` (FSR clip, ignition delay/scatter, loop jitter, mass/iyy/cg dispersion,
  knobs); `Firmware/sim/montecarlo.py` (new), `validate.py` (new), `sweep_html.py` (isolate wind axis + caveat).

## 2026-07-02 — Margin coast IS torque-commanded; RLS self-ID doesn't help (physical authority limit, not a b0 gap)
- **Question (user):** "are you using torque commanding on the margin mod like with TVC? you should do that."
- **Finding 1 — already there:** `ctrlCoast` computes `deploy = depN + alphaDes/b0` — angular-acceleration
  commanding via authority inversion, structurally identical to the TVC loops' `u = alphaDes/keff`. The ADRC ESO
  lumps+cancels the residual body weathercock. So the loop shape the question asks for was already in place.
- **The real gap vs TVC:** TVC RLS-identifies keff online; the margin b0 is an open-loop fin-geometry model. Added
  a harness knob `--finauth <scale>` (scales the PLANT fin effectiveness while the firmware keeps its nominal
  model) to measure how much that matters.
- **Finding 2 — the ESO already covers realistic model error:** across true/modeled fin ratio 0.7–2.0× the
  torque-commander lands fine (the ESO absorbs the mismatch); failure is one-sided (over-estimating b0 is the
  dangerous direction — under-commands and tumbles; under-estimating is trimmed out).
- **Finding 3 — the residual cliff is PHYSICAL, not estimation:** below ~0.55× it tumbles even in DEAD-CALM air.
  The mechanism is weathercock-steering: neutral fold dep*≈0.48 (torque-free at any attitude); fold below → nose-
  first stable, above → tail-first stable (retrograde). The retrograde slew is driven by the NET tail-first
  restoring at full deploy = (fin_full − body_weathercock). Nominal fin_full is 2.08× body → NET 1.08× body
  (comfortable, ~7× margin to the cliff). At 0.5× fin_full ≈ body → NET ≈ 0 → no authority to force/hold engine-
  down → tumble; at 0.4× fin_full < body, period. (Earlier wording "barely cancels the weathercock at ratio 1.04"
  conflated fin/body with net/body — corrected: the working number is (fin_full−body)/body, 1.08 nominal.) No b0
  identification manufactures that authority — the lever is fin SIZING so fin_full stays ≳1.2× body (net ≳0.2×).
- **Tried & removed:** a one-sided tracking-failure adaptation of a `finTrust` scale (coast-robust analog of RLS,
  since keff-style regression is confounded during coast — the fin regulates a comparable-size weathercock so good
  tracking makes the deploy-vs-accel slope read ~0). It left the nominal untouched (good) but did NOT move the
  physical cliff (correct — can't). Removed: complexity + new failure surface for zero benefit in the controllable
  range. Kept the reasoning as a code note on `marginAuthority`, and kept `--finauth` as a robustness-probe knob.
- **Net:** margin torque commanding confirmed + characterized; robust to ±~40% fin-model error via the ESO alone;
  the "size the fin to keep full-deploy authority ≳1.2× the body weathercock" rule is the actionable hardware spec.
- **Files:** `Firmware/Sysiphus_Landing.cpp` (comment/notes only — behavior unchanged, 100/100 preserved);
  `Firmware/sim/sim_main.cpp` (`--finauth` knob + `FIN_AUTH` plant scale).

## 2026-07-01 (pm-4) — Honest fold-out margin-fin plant + trim-ride coast + EKF lead-comp: 100/100
Five coupled changes (user: fins must track the migrating CG; make the filter better; fix the remaining bad flights).
- **EKF lead-compensation at FUSION (firmware):** the baro reading is the altitude ~40 ms ago and a GPS fix is the
  position ~200 ms ago; fusing them raw dragged estZ ~1.1 m high at 24 m/s and estX ~v·0.2 behind (the "drifting
  ghost"). Now `ekfStep` fuses `meas + estV*lag` (BARO_LAG_S=0.040, GPS_LAT_S=0.20) so estZ/estX track the true
  present state (est error ~0.1 m through the whole burn); the old point-of-use lead in `ignitionReached` was
  REMOVED (it would double-compensate). Don't inflate BARO_LAG_S: overcomp reads LOW → early ignition → hop.
- **Hoverslam stop-margin (asymmetric cost):** with unbiased estimates the old net-calibration was exposed — the
  burn arrested ~0.4 m ABOVE the deck and the un-cuttable solid (T/W>1) relaunched it (~5-6 m/s impact) where the
  old biased chain arrived ~3 m/s hot and stuck. Arriving hot costs ~1 m/s on the legs; arresting airborne costs
  ~5+. So ignite when a burn started STOP_MARGIN_M=0.50 m HIGHER would just reach the deck (margin on the START
  altitude — the predictor's return value saturates at ~-vz·dt, a negative threshold never fires).
- **Leg contact height (harness realism):** deployed legs touch LEG_H=0.08 m below the body reference. Without it,
  contact was a knife-edge at exactly pz=0 — a burn nulling v 4 cm up "hovered" and relaunched, an artifact legs
  exist to absorb. Also: TD is NOT declared while the burn clock says the solid is still thrusting (>90% burn
  gate in `hBurn`) — declaring "landed" mid-air at 0.15 m cut TVC and the hop fell back uncontrolled/tilted.
- **Fold-out margin-fin plant (harness, replaces the signed-direct-moment abstraction):** a symmetric fold-out
  canard pair on ONE servo can only SHIFT THE CP: moment `-q·sin(aoa)·[S_REF·CN_BODY·l_body - dep·S_FIN·CN_FIN·l_fin]`,
  zero at aoa 0/180, sign follows aoa, arms referenced to an EXPLICIT MIGRATING CG (propellant burns → CG forward;
  legs → aft; the TVC nozzle arm rides the same xcg). Neutral deploy dep* is NOT 50% and moves with the CG
  (≈0.37 at launch → ≈0.48 in coast). The old model let the fin fight the weathercock statically at any aoa —
  physically flattering; under the honest plant that policy diverges (tailwind cell hit −40° at ignition).
- **Trim-ride margin-modulation coast controller (firmware):** at the CG-scheduled dep* the net aero moment is zero
  AT ANY ATTITUDE → holding ground-retrograde far off the airflow (strong headwind) costs nothing. Policy: sit at
  dep* and let the ADRC modulate deploy around it through the SIGNED aoa-mediated authority, recovered wind-
  agnostically from the sensed LATERAL specific force (`m·sfx ≈ q·S·CD·sin(aoa)` → magnitude AND sign; the old
  gauge used |sf| and threw the sign away). A stability BIAS fallback was tried first and is WRONG in wind: it lets
  the flow capture the vehicle onto the wind-tilted aero equilibrium (−44° in an 8 m/s headwind), then the escape
  is sin(aoa)-limited and arrives at ignition at +54°/s. Fallback below B0_COAST_MIN is dep* (torque-free), not a bias.
- **Slew-matched actuator observers + coast bandwidth (firmware):** the anti-windup observers claimed 10× the real
  servo rate (margin: 20/s vs real ~2.1/s for a 150 deg/s servo over 70°), and coast ran at WC=6 demanding ~20× the
  fin's ~1-2 rad/s² authority → the command chattered on the 0/1 rails and the real servo lowpassed the chatter to
  ~neutral (vehicle coasting on momentum). Observers now derive from SERVO_SLEW_DPS; coast has its own WC_COAST=2.0/
  W0_COAST=10 (the Π-research lesson: bandwidth must match actuator authority/slew — again).
- **Result: 100/100 PASS** (targets 0/8/16/24 × wind −4…+4 × 5 gust seeds): vz worst 4.9 (typ ~3.5), |miss| ≤2.3 m,
  tilt ≤3.1° (sweep.html: worst median |miss| 1.6, worst-of-seed 1.99). Probes at W=±5, ±6 also PASS (old envelope
  edge was W=−5 failing). Supersedes the pm-3 caveat: the trim-ride controller no longer fights the weathercock at
  all, so the "fin only 0.9× the weathercock at 90° aoa" thin-margin concern is moot.
- **Files:** `Firmware/Sysiphus_Landing.cpp` (EKF fusion comp, STOP_MARGIN_M, hBurn gate, fold-out fin constants,
  depNeutralCoast, signed marginAuthority, ctrlCoast trim-ride, WC_COAST/W0_COAST, SERVO_SLEW_DPS observers, fins
  stowed in powered phases); `Firmware/sim/sim_main.cpp` (migrating-CG margin plant, LEG_H contact).

## 2026-07-01 (pm-3) — Coast-apogee tumble was a CONTROLLER bug, not a physical limit (16/20 → 24/24)
- **Prior belief (overturned):** W≤−4 far-target apogee tumble was "likely a physical authority limit" (weak margin
  fin at low apogee airspeed). **Wrong.** It was the fin-authority ESTIMATE using GROUND velocity.
- **Root cause:** `marginAuthority()` computed authority from `sqrt(estVx²+estVz²)` (ground speed). In a headwind at
  apogee ground speed collapses to ~2 m/s, so the estimate read ~10× too low and the `b0<0.5` guard DISABLED the
  fin — exactly at apogee. But the AIR-relative speed is ~6 m/s (ground 2 − headwind −4), so real authority existed.
  The fin sat neutral through apogee, the vehicle weathercocked, `th` ran to ~24° at ~43°/s, blew past 90°, tumbled.
- **Fix:** gauge fin authority from the SENSED aero specific force (IMU), which is wind-agnostic — during coast
  sf = aero force/mass, and both fin moment and body drag scale with q_dyn, so authority ∝ |sf_aero| (S_ref cancels):
  `K_MARGIN = 2·SM_MAX·CN_FIN·D_REF·mass/(IYY·CD)`, low-passed. Now the fin engages at apogee with the correct b0,
  arrests the divergence before it runs away. (This is the web app's accelerometer-based canard idea.)
- **Result:** **24/24 PASS** over targets 0/8/16/24 × wind −5…+4 (was 16/20; W=−5 failed everywhere before).
  Robust across 5 gust seeds on the worst cases (T24/W−5, T24/W−4): all PASS, final tilt 0–11°, no tumbles.
- **Caveat (still true):** the fin is only ~0.9× the weathercock moment at a sustained 90° aoa, so the margin is
  thin — it works by engaging EARLY and never letting aoa run away. Bigger fins (↑CN_FIN/SM_MAX) would add margin;
  not needed for the current ±5 m/s envelope but worth noting for stronger winds.
- **Files:** `Firmware/Sysiphus_Landing.cpp` (`marginAuthority` sensed-force; `sfBx/sfBz/sfAeroLP` in `readIMU`;
  `DRAG_CD` moved to config; `K_MARGIN`).

## 2026-07-01 (pm-2) — Near-ground wobble: keff reset + legs-at-ignition + gentler divert
- **Symptom (user):** rocket induces oscillations right as the legs deploy, just above the ground — looked like it
  was leaning hard to chase the target too close to the deck.
- **Root causes (three, from the burn trace):** (1) `keff_est` entered the landing burn STALE (~2.3 from coast)
  while true keff≈7 → the controller over-actuated 3× → `uTVC` thrashed ±1.5, `q` swung ±21°/s for the first
  ~0.15 s. (2) Legs deployed 1 s into the burn (~4 m up) → instant ~25% keff drop (+20% Iyy, −10% arm) that the
  controller couldn't track in the ~0.5 s left before touchdown → th −2→−3.3°, rate reversal = the near-ground
  kick. (3) Divert tilt capped at 24° and ramped out only by 2.5 m → a target-chasing lean near the ground.
- **Fix:** (1) reset `keff_est=KEFF_NOM` at ignition. (2) deploy legs AT ignition so the plant is constant for the
  whole controlled descent (RLS IDs it once, ~1.5 s to settle); in the SIL legs change only keff, not drag, so
  braking/ignition are unaffected. (3) divert tilt cap 24°→15° and ramp fully out by 4 m (was 2.5 m) → last ~4 m
  fly purely retrograde→vertical. Land-safe-first over hit-the-mark.
- **Result:** peak burn rate ~21→**2–14.5°/s**, early-burn thrash gone, legs kick gone; **still 16/20**, and
  **miss IMPROVED to <0.4 m** (less tilt → cleaner nominal tracking), vz unchanged ~3.3 m/s. Terminal reversals
  are now small-amplitude settling (3–5°/s), not a violent kick.
- **On "make boost more accurate":** not needed — miss is already <0.4 m, at/below the GPS+EKF sensor floor
  (~0.6 m noise); the residual is stochastic gust drift + sensor noise, not boost aim error. The safe move was to
  DE-weight terminal target-chasing (done), which improved miss anyway.
- **Files:** `Firmware/Sysiphus_Landing.cpp` (ignition block: keff reset + legs; STILT2 24→15°; divert ramp).

## 2026-07-01 — SIL landings: fixed a fake-PASS harness bug + curve/drag-aware causal ignition (0→16/20)
- **Two real bugs found, one masking the other.**
  1. **SIL harness under-reported impact (critical).** `stepPhysics` kept integrating after touchdown, so gravity
     re-crossed `pz<=0` every step and OVERWROTE `impactVz` with a ~−0.01 m/s "re-impact." Every prior verdict was
     therefore fake-PASS. Fix: `if(landed) return;` at the top of `stepPhysics` (latch true impact, freeze on ground).
     Honest re-baseline: **0/20 PASS**, ~6.7 m/s impacts.
  2. **Suicide-burn over-braked → solid-motor hop → free-fall.** The causal ignition used a FLAT average thrust
     (`LAND_THRUST_AVG_N`); the real curve is front-loaded (peak 28 N) so it nulled velocity ~2 m up. A solid motor
     can't cut, so it lifted back off, burned out, and free-fell the last ~2 m — the "oscillation before touchdown"
     the user saw. The old `estZ<2.0 && estVz>-TD_VZ` gate then declared a (fake) soft TD mid-air.
- **Fix (firmware):** replaced the flat-thrust braking-distance formula with a **causal forward predictor** that
  integrates "ignite now" under gravity + the KNOWN landing-motor thrust curve (`landThrustAt`) + aero drag
  (`predictStopAlt`), and ignites when the predicted stop-altitude crosses 0. Added the motor curve constants
  (`LAND_BURN_S/PLATEAU_N/PEAK_N`, `DRAG_CD`) as datasheet `// <<< SET` values. Removed the free-fall ignition-lead
  projection (double-counted the spin-up ramp → fired ~0.1 s early). Tightened the TD gate 2.0 m→0.5 m.
- **Result (aim-in-wind, gusts, targets 0/8/16/24 × wind −4…+4):** **16/20 PASS**, impact **2.2–3.6 m/s**
  (robust across gust seeds), miss <0.9 m, tilt ±8°. Was 0/20 / ~6.7 m/s crash.
- **Honest limit — can't fully match the web app (~0 m/s).** The web app hits ~0 via (a) an OFFLINE golden-section
  ignition search over a ~0.1 m-wide knife-edge (its own comments say the online formula can't hit it) and (b) a
  THROTTLEABLE engine that CUTS thrust at v=0. A solid motor can do neither: a causal controller can't replay the
  flight to find the knife-edge, and can't cut, so it always slightly over/under-brakes. ~3 m/s is near the causal
  floor for this motor. Real lever to get web-app-soft on hardware = a throttleable/extinguishable landing motor.
- **Not fixed (separate, physical):** W=−4 far targets tumble at apogee — a 6 m/s air-relative crosswind pitches the
  vehicle over and the margin fin (authority ∝ V²) has none at the ~2 m/s apogee airspeed. Known coast-authority limit.
- **Files:** `Firmware/Sysiphus_Landing.cpp` (ignition + TD gate), `Firmware/sim/sim_main.cpp` (impact latch).

## 2026-06-30 — Terminal oscillation: root-caused to reference geometry, fixed with speed-blend + divert ramp
- **Deeper look:** the "sometimes" end wobble was NOT gimbal saturation (uTVC≈0.4, far from U_MAX) — it was an
  abrupt ATTITUDE REFERENCE: (a) the hard vertical-hold step at 3.5 m, and (b) `atan2(-vx,-vz)` whipping as the
  burn drives estVz→0. The rocket enters the terminal already tilted with a rate, and the ADRC overshoots.
- **Fix:** (1) blend retrograde→vertical by SPEED, not altitude: `thRetro = atan2(-vx,-vz)·w(V)`, w:1 above 6 m/s
  → 0 by 2 m/s. This de-weights atan2 exactly when it's ill-conditioned (V→0) and smoothly points vertical as the
  burn nulls velocity. (2) Ramp the divert OUT over 7→2.5 m (was a hard freeze at 3.5 m) so the rocket finishes
  correcting position with height to spare and settles vertical for the last couple metres.
- **Rejected:** raising the divert bandwidth (WCX 1.6→2.8 overshot into big tilts); an altitude-based retrograde
  blend (rotated too hard near the ground, overshot vertical, final tilt 26°).
- **Result (wind −2…+4 × targets 0/8/16/24):** 16/16 PASS, **mean |miss| 0.69 m (best yet), mean tilt 4.0°,
  terminal reversals 6 across 16 landings (mean 0.4/landing), mean peak-tilt below 3 m = 5.4°.** The residual
  T16/W4 excursion (~10° at ~3 m recovering to ~6°) is the rocket legitimately holding a retrograde tilt to kill
  the crosswind velocity, then recovering as speed→0 — a single correction, not a limit cycle.
- **Date:** 2026-06-30. Files: Firmware/Sysiphus_Landing.cpp. memory ROUND 37.

## 2026-06-30 — Terminal wobble fix + earlier legs + leg CG/inertia model
- **Requests:** (a) landings wobble near the ground; (b) model the CG/inertia change when legs deploy;
  (c) deploy legs earlier (~1 s into the burn).
- **(c) Legs earlier:** firmware now fires P_LEGS 1 s into the landing burn (legsFired flag, reset at
  ignition) instead of at touchdown — legs are extended ~pz 5–6 m before landing.
- **(b) Leg CG/inertia:** harness applies a step change when legs deploy — Iyy ×1.20 and nozzle moment arm
  ×0.90 (CG shifts aft), i.e. keff drops ~25%. The firmware rides it via its online RLS keff (b0) — the
  trace shows keff 13.4→10.7 with no loss of control, validating the acceleration-command design.
- **(a) Terminal wobble:** below 3.5 m the firmware held vertical BUT the divert kept chasing small position
  errors → th swung ±12°. Fix (matches the JS sim): below 3.5 m with |deviation|<1.5 m, freeze the divert and
  hold vertical (accept a <1.5 m residual rather than wobble). 
- **Result (wind −2…+4 × targets 0/8/16/24):** **16/16 PASS, mean |miss| 0.97 m, mean tilt 4.0° (was 10.4°,
  max 34.7°→13.8°)** — near-vertical, soft, on-target, now *with* the leg mass change and early deploy.
- **Date:** 2026-06-30. Files: Firmware/Sysiphus_Landing.cpp, sim/sim_main.cpp. memory ROUND 36.

## 2026-06-30 — Wind accuracy: aim the nominal INTO the wind + realistic gusts (matches online sim)
- **Problem:** landings not accurate in wind (miss ~1.5-2 m, tilts 40-60°). Root cause: the nominal was
  generated wind-free, so the divert had to fight the ENTIRE steady wind (large residual). The online sim
  aimed its nominal IN the prevailing wind, so its divert only fought the (small) gusts.
- **Fix:** (1) generate the nominal IN the actual wind — bisect the ascent kick per (target,wind) so the
  ballistic hits the target in that wind, then record that descent as the nominal; (2) added a realistic OU
  gust on the steady wind (`--gusts`, std 0.35·|wind|, like the online sim) so the divert is tested on gusts.
  Now the divert only corrects gusts → tight. (Also tried raising the divert bandwidth WCX 1.6→2.8: it
  overshot into big tilts — reverted; the fix is aiming, not gain.)
- **Result (wind −2…+4, 4 targets, 16 cells):** 15/16 PASS, **mean |miss| 0.97 m (max 2.68), mean tilt
  10.4°** — on par with the online sim's ~0.6–1.1 m. Tilts dropped from 40–60° to mostly <16°.
- **Edge limit:** strong headwind (−4 m/s) + downrange still fails — mostly a graceful chute-abort (soft,
  vz≈0, tumble caught by the 80° attitude guard), one hard case. Same control-authority corner the online
  sim hit (it missed ~11 m there). 
- **Practical note (real flight):** like the online sim, you generate/upload the nominal for the EXPECTED
  wind (measured pre-launch); the onboard divert then handles the gusts. Not a slider you retarget in flight.
- **Date:** 2026-06-30. Files: Firmware/Sysiphus_Landing.cpp, sim/sim_main.cpp (gusts), sim/sweep_html.py. memory ROUND 35.

## 2026-06-30 — Ported the JS sim's nominal-tracking divert -> downrange landings now match the sim
- **Problem:** firmware downrange landings were badly tilted/off (crude "regulate straight to target" divert
  arrives with leftover horizontal velocity). User: "not even close to as good as the web-app sim."
- **Fix (ported the JS sim's real divert):** (1) nominal-trajectory tracking — the divert nulls only the
  DEVIATION from a pre-planned gust-free descent (translational ADRC: ESO on dx/dvx -> desired horizontal
  accel -> thrust tilt), so it doesn't chase the whole gap and overshoot; (2) generate the nominal by a
  pure-ballistic gust-free run (`--nodivert` + `--nomout`; `--nomin` loads it); (3) divert is BURN-only
  (coast flies ballistic to the target, like the sim); (4) hold VERTICAL below 3.5 m (near touchdown
  estVz->0 makes atan2(-vx,-vz) swing sideways -> was the 70° tilt). New runtime globals: g_nomZ/X/Vx,
  g_divertOn; harness `--nomout/--nomin/--nodivert`; sweep does kick-bisection + nominal per target.
- **Result (target×wind sweep, 20 cells):** flipped from ~5/20 to **17/20 PASS**. Downrange now lands
  on-target (miss <2 m), soft (0.01 m/s), ~vertical (tilt mostly <15°, ≤27° in wind). Remaining 3 failures
  are the extreme corner (target 24 + strong headwind) — a control-authority wall, the same one the JS sim
  hit. Firmware landing quality now matches the web-app sim across the envelope.
- **Caveat:** the nominal must be generated for each target (kick bisected so the ballistic lands on target);
  the divert corrects gusts around it. Tilt in strong wind (~25°) is higher than the sim's typical few degrees
  — tunable (raise the vertical-hold altitude / terminal gains) but within the soft-landing gate.
- **Date:** 2026-06-30. Files: Firmware/Sysiphus_Landing.cpp, sim/sim_main.cpp, sim/sweep_html.py. memory ROUND 34.

## 2026-06-30 — SIL: interactive weakness explorer (target×wind sweep) + legs-visual fix
- **What:** `Firmware/sim/sweep_html.py` runs the EXACT firmware across a target×wind grid (bisects the
  ascent kick per target, then sweeps wind) and emits one self-contained `sweep.html` with TARGET and
  WIND sliders, a **PASS/FAIL heatmap** (click a cell), an outcome readout, and the animated replay.
  Added runtime knobs to `sim.exe` (`--wind --target --kick --tw --land`); made the firmware's ascent
  kick (`g_kickDeg`) and divert target (`g_targetX`) runtime-settable. Fixed the legs/chute never
  showing: logging was gated on `inFlight`, which drops the instant it lands (before the deploy frames)
  — regated to `sawBoost` (stays true through touchdown). Legs now render (45 frames).
- **Weakness the tool immediately found:** vertical (target 0) lands clean/soft across wind ±4 m/s
  (miss <1 m), but **downrange targets land badly tilted** (41–55°, some tumble) — the firmware's divert
  is a crude "regulate straight to target," which fights the vertical hold near the ground (exactly the
  overshoot the JS sim avoided with nominal-trajectory tracking). Clear next firmware improvement:
  port the sim's nominal-tracking divert. This is the SIL doing its job — surfacing a design weakness.
- **Caveat:** the divert simplification (direct regulate-to-target) is what makes downrange fail; it's a
  real firmware limitation the tool exposes, not a harness artifact.
- **Date:** 2026-06-30. Files: Firmware/sim/sweep_html.py, sweep.html, flight.html. memory ROUND 33.

## 2026-06-30 — SIL RUN: full PASS flight; 3 firmware bugs caught+fixed; animated HTML
- **Decision/outcome:** Got the SIL running on the host via `ziglang` (pip-installable C++ compiler;
  installed to a LOCAL temp dir because the MATLAB cloud drive drops the large binary). The EXACT
  firmware now flies a full **PASS** in sim: boost → coast (margin fins hold retrograde) → suicide
  burn → soft landing (≈0.01 m/s), on target (0.77 m), vertical (3.4°), apogee 56 m. Added
  `build_zig.bat` (turnkey, Python-only) and `make_flight_html.py` (animated replay HTML like the
  landing sims — truth + EKF ghost + HUD). Artifacts: `Firmware/sim/flight.csv`, `flight.html`.
- **Bugs the SIL caught (the whole point):**
  1. **Retrograde target during ascent** — `thRetro = atan2(-estVx, fmax(-estVz,0.1))` turned wind-
     induced sideways velocity into a large sideways attitude target while still climbing → margin
     fins tumbled it. Fixed: only reorient to retrograde once descending (`estVz < -0.5`), else hold vertical.
  2. **EKF dt error** — `ekfStep` runs at the 200 Hz sensor rate but `kfPredict` integrated with
     `DT_CTRL` = 10 ms (100 Hz) → filter propagated 2× too fast → `estVz` corrupted (baro kept `estZ`
     anchored, hid it). Fixed: pass the actual `DT_S` to `kfPredict`. `estVz` now tracks truth to ~0.05 m/s.
  3. **Landing-motor over-thrust** — T/W≈4.5, 1.5 s burn over-nulled the descent and bounced the
     rocket back up (non-throttleable-solid hoverslam problem). Fixed (default rocket): sized the
     landing motor (avg 20 N, ~2 s) so deceleration nulls vz ~as it burns out at the ground.
- **Verification:** deterministic (seed 1); rebuilt/rerun after each fix; VERDICT PASS.
- **Caveat:** default-rocket physics/aero are illustrative (near-neutral airframe required for margin-
  controlled descent). Validates code LOGIC + the GNC sequence, not real aero/actuators/motor curves.
- **Date:** 2026-06-30. See memory ROUND 32.

## 2026-06-30 — SIL: software-in-the-loop harness runs the EXACT firmware
- **Decision:** New `Firmware/sim/` — a host harness that compiles the unmodified flight firmware
  (`Sysiphus_Landing.cpp`) against thin Arduino/Teensy shim headers (`shims/`: Serial, Wire, PWMServo,
  MPU6050, BMP280, SD, millis/delay/digital I/O) backed by a 2D physics model in `sim_main.cpp`. The
  harness feeds synthetic IMU/baro/GPS from the true rocket state (with biases+noise), captures the
  firmware's servo/pyro outputs to drive the dynamics, auto-presses the button through the full profile,
  and logs truth-vs-EKF CSV + a PASS/FAIL summary. The firmware's `setup()`/`loop()` run unmodified.
- **Reasoning:** User wants to test the exact flight code before launch. SIL (compile the real firmware
  with a mocked hardware layer) is the standard way; far higher fidelity than re-implementing the logic.
- **Validation:** No host C++ compiler on this machine, so I couldn't RUN it here — BUT verified it
  **compiles and links** cleanly with the bundled ARM cross-compiler (arm-none-eabi-g++ -fsyntax-only on
  both TUs = 0 errors; full compile+link = 0 undefined references to our symbols; the extern interface
  state/th/estZ/setup/loop/delay/Serial/… all resolve). So a host build should succeed first try.
- **Alternatives:** Re-implement the control/EKF in Python/JS (rejected — not the exact code, the user's
  explicit requirement); Wokwi/hardware-in-loop (external, not scriptable); PlatformIO native (needs a
  host compiler too). 
- **Tradeoffs:** 2D (pitch plane + vertical), matching the sim + single-plane firmware; the aero model is
  a reasonable default, not the real airframe; physics constants must be kept consistent with the firmware
  config (default rocket set: IYY=0.012, MASS_LAND=0.80, LAND_THRUST_AVG=35 in both). Passing the SIL
  validates code LOGIC, not real aero/actuators/motor curves.
- **Risks:** Runtime flight behavior not yet verified here (couldn't run — needs a host g++; recommended
  WinLibs/MSYS2/WSL, or `pip install ziglang`). Compilation is verified; runtime is the open item.
- **Date:** 2026-06-30. See `Firmware/sim/README.md`, memory ROUND 31.

## 2026-06-30 — New-rocket landing firmware: full sim GNC port (TVC + margin fins)
- **Decision:** New file `Firmware/Sysiphus_Landing.cpp` — a from-scratch propulsive-landing flight firmware porting the
  validated sim's full GNC: pad calibration, per-axis INS/GPS Kalman filter ([pos,vel,accel-bias] for vertical z + the
  controlled horizontal x), gyro attitude, acceleration-commanded ADRC, the sim's **two-actuator** scheme (TVC gimbal
  under thrust + **forward margin-fin servo** during coast), causal suicide-burn ignition, retrograde-reorient-during-
  coast, divert, full state machine, NMEA GPS, SD logging. Chute (P_CHUTE) is abort-only. Deleted the old research-flight
  firmware (`Research_Flight.cpp`, `main.cpp`) per the user (old STS hypothesis).
- **Reasoning:** User corrected my assumption — the **new rocket has margin fins on a servo**, so it *does* have attitude
  authority during unpowered coast (the sim's canard/margin actuator). That makes the sim's exact architecture realizable:
  reorient to retrograde during coast via the margin fins, then TVC suicide burn. Single control plane (pitch), matching
  the 2D sim. The margin-fin inversion is the sim's canard inversion (deploy = 0.5 + αdes/authority, authority from
  airspeed). ADRC b0 switches by phase: keff (online RLS) under TVC, aero authority under margin fins.
- **Alternatives:** Chute-stabilized descent + landing burn (my first draft — wrong, the rocket has margin fins);
  extend SisyphusCode in place (rejected — user wanted a new-rocket build, clean file).
- **Tradeoffs:** Single plane only (faithful to the 2D sim); a real free-flight rocket also needs the yaw plane
  (duplicate ESO/inversion on a 2nd TVC axis + aero stability for coast yaw) — flagged in-file, not implemented. The
  margin inversion needs aero params (S_ref, CN, d_ref, sm_max) from CFD — tagged `<<< SET`.
- **Risks:** **NOT compile-tested, NEVER hardware-tested** — faithful port of sim logic, must be dry-run/bench-tested
  per phase before propellant. Many constants are placeholders (`<<< SET`/`<<< MISSION`). Aero stability on ascent and
  the rocket arriving controllable into coast are prerequisites the code can't supply.
- **Date:** 2026-06-30. SisyphusCode.cpp kept as the old-rocket ascent-attitude reference. See memory ROUND 30.

## 2026-06-30 — Firmware: ADRC added as a compile-selectable attitude controller (default LQR preserved)
- **Decision:** In `Firmware/SisyphusCode.cpp`, add the validated-sim **ADRC** attitude controller behind a compile switch
  `CONTROL_MODE` (`CONTROL_LQR` default, `CONTROL_ADRC` opt-in). ADRC uses a 3rd-order ESO on the attitude estimate and
  the firmware's **online keff estimate as b0** (the existing RLS identification feeds the ADRC inversion `u = αdes/keff`),
  with anti-windup (ESO driven by the measured actuator) and all existing safety shields/state-machine/logging intact.
- **Reasoning:** Reconciling the firmware with the converged sim. Key finding from reading the firmware: `SisyphusCode.cpp`
  is the *ascent attitude* controller (adaptive-LQR + recovery), **not** the landing GNC the sim implements — and the
  board has **no GPS** (MPU6050+BMP280), so the sim's EKF/suicide-burn/divert are not portable yet. The ADRC attitude law
  is the one converged, portable, high-value piece (the user's stated edge). Making it opt-in (default LQR) means **zero
  behavior change** until bench-tested, the responsible way to introduce a new control law to flight code.
- **Alternatives:** Replace LQR outright (risky — discards a flight-proven controller, changes behavior untested); port
  the full landing GNC (impossible now — no GPS, landing PCB not built); leave firmware as-is (ignores the directive).
- **Tradeoffs:** ADRC at 100 Hz uses the true control dt for the ESO (the legacy code's DT_S=5ms vs 10ms-rate mismatch is
  a separate latent issue, left untouched). ESO is theta-only (faithful to the sim); a gyro-augmented ESO could be better.
- **Risks:** **NOT compile-tested** (no Teensy toolchain here) and **never hardware-tested** — must compile + bench-test
  before flight. The documented first-launch failure (7/13/25) was *aerodynamic instability*, not the controller; per the
  project's own Finding 3, ADRC cannot rescue a divergent airframe — positive static margin is the prerequisite.
- **Date:** 2026-06-30. See README CONTROL_MODE, memory `hardware-validation`. `main.cpp` flagged as a deletion candidate
  (superseded, old pin map) — not deleted pending confirmation.

## 2026-06-30 — Strapdown gravity-projection coupling + pad calibration (calibration is the dominant lever)
- **Decision:** Make the EKF a true strapdown: the accelerometer is read in the BODY frame and rotated to world by the
  **estimated** attitude (`s[4]+th_err`), so attitude error leaks gravity/thrust into the horizontal channel — the real
  dominant INS drift, and it ties attitude error → position error. Add **pre-launch pad calibration** (estimate each
  gyro/accel bias from ~600 stationary samples, subtract → small residual) and accel **misalignment** (axial SF leaks
  into the lateral axis at high g; a static 1-g pad cal removes only its g-part). UI toggle `CALIB`.
- **Reasoning:** Two gaps flagged in prior self-reviews: (1) no attitude→position coupling (real INS drift mechanism),
  (2) attitude was modeled uncalibrated, which is both unrealistic (every controller calibrates on the pad) and made
  the tilt a worst case. Modeling both makes the sim a faithful INS and lets it answer "what actually limits a real
  flight."
- **Result (wind=4, 60 flights/cell):** Pad calibration is the **dominant lever** — no-GPS miss 2.92→0.26 m, touchdown
  tilt 4.1°→1.0°, position-estimate error 3.46→0.06 m. **GPS helps iff IMU drift > GPS bias:** uncalibrated it helps
  (2.92→0.81 m), but *calibrated it HURTS* (0.26→0.81 m) because the loosely-coupled KF gets pulled into the GPS's ~0.8 m
  correlated bias (modeled white in R, unobservable → can't be rejected). This unifies all prior GPS confusion
  (ROUND 24/25/26). Misalignment largely **cancels** over a return-to-rest hop (∫(SF−g)=Δv=0) so it doesn't dominate a
  short hop. wind=0 regression unchanged (miss 0.00, soft, vertical).
- **Alternatives:** Keep the world-frame accel abstraction (no attitude→position coupling — hides the real mechanism);
  model the GPS bias as an estimated state (unobservable without a second absolute reference — can't); skip
  calibration (unrealistic). 
- **Tradeoffs:** Per-axis decoupled KF folds the attitude-rotation error into the accel-bias state (1st-order OK). The
  calibrated-no-GPS 0.26 m is a **best case** — the model still omits accel scale-factor, vibration-induced bias,
  temperature drift, and gyro g-sensitivity, which would add to it. So "calibrated short hop barely needs GPS" is the
  optimistic end; treat ~0.5–1 m as the honest expectation with a fuller error budget.
- **Risks:** KF Q/R/P0 and the sensor σ's are hand-set (could be spec-derived from the MPU6050/u-blox datasheets);
  attitude still has no in-flight reference (correct for a rocket — no clean gravity vector in flight). **Date:** 2026-06-30.
  See memory ROUND 28, EXPERIMENTS 2026-06-30.

## 2026-06-30 — Replace ad-hoc fusion with a real EKF (fixes the vertical "lands downwind")
- **Decision:** Replace the piecemeal per-channel estimator (gyro-integrate attitude, baro-as-truth, single-gain
  complementary GPS on a drifting accel position) with a proper **loosely-coupled INS/GPS Kalman filter**: two
  per-axis 3-state filters, state `[position, velocity, accel-bias]`, that strapdown-integrate the accelerometer
  (predict) and fuse baro (every step) + GPS (8 Hz, pos+vel) with covariance-weighted gains. Attitude stays
  gyro-only (no in-flight gravity reference → no measurement to fuse). UI relabeled "EKF (IMU+baro)".
- **Reasoning:** User reported vertical launches "land a bit downwind … not tilting enough," and asked whether a
  "big filter" fused the data. Diagnosis settled it: the **aim is mathematically exact** — the ideal ballistic
  *and* divert land at x=0.00 in any wind (kick −13.7° at 3 m/s, −19° at 5 m/s). The downwind landing was the
  **estimator**, not the tilt: the ad-hoc filter's position estimate sat ~1 m downwind, so the controller flew the
  rocket to the wrong place. The user's "is it running a real filter?" instinct was the actual root cause.
- **Result:** EKF beats the ad-hoc on every axis. GPS-on: miss 0.6–1.1 m (was ~1.0), **estimate error 0.57 m (was
  ~1.0)**, smooth (11–12 reversals). The bias state estimates & removes the accel bias → no growing drift; the
  covariance-weighted gain self-tunes (low when confident → no lurching). Vertical signed landing now −0.5 m
  (slightly *upwind*, unbiased), residual ~1 m = irreducible GPS bias (1 m CEP), not tilt. GPS-off: horizontal is
  unobservable (no absolute ref) → drifts 1.6–1.8 m, as physics dictates. wind=0 regression unchanged (miss 0.00).
- **Alternatives:** Keep the tuned-down fixed-gain fusion (works but hand-tuned, doesn't estimate bias → can't
  remove drift); full strapdown EKF with body/world Jacobians + attitude fusion (overkill — no in-flight attitude
  measurement exists, so it wouldn't improve attitude); tightly-coupled GPS (pseudorange-level — far beyond scope).
- **Tradeoffs:** Per-axis decoupling assumes the attitude-rotation error folds into the accel-bias state (true to
  first order); the GPS *correlated* bias is modeled as white in R, so the KF can't fully remove it → ~1 m floor
  remains (honest — that's real consumer-GPS behavior). Two 3×3 KFs run only on the flown pass (search stays exact).
- **Risks:** KF tuning (Q/R/P0) is hand-set, not identified; could be re-derived from the real sensor spec sheet.
  Attitude still gyro-only → slow tilt drift unaddressed (would need a mag/accel-tilt reference on a real vehicle).
- **Date:** 2026-06-30. Supersedes the fixed-gain fusion entry below. See memory ROUND 27, EXPERIMENTS 2026-06-30.

## 2026-06-30 — GPS fusion gains were too high (user: "looks better with no GPS")
- **Decision:** Lower the loosely-coupled GPS gains (KP_GPS 0.4→0.12, KV_GPS 0.5→0.15) and the translational-divert
  bandwidth (wcx 2.5→1.6, ESO w0x 10→6) so GPS corrects the slow drift gently instead of slamming each noisy fix in.
- **Reasoning:** User reported the flight *looked* better with GPS off. Diagnosis (wind=3, 45 flights): GPS-on was
  *more accurate* (miss 0.84 vs 2.05 m) but made a few BIG visible ±5° correction leans; GPS-off made many tiny ±1°
  wiggles while drifting off — "calm because it's given up on the target." The high fusion gain was injecting GPS
  noise into the estimate → the divert lurched. After the fix, GPS-on has FEWER gimbal reversals than off (24 vs 33)
  AND half the miss — better on both. The perceptual "smoother" was passive drift, not better control.
- **Alternatives:** A proper EKF (covariance-weighted gain — the principled version; fixed low gain approximates it);
  rate-limit the divert lean; leave it (dishonest — it was a real over-aggressive filter).
- **Tradeoffs:** Lower gain corrects drift slower, but the drift is slow so it still bounds to ~GPS accuracy (~1 m);
  active wind correction inherently involves more *visible* motion than passively drifting off-target.
- **Risks:** Gains hand-tuned, not covariance-derived; a real EKF would adapt. **Date:** 2026-06-30. memory ROUND 26.

## 2026-06-30 — Landing sim: realistic IMU+baro sensor fusion (controller flies on ESTIMATES)
- **Decision:** The controller flies on fused sensor estimates, not truth: gyro→attitude (slow bias drift, no absolute
  ref), baro→altitude (noisy but bounded), accelerometer→horizontal velocity & position by integration (DRIFTS, no
  GPS). True state still drives physics/rendering/touchdown. UI toggle (ideal vs realistic); noise only on the flown
  pass so the search/aim stay deterministic.
- **Reasoning:** The single biggest "would it actually fly" gap (flagged in three prior self-reviews). It makes the
  sim a faithful predictor of hardware behavior and exposes the real precision limit.
- **Alternatives:** full EKF (overkill for the demo); keep ideal state (dishonest, hides the limit); model only the
  position drift (less complete).
- **Tradeoffs:** Reveals that sub-meter pinpoint is NOT achievable with IMU+baro — horizontal position drifts ~1 m
  (accel double-integration), which caps precision (ideal 0.0 m → realistic ~0.9–1.5 m). The prior sub-meter demo was
  an artifact of perfect state knowledge. Apogee/descent detection had to move to a robust baro-trend (true-vz
  threshold) because instantaneous noisy vz false-triggers on the pad.
- **Risks:** IMU modeled ~ideal in places (canard reads a clean lateral force; no attitude-error→position cross-
  coupling; horizontal position has no absolute reference at all). It shows the floor, not a full error budget.
- **Date:** 2026-06-30. See memory `hardware-validation.md` ROUND 23, EXPERIMENTS.md, TODO_AI.md P1.

## 2026-06-30 — Canard control must invert on AIR-relative flow, not ground velocity
- **Decision:** In `tools/landing_interactive.py`, the canard margin-control inversion computes its aero
  state (AoA, dynamic pressure, normal force `N`, hence the control effectiveness `den`) from the
  **air-relative** velocity `(vx − windvx, vz)`, not the ground velocity.
- **Reasoning:** The canard is an aerodynamic surface; its moment and **sign** follow the airflow. The
  physics (`derivInto`) already uses air-relative velocity, but the controller inverted on ground
  velocity. In wind this gave the wrong AoA sign → canard deployed the wrong way (pinpoint rotated the
  wrong direction); on a vertical descent the ground horizontal velocity is ~0 so the controller saw
  ~zero authority (`den≈0`) and couldn't counter the wind → the rocket was rotated off the pad.
- **Alternatives:** (a) estimate the wind/AoA with an observer (more realistic, more complex); (b) model
  an explicit AoA vane sensor; (c) leave it (broken in wind). Using the known wind is a stand-in for
  AoA sensing — the minimal correct change.
- **Tradeoffs:** Uses the true wind in the controller (slight idealization — represents AoA sensing).
  Identical at wind=0 (`avx=vx`).
- **Risks:** None at wind=0 (verified). The idealization (perfect AoA knowledge incl. gusts) overstates
  real sensing; a noisy AoA model would degrade it. TVC phases unaffected (thrust vectoring is wind-
  independent).
- **Date:** 2026-06-30. Fixed both reported failure modes; far-target+strong-headwind remains an
  authority limit. See `memory/hardware-validation.md` ROUND 21.

## 2026-06-30 — Landing sim: nominal-trajectory tracking for the divert
- **Decision:** In `tools/landing_interactive.py`, the wind divert corrects only the *deviation* from
  a stored gust-free nominal trajectory (retrograde aim), not the absolute `x → target` error.
- **Reasoning:** Regulating `x → target` directly made the divert chase the ~10 m gap a suicide burn
  naturally closes during braking, building horizontal velocity and overshooting; the search (ballistic)
  and flown (divert) trajectories diverged.
- **Alternatives:** (a) divert in both search and flown (findKick degenerate); (b) accept ~1.2 m
  retrograde-only miss; (c) boostback to above the pad + vertical descent (cleaner but bigger change).
- **Tradeoffs:** Nominal-tracking assumes the flown path stays near the nominal — breaks in strong wind
  at short/far targets. Touchdowns are firmer in wind (divert tilt steals vertical brake).
- **Risks:** Edge cases (tg20 strong wind, far+headwind) still degrade; documented as authority limits.
- **Date:** 2026-06-30. See `memory/hardware-validation.md` ROUND 20.

## 2026-06-30 — Landing sim: acceleration-commanded control + ADRC
- **Decision:** All attitude controllers command *angular acceleration* and invert through the current
  `thrust·L_nozzle/Iyy`; ADRC (ESO) is the attitude controller with anti-windup on the realized accel.
- **Reasoning:** Makes attitude control exactly invariant to the thrust curve, mass, CG, and Iyy
  migration; ADRC then estimates/cancels the wind disturbance. This is the project's core ADRC thesis
  applied to the landing demo.
- **Alternatives:** Angle-command PD (thrust-dependent gain); gain-scheduling on 1/thrust (approx).
- **Tradeoffs:** Invariance holds only when the gimbal is unsaturated; anti-windup needed for the ESO.
- **Risks:** ESO windup during actuator saturation (fixed by feeding realized accel). **Date:** 2026-06-30.

## 2026-06-25 — Paper reframed CAUSAL-FIRST; Π is a coordinate, not a law
- **Decision:** The load-bearing claim is causal (removing servo slew saturation restores PID to
  SR≈0.99 across 142 designs). Π = keff·τ² is the organizing *coordinate* that locates saturation
  onset, explicitly **not** a claimed law.
- **Reasoning:** A 12-direction novelty search gated everything to known/active work; the defensible
  asset is rigorous application + a self-correcting process, anchored to a causal result.
- **Alternatives:** Lead with the Π predictor (AUC framing invites base-rate objections) or the
  classifier (FRAGILE/EASY reads as natural classes — dropped).
- **Tradeoffs:** A coordinate is less headline-grabbing than a "law" but survives scrutiny.
- **Risks:** Π exponent (τ¹ vs τ²) is under-determined on the main population; stated as such.
- **Date:** 2026-06-25. See `CLAUDE.md` top + `memory/paper-claims-and-narrative.md`.

## 2026-06-20 — Π parameter is keff·τ², not θ̈_max·τ²
- **Decision:** Use Π = keff·lat² (keff = control effectiveness) for all theory/design; θ̈_max·lat²
  (includes max_gimbal via u_max) is stale.
- **Reasoning:** The gain ceiling is keff-independent; the floor depends on keff, not u_max. Adding
  max_gimbal only added noise (ΔCV≈−0.14). keff·τ² is theory-derivable (ceiling/floor product).
- **Alternatives:** Keep θ̈_max·τ² (intuitive but mechanistically wrong).
- **Tradeoffs:** Numerical Π thresholds from earlier θ̈-based runs are approximate pending recompute.
- **Risks:** Cross-referencing two Π scales; mitigated by flagging stale references. **Date:** 2026-06-20.

## 2026-06-15 — Robustness classification: finer gain search + ≥30 seeds
- **Decision:** Re-derive the at-risk population with a finer joint Kp×Kd search and 30 fresh disjoint
  seeds + Wilson CIs; treat gain-search adequacy and seed count as *two* independent noise sources.
- **Reasoning:** 3-seed binary tests cannot resolve a 0.80 threshold sitting between 2/3 and 1; the
  decoupled Kd search had blind spots. Each correction pass *strengthened* the central signal
  (AUC 0.943→0.957→0.975), evidence the noise was genuinely noise.
- **Alternatives:** Keep 3-seed counts (underpowered); full n=2400 rerun (too costly).
- **Tradeoffs:** First-order correction on the at-risk subset only, not a full rerun.
- **Risks:** Downstream stats on old labels go stale on each pass; tracked in CLAUDE.md. **Date:** 2026-06-15.

## 2026-06-13 — Servo slew unit-error fix + realistic design space
- **Decision:** Fix the π/180 slew-rate unit error (servos were 57.3× too slow) and restrict the
  design space to realistic hobby hardware (servo_slew 60–200 deg/s, Iyy 0.005–0.100, mass 0.5–1.2 kg).
- **Reasoning:** The bug invalidated all prior INFEASIBLE findings and the Iyy×wind boundary; with
  realistic servos, INFEASIBLE≈0 and the story becomes gain-selection / sim-to-real, not GO/NOGO.
- **Alternatives:** Keep the broad space (unphysical designs dominate).
- **Tradeoffs:** Invalidated a large body of earlier results (Exp4/Exp5 slew curves).
- **Risks:** Re-deriving everything; accepted as necessary. **Date:** 2026-06-13.

## 2026-06-02 — Migrate simulator from MATLAB to modular Python
- **Decision:** Rebuild the sim as composable Python modules (`sim/`) with a validation suite.
- **Reasoning:** Reproducibility, one-nonideality-per-module composability, validate.py guardrails;
  MATLAB scripts were monolithic and hard to share/test.
- **Alternatives:** Stay in MATLAB (57 `.m` files remain as legacy).
- **Tradeoffs:** Dual-language transition period; `.m` files now legacy.
- **Risks:** Physics-port errors → mitigated by `validate.py` conservation/stability checks. **Date:** 2026-06-02.

<!-- TEMPLATE — copy to the top for each new decision
## YYYY-MM-DD — <short title>
- **Decision:**
- **Reasoning:**
- **Alternatives:**
- **Tradeoffs:**
- **Risks:**
- **Date:**
-->
