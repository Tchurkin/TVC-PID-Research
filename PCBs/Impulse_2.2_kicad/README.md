# Impulse 2.2 — native KiCad rebuild

Clean-room rebuild of Impulse 2.1 (EasyEDA import) on **native KiCad libraries**. Same wiring,
same placement, same board outline — none of the conversion baggage (stranded silk, broken
annotation, pad-name mismatches).

- **Netlist verified identical** to Impulse 2.1 (183 connections, checked three ways: old PCB →
  new PCB → KiCad-exported schematic netlist). ERC: 0 violations.
- **Forward/back annotation works** — schematic and PCB are properly UUID-linked. "Update PCB
  from Schematic" is safe here (unlike 2.1).
- Unrouted (as was 2.1): DRC shows only ratsnest items + 6 courtyard overlaps from the packed
  import placement — re-place/route freely, silk moves with parts.
- Custom parts live in `Impulse22.pretty` / `Impulse22.kicad_sym` (Teensy 4.1 48-pin header
  footprint, 9mm/4.0mm-pitch buzzer).

## Deliberate changes vs 2.1
- Net renames for readability: `$1N50240`→`VBAT_SW`, `$1N52591`→`BUCK_5V`, `$1N33864`→`BUCK_FB`,
  `$1N33870`→`BUCK_SW`, `$1N50177`→`PYRO_ARM`, pyro channels→`PYROn_OUT`/`PYROn_LED`,
  RGB→`RGB_R/G/B`, etc.
- Ref renames (KiCad annotation needs a trailing digit): `X`→`X1`, `Y`→`Y1`, `PYRO`→`PYRO1`,
  `Out`→`OUT1`, `MAIN`→`BAT_MAIN1`, `MAIN1`→`AUX_3V3_1`, `LED`→`LED1`, `BUTTON`→`BUTTON1`,
  `BUZZER`→`BUZZER1`, `P1O`→`P1O1`, headers get a `1` suffix.
- `BUTTON` net merge: old nets `BUTTON` + `$1N14383` were one node through the switch's
  internally-bonded pins 3/4 — now a single `BUTTON` net on SW_PUSH pad pair 2.
- Teensy pads renamed to plain `0`–`41`, `VIN`, `3V3_1/2`, `GND1-3`.
- LED footprint pad numbering follows KiCad convention (1=K, 2=A — polarity preserved from 2.1).
- U4/LM2594 pinout verified against datasheet (4=FB, 7=VIN, 8=OUT; 5/6 both to GND).

## Schematic drawing style
The schematic **reproduces the hand-drawn Impulse 2.1 layout** — the 6 subsystem boxes and their
titles ("Voltage Regulator…", "Pyros and continuity check", "UI peripherals", "Servos (9 gram)",
"External Sensor Board", "Backup Sensors"), the original wires, and label placement — rebuilt with
native symbols. Where a native symbol's pin lands slightly off the old EasyEDA pin, a short stub
wire bridges it; only genuine cross-subsystem signals use labels (as in your original). Netlist
verified **byte-for-byte identical to the PCB** (183 connections, checked against KiCad's own
exporter). ERC: 0 errors, 67 warnings — all cosmetic (off-grid endpoints from the EasyEDA sub-grid,
lib-symbol-cache notes), same class the 2.1 import had.

Generators:
- `_gen_pcb.py` — reads `../Impulse_2.1_kicad/_inventory.json`, emits the native PCB.
- `_gen_sch2.py` — reads the PCB + the old schematic geometry, emits the layout-preserving sheet.
  (`_gen_sch.py` is the earlier grid-layout version, kept for reference.)
- `_patch_labels.py` — run AFTER `_gen_sch2.py`; uses KiCad's netlist as ground truth to drop a
  label on any pin a sub-grid stub missed (currently the 3 RGB-resistor pins).
- `_sym_uuids.json` links schematic symbols to PCB footprints; `_pin_nets.json` feeds the patcher.

Regen recipe: `python _gen_pcb.py && python _gen_sch2.py && python _patch_labels.py`

## Power redesign (2026-07-02)
Replaced the 150 kHz LM2594 circuit + DROK module + OR-diodes + 5 selector jumpers with two
1.1 MHz synchronous bucks (`_redesign_sch.py` / `_redesign_pcb.py`):
- **U2 = AP63205WU-7** (5 V / 2 A, TSOT-23-6) → `5V-CLEAN` (Teensy + sensors)
- **U3 = AP63305WU-7** (5 V / 3 A, same family/pinout — *verify pinout vs datasheet before fab*)
  → `SERVO_EN1` jumper → `5V-DIRTY` (servos)
- Each: 10 µF in (C1/C2), 100 nF BST (C3/C4), 6.8/4.7 µH SRN6045 (L1/L2), 2×22 µF out (C5–C8).
- **C10 = 220 µF** bulk on the pyro 7.4 V rail (was missing entirely).
- Jumpers kept: `PYRO_ON1` (pyro arming), `SERVO_EN1` (servo disable), SW1 master switch.
  Removed: `DROK_exclusive1` (bridged clean↔dirty!), `LM2594_enable1`, `logic_power_reg1`,
  `servo_power_reg1`, `power_tap1`.
- Net `Net-(SW1-B)` → `VBAT`. ERC 0/0; sch==pcb verified (204 connections).
