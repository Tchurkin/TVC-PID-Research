# Sensor Board — Design Spec (Impulse 2.1 stack)

A small round sensor PCB that lives in its **own vibration-damped mechanical module** above the
main board, connected by a flexible wire harness (locking JST-GH). Carries the *primary* attitude
+ altitude sensors; the main board keeps a BMP280 + MPU6050 as redundant backups.

Files: `Sensor_Board.kicad_pcb` (placed, netlisted, unrouted), `_generator.py` (reproduces it).

## Components (chosen)
| Ref | Part | Bus | Notes / LCSC |
|-----|------|-----|--------------|
| U1 | **ICM-42688-P** IMU (±16 g, ±2000 dps, low-noise, no fusion) | **SPI** | LGA-14 3×2.5 mm. LCSC C2996607 |
| U2 | **DPS310** barometer (~±0.06 m rel.) | **I²C** | LGA-8 2×2.5 mm. LCSC C107104 |
| C1 | 100 nF | — | ICM VDD decoupling (0402) |
| C2 | 10 nF | — | ICM VDDIO decoupling (0402) |
| C3 | 100 nF | — | DPS310 VDD decoupling (0402) |
| C4 | 100 nF | — | DPS310 VDDIO decoupling (0402) |
| C5 | 10 µF | — | local 3V3 bulk (0805) |
| FB1 | Ferrite bead (600 Ω @100 MHz) | — | 3V3_RAW → 3V3 isolation (0402) |
| R1 | 10 kΩ | — | ICM CS pull-up (keeps SPI deselected on boot) |
| J1 | **2×5 0.1″ pin header** (PinHeader_2x05_P2.54mm) | — | 10-pin ribbon harness to main board |
| H1–H3 | M3 mounting holes | — | for silicone grommets (damping) |

## Interface connector J1 — pinout (main ↔ sensor), 2×5 0.1″ header, straight-through ribbon
| Pin | Net | Pin | Net |
|-----|-----|-----|-----|
| 1 | +3V3 (RAW → FB1) | 6 | CS_IMU |
| 2 | GND | 7 | INT_IMU |
| 3 | SCK | 8 | SDA |
| 4 | MOSI (→ ICM SDI) | 9 | SCL |
| 5 | MISO (← ICM SDO) | 10 | GND |

Two grounds (pins 2, 10) bracket the signal group. **No baro-INT line** — the DPS310 is polled
(its interrupt shares the SDO/address pin, unusable as a separate INT in I²C mode); the fast IMU
gets the INT instead. Straight-through 10-wire ribbon, main pin N ↔ sensor pin N.
⚠️ Plain headers can back out under vibration — **retain the connector** (friction-lock housing,
hot-glue bead, or a lockbar/zip-tie) since this is a flight vehicle.

## Bus architecture
- **ICM-42688 on SPI** (fast, deterministic). Read on its **INT1 data-ready interrupt** — this keeps
  loop timing fixed, which matters because latency τ dominates your Π = keff·τ² gain-ceiling result.
- **DPS310 on I²C**, shared with the main board's BMP280/MPU6050 (same SDA/SCL through the harness).
  Put the I²C **pull-ups (4.7 kΩ) on the MAIN board only** — not here — so the shared bus has one set.
- **DPS310 strapping:** CSB → 3V3 (selects I²C); SDO/ADDR → GND (I²C address 0x77). BMP280 backup on
  main board should use the other address (0x76) to avoid a clash.

## Power
Sensors draw <10 mA. Take **3V3 from the main board** through **FB1 (ferrite) + C5 (10 µF) bulk** for
clean sensor power; local 100 nF/10 nF at each IC. No regulator needed on this board.

## ⚠️ Verify before fab (IC pin mapping)
The footprints are correct KiCad library parts, but I assigned pad→net using the standard
InvenSense/Infineon family pinouts **from memory** — **check U1 (ICM-42688-P) and U2 (DPS310) pad
numbers against their datasheets** before routing. Power/ground/SPI/I²C *functions* are right; the
physical pad numbering is the thing to confirm. (Dropping in the verified symbols from SnapEDA/
easyeda2kicad and re-associating is the safe way.)

## Vibration damping / mechanical
- Mount on **3× M3 silicone grommets** inside the sensor module; **main board stays rigid**.
- Tune grommet stiffness so the **mount resonance sits ABOVE your control bandwidth but BELOW the
  motor/aero vibration** you're filtering. Too soft → the board wobbles relative to the airframe →
  the IMU measures the board, not the rocket → adds phase lag (τ) → compresses the gain ceiling.
  Band-pass, not "softer = better."
- The flexible harness (not a rigid header) is what lets the grommets actually work.

## Firmware notes
- ICM-42688 SPI up to 24 MHz; use INT1 data-ready for fixed-rate reads.
- DPS310 + BMP280 + MPU6050 all on I²C — enumerate all three; use DPS310 as primary baro, BMP280 as backup.
- Log ComputeTime; a rate filter (EMA α≈1/(L+1)) on the gyro is optional per experiment G.

## Status
`Sensor_Board.kicad_pcb` = components placed inside a ~30 mm round outline + 3 grommet holes, full
netlist baked into pads (ratsnest ready), **unrouted**. Route in KiCad; verify IC pinouts first.
