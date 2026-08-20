# Impulse 2.3 — upsized pyro switching path

Forked from the **ordered** Impulse 2.2 files (the state manufactured 2026-08-01) on 2026-08-12,
after **Q21, the pyro rail ARM switch, burned out on the bench**. Revised 2026-08-13 once the
initiator was actually measured.

## The design point moved

The initiator measures **~0.3 Ω**, not the ~1 Ω the firmware comments assumed. That puts the firing
loop at **12–17 A**, against the 2–4 A the 2026-07-23 log entry sized the board for. At 14 A held for
the firmware's 900 ms:

| part | P | ΔT over 900 ms | verdict |
|---|---|---|---|
| AO3401A SOT-23 (the part that died) | 16.5 W | ~2100 °C | obliterated — fully explains the failure |
| CSD17313Q2 SON 2×2 (original firing FETs) | 11.5 W | ~254 °C | **would also fail** on a sustained pulse |

So the whole switching path is upsized, not just the part that died.

## What changed

Nothing outside these **six** parts: a netlist diff against 2.2 shows **77 nets in both, none added
or removed, and zero membership changes outside the swapped FETs**.

| | 2.2 | 2.3 |
|---|---|---|
| Q20 main battery switch (P-ch) | AO3401A, SOT-23 | **AON6403**, DFN 5×6 — LCSC **C2760089** |
| Q21 arm switch (P-ch) | AO3401A, SOT-23 | **AON6403**, DFN 5×6 — LCSC **C2760089** |
| Q21 R<sub>DS(on)</sub> | 60 mΩ @ −4.5 V | **4.3 mΩ @ −4.5 V** |
| Q16–Q19 firing FETs (N-ch) | CSD17313Q2, SON 2×2 | **CSD17301Q5A**, SON 5×6 — LCSC **C129940** |
| Q16–Q19 R<sub>DS(on)</sub> | 42 mΩ @ 3 V | **2.9 mΩ @ 3 V** |
| loop on-resistance | 84 mΩ hot | **10 mΩ hot** |
| drain → connector | trace | trace (see correction below) |
| pyro rail copper | 1.0 mm | 1.2–2.5 mm |

At 20 A / 900 ms the two halves are **matched**: **2.4 W** in each AON6403 (Tj ≈ 44 °C; survives the
pulse *indefinitely* — Tj ≈ 121 °C even at steady state) and **2.1 W** in each CSD17301Q5A
(Tj ≈ 48 °C). Neither is the weak half.

Lower R<sub>DS(on)</sub> does **not** buy more current — at 0.3 Ω the FETs are only ~4 % of the loop,
so the initiator and pack internal resistance set it. What it buys is thermal margin, which is why
the firing FETs are the 2.9 mΩ CSD17301Q5A rather than the 9.5 mΩ CSD17302Q5A (5.3 W, Tj ≈ 84 °C —
the weak half). The 17301 also carries **414 mJ E<sub>AS</sub> vs 61 mJ**, the margin that absorbs the
inductive kick when the nichrome opens ~20 A. Same footprint, same pin map, so it cost no layout.

> **The CSD17301Q5A's 2.9 mΩ is guaranteed at V<sub>GS</sub> = 3 V.** That is the binding
> requirement — the Teensy drives these gates directly, with no gate driver. Never substitute a part
> specified only at 4.5 V. (And never a P-channel: Q16–Q19 are low-side switches with sources at GND.)

## Layout notes

- The firing FETs are placed at **rot 270** so each 4.6 × 4.7 mm drain tab sits directly over its
  connector's pin-1 pad. Same net, so the firing current gets a **zero-length connection instead of
  a trace**, and the gate lands south-east facing the incoming PYROn traces.
- They shifted north to y = 45.5 to clear a via field (CS_IMU, INT_IMU, GPS_RX, PYRO_SW) that the
  lead rows had landed on. Q17 also sits east of its connector because the PI battery header
  occupies its natural spot.
- Q21 rotated 90° with its tab north — this also improved its side clearance from 0.48 mm to ~1.4 mm.
- ⚠ **CORRECTED 2026-08-18.** This file previously claimed each drain tab *overlaps* its connector's
  pin-1 pad, making every `Net-(PnO-Pin_1)` track "continuity-sense only (~0.6 mA)" and justifying
  narrowing them to 0.3 mm. **Measured: they are 3.13 mm apart on every channel — they never
  overlapped.** Those 0.3 mm tracks were therefore carrying the full ~18 A firing current, against an
  IPC-2221 1 oz rating of 1.0 A. On re-route, the FET-tab→connector run must be ≥1.5 mm; only the
  long runs to R37/R41/R42/R43 are genuinely sense-only.

## Verification

| check | 2.2 baseline | 2.3 (re-routed 2026-08-18) |
|---|---|---|
| ERC errors | 0 | **0** |
| DRC schematic parity | 0 | **0** |
| clearance / shorts / edge / hole / mask-bridge / dangling | 0 | **0** |
| courtyard overlaps | 20 | **20** |
| pth_inside_courtyard | 14 | **14** |
| stranded pads | 0 | **0** |
| GND pour fragments (pad-less, cosmetic) | — | 3 |

Netlist vs 2.2: 77 nets both, none added or removed, **zero membership changes outside the six FETs**.

After the 2026-08-18 re-placement both counts match the 2.2 baseline exactly. Silk warnings are cosmetic.


## Re-route (2026-08-18)

The board was unrouted, 27 footprints re-placed by hand, then re-routed:

- **freerouting 2.2.4** (JRE 25, spaceless path) took 155 nets to **2 unrouted** in 2m46s. GND was
  left to the pours by exporting the zones as DSN `(plane ...)` blocks.
- The 2 it refused were both **low-current nets carrying a power netclass width**: VBAT is MAIN_PWR
  (1.2 mm) but U3 pin 5 is the buck's **ENABLE** pin (microamps, and 1.2 mm will not thread a
  SOT-23-6 pin field), and `Net-(P4O-Pin_1)` is PYRO_DRAIN (1.0 mm) but the R42 leg is the
  continuity **sense** branch (~0.6 mA). freerouting never necks below the class width. Both were
  hand-routed at signal width — VBAT with a 2-via B.Cu hop under the buck, P4O with a grid A*
  route (`_route_astar.py`, 69 segments / 3 vias, 64.6 mm) whose every segment was then re-verified
  with the exact checker.
- `_widen_rails.py` then widened 70 pyro segments. **The firing path (FET drain tab → connector
  pin 1) is now 2.50 mm on all four channels** — 4.6 A IPC steady, ~42 °C adiabatic rise at 18 A
  over 200 ms. It was 0.30 mm before (1.0 A rated), which was the defect described above.
- GND pours stitched with 39 **pour-aware** vias. A naive grid stitch was tried first and was worse
  (16 dangling vias in voids, 10 edge violations) because clearance-checking a via says nothing
  about whether copper exists there; the working version reads the filled polygons back and only
  places a via that lands inside real pour on 2+ layers.

3 pad-less GND pour fragments remain — floating copper, no stranded pads, same class of cosmetic
item this project documented on the sensor board.

## Before ordering

Read `fab/ORDER_NOTES.txt`. The thing that genuinely needs a human eye:

> **Rotations for all six FETs are unverified.** These are new custom footprints, so none of 2.2's
> hand-converged JLC rotation calibration carries over. The CPL emits Q21 = 90 (top),
> Q16–Q19 = 90 and Q20 = 180 (bottom). Check all six in JLC's preview against the silk pin-1 ticks — this
> project's notes call the JLC preview the only ground truth, and a 180° error on a pyro FET is a
> dead channel or one that fires on power-up.

Pin maps differ between the two packages, and swapping them would be fatal:

```
AON6403      pads 1,2,3 = SOURCE   pad 4 = GATE   pad 5 = DRAIN tab
CSD17301Q5A  pad 1 = DRAIN tab     pad 5 = GATE   pads 6,7,8 = SOURCE
```

Both derive from KiCad's `PQFN-8-EP_6x5mm_P1.27mm_Generic` land pattern, whose own description names
exactly these two packages.

## Firmware implication

At 14 A the wire itself dissipates **~59 W**, so it cuts far faster than the 900 ms the firmware
holds the channel on. The silicon now survives that, but holding on after the cut still sags the pack
to ~6.2–6.8 V. Measure the actual cut time and shorten the pulse; keep the burst-fire logic in
`Impulse22_PyroTest.ino` as defence in depth.

## Scripts (in order)

| script | what it does |
|---|---|
| `_mk23.py` | fork 2.2 → 2.3 (copy, rename, rewrite 214 internal references) |
| `_mk_fp_5x6.py` | generate the two 5×6 footprints into `Impulse22.pretty` |
| `_upsize_pyro_silicon.py` | schematic: swap all 6 FET symbols + pin maps |
| `_place_pyro_fets.py <ref> [--force\|--probe <rot>]` | PCB footprint swap + placement (one per process) |
| `_guard_placement.py snapshot\|repair <pcb> <json> [ref]` | **run around every pcbnew placement** — catches collateral footprint movement |
| `_route_q20.py --cut/--check/--add` | Q20 gate/source copper + removal of the obsolete VBAT stub |
| `_fab_q20.py` | patch BOM/CPL for Q20 and rezip gerbers |
| `_reroute_upsized.py --cut/--check/--add` | pyro re-route around the new packages |
| `_fix_route_conflicts.py --cut/--check/--add` | resolve collisions *between* the new tracks |
| `_widen_rails.py` | widen the rail against live geometry and the board outline |
| `_refill_zones.py` | re-pour the 4 GND zones (KiCad python) |
| `_dump_geom.py` / `_geom.py` / `_fp_extent.py` | pcbnew geometry dump + clearance checker |

`_dump_geom.py`, `_refill_zones.py`, `_fp_extent.py` and `_place_pyro_fets.py` need KiCad's
interpreter (`C:/Program Files/KiCad/10.0/bin/python.exe`); the rest run on system Python.

## Three things the routing taught

1. **Check new copper against the *pending* set, not just the existing board.** Sizing/placing each
   addition against the pre-batch board let additions collide with each other — it produced a real
   PYRO1↔PYRO_G short and a via sitting on a rail. This bit twice (also in the widening pass, which
   produced a P3O↔7.4V short and a trace 0.383 mm from the 35 mm-radius edge).
2. **Scan for legal vias; don't reason about them.** `In2` carries a board-wide horizontal signal bus
   (CS_IMU 51.564, SENS_DET 52.374, INT_IMU 53.183, MISO 54.038) whose 0.81 mm gaps fit no via. That
   single fact invalidates whole classes of route, and a 2-D scan finds it in seconds.
3. **Take footprint geometry from pcbnew, never from parsing.** A hand-rolled parser got the
   rotation *sign* wrong and reported R67's two pads with their nets swapped — which would have been
   a real short.

## Q20 — done (2026-08-17)

Q20, the main battery switch, is now the **same AON6403 as Q21**, so it shares the feeder and adds
no BOM line — and **AO3401A is gone from the board entirely**. It was a **single point of failure
for the whole avionics stack** (the Teensy that fires the chute is powered through it), and a
sustained multi-servo stall sat it at ~0.43 W in SOT-23 → **Tj ≈ 135 °C continuously**, not for a
900 ms pulse. At 4.3 mΩ that is ~0.03 W.

It moved to **(175.700, 79.500) rot 0** on B.Cu, east of the buzzer: its old pocket has only 4.52 mm
between the BUZZER's through-hole pads and R66, and a DFN5x6 needs 4.96 plus clearance. East is free
because **VBAT already left Q20 eastward** (to C39, then via In2 round to R54/U2/U3), so the drain
tab now sits on the path it already took; only VBAT_RAW lengthens (~4 mm, ≈2 mΩ). The gate takes a
two-via In1 jog, same reason as Q21's — it sits on the far side of the source-pad column.

Q20 added **zero** new DRC items: courtyard 21 and pth_inside_courtyard 18, identical to the pre-Q20
baseline.

## ⚠ Recovery note — the board was found broken on 2026-08-17

At the start of that session `Impulse_2.3.kicad_pcb` was in an **unverified, broken state**: 15
unconnected and 136 DRC errors, with Q16–Q19 displaced ~3 mm south onto the CS_IMU/INT_IMU via field
that the y=45.5 placement exists to avoid, Q21's rotation reset 90 → 0, and PI/C15/R67/R68 shifted.
The last verified-clean artifact was `drc_mag_main.json` (2026-08-15).

Recovery was: restore `Impulse_2.3.kicad_pcb.pre_mag`, re-apply the `GPS_PPS`→`CS_MAG` rename, redo
Q20. The broken state is preserved as `Impulse_2.3.kicad_pcb.corrupt_20260817` in case any of it was
wanted. **If you deliberately re-placed those parts on 2026-08-16, that work is in the corrupt file
and was discarded here — say so and it can be re-applied properly.**

The lesson is now enforced in code: a pcbnew `LoadBoard/Remove/Add/SaveBoard` cycle is **not** a safe
no-op for the rest of the board, so `_guard_placement.py` snapshots every footprint's `(at x y rot)`
before such a run and restores anything that moved.
