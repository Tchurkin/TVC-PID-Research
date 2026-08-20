# Impulse 2.3 — resumable checkpoint

**Purpose of this file.** Everything needed to pick this board up cold, without the conversation that
produced it. If you are a future session: read this, then `README.md`, then `fab/ORDER_NOTES.txt`.
Re-run `python _checkpoint.py` before trusting any statement below — it re-derives the verified state
from the board itself and fails loudly if anything regressed.

---

## Where this stands

**DO NOT ORDER.** *(2026-08-20: blocker 1 FIXED, blocker 2 and the doc/CPL items still open.)*

**FIXED 2026-08-20:** a 2.0 mm F.Cu run at y=41.5 now parallels the In1 2.0 + In2 1.4 pyro
distribution rail, bonding directly to all four through-hole P#O.2 pads. Combined cross-section
**0.1217 mm2 = 3.48 mm of 1 oz**, which clears the 3.44 mm target (was 1.48 mm = 43%). The chute
drain leg went 2.00 -> 3.00 mm. DRC back to baseline exactly: 63 violations, all accepted classes,
0 clearance, 0 parity, 3 unconnected.

**STILL UNDERSIZED (measured, not guessed):**
-  necks to **1.20 mm F.Cu over 2.5 mm** entering Q21. Blocked on three sides by Q21 pad 4
  (PYRO_G), a CS_MAG via, and the PYRO_G track; nothing wider than 1.20 fits. Fixing it means moving
  the CS_MAG via or rerouting PYRO_G. **This carries the full 18 A and is the worst remaining leg.**
-  has one **2.00 mm x 0.6 mm** B.Cu segment at (137.600,48.500)->(137.150,48.950)
  that had no room to widen.
- The **GND return is still unsized** -- Q18 returns 18 A through 0.0262 mm2.
- VBAT (In2 1.20 mm) and 5V-DIRTY (In2 1.20 mm, 2.8 A) unfixed.
- All the doc/CPL/part-identity findings in AUDIT_2026-08-19.md are unfixed.

⚠ **New tooling gap found the hard way:**  does not model ZONES. Adding the F.Cu rail
produced six zone-clearance violations that my pre-write check passed. **Always refill zones and
re-run kicad-cli DRC after adding copper** -- the clearance checker alone is not sufficient.
 A six-lens adversarial audit on 2026-08-19 found confirmed blockers. Full capture:
`AUDIT_2026-08-19.md` (40 findings, 22 verified verdicts). The headline ones:

1. **Inner-layer copper is 15.2 um, not 35 um.** JLC's 4-layer stackup uses 0.5 oz inner foil. The
   sizing work below treated In1/In2 as 1 oz, so the pyro distribution rail to P2O/P3O/P4O -- which
   includes **the chute** -- has **43%** of the cross-section the design's own criterion requires.
   Every "effective width" figure in the table below that leans on an inner layer is overstated.
   Same root cause hits VBAT (23 mm of 1.20 mm In2) and 5V-DIRTY (18.7 mm carrying 2.8 A).
2. **The GND return was never sized.** Q18, the chute FET, has a return of 0.0262 mm^2 -- 4.2x worse
   than the other three channels and 22% of its own drain feed.
3. **The gerbers in `fab/` predate the 2026-08-19 re-sizing.** Ordering that zip fabricates a board
   without any of the pyro current-path work. (`_checkpoint.py` now catches this.)
4. **The "3.13 mm apart" correction is itself a tool artifact** -- the real FET-tab-to-connector-pad
   gap is **0.631 mm**. So both the original claim AND its correction were wrong.
5. **The CPL rotations quoted in the order docs do not match the CPL file** for Q20 and Q21 -- the
   exact two values a human is told to verify in JLC's preview. Q20's documented position is stale
   too (3.79 mm east, 3.14 mm north of where the docs say, rot 90 not 180).
6. **`TI_SON5x6_Q5A`'s silk pin-1 index and F.Fab chamfer mark pad 5 (the gate), not pad 1** -- which
   defeats the visual rotation check it exists to support.
7. **CPL Mid X/Mid Y for 14 THT parts is the raw footprint anchor** -- `_regen_cpl.py` dropped the
   position correction the 2.2 order carried.
8. **Part-identity mismatches:** the TI footprint description names CSD17302Q5A / C553151 while the
   fitted part is CSD17301Q5A / C129940; Q22's symbol says IRLML6244 while its value and BOM say
   IRLML6344, with no LCSC code to disambiguate.
9. **2.9 mOhm is the TYPICAL, not the guaranteed, value at V_GS = 3 V** -- the datasheet's max in
   that row is **3.7 mOhm**. Stated as "GUARANTEED" throughout the docs below. Thermal margins were
   also signed off using datasheet RthJA, which assumes 1 in^2 of copper; this board gives the tabs
   25-61 mm^2.
10. **Q21 has never been analysed for a two-channel concurrent fire**, which the flight firmware
    actually commands.

Everything below this section predates the audit and is **not** to be trusted where it conflicts.

## Why 2.3 exists

Q21, the pyro rail arm switch on the fabricated 2.2, **burned out on the bench**. Root cause: the
initiator measures **~0.30 Ω**, not the ~1 Ω the firmware comments assumed, so the firing loop is
**12–18 A**, not the 2–4 A the board was designed for. At that current the AO3401A in SOT-23
dissipated ~16.5 W in a package rated for well under 1 W. The original firing FETs (CSD17313Q2,
SON 2×2, ~11.5 W) would have failed next. So the **whole switching path** was upsized, not just the
part that died.

**Design point (Braxton's ruling, 2026-08-19):** assume the nichrome **never clears**. The on-time
alone bounds the energy. Flight firmware commands **900 ms**, so 900 ms at 18 A is the worst case
everything is sized against. Do **not** shorten the pyro duration to make the numbers easier — a
failed cut is far worse than a warm trace.

## What changed vs 2.2 — exactly six parts

A netlist diff shows **77 nets in both revisions, none added or removed, and zero membership changes
outside the six swapped FETs.** Nothing else about the board's function moved.

| | 2.2 | 2.3 |
|---|---|---|
| Q20 main battery switch (P-ch) | AO3401A SOT-23, 60 mΩ | **AON6403** DFN 5×6, 4.3 mΩ @ −4.5 V — LCSC C2760089 |
| Q21 pyro arm switch (P-ch) | AO3401A SOT-23, 60 mΩ | **AON6403** DFN 5×6, 4.3 mΩ @ −4.5 V — LCSC C2760089 |
| Q16–Q19 firing FETs (N-ch) | CSD17313Q2 SON 2×2 | **CSD17301Q5A** SON 5×6, 2.9 mΩ **guaranteed @ V_GS = 3 V** — LCSC C129940 |
| Q22 buzzer | IRLML6344 | unchanged |

Q20 was upsized to match Q21 at Braxton's instruction — the two switches must not bottleneck each
other. At 20 A / 900 ms: AON6403 ≈ 2.2 W, CSD17301Q5A ≈ 2.1 W (Tj ≈ 48 °C). Matched within 15 %.

The 2.9 mΩ figure is chosen deliberately: the Teensy drives those gates **directly at 3.3 V**, so the
part must be specified at V_GS = 3 V, not the 4.5 V or 10 V row a datasheet leads with.

**Sensor interface is untouched** — 2.3 works with the existing v1 sensor board *and* the v2
magnetometer board, because the only sensor-side change (ribbon pin 8 GPS_PPS → CS_MAG) is a net
rename, not a copper change.

## Copper sizing — and why IPC is the wrong rule here

For an 18 A **pulse**, IPC-2221's continuous-current table demands ~16 mm of trace, which is absurd on
a 70 mm board. The governing criterion is **adiabatic rise over the pulse** (`dT/dt = rho*J^2/c_v`).
That is a *conservative upper bound*: FR4's thermal diffusion length over 900 ms is ~0.34 mm, and the
GND plane sits ~0.2 mm below the trace, so the plane starts absorbing heat well within the pulse.

| leg | effective width | dT @ 500 ms | dT @ 900 ms |
|---|---|---|---|
| PI to Q21 | 2.5 mm straight trunk | 84 °C | 151 °C |
| Q21 / C10 to P1O.2 | 2.92 mm | 77 °C | 139 °C |
| distribution rail to P2–P4 | 3.4 mm (In2 1.4 in parallel with In1 2.0) | 57 °C | 103 °C |
| P#O.1 to FET tab | 3.40 mm uniform | 57 °C | 103 °C |

Two rules were adopted after they were violated once each:

1. **One width per contiguous run.** Per-segment maximising produced runs like
   `3.40/2.80/3.00/2.80/2.50/2.80/3.00` with 0.15 mm segments — A-star corner artifacts, not intent.
   Width now changes only at genuine branches. Three mid-run changes remain, each one forced and
   documented in `fab/ORDER_NOTES.txt`.
2. **A floor, always.** The uniformity rule without a floor dropped a 47 mm VBAT run from 1.2 mm to
   **0.40 mm**, below its 1.0 mm requirement, purely for tidiness. `_uniform_runs.py` now leaves a
   run ragged and reports it rather than narrowing it to fit.

Sense / gate / decoupling stubs (µA–mA) are at 0.30 mm. Narrowing 76 of them is what freed the space
that made the real widening possible.

## Decisions already taken — do not re-litigate

- **No pyro fuse.** A shorted FET draws the same 18 A as a deliberate fire, so no rating discriminates
  between them, and the fuse would sit in the chute-deployment path. Rejected on purpose.
- **Main-battery fuse optional**, inline in the pigtail if wanted — not on the board.
- **PI (pyro battery in)**: soldered pigtail + XT30. The JST-XH housing is a 3 A part; the battery
  leads themselves are keyed JST, which is fine, but 18 A wants a soldered joint.
- **Firmware `RAIL_MIN_V` needs no change.** An earlier claim that it would abort a fire mid-pulse was
  **wrong**: `PyroTest` samples `railV()` only *after* the channel is driven low plus 15 ms settle
  (i.e. unloaded), and `Ascent_TVC` uses `PYRO_V_MIN_V = 6.80` purely as an arm-time gate / LED / log,
  never as an in-fire abort.

## Verified state

Regenerated by `_checkpoint.py`. Anything outside the accepted baseline fails the script.

<!--VERIFY-->
```
Impulse 2.3 checkpoint -- 2026-08-19

geom.json   407 pads, 851 tracks, 100 vias  (regenerated from pcbnew)

DRC   schematic parity : 0 OK
      unconnected      : 3 (accepted 3)
      courtyards_overlap    : 20 (baseline 20)  ok
      lib_footprint_mismatch: 14 (baseline 14)  ok
      pth_inside_courtyard  : 14 (baseline 14)  ok
      silk_over_copper      : 4 (baseline 4)  ok
      silk_overlap          : 11 (baseline 11)  ok

ERC   errors           : 0 OK

fab   fab/Impulse_2.3_BOM.csv            STALE -- older than the board  ***
fab   fab/Impulse_2.3_CPL.csv            STALE -- older than the board  ***
fab   fab/Impulse_2.3_gerbers.zip        STALE -- older than the board  ***

RESULT: *** SOMETHING REGRESSED -- do not order ***
note: checkpoint: board frozen for final audit
```
<!--/VERIFY-->

**What the accepted baseline items are, and why they are accepted:**

- `courtyards_overlap` (20) and `pth_inside_courtyard` (14) — identical to the fabricated 2.2, which
  works. Through-hole connectors deliberately overlapping SMD courtyards.
- `lib_footprint_mismatch` (14), `silk_overlap` (11), `silk_over_copper` (4) — cosmetic, pre-existing.
- 3 unconnected — GND pour **Zone-to-Zone** fragments with **zero stranded pads**. No pad is
  unconnected.

## Tooling — and the traps in it

| script | what it does |
|---|---|
| `_checkpoint.py` | **run this first.** Regenerates geom + DRC + ERC, checks against baseline, stamps this file |
| `_dump_geom.py` | dumps pad/track/via geometry **from pcbnew itself** into `geom.json` |
| `_geom.py` | clearance checker: pads as **oriented rectangles**, `via_ok` checks all four layers |
| `_classify_nets.py` | splits a power net into current-carrying **core** vs sense/gate **stubs** |
| `_size_by_current.py`, `_uniform_runs.py` | trace sizing, with the floor rule |
| `_stitch_gnd.py` | **pour-aware** GND stitching |
| `_route_astar.py` | grid A-star for connections freerouting refuses |
| `_guard_placement.py` | snapshot/repair around any pcbnew placement run |
| `_fab_q20.py`, `_regen_cpl.py`, `_mk_fab.py` | fab package generation |

**Traps that have already cost real time — do not rediscover them:**

- A hand-rolled s-expression parser got the pad **rotation sign** wrong and reported R67's pads with
  nets swapped. Take geometry from **pcbnew**, never from a hand parser.
- `LoadBoard / Remove / Add / SaveBoard` in pcbnew is **not** a safe no-op, and `ImportSpecctraSES`
  mistypes later `Get*` calls. **Unroute by text surgery**, never with a `Remove()` loop.
- A naive grid GND stitch took DRC from 4 to 19 unconnected and left 16 vias dangling in voids.
  `via_ok` proves *clearance*; it says nothing about whether copper *exists* there.
- A widening pass sized every segment against **pre-pass** geometry and produced a real short.
  Check against **live** geometry, include a board-edge test, and never narrow below the original.
- `_geom.py` is ~60 µm **stricter** than KiCad DRC. KiCad DRC is authoritative.
- An earlier bottleneck helper bonded pads only at trace **endpoints**, so it falsely reported
  "NO PATH" when a trace joins a pad by overlapping its body.
- A 1.7 mm vertical track's 0.85 mm end cap extends past its stated end point: one ending at
  y = 49.3 actually reaches 50.15 and collides with PI.2 GND at 50.404. y = 49.2 is the limit.
- freerouting 2.2.4 needs **Java 25**, and the DSN path must contain **no spaces**.

## Claims that were checked and found FALSE

Recorded because each was believed, written into the docs, and used to justify a decision.

1. **"The FET drain tabs overlap the connector pin-1 pads."** They are **3.13 mm apart** on all four
   channels. This false claim had been used to justify narrowing every `Net-(PnO-Pin_1)` to 0.30 mm
   as "sense only" — traces that were actually carrying the full 18 A. Now 3.40 mm.
2. **"R68 blocks a thicker PI to Q21 trace."** Stated twice, wrong twice. The real obstruction was a
   **GND stitching via at (161.700, 47.900)** placed by my own script. Removed; PI to Q21 is now a
   single straight 2.5 mm trunk.
3. **"P1/P2 failing means a shorted FET / fire-on-arm hazard."** `FAILURE_MODES.md` G8b already
   documented a bench GROUND_TEST that **fired P1 and P4 on 2026-08-15**. A spent (open) wire reads
   identically to the fault I claimed.
4. **"The battery inputs are unkeyed."** Read from a footprint name. Braxton: the leads are keyed JST.
5. **"`RAIL_MIN_V` would abort a fire mid-pulse."** See above — it never samples under load.

**Braxton's visual review has caught four things the numerical checks missed** (the false tab overlap,
the GND via blamed on R68, pinched pad entries, and a half-covered fix for them). His eyes on the
board are not a formality; they are part of the verification.

## Residual risk — what is NOT established

- **The six FET rotations are unverified.** Braxton checks these in **JLC's placement preview** before
  ordering. A rotated DFN is a dead board. This is the single largest open risk.
- **The board has never been fabricated.** Every thermal number is calculation, not measurement.
- **The 900 ms / never-clears assumption is deliberately pessimistic**, but the real melt time
  (Braxton estimates ~0.5 s for thin nichrome) has not been measured.
- Nothing here substitutes for the **on-arrival bench test**: fire every channel into a dummy load,
  and run pin-1-to-GND on all four channels of the 2.2 board, **before** fitting a live initiator.

## Next actions

**Braxton's:**
1. Verify the six FET rotations in JLC's preview.
2. Finish the KiCad visual review.
3. Solder the PI pigtail + inline XT30.
4. On arrival: every channel into a dummy load before any live initiator.

**Open / recommended:**
- Add the `pyro_min_v` log-reading trap note to `FAILURE_MODES.md`.
- Regenerate the fab package whenever the board changes — `_checkpoint.py` fails if it is stale.
