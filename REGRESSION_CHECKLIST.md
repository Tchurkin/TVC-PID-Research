# REGRESSION_CHECKLIST.md — Verify After Relevant Changes

> Run the items whose **Trigger** matches your change before considering it done. Paste the actual
> output when reporting. Add a new row whenever new user-visible functionality is introduced.

Last updated: 2026-06-30

---

> ⚠️ **Baseline as of 2026-06-30: `validate.py` is 7/9** — two **stale** checks fail (stability-physics
> 2a uses legacy `p_unstable`; component-isolation 3c disturbance-RMS is inverted). See TODO_AI.md P0.
> Until those are fixed, "validate.py passed" means *the other 7*; do not read green into 2a/3c.

## Core simulator (`sim/`)
| Check | How | Trigger |
|------|-----|---------|
| Validation suite: no NEW failures (baseline 7/9) | `cd sim && python validate.py` → still 7 pass, same 2 stale fails | any change under `sim/` |
| Determinism | same seed → identical trajectory (covered by validate.py) | RNG / integrator / control change |
| Stability physics | uncontrolled unstable diverges, stable damps (validate.py) | plant/aero/controller change |
| Component isolation | each nonideality off → predictable behavior (validate.py) | fidelity / module change |
| Mini-Exp1 smoke (n=20) | no systematic failure (validate.py) | `experiment_runner` / design_space change |
| `build_*` factories unchanged in signature | grep callers in `tools/` still pass | editing `design_space.py` |

## Experiment scripts (`tools/`)
| Check | How | Trigger |
|------|-----|---------|
| Output CSV regenerates with same schema | run the script; diff header vs prior CSV | editing an experiment |
| Seeds disjoint from prior passes | check script header seed range | any re-derivation |
| Provenance line printed (n, seeds, key metric) | run and read stdout | new/edited experiment |
| No stale numbers cited downstream | grep CLAUDE.md / memory / paper for the old value | a headline number changed |

## Landing sims (`tools/landing_interactive.py`, `landing_animate.py`)
| Check | How | Trigger |
|------|-----|---------|
| HTML regenerates | `python tools/landing_interactive.py` (or `landing_animate.py`) | any edit |
| JS parses | extract `<script>`, `node --check` the harness | JS edit |
| Vertical (wind=0) lands soft on pad, vertical | headless Node: miss≈0, vz∈[-2,0], th≈0° | guidance/control edit |
| Pinpoint lands on target, engine-down | miss < ~1 m at wind=0; report vz, th | guidance/divert edit |
| In wind: mid/far targets on-target (miss<1.5 m), vertical-ish | 15 gust realizations, report mean/max | wind/ADRC/divert edit |
| Coast flip overshoot small | trace coast θ past retroHold (report degrees) | coast-flip edit |
| No clamp/loiter at touchdown; tail rests on ground | last frame z·ppm ≈ BL | render / Z_TD edit |
| Recompute responsive (<~1 s warm) | time a warm recompute | search / per-step cost edit |
| `landing_animate.py` prints landed_vz / x_td / th_td | run it; values match subtitles | param/subtitle edit |

## Firmware (`Firmware/`)
| Check | How | Trigger |
|------|-----|---------|
| Safety interlocks / arming / failsafe intact | review diff against the interlock | any firmware edit |
| Claim traces to evidence | cross-check `FIRMWARE_VALIDATION_RESULTS.md` | new firmware claim |
| Sim/bench prediction exists first | simulation-first (AI_RULES §9) | hardware-facing change |

## Docs / provenance
| Check | How | Trigger |
|------|-----|---------|
| Paper pair in sync | update `Paper_Condensed.md` when the draft changes (same session) | paper edit |
| Subtitles/readouts match measured values | compare to the run's printed numbers | demo output change |
| DESIGN_LOG / EXPERIMENTS updated | an entry added | meaningful decision / new experiment |
