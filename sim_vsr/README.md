# sim_vsr — Variable-Stability TVC Rocket Simulator

A purpose-built, modular **longitudinal-flight** simulator for a thrust-vectored model rocket
that can **actively change its static margin in flight** (morphing surface or moving mass) to
switch between *stable* and *maneuverable* configurations. It is separate from the project's
core attitude-hold simulator (`sim/`), which assumes a fixed static margin and gimbal-only
control.

## Why a new architecture
The core `sim/` answers "can a PID hold attitude?" for a *fixed* airframe. The variable-stability
question needs three things `sim/` does not have:
1. **Full ascent dynamics** (airspeed-dependent aero, flight-path angle, angle of attack) — the
   maneuver benefit lives in the coupling between body rotation and the velocity vector.
2. **Static margin as a controlled state**, driven by a rate-limited morph actuator.
3. **Dual control**: a TVC attitude controller *and* a stability scheduler, running together.

## Modules
| file | role |
|------|------|
| `vehicle.py`  | mass/inertia/thrust schedules, aerodynamics, planar 6-state EOM (`deriv`) |
| `actuators.py`| `TVCActuator` (slew/deadband/backlash/saturation, `slew_sat_frac`) + `MorphActuator` (rate/range-limited static margin) |
| `sensors.py`  | `IMU` (gyro/attitude bias, white noise, FIFO latency) |
| `control.py`  | `TVCController` (PID **or** ADRC/ESO) + `StabilityScheduler` (fixed_stable / fixed_unstable / switched with hysteresis) |
| `scenario.py` | `Scenario` (thrust curve, ramped attitude command, OU wind, rail-exit speed) |
| `simulate.py` | RK4 integrator + flight metrics (`SimResult`) |

## Conventions
- Planar (pitch plane). Angles from **horizontal**: straight-up launch → `gamma = theta = 90°`.
- `theta` body pitch, `gamma` flight-path angle, `alpha = theta − gamma` angle of attack.
- `static_margin` in **calibers** (>0 stable: CP behind CG; <0 unstable).
- Gimbal sign: `+delta` pitches the nose up.
- **Pi** is reported in the project's control-unit scale (`keff_cu × latency²`) so it is directly
  comparable to `Pi_crit ≈ 275`. Internally `keff` is `T·L_nozzle/Iyy` in rad/s²/rad-gimbal.

## Run it
```bash
python tools/vsr_demo.py             # 3 stability modes × 2 controllers, full physics, 8 seeds
python tools/vsr_benefit_envelope.py # WHERE switched beats fixed_stable (authority sweep)
```

## What the simulator currently shows (honest)
- In the **safe / adequate-authority** regime (Pi≈24, gentle maneuver) `switched` is **no faster**
  than `fixed_stable` — TVC alone handles it. *Not a win.*
- There is a **narrow authority-limited band** (e.g. small gimbal, moderate-large maneuver) where
  `switched` gives a **small, real benefit**: higher success and ~25% faster settle, because the
  unstable fling assists a weak TVC and the stable catch settles it.
- **Below** that band (TVC too weak) `switched` **diverges** — the fling overshoots and weak TVC
  cannot recover. Variable stability is *harmful* there.

The scientific deliverable is this **operating envelope** — a quantified map of the narrow region
where active stability modulation helps and the adjacent region where it is actively dangerous —
not a blanket "variable stability is better" claim.

## Canard model (the physical mechanism)
`vehicle.deriv` models a **deployable forward canard**: `N_c = q·S_c·CN_c·(α + δc)` at a forward
arm. The **α-term destabilizes** (deploying the canard shifts CP forward → margin modulation); the
**δc-term steers** (raw canard control). Same surface, same servo — the difference is how you drive
it. Canard authority scales with `q ∝ V²` (weak slow, strong fast); TVC scales with thrust (dead in
coast). `CanardParams.dmargin_full()` gives the static-margin reduction from full deployment.

`tools/vsr_canard_comparison.py` compares four modes (tvc_only / margin_mod / canard_only /
canard_tvc) across powered, high-speed, and coast regimes. **Honest findings:**
- **Powered, adequate TVC:** all modes ≈ equal; margin modulation gives *no* benefit.
- **High-speed, weak TVC:** raw canard control *dominates* (canard authority ∝ q keeps pace with
  the aero loads it must overcome, while fixed-thrust TVC is outscaled); margin modulation gives
  only partial help and is dominated by canard control in the same regime.
- **Coast (thrust = 0):** **only raw canard control works** — TVC and margin-modulation both fail
  because they depend on thrust. This is the decisive, demonstrable canard advantage.
- **Trade:** margin modulation is mechanically simpler (binary deploy/retract, even a solenoid;
  no fast proportional servo, no airspeed gain-scheduling) but is a *dominated strategy* — anywhere
  it helps, raw canard control helps more, and canard control uniquely adds coast/apogee authority.
  Raw canard control needs a fast, q-scheduled servo (modeled here).

## Caveats / scope
- Linear `CN_alpha` aero (no stall) — keep `|alpha| ≲ 20°` for validity; the ramped command and
  rail-exit start keep it there in nominal runs.
- Planar only (no roll/yaw coupling), point-mass + rigid-body, single fixed Kd per controller.
- The abstract `MorphActuator` path treats static margin as a free state (no surface physics) and
  is kept only for the legacy demo/envelope. The **canard model** (`deriv` + `CanardParams`) is the
  faithful path: it models the surface's own normal force, its CP-shift, and its control moment.
  Still missing from the canard model (next extensions before any hardware claim): the canard's
  **downwash on the aft fins**, the **deployment transient** (a fold-out impulse/asymmetry), and
  canard **stall** above ~15–20° local AoA — all of which would narrow the margin-modulation and
  high-α benefits.
