# Measured τ, and what it means for the simulated latency range

**Measured 2026-08-13** on the flight vehicle with `Firmware/Bench_Tau_ICM`. This closes the last
open Sep-7 gate item and settles a question the paper has carried as an assumption since the start.

---

## 1. The measurement

Sensor path is the **ICM-42688-P on SPI1** — Ascent_TVC's primary IMU. (It contains no BNO055; the
only `bno` in that file is a local variable `bnow` in the baro scheduler.)

| term | value | how |
|---|---|---|
| ODR | **1 kHz** both gyro and accel | reg 0x4F/0x50 = 0x06, read back |
| true sample period | **0.984 ms** | minimum observed update interval |
| SPI 14-byte burst | **0.018 ms** | median of 500 reps |
| UI filter | BW code **1** = ODR/4 = 250 Hz | reg 0x52 = 0x11, **power-on default — never written by the firmware** |
| UI filter group delay | ~0.5–2 ms | datasheet lookup, the one unmeasured term |
| flight loop period | **~3.45 ms** | `CTL###.CSV` `loop_us_median`; ⚠ needs a fresh bench capture, see §5 |

**τ_sensor+compute = filter delay + half the sample period + half the loop period ≈ 2.7 – 4.2 ms**,
call it **~3.2 ms**.

**The paper assumes 0.035 s. The vehicle is 8–13× faster than that.**

That conclusion is robust to the one unmeasured term: even a 4× error in the filter delay leaves τ an
order of magnitude below the assumption, because the loop-period term dominates.

---

## 2. Is the simulated 5–30 ms range wrong?

**No — and this is the useful finding.** The range is right for the architecture it represents. The
vehicle is faster than that architecture.

**Hobby TVC overwhelmingly uses a *fusion* IMU.** BPS.space — the reference implementation in this
space — has used the Bosch BNO055 across multiple generations of its flight computers (Relay used an
LSM9DS0/BNO055; the Vector board, the first of the Signal avionics family, used a BNO055). A fusion
IMU has a τ floor set by published specs alone:

| term | BNO055 | source |
|---|---|---|
| fused output rate, NDOF | **100 Hz → 10 ms period** | Bosch datasheet; confirmed in practice |
| → mean sampling age | **5.0 ms** (worst 10.0) | half the period |
| quaternion register read | **1–6 ms** | community measurement |
| fusion filter group delay | **not published by Bosch** | — |
| loop period | not published for most projects | — |

**So a BNO055 vehicle's τ starts at 6–11 ms before you count the fusion filter or the loop.** Add a
plausible loop period and an unpublished filter delay and 15–30 ms is entirely ordinary. **The sim's
5–30 ms range is a reasonable envelope for fusion-IMU hobby TVC.**

**This vehicle is a different architecture:** a *raw* IMU at 1 kHz with attitude integrated on the
Teensy at ~290 Hz. Sampling age drops from 5 ms to 0.5 ms and there is no black-box fusion stage at
all. ~3.2 ms is not an error in the model — it is what that design choice buys.

**Recommendation: do not re-sample the design space.** The range describes the population it was
drawn for. What changes is the *paper's* claim: this vehicle sits below the sampled range, and §7
should say so as a scope statement rather than placing a marker on the axis.

---

## 3. ⚠ The simulator structurally cannot represent this vehicle

`sim/design_space.py:201` clips `latency_steps` to **[1, 6]**, and `dt = 0.005 s`. So:

- minimum representable τ = 1 step = **5.0 ms**
- maximum representable τ = 6 steps = **30.0 ms**

**τ = 3.2 ms is below the minimum the simulator can express at this timestep.** Modelling it would
require a finer `dt` and a re-run of everything downstream — not a parameter change. This is a real
limitation and belongs in §9, stated plainly.

---

## 4. Consequences

**For §5 (the gain ceiling).** `ceiling ≈ 0.0661/τ` predicts **15.7 – 24.3** at the measured τ,
against **1.9** at the assumed 0.035 s. The vehicle has roughly an order of magnitude more gain
headroom than the paper's own working assumption implied.

⚠ **This is an extrapolation, not a validated prediction.** The law was fitted over 5–30 ms. Applying
it at 3.2 ms is outside the fitted range in the direction where nothing was measured. **Do not quote
the ceiling number for this vehicle as if it were validated.** See §6 for the cheap experiment that
would fix that.

**For the firmware's `TVC_WN` question.** `Ascent_TVC.ino:269` weighs raising wn 8 → 12. The delay
argument now *supports* that — τ is far smaller than assumed, so the delay-set ceiling is far higher.
It does **not** settle it: the firmware's own stated reason for holding at 8.0/1.0 is servo backlash
and deadband, which the simulator cannot express and which this measurement says nothing about. The
binding constraint is mechanical, not delay.

**For the failure-map axis.** authority×delay goes as τ², so a 10.9× error in τ is a **~120×** error
in axis position. Any statement placing this vehicle on that axis using the assumed τ was wrong by
two orders of magnitude.

---

## 5. Still to close

1. **Fresh loop period.** ASC038's `loop_us_median = 3450` came from the SD-starved flight
   (mean 27683 µs) that the 2026-08-04 fix addressed. Bench-run Ascent_TVC, let it write a
   `CTL###.CSV`, read `loop_us_median`. This is the largest term in τ, so it is worth the ten minutes.
2. **Datasheet row.** ICM-42688-P UI filter group delay at BW code 1, ODR 1 kHz. The only unmeasured
   term.
3. **Then Fig 11** — currently specified as a bench-τ dead-time distribution, which was written for
   the actuator measurement. It needs re-specifying for this decomposition instead.

---

## 6. The experiment this measurement earns — ATTEMPTED 2026-08-13, BLOCKED

**`tools/ceiling_fast_regime.py` was written and run. Its preflight control failed, so the
extrapolation caveat in §4 STANDS and §5 must keep it.**

**Design.** Three arms separating dt from τ, because τ = latency_steps × dt and reaching τ < 5 ms
requires dt < 0.005:

| arm | dt | latency_steps | τ | role |
|---|---|---|---|---|
| A | 0.005 | 1–6 | 5–30 ms | replication — must reproduce the published −1.067 |
| B | 0.001 | 5–30 | 5–30 ms | **dt control at matched τ** |
| C | 0.001 | 1–4 | 1–4 ms | the novel fast regime |

**Result: the simulator is not dt-invariant, and the cause is specific.** Same design, same τ = 5 ms,
two timesteps:

| | dt = 0.005 × 1 step | dt = 0.001 × 5 steps | RMS ratio |
|---|---|---|---|
| noise **OFF**, Kp 5 / 20 / 60 | SR 1.0, rms 0.91 / 0.58 / 0.43 | SR 1.0, rms 0.92 / 0.46 / 0.46 | 1.01 / 0.79 / 1.06 |
| noise **ON**, Kp 5 / 20 / 60 | SR 1.0, rms 2.04 / 1.80 / 1.70 | SR 0.4 / 0.2 / **0.0**, rms 11.75 / 8.90 / 18.07 | 5.8 / 5.0 / **10.6** |

With noise off the plant, actuator and latency pipeline agree across timesteps — those are all
dt-invariant. With noise on they do not. **`sim/sensor_model.py:117` adds
`gyro_noise_std * randn()` once per STEP with no dt scaling**; its own docstring calls the units
"rad/s per sqrt step". The bias random walk on line 111 *is* scaled, by √dt, which is why only the
white-noise term misbehaves.

So halving dt does not hold the physical noise fixed, and any ceiling measured at a finer dt would
differ from the published ones for a reason that is **not τ**. The tool detects this in a preflight
and refuses to spend the compute.

**This invalidates nothing published.** Every prior result used dt = 0.005 throughout, so they are
internally consistent. It means only that the latency axis cannot be *extended* by lowering dt.

**What it would take to unblock.** Give `gyro_noise_std` a physical definition — a gyro's noise is a
density in °/s/√Hz, so the per-sample σ should scale with the sample rate rather than being fixed —
then re-validate the whole simulator at the new dt. That is a modelling change, not a scale factor,
and it would put every existing result in scope for re-checking. **Not a September job.**

## 7. The original framing of that experiment, for whoever picks it up

**Does the 1/τ ceiling law hold below 5 ms?** It was fitted over 5–30 ms and this vehicle lives at
3.2 ms. That is a focused question, not a campaign: reduce the simulator's `dt`, re-run the ceiling
protocol over an extended τ range including the fast regime, and check whether the −1.067 exponent
survives. Positive control first — it must reproduce the published exponent over the original 5–30 ms
range before its novel extension means anything.

If it holds, §5 gets a validated prediction for this vehicle instead of an extrapolation. If it
breaks, that is a more interesting result and a genuine limit on the law's scope.

---

## Sources

- [BPS.space — Avionics](https://bps.space/pages/avionics)
- [BPS.space — Signal](https://joe-barnard-oh4g.squarespace.com/signal)
- [Signal TVC review (40 Hz logging, ±5° gimbal, 13–25 Hz mount bandwidth)](https://www.rocketreviews.com/signal-thrust-vector-control-tvc.html)
- [Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [BNO055 ODR and read times (kriswiner)](https://github.com/kriswiner/BNO055/issues/4)
- [Adafruit BNO055 guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor?view=all)
