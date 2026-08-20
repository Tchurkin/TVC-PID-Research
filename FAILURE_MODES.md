# FAILURE_MODES.md — next flight (Impulse 2.2)

> Written 2026-08-04 after ASC038. One row per way the flight can fail, with **who catches it**.
> A mode is only "mitigated" if something *automatic* prevents or detects it — a procedure you might
> forget is not a mitigation, it is a hope. Modes the SIL cannot express are called out explicitly,
> because that is exactly the mistake that let ASC038 fly (see `DESIGN_LOG.md` 2026-08-04 pm).

**Status key** — 🟢 automatic mitigation · 🟡 detected but not prevented · 🔴 neither: hardware/procedure only

---

## A. Electrical / power

| # | Failure | Status | Detail |
|---|---|---|---|
| A1 | Pack too flat to fly | 🟢 | `VBAT_SENSE` pre-arm gate at **7.20 V** refuses to ignite. SIL: 7.1 V and 6.6 V both held on the pad. |
| A2 | Pack sags in flight under servo load (**ASC036/ASC038 prime suspect**) | 🟡 | `vbat_min_v` sampled at loop rate, written to the CTL header. SIL: a 7.5 V / 0.6 Ω pack with a stalling servo passes the pad gate then sags to **6.51 V** — now recorded instead of invisible. Cannot be *prevented* in firmware. |
| A3 | Servo 5 V rail collapses | 🟡 | `servo_min_v` logged; pre-arm gate at 4.50 V. |
| A4 | Pyro rail too low to heat a melt wire | 🟢 | `PYRO_V_SENSE` pre-arm gate at 6.80 V. |
| A5 | MCU brownout → reset in flight | 🟢 | `SRC_SRSR` reset cause + EEPROM phase → prints `*** BROWNOUT: power lost MID-FLIGHT ***`; EEPROM resume protects the chute. **Now actually tested** (2026-08-09): the harness snapshots the world, exits, and relaunches, so the rebooted firmware gets genuinely fresh globals. Reset swept across the whole flight — coast/apogee/descent resets all PASS with the chute at ~57 m; boost resets tumble (resume never restarts TVC, by design) but recover the airframe. Chute never fires under thrust. `RESUME_MAX_BOOTS=3` verified to stop retrying. **This found A5b.** |
| A5b | **`looksAirborne()` blind at burnout** | 🟢 | Found 2026-08-09, **fixed**. It took min/max across a 400 ms window and asked whether the *whole* window was free-fall or the *whole* window was thrust. A window straddling burnout is neither, so both tests failed and the vehicle — at **37 m climbing at 19.7 m/s** — was declared to be on the ground; the chute never came out. It was the only reset time in the entire flight that lost the vehicle, and it sat exactly at burnout: peak servo current, the likeliest moment to brown out. Now judged **per-sample** (thrust *and* free-fall both deviate from 1 g), sustained over 60% of the window. |
| A6 | Battery connector intermittent under vibration | 🔴 | **Hardware.** Nothing detects a momentary disconnect except as a brownout after the fact. |

## B. Pyro / recovery

| # | Failure | Status | Detail |
|---|---|---|---|
| B1 | Chute igniter open / not connected | 🟢 | `PYRO4_SENSE` continuity gate **refuses to arm**. SIL verified: held on the pad. This alone justifies 2.2. |
| B7 | Marginal pyro joint reads as "ok" | 🟢 | Only P4 and P1 were ever reported, and only as booleans — so a channel at 0.95 V and one at 1.85 V looked identical against the 0.90 V threshold, though the first is a joint about to open in flight. `printPyroStatus()` now prints **all four channels with raw sense volts**, live at ARM (2 s cadence, so you can re-crimp and watch it change), at the pre-arm gate, and in the CTL header. Measured with the FETs **off** — the sense circuit runs ~0.6 mA through the initiator via the 10k and the LED clamp — so nothing is energised to read it. |
| B8 | Bench-testing the fire path needs live pyros | 🟢 | `GT_PYRO_LIVE` releases the **recovery** channels under `GROUND_TEST` so the FET switching, the 1000 ms pulse and the post-fire continuity transition (CONT → open) can be exercised for real. **P3 is excluded unconditionally** — `pin != P3` at the driver, so "a ground test never lights a motor" is a property rather than a promise. Gate-locked both ways: `p3_ever_high=0` while `gt_attempt` confirms P3 *was* requested and refused, and the recovery channels *do* fire so the test isn't vacuous. `Firmware/Impulse22_PyroTest` remains the better tool for proving the hardware alone (single channel, hold-to-fire, dummy-load procedure). |
| B2 | Legs igniter open | 🟡 | Checked and reported, deliberately **not** gated — legs are not survival-critical and a false abort is worse. |
| B3 | Pyro commanded but wire never burns | 🟡 | Post-fire continuity in the CTL header. `P4=1` after firing = the wire is still intact = it never got hot. Distinguishes "firmware didn't fire" from "hardware didn't respond" — the exact ASC038 unknown. |
| B4 | **Canopy released too low** (ASC038: ~1.5 m) | 🟡 | Deploy timer bounds it at TVC-stop +`DEPLOY_AFTER_TVC_S`, and the 300 ms of blocking beeps now run *after* the pyro. **MEASURED 2026-08-16 (Braxton, bench): melt-wire → canopy clear ≈ 600 ms — `n=1`.** This was the dominant unmeasured term and the #1 item on the hardware list. Consequences: (a) the 1000 ms `PYRO_MS_CUTTER` pulse already clears it by 400 ms, so **there is no case for raising it** — see B11, the risk runs the other way; (b) at apogee the vehicle is near zero vertical speed, so 600 ms costs only ~1.8 m of altitude, against ASC038's ~18.5 m where the same delay was spent in fast descent. **Caveat: one observation.** It bounds nothing about spread, and the flight-relevant quantity is the worst case, not the typical. Repeat with the chute packed as flown before this is treated as characterised. |
| B11 | **Chute FET over-fired by a slow SD write** (found 2026-08-16) | 🟢 | A pyro gate only drops when `updatePyros()` runs *after* its stamped duration expires. `dumpFlightLog()` and `dumpControlLog()` tick it inside their write loops; **`dumpMotorLog()` did not** — and it runs between them on the recovery path, immediately after `triggerPyro(PYRO_CHUTE)`. At ASC038's measured 143 ms per SD write, a few hundred motor rows hold the chute FET on for **seconds** past its 1000 ms. That is the 11.5 W / ~254 °C budget the 2.3 README says destroys a CSD17313Q2, landing on **P4 — one of only two proven channels, with no spare**, and it is the leading explanation for how Q16 died on P1. Now ticks per row. Note the cause: the shared `void updatePyros();` forward declaration sits ~160 lines *below* `dumpMotorLog()`, so this dump could not have called it even if someone had tried — the two dumps that do tick it are both defined after that line. **Over-firing, not under-firing, is the failure mode on this channel.** |
| B5 | Chute deploys during fast ascent → shredded | 🟢 | Deploy timing chosen from SIL dispersion sweep: worst case +8.6 m/s at 5.65 s (vs +19.3 m/s if fired at TVC cutoff). |
| B6 | Chute tangles / packing failure | 🔴 | **Hardware.** No sensing possible. |
| B9 | **Health-fallback deploy fired into a climb** | 🟢 | The `healthLevel >= HEALTH_FALLBACK && !poweredFlight` branch fires the moment `poweredFlight` drops — i.e. **at burnout**, ~+42 m/s on the 0.575 kg airframe — and was the only deploy trigger with no speed or direction guard at all (the baro path requires `vert_vel < 0`, the TVC-stop timer requires `!thrusting` and waits 5.5 s). Any of `F_ATT_STALE`, `F_NAN_ATT`, `F_OSCILLATION` raised during boost reached it, and the staleness detector is code rewritten 2026-08-15. Same shred that `DEPLOY_AFTER_TVC_S` was raised 1.0 → 5.5 to prevent, through a door that never got the guard. Now requires descending. ⚠ **The first fix used +8.0 m/s and was measured wrong** — it deployed at a true **+10.0 m/s** because the guard reads `vert_vel`, the firmware's own estimate, which is precisely what is degraded in this branch. A threshold set at the value you are willing to tolerate has no margin left for the error in the number it is testing; requiring *descending* spends that error as margin. Can only ever delay toward the better-tested triggers, never skip recovery. |
| B10 | **`emergency()` deploys with no velocity guard and no thrust inhibit** | 🟡 | **RULED, 2026-08-16 (Braxton): this is intended and must stay.** His words — *"emergency shouldn't be gated by velocity. it should ALWAYS fire if the rocket is tumbling"*, and separately *"im fine with parachute firing on ascent if its past 45 degrees"*. So every abort path fires `triggerPyro(PYRO_CHUTE)` immediately, under live thrust, at any speed, and that is the accepted trade ("losing the mission beats losing the airframe", C2/C2b). **What it costs, measured, so nobody rediscovers it as a bug:** on the C2e positive-control case the chute deployed at **+33.7 m/s climbing, 141.9 m of a 193.6 m apogee** — very likely a shredded canopy. Locked in `regression.py` 5h with an explicit note that anyone "fixing" the climbing deploy is reversing a decision, not repairing a defect. 🟡 rather than 🟢 only because the canopy is not expected to survive it. |

## C. Control / estimation

| # | Failure | Status | Detail |
|---|---|---|---|
| C1 | Loop starved → estimator freezes (**ASC038 root cause**) | 🟢 | SD writes removed from the control loop; `dt` clamped not skipped; `dt_long`/`dt_dropped` in the CTL header. SIL: **identical PASS at 0/143/300/600 ms** of card latency. |
| C2 | Estimator dead → tilt abort blind | 🟢 | Second abort on **raw body rate** (150 °/s sustained 300 ms), independent of the quaternion. 0/150 false positives, fires on real tumbles. |
| C2b | **Both IMUs dead → tilt AND rate abort both blind** | 🟢 | Found 2026-08-08: both aborts read the IMU, so total inertial loss silenced both — measured **179.8° / 800 °/s with zero aborts and the chute still packed**. Third **blind abort** (`BLIND_ABORT_MS = 250`) now deploys on the *absence* of a trustworthy sensor. Chute out **0.39 s** after the sensors die at t=0.5/1.5/2.5 s. Mission is lost (no control is possible), airframe is not. 0 false positives in 480 dispersed healthy flights. |
| C2c | **Failover to a sensor that is not there** (found 2026-08-16, launch morning) | 🟢 | **New that week, and it would have flown.** `resetFlightState()` — added 2026-08-16 to fix the multi-flight bug — set `imuOk[BOTH] = true` unconditionally, discarding setup()'s probe, the *only* determination of whether the sensor board is physically present. It runs at the top of every `countdown()`, so the firmware entered **every flight** believing an absent ICM was available. In the SIL (ICM genuinely absent) the first wedged-MPU read failed over to it, the read failed, `F_IMU_BOTH_DEAD` latched → blind abort → **chute under thrust**. One recoverable sensor fault became total inertial loss because the fallback was imaginary; on the pad the trigger is an unseated sensor ribbon. Now **re-probes** (`sensorCablePresent()` + a live read of each source), which is also what the function's own contract asks for — a fresh power-on runs the probe rather than assuming its result. Locked: `imu_failovers=0 / imu_primary_ok=0 / imu_active=1` under `--imustuck`. The healthy-flight locks could never catch it — a healthy flight never *attempts* a failover, so `imuOk[]` is never consulted. |
| C2d | **Blind abort keyed on a latched bit** | 🟢 | `faultFlags` is a pure latch that nothing clears in flight, so keying the blind abort on `F_IMU_BOTH_DEAD` made its own `else` ("a failover recovered us") dead code. One **transient** loss of both sources armed the abort permanently and fired the chute 250 ms later, under thrust, with both sensors reporting normally again. The dwell timer was supposed to reject momentary glitches; the latch made every glitch irreversible instead. Now keyed on a live `imuBlindNow`, re-evaluated every iteration — the fault bit stays latched for the flight record. `controlInhibited` is cleared with it, so a recovered sensor does not fly the rest of the flight gimbal-neutral. Sustained failures still abort (existing `--imustuck` locks unmoved). |
| C2e | **Tumble AFTER burnout raised no abort at all** | 🟢 | Fixed 2026-08-16 on Braxton's ruling. `emergency()` returned at its first line unless `poweredFlight`, so a vehicle that departed during coast simply waited for the ordinary apogee/timer path. Now gated on `inFlight`. **The three aborts deliberately do NOT all extend, and the asymmetry is the whole design:** **TILT** stays powered-only — a healthy rocket pitches over through apogee and this project's own MC puts **coast tilt p90 at ~77°** against a 45° threshold, so extending it would abort the *majority of healthy flights* and deploy at ~30 m/s for nothing (Braxton confirmed the boost behaviour is what he wants kept: past 45° on ascent still fires). **RATE** extends — 150 °/s sustained 300 ms on the raw gyro is the actual tumble discriminator, since a ballistic arc-over is slow. **BLIND** stays powered-only — "no inertial source" is not "tumbling", and in coast the baro apogee, the TVC-stop timer and the burnout backup all deploy with no IMU at all. Gated on `inFlight` rather than `liftoffConfirmed` because **bench TVC mode** depends on `emergency()` returning early and hand motion trivially exceeds 150 °/s for 300 ms — and `liftoffConfirmed` latches on a 1.5 g transient that ordinary handling produces (CTL003), so it would *not* have been safe. Locked both ways: 0 aborts across 80 dispersed healthy flights, plus a positive control (unstable airframe departs in coast → `F_RATE_ABORT`). |
| C3 | Servo sign wrong | 🔴 | **Bench pitch-the-nose test is the only authority.** New board = new harness = must re-run. |
| C4 | **Servo axes swapped on the new board** | 🔴 | ⚠ Board nets say pin 3 = SERVOX, pin 4 = SERVOY; firmware uses X=4, Y=3. This crossing is what every successful flight used (2.2 netlist ≡ 2.1). **Do not "fix" it — verify it.** Pitch the nose in X only and confirm only the X gimbal responds. |
| C5 | **Thrust misalignment eats authority (ASC007, ASC037)** — *now the dominant failure mode* | 🔴 | **Mechanical, and software cannot substitute.** Measured from flight logs (mean commanded TVC over the burn = the misalignment being trimmed): **ASC036 = 1.12°** flew clean; **ASC037 ≥ 5.2°** railed for **85%** of the burn. Sim sensitivity (600 flights/row): σ=0.5° → 100% PASS, σ=1.0° → 94.8%, σ=1.5° → 83.8%, σ=2.0° → 75.0%. `I_GAIN` 0.20→0.80 buys **+0.8 points** — the steady-state gimbal command equals the misalignment whatever the gain, and biasing the servo centre to absorb 2° would need 7° of throw the linkage does not have (bench: 5.00° works, 5.25° binds). **Boresight the nozzle; nothing else moves this number.** |
| C6 | Gyro bias cal corrupted by movement on the pad | 🟢 | Pre-arm drift gate (>10 °/s aborts). |
| C8 | **Loop-rate-dependent thresholds** (found by ground test, 2026-08-15) | 🟢 | The SPI sensor board took a control iteration from **3450 µs → 85 µs** (measured, `CTL001`). The attitude-staleness detector compared a *per-iteration* tilt movement against a fixed 0.05°, which silently encodes a loop period — and it inverted: at 3.5 ms it could only fire below 14 °/s (excluded by its own 25 °/s gate, so never); at 85 µs it fires below **588 °/s**, i.e. on any real rotation. On the bench it declared a perfectly-tracking estimate stale, cleared `attitudeTrusted` and dropped the vehicle to rate damping. Boost body rate is p90 ~53 °/s, so **it would have fired on every flight**. Rewritten as a rate-vs-rate ratio integrated over a fixed 0.15 s window, so the loop period cancels instead of being baked into a constant. |
| C8b | The detector had never been tested — in *either* form | 🟢 | No coverage existed before 2026-08-15. Now locked in both directions: silent on healthy flights at both loop speeds and under heavy gust, and a **positive control** requiring it to still fire on a genuinely frozen estimator. That control caught a hole in the first rewrite, which stopped accumulating when `dt` left its sanity range — i.e. went blind at exactly the moment its own failure arrives, the same defect C2b recorded about the aborts. |
| C8c | SIL could not express a fast loop | 🟢 | Every timing knob only made the loop *slower*, and `sim_advance()` takes whole milliseconds, so sub-ms loops were structurally unreachable — the same class of blindness as ASC038, opposite direction. `--fastloop N` advances 1 ms every N calls, reproducing what the firmware sees (N−1 iterations at `dt==0`, then one at `dt==1 ms`). |
| C8d | `loop_us_median` was a constant | 🟢 | 100 µs bins over 0–12.8 ms, sized for the 3450 µs loop. At 85 µs everything lands in bin 0 and the "median" was that bin's midpoint — `CTL001` printed `loop_us_median=50` beside `loop_us_min=84`, a median *below* the minimum. Bins are 10 µs now; gate asserts median ≥ min. |
| C9 | **`IMU_STUCK_N=25` margin eroded by the fast loop** | 🟡 | At an ~11.7 kHz loop reading a 1 kHz sensor, ~11–12 *legitimately identical* consecutive reads are normal, against a threshold of 25 — only ~2× margin, where the old 3.5 ms loop had ~70×. Not currently false-firing (gate is clean, and the detector also requires powered flight), but it is one ODR change away. **Flagged, not yet changed.** |
| C7 | Loop slower than designed → more transport delay | 🟡 | Now measured (`loop_us_*`). Retuned gains verified robust to **0.30 s** lag, 10× the assumed value. |

## D. Sensors

| # | Failure | Status | Detail |
|---|---|---|---|
| D1 | Sensor ribbon unplugged | 🟢 | `SENS_DET` (pin 32) readable. *Only if you fly the SPI sensor board — see the recommendation below.* |
| D2 | I²C bus hang → `readIMU()` blocks | 🟢 | **The old entry here was wrong.** Checked against `WireIMXRT.cpp` (Teensyduino 1.59): the driver already bounds every transfer — `CLOCK_STRETCH_TIMEOUT` 15 ms in hardware (`MCFGR3 PINLOW`) plus a 50 ms generic bail-out returning error 4. A stuck bus costs ~50 ms per read, **not** forever, and there is no `setWireTimeout()` on this core to call. The real defect was ours: `imuReadSource()` **discarded `getEvent()`'s return value**, so a timed-out read was published as a live sample and every protection keyed off that bool — failover, stuck detection, both-dead — was silently disabled. Fixed 2026-08-09. SIL `--i2chang` now verifies the full chain: failed read → failover → both-dead → blind abort → chute out at 5.2/15.9/31.6/56.3 m. |
| D5 | **Firmware lockup (not a bus fault)** | 🟡 | Nothing returns, so nothing feeds WDOG1 — the watchdog is the only defence, and it did not exist in simulation until 2026-08-09 (`wdtFeed`/`wdtStart` were no-ops off-target). SIL `--firmwarehang` shows a **transient** lockup now recovers (watchdog reset → resume → chute at 54.2 m). This also exposed that the watchdog was armed *last* in `setup()`, so a reboot hung again inside the IMU probe with nothing watching; it is now armed **early when `persist.phase != 0`** (in-flight reboot only, pad behaviour unchanged). A **permanent** lockup remains unrecoverable — no parachute can be deployed by code that cannot run — and is locked as a documented limit. |
| D6 | Lockup within ~1.5 s of apogee | 🟡 | `WATCHDOG_SECONDS = 4`, but the fall from ~58 m takes only ~3.4 s, so a lockup near apogee deploys at **0.0 m** — too late. Shortening the timeout would fix it *only* if every blocking path (the SD log dumps in particular) feeds the watchdog; that is unverified, and a spurious reset mid-dump would be a regression. **Open decision, not yet changed.** |
| D3 | Baro fails → no apogee detect | 🟢 | Deploy timer + burnout backup timer both fire without the baro. |
| D4 | IMU orientation differs on the new sensor board | 🔴 | ICM-42688-P axes ≠ MPU6050 axes. **Any driver change invalidates the sign check.** |

## D-GPS. Passive GPS + horizontal navigator (added 2026-08-15)

> The entire subsystem is **instrumentation**. It exists so this ascent flight validates the GPS/IMU/baro
> fusion the *landing* firmware depends on, and nothing in it may reach a servo, a pyro, an abort or the
> apogee decision. That is why every row below is about **losing data**, never about losing the vehicle —
> and it is a regression lock, not a promise: `regression.py` 6f runs the same seed with the receiver
> healthy and dead and requires the control outcome to be **byte-identical**.

| # | Failure | Status | Detail |
|---|---|---|---|
| G1 | GPS never gets a fix on the pad | 🟢 | No datum → horizontal columns stay zero and `gps_datum_set=0` says so. Deliberately **not** an arm gate: passive instrumentation must never be able to scrub a launch. Fix state is printed every second of the countdown so it is a *human* go/no-go, not a firmware one. |
| G2 | Fix lost through boost (~3 g + vibration) | 🟡 | **The most likely real outcome, and the thing the flight is asking about.** Fusion stops, the filter coasts on the IMU, `fix_lost`/`reacquires` record it. SIL `--gpsdropat` verifies the flight is untouched. |
| G3 | Bad first-fix datum poisons every relative reading | 🟢 | Datum gated on ≥6 sats **and** HDOP ≤2.5, and never captured after liftoff. This is the mistake that put 28 m of error into the bench GPS-vertical run. |
| G4 | Degraded fix fused as if it were good | 🟢 | Measurement noise is scaled by HDOP; above `GPS_HDOP_MAX_USE`=8 the fix is logged but **not** fused. |
| G5 | Babbling / shorted RX line stalls the control loop | 🟢 | `pumpGPS()` is bounded at `GPS_MAX_BYTES_PER_CALL`=96 bytes per call — bounded *by construction*, not by trusting the receiver. This is the ASC038 failure class (unbounded work in the loop) and it does not get to recur through a new door. |
| G6 | Pad-azimuth solve does not converge | 🟢 | **It does not, and it is no longer relied on.** Measured over 25 seeds on a near-vertical ascent (~6 m of horizontal travel): median heading error **17°** with an ideal accelerometer, **22°** at 30 mg of bias, and **27° with a 10 Hz receiver** — more data making it *worse* is the signature of a systematic limit (inertial drift), not a noisy one. The filter now uses an **entered** pad orientation (`GPS_PAD_HEADING_DEG`, `NAV_HDG_FROM_PAD=true`) and the solve runs alongside as a scored experiment with a known truth. |
| G6b | Pad orientation entered wrong | 🟢 | Forgiving, and measured rather than assumed. 11 seeds, healthy 1 Hz receiver, dispersed 30 mg bias: entry error 0/5/15/30/45/90/180° → pos_err ×1.00/1.10/1.05/1.06/1.18/1.81/3.28. **Being 45° out costs ~18%**, so a rough compass bearing is ample; declination does not need correcting. Gate locks both ends — 90° must degrade ≥1.4× (proving the heading is actually wired in) and 45° must cost <1.6×. |
| G6c | **World/NEU handedness** | 🟢 | (N,E,U) is **left**-handed (`N × E = −U`); the firmware's world frame is right-handed with +Z up. The conversion is a rotation **composed with a reflection**. The first implementation used a plain rotation and the SIL passed it, because the harness carried the same wrong convention — a self-consistency test proving only that two copies of one mistake agree. Fixed, and now locked by a physical assertion (at ψ=0, world +Y must map to **West**). |
| G7 | Inertial drift budget unknown | 🔴 | `ins_gps_resid_m` measures it, but **the SIL cannot predict it**: the harness has no lateral accelerometer bias or noise, so its 0.10 m residual is an algorithm check, not a drift estimate. Only the flight answers this. |
| G8 | Log row truncated by the widened format | 🟢 | `snprintf` truncates silently; buffer widened 160→288 B and the gate asserts every ASC row has all 28 columns. |

## G8. Ground-test build reaches the pad

| # | Failure | Status | Detail |
|---|---|---|---|
| G8 | **`GROUND_TEST` left enabled on a flight build** | 🟢 | The flag bypasses all four pre-arm gates so the sequence can run on a bench with no igniters and no pyro pack. Forgetting it is the obvious hazard, so it is **self-limiting rather than remembered**: `triggerPyro()` refuses **every** channel, so nothing can light. Three reminders: boot banner + triple chirp at every power-on, magenta-and-chirp (not the red alarm) on each bypassed gate, and **`regression.py` hard-exits 2 and refuses to run**. |
| G8b | **🔴→🟢 GROUND_TEST fired live pyro channels on the bench (2026-08-15)** | 🟢 | **This actually happened.** The first version inhibited only P3 and *argued* the rest followed: with no ignition `liftoffConfirmed` could never go true, so the existing liftoff interlock would hold P1/P4. Wrong. `liftoffConfirmed` latches on a **transient** above `THRUST_ONSET_G` = 1.5 g sampled at ~11.7 kHz, and ordinary handling produces that — the 20 Hz log peaked at only 1.20 g and 0.04 m, so the spike fell between log samples. On `CTL003` it latched (`faults=0x000`, i.e. `F_NO_LIFTOFF` never raised), the firmware took the normal recovery path, and **both melt wires were energised on a bench**; postfire continuity went `P1=1/P4=1` on the prior run to `P1=0/P4=0` on that one. Fixed by moving the inhibit to `triggerPyro()` — the single choke point every call passes through, which no present or future call site can fail to inherit. **The lesson is general: a safety property that "follows from an existing mechanism" is an argument, not a guarantee.** An interlock against *"the motor never lit"* is not an interlock against *"someone picked the rocket up."* Now gate-locked with a positive control — `--groundtest` flies the full trajectory and asserts no pyro gate ever goes high **while `gt_attempt` confirms the firmware requested the chute and legs**, so the pass cannot come from the path never being taken. |

## H. Pad interface (ARM / countdown) — bench-reported 2026-08-15

| # | Failure | Status | Detail |
|---|---|---|---|
| H1 | **Countdown abort press not seen** | 🟢 | **Was real, now fixed.** The button was read at ONE INSTANT per tick, between the blocking `beep(...,200)` and `delay(800)` — so for essentially the whole second it was invisible, and the abort only worked if the button happened to still be held at that instant. Not a debounce problem: the press was never sampled. `cdWait()` now polls in 5 ms slices and **latches** the press. |
| H2 | Last 2.2 s of the countdown could not be scrubbed | 🟢 | The 1.2 s gyro calibration at T-3 and the 1 s pre-ignition settle sampled nothing at all — the window where you are most likely to spot a problem and least able to act. Both now poll, and there is a final abort check immediately before ignition. |
| H3 | Fixing the abort perturbs flight timing | 🟢 | The old comment defended one-shot sampling because a naive polling loop "measurably changed the countdown's duration and walked an injected reset across the burnout boundary in regression 6b". `cdWait()`'s slices sum to **exactly** the duration they replaced; locked by asserting the full outcome line is identical with a non-aborting press injected. |
| H4 | Contact bounce scrubs a launch | 🟢 | 30 ms of continuous press required. A real tap is 50–150 ms; gate gives a 10 ms bounce flying normally and a 150 ms tap scrubbing. |
| H5 | The arm press cancels the countdown it just started | 🟢 | 1 s deadtime, during which the latch is actively cleared rather than merely ignored. Gate-locked. |
| H6 | **Stuck/shorted button hangs the abort forever** | 🟢 | The press-consume wait was `while(digitalRead(BUTTON)==HIGH) wdtFeed();` — unbounded, **and feeding the watchdog**, i.e. the one hang the watchdog cannot rescue. Now bounded at 5 s with a `delay(1)` so the bound is reachable. (`exitOnButton()` already had this bound; the abort path never got it.) |
| H7 | Fifth ARM press acknowledged for 5 ms | 🟢 | `armed` is set inside the edge handler, so the loop exited on the next 5 ms pass: presses 1–4 chirped for `PRESS_CHIRP_MS` = 90 ms, press 5 for one pass — 18× shorter. Press 5 now holds the same 90 ms as the other four. Gate pins `cd_start_ms`. |
| H9 | Second press needed to clear a scrub | 🟢 | `countdown()` fails for two unrelated reasons and both used to land in the same solid-red hold awaiting a button press. A **deliberate abort** now returns to ARM by itself — the press was already consumed, the falling three-tone already confirmed it, servos and pyros are already safe. A **pre-arm refusal** (flat pack, open chute igniter, sick sensor bus, vehicle leaning) still holds red until acknowledged: nobody asked for those, and you must not be able to walk away from a dead parachute igniter because the board quietly re-armed. Gate-locked in both directions. |
| H8 | Dead air on the pad path | 🟢 | Three blocking waits that did nothing for flight: a bare `delay(500)` between the last arm press and the first countdown tick (`countdown()` re-neutralises the servos anyway), a 500 ms white LED flash on every entry to ARM (→ 120 ms, just as visible), and a **3 s wait for a USB monitor that is never attached on a pad** (→ 250 ms; with a monitor open it costs zero either way). Countdown now starts at `cd_start_ms` **405 ms, from 1200** — and boot is ~2.75 s quicker on the pad. |

## E. Motor / timing

| # | Failure | Status | Detail |
|---|---|---|---|
| E1 | Late ignition → TVC stops while thrust remains | 🟢 | `TVC_EXTEND_S = 1.0 s`. Measured lag 462 ms (3 flights) and 920 ms (ASC038) — covered. |
| E2 | Motor underperforms / CATO | 🟡 | `MTR###.CSV` records the flown curve. |
| E3 | No ignition | 🟡 | Detected post-hoc (`ignition_lag_ms`, flat axial trace). |

## F. Software / data

| # | Failure | Status | Detail |
|---|---|---|---|
| F1 | SD card slow or full | 🟢 | No longer in the control loop; only the recovery dump is affected. |
| F2 | Log buffer overrun | 🟢 | Guarded and flagged (`*** BUFFER FULL ***`). |
| F3 | EEPROM resume fires spuriously on the ground | 🟢 | Requires positive evidence of flight (free-fall/thrust or >90 °/s). SIL-tested with a stale record. |

---

## What is now in the SIL that was not before

| knob | models | added |
|---|---|---|
| `--sdlatency` | SD write cost charged to the simulated clock | after ASC038 — reproduces the tumble at 143 ms |
| `--servotau` | actuator lag (was a fixed, unmeasured 0.03 s) | gain-robustness sweeps |
| `--ignlag` | late ignition | measured 462–920 ms |
| `--mass` | all-up mass | measured 927 g |
| `--vbat`, `--rint`, `--stallA` | pack model V = Voc − I·Rint, servo/stall current | Impulse 2.2 |
| `--brownv` | brownout threshold | Impulse 2.2 |
| `--pyroopen` | open igniter on a channel | Impulse 2.2 |
| `--imustuck`, `--imuslow` | a wedged / slow inertial sensor | redundant-IMU work; exposed C2b |
| `--loopjitter`, `--loopjitterflight` | variable loop period (was hard-coded 5 ms) | 2026-08-08 — until then the loop-health governor had **never executed** in simulation |
| `p4t`/`p4alt`/`p4vz` output | *when* and *in what state* the chute fired | 2026-08-08 — `p4descent` alone scores ASC038's 1.5 m canopy as a success |
| `--resetat`, `--statein/--stateout` | **a real mid-flight MCU reset**: snapshot the world, exit, relaunch so firmware globals are genuinely fresh | 2026-08-09 — before this, `setup()` ran once per flight and the entire brownout-recovery feature had never executed |
| `--seedboots` | pre-loads `persist.boots` so `RESUME_MAX_BOOTS` is reachable | 2026-08-09 — a rail sick enough to reboot 3× for real never leaves the ground |
| watchdog (WDOG1) | the firmware's own `WATCHDOG_SECONDS` deadline | 2026-08-09 — `wdtFeed`/`wdtStart` were no-ops off-target, so the watchdog did not exist in simulation |
| `--i2chang` | wedged **bus**: the read costs 50 ms and FAILS | 2026-08-09 |
| `--firmwarehang`, `--firmwarehangdur` | wedged **firmware**: nothing returns, nothing feeds the watchdog; duration separates transient from permanent | 2026-08-09 |

**The gate itself could not fail safe** (found 2026-08-16). `run()` called `subprocess.run` with no
`timeout=`, so a firmware loop that never advances the simulated clock wedged the whole suite *forever*
instead of failing. Two consecutive gate runs sat dead for ~28 minutes having printed nothing, and
**"the test never finished" reads exactly like "the test never ran"** — the same shape as the ASC038
lesson that a harness which cannot express a failure cannot report it. The simulator does guard its own
clock (a 60 s *simulated*-time stop) but that guard is useless against a loop that never calls
`sim_advance()`: simulated time simply stops while the process spins. Now bounded at 60 s of **wall
clock**, printing the exact argv so the hang is reproducible by hand. The slowest legitimate run in the
suite is well under a second, so a trip cannot be a false positive.

**Still NOT expressible in the SIL** — and therefore *never* covered by a SIL PASS:
servo axis/sign errors (C3, C4) · IMU orientation (D4) · connector intermittents (A6) ·
canopy packing and melt-wire timing (B4, B6) · assembly/orientation errors on a new board.

**Two further blind spots, both confirmed 2026-08-16 and both worth stating before any "gate is green":**
- **Every SIL run flies the MPU-6050 backup.** The SPI shim returns 0x00, so the ICM-42688-P reads as
  absent and the firmware fails over on the first read — but the vehicle flies the **ICM as primary**
  (`imu_active=0`, CTL001). A green gate is evidence about the *backup* sensor path. This is why the
  pitch-the-nose check is the only verification the flying sensor's axis map ever gets, and why
  `ICM_AXES_VERIFIED = 1` silencing the boot banner is a real hazard rather than a nuisance.
  Related: the SPI shim also costs **zero simulated time**, so any flight that selects the primary
  freezes the simulated clock — the mechanism behind the launch-morning hang.
- **The health-fallback deploy branch (B9) is unreachable in the SIL.** Reaching it needs
  `HEALTH_FALLBACK` without an abort, and the oscillation detector cannot fire on its own because the
  model has no servo backlash or deadband (see the simplifications table). Driving `--pgain` to 6.0
  changes nothing. B9's guard is therefore verified not to regress the nominal path, and not verified
  on the path it exists to protect.

**Known physics simplifications** (these bound what a PASS means, and none are currently modelled):

| simplification | consequence | judged |
|---|---|---|
| **No lateral translation** — world velocity is purely vertical, so angle of attack ≡ tilt | a real tilted rocket accelerates sideways and its velocity vector swings toward its axis, *reducing* aoa. The model therefore **over**-states the aero moment at large tilt, and wind is applied as a pure torque with no weathercocking or downrange drift | the largest remaining fidelity gap; needs 2 more translational DOF |
| **Mass, Iyy and moment arm are constant through the burn** | ~50 N·s of propellant leaves the aft end: L grows ~4%, Iyy falls ~5%, so keff drifts ~9% over the burn. Second-order against the 26 N → 14 N thrust decay, which *is* modelled | real but modest |
| Gust is a pure sinusoid, not broadband turbulence | under-states short-period disturbance content | modest |
| Baro is truth + 0.15 m noise — no stuck reading, bias, drift, or deploy-transient | apogee detection looks more reliable than it is; the deploy timer is the primary trigger, which limits the exposure | modest |
| No servo backlash or deadband | a classic limit-cycle source. The firmware's oscillation detector has therefore only ever been exercised by injected extremes, never by a mechanism that produces limit cycles on its own | worth building next |

---

## Hardware to optimise, in priority order

1. ~~**Measure the melt-wire → canopy-clear time.**~~ **DONE 2026-08-16: ≈600 ms, `n=1`** (Braxton, bench).
   Comfortably under the ~1 s that would have demanded a mechanical fix, so no thinner nichrome / looser
   packing / stronger spring is needed. It also settles the pulse-duration question in the *opposite*
   direction to intuition: `PYRO_MS_CUTTER` is 1000 ms, so the wire is through with 400 ms to spare and
   raising it would only add FET heating (B11). **Remaining work is cheap and worth doing: repeat it.**
   One sample gives a typical value and says nothing about the spread, and the number that matters for
   flight is the worst case. Not a launch blocker — the margin is on the right side either way.
2. **Battery internal resistance.** A2 is the prime suspect for two lost flights. Measure sag under a
   1–2 A pulse. If the pack sags more than ~0.5 V, replace it — the pre-arm gate reads open-circuit
   voltage and *cannot* see a high-Rint pack that looks fine at rest.
3. **Gimbal travel margin.** Currently 5.00° works and 5.25° binds — under 5% margin, and `MAX_TILT`
   sits exactly on the tested limit. Opening the mechanical travel buys authority against C5 directly.
4. **Nozzle boresight.** Misalignment comes straight off the authority budget 1:1. ASC036 achieved 0.66°;
   ASC007's 4.28° nearly aborts on its own.
5. **Strain-relieve every connector** (A6) — battery, servo, pyro, sensor ribbon. Vibration-induced
   intermittents are undetectable in flight and present as unexplained brownouts.
6. **Verify MEMS part orientation on delivery** before first power-up. `DESIGN_LOG.md` 2026-08-01 flags
   both LGA sensors as the only practically non-reworkable orientation failures on the board.

---

## Recommendation for the FIRST 2.2 flight

**Fly the backup I²C sensors (MPU6050 + BMP280, `SDA`=18/`SCL`=19), not the SPI sensor board.**

The 2.2 features worth having on flight one — rail voltage sensing and pyro continuity — are ADC reads and
need no new sensor driver. The SPI ICM-42688-P/DPS310 path needs a new driver, a new axis convention and a
fresh sign verification (D4, C3), and would put unproven code in the control loop on the maiden flight of an
unproven board. Fly the known-good sensor path, bank the electrical improvements, and qualify the sensor
board on the flight after.
