# TVC Rocket Research — Claude Context

> **Slimmed 2026-07-01.** The full simulation-phase lab notebook (Π = keff·τ² saturation research,
> Exp1–Exp5 correction history, STS paper development — ~3500 lines) is archived at
> [archive/CLAUDE_pi_research_2026-06.md](archive/CLAUDE_pi_research_2026-06.md). Read that only for a
> specific historical result. This file covers the **current phase**.

## ▶ JUMP-IN STATE (short-term — a new chat should read this first)
**Updated: 2026-07-03.** *(Keep this block current — it's the "resume here" pointer.)*
- **Goal right now:** flight-readiness. The 3-D gap is CLOSED (below); remaining needle-movers are HARDWARE
  (crushable legs, motor characterization, build alignment) and an estimator-aiding upgrade for the sensor-drift tail.
- **LATEST (2026-07-03): FULL 3-D BUILD DONE — quaternion GNC, 2-axis TVC, roll control, CG-misalignment roll
  coupling.** Full detail: DESIGN_LOG 2026-07-03. Harness is 6DOF (quaternion, Euler eqs WITH gyroscopic
  roll-lateral coupling at IZZ≈IYY/65, 2-axis gimbal at the nozzle with lateral CG offset → **M_roll =
  T·(cgy·δa − cgx·δb)**: every TVC correction leaks roll; nozzle cant; fin-misalignment roll; crosswind
  `--windy`; 3-axis IMU w/ FSR; GPS x+y). Firmware: quaternion strapdown (measured-dt), world tiltX/tiltY +
  swing-twist roll, per-tilt-channel ADRC with actuation DEROTATED through measured roll (control stays correct
  WHILE rolling — verified tracking at 200 dps), 3-channel EKF, 2-axis wind estimate + guidance (crosswind
  nulled: y-miss ≤2.1 m @ ±4 m/s), 3-D retro cone, margin coast = same trim-ride torque-commander with a VECTOR
  authority (+k·(sfx,sfy) in tilt-channel coords), **roll-rate ESO on the DIFFERENTIAL fin fold** (authority ∝
  sin(aoa): strong at high aoa exactly as intended; roll ANGLE free, rate bounded; 720 dps abort).
  - **Roll-divergence results:** cgoff 2–4 mm → roll 7–42 dps, tilt <3°, PASS. Realistic nozzle cant → ~200 dps
    builds during BOOST (fins can't bite at aoa≈0 — physics), fins catch it in coast; lands fine. Extreme cant →
    correct ABORT. Combined diagonal-wind+cgoff+cant: 0.8–1.4 m/s touchdowns.
  - **Traps fixed (sign/pairing):** gimbal sign (F_lat=−T·sinδ), axis pairing (torque about +Y drives tilt-X;
    keff RLS regresses gyroY), TD gate now requires the FULL burn elapsed (early TD + CG offset = unopposed
    torque → 55° tilt in the hop). Geometric roll-twist through big swings is REAL (46° with ~0 roll rate) and
    the firmware tracks it within 1.5° — that's what makes the derotation load-bearing.
  - **Honest 3-D baseline:** nominal wind grid 81/100 (vz p50 3.2 / p90 5.4 / max 8.6, incl. 25 ms motor
    dead-time). Tail = one bad 3-axis gyro-drift draw (gravity leakage → velocity bias → knife-edge hops 5-6 m/s
    + ~4 m miss) — estimator-aiding work, in TODO. 3-D MC (realistic + roll-coupling dispersions): soft<5 21%,
    legs 8 m/s 63%, **legs 10 m/s 82%**, aborts 1.3% (chute). vs 2-D: −5..12 pts = the price of 3-D honesty;
    the crushable-legs + characterized-motor conclusion STANDS. `validate.py` is now a DISTRIBUTIONAL guard
    (pass-floor 70% + hard caps). New knobs: `--windy --cgoffx --cgoffy --rollcant --finmis`.
- **LATEST (2026-07-03 pm): 3-D sweep RENDERER + a reverted control experiment.** `sweep_html.py` now renders a
  self-contained 3-D scene (orbit drag/wheel, top/side buttons): shaded rocket posed by the logged TRUE quaternion
  (Qw..Qz now in the CSV), 4 fold-out margin fins (common fold=deploy, differential=roll), 3 legs swinging out at
  ignition, gimballed exhaust, ground/target/EKF-marker, crossrange+roll HUD. Node-smoke-tested over all 10,098
  grid frames (0 NaN, 0 exceptions). **Control note:** the user pointed out the IMU is co-mounted with the TVC
  (each gimbal axis == its gyro axis), so a body-frame-native controller (no roll derotation) is the honest form.
  I attempted it, hit an instability I couldn't root-cause fast (documented in DESIGN_LOG 2026-07-03 pm: the key
  lesson = never fold a guidance-steered reference into the ESO MEASUREMENT), and REVERTED to the validated
  derotation controller -- which IS the co-frame mapping (rotates by the roll the co-located gyros measure). The
  body-native form is a clean follow-up worth finishing.
- **FLIGHT-READINESS VERDICT (2026-07-02): NOT ready to fly, for two reasons — one fundamental, one architectural.**
  Built the missing tools: `Firmware/sim/montecarlo.py` (joint build/motor/timing dispersion Monte Carlo +
  leg-rating survivability) and `validate.py` (fast nominal regression guard, 36/36). Added the failure-mode
  realism the sweep lacked: sensor FSR saturation, ignition dead-time+scatter, loop jitter, mass/inertia/CG/fin
  dispersions (all runtime knobs). **The clean-wind sweep (100/100) only ever tested WIND** — under joint
  realistic dispersions the soft-landing pass rate is ~33%. Full detail: DESIGN_LOG 2026-07-02 (pm).
  - **(1) FUNDAMENTAL (hardware):** the hoverslam depends on the LANDING motor's thrust/mass, which is
    UNOBSERVABLE before the solid commits (can't measure landing thrust pre-ignition; boost conflates mass with
    ascent thrust; a solid can't throttle/cut). So near-zero soft touchdown is unreachable with a solid — this
    quantifies the long-standing "throttleable-motor" ceiling. MC survivability (all dispersions, N=150): soft
    <5 m/s 33%; but with legs rated **8 m/s -> 70%, 10 m/s -> 87%**; and with a **static-fire-characterized
    landing motor (±1.5%) + 8-10 m/s legs -> 91-98%.** So the vehicle is a viable CONTROLLED-DESCENT-TO-
    CRUSHABLE-LEGS lander, not a near-zero soft-lander. Levers: crushable legs (~10 m/s) > characterized motor >
    (for true soft) throttleable motor.
  - **(2) ARCHITECTURAL (firmware): ~~still 2-D~~ CLOSED 2026-07-03** — yaw plane, roll handling, and the
    CG-misalignment roll coupling are built and SIL-validated (see the LATEST bullet above).
  - **Real firmware gaps found+FIXED via the new realism** (invisible at nominal): ignition motor dead-time now
    lead-compensated (`IGN_LEAD_S` free-fall projection in `ignitionReached`); EKF+attitude now integrate on
    MEASURED dt (a loop stall no longer loses time → jitter benign; the Π latency lesson in firmware); sensor FSR
    modeled. **In-flight mass ID was built then DEFAULTED OFF** (`g_massIdOn=false`) — boost can't separate mass
    from thrust so it poisons the hoverslam (MC: 33% off vs 10% on); weigh the rocket instead.
  - **Next actions (open):** biggest needle-movers are HARDWARE (crushable legs, motor characterization) and the
    3-D/yaw firmware build. New knobs: `--massmult --iyymult --cgoff --finauth --igndelay --ignjit --accfsr
    --gyrofsr --jitter --massid/--nomassid`. Run `python montecarlo.py` (SCENARIO=realistic|characterized|broad)
    for the dispersion picture; `python validate.py` as the nominal regression guard.
- **(historical goal:) make the propulsive-landing firmware land as cleanly in the SIL as the old web-app
  sim did — soft on-target touchdown.**
- **LATEST (pm-4): 100/100 PASS under the most honest physics yet** (0/8/16/24 m × −4…+4 m/s × 5 seeds; vz worst
  4.9, typ ~3.5; |miss| ≤2.3; tilt ≤3.1°; W=±5/±6 probes also pass — old edge was W=−5 failing). Five coupled
  changes, full detail in DESIGN_LOG pm-4: (1) **EKF lead-comp at fusion** — baro (+estVz·0.040) and GPS
  (+estVx·0.20) measurements propagated to "now"; est errors ~0.1 m through the burn; the ghost hugs the rocket
  again; ignitionReached's old point-of-use lead REMOVED (would double-comp). (2) **Hoverslam stop-margin** —
  unbiased estimates exposed that "arrest exactly at 0" actually arrests ~0.4 m high → un-cuttable solid relaunches
  (hop, ~5-6 m/s) vs arriving ~1-3 m/s hot (sticks on legs): asymmetric cost → ignite when a burn from
  STOP_MARGIN_M=0.5 m HIGHER would just reach the deck. (3) **Leg contact height** in the harness (LEG_H=0.08 —
  point-contact-at-exactly-0 was a knife-edge artifact) + TD no longer declared while the solid still thrusts
  (>90% burn-clock gate; premature TD cut TVC mid-hop). (4) **HONEST fold-out margin-fin plant** (user request:
  fins track the migrating CG): one-servo symmetric fold-out canards only SHIFT THE CP — moment ∝ −q·sin(aoa)·
  [body − dep·fin] with arms on an EXPLICIT migrating xcg (propellant→fwd, legs→aft; TVC arm rides it too);
  neutral dep* ≈0.37 launch → ≈0.48 coast, NOT a fixed 45°. (5) **Trim-ride coast controller**: sit at the
  CG-scheduled dep* (torque-free at ANY attitude — holding ground-retro in headwind costs nothing) and ADRC-
  modulate around it via SIGNED authority from the lateral specific force (m·sfx ≈ q·S·CD·sin aoa). A stability-
  bias fallback CAPTURES the vehicle onto the wind-tilted aero equilibrium (−44° @ 8 m/s headwind) — use dep*.
  Plus slew-matched actuator observers (SERVO_SLEW_DPS; old ones claimed 10× the real margin-servo rate) and a
  coast-matched ADRC bandwidth WC_COAST=2/W0_COAST=10 (WC=6 demanded ~20× the fin authority → rail chatter that
  the servo lowpassed to neutral = the old headwind divergence).
- **What just happened (this session):** fixed the terminal-descent bug AND a SIL harness bug that was
  masking it. (1) **Harness (`sim_main.cpp`):** `stepPhysics` kept integrating after landing → gravity
  re-crossed `pz<=0` every step → OVERWROTE `impactVz` with ~−0.01 → every prior "PASS" was fake. Fixed
  with `if(landed) return;`. Honest baseline was **0/20**. (2) **Firmware ignition:** the causal suicide
  burn used a FLAT avg thrust and over-braked (nulled v ~2 m up); the solid motor then hopped and
  free-fell — the "oscillation before touchdown." Replaced with a **causal forward predictor**
  (`predictStopAlt`) integrating "ignite now" under gravity + the known thrust curve (`landThrustAt`) +
  drag; ignite when predicted stop-alt crosses 0. Tightened TD gate 2.0→0.5 m. **Now 16/20, impact
  2.2–3.6 m/s (was ~6.7 crash), robust across gust seeds.** sweep.html regenerated.
- **Then fixed the near-ground WOBBLE (user report):** (a) `keff_est` entered the burn stale (~2.3 vs true ~7)
  → 3× over-actuation thrash; (b) legs deployed 1 s into burn (~4 m) → 25% keff jump untrackable before TD;
  (c) divert leaned up to 24° chasing target near the deck. Fix: reset `keff_est=KEFF_NOM` at ignition;
  **deploy legs AT ignition** (constant plant, ~1.5 s to settle; legs change only keff not drag in-sim); divert
  cap 24°→15° and ramp out by 4 m. Peak burn rate ~21→2–14.5°/s, wobble gone, **miss improved <0.4 m**, vz
  unchanged. Boost aim was already at the sensor floor — no change needed.
- **Terminal WOBBLE fully resolved (2026-07-01, this session) — divert was the culprit.** The residual
  ~10° swing right before burnout (worst in headwind/far targets) was NOT keff or ignition: instrumenting the
  attitude reference (`thRetro`+`divert`) showed the divert leans **+10° into the target**, which HOLDS cross-range
  velocity that retrograde then must null in the last ~5 m → a +4°→−11° reference swing. Proof: retrograde-only
  (the `--nodivert` nominal) is perfectly monotone (th −8°→0°, vx→0, upright). Divert-only winds up (overshoots
  +5 m, lands with 3.9 m/s lateral). **Fix: `DIVERT_IN_BURN=0` in `ctrlBurn` → pure-retrograde terminal descent.**
  Also **froze `keff_est` during the burn** (RLS was driving b0 to ~3.6 vs true ~7.4 because the `AERO_DAMP` term
  dwarfs real low-speed damping → 2× over-command). Result: **20/20 PASS, tilt ≤4.8° everywhere (was 10.3°),
  ~0 dps at TD.** Cost: worst-case miss ~1.7→2.7 m (all ≪5 m gate) — accepted per land-safe>hit-mark. To hit a
  tight pad later, raise `DIVERT_IN_BURN` toward 1 (wobble returns). vz floor unchanged (~3 m/s).
- **Headwind undershoot SOLVED → WIND-ADAPTIVE ASCENT + pure-retrograde landing (2026-07-01, this session).**
  This is the current landing architecture; it replaced the earlier burn-divert experiments (position-divert in the
  landing burn is what caused the pre-landing wobble — user: "no, that's bad"). **Diagnosis chain:** the headwind
  miss is a GUST that displaces the vehicle downwind during the ~4 s unpowered COAST (traced px@ignition: a bad-seed
  −2 m landing was already ~1.8 m short at burn start). Coast has NO horizontal authority (thrust off; body-lift
  crab was tried + reverted — realistic Cn≈2 turns the retrograde 35° tilt into a plant-breaking sideforce). The
  burn is too short to fix it without a wobble. And the "always undershoots" look was amplified by `sweep_html.py`
  running ONE fixed gust seed. **Fix:** correct downrange UP FRONT in the boost, the one phase with thrust authority.
  `updateWind()` now estimates wind in BOTH boost (thrust inferred from vertical accel incl. vertical-drag term →
  horizontal residual = drag → wind; converges ~0.3 m/s by ~1.2 s into the 3.45 s boost) and descent. `predictLandPx()`
  forward-integrates remaining boost + ballistic coast (under windEst) to the ignition altitude; `ctrlBoost` steers
  the kick (proportional predictor-corrector, `g_kickCmd`) to null the predicted miss. Landing = PURE RETROGRADE
  (`ctrlBurn`, no divert). **Result (grid, 5 gust seeds, ONE calm-aimed kick per target, no per-wind tuning):
  every cell median |miss| ≤0.29 m, worst ≤1.19 m, tilt ≤4.8°** — accurate AND smooth, beating the burn-divert on
  both (that had 8–11° peaks). Robust to UNFORECAST wind by construction (the vehicle self-aims). Residual ~1.2 m
  worst is gust scatter (now centred on target, not systematically short). Key knobs: guidance gain 0.15, kick clamp
  [−8,15]°, `GUIDE_IGN_ALT=22`, `ASC_*` thrust curve. `g_divertOn` (via `--nodivert`) now gates the GUIDANCE off for
  the calm-kick bisection. sweep_html: one calm kick per target + multi-seed (median shown, mean/worst reported).
- **REALISM PASS + honest touchdown (2026-07-01, this session).** Added to the SIL harness (`sim_main.cpp`,
  firmware untouched except a mass-aware ignition predictor + ADRC bandwidth now runtime): (1) **servo dynamics** —
  slew-rate + 30 ms lag on TVC & margin servos (was instant). **KEY FINDING: servo slew is a hard control limit** —
  at the user's **60 deg/s** the stock ADRC (WC6/W030) TUMBLES (slew saturation, esp. the weak margin-fin coast
  reorient); needs >=~100 deg/s to fly WC6/W030. Detuning to **WC2/W08** (slew-matched) flies the whole envelope at
  60 deg/s with no tumble, but the sluggish loop tracks the wind-guidance poorly -> misses 2-6 m in wind (calm <1 m),
  tilt to ~16 deg. ADRC bandwidth MUST match servo speed; faster servo (>=~150 deg/s, restore ~6/30) is the fix for
  soft AND on-target. `SLEW`/`WC`/`W0` env knobs sweep this in the SIL. (2) **propellant mass/inertia depletion** over both burns (`LAND_PROP`
  /`ASC_PROP`=0.03 kg; predictStopAlt now models the landing mass drop via `LAND_PROP_KG`); (3) **wind shear** —
  power-law `windAt(z)` (α=0.14, ref 10 m); (4) **sensor timing** — baro 30 ms lag @50 Hz, gyro-bias random walk,
  200 ms GPS latency. Also **fixed a measurement bug**: the sim stopped 1.5 s after TD was *declared* (0.15 m),
  cutting off before the solid-motor HOP fell back — under-reporting vz. Now runs to true ground contact.
  **Honest finding:** with all realism the vehicle still lands ACCURATE (miss <1.5 m typ, worst 2.2) and UPRIGHT
  (tilt <4°), but touchdown **vz rose from ~3.6 (idealized) to ~7 m/s** — breakdown: mass depletion (T/W rises as
  propellant burns → over-brake → bigger hop) +~1.5, baro lag (biases the hoverslam's estZ/estVz → mistimed
  ignition) +~2. So the sweep now reads 0/5 on the 5 m/s SOFT gate everywhere — *only on speed*, not accuracy/attitude.
  **Baro-lag hoverslam fix (this session, requested):** the fused estZ/estVz lag ~45 ms (baro+filter) -> at 20+ m/s estZ read ~1 m high -> mistimed ignition -> over-brake/hop (the +~2 m/s baro contribution). `ignitionReached` now LEAD-COMPENSATES: zc=estZ+estVz*BARO_LAG_S, vc=estVz+lastAccZ*BARO_LAG_S (BARO_LAG_S=0.045). Recovers vz ~7 -> ~3 m/s (down to <1) WITH a fast servo -> soft AND on-target restored under full realism. At 60 deg/s the servo still caps accuracy (wind misses 2-6 m). Net: soft landing needs baro-lag comp (done) + a >=~150 deg/s servo. Reconfirms: soft touchdown needs a throttleable/extinguishable motor (cuts at v=0) OR a baro-lag/mass-robust
  hoverslam OR shock-absorbing legs — NOT more control tuning. The idealized SIL was optimistic on touchdown speed.
- **Animation upgraded:** `sweep_html.py` + `make_flight_html.py` draw the forward margin fins as FOLD-OUT
  canards (deploy=0 folded flat along the body & hidden = full stable; deploy=1 swung upward to 90° out =
  full unstable; drawn behind the body so folded fins vanish), the legs swinging out at deploy (latched past
  the pyro pulse), and a TVC-gimballed exhaust; playback is REAL-TIME (the old loop played ~3.6x fast).
  NOTE (superseded pm-4): the ghost drift WAS honest sensor lag (200 ms GPS, ~40 ms baro) fused uncompensated;
  the EKF now lead-compensates both measurements at fusion, so the ghost hugs the rocket again (~0.1 m).
- **Honest ceiling (don't naively chase 0):** the web app lands ~0 m/s only via an OFFLINE ignition
  search over a ~0.1 m knife-edge + a THROTTLEABLE engine that CUTS at v=0. A solid-motor causal
  controller can do neither → ~3 m/s is near its floor. The lever to get web-app-soft on hardware is a
  throttleable/extinguishable landing motor, not more firmware tuning.
- **Coast-apogee tumble FIXED (was thought physical, was a controller bug):** `marginAuthority()` gauged fin
  authority from GROUND speed, which collapses at apogee in a headwind (~2 m/s) → fin DISABLED (`b0<0.5`) exactly
  when needed, though air-relative speed (~6 m/s) meant real authority existed → weathercock tumble. Fixed by
  gauging authority from the SENSED aero specific force (wind-agnostic): authority ∝ |sf_aero|, `K_MARGIN` const,
  `sfAeroLP` low-pass in `readIMU`. **Now 24/24 over targets 0/8/16/24 × wind −5…+4** (was 16/20; W=−5 failed
  everywhere). Robust across 5 gust seeds. Caveat: fin is ~0.9× the weathercock at 90° aoa — thin margin, works by
  engaging early; bigger fins would add margin for >5 m/s winds.
- **Next actions (open):** optionally squeeze terminal 3→~2 m/s via ignition-predictor fidelity (burn-time cosT,
  EKF end-of-descent alt lag) — diminishing returns vs the solid-motor knife-edge. Otherwise SIL is landing
  soft+on-target+upright across the full envelope; next real needle-mover is HARDWARE/flight.
- **Honest terminal ceiling stands:** ~3 m/s solid-motor floor (can't cut like the web app's throttleable engine).
- **How to build/test the SIL:** `cd Firmware/sim`, build with ziglang (`build_zig.bat`, or the
  scratchpad `zig c++ ... sim_main.cpp ../Sysiphus_Landing.cpp -o sim.exe`). Run e.g.
  `sim.exe --wind 4 --target 16 --gusts`; sweep+visualize via `python sweep_html.py` → `sweep.html`.
  Details in the CURRENT PHASE section below.

## Working style (current phase)
- Keep outputs **concise**. We're in the practical hardware/firmware phase, not chasing STS — you do
  NOT need the heavy experimental rigor or the 8-section critical-thinking format from the paper era.
  Still: report outcomes faithfully, never write expected numbers as measured, flag real risks.
- Act as a **skeptical reviewer** on substance (falsify before confirm, name confounds, quantify when
  it matters) but say it briefly.

## 🛠 AI Operating Environment — read/maintain these
Six living workspace docs govern this repo: **AI_RULES.md** (binding constraints) ·
**PROJECT_CONTEXT.md** (architecture & module map) · **DESIGN_LOG.md** (decisions — append at top) ·
**EXPERIMENTS.md** (experiment log) · **REGRESSION_CHECKLIST.md** (verify-after-change) ·
**TODO_AI.md** (tech debt). Before significant changes, state intent / files-likely-to-change / risks;
prefer small reversible edits; confirm before large refactors, API changes, mass renames, or deleting
substantial code. Run `cd sim && python validate.py` before reporting core-sim results.

---

## ⚡ CURRENT PHASE (2026-07-01) — hardware & firmware

The **simulation research phase is complete and archived** (paper: paper/archive/2026-06-25_sts_
candidate/). The novelty search found no new principle; the asset is rigorous application + a strong
self-correcting process. Do NOT re-open gated simulation directions or re-inflate novelty/finalist
claims without new evidence.

**Active work: propulsive-landing rocket — simulator → firmware → SIL testing → flight.**
- `Firmware/Sysiphus_Landing.cpp` — flight firmware (Teensy). Landing GNC: per-axis Kalman filter,
  ADRC attitude control (TVC in powered phases, forward **margin fins** in coast), online RLS keff as
  ADRC b0, suicide-burn (hoverslam) ignition, nominal-trajectory-tracking divert.
- `Firmware/sim/` — **software-in-the-loop (SIL) harness**: compiles the EXACT firmware against mocked
  Arduino/Teensy shims + a 2-D physics model, so `setup()`/`loop()` run unmodified. Build with ziglang
  (`build_zig.bat`). Knobs: `--wind --target --kick --gusts --seed --tw --land --nodivert
  --nomout/--nomin`. `sweep_html.py` → `sweep.html` (target×wind sliders, PASS/FAIL heatmap,
  animation); `make_flight_html.py` renders one flight.
- Landing-sim family: `tools/landing_sim3dof.py`, `tools/landing_interactive.py` (self-contained JS in
  `outputs/landing_interactive.html`), `tools/landing_animate.py`; plus `sim_vsr/` (variable-
  stability / margin-modulation / canard sim).

**Firmware WIP (paused 2026-07-01) — terminal descent quality.** Two coupled SIL bugs found:
(1) touchdown declared ~1.7 m too high via the `estZ<2.0 && estVz>-TD_VZ_M` gate because the solid
motor's peak thrust (28 N) over-brakes vs the model's 20 N avg → velocity nulls high while still
burning; (2) control is then cut → rocket free-falls the last ~2 m uncontrolled and attitude drifts
= the "oscillation right before touchdown" the user reported. Also found a **coast-apogee tumble for
W ≤ −4 far targets** (weak margin fin can't cancel wind weathercock at low apogee airspeed → diverges/
aborts — a physical authority limit). A retrograde command-shaping fix (speed-blend + 35° cone cap,
`retroTarget()`) is in for coast+burn but does NOT fix the apogee tumble. **Next:** fix suicide-burn
timing so the burn nulls velocity AT the ground (couple ignition-altitude model to the real thrust
curve/peak) and retains TVC authority to contact; tighten the TD gate. SIL binary builds to the
session scratchpad; `--seed` added but gusts weren't yet varying per seed.

---

## Project purpose (archived research — context only)
The sim study asked how physical rocket properties set (1) the controllable/uncontrollable boundary,
(2) required simulator fidelity, (3) the robustness–maneuverability tradeoff. Archived headline:
removing servo **slew-rate** saturation restores PID to SR≈0.99 across 142 designs; **Π = keff·τ²** is
the coordinate locating the saturation onset (not a claimed law); above Π_crit≈275 (bare-metal FIFO
~177) PID enters a bang-bang limit cycle that ADRC's ESO escapes. Full detail + retractions in the
archive. NOT a PID-tuning project.

---

## Key user context (memory/)
- Strong, reliable control/aero intuition — engage at full rigor on substance, welcome pushback
  ([[user-physics-strength]]).
- Landing-sim tuning: fixed **average rocket** + fast single sims, **no per-test optimization** scans
  ([[feedback-no-per-test-optimization]]).
- Keep `Paper_Condensed.md` synced with `Tentative_Paper_Draft.md` ([[feedback-keep-papers-in-sync]]).
- Hardware flight is the only STS needle-mover; a hardware surprise is the realistic finalist lever
  ([[project-direction]], [[hardware-validation]]).

## Firmware/hardware safety
`Firmware/` controls real hardware — never weaken a safety interlock, arming check, or failsafe;
validate in the SIL before hardware. Flight claims must trace to evidence (AI_RULES.md §6).
