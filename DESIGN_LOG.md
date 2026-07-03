# DESIGN_LOG.md — Engineering Decision Record

> Append-only. **Add new entries at the TOP** (reverse chronological). Never edit or delete a prior
> entry — if a decision is reversed, add a new entry that supersedes it and link back. Each entry:
> Decision / Reasoning / Alternatives / Tradeoffs / Risks / Date.

This log is seeded (2026-06-30) from decisions already evident in `CLAUDE.md` and `memory/`; those
files remain authoritative for the full detail. New decisions go here going forward.

---

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
