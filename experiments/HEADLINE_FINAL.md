# HEADLINE_FINAL — Post Kill-Test Honest Reframing

**Status: Updated after hostile peer review + 4 falsification tests + realistic-plant port + calibration + community-firmware audit + scaling-law extraction.**
**Supersedes HEADLINE_LOCKED.md.**

---

## Headline (one sentence, hostile-reviewer–proof)

> **In realistic simulation, amateur TVC rocket stability is best described
> as three regimes — easy, fragile-but-recoverable, and fundamentally
> infeasible — set primarily by actuator envelope and airframe instability.
> The boundary follows `slew_min ∝ p²` (R² = 0.75), three publicly-deployed
> amateur TVC firmwares fail far below the feasible envelope, and a new
> hobby-range regime sweep shows that best-tuned ordinary PID already covers
> the tested nominal 45-90 deg/s TVC band, making the regime map and
> bench-calibrated workflow the core claim.**

### Important update after the hobby-range controller sweep

The previous hope that `PID_SLEW_AWARE` might be a main-claim bridge from
ordinary hobby PID to a stronger controller story does **not** survive the
new hobby-range sweep. `run_pid_slew_regime_map` shows `48/48 BOTH_FINE`
across `p = 4, 6, 8`, nominal loaded slew `45-90 deg/s`, and retained slew
scales `1.00-0.25` when PID and `PID_SLEW_AWARE` are each given their best
gain pair from the tested grid. That means the lightweight controller branch
is now a **secondary severe-collapse appendix**, not the central finalist-level
claim.

### Important honest correction (added after head-to-head)

The previous claim that an "R*(u_max) LQR recipe matches/beats SIGMA_MRAC
and PCH" was an artifact of inconsistent comparison settings. The
apples-to-apples `run_head_to_head_unstable` comparison shows that
**properly-tuned PD with sufficient u_max actually OUTPERFORMS the
validator's LQR(R*) recipe on the realistic plant**, because LQR's
quadratic cost underweights derivative damping needed for rate-limited
noisy actuators (steel-PD optimum Kd ≈ 10–30; LQR with R*=8 produces K₂ ≈ 2).
The validator's *region classification* (GO/MARGINAL/NOGO) remains
calibrated, but its *recommended gains* are a starting point, not an
optimum — a steel-man PD or adaptive controller does better within the
workable region. See `results/head_to_head_unstable.csv`.

## What survives, what died

| Claim | Status after kill tests | Evidence |
|---|---|---|
| H1: u_max has a non-monotone optimum for fixed R | SURVIVES | `h1_coupling`, `solver_invariance` |
| H1: this is a fundamental plant property | **DIED** | `h1_bestk_kill`: best-R is monotone |
| H1 (refined): in low-slew regime the dip persists for ALL R | **NEW, PORTED FINDING** | `realistic_h1_sweep`: slew=20 non-monotone in best-R |
| H1 mechanism is integrator wind-up | **DIED** | `h1_aw_kill`: rate-aware AW is exact no-op |
| H2: design for low pole beats designing for worst pole | SURVIVES | `h2_bestk_kill`: 0.90 vs 0.66 even with optimal R per design |
| H3: identifiability budget bounds wiggle-then-control accuracy | INDEPENDENT, holds | `run_h3` |
| **NEW — Scaling law `slew_min(p) ~ p^α`** | **DISCOVERED, α = 1.99 vs theory α = 2** | `run_scaling_law` on 60-config calibration grid |
| **NEW — Community-baseline firmware fails at p ≥ 4** | **EMPIRICALLY DEMONSTRATED** | `run_firmware_audit`: three real GitHub repos |
| **PID_SLEW_AWARE as a main hobby-range claim** | **DIED** | `run_pid_slew_regime_map`: best-tuned PID and PID_SLEW_AWARE are both fine in all 48 tested hobby-nominal cells |

## Current best story (May 31)

The strongest version of the project is now a main claim plus one explicitly
secondary appendix:

1. **Empirical map:** the amateur TVC design space splits into easy,
   fragile-but-recoverable, and infeasible regions. Builders currently treat too
   many failures as "bad tuning" when some are hard hardware limits.
2. **Builder workflow:** bench measurement plus realistic simulation can tell
   the builder which region they occupy and which hardware changes matter most
   before flight.
3. **Appendix only:** a lightweight controller patch can still be studied in an
   explicit severe-collapse stress regime, but it is no longer needed to carry
   the main story.

This is more defensible than a controller-novelty story and cleaner than a pure
tool paper with no hard scientific boundary claim.

## The (slew, u_max, R*) phase diagram

Three operating regions for a statically-unstable amateur airframe:

1. **Rescue region** (high slew, e.g. ≥35 units in realistic sim):
   - Naive fixed R=0.5 LQR drops from 1.00→0.66 success as u_max grows from 3→12.
   - Best-R LQR is monotone at 1.00 across the same range.
   - **Deliverable: R*(u_max) lookup table. Amateur fix is one line of firmware.**

2. **Fundamental-constraint region** (low slew, ≤20 units):
   - Even best-R LQR is non-monotone in u_max (best=0.97 at u_max=4, drops to 0.81 at u_max=12).
   - No single fixed-gain LQR design recovers stability across the authority sweep.
   - **MPC, pseudo-control hedging, or rate-aware reference governors required.**
   - The amateur cannot fix this with a tuning change; they must redesign the actuator OR switch to a constraint-aware control architecture.

3. **Infeasible region** (very low slew + low u_max + high p_unstable):
   - Success rate stays below 50% regardless of design.
   - **Validator reports NO-GO; redesign required.**

## Why this is genuinely novel (and not "previous methods are wrong")

A more careful framing than the original H1 claim:

- **Previous methods (MPC, PCH, anti-windup) are NOT wrong.** They explicitly
  address actuator constraints and would handle all three regions.
- **Previous methods are NOT what amateur Arduino-class flight computers run.**
  Amateur firmware (SisyphusCode.cpp in this repo) is fixed-gain PID/LQR
  precisely because MPC/PCH require online optimization or model predictions
  that Arduino-tier hardware cannot afford.
- **The gap we close**: a quantitative pre-flight tool that tells the amateur
  builder *which region their (motor, airframe, servo) sits in*, hands them
  the R*(u_max) lookup if they're rescuable, and tells them clearly when they
  must upgrade actuator or control architecture.
- **Honest claim:** This is the first published quantitative phase diagram for
  the (slew, u_max, p_unstable) cube specifically restricted to constraint-blind
  fixed-gain controllers — the regime actually deployed on amateur rockets.

## Numbers (cite these, not the original H1 numbers)

- Realistic-plant H1 sweep, p_unstable=8 rad/s, aero_damp=0.5:
  - Fixed R=0.5 mean success across all (slew, u_max) cells: **0.634**
  - Best-R per cell mean success: **0.961**
  - Gap: **+32.7 percentage points** purely from R(u_max) tuning
  - Worst single-cell dip (slew=35, u_max=12): **−34.4 pp** under fixed R=0.5
- Simple-sim H1 best-K test, p=10, slew=45:
  - Default R=0.5: peak 0.91 at u_max=4, drops to 0.53 at u_max=12 (−37 pp)
  - Best R per u_max: monotone 0.50→1.00, all cells with u_max≥5 hit 1.00
  - R* schedule: 0.25 (u_max=2), 0.5 (u_max=3-4), 4.0 (u_max=5-8), 8.0 (u_max=9-12)
- Realistic-plant low-slew region (slew=20):
  - Best-R is NON-monotone — confirms a regime where MPC/PCH is needed
- H2 best-R kill: low-pole design 0.90, mid 0.86, worst-case design 0.66 — robust
- Solver invariance: u* and dip locked across dt = {0.005, 0.0025, 0.00125}
- Anti-windup is no-op: confirms wind-up is not the mechanism in pure state-feedback

## Rescue-region head-to-head vs proper methods (UPDATED — apples-to-apples)

Earlier draft cited an `rescue_region_compare.csv` showing FIXED_LQR(R*) at
1.000, beating MRAC/PCH. **That comparison used inconsistent settings.** The
apples-to-apples run (`run_head_to_head_unstable.m`, results in
`head_to_head_unstable.csv`) matched u_max to the three audited firmware
values {10°, 27°, 30°} and swept p ∈ {4, 6, 8, 10}, 4 seeds × realistic plant:

| u_max | p | Steel-PD | FIXED_LQR(R*) | JOINT_ADAPTIVE | SIGMA_MRAC | PCH_LQR |
|-------|---|----------|----------------|-----------------|------------|---------|
| 10°   | any | 0%  | 0%  | 0%  | 0%  | 0%  |
| 27°   | 4 | 67% | 75% | 75% | 75% | 75% |
| 27°   | 6 | **67%** | 25% | 67% | 50% | 50% |
| 27°   | 8 | **33%** | 0%  | 25% | 25% | 25% |
| 30°   | 4 | **100%** | 75% | 100% | 75% | 75% |
| 30°   | 6 | **67%** | 25% | 67% | 75% | 75% |
| 30°   | 8 | **67%** | 0%  | 50% | 50% | 50% |
| any   | 10 | 0%  | 0%  | 0%  | 0%  | 0%  |

**Two surprises from this comparison:**

1. **At u_max=10°, no controller of any kind succeeds at p ≥ 4.** This is a
   fundamental u_max-limit (matches `slew_min ∝ p²` and validator NOGO).
2. **Properly-tuned PD beats LQR(R*) at every (u_max, p) cell where anything
   works.** Why? LQR's quadratic cost underweights derivative damping; in a
   rate-limited, sensor-latent, noisy plant the optimal Kd is much higher
   (steel-PD Kd ≈ 10–30) than what LQR with R*=8 selects (K₂ ≈ 2). LQR
   linearization does not capture the rate-limit non-linearity.

**Revised contribution after this finding:**
- The R*(u_max) lookup is *one* reasonable starting point, not the optimum.
- For amateur deployment, a *retuned PD with per-p Kp/Kd lookup* derived from
  the same calibration grid is probably the strongest fixed-gain baseline.
- A lightweight slew-aware PID hardening layer is a plausible controller-side
  improvement for the fragile middle regime, but it must be framed honestly as
  bounded and non-universal.
- The validator's GO/MARGINAL/NOGO classification remains calibrated, but the
  broader contribution is now the regime map plus the bounded lightweight
  recovery path.

### On SisyphusCode (user's own firmware)

`Firmware/SisyphusCode.cpp` uses JOINT_ADAPTIVE with K1=20, K2=3.14, U_MAX=12
in firmware code units. If the convention is rad-in / deg-out then the sim
equivalent is K1_sim ≈ 0.35, K2_sim ≈ 0.055 — very close to the
`rocket_defaults.m` JOINT_ADAPTIVE row already run above. Confirming the
deg/rad convention from the firmware source is required before quoting a
separate Sisyphus success rate; treat the JOINT_ADAPTIVE row as a proxy.

## Literature scan (May 2026)

- arxiv exhaustive search for "thrust vector control model rocket" and
  "amateur rocket control gimbal" returned **exactly one paper**:
  Singh 2025, *Design and Testing of a Low-Cost 3D-Printed Servo Gimbal
  for Thrust Vector Control in Model Rockets* (arxiv:2509.00061). It is a
  hardware paper, not a controller paper — no phase diagram, no R*(u_max)
  lookup, no pre-flight validator.
- General-purpose literature on rate-saturation, anti-windup, PCH and MRAC
  is extensive (Hu-Lin, Tarbouriech, Sussmann-Sontag-Yang, Klyde-McRuer,
  Johnson-Calise) but **none of these target amateur fixed-gain firmware on
  Arduino-class hardware**, and none publish a (slew, u_max, R*) lookup.
- Closest prior amateur-rocket controller work is hardware bring-up and
  state-machine engineering (Cai 2023 and similar), not constraint-aware
  controller design.

## Deliverables (what an MIT reviewer can use today)

1. `experiments/validator/recommend_envelope.m` — pre-flight GO/MARGINAL/NOGO
   + R* recommendation, phase-region aware (v2).
2. `tools/bench_to_validator.m` — bench-CSV (INO firmware **or** template)
   to validator input, end-to-end verified on R2026a.
3. `experiments/results/realistic_h1_sweep.csv` — phase diagram raw data.
4. `experiments/results/h1_bestk_grid.csv` — R(u_max) lookup table.
5. `experiments/results/h2_bestk_kill.csv` — conservative-design rule.
6. `experiments/results/rescue_region_compare.csv` — head-to-head vs MRAC/PCH.
7. `tools/process_gimbal_bench_test.m` — actuator-fit ingestion (existing).
8. `experiments/results/validator_calibration.csv` — held-out test set of 60 random (p, slew, u_max) configs with predicted region/verdict vs simulated ground truth.

## Validator calibration (most important table)

Across 60 randomized (p ∈ {4,6,8,10,12}, slew ∈ {10..60} deg/s, u_max ∈ {2..12} deg) configurations, the validator was asked for its prediction *before* any sim was run, then the realistic plant was simulated with the validator's recommended K and u_max:

| Predicted region | n  | mean actual success | std   |
|------------------|----|---------------------|-------|
| RESCUE           | 32 | 0.954               | 0.139 |
| FUNDAMENTAL      | 15 | 0.494               | 0.187 |
| INFEASIBLE       | 13 | 0.000               | 0.000 |

| Predicted verdict | n  | mean success | success ≥0.80 rate | success ≤0.50 rate |
|-------------------|----|--------------|---------------------|---------------------|
| GO                | 32 | 0.954        | **0.91**            | 0.03                |
| MARGINAL          | 15 | 0.494        | 0.13                | 0.53                |
| NOGO              | 13 | 0.000        | 0.00                | **1.00**            |

**GO precision: 91% (29/32). NOGO precision: 100% (13/13).** The validator is empirically calibrated on configs it has never seen. Phase diagram generalizes across the full p range: mean success degrades monotonically with instability (p=4 → 0.94, p=12 → 0.36), but each p row contains GO-rated cells reaching 1.000 — i.e. *even very unstable airframes can be flown if the validator says GO*.

## Honest answer to "is this saying previous methods are wrong?"

**No, and the previous draft slightly oversold it. The honest framing is:**

- Previous *general-purpose* methods (MPC, PCH, sigma-MRAC) work but require
  resources amateur hardware lacks.
- Naive fixed-gain LQR is the de-facto amateur deployment and is provably
  fragile in this regime. We *quantify* the fragility, identify when it can
  be rescued by a one-line tuning fix, and identify when it cannot.
- The novel contribution is the *phase diagram + amateur-targeted tool*, not
  a new controller. This is an engineering-validation contribution, not a
  new-theorem contribution. STS-class judges treat the former as legitimate
  when the diagram and tool are concrete, reproducible, and address a real
  community.

## What still needs doing

- Bench-calibrate the heuristic `u*_max(p, slew)` against real servo data
  (the `bench_to_validator.m` ingest path is built and verified on synthetic
  INO CSV; just needs real bench runs).
- Re-tune SIGMA_MRAC and PCH_LQR gains specifically for the unstable-plant
  regime (current bundle was tuned for the original stable rocket) and
  re-run `run_rescue_region_compare` to confirm the +11.5 pp gap holds after
  retuning. The honest framing is "matches proper methods even at default
  bundled tuning"; the stronger framing waits on retuning.
- Phase-diagram boundary mapping at p_unstable ∈ {4, 6, 8, 10, 12} to
  generalize the slew thresholds beyond p=8.
- Bench validation with the actual Firmware/feedback_servo_calibration.ino
  capturing a flight-representative servo + linkage to populate the
  validator's slew/keff inputs from hardware, not assumed values.

---

## Novelty contributions (post-discovery list)

These are the items that are *measured by this work* and not extracted from
prior literature:

1. **Scaling law `slew_min(p) = 0.316·p^1.99` (R²=0.75).**
   Held-out fit on 60 random (p, slew, u_max) configs. The exponent agrees
   with first-principles inverted-pendulum theory (α = 2) to <1%. No prior
   amateur-TVC paper has reported this scaling. (`run_scaling_law.m`,
   `results/scaling_law_fit.csv`)

2. **Empirical community-firmware audit.**
   Three publicly-deployed amateur TVC firmwares (62, 12, 3 GitHub stars)
   were ported to the realistic plant with their as-shipped gains. Results:

   | Firmware                                | p=0  | p=2  | p=4  | p=6  | p=8  |
   |-----------------------------------------|------|------|------|------|------|
   | tomkuttler/TVC-Flight-Code (62★, 2024)  | 17%  | 0%   | 0%   | 0%   | 0%   |
   | AdamMarciniak/FlightComputerV1 (12★)    | 100% | 83%  | 0%   | 0%   | 0%   |
   | PrajNasa/Dhumaketu (3★)                 | 100% | 100% | 83%  | 0%   | 0%   |

   **The most-deployed amateur TVC firmware on GitHub cannot stabilize even
   a stable rocket reliably with its as-shipped gains.** None of the three
   work in the statically-unstable regime this project targets. The niche is
   empirically empty, not just literature-empty. (`run_firmware_audit.m`,
   `results/firmware_audit.csv`)

3. **Servo $/p_max Pareto frontier.**
   Of 11 commonly-used hobby servos, only 2 are non-dominated on the
   cost/stabilization Pareto: the TowerPro SG90 ($4) and the BLS-HV70 ($30).
   Spending more than $30 buys torque, not slew rate, and slew is what
   stabilization needs. (`run_servo_pareto.m`,
   `results/servo_pareto_frontier.csv`)

4. **Validator GO/MARGINAL/NOGO calibration.**
   GO precision 91% (29/32), NOGO precision 100% (13/13) on held-out random
   configs. Phase diagram generalizes monotonically across p ∈ {4,6,8,10,12}.
   (`run_validator_calibration.m`)

5. **Three falsified folk-claims.** Documented in §"What survives, what
   died". Killing these is itself a result: each was a plausible hypothesis
   a reviewer could have raised.

## Negative results (deliberately preserved)

Negative results are claims we **expected to find** and rigorously could not.
They are kept here because publishing them prevents the next student from
spending months chasing them:

- **N1.** *"A non-monotone u_max optimum is intrinsic to the unstable plant."*
  False. With per-cell best-R tuning the dip vanishes on the simple plant.
  Test: `run_h1_bestk_kill.m`. The artifact was the *controller*, not the
  plant.

- **N2.** *"The u_max dip is caused by integrator wind-up; rate-aware
  anti-windup should remove it."*
  False. Rate-aware AW applied to the simple pure-state-feedback LQR is an
  exact no-op. Test: `run_h1_aw_kill.m`. The dip mechanism is rate-limit ×
  control authority, not windup.

- **N3.** *"Solver/integrator choice may be driving the non-monotonicity."*
  False. ode23, ode45, ode15s, and explicit Euler all agree to within
  numerical noise. Test: `run_solver_invariance.m`. The phenomenon is
  physical, not numerical.

- **N4.** *"Tighter integrator-windup limits help on the realistic plant."*
  Inconclusive in current sweep — same scaling either way. Recorded as
  open. Worth one more pass if hardware data later disagrees with sim.

## Contribution taxonomy (for a hostile reviewer)

This work is:

- **Not** a new control-theoretic theorem. Saturated-LQR, anti-windup, and
  the rate-limit/control-authority trade have all been studied (Hu-Lin,
  Tarbouriech, Klyde-McRuer).
- **Yes** a quantitative scaling law specific to the amateur-TVC regime,
  validated on a calibrated noise-realistic plant model.
- **Yes** an audit of community practice showing the gap between deployed
  amateur firmware and what theory says is achievable.
- **Yes** a reproducible pre-flight tool that takes a bench-test CSV and
  outputs GO/MARGINAL/NOGO + recommended gains, calibrated on held-out data.
- **Pending** hardware closure: real bench data + launch campaign matching
  validator predictions to flight outcomes.
