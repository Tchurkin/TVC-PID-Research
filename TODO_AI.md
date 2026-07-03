# TODO_AI.md — Technical Debt & Future Improvements

> Prioritized backlog of debt, risks, and improvements. Not feature work — that lives in the paper /
> STS roadmap. Check items off or move to DESIGN_LOG when resolved. **Do not act on P2/P3 items
> without confirmation** (several imply refactors). P0/P1 are low-risk and can be proposed proactively.

Last updated: 2026-07-03

---

## P1 — Firmware GNC (3-D build, 2026-07-03)
- [ ] **Estimator aiding for the 3-axis gyro-drift tail.** The 3-D nominal baseline is 81/100; the tail is one
      bad gyro-random-walk draw: ~2° attitude drift → gravity leaks into the velocity estimates → knife-edge
      arrests ~0.5 m high (5-6 m/s hops) + ~4 m misses, at every wind incl. calm. Tried and insufficient:
      Q_BIAS loosening (8 Hz position-only GPS can't identify 3-axis leakage). Real fixes, in order of value:
      (a) accelerometer gravity-vector attitude aiding while UNPOWERED (coast: sf ≈ drag, small and known
      direction — near-apogee windows give clean gravity reference), (b) GPS velocity fusion (RMC sentence has
      ground speed/course; the parser only reads GGA), (c) better gyro (BMI088-class random walk).
- [ ] **Chute descent is not modeled in the harness.** Aborted flights free-fall ballistically, so an abort's
      "impact speed" is meaningless; montecarlo.py counts aborts separately (1.3% realistic). Add a simple
      chute drag term on P_CHUTE if abort-drift/landing-zone numbers ever matter.
- [ ] **Margin/roll fin interaction is simplified.** Differential-fold roll torque is modeled independent of
      the common fold (one fin can open from stowed) and lumped across the pair; roll authority direction is
      assumed geometry-fixed. Fine for control design; refine with CFD/bench numbers before trusting margins <2x.

---

## P0 — The validation guardrail is RED (highest priority)
- [ ] **`sim/validate.py` baseline is 7/9 (2 failing) as of 2026-06-30.** The suite that AI_RULES §8
      requires before reporting core-sim results is itself failing. Both failures look like **stale
      tests**, not physics regressions (the other 7 pass, incl. determinism + mini-Exp1):
      1. *Stability physics 2a* — "Uncontrolled (p=1.5) |theta| grew 1.7→1.7°": the test drives
         instability via the **legacy `p_unstable`** (plant_dynamics.py:102 "kept for compatibility"),
         but the current plant diverges through `Cm_alpha × q_dyn` (lines 171-215); in this static
         attitude-hold scenario `q_dyn≈0`, so nothing diverges. **Fix:** rewrite the test to use a
         negative `Cm_alpha`/positive `lambda_aero` with nonzero airspeed, OR mark p_unstable's role
         clearly. Verify against plant_dynamics before deciding test-vs-physics.
      2. *Component isolation 3c* — "Disturbance increases RMS (off=4.12, on=0.82)": RMS is **higher
         with disturbance OFF**, the opposite of the assertion. Likely a stale config/metric after the
         design-space overhaul. **Fix:** trace `build_disturbance(REF)` off-vs-on and the RMS metric.
      Confirm with the maintainer whether to (a) update the tests or (b) treat as a real regression
      before changing `sim/`. Until green, do not treat "validate.py passed" as meaningful for these
      two properties.

## P0 — Reproducibility (low risk, high value)
- [ ] **Add a dependency manifest.** No `requirements.txt`/`pyproject.toml` exists. Pin the actually-used
      libs: numpy, pandas, scipy, scikit-learn, joblib, matplotlib, plotly, hdbscan (Python 3.12).
      A reader cannot currently reproduce results from a clean clone.
- [ ] **Add a one-command setup + validate** (`README` snippet or `Makefile`): create venv, install,
      `cd sim && python validate.py`. Make the entry path obvious.

## P1 — Testability & guardrails (low–medium risk)
- [ ] **Wire `validate.py` into a runnable check** invoked before reporting sim results (manual is fine;
      a tiny `tools/check.py` that runs it + prints PASS/FAIL would help).
- [ ] **No harness for the landing sims.** The JS sim in `landing_interactive.py` has no automated
      check. Add a small headless-Node smoke test (wind=0 soft+vertical+on-target) per
      REGRESSION_CHECKLIST, ideally committed as a script.
- [ ] **`tools/` index.** 173 scripts; current-vs-superseded is only knowable via CLAUDE.md. Generate a
      `tools/INDEX.md` (script → one-line purpose → output CSV → status: current/superseded).

## P2 — Architecture (needs confirmation before acting)
- [ ] **Physics duplication across three sims.** Plant/aero/F-15 motor are reimplemented in `sim/`,
      `tools/landing_sim3dof.py`, and `landing_interactive.py` (JS). They have diverged. Options:
      (a) extract a shared Python physics core the landing sims import; (b) cross-validation test that
      asserts the landing sim matches `sim/` on a reference trajectory; (c) accept divergence but
      document it loudly. Recommend (b) first (cheap, catches drift) before any (a) refactor.
- [ ] **Controllers split between `sim/controller.py` (PID, ADRC) and one-off `tools/` scripts (LQR,
      SMC, MPC).** Consider promoting the validated controllers into `controller.py` with a common
      `step_*` interface so comparisons share one code path. (Refactor — confirm first.)
- [ ] **Landing edge cases** (tg20 in strong wind, far+headwind): control-authority limits of the
      suicide-burn + divert architecture. A boostback-to-above-pad + vertical descent would likely fix
      them uniformly. Bigger change — confirm before pursuing (see EXPERIMENTS 2026-06-30).

## P1 — Landing sim: realistic sensor model
- [x] **DONE 2026-06-30:** realistic IMU+baro sensor fusion added (gyro bias/noise→attitude drift, baro→altitude,
      accel-integration→horizontal vel/pos DRIFT), controller flies on estimates, UI toggle + drift readout. Finding:
      ~1 m position drift caps precision (ideal 0.0 m → realistic ~0.9–1.5 m). See DESIGN_LOG / EXPERIMENTS 2026-06-30.
- [ ] **Beat the drift: add a GPS/vision/beacon (absolute position) toggle** and quantify the precision recovered —
      directly informs the real-flight sensor BOM (do we need GPS for the landing accuracy we want?).
- [ ] **Tighten the sensor realism:** the canard reads a clean lateral force (no accel noise on N_imu); there's no
      attitude-error→horizontal-accel cross-coupling (which makes real INS position drift grow cubically); horizontal
      position has no absolute reference. Build a fuller error budget before trusting the ~1 m number quantitatively.
- [ ] **Far target + strong headwind is a control-authority wall** (tg55 at −7 m/s → ~11 m miss). Not a software
      bug. Only a boostback-to-above-pad trajectory or higher T/W fixes it; flagged, not pursued without confirmation.
- [ ] **Far target + strong headwind is a control-authority wall** (tg55 at −7 m/s → ~11 m miss). Not a software
      bug. Only a boostback-to-above-pad trajectory or higher T/W fixes it; flagged, not pursued without confirmation.

## P3 — Cleanup (low urgency)
- [ ] **57 legacy MATLAB `.m` files** from the pre-Python era, still referenced in `README.md`.
      Decide: archive under `archive/matlab/` and update README, or port if still used.
- [ ] **Stale results in `experiments/results/`.** Many CSVs are superseded (CLAUDE.md documents the
      retractions). Tag superseded CSVs (a sibling `*.SUPERSEDED` note or a manifest) so a reader does
      not cite a retracted run.
- [ ] **CLAUDE.md is very large.** It is the master notebook *and* the de-facto `tools/` index *and*
      the correction history. Consider splitting the index out (see P1 `tools/INDEX.md`) to keep it
      load-bearing but lighter.

## Notes
- Touchdown is firmer in wind for the landing sim (divert tilt steals vertical brake) — inherent to
  fixed thrust; only a throttle/2-burn model removes it. Tracked as a known tradeoff, not a bug.
- Π exponent (τ¹ vs τ²) is under-determined on the latency 1–6 population; a lat 1–12 re-fit with the
  selection-artifact control is the open research item (see EXPERIMENTS / CLAUDE.md).
