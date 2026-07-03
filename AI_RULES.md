# AI_RULES.md — Project Constraints for All Future Work

> Living document. These are **binding constraints** for AI and human contributors, derived from
> the existing codebase conventions and the project's research philosophy. If a rule must be
> broken, say so explicitly and record why in `DESIGN_LOG.md`.

Last updated: 2026-06-30

---

## 1. Research integrity (overrides everything)
- Act as a **skeptical reviewer**. Do not optimize for exciting conclusions. Falsify before confirm.
- **Report outcomes faithfully**: if a test fails, say so with the output; if a step was skipped,
  say it. Never write expected/aspirational numbers in place of measured ones (subtitles, tables,
  memory, paper all included).
- Quantify uncertainty (CIs, seed counts, CV). A binary pass/fail with `n` seeds cannot resolve
  probabilities finer than `1/n` (see CLAUDE.md 3-seed lesson). Use ≥7 seeds for robustness calls.
- When a number changes, **supersede** it everywhere it is cited (CLAUDE.md, memory, paper, README).
  Do not leave stale numbers next to corrected ones without a staleness flag.

## 2. Coding standards
- Python 3.12. NumPy/pandas idioms. Prefer pure functions and explicit state over hidden globals.
- Parameters are **frozen `@dataclass` `*Params`** objects; never sprinkle magic constants in logic —
  name them (e.g., `CLMAX`, `VE`, `Z_TD`) with a unit comment.
- Construct designs only through `design_space.build_*` / `sample_lhs` / `REF`. Do not hand-build
  plant/actuator/sensor dicts in experiment scripts.
- Vectorize hot loops; in the JS landing sim, keep the RK4 inner loop allocation-free (a regression
  to per-step array allocation cost ~2×, see DESIGN_LOG).

## 3. Naming conventions
- `*Params` (frozen dataclass), `*State` (mutable per-step), `step_<controller>` (one control step),
  `build_<thing>` (factory), `run_<experiment>` (orchestrator), `tools/<topic>_<variant>.py` (study).
- Results CSV mirrors the script: `tools/foo.py` → `experiments/results/foo_py.csv`.
- Seeds: name the disjoint range in the script header; never reuse a prior pass's seed range when
  re-deriving (it conflates classification noise with the effect under study).

## 4. Architecture principles
- `sim/` is the **canonical physics**. New physics goes there first, behind a `fidelity_config` flag.
- Do **not** add a fourth physics implementation. If the JS landing sim or `landing_sim3dof.py` needs
  a physics change, note the duplication in `TODO_AI.md` and keep the change consistent across copies.
- One nonideality per module; composability over monoliths. Keep controllers separable from the plant.
- Prefer small, reversible, well-scoped edits. No speculative refactors.

## 5. Performance constraints
- Experiment scripts run thousands of sims; an inner-loop allocation or redundant `simulate` call is
  a real cost. Profile before "optimizing"; measure warm (JIT/cache) timings, not cold.
- Interactive/browser sims must stay responsive: the landing recompute is debounced and must remain
  well under ~1 s warm. Avoid per-step heap allocation in JS.

## 6. Safety constraints (firmware / hardware)
- `Firmware/` controls real hardware. Never weaken a safety interlock, arming check, or failsafe.
- Hardware-facing changes require a simulation or bench-validation path first (see §9).
- Flight/firmware claims must trace to evidence (`FIRMWARE_VALIDATION_RESULTS.md`,
  `flight_claim_observability_matrix.md`). No unbacked flight claims.

## 7. Documentation expectations
- Every experiment script: a header docstring stating hypothesis, method, seed range, output CSV.
- Every meaningful decision: an entry in `DESIGN_LOG.md` (decision/reasoning/alternatives/tradeoffs/
  risks/date). Every experiment: an entry in `EXPERIMENTS.md`.
- Update `PROJECT_CONTEXT.md` when module structure changes; `REGRESSION_CHECKLIST.md` when new
  user-visible functionality is added.
- Code comments explain *why* (mechanism, unit, gotcha), not *what*.

## 8. Testing expectations
- `cd sim && python validate.py` must pass before reporting any experimental result that depends on
  the core sim. If a change could affect the physics, run it and paste the result.
- New core-sim functionality should add a check to `validate.py` (determinism + a physical sanity
  property). The landing sims have no harness — verify them with a headless Node run (see
  REGRESSION_CHECKLIST.md) and report measured touchdown numbers.
- Prefer a validation harness / simulation **before** any hardware-dependent implementation.

## 9. Simulation-first philosophy
- For control / estimation / planning / tuning work, propose or build a simulation or validation
  harness before hardware-dependent code, whenever practical. Hardware data outranks sim, but sim
  must come first to make a falsifiable prediction the hardware can check.

## 10. Refactoring rules
- Small and reversible by default. **Ask for confirmation before**: large architectural refactors,
  deleting substantial code, changing public APIs (`simulate`, `build_*`, `step_*`, `*Params` fields),
  renaming many files, or reorganizing major structure.
- Do not change a public interface without a strong, stated engineering reason.
- When deleting/overwriting, inspect the target first; if it contradicts how it was described or you
  did not create it, surface that instead of proceeding.

## 11. Logging conventions
- `validate.py` uses PASS/FAIL with ANSI colors. Match that style for new check output.
- Experiment scripts print a one-line provenance summary (n, seeds, key metric) and write the CSV.
- No noisy per-step prints in committed code; gate debug output behind a flag.
