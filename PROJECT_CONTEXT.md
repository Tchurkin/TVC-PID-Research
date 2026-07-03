# PROJECT_CONTEXT.md — Architecture & Module Relationships

> High-level map of the repository for humans and AI collaborators. Keep this current when
> modules are added/removed or responsibilities shift. For *research findings* see `CLAUDE.md`
> and `memory/`; for *decisions* see `DESIGN_LOG.md`; this file is **structure only**.

Last updated: 2026-06-30

---

## 1. Purpose

Research codebase for **quantifying the stability / controllability boundary of amateur
thrust-vector-control (TVC) rockets** (STS 2027). The contribution is a *measurement-to-decision
pipeline* and an empirical stability map — **not** new control theory. The primary scientific
goal is understanding design-space structure and simulator-fidelity requirements. See `README.md`
for the mission and headline results.

## 2. Repository Map

| Path | Role |
|------|------|
| `sim/` | **Core research simulator** (Python). The source of truth for the physics. |
| `sim_vsr/` | Variable-stability-rocket sim — margin-modulation / canard control research. |
| `tools/` | Current landing/VSR sims (landing_*, vsr_*, variable_stability, feasibility). The ~126 sim-phase Π/Exp study scripts were removed 2026-07-01 (recoverable from git history/GitHub). |
| `experiments/results/` | Sim-phase evidence CSVs were removed 2026-07-01 (on GitHub history). Was the archived-paper evidence base. |
| `paper/` | Manuscript + archive (`paper/archive/2026-06-25_sts_candidate/`). |
| `Firmware/` | Arduino/C++ TVC firmware + flight journals. |
| `outputs/` | Generated HTML visualizations (self-contained). |
| `sim/` `sim_vsr/` `tools/landing_*` | **Three separate simulators — see §4.** |
| `memory/` | AI long-term memory (structured findings index in `MEMORY.md`). |
| `CLAUDE.md` | Master lab notebook / context doc (large; the de-facto `tools/` index). |
| `archive/`, `data/`, `docs/`, `Sources/`, `Rocket data/` | Supporting material. |
| `*.m` (57 files) | **Legacy MATLAB** from the pre-Python era. Do not extend; migrate if touched. |

## 3. Core Simulator (`sim/`) — module responsibilities

Physics modules (each owns one nonideality, composable via `fidelity_config`):
- `plant_dynamics.py` — rigid-body rotational + translational dynamics, `PlantParams`.
- `aero_model.py` — normal force, drag, pitch damping, static margin.
- `actuator_model.py` — TVC servo: slew, travel, deadband, backlash; `ActuatorParams`.
- `sensor_model.py` — gyro/accel noise, bias, latency; `SensorParams`.
- `motor_model.py` — thrust curve (F-15 class), mass/CG over burn.
- `disturbance_model.py` — wind / gusts; `DisturbanceParams`.

Orchestration & analysis:
- `simulator.py` — `simulate(...)`, `ScenarioParams`; integrates the plant under a controller.
- `controller.py` — `PIDParams`/`step_pid`, `ADRCParams`/`step_adrc`. (LQR/SMC/MPC are one-off
  scripts in `tools/`, **not** in the core controller — see TODO_AI.md.)
- `design_space.py` — `sample_lhs`, the `REF` reference design, `build_plant/actuator/sensor/
  disturbance/scenario` factories. **All experiments construct designs through these.**
- `fidelity_config.py` / `fidelity_atlas.py` — fidelity ablation (which nonidealities are on).
- `experiment_runner.py` — `run_exp1` etc.; the large experiment orchestrator (~2.8k LOC).
- `experiment_protocol.py`, `local_analysis.py`, `units.py` — protocol, local sensitivity, units.
- `validate.py` — **the test suite** (determinism, stability physics, isolation, monotonicity,
  mini-Exp1). Run: `cd sim && python validate.py`.

Data flow: `design_space.build_*` → `simulator.simulate` → metrics → `experiment_runner` →
`experiments/results/*.csv` → `tools/*.py` analysis → `outputs/*.html` + paper.

## 4. The three simulators (important architectural tension)

There are **three independent physics implementations**:
1. `sim/` — Python, the research source of truth (validated by `validate.py`).
2. `tools/landing_sim3dof.py` — Python, a separate 3-DOF landing model (own `f15`, aero, deriv).
3. `tools/landing_interactive.py` — generates a **self-contained JS** sim (own physics, controllers,
   ADRC, wind) embedded in `outputs/landing_interactive.html`. `tools/landing_animate.py` is a
   related static-animation generator.

These **share no code** and have diverged (e.g., the F-15 curve and aero are reimplemented in each).
The JS landing sim cannot be checked by `validate.py`. Treat `sim/` as canonical; when a landing-sim
result needs to be trusted as physics (not a demo), cross-check against `sim/`. See TODO_AI.md.

## 5. Documentation hierarchy

- `README.md` — mission, contribution, headline results (public-facing).
- `CLAUDE.md` — master research notebook: every claim → script + CSV, plus the correction history.
  Authoritative for *what is currently believed* and *what was retracted*.
- `memory/MEMORY.md` + `memory/*.md` — AI long-term memory (one fact per file).
- `PROJECT_CONTEXT.md` (this) — structure. `AI_RULES.md` — constraints. `DESIGN_LOG.md` — decisions.
  `EXPERIMENTS.md` — experiment records. `REGRESSION_CHECKLIST.md` — what to re-verify.
  `TODO_AI.md` — technical debt.

## 6. Conventions (observed; codified in AI_RULES.md)

- Parameters are frozen `@dataclass` `*Params` objects; designs built via `build_*` factories.
- Determinism via explicit integer seeds; disjoint seed ranges per correction pass.
- Results are CSVs under `experiments/results/`; one script ↔ one (or few) CSV.
- "Skeptical reviewer" research philosophy: falsify before confirm; quantify uncertainty;
  re-derive on corrected populations (see CLAUDE.md correction history).
