# TVC PID Research — Project Log

Chronological record of major work sessions, decisions, and findings.

---

## 2026-06-02 — Python Simulator Rebuild

**What happened:**
Complete rewrite of the TVC simulator in Python (replacing MATLAB).
9 modular files: plant, actuator, sensor, disturbance, scenario, controller, simulator, design_space, experiment_runner.

All 10 validation tests pass. Python is now the source of truth.

**Key fix:** p_geom double-count bug found and resolved in design_space.py.

**Artifacts produced:**
- `sim/` — all simulator modules
- `experiments/results/exp1_results_py.csv` (N=1200 Exp1)
- `experiments/results/exp4_results_py.csv` (N=1200 Exp4, original cumulative version)

---

## 2026-06-03 — Exp4 Fidelity Ablation + Exp5 Topology (Full Run)

**What happened:**
- Added `sim/fidelity_config.py` — independent 7-module toggle system (replaces cumulative L0-L5 ladder)
- Added `sim/local_analysis.py` — gradient/curvature/topology/evolution path tools for Exp5
- Extended `sim/experiment_runner.py` with `run_exp4_ablation()` and `run_exp5_landscape()`
- Ran full N=1200 Exp4 ablation (3 seeds/condition) and Exp5 landscape in parallel
- Total runtime ~20 min (20 CPU workers)

**Key architectural decisions:**
- Gains frozen at Exp1 best_Kp/best_Kd for both Exp4 and Exp5 (measures physical sensitivity, not controller adaptability)
- Exp4 uses delta_success (GO/NOGO flip) as primary metric, not delta_rms
- Exp5 gradients are range-scaled to remove unit bias (Iyy kg·m² vs dimensionless parameters)

**Bugs fixed this session:**
- `settling_time_s` KeyError in `_eval_one_fidelity` (missing fields in runs dict)
- `estimate_p_unstable` NameError in diminishing returns (missing import)
- Unicode `±` encoding error on Windows cp1252 console
- `multi_class` kwarg removed in sklearn 1.8 (LogisticRegression)
- Unit bias in gradient ranking — Iyy appeared dominant due to tiny kg·m² units; fixed with range-scaling
- Diminishing returns was sweeping a single INFEASIBLE outlier; fixed with population sweep (100 designs × 40 points)

**Artifacts produced:**
- `experiments/results/exp4_ablation_study_py.csv`
- `experiments/results/exp5_gradient_field_py.csv`
- `experiments/results/exp5_evolution_paths_py.csv`
- `experiments/results/exp5_diminishing_returns_py.csv`

---

## 2026-06-03 — Fidelity Dominance Analysis + Figures

**What happened:**
- Created `tools/fidelity_dominance.py` — regime-stratified fidelity dominance using delta_success
- Added dual-metric analysis: compares RMS-dominant vs Decision-dominant modules per design
- Produces 6 publication-quality figures

**Key findings:**

Wind is decision-dominant in all regimes (not sensor_noise as previously believed):
- EASY: Wind 55%, Slew 25%, Sensor noise 7%
- FRAGILE: Wind 50%, Slew 21%, Sensor noise 20%
- INFEASIBLE: Wind 73%, Sensor noise 22%

Metric choice determines the ranking:
- RMS-dominant agrees with decision-dominant only 22.5% (EASY), 45.9% (FRAGILE), 85.1% (INFEASIBLE)
- Sensor noise tops RMS ranking; wind tops GO/NOGO ranking

Fidelity complexity by regime:
- EASY: 47% of designs need zero modules (simple simulator OK)
- FRAGILE: 22% can use simple simulator
- INFEASIBLE: 0% can use simple simulator (all need 2-3 modules)

**Known open issue — Exp4 baseline mismatch:**
Exp4 "full fidelity" includes a thrust_var fault (keff drops 15% at t=1.5s) that was NOT present
when Exp1 regime labels were computed. Only 71.3% of designs agree on GO/NOGO between the two
conditions. This is a methodological concern that must be resolved before publication.

**Figures produced:**
- `paper/figures/fidelity_regime_bars.png`
- `paper/figures/fidelity_spatial_scatter.png`
- `paper/figures/fidelity_handoff.png`
- `paper/figures/fidelity_multisensitivity.png`
- `paper/figures/fidelity_module_dominance_maps.png`
- `paper/figures/fidelity_metric_agreement.png`

---

## Open Items (as of 2026-06-03)

**Must fix before final claims:**
1. Exp4 baseline mismatch — align Exp4 full-fidelity with Exp1 evaluation conditions
2. Multi-seed gradient validation for Exp5 (current: single seed, noisy)

**Should do for paper quality:**
3. Representative trajectory figures — 3 example flights (EASY / FRAGILE / INFEASIBLE)
4. p_unstable physical derivation — replace magic constants with proper EOM

**Lower priority:**
5. Local surrogate fitting for more reliable Exp5 gradients
6. Flight validation: bench + actual flight to falsify simulator predictions
