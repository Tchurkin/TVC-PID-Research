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

## 2026-06-03 — STS Finalist Readiness Audit + Fixes

**What happened:**
Full audit of workspace against STS finalist requirements.  Fixed 5 code issues,
added 3 new analysis tools, cleaned workspace, committed all prior untracked work.

**Issues fixed:**

1. **Exp5 topology classifier hardened** (`sim/local_analysis.py`)
   Added `cliff_abs_min=0.05` guard — cliff class now requires the gradient spike
   to be absolutely significant, not just relatively large vs. near-zero noise.
   Previous code could classify noisy flat designs as "cliff" due to stochastic spikes.

2. **Multi-seed gradients wired** (`sim/experiment_runner.py`)
   Added `EXP5_N_SEEDS_GRAD = 3` constant. Exp5 gradient field now averages over
   3 simulation seeds instead of 1. Reduces stochastic variance ~1.7×.
   3-seed re-run kicked off; will update exp5_gradient_field_py.csv when done.

3. **p_unstable derivation documented** (`sim/design_space.py`)
   Replaced TODO comment with full EOM derivation sketch showing how the formula
   relates to linearised pitch dynamics. Magic constants (10, 52, 35) explained
   as reference-value calibration factors.

4. **Exp4A decision audit tool** (`tools/exp4a_decision_audit.py`)
   New tool quantifying the Exp4 baseline mismatch.
   Key finding: FRAGILE regime shows only **50.3%** GO/NOGO agreement between
   Exp4 full-fidelity and Exp1 ground truth — the thrust_var fault coin-flips
   half of all fragile designs. INFEASIBLE is unaffected (99.6% agreement).
   Produces 3 new figures in paper/figures/.

5. **Representative trajectory figures** (`paper/make_figures.py`)
   Added `fig_representative_trajectories()` — plots θ(t) and u(t) for the
   median-p_unstable design from each regime. Produces trajectories_by_regime.png.
   This is the "show me the simulation" figure that builds paper credibility.

**Workspace cleanup:**
- Committed 91 stale MATLAB experiment file deletions (Direction_B, ModelRocket outputs)
- Committed all untracked Python simulator work (230+ files) in one structured commit
- Removed ~100 obsolete MATLAB experiment scripts and result CSVs
- All meaningful work now tracked in git

**New figures produced:**
- `paper/figures/exp4a_baseline_mismatch.png`
- `paper/figures/exp4a_decision_flips.png`
- `paper/figures/exp4a_fidelity_complexity.png`
- `paper/figures/trajectories_by_regime.png`

---

## 2026-06-03 — Exp4Simple Paired Comparison + Exp4A Audit Extension

**What happened:**

Implemented and ran `exp4simple`: direct paired evaluation of
`FidelityConfig.simple()` vs `FidelityConfig.full()` for all 1200 designs
(gains frozen at Exp1 values, 3 seeds per condition).  Runtime ~40s.

Extended `tools/exp4a_decision_audit.py` to load and analyse actual paired data.
Added `fig_simple_confusion()` — per-regime confusion matrix.
Added `table_simple_vs_full()` — false approval and false rejection rates.
Total figures now 4 (was 3).

**Key findings (exp4simple):**

Simple-model decision error rates (simple vs full-fidelity):
- INFEASIBLE: **99.6% false approval rate** — a simple simulator approves essentially
  every physically unstable design (only 1 of 235 INFEASIBLE designs is correctly
  rejected by a simple simulator)
- FRAGILE: **50.3% agreement** — coin flip; simple model is useless for borderline designs
- EASY: **78.3% agreement** — simple model mostly correct but fails 21.7% of EASY designs
- **Zero false rejections in all regimes** — simple model is systematically optimistic

Architectural note: The `simple_vs_exp1` comparison showed 100% agreement for
EASY/FRAGILE (trivially — both say GO) and 0% for INFEASIBLE (simple always approves
what exp1 correctly rejects). The `simple_vs_full` comparison is the more informative
metric for publication claims.

**Bug fixed:**
`exp1_go` in `_eval_design_exp4simple` was using `row.get("success_rate", 0)` which
doesn't exist in the Exp1 CSV.  Fixed to use `nominal_success_rate`.

**Artifacts produced:**
- `experiments/results/exp4_simple_vs_full_py.csv` (1200 rows, paired decisions)
- `experiments/results/exp4a_decision_audit.csv` (updated)
- `paper/figures/exp4a_simple_confusion.png` (NEW — confusion matrix, 4 panels)
- `paper/figures/exp4a_baseline_mismatch.png` (updated)
- `paper/figures/exp4a_decision_flips.png` (updated)
- `paper/figures/exp4a_fidelity_complexity.png` (updated)

---

---

## 2026-06-03 — Regime Split + Analysis Hardening

**What happened:**

Four-class regime scheme introduced: EASY / MARGINAL / FRAGILE / INFEASIBLE.
- Old FRAGILE (n=477) split into MARGINAL (n=300, robustness=1.0, backlash-limited)
  and FRAGILE (n=177, robustness<1.0, genuinely wind-sensitive).
- All CSVs relabeled; no re-simulation required.

Exp4 dominance analysis replaced:
- Old: tie-broken "dominant module" (63% of designs had arbitrary tie-breaks).
- New: frequency-of-effect — % of designs where ablating each module causes any
  decision flip, split by positive/negative direction.
- Slew bidirectionality now visible in figure (33% helps / 30% hurts in EASY).
- Thrust-fault caveat added to all Exp4 figure captions.
- New figure: fidelity_effect_frequency.png

Exp5 CSV hardened:
- Added grad_scaled_* columns (range-scaled); raw grad_rms_* retained.
- Topology classification removed from paper figures (not validated vs regime boundaries).
- Regime-stratified servo slew payoff curves added (exp5_slew_stratified_py.csv).

Three methodological robustness analyses added:

1. MARGINAL mechanism (marginal_mechanism.png):
   - Backlash is dominant cause: MARGINAL mean=0.148 vs EASY mean=0.099 (d=0.73)
   - Backlash-RMS correlation r=0.384, p=3.8e-29 within EASY+MARGINAL pool
   - Logistic regression AUC=0.806 (EASY vs MARGINAL classification)
   - Physical mechanism: actuator play creates dead zone; persistent tracking error
   - MARGINAL needs better actuators, not better control

2. Threshold sensitivity (threshold_sensitivity.png):
   - Under +/-20% EASY_RMS_DEG variation: only 8.8% of designs change EASY/MARGINAL
   - FRAGILE count: 177, INFEASIBLE count: 235 — both completely stable at ALL thresholds
   - FRAGILE_SUCCESS_RATE: +/-20% band is within the flat discrete region (3-seed metric)
   - Key sentence: "Regime structure qualitatively unchanged under +/-20% threshold variation"

3. Seed sensitivity (seed_sensitivity.png):
   - 74.4% of Exp4 designs have unambiguous outcomes (success_rate = 0 or 1.0)
   - Population-level SE dominated by n_designs (~2.3pp), not n_seeds
   - Adding seeds 4-5 reduces SE by <0.5pp — below detection threshold
   - Defense: population-level claims are robust; Wilson CI accounts for sampling uncertainty

**Artifacts produced:**
- tools/regime_robustness_analysis.py
- paper/figures/marginal_mechanism.png
- paper/figures/threshold_sensitivity.png
- paper/figures/seed_sensitivity.png
- paper/figures/fidelity_effect_frequency.png
- paper/figures/exp5_slew_payoff_stratified.png
- experiments/results/exp5_slew_stratified_py.csv

---

## Open Items (as of 2026-06-03, end of session)

**Must fix before final claims:**
1. ~~Exp4 baseline mismatch~~ — QUANTIFIED and disclosed in all figure captions.
2. ~~Multi-seed gradient validation~~ — DONE (3-seed run complete).
3. ~~Exp4simple paired comparison~~ — DONE (99.6% INFEASIBLE false approval rate).
4. ~~FRAGILE regime split~~ — DONE (MARGINAL introduced; all CSVs relabeled).
5. ~~Exp4 dominance tie-breaking~~ — DONE (replaced with frequency-of-effect).

**Should do for paper quality:**
6. Increase Exp5 to 5 seeds (Iyy at 13.9% may still be noise)
7. Hardware bench: measure actual servo slew, deadband, backlash
8. p_unstable: replace calibration constants with real EOM derivation from hardware

**Lower priority:**
9. Flight validation campaign (6-12 launches per roadmap)
10. Local surrogate fitting for more reliable Exp5 gradients

---

## 2026-06-05 — Slew Bug Fix + Design Space Overhaul

**What happened:**

Critical bug found: slew formula had a π/180 unit error — servos modeled 57.3× too slow.
Correct formula: `slew_max = slew_deg_s × (REF_U_MAX / REF_MAX_GIMBAL_DEG)` = slew_deg_s × 0.8 CU/s.

All prior results (EASY=850, MARGINAL=241, FRAGILE=68, INFEASIBLE=41) are invalid.

**Design space updated to realistic hardware:**
- servo_slew_deg_s: [60, 200] (previously [5, 200]; slow servos not realistic for hobby TVC)
- static_margin: [-0.30, +0.30] — SIGNED (negative=stable); previously [0.02, 0.30]
- motor_scale: [0.5, 3.0] — new parameter replacing old "thrust" proxy
- T/W > 1 filter added: iteratively resamples until all designs can lift off

**dyn_aero reference pressure fix:**
When dyn_aero=OFF, reference q_dyn is now design-specific (v_mid-based).
Old: hardcoded 540 Pa — inflated aerodynamics 10-100× for most designs.

**Autotune methodology refined:**
2-seed autotune (seeds 1+2), success_rate primary, RMS tiebreaker.
Old single-seed approach caused 87% of designs to default to Kp=5 (gain floor artifact).

**New regime counts (post-fix, [60,200]):**
- EASY: 1144 (95.3%) — later revised to 1169 after final run
- MARGINAL: 43 (3.6%) — identified as slow-servo cluster (median slew=12.5 deg/s)
- FRAGILE: 13 (1.1%) — gain-sensitive
- INFEASIBLE: 0 — gone; was entirely slew artifact

**Rejected finding:**
The Iyy × wind controllability boundary is DEAD. With correct slew rates, no designs are
infeasible. Both Iyy and wind_strength have near-zero correlation with regime (r<0.05).

**Aerodynamic instability tested:**
Direct comparison: stable vs. unstable designs at multiple slew rates and maneuver amplitudes.
Finding: stable designs always outperform unstable; gap grows with maneuver amplitude.
At 10° maneuver, stable=86.8% vs unstable=68.8% at slew=120 (18pp gap). F-16 analogy
fails for hobby TVC (bandwidth margin 2-5× vs 15-40× for F-16).

**Artifacts produced:**
- `experiments/results/exp1_regime_index_py.csv` (regenerated with new design space)
- `sim/design_space.py` — slew fix, signed static_margin, T/W filter, dyn_aero fix

---

## 2026-06-06 — S2R Gain Transfer Study + Gain Mechanism Experiment

**What happened:**

Implemented and ran full S2R (Sim-to-Real) gain transfer study for all 1200 designs.
Protocol: tune Kp/Kd in FidelityConfig.simple() → deploy in FidelityConfig.full() → measure SR gap.

**Key finding (S2R, n=1200):**
Simple model ALWAYS selects Kp=2 for every design (no disturbances → RMS tiebreaker picks Kp=2).
Full physics needs Kp=80 for 67.6% of EASY designs (wind rejection requires high gains).

| Regime   | n    | SR(simple→full) | SR(full→full) | Gap   | False Rejection |
|----------|------|-----------------|---------------|-------|-----------------|
| EASY     | 1169 | 0.958           | 1.000         | 0.042 | 2.8%            |
| MARGINAL | 6    | 0.611           | 0.667         | 0.056 | 33.3%           |
| FRAGILE  | 25   | 0.533           | 0.933         | 0.400 | 52.0%           |

False approval rate = 0.0% everywhere. Old "99.6% false approval" story fully dead.

**New regime counts (final, [60,200] design space, exp1_regime_index_py.csv 2026-06-06):**
- EASY: 1169 (97.4%)
- MARGINAL: 6 (0.5%) — now wind-limited (fast servos ~140 deg/s, high wind 0.325 mean)
- FRAGILE: 25 (2.1%) — median best_Kp=80 (hitting Kp grid ceiling)
- INFEASIBLE: 0

**Gain mechanism experiment implemented (`run_exp4_gain_mechanism`):**
Question: which fidelity module(s) cause the tuner to select Kp=2 vs Kp=40-80?
Priority run: modules [wind, sensor_noise, slew, latency] + pair (wind+sensor_noise).
Extended Kp grid [2,5,15,40,80,160] — extends beyond Exp1 ceiling.
1 tune seed (wind-OFF is deterministic), 3 eval seeds.
Runtime: ~20 minutes on 20 cores. In progress as of session end.

**CLAUDE.md updated:** New thesis, regime counts, S2R table, gain mechanism entry.

**Analysis script prepared:** `analyze_gain_mechanism.py` — runs when CSV is available.

**GAIN MECHANISM RESULT (exp4_gain_mechanism_py.csv, 2026-06-06):**
Phase transition: removing BOTH wind+sensor_noise → 100% pick Kp=2 (super-additive by 28×).
Single ablations: wind alone → 2.4% Kp=2; noise alone → 1.2% Kp=2.
Mechanism: disturbance-free vacuum degenerates the gain landscape — all Kp achieve success=1,
RMS tiebreaker picks minimum. This is structural: NOT a specific physics error, but the absence
of ALL disturbances simultaneously. Either disturbance alone prevents the collapse.
FRAGILE: wind+noise OFF → 52% false rejection. MARGINAL: wind OFF → Kp overshoots to 120.

**Artifacts produced:**
- `experiments/results/exp4_s2r_gains_py.csv` — S2R results (n=1200)
- `experiments/results/exp4_gain_mechanism_py.csv` — gain mechanism (in progress)
- `analyze_gain_mechanism.py` — analysis script
- `run_experiments.py` — added `exp4_mechanism` and `exp4_mech_priority` commands
- `sim/experiment_runner.py` — `run_exp4_gain_mechanism()` + `_eval_design_gain_mechanism()`
- `sim/design_space.py` — fixed stale docstrings (old slew formula, stale static_margin comment)

---

## 2026-06-07 — Physical Predictor + ADRC Discovery + Flight Signature

**What happened:**

Derived the physical formula for gain sensitivity and validated it.
theta_ddot_max = T × sin(max_gimbal_rad) × l_nozzle / Iyy = keff_full × u_max.
AUC=0.911 on n=25 FRAGILE (old labels, Kp=80 grid — later superseded by AUC=0.855 on n=16).

Ran Q-A Kp sweep (n=50 designs, Kp=[1,320], per-design best_Kd):
- kp_floor increases with keff_full (log-log r=+0.582): over-actuation drives floor
- window_ratio collapses with keff_full (r=-0.606) and rises with Iyy (r=+0.611)
- FRAGILE = high keff_full + low Iyy → narrow window
- keff_simple ≠ keff_full discovered: r=0.008 (uncorrelated); FRAGILE 80% have keff_full > keff_simple

ADRC (Active Disturbance Rejection Control) experiment — unexpected finding:
ADRC completely eliminates the FRAGILE problem. 25/25 FRAGILE designs achieve SR=1.000.
Mechanism: ESO cancels wind disturbances independently from tracking control → removes the
coupled wind/gain constraint that creates the floor/ceiling squeeze in PID.
b0 tolerance: 2× mismatch → SR=0.983; wrong keff_simple → SR=0.920. Very robust.
PID at best_Kp: FRAGILE SR=0.754. ADRC: SR=1.000. (n=50, 7 seeds)

theta0_bias_std units bug discovered: simulator.py uses radians, stored as 3.0 = 171.9° std.
Gain mechanism study INVALIDATED — with ~350° initial angles, all Kp fail and tie-break picks Kp=2.
Fix pending: should use 3*pi/180 = 0.05236 rad.

Flight signature experiment (Finding 5):
Single test flight at Kp=2 detects FRAGILE with AUC=0.947 (RMS), precision=1.00, recall=0.80.
Threshold: RMS > 7.6° → confirmed FRAGILE. Works at all wind levels (AUC 0.924-1.000 by tercile).
Outperforms spec formula (AUC=0.890 on same 50 designs).

kp_required formula: kp_optimal = 11.2 × keff_full^0.85 (R²=0.64, r=0.799).
Wind adds only 0.055 to R² — negligible. keff_full is sole driver of required Kp.

**Artifacts produced:**
- `experiments/results/adrc_summary_py.csv` — ADRC vs PID comparison (n=50)
- `experiments/results/flight_signature_py.csv` — flight signature detection (n=50, 7 seeds)
- `experiments/results/kp_surface_py.csv` — kp_required surface (keff × wind)
- `experiments/results/kp_window_sweep_py.csv` — Q-A gain window sweep (n=50)
- `tools/adrc_vs_pid_exp.py`, `tools/flight_signature_analysis.py`, `tools/kp_formula_sweep.py`

---

## 2026-06-08 — ADRC Stress Tests + Pareto Null

**What happened:**

ADRC stress tests (n=1200, full design space):
- ADRC at 10° tilt: 100% SR. PID best: 26% SR. (7 seeds, full physics)
- b0 tolerance: ×0.5 → SR=1.000, ×2 → SR=0.983, ×4 → SR=0.810
- Tripling wind barely changes ADRC failure rate (10% → 12%)
- EASY at ωc=12: rise=0.225s same as PID, RMS=7.2° (4× better than PID)
- High-b0 FRAGILE (b0>15) unstable at ωc≥10; ωc=5 safe for all n=1200

Pareto (robustness vs maneuverability) — null result:
r(nom_sr, rise_time_best) = -0.005. No clean Pareto exists in this formulation.
Authority saturation threshold: theta_ddot > 70 rad/s² → step response saturates at 0.22s;
over_sr drops from 0.997 to 0.704. Below 70: step still improving. Saturation ≠ speed gain.
"FRAGILE is faster" hypothesis DISPROVEN. Practical meaning: authority beyond 70 rad/s²
buys robustness cost with negligible speed gain (+7% in Kp=80 group).

ADRC step tracking (Finding 6, n=1200):
PID SR=0.080 vs ADRC SR=0.972 (12×) for 15° step command.
Tracking RMS: 47.9° (PID) vs 2.3° (ADRC) = 21× improvement.
Improvement universal across all authority bins (11-25×).

**Artifacts produced:**
- `experiments/results/adrc_sweep_py.csv` — ADRC bandwidth sweep
- `experiments/results/adrc_step_tracking_py.csv` — step tracking comparison (n=1200)
- `experiments/results/pitch_tracking_pareto_py.csv` — Pareto analysis
- `tools/adrc_step_tracking.py`, `tools/pitch_tracking_pareto.py`

---

## 2026-06-09 — Relay Study + Governing Equation Chain + Gain Window v2

**What happened:**

Relay autotuning study (n=50, Åström-Hägglund 1984):
Ran relay feedback at Kp=2 (below kp_floor for FRAGILE) to measure bang-bang oscillation amplitude A.
rho(theta_ddot, A_deg) = +0.781; rho(keff_full, A_deg) = +0.812 (slightly stronger).
Power law: A ≈ 0.95° × theta_ddot^0.57 (probe flight at Kp=2).
K_u = 4×u_max/(π×A_rad): r(K_u_measured, K_u_theory) = 1.000 (n=50, exact formula).
EASY K_u median=107 vs FRAGILE K_u median=39 (2.8×, Mann-Whitney p=3.5e-05).
FRAGILE K_u ≈ 40 ≈ wind floor (40-80) → gain ceiling has closed to floor.

Oscillation mechanism: NOT slew-limited. Measured T_u=1.85s vs slew-limit theory T_u=0.17s
(11× mismatch). Oscillation is disturbance-driven bang-bang, not servo-rate-limited limit cycle.
Old slew-based ceiling derivation REJECTED.

Gain window sweep v2 (n=12 designs, per-design best_Kd, 7 seeds):
Window table confirms qualitative pattern: td < 30 → ceiling ≥ 320 (universal tolerance).
td > 100 → ceiling compresses to 127-202 for FRAGILE and some EASY.
OVER_SCALE=1.40 noted: exp1 over test scales both Kp and Kd by 1.40× (not 2×).
R0759 anomaly investigated: mild-FRAGILE SR@1.4x=0.71, expected single-seed failure rate=29%.

**Artifacts produced:**
- `experiments/results/relay_autotune_py.csv` — relay study (n=50)
- `experiments/results/relay_easy_comparison_py.csv` — EASY vs FRAGILE relay comparison
- `experiments/results/kp_window_sweep_v2_py.csv` — gain window sweep (n=12, 7 seeds)
- `tools/relay_autotune_test.py`, `tools/relay_easy_comparison.py`, `tools/kp_window_sweep_v2.py`

---

## 2026-06-10 — autotune_continuous + S2R Update + Theta0 Fix

**What happened:**

**autotune_continuous** implemented — replaces old 5×5 grid (Kp=[2,5,15,40,80], capped at 80):
- Kp log-search [1,320]: 10-point coarse + 6-point refine
- Kd probe [1,4,16,64] at Kp=40; best Kd from probe, then optimize Kp
- Primary: 2-seed SR average; tiebreak: RMS
- Removes Kp=80 cap that had classified 9 genuinely EASY designs as FRAGILE

**New regime counts (autotune_continuous, [60,200] design space, n=1200):**
- EASY: 1171 (97.6%)
- MARGINAL: 11 (0.9%) — wind-limited; nom_sr < 0.80 but robustness=1.00
- FRAGILE: 16 (1.3%) — narrow gain window; 14/16 ceiling-limited
- INFEASIBLE: 2 (0.2%) — R0491 and R0688; nom_sr=0.333 at best Kp up to 320

Key changes from old grid: FRAGILE n=25→16 (9 rescued at Kp>80). INFEASIBLE n=0→2 (needed Kp
uncapped search to confirm they can't be solved). AUC for theta_ddot FRAGILE prediction: 0.911→0.855.

**theta0 fix** (simple model gain study):
Gain mechanism study used theta0_bias_std=3.0 RADIANS (= 171.9° std) instead of 3 degrees.
Fixed to theta0_fixed_deg=10.0 + autotune_continuous (Kp=[1,320]).
S2R result with fix: EASY 12.2% false rejection (up from 3%), FRAGILE 56.2% (up from 52%).
False rejection WORSENED because the root cause is absent disturbances, not initial conditions.

**Updated S2R table (n=1200, theta0=10°, autotune_continuous):**
| Regime   | n    | SR(simple→full) | SR(full→full) | False Rejection |
|----------|------|-----------------|---------------|-----------------|
| EASY     | 1171 | 0.838           | 1.000         | 12.2%           |
| MARGINAL | 11   | 0.424           | 0.667         | 54.5%           |
| FRAGILE  | 16   | 0.417           | 0.938         | 56.2%           |
False approval = 0.0% everywhere.

Bimodal failure modes discovered: 53% under-tuned (high-keff designs get Kp<10 from discrete
ceiling), 17% over-tuned (low-keff designs get Kp=250-320 from RMS minimization).

**Residual analysis for theta_ddot FRAGILE predictor (H1–H4):**
H1 (slew): REJECTED — r=+0.013, delta AUC=+0.000
H2 (wind): REJECTED — r(wind,FRAGILE)=-0.001, FN mean wind < EASY mean wind; delta=-0.014
H3 (aero stability): REJECTED — r=+0.034; delta=-0.006
H4 (interaction ratios): MARGINAL — best delta=+0.004 (below 0.03 threshold)
Best 2-variable: log_td + log_keff = CV-AUC=0.865 (+0.011 over theta_ddot alone).
Conclusion: theta_ddot_max is the sole predictor. FRAGILE is a MECHANICAL property.

**Artifacts produced:**
- `experiments/results/exp1_regime_index_py.csv` — regenerated with autotune_continuous
- `experiments/results/exp4_s2r_gains_py.csv` — updated S2R with theta0=10°
- `experiments/results/excitation_objectivity_py.csv` — theta0 objectivity test
- `experiments/results/stratified_auc_py.csv` — AUC by wind/slew/aero tercile
- `tools/fragile_residual_analysis.py`, `tools/statistical_validation.py`
- `sim/experiment_runner.py` — autotune_continuous added
- CLAUDE.md — major revision (new regime counts, DEAD findings, current thesis)

---

## 2026-06-11 — FN Forensics + STS-Gold Visualization Suite

**What happened:**

**False negative forensics (6-step mechanistic analysis):**
4 designs have theta_ddot < 70 but FRAGILE: R0804 (td=35.7), R0047 (td=40.4),
R0680 (td=62.3), R0452 (td=63.0). All ceiling-limited.

Critical discovery: exp1 over test uses OVER_SCALE=1.40 (scales BOTH Kp AND Kd by 1.40×),
not 2× as previously assumed. Prior fn_forensics analysis used wrong scale throughout.

Corrected evaluation: all 4 FNs GENUINELY fail seed=1 at 1.40×Kp/1.40×Kd with full
fidelity config. FRAGILE classification is CORRECT — not an evaluation artifact.

True SR at 1.40× (7-seed, full fidelity):
- R0804: SR=0.86 (mild — 14% failure probability)
- R0047: SR=0.86 (mild)
- R0680: SR=0.71 (moderate — 29% failure)
- R0452: SR=0.29 (severe — 71% failure)

Critical comparison: TP FRAGILE designs R0084 (td=163), R0373 (td=202), R0405 (td=114)
all show SR@1.4x=0.86 — identical to FNs R0804/R0047. theta_ddot predicts SEVERE FRAGILE
(SR<0.57) but cannot distinguish MILD FRAGILE (0.71-0.86) from EASY. This is expected given
the AUC=0.855 (not 1.0) and wide Youden-J CI [35.7, 96.3] rad/s².

Scientific conclusion: 4 FNs are near-threshold designs in the MILD FRAGILE zone.
No new physical mechanism needed. Keep theta_ddot as sole predictor (AUC=0.855).
Same θ̈→bang-bang→K_u chain operates for all 4 FNs; lower θ̈ → smaller amplitude → milder effect.

**STS-gold interactive visualization suite created:**
Five interactive Plotly HTML files from existing sim data:

- `outputs/sts_gold_0_atlas_3d.html` / `atlas_3d_interactive.html` (UPDATED):
  Replaces old fidelity-module coloring. New story: regime classification.
  Default axes: θ̈_max × wind × Iyy. Yellow plane at θ̈=70 rad/s² threshold.
  Axis-swap dropdowns (7 variables). Hover shows S2R info per design.

- `outputs/sts_gold_1_regime_scatter.html`: Log-scale 3D regime scatter.
  Key visual: FRAGILE (red) spreads across all wind levels → FRAGILE is mechanical.

- `outputs/sts_gold_2_gain_windows.html`: SR(Kp) curves for 12 designs.
  Green (low θ̈, wide window) → red (high θ̈, narrow window).
  Shows window collapse mechanism directly from v2 sweep data.

- `outputs/sts_gold_3_s2r_mismatch.html`: Kp_simple vs Kp_full scatter (n=1200).
  X markers = false rejections. Confirms 56% FRAGILE / 12% EASY / 0% false approval.

- `outputs/sts_gold_4_flight_detection.html`: Box plots of flight RMS at Kp=2.
  7.6° threshold line. FRAGILE 11.3° vs EASY 3.9° (2.9×). AUC=0.947.

- `outputs/sts_gold_index.html`: Master index linking all five figures.

**Artifacts produced:**
- `tools/make_sts_atlas.py` — regenerates atlas_3d_interactive.html
- `tools/make_sts_flight_sig.py` — generates flight detection boxplot
- `tools/fn_forensics.py` — mechanistic FN investigation (6-step)
- `outputs/sts_gold_*.html` — STS-gold visualization suite (tracked in git)

---

## Status: Simulation Phase Complete (2026-06-11)

All six primary findings are confirmed from simulation:
1. p_unstable has near-zero correlation with regime (r≈0, confirmed)
2. θ̈_max = T·sin(δ)·L/Iyy predicts FRAGILE with AUC=0.855 [0.765,0.931] (CV)
3. Aerodynamic instability does not help maneuverability
4. Simple-model gains cause 56% false rejection for FRAGILE, 12% for EASY, 0% false approval
5. Single flight at Kp=2 detects FRAGILE: AUC=0.947, precision=1.00 (RMS > 7.6°)
6. ADRC eliminates gain sensitivity: 12× better SR, 21× better tracking RMS vs PID

Next phase: hardware flight validation (6-12 flights per roadmap).
Priority: Kp=2 detection experiment → Kp_simple vs Kp_full improvement → θ̈ threshold hardware test.

---

## 2026-06-11 — Paper Draft Complete Rewrite

**What happened:**

`paper/Tentative_Paper_Draft.md` rewritten from scratch. Previous draft was completely outdated:
described MATLAB workflow, PCH controllers, slew_min scaling law, and "infeasible regimes" as the
primary story — none of which reflect the current study.

**New paper structure:**
- Title: "Simulator Fidelity and Gain Selection in Hobby-Scale TVC Rockets"
- Working title: "When does your flight simulator lie to you?"
- Abstract: θ̈_max formula → AUC=0.855 → 56% false rejection → ADRC fix
- Introduction: The 4-hypothesis negative result journey (3 of 4 wrong)
- Background: What is/isn't novel; explicit positioning against prior literature
- Methods: Python sim, LHS n=1200, autotune_continuous, T/W filter, all 6 regime criteria
- Results: All 6 findings with correct numbers, confidence levels, and limitations
- Discussion: Sim-to-real narrative, FRAGILE base rate (1.3%), ADRC as architectural fix
- Hardware validation plan: Priority A-D experiments, minimum success criteria
- STS framing: 4-hypothesis narrative; probability estimates with and without hardware
- Appendix: Key data files table, rejected findings list (6 documented)

**Artifacts produced:**
- `paper/Tentative_Paper_Draft.md` — complete rewrite

---

## 2026-06-12 — Relay Study Rerun + Paper K_u Validation

**What happened:**

**Relay rerun with current labels (n=41: 25 EASY + 16 FRAGILE):**
Previous relay study used Kp=80-capped labels (n=50 old labels). K_u claim was based on
old "FRAGILE" which included 16 designs now correctly EASY. Reran relay_easy_comparison.py
to get K_u for all 16 current-FRAGILE designs.

**Results (correct labels):**
- EASY median K_u = 108 [18, 356] (n=25 spanning full theta_ddot range)
- FRAGILE median K_u = 39 [24, 128] (n=16, all current FRAGILE)
- Ratio = 2.80×, Mann-Whitney p = 0.0072 (p < 0.01, significant)
- Step 2: rho(theta_ddot, A_deg) = 0.616, p=1.8e-5 (was 0.781 with old labels)
- Power law updated: A ≈ 1.63° × theta_ddot^0.40 (was 0.95° × theta_ddot^0.57)

**Key findings:**
1. K_u separation 2.8× holds at median — core claim VALIDATED
2. R0115 outlier: K_u=128 but best_Kp=253 (high-floor FRAGILE). Probe at Kp=2 measures
   wind-driven oscillation, not bang-bang. Excluding R0115: FRAGILE median=38, p=0.0037.
3. r(log_theta_ddot, log_K_u) = +0.004 at individual level (essentially zero).
   The ceiling-compression is a GROUP-LEVEL effect, not individual-level.
   keff_full is the better individual predictor: r=-0.407, p=0.008.
4. FN FRAGILE designs (R0804, R0047, R0680, R0452) have K_u median=38 — same as TP FRAGILE.
   NOT "milder K_u compression" as previously claimed. The mild classification refers
   to SR@1.4x, not K_u. Paper Section 4.5 corrected.
5. 15/16 FRAGILE have over_sr=0.0 (ceiling-limited). Direct evidence for ceiling compression
   from n=1200 classification, independent of relay study.

**Paper updates (paper/Tentative_Paper_Draft.md):**
- Abstract: rho=0.62, K_u=108, p=0.0072
- Section 4.4 Step 2: updated rho, n, power law, amplitude range
- Section 4.4 Step 4: updated K_u table, p-value, R0115/R0336 notes
- Added group-level caveat: r(log_td, log_K_u)=0.004 at individual level
- Added direct ceiling evidence: 15/16 FRAGILE ceiling-limited from n=1200
- Section 4.5: corrected FN K_u claim (same K_u as TP, not milder; keff_full is driver)
- Executive Summary: updated mechanism chain numbers

**Artifacts produced/updated:**
- `experiments/results/relay_easy_comparison_py.csv` — rerun with current labels (n=41)
- `paper/Tentative_Paper_Draft.md` — K_u and mechanistic chain sections updated
- `paper/Executive_Summary_STS.md` — mechanism diagram updated
- `CLAUDE.md` — governing equation chain numbers updated


## 2026-06-13 -- H5 Latency Discovery: Combined Predictor AUC=0.924

**Trigger:** User asked to add latency limitation to paper and verify all design space ranges are
realistic for hobby TVC hardware. Investigation revealed latency was ALREADY in the LHS design
space [1-6 steps = 5-30ms] and is a significant independent predictor of FRAGILE.

**Discovery: latency_steps omitted from H1-H4 exhaustive search**
tools/fragile_residual_analysis.py used 10 features: log_td, log_keff, log_wind, log_slew,
wind_x_keff, wind_x_td, td_div_slew, keff_div_slew, static_margin, log_authority.
latency_steps was in the design space but NOT in the feature list -- an oversight.

**H5 Result (latency_steps, hardware variable):**
- r(latency_steps, FRAGILE) = +0.139
- All 16 FRAGILE have latency >= 4 steps (20ms+); binomial p = 2.04e-05 vs 50.9% base rate
- r(latency_steps, theta_ddot) = -0.033 -- INDEPENDENT, not a proxy
- AUC(latency alone) = 0.836 (10-fold CV)
- AUC(theta_ddot + latency) = 0.924 +/- 0.085 (10-fold CV); delta = +0.072
- log(theta_ddot x latency) single variable gives identical AUC = 0.924 -- multiplicative
- P(delta > 0.03) = 1.00 over 3000 bootstrap resamples

**Physical mechanism:**
theta_ddot x tau_latency [rad/s^2 x s = rad/s] = max angular velocity before loop responds.
Phase lag at 5Hz: 1-step=9 deg, 3-step=27 deg, 6-step=54 deg -- independently compresses ceiling.
All 4 FN designs (R0804, R0047, R0680, R0452) have latency_steps=6 (30ms maximum).

**REVISED NARRATIVE:**
OLD: "FRAGILE is MECHANICAL. H1-H4 all rejected. theta_ddot is optimal (AUC=0.855)."
NEW: "FRAGILE is HARDWARE: mechanical authority (AUC=0.855) + control loop latency (combined=0.924).
     H1-H4 (environmental) rejected; H5 (hardware latency) confirmed."

**Design space realism:** latency [1-6] = [5-30ms] covers Teensy to slow Arduino. Realistic.
Behavior >30ms untested -- noted as limitation in paper. No rerun needed.

**Files updated:**
- paper/Tentative_Paper_Draft.md (Sec 1.3, 4.2, 4.5, 7.3, 9.1)
- paper/Executive_Summary_STS.md (hypothesis table)
- CLAUDE.md (H5, revised FN explanation, updated PHYSICAL INTERPRETATION)
- memory/authority-iyy-ratio.md, memory/project-direction.md

