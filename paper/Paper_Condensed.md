# A Control-Authority Parameter Governing Gain-Window Limits in Hobby-Scale TVC Rockets

**Braxton Herold**

---

## Thesis

A TVC attitude-hold controller's safe gain window is governed primarily by its **ceiling**, which falls with control-loop latency:

> **Kp_ceiling ≈ 0.042 / τ [rad/rad]** &emsp; (DIPDT phase-margin; keff-independent)

The gain floor — minimum Kp for wind rejection — is near-zero under optimal gain selection. **Window risk is ceiling-dominated.** Each latency doubling compresses the window by approximately 2× (ceiling-only). Going from Arduino-class (10-step latency) to Teensy-class (2-step) improves the ceiling from ~0.84 to ~4.2 [rad/rad] (5×).

⚠️ **Correction (2026-06-22):** An earlier version of this paper reported a "double squeeze" — floor rising as τ⁺¹ and ceiling falling as τ⁻¹ giving τ⁻² net compression (3.7× per latency doubling). The floor formula (Kp_floor ≈ 0.06 × keff × latency) was a frozen Kd = 0.022 s measurement artifact. Under existential Kd search, floors are near-zero. The 3.7× figure is wrong; use **~2× per latency doubling** (ceiling only). The floor formula and "double squeeze" section are removed below.

The governing risk parameter is:

> **Π = k_eff × τ²** [-]  (dimensionless)

where k_eff = T_avg × L_nozzle / Iyy [s⁻²]. The ceiling 0.042/τ is keff-independent across the full design space (keff regression coefficient ≈ 0; formula approximately valid in both delay-dominated and saturation-dominated mechanism regimes). Π predicts calibration failure: disturbance-free autotune calibration failure rate is 9.1% at Π < 0.34 vs. 61.3% at Π ≥ 0.34 (6.7× jump; n = 2,400). Π ≳ 0.32 → enter saturation regime → use ADRC. *(Scope: the 0.32 onset is empirical, within FIFO-delay implementations; bare-metal integer FIFO onset ≈ 0.20; IIR rate filtering eliminates it. See full paper Section 4.4.6.)*

k_eff contains no max_gimbal_deg. **A builder assessing gain-window risk needs thrust, nozzle moment arm, and Iyy — not gimbal travel.**

**Single-design falsification of "authority alone causes danger" (R0522):** the dataset's highest-authority design (k_eff ≈ 1,900 s⁻², the maximum in 2,400 LHS designs) runs at latency = 1 step, giving Π = 0.05 — far below saturation onset. Measured fsat = 0.018, SR = 1.000. A keff-only predictor flags R0522 as the most dangerous design in the study; Π correctly identifies it as perfectly safe. Authority without delay produces no transition — only the product keff × τ² does.

**The argument in six steps** (each link a distinct experiment):
1. **Classical delay theory holds** — a latency-determined ceiling Kp ≈ 0.042/τ exists (DIPDT; §3.1). The anchor, not the novelty.
2. **A delay–authority parameter, not authority alone, organizes the behavior** — Π = keff × τ²; R0522 is the one-line falsification of authority-alone explanations (above).
3. **A regime transition exists** — fsat rises monotonically with Π (binned means 0.007 → 0.59 across Π tiers), onset near Π ≈ 0.2–0.4 (saturation regime map, **n = 142** after replication; ρ(log Π, fsat) = **+0.55**, revised down from an optimistic n = 29 ρ = 0.80; **Figure 1**). Caveat: on this latency-1–6 population Π ≈ k_eff in rank terms (ρ 0.55 vs 0.54) — the τ²-specific evidence is R0522 + the lat-1–12 stress data + theory, not this correlation. A transition band, not a magic constant.
4. **The transition is causally tied to saturation — the load-bearing result** — remove slew saturation and PID recovers: SR = 1.000 for all 15 (2×2 factorial, §4) and SR ≈ 0.99 across 142 designs (regime-map sample, §3). Strongest, most replicated, least sample-fragile result; steps 2–3 locate the onset, step 4 establishes the cause.
5. **Architecture matters exactly here** — ADRC's advantage over optimal PID emerges in the same region where saturation becomes dominant (§4). A coincidence-of-onset claim, not a full causal account of why ADRC wins.
6. **The mechanism is isolated by ablation** — a rate filter that smooths the derivative channel while preserving total delay removes the transition (§4.4.6). Equal delay + smoothed transients → no transition ⇒ transients, not delay duration, drive it. Mechanism by falsifiable prediction, not correlation.

---

## Abstract

**The central, most robust finding is causal:** delayed TVC attitude control has a saturation-dominated regime in which classical PID tuning fails, and the failure is *caused* by servo slew saturation — across **142 designs**, replacing the rate-limited servo with an idealized infinitely-fast one restores PID success to **SR ≈ 0.99**, including designs where the real servo gives 0.60 or worse. The dimensionless authority–delay parameter **Π = k_eff × τ²** organizes *where* that transition begins: below a band (Π ≈ 0.2–0.4) the servo operates linearly and any sub-ceiling gain works; above it, aerodynamic disturbances lock it into the persistent bang-bang PID cannot tune around, and where PID and ADRC diverge. The correlations, regression, and ceiling law below locate and quantify the onset; the causal result above is the one least likely to move under additional data.

The safe proportional gain window for a hobby-scale TVC rocket is governed primarily by its ceiling: **Kp_ceiling ≈ 0.042/τ [rad/rad]** (DIPDT phase-margin; keff-independent across the full design space). The floor is near-zero under optimal gain selection. Window risk is ceiling-dominated; each latency doubling compresses the window by ~2×. The risk parameter Π = k_eff × τ² predicts saturation onset (Π ≳ 0.20–0.32 for FIFO implementations; see scope note) and calibration failure.

Π also predicts disturbance-free calibration failure: false-approval rate is 9.1% at Π < 0.34 vs. 61.3% at Π ≥ 0.34 (6.7× jump; n = 2,400; 50% threshold at Π ≈ 0.41). Mechanism: high keff drives still-air autotune to Kp ≈ 7.0 [rad/rad]; high latency compresses the real ceiling below 7.0; Π captures both.

A continuous regression (n = 262 designs, CV R² = 0.33 ± 0.09, p = 5.8×10⁻¹¹) confirms a monotone dose-response from td = 0 to ≈300 rad/s², plateauing above that with latency as the binding risk factor. Binary secondary result: AUC = 0.975, Cohen's d = 3.71. Controlled single-airframe experiment (only Iyy varies): window 142× → 33× with no other parameter changed.

Reactive controllers (PID, LQR, SMC, MPC H=1) all face ρ ≈ −0.75–−0.81; non-reactive approaches (ADRC ESO, MPC H=5) escape it (full-physics audit 2026-06-24: H=5 ρ = −0.052, p = 0.718). A 2×2 saturation factorial (n = 15) confirms slew saturation is both necessary and sufficient for PID failure: PID-without-saturation achieves SR = 1.000 even at Π_td = 14.5. ADRC's ESO prevents saturation (slew_frac = 0.000) by canceling disturbances upstream; H=5 MPC prevents saturation by planning bounded command sequences ahead.

---

## 1. Introduction

Hobby TVC builders tune gains in disturbance-free simulators — no wind, no slew limits. In still air, higher Kp gives faster step recovery until discrete-time ringing. This approach reveals nothing about two boundaries governing real flight: the **wind-rejection floor** (minimum Kp to damp gusts) and the **limit-cycle ceiling** (maximum Kp before slew saturation drives bang-bang oscillation). For gain-sensitive designs, a still-air-tuned gain falls outside the real-physics window **63.9% of the time**.

Four hypotheses about what determines gain sensitivity were screened across n = 2,400 designs:

| Hypothesis | Verdict | Precise statement |
|---|---|---|
| H1: High wind narrows window | Not confirmed | Wind magnitude does not predict gain-window width after controlling for authority and latency (r = −0.008; adding wind to the best model lowers AUC) |
| H2: Slow servos narrow window | Rejected | r = +0.048; adding slew lowers window_ratio CV R² by 0.017 |
| H3: Aerodynamic instability | Rejected | r = −0.001; static_margin adds nothing after authority |
| H4: Environmental interactions | Rejected | All pairwise deltas negative |
| H5: Control-loop latency | Confirmed | ΔAUc = +0.028; combined AUC = 0.972; two false-negatives both have latency_steps = 6 |

The answer is Newton's second law for rotation: θ̈_max = T·sin(δ_max)·L_nozzle / Iyy, combined with control-loop latency.

---

## 2. Methods

**Design space (n = 2,400 LHS):** mass [0.50, 1.20] kg; Iyy [0.005, 0.100] kg·m²; servo_slew [60, 200] °/s; motor_scale [0.5, 3.0]; max_gimbal [2, 15]°; latency [1, 6] steps; wind_strength [0.05, 0.45]; deadband, backlash, static_margin. T/W > 1 filter enforced.

**Simulator:** 10 toggleable physics modules (nonlinear aero, dynamic pressure, thrust curve, CG shift, slew, backlash, deadband, latency, Ornstein–Uhlenbeck wind, sensor noise). All active unless stated. 200 Hz control loop.

**Physical predictor:**
```
keff = T_avg × L_nozzle / Iyy    [s⁻²; rotational authority per radian of gimbal; independent of max_gimbal]
τ    = latency_steps / loop_hz    [s; total loop delay]
Π    = keff × τ²                  [-; dimensionless risk parameter]
```

**Gain search:** Kp log-search [1, 320] (10-point + 6-point refine). Final correction used a finer 18×7 joint Kp×Kd grid (126 combos, 30 fresh evaluation seeds).

---

## 3. The Ceiling-Dominated Gain Window

### 3.1 Theoretical derivation

**Ceiling** comes from phase-margin analysis of a PD controller with double-integrator plant and time delay (DIPDT framework). When k_eff × τ < 1 (the hobby-TVC regime): Kp_ceiling ≈ 0.9/τ [keff-independent]. Empirical correction for slew-saturated bang-bang: 2.1× → **Kp_max ≈ 0.042/τ [rad/rad]** at 200 Hz. Measured latency exponent: −1.036 (theory: −1.0). Measured keff exponent: +0.059 ≈ **0** (theory: 0). Both confirmed within 6% of prediction.

**Floor** is near-zero under optimal gain selection. ⚠️ This document previously stated "Kp_floor ≈ 0.06 × keff × latency" — that formula was derived with a frozen Kd = 0.022 s across all designs, which is suboptimal and inflates measured floors. Under an existential Kd search (the correct protocol), floors are near-zero for the vast majority of hobby-TVC designs. The floor formula has been retired. Do not cite it.

**Window ≈ ceiling ≈ 0.042/τ [rad/rad].** Each latency doubling → ~2× compression (ceiling-only). ⚠️ The previous "3.7× per latency doubling" figure came from the now-retired floor formula and is wrong.

**Dual bang-bang regimes** confirm two ceiling *mechanisms*. Low-authority (td ≈ 22 rad/s²): slew_frac ≈ 0.06 throughout — linear proportional regime; ceiling is DIPDT phase-margin. High-authority (td ≈ 208): slew_frac ≈ **0.63 at every Kp** — permanent bang-bang from wind. Smith predictor test confirms the two-mechanism picture: at keff ≤ 550 s⁻², Smith raises ceiling 1.0–8.2× — minimal at low latency (1.0–1.4× at lat = 3), strongest at high latency (2.5–8.2× at lat = 6) — (delay-dominated); at keff = 1,146 s⁻², Smith's linear model collapses to 1 valid Kp while PID retains a 50× window at ceiling ≈ 203 CU — the formula value is approximately preserved in both regimes, but the mechanism differs. ADRC (ESO) succeeds in both by preventing saturation.

**On the τ² exponent (scope — predicted, not proven).** The window-compression exponent is τ⁻¹ (ceiling-dominated). The *second* power of τ in Π = k_eff × τ² comes from a different argument: the blind-spot kinematics, where displacement during one blind window grows as ½·k_eff·τ². That derivation *predicts* τ², and the saturation collapse onto k_eff × τ² (ρ ≈ 0.80, matching the analytical blind-spot ratio) is *consistent with* it — but a controlled latency sweep at fixed authority could not cleanly separate τ^1.6 from τ^2.0 from τ^2.2. Stated reviewer-safely: **the blind-spot derivation predicts τ² scaling, and the data are consistent with that prediction, though the exponent is not uniquely identified by the available experiments.** What is firmly established: both authority and latency matter strongly, and their *product* (not either alone) collapses the data.

### 3.2 Experimental confirmation: continuous regression (n = 262)

Outcome: continuous over-robustness SR (never binarized). Two stratified sampling passes plus a high-authority extension; finer joint gain search and 15 fresh evaluation seeds per design.

| θ̈_max bin (rad/s²) | n | Mean SR | % below 0.80 |
|---|---|---|---|
| < 40 | 103 | 0.980 | 1.9% |
| 40–80 | 36 | 0.985 | 0.0% |
| 80–120 | 24 | 0.953 | 4.2% |
| 120–180 | 23 | 0.875 | **26.1%** |
| 180–300 | 24 | 0.806 | **41.7%** |
| > 300 | 52 | 0.772 | **46.2%** (plateau) |

Pooled OLS (n = 262): 5-fold CV **R² = 0.33 ± 0.09, p = 5.8×10⁻¹¹**. Above td ≈ 300, latency — not authority — is the binding constraint (r(log td, SR) = +0.067, p = 0.68 in the extension range). The R² is modest because over-SR is right-censored at 1.0 for wide-window designs. Direct window_ratio measurement (32-point Kp sweep, n = 82 non-censored) achieves CV R² = **0.616** — confirming the censoring artifact accounts for most of the gap.

### 3.3 Controlled single-airframe experiment

Same rocket; same motor, gimbal, slew, latency, wind seed; only Iyy varied across 18 log-spaced values. Kp swept to 1,280 (no censoring). Result in the high-authority zone (td = 93–189 rad/s²):

| Iyy (kg·m²) | td (rad/s²) | Gain window |
|---|---|---|
| 0.005 | 189 | 33× |
| 0.007 | 158 | 39× |
| 0.010 | 133 | 57× |
| 0.015 | 111 | 82× |
| 0.020 | 93 | 142× |

Measured floor rises 3.6× while ceiling drops 1.4× — but this decomposition was made with a fixed Kd = 0.044 s. ⚠️ Under an existential Kd search the floor would be lower; the ceiling-compression story would dominate. The qualitative finding (window narrows with Iyy) is real; the floor/ceiling attribution is Kd-protocol-dependent. This eliminates confounding from different designs — all measurements share one airframe.

### 3.4 Interaction model (CV R² = 0.671)

The latency exponent is not fixed at −1.88; it grows with authority:

| keff tier | keff (s⁻²) | Latency exponent | Window per latency doubling |
|---|---|---|---|
| Low | ≈ 230 | −0.85 | 1.8× |
| Mid | ≈ 640 | −2.23 | 4.7× |
| High | ≈ 1,400 | −3.19 | 9.1× |

A continuous interaction model captures this: log(window) = C + a·log(k_eff) + b·log(k_eff)·log(τ), giving **CV R² = 0.671** vs 0.602 for the baseline (ΔCV = +0.069). Formula: window ≈ K / k_eff^(0.20 + 0.83·log(τ)). For k_eff ≈ 690 s⁻² (k_eff_CU ≈ 15), upgrading from 6-step to 2-step latency (3× reduction) widens the window by ≈ 12× (k_eff_CU^0.91 = 15^0.91 ≈ 12) — not the ~2× ceiling-only average. Note: formula coefficients were fit on k_eff_CU; convert via k_eff_CU = k_eff_phys × 0.022.

---

## 4. Controller Invariance

The Π constraint is specific to **reactive** saturation. Full-physics audit (2026-06-24, n = 50) resolved the open question:

| Architecture | frac_pass metric | ρ(Π, frac_pass) | p-value | Class |
|---|---|---|---|---|
| PID | log(window_ratio) | ≈ −0.78 | — | Reactive — constrained |
| LQR (40 Q/R values) | n_pass / 40 | −0.747 | 4.8×10⁻¹⁰ | Reactive — constrained |
| SMC (120 combos) | n_pass_best / 20 | −0.789 | 9.6×10⁻¹² | Reactive — constrained |
| MPC H=1 (saturated clip) | frac_pass / 12 | −0.807 | 1.5×10⁻¹² | Reactive — constrained (worse than PD) |
| **MPC H=5 (5-step QP)** | frac_pass / 12 | **−0.052** | **0.718** | **Non-reactive — escapes Π** |
| ADRC (ESO) | frac_pass / 12 | ~0 | — | **Non-reactive — escapes Π** |

Reactive controllers (PID, LQR, SMC, H=1) all face ρ ≈ −0.75. The common thread: observe current state → apply proportional/integral command → encounter saturation as an unplanned consequence. Both non-reactive approaches escape it: ADRC's ESO cancels disturbances *upstream* of saturation; H=5 MPC plans bounded command sequences *ahead*, preventing saturation proactively. **The principle: any mechanism preventing reactive saturation escapes Π.** Classical integral PID (Ki ≠ 0, n = 50): ρ drops to −0.488, marginal. At extreme Π (R2080), PID-I has zero valid (Kp, Ki) combinations; ADRC finds exactly one valid ωc with SR = 1.00. Integral action is insufficient; an ESO (or H≥5 planning) is required. ADRC remains the practical choice: 50/50 designs fully solved at all Kp tested; H=5 MPC achieves 41/50 at 5× computational cost.

**Why reactive saturation is the bottleneck** (2×2 saturation factorial, n = 15):

| Condition | SR (low Π) | SR (Π_td = 6.0) | SR (Π_td = 14.5) | slew_frac |
|---|---|---|---|---|
| PID + saturation | 1.000 | 0.867 | 0.600 | 0–71% |
| PID − saturation | 1.000 | **1.000** | **1.000** | 0% |
| ADRC + saturation | 1.000 | 0.867 | 0.533 | **0.000** |

**PID without slew saturation achieves SR = 1.000 at every Π tested, including Π_td = 14.5 (Π_keff ≈ 1.4).** Saturation is both necessary and sufficient for PID failure. **This is the project's most robust result and it replicates at scale: the larger regime-map sample (n = 142, §3) reproduces it — mean SR_nosat ≈ 0.99, only 2/142 below 0.90.** ADRC's ESO estimates the wind disturbance from the state trajectory and cancels it upstream of the control output, before the servo limit — so the servo command never reaches saturation (slew_frac = 0.000 for all 15 designs). ρ(Π, slew_frac_pid) = +0.875, p = 2×10⁻⁵.

**Mechanistic closure — what drives the transition (rate-filter ablation, full paper §4.4.6).** Replacing the FIFO delay with an EMA filter of *equal total lag* (firstlag model, n = 10) drops slew saturation to fsat = 0.000 at every Π tested, including Π = 1.38. Same delay, smoothed transients → no saturation. If delay duration alone drove the transition (the linear DIPDT phase-lag picture), a filter preserving total lag should not help. That it does identifies the cause as the unsmoothed, high-amplitude derivative-channel transients produced by aerodynamic disturbances under latency — not phase lag per se. Predicting that attenuating transient amplitude eliminates the phenomenon, then confirming it by controlled ablation, is mechanism identification rather than correlation. Together with R0522 (no transition from authority alone), this pins the driver to the product keff × τ².

Note: Carlson (2025, arXiv:2501.11374) proves bandwidth-tuned ADRC is mathematically equivalent to a 2-DOF PID with measurement filter in the linear regime. The saturation test confirms this: without saturation, both achieve SR = 1.000 identically. The divergence is purely in the nonlinear (saturated) regime — "two disturbance-handling conventions that diverge under saturation."

**Cross-system generalization (§4.0.5):** ρ(log Π, peak SR) is negative across three second-order systems: TVC (ρ = −0.668), quadrotor roll (ρ = −0.937, n = 50, p = 1.72×10⁻²³), and inverted pendulum with gravity destabilizer (ρ = −0.647, n = 25, p = 4.75×10⁻⁴). The direction — higher Π, lower achievable SR — is consistent across all three tested systems. Π_crit and the saturation mechanism are TVC-specific. Data: `quad_gen_extended_py.csv`, `gen_pendulum_py.csv`.

**Design rule** (Π = k_eff × τ²):

| Π | Recommendation |
|---|---|
| < 0.34 | PID with any reasonable tuning (SR = 1.000 for all tested designs) |
| 0.34–1.03 | Windy simulator for PID tuning; ADRC (ω₀/ωc = 5) safe for all tested designs in this range |
| > 1.03 | ADRC with ω₀/ωc ≥ 8; increase ratio as Π grows |

---

## 5. Binary Classification Audit (Secondary)

The central finding is the saturation regime transition (thesis, §3–4); the continuous regression is its dose-response quantification. Binary classification is kept as a simplified decision rule and audit trail, not a headline.

The initial 3-seed test (SR ∈ {0, 1/3, 2/3, 1}; threshold 0.80 between 2/3 and 1.0) introduced classification noise. Two correction passes:

**Audit 1** (15-seed re-evaluation): 43/45 original FRAGILE designs were one seed-flip from reclassification. AUC 0.944 → 0.957.

**Audit 2** (finer 18×7 joint search + 30 seeds): proof case R0475 was labeled INFEASIBLE with suboptimal gains (SR ≈ 0.40); finer search found SR = 0.90. AUC 0.957 → **0.975**; Cohen's d 1.74 → **3.71** (p = 3.6×10⁻¹³). Each correction strengthened the signal.

**Final population:** EASY = 2,362, FRAGILE = 36, INFEASIBLE = 2, n = 2,400.

θ̈_max computed from hardware specs (no training) achieves CV AUC = 0.991 — tying or beating all trained ML models. Dropping Iyy from the formula collapses CV AUC by 0.346. The formula is not circular: it is derived from Newton's second law, not from the classification labels.

---

## 6. Practical Implications

### 6.1 Why disturbance-free simulators cause calibration failures

| Design class | False approval (sim GO, real uncontrollable) | Calibration failure (sim gain fails in real flight) |
|---|---|---|
| Wide-window (n = 2,362) | 0.0% | 10.1% |
| Narrow-window (n = 36) | 0.0% | **63.9%** |
| Uncontrollable (n = 2) | **100.0%** | 0.0% |

The 2 genuinely uncontrollable designs show 100% false approval from disturbance-free simulators (SR ≈ 0.67 with no wind vs SR ≤ 0.30 achievable at any gain in real wind). This reverses the earlier "simple models are never dangerous" conclusion and is the strongest argument for including wind in any pre-flight simulation.

**Π predicts when calibration fails (n = 2,400):**

| Π | n | Calibration failure rate |
|---|---|---|
| < 0.34 | 2,320 | 9.1% (baseline) |
| ≥ 0.34 | 80 | **61.3%** (6.7× jump) |

50% calibration failure threshold: Π ≈ 0.41. Within overtuned designs, severity (kp_simple/ceiling) rises from 1.8× to 4.2× as Π increases — both probability and consequence of overtuning rise together. **Practical rule:** if Π > 0.41 and kp_simple > 0.042/τ, the autotune has overshot the real window. Cap at 0.042/τ, use a windy+latency simulator, or switch to ADRC. Data: `pi_s2r_gap_summary_py.csv`.

### 6.2 Single test flight detects narrow-window designs

Fly at Kp = 0.044 [rad/rad]; measure RMS attitude error. Results (final population, n = 36 + 36, 7 seeds each): AUC = **0.954** [0.907, 0.989]; narrow-window mean RMS = 13.3° ± 5.2°; wide-window = 3.8° ± 2.7°. Threshold RMS > **7.6°** (F1 = 0.89). AUC (1-seed) = 0.921 — a single flight suffices for screening.

### 6.3 Builder formulas

```
keff      = T_avg × L_nozzle / Iyy    [s⁻²; no gimbal needed]
τ         = latency_steps / loop_hz   [s; total loop delay]
Π         = keff × τ²                 [-; dimensionless risk parameter]
Kp_ceil   ≈ 0.042 / τ                [rad/rad; ceiling, keff-independent (coeff ≈ 0 in regression)]
           ≈ 380 / latency_steps      [simulator native units; same ceiling]
Kp_start  ≈ 0.025 / τ               [rad/rad; Ziegler-Nichols PD start; requires only τ, not keff]
Kd_start   = 0.012 s                 [constant across designs; latency- and keff-independent; validated n=20, SR≥0.80 for Π<0.99]
```

⚠️ Floor formula removed — Kp_floor ≈ 0.06 × keff × latency was a Kd=1.0 artifact and should not be cited.

Risk tiers:
- **Π < 0.34**: disturbance-free autotune OK (9.1% calibration failure rate)
- **Π 0.34–1.0**: Kp_simple likely > ceiling → cap at 0.042/τ; use ADRC for clean solution
- **Π > 1.0**: saturation-dominated ceiling; ADRC required (ω₀/ωc = 5–20)

`tools/gain_advisor.py` computes keff, Pi, Kp_ceiling, and ADRC ωc ceiling from hardware specs.

---

## 7. Discussion and Limitations

**What is robust:** The ranking (higher Π → narrower window) survived four independent attempts to disprove it — two classification audits, ML comparison, continuous re-derivation — and was strengthened by each. Wind magnitude, servo speed, and aerodynamic instability do not predict gain-window width after authority and latency are considered.

**What is environment-specific:** The 54.8 rad/s² Youden-J threshold and the 7.6° flight-detection threshold depend on the specific wind distribution and success criterion in these simulations. Rankings are robust; thresholds are not universal constants.

**What is exploratory:** ADRC's adaptive ceiling law (n = 46 cells; exponent uncertainty). The combined window-ratio formula was validated against held-out data and **failed** (AUC = 0.500; floor alone R² = −0.003) — the ceiling formula retains support; the combined formula does not.

**What is retired:** Floor formula Kp_floor ≈ 0.06 × keff × latency — confirmed to be a Kd=1.0 protocol artifact, not physics. "Double squeeze" (3.7× per latency doubling) — wrong. Use ~2× (ceiling-only).

**The largest self-correction:** the floor law above was a *headline result*, with a held-out R² = 0.71 and a "3.7× per latency doubling" claim. The project's own follow-ups retired it (Kd-artifact → existential Kd search collapsing floors to near-zero → mediation/minimal-physics tests). The transition survived the retirement because Π's second power of τ comes from the blind-spot kinematics (§3.1), not the floor.

**What is unvalidated:** Everything is simulation-only. Hardware validation required for the floor/ceiling formula coefficients, flight-detection threshold, and Youden-J boundary.

**Confidence tiering (decreasing):**
1. **Saturation causes the failure** — PID-nosat = 1.000 (n = 15 factorial) *and* SR_nosat ≈ 0.99 (n = 142 regime map); necessary and sufficient. The most robust, most replicated result; the one least likely to move under more data. **This is the headline.**
2. The ranking result: authority/θ̈_max outperforms every alternative for the binary label (AUC = 0.975) — strongly supported
3. Iyy cannot be replaced by mass: drop-one ΔCV AUC = −0.346 — strongly supported
4. Ceiling-dominated window (floor near-zero under optimal Kd) — strong; floor formula retired as artifact
5. Reactive-controller invariance (PID/LQR/SMC/H=1): ρ ≈ −0.75 each; H=5 MPC escapes (ρ = −0.052, full-physics confirmed) — strong (simulation only)
6. The transition *correlation magnitude* (ρ = 0.55, n = 142) and the Π-vs-keff separation — moderate; on the main population (latency 1–6) Π ties keff, so the product-specific claim rests on R0522 + the lat-1–12 data + theory
7. Simulator absolute correctness — weakest; hardware unvalidated

---

## 8. Hardware Validation Plan

Highest-value experiments (in order):

1. **Matched-configuration test:** vary only Iyy via ballast redistribution on one rocket; same motor, gimbal, MCU, wind conditions. Measure Kp window at each configuration. Confirms §3.3 with real hardware without requiring multiple builds.
2. **MCU swap:** same rocket, same gains, Arduino vs Teensy. Predicts and confirms latency-driven window collapse without mechanical change.
3. **Kp = 0.044 [rad/rad] detection:** fly narrow-window design; expect RMS > 6.0°. Then re-tune within [Kp_floor, Kp_ceiling] and fly again. Confirms §6.2 and §6.3 in one experiment.
4. **Rate-filter ablation (cheapest; firmware-only; most direct mechanism test):** one high-Π design (Π > 0.4), one fixed Kp, toggle only the rate path — raw gyro vs. smoothed estimate (onboard complementary/Madgwick or software EMA, α ≈ 1/(L+1)). §4.4.6 predicts visible servo chatter in the raw condition, substantially reduced when smoothed. If smoothing does **not** help, the mechanism claim (step 6) is wrong. Directly tests "transients, not delay duration, drive the transition" with no mechanical change.

---

## Summary of Novel Contributions

| Contribution | Evidence |
|---|---|
| **Saturation regime transition organized by Π = k_eff × τ² (central contribution)**: fsat rises monotonically with Π (clean binned dose-response; ρ = 0.55 on n = 142, replicated from an optimistic n = 29 ρ = 0.80); onset band Π ≈ 0.2–0.4; the classical linear PID/ADRC equivalence breaks down above it. On the lat-1–6 population Π ≈ k_eff in rank terms; product-specific evidence is R0522 + lat-1–12 data + theory | **Three independent legs: structural diagnostic (Figure 1, n = 142), causal saturation-removal (SR_nosat ≈ 0.99, n = 142), ADRC-advantage onset alignment; robust to wind/servo/plant-structure/probe-gain** |
| Ceiling-dominated window: Kp_ceiling ≈ 0.042/τ (keff-independent); floor near-zero | DIPDT theory + 5 spot-checks; floor formula retired as Kd=1.0 artifact |
| **Π predicts calibration failure: FR 9.1% → 61.3% (6.7×) at Π = 0.34; threshold Π ≈ 0.41** | **n = 2,400, no new simulations** |
| Π = k_eff × τ² as saturation-regime risk parameter | Slope −0.971 vs theory −1.000; f_sat adds zero after Π |
| Interaction model: window ∝ 1/k_eff^(0.20+0.83·log(τ)) | CV R² = 0.671; explains keff-tier latency sensitivity |
| Reactive-controller invariance: PID/LQR/SMC/H=1 all ρ ≈ −0.75; H=5 MPC ρ = −0.052 escapes | Four independent tests, n = 50 each; full-physics audit 2026-06-24 |
| **Cross-system generalization: ρ(log Π, SR) < 0 for TVC (−0.668), quadrotor (−0.937), pendulum (−0.647)** | Three systems, independent physics; direction consistent across all three; Π_crit TVC-specific |
| Saturation: necessary AND sufficient for PID failure | 2×2 factorial, n = 15; PID-nosat = 1.000 everywhere |
| Zero-calibration tuning: Kp = 0.025/τ, Kd = 0.012 s; requires only τ | n = 20, 100% SR ≥ 0.80 at Π < 0.99; beats stored optimal on extreme case |
| Matched single-airframe: window 142× → 33× from Iyy alone | No confound; cleanest causal test (floor/ceiling decomp. is Kd-dependent) |
| Binary AUC = 0.975, d = 3.71 (twice-corrected; each pass strengthened) | Secondary decision rule; formula beats trained ML |
| Disturbance-free simulator: 100% false approval for uncontrollable designs | n = 2; reverses prior claim |
| Flight detection AUC = 0.954, threshold 7.6° | Simulation only; hardware needed |

---

## Appendix: Key Data Files

| File | Contents |
|---|---|
| `exp1_final_population_py.csv` | n = 2,400 final classification |
| `regression_pooled_py.csv` | Continuous regression n = 262 |
| `pi_s2r_gap_summary_py.csv`, `pi_s2r_gap_py.csv` | Π as calibration failure predictor (§6.1) |
| `window_ratio_v2_py.csv` | Window_ratio regression n = 116 (non-censored: 82) |
| `performance_frontier_py.csv` | PID/ADRC frontier n = 63 |
| `adrc_saturation_test_py.csv` | 2×2 saturation factorial n = 15 |
| `lqr_controller_test_py.csv` | LQR invariance n = 50 |
| `smc_controller_test_py.csv` | SMC invariance n = 50 |
| `mpc_controller_test_py.csv`, `mpc_rho_summary_py.csv` | MPC simplified physics: H=1 ρ=−0.969, H=5 frac_pass=1.000 (ceiling-only) |
| `mpc_full_physics_audit_rho_py.csv`, `mpc_full_physics_audit_py.csv` | MPC full-physics audit 2026-06-24: PD ρ=−0.701, H=1 ρ=−0.807, H=5 ρ=−0.052 |
| `flight_sig_final_py.csv` | Flight detection n = 36+36 |
| `matched_config_extended_kp_py.csv` | Single-airframe Kp sweep |
| `tools/gain_advisor.py` | Builder tool: Kp range from hardware specs |
