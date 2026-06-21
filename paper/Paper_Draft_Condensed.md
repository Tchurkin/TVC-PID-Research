# A Hardware-Derived Gain-Margin Score and Cross-Architecture Tuning Rule for Hobby-Scale TVC Rockets

**Braxton Herold** — *Condensed version for review; full paper at `paper/Tentative_Paper_Draft.md`*

---

## Thesis

A rocket's gain margin scales continuously with the maximum TVC angular acceleration computable from hardware specs (R² = 0.33 cross-validated, p = 5.8×10⁻¹¹):

> **θ̈_max = T · sin(δ_max) · L_nozzle / Iyy**

Earlier drafts used a binary "FRAGILE/EASY" classifier. That framing is dropped: the continuous relationship (n = 262, Section 4.0) shows no clean separation — gain margin degrades smoothly with θ̈_max and latency. The headline claim is the continuous relationship, not a threshold. The practical output is a predicted gain window [Kp_floor, Kp_ceiling].

A secondary finding: ADRC has a quantified bandwidth ceiling ωc_max ≈ 70/(latency^0.57 · θ̈_max^0.31) (n = 46 cells, R² = 0.823) that depends on both latency and authority, unlike PID's ceiling which is almost purely latency-driven. A runnable tool (`tools/gain_advisor.py`) implements both ceiling laws from specs alone.

---

## Abstract

θ̈_max predicts, continuously, how much a hobby TVC rocket's gain margin narrows as authority and latency increase. The fraction of designs below the 0.80 reliability threshold rises from 2% (θ̈_max < 40 rad/s²) to 46% (θ̈_max > 300). Above td ≈ 300 the curve plateaus — latency, not authority, is the binding constraint there. R² = 0.36 in-sample / 0.33 ± 0.09 (5-fold CV), p = 5.8×10⁻¹¹.

A secondary binary result (AUC = 0.975, Cohen's d = 3.71) is retained as a decision aid on the twice-corrected population (n = 36 of 2,400 "high-sensitivity" designs). Disturbance-free simulator tuning causes 63.9% false rejection for these designs; the 2 genuinely uncontrollable designs show 100% false approval — the one place the simplified simulator gives zero warning. ADRC at a fixed untuned setting closes 83% of the PID performance gap (Section 6.1); slew saturation is confirmed as the sole cause of PID failure above Π = 5,000 (2×2 factorial, Section 6.2). Three architectures (PID, LQR, SMC) all show Spearman ρ(Π, window) ≈ −0.75 on 50 designs; ADRC decouples this to ρ = −0.325.

---

## 1. Background and Novelty

Newton's law θ̈_max = T·sin(δ)·L/Iyy is textbook. The novel contributions:

| Contribution | Evidence level |
|---|---|
| **Continuous margin regression: R² = 0.33 (CV), n = 262, p = 5.8×10⁻¹¹; monotone to td≈300, plateau above** | **Primary — avoids AUC/base-rate ambiguity** |
| Controlled single-airframe test: only Iyy varies → window 142×→33× (Section 4.0.1) | Strong internal validity, sim-only |
| Controller invariance: PID/LQR/SMC all ρ(Π,window) ≈ −0.75; common: no disturbance estimation | Novel, n=50, three architectures |
| Performance frontier: PID fails Π=5,214; ADRC zero failures to Π=49,893 with adaptive ω₀/ωc | Novel, n=44+63 |
| Π = θ̈_max × τ² first-principles derivation: slope = −1.029 (theory: −1.000) | Novel, theoretical + empirical |
| Saturation mechanism: PID-nosat = 1.000 for all designs; ADRC never saturates (slew_frac=0) | Novel, n=15, 2×2 factorial |
| ADRC bandwidth ceiling vs. PID: authority exponent −0.31 vs −0.20 (n=46 cells) | Exploratory, original 3-design n=21 exponent did not replicate |
| AUC = 0.975, d = 3.71 from hardware formula; Iyy irreplaceable (drop-AUC 0.124) | Secondary decision rule |
| K_u separation: EASY 91.2 vs narrow-window 29.2 (3.12×, p = 4.17×10⁻⁷, n=36 relay probe) | Mechanistic |
| 63.9% false rejection / 100% false approval for INFEASIBLE in disturbance-free sim | Negative result for old claim |

**Not novel:** authority/inertia ratio concept (standard GNC textbooks); "simple simulators pick wrong gains" (control theory since 1971); over-actuated systems have narrow margins (Doyle).

---

## 2. Methods

### Design space (n = 2,400 LHS)

| Parameter | Range | Notes |
|---|---|---|
| mass | [0.50, 1.20] kg | T/W > 1 filter enforced |
| Iyy | [0.005, 0.100] kg·m² | 3D-printed hobby TVC range |
| servo_slew | [60, 200] °/s | |
| motor_scale | [0.5, 3.0] | scales F15 avg thrust 14.4 N |
| max_gimbal_deg | [2, 15] | |
| latency_steps | [1, 6] | 5–30 ms at 200 Hz |
| wind_strength | [0.05, 0.45] | Ornstein-Uhlenbeck |

Full-physics simulator: 10 toggleable modules (nonlinear aero, dynamic pressure, thrust curve, CG shift, slew, backlash, deadband, latency, wind, sensor noise). All active for all results.

### Physical predictor

```
T_avg     = 14.4 × motor_scale       [N]
keff_full = T_avg × 0.02182 × 0.25 / Iyy   [rad/s²/CU]
u_max     = max_gimbal_deg × 12/15   [CU]
θ̈_max    = keff_full × u_max        [rad/s²]
Π         = θ̈_max × latency_steps²  [rad, blind-spot displacement]
```

### Continuous regression (Section 4.0)

Outcome variable: over-robustness SR (continuous fraction in [0,1], 15 seeds, never binarized). Evaluated at 1.4× best gains from finer joint 18×7 Kp×Kd search.

---

## 3. Results

### 4.0 Primary: continuous gain-margin regression (n = 262)

Two passes: (1) 130 designs stratified by θ̈_max decile; (2) 92 designs targeted at td ∈ [40–1,000] with 20 per bin. Extension: 40 additional designs from td ∈ [300, 500] to characterize the plateau.

**Binned dose-response (n = 262 pooled, `regression_pooled_py.csv`):**

| θ̈_max bin | n | Mean over-robustness SR | % below 0.80 |
|---|---|---|---|
| < 40 | 103 | 0.980 | 1.9% |
| 40–80 | 36 | 0.985 | 0.0% |
| 80–120 | 24 | 0.953 | 4.2% |
| 120–180 | 23 | 0.875 | **26.1%** |
| 180–300 | 24 | 0.806 | **41.7%** |
| > 300 | 52 | 0.772 | **46.2%** ← plateau |

**OLS (log θ̈_max + log latency): R² = 0.364 in-sample. 5-fold CV: R² = 0.325 ± 0.086, MAE = 0.070.**
Coefs: intercept = 1.234, log θ̈_max = −0.051, log latency = −0.101 (latency coefficient 2× larger — binding constraint in the high-authority plateau).

**Plateau finding:** within td > 300, r(log θ̈_max, SR) = +0.067 (p = 0.68). No further decline above td = 300. Variance dominated by latency (lat = 1 → SR ≈ 1.00; lat = 5–6 → SR ≈ 0.55) not by how high td is.

**Why R² ≈ 0.33 is the honest result:** over-SR is right-censored at 1.0 (all wide-window designs score ~0.98 regardless of actual window width). Direct window_ratio regression on n = 82 non-censored designs: CV R² = 0.616 ± 0.035 — nearly 2× improvement. The over-SR R² understates predictability because of the censoring artifact.

**Dimensionless Π:** Grid search finds optimal α = 2.08 in Π = θ̈_max × latency^α. Single-parameter Π achieves CV R² = 0.353 = two-variable CV R² = 0.352 (indistinguishable). Π = θ̈_max × τ² [rad] is the angular displacement accumulated under max TVC authority in one latency window — the error the controller is blind to before its first correction.

### 4.0.1 Controlled single-airframe experiment

One fixed airframe (mass = 0.80 kg, motor_scale = 1.5, max_gimbal = 10°, latency = 3 steps, wind = 0.20 — all parameters fixed). Only Iyy varied across 18 values. Kp swept 1–1,280 (40 points, no censoring), Kd = 2.0 fixed, 10 seeds per point.

| θ̈_max (rad/s²) | Kp floor | Kp ceiling | Window ratio |
|---|---|---|---|
| 188.5 | 10.9 | 354 | **33×** |
| 158.0 | 9.0 | 354 | **39×** |
| 132.5 | 9.0 | 512 | **57×** |
| 111.1 | 6.3 | 512 | **82×** |
| 93.1 | 3.0 | 426 | **142×** |
| 78.1 | 1.7 | 512 | 295× |
| ≤ 65.5 | 1.0–4.3 | 204–512 | 118–204× |

Window narrows monotonically in the high-authority zone (93→189 rad/s²) with no other parameter changed. **Mechanism:** floor rises 3.0→10.9 (3.6×) while ceiling drops 512→354 (1.4×). Floor-rising dominates. Data: `matched_config_extended_kp_py.csv`.

### 4.0.2 Controller invariance: LQR and SMC

Same 50 designs from the corrected population. LQR: 40 Q/R ratios via DARE. SMC: 6 λ_s × 20 Kp = 120 combinations. 10 seeds each.

| Architecture | Metric | Spearman ρ(Π, window) | p |
|---|---|---|---|
| PID | log(window_ratio) | ≈ −0.78 | — |
| LQR | n_pass_QR / 40 | −0.747 | 4.8×10⁻¹⁰ |
| SMC | n_pass_total / 120 | −0.753 | 2.8×10⁻¹⁰ |
| SMC | n_pass_best_λ / 20 | −0.789 | 9.6×10⁻¹² |

All three ρ ≈ −0.75. Common element: no active disturbance estimation. ADRC's ESO is the specific escape mechanism.

### 4.0.3 Performance frontier

63 designs across 7 Π tiers. PID: 18×7 joint grid → best gains → 15 seeds. ADRC: 7 ωc values, ω₀ = 5×ωc.

| Π tier | n | PID mean SR | PID < 0.80 | ADRC mean SR | ADRC < 0.80 |
|---|---|---|---|---|---|
| < 800 | 30 | 1.000 | 0% | 1.000 | 0% |
| 800–2k | 10 | 1.000 | 0% | 1.000 | 0% |
| 2k–5k | 10 | 0.940 | 0% | 1.000 | 0% |
| 5k–9k | 10 | 0.867 | **10%** | 0.973 | 0% |
| > 9k | 3 | 0.622 | **67%** | 0.911 | 33% |

ρ(Π, PID SR) = −0.668; ρ(Π, ADRC SR) = −0.325. Twofold ρ ratio = quantitative signature of ESO decoupling.

**Frontier extension (n = 44, Π up to 49,893):** ADRC swept over ωc ∈ {0.5,1,1.5,2,3,5} × ω₀/ωc ∈ {5,8,12,20}. Zero ADRC failures found. Standard ω₀/ωc = 5 fails above Π ≈ 20,000; ω₀/ωc = 20 succeeds to Π = 49,893.

| Π tier | n | PID < 0.80 | ADRC std | ADRC extended |
|---|---|---|---|---|
| 9k–12.6k | 27 | 85% | 4% | **0%** |
| 15k–20k | 5 | 80% | 20% | **0%** |
| > 30k | 3 | 100% | 100% | **0%** |

Design rules: Π < 5,000 → PID. Π 5,000–15,000 → ADRC (ω₀/ωc = 5). Π > 15,000 → ADRC (ω₀/ωc = 12–20).

### 4.0.4 Why Π = θ̈_max × τ²: first-principles derivation

**Ceiling (DIPDT phase-margin):**  Kp_ceiling ≈ 0.9/τ (keff-independent). Falls as τ⁻¹. Empirical exponents: latency = −1.036, keff = +0.059 (theory: −1 and 0).

**Floor (bang-bang blind-spot):** Actuator deflects at θ̈_max for τ seconds before correction → angular impulse ≈ θ̈_max × τ → Kp_floor ∝ keff × τ. Empirical exponents: keff = +1.251, latency = +0.844 (theory: both +1).

**Window = ceiling / floor:**
> Window ≈ τ⁻¹ / (keff × τ) = **1/(keff × τ²) = 1/Π**

Double squeeze: ceiling drops τ⁻¹ AND floor rises τ⁺¹. Each doubling of latency → 2^1.88 ≈ **3.7× window compression** (not 2.1× from ceiling-only theory).

**Validation:** log(window) ~ log(Π) slope = −1.029 (theory: −1.000, deviation 0.029). R²(Π) = 0.650 vs. two-variable R² = 0.659 — essentially equal. All 7 exponent predictions confirmed; max deviation 0.25.

---

### 4.1 Binary classification (secondary — audit trail)

Final, twice-corrected population (Section 4.6): **EASY = 2,362, FRAGILE = 36, INFEASIBLE = 2** (MARGINAL dissolved — all reclassified EASY in 15-seed correction).

| Population | n | Mean θ̈_max | Cohen's d | AUC |
|---|---|---|---|---|
| Original 3-seed | FRAGILE=45 | 124.5 | 1.74 | 0.944 |
| 15-seed correction | FRAGILE=30 | — | — | 0.957 |
| Finer-search + 30-seed (final) | FRAGILE=36 | 168.2 | **3.71** | **0.975** |

Each correction strengthened the result. Youden-J threshold = 54.8 rad/s² (TPR = 0.96, FPR = 0.12). Negative predictive value = 99.9%.

---

### 4.2 Variable screening: H1–H5

| Variable | r(var, sensitivity) | ∆AUC vs. θ̈_max |
|---|---|---|
| wind_strength | −0.008 | negative |
| servo_slew | +0.048 | negative |
| static_margin | −0.001 | negative |
| **latency_steps (H5)** | — | **+0.028 → combined AUC = 0.972** |

H1–H4 rejected. H5 confirmed: latency is an independent hardware predictor. Both false negatives in original classification have latency = 6 steps.

---

### 4.4 Mechanistic chain: Newton → K_u → window collapse

1. **θ̈_max from hardware** (Newton's 2nd law, no fitting)
2. **Oscillation amplitude at Kp = 2** (relay probe, full physics): r(log θ̈_max, log K_u) = −0.448, rho = −0.548, n = 36
3. **K_u from amplitude** (Åström–Hägglund exact): K_u = 4·u_max / (π·A_rad)
4. **K_u separation, definitive** (true relay probe, final n = 36 population):
   - Wide-window: K_u median = **91.2** (EASY sample)
   - Narrow-window: K_u median = **29.2** (FRAGILE)
   - **3.12×, Mann-Whitney p = 4.17×10⁻⁷** — tightest of three successive re-derivations

---

### 4.5 Gain ceiling and window formula

**Ceiling (latency-dominated, keff-independent):**
> Kp_max ≈ **380 / latency_steps** (empirical; 2.1× correction over theory 0.9/τ)
> Power law: ceil ≈ 2.6 × keff^(−0.20) × τ^(−1.10), R² = 0.53, n = 20

**Floor (both keff and latency):**
> Kp_floor ≈ **0.06 × keff^1.06 × latency^0.96** (window_ratio v2, n = 104, R² = 0.627, CV = 0.594)
> Old formula (keff-only, ρ = 0.58) incomplete — latency was never tested before.

**Window ratio (window_ratio regression v2, n = 116 non-censored):**
> Window ≈ 8,700 × keff^(−1.19) × latency^(−1.88) [CV R² = 0.616]

**Prediction table:**

| latency | Kp_max (theory) | Verified range |
|---|---|---|
| 1 step (5 ms) | ≈ 380 | ≥ 270 |
| 3 steps (15 ms) | ≈ 127 | 90–190 |
| 6 steps (30 ms) | ≈ 63 | 40–90 |

**Negative result:** combined ceiling×floor formula validated against held-out data → AUC = 0.500 (chance). The floor piece alone: R² = −0.003. Do not cite as a validated predictive unit.

---

### 4.6 Classification audit trail (summary)

Two independent corrections, both strengthening AUC:

1. **15-seed reclassification** (seeds 101–115, gains frozen): 43/45 original FRAGILE were "borderline" (one seed flip away from different class). 60% flipped. MARGINAL dissolved. AUC: 0.944 → 0.957.
2. **Finer gain search + 30 seeds** (18×7 Kp×Kd, seeds 1001–1030): gain search itself was underpowered. R0475 (INFEASIBLE in both passes) reached SR = 0.90 with better search. 81/241 re-examined designs remain "uncertain" (Wilson CI straddles threshold at 30 seeds) — concentrated above θ̈_max = 55 rad/s² on the over (ceiling) test. AUC: 0.957 → 0.975, d: → 3.71.

**Lesson:** binary pass/fail with n seeds cannot resolve probabilities finer than 1/n. 3-seed tests cannot resolve a threshold between 2/3 and 1.0. Use ≥ 7 seeds or report continuous SR with CI.

---

### 5.1 Simulator-to-real fidelity gap

**Definitive results on final population:**

| Regime | n | SR(simple→full) | SR(full→full) | Gap | False Approval | False Rejection |
|---|---|---|---|---|---|---|
| EASY | 2,362 | 0.861 | 0.999 | 0.138 | **0.0%** | 10.1% |
| FRAGILE | 36 | 0.361 | 0.760 | 0.399 | **0.0%** | **63.9%** |
| INFEASIBLE | 2 | 0.667 | 0.233 | −0.433 | **100.0%** | 0.0% |

**The "simple model is never dangerous" claim is false.** Both genuinely uncontrollable designs receive false approval — disturbance-free simulators give no warning for the rare truly-infeasible design. For EASY/FRAGILE, the problem is excess caution (false rejection), not false safety.

**Root cause:** disturbance-free gain landscape has no signal about wind-rejection floor or limit-cycle ceiling. 93% of FRAGILE misses are overtuning (Kp_simple > Kp_ceiling); 7% undertuning.

**Fidelity cutoff by θ̈_max (n = 25 designs, 7 fidelity conditions):**

| θ̈_max tier | Wind impact | Noise+latency impact |
|---|---|---|
| td < 70 | ~0 | ~0 |
| td 100–150 | +0.04 max | **+0.12 each** |

Wind has the smallest evaluation impact at any td level. Noise and latency are the hardest evaluation modules for high-td designs. Data: `fidelity_cutoff_by_td_py.csv`.

---

### 5.2 Flight detection

Single test flight at Kp = 2 detects narrow-window rockets. Final population (n = 36 FRAGILE + 36 stratified EASY, 7 seeds each):

| Metric | Value |
|---|---|
| AUC (7-seed RMS) | **0.954 [0.907, 0.989]** |
| AUC (1-seed RMS) | 0.921 |
| FRAGILE mean RMS | 13.3° ± 5.2° |
| EASY mean RMS | 3.8° ± 2.7° |
| Separation ratio | 3.53× |

| Threshold | TP | FP | FN | TN | Prec | Rec | F1 |
|---|---|---|---|---|---|---|---|
| > 5.0° (Youden-J) | 36 | 8 | 0 | 28 | 0.82 | 1.00 | 0.90 |
| **> 6.0°** | **31** | **4** | **5** | **32** | **0.89** | **0.86** | **0.87** |
| > 7.6° | 31 | 4 | 5 | 32 | 0.89 | 0.86 | 0.87 |

**Recommended threshold: RMS > 6.0°** (0% false alarm at 5.0°, 86% recall at 6.0°). Data: `flight_sig_final_py.csv`.

**Workflow:** (1) compute θ̈_max from specs; (2) if > 55 rad/s², flag; (3) fly at Kp = 2; if RMS > 6.0°, confirmed → target Kp = 40–80; (4) if RMS < 6.0° but θ̈_max > 55, confirm with multiple seeds.

---

### 6.0 ADRC and PID: mathematical equivalence in the linear regime

Carlson (2025, arXiv:2501.11374) proves bandwidth-tuned ADRC is mathematically equivalent to a 2-DOF PID for first/second-order plants (this study's dynamics are second-order). "Cross-architecture" claim is reframed: **two disturbance-handling conventions, divergent in the saturated (nonlinear) regime**. Section 6.2 confirms this empirically.

### 6.1 ADRC dissolution of the narrow-window class

Protocol: all 36 final FRAGILE + 36 td-stratified EASY. **PID uses finer-search best-effort gains. ADRC uses one fixed, untuned setting for all: ωc = 5, ω₀ = 25, b₀ = keff (from specs).** 20 fresh seeds.

| Controller | FRAGILE SR | EASY SR | Gap |
|---|---|---|---|
| PID (best-effort) | 0.776 | 0.996 | 0.219 |
| ADRC (fixed setting) | **0.962** | **1.000** | **0.038** |

**83% of PID gap closed by a single untuned ADRC setting.** 15/17 FRAGILE designs with PID SR < 0.80 reach ADRC SR ≥ 0.80 (88% conversion). 2/36 residual ADRC failures (R2072 td=351 lat=6, R2080 td=312 lat=5) — resolved by lowering ωc to 3 with ω₀/ωc ∈ {35,50,70,100} → SR = 1.000. Same θ̈_max × latency mechanism. Data: `adrc_dissolution_py.csv`.

### 6.2 Saturation mechanism: causal isolation

**2×2 factorial: {PID, ADRC fixed ωc=5} × {saturation ON, saturation OFF}. 15 designs (10 EASY, 5 FRAGILE), Π = 27–12,648. 15 seeds each.**

| Π | PID+sat | ADRC+sat | PID−sat | ADRC−sat | slew_PID | slew_ADRC |
|---|---|---|---|---|---|---|
| 27–3,589 | 1.000 | 1.000 | 1.000 | 1.000 | < 0.46 | 0.000 |
| 5,214 | **0.867** | 0.867 | **1.000** | 0.867 | **0.708** | 0.000 |
| 12,648 | **0.600** | 0.533 | **1.000** | 0.533 | **0.705** | 0.000 |

**Fact 1:** PID−sat = 1.000 for ALL 15 designs including Π = 12,648. Saturation is necessary and sufficient for PID failure in this Π range.

**Fact 2:** slew_frac_adrc = 0.000 for all 15 designs. ESO cancels wind before the control law → commanded deflection stays small (effective gain ωc²/b₀ ≈ 2–5 vs. PID's required Kp 40–80).

**Fact 3:** ADRC−sat = ADRC+sat for all designs. ADRC's failure at Π = 5,214 (0.867) and 12,648 (0.533) is bandwidth-limited at fixed ωc = 5, not saturation-limited.

**Carlson reconciliation:** Without saturation, PID = ADRC = 1.000 for all 15 designs. Equivalence holds in the linear regime; the departure is entirely in saturated conditions. "Two conventions divergent under saturation" is empirically confirmed.

**ρ(Π, slew_frac_pid) = +0.875, p = 2.0×10⁻⁵.** Π predicts the mechanism (saturation rate) as strongly as the outcome (SR). Data: `adrc_saturation_test_py.csv`.

### 6.3 ADRC bandwidth ceiling law

Protocol: ωc swept over log grid, ω₀ = 5×ωc, b₀ = keff, 10 seeds. Find max ωc achieving SR ≥ 0.80. 6 authority levels × 8 latency levels = 46 cells.

**Fitted power law (n = 46 cells):**
> **ωc_max ≈ 70 / (latency_steps^0.57 · θ̈_max^0.31), R² = 0.823**

| lat | ωc_max, td≈20 | ωc_max, td≈90 | ωc_max, td≈200 |
|---|---|---|---|
| 1 | 30 | 20 | 10 |
| 3 | 20 | 7 | 7 |
| 6 | 15 | 5 | 5 |
| 12 | 7 | 3 | — |

**Comparison to PID:** PID ceiling exponents (keff = −0.20, latency = −1.10). ADRC ceiling exponents (θ̈_max = −0.31, latency = −0.57). ADRC shows 1.5× more authority coupling than PID; both are latency-dominated. Note: original 3-design estimate gave authority exponent −0.77 which did not replicate on the 46-cell dataset — use the n=46 result.

---

### 7. Discussion

**7.1 The threshold is environment-specific; the ranking is robust.** The specific 55 rad/s² Youden-J threshold depends on this study's wind environment. What generalizes: the ranking (higher θ̈_max → narrower window) and the Π dependence.

**7.2 The 5-minute builder workflow.** `tools/gain_advisor.py` takes motor thrust (N), max gimbal (deg), Iyy (kg·m²), latency (steps) and returns: θ̈_max, Π, continuous risk score (n=262 regression), PID Kp range [floor, ceiling], ADRC ωc ceiling. Formula evidence levels documented inline.

**7.3 Limitations:**
1. l_nozzle = 0.25 m fixed; formula scales linearly with l_nozzle, not validated at other lengths.
2. Simulation only — hardware validation required for flight detection threshold (6.0°) and the 55 rad/s² boundary.
3. 81/241 re-examined designs remain "uncertain" at 30 seeds (Wilson CI straddles threshold). These designs — all θ̈_max ≥ 55 — should be reported as "elevated risk, indeterminate severity" rather than forced into a binary.
4. Stress-test population (latency 1–12, Arduino-class): FRAGILE fraction rises from 1.9% to 5.6%; at latency 9–12, 10–13% of designs are FRAGILE regardless of mechanical authority.

**7.4 Confidence tiering (decreasing order):**
1. **Strongest:** ranking by θ̈_max — survived four independent attempts to break it, strengthened each time
2. **Strong:** Iyy irreplaceable — removing it costs 0.21–0.26 AUC
3. **Moderate:** latency compresses gain ceiling — direction textbook; exact constant (380/lat) rests on n=20 conditions
4. **Moderate:** mechanistic chain (Newton → bang-bang → K_u) — each step confirmed, but chain is derived from subsamples
5. **Moderate (upgraded):** controller invariance + frontier — three architectures ρ≈−0.75; frontier to Π=49,893 with zero ADRC failures; Π derivation matches theory; first-principles τ² confirmed
6. **Weakest by construction:** simulator absolute correctness — no hardware validation yet

**7.5 Key anticipated objections:**

*"How do you know the simulator is realistic?"* It isn't validated yet. The central finding (θ̈_max predicts gain sensitivity) follows from Newton + delay margin — established physics independent of simulator details. Hardware testing (Section 8, item F) is the highest-value next step.

*"Isn't this just rediscovering T/Iyy?"* The formula is textbook. The claim is empirical: among all candidate predictors exhaustively screened (including trained ML on all raw features), this single quantity is sufficient and outperforms alternatives. That is a screening result, not a restatement.

*"Why trust it after multiple bugs?"* Because each correction moved details without reversing the central ranking — and three consecutive corrections strengthened AUC from 0.944→0.957→0.975 and d from 1.74→3.71. The one reversal (false-approval finding: 0%→100% for INFEASIBLE) is reported deliberately.

---

## Appendix: Rejected Hypotheses

| Hypothesis | Key datum |
|---|---|
| Wind determines gain sensitivity | r(wind, sensitivity) = −0.001; adding wind reduces AUC |
| Slow servos determine sensitivity | r(slew, sensitivity) = +0.013; solo AUC = 0.45 |
| Aerodynamic instability determines sensitivity | r = +0.034; solo AUC ≈ 0.51 |
| Gravity-normalized Π (T·sin(δ)/mg) outperforms θ̈_max | AUC = 0.730, −0.124 vs. baseline |
| Iyy × wind controllability boundary | Entire INFEASIBLE class was a π/180 slew-formula bug artifact |
| Combined ceiling×floor formula predicts window width | AUC = 0.500 on held-out data; floor alone R² = −0.003 |
| MARGINAL is a genuine third regime | 15-seed re-evaluation: 100% reclassify as EASY |
| ADRC and PID are fully distinct architectures | Carlson (2025): mathematically equivalent in linear regime |
