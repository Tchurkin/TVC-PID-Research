# TVC Rocket Research – Claude Context

## Project Purpose

This project investigates how physical rocket properties determine:

1. The boundary between controllable and uncontrollable TVC rockets.
2. The simulator fidelity required to make correct engineering design decisions.
3. The robustness–maneuverability tradeoff in low-cost TVC rockets.

This is NOT a PID tuning project.

The primary scientific goal is understanding design-space structure and simulator fidelity requirements.

---

# Research Philosophy

Act as a skeptical reviewer.

Do NOT optimize for exciting conclusions.

Do NOT defend previous work automatically.

Always:

* inspect data before drawing conclusions
* quantify uncertainty
* search for alternative explanations
* identify circular reasoning
* identify confounding variables
* distinguish visualization from evidence
* distinguish correlation from causation

Prefer falsification over confirmation.

If a conclusion is weak, say so.

---

# Current Thesis Direction

Current working thesis (updated 2026-06-10, autotune_continuous + theta0=10° fix):

⚠️ MAJOR REVISION: The slew formula had a π/180 unit error (servos modeled 57.3× too slow).
After fix AND restricting design space to realistic hardware (servo_slew [60,200]):
INFEASIBLE was 0. The Iyy × wind_strength controllability boundary is DEAD.
All prior INFEASIBLE findings, Exp4 fidelity results, and Exp5 slew payoff curves are INVALID.

NEW THESIS: Simulator fidelity affects GAIN SELECTION more than GO/NOGO decisions.
Simple model with theta0=10° (autotune_continuous, Kp up to 320) picks VARIABLE gains driven
by discrete-time numerical stability and RMS-minimization for step response — neither correlates
with wind rejection requirements. FRAGILE designs (narrow window) suffer 56.2% false rejection.
EASY designs suffer 12.2% false rejection (gain too low OR too high from simple model).

Gain co-design is the central mechanism: which simulator you tune in determines which gains
you select, and that gain choice determines whether the design appears viable or not.

The S2R fidelity story:
— Old (DEAD): simple model falsely approves INFEASIBLE designs (99.6%)
— Current: simple model picks erratic gains (median Kp≈47-58, bimodal); 56% false REJECTIONS
  for FRAGILE, 12% for EASY; 0% false approvals anywhere.

IMPORTANT: p_unstable still has near-zero correlation with regime (CONFIRMED).
The Iyy × wind controllability boundary does NOT exist with realistic hardware.
INFEASIBLE=2 (R0491, R0688): genuinely cannot exceed nom_sr=0.333 at any Kp up to 320.

The project is studying:
1. Which fidelity modules drive the gain-selection gap (Exp4 gain mechanism study)
2. Whether the gain co-design mechanism can be explained mechanistically (wind vs noise vs other)
3. Hardware validation of the sim-to-real gain gap prediction

---

# Experiment Status

## Exp1 – Regime Mapping

Status: Major revision complete (2026-06-05). All prior Exp1 results are SUPERSEDED.

Methodology (current):
* 1200 LHS designs with T/W > 1 feasibility filter (no longer includes physically unliftable designs)
* Full-physics evaluation: nonlinear_aero + dyn_aero + thrust_curve + cg_shift ON
* Gain search: autotune_continuous — Kd probe [1,4,16,64] at Kp=40; Kp log-search [1,320]
  (10-point coarse + 6-point refine); NO hard cap. Replaces old 5×5 grid (capped at Kp=80).
* Autotune objective: highest mean success rate across seeds 1+2; tiebreak by minimum RMS
* Nominal evaluation: 3-seed average (wind is stochastic; single-seed is unreliable)
* Under/over robustness: single seed=1 (binary pass/fail sufficient)

Design space (current):
* servo_slew_deg_s: [60, 200], static_margin: [-0.30, 0.30], Cm_alpha: [-90, -15]
* motor_scale: [0.5, 3.0], max_gimbal_deg: [2, 15]

HISTORICAL CONTEXT (pre-slew-fix, invalid):
Old counts: EASY=850, MARGINAL=241, FRAGILE=68, INFEASIBLE=41.
Old MARGINAL was slow-servo cluster (servo_slew < 20 deg/s, median 12.5).
All these results used the buggy slew formula — do not cite.

CURRENT counts ([60,200] design space, exp1_regime_index_py.csv, 2026-06-10):
* EASY     (n=1171, 97.6%): all gain conditions pass, nom_sr ≥ 0.80
* MARGINAL (n=11,    0.9%): nom_sr < 0.80, robustness=1.00 — WIND-LIMITED
  — high wind_strength; small Iyy; even aggressive gains can't guarantee success on all wind seeds
  — best_Kp median=96.3 (full physics); simple model gives Kp≈27 → 54.5% false rejection
* FRAGILE  (n=16,    1.3%): fails over-robustness condition (doubled Kp causes failure)
  — best_Kp range=[3.6, 320]: bimodal (low-ceiling designs need Kp=3-16; high-floor need Kp=80+)
  — median Iyy≈0.013 (low inertia → high sensitivity to gain errors)
  — simple model picks erratic Kp (median≈58); full physics needs Kp=64 median → 56.2% false rejection
* INFEASIBLE (n=2,   0.2%): R0491 and R0688 — nom_sr=0.333 at best Kp (up to 320)
  — genuinely uncontrollable: no gain makes them succeed reliably
  — previously appeared FRAGILE under the Kp=80-capped grid (grid artifact, now corrected)

Key findings (2026-06-06, [60,200] design space):

FINDING 1: p_unstable has near-zero correlation with regime (r≈0). CONFIRMED.

FINDING 2 (REVISED 2026-06-10): The max TVC angular acceleration predicts FRAGILE with AUC=0.855.
  PHYSICAL FORMULA (derivable from hardware specs, no fitting required):
    theta_ddot_max = T_avg [N] × sin(max_gimbal_rad) × l_nozzle [m] / Iyy [kg·m²]
    ≈ T_avg × (max_gimbal_deg × pi/180) × l_nozzle / Iyy  (small angle, <2% error at 15°)

  This IS authority_inertia_ratio in physical units:
    authority_inertia_ratio = max_gimbal_deg × motor_scale / Iyy
    theta_ddot_max = authority_ratio × (F15_avg × pi/180 × l_nozzle) = authority_ratio × 0.0628
  Also: theta_ddot_max = keff_full × u_max  where keff_full = F15 × motor_scale × CU_TO_RAD × l / Iyy
    and CU_TO_RAD = pi/180 × 15/12 = 0.02182 (verified from actuator code; u_max = max_gimbal × 12/15)

  Population statistics (n=1200, 2026-06-10, new regime labels from autotune_continuous):
    EASY:     mean theta_ddot_max =  47.2 rad/s²  (median 37.8, range [3.9, 233.0])
    FRAGILE:  mean theta_ddot_max = 110.7 rad/s²  (median 101.9, range [35.7, 201.7])
    MARGINAL: mean theta_ddot_max = 105.5 rad/s²  (median 111.6, range [40.2, 144.9])
  AUC for predicting FRAGILE: 0.855 [0.765, 0.931] (CV: 0.854 ± 0.077, n=10,000 bootstraps)
    NOTE: Old AUC=0.911 was on n=25 FRAGILE labels with Kp=80 grid cap. With autotune_continuous,
    9 grid-capped designs are correctly EASY → n=16 FRAGILE → AUC drops to 0.855.
    Cohen d=1.85 (FRAGILE vs EASY theta_ddot), t=7.33, p<0.0001, n=1187.
  keff_full alone gives IDENTICAL CV-AUC=0.854 ± 0.057 (same as theta_ddot_max, despite NOT using
    max_gimbal explicitly). keff_full range: EASY mean=7.06, FRAGILE mean=12.88 (1.82× separation).

  Practical threshold: Youden-J = 62.3 rad/s² [35.7, 96.3] 95% CI
    At threshold=70 rad/s² (for zero false approvals in this population):
      TP=12, FP=233, FN=4, TN=938 → precision=0.049, recall=0.750, spec=0.801
    Below 35 rad/s²: no FRAGILE designs confirmed in this study.
    Above 100 rad/s²: strongly gain-sensitive.
  NOTE: Low precision (4.9%) reflects rare base rate (1.3%). AUC=0.855 is the right ranking metric.

  authority_inertia_ratio = (max_gimbal_deg × motor_scale) / Iyy — identical AUC to theta_ddot_max.
  FRAGILE mean authority = 1762, EASY mean = 751 (2.3× higher), p<0.0001.
  Does NOT predict required Kp level (wind drives Kp level, not authority).
  Framing: "over-actuated" rockets (high authority per unit inertia) have narrow gain windows.
  14/16 FRAGILE ceiling-limited (over_sr=0, under_sr=1); 1/16 floor-limited (R0336, over_sr=1);
  1/16 both-fail (R0748). The over-test failure (ceiling) accounts for 87.5% of FRAGILE cases.

  THETA_DDOT IS THE OPTIMAL PREDICTOR — RESIDUAL ANALYSIS COMPLETE (2026-06-10):
  Exhaustive H1-H4 hypothesis tests and two-variable search found NO improvement ≥ 0.03.

  H1 (servo_slew): REJECTED — r(slew, FRAGILE)=+0.013, solo AUC=0.45; delta=-0.000 added to td.
  H2 (wind_strength): REJECTED — r(wind, FRAGILE)=-0.001 (FN mean wind=0.213 < EASY=0.246!);
    adding wind to theta_ddot: delta=-0.014 (hurts). Wind does NOT explain the false negatives.
  H3 (aerodynamic stability): REJECTED — r(static_margin, FRAGILE)=+0.034; delta=-0.006.
  H4 (interaction ratios): MARGINAL — best is wind/keff_full (+0.004). Below 0.03 threshold.

  Best two-variable model (exhaustive search over 10 features × 10 features = 45 pairs):
    log_theta_ddot + log_keff_full = CV-AUC=0.865 ± 0.058  (delta=+0.011 over theta_ddot alone)
    log_keff_full + log_theta_ddot/slew = 0.856 ± 0.074 (delta=+0.002)
  All deltas < 0.03 → keep theta_ddot_max as sole predictor.

  FALSE NEGATIVES (n=4, theta_ddot < 70 but FRAGILE, all ceiling-limited):
    R0804: td=35.7, keff=7.29, max_gimbal=6.1°, wind=0.241 — small gimbal, moderate keff
    R0047: td=40.4, keff=8.74, max_gimbal=5.8°, wind=0.328 — small gimbal, moderate keff + high wind
    R0680: td=62.3, keff=6.57, max_gimbal=11.9°, wind=0.099 — near threshold, unusual low wind
    R0452: td=63.0, keff=12.70, max_gimbal=6.2°, wind=0.181 — small gimbal, high keff

  FORENSIC ANALYSIS COMPLETE (2026-06-11): mechanistic 6-step investigation confirms:

  Over test protocol: OVER_SCALE=1.40 — exp1 scales BOTH Kp AND Kd by 1.40×, not 2×.
  All 4 FNs GENUINELY fail seed=1 at 1.40×Kp/1.40×Kd with full fidelity config.
  The FRAGILE classification is CORRECT for all 4 — not an evaluation artifact.

  True SR at 1.40× (7-seed, full fidelity) — SEVERITY VARIES:
    R0804 (td=35.7): SR@1.4x=0.86 — mildly gain-sensitive (14% failure probability)
    R0047 (td=40.4): SR@1.4x=0.86 — mildly gain-sensitive (14% failure probability)
    R0680 (td=62.3): SR@1.4x=0.71 — moderately gain-sensitive (29% failure)
    R0452 (td=63.0): SR@1.4x=0.29 — more severely gain-sensitive (71% failure)

  CRITICAL COMPARISON: Some TP FRAGILE designs show IDENTICAL SR at 1.40×:
    R0084 (td=163.2, TP): SR@1.4x=0.86  — same severity as FN R0804/R0047
    R0373 (td=201.7, TP): SR@1.4x=0.86  — same severity as FN R0804/R0047
    R0405 (td=113.5, TP): SR@1.4x=0.86  — same severity as FN R0804/R0047
  Conclusion: theta_ddot_max predicts which designs fall into the SEVERE category
    (SR@1.4x < 0.57), but the MILD category (SR@1.4x = 0.71-0.86) spans both FN
    and TP labels. The boundary between MILD and SEVERE FRAGILE is probabilistic.

  Physical explanation: these designs have low theta_ddot_max due to SMALL max_gimbal but
    keff_full (per-unit-control sensitivity, independent of max_gimbal) is moderate-to-high.
    keff_full alone would catch R0452 (keff=12.70) but misses R0680 (keff=6.57 near EASY mean).
    Combined model (td + keff) improves AUC by only +0.011 (within CV noise ±0.077).

  SCIENTIFIC CONCLUSION (Step 6, A+C hybrid):
  These 4 designs are NOT revealing a new physical mechanism. They ARE genuinely FRAGILE
  (all fail seed=1 at 1.40×), but represent the MILD end of FRAGILE severity (SR=0.71-0.86),
  which overlaps with some high-td TP designs. The theta_ddot threshold at ~62-70 rad/s²
  separates SEVERE from MILD+EASY, but cannot discriminate MILD FRAGILE from EASY.
  The same theta_ddot → bang-bang amplitude → K_u chain operates for all 4 FNs; their
  lower td simply corresponds to smaller amplitudes and less K_u compression.
  R0452 (keff=12.70 ≈ FRAGILE mean) demonstrates that high keff_full drives more severe
  sensitivity even at borderline theta_ddot, consistent with the keff AUC tie result.
  NO additional predictor is needed. Keep theta_ddot_max as sole design rule (AUC=0.855).

  Why adding variables hurts or barely helps:
    r(wind_strength, FRAGILE) = −0.001 (near zero, now confirmed with n=16 labels).
    FRAGILE mean wind = 0.255 ± 0.134 vs EASY mean wind = 0.246 ± 0.117 — IDENTICAL.
    Similarly: r(servo_slew, FRAGILE) = +0.013 — noise.
    The false negatives actually have LOWER wind than comparable EASY designs.
    Multiplying/dividing theta_ddot by noise variables DEGRADES or barely changes signal.

  PHYSICAL INTERPRETATION:
  FRAGILE is a MECHANICAL property, not an environmental one.
  Gain sensitivity is determined by T, sin(δ_max), L_nozzle, Iyy — the hardware.
  keff_full (= angular accel per CU, independent of max_gimbal) is an equally good predictor
    because the gain-window ceiling is set by proportional loop gain = Kp × keff_full.
    Small-max_gimbal designs have low theta_ddot (saturated acceleration) but same keff_full
    (per-unit sensitivity) → ceiling compresses at similar Kp levels as large-gimbal designs.
  theta_ddot_max ≈ keff_full × u_max; the two AUCs tie because each predicts half the population.

  This turns the AUC=0.855 result from "useful predictor" to "governing rule":
  θ̈_max = T × sin(δ_max) × L_nozzle / Iyy  →  threshold ~62-70 rad/s²  →  gain-sensitive

  PHYSICAL MECHANISM (Q-A sweep n=50, 2026-06-07):
    kp_floor increases with keff_full (r=+0.58, log-log): over-actuation causes overshoot at low Kp
    kp_ceiling: best correlate is slew/(keff×u_max) (r=+0.58 in Q-A sweep) but
      this mechanism is NOT confirmed by v2 window sweep or relay oscillation timing.
      Measured T_u=1.85s vs slew-limit theory T_u=0.17s (11× mismatch) — the ceiling
      is NOT determined by slew-limited oscillation in this regime. The r=+0.58 from
      Q-A may reflect a correlated variable, not the true mechanism. Do not cite.
    window_ratio (ceil/floor) decreases with keff_full (r=-0.61) and Iyy increases it (r=+0.61).
    FRAGILE = narrow gain window = keff_full high AND Iyy low.

  GOVERNING EQUATION CHAIN (2026-06-09, relay+sweep study, n=50):
  ⚠️ This extends and partially supersedes the Q-A floor/ceiling regressions above.

  Step 1 — Physical quantity (Newton's 2nd law, no fitting):
    θ̈_max = T·sin(δ_max)·L_nozzle / Iyy = keff_full × u_max

  Step 2 — Bang-bang oscillation amplitude at probe gain Kp=2 (empirical, n=50):
    rho(theta_ddot, A_deg@Kp2) = +0.781  — primary empirical relationship
    rho(keff_full,  A_deg@Kp2) = +0.812  (slightly stronger than theta_ddot alone)
    Power law: A ≈ 0.95° × theta_ddot^0.57  (probe flight at Kp=2, full fidelity)
    A rises from ~5° (EASY, td<40) to ~21° (FRAGILE, td>150)
    NOTE: oscillation is NOT a slew-limited limit cycle (measured T_u=1.85s vs theory 0.17s).
    It is disturbance-driven: high keff → more aggressive response → larger bang-bang overshoot.

  Step 3 — Describing function formula (Åström-Hägglund 1984, exact derivation):
    K_u = 4·u_max / (π·A_rad)   [ultimate gain from oscillation amplitude]
    Verified: r(K_u_measured, K_u_theory) = 1.000 (n=50)
    => K_u ∝ u_max / A ∝ u_max / theta_ddot^0.57 ∝ 1/keff^0.57 (approximately)

  Step 4 — EASY vs FRAGILE K_u comparison (n=50, relay probe study, 2026-06-09):
    EASY    K_u: median=107  [55-123]  (gain ceiling proxy: high = lots of headroom)
    FRAGILE K_u: median= 39  [27-48]   (gain ceiling proxy: low = ceiling near floor)
    Separation: 2.8× median ratio  (Mann-Whitney p=3.5e-05)
    FRAGILE K_u ≈ 40 ≈ best_Kp_min from wind floor (40-80) → CEILING HAS CLOSED TO FLOOR

  Step 5 — Gain window from direct Kp sweep (12 designs, per-design best_Kd, 7 seeds):
    NOTE: K_u is computed FROM amplitude A using the relay formula — it is NOT an
    independent measurement. The useful finding is the amplitude separation (Step 2).
    K_u is reported here as a ceiling PROXY only (tells you where oscillation onset would
    be if the bang-bang dynamics were harmonic at the measured amplitude).

    Window table (v2, 2026-06-09, tools/kp_window_sweep_v2.py):
      td=  3.9 EASY     ceiling=320 floor=1  window=320×
      td= 12.1 FRAGILE  ceiling=127 floor=1  window=127×  [see anomaly note]
      td= 27.3 EASY     ceiling=320 floor=1  window=320×
      td= 50.0 EASY     ceiling=320 floor=1  window=320×
      td= 51.0 EASY     ceiling=320 floor=1  window=320×
      td= 62.1 EASY     ceiling=202 floor=1  window=202×
      td= 72.9 EASY     ceiling=160 floor=2  window= 80×
      td= 99.5 EASY     ceiling=127 floor=2  window= 64×
      td=105.8 FRAGILE  ceiling=127 floor=2  window= 64×
      td=138.2 FRAGILE  ceiling=127 floor=8  window= 16×  [clearest FRAGILE: peakSR=0.86]
      td=212.6 FRAGILE  ceiling=202 floor=5  window= 40×  [see anomaly note]
      td=230.5 EASY     ceiling=320 floor=1  window=320×  [see anomaly note]

    Power law (v2): window ≈ 670 × theta_ddot^-0.41  r=-0.480  ← NOT RELIABLE (n=12)
    DO NOT CITE this exponent. The r=-0.480 is too weak and n=12 too small.

    Qualitative pattern (what IS robust):
      td < 30: all designs have ceiling ≥ 320 (universal tolerance below this range)
      td > 100: ceiling compresses to 127-202 for all FRAGILE and some EASY designs
      FRAGILE classification: all 4 FRAGILE in sweep have ceiling ≤ 202. The exp1 over
        test uses OVER_SCALE=1.40 (not 2×), scaling BOTH Kp and Kd — Kp=160 is ~1.4×
        best_Kp for the td≈138 design, which exceeds ceiling → over test fails.

    NOTE — R0523: This was a false-negative in the old n=25 Kp=80-capped run (td=12.1, Kp=80
      cap artificially created FRAGILE label). With autotune_continuous, best_Kp search is
      uncapped; R0523 is now CORRECTLY labeled EASY. Not a false negative in current data.

    ANOMALY — R0759 vs R0255 (both td≈210-230, different regimes):
      R0759 (FRAGILE): over_sr=0.000 in exp1 (1 seed at 1.40×best_Kp; wind=0.407)
      R0255 (EASY): over_sr=1.000 (passes at 1.40×best_Kp; wind=0.357)
      v2 sweep shows R0759 ceiling=202 (SR=0.71 at Kp=160 with 7 seeds). The FRAGILE
      classification is a probabilistic edge case consistent with the mild-FRAGILE category:
      P(fail seed=1 | p=0.71) = 0.29. Both higher wind (0.407 vs 0.357) and seed variance
      contribute; forensic analysis shows this pattern is EXPECTED for mild-FRAGILE designs.
      NOTE: exp1 over test uses 1 seed only (seed=1), not 3 seeds — earlier note was wrong.

  GOVERNING EQUATION SUMMARY (updated 2026-06-09):
    "High θ̈_max rockets oscillate more aggressively at any sub-optimal Kp (rho=0.78).
     By the describing function, the amplitude increase means K_u ≈ 39 for FRAGILE vs
     107 for EASY (2.8×). When K_u ≈ wind_floor (40-80), no valid gain exists."

    What IS supported:
      - Newton → θ̈_max: derivable from hardware specs (no fitting)
      - θ̈_max → bang-bang amplitude: empirical (rho=0.78, n=50)
      - Amplitude → K_u: exact via describing function (definitional, r=1.000)
      - K_u separation EASY vs FRAGILE: robust (2.8×, p=3.5e-05, n=50)
      - Qualitative ceiling compression at td>100: confirmed but not quantified

    What is NOT supported:
      - Power law window ∝ theta_ddot^B: r=-0.48 too weak (n=12 sweep)
      - The v1 exponent ^-1.69 was Kd=8 confounded — DO NOT CITE
      - Slew-based ceiling derivation: oscillation T_u=1.85s vs theory 0.17s (11× mismatch)

    The amplitude exponent 0.57 is empirical; the K_u formula is exact (1984 theory).
    The threshold ~70-100 rad/s² is environment-dependent (wind, slew, success gate).

FINDING 3 (SURVIVES): Aerodynamic instability does not help maneuverability.
  Stable designs outperform unstable on all tasks. Gap grows with maneuver amplitude.
  At 10° maneuver, stable=86.8% vs unstable=68.8% at slew=120 (18pp gap, does not close).
  In Exp1 attitude-hold population: stable mean nom_sr=0.999 vs unstable=0.995 (small but p=0.032).

FINDING 4 (S2R, n=1200, 2026-06-10): Simulator fidelity changes GAIN SELECTION more than GO/NOGO.
  | Regime   | n    | SR(simple gains) | SR(full gains) | Gap   | False Rejection |
  |----------|------|-----------------|----------------|-------|-----------------|
  | EASY     | 1171 | 0.838           | 1.000          | 0.162 | 12.2%           |
  | MARGINAL | 11   | 0.424           | 0.667          | 0.242 | 54.5%           |
  | FRAGILE  | 16   | 0.417           | 0.938          | 0.521 | 56.2%           |
  Simple model (theta0=10°, autotune_continuous, no disturbances) picks gains driven by
  discrete numerical stability + RMS minimization, NOT wind rejection requirements:
    EASY:     median Kp_simple=46.8 vs Kp_full=70.2  — bimodal: 53% undertuned (Kp<10),
              17% overtuned (Kp>200 causes ceiling miss in full physics)
    MARGINAL: median Kp_simple=26.7 vs Kp_full=96.3  — systematically undertuned (3.6×)
    FRAGILE:  median Kp_simple=58.2 vs Kp_full=64.4  — close median but erratic individuals;
              some picks Kp=1-3 (below floor), some picks Kp=252-320 (above ceiling)
  False approval rate = 0.0% across all regimes.
  Adding theta0=10° WORSENED false rejection vs old Kp=2 baseline (EASY: 12% vs 3%, FRAGILE:
  56% vs 52%), confirming the problem is not initial conditions but absence of disturbance physics.

FINDING 5 (FLIGHT SIGNATURE, n=50, 2026-06-07): Single test flight at Kp=2 detects FRAGILE
  designs with AUC=0.947 (RMS), outperforming the spec formula (AUC=0.890 on same 50 designs).

  METHOD: Deploy Kp=2 in full fidelity (wind + slew + noise + latency). Measure flight RMS.
  DATA: 25 FRAGILE + 25 EASY designs, 7 seeds each. Source: flight_signature_py.csv.

  AUC by metric (single flight):
    AUC(rms, 1 seed) = 0.947   ← primary detector
    AUC(peak, 1 seed) = 0.941
    AUC(slew_sat, 1 seed) = 0.920
    AUC(osc, 1 seed) = 0.654   (weaker — bang-bang oscillation, high variance)

  AUC by metric (7-seed average):
    AUC(rms) = 0.963, AUC(peak) = 0.968, AUC(slew_sat) = 0.925

  DETECTION THRESHOLD (single flight, zero false positives):
    RMS > 7.6° → FRAGILE predicted: precision=1.00, recall=0.80, F1=0.89
    Interpretation: if the test flight has RMS > 7.6°, the rocket is DEFINITELY gain-sensitive.
    If RMS < 7.6°, it's probably EASY but 20% of FRAGILE designs might be missed.
    Complementary use with spec formula: spec screens, flight confirms.

  DETECTION THRESHOLD (7-seed, best F1):
    RMS > 6.6°: precision=0.92, recall=0.92, F1=0.92 (TP=23, FP=2, FN=2, TN=23)
    Peak > 12.2°: precision=0.92, recall=0.92, F1=0.92 (same TP/FP)
    slew_sat > 0.5: precision=0.85, recall=0.92, F1=0.88

  CLASS SEPARATION at Kp=2 (7-seed means):
    RMS: FRAGILE=11.3° (+/-3.9°) vs EASY=3.9° (+/-1.7°), ratio=2.9x
    slew_sat: FRAGILE=0.66 vs EASY=0.28, ratio=2.4x
    Mean SR: FRAGILE=0.53 vs EASY=0.96
  NOTE: Flight signature was computed on OLD n=25 FRAGILE + 25 EASY from pre-autotune_continuous
  run. New FRAGILE is n=16 (9 reclassified to EASY). AUC numbers need recomputation on new labels.

  WIND CONFOUND CHECK: AUC by wind_strength tercile
    Low wind  [0.06-0.18]: AUC(rms)=0.924 — high even at low wind
    Mid wind  [0.19-0.31]: AUC(rms)=1.000 — perfect separation
    High wind [0.32-0.41]: AUC(rms)=0.986
    Conclusion: detection is NOT a wind confound. High RMS in FRAGILE reflects gain-induced
    bang-bang oscillation, not just wind magnitude.

  SPEC vs FLIGHT comparison (same 50 designs):
    authority_inertia_ratio (spec):   AUC=0.890
    RMS from 1 test flight (flight):  AUC=0.947  → +0.058 improvement
    RMS from 7 seeds (ground truth):  AUC=0.963

  PHYSICAL EXPLANATION: FRAGILE designs at Kp=2 enter bang-bang oscillation (slew_sat~0.66)
  because Kp=2 is below or near the kp_floor. The servo permanently saturates trying to reject
  wind disturbances with insufficient proportional gain. This creates persistent large-amplitude
  oscillations that are clearly visible in a single flight.

  PRACTICAL WORKFLOW:
    Step 1: Compute theta_ddot_max = T × sin(max_gimbal_rad) × l_nozzle / Iyy from specs
    Step 2: If theta_ddot_max > 70 rad/s², suspect gain sensitivity
    Step 3: Fly at Kp=2; if RMS > 7.6°, confirmed FRAGILE → tune Kp=40-80
    Step 4: If RMS < 7.6° but ratio > 70, fly a few more seeds to be sure

  CAVEAT: Tested on 50 designs (25F + 25E), all from [60,200] design space. Threshold (7.6°)
  depends on success gate (RMS < 15°) and evaluation duration. Validate before use on
  different hardware configurations.

FINDING 6 (ADRC STEP TRACKING, n=1200, 2026-06-09): ADRC achieves 21× lower tracking RMS
  and 12× better success rate vs PID for a 15° step command.

  SETUP: theta_step_deg=15, theta_step_time_s=1.0, t_end=4.0s, 5 seeds, full physics.
  PID at best_Kp (≈80 for most designs), ADRC at omega_c=5, omega0=25, b0 from specs.

  | Metric          | PID (Kp=80) | ADRC (ωc=5) | Improvement |
  |-----------------|-------------|-------------|-------------|
  | Success rate    | 0.080       | 0.972       | 12.2×       |
  | Tracking RMS    | 47.9°       | 2.3°        | 21×         |
  | Peak angle      | 92°         | 17°         | 5.4×        |
  | Rise time       | 0.242s      | 0.804s      | 3.3× slower |

  By regime: EASY (PID SR=0.08 → ADRC SR=0.97), FRAGILE (PID SR=0.02 → ADRC SR=0.87).
  ADRC improvement is UNIVERSAL across all authority bins (11-25×).

  PHYSICAL MECHANISM: PID requires Kp=80 for wind rejection → servo saturates during
  step → 77° overshoot → 92% failure. ADRC ESO cancels wind before control law → servo
  never saturates → smooth 2° overshoot → 97% success. These are INDEPENDENT constraints
  in ADRC (ESO bandwidth handles wind; ωc handles tracking speed) but COUPLED in PID.

  AUTHORITY SATURATION (from pitch_tracking_pareto_py.csv, n=1200, 2026-06-09):
  θ̈_max > 70 rad/s²: step response saturates at 0.22s; over_sr drops from 0.997 to 0.704.
  θ̈_max < 70 rad/s²: step response still improving (0.274→0.238s); full robustness.
  The gain-sensitivity threshold (70 rad/s²) is ALSO the agility-saturation threshold.
  Practical meaning: increasing authority beyond 70 rad/s² costs robustness without speed gain.
  Files: pitch_tracking_pareto_py.csv, adrc_step_tracking_py.csv.

  PARETO FRONTIER (NULL RESULT 2026-06-09):
  r(nom_sr, rise_time_best) = -0.005 — no clean R-M Pareto exists in this formulation.
  The "FRAGILE is faster" hypothesis is confounded by gain-selection (FRAGILE designs with
  best_Kp=5 are slow). Within Kp=80 group (n=809): r=-0.603 (mechanism real but effect small,
  7% speed difference for 2.56× authority ratio). The Pareto is highly asymmetric:
  negligible agility benefit (+7%) at massive robustness cost (gain window collapses).
  Do NOT claim a Pareto frontier — the authority saturation story replaces it.

  ADRC FAILURE CASES: 3% of designs (32/1200) have ADRC SR < 0.80. These are high-b0
  designs where success gates are borderline. Tracking quality (trms=6-12°) is still
  far superior to PID (47.9°). The SR failure is likely due to success-gate sensitivity
  to the step transient, not a fundamental ADRC limitation.

  ADRC BANDWIDTH CONSTRAINT for FRAGILE:
  High-b0 FRAGILE designs (b0>15) become unstable at ωc≥10 (tested: R0267, b0=17.4).
  ωc=5 is safe for ALL designs (n=1200). ωc=12 works for EASY designs (b0<12).
  For EASY at ωc=12: rise=0.225s (same as PID), trms=7.2° (4× better than PID).

Confidence: HIGH for FINDING 1, HIGH for FINDING 4 (S2R, n=1200), HIGH for FINDING 2.
HIGH for FINDING 5 (n=50, clean wind confound check, zero false positives at 1.00 threshold).
HIGH for FINDING 6 (n=1200, universal across all authority bins).
FINDING 2: theta_ddot_max gives CV AUC=0.855 [0.765,0.931] on new n=16 FRAGILE labels. keff_full
alone gives identical AUC=0.854. No additional variable improves by ≥0.03. Old AUC=0.911 was
on old n=25 grid-capped labels — now SUPERSEDED by AUC=0.855.

---

## Exp4 – Fidelity Ablation

Status: Complete (N=1200, 3 seeds per condition)

Goal:

Determine which simulator fidelity terms change engineering conclusions.

Modules:

* wind
* sensor_noise
* slew
* backlash
* latency
* thrust_var
* deadband

⚠️ ALL OLD EXP4 RESULTS INVALID — based on pre-slew-fix simulator and old design space.
Do NOT cite old frequency-of-effect rates, false-approval rates, or module dominance rankings.

CURRENT S2R results (2026-06-10, full population n=1200, exp4_s2r_gains_py.csv):

Simple model (theta0=10°, autotune_continuous) picks VARIABLE Kp driven by discrete numerical
stability + RMS minimization. Gains are NOT correlated with wind rejection requirements.
Full physics optimal: EASY median Kp=70.2, MARGINAL median Kp=96.3, FRAGILE median Kp=64.4.

| Regime   | n    | SR(simple→full) | SR(full→full) | Gap   | False Approval | False Rejection |
|----------|------|-----------------|---------------|-------|----------------|-----------------|
| EASY     | 1171 | 0.838           | 1.000         | 0.162 | 0.0%           | 12.2%           |
| MARGINAL | 11   | 0.424           | 0.667         | 0.242 | 0.0%           | 54.5%           |
| FRAGILE  | 16   | 0.417           | 0.938         | 0.521 | 0.0%           | 56.2%           |

Key insight: simple model is NOT dangerous (0% false approvals anywhere).
FRAGILE designs: 56% get falsely REJECTED with simple-model gains (gain too low OR too high).
EASY designs: 12% false rejection — simple model picks gains outside the full-physics window.
False rejection increased vs old Kp=2 baseline: adding physical theta0 didn't fix the problem.
Root cause: disturbance-free simulator creates gain landscape with NO signal about wind rejection.

GAIN MECHANISM STUDY (exp4_gain_mechanism_py.csv) — STATUS: DOUBLY INVALID, NEEDS FULL RERUN

⚠️ ARTIFACT 1 (theta0=0, original): With theta0=0 AND wind+noise ablated → RMS=0 for ALL Kp →
autotune picks Kp=2 by grid tie-breaking. CONFIRMED: Kp=2,5,15,40,80,160 all give RMS=0.

⚠️ ARTIFACT 2 (theta0=3.0 radians, NEWLY DISCOVERED 2026-06-07): The "fix" set theta0_bias_std=3.0
which is 3.0 RADIANS = 171.9° standard deviation (NOT 3 degrees as described in CLAUDE.md).
Effect: seed-draws like z=2.04 give theta0=6.1 rad = 350° (rocket starts nearly inverted!).
CONSEQUENCE: With huge initial angles (>90°), max_theta > 70° immediately → ALL Kp fail →
tie-breaking STILL picks Kp=2 → SAME artifact as theta0=0, just for a different reason.
CONFIRMED: SR(Kp=40 with theta0=3.0 rad) = 0.10 for R0491; SR should be 0.75+ with correct theta0.

FIX IMPLEMENTED (2026-06-10):
  Simple-model autotune now uses theta0_fixed_deg=10.0 + autotune_continuous (Kp=[1,320]).
  Kp=80 grid cap is gone. GAIN_MECH_TUNE_THETA0_STD fixed to 3*pi/180 = 0.05236 rad.

RESULT OF FIX: simple model now picks VARIABLE Kp (median 46.8-58.2 by regime), but false
rejection WORSENED (EASY 12.2% vs old 2.8%, FRAGILE 56% vs old 52%). The problem is NOT the
initial condition — it is the ABSENCE OF DISTURBANCE PHYSICS. Two failure modes discovered:
  1. Undertuning (53% of false-rejected EASY): high-keff designs get Kp<10 in simple model
     (discrete numerical ceiling from keff×Kp instability). Kp=10 fails in full-physics wind.
  2. Overtuning (17% of false-rejected EASY): low-keff designs get Kp=250-320 (RMS min pushes
     to search ceiling). Kp=300 exceeds true ceiling in full physics → oscillation → failure.
For FRAGILE: same bimodal behavior; narrow window means ANY mismatch causes failure.

WHAT IS VALID:
- S2R false rejection is real and confirmed: 56% for FRAGILE, 12% for EASY (updated numbers)
- False approval = 0% (simple model never declares a failing design as passing)
- Root cause: disturbance-free simulator gain landscape is uninformative about wind rejection

DO NOT CITE: any numbers from exp4_gain_mechanism_py.csv (both runs are invalid artifacts).
Gain mechanism re-run with corrected theta0 (3 degrees) is still pending if needed.

Confidence: HIGH for S2R results (n=1200, autotune_continuous).

---

## Exp5 – Design-Space Topology

Status: Partial — topology analysis removed; stratified slew payoff is new lead result

Current outputs:

* gradient field (central finite differences, 3-seed average, range-scaled)
  — Exp5 CSV now has both grad_rms_* (raw, physical units) and grad_scaled_* (range-scaled)
  — Do NOT compare raw grad_rms_* columns across parameters (unit bias)
* evolution paths (5 gradient-descent steps per design, 7200 rows)
* diminishing returns curves (population-level, top 5 parameters)
* NEW: regime-stratified servo slew payoff curves (exp5_slew_stratified_py.csv)

Topology analysis REMOVED from paper figures (2026-06-03):
* Cliff/bowl classification is uniformly distributed across regimes (~15-17% cliff in all)
* Does NOT correlate with regime boundaries or stability proximity
* Not validated by seed-variance test
* Do not cite topology results in paper

⚠️ ALL EXP5 SLEW PAYOFF RESULTS ARE INVALID — based on pre-slew-fix simulator.
The regime-stratified slew payoff curves (EASY/MARGINAL/FRAGILE/INFEASIBLE) used the buggy
slew formula. With INFEASIBLE=0 and MARGINAL redefined as slow-servo, these curves are void.

Surviving result: Exp5 gradient/topology analysis was already removed from paper figures (2026-06-03).
There are no surviving Exp5 results ready for publication.

Known issues:

* Stratified payoff uses full fidelity (thrust_var fault included) — success rates are
  lower than Exp1 values; relative regime ordering is informative, absolute levels are not
* 3-seed gradients: Iyy at 13.9% best_param may still be noise
* INFEASIBLE gradient analysis removed (numerically unreliable for diverging trajectories)
* Gains frozen from Exp1; not re-tuned per slew value — bidirectionality still present

Confidence:

* Stratified slew payoff: MEDIUM-HIGH (clear qualitative ordering across regimes)
* Diminishing returns: MEDIUM-HIGH
* Terrain maps: MEDIUM
* Gradient bottlenecks: LOW (do not cite without multi-seed validation)
* Evolution paths: LOW (do not cite without multi-seed validation)

---

# Known Methodological Concerns

These issues must be remembered during future analysis.

## keff_simple ≠ keff_phys — TWO DIFFERENT EFFECTIVE GAINS (discovered 2026-06-07)

The design space has TWO distinct keff values:
  keff_simple = control_effectiveness column (LHS-sampled, range [5.01, 14.00]) — SIMPLE mode
  keff_full   = T × CU_TO_RAD × l_nozzle / Iyy (range [1.53, 21.93]) — FULL PHYSICS mode
  where CU_TO_RAD = π/180 × 15/12 = 0.02182 rad/CU (max_gimbal cancels)

Correlation: r(keff_simple, keff_full) = 0.008 — UNCORRELATED (LHS-sampled independently).

Direction of mismatch (n=1200):
  Overall:  74% have keff_simple > keff_full (simple more responsive)
            26% have keff_full > keff_simple (physics more responsive)
  FRAGILE:  80% have keff_full > keff_simple  ← FRAGILE designs reversed!
            mean ratio keff_full/keff_simple = 1.60 (physics 1.6× higher for FRAGILE)
  EASY:     mean ratio = 0.81 (simple is typically higher for EASY)

Example — R0491 (FRAGILE): keff_simple=7.553, keff_full=17.87, ratio=2.37
Example — REF design: keff_simple=8.0, keff_full=4.36 (SIMPLE IS HIGHER at reference conditions)

Consequence for gain selection:
  PRIMARY reason for Kp=2 selection: NO WIND in simple model → any Kp achieves SR=1.0 → grid min
  COMPOUNDING factor: FRAGILE designs have keff_full ≈ 1.6× keff_simple → higher bang-bang
    authority → slightly elevated kp_floor in full physics vs what simple model would expect.
    Contributing factor ~1.6× to kp_floor elevation; remaining 10-20× comes from wind-driven
    bang-bang requirements that the simple model never sees.

This is a structural finding (the two keffs are uncorrelated), not a strong quantitative claim.
The primary S2R story is the wind issue, not the keff mismatch.

## Gain Window is bang-bang limited (discovered 2026-06-07)

FRAGILE designs with high keff_phys operate in slew-saturated bang-bang mode:
- slew_sat_frac ≈ 0.78 for R0491 at ALL Kp values (including Kp=2 and Kp=80)
- The servo is permanently at maximum slew rate due to wind-driven angular rates

In this regime: end_mean ∝ 1/Kp (confirmed: 16.8° at Kp=12 → 10.4° at Kp=20 ≈ 12/20 ratio)
kp_floor: where bang-bang limit cycle amplitude drops below 15° end_error threshold
  Limit cycle amplitude ≈ C × d_eff × tau_gust / Kp
  kp_floor ≈ C × d_eff × tau_gust / (15° in radians)

kp_ceiling: where bang-bang chatter/oscillation becomes unstable
  Best empirical predictor: authority_ratio = max_gimbal × motor / Iyy (r=-0.545)
  Formula: r(log_keff_phys, log_ceiling) = -0.458 (correct direction: higher keff → lower ceiling)
  Physical reason: higher keff in bang-bang → larger overshoot pulse → oscillation sooner

EMPIRICAL REGRESSIONS FROM Q-A SWEEP (n=50, Kp=[1,320], 2026-06-07):
  keff_full = T × 0.02182 × l_nozzle / Iyy   [rad/s²/CU, max_gimbal cancels]
  keff_full range: [2.04, 21.51] rad/s²/CU; EASY mean=6.63, FRAGILE mean=13.77

  kp_floor: log-log r(keff_full, kp_floor) = +0.582   ← BEST single predictor
    kp_floor ≈ 0.35 × keff_full^0.70   (regression, n=50)
    d_eff/keff formula gives r=-0.195 (wrong direction) — disturbance is NOT the floor driver

  kp_ceiling: best predictor in Q-A sweep is slew/(keff×u_max), r = +0.579
    kp_ceiling ∝ slew_code / (keff_full × u_max_code)  [servo speed relative to authority]
    log-log regression: kp_ceiling ≈ 158 × (slew/(keff×u_max))^0.32   (r=0.579)
    ⚠️ SLEW MECHANISM NOT CONFIRMED (2026-06-09): measured relay oscillation T_u=1.85s
    vs slew-limit theory T_u=0.17s (11× mismatch). Oscillation is disturbance-driven,
    not slew-limited. The r=0.579 likely reflects correlation, not true ceiling mechanism.

  window_ratio = kp_ceiling/kp_floor:
    r(keff_full, window_ratio) = -0.606; r(Iyy, window_ratio) = +0.611
    FRAGILE = keff_full high + Iyy low → narrow window

  AUC results (Q-A subset n=50): keff_full=0.870, authority_ratio=0.890
  Full population AUC (n=1198, new n=16 FRAGILE labels, 2026-06-10):
    keff_full=0.848 (Full), CV=0.854 ± 0.057
    authority_ratio=0.855 (Full), CV=0.854 ± 0.077
    theta_ddot_max=0.855 (Full), CV=0.854 ± 0.077  ← all three are EQUIVALENT
  NOTE: Old values (keff=0.873, authority=0.911) were on n=25 grid-capped labels — SUPERSEDED.
  The AUC tie between keff_full and theta_ddot_max is explained by the residual analysis:
    keff catches designs with high per-unit sensitivity (small max_gimbal + high keff);
    theta_ddot catches designs with high total authority (large max_gimbal + high keff).
    They fail on different designs, but neither is better overall (AUC tied).

NOTE: keff_full × u_max = theta_ddot_max (max TVC angular acc) = authority_ratio × 0.0628.
  u_max = max_gimbal_deg × 12/15 (verified from simulator code).
  CU_TO_RAD = π/180 × 15/12 = 0.02182 (constant for all designs regardless of max_gimbal).
  keff_full is INDEPENDENT of max_gimbal (per-unit sensitivity at any gimbal angle is constant).

IMPLICATIONS FOR FORMULA DERIVATION:
  The FRAGILE condition is not from linear bandwidth/phase-margin analysis.
  It comes from nonlinear bang-bang dynamics in slew-saturated regime.
  The physical formula theta_ddot_max = T × sin(max_gimbal) × l / Iyy gives AUC=0.855.
  A closed-form threshold from first principles requires nonlinear bang-bang analysis
  (boundary depends on wind strength, servo speed, success criteria). The AUC=0.855
  from theta_ddot_max is currently the best single-number design rule.

## theta0_bias_std Units Bug (discovered 2026-06-07)

simulator.py line 153: theta0 = scenario.theta0_bias_std * rng.standard_normal()
theta0 is stored/used in RADIANS. theta0_bias_std=3.0 means 3.0 RADIANS = 171.9° std.

CORRECT usage:
  For 3-degree std: theta0_bias_std = 3 * pi/180 = 0.05236
  For 0 initial offset (Exp1 standard): theta0_bias_std = 0.0
  theta0_fixed_deg (line 150) IS in degrees and converts to radians ✓

IMPACT: Exp1, Q-A sweep, Q-C basin sweep all use theta0=0.0 → UNAFFECTED.
  Only the gain mechanism study used theta0=3.0 rad → INVALIDATED (see Exp4 section).

## Exp4 Baseline Mismatch — QUANTIFIED

Exp4 full-fidelity conditions differ from Exp1 evaluation conditions because
Exp4 includes a thrust_var fault (keff drops 15% at t=1.5s) that was NOT present
during Exp1 regime labeling.

Quantified impact (from Exp4A audit, 2026-06-03):
- EASY: 78.3% GO/NOGO agreement  [74.4%, 81.7%]
- FRAGILE: 50.3% agreement  [45.8%, 54.8%] — effectively a coin flip
- INFEASIBLE: 99.6% agreement (thrust fault doesn't change uncontrollable verdict)

Interpretation: The baseline mismatch primarily affects FRAGILE designs.
Any fidelity claim involving FRAGILE designs must caveat this.

Next step: Run exp4simple (FidelityConfig.simple() for all 1200 designs) to get
actual paired simple-vs-full decision comparison.

---

## Decision Dominance Tie Risk

Decision dominance based on delta_success may contain many ties.

Before accepting dominance rankings:

* audit tie frequency
* audit tie-breaking behavior
* verify conclusions are stable

---

## Gradient Reliability

Current Exp5 gradients use 3-seed averaging (improved from single-seed).
3 seeds reduces stochastic variance ~1.7× but is not fully converged.

Potential future fixes:

* 5+ seed gradients (preferred for publication)
* local response surfaces
* surrogate-based derivatives

Topology classification has been hardened with absolute gradient guard
(cliff_abs_min=0.05) to prevent noise-spike misclassification.

---

## Regime Circularity

Be careful when a later analysis "discovers" the importance of variables already used in regime construction.

Always check for circular reasoning.

## Autotune Methodology Sensitivity (updated 2026-06-10)

Current autotune: autotune_continuous (2-seed SR primary, RMS tiebreak; Kp=[1,320] log-search).
Old grid (5×5 KP=[2,5,15,40,80], KD=[1,2,8,16,32]) was REPLACED due to Kp=80 hard cap causing
9 genuinely-solvable FRAGILE designs to be misclassified (needed Kp=88-320).

Sensitivity comparison:
* Old grid (capped at 80): EASY=1169, MARGINAL=6, FRAGILE=25, INFEASIBLE=0
* autotune_continuous (uncapped): EASY=1171, MARGINAL=11, FRAGILE=16, INFEASIBLE=2

Key difference: FRAGILE count dropped 25→16 (9 designs solved at Kp>80). INFEASIBLE=2 confirmed
(R0491, R0688: no Kp up to 320 achieves nom_sr>0.333). Regime boundary shifts by ~9 designs.

Practical guidance: Report EASY+MARGINAL as "controllable" (98.5%) and FRAGILE+INFEASIBLE
as "at-risk" (1.5%). The FRAGILE count (n=16) is more accurate than old n=25 which included
grid-cap artifacts.

## Gain Grid Design (2026-06-05)

Current grid: KP=[2,5,15,40,80], KD=[1,2,8,16,32] (5×5 = 25 combinations)
Autotune: 2-seed average success rate primary, then RMS tiebreaker, then best-effort RMS.

Known limitation: 14/41 INFEASIBLE designs still show under_sr > nom_sr (34%).
These are near-boundary designs where the true optimal gains lie between grid points.
Continuous optimization (Bayesian or gradient-based) would resolve this but is out of scope.

## T/W Filter (2026-06-05)

Design space now enforces T/W > 1 in sample_lhs() — iteratively resamples until all
designs have motor thrust exceeding rocket weight. Without this filter, 20.7% of designs
were physically unliftable but appeared EASY in simulation (near-zero aerodynamic forces).

## dyn_aero Reference Pressure (2026-06-05)

When dyn_aero=OFF, the constant reference q_dyn is now per-design:
  q = 0.5 × 1.20 × v_mid²  where  v_mid = max(0.5, (T_eff - m×g)/m) × (t_end/2)
This ranges from ~0.3 Pa (low motor_scale + heavy) to ~1900 Pa (high motor_scale + light).
Old value: hardcoded 540 Pa for all designs — inflated aerodynamic forces by 10-100× for
most designs, making dyn_aero ablation results partly a calibration artifact.

---

# Rejected or Unsupported Claims

Do not present these as established findings.

* Sensor noise globally dominates simulator fidelity requirements.
  (Disproven by delta_success analysis — wind dominates GO/NOGO decisions.)
* Infeasible rockets are primarily aerodynamic-limited.
  (DISPROVEN 2026-06-05: INFEASIBLE is driven by high wind_strength + low Iyy, not high p_unstable.)
* p_unstable is the primary predictor of controllability.
  (DISPROVEN 2026-06-05: p_unstable has near-zero correlation with regime under full-physics eval.
   Iyy and wind_strength are the dominant predictors.)
* Aerodynamic instability improves wind resistance or maneuverability for TVC attitude-hold.
  (DISPROVEN: instability amplifies wind disturbances; stable aerodynamics help absorb wind.
   The fighter-jet analogy does not apply to attitude-hold TVC rockets.)
* slew × latency interaction is super-additive.
  (DISPROVEN: mean interaction ratio = 0.55 across 50 tests — strongly sub-additive.
   Both ablations push gains in the same direction; combined effect saturates at single-ablation level.)
* Gradient bottlenecks are validated design levers.
* Evolution paths represent physically validated improvement trajectories.
* Topology classes have been proven reproducible.

These remain hypotheses or open questions.

---

# High-Priority Open Questions

1. Which fidelity terms matter in each regime?
   — Partially answered: wind dominates GO/NOGO; sensor_noise dominates RMS.

2. Where do fidelity handoffs occur?
   — Partially answered: spatial maps in fidelity_dominance.py figures.

3. Can fidelity requirements be predicted from physical rocket properties?
   — Open: needs Fidelity Requirement Atlas (atlas-style classifier).

4. Does the stability frontier also predict simulator complexity requirements?
   — Partial: INFEASIBLE needs 2-3 modules always; EASY can often use simple sim.

5. Which Exp5 outputs survive multi-seed validation?
   — Partially answered: 3-seed run complete. servo_slew dropped from 42% → 25% (noise
     confirmed). Iyy enters top-3 (13.9%) but may still be noise — needs 5-seed validation.

6. Which simulator findings survive real flight testing?
   — Not started: requires hardware bench data + 6-12 flights.

7. What is the actual simple-model vs full-model decision disagreement rate?
   — ANSWERED (exp4simple, 2026-06-03): INFEASIBLE false approval rate = 99.6%;
     FRAGILE agreement = 50.3% (coin flip); EASY agreement = 78.3%.
     Simple model is always optimistic — zero false rejections across all regimes.

---

# Novelty Assessment (2026-06-06)

## What is NOT novel (textbook or well-known)

* Authority/inertia ratio concept — T_max/Iyy appears in every spacecraft attitude control
  textbook (Wie; Sidi). The ratio max_gimbal × thrust / Iyy is standard in GNC design.
* "Simple simulators without disturbances pick wrong gains" — known in control theory since
  Astrom 1971 (self-tuning regulators) and extensively studied in robotics sim-to-real work.
* Over-actuated systems have narrow gain windows — core of robust control theory (Doyle et al).

## What IS novel (specific to this study)

* Systematic quantification of gain sensitivity across a 1200-design LHS study for hobby TVC.
* authority/Iyy ratio as a single-feature predictor for hobby TVC: CV AUC=0.855 [0.765,0.931]
  (threshold ~62-70 rad/s²). No environmental variable (wind, slew, aerodynamics) improves AUC
  by ≥0.03 (exhaustive two-variable search). FRAGILE is a mechanical property, not environmental.
* 56% false rejection rate for gain-sensitive designs when tuning in disturbance-free simulator.
* Aerodynamic instability is irrelevant to gain sensitivity in hobby-scale TVC (r≈0, confirmed).
* The specific domain (hobby-scale TVC, 0.5-3 kg, low-cost servos) is not covered in prior literature.
* PHYSICAL FORMULA (2026-06-10): theta_ddot_max = T × sin(max_gimbal_rad) × l_nozzle / Iyy gives
  AUC=0.855 from hardware specs alone — no fitting, no tuning, directly derivable and testable.
  FRAGILE mean=110.7 rad/s², EASY mean=47.2 rad/s² (2.3× separation, Cohen d=1.85, n=1187).
  keff_full (per-unit sensitivity, max_gimbal-independent) gives IDENTICAL AUC=0.854.
  This converts the authority/Iyy proxy to physical units a builder can compute from specs.
* FLIGHT DETECTION (2026-06-07): Single test flight at Kp=2 detects gain-sensitive rockets with
  AUC=0.947 (RMS) — slightly better than the spec formula (AUC=0.890 on same designs, n=50).
  Detection threshold RMS > 7.6°: precision=1.00, recall=0.80 (zero false alarms).
  Robust across wind levels (AUC 0.924-1.000 by wind tercile) — not a wind confound.
  Practical workflow: compute spec formula, then confirm with single test flight at Kp=2.

## Honest STS Assessment

State regional / ISEF qualifier: >85% probability (strong, well-conducted study).
STS semifinalist (300/1800): 35-50% probability (competition claims are not conceptually new but
  are novel applications with rigorous quantification).
STS finalist (40/300):
  Without hardware: 25-35% (formula + flight detection workflow is now a complete engineering
    tool, not just a classifier. AUC=0.947 from a single flight is a strong quantitative claim).
  With hardware (Kp=2 detection confirmed, Kp=80 improvement confirmed): 45-60%.
  What would push to >65%: hardware demo where flight signature correctly diagnoses FRAGILE,
    retuning to Kp=80 succeeds, and a second EASY rocket shows RMS < 7.6° at Kp=2.

What STS judges want to hear: the negative result journey (4 hypotheses, 3 were wrong, found bugs,
fixed them). That scientific integrity story is rarer and more compelling than a clean positive result.
Frame as: "I built a TVC rocket. My first three hypotheses were wrong. Here is exactly why, how I
found out, and what the real answer is."

# Flight Validation Plan

Priority (updated 2026-06-07):

1. PRIMARY: Kp=2 detection experiment — confirm RMS > 7.6° threshold predicts FRAGILE
   Test: fly high-ratio design at Kp=2 in moderate wind → expect RMS > 7.6°, poor SR
   Test: fly low-ratio (EASY) design at Kp=2 → expect RMS < 7.6°, high SR
   Simulator prediction: mean RMS 11.3° for FRAGILE vs 3.9° for EASY at Kp=2
   This validates FINDING 5 and is the most actionable new result.

2. Kp=simple vs Kp=full performance — confirm 56% false rejection prediction
   Test: fly same high-ratio design with Kp_full → expect dramatic improvement
   Simulator prediction: SR goes from 0.417 to 0.938 for FRAGILE designs
   This validates FINDING 4 (S2R) and is the central mechanism claim.

3. Authority/Iyy ratio threshold — confirm theta_ddot_max > 70 rad/s² predicts gain sensitivity
   Compare high-ratio vs low-ratio hardware configurations
   This validates FINDING 2 (AUC=0.855 formula from specs alone).

4. Regime boundary validation — confirm FRAGILE vs EASY classification in hardware.

5. Sensor fidelity effects (lower priority; mechanism experiment must be clean first).

Do not assume simulator correctness.
Flight data has higher evidential value than simulation results.

COMBINED WORKFLOW (sim-to-real + flight detection):
  Pre-flight: compute theta_ddot_max from specs. If > 70 rad/s², flag as gain-sensitive.
  Test flight: fly at Kp=2. If RMS > 7.6°, confirmed FRAGILE → switch to Kp=40-80.
  If RMS < 7.6°, probably EASY — Kp=2 is sufficient or small increase suffices.
  This eliminates need to fly at every Kp value to find the right gain.

---

# Response Style

Explain concepts clearly.

Assume the researcher is still learning advanced experimental design and statistics.

When identifying flaws:

* explain why they matter
* explain their practical impact
* suggest possible fixes

Avoid unnecessary jargon.

Teach while critiquing.
