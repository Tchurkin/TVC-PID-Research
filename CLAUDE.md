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

# MAJOR CORRECTION (2026-06-15): 3-Seed Classification Noise in Exp1

⚠️ The n=2400 Exp1 regime counts (EASY=2347, MARGINAL=5, FRAGILE=45, INFEASIBLE=3) used a
3-seed binary robustness test. With exactly 3 binary seeds, SR can only equal {0, 1/3, 2/3, 1}.
The EASY_ROBUSTNESS threshold (0.80) sits between 2/3 (0.667) and 1.0 — meaning the test
cannot reliably distinguish a truly-robust design (true p_success ~0.85-0.95) from a truly-
fragile one (true p_success ~0.5-0.7); both commonly land on SR=2/3 by chance.

DIAGNOSTIC: 43/45 (95.6%) of original FRAGILE labels were "borderline" — one seed-flip away
from a different classification (under_sr or over_sr in {1/3, 2/3} rather than {0,1}).

CORRECTION METHOD (tools/exp1_reclassify_15seed.py): re-evaluated all FRAGILE+MARGINAL+
INFEASIBLE (53) plus all EASY with theta_ddot_max > 70 rad/s² (196) — 249 designs total —
with 15 FRESH seeds (101-115, disjoint from original 1-3), gains FROZEN (not re-tuned, to
isolate classification noise from gain-selection variance).

CORRECTED POPULATION (n=2400):
| Regime     | Original (3-seed) | Corrected (15-seed) |
|------------|--------------------|-----------------------|
| EASY       | 2347               | 2365                  |
| FRAGILE    | 45                 | 30 (-33%)              |
| MARGINAL   | 5                  | 2 (-60%, effectively dissolved) |
| INFEASIBLE | 3                  | 3 (net unchanged, 2 swapped)|

KEY RESULT: AUC(theta_ddot_max predicting FRAGILE) IMPROVED with correction: 0.943 → 0.957.
Removing noise-driven false-FRAGILE labels tightened the true population around its physical
signal — confirmatory, not just a fix. 60% of original FRAGILE flipped (25/27 to EASY, 2 to
INFEASIBLE). MARGINAL DISSOLVED: all 5 original MARGINAL flipped to EASY — the "wind-limited
MARGINAL regime" described elsewhere in this file was very likely a 3-seed sampling artifact,
not a real third regime. A small number of high-td EASY designs (10/196 = 5.1%) flipped TO
FRAGILE — these are the genuine false negatives the original 3-seed test missed.

CAVEAT: first-order correction (gains frozen, not re-tuned) on the at-risk + high-td-EASY
subset only, not a full n=2400 rerun. Low-td EASY designs were not re-checked (boundary v2
showed wide-margin windows there, so 3-seed noise is not a practical risk).

IMPLICATIONS:
- All downstream statistics computed on the OLD n=45 FRAGILE labels (Cohen's d, K_u
  separation, flight detection thresholds, relay study correlations, the combined
  theta_ddot×latency AUC=0.972) are POTENTIALLY STALE pending re-derivation on the corrected
  n=30 population. Treat them as approximately right in direction, not exact, until re-run.
- MARGINAL should be DROPPED from the taxonomy going forward — treat the scheme as 3-class
  (EASY/FRAGILE/INFEASIBLE), pending confirmation MARGINAL doesn't reappear with a larger LHS.
- The boundary v2 "EASY/FRAGILE converge at theta_ddot>100" finding is PARTLY a noise artifact:
  some high-td FRAGILE designs used in that study were themselves noise-driven mislabels
  (60.9% flip rate among FRAGILE with td>100, vs 10.0% for EASY — see
  tools/high_td_seed_stability.py). The qualitative "windows narrow for both groups at high
  td" conclusion may still hold, but needs re-verification with corrected labels.
- Lesson for future experiment design: a binary pass/fail criterion with n seeds cannot
  resolve probabilities finer than 1/n. 3-seed tests are categorically too coarse for a
  threshold (0.80) that sits between two of the four possible step values. Future robustness
  tests should use ≥7 seeds, or report continuous SR with a confidence interval.

Files: tools/exp1_reclassify_15seed.py, tools/high_td_seed_stability.py,
experiments/results/exp1_reclassify_15seed_py.csv, experiments/results/exp1_corrected_population_py.csv.

---

# MAJOR CORRECTION v2 (2026-06-15): Gain-Search Optimality Was ALSO a Confound

⚠️ The 15-seed correction above (v1) froze the ORIGINAL autotune_continuous gains and only
re-evaluated classification noise. tools/gain_search_optimality_audit.py then discovered the
gain search itself has structural blind spots: Kd is decoupled from Kp (probed once at a fixed
reference Kp=40, then frozen while Kp is searched alone), and the Kp grid (10 coarse points,
~2.16× step) is coarser than some documented gain windows (1.5–3.3×). Re-evaluating the v1
"corrected" population's frozen gains with a FRESH disjoint seed batch (7 seeds, 301-307) flipped
56.7% of FRAGILE, 100% of MARGINAL, and 100% of INFEASIBLE labels — i.e., v1 was itself still
underpowered, now at the 0.35 (INFEASIBLE) threshold as well as the 0.80 (robustness) threshold.

CONCRETE PROOF CASE: R0475, labeled INFEASIBLE in BOTH the original 3-seed run and the v1
15-seed correction (frozen gains Kp=155.45, Kd=1.0, true SR≈0.40 over 40 seeds at those gains)
— turned out to simply have bad gains. With a finer joint Kp×Kd search, true nominal SR=0.90.
The design was never physically uncontrollable; the search never found its real optimum.

CORRECTION METHOD v2 (tools/exp1_final_correction.py): for the 241 designs most at risk (35
at-risk from v1 [FRAGILE+MARGINAL+INFEASIBLE] + 206 EASY with theta_ddot_max > 70 rad/s²):
  1. Re-ran gain selection with a FINER JOINT Kp×Kd grid (18×7=126 combos, 3-seed SR/RMS select)
     instead of the decoupled autotune_continuous search.
  2. Evaluated nominal/under/over robustness with 30 FRESH seeds (1001-1030, disjoint from all
     prior seed ranges) using the new gains.
  3. Computed a Wilson 95% CI on each success rate; flagged a design "uncertain" if its CI still
     straddles the relevant decision threshold (0.35 nominal, 0.80 under/over) even at n=30.

CORRECTED POPULATION v2 (n=2400):
| Regime     | v1 (15-seed, frozen gains) | v2 (finer search + 30 seeds) |
|------------|------------------------------|---------------------------------|
| EASY       | 2365                         | 2362                            |
| FRAGILE    | 30                           | 36                              |
| MARGINAL   | 2                            | 0 (fully dissolved)              |
| INFEASIBLE | 3                            | 2                                |

KEY RESULT: AUC(theta_ddot_max → FRAGILE) IMPROVED AGAIN: 0.957 → 0.975. Cohen's d jumped
1.74 → 3.71 (t=11.2, p=3.6e-13). This is the third consecutive correction pass where fixing a
genuine statistical/search flaw made the central finding STRONGER, not weaker — strong evidence
the theta_ddot_max signal is real and the noise was genuinely noise, not signal being thrown out.

POPULATION CHURN: only 15/30 v1-FRAGILE designs remained FRAGILE in v2. 15 flipped to EASY/
INFEASIBLE (median td≈97, i.e. the lower-td half of v1 FRAGILE). 21 NEW designs entered FRAGILE
from the high-td EASY pool (median td≈149) — these were genuine false negatives in v1 that a
better search exposed. Net effect: the FRAGILE population's mean theta_ddot_max rose sharply
(124.5 → 168.2 rad/s²) because the correction disproportionately removed low-severity members
and added high-severity ones that v1's coarser search had missed.

CONTINUUM CONFIRMATION: FRAGILE min td (58.8) is still far below several EASY designs' td
(top EASY td values reach 411.8, 376.6, 334.8 — all above the FRAGILE minimum). No clean
separating value exists; this is additional direct evidence for the FRAGILE-as-continuum
view already noted in the paper (Section 7.4) — theta_ddot_max predicts the DEGREE of gain-
window narrowing, and the binary label is a thresholded view of a continuous quantity.

UNCERTAINTY THAT SURVIVES 30 SEEDS: 81/241 re-examined designs (33.6%) remain "uncertain" —
their CI still straddles a decision threshold even at n=30. Of these, 70/241 (29%) are
uncertain specifically on the OVER (ceiling) test at the 0.80 threshold, vs only 20/241 on
UNDER and 2/241 on NOMINAL — confirming ceiling-limited behavior (not floor/wind) is the
dominant source of irreducible classification noise. ALL 81 uncertain designs have
theta_ddot_max ≥ 55.05 rad/s² (mean 158.5) — exactly the Youden-J boundary region (54.8
rad/s²) identified by the spec formula. This means: for designs below ~55 rad/s², the
EASY/FRAGILE call is statistically solid; above it, a meaningful fraction of individual
designs may be inherently near a 50/50 coin flip under any realistic seed budget, and should
be reported as "elevated risk, indeterminate severity" rather than forced into one bin.

IMPLICATIONS:
- This is now the DEFINITIVE population for any new analysis: EASY=2362, FRAGILE=36,
  INFEASIBLE=2, MARGINAL=0 (n=2400). Use exp1_final_population_py.csv going forward.
- AUC=0.975, Cohen's d=3.71 are the new headline numbers, superseding v1's 0.957/1.74.
- Downstream stats computed on the v1 n=30 FRAGILE population (K_u separation, flight
  detection thresholds, the n=45/n=30-based S2R and flight-signature tables elsewhere in this
  file) are now STALE AGAIN pending re-derivation on the n=36 v2 population.
- The "uncertain" flag (81 designs) should be reported explicitly wherever individual design
  classifications are cited — do not present FRAGILE/EASY as a clean binary for designs in
  the 55-200 rad/s² range without the caveat that some fraction are statistically irreducible.
- Lesson: gain-search adequacy is a confound just like seed count. Any future robustness study
  must treat "did we find the true optimal gains" and "did we sample enough seeds" as two
  separate sources of classification noise, both of which were independently found to bias
  this population toward false EASY/false INFEASIBLE in earlier passes.

Files: tools/gain_search_optimality_audit.py, tools/exp1_final_correction.py,
experiments/results/gain_search_optimality_audit_py.csv, experiments/results/exp1_final_correction_py.csv,
experiments/results/exp1_final_population_py.csv.

FLIGHT SIGNATURE RE-DERIVATION ON FINAL POPULATION (2026-06-15, tools/flight_sig_rerun_final.py):
Re-ran the Kp=2 flight-detection test (7 seeds, full physics) on the final n=36 FRAGILE +
n=36 stratified-by-td EASY population (flight_sig_final_py.csv, 504 rows). Result:
  AUC(RMS, 7-seed): 0.954 [0.907, 0.989]  (was 0.943 [0.888, 0.984] on stale n=45 population)
  AUC(theta_ddot alone, same 72 designs): 0.962  (flight detection no longer beats spec-alone)
  Class separation: FRAGILE RMS=13.3°±5.2° vs EASY RMS=3.8°±2.7° — ratio 3.53× (was 2.90×)
  Best threshold by F1: 7.6° (prec=0.91, rec=0.86, F1=0.89) — SAME threshold as before, still
  optimal; Youden-J lands at 8.0° (TPR=0.86, FPR=0.08), 6.0° gives best recall (0.94, F1=0.88).
  AUC(1-seed): 0.921 (was 0.853 on old population — single-flight detection got more reliable
  because the new FRAGILE set is more severe on average, td mean 168.2 vs 124.5 rad/s²).
CONCLUSION: flight detection finding is CONFIRMED and slightly strengthened on the final
population. Recommended workflow and 7.6°/6.0° thresholds are unchanged from before.
Use flight_sig_final_py.csv and these numbers going forward; flight_sig_updated_py.csv (n=45) is superseded.

---

WINDOW-RATIO FORMULA: NEGATIVE RESULT (2026-06-15, tools/window_formula_validation.py)
The combined ceiling×floor window formula (K_u≈380/latency; Kp_floor≈0.35×keff^0.70), each
separately fit in earlier studies, was validated against the held-out gain_window_v2_summary
dataset (180 rows, never used to fit either piece). Result: ceiling alone has moderate support
(log-log R²=0.474); floor alone FAILS (R²=−0.003, worse than predicting the mean); combined
formula's AUC for ranking FRAGILE vs EASY by predicted window = 0.500 (chance level), despite
the ground-truth window itself achieving AUC=0.577. Formula systematically overpredicts
FRAGILE windows (mean residual −0.291) and underpredicts EASY windows (+0.196) — an unmodeled
regime-dependent floor-elevation effect. CONCLUSION: do not cite the combined window-ratio
formula as predictive; it was never validated as a unit and does not survive validation now
that it has been tested. The component formulas (ceiling, floor) retain their original
individual support levels documented elsewhere in this file.

---

# NEW (2026-06-16): Continuous reframing, builder tool, and cross-architecture design rule

Triggered by a user critique: the binary FRAGILE/EASY framing invites two valid objections —
(1) AUC on a ~1.5% base rate is easy to mistake for "accuracy" (a trivial always-EASY classifier
gets ~98.5% accuracy and is useless); (2) the project's own boundary experiment and the "uncertain"
Wilson-CI flag (33.6% of at-risk designs) already suggested FRAGILE/EASY is a thresholded
continuum, not two natural classes — so why lead with a classifier at all? Three new experiments
address this directly with fresh data, not just reframed prose.

## Continuous margin regression (tools/continuous_margin_regression.py + tools/elbow_characterization.py)

Re-ran the central question as a CONTINUOUS regression instead of a classification. Method: for
each design, run the same finer joint Kp×Kd search used in the final population correction
(Section "MAJOR CORRECTION v2"), then evaluate over-robustness success rate (15 fresh seeds,
continuous fraction in [0,1], not binarized at 0.80) — the dominant failure mode per the final
population (~87% of FRAGILE designs are ceiling-limited). Two passes: (1) n=130, designs
stratified by theta_ddot_max decile from a 4000-design LHS pool — found a LOW R²=0.075 when fit
naively because the decile stratification left only 8 of 130 points above the danger zone
(td>90), giving a misleadingly flat/noisy slope; (2) a follow-up, n=92, specifically targeting
5 bins across td∈[40,1000] with 20 designs per bin (proper power in the region that matters).

**Combined n=222 dataset (`experiments/results/combined_margin_regression_py.csv`):**
r(log td, over_sr) = -0.421 (p=5.8e-11). Binned means show a genuinely smooth, monotonic
dose-response curve, NOT a sharp step: td<40 → mean over_sr=0.980 (n=103); 40-80 → 0.985 (n=36);
80-120 → 0.953 (n=24); 120-180 → 0.875 (n=23, 26% below 0.80); 180-300 → 0.806 (n=24, 42% below
0.80); >300 → 0.783 (n=12, 33% below 0.80). Linear-in-log-td OLS: R²=0.177 alone, **R²=0.274 with
log(latency) added** (coefs: intercept=1.168, log_td=-0.0396, log_lat=-0.0720 — both negative,
consistent with established mechanism). **5-fold CV: R²=0.288±0.103, MAE=0.070±0.011** — close to
in-sample (0.321/0.069), so this is not badly overfit on the combined, properly-stratified set.

**Conclusion: the continuum framing is correct and now has real, well-powered, continuous data
behind it** (n=222, not the n=12 boundary-experiment sample that produced a weak/negative result
previously). theta_ddot_max + latency predicts a continuous degree of gain-margin loss with a
modest but real and statistically robust R²≈0.27-0.29 — this is the right primary framing,
methodologically defensible against the AUC/base-rate objection, and should supersede the binary
classifier as the paper's headline claim. The binary FRAGILE/EASY label (AUC=0.975) is retained
as a secondary, simplified decision rule for builders who want a single threshold, not as
evidence of two natural rocket populations.

**Caveat:** the n=130 decile-stratified pass independently confirms something useful: below
td≈90 rad/s², with a properly-executed (finer, joint) gain search, margin is essentially perfect
(mean 0.98, only minor 15-seed-noise blips) REGARDLESS of latency up to 6 steps — including at
combinations (e.g. td≈15-55, latency=6) that the ORIGINAL coarser autotune_continuous search had
flagged as the "MILD FRAGILE / latency-driven false-negative zone" (D800, D1523, Section 4.5).
This raises the possibility that those two false negatives were also partly a search-quality
artifact, not a pure latency effect — consistent with this project's repeated finding that a
better gain search shrinks, rather than grows, the gain-sensitive population. Not yet confirmed
on those exact two designs; flagged as a follow-up.

## ADRC bandwidth ceiling — a genuinely novel cross-architecture design rule (tools/adrc_bandwidth_ceiling.py)

PID has an established gain ceiling law, Kp_max ≈ 380/latency_steps (the "GAIN CEILING EQUATION"
section below). ADRC's closed-loop bandwidth omega_c plays the same structural role (how fast the
controller reacts), and the ADRC dissolution test (Finding 7) already showed its 2 residual
failures were fixed by LOWERING omega_c for high-latency designs — same delay mechanism. This
experiment quantifies that: 3 reference designs at theta_ddot_max ≈ 20 / 90 / 200 rad/s²,
latency_steps ∈ {1,2,3,4,6,8,10,12} (extended past 6 to the Arduino-class range), omega_c swept
over a log grid with omega0 = 5×omega_c fixed, 10 seeds, full physics with wind. For each cell,
found the largest omega_c achieving success rate ≥ 0.80 — the ADRC ceiling, exactly analogous to
PID's Kp_max.

**ORIGINAL RESULT (n=21, 3 reference designs): omega_c_max ≈ 645 / (latency_steps^0.66 ×
theta_ddot_max^0.77), R²=0.936.**

⚠️ REVISED 2026-06-17 (tools/adrc_ceiling_extended.py, n=46 cells, 6 authority levels):
omega_c_max ≈ 70 / (latency_steps^0.57 × theta_ddot_max^0.31), R²=0.823.
The n=21 authority exponent (-0.77) DID NOT REPLICATE with 6 reference designs. The original
3-design sample was confounding between-design hardware variation with the authority effect.

CORRECTED CHARACTERIZATION: ADRC's ceiling is primarily latency-governed (exponent -0.57) with
a secondary authority effect (exponent -0.31). The authority exponent (-0.31) is 1.5× PID's
keff exponent (-0.20) — real but moderate coupling, NOT "comparable in strength to latency."
The qualitative structural difference from PID survives: ADRC shows more authority dependence
than PID, but latency dominates both. The "most genuinely novel" framing was overconfident.

Files: tools/adrc_bandwidth_ceiling.py (original, n=21),
tools/adrc_ceiling_extended.py (n=46, 6 td levels, corrected formula),
experiments/results/adrc_bandwidth_ceiling_summary_py.csv,
experiments/results/adrc_ceiling_extended_py.csv,
experiments/results/adrc_ceiling_extended_summary_py.csv.

## Builder tool: tools/gain_advisor.py

A runnable CLI/library tool that takes hardware specs (thrust, max_gimbal_deg, Iyy, l_nozzle,
latency_steps) and outputs: theta_ddot_max, a CONTINUOUS predicted gain-margin score (from the
n=222 regression above, not a binary label), a risk tier (5 levels, not GO/NOGO), a recommended
PID Kp range [floor, ceiling], and a recommended ADRC omega_c ceiling — i.e. actionable numbers
for whichever architecture the builder picks, not just a classification. Every formula's source
experiment and evidence strength is documented inline in the file (PROVENANCE block) so a
skeptical reader can independently check what backs each number. This is intended to replace
"compute theta_ddot_max and compare to a threshold" as the project's practical deliverable.

Files: tools/continuous_margin_regression.py, tools/elbow_characterization.py,
tools/adrc_bandwidth_ceiling.py, tools/gain_advisor.py,
experiments/results/combined_margin_regression_py.csv,
experiments/results/adrc_bandwidth_ceiling_summary_py.csv.

## Regression extension — plateau above td ≈ 300 (2026-06-16, tools/regression_extension.py)

The original combined n=222 dataset had only n=12 designs above td=300 rad/s² (from a 4000-design
LHS pool where td>300 requires extreme parameter combinations: low Iyy + high motor_scale + high
max_gimbal ≈ 0.3% of the space). This was underpowered for characterizing the high-authority tail.
Extension: 40 new designs with td ∈ [301, 496] rad/s² drawn from a 20,000-design pool (seed=999),
stratified into 4 sub-bins across the td>300 range. Same protocol: finer joint Kp×Kd search
(18×7=126 combos, search seeds 1-3), 15-seed evaluation (seeds 8001-8015, disjoint from all prior
passes), full physics. Saved to regression_extension_py.csv; merged with combined_margin_regression
_py.csv to form regression_pooled_py.csv (n=262).

KEY FINDING — PLATEAU ABOVE td ≈ 300:
  r(log_td, over_sr) = +0.067 (p=0.68) within the extension range — essentially zero.
  Mean over_sr in the extension: 0.768 (vs 0.783 for the original n=12 at td>300 — consistent).
  The dose-response curve does NOT continue declining above td=300. It plateaus.
  Mechanism: Kp_max ≈ 380/latency is the binding hard constraint at this authority level.
  Once the latency-determined ceiling is dominant, adding more authority cannot further compress
  margins — the latency ceiling is already more restrictive than what extra td would shift.
  Within the extension: latency=1 designs achieve SR≈1.00 regardless of td>300;
  latency=5-6 designs average SR≈0.55 — latency, not td, is the discriminating variable.

POOLED STATISTICS (n=262):
  OLS: R²=0.364 in-sample; intercept=1.234, log_td=-0.051, log_latency=-0.101
  Note: |log_latency coef| (0.101) > |log_td coef| (0.051) in the pooled fit — consistent with
  latency being the dominant constraint in the high-authority regime.
  5-fold CV R²: 0.325 ± 0.086 (stable, not overfit).
  These supersede the n=222 numbers (R²=0.274 in-sample, CV R²=0.288±0.103) as the definitive fit.

UPDATED BINNED DOSE-RESPONSE (n=262):
  td<40:      mean over_sr=0.980 (n=103)
  td 40-80:   mean over_sr=0.985 (n=36)
  td 80-120:  mean over_sr=0.953 (n=24)
  td 120-180: mean over_sr=0.875 (n=23, 26% below 0.80)
  td 180-300: mean over_sr=0.806 (n=24, 42% below 0.80)
  td >300:    mean over_sr=0.772 (n=52, 46% below 0.80) — plateau, no further decline vs td 180-300

Files: tools/regression_extension.py, experiments/results/regression_extension_py.csv,
experiments/results/regression_pooled_py.csv.

## Window ratio regression (2026-06-17): direct gain-margin measurement replaces over_sr

Motivation: the n=262 over_sr continuous regression (CV R²=0.325) has two structural problems:
(1) over_sr is right-censored at 1.0 — easy designs all score 0.98-1.00 regardless of window width;
(2) "SR at 1.4× best_Kp" is not what the ceiling/floor theory directly predicts.
Window_ratio = Kp_ceiling / Kp_floor measures the gain margin width the theory actually models.

METHOD (tools/window_ratio_regression.py): 10,000-design LHS pool (seed=8888), stratified
into 6 td bins × 4 latency bins, 5 designs per cell (120 total). Per design: finer joint
Kp×Kd search (18×7 grid, seeds 1-3), then Kp sweep (16 log-spaced [0.5, 480], Kd frozen,
seeds 20001-20015, SR≥0.80 threshold). Floor = min passing Kp, ceiling = max passing Kp.

RESULT v1 (n=120, 16-pt sweep over [0.5, 480]):
  n_valid=113/120, floor_censored=39 (34%), ceil_censored=40 (35%)
  log(keff) + log(latency): R²=0.682  CV=0.656±0.087  coefs=[-1.046×log_keff, -1.710×log_lat]
  log(td × latency) product: R²=0.556  CV=0.498±0.066  coef=-1.079
  IMPROVEMENT: CV R²=0.656 vs baseline over_sr CV R²=0.325 — 2× improvement.

METHOD v2 (tools/window_ratio_resweep_v2.py): same 120 designs, frozen Kd from v1,
32-point sweep over [0.1, 800] (step 1.34× vs 1.58×), fresh seeds 22001-22015.
Censoring dropped to floor=12 (10%), ceiling=23 (20%). Max window_ratio now 8000× (was 960×).

RESULT v2 (n=120, 32-pt sweep over [0.1, 800]):
  n_valid=116/120, floor_censored=12 (10%), ceil_censored=23 (20%)
  log(keff) + log(latency): R²=0.659  CV=0.616±0.080  coefs=[-1.192×log_keff, -1.880×log_lat]
  NOTE: CV R² slightly LOWER than v1 (0.616 vs 0.656) because uncensoring exposed real variance
  among easy designs (windows 1000-8000×) that keff/lat can't fully predict. V2 is a more honest
  estimate; v1 appeared better because easy designs all collapsed to window≈960 in the censored range.

CEILING vs FLOOR DECOMPOSITION (v2, non-censored designs only):
  Ceiling (n=93): ceil ~ keff + lat  R²=0.442  coefs: keff=-0.005 (≈0!), lat=-0.862
  Floor   (n=104): floor ~ keff only  R²=0.419  coefs: keff=+1.068
           floor ~ keff + lat  R²=0.627  CV=0.594±0.081  coefs: keff=+1.061, lat=+0.964

KEY FINDINGS:
1. CEILING IS INDEPENDENT OF KEFF: keff coefficient for ceiling ≈ -0.005 ≈ 0. The ceiling
   depends ONLY on latency. Consistent with K_u_theory ≈ 380/latency (DIPDT, keff-independent
   in the dominant keff×τ << 1 regime). Empirical ceiling exponent on latency: -0.862 (theory: -1.10;
   discrepancy because population mixing of easy/hard designs flattens the effective slope).

2. NEW FINDING — FLOOR ALSO DEPENDS ON LATENCY: The gain floor rises with latency at almost
   the same strength as it rises with keff (exponent +0.964 for lat vs +1.061 for keff). Adding
   latency to the floor model raises CV R² from 0.376 to 0.594. OLD THEORY (floor ~ keff^0.70 only,
   from relay study) was incomplete — latency was not tested. MECHANISM: at high latency, the
   control loop cannot respond to wind perturbations for τ seconds; the rocket accumulates
   angular error before correction acts, requiring higher Kp to maintain SR≥0.80. This is
   analogous to how keff raises the floor (more authority → more aggressive bang-bang response
   → needs more gain to stay within bounds), but through a time-delay rather than an amplitude path.

3. COMBINED LATENCY EXPONENT = -(ceiling) - (floor) = -0.862 - 0.964 = -1.83 (measured: -1.88).
   This is the DOUBLE SQUEEZE: ceiling drops AND floor rises with latency simultaneously.
   The old theory only modeled the ceiling drop (-1.10), giving a window exponent of ~-1.10.
   The corrected combined exponent is ~-1.85, explaining why previous window estimates understated
   the latency sensitivity. Practical implication: each doubling of latency compresses the
   window by 2^1.88 = 3.7× — much more than the 2^1.10 = 2.1× from ceiling-only theory.

4. LATENCY SWEEP CONFIRMATION (tools/latency_ceiling_sweep.py, 3 fixed designs × 6 latency levels):
   Floor exponent on latency: td=50: +0.39 (weak, R²=0.17); td=120: +1.38 (R²=0.73); td=250: +1.46 (R²=0.79)
   Ceiling exponent on latency: td=120: -2.52 (R²=0.92, n=4); td=50: -0.76 (R²=0.64, n=5)
   td=250, latency≥5: NO valid Kp window found (INFEASIBLE at full physics — confirms stress-test finding)
   CAUTION: quantization (16 discrete Kp points) makes these per-design exponents noisy; use
   the population regression exponents as the more robust estimate.

COMBINED FORMULA (updated, 2026-06-20 correction):
  Kp_ceiling(lat) ≈ 380 / lat^1.0   [theoretical; spot-check validated; USE THIS]
    NOTE: regression gives 520/lat^0.86 (R²=0.442, 22% ceiling censoring) — this deviates 14%
    from the theoretical -1.0 exponent due to censoring artifact. The regression overcounts
    easy designs with very high ceilings (>800) that are censored at the tested range, flattening
    the apparent latency slope. The 380/lat formula is validated directly by 5 spot-check
    simulations and by the DIPDT phase-margin derivation; use it instead of 520/lat^0.86.
  Kp_floor(keff, lat) ≈ 0.06 × keff^1.06 × lat^0.96  [from v2 floor regression, R²=0.627]
    NOTE: theoretical prediction is keff^1.0 × lat^1.0 (integers). Measured exponents (1.06, 0.96)
    are within 6% of theoretical; the deviation is measurement noise, not a real physical effect.
    Simplified form: Kp_floor ≈ 0.06 × keff × lat  [theoretical, within regression uncertainty]
  Window(keff, lat) ≈ (380/0.06) × keff^(-1.0) × lat^(-2.0) ≈ 6300 × keff^(-1.0) × lat^(-2.0)
    [theoretical form, consistent with Pi = keff × lat²]
    Empirical regression form: 8700 × keff^(-1.06) × lat^(-1.82)  [for honest uncertainty reporting]
  NOTE: individual design windows vary widely around these formulas (CV R²≈0.59-0.62).

LIMITATION: all analysis uses a single fixed Kd per design during the Kp sweep. True window
  (with optimal Kd at each Kp) would be wider, but the fixed-Kd window correctly represents
  what a builder gets with a single tuned Kd — the practical question.

Files: tools/window_ratio_regression.py, tools/window_ratio_resweep_v2.py,
tools/window_ratio_analysis.py, tools/latency_ceiling_sweep.py,
experiments/results/window_ratio_regression_py.csv, experiments/results/window_ratio_v2_py.csv,
experiments/results/window_ratio_sweep_detail_py.csv, experiments/results/window_ratio_v2_sweep_py.csv,
experiments/results/latency_ceiling_sweep_py.csv.

WINDOW RATIO v3 (2026-06-17, tools/window_ratio_v3_fixed_wind.py):
Same 120 designs + same opt_Kd from v2. Two changes: (1) wind_strength FIXED to 0.25 for all
designs (removes the LHS wind variability that prevents keff/latency from being the sole predictor);
(2) 20 eval seeds instead of 15 (reduces stochastic noise on SR ≈ 18%). Kp range extended at low
end to 0.05 (was 0.1), keeping 32-pt sweep. Seeds 23001-23020.

RESULT v3 (n=120, 32-pt [0.05,800], fixed wind=0.25, 20 seeds):
  n_valid=114/120, floor_censored=16 (14%), ceil_censored=22 (19%)
  log(keff) + log(latency): R²=0.625  CV=0.601±0.066  coefs=[-1.451×log_keff, -1.927×log_lat]
  NOTE: v3 CV (0.601) is NOT better than v2 (0.616) — the difference is within 1 standard deviation
  of both estimates. Fixing wind_strength made NO MEANINGFUL IMPROVEMENT in predictability.

CEILING (n=92 non-censored): keff coeff = -0.019 ≈ 0 (consistent with v2's -0.005, confirms independence)
  Formula: ceiling ≈ 544 / latency^0.89 [keff-independent; same conclusion as v2]
FLOOR (n=98 non-censored): keff exponent=+1.074, lat exponent=+0.978 (R²=0.582, CV=0.446±0.284)
  Formula: floor ≈ 0.060 × keff^1.074 × latency^0.978 [exponents consistent with v2: 1.061, 0.964]
  NOTE: floor CV in v3 (0.446) is WORSE than v2 (0.594) with much larger variance (±0.284 vs ±0.081).
  Cause: fixing wind at 0.25 makes some designs (those with originally high LHS wind) easier,
  pushing their floors below 0.05 → censored out → smaller valid set + bimodal floor distribution.
  V2's floor regression (n=104, variable wind) is the more reliable estimate.

NEGATIVE FINDING: wind_strength is NOT the primary remaining confound.
The ~40% unexplained variance in window_ratio comes from other sources:
  1. Backlash/deadband/slew variation (per-design LHS parameters, not modeled in formula)
  2. Fundamental stochastic noise in SR measurement (20 seeds, ~9% SE)
  3. Per-design variance not captured by the two-parameter keff+latency formula
Conclusion: CV R²≈0.60-0.62 is near the achievable limit for a keff+latency formula on variable
designs. The formula explains what it's supposed to explain; the rest is genuine residual variance.

EXPONENT ROBUSTNESS CONFIRMED: v2 and v3 give nearly identical floor exponents (keff: 1.061 vs
1.074; lat: 0.964 vs 0.978) despite completely different wind conditions. This confirms the formula
is robust — the floor-latency dependency is a real physical effect, not a wind artifact.

Files: tools/window_ratio_v3_fixed_wind.py, experiments/results/window_ratio_v3_py.csv,
experiments/results/window_ratio_v3_sweep_py.csv.

## LATENCY EXTENSION (2026-06-20, tools/window_ratio_lat_extension.py):
Extends window_ratio dataset to latency_steps 7-12 (Arduino-class MCUs: I²C sensors, 50-100 Hz loops).
Protocol: 6 td bins × 4 lat bins (7/8/10/12), 5 per cell; td=[350,600) unpopulated at lat 7-12
-> 100 designs total. Same joint 18x7 Kp×Kd search (seeds 1-3), 32-pt Kp sweep [0.05,400],
15 eval seeds 25001-25015. Saved: window_ratio_lat_ext_py.csv, window_ratio_pooled_py.csv (220 rows).

POOLED REGRESSION (n=148 non-censored, lat 1-12):
  Free fit: keff=-1.491, lat=-1.424 (theory: -1, -2)  CV R2=0.605
  Tau exponent sweep peak at tau^1.0 (CV=0.606) NOT tau^2.0 (CV=0.541) -- apparent shift
  u_max addition: delta CV = -0.003 (confirmed irrelevant at lat 7-12 too)

KEY FINDING -- tau exponent is keff-tier-dependent:
Within the v2 lat 1-6 data (n=82), regressing log(window) vs log(latency) within each keff tier:
  Low keff  (mean=5.4  rad/s2/CU, n~27): lat exponent = -0.85  (1.8x per doubling)
    Floor barely rises with latency for mild bang-bang; only ceiling compresses -> tau^-1 regime.
  Mid keff  (mean=14.2 rad/s2/CU, n~27): lat exponent = -2.23  (4.7x per doubling)
    Both ceiling and floor active; closest to theoretical -2.0 (double squeeze).
  High keff (mean=30.8 rad/s2/CU, n~28): lat exponent = -3.19  (9.1x per doubling)
    Violent bang-bang amplifies floor-latency coupling beyond linear model.

SELECTION ARTIFACT (explains pooled tau^1.0 peak):
  5/100 lat 7-12 designs have no valid window; ALL 5 are high-keff (keff=20-26).
  Mean keff drops from ~15 at lat 1-6 to ~11 at lat=12 -- pool is biased low-keff at high lat.
  Low-keff designs (tau^-0.85) dominate lat 1-12 pooled data -> pooled exponent shifts to ~-1.4.
  The physics has not changed; the surviving population has changed.

PREDICTED vs OBSERVED window (formula 6300/keff×lat²) at lat 7-12:
  lat=7:  median obs/pred ~ 2.96  (formula predicts ~3x tighter than observed)
  lat=8:  median obs/pred ~ 2.96
  lat=10: median obs/pred ~ 2.74
  lat=12: median obs/pred ~ 3.03
  Formula overpredicts compression for surviving low-keff designs at extreme latency.

PRACTICAL IMPLICATIONS:
  "3.7x per doubling" (tau^-1.88 from v2 lat 1-6) is a population average.
  Narrow-window designs (keff >= 10): sensitivity is 4.7-9.1x per doubling -- WORSE than avg.
  Wide-window designs (keff <= 5): sensitivity is only 1.8x per doubling -- safer than avg.
  The tau^2 theory applies best in the moderate-keff regime (keff ~14 rad/s2/CU).
  The formula window ~ 6300/keff×lat^2 is conservative for Arduino-class LOW-keff designs
  (they have wider windows than predicted), approximately correct for moderate-to-high keff
  (the safety-critical case). Do not cite the pooled tau^1.0 exponent from lat 1-12 data
  without noting the selection artifact.

Files: tools/window_ratio_lat_extension.py,
experiments/results/window_ratio_lat_ext_py.csv,
experiments/results/window_ratio_pooled_py.csv.

## NEW (2026-06-20): Dual-regime bang-bang; exhaustive regression confirms stochastic residual

DUAL-REGIME BANG-BANG (tools/bang_bang_transition.py, 2026-06-20):
Kp sweep with simultaneous slew_sat_frac measurement, 3 designs at latency=3, td≈[22, 80, 208]
rad/s², 24 log-spaced Kp points from floor/4 to 2.5×ceiling, 5 seeds each.

KEY FINDING — TWO PHYSICALLY DISTINCT OPERATING REGIMES:
  Low-authority (td≈22): slew_frac≈0.06 throughout entire gain window (floor to ceiling).
    Servo NOT saturated during normal attitude hold; system operates in linear proportional regime.
    Bang-bang onset visible as sharp SR drop at Kp_ceiling with simultaneous slew jump.
    The gain ceiling here is EXACTLY the DIPDT phase-margin mechanism (linear instability at high Kp).
    Floor is low (≈1-3) because Kp=1 suffices to reject wind with linear proportional control.

  High-authority (td≈208): slew_frac≈0.63 AT EVERY KP, including at values BELOW the gain floor.
    Wind disturbances alone drive permanent servo saturation regardless of Kp choice — the rocket
    is already in bang-bang from wind at every gain setting tested (floor/4 through 2.5×ceiling).
    The gain window is NOT "the range before bang-bang starts" — it is the range of Kp where
    wind-driven bang-bang oscillation amplitude stays within the success criterion (RMS < 15°).

IMPLICATION FOR FLOOR MECHANISM (confirms floor ≈ 0.06 × keff × latency):
  High-authority designs need higher Kp because an EXISTING bang-bang amplitude must be damped.
  Floor rises with keff because: higher keff → more aggressive bang-bang response to wind →
    larger oscillation amplitude at any Kp → need more Kp to damp it below threshold.
  Floor rises with latency because: higher latency → rocket accumulates angular error for τ sec
    before correction → larger oscillation amplitude at any Kp → same damping requirement.
  Both paths increase the bang-bang amplitude that Kp must actively suppress — hence keff×latency,
  not keff alone (as the pre-v2 formula keff^0.70 incorrectly suggested).

This is the mechanistic validation of the floor-latency finding from window_ratio regression v2.
It shows the mechanism is not just an empirical fit (exponents 1.06, 0.96 close to integers) but
reflects real physics visible in individual flight traces: slew_frac=0.63 flat across all Kp.

Files: tools/bang_bang_transition.py, experiments/results/bang_bang_transition_py.csv,
outputs/bang_bang_transition.html.

EXHAUSTIVE REGRESSION — RESIDUAL IS STOCHASTIC (tools/window_ratio_full_regression.py, 2026-06-20):
Tested every available design parameter as an addition to the baseline log(keff)+log(latency) model
(n=82 non-censored, 5-fold CV). Results:
  log(wind_strength): +0.013 (best single addition)
  static_margin: +0.012
  log(Iyy), log(motor_scale), log(servo_slew), log(u_max), log(max_gimbal): -0.003 to 0.000
  Full 9-predictor model: CV R² = 0.536 (WORSE than baseline 0.602, from overfitting)
  Within-cell variance (same keff×latency tier): 1.32 vs unexplained residual 1.40 — essentially equal.
CONCLUSION: The ~40% unexplained variance is stochastic (seed noise, per-design backlash/deadband
  variation, 32-point Kp sweep discretization), NOT a missing predictor. keff+latency is the
  complete parametric formula; no design factor meaningfully improves it.
  log(max_gimbal) specifically: delta CV = -0.003 (noise). Builder does NOT need to measure gimbal
  travel to assess gain-window risk.

Files: tools/window_ratio_full_regression.py. Data: experiments/results/window_ratio_v2_py.csv.

## NEW (2026-06-17): LQR controller-invariance test

LQR CONTROLLER-INVARIANCE (tools/lqr_controller_test.py, 2026-06-17):
50 designs stratified by theta_ddot_max (5 bins x 10 designs) from the final corrected
population. Discrete LQR gains via DARE at 40 Q/R ratios (0.001 to 100000, log-spaced).
Window metric: n_pass_qr = number of Q/R ratios achieving SR >= 0.80 over 10 full-physics
seeds (5001-5010, disjoint from all prior experiments).

KEY RESULT: Spearman rho(Pi = td * latency^2, n_pass_qr) = -0.747, p = 4.8e-10.
The combined dimensionless Pi predicts LQR gain feasibility stronger than td alone (-0.631).

ZERO-WINDOW DESIGNS (n=2):
  R0475 (td=193, lat=5): peak_sr=0.50 at ALL 40 Q/R values -- no valid LQR setting exists
  R2080 (td=312, lat=5): peak_sr=0.70 at ALL 40 Q/R values -- no valid LQR setting exists
  R0303 (td=218, lat=6): exactly 1 valid Q/R (window_qr = 1.0) -- single point, no margin

WINDOW BY TD x LATENCY (median n_pass_qr out of 40 tested):
  td<40: lat 1-2 = 40, lat 3-4 = 27, lat 5-6 = 29
  td 40-120: lat 1-2 = 28, lat 3-4 = 17, lat 5-6 = 15
  td 120-200: lat 1-2 = 21, lat 3-4 = 13, lat 5-6 = 5
  td >200: lat 1-2 = 21, lat 3-4 = 13, lat 5-6 = 1

INTERPRETATION: Window compression is NOT a PID parameterization artifact.
LQR (optimal linear state-feedback, no heuristic) faces the same Pi-driven constraint.
Upgrades the ADRC "provisional" confidence tier to Moderate: linear feedback cannot escape
the delay x authority constraint regardless of parameterization.

Files: tools/lqr_controller_test.py, experiments/results/lqr_controller_test_py.csv,
experiments/results/lqr_gain_sweep_py.csv.

## NEW (2026-06-17): SMC controller-invariance test

SMC CONTROLLER-INVARIANCE (tools/smc_controller_test.py, 2026-06-17):
Same 50 designs as LQR test. SMC with boundary layer is implemented as PID with Kd = Kp/lambda_s
(exactly equivalent in the linear region). Swept LAMBDA_S_GRID = [0.2, 0.5, 1.0, 2.0, 5.0, 10.0]
x KP_GRID = 20 log-spaced values [1, 400] = 120 combos per design. 10 seeds (6001-6010, disjoint).
n_pass_total = combos achieving SR >= 0.80; n_pass_best_lam = best per-lambda_s slice (out of 20).

KEY RESULT:
  Spearman rho(Pi, n_pass_total)    = -0.753, p = 2.82e-10
  Spearman rho(Pi, n_pass_best_lam) = -0.789, p = 9.62e-12
  LQR comparison: rho(Pi, n_pass_qr) = -0.747 — INDISTINGUISHABLE from SMC.

ZERO-WINDOW DESIGNS: None. Every design found at least one valid (lambda_s, Kp) pair.
  The 2 LQR zero-window designs (R0475, R2080) do find valid SMC settings — SMC's 6 lambda_s
  values provide more flexibility than LQR's single Q/R-constrained Kd/Kp ratio, so the grid
  search happens to intersect the narrow valid region. This is a search-space coverage effect,
  not an architectural difference. The designs are near the window edge for both architectures.

WINDOW BY TD x LATENCY (median n_pass_best_lam out of 20):
  td<40, lat 1-2: 20; lat 3-4: 20; lat 5-6: 19.5
  td 40-80, lat 1-2: 20; lat 3-4: 19
  td 80-120, lat 1-2: 20; lat 3-4: 12; lat 5-6: 10
  td 120-200, lat 1-2: 18; lat 3-4: 8.5; lat 5-6: 7.0
  td >200, lat 1-2: 17.5; lat 3-4: 14; lat 5-6: 7.0

INTERPRETATION: Controller invariance is now confirmed across THREE architectures (PID, LQR, SMC).
All three have rho(Pi, window_metric) ≈ -0.75. None escapes the Pi-driven constraint. The common
element: all three are open-loop-style feedback WITHOUT active disturbance estimation. ADRC's ESO
is the specific mechanism that decouples wind rejection from gain selection; the constraint is not
a property of any particular parameterization but of linear feedback under delay without an observer.

COMBINED RESULT (three architectures):
  PID window_ratio: Spearman rho(Pi, log_window_ratio) ≈ -0.78 (n=82 window-ratio dataset)
  LQR n_pass:       rho(Pi, n_pass_qr)   = -0.747 (n=50)
  SMC n_pass_total: rho(Pi, n_pass_total) = -0.753 (n=50)
  SMC n_pass_best:  rho(Pi, n_pass_best)  = -0.789 (n=50)
All consistent at rho ≈ -0.75. This is the architecture-invariant signature of the Pi constraint.

Files: tools/smc_controller_test.py, experiments/results/smc_controller_test_py.csv,
experiments/results/smc_sweep_detail_py.csv.

## NEW (2026-06-20): Observer universality — is ESO specifically what breaks the Pi constraint?

OBSERVER UNIVERSALITY (tools/observer_universality_test.py, 2026-06-20):
Research question: PID/LQR/SMC all face rho(Pi, window) ≈ -0.75 with no disturbance estimation.
ADRC with ESO achieves weaker dependence. Does classical integral PID (the simplest "disturbance
estimator") also break the constraint, or is ESO uniquely different?

Same 50 designs as LQR/SMC test. Seeds 11001-11010. Three architectures:
  A. PD (Ki=0):  12 Kp values [1..400] × best_Kd
  B. PID-I:      same 12 Kp × 5 Ki values ∈ {0, 0.05, 0.2, 1.0, 4.0} = 60 combos
  C. ADRC:       omega_c ∈ {1.5, 2, 3, 5, 7, 10, 15}, omega0=5×omega_c, b0=keff (7 values)

⚠️ Pi = keff × lat² (corrected formula) used throughout. The CSV was re-saved with corrected Pi.
    Pi range: 0.8 – 773.5 (using keff × lat²; was 4.4–7830.5 with old td × lat²).

RESULTS — Spearman rho(log_Pi_keff, peak achievable SR):
  PD (Ki=0):   rho = -0.594, p = 5.51e-06  [strongly Pi-dependent]
  PID-I:       rho = -0.488, p = 3.29e-04  [Pi-dependent; integral helps marginally]
  ADRC (ESO):  rho = -0.283, p = 4.65e-02  [weak but SIGNIFICANT dependence — see note]
  NOTE: With the corrected Pi_keff, ADRC's peak_SR rho is now marginally significant (p=0.046),
  unlike with the old Pi_td (p=0.15). ADRC does NOT achieve completely Pi-independent success —
  it reduces the sensitivity but does not eliminate it at a fixed omega0/omega_c=5 ratio.

RESULTS — Spearman rho(log_Pi_keff, n_pass fraction):
  PD:    rho = -0.749, p = 4.12e-10  [window narrows strongly with Pi]
  PID-I: rho = -0.786, p = 1.41e-11  [integral actually makes window NARROWER — see below]
  ADRC:  rho = -0.830, p = 8.95e-14  [ADRC most negative n_pass rho]
  COUNTERINTUITIVE: ADRC has the most negative n_pass rho. At high Pi, only 1-2 of 7 omega_c
  values work — the ADRC window is very narrow. But those 1-2 achieve SR=1.00. Pi narrows the
  ADRC window without eliminating it; Pi eliminates PD/PID-I's window entirely.

HIGH-PI COMPARISON (decisive cases):
  Design         Pi_keff  PD    PID-I  ADRC
  R0236 (td=169, lat=6)  773.5  0.80  1.00   1.00   ← integral HELPS here
  R2080 (td=312, lat=5)  740.2  0.70  0.70   1.00   ← integral FAILS; ESO succeeds
  R0303 (td=218, lat=6)  768.4  0.90  0.90   1.00   ← integral no improvement; ESO succeeds
  R0117 (td=37.8, lat=6) 525.1  0.90  1.00   0.80   ← ADRC underperforms (grid miss, lat=6 edge)

R2080 is the decisive case: PID-I has ZERO valid (Kp, Ki) combos (n_pass_pidi=0), while ADRC
finds exactly 1 valid omega_c achieving SR=1.00. This is not a search artifact — it is a
genuine architectural difference.

CONCLUSION:
1. Integral action PARTIALLY helps at intermediate Pi: R0236 improved 0.80→1.00. 2 designs total.
2. At extreme Pi (>700 on keff scale), integral FAILS same as PD. R2080: n_pass_pidi=0.
3. ESO is architecturally distinct from integral — not "any disturbance estimator" works.
   Delta_rho(PD→PID-I) = +0.106 on peak_SR; delta_rho(PD→ADRC) = +0.311. ESO 3× more effective.
4. ADRC residual Pi dependence (rho=-0.283, now significant) means a FIXED omega0/omega_c=5 is
   not universally optimal — designs at extreme Pi need omega0/omega_c=12-20 (adrc_frontier_extension).

PHYSICAL MECHANISM — why integral fails where ESO succeeds:
  Integral: accumulates error × time to estimate disturbance → corrects AFTER error appears →
    anti-windup blocks accumulation when already saturated → cannot cancel disturbance in saturation
  ESO:     estimates disturbance from STATE DERIVATIVE mismatch (z3 ← state trajectory) →
    subtracts from u0 = (Kp*e - Kd*z2 - z3)/b0 BEFORE saturation clip → cancellation happens
    upstream of the servo limit → servo never saturates → Pi constraint lifted
  This is confirmed by FINDING 8 (adrc_saturation_test): PID-nosat = 1.000 for all 15 designs
  (saturation is sole cause of PID failure); ADRC never saturates (slew_frac=0). Integral only
  helps when NOT saturated; ESO prevents saturation in the first place.

Files: tools/observer_universality_test.py (Pi bug fixed 2026-06-20),
experiments/results/observer_universality_py.csv (Pi re-saved as keff×lat²).

## NEW (2026-06-17): Performance frontier — where PID fails, how far ADRC extends

PERFORMANCE FRONTIER (tools/performance_frontier.py, 2026-06-17):
63 designs from exp1_final_population stratified into 7 Pi tiers. For each design:
  PID: 18×7 joint Kp×Kd grid (3 search seeds), then evaluate best gains with 15 seeds (7001-7015).
  ADRC: sweep omega_c ∈ {1.5, 2, 3, 5, 7, 10, 15}, omega0=5×omega_c, b0=keff, evaluate 15 seeds.
  Output: peak_pid_sr, peak_adrc_sr (best achievable SR under optimal tuning for each architecture).

KEY RESULT — PERFORMANCE TABLE BY PI TIER:
  Pi tier    n  PID mean  PID<80%  ADRC mean  ADRC<80%  Delta mean
  <100       10   1.000    0%       1.000       0%         0.000
  100-300    10   1.000    0%       1.000       0%         0.000
  300-800    10   1.000    0%       1.000       0%         0.000
  800-2k     10   1.000    0%       1.000       0%         0.000
  2k-5k      10   0.940    0%       1.000       0%         0.060
  5k-9k      10   0.867   10%       0.973       0%         0.107
  >9k         3   0.622   67%       0.911      33%         0.289

FRONTIER CROSSINGS:
  First PID SR<0.90:  Pi = 2,616   ADRC: Pi = 7,795   → ADRC extends 3.0× in Pi
  First PID SR<0.80:  Pi = 5,214   ADRC: Pi = 12,648  → ADRC extends 2.4× in Pi
  First PID SR<0.50:  Pi = 5,214   ADRC: Pi = ∞ (never below 0.50 in tested range)

Designs where ADRC succeeds but PID fails (sr_delta > 0.1):
  R2106 (FRAGILE, td=144.8, lat=6, Pi=5214): PID SR=0.467, ADRC SR=0.933, best_wc=5
  R1513 (EASY,    td=283.5, lat=6, Pi=10206): PID SR=0.600, ADRC SR=1.000, best_wc=5

Only design where BOTH fail:
  R2072 (FRAGILE, td=351.3, lat=6, Pi=12648): PID SR=0.333, ADRC SR=0.733 (best_wc=5)
  CAVEAT: R2072 was previously shown to reach ADRC SR=1.000 with omega0/omega_c=7-20
  (adrc_ceiling_omega0_sweep, 2026-06-15). The frontier experiment uses omega0=5×omega_c
  (standard bandwidth-separation rule). With non-standard omega0/omega_c, even R2072 is solvable.

Spearman correlations:
  rho(Pi, peak_pid_sr)  = -0.668, p = 2.15e-09  (strong)
  rho(Pi, peak_adrc_sr) = -0.325, p = 9.35e-03  (weak — ESO partially decouples Pi dependence)
  rho(Pi, sr_delta)     = +0.558, p = 2.02e-06  (ADRC advantage grows with Pi)

INTERPRETATION: This is the most practical summary of the Pi constraint:
  Pi < 5000: PID achieves SR=1.0 with optimal tuning. No ADRC benefit in attitude hold.
  Pi 5000-12000: PID starts failing; ADRC (omega_c=5, omega0=25) succeeds for all but 1 design.
  Pi > 12000: At standard omega0/omega_c=5, the single extreme design (R2072) narrowly fails ADRC.
    With omega0/omega_c adjusted to 10-20, R2072 is also solvable (separately confirmed).
  Design rule: Pi = theta_ddot_max × latency_steps² > 5000 → use ADRC over PID.
  The twofold ρ difference (PID: -0.668 vs ADRC: -0.325) is the quantitative signature of ESO
  decoupling — the same Pi parameter predicts PID failure twice as strongly as ADRC failure,
  because ESO handles the wind-disturbance component of the Pi constraint.

FILES: tools/performance_frontier.py, experiments/results/performance_frontier_py.csv.

## NEW (2026-06-18): ADRC frontier extension + Pi theoretical derivation

ADRC FRONTIER EXTENSION (tools/adrc_frontier_extension.py, 2026-06-18):
The original performance_frontier.py stopped at Pi=12,648 (the corrected-population maximum) —
leaving open whether ADRC's frontier was actually AT 12,648 or simply BEYOND 12,648. This
experiment tests 44 designs from the stress-test population (latency 1-12 steps), all with Pi>9000,
up to Pi=49,893 (3.9× beyond). ADRC sweep: omega_c ∈ {0.5,1,1.5,2,3,5} × omega0/omega_c ∈
{5,8,12,20} = 24 settings per design. 15 eval seeds (9001-9015, disjoint from all prior).

KEY RESULT BY PI TIER:
  Pi tier   n  PID<80%  ADRC-std<80%  ADRC-ext<80%
  9k-12.6k  27   85%        4%          0%
  12.6k-15k  6   83%        0%          0%
  15k-20k    5   80%       20%          0%
  20k-30k    3  100%       33%          0%
  >30k       3  100%      100%          0%   ← standard ratio UNIVERSALLY fails above Pi=30k

Zero ADRC failures (extended grid) up to Pi=49,893. Most extreme design (Pi=49,893, td=346, lat=12):
  PID SR=0.000, ADRC-standard SR=0.467, ADRC-extended (omega_c=2, omega0/omega_c=20) SR=0.933.
42/44 designs required omega0/omega_c > 5; 29/44 required omega0/omega_c = 20.

KEY FINDING: ADRC has NO FIXED Pi CEILING within the tested range. It has a tuning parameter
(omega0/omega_c ratio) that must scale with Pi:
  Pi < 15,000: standard omega0/omega_c=5 works
  Pi 15,000-50,000: need omega0/omega_c = 12-20 (latency-adaptive)
  Pi > 50,000: not yet tested

Updated design rule (supersedes old "Pi>5000 → use ADRC"):
  Pi < 5,000: PID (optimal Kp)
  Pi 5,000-15,000: ADRC (omega0/omega_c=5)
  Pi 15,000-50,000: ADRC (omega0/omega_c=12-20)
  Pi > 50,000: unknown

FILES: tools/adrc_frontier_extension.py, experiments/results/adrc_frontier_extension_py.csv.

Pi THEORETICAL DERIVATION (tools/pi_theory_validation.py, 2026-06-18):
⚠️ CORRECTED 2026-06-20: Pi = keff × tau² (NOT theta_ddot_max × tau² as originally stated).
Derives why Pi = keff × tau² (not tau^1 or tau^3) from first principles:
  Ceiling (DIPDT phase-margin): Kp_ceiling ≈ 0.9/tau  [keff-independent, tau^(-1)]
  Floor (bang-bang blind-spot): Kp_floor ∝ keff × tau  [floor driven by keff, not by theta_ddot_max]
  Window = ceiling/floor ∝ (tau^-1)/(keff × tau) = 1/(keff × tau²) = 1/Pi

WHY keff, NOT theta_ddot_max: theta_ddot_max = keff × u_max where u_max = max_gimbal × 12/15.
  The ceiling is keff-independent (confirmed: coefficient ≈ 0). The floor depends on keff, not u_max.
  Adding log(u_max) to the window regression: CV R² = -0.142 alone (hurts), coef = -0.088 when
  both keff and td are included (vs keff coef = -1.001). The max_gimbal factor in theta_ddot_max
  adds noise. CORRECT FORMULA: Pi = keff × tau² where keff = F15 × motor_scale × CU_TO_RAD × l / Iyy.
  Note keff is independent of max_gimbal_deg — one fewer parameter for builders to measure.

VALIDATION on window_ratio v2 (n=82 non-censored):
  keff + latency:   CV R²=0.535  (vs td + latency: 0.449)
  Pi_keff single:   CV R²=0.546  (vs Pi_td single: 0.452)
  keff coefficient in joint model (keff + td + latency): -1.001 ≈ -1.000 (exact theoretical prediction)
  td coefficient in same model: -0.088 ≈ 0 (td adds nothing once keff is included)
  All 7 theoretical exponent predictions confirmed (ceiling keff=0, lat=-1; floor keff=+1, lat=+1;
  window keff=-1, lat=-2). Max deviation 0.25 (floor keff: 1.251 vs theory 1.0).

PHYSICAL MEANING: Pi = keff × tau² [rad/CU] — angular displacement per control unit accumulated
during one latency window. The controller commands u CU and is blind for tau seconds; during that
time the system rotates Pi × u radians with no corrective feedback. Higher keff (more torque per
CU) or longer tau → more blind-spot error → floor rises and ceiling drops simultaneously.
Floor rises AND ceiling drops with tau because both trace to the same latency:
  ceiling → phase margin erodes under delay → falls as tau^(-1)
  floor → blind-spot impulse per correction cycle grows → rises as tau^(+1)
  combined: tau^(-2) = DOUBLE SQUEEZE (each doubling of latency → 3.7× window compression, not 2.1×)

tau² is NOT empirical — it is theoretically predicted from two independently established mechanisms.
NOTE: the previous "theta_ddot_max × tau²" interpretation (angular displacement at MAXIMUM authority)
is intuitive but mechanistically wrong — max_gimbal enters through u_max which affects neither the
floor nor the ceiling formula. Use keff × tau² for theory and design calculations.

FILES: tools/pi_theory_validation.py. Data: experiments/results/window_ratio_v2_py.csv.

## NEW (2026-06-17): Pi parameter, window_ratio as primary metric, Kp_miss S2R law

THREE NEW ANALYSIS RESULTS (no new simulations required — pure data analysis on existing CSVs):

DIMENSIONLESS Pi PARAMETER (tools/pi_parameter_test.py):
⚠️ CORRECTED 2026-06-20: Pi = keff × latency² is the mechanistically correct formula.
Original over_sr regression (n=262) used td instead of keff — td includes u_max (max_gimbal noise).
UPDATED NUMBERS (window_ratio v2, n=82 non-censored):
  Pi_keff = keff × latency²:  CV R²=0.546  (vs Pi_td = td × latency²: CV R²=0.452)
  keff + latency (two-var):   CV R²=0.535  (vs td + latency: 0.449)
  keff coef in joint model: -1.001 ≈ -1.000 (theory exact); td coef = -0.088 ≈ 0
Physical meaning: Pi = keff × tau² [rad/CU] = angular displacement per control unit per latency
window. Cleaner than theta_ddot_max × tau² because max_gimbal (which separates the two) adds only
noise (r(log_u_max, log_window) = -0.134, CV alone = -0.142).
IMPLICATION: both predictors (authority, latency) combine into a single physically-meaningful product.
keff doesn't require knowing max_gimbal_deg — it's determined by T, motor_scale, l_nozzle, Iyy alone.

NOTE: Prior references to "Pi = theta_ddot_max × latency²" throughout this file are STALE.
Use Pi = keff × latency² for all new analyses. Numerical thresholds from the performance_frontier
experiment (Pi=5000, 12000, etc.) were computed with td-based Pi and should be treated as
approximate pending recalculation; the QUALITATIVE design rule (low Pi = safe, high Pi = ADRC needed)
is unchanged.

WINDOW_RATIO AS PRIMARY METRIC: over_sr is right-censored at 1.0 for wide-window designs
(all score ~0.98 regardless of actual window). window_ratio = Kp_ceiling/Kp_floor is the
uncensored, theory-motivated metric. CV R²=0.616 (window_ratio, n=82) vs 0.325 (over_sr, n=262).
The R²=0.33 modest result is partly a measurement artifact; on the right metric, R²≈0.62.

Kp_MISS S2R SCALING (tools/kp_miss_scaling.py):
Kp_miss = max(0, Kp_floor-Kp_simple) + max(0, Kp_simple-Kp_ceiling).
PERFECT MATCH: predicted Kp_miss>0.5 rate = 63.9% for narrow-window == actual FR 63.9%.
Dominant mechanism: OVERTUNING (93% of misses). Simple model picks Kp too HIGH, Kp_simple
> Kp_ceiling (=380/lat). Undertuning (Kp_simple < Kp_floor) is rare (7%).
The SIZE of Kp_miss does NOT scale cleanly with td (R²=0.007) — once you've missed the window,
the magnitude is dominated by how far the simple model pushed Kp, not by the design's authority.
INFEASIBLE (n=2): 0% Kp_miss but 100% false APPROVAL — the S2R danger zone.
Files: tools/kp_miss_scaling.py, experiments/results/kp_miss_scaling_py.csv.

## External review response (2026-06-16): matched-configuration test + claim narrowing

An external (ChatGPT) review of the rewritten paper flagged: (1) FRAGILE/EASY still read as natural
classes in places despite Section 4.0's continuum framing; (2) every result depends on this
project's own simulator, never checked against reality; (3) the mechanistic chain (Newton->bang-
bang->K_u) is weaker evidence than the predictor itself and should be presented with lower
confidence; (4) the ADRC dissolution test shows "an alternative architecture helps," not "FRAGILE
is a PID-specific artifact" (would need LQR/MPC/gain-scheduling/adaptive-PID tested too); (5) AUC
language sometimes reads as "near-classifier" when it's a ranking metric. Also proposed the single
most valuable missing experiment: hold one airframe fixed and vary ONLY Iyy, sweeping Kp finely at
each level, to get a clean (unconfounded) success-rate-vs-Kp curve per authority level.

Ran that exact experiment (tools/matched_configuration_test.py, n=2250 sims, 9 Iyy levels x 25 Kp
points x 10 seeds, every other parameter fixed). Initial result showed window narrowing 122x -> 29x
-> 11x but ceilings were capped at 320 for all high-td designs, leaving the mechanism ambiguous.

EXTENDED SWEEP (2026-06-16, tools/matched_config_extended_kp.py, 18 Iyy levels x 40 Kp points x
10 seeds, Kp up to 1280, static_margin corrected to +0.10 = stable, fresh seeds 13001+):
- All 5 high-td designs (td>=80) found TRUE ceilings within [1, 1280]. None capped.
- Window narrows monotonically in high-authority zone: 142x -> 82x -> 57x -> 39x -> 33x
  (td=93->111->133->158->189 rad/s^2). Matches the qualitative claim but with cleaner numbers.
- Mechanism: BOTH components. Floor rises from 3.0 to 10.9 (3.6x) as td increases 93->189.
  Ceiling drops modestly from 512 to 354 (1.4x) over same range. Floor is the dominant driver.
- Low-authority zone (td<80): windows 118-295x, noisy, floor~1-5. Wide and forgiving throughout.
Files: tools/matched_config_extended_kp.py, experiments/results/matched_config_extended_kp_py.csv,
       experiments/results/matched_config_extended_kp_summary_py.csv.

FIDELITY CUTOFF BY TD (2026-06-16, tools/fidelity_cutoff_by_td.py):
Measured delta_SR = SR_ablated - SR_full (positive = module was adding difficulty) for 25 designs
(5 per td tier) at their physics-optimal Kp. 7 conditions: full, no_wind, no_slew, no_noise,
no_backlash, no_deadband, min_latency. 10 fresh seeds (14001+).
Results (mean delta_SR by tier):
  td<40:       ALL modules: delta~0 (full physics achieves SR~1.0, no room to improve)
  td40-70:     ALL modules: delta~0 (same)
  td70-100:    no_wind=+0.02, no_slew=+0.02, no_noise=+0.02, min_latency=+0.02 (driven by 1 FRAGILE)
  td100-150:   no_wind=+0.04, no_slew=+0.10, min_latency=+0.12, no_noise=+0.12, no_backlash=+0.08
  td>150:      all deltas 0.00-0.04 (these designs mostly EASY despite high td)
KEY FINDING: Wind has the SMALLEST evaluation impact at any td level (max +0.04). Noise and latency
are the HARDEST evaluation modules for high-td designs (+0.12). This is consistent with the S2R
finding (wind matters for TUNING, not for evaluation at a wind-optimized Kp). A builder who has the
right Kp but no latency/noise in their eval model suffers more than one who drops wind from eval.
Files: tools/fidelity_cutoff_by_td.py, experiments/results/fidelity_cutoff_by_td_py.csv.

This is the cleanest internal-validity evidence in the project (no confound between different
designs), though still simulation-only — the natural next step is the literal hardware version.

Also narrowed the ADRC section's conclusion (no longer claims "FRAGILE is 94% a PID architecture
property," now states "ADRC demonstrates an alternative exists; doesn't rule in/out PID specifically
without testing LQR/MPC/etc."), and added an explicit confidence-tiering section to the paper
(predictor > Iyy-irreplaceability > latency mechanism > full mechanistic chain > ADRC cross-
architecture claim > simulator's absolute correctness, in decreasing order of trust).

Files: tools/matched_configuration_test.py, experiments/results/matched_configuration_py.csv,
experiments/results/matched_configuration_summary_py.csv.

## Full assessment pass (2026-06-16): consistency fixes + ADRC-equivalence literature finding

A full top-to-bottom re-read of the paper found and fixed several stale numbers left from the
rapid v2 edit pass: Section 1.4 pointed at the wrong primary section (4.1 instead of 4.0);
Section 7.1/7.2 still cited old 70 rad/s²/1.3% instead of final-population 55 rad/s²/1.5%; the
Authority Trap visualization section silently mixed in n=16 pre-correction stats with no
staleness flag; Appendix A and the novelty table were missing the matched-configuration test.

A fresh literature search (not just re-asserting the 2026-06-15 check) found a consequential
result: Carlson (2025, arXiv:2501.11374) proves linear ADRC tuned with the bandwidth method —
exactly the omega_c/omega0/b0 parameterization used throughout this project — is mathematically
EQUIVALENT to a 2-DOF PID with set-point weighting and a measurement low-pass filter, for
first/second-order plants (this project's rocket dynamics are second-order). This means the
paper's "cross-architecture" framing (Section 6) overstated how distinct PID and ADRC are.
Added new paper Section 6.0 disclosing this and reframing the claim: the two are closely related
LINEAR structures, and the empirical ceiling-law comparison survives specifically because it's
about the NONLINEAR/saturated regime (actuator slew saturation -> bang-bang for PID; ESO keeps
estimating disturbance through saturation for ADRC) where the linear equivalence provably breaks
down — "two disturbance-handling conventions, divergent under saturation" replaces "two unrelated
architectures" everywhere in the paper. Logged as a fifth self-correction beat (Section 9.2) —
found via the project's own literature check, not an external reviewer.

STS probability estimate held at 55-66% sim-only / 68-80% with hardware (this round was
consolidation/honesty, not new capability). Paper Section 9.5 next-steps, updated status
(2026-06-16 session):
COMPLETED: (2) ADRC omega0-ratio sweep — all 3 zero-ceiling cells (td~200, lat 8/10/12) are
  solvable at omega0/wc=10 with SR=1.000; tools/adrc_ceiling_omega0_sweep.py, 800 sims, Section
  6.2 caveat fully resolved and updated; (3) raw-features ML baseline re-run on final n=36 FRAGILE
  population — theta_ddot alone AUC=0.975 ties best trained model, log(td x lat) leads all at
  CV=0.991, Section 4.3.1 updated; (4) Authority Trap visualizations regenerated on final n=36
  population (all 6 HTML outputs, EASY=2362 FRAGILE=36, Cohen d=3.71).
COMPLETED (added 2026-06-16 continuation): extended Kp sweep to 1280 (true ceilings found, floor-
  driven mechanism confirmed); fidelity cutoff by td (noise+latency dominant at td>100, wind
  smallest evaluation impact — distinguishes tuning vs. evaluation role of each module);
  static_margin sign corrected to standard aerospace convention throughout matched-config tools.
REMAINING: (1) hardware matched-configuration test [highest value]; (5) extend continuous
  regression past n=222 above td=300; (6) narrative compression for presentation.

---

# Current Thesis Direction

Current working thesis (updated 2026-06-13, new design space + 3-seed robustness):

⚠️ MAJOR REVISION: The slew formula had a π/180 unit error (servos modeled 57.3× too slow).
After fix AND restricting design space to realistic hardware (servo_slew [60,200]):
INFEASIBLE was 0. The Iyy × wind_strength controllability boundary is DEAD.
All prior INFEASIBLE findings, Exp4 fidelity results, and Exp5 slew payoff curves are INVALID.

⚠️ DESIGN SPACE UPDATE (2026-06-13): Iyy range expanded to [0.005, 0.100] (was [0.010, 0.040])
and mass capped at [0.50, 1.20] kg to match real hobby TVC rockets (3D-printed: Iyy~0.004,
F15 motor: T/W breaks at ~1.4 kg). New 3-seed robustness test replaces 1-seed (ROBUSTNESS_
SUCCESS_RATE=0.80). Definitive n=2400 run: EASY=2347, MARGINAL=5, FRAGILE=45, INFEASIBLE=3.
S2R re-run (n=2400) in progress — expect qualitatively similar false rejection pattern.

NEW THESIS: Simulator fidelity affects GAIN SELECTION more than GO/NOGO decisions.
Simple model with theta0=10° (autotune_continuous, Kp up to 320) picks VARIABLE gains driven
by discrete-time numerical stability and RMS-minimization for step response — neither correlates
with wind rejection requirements. FRAGILE designs (narrow window) suffer high false rejection
(~56% in prior space). EASY designs suffer ~12% false rejection (gain too low OR too high).

Gain co-design is the central mechanism: which simulator you tune in determines which gains
you select, and that gain choice determines whether the design appears viable or not.

The S2R fidelity story:
— Old (DEAD): simple model falsely approves INFEASIBLE designs (99.6%)
— Current: simple model picks erratic gains (bimodal); false REJECTIONS dominate; 0% false approvals.

IMPORTANT: p_unstable still has near-zero correlation with regime (CONFIRMED).
The Iyy × wind controllability boundary does NOT exist with realistic hardware.
INFEASIBLE=3 in n=2400 run (0.1%): genuinely uncontrollable; cannot exceed nom_sr=0.35 at any Kp.

The project is studying:
1. Which fidelity modules drive the gain-selection gap (Exp4 gain mechanism study)
2. Whether the gain co-design mechanism can be explained mechanistically (wind vs noise vs other)
3. Hardware validation of the sim-to-real gain gap prediction

---

# Experiment Status

## Exp1 – Regime Mapping

Status: Major revision complete (2026-06-13). All prior Exp1 results are SUPERSEDED.

Methodology (current):
* 2400 LHS designs with T/W > 1 feasibility filter (n=1200 preliminary run superseded)
* Full-physics evaluation: nonlinear_aero + dyn_aero + thrust_curve + cg_shift ON
* Gain search: autotune_continuous — Kd probe [1,4,16,64] at Kp=40; Kp log-search [1,320]
  (10-point coarse + 6-point refine); NO hard cap. Replaces old 5×5 grid (capped at Kp=80).
* Autotune objective: highest mean success rate across seeds 1+2; tiebreak by minimum RMS
* Nominal evaluation: 3-seed average (wind is stochastic; single-seed is unreliable)
* Under/over robustness: 3 seeds, ROBUSTNESS_SUCCESS_RATE=0.80 threshold
  (replaces 1-seed test; 36% detection at true p_fail=0.14 vs 14% with 1 seed)
  SR ∈ {0, 0.333, 0.667, 1.0} with 3 seeds; threshold=0.80 means any failing seed → FRAGILE

Design space (current, updated 2026-06-13):
* mass: [0.50, 1.20] kg — F15 motor T/W=1 breaks at ~1.47 kg; realistic hobby TVC range
* Iyy: [0.005, 0.100] kg·m² — expanded from [0.010, 0.040]; lightweight 3D-printed Iyy ~0.004
* servo_slew_deg_s: [60, 200], static_margin: [-0.30, 0.30], Cm_alpha: [-90, -15]
* motor_scale: [0.5, 3.0], max_gimbal_deg: [2, 15], latency_steps: [1, 6]
* wind_strength: [0.05, 0.45], deadband: [0.00, 0.15], backlash: [0.00, 0.25]
RATIONALE: Old Iyy=[0.010, 0.040] mathematically prevented most designs from reaching θ̈_max > 70
  unless max_gimbal was large AND motor_scale was high → artificially low FRAGILE rate (1.3%).
  Real hobby TVC rockets (100g-500g body, 3D-printed, F15 motor) have Iyy=0.004-0.015.

HISTORICAL CONTEXT (pre-slew-fix, invalid):
Old counts: EASY=850, MARGINAL=241, FRAGILE=68, INFEASIBLE=41.
Old MARGINAL was slow-servo cluster (servo_slew < 20 deg/s, median 12.5).
All these results used the buggy slew formula — do not cite.

SUPERSEDED n=1200, old design space ([60,200], Iyy=[0.010,0.040], 2026-06-10):
EASY=1171, MARGINAL=11, FRAGILE=16, INFEASIBLE=2. AUC=0.855 [0.765,0.931].
Do NOT cite these counts. New design space has different FRAGILE population.

SUPERSEDED n=1200, new design space, 3-seed (2026-06-13 first run):
EASY=1178, MARGINAL=2, FRAGILE=19, INFEASIBLE=1. AUC=0.879.
The n=1200 run had 3 anomalous low-td FRAGILE (td~14-19) that were 3-seed stochastic false
positives — they did NOT appear in the n=2400 run, confirming stochastic origin.
Do NOT cite these counts or this AUC. Use n=2400 numbers below.

CURRENT counts (3-seed, original, new design space, n=2400, exp1_regime_index_py.csv, 2026-06-13):
* EASY     (n=2347, 97.8%): nom_sr ≥ 0.80 AND over_sr ≥ 0.80 AND under_sr ≥ 0.80 at 1.4× Kp/Kd
* MARGINAL (n=5,    0.2%): nom_sr < 0.80, all 3 robustness seeds pass — WIND-LIMITED
* FRAGILE  (n=45,   1.9%): any of {over, under} robustness fails (sr < 0.80 with 3 seeds)
  — 39/45 ceiling-limited (over fails, under passes); 3 floor-limited; 3 both-fail
  — theta_ddot_max range: [36.1, 330.6] rad/s²; NO anomalous low-td designs
  — Two FNs below Youden threshold: D800 (td=36.1, lat=6) and D1523 (td=38.3, lat=6)
    Both have latency_steps=6 — consistent with H5 phase-lag ceiling compression
  — 3-seed test sensitivity: P(catch|true p_fail=0.14) = 36% vs 14% with 1 seed
* INFEASIBLE (n=3,  0.1%): genuinely uncontrollable; nom_sr < 0.35 at any Kp up to 320

⚠️ SUPERSEDED BY 15-SEED CORRECTION (2026-06-15, see "MAJOR CORRECTION" section above):
CORRECTED counts: EASY=2365 (98.5%), FRAGILE=30 (1.25%), MARGINAL=2 (~0.08%, effectively
dissolved), INFEASIBLE=3 (0.125%, composition changed). The 3-seed counts above are kept for
historical/audit reference only — use the corrected counts for any new analysis or writing.

Key findings (2026-06-13, new design space, n=2400, 3-seed robustness):

FINDING 1: p_unstable has near-zero correlation with regime (r≈0). CONFIRMED.

FINDING 2 (REVISED 2026-06-13, new design space, n=2400): The max TVC angular acceleration predicts FRAGILE.
  AUC=0.944 [0.927, 0.959] on new design space (n=2400, n_FRAGILE=45, 3-seed robustness).
  10-fold CV: AUC=0.944 ± 0.018. This was the definitive result on 3-seed labels.
  Combined with latency: AUC=0.972 (3-seed labels; not yet re-validated on corrected labels).
  NOTE: n=1200 run gave AUC=0.879 due to 3 stochastic false-positive FRAGILE labels — those
  designs vanished in n=2400 LHS sample, confirming they were 3-seed classification noise.

  ⚠️ UPDATE 2026-06-15: 15-seed reclassification (see MAJOR CORRECTION section) IMPROVED this
  result: AUC=0.957 on the corrected population (n_FRAGILE=30). This is now the definitive AUC
  for theta_ddot_max alone. The theta_ddot×latency combined AUC=0.972 has NOT yet been
  re-validated on corrected labels — treat as approximate pending re-run.

  PHYSICAL FORMULA (derivable from hardware specs, no fitting required):
    theta_ddot_max = T_avg [N] × sin(max_gimbal_rad) × l_nozzle [m] / Iyy [kg·m²]
    ≈ T_avg × (max_gimbal_deg × pi/180) × l_nozzle / Iyy  (small angle, <2% error at 15°)

  This IS authority_inertia_ratio in physical units:
    authority_inertia_ratio = max_gimbal_deg × motor_scale / Iyy
    theta_ddot_max = authority_ratio × (F15_avg × pi/180 × l_nozzle) = authority_ratio × 0.0628
  Also: theta_ddot_max = keff_full × u_max  where keff_full = F15 × motor_scale × CU_TO_RAD × l / Iyy
    and CU_TO_RAD = pi/180 × 15/12 = 0.02182 (verified from actuator code; u_max = max_gimbal × 12/15)

  Population statistics (n=2400, 2026-06-13, new design space):
    EASY:     mean theta_ddot_max =  28.6 rad/s²  (median 17.2; high-Iyy designs now included)
    FRAGILE:  mean theta_ddot_max = 124.5 rad/s²  (median 104.4, range [36.1, 330.6])
    Ratio: FRAGILE/EASY mean = 4.4× (wide Iyy range drives large separation)
    Cohen d=1.74, t=16.59, p=1.28e-58 (highly significant)
  AUC for predicting FRAGILE (definitive): 0.944 [0.927, 0.959] (10-fold CV: 0.944 ± 0.018)
    Combined log(theta_ddot × latency): AUC=0.972 [0.962, 0.980] (10-fold CV: 0.972 ± 0.011)
    H5 delta = +0.028 (below 0.03 threshold — theta_ddot alone is already so good that
    latency adds less, but 2 FNs both have latency=6 confirming the mechanism qualitatively)

  Practical threshold: Youden-J = 54.8 rad/s² (TPR=0.96, FPR=0.12)
    At threshold=50: TP=43, FP=311, FN=2, TN=2041 — catches 43/45 FRAGILE (96%)
    At threshold=70: TP=33, FP=201, FN=12, TN=2151 — more conservative (more false NEGATIVES)
    At threshold=100: TP=23, FP=97, FN=22, TN=2255 — precision=0.19, recall=0.51
  NOTE: Low precision reflects rare base rate (1.9%). AUC=0.944 is the right ranking metric.
  PRACTICAL RULE: if theta_ddot_max > 55 rad/s² → flag for careful gain tuning with wind sim.

  OLD SPACE RESULTS (Iyy=[0.010,0.040], n=16 FRAGILE) — SUPERSEDED:
    EASY mean θ̈_max=47.2, FRAGILE mean=110.7, AUC=0.855, d=1.85, Youden-J=62.3 rad/s²
    Do NOT cite these numbers.

  authority_inertia_ratio = (max_gimbal_deg × motor_scale) / Iyy — equivalent predictor to theta_ddot_max.
  Does NOT predict required Kp level (wind drives Kp level, not authority).
  Framing: "over-actuated" rockets (high authority per unit inertia) have narrow gain windows.

  PREDICTOR SCREENING — REVISED 2026-06-13 (H5 latency; new design space results):

  Results on OLD design space (n=1200, n_FRAGILE=16, Iyy=[0.010,0.040]):
  H1 (servo_slew): REJECTED — r(slew, FRAGILE)=+0.013, solo AUC=0.45; delta=-0.000 added to td.
  H2 (wind_strength): REJECTED — r(wind, FRAGILE)=-0.001 (FN mean wind=0.213 < EASY=0.246!);
    adding wind to theta_ddot: delta=-0.014 (hurts). Wind does NOT explain the false negatives.
  H3 (aerodynamic stability): REJECTED — r(static_margin, FRAGILE)=+0.034; delta=-0.006.
  H4 (interaction ratios): REJECTED — best is wind/keff_full (+0.004). Below 0.03 threshold.
  H5 (latency_steps, hardware): CONFIRMED — AUC alone=0.836, delta=+0.072, combined AUC=0.924.
    r(latency_steps, FRAGILE) = +0.139. Binomial p=2.04e-05 (all 16 FRAGILE have latency≥4).
    r(latency_steps, theta_ddot) = -0.033 — INDEPENDENT predictor, not a proxy.
    10-fold CV: AUC(theta_ddot alone)=0.852±0.154; AUC(combined)=0.924±0.085.
    log(theta_ddot × latency_steps) as SINGLE variable gives CV-AUC=0.924.
    Physical meaning: theta_ddot × tau_latency [rad/s²·s = rad/s] = maximum angular velocity
      the rocket accumulates in one control-loop latency window before correction acts.
    Phase lag at 5Hz: 1-step=9°, 3-step=27°, 6-step=54° — 54° phase lag is significant.

  Results on NEW design space (n=2400, n_FRAGILE=45, Iyy=[0.005,0.100], 2026-06-13):
  H1 (wind_strength): REJECTED — r(wind, FRAGILE)=-0.008 (essentially zero)
  H2 (servo_slew):    REJECTED — r(slew, FRAGILE)=+0.048; delta=-0.008 (hurts)
  H3 (static_margin): REJECTED — r(static_margin, FRAGILE)=-0.001; delta=-0.002
  H4 (Cm_alpha):      REJECTED — r(Cm_alpha, FRAGILE)=-0.030; delta=-0.484 (Cm_alpha encoded backward)
  H5 (latency_steps): delta=+0.028 — below 0.03 threshold on n=45 FRAGILE (theta_ddot alone AUC=0.944
    already high). Combined AUC=0.972. Two FNs both have latency=6 — mechanism confirmed qualitatively.
    Physical: latency contributes ~54° phase lag at 5Hz (6-step) → ceiling compression independent of td.
  NOTE: H5 gave delta=+0.072 on old space (n=16 FRAGILE, lower base AUC=0.855). The smaller delta
    on new data reflects the higher base AUC, not a weaker mechanism. Combined predictor is still best.

  NOTE: latency_steps was OMITTED from the original 10-feature exhaustive search in
    tools/fragile_residual_analysis.py. The 10 features were: log_td, log_keff, log_wind,
    log_slew, wind_x_keff, wind_x_td, td_div_slew, keff_div_slew, static_margin, log_authority.
    latency_steps was present in the design space [1,6] all along — it was simply not tested.

  H1-H4 best two-variable model (10 environmental features × 10 = 45 pairs):
    log_theta_ddot + log_keff_full = CV-AUC=0.865 ± 0.058  (delta=+0.011 over theta_ddot alone)
    log_keff_full + log_theta_ddot/slew = 0.856 ± 0.074 (delta=+0.002)
  All environmental/mechanical deltas < 0.03.

  COMBINED PREDICTOR (theta_ddot + latency, 2026-06-13):
    log_theta_ddot + log_latency: CV-AUC=0.924 ± 0.085 (delta=+0.072 over theta_ddot alone)
    log(td × latency) single var: CV-AUC=0.924 ± 0.084 (identical — multiplicative combination)
  Preferred rule: compute theta_ddot AND latency_steps.
    - If theta_ddot > 55 rad/s² → flag regardless of latency (Youden-J threshold, n=2400)
    - If theta_ddot in [36,55] AND latency ≥ 5 steps (≥25ms) → also flag (latency FN zone)
    - If theta_ddot < 36 rad/s² → EASY even at max latency (no FRAGILE in this range, n=2400)

  FALSE NEGATIVES (n=2, theta_ddot < 55 but FRAGILE, n=2400 definitive run):
    D800:  td=36.1, latency=6 — at the very bottom of FRAGILE range; latency=6 pushes over edge
    D1523: td=38.3, latency=6 — same pattern; both are mild-severity (SR@1.4x likely 0.71-0.86)
  BOTH FNs have latency_steps=6 (30ms maximum). High latency independently compresses the
    gain ceiling (54° phase lag at 5Hz), explaining FRAGILE at moderate theta_ddot.
  NOTE: n=1200 run had 4 FNs (R0804/R0047/R0680/R0452); these SHIFTED to different design
    ids in the n=2400 LHS but the pattern is identical — all have latency=6 and td=36-65 rad/s².
  Combined predictor (log_td + log_latency) closes most of the FN gap.

  FORENSIC ANALYSIS (n=2400 FNs, CONFIRMING PATTERN):
  Over test protocol: OVER_SCALE=1.40 — exp1 scales BOTH Kp AND Kd by 1.40×, not 2×.
  Both FNs are ceiling-limited: they pass nominal but fail over-test at 1.4× best_Kp.
  The FRAGILE classification is CORRECT (verified via design type: latency=6 + low td).

  PHYSICAL INTERPRETATION (n=2400, 2026-06-13):
  FRAGILE is a HARDWARE property determined by two independent factors:
  1. Mechanical authority: T, sin(δ_max), L_nozzle, Iyy → theta_ddot_max (AUC=0.944)
  2. Control loop latency: tau_latency → phase lag → ceiling compression (delta AUC=+0.028)
  Combined: theta_ddot × tau_latency [rad/s] = max angular velocity before correction acts.
  keff_full (= angular accel per CU, independent of max_gimbal) is equivalent to theta_ddot.
  The 2-FN residual at low td is latency-driven: 54° phase lag at 6 steps compresses ceiling.
  Adding latency explains the FN population — not a new mechanism, but a second hardware factor.

  Updated governing rule (definitive, n=2400):
  log(theta_ddot_max × latency_steps) → product threshold  →  AUC=0.972 [0.962, 0.980]
  OR: theta_ddot_max alone at threshold ~55 rad/s² → AUC=0.944 (simpler, use when latency unknown)

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

  Step 2 — Bang-bang oscillation amplitude at probe gain Kp=2 (empirical, n=41, RERUN 2026-06-12):
    rho(theta_ddot, A_deg@Kp2) = +0.616, p=1.8e-05  — primary empirical relationship
    Power law: A ≈ 1.63° × theta_ddot^0.40  (probe flight at Kp=2, full fidelity, n=25 EASY + 16 FRAGILE)
    FRAGILE mean A = 14.9° vs EASY mean A = 6.5° (2.3× separation)
    NOTE: oscillation is NOT a slew-limited limit cycle (measured T_u=1.85s vs theory 0.17s).
    It is disturbance-driven: high keff → more aggressive response → larger bang-bang overshoot.
    OLD n=50 result (rho=0.781) used old FRAGILE labels with Kp=80 cap — superseded.

  Step 3 — Describing function formula (Åström-Hägglund 1984, exact derivation):
    K_u = 4·u_max / (π·A_rad)   [ultimate gain from oscillation amplitude]
    Verified: r(K_u_measured, K_u_theory) = 1.000. Larger oscillation → lower ceiling estimate.

  Step 4 — EASY vs FRAGILE K_u comparison:

  ⚠️ DEFINITIVE RE-DERIVATION (2026-06-15, tools/relay_final_comparison.py, n=36 FRAGILE
  final v2 population): true relay probe (Kp=2, 5 seeds, amplitude+period extraction from
  zero-crossings, NOT the RMS-approximation used below) re-run on the final corrected
  population, matched to a 36-design td-stratified EASY sample.
    EASY    K_u: mean=118.0±96.5  median=91.2   (n=36)
    FRAGILE K_u: mean=33.9±17.4   median=29.2   (n=36)
    Separation: 3.12× (median), Mann-Whitney p=4.17e-07
    r(log theta_ddot, log K_u) = -0.448; power law K_u ≈ 154 × theta_ddot^-0.29 (still too
      noisy to cite as a precise exponent — consistent with the v2 window-ratio negative
      result above; the qualitative direction is what's robust, not the fitted slope)
  This is the FIFTH consecutive correction pass (3-seed→15-seed→finer-search+30-seed→
  flight-sig rerun→relay rerun) where re-deriving on a more rigorous population STRENGTHENED
  the separation: p=0.0072 (n=41, old labels) → p=2.9e-05 (n=45, RMS-approx) → p=4.17e-07
  (n=36, true relay probe, final population). Use this n=36 true-relay result going forward;
  the RMS-approximation numbers immediately below are superseded for citation purposes but
  kept for audit trail (they used a cheaper proxy method, not a literal relay extraction).
  Data: experiments/results/relay_final_comparison_py.csv.

  OLD (n=41, relay probe, old Iyy range, 2026-06-12):
    EASY    K_u: median=108  [18-356]  FRAGILE K_u: median=39  [24-128]
    Separation: 2.8× (Mann-Whitney p=0.0072)

  UPDATED estimate (n=45 FRAGILE, n=45 EASY, from flight_sig Kp=2 data, 2026-06-13):
    Method: A_deg ≈ RMS_mean × √2 (sinusoidal approx); K_u = 4×u_max/(π×A_rad)
    EASY    K_u: median=82  [9-426]   (n=45; lower than old due to low-td EASY with high RMS)
    FRAGILE K_u: median=39  [13-110]  (n=45; IDENTICAL to old relay study — remarkably stable)
    Separation: 2.1× (Mann-Whitney p=2.9e-05); K_u estimate correlation with td: r=-0.223
    FRAGILE K_u ≈ 39 ≈ best_Kp_min from wind floor → CEILING HAS CLOSED TO FLOOR (confirmed)
    NOTE: EASY K_u drops from 108→82 because new space includes very low-td EASY designs that
    oscillate at Kp=2 from severe under-tuning (not ceiling compression); confounds K_u estimate.
    The FRAGILE K_u=39 is the key stable number — it confirms ceiling~floor for sensitive designs.
    NOTE — R0115 outlier: K_u=128 because best_Kp=253 (high-floor FRAGILE); probe at Kp=2
      measures wind-driven oscillation, not bang-bang → K_u overestimated. Excluding R0115:
      FRAGILE median=38, range=[24, 80], p=0.0037.
    NOTE — R0336: floor-limited FRAGILE (fails under-robustness test, passes ceiling test).
    OLD n=50 result (p=3.5e-05) used old FRAGILE labels — superseded by this rerun.

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

  GOVERNING EQUATION SUMMARY (UPDATED 2026-06-15, definitive n=36 true-relay re-derivation):
    "High θ̈_max rockets oscillate more aggressively at any sub-optimal Kp. By the describing
     function, the amplitude increase means K_u ≈ 29.2 (median) for FRAGILE vs 91.2 for EASY
     (3.12×, p=4.17e-07, n=36 true relay probe, final v2 population — tools/relay_final_
     comparison.py). When K_u closes toward the wind floor, no valid gain exists.
     AUC=0.975 from θ̈_max alone (n=2400, final population)."

    What IS supported:
      - Newton → θ̈_max: derivable from hardware specs (no fitting)
      - θ̈_max → bang-bang amplitude: empirical, direction confirmed on final population
        (r(log td, log K_u)=-0.448, rho=-0.548, n=36 true relay probe, 2026-06-15)
      - Amplitude → K_u: exact via describing function (definitional, r=1.000)
      - K_u separation EASY vs FRAGILE: DEFINITIVE 3.12× (median 91.2 vs 29.2), p=4.17e-07,
        n=36 true relay probe on final v2 population — strongest p-value across three
        consecutive re-derivations (0.0072 → 2.9e-05 → 4.17e-07)
      - Population-level AUC: 0.975 (n=2400, final v2 population, see Finding 2)
      - Qualitative ceiling compression at td>100: confirmed but not quantified

    SUPERSEDED — old K_u numbers, kept for audit trail only (do not cite for new analysis):
      (2026-06-12, n=41 relay study, old labels): EASY K_u=108 [18-356] vs FRAGILE K_u=39
        [24-128], 2.8×, p=0.0072.
      (2026-06-14, RMS-approximation method, n=45 labels): EASY K_u median=82, FRAGILE
        median=39 (estimated via A_deg≈RMS×√2, not a literal relay probe), 2.1×, p=2.9e-05.

    What is NOT supported:
      - Power law window ∝ theta_ddot^B: r=-0.48 too weak (n=12 sweep)
      - The v1 exponent ^-1.69 was Kd=8 confounded — DO NOT CITE
      - Slew-based ceiling derivation: oscillation T_u=1.85s vs theory 0.17s (11× mismatch)
      - Boundary equation: latency dominates ceiling (320→40-90 at lat=6 for td>60); wind barely shifts floor; see BOUNDARY EXPERIMENT v2 below

  BOUNDARY EXPERIMENT v1 (2026-06-14, tools/gain_window_boundary.py) — METHODOLOGICALLY FLAWED:
  Selected only EASY designs → floors were 1-5 → no collapse possible. Do NOT cite v1 as "null result."

  BOUNDARY EXPERIMENT v2 (2026-06-14, tools/gain_window_boundary_v2.py) — DEFINITIVE:
  Protocol: 6 FRAGILE + 6 matched EASY designs at same theta_ddot levels [36-210 rad/s²];
    sweep wind [0.05-0.42] × latency [1,3,6] × 15 Kp × 7 seeds = 18,900 sims.
  Data: experiments/results/gain_window_v2_summary_py.csv (180 rows)

  RESULTS:
  1. LATENCY IS THE DOMINANT COMPRESSOR: Ceiling drops from 320 to 40-90 at lat=6 for all designs
     with td>60 — a 4-8× compression regardless of regime. Wind does NOT shift floor consistently
     (<5 Kp units change across full wind range; effect noisy and non-monotonic).

  2. FRAGILE CLEARLY NARROWER THAN EASY — but only at td < 70 rad/s²:
     td=36: FRAGILE floor=8-27, ratio=7-40× vs EASY floor=1-3, ratio=43-320×  (3-8× difference)
     td=67: FRAGILE floor=5-8, ratio=11-18× at lat=6 vs EASY floor=1, ratio=90-130× (6-8× wider)
     → At low theta_ddot (36-67), FRAGILE designs genuinely have narrower windows under same conditions.

  3. AT HIGH THETA_DDOT (>100), EASY AND FRAGILE WINDOWS CONVERGE:
     td=128: FRAGILE 8-13× vs EASY 5-20× at lat=6 — essentially identical
     td=171: FRAGILE 1.5-8× vs EASY 1.0-5× at lat=6 — EASY AS NARROW OR NARROWER
     td=210: FRAGILE 1.5-3.3× vs EASY 1.0-12× at lat=6 — MIXED
     → The FRAGILE classification at td>100 may partially reflect 3-seed sampling noise in exp1.
     → At lat=6, all designs with td>100 rad/s² have narrow windows (ratio 1-15×) regardless of regime.

  4. NO DESIGN COLLAPSES TO ZERO: Across all 180 conditions, no design had zero viable Kp.
     FRAGILE means "narrow window," not "no window." Minimum ratio was 1.0× (R1715, EASY).

  5. THE 55 RAD/S² THRESHOLD IS VALIDATED:
     Below 55: even at worst lat=6+wind=0.42, windows stay ≥40× (confirmed for td=36)
     55-100: ceiling starts compressing; FRAGILE distinguishable from EASY by ~4-8×
     >100: both regimes show narrow windows at lat=6; distinction becomes environment-dependent

  INTERPRETATION (corrected from v1 null result):
    Latency matters enormously (ceiling 320→40-90 at lat=6). Wind barely shifts the floor.
    FRAGILE vs EASY is mechanically distinct at td=36-70 rad/s². At higher td (>100), the
    window narrowing is universal — all designs at td>100 with lat=6 are at risk. The FRAGILE
    label at high td reflects which specific seeds happened to fail the 3-seed robustness test
    during exp1 classification. The practical recommendation remains: theta_ddot > 55 → flag;
    theta_ddot > 100 + lat ≥ 5 → careful gain tuning required regardless of exp1 regime label.

  KEFF COMPARISON (same theta_ddot, FRAGILE vs EASY): keff is NOT a reliable differentiator.
    td=36: FRAGILE keff=8.21 vs EASY keff=7.91 (FRAGILE slightly higher — consistent with theory)
    td=67: FRAGILE keff=7.06 vs EASY keff=6.22 (FRAGILE higher — consistent)
    td=128: FRAGILE keff=19.69 vs EASY keff=19.65 (tied — no distinction)
    td=171: FRAGILE keff=16.45 vs EASY keff=25.57 (EASY higher — OPPOSITE of theory)
    The "high keff causes elevated floor" hypothesis explains td<70 pairs but not td>100 pairs.
    DO NOT claim keff differentiates FRAGILE from EASY in the general case.

    The amplitude exponent 0.40 is empirical (relay study, n=41); K_u formula is exact.
    The practical threshold is 55 rad/s² (Youden-J, n=2400); latency dominates threshold behavior.

  GAIN CEILING EQUATION (2026-06-14, gain_margin_equation.py):
  Derived from phase margin analysis for PD + double integrator + delay (DIPDT framework):
    EXACT: solve x × arctan(x) = keff × Kd² × τ  →  K_u_theory = keff / x²
    APPROX (keff × τ < 1.0): K_u_theory ≈ 0.9 / τ  (keff-independent, ±20%)
    EMPIRICAL (simulation with bang-bang + SR=0.80):
      K_u_sim ≈ 2.1 × K_u_theory ≈ 1.9 / τ = 380 / latency_steps  (at 200 Hz)
      Power law fit: ceil = 2.6 × keff^(-0.20) × τ^(-1.10)  [R²=0.53, n=20]
      Latency exponent -1.10 ≈ -1 (consistent with linear theory)
      keff exponent -0.20 (small — latency dominates ceiling, not plant gain)

  PREDICTION TABLE (at 200 Hz, empirical ceiling law):
    lat=1 (5ms):  Kp_max ≈ 380  (verified: ceiling ≥270 in sim)
    lat=3 (15ms): Kp_max ≈ 127  (verified: 90-190 in sim)
    lat=6 (30ms): Kp_max ≈ 63   (verified: 40-90 in sim)

  SPOT CHECK (5 cases, all correct):
    R1526 Kp=60 lat=6 wind=0.22: SR=1.00 ✓ PASS (below ceil=90)
    R1526 Kp=90 lat=6 wind=0.22: SR=0.86 ✓ PASS (at ceil=90)
    R1526 Kp=130 lat=6 wind=0.22: SR=0.43 ✓ FAIL (above ceil=90)
    R0728 Kp=60 lat=6 wind=0.22: SR=0.86 ✓ PASS (at ceil=60)
    R0728 Kp=90 lat=6 wind=0.22: SR=0.57 ✓ FAIL (above ceil=60)

  GAIN FLOOR: from relay study (n=41): Kp_floor ≈ 0.35 × keff^0.70 (rho=0.58) — PARTIAL.
    UPDATED (2026-06-17, window_ratio regression v2, n=104 non-censored):
    Kp_floor ≈ 0.06 × keff^1.06 × latency^0.96   (R²=0.627, CV=0.594)
    ⚠️ OLD FORMULA (keff-only) gives R²=0.419 — latency was simply not tested before.
    MECHANISM: high latency → rocket accumulates angular error for τ sec before correction →
    needs higher Kp to reject wind within bounds (raises floor). Effect is as strong as keff.
    In boundary experiment: EASY floor=1-8, FRAGILE floor=5-27 at lat=6 — CONSISTENT.

  WINDOW RATIO ≈ Kp_ceiling / Kp_floor ≈ (380/latency) / (0.06 × keff^1.06 × latency^0.96)
    Simplified: window ≈ 6300 × keff^(-1.06) × latency^(-1.96) [from formula components]
    Empirical (v2 regression, n=116): window ≈ 8700 × keff^(-1.19) × latency^(-1.88)
    Each doubling of latency → window compressed by 2^1.88 = 3.7× (not 2^1.10 = 2.1× as before)

  LITERATURE STATUS:
    KNOWN: DIPDT stability condition (Di Ruscio 2010/2017; ISA Transactions 2017)
    KNOWN: Describing function K_u = 4×u_max/(π×A) (Åström-Hägglund 1984)
    NOVEL: K_u_theory × τ ≈ 0.9 approximation for keff×τ << 1 regime
    NOVEL: Empirical 2.1× correction factor for slew-saturated bang-bang actuators
    NOVEL: Hobby-TVC authority/Iyy → gain window quantification (no prior work found)

STRESS TEST (2026-06-14, tools/exp1_stress_test.py, exp1_stress_test_py.csv) — DEFINITIVE:
  Protocol: n=2400 NEW LHS (seed=99), latency [1,12] steps, Iyy ≥ mass×0.010 filter.
  Represents realistic hobbyist hardware: Arduino/I²C setups → 50-100Hz control loops → lat=8-12.
  Standard success criterion (30-deg/70-deg), same autotune, same 3-seed robustness.

  RESULTS:
  | Regime     | Original (lat 1-6) | Stress test (lat 1-12, Iyy filter) |
  |------------|--------------------|------------------------------------|
  | EASY       | 2347 (97.8%)       | 2241 (93.4%)                       |
  | MARGINAL   |    5 (0.2%)        |    8 (0.3%)                        |
  | FRAGILE    |   45 (1.9%)        |  135 (5.6%)   ← 3× increase        |
  | INFEASIBLE |    3 (0.1%)        |   16 (0.7%)   ← 5× increase        |
  Total at-risk: 53 (2.2%) → 159 (6.6%)

  AUC (predicting FRAGILE):
    theta_ddot alone:    0.944 → 0.919  (drops because latency-driven FRAGILE now dominate)
    theta_ddot×latency:  0.972 → 0.951  (delta now +0.032, above 0.03 threshold)

  FRAGILE BY LATENCY:
    lat=1-2 (5-10ms, fast Teensy): 0% FRAGILE, 0 INFEASIBLE
    lat=3-4 (15-20ms):             ~1% FRAGILE (high-td only)
    lat=5-6 (25-30ms):             3.5% FRAGILE
    lat=7-8 (35-40ms):             4.7-6.5% FRAGILE
    lat=9-12 (45-60ms, Arduino):   10-13% FRAGILE, 1-3 INFEASIBLE per bin
  Tearney group: lat≤6 → FRAGILE=1.7% (matches original); lat≥7 → FRAGILE=9.4% (5.5×!)

  FRAGILE theta_ddot in stress test:
    lat≤6:  mean=98.1, min=14.9 rad/s²  (high-authority, matches original pattern)
    lat≥7:  mean=72.6, min=20.3 rad/s²  (moderate authority — latency is the bottleneck)
  INFEASIBLE: all at lat≥6, mean td=166.3 rad/s² — high authority + slow MCU = no valid gain

  KEY FINDING: MCU speed is the dominant risk factor for the Arduino-class population.
  At lat=9-12, 10-13% of designs are FRAGILE regardless of mechanical authority.
  At lat=1-2, 0% of designs are FRAGILE (ceiling ~380, far above any realistic floor).
  The Kp_max ≈ 380/latency formula predicts this: lat=10 → ceiling=38, floor~40 for typical design → FRAGILE.

  IYY FILTER EFFECT (from tools/stress_test_analysis.py, existing n=2400 data):
    Removed 80 designs (3.3%): Iyy < mass×0.010 (physically implausible mass distribution)
    FRAGILE removed: 7/45 (16%) — these were the highest-td FRAGILE (mean td=143.5 rad/s²)
    AUC after filter: 0.953 (vs 0.943 unfiltered) — formula becomes MORE accurate on credible designs
    Conclusion: Iyy filter improves both physical credibility AND predictor accuracy.

  TIGHTER CRITERION EFFECT (band_20: max_theta < 20 deg):
    EASY designs: 100% already achieve 20-deg criterion (band_20_rate = 1.0 for ALL 2347 EASY)
    FRAGILE designs: 8/45 (17.8%) fail the 20-deg criterion nominally
    Conclusion: EASY/FRAGILE distinction is SHARP even under tight criterion. No borderline EASY.
    The 70-deg success gate is not artificially lenient — EASY designs hold 20-deg comfortably.

  IMPLICATION FOR PAPER: The stress test confirms and extends the main finding.
    Under realistic Arduino-class conditions, FRAGILE fraction triples (1.9%→5.6%) and latency
    becomes an essential predictor (delta AUC +0.032, now above 0.03 threshold).
    The practical recommendation changes: for Arduino users (lat≥8), compute θ̈_max × latency.
    For fast MCU users (lat≤4), θ̈_max alone is sufficient.
    Hardware choice (MCU speed) is now established as equally important as mechanical design.

FINDING 3 (SURVIVES): Aerodynamic instability does not help maneuverability.
  Stable designs outperform unstable on all tasks. Gap grows with maneuver amplitude.
  At 10° maneuver, stable=86.8% vs unstable=68.8% at slew=120 (18pp gap, does not close).
  In Exp1 attitude-hold population: stable mean nom_sr=0.999 vs unstable=0.995 (small but p=0.032).

FINDING 4 (S2R): Simulator fidelity changes GAIN SELECTION more than GO/NOGO.
  DEFINITIVE RESULTS (n=2400, new design space, Iyy=[0.005,0.100], 2026-06-13):
  | Regime   | n    | SR(simple→full) | SR(full→full) | Gap   | False Approval | False Rejection |
  |----------|------|-----------------|---------------|-------|----------------|-----------------|
  | EASY     | 2347 | 0.863           | 1.000         | 0.137 | 0.0%           | 9.8%            |
  | MARGINAL | 5    | 0.467           | 0.667         | 0.200 | 0.0%           | 60.0%           |
  | FRAGILE  | 45   | 0.430           | 0.919         | 0.489 | 0.0%           | 57.8%           |

  Kp comparison (median):
    EASY:     Kp_simple=36.7  vs Kp_full=69.6   (1.9× gap; simple underestimates wind floor)
    MARGINAL: Kp_simple=320.0 vs Kp_full=59.2   (5.4× overtuned; disturbance-free → search ceiling wins)
    FRAGILE:  Kp_simple=88.8  vs Kp_full=88.8   (medians match but distribution is bimodal;
              some picks below floor, some above ceiling → 57.8% window miss)

  Key insight: simple model is NOT dangerous (0.0% false approvals across all regimes).
  FRAGILE designs: 57.8% get falsely REJECTED (gain too low OR too high — bimodal).
  EASY designs: 9.8% false rejection (slightly better than old space 12.2%).
  MARGINAL: 60% false rejection — disturbance-free search hits ceiling (Kp=320) while
    true optimal is ~59, so MARGINAL appears to need ultra-high gain for no reason.
  False rejection confirmed WORSENED vs old Kp=2 baseline, confirming root cause is
  absence of disturbance physics (not initial conditions).

  OLD SPACE (n=1200, Iyy=[0.010,0.040]) — SUPERSEDED. Do NOT cite:
  EASY 12.2%, MARGINAL 54.5%, FRAGILE 56.2%. Qualitative pattern confirmed; new rates are definitive.

  ⚠️ RE-DERIVED ON FINAL (v2) POPULATION (2026-06-15): the table above used the OLD 3-seed
  regime labels AND, critically, used each design's frozen (possibly suboptimal) best_Kp as
  the "full-physics reference" gain — exactly the optimality problem found in MAJOR CORRECTION
  v2. Re-derivation (merging exp4_s2r_gains_py.csv's simple-tuned SR with exp1_final_correction
  _py.csv's finer-search 30-seed nominal SR for the 241 at-risk designs, final regime labels):

  | Regime     | n    | SR(simple→full) | SR(full→full, re-optimized) | Gap   | False Approval | False Rejection |
  |------------|------|-----------------|------------------------------|-------|-----------------|-------------------|
  | EASY       | 2362 | 0.861           | 0.999                        | 0.138 | 0.0%            | 10.1%             |
  | FRAGILE    | 36   | 0.361           | 0.760                        | 0.399 | 0.0%            | 63.9%             |
  | INFEASIBLE | 2    | 0.667           | 0.233                        | -0.433| **100.0%**      | 0.0%              |

  ⚠️ THE "SIMPLE MODEL IS NEVER DANGEROUS" CLAIM IS FALSE. With only 2 genuinely INFEASIBLE
  survivors (down from 3, after correcting the gain-search confound), BOTH show false approval:
  the simple (disturbance-free) model rates them as GO (SR_simple≈0.67, since with no wind any
  Kp looks fine) while the best achievable full-physics SR — even with the finer joint search
  hitting Kp up to 320 (search ceiling) — is only 0.17-0.30, genuinely below the 0.35 INFEASIBLE
  cutoff. n=2 is too small for a rate estimate (could be 50-100% true rate), but the DIRECTION
  is now opposite to the old claim: for the rare design that is genuinely uncontrollable in real
  wind, a disturbance-free simple simulator is actively dangerous, not merely overly conservative.
  This restores the pre-slew-fix "INFEASIBLE is a real danger zone" intuition that the slew-bug-fix
  and 3-seed correction history had appeared to kill — it was never fully dead, just shrunk to
  0.08% of the population (2/2400) and obscured by gain-search noise in the larger at-risk set.
  PRACTICAL IMPLICATION: a builder relying solely on a disturbance-free simulator has no warning
  signal for the rare truly-infeasible design — this is the strongest argument in the whole
  project for flight testing (or at minimum, a windy simulator) before committing to hardware.

  EASY/FRAGILE false-rejection rates are essentially unchanged (10.1% vs 9.8%; 63.9% vs 57.8% —
  FRAGILE rose slightly because the v2-corrected FRAGILE population skews to higher td/lower SR).
  Data: experiments/results/exp4_s2r_gains_final_py.csv.

FINDING 5 (FLIGHT SIGNATURE, RE-DERIVED 2026-06-15 on final n=36 FRAGILE population):
  Single test flight at Kp=2 detects FRAGILE designs. AUC=0.954 [0.907, 0.989] from 7-seed mean RMS
  (was 0.943 [0.888, 0.984] on the stale n=45 population — CONFIRMED, slightly strengthened).
  Class separation also improved: ratio 3.53× (was 2.90×); FRAGILE RMS=13.3°±5.2° vs EASY
  RMS=3.8°±2.7°. Recommended threshold UNCHANGED at 7.6° (now F1=0.89, the best of any
  threshold tested); 6.0° remains the best-recall option (rec=0.94, F1=0.88).
  AUC(1-seed)=0.921 (up from 0.853) — single-flight detection is now more reliable because the
  corrected FRAGILE population is more severe on average (mean td 168.2 vs 124.5 rad/s²).
  Data: experiments/results/flight_sig_final_py.csv (504 rows, 36 FRAGILE + 36 EASY, 7 seeds each).
  Original n=45 data (flight_sig_updated_py.csv) is superseded; kept below for audit trail.

  ORIGINAL RUN (2026-06-13, n=45 FRAGILE, superseded by above):
  Data: experiments/results/flight_sig_updated_py.csv (630 rows, 45 FRAGILE + 45 EASY, 7 seeds each)

  UPDATED AUC by method (n=45 FRAGILE, 45 stratified EASY):
    RMS from 7 seeds:        AUC=0.943 [0.888, 0.984]  ← DEFINITIVE
    RMS from 1 seed:         AUC=0.853
    theta_ddot alone:        AUC=0.935  (on same balanced sample)
    Flight vs spec delta:    +0.008  (marginal; both methods nearly equivalent)

  KEY INSIGHT: On old n=16 FRAGILE labels, flight detection (AUC=0.870) appeared much better
  than spec-alone (AUC=0.662) on the same 41 designs — a +0.208 gap. On current n=45 FRAGILE
  labels, both are AUC≈0.94. The old gap was an artifact of the biased 41-design sample
  underrepresenting the spec formula's true power (pop-level AUC=0.944). Flight detection
  is now correctly understood as an empirical COMPLEMENT to the spec formula, not a superior
  predictor. Value is practical: builders without precise Iyy measurements can use flight test
  instead of computing theta_ddot from specs.

  CLASS SEPARATION at Kp=2 (7-seed means, n=45 FRAGILE labels):
    RMS: FRAGILE=11.0° (+/-4.4°) vs EASY=3.8° (+/-2.6°), ratio=2.90×

  DETECTION THRESHOLDS (7-seed mean RMS, n=45 FRAGILE):
    RMS > 5.0° (Youden-J): TP=45 FP=8 FN=0 TN=37 | prec=0.85 rec=1.00 F1=0.92 (zero FNs)
    RMS > 6.0° (recommended): TP=39 FP=5 FN=6 TN=40 | prec=0.89 rec=0.87 F1=0.88
    RMS > 7.6° (old threshold): TP=31 FP=4 FN=14 TN=41 | prec=0.89 rec=0.69 F1=0.78
    RMS > 11.0° (conservative): TP=22 FP=1 FN=23 TN=44 | prec=0.96 rec=0.49 F1=0.65
  RECOMMENDED threshold: RMS > 6.0° (F1=0.88, 87% recall, 89% precision, 11% false alarm rate)
  The old 7.6° threshold was on n=16 FRAGILE and is no longer optimal. Use 6.0°.

  WIND CONFOUND CHECK (original study, old labels — wind independence still valid):
    Low wind: AUC=0.924, Mid wind: AUC=1.000, High wind: AUC=0.986
    Detection is NOT a wind confound.

  PHYSICAL EXPLANATION: FRAGILE designs at Kp=2 enter bang-bang oscillation (slew_sat~0.66)
  because Kp=2 is below kp_floor. The servo permanently saturates trying to reject wind
  disturbances with insufficient proportional gain → persistent large-amplitude oscillations.
  Missed FRAGILE (6 FNs at thresh=6.0°) have LOW floor (latency-driven) so Kp=2 is not far
  below floor → low oscillation → not detected. False alarms have HIGH floor but WIDE window.

  PRACTICAL WORKFLOW:
    Step 1: Compute theta_ddot_max = T × sin(max_gimbal_rad) × l_nozzle / Iyy from specs
    Step 2: If theta_ddot_max > 55 rad/s², flag for careful gain selection
    Step 3: Fly at Kp=2; if RMS > 6.0°, confirmed FRAGILE → tune Kp=40-80
    Step 4: If RMS < 6.0° but theta_ddot > 55, confirm with multiple flight seeds

  CAVEAT: Tested on 90 designs (45F + 45E) from [60,200] design space. Threshold (6.0°)
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

FINDING 7 (ADRC DISSOLUTION TEST, 2026-06-15, n=36 FRAGILE final population): the FRAGILE
class is mostly a PID-architecture artifact, not a fixed physical wall — with one
latency-driven exception that the existing theta_ddot x latency mechanism already predicts.

MOTIVATION: every prior ADRC result (Finding 6, old new-direction-adrc.md memory) was computed
on STALE FRAGILE populations (n=16-25, pre-correction). This is the first ADRC test run on the
v2-final, gain-search-corrected n=36 FRAGILE population (see "MAJOR CORRECTION v2" above), and
the first to test a SINGLE UNIVERSAL (not oracle-tuned, not per-design-searched) ADRC setting
against the best-effort PID gains found by the same 126-combo joint search used to certify the
population itself — the fairest possible comparison.

PROTOCOL (tools/adrc_fragility_dissolution_test.py): all 36 final-FRAGILE designs + a
td-stratified 36-design EASY sample, exact attitude-hold task that defines FRAGILE (theta_ref=0,
t_end=3s, full physics incl. wind), 20 fresh seeds (3001+). PID uses the finer-search gains
from exp1_final_correction.py (best achievable, not a strawman). ADRC uses ONE fixed setting
for every design: omega_c=5, omega0=25, b0=keff_full computed directly from specs (no fitting,
no per-design search).

RESULTS:
  FRAGILE (n=36): PID best-effort SR=0.776, RMS=5.3°  ->  ADRC universal SR=0.962, RMS=2.8°
  EASY    (n=36): PID best-effort SR=0.996, RMS=1.2°  ->  ADRC universal SR=1.000, RMS=0.7°
  PID  FRAGILE-EASY SR gap: 0.219
  ADRC FRAGILE-EASY SR gap: 0.038  (83% of the gap closed by a single untuned ADRC setting)
  15/17 FRAGILE designs with PID SR<0.80 reach ADRC SR>=0.80 (88% conversion rate)
  Only 2/36 FRAGILE designs still have ADRC SR<0.80 at this fixed setting (R2072: td=351,
    lat=6, ADRC SR=0.50; R2080: td=312, lat=5, ADRC SR=0.75) — both are the two most extreme
    combinations of authority AND latency in the whole FRAGILE population.

WITHIN-FRAGILE CORRELATION: r(td, adrc_sr) = -0.577 (residual difficulty still scales with
  theta_ddot, but far more weakly than r(td, pid_sr) = -0.249 in the wrong direction would
  suggest — PID's failure is NOT cleanly td-ordered because it is bimodal/ceiling-floor driven,
  while ADRC's residual failure IS cleanly td-ordered, i.e. closer to a real physical limit).

FOLLOW-UP ON THE 2 RESIDUAL FAILURES: swept omega_c x omega0 for R2072 (worst case, td=351,
  lat=6). Fixed setting (wc=5, w0=25) -> SR=0.50. Lowering wc to 3 (slower closed-loop bandwidth,
  appropriate for high-latency loops per standard ADRC bandwidth-separation guidance) at
  w0 in {35,50,70,100} -> SR=1.00 in all cases. The residual failure is NOT an ADRC ceiling —
  it is solved by the SAME mechanism already documented for PID (Finding 2, H5 latency): high
  latency_steps demands a slower controller bandwidth. A single universal (wc, w0) pair cannot
  be optimal for both lat=1 and lat=6 designs; a latency-conditioned ADRC rule (e.g. wc=3 when
  latency>=5) would likely close the gap to ~100% with no per-design tuning.

INTERPRETATION: this is a more careful and more credible result than the prior (superseded)
"ADRC achieves SR=1.000 for ALL FRAGILE designs" claim in new-direction-adrc.md, which used the
stale n=25 population and an oracle-style omega0=10 search. The corrected picture: FRAGILE is
~94% (34/36) a PID-specific artifact that a fixed, non-oracle ADRC setting dissolves outright,
and the remaining ~6% (2/36, both extreme-latency) is the SAME theta_ddot x latency mechanism
already established for PID, now showing up as an ADRC bandwidth-tuning requirement instead of
an unconditional success. This is a STRONGER claim than "ADRC fixes everything" because it
shows the predictor (theta_ddot x latency) generalizes ACROSS controller architectures — it is
not a PID-specific artifact of the prediction itself, only of the failure mode it predicts.

Files: tools/adrc_fragility_dissolution_test.py, experiments/results/adrc_dissolution_py.csv.
Supersedes: new-direction-adrc.md's n=25 oracle-tuned claim (omega0=10, SR=1.000 universally) —
that population is stale and that omega0 choice was not stress-tested against extreme-latency
designs, which did not exist in the same proportion in the old n=1200 design space.

FINDING 8 (SATURATION MECHANISM TEST, 2026-06-18, tools/adrc_saturation_test.py, n=15):
Causal isolation of WHY ADRC outperforms PID at high Pi using a 2×2 factorial design:
{PID, ADRC fixed omega_c=5} × {saturation on (real hardware), saturation off (ideal servo)}.
15 designs: 10 EASY + 5 FRAGILE, Pi range 27–12,648. Seeds 10001-10015.

THREE KEY FACTS:
1. PID-nosat = 1.000 for ALL 15 designs including Pi=12,648 (PID normally SR=0.600 there).
   Slew saturation is both necessary AND sufficient for PID failure in this Pi range.
2. slew_frac_adrc = 0.000 for all 15 designs. ADRC's ESO keeps effective gain low
   (omega_c²/b0 ≈ 2-5) vs PID's required Kp (40-80) → servo never saturates. At Pi=5214,
   PID saturates 70.8% of timesteps while ADRC saturates 0%.
3. Removing saturation does NOT improve ADRC: ADRC-sat = ADRC-nosat for all designs.
   ADRC's failure at Pi=5214 (SR=0.867) and Pi=12648 (SR=0.533) is bandwidth-limited
   (fixed omega_c=5 is suboptimal for extreme latency=6 designs), not saturation-limited.

CORRELATIONS: rho(Pi, slew_frac_pid) = +0.875, p=2.0e-5. Pi predicts the MECHANISM (how
  often PID saturates) with the same strength it predicts the OUTCOME. This closes the
  causal loop: Pi → saturation rate → SR.

CARLSON RECONCILIATION (2026-06-16): Without saturation (linear regime), PID = ADRC = 1.000
  for all 15 designs — exactly the Carlson (2025) equivalence. The departure from equivalence
  is ENTIRELY in the saturated conditions. "Two conventions divergent under saturation" is now
  empirically confirmed, not just argued from the theorem.

NUANCE: Fixed omega_c=5 ADRC underperforms optimal PID at Pi=12,648 (0.533 vs 0.600).
  The full ADRC advantage (performance_frontier: ADRC SR=0.733 at same design) requires
  both ESO (prevents saturation) AND appropriate omega_c tuning (Section 4.0.3). This
  experiment isolates the mechanism; it does not replace the frontier result.

Files: tools/adrc_saturation_test.py, experiments/results/adrc_saturation_test_py.csv.

Confidence: HIGH for FINDING 1, HIGH for FINDING 4 (S2R, n=1200), HIGH for FINDING 2.
HIGH for FINDING 5 (n=50, clean wind confound check, zero false positives at 1.00 threshold).
HIGH for FINDING 7 (n=36 final population, fair best-effort-PID-vs-universal-ADRC comparison,
residual failures mechanistically explained and resolved by an independent bandwidth sweep).
HIGH for FINDING 8 (n=15, 2×2 factorial design, three independently-confirmable facts;
  PID-nosat=1.000 is the cleanest single number in the project).
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

## Autotune Methodology Sensitivity (updated 2026-06-13)

Current autotune: autotune_continuous (2-seed SR primary, RMS tiebreak; Kp=[1,320] log-search).
Old grid (5×5 KP=[2,5,15,40,80], KD=[1,2,8,16,32]) was REPLACED due to Kp=80 hard cap causing
9 genuinely-solvable FRAGILE designs to be misclassified (needed Kp=88-320).

Current robustness methodology: 3-seed over/under test with ROBUSTNESS_SUCCESS_RATE=0.80.
Old methodology used 1 seed (binary pass/fail, threshold=0.35 for both nominal and robustness).
The 3-seed test detects true p_fail=0.14 designs with 36% probability (vs 14% with 1 seed).
Threshold=0.80 with 3 binary seeds means: SR ∈ {0, 0.333, 0.667, 1.0}; any single seed failure
at ±40% Kp triggers FRAGILE. This is the main reason FRAGILE count rose from 16→19.

Sensitivity comparison (old space Iyy=[0.010,0.040], n=1200):
* Old grid (capped at 80):              EASY=1169, MARGINAL=6,  FRAGILE=25, INFEASIBLE=0
* autotune_continuous + 1-seed:         EASY=1171, MARGINAL=11, FRAGILE=16, INFEASIBLE=2
* autotune_continuous + 3-seed (0.80):  EASY=1178+Δ (pending re-run on old space)

New space (Iyy=[0.005,0.100], mass=[0.50,1.20], n=1200, autotune_continuous + 3-seed):
  EASY=1178, MARGINAL=2, FRAGILE=19, INFEASIBLE=1

Practical guidance: Report EASY+MARGINAL as "controllable" (>98%) and FRAGILE+INFEASIBLE
as "at-risk" (~2%). The 3-seed test is the current standard; old 1-seed counts are superseded.

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
