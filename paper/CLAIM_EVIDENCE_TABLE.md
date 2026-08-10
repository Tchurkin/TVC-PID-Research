# Claim / Evidence Table — post-audit, 2026-08-04

Reference sheet for writing the STS report. Every surviving claim with its statistic, its n, and the
file that proves it. **Numbers only — the prose is Braxton's.**

Recover the sim-phase CSVs first: `git archive bb22d36^ experiments/results tools | tar -x -C <scratch>`

---

## C0 — The failure map: who fails  *(LEAD FINDING, §4 — added 2026-08-09 per the emphasis ruling)*

This section did not exist while C1 was "the headline." It is now what the paper leads with.

**C-INERTIA — failing builds are low-inertia builds.**

| quantity | value |
|---|---|
| **P(random failure has lower Iyy than random survivor)** | **0.937** — Mann-Whitney **p = 9.45e-20**, rank-biserial **0.875** |
| **median failing design's position in the population Iyy distribution** | **5.4th percentile** (IQR 1.1-8.9); 17/36 below the 5th, 28/36 below the 10th |
| median Iyy of failing designs | **0.19×** the population median |
| ~~failing designs below the 25th pctile~~ | ~~36 of 36~~ — **RETIRED 2026-08-10, selected threshold** |

⚠ **Do not use the 25th-percentile framing.** It is true, but the binding failure sits at the
**24.79th** percentile, so 25 is the smallest round number producing 100% and reads as a cutoff chosen
to flatter (33 of 36 at the 20th). The threshold-free statistics above are stronger *and* carry a
p-value. Caught by an adversarial check of §4's lead claim, 2026-08-10.
| unregularized joint fit, standardized, bootstrap 95% CIs | log(1/Iyy) **+2.37** [1.94, 3.14] · log(T·L) **+1.77** [1.30, 2.45] · log(τ) **+2.90** [2.24, 3.92] |

All three coefficients positive with CIs excluding zero — **do not claim thrust is protective**. An
earlier "it is inertia, not authority" reading came from L2 regularization penalising a manually-added
intercept column and was retracted.

**C-FRONTIER — achievable performance degrades with authority×delay.** ρ = **−0.692**, p = 3.4e-10,
n = 63, under per-design optimal tuning with fresh evaluation seeds.

### ⚠ The protocol defence — the single most load-bearing sentence in the paper

A reviewer who has read §6 will ask: *you showed your tuning protocol manufactures spurious
hardware-indexed structure; why is the inertia result not the same artifact?* The answer is
**structural, not statistical**, and it settles the question outright:

| section | tuner | does D go stale while P moves? |
|---|---|---|
| §6 (the artifact) | `autotune_continuous` — probe Kd once at Kp = 40, freeze, sweep Kp to 320 | **yes — that is the mechanism** |
| §4 / §5 (population labels) | `autotune_grid` — nested double loop over Kp × Kd, **5 × 5 = 25 combinations scored jointly**, 2 search seeds each | **no — impossible by construction** |

`sim/experiment_runner.py:205`, grids at `:69`. Because `autotune_grid` never holds D fixed while P
sweeps, the mismatch that generates the artifact cannot arise in the data behind C-INERTIA or
C-FRONTIER. Corroborating but *not* load-bearing: re-tuning the whole population on the restored
simulator moved the labels **1.9%**.

**State the structural argument first, in §3 and again in §4.** It was documented nowhere until
2026-08-09 — the paper could not previously defend its own lead claim.

**Evidence:** `exp1_final_population_py.csv`; §4.0.3 frontier data; `tools/population_retune.py`

## C1 — Stale-D tuning fails; preserving the P:D ratio fixes it  *(finding 3, §6 — NOT the headline; see the emphasis ruling)*

| | auth×delay < 300 | auth×delay ≥ 300 | ρ vs auth×delay |
|---|---|---|---|
| **Arm A** — stale D (D fixed at P=40, P swept to 320) | 0.087 | **56 of 80 = 0.700** | **+0.236** (p = 1e-31) |
| published result | 0.091 | 0.613 | — |
| **Arm B** — ratio preserved (Kd = 0.05·Kp) | 0.022 | **1 of 80 = 0.0125** | −0.117 (wrong sign) |

Bin-by-bin reproduction (Arm A vs published): 0.060/0.068 · 0.084/0.093 · **0.120/0.120** ·
**0.238/0.238** · 0.386/0.313 · 0.673/0.582 · 0.783/0.696

- n = 2400, arms paired on the same designs. Arm A uses seeds (1,2,3) — the original's seeds, because
  reproducing means reproducing. Arm B uses 340001–340015.
- Mechanism: `autotune_continuous` probes Kd once at Kp = 40, freezes it, sweeps Kp to 320.
- **Evidence:** `tools/s2r_replication.py` → `experiments/results/s2r_replication_py.csv`; commit `589c29e`
- **Must state (framing ledger, 2026-08-09):** call this **stale-D**, never "sequential tuning" —
  manual practice is *alternating* and preserves the ratio, so it sits outside this regime. External
  validity is a **prescription** (maintain the P:D ratio; re-tune D after any P change before judging
  stability) aimed at **autotuners and sim campaigns**, not an ethnographic claim about builders.

## C2 — The gain ceiling is set by delay and is independent of authority

| quantity | value |
|---|---|
| keff coefficient | **−0.082**, SE 0.067, t = −1.2, 95% CI **[−0.21, +0.05]** |
| τ exponent | **−1.067** (theory −1) |
| R² | 0.528 (published v2: 0.442) |
| positive control ρ(latency, ceiling) | **−0.762** vs published −0.74 |
| n | 104 uncensored, on v2's exact designs |

- Recalibrated constant **0.0661/τ** (median ratio 1.14); published 0.042/τ underpredicts ~1.6× and
  functions as a conservative bound. 5th-percentile bound 0.0231/τ. 90% of designs in [0.35×, 2.03×].
- **Two caveats that must appear:** (a) the ceiling is **8.8×** higher with Kd free (paired, 94% of
  designs, p = 4.9e-9) — state the claim as *at a fixed Kd*; (b) sample-conditional, ρ = −0.76 on
  v2's authority-stratified designs vs **−0.38** on a population-representative sample.
- **Evidence:** `tools/ceiling_kd_free.py` → `ceiling_kd_free_py.csv`; holdout `floor_formula_holdout_py.csv`

## C3 — A failing design is identifiable from one instrumented flight

| class | attitude RMS | saturated fraction |
|---|---|---|
| narrow-window | **13.3° ± 5.2°** | 0.60 |
| EASY | **3.8° ± 2.7°** | 0.14 |

AUC **0.954** [0.907, 0.989] on 7-seed means; **0.907** mean for a single flight (per-seed 0.874–0.948).

- Balanced 36/36, so the AUC flatters deployment prevalence. **Untested on hardware** — this is the
  prediction the flight campaign checks.
- **Evidence:** `experiments/results/flight_sig_final_py.csv` (72 designs × 7 seeds)

## C3b — The signature survives a pre-registered retrospective hardware test  *(new 2026-08-09)*

| pass | result | p (one-sided exact) |
|---|---|---|
| **roll-aware replay (primary)** | **5 of 5** — TP 3, FN 0, FP 0, TN 2 | **0.100** *(= the archive's ceiling, declared before running)* |
| as-logged (secondary) | 2 of 5 — TP 2, FN 1, FP 1, TN 1 | 0.700 |

Threshold **5.62°** carried from simulation with **zero parameters refit**; grey zone [5.37, 5.62]
declared in advance. Flown healthy 3.61 / 5.08° vs simulated EASY 3.8 ± 2.7 — **calibration transfers**.
Flown failures 9.61 / 58.4 / 90.0° vs simulated FRAGILE 13.3 ± 5.2 (max ≈ 25) — **it does not**.

- **Must state:** the p = 0.100 ceiling in the same sentence as the 5/5; that ASC031 clears the grey
  floor by only 0.29°; that the diagnostic inherits its estimator's failure modes (2/5 as-logged); and
  that a no-motor ground test scores 7.6°, so the rule is valid only inside a powered boost.
- **Evidence:** `tools/retro_flight_signature.py` → `paper/retro_flight_signature.csv`;
  pre-registration `paper/RETRO_FLIGHT_SIG_SPEC.md` committed at `5f49641` *before* the run; Fig 13.

## C4 — Self-correction record  *(methods chapter, not the thesis)*

| item | number |
|---|---|
| pre-registered `frozen_kd_artifact_test` | **8/8** designs past its own "significant" threshold; floor inflated 1.8–2.9× |
| floor law across protocols | keff exponent **+1.06 → +0.21 → −0.20** (v2 → v4 → v5) |
| causal saturation claim | **128/142** designs already at SR = 1.000 with saturation ON; paired Δ **+0.005**; Wilcoxon **p = 0.128** |
| the n=15 factorial | rests on **2** designs; ADRC *worse* than PID (0.533 vs 0.600) at R2072 |
| binary label softness | **29 of 36** positives flagged `uncertain` |
| AUC across passes | not comparable — labels changed between passes |
| `p_unstable` | r = **−0.004** (p = 0.84); legacy, unread by the EOM; column inverted for 1196/2400 rows |
| `validate.py` | one check asserted **nothing** for its entire existence (`theta_true` constant across 601 samples) |

- The 2026-06-22 experiments (`frozen_kd_artifact_test`, `window_ratio_v4/v5/v5b`) were run and
  **never written into the research log** — which is why the paper kept building on `window_ratio`.
- **Evidence:** those tools + CSVs at `bb22d36^`; `archive/CLAUDE_pi_research_2026-06.md`

---

## Retired — do not restate as findings

Π = keff·τ² as a parameter · the causal saturation-removal claim · the entire `window_ratio` family
and the controller-invariance results scored on it · "each correction strengthened the AUC" ·
the four-number screen as a *controllability* screen (AUC 0.985 on the original label,
**0.57** at 2×/3× probes; 5 of 16 original positives reappear) · "strong p_unstable interaction"

---

## Claims ledger — framing constraints (violations must not drift back in)

| date | ruling | why |
|---|---|---|
| 2026-08-09 | **The mechanism is "stale-D" / "decoupled" tuning — D held fixed while P sweeps. NEVER call it "sequential tuning."** | Readers hear "sequential" as ordinary manual practice. Real manual practice is **alternating** (adjust P, re-adjust D, repeat), which roughly preserves the ratio and sits **outside** the artifact regime. *(Braxton, domain knowledge.)* |
| 2026-08-09 | **No claim that common practice sits inside the artifact regime.** External validity is a **prescription**, not an ethnography: *maintain the P:D ratio; re-tune D after any change to P before judging stability.* Audience: **autotuners and simulation campaigns.** | This project measured **zero** manual tuning. The evidence is about what an autotuner does — which is exactly the protocol that produced six retired sections of this project's own prior work. |
| 2026-08-09 | **PREVALENCE + FRAMING for C-KD.** Never claim the decoupled protocol is *common*; prevalence elsewhere is **unknown and stated as unknown**. C-KD is **conditional**: *if* a pipeline decouples the gains, this is what it manufactures. And the result is the **masquerade, not the crashing** — stale-D fails *selectively, in proportion to hardware* (~4× worse at low authority×delay, ~56× at high), so it emerges as a smooth dose-response that reads as a physical law. Also: never write "it was all an artifact" — the ceiling survived the same test. | "Bad tuning breaks rockets" is trivial. That an inadequate protocol produces *counterfeit laws* rather than noise is not, and the existence proof is this project's own six retired sections plus the pre-registered test dated before the run. The survives/dies contrast is what shows the diagnostic discriminates. *(Braxton, via counsellor relay.)* |
| 2026-08-09 | **EMPHASIS (final).** The paper is the **positive characterization**. Abstract and §1 lead with the failure map + inertia (C-INERTIA, C-FRONTIER) and the 1/τ ceiling (C-CEILING). The tuning result (C-S2R, C-KD) is **finding 3**, framed as a **discovery about tuning methodology** — prescription + warning to autotuners and sim campaigns. **Not a retraction story.** The six retired sections are **one paragraph**; §8 is **~2 pp**; nothing is labelled "the headline." | The findings stand on their own and a reader meeting this work for the first time should get the science, not the correction history. An audit-led paper reads as a confession and buries two results that are independently worth reporting. *(Braxton, final ruling.)* |
| 2026-08-09 | Axis called **authority × delay**, a plotting coordinate. Never Π, never "dimensionless", never "invariant". | Braxton's claims-out list. Four self-inflicted violations were caught in the outline before it shipped. |
| 2026-08-09 | Report **counts** for the headline (56/80 vs 1/80), not a one-decimal rate. | 1/80 = 1.25%; "1.3%" and "1.2%" are both rounding artifacts of a single event. 95% Wilson CIs [59.2%, 78.9%] vs [0.22%, 6.75%] are non-overlapping — the honest way to show it survives its own uncertainty. |

**Consequence for gap G1.** The firmware survey was scoped to establish that sequential tuning is
the community's practice. That claim is now **withdrawn as unsupportable**, so G1 no longer buys
external validity for C-KD and drops well down the priority list. If it is run at all, the only
question it can answer is narrower and more honest: *do any hobby TVC firmwares ship an autotuner,
and does it hold D stale while sweeping P?* Most ship hand-set gains, so the likely finding is "not
applicable" — which is itself worth one sentence in §9 rather than a weekend of reading.

---

## Gaps, ranked by how much they'd improve the paper

**G1 — [DOWNGRADED 2026-08-09] Firmware autotuner survey.** Originally scoped to show sequential
tuning is common practice; that claim is withdrawn (see the ledger — manual practice is alternating
and preserves the ratio). What remains is narrow: do any hobby TVC firmwares ship an autotuner, and
does it hold D stale while sweeping P? Most ship hand-set gains, so expect "not applicable" — one
sentence in §9. **No longer the highest-value item; G2/G3 now lead.**

**G2 — Measure τ on the vehicle.** `Firmware/Bench_Latency/`. Every hardware prediction is indexed on
it and currently uses an assumed 0.035 s. Half a day.

**G3 — [PARTLY DONE 2026-08-09] Fly the C3 prediction.** The *retrospective* half is done (C3b): 5/5
on archived flights, p = 0.100, calibration transfer measured. What a new flight would add is an
out-of-sample case at a known gain with a sound estimator — worth having, no longer the gating item.

**G4 — Iyy / ballast sweep.** Inertia is the largest single coefficient in every fit, and ballast is
the cheapest controlled manipulation available. Note the rebuild already moved keff 257 → 130.

**G5 — Re-score LQR/SMC/MPC on a common metric.** Their own `n_pass` statistics are sound; the PID
row they were compared against used the retired `window_ratio`. ~30 lines to make §5 coherent.

---

## Report mechanics (verified against the 2027 rules)

20 pages max, **appendices count**; title page, abstract, and bibliography do not. ≥11pt, 1.5
spacing, 1" margins, single column, page numbers bottom-right after the abstract. **Every figure must
be cited, including your own** — failure to cite an image can disqualify. No links outside the
bibliography. PDF ≤ 4 MB, named `LASTNAME.FIRSTNAME.ZIPCODE`.

Deadline **2026-11-05, 8pm ET — application *and* all recommendations.** Request recommenders early
September; the same teachers are writing college letters in the same weeks.
