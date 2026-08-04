# Claim / Evidence Table — post-audit, 2026-08-04

Reference sheet for writing the STS report. Every surviving claim with its statistic, its n, and the
file that proves it. **Numbers only — the prose is Braxton's.**

Recover the sim-phase CSVs first: `git archive bb22d36^ experiments/results tools | tar -x -C <scratch>`

---

## C1 — Sequential gain tuning fails; coupled tuning fixes it  *(the headline)*

| | Π < 300 | Π ≥ 300 | ρ(log Π, fail) |
|---|---|---|---|
| **Arm A** — sequential (P then D), original protocol | 0.087 | **0.700** | **+0.236** (p = 1e-31) |
| published result | 0.091 | 0.613 | — |
| **Arm B** — coupled (Kd = 0.05·Kp) | 0.022 | **0.013** | −0.117 (wrong sign) |

Bin-by-bin reproduction (Arm A vs published): 0.060/0.068 · 0.084/0.093 · **0.120/0.120** ·
**0.238/0.238** · 0.386/0.313 · 0.673/0.582 · 0.783/0.696

- n = 2400, arms paired on the same designs. Arm A uses seeds (1,2,3) — the original's seeds, because
  reproducing means reproducing. Arm B uses 340001–340015.
- Mechanism: `autotune_continuous` probes Kd once at Kp = 40, freezes it, sweeps Kp to 320.
- **Evidence:** `tools/s2r_replication.py` → `experiments/results/s2r_replication_py.csv`; commit `589c29e`
- **Must state:** that sequential tuning is the standard method (Ziegler–Nichols is sequential) is
  currently an argument, not a citation. Needs the firmware survey — see gap G1.

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

## Gaps, ranked by how much they'd improve the paper

**G1 — Establish that sequential tuning is common practice.** Survey the three GitHub firmwares
already audited in `HEADLINE_FINAL.md`, published build logs, BPS.space's documented process. Not a
simulation. This is what makes C1 a claim about the field rather than about one script. **Highest
value remaining.**

**G2 — Measure τ on the vehicle.** `Firmware/Bench_Latency/`. Every hardware prediction is indexed on
it and currently uses an assumed 0.035 s. Half a day.

**G3 — Fly the C3 prediction.** Converts the one falsifiable hardware claim from proposed to tested.

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
