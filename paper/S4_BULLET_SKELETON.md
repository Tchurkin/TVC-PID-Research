# §4 — bullet skeleton and completeness ledger

Companion to `S4_WRITING_PLAN.md` (paragraph-level). This is the level below: **one bullet per point
you have to make**, in the order you make them, so you can expand bullets into sentences.

**Bullets are specifications, not sentences.** Nothing here is phrased so it can be pasted — that is
deliberate and required (STS Appendix 4). Where a bullet carries a number, the number is your data;
the sentence around it has to be yours.

---

## Part 1 — the skeleton

### P1 · Opener
- → open on the builder's position: parameters are fixed at build time, tunability is discovered after
- → name the four measurable quantities: **thrust, nozzle moment arm, pitch inertia, loop delay**
- → state the question as *prediction before construction*
- → say what §4 delivers: a map over those quantities
- ✗ do not preview §6, do not mention tuning method, do not use the word "artifact"

### P2 · The population
- → sampling: **2,400 designs, Latin hypercube**
- → one line on ranges; `max_gimbal_deg` **2.0–15.0°, median 8.6°** is the one worth quoting
- → full physics: ten fidelity modules, name-drop only
- → point back to **Fig 1** rather than re-describing the space
- ✗ do not re-open Methods; two sentences of scope maximum

### P3 · Fig 3, the map
- → **cite "Figure 3" by number in the running text** (uncited figure = disqualification risk)
- → axes: pitch inertia × loop delay; 2,400 points; failures marked
- → tell the reader what the eye catches *before* quantifying: failures cluster at low inertia
- → one sentence on what "failing" means here, deferring the label's softness to P10
- ✗ do not write "every failure lies left of here" — true (0.02904 vs 0.02920) but a 0.5% margin reads
  as false and invites a ruler

### P4 · The lead number
- → **36 of 36** failing designs below the population **25th percentile** of Iyy
- → median Iyy of failures = **0.19×** the population median
- → report as a **count**, not "100%" — the count carries the sample size in the same breath
- → one sentence of physical reading: disturbance angular acceleration scales as 1/Iyy while control
  authority is capped by max gimbal, so no gain choice buys it back
- ✗ do not say inertia is the *only* thing that matters — P5 immediately qualifies that

### P5 · The joint fit
- → specify the fit before the numbers: **unregularized**, standardized predictors, bootstrap 95% CIs
- → log(1/Iyy) **+2.37** [1.94, 3.14]
- → log(T·L) **+1.77** [1.30, 2.45]
- → log(τ) **+2.90** [2.24, 3.92]
- → state explicitly: all three positive, all CIs exclude zero
- → keff as a legitimate *grouping*: raw-log ratio T·L/(1/Iyy) = **1.18** where keff would require 1.00
- → dropping T·L **costs** accuracy: CV AUC **0.9829 → 0.9687**
- ✗ **claims-out:** never that thrust is irrelevant, protective, or "doesn't matter"

### P6 · The retracted reading
- → prior reading was "it is inertia, **not** authority" (log(T·L) coefficient came out −0.18)
- → cause: **L2 regularization penalising a manually-added intercept column**
- → refitting unregularized reversed it — which is why P5 specifies "unregularized"
- ✗ hard cap **2–3 sentences**. This wants to become a story. The audit is §8.

### P7 · ⚠ Protocol defence — **DO NOT CUT**
- → raise the objection yourself, in the reader's voice: *§6 shows the tuning protocol manufactures
  spurious hardware-indexed structure — why is this not the same artifact?*
- → answer structurally, not statistically:
  - §6's artifact tuner = `autotune_continuous` — Kd probed **once** at Kp = 40, frozen, Kp swept to 320
  - §4/§5 label tuner = `autotune_grid` — nested **Kp × Kd** loop, **5 × 5 = 25** combinations scored
    jointly, **2 search seeds** each
  - → therefore D is **never** held stale while P moves in the §4 data; the mechanism **cannot arise**
- → *then*, as corroboration only: full re-tune on the restored simulator moved labels **1.9%**
- ✗ do not lead with the 1.9% — it makes a definitional defence look like a statistical one
- ✗ do not footnote this, do not defer to §8, do not shorten below ~120 words

### P8 · Fig 4, the frontier
- → **cite "Figure 4" by number**
- → what is plotted: peak achievable success rate vs authority×delay
- → ρ = **−0.692**, p = **3.4e-10**, n = **63**
- → conditions: **per-design optimal tuning, fresh evaluation seeds**
- → one clause on why it survived when the window sections did not: never scored on the retired
  `window_ratio` metric
- ✗ **axis-naming rule:** "authority×delay", a plotting coordinate. Never Π, never dimensionless,
  never "a parameter I found"

### P9 · What the frontier means *(cut first if long)*
- → the degradation is **graded, not a cliff** — no threshold to hunt for
- → practical reading: a marginal design is not disqualified, it is *expensive*
- → no new numbers

### P10 · The honest limit
- → **29 of 36** positives flagged `uncertain` by the project's own Wilson-interval check
- → four-number screen: AUC **0.985** against the original label, **0.57** against an independent
  harsher probe
- → only **5 of 16** original positives fail when the gain is pushed 2×
- → land on the distinction: **the map is real; the binary line drawn on it is not sharp**
- ✗ **claims-out:** never call it a controllability screen — it predicts the original labelling
  procedure

### P11 · Close
- → restate the deliverable: screen from four measurable numbers before cutting metal
- → hand to §5: §4 asked *who* fails, §5 asks *why*
- ✗ no new numbers, no summary of the whole paper

### SIDEBAR · "What surprised a builder" (0.5 pp, boxed)
- → five rows, fully specified in `OUTLINE_STS.md` §4 — servo speed, wind, stability margin, thrust,
  inertia
- → each row: expectation → measurement → implication
- → keep the servo-speed nuance: failing designs **do** saturate (0.597 vs 0.089 of burn), so
  saturation is a **symptom** worth watching, not the cause
- ✗ two candidates were vetted and **rejected** — the broad "aerodynamics contributes nothing" slogan,
  and the dead-zone dither row. Ship neither.

---

## Part 2 — completeness ledger

Every number that must appear somewhere in §4. Tick as you place them.

| # | value | where | ✓ |
|---|---|---|---|
| 1 | 2,400 designs, Latin hypercube | P2 | ☐ |
| 2 | max_gimbal 2.0–15.0°, median 8.6° | P2 | ☐ |
| 3 | Figure 3 cited by number | P3 | ☐ |
| 4 | 36 of 36 below 25th pctile Iyy | P4 | ☐ |
| 5 | median Iyy 0.19× population | P4 | ☐ |
| 6 | log(1/Iyy) +2.37 [1.94, 3.14] | P5 | ☐ |
| 7 | log(T·L) +1.77 [1.30, 2.45] | P5 | ☐ |
| 8 | log(τ) +2.90 [2.24, 3.92] | P5 | ☐ |
| 9 | raw-log ratio 1.18 vs keff's 1.00 | P5 | ☐ |
| 10 | CV AUC 0.9829 → 0.9687 without T·L | P5 | ☐ |
| 11 | L2/intercept cause of the retraction | P6 | ☐ |
| 12 | autotune_grid = 5×5 joint, 2 seeds | P7 | ☐ |
| 13 | autotune_continuous = probe at 40, freeze, sweep to 320 | P7 | ☐ |
| 14 | 1.9% label movement (corroboration) | P7 | ☐ |
| 15 | Figure 4 cited by number | P8 | ☐ |
| 16 | ρ = −0.692, p = 3.4e-10, n = 63 | P8 | ☐ |
| 17 | 29 of 36 flagged uncertain | P10 | ☐ |
| 18 | AUC 0.985 vs 0.57 | P10 | ☐ |
| 19 | 5 of 16 fail at 2× | P10 | ☐ |
| 20 | sidebar: 0.597 vs 0.089 saturated fraction | sidebar | ☐ |

**Forbidden-move check (all must stay ☐ unchecked):**
☐ the word Π · ☐ "dimensionless" or "invariant" · ☐ thrust called irrelevant/protective ·
☐ "controllability screen" · ☐ "every failure lies left of here" · ☐ any reliance on §6 ·
☐ the dead-zone dither row · ☐ "aerodynamics contributes nothing" as a broad claim

---

## Part 3 — paper-wide coverage, so nothing is orphaned

Every surviving claim and the one section that owns it. If a claim has no section, it will be dropped;
if a section has no claim, it is padding.

| claim | owned by | status |
|---|---|---|
| C-INERTIA | §4 P4–P6 | you are here |
| C-FRONTIER | §4 P8–P9 | you are here |
| C-CEILING | §5 | numbers settled, 3.0 pp |
| C-S2R | §6 | numbers settled |
| C-KD | §6 | conditional; prevalence stated unknown |
| C-FLIGHT | §7 | AUC 0.954 |
| **C3b — retrospective test** | §7 | **new 2026-08-09**, 5/5, p = 0.100 |
| C-AUDIT | §8, ~2 pp | retired work = ONE paragraph |
| measured τ | §7 + Fig 11 | ⚠ **still missing — only open Sep-7 gate item** |

**Two paper-wide obligations easiest to forget while drafting:**
- **Every figure cited by number in running text, including your own.** Uncited figures can disqualify.
- **§7 must say "slew-rate saturated fraction"**, never bare "saturated fraction" — the published
  0.60/0.14 is a rate limit, and the real flights hit the *position* stop, which is identically zero
  across all 504 simulated runs.
