# §4 — everything needed to write it, from scratch. Self-contained.

**Nothing else needs opening.** The paragraph plan, the bullet skeleton, the outline's §4 notes, the
sidebar rows and the claim/evidence entry are all folded in here. If a fact isn't in this file, §4
doesn't need it.

**Bullets are specifications, not sentences.** Every number here is your data; the sentence around it
has to be yours. AI-drafted report prose is disqualifying under STS Appendix 4.

---

# PART 0 — HOW TO WORK

**Target: 1,100–1,300 words** of body prose plus two figures, inside **3.75 pp** (3.25 body + 0.5
boxed sidebar). Eleven paragraphs. If you reach 1,600 words you've overwritten by a page — cut P9
first, then P11, never P7.

**Section job:** answer *which builds fail, and can you tell before you build one?* This is the title
section and the paper's lead finding. A reader who stops after §4 should still have gotten the paper.

**Suggested order.** Write **P4 first** — it's the lead number and the section's spine, and once its
voice is set the rest follows. Then P3 (the figure it sits on), then P1–P2, then P5–P7, then P8–P11.
The sidebar last; it's a table, not prose.

**Loop:** send me one paragraph at a time and I'll attack it immediately. Critique is unrestricted.
Word-level grammar polish is available later and gets logged as `LANG` per Appendix 4.

### Four facts that are easy to get wrong
These are the ones a controls judge checks, and three of them have already been stated incorrectly in
this project — including once by me.

| don't write | do write | why |
|---|---|---|
| "6 DOF" | **single-axis (planar pitch)** | `sim/simulator.py` carries two state variables: one angle, one rate. The 6DOF quaternion sim is `Firmware/sim_ascent/ascent_sim.cpp` — a *different program* for firmware SIL, not this campaign. |
| "peak RMSE > 20°" or any single failure number | **four simultaneous gates** (§A) | No single-number criterion exists in the code. "Peak RMSE" also conflates two separate gates. |
| "ten physics modules" | **eleven** | 7 original + 4 added 2026-06-04. |
| modules "shown in Fig. 1" | **no figure shows them** | Fig 1 is design-space coverage. Cite it for coverage; list the modules in text. |

---

# PART A — REFERENCE FACTS

Verified against the code, 2026-08-10.

### The simulator
- **Single-axis planar pitch.** State: `theta` (angle), `q` (rate). `sim/simulator.py:152`
- **dt = 0.005 s (200 Hz)**; run length **t_end = 3.0 s**
- Reference **θ_ref = 0°**, initial condition **θ₀ = 0°** — the task is *hold vertical under
  disturbance*, not recover from an initial offset
- Deterministic: same seed → identical trajectory

### The eleven fidelity modules (`sim/fidelity_config.py:114`)
wind · backlash · slew rate limit · latency · sensor noise · thrust variation · deadband ·
nonlinear aero (sin θ vs θ) · dynamic pressure from integrated velocity · realistic thrust curve T(t) ·
CG/inertia shift from propellant burn

### Success gates — ALL FOUR must hold simultaneously, per run (`experiment_protocol.py:75`)
| gate | quantity | threshold |
|---|---|---|
| stability | max_theta_deg | < 70° |
| tracking | rms_error_deg | < 15° |
| terminal | end_error_deg | < 15° |
| peak | peak_error_deg | < 50° |

### Labels (`sim/units.py:100`)
- **EASY** — robustness = 1.00 (all three gain sets pass) **and** success rate ≥ 0.80, RMS ≤ 8°,
  saturation fraction ≤ 0.60, settling ≤ 3.0 s, oscillation score ≤ 3.0
- **FRAGILE** — success rate ≥ 0.35, RMS ≤ 16.0°, but fails the EASY quality gates
- Robustness is tested at **±40% Kp/Kd**, 3 seeds, all 3 must pass
- Population: **2,400 designs = 2,362 EASY + 36 FRAGILE + 2 INFEASIBLE**
- In §4 you may call FRAGILE "failed to tune" — Fig 3's legend already does

### The two tuners — this distinction is the whole of P7
| | `autotune_grid` (§4/§5 labels) | `autotune_continuous` (§6 only) |
|---|---|---|
| search | nested **Kp × Kd** double loop | Kd probed **once** at Kp = 40 over {1, 4, 16, 64}, then **frozen** |
| grid | Kp ∈ {2, 5, 15, 40, 80} × Kd ∈ {1, 2, 8, 16, 32} = **25 pairs** | Kp then swept 1 → 320 |
| seeds | 2 search seeds | — |
| D goes stale? | **no — impossible by construction** | **yes — that is the mechanism** |
| where | `experiment_runner.py:205`, grids at `:69` | `experiment_runner.py:246` |

### The figures §4 owns
- **Fig 3** `fig03_failure_map.png` — x = pitch inertia (log), y = loop delay in control steps.
  2,364 tunable (small blue) vs 36 failed (large orange). Two dashed median lines: orange = failed
  median **0.0102**, blue = tunable median **0.0534**. No cutoff line.
- **Fig 4** `fig04_performance_frontier.png` — x = authority×delay (log, a plotting coordinate),
  y = peak achievable success rate, with a rolling-median trend line.
- **Fig 1** (owned by §3) — design-space coverage, Iyy × latency coloured by keff. Cite for coverage.

---

# PART B — THE ELEVEN PARAGRAPHS

### P1 · Opener — ~80 words, no figure
- → the builder's position: parameters are fixed at build time, tunability is discovered after
- → name the four measurable quantities: **thrust, nozzle moment arm, pitch inertia, loop delay**
- → frame the question as *prediction before construction*
- → say what §4 delivers: a map over those quantities
- ✗ don't hedge with "parameters **like** thrust…" — these are the complete set and the paper's spine
- ✗ don't preview §6, don't use the word "artifact"

### P2 · The population — ~90 words
- → **2,400 designs, Latin hypercube**
- → `max_gimbal_deg` **2.0–15.0°, median 8.6°** is the range worth quoting
- → **eleven** fidelity modules, **single-axis pitch** dynamics
- → cite Fig 1 for coverage only
- ✗ two sentences of scope, then move — don't re-open Methods

### P3 · Fig 3, the map — ~120 words — **TITLE FIGURE**
- → **cite "Figure 3" by number in the running text** (uncited figure = disqualification risk)
- → axes: pitch inertia × loop delay; 2,364 tunable vs 36 failed
- → say what the eye catches *before* quantifying: failures cluster at low inertia
- → state the real failure definition (the four gates), one sentence
- ✗ never "every failure lies left of here" — true (0.02904 vs 0.02920) but a 0.5% margin reads as
  false and invites a reviewer with a ruler

### P4 · The lead number — ~110 words — **write this one first**
- → **93.7%** — a randomly chosen failing design has lower pitch inertia than a randomly chosen
  surviving one
- → **Mann-Whitney p = 9.45 × 10⁻²⁰**, **rank-biserial 0.875**, n = 36 vs 2,364
- → median failing design at the **5.4th percentile** of the population inertia distribution
  (IQR 1.1–8.9)
- → **17 of 36** below the 5th, **28 of 36** below the 10th
- → median Iyy of failures **0.19×** the population median
- → say the distribution is a **gradient, not a cutoff** — this is the sentence that protects the
  paragraph
- → mechanism: disturbance angular acceleration scales as 1/Iyy while control authority is capped by
  max gimbal, so no gain choice buys it back
- ✗ **never** "all 36 below the 25th percentile" — the binding case sits at the **24.79th**, so 25 is
  the smallest round number that gives 100%; at the 20th it's 33 of 36
- ✗ don't attach "the graph shows" to a computed statistic — figures show, statistics are computed
- ✗ don't say inertia is the *only* thing that matters; P5 qualifies it immediately

### P5 · The joint fit — ~140 words
- → specify the fit **before** the numbers: unregularized, standardized predictors, bootstrap 95% CIs
- → log(1/Iyy) **+2.37** [1.94, 3.14]
- → log(T·L) **+1.77** [1.30, 2.45]
- → log(τ) **+2.90** [2.24, 3.92]
- → all three positive, all CIs exclude zero
- → keff = T·L/Iyy is a legitimate *grouping*: raw-log ratio **1.18**, where keff would require 1.00
- → dropping T·L costs accuracy: CV AUC **0.9829 → 0.9687**
- ✗ **claims-out:** never that thrust is irrelevant, protective, or doesn't matter
- **This must be prose, not a bulleted list.** A results section with a raw list of coefficients reads
  as notes.

### P6 · The retracted reading — ~60 words, two to three sentences
- → the prior reading was "inertia, **not** authority" (log(T·L) came out **−0.18**)
- → cause: **L2 regularization penalising a manually-added intercept column**
- → refitting unregularized reversed it — which is why P5 specifies "unregularized"
- ✗ hard cap at three sentences. This wants to become a story; the audit is §8.

### P7 · ⚠ PROTOCOL DEFENCE — ~130 words — **DO NOT CUT**
Raise the objection in the reader's voice, then answer it. The argument as a chain:
1. The §6 artifact arises from one mechanism: a design's best gain lands far from where D was probed,
   so D is mismatched to the final P — and the mismatch **grows with authority×delay**.
2. That mechanism **requires** a tuner that fixes D once and then moves P away from it.
3. `autotune_grid` scores **(P, D) pairs**, 25 of them, each on its own merits. There is no "the D
   value" for P to move away from.
4. ∴ the mechanism **cannot operate** on §4's data. Not "we checked and it didn't happen" — it cannot
   happen.
- → *then*, as corroboration only: full re-tune moved labels **1.9%**
- ✗ don't lead with the 1.9% — it makes a definitional defence look statistical
- ✗ don't footnote it, don't defer to §8, don't go below ~120 words
- **Step 3 is what makes it land.** The reader must grasp that a joint search has no privileged D.

### P8 · Fig 4, the frontier — ~130 words
- → **cite "Figure 4" by number**
- → what's plotted: peak achievable success rate vs authority×delay
- → ρ = **−0.692**, p = **3.4 × 10⁻¹⁰**, n = **63**
- → conditions: **per-design optimal tuning, fresh evaluation seeds**
- → one clause on why it survived when the window sections didn't: it was never scored on the retired
  `window_ratio` metric
- ✗ **axis rule:** "authority×delay", a plotting coordinate. Never Π, never dimensionless, never a
  parameter you discovered

### P9 · What the frontier means — ~80 words *(cut first if long)*
- → degradation is **graded, not a cliff** — there's no threshold to hunt for
- → practical reading: a marginal design isn't disqualified, it's *expensive*
- → no new numbers

### P10 · The honest limit — ~120 words
- → **29 of 36** positives flagged `uncertain` by the project's own Wilson-interval check
- → the four-number screen scores AUC **0.985** against the original label but **0.57** against an
  independent harsher probe
- → only **5 of 16** original positives fail when the gain is pushed 2×
- → land on the distinction: **the map is real; the binary line drawn on it is not sharp**
- ✗ **claims-out:** never call it a controllability screen — it predicts the original labelling procedure

### P11 · Close — ~80 words
- → restate the deliverable: screen from four measurable numbers before cutting metal
- → hand to §5: §4 asked *who* fails, §5 asks *why*
- ✗ no new numbers, no summary of the whole paper

---

# PART C — THE SIDEBAR, IN FULL

**"What surprised a builder" — 0.5 pp, boxed, inside §4.** Communication device, **no new claims** —
every row is established elsewhere in the corpus. Four things a builder expects to matter that don't,
then the one that does.

| a builder expects | what was measured | implication |
|---|---|---|
| **A faster servo buys control.** Rate limit is what you upgrade. | Servo speed does not mark a failing design: r = **+0.037** (p = 0.071, n = 2,400) across 60–200 °/s. Removing the rate limit entirely moves success **+0.005** (Wilcoxon p = 0.128, n = 142). | Spend the money elsewhere. **Keep this nuance:** failing designs *do* saturate — **0.597** of the burn vs **0.089** — so saturation is a **symptom worth watching** (it's what §7 detects) but not the cause. |
| **Wind is the enemy.** | Wind strength does not mark a failing design: r = **−0.000** (p = 1.0) over a 9× range (0.05–0.45), and doesn't predict still-air-tuning failure either (ρ = +0.017, p = 0.4, n = 2,400). | Wind sets *how hard* the job is, not *which builds* fail. The four-number screen is unaffected by the day's weather. |
| **An unstable airframe is harder to fly.** | Stability margin does not mark a failing design: r(static margin) = **+0.011** (p = 0.6), r(Cm_alpha) = **−0.004** (p = 0.86). | Under active TVC at this scale, fin/CG stability is not the discriminator. **Scope narrowly** — see below. |
| **A more powerful rocket needs a gentler gain.** | The gain ceiling is set by loop delay and is independent of authority — keff exponent **−0.005**, **−0.071**, **−0.082** across three independent measurements, every CI spanning zero. | The ceiling formula needs your loop rate, not your thrust. A powerful rocket isn't automatically twitchy. |
| **Build it light.** | Light means low pitch inertia. The **median failing design sits at the 5.4th percentile** of the population inertia distribution; a failure has lower Iyy than a survivor **93.7%** of the time. | The one intuition that *should* change behaviour: mass at the ends is control authority you already paid for. |

**Two rows vetted and REJECTED — ship neither:**
- *"Aerodynamics contributes almost nothing at this scale."* Supported only in the **narrow** form
  above. The broad claim is contradicted by the minimal-physics study, which found aerodynamic
  coupling **necessary** to generate the disturbance environment at all — direct wind alone was
  10–100× too weak.
- *"Adding wind makes the vehicle settle better"* (dead-zone dither). Real, and locked in
  `sim/validate.py`, but the magnitude moves ~5× with sensor-noise configuration and the linear
  actuator improves too, so the "it's the dead zone" story doesn't hold.

---

# PART D — COMPLETENESS LEDGER

| # | value | ¶ | ✓ |
|---|---|---|---|
| 1 | 2,400 designs, Latin hypercube | P2 | ☐ |
| 2 | max_gimbal 2.0–15.0°, median 8.6° | P2 | ☐ |
| 3 | **eleven** fidelity modules, **single-axis pitch** | P2 | ☐ |
| 4 | Figure 3 cited by number | P3 | ☐ |
| 5 | the four success gates (70 / 15 / 15 / 50) | P3 | ☐ |
| 6 | CLES **93.7%** | P4 | ☐ |
| 7 | Mann-Whitney **p = 9.45e-20**, rank-biserial **0.875** | P4 | ☐ |
| 8 | median failure at **5.4th pctile** (IQR 1.1–8.9) | P4 | ☐ |
| 9 | **17 of 36** below 5th, **28 of 36** below 10th | P4 | ☐ |
| 10 | median Iyy **0.19×** population | P4 | ☐ |
| 11 | log(1/Iyy) +2.37 [1.94, 3.14] | P5 | ☐ |
| 12 | log(T·L) +1.77 [1.30, 2.45] | P5 | ☐ |
| 13 | log(τ) +2.90 [2.24, 3.92] | P5 | ☐ |
| 14 | raw-log ratio **1.18** vs keff's 1.00 | P5 | ☐ |
| 15 | CV AUC **0.9829 → 0.9687** without T·L | P5 | ☐ |
| 16 | L2 / intercept column as the retraction's cause | P6 | ☐ |
| 17 | `autotune_grid` = **25 joint pairs, 2 seeds** | P7 | ☐ |
| 18 | `autotune_continuous` = probe at Kp 40, freeze, sweep to 320 | P7 | ☐ |
| 19 | **1.9%** label movement (corroboration only) | P7 | ☐ |
| 20 | Figure 4 cited by number | P8 | ☐ |
| 21 | ρ = **−0.692**, p = 3.4e-10, n = 63 | P8 | ☐ |
| 22 | **29 of 36** flagged uncertain | P10 | ☐ |
| 23 | AUC **0.985** vs **0.57** | P10 | ☐ |
| 24 | **5 of 16** fail at 2× | P10 | ☐ |
| 25 | sidebar: 0.597 vs 0.089 saturated fraction | sidebar | ☐ |

### Forbidden moves — every box must stay UNCHECKED
☐ the word Π · ☐ "dimensionless" / "invariant" / "a parameter I discovered" ·
☐ thrust called irrelevant or protective · ☐ "controllability screen" ·
☐ "all failures below the 25th percentile" · ☐ "every failure lies left of here" ·
☐ "6 DOF" · ☐ "the graph shows" attached to a computed statistic ·
☐ any reliance on §6 · ☐ the dead-zone dither row · ☐ "aerodynamics contributes nothing" broadly

### Style rules that hold for all 20 pages
- **Person:** pick "I" or passive and hold it for the whole paper. Don't mix.
- **Counts, not bare percentages**, where a count is available (17 of 36, 28 of 36, 5 of 16).
- **Every figure cited by number in running text, including your own.** Uncited images can disqualify.
- No filler adverbs ("Interestingly"), no "Here are…" lead-ins.
- Say what a number *is* before you say what it *means*.

---

# PART E — DONE CHECK

- [ ] P5 is prose, not a bulleted list
- [ ] P7 exists, sits in the body, and leads with the structural argument
- [ ] P10 is present — a §4 without the soft-label limit is overclaimed
- [ ] Figures 3 and 4 each cited by number
- [ ] Ledger items 1–25 all placed
- [ ] Every forbidden-move box still unchecked
- [ ] None of the four Part-0 facts stated wrongly
- [ ] 1,100–1,300 words excluding the sidebar
- [ ] §4 reads standalone — nothing in it depends on §6
