# STS Paper Outline — claim selection, figures, page budget

**Working title (Braxton's, his to change):** *Why Thrust-Vectored Model Rockets Fail: A Measured
Failure Map for Small TVC Vehicles*

**Hard cap: 20 pages.** Title page (p.1) and abstract (p.2) and bibliography are FREE — they do not
count. **Appendices DO count.** So: ~20 pages of body, figures inline.

**Status of this document:** structure and claim selection only. Braxton writes every sentence.
Per-section notes say *what goes there and which numbers*, never how to phrase it.


**Axis-naming rule (Braxton's claims-out list).** The underlying analyses bin on keff·τ². In the
paper that axis is called **authority×delay** and is presented as a *plotting coordinate only* —
never named Π, never described as dimensionless, invariant, or a discovered parameter. The screen is
stated in terms of the four measurable numbers (thrust, nozzle arm, Iyy, loop delay), not a composite.


**Mechanism-naming rule (Braxton, 2026-08-09 — domain correction).** The artifact is **D held fixed
while P sweeps** — call it **stale-D** or **decoupled** tuning. That is what `autotune_continuous`
does: probe D once at P = 40, freeze it, sweep P to 320.

**Do NOT call it "sequential tuning."** Readers hear that as normal manual practice, and normal
manual practice is *alternating* — a builder adjusts P, then re-adjusts D, iteratively, which roughly
maintains the ratio and therefore sits **outside** the artifact regime. **This project has no data on
manual tuning.** Any sentence implying hobbyists tune the way the autotuner does is unsupported and
must not appear in §6, §10, or the abstract.

**Prevalence rule (Braxton, 2026-08-09).** **The paper never claims the decoupled protocol is
common.** Prevalence in other people's tools is **unknown and must be stated as unknown** — there is no
survey, and the gap that would have provided one (G1) was withdrawn. C-KD is a **conditional** claim:
*if* a tuner or a simulation campaign decouples the gains, here is what it manufactures, here is the
diagnostic, here is the fix. A conditional warning does not need prevalence data to be worth
publishing; it needs the condition to be easy to fall into, and this project is the existence proof.

**And the result is the masquerade, not the crashing.** "Don't freeze D while sweeping P or things
break" is trivial and would not deserve a section. What is not trivial: stale-D tuning does **not**
degrade uniformly. It fails *selectively, in proportion to hardware quantities* — 0.087 → 0.700 across
authority×delay in Arm A versus 0.022 → 0.013 in Arm B, so the bad tuner is ~4× worse at low
authority×delay and ~56× worse at high. That arrives as a smooth monotone dose-response against
measurable hardware parameters (ρ = +0.236, p = 1e-31), i.e. **exactly what a physical limit looks
like**. An inadequate tuning protocol does not add noise to an experiment; it adds **counterfeit
laws** — thresholds, scaling exponents, regime boundaries — that careful analysis will mistake for
vehicle physics.

**Say which structure dissolved and which held.** Do not write "it was all an artifact." Under the
same free-Kd test the window/floor family collapsed (keff exponent −0.803 → +0.196, floor reversing
sign +0.662 → −0.848) while **C-CEILING survived** (−0.071 at fixed Kd, matching −0.082 measured
independently, positive control ρ −0.762 vs published −0.74). A diagnostic that kills everything is
not a diagnostic. One that discriminates is the whole point, and it is why the ceiling is still in
this paper as a lead finding.

**Emphasis ruling (Braxton, 2026-08-09 — final).** This paper is the **positive characterization**.
The abstract and §1 lead with **finding 1, the failure map (who fails — inertia, the frontier)** and
**finding 2, the 1/τ gain ceiling**. The decoupled-tuning result is **finding 3**, and it is presented
as a **discovery about tuning methodology** — a prescription plus a warning aimed at autotuners and
simulation campaigns — **not as a retraction story**. Concretely:

- The abstract's first sentence of results is inertia/the map. Not the artifact.
- §6 is *a finding*, not "the headline." Nothing in the paper is labelled the headline any more.
- The six retired sections are **one paragraph**, in §8, and §8 is **~2 pp**. The audit is a
  methodology credential, not the story.
- No sentence anywhere frames the paper as being about correcting earlier work.

**All sections are decided.** The one conditional — §5's window sections — was resolved 2026-08-09 by
`tools/window_kd_free.py`: the control passed and the window narrowing did not survive a free Kd, so
those sections are cut. See §5.

---

## Page budget — lands at exactly 20, both branches

**Sep 7 means "20 pages, formatted."** One page over cap is not a finished deliverable; the page
limit is a disqualification-class rule, not a style preference. The trim below is already made — it
is not deferred to October.

**Base case — the §5 window sections are CUT** (see §5):

| § | Section | pp | Carries |
|---|---|---|---|
| 1 | Introduction — the builder's problem | 1.5 | motivation, the four numbers, the question |
| 2 | Background — what is already known | 0.75 | delay-margin theory; what is NOT new here |
| 3 | Methods — simulator, design space, protocols | 2.5 | credibility; the seed/threshold discipline |
| 4 | The failure map — who fails (incl. 0.5 pp surprise sidebar) | 3.75 | **C-INERTIA**, **C-FRONTIER** *(lead finding 1)* |
| 5 | The gain ceiling — why they fail | **3.0** | **C-CEILING** *(lead finding 2)* |
| 6 | Sim-to-real — what tuning method does to the result | 3.5 | **C-S2R**, **C-KD** *(finding 3)* |
| 7 | Flight signature — the hardware test | 2.0 | **C-FLIGHT** + measured τ dataset |
| 8 | Methods audit — how the measurements were checked | **2.0** | **C-AUDIT** (credential, not story) |
| 9 | Limitations and what is not claimed | 0.5 | the negative space |
| 10 | Impact for builders | 0.5 | impact paragraph, NOT the thesis |
| | **total** | **20.0** | |

**Conditional resolved 2026-08-09:** the rerun cut the window sections, so this is the live budget.
§3 gave up 0.5 pp to fund §4's surprise sidebar — Methods is the right donor because its figure
captions carry the most load.

**What was cut to get from 21 to 20, and why:**
- §2 Background 1.0 → 0.75. STS guidelines forbid a long literature history anyway.
- §4 3.5 → 3.25, by dropping **Fig 5** (the coefficient plot). It was already flagged first-cut; the
  same coefficients read fine as a table inline.
- §9 Limitations 0.75 → 0.5. A tight list is more credible than a discursive one.
- §10 Impact 0.5. Braxton's own instruction: hobbyist usefulness is the impact paragraph, not the
  thesis — a paragraph is half a page.

**Protected at full budget:** §4 and §5 (the two lead findings), §6 (finding 3), §3 (rigor is what
Top 300 judges). Do not raid these. §8 is the donor if anything overruns — it was cut 2.5 → 2.0 by the
emphasis ruling and can go to 1.75 before it stops carrying the pre-registered test.

**Reconciled 2026-08-09:** the per-section headings below had drifted from this table (§9 and §10 still
read 0.75, which summed to 20.5, and §5's note claimed §3 keeps 3.0 after §3 was already cut to 2.5).
The table is authoritative; headings now match it.

Figure count drops to **11** with Fig 5 cut.


---

## Abstract skeleton — slot order is the emphasis ruling made concrete

**Structure only. Braxton writes every sentence.** ~200 words, one slot per sentence-ish. The order
below is not stylistic — it is what the ruling requires, and it is the easiest thing in the paper to
drift back out of order while drafting.

| # | slot | what goes in it | numbers available |
|---|---|---|---|
| 1 | problem | A builder can measure thrust, nozzle arm, pitch inertia and loop delay before cutting metal, and cannot currently predict from them whether the vehicle will be tunable. | the four numbers |
| 2 | what was done | 2,400-design simulated space, 10 fidelity modules, per-design gain search. | n = 2400 |
| 3 | **finding 1 — the map** | Failing builds are **low-inertia** builds; achievable performance falls with authority×delay under optimal tuning. | **100%** of 36 failures below the 25th pctile of Iyy; ρ = **−0.692**, p = 3.4e-10 |
| 4 | **finding 2 — the ceiling** | The usable-gain ceiling is set by loop delay and is **independent of control authority**, at a fixed Kd. | τ exponent **−1.067**; keff **−0.082** CI [−0.21, +0.05]; recalibrated **0.0661/τ** |
| 5 | **finding 3 — tuning method** | *Then*, and only here: a decoupled gain search does not merely add noise — it produces a smooth, hardware-indexed failure trend that reads as a physical law and is not one. Tying D to P removes it. | **56/80 → 1/80**; ρ **+0.236** vs **−0.117** |
| 6 | the prescription, **conditional** | Maintain the P:D ratio; re-tune D after any change to P before judging stability. Named audience: automated tuners and simulation campaigns. **No prevalence claim** — "if your pipeline does this," never "pipelines do this." | — |
| 7 | the hardware test | One instrumented flight distinguishes a marginal build. | AUC **0.954** [0.907, 0.989] |
| 8 | scope | Simulation, with the measured-τ / flight qualification from §7. | — |

**Slot 5 must not migrate above slot 3.** If a draft's first result sentence is about tuning method,
the ruling has been violated. **The word "retraction," and any framing of the paper as a correction of
earlier work, does not appear in the abstract at all.**

---

## Claims that make the cut

Ordered by the emphasis ruling: leads first, tuning method third.

| tag | claim | strongest number | § | fig |
|---|---|---|---|---|
| **C-INERTIA** | *(lead 1)* Failing builds are low-inertia builds | **100%** of the 36 failing designs below the 25th pctile of Iyy | 4 | 3 |
| **C-FRONTIER** | *(lead 1)* Achievable performance degrades with authority×delay under optimal tuning | ρ = **−0.692** vs authority×delay, p=3.4e-10, n=63 | 4 | 4 |
| **C-CEILING** | *(lead 2)* The usable-gain ceiling is set by loop delay, not authority — **at a fixed Kd** | keff **−0.082** CI [−0.21,+0.05]; τ **−1.067**; control ρ −0.762 vs −0.74 | 5 | 6 |
| **C-S2R** | *(finding 3)* A gain tuned in still air fails under full physics, with a monotone dose-response | 6.0% → **78.3%** across authority×delay bins, n=2400 | 6 | 7 |
| **C-KD** | *(finding 3)* A decoupled gain search (D fixed while P sweeps) generates **spurious hardware-indexed structure** — it counterfeits a law rather than adding noise; re-tuning D with P removes it. Conditional; prevalence unknown | **56/80 → 1/80** at high risk; ρ +0.236 vs −0.117; ~4× worse at low authority×delay vs ~56× at high | 6 | 8, 9 |
| **C-FLIGHT** | A failing build is identifiable from one instrumented flight | RMS **13.3°±5.2°** vs **3.8°±2.7°**; AUC **0.954** [0.907,0.989] | 7 | 10, 11 |
| **C-AUDIT** | The measurement protocol, not the hardware, drove several published effects | pre-registered test **8/8**; floor exponent +1.06→+0.21→−0.20 | 8 | 12 |

## Claims cut — must not reappear anywhere in the text

Π = keff·τ² and every dimensionless-invariant framing · the causal saturation-removal result ·
the `window_ratio` family *as originally measured* · the binary classifier as a headline ·
"thrust is irrelevant or protective" · "strong p_unstable interaction" ·
"each correction strengthened the AUC" · **"sequential tuning is what hobbyists do" / any claim that
common manual practice sits inside the artifact regime** · **any claim that decoupled tuning is
*common* in other people's tools — prevalence is unmeasured and must be stated as unknown
(2026-08-09)** · **"it was all an artifact"** — the ceiling survived the same test — manual practice is *alternating* and this
project measured no manual tuning at all (Braxton, domain correction, 2026-08-09)

---

## Section notes

### §1 Introduction — 1.5 pp
The builder tunes in a still-air simulator, flies, and the rocket oscillates or leans. Frame the
question as **prediction before construction**: given thrust, nozzle arm, pitch inertia and loop
delay, can you tell whether this build will be tunable? State the four numbers explicitly here —
they are the paper's spine.

**Lead order is fixed by the emphasis ruling.** The contribution paragraph previews, in this order:
(1) the failure map — which builds fail, and that it is inertia; (2) the gain ceiling, and that it is
set by delay rather than authority; (3) that how the gain is searched for changes which designs are
found to fail, which is a result about tuning methodology with a one-line prescription. **The intro
does not open on, or build toward, the tuning result**, and it does not describe the paper as revising
earlier work. Close on what the paper measures, not what it discovers.
*No figure.*

### §2 Background — 0.75 pp
What is already known: the delay-margin gain ceiling is classical. Mention Ziegler–Nichols as the
classical *procedure* for finding gains — but do NOT use it to argue that builders leave D stale;
that inference is not supported. Say plainly that **C-CEILING is a re-derivation
in a new domain, not a discovery** — claiming otherwise is the fastest way to lose a controls
reviewer. Note the absence of hobby-scale TVC in the literature. STS guidelines forbid a long
literature history: keep it short.
*No figure.*

### §3 Methods — 2.5 pp
Simulator and the 10 fidelity modules; the 2,400-design LHS space with parameter ranges; the
gain-search protocols (this matters — the whole paper turns on tuning method, so both the stale-D
and ratio-preserving tuners need describing precisely, in terms of what each does to D when P moves); disjoint seed ranges per experiment.

**Name the two tuners separately and say which section uses which** — this is what lets §4 defend
itself against §6. `autotune_grid`: joint 5 × 5 Kp × Kd grid, 2 search seeds, used for the §4/§5
population labels. `autotune_continuous`: Kd probed once at Kp = 40 then frozen while Kp sweeps to 320,
used for §6's sim-to-real arms and the source of the artifact. A reader must be able to tell, from §3
alone, that the failure map was never tuned the way §6 shows to be dangerous.

**Two discipline items that belong here and are worth the space:**
- **One success criterion, stated once.** SR ≥ 0.80 throughout. Where the earlier work reported
  against the 0.35 INFEASIBLE gate, say so explicitly — this is the SR<0.50 vs SR<0.80 inconsistency
  Braxton flagged, and the fix is to report the dose-response curve at a single stated threshold.
- **Seed resolution.** n seeds cannot resolve a rate finer than 1/n. The original 3-seed evaluation
  could not distinguish 0.67 from 0.80.

*Fig 1 — design space coverage (Iyy × latency, colored by keff).
Fig 2 — the two tuning protocols side by side, as a diagram. This figure does a lot of work in §6;
put it here so §6 can just refer back.*

### §4 The failure map — who fails — 3.75 pp (3.25 body + 0.5 sidebar)
The paper's title section. Which builds fail, as a map over the measurable numbers.

C-INERTIA: 100% of the 36 failing designs sit below the population 25th percentile of Iyy; median
Iyy 0.19× the population median. Report the **unregularized** joint fit with bootstrap CIs —
log(1/Iyy) +2.37 [1.94, 3.14], log(T·L) +1.77 [1.30, 2.45], log(τ) +2.90 [2.24, 3.92] — all three
positive, all CIs excluding zero. **keff = T·L/Iyy is a legitimate grouping** (raw-log ratio
T·L/(1/Iyy) = 1.18 where keff requires 1.00). Do not claim thrust is protective.

C-FRONTIER: §4.0.3's performance frontier, ρ = −0.692 against authority×delay (p=3.4e-10, n=63),
measured under per-design optimal tuning with fresh evaluation seeds — so it is *not* contaminated by
the frozen-Kd metric. Say that explicitly — it is why this survived when the window sections did not.

**⚠ THE SHARPEST REVIEWER QUESTION, AND IT HAS A GOOD ANSWER — put it in the text, do not wait to be
asked.** *"§6 shows your tuning protocol manufactured spurious hardware-indexed structure. Why should I
believe §4's inertia result isn't the same artifact?"* Because **the two sections use different
tuners, and §4's is immune by construction**:

- §6's artifact comes from `autotune_continuous` — probe Kd **once** at Kp = 40, freeze it, sweep Kp
  to 320. Decoupled, and that is exactly the mechanism.
- §4's labels come from `autotune_grid` — a **nested double loop over Kp × Kd**, 5 × 5 = 25 combinations
  scored jointly, 2 search seeds each (`experiment_runner.py:205`, `KP_GRID`/`KD_GRID` at :69). **D is
  never held stale while P moves**, so the mismatch that generates the artifact cannot arise.

Corroborating, not load-bearing: re-tuning the whole population on the restored simulator moved the
labels **1.9%**. State the structural argument first — it is the one that actually settles it.

**Honest limit that must appear here:** the failing-design *label* is soft. 29 of its 36 positives
are flagged uncertain by the project's own Wilson-interval check, and a four-number screen trained on
it scores AUC 0.985 against that label but **0.57** against an independent harsher probe. The map is
real; the binary line drawn on it is not sharp.

**SIDEBAR — "What surprised a builder" (0.5 pp, boxed, inside §4).** Expectation / measurement /
implication, one row each. A **communication device, not new claims** — every row below is already
established in the corpus and carries its citation. Four things a builder expects to matter that do
not, then the one that does.

| a builder expects | what was measured | implication |
|---|---|---|
| **A faster servo buys control.** Rate limit is the thing you upgrade. | Servo speed does not mark a failing design: r = **+0.037** (p = 0.071, n = 2400) across 60–200 °/s. Removing the rate limit *entirely* moves success **+0.005** (Wilcoxon p = 0.128, n = 142). | Spend the money elsewhere. **Nuance that must stay:** failing designs *do* saturate — 0.597 of the burn vs 0.089 — so saturation is a **symptom worth watching** (it is what §7's flight signature detects) but not the cause. |
| **Wind is the enemy.** Gusts are what you tune against. | Wind strength does not mark a failing design: r = **−0.000** (p = 1.0) over a 9× range (0.05–0.45), and it does not predict still-air-tuning failure either (ρ = +0.017, p = 0.4, n = 2400). | Wind sets *how hard* the job is, not *which builds* fail. Screening on the four numbers is unaffected by the day's weather. |
| **An aerodynamically unstable airframe is harder to fly.** | Stability margin does not mark a failing design: r(static margin) = **+0.011** (p = 0.6), r(Cm_alpha) = **−0.004** (p = 0.86). | Under active TVC at this scale, fin/CG stability is not the discriminator. **Scope this narrowly** — see the rejected row below. |
| **A more powerful rocket needs a gentler gain.** | The gain ceiling is set by loop delay and is independent of authority — keff exponent **−0.005**, **−0.071**, **−0.082** across three independent measurements, every CI spanning zero. | The ceiling formula needs your loop rate, not your thrust. A powerful rocket is not automatically a twitchy one. |
| **Build it light.** | Light means low pitch inertia, and **all 36** failing designs sit below the population 25th percentile of Iyy (median 0.19× the population). | The one intuition that *should* change behaviour: mass at the ends is control authority you already paid for. |

**Two candidates vetted and REJECTED — do not ship:**
- *"Aerodynamics contributes almost nothing at this scale."* Supported only in the **narrow** form
  above (stability margin does not predict failure). The broad claim is contradicted by the
  minimal-physics study, which found aerodynamic coupling **necessary** to generate the disturbance
  environment at all — direct wind alone was 10–100× too weak. Ship the narrow row, not the slogan.
- *"Adding wind makes the vehicle settle better"* (dead-zone dither). Real and locked in
  `sim/validate.py`, but the magnitude moves ~5× with sensor-noise configuration and the linear
  actuator improves too, so the "it is the dead zone" story does not hold up. Not robust enough for
  a builder-facing row.

*Fig 3 — the failure map itself: Iyy × latency, 2,400 designs, failures marked. Title figure.
Fig 4 — performance frontier, peak achievable SR vs authority×delay.
Fig 5 — coefficient plot, the three factors with bootstrap CIs. (Cut this one first if §4 overruns.)*

### §5 The gain ceiling — why they fail — 3.0 pp — **lead finding 2**
C-CEILING with all three honesty items stated in the text, not a footnote:
1. **"At a fixed Kd."** The ceiling is 8.8× higher when Kd is free (paired, 94% of designs,
   p=4.9e-9). A builder flies one Kd, so the fixed-Kd case is the operationally relevant one — say
   that, do not hide it.
2. **The constant.** 0.042/τ underpredicts real ceilings by ~1.8× (median ratio 1.81, MARE 54%,
   within-2× 58%). Recalibrated **0.0661/τ**; conservative 5th-percentile bound 0.0231/τ. Frame
   0.042/τ as a **conservative screen**, not a boundary — "a gain above this will oscillate" is not
   what the data says.
3. **Sample conditionality.** ρ = −0.76 on the authority-stratified sample vs **−0.38** on a
   population-representative one. Same code, same seeds, only the design set differs.

**RESOLVED 2026-08-09 — the window sections are CUT.** `tools/window_kd_free.py`, 120 designs,
positive control PASSED (replication within 1.86 SE and 1.45 SE of v2's published exponents, with
adequate precision). Result:

- The window's authority dependence **collapses**: keff exponent **−0.803 (SE 0.209)** with Kd frozen
  at v2's value, **+0.196 (SE 0.309)** with Kd free — a confidence interval spanning zero.
- The mechanism is the **floor**, and it *reverses sign*: **+0.662 (SE 0.144)** frozen versus
  **−0.848 (SE 0.116)** free, t = −7.3. The floor rising with authority is what closed the window,
  and it does not survive a free Kd.
- Freeing Kd widens the window by a median **13.6×** (n=115, 97% of designs wider, Wilcoxon
  p = 5.9×10⁻²⁰) — a paired result that does not depend on the regression.
- With Kd free the window is mostly **unmeasurable**: 80% ceiling-censored, median window 8000×,
  i.e. the entire tested grid. For most designs, any gain works once D tracks P.

§5 is **3.0 pages** — raised from 2.5 by the emphasis ruling and funded by §8's cut, since the ceiling
is now a lead finding and its three honesty items each need room in the running text.
The window result becomes **one paragraph in §8** as the fifth independent confirmation of the
Kd artifact.

**One thing this test also did: it corroborated C-CEILING a third time.** The ceiling's authority
exponent came out **−0.071 (SE 0.059)** at fixed Kd — indistinguishable from zero, matching the
−0.082 measured independently on v2's designs. The ceiling is authority-independent; it was the
floor that was carrying the artifact all along.

*Fig 6 — measured ceiling vs 1/τ, log-log, colored by keff (shows the authority-independence
visually), with both 0.042/τ and the recalibrated fit drawn.*

### §6 Sim-to-real: what the tuning method does to the result — 3.5 pp — **finding 3**

**Framing (emphasis ruling + prevalence rule).** This is a **discovery about tuning methodology**,
stated positively and in one sentence: **a decoupled gain search generates spurious structure that
masquerades as vehicle physics.** Not "bad tuning breaks rockets" — that is obvious and would not earn
a section. The claim is **conditional** (if your pipeline decouples, this is what you will measure) and
**prevalence elsewhere is stated as unknown.** It is not the
paper's headline and it is not a retraction. Write it as a finding with a prescription attached — a
reader who has never seen this project's earlier work should read §6 straight through and take away a
usable rule, with no sense that anything is being walked back. The audit belongs in §8.

Build it in this order:

1. **The phenomenon (C-S2R).** Still-air-tuned gains fail under full physics, monotone in
   authority×delay: 6.0% → 78.3% across bins, n=2400. Report the **curve**, one threshold.
2. **The mechanism that produces it (C-KD).** Two arms on the same 2,400 designs, differing only in what the tuner does
   to D when P moves. Arm A (**stale D** — D probed once at P = 40 and frozen while P sweeps to 320,
   which is what the autotuner does) reproduces the published curve **bin by bin** — quote
   0.120/0.120 and 0.238/0.238 as the exact hits. Arm B (Kd tied to Kp) gives 0.022 → 0.013.
   ρ vs authority×delay: **+0.236** (p=1e-31) vs **−0.117** (wrong sign). Failure at high risk **56 of 80 → 1 of 80** designs.
   **Report counts, not a 1-decimal rate:** 1/80 = 1.25%, so "1.3%" and "1.2%" are both rounding
   artifacts of a single event. 95% Wilson CIs are [59.2%, 78.9%] and [0.22%, 6.75%] — non-overlapping
   by a wide margin, which is the honest way to show the effect survives its own uncertainty.
3. **The mechanism, and why it counterfeits a law.** `autotune_continuous` probes Kd once at Kp=40,
   freezes it, sweeps Kp to 320; designs whose best gain lands far from the probe get a mismatched Kd,
   and **the mismatch grows with authority×delay**. That last clause is the whole finding: the damage
   is not uniform, it is *indexed to hardware*, which is why it emerges as a smooth dose-response
   rather than as scatter. Give the two ratios — stale-D is ~4× worse than ratio-preserving at low
   authority×delay and ~56× worse at high. **The masquerade is the result, not the failures.**
4. **External validity — state it as a PRESCRIPTION, not an ethnography.** The paper has no data on
   how humans tune, and manual practice is *alternating* (adjust P, re-adjust D, repeat), which
   roughly preserves the ratio and therefore sits outside the regime measured here. So the claim is
   **not** "this is what builders do." It is:

   > **Maintain the P:D ratio. Re-tune D after any change to P, before judging stability.**

   Say who it applies to **conditionally**: any **automated tuner or simulation campaign** that fixes
   D once and then sweeps P — which is exactly the protocol that produced six retired sections of this
   project's own prior work, so the paper has direct evidence that the condition is easy to fall into.
   **State plainly that prevalence in other tools is unknown**: no survey was performed, and the claim
   is therefore "if your pipeline does this" and not "pipelines do this." A builder tuning by hand is
   already doing the right thing; the value to them is knowing *why* the ratio matters.

*Fig 7 — failure rate vs authority×delay, the dose-response curve.
Fig 8 — the two arms overlaid: same designs, same axis, one line each. The paper's key figure.
Fig 9 — the mechanism, as a Kp×Kd plane showing where the frozen-Kd choice lands vs the coupled path.*

### §7 Flight signature and hardware — 2.0 pp
C-FLIGHT: RMS 13.3° ± 5.2° vs 3.8° ± 2.7°, **slew-rate** saturated fraction 0.60 vs 0.14, AUC 0.954
[0.907, 0.989]; single flight ≈ 0.907. Note the balanced 36/36 construction inflates AUC relative to
deployment prevalence.

⚠ **Say "slew-rate saturated fraction", never bare "saturated fraction."** It is the fraction of steps
where the actuator *rate* limit binds. A builder reads "saturated" as *the gimbal hit its stops*, which
is a different quantity — and one that is **identically zero across all 504 simulated runs** while the
real flights hit it constantly (LOG001 100% of boost rows, ASC007 23.9%). The two must never share a
table. Relabel in the text and in Fig 10's caption.

**RETROSPECTIVE HARDWARE TEST — ran 2026-08-09, pre-registered, `paper/RETRO_FLIGHT_SIG_SPEC.md`.**
The sim-derived rule (FAILING if boost-window attitude RMS ≥ 5.62°, threshold carried from simulation
with **nothing refit**) was applied to all five archived flights. **5 of 5 correct**, TP 3 / TN 2,
one-sided exact **p = 0.100 — which was declared as the archive's ceiling before the run.** State the
ceiling in the same sentence as the result. Three qualifications belong here, none of them foreseeable
from simulation, and all three are more interesting than the 5/5:

1. **The signature is only as good as the attitude estimate feeding it.** Same rule on the *as-logged*
   attitude scores **2 of 5 (p = 0.700)** — chance. Both errors are documented estimator faults:
   ASC038's estimator froze (pre-declared in the spec before the run) so a vehicle that reached ≥161°
   scores 2.42° and reads HEALTHY; ASC031's naive estimator inverted past ~120° roll and produced a
   false positive. **Hardware produced this; simulation cannot.**
2. **The healthy calibration transfers; the failure calibration does not.** Flown healthy 3.61° and
   5.08° sit inside the simulated EASY distribution (3.8 ± 2.7). But flown failures reach **58.4° and
   90.0°** against a simulated FRAGILE population that tops out near 25°. The simulator reproduces a
   working vehicle well and **substantially understates how bad a failing one gets** — every simulated
   design is tuned and merely fragile; a real vehicle losing control is unbounded.
3. **It fires on hand motion.** ASC037, a ground test with no motor, scores 7.6°. The rule means
   something only inside a genuine powered boost — say so, or "one flight diagnoses a build" overclaims.

**Report ASC031 honestly:** at 5.08° it clears the grey-zone floor (5.37°) by 0.29°, ~5%, and it is the
one flight whose verdict flips between the two analysis passes. One of the five rests on a
pre-registered analysis choice, not on a comfortable margin.

**The hardware dataset goes here** — measured τ from the bench (spec posted to the engineer), the
bifilar Iyy, and whatever flight data exists by the deadline. Even a bench-only τ makes §5 and §6
concrete rather than parametric: it places *this* vehicle on the paper's own axes.

**Qualification that must appear, and it is the engineer's finding, credited:** a simulator PASS is
only evidence about failure modes the model can represent. ASC038 tumbled from a 143 ms SD write
inside the control loop — a failure the SIL could not express, because its clock is synthetic and
file writes cost zero simulated time. This paper's simulation results carry the same qualification.
Flight record: **2 of 6 attempts controlled.**

*Fig 10 — RMS distributions by class, with the diagnostic threshold (caption must say slew-rate).
Fig 11 — measured bench τ: the dead-time distribution across randomized PWM phase.
Fig 13 — **the retrospective test**: five flown flights on the simulation's own RMS scale, log axis,
pre-registered threshold and grey zone drawn. `paper/figures/fig13_retro_flight_test.png`.*

### §8 Methods audit — 2.0 pp — **a credential, not the story**
Not an apology, not a limitations section, and — per the emphasis ruling — **not the paper's arc**.
Cut 2.5 → 2.0; the half page went to §5. The claim is narrow and positive: several measured effects
turned out to be properties of the measurement protocol, and here is the discipline that caught them.

**Hard limit: the retired work is ONE paragraph.** All six retired sections — the `window_ratio`
family, the causal saturation result, the controller-invariance comparisons scored on the retired
metric — get a single paragraph naming what was withdrawn and why. Do not narrate them one at a time.
A reader should finish §8 thinking *this student checks his own work*, not *this student made six
mistakes.*

Lead with the **pre-registered test**: `frozen_kd_artifact_test.py`, 2026-06-22, decision criteria
written into the file *before* the run, asymmetric so the easy outcome was "carry on" — and 8 of 8
designs cleared the *most severe* threshold instead. That is the strongest single item in the paper
for the Student Contribution criterion.

Then the cascade: floor exponent +1.06 → +0.21 → −0.20 across protocols; the causal saturation claim
(128 of 142 designs already at SR=1.000 with saturation on; paired Δ +0.005; Wilcoxon p=0.128); a
validation check that asserted nothing for its entire existence (`theta_true` constant across 601
samples); a stored instability column inverted for 1,196 of 2,400 rows.

**The discriminating result belongs here, and it is what makes §8 a credential rather than a mea
culpa:** the same free-Kd test that dissolved the window/floor family left the gain ceiling standing
(−0.071 at fixed Kd vs −0.082 measured independently; positive control ρ −0.762 vs published −0.74).
A test that kills everything is not a test. Say so in one sentence.

Close on the general rule the audit produced: **every experiment needs a positive control**, and a
result that matches expectation is the one to distrust.

*Fig 12 — the floor exponent across the four protocols, with the pre-registered threshold drawn.*

### §9 Limitations and what is not claimed — 0.5 pp
No mechanism isolated. No dimensionless invariant; the τ exponent is not identified (1.71 / 1.35 /
controlled test favored 1). No cross-architecture claim. Simulation-only except where §7 says
otherwise, with the ASC038 qualification. The failing-design label is soft. **On §6's scope:** this
project measured no manual tuning and surveyed no other codebase, so nothing is claimed about what
builders do *or* about how common the decoupled protocol is elsewhere — §6 is a conditional result
plus a documented case study of one pipeline. State it as scope, in one sentence, not as a confession.

### §10 Impact for builders — 0.5 pp
**Impact paragraph, not the thesis.** Maintain the P:D ratio — re-tune D after any change to P
before judging stability. Aim this at autotuners and sim campaigns, **conditionally** ("if your
pipeline does this"), with no claim about how widespread that is; do NOT imply hand-tuners are doing
it wrong, since alternating manual practice already preserves the ratio. Screen a
design before cutting metal from four measurable numbers. Use 0.042/τ as a conservative gain limit.
One instrumented flight at a known gain diagnoses a marginal build.

---

## Figure list — 12

1. Design-space coverage · 2. The two tuning protocols (diagram) · 3. **The failure map** ·
4. Performance frontier · 5. Coefficient plot with CIs *(first cut)* · 6. Ceiling vs 1/τ ·
7. S2R dose-response · 8. **Two arms overlaid** · 9. Kd/Kp mechanism plane · 10. Flight-signature
distributions · 11. Measured bench τ · 12. Floor exponent across protocols

Figures 3 and 8 carry the paper. Every figure needs an in-text citation **including your own** —
uncited images can disqualify.

---

## Format checklist

Title p.1 · abstract p.2 · bibliography at end — none count toward 20. **Appendices count.**
≥11pt, 1.5 spacing, 1" margins, single column, page numbers bottom-right starting after the abstract.
No hyperlinks outside references. PDF ≤4 MB, named `LASTNAME.FIRSTNAME.ZIPCODE`. Every figure cited.
Appendix 11 risk form (rocket motors) — dad as Designated Supervisor.

## Gaps ranked, for September

1. **Measured τ** — spec posted to the engineer; makes §7 concrete, and it is the only open Sep-7
   gate item.
2. **Flight data** — tests C-FLIGHT on hardware; the one falsifiable prediction the paper makes.
3. ~~Kd-free window rerun~~ — **DONE 2026-08-09.** Control passed; §5's window sections are cut.
4. ~~Firmware survey~~ — **DOWNGRADED 2026-08-09.** It existed to argue stale-D is common practice;
   that claim is withdrawn (manual practice is alternating), so it no longer buys §6 anything. At
   most one sentence in §9. Not September work.
5. **Re-score LQR/SMC on a common metric** — moot; the window sections are cut.
