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
| 4 | The failure map — who fails (incl. 0.5 pp surprise sidebar) | 3.75 | **C-INERTIA**, **C-FRONTIER** |
| 5 | The gain ceiling — why they fail | 2.5 | **C-CEILING** |
| 6 | Sim-to-real — the tuning method is the cause | 3.5 | **C-S2R**, **C-KD** (the headline) |
| 7 | Flight signature — the hardware test | 2.0 | **C-FLIGHT** + measured τ dataset |
| 8 | Methods audit — how the measurements were checked | 2.5 | **C-AUDIT** (methodology highlight) |
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

**Protected at full budget:** §6 (headline), §8 (the pre-registered test), §4 (title section), §3
(rigor is what Top 300 judges). Do not raid these.

Figure count drops to **11** with Fig 5 cut.


---

## Claims that make the cut

| tag | claim | strongest number | § | fig |
|---|---|---|---|---|
| **C-KD** | Sequential tuning (P then D) causes the failures; coupling Kd to Kp removes them | **56/80 → 1/80** at high risk; ρ +0.236 vs −0.117 | 6 | 8, 9 |
| **C-S2R** | A gain tuned in still air fails under full physics, with a monotone dose-response | 6.0% → **78.3%** across authority×delay bins, n=2400 | 6 | 7 |
| **C-CEILING** | The usable-gain ceiling is set by loop delay, not authority — **at a fixed Kd** | keff **−0.082** CI [−0.21,+0.05]; τ **−1.067**; control ρ −0.762 vs −0.74 | 5 | 6 |
| **C-INERTIA** | Failing builds are low-inertia builds | **100%** of the 36 failing designs below the 25th pctile of Iyy | 4 | 3 |
| **C-FRONTIER** | Achievable performance degrades with authority×delay under optimal tuning | ρ = **−0.692** vs authority×delay, p=3.4e-10, n=63 | 4 | 4 |
| **C-FLIGHT** | A failing build is identifiable from one instrumented flight | RMS **13.3°±5.2°** vs **3.8°±2.7°**; AUC **0.954** [0.907,0.989] | 7 | 10, 11 |
| **C-AUDIT** | The measurement protocol, not the hardware, drove several published effects | pre-registered test **8/8**; floor exponent +1.06→+0.21→−0.20 | 8 | 12 |

## Claims cut — must not reappear anywhere in the text

Π = keff·τ² and every dimensionless-invariant framing · the causal saturation-removal result ·
the `window_ratio` family *as originally measured* · the binary classifier as a headline ·
"thrust is irrelevant or protective" · "strong p_unstable interaction" ·
"each correction strengthened the AUC"

---

## Section notes

### §1 Introduction — 1.5 pp
The builder tunes in a still-air simulator, flies, and the rocket oscillates or leans. Frame the
question as **prediction before construction**: given thrust, nozzle arm, pitch inertia and loop
delay, can you tell whether this build will be tunable? State the four numbers explicitly here —
they are the paper's spine. Close on what the paper measures, not what it discovers.
*No figure.*

### §2 Background — 0.75 pp
What is already known: the delay-margin gain ceiling is classical (Ziegler–Nichols; standard
sequential PID tuning is P-then-D by construction). Say plainly that **C-CEILING is a re-derivation
in a new domain, not a discovery** — claiming otherwise is the fastest way to lose a controls
reviewer. Note the absence of hobby-scale TVC in the literature. STS guidelines forbid a long
literature history: keep it short.
*No figure.*

### §3 Methods — 2.5 pp
Simulator and the 10 fidelity modules; the 2,400-design LHS space with parameter ranges; the
gain-search protocols (this matters — the whole paper turns on tuning method, so both the sequential
and coupled tuners need describing precisely); disjoint seed ranges per experiment.

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

### §5 The gain ceiling — why they fail — 2.5 pp
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

So §5 stays at **2.5 pages** and §3 keeps its 3.0 — the base budget stands, no reallocation needed.
The window result becomes **one paragraph in §8** as the fifth independent confirmation of the
Kd artifact.

**One thing this test also did: it corroborated C-CEILING a third time.** The ceiling's authority
exponent came out **−0.071 (SE 0.059)** at fixed Kd — indistinguishable from zero, matching the
−0.082 measured independently on v2's designs. The ceiling is authority-independent; it was the
floor that was carrying the artifact all along.

*Fig 6 — measured ceiling vs 1/τ, log-log, colored by keff (shows the authority-independence
visually), with both 0.042/τ and the recalibrated fit drawn.*

### §6 Sim-to-real: the tuning method is the cause — 3.5 pp — **THE HEADLINE**
Build it in this order:

1. **The phenomenon (C-S2R).** Still-air-tuned gains fail under full physics, monotone in
   authority×delay: 6.0% → 78.3% across bins, n=2400. Report the **curve**, one threshold.
2. **The cause (C-KD).** Two arms on the same 2,400 designs, differing only in the tuner. Arm A
   (sequential, the standard method) reproduces the published curve **bin by bin** — quote
   0.120/0.120 and 0.238/0.238 as the exact hits. Arm B (Kd tied to Kp) gives 0.022 → 0.013.
   ρ vs authority×delay: **+0.236** (p=1e-31) vs **−0.117** (wrong sign). Failure at high risk **56 of 80 → 1 of 80** designs.
   **Report counts, not a 1-decimal rate:** 1/80 = 1.25%, so "1.3%" and "1.2%" are both rounding
   artifacts of a single event. 95% Wilson CIs are [59.2%, 78.9%] and [0.22%, 6.75%] — non-overlapping
   by a wide margin, which is the honest way to show the effect survives its own uncertainty.
3. **The mechanism.** `autotune_continuous` probes Kd once at Kp=40, freezes it, sweeps Kp to 320;
   designs whose best gain lands far from the probe get a mismatched Kd, and the mismatch grows with
   authority×delay.
4. **Why it generalizes.** Sequential tuning is the standard method — Ziegler–Nichols finds Ku with
   D off, then reads Kd from a table. **This needs the firmware survey to be a claim about the
   field** (see Gaps). Without it, state it as: the textbook method fails on high-risk designs.

*Fig 7 — failure rate vs authority×delay, the dose-response curve.
Fig 8 — the two arms overlaid: same designs, same axis, one line each. The paper's key figure.
Fig 9 — the mechanism, as a Kp×Kd plane showing where the frozen-Kd choice lands vs the coupled path.*

### §7 Flight signature and hardware — 2.0 pp
C-FLIGHT: RMS 13.3° ± 5.2° vs 3.8° ± 2.7°, saturated fraction 0.60 vs 0.14, AUC 0.954
[0.907, 0.989]; single flight ≈ 0.907. Note the balanced 36/36 construction inflates AUC relative to
deployment prevalence.

**The hardware dataset goes here** — measured τ from the bench (spec posted to the engineer), the
bifilar Iyy, and whatever flight data exists by the deadline. Even a bench-only τ makes §5 and §6
concrete rather than parametric: it places *this* vehicle on the paper's own axes.

**Qualification that must appear, and it is the engineer's finding, credited:** a simulator PASS is
only evidence about failure modes the model can represent. ASC038 tumbled from a 143 ms SD write
inside the control loop — a failure the SIL could not express, because its clock is synthetic and
file writes cost zero simulated time. This paper's simulation results carry the same qualification.
Flight record: **2 of 6 attempts controlled.**

*Fig 10 — RMS distributions by class, with the diagnostic threshold.
Fig 11 — measured bench τ: the dead-time distribution across randomized PWM phase.*

### §8 Methods audit — 2.5 pp — **the methodology highlight**
Not an apology and not a limitations section. The claim: several published effects were properties of
the measurement protocol, and here is how that was found and quantified.

Lead with the **pre-registered test**: `frozen_kd_artifact_test.py`, 2026-06-22, decision criteria
written into the file *before* the run, asymmetric so the easy outcome was "carry on" — and 8 of 8
designs cleared the *most severe* threshold instead. That is the strongest single item in the paper
for the Student Contribution criterion.

Then the cascade: floor exponent +1.06 → +0.21 → −0.20 across protocols; the causal saturation claim
(128 of 142 designs already at SR=1.000 with saturation on; paired Δ +0.005; Wilcoxon p=0.128); a
validation check that asserted nothing for its entire existence (`theta_true` constant across 601
samples); a stored instability column inverted for 1,196 of 2,400 rows.

Close on the general rule the audit produced: **every experiment needs a positive control**, and a
result that matches expectation is the one to distrust.

*Fig 12 — the floor exponent across the four protocols, with the pre-registered threshold drawn.*

### §9 Limitations and what is not claimed — 0.75 pp
No mechanism isolated. No dimensionless invariant; the τ exponent is not identified (1.71 / 1.35 /
controlled test favored 1). No cross-architecture claim. Simulation-only except where §7 says
otherwise, with the ASC038 qualification. The failing-design label is soft. Sequential tuning being
*the community's* practice is argued, not yet surveyed.

### §10 Impact for builders — 0.75 pp
**Impact paragraph, not the thesis.** Tie Kd to Kp rather than tuning them separately. Screen a
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

1. **Firmware survey** — three GitHub firmwares already audited in `HEADLINE_FINAL.md`, plus
   BPS.space's documented process. Converts §6's generalization from argument to evidence. **Highest
   value, and it is a weekend of reading, not a compute campaign.**
2. **Measured τ** — spec going to the engineer; makes §7 concrete.
3. **Kd-free window rerun** — running; decides §5's conditional page.
4. **Re-score LQR/SMC on a common metric** — only if §5's window sections live.
