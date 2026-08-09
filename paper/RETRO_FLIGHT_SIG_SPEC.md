# Retrospective flight-signature test — pre-registration and spec

> **UPDATE 2026-08-09 — prep run complete, and it changed the spec.** The threshold-freezing rerun
> (`tools/flight_sig_usat_threshold.py`, 504 sims, positive control PASSED) produced three results that
> alter this document: **feature 2 is deleted** (§1), the RMS threshold ships with a **declared grey
> zone** (§1), and the paper needs a **wording fix in §7** (§4, T8). Details inline, marked ⟪PREP⟫.
> Still true: no flight log has been touched and no flight signature quantity has been computed.

**Status: SPEC, nothing run yet.** Written 2026-08-09, before any signature quantity has been computed
from any flight log. Braxton's proposal: evaluate the sim-derived failure signature against the
archived flight set, with the decision rule fixed in advance.

**Nothing like this exists yet.** No tool in `tools/`, no analysis in the repo, no figure. The sim side
is complete (`flight_sig_final_py.csv`, 72 designs × 7 seeds, recoverable from `bb22d36^`); the flight
side has never been touched. This document is the missing half.

---

## 0. What this test can and cannot do — read before running

**It cannot produce a significant result.** Five usable flights, two of them controlled. If the
signature classifies all five correctly, the one-sided exact p is **1/C(5,2) = 0.10**. That is the
ceiling, and it is fixed by the archive, not by the analysis. Anyone who reports "p < 0.05" from this
data has done something wrong.

**So why run it.** Three reasons, in descending order of value:

1. **Falsification.** A controlled flight scoring above the threshold, or an uncontrolled one below it,
   would qualify or kill C-FLIGHT for ~2 hours of work. Cheap falsification of a live claim is worth
   more than another confirmation of something already at AUC 0.954 in simulation.
2. **Calibration transfer — the most likely thing to change the paper.** The sim says healthy = 3.8°
   RMS and failing = 13.3°. Nobody has ever checked whether real flights land on that scale at all. If
   a healthy real flight reads 8°, the *threshold* does not transfer even if the *separation* does, and
   §7 has to say so.
3. **The estimator-dependence finding** (see T2). This one is hardware-only — simulation cannot produce
   it — and it is likely the single most interesting sentence this test generates.

**Framing for the paper:** this is a falsification check and a calibration check, not a validation.
Write it that way in §7.

---

## 1. Pre-registered decision rule — FROZEN

**Zero parameters are fit on flight data.** Every threshold below comes from the simulation study. That
is the real protection here, and it is stronger than blindness (see T5 on what was already visible).

### Primary rule — RMS attitude error

> **Predict FAILING if max(RMS_x, RMS_y) ≥ 5.62°, else HEALTHY.**

- **5.62°** is the Youden-optimal cut on the 72 sim designs (7-seed means): TP 35/36, FN 1/36,
  FP 7/36, TN 29/36, J = 0.778. Derived from `flight_sig_final_py.csv` **before** any flight log was
  opened. It is not adjustable.

- **Per axis, then take the worse.** The sim is single-axis pitch about θ_ref = 0. Taking each flight
  axis separately matches that construct exactly and avoids inventing a scale factor — combining into a
  tilt magnitude would need a √2 correction that assumes independent equal-variance axes, which is an
  extra assumption with nothing supporting it. The flight-level statistic is the **worse axis**, which
  is the operationally meaningful question ("does this build show the signature").
- **Reference is 0°**, matching `build_scenario()`: `theta_ref = 0.0`, `theta0_fixed_deg = 0.0`,
  `t_end = 3.0 s`. The sim task is *hold vertical under disturbance from a level start*, which is a
  genuinely good match to a boost. This was checked, not assumed — an initial-condition-decay task
  would not have transferred.

⟪PREP⟫ **Declared grey zone: 5.37° – 5.62°. A flight landing inside it is reported INDETERMINATE, not
classified.** Re-deriving the same threshold on the current simulator gives **5.37°**, not 5.62° — the
same 72 designs and the same 7 seeds, but the published per-run values are not reproducible on today's
code (only 1 of 504 runs matches; design-mean RMS differs by up to 9.35°). The *distributions* are
statistically indistinguishable — FRAGILE 13.35 ± 5.19 published vs 13.33 ± 5.08 rerun, KS p = 1.000;
EASY 3.79 ± 2.74 vs 3.78 ± 2.62, KS p = 0.885 — so the physics did not change; the RNG consumption path
did. Had the physics moved, the distributions would have moved with it.

That means the threshold is only determined to about ±5%, and pretending otherwise would be false
precision. **Declaring the grey zone now, before seeing any flight value, is the whole point** — it
removes the temptation to argue after the fact about a flight that lands at 5.5°. Freeze the *published*
5.62° as the nominal cut, because that is the paper's own dataset, and report anything in [5.37, 5.62]
as undecided.

### Secondary rule — ⟪PREP⟫ **DELETED. There is no defensible saturation feature.**

**The test is single-feature: RMS only.** This is not a simplification for convenience; the second
feature turned out to be unmeasurable in one direction and undefined in the other.

**Why the published feature can't be used.** The paper's "saturated fraction 0.60 vs 0.14" is
`slew_sat_frac` — the fraction of steps where the *rate* limit binds, at the sim's dt = 0.005 s
(200 Hz). **The flight logs run at 19–20 Hz.** A servo slew limit of a few hundred °/s cannot be
resolved from 50 ms samples. Not measurable, and no care makes it so.

**Why the obvious substitute can't be used either.** The plan was to re-derive a threshold for the
*position* clamp (`u_cmd_sat_frac`), which flights do give directly as rows at ±MAX_TILT. The rerun
shows **`u_cmd_sat_frac` is identically 0.000 for every design, every seed — 0 of 504 runs nonzero**,
FRAGILE and EASY alike. The signature study runs at a fixed Kp = 2 / Kd = 1, and at those gains the
commanded deflection is roughly an order of magnitude below the gimbal limit, so the position clamp
never engages. Only the rate limit does (441 of 504 runs nonzero, max 0.810). The sim therefore offers
**no basis whatsoever** for a position-saturation threshold, and a Youden fit on it returns the
degenerate cut 0.0000 with J = 0.000 — a rule that labels all 72 designs FAILING.

**So the two "saturation" numbers are different physical quantities wearing the same word.** The sim's
0.60 / 0.14 is a *rate* limit binding at low gain. The flight archive's 88.6% (LOG001) and 17.4%
(ASC007) are the *gimbal sitting on its mechanical stop* at flight gains. Putting them in the same
table would be a category error, and it is one this pre-registration exists to have caught. It also
means the flights saturate in a way none of the 72 sim designs ever does — worth one sentence in §7 as
a fidelity gap, and see T8.

### Reporting

Confusion table over 5 flights, each flight named, plus the one-sided exact p with its 0.10 ceiling
stated in the same sentence. **No AUC** — an AUC on 2 vs 3 is not a meaningful summary and inviting the
reader to compare it to 0.954 would be misleading.

---

## 2. Data inventory — what is actually in the archive

| run | date | Hz | rows | schema | outcome | role in this test |
|---|---|---|---|---|---|---|
| **ASC038** | 2026-08-04 | 8.5 *(starved)* | 46 | full + estZ | FAILURE — uncontrolled | test, **two analyses** (T2) |
| **ASC036** | 2026-07-20 | 19.2 | 68 | full + estZ | PARTIAL — controlled boost | test, **HEALTHY** |
| **ASC031** | 2026-07-12 | 20.0 | 125 | full | SUCCESS | test, **HEALTHY** |
| **ASC007** | 2026-07-07 | 19.2 | 46 | full | FAILURE — self-aborted | test, **FAILING** |
| **LOG001** | ≤2026-06-28 | 19.2 | 70 | no Phase col | FAILURE — diverged | test, **FAILING** |
| ASC037 | *unlogged* | 19.2 | 134 | full + estZ | **STATIC FIRE** — see below | pipeline control only |
| RES005 | ≤2026-04-19 | 20.0 | 126 | TVC off | passive | pipeline control only |
| MTR000 | 2026-07-20 | — | 343 | motor only | no ignition | excluded |

**n = 5: two controlled, three uncontrolled.**

**⚠ `ASC037.CSV` is on the card and in no log.** Characterized 2026-08-09 (metadata only, no signature
quantity computed): 134 rows, 6.96 s, thrust present (peak axial 22.24 m/s² against 8.7 at rest), TVC
commanded on every row, Phase advancing 1 → 2 — but **maximum altitude 0.78 m**. It never left the
ground. Almost certainly the static fire matching the untracked `Firmware/StaticFire/`. **It needs a
`FLIGHT_LOG.md` row with a `[CONFIRM]` on its purpose**, independently of this test. It is excluded
from the signature test (a clamped vehicle has no attitude dynamics) but is a *useful pipeline control*:
thrust is real, so the boost-window detector must find a burn, and attitude RMS must come out small.

**Labels are pre-assigned from documented outcome only** — recovered-and-controlled vs not — taken from
`FLIGHT_LOG.md` as written before this test existed. They are **not** derived from tilt, saturation, or
anything else the signature uses.

---

## 3. Metric extraction — exact definitions

**Boost window.** `Phase == 1` for schemas that carry it (ASC007, ASC031, ASC036, ASC037, ASC038). For
LOG001, which predates the Phase column, the window is first-to-last row with axial acceleration above
the 1.5 g onset threshold used by `MTR###` logging. State which rule each flight used.

**Attitude.** Two passes, both reported:

- **(a) Roll-aware replay — PRIMARY.** Re-integrate attitude from the logged raw body rates with the
  quaternion propagation used by the current firmware. Required because the naive estimator that flew
  on ASC007/ASC031/LOG001 **inverts past ~120° roll** (documented in `FLIGHT_LOG.md`: X-channel sign
  disagreement on 69% of ASC007 rows above 120° roll), and every one of those flights rolled 170–300°.
  Using the as-logged estimate as primary would measure the estimator's bug, not the vehicle.
- **(b) As-logged — SECONDARY.** What the flight computer actually believed. This is the version a real
  builder's diagnostic would see, so it answers a different and also useful question.

**Saturation.** Fraction of boost rows at ≥0.95 × MAX_TILT. Note per flight that `TVCx/TVCy` (and
LOG001's `ServoX/ServoY`) are **post-clamp**, so this measures rows *at* the limit and cannot show how
much command was being discarded. The 500 Hz `CTL000.CSV` pre-clamp channel exists for ASC038 only.

---

## 4. Threats, and how each is handled

**T1 — Construct mismatch. The most important one.** The sim's FRAGILE label means *narrow gain
window*, a tunability property. The real failures have different root causes: ASC007 was thrust
misalignment consuming authority; LOG001 is pre-measurement and unattributed; ASC038 was SD-write
starvation freezing the estimator. **None of these is "this design has a narrow gain window."**

So this test does **not** validate the FRAGILE construct, and must never be described as doing so. What
it tests is the claim as C-FLIGHT is actually worded — *a failing build is identifiable from one
instrumented flight*. If the signature separates controlled from uncontrolled flights whose root causes
the sim never modelled, that is arguably a **stronger** operational result (the diagnostic generalizes
past its training construct) — but it is a different claim, and §7 must say which one is being made.

**T2 — Estimator dependence, and a pre-declared expected miss.** ASC038's `dt < 0.1` guard skipped
`quatPropagate()` for the whole flight, so the logged attitude **froze** at tiltX −0.10 / tiltY −2.43
while real body rates reached 248 °/s. The as-logged RMS will therefore be near zero and the primary
rule will call ASC038 **HEALTHY** on a flight that reached ≥161° of true tilt.

**This is pre-declared, now, before running.** It is *not* evidence against the signature's physics —
it is evidence that **the signature inherits its estimator's failure modes**, which is a genuine finding
that only hardware could produce. Handling: report analysis (a) replayed-from-raw-rates as the physics
test, report analysis (b) as-logged as the estimator test, and state the divergence as a result rather
than burying it. Counting this as a "miss" against C-FLIGHT would be wrong; hiding it would be worse.

**T3 — Roll inversion.** Handled by the replay in §3(a). Report both passes so the size of the
correction is visible.

**T4 — Heterogeneous vehicles.** ASC007/031/036 flew the destroyed airframe (keff ≈ 257); ASC038 flew
the rebuild (keff ≈ 130); LOG001 predates the mass/inertia measurement entirely. The signature is
claimed to be design-independent, so this is in scope — but with n = 5 it also means no two flights are
strictly comparable. State it.

**T5 — Pre-registration is partial, and here is exactly how.** `FLIGHT_LOG.md` already publishes, for
every flight, the peak boost tilt and the TVC saturation percentage. **I am therefore not blind to
feature 2 or to a close correlate of feature 1.** Do not claim a blind test.

What *is* true and is the actual protection: **the thresholds are derived entirely from the simulation
study and nothing is fit on flight data**, so the rule cannot have been tuned to these five flights.
RMS over the boost window has never been computed for any flight and is genuinely unseen. Say both
things plainly in §7 — a paper that overstates its own pre-registration deserves the reviewer it gets.

**T6 — Sample rate.** 19–20 Hz vs the sim's 200 Hz. RMS of a smooth attitude signal is robust to
decimation; the slew-rate feature is not, which is why it is replaced (§1). ASC038 logged at an
effective 8.5 Hz because the loop was starved — flag its RMS as the least trustworthy of the five.

⟪PREP⟫ **T8 — Paper wording fix, independent of this test.** §7 currently reports "saturated fraction
0.60 vs 0.14" with no qualifier. A builder reads "saturated" as *the gimbal hit its stops*, and that is
**not** what the number measures — it is the slew-rate limit binding. Relabel it **"slew-rate saturated
fraction"** wherever it appears (§7 text and Fig 10's caption). This is a clarity fix that stands on its
own whether or not the retrospective test ever runs.

**T7 — Truncated logs.** ASC036 stops at burnout with 3.49 s of a 3.65 s window, so its boost coverage
is ~96% and RMS is nearly complete. ASC007 ends at its abort, 2.31 s in — its window is *the flight it
had*, which is correct for this purpose but worth a note.

---

## 5. Prep work required before the test can run

1. **Verify `sim/` (rule 5).** `git status sim/` clean ✔ (checked 2026-08-09); `validate.py` must be
   10/10 — running at time of writing.
2. ~~Freeze `T_sat`~~ — **DONE 2026-08-09, and the answer was "there is no such threshold."**
   `tools/flight_sig_usat_threshold.py`, 504 sims. Positive control PASSED to two decimals on all four
   published statistics (FRAGILE RMS 13.33 vs 13.30, EASY 3.78 vs 3.80, slew 0.60 and 0.14 exact), so
   the null result is interpretable rather than a broken run. Feature 2 is deleted; see §1.
3. **Add the `ASC037` row to `FLIGHT_LOG.md`** with `[CONFIRM]` on whether it was a planned static fire.
4. **Write the extraction tool** — single feature now, which makes it smaller: boost-window detection,
   the roll-aware replay, per-axis RMS, and the two ASC038 passes.
5. Then, and only then, run it.

**Note for anyone reusing the sim's per-run outputs.** Historical CSVs at `bb22d36^` reproduce in
*distribution* but not run-for-run on today's simulator (§1 grey zone). Aggregate results from that era
stand; any analysis that needs a specific design-seed value must regenerate it rather than read it from
the old CSV.

## 6. Deliverable

**Figure:** flight points overlaid on the existing Fig 10 sim distributions — five markers on the RMS
axis against the FRAGILE/EASY densities, with the 5.62° threshold drawn. That single panel shows
separation *and* calibration transfer at once, which is the whole point.

**Text:** ~0.4 pp inside §7's existing 2.0 pp budget. No new page allocation. If the result is a clean
separation it is one paragraph plus the figure; if the threshold fails to transfer, that is the more
interesting paragraph and the same length.

**Effort:** roughly half a day, entirely on data already on disk. This is the cheapest genuine hardware
validation available, which is exactly why it is worth doing before the ballast sweep or the
delay-injection stand — both of which need flight time that the landing campaign owns.
