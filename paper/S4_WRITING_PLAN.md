# §4 — paragraph-by-paragraph writing plan

**What this is.** A build order for §4 with every number docked to the paragraph that needs it, so you
are writing rather than hunting through CSVs. **No sentences here are drafts.** Each entry says what a
paragraph must *accomplish* and what it must *carry*; the words are yours. Per `AI_LOG.md` and STS
Appendix 4, AI-drafted report prose is disqualifying — this document is structure, which is not.

**Budget: 3.75 pp** = 3.25 body + 0.5 boxed sidebar. At 11 pt / 1.5 spacing / 1" margins that is
roughly **1,100–1,300 words of body text plus Figures 3 and 4**. Eleven paragraphs. If you land at 1,600
words you have overwritten by a page and something must go — cut from P9 and P11 first, never P7.

**Section job:** answer *which builds fail, and can you tell before you build one?* This is the title
section and the paper's lead finding. A reader who stops after §4 should still have gotten the paper.

---

## The eleven paragraphs

### P1 — Opener (~80 words, no figure)
**Accomplishes:** states the question §4 answers and the terms it answers it in.
**Carries:** the four measurable numbers — thrust, nozzle moment arm, pitch inertia, loop delay.
**Do:** commit to *prediction before construction* as the frame in the first two sentences.
**Don't:** preview the tuning result. Under the emphasis ruling §4 stands alone.

### P2 — The design space (~90 words)
**Accomplishes:** tells the reader what population the map is over, so the map means something.
**Carries:** 2,400 designs, Latin-hypercube sampled; `max_gimbal_deg` 2.0–15.0 (median 8.6); the ten
fidelity modules by name-drop only. Refer back to Fig 1 rather than re-describing it.
**Don't:** re-open Methods. Two sentences of scope, then move.

### P3 — Fig 3, the failure map (~120 words) — **TITLE FIGURE**
**Accomplishes:** puts the map in front of the reader and tells them how to read it.
**Carries:** axes are Iyy × latency, 2,400 points, failures marked. **Cite Fig 3 explicitly in the
text** — an uncited figure is a disqualification risk, including your own.
**Do:** say what the eye should catch (failures clustered at low Iyy) before you quantify it.
**Don't:** write "every failure lies left of here." It is *true* (0.02904 vs 0.02920) but the margin is
0.5%, so it reads as false and invites a reviewer to measure your figure with a ruler.

### P4 — C-INERTIA, the headline (~110 words)
**Accomplishes:** the paper's lead number.
**Carries:** **36 of 36** failing designs below the population 25th percentile of Iyy; median Iyy
**0.19×** the population median.
**Do:** report the **count**, not "100%" alone — 36/36 tells the reader the sample size in the same
breath. This is the same discipline as 56/80 vs "70%" in §6.

### P5 — The joint fit (~140 words)
**Accomplishes:** shows inertia dominates without overclaiming that the others do nothing.
**Carries:** unregularized, standardized, bootstrap 95% CIs —
log(1/Iyy) **+2.37** [1.94, 3.14] · log(T·L) **+1.77** [1.30, 2.45] · log(τ) **+2.90** [2.24, 3.92].
Raw-log ratio T·L/(1/Iyy) = **1.18** where keff = T·L/Iyy would require 1.00.
**Do:** state plainly that all three are positive with CIs excluding zero.
**Don't — this is a claims-out item:** never write that thrust is irrelevant or protective. Dropping
T·L *costs* accuracy (CV AUC 0.9829 → 0.9687).

### P6 — The retracted reading (~60 words, one to three sentences)
**Accomplishes:** buys credibility cheaply and inoculates against a reviewer who refits your data.
**Carries:** an earlier "it is inertia, not authority" reading came from L2 regularization penalising a
manually-added intercept column; refitting unregularized reversed it.
**Don't:** let this grow. It is a subordinate clause's worth of honesty, not a story. The audit is §8.

### P7 — ⚠ THE PROTOCOL DEFENCE (~130 words) — **DO NOT CUT THIS**
**Accomplishes:** answers, before it is asked, the sharpest question a reviewer can put to this paper:
*§6 shows your tuning protocol manufactures spurious hardware-indexed structure — why is this result
not the same artifact?*
**Carries the structural argument, which settles it outright:**

| | tuner | D held stale while P sweeps? |
|---|---|---|
| §6, the artifact | `autotune_continuous` — probe Kd once at Kp = 40, freeze, sweep Kp to 320 | **yes, that is the mechanism** |
| §4/§5 labels | `autotune_grid` — nested Kp × Kd loop, **5 × 5 = 25 combinations scored jointly**, 2 search seeds | **no — impossible by construction** |

**Do:** lead with the structural argument. Mention the 1.9% label movement under full re-tune *after*,
as corroboration — it is the weaker claim and putting it first makes the defence look statistical when
it is actually definitional.
**Placement:** here, not in a footnote and not deferred to §8. A reader must reach §6 already knowing
§4 is immune.

### P8 — C-FRONTIER + Fig 4 (~130 words)
**Accomplishes:** turns a binary map into a graded one.
**Carries:** ρ = **−0.692**, p = 3.4e-10, n = 63, peak achievable success rate vs authority×delay,
under **per-design optimal tuning with fresh evaluation seeds**. Cite Fig 4 explicitly.
**Do:** say *why* it survived when the window sections did not — it was never scored on the retired
`window_ratio` metric. One clause.
**Axis-naming rule:** call it **authority×delay**, a plotting coordinate. Never Π, never dimensionless,
never a discovered parameter.

### P9 — What the frontier means (~80 words) — *first to cut if long*
**Accomplishes:** the practical reading — degradation is graded, so there is no cliff to hunt for.
**Carries:** nothing new. Interpretation only.

### P10 — The honest limit (~120 words)
**Accomplishes:** bounds the claim before a reviewer does.
**Carries:** **29 of 36** positives flagged `uncertain` by the project's own Wilson-interval check; the
four-number screen scores AUC **0.985** against the original label but **0.57** against an independent
harsher probe, with only **5 of 16** original positives failing at 2×.
**Do:** land on the distinction — *the map is real; the binary line drawn on it is not sharp.*
**Don't:** call it a controllability screen. Claims-out. It predicts the original labelling procedure.

### P11 — Close and hand off (~80 words)
**Accomplishes:** states what the map is for and points at §5.
**Carries:** screening from four measurable numbers before cutting metal; §5 asks *why* the failures
happen.

### SIDEBAR — "What surprised a builder" (0.5 pp, boxed, 5 rows)
Already specified in `OUTLINE_STS.md` §4 with all five rows and their statistics. **Communication
device, no new claims.** Two candidates were vetted and rejected — the broad "aerodynamics contributes
nothing" slogan and the dead-zone dither row. Do not ship either.

---

## Before you call §4 done

- [ ] Figures 3 and 4 each cited **by number in the running text**.
- [ ] Every count reported as a count (36 of 36), not a bare percentage.
- [ ] The word Π appears nowhere; the axis is "authority×delay" every time.
- [ ] No sentence says thrust is irrelevant or protective.
- [ ] P7 exists, is in the body, and leads with the structural argument.
- [ ] The soft-label limit (P10) is present — a §4 without it is overclaimed.
- [ ] Nothing previews or leans on §6. §4 stands alone.
- [ ] Word count 1,100–1,300 excluding the sidebar.

## Two things I can do once it exists

1. **Critique it** — hostile-reviewer pass on logic, whether each number supports the sentence it sits
   in, unstated assumptions, and anything a controls reviewer would attack. Unrestricted, and the most
   valuable thing available here.
2. **Minor grammar polish (`LANG`)** — permitted under Appendix 4 *with* a prompt-log entry and
   disclosure in Task 5. Word-level only; I will not rewrite sentences or restructure paragraphs.

Write P1–P4 first and show me those four. They set the voice for the whole paper, and it is much
cheaper to correct the voice at 400 words than at 20 pages.
