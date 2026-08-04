# TVC Rocket Research – Claude Context

> Last substantive update: **2026-08-04**, after a full claim-survival audit.
> Read `~/.claude/shared/STATUS.md` at session start — other Claude sessions (engineer,
> college-apps) post cross-cutting news there, and this project has already needed it several times.

---

## ⚡ CURRENT STATUS — read this first

**Phase:** hardware / firmware. The simulation campaign is complete and has been audited.

**Two separate deliverables. Do not fuse them:**
- **Regeneron STS** (due 2026-11-05, 8pm ET; self-imposed go/no-go on Labor Day 2026-09-07) —
  the TVC simulation paper. Completed research with data, which is what STS requires.
- **ISEF / GSDSEF** (~March 2027) — the propulsive-landing rocket. Ongoing engineering that will
  not be finished by November but will be by spring, and ISEF rewards flight validation.

**Writing rule (binding):** Braxton writes every word of paper and essay prose. STS Appendix 4 makes
AI-drafted reports a disqualifying violation. Claude analyses, verifies, critiques, checks math, and
writes CODE — never prose. Log AI use in `AI_LOG.md` (Part B, contemporaneous).

**Vehicle (MEASURED 2026-08-03, post-ASC036 rebuild — ASC036 destroyed the previous airframe):**
mass 0.927 kg, Iyy 0.0176 kg·m² (bifilar), L = 0.16 m, linkage 4.5:1 → **keff ≈ 130 rad/s² per rad**.
The old airframe (flew ASC007/031/036) was Iyy 0.0078, mass 0.818, L 0.14 → keff 257. **Those numbers
describe a different rocket and must not be reused.** Flight gains are still pole-placed for keff=257
and are therefore under-gained on the rebuild — flagged in the firmware header, not yet applied.

---

# Research Philosophy

Act as a skeptical reviewer. Do NOT optimize for exciting conclusions. Do NOT defend previous work
automatically. Always: inspect data before drawing conclusions; quantify uncertainty; search for
alternative explanations; identify circular reasoning and confounds; distinguish visualization from
evidence, and correlation from causation. Prefer falsification over confirmation. If a conclusion is
weak, say so.

## Hard-won methodological rules (2026-08 audit — these cost real time)

1. **Every experiment needs a positive control.** If a new protocol cannot reproduce a known result,
   its novel result means nothing. Three void runs were spent learning this on the gain-ceiling
   experiment.
2. **A striking result that matches your expectation is the one to distrust.** Every self-inflicted
   error this week produced a number that flattered the hypothesis going in.
3. **Check which subset you computed on.** The same ceiling run gave keff = −0.34 (t = −5.2) on a
   censoring-selected paired subset and −0.08 on the full uncensored set — opposite conclusions,
   same data.
4. **Protocol-conditional effects are the default, not the exception.** Frozen Kd, sample
   stratification, seed count, grid ceilings, and initial conditions have each been shown to create
   or destroy a headline result in this project.
5. **Simulation results are void unless `sim/` is verified.** Check `git status sim/` and run
   `cd sim && python validate.py` (must be 10/10) before trusting any run.

---

# What the audit established (2026-08-02 → 08-04)

## The central result

**Effects attributed to rocket hardware were substantially artifacts of tuning the derivative gain
independently of the proportional gain.** `tools/s2r_replication.py`, n=2400, two arms paired on the
same designs:

| | Π < 300 | Π ≥ 300 |
|---|---|---|
| Arm A — sequential tuning (P then D), the published protocol | 0.087 | **0.700** |
| published | 0.091 | 0.613 |
| Arm B — coupled tuning (Kd tied to Kp) | 0.022 | **0.013** |

Arm A reproduces the published curve **bin by bin**; Arm B is identical except the tuner ties Kd to
Kp. ρ(log Π, fail): A = +0.236 (p = 1e-31), B = −0.117 (wrong sign). Failure at high Π drops
**70% → 1.3%**.

Mechanism: `autotune_continuous` probes Kd once at Kp = 40, freezes it, then sweeps Kp to 320, so
designs whose best gain lands far from the probe get a mismatched Kd — and the mismatch grows with
authority × delay. Sequential tuning is the *standard* method (Ziegler–Nichols is sequential), so
this is a finding about common practice, not about one script.

Same cause, four independent appearances: the `window_ratio` family (floor inflated 1.8–2.9×, keff
exponent +1.06 → +0.21 → −0.20), the gain ceiling (**8.8× higher** with Kd free), sim-to-real
(above), and Braxton's own pre-registered `frozen_kd_artifact_test` (8/8 past its own bar, 2026-06-22).

## Alive

- **Gain ceiling ∝ 1/τ, independent of authority.** Independently replicated on v2's protocol and
  designs: keff coef **−0.082, CI [−0.21, +0.05]**, τ exponent **−1.067**, positive control
  ρ = −0.762 vs published −0.74. State it as "**at a fixed Kd**" — the ceiling is 8.8× higher when Kd
  is free, and a builder flies one Kd. Constant recalibrated: **0.0661/τ** (published 0.042/τ is
  ~1.6× low and sits nearer a conservative bound). Effect is sample-conditional: ρ = −0.76 on v2's
  authority-stratified designs, **−0.38** on a population-representative sample.
- **Flight-detection signature.** RMS 13.3° ± 5.2° vs 3.8° ± 2.7°, saturated fraction 0.60 vs 0.14,
  AUC 0.954 [0.907, 0.989]; single flight ≈ 0.907. Untested on hardware.
- **Exp1's reference gains are sound.** 0.973 out-of-sample vs 0.998 recorded; re-tuning helps only
  6% of designs. (An earlier claim that they were single-seed overfit was **wrong** — it came from
  reading a corrupted working tree; the real `autotune_grid` uses two search seeds and says so.)

## Dead — do not restate as findings

- **Π = keff·τ² as a parameter.** Ties or loses to keff alone; the controlled exponent test favoured
  α = 1 over 2.
- **The causal saturation-removal claim.** 128 of 142 designs already score SR = 1.000 with
  saturation ON; mean paired effect +0.005, Wilcoxon p = 0.128. The n=15 factorial rests on 2
  designs, and ADRC is *worse* than PID (0.533 vs 0.600) on the extreme design R2072.
- **The whole `window_ratio` family**, including the LQR/SMC/MPC controller-invariance results as
  scored on that metric. (LQR/SMC's own `n_pass` statistics are sound; the PID row they were compared
  against is not.)
- **"Each correction strengthened the AUC (0.944 → 0.957 → 0.975)."** The labels changed between
  passes; the AUCs are not comparable.
- **The four-number screen as a controllability screen.** AUC 0.985 against the original FRAGILE
  label but **0.57** against "actually fails when the gain is pushed 2× or 3×"; only 5 of 16 original
  positives fail at 2×. It predicts the original labelling procedure. Retain it, if at all, as
  "predicts sensitivity to sequential tuning."
- **"Strong p_unstable interaction."** r = −0.004 (p = 0.84). `p_unstable` is legacy and unread by
  the EOM (`plant_dynamics.py:102`); the EOM uses `lambda_aero`, which does no better (−0.008).
  Worse, the stored column is inverted — 1196 of 2400 rows read 0 while spanning `lambda_aero` up to
  +61.5. The paper already rejected this as H3; only this file had asserted it.

---

# Where things live

- **Sim-phase tools and result CSVs were deleted from the working tree** by `bb22d36`. Recover with
  `git archive bb22d36^ experiments/results tools | tar -x -C <scratch>`. Restore to a scratch dir,
  never into the repo.
- The Π-research context file is archived at `archive/CLAUDE_pi_research_2026-06.md`.
- Cross-session board: `~/.claude/shared/STATUS.md`.
- ⚠ `sim/` was found reverted to a pre-2026-06-05 state (57.3× servo-slew bug, `experiment_runner.py`
  missing 1,734 lines), almost certainly the MATLAB-Drive sync corruption behind the 2026-07-13
  relocation. The corruption was never committed. Verify before every campaign.

---

# Open questions

1. **Does the flight signature survive real flight?** The only claim with a cheap hardware test.
2. **Is sequential tuning demonstrably the community's practice?** Needs a survey of the three
   GitHub firmwares already audited in `HEADLINE_FINAL.md`, not a simulation. This is what turns the
   central result from "a property of one tuner" into a claim about the field.
3. **What is τ on the real vehicle?** Still unmeasured — `Firmware/Bench_Latency/` exists to measure
   it. Every hardware prediction is indexed on it and currently uses an assumed 0.035 s.
4. **Should the flight gains be rescaled for keff = 130?** P = 0.492 / D = 0.123 restores the design
   target, and both scale by the same factor so the P/D ratio is preserved. Needs SIL + bench check.

---

# Flight Validation Plan

Attempt to falsify, not confirm. Flight data outranks simulation. Priority: (1) measure τ and the
servo slew on the bench; (2) the flight signature at a known gain; (3) an Iyy/ballast sweep, now the
most informative hardware experiment since inertia is the largest single coefficient in every fit.

---

# Response Style

Explain concepts clearly. Assume the researcher is still learning advanced experimental design and
statistics. When identifying flaws: explain why they matter, explain their practical impact, and
suggest fixes. Avoid unnecessary jargon. Teach while critiquing.
