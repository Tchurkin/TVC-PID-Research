# Where the STS paper stopped, and why — 2026-08-21

**STS is OFF. Final.** Braxton's call, recorded on the shared board 2026-08-21. Do not reopen unless
he raises it unprompted; it has been decided and undecided three times already.

**⚠ It was NOT stopped because the science failed.** The Sep-7 gate was **passing, four weeks early** —
all three conditions met. It was stopped on timing: Top 300 lands Jan 7, after MIT EA (mid-Dec), UC
(Nov 30) and the whole early wave; and the paper does not feed ISEF, which is the propulsive-landing
rocket, a different artifact with a different narrative. So the 20 pages were never shared prep.

**Retarget: GSDSEF (~Jan–Feb) → ISEF (~May).** Still live: simulation, claim audits, supporting the
landing. `AI_LOG.md` discipline continues — ISEF has its own AI/ethics rules, just no November clock.

---

## What is finished and committed

| artifact | state |
|---|---|
| `OUTLINE_STS.md` | Complete. 10 sections, **exactly 20.0 pp**, 11 figures, all claims tagged, all standing rules recorded |
| `CLAIM_EVIDENCE_TABLE.md` | Every surviving claim with statistic, n, proving file; retired list; **claims ledger** of dated framing rulings |
| `TAU_MEASUREMENT.md` | τ measured at **~3.2 ms** vs 0.035 s assumed; why the 5–30 ms sim range still stands; blocked fast-regime experiment |
| `RETRO_FLIGHT_SIG_SPEC.md` | Pre-registration **+ results** (5/5, p = 0.100) **+ 08-21 amendment** for the two prospective flights |
| `S4_EVERYTHING.md` | Self-contained §4 writing guide — reference facts, 11 paragraphs, sidebar, ledger |
| `figures/` | **10 of 11 built** at 300 dpi. Fig 11 (τ decomposition) specified but not built |
| `hand-written section 4.md` | Braxton's draft, ~1,180 words, all 11 paragraphs. **His prose, uncritiqued fixes outstanding** |

Tools: `retro_flight_signature.py` · `flight_sig_usat_threshold.py` · `ceiling_fast_regime.py` ·
`triage_card.py` · `window_kd_free.py` · `s2r_replication.py` · `ceiling_kd_free.py`
Firmware: `Bench_Tau_ICM/` (the τ measurement, verified on a real Teensy compile)

## The results, in one place

- **C-INERTIA** — a failing design has lower pitch inertia than a surviving one **93.7%** of the time,
  Mann-Whitney **p = 9.45e-20**, median failure at the **5.4th percentile**.
- **C-CEILING** — ceiling ∝ 1/τ, authority-independent at fixed Kd. Replicated **3×**. Constant
  recalibrated to **0.0661/τ**. Fitted over **5–30 ms only**.
- **C-KD** — stale-D tuning manufactures hardware-indexed structure: **56/80 → 1/80**. Confirmed **5×**
  across independent analyses. Conditional claim; prevalence unknown.
- **C-FLIGHT / C3b** — pre-registered retrospective test, **5 of 5**, p = 0.100 (the archive's ceiling).
  Two further **prospective** flights pending (see the amendment).
- **τ** — measured **~3.2 ms**, 8–13× below the assumption; repositions the vehicle by ~120× on the
  authority×delay axis and puts it *below* the simulated range.

## If this is ever picked up again

1. **§4 fixes first** — the draft's P10 contradicts itself (concede the *screen*, not the map), and four
   ledger numbers are missing. All in `S4_EVERYTHING.md`.
2. **Score ASC045/ASC046** — CSVs were never committed to `Rocket data/`. Must go through
   `tools/retro_flight_signature.py`, not hand-computed RMS. ASC046 needs a powered-only window first.
3. **Fig 11** — spec is in the outline; needs a fresh bench `loop_us_median`.
4. **The fast-regime experiment stays blocked** until `gyro_noise_std` gets a physical definition.
   Read `ceiling_fast_regime.py`'s preflight before trying again.

**Nothing here is orphaned or half-committed.** The claim ledger and the standing naming/emphasis rules
are the most valuable things to preserve — they are what stops retired framings drifting back in.
