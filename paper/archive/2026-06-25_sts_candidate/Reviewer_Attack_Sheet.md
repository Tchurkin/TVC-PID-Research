# Reviewer Attack Sheet — TVC Saturation-Transition Paper

**Purpose.** A working document for mentors, controls reviewers, and the author. It lists the
strongest objections a skeptical controls PhD reviewer would raise, the paper's *current* honest
response (with section pointers), and the single experiment that would actually settle each one.
The point is to make an external reviewer's hour productive: instead of re-deriving what is already
known, they can go straight to the live weak points.

**How to read the severity column.** "Severity" = how much this objection threatens the central
claim *if left unaddressed*. "Defensibility now" = how well the current manuscript already answers
it. The objections are ordered by what the author considers the most genuinely vulnerable first.

The central claim under attack, stated once: *In delayed TVC attitude control there is a regime
transition — organized by the dimensionless parameter Π = k_eff × τ² — from a linear regime (any
sub-ceiling gain works) into a saturation-dominated regime where classical PID/ADRC linear
equivalence breaks down. The transition is established by a saturation diagnostic (fsat), a causal
intervention (saturation removal), and the onset of ADRC's advantage, and its mechanism is isolated
by a rate-filter ablation.*

---

## The objections, most-vulnerable first

| # | Criticism | Severity | Defensibility now | Current response (paper §) | What would settle it |
|---|-----------|----------|-------------------|----------------------------|----------------------|
| 1 | **Simulation-only.** "Cool simulator. Does reality care?" Every claim rests on one simulator. | **High** | Weak — openly conceded | Confined to honest scope language throughout; full validation plan in §8.1. No physical data exists yet. | **Hardware experiment G** (§8.1): raw vs. IIR-filtered gyro, same rocket/gains/motor/wind, on a high-Π build. Measure fsat, control effort, oscillation, SR. This tests the *mechanism*, not just a prediction. |
| 2 | **The headline transition rested on the smallest dataset (n = 29).** | **Medium** (was High; now replicated) | Strong, with one honest concession | **RESOLVED by replication (2026-06-24, `saturation_transition_large.py`, combined n = 142).** Outcome reported straight: (a) the binned dose-response *strengthened* — fsat rises cleanly 0.007 → 0.59 across Π tiers; (b) the causal test *strengthened* — SR_nosat ≈ 0.99 across 142 designs (Finding 8 extended from n = 15); (c) the rank correlation was *revised down*, ρ = 0.55 (n = 142) vs the optimistic 0.80 (n = 29); (d) a new concession surfaced — on the latency-1–6 population Π does **not** out-rank k_eff (ρ 0.55 vs 0.54), so the product-specific claim now rests on R0522, the latency-extended data, and theory, not the aggregate correlation (§4.0.0). | Residual: the high-Π tail (Π > 800) is still only 5 designs because the population physically lacks them. A fresh-LHS high-Π extension would populate the saturated end, but the onset (the claim under attack) is now well-sampled. |
| 3 | **τ² is not proven.** The exponent is the most mathematically vulnerable piece. | **Medium** | Strong — explicitly hedged | "The blind-spot derivation predicts τ² scaling, and the data are consistent with that prediction, though the exponent is not uniquely identified by the available experiments" (§4.0.4). The window-compression exponent is τ⁻¹ (ceiling-only); τ² lives only in the saturation-onset parameter, from blind-spot kinematics. A controlled latency sweep could not separate τ^1.6/2.0/2.2. | **Fixed-k_eff latency sweep** with enough latency levels and seeds to fit the exponent with a tight CI — explicitly the experiment named as currently inconclusive. Likely still won't *prove* 2.0, but would bound it. |
| 4 | **ADRC causality is incomplete.** "Maybe ADRC wins because it produces smoother control, not because it avoids saturation." | **Medium** | Strong — claim already narrowed | Only *coincidence of onset* is claimed (§1.4 step 5, §6.2 conclusion). What is *measured*: PID-nosat = 1.000 (saturation necessary+sufficient for PID failure), ADRC slew_frac = 0.000. What is *not* claimed: that disturbance cancellation is the sole reason ADRC wins. | **ADRC saturation-forcing experiment**: force ADRC into saturation (cap its effective authority) and check whether its advantage disappears. If it does, the saturation-avoidance mechanism is isolated. |
| 5 | **Is this a phenomenon of your simulator or of delayed TVC systems?** The deepest version of objection 1. | **High** | Medium-strong | Cross-system generalization: ρ(log Π, SR) < 0 for TVC (−0.668), quadrotor (−0.937, n = 50), inverted pendulum (−0.647) — direction consistent across three different second-order plants (§4.0.5). Plant-structure invariance: Π_crit holds across ×4 variation of Cm_alpha (exact zero effect), Iyy, motor_scale, max_gimbal (§4.4.x). Minimal-physics test isolates which mechanism needs aero. | **Hardware (G)** plus, ideally, the same Π-sweep on a second physical platform (e.g., a Betaflight quad with software loop-rate changes sweeping τ at fixed hardware). |
| 6 | **Π_crit is not a universal constant** (you report 177 / 233 / 275 / 321 / 407). | **Low** | Strong — reframed as a feature | Reported as a transition *band* (Π ≈ 0.2–0.5 physical), not a line. The spread tracks the delay implementation (bare-metal FIFO → partial-lag), and the *direction* of the shift follows the mechanism (§4.4.6). A transition that landed at exactly one value for every implementation would be suspicious. | More delay implementations (RTOS profiles, hardware timers). Marginal value — already a robustness result. |
| 7 | **AUC on a ~1.5% base rate hides a base-rate trick.** | **Low** | Strong — superseded | The binary classifier is explicitly demoted to audit trail / a one-number builder screen (§4.1–4.6). The headline is the transition (§4.0.0) and its continuous dose-response (§4.0, R² = 0.33 CV), which carry no base-rate ambiguity. | None needed; framing already concedes the point. |
| 8 | **ADRC = filtered PID (Carlson 2025), so "cross-architecture" is overstated.** | **Low** | Strong — self-disclosed | Found by the project's own literature check, not a reviewer; reframed everywhere to "two disturbance-handling conventions, divergent in the saturated regime" (§6.0). The divergence is real because the linear equivalence provably breaks under saturation — confirmed by the 2×2 factorial (linear regime: PID = ADRC = 1.000). | None needed; disclosed and reframed. |
| 9 | **The probe gain Kp = 190/τ was chosen — is the transition an artifact of that operating point?** | **Low** | Strong | Gain-scaling probe test: fsat and Π_crit are essentially unchanged across a 6× range of probe gains (§4.4.x / gain_scaling_probe). The probe at half the linear ceiling guarantees any saturation seen is nonlinear, not linear-instability. | None needed. |
| 10 | **You retired a headline result (the floor law) — why trust the rest?** | **Low** | Strong — turned into an asset | The floor-law collapse is documented as the project's largest self-correction (§9.2): a confident scaling law (held-out R² = 0.71) dismantled by the project's own follow-ups. The transition survived *because* it never depended on the floor (Π's τ² is from blind-spot kinematics). Self-falsification of one's own most confident result is evidence of method. | None needed; the surviving claim is independent of the retired one. |

---

## The three to actually worry about

1. **Hardware (objection 1/5).** Nothing in the paper is physical. One clean experiment G result is worth more than any amount of additional simulation. This is the single highest-leverage item in the whole project.
2. **n = 29 headline dataset (objection 2) — NOW CLOSED.** Replicated at n = 142 (2026-06-24). The dose-response and causal test strengthened; the correlation honestly dropped to 0.55; a new keff-vs-product concession surfaced and is now disclosed. The "why the smallest dataset?" question is gone; what remains is the honest nuance that latency's narrow range in the main population prevents the τ² factor from being separately identifiable there.
3. **τ² identifiability (objection 3).** Already hedged correctly. Do not over-defend it; the reviewer-proof sentence is enough. A fixed-k_eff latency sweep would bound the exponent but is lower priority than 1–2.

## What is NOT worth defending further

Objections 6–10 are already at "strong" defensibility. Spending more effort there is diminishing
returns — the reframings (band not constant; binary as audit trail; Carlson disclosed; probe-gain
invariance; floor-law collapse as method) are complete. A reviewer who presses these is pressing on
the project's strongest, most self-aware ground.

---

*Maintained alongside `paper/Tentative_Paper_Draft.md` and `paper/Paper_Condensed.md`. Section
numbers refer to the full draft. Update when a new experiment closes one of the "what would settle
it" cells.*
