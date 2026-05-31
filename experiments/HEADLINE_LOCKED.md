# Locked Scientific Headline (Updated) — Unified H1 + H2 Finding

**Date:** May 2026 discovery run (session 2)
**Status:** LOCKED — two reproducible findings from controlled simulation, sharing one mechanism.

---

## Unified Headline

**Constrained-TVC stabilization is anti-monotone in controller aggression.**
For unstable amateur airframes with rate-limited gimbal actuators, stabilization success **peaks at deliberately under-tuned operating points** along two axes simultaneously:
1. **Actuator authority axis (H1).** Success is non-monotone in saturation limit `u_max`: there is a slew-and-instability-dependent optimum `u*_max(p, slew)`. Adding authority beyond it degrades success by 8–40 %.
2. **Controller design-point axis (H2).** Under realistic stress, an LQR sized for the *low* end of the expected pole range outperforms LQR sized for the midpoint or worst-case end by 12–40 % across most operating cells. Midpoint dominates only when the burn-out pole exceeds ≈4× the pre-burn pole.

Both effects share one mechanism: **aggressive feedback commands excursions the rate-limited actuator cannot deliver in time, producing wind-up and loss of recovery margin during the unstable transient.** Classical single-limit composition and standard "design for worst case + add headroom" practice both miss this regime.

## Practical design rules emerging from the data

1. **Size `u_max` near the H1 optimum**, not at the maximum the servo can deliver. `u*_max` grows roughly linearly with slew and with `p`.
2. **Design LQR for the conservative end of the expected pole range** — the under-tuned controller dominates unless your burn shifts the pole by 4× or more.
3. **Slow drift is not inherently safer.** With a fixed controller designed for `p0`, slower pole drift to `p1=16` gives 54 % success vs 96 % at fast drift (counter-intuitive: longer burns expose the system longer to controller-pole mismatch).

## Evidence

### H1 lock evidence (sessions 1–2)
- ~6 000 monte-carlo trials. `experiments/results/h1_lock_summary.csv`.
- p ∈ {8, 10, 12} × slew ∈ {25, 45, 80} × two LQR designs × 21 u_max points.
- Drops past optimum: -0.08 to -0.40 in success rate.
- Reproduces in both aggressive (Q=diag(400,2), R=0.5) and gentle (Q=diag(80,1), R=4) LQR.

### H2 lock evidence (session 2)
- `experiments/results/h2_drift_long.csv` (low-stress) and `experiments/results/h2_stress_summary.csv` (stressed).
- Low-stress: design=mid yields 100 % success across the full (p1, t_burn) grid. p0-design fails at p1=16; p1-design fails at p1=16 with slow drift.
- Stressed (u_max=3.5, slew=25, gust_std=3.5): mean success p0=0.82, mid=0.71, p1=0.42.
- Counter-intuitive slow-drift penalty: with p0-design, t_burn=4 s gives 0.54 success at p1=16 vs t_burn=0.5 s giving 0.96.

## Practical tool: the bench → safe-envelope validator

The bench INO `Firmware/feedback_servo_calibration.ino` already measures slew rate (rolling_up_net_slew_deg_s, rolling_down_net_slew_deg_s) and endpoint utilization. The validator pipeline is:

```
1. Bench: run feedback_servo_calibration.ino → measured slew (deg/s), measured deflection envelope
2. Pre-flight estimate: compute p from airframe geometry + thrust + CG margin
3. MATLAB lookup: experiments/validator/recommend_envelope.m
     -> recommended u_max  (from H1 optimum)
     -> recommended Q,R    (from H2 design rule = "conservative end")
     -> go/no-go flag      (controllable region check)
4. Output: numbers to flash into SisyphusCode.cpp limits
```

This is the deliverable for amateur TVC builders, now anchored to two real scientific findings instead of being engineering glue.

## What is still NOT shown

- **H3 (identifiability):** Can we reliably estimate `p`, `keff`, slew from a pre-launch "wiggle test"? If not, the validator's recommendations are only as good as a human estimate of p. Scaffold building next.
- **Constraint-aware controllers:** Have not yet tested whether sigma-MRAC, anti-windup, or MPC eliminates either H1 or H2 effect. If they do, the framing shifts to "naive LQR is fragile in a predictable way."
- **Realistic-plant transfer:** All findings are in `sim_unstable.m`. Need to add a static-instability term to `simulate_case_realistic.m` and confirm both findings persist on the project's primary plant.
- **External model cross-check:** Per user preference, this should be cross-validated with at least one other LLM before paper draft.

## Why this can plausibly clear the STS finalist bar

- **Two independent counter-intuitive findings** (H1, H2) with one **unified mechanism** — that's a coherent narrative, not a single anecdote.
- **Reproducible** across multiple parameter axes, two LQR designs, ~10 000 total monte-carlo trials.
- **Connects to existing theory gap** (multi-limit composition for unstable plants; constraint-aware LQR design).
- **Direct hardware-actionable deliverable** (the validator tool) that other amateurs can use.
- **Bench-rig validation path** already exists in the workspace.

## Files (this session)

- `experiments/sim_unstable.m`, `experiments/classify_outcome.m`, `experiments/design_lqr_unstable.m`
- H1: `h1_coupling_sweep.m`, `h1_analyze_coupling.m`, `run_h1.m`, `run_h1_stressed.m`, `run_h1_verify.m`, `run_h1_lock.m`
- H2: `h2_timevarying_sweep.m`, `h2_driftrate_sweep.m`, `run_h2.m`, `run_h2_drift.m`, `run_h2_stress.m`
- Evidence: `results/h1_lock_summary.csv`, `results/h2_drift_long.csv`, `results/h2_stress_summary.csv`

## Next actions

1. Build H3 (pre-launch identifiability) scaffold — last leg.
2. Build validator stub `experiments/validator/recommend_envelope.m`.
3. Cross-check both findings against `simulate_case_realistic.m`.
4. Cross-check narrative with second model per user preference.
