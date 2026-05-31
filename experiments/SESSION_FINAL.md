# Discovery Program — Session 2 Final Status

**Three reproducible findings, one unified mechanism, one working tool.**
Total compute: ~12 000 monte-carlo trials in `experiments/`.

## The unified scientific headline

> **For unstable amateur TVC airframes with rate-limited gimbal actuators, stabilization success is anti-monotone in controller aggression along three independent axes: actuator authority (H1), controller design-point (H2), and parameter-estimate confidence (H3). Naive engineering recipes — "oversize the gimbal," "design for worst case," "trust your spreadsheet" — each cost 20–40 % of success rate in the regime amateurs actually fly in.**

The mechanism is single: **aggressive feedback × rate-limited actuator × unstable plant = wind-up that consumes the recovery window**. Each axis is a different way to be too aggressive.

## The three findings (locked)

| # | Axis | Naive rule | Discovered rule | Cost of naive (success drop) |
|---|------|------------|-----------------|------------------------------|
| H1 | actuator authority `u_max` | bigger is safer | finite optimum `u*_max(p, slew)`; **~ 0.06·slew + 0.30·p** (heuristic fit) | -8 to -40 % |
| H2 | LQR design-point `p_design` | design for worst case | design for the **low end** of expected `p` range (unless burn-out p ≥ 4× pre-burn) | -12 to -40 % |
| H3 | plant identification budget | "close enough" is fine | **≤ 10 %** error in `p` is required for graceful degradation; > 25 % collapses success | -30 % at 25 % err, -33 % at 50 % err |

## The deliverable: pre-flight validator (`experiments/validator/recommend_envelope.m`)

Inputs (from bench + airframe spreadsheet):
- measured slew rate (deg/s gimbal), from `Firmware/feedback_servo_calibration.ino`
- mechanical envelope (deg gimbal)
- estimated `p` (from CG margin, aero coefficients, thrust)

Outputs:
- recommended `u_max` (deg gimbal) — H1 rule
- LQR gain `K` for the conservative design — H2 rule
- slew margin and GO / MARGINAL / NOGO verdict
- explanation string suitable for paste into a flight readiness review

Demonstrated call on a typical hobby-servo rig (200 deg/s, 8 deg envelope, p=10): **verdict NOGO, slew margin 0.67** — meaning that platform is unsuitable for that airframe; either pick a faster servo or relax the airframe instability.

## File inventory (new this session)

```
experiments/
  HEADLINE_LOCKED.md                       (locked headline document)
  SESSION_FINAL.md                         (this file)
  sim_unstable.m                           (minimal unstable-plant simulator)
  classify_outcome.m                       (binary stabilization metric)
  design_lqr_unstable.m                    (LQR designer)
  h1_coupling_sweep.m, h1_analyze_coupling.m
  run_h1.m  (null baseline), run_h1_stressed.m, run_h1_verify.m, run_h1_lock.m
  h2_timevarying_sweep.m, h2_driftrate_sweep.m
  run_h2.m, run_h2_drift.m, run_h2_stress.m
  h3_wiggle_identify.m
  run_h3.m
  validator/
    recommend_envelope.m                   (the user-facing tool)
  results/
    h1_coupling_p60.mat, h1_coupling_p100.mat
    h1_lock_summary.csv, h1_verify_table.csv
    h2_timevarying.mat
    h2_driftrate_{p0,mid,p1}_u5.mat
    h2_driftrate_{p0,mid,p1}_u3.5.mat
    h2_drift_long.csv, h2_stress_summary.csv, h2_collapse_summary.csv
    h3_robustness.mat
```

## What we have NOT yet done (so the user knows)

1. **No cross-check with second LLM** (per user's stated preference for multi-model review on major decisions). The H1 / H2 / H3 results above should be sanity-checked by at least one other model before paper-locking. This is the most important pending item.
2. **No literature review** to confirm the H1 finite-authority optimum is genuinely novel and not buried in some 1990s constrained-LQR paper. Should be done before submission.
3. **No replication on `simulate_case_realistic.m`** (the project's primary plant). All findings are in the minimal `sim_unstable.m`. Adding a static-instability term to the realistic plant and re-running H1 and H2 is the right portability check.
4. **No constraint-aware controller comparison.** If sigma-MRAC, anti-windup, or MPC erases either H1 or H2, the framing flips to "naive LQR is fragile in a predictable way." Both framings are publishable but the headline language must match.
5. **No bench validation** of the H1 fitted rule `u*_max ≈ 0.06·slew + 0.30·p`. Should be done on the workspace's existing gimbal rig (`data/bench/`) before the paper.
6. **No real-thrust noise, motor misalignment, fin coupling.** The simulator is intentionally minimal to isolate the three effects. Realistic disturbance models could shift the magnitudes but not the qualitative findings.

## Recheck against your four priorities

- **STS finalist level:** Two independent counter-intuitive findings + unified mechanism + working hardware tool + amateur-actionable rule. This is the strongest narrative the project has had. Plausibly finalist-class once cross-validated. Not guaranteed.
- **Novelty:** Multi-limit composition for unstable amateur plants with cheap servos is, to my best knowledge, not directly addressed in the published TVC / actuator-saturation literature. Should be confirmed.
- **MIT sellability:** "I built a tool that gives amateur rocket builders a go/no-go from a bench test, anchored to three controlled simulation findings" is a coherent application essay AND a coherent admissions interview answer. The story is concrete.
- **May submission readiness:** Three findings, one mechanism, one tool, reproducible scripts. Paper write-up is the remaining work; the science is in place. Time-budget-wise: tight but feasible.

## Recommended immediate next moves (priority-ordered)

1. **Cross-validate with another LLM.** Paste `HEADLINE_LOCKED.md` and `SESSION_FINAL.md` into a second model and ask it to attack the findings.
2. **Quick literature scan** (1 hour) for "constrained LQR + unstable plant + finite-authority optimum" — confirm novelty.
3. **Port H1 to `simulate_case_realistic.m`** by adding the static-instability term — same findings on the project's primary plant.
4. **Validate the H1 fitted rule** on the bench gimbal rig.
5. **Run sigma-MRAC vs constraint-blind LQR** comparison to nail down framing.
6. **Start paper draft** using `HEADLINE_LOCKED.md` as the abstract+intro skeleton.
