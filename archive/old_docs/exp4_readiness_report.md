# Exp4 Readiness Report

## Scope
This is an audit-only readiness assessment for the Exp4 thesis question:

Where do simplified simulations stop making correct design decisions?

No experiment code was modified.

## Current Evidence Base
Primary sources reviewed:
- experiments/run_exp34_regime_importance.m
- experiments/results/exp34_factor_toggle_ranges.csv
- experiments/results/exp34_regime_factor_importance.csv
- experiments/results/exp34_regime_top3_frequency.csv
- experiments/results/exp34_interaction_audit.csv
- experiments/results/exp34_interaction_audit_regime_medians.csv

## Current Strengths
- Exp4 already has regime-conditioned factor attribution over many cells (EASY, FRAGILE, INFEASIBLE).
- Full realistic versus idealized assumptions are explicit and auditable.
- Factor toggles are transparent and exported (deadband, backlash, latency, sensor noise bundle, thrust mismatch, aero mismatch, effectiveness drift).
- The current outputs already support a robust physical attribution narrative:
  - Backlash is rank-1 in 100 percent of sampled cells in each regime.
  - Aero mismatch is typically rank-2.
  - Sensor noise and deadband appear as context-dependent secondary effects.
- Interaction audits exist and show non-additivity (combined effects differ from sum of single-factor effects), which is essential for honest sim-to-real interpretation.

## Current Weaknesses
- Exp4 currently optimizes and scores recovery-style metrics, not design-decision correctness.
- The pipeline does not currently compute direct simple-versus-high-fidelity decision disagreement for each regime cell.
- The pipeline does not currently compute boundary displacement (how the EASY/FRAGILE/INFEASIBLE frontier moves between models).
- The pipeline does not currently compute classification error rates for simplified-model decisions relative to high-fidelity decisions.
- Gust is present in baseline realism but not isolated as an Exp4 factor in the current factor list.

## Direct Answer To Required Readiness Checks
- Regime disagreement metric available now: No.
- Boundary displacement metric available now: No.
- Classification error rate metric available now: No.

## Missing Analyses (Minimum Needed)
- Cellwise decision comparison:
  - For each phase-diagram cell, produce decision labels under simplified assumptions and high-fidelity assumptions using the same decision rule.
  - Output confusion matrix and disagreement rate by regime.
- Boundary displacement:
  - Extract decision frontiers from both models and compute per-p instability shifts in required slew and/or authority.
- Error decomposition:
  - Attribute disagreement to mismatch sources by toggling factors and measuring disagreement reduction, not just success recovery.
- Robust uncertainty bands:
  - Add confidence intervals on disagreement rates and frontier shift metrics.

## Exact Next Experiment Needed
Exp4A: Decision-Disagreement Audit (proposal only)

Design:
- Use the frozen Exp1 phase grid as the common design space.
- For each cell, tune once using the designated training model (locked protocol), then evaluate decisions under:
  - simplified model
  - high-fidelity model
- Apply the same regime classifier to both outputs.

Primary outputs:
- cell-level paired decision table
- regime confusion matrix
- disagreement rate by regime
- frontier displacement table and plot

Pass criteria for readiness to claim the thesis statement:
- Quantified disagreement rates with confidence intervals.
- Quantified boundary displacement with confidence intervals.
- Factor-attributed disagreement reduction showing which realism terms fix wrong decisions.

## Bottom-Line Readiness Verdict
Exp4 is ready for mismatch attribution claims, but not yet ready for the core decision-correctness claim.

Current status:
- Ready: Which mismatch sources matter by regime.
- Not yet ready: Where simplified simulation makes wrong design decisions.

The missing piece is a direct paired decision-disagreement analysis, not more generic recovery metrics.