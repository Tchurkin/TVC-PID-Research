# SIMULATION_CREDIBILITY_REPORT

## Adversarial Verdict
- Status: CONDITIONAL CREDIBILITY
- Interpretation: core trends are present, but multiple claims are vulnerable to artifacts.

## Key Risks
- Sampled but not runtime-wired parameters: mass, Iyy, static_margin, Cm_alpha, thrust.
- Backlash baseline balanced accuracy: 0.6223; worst perturbation: hold_backlash_constant (-0.2585).
- Max regime flip rate under threshold perturbation: 0.018 at -5pct.
- Repeatability mean pairwise JSD: 0.0010; mean feature-rank Spearman: 0.6630.
- Exp4 disagreement rate vs highest-fidelity reference: 0.560.
- Exp5 winner retention under perturbation: 1.000.

## Claim Tightening
- Treat unwired sampled parameters as metadata unless connected into runtime state update.
- Always publish threshold-sensitivity and seed-repeatability with regime claims.
- Present Exp4/Exp5 decisions as confidence-banded, not deterministic.