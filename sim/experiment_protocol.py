"""
experiment_protocol.py — Canonical experiment specification for the TVC stability study.

This module is the single source of truth for ALL experiment design decisions.
Nothing in this module may be changed without updating the corresponding STS section.

Frozen as of 2026-06-02.  All downstream analysis code imports from here.

Research question answered by this protocol
────────────────────────────────────────────
"What is the probabilistic stability frontier of low-cost TVC rockets in
actuator-limited, disturbance-driven regimes?"

Reproducibility guarantee
─────────────────────────
Given the same SEED_PRIMARY and N_DESIGNS_EXP1, the full pipeline is
deterministic.  No analysis step introduces additional randomness beyond
bootstrap resampling, which is also seeded (SEED_BOOTSTRAP_BASE).
"""

from __future__ import annotations

# ── Sample size ───────────────────────────────────────────────────────────────

N_DESIGNS_EXP1 = 1200
# Statistical power rationale: with N=1200 and expected INFEASIBLE ~20%,
# we have ~240 INFEASIBLE designs — sufficient for logistic boundary estimation
# with 12 predictors (20:1 events-per-variable rule).

N_DESIGNS_MIN = 400
# Minimum acceptable sample for a valid run (50 per regime is the hard floor
# for any regime-stratified analysis).

# ── Seeds ─────────────────────────────────────────────────────────────────────

SEED_PRIMARY = 42
# Controls LHS sampling of the design space (reproducible design matrix).

SEED_BOOTSTRAP_BASE = 100
# Base seed for bootstrap resampling in analysis layer.
# Bootstrap trial k uses seed = SEED_BOOTSTRAP_BASE + k.

SEED_SIMULATION = 1
# Single seed used for all simulator evaluations in Exp1.
# Rationale: autotuning already averages over gain perturbation (0.6×, 1.0×, 1.4×),
# providing three quasi-independent outcome samples per design.  A multi-seed
# re-evaluation is deferred to a future sensitivity study.

# ── Repeats per design ────────────────────────────────────────────────────────

N_GAIN_SETS_PER_DESIGN = 3
# {nominal, under (×0.60), over (×1.40)}.  Robustness = fraction of gain sets
# that pass the FRAGILE success threshold.  This is NOT a Monte-Carlo estimate
# of P(success | design) — it is a gain-robustness score.

N_BOOTSTRAP = 500
# Bootstrap iterations for confidence interval estimation in analysis layer.

# ── Metrics recorded per simulation run ──────────────────────────────────────

RUN_METRICS = [
    "success",            # bool — all four success gates satisfied
    "rms_error_deg",      # float — RMS tracking error over full burn (deg)
    "peak_error_deg",     # float — maximum instantaneous tracking error (deg)
    "end_error_deg",      # float — terminal tracking error at t=t_end (deg)
    "max_theta_deg",      # float — maximum absolute pitch angle (deg)
    "u_cmd_sat_frac",     # float — fraction of steps where PID output was clipped
    "slew_sat_frac",      # float — fraction of steps where slew limit was active
    "settling_time_s",    # float — time to first enter and stay within ±5 deg (s)
    "oscillation_score",  # float — signed zero-crossing count of tracking error
]

# ── Success definition (four gates, all must be satisfied simultaneously) ─────

SUCCESS_GATES = {
    "stability":  ("max_theta_deg",   "<",  70.0),  # no structural / recovery failure
    "tracking":   ("rms_error_deg",   "<",  15.0),  # acceptable RMS over full burn
    "terminal":   ("end_error_deg",   "<",  15.0),  # acceptable end-state accuracy
    "peak":       ("peak_error_deg",  "<",  50.0),  # tolerable peak transient
}
# Gate decomposition enables failure-mode attribution:
#   stability failure only  → divergence / instability
#   tracking failure only   → steady-state disturbance rejection
#   terminal failure only   → gyro drift / sensor bias accumulation
#   peak failure only       → large transient (launch perturbation, fault)
#   multiple failures       → compounded / cascade failure

# ── Regime classification ─────────────────────────────────────────────────────

REGIME_DEFINITIONS = {
    "EASY": {
        "code": 2,
        "description": (
            "Controller robustness = 1.0 (all three gain sets pass FRAGILE threshold) "
            "plus quality gates: nominal_success_rate >= 0.80, rms <= 8 deg, "
            "settling <= 3 s, oscillation_score <= 3, u_sat_frac <= 0.60."
        ),
    },
    "FRAGILE": {
        "code": 1,
        "description": (
            "Robustness > 0 (at least one gain set passes) plus minimal quality: "
            "nominal_success_rate >= 0.35, rms <= 16 deg.  "
            "Margins are slim — gains must be well-tuned."
        ),
    },
    "INFEASIBLE": {
        "code": 0,
        "description": (
            "No tested gain set achieves FRAGILE-threshold success, OR quality "
            "gates for FRAGILE fail.  Physics prevents stable TVC control with "
            "the available actuator authority."
        ),
    },
}

# ── Fidelity ladder (Exp4) ────────────────────────────────────────────────────

FIDELITY_PROTOCOL = {
    "tune_at": "L5_FULL",
    "evaluate_at": ["L0_BASELINE", "L1_SATURATION", "L2_SLEW_LIMIT",
                    "L3_ACTUATOR_NONLINEAR", "L4_SENSOR_EFFECTS", "L5_FULL"],
    "rationale": (
        "Gains tuned once at highest fidelity, then frozen.  Evaluating at lower "
        "fidelity without re-tuning isolates the DECISION IMPACT of each omitted "
        "physics layer.  Re-tuning at each level (as in the MATLAB baseline) "
        "measures controller adaptability, not simulator fidelity necessity."
    ),
}

# ── Probabilistic output contract ─────────────────────────────────────────────

PROBABILISTIC_OUTPUTS = {
    "p_success":       "P(regime in {EASY, FRAGILE}) — RF-predicted probability of at least partial success",
    "p_easy":          "P(regime = EASY) — RF-predicted probability of full robustness",
    "p_infeasible":    "P(regime = INFEASIBLE) — RF-predicted probability of divergence",
    "expected_rms":    "E[rms_error_deg | design features] — RF regressor prediction",
    "p_success_lo95":  "2.5th bootstrap percentile of p_success estimate",
    "p_success_hi95":  "97.5th bootstrap percentile of p_success estimate",
}
# These are MODEL PREDICTIONS from fitted classifiers/regressors, not empirical
# multi-seed simulation estimates.  They should be interpreted as
# "what fraction of designs with similar features succeeded in the training data?"

# ── Design features used for model training ───────────────────────────────────

DESIGN_FEATURES = [
    "p_unstable",             # proxy open-loop instability rate (1/s)
    "control_effectiveness",  # keff (rad/s² / code unit)
    "servo_slew_deg_s",       # max gimbal angular velocity (physical deg/s)
    "max_gimbal_deg",         # gimbal travel limit (deg)
    "deadband",               # stiction threshold (code units)
    "backlash",               # mechanical play (code units)
    "latency_steps",          # sensor pipeline latency (integer steps × 5 ms)
    "wind_strength",          # gust disturbance standard deviation
    "mass",                   # vehicle mass (kg)
    "Iyy",                    # pitch moment of inertia (kg·m²)
    "thrust",                 # mean thrust (N)
]

# ── Output artifact paths (relative to project root) ─────────────────────────

ARTIFACT_PATHS = {
    "raw_runs":     "results/raw_runs.parquet",
    "regime_model": "results/regime_model.pkl",
    "frontier_map": "results/frontier_map.parquet",
    "sensitivity":  "results/sensitivity.json",
}
