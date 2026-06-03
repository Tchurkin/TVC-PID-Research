# PHYSICS_WIRING_VERIFICATION

- Status: PASS
- Interpretation: all five physical parameters moved the runtime derivative with the expected sign and a non-negligible magnitude.

## Verification Protocol
- Simulator: `simulate_case_realistic` with deterministic seed 1.
- Probe horizon: one runtime step (`dt = 0.0050 s`).
- Controller: zero-gain PID to isolate the plant derivative.
- Acceptance thresholds are defined once in `verification_constants` inside `experiments/verify_physics_wiring.m`.

## Results

| parameter | context | expected | low | ref | high | low resp | ref resp | high resp | overall | notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|
| mass | disturbance_response_qdot | descending | 0.84 | 1.2 | 1.56 | 1.42857 | 1 | 0.769231 | PASS | PASS |
| Iyy | control_authority_qdot | descending | 0.0126 | 0.018 | 0.0234 | 11.4286 | 8 | 6.15385 | PASS | PASS |
| static_margin | geometry_qdot | ascending | 0.05 | 0.1 | 0.15 | 0.25 | 1 | 2.25 | PASS | PASS |
| Cm_alpha | geometry_qdot | descending | -78 | -52 | -26 | 2.25 | 1 | 0.25 | PASS | PASS |
| thrust | control_authority_qdot | ascending | 24.5 | 35 | 45.5 | 5.6 | 8 | 10.4 | PASS | PASS |

## Geometry Path Finding
- `static_margin` and `Cm_alpha` are swept WITHOUT pinning `p_unstable`, so they drive the unstable pole through the real derivation path used by Exp1 (`p_unstable = estimate_p_unstable(static_margin, Cm_alpha, thrust)`).
- The redundant `p_geom`/`p_geom_delta` channel (formerly computed by the identical formula as `p_unstable`, double-counting geometry) has been removed, so geometry now enters the derivative exactly once.

