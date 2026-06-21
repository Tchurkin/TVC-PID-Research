"""
tools/gain_advisor.py

A genuinely usable builder tool: given hardware specifications alone (no
simulation, no tuning, no flight test), recommend either a PID gain range or
an ADRC bandwidth, plus a continuous risk score -- not a binary GO/NOGO.

Every formula used here is sourced from a specific experiment in this
project, listed in PROVENANCE below, so a skeptical reader can check exactly
what evidence backs each number and how strong that evidence is.

Usage (library):
    from gain_advisor import recommend
    rec = recommend(thrust_n=14.4, max_gimbal_deg=10.0, l_nozzle_m=0.25,
                     Iyy_kgm2=0.015, latency_steps=4)
    print(rec.summary())

Usage (CLI):
    python tools/gain_advisor.py --thrust 14.4 --gimbal 10 --l_nozzle 0.25 \
        --iyy 0.015 --latency 4
"""

from __future__ import annotations
import argparse
import numpy as np
from dataclasses import dataclass

# ── Constants (units.py) ────────────────────────────────────────────────────
REF_MAX_GIMBAL_DEG = 15.0
REF_U_MAX = 12.0
CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX  # rad per code-unit, 0.02182

# ── PROVENANCE ───────────────────────────────────────────────────────────────
# theta_ddot_max formula:      Newton's 2nd law, no fitting (exact).
# PID Kp ceiling (380/latency): DIPDT theory + empirical (tools/gain_window_boundary_v2.py
#     R^2=0.53 n=20; tools/window_ratio_regression.py v2 n=93 uncensored).
#     KEY FINDING (2026-06-17): ceiling is INDEPENDENT OF KEFF (keff coeff ≈ 0.005 ≈ 0).
#     Empirical latency exponent -0.86 (population regression); -1.0 from DIPDT theory.
#     380/latency is a reasonable approximate formula (DIPDT + 2.1x bang-bang correction).
# PID Kp floor (UPDATED 2026-06-17, tools/window_ratio_resweep_v2.py n=104 non-censored):
#     NEW FORMULA: 0.06 * keff^1.06 * latency^0.96  (R^2=0.627, CV R^2=0.594)
#     OLD formula (0.35 * keff^0.70 from relay study, keff-only, rho=0.58) was INCOMPLETE:
#     latency was not tested. New finding: floor rises with latency as strongly as with keff
#     (both exponents ≈ +1.0). Mechanism: high latency → angular error accumulates for τ sec
#     before correction acts → need higher Kp to keep SR>=0.80. The WINDOW-RATIO FORMULA
#     NEGATIVE RESULT (AUC=0.500 on held-out data, CLAUDE.md) tested the old keff-only floor;
#     the updated formula with latency has not yet been externally validated.
# Continuous margin model (over_sr ~ log td + log lat): tools/continuous_margin_
#     regression.py + tools/elbow_characterization.py, n=222, R^2=0.274, p=5.8e-11.
#     NOTE: the window_ratio regression (v2, 2026-06-17) gives CV R^2=0.616 with the same
#     predictors but a better-specified outcome (direct window width, not over_sr). The
#     over_sr model is retained here as the margin score since it's in [0,1] and interpretable.
# ADRC bandwidth ceiling (omega_c_max ~ 645/(lat^0.66 * td^0.77)):
#     tools/adrc_bandwidth_ceiling.py, n=21 valid cells (3 authority levels x
#     up to 8 latencies), R^2=0.936. Smaller sample than the PID formulas;
#     treat as a first-pass result pending a larger sweep.

L_NOZZLE_DEFAULT = 0.25  # m, fixed airframe assumption used throughout this project


@dataclass
class Recommendation:
    theta_ddot_max: float
    keff_full: float
    latency_steps: int
    predicted_margin: float       # continuous, [0,1]-ish; over_sr regression
    predicted_window_ratio: float # predicted Kp ceiling/floor ratio; window regression
    pid_kp_floor: float
    pid_kp_ceiling: float
    pid_window_ratio: float       # from floor/ceiling formulas directly
    adrc_omega_c_max: float
    risk_tier: str

    def summary(self) -> str:
        extrapolation_warn = (
            "\n  WARNING: keff_full > 20 -- floor formula extrapolates beyond training data;"
            " window ratio likely OVERPREDICTED (actual risk higher than shown)"
            if self.keff_full > 20 else ""
        )
        lines = [
            f"theta_ddot_max = {self.theta_ddot_max:.1f} rad/s^2  "
            f"(keff_full = {self.keff_full:.2f} rad/s^2 per code-unit)",
            f"latency_steps = {self.latency_steps}",
            "",
            f"Predicted gain-margin score: {self.predicted_margin:.2f} (1.0=no risk; over_sr model, CV R2=0.325)",
            f"Predicted window ratio: {self.predicted_window_ratio:.1f}x ceiling/floor "
            f"(window regression, CV R2=0.616){extrapolation_warn}",
            f"Risk tier: {self.risk_tier}",
            "",
            "If using PID:",
            f"  Recommended Kp range: [{self.pid_kp_floor:.1f}, {self.pid_kp_ceiling:.1f}]"
            f"  (formula-predicted window ratio {self.pid_window_ratio:.1f}x)",
            "  Tune in a full-physics simulator with wind, not a disturbance-free one --",
            "  disturbance-free tuning has a documented ~64% false-rejection rate for",
            "  designs in this risk range, and (rarely) 100% false approval for the most",
            "  extreme designs (see CLAUDE.md Finding 4 / paper Section 5.1).",
            "",
            "If using ADRC (Active Disturbance Rejection Control):",
            f"  Recommended omega_c <= {self.adrc_omega_c_max:.1f} rad/s "
            f"(omega0 ~ 5x omega_c as a starting point)",
            "  ADRC closes most of the PID gap for high-risk designs (Section 6); if the",
            "  PID window above looks dangerously narrow, ADRC is the more robust choice.",
        ]
        return "\n".join(lines)


def recommend(
    thrust_n: float,
    max_gimbal_deg: float,
    Iyy_kgm2: float,
    latency_steps: int,
    l_nozzle_m: float = L_NOZZLE_DEFAULT,
) -> Recommendation:
    keff_full = thrust_n * CU_TO_RAD * l_nozzle_m / Iyy_kgm2
    u_max = max_gimbal_deg * REF_U_MAX / REF_MAX_GIMBAL_DEG
    theta_ddot_max = keff_full * u_max

    # Continuous margin model: over_sr ~ a + b*log(td) + c*log(latency)
    # Fit on n=222, R^2=0.274 (tools/continuous_margin_regression.py +
    # tools/elbow_characterization.py). Clipped to [0,1] -- the linear fit can
    # extrapolate outside that range for extreme inputs.
    a, b, c = 1.168, -0.0396, -0.0720
    margin = a + b * np.log(theta_ddot_max) + c * np.log(latency_steps)
    margin = float(np.clip(margin, 0.0, 1.0))

    pid_kp_ceiling = 380.0 / latency_steps
    # Updated 2026-06-17: floor depends on BOTH keff AND latency (R^2=0.627, CV=0.594).
    # Old formula (0.35*keff^0.70, keff-only) was underpowered — latency simply was never tested.
    pid_kp_floor   = 0.06 * keff_full ** 1.061 * latency_steps ** 0.964
    window_ratio   = pid_kp_ceiling / pid_kp_floor if pid_kp_floor > 0 else float("inf")

    # Predicted window ratio from population regression (2026-06-17, tools/window_ratio_resweep_v2.py):
    # log(window) ~ 9.07 - 1.19*log(keff) - 1.88*log(lat)  [n=116, R^2=0.659]
    # CAVEAT: floor formula (0.06*keff^1.06*lat^0.96) was fit on keff < ~20.
    # For extreme designs (keff > 20, td > 200), the floor is observed to be 5-10x larger
    # than the formula predicts (e.g. td=250 design: formula gives floor=8 at lat=4, measured floor=77).
    # The window ratio will be significantly overpredicted (risk understated) for such designs.
    predicted_window_ratio = 8700.0 * keff_full ** (-1.19) * latency_steps ** (-1.88)

    # ADRC bandwidth ceiling: omega_c_max ~ A / (latency^p * td^q)
    # tools/adrc_bandwidth_ceiling.py, n=21, R^2=0.936.
    A, p, q = 645.0, 0.66, 0.77
    adrc_omega_c_max = A / (latency_steps ** p * theta_ddot_max ** q)

    if theta_ddot_max < 36:
        tier = "LOW RISK -- below the smallest theta_ddot_max ever observed to need careful tuning in this study"
    elif theta_ddot_max < 55:
        tier = "LOW-MODERATE -- tune with wind in the simulator if latency_steps >= 5"
    elif theta_ddot_max < 90:
        tier = "MODERATE -- gain margin starts to narrow; full-physics tuning recommended"
    elif theta_ddot_max < 150:
        tier = "ELEVATED -- gain margin measurably narrower; consider ADRC or careful PID tuning + flight test at Kp=2"
    else:
        tier = "HIGH -- gain margin substantially narrowed in this study's population; ADRC strongly recommended, or extensive full-physics PID tuning"

    return Recommendation(
        theta_ddot_max=theta_ddot_max,
        keff_full=keff_full,
        latency_steps=latency_steps,
        predicted_margin=margin,
        predicted_window_ratio=predicted_window_ratio,
        pid_kp_floor=pid_kp_floor,
        pid_kp_ceiling=pid_kp_ceiling,
        pid_window_ratio=window_ratio,
        adrc_omega_c_max=adrc_omega_c_max,
        risk_tier=tier,
    )


def main():
    ap = argparse.ArgumentParser(description="TVC gain advisor: predict gain sensitivity from hardware specs alone.")
    ap.add_argument("--thrust", type=float, required=True, help="Average motor thrust, N")
    ap.add_argument("--gimbal", type=float, required=True, help="Max gimbal deflection, deg")
    ap.add_argument("--iyy", type=float, required=True, help="Pitch moment of inertia, kg*m^2")
    ap.add_argument("--latency", type=int, required=True, help="Control loop latency, steps (5ms each at 200Hz)")
    ap.add_argument("--l_nozzle", type=float, default=L_NOZZLE_DEFAULT, help="Nozzle-to-CG moment arm, m")
    args = ap.parse_args()

    rec = recommend(
        thrust_n=args.thrust, max_gimbal_deg=args.gimbal, Iyy_kgm2=args.iyy,
        latency_steps=args.latency, l_nozzle_m=args.l_nozzle,
    )
    print(rec.summary())


if __name__ == "__main__":
    main()
