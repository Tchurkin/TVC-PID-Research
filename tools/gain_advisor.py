"""
tools/gain_advisor.py

Builder tool: given hardware specifications, predict gain window risk and
recommend a PID Kp range and ADRC bandwidth ceiling — all in physical units.

Physical units throughout (no simulator-internal "control unit" conversions exposed):
  keff  = T_avg × L_nozzle / Iyy   [s⁻²]   rotational authority per radian of gimbal
  τ     = total loop delay           [s]       measurement-to-actuation latency
  Π     = keff × τ²                 [-]       dimensionless risk parameter (saturation onset)

Usage (CLI — delay in seconds):
    python tools/gain_advisor.py --thrust 14.4 --l_nozzle 0.25 --iyy 0.015 --tau 0.025

Usage (CLI — delay as latency steps):
    python tools/gain_advisor.py --thrust 14.4 --l_nozzle 0.25 --iyy 0.015 --lat 5 --loop_hz 200

Usage (CLI — include gimbal to enable margin score and ADRC ceiling):
    python tools/gain_advisor.py --thrust 14.4 --l_nozzle 0.25 --iyy 0.015 --tau 0.025 --gimbal 10

Usage (library):
    from gain_advisor import recommend
    rec = recommend(thrust_n=14.4, l_nozzle_m=0.25, Iyy_kgm2=0.015, tau_s=0.025)
    print(rec.summary())
"""

from __future__ import annotations
import argparse
import numpy as np
from dataclasses import dataclass

# ── Internal simulator constants — not exposed to users ──────────────────────
# Needed only for converting fit coefficients (derived in the simulator's internal
# CU unit system) into physical-unit output formulas. Builders never need these.
_CU_TO_RAD = np.pi / 180.0 * 15.0 / 12.0  # 0.02182 rad per code-unit
_DT_SIM    = 0.005                           # simulator timestep [s] = 200 Hz loop

# ── Π_crit thresholds (dimensionless) ────────────────────────────────────────
# Source: tools/delay_model_robustness_test.py (n=10, 4 delay-model types).
# Π_crit_physical = Π_crit_CU × _DT_SIM² / _CU_TO_RAD
# bare_metal = 177: integer FIFO; most bare-metal TVC builds sit here
# rtos       = 275: RTOS with ±1-step scheduling jitter
# partial_iir= 407: half-time-constant IIR + remaining FIFO buffer
# filtered   = inf: Madgwick / Mahony / complementary filter eliminates saturation
_PI_CRIT = {
    "bare_metal":  177  * _DT_SIM**2 / _CU_TO_RAD,  # ≈ 0.203
    "rtos":        275  * _DT_SIM**2 / _CU_TO_RAD,  # ≈ 0.315
    "partial_iir": 407  * _DT_SIM**2 / _CU_TO_RAD,  # ≈ 0.466
    "filtered":    float("inf"),
}

# ── Kp ceiling constant ───────────────────────────────────────────────────────
# Kp_ceiling [rad/rad, dimensionless] ≈ _KP_CEIL_CONST / τ [s]
# Derivation: K_u_CU ≈ 380/lat (DIPDT + 2.1× bang-bang correction)
#   → K_u_physical = K_u_CU × _CU_TO_RAD = 380 × _CU_TO_RAD × _DT_SIM / τ_s
_KP_CEIL_CONST = 380.0 * _CU_TO_RAD * _DT_SIM           # ≈ 0.04146 [s]
_KP_CEIL_CONST_CONSERVATIVE = 0.9 * _KP_CEIL_CONST       # ≈ 0.03731 [s]

# ── PROVENANCE ────────────────────────────────────────────────────────────────
# keff = T × L / Iyy:
#   Newton's 2nd law (exact). Units: s⁻².
#
# Π = keff × τ²:
#   From bang-bang blind-spot impulse ∝ keff × τ². Validated by: (a) saturation
#   regime map (n=29, ρ=0.808 vs theory, p=2e-7); (b) performance frontier (n=63,
#   ADRC advantage first appears at Π≈0.37, 17% above saturation onset); (c) causal
#   2×2 factorial (PID-nosat SR=1.000 for all 15 designs; ADRC slew_frac=0.000).
#
# Kp_ceiling [rad/rad] ≈ 0.042/τ:
#   DIPDT phase-margin analysis + 2.1× empirical bang-bang correction (keff-independent
#   at moderate Π; keff coefficient ≈ 0.005 ≈ 0 in window_ratio regression, n=93).
#   5 direct spot-check simulations all correct. Conservative uses 90% of this.
#   Held-out validation: median pred/meas = 0.55× (underpredicts; safe direction).
#
# Kp_floor [rad/rad]:
#   Converted from: 0.06 × keff_CU^1.061 × lat^0.964 (window_ratio v2, n=104,
#   CV R²=0.594). CAVEAT: measured with frozen Kd=1.0 — may overstate the floor.
#   keff > 20 CU (~keff > 915 s⁻²): add 1.5× safety margin (formula underpredicts here).
#
# Continuous margin model:
#   over_sr ≈ 0.787 - 0.040×log(θ̈_max) - 0.072×log(τ) (n=262, CV R²=0.325, p=5.8e-11).
#   Requires max_gimbal_deg (optional parameter).
#
# ADRC ceiling:
#   ωc_max ≈ 3.41/(τ^0.57 × θ̈_max^0.31) [rad/s] (n=46, R²=0.823; exploratory).
#   Requires max_gimbal_deg (optional parameter).
#
# Kd starting point:
#   Population median Kd_best = 1.0 in the simulator's internal units;
#   range observed: [1.0, 4.0]. The Z-N formula gives Kd = 0.57 (lower bound);
#   1.0 is a better default. Increase if oscillation persists.


@dataclass
class Recommendation:
    keff: float                           # [s⁻²]
    tau_s: float                          # [s]
    pi_val: float                         # Π = keff × τ² [-]
    pi_crit: float                        # firmware-specific threshold [-]
    firmware_type: str
    theta_ddot_max: float | None          # [rad/s²]; None if max_gimbal not given
    predicted_margin: float | None        # [0, 1]; None if no theta_ddot
    pid_kp_ceiling: float                 # [rad/rad]
    pid_kp_ceiling_conservative: float    # [rad/rad]
    pid_kp_floor: float | None            # [rad/rad]
    pid_window_ratio: float | None        # [-]
    adrc_omega_c_max: float | None        # [rad/s]

    @property
    def risk_tier(self) -> str:
        p = self.pi_val
        if p < 0.10:
            return "NEGLIGIBLE — linear regime, no saturation risk at any realistic Kp"
        elif p < _PI_CRIT["bare_metal"]:
            return "LOW — linear regime for all MCU types; ceiling cap sufficient"
        elif p < _PI_CRIT["rtos"]:
            return "MODERATE — saturation onset for bare-metal MCUs; tune with wind"
        elif p < _PI_CRIT["partial_iir"]:
            return "ELEVATED — saturation regime for most MCU types; full-physics tuning required"
        elif p < 1.0:
            return "HIGH — deep saturation; ADRC or rate filter strongly recommended"
        else:
            return "CRITICAL — Π > 1.0; extreme saturation regime; ADRC with adaptive ω₀/ωc required"

    def summary(self) -> str:
        above = self.pi_val > self.pi_crit
        regime = "ABOVE saturation onset" if above else "below saturation onset"
        cal_fail = "~61%" if self.pi_val >= 0.40 else ("~58%" if self.pi_val >= 0.32 else "~9%")
        high_keff_warn = (self.keff * _CU_TO_RAD) > 20

        lines = [
            "── TVC Gain Advisor ────────────────────────────────────────────",
            f"keff  = T × L / Iyy = {self.keff:.1f} s⁻²",
            f"τ     = loop delay  = {self.tau_s * 1000:.1f} ms",
            f"Π     = keff × τ²  = {self.pi_val:.4f}  ({regime}; Π_crit = {self.pi_crit:.3f} for {self.firmware_type})",
            f"Risk:   {self.risk_tier}",
            "",
        ]

        if self.theta_ddot_max is not None:
            lines.append(f"θ̈_max (at max gimbal) = {self.theta_ddot_max:.1f} rad/s²")
        if self.predicted_margin is not None:
            lines.append(
                f"Gain-margin score:  {self.predicted_margin:.2f}  "
                "(1.0=no risk; n=262 regression, CV R²=0.33)"
            )

        lines += [
            "",
            "── PID ─────────────────────────────────────────────────────────",
            f"  Kp ceiling:       < {self.pid_kp_ceiling:.4f} rad/rad  "
            f"(= {self.pid_kp_ceiling * 180 / np.pi:.3f} °/°)",
            f"  Conservative:     < {self.pid_kp_ceiling_conservative:.4f} rad/rad",
        ]

        if self.pid_kp_floor is not None:
            lines += [
                f"  Kp floor (est.):  ≥ {self.pid_kp_floor:.5f} rad/rad  "
                f"(upper bound; may overstate floor)",
                f"  Window ratio:     {self.pid_window_ratio:.0f}×",
            ]
            if high_keff_warn:
                lines.append("  ⚠  Very high authority: floor formula likely underpredicts by ≥ 1.5×.")

        lines += [
            "  Kd starting point: 1.0 (population median; range [1.0, 4.0])",
            f"  Disturbance-free calibration failure rate at this Π: {cal_fail}",
            "  Tune in a full-physics simulator WITH wind, not still air.",
            "",
        ]

        if self.adrc_omega_c_max is not None:
            lines += [
                "── ADRC ────────────────────────────────────────────────────────",
                f"  ωc ceiling:  ≤ {self.adrc_omega_c_max:.1f} rad/s  |  start with ω₀ = 5×ωc",
            ]
            if self.pi_val > _PI_CRIT["rtos"]:
                lines.append(
                    "  ADRC strongly recommended over PID at this Π "
                    "(ESO prevents saturation; PID cannot)."
                )

        lines.append("────────────────────────────────────────────────────────────────")
        return "\n".join(lines)


def recommend(
    thrust_n: float,
    l_nozzle_m: float,
    Iyy_kgm2: float,
    tau_s: float | None = None,
    latency_steps: int | None = None,
    loop_hz: float = 200.0,
    max_gimbal_deg: float | None = None,
    firmware_type: str = "bare_metal",
) -> Recommendation:
    """
    Compute gain window risk from hardware specs.

    Parameters
    ----------
    thrust_n       : average motor thrust [N]
    l_nozzle_m     : nozzle-to-CG moment arm [m]
    Iyy_kgm2       : pitch moment of inertia [kg·m²]
    tau_s          : total loop delay in seconds — preferred; directly measurable
                     (IMU read → filter → PID → servo start-of-motion)
    latency_steps  : loop delay in integer steps; used only when tau_s is None;
                     requires loop_hz to convert
    loop_hz        : control loop frequency [Hz]; default 200
    max_gimbal_deg : maximum physical gimbal travel [°]; optional — enables the
                     continuous margin score and the ADRC bandwidth ceiling recommendation
    firmware_type  : 'bare_metal' | 'rtos' | 'partial_iir' | 'filtered'
                     Determines which Π_crit threshold applies (see PROVENANCE above)
    """
    if tau_s is None:
        if latency_steps is None:
            raise ValueError("Provide either tau_s [s] or latency_steps + loop_hz.")
        tau_s = latency_steps / loop_hz

    lat_equiv = tau_s * loop_hz  # effective latency in steps (for fit formulas)

    # ── Core physical quantities ────────────────────────────────────────────
    keff   = thrust_n * l_nozzle_m / Iyy_kgm2   # [s⁻²]
    pi_val = keff * tau_s**2                      # [-] dimensionless

    pi_crit = _PI_CRIT.get(firmware_type, _PI_CRIT["bare_metal"])

    # ── Gain ceiling (keff-independent) ────────────────────────────────────
    pid_kp_ceiling              = _KP_CEIL_CONST              / tau_s  # [rad/rad]
    pid_kp_ceiling_conservative = _KP_CEIL_CONST_CONSERVATIVE / tau_s  # [rad/rad]

    # ── Gain floor (approximate; frozen-Kd artifact — treat as upper bound) ─
    keff_cu = keff * _CU_TO_RAD                  # [rad/s²/CU] — internal use only
    pid_kp_floor = (
        0.06 * (keff_cu ** 1.061) * (lat_equiv ** 0.964) * _CU_TO_RAD
    )  # [rad/rad]
    if keff_cu > 20:
        pid_kp_floor *= 1.5  # safety margin: formula underpredicts at high authority

    pid_window_ratio = (
        pid_kp_ceiling / pid_kp_floor if pid_kp_floor > 0 else float("inf")
    )

    # ── Optional: theta_ddot_max and models that require it ─────────────────
    theta_ddot_max    = None
    predicted_margin  = None
    adrc_omega_c_max  = None

    if max_gimbal_deg is not None:
        theta_ddot_max = keff * max_gimbal_deg * (np.pi / 180.0)   # [rad/s²]

        # Continuous margin model (n=262, CV R²=0.325) — coefficients in physical units
        predicted_margin = float(np.clip(
            0.787 - 0.0396 * np.log(theta_ddot_max) - 0.0720 * np.log(tau_s),
            0.0, 1.0,
        ))

        # ADRC ceiling (n=46, R²=0.823, exploratory) — coefficients in physical units
        adrc_omega_c_max = 3.41 / (tau_s ** 0.57 * theta_ddot_max ** 0.31)

    return Recommendation(
        keff=keff,
        tau_s=tau_s,
        pi_val=pi_val,
        pi_crit=pi_crit,
        firmware_type=firmware_type,
        theta_ddot_max=theta_ddot_max,
        predicted_margin=predicted_margin,
        pid_kp_ceiling=pid_kp_ceiling,
        pid_kp_ceiling_conservative=pid_kp_ceiling_conservative,
        pid_kp_floor=pid_kp_floor,
        pid_window_ratio=pid_window_ratio,
        adrc_omega_c_max=adrc_omega_c_max,
    )


def main():
    ap = argparse.ArgumentParser(
        description="TVC gain advisor: predict saturation regime and Kp window from hardware specs."
    )
    ap.add_argument("--thrust",   type=float, required=True, help="Average motor thrust [N]")
    ap.add_argument("--l_nozzle", type=float, required=True, help="Nozzle-to-CG moment arm [m]")
    ap.add_argument("--iyy",      type=float, required=True, help="Pitch moment of inertia [kg·m²]")

    delay_grp = ap.add_mutually_exclusive_group(required=True)
    delay_grp.add_argument("--tau", type=float, help="Total loop delay [s]  (e.g. 0.025 for 25 ms)")
    delay_grp.add_argument("--lat", type=int,   help="Loop delay in integer steps (with --loop_hz)")

    ap.add_argument("--loop_hz",  type=float, default=200.0,
                    help="Control loop frequency [Hz]; default 200")
    ap.add_argument("--gimbal",   type=float, default=None,
                    help="Max gimbal travel [°] (optional; enables margin score and ADRC ceiling)")
    ap.add_argument("--firmware", type=str,   default="bare_metal",
                    choices=list(_PI_CRIT.keys()),
                    help="Firmware delay model (bare_metal/rtos/partial_iir/filtered)")
    args = ap.parse_args()

    rec = recommend(
        thrust_n=args.thrust,
        l_nozzle_m=args.l_nozzle,
        Iyy_kgm2=args.iyy,
        tau_s=args.tau,
        latency_steps=args.lat,
        loop_hz=args.loop_hz,
        max_gimbal_deg=args.gimbal,
        firmware_type=args.firmware,
    )
    print(rec.summary())


if __name__ == "__main__":
    main()
