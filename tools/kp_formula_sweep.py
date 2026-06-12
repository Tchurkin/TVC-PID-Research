"""
kp_formula_sweep.py

Two experiments to test whether Kp_required has a closed-form derivation:

SWEEP A: kp_required surface over (keff_full, wind_strength)
  Fix everything except Iyy (to control keff) and wind_strength.
  Find kp_floor and kp_optimal at each grid point.
  Test: does log(kp_floor) = a + b*log(keff) + c*log(wind)?
  Physics prediction (overshoot):  kp_floor ∝ keff^+1  (high keff → more overshoot → need more Kp)
  Physics prediction (wind):       kp_floor ∝ wind^+1 / keff  (limit cycle from wind)
  These have OPPOSITE signs for keff — the surface shape reveals which dominates.

SWEEP B: kp_ceiling surface over (keff_full, servo_slew)
  Find kp_ceiling (max Kp before oscillation/SR degrades) at each grid point.
  Test: kp_ceiling ∝ (slew / keff)^α  (bang-bang chatter threshold)
  Physical derivation: servo can't keep up at high Kp × keff → oscillation → SR drops.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))

import numpy as np
import pandas as pd

from design_space import REF, build_plant, build_actuator, build_sensor, build_disturbance
from simulator import simulate, ScenarioParams
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX  # 0.02182 rad/CU
L_NOZZLE  = 0.25  # m, fixed for all designs

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)

EVAL_SEEDS  = list(range(1, 9))
KD_FIXED    = 8
SR_FLOOR    = 0.50  # threshold for defining kp_floor and kp_ceiling
SR_OPTIMAL  = 0.75  # threshold for kp_optimal

# ── Sweep A: kp_required surface ──────────────────────────────────────────────

KEFF_TARGETS = [2.0, 4.0, 8.0, 14.0, 20.0]
WIND_LEVELS  = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]
KP_GRID_A    = [1, 2, 5, 10, 20, 40, 80, 160]

def make_design_keff(keff_target: float, wind: float, slew: float = 120.0) -> dict:
    """Build REF design with Iyy set to hit keff_target and wind/slew overridden."""
    d = dict(REF)
    d["Iyy"]             = F15_AVG_THRUST_N * d["motor_scale"] * CU_TO_RAD * L_NOZZLE / keff_target
    d["wind_strength"]   = wind
    d["servo_slew_deg_s"] = slew
    d["static_margin"]   = 0.0   # neutral stability — isolates gain mechanics
    d["lambda_aero"]     = 0.0
    return d


def eval_kp(design: dict, Kp: float, Kd: float) -> tuple[float, float]:
    """Return (mean_sr, mean_rms) over EVAL_SEEDS."""
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = ScenarioParams(t_end=3.0, theta_ref=0.0)
    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    results = [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in EVAL_SEEDS]
    sr  = sum(r.success for r in results) / len(results)
    rms = sum(r.rms_error_deg for r in results) / len(results)
    return sr, rms


def find_kp_metrics(kp_srs: list[tuple]) -> dict:
    """
    Given [(Kp, sr), ...] sorted by Kp, return floor, optimal, ceiling.
    kp_floor:   lowest Kp where SR >= SR_FLOOR
    kp_optimal: Kp with highest SR (first if tie)
    kp_ceiling: highest Kp before SR drops >= 0.2 below peak SR (None if no rolloff observed)
    """
    if not kp_srs:
        return {"kp_floor": None, "kp_optimal": None, "kp_ceiling": None, "sr_peak": 0.0}
    srs   = [sr for _, sr in kp_srs]
    kps   = [kp for kp, _ in kp_srs]
    sr_peak = max(srs)
    kp_floor = next((kp for kp, sr in kp_srs if sr >= SR_FLOOR), None)
    kp_optimal = kps[srs.index(sr_peak)]
    # ceiling: last Kp where SR >= peak - 0.20, before final drop
    kp_ceiling = None
    for kp, sr in reversed(kp_srs):
        if sr >= sr_peak - 0.20:
            kp_ceiling = kp
            break
    return {"kp_floor": kp_floor, "kp_optimal": kp_optimal,
            "kp_ceiling": kp_ceiling, "sr_peak": sr_peak}


print("=== SWEEP A: kp_required surface (keff × wind) ===")
rows_a = []
total_a = len(KEFF_TARGETS) * len(WIND_LEVELS)
done_a = 0

for keff in KEFF_TARGETS:
    for wind in WIND_LEVELS:
        done_a += 1
        design = make_design_keff(keff, wind)
        kp_srs = []
        for Kp in KP_GRID_A:
            sr, rms = eval_kp(design, Kp, KD_FIXED)
            kp_srs.append((Kp, sr))
            rows_a.append({"keff_target": keff, "wind_strength": wind,
                           "Kp": Kp, "sr": sr, "rms": rms})
        m = find_kp_metrics(kp_srs)
        print(f"  [{done_a:3d}/{total_a}] keff={keff:.0f} wind={wind:.2f} "
              f"floor={m['kp_floor']} optimal={m['kp_optimal']} "
              f"ceil={m['kp_ceiling']} peak_sr={m['sr_peak']:.2f}")

df_a = pd.DataFrame(rows_a)
out_a = os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "kp_surface_py.csv")
df_a.to_csv(out_a, index=False)
print(f"\nSaved {len(df_a)} rows -> {out_a}\n")


# ── Sweep B: kp_ceiling vs servo_slew ─────────────────────────────────────────

KEFF_TARGETS_B = [4.0, 8.0, 14.0, 20.0]
SLEW_LEVELS    = [60, 80, 120, 160, 200, 300]
WIND_FIXED     = 0.25
KP_GRID_B      = [2, 5, 10, 20, 40, 80, 160, 320]

print("=== SWEEP B: kp_ceiling vs servo_slew ===")
rows_b = []
total_b = len(KEFF_TARGETS_B) * len(SLEW_LEVELS)
done_b = 0

for keff in KEFF_TARGETS_B:
    for slew in SLEW_LEVELS:
        done_b += 1
        design = make_design_keff(keff, WIND_FIXED, slew)
        kp_srs = []
        for Kp in KP_GRID_B:
            sr, rms = eval_kp(design, Kp, KD_FIXED)
            kp_srs.append((Kp, sr))
            rows_b.append({"keff_target": keff, "servo_slew": slew, "wind_strength": WIND_FIXED,
                           "Kp": Kp, "sr": sr, "rms": rms})
        m = find_kp_metrics(kp_srs)
        print(f"  [{done_b:3d}/{total_b}] keff={keff:.0f} slew={slew:3d} "
              f"floor={m['kp_floor']} optimal={m['kp_optimal']} "
              f"ceil={m['kp_ceiling']} peak_sr={m['sr_peak']:.2f}")

df_b = pd.DataFrame(rows_b)
out_b = os.path.join(os.path.dirname(__file__), "..", "experiments", "results", "servo_spec_py.csv")
df_b.to_csv(out_b, index=False)
print(f"\nSaved {len(df_b)} rows -> {out_b}")
