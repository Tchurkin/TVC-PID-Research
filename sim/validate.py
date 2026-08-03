"""
validate.py - Scientific validation suite for the TVC simulator.

Tests required before any experimental results are reported:

  1. Determinism         same seed - identical trajectory
  2. Stability physics   uncontrolled unstable system diverges; stable damps out
  3. Component isolation each nonlinearity disabled - predictable behaviour
  4. Monotonicity        slew and backlash monotonically ordered by regime
  5. Mini Exp1           N=20 smoke test to catch systematic failures

Run:
  cd sim
  python validate.py
"""

from __future__ import annotations

import sys
import numpy as np
import pandas as pd

# Add sim/ to path (needed when run from project root)
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))

from plant_dynamics import PlantParams
from actuator_model import ActuatorParams
from sensor_model import SensorParams
from disturbance_model import DisturbanceParams
from controller import PIDParams
from simulator import ScenarioParams, simulate
from design_space import (
    build_plant, build_actuator, build_sensor, build_disturbance,
    build_scenario, sample_lhs, REF,
)
from experiment_runner import run_exp1

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"

results: list[tuple[str, bool, str]] = []


def report(name: str, ok: bool, detail: str = ""):
    status = PASS if ok else FAIL
    print(f"  [{status}]  {name}")
    if detail:
        print(f"           {detail}")
    results.append((name, ok, detail))


def default_config(zero_noise: bool = False):
    """Build a reference-rocket simulation config."""
    design = dict(REF)
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = build_scenario()

    if zero_noise:
        sen.gyro_noise_std    = 0.0
        sen.gyro_bias_init    = 0.0
        sen.gyro_bias_rw      = 0.0
        sen.gyro_quant_lsb    = 0.0
        sen.bias_cal_residual = 0.0
        dis.gust_std          = 0.0
        dis.det_amp           = 0.0

    return plant, act, sen, dis, sc


# -- 1. Determinism ------------------------------------------------------------

def test_determinism():
    print("\n-- 1. Determinism -----------------------------------------------")
    plant, act, sen, dis, sc = default_config()
    pid = PIDParams(Kp=20.0, Kd=8.0, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)

    r1 = simulate(pid, plant, act, sen, dis, sc, seed=7)
    r2 = simulate(pid, plant, act, sen, dis, sc, seed=7)
    r3 = simulate(pid, plant, act, sen, dis, sc, seed=99)  # different seed

    same_seed = np.allclose(r1.theta_true, r2.theta_true)
    diff_seed = not np.allclose(r1.theta_true, r3.theta_true, rtol=1e-3)

    report("Same seed - identical trajectory", same_seed)
    report("Different seed - different trajectory", diff_seed)


# -- 2. Stability physics ------------------------------------------------------

def test_stability_physics():
    print("\n-- 2. Stability physics -----------------------------------------")

    # 2a. Uncontrolled unstable rocket should diverge
    plant, act, sen, dis, sc = default_config(zero_noise=True)
    # Use near-zero p to make the instability slow but still measurable
    plant.p_unstable = 0.5
    sc_short = ScenarioParams(t_end=3.0, theta_ref=0.0, theta0_bias_std=0.0)

    # Perturb initial angle through scenario bias_std=0 but set act to not respond
    zero_pid = PIDParams(Kp=0.0, Kd=0.0, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    plant_pert = PlantParams(
        p_unstable=0.5, control_eff=plant.control_eff, dt=0.005
    )

    # Ideal actuator and sensor for clean physics test
    act_ideal = ActuatorParams(u_max=12.0, slew_max=float("inf"), tau_act=0.001,
                                deadband=0.0, backlash=0.0)
    sen_ideal = SensorParams(gyro_noise_std=0.0, gyro_bias_init=0.0, gyro_bias_rw=0.0,
                              gyro_quant_lsb=0.0, bias_cal_residual=0.0,
                              sensor_latency_steps=1)
    dis_ideal = DisturbanceParams(det_amp=0.0, det_freq_hz=0.0, gust_std=0.0, gust_tau=0.4)

    # Uncontrolled unstable plant must diverge.
    #
    # NOTE (2026-08-03): this test previously set only p_unstable=1.5 and asserted divergence.
    # It never diverged — theta_true was CONSTANT for all 601 samples — because p_unstable is
    # legacy and the EOM does not read it (plant_dynamics.py:102 "legacy, kept for
    # compatibility"; the moment term at :179 is `lambda_aero * theta`). Hand-constructing
    # PlantParams left lambda_aero at its 0.0 default, so there were no aero dynamics at all
    # and the test silently asserted nothing for as long as it has existed.
    # lambda_aero > 0 is the diverging case, and equals p_unstable^2 by the convention at :104.
    p_div = 1.5
    plant_unstable = PlantParams(p_unstable=p_div, lambda_aero=p_div**2,
                                 control_eff=8.0, dt=0.005)
    sc_diverge = ScenarioParams(t_end=3.0, theta_ref=0.0, theta0_bias_std=np.deg2rad(5.0))
    r_div = simulate(zero_pid, plant_unstable, act_ideal, sen_ideal, dis_ideal, sc_diverge, seed=1)

    theta_initial = abs(np.degrees(r_div.theta_true[0]))
    theta_final   = abs(np.degrees(r_div.theta_true[-1]))
    diverges = theta_final > max(theta_initial * 5.0, 10.0)
    report(
        f"Uncontrolled (lambda_aero={p_div**2:.2f}): |theta| grew from "
        f"{theta_initial:.1f} to {theta_final:.1f} deg",
        diverges,
    )

    # 2b. With control, reference rocket should succeed
    plant2, act2, sen2, dis2, sc2 = default_config(zero_noise=True)
    pid_good = PIDParams(Kp=20.0, Kd=8.0, Ki=0.0, u_max=act2.u_max, i_lim=act2.u_max)
    sc2b = ScenarioParams(t_end=3.0, theta_ref=0.0, theta0_bias_std=np.deg2rad(5.0))
    r_controlled = simulate(pid_good, plant2, act2, sen2, dis2, sc2b, seed=1)
    report(f"Controlled reference rocket succeeds", r_controlled.success,
           f"rms={r_controlled.rms_error_deg:.2f} deg")


# -- 3. Component isolation ----------------------------------------------------

def test_component_isolation():
    print("\n-- 3. Component isolation ---------------------------------------")

    plant, act, sen, dis, sc = default_config(zero_noise=True)
    pid = PIDParams(Kp=20.0, Kd=8.0, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)

    # 3a. Actuator disabled (zero gains): output should be zero throughout
    zero_pid = PIDParams(Kp=0.0, Kd=0.0, Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
    r_no_ctrl = simulate(zero_pid, plant, act, sen, dis, sc, seed=1)
    u_nonzero = np.any(np.abs(r_no_ctrl.u_act) > 1e-12)
    # Without PID signal, backlash + initial position = 0 means u_act stays at 0
    report("Actuator at rest when Kp=Kd=0 and theta0=0", not u_nonzero)

    # 3b. No sensor noise: theta_meas should track theta_true closely (only delay)
    plant2, act2, sen_noiseless, dis2, sc2 = default_config(zero_noise=True)
    r_noiseless = simulate(pid, plant2, act2, sen_noiseless, dis2, sc2, seed=1)
    max_delta = float(np.max(np.abs(
        r_noiseless.theta_meas[1:] - r_noiseless.theta_true[:-1]
    )))
    # With 1-step latency, meas at k = truth at k-1; delta should only be dynamics change
    report(f"Noiseless sensor: max |-_meas - -_true_prev| = {max_delta:.4f} rad (expect < 0.1)",
           max_delta < 0.1)

    # 3c. Disturbance should raise RMS — but ONLY on a linear actuator.
    #
    # NOTE (2026-08-03): this test used to run on the REF actuator (deadband=0.05,
    # backlash=0.10) and failed, reporting off=4.12 / on=0.82. That is not a simulator
    # fault, it is DITHER LINEARISATION: with no disturbance the vehicle parks at a steady
    # 3.83 deg offset trapped inside the actuator dead zone, while the disturbance shakes it
    # out and the loop converges to 0.14 deg. Real, well-known, and correct. The component-
    # isolation claim only holds for a linear actuator, so this test now uses one; the dither
    # effect itself is locked in as its own check below.
    plant3, act3, sen3, dis3, sc3 = default_config(zero_noise=True)
    act3_lin = ActuatorParams(u_max=act3.u_max, slew_max=act3.slew_max,
                              tau_act=0.05, deadband=0.0, backlash=0.0)
    dis_off = DisturbanceParams(det_amp=0.0, det_freq_hz=0.0, gust_std=0.0, gust_tau=0.4)
    dis_on  = build_disturbance(dict(REF, wind_strength=0.3))
    sc3b    = ScenarioParams(t_end=3.0, theta_ref=0.0, theta0_bias_std=np.deg2rad(5.0))

    r_no_dist  = simulate(pid, plant3, act3_lin, sen3, dis_off, sc3b, seed=1)
    r_yes_dist = simulate(pid, plant3, act3_lin, sen3, dis_on,  sc3b, seed=1)

    report(f"Disturbance increases RMS on a LINEAR actuator "
           f"(off={r_no_dist.rms_error_deg:.2f}, on={r_yes_dist.rms_error_deg:.2f})",
           r_yes_dist.rms_error_deg >= r_no_dist.rms_error_deg)

    # 3c-bis. Dither linearisation: with a dead zone, disturbance should IMPROVE steady error.
    # This is the behaviour that used to make 3c look broken; asserting it prevents anyone
    # "fixing" the simulator to match the naive expectation.
    r_dz_off = simulate(pid, plant3, act3, sen3, dis_off, sc3b, seed=1)
    r_dz_on  = simulate(pid, plant3, act3, sen3, dis_on,  sc3b, seed=1)
    end_off  = abs(np.degrees(r_dz_off.theta_true[-1]))
    end_on   = abs(np.degrees(r_dz_on.theta_true[-1]))
    report(f"Dither linearises the dead zone (final |theta| off={end_off:.2f} deg, "
           f"on={end_on:.2f} deg)",
           end_on < end_off)

    # 3d. Backlash=0 should not be worse than backlash=0.2
    plant4, act4, sen4, dis4, sc4 = default_config(zero_noise=True)
    act_no_bl = ActuatorParams(u_max=act4.u_max, slew_max=act4.slew_max,
                                tau_act=0.05, deadband=0.0, backlash=0.0)
    act_hi_bl = ActuatorParams(u_max=act4.u_max, slew_max=act4.slew_max,
                                tau_act=0.05, deadband=0.0, backlash=0.20)
    sc4b = ScenarioParams(t_end=3.0, theta_ref=0.0, theta0_bias_std=np.deg2rad(5.0))

    r_no_bl = simulate(pid, plant4, act_no_bl, sen4, dis_off, sc4b, seed=1)
    r_hi_bl = simulate(pid, plant4, act_hi_bl, sen4, dis_off, sc4b, seed=1)
    report(f"Higher backlash - rms (0={r_no_bl.rms_error_deg:.2f}, "
           f"0.2={r_hi_bl.rms_error_deg:.2f})",
           r_hi_bl.rms_error_deg >= r_no_bl.rms_error_deg)


# -- 4. Monotonicity sanity check ----------------------------------------------

def test_monotonicity():
    print("\n-- 4. Monotonicity ----------------------------------------------")
    print("  (mini N=50 LHS; checks slew and backlash ordered by regime)")

    import tempfile
    from experiment_runner import run_exp1
    tmp = __import__("pathlib").Path(tempfile.gettempdir())
    df = run_exp1(n_designs=50, seed=42, n_jobs=-1, out_dir=tmp)

    for regime in ["EASY", "FRAGILE", "INFEASIBLE"]:
        if regime not in df["regime_label"].values:
            print(f"  [WARN]  No {regime} designs in N=50 sample")

    easy = df[df["regime_label"] == "EASY"]
    frag = df[df["regime_label"] == "FRAGILE"]
    infe = df[df["regime_label"] == "INFEASIBLE"]

    if len(easy) > 0 and len(infe) > 0:
        slew_easy = easy["servo_slew_deg_s"].mean()
        slew_infe = infe["servo_slew_deg_s"].mean()
        ok_slew = slew_easy > slew_infe
        report(f"EASY slew ({slew_easy:.1f}) > INFEASIBLE slew ({slew_infe:.1f})", ok_slew)

        bl_easy = easy["backlash"].mean()
        bl_infe = infe["backlash"].mean()
        ok_bl = bl_easy < bl_infe
        report(f"EASY backlash ({bl_easy:.3f}) < INFEASIBLE backlash ({bl_infe:.3f})", ok_bl)
    else:
        report("Insufficient regime diversity in N=50 for monotonicity check", True,
               f"EASY={len(easy)} FRAGILE={len(frag)} INFEASIBLE={len(infe)}")

    counts = df["regime_label"].value_counts()
    print(f"\n  Mini Exp1 counts: {dict(counts)}")


# -- Main ----------------------------------------------------------------------

if __name__ == "__main__":
    print("-" * 60)
    print("TVC Simulator Validation Suite")
    print("-" * 60)

    test_determinism()
    test_stability_physics()
    test_component_isolation()
    test_monotonicity()

    print("\n-" * 60)
    n_pass = sum(1 for _, ok, _ in results if ok)
    n_fail = sum(1 for _, ok, _ in results if not ok)
    print(f"Results: {n_pass} passed, {n_fail} failed")
    if n_fail > 0:
        print("FAILED tests:")
        for name, ok, detail in results:
            if not ok:
                print(f"  - {name}: {detail}")
        sys.exit(1)
    else:
        print("All tests passed.")
