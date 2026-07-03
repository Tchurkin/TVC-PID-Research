"""
tools/vsr_demo.py  (2026-06-26)

Smoke-test + demonstration of the sim_vsr full architecture. Flies a commanded 30-deg
pitch-over maneuver under three stability schedules x two TVC controllers, full physics
(thrust curve, variable mass/inertia, aero on angle of attack, gimbal slew/deadband,
IMU noise+latency, OU wind), and prints a metrics comparison.

Question it answers: does ACTIVE variable stability (fling unstable, catch stable) reach a
commanded attitude faster/cleaner than a fixed-stable airframe, at realistic morph speeds --
and does the TVC controller choice (PID vs ADRC) matter?
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from sim_vsr import (VehicleParams, TVCParams, MorphParams, IMUParams,
                     TVCController, PIDGains, ADRCGains,
                     StabilityScheduler, SchedulerCfg, Scenario, simulate)


def run_cell(mode, ctrl_kind, vp, scn, morph_rate=15.0, seeds=range(8)):
    tvc_p = TVCParams(u_max_deg=7.0, slew_max_deg_s=120.0, deadband_deg=0.2, backlash_deg=0.3)
    morph_p = MorphParams(sm_min=-0.4, sm_max=0.9, rate_cal_s=morph_rate)
    imu_p = IMUParams(gyro_noise_dps=0.6, gyro_bias_dps=0.3, theta_noise_deg=0.3, latency_steps=2)
    cfg = SchedulerCfg(mode=mode, sm_stable=0.8, sm_unstable=-0.3,
                       switch_in_deg=6.0, switch_out_deg=10.0)
    rows = []
    for sd in seeds:
        if ctrl_kind == "pid":
            ctrl = TVCController("pid", PIDGains(Kp=0.3, Kd=0.06, Ki=0.6), scn.dt)
        else:
            # b0 ~ keff at maneuver (T~14N, L~0.35m, Iyy~0.016) ~ 300; use specs-derived estimate
            _, Iyy = (vp.m_dry, vp.Iyy_dry)
            b0 = 14.0 * vp.L_nozzle / vp.Iyy_dry
            ctrl = TVCController("adrc", ADRCGains(omega_c=6.0, omega0=30.0, b0=b0), scn.dt)
        sched = StabilityScheduler(cfg)
        r = simulate(vp, tvc_p, morph_p, imu_p, ctrl, sched, scn, seed=sd)
        rows.append(r.metrics)
    return rows


def agg(rows):
    def mean_of(key, default=np.nan):
        vals = [r[key] for r in rows if r.get(key) is not None]
        return np.mean(vals) if vals else default
    return dict(
        success=np.mean([r["success"] for r in rows]),
        diverged=np.mean([r["diverged"] for r in rows]),
        t_reach=mean_of("t_reach"),
        settle=mean_of("settle_time"),
        overshoot=mean_of("overshoot_deg"),
        rms=mean_of("rms_err_deg"),
        max_alpha=mean_of("max_alpha_deg"),
        slew_sat=mean_of("slew_sat_frac"),
        Pi=mean_of("Pi"),
    )


def main():
    vp = VehicleParams()
    scn = Scenario(t_end=2.0, dt=0.005, launch_speed=15.0, maneuver_time=0.8,
                   maneuver_deg=20.0, maneuver_ramp=0.4, wind_sigma=1.5, wind_tau=0.4)
    print("=== sim_vsr DEMO: 20-deg pitch-over maneuver, powered flight, full physics, 8 seeds ===")
    print(f"  vehicle: m={vp.m_dry}-{vp.m_dry+vp.m_prop:.2f}kg Iyy={vp.Iyy_dry}-{vp.Iyy_wet} "
          f"L_nozzle={vp.L_nozzle}m  CN_a={vp.CN_alpha}")
    print(f"  TVC: u_max=7deg slew=120deg/s lat=2steps   morph rate=15 cal/s (~70ms flip)")
    print(f"  schedule: stable sm=+0.8cal, unstable sm=-0.3cal, switch 6/10deg hysteresis\n")

    hdr = f"  {'controller':5} {'mode':14} {'succ':>5} {'div':>5} {'t_reach':>8} {'settle':>7} {'over':>6} {'rms':>6} {'maxA':>6} {'slewSat':>8} {'Pi':>7}"
    for ctrl_kind in ["pid", "adrc"]:
        print(f"--- TVC controller: {ctrl_kind.upper()} ---")
        print(hdr)
        for mode in ["fixed_stable", "fixed_unstable", "switched"]:
            a = agg(run_cell(mode, ctrl_kind, vp, scn))
            def f(x, w=8, p=2):
                return ("{:>%d}" % w).format("-" if (x is None or (isinstance(x, float) and np.isnan(x))) else f"{x:.{p}f}")
            print(f"  {ctrl_kind:5} {mode:14} {a['success']:5.2f} {a['diverged']:5.2f} "
                  f"{f(a['t_reach'])} {f(a['settle'],7)} {f(a['overshoot'],6,1)} {f(a['rms'],6,1)} "
                  f"{f(a['max_alpha'],6,1)} {f(a['slew_sat'],8,3)} {f(a['Pi'],7,0)}")
        print()

    print("READ: 'switched' should reach the setpoint with lower t_reach/settle than fixed_stable")
    print("without diverging. If fixed_unstable diverges (success~0) but switched succeeds, the")
    print("catch-stable phase is doing its job. Compare PID vs ADRC slewSat: ADRC should be lower.")


if __name__ == "__main__":
    main()
