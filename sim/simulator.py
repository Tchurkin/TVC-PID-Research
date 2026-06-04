"""
simulator.py — Main simulation loop for 1-DOF TVC rocket.

This module is the single integration point for all sub-models.
No physics, control law, or sensor logic lives here — only orchestration.

Call graph per timestep k
─────────────────────────
  disturbance_model.step_disturbance()  →  d_eff
  controller.step_pid()                 →  u_cmd
  actuator_model.step_actuator()        →  u_act
  plant_dynamics.step_plant()           →  (theta, q)
  sensor_model.step_sensor()            →  (theta_ctrl, q_ctrl)

The controller always reads the PREVIOUS step's delayed sensor output
(the latency buffer from step k-1) so that the pipeline is causal.

Fault injection
───────────────
  ScenarioParams supports optional midburn faults:
      fault_time_s        — when the fault activates
      keff_fault_post     — control effectiveness multiplier after fault
      damp_fault_post     — aero damping multiplier after fault
      disturb_bias_post   — additional constant disturbance after fault (rad/s²)
  Default values keep these disabled (keff_fault=1.0, damp_fault=1.0, etc.).

Outputs
───────
  SimResult contains:
      trajectory arrays: t, theta_true, q_true, u_act, theta_meas, q_meas
      scalar metrics:    success, rms_error_deg, peak_error_deg, end_error_deg,
                         max_theta_deg, u_cmd_sat_frac, slew_sat_frac,
                         settling_time_s, oscillation_score
  All angle outputs are in degrees.  Arrays are shape (N,) with N = t_end/dt + 1.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from units import (
    SUCCESS_MAX_THETA_DEG, SUCCESS_END_ERROR_DEG,
    SUCCESS_RMS_ERROR_DEG, SUCCESS_PEAK_ERROR_DEG,
)
from plant_dynamics import PlantParams, step_plant
from actuator_model import ActuatorParams, ActuatorState, step_actuator
from sensor_model import SensorParams, SensorState, init_sensor_state, step_sensor
from disturbance_model import DisturbanceParams, GustState, init_gust_state, step_disturbance
from controller import PIDParams, PIDState, step_pid


@dataclass
class ScenarioParams:
    t_end: float = 3.0                  # burn duration (s)
    theta_ref: float = 0.0              # constant pitch reference (rad)
    theta0_bias_std: float = 0.0        # std of random initial angle offset (rad)
    # Optional midburn fault
    fault_time_s: float = float("inf")  # time at which fault activates (s)
    keff_fault_post: float = 1.0        # keff multiplier after fault
    damp_fault_post: float = 1.0        # aero damp multiplier after fault
    disturb_bias_post: float = 0.0      # extra constant disturbance after fault (rad/s²)


@dataclass
class SimResult:
    # Trajectory arrays (length N)
    t: np.ndarray
    theta_true: np.ndarray
    q_true: np.ndarray
    u_act: np.ndarray
    theta_meas: np.ndarray
    q_meas: np.ndarray
    # Scalar metrics
    success: bool
    rms_error_deg: float
    peak_error_deg: float
    end_error_deg: float
    max_theta_deg: float
    u_cmd_sat_frac: float
    slew_sat_frac: float
    settling_time_s: float
    oscillation_score: float


def simulate(
    pid: PIDParams,
    plant: PlantParams,
    actuator: ActuatorParams,
    sensor: SensorParams,
    disturbance: DisturbanceParams,
    scenario: ScenarioParams,
    seed: int = 1,
) -> SimResult:
    """
    Run one complete simulation from t=0 to t=t_end.

    Parameters are immutable after the call.  All random state is seeded from
    `seed`; same seed → identical trajectory (determinism guarantee).

    Returns a SimResult with trajectory arrays and scalar metrics.
    """
    rng = np.random.default_rng(seed)

    dt = plant.dt
    N = int(round(scenario.t_end / dt)) + 1
    t = np.arange(N) * dt

    # ── Detect high-fidelity physical modules ─────────────────────────────────
    # These are set on the scenario by apply_fidelity_config() when new flags are on.
    use_dyn_aero    = getattr(scenario, "use_dyn_aero",    False)
    use_thrust_curve = getattr(scenario, "use_thrust_curve", False)
    use_cg_shift    = getattr(scenario, "use_cg_shift",    False)
    use_nonlinear   = getattr(scenario, "use_nonlinear_aero", False)

    # Import new physics modules only when needed
    aero_params  = None
    motor_params = None
    motor_state  = None
    if use_dyn_aero or use_nonlinear or use_thrust_curve or use_cg_shift:
        from aero_model import AeroParams, dynamic_pressure, velocity_step
        from motor_model import MotorParams, MotorState, motor_thrust_and_mdot
        from motor_model import motor_vehicle_mass, motor_cg_position, motor_inertia_correction
        aero_params  = AeroParams.from_design(plant.Cm_alpha_signed, plant.static_margin)
        motor_params = MotorParams.F15(rocket_length=2.0 * plant.l_nozzle_m)
        motor_state  = MotorState.initial(motor_params)

    # Flight state for dynamic pressure integration
    v_flight = 0.01  # m/s — small nonzero to avoid division by zero in damping

    # ── Allocate output arrays ────────────────────────────────────────────────
    theta_arr = np.zeros(N)
    q_arr     = np.zeros(N)
    u_act_arr = np.zeros(N)
    theta_m_arr = np.zeros(N)
    q_m_arr     = np.zeros(N)

    # ── Initial conditions ────────────────────────────────────────────────────
    theta0 = scenario.theta0_bias_std * rng.standard_normal()
    theta_arr[0]   = theta0
    q_arr[0]       = 0.0
    u_act_arr[0]   = 0.0

    # ── Initialise sub-model states ───────────────────────────────────────────
    act_state   = ActuatorState(u_servo=0.0, u_output=0.0)
    sens_state  = init_sensor_state(sensor, theta0, rng)
    # Stationary gust initialisation — draws from N(0, gust_std) at t=0
    gust_state  = init_gust_state(disturbance, rng)
    pid_state   = PIDState(i_state=0.0, q_ctrl_prev=0.0)

    # Seed measurements at k=0 from initial condition
    theta_m_arr[0] = sens_state.theta_integrated
    q_m_arr[0]     = 0.0

    # The controller sees the PREVIOUS step's delayed measurements.
    # At k=1 the latency buffer holds the initial condition.
    theta_ctrl = theta_m_arr[0]
    q_ctrl     = q_m_arr[0]

    # ── Main loop ─────────────────────────────────────────────────────────────
    for k in range(1, N):
        post_fault = t[k] >= scenario.fault_time_s
        keff_fault = scenario.keff_fault_post if post_fault else 1.0
        damp_fault = scenario.damp_fault_post if post_fault else 1.0
        bias_post  = scenario.disturb_bias_post if post_fault else 0.0

        # ── Resolve current-step physical quantities ──────────────────────────
        # These are constant in simple mode and time-varying in full mode.

        if use_thrust_curve and motor_params is not None:
            T_now, mdot = motor_thrust_and_mdot(motor_params, t[k])
            motor_state.advance(mdot, dt)
        else:
            from units import REF_THRUST_N
            T_now = plant.thrust_scale * REF_THRUST_N
            mdot  = 0.0

        if use_cg_shift and motor_params is not None and motor_state is not None:
            m_dry_casing = plant.m_kg + motor_params.m_casing
            m_total  = motor_vehicle_mass(plant.m_kg, motor_params, motor_state)
            x_cg_now = motor_cg_position(
                plant.l_nozzle_m * 0.56,  # dry CG at 56% of (2×l_nozzle) rocket length
                m_dry_casing, motor_params, motor_state
            )
            I_yy_now = motor_inertia_correction(
                plant.Iyy_kgm2, plant.l_nozzle_m * 0.56,
                m_dry_casing, motor_params, motor_state, x_cg_now
            )
        else:
            m_total  = plant.m_kg
            I_yy_now = plant.Iyy_kgm2

        if use_dyn_aero and aero_params is not None:
            q_dyn_now = dynamic_pressure(v_flight)
            # Advance velocity (translational dynamics)
            v_flight = velocity_step(
                v_flight, m_total, T_now,
                theta_arr[k - 1], q_dyn_now, aero_params, dt=dt
            )
        else:
            # Constant reference dynamic pressure from design space
            # At average flight speed for the reference motor
            v_ref_avg = 30.0  # m/s typical average speed for F-class rocket
            q_dyn_now = 0.5 * 1.20 * v_ref_avg**2  # ≈ 540 Pa

        # 1. Disturbance
        d_raw = step_disturbance(
            gust_state, disturbance, t[k], dt, rng,
            inertia_scale=plant.inertia_scale,
            mass_scale=plant.mass_scale,
        )
        d_eff = d_raw + bias_post * plant.inertia_scale / max(0.5, plant.mass_scale)

        # 2. Controller (uses previous step's delayed measurements)
        u_cmd = step_pid(pid_state, pid, theta_ctrl, q_ctrl, scenario.theta_ref, dt)

        # 3. Actuator
        u_act = step_actuator(act_state, actuator, u_cmd, dt)
        u_act_arr[k] = u_act

        # 4. Plant — route to simple or full physics based on scenario flags
        theta_new, q_new = step_plant(
            theta_arr[k - 1], q_arr[k - 1], u_act, d_eff,
            plant, keff_fault, damp_fault,
            # High-fidelity args (None → simple mode if flags are off)
            aero_params  = aero_params if (use_dyn_aero or use_nonlinear) else None,
            q_dyn        = q_dyn_now   if (use_dyn_aero or use_nonlinear) else None,
            v            = v_flight    if (use_dyn_aero or use_nonlinear) else None,
            T_now        = T_now       if (use_dyn_aero or use_nonlinear) else None,
            I_yy_now     = I_yy_now    if (use_dyn_aero or use_nonlinear) else None,
            nonlinear_aero = use_nonlinear,
        )
        theta_arr[k] = theta_new
        q_arr[k]     = q_new

        # 5. Sensor (advances latency buffer, returns delayed measurements)
        q_ctrl, theta_ctrl = step_sensor(
            sens_state, sensor, q_new, theta_new, dt, rng
        )
        q_m_arr[k]     = q_ctrl
        theta_m_arr[k] = theta_ctrl

    # ── Metrics ───────────────────────────────────────────────────────────────
    theta_d    = np.degrees(theta_arr)
    theta_ref_d = np.degrees(scenario.theta_ref)
    err_d      = theta_d - theta_ref_d

    rms_e  = float(np.sqrt(np.mean(err_d ** 2)))
    peak_e = float(np.max(np.abs(err_d)))
    end_e  = float(np.abs(err_d[-1]))
    max_th = float(np.max(np.abs(theta_d)))

    success = (
        max_th  < SUCCESS_MAX_THETA_DEG
        and end_e   < SUCCESS_END_ERROR_DEG
        and rms_e   < SUCCESS_RMS_ERROR_DEG
        and peak_e  < SUCCESS_PEAK_ERROR_DEG
    )

    # Saturation fractions
    u_sat_frac = float(np.mean(np.abs(u_act_arr) >= 0.95 * actuator.u_max))
    if actuator.slew_max < 1e6:
        slew_rates = np.abs(np.diff(u_act_arr)) / dt
        slew_sat_frac = float(np.mean(slew_rates >= 0.95 * actuator.slew_max))
    else:
        slew_sat_frac = 0.0

    # Settling time: last time |theta| ≥ 2 deg
    settled = np.abs(theta_d) < 2.0
    settling_time = float(t[-1])
    for i in range(N - 1, -1, -1):
        if not settled[i]:
            settling_time = float(t[min(i + 1, N - 1)])
            break
    else:
        settling_time = 0.0

    # Oscillation score: zero-crossing count of tracking error
    es = np.sign(err_d)
    nz = es[es != 0]
    osc_score = float(np.sum(np.abs(np.diff(nz)) > 0)) if len(nz) > 1 else 0.0

    return SimResult(
        t=t,
        theta_true=theta_arr,
        q_true=q_arr,
        u_act=u_act_arr,
        theta_meas=theta_m_arr,
        q_meas=q_m_arr,
        success=success,
        rms_error_deg=rms_e,
        peak_error_deg=peak_e,
        end_error_deg=end_e,
        max_theta_deg=max_th,
        u_cmd_sat_frac=u_sat_frac,
        slew_sat_frac=slew_sat_frac,
        settling_time_s=settling_time,
        oscillation_score=osc_score,
    )
