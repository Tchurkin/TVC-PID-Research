function cfg = s2r_build_config()
% Build all simulation, controller, mismatch, and analysis settings.

cfg.execution.rng_seed = 42;
cfg.execution.mode = "quick"; % "quick" for iteration, "full" for paper runs. "ultra_quick" for testing only.

cfg.paths.telemetry_dir = fullfile(pwd, 'real_flight_data_hypothetical');
cfg.paths.output_root = fullfile(pwd, 'outputs', 's2r_decomposition');

% Time settings
cfg.sim.dt = 0.01;
cfg.sim.t_max = 6.0;
cfg.sim.ref_theta_deg = 0.0;

% Vehicle baseline
cfg.vehicle.g0 = 9.80665;
cfg.vehicle.rho = 1.225;
cfg.vehicle.mu_air = 1.81e-5;
cfg.vehicle.d_ref_m = 0.075;
cfg.vehicle.S_ref_m2 = pi * (cfg.vehicle.d_ref_m / 2)^2;
cfg.vehicle.L_ref_m = 0.55;
cfg.vehicle.lever_arm_m = 0.24;
cfg.vehicle.mass_wet_kg = 1.85;
cfg.vehicle.mass_dry_kg = 1.63;
cfg.vehicle.burn_time_s = 1.65;
cfg.vehicle.Iyy_wet = 0.095;
cfg.vehicle.Iyy_dry = 0.082;
cfg.vehicle.max_gimbal_deg = 10.0;

% Motor and thrust model
cfg.motor.thrust_nominal_N = 46.0;
cfg.motor.thrust_curve_t = [0.0 0.15 0.4 0.9 1.3 1.65 6.0]';
cfg.motor.thrust_curve_N = [0.0 35.0 47.0 44.0 38.0 18.0 0.0]';
cfg.motor.mc_variation_pct = 5.0;

% Aerodynamic LUTs over alpha (deg) and Reynolds number.
alpha_grid = (-20:5:20)';
re_grid = [2e4 4e4 6e4 8e4 1e5]';
[A, R] = ndgrid(alpha_grid, re_grid);
cd0 = 0.45 + 0.08 * (A / 20).^2;
cd_re = 0.06 * (8e4 ./ max(R, 1));
cm_alpha = -0.08 * (A / 10) .* (1 + 0.15 * (R - 6e4) / 6e4);
cfg.aero.alpha_grid_deg = alpha_grid;
cfg.aero.re_grid = re_grid;
cfg.aero.Cd_table = cd0 + cd_re;
cfg.aero.Cm_table = cm_alpha;
cfg.aero.Cmq = 0.045;

% Actuator nominal transfer function G(s)=wn^2/(s^2+2*zeta*wn*s+wn^2) * e^{-tau s}
cfg.actuator.wn_rad_s = 42.0;
cfg.actuator.zeta = 0.72;
cfg.actuator.delay_s = 0.045;
cfg.actuator.slew_deg_s = 70.0;
cfg.actuator.deadband_deg = 0.12;

% Sensor model
cfg.sensor.theta_noise_deg = 0.18;
cfg.sensor.q_noise_deg_s = 1.1;

% Controllers
cfg.controllers.list = {"PID", "ADRC"};

cfg.pid.kp = 2.3;
cfg.pid.kd = 0.82;

cfg.adrc.omega_o = 18.0;
cfg.adrc.kp = 15.0;
cfg.adrc.kd = 5.0;

% Mismatch sources and sampling ranges (fractional unless unit specified)
cfg.mismatch.sources = {
    'actuator_delay', ...
    'servo_slew', ...
    'deadband', ...
    'sensor_noise', ...
    'inertia', ...
    'aero_coeff', ...
    'thrust_misalignment', ...
    'wind_torque'};

cfg.mismatch.range.actuator_delay = [0.0, 0.040]; % +seconds
cfg.mismatch.range.servo_slew = [0.0, -40.0]; % delta deg/s
cfg.mismatch.range.deadband = [0.0, 0.35]; % +deg
cfg.mismatch.range.sensor_noise = [0.0, 2.5]; % scale factor multiplier added as 1+x
cfg.mismatch.range.inertia = [-0.25, 0.25]; % fraction
cfg.mismatch.range.aero_coeff = [-0.35, 0.35]; % fraction
cfg.mismatch.range.thrust_misalignment = [-2.0, 2.0]; % deg
cfg.mismatch.range.wind_torque = [0.0, 0.45]; % N*m

% Analysis and decomposition
if cfg.execution.mode == "full"
    cfg.analysis.mc_runs_per_source = 1000;
    cfg.analysis.stress_map_samples = 2000;
    cfg.analysis.inc_fidelity_mc_runs = 60;
    cfg.analysis.morris_trajectories = 10;
elseif cfg.execution.mode == "ultra_quick"
    % Ultra-light mode for testing: 1 launch, minimal samples
    cfg.analysis.max_launches = 1;
    cfg.analysis.mc_runs_per_source = 20;
    cfg.analysis.stress_map_samples = 50;
    cfg.analysis.inc_fidelity_mc_runs = 5;
    cfg.analysis.morris_trajectories = 2;
else  % "quick"
    cfg.analysis.mc_runs_per_source = 120;
    cfg.analysis.stress_map_samples = 300;
    cfg.analysis.inc_fidelity_mc_runs = 20;
    cfg.analysis.morris_trajectories = 5;
end
cfg.analysis.max_launches = inf;
cfg.analysis.stability_theta_deg = 45;
cfg.analysis.stability_q_deg_s = 450;
cfg.analysis.agree_rmse_deg = 6.0;
cfg.analysis.agree_trend_corr = 0.7;

% Incremental fidelity and Morris screening run-counts
if cfg.execution.mode == "full"
    cfg.analysis.inc_fidelity_mc_runs = 60;
    cfg.analysis.morris_trajectories  = 10;
else
    cfg.analysis.inc_fidelity_mc_runs = 20;
    cfg.analysis.morris_trajectories  = 5;
end

% FidelityConfig: toggle each mismatch source on/off.
% All true by default (OFAT analysis unchanged).
% Set to false to build a reduced-fidelity simulation for incremental analysis.
cfg.fidelity_config.use_actuator_delay      = true;
cfg.fidelity_config.use_servo_slew          = true;
cfg.fidelity_config.use_deadband            = true;
cfg.fidelity_config.use_sensor_noise        = true;
cfg.fidelity_config.use_inertia             = true;
cfg.fidelity_config.use_aero_coeff          = true;
cfg.fidelity_config.use_thrust_misalignment = true;
cfg.fidelity_config.use_wind_torque         = true;

% Mismatch tier classification: 1 = Tier 1 (likely dominant), 2 = Tier 2 (secondary)
cfg.mismatch.tier.actuator_delay      = 1;
cfg.mismatch.tier.servo_slew          = 1;
cfg.mismatch.tier.deadband            = 1;
cfg.mismatch.tier.sensor_noise        = 1;
cfg.mismatch.tier.inertia             = 2;
cfg.mismatch.tier.aero_coeff          = 2;
cfg.mismatch.tier.thrust_misalignment = 2;
cfg.mismatch.tier.wind_torque         = 2;

end
