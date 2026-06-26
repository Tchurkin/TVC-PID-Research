function P = default_rocket_params()
%DEFAULT_ROCKET_PARAMS Shared parameter struct for locked-direction studies.

P.rocket = struct();
P.rocket.mass = 1.20;
P.rocket.Iyy = 0.018;
P.rocket.static_margin = 0.10;
P.rocket.Cm_alpha = -52.0;
P.rocket.control_effectiveness = 8.0;
P.rocket.thrust = 35.0;
P.rocket.servo_slew = 75.0;      % loaded gimbal slew (deg/s)
P.rocket.max_gimbal = 15.0;      % physical gimbal half-range (deg)
P.rocket.deadband = 0.05;        % command units in realistic model
P.rocket.backlash = 0.10;        % command units in realistic model
P.rocket.latency = 3;            % samples
P.rocket.wind_strength = 0.15;   % gust sigma (rad/s^2)

P.analysis = struct();
P.analysis.t_end = 3.0;
P.analysis.disturbance_amp = 0.05;
P.analysis.disturbance_freq_hz = 0.80;
P.analysis.theta0_deg_set = [0 3];
P.analysis.seeds = 1:3;
P.analysis.success_rms_deg = 15.0;
P.analysis.success_peak_deg = 50.0;
P.analysis.success_end_deg = 15.0;
P.analysis.success_max_theta_deg = 70.0;
P.analysis.settle_band_deg = 2.0;
P.analysis.oscillation_min_amp_deg = 1.0;

P.tuning = struct();
P.tuning.Kp_grid = [10 20 45 80];
P.tuning.Kd_grid = [4 8 16 32];
P.tuning.under_scale = 0.60;
P.tuning.over_scale = 1.40;

P.classification = struct();
P.classification.easy_success = 0.80;
P.classification.easy_robustness = 1.00;
P.classification.easy_u_sat_frac = 0.60;
P.classification.easy_slew_sat_frac = 1.00;
P.classification.easy_rms_error_deg = 8.0;
P.classification.easy_settling_time_s = 3.0;
P.classification.easy_oscillation_score = 3.0;
P.classification.fragile_success = 0.35;
P.classification.fragile_rms_error_deg = 16.0;
P.classification.infeasible_robustness = 0.0;
