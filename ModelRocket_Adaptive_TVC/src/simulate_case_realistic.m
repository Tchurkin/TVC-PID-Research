function out = simulate_case_realistic(controller_name, sc, cfg, seed, realism)
%SIMULATE_CASE_REALISTIC  1-DOF TVC sim with realistic sensors + actuator.
%
%  Models a hobby model-rocket flight computer with:
%   - MEMS gyro: white noise + bias random walk + ADC quantization
%   - PURE GYRO INTEGRATION for pitch (no tilt-from-gravity correction):
%     during powered flight |a_thrust| >> |g|, so the accelerometer
%     cannot disambiguate gravity from thrust. Real hobby flight
%     computers freeze tilt-aiding at liftoff and accept gyro drift
%     for the duration of the burn. We model that here.
%   - Optional pre-launch bias calibration: average gyro for `bias_cal_s`
%     seconds before fault_time, subtract the estimate. Mimics the
%     standard "hold still on the pad" calibration step.
%   - 1-sample sensor latency
%   - Servo deadband + backlash on direction reversal + position-feedback noise
%   - Wind gust as 1st-order Dryden-shaped torque
%   - Optional keff drift during burn (mass loss)
%
%  realism struct (defaults applied if missing):
%   .gyro_noise_std   (rad/s)   white gyro noise sigma         [0.015]
%   .gyro_bias_init   (rad/s)   initial bias 1-sigma           [0.010]
%   .gyro_bias_rw     (rad/s/sqrt(s)) bias random walk         [0.005]
%   .gyro_quant_lsb   (rad/s)   quantization step              [2*pi/4000]
%   .bias_cal_s       (s)       pre-flight bias calibration window [0.5]
%   .bias_cal_residual_frac     fraction of bias remaining after cal [0.10]
%   .sensor_latency_steps (#)   one-step latency               [1]
%   .servo_deadband   (units)   ignored command                [0.05]
%   .servo_backlash   (units)   reversal slop                  [0.10]
%   .servo_pos_noise  (units)   servo-feedback noise sigma     [0.01]
%   .gust_std         (rad/s^2) Dryden torque sigma            [0.30]
%   .gust_tau         (s)       gust correlation time          [0.40]
%   .keff_drift_rate  (1/s)     relative drift rate            [0.0]
%   .theta_init_bias  (rad)     initial pose-bias 1-sigma      [deg2rad(0.5)]
%
%  The controllers see ONLY the noisy, quantized, drift-prone measurements.

if nargin < 4, seed = 1; end
if nargin < 5, realism = struct(); end
realism = apply_defaults(realism, default_realism());

rng(seed);

dt = cfg.dt;
t  = (0:dt:sc.t_end)';
N  = numel(t);

% True states
theta_true = zeros(N,1);
q_true     = zeros(N,1);
u_act      = zeros(N,1);
u_cmd      = zeros(N,1);

% Measured / observed (what the controller sees)
theta_meas = zeros(N,1);
q_meas     = zeros(N,1);
u_act_meas = zeros(N,1);

% Estimator log channels
keff_est_log   = nan(N,1);
slew_est_log   = nan(N,1);
gain_scale_log = nan(N,1);
sat_log        = nan(N,1);
sat_inst_log   = nan(N,1);
sat_streak_log = nan(N,1);
keff_gate_log  = nan(N,1);
keff_scale_log = nan(N,1);
slew_scale_log = nan(N,1);
demand_rate_log = nan(N,1);
du_obs_lp_log   = nan(N,1);
disturb_lp_log  = nan(N,1);
pos_limited_log = nan(N,1);
confidence_log  = nan(N,1);
adapt_weight_log = nan(N,1);
shield_active_log = nan(N,1);
shield_slew_log = nan(N,1);
shield_attitude_log = nan(N,1);
demand_rate_decoupled_log = nan(N,1);

theta_true(1) = cfg.plant.theta0 + realism.theta_init_bias * randn();
q_true(1)     = cfg.plant.q0;
u_act(1)      = cfg.plant.u_act0;
theta_meas(1) = theta_true(1);
q_meas(1)     = q_true(1);
u_act_meas(1) = u_act(1);

% Gyro bias (random walk). True bias the sensor experiences:
gyro_bias_true = realism.gyro_bias_init * randn();

% Pre-flight "hold still on the pad" bias calibration:
% Average the gyro for bias_cal_s seconds with the rocket motionless,
% then subtract. The residual is whatever the calibration missed
% (finite averaging window, post-cal bias drift during the flight).
%   - bias_cal_residual_frac models the finite-window averaging error
%     (e.g. 0.10 means 10% of the initial bias survives calibration)
%   - the random-walk component then accumulates from t=0 onward
bias_estimate = (1 - realism.bias_cal_residual_frac) * gyro_bias_true;

% Backlash state: last commanded direction
last_du_sign = 0;

% Gust state (1st-order shaped noise)
gust = 0;
gust_alpha = exp(-dt / max(1e-3, realism.gust_tau));
gust_sigma_step = realism.gust_std * sqrt(1 - gust_alpha^2);

% Theta from PURE GYRO INTEGRATION (no tilt aid -- thrust dominates g during burn)
theta_int = theta_meas(1);

% Latency buffers (length = lat samples of delay)
lat = max(1, realism.sensor_latency_steps);
buf_theta = repmat(theta_meas(1), lat, 1);
buf_q     = repmat(q_meas(1), lat, 1);
buf_uact  = repmat(u_act(1),    lat, 1);

% Previous observed samples (one-controller-step back)
q_obs_prev  = q_meas(1);
ua_obs_prev = u_act(1);

ctrl_name = upper(string(controller_name));
ctrl      = cfg.controllers.(ctrl_name);
adapt     = struct();

for k = 2:N
    tk = t(k);

    % Plant params (with optional keff drift)
    keff = cfg.plant.control_eff;
    if realism.keff_drift_rate ~= 0
        keff = keff * exp(-realism.keff_drift_rate * tk);
    end
    aero_damp = cfg.plant.aero_damp;
    tau_act   = cfg.plant.tau_act;
    slew_max  = cfg.plant.slew_max;

    if tk >= sc.fault_time
        keff      = keff      * sc.control_eff_scale_post;
        aero_damp = aero_damp * sc.aero_damp_scale_post;
        tau_act   = tau_act   * sc.tau_scale_post;
        slew_max  = slew_max  * sc.slew_scale_post;
    end

    % --- Disturbance: nominal sinusoid + Dryden-shaped gust ----------
    sinus = sc.disturbance_amp * sin(2*pi*sc.disturbance_freq_hz*tk);
    if tk >= sc.fault_time
        sinus = sc.disturb_scale_post * sinus + sc.disturb_bias_post;
    end
    gust = gust_alpha * gust + gust_sigma_step * randn();
    disturb = sinus + gust;

    % ---- Controller sees latent measurements ------------------------
    th_obs = buf_theta(end);    % delayed by `lat` samples
    q_obs  = buf_q(end);
    ua_obs = buf_uact(end);

    switch ctrl_name
        case "FIXED_LQR"
            x = [th_obs; q_obs];
            u_cmd(k) = max(-ctrl.u_max, min(ctrl.u_max, -ctrl.K * x));

        case "PID"
            if k == 2
                adapt.pid_integrator = 0;
                adapt.pid_e_prev     = -th_obs;
            end
            pid_state.integrator = adapt.pid_integrator;
            pid_state.e_prev     = adapt.pid_e_prev;
            [u, pid_state_new] = pid_layer(th_obs, pid_state, ctrl, dt);
            adapt.pid_integrator = pid_state_new.integrator;
            adapt.pid_e_prev     = pid_state_new.e_prev;
            u_cmd(k) = u;

        case "ADAPTIVE_KEFF_LQR"
            [u, adapt, d] = lqr_layer_keff_adaptive( ...
                th_obs, q_obs, ua_obs, q_obs_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            keff_est_log(k)   = d.keff_est;
            gain_scale_log(k) = d.gain_scale;

        case "SLEW_ADAPTIVE"
            ucmd_prev = u_cmd(max(1, k-1));
            [u, adapt, d] = lqr_layer_slew_adaptive( ...
                th_obs, q_obs, ua_obs, ua_obs_prev, ucmd_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            slew_est_log(k)   = d.slew_est;
            gain_scale_log(k) = d.gain_scale;
            sat_log(k)        = d.saturating;
            sat_inst_log(k)   = d.saturating_inst;
            sat_streak_log(k) = d.sat_streak;
            slew_scale_log(k) = d.slew_scale;
            demand_rate_log(k)= d.demand_rate;
            du_obs_lp_log(k)  = d.du_obs_lp;
            shield_active_log(k) = d.shield_active;
            shield_slew_log(k) = d.shield_slew;
            shield_attitude_log(k) = d.shield_attitude;

        case "SIGMA_MRAC"
            [u, adapt, d] = lqr_layer_sigma_mrac( ...
                th_obs, q_obs, ua_obs, q_obs_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            keff_est_log(k)   = d.keff_est;
            gain_scale_log(k) = d.gain_scale;

        case "PCH_LQR"
            [u, adapt, d] = lqr_layer_pch_lqr( ...
                th_obs, q_obs, ua_obs, q_obs_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            keff_est_log(k)   = d.keff_est;
            slew_est_log(k)   = d.slew_est;
            gain_scale_log(k) = d.gain_scale;

        case "JOINT_ADAPTIVE"
            ucmd_prev = u_cmd(max(1, k-1));
            [u, adapt, d] = lqr_layer_joint_adaptive( ...
                th_obs, q_obs, ua_obs, ua_obs_prev, q_obs_prev, ucmd_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            keff_est_log(k)   = d.keff_est;
            slew_est_log(k)   = d.slew_est;
            gain_scale_log(k) = d.gain_scale;
            sat_log(k)        = d.saturating;
            sat_inst_log(k)   = d.saturating_inst;
            sat_streak_log(k) = d.sat_streak;
            keff_gate_log(k)  = d.keff_gate;
            keff_scale_log(k) = d.keff_scale;
            slew_scale_log(k) = d.slew_scale;
            demand_rate_log(k)= d.demand_rate;
            du_obs_lp_log(k)  = d.du_obs_lp;
            disturb_lp_log(k) = d.disturb_lp;
            if isfield(d, 'pos_limited')
                pos_limited_log(k) = d.pos_limited;
            end
            confidence_log(k) = d.confidence;
            adapt_weight_log(k) = d.adapt_weight;
            shield_active_log(k) = d.shield_active;
            shield_slew_log(k) = d.shield_slew;
            shield_attitude_log(k) = d.shield_attitude;
            demand_rate_decoupled_log(k) = d.demand_rate_decoupled;

        otherwise
            error('simulate_case_realistic:UnknownController', ...
                'Unknown controller: %s', ctrl_name);
    end

    % Save current observed samples for next iteration's *_prev
    q_obs_prev  = q_obs;
    ua_obs_prev = ua_obs;

    if tk < sc.fault_time && isfield(sc, 'startup_ramp_s') && sc.startup_ramp_s > 0
        startup_scale = min(1.0, tk / sc.startup_ramp_s);
        u_cmd(k) = startup_scale * u_cmd(k);
    end

    % ---- Servo nonlinearities: deadband + backlash ----------------
    u_eff = u_cmd(k);
    if abs(u_eff - u_act(k-1)) < realism.servo_deadband
        u_eff = u_act(k-1);   % deadband: small change ignored
    end
    du_cmd  = (u_eff - u_act(k-1)) / max(1e-5, tau_act);
    du_dir  = sign(du_cmd);
    if du_dir ~= 0 && du_dir ~= last_du_sign && last_du_sign ~= 0
        % Direction reversal: lose `backlash` units of motion before tracking
        backlash_off = realism.servo_backlash * du_dir;
        u_act(k-1) = u_act(k-1) - backlash_off;   % effective servo lag
    end
    if du_dir ~= 0, last_du_sign = du_dir; end

    du = max(-slew_max, min(slew_max, du_cmd));
    u_act(k) = u_act(k-1) + dt * du;
    u_act(k) = max(-cfg.plant.u_max, min(cfg.plant.u_max, u_act(k)));

    % ---- Plant dynamics --------------------------------------------
    % Static instability: rocket with CG aft of CP has positive p^2*theta
    % restoring(*destabilizing) term. If cfg.plant.p_unstable==0 this reduces
    % to the original stable plant.
    if isfield(cfg.plant, 'p_unstable')
        p_unst = cfg.plant.p_unstable;
    else
        p_unst = 0;
    end
    qdot      = keff * u_act(k) - aero_damp * q_true(k-1) + p_unst^2 * theta_true(k-1) + disturb;
    q_true(k) = q_true(k-1) + dt * qdot;
    theta_true(k) = theta_true(k-1) + dt * q_true(k);

    % ---- Sensor model ---------------------------------------------
    % True sensor bias evolves as a random walk
    gyro_bias_true = gyro_bias_true + realism.gyro_bias_rw * sqrt(dt) * randn();
    % Raw gyro reading
    q_raw = q_true(k) + gyro_bias_true + realism.gyro_noise_std * randn();
    if realism.gyro_quant_lsb > 0
        q_raw = realism.gyro_quant_lsb * round(q_raw / realism.gyro_quant_lsb);
    end
    % Apply pre-flight bias calibration (constant offset subtracted)
    q_corrected = q_raw - bias_estimate;
    q_meas(k)   = q_corrected;

    % Pitch via PURE GYRO INTEGRATION.
    % On a model rocket under thrust, |a_thrust| is 5-15 g and the
    % accelerometer cannot disambiguate gravity from thrust, so we
    % cannot complement with a tilt-from-gravity reference. The bias
    % residual will integrate into accumulated drift over the flight
    % (~7 deg over 10 s with default MEMS bias parameters and 0.5 s cal).
    theta_int = theta_int + dt * q_corrected;
    theta_meas(k) = theta_int;

    % Actuator position feedback noise
    u_act_meas(k) = u_act(k) + realism.servo_pos_noise * randn();

    % Push to latency buffer
    buf_theta = [buf_theta(2:end); theta_meas(k)];
    buf_q     = [buf_q(2:end);     q_meas(k)];
    buf_uact  = [buf_uact(2:end);  u_act_meas(k)];
end

out.time          = t;
out.theta         = theta_true;       % truth, used for scoring
out.theta_meas    = theta_meas;       % what the FCC saw
out.q             = q_true;
out.q_meas        = q_meas;
out.u_cmd         = u_cmd;
out.u_act         = u_act;
out.u_act_meas    = u_act_meas;
out.keff_est      = keff_est_log;
out.slew_est      = slew_est_log;
out.gain_scale    = gain_scale_log;
out.saturating    = sat_log;
out.saturating_inst = sat_inst_log;
out.sat_streak    = sat_streak_log;
out.keff_gate     = keff_gate_log;
out.keff_scale    = keff_scale_log;
out.slew_scale    = slew_scale_log;
out.demand_rate   = demand_rate_log;
out.du_obs_lp     = du_obs_lp_log;
out.disturb_lp    = disturb_lp_log;
out.pos_limited   = pos_limited_log;
out.confidence    = confidence_log;
out.adapt_weight  = adapt_weight_log;
out.shield_active = shield_active_log;
out.shield_slew   = shield_slew_log;
out.shield_attitude = shield_attitude_log;
out.demand_rate_decoupled = demand_rate_decoupled_log;
out.controller    = ctrl_name;
out.scenario      = sc.kind;
out.seed          = seed;
end


function r = default_realism()
r.gyro_noise_std      = 0.015;            % ~0.86 deg/s 1-sigma (MPU6050 class)
r.gyro_bias_init      = 0.010;            % ~0.57 deg/s initial bias
r.gyro_bias_rw        = 0.005;            % rad/s / sqrt(s) (slow drift)
r.gyro_quant_lsb      = 2*pi/4000;        % 12-bit over +/- 1000 deg/s
r.bias_cal_s          = 0.5;              % pre-flight bias calibration window
r.bias_cal_residual_frac = 0.10;          % 10% of initial bias survives cal
r.sensor_latency_steps= 1;                % 1 frame (~5 ms at 200 Hz)
r.servo_deadband      = 0.05;             % units (~0.4% of u_max)
r.servo_backlash      = 0.10;             % reversal slop
r.servo_pos_noise     = 0.01;             % servo feedback noise
r.gust_std            = 0.30;             % rad/s^2 disturbance sigma
r.gust_tau            = 0.40;             % s correlation
r.keff_drift_rate     = 0.0;              % off by default
r.theta_init_bias     = deg2rad(0.5);
end


function s = apply_defaults(s, defs)
fns = fieldnames(defs);
for i = 1:numel(fns)
    if ~isfield(s, fns{i})
        s.(fns{i}) = defs.(fns{i});
    end
end
end
