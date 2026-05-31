function out = simulate_case(controller_name, sc, cfg, seed)
%SIMULATE_CASE  1-DOF TVC pitch simulation.
%
%  Plant:   theta_dot = q
%           q_dot     = keff(t) * u_act - aero_damp(t) * q + disturbance
%  Actuator (truth, not modelled in controllers):
%           tau_act * u_act_dot = u_cmd - u_act    (first-order lag)
%
%  Supported controllers:
%    FIXED_PID          -- pure-P baseline, unstable when keff spikes
%    FIXED_LQR          -- 2-state LQR (ignores actuator), unstable at fault
%    ADAPTIVE_KEFF_LQR  -- online keff identification + linear gain rescaling

if nargin < 4, seed = 1; end
rng(seed);

dt = cfg.dt;
t  = (0:dt:sc.t_end)';
N  = numel(t);

theta   = zeros(N,1);
q       = zeros(N,1);
u_act   = zeros(N,1);
u_cmd   = zeros(N,1);
keff_est_log = nan(N,1);
slew_est_log = nan(N,1);
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

theta(1) = cfg.plant.theta0 + deg2rad(0.1)*randn();
q(1)     = cfg.plant.q0;
u_act(1) = cfg.plant.u_act0;

ctrl_name = upper(string(controller_name));
ctrl      = cfg.controllers.(ctrl_name);

% Controller state
pid_state = struct('integrator', 0, 'e_prev', -theta(1));
adapt     = struct();

for k = 2:N
    tk = t(k);

    % Plant parameters (may change at fault time)
    keff     = cfg.plant.control_eff;
    aero_damp = cfg.plant.aero_damp;
    tau_act  = cfg.plant.tau_act;
    slew_max = cfg.plant.slew_max;

    if tk >= sc.fault_time
        keff      = keff      * sc.control_eff_scale_post;
        aero_damp = aero_damp * sc.aero_damp_scale_post;
        tau_act   = tau_act   * sc.tau_scale_post;
        slew_max  = slew_max  * sc.slew_scale_post;
    end

    disturb = sc.disturbance_amp * sin(2*pi*sc.disturbance_freq_hz*tk);
    if tk >= sc.fault_time
        disturb = sc.disturb_scale_post * disturb + sc.disturb_bias_post;
    end

    % ---- Controller ------------------------------------------------
    switch ctrl_name
        case "FIXED_PID"
            [u, pid_state] = pid_layer(theta(k-1), pid_state, ctrl, dt);
            u_cmd(k) = u;

        case "FIXED_LQR"
            x = [theta(k-1); q(k-1)];
            u_cmd(k) = max(-ctrl.u_max, min(ctrl.u_max, -ctrl.K * x));

        case "ADAPTIVE_KEFF_LQR"
            q_prev = q(max(1, k-2));
            [u, adapt, d] = lqr_layer_keff_adaptive( ...
                theta(k-1), q(k-1), u_act(k-1), q_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            keff_est_log(k)   = d.keff_est;
            gain_scale_log(k) = d.gain_scale;

        case "SLEW_ADAPTIVE"
            uact_prev = u_act(max(1, k-2));
            ucmd_prev = u_cmd(max(1, k-1));
            [u, adapt, d] = lqr_layer_slew_adaptive( ...
                theta(k-1), q(k-1), u_act(k-1), uact_prev, ucmd_prev, adapt, ctrl, dt);
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

        case "JOINT_ADAPTIVE"
            q_prev    = q(max(1, k-2));
            uact_prev = u_act(max(1, k-2));
            ucmd_prev = u_cmd(max(1, k-1));
            [u, adapt, d] = lqr_layer_joint_adaptive( ...
                theta(k-1), q(k-1), u_act(k-1), uact_prev, q_prev, ucmd_prev, adapt, ctrl, dt);
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

        case "SIGMA_MRAC"
            q_prev = q(max(1, k-2));
            [u, adapt, d] = lqr_layer_sigma_mrac( ...
                theta(k-1), q(k-1), u_act(k-1), q_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            keff_est_log(k)   = d.keff_est;
            gain_scale_log(k) = d.gain_scale;

        case "PCH_LQR"
            q_prev = q(max(1, k-2));
            [u, adapt, d] = lqr_layer_pch_lqr( ...
                theta(k-1), q(k-1), u_act(k-1), q_prev, adapt, ctrl, dt);
            u_cmd(k) = u;
            keff_est_log(k)   = d.keff_est;
            slew_est_log(k)   = d.slew_est;   % constant nominal for PCH
            gain_scale_log(k) = d.gain_scale;

        otherwise
            error('simulate_case:UnknownController', ...
                'Unknown controller: %s', ctrl_name);
    end

    % Startup scheduling: ramp control authority to avoid launch transient ringing.
    if tk < sc.fault_time && isfield(sc, 'startup_ramp_s') && sc.startup_ramp_s > 0
        startup_scale = min(1.0, tk / sc.startup_ramp_s);
        u_cmd(k) = startup_scale * u_cmd(k);
    end

    % ---- Actuator (1st-order lag, slew-limited) --------------------
    du_cmd  = (u_cmd(k) - u_act(k-1)) / max(1e-5, tau_act);
    du      = max(-slew_max, min(slew_max, du_cmd));
    u_act(k) = u_act(k-1) + dt * du;
    u_act(k) = max(-cfg.plant.u_max, min(cfg.plant.u_max, u_act(k)));

    % ---- Plant dynamics --------------------------------------------
    qdot  = keff * u_act(k) - aero_damp * q(k-1) + disturb;
    q(k)  = q(k-1) + dt * qdot;
    theta(k) = theta(k-1) + dt * q(k);
end

out.time          = t;
out.theta         = theta;
out.q             = q;
out.u_cmd         = u_cmd;
out.u_act         = u_act;
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
