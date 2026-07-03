function [u_cmd, adapt_new, diag] = lqr_layer_slew_adaptive( ...
    theta, q_meas, u_act_meas, u_act_prev, u_cmd_prev, adapt_prev, ctrl, dt)
%LQR_LAYER_SLEW_ADAPTIVE  Slew-only adaptive LQR scaling baseline.
%
% Purpose:
%   Isolate actuator-envelope adaptation without keff identification.
%   This is the clean single-channel comparator against JOINT_ADAPTIVE.

adapt_new = adapt_prev;

if ~isfield(adapt_new, 'slew_est'),   adapt_new.slew_est   = ctrl.slew_nominal; end
if ~isfield(adapt_new, 'sat_streak'), adapt_new.sat_streak = 0; end
if ~isfield(adapt_new, 'du_obs_lp'),  adapt_new.du_obs_lp  = 0; end
if ~isfield(adapt_new, 'demand_lp'),  adapt_new.demand_lp  = 0; end

% Observe actuator rate
lp_alpha = 0.30;
du_obs_inst = (u_act_meas - u_act_prev) / max(1e-6, dt);
adapt_new.du_obs_lp = (1 - lp_alpha) * adapt_new.du_obs_lp + lp_alpha * abs(du_obs_inst);
abs_du_obs = adapt_new.du_obs_lp;

% Demand rate proxy
demand_rate_inst = abs(u_cmd_prev - u_act_prev) / max(1e-3, ctrl.tau_act_assumed);
adapt_new.demand_lp = (1 - lp_alpha) * adapt_new.demand_lp + lp_alpha * demand_rate_inst;
demand_rate = adapt_new.demand_lp;

sat_noise_floor = 0;
if isfield(ctrl, 'sat_noise_floor'), sat_noise_floor = ctrl.sat_noise_floor; end
sat_min_motion = ctrl.slew_nominal * 0.10;

saturating_inst = (demand_rate > 1.20 * abs_du_obs + sat_noise_floor) && ...
                  (abs_du_obs > sat_min_motion);

sat_decay = 1;
if isfield(ctrl, 'sat_decay'), sat_decay = ctrl.sat_decay; end
if saturating_inst
    adapt_new.sat_streak = adapt_new.sat_streak + 1;
else
    adapt_new.sat_streak = max(0, adapt_new.sat_streak - sat_decay);
end
saturating = (adapt_new.sat_streak >= ctrl.sat_streak_min);

% Slew estimator
if saturating
    new_se = (1 - ctrl.slew_alpha_sat) * abs_du_obs + ctrl.slew_alpha_sat * adapt_new.slew_est;
    adapt_new.slew_est = min(adapt_new.slew_est, new_se);
else
    adapt_new.slew_est = (1 - ctrl.slew_alpha_relax) * adapt_new.slew_est ...
                       + ctrl.slew_alpha_relax * ctrl.slew_nominal;
end
adapt_new.slew_est = max(ctrl.slew_min, min(ctrl.slew_nominal, adapt_new.slew_est));

% Slew-only scaling
slew_health = adapt_new.slew_est / ctrl.slew_nominal;
slew_scale = max(ctrl.slew_scale_min, min(1.0, slew_health));
K_eff = ctrl.K_nominal * slew_scale;

% State feedback
x = [theta; q_meas];
u_cmd = -K_eff * x;

% Same safety shields as JOINT
max_cmd_step = ctrl.safety_cmd_slew_frac * adapt_new.slew_est * dt;
u_cmd_slew_limited = min(u_cmd_prev + max_cmd_step, max(u_cmd_prev - max_cmd_step, u_cmd));
shield_slew = abs(u_cmd_slew_limited - u_cmd) > 1e-9;
u_cmd = u_cmd_slew_limited;

shield_attitude = false;
if abs(theta) > ctrl.theta_guard_rad && sign(u_cmd) == sign(theta)
    u_cmd = 0;
    shield_attitude = true;
end

u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));

diag.slew_est = adapt_new.slew_est;
diag.saturating = saturating;
diag.saturating_inst = saturating_inst;
diag.sat_streak = adapt_new.sat_streak;
diag.slew_scale = slew_scale;
diag.gain_scale = slew_scale;
diag.demand_rate = demand_rate;
diag.du_obs_lp = abs_du_obs;
diag.shield_slew = shield_slew;
diag.shield_attitude = shield_attitude;
diag.shield_active = shield_slew || shield_attitude;
end
