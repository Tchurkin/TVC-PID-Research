function [u_cmd, state_new, diag] = pid_slew_aware_layer( ...
    theta_meas, u_act_meas, u_act_prev, u_cmd_prev, state_prev, ctrl, dt)
%PID_SLEW_AWARE_LAYER  PID with derivative-preserving slew adaptation.
%
% The controller estimates the delivered actuator slew online from measured
% actuator motion. When rate saturation is detected, it reduces Kp much more
% aggressively than Kd, preserving damping while backing off the command
% stiffness that causes rate-limited divergence.

state_new = state_prev;

if ~isfield(state_new, 'slew_est'),   state_new.slew_est   = ctrl.slew_nominal; end
if ~isfield(state_new, 'sat_streak'), state_new.sat_streak = 0; end
if ~isfield(state_new, 'du_obs_lp'),  state_new.du_obs_lp  = 0; end
if ~isfield(state_new, 'demand_lp'),  state_new.demand_lp  = 0; end

% Observe delivered actuator rate.
lp_alpha = 0.30;
du_obs_inst = (u_act_meas - u_act_prev) / max(1e-6, dt);
state_new.du_obs_lp = (1 - lp_alpha) * state_new.du_obs_lp + lp_alpha * abs(du_obs_inst);
abs_du_obs = state_new.du_obs_lp;

% Estimate demanded rate from the command backlog across the actuator lag.
demand_rate_inst = abs(u_cmd_prev - u_act_prev) / max(1e-3, ctrl.tau_act_assumed);
state_new.demand_lp = (1 - lp_alpha) * state_new.demand_lp + lp_alpha * demand_rate_inst;
demand_rate = state_new.demand_lp;

sat_min_motion = ctrl.slew_nominal * 0.10;
saturating_inst = (demand_rate > 1.20 * abs_du_obs + ctrl.sat_noise_floor) && ...
                  (abs_du_obs > sat_min_motion);

if saturating_inst
    state_new.sat_streak = state_new.sat_streak + 1;
else
    state_new.sat_streak = max(0, state_new.sat_streak - ctrl.sat_decay);
end
saturating = (state_new.sat_streak >= ctrl.sat_streak_min);

% Slew estimator.
if saturating
    new_slew_est = (1 - ctrl.slew_alpha_sat) * abs_du_obs + ctrl.slew_alpha_sat * state_new.slew_est;
    state_new.slew_est = min(state_new.slew_est, new_slew_est);
else
    state_new.slew_est = (1 - ctrl.slew_alpha_relax) * state_new.slew_est ...
                       + ctrl.slew_alpha_relax * ctrl.slew_nominal;
end
state_new.slew_est = max(ctrl.slew_min, min(ctrl.slew_nominal, state_new.slew_est));

% Error and derivative from measured attitude.
e  = -theta_meas;
de = (e - state_prev.e_prev) / dt;

% Freeze / bleed the integrator when rate-limited.
if saturating
    state_new.integrator = ctrl.integrator_bleed * state_prev.integrator;
else
    state_new.integrator = state_prev.integrator + e * dt;
end
state_new.integrator = max(-ctrl.i_lim, min(ctrl.i_lim, state_new.integrator));
state_new.e_prev = e;

% Scale Kp harder than Kd as slew health collapses.
slew_health = state_new.slew_est / ctrl.slew_nominal;
kp_scale = max(ctrl.kp_scale_min, min(1.0, slew_health ^ ctrl.kp_scale_power));
kd_scale = max(ctrl.kd_scale_min, min(1.0, slew_health ^ ctrl.kd_scale_power));

u_raw = (ctrl.Kp * kp_scale) * e + ctrl.Ki * state_new.integrator + (ctrl.Kd * kd_scale) * de;

% Apply the same style of lightweight command shield used in the adaptive LQRs.
max_cmd_step = ctrl.safety_cmd_slew_frac * state_new.slew_est * dt;
u_shaped = min(u_cmd_prev + max_cmd_step, max(u_cmd_prev - max_cmd_step, u_raw));
shield_slew = abs(u_shaped - u_raw) > 1e-9;
u_cmd = u_shaped;

shield_attitude = false;
if abs(theta_meas) > ctrl.theta_guard_rad && sign(u_cmd) == sign(theta_meas)
    u_cmd = 0;
    shield_attitude = true;
end

u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));

if nargout >= 3
    diag.slew_est        = state_new.slew_est;
    diag.saturating      = saturating;
    diag.saturating_inst = saturating_inst;
    diag.sat_streak      = state_new.sat_streak;
    diag.kp_scale        = kp_scale;
    diag.kd_scale        = kd_scale;
    diag.demand_rate     = demand_rate;
    diag.du_obs_lp       = abs_du_obs;
    diag.shield_slew     = shield_slew;
    diag.shield_attitude = shield_attitude;
    diag.shield_active   = shield_slew || shield_attitude;
end
end