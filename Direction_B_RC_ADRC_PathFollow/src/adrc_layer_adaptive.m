function [u_cmd, z_new, adapt_new, diag] = adrc_layer_adaptive(y_ct, u_act_meas, z_prev, u_prev, adapt_prev, params, dt)
%ADRC_LAYER_ADAPTIVE ADRC with online adaptation from actuator RLS estimates.

adapt_new = adapt_prev;
if ~isfield(adapt_new, 't'), adapt_new.t = 0; end
if ~isfield(adapt_new, 'omega_o_eff'), adapt_new.omega_o_eff = params.omega_o_nom; end
if ~isfield(adapt_new, 'gain_est'), adapt_new.gain_est = params.gain_nom; end
if ~isfield(adapt_new, 'tau_est'), adapt_new.tau_est = params.tau_nom; end
if ~isfield(adapt_new, 'rls') || isempty(adapt_new.rls)
    adapt_new.rls = struct('theta', [0.90; 0.08], 'P', 50 * eye(2));
end
if ~isfield(adapt_new, 'u_act_prev'), adapt_new.u_act_prev = 0; end
if ~isfield(adapt_new, 'u_cmd_prev'), adapt_new.u_cmd_prev = 0; end
if ~isfield(adapt_new, 'has_prev'), adapt_new.has_prev = false; end

adapt_new.t = adapt_new.t + dt;

pred_err = 0;
if adapt_new.has_prev
    [adapt_new.rls, rls_diag] = rls_estimator(u_act_meas, adapt_prev.u_act_prev, adapt_prev.u_cmd_prev, adapt_prev.rls, params, dt);
    adapt_new.gain_est = rls_diag.gain_est;
    adapt_new.tau_est = rls_diag.tau_est;
    pred_err = rls_diag.pred_err;
end

tau_ratio = params.tau_nom / max(1e-3, adapt_new.tau_est);
gain_ratio = adapt_new.gain_est / max(1e-3, params.gain_nom);
omega_target = params.omega_o_nom * sqrt(max(0.25, min(2.5, gain_ratio))) * sqrt(max(0.25, min(2.5, tau_ratio)));

if adapt_new.t < params.adapt_guard_s
    omega_target = params.omega_o_nom;
end

omega_step = params.omega_o_rate * dt;
adapt_new.omega_o_eff = adapt_new.omega_o_eff + max(-omega_step, min(omega_step, omega_target - adapt_new.omega_o_eff));

l1 = 3 * adapt_new.omega_o_eff;
l2 = 3 * adapt_new.omega_o_eff^2;
l3 = adapt_new.omega_o_eff^3;

% Controller bandwidth scaling reduces aggressiveness when actuator authority
% drops or lag increases, while preserving enough authority for recovery.
if ~isfield(params, 'tau_exp'), params.tau_exp = 0.55; end
if ~isfield(params, 'gain_exp'), params.gain_exp = 0.35; end
if ~isfield(params, 'bw_floor'), params.bw_floor = 0.35; end
bw_core = (max(0.20, min(2.0, tau_ratio))^params.tau_exp) * (max(0.20, min(2.0, gain_ratio))^params.gain_exp);
bw_scale = max(params.bw_floor, min(1.60, bw_core));
if adapt_new.t < params.adapt_guard_s
    bw_scale = 1.0;
end
kp_eff = params.Kp * bw_scale^2;
kd_eff = params.Kd * bw_scale;

b0_eff = params.b0_nominal * max(0.35, min(1.8, adapt_new.gain_est));
if adapt_new.t < params.adapt_guard_s
    b0_eff = params.b0_nominal;
end

% ESO update
err_y = y_ct - z_prev(1);
z1 = z_prev(1) + dt * (z_prev(2) + l1 * err_y);
z2 = z_prev(2) + dt * (z_prev(3) + b0_eff * u_prev + l2 * err_y);
z3 = z_prev(3) + dt * (l3 * err_y);
z_new = [z1; z2; z3];

u0 = kp_eff * (0 - z1) + kd_eff * (0 - z2);
u_cmd = (u0 - z3) / max(1e-6, b0_eff);
u_cmd = max(-params.u_max, min(params.u_max, u_cmd));

diag = struct();
diag.omega_o_eff = adapt_new.omega_o_eff;
diag.gain_est = adapt_new.gain_est;
diag.tau_est = adapt_new.tau_est;
diag.b0_eff = b0_eff;
diag.kp_eff = kp_eff;
diag.kd_eff = kd_eff;
diag.bw_scale = bw_scale;
diag.pred_err = pred_err;

% Store latest signals for next-step RLS update.
adapt_new.u_act_prev = u_act_meas;
adapt_new.u_cmd_prev = u_cmd;
adapt_new.has_prev = true;
end
