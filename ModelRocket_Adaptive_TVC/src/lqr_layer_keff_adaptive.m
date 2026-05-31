function [u_cmd, adapt_new, diag] = lqr_layer_keff_adaptive( ...
    theta, q_meas, u_act_meas, q_prev, adapt_prev, ctrl, dt)
%LQR_LAYER_KEFF_ADAPTIVE  Online keff identification + linear gain rescaling.
%
%  Model used for identification:
%    alpha(k) ~= keff * u_act(k)
%    where alpha = q_dot + aero_damp*q  (angular accel proxy)
%
%  Gain scaling rule (linear, not sqrt):
%    K_eff = K_nominal * (keff_nom / keff_est)
%
%  This keeps the effective loop gain  keff_actual * K_eff  near its
%  nominal design value regardless of what keff_actual does.

adapt_new = adapt_prev;

% Initialise persistent state
if ~isfield(adapt_new, 't'),         adapt_new.t       = 0; end
if ~isfield(adapt_new, 'keff_est'),  adapt_new.keff_est = ctrl.keff_nom; end
if ~isfield(adapt_new, 'alpha_lp'),  adapt_new.alpha_lp = 0; end
if ~isfield(adapt_new, 'S_uu'),      adapt_new.S_uu    = 1.0; end
if ~isfield(adapt_new, 'S_uy'),      adapt_new.S_uy    = ctrl.keff_nom; end

adapt_new.t = adapt_new.t + dt;

% Angular acceleration proxy: alpha = q_dot + aero_damp * q
qdot_meas      = (q_meas - q_prev) / max(1e-6, dt);
alpha_proxy    = qdot_meas + ctrl.aero_damp * q_meas;
adapt_new.alpha_lp = (1 - ctrl.alpha_beta) * adapt_new.alpha_lp ...
                   + ctrl.alpha_beta * alpha_proxy;

% Recursive correlation update (forgetting-factor RLS on scalar keff)
if adapt_new.t >= ctrl.adapt_guard_s && abs(u_act_meas) >= ctrl.delta_min
    adapt_new.S_uu = ctrl.lambda_rls * adapt_new.S_uu + u_act_meas^2;
    adapt_new.S_uy = ctrl.lambda_rls * adapt_new.S_uy ...
                   + u_act_meas * adapt_new.alpha_lp;
    raw_keff = adapt_new.S_uy / max(1e-6, adapt_new.S_uu);
    raw_keff = min(ctrl.keff_max, max(ctrl.keff_min, raw_keff));
    adapt_new.keff_est = (1 - ctrl.keff_beta) * adapt_new.keff_est ...
                       + ctrl.keff_beta * raw_keff;
end

% Linear gain rescaling: keeps keff_actual * K_eff ~= keff_nom * K_nominal
gain_scale = ctrl.keff_nom / max(1e-6, adapt_new.keff_est);
gain_scale = min(ctrl.max_scale, max(ctrl.min_scale, gain_scale));
K_eff = ctrl.K_nominal * gain_scale;

% State feedback command
x     = [theta; q_meas];
u_cmd = -K_eff * x;

% Persistent excitation dither for keff observability
if isfield(ctrl, 'id_dither_amp') && ctrl.id_dither_amp > 0
    u_cmd = u_cmd + ctrl.id_dither_amp * sin(2*pi*ctrl.id_dither_hz * adapt_new.t);
end

u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));

diag.keff_est   = adapt_new.keff_est;
diag.gain_scale = gain_scale;
diag.K_eff      = K_eff;
end
