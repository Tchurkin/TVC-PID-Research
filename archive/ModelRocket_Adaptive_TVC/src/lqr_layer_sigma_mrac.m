function [u_cmd, adapt_new, diag] = lqr_layer_sigma_mrac( ...
    theta, q_meas, u_act_meas, q_prev, adapt_prev, ctrl, dt)
%LQR_LAYER_SIGMA_MRAC  Gradient MRAC of keff with sigma-modification leakage.
%
%  Standard adaptive-control baseline (Ioannou & Sun 1996; Narendra &
%  Annaswamy 1989). Replaces the bare RLS estimator used in the
%  ADAPTIVE_KEFF_LQR baseline with a gradient law plus a leakage term that
%  prevents parameter drift when the regressor is poorly excited or
%  contaminated (e.g., during actuator saturation):
%
%      d/dt keff_est = gamma * u_act * ( alpha - keff_est * u_act )
%                      - sigma * ( keff_est - keff_nom )
%
%  The sigma term is the canonical fix for the "bursting" / drift problem
%  in classical MRAC. It does NOT solve the slew-rate failure mode -- it
%  just keeps the parameter from running away. This is exactly the
%  comparison we want: a properly-implemented single-parameter adaptive
%  controller, not a deliberately fragile one.
%
%  Gain rescaling identical to ADAPTIVE_KEFF_LQR:
%      K_eff = K_nominal * (keff_nom / keff_est)

adapt_new = adapt_prev;

if ~isfield(adapt_new, 't'),         adapt_new.t        = 0; end
if ~isfield(adapt_new, 'keff_est'),  adapt_new.keff_est = ctrl.keff_nom; end
if ~isfield(adapt_new, 'alpha_lp'),  adapt_new.alpha_lp = 0; end

adapt_new.t = adapt_new.t + dt;

% Angular acceleration proxy (same as keff-adaptive baseline)
qdot_meas   = (q_meas - q_prev) / max(1e-6, dt);
alpha_proxy = qdot_meas + ctrl.aero_damp * q_meas;
adapt_new.alpha_lp = (1 - ctrl.alpha_beta) * adapt_new.alpha_lp ...
                   + ctrl.alpha_beta * alpha_proxy;

% Gradient law with sigma-modification leakage
if adapt_new.t >= ctrl.adapt_guard_s
    pred_err = adapt_new.alpha_lp - adapt_new.keff_est * u_act_meas;
    grad_term = ctrl.gamma_grad * u_act_meas * pred_err;
    leak_term = ctrl.sigma_mod * (adapt_new.keff_est - ctrl.keff_nom);
    adapt_new.keff_est = adapt_new.keff_est + dt * (grad_term - leak_term);
    adapt_new.keff_est = min(ctrl.keff_max, max(ctrl.keff_min, adapt_new.keff_est));
end

% Linear gain rescaling
gain_scale = ctrl.keff_nom / max(1e-6, adapt_new.keff_est);
gain_scale = min(ctrl.max_scale, max(ctrl.min_scale, gain_scale));
K_eff = ctrl.K_nominal * gain_scale;

x     = [theta; q_meas];
u_cmd = -K_eff * x;
u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));

diag.keff_est   = adapt_new.keff_est;
diag.gain_scale = gain_scale;
diag.K_eff      = K_eff;
end
