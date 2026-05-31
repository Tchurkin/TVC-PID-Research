function [u_cmd, adapt_new, diag] = lqr_layer_pch_lqr( ...
    theta, q_meas, u_act_meas, q_prev, adapt_prev, ctrl, dt)
%LQR_LAYER_PCH_LQR  LQR with Pseudo-Control Hedging on the actuator rate.
%
%  Pseudo-Control Hedging (Johnson & Calise, 2000-2003; flown on X-36 and
%  GTMax). Standard fix for adaptive control with rate-limited actuators:
%  the control command is hedged so that the controller never asks for
%  more than the (assumed-known) actuator can deliver, and the adaptive
%  law is gated off when the hedge is active so it never trains on data
%  produced inside the saturation nonlinearity.
%
%  Implementation (state-feedback variant):
%    1. Compute nominal LQR command   u_raw = -K_eff * x
%    2. Compute demanded actuator rate du_dem = (u_raw - u_act) / tau_act
%    3. If |du_dem| > slew_max_assumed, clip:
%         du_clip = sign(du_dem) * slew_max_assumed
%         u_cmd  = u_act + tau_act * du_clip
%    4. Adaptive update gated OFF when hedge fires.
%
%  This is the right baseline for any reviewer who asks "why don't you
%  just use anti-windup / PCH?". CRITICAL: PCH assumes the actuator
%  envelope (slew_max_assumed, tau_act_assumed) is KNOWN. When the true
%  envelope drops below the assumed value, PCH continues to hedge to the
%  wrong (too-fast) limit and provides no protection. That is exactly the
%  regime where the joint-adaptive controller's online identification
%  pays off.

adapt_new = adapt_prev;

if ~isfield(adapt_new, 't'),         adapt_new.t        = 0; end
if ~isfield(adapt_new, 'keff_est'),  adapt_new.keff_est = ctrl.keff_nom; end
if ~isfield(adapt_new, 'alpha_lp'),  adapt_new.alpha_lp = 0; end

adapt_new.t = adapt_new.t + dt;

% --- Adaptive law (sigma-mod gradient, gated by hedge) -----------------
qdot_meas   = (q_meas - q_prev) / max(1e-6, dt);
alpha_proxy = qdot_meas + ctrl.aero_damp * q_meas;
adapt_new.alpha_lp = (1 - ctrl.alpha_beta) * adapt_new.alpha_lp ...
                   + ctrl.alpha_beta * alpha_proxy;

% --- Compute raw LQR command -------------------------------------------
gain_scale = ctrl.keff_nom / max(1e-6, adapt_new.keff_est);
gain_scale = min(ctrl.max_scale, max(ctrl.min_scale, gain_scale));
K_eff = ctrl.K_nominal * gain_scale;

x     = [theta; q_meas];
u_raw = -K_eff * x;
u_raw = max(-ctrl.u_max, min(ctrl.u_max, u_raw));

% --- Pseudo-control hedge on actuator rate -----------------------------
du_demand = (u_raw - u_act_meas) / max(1e-3, ctrl.tau_act_assumed);
hedge_active = abs(du_demand) > ctrl.slew_max_assumed;

if hedge_active
    du_clipped = sign(du_demand) * ctrl.slew_max_assumed;
    u_cmd = u_act_meas + ctrl.tau_act_assumed * du_clipped;
    u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));
else
    u_cmd = u_raw;
end

% --- Adaptive update gated OFF during hedge ----------------------------
if (adapt_new.t >= ctrl.adapt_guard_s) && (~hedge_active)
    pred_err  = adapt_new.alpha_lp - adapt_new.keff_est * u_act_meas;
    grad_term = ctrl.gamma_grad * u_act_meas * pred_err;
    leak_term = ctrl.sigma_mod * (adapt_new.keff_est - ctrl.keff_nom);
    adapt_new.keff_est = adapt_new.keff_est + dt * (grad_term - leak_term);
    adapt_new.keff_est = min(ctrl.keff_max, max(ctrl.keff_min, adapt_new.keff_est));
end

diag.keff_est    = adapt_new.keff_est;
diag.gain_scale  = gain_scale;
diag.K_eff       = K_eff;
diag.hedge       = hedge_active;
diag.slew_est    = ctrl.slew_max_assumed;  % constant: PCH does not identify
end
