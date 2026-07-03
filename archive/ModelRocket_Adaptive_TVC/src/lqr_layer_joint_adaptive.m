function [u_cmd, adapt_new, diag] = lqr_layer_joint_adaptive( ...
    theta, q_meas, u_act_meas, u_act_prev, q_prev, u_cmd_prev, adapt_prev, ctrl, dt)
%LQR_LAYER_JOINT_ADAPTIVE  Online identification of keff AND slew envelope,
%  with bandwidth-matching gain scaling.
%
%  Two parallel estimators:
%    (1) keff_est: same recursive correlation as keff-only adaptive, but
%        GATED OFF when servo is slew-saturated (regressor invalid).
%    (2) slew_est: tracks the actuator's observable peak rate when it is
%        clearly trying to move faster than nominal. Decays slowly back
%        toward nominal when not saturating.
%
%  Bandwidth-matching gain scale:
%    slew_scale = slew_est / slew_nominal     (<= 1 when degraded)
%    keff_scale = keff_nom  / keff_est        (>1 when motor weakens)
%    K_eff = K_nominal * keff_scale * slew_scale
%
%  Rationale: if the actuator's bandwidth is reduced by factor r, the
%  closed-loop bandwidth must shrink by the same factor for the loop to
%  remain in the linear regime the LQR was designed for. Equivalently,
%  this keeps the actuator-rate demand within the available envelope.

adapt_new = adapt_prev;

% --- Initialise persistent state ----------------------------------------
if ~isfield(adapt_new, 't'),          adapt_new.t          = 0; end
if ~isfield(adapt_new, 'keff_est'),   adapt_new.keff_est   = ctrl.keff_nom; end
if ~isfield(adapt_new, 'alpha_lp'),   adapt_new.alpha_lp   = 0; end
if ~isfield(adapt_new, 'S_uu'),       adapt_new.S_uu       = 1.0; end
if ~isfield(adapt_new, 'S_uy'),       adapt_new.S_uy       = ctrl.keff_nom; end
if ~isfield(adapt_new, 'slew_est'),   adapt_new.slew_est   = ctrl.slew_nominal; end
if ~isfield(adapt_new, 'sat_streak'), adapt_new.sat_streak = 0; end
if ~isfield(adapt_new, 'du_obs_lp'),  adapt_new.du_obs_lp  = 0; end
if ~isfield(adapt_new, 'demand_lp'),  adapt_new.demand_lp  = 0; end
if ~isfield(adapt_new, 'disturb_lp'), adapt_new.disturb_lp = 0; end
if ~isfield(adapt_new, 'resid_lp'),   adapt_new.resid_lp   = 0; end
if ~isfield(adapt_new, 'keff_disabled'), adapt_new.keff_disabled = false; end

adapt_new.t = adapt_new.t + dt;

% --- Observe actuator rate (low-pass filtered for noise robustness) ----
% Raw differentiation of noisy u_act would amplify noise by 1/dt.
% Filter the MAGNITUDE so that rapid back-and-forth swings (which still
% indicate active motion) aren't averaged to ~0.
du_obs_inst = (u_act_meas - u_act_prev) / max(1e-6, dt);
lp_alpha    = 0.30;     % EWMA pole; ~3-sample effective window
adapt_new.du_obs_lp = (1 - lp_alpha) * adapt_new.du_obs_lp + lp_alpha * abs(du_obs_inst);
abs_du_obs = adapt_new.du_obs_lp;

% Lag-model demand rate, also LP-smoothed (magnitude)
demand_rate_inst = abs(u_cmd_prev - u_act_prev) / max(1e-3, ctrl.tau_act_assumed);
adapt_new.demand_lp = (1 - lp_alpha) * adapt_new.demand_lp + lp_alpha * demand_rate_inst;
demand_rate = adapt_new.demand_lp;

% Disturbance-decoupled gate input: subtract likely disturbance-driven
% demand so persistent gusts do not trigger false slew-fault commits.
demand_rate_decoupled = max(0, demand_rate - ctrl.gate_disturb_gain * abs(adapt_new.disturb_lp));

% Position-limit guard: avoid classifying command-vs-motion mismatch as
% slew saturation when the actuator is mostly stuck near its position
% limits (common under severe authority loss / high disturbance).
pos_sat_frac = 0.92;
if isfield(ctrl, 'slew_detect_pos_sat_frac')
    pos_sat_frac = ctrl.slew_detect_pos_sat_frac;
end
pos_limited = (abs(u_act_meas) >= pos_sat_frac * ctrl.u_max) && ...
              (abs(u_cmd_prev) >= pos_sat_frac * ctrl.u_max) && ...
              (sign(u_act_meas) == sign(u_cmd_prev));

% Saturation detector with noise-floor margin:
%   require demand to exceed observed by a multiplicative AND additive margin
%   AND require observed motion to be substantial (well above noise floor)
sat_noise_floor = isfield(ctrl, 'sat_noise_floor') * 0;
if isfield(ctrl, 'sat_noise_floor'); sat_noise_floor = ctrl.sat_noise_floor; end
sat_min_motion = ctrl.slew_nominal * 0.10;     % require ~10% of nominal
saturating_inst = (demand_rate_decoupled > 1.20 * abs_du_obs + sat_noise_floor) && ...
                  (abs_du_obs > sat_min_motion) && ...
                  (~pos_limited);

% Persistent-saturation gate: ignore single-sample saturation spikes from
% disturbance transients. Require N_sat consecutive saturated samples.
% Asymmetric: accumulate fast, decay slowly so transient noise drop-outs
% don't reset the latch.
sat_decay = 1;
if isfield(ctrl, 'sat_decay'); sat_decay = ctrl.sat_decay; end
if saturating_inst
    adapt_new.sat_streak = adapt_new.sat_streak + 1;
else
    adapt_new.sat_streak = max(0, adapt_new.sat_streak - sat_decay);
end
saturating = (adapt_new.sat_streak >= ctrl.sat_streak_min);

% --- Estimator (1): keff via correlation, gated by NO saturation --------
qdot_meas = (q_meas - q_prev) / max(1e-6, dt);
alpha_proxy = qdot_meas + ctrl.aero_damp * q_meas;
adapt_new.alpha_lp = (1 - ctrl.alpha_beta) * adapt_new.alpha_lp ...
                   + ctrl.alpha_beta * alpha_proxy;

% Residual proxy for confidence and disturbance-decoupled gating.
resid_inst = adapt_new.alpha_lp - adapt_new.keff_est * u_act_meas;
adapt_new.resid_lp = (1 - ctrl.disturb_alpha) * adapt_new.resid_lp + ctrl.disturb_alpha * abs(resid_inst);
adapt_new.disturb_lp = (1 - ctrl.disturb_alpha) * adapt_new.disturb_lp + ctrl.disturb_alpha * resid_inst;

% Confidence-aware adaptation weight (0..1): low residual + good excitation.
excitation_conf = min(1.0, abs(u_act_meas) / max(ctrl.delta_min, 1e-3));
resid_scale = max(1e-3, abs(adapt_new.alpha_lp) + abs(ctrl.keff_nom * u_act_meas));
resid_norm = adapt_new.resid_lp / resid_scale;
resid_conf = 1.0 / (1.0 + ctrl.conf_resid_gain * resid_norm);
adapt_conf = max(ctrl.conf_min, min(1.0, excitation_conf * resid_conf));

keff_gate = (adapt_new.t >= ctrl.adapt_guard_s) && ...
            (abs(u_act_meas) >= ctrl.delta_min) && ...
            (~saturating);

if keff_gate
    weighted_u2 = (adapt_conf^2) * u_act_meas^2;
    weighted_uy = (adapt_conf^2) * u_act_meas * adapt_new.alpha_lp;
    adapt_new.S_uu = ctrl.lambda_rls * adapt_new.S_uu + weighted_u2;
    adapt_new.S_uy = ctrl.lambda_rls * adapt_new.S_uy ...
                   + weighted_uy;
    raw_keff = adapt_new.S_uy / max(1e-6, adapt_new.S_uu);
    raw_keff = min(ctrl.keff_max, max(ctrl.keff_min, raw_keff));
    beta_eff = ctrl.keff_beta * (ctrl.conf_floor_blend + (1 - ctrl.conf_floor_blend) * adapt_conf);
    adapt_new.keff_est = (1 - beta_eff) * adapt_new.keff_est ...
                       + beta_eff * raw_keff;
end

% --- Estimator (2): slew envelope, updated ONLY when saturating ---------
% When saturating, the observed |du| is the servo's current achievable rate.
% Snap-down on detected saturation (fast response); slow relaxation back
% toward nominal during long unsaturated stretches.
if saturating
    % Hard snap down to observed rate, with small EW blending for noise
    new_se = (1 - ctrl.slew_alpha_sat) * abs_du_obs + ctrl.slew_alpha_sat * adapt_new.slew_est;
    adapt_new.slew_est = min(adapt_new.slew_est, new_se);
else
    % Slow relaxation back to nominal when there is no evidence of degradation
    adapt_new.slew_est = (1 - ctrl.slew_alpha_relax) * adapt_new.slew_est ...
                       + ctrl.slew_alpha_relax * ctrl.slew_nominal;
end
adapt_new.slew_est = max(ctrl.slew_min, min(ctrl.slew_nominal, adapt_new.slew_est));

% --- Bandwidth-matching gain scaling ------------------------------------
% Arbitration: when slew is degrading, the keff regressor is contaminated
% by saturation transients and unreliable. Freeze keff_scale at 1 (use the
% nominal keff_nom) and rely on slew adaptation. Otherwise apply keff scale.
slew_health = adapt_new.slew_est / ctrl.slew_nominal;       % 0..1

% Latch off keff scaling after large excursions; this prevents run-away
% gain reshaping in extreme nonlinear regimes and recovers slew-safe behavior.
if isfield(ctrl, 'keff_disable_theta_rad')
    if abs(theta) >= ctrl.keff_disable_theta_rad
        adapt_new.keff_disabled = true;
    end
end

if slew_health < ctrl.slew_health_keff_freeze || adapt_new.keff_disabled
    keff_scale = 1.0;
else
    scale_min = ctrl.min_scale;
    scale_max = ctrl.max_scale;
    if isfield(ctrl, 'joint_min_scale'), scale_min = ctrl.joint_min_scale; end
    if isfield(ctrl, 'joint_max_scale'), scale_max = ctrl.joint_max_scale; end
    keff_scale = ctrl.keff_nom / max(1e-6, adapt_new.keff_est);
    keff_scale = min(scale_max, max(scale_min, keff_scale));
end

slew_scale = max(ctrl.slew_scale_min, min(1.0, slew_health));

K_eff = ctrl.K_nominal * keff_scale * slew_scale;

% --- State feedback command ---------------------------------------------
x     = [theta; q_meas];
u_cmd = -K_eff * x;

% Safety shield: keep commanded motion inside online identified envelope.
max_cmd_step = ctrl.safety_cmd_slew_frac * adapt_new.slew_est * dt;
u_cmd_slew_limited = min(u_cmd_prev + max_cmd_step, max(u_cmd_prev - max_cmd_step, u_cmd));
shield_slew = abs(u_cmd_slew_limited - u_cmd) > 1e-9;
u_cmd = u_cmd_slew_limited;

% Attitude guard: never command further away once outside guard angle.
shield_attitude = false;
if abs(theta) > ctrl.theta_guard_rad && sign(u_cmd) == sign(theta)
    u_cmd = 0;
    shield_attitude = true;
end

u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));

% --- Diagnostics --------------------------------------------------------
diag.keff_est   = adapt_new.keff_est;
diag.slew_est   = adapt_new.slew_est;
diag.gain_scale = keff_scale * slew_scale;
diag.K_eff      = K_eff;
diag.saturating = saturating;
diag.saturating_inst = saturating_inst;
diag.sat_streak  = adapt_new.sat_streak;
diag.keff_gate   = keff_gate;
diag.keff_scale  = keff_scale;
diag.slew_scale  = slew_scale;
diag.demand_rate = demand_rate;
diag.demand_rate_decoupled = demand_rate_decoupled;
diag.du_obs_lp   = abs_du_obs;
diag.disturb_lp  = adapt_new.disturb_lp;
diag.pos_limited = pos_limited;
diag.confidence  = adapt_conf;
diag.adapt_weight = adapt_conf^2;
diag.keff_disabled = adapt_new.keff_disabled;
diag.shield_slew = shield_slew;
diag.shield_attitude = shield_attitude;
diag.shield_active = shield_slew || shield_attitude;
end
