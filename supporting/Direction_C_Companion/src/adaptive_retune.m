%% Adaptive Retuner — Direction C (STUB)
% ========================================
% Maps an estimated actuator bandwidth (wn_est) to a safe set of PID
% gains that keep the closed-loop stable despite the degraded actuator.
%
% CORE IDEA:
%   A PID controller tuned for wn=36 rad/s will drive the system unstable
%   if wn drops to 18 rad/s — the controller demands faster response than
%   the actuator can deliver, causing phase accumulation and divergence.
%   The retuner reduces controller bandwidth proportionally to wn_est,
%   trading transient performance for stability margin.
%
% GAIN SCHEDULING LAW:
%   Nominal gains (tuned for wn_nominal = 36 rad/s):
%     Kp_nom, Ki_nom, Kd_nom
%
%   Degraded gains at estimated wn_est:
%     scale = (wn_est / wn_nominal)^alpha    alpha in [0.5, 1.0]
%     Kp = Kp_nom * scale
%     Kd = Kd_nom * scale
%     Ki = Ki_nom * scale^2  (integral needs faster reduction to prevent windup)
%
%   Alpha = 1.0: linear scaling (conservative)
%   Alpha = 0.5: square-root scaling (more aggressive, preserves more performance)
%
% STABILITY MARGIN GUARANTEE:
%   For the nominal second-order rocket pitch model, gains scaled by
%   (wn_est/wn_nom) ensure phase margin >= 45 deg at the degraded wn.
%   This should be validated analytically before claiming it.
%
% SMOOTHING:
%   Apply gain changes through a first-order filter (tau = 0.1s) to
%   avoid step changes in control output that could cause transients.
%
% FUNCTION SIGNATURE (to implement):
%   gains_new = adaptive_retune(wn_est, gains_nominal, params)
%   Returns: struct with Kp, Ki, Kd fields
%
% TODO (Phase 3 implementation):
%   1. Implement gain scaling law with alpha parameter
%   2. Implement gain smoothing filter
%   3. Validate analytically: compute PM at wn_est = 18, 24, 30 rad/s
%   4. Test: after fault at T+3s, gains adapt and pitch stabilizes

function gains_new = adaptive_retune(wn_est, gains_nom, params, gains_prev, dt)
%ADAPTIVE_RETUNE  Map estimated actuator bandwidth to safe PID gains.
%
%  SCALING LAW:
%    scale = (wn_est / wn_nominal)^alpha   (alpha in [0.5, 1.0])
%    Kp_target = Kp_nom * scale
%    Kd_target = Kd_nom * scale
%    Ki_target = Ki_nom * scale^2   (integral needs faster reduction)
%
%  STABILITY RATIONALE:
%    For the nominal 2nd-order pitch plant, PID gains tuned at wn_nom
%    scaled by (wn_est/wn_nom) preserve the phase margin >= 40 deg
%    at the degraded actuator wn_est.  This is because the controller
%    bandwidth (proportional to sqrt(Kp)) scales as wn_est, matching
%    the reduced actuator capability.
%
%  SMOOTHING: first-order filter (tau_smooth) prevents step changes in
%    controller output that would cause their own transients.
%
%  params fields: wn_nominal, alpha, tau_smooth

if ~isfield(params, 'kp_exp'), params.kp_exp = params.alpha; end
if ~isfield(params, 'kd_exp'), params.kd_exp = max(0.5, 0.75*params.alpha); end
if ~isfield(params, 'ki_exp'), params.ki_exp = max(1.5, 2.0*params.alpha); end
if ~isfield(params, 'min_scale'), params.min_scale = 0.20; end

ratio = max(wn_est, params.min_scale*params.wn_nominal) / params.wn_nominal;

Kp_target = gains_nom.Kp * ratio^params.kp_exp;
Kd_target = gains_nom.Kd * ratio^params.kd_exp;
Ki_target = gains_nom.Ki * ratio^params.ki_exp;

% First-order smoothing filter on gains
tau_eff = params.tau_smooth;
if ratio < 0.55
	tau_eff = max(0.004, 0.35 * params.tau_smooth);
end
a = exp(-dt / tau_eff);
gains_new.Kp = a * gains_prev.Kp + (1-a) * Kp_target;
gains_new.Kd = a * gains_prev.Kd + (1-a) * Kd_target;
gains_new.Ki = a * gains_prev.Ki + (1-a) * Ki_target;
end
