%% Actuator Health Monitor — Direction C (STUB)
% ==============================================
% Online estimator of current gimbal natural frequency (wn) from the
% closed-loop command/response residual during flight.
%
% CORE IDEA:
%   The gimbal has a known nominal second-order response:
%     G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
%   When wn degrades (battery drop, friction), the gimbal lags behind
%   its command more than expected. This creates a predictable residual:
%     r(t) = delta_measured(t) - delta_predicted_nominal(t)
%   A sliding-window RLS or MRAS estimator fits wn_est from this residual.
%
% ESTIMATION APPROACH (Model Reference Adaptive System, MRAS):
%   Reference model: nominal gimbal PT2 (wn=36, zeta=0.82)
%   Adaptive model:  same structure, wn_est updated by gradient law
%   Update law:
%     wn_est_dot = -gamma * e * dphi_dwn
%   where e = delta_meas - delta_predicted, gamma = adaptation gain
%
% FAULT DETECTION TRIGGER:
%   If wn_est < wn_threshold (default: 0.65 * wn_nominal = 23.4 rad/s)
%   for more than N_confirm consecutive samples (default: 10 at 200 Hz)
%   => trigger adaptation (call adaptive_retune)
%
% FALSE POSITIVE GUARD:
%   Do NOT trigger during the first 0.5s (startup transient)
%   Do NOT trigger if control input is near zero (no excitation)
%   Require N_confirm consecutive samples (not just one sample)
%
% FUNCTION SIGNATURE (to implement):
%   [wn_est, fault_flag] = actuator_health_monitor(delta_cmd, delta_meas, ...
%                                                   state_prev, params, dt)
%   Returns:
%     wn_est     — current bandwidth estimate (rad/s)
%     fault_flag — 1 if fault confirmed, 0 otherwise
%     state_new  — updated monitor state for next call
%
% TUNING PARAMETERS:
%   wn_nominal    = 36.0   rad/s  (from assumed_gimbal_profile.csv)
%   zeta_nominal  = 0.82          (from assumed_gimbal_profile.csv)
%   wn_threshold  = 23.4   rad/s  (65% of nominal)
%   gamma         = 5.0           (MRAS adaptation gain, tune for latency vs noise)
%   N_confirm     = 10            (samples before triggering, at 200 Hz = 50 ms)
%
% TODO (Phase 3 implementation):
%   1. Implement discrete MRAS estimator
%   2. Add startup guard and excitation check
%   3. Add N_confirm counter
%   4. Test: inject wn=18 at T+3s, confirm detection within 0.5s
%   5. Test: no fault => zero false positives over 10s run

function [wn_est, fault_flag, state_new] = actuator_health_monitor(delta_cmd, delta_meas, state_prev, params, dt)
%ACTUATOR_HEALTH_MONITOR  Sliding-window bandwidth estimator + fault detector.
%
%  METHOD: Normalized tracking error in a sliding window.
%    When wn degrades, the actuator falls behind its command.
%    Normalized error = rms(delta_cmd - delta_meas) / rms(delta_cmd)
%    grows with degradation.  Maps to wn_est via calibrated linear mapping.
%
%    Mapping (derived from PT2 frequency response at controller bandwidth):
%      e_norm_nominal  ~ 0.12  (wn=36, zeta=0.82, at omega_c=4 rad/s)
%      e_norm_degraded ~ 0.30  (wn=18, 50% degradation)
%      wn_est = wn_nominal * max(0.20, 1 - sensitivity * max(0, e_norm - e_norm_floor))
%
%  FAULT TRIGGER: wn_est < wn_threshold for N_confirm consecutive samples.
%  FALSE POSITIVE GUARDS: startup window, minimum excitation check.
%
%  params fields:
%    wn_nominal, wn_threshold, sensitivity, e_norm_floor
%    startup_guard_s, min_excitation_var, N_confirm, window_len

state_new = state_prev;
state_new.t_elapsed = state_prev.t_elapsed + dt;

% Backward-compatible defaults for newly added state fields
if ~isfield(state_prev, 'e_ref'), state_new.e_ref = params.e_norm_floor; end
if ~isfield(state_prev, 'fault_latched'), state_new.fault_latched = 0; end
if ~isfield(state_prev, 'e_norm_lp'), state_new.e_norm_lp = params.e_norm_floor; end
if ~isfield(state_prev, 'wn_lp'), state_new.wn_lp = params.wn_nominal; end

% Shift sliding window buffers
state_new.cmd_buf  = [delta_cmd;  state_prev.cmd_buf(1:end-1)];
state_new.meas_buf = [delta_meas; state_prev.meas_buf(1:end-1)];

% Default: nominal estimate, no fault
wn_est     = params.wn_nominal;
fault_flag = 0;

% Guard: startup transient
if state_new.t_elapsed < params.startup_guard_s
	state_new.wn_est         = wn_est;
	state_new.confirm_count  = 0;
	return;
end

% Guard: insufficient excitation (controller output near zero)
var_cmd = var(state_new.cmd_buf);
if var_cmd < params.min_excitation_var
	state_new.wn_est        = state_prev.wn_est;
	state_new.confirm_count = state_prev.confirm_count;
	wn_est = state_prev.wn_est;
	return;
end

% Normalized tracking error RMS
err_buf = state_new.cmd_buf - state_new.meas_buf;
e_norm  = sqrt(mean(err_buf.^2) / var_cmd);

% Filter residual metric to suppress jitter in wn estimate
alpha_e = exp(-dt / params.tau_e_norm);
state_new.e_norm_lp = alpha_e * state_prev.e_norm_lp + (1 - alpha_e) * e_norm;
e_use = state_new.e_norm_lp;

% Baseline tracking before arming: learn nominal residual level and do not trigger
if state_new.t_elapsed < params.arm_time_s
	e_ref_prev = state_new.e_ref;
	state_new.e_ref = 0.98 * e_ref_prev + 0.02 * e_use;
	state_new.wn_est = params.wn_nominal;
	state_new.confirm_count = 0;
	wn_est = params.wn_nominal;
	state_new.wn_lp = wn_est;
	return;
end

% Slowly adapt baseline after arming (for drift), but not during large excursions
if e_use < state_new.e_ref + params.trigger_margin
	state_new.e_ref = 0.995 * state_new.e_ref + 0.005 * e_use;
end

e_floor = max(params.e_norm_floor, state_new.e_ref + params.trigger_margin);

% Map filtered residual to raw wn estimate
wn_raw = params.wn_nominal * max(0.20, 1 - params.sensitivity * max(0, e_use - e_floor));

% Low-pass and rate-limit wn estimate to avoid jumps (e.g., 37 -> 8)
alpha_wn = exp(-dt / params.tau_wn_est);
wn_lp = alpha_wn * state_prev.wn_lp + (1 - alpha_wn) * wn_raw;
dw_max = params.wn_rate_limit * dt;
wn_est = state_prev.wn_lp + max(-dw_max, min(dw_max, wn_lp - state_prev.wn_lp));
wn_est = max(0.20 * params.wn_nominal, min(1.15 * params.wn_nominal, wn_est));
state_new.wn_lp = wn_est;
state_new.wn_est = wn_est;

% Confirm-counter logic
if (wn_est < params.wn_threshold) && (e_use > state_new.e_ref + params.trigger_margin)
	state_new.confirm_count = state_prev.confirm_count + 1;
else
	state_new.confirm_count = 0;
end

if state_new.confirm_count >= params.N_confirm || state_prev.fault_latched
	fault_flag = 1;
	state_new.fault_latched = 1;
end
end
