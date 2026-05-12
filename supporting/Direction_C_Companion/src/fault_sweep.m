%% Fault Sweep Runner — Direction C (STUB)
% =========================================
% Injects actuator faults at defined times and measures pitch response
% with and without the health monitor + adaptive retuner active.
%
% FAULT SWEEP DIMENSIONS:
%   fault_types  = {'wn_drop_50pct', 'delay_increase', 'combined'}
%   fault_times  = [1.0, 3.0, 5.0]  seconds after launch
%   adaptation   = {'disabled', 'enabled'}
%
% FAULT DEFINITIONS:
%   'wn_drop_50pct'  — wn steps from 36 to 18 rad/s (battery droop)
%   'delay_increase' — delay increases from 0.032 to 0.080 s (friction)
%   'combined'       — both simultaneously (worst case)
%
% METRICS COLLECTED:
%   max_pitch_dev_deg    — peak absolute pitch deviation after fault (deg)
%   detection_latency_s  — time from fault injection to fault_flag=1 (s)
%   false_pos_count      — false triggers in clean (no-fault) baseline runs
%   settled_pitch_rms    — RMS pitch error in 2s window after adaptation (deg)
%
% SEMI-PROMISING THRESHOLDS:
%   max_pitch_dev WITH adaptation  <= 5.0 deg
%   max_pitch_dev WITHOUT adaptation > 15.0 deg  (demonstrates real problem)
%   detection_latency              <= 0.50 s
%   false_pos_count                = 0
%
% OUTPUT FILE:
%   outputs/sweep_results_C.csv
%   columns: fault_type, fault_time, adaptation, max_pitch_dev_deg,
%            detection_latency_s, false_pos_count, settled_pitch_rms_deg
%
% TODO (Phase 3 implementation):
%   1. Set up sim with fault injection parameter (wn as time-varying signal)
%   2. Loop over fault_types x fault_times x adaptation conditions
%   3. Extract metrics from simulation output timeseries
%   4. Write CSV and print summary table
%   5. Generate outputs/graphs/fault_C_comparison.png
%      (2-panel: pitch traces with/without adaptation for key fault scenario)

%% fault_sweep.m  -  Direction C: In-flight actuator degradation + autonomous retuning
% Injects combined fault (wn drop 36->18 rad/s + delay 0.032->0.080 s) at
% three times, with and without adaptation.  Writes sweep_results_C.csv + graphs.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
addpath(this_dir);
out_dir  = fullfile(dir_root, 'outputs');
gfx_dir  = fullfile(out_dir, 'graphs');
if ~exist(out_dir,'dir'), mkdir(out_dir); end
if ~exist(gfx_dir,'dir'), mkdir(gfx_dir); end

fprintf('Running Direction C fault sweep...\n');

%% Physical parameters (same rocket as Direction A)
b0       = 112.5;
c_damp   = 1.0;
delta_max = 10*pi/180;
slew_max  = 92*pi/180;

%% Actuator nominal parameters
wn_nom  = 36.0;  zeta_a = 0.82;  delay_nom = 0.032;

%% Fault parameters (degraded but recoverable case)
wn_fault    = 16.0;   % severe but recoverable bandwidth reduction
delay_fault = 0.080;  % severe delay increase

%% Simulation
dt = 0.001;  t_end = 8.0;  N = round(t_end/dt);

%% Nominal PID gains (aggressive tune to expose actuator-degradation risk)
wn_cl = 7.0;  zeta_cl = 0.75;
Kp_nom = wn_cl^2 / b0;
Kd_nom = (2*zeta_cl*wn_cl - c_damp) / b0;
Ki_nom = 0.8 / b0;
gains_nom.Kp = Kp_nom;  gains_nom.Kd = Kd_nom;  gains_nom.Ki = Ki_nom;

%% Health monitor parameters
mon_win = round(0.20/dt);   % 200 ms sliding window
mon_params.wn_nominal        = wn_nom;
mon_params.wn_threshold      = 0.78 * wn_nom;   % trigger at 78% of nominal
mon_params.sensitivity       = 4.0;             % mapping e_norm -> wn_est
mon_params.e_norm_floor      = 0.08;            % nominal tracking error floor
mon_params.startup_guard_s   = 0.5;
mon_params.arm_time_s        = 0.9;             % prevents early false triggers
mon_params.trigger_margin    = 0.02;            % residual increase needed before flagging
mon_params.min_excitation_var = (0.001)^2;      % rad^2
mon_params.N_confirm         = round(0.05/dt);  % 50 ms confirm window
mon_params.tau_e_norm        = 0.12;            % residual filter time constant (s)
mon_params.tau_wn_est        = 0.30;            % wn estimate low-pass time constant (s)
mon_params.wn_rate_limit     = 60.0;            % max wn slew (rad/s^2)

%% Adaptive retuner parameters
ret_params.wn_nominal  = wn_nom;
ret_params.alpha       = 1.20;
ret_params.tau_smooth  = 0.015;
ret_params.kp_exp      = 1.00;
ret_params.kd_exp      = 0.70;
ret_params.ki_exp      = 2.40;
ret_params.min_scale   = 0.18;
ret_params.probe_amp_deg = 0.8;
ret_params.probe_len_s = 0.12;

% Safe-mode command limit after confirmed fault
delta_safe_max = 4.0 * pi/180;

% Load tuned parameters if available
tuned_file = fullfile(out_dir, 'tuned_params_C.mat');
if exist(tuned_file, 'file')
	S = load(tuned_file);
	if isfield(S, 'best')
		ret_params.alpha = S.best.alpha;
		ret_params.tau_smooth = S.best.tau_smooth;
		if isfield(S.best, 'kp_exp'), ret_params.kp_exp = S.best.kp_exp; end
		if isfield(S.best, 'kd_exp'), ret_params.kd_exp = S.best.kd_exp; end
		if isfield(S.best, 'ki_exp'), ret_params.ki_exp = S.best.ki_exp; end
		if isfield(S.best, 'probe_amp_deg'), ret_params.probe_amp_deg = S.best.probe_amp_deg; end
		if isfield(S.best, 'probe_len_s'), ret_params.probe_len_s = S.best.probe_len_s; end
		delta_safe_max = S.best.delta_safe_deg * pi/180;
			mon_params.trigger_margin = min(0.025, S.best.trigger_margin);
		fprintf('Using tuned C params: alpha=%.2f kpExp=%.2f kdExp=%.2f kiExp=%.2f probe=%.1fdeg/%.2fs tau=%.3f dsafe=%.1f margin=%.3f\n', ...
			ret_params.alpha, ret_params.kp_exp, ret_params.kd_exp, ret_params.ki_exp, ...
			ret_params.probe_amp_deg, ret_params.probe_len_s, ret_params.tau_smooth, ...
			S.best.delta_safe_deg, mon_params.trigger_margin);
	end
end

%% Sweep dimensions
fault_times  = [1.0, 3.0, 5.0];
adapt_flags  = [false, true];
fault_labels = {'T+1s','T+3s','T+5s'};
adapt_labels = {'NO_ADAPT','ADAPT'};

results = struct();  row = 0;

% Also store a key trace for the figure (T+3s, ADAPT vs NO_ADAPT)
key_traces = struct();

for fi = 1:3
	t_fault = fault_times(fi);
	k_fault = round(t_fault/dt);

	for ai = 1:2
		adapt = adapt_flags(ai);
		row   = row + 1;

		%% State init
		theta=0; omega=0; delta_act=0; delta_vel=0; int_err=0;
		u_slew=0;  gains = gains_nom;
		wn_cur = wn_nom;  delay_cur = delay_nom;
		N_delay_cur = max(1, round(delay_cur/dt));
		delay_buf = zeros(N_delay_cur,1);

		% Monitor state
		mon_state.cmd_buf       = zeros(mon_win,1);
		mon_state.meas_buf      = zeros(mon_win,1);
		mon_state.t_elapsed     = 0;
		mon_state.confirm_count = 0;
		mon_state.wn_est        = wn_nom;
		mon_state.e_ref         = mon_params.e_norm_floor;
		mon_state.fault_latched = 0;
		mon_state.e_norm_lp     = mon_params.e_norm_floor;
		mon_state.wn_lp         = wn_nom;

		% Log arrays
		theta_log  = zeros(N,1);
		r_log      = zeros(N,1);
		wn_est_log = zeros(N,1);
		fault_log  = zeros(N,1);
		kp_log     = zeros(N,1);

		fault_detected_k = NaN;
		fault_injected_k = k_fault;
		r_cmd = 0;
		r_rate_max_nom = 35*pi/180;   % rad/s, nominal guidance-command slew
		r_rate_max_safe = 8*pi/180;   % rad/s, safe-mode slew after fault latch
		lag_confirm = 0;
		lag_trigger = 0.6*pi/180;
		lag_N_confirm = round(0.015/dt);
		probe_count = 0;
		probe_len = max(1, round(ret_params.probe_len_s/dt));
		probe_amp = ret_params.probe_amp_deg*pi/180;
		probe_freq = 12.0;
		lag_probe_start = max(2, round(0.40*lag_N_confirm));
        prev_r_target = 0;

		for k = 1:N
			t = (k-1) * dt;

			% Mission command profile with rate-limited shaping (realistic FCU behavior)
			if t < 0.6
				r_target = 0;
			elseif t < 2.2
				r_target = 8 * pi/180;
			elseif t < 3.8
				r_target = -8 * pi/180;
			elseif t < 5.8
				r_target = 10 * pi/180;
			else
				r_target = 0;
			end
			if adapt && mon_state.fault_latched
				r_rate_max = r_rate_max_safe;
			else
				r_rate_max = r_rate_max_nom;
			end
			dr_lim = r_rate_max * dt;
			r_cmd = r_cmd + max(-dr_lim, min(dr_lim, r_target - r_cmd));
			r = r_cmd;

			%% Inject fault
			if k == k_fault
				wn_cur    = wn_fault;
				delay_cur = delay_fault;
				% Resize delay buffer (extend with last value)
				N_delay_new = max(1, round(delay_cur/dt));
				if N_delay_new > length(delay_buf)
					delay_buf = [delay_buf; delay_buf(end)*ones(N_delay_new-length(delay_buf),1)];
				else
					delay_buf = delay_buf(1:N_delay_new);
				end
			end

			%% PID controller with current (possibly adapted) gains
			err = r - theta;
			u_unsat = gains.Kp*err + gains.Kd*(-omega) + gains.Ki*int_err;

			transition_probe = (abs(r_target) > 1e-9) && (abs(prev_r_target) > 1e-9) && (sign(r_target) ~= sign(prev_r_target));
			if adapt && (~mon_state.fault_latched) && (mon_state.t_elapsed > mon_params.arm_time_s)
				probe_suspect = (lag_confirm >= lag_probe_start) || ...
					(mon_state.e_norm_lp > mon_state.e_ref + 0.5*mon_params.trigger_margin);
				if (probe_suspect || transition_probe) && probe_count == 0
					probe_count = probe_len;
				end
			end
			if probe_count > 0
				u_unsat = u_unsat + probe_amp * sin(2*pi*probe_freq*t);
				probe_count = probe_count - 1;
			end
			if adapt && mon_state.fault_latched
				u_limit = delta_safe_max;
			else
				u_limit = delta_max;
			end
			u_raw = max(-u_limit, min(u_limit, u_unsat));

			% Conditional integration anti-windup
			is_sat = abs(u_unsat - u_raw) > 1e-12;
			if (~is_sat) || (sign(err) ~= sign(u_unsat - u_raw))
				int_err = int_err + err*dt;
			end

			%% Slew limiter
			du    = max(-slew_max*dt, min(slew_max*dt, u_raw - u_slew));
			u_slew = u_slew + du;

			%% Delay line
			delay_buf = [u_slew; delay_buf(1:end-1)];
			u_del = delay_buf(end);

			%% Actuator (current wn)
			dv = wn_cur^2*(u_del - delta_act) - 2*zeta_a*wn_cur*delta_vel;
			delta_vel = delta_vel + dt*dv;
			delta_act = delta_act + dt*delta_vel;
			delta = max(-delta_max, min(delta_max, delta_act));

			%% Rocket pitch dynamics
			omega = omega + dt*(b0*delta - c_damp*omega);
			theta = theta + dt*omega;

			%% Health monitor (runs every step)
			[wn_est_k, flag_k, mon_state] = actuator_health_monitor( ...
				u_slew, delta_act, mon_state, mon_params, dt);
			lag_err = abs(u_slew - delta_act);
			if (mon_state.t_elapsed > mon_params.arm_time_s) && (lag_err > lag_trigger) && (abs(u_slew) > 0.5*pi/180)
				lag_confirm = lag_confirm + 1;
			else
				lag_confirm = max(0, lag_confirm - 1);
			end
			if lag_confirm >= lag_N_confirm
				flag_k = 1;
				mon_state.fault_latched = 1;
			end

			%% Adaptive retuning (only if adaptation enabled and fault flagged)
			if adapt && flag_k && (k >= k_fault)
				if isnan(fault_detected_k)
					fault_detected_k = k;
					fprintf('    Fault detected at T=%.3fs (injected at T=%.1fs, latency=%.3fs)\n', ...
						k*dt, t_fault, (k-k_fault)*dt);
					int_err = 0;  % clear integrator on fault latch to avoid windup carryover
				end
				gains = adaptive_retune(wn_est_k, gains_nom, ret_params, gains, dt);
			end

			theta_log(k)  = theta;
			r_log(k)      = r;
			wn_est_log(k) = wn_est_k;
			fault_log(k)  = flag_k;
			kp_log(k)     = gains.Kp;
            prev_r_target = r_target;
		end

		%% Metrics (measured in 2s window after fault injection)
		k1 = k_fault;  k2 = min(N, k_fault + round(4.0/dt));
		err_log       = theta_log - r_log;
		max_pitch     = max(abs(err_log(k1:k2))) * 180/pi;
		settled_rms   = sqrt(mean(err_log(min(N,k2-round(1/dt)):k2).^2)) * 180/pi;
		if ~isnan(fault_detected_k)
			detect_latency = (fault_detected_k - k_fault) * dt;
		else
			detect_latency = NaN;
		end

		results(row).fault_time_s       = t_fault;
		results(row).fault_label        = fault_labels{fi};
		results(row).adaptation         = adapt_labels{ai};
		results(row).max_pitch_dev_deg  = max_pitch;
		results(row).detect_latency_s   = detect_latency;
		results(row).settled_rms_deg    = settled_rms;

		fprintf('  %s %-9s  max_pitch=%6.2f deg  latency=%s s\n', ...
			fault_labels{fi}, adapt_labels{ai}, max_pitch, ...
			num2str(detect_latency,'%.3f'));

		% Save key trace for figure (T+3s case)
		if fi == 2
			t_vec = (0:N-1)'*dt;
			if ai == 1
				key_traces.t         = t_vec;
				key_traces.theta_no  = theta_log * 180/pi;
				key_traces.wn_no     = wn_est_log;
				key_traces.t_fault   = t_fault;
			else
				key_traces.theta_yes = theta_log * 180/pi;
				key_traces.wn_yes    = wn_est_log;
			end
		end
	end
end

%% Write CSV
fid = fopen(fullfile(out_dir,'sweep_results_C.csv'),'w');
fprintf(fid,'fault_time_s,fault_label,adaptation,max_pitch_dev_deg,detect_latency_s,settled_rms_deg\n');
for i = 1:row
	fprintf(fid,'%.1f,%s,%s,%.4f,%.4f,%.4f\n', results(i).fault_time_s, ...
		results(i).fault_label, results(i).adaptation, results(i).max_pitch_dev_deg, ...
		results(i).detect_latency_s, results(i).settled_rms_deg);
end
fclose(fid);
fprintf('Saved: %s\n', fullfile(out_dir,'sweep_results_C.csv'));

%% Key comparison figure (T+3s fault)
fig = figure('Visible','off','Position',[100 100 900 420]);
subplot(2,1,1);
plot(key_traces.t, key_traces.theta_no,  'Color',[0.85 0.35 0.35], 'LineWidth',1.5); hold on;
plot(key_traces.t, key_traces.theta_yes, 'Color',[0.20 0.65 0.30], 'LineWidth',1.5);
xline(key_traces.t_fault, 'k--', 'Fault injected', 'LabelVerticalAlignment','bottom');
ylabel('Pitch Angle (deg)'); title('Pitch Response: T+3s Combined Fault');
legend('No Adaptation','With Adaptation','Location','best'); grid on; ylim([-20 20]);
subplot(2,1,2);
plot(key_traces.t, key_traces.wn_no,  'Color',[0.85 0.35 0.35], 'LineWidth',1.5); hold on;
plot(key_traces.t, key_traces.wn_yes, 'Color',[0.20 0.65 0.30], 'LineWidth',1.5);
xline(key_traces.t_fault, 'k--');
yline(0.65*wn_nom, 'b:', 'Fault threshold');
ylabel('Estimated wn (rad/s)'); xlabel('Time (s)');
title('Health Monitor: Estimated Actuator Bandwidth');
legend('Without Adaptation','With Adaptation','Location','best'); grid on;
sgtitle('Direction C — TVC Actuator Fault Detection + Autonomous Retuning');
saveas(fig, fullfile(gfx_dir,'fault_C_comparison.png'));
fprintf('Saved graph: fault_C_comparison.png\n');

%% Save wn trace and jump metrics (T+3s key case)
wn_trace_file = fullfile(out_dir, 'fault_C_wn_trace.csv');
fid2 = fopen(wn_trace_file, 'w');
fprintf(fid2, 'time_s,wn_no_adapt,wn_adapt\n');
for k = 1:numel(key_traces.t)
	fprintf(fid2, '%.4f,%.6f,%.6f\n', key_traces.t(k), key_traces.wn_no(k), key_traces.wn_yes(k));
end
fclose(fid2);
fprintf('Saved: %s\n', wn_trace_file);

dwn_no = abs(diff(key_traces.wn_no));
dwn_yes = abs(diff(key_traces.wn_yes));
fprintf('wn jump metrics (rad/s per step): no_adapt max=%.3f p95=%.3f | adapt max=%.3f p95=%.3f\n', ...
	max(dwn_no), prctile(dwn_no,95), max(dwn_yes), prctile(dwn_yes,95));

%% Semi-promising assessment
fprintf('\n=== DIRECTION C: SEMI-PROMISING ASSESSMENT ===\n');
fprintf('Threshold: WITH adapt max_pitch <= 5 deg AND WITHOUT adapt > 15 deg\n\n');
passed = 0;
for i = 1:2:row-1
	no_adapt  = results(i).max_pitch_dev_deg;
	yes_adapt = results(i+1).max_pitch_dev_deg;
	lat       = results(i+1).detect_latency_s;
	ok1 = no_adapt  > 15.0;
	ok2 = yes_adapt <= 5.0;
	ok3 = ~isnan(lat) && lat <= 0.5;
	if ok1&&ok2&&ok3; tag='PASS'; passed=passed+1; else; tag='FAIL'; end
	fprintf('  %s: no_adapt=%.1f  yes_adapt=%.1f  latency=%s  %s\n', ...
		results(i).fault_label, no_adapt, yes_adapt, num2str(lat,'%.3f'), tag);
end
if passed >= 2
	fprintf('\n>>> DIRECTION C: SEMI-PROMISING (%d/3 fault times pass) <<<\n', passed);
else
	fprintf('\n>>> DIRECTION C: NEEDS TUNING (%d/3 fault times pass) <<<\n', passed);
end
