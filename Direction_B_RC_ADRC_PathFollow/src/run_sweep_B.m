%% Sweep Runner — Direction B (STUB)
% ====================================
% Runs the 3 x 2 sweep (crosswind speed x controller) and writes
% results to outputs/sweep_results_B.csv.
%
% SWEEP DIMENSIONS:
%   wind_cases   = {'low', 'medium', 'high'}
%   controllers  = {'L1', 'ADRC_L1'}
%
% METRICS COLLECTED:
%   crosstrack_rms_m     — RMS cross-track error over 500 m leg (m)
%   peak_crosstrack_m    — peak absolute cross-track deviation (m)
%   aileron_var_deg2     — variance of aileron deflection (control effort)
%   track_completion     — fraction of track within 5 m band (%)
%
% SEMI-PROMISING THRESHOLD:
%   ADRC+L1 peak_crosstrack_m <= 0.60 * L1 peak_crosstrack_m
%   AND ADRC+L1 aileron_var_deg2 <= 1.10 * L1 aileron_var_deg2
%
% OUTPUT FILE:
%   outputs/sweep_results_B.csv
%
% TODO (Phase 4 implementation):
%   1. Load scenario from crosswind_scenario
%   2. Simulate both controllers using ode45 (or fixed-step RK4)
%   3. Extract metrics, write CSV, print table
%   4. Generate outputs/graphs/sweep_B_summary.png

%% run_sweep_B.m  -  Direction B: L1 guidance vs ADRC path following in crosswind
% Runs 3 wind cases x 2 controllers, writes sweep_results_B.csv + graphs.
%
% PLANT MODEL (simplified fixed-wing lateral dynamics):
%   y_ct_dot   = vy_ct + w_wind        [crosstrack position]
%   vy_ct_dot  = u_actual              [crosstrack velocity via bank angle]
%   u_actual_dot = (u_cmd - u_actual) / tau_act    [actuator lag]
%
%   u_cmd  = lateral accel command from controller (m/s^2)
%   w_wind = crosswind (m/s) -> appears as velocity disturbance at y_ct_dot
%
% KEY: L1 only reacts to y_ct (no wind model).
%      ADRC estimates w_wind via ESO and pre-compensates.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
addpath(this_dir);
out_dir  = fullfile(dir_root, 'outputs');
gfx_dir  = fullfile(out_dir, 'graphs');
if ~exist(out_dir,'dir'), mkdir(out_dir); end
if ~exist(gfx_dir,'dir'), mkdir(gfx_dir); end

fprintf('Running Direction B sweep...\n');

%% Sweep dimensions
wind_cases   = {'low','medium','high'};
ctrl_names   = {'L1','ADRC'};
dt = 0.010;  t_end = 60.0;  SEED = 77;   % 60s flight, 100 Hz

%% L1 parameters
l1_params.L1_period = 20.0;   % s  (ArduPilot default)
l1_params.u_max     = 4.0;    % m/s^2 (max lateral accel ~15 deg bank)

%% ADRC parameters
omega_c = 0.40;   % slightly softer bandwidth to reduce effort spikes
omega_o = 1.60;   % lower observer bandwidth for smoother estimate
adrc_params.b0    = 1.0;
adrc_params.Kp    = omega_c^2;
adrc_params.Kd    = 2 * omega_c;
adrc_params.l1    = 3 * omega_o;
adrc_params.l2    = 3 * omega_o^2;
adrc_params.l3    = omega_o^3;
adrc_params.u_max = l1_params.u_max;

%% Hybrid ADRC-over-L1 blending and rate limit
blend.k_adrc = 0.60;             % 0 -> pure L1, 1 -> pure ADRC
blend.du_max = 35.0;             % max command slew (m/s^3)

% Load tuned parameters if available
tuned_file = fullfile(out_dir, 'tuned_params_B.mat');
if exist(tuned_file, 'file')
	S = load(tuned_file);
	if isfield(S, 'best')
		omega_c = S.best.omega_c;
		omega_o = S.best.omega_o;
		adrc_params.Kp = omega_c^2;
		adrc_params.Kd = 2 * omega_c;
		adrc_params.l1 = 3 * omega_o;
		adrc_params.l2 = 3 * omega_o^2;
		adrc_params.l3 = omega_o^3;
		blend.k_adrc = S.best.k_adrc;
		blend.du_max = S.best.du_max;
		fprintf('Using tuned B params: oc=%.2f oo=%.2f kadrc=%.2f dumax=%.1f\n', ...
			omega_c, omega_o, blend.k_adrc, blend.du_max);
	end
end

results = struct();  row = 0;

% Save medium-wind traces for figure
key_traces = struct();

for wi = 1:3
	sc = crosswind_scenario(wind_cases{wi}, dt, t_end, SEED+wi);
	N  = sc.N;

	for ci = 1:2
		row = row + 1;

		% State init
		y_ct = 0;  vy_ct = 0;  u_actual = 0;
		z_adrc = zeros(3,1);  u_prev = 0; u_cmd_prev = 0;

		y_log  = zeros(N,1);
		u_log  = zeros(N,1);

		for k = 1:N
			% Controller
			switch ci
				case 1  % L1
					[u_cmd, ~] = l1_guidance(y_ct, vy_ct, sc.V, l1_params);
				case 2  % ADRC
					[u_l1, ~] = l1_guidance(y_ct, vy_ct, sc.V, l1_params);
					[u_adrc, z_adrc] = adrc_layer(y_ct, z_adrc, u_prev, adrc_params, dt);
					u_mix = (1 - blend.k_adrc) * u_l1 + blend.k_adrc * u_adrc;
					du_lim = blend.du_max * dt;
					u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
			end
			u_prev = u_cmd;
			u_cmd_prev = u_cmd;

			% Actuator lag (first-order)
			u_actual = u_actual + dt * (u_cmd - u_actual) / sc.tau_act;
			u_actual = max(-sc.u_max, min(sc.u_max, u_actual));

			% Lateral dynamics
			vy_ct = vy_ct + dt * u_actual;
			vy_ct = max(-20, min(20, vy_ct));         % hard clip (divergence guard)
			y_ct  = y_ct  + dt * (vy_ct + sc.w_wind(k));

			y_log(k) = y_ct;
			u_log(k) = u_actual;
		end

		% Metrics (after 5s settling)
		k_start = round(5.0/dt);
		ct_rms   = sqrt(mean(y_log(k_start:end).^2));
		ct_peak  = max(abs(y_log(k_start:end)));
		u_var    = var(u_log(k_start:end));
		in_band  = mean(abs(y_log(k_start:end)) <= 5.0) * 100;  % % within 5 m band

		results(row).wind_case        = wind_cases{wi};
		results(row).controller       = ctrl_names{ci};
		results(row).crosstrack_rms_m = ct_rms;
		results(row).peak_crosstrack_m = ct_peak;
		results(row).accel_var        = u_var;
		results(row).pct_in_5m_band   = in_band;

		fprintf('  %-6s %-4s  ct_rms=%5.2f m  peak=%5.2f m  u_var=%.3f\n', ...
			wind_cases{wi}, ctrl_names{ci}, ct_rms, ct_peak, u_var);

		% Save medium traces
		if wi == 2
			t_vec = (0:N-1)'*dt;
			if ci==1; key_traces.t=t_vec; key_traces.y_l1=y_log; key_traces.u_l1=u_log;
			else;     key_traces.y_adrc=y_log; key_traces.u_adrc=u_log; key_traces.w=sc.w_wind; end
		end
	end
end

%% Write CSV
fid = fopen(fullfile(out_dir,'sweep_results_B.csv'),'w');
fprintf(fid,'wind_case,controller,crosstrack_rms_m,peak_crosstrack_m,accel_var,pct_in_5m_band\n');
for i=1:row
	fprintf(fid,'%s,%s,%.4f,%.4f,%.6f,%.2f\n', results(i).wind_case, ...
		results(i).controller, results(i).crosstrack_rms_m, results(i).peak_crosstrack_m, ...
		results(i).accel_var, results(i).pct_in_5m_band);
end
fclose(fid);
fprintf('Saved: %s\n', fullfile(out_dir,'sweep_results_B.csv'));

%% Key figure (medium crosswind traces)
fig = figure('Visible','off','Position',[100 100 900 480]);
subplot(3,1,1);
plot(key_traces.t, key_traces.w, 'Color',[0.6 0.6 0.6], 'LineWidth',1);
ylabel('Crosswind (m/s)'); title('5 m/s Mean Crosswind'); grid on;
subplot(3,1,2);
plot(key_traces.t, key_traces.y_l1,   'Color',[0.35 0.55 0.85], 'LineWidth',1.5); hold on;
plot(key_traces.t, key_traces.y_adrc, 'Color',[0.85 0.35 0.35], 'LineWidth',1.5);
yline(5,'k--'); yline(-5,'k--');
ylabel('Cross-track Error (m)'); title('Path Deviation: L1 vs ADRC');
legend('L1','ADRC','5m band','Location','best'); grid on;
subplot(3,1,3);
plot(key_traces.t, key_traces.u_l1,   'Color',[0.35 0.55 0.85], 'LineWidth',1.5); hold on;
plot(key_traces.t, key_traces.u_adrc, 'Color',[0.85 0.35 0.35], 'LineWidth',1.5);
ylabel('Lateral Accel (m/s^2)'); xlabel('Time (s)');
title('Control Effort'); legend('L1','ADRC','Location','best'); grid on;
sgtitle('Direction B — RC-Plane ADRC Path Following (5 m/s Crosswind)');
saveas(fig, fullfile(gfx_dir,'sweep_B_summary.png'));
fprintf('Saved graph: sweep_B_summary.png\n');

%% Semi-promising assessment
fprintf('\n=== DIRECTION B: SEMI-PROMISING ASSESSMENT ===\n');
fprintf('Threshold: ADRC peak_ct <= 0.60 x L1 peak_ct  AND  accel_var <= 1.10 x L1 accel_var\n\n');
passed = 0;
for i = 1:2:row-1
	r_ct  = results(i+1).peak_crosstrack_m / results(i).peak_crosstrack_m;
	r_var = results(i+1).accel_var          / results(i).accel_var;
	ok1 = r_ct  <= 0.60;
	ok2 = r_var <= 1.10;
	if ok1&&ok2; tag='PASS'; passed=passed+1; else; tag='FAIL'; end
	fprintf('  %-6s: ct_ratio=%.3f  var_ratio=%.3f  %s\n', ...
		results(i).wind_case, r_ct, r_var, tag);
end
if passed >= 2
	fprintf('\n>>> DIRECTION B: SEMI-PROMISING (%d/3 wind cases pass) <<<\n', passed);
else
	fprintf('\n>>> DIRECTION B: NEEDS TUNING (%d/3 wind cases pass) <<<\n', passed);
end
