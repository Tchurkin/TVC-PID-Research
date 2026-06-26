% run_pitch_program_head_to_head.m
% Head-to-head maneuver test for the new accessibility/unstable-airframe story.
%
% Question tested:
%   Can a bench-realistic tuning workflow expand an amateur TVC rocket's
%   pitch-program envelope into reduced-stability airframes and high wind,
%   compared with a benign/basic-sim PID tuning workflow?
%
% Methods:
%   AMATEUR_BASIC_PID  - PD gains tuned on the same maneuver in a clean model
%                        with no sensor bias, deadband, backlash, or gust.
%   BENCH_TUNED_PID   - PD gains tuned in the full realistic model.
%   BENCH_TUNED_PCH   - PCH-LQR gains tuned in the full realistic model.
%
% All methods are deployed on the same realistic plant with the same command,
% actuator limits, sloppy-servo effects, and Dryden gust intensity.

clear; clc;

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(genpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src')));

results_dir = fullfile(here, 'results');
graphs_dir = fullfile(results_dir, 'graphs');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'); mkdir(graphs_dir); end

opts = experiment_opts();

p_list = [0 4 6 8 10];
gust_list = [0.30 0.90 1.50];
methods = ["AMATEUR_BASIC_PID", "BENCH_TUNED_PID", "BENCH_TUNED_PCH"];

detail_rows = cell(0, 19);
summary_rows = cell(0, 15);
trace_bank = struct();

fprintf('=== PITCH-PROGRAM HEAD-TO-HEAD ===\n');
fprintf('Command: 0 -> %.1f deg -> 0, t_end=%.2f s\n', opts.cmd_deg, opts.t_end);
fprintf('Realistic actuator: slew=%.1f code units/s, u_max=%.1f code units\n', ...
    opts.slew_code, opts.u_max_code);

for ip = 1:numel(p_list)
    p_val = p_list(ip);
    cfg_base = base_config(p_val, opts);
    sc = pitch_program_scenario(cfg_base, opts);

    basic_realism = ideal_profile();
    tune_opts = opts;
    tune_opts.seeds = opts.tune_seeds;
    tune_opts.theta0_deg_set = opts.tune_theta0_deg_set;
    [basic_Kp, basic_Kd, basic_design] = tune_controller('PID', cfg_base, sc, tune_opts, basic_realism);

    fprintf('\n--- p_unstable = %.1f ---\n', p_val);
    fprintf('  AMATEUR_BASIC_PID tuned in clean sim: Kp=%.1f Kd=%.1f success=%.2f rms=%.2f\n', ...
        basic_Kp, basic_Kd, basic_design.success_rate, basic_design.rms_error_deg);

    for ig = 1:numel(gust_list)
        gust_std = gust_list(ig);
        full_realism = full_realistic_profile(gust_std);

        [bench_pid_Kp, bench_pid_Kd, bench_pid_design] = tune_controller('PID', cfg_base, sc, tune_opts, full_realism);
        [bench_pch_Kp, bench_pch_Kd, bench_pch_design] = tune_controller('PCH_LQR', cfg_base, sc, tune_opts, full_realism);

        method_gains = [
            basic_Kp,     basic_Kd;
            bench_pid_Kp, bench_pid_Kd;
            bench_pch_Kp, bench_pch_Kd];
        design_rates = [basic_design.success_rate; bench_pid_design.success_rate; bench_pch_design.success_rate];

        fprintf('  gust_std=%.2f: basicPID %.1f/%.1f, benchPID %.1f/%.1f, benchPCH %.1f/%.1f\n', ...
            gust_std, basic_Kp, basic_Kd, bench_pid_Kp, bench_pid_Kd, bench_pch_Kp, bench_pch_Kd);

        for im = 1:numel(methods)
            method = methods(im);
            ctrl_name = controller_name(method);
            cfg_eval = configure_controller(cfg_base, ctrl_name, method_gains(im,1), method_gains(im,2));
            metrics = evaluate_controller(ctrl_name, cfg_eval, sc, opts, full_realism);

            summary_rows(end+1,:) = {method, p_val, gust_std, method_gains(im,1), method_gains(im,2), ...
                design_rates(im), metrics.success_rate, metrics.rms_error_deg, metrics.peak_error_deg, ...
                metrics.end_error_deg, metrics.max_theta_deg, metrics.u_cmd_sat_frac, metrics.slew_sat_frac, ...
                metrics.mean_control_rms, metrics.n_trials}; %#ok<SAGROW>

            for ir = 1:size(metrics.trials, 1)
                detail_rows(end+1,:) = {method, p_val, gust_std, method_gains(im,1), method_gains(im,2), ...
                    metrics.trials.seed(ir), metrics.trials.theta0_deg(ir), metrics.trials.success(ir), ...
                    metrics.trials.rms_error_deg(ir), metrics.trials.peak_error_deg(ir), ...
                    metrics.trials.end_error_deg(ir), metrics.trials.max_theta_deg(ir), ...
                    metrics.trials.u_cmd_sat_frac(ir), metrics.trials.slew_sat_frac(ir), ...
                    metrics.trials.control_rms(ir), opts.cmd_deg, opts.slew_code, opts.u_max_code, opts.servo_deadband}; %#ok<SAGROW>
            end

            if p_val == opts.trace_p && abs(gust_std - opts.trace_gust_std) < 1e-9
                trace_key = matlab.lang.makeValidName(char(method));
                trace_bank.(trace_key) = run_representative(ctrl_name, cfg_eval, sc, full_realism, opts);
            end
        end
    end
end

Summary = cell2table(summary_rows, 'VariableNames', { ...
    'method', 'p_unstable', 'gust_std', 'Kp', 'Kd', 'design_success_rate', ...
    'deploy_success_rate', 'rms_error_deg', 'peak_error_deg', 'end_error_deg', ...
    'max_theta_deg', 'u_cmd_sat_frac', 'slew_sat_frac', 'control_rms', 'n_trials'});
Details = cell2table(detail_rows, 'VariableNames', { ...
    'method', 'p_unstable', 'gust_std', 'Kp', 'Kd', 'seed', 'theta0_deg', 'success', ...
    'rms_error_deg', 'peak_error_deg', 'end_error_deg', 'max_theta_deg', ...
    'u_cmd_sat_frac', 'slew_sat_frac', 'control_rms', 'cmd_deg', 'slew_code', ...
    'u_max_code', 'servo_deadband'});

writetable(Summary, fullfile(results_dir, 'pitch_program_summary.csv'));
writetable(Details, fullfile(results_dir, 'pitch_program_head_to_head.csv'));

plot_success_heatmap(Summary, methods, p_list, gust_list, fullfile(graphs_dir, 'pitch_program_success_heatmap.png'));
plot_representative_trace(trace_bank, methods, fullfile(graphs_dir, 'pitch_program_representative_trace.png'));

fprintf('\nSaved: experiments/results/pitch_program_summary.csv\n');
fprintf('Saved: experiments/results/pitch_program_head_to_head.csv\n');
fprintf('Saved: experiments/results/graphs/pitch_program_success_heatmap.png\n');
fprintf('Saved: experiments/results/graphs/pitch_program_representative_trace.png\n');
fprintf('DONE.\n');


function opts = experiment_opts()
opts.t_end = 3.60;
opts.cmd_deg = 20.0;
opts.cmd_start_s = 0.55;
opts.cmd_ramp_s = 0.75;
opts.cmd_hold_s = 0.85;
opts.slew_code = 60.0;
opts.u_max_code = 12.0;
opts.servo_deadband = 0.05;
opts.servo_backlash = 0.10;
opts.theta0_deg_set = [0 3];
opts.tune_theta0_deg_set = [0 3];
opts.seeds = 1:6;
opts.tune_seeds = 1:3;
opts.Kp_grid = [5 10 15 20 30 45];
opts.Kd_grid = [2 4 6 8 12 16];
opts.success_rms_error_deg = 6.0;
opts.success_peak_error_deg = 15.0;
opts.success_end_error_deg = 6.0;
opts.success_max_theta_deg = 55.0;
opts.trace_p = 8;
opts.trace_gust_std = 0.90;
opts.trace_seed = 2;
opts.trace_theta0_deg = 3;
end


function cfg = base_config(p_unstable, opts)
cfg = rocket_defaults();
cfg.t_end_demo = opts.t_end;
cfg.plant.theta0 = 0;
cfg.plant.aero_damp = 0.5;
cfg.plant.control_eff = 8.0;
cfg.plant.keff_nom = 8.0;
cfg.plant.p_unstable = p_unstable;
cfg.plant.tau_act = 0.05;
cfg.plant.slew_max = opts.slew_code;
cfg.plant.u_max = opts.u_max_code;

cfg.controllers.FIXED_LQR.u_max = cfg.plant.u_max;
cfg.controllers.PCH_LQR.u_max = cfg.plant.u_max;
cfg.controllers.PCH_LQR.K_nominal = [20 8];
cfg.controllers.PCH_LQR.keff_nom = cfg.plant.keff_nom;
cfg.controllers.PCH_LQR.aero_damp = cfg.plant.aero_damp;
cfg.controllers.PCH_LQR.tau_act_assumed = cfg.plant.tau_act;
cfg.controllers.PCH_LQR.slew_max_assumed = cfg.plant.slew_max;

cfg.controllers.PID = struct('Kp', 20, 'Ki', 0, 'Kd', 8, ...
    'u_max', cfg.plant.u_max, 'i_lim', cfg.plant.u_max);
end


function sc = pitch_program_scenario(cfg, opts)
sc = rocket_scenario('nominal', cfg);
sc.kind = "PITCH_PROGRAM";
sc.t_end = opts.t_end;
sc.disturbance_amp = 0.05;
sc.disturbance_freq_hz = 0.80;
sc.theta_ref_fun = @(t) pitch_ref_rad(t, opts);
sc.q_ref_fun = @(t) pitch_rate_ref_rad_s(t, opts);
end


function theta = pitch_ref_rad(t, opts)
amp = deg2rad(opts.cmd_deg);
t0 = opts.cmd_start_s;
t1 = t0 + opts.cmd_ramp_s;
t2 = t1 + opts.cmd_hold_s;
t3 = t2 + opts.cmd_ramp_s;
if t < t0
    theta = 0;
elseif t < t1
    theta = amp * (t - t0) / opts.cmd_ramp_s;
elseif t < t2
    theta = amp;
elseif t < t3
    theta = amp * (1 - (t - t2) / opts.cmd_ramp_s);
else
    theta = 0;
end
end


function q_ref = pitch_rate_ref_rad_s(t, opts)
amp = deg2rad(opts.cmd_deg);
t0 = opts.cmd_start_s;
t1 = t0 + opts.cmd_ramp_s;
t2 = t1 + opts.cmd_hold_s;
t3 = t2 + opts.cmd_ramp_s;
if t >= t0 && t < t1
    q_ref = amp / opts.cmd_ramp_s;
elseif t >= t2 && t < t3
    q_ref = -amp / opts.cmd_ramp_s;
else
    q_ref = 0;
end
end


function realism = full_realistic_profile(gust_std)
realism = struct( ...
    'gyro_noise_std', 0.015, 'gyro_bias_init', 0.010, 'gyro_bias_rw', 0.005, ...
    'gyro_quant_lsb', 2 * pi / 4000, 'bias_cal_residual_frac', 0.10, ...
    'sensor_latency_steps', 3, 'servo_deadband', 0.05, 'servo_backlash', 0.10, ...
    'servo_pos_noise', 0.01, 'gust_std', gust_std, 'gust_tau', 0.40, ...
    'keff_drift_rate', 0.10, 'theta_init_bias', deg2rad(0.5));
end


function realism = ideal_profile()
realism = struct( ...
    'gyro_noise_std', 0.0, 'gyro_bias_init', 0.0, 'gyro_bias_rw', 0.0, ...
    'gyro_quant_lsb', 0.0, 'bias_cal_residual_frac', 0.0, ...
    'sensor_latency_steps', 1, 'servo_deadband', 0.0, 'servo_backlash', 0.0, ...
    'servo_pos_noise', 0.0, 'gust_std', 0.0, 'gust_tau', 0.40, ...
    'keff_drift_rate', 0.0, 'theta_init_bias', 0.0);
end


function ctrl_name = controller_name(method)
if method == "BENCH_TUNED_PCH"
    ctrl_name = 'PCH_LQR';
else
    ctrl_name = 'PID';
end
end


function cfg = configure_controller(cfg, ctrl_name, Kp, Kd)
switch upper(string(ctrl_name))
    case "PID"
        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Ki = 0;
        cfg.controllers.PID.Kd = Kd;
        cfg.controllers.PID.u_max = cfg.plant.u_max;
        cfg.controllers.PID.i_lim = cfg.plant.u_max;
    case "PCH_LQR"
        cfg.controllers.PCH_LQR.K_nominal = [Kp Kd];
        cfg.controllers.PCH_LQR.u_max = cfg.plant.u_max;
        cfg.controllers.PCH_LQR.keff_nom = cfg.plant.keff_nom;
        cfg.controllers.PCH_LQR.aero_damp = cfg.plant.aero_damp;
        cfg.controllers.PCH_LQR.tau_act_assumed = cfg.plant.tau_act;
        cfg.controllers.PCH_LQR.slew_max_assumed = cfg.plant.slew_max;
    otherwise
        error('configure_controller:unknownController', 'Unknown controller %s', ctrl_name);
end
end


function [best_Kp, best_Kd, best_metrics] = tune_controller(ctrl_name, cfg, sc, opts, realism)
best_rate = -inf;
best_rms = inf;
best_peak = inf;
best_Kp = NaN;
best_Kd = NaN;
best_metrics = struct('success_rate', NaN, 'rms_error_deg', NaN, 'peak_error_deg', NaN);
for Kp = opts.Kp_grid
    for Kd = opts.Kd_grid
        cfg_try = configure_controller(cfg, ctrl_name, Kp, Kd);
        metrics = evaluate_controller(ctrl_name, cfg_try, sc, opts, realism);
        better = metrics.success_rate > best_rate || ...
            (abs(metrics.success_rate - best_rate) < 1e-9 && metrics.rms_error_deg < best_rms) || ...
            (abs(metrics.success_rate - best_rate) < 1e-9 && abs(metrics.rms_error_deg - best_rms) < 1e-9 && metrics.peak_error_deg < best_peak);
        if better
            best_rate = metrics.success_rate;
            best_rms = metrics.rms_error_deg;
            best_peak = metrics.peak_error_deg;
            best_Kp = Kp;
            best_Kd = Kd;
            best_metrics = metrics;
        end
    end
end
end


function metrics = evaluate_controller(ctrl_name, cfg, sc, opts, realism)
n_trials = numel(opts.seeds) * numel(opts.theta0_deg_set);
seed_col = zeros(n_trials, 1);
theta0_col = zeros(n_trials, 1);
success_col = false(n_trials, 1);
rms_col = zeros(n_trials, 1);
peak_col = zeros(n_trials, 1);
end_col = zeros(n_trials, 1);
max_theta_col = zeros(n_trials, 1);
u_sat_col = zeros(n_trials, 1);
slew_sat_col = zeros(n_trials, 1);
control_rms_col = zeros(n_trials, 1);

idx = 0;
for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1;
        seed = opts.seeds(iseed) + 100 * it;
        seed_col(idx) = seed;
        theta0_col(idx) = opts.theta0_deg_set(it);
        try
            out = simulate_case_realistic(ctrl_name, sc, cfg, seed, realism);
            [success_col(idx), rms_col(idx), peak_col(idx), end_col(idx), max_theta_col(idx), ...
                u_sat_col(idx), slew_sat_col(idx), control_rms_col(idx)] = score_run(out, cfg, opts);
        catch
            success_col(idx) = false;
            rms_col(idx) = 90;
            peak_col(idx) = 90;
            end_col(idx) = 90;
            max_theta_col(idx) = 90;
            u_sat_col(idx) = 1;
            slew_sat_col(idx) = 1;
            control_rms_col(idx) = cfg.plant.u_max;
        end
    end
end

metrics.success_rate = mean(success_col);
metrics.rms_error_deg = mean(rms_col);
metrics.peak_error_deg = mean(peak_col);
metrics.end_error_deg = mean(end_col);
metrics.max_theta_deg = mean(max_theta_col);
metrics.u_cmd_sat_frac = mean(u_sat_col);
metrics.slew_sat_frac = mean(slew_sat_col);
metrics.mean_control_rms = mean(control_rms_col);
metrics.n_trials = n_trials;
metrics.trials = table(seed_col, theta0_col, success_col, rms_col, peak_col, end_col, ...
    max_theta_col, u_sat_col, slew_sat_col, control_rms_col, 'VariableNames', ...
    {'seed', 'theta0_deg', 'success', 'rms_error_deg', 'peak_error_deg', ...
    'end_error_deg', 'max_theta_deg', 'u_cmd_sat_frac', 'slew_sat_frac', 'control_rms'});
end


function [success, rms_error_deg, peak_error_deg, end_error_deg, max_theta_deg, u_sat_frac, slew_sat_frac, control_rms] = score_run(out, cfg, opts)
err = out.theta - out.theta_ref;
if ~all(isfinite(out.theta)) || ~all(isfinite(err))
    rms_error_deg = 90;
    peak_error_deg = 90;
    end_error_deg = 90;
    max_theta_deg = 90;
    u_sat_frac = 1;
    slew_sat_frac = 1;
    control_rms = cfg.plant.u_max;
    success = false;
    return;
end

rms_error_deg = min(90, rad2deg(sqrt(mean(err.^2))));
peak_error_deg = min(90, rad2deg(max(abs(err))));
end_error_deg = min(90, rad2deg(abs(err(end))));
max_theta_deg = min(90, rad2deg(max(abs(out.theta))));
u_sat_frac = mean(abs(out.u_cmd) >= 0.99 * cfg.plant.u_max);
slew_rate = abs(diff(out.u_act)) / cfg.dt;
slew_sat_frac = mean(slew_rate >= 0.99 * cfg.plant.slew_max);
control_rms = sqrt(mean(out.u_act.^2));

success = rms_error_deg <= opts.success_rms_error_deg && ...
          peak_error_deg <= opts.success_peak_error_deg && ...
          end_error_deg <= opts.success_end_error_deg && ...
          max_theta_deg <= opts.success_max_theta_deg;
end


function out = run_representative(ctrl_name, cfg, sc, realism, opts)
cfg.plant.theta0 = deg2rad(opts.trace_theta0_deg);
out = simulate_case_realistic(ctrl_name, sc, cfg, opts.trace_seed, realism);
end


function plot_success_heatmap(T, methods, p_list, gust_list, out_path)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1120 360]);
for im = 1:numel(methods)
    M = nan(numel(p_list), numel(gust_list));
    for ip = 1:numel(p_list)
        for ig = 1:numel(gust_list)
            mask = T.method == methods(im) & T.p_unstable == p_list(ip) & abs(T.gust_std - gust_list(ig)) < 1e-9;
            if any(mask)
                M(ip, ig) = T.deploy_success_rate(mask);
            end
        end
    end
    subplot(1, numel(methods), im);
    imagesc(gust_list, p_list, M, [0 1]);
    axis xy;
    title(strrep(char(methods(im)), '_', '\_'));
    xlabel('gust std (rad/s^2)');
    ylabel('p unstable (1/s)');
    xticks(gust_list);
    yticks(p_list);
    colormap(gca, parula(256));
    colorbar;
    for ip = 1:numel(p_list)
        for ig = 1:numel(gust_list)
            text(gust_list(ig), p_list(ip), sprintf('%.2f', M(ip, ig)), ...
                'HorizontalAlignment', 'center', 'Color', text_color(M(ip, ig)), 'FontWeight', 'bold');
        end
    end
end
exportgraphics(fig, out_path, 'Resolution', 200);
close(fig);
end


function plot_representative_trace(trace_bank, methods, out_path)
if isempty(fieldnames(trace_bank))
    return;
end
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 980 640]);
colors = lines(numel(methods));

subplot(2,1,1);
hold on;
first_key = matlab.lang.makeValidName(char(methods(1)));
if isfield(trace_bank, first_key)
    plot(trace_bank.(first_key).time, rad2deg(trace_bank.(first_key).theta_ref), 'k--', 'LineWidth', 1.8);
end
legend_entries = {"reference"};
for im = 1:numel(methods)
    key = matlab.lang.makeValidName(char(methods(im)));
    if isfield(trace_bank, key)
        out = trace_bank.(key);
        plot(out.time, rad2deg(out.theta), 'LineWidth', 1.4, 'Color', colors(im,:));
        legend_entries{end+1} = strrep(char(methods(im)), '_', '\_'); %#ok<AGROW>
    end
end
grid on;
ylabel('pitch angle (deg)');
title('Representative pitch-program tracking');
legend(legend_entries, 'Location', 'best');

subplot(2,1,2);
hold on;
for im = 1:numel(methods)
    key = matlab.lang.makeValidName(char(methods(im)));
    if isfield(trace_bank, key)
        out = trace_bank.(key);
        plot(out.time, out.u_act, 'LineWidth', 1.3, 'Color', colors(im,:));
    end
end
grid on;
xlabel('time (s)');
ylabel('delivered command (code units)');
title('Delivered actuator motion');

exportgraphics(fig, out_path, 'Resolution', 200);
close(fig);
end


function c = text_color(v)
if isnan(v) || v < 0.55
    c = [1 1 1];
else
    c = [0 0 0];
end
end