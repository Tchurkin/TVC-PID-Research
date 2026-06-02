% run_pitch_program_stress_sweep.m
% Harsher follow-up to run_pitch_program_head_to_head.m.
%
% This sweep is intentionally closer to the actual claim:
%   - reduced-stability airframes,
%   - high wind / gusts,
%   - sloppy TVC hardware,
%   - and a maneuvering pitch program rather than simple vertical hold.
%
% Baselines:
%   AMATEUR_NOMINAL_PID  - one PID tuned only on an easy stable clean model.
%   CLEAN_RETUNED_PID    - generous steelman: per-cell PID tuned in a clean sim.
%   BENCH_TUNED_PID      - per-cell PID tuned in the full measured-realism sim.
%   BENCH_TUNED_PCH      - per-cell PCH-LQR tuned in the full measured-realism sim.

clear; clc;

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(genpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src')));

results_dir = fullfile(here, 'results');
graphs_dir = fullfile(results_dir, 'graphs');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'); mkdir(graphs_dir); end

profiles = actuator_profiles();
p_list = [6 8 10 12];
gust_list = [0.9 1.5 2.5];
methods = ["AMATEUR_NOMINAL_PID", "CLEAN_RETUNED_PID", "BENCH_TUNED_PID", "BENCH_TUNED_PCH"];
opts = base_opts();

nom = profiles(1);
nom_cfg = base_config(0, nom);
nom_sc = pitch_program_scenario(nom_cfg, nom);
[nominal_Kp, nominal_Kd, nominal_design] = tune_controller('PID', nom_cfg, nom_sc, opts, ideal_profile(nom, 0.0));

fprintf('=== PITCH-PROGRAM STRESS SWEEP ===\n');
fprintf('Nominal amateur PID: Kp=%.1f Kd=%.1f (stable clean design success %.2f)\n', ...
    nominal_Kp, nominal_Kd, nominal_design.success_rate);

summary_rows = cell(0, 18);
detail_rows = cell(0, 20);

for iprof = 1:numel(profiles)
    profile = profiles(iprof);
    fprintf('\n=== Profile: %s | cmd=%.0f deg, slew=%.1f, u_max=%.1f, deadband=%.2f, backlash=%.2f ===\n', ...
        profile.name, profile.cmd_deg, profile.slew_code, profile.u_max_code, profile.servo_deadband, profile.servo_backlash);

    for ip = 1:numel(p_list)
        p_val = p_list(ip);
        cfg_base = base_config(p_val, profile);
        sc = pitch_program_scenario(cfg_base, profile);

        [clean_Kp, clean_Kd, clean_design] = tune_controller('PID', cfg_base, sc, opts, ideal_profile(profile, 0.0));
        fprintf('  p=%.0f clean-retuned PID: Kp=%.1f Kd=%.1f design=%.2f\n', ...
            p_val, clean_Kp, clean_Kd, clean_design.success_rate);

        for ig = 1:numel(gust_list)
            gust_std = gust_list(ig);
            real = full_realistic_profile(profile, gust_std);
            [bench_Kp, bench_Kd, bench_design] = tune_controller('PID', cfg_base, sc, opts, real);
            [pch_Kp, pch_Kd, pch_design] = tune_controller('PCH_LQR', cfg_base, sc, opts, real);

            gain_table = [
                nominal_Kp, nominal_Kd;
                clean_Kp,   clean_Kd;
                bench_Kp,   bench_Kd;
                pch_Kp,     pch_Kd];
            design_table = [nominal_design.success_rate; clean_design.success_rate; bench_design.success_rate; pch_design.success_rate];

            fprintf('    gust=%.1f gains: nominal %.0f/%.0f | clean %.0f/%.0f | bench %.0f/%.0f | pch %.0f/%.0f\n', ...
                gust_std, nominal_Kp, nominal_Kd, clean_Kp, clean_Kd, bench_Kp, bench_Kd, pch_Kp, pch_Kd);

            for im = 1:numel(methods)
                method = methods(im);
                ctrl_name = controller_name(method);
                cfg_eval = configure_controller(cfg_base, ctrl_name, gain_table(im,1), gain_table(im,2));
                metrics = evaluate_controller(ctrl_name, cfg_eval, sc, opts, real);

                summary_rows(end+1,:) = {string(profile.name), method, p_val, gust_std, profile.cmd_deg, ...
                    profile.slew_code, profile.u_max_code, profile.servo_deadband, profile.servo_backlash, ...
                    gain_table(im,1), gain_table(im,2), design_table(im), metrics.success_rate, ...
                    metrics.rms_error_deg, metrics.peak_error_deg, metrics.end_error_deg, ...
                    metrics.u_cmd_sat_frac, metrics.slew_sat_frac}; %#ok<SAGROW>

                for ir = 1:height(metrics.trials)
                    detail_rows(end+1,:) = {string(profile.name), method, p_val, gust_std, profile.cmd_deg, ...
                        profile.slew_code, profile.u_max_code, profile.servo_deadband, profile.servo_backlash, ...
                        gain_table(im,1), gain_table(im,2), metrics.trials.seed(ir), metrics.trials.theta0_deg(ir), ...
                        metrics.trials.success(ir), metrics.trials.rms_error_deg(ir), metrics.trials.peak_error_deg(ir), ...
                        metrics.trials.end_error_deg(ir), metrics.trials.max_theta_deg(ir), ...
                        metrics.trials.u_cmd_sat_frac(ir), metrics.trials.slew_sat_frac(ir)}; %#ok<SAGROW>
                end
            end
        end
    end
end

Summary = cell2table(summary_rows, 'VariableNames', { ...
    'profile', 'method', 'p_unstable', 'gust_std', 'cmd_deg', 'slew_code', 'u_max_code', ...
    'servo_deadband', 'servo_backlash', 'Kp', 'Kd', 'design_success_rate', ...
    'deploy_success_rate', 'rms_error_deg', 'peak_error_deg', 'end_error_deg', ...
    'u_cmd_sat_frac', 'slew_sat_frac'});
Details = cell2table(detail_rows, 'VariableNames', { ...
    'profile', 'method', 'p_unstable', 'gust_std', 'cmd_deg', 'slew_code', 'u_max_code', ...
    'servo_deadband', 'servo_backlash', 'Kp', 'Kd', 'seed', 'theta0_deg', 'success', ...
    'rms_error_deg', 'peak_error_deg', 'end_error_deg', 'max_theta_deg', ...
    'u_cmd_sat_frac', 'slew_sat_frac'});

writetable(Summary, fullfile(results_dir, 'pitch_program_stress_summary.csv'));
writetable(Details, fullfile(results_dir, 'pitch_program_stress_trials.csv'));
plot_profile_bars(Summary, methods, profiles, fullfile(graphs_dir, 'pitch_program_stress_bars.png'));
plot_best_delta(Summary, profiles, p_list, gust_list, fullfile(graphs_dir, 'pitch_program_stress_delta.png'));
plot_workflow_delta(Summary, profiles, p_list, gust_list, fullfile(graphs_dir, 'pitch_program_workflow_delta.png'));

fprintf('\nSaved: experiments/results/pitch_program_stress_summary.csv\n');
fprintf('Saved: experiments/results/pitch_program_stress_trials.csv\n');
fprintf('Saved: experiments/results/graphs/pitch_program_stress_bars.png\n');
fprintf('Saved: experiments/results/graphs/pitch_program_stress_delta.png\n');
fprintf('Saved: experiments/results/graphs/pitch_program_workflow_delta.png\n');
fprintf('DONE.\n');


function opts = base_opts()
opts.seeds = 1:4;
opts.tune_seeds = 1:2;
opts.theta0_deg_set = [0 3];
opts.tune_theta0_deg_set = [0 3];
opts.Kp_grid = [5 10 15 20 30 45 60];
opts.Kd_grid = [4 8 12 16 24];
opts.success_rms_error_deg = 7.0;
opts.success_peak_error_deg = 18.0;
opts.success_end_error_deg = 7.0;
opts.success_max_theta_deg = 65.0;
end


function profiles = actuator_profiles()
profiles = struct([]);
profiles(1).name = 'HOBBY_NOMINAL';
profiles(1).cmd_deg = 20;
profiles(1).cmd_start_s = 0.55;
profiles(1).cmd_ramp_s = 0.75;
profiles(1).cmd_hold_s = 0.85;
profiles(1).t_end = 3.60;
profiles(1).slew_code = 60;
profiles(1).u_max_code = 12;
profiles(1).servo_deadband = 0.05;
profiles(1).servo_backlash = 0.10;
profiles(1).sensor_latency_steps = 3;

profiles(2).name = 'SLOPPY_TVC';
profiles(2).cmd_deg = 25;
profiles(2).cmd_start_s = 0.55;
profiles(2).cmd_ramp_s = 0.60;
profiles(2).cmd_hold_s = 0.75;
profiles(2).t_end = 3.35;
profiles(2).slew_code = 30;
profiles(2).u_max_code = 10;
profiles(2).servo_deadband = 0.15;
profiles(2).servo_backlash = 0.30;
profiles(2).sensor_latency_steps = 4;

profiles(3).name = 'HARD_LIMIT';
profiles(3).cmd_deg = 25;
profiles(3).cmd_start_s = 0.55;
profiles(3).cmd_ramp_s = 0.55;
profiles(3).cmd_hold_s = 0.70;
profiles(3).t_end = 3.25;
profiles(3).slew_code = 20;
profiles(3).u_max_code = 8;
profiles(3).servo_deadband = 0.20;
profiles(3).servo_backlash = 0.40;
profiles(3).sensor_latency_steps = 4;
end


function cfg = base_config(p_unstable, profile)
cfg = rocket_defaults();
cfg.t_end_demo = profile.t_end;
cfg.plant.theta0 = 0;
cfg.plant.aero_damp = 0.5;
cfg.plant.control_eff = 8.0;
cfg.plant.keff_nom = 8.0;
cfg.plant.p_unstable = p_unstable;
cfg.plant.tau_act = 0.05;
cfg.plant.slew_max = profile.slew_code;
cfg.plant.u_max = profile.u_max_code;
cfg.controllers.PID = struct('Kp', 20, 'Ki', 0, 'Kd', 8, 'u_max', cfg.plant.u_max, 'i_lim', cfg.plant.u_max);
cfg.controllers.PCH_LQR.u_max = cfg.plant.u_max;
cfg.controllers.PCH_LQR.K_nominal = [20 8];
cfg.controllers.PCH_LQR.keff_nom = cfg.plant.keff_nom;
cfg.controllers.PCH_LQR.aero_damp = cfg.plant.aero_damp;
cfg.controllers.PCH_LQR.tau_act_assumed = cfg.plant.tau_act;
cfg.controllers.PCH_LQR.slew_max_assumed = cfg.plant.slew_max;
end


function sc = pitch_program_scenario(cfg, profile)
sc = rocket_scenario('nominal', cfg);
sc.kind = "PITCH_PROGRAM_STRESS";
sc.t_end = profile.t_end;
sc.disturbance_amp = 0.05;
sc.disturbance_freq_hz = 0.80;
sc.theta_ref_fun = @(t) pitch_ref_rad(t, profile);
sc.q_ref_fun = @(t) pitch_rate_ref_rad_s(t, profile);
end


function theta = pitch_ref_rad(t, profile)
amp = deg2rad(profile.cmd_deg);
t0 = profile.cmd_start_s;
t1 = t0 + profile.cmd_ramp_s;
t2 = t1 + profile.cmd_hold_s;
t3 = t2 + profile.cmd_ramp_s;
if t < t0
    theta = 0;
elseif t < t1
    theta = amp * (t - t0) / profile.cmd_ramp_s;
elseif t < t2
    theta = amp;
elseif t < t3
    theta = amp * (1 - (t - t2) / profile.cmd_ramp_s);
else
    theta = 0;
end
end


function q_ref = pitch_rate_ref_rad_s(t, profile)
amp = deg2rad(profile.cmd_deg);
t0 = profile.cmd_start_s;
t1 = t0 + profile.cmd_ramp_s;
t2 = t1 + profile.cmd_hold_s;
t3 = t2 + profile.cmd_ramp_s;
if t >= t0 && t < t1
    q_ref = amp / profile.cmd_ramp_s;
elseif t >= t2 && t < t3
    q_ref = -amp / profile.cmd_ramp_s;
else
    q_ref = 0;
end
end


function realism = full_realistic_profile(profile, gust_std)
realism = struct( ...
    'gyro_noise_std', 0.015, 'gyro_bias_init', 0.010, 'gyro_bias_rw', 0.005, ...
    'gyro_quant_lsb', 2 * pi / 4000, 'bias_cal_residual_frac', 0.10, ...
    'sensor_latency_steps', profile.sensor_latency_steps, 'servo_deadband', profile.servo_deadband, ...
    'servo_backlash', profile.servo_backlash, 'servo_pos_noise', 0.01, ...
    'gust_std', gust_std, 'gust_tau', 0.40, 'keff_drift_rate', 0.10, ...
    'theta_init_bias', deg2rad(0.5));
end


function realism = ideal_profile(~, gust_std)
realism = struct( ...
    'gyro_noise_std', 0.0, 'gyro_bias_init', 0.0, 'gyro_bias_rw', 0.0, ...
    'gyro_quant_lsb', 0.0, 'bias_cal_residual_frac', 0.0, ...
    'sensor_latency_steps', 1, 'servo_deadband', 0.0, 'servo_backlash', 0.0, ...
    'servo_pos_noise', 0.0, 'gust_std', gust_std, 'gust_tau', 0.40, ...
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
tune_opts = opts;
tune_opts.seeds = opts.tune_seeds;
tune_opts.theta0_deg_set = opts.tune_theta0_deg_set;
best_rate = -inf;
best_rms = inf;
best_Kp = NaN;
best_Kd = NaN;
best_metrics = struct('success_rate', NaN, 'rms_error_deg', NaN);
for Kp = opts.Kp_grid
    for Kd = opts.Kd_grid
        cfg_try = configure_controller(cfg, ctrl_name, Kp, Kd);
        metrics = evaluate_controller(ctrl_name, cfg_try, sc, tune_opts, realism);
        if metrics.success_rate > best_rate || ...
           (abs(metrics.success_rate - best_rate) < 1e-9 && metrics.rms_error_deg < best_rms)
            best_rate = metrics.success_rate;
            best_rms = metrics.rms_error_deg;
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
                u_sat_col(idx), slew_sat_col(idx)] = score_run(out, cfg, opts);
        catch
            success_col(idx) = false;
            rms_col(idx) = 90;
            peak_col(idx) = 90;
            end_col(idx) = 90;
            max_theta_col(idx) = 90;
            u_sat_col(idx) = 1;
            slew_sat_col(idx) = 1;
        end
    end
end

metrics.success_rate = mean(success_col);
metrics.rms_error_deg = mean(rms_col);
metrics.peak_error_deg = mean(peak_col);
metrics.end_error_deg = mean(end_col);
metrics.u_cmd_sat_frac = mean(u_sat_col);
metrics.slew_sat_frac = mean(slew_sat_col);
metrics.trials = table(seed_col, theta0_col, success_col, rms_col, peak_col, end_col, ...
    max_theta_col, u_sat_col, slew_sat_col, 'VariableNames', ...
    {'seed', 'theta0_deg', 'success', 'rms_error_deg', 'peak_error_deg', ...
    'end_error_deg', 'max_theta_deg', 'u_cmd_sat_frac', 'slew_sat_frac'});
end


function [success, rms_error_deg, peak_error_deg, end_error_deg, max_theta_deg, u_sat_frac, slew_sat_frac] = score_run(out, cfg, opts)
err = out.theta - out.theta_ref;
if ~all(isfinite(out.theta)) || ~all(isfinite(err))
    rms_error_deg = 90;
    peak_error_deg = 90;
    end_error_deg = 90;
    max_theta_deg = 90;
    u_sat_frac = 1;
    slew_sat_frac = 1;
    success = false;
    return;
end
rms_error_deg = min(90, rad2deg(sqrt(mean(err.^2))));
peak_error_deg = min(90, rad2deg(max(abs(err))));
end_error_deg = min(90, rad2deg(abs(err(end))));
max_theta_deg = min(90, rad2deg(max(abs(out.theta))));
u_sat_frac = mean(abs(out.u_cmd) >= 0.99 * cfg.plant.u_max);
slew_sat_frac = mean(abs(diff(out.u_act)) / cfg.dt >= 0.99 * cfg.plant.slew_max);
success = rms_error_deg <= opts.success_rms_error_deg && ...
          peak_error_deg <= opts.success_peak_error_deg && ...
          end_error_deg <= opts.success_end_error_deg && ...
          max_theta_deg <= opts.success_max_theta_deg;
end


function plot_profile_bars(T, methods, profiles, out_path)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1050 420]);
mean_success = zeros(numel(profiles), numel(methods));
for ip = 1:numel(profiles)
    for im = 1:numel(methods)
        mask = T.profile == string(profiles(ip).name) & T.method == methods(im);
        mean_success(ip, im) = mean(T.deploy_success_rate(mask));
    end
end
bar(mean_success);
ylim([0 1]);
grid on;
ax = gca;
ax.TickLabelInterpreter = 'none';
xticklabels({profiles.name});
ylabel('mean success across p/wind cells');
legend(cellstr(methods), 'Location', 'southoutside', 'Orientation', 'horizontal', 'Interpreter', 'none');
title('Maneuver success under increasingly sloppy TVC envelopes');
exportgraphics(fig, out_path, 'Resolution', 200);
close(fig);
end


function plot_best_delta(T, profiles, p_list, gust_list, out_path)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1140 380]);
for iprof = 1:numel(profiles)
    M = nan(numel(p_list), numel(gust_list));
    for ip = 1:numel(p_list)
        for ig = 1:numel(gust_list)
            mask_base = T.profile == string(profiles(iprof).name) & T.method == "AMATEUR_NOMINAL_PID" & ...
                T.p_unstable == p_list(ip) & abs(T.gust_std - gust_list(ig)) < 1e-9;
            mask_best = T.profile == string(profiles(iprof).name) & T.method == "BENCH_TUNED_PCH" & ...
                T.p_unstable == p_list(ip) & abs(T.gust_std - gust_list(ig)) < 1e-9;
            if any(mask_base) && any(mask_best)
                M(ip, ig) = T.deploy_success_rate(mask_best) - T.deploy_success_rate(mask_base);
            end
        end
    end
    subplot(1, numel(profiles), iprof);
    imagesc(gust_list, p_list, M, [-1 1]);
    axis xy;
    colormap(gca, redblue_map());
    colorbar;
    xticks(gust_list);
    yticks(p_list);
    xlabel('gust std (rad/s^2)');
    ylabel('p unstable (1/s)');
    title(sprintf('%s: PCH - nominal PID', strrep(profiles(iprof).name, '_', '\_')));
    for ip = 1:numel(p_list)
        for ig = 1:numel(gust_list)
            text(gust_list(ig), p_list(ip), sprintf('%+.2f', M(ip, ig)), ...
                'HorizontalAlignment', 'center', 'Color', 'k', 'FontWeight', 'bold');
        end
    end
end
exportgraphics(fig, out_path, 'Resolution', 200);
close(fig);
end


function plot_workflow_delta(T, profiles, p_list, gust_list, out_path)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1140 380]);
for iprof = 1:numel(profiles)
    M = nan(numel(p_list), numel(gust_list));
    for ip = 1:numel(p_list)
        for ig = 1:numel(gust_list)
            common = T.profile == string(profiles(iprof).name) & ...
                T.p_unstable == p_list(ip) & abs(T.gust_std - gust_list(ig)) < 1e-9;
            amateur_mask = common & (T.method == "AMATEUR_NOMINAL_PID" | T.method == "CLEAN_RETUNED_PID");
            workflow_mask = common & (T.method == "BENCH_TUNED_PID" | T.method == "BENCH_TUNED_PCH");
            if any(amateur_mask) && any(workflow_mask)
                M(ip, ig) = max(T.deploy_success_rate(workflow_mask)) - max(T.deploy_success_rate(amateur_mask));
            end
        end
    end
    subplot(1, numel(profiles), iprof);
    imagesc(gust_list, p_list, M, [-1 1]);
    axis xy;
    colormap(gca, redblue_map());
    colorbar;
    xticks(gust_list);
    yticks(p_list);
    xlabel('gust std (rad/s^2)');
    ylabel('p unstable (1/s)');
    title(sprintf('%s: workflow - amateur', strrep(profiles(iprof).name, '_', '\_')));
    for ip = 1:numel(p_list)
        for ig = 1:numel(gust_list)
            text(gust_list(ig), p_list(ip), sprintf('%+.2f', M(ip, ig)), ...
                'HorizontalAlignment', 'center', 'Color', 'k', 'FontWeight', 'bold');
        end
    end
end
exportgraphics(fig, out_path, 'Resolution', 200);
close(fig);
end


function cmap = redblue_map()
n = 256;
x = linspace(-1, 1, n)';
cmap = [max(0, -x), 1 - abs(x), max(0, x)];
cmap = 0.15 + 0.85 * cmap;
end