% run_basic_vs_better_sim.m
% Designer-sim ablation that directly answers user-question 2.2:
%   What if a hobbyist designs their PID on a BASIC sim (no realism — just
%   the ideal pendulum) vs designing on a BETTER sim (the same realistic
%   model the bench-calibrated preflight tool would use)?
%
% Both controllers are then deployed on the same realistic plant. If the
% better-sim controller dominates, that is the simulator-side justification
% for the bench-calibrated preflight pipeline: it removes the modelling
% optimism that comes from designing against a too-clean sim.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here, 'results'), 'dir'); mkdir(fullfile(here, 'results')); end

cfg0 = rocket_defaults();
cfg0.plant.aero_damp = 0.5;
cfg0.plant.control_eff = 8.0;
cfg0.plant.keff_nom = 8.0;
cfg0.t_end_demo = 3.0;

% Sweep across regimes by p and actuator margin. Same plant/actuator for
% both design philosophies; only the realism model used during tuning differs.
cases = {
    'LOW_DEMAND',        4, 18.0, 10.2;
    'BOUNDARY',          8, 12.0,  6.0;
    'NEAR_LIMIT',       10,  9.0,  4.5;
    'AGGRESSIVE',       12,  9.0,  4.5;
};

opts.theta0_deg_set   = [3 6];
opts.seeds            = 1:6;
opts.Kp_grid          = [1 3 10 30];
opts.Kd_grid          = [0.3 1 3 10];
opts.success_rms_deg  = 10.0;
opts.success_peak_deg = 40.0;
opts.success_end_deg  = 8.0;
opts.disturb_amp      = 0.10;
opts.disturb_freq_hz  = 0.80;
opts.t_end            = 3.0;

ideal = ideal_profile();
full_real = full_realistic_profile();

rows = {};
for ic = 1:size(cases, 1)
    cname = cases{ic, 1};
    p_val = cases{ic, 2};
    slew_code = cases{ic, 3};
    umax_code = cases{ic, 4};

    cfg = cfg0;
    cfg.plant.p_unstable = p_val;
    cfg.plant.slew_max = slew_code;
    cfg.plant.u_max = umax_code;
    cfg.controllers.PID.Ki = 0.0;
    cfg.controllers.PID.u_max = umax_code;
    cfg.controllers.PID.i_lim = umax_code;

    % BASIC-SIM design: tune PD on the ideal pendulum.
    [bKp, bKd, b_design_rate] = tune_pd(cfg, opts, ideal);
    % BETTER-SIM design: tune PD on the realistic stack.
    [gKp, gKd, g_design_rate] = tune_pd(cfg, opts, full_real);

    % Deploy both on the realistic plant.
    cfg.controllers.PID.Kp = bKp; cfg.controllers.PID.Kd = bKd;
    [b_dep, b_rms, b_peak] = evaluate_pid(cfg, opts, full_real);
    cfg.controllers.PID.Kp = gKp; cfg.controllers.PID.Kd = gKd;
    [g_dep, g_rms, g_peak] = evaluate_pid(cfg, opts, full_real);

    fprintf('\n=== %s (p=%d slew=%.1f umax=%.1f) ===\n', cname, p_val, slew_code, umax_code);
    fprintf('BASIC_SIM_PD  : Kp=%5.1f Kd=%4.1f  design_rate=%.2f  deployed=%.2f rms=%.2f\n', ...
        bKp, bKd, b_design_rate, b_dep, b_rms);
    fprintf('BETTER_SIM_PD : Kp=%5.1f Kd=%4.1f  design_rate=%.2f  deployed=%.2f rms=%.2f\n', ...
        gKp, gKd, g_design_rate, g_dep, g_rms);

    rows(end+1,:) = {string(cname), string('BASIC_SIM_PD'),  p_val, slew_code, umax_code, bKp, bKd, b_design_rate, b_dep, b_rms, b_peak}; %#ok<SAGROW>
    rows(end+1,:) = {string(cname), string('BETTER_SIM_PD'), p_val, slew_code, umax_code, gKp, gKd, g_design_rate, g_dep, g_rms, g_peak}; %#ok<SAGROW>
end

T = cell2table(rows, 'VariableNames', { ...
    'case', 'design_policy', 'p_unstable', 'slew_code', 'u_max_code', ...
    'Kp', 'Kd', 'design_sim_success', 'realistic_deploy_success', ...
    'realistic_rms_deg', 'realistic_peak_deg'});
writetable(T, fullfile(here, 'results', 'basic_vs_better_sim.csv'));
fprintf('\nSaved: experiments/results/basic_vs_better_sim.csv\n');
fprintf('DONE.\n');


function realism = full_realistic_profile()
realism = struct( ...
    'gyro_noise_std', 0.015, 'gyro_bias_init', 0.010, 'gyro_bias_rw', 0.005, ...
    'gyro_quant_lsb', 2 * pi / 4000, 'bias_cal_residual_frac', 0.10, ...
    'sensor_latency_steps', 3, 'servo_deadband', 0.05, 'servo_backlash', 0.10, ...
    'servo_pos_noise', 0.01, 'gust_std', 0.30, 'gust_tau', 0.40, ...
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

function [best_Kp, best_Kd, best_rate] = tune_pd(cfg, opts, realism)
best_rate = -inf; best_rms = inf; best_Kp = NaN; best_Kd = NaN;
for Kp = opts.Kp_grid
    for Kd = opts.Kd_grid
        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Kd = Kd;
        [rate, rms_deg] = evaluate_pid(cfg, opts, realism);
        if rate > best_rate || (abs(rate - best_rate) < 1e-9 && rms_deg < best_rms)
            best_rate = rate; best_rms = rms_deg; best_Kp = Kp; best_Kd = Kd;
        end
    end
end
end

function [rate, mean_rms_deg, mean_peak_deg] = evaluate_pid(cfg, opts, realism)
sc = rocket_scenario('nominal', cfg);
sc.t_end = opts.t_end;
sc.disturbance_amp = opts.disturb_amp;
sc.disturbance_freq_hz = opts.disturb_freq_hz;
wins = 0; n = 0; idx = 0;
nT = numel(opts.seeds) * numel(opts.theta0_deg_set);
rms_vals = zeros(nT,1); peak_vals = zeros(nT,1);
for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1; n = n + 1;
        try
            out = simulate_case_realistic('PID', sc, cfg, opts.seeds(iseed) + 100 * it, realism);
            rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
            peak_deg = rad2deg(max(abs(out.theta)));
            end_deg = rad2deg(abs(out.theta(end)));
            if ~all(isfinite(out.theta))
                rms_deg = 90; peak_deg = 90; end_deg = 90;
            else
                rms_deg = min(rms_deg, 90);
                peak_deg = min(peak_deg, 90);
                end_deg = min(end_deg, 90);
            end
            rms_vals(idx) = rms_deg; peak_vals(idx) = peak_deg;
            stable = all(isfinite(out.theta)) && ...
                     rms_deg <= opts.success_rms_deg && ...
                     peak_deg <= opts.success_peak_deg && ...
                     end_deg <= opts.success_end_deg;
            wins = wins + double(stable);
        catch
            rms_vals(idx) = 90; peak_vals(idx) = 90;
        end
    end
end
rate = wins / n; mean_rms_deg = mean(rms_vals); mean_peak_deg = mean(peak_vals);
end
