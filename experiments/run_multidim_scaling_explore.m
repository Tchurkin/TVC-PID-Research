% run_multidim_scaling_explore.m
% Exploratory test for a deeper simulator-side discovery:
%   At fixed control authority, does the minimum slew needed for
%   stabilization collapse into a clean joint power law over
%   p_unstable, sensor latency, and gyro noise?
%
% We give each cell a fair chance by coarse-retuning PD gains.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));

cfg = rocket_defaults();
cfg.t_end_demo = 3.0;
cfg.plant.aero_damp = 0.5;
cfg.plant.control_eff = 8.0;
cfg.plant.u_max = 12.0;

p_list = [4 6 8 10];
latency_steps_list = [1 2 4 8];
gyro_noise_list = [0.005 0.010 0.020 0.040];
slew_code_list = [6 9 12 18 24 36];

Kp_grid = [1 3 10 30];
Kd_grid = [0.3 1 3 10];

success_thr = 0.50;
seeds = 1:2;
theta0_list = deg2rad([3 6]);

rows = [];
best_gain_rows = [];

fprintf('=== MULTI-DIM SCALING EXPLORATION ===\n');
fprintf('Cells: %d\n', numel(p_list) * numel(latency_steps_list) * numel(gyro_noise_list) * numel(slew_code_list));

for ip = 1:numel(p_list)
    p_unstable = p_list(ip);
    cfg.plant.p_unstable = p_unstable;
    fprintf('\n--- p = %g rad/s ---\n', p_unstable);

    for il = 1:numel(latency_steps_list)
        latency_steps = latency_steps_list(il);

        for in = 1:numel(gyro_noise_list)
            gyro_noise_std = gyro_noise_list(in);

            for is = 1:numel(slew_code_list)
                slew_code = slew_code_list(is);
                cfg.plant.slew_max = slew_code;
                cfg.controllers.PID.u_max = cfg.plant.u_max;
                cfg.controllers.PID.i_lim = cfg.plant.u_max;
                cfg.controllers.PID.Ki = 0.0;

                realism = struct( ...
                    'gyro_noise_std', gyro_noise_std, ...
                    'sensor_latency_steps', latency_steps, ...
                    'gust_std', 0.30, ...
                    'gust_tau', 0.40, ...
                    'servo_deadband', 0.05, ...
                    'servo_backlash', 0.10, ...
                    'gyro_bias_rw', 0.005, ...
                    'gyro_bias_init', 0.010, ...
                    'gyro_quant_lsb', 2*pi/4000, ...
                    'servo_pos_noise', 0.01, ...
                    'theta_init_bias', deg2rad(0.5));

                best_rate = -inf;
                best_theta_rms = inf;
                best_Kp = NaN;
                best_Kd = NaN;

                for Kp = Kp_grid
                    for Kd = Kd_grid
                        cfg.controllers.PID.Kp = Kp;
                        cfg.controllers.PID.Kd = Kd;
                        [rate, mean_theta_rms] = evaluate_pid_cell(cfg, realism, seeds, theta0_list);
                        if rate > best_rate || (abs(rate - best_rate) < 1e-9 && mean_theta_rms < best_theta_rms)
                            best_rate = rate;
                            best_theta_rms = mean_theta_rms;
                            best_Kp = Kp;
                            best_Kd = Kd;
                        end
                    end
                end

                rows(end+1,:) = [ ...
                    p_unstable, latency_steps, cfg.dt * latency_steps, gyro_noise_std, ...
                    slew_code, best_rate, best_theta_rms]; %#ok<SAGROW>
                best_gain_rows(end+1,:) = [best_Kp, best_Kd]; %#ok<SAGROW>
            end
        end
    end
end

cellsT = array2table([rows, best_gain_rows], 'VariableNames', ...
    {'p_unstable','latency_steps','latency_s','gyro_noise_std','slew_code_units', ...
     'best_success','best_theta_rms_deg','best_Kp','best_Kd'});
writetable(cellsT, fullfile(here, 'results', 'multidim_scaling_cells.csv'));

boundary_rows = [];
for ip = 1:numel(p_list)
    for il = 1:numel(latency_steps_list)
        for in = 1:numel(gyro_noise_list)
            mask = cellsT.p_unstable == p_list(ip) & ...
                   cellsT.latency_steps == latency_steps_list(il) & ...
                   abs(cellsT.gyro_noise_std - gyro_noise_list(in)) < 1e-12;
            sliceT = sortrows(cellsT(mask,:), 'slew_code_units');
            good = find(sliceT.best_success >= success_thr, 1, 'first');
            if isempty(good)
                slew_boundary = NaN;
            else
                slew_boundary = sliceT.slew_code_units(good);
            end
            boundary_rows(end+1,:) = [ ...
                p_list(ip), latency_steps_list(il), cfg.dt * latency_steps_list(il), ...
                gyro_noise_list(in), slew_boundary]; %#ok<SAGROW>
        end
    end
end

boundaryT = array2table(boundary_rows, 'VariableNames', ...
    {'p_unstable','latency_steps','latency_s','gyro_noise_std','slew_boundary_code'});
writetable(boundaryT, fullfile(here, 'results', 'multidim_scaling_boundary.csv'));

valid = isfinite(boundaryT.slew_boundary_code);
if sum(valid) >= 8
    X = [ones(sum(valid),1), log(boundaryT.p_unstable(valid)), ...
         log(boundaryT.latency_s(valid)), log(boundaryT.gyro_noise_std(valid))];
    y = log(boundaryT.slew_boundary_code(valid));
    coef = X \ y;
    pred_log = X * coef;
    pred = exp(pred_log);
    obs = boundaryT.slew_boundary_code(valid);
    ss_res = sum((obs - pred).^2);
    ss_tot = sum((obs - mean(obs)).^2);
    R2 = 1 - ss_res / max(ss_tot, eps);

    fitT = table(exp(coef(1)), coef(2), coef(3), coef(4), R2, ...
        'VariableNames', {'coef_a','alpha_p','beta_latency','gamma_noise','R2'});
    writetable(fitT, fullfile(here, 'results', 'multidim_scaling_fit.csv'));

    fprintf('\n=== JOINT POWER-LAW FIT ===\n');
    fprintf('slew_boundary = %.3f * p^%.3f * latency_s^%.3f * gyro_noise^%.3f\n', ...
        exp(coef(1)), coef(2), coef(3), coef(4));
    fprintf('R^2 = %.3f\n', R2);
else
    warning('Not enough valid boundary cells to fit a joint power law.');
end

fprintf('\nSaved: experiments/results/multidim_scaling_cells.csv\n');
fprintf('Saved: experiments/results/multidim_scaling_boundary.csv\n');
fprintf('DONE.\n');


function [rate, mean_theta_rms_deg] = evaluate_pid_cell(cfg, realism, seeds, theta0_list)
sc = rocket_scenario('nominal', cfg);
sc.t_end = cfg.t_end_demo;
sc.disturbance_amp = 0.05;
sc.disturbance_freq_hz = 0.8;

wins = 0;
n_trials = 0;
theta_rms_deg_vals = zeros(numel(seeds) * numel(theta0_list), 1);
idx = 0;

for it = 1:numel(theta0_list)
    cfg.plant.theta0 = theta0_list(it);
    for iseed = 1:numel(seeds)
        idx = idx + 1;
        n_trials = n_trials + 1;
        try
            out = simulate_case_realistic('PID', sc, cfg, seeds(iseed) + 100 * it, realism);
            theta_rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
            theta_rms_deg_vals(idx) = theta_rms_deg;
            stable = all(isfinite(out.theta)) && ...
                     max(abs(out.theta)) < deg2rad(40) && ...
                     theta_rms_deg <= 10 && ...
                     abs(out.theta(end)) < deg2rad(8);
            wins = wins + double(stable);
        catch
            theta_rms_deg_vals(idx) = 90;
        end
    end
end

rate = wins / n_trials;
mean_theta_rms_deg = mean(theta_rms_deg_vals);
end