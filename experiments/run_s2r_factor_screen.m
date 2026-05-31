% run_s2r_factor_screen.m
% Regime-aware sim-to-real screen:
% tune PD on an idealized plant, then measure which realism blocks hurt most
% in each regime. This supports the new paper direction by identifying the
% smallest set of nonidealities that must be carried into the preflight tool.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));

cfg0 = rocket_defaults();

regimes = {
    'LOW_DEMAND',       4, 18.0, 10.2;
    'BOUNDARY',         8, 12.0,  6.0;
    'ACTUATOR_LIMITED',10,  9.0,  4.5;
};

profiles = factor_profiles();

opts.theta0_deg_set   = [3 6];
opts.seeds            = 1:3;
opts.Kp_grid          = [1 3 10 30];
opts.Kd_grid          = [0.2 1 3 10];
opts.success_rms_deg  = 10.0;
opts.success_peak_deg = 40.0;
opts.success_end_deg  = 8.0;
opts.disturb_amp      = 0.10;
opts.disturb_freq_hz  = 0.80;
opts.t_end            = 3.0;

rows = {};
for ir = 1:size(regimes, 1)
    name = regimes{ir, 1};
    p_val = regimes{ir, 2};
    slew_code = regimes{ir, 3};
    umax_code = regimes{ir, 4};

    cfg = cfg0;
    cfg.t_end_demo = opts.t_end;
    cfg.plant.p_unstable = p_val;
    cfg.plant.slew_max = slew_code;
    cfg.plant.u_max = umax_code;
    cfg.controllers.PID.Ki = 0.0;
    cfg.controllers.PID.u_max = umax_code;
    cfg.controllers.PID.i_lim = umax_code;

    [best_Kp, best_Kd, ideal_rate, ideal_rms, ideal_peak] = tune_pd(cfg, opts, profiles.IDEAL);
    fprintf('\n=== %s ===\n', name);
    fprintf('Ideal-trained PD: Kp=%.2f  Kd=%.2f  success=%.2f\n', best_Kp, best_Kd, ideal_rate);

    cfg.controllers.PID.Kp = best_Kp;
    cfg.controllers.PID.Kd = best_Kd;
    profile_names = fieldnames(profiles);
    for ip = 1:numel(profile_names)
        prof_name = profile_names{ip};
        realism = profiles.(prof_name);
        [rate, mean_rms_deg, mean_peak_deg] = evaluate_pid(cfg, opts, realism);
        fprintf('  %-22s success=%.2f  drop=%.2f  rms=%.2f deg\n', ...
            prof_name, rate, ideal_rate - rate, mean_rms_deg);

        rows(end+1, :) = { ...
            string(name), string(prof_name), p_val, slew_code, umax_code, ...
            best_Kp, best_Kd, ideal_rate, rate, ideal_rate - rate, ...
            ideal_rms, mean_rms_deg, ideal_peak, mean_peak_deg}; %#ok<SAGROW>
    end
end

T = cell2table(rows, 'VariableNames', { ...
    'regime', 'factor_profile', 'p_unstable', 'slew_code', 'u_max_code', ...
    'tuned_Kp', 'tuned_Kd', 'ideal_success', 'profile_success', 'success_drop', ...
    'ideal_rms_deg', 'profile_rms_deg', 'ideal_peak_deg', 'profile_peak_deg'});
writetable(T, fullfile(here, 'results', 's2r_factor_screen.csv'));
fprintf('\nSaved: experiments/results/s2r_factor_screen.csv\n');
fprintf('DONE.\n');


function profiles = factor_profiles()
profiles = struct();

profiles.IDEAL = struct( ...
    'gyro_noise_std', 0.0, ...
    'gyro_bias_init', 0.0, ...
    'gyro_bias_rw', 0.0, ...
    'gyro_quant_lsb', 0.0, ...
    'bias_cal_residual_frac', 0.0, ...
    'sensor_latency_steps', 1, ...
    'servo_deadband', 0.0, ...
    'servo_backlash', 0.0, ...
    'servo_pos_noise', 0.0, ...
    'gust_std', 0.0, ...
    'gust_tau', 0.40, ...
    'keff_drift_rate', 0.0, ...
    'theta_init_bias', 0.0);

profiles.SENSOR_LATENCY = copy_with(profiles.IDEAL, struct( ...
    'sensor_latency_steps', 4));

profiles.SENSOR_NOISE = copy_with(profiles.IDEAL, struct( ...
    'gyro_noise_std', 0.015, ...
    'gyro_bias_init', 0.010, ...
    'gyro_bias_rw', 0.005, ...
    'gyro_quant_lsb', 2 * pi / 4000, ...
    'bias_cal_residual_frac', 0.10, ...
    'theta_init_bias', deg2rad(0.5)));

profiles.ACTUATOR_NONLINEARITY = copy_with(profiles.IDEAL, struct( ...
    'servo_deadband', 0.05, ...
    'servo_backlash', 0.10, ...
    'servo_pos_noise', 0.01));

profiles.GUST = copy_with(profiles.IDEAL, struct( ...
    'gust_std', 0.30, ...
    'gust_tau', 0.40));

profiles.KEFF_DRIFT = copy_with(profiles.IDEAL, struct( ...
    'keff_drift_rate', 0.10));

profiles.ALL_REALISTIC = struct( ...
    'gyro_noise_std', 0.015, ...
    'gyro_bias_init', 0.010, ...
    'gyro_bias_rw', 0.005, ...
    'gyro_quant_lsb', 2 * pi / 4000, ...
    'bias_cal_residual_frac', 0.10, ...
    'sensor_latency_steps', 2, ...
    'servo_deadband', 0.05, ...
    'servo_backlash', 0.10, ...
    'servo_pos_noise', 0.01, ...
    'gust_std', 0.30, ...
    'gust_tau', 0.40, ...
    'keff_drift_rate', 0.10, ...
    'theta_init_bias', deg2rad(0.5));
end


function [best_Kp, best_Kd, best_rate, best_rms, best_peak] = tune_pd(cfg, opts, realism)
best_rate = -inf;
best_rms = inf;
best_peak = inf;
best_Kp = NaN;
best_Kd = NaN;

for Kp = opts.Kp_grid
    for Kd = opts.Kd_grid
        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Kd = Kd;
        [rate, mean_rms_deg, mean_peak_deg] = evaluate_pid(cfg, opts, realism);
        if rate > best_rate || (abs(rate - best_rate) < 1e-9 && mean_rms_deg < best_rms)
            best_rate = rate;
            best_rms = mean_rms_deg;
            best_peak = mean_peak_deg;
            best_Kp = Kp;
            best_Kd = Kd;
        end
    end
end
end


function [rate, mean_rms_deg, mean_peak_deg] = evaluate_pid(cfg, opts, realism)
sc = rocket_scenario('nominal', cfg);
sc.t_end = opts.t_end;
sc.disturbance_amp = opts.disturb_amp;
sc.disturbance_freq_hz = opts.disturb_freq_hz;

wins = 0;
n_trials = 0;
rms_vals = zeros(numel(opts.seeds) * numel(opts.theta0_deg_set), 1);
peak_vals = zeros(numel(opts.seeds) * numel(opts.theta0_deg_set), 1);
idx = 0;

for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1;
        n_trials = n_trials + 1;
        try
            out = simulate_case_realistic('PID', sc, cfg, opts.seeds(iseed) + 100 * it, realism);
            theta_rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
            theta_peak_deg = rad2deg(max(abs(out.theta)));
            theta_end_deg = rad2deg(abs(out.theta(end)));
            if ~all(isfinite(out.theta))
                theta_rms_deg = 90;
                theta_peak_deg = 90;
                theta_end_deg = 90;
            else
                theta_rms_deg = min(theta_rms_deg, 90);
                theta_peak_deg = min(theta_peak_deg, 90);
                theta_end_deg = min(theta_end_deg, 90);
            end
            rms_vals(idx) = theta_rms_deg;
            peak_vals(idx) = theta_peak_deg;
            stable = all(isfinite(out.theta)) && ...
                     theta_rms_deg <= opts.success_rms_deg && ...
                     theta_peak_deg <= opts.success_peak_deg && ...
                     theta_end_deg <= opts.success_end_deg;
            wins = wins + double(stable);
        catch
            rms_vals(idx) = 90;
            peak_vals(idx) = 90;
        end
    end
end

rate = wins / n_trials;
mean_rms_deg = mean(rms_vals);
mean_peak_deg = mean(peak_vals);
end


function out = copy_with(base, updates)
out = base;
fns = fieldnames(updates);
for i = 1:numel(fns)
    out.(fns{i}) = updates.(fns{i});
end
end