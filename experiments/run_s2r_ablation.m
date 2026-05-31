% run_s2r_ablation.m
% Leave-one-out sim-to-real ablation.
%
% Methodology (the FIXED version of run_s2r_factor_screen.m):
%   1) Build one canonical FULL_REALISTIC realism profile.
%   2) For each regime (LOW_DEMAND / BOUNDARY / ACTUATOR_LIMITED), tune the
%      best PD on the full realistic plant in that regime. This is the
%      controller a measurement-aware preflight tool would deploy.
%   3) For each factor (latency, gyro noise, deadband+backlash, gust,
%      keff drift), build an ABLATED world that is identical to
%      FULL_REALISTIC except that single factor is reset to its IDEAL value.
%   4) Hold PD fixed and evaluate on the ablated world.
%   5) The success RECOVERY = ablated_success - full_success tells us how
%      much of the failure that single factor was responsible for. The
%      factor with the largest positive recovery is the dominant problem.
%
% This avoids the pathology of the old factor screen, where each factor was
% added in isolation to an ideal world (so a factor that only matters in
% combination with another factor looked harmless) and where ALL_REALISTIC
% was not the union of the single-factor profiles.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here, 'results'), 'dir'); mkdir(fullfile(here, 'results')); end

cfg0 = rocket_defaults();

regimes = {
    'LOW_DEMAND',       4, 18.0, 10.2;
    'BOUNDARY',         8, 12.0,  6.0;
    'ACTUATOR_LIMITED',10,  9.0,  4.5;
};

full_realism = full_realistic_profile();
ideal_realism = ideal_profile();

% Factors to ablate, each one resets a subset of fields to their IDEAL values.
ablations = {
    'NONE_FULL_REAL',     {};
    'NO_SENSOR_LATENCY',  {'sensor_latency_steps'};
    'NO_SENSOR_NOISE',    {'gyro_noise_std','gyro_bias_init','gyro_bias_rw','gyro_quant_lsb','bias_cal_residual_frac','theta_init_bias'};
    'NO_ACTUATOR_NONLIN', {'servo_deadband','servo_backlash','servo_pos_noise'};
    'NO_GUST',            {'gust_std'};
    'NO_KEFF_DRIFT',      {'keff_drift_rate'};
};

opts.theta0_deg_set   = [3 6];
opts.seeds            = 1:6;          % 12 trials per evaluation
opts.Kp_grid          = [1 3 10 30];
opts.Kd_grid          = [0.3 1 3 10];
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

    % Tune PD on FULL_REALISTIC for this regime.
    [best_Kp, best_Kd, baseline_rate, baseline_rms] = tune_pd(cfg, opts, full_realism);
    fprintf('\n=== %s (p=%d, slew=%.1f, umax=%.1f) ===\n', name, p_val, slew_code, umax_code);
    fprintf('Tuned-on-full PD: Kp=%.2f  Kd=%.2f  baseline_success=%.3f (CI %.3f)\n', ...
        best_Kp, best_Kd, baseline_rate, wilson_halfwidth(baseline_rate, n_trials(opts)));

    cfg.controllers.PID.Kp = best_Kp;
    cfg.controllers.PID.Kd = best_Kd;

    for ia = 1:size(ablations, 1)
        ab_name = ablations{ia, 1};
        ab_fields = ablations{ia, 2};
        ablated = full_realism;
        for k = 1:numel(ab_fields)
            f = ab_fields{k};
            ablated.(f) = ideal_realism.(f);
        end
        [rate, rms_deg, peak_deg] = evaluate_pid(cfg, opts, ablated);
        recovery = rate - baseline_rate;
        ci_half = wilson_halfwidth(rate, n_trials(opts));
        fprintf('  %-22s success=%.3f (CI %.3f)  recovery=%+0.3f\n', ...
            ab_name, rate, ci_half, recovery);

        rows(end+1, :) = { ...
            string(name), string(ab_name), p_val, slew_code, umax_code, ...
            best_Kp, best_Kd, baseline_rate, rate, recovery, ci_half, ...
            rms_deg, peak_deg}; %#ok<SAGROW>
    end
end

T = cell2table(rows, 'VariableNames', { ...
    'regime', 'ablation', 'p_unstable', 'slew_code', 'u_max_code', ...
    'tuned_Kp', 'tuned_Kd', 'full_real_success', 'ablated_success', ...
    'success_recovery', 'ci_halfwidth_95', 'ablated_rms_deg', 'ablated_peak_deg'});
writetable(T, fullfile(here, 'results', 's2r_ablation.csv'));
fprintf('\nSaved: experiments/results/s2r_ablation.csv\n');
fprintf('DONE.\n');


function nt = n_trials(opts)
nt = numel(opts.seeds) * numel(opts.theta0_deg_set);
end

function hw = wilson_halfwidth(p, n)
% 95% Wilson score interval halfwidth for a binomial proportion.
z = 1.96;
denom = 1 + z^2 / n;
center = (p + z^2 / (2 * n)) / denom;
margin = z * sqrt(p * (1 - p) / n + z^2 / (4 * n^2)) / denom;
% halfwidth around the empirical p
hw = margin;
end


function realism = full_realistic_profile()
realism = struct( ...
    'gyro_noise_std', 0.015, ...
    'gyro_bias_init', 0.010, ...
    'gyro_bias_rw', 0.005, ...
    'gyro_quant_lsb', 2 * pi / 4000, ...
    'bias_cal_residual_frac', 0.10, ...
    'sensor_latency_steps', 3, ...
    'servo_deadband', 0.05, ...
    'servo_backlash', 0.10, ...
    'servo_pos_noise', 0.01, ...
    'gust_std', 0.30, ...
    'gust_tau', 0.40, ...
    'keff_drift_rate', 0.10, ...
    'theta_init_bias', deg2rad(0.5));
end


function realism = ideal_profile()
realism = struct( ...
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
end


function [best_Kp, best_Kd, best_rate, best_rms] = tune_pd(cfg, opts, realism)
best_rate = -inf;
best_rms = inf;
best_Kp = NaN;
best_Kd = NaN;
for Kp = opts.Kp_grid
    for Kd = opts.Kd_grid
        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Kd = Kd;
        [rate, rms_deg] = evaluate_pid(cfg, opts, realism);
        if rate > best_rate || (abs(rate - best_rate) < 1e-9 && rms_deg < best_rms)
            best_rate = rate;
            best_rms = rms_deg;
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

wins = 0; n = 0; idx = 0;
nT = numel(opts.seeds) * numel(opts.theta0_deg_set);
rms_vals = zeros(nT, 1);
peak_vals = zeros(nT, 1);

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
            rms_vals(idx) = rms_deg;
            peak_vals(idx) = peak_deg;
            stable = all(isfinite(out.theta)) && ...
                     rms_deg <= opts.success_rms_deg && ...
                     peak_deg <= opts.success_peak_deg && ...
                     end_deg <= opts.success_end_deg;
            wins = wins + double(stable);
        catch
            rms_vals(idx) = 90;
            peak_vals(idx) = 90;
        end
    end
end
rate = wins / n;
mean_rms_deg = mean(rms_vals);
mean_peak_deg = mean(peak_vals);
end
