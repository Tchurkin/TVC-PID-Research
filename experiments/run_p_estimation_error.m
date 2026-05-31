% run_p_estimation_error.m
% Tests the central preflight claim:
%   When the designer's assumed instability p_assumed differs from the true
%   plant p_true, a NAIVE controller (designed on a basic sim using the
%   wrong p_assumed) degrades; a MEASUREMENT-AWARE controller (designed on
%   a realistic sim using the correct p_true, as a bench-calibrated tool
%   would do) should stay healthy.
%
% This is the simulator-side evidence for "measurement changes the gain
% choice" — distinct from the actuator slew / u_max story already covered
% by run_preflight_nominal_vs_measured.m.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here, 'results'), 'dir'); mkdir(fullfile(here, 'results')); end

cfg0 = rocket_defaults();
cfg0.plant.aero_damp = 0.5;
cfg0.plant.control_eff = 8.0;
cfg0.plant.keff_nom = 8.0;
cfg0.t_end_demo = 3.0;

% Fixed (moderately challenging) actuator envelope: the actuator is NOT the
% bottleneck here, the plant-instability estimate is.
slew_code = 14.0;
umax_code = 8.0;

p_true_list  = [6 8 10 12];
ratio_list   = [0.70 0.85 1.00 1.15 1.30];   % p_assumed = ratio * p_true

opts.theta0_deg_set   = [3 6];
opts.seeds            = 1:5;
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

% Precompute NAIVE designs (tuned on basic/ideal sim at the assumed p).
unique_p_assumed = unique(reshape(p_true_list(:) * ratio_list(:)', 1, []));
naive = struct();
for k = 1:numel(unique_p_assumed)
    p_a = unique_p_assumed(k);
    cfg = build_cfg(cfg0, p_a, slew_code, umax_code);
    [Kp, Kd, rate] = tune_pd(cfg, opts, ideal);
    naive(k).p = p_a;
    naive(k).Kp = Kp;
    naive(k).Kd = Kd;
    naive(k).design_rate = rate;
    fprintf('NAIVE design @ p_assumed=%5.2f -> Kp=%5.1f Kd=%4.1f (ideal-sim success=%.2f)\n', ...
        p_a, Kp, Kd, rate);
end

% Precompute MEASURED designs (tuned on full-realistic sim at the TRUE p).
measured = struct();
for k = 1:numel(p_true_list)
    p_t = p_true_list(k);
    cfg = build_cfg(cfg0, p_t, slew_code, umax_code);
    [Kp, Kd, rate] = tune_pd(cfg, opts, full_real);
    measured(k).p = p_t;
    measured(k).Kp = Kp;
    measured(k).Kd = Kd;
    measured(k).design_rate = rate;
    fprintf('MEAS  design @ p_true=%5.2f    -> Kp=%5.1f Kd=%4.1f (real-sim success=%.2f)\n', ...
        p_t, Kp, Kd, rate);
end

% Deployment evaluation: every policy is evaluated on the TRUE plant
% under FULL_REALISTIC realism.
rows = {};
for it = 1:numel(p_true_list)
    p_t = p_true_list(it);
    cfg_true = build_cfg(cfg0, p_t, slew_code, umax_code);

    meas_entry = measured(arrayfun(@(s) s.p == p_t, measured));
    for ir = 1:numel(ratio_list)
        p_a = ratio_list(ir) * p_t;
        % find naive matching p_assumed (within tol)
        idx = find(abs([naive.p] - p_a) < 1e-9, 1, 'first');
        naive_entry = naive(idx);

        % NAIVE deployed on true plant
        cfg_dep = cfg_true;
        cfg_dep.controllers.PID.Kp = naive_entry.Kp;
        cfg_dep.controllers.PID.Kd = naive_entry.Kd;
        [rate_n, rms_n, peak_n] = evaluate_pid(cfg_dep, opts, full_real);

        % MEASURED deployed on true plant
        cfg_dep.controllers.PID.Kp = meas_entry.Kp;
        cfg_dep.controllers.PID.Kd = meas_entry.Kd;
        [rate_m, rms_m, peak_m] = evaluate_pid(cfg_dep, opts, full_real);

        ratio = ratio_list(ir);
        rows(end+1,:) = {string('NAIVE_BASIC_SIM'),    p_t, ratio, p_a, naive_entry.Kp, naive_entry.Kd, rate_n, rms_n, peak_n}; %#ok<SAGROW>
        rows(end+1,:) = {string('MEASURED_REAL_SIM'),  p_t, ratio, p_t, meas_entry.Kp,  meas_entry.Kd,  rate_m, rms_m, peak_m}; %#ok<SAGROW>

        fprintf('p_true=%2d ratio=%.2f (p_assumed=%5.2f): NAIVE=%.2f  MEASURED=%.2f  delta=%+0.2f\n', ...
            p_t, ratio, p_a, rate_n, rate_m, rate_m - rate_n);
    end
end

T = cell2table(rows, 'VariableNames', { ...
    'policy', 'p_true', 'p_ratio_assumed_over_true', 'p_assumed', ...
    'Kp', 'Kd', 'success_rate', 'rms_deg', 'peak_deg'});
writetable(T, fullfile(here, 'results', 'p_estimation_error.csv'));
fprintf('\nSaved: experiments/results/p_estimation_error.csv\n');
fprintf('DONE.\n');


function cfg = build_cfg(cfg0, p_val, slew_code, umax_code)
cfg = cfg0;
cfg.plant.p_unstable = p_val;
cfg.plant.slew_max = slew_code;
cfg.plant.u_max = umax_code;
cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.u_max = umax_code;
cfg.controllers.PID.i_lim = umax_code;
end

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
