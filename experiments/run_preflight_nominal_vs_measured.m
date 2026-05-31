% run_preflight_nominal_vs_measured.m
% Compare three preflight-design policies on the actual degraded plant:
%   1) NOMINAL_PD    - tuned assuming nominal actuator capability
%   2) MEASURED_PD   - tuned using the measured actuator capability
%   3) VALIDATOR_LQR - current validator recommendation on the same measured plant
%
% This is the direct sim-side argument for the new project direction:
% if measured actuator behavior matters, MEASURED_PD should outperform
% nominal-assumption tuning on degraded near-boundary cases.

clear; clc;

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(here);
addpath(fullfile(here, 'validator'));
addpath(genpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src')));

cfg0 = rocket_defaults();
cfg0.plant.aero_damp = 0.5;
cfg0.plant.control_eff = 8.0;
cfg0.plant.keff_nom = 8.0;
cfg0.t_end_demo = 3.0;

servo_max_deg_nominal = 10.0;
nominal_loaded_slew_deg_s = 90.0;
code_scale = 12.0 / deg2rad(servo_max_deg_nominal);

p_values = [6 8 10 12];
slew_scale_list = [1.00 0.70 0.50 0.35];
util_frac_list  = [1.00 0.75 0.55 0.35];

opts.theta0_deg_set   = [3 6];
opts.seeds            = 1:3;
opts.Kp_grid          = [1 3 10 30];
opts.Kd_grid          = [0.2 1 3 10];
opts.success_rms_deg  = 10.0;
opts.success_peak_deg = 40.0;
opts.success_end_deg  = 8.0;
opts.disturb_amp      = 0.15;
opts.disturb_freq_hz  = 0.80;
opts.t_end            = 3.0;

realism = all_realistic_profile();
naive_design_realism = ideal_profile();

fprintf('=== PREFLIGHT NOMINAL vs MEASURED SWEEP ===\n');
fprintf('Nominal actuator assumption: slew=%.1f deg/s, u_max=%.1f deg\n', ...
    nominal_loaded_slew_deg_s, servo_max_deg_nominal);

nominal_design = struct();
for ip = 1:numel(p_values)
    p_val = p_values(ip);
    cfg_design = build_pid_cfg(cfg0, p_val, nominal_loaded_slew_deg_s, ...
        servo_max_deg_nominal, servo_max_deg_nominal, code_scale);
    [best_Kp, best_Kd, best_rate, best_rms] = tune_pd(cfg_design, opts, naive_design_realism);
    nominal_design(ip).p = p_val;
    nominal_design(ip).Kp = best_Kp;
    nominal_design(ip).Kd = best_Kd;
    nominal_design(ip).success = best_rate;
    nominal_design(ip).rms = best_rms;
    fprintf('  nominal design @ p=%2d -> Kp=%5.1f Kd=%4.1f success=%.2f\n', ...
        p_val, best_Kp, best_Kd, best_rate);
end

rows = {};
for ip = 1:numel(p_values)
    p_val = p_values(ip);
    for is = 1:numel(slew_scale_list)
        slew_scale = slew_scale_list(is);
        for iu = 1:numel(util_frac_list)
            util_frac = util_frac_list(iu);
            actual_slew_deg_s = nominal_loaded_slew_deg_s * slew_scale;
            actual_u_max_deg = servo_max_deg_nominal * util_frac;

            cfg_actual = build_pid_cfg(cfg0, p_val, actual_slew_deg_s, ...
                actual_u_max_deg, actual_u_max_deg, code_scale);

            % 1) Nominal-assumption PD: tuned on optimistic actuator, deployed on degraded actuator.
            nom = nominal_design(ip);
            cfg_nom = cfg_actual;
            cfg_nom.controllers.PID.Kp = nom.Kp;
            cfg_nom.controllers.PID.Kd = nom.Kd;
            cfg_nom.controllers.PID.u_max = deg2rad(servo_max_deg_nominal) * code_scale;
            cfg_nom.controllers.PID.i_lim = cfg_nom.controllers.PID.u_max;
            [rate_nom, rms_nom, peak_nom] = evaluate_controller('PID', cfg_nom, opts, realism);
            rows(end+1,:) = {string('NOMINAL_PD'), p_val, slew_scale, util_frac, ...
                actual_slew_deg_s, actual_u_max_deg, nominal_loaded_slew_deg_s, ...
                servo_max_deg_nominal, nom.Kp, nom.Kd, rate_nom, rms_nom, peak_nom}; %#ok<SAGROW>

            % 2) Measured-aware PD: tuned on the measured actuator capability.
            [meas_Kp, meas_Kd, meas_rate_design, meas_rms_design] = tune_pd(cfg_actual, opts, realism);
            cfg_meas = cfg_actual;
            cfg_meas.controllers.PID.Kp = meas_Kp;
            cfg_meas.controllers.PID.Kd = meas_Kd;
            [rate_meas, rms_meas, peak_meas] = evaluate_controller('PID', cfg_meas, opts, realism);
            rows(end+1,:) = {string('MEASURED_PD'), p_val, slew_scale, util_frac, ...
                actual_slew_deg_s, actual_u_max_deg, actual_slew_deg_s, ...
                actual_u_max_deg, meas_Kp, meas_Kd, rate_meas, rms_meas, peak_meas}; %#ok<SAGROW>

            % 3) Current validator LQR recommendation on the same measured plant.
            meas = struct('slew_deg_per_s', actual_slew_deg_s, ...
                'servo_max_deg', servo_max_deg_nominal, ...
                'p_est', p_val, ...
                'keff_est', cfg0.plant.control_eff, ...
                'damp_est', cfg0.plant.aero_damp);
            evalc('rec = recommend_envelope(meas);');
            lqr_u_max_deg = min(actual_u_max_deg, rec.u_max_recommended_deg);
            if lqr_u_max_deg <= 0.05
                rate_lqr = 0.0; rms_lqr = 90.0; peak_lqr = 90.0;
            else
                cfg_lqr = cfg_actual;
                cfg_lqr.controllers.FIXED_LQR.K = rec.K;
                cfg_lqr.controllers.FIXED_LQR.K_nominal = rec.K;
                cfg_lqr.controllers.FIXED_LQR.u_max = deg2rad(lqr_u_max_deg) * code_scale;
                [rate_lqr, rms_lqr, peak_lqr] = evaluate_controller('FIXED_LQR', cfg_lqr, opts, realism);
            end
            rows(end+1,:) = {string('VALIDATOR_LQR'), p_val, slew_scale, util_frac, ...
                actual_slew_deg_s, actual_u_max_deg, actual_slew_deg_s, ...
                lqr_u_max_deg, rec.K(1), rec.K(2), rate_lqr, rms_lqr, peak_lqr}; %#ok<SAGROW>
        end
    end
end

T = cell2table(rows, 'VariableNames', { ...
    'policy', 'p_unstable', 'slew_scale', 'util_frac', ...
    'actual_slew_deg_s', 'actual_u_max_deg', 'design_slew_deg_s', ...
    'design_u_max_deg', 'gain1', 'gain2', 'success_rate', 'rms_deg', 'peak_deg'});
writetable(T, fullfile(here, 'results', 'preflight_nominal_vs_measured.csv'));

summary_rows = {};
policies = unique(T.policy, 'stable');
for ip = 1:numel(p_values)
    p_val = p_values(ip);
    for ipol = 1:numel(policies)
        pol = policies(ipol);
        sel_all = T.p_unstable == p_val & T.policy == pol;
        sel_deg = sel_all & (T.slew_scale < 0.999 | T.util_frac < 0.999);
        summary_rows(end+1,:) = {pol, p_val, string('ALL_CASES'), ...
            mean(T.success_rate(sel_all)), mean(T.rms_deg(sel_all))}; %#ok<SAGROW>
        summary_rows(end+1,:) = {pol, p_val, string('DEGRADED_ONLY'), ...
            mean(T.success_rate(sel_deg)), mean(T.rms_deg(sel_deg))}; %#ok<SAGROW>
    end
end
S = cell2table(summary_rows, 'VariableNames', { ...
    'policy', 'p_unstable', 'slice', 'mean_success_rate', 'mean_rms_deg'});
writetable(S, fullfile(here, 'results', 'preflight_nominal_vs_measured_summary.csv'));

fprintf('\nDegraded-case mean success by policy:\n');
for ip = 1:numel(p_values)
    p_val = p_values(ip);
    fprintf('  p=%2d\n', p_val);
    for ipol = 1:numel(policies)
        pol = policies(ipol);
        sel = T.p_unstable == p_val & T.policy == pol & ...
              (T.slew_scale < 0.999 | T.util_frac < 0.999);
        fprintf('    %-14s success=%.3f  rms=%.2f deg\n', ...
            pol, mean(T.success_rate(sel)), mean(T.rms_deg(sel)));
    end
end

fprintf('\nSaved: experiments/results/preflight_nominal_vs_measured.csv\n');
fprintf('Saved: experiments/results/preflight_nominal_vs_measured_summary.csv\n');
fprintf('DONE.\n');


function cfg = build_pid_cfg(cfg0, p_val, slew_deg_s, plant_u_max_deg, ctrl_u_max_deg, code_scale)
cfg = cfg0;
cfg.plant.p_unstable = p_val;
cfg.plant.slew_max = deg2rad(slew_deg_s) * code_scale;
cfg.plant.u_max = deg2rad(plant_u_max_deg) * code_scale;
cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.u_max = deg2rad(ctrl_u_max_deg) * code_scale;
cfg.controllers.PID.i_lim = cfg.controllers.PID.u_max;
cfg.controllers.FIXED_LQR.u_max = cfg.plant.u_max;
end


function realism = all_realistic_profile()
realism = struct( ...
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
        [rate, mean_rms_deg] = evaluate_controller('PID', cfg, opts, realism);
        if rate > best_rate || (abs(rate - best_rate) < 1e-9 && mean_rms_deg < best_rms)
            best_rate = rate;
            best_rms = mean_rms_deg;
            best_Kp = Kp;
            best_Kd = Kd;
        end
    end
end
end


function [rate, mean_rms_deg, mean_peak_deg] = evaluate_controller(ctrl_name, cfg, opts, realism)
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
            out = simulate_case_realistic(ctrl_name, sc, cfg, opts.seeds(iseed) + 100 * it, realism);
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