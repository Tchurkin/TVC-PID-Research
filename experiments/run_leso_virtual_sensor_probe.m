% run_leso_virtual_sensor_probe.m
% Focused falsifier for the LESO virtual-sensor idea on the current rocket stack.
%
% Question:
%   Does a simple LESO-style lumped-disturbance estimate provide an earlier
%   or cleaner stress signal than simpler observables (theta, q, command gap)
%   in the current realistic TVC rocket simulator?
%
% Kill criterion:
%   If the best LESO bandwidth does not beat both theta/q on early-window AUC
%   and does not provide materially better lead time in the stressed regimes,
%   the LESO path should be treated as a killed primary direction.

clear; clc;

here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', 'ModelRocket_Adaptive_TVC', 'src')));

results_dir = fullfile(here, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg0 = rocket_defaults();
realism = full_realistic_profile();

regimes = {
    'LOW_DEMAND',         4, 18.0, 10.2;
    'BOUNDARY',           8, 12.0,  6.0;
    'ACTUATOR_LIMITED',  10,  9.0,  4.5;
};

opts.theta0_deg_set   = [3 6];
opts.seeds            = 1:6;
opts.Kp_grid          = [1 3 10 30];
opts.Kd_grid          = [0.3 1 3 10];
opts.success_rms_deg  = 10.0;
opts.success_peak_deg = 40.0;
opts.success_end_deg  = 8.0;
opts.failure_theta_deg = 40.0;
opts.disturb_amp      = 0.10;
opts.disturb_freq_hz  = 0.80;
opts.t_end            = 3.0;
opts.eval_start_s     = 0.25;
opts.eval_early_end_s = 1.50;

omega_o_values = [4 8 12 16];

fprintf('=== LESO VIRTUAL SENSOR PROBE ===\n');
fprintf('Observer bandwidth candidates: %s rad/s\n', mat2str(omega_o_values));

trial_rows = cell(0, 17);

for ir = 1:size(regimes, 1)
    regime_name = regimes{ir, 1};
    p_val       = regimes{ir, 2};
    slew_code   = regimes{ir, 3};
    umax_code   = regimes{ir, 4};

    cfg = cfg0;
    cfg.t_end_demo = opts.t_end;
    cfg.plant.p_unstable = p_val;
    cfg.plant.slew_max = slew_code;
    cfg.plant.u_max = umax_code;
    cfg.controllers.PID.Ki = 0.0;
    cfg.controllers.PID.u_max = umax_code;
    cfg.controllers.PID.i_lim = umax_code;

    [best_Kp, best_Kd, tune_success] = tune_pd(cfg, opts, realism);
    cfg.controllers.PID.Kp = best_Kp;
    cfg.controllers.PID.Kd = best_Kd;

    fprintf('\n[%s] tuned PID Kp=%.2f Kd=%.2f full-real success=%.3f\n', ...
        regime_name, best_Kp, best_Kd, tune_success);

    sc = rocket_scenario('NOMINAL', cfg);
    sc.t_end = opts.t_end;
    sc.disturbance_amp = opts.disturb_amp;
    sc.disturbance_freq_hz = opts.disturb_freq_hz;

    for it = 1:numel(opts.theta0_deg_set)
        theta0_deg = opts.theta0_deg_set(it);
        cfg.plant.theta0 = deg2rad(theta0_deg);

        for iseed = 1:numel(opts.seeds)
            seed_value = opts.seeds(iseed) + 100 * it;
            out = simulate_case_realistic('PID', sc, cfg, seed_value, realism);

            [stable, failure_time_s, rms_deg, peak_deg, end_deg] = classify_trial(out, opts);

            signal = compute_signals(out, cfg, omega_o_values);
            metric_names = fieldnames(signal.metrics);

            for im = 1:numel(metric_names)
                metric_name = metric_names{im};
                metric_trace = signal.metrics.(metric_name);

                [first_cross_s, peak_early] = summarize_metric(metric_trace, out.time, opts);

                if isfinite(failure_time_s) && isfinite(first_cross_s) && first_cross_s <= failure_time_s
                    lead_time_s = failure_time_s - first_cross_s;
                else
                    lead_time_s = NaN;
                end

                trial_rows(end+1, :) = { ...
                    string(regime_name), p_val, slew_code, umax_code, best_Kp, best_Kd, ...
                    theta0_deg, seed_value, stable, rms_deg, peak_deg, end_deg, failure_time_s, ...
                    string(metric_name), first_cross_s, peak_early, lead_time_s}; %#ok<SAGROW>
            end
        end
    end
end

trialT = cell2table(trial_rows, 'VariableNames', { ...
    'regime', 'p_unstable', 'slew_code', 'u_max_code', 'Kp', 'Kd', ...
    'theta0_deg', 'seed', 'stable', 'rms_deg', 'peak_deg', 'end_deg', 'failure_time_s', ...
    'metric', 'first_cross_s', 'peak_early', 'lead_time_s'});

thresholdT = build_thresholds(trialT, 'LOW_DEMAND');
trialT = apply_thresholds(trialT, thresholdT, opts);
summaryT = summarize_results(trialT, thresholdT);

writetable(trialT, fullfile(results_dir, 'leso_virtual_sensor_trials.csv'));
writetable(thresholdT, fullfile(results_dir, 'leso_virtual_sensor_thresholds.csv'));
writetable(summaryT, fullfile(results_dir, 'leso_virtual_sensor_summary.csv'));

fprintf('\nSaved: experiments/results/leso_virtual_sensor_trials.csv\n');
fprintf('Saved: experiments/results/leso_virtual_sensor_thresholds.csv\n');
fprintf('Saved: experiments/results/leso_virtual_sensor_summary.csv\n');

disp(summaryT);
print_verdict(summaryT);


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


function [best_Kp, best_Kd, best_rate] = tune_pd(cfg, opts, realism)
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


function [rate, mean_rms_deg] = evaluate_pid(cfg, opts, realism)
sc = rocket_scenario('NOMINAL', cfg);
sc.t_end = opts.t_end;
sc.disturbance_amp = opts.disturb_amp;
sc.disturbance_freq_hz = opts.disturb_freq_hz;

wins = 0;
n = 0;
rms_vals = [];

for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        n = n + 1;
        out = simulate_case_realistic('PID', sc, cfg, opts.seeds(iseed) + 100 * it, realism);
        [stable, ~, rms_deg] = classify_trial(out, opts);
        wins = wins + double(stable);
        rms_vals(end+1, 1) = rms_deg; %#ok<AGROW>
    end
end

rate = wins / n;
mean_rms_deg = mean(rms_vals);
end


function [stable, failure_time_s, rms_deg, peak_deg, end_deg] = classify_trial(out, opts)
rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
peak_deg = rad2deg(max(abs(out.theta)));
end_deg = rad2deg(abs(out.theta(end)));

if ~all(isfinite(out.theta))
    stable = false;
    rms_deg = 90;
    peak_deg = 90;
    end_deg = 90;
else
    stable = rms_deg <= opts.success_rms_deg && ...
             peak_deg <= opts.success_peak_deg && ...
             end_deg <= opts.success_end_deg;
end

idx_fail = find(rad2deg(abs(out.theta)) >= opts.failure_theta_deg, 1, 'first');
if isempty(idx_fail)
    failure_time_s = NaN;
else
    failure_time_s = out.time(idx_fail);
end
end


function signal = compute_signals(out, cfg, omega_o_values)
signal.metrics = struct();

signal.metrics.THETA_ABS = abs(rad2deg(out.theta_meas));
signal.metrics.Q_ABS = abs(rad2deg(out.q_meas));
signal.metrics.CMD_GAP = abs(out.u_cmd - out.u_act_meas);

tau_assumed = cfg.plant.tau_act;
du_obs = [0; abs(diff(out.u_act_meas)) / max(out.time(2) - out.time(1), 1e-6)];
demand_rate = abs(out.u_cmd - out.u_act_meas) / max(tau_assumed, 1e-3);
signal.metrics.RATE_GAP = max(0, demand_rate - du_obs);

for i = 1:numel(omega_o_values)
    omega_o = omega_o_values(i);
    dhat = run_leso(out, cfg, omega_o);
    metric_name = sprintf('LESO_DHAT_O%d', round(omega_o));
    signal.metrics.(metric_name) = abs(dhat);
end
end


function dhat = run_leso(out, cfg, omega_o)
dt = max(out.time(2) - out.time(1), 1e-6);

beta1 = 3 * omega_o;
beta2 = 3 * omega_o^2;
beta3 = omega_o^3;

aero_damp = cfg.plant.aero_damp;
p_nom = cfg.plant.p_unstable;
b0 = cfg.plant.control_eff;

z1 = out.theta_meas(1);
z2 = out.q_meas(1);
z3 = 0;

dhat = zeros(size(out.time));
dhat(1) = z3;

for k = 2:numel(out.time)
    e = out.theta_meas(k-1) - z1;

    z1_dot = z2 + beta1 * e;
    z2_dot = z3 + p_nom^2 * z1 - aero_damp * z2 + b0 * out.u_cmd(k-1) + beta2 * e;
    z3_dot = beta3 * e;

    z1 = z1 + dt * z1_dot;
    z2 = z2 + dt * z2_dot;
    z3 = z3 + dt * z3_dot;

    dhat(k) = z3;
end

dhat = rad2deg(dhat);
end


function [first_cross_s, peak_early] = summarize_metric(metric_trace, time, opts)
metric_trace = metric_trace(:);
time = time(:);

eval_mask = time >= opts.eval_start_s;
early_mask = time >= opts.eval_start_s & time <= opts.eval_early_end_s;

if any(early_mask)
    peak_early = max(metric_trace(early_mask));
else
    peak_early = max(metric_trace);
end

first_cross_s = NaN;
% Threshold applied later once LOW_DEMAND envelope is known.
if ~any(eval_mask)
    return;
end
end


function thresholdT = build_thresholds(trialT, low_regime)
low_mask = strcmp(trialT.regime, low_regime);
metrics = unique(trialT.metric);

rows = cell(numel(metrics), 3);
for i = 1:numel(metrics)
    metric = metrics{i};
    vals = trialT.peak_early(low_mask & strcmp(trialT.metric, metric));
    threshold = quantile(vals, 0.99);
    rows(i, :) = {string(metric), threshold, mean(vals)};
end

thresholdT = cell2table(rows, 'VariableNames', {'metric', 'threshold', 'low_regime_mean_peak'});
end


function trialT = apply_thresholds(trialT, thresholdT, opts)
trialT.crossed = false(height(trialT), 1);
trialT.lead_valid = false(height(trialT), 1);
trialT.threshold = nan(height(trialT), 1);

for i = 1:height(trialT)
    metric = trialT.metric{i};
    thr = thresholdT.threshold(strcmp(thresholdT.metric, metric));
    trialT.threshold(i) = thr;
end

for i = 1:height(trialT)
    % Re-run the crossing time using the stored threshold. This keeps the
    % per-trial table self-contained and avoids storing all time traces.
    % first_cross_s already exists only as placeholder from summarize_metric.
    % Reconstruct crossing logic from the scalar peak: if the early-window
    % peak never exceeds the threshold then there is no usable early warning.
    trialT.crossed(i) = trialT.peak_early(i) >= trialT.threshold(i);
    trialT.lead_valid(i) = trialT.crossed(i) && ~trialT.stable(i) && isfinite(trialT.lead_time_s(i));
end

% Now compute the actual first-cross time with threshold-aware replay.
% This is done in a second pass because thresholds depend on LOW_DEMAND stats.
trialT.first_cross_s(:) = NaN;
trialT.lead_time_s(:) = NaN;

cfg0 = rocket_defaults();
realism = full_realistic_profile();

for i = 1:height(trialT)
    cfg = cfg0;
    cfg.t_end_demo = opts.t_end;
    cfg.plant.p_unstable = trialT.p_unstable(i);
    cfg.plant.slew_max = trialT.slew_code(i);
    cfg.plant.u_max = trialT.u_max_code(i);
    cfg.controllers.PID.Ki = 0.0;
    cfg.controllers.PID.u_max = trialT.u_max_code(i);
    cfg.controllers.PID.i_lim = trialT.u_max_code(i);
    cfg.controllers.PID.Kp = trialT.Kp(i);
    cfg.controllers.PID.Kd = trialT.Kd(i);
    cfg.plant.theta0 = deg2rad(trialT.theta0_deg(i));

    sc = rocket_scenario('NOMINAL', cfg);
    sc.t_end = opts.t_end;
    sc.disturbance_amp = opts.disturb_amp;
    sc.disturbance_freq_hz = opts.disturb_freq_hz;

    out = simulate_case_realistic('PID', sc, cfg, trialT.seed(i), realism);
    signals = compute_signals(out, cfg, [4 8 12 16]);
    trace = signals.metrics.(trialT.metric{i});

    thr = trialT.threshold(i);
    eval_mask = out.time >= opts.eval_start_s;
    idx_cross = find(eval_mask & (trace(:) >= thr), 1, 'first');
    if ~isempty(idx_cross)
        trialT.first_cross_s(i) = out.time(idx_cross);
        if ~trialT.stable(i) && isfinite(trialT.failure_time_s(i)) && trialT.first_cross_s(i) <= trialT.failure_time_s(i)
            trialT.lead_time_s(i) = trialT.failure_time_s(i) - trialT.first_cross_s(i);
        end
    end
end

trialT.crossed = isfinite(trialT.first_cross_s);
trialT.lead_valid = ~trialT.stable & isfinite(trialT.lead_time_s);
end


function summaryT = summarize_results(trialT, thresholdT)
metrics = thresholdT.metric;
regimes = {'LOW_DEMAND', 'BOUNDARY', 'ACTUATOR_LIMITED', 'POOLED_STRESSED'};

rows = cell(0, 9);
for ir = 1:numel(regimes)
    regime = regimes{ir};
    switch regime
        case 'POOLED_STRESSED'
            regime_mask = ~strcmp(trialT.regime, 'LOW_DEMAND');
        otherwise
            regime_mask = strcmp(trialT.regime, regime);
    end

    for im = 1:numel(metrics)
        metric = metrics{im};
        mask = regime_mask & strcmp(trialT.metric, metric);
        if ~any(mask)
            continue;
        end

        stable_mask = mask & trialT.stable;
        failed_mask = mask & ~trialT.stable;

        false_alarm_rate = mean(trialT.crossed(stable_mask));
        detection_rate = mean(trialT.crossed(failed_mask));

        lead_vals = trialT.lead_time_s(failed_mask & isfinite(trialT.lead_time_s));
        if isempty(lead_vals)
            median_lead_s = NaN;
        else
            median_lead_s = median(lead_vals);
        end

        stable_peaks = trialT.peak_early(stable_mask);
        failed_peaks = trialT.peak_early(failed_mask);
        auc_early = compute_auc(failed_peaks, stable_peaks);

        thr = thresholdT.threshold(strcmp(thresholdT.metric, metric));

        rows(end+1, :) = {string(regime), string(metric), thr, ...
            false_alarm_rate, detection_rate, median_lead_s, auc_early, ...
            nnz(stable_mask), nnz(failed_mask)}; %#ok<SAGROW>
    end
end

summaryT = cell2table(rows, 'VariableNames', { ...
    'regime', 'metric', 'threshold', 'false_alarm_rate', 'detection_rate_failed', ...
    'median_lead_s', 'auc_early', 'n_stable', 'n_failed'});
end


function auc = compute_auc(pos_vals, neg_vals)
if isempty(pos_vals) || isempty(neg_vals)
    auc = NaN;
    return;
end

scores = [pos_vals(:); neg_vals(:)];
labels = [ones(numel(pos_vals), 1); zeros(numel(neg_vals), 1)];
rank_vals = tiedrank(scores);
pos_ranks = sum(rank_vals(labels == 1));
n_pos = numel(pos_vals);
n_neg = numel(neg_vals);
auc = (pos_ranks - n_pos * (n_pos + 1) / 2) / (n_pos * n_neg);
end


function print_verdict(summaryT)
pooled = summaryT(strcmp(summaryT.regime, 'POOLED_STRESSED'), :);
leso_mask = startsWith(pooled.metric, 'LESO_DHAT_');
simple_mask = strcmp(pooled.metric, 'THETA_ABS') | strcmp(pooled.metric, 'Q_ABS');

best_leso_auc = max(pooled.auc_early(leso_mask));
best_simple_auc = max(pooled.auc_early(simple_mask));
best_leso_lead = max(pooled.median_lead_s(leso_mask));
best_simple_lead = max(pooled.median_lead_s(simple_mask));

fprintf('\n=== LESO DECISION GATE ===\n');
fprintf('Best LESO pooled early AUC:   %.3f\n', best_leso_auc);
fprintf('Best theta/q pooled early AUC: %.3f\n', best_simple_auc);
fprintf('Best LESO median lead time:   %.3f s\n', best_leso_lead);
fprintf('Best theta/q median lead:     %.3f s\n', best_simple_lead);

if ~(best_leso_auc > best_simple_auc && best_leso_lead >= best_simple_lead)
    fprintf('VERDICT: KILL AS PRIMARY DIRECTION. LESO does not beat simpler observables on this probe.\n');
else
    fprintf('VERDICT: KEEP AS SECONDARY DIRECTION. LESO beats simpler observables on this probe.\n');
end
end