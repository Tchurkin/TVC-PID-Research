% run_exp34_regime_importance.m
% Exp4: regime-conditioned factor importance with many cells per regime.
%
% Workflow:
%   1) Load frozen Exp1 map and sample many cells per regime (boundary-weighted).
%   2) For each sampled cell, tune one PD on full realism only.
%   3) Hold gains fixed and run leave-one-factor-out ablations.
%   4) Rank top-3 factors per cell; aggregate medians by regime.
%   5) Export publication-ready regime x factor heatmap metrics.

clear; clc;

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
framework_dir = fullfile(here, 'framework');

addpath(genpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src')));
addpath(fullfile(framework_dir, 'plant'));
addpath(fullfile(framework_dir, 'controllers'));
addpath(fullfile(framework_dir, 'analysis'));

results_dir = fullfile(here, 'results');
graphs_dir = fullfile(results_dir, 'graphs');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'); mkdir(graphs_dir); end

phase_path = fullfile(results_dir, 'phase_diagram.csv');
if ~exist(phase_path, 'file')
    error('run_exp34_regime_importance:missingExp1', ...
        'Missing %s. Run run_locked_exp1 first.', phase_path);
end

P = default_rocket_params();
phaseT = readtable(phase_path);

target_per_regime = 24;  % >=10 required; prefer 20+.
cells = select_regime_cells_many(phaseT, target_per_regime);
writetable(cells, fullfile(results_dir, 'exp34_selected_cells.csv'));

opts = struct();
opts.theta0_deg_set = P.analysis.theta0_deg_set;
opts.seeds = P.analysis.seeds;
opts.success_rms_deg = P.analysis.success_rms_deg;
opts.success_peak_deg = P.analysis.success_peak_deg;
opts.success_end_deg = P.analysis.success_end_deg;
opts.success_max_theta_deg = P.analysis.success_max_theta_deg;

baseline = baseline_profile(P);
ideal = ideal_profile();
factors = factor_specs();
toggleT = build_factor_toggle_table(factors, baseline, ideal);

rows = cell(0, 27);
rank_rows = cell(0, 14);
interaction_rows = cell(0, 27);

fprintf('=== EXP4 REGIME IMPORTANCE (MANY-CELL) ===\n');
fprintf('Requested per regime: %d\n', target_per_regime);
disp(groupcounts(cells, 'regime_label'));

for ic = 1:height(cells)
    c = cells(ic, :);
    cell_id = ic;

    override = struct();
    override.p_unstable = c.p_unstable;
    override.servo_slew = c.servo_slew_deg_s;
    override.u_max_frac = c.best_u_max_frac;
    override.deadband = P.rocket.deadband;
    override.backlash = P.rocket.backlash;
    override.latency = P.rocket.latency;
    override.wind_strength = P.rocket.wind_strength;

    [cfg, sc, ~] = build_realistic_cfg(P, override);
    sc = apply_baseline_scenario(sc, baseline.scenario);

    tuned = autotune_pd_grid(cfg, sc, baseline.realism, P);
    cfg = configure_pid_controller(cfg, tuned.Kp, tuned.Kd);

    full_eval = evaluate_fixed_pd(cfg, sc, opts, baseline.realism);
    full_s2r = 0.5 * (full_eval.mean_u_sat + full_eval.mean_slew_sat);

    fprintf('\n[%3d/%3d] %s p=%g slew=%g gimbal=%g | Kp=%.1f Kd=%.1f full=%.3f\n', ...
        ic, height(cells), c.regime_label{1}, c.p_unstable, c.servo_slew_deg_s, c.max_gimbal_deg, ...
        tuned.Kp, tuned.Kd, full_eval.rate);

    cell_scores = zeros(numel(factors), 1);
    cell_meta = cell(numel(factors), 1);
    for ifac = 1:numel(factors)
        fac = factors(ifac);

        ablated_realism = baseline.realism;
        for jf = 1:numel(fac.realism_fields)
            f = fac.realism_fields{jf};
            ablated_realism.(f) = ideal.realism.(f);
        end
        ablated_sc = sc;
        for js = 1:numel(fac.scenario_fields)
            sf = fac.scenario_fields{js};
            ablated_sc.(sf) = ideal.scenario.(sf);
        end

        ev = evaluate_fixed_pd(cfg, ablated_sc, opts, ablated_realism);

        success_recovery = ev.rate - full_eval.rate;
        rms_recovery = full_eval.mean_rms_deg - ev.mean_rms_deg;
        s2r = 0.5 * (ev.mean_u_sat + ev.mean_slew_sat);
        s2r_reduction = full_s2r - s2r;

        score = importance_score(success_recovery, rms_recovery, s2r_reduction, full_eval.mean_rms_deg, full_s2r);
        cell_scores(ifac) = score;

        rows(end + 1, :) = { ...
            cell_id, string(c.regime_label{1}), c.p_unstable, c.servo_slew_deg_s, c.max_gimbal_deg, c.best_u_max_frac, ...
            tuned.Kp, tuned.Kd, string(fac.name), ...
            full_eval.rate, ev.rate, success_recovery, ...
            full_eval.mean_rms_deg, ev.mean_rms_deg, rms_recovery, ...
            full_eval.mean_peak_deg, ev.mean_peak_deg, full_eval.mean_end_deg, ev.mean_end_deg, ...
            full_eval.mean_u_sat, ev.mean_u_sat, full_eval.mean_slew_sat, ev.mean_slew_sat, ...
            full_s2r, s2r, s2r_reduction, score};

        cell_meta{ifac} = struct( ...
            'name', string(fac.name), ...
            'success_recovery', success_recovery, ...
            'rms_recovery', rms_recovery, ...
            's2r_reduction', s2r_reduction, ...
            'score', score);
    end

    alloff_sc = sc;
    alloff_sc = apply_baseline_scenario(alloff_sc, ideal.scenario);
    alloff_eval = evaluate_fixed_pd(cfg, alloff_sc, opts, ideal.realism);
    alloff_s2r = 0.5 * (alloff_eval.mean_u_sat + alloff_eval.mean_slew_sat);

    combined_success_recovery = alloff_eval.rate - full_eval.rate;
    combined_rms_recovery = full_eval.mean_rms_deg - alloff_eval.mean_rms_deg;
    combined_s2r_reduction = full_s2r - alloff_s2r;
    combined_score = importance_score(combined_success_recovery, combined_rms_recovery, combined_s2r_reduction, full_eval.mean_rms_deg, full_s2r);

    sum_single_success = sum(cellfun(@(m) m.success_recovery, cell_meta));
    sum_single_rms = sum(cellfun(@(m) m.rms_recovery, cell_meta));
    sum_single_s2r = sum(cellfun(@(m) m.s2r_reduction, cell_meta));
    sum_single_score = sum(cell_scores);

    interaction_rows(end + 1, :) = { ...
        cell_id, string(c.regime_label{1}), c.p_unstable, c.servo_slew_deg_s, c.max_gimbal_deg, c.best_u_max_frac, ...
        tuned.Kp, tuned.Kd, ...
        full_eval.rate, alloff_eval.rate, combined_success_recovery, sum_single_success, combined_success_recovery - sum_single_success, ...
        full_eval.mean_rms_deg, alloff_eval.mean_rms_deg, combined_rms_recovery, sum_single_rms, combined_rms_recovery - sum_single_rms, ...
        full_s2r, alloff_s2r, combined_s2r_reduction, sum_single_s2r, combined_s2r_reduction - sum_single_s2r, ...
        combined_score, sum_single_score, combined_score - sum_single_score, ...
        safe_ratio(combined_score, sum_single_score)};

    imp_pct = zeros(size(cell_scores));
    pos = max(0, cell_scores);
    denom = sum(pos);
    if denom > 0
        imp_pct = 100 * pos / denom;
    end

    [~, order] = sort(cell_scores, 'descend');
    topk = min(3, numel(order));
    for rk = 1:topk
        idx = order(rk);
        m = cell_meta{idx};
        rank_rows(end + 1, :) = { ...
            cell_id, string(c.regime_label{1}), rk, m.name, imp_pct(idx), ...
            m.success_recovery, m.rms_recovery, m.s2r_reduction, ...
            c.p_unstable, c.servo_slew_deg_s, c.max_gimbal_deg, c.best_u_max_frac, tuned.Kp, tuned.Kd};
    end
end

importanceT = cell2table(rows, 'VariableNames', { ...
    'cell_id', 'regime', 'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'u_max_frac', ...
    'tuned_Kp', 'tuned_Kd', 'factor', ...
    'full_success', 'factor_success', 'success_recovery', ...
    'full_rms_deg', 'factor_rms_deg', 'rms_recovery_deg', ...
    'full_peak_deg', 'factor_peak_deg', 'full_end_deg', 'factor_end_deg', ...
    'full_u_sat_frac', 'factor_u_sat_frac', 'full_slew_sat_frac', 'factor_slew_sat_frac', ...
    'full_s2r', 'factor_s2r', 's2r_reduction', 'importance_score'});
importanceT.factor_importance_pct = compute_importance_pct_allrows(importanceT);

rankT = cell2table(rank_rows, 'VariableNames', { ...
    'cell_id', 'regime', 'rank', 'factor', 'importance_pct', ...
    'success_recovery', 'rms_recovery_deg', 's2r_reduction', ...
    'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'u_max_frac', 'tuned_Kp', 'tuned_Kd'});

aggT = summarize_regime_factor_medians(importanceT);
top3T = summarize_top3(rankT);
interactionT = cell2table(interaction_rows, 'VariableNames', { ...
    'cell_id', 'regime', 'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'u_max_frac', ...
    'tuned_Kp', 'tuned_Kd', ...
    'full_success', 'alloff_success', 'combined_success_recovery', 'sum_single_success_recovery', 'interaction_gap_success', ...
    'full_rms_deg', 'alloff_rms_deg', 'combined_rms_recovery_deg', 'sum_single_rms_recovery_deg', 'interaction_gap_rms_recovery_deg', ...
    'full_s2r', 'alloff_s2r', 'combined_s2r_reduction', 'sum_single_s2r_reduction', 'interaction_gap_s2r_reduction', ...
    'combined_score', 'sum_single_score', 'interaction_gap_score', 'combined_to_sum_score_ratio'});
interactionAggT = summarize_interaction_gaps(interactionT);

writetable(importanceT, fullfile(results_dir, 'exp34_regime_factor_importance.csv'));
writetable(rankT, fullfile(results_dir, 'exp34_cell_top3_rankings.csv'));
writetable(aggT, fullfile(results_dir, 'exp34_regime_factor_medians.csv'));
writetable(top3T, fullfile(results_dir, 'exp34_regime_top3_frequency.csv'));
writetable(interactionT, fullfile(results_dir, 'exp34_interaction_audit.csv'));
writetable(interactionAggT, fullfile(results_dir, 'exp34_interaction_audit_regime_medians.csv'));
writetable(toggleT, fullfile(results_dir, 'exp34_factor_toggle_ranges.csv'));

plot_heatmaps(aggT, fullfile(graphs_dir, 'exp34_regime_factor_heatmaps.png'));

fprintf('\nSaved: %s\n', fullfile(results_dir, 'exp34_selected_cells.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp34_regime_factor_importance.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp34_cell_top3_rankings.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp34_regime_factor_medians.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp34_regime_top3_frequency.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp34_interaction_audit.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp34_interaction_audit_regime_medians.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp34_factor_toggle_ranges.csv'));
fprintf('Saved: %s\n', fullfile(graphs_dir, 'exp34_regime_factor_heatmaps.png'));
fprintf('DONE.\n');


function cells = select_regime_cells_many(T, target_per_regime)
labels = {'EASY', 'FRAGILE', 'INFEASIBLE'};
T.p_unstable = double(T.p_unstable);
T.servo_slew_deg_s = double(T.servo_slew_deg_s);
T.max_gimbal_deg = double(T.max_gimbal_deg);

rows = cell(0, 12);
for i = 1:numel(labels)
    label = labels{i};
    sub = T(strcmpi(string(T.regime_label), label), :);
    if isempty(sub)
        continue;
    end

    others = T(~strcmpi(string(T.regime_label), label), :);
    boundary = closeness_to_set(sub, others);
    Xn = normalize_rows([sub.p_unstable, sub.servo_slew_deg_s, sub.max_gimbal_deg]);

    n_pick = min(height(sub), max(10, target_per_regime));
    picked = greedy_boundary_spread_pick(Xn, boundary, n_pick, 0.75);

    for k = 1:numel(picked)
        j = picked(k);
        rows(end + 1, :) = { ...
            string(sub.regime_label{j}), sub.p_unstable(j), sub.servo_slew_deg_s(j), sub.max_gimbal_deg(j), ...
            sub.best_u_max_frac(j), sub.nominal_success_rate(j), sub.under_success_rate(j), sub.over_success_rate(j), sub.robustness(j), ...
            boundary(j), k, n_pick};
    end
end

cells = cell2table(rows, 'VariableNames', { ...
    'regime_label', 'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', ...
    'best_u_max_frac', 'nominal_success_rate', 'under_success_rate', 'over_success_rate', 'robustness', ...
    'boundary_score', 'selection_rank', 'target_count'});

cells = sortrows(cells, {'regime_label', 'selection_rank'});
end


function picked = greedy_boundary_spread_pick(X, boundary, n_pick, boundary_weight)
n = size(X, 1);
picked = zeros(n_pick, 1);
available = true(n, 1);

[~, first_idx] = max(boundary);
picked(1) = first_idx;
available(first_idx) = false;

for k = 2:n_pick
    avail_idx = find(available);
    if isempty(avail_idx)
        picked = picked(1:k-1);
        break;
    end

    b = boundary(avail_idx);
    b = normalize01(b);

    dmin = zeros(numel(avail_idx), 1);
    selX = X(picked(1:k-1), :);
    for i = 1:numel(avail_idx)
        di = sum(abs(selX - X(avail_idx(i), :)), 2);
        dmin(i) = min(di);
    end
    dmin = normalize01(dmin);

    score = boundary_weight * b + (1 - boundary_weight) * dmin;
    [~, imax] = max(score);
    idx = avail_idx(imax);
    picked(k) = idx;
    available(idx) = false;
end
end


function baseline = baseline_profile(P)
baseline.realism = struct( ...
    'gyro_noise_std', 0.015, ...
    'gyro_bias_init', 0.010, ...
    'gyro_bias_rw', 0.005, ...
    'gyro_quant_lsb', 2 * pi / 4000, ...
    'bias_cal_residual_frac', 0.10, ...
    'sensor_latency_steps', P.rocket.latency, ...
    'servo_deadband', P.rocket.deadband, ...
    'servo_backlash', P.rocket.backlash, ...
    'servo_pos_noise', 0.01, ...
    'gust_std', P.rocket.wind_strength, ...
    'gust_tau', 0.40, ...
    'keff_drift_rate', 0.10, ...
    'theta_init_bias', deg2rad(0.5));

% Scenario mismatch terms make thrust/aero factors explicit in Exp4.
baseline.scenario = struct( ...
    'fault_time', 0.0, ...
    'control_eff_scale_post', 0.85, ...
    'aero_damp_scale_post', 0.75, ...
    'tau_scale_post', 1.0, ...
    'slew_scale_post', 1.0, ...
    'disturb_scale_post', 1.0, ...
    'disturb_bias_post', 0.0);
end


function ideal = ideal_profile()
ideal.realism = struct( ...
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

ideal.scenario = struct( ...
    'fault_time', 0.0, ...
    'control_eff_scale_post', 1.0, ...
    'aero_damp_scale_post', 1.0, ...
    'tau_scale_post', 1.0, ...
    'slew_scale_post', 1.0, ...
    'disturb_scale_post', 1.0, ...
    'disturb_bias_post', 0.0);
end


function factors = factor_specs()
factors = [ ...
    struct('name', 'deadband', 'realism_fields', {{'servo_deadband'}}, 'scenario_fields', {{}}), ...
    struct('name', 'backlash', 'realism_fields', {{'servo_backlash'}}, 'scenario_fields', {{}}), ...
    struct('name', 'latency', 'realism_fields', {{'sensor_latency_steps'}}, 'scenario_fields', {{}}), ...
    struct('name', 'sensor_noise', 'realism_fields', {{'gyro_noise_std','gyro_bias_init','gyro_bias_rw','gyro_quant_lsb','bias_cal_residual_frac','theta_init_bias'}}, 'scenario_fields', {{}}), ...
    struct('name', 'thrust_mismatch', 'realism_fields', {{}}, 'scenario_fields', {{'control_eff_scale_post'}}), ...
    struct('name', 'aero_mismatch', 'realism_fields', {{}}, 'scenario_fields', {{'aero_damp_scale_post'}}), ...
    struct('name', 'effectiveness_drift', 'realism_fields', {{'keff_drift_rate'}}, 'scenario_fields', {{}}) ...
    ];
end


function sc = apply_baseline_scenario(sc, baseline_sc)
fns = fieldnames(baseline_sc);
for i = 1:numel(fns)
    sc.(fns{i}) = baseline_sc.(fns{i});
end
end


function score = importance_score(success_recovery, rms_recovery, s2r_reduction, full_rms, full_s2r)
succ_term = max(0, success_recovery);
rms_term = max(0, rms_recovery) / max(1e-6, full_rms);
s2r_term = max(0, s2r_reduction) / max(1e-6, full_s2r);
score = succ_term + 0.5 * rms_term + 0.5 * s2r_term;
end


function out = evaluate_fixed_pd(cfg, sc, opts, realism)
n_trials = numel(opts.theta0_deg_set) * numel(opts.seeds);
wins = 0;
rms_vals = zeros(n_trials, 1);
peak_vals = zeros(n_trials, 1);
end_vals = zeros(n_trials, 1);
u_sat_vals = zeros(n_trials, 1);
slew_sat_vals = zeros(n_trials, 1);

idx = 0;
for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1;
        seed = opts.seeds(iseed) + 100 * it;
        try
            out_sim = simulate_case_realistic('PID', sc, cfg, seed, realism);
            [stable, rms_deg, peak_deg, end_deg, u_sat, slew_sat] = score_run(out_sim, cfg, opts);
            wins = wins + double(stable);
            rms_vals(idx) = rms_deg;
            peak_vals(idx) = peak_deg;
            end_vals(idx) = end_deg;
            u_sat_vals(idx) = u_sat;
            slew_sat_vals(idx) = slew_sat;
        catch
            rms_vals(idx) = 90;
            peak_vals(idx) = 90;
            end_vals(idx) = 90;
            u_sat_vals(idx) = 1;
            slew_sat_vals(idx) = 1;
        end
    end
end

out.rate = wins / n_trials;
out.mean_rms_deg = mean(rms_vals);
out.mean_peak_deg = mean(peak_vals);
out.mean_end_deg = mean(end_vals);
out.mean_u_sat = mean(u_sat_vals);
out.mean_slew_sat = mean(slew_sat_vals);
end


function [stable, rms_deg, peak_deg, end_deg, u_sat_frac, slew_sat_frac] = score_run(out, cfg, opts)
if ~all(isfinite(out.theta))
    stable = false;
    rms_deg = 90;
    peak_deg = 90;
    end_deg = 90;
    u_sat_frac = 1;
    slew_sat_frac = 1;
    return;
end

rms_deg = min(90, rad2deg(sqrt(mean(out.theta .^ 2))));
peak_deg = min(90, rad2deg(max(abs(out.theta))));
end_deg = min(90, rad2deg(abs(out.theta(end))));
max_theta_deg = min(90, rad2deg(max(abs(out.theta))));
u_sat_frac = mean(abs(out.u_cmd) >= 0.99 * cfg.plant.u_max);
slew_sat_frac = mean(abs(diff(out.u_act)) / cfg.dt >= 0.99 * cfg.plant.slew_max);

stable = rms_deg <= opts.success_rms_deg && ...
    peak_deg <= opts.success_peak_deg && ...
    end_deg <= opts.success_end_deg && ...
    max_theta_deg <= opts.success_max_theta_deg;
end


function aggT = summarize_regime_factor_medians(T)
regimes = unique(string(T.regime));
factors = unique(string(T.factor));
all_pct = compute_importance_pct_allrows(T);
rows = cell(0, 10);
for i = 1:numel(regimes)
    for j = 1:numel(factors)
        mask = string(T.regime) == regimes(i) & string(T.factor) == factors(j);
        if ~any(mask)
            continue;
        end

        sub = T(mask, :);
        rows(end + 1, :) = { ...
            regimes(i), factors(j), ...
            median(all_pct(mask)), ...
            median(sub.success_recovery), ...
            median(sub.rms_recovery_deg), ...
            median(sub.s2r_reduction), ...
            mean(sub.success_recovery > 0), ...
            mean(sub.s2r_reduction > 0), ...
            height(sub), numel(unique(sub.cell_id))};
    end
end

aggT = cell2table(rows, 'VariableNames', { ...
    'regime', 'factor', 'median_importance_pct', 'median_success_increase', ...
    'median_rms_recovery_deg', 'median_s2r_reduction', ...
    'frac_cells_success_positive', 'frac_cells_s2r_positive', ...
    'n_rows', 'n_cells'});

aggT = sortrows(aggT, {'regime', 'median_importance_pct'}, {'ascend', 'descend'});
end


function pct = compute_importance_pct_allrows(T)
score = max(0, T.importance_score);
pct = zeros(height(T), 1);
ids = unique(T.cell_id);
for i = 1:numel(ids)
    mask = T.cell_id == ids(i);
    s = score(mask);
    den = sum(s);
    if den > 0
        pct(mask) = 100 * s / den;
    end
end
end


function top3T = summarize_top3(rankT)
regimes = unique(string(rankT.regime));
factors = unique(string(rankT.factor));
rows = cell(0, 8);
for i = 1:numel(regimes)
    sub_r = rankT(string(rankT.regime) == regimes(i), :);
    n_cells = numel(unique(sub_r.cell_id));
    for j = 1:numel(factors)
        sub = sub_r(string(sub_r.factor) == factors(j), :);
        if isempty(sub)
            continue;
        end
        rows(end + 1, :) = { ...
            regimes(i), factors(j), ...
            sum(sub.rank == 1), ...
            sum(sub.rank == 2), ...
            sum(sub.rank == 3), ...
            mean(sub.rank == 1), ...
            median(sub.importance_pct), ...
            n_cells};
    end
end

top3T = cell2table(rows, 'VariableNames', { ...
    'regime', 'factor', 'count_rank1', 'count_rank2', 'count_rank3', ...
    'frac_rank1', 'median_importance_pct_top3', 'n_cells'});
top3T = sortrows(top3T, {'regime', 'frac_rank1'}, {'ascend', 'descend'});
end


function agg = summarize_interaction_gaps(T)
regimes = unique(string(T.regime));
rows = cell(0, 13);
for i = 1:numel(regimes)
    mask = string(T.regime) == regimes(i);
    sub = T(mask, :);
    rows(end + 1, :) = { ...
        regimes(i), ...
        median(sub.combined_success_recovery), median(sub.sum_single_success_recovery), median(sub.interaction_gap_success), ...
        median(sub.combined_rms_recovery_deg), median(sub.sum_single_rms_recovery_deg), median(sub.interaction_gap_rms_recovery_deg), ...
        median(sub.combined_s2r_reduction), median(sub.sum_single_s2r_reduction), median(sub.interaction_gap_s2r_reduction), ...
        median(sub.combined_score), median(sub.sum_single_score), median(sub.interaction_gap_score)};
end

agg = cell2table(rows, 'VariableNames', { ...
    'regime', ...
    'median_combined_success_recovery', 'median_sum_single_success_recovery', 'median_interaction_gap_success', ...
    'median_combined_rms_recovery_deg', 'median_sum_single_rms_recovery_deg', 'median_interaction_gap_rms_recovery_deg', ...
    'median_combined_s2r_reduction', 'median_sum_single_s2r_reduction', 'median_interaction_gap_s2r_reduction', ...
    'median_combined_score', 'median_sum_single_score', 'median_interaction_gap_score'});
end


function T = build_factor_toggle_table(factors, baseline, ideal)
rows = cell(0, 7);
for i = 1:numel(factors)
    fac = factors(i);
    for j = 1:numel(fac.realism_fields)
        f = fac.realism_fields{j};
        b = baseline.realism.(f);
        q = ideal.realism.(f);
        rows(end + 1, :) = {string(fac.name), "realism", string(f), b, q, q - b, abs(q - b)};
    end
    for j = 1:numel(fac.scenario_fields)
        f = fac.scenario_fields{j};
        b = baseline.scenario.(f);
        q = ideal.scenario.(f);
        rows(end + 1, :) = {string(fac.name), "scenario", string(f), b, q, q - b, abs(q - b)};
    end
end

T = cell2table(rows, 'VariableNames', { ...
    'factor', 'domain', 'field', 'baseline_value', 'ideal_value', 'delta', 'abs_delta'});
end


function r = safe_ratio(a, b)
if abs(b) < 1e-12
    if abs(a) < 1e-12
        r = 1.0;
    else
        r = NaN;
    end
else
    r = a / b;
end
end


function plot_heatmaps(aggT, out_png)
regimes = ["EASY", "FRAGILE", "INFEASIBLE"];
factors = ["deadband", "backlash", "latency", "sensor_noise", "thrust_mismatch", "aero_mismatch", "effectiveness_drift"];

M_imp = grid_metric(aggT, regimes, factors, 'median_importance_pct');
M_succ = grid_metric(aggT, regimes, factors, 'median_success_increase');
M_s2r = grid_metric(aggT, regimes, factors, 'median_s2r_reduction');

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1480 520]);
tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
imagesc(M_imp);
title('Median importance (%)');
stylize_axes(regimes, factors);
annotate_cells(M_imp, '%.1f');

nexttile;
imagesc(M_succ);
title('Median success increase');
stylize_axes(regimes, factors);
annotate_cells(M_succ, '%.3f');

nexttile;
imagesc(M_s2r);
title('Median S2R reduction');
stylize_axes(regimes, factors);
annotate_cells(M_s2r, '%.3f');

colormap(parula(256));
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Label.String = 'Metric value';

exportgraphics(fig, out_png, 'Resolution', 260);
close(fig);
end


function M = grid_metric(T, regimes, factors, varname)
M = nan(numel(regimes), numel(factors));
for i = 1:numel(regimes)
    for j = 1:numel(factors)
        mask = string(T.regime) == regimes(i) & string(T.factor) == factors(j);
        if any(mask)
            M(i, j) = median(T.(varname)(mask));
        end
    end
end
end


function stylize_axes(regimes, factors)
axis tight;
grid on;
set(gca, 'YTick', 1:numel(regimes), 'YTickLabel', cellstr(regimes), ...
    'XTick', 1:numel(factors), 'XTickLabel', strrep(cellstr(factors), '_', '\_'), ...
    'XTickLabelRotation', 25, 'FontSize', 10);
ylabel('Regime');
xlabel('Factor');
end


function annotate_cells(M, fmt)
for r = 1:size(M, 1)
    for c = 1:size(M, 2)
        if isfinite(M(r, c))
            text(c, r, sprintf(fmt, M(r, c)), 'HorizontalAlignment', 'center', ...
                'Color', 'w', 'FontWeight', 'bold', 'FontSize', 9);
        end
    end
end
end


function c = closeness_to_set(A, B)
if isempty(B)
    c = zeros(height(A), 1);
    return;
end

Ar = [A.p_unstable, A.servo_slew_deg_s, A.max_gimbal_deg];
Br = [B.p_unstable, B.servo_slew_deg_s, B.max_gimbal_deg];

all_pts = [Ar; Br];
mn = min(all_pts, [], 1);
mx = max(all_pts, [], 1);
span = max(mx - mn, 1e-9);

Ar = (Ar - mn) ./ span;
Br = (Br - mn) ./ span;

c = zeros(size(Ar, 1), 1);
for i = 1:size(Ar, 1)
    d = sum(abs(Br - Ar(i, :)), 2);
    c(i) = 1 / (1 + min(d));
end
end


function Xn = normalize_rows(X)
mn = min(X, [], 1);
mx = max(X, [], 1);
span = max(mx - mn, 1e-9);
Xn = (X - mn) ./ span;
end


function y = normalize01(x)
x = double(x);
mn = min(x);
mx = max(x);
if abs(mx - mn) < 1e-12
    y = zeros(size(x));
else
    y = (x - mn) ./ (mx - mn);
end
end