function [trajT, summaryT] = run_exp4_fidelity_ladder(maxCells)
%RUN_EXP4_FIDELITY_LADDER
% Exp4: minimum simulator fidelity needed to reproduce highest-fidelity decision.
%
% Decision definition: GO / MARGINAL / NOGO from tuned-PD success rate.

if nargin < 1
    maxCells = inf;
end

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
fw = fullfile(here, 'framework');
results_dir = fullfile(here, 'results');
graphs_dir = fullfile(results_dir, 'graphs');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'); mkdir(graphs_dir); end

addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(fw, 'plant'));
addpath(fullfile(fw, 'controllers'));
addpath(fullfile(fw, 'analysis'));

index_path = fullfile(results_dir, 'exp1_regime_index.csv');
phase_path = fullfile(results_dir, 'phase_diagram.csv');
if exist(index_path, 'file')
    idxT = readtable(index_path);
elseif exist(phase_path, 'file')
    idxT = readtable(phase_path);
    if ~ismember('rocket_id', idxT.Properties.VariableNames)
        idxT.rocket_id = compose("R%04d", (1:height(idxT))');
    end
    if ~ismember('boundary_distance', idxT.Properties.VariableNames)
        idxT.boundary_distance = zeros(height(idxT), 1);
    end
else
    error('run_exp4_fidelity_ladder:missingExp1', ...
        'Missing %s and %s. Run Exp1 first.', index_path, phase_path);
end

if isfinite(maxCells)
    idxT = idxT(1:min(height(idxT), maxCells), :);
end

P = default_rocket_params();
rows = cell(0, 27);
summary_rows = cell(0, 20);

for i = 1:height(idxT)
    r = idxT(i, :);
    P_cell = params_from_row(P, r);
    levels = fidelity_levels(P_cell);
    [decisions, level_metrics] = evaluate_cell_across_levels(r, levels, P_cell);

    ref_decision = decisions{end};
    first_match = find(strcmp(decisions, ref_decision), 1, 'first');
    if isempty(first_match)
        first_match = numel(levels);
    end

    if first_match <= 1
        dominant_missing = "NONE";
    else
        dominant_missing = string(levels(first_match).added_physics);
    end

    history = strjoin(compose("%s:%s", string({levels.name})', string(decisions)'), ' -> ');

    for k = 1:numel(levels)
        m = level_metrics(k);
        rows(end + 1, :) = { ...
            string(r.rocket_id), string(r.regime_label), double(r.boundary_distance), ...
            string(levels(k).name), string(decisions{k}), m.success_rate, m.rms_error_deg, ...
            m.u_sat_frac, m.slew_sat_frac, m.Kp, m.Kd, ...
            string(levels(k).added_physics), string(ref_decision), first_match, history, ...
            P_cell.rocket.mass, P_cell.rocket.Iyy, P_cell.rocket.static_margin, ...
            P_cell.rocket.Cm_alpha, P_cell.rocket.control_effectiveness, ...
            P_cell.rocket.thrust, P_cell.rocket.deadband, P_cell.rocket.backlash, ...
            P_cell.rocket.latency, P_cell.rocket.wind_strength, ...
            P_cell.rocket.servo_slew, P_cell.rocket.max_gimbal}; %#ok<AGROW>
    end

    summary_rows(end + 1, :) = { ...
        string(r.rocket_id), string(r.regime_label), double(r.boundary_distance), ...
        string(ref_decision), first_match, string(levels(first_match).name), ...
        dominant_missing, history, ...
        P_cell.rocket.mass, P_cell.rocket.Iyy, P_cell.rocket.static_margin, ...
        P_cell.rocket.Cm_alpha, P_cell.rocket.control_effectiveness, ...
        P_cell.rocket.thrust, P_cell.rocket.deadband, P_cell.rocket.backlash, ...
        P_cell.rocket.latency, P_cell.rocket.wind_strength, ...
        P_cell.rocket.servo_slew, P_cell.rocket.max_gimbal}; %#ok<AGROW>
end

trajT = cell2table(rows, 'VariableNames', { ...
    'rocket_id', 'regime_label', 'boundary_distance', 'fidelity_level', ...
    'decision', 'success_rate', 'rms_error_deg', 'u_sat_frac', ...
    'slew_sat_frac', 'Kp', 'Kd', 'added_physics', ...
    'reference_decision', 'first_correct_fidelity_level', 'decision_history', ...
    'mass', 'Iyy', 'static_margin', 'Cm_alpha', 'control_effectiveness', ...
    'thrust', 'deadband', 'backlash', 'latency', 'wind_strength', ...
    'nominal_servo_slew_deg_s', 'nominal_max_gimbal_deg'});
trajT = annotate_pipeline_table(trajT, P, 'Exp4', 'fidelity_ladder_trajectories');

summaryT = cell2table(summary_rows, 'VariableNames', { ...
    'rocket_id', 'regime_label', 'boundary_distance', 'reference_decision', ...
    'first_correct_fidelity_level', 'first_correct_fidelity_name', ...
    'dominant_missing_physics', 'decision_history', ...
    'mass', 'Iyy', 'static_margin', 'Cm_alpha', 'control_effectiveness', ...
    'thrust', 'deadband', 'backlash', 'latency', 'wind_strength', ...
    'nominal_servo_slew_deg_s', 'nominal_max_gimbal_deg'});
summaryT = annotate_pipeline_table(summaryT, P, 'Exp4', 'fidelity_ladder_summary');

writetable(trajT, fullfile(results_dir, 'exp4_fidelity_decision_trajectories.csv'));
writetable(summaryT, fullfile(results_dir, 'exp4_first_correct_fidelity.csv'));
plot_first_correct_heatmap(summaryT, fullfile(graphs_dir, 'exp4_first_correct_fidelity_heatmap.png'));

fprintf('Saved: %s\n', fullfile(results_dir, 'exp4_fidelity_decision_trajectories.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp4_first_correct_fidelity.csv'));
fprintf('Saved: %s\n', fullfile(graphs_dir, 'exp4_first_correct_fidelity_heatmap.png'));
end


function levels = fidelity_levels(P)
levels = struct([]);

levels(1).name = 'L0_BASELINE';
levels(1).added_physics = 'NONE';
levels(1).apply = @(cfg, sc, realism) set_level(cfg, sc, realism, struct( ...
    'u_scale', 4.0, 'slew_scale', 4.0, 'deadband', 0.0, 'backlash', 0.0, ...
    'latency', 1, 'sensor_on', false, 'aero_uncertainty_on', false));

levels(2).name = 'L1_SATURATION';
levels(2).added_physics = 'ACTUATOR_SATURATION';
levels(2).apply = @(cfg, sc, realism) set_level(cfg, sc, realism, struct( ...
    'u_scale', 1.0, 'slew_scale', 4.0, 'deadband', 0.0, 'backlash', 0.0, ...
    'latency', 1, 'sensor_on', false, 'aero_uncertainty_on', false));

levels(3).name = 'L2_SLEW_LIMIT';
levels(3).added_physics = 'SLEW_LIMIT';
levels(3).apply = @(cfg, sc, realism) set_level(cfg, sc, realism, struct( ...
    'u_scale', 1.0, 'slew_scale', 1.0, 'deadband', 0.0, 'backlash', 0.0, ...
    'latency', 1, 'sensor_on', false, 'aero_uncertainty_on', false));

levels(4).name = 'L3_ACTUATOR_NONLINEAR';
levels(4).added_physics = 'DEADBAND_BACKLASH';
levels(4).apply = @(cfg, sc, realism) set_level(cfg, sc, realism, struct( ...
    'u_scale', 1.0, 'slew_scale', 1.0, 'deadband', P.rocket.deadband, 'backlash', P.rocket.backlash, ...
    'latency', 1, 'sensor_on', false, 'aero_uncertainty_on', false));

levels(5).name = 'L4_SENSOR_EFFECTS';
levels(5).added_physics = 'SENSOR_LATENCY_NOISE';
levels(5).apply = @(cfg, sc, realism) set_level(cfg, sc, realism, struct( ...
    'u_scale', 1.0, 'slew_scale', 1.0, 'deadband', P.rocket.deadband, 'backlash', P.rocket.backlash, ...
    'latency', P.rocket.latency, 'sensor_on', true, 'aero_uncertainty_on', false));

levels(6).name = 'L5_FULL';
levels(6).added_physics = 'AERO_UNCERTAINTY_AND_DRIFT';
levels(6).apply = @(cfg, sc, realism) set_level(cfg, sc, realism, struct( ...
    'u_scale', 1.0, 'slew_scale', 1.0, 'deadband', P.rocket.deadband, 'backlash', P.rocket.backlash, ...
    'latency', P.rocket.latency, 'sensor_on', true, 'aero_uncertainty_on', true));
end


function [cfg, sc, realism] = set_level(cfg, sc, realism, opts)
cfg.plant.u_max = cfg.plant.u_max * opts.u_scale;
cfg.plant.slew_max = cfg.plant.slew_max * opts.slew_scale;

realism.servo_deadband = opts.deadband;
realism.servo_backlash = opts.backlash;
realism.sensor_latency_steps = max(1, round(opts.latency));

if opts.sensor_on
    realism.gyro_noise_std = 0.015;
    realism.gyro_bias_init = 0.010;
    realism.gyro_bias_rw = 0.005;
    realism.gyro_quant_lsb = 2 * pi / 4000;
    realism.bias_cal_residual_frac = 0.10;
    realism.theta_init_bias = deg2rad(0.5);
else
    realism.gyro_noise_std = 0;
    realism.gyro_bias_init = 0;
    realism.gyro_bias_rw = 0;
    realism.gyro_quant_lsb = 0;
    realism.bias_cal_residual_frac = 0;
    realism.theta_init_bias = 0;
end

if opts.aero_uncertainty_on
    sc.control_eff_scale_post = 0.85;
    sc.aero_damp_scale_post = 0.75;
    realism.keff_drift_rate = 0.10;
    realism.gust_std = max(realism.gust_std, 0.15);
else
    sc.control_eff_scale_post = 1.0;
    sc.aero_damp_scale_post = 1.0;
    realism.keff_drift_rate = 0.0;
    realism.gust_std = 0.0;
end
end


function [decisions, metrics] = evaluate_cell_across_levels(r, levels, P)
fw = fileparts(mfilename('fullpath'));
addpath(fullfile(fw, 'framework', 'plant'));
addpath(fullfile(fw, 'framework', 'controllers'));
addpath(fullfile(fw, 'framework', 'analysis'));

P_fast = P;
P_fast.analysis.theta0_deg_set = 0;
P_fast.analysis.seeds = 1;
P_fast.tuning.Kp_grid = [20 45 80];
P_fast.tuning.Kd_grid = [8 16 32];

override = struct();
override.p_unstable = r.p_unstable;
override.servo_slew = r.servo_slew_deg_s;
override.u_max_frac = r.best_u_max_frac;
override.control_effectiveness = value_or_default(r, 'control_effectiveness', P.rocket.control_effectiveness);
override.deadband = value_or_default(r, 'deadband', P.rocket.deadband);
override.backlash = value_or_default(r, 'backlash', P.rocket.backlash);
override.latency = value_or_default(r, 'latency_steps', value_or_default(r, 'latency', P.rocket.latency));
override.wind_strength = value_or_default(r, 'wind_strength', P.rocket.wind_strength);
[cfg_base, sc_base, realism_base] = build_realistic_cfg(P, override);

decisions = cell(numel(levels), 1);
metrics = repmat(struct('success_rate', 0, 'rms_error_deg', 90, 'u_sat_frac', 1, ...
    'slew_sat_frac', 1, 'Kp', NaN, 'Kd', NaN), numel(levels), 1);

for k = 1:numel(levels)
    cfg = cfg_base;
    sc = sc_base;
    realism = realism_base;
    [cfg, sc, realism] = levels(k).apply(cfg, sc, realism);

    tuned = autotune_pd_grid(cfg, sc, realism, P_fast);
    eval_cfg = configure_pid_controller(cfg, tuned.Kp, tuned.Kd);
    ev = evaluate_stability_cell(eval_cfg, sc, realism, P_fast);

    decisions{k} = verdict_from_success(ev.success_rate);
    metrics(k).success_rate = ev.success_rate;
    metrics(k).rms_error_deg = ev.rms_error_deg;
    metrics(k).u_sat_frac = ev.u_cmd_sat_frac;
    metrics(k).slew_sat_frac = ev.slew_sat_frac;
    metrics(k).Kp = tuned.Kp;
    metrics(k).Kd = tuned.Kd;
end
end


function P_out = params_from_row(P_in, r)
P_out = P_in;
P_out.rocket.mass = value_or_default(r, 'mass', P_in.rocket.mass);
P_out.rocket.Iyy = value_or_default(r, 'Iyy', P_in.rocket.Iyy);
P_out.rocket.static_margin = value_or_default(r, 'static_margin', P_in.rocket.static_margin);
P_out.rocket.Cm_alpha = value_or_default(r, 'Cm_alpha', P_in.rocket.Cm_alpha);
P_out.rocket.control_effectiveness = value_or_default(r, 'control_effectiveness', P_in.rocket.control_effectiveness);
P_out.rocket.thrust = value_or_default(r, 'thrust', P_in.rocket.thrust);
P_out.rocket.deadband = value_or_default(r, 'deadband', P_in.rocket.deadband);
P_out.rocket.backlash = value_or_default(r, 'backlash', P_in.rocket.backlash);
P_out.rocket.latency = value_or_default(r, 'latency_steps', value_or_default(r, 'latency', P_in.rocket.latency));
P_out.rocket.wind_strength = value_or_default(r, 'wind_strength', P_in.rocket.wind_strength);
P_out.rocket.servo_slew = value_or_default(r, 'servo_slew_deg_s', value_or_default(r, 'nominal_servo_slew_deg_s', P_in.rocket.servo_slew));
P_out.rocket.max_gimbal = value_or_default(r, 'max_gimbal_deg', value_or_default(r, 'nominal_max_gimbal_deg', P_in.rocket.max_gimbal));
end


function v = value_or_default(r, name, fallback)
if ismember(name, r.Properties.VariableNames)
    vv = r.(name)(1);
    if iscell(vv)
        vv = vv{1};
    end
    if isstring(vv) || ischar(vv)
        vnum = str2double(string(vv));
        if isfinite(vnum)
            v = vnum;
        else
            v = fallback;
        end
        return;
    end
    if isnumeric(vv) && isfinite(vv)
        v = double(vv);
        return;
    end
end
v = fallback;
end


function verdict = verdict_from_success(rate)
if rate >= 0.80
    verdict = 'GO';
elseif rate >= 0.35
    verdict = 'MARGINAL';
else
    verdict = 'NOGO';
end
end


function plot_first_correct_heatmap(summaryT, out_png)
if isempty(summaryT)
    return;
end

regimes = unique(string(summaryT.regime_label), 'stable');
maxL = max(summaryT.first_correct_fidelity_level);
M = nan(numel(regimes), maxL);
for i = 1:numel(regimes)
    sub = summaryT(string(summaryT.regime_label) == regimes(i), :);
    for L = 1:maxL
        M(i, L) = mean(sub.first_correct_fidelity_level == L);
    end
end

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 920 420]);
imagesc(M);
axis tight;
colormap(parula(256));
colorbar;
set(gca, 'YTick', 1:numel(regimes), 'YTickLabel', cellstr(regimes), ...
    'XTick', 1:maxL, 'XTickLabel', compose('L%d', 0:maxL-1), 'FontSize', 11);
xlabel('First Correct Fidelity Level');
ylabel('Regime');
title('Exp4 First Correct Fidelity Level Distribution by Regime');

for r = 1:size(M,1)
    for c = 1:size(M,2)
        if isfinite(M(r,c))
            text(c, r, sprintf('%.2f', M(r,c)), 'HorizontalAlignment', 'center', ...
                'Color', 'w', 'FontWeight', 'bold', 'FontSize', 10);
        end
    end
end

exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);
end
