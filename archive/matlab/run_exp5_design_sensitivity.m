function [cellT, summaryT, gradientT] = run_exp5_design_sensitivity(maxCells)
%RUN_EXP5_DESIGN_SENSITIVITY
% Exp5 design-lever experiment:
% For each rocket/regime cell, identify the physical modification that
% maximizes stability and maneuverability improvement.

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
    error('run_exp5_design_sensitivity:missingExp1', ...
        'Missing %s and %s. Run Exp1 first.', index_path, phase_path);
end

if isfinite(maxCells)
    idxT = idxT(1:min(height(idxT), maxCells), :);
end

P = default_rocket_params();
levers = design_levers(P);
rows = cell(0, 30);
summary_rows = cell(0, 23);

for i = 1:height(idxT)
    r = idxT(i, :);
    P_cell = params_from_row(P, r);
    [base, evalRows] = evaluate_design_levers_for_cell(r, levers, P_cell);

    [~, best_i] = max([evalRows.tradeoff_score]);
    [~, worst_i] = min([evalRows.tradeoff_score]);
    best = evalRows(best_i);
    worst = evalRows(worst_i);

    for k = 1:numel(evalRows)
        e = evalRows(k);
        rows(end + 1, :) = { ...
            string(r.rocket_id), string(r.regime_label), double(r.boundary_distance), ...
            string(e.design_lever), e.base_success_rate, e.lever_success_rate, e.expected_success_improvement, ...
            e.base_rms_deg, e.lever_rms_deg, e.expected_stability_gain, ...
            e.base_envelope_deg, e.lever_envelope_deg, e.expected_maneuverability_gain, ...
            e.tradeoff_score, e.Kp, e.Kd, e.base_Kp, e.base_Kd, ...
            P_cell.rocket.mass, P_cell.rocket.Iyy, P_cell.rocket.static_margin, ...
            P_cell.rocket.Cm_alpha, P_cell.rocket.control_effectiveness, ...
            P_cell.rocket.thrust, P_cell.rocket.deadband, P_cell.rocket.backlash, ...
            P_cell.rocket.latency, P_cell.rocket.wind_strength, ...
            P_cell.rocket.servo_slew, P_cell.rocket.max_gimbal}; %#ok<AGROW>
    end

    summary_rows(end + 1, :) = { ...
        string(r.rocket_id), string(r.regime_label), double(r.boundary_distance), ...
        string(best.design_lever), best.expected_success_improvement, ...
        best.expected_stability_gain, best.expected_maneuverability_gain, best.tradeoff_score, ...
        string(worst.design_lever), worst.tradeoff_score, ...
        sprintf('base_success=%.3f base_env=%.1f', base.success_rate, base.envelope_cmd_deg), ...
        P_cell.rocket.mass, P_cell.rocket.Iyy, P_cell.rocket.static_margin, ...
        P_cell.rocket.Cm_alpha, P_cell.rocket.control_effectiveness, ...
        P_cell.rocket.thrust, P_cell.rocket.deadband, P_cell.rocket.backlash, ...
        P_cell.rocket.latency, P_cell.rocket.wind_strength, ...
        P_cell.rocket.servo_slew, P_cell.rocket.max_gimbal}; %#ok<AGROW>
end

cellT = cell2table(rows, 'VariableNames', { ...
    'rocket_id', 'regime_label', 'boundary_distance', 'design_lever', ...
    'base_success_rate', 'lever_success_rate', 'expected_success_improvement', ...
    'base_rms_deg', 'lever_rms_deg', 'expected_stability_gain', ...
    'base_envelope_deg', 'lever_envelope_deg', 'expected_maneuverability_gain', ...
    'tradeoff_score', 'lever_Kp', 'lever_Kd', 'base_Kp', 'base_Kd', ...
    'mass', 'Iyy', 'static_margin', 'Cm_alpha', 'control_effectiveness', ...
    'thrust', 'deadband', 'backlash', 'latency', 'wind_strength', ...
    'nominal_servo_slew_deg_s', 'nominal_max_gimbal_deg'});
cellT = annotate_pipeline_table(cellT, P, 'Exp5', 'design_lever_cells');

summaryT = cell2table(summary_rows, 'VariableNames', { ...
    'rocket_id', 'regime_label', 'boundary_distance', 'best_design_lever', ...
    'expected_improvement', 'stability_gain_potential', ...
    'maneuverability_gain_potential', 'tradeoff_score', ...
    'worst_bottleneck', 'worst_tradeoff_score', 'baseline_signature', ...
    'mass', 'Iyy', 'static_margin', 'Cm_alpha', 'control_effectiveness', ...
    'thrust', 'deadband', 'backlash', 'latency', 'wind_strength', ...
    'nominal_servo_slew_deg_s', 'nominal_max_gimbal_deg'});
summaryT = annotate_pipeline_table(summaryT, P, 'Exp5', 'design_lever_summary');

gradientT = summarize_gradient_map(cellT);
gradientT = annotate_pipeline_table(gradientT, P, 'Exp5', 'design_lever_gradient_map');

writetable(cellT, fullfile(results_dir, 'exp5_design_lever_cells.csv'));
writetable(summaryT, fullfile(results_dir, 'exp5_design_lever_summary.csv'));
writetable(gradientT, fullfile(results_dir, 'exp5_design_lever_gradient_map.csv'));
plot_gradient_map(gradientT, fullfile(graphs_dir, 'exp5_design_lever_gradient_map.png'));

fprintf('Saved: %s\n', fullfile(results_dir, 'exp5_design_lever_cells.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp5_design_lever_summary.csv'));
fprintf('Saved: %s\n', fullfile(results_dir, 'exp5_design_lever_gradient_map.csv'));
fprintf('Saved: %s\n', fullfile(graphs_dir, 'exp5_design_lever_gradient_map.png'));
end


function [base, evalRows] = evaluate_design_levers_for_cell(r, levers, P)
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

base_tuned = autotune_pd_grid(cfg_base, sc_base, realism_base, P_fast);
base_cfg = configure_pid_controller(cfg_base, base_tuned.Kp, base_tuned.Kd);
base_stab = evaluate_stability_cell(base_cfg, sc_base, realism_base, P_fast);
base_env = evaluate_command_envelope(base_cfg, sc_base, realism_base, struct());

base.success_rate = base_stab.success_rate;
base.rms_error_deg = base_stab.rms_error_deg;
base.envelope_cmd_deg = base_env.envelope_cmd_deg;
base.Kp = base_tuned.Kp;
base.Kd = base_tuned.Kd;

evalRows = repmat(struct(), numel(levers), 1);
for i = 1:numel(levers)
    cfg = cfg_base;
    sc = sc_base;
    realism = realism_base;
    [cfg, sc, realism] = levers(i).apply(cfg, sc, realism);

    tuned = autotune_pd_grid(cfg, sc, realism, P_fast);
    eval_cfg = configure_pid_controller(cfg, tuned.Kp, tuned.Kd);
    ev_stab = evaluate_stability_cell(eval_cfg, sc, realism, P_fast);
    ev_env = evaluate_command_envelope(eval_cfg, sc, realism, struct());

    success_gain = ev_stab.success_rate - base.success_rate;
    stability_gain = base.rms_error_deg - ev_stab.rms_error_deg;
    maneuver_gain = ev_env.envelope_cmd_deg - base.envelope_cmd_deg;

    % Weighted to prioritize robustness while preserving maneuver authority.
    tradeoff_score = success_gain + 0.25 * stability_gain / max(1, base.rms_error_deg) + ...
        0.35 * maneuver_gain / 30.0;

    evalRows(i).design_lever = levers(i).name;
    evalRows(i).base_success_rate = base.success_rate;
    evalRows(i).lever_success_rate = ev_stab.success_rate;
    evalRows(i).expected_success_improvement = success_gain;
    evalRows(i).base_rms_deg = base.rms_error_deg;
    evalRows(i).lever_rms_deg = ev_stab.rms_error_deg;
    evalRows(i).expected_stability_gain = stability_gain;
    evalRows(i).base_envelope_deg = base.envelope_cmd_deg;
    evalRows(i).lever_envelope_deg = ev_env.envelope_cmd_deg;
    evalRows(i).expected_maneuverability_gain = maneuver_gain;
    evalRows(i).tradeoff_score = tradeoff_score;
    evalRows(i).Kp = tuned.Kp;
    evalRows(i).Kd = tuned.Kd;
    evalRows(i).base_Kp = base.Kp;
    evalRows(i).base_Kd = base.Kd;
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


function levers = design_levers(P)
levers = struct([]);

levers(1).name = 'increase_servo_slew_20pct';
levers(1).apply = @(cfg, sc, realism) apply_scale(cfg, sc, realism, struct('slew', 1.20));

levers(2).name = 'increase_gimbal_15pct';
levers(2).apply = @(cfg, sc, realism) apply_scale(cfg, sc, realism, struct('u_max', 1.15));

levers(3).name = 'reduce_backlash_50pct';
levers(3).apply = @(cfg, sc, realism) apply_scale(cfg, sc, realism, struct('backlash', 0.50));

levers(4).name = 'reduce_deadband_50pct';
levers(4).apply = @(cfg, sc, realism) apply_scale(cfg, sc, realism, struct('deadband', 0.50));

levers(5).name = 'reduce_latency_1sample';
levers(5).apply = @(cfg, sc, realism) apply_scale(cfg, sc, realism, struct('latency', -1));

levers(6).name = 'increase_control_effectiveness_10pct';
levers(6).apply = @(cfg, sc, realism) apply_scale(cfg, sc, realism, struct('keff', 1.10));

levers(7).name = 'increase_aero_damping_10pct';
levers(7).apply = @(cfg, sc, realism) apply_scale(cfg, sc, realism, struct('aero_damp', 1.10));

% keep P referenced to avoid linter warnings in some MATLAB setups.
if nargin > 0 && isempty(P)
    error('design_levers:invalid', 'Parameter struct must not be empty.');
end
end


function [cfg, sc, realism] = apply_scale(cfg, sc, realism, delta)
if isfield(delta, 'slew')
    cfg.plant.slew_max = cfg.plant.slew_max * delta.slew;
end
if isfield(delta, 'u_max')
    cfg.plant.u_max = cfg.plant.u_max * delta.u_max;
    cfg.controllers.PID.u_max = cfg.plant.u_max;
    cfg.controllers.PID.i_lim = cfg.plant.u_max;
end
if isfield(delta, 'backlash')
    realism.servo_backlash = realism.servo_backlash * delta.backlash;
end
if isfield(delta, 'deadband')
    realism.servo_deadband = realism.servo_deadband * delta.deadband;
end
if isfield(delta, 'latency')
    realism.sensor_latency_steps = max(1, round(realism.sensor_latency_steps + delta.latency));
end
if isfield(delta, 'keff')
    cfg.plant.control_eff = cfg.plant.control_eff * delta.keff;
    cfg.plant.keff_nom = cfg.plant.control_eff;
end
if isfield(delta, 'aero_damp')
    cfg.plant.aero_damp = cfg.plant.aero_damp * delta.aero_damp;
end
end


function gradientT = summarize_gradient_map(cellT)
regimes = unique(string(cellT.regime_label), 'stable');
levers = unique(string(cellT.design_lever), 'stable');
rows = cell(0, 6);
for i = 1:numel(regimes)
    for j = 1:numel(levers)
        sub = cellT(string(cellT.regime_label) == regimes(i) & string(cellT.design_lever) == levers(j), :);
        if isempty(sub)
            continue;
        end
        rows(end + 1, :) = { ...
            regimes(i), levers(j), mean(sub.expected_success_improvement, 'omitnan'), ...
            mean(sub.expected_stability_gain, 'omitnan'), ...
            mean(sub.expected_maneuverability_gain, 'omitnan'), ...
            mean(sub.tradeoff_score, 'omitnan')}; %#ok<AGROW>
    end
end

gradientT = cell2table(rows, 'VariableNames', { ...
    'regime_label', 'design_lever', 'mean_success_gain', ...
    'mean_stability_gain', 'mean_maneuverability_gain', 'mean_tradeoff_score'});
end


function plot_gradient_map(gradientT, out_png)
if isempty(gradientT)
    return;
end

regimes = unique(string(gradientT.regime_label), 'stable');
levers = unique(string(gradientT.design_lever), 'stable');
M = nan(numel(regimes), numel(levers));
for i = 1:numel(regimes)
    for j = 1:numel(levers)
        mask = string(gradientT.regime_label) == regimes(i) & string(gradientT.design_lever) == levers(j);
        if any(mask)
            M(i, j) = gradientT.mean_tradeoff_score(mask);
        end
    end
end

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1280 420]);
imagesc(M);
axis tight;
colormap(parula(256));
colorbar;
set(gca, 'YTick', 1:numel(regimes), 'YTickLabel', cellstr(regimes), ...
    'XTick', 1:numel(levers), 'XTickLabel', strrep(cellstr(levers), '_', '\_'), ...
    'XTickLabelRotation', 25, 'FontSize', 10);
title('Exp5 Design-Lever Tradeoff Score by Regime');
ylabel('Regime');
xlabel('Design lever');

for r = 1:size(M,1)
    for c = 1:size(M,2)
        if isfinite(M(r,c))
            text(c, r, sprintf('%.3f', M(r,c)), 'HorizontalAlignment', 'center', ...
                'Color', 'w', 'FontWeight', 'bold', 'FontSize', 9);
        end
    end
end

exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);
end
