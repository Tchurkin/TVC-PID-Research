function [cellMetricsT, regimeSummaryT, statsT] = run_exp2_physical_audit()
%RUN_EXP2_PHYSICAL_AUDIT
% Physics-first audit of Exp2 using continuous maneuverability metrics.

this_dir = fileparts(mfilename('fullpath'));
framework_dir = fullfile(this_dir, 'framework');
proj_dir = fileparts(this_dir);

addpath(fullfile(proj_dir, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(framework_dir, 'plant'));
addpath(fullfile(framework_dir, 'controllers'));

P = default_rocket_params();

results_dir = fullfile(this_dir, 'results');
graphs_dir = fullfile(results_dir, 'graphs');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'); mkdir(graphs_dir); end

phase_path = fullfile(results_dir, 'phase_diagram.csv');
if ~exist(phase_path, 'file')
    error('run_exp2_physical_audit:missingExp1', 'Missing %s. Run Exp1 first.', phase_path);
end
phaseT = readtable(phase_path);

exp2_cell_path = fullfile(results_dir, 'exp2_maneuverability_cells.csv');
if exist(exp2_cell_path, 'file')
    exp2CellT = readtable(exp2_cell_path);
else
    exp2CellT = table();
end

opts = struct();
opts.fast_mode = true;
opts.cmd_deg = 20.0;
opts.cmd_start_s = 0.50;
opts.t_end = 3.0;
opts.settle_band_deg = 2.0;
opts.overshoot_floor_deg = 1.0;
opts.min_settle_window_s = 0.20;
opts.fail_err_deg = 30.0;
opts.fail_theta_deg = 60.0;
if opts.fast_mode
    opts.eval_seeds = P.analysis.seeds(1:min(2, numel(P.analysis.seeds)));
    opts.eval_theta0_deg_set = P.analysis.theta0_deg_set(1);
else
    opts.eval_seeds = P.analysis.seeds;
    opts.eval_theta0_deg_set = P.analysis.theta0_deg_set;
end

fprintf('=== EXP2 PHYSICAL AUDIT ===\n');
fprintf('Cells: %d | cmd=%.1f deg | seeds=%s | theta0=%s\n', ...
    height(phaseT), opts.cmd_deg, mat2str(opts.eval_seeds), mat2str(opts.eval_theta0_deg_set));

cellRows = cell(0, 26);

for i = 1:height(phaseT)
    r = phaseT(i, :);

    override = struct();
    override.p_unstable = r.p_unstable;
    override.servo_slew = r.servo_slew_deg_s;
    override.u_max_frac = r.best_u_max_frac;
    override.deadband = P.rocket.deadband;
    override.backlash = P.rocket.backlash;
    override.latency = P.rocket.latency;
    override.wind_strength = P.rocket.wind_strength;
    override.t_end = opts.t_end;

    [cfg, sc, realism] = build_realistic_cfg(P, override);
    [Kp_use, Kd_use] = pick_cell_gains(r, exp2CellT);
    cfg = configure_pid_controller(cfg, Kp_use, Kd_use);
    sc_cmd = apply_step_program(sc, opts);

    m = evaluate_continuous_cell(cfg, sc_cmd, realism, opts);

    cellRows(end + 1, :) = { ...
        i, string(r.regime_label{1}), r.regime_code, ...
        r.robustness, r.p_unstable, r.servo_slew_deg_s, r.max_gimbal_deg, r.best_u_max_frac, ...
        Kp_use, Kd_use, ...
        m.max_ang_accel_deg_s2, m.rise_time_s, m.settling_time_s, m.overshoot_pct, ...
        m.peak_att_rate_deg_s, m.control_effort_usage, m.actuator_sat_frac, m.slew_sat_frac, ...
        m.success_like_rate, m.rms_err_deg, m.peak_err_deg, m.end_err_deg, m.n_trials, ...
        m.rise_reached_frac, m.settled_frac, m.bounded_window_s}; %#ok<AGROW>

    fprintf(['  cell %3d/%3d | %s p=%g slew=%g gimbal=%g | ' ...
        'acc=%.1f rate=%.1f rise=%.2f settle=%.2f\n'], ...
        i, height(phaseT), r.regime_label{1}, r.p_unstable, r.servo_slew_deg_s, r.max_gimbal_deg, ...
        m.max_ang_accel_deg_s2, m.peak_att_rate_deg_s, m.rise_time_s, m.settling_time_s);
end

cellMetricsT = cell2table(cellRows, 'VariableNames', { ...
    'cell_id', 'regime_label', 'regime_code', ...
    'robustness', 'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'u_max_frac', ...
    'Kp', 'Kd', ...
    'max_ang_accel_deg_s2', 'rise_time_s', 'settling_time_s', 'overshoot_pct', ...
    'peak_att_rate_deg_s', 'control_effort_usage', 'actuator_sat_frac', 'slew_sat_frac', ...
    'success_like_rate', 'rms_err_deg', 'peak_err_deg', 'end_err_deg', 'n_trials', ...
    'rise_reached_frac', 'settled_frac', 'bounded_window_s'});

cellMetricsT.boundary_slew_deg_s = estimate_boundary_slew(phaseT, cellMetricsT);
cellMetricsT.slew_margin_to_boundary = cellMetricsT.servo_slew_deg_s - cellMetricsT.boundary_slew_deg_s;
cellMetricsT.agility_index = build_agility_index(cellMetricsT);

regimeSummaryT = summarize_by_regime(cellMetricsT);
statsT = build_audit_stats(cellMetricsT);

cell_csv = fullfile(results_dir, 'exp2_physical_metrics_cells.csv');
regime_csv = fullfile(results_dir, 'exp2_physical_metrics_regime_summary.csv');
stats_csv = fullfile(results_dir, 'exp2_physical_audit_stats.csv');

writetable(cellMetricsT, cell_csv);
writetable(regimeSummaryT, regime_csv);
writetable(statsT, stats_csv);

plot_metric_vs_robustness(cellMetricsT, fullfile(graphs_dir, 'exp2_physical_metric_vs_robustness.png'));
plot_regime_distributions(cellMetricsT, fullfile(graphs_dir, 'exp2_physical_regime_distributions.png'));
plot_pareto(cellMetricsT, fullfile(graphs_dir, 'exp2_physical_pareto.png'));

fprintf('\nSaved: %s\n', cell_csv);
fprintf('Saved: %s\n', regime_csv);
fprintf('Saved: %s\n', stats_csv);
fprintf('Saved: %s\n', fullfile(graphs_dir, 'exp2_physical_metric_vs_robustness.png'));
fprintf('Saved: %s\n', fullfile(graphs_dir, 'exp2_physical_regime_distributions.png'));
fprintf('Saved: %s\n', fullfile(graphs_dir, 'exp2_physical_pareto.png'));


function [Kp_use, Kd_use] = pick_cell_gains(r, exp2CellT)
Kp_use = r.best_Kp;
Kd_use = r.best_Kd;

if isempty(exp2CellT)
    return;
end

mask = exp2CellT.p_unstable == r.p_unstable & ...
    exp2CellT.servo_slew_deg_s == r.servo_slew_deg_s & ...
    exp2CellT.max_gimbal_deg == r.max_gimbal_deg;

idx = find(mask, 1);
if ~isempty(idx)
    Kp_use = exp2CellT.Kp(idx);
    Kd_use = exp2CellT.Kd(idx);
end


function sc = apply_step_program(sc, opts)
sc.kind = "EXP2_PHYSICAL_STEP";
sc.t_end = opts.t_end;
sc.theta_ref_fun = @(t) step_ref_rad(t, opts);
sc.q_ref_fun = @(t) 0.0;


function th = step_ref_rad(t, opts)
if t >= opts.cmd_start_s
    th = deg2rad(opts.cmd_deg);
else
    th = 0.0;
end


function M = evaluate_continuous_cell(cfg, sc, realism, opts)
n_trials = numel(opts.eval_theta0_deg_set) * numel(opts.eval_seeds);

accel = nan(n_trials, 1);
rise = nan(n_trials, 1);
settle = nan(n_trials, 1);
overshoot = nan(n_trials, 1);
peak_rate = nan(n_trials, 1);
effort = nan(n_trials, 1);
usat = nan(n_trials, 1);
ssat = nan(n_trials, 1);
succ_like = nan(n_trials, 1);
rms_err = nan(n_trials, 1);
peak_err = nan(n_trials, 1);
end_err = nan(n_trials, 1);
rise_reached = nan(n_trials, 1);
settle_reached = nan(n_trials, 1);
bounded_window = nan(n_trials, 1);

idx = 0;
for it = 1:numel(opts.eval_theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.eval_theta0_deg_set(it));
    for iseed = 1:numel(opts.eval_seeds)
        idx = idx + 1;
        seed = opts.eval_seeds(iseed) + 100 * it;
        try
            out = simulate_case_realistic('PID', sc, cfg, seed, realism);
            [accel(idx), rise(idx), settle(idx), overshoot(idx), peak_rate(idx), effort(idx), ...
                usat(idx), ssat(idx), succ_like(idx), rms_err(idx), peak_err(idx), end_err(idx), ...
                rise_reached(idx), settle_reached(idx), bounded_window(idx)] = ...
                extract_run_metrics(out, cfg, opts);
        catch
            accel(idx) = 0;
            rise(idx) = nan;
            settle(idx) = nan;
            overshoot(idx) = 100;
            peak_rate(idx) = 0;
            effort(idx) = 1;
            usat(idx) = 1;
            ssat(idx) = 1;
            succ_like(idx) = 0;
            rms_err(idx) = 90;
            peak_err(idx) = 90;
            end_err(idx) = 90;
            rise_reached(idx) = 0;
            settle_reached(idx) = 0;
            bounded_window(idx) = 0;
        end
    end
end

M = struct();
M.max_ang_accel_deg_s2 = mean(accel, 'omitnan');
M.rise_time_s = mean(rise, 'omitnan');
M.settling_time_s = mean(settle, 'omitnan');
M.overshoot_pct = mean(overshoot, 'omitnan');
M.peak_att_rate_deg_s = mean(peak_rate, 'omitnan');
M.control_effort_usage = mean(effort, 'omitnan');
M.actuator_sat_frac = mean(usat, 'omitnan');
M.slew_sat_frac = mean(ssat, 'omitnan');
M.success_like_rate = mean(succ_like, 'omitnan');
M.rms_err_deg = mean(rms_err, 'omitnan');
M.peak_err_deg = mean(peak_err, 'omitnan');
M.end_err_deg = mean(end_err, 'omitnan');
M.rise_reached_frac = mean(rise_reached, 'omitnan');
M.settled_frac = mean(settle_reached, 'omitnan');
M.bounded_window_s = mean(bounded_window, 'omitnan');
M.n_trials = n_trials;


function [accel_deg_s2, rise_s, settle_s, overshoot_pct, peak_rate_deg_s, effort_use, ...
    u_sat_frac, slew_sat_frac, success_like, rms_err_deg, peak_err_deg, end_err_deg, ...
    rise_reached, settle_reached, bounded_window_s] = extract_run_metrics(out, cfg, opts)

t = out.time;
dt = cfg.dt;
t0 = opts.cmd_start_s;
amp = deg2rad(opts.cmd_deg);

mask = t >= t0;
theta = out.theta(mask);
q = out.q(mask);
u_cmd = out.u_cmd(mask);
u_act = out.u_act(mask);
t_seg = t(mask);

if isempty(theta)
    accel_deg_s2 = 0;
    rise_s = cfg.t_end_demo;
    settle_s = cfg.t_end_demo;
    overshoot_pct = 0;
    peak_rate_deg_s = 0;
    effort_use = 1;
    u_sat_frac = 1;
    slew_sat_frac = 1;
    success_like = 0;
    rms_err_deg = 90;
    peak_err_deg = 90;
    end_err_deg = 90;
    rise_reached = 0;
    settle_reached = 0;
    bounded_window_s = 0;
    return;
end

ref = amp * ones(size(theta));
err = theta - ref;

fail_idx = find(abs(err) > deg2rad(opts.fail_err_deg) | abs(theta) > deg2rad(opts.fail_theta_deg), 1, 'first');
if isempty(fail_idx)
    iend = numel(theta);
else
    iend = max(1, fail_idx - 1);
end

theta_b = theta(1:iend);
q_b = q(1:iend);
err_b = err(1:iend);
t_b = t_seg(1:iend);
bounded_window_s = t_b(end) - t0;

qdot = diff(q_b) / dt;
accel_deg_s2 = rad2deg(max(abs(qdot), [], 'omitnan'));
peak_rate_deg_s = rad2deg(max(abs(q_b), [], 'omitnan'));

target90 = 0.90 * amp;
ireach = find(theta_b >= target90, 1, 'first');
if isempty(ireach)
    rise_s = nan;
    rise_reached = 0;
else
    rise_s = t_b(ireach) - t0;
    rise_reached = 1;
end

band = deg2rad(max(opts.settle_band_deg, 0.02 * opts.cmd_deg));
[settle_s, settled_ok] = estimate_settle_time_step(t_b, err_b, band, opts.min_settle_window_s);
if ~settled_ok
    settle_s = nan;
    settle_reached = 0;
else
    settle_reached = 1;
end

peak_theta = max(theta_b, [], 'omitnan');
if amp > 0
    overshoot_pct = max(0, 100 * (peak_theta - amp) / max(amp, deg2rad(opts.overshoot_floor_deg)));
else
    overshoot_pct = 0;
end

effort_use = mean(abs(u_cmd) / max(cfg.plant.u_max, 1e-6), 'omitnan');
u_sat_frac = mean(abs(u_cmd) >= 0.99 * cfg.plant.u_max, 'omitnan');
slew_sat_frac = mean(abs(diff(u_act)) / dt >= 0.99 * cfg.plant.slew_max, 'omitnan');

rms_err_deg = min(90, rad2deg(sqrt(mean(err .^ 2, 'omitnan'))));
peak_err_deg = min(90, rad2deg(max(abs(err), [], 'omitnan')));
end_err_deg = min(90, rad2deg(abs(err(end))));
success_like = double(rms_err_deg <= 15 && peak_err_deg <= 50 && end_err_deg <= 15 && settle_reached == 1);


function [ts, ok] = estimate_settle_time_step(t, err, band, min_win_s)
ts = t(end) - t(1);
ok = false;
if isempty(t)
    return;
end

dt = median(diff(t));
win_n = max(1, ceil(min_win_s / max(dt, 1e-6)));
inside = abs(err) <= band;

for k = 1:numel(t)
    tail = inside(k:end);
    if numel(tail) < win_n
        break;
    end
    if all(tail)
        ts = t(k) - t(1);
        ok = true;
        return;
    end
end


function b = estimate_boundary_slew(phaseT, cellT)
b = nan(height(cellT), 1);
for i = 1:height(cellT)
    p = cellT.p_unstable(i);
    g = cellT.max_gimbal_deg(i);
    sub = phaseT(phaseT.p_unstable == p & phaseT.max_gimbal_deg == g, :);
    easy = sub.servo_slew_deg_s(sub.regime_code == 2);
    if isempty(easy)
        b(i) = nan;
    else
        b(i) = min(easy);
    end
end


function agility = build_agility_index(T)
z_acc = robust_z(T.max_ang_accel_deg_s2);
z_rate = robust_z(T.peak_att_rate_deg_s);
rise_idx = T.rise_time_s;
if any(isfinite(rise_idx))
    rise_fill = max(rise_idx(isfinite(rise_idx))) + 0.5;
else
    rise_fill = 5.0;
end
rise_idx(~isfinite(rise_idx)) = rise_fill;
z_rise = robust_z(rise_idx);

over_idx = log1p(max(T.overshoot_pct, 0));
z_over = robust_z(over_idx);
z_settled = robust_z(T.settled_frac);

agility = z_acc + z_rate - z_rise - 0.5 * z_over + 0.75 * z_settled;


function z = robust_z(x)
med = median(x, 'omitnan');
madv = median(abs(x - med), 'omitnan');
if madv < 1e-9
    z = zeros(size(x));
else
    z = (x - med) / (1.4826 * madv);
end


function R = summarize_by_regime(T)
metrics = { ...
    'max_ang_accel_deg_s2', 'rise_time_s', 'settling_time_s', 'settled_frac', 'overshoot_pct', ...
    'peak_att_rate_deg_s', 'control_effort_usage', 'actuator_sat_frac', 'slew_sat_frac', ...
    'agility_index', 'rise_reached_frac', 'settled_frac', 'bounded_window_s'};

regs = ["EASY"; "FRAGILE"; "INFEASIBLE"];
rows = cell(0, 6);
for ir = 1:numel(regs)
    sub = T(string(T.regime_label) == regs(ir), :);
    for im = 1:numel(metrics)
        x = sub.(metrics{im});
        rows(end + 1, :) = {regs(ir), string(metrics{im}), ...
            median(x, 'omitnan'), quantile(x, 0.25), quantile(x, 0.75), height(sub)}; %#ok<AGROW>
    end
end

R = cell2table(rows, 'VariableNames', ...
    {'regime_label', 'metric', 'median', 'q25', 'q75', 'n_cells'});


function S = build_audit_stats(T)
rows = cell(0, 4);

metrics = { ...
    'max_ang_accel_deg_s2', 'rise_time_s', 'settling_time_s', 'settled_frac', 'overshoot_pct', ...
    'peak_att_rate_deg_s', 'control_effort_usage', 'actuator_sat_frac', 'slew_sat_frac', ...
    'agility_index'};

for i = 1:numel(metrics)
    m = metrics{i};
    [rhoR, pR] = spearman_simple(T.robustness, T.(m));
    rows(end + 1, :) = {sprintf('spearman_robustness__%s', m), rhoR, pR, height(T)}; %#ok<AGROW>

    valid = isfinite(T.slew_margin_to_boundary);
    [rhoB, pB] = spearman_simple(T.slew_margin_to_boundary(valid), T.(m)(valid));
    rows(end + 1, :) = {sprintf('spearman_boundary_margin__%s', m), rhoB, pB, sum(valid)}; %#ok<AGROW>
end

% Threshold-artifact check: does continuous agility vary within same envelope level?
if exist(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'exp2_maneuverability_cells.csv'), 'file')
    E = readtable(fullfile(fileparts(mfilename('fullpath')), 'results', 'exp2_maneuverability_cells.csv'));
    J = innerjoin(T, E(:, {'cell_id', 'envelope_cmd_deg'}), 'Keys', 'cell_id');
    levels = unique(J.envelope_cmd_deg);
    within_var = 0;
    total_n = 0;
    for k = 1:numel(levels)
        x = J.agility_index(J.envelope_cmd_deg == levels(k));
        if numel(x) >= 3
            within_var = within_var + var(x, 1, 'omitnan') * numel(x);
            total_n = total_n + numel(x);
        end
    end
    if total_n > 0
        within_var = within_var / total_n;
    else
        within_var = nan;
    end
    total_var = var(J.agility_index, 1, 'omitnan');
    frac_within = within_var / max(total_var, 1e-12);

    [rhoEnv, pEnv] = spearman_simple(J.envelope_cmd_deg, J.agility_index);
    rows(end + 1, :) = {'artifact_check_within_group_var_frac_agility', frac_within, nan, height(J)}; %#ok<AGROW>
    rows(end + 1, :) = {'artifact_check_spearman_envelope_vs_agility', rhoEnv, pEnv, height(J)}; %#ok<AGROW>

    same_env_step = J.envelope_cmd_deg == 10;
    if any(same_env_step)
        x = J.agility_index(same_env_step);
        rows(end + 1, :) = {'artifact_check_iqr_agility_at_env10', iqr(x), nan, numel(x)}; %#ok<AGROW>
    end
end

% Tradeoff check: top agility distribution and Pareto density by regime.
q75 = quantile(T.agility_index, 0.75);
isTop = isfinite(T.agility_index) & T.agility_index >= q75;
for rg = ["EASY"; "FRAGILE"; "INFEASIBLE"]'
    if any(isTop)
        fracTop = mean(string(T.regime_label(isTop)) == rg);
    else
        fracTop = nan;
    end
    fracAll = mean(string(T.regime_label) == rg);
    enrich = fracTop / max(fracAll, 1e-12);
    rows(end + 1, :) = {sprintf('top_agility_enrichment__%s', rg), enrich, nan, sum(isTop)}; %#ok<AGROW>
end

validPareto = isfinite(T.robustness) & isfinite(T.agility_index);
isPareto = false(height(T), 1);
if any(validPareto)
    isPareto(validPareto) = pareto_mask(T.robustness(validPareto), T.agility_index(validPareto));
end
for rg = ["EASY"; "FRAGILE"; "INFEASIBLE"]'
    fracPareto = mean(string(T.regime_label(isPareto)) == rg);
    rows(end + 1, :) = {sprintf('pareto_regime_fraction__%s', rg), fracPareto, nan, sum(isPareto)}; %#ok<AGROW>
end
rows(end + 1, :) = {'pareto_count', sum(isPareto), nan, height(T)}; %#ok<AGROW>

S = cell2table(rows, 'VariableNames', {'stat_name', 'value', 'p_value', 'n'});


function [rho, pval] = spearman_simple(x, y)
mask = isfinite(x) & isfinite(y);
x = x(mask);
y = y(mask);
n = numel(x);
if n < 4
    rho = nan;
    pval = nan;
    return;
end

rx = rank_ties(x);
ry = rank_ties(y);
C = corrcoef(rx, ry);
rho = C(1,2);

t = rho * sqrt((n - 2) / max(1e-12, 1 - rho^2));
pval = 2 * tcdf(-abs(t), n - 2);


function r = rank_ties(x)
[xs, ord] = sort(x);
r = zeros(size(x));
i = 1;
while i <= numel(x)
    j = i;
    while j < numel(x) && xs(j + 1) == xs(i)
        j = j + 1;
    end
    rr = (i + j) / 2;
    r(ord(i:j)) = rr;
    i = j + 1;
end


function isPareto = pareto_mask(x, y)
n = numel(x);
isPareto = true(n, 1);
for i = 1:n
    for j = 1:n
        if j == i
            continue;
        end
        dominates = (x(j) >= x(i) && y(j) >= y(i)) && (x(j) > x(i) || y(j) > y(i));
        if dominates
            isPareto(i) = false;
            break;
        end
    end
end


function plot_metric_vs_robustness(T, out_png)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1400 850]);
tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

plot_one(nexttile, T.robustness, T.max_ang_accel_deg_s2, 'Robustness', 'Max angular accel (deg/s^2)');
plot_one(nexttile, T.robustness, T.peak_att_rate_deg_s, 'Robustness', 'Peak attitude rate (deg/s)');
plot_one(nexttile, T.robustness, T.rise_time_s, 'Robustness', 'Rise time (s)');
plot_one(nexttile, T.robustness, T.settling_time_s, 'Robustness', 'Settling time (s)');
plot_one(nexttile, T.robustness, T.overshoot_pct, 'Robustness', 'Overshoot (%)');
plot_one(nexttile, T.robustness, T.agility_index, 'Robustness', 'Agility index (z-composite)');

sgtitle('Exp2 Physical Audit: Continuous Maneuverability vs Robustness');
exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);


function plot_one(ax, x, y, xlab, ylab)
axes(ax); %#ok<LAXES>
scatter(x, y, 28, 'filled', 'MarkerFaceAlpha', 0.75);
valid = isfinite(x) & isfinite(y);
if ~any(valid)
    return;
end
scatter(x(valid), y(valid), 28, 'filled', 'MarkerFaceAlpha', 0.75);
grid on;
xlabel(xlab);
ylabel(ylab);
hold on;
p = polyfit(x(valid), y(valid), 1);
xx = linspace(min(x(valid)), max(x(valid)), 100);
yy = polyval(p, xx);
plot(xx, yy, 'k-', 'LineWidth', 1.4);


function plot_regime_distributions(T, out_png)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1400 850]);
tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

box_by_regime(nexttile, T, 'max_ang_accel_deg_s2', 'Max angular accel (deg/s^2)');
box_by_regime(nexttile, T, 'peak_att_rate_deg_s', 'Peak attitude rate (deg/s)');
box_by_regime(nexttile, T, 'rise_time_s', 'Rise time (s)');
box_by_regime(nexttile, T, 'settling_time_s', 'Settling time (s)');
box_by_regime(nexttile, T, 'overshoot_pct', 'Overshoot (%)');
box_by_regime(nexttile, T, 'agility_index', 'Agility index');

sgtitle('Exp2 Physical Audit: Maneuverability by Exp1 Regime');
exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);


function box_by_regime(ax, T, varName, ylab)
axes(ax); %#ok<LAXES>
rg = categorical(string(T.regime_label), ["INFEASIBLE", "FRAGILE", "EASY"]);
boxchart(rg, T.(varName));
grid on;
xlabel('Regime');
ylabel(ylab);


function plot_pareto(T, out_png)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 900 620]);
hold on;

rg = string(T.regime_label);
clr = zeros(height(T), 3);
clr(rg == "INFEASIBLE", :) = repmat([0.85 0.30 0.25], sum(rg == "INFEASIBLE"), 1);
clr(rg == "FRAGILE", :) = repmat([0.95 0.82 0.35], sum(rg == "FRAGILE"), 1);
clr(rg == "EASY", :) = repmat([0.40 0.75 0.45], sum(rg == "EASY"), 1);

scatter(T.robustness, T.agility_index, 36, clr, 'filled', 'MarkerFaceAlpha', 0.85);

isPareto = pareto_mask(T.robustness, T.agility_index);
scatter(T.robustness(isPareto), T.agility_index(isPareto), 70, 'ko', 'LineWidth', 1.4);

grid on;
xlabel('Robustness score');
ylabel('Agility index');
title('Exp2 Physical Audit: Robustness-Agility Pareto Front');
legend({'Cells', 'Pareto front'}, 'Location', 'best');

exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);