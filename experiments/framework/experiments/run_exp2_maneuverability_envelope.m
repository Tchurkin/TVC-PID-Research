function [cellT, trialT, regimeSummaryT] = run_exp2_maneuverability_envelope()
%RUN_EXP2_MANEUVERABILITY_ENVELOPE
% Experiment 2: map maneuverability envelope on top of locked Exp1 cells.

this_dir = fileparts(mfilename('fullpath'));
framework_dir = fileparts(this_dir);
exp_dir = fileparts(framework_dir);
proj_dir = fileparts(exp_dir);

addpath(fullfile(proj_dir, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(framework_dir, 'plant'));
addpath(fullfile(framework_dir, 'controllers'));

P = default_rocket_params();
results_dir = fullfile(exp_dir, 'results');
graphs_dir = fullfile(results_dir, 'graphs');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'); mkdir(graphs_dir); end

phase_path = fullfile(results_dir, 'phase_diagram.csv');
if ~exist(phase_path, 'file')
    error('run_exp2_maneuverability_envelope:missingExp1', ...
        'Missing %s. Run run_locked_exp1 first.', phase_path);
end

phaseT = readtable(phase_path);

opts = struct();
opts.cmd_grid_deg = [5 10 15 20 25 30];
opts.eval_seeds = P.analysis.seeds;
opts.eval_theta0_deg_set = P.analysis.theta0_deg_set;
opts.envelope_success_gate = 0.50;
opts.envelope_rms_err_gate_deg = 15.0;
opts.envelope_peak_err_gate_deg = 35.0;
opts.reference_cmd_deg = 20.0;
opts.cmd_start_s = 0.55;
opts.cmd_ramp_s = 0.75;
opts.cmd_hold_s = 0.85;
opts.t_end = max(P.analysis.t_end, opts.cmd_start_s + 2 * opts.cmd_ramp_s + opts.cmd_hold_s + 0.50);
opts.tune_Kp_grid = [10 20 30 45 60];
opts.tune_Kd_grid = [4 8 12 16 24];

fprintf('=== EXP2: MANEUVERABILITY ENVELOPE ===\n');
fprintf('Loaded Exp1 cells: %d\n', height(phaseT));
fprintf('Command sweep (deg): %s\n', mat2str(opts.cmd_grid_deg));

trial_rows = cell(0, 19);
cell_rows = cell(0, 17);

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
    [Kp_use, Kd_use] = tune_maneuver_pid(cfg, sc, realism, opts, r.best_Kp, r.best_Kd);
    cfg = configure_pid_controller(cfg, Kp_use, Kd_use);

    cmd_success = zeros(numel(opts.cmd_grid_deg), 1);
    cmd_rms_err = zeros(numel(opts.cmd_grid_deg), 1);
    cmd_peak_err = zeros(numel(opts.cmd_grid_deg), 1);
    cmd_end_err = zeros(numel(opts.cmd_grid_deg), 1);
    cmd_u_sat = zeros(numel(opts.cmd_grid_deg), 1);
    cmd_slew_sat = zeros(numel(opts.cmd_grid_deg), 1);

    for ic = 1:numel(opts.cmd_grid_deg)
        cmd_deg = opts.cmd_grid_deg(ic);
        sc_cmd = apply_pitch_program(scfg(sc), cmd_deg, opts);

        m = evaluate_maneuver_cell(cfg, sc_cmd, realism, opts);
        cmd_success(ic) = m.success_rate;
        cmd_rms_err(ic) = m.rms_error_deg;
        cmd_peak_err(ic) = m.peak_error_deg;
        cmd_end_err(ic) = m.end_error_deg;
        cmd_u_sat(ic) = m.u_cmd_sat_frac;
        cmd_slew_sat(ic) = m.slew_sat_frac;

        trial_rows(end + 1, :) = { ...
            i, string(r.regime_label{1}), r.regime_code, ...
            r.p_unstable, r.servo_slew_deg_s, r.max_gimbal_deg, r.best_u_max_frac, ...
            Kp_use, Kd_use, cmd_deg, ...
            m.success_rate, m.rms_error_deg, m.peak_error_deg, m.end_error_deg, ...
            m.max_theta_deg, m.u_cmd_sat_frac, m.slew_sat_frac, ...
            m.settling_time_s, m.n_trials}; %#ok<AGROW>
    end

    pass_mask = cmd_success >= opts.envelope_success_gate & ...
        cmd_rms_err <= opts.envelope_rms_err_gate_deg & ...
        cmd_peak_err <= opts.envelope_peak_err_gate_deg;

    if any(pass_mask)
        envelope_cmd_deg = max(opts.cmd_grid_deg(pass_mask));
    else
        envelope_cmd_deg = 0;
    end

    [~, i20] = min(abs(opts.cmd_grid_deg - 20));
    pass_at_20 = pass_mask(i20);

    cell_rows(end + 1, :) = { ...
        i, string(r.regime_label{1}), r.regime_code, ...
        r.p_unstable, r.servo_slew_deg_s, r.max_gimbal_deg, r.best_u_max_frac, ...
        Kp_use, Kd_use, ...
        envelope_cmd_deg, envelope_cmd_deg / max(opts.cmd_grid_deg), ...
        opts.cmd_grid_deg(i20), pass_at_20, ...
        cmd_success(i20), cmd_rms_err(i20), cmd_peak_err(i20), cmd_slew_sat(i20)}; %#ok<AGROW>

    fprintf(['  cell %3d/%3d | %s p=%g slew=%g gimbal=%g -> env=%g deg, ' ...
        'pass@20=%d (succ=%.2f)\n'], ...
        i, height(phaseT), r.regime_label{1}, r.p_unstable, r.servo_slew_deg_s, r.max_gimbal_deg, ...
        envelope_cmd_deg, pass_at_20, cmd_success(i20));
end

trialT = cell2table(trial_rows, 'VariableNames', { ...
    'cell_id', 'regime_label', 'regime_code', ...
    'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'u_max_frac', ...
    'Kp', 'Kd', 'cmd_deg', ...
    'success_rate', 'rms_error_deg', 'peak_error_deg', 'end_error_deg', ...
    'max_theta_deg', 'u_cmd_sat_frac', 'slew_sat_frac', ...
    'settling_time_s', 'n_trials'});

cellT = cell2table(cell_rows, 'VariableNames', { ...
    'cell_id', 'regime_label', 'regime_code', ...
    'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'u_max_frac', ...
    'Kp', 'Kd', ...
    'envelope_cmd_deg', 'envelope_cmd_ratio', ...
    'reference_cmd_deg', 'pass_at_reference_cmd', ...
    'success_at_reference_cmd', 'rms_err_at_reference_cmd_deg', ...
    'peak_err_at_reference_cmd_deg', 'slew_sat_at_reference_cmd'});

regimeSummaryT = summarize_regime_envelope(cellT);

trial_csv = fullfile(results_dir, 'exp2_maneuverability_trials.csv');
cell_csv = fullfile(results_dir, 'exp2_maneuverability_cells.csv');
regime_csv = fullfile(results_dir, 'exp2_regime_envelope_summary.csv');

writetable(trialT, trial_csv);
writetable(cellT, cell_csv);
writetable(regimeSummaryT, regime_csv);

unified_png = fullfile(graphs_dir, 'exp2_regime_and_maneuverability_unified.png');
frontier_png = fullfile(graphs_dir, 'exp2_required_slew_for_20deg_cmd.png');
plot_exp2_unified(phaseT, cellT, unified_png);
plot_required_slew_for_reference(cellT, frontier_png);

fprintf('\nSaved: %s\n', trial_csv);
fprintf('Saved: %s\n', cell_csv);
fprintf('Saved: %s\n', regime_csv);
fprintf('Saved: %s\n', unified_png);
fprintf('Saved: %s\n', frontier_png);


function sc2 = scfg(sc)
sc2 = sc;


function [best_Kp, best_Kd] = tune_maneuver_pid(cfg, sc, realism, opts, Kp0, Kd0)
best_Kp = Kp0;
best_Kd = Kd0;
best_success = -inf;
best_rms = inf;

for i = 1:numel(opts.tune_Kp_grid)
    for j = 1:numel(opts.tune_Kd_grid)
        Kp_try = opts.tune_Kp_grid(i);
        Kd_try = opts.tune_Kd_grid(j);
        cfg_try = configure_pid_controller(cfg, Kp_try, Kd_try);
        sc_try = apply_pitch_program(scfg(sc), opts.reference_cmd_deg, opts);
        m = evaluate_maneuver_cell(cfg_try, sc_try, realism, opts);

        better = m.success_rate > best_success || ...
            (abs(m.success_rate - best_success) < 1e-9 && m.rms_error_deg < best_rms);
        if better
            best_success = m.success_rate;
            best_rms = m.rms_error_deg;
            best_Kp = Kp_try;
            best_Kd = Kd_try;
        end
    end
end


function sc = apply_pitch_program(sc, cmd_deg, opts)
sc.kind = "PITCH_PROGRAM_EXP2";
sc.t_end = opts.t_end;
sc.theta_ref_fun = @(t) pitch_ref_rad(t, cmd_deg, opts);
sc.q_ref_fun = @(t) pitch_rate_ref_rad_s(t, cmd_deg, opts);


function theta = pitch_ref_rad(t, cmd_deg, opts)
amp = deg2rad(cmd_deg);
t0 = opts.cmd_start_s;
t1 = t0 + opts.cmd_ramp_s;
t2 = t1 + opts.cmd_hold_s;
t3 = t2 + opts.cmd_ramp_s;
if t < t0
    theta = 0;
elseif t < t1
    theta = amp * (t - t0) / opts.cmd_ramp_s;
elseif t < t2
    theta = amp;
elseif t < t3
    theta = amp * (1 - (t - t2) / opts.cmd_ramp_s);
else
    theta = 0;
end


function q_ref = pitch_rate_ref_rad_s(t, cmd_deg, opts)
amp = deg2rad(cmd_deg);
t0 = opts.cmd_start_s;
t1 = t0 + opts.cmd_ramp_s;
t2 = t1 + opts.cmd_hold_s;
t3 = t2 + opts.cmd_ramp_s;
if t >= t0 && t < t1
    q_ref = amp / opts.cmd_ramp_s;
elseif t >= t2 && t < t3
    q_ref = -amp / opts.cmd_ramp_s;
else
    q_ref = 0;
end


function metrics = evaluate_maneuver_cell(cfg, sc, realism, opts)
n_trials = numel(opts.eval_theta0_deg_set) * numel(opts.eval_seeds);
success_col = false(n_trials, 1);
rms_col = zeros(n_trials, 1);
peak_col = zeros(n_trials, 1);
end_col = zeros(n_trials, 1);
max_theta_col = zeros(n_trials, 1);
u_sat_col = zeros(n_trials, 1);
slew_sat_col = zeros(n_trials, 1);
settling_col = zeros(n_trials, 1);

idx = 0;
for it = 1:numel(opts.eval_theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.eval_theta0_deg_set(it));
    for iseed = 1:numel(opts.eval_seeds)
        idx = idx + 1;
        seed = opts.eval_seeds(iseed) + 100 * it;
        try
            out = simulate_case_realistic('PID', sc, cfg, seed, realism);
            [success_col(idx), rms_col(idx), peak_col(idx), end_col(idx), max_theta_col(idx), ...
                u_sat_col(idx), slew_sat_col(idx), settling_col(idx)] = score_maneuver_run(out, cfg, opts);
        catch
            success_col(idx) = false;
            rms_col(idx) = 90;
            peak_col(idx) = 90;
            end_col(idx) = 90;
            max_theta_col(idx) = 90;
            u_sat_col(idx) = 1;
            slew_sat_col(idx) = 1;
            settling_col(idx) = cfg.t_end_demo;
        end
    end
end

metrics = struct();
metrics.success_rate = mean(success_col);
metrics.rms_error_deg = mean(rms_col);
metrics.peak_error_deg = mean(peak_col);
metrics.end_error_deg = mean(end_col);
metrics.max_theta_deg = mean(max_theta_col);
metrics.u_cmd_sat_frac = mean(u_sat_col);
metrics.slew_sat_frac = mean(slew_sat_col);
metrics.settling_time_s = mean(settling_col);
metrics.n_trials = n_trials;


function [success, rms_err_deg, peak_err_deg, end_err_deg, max_theta_deg, u_sat, slew_sat, t_settle] = score_maneuver_run(out, cfg, opts)
if ~all(isfinite(out.theta)) || ~all(isfinite(out.theta_ref))
    success = false;
    rms_err_deg = 90;
    peak_err_deg = 90;
    end_err_deg = 90;
    max_theta_deg = 90;
    u_sat = 1;
    slew_sat = 1;
    t_settle = cfg.t_end_demo;
    return;
end

err = out.theta - out.theta_ref;
rms_err_deg = min(90, rad2deg(sqrt(mean(err .^ 2))));
peak_err_deg = min(90, rad2deg(max(abs(err))));
end_err_deg = min(90, rad2deg(abs(err(end))));
max_theta_deg = min(90, rad2deg(max(abs(out.theta))));
u_sat = mean(abs(out.u_cmd) >= 0.99 * cfg.plant.u_max);
slew_sat = mean(abs(diff(out.u_act)) / cfg.dt >= 0.99 * cfg.plant.slew_max);
t_settle = estimate_settling_time_tracking(out.time, err, deg2rad(2.0));

success = rms_err_deg <= opts.envelope_rms_err_gate_deg && ...
    peak_err_deg <= opts.envelope_peak_err_gate_deg;


function t_settle = estimate_settling_time_tracking(t, err, band_rad)
t_settle = t(end);
inside = abs(err) <= band_rad;
for k = 1:numel(t)
    if all(inside(k:end))
        t_settle = t(k);
        return;
    end
end


function S = summarize_regime_envelope(cellT)
regimes = unique(string(cellT.regime_label));
rows = cell(0, 9);
for i = 1:numel(regimes)
    mask = string(cellT.regime_label) == regimes(i);
    sub = cellT(mask, :);
    rows(end + 1, :) = { ...
        regimes(i), ...
        median(sub.envelope_cmd_deg), ...
        mean(sub.envelope_cmd_deg), ...
        min(sub.envelope_cmd_deg), ...
        max(sub.envelope_cmd_deg), ...
        mean(sub.pass_at_reference_cmd), ...
        median(sub.success_at_reference_cmd), ...
        median(sub.rms_err_at_reference_cmd_deg), ...
        height(sub)};
end

S = cell2table(rows, 'VariableNames', { ...
    'regime_label', ...
    'median_envelope_cmd_deg', 'mean_envelope_cmd_deg', ...
    'min_envelope_cmd_deg', 'max_envelope_cmd_deg', ...
    'frac_pass_reference_cmd', 'median_success_reference_cmd', ...
    'median_rms_err_reference_cmd_deg', 'n_cells'});


function plot_exp2_unified(phaseT, cellT, out_png)
p_vals = unique(phaseT.p_unstable)';
slew_vals = unique(phaseT.servo_slew_deg_s)';
gimbal_vals = unique(phaseT.max_gimbal_deg)';

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1500 780]);
tiledlayout(2, numel(gimbal_vals), 'Padding', 'compact', 'TileSpacing', 'compact');

cmap_regime = [0.85 0.30 0.25; 0.95 0.82 0.35; 0.40 0.75 0.45];

for ig = 1:numel(gimbal_vals)
    g = gimbal_vals(ig);

    R = nan(numel(slew_vals), numel(p_vals));
    E = nan(numel(slew_vals), numel(p_vals));

    for is = 1:numel(slew_vals)
        for ip = 1:numel(p_vals)
            m1 = phaseT.max_gimbal_deg == g & phaseT.servo_slew_deg_s == slew_vals(is) & phaseT.p_unstable == p_vals(ip);
            if any(m1)
                R(is, ip) = phaseT.regime_code(find(m1, 1));
            end

            m2 = cellT.max_gimbal_deg == g & cellT.servo_slew_deg_s == slew_vals(is) & cellT.p_unstable == p_vals(ip);
            if any(m2)
                E(is, ip) = cellT.envelope_cmd_deg(find(m2, 1));
            end
        end
    end

    nexttile;
    imagesc(p_vals, slew_vals, R, [0 2]);
    axis xy;
    grid on;
    colormap(gca, cmap_regime);
    cb = colorbar;
    cb.Ticks = [0 1 2];
    cb.TickLabels = {'INF', 'FRAG', 'EASY'};
    title(sprintf('Exp1 regime (gimbal %.0f deg)', g));
    xlabel('p (1/s)');
    ylabel('slew (deg/s)');

    nexttile;
    imagesc(p_vals, slew_vals, E);
    axis xy;
    grid on;
    colormap(gca, turbo(256));
    cb2 = colorbar;
    cb2.Label.String = 'Max safe cmd (deg)';
    title(sprintf('Exp2 envelope (gimbal %.0f deg)', g));
    xlabel('p (1/s)');
    ylabel('slew (deg/s)');

    for is = 1:numel(slew_vals)
        for ip = 1:numel(p_vals)
            if isfinite(E(is, ip))
                text(p_vals(ip), slew_vals(is), sprintf('%.0f', E(is, ip)), ...
                    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'k', 'FontSize', 8);
            end
        end
    end
end

sgtitle('Unified Exp1+Exp2: Regime Topology and Maneuverability Envelope');
exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);


function plot_required_slew_for_reference(cellT, out_png)
ref_cmd = mode(cellT.reference_cmd_deg);
p_vals = unique(cellT.p_unstable)';
g_vals = unique(cellT.max_gimbal_deg)';

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 960 560]);
hold on;
clr = lines(numel(g_vals));

for ig = 1:numel(g_vals)
    g = g_vals(ig);
    y = nan(size(p_vals));
    for ip = 1:numel(p_vals)
        sub = cellT(cellT.max_gimbal_deg == g & cellT.p_unstable == p_vals(ip), :);
        if isempty(sub)
            continue;
        end
        sub = sortrows(sub, 'servo_slew_deg_s', 'ascend');
        idx = find(sub.pass_at_reference_cmd, 1, 'first');
        if ~isempty(idx)
            y(ip) = sub.servo_slew_deg_s(idx);
        end
    end
    plot(p_vals, y, 'o-', 'LineWidth', 1.8, 'Color', clr(ig, :), ...
        'DisplayName', sprintf('gimbal %.0f deg', g));
end

grid on;
xlabel('Instability p (1/s)');
ylabel(sprintf('Required slew for %.0f deg command (deg/s)', ref_cmd));
title('Exp2 Reference Maneuver Frontier');
legend('Location', 'northwest');
exportgraphics(fig, out_png, 'Resolution', 240);
close(fig);