function [rawT, thresholdT, transitionT, designRuleT] = run_exp5_threshold_discovery()
%RUN_EXP5_THRESHOLD_DISCOVERY Legacy wrapper retained for compatibility.
%
% Exp5 is now a local design-sensitivity module. This wrapper forwards to
% the new isolated Exp5 runner so older entry points still execute safely.

[cellT, sensT, regimeT] = run_exp5_design_sensitivity();
rawT = cellT;
thresholdT = sensT;
transitionT = regimeT;
designRuleT = table();
return;

this_dir = fileparts(mfilename('fullpath'));
proj_dir = fileparts(this_dir);
framework_dir = fullfile(this_dir, 'framework');
results_dir = fullfile(this_dir, 'results');
graphs_dir = fullfile(results_dir, 'graphs');

addpath(fullfile(proj_dir, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(framework_dir, 'plant'));
addpath(fullfile(framework_dir, 'controllers'));

if ~exist(results_dir, 'dir'), mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'), mkdir(graphs_dir); end

phase_path = fullfile(results_dir, 'phase_diagram.csv');
if ~exist(phase_path, 'file')
    error('run_exp5_threshold_discovery:missingExp1', ...
        'Missing %s. Run run_locked_exp1 first.', phase_path);
end

phaseT = readtable(phase_path);
exp2_cell_path = fullfile(results_dir, 'exp2_maneuverability_cells.csv');
if exist(exp2_cell_path, 'file')
    exp2CellT = readtable(exp2_cell_path);
else
    exp2CellT = table();
end

P = default_rocket_params();
opts = build_exp5_options(P);
cases = select_representative_cases(phaseT, exp2CellT, P);
params = build_parameter_sweeps();

fprintf('=== EXP5: PARAMETER SUFFICIENCY AND BOTTLENECK THRESHOLDS ===\n');
fprintf('Representative cases: %d | Parameters: %d\n', numel(cases), numel(params));
for ic = 1:numel(cases)
    fprintf('  %s -> source=%s p=%g slew=%g gimbal=%g Kp=%g Kd=%g\n', ...
        cases(ic).case_label, cases(ic).source_regime, cases(ic).p_unstable, ...
        cases(ic).servo_slew_deg_s, cases(ic).max_gimbal_deg, cases(ic).Kp, cases(ic).Kd);
end

raw_rows = cell(0, 27);
for ic = 1:numel(cases)
    base_case = cases(ic);
    fprintf('\n--- %s ---\n', base_case.case_label);
    for ip = 1:numel(params)
        param = params(ip);
        fprintf('  Sweeping %-18s values=%s\n', param.name, mat2str(param.values));
        for iv = 1:numel(param.values)
            sweep_value = param.values(iv);
            applied = apply_sweep_value(base_case, param.name, sweep_value, P);
            [cfg, sc, realism] = build_case_cfg(P, applied, opts);
            metrics = evaluate_exp5_point(cfg, sc, realism, opts);
            bottleneck = classify_active_bottleneck(param.name, metrics);

            raw_rows(end + 1, :) = { ...
                base_case.case_id, base_case.case_label, base_case.source_regime, ...
                base_case.p_unstable, base_case.servo_slew_deg_s, base_case.max_gimbal_deg, ...
                base_case.Kp, base_case.Kd, ...
                param.name, sweep_value, ...
                applied.max_gimbal_deg, applied.servo_slew_deg_s, applied.Kp, applied.Kd, ...
                metrics.envelope_cmd_deg, metrics.envelope_cmd_ratio, ...
                metrics.bounded_window_s, metrics.bounded_window_norm, ...
                metrics.overshoot_pct, metrics.actuator_sat_frac, metrics.slew_sat_frac, ...
                metrics.success_like_rate, metrics.rms_err_deg, metrics.peak_err_deg, ...
                metrics.reference_success_rate, metrics.settled_frac, bottleneck}; %#ok<AGROW>

            fprintf(['    %-18s=%-8g | env=%4.1f deg bw=%.2f ov=%.1f ' ...
                'usat=%.2f ssat=%.2f -> %s\n'], ...
                param.name, sweep_value, metrics.envelope_cmd_deg, metrics.bounded_window_norm, ...
                metrics.overshoot_pct, metrics.actuator_sat_frac, metrics.slew_sat_frac, bottleneck);
        end
    end
end

rawT = cell2table(raw_rows, 'VariableNames', { ...
    'case_id', 'case_label', 'source_regime', ...
    'base_p_unstable', 'base_servo_slew_deg_s', 'base_max_gimbal_deg', 'base_Kp', 'base_Kd', ...
    'parameter', 'param_value', ...
    'applied_max_gimbal_deg', 'applied_servo_slew_deg_s', 'applied_Kp', 'applied_Kd', ...
    'envelope_cmd_deg', 'envelope_cmd_ratio', ...
    'bounded_window_s', 'bounded_window_norm', ...
    'overshoot_pct', 'actuator_sat_frac', 'slew_sat_frac', ...
    'success_like_rate', 'rms_err_deg', 'peak_err_deg', ...
    'reference_success_rate', 'settled_frac', 'active_bottleneck'});
rawT = sortrows(rawT, {'case_id', 'parameter', 'param_value'});

thresholdT = build_threshold_summary(rawT, cases, params, opts);
transitionT = build_bottleneck_transitions(rawT);
designRuleT = build_design_rules(thresholdT, transitionT, params);

raw_csv = fullfile(results_dir, 'exp5_raw_results.csv');
threshold_csv = fullfile(results_dir, 'exp5_threshold_summary.csv');
transition_csv = fullfile(results_dir, 'exp5_bottleneck_transitions.csv');
rules_csv = fullfile(results_dir, 'exp5_design_rules.csv');

writetable(rawT, raw_csv);
writetable(thresholdT, threshold_csv);
writetable(transitionT, transition_csv);
writetable(designRuleT, rules_csv);

plot_parameter_dashboard(rawT, thresholdT, cases, 'max_gimbal_deg', ...
    fullfile(graphs_dir, 'exp5_fig1_gimbal_authority_diminishing_returns.png'), ...
    'Gimbal Authority Diminishing Returns');
plot_parameter_dashboard(rawT, thresholdT, cases, 'servo_slew_deg_s', ...
    fullfile(graphs_dir, 'exp5_fig2_servo_slew_diminishing_returns.png'), ...
    'Servo Slew Diminishing Returns');
plot_parameter_dashboard(rawT, thresholdT, cases, 'Kp', ...
    fullfile(graphs_dir, 'exp5_fig3_kp_performance_curve.png'), ...
    'Kp Performance Curve');
plot_parameter_dashboard(rawT, thresholdT, cases, 'Kd', ...
    fullfile(graphs_dir, 'exp5_fig4_kd_performance_curve.png'), ...
    'Kd Performance Curve');
plot_bottleneck_transition_panels(rawT, cases, params, ...
    fullfile(graphs_dir, 'exp5_fig5_bottleneck_transition_plots.png'));
plot_case_studies(rawT, thresholdT, cases, params, ...
    fullfile(graphs_dir, 'exp5_fig6_three_representative_case_studies.png'));

fprintf('\nSaved: %s\n', raw_csv);
fprintf('Saved: %s\n', threshold_csv);
fprintf('Saved: %s\n', transition_csv);
fprintf('Saved: %s\n', rules_csv);
fprintf('Saved Exp5 figures under: %s\n', graphs_dir);
fprintf('Exp5 complete.\n');
end


function opts = build_exp5_options(P)
opts.seeds = P.analysis.seeds;
opts.theta0_deg_set = P.analysis.theta0_deg_set;

opts.reference_cmd_deg = 20.0;
opts.step_cmd_start_s = 0.50;
opts.step_t_end = 3.0;
opts.settle_band_deg = 2.0;
opts.min_settle_window_s = 0.20;
opts.fail_err_deg = 30.0;
opts.fail_theta_deg = 60.0;
opts.overshoot_floor_deg = 1.0;

opts.cmd_grid_deg = [5 10 15 20 25 30];
opts.envelope_success_gate = 0.50;
opts.envelope_rms_err_gate_deg = 15.0;
opts.envelope_peak_err_gate_deg = 35.0;
opts.envelope_cmd_start_s = 0.55;
opts.envelope_cmd_ramp_s = 0.75;
opts.envelope_cmd_hold_s = 0.85;
opts.envelope_t_end = max(P.analysis.t_end, ...
    opts.envelope_cmd_start_s + 2 * opts.envelope_cmd_ramp_s + opts.envelope_cmd_hold_s + 0.50);

opts.failure_frac = 0.05;
opts.practical_frac = 0.90;
opts.diminishing_return_frac = 0.05;
end


function cases = select_representative_cases(phaseT, exp2CellT, P)
specs = struct( ...
    'case_id', {'easy_regime', 'fragile_regime', 'hard_regime'}, ...
    'case_label', {'Easy regime', 'Fragile regime', 'Hard regime'}, ...
    'regime_code', {2, 1, 0});

cases = repmat(struct( ...
    'case_id', '', 'case_label', '', 'source_regime', '', 'p_unstable', 0, ...
    'servo_slew_deg_s', 0, 'max_gimbal_deg', 0, 'Kp', 0, 'Kd', 0, ...
    'control_effectiveness', P.rocket.control_effectiveness), 1, numel(specs));

for i = 1:numel(specs)
    sub = phaseT(phaseT.regime_code == specs(i).regime_code, :);
    if isempty(sub)
        if specs(i).regime_code == 0
            sub = phaseT;
        else
            error('run_exp5_threshold_discovery:missingRegime', ...
                'Could not find representative cell for regime code %d.', specs(i).regime_code);
        end
    end

    pick = choose_median_cell(sub);
    Kp_use = double(pick.best_Kp);
    Kd_use = double(pick.best_Kd);

    if ~isempty(exp2CellT)
        mask = exp2CellT.p_unstable == pick.p_unstable & ...
            exp2CellT.servo_slew_deg_s == pick.servo_slew_deg_s & ...
            exp2CellT.max_gimbal_deg == pick.max_gimbal_deg;
        idx = find(mask, 1, 'first');
        if ~isempty(idx)
            Kp_use = exp2CellT.Kp(idx);
            Kd_use = exp2CellT.Kd(idx);
        end
    end

    cases(i).case_id = specs(i).case_id;
    cases(i).case_label = specs(i).case_label;
    cases(i).source_regime = char(string(pick.regime_label(1)));
    cases(i).p_unstable = double(pick.p_unstable(1));
    cases(i).servo_slew_deg_s = double(pick.servo_slew_deg_s(1));
    cases(i).max_gimbal_deg = double(pick.max_gimbal_deg(1));
    cases(i).Kp = Kp_use;
    cases(i).Kd = Kd_use;
end
end


function row = choose_median_cell(T)
X = [double(T.p_unstable), double(T.servo_slew_deg_s), double(T.max_gimbal_deg)];
center = median(X, 1, 'omitnan');
scale = max(1e-6, max(X, [], 1) - min(X, [], 1));
dist = sum(((X - center) ./ scale) .^ 2, 2);
if ismember('robustness', T.Properties.VariableNames)
    dist = dist - 0.05 * double(T.robustness);
end
[~, idx] = min(dist);
row = T(idx, :);
end


function params = build_parameter_sweeps()
params(1).name = 'max_gimbal_deg';
params(1).label = 'Max Gimbal Authority (deg)';
params(1).values = [2 4 6 8 10 12 15 18];

params(2).name = 'servo_slew_deg_s';
params(2).label = 'Servo Slew (deg/s)';
params(2).values = [10 15 20 30 45 60 75 90 120];

params(3).name = 'Kp';
params(3).label = 'Proportional Gain Kp';
params(3).values = [2 5 8 10 15 20 30 45 60];

params(4).name = 'Kd';
params(4).label = 'Derivative Gain Kd';
params(4).values = [0 2 4 6 8 12 16 24 32];
end


function applied = apply_sweep_value(base_case, param_name, sweep_value, P)
applied = struct();
applied.p_unstable = base_case.p_unstable;
applied.max_gimbal_deg = base_case.max_gimbal_deg;
applied.servo_slew_deg_s = base_case.servo_slew_deg_s;
applied.Kp = base_case.Kp;
applied.Kd = base_case.Kd;
applied.control_effectiveness = P.rocket.control_effectiveness;

switch param_name
    case 'max_gimbal_deg'
        applied.max_gimbal_deg = sweep_value;
        applied.control_effectiveness = P.rocket.control_effectiveness * ...
            (sweep_value / max(base_case.max_gimbal_deg, 1e-6));
    case 'servo_slew_deg_s'
        applied.servo_slew_deg_s = sweep_value;
    case 'Kp'
        applied.Kp = sweep_value;
    case 'Kd'
        applied.Kd = sweep_value;
    otherwise
        error('Unknown sweep parameter: %s', param_name);
end
end


function [cfg, sc, realism] = build_case_cfg(P, applied, opts)
P_case = P;
P_case.rocket.max_gimbal = applied.max_gimbal_deg;

override = struct();
override.p_unstable = applied.p_unstable;
override.servo_slew = applied.servo_slew_deg_s;
override.control_effectiveness = applied.control_effectiveness;
override.deadband = P.rocket.deadband;
override.backlash = P.rocket.backlash;
override.latency = P.rocket.latency;
override.wind_strength = P.rocket.wind_strength;
override.t_end = max(opts.step_t_end, opts.envelope_t_end);

[cfg, sc, realism] = build_realistic_cfg(P_case, override);
cfg = configure_pid_controller(cfg, applied.Kp, applied.Kd);
end


function metrics = evaluate_exp5_point(cfg, sc, realism, opts)
env = evaluate_maneuverability_envelope(cfg, sc, realism, opts);
step = evaluate_step_metrics(cfg, sc, realism, opts);

metrics = step;
metrics.envelope_cmd_deg = env.envelope_cmd_deg;
metrics.envelope_cmd_ratio = env.envelope_cmd_ratio;
metrics.reference_success_rate = env.reference_success_rate;
end


function M = evaluate_maneuverability_envelope(cfg, sc, realism, opts)
cmd_success = zeros(numel(opts.cmd_grid_deg), 1);
cmd_rms = zeros(numel(opts.cmd_grid_deg), 1);
cmd_peak = zeros(numel(opts.cmd_grid_deg), 1);

for ic = 1:numel(opts.cmd_grid_deg)
    sc_cmd = apply_pitch_program(sc, opts.cmd_grid_deg(ic), opts);
    m = evaluate_maneuver_cell(cfg, sc_cmd, realism, opts);
    cmd_success(ic) = m.success_rate;
    cmd_rms(ic) = m.rms_error_deg;
    cmd_peak(ic) = m.peak_error_deg;
end

pass_mask = cmd_success >= opts.envelope_success_gate & ...
    cmd_rms <= opts.envelope_rms_err_gate_deg & ...
    cmd_peak <= opts.envelope_peak_err_gate_deg;

if any(pass_mask)
    envelope_cmd_deg = max(opts.cmd_grid_deg(pass_mask));
else
    envelope_cmd_deg = 0;
end

[~, iref] = min(abs(opts.cmd_grid_deg - opts.reference_cmd_deg));

M = struct();
M.envelope_cmd_deg = envelope_cmd_deg;
M.envelope_cmd_ratio = envelope_cmd_deg / max(opts.cmd_grid_deg);
M.reference_success_rate = cmd_success(iref);
end


function metrics = evaluate_maneuver_cell(cfg, sc, realism, opts)
n_trials = numel(opts.theta0_deg_set) * numel(opts.seeds);
success_col = false(n_trials, 1);
rms_col = zeros(n_trials, 1);
peak_col = zeros(n_trials, 1);
end_col = zeros(n_trials, 1);
max_theta_col = zeros(n_trials, 1);
u_sat_col = zeros(n_trials, 1);
slew_sat_col = zeros(n_trials, 1);
settling_col = zeros(n_trials, 1);

idx = 0;
for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1;
        seed = opts.seeds(iseed) + 100 * it;
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
end


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
end


function t_settle = estimate_settling_time_tracking(t, err, band_rad)
t_settle = t(end);
inside = abs(err) <= band_rad;
for k = 1:numel(t)
    if all(inside(k:end))
        t_settle = t(k);
        return;
    end
end
end


function sc_cmd = apply_pitch_program(sc, cmd_deg, opts)
sc_cmd = sc;
sc_cmd.kind = "PITCH_PROGRAM_EXP5";
sc_cmd.t_end = opts.envelope_t_end;
sc_cmd.theta_ref_fun = @(t) pitch_ref_rad(t, cmd_deg, opts);
sc_cmd.q_ref_fun = @(t) pitch_rate_ref_rad_s(t, cmd_deg, opts);
end


function theta = pitch_ref_rad(t, cmd_deg, opts)
amp = deg2rad(cmd_deg);
t0 = opts.envelope_cmd_start_s;
t1 = t0 + opts.envelope_cmd_ramp_s;
t2 = t1 + opts.envelope_cmd_hold_s;
t3 = t2 + opts.envelope_cmd_ramp_s;

if t < t0
    theta = 0;
elseif t < t1
    theta = amp * (t - t0) / opts.envelope_cmd_ramp_s;
elseif t < t2
    theta = amp;
elseif t < t3
    theta = amp * (1 - (t - t2) / opts.envelope_cmd_ramp_s);
else
    theta = 0;
end
end


function q_ref = pitch_rate_ref_rad_s(t, cmd_deg, opts)
amp = deg2rad(cmd_deg);
t0 = opts.envelope_cmd_start_s;
t1 = t0 + opts.envelope_cmd_ramp_s;
t2 = t1 + opts.envelope_cmd_hold_s;
t3 = t2 + opts.envelope_cmd_ramp_s;

if t >= t0 && t < t1
    q_ref = amp / opts.envelope_cmd_ramp_s;
elseif t >= t2 && t < t3
    q_ref = -amp / opts.envelope_cmd_ramp_s;
else
    q_ref = 0;
end
end


function M = evaluate_step_metrics(cfg, sc, realism, opts)
n_trials = numel(opts.theta0_deg_set) * numel(opts.seeds);

overshoot = nan(n_trials, 1);
usat = nan(n_trials, 1);
ssat = nan(n_trials, 1);
succ_like = nan(n_trials, 1);
rms_err = nan(n_trials, 1);
peak_err = nan(n_trials, 1);
settle_reached = nan(n_trials, 1);
bounded_window = nan(n_trials, 1);

idx = 0;
for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1;
        seed = opts.seeds(iseed) + 100 * it;
        try
            out = simulate_case_realistic('PID', apply_step_program(sc, opts), cfg, seed, realism);
            [overshoot(idx), usat(idx), ssat(idx), succ_like(idx), rms_err(idx), ...
                peak_err(idx), settle_reached(idx), bounded_window(idx)] = extract_step_metrics(out, cfg, opts);
        catch
            overshoot(idx) = 100;
            usat(idx) = 1;
            ssat(idx) = 1;
            succ_like(idx) = 0;
            rms_err(idx) = 90;
            peak_err(idx) = 90;
            settle_reached(idx) = 0;
            bounded_window(idx) = 0;
        end
    end
end

M = struct();
M.overshoot_pct = mean(overshoot, 'omitnan');
M.actuator_sat_frac = mean(usat, 'omitnan');
M.slew_sat_frac = mean(ssat, 'omitnan');
M.success_like_rate = mean(succ_like, 'omitnan');
M.rms_err_deg = mean(rms_err, 'omitnan');
M.peak_err_deg = mean(peak_err, 'omitnan');
M.settled_frac = mean(settle_reached, 'omitnan');
M.bounded_window_s = mean(bounded_window, 'omitnan');
M.bounded_window_norm = M.bounded_window_s / max(opts.step_t_end - opts.step_cmd_start_s, 1e-6);
end


function sc_cmd = apply_step_program(sc, opts)
sc_cmd = sc;
sc_cmd.kind = "EXP5_STEP";
sc_cmd.t_end = opts.step_t_end;
sc_cmd.theta_ref_fun = @(t) step_ref_rad(t, opts);
sc_cmd.q_ref_fun = @(t) 0.0;
end


function th = step_ref_rad(t, opts)
if t >= opts.step_cmd_start_s
    th = deg2rad(opts.reference_cmd_deg);
else
    th = 0.0;
end
end


function [overshoot_pct, u_sat_frac, slew_sat_frac, success_like, rms_err_deg, peak_err_deg, settle_reached, bounded_window_s] = extract_step_metrics(out, cfg, opts)
t = out.time;
dt = cfg.dt;
t0 = opts.step_cmd_start_s;
amp = deg2rad(opts.reference_cmd_deg);

mask = t >= t0;
theta = out.theta(mask);
u_cmd = out.u_cmd(mask);
u_act = out.u_act(mask);
t_seg = t(mask);

if isempty(theta)
    overshoot_pct = 100;
    u_sat_frac = 1;
    slew_sat_frac = 1;
    success_like = 0;
    rms_err_deg = 90;
    peak_err_deg = 90;
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
err_b = err(1:iend);
t_b = t_seg(1:iend);
bounded_window_s = t_b(end) - t0;

band = deg2rad(max(opts.settle_band_deg, 0.02 * opts.reference_cmd_deg));
[~, settled_ok] = estimate_settle_time_step(t_b, err_b, band, opts.min_settle_window_s);
settle_reached = double(settled_ok);

peak_theta = max(theta_b, [], 'omitnan');
overshoot_pct = max(0, 100 * (peak_theta - amp) / max(amp, deg2rad(opts.overshoot_floor_deg)));
u_sat_frac = mean(abs(u_cmd) >= 0.99 * cfg.plant.u_max, 'omitnan');
slew_sat_frac = mean(abs(diff(u_act)) / dt >= 0.99 * cfg.plant.slew_max, 'omitnan');

rms_err_deg = min(90, rad2deg(sqrt(mean(err .^ 2, 'omitnan'))));
peak_err_deg = min(90, rad2deg(max(abs(err), [], 'omitnan')));
end_err_deg = min(90, rad2deg(abs(err(end))));
success_like = double(rms_err_deg <= 15 && peak_err_deg <= 50 && end_err_deg <= 15 && settle_reached == 1);
end


function [ts, ok] = estimate_settle_time_step(t, err, band, min_win_s)
ts = 0;
ok = false;
if isempty(t)
    return;
end
if numel(t) < 2
    ok = all(abs(err) <= band);
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
end


function bottleneck = classify_active_bottleneck(param_name, metrics)
if metrics.envelope_cmd_ratio >= 0.95 && metrics.bounded_window_norm >= 0.95
    bottleneck = 'diminishing_returns';
    return;
end
if metrics.slew_sat_frac >= 0.55 && metrics.slew_sat_frac >= metrics.actuator_sat_frac + 0.05
    bottleneck = 'slew_limited';
    return;
end
if metrics.actuator_sat_frac >= 0.55
    bottleneck = 'authority_limited';
    return;
end
if strcmp(param_name, 'Kp') || strcmp(param_name, 'Kd') || metrics.overshoot_pct >= 15 || metrics.bounded_window_norm < 0.90
    bottleneck = 'controller_limited';
else
    bottleneck = 'mixed_limited';
end
end


function thresholdT = build_threshold_summary(rawT, cases, params, opts)
rows = cell(0, 18);
for ic = 1:numel(cases)
    for ip = 1:numel(params)
        mask = strcmp(rawT.case_id, cases(ic).case_id) & strcmp(rawT.parameter, params(ip).name);
        sub = sortrows(rawT(mask, :), 'param_value');
        x = sub.param_value;
        metric = sub.envelope_cmd_deg;
        [fail_t, practical_t, dr_t, max_env, env_at_practical, env_at_dr] = detect_thresholds( ...
            x, metric, opts.failure_frac, opts.practical_frac, opts.diminishing_return_frac);

        practical_idx = find_closest_index(x, practical_t);
        fail_idx = find_closest_index(x, fail_t);
        dr_idx = find_closest_index(x, dr_t);

        rows(end + 1, :) = { ...
            cases(ic).case_id, cases(ic).case_label, cases(ic).source_regime, params(ip).name, ...
            get_base_value(cases(ic), params(ip).name), ...
            fail_t, practical_t, dr_t, max_env, env_at_practical, env_at_dr, ...
            sub.bounded_window_norm(practical_idx), sub.overshoot_pct(practical_idx), ...
            sub.actuator_sat_frac(practical_idx), sub.slew_sat_frac(practical_idx), ...
            sub.active_bottleneck{fail_idx}, sub.active_bottleneck{practical_idx}, ...
            sub.active_bottleneck{dr_idx}}; %#ok<AGROW>
    end
end

thresholdT = cell2table(rows, 'VariableNames', { ...
    'case_id', 'case_label', 'source_regime', 'parameter', 'base_value', ...
    'failure_threshold', 'minimum_practical_threshold', 'diminishing_return_threshold', ...
    'max_envelope_cmd_deg', 'envelope_at_practical_cmd_deg', 'envelope_at_dr_cmd_deg', ...
    'bounded_window_norm_at_practical', 'overshoot_pct_at_practical', ...
    'actuator_sat_frac_at_practical', 'slew_sat_frac_at_practical', ...
    'failure_bottleneck', 'practical_bottleneck', 'dr_bottleneck'});
end


function value = get_base_value(c, param_name)
switch param_name
    case 'max_gimbal_deg'
        value = c.max_gimbal_deg;
    case 'servo_slew_deg_s'
        value = c.servo_slew_deg_s;
    case 'Kp'
        value = c.Kp;
    case 'Kd'
        value = c.Kd;
    otherwise
        value = nan;
end
end


function [fail_t, practical_t, dr_t, max_metric, metric_at_practical, metric_at_dr] = detect_thresholds(xvals, metric, fail_frac, practical_frac, dr_frac)
[xvals, order] = sort(xvals(:));
metric = metric(order);
max_metric = max(metric);

if max_metric <= 0
    fail_t = xvals(end);
    practical_t = xvals(end);
    dr_t = xvals(end);
    metric_at_practical = 0;
    metric_at_dr = 0;
    return;
end

fail_gate = fail_frac * max_metric;
practical_gate = practical_frac * max_metric;
delta_gate = dr_frac * max_metric;

idx_fail = find(metric > fail_gate, 1, 'first');
idx_practical = find(metric >= practical_gate, 1, 'first');
if isempty(idx_fail), idx_fail = numel(xvals); end
if isempty(idx_practical), idx_practical = numel(xvals); end

fail_t = xvals(idx_fail);
practical_t = xvals(idx_practical);
metric_at_practical = metric(idx_practical);

idx_dr = numel(xvals);
for k = idx_practical:numel(xvals)-1
    if metric(k + 1) - metric(k) < delta_gate
        idx_dr = k;
        break;
    end
end
dr_t = xvals(idx_dr);
metric_at_dr = metric(idx_dr);
end


function idx = find_closest_index(xvals, target)
[~, idx] = min(abs(xvals - target));
end


function transitionT = build_bottleneck_transitions(rawT)
rows = cell(0, 8);
case_ids = unique(rawT.case_id, 'stable');
params = unique(rawT.parameter, 'stable');

for ic = 1:numel(case_ids)
    for ip = 1:numel(params)
        mask = strcmp(rawT.case_id, case_ids{ic}) & strcmp(rawT.parameter, params{ip});
        sub = sortrows(rawT(mask, :), 'param_value');
        if isempty(sub)
            continue;
        end

        start_idx = 1;
        current_bn = sub.active_bottleneck{1};
        for k = 2:height(sub)
            if ~strcmp(sub.active_bottleneck{k}, current_bn)
                rows(end + 1, :) = pack_transition_row(sub, start_idx, k - 1, current_bn); %#ok<AGROW>
                start_idx = k;
                current_bn = sub.active_bottleneck{k};
            end
        end
        rows(end + 1, :) = pack_transition_row(sub, start_idx, height(sub), current_bn); %#ok<AGROW>
    end
end

transitionT = cell2table(rows, 'VariableNames', { ...
    'case_id', 'case_label', 'parameter', 'bottleneck', ...
    'start_value', 'end_value', 'n_points', 'range_text'});
end


function row = pack_transition_row(sub, start_idx, end_idx, bottleneck)
start_value = sub.param_value(start_idx);
end_value = sub.param_value(end_idx);
if abs(start_value - end_value) < 1e-9
    range_text = sprintf('%.3g', start_value);
else
    range_text = sprintf('%.3g-%.3g', start_value, end_value);
end

row = { ...
    sub.case_id{start_idx}, sub.case_label{start_idx}, sub.parameter{start_idx}, bottleneck, ...
    start_value, end_value, end_idx - start_idx + 1, range_text};
end


function designRuleT = build_design_rules(thresholdT, transitionT, params)
rows = cell(0, 9);
for i = 1:height(thresholdT)
    tr = thresholdT(i, :);
    trans = transitionT(strcmp(transitionT.case_id, tr.case_id{1}) & strcmp(transitionT.parameter, tr.parameter{1}), :);
    story = summarize_transition_story(trans);

    rule_text = sprintf(['%s %s: below %.3g performance is bottlenecked by %s; ' ...
        '%.3g reaches 90%% of max envelope; gains beyond %.3g show diminishing returns. %s'], ...
        tr.case_label{1}, pretty_param_label(params, tr.parameter{1}), ...
        tr.failure_threshold, tr.failure_bottleneck{1}, ...
        tr.minimum_practical_threshold, tr.diminishing_return_threshold, story);

    rows(end + 1, :) = { ...
        tr.case_id{1}, tr.case_label{1}, tr.parameter{1}, ...
        tr.failure_threshold, tr.minimum_practical_threshold, tr.diminishing_return_threshold, ...
        tr.failure_bottleneck{1}, tr.practical_bottleneck{1}, rule_text}; %#ok<AGROW>
end

designRuleT = cell2table(rows, 'VariableNames', { ...
    'case_id', 'case_label', 'parameter', ...
    'failure_threshold', 'minimum_practical_threshold', 'diminishing_return_threshold', ...
    'failure_bottleneck', 'practical_bottleneck', 'design_rule'});
end


function label = pretty_param_label(params, param_name)
label = param_name;
for i = 1:numel(params)
    if strcmp(params(i).name, param_name)
        label = params(i).label;
        return;
    end
end
end


function story = summarize_transition_story(trans)
if isempty(trans)
    story = 'No bottleneck transitions detected in the tested range.';
    return;
end

parts = cell(height(trans), 1);
for i = 1:height(trans)
    parts{i} = sprintf('%s: %s', trans.range_text{i}, strrep(trans.bottleneck{i}, '_', '-'));
end
story = strjoin(parts, '; ');
end


function plot_parameter_dashboard(rawT, thresholdT, cases, param_name, out_png, fig_title)
sub = rawT(strcmp(rawT.parameter, param_name), :);
colors = lines(numel(cases));
params = build_parameter_sweeps();

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1200 760]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

metric_specs = { ...
    'envelope_cmd_deg', 'Maneuverability Envelope (deg)'; ...
    'bounded_window_norm', 'Bounded Window (norm.)'; ...
    'overshoot_pct', 'Overshoot (%)'; ...
    'actuator_sat_frac', 'Actuator Saturation'};

for im = 1:size(metric_specs, 1)
    ax = nexttile;
    hold(ax, 'on');
    for ic = 1:numel(cases)
        case_sub = sortrows(sub(strcmp(sub.case_id, cases(ic).case_id), :), 'param_value');
        plot(ax, case_sub.param_value, case_sub.(metric_specs{im, 1}), '-o', ...
            'Color', colors(ic, :), 'LineWidth', 1.8, 'MarkerSize', 5, ...
            'DisplayName', cases(ic).case_label);
        if im == 1
            th_case = thresholdT(strcmp(thresholdT.case_id, cases(ic).case_id) & strcmp(thresholdT.parameter, param_name), :);
            if ~isempty(th_case)
                add_threshold_lines(ax, th_case);
            end
        end
    end
    xlabel(ax, pretty_param_label(params, param_name));
    ylabel(ax, metric_specs{im, 2});
    grid(ax, 'on');
    box(ax, 'on');
    if im == 1
        legend(ax, 'Location', 'best');
    end
end

sgtitle(fig, fig_title, 'FontWeight', 'bold');
exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);
end


function add_threshold_lines(ax, th_case)
if ~isnan(th_case.failure_threshold)
    xline(ax, th_case.failure_threshold, '--', 'Color', [0.80 0.20 0.20], 'LineWidth', 1.2);
end
if ~isnan(th_case.minimum_practical_threshold)
    xline(ax, th_case.minimum_practical_threshold, '--', 'Color', [0.15 0.60 0.20], 'LineWidth', 1.2);
end
if ~isnan(th_case.diminishing_return_threshold)
    xline(ax, th_case.diminishing_return_threshold, ':', 'Color', [0.35 0.20 0.75], 'LineWidth', 1.2);
end
end


function plot_bottleneck_transition_panels(rawT, cases, params, out_png)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1200 760]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
colors = bottleneck_colors();

for ip = 1:numel(params)
    ax = nexttile;
    hold(ax, 'on');
    for ic = 1:numel(cases)
        sub = rawT(strcmp(rawT.case_id, cases(ic).case_id) & strcmp(rawT.parameter, params(ip).name), :);
        sub = sortrows(sub, 'param_value');
        code = zeros(height(sub), 1);
        for j = 1:height(sub)
            code(j) = bottleneck_code(sub.active_bottleneck{j});
        end
        if ~isempty(code)
            scatter(ax, sub.param_value, ic * ones(height(sub), 1), 90, colors(code, :), 'filled');
            plot(ax, sub.param_value, ic * ones(height(sub), 1), '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
        end
    end
    xlabel(ax, params(ip).label);
    ylabel(ax, 'Case');
    yticks(ax, 1:numel(cases));
    yticklabels(ax, {cases.case_label});
    title(ax, params(ip).label);
    grid(ax, 'on');
    box(ax, 'on');
end

legend_entries = {'authority-limited', 'slew-limited', 'controller-limited', 'diminishing-returns', 'mixed-limited'};
ha = axes(fig, 'Visible', 'off');
hold(ha, 'on');
for i = 1:numel(legend_entries)
    scatter(ha, nan, nan, 90, colors(i, :), 'filled', 'DisplayName', legend_entries{i});
end
legend(ha, 'Location', 'southoutside', 'Orientation', 'horizontal');

sgtitle(fig, 'Exp5 Bottleneck Transition Plots', 'FontWeight', 'bold');
exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);
end


function plot_case_studies(rawT, thresholdT, cases, params, out_png)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1500 900]);
tiledlayout(numel(cases), numel(params), 'Padding', 'compact', 'TileSpacing', 'compact');
case_colors = lines(numel(cases));

for ic = 1:numel(cases)
    for ip = 1:numel(params)
        ax = nexttile;
        sub = rawT(strcmp(rawT.case_id, cases(ic).case_id) & strcmp(rawT.parameter, params(ip).name), :);
        sub = sortrows(sub, 'param_value');
        plot(ax, sub.param_value, sub.envelope_cmd_deg, '-o', 'Color', case_colors(ic, :), ...
            'LineWidth', 1.8, 'MarkerSize', 5);
        hold(ax, 'on');
        yyaxis(ax, 'right');
        plot(ax, sub.param_value, sub.actuator_sat_frac, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.0);
        ylabel(ax, 'Actuator sat');
        yyaxis(ax, 'left');

        th = thresholdT(strcmp(thresholdT.case_id, cases(ic).case_id) & strcmp(thresholdT.parameter, params(ip).name), :);
        if ~isempty(th)
            add_threshold_lines(ax, th);
        end

        grid(ax, 'on');
        box(ax, 'on');
        xlabel(ax, params(ip).label);
        ylabel(ax, 'Envelope (deg)');
        if ic == 1
            title(ax, params(ip).label);
        end
        if ip == 1
            text(ax, 0.02, 0.95, cases(ic).case_label, 'Units', 'normalized', ...
                'FontWeight', 'bold', 'VerticalAlignment', 'top');
        end
    end
end

sgtitle(fig, 'Exp5 Three Representative Rocket Case Studies', 'FontWeight', 'bold');
exportgraphics(fig, out_png, 'Resolution', 220);
close(fig);
end


function colors = bottleneck_colors()
colors = [ ...
    0.85 0.33 0.10; ...
    0.00 0.45 0.74; ...
    0.49 0.18 0.56; ...
    0.47 0.67 0.19; ...
    0.55 0.55 0.55];
end


function code = bottleneck_code(label)
switch char(label)
    case 'authority_limited'
        code = 1;
    case 'slew_limited'
        code = 2;
    case 'controller_limited'
        code = 3;
    case 'diminishing_returns'
        code = 4;
    otherwise
        code = 5;
end
end