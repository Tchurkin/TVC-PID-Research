function T = run_exp1_stability_authority_frontier(n_designs, rng_seed)
%RUN_EXP1_STABILITY_AUTHORITY_FRONTIER
% Experiment 1: map EASY/FRAGILE/INFEASIBLE over high-dimensional design space.
%
% Uses Latin Hypercube Sampling over key physical design parameters instead of
% a small Cartesian grid. This preserves compatibility with Exp4/Exp5 while
% exposing meaningful parameter-space variation in pipeline outputs.

if nargin < 1 || isempty(n_designs)
    n_designs = 1200;
end
if nargin < 2 || isempty(rng_seed)
    rng_seed = 42;
end

this_dir = fileparts(mfilename('fullpath'));
framework_dir = fileparts(this_dir);
exp_dir = fileparts(framework_dir);
proj_dir = fileparts(exp_dir);

addpath(fullfile(proj_dir, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(framework_dir, 'plant'));
addpath(fullfile(framework_dir, 'controllers'));
addpath(fullfile(framework_dir, 'analysis'));

P = default_rocket_params();
designT = sample_design_space_lhs(n_designs, rng_seed, P);

results_dir = fullfile(exp_dir, 'results');
graphs_dir = fullfile(results_dir, 'graphs');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
if ~exist(graphs_dir, 'dir'); mkdir(graphs_dir); end

rows = cell(0, 39);
use_fast_eval = height(designT) >= 800;

fprintf('=== EXP1: STABILITY AUTHORITY FRONTIER ===\n');
fprintf('LHS designs: N=%d, seed=%d\n\n', height(designT), rng_seed);
if use_fast_eval
    fprintf('Fast-eval mode enabled (reduced Monte Carlo/tuning grids for throughput).\n\n');
end

for i = 1:height(designT)
    d = designT(i, :);

    P_case = P;
    P_case.rocket.mass = d.mass;
    P_case.rocket.Iyy = d.Iyy;
    P_case.rocket.static_margin = d.static_margin;
    P_case.rocket.Cm_alpha = d.Cm_alpha;
    P_case.rocket.control_effectiveness = d.control_effectiveness;
    P_case.rocket.thrust = d.thrust;
    P_case.rocket.servo_slew = d.servo_slew_deg_s;
    P_case.rocket.max_gimbal = d.max_gimbal_deg;
    P_case.rocket.deadband = d.deadband;
    P_case.rocket.backlash = d.backlash;
    P_case.rocket.latency = d.latency_steps;
    P_case.rocket.wind_strength = d.wind_strength;
    if use_fast_eval
        P_case.analysis.theta0_deg_set = 0;
        P_case.analysis.seeds = 1;
        P_case.tuning.Kp_grid = [20 45 80];
        P_case.tuning.Kd_grid = [8 16 32];
    end

    override = struct();
    override.p_unstable = d.p_unstable;
    override.control_effectiveness = d.control_effectiveness;
    override.servo_slew = d.servo_slew_deg_s;
    override.u_max_frac = d.best_u_max_frac;
    override.deadband = d.deadband;
    override.backlash = d.backlash;
    override.latency = d.latency_steps;
    override.wind_strength = d.wind_strength;

    [cfg, sc, realism] = build_realistic_cfg(P_case, override);
    tuned = autotune_pd_grid(cfg, sc, realism, P_case);

    nominal = tuned;
    under_cfg = configure_pid_controller(cfg, P_case.tuning.under_scale * tuned.Kp, P_case.tuning.under_scale * tuned.Kd);
    over_cfg = configure_pid_controller(cfg, P_case.tuning.over_scale * tuned.Kp, P_case.tuning.over_scale * tuned.Kd);
    under = evaluate_stability_cell(under_cfg, sc, realism, P_case);
    over = evaluate_stability_cell(over_cfg, sc, realism, P_case);

    robust_gate = P_case.classification.fragile_success;
    n_success_sets = double(nominal.success_rate >= robust_gate) + ...
        double(under.success_rate >= robust_gate) + ...
        double(over.success_rate >= robust_gate);
    robustness = n_success_sets / 3.0;

    [label, code] = classify_regime(nominal, robustness, P_case);

    rows(end + 1, :) = { ...
        d.p_unstable, ...
        d.servo_slew_deg_s, ...
        d.max_gimbal_deg, ...
        d.best_u_max_frac, ...
        tuned.Kp, ...
        tuned.Kd, ...
        nominal.success_rate, ...
        under.success_rate, ...
        over.success_rate, ...
        robustness, ...
        n_success_sets, ...
        nominal.rms_error_deg, ...
        nominal.peak_error_deg, ...
        nominal.end_error_deg, ...
        nominal.max_theta_deg, ...
        nominal.u_cmd_sat_frac, ...
        nominal.slew_sat_frac, ...
        nominal.settling_time_s, ...
        nominal.oscillation_score, ...
        under.u_cmd_sat_frac, ...
        over.u_cmd_sat_frac, ...
        under.settling_time_s, ...
        over.settling_time_s, ...
        string(label), ...
        code, ...
        d.wind_strength, ...
        d.latency_steps, ...
        robust_gate, ...
        d.mass, ...
        d.Iyy, ...
        d.static_margin, ...
        d.Cm_alpha, ...
        d.control_effectiveness, ...
        d.thrust, ...
        d.deadband, ...
        d.backlash, ...
        d.nominal_servo_slew_deg_s, ...
        d.nominal_max_gimbal_deg, ...
        d.design_id}; %#ok<AGROW>

    if mod(i, 100) == 0 || i == height(designT)
        fprintf('  processed %d/%d designs\n', i, height(designT));
    end
end

T = cell2table(rows, 'VariableNames', { ...
    'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'best_u_max_frac', ...
    'best_Kp', 'best_Kd', ...
    'nominal_success_rate', 'under_success_rate', 'over_success_rate', ...
    'robustness', 'n_success_gain_sets', ...
    'rms_error_deg', 'peak_error_deg', 'end_error_deg', 'max_theta_deg', ...
    'u_cmd_sat_frac', 'slew_sat_frac', 'settling_time_s', 'oscillation_score', ...
    'under_u_sat_frac', 'over_u_sat_frac', 'under_settling_time_s', ...
    'over_settling_time_s', 'regime_label', 'regime_code', ...
    'wind_strength', 'latency_steps', 'robust_success_threshold', ...
    'mass', 'Iyy', 'static_margin', 'Cm_alpha', 'control_effectiveness', ...
    'thrust', 'deadband', 'backlash', 'nominal_servo_slew_deg_s', ...
    'nominal_max_gimbal_deg', 'design_id'});

T.rocket_id = make_rocket_ids(height(T));
T.boundary_distance = compute_boundary_distance(T);
T = annotate_pipeline_table(T, P, 'Exp1', 'regime_structure');

csv_path = fullfile(results_dir, 'phase_diagram.csv');
index_path = fullfile(results_dir, 'exp1_regime_index.csv');
png_map_path = fullfile(graphs_dir, 'phase_diagram.png');
png_frontier_path = fullfile(graphs_dir, 'required_slew_frontier.png');

writetable(T, csv_path);
idx_cols = {'rocket_id', 'regime_label', 'regime_code', 'boundary_distance', ...
    'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'best_u_max_frac', ...
    'best_Kp', 'best_Kd', 'nominal_success_rate', 'under_success_rate', ...
    'over_success_rate', 'robustness', 'mass', 'Iyy', 'static_margin', ...
    'Cm_alpha', 'control_effectiveness', 'thrust', 'deadband', 'backlash', ...
    'latency_steps', 'wind_strength', 'nominal_servo_slew_deg_s', ...
    'nominal_max_gimbal_deg', 'design_id'};
idx_cols = idx_cols(ismember(idx_cols, T.Properties.VariableNames));
writetable(T(:, idx_cols), index_path);
plot_phase_diagram(T, png_map_path, png_frontier_path);

fragile_cells = sum(T.regime_code == 1);
fprintf('\nRegime counts: EASY=%d FRAGILE=%d INFEASIBLE=%d\n', ...
    sum(T.regime_code == 2), fragile_cells, sum(T.regime_code == 0));
if fragile_cells == 0
    warning('run_exp1_stability_authority_frontier:noFragile', ...
        'No FRAGILE cells detected. Revisit classifier thresholds or sweep envelope before Exp2.');
end

fprintf('\nSaved: %s\n', csv_path);
fprintf('Saved: %s\n', index_path);
fprintf('Saved: %s\n', png_map_path);
fprintf('Saved: %s\n', png_frontier_path);
end


function ids = make_rocket_ids(n)
ids = strings(n, 1);
for i = 1:n
    ids(i) = sprintf('R%04d', i);
end
end


function d = compute_boundary_distance(T)
design_axes = {'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', 'static_margin', ...
    'control_effectiveness', 'thrust', 'deadband', 'backlash', 'latency_steps', ...
    'wind_strength', 'mass', 'Iyy', 'Cm_alpha'};
design_axes = design_axes(ismember(design_axes, T.Properties.VariableNames));
P = table2array(T(:, design_axes));
P = double(P);
mn = min(P, [], 1);
mx = max(P, [], 1);
span = max(mx - mn, 1e-9);
Pn = (P - mn) ./ span;

d = zeros(height(T), 1);
codes = double(T.regime_code);
for i = 1:height(T)
    other = find(codes ~= codes(i));
    if isempty(other)
        min_dist = 0;
    else
        delta = abs(Pn(other, :) - Pn(i, :));
        min_dist = min(sum(delta, 2));
    end

    if codes(i) == 2
        sgn = 1;
    elseif codes(i) == 0
        sgn = -1;
    else
        sgn = 0;
    end
    d(i) = sgn * min_dist;
end
end


function designT = sample_design_space_lhs(n_designs, rng_seed, P)
spec_names = {'mass', 'Iyy', 'static_margin', 'Cm_alpha', 'control_effectiveness', ...
    'thrust', 'servo_slew_deg_s', 'max_gimbal_deg', 'latency_steps', ...
    'deadband', 'backlash', 'wind_strength'};
mins = [0.90, 0.010, 0.04, -70.0, 5.0, 25.0, 20.0, 8.0, 1.0, 0.00, 0.00, 0.05];
maxs = [2.10, 0.040, 0.24, -30.0, 14.0, 50.0, 120.0, 15.0, 6.0, 0.15, 0.25, 0.45];
is_int = [false, false, false, false, false, false, false, false, true, false, false, false];

allT = table();
iter = 0;
while height(allT) < n_designs
    iter = iter + 1;
    n_need = n_designs - height(allT);
    n_batch = max(200, ceil(1.8 * n_need));

    U = latin_hypercube_matrix(n_batch, numel(spec_names), rng_seed + iter);
    X = mins + U .* (maxs - mins);
    for j = 1:numel(is_int)
        if is_int(j)
            X(:, j) = round(X(:, j));
        end
    end

    batchT = array2table(X, 'VariableNames', spec_names);
    batchT = unique(batchT, 'rows', 'stable');
    allT = [allT; batchT]; %#ok<AGROW>
    allT = unique(allT, 'rows', 'stable');
end

designT = allT(1:n_designs, :);

% Derive p_unstable from sampled physical parameters.
designT.p_unstable = max(0.0, 10.0 .* designT.static_margin .* abs(designT.Cm_alpha) ./ 52.0 .* sqrt(35.0 ./ designT.thrust));
designT.best_u_max_frac = max(0.05, min(1.0, designT.max_gimbal_deg ./ P.rocket.max_gimbal));
designT.nominal_servo_slew_deg_s = designT.servo_slew_deg_s;
designT.nominal_max_gimbal_deg = designT.max_gimbal_deg;
designT.design_id = (1:height(designT))';
end


function U = latin_hypercube_matrix(n, d, rng_seed)
rng(rng_seed);
U = zeros(n, d);
for j = 1:d
    perm = randperm(n)';
    U(:, j) = ((perm - 1) + rand(n, 1)) / n;
end
end
