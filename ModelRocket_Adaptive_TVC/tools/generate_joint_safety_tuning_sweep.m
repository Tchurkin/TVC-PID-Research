function generate_joint_safety_tuning_sweep(root_dir)
%GENERATE_JOINT_SAFETY_TUNING_SWEEP  Coordinate sweep for safety-cert JOINT tuning.
%
% Produces:
%   outputs/data/joint_safety_tuning_sweep.csv
%   outputs/data/joint_safety_tuning_best.csv
%
% Goal: find safety-cert settings that preserve pass rate while reducing
% false-saturation triggers and avoiding RMS regression.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

out_dir = fullfile(root_dir, 'outputs', 'data');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

cfg_base = rocket_defaults();
cfg_base.plant.theta0 = deg2rad(5);

sc = rocket_scenario("SLEW_DEGRADATION", cfg_base);
sc.t_end = 10.0;
sc.disturbance_amp = 3.5;
sc.disturbance_freq_hz = 1.5;

seeds = 1:20;

% Legacy benchmark (must not regress substantially).
legacy = struct();
legacy.gate_disturb_gain = 0.0;
legacy.conf_min = 1.0;
legacy.conf_resid_gain = 0.0;
legacy.conf_floor_blend = 1.0;
legacy.safety_cmd_slew_frac = 10.0;
legacy.theta_guard_rad = deg2rad(90);
legacy_metrics = eval_variant(cfg_base, sc, seeds, legacy);

% Start from current safety-cert settings.
best = struct();
best.gate_disturb_gain = 0.10;
best.conf_min = 0.75;
best.conf_resid_gain = 1.0;
best.conf_floor_blend = 0.85;
best.safety_cmd_slew_frac = 4.0;
best.theta_guard_rad = deg2rad(80);
best_metrics = eval_variant(cfg_base, sc, seeds, best);

rows = {};
iter = 1;
rows(end+1,:) = row_from_eval(iter, "legacy", legacy, legacy_metrics, legacy_metrics); %#ok<AGROW>
iter = iter + 1;
rows(end+1,:) = row_from_eval(iter, "start_safety_cert", best, best_metrics, legacy_metrics); %#ok<AGROW>
iter = iter + 1;

sweep = {
    struct('name','gate_disturb_gain',   'vals',[0.00 0.05 0.10 0.15 0.20])
    struct('name','conf_min',            'vals',[0.70 0.80 0.90 1.00])
    struct('name','conf_resid_gain',     'vals',[0.0 0.5 1.0 1.5])
    struct('name','conf_floor_blend',    'vals',[0.80 0.90 1.00])
    struct('name','safety_cmd_slew_frac','vals',[3.0 4.0 5.0 6.0 8.0 10.0])
    struct('name','theta_guard_rad',     'vals',deg2rad([75 80 85 90]))
};

for p = 1:numel(sweep)
    pname = sweep{p}.name;
    vals  = sweep{p}.vals;

    local_best = best;
    local_metrics = best_metrics;
    local_score = score_metrics(local_metrics, legacy_metrics);

    for vi = 1:numel(vals)
        cand = best;
        cand.(pname) = vals(vi);

        m = eval_variant(cfg_base, sc, seeds, cand);
        s = score_metrics(m, legacy_metrics);

        rows(end+1,:) = row_from_eval(iter, pname, cand, m, legacy_metrics); %#ok<AGROW>
        iter = iter + 1;

        if s > local_score
            local_best = cand;
            local_metrics = m;
            local_score = s;
        end
    end

    best = local_best;
    best_metrics = local_metrics;
end

T = cell2table(rows, 'VariableNames', {
    'iter','sweep_param', ...
    'gate_disturb_gain','conf_min','conf_resid_gain','conf_floor_blend','safety_cmd_slew_frac','theta_guard_rad_deg', ...
    'pass_frac','rms_post_deg','peak_post_deg','false_sat_frac','late_det_frac','conf_mean_post','shield_frac_post', ...
    'score','viable'
});

csv_all = fullfile(out_dir, 'joint_safety_tuning_sweep.csv');
writetable(T, csv_all);

best_row = table({"best"}, best.gate_disturb_gain, best.conf_min, best.conf_resid_gain, ...
    best.conf_floor_blend, best.safety_cmd_slew_frac, rad2deg(best.theta_guard_rad), ...
    best_metrics.pass_frac, best_metrics.rms_post_deg, best_metrics.peak_post_deg, ...
    best_metrics.false_sat_frac, best_metrics.late_det_frac, best_metrics.conf_mean_post, ...
    best_metrics.shield_frac_post, score_metrics(best_metrics, legacy_metrics), ...
    metrics_viable(best_metrics, legacy_metrics), ...
    'VariableNames', {
    'tag','gate_disturb_gain','conf_min','conf_resid_gain','conf_floor_blend','safety_cmd_slew_frac','theta_guard_rad_deg', ...
    'pass_frac','rms_post_deg','peak_post_deg','false_sat_frac','late_det_frac','conf_mean_post','shield_frac_post', ...
    'score','viable'});

csv_best = fullfile(out_dir, 'joint_safety_tuning_best.csv');
writetable(best_row, csv_best);

fprintf('\n=== JOINT SAFETY TUNING SWEEP SUMMARY ===\n');
fprintf('Legacy   : pass=%5.2f rms=%6.2f false_sat=%5.1f%% late_det=%5.1f%% conf=%4.2f\n', ...
    legacy_metrics.pass_frac, legacy_metrics.rms_post_deg, 100*legacy_metrics.false_sat_frac, ...
    100*legacy_metrics.late_det_frac, legacy_metrics.conf_mean_post);
fprintf('Best cand: pass=%5.2f rms=%6.2f false_sat=%5.1f%% late_det=%5.1f%% conf=%4.2f\n', ...
    best_metrics.pass_frac, best_metrics.rms_post_deg, 100*best_metrics.false_sat_frac, ...
    100*best_metrics.late_det_frac, best_metrics.conf_mean_post);

fprintf('Best parameters:\n');
fprintf('  gate_disturb_gain   = %.3f\n', best.gate_disturb_gain);
fprintf('  conf_min            = %.3f\n', best.conf_min);
fprintf('  conf_resid_gain     = %.3f\n', best.conf_resid_gain);
fprintf('  conf_floor_blend    = %.3f\n', best.conf_floor_blend);
fprintf('  safety_cmd_slew_frac= %.3f\n', best.safety_cmd_slew_frac);
fprintf('  theta_guard_rad     = %.3f deg\n', rad2deg(best.theta_guard_rad));

fprintf('Saved sweep: %s\n', csv_all);
fprintf('Saved best : %s\n\n', csv_best);
end


function metrics = eval_variant(cfg_base, sc, seeds, params)
% Evaluate one parameter set over multiple seeds in realistic simulation.

n = numel(seeds);
pass = zeros(n,1);
rmsv = zeros(n,1);
peak = zeros(n,1);
false_sat = zeros(n,1);
late_det = zeros(n,1);
conf = zeros(n,1);
shield = zeros(n,1);

for i = 1:n
    cfg = cfg_base;
    cfg.controllers.JOINT_ADAPTIVE.gate_disturb_gain = params.gate_disturb_gain;
    cfg.controllers.JOINT_ADAPTIVE.conf_min = params.conf_min;
    cfg.controllers.JOINT_ADAPTIVE.conf_resid_gain = params.conf_resid_gain;
    cfg.controllers.JOINT_ADAPTIVE.conf_floor_blend = params.conf_floor_blend;
    cfg.controllers.JOINT_ADAPTIVE.safety_cmd_slew_frac = params.safety_cmd_slew_frac;
    cfg.controllers.JOINT_ADAPTIVE.theta_guard_rad = params.theta_guard_rad;

    out = simulate_case_realistic("JOINT_ADAPTIVE", sc, cfg, seeds(i), struct());

    pre = out.time < sc.fault_time & out.time > 1.0;
    pm = out.time >= sc.fault_time;

    rms_post = rad2deg(rms(out.theta(pm)));
    peak_post = rad2deg(max(abs(out.theta(pm))));
    pass(i) = (rms_post < 20) && (peak_post < 60);
    rmsv(i) = rms_post;
    peak(i) = peak_post;

    sat_pre = mean(out.saturating(pre), 'omitnan');
    slew_pre = mean(out.slew_scale(pre), 'omitnan');
    false_sat(i) = (sat_pre > 0.25) && (slew_pre < 0.85);

    tpost = out.time(pm) - sc.fault_time;
    ssp = out.slew_scale(pm);
    idx_det = find(ssp < 0.60, 1, 'first');
    if isempty(idx_det)
        late_det(i) = 1.0;
    else
        late_det(i) = double(tpost(idx_det) > 0.75);
    end

    conf(i) = mean(out.confidence(pm), 'omitnan');
    shield(i) = mean(out.shield_active(pm), 'omitnan');
end

metrics.pass_frac = mean(pass, 'omitnan');
metrics.rms_post_deg = mean(rmsv, 'omitnan');
metrics.peak_post_deg = mean(peak, 'omitnan');
metrics.false_sat_frac = mean(false_sat, 'omitnan');
metrics.late_det_frac = mean(late_det, 'omitnan');
metrics.conf_mean_post = mean(conf, 'omitnan');
metrics.shield_frac_post = mean(shield, 'omitnan');
end


function s = score_metrics(m, legacy)
% Higher is better. Penalize regressions from legacy.
viable = metrics_viable(m, legacy);
base = 220*m.pass_frac - 2.0*m.rms_post_deg - 120*m.false_sat_frac ...
    - 60*m.late_det_frac + 12*m.conf_mean_post - 8*m.shield_frac_post;
if viable
    s = base;
else
    % Strong penalty for non-viable settings.
    s = base - 500;
end
end


function tf = metrics_viable(m, legacy)
% Must preserve pass rate and avoid major RMS regression versus legacy.
tf = (m.pass_frac >= legacy.pass_frac - 1e-6) && ...
     (m.rms_post_deg <= 1.10 * legacy.rms_post_deg);
end


function row = row_from_eval(iter, sweep_param, p, m, legacy)
row = {
    iter, char(string(sweep_param)), ...
    p.gate_disturb_gain, p.conf_min, p.conf_resid_gain, p.conf_floor_blend, p.safety_cmd_slew_frac, rad2deg(p.theta_guard_rad), ...
    m.pass_frac, m.rms_post_deg, m.peak_post_deg, m.false_sat_frac, m.late_det_frac, m.conf_mean_post, m.shield_frac_post, ...
    score_metrics(m, legacy), metrics_viable(m, legacy)
};
end
