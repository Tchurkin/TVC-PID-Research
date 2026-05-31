function generate_joint_ablation_demo(root_dir)
%GENERATE_JOINT_ABLATION_DEMO  Component ablation of JOINT_ADAPTIVE.
%
%  Tests which pieces actually create robustness gains:
%   A) Full JOINT_ADAPTIVE (reference)
%   B) No keff-freeze arbitration under slew degradation
%   C) No slew scaling (only keff scaling active)
%   D) Weak saturation gate (very permissive, noise-prone)
%   E) keff-RLS baseline (single-estimator comparator)
%
%  Runs each variant in both clean and realistic simulation models.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
data_dir  = fullfile(root_dir, 'outputs', 'data');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end
if ~exist(data_dir,  'dir'), mkdir(data_dir);  end

cfg0 = rocket_defaults();
cfg0.plant.theta0 = deg2rad(5);
sc = rocket_scenario("SLEW_DEGRADATION", cfg0);
sc.disturbance_amp     = 3.0;
sc.disturbance_freq_hz = 1.5;
sc.t_end               = 10.0;
seed = 7;

variants = {
    struct('name','Full JOINT',           'ctrl','JOINT_ADAPTIVE',     'kind','full')
    struct('name','No keff freeze',       'ctrl','JOINT_ADAPTIVE',     'kind','no_freeze')
    struct('name','No slew scaling',      'ctrl','JOINT_ADAPTIVE',     'kind','no_slew_scale')
    struct('name','Weak sat gate',        'ctrl','JOINT_ADAPTIVE',     'kind','weak_gate')
    struct('name','keff-RLS baseline',    'ctrl','ADAPTIVE_KEFF_LQR',  'kind','keff_baseline')
};

rows = [];
for vi = 1:numel(variants)
    % clean model
    cfg = apply_variant(cfg0, variants{vi}.kind);
    out_clean = simulate_case(variants{vi}.ctrl, sc, cfg, seed);

    % realistic model
    cfg = apply_variant(cfg0, variants{vi}.kind);
    out_real = simulate_case_realistic(variants{vi}.ctrl, sc, cfg, seed, struct());

    pm_c = out_clean.time >= sc.fault_time;
    pm_r = out_real.time  >= sc.fault_time;

    clean_rms = rad2deg(rms(out_clean.theta(pm_c)));
    clean_pk  = rad2deg(max(abs(out_clean.theta(pm_c))));
    real_rms  = rad2deg(rms(out_real.theta(pm_r)));
    real_pk   = rad2deg(max(abs(out_real.theta(pm_r))));

    % Detection metrics only valid for JOINT variants
    slew_mean_post = NaN;
    keff_gate_post = NaN;
    if isfield(out_real, 'slew_scale') && any(~isnan(out_real.slew_scale(pm_r)))
        slew_mean_post = mean(out_real.slew_scale(pm_r), 'omitnan');
    end
    if isfield(out_real, 'keff_gate') && any(~isnan(out_real.keff_gate(pm_r)))
        keff_gate_post = mean(out_real.keff_gate(pm_r), 'omitnan');
    end

    rows = [rows; vi, clean_rms, clean_pk, real_rms, real_pk, slew_mean_post, keff_gate_post]; %#ok<AGROW>
end

T = array2table(rows, 'VariableNames', ...
    {'variant_idx','clean_rms_post_deg','clean_peak_post_deg','real_rms_post_deg','real_peak_post_deg','slew_scale_post_mean','keff_gate_post_frac'});
T.variant = strings(height(T),1);
for i = 1:height(T), T.variant(i) = variants{i}.name; end
T = movevars(T, 'variant', 'Before', 'variant_idx');

csv_path = fullfile(data_dir, 'joint_ablation.csv');
writetable(T, csv_path);

% Figure
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1320 560]);
labels = string(T.variant);
x = 1:numel(labels);

ax1 = subplot(1,2,1); hold(ax1,'on'); grid(ax1,'on');
bar(ax1, x-0.18, T.clean_rms_post_deg, 0.35, 'FaceColor', [0.20 0.45 0.85], 'DisplayName', 'Clean sim');
bar(ax1, x+0.18, T.real_rms_post_deg,  0.35, 'FaceColor', [0.10 0.65 0.30], 'DisplayName', 'Realistic sim');
set(ax1, 'XTick', x, 'XTickLabel', labels, 'XTickLabelRotation', 18, 'FontSize', 10);
ylabel(ax1, 'Post-fault RMS (deg)', 'FontSize', 11, 'FontWeight', 'bold');
title(ax1, 'Ablation impact on post-fault RMS', 'FontSize', 12, 'FontWeight', 'bold');
legend(ax1, 'Location', 'northwest');

ax2 = subplot(1,2,2); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, x, T.slew_scale_post_mean, '-o', 'Color', [0.10 0.65 0.30], 'LineWidth', 2.0, 'DisplayName', 'Mean slew scale (post)');
plot(ax2, x, T.keff_gate_post_frac, '-s', 'Color', [0.80 0.35 0.15], 'LineWidth', 2.0, 'DisplayName', 'keff gate fraction (post)');
yline(ax2, 0.7, ':k', 'LineWidth', 1.2, 'DisplayName', 'keff-freeze threshold');
set(ax2, 'XTick', x, 'XTickLabel', labels, 'XTickLabelRotation', 18, 'FontSize', 10);
ylabel(ax2, 'Estimator/gating diagnostics', 'FontSize', 11, 'FontWeight', 'bold');
title(ax2, 'Why ablations fail: estimator behavior', 'FontSize', 12, 'FontWeight', 'bold');
legend(ax2, 'Location', 'southwest');
ylim(ax2, [0 1.05]);

sgtitle({'Joint controller ablation study (slew-degradation scenario)'; ...
    'Shows contribution of arbitration, slew scaling, and robust sat gate'}, ...
    'FontSize', 13, 'FontWeight', 'bold');

png_path = fullfile(graph_dir, 'joint_ablation_demo.png');
exportgraphics(fig, png_path, 'Resolution', 220);
close(fig);

fprintf('\n=== JOINT ABLATION SUMMARY ===\n');
for i = 1:height(T)
    fprintf('  %-20s  clean RMS=%7.2f deg  realistic RMS=%7.2f deg  slew_scale_post=%5.2f  keff_gate_post=%5.2f\n', ...
        char(T.variant(i)), T.clean_rms_post_deg(i), T.real_rms_post_deg(i), T.slew_scale_post_mean(i), T.keff_gate_post_frac(i));
end
fprintf('Saved figure: %s\n', png_path);
fprintf('Saved data:   %s\n\n', csv_path);
end


function cfg = apply_variant(cfg, kind)
% Apply JOINT ablation tweaks.
switch string(kind)
    case "full"
        % no change
    case "no_freeze"
        cfg.controllers.JOINT_ADAPTIVE.slew_health_keff_freeze = 0.0;
    case "no_slew_scale"
        cfg.controllers.JOINT_ADAPTIVE.slew_scale_min = 1.0;
    case "weak_gate"
        cfg.controllers.JOINT_ADAPTIVE.sat_streak_min = 1;
        cfg.controllers.JOINT_ADAPTIVE.sat_noise_floor = 0.0;
        cfg.controllers.JOINT_ADAPTIVE.sat_decay = 1.0;
    case "keff_baseline"
        % comparator only; no JOINT fields used
    otherwise
        error('apply_variant:UnknownVariant', 'Unknown ablation variant: %s', kind);
end
end
