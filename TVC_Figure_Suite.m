%% ========================================================================
%% TVC FIGURE SUITE FROM OUTPUT TABLES
%% ========================================================================

clear; close all; clc;

root_dir = pwd;
out_dir = fullfile(root_dir, 'outputs');
fig_dir = fullfile(out_dir, 'figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

warn_state = warning('query', 'MATLAB:table:ModifiedAndSavedVarnames');
warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');

regime = readtable(fullfile(out_dir, 'regime_effect_summary.csv'), 'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');
atlas = readtable(fullfile(out_dir, 'control_limitation_regime_atlas.csv'), 'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');
results = readtable(fullfile(out_dir, 'fidelity_results_by_regime.csv'), 'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');
thresh = readtable(fullfile(out_dir, 'aerodynamic_modeling_thresholds.csv'), 'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');
guidance = readtable(fullfile(out_dir, 'builder_modeling_guidance_by_regime.csv'), 'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');

warning(warn_state.state, 'MATLAB:table:ModifiedAndSavedVarnames');

fprintf('Loaded output tables. Generating figure suite...\n');

%% Figure 1: Delay-vs-aero margin by regime with CI
f1 = figure('Name', '01 Delay minus Aero Margin');
hold on; grid on;
errorbar(regime.rate_center_deg_s, regime.mean_effect_margin_deg, ...
    regime.mean_effect_margin_deg - regime.margin_ci_low_deg, ...
    regime.margin_ci_high_deg - regime.mean_effect_margin_deg, ...
    'o-', 'LineWidth', 2, 'MarkerSize', 8);
yline(0, '--k', 'LineWidth', 1.5);
xlabel('Rate regime center (deg/s)');
ylabel('Delay minus aero residual (deg RMS)');
title('Matched Effect Margin by Regime (95% CI)');
saveas(f1, fullfile(fig_dir, '11_margin_by_regime.png'));
close(f1);

%% Figure 2: Stability mismatch by regime with CI
f2 = figure('Name', '02 Stability mismatch by regime');
hold on; grid on;
errorbar(regime.rate_center_deg_s, 100*regime.stability_mismatch_share, ...
    100*(regime.stability_mismatch_share - regime.mismatch_ci_low), ...
    100*(regime.mismatch_ci_high - regime.stability_mismatch_share), ...
    's-', 'LineWidth', 2, 'MarkerSize', 8);
yline(100*thresh.mismatch_threshold(1), '--r', 'LineWidth', 1.5);
xlabel('Rate regime center (deg/s)');
ylabel('Stability mismatch (%)');
title('Wrong Stability Decision Rate by Regime');
saveas(f2, fullfile(fig_dir, '12_stability_mismatch_by_regime.png'));
close(f2);

%% Figure 3: CP-ahead and static margin trend by regime
f3 = figure('Name', '03 CP and static margin trend');
tiledlayout(1, 2);

nexttile;
plot(regime.rate_center_deg_s, regime.mean_cp_ahead_share_truth, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
yline(thresh.cp_ahead_share_for_aero_model(1), '--r', 'LineWidth', 1.5);
yline(0.12, '--k', 'LineWidth', 1.0);
xlabel('Rate regime center (deg/s)');
ylabel('CP-ahead exposure share');
title('CP Ahead Exposure by Regime');

nexttile;
plot(regime.rate_center_deg_s, regime.mean_static_margin_cal_truth, 'd-', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
yline(0, '--k', 'LineWidth', 1.5);
xlabel('Rate regime center (deg/s)');
ylabel('Static margin (calibers)');
title('Static Margin by Regime');

saveas(f3, fullfile(fig_dir, '13_cp_static_margin_by_regime.png'));
close(f3);

%% Figure 4: Limitation regime composition (stacked)
f4 = figure('Name', '04 limitation composition');
Y = [atlas.delay_limited_share, atlas.aero_limited_share, atlas.mixed_interaction_share, atlas.model_mismatch_limited_share];
bar(atlas.rate_center_deg_s, Y, 'stacked');
grid on;
ylim([0 1]);
xlabel('Rate regime center (deg/s)');
ylabel('Share of cases');
title('Control Limitation Composition by Regime');
legend({'Delay-limited','Aero-limited','Mixed interaction','Model mismatch-limited'}, 'Location', 'eastoutside');
saveas(f4, fullfile(fig_dir, '14_limitation_composition.png'));
close(f4);

%% Figure 5: Delay heatmap of mismatch
valid = results(results.comparison_valid == 1, :);
tau_act_vals = unique(valid.tau_actuator_ms);
tau_sens_vals = unique(valid.tau_sensor_ms);

mismatch_map = nan(numel(tau_act_vals), numel(tau_sens_vals));
cp_map = nan(numel(tau_act_vals), numel(tau_sens_vals));

for i = 1:numel(tau_act_vals)
    for j = 1:numel(tau_sens_vals)
        idx = valid.tau_actuator_ms == tau_act_vals(i) & valid.tau_sensor_ms == tau_sens_vals(j);
        block = valid(idx, :);
        if ~isempty(block)
            mismatch_map(i, j) = mean(block.stability_mismatch);
            cp_map(i, j) = mean(block.cp_ahead_share_truth);
        end
    end
end

f5 = figure('Name', '05 delay mismatch heatmap');
imagesc(tau_sens_vals, tau_act_vals, mismatch_map);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('Sensor delay (ms)');
ylabel('Actuator delay (ms)');
title('Stability Mismatch Heatmap');
saveas(f5, fullfile(fig_dir, '15_mismatch_heatmap_delay.png'));
close(f5);

%% Figure 6: Delay heatmap of CP-ahead exposure
f6 = figure('Name', '06 cp ahead heatmap');
imagesc(tau_sens_vals, tau_act_vals, cp_map);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('Sensor delay (ms)');
ylabel('Actuator delay (ms)');
title('CP-Ahead Exposure Heatmap');
saveas(f6, fullfile(fig_dir, '16_cp_ahead_heatmap_delay.png'));
close(f6);

%% Figure 7: CP-ahead vs mismatch scatter
f7 = figure('Name', '07 cp ahead vs mismatch');
scatter(valid.cp_ahead_share_truth, double(valid.stability_mismatch), 22, valid.rate_center_deg_s, 'filled');
grid on;
colormap('turbo');
cb = colorbar;
cb.Label.String = 'Rate regime center (deg/s)';
xlabel('CP-ahead exposure share');
ylabel('Stability mismatch (0/1)');
title('Decision Risk vs CP-Forward Exposure');
saveas(f7, fullfile(fig_dir, '17_cp_ahead_vs_mismatch_scatter.png'));
close(f7);

%% Figure 8: Distribution summary histograms
f8 = figure('Name', '08 distribution histograms');
tiledlayout(1, 2);

nexttile;
histogram(valid.cp_ahead_share_truth, 14);
grid on;
xlabel('CP-ahead exposure share');
ylabel('Count');
title('Distribution: CP-Ahead Exposure');

nexttile;
histogram(valid.mean_static_margin_cal_truth, 14);
grid on;
xlabel('Mean static margin (cal)');
ylabel('Count');
title('Distribution: Static Margin');

saveas(f8, fullfile(fig_dir, '18_distribution_histograms.png'));
close(f8);

%% Figure 9: Builder guidance summary chart
f9 = figure('Name', '09 builder guidance');
gvars = guidance.Properties.VariableNames;
x_idx = find(contains(gvars, 'rate_center', 'IgnoreCase', true), 1, 'first');
cp_idx = find(contains(gvars, 'cp_ahead_share', 'IgnoreCase', true), 1, 'first');
mm_idx = find(contains(gvars, 'stability_mismatch_share', 'IgnoreCase', true), 1, 'first');

if isempty(x_idx), x_idx = 1; end
if isempty(cp_idx), cp_idx = min(3, numel(gvars)); end
if isempty(mm_idx), mm_idx = min(4, numel(gvars)); end

x_name = gvars{x_idx};
cp_name = gvars{cp_idx};
mm_name = gvars{mm_idx};

x_vals = to_numeric_col(guidance.(x_name));
cp_vals = to_numeric_col(guidance.(cp_name));
mm_vals = to_numeric_col(guidance.(mm_name));

bar(x_vals, [cp_vals, mm_vals], 'grouped');
grid on;
yline(thresh.cp_ahead_share_for_aero_model(1), '--r', 'LineWidth', 1.5);
yline(thresh.mismatch_threshold(1), '--k', 'LineWidth', 1.5);
xlabel('Rate regime center (deg/s)');
ylabel('Share');
title('Builder Guidance Inputs by Regime');
legend({'CP-ahead exposure','Stability mismatch','CP threshold','Mismatch threshold'}, 'Location', 'best');
saveas(f9, fullfile(fig_dir, '19_builder_guidance_inputs.png'));
close(f9);

%% Figure 10: STS montage from this suite
montage_files = {
    '11_margin_by_regime.png'
    '12_stability_mismatch_by_regime.png'
    '13_cp_static_margin_by_regime.png'
    '14_limitation_composition.png'
    '15_mismatch_heatmap_delay.png'
    '16_cp_ahead_heatmap_delay.png'
    '17_cp_ahead_vs_mismatch_scatter.png'
    '18_distribution_histograms.png'
    '19_builder_guidance_inputs.png'
};

tile_rows = 3;
tile_cols = 3;
tile_h = 380;
tile_w = 520;
canvas = uint8(255 * ones(tile_rows * tile_h, tile_cols * tile_w, 3));

for idx = 1:numel(montage_files)
    img_path = fullfile(fig_dir, montage_files{idx});
    if ~isfile(img_path)
        continue;
    end
    img = imread(img_path);
    img = imresize(img, [tile_h, tile_w]);
    r = floor((idx - 1) / tile_cols);
    c = mod((idx - 1), tile_cols);
    rr = (r * tile_h + 1):((r + 1) * tile_h);
    cc = (c * tile_w + 1):((c + 1) * tile_w);
    canvas(rr, cc, :) = img;
end

imwrite(canvas, fullfile(fig_dir, '99_sts_figure_montage.png'));

fprintf('Figure suite complete. Files saved to outputs/figures\n');

function out = to_numeric_col(v)
    if isnumeric(v)
        out = v;
    elseif isstring(v)
        out = str2double(v);
    elseif iscell(v)
        out = str2double(string(v));
    else
        out = str2double(string(v));
    end
end
