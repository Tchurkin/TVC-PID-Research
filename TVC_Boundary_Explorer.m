%% ========================================================================
%% TVC BOUNDARY EXPLORER - Decision Maps and Eureka Visualization
%% When does aerodynamic stability dominate over delay?
%% What can you afford given your hardware constraints?
%% ========================================================================

clear; close all; clc;

fprintf('\n========== TVC DECISION BOUNDARY ANALYSIS ==========\n\n');

%% Read simulation outputs
out_dir = fullfile(pwd, 'outputs');
extreme_tbl = readtable(fullfile(out_dir, 'extreme_regime_exploration.csv'), ...
    'Delimiter', ',', 'VariableNamingRule', 'preserve', 'TextType', 'string');

%% Define control-loop parameters for normalization
f_bw = 3.0;  % Hz, control bandwidth for phase-lag calculation

%% Extract and compute normalized metrics
rate = table2array(extreme_tbl(:, 'rate_center_deg_s'));
tau_act = table2array(extreme_tbl(:, 'tau_actuator_ms'));
tau_sens = table2array(extreme_tbl(:, 'tau_sensor_ms'));
proc_delay = table2array(extreme_tbl(:, 'processing_delay_ms'));
aero_scale = table2array(extreme_tbl(:, 'aero_scale'));
cp_ahead_binary = table2array(extreme_tbl(:, 'cp_ahead_share_truth'));
effect_margin = table2array(extreme_tbl(:, 'effect_margin_deg'));
stability_mismatch = table2array(extreme_tbl(:, 'stability_mismatch'));

% Total delay as phase lag at control bandwidth
phi_act = atan(2*pi*f_bw.*tau_act/1000) * 180/pi;
phi_sens = atan(2*pi*f_bw.*tau_sens/1000) * 180/pi;
phi_proc = 180 * f_bw * (proc_delay/1000);
total_phase_lag = phi_act + phi_sens + phi_proc;

% Aero phase lag (proportional to aero_scale deviation from baseline)
aero_phase_equiv = 10 * (aero_scale - 1.0);  % Arbitrary but consistent scaling

% Normalized "delay pressure" vs "aero pressure"
delay_pressure = total_phase_lag;
aero_pressure = abs(aero_phase_equiv);
relative_aero_dominance = (aero_pressure - delay_pressure) ./ (aero_pressure + delay_pressure + eps);

% Aero dominates when effect_margin < 0 OR relative_aero_dominance > threshold
aero_dominant = effect_margin < 0;

fprintf('Data summary:\n');
fprintf('  Total rows: %d\n', height(extreme_tbl));
fprintf('  Aero-dominant rows: %d (%.1f%%)\n', sum(aero_dominant), 100*mean(aero_dominant));
fprintf('  CP-ahead cases: %d (%.1f%% of total)\n', sum(cp_ahead_binary), 100*mean(cp_ahead_binary));
fprintf('  CP-ahead cases that are aero-dominant: %d (%.1f%% of CP-ahead)\n', ...
    sum(aero_dominant & cp_ahead_binary), 100*mean(aero_dominant(cp_ahead_binary>0.5)));
fprintf('\n');

%% ========================================================================
%% EUREKA GRAPH 1: Aero vs Delay Pressure Space
%% Shows the "battle" between mechanisms
%% ========================================================================

fig1 = figure('Name', 'Aero vs Delay Pressure Space');
tiledlayout(2, 2);

nexttile;
scatter(delay_pressure, aero_pressure, 20, rate, 'filled', 'MarkerFaceAlpha', 0.6);
colorbar;
hold on;
plot([0, 100], [0, 100], 'k--', 'LineWidth', 2, 'DisplayName', 'Equal pressure line');
xlabel('Total Phase Lag from Delays (deg)');
ylabel('Effective Aero Phase Lag (deg)');
title('Aero vs Delay: Who Wins?');
grid on;
legend;
set(gca, 'XScale', 'log', 'YScale', 'log');

nexttile;
scatter(delay_pressure(aero_dominant), aero_pressure(aero_dominant), 20, rate(aero_dominant), 'filled', ...
    'MarkerFaceAlpha', 0.7, 'DisplayName', 'Aero dominant');
hold on;
scatter(delay_pressure(~aero_dominant), aero_pressure(~aero_dominant), 20, rate(~aero_dominant), ...
    's', 'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', 'Delay dominant');
plot([0, 100], [0, 100], 'k--', 'LineWidth', 2);
xlabel('Total Phase Lag from Delays (deg)');
ylabel('Effective Aero Phase Lag (deg)');
title('Colored by Dominance (circle=aero, square=delay)');
grid on;
legend;
set(gca, 'XScale', 'log', 'YScale', 'log');

% Histogram of dominance
nexttile;
histogram(relative_aero_dominance, 30, 'FaceColor', 'blue', 'FaceAlpha', 0.7);
hold on;
xline(0, 'r-', 'LineWidth', 2, 'DisplayName', 'Equal pressure');
xlabel('Relative Aero Dominance (-1=pure delay, +1=pure aero)');
ylabel('Count');
title('Distribution of Mechanism Dominance');
legend;

% Regime separation
nexttile;
for r = unique(rate)'
    idx = rate == r & aero_dominant;
    plot(delay_pressure(idx), aero_pressure(idx), 'o', 'MarkerSize', 8, ...
        'DisplayName', sprintf('Rate %.0f°/s', r));
end
plot([0, 100], [0, 100], 'k--', 'LineWidth', 2);
xlabel('Total Phase Lag (deg)');
ylabel('Aero Phase Lag (deg)');
title('Aero-Dominant Cases by Regime (only)');
grid on;
legend;
set(gca, 'XScale', 'log', 'YScale', 'log');

save_plot_if_enabled(fig1, fullfile(out_dir, 'figures', '20_aero_vs_delay_pressure.png'));

%% ========================================================================
%% EUREKA GRAPH 2: Decision Map - Rate vs Delay Budget
%% Given your rate regime, how much delay can you afford?
%% ========================================================================

fig2 = figure('Name', 'Decision Map: Rate vs Delay Budget');

% For each rate, for each delay level, what's the aero-dominant share?
rates_unique = unique(rate);
delay_percentiles = prctile(total_phase_lag, [10, 25, 50, 75, 90]);

aero_share_by_rate_and_delay = zeros(numel(rates_unique), numel(delay_percentiles));
for i = 1:numel(rates_unique)
    r = rates_unique(i);
    for j = 1:numel(delay_percentiles)
        idx = (rate == r) & (total_phase_lag <= delay_percentiles(j));
        if sum(idx) > 0
            aero_share_by_rate_and_delay(i, j) = mean(aero_dominant(idx));
        end
    end
end

tiledlayout(1, 2);
nexttile;
plot(delay_percentiles, aero_share_by_rate_and_delay', 'o-', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
yline(0.50, 'k--', 'LineWidth', 1.5, 'DisplayName', '50% aero-dominant threshold');
xlabel('Total Delay (phase lag at 3 Hz, degrees)');
ylabel('Share of Cases Where Aero Dominates');
title('Decision Map: How Much Delay Can You Afford?');
grid on;
legend(arrayfun(@(x) sprintf('Rate %.0f°/s', x), rates_unique, 'UniformOutput', false));
ylim([0, 1]);

% Trade-off zone visualization
nexttile;
data_for_heatmap = zeros(numel(rates_unique), 4);
for i = 1:numel(rates_unique)
    r = rates_unique(i);
    idx_r = rate == r;
    % For each rate, show the delay range where aero share is 25%, 50%, 75%
    delay_at_r = total_phase_lag(idx_r);
    aero_at_r = aero_dominant(idx_r);
    
    for pct = 1:3
        target_share = 0.25 * pct;
        [~, closest_idx] = min(abs(aero_share_by_rate_and_delay(i, :) - target_share));
        data_for_heatmap(i, pct) = delay_percentiles(closest_idx);
    end
    data_for_heatmap(i, 4) = sum(aero_at_r) / numel(aero_at_r);
end

imagesc(1:4, 1:numel(rates_unique), data_for_heatmap);
set(gca, 'YTick', 1:numel(rates_unique), 'YTickLabel', rates_unique);
set(gca, 'XTick', 1:4, 'XTickLabel', {'25% aero', '50% aero', '75% aero', 'All aero%'});
colorbar;
title('Delay Budgets (deg phase lag) by Regime and Aero Threshold');
ylabel('Rate Regime (deg/s)');

save_plot_if_enabled(fig2, fullfile(out_dir, 'figures', '21_decision_map_rate_vs_delay.png'));

%% ========================================================================
%% EUREKA GRAPH 3: CP-Forward Effect on Aero Dominance
%% Shows why CP placement matters
%% ========================================================================

fig3 = figure('Name', 'CP-Forward Impact on Aero Dominance');
tiledlayout(2, 2);

% Panel 1: Aero-dominant share by rate and CP position
nexttile;
for r = rates_unique'
    cp_behind = (rate == r) & (cp_ahead_binary == 0);
    cp_ahead = (rate == r) & (cp_ahead_binary == 1);
    pct_behind = mean(aero_dominant(cp_behind));
    pct_ahead = mean(aero_dominant(cp_ahead));
    plot([0, 1], [pct_behind, pct_ahead], 'o-', 'LineWidth', 2, 'MarkerSize', 10, ...
        'DisplayName', sprintf('%.0f°/s', r));
end
xlabel('CP Position (0=behind CG, 1=ahead of CG)');
ylabel('Aero-Dominant Case Share');
title('How Much Does CP Forward Placement Matter?');
grid on;
legend;
ylim([0, 1]);

% Panel 2: Stability mismatch vs CP position
nexttile;
scatter(cp_ahead_binary, stability_mismatch, 20, rate, 'filled', 'MarkerFaceAlpha', 0.6);
colorbar;
xlabel('CP Ahead of CG (0=behind, 1=ahead)');
ylabel('Stability Mismatch (wrong prediction)');
title('Prediction Risk vs CP Forward Exposure');
ylim([0, 1]);
grid on;

% Panel 3: Effect margin distribution by CP position
nexttile;
behind_margins = effect_margin(cp_ahead_binary==0);
ahead_margins = effect_margin(cp_ahead_binary==1);
scatter(0.9 + 0.02*randn(size(behind_margins)), behind_margins, 20, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
scatter(1.1 + 0.02*randn(size(ahead_margins)), ahead_margins, 20, 'r', 'filled', 'MarkerFaceAlpha', 0.3);
plot([0.85, 0.95], [median(behind_margins), median(behind_margins)], 'b-', 'LineWidth', 3, 'DisplayName', 'Median (behind)');
plot([1.05, 1.15], [median(ahead_margins), median(ahead_margins)], 'r-', 'LineWidth', 3, 'DisplayName', 'Median (ahead)');
set(gca, 'XTick', [0.9, 1.1], 'XTickLabel', {'CP Behind CG', 'CP Ahead of CG'});
ylabel('Effect Margin (delay - aero residual)');
title('Delay Advantage: Behind vs Ahead');
grid on;
legend;

% Panel 4: Rate-dependent sensitivity to CP placement
nexttile;
for r = rates_unique'
    cp_behind_aero = (rate == r) & (cp_ahead_binary == 0) & aero_dominant;
    cp_ahead_aero = (rate == r) & (cp_ahead_binary == 1) & aero_dominant;
    behind_share = sum(cp_behind_aero) / sum((rate == r) & (cp_ahead_binary == 0));
    ahead_share = sum(cp_ahead_aero) / sum((rate == r) & (cp_ahead_binary == 1));
    if ~isnan(behind_share) && ~isnan(ahead_share)
        plot([0, 1], [behind_share, ahead_share], 'o-', 'LineWidth', 2.5, 'MarkerSize', 10);
    end
end
xlabel('CP Position');
ylabel('Aero-Dominant Share at Each Rate');
title('Rate-Dependent Impact of CP Forward (gradient = sensitivity)');
grid on;
legend(arrayfun(@(x) sprintf('%.0f°/s', x), rates_unique, 'UniformOutput', false), 'Location', 'NorthWest');
ylim([0, 1]);

save_plot_if_enabled(fig3, fullfile(out_dir, 'figures', '22_cp_forward_impact.png'));

%% ========================================================================
%% SUMMARY TABLE: Where Is Aero Dominant?
%% ========================================================================

summary_zones = table;
summary_zones.Description = ["Low Rate, Long Delay, CP Behind"; ...
                             "Low Rate, Long Delay, CP Ahead"; ...
                             "High Rate, Short Delay, CP Behind"; ...
                             "High Rate, Short Delay, CP Ahead"; ...
                             "Overall (all rates)"];

for i = 1:height(summary_zones)
    if i == 1
        idx = (rate <= 80) & (total_phase_lag >= 50) & (cp_ahead_binary == 0);
    elseif i == 2
        idx = (rate <= 80) & (total_phase_lag >= 50) & (cp_ahead_binary == 1);
    elseif i == 3
        idx = (rate >= 250) & (total_phase_lag <= 20) & (cp_ahead_binary == 0);
    elseif i == 4
        idx = (rate >= 250) & (total_phase_lag <= 20) & (cp_ahead_binary == 1);
    else
        idx = ones(size(rate), 'logical');
    end
    
    if sum(idx) > 0
        summary_zones.Count(i) = sum(idx);
        summary_zones.AeroDominantShare(i) = mean(aero_dominant(idx));
        summary_zones.MeanStabilityMismatch(i) = mean(stability_mismatch(idx));
        summary_zones.MeanEffectMargin(i) = mean(effect_margin(idx));
    else
        summary_zones.Count(i) = 0;
        summary_zones.AeroDominantShare(i) = nan;
        summary_zones.MeanStabilityMismatch(i) = nan;
        summary_zones.MeanEffectMargin(i) = nan;
    end
end

writetable(summary_zones, fullfile(out_dir, 'boundary_zone_summary.csv'));

fprintf('\nBoundary Zone Summary:\n');
disp(summary_zones);

fprintf('\nFigures saved:\n');
fprintf('  - figures/20_aero_vs_delay_pressure.png\n');
fprintf('  - figures/21_decision_map_rate_vs_delay.png\n');
fprintf('  - figures/22_cp_forward_impact.png\n');
fprintf('  - boundary_zone_summary.csv\n');
fprintf('\nDone.\n\n');

%% ========================================================================
%% LOCAL FUNCTION
%% ========================================================================

function save_plot_if_enabled(fig, filepath)
    [dir, ~, ~] = fileparts(filepath);
    if ~exist(dir, 'dir')
        mkdir(dir);
    end
    exportgraphics(fig, filepath, 'Resolution', 150);
    close(fig);
end
