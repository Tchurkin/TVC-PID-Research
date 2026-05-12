%% generate_sts_gold_graphs_adaptive.m
% Publication-style figures for adaptive ADRC direction.

root_dir = fileparts(fileparts(mfilename('fullpath')));
out_dir = fullfile(root_dir, 'outputs');
gold_dir = fullfile(out_dir, 'sts_gold_adaptive');
if ~exist(gold_dir, 'dir'), mkdir(gold_dir); end

sweep_file = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'sweep_results_adaptive.csv');
stress_file = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'industry_stress_metrics_adaptive.csv');
deg_file = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'degradation_results_adaptive.csv');
trace_file = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'degradation_trace_adaptive.mat');
score_file = fullfile(out_dir, 'sts_scoreboard_adaptive.csv');
council_file = fullfile(out_dir, 'council_direction_B_adaptive_assessment.csv');

must_exist = {sweep_file, stress_file, deg_file, trace_file, score_file, council_file};
for i = 1:numel(must_exist)
    if ~exist(must_exist{i}, 'file')
        error('Missing required file: %s', must_exist{i});
    end
end

S = readtable(sweep_file, 'TextType', 'string');
M = readtable(stress_file, 'TextType', 'string');
D = readtable(deg_file, 'TextType', 'string');
SB = readtable(score_file, 'TextType', 'string');
CB = readtable(council_file, 'TextType', 'string');
Tpack = load(trace_file);
T = Tpack.trace_pack;

wind_order = [5 8 10 12 15 20];
wind_labels = ["5 m/s", "8 m/s", "10 m/s", "12 m/s", "15 m/s", "20 m/s"];

l1_peak = lookup_metric(S, wind_order, "L1", "peak_ct_m");
pid_peak = lookup_metric(S, wind_order, "PID_CASCADE", "peak_ct_m");
fix_peak = lookup_metric(S, wind_order, "ADRC_FIXED", "peak_ct_m");
adp_peak = lookup_metric(S, wind_order, "ADRC_ADAPTIVE", "peak_ct_m");

l1_rms = lookup_metric(S, wind_order, "L1", "rms_ct_m");
pid_rms = lookup_metric(S, wind_order, "PID_CASCADE", "rms_ct_m");
fix_rms = lookup_metric(S, wind_order, "ADRC_FIXED", "rms_ct_m");
adp_rms = lookup_metric(S, wind_order, "ADRC_ADAPTIVE", "rms_ct_m");

fig1 = figure('Visible', 'off', 'Position', [100 100 1200 460], 'Color', 'w');
subplot(1,2,1);
plot(wind_order, l1_peak, '-o', 'LineWidth', 2.0, 'Color', [0.20 0.45 0.80]); hold on;
plot(wind_order, pid_peak, '-s', 'LineWidth', 2.0, 'Color', [0.45 0.45 0.45]);
plot(wind_order, fix_peak, '-d', 'LineWidth', 2.0, 'Color', [0.20 0.65 0.30]);
plot(wind_order, adp_peak, '-^', 'LineWidth', 2.4, 'Color', [0.86 0.24 0.15]);
yline(20, '--', 'Guardrail 20m', 'Color', [0.35 0.35 0.35]);
set(gca, 'XTick', wind_order, 'XTickLabel', wind_labels, 'FontSize', 11);
grid on;
xlabel('Mean crosswind'); ylabel('Peak cross-track (m)');
title('Peak error vs wind');
legend({'L1','PID cascade','ADRC fixed','ADRC adaptive'}, 'Location', 'northwest');

subplot(1,2,2);
plot(wind_order, l1_rms, '-o', 'LineWidth', 2.0, 'Color', [0.20 0.45 0.80]); hold on;
plot(wind_order, pid_rms, '-s', 'LineWidth', 2.0, 'Color', [0.45 0.45 0.45]);
plot(wind_order, fix_rms, '-d', 'LineWidth', 2.0, 'Color', [0.20 0.65 0.30]);
plot(wind_order, adp_rms, '-^', 'LineWidth', 2.4, 'Color', [0.86 0.24 0.15]);
yline(5, '--', 'Target 5m RMS', 'Color', [0.35 0.35 0.35]);
set(gca, 'XTick', wind_order, 'XTickLabel', wind_labels, 'FontSize', 11);
grid on;
xlabel('Mean crosswind'); ylabel('RMS cross-track (m)');
title('RMS error vs wind');
legend({'L1','PID cascade','ADRC fixed','ADRC adaptive'}, 'Location', 'northwest');

sgtitle('STS-Gold Adaptive Fig 1: Nominal wind benchmark');
exportgraphics(fig1, fullfile(gold_dir, 'adaptive_gold_01_nominal_vs_wind.png'), 'Resolution', 300);
close(fig1);

fig2 = figure('Visible', 'off', 'Position', [120 120 1200 460], 'Color', 'w');
subplot(1,2,1);
bar(categorical(M.wind_label), [M.peak_ratio_vs_adrcfixed M.rms_ratio_vs_adrcfixed], 'grouped');
grid on; yline(1.0, '--', 'Parity');
ylabel('Adaptive / Fixed ADRC ratio');
title('Nominal ratio vs fixed ADRC');
legend({'Peak ratio','RMS ratio'}, 'Location', 'northwest');

subplot(1,2,2);
bar(categorical(M.wind_label), [double(M.fail_reduction_vs_adrcfixed) M.in5_gain_vs_adrcfixed_pct], 'grouped');
grid on;
ylabel('Improvement vs fixed ADRC');
title('Reliability and in-band gains');
legend({'Fail reduction','In5 gain (% points)'}, 'Location', 'northwest');

sgtitle('STS-Gold Adaptive Fig 2: Relative gains under stress');
exportgraphics(fig2, fullfile(gold_dir, 'adaptive_gold_02_relative_gains.png'), 'Resolution', 300);
close(fig2);

fig3 = figure('Visible', 'off', 'Position', [130 130 1250 500], 'Color', 'w');
plot(T.t, T.L1.y, 'LineWidth', 1.8, 'Color', [0.20 0.45 0.80]); hold on;
plot(T.t, T.PID_CASCADE.y, 'LineWidth', 1.8, 'Color', [0.45 0.45 0.45]);
plot(T.t, T.ADRC_FIXED.y, 'LineWidth', 2.0, 'Color', [0.20 0.65 0.30]);
plot(T.t, T.ADRC_ADAPTIVE.y, 'LineWidth', 2.4, 'Color', [0.86 0.24 0.15]);
xline(T.degrade_time, '--', 'Actuator degradation onset', 'Color', [0.3 0.3 0.3]);
yline(5, ':', '5m band', 'Color', [0.5 0.5 0.5]);
yline(-5, ':', 'Color', [0.5 0.5 0.5]);
grid on;
xlabel('Time (s)'); ylabel('Cross-track y (m)');
title('Degradation scenario: adaptive recovery vs fixed-gain collapse margin');
legend({'L1','PID cascade','ADRC fixed','ADRC adaptive'}, 'Location', 'northwest');
exportgraphics(fig3, fullfile(gold_dir, 'adaptive_gold_03_degradation_trace.png'), 'Resolution', 300);
close(fig3);

fig4 = figure('Visible', 'off', 'Position', [150 150 1200 460], 'Color', 'w');
subplot(1,2,1);
plot(T.t, T.ADRC_ADAPTIVE.gain_est, 'LineWidth', 2.2, 'Color', [0.86 0.24 0.15]); hold on;
yline(1.0, '--', 'Nominal gain', 'Color', [0.4 0.4 0.4]);
yline(T.cfg.degrade_gain_scale, '--', 'Degraded gain', 'Color', [0.2 0.2 0.2]);
xline(T.degrade_time, '--', 'Fault');
grid on; xlabel('Time (s)'); ylabel('Estimated gain');
title('RLS gain estimate convergence');

subplot(1,2,2);
plot(T.t, T.ADRC_ADAPTIVE.omega_o, 'LineWidth', 2.2, 'Color', [0.20 0.65 0.30]); hold on;
xline(T.degrade_time, '--', 'Fault');
grid on; xlabel('Time (s)'); ylabel('Observer bandwidth \omega_o (rad/s)');
title('Adaptive observer retuning over time');

sgtitle('STS-Gold Adaptive Fig 4: Identification and adaptation dynamics');
exportgraphics(fig4, fullfile(gold_dir, 'adaptive_gold_04_rls_and_bandwidth.png'), 'Resolution', 300);
close(fig4);

fig5 = figure('Visible', 'off', 'Position', [160 160 1200 460], 'Color', 'w');
controllers = string(D.controller);
bar(categorical(controllers), [D.recovery_time_s D.post_fault_rms], 'grouped');
grid on;
ylabel('Value');
title('Post-fault recovery metrics');
legend({'Recovery time (s)','Post-fault RMS (m)'}, 'Location', 'northwest');
exportgraphics(fig5, fullfile(gold_dir, 'adaptive_gold_05_recovery_metrics.png'), 'Resolution', 300);
close(fig5);

% Combined submission-ready plate (2x3) with A-F labels.
fig6 = figure('Visible', 'off', 'Position', [100 100 1700 1100], 'Color', 'w');
tl = tiledlayout(fig6, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
tile_files = {
    'adaptive_gold_01_nominal_vs_wind.png', ...
    'adaptive_gold_02_relative_gains.png', ...
    'adaptive_gold_03_degradation_trace.png', ...
    'adaptive_gold_04_rls_and_bandwidth.png', ...
    'adaptive_gold_05_recovery_metrics.png', ...
    'adaptive_gold_05_recovery_metrics.png'};
tile_labels = {'A','B','C','D','E','F'};

for ti = 1:numel(tile_files)
    ax = nexttile(tl, ti);
    img = imread(fullfile(gold_dir, tile_files{ti}));
    image(ax, img);
    axis(ax, 'image');
    axis(ax, 'off');
    text(ax, 0.02, 0.96, tile_labels{ti}, ...
        'Units', 'normalized', ...
        'FontSize', 20, ...
        'FontWeight', 'bold', ...
        'Color', [0.10 0.10 0.10], ...
        'BackgroundColor', [1 1 1], ...
        'Margin', 2, ...
        'VerticalAlignment', 'top');
end

title(tl, 'Adaptive Direction B STS Figure Plate');
exportgraphics(fig6, fullfile(gold_dir, 'adaptive_gold_plate_2x3.png'), 'Resolution', 300);
close(fig6);

summary_file = fullfile(gold_dir, 'adaptive_gold_summary.txt');
fid = fopen(summary_file, 'w');
fprintf(fid, 'Adaptive Direction B STS-Gold Summary\n');
fprintf(fid, 'Council score: %.2f\n', CB.council_score(1));
fprintf(fid, 'Verdict: %s\n', CB.verdict(1));
fprintf(fid, 'Decision: %s\n', CB.main_decision(1));
fprintf(fid, 'Overall adaptive STS score: %.2f (%s)\n', SB.overall_score(1), SB.readiness(1));
fprintf(fid, 'B score: %.2f | Adaptive score: %.2f\n', SB.B_score(1), SB.adaptive_score(1));
fprintf(fid, 'High-wind Adaptive/Fixed peak ratio: %.3f\n', mean(M.peak_ratio_vs_adrcfixed(ismember(M.wind_label, ["10mps","12mps","15mps"]))));
fprintf(fid, 'High-wind Adaptive/Fixed RMS ratio: %.3f\n', mean(M.rms_ratio_vs_adrcfixed(ismember(M.wind_label, ["10mps","12mps","15mps"]))));
fprintf(fid, 'Figure plate: adaptive_gold_plate_2x3.png\n');
fix_row = D(strcmp(D.controller, "ADRC_FIXED"), :);
ad_row = D(strcmp(D.controller, "ADRC_ADAPTIVE"), :);
if ~isempty(fix_row) && ~isempty(ad_row)
    rec = (fix_row.post_fault_rms - ad_row.post_fault_rms) / max(1e-6, fix_row.post_fault_rms);
    fprintf(fid, 'Recovery ratio (post-fault RMS improvement): %.3f\n', rec);
end
fclose(fid);

fprintf('Saved adaptive STS-gold graphs in: %s\n', gold_dir);

function vals = lookup_metric(T, wind_order, controller_name, metric_name)
vals = nan(size(wind_order));
for i = 1:numel(wind_order)
    mask = (T.wind_mps == wind_order(i)) & strcmp(T.controller, controller_name);
    if any(mask)
        vals(i) = T.(metric_name)(find(mask, 1, 'first'));
    end
end
end
