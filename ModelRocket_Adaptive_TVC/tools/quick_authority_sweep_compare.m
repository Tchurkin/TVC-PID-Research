function quick_authority_sweep_compare(root_dir)
%QUICK_AUTHORITY_SWEEP_COMPARE Compare controllers under authority-loss faults.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

data_dir = fullfile(root_dir, 'outputs', 'data');
graph_dir = fullfile(root_dir, 'outputs', 'graphs');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end

cfg = rocket_defaults();
seeds = 1:4;

% Post-fault control effectiveness scales (1.0 = no drop).
scales = [1.00 0.90 0.80 0.70 0.60 0.50 0.40 0.30 0.20];
ctrls = ["FIXED_LQR", "ADAPTIVE_KEFF_LQR", "JOINT_ADAPTIVE", "SLEW_ADAPTIVE"];
pretty = ["Fixed LQR", "keff-only", "Joint Adaptive", "slew-aware"];

fprintf('Authority scale sweep (HIGH_KEFF_FAULT variant, realistic model)\n');
fprintf('Scale  Drop%%   Fixed    keff-only   Joint    slew-only\n');

rows = [];

for scf = scales
    vals = nan(numel(ctrls), numel(seeds));
    for si = 1:numel(seeds)
        seed = seeds(si);

        sc = rocket_scenario("HIGH_KEFF_FAULT", cfg);
        sc.t_end = 10.0;
        sc.control_eff_scale_post = scf;

        % Keep disturbances modest so this isolates authority-loss effects.
        sc.disturbance_amp = 0.20;
        sc.disturb_scale_post = 1.5;
        sc.disturbance_freq_hz = 0.6;

        for ci = 1:numel(ctrls)
            out = simulate_case_realistic(ctrls(ci), sc, cfg, seed, struct());
            pm = out.time >= sc.fault_time;
            vals(ci, si) = rad2deg(rms(out.theta(pm)));
        end
    end

    m = mean(vals, 2, 'omitnan');
    s = std(vals, 0, 2, 'omitnan');
    for ci = 1:numel(ctrls)
        rows(end+1,:) = [1, scf, (1 - scf) * 100, ci, m(ci), s(ci), nan]; %#ok<AGROW>
    end
    fprintf('%4.2f   %5.1f   %7.2f   %9.2f   %7.2f   %9.2f\n', ...
        scf, (1 - scf) * 100, m(1), m(2), m(3), m(4));
end

fprintf('\nSevere stress spot-check (80%% authority drop, baseline fault wind):\n');
fprintf('Controller         mean RMS (deg)   std\n');

vals = nan(numel(ctrls), numel(seeds));
for si = 1:numel(seeds)
    sc = rocket_scenario("HIGH_KEFF_FAULT", cfg);
    sc.t_end = 10.0;
    sc.control_eff_scale_post = 0.20;
    sc.disturbance_amp = 0.30;
    sc.disturb_scale_post = 4.0;
    sc.disturbance_freq_hz = 0.30;

    for ci = 1:numel(ctrls)
        out = simulate_case_realistic(ctrls(ci), sc, cfg, seeds(si), struct());
        pm = out.time >= sc.fault_time;
        vals(ci, si) = rad2deg(rms(out.theta(pm)));
    end
end

for ci = 1:numel(ctrls)
    m = mean(vals(ci,:), 'omitnan');
    s = std(vals(ci,:), 'omitnan');
    rows(end+1,:) = [2, 0.20, 80.0, ci, m, s, 1.0]; %#ok<AGROW>
    fprintf('%-16s   %12.2f   %5.2f\n', ctrls(ci), m, s);
end

T = array2table(rows, 'VariableNames', ...
    {'test_case','control_eff_scale_post','authority_drop_pct','ctrl_idx','mean_rms_deg','std_rms_deg','stress_case'});
T.controller = strings(height(T),1);
for i = 1:height(T)
    T.controller(i) = pretty(T.ctrl_idx(i));
end

csv_path = fullfile(data_dir, 'authority_sweep_compare.csv');
writetable(T, csv_path);
fprintf('\nSaved data: %s\n', csv_path);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 520]);

ax1 = subplot(1,2,1);
hold(ax1, 'on'); grid(ax1, 'on');
colors = {[0.15 0.25 0.85], [0.85 0.45 0.10], [0.00 0.55 0.22], [0.60 0.20 0.20]};
for ci = 1:numel(ctrls)
    means = nan(size(scales));
    stds = nan(size(scales));
    for si = 1:numel(scales)
        mask = (T.test_case == 1) & (T.ctrl_idx == ci) & (abs(T.control_eff_scale_post - scales(si)) < 1e-12);
        means(si) = T.mean_rms_deg(mask);
        stds(si) = T.std_rms_deg(mask);
    end
    errorbar(ax1, 100 * (1 - scales), means, stds, 'LineWidth', 2.0, ...
        'Color', colors{ci}, 'Marker', 'o', 'MarkerFaceColor', colors{ci}, ...
        'DisplayName', pretty(ci));
end
xlabel(ax1, 'Authority drop (%)', 'FontWeight', 'bold');
ylabel(ax1, 'Post-fault RMS pitch (deg)', 'FontWeight', 'bold');
title(ax1, 'Moderate Disturbance (Isolated Authority Loss)', 'FontWeight', 'bold');
legend(ax1, 'Location', 'northwest');

ax2 = subplot(1,2,2);
hold(ax2, 'on'); grid(ax2, 'on');
mask2 = T.test_case == 2;
T2 = T(mask2,:);
[~, ord] = sort(T2.ctrl_idx);
T2 = T2(ord,:);
bar(ax2, categorical(T2.controller), T2.mean_rms_deg, 'FaceColor', [0.30 0.30 0.30]);
er = errorbar(ax2, 1:height(T2), T2.mean_rms_deg, T2.std_rms_deg, '.k');
er.LineWidth = 1.2;
ylabel(ax2, 'Post-fault RMS pitch (deg)', 'FontWeight', 'bold');
title(ax2, 'Severe Stress (80% Drop + Baseline Fault Wind)', 'FontWeight', 'bold');

sgtitle({"Authority-Loss Sensitivity Comparison"; sprintf('Mean +/- 1sigma over %d seeds', numel(seeds))}, 'FontWeight', 'bold');

png_path = fullfile(graph_dir, 'authority_sweep_compare.png');
exportgraphics(fig, png_path, 'Resolution', 220);
close(fig);
fprintf('Saved figure: %s\n', png_path);
end
