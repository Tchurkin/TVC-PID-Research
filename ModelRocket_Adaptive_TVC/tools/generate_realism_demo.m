function generate_realism_demo(root_dir)
%GENERATE_REALISM_DEMO  Stress test all 5 controllers under realistic
%  sensor noise, servo nonlinearities, and Dryden gusts.
%
%  Three scenes:
%   (1) Single-trace headline run on slew-degradation with full realism on
%   (2) Monte-Carlo over (controller, scenario) at "flight realism" preset
%   (3) Sensitivity sweep: vary gyro_noise_std and gust_std, plot how
%       JOINT_ADAPTIVE's metrics degrade

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
data_dir  = fullfile(root_dir, 'outputs', 'data');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end
if ~exist(data_dir,  'dir'), mkdir(data_dir);  end

cfg = rocket_defaults();

ctrl_names = ["FIXED_LQR", "ADAPTIVE_KEFF_LQR", "SIGMA_MRAC", "PCH_LQR", "JOINT_ADAPTIVE"];
pretty     = ["Fixed LQR", "keff-RLS", "\sigma-mod MRAC", "PCH", "Joint Adaptive"];
colors     = {[0.15 0.25 0.85], [0.85 0.45 0.10], [0.55 0.27 0.55], [0.40 0.40 0.40], [0.0 0.55 0.22]};
lstyle     = {'--', ':', '-.', ':', '-'};
lwidth     = [1.6, 1.6, 1.8, 1.8, 2.6];

% ============== SCENE 1: headline trace ==============================
realism = struct();   % all defaults: full noise on
cfg.plant.theta0 = deg2rad(5);
sc = rocket_scenario("SLEW_DEGRADATION", cfg);
sc.disturbance_amp     = 3.0;
sc.disturbance_freq_hz = 1.5;
sc.t_end               = 10.0;
seed = 7;

sims = struct();
for c = 1:numel(ctrl_names)
    sims(c).out = simulate_case_realistic(ctrl_names(c), sc, cfg, seed, realism);
end

slew_pre  = cfg.plant.slew_max;
slew_post = cfg.plant.slew_max * sc.slew_scale_post;

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1340 760]);

ax1 = subplot(2,1,1);
hold(ax1,'on'); grid(ax1,'on');
for c = 1:numel(ctrl_names)
    th = rad2deg(sims(c).out.theta);
    th_clip = max(-50, min(50, th));
    plot(ax1, sims(c).out.time, th_clip, ...
        'Color', colors{c}, 'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), ...
        'DisplayName', pretty(c));
end
xline(ax1, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, 'HandleVisibility','off');
yline(ax1, 0, ':k', 'LineWidth', 1.0, 'Alpha', 0.4, 'HandleVisibility','off');
ylabel(ax1, 'TRUE pitch (deg, clipped \pm50)', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax1, [-50 50]);
legend(ax1, 'Location', 'northwest', 'FontSize', 10);
title(ax1, sprintf('Pitch (truth) under realistic sensors+actuator   |   slew %.1f \\rightarrow %.1f units/s', ...
    slew_pre, slew_post), 'FontSize', 12, 'FontWeight', 'bold');
set(ax1, 'FontSize', 10);

% Show what JOINT_ADAPTIVE's slew estimator reports
ax2 = subplot(2,1,2);
hold(ax2,'on'); grid(ax2,'on');
ja = sims(end).out;
yyaxis(ax2,'left');
plot(ax2, ja.time, ja.slew_est, '-', 'Color', colors{end}, 'LineWidth', 2.4, ...
    'DisplayName', 'slew_{est} (joint)');
plot(ax2, [0 sc.fault_time], [slew_pre slew_pre], 'k--', 'LineWidth', 1.2, ...
    'DisplayName', 'true slew');
plot(ax2, [sc.fault_time sc.t_end], [slew_post slew_post], 'k--', 'LineWidth', 1.2, ...
    'HandleVisibility','off');
ylabel(ax2, 'slew_{est} (units/s)', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax2, [0 1.3*slew_pre]);

yyaxis(ax2,'right');
plot(ax2, ja.time, ja.gain_scale, '-.', 'Color', colors{end}, 'LineWidth', 2.0, ...
    'DisplayName', 'gain scale (joint)');
ylabel(ax2, 'gain scale', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax2, [0 1.5]);
xline(ax2, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, 'HandleVisibility','off');
xlabel(ax2, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
legend(ax2, 'Location', 'east', 'FontSize', 10);
title(ax2, 'Joint estimator behavior with noisy sensors', ...
    'FontSize', 12, 'FontWeight', 'bold');
set(ax2, 'FontSize', 10);

sgtitle({'Realism stress test: gyro noise + bias drift + ADC quantization + servo deadband+backlash + Dryden gust + 1-step latency'}, ...
    'FontSize', 12, 'FontWeight', 'bold');

out_path = fullfile(graph_dir, 'realism_headline.png');
exportgraphics(fig, out_path, 'Resolution', 220);
close(fig);

fprintf('\n=== REALISM HEADLINE (single seed=%d, slew degradation) ===\n', seed);
for c = 1:numel(ctrl_names)
    pm = sims(c).out.time >= sc.fault_time;
    rms_post = rad2deg(rms(sims(c).out.theta(pm)));
    pk_post  = rad2deg(max(abs(sims(c).out.theta(pm))));
    fprintf('  %-22s   RMS_post = %8.2f deg   peak = %8.2f deg\n', ...
        char(pretty(c)), rms_post, pk_post);
end
fprintf('Saved: %s\n', out_path);

% ============== SCENE 2: Monte Carlo at "flight realism" =============
n_seeds = 30;
scenarios = ["SLEW_DEGRADATION", "HIGH_KEFF_FAULT"];
sc_pretty = ["Slew Degradation", "Motor Drop"];

fprintf('\n=== REALISM MONTE CARLO (%d seeds) ===\n', n_seeds);
mc_results = [];   % [s_idx, c_idx, seed, rms_pre, rms_post, pk_post]
for s = 1:numel(scenarios)
    cfg2 = cfg;
    cfg2.plant.theta0 = deg2rad(5);
    sc = rocket_scenario(scenarios(s), cfg2);
    sc.t_end = 10.0;
    if scenarios(s) == "SLEW_DEGRADATION"
        sc.disturbance_amp = 3.0;
        sc.disturbance_freq_hz = 1.5;
    end
    for c = 1:numel(ctrl_names)
        for seed_i = 1:n_seeds
            out = simulate_case_realistic(ctrl_names(c), sc, cfg2, seed_i, realism);
            pre = out.time < sc.fault_time & out.time > 1.0;
            pm  = out.time >= sc.fault_time;
            rms_pre  = rad2deg(rms(out.theta(pre)));
            rms_post = rad2deg(rms(out.theta(pm)));
            pk_post  = rad2deg(max(abs(out.theta(pm))));
            mc_results(end+1, :) = [s, c, seed_i, rms_pre, rms_post, pk_post]; %#ok<AGROW>
        end
    end
end

% Save CSV
T = array2table(mc_results, 'VariableNames', ...
    {'scenario_idx','ctrl_idx','seed','rms_pre','rms_post','pk_post'});
csv_path = fullfile(data_dir, 'realism_montecarlo.csv');
writetable(T, csv_path);
fprintf('Saved data: %s\n', csv_path);

% Print summary table
fprintf('\n  %-22s', 'Controller');
for s = 1:numel(scenarios), fprintf('   %-30s', sprintf('%s (mean +/- std post)', sc_pretty(s))); end
fprintf('\n');
for c = 1:numel(ctrl_names)
    fprintf('  %-22s', char(pretty(c)));
    for s = 1:numel(scenarios)
        mask = mc_results(:,1)==s & mc_results(:,2)==c;
        vals = mc_results(mask, 5);
        fprintf('   %8.2f +/- %-7.2f deg     ', mean(vals), std(vals));
    end
    fprintf('\n');
end

% ============== SCENE 3: noise sensitivity sweep =====================
fprintf('\n=== NOISE SENSITIVITY SWEEP (Joint vs Sigma-MRAC vs PCH) ===\n');
noise_levels = [0.005, 0.010, 0.015, 0.025, 0.040, 0.060];   % rad/s gyro sigma
gust_levels  = [0.10, 0.20, 0.30, 0.50, 0.80];               % rad/s^2 gust sigma
n_seeds_sens = 12;

sens_ctrls = ["SIGMA_MRAC", "PCH_LQR", "JOINT_ADAPTIVE"];
sens_pretty = ["\sigma-mod MRAC", "PCH", "Joint Adaptive"];
sens_colors = {[0.55 0.27 0.55], [0.40 0.40 0.40], [0.0 0.55 0.22]};

% Sub-sweep A: gyro noise
sens_a = nan(numel(sens_ctrls), numel(noise_levels), n_seeds_sens);
for ni = 1:numel(noise_levels)
    rl = realism;
    rl.gyro_noise_std = noise_levels(ni);
    sc = rocket_scenario("SLEW_DEGRADATION", cfg);
    sc.disturbance_amp = 3.0; sc.disturbance_freq_hz = 1.5; sc.t_end = 10.0;
    for ci = 1:numel(sens_ctrls)
        for seed_i = 1:n_seeds_sens
            cfg2 = cfg; cfg2.plant.theta0 = deg2rad(5);
            out = simulate_case_realistic(sens_ctrls(ci), sc, cfg2, seed_i, rl);
            pm = out.time >= sc.fault_time;
            sens_a(ci, ni, seed_i) = rad2deg(rms(out.theta(pm)));
        end
    end
end

% Sub-sweep B: gust amplitude
sens_b = nan(numel(sens_ctrls), numel(gust_levels), n_seeds_sens);
for gi = 1:numel(gust_levels)
    rl = realism;
    rl.gust_std = gust_levels(gi);
    sc = rocket_scenario("SLEW_DEGRADATION", cfg);
    sc.disturbance_amp = 3.0; sc.disturbance_freq_hz = 1.5; sc.t_end = 10.0;
    for ci = 1:numel(sens_ctrls)
        for seed_i = 1:n_seeds_sens
            cfg2 = cfg; cfg2.plant.theta0 = deg2rad(5);
            out = simulate_case_realistic(sens_ctrls(ci), sc, cfg2, seed_i, rl);
            pm = out.time >= sc.fault_time;
            sens_b(ci, gi, seed_i) = rad2deg(rms(out.theta(pm)));
        end
    end
end

% Plot
fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1320 540]);
axA = subplot(1,2,1); hold(axA,'on'); grid(axA,'on'); set(axA,'YScale','log');
for ci = 1:numel(sens_ctrls)
    means = squeeze(mean(sens_a(ci,:,:), 3));
    stds  = squeeze(std(sens_a(ci,:,:), 0, 3));
    errorbar(axA, rad2deg(noise_levels), means, stds, ...
        'Color', sens_colors{ci}, 'LineWidth', 2.2, 'Marker', 'o', ...
        'MarkerFaceColor', sens_colors{ci}, 'MarkerSize', 6, ...
        'DisplayName', sens_pretty(ci));
end
xlabel(axA, 'Gyro noise \sigma (deg/s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(axA, 'RMS post-fault pitch (deg, log)', 'FontSize', 11, 'FontWeight', 'bold');
title(axA, 'Noise sensitivity (gust held nominal)', 'FontSize', 12, 'FontWeight', 'bold');
legend(axA, 'Location', 'northwest', 'FontSize', 10);
ylim(axA, [0.5 1e3]);

axB = subplot(1,2,2); hold(axB,'on'); grid(axB,'on'); set(axB,'YScale','log');
for ci = 1:numel(sens_ctrls)
    means = squeeze(mean(sens_b(ci,:,:), 3));
    stds  = squeeze(std(sens_b(ci,:,:), 0, 3));
    errorbar(axB, gust_levels, means, stds, ...
        'Color', sens_colors{ci}, 'LineWidth', 2.2, 'Marker', 'o', ...
        'MarkerFaceColor', sens_colors{ci}, 'MarkerSize', 6, ...
        'DisplayName', sens_pretty(ci));
end
xlabel(axB, 'Gust torque \sigma (rad/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(axB, 'RMS post-fault pitch (deg, log)', 'FontSize', 11, 'FontWeight', 'bold');
title(axB, 'Gust sensitivity (gyro noise held nominal)', 'FontSize', 12, 'FontWeight', 'bold');
legend(axB, 'Location', 'northwest', 'FontSize', 10);
ylim(axB, [0.5 1e3]);

sgtitle({'Realism sensitivity: how does each controller degrade with worse sensors / harder gusts?'; ...
    sprintf('All on slew-degradation scenario, %d seeds per point', n_seeds_sens)}, ...
    'FontSize', 12, 'FontWeight', 'bold');

png_path = fullfile(graph_dir, 'realism_sensitivity.png');
exportgraphics(fig2, png_path, 'Resolution', 220);
close(fig2);
fprintf('Saved figure: %s\n', png_path);

% Console
fprintf('\nGyro noise sweep (mean RMS post, deg):\n');
fprintf('  %-22s', 'Gyro sigma (deg/s)');
for ni = 1:numel(noise_levels), fprintf('  %6.2f', rad2deg(noise_levels(ni))); end; fprintf('\n');
for ci = 1:numel(sens_ctrls)
    fprintf('  %-22s', char(sens_pretty(ci)));
    for ni = 1:numel(noise_levels)
        fprintf('  %6.2f', mean(sens_a(ci,ni,:)));
    end
    fprintf('\n');
end

fprintf('\nGust amplitude sweep (mean RMS post, deg):\n');
fprintf('  %-22s', 'Gust sigma (rad/s^2)');
for gi = 1:numel(gust_levels), fprintf('  %6.2f', gust_levels(gi)); end; fprintf('\n');
for ci = 1:numel(sens_ctrls)
    fprintf('  %-22s', char(sens_pretty(ci)));
    for gi = 1:numel(gust_levels)
        fprintf('  %6.2f', mean(sens_b(ci,gi,:)));
    end
    fprintf('\n');
end

end
