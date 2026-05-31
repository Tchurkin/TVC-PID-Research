function generate_high_keff_demo(root_dir)
%GENERATE_HIGH_KEFF_DEMO
%  Demonstrates keff-spike instability and adaptive recovery.
%
%  Setup:
%   - Rocket starts at 35 deg pitch offset
%   - All controllers stabilise normally for the first 3 s
%   - At t = 3 s, keff jumps from 8 to 28 (3.5x spike)
%   - Fixed controllers: effective loop gain exceeds Routh boundary -> fishtail
%   - Adaptive controller: detects keff change, scales LQR gains down,
%     maintains stability and drives angle back to 0

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end

% ---- Scenario -------------------------------------------------------
cfg = rocket_defaults();
cfg.plant.theta0 = 0;                 % start at rest (no initial-condition transient)

sc  = rocket_scenario("HIGH_KEFF_FAULT", cfg);
seed = 7;

% ---- Run simulations ------------------------------------------------
ctrl_names = ["FIXED_LQR", "ADAPTIVE_KEFF_LQR", "SIGMA_MRAC", "PCH_LQR", "JOINT_ADAPTIVE"];
pretty     = ["Fixed LQR", "keff-RLS", "\sigma-mod MRAC", "PCH (assumes nominal slew)", "Joint Adaptive"];
colors     = {[0.15 0.25 0.85], [0.85 0.45 0.10], [0.55 0.27 0.55], [0.40 0.40 0.40], [0.0 0.55 0.22]};
lstyle     = {'--', ':', '-.', ':', '-'};
lwidth     = [1.6, 1.6, 1.8, 1.8, 2.6];

sims = struct();
for c = 1:numel(ctrl_names)
    sims(c).out  = simulate_case(ctrl_names(c), sc, cfg, seed);
    sims(c).name = ctrl_names(c);
end

% ---- Routh boundary for annotation ----------------------------------
% keff_actual * loop_gain_nominal / keff_nom = stability boundary
routh_bound_kp = (cfg.plant.aero_damp + 1/cfg.plant.tau_act) ...
               * cfg.plant.aero_damp / cfg.plant.control_eff;   % = 5.44/8

% ---- Figure ---------------------------------------------------------
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1320 860]);

% ---------- Panel 1: Pitch angle -------------------------------------
ax1 = subplot(3,1,1);
hold(ax1,'on'); grid(ax1,'on');
for c = 1:numel(ctrl_names)
    plot(ax1, sims(c).out.time, rad2deg(sims(c).out.theta), ...
        'Color', colors{c}, 'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), ...
        'DisplayName', pretty(c));
end
xline(ax1, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, ...
    'Label', sprintf('keff drops to %.0f%% of nominal', 100*sc.control_eff_scale_post), ...
    'LabelVerticalAlignment', 'top', 'FontSize', 10);
yline(ax1, 0, ':k', 'LineWidth', 1.1, 'Alpha', 0.5);
ylabel(ax1, 'Pitch Angle (deg)', 'FontSize', 11, 'FontWeight', 'bold');
legend(ax1, 'Location', 'northeast', 'FontSize', 10);
keff_fault = cfg.plant.control_eff * sc.control_eff_scale_post;
title(ax1, sprintf('Sustained Wind Disturbance  |  Motor Authority Drop at t = %.1f s  (keff: %.1f → %.1f)', ...
    sc.fault_time, cfg.plant.control_eff, keff_fault), 'FontSize', 12, 'FontWeight', 'bold');
set(ax1, 'FontSize', 10);

% ---------- Panel 2: keff estimate (both adaptive controllers) -------
ax2 = subplot(3,1,2);
hold(ax2,'on'); grid(ax2,'on');
% True keff trajectory
keff_truth = cfg.plant.control_eff * ones(size(sims(1).out.time));
keff_truth(sims(1).out.time >= sc.fault_time) = cfg.plant.control_eff * sc.control_eff_scale_post;
plot(ax2, sims(1).out.time, keff_truth, 'k--', 'LineWidth', 1.4, 'DisplayName', 'True keff');
for c = 2:numel(ctrl_names)
    plot(ax2, sims(c).out.time, sims(c).out.keff_est, 'Color', colors{c}, ...
        'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), 'DisplayName', sprintf('keff_{est}: %s', pretty(c)));
end
xl2 = xline(ax2, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55);
xl2.HandleVisibility = 'off';
ylabel(ax2, 'keff (rad/s² per rad)', 'FontSize', 11, 'FontWeight', 'bold');
legend(ax2, 'Location', 'east', 'FontSize', 10);
title(ax2, 'Adaptive keff Estimator — Detects Loss of Control Authority', ...
    'FontSize', 12, 'FontWeight', 'bold');
set(ax2, 'FontSize', 10);

% ---------- Panel 3: adaptive gain scale ----------------------------
ax3 = subplot(3,1,3);
hold(ax3,'on'); grid(ax3,'on');
for c = 2:numel(ctrl_names)
    plot(ax3, sims(c).out.time, sims(c).out.gain_scale, 'Color', colors{c}, ...
        'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), 'DisplayName', sprintf('gain scale: %s', pretty(c)));
end
yline(ax3, 1.0, '--k', 'LineWidth', 1.2, 'Alpha', 0.6, 'DisplayName', 'Nominal (1.0)');
xl3 = xline(ax3, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55);
xl3.HandleVisibility = 'off';
xlabel(ax3, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax3, 'LQR Gain Scale', 'FontSize', 11, 'FontWeight', 'bold');
legend(ax3, 'Location', 'east', 'FontSize', 10);
title(ax3, 'Adaptive Increases LQR Gains to Compensate for Lost Authority', ...
    'FontSize', 12, 'FontWeight', 'bold');
set(ax3, 'FontSize', 10);

sgtitle({'TVC Rocket — Motor Authority Decay Demo'; ...
    'Fixed controllers cannot reject wind disturbance with reduced authority; adaptive boosts gains'}, ...
    'FontSize', 13, 'FontWeight', 'bold');

% Save
out_path = fullfile(graph_dir, 'keff_spike_demo.png');
exportgraphics(fig, out_path, 'Resolution', 220);
close(fig);

% ---- Console summary ------------------------------------------------
fprintf('\n=== keff SPIKE DEMO — POST-FAULT METRICS ===\n');
fprintf('Scenario: HIGH_KEFF_FAULT  |  keff: %.0f -> %.0f (%.1fx)  |  fault_time: %.1f s\n\n', ...
    cfg.plant.control_eff, cfg.plant.control_eff * sc.control_eff_scale_post, ...
    sc.control_eff_scale_post, sc.fault_time);

for c = 1:numel(ctrl_names)
    out      = sims(c).out;
    pm       = out.time >= sc.fault_time;
    rms_post = sqrt(mean(out.theta(pm).^2));
    pp_post  = max(out.theta(pm)) - min(out.theta(pm));
    fprintf('  %-22s  RMS = %.3f deg   peak-to-peak = %.3f deg   final = %.2f deg\n', ...
        char(ctrl_names(c)), rad2deg(rms_post), rad2deg(pp_post), rad2deg(out.theta(end)));
end
fprintf('\nSaved: %s\n\n', out_path);
end
