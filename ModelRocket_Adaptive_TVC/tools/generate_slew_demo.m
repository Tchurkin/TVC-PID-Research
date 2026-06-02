function generate_slew_demo(root_dir)
%GENERATE_SLEW_DEMO  Headline failure mode: servo slew rate degrades mid-flight.
%
%  Demonstrates:
%   - FIXED_LQR limit-cycles (demands more slew than servo can deliver)
%   - ADAPTIVE_KEFF_LQR makes it WORSE (misidentifies slew lag as keff drop,
%     raises gains, demands even more slew)
%   - JOINT_ADAPTIVE detects the slew envelope, freezes keff identification,
%     scales gains by bandwidth ratio, settles cleanly
%
%  This is the publication hero plot.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end

% ---- Scenario -------------------------------------------------------
cfg = rocket_defaults();
% Keep the old severe-collapse actuator stack for this controller failure-mode
% demo. Current nominal sim assumptions are faster and belong in the regime map.
cfg.plant.slew_max = 12.0;
cfg.controllers.JOINT_ADAPTIVE.slew_nominal = cfg.plant.slew_max;
cfg.controllers.JOINT_ADAPTIVE.slew_min = 0.10 * cfg.plant.slew_max;
cfg.controllers.PCH_LQR.slew_max_assumed = cfg.plant.slew_max;
cfg.plant.theta0 = deg2rad(5);

sc  = rocket_scenario("SLEW_DEGRADATION", cfg);
sc.disturbance_amp     = 3.0;
sc.disturbance_freq_hz = 1.5;
sc.t_end               = 10.0;
seed = 7;

% ---- Run simulations ------------------------------------------------
ctrl_names = ["FIXED_LQR", "ADAPTIVE_KEFF_LQR", "SIGMA_MRAC", "PCH_LQR", "JOINT_ADAPTIVE"];
pretty     = ["Fixed LQR", "keff-RLS (no leakage)", "\sigma-mod MRAC", "PCH (assumes nominal slew)", "Joint Adaptive (proposed)"];
colors     = {[0.15 0.25 0.85], [0.85 0.45 0.10], [0.55 0.27 0.55], [0.40 0.40 0.40], [0.0 0.55 0.22]};
lstyle     = {'--', ':', '-.', ':', '-'};
lwidth     = [1.6, 1.6, 1.8, 1.8, 2.6];

sims = struct();
for c = 1:numel(ctrl_names)
    sims(c).out  = simulate_case(ctrl_names(c), sc, cfg, seed);
end

% Compute display y-limit: clip diverging traces but keep them visible
all_theta_clipped = [];
for c = 1:numel(ctrl_names)
    th = rad2deg(sims(c).out.theta);
    th = max(-50, min(50, th));        % clip to readable window
    all_theta_clipped = [all_theta_clipped; th]; %#ok<AGROW>
end

% ---- Figure ---------------------------------------------------------
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1340 980]);

% ---------- Panel 1: Pitch angle (CLIPPED to ±50 for readability) ----
ax1 = subplot(4,1,1);
hold(ax1,'on'); grid(ax1,'on');
for c = 1:numel(ctrl_names)
    th = rad2deg(sims(c).out.theta);
    th_clip = max(-55, min(55, th));
    plot(ax1, sims(c).out.time, th_clip, ...
        'Color', colors{c}, 'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), ...
        'DisplayName', pretty(c));
end
xline(ax1, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, ...
    'Label', sprintf('servo slew drops to %.0f%%', 100*sc.slew_scale_post), ...
    'LabelVerticalAlignment', 'top', 'FontSize', 10, 'HandleVisibility','off');
yline(ax1, 0, ':k', 'LineWidth', 1.0, 'Alpha', 0.5, 'HandleVisibility','off');
ylabel(ax1, 'Pitch angle (deg, clipped ±55°)', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax1, [-55 55]);
legend(ax1, 'Location', 'northwest', 'FontSize', 10);
title(ax1, 'Pitch response under mid-flight servo-slew degradation', ...
    'FontSize', 12, 'FontWeight', 'bold');
set(ax1, 'FontSize', 10);

% ---------- Panel 2: Servo rate (saturated against degraded envelope)
ax2 = subplot(4,1,2);
hold(ax2,'on'); grid(ax2,'on');
slew_pre  = cfg.plant.slew_max;
slew_post = cfg.plant.slew_max * sc.slew_scale_post;
for c = 1:numel(ctrl_names)
    out = sims(c).out;
    du  = abs(diff(out.u_act))/cfg.dt;
    plot(ax2, out.time(2:end), du, ...
        'Color', colors{c}, 'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), ...
        'DisplayName', pretty(c));
end
plot(ax2, [0 sc.fault_time], [slew_pre slew_pre], 'k--', 'LineWidth', 1.4, ...
    'DisplayName', sprintf('Slew limit (pre = %.1f)', slew_pre));
plot(ax2, [sc.fault_time sc.t_end], [slew_post slew_post], 'k-.', 'LineWidth', 1.4, ...
    'DisplayName', sprintf('Slew limit (post = %.1f)', slew_post));
xline(ax2, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, 'HandleVisibility','off');
ylabel(ax2, '|du_{act}/dt| (units/s)', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax2, [0 1.2*slew_pre]);
legend(ax2, 'Location', 'northeast', 'FontSize', 9, 'NumColumns', 2);
title(ax2, 'Actuator slew demand vs. envelope', 'FontSize', 12, 'FontWeight', 'bold');
set(ax2, 'FontSize', 10);

% ---------- Panel 3: keff estimates (true vs. adaptive vs. joint) ----
ax3 = subplot(4,1,3);
hold(ax3,'on'); grid(ax3,'on');
keff_truth = cfg.plant.control_eff * ones(size(sims(1).out.time));   % unchanged in slew scenario
plot(ax3, sims(1).out.time, keff_truth, 'k--', 'LineWidth', 1.4, 'DisplayName', 'True keff');
for c = 2:numel(ctrl_names)
    plot(ax3, sims(c).out.time, sims(c).out.keff_est, 'Color', colors{c}, ...
        'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), 'DisplayName', sprintf('keff_{est}: %s', pretty(c)));
end
xline(ax3, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, 'HandleVisibility','off');
ylabel(ax3, 'keff estimate', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax3, [0 max(15, 1.2*max(sims(2).out.keff_est, [], 'omitnan'))]);
legend(ax3, 'Location', 'east', 'FontSize', 8);
title(ax3, 'keff identification: bare RLS misidentifies slew lag; \sigma-mod resists drift; joint freezes ID', ...
    'FontSize', 12, 'FontWeight', 'bold');
set(ax3, 'FontSize', 10);

% ---------- Panel 4: Slew estimate + gain scale (joint only) ---------
ax4 = subplot(4,1,4);
hold(ax4,'on'); grid(ax4,'on');
joint_idx = numel(ctrl_names);                % JOINT_ADAPTIVE
pch_idx   = find(ctrl_names == "PCH_LQR");    % PCH for comparison

yyaxis(ax4, 'left');
plot(ax4, sims(joint_idx).out.time, sims(joint_idx).out.slew_est, ...
    'Color', colors{joint_idx}, 'LineStyle', '-', 'LineWidth', 2.4, ...
    'DisplayName', 'slew_{est} (joint, online)');
plot(ax4, sims(pch_idx).out.time, sims(pch_idx).out.slew_est, ...
    'Color', colors{pch_idx}, 'LineStyle', ':', 'LineWidth', 1.8, ...
    'DisplayName', 'slew_{assumed} (PCH, fixed)');
plot(ax4, [0 sc.fault_time], [slew_pre slew_pre], 'k--', 'LineWidth', 1.2, ...
    'DisplayName', 'true slew');
plot(ax4, [sc.fault_time sc.t_end], [slew_post slew_post], 'k--', 'LineWidth', 1.2, ...
    'HandleVisibility','off');
ylabel(ax4, 'Slew envelope estimate', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax4, [0 1.2*slew_pre]);

yyaxis(ax4, 'right');
gs_joint = sims(joint_idx).out.gain_scale;
plot(ax4, sims(joint_idx).out.time, gs_joint, 'Color', colors{joint_idx}, ...
    'LineStyle', '-.', 'LineWidth', 2.0, 'DisplayName', 'gain scale (joint)');
yline(ax4, 1.0, ':k', 'LineWidth', 1.0, 'Alpha', 0.4, 'HandleVisibility','off');
ylabel(ax4, 'LQR gain scale', 'FontSize', 11, 'FontWeight', 'bold');
ylim(ax4, [0 max(2.0, 1.2*max(gs_joint, [], 'omitnan'))]);

xline(ax4, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, 'HandleVisibility','off');
xlabel(ax4, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
legend(ax4, 'Location', 'east', 'FontSize', 9);
title(ax4, 'Joint estimator: slew_{est} drops, gain scales DOWN to match envelope', ...
    'FontSize', 12, 'FontWeight', 'bold');
set(ax4, 'FontSize', 10);

sgtitle({'Failure mode: servo slew degradation — naive adaptation diverges, joint adaptation recovers'; ...
    sprintf('Disturbance: %.1f rad/s² @ %.1f Hz   |   Slew: %.1f → %.1f units/s   |   Tau_{act} = %.0f ms', ...
        sc.disturbance_amp, sc.disturbance_freq_hz, slew_pre, slew_post, 1000*cfg.plant.tau_act)}, ...
    'FontSize', 13, 'FontWeight', 'bold');

out_path = fullfile(graph_dir, 'slew_degradation_demo.png');
exportgraphics(fig, out_path, 'Resolution', 220);
close(fig);

% ---- Console summary ------------------------------------------------
fprintf('\n=== SLEW DEGRADATION DEMO — POST-FAULT METRICS ===\n');
fprintf('Scenario: SLEW_DEGRADATION  |  slew: %.1f -> %.1f units/s (%.0f%%)  |  fault_time: %.1f s\n\n', ...
    slew_pre, slew_post, 100*sc.slew_scale_post, sc.fault_time);
for c = 1:numel(ctrl_names)
    out      = sims(c).out;
    pre_mask = out.time < sc.fault_time & out.time > 1.0;
    pm       = out.time >= sc.fault_time;
    rms_pre  = rad2deg(rms(out.theta(pre_mask)));
    rms_post = rad2deg(rms(out.theta(pm)));
    pk_post  = rad2deg(max(abs(out.theta(pm))));
    fprintf('  %-30s  RMS pre=%6.2f   RMS post=%7.2f   peak post=%7.2f deg\n', ...
        char(pretty(c)), rms_pre, rms_post, pk_post);
end
fprintf('\nSaved: %s\n\n', out_path);
end
