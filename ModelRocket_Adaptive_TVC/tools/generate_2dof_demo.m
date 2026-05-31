function generate_2dof_demo(root_dir)
%GENERATE_2DOF_DEMO  Pitch + yaw two-axis demo with yaw-channel slew fault.
%
%  Shows the joint adaptive architecture generalizes axis-wise:
%   - YAW axis takes the slew degradation
%   - PITCH axis is healthy but gets cross-coupled torque from yaw
%   - JOINT_ADAPTIVE per axis correctly identifies its own envelope
%   - PCH per axis hedges to (wrong) nominal slew on yaw
%   - FIXED diverges on yaw, drags pitch with it via coupling

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end

cfg = rocket_defaults();
cfg.plant.theta0 = deg2rad(5);

sc = rocket_scenario("SLEW_DEGRADATION", cfg);
sc.disturbance_amp     = 2.5;
sc.disturbance_freq_hz = 1.5;
sc.t_end               = 10.0;
seed = 7;

ctrl_names = ["FIXED_LQR", "PCH_LQR", "JOINT_ADAPTIVE"];
pretty     = ["Fixed LQR", "PCH (assumes nominal slew)", "Joint Adaptive (proposed)"];
colors     = {[0.15 0.25 0.85], [0.40 0.40 0.40], [0.0 0.55 0.22]};
lstyle     = {'--', ':', '-'};
lwidth     = [1.6, 1.8, 2.6];

sims = struct();
for c = 1:numel(ctrl_names)
    sims(c).out = simulate_case_2dof(ctrl_names(c), sc, cfg, seed);
end

slew_post_yaw = cfg.plant.slew_max * sc.slew_scale_post;

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1340 900]);

ax_titles = {'PITCH axis (healthy, receives cross-coupled torque from yaw)', ...
             'YAW axis (slew envelope drops to 25%% of nominal at fault)'};
for ax = 1:2
    axh = subplot(2, 1, ax);
    hold(axh, 'on'); grid(axh, 'on');
    for c = 1:numel(ctrl_names)
        th = rad2deg(sims(c).out.theta(:, ax));
        th_clip = max(-50, min(50, th));
        plot(axh, sims(c).out.time, th_clip, ...
            'Color', colors{c}, 'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), ...
            'DisplayName', pretty(c));
    end
    xline(axh, sc.fault_time, '--k', 'LineWidth', 1.6, 'Alpha', 0.55, ...
        'HandleVisibility','off');
    yline(axh, 0, ':k', 'LineWidth', 1.0, 'Alpha', 0.4, 'HandleVisibility','off');
    ylabel(axh, 'Angle (deg, clipped \pm50)', 'FontSize', 11, 'FontWeight', 'bold');
    title(axh, ax_titles{ax}, 'FontSize', 12, 'FontWeight', 'bold');
    if ax == 1
        legend(axh, 'Location', 'northwest', 'FontSize', 10);
    else
        xlabel(axh, 'Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
    end
    ylim(axh, [-50 50]);
    set(axh, 'FontSize', 10);
end

sgtitle({'Cross-axis test: yaw-channel slew degradation with pitch-yaw aerodynamic coupling'; ...
    sprintf('Yaw slew: %.1f \\rightarrow %.1f units/s   |   coupling gain = 0.30   |   per-axis joint estimator', ...
        cfg.plant.slew_max, slew_post_yaw)}, ...
    'FontSize', 13, 'FontWeight', 'bold');

out_path = fullfile(graph_dir, 'two_dof_demo.png');
exportgraphics(fig, out_path, 'Resolution', 220);
close(fig);

% ---- Console summary ----------------------------------------------
fprintf('\n=== TWO-DOF DEMO -- POST-FAULT METRICS (deg, %d s window) ===\n', ...
    round(sc.t_end - sc.fault_time));
fprintf('  %-32s   %-22s   %-22s\n', 'Controller', 'PITCH (RMS / peak)', 'YAW (RMS / peak)');
for c = 1:numel(ctrl_names)
    out = sims(c).out;
    pm  = out.time >= sc.fault_time;
    rms_p = rad2deg(rms(out.theta(pm, 1))); pk_p = rad2deg(max(abs(out.theta(pm, 1))));
    rms_y = rad2deg(rms(out.theta(pm, 2))); pk_y = rad2deg(max(abs(out.theta(pm, 2))));
    fprintf('  %-32s   %7.2f / %7.2f         %7.2f / %7.2f\n', ...
        char(pretty(c)), rms_p, pk_p, rms_y, pk_y);
end
fprintf('\nSaved: %s\n\n', out_path);
end
