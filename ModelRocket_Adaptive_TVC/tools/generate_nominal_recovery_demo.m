function generate_nominal_recovery_demo(root_dir)
%GENERATE_NOMINAL_RECOVERY_DEMO
%  Tip-over envelope: shows how each controller responds to large initial
%  pitch under realistic slew limits. The fixed LQR diverges past ~12 deg
%  because the slew envelope cannot follow its commanded steps; the joint
%  adaptive controller throttles itself based on slew_est and survives.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end

cfg = rocket_defaults();
seed = 1;

ctrl_names  = ["FIXED_LQR", "ADAPTIVE_KEFF_LQR", "SIGMA_MRAC", "PCH_LQR", "JOINT_ADAPTIVE"];
pretty      = ["Fixed LQR", "keff-RLS", "\sigma-mod MRAC", "PCH", "Joint Adaptive"];
colors      = {[0.15 0.25 0.85], [0.85 0.45 0.10], [0.55 0.27 0.55], [0.40 0.40 0.40], [0.0 0.55 0.22]};
lstyle      = {'--', ':', '-.', ':', '-'};
lwidth      = [1.4, 1.4, 1.6, 1.6, 2.6];

theta0_list = [5, 10, 15, 20];

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1400 880]);
for ti = 1:numel(theta0_list)
    th0 = theta0_list(ti);
    cfg.plant.theta0 = deg2rad(th0);
    sc = rocket_scenario("NOMINAL", cfg);
    sc.disturbance_amp = 0.0;
    sc.t_end           = 5.0;

    ax = subplot(2, 2, ti);
    hold(ax, 'on'); grid(ax, 'on');
    for c = 1:numel(ctrl_names)
        out = simulate_case(ctrl_names(c), sc, cfg, seed);
        th  = rad2deg(out.theta);
        th_clip = max(-3*th0, min(3*th0, th));
        plot(ax, out.time, th_clip, ...
            'Color', colors{c}, 'LineStyle', lstyle{c}, 'LineWidth', lwidth(c), ...
            'DisplayName', pretty(c));
    end
    yline(ax, 0, ':k', 'LineWidth', 1.0, 'Alpha', 0.4, 'HandleVisibility','off');
    xlabel(ax, 'Time (s)', 'FontSize', 10);
    ylabel(ax, 'Pitch (deg)', 'FontSize', 10);
    title(ax, sprintf('\\theta_0 = %d deg  (clipped to +/-%d deg)', th0, 3*th0), ...
        'FontSize', 11, 'FontWeight', 'bold');
    if ti == 1
        legend(ax, 'Location', 'northeast', 'FontSize', 9);
    end
    set(ax, 'FontSize', 10);
end

sgtitle({'Tip-over recovery envelope under realistic slew limit (0.25 rad/s gimbal)'; ...
    'Fixed LQR diverges past ~12 deg; joint adaptive throttles via slew envelope and recovers'}, ...
    'FontSize', 12, 'FontWeight', 'bold');

out_path = fullfile(graph_dir, 'tipover_envelope_demo.png');
exportgraphics(fig, out_path, 'Resolution', 220);
close(fig);

% Console summary
fprintf('\n=== TIP-OVER ENVELOPE (peak |theta|, settle time within 2 deg) ===\n');
fprintf('  %-22s', 'Controller');
for th0 = theta0_list, fprintf('  th0=%d        ', th0); end
fprintf('\n');
for c = 1:numel(ctrl_names)
    fprintf('  %-22s', char(pretty(c)));
    for th0 = theta0_list
        cfg.plant.theta0 = deg2rad(th0);
        sc = rocket_scenario("NOMINAL", cfg);
        sc.disturbance_amp = 0.0; sc.t_end = 5.0;
        out = simulate_case(ctrl_names(c), sc, cfg, seed);
        pk = rad2deg(max(abs(out.theta)));
        idx = find(abs(rad2deg(out.theta)) < 2.0 & out.time > 0.3, 1);
        if isempty(idx), ts_str = ' DIV'; else, ts_str = sprintf('%4.2fs', out.time(idx)); end
        fprintf('  pk=%6.1f %s', pk, ts_str);
    end
    fprintf('\n');
end
fprintf('Saved: %s\n', out_path);
end
