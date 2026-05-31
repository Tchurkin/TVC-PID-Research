function generate_phase_diagram_reference_vehicles()
%GENERATE_PHASE_DIAGRAM_REFERENCE_VEHICLES  Validator phase diagram in
% physical units, with hobbyist-class reference vehicles overlaid.
%
% The validator (recommend_envelope) partitions the (p, slew) plane into
% RESCUE / FUNDAMENTAL / INFEASIBLE regions. This figure plots those
% boundaries and adds reference vehicle dots so readers familiar with the
% propulsive-landing hobby community (BPS.space, etc.) can place themselves.
%
% Reference vehicle estimates are PUBLIC-VIDEO ESTIMATES, not measurements
% from those projects. They are labeled as such in the figure.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'validator'));
gfxdir = fullfile(here, 'results', 'graphs');
if ~exist(gfxdir, 'dir'); mkdir(gfxdir); end

set(groot, 'defaultFigureVisible', 'off');

% Phase-region grid in physical units.
p_grid    = linspace(2, 14, 121);
slew_grid = linspace(5, 200, 121);
servo_max_deg = 7.0;   % representative hobbyist gimbal travel

[P, S] = meshgrid(p_grid, slew_grid);
region_code = zeros(size(P));   % 1=RESCUE, 0.5=FUNDAMENTAL, 0=INFEASIBLE

for i = 1:numel(P)
    meas = struct('slew_deg_per_s', S(i), 'servo_max_deg', servo_max_deg, ...
        'p_est', P(i), 'keff_est', 8.0, 'damp_est', 0.5);
    evalc('rec = recommend_envelope(meas);');
    switch rec.region
        case 'RESCUE',      region_code(i) = 1.0;
        case 'FUNDAMENTAL', region_code(i) = 0.5;
        otherwise,          region_code(i) = 0.0;
    end
end

fig = figure('Position', [80 80 1100 720], 'Color', 'w');
ax = axes('Position', [0.10 0.12 0.74 0.78]); hold(ax, 'on');
imagesc(ax, p_grid, slew_grid, region_code);
set(ax, 'YDir', 'normal');
caxis(ax, [0 1]);

cmap = [ ...
    0.85 0.30 0.25;    % INFEASIBLE (red)
    0.95 0.85 0.40;    % FUNDAMENTAL (amber)
    0.40 0.75 0.45];   % RESCUE (green)
colormap(ax, cmap);

xlabel(ax, 'Airframe instability p (rad/s)', 'FontSize', 12);
ylabel(ax, 'Loaded gimbal slew rate (deg/s)', 'FontSize', 12);
title(ax, sprintf(['Preflight Phase Diagram with Hobbyist Reference Vehicles' ...
    '  (servo travel = %.1f deg)'], servo_max_deg), 'FontSize', 13);

% Region legend via dummy patches
h_resc = patch(ax, NaN, NaN, cmap(3,:), 'EdgeColor', 'k');
h_fund = patch(ax, NaN, NaN, cmap(2,:), 'EdgeColor', 'k');
h_inf  = patch(ax, NaN, NaN, cmap(1,:), 'EdgeColor', 'k');

% --- Reference vehicles (public-video estimates) ----------------------
% Each row: name, p_est (rad/s), loaded_slew (deg/s)
refs = {
    'BPS Sprint-class (est.)',          7.0,  90;   % small TVC, MG90/HS-5070 class
    'BPS Signal/Echo-class (est.)',     5.5, 110;   % larger TVC, faster servo
    'Typical hobby F-motor TVC (est.)', 6.0,  60;   % SG90-class
    'Educational kit, slow servo (est.)', 8.0, 30;
    'Your rocket (TBD - update)',       6.5,  75;
};

marker_colors = lines(size(refs, 1));
ref_handles = gobjects(size(refs, 1), 1);
for k = 1:size(refs, 1)
    name = refs{k, 1};
    p_v  = refs{k, 2};
    s_v  = refs{k, 3};
    ref_handles(k) = plot(ax, p_v, s_v, 'o', 'MarkerSize', 12, ...
        'MarkerFaceColor', marker_colors(k,:), 'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.5);
    text(ax, p_v + 0.15, s_v + 4, name, 'FontSize', 10, ...
        'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.65], 'Margin', 2);
end

legend(ax, [h_resc h_fund h_inf], {'RESCUE (GO with tuned PD)', ...
    'FUNDAMENTAL (MARGINAL)', 'INFEASIBLE (NOGO)'}, ...
    'Location', 'northeastoutside', 'FontSize', 10);
grid(ax, 'on');
xlim(ax, [p_grid(1) p_grid(end)]);
ylim(ax, [slew_grid(1) slew_grid(end)]);

% Disclaimer
annotation('textbox', [0.10 0.00 0.84 0.045], 'String', ...
    ['Reference-vehicle positions are public-video estimates of ' ...
     'p and loaded slew, not measurements taken from those projects. ' ...
     'Final positions should be taken from each builder''s own bench data.'], ...
    'EdgeColor', 'none', 'FontSize', 9, 'FontAngle', 'italic', ...
    'Color', [0.30 0.30 0.30], 'HorizontalAlignment', 'center');

out_path = fullfile(gfxdir, 'phase_diagram_reference_vehicles.png');
try
    exportgraphics(fig, out_path, 'Resolution', 200);
catch
    saveas(fig, out_path);
end
close(fig);
fprintf('Saved: %s\n', out_path);
end
