function generate_preflight_workflow_figure()
%GENERATE_PREFLIGHT_WORKFLOW_FIGURE  One-page diagram of the bench-calibrated
% preflight workflow proposed by this project. Six boxes, five arrows,
% builder-readable language. Saved to results/graphs/.

here = fileparts(mfilename('fullpath'));
gfxdir = fullfile(here, 'results', 'graphs');
if ~exist(gfxdir, 'dir'); mkdir(gfxdir); end

set(groot, 'defaultFigureVisible', 'off');
fig = figure('Position', [80 80 1500 600], 'Color', 'w');
ax = axes('Position', [0.02 0.05 0.96 0.78]); hold(ax, 'on');
axis(ax, 'off'); xlim(ax, [0 100]); ylim(ax, [0 40]);

title_h = title(ax, ['Preflight Workflow: Replacing "Tune Until It Flies" ' ...
    'with Measure, Verify, Tune'], 'FontSize', 16, 'FontWeight', 'bold');
set(title_h, 'Position', [50 42 0]);

% Six-box pipeline: bench rig -> CSV -> parser -> validator -> verdict+gains -> fly
boxes = {
    'BENCH RIG',           'analog-feedback servo';
    'BENCH CSV',           'loaded slew, travel, repeatability';
    'PARSER',              'bench\_to\_validator.m';
    'VALIDATOR + AUTOTUNE','bench\_to\_autotune.m';
    'VERDICT + GAINS',     'GO / MARGINAL / NOGO  +  Kp, Kd';
    'FLY (or do not)',     'informed decision';
};
n = size(boxes, 1);
box_w = 14; box_h = 12;
y_center = 20;
x_centers = linspace(8, 92, n);

face_colors = [ ...
    0.85 0.92 1.00;     % bench
    0.85 0.92 1.00;     % csv
    0.92 0.88 0.98;     % parser
    0.92 0.88 0.98;     % validator
    0.80 0.95 0.82;     % verdict
    1.00 0.92 0.80;     % fly
];

for i = 1:n
    x = x_centers(i);
    rectangle(ax, 'Position', [x - box_w/2, y_center - box_h/2, box_w, box_h], ...
        'Curvature', [0.15 0.15], 'FaceColor', face_colors(i,:), ...
        'EdgeColor', [0.20 0.20 0.20], 'LineWidth', 1.5);
    text(ax, x, y_center + 2.2, boxes{i,1}, 'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', 'FontSize', 11);
    text(ax, x, y_center - 2.0, boxes{i,2}, 'HorizontalAlignment', 'center', ...
        'FontSize', 9, 'Color', [0.25 0.25 0.25], 'Interpreter', 'tex');
end

% Arrows between boxes
for i = 1:n-1
    x_start = x_centers(i) + box_w/2 + 0.2;
    x_end   = x_centers(i+1) - box_w/2 - 0.2;
    annotation('arrow', ...
        norm_x(ax, [x_start, x_end]), norm_y(ax, [y_center, y_center]), ...
        'LineWidth', 1.8, 'HeadStyle', 'vback2', 'HeadLength', 10, 'HeadWidth', 10);
end

% Status-quo annotation below
text(ax, 50, 8.5, 'Status quo: tune-by-crash on the launchpad (copy gains, iterate, hope)', ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', [0.60 0.20 0.20], ...
    'FontAngle', 'italic');
text(ax, 50, 5.5, ['This workflow: every step is bench-grounded; ' ...
    'the decision to fly is quantitative and reproducible'], ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', [0.15 0.45 0.20], ...
    'FontWeight', 'bold');

out_path = fullfile(gfxdir, 'preflight_workflow.png');
try
    exportgraphics(fig, out_path, 'Resolution', 200);
catch
    saveas(fig, out_path);
end
close(fig);
fprintf('Saved: %s\n', out_path);
end


function n = norm_x(ax, x)
% Convert data-x in `ax` to normalized figure coords for `annotation`.
pos = get(ax, 'Position');     % [left bottom w h] in figure units
xlims = xlim(ax);
n = pos(1) + (x - xlims(1)) / (xlims(2) - xlims(1)) * pos(3);
end

function n = norm_y(ax, y)
pos = get(ax, 'Position');
ylims = ylim(ax);
n = pos(2) + (y - ylims(1)) / (ylims(2) - ylims(1)) * pos(4);
end
