function plot_phase_diagram(T, out_map_png, out_frontier_png, flightT)
%PLOT_PHASE_DIAGRAM Plot physical regime maps and required-slew frontier.

if nargin < 4
    flightT = table();
end

% Use representative gimbal slices so plots remain meaningful for both
% grid sweeps and continuous LHS runs.
max_gimbal_slices = 4;
[g_idx, g_labels] = build_gimbal_groups(T.max_gimbal_deg, max_gimbal_slices);
n_tiles = max(1, numel(g_labels));
n_cols = max(1, ceil(sqrt(n_tiles)));
n_rows = ceil(n_tiles / n_cols);

regime_codes = [0 1 2];
regime_labels = {'INFEASIBLE', 'FRAGILE', 'EASY'};
regime_colors = [0.85 0.30 0.25; 0.95 0.82 0.35; 0.40 0.75 0.45];

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1300 820]);
tlo = tiledlayout(fig, n_rows, n_cols, 'TileSpacing', 'compact', 'Padding', 'compact');
for ig = 1:n_tiles
    ax = nexttile(tlo, ig);
    hold(ax, 'on');
    grid(ax, 'on');

    sub = T(g_idx == ig, :);
    for ir = 1:numel(regime_codes)
        rc = regime_codes(ir);
        m = sub.regime_code == rc;
        if any(m)
            scatter(ax, sub.p_unstable(m), sub.servo_slew_deg_s(m), 28, ...
                'MarkerFaceColor', regime_colors(ir, :), ...
                'MarkerEdgeColor', 'k', ...
                'MarkerFaceAlpha', 0.85, ...
                'DisplayName', regime_labels{ir});
        end
    end

    if height(sub) <= 80 && ismember('robustness', sub.Properties.VariableNames)
        for k = 1:height(sub)
            text(ax, sub.p_unstable(k), sub.servo_slew_deg_s(k), sprintf(' r=%.2f', sub.robustness(k)), ...
                'FontSize', 7, 'Color', [0.10 0.10 0.10]);
        end
    end

    if ~isempty(flightT) && all(ismember({'p_unstable','servo_slew_deg_s','max_gimbal_deg'}, flightT.Properties.VariableNames))
        ft_sel = false(height(flightT), 1);
        if numel(unique(T.max_gimbal_deg)) <= max_gimbal_slices
            g_val = unique(T.max_gimbal_deg);
            g_target = g_val(ig);
            ft_sel = abs(flightT.max_gimbal_deg - g_target) < 1e-9;
        else
            g_edges = quantile(double(T.max_gimbal_deg), linspace(0, 1, n_tiles + 1));
            lo = g_edges(ig);
            hi = g_edges(ig + 1);
            if ig < n_tiles
                ft_sel = flightT.max_gimbal_deg >= lo & flightT.max_gimbal_deg < hi;
            else
                ft_sel = flightT.max_gimbal_deg >= lo & flightT.max_gimbal_deg <= hi;
            end
        end
        if any(ft_sel)
            ft = flightT(ft_sel, :);
            scatter(ax, ft.p_unstable, ft.servo_slew_deg_s, 90, 'kd', 'filled', 'MarkerEdgeColor', 'w');
        end
    end

    xlabel(ax, 'Instability demand p (1/s)');
    ylabel(ax, 'Loaded slew rate (deg/s)');
    title(ax, g_labels{ig});
end
legend(nexttile(tlo, 1), 'Location', 'best');
title(tlo, 'Experiment 1 Physical Regime Maps (robustness annotated when sparse)');
exportgraphics(fig, out_map_png, 'Resolution', 220);
close(fig);

fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 900 540]);
hold on;
clr = lines(n_tiles);
n_p_bins = min(14, max(6, round(sqrt(height(T)))));
p_edges = quantile(double(T.p_unstable), linspace(0, 1, n_p_bins + 1));
p_edges = uniquetol(p_edges, 1e-9);
if numel(p_edges) < 2
    p_edges = [min(T.p_unstable) max(T.p_unstable) + eps(max(T.p_unstable))];
end
x_ctr = 0.5 * (p_edges(1:end-1) + p_edges(2:end));

for ig = 1:n_tiles
    sub_g = T(g_idx == ig, :);
    y_frag = nan(size(x_ctr));
    y_easy = nan(size(x_ctr));
    for ip = 1:numel(x_ctr)
        if ip < numel(x_ctr)
            in_bin = sub_g.p_unstable >= p_edges(ip) & sub_g.p_unstable < p_edges(ip + 1);
        else
            in_bin = sub_g.p_unstable >= p_edges(ip) & sub_g.p_unstable <= p_edges(ip + 1);
        end
        sub = sub_g(in_bin, :);
        if isempty(sub)
            continue;
        end
        frag = sub.servo_slew_deg_s(sub.regime_code >= 1);
        easy = sub.servo_slew_deg_s(sub.regime_code == 2);
        if ~isempty(frag)
            y_frag(ip) = min(frag);
        end
        if ~isempty(easy)
            y_easy(ip) = min(easy);
        end
    end

    if any(isfinite(y_frag))
        plot(x_ctr, y_frag, 'o-', 'Color', clr(ig, :), 'LineWidth', 1.8, ...
            'DisplayName', sprintf('Fragile+ frontier, %s', g_labels{ig}));
    end
    if any(isfinite(y_easy))
        plot(x_ctr, y_easy, 's--', 'Color', clr(ig, :), 'LineWidth', 1.6, ...
            'DisplayName', sprintf('Easy frontier, %s', g_labels{ig}));
    end
end

grid on;
xlabel('Instability demand p (1/s)');
ylabel('Required loaded slew rate (deg/s)');
title('Required Slew vs Instability (binned phase-transition frontier)');
legend('Location', 'northwest');
ylim([min(T.servo_slew_deg_s) max(T.servo_slew_deg_s)]);
exportgraphics(fig2, out_frontier_png, 'Resolution', 240);
close(fig2);

end


function [g_idx, g_labels] = build_gimbal_groups(gimbal_values, max_groups)
g = double(gimbal_values(:));
u = unique(g);
if numel(u) <= max_groups
    g_idx = zeros(size(g));
    g_labels = cell(numel(u), 1);
    for i = 1:numel(u)
        g_idx(abs(g - u(i)) < 1e-12) = i;
        g_labels{i} = sprintf('Max gimbal = %.1f deg', u(i));
    end
    return;
end

edges = quantile(g, linspace(0, 1, max_groups + 1));
edges = uniquetol(edges, 1e-9);
if numel(edges) < 2
    edges = [min(g), max(g) + eps(max(g))];
end

n_groups = numel(edges) - 1;
g_idx = zeros(size(g));
g_labels = cell(n_groups, 1);
for i = 1:n_groups
    lo = edges(i);
    hi = edges(i + 1);
    if i < n_groups
        in = g >= lo & g < hi;
    else
        in = g >= lo & g <= hi;
    end
    g_idx(in) = i;
    g_labels{i} = sprintf('Max gimbal %.1f-%.1f deg', lo, hi);
end

missing = g_idx == 0;
if any(missing)
    g_idx(missing) = n_groups;
end

end
