function plot_phase_diagram(T, out_map_png, out_frontier_png, flightT)
%PLOT_PHASE_DIAGRAM Plot physical regime maps and required-slew frontier.

if nargin < 4
    flightT = table();
end

p_vals = unique(T.p_unstable)';
slew_vals = unique(T.servo_slew_deg_s)';
gimbal_vals = unique(T.max_gimbal_deg)';

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1300 820]);
for ig = 1:numel(gimbal_vals)
    gimbal = gimbal_vals(ig);
    mask_g = T.max_gimbal_deg == gimbal;

    R = nan(numel(slew_vals), numel(p_vals));
    Rob = nan(numel(slew_vals), numel(p_vals));
    for is = 1:numel(slew_vals)
        for ip = 1:numel(p_vals)
            mask = mask_g & T.p_unstable == p_vals(ip) & T.servo_slew_deg_s == slew_vals(is);
            if any(mask)
                row = find(mask, 1);
                R(is, ip) = T.regime_code(row);
                Rob(is, ip) = T.robustness(row);
            end
        end
    end

    subplot(2, 2, ig);
    imagesc(p_vals, slew_vals, R, [0 2]);
    axis xy;
    grid on;
    cmap = [0.85 0.30 0.25; 0.95 0.82 0.35; 0.40 0.75 0.45];
    colormap(gca, cmap);
    cb = colorbar;
    cb.Ticks = [0 1 2];
    cb.TickLabels = {'INFEASIBLE', 'FRAGILE', 'EASY'};
    xlabel('Instability demand p (1/s)');
    ylabel('Loaded slew rate (deg/s)');
    title(sprintf('Max gimbal = %.0f deg', gimbal));

    for is = 1:numel(slew_vals)
        for ip = 1:numel(p_vals)
            if isfinite(R(is, ip))
                text(p_vals(ip), slew_vals(is), sprintf('r=%.2f', Rob(is, ip)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8, ...
                    'FontWeight', 'bold', 'Color', 'k');
            end
        end
    end

    if ~isempty(flightT) && all(ismember({'p_unstable','servo_slew_deg_s','max_gimbal_deg'}, flightT.Properties.VariableNames))
        hold on;
        sel = abs(flightT.max_gimbal_deg - gimbal) < 1e-9;
        if any(sel)
            ft = flightT(sel, :);
            scatter(ft.p_unstable, ft.servo_slew_deg_s, 70, 'kd', 'filled', 'MarkerEdgeColor', 'w');
        end
    end
end

sgtitle('Experiment 1 Physical Regime Maps (robustness annotated)');
exportgraphics(fig, out_map_png, 'Resolution', 220);
close(fig);

fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 900 540]);
hold on;
clr = lines(numel(gimbal_vals));
for ig = 1:numel(gimbal_vals)
    gimbal = gimbal_vals(ig);
    y_frag = nan(size(p_vals));
    y_easy = nan(size(p_vals));
    for ip = 1:numel(p_vals)
        sub = T(T.max_gimbal_deg == gimbal & T.p_unstable == p_vals(ip), :);
        if isempty(sub)
            continue;
        end
        sub = sortrows(sub, 'servo_slew_deg_s', 'ascend');
        idx_frag = find(sub.regime_code >= 1, 1, 'first');
        idx_easy = find(sub.regime_code == 2, 1, 'first');
        if ~isempty(idx_frag)
            y_frag(ip) = sub.servo_slew_deg_s(idx_frag);
        end
        if ~isempty(idx_easy)
            y_easy(ip) = sub.servo_slew_deg_s(idx_easy);
        end
    end

    plot(p_vals, y_frag, 'o-', 'Color', clr(ig, :), 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Fragile+ frontier, gimbal %.0f deg', gimbal));
    plot(p_vals, y_easy, 's--', 'Color', clr(ig, :), 'LineWidth', 1.6, ...
        'DisplayName', sprintf('Easy frontier, gimbal %.0f deg', gimbal));
end

grid on;
xlabel('Instability demand p (1/s)');
ylabel('Required loaded slew rate (deg/s)');
title('Required Slew vs Instability (phase-transition frontier)');
legend('Location', 'northwest');
ylim([min(slew_vals) max(slew_vals)]);
exportgraphics(fig2, out_frontier_png, 'Resolution', 240);
close(fig2);
