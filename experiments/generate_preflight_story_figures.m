function generate_preflight_story_figures()
%GENERATE_PREFLIGHT_STORY_FIGURES  Build visuals for the bench-calibrated
% preflight validator/autotune direction.

here = fileparts(mfilename('fullpath'));
resdir = fullfile(here, 'results');
gfxdir = fullfile(resdir, 'graphs');
if ~exist(gfxdir, 'dir')
    mkdir(gfxdir);
end

Tval = readtable(fullfile(resdir, 'validator_calibration.csv'));
Tscale = readtable(fullfile(resdir, 'scaling_law_slew_vs_p.csv'));
Tfit = readtable(fullfile(resdir, 'scaling_law_fit.csv'));
Tservo = readtable(fullfile(resdir, 'servo_pareto_all.csv'));
Tfront = readtable(fullfile(resdir, 'servo_pareto_frontier.csv'));
Ts2r = readtable(fullfile(resdir, 's2r_factor_screen.csv'));
Tcmp = readtable(fullfile(resdir, 'preflight_nominal_vs_measured.csv'));

set(groot, 'defaultFigureVisible', 'off');

make_dashboard(Tval, Tscale, Tfit, Tservo, Tfront, Ts2r, ...
    fullfile(gfxdir, 'preflight_story_dashboard.png'));
make_comparison_figure(Tcmp, ...
    fullfile(gfxdir, 'preflight_nominal_vs_measured.png'));

fprintf('Saved: experiments/results/graphs/preflight_story_dashboard.png\n');
fprintf('Saved: experiments/results/graphs/preflight_nominal_vs_measured.png\n');
end


function make_dashboard(Tval, Tscale, Tfit, Tservo, Tfront, Ts2r, out_path)
fig = figure('Position', [100 100 1400 980], 'Color', 'w');
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Bench-Calibrated Preflight Story', 'FontSize', 18, 'FontWeight', 'bold');

nexttile;
hold on;
verdicts = {'GO', 'MARGINAL', 'NOGO'};
xpos = 1:numel(verdicts);
for i = 1:numel(verdicts)
    sel = strcmp(Tval.verdict, verdicts{i});
    y = Tval.actual_success(sel);
    jitter = linspace(-0.12, 0.12, max(1, numel(y)))';
    scatter(xpos(i) + jitter(1:numel(y)), y, 36, 'filled', ...
        'MarkerFaceAlpha', 0.70);
    if ~isempty(y)
        plot([xpos(i)-0.18 xpos(i)+0.18], [mean(y) mean(y)], 'k-', 'LineWidth', 2);
    end
end
yline(0.80, '--', 'GO threshold', 'Color', [0.15 0.55 0.15], 'LineWidth', 1.2);
yline(0.50, '--', 'NOGO threshold', 'Color', [0.65 0.25 0.25], 'LineWidth', 1.2);
xlim([0.5 3.5]);
ylim([0 1.05]);
set(gca, 'XTick', xpos, 'XTickLabel', verdicts, 'FontSize', 11);
ylabel('Actual Success Rate');
title('Validator Calibration');
grid on;
hold off;

nexttile;
hold on;
scatter(Tscale.p_unstable, Tscale.slew_min_observed, 70, [0.05 0.35 0.75], 'filled');
p_line = linspace(min(Tscale.p_unstable), max(Tscale.p_unstable), 200);
a = Tfit.coef_a(1);
alpha = Tfit.exponent_alpha(1);
R2 = Tfit.R2(1);
plot(p_line, a * p_line.^alpha, 'Color', [0.85 0.25 0.15], 'LineWidth', 2.5);
xlabel('Instability Demand p');
ylabel('Boundary Slew (deg/s)');
title('Empirical Boundary Law');
grid on;
text(min(p_line) + 0.1, max(Tscale.slew_min_observed) * 0.90, ...
    sprintf('slew_{min}(p)=%.3f p^{%.2f}\nR^2=%.2f', a, alpha, R2), ...
    'FontSize', 11, 'BackgroundColor', [1 1 1 0.75], 'Margin', 6);
hold off;

nexttile;
hold on;
scatter(Tservo.price_usd, Tservo.p_max_predicted, 52, [0.70 0.70 0.70], 'filled', ...
    'MarkerFaceAlpha', 0.7);
plot(Tfront.price_usd, Tfront.p_max_predicted, '-o', 'Color', [0.85 0.25 0.15], ...
    'LineWidth', 2.0, 'MarkerFaceColor', [0.85 0.25 0.15]);
for i = 1:height(Tfront)
    text(Tfront.price_usd(i) + 2, Tfront.p_max_predicted(i) + 0.15, ...
        Tfront.servo{i}, 'FontSize', 10);
end
xlabel('Servo Price (USD)');
ylabel('Predicted p_{max}');
title('Consumer Servo Pareto Frontier');
grid on;
hold off;

nexttile;
regimes = {'LOW_DEMAND', 'BOUNDARY', 'ACTUATOR_LIMITED'};
factors = {'SENSOR_LATENCY', 'SENSOR_NOISE', 'ACTUATOR_NONLINEARITY', 'GUST', 'KEFF_DRIFT', 'ALL_REALISTIC'};
H = nan(numel(factors), numel(regimes));
for i = 1:numel(factors)
    for j = 1:numel(regimes)
        sel = strcmp(Ts2r.factor_profile, factors{i}) & strcmp(Ts2r.regime, regimes{j});
        if any(sel)
            H(i, j) = Ts2r.success_drop(find(sel, 1));
        end
    end
end
imagesc(H);
axis tight;
colormap(gca, parula(256));
colorbar;
set(gca, 'XTick', 1:numel(regimes), 'XTickLabel', strrep(regimes, '_', '\_'), ...
    'YTick', 1:numel(factors), 'YTickLabel', strrep(factors, '_', '\_'), 'FontSize', 10);
title('Which Sim-to-Real Factors Hurt, by Regime');
for i = 1:size(H,1)
    for j = 1:size(H,2)
        if isfinite(H(i,j))
            text(j, i, sprintf('%.2f', H(i,j)), 'HorizontalAlignment', 'center', ...
                'Color', 'w', 'FontWeight', 'bold', 'FontSize', 10);
        end
    end
end

safe_export(fig, out_path);
close(fig);
end


function make_comparison_figure(Tcmp, out_path)
fig = figure('Position', [110 110 1400 980], 'Color', 'w');
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Why Measured Preflight Tuning Can Matter', 'FontSize', 18, 'FontWeight', 'bold');

slews = sort(unique(Tcmp.slew_scale), 'descend');
utils = sort(unique(Tcmp.util_frac), 'descend');

nexttile;
plot_delta_heatmap(Tcmp, 10, slews, utils, 'MEASURED_PD', 'VALIDATOR_LQR');
title('Measured PD - Validator LQR Success, p=10');

nexttile;
plot_delta_heatmap(Tcmp, 12, slews, utils, 'MEASURED_PD', 'VALIDATOR_LQR');
title('Measured PD - Validator LQR Success, p=12');

nexttile;
plot_policy_bar(Tcmp, 'success_rate', 'Mean Success on Degraded Cases');

nexttile;
plot_policy_bar(Tcmp, 'rms_deg', 'Mean RMS Error on Degraded Cases');

safe_export(fig, out_path);
close(fig);
end


function plot_delta_heatmap(Tcmp, p_val, slews, utils, num_policy, den_policy)
M = nan(numel(utils), numel(slews));
for iu = 1:numel(utils)
    for is = 1:numel(slews)
        sel_num = Tcmp.p_unstable == p_val & strcmp(Tcmp.policy, num_policy) & ...
                  abs(Tcmp.slew_scale - slews(is)) < 1e-9 & abs(Tcmp.util_frac - utils(iu)) < 1e-9;
        sel_den = Tcmp.p_unstable == p_val & strcmp(Tcmp.policy, den_policy) & ...
                  abs(Tcmp.slew_scale - slews(is)) < 1e-9 & abs(Tcmp.util_frac - utils(iu)) < 1e-9;
        if any(sel_num) && any(sel_den)
            M(iu, is) = Tcmp.success_rate(find(sel_num,1)) - Tcmp.success_rate(find(sel_den,1));
        end
    end
end

imagesc(M, [-1 1]);
axis tight;
colormap(gca, turbo(256));
colorbar;
set(gca, 'XTick', 1:numel(slews), 'XTickLabel', slews, ...
    'YTick', 1:numel(utils), 'YTickLabel', utils, 'FontSize', 10);
xlabel('Actual Slew Scale');
ylabel('Endpoint Utilization');
for iu = 1:size(M,1)
    for is = 1:size(M,2)
        if isfinite(M(iu,is))
            text(is, iu, sprintf('%+.2f', M(iu,is)), 'HorizontalAlignment', 'center', ...
                'Color', 'w', 'FontWeight', 'bold', 'FontSize', 10);
        end
    end
end
end


function plot_policy_bar(Tcmp, metric_name, ttl)
p_values = sort(unique(Tcmp.p_unstable));
policies = {'NOMINAL_PD', 'MEASURED_PD', 'VALIDATOR_LQR'};
Y = nan(numel(p_values), numel(policies));
for ip = 1:numel(p_values)
    for jp = 1:numel(policies)
        sel = Tcmp.p_unstable == p_values(ip) & strcmp(Tcmp.policy, policies{jp}) & ...
              (Tcmp.slew_scale < 0.999 | Tcmp.util_frac < 0.999);
        if any(sel)
            Y(ip, jp) = mean(Tcmp.(metric_name)(sel));
        end
    end
end

bar(Y, 'grouped');
set(gca, 'XTick', 1:numel(p_values), 'XTickLabel', p_values, 'FontSize', 10);
xlabel('Instability Demand p');
ylabel(strrep(metric_name, '_', ' '));
title(ttl);
legend(strrep(policies, '_', '\_'), 'Location', 'northwest');
grid on;
end


function safe_export(fig, out_path)
try
    exportgraphics(fig, out_path, 'Resolution', 180);
catch
    saveas(fig, out_path);
end
end