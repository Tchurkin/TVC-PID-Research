function generate_robustness_sweep(root_dir)
%GENERATE_ROBUSTNESS_SWEEP  Multi-seed Monte Carlo over (controller, fault).
%
%  For each (controller, scenario) pair, sweeps disturbance amplitude
%  and computes RMS pitch error post-fault across seeds. Produces:
%   - outputs/data/robustness_sweep.csv
%   - outputs/graphs/robustness_sweep.png

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
data_dir  = fullfile(root_dir, 'outputs', 'data');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end
if ~exist(data_dir,  'dir'), mkdir(data_dir);  end

ctrl_names = ["FIXED_LQR", "ADAPTIVE_KEFF_LQR", "SLEW_ADAPTIVE", "PCH_LQR", "JOINT_ADAPTIVE"];
pretty     = ["Fixed LQR", "keff-only", "slew-only", "PCH", "Joint Adaptive"];
colors     = {[0.15 0.25 0.85], [0.85 0.45 0.10], [0.60 0.20 0.20], [0.40 0.40 0.40], [0.0 0.55 0.22]};

scenarios   = ["SLEW_DEGRADATION", "HIGH_KEFF_FAULT"];
sc_pretty   = ["Slew Degradation", "Motor Authority Drop"];
amps        = [0.5 1.0 1.5 2.0 2.5 3.0 3.5 4.0];
n_seeds     = 8;

cfg = rocket_defaults();

results = [];   % rows: scenario, ctrl, amp, seed, rms_post, pk_post

for s = 1:numel(scenarios)
    sc_kind = scenarios(s);
    for c = 1:numel(ctrl_names)
        for amp = amps
            for seed = 1:n_seeds
                cfg2 = cfg;
                cfg2.plant.theta0 = deg2rad(5);
                sc = rocket_scenario(sc_kind, cfg2);
                sc.t_end = 10.0;
                if sc_kind == "SLEW_DEGRADATION"
                    sc.disturbance_amp = amp;
                    sc.disturbance_freq_hz = 1.5;
                else
                    % HIGH_KEFF_FAULT: amp acts as multiplier on baseline disturb_scale_post
                    sc.disturbance_amp = 0.30;
                    sc.disturb_scale_post = 4.0 * amp;   % amp=1 -> ×4 base, amp=4 -> ×16
                end
                out = simulate_case_realistic(ctrl_names(c), sc, cfg2, seed, struct());
                pm  = out.time >= sc.fault_time;
                rms_post = rad2deg(rms(out.theta(pm)));
                pk_post  = rad2deg(max(abs(out.theta(pm))));
                results(end+1, :) = [s, c, amp, seed, rms_post, pk_post]; %#ok<AGROW>
            end
        end
    end
end

% Save CSV
T = array2table(results, 'VariableNames', ...
    {'scenario_idx','ctrl_idx','disturb_amp','seed','rms_post_deg','peak_post_deg'});
csv_path = fullfile(data_dir, 'robustness_sweep.csv');
writetable(T, csv_path);
fprintf('Saved data: %s\n', csv_path);

% ---- Figure: RMS post vs disturbance amplitude, per scenario --------
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1280 620]);
for s = 1:numel(scenarios)
    ax = subplot(1, 2, s);
    hold(ax,'on'); grid(ax,'on');
    set(ax, 'YScale', 'log');
    for c = 1:numel(ctrl_names)
        means = nan(size(amps));
        stds  = nan(size(amps));
        for ai = 1:numel(amps)
            mask = results(:,1)==s & results(:,2)==c & results(:,3)==amps(ai);
            vals = results(mask, 5);
            means(ai) = mean(vals);
            stds(ai)  = std(vals);
        end
        errorbar(ax, amps, means, stds, ...
            'Color', colors{c}, 'LineWidth', 2.2, 'Marker', 'o', ...
            'MarkerFaceColor', colors{c}, 'MarkerSize', 6, ...
            'DisplayName', pretty(c));
    end
    xlabel(ax, 'Disturbance amplitude', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel(ax, 'Post-fault RMS pitch (deg, log10 scale)', 'FontSize', 11, 'FontWeight', 'bold');
    title(ax, sc_pretty(s), 'FontSize', 12, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northwest', 'FontSize', 10);
    ylim(ax, [0.05 1e4]);
    set(ax, 'FontSize', 10);
end
sgtitle({'Multi-seed robustness sweep'; ...
    sprintf('Mean ± 1σ over %d seeded realistic runs  |  log10 RMS (10^-1 = 0.1 deg)  |  lower is better', n_seeds)}, ...
    'FontSize', 13, 'FontWeight', 'bold');

png_path = fullfile(graph_dir, 'robustness_sweep.png');
exportgraphics(fig, png_path, 'Resolution', 220);
close(fig);
fprintf('Saved figure: %s\n', png_path);

% ---- Console summary at one operating point ------------------------
fprintf('\n=== ROBUSTNESS SUMMARY (mean RMS post-fault, deg, %d seeds) ===\n', n_seeds);
fprintf('  %-22s', 'Disturb amp');
for ai = 1:numel(amps); fprintf('  %5.1f', amps(ai)); end; fprintf('\n');
for s = 1:numel(scenarios)
    fprintf('--- %s ---\n', sc_pretty(s));
    for c = 1:numel(ctrl_names)
        fprintf('  %-22s', pretty(c));
        for ai = 1:numel(amps)
            mask = results(:,1)==s & results(:,2)==c & results(:,3)==amps(ai);
            fprintf('  %5.1f', mean(results(mask, 5)));
        end
        fprintf('\n');
    end
end
end
