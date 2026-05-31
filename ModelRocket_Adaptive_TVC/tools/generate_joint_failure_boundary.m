function generate_joint_failure_boundary(root_dir)
%GENERATE_JOINT_FAILURE_BOUNDARY  Explore where JOINT_ADAPTIVE fails and why.
%
%  Sweeps disturbance amplitude and gust sigma under realistic sensing.
%  Computes pass-fraction and classifies dominant failure cause:
%   0 pass
%   1 false early saturation trigger
%   2 late slew-fault detection
%   3 confidence collapse / estimator contamination
%   4 authority exhausted (disturbance beyond envelope)
%   5 safety-shield dominated (controller clipped by safety envelope)

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

graph_dir = fullfile(root_dir, 'outputs', 'graphs');
data_dir  = fullfile(root_dir, 'outputs', 'data');
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end
if ~exist(data_dir,  'dir'), mkdir(data_dir);  end

cfg = rocket_defaults();
cfg.plant.theta0 = deg2rad(5);
sc = rocket_scenario("SLEW_DEGRADATION", cfg);
sc.t_end = 10.0;
sc.disturbance_freq_hz = 1.5;

amp_grid  = 1.0:0.5:6.0;
gust_grid = [0.10 0.20 0.30 0.40 0.50 0.65 0.80 1.00];
n_seeds = 10;

pass_frac = nan(numel(gust_grid), numel(amp_grid));
dom_cause = nan(numel(gust_grid), numel(amp_grid));

rows = [];  % [gust_idx amp_idx seed amp gust pass cause rms peak falseSat lateDet confCollapse authorityExhaust shieldDom]

for gi = 1:numel(gust_grid)
    for ai = 1:numel(amp_grid)
        counts = zeros(1,6);  % causes 0..5 stored at idx+1
        n_pass = 0;

        for seed = 1:n_seeds
            rl = struct();
            rl.gust_std = gust_grid(gi);
            sc.disturbance_amp = amp_grid(ai);

            out = simulate_case_realistic("JOINT_ADAPTIVE", sc, cfg, seed, rl);
            pre = out.time < sc.fault_time & out.time > 1.0;
            pm  = out.time >= sc.fault_time;

            rms_post = rad2deg(rms(out.theta(pm)));
            pk_post  = rad2deg(max(abs(out.theta(pm))));
            pass = (rms_post < 20) && (pk_post < 60);

            false_sat = false;
            late_det  = false;
            conf_collapse = false;
            authority_exhaust = false;
            shield_dominated = false;

            if any(pre)
                sat_pre = mean(out.saturating(pre), 'omitnan');
                ss_pre  = mean(out.slew_scale(pre), 'omitnan');
                false_sat = (sat_pre > 0.25) && (ss_pre < 0.85);
            end

            if any(pm)
                tpost = out.time(pm) - sc.fault_time;
                ssp   = out.slew_scale(pm);
                kgp   = out.keff_gate(pm);
                kep   = out.keff_est(pm);

                idx_det = find(ssp < 0.60, 1, 'first');
                if isempty(idx_det)
                    late_det = true;
                else
                    late_det = tpost(idx_det) > 0.75;
                end

                contam_mask = (ssp < 0.70) & (kgp > 0.5);
                contam_frac = mean(contam_mask, 'omitnan');
                keff_dev = abs(median(kep, 'omitnan') - cfg.plant.keff_nom) / cfg.plant.keff_nom;

                conf_post = mean(out.confidence(pm), 'omitnan');
                shield_frac = mean(out.shield_active(pm), 'omitnan');
                conf_collapse = (conf_post < 0.30) || (contam_frac > 0.20) || (keff_dev > 0.60);
                authority_exhaust = (pk_post > 120) && (mean(ssp, 'omitnan') < 0.35);
                shield_dominated = (shield_frac > 0.60) && ~authority_exhaust;
            end

            % cause classification
            if pass
                cause = 0;
                n_pass = n_pass + 1;
            else
                if false_sat
                    cause = 1;
                elseif late_det
                    cause = 2;
                elseif conf_collapse
                    cause = 3;
                elseif authority_exhaust
                    cause = 4;
                elseif shield_dominated
                    cause = 5;
                else
                    cause = 4;
                end
            end

            counts(cause + 1) = counts(cause + 1) + 1;
            rows(end+1,:) = [gi, ai, seed, amp_grid(ai), gust_grid(gi), pass, cause, ...
                rms_post, pk_post, false_sat, late_det, conf_collapse, authority_exhaust, shield_dominated]; %#ok<AGROW>
        end

        pass_frac(gi, ai) = n_pass / n_seeds;
        [~, imax] = max(counts);
        dom_cause(gi, ai) = imax - 1;
    end
end

% Save raw and grid summaries
rawT = array2table(rows, 'VariableNames', ...
    {'gust_idx','amp_idx','seed','disturb_amp','gust_std','pass','cause_code','rms_post_deg','peak_post_deg','false_sat','late_detection','conf_collapse','authority_exhaust','shield_dominated'});
raw_csv = fullfile(data_dir, 'joint_failure_boundary_raw.csv');
writetable(rawT, raw_csv);

grid_rows = [];
for gi = 1:numel(gust_grid)
    for ai = 1:numel(amp_grid)
        grid_rows(end+1,:) = [amp_grid(ai), gust_grid(gi), pass_frac(gi,ai), dom_cause(gi,ai)]; %#ok<AGROW>
    end
end
G = array2table(grid_rows, 'VariableNames', {'disturb_amp','gust_std','pass_fraction','dominant_cause'});
grid_csv = fullfile(data_dir, 'joint_failure_boundary_grid.csv');
writetable(G, grid_csv);

% Figure
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [70 70 1400 560]);

ax1 = subplot(1,2,1);
imagesc(ax1, amp_grid, gust_grid, pass_frac);
set(ax1, 'YDir', 'normal', 'FontSize', 10);
colormap(ax1, parula(256));
cb1 = colorbar(ax1); cb1.Label.String = 'Pass fraction';
xlabel(ax1, 'Disturbance amplitude', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax1, 'Gust \sigma (rad/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title(ax1, 'JOINT_ADAPTIVE pass boundary (10 seeds/cell)', 'FontSize', 12, 'FontWeight', 'bold');

ax2 = subplot(1,2,2);
imagesc(ax2, amp_grid, gust_grid, dom_cause);
set(ax2, 'YDir', 'normal', 'FontSize', 10);
% custom categorical colormap for causes 0..5
cmap = [0.15 0.60 0.20;   % 0 pass (green)
        0.98 0.75 0.20;   % 1 false sat
        0.95 0.45 0.10;   % 2 late detect
    0.65 0.35 0.75;   % 3 confidence collapse
    0.70 0.20 0.20;   % 4 authority exhausted
    0.10 0.45 0.75];  % 5 shield dominated
colormap(ax2, cmap);
caxis(ax2, [0 5]);
cb2 = colorbar(ax2);
cb2.Ticks = 0:5;
cb2.TickLabels = {'pass','false sat','late detect','conf collapse','authority','shield'};
cb2.Label.String = 'Dominant cause';
xlabel(ax2, 'Disturbance amplitude', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax2, 'Gust \sigma (rad/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title(ax2, 'Dominant failure cause map', 'FontSize', 12, 'FontWeight', 'bold');

sgtitle({'Joint controller failure boundary and root-cause map (realistic model)'; ...
    'Cause codes: 1 false sat, 2 late detection, 3 confidence collapse, 4 authority exhausted, 5 shield dominated'}, ...
    'FontSize', 13, 'FontWeight', 'bold');

png_path = fullfile(graph_dir, 'joint_failure_boundary.png');
exportgraphics(fig, png_path, 'Resolution', 240);
close(fig);

fprintf('\n=== JOINT FAILURE BOUNDARY SUMMARY ===\n');
fprintf('Max disturbance with >=0.8 pass fraction by gust level:\n');
for gi = 1:numel(gust_grid)
    idx = find(pass_frac(gi,:) >= 0.8, 1, 'last');
    if isempty(idx)
        fprintf('  gust=%4.2f  no region with pass>=0.8\n', gust_grid(gi));
    else
        fprintf('  gust=%4.2f  amp<=%4.2f\n', gust_grid(gi), amp_grid(idx));
    end
end

fprintf('Saved figure: %s\n', png_path);
fprintf('Saved data:   %s\n', raw_csv);
fprintf('Saved data:   %s\n\n', grid_csv);
end
