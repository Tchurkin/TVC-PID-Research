% run_pid_slew_regime_map.m
% Best-tuned comparison of naive PID vs PID_SLEW_AWARE across
% hobby-relevant nominal loaded gimbal slews and slew-loss severities.
%
% Honest question this script answers:
%   once nominal TVC speed is moved out of the legacy ~15 deg/s corner and
%   into a more believable 45-90 deg/s loaded band, does the lightweight
%   controller still matter?
%
% Method:
%   - realistic plant, same SLEW_DEGRADATION scenario
%   - p_unstable in {4,6,8}
%   - nominal loaded gimbal slew in {45,60,75,90} deg/s
%   - post-fault slew scale in {1.00,0.50,0.35,0.25}
%   - small gain grid, best-tuned separately for PID and PID_SLEW_AWARE
%
% Output:
%   experiments/results/pid_slew_regime_raw.csv
%   experiments/results/pid_slew_regime_summary.csv

clear; clc;

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(genpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src')));

cfg_base = rocket_defaults();
cfg_base.plant.theta0 = deg2rad(5);
cfg_base.plant.aero_damp = 0.5;

p_values = [4 6 8];
nominal_slew_dps = [45 60 75 90];
slew_scale_post_values = [1.00 0.50 0.35 0.25];
gain_grid = [ ...
    15 5; ...
    20 5; ...
    20 8; ...
    25 8];
seeds = 1:6;
theta_fail = deg2rad(40);
code_per_deg_per_s = 12 / 15;

ctrl_names = ["PID", "PID_SLEW_AWARE"];
raw_rows = cell(0, 10);
summary_rows = cell(0, 14);

fprintf('=== PID vs PID_SLEW_AWARE REGIME MAP ===\n');
fprintf('p in %s\n', mat2str(p_values));
fprintf('nominal loaded slew (deg/s) in %s\n', mat2str(nominal_slew_dps));
fprintf('post-fault slew scales in %s\n', mat2str(slew_scale_post_values));
fprintf('gain grid has %d pairs\n\n', size(gain_grid,1));

for p = p_values
    fprintf('p = %g\n', p);
    for nominal_slew = nominal_slew_dps
        for slew_scale_post = slew_scale_post_values
            cfg = cfg_base;
            cfg.plant.p_unstable = p;
            cfg.plant.slew_max = nominal_slew * code_per_deg_per_s;

            sc = rocket_scenario('SLEW_DEGRADATION', cfg);
            sc.slew_scale_post = slew_scale_post;
            sc.t_end = 8.0;

            best = repmat(empty_best(), numel(ctrl_names), 1);

            for gi = 1:size(gain_grid,1)
                Kp = gain_grid(gi,1);
                Kd = gain_grid(gi,2);
                cfg_gain = apply_pid_gains(cfg, Kp, Kd);

                for ci = 1:numel(ctrl_names)
                    ctrl_name = ctrl_names(ci);
                    [pass_rate, median_rms, mean_rms] = eval_controller(ctrl_name, sc, cfg_gain, seeds, theta_fail);

                    raw_rows(end+1,:) = { ...
                        char(ctrl_name), p, nominal_slew, nominal_slew * slew_scale_post, ...
                        slew_scale_post, Kp, Kd, pass_rate, median_rms, mean_rms}; %#ok<SAGROW>

                    if is_better(pass_rate, median_rms, mean_rms, best(ci))
                        best(ci).Kp = Kp;
                        best(ci).Kd = Kd;
                        best(ci).pass_rate = pass_rate;
                        best(ci).median_rms = median_rms;
                        best(ci).mean_rms = mean_rms;
                    end
                end
            end

            pid_best = best(1);
            psa_best = best(2);
            effect_label = classify_effect(pid_best, psa_best);
            summary_rows(end+1,:) = { ...
                p, nominal_slew, nominal_slew * slew_scale_post, slew_scale_post, ...
                pid_best.Kp, pid_best.Kd, pid_best.pass_rate, pid_best.median_rms, pid_best.mean_rms, ...
                psa_best.Kp, psa_best.Kd, psa_best.pass_rate, psa_best.median_rms, effect_label}; %#ok<SAGROW>

            fprintf(['  nominal=%3g dps  post=%5.2f dps  ' ...
                'PID(best %2.0f/%2.0f)=%.0f%%  PSA(best %2.0f/%2.0f)=%.0f%%  %s\n'], ...
                nominal_slew, nominal_slew * slew_scale_post, ...
                pid_best.Kp, pid_best.Kd, 100*pid_best.pass_rate, ...
                psa_best.Kp, psa_best.Kd, 100*psa_best.pass_rate, ...
                effect_label);
        end
    end
    fprintf('\n');
end

raw = cell2table(raw_rows, 'VariableNames', { ...
    'controller', 'p_unstable', 'nominal_loaded_slew_dps', ...
    'post_fault_loaded_slew_dps', 'slew_scale_post', ...
    'Kp', 'Kd', 'pass_rate', 'post_fault_median_rms_deg', ...
    'post_fault_mean_rms_deg'});

summary = cell2table(summary_rows, 'VariableNames', { ...
    'p_unstable', 'nominal_loaded_slew_dps', 'post_fault_loaded_slew_dps', ...
    'slew_scale_post', 'pid_best_Kp', 'pid_best_Kd', 'pid_best_pass_rate', ...
    'pid_best_median_rms_deg', 'pid_best_mean_rms_deg', ...
    'psa_best_Kp', 'psa_best_Kd', 'psa_best_pass_rate', ...
    'psa_best_median_rms_deg', 'effect_label'});

writetable(raw, fullfile(here, 'results', 'pid_slew_regime_raw.csv'));
writetable(summary, fullfile(here, 'results', 'pid_slew_regime_summary.csv'));

fprintf('Saved: experiments/results/pid_slew_regime_raw.csv\n');
fprintf('Saved: experiments/results/pid_slew_regime_summary.csv\n');
labels = string(summary.effect_label);
[uniq_labels, ~, label_idx] = unique(labels);
label_counts = accumarray(label_idx, 1);
disp(table(uniq_labels, label_counts, 'VariableNames', {'effect_label', 'count'}));

function cfg = apply_pid_gains(cfg, Kp, Kd)
cfg.controllers.PID.Kp = Kp;
cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.Kd = Kd;
cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;

cfg.controllers.PID_SLEW_AWARE.Kp = Kp;
cfg.controllers.PID_SLEW_AWARE.Ki = 0.0;
cfg.controllers.PID_SLEW_AWARE.Kd = Kd;
cfg.controllers.PID_SLEW_AWARE.u_max = cfg.plant.u_max;
cfg.controllers.PID_SLEW_AWARE.i_lim = cfg.plant.u_max;
cfg.controllers.PID_SLEW_AWARE.slew_nominal = cfg.plant.slew_max;
cfg.controllers.PID_SLEW_AWARE.slew_min = 0.10 * cfg.plant.slew_max;
end

function [pass_rate, median_rms, mean_rms] = eval_controller(ctrl_name, sc, cfg, seeds, theta_fail)
rms_post = nan(numel(seeds), 1);
wins = 0;
for si = 1:numel(seeds)
    out = simulate_case_realistic(ctrl_name, sc, cfg, seeds(si), struct());
    pm = out.time >= sc.fault_time;
    rms_post(si) = sqrt(mean(out.theta(pm).^2)) * 180 / pi;
    if all(abs(out.theta(pm)) < theta_fail)
        wins = wins + 1;
    end
end
pass_rate = wins / numel(seeds);
median_rms = median(rms_post);
mean_rms = mean(rms_post);
end

function tf = is_better(pass_rate, median_rms, mean_rms, best)
tol = 1e-9;
if pass_rate > best.pass_rate + tol
    tf = true;
    return;
end
if abs(pass_rate - best.pass_rate) <= tol && median_rms < best.median_rms - tol
    tf = true;
    return;
end
if abs(pass_rate - best.pass_rate) <= tol && abs(median_rms - best.median_rms) <= tol ...
        && mean_rms < best.mean_rms - tol
    tf = true;
    return;
end
tf = false;
end

function out = empty_best()
out = struct('Kp', NaN, 'Kd', NaN, 'pass_rate', -inf, 'median_rms', inf, 'mean_rms', inf);
end

function label = classify_effect(pid_best, psa_best)
pass_delta = psa_best.pass_rate - pid_best.pass_rate;
tol = 1e-9;
if pid_best.pass_rate >= 0.999 && psa_best.pass_rate >= 0.999
    label = 'BOTH_FINE';
elseif psa_best.pass_rate > pid_best.pass_rate + tol
    if psa_best.pass_rate >= 0.999 && pid_best.pass_rate < 0.999
        label = 'PSA_RESCUE';
    else
        label = 'PSA_HELPS';
    end
elseif pid_best.pass_rate > psa_best.pass_rate + tol
    label = 'PSA_HURTS';
elseif abs(pass_delta) <= tol && psa_best.median_rms < pid_best.median_rms - tol
    label = 'PSA_SMOOTHER';
else
    label = 'NEAR_TIE';
end
end