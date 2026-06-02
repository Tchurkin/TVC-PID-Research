% run_pid_slew_probe.m
% Focused probe for a new implementation direction:
%   a slew-aware PID that preserves derivative damping when actuator
%   bandwidth collapses.
%
% Cheap falsifier for the hypothesis:
%   if this controller cannot beat fixed PID on the realistic
%   SLEW_DEGRADATION scenario using the same nominal gains, kill it.
% We sweep both the original stable plant (p=0) and representative
% unstable poles, because success only on the stable plant would be weak
% evidence for the actual model-rocket regime.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));

cfg_base = rocket_defaults();
% This probe is intentionally a severe-collapse stress test, not the
% current hobby-nominal actuator stack. Keep the old slow-slew setup here
% so the controller branch remains an explicitly bounded appendix result.
cfg_base.plant.slew_max = 12.0;
cfg_base.plant.theta0 = deg2rad(5);
cfg_base.plant.aero_damp = 0.5;
cfg_base.controllers.PID_SLEW_AWARE.slew_nominal = cfg_base.plant.slew_max;
cfg_base.controllers.PID_SLEW_AWARE.slew_min = 0.10 * cfg_base.plant.slew_max;
cfg_base.controllers.JOINT_ADAPTIVE.slew_nominal = cfg_base.plant.slew_max;
cfg_base.controllers.JOINT_ADAPTIVE.slew_min = 0.10 * cfg_base.plant.slew_max;

p_values = [0 4 6 8];

gain_grid = [ ...
    15 5; ...
    20 5; ...
    20 8; ...
    25 8];

seeds = 1:6;
theta_fail = deg2rad(40);

rows = cell(0, 7);

fprintf('=== PID SLEW-AWARE PROBE (legacy severe-collapse stress test) ===\n');
fprintf('sweep: p in %s, gains in %d pairs\n\n', mat2str(p_values), size(gain_grid,1));

for p = p_values
    cfg = cfg_base;
    cfg.plant.p_unstable = p;
    sc = rocket_scenario('SLEW_DEGRADATION', cfg);
    sc.t_end = 8.0;

    fprintf('p = %g  | fault t=%.1fs, post-fault slew scale=%.2f\n', ...
        p, sc.fault_time, sc.slew_scale_post);

    for i = 1:size(gain_grid,1)
        Kp = gain_grid(i,1);
        Kd = gain_grid(i,2);

        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Ki = 0;
        cfg.controllers.PID.Kd = Kd;
        cfg.controllers.PID.u_max = cfg.plant.u_max;
        cfg.controllers.PID.i_lim = cfg.plant.u_max;

        cfg.controllers.PID_SLEW_AWARE.Kp = Kp;
        cfg.controllers.PID_SLEW_AWARE.Ki = 0;
        cfg.controllers.PID_SLEW_AWARE.Kd = Kd;
        cfg.controllers.PID_SLEW_AWARE.u_max = cfg.plant.u_max;
        cfg.controllers.PID_SLEW_AWARE.i_lim = cfg.plant.u_max;

        for ctrl_name = ["PID", "PID_SLEW_AWARE"]
            wins = 0;
            rms_post = nan(numel(seeds), 1);
            for s = 1:numel(seeds)
                out = simulate_case_realistic(ctrl_name, sc, cfg, seeds(s), struct());
                pm = out.time >= sc.fault_time;
                rms_post(s) = sqrt(mean(out.theta(pm).^2)) * 180/pi;
                if all(abs(out.theta(pm)) < theta_fail)
                    wins = wins + 1;
                end
            end
            pass_rate = wins / numel(seeds);
            mean_rms = mean(rms_post);
            median_rms = median(rms_post);
            fprintf('  %-15s  Kp=%4.0f Kd=%4.0f  pass=%.0f%%  medRMS=%6.2f deg  meanRMS=%8.2f deg\n', ...
                ctrl_name, Kp, Kd, 100*pass_rate, median_rms, mean_rms);
            rows(end+1,:) = {char(ctrl_name), p, Kp, Kd, pass_rate, median_rms, mean_rms}; %#ok<SAGROW>
        end
    end

    % Reference point from the existing adaptive stack.
    joint_wins = 0;
    joint_rms = nan(numel(seeds), 1);
    for s = 1:numel(seeds)
        out = simulate_case_realistic('JOINT_ADAPTIVE', sc, cfg, seeds(s), struct());
        pm = out.time >= sc.fault_time;
        joint_rms(s) = sqrt(mean(out.theta(pm).^2)) * 180/pi;
        if all(abs(out.theta(pm)) < theta_fail)
            joint_wins = joint_wins + 1;
        end
    end
    fprintf('  %-15s  defaults      pass=%.0f%%  medRMS=%6.2f deg  meanRMS=%8.2f deg\n\n', ...
        'JOINT_ADAPTIVE', 100*(joint_wins/numel(seeds)), median(joint_rms), mean(joint_rms));
    rows(end+1,:) = {'JOINT_ADAPTIVE', p, NaN, NaN, joint_wins / numel(seeds), median(joint_rms), mean(joint_rms)}; %#ok<SAGROW>
end

T = cell2table(rows, 'VariableNames', ...
    {'controller','p_unstable','Kp','Kd','pass_rate','post_fault_median_rms_deg','post_fault_mean_rms_deg'});
writetable(T, fullfile('results', 'pid_slew_probe.csv'));
fprintf('\nSaved: experiments/results/pid_slew_probe.csv\n');