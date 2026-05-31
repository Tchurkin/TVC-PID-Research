% run_resonance_sweep.m
% Search for a frequency-coupled failure band: even with fair PD tuning,
% does a sinusoidal disturbance near the unstable-plant natural frequency
% or actuator bandwidth cause a sharp collapse in success?

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));

cfg = rocket_defaults();
cfg.t_end_demo = 3.0;
cfg.plant.aero_damp = 0.5;
cfg.plant.control_eff = 8.0;
cfg.plant.slew_max = 12.0;
cfg.plant.u_max = 12.0;

p_list = [2 4 6 8];
tau_list = [0.03 0.05 0.08 0.12];
freq_list_hz = [0.2 0.4 0.7 1.0 1.4 2.0 3.0 4.5 6.0];

Kp_grid = [1 3 10 30];
Kd_grid = [0.3 1 3 10];

seeds_tune = 1:2;
seeds_eval = 1:3;
theta0_list = deg2rad([3 6]);

cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;

realism = struct();

rows = [];
summary_rows = [];

fprintf('=== RESONANCE SWEEP ===\n');

for ip = 1:numel(p_list)
    cfg.plant.p_unstable = p_list(ip);
    nat_freq_hz = p_list(ip) / (2*pi);

    for it = 1:numel(tau_list)
        cfg.plant.tau_act = tau_list(it);
        act_bw_hz = 1 / (2*pi*tau_list(it));

        [best_Kp, best_Kd] = tune_pid_for_case(cfg, seeds_tune, theta0_list);
        fprintf('p=%g, tau=%.3f -> tuned Kp=%.1f Kd=%.1f\n', p_list(ip), tau_list(it), best_Kp, best_Kd);

        worst_rate = inf;
        worst_freq = NaN;
        best_rate = -inf;

        for ifr = 1:numel(freq_list_hz)
            sc = rocket_scenario('nominal', cfg);
            sc.t_end = cfg.t_end_demo;
            sc.disturbance_amp = 0.30;
            sc.disturbance_freq_hz = freq_list_hz(ifr);

            cfg.controllers.PID.Kp = best_Kp;
            cfg.controllers.PID.Kd = best_Kd;
            [rate, mean_theta_rms] = evaluate_pid_case(cfg, sc, realism, seeds_eval, theta0_list);

            rows(end+1,:) = [ ...
                p_list(ip), tau_list(it), nat_freq_hz, act_bw_hz, freq_list_hz(ifr), ...
                freq_list_hz(ifr) / nat_freq_hz, freq_list_hz(ifr) / act_bw_hz, rate, mean_theta_rms, ...
                best_Kp, best_Kd]; %#ok<SAGROW>

            worst_rate = min(worst_rate, rate);
            best_rate = max(best_rate, rate);
            if abs(rate - worst_rate) < 1e-12
                worst_freq = freq_list_hz(ifr);
            end
        end

        summary_rows(end+1,:) = [ ...
            p_list(ip), tau_list(it), nat_freq_hz, act_bw_hz, worst_freq, ...
            worst_freq / nat_freq_hz, worst_freq / act_bw_hz, best_rate, worst_rate, best_rate - worst_rate, ...
            best_Kp, best_Kd]; %#ok<SAGROW>
    end
end

rawT = array2table(rows, 'VariableNames', ...
    {'p_unstable','tau_act_s','nat_freq_hz','act_bw_hz','disturb_freq_hz', ...
     'freq_over_nat','freq_over_act_bw','success_rate','mean_theta_rms_deg','best_Kp','best_Kd'});
writetable(rawT, fullfile(here, 'results', 'resonance_sweep_raw.csv'));

summaryT = array2table(summary_rows, 'VariableNames', ...
    {'p_unstable','tau_act_s','nat_freq_hz','act_bw_hz','worst_freq_hz', ...
     'worst_freq_over_nat','worst_freq_over_act_bw','best_rate','worst_rate','drop_across_freq','best_Kp','best_Kd'});
writetable(summaryT, fullfile(here, 'results', 'resonance_sweep_summary.csv'));

fprintf('Median worst_freq / nat_freq = %.3f\n', median(summaryT.worst_freq_over_nat));
fprintf('Median worst_freq / act_bw   = %.3f\n', median(summaryT.worst_freq_over_act_bw));
fprintf('Largest success drop across frequency = %.3f\n', max(summaryT.drop_across_freq));
fprintf('Saved: experiments/results/resonance_sweep_raw.csv\n');
fprintf('Saved: experiments/results/resonance_sweep_summary.csv\n');
fprintf('DONE.\n');


function [best_Kp, best_Kd] = tune_pid_for_case(cfg, seeds, theta0_list)
Kp_grid = [1 3 10 30];
Kd_grid = [0.3 1 3 10];

sc = rocket_scenario('nominal', cfg);
sc.t_end = cfg.t_end_demo;
sc.disturbance_amp = 0.30;
sc.disturbance_freq_hz = 0.2;

best_rate = -inf;
best_theta_rms = inf;
best_Kp = Kp_grid(1);
best_Kd = Kd_grid(1);

for Kp = Kp_grid
    for Kd = Kd_grid
        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Kd = Kd;
        [rate, mean_theta_rms] = evaluate_pid_case(cfg, sc, struct(), seeds, theta0_list);
        if rate > best_rate || (abs(rate - best_rate) < 1e-9 && mean_theta_rms < best_theta_rms)
            best_rate = rate;
            best_theta_rms = mean_theta_rms;
            best_Kp = Kp;
            best_Kd = Kd;
        end
    end
end
end


function [rate, mean_theta_rms_deg] = evaluate_pid_case(cfg, sc, realism, seeds, theta0_list)
wins = 0;
n_trials = 0;
theta_rms_deg_vals = zeros(numel(seeds) * numel(theta0_list), 1);
idx = 0;

for it = 1:numel(theta0_list)
    cfg.plant.theta0 = theta0_list(it);
    for iseed = 1:numel(seeds)
        idx = idx + 1;
        n_trials = n_trials + 1;
        try
            out = simulate_case_realistic('PID', sc, cfg, seeds(iseed) + 100 * it, realism);
            theta_rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
            theta_rms_deg_vals(idx) = theta_rms_deg;
            stable = all(isfinite(out.theta)) && ...
                     max(abs(out.theta)) < deg2rad(40) && ...
                     theta_rms_deg <= 10 && ...
                     abs(out.theta(end)) < deg2rad(8);
            wins = wins + double(stable);
        catch
            theta_rms_deg_vals(idx) = 90;
        end
    end
end

rate = wins / n_trials;
mean_theta_rms_deg = mean(theta_rms_deg_vals);
end