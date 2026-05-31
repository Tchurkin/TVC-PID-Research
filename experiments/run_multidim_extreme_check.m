% run_multidim_extreme_check.m
% Narrow disambiguation sweep for the joint-law result.
% The first multidimensional scan found essentially zero latency/noise effect.
% This script stress-tests that conclusion at p=10 with much harsher sensor settings.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));

cfg = rocket_defaults();
cfg.t_end_demo = 3.0;
cfg.plant.aero_damp = 0.5;
cfg.plant.control_eff = 8.0;
cfg.plant.u_max = 12.0;
cfg.plant.p_unstable = 10.0;

latency_steps_list = [1 4 8 16];
gyro_noise_list = [0.005 0.040 0.080 0.120];
slew_code_list = [6 9 12 18];
Kp_grid = [1 3 10 30];
Kd_grid = [0.3 1 3 10];
seeds = 1:2;
theta0_list = deg2rad([3 6]);

cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;
cfg.controllers.PID.Ki = 0.0;

rows = [];

for il = 1:numel(latency_steps_list)
    for in = 1:numel(gyro_noise_list)
        for is = 1:numel(slew_code_list)
            cfg.plant.slew_max = slew_code_list(is);
            realism = struct( ...
                'gyro_noise_std', gyro_noise_list(in), ...
                'sensor_latency_steps', latency_steps_list(il), ...
                'gust_std', 0.45, ...
                'gust_tau', 0.40, ...
                'servo_deadband', 0.05, ...
                'servo_backlash', 0.10, ...
                'gyro_bias_rw', 0.005, ...
                'gyro_bias_init', 0.010, ...
                'gyro_quant_lsb', 2*pi/4000, ...
                'servo_pos_noise', 0.01, ...
                'theta_init_bias', deg2rad(0.5));

            best_rate = -inf;
            best_Kp = NaN;
            best_Kd = NaN;

            for Kp = Kp_grid
                for Kd = Kd_grid
                    cfg.controllers.PID.Kp = Kp;
                    cfg.controllers.PID.Kd = Kd;
                    rate = evaluate_case(cfg, realism, seeds, theta0_list);
                    if rate > best_rate
                        best_rate = rate;
                        best_Kp = Kp;
                        best_Kd = Kd;
                    end
                end
            end

            rows(end+1,:) = [latency_steps_list(il), gyro_noise_list(in), slew_code_list(is), best_rate, best_Kp, best_Kd]; %#ok<SAGROW>
        end
    end
end

T = array2table(rows, 'VariableNames', ...
    {'latency_steps','gyro_noise_std','slew_code_units','best_success','best_Kp','best_Kd'});
writetable(T, fullfile(here, 'results', 'multidim_extreme_check.csv'));

fprintf('=== EXTREME SENSOR CHECK (p=10) ===\n');
disp(T);
fprintf('Saved: experiments/results/multidim_extreme_check.csv\n');
fprintf('DONE.\n');


function rate = evaluate_case(cfg, realism, seeds, theta0_list)
sc = rocket_scenario('nominal', cfg);
sc.t_end = cfg.t_end_demo;
sc.disturbance_amp = 0.15;
sc.disturbance_freq_hz = 0.8;

wins = 0;
n_trials = 0;
for it = 1:numel(theta0_list)
    cfg.plant.theta0 = theta0_list(it);
    for iseed = 1:numel(seeds)
        n_trials = n_trials + 1;
        try
            out = simulate_case_realistic('PID', sc, cfg, seeds(iseed) + 100 * it, realism);
            theta_rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
            stable = all(isfinite(out.theta)) && ...
                     max(abs(out.theta)) < deg2rad(40) && ...
                     theta_rms_deg <= 10 && ...
                     abs(out.theta(end)) < deg2rad(8);
            wins = wins + double(stable);
        catch
        end
    end
end
rate = wins / n_trials;
end