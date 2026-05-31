% run_firmware_audit_steelman.m
% Steel-man version: for each open-source firmware's CONTROLLER STRUCTURE
% (PD/PID + u_max), grid-search optimal gains per p_unstable and report what
% the best-tuned version of THAT architecture can achieve.
%
% This addresses the "you indicted them on the wrong tunings" objection.
% The honest claim that survives the steel-man is: even with optimal
% per-p PID retuning of their architecture, they cap out below p where
% LQR with R*(u_max) succeeds.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));

cfg = rocket_defaults();

% Firmware controller structures (PD; Ki=0 in steel-man because integrator
% wind-up is its own failure mode; we want the optimal stabilization-only fit)
firmwares = {
%   name                         u_max_deg  shipped_Kp shipped_Kd
    'tomkuttler/TVC-Flight',     10,        0.60,      0.125
    'AdamMarciniak/FCV1',        27,        1.00,      2.00
    'PrajNasa/Dhumaketu',        30,        3.55,      2.05
};

p_values    = [4 6 8 10];           % the discriminating range
n_seeds     = 3;
success_thr = 10;                    % deg-RMS

% Gain grids (logarithmic spacing) — kept small for runtime
Kp_grid = [1 3 10 30 100];
Kd_grid = [0.2 1 3 10 30];

realism = struct( ...
    'noise_th_std_deg',   0.20, ...
    'noise_q_std_deg_s',  2.0, ...
    'bias_rw_std',        0.005, ...
    'quant_th_deg',       0.10, ...
    'quant_q_deg_s',      0.50, ...
    'sensor_latency_steps', 2, ...
    'gust_tau',           0.4, ...
    'gust_std',           1.0, ...
    'keff_drift_rate',    0.0, ...
    'backlash_deg',       0.20, ...
    'deadband_deg',       0.05, ...
    'cal_scale',          1.0, ...
    'cal_bias_deg',       0.0);

results = [];
for i = 1:size(firmwares,1)
    name     = firmwares{i,1};
    umax_deg = firmwares{i,2};
    Kp_ship  = firmwares{i,3};
    Kd_ship  = firmwares{i,4};
    fprintf('\n=== %s  (u_max=%g deg, shipped Kp=%.2f Kd=%.2f) ===\n', name, umax_deg, Kp_ship, Kd_ship);

    cfg.controllers.PID.Ki    = 0.0;
    cfg.controllers.PID.u_max = umax_deg * pi/180;
    cfg.controllers.PID.i_lim = umax_deg * pi/180;

    for p = p_values
        cfg.plant.p_unstable = p;

        best_rate = 0; best_Kp = NaN; best_Kd = NaN;
        % grid search
        for Kp = Kp_grid
            for Kd = Kd_grid
                cfg.controllers.PID.Kp = Kp;
                cfg.controllers.PID.Kd = Kd;
                succ = 0;
                for s = 1:n_seeds
                    sc = rocket_scenario('nominal');
                    try
                        out = simulate_case_realistic('PID', sc, cfg, s, realism);
                        if all(isfinite(out.theta)) && sqrt(mean(out.theta.^2))*180/pi <= success_thr
                            succ = succ + 1;
                        end
                    catch
                        % crash counts as fail
                    end
                end
                rate = succ / n_seeds;
                if rate > best_rate
                    best_rate = rate; best_Kp = Kp; best_Kd = Kd;
                end
            end
        end

        % Also run shipped gains for comparison
        cfg.controllers.PID.Kp = Kp_ship;
        cfg.controllers.PID.Kd = Kd_ship;
        succ_ship = 0;
        for s = 1:n_seeds
            sc = rocket_scenario('nominal');
            try
                out = simulate_case_realistic('PID', sc, cfg, s, realism);
                if all(isfinite(out.theta)) && sqrt(mean(out.theta.^2))*180/pi <= success_thr
                    succ_ship = succ_ship + 1;
                end
            catch
            end
        end
        ship_rate = succ_ship / n_seeds;

        fprintf('  p=%2d : SHIPPED gains=%3.0f%%  |  BEST-TUNED gains=%3.0f%% @ Kp=%5.1f Kd=%5.1f\n', ...
            p, ship_rate*100, best_rate*100, best_Kp, best_Kd);

        results(end+1,:) = [i, p, ship_rate, best_rate, best_Kp, best_Kd]; %#ok<SAGROW>
    end
end

% Save
T = table(firmwares(results(:,1),1), results(:,2), results(:,3), results(:,4), ...
          results(:,5), results(:,6), ...
    'VariableNames', {'firmware','p_unstable','shipped_success','best_tuned_success','best_Kp','best_Kd'});
writetable(T, fullfile(here, 'results', 'firmware_audit_steelman.csv'));
fprintf('\nSaved: experiments/results/firmware_audit_steelman.csv\n');
fprintf('DONE.\n');
