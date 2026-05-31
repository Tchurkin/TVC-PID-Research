% run_firmware_audit.m
% Take PID gains and saturation limits from REAL, publicly-deployed open-source
% amateur TVC firmwares on GitHub and run each one through the realistic plant
% across a sweep of p_unstable. Report at what instability each crashes.
%
% This is a community-practice audit: does what amateurs are actually flying
% line up with our validator's predictions?
%
% Sources (all MIT or unlicensed-public):
%   1. tomkuttler/TVC-Rocket-Flight-Code  (62 stars, flown 2021-2024)
%      Rocket.h:  PID(0.6, 0.05, 0.125, 10)   ; gimbal lim 10 deg
%   2. AdamMarciniak/FlightComputerV1     (12 stars)
%      main.ino:  KP=1, KI=0, KD=2          ; gimbal lim ~27 deg
%   3. PrajNasa/Thrust-Vector-Control-of-Model-Rockets / "Dhumaketu" (3 stars)
%      Dhumaketu.ino: Kp=3.55, Ki=0.005, Kd=2.05 ; gimbal lim 30 deg
%
% Gains are angle-domain controllers, output in deg of motor mount.
% Internally our plant expects radians for theta error and outputs rad for u;
% we convert the firmware gains to rad-domain so the controller sees the same
% units the plant uses. PID with input in rad and output in rad:
%   Kp_rad_to_rad = Kp_deg_to_deg              (dimensionless, rad/rad scales identically)
%   Ki_rad_to_rad = Ki_deg_to_deg
%   Kd_rad_to_rad = Kd_deg_to_deg
% (Both error and output are degrees, so the ratio is preserved when converted to rad.)

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));

cfg = rocket_defaults();

firmwares = {
%   name                         Kp     Ki     Kd     u_max_deg  stars
    'tomkuttler/TVC-Flight',     0.60,  0.05,  0.125, 10,        62
    'AdamMarciniak/FCV1',        1.00,  0.00,  2.00,  27,        12
    'PrajNasa/Dhumaketu',        3.55,  0.005, 2.05,  30,         3
};

p_values    = [0 2 4 6 8 10 12];     % p_unstable to sweep
n_seeds     = 6;                      % per cell
success_thr = 10;                     % deg-RMS threshold for "succeeded"

results = [];
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

for i = 1:size(firmwares,1)
    fname = firmwares{i,1};
    Kp = firmwares{i,2}; Ki = firmwares{i,3}; Kd = firmwares{i,4};
    umax_deg = firmwares{i,5}; stars = firmwares{i,6};

    fprintf('\n=== %s (%d stars) ===\n', fname, stars);
    fprintf('  PID: Kp=%.3f Ki=%.4f Kd=%.3f  u_max=%g deg\n', Kp, Ki, Kd, umax_deg);

    cfg.controllers.PID.Kp    = Kp;
    cfg.controllers.PID.Ki    = Ki;
    cfg.controllers.PID.Kd    = Kd;
    cfg.controllers.PID.u_max = umax_deg * pi/180;   % rad
    cfg.controllers.PID.i_lim = umax_deg * pi/180;   % rad

    for p = p_values
        cfg.plant.p_unstable = p;
        succ = 0; rms_sum = 0;
        for s = 1:n_seeds
            sc = rocket_scenario('nominal');
            try
                out = simulate_case_realistic('PID', sc, cfg, s, realism);
                th_rms = sqrt(mean(out.theta.^2)) * 180/pi;
                rms_sum = rms_sum + th_rms;
                if th_rms <= success_thr; succ = succ + 1; end
            catch ME
                rms_sum = rms_sum + 90; % counted as crash
            end
        end
        rate = succ / n_seeds;
        mean_rms = rms_sum / n_seeds;
        results(end+1,:) = [i, p, rate, mean_rms]; %#ok<SAGROW>
        fprintf('  p=%2d : success=%3.0f%%  mean_rms=%5.1f deg\n', p, rate*100, mean_rms);
    end
end

% Save
T = table(firmwares(results(:,1),1), results(:,2), results(:,3), results(:,4), ...
    'VariableNames', {'firmware','p_unstable','success_rate','mean_rms_deg'});
writetable(T, fullfile(here, 'results', 'firmware_audit.csv'));
fprintf('\nSaved: experiments/results/firmware_audit.csv\n');

% Validator predictions for each firmware (using SG90-class slew = 90 deg/s gimbal)
fprintf('\n=== VALIDATOR PREDICTIONS (assuming SG90-class servo, gimbal slew=90 deg/s) ===\n');
addpath(fullfile(here, 'validator'));
for i = 1:size(firmwares,1)
    umax_deg = firmwares{i,5};
    for p = [4 8 12]
        try
            m = struct('slew_deg_per_s', 90, 'servo_max_deg', umax_deg, ...
                       'p_est', p, 'keff_est', cfg.plant.control_eff, ...
                       'damp_est', cfg.plant.aero_damp);
            rec = recommend_envelope(m);
            fprintf('  %-28s @ p=%2d : %s / %s\n', firmwares{i,1}, p, rec.region, rec.go_nogo);
        catch ME
            fprintf('  %-28s @ p=%2d : ERR %s\n', firmwares{i,1}, p, ME.message);
        end
    end
end

fprintf('\nDONE.\n');
