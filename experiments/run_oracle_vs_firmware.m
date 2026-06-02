% run_oracle_vs_firmware.m
% Gating experiment for direction E2: "MPC offline, PID online" bridge.
%
% For each publicly-deployed amateur TVC firmware (defining its u_max)
% and each airframe instability p, grid-search PD gains to find the
% empirical optimum ("oracle PD"). Compare:
%   - BASELINE : firmware as shipped
%   - ORACLE   : best PD gains found by grid search
%
% Decision rule for committing to direction E2:
%   PASS    if >=3 cells exist where BASELINE_succ < 0.50 and ORACLE_succ >= 0.80
%   PARTIAL if >=1 such cell
%   FAIL    otherwise

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));
resdir = fullfile(here, 'results');
if ~exist(resdir, 'dir'); mkdir(resdir); end

cfg_base = rocket_defaults();

firmwares = {
%   name                         Kp     Ki     Kd     u_max_deg
    'tomkuttler/TVC-Flight',     0.60,  0.05,  0.125, 10
    'AdamMarciniak/FCV1',        1.00,  0.00,  2.00,  27
    'PrajNasa/Dhumaketu',        3.55,  0.005, 2.05,  30
};

% PD grid for oracle search
Kp_grid = [0.3 0.6 1.0 2.0 4.0 8.0 16.0];
Kd_grid = [0.1 0.5 1.0 2.0 5.0 10.0 20.0 40.0];

p_values    = [0 2 4 6 8 10];
n_seeds     = 4;
success_thr = 10;       % deg-RMS

realism = struct( ...
    'gyro_noise_std',     0.015, ...
    'gyro_bias_init',     0.010, ...
    'gyro_bias_rw',       0.005, ...
    'gyro_quant_lsb',     2*pi/4000, ...
    'bias_cal_s',         0.5, ...
    'bias_cal_residual_frac', 0.10, ...
    'sensor_latency_steps', 2, ...
    'servo_deadband',     0.05, ...
    'servo_backlash',     0.10, ...
    'servo_pos_noise',    0.01, ...
    'gust_std',           1.0, ...
    'gust_tau',           0.4, ...
    'keff_drift_rate',    0.0, ...
    'theta_init_bias',    deg2rad(0.5));

rows = [];

for i = 1:size(firmwares,1)
    fname    = firmwares{i,1};
    Kp_fw    = firmwares{i,2};
    Ki_fw    = firmwares{i,3};
    Kd_fw    = firmwares{i,4};
    umax_deg = firmwares{i,5};
    u_max_rad = umax_deg * pi/180;

    fprintf('\n=== %s (u_max=%g deg) ===\n', fname, umax_deg);
    fprintf('  %-3s | %-12s | %-25s | %s\n', 'p', 'BASELINE', 'ORACLE PD (Kp/Kd)', 'rescue?');
    fprintf('  ----+--------------+---------------------------+---------\n');

    for p = p_values
        cfg = cfg_base;
        cfg.plant.p_unstable = p;

        % --- BASELINE: firmware as shipped ---
        cfg.controllers.PID.Kp = Kp_fw;
        cfg.controllers.PID.Ki = Ki_fw;
        cfg.controllers.PID.Kd = Kd_fw;
        cfg.controllers.PID.u_max = u_max_rad;
        cfg.controllers.PID.i_lim = u_max_rad;

        succ_base = 0;
        for s = 1:n_seeds
            sc = rocket_scenario('nominal', cfg);
            try
                out = simulate_case_realistic('PID', sc, cfg, s, realism);
                rms_deg = sqrt(mean(out.theta.^2)) * 180/pi;
                if rms_deg <= success_thr; succ_base = succ_base + 1; end
            catch
                % crashed => not counted
            end
        end
        rate_base = succ_base / n_seeds;

        % --- ORACLE: grid-search PD over (Kp, Kd) ---
        best_rate = -1; best_Kp = NaN; best_Kd = NaN;
        for ikp = 1:numel(Kp_grid)
            for ikd = 1:numel(Kd_grid)
                cfg.controllers.PID.Kp = Kp_grid(ikp);
                cfg.controllers.PID.Ki = 0;
                cfg.controllers.PID.Kd = Kd_grid(ikd);
                cfg.controllers.PID.u_max = u_max_rad;
                cfg.controllers.PID.i_lim = u_max_rad;

                succ_oracle = 0;
                for s = 1:n_seeds
                    sc = rocket_scenario('nominal', cfg);
                    try
                        out = simulate_case_realistic('PID', sc, cfg, s, realism);
                        rms_deg = sqrt(mean(out.theta.^2)) * 180/pi;
                        if rms_deg <= success_thr; succ_oracle = succ_oracle + 1; end
                    catch
                    end
                end
                rate_oracle = succ_oracle / n_seeds;
                if rate_oracle > best_rate
                    best_rate = rate_oracle;
                    best_Kp = Kp_grid(ikp);
                    best_Kd = Kd_grid(ikd);
                end
            end
        end

        rescue = (rate_base < 0.50) && (best_rate >= 0.80);
        tag = ' ';
        if rescue, tag = 'RESCUE'; end
        if (rate_base < 0.50) && (best_rate >= 0.50) && ~rescue, tag = 'partial'; end

        fprintf('  %2d  | %3.0f%%        | %3.0f%% (Kp=%4.1f Kd=%4.1f)    | %s\n', ...
            p, rate_base*100, best_rate*100, best_Kp, best_Kd, tag);

        rows(end+1,:) = [i, p, rate_base, best_rate, best_Kp, best_Kd, ...
                         double(rescue)]; %#ok<SAGROW>
    end
end

% --- Summary --------------------------------------------------------------
n_rescue = sum(rows(:,7));
n_baseline_fail = sum(rows(:,3) < 0.50);
n_oracle_strong = sum(rows(:,4) >= 0.80);
gap_recovered = 0;
gap_total = 0;
for r = 1:size(rows,1)
    base = rows(r,3); orac = rows(r,4);
    gap_total = gap_total + max(0, 1.0 - base);
    gap_recovered = gap_recovered + max(0, orac - base);
end
gap_frac = gap_recovered / max(1e-9, gap_total);

fprintf('\n=== E2 GATING SUMMARY ===\n');
fprintf('  Cells where firmware fails (<50%%):       %d / %d\n', n_baseline_fail, size(rows,1));
fprintf('  Cells where oracle PD is strong (>=80%%): %d / %d\n', n_oracle_strong, size(rows,1));
fprintf('  Rescued cells (firm<50 & oracle>=80):    %d\n', n_rescue);
fprintf('  Gap-recovery fraction:                   %.1f%%  (oracle gain / max possible gain)\n', 100*gap_frac);

if n_rescue >= 3
    fprintf('  -> PASS. E2 empirical core supported. Worth proceeding to bench-distillation.\n');
elseif n_rescue >= 1
    fprintf('  -> PARTIAL. Some rescue possible but narrow.\n');
else
    fprintf('  -> FAIL. Even oracle PD cannot rescue firmware-fail cells. Failures are fundamental.\n');
end

% Save CSV
T = table( ...
    firmwares(rows(:,1),1), rows(:,2), rows(:,3), rows(:,4), rows(:,5), rows(:,6), ...
    logical(rows(:,7)), ...
    'VariableNames', {'firmware','p_unstable', ...
                      'baseline_succ','oracle_succ', ...
                      'best_Kp','best_Kd','rescue'});
writetable(T, fullfile(resdir, 'oracle_vs_firmware.csv'));
fprintf('\nSaved: experiments/results/oracle_vs_firmware.csv\n');
