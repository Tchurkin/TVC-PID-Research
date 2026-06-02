% run_rg_vs_pid.m
% A/B test: amateur-firmware PID gains vs the same gains augmented with a
% bench-calibrated Scalar Reference Governor (PID_RG).
%
% For each of three publicly-deployed open-source amateur TVC firmwares,
% sweep p_unstable and compare:
%   (A) PID    -- as shipped
%   (B) PID_RG -- same gains + RG layer parameterized from "bench" knowledge
%                 (here: the true plant params; in deployment: process_gimbal_bench_test)
%
% Decision rule for committing to direction D1 (RG-on-Arduino):
%   - RG rescues >= 3 cells across firmwares (success rate jumps from
%     <0.50 with PID to >=0.50 with PID_RG)
%   - AND RG produces 0 regressions where PID succeeds (>=0.80) and
%     PID_RG drops below 0.80.
% If both pass, D1's empirical core is supported -> commit to flight campaign.

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

p_values    = [0 2 4 6 8 10];
n_seeds     = 6;
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
    Kp       = firmwares{i,2};
    Ki       = firmwares{i,3};
    Kd       = firmwares{i,4};
    umax_deg = firmwares{i,5};

    fprintf('\n=== %s ===\n', fname);
    fprintf('  PID: Kp=%.3f Ki=%.4f Kd=%.3f  u_max=%g deg\n', Kp, Ki, Kd, umax_deg);
    fprintf('  %-3s | %-15s | %-15s | %s\n', 'p', 'PID succ/rms', 'PID_RG succ/rms', 'rescue?');
    fprintf('  ----+-----------------+-----------------+--------\n');

    for p = p_values
        cfg = cfg_base;
        cfg.plant.p_unstable = p;

        % Configure both controllers with the same firmware gains
        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Ki = Ki;
        cfg.controllers.PID.Kd = Kd;
        cfg.controllers.PID.u_max = umax_deg * pi/180;
        cfg.controllers.PID.i_lim = umax_deg * pi/180;

        cfg.controllers.PID_RG.Kp = Kp;
        cfg.controllers.PID_RG.Ki = Ki;
        cfg.controllers.PID_RG.Kd = Kd;
        cfg.controllers.PID_RG.u_max = umax_deg * pi/180;
        cfg.controllers.PID_RG.i_lim = umax_deg * pi/180;
        % RG predictor uses the current p (in deployment, this is the
        % preflight estimate from the validator)
        cfg.controllers.PID_RG.rg.p_assumed = p;
        cfg.controllers.PID_RG.rg.keff_assumed = cfg.plant.control_eff;
        cfg.controllers.PID_RG.rg.aero_damp_assumed = cfg.plant.aero_damp;
        cfg.controllers.PID_RG.rg.tau_act_assumed = cfg.plant.tau_act;

        succ_pid = 0; succ_rg = 0;
        rms_pid_sum = 0; rms_rg_sum = 0;

        for s = 1:n_seeds
            sc = rocket_scenario('nominal', cfg);

            % Run PID (baseline)
            try
                out_pid = simulate_case_realistic('PID', sc, cfg, s, realism);
                rms_pid = sqrt(mean(out_pid.theta.^2)) * 180/pi;
            catch
                rms_pid = 180;
            end
            rms_pid_sum = rms_pid_sum + min(rms_pid, 180);
            if rms_pid <= success_thr; succ_pid = succ_pid + 1; end

            % Run PID_RG (same seed for paired comparison)
            try
                out_rg = simulate_case_realistic('PID_RG', sc, cfg, s, realism);
                rms_rg = sqrt(mean(out_rg.theta.^2)) * 180/pi;
            catch
                rms_rg = 180;
            end
            rms_rg_sum = rms_rg_sum + min(rms_rg, 180);
            if rms_rg <= success_thr; succ_rg = succ_rg + 1; end
        end

        rate_pid = succ_pid / n_seeds;
        rate_rg  = succ_rg  / n_seeds;
        rms_pid_mean = rms_pid_sum / n_seeds;
        rms_rg_mean  = rms_rg_sum  / n_seeds;

        rescue = (rate_pid < 0.50) && (rate_rg >= 0.50);
        regress = (rate_pid >= 0.80) && (rate_rg < 0.80);
        tag = ' ';
        if rescue,  tag = 'RESCUE';   end
        if regress, tag = 'REGRESS!'; end

        fprintf('  %2d  | %3.0f%% / %5.1f deg | %3.0f%% / %5.1f deg | %s\n', ...
            p, rate_pid*100, rms_pid_mean, rate_rg*100, rms_rg_mean, tag);

        rows(end+1,:) = [i, p, rate_pid, rms_pid_mean, rate_rg, rms_rg_mean, ...
                         double(rescue), double(regress)]; %#ok<SAGROW>
    end
end

% --- Summary --------------------------------------------------------------
n_rescue  = sum(rows(:,7));
n_regress = sum(rows(:,8));

fprintf('\n=== D1 GATING SUMMARY ===\n');
fprintf('  rescues (PID fails, PID_RG succeeds):     %d\n', n_rescue);
fprintf('  regressions (PID succeeds, PID_RG fails): %d\n', n_regress);

if n_rescue >= 3 && n_regress == 0
    fprintf('  -> PASS. D1 empirical core supported. Commit to flight campaign.\n');
elseif n_rescue >= 1 && n_regress == 0
    fprintf('  -> PARTIAL. RG helps in some cells, no regressions. Worth more sim tuning.\n');
elseif n_regress > 0
    fprintf('  -> FAIL (regressions present). RG is hurting nominal cases. Revisit design.\n');
else
    fprintf('  -> FAIL (no rescues). RG provides no measurable benefit on these firmwares.\n');
end

% Save CSV
T = table( ...
    firmwares(rows(:,1),1), rows(:,2), rows(:,3), rows(:,4), rows(:,5), rows(:,6), ...
    logical(rows(:,7)), logical(rows(:,8)), ...
    'VariableNames', {'firmware','p_unstable', ...
                      'pid_success','pid_rms_deg', ...
                      'rg_success','rg_rms_deg','rescue','regression'});
writetable(T, fullfile(resdir, 'rg_vs_pid.csv'));
fprintf('\nSaved: experiments/results/rg_vs_pid.csv\n');
