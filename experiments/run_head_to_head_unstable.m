% run_head_to_head_unstable.m
% Apples-to-apples comparison at u_max ∈ {10, 27, 30} deg (matching the three
% audited firmwares) across p_unstable ∈ {4, 6, 8, 10}.
%
% Controllers compared (each given their best chance):
%   1. STEEL_PD       — best per-cell PD retune from firmware_audit_steelman
%   2. FIXED_LQR_Rstar — LQR with K computed from R*(u_max) lookup at each (p, u_max)
%   3. JOINT_ADAPTIVE — user's SisyphusCode controller, as-shipped
%   4. SIGMA_MRAC     — bundled gains
%   5. PCH_LQR        — bundled gains
%
% This is the publication-grade comparison.

clear; clc;
addpath(genpath('..\ModelRocket_Adaptive_TVC\src'));
here = fileparts(mfilename('fullpath'));

cfg = rocket_defaults();

% Load steel-man PD optimum
steel = readtable(fullfile(here,'results','firmware_audit_steelman.csv'));

umax_list = [10 27 30];
p_list    = [4 6 8 10];
n_seeds   = 4;
succ_thr  = 10;

realism = struct( ...
    'noise_th_std_deg', 0.20, 'noise_q_std_deg_s', 2.0, 'bias_rw_std', 0.005, ...
    'quant_th_deg', 0.10, 'quant_q_deg_s', 0.50, 'sensor_latency_steps', 2, ...
    'gust_tau', 0.4, 'gust_std', 1.0, 'keff_drift_rate', 0.0, ...
    'backlash_deg', 0.20, 'deadband_deg', 0.05, 'cal_scale', 1.0, 'cal_bias_deg', 0.0);

% R*(u_max) lookup (from h1_bestk + realistic_h1_sweep)
function R = Rstar(umax_deg)
    if     umax_deg <= 2.5,  R = 0.25;
    elseif umax_deg <= 4.5,  R = 0.50;
    elseif umax_deg <= 8.5,  R = 4.00;
    else,                    R = 8.00;
    end
end

% LQR designer
function K = lqr_design(p_unstable, keff, damp, R)
    A = [0 1; p_unstable^2 -damp];
    B = [0; keff];
    Q = diag([1, 0.1]);
    K = lqr(A, B, Q, R);
end

function rate = run_case(ctrl_name, cfg, p, n_seeds, succ_thr, realism)
    cfg.plant.p_unstable = p;
    succ = 0;
    for s = 1:n_seeds
        sc = rocket_scenario('nominal');
        try
            out = simulate_case_realistic(ctrl_name, sc, cfg, s, realism);
            if all(isfinite(out.theta)) && sqrt(mean(out.theta.^2))*180/pi <= succ_thr
                succ = succ + 1;
            end
        catch
        end
    end
    rate = succ / n_seeds;
end

results = [];
for umax = umax_list
    fprintf('\n========= u_max = %d deg =========\n', umax);

    % Configure all controllers with same u_max
    cfg.controllers.FIXED_LQR.u_max     = umax * pi/180;
    cfg.controllers.SIGMA_MRAC.u_max    = umax * pi/180;
    cfg.controllers.PCH_LQR.u_max       = umax * pi/180;
    cfg.controllers.JOINT_ADAPTIVE.u_max= umax * pi/180;
    cfg.controllers.PID.Ki = 0; cfg.controllers.PID.u_max = umax*pi/180; cfg.controllers.PID.i_lim = umax*pi/180;

    for p = p_list
        % Steel PD optimum (look up best Kp, Kd from steel CSV)
        % Approximate by matching closest u_max
        mask = (abs(steel.p_unstable - p) < 0.1);
        % We saved per-firmware; pick the row whose u_max matches
        % (firmware index 1=u_max10, 2=u_max27, 3=u_max30)
        if umax == 10, fidx = 1; elseif umax == 27, fidx = 2; else, fidx = 3; end
        fw_names = {'tomkuttler/TVC-Flight','AdamMarciniak/FCV1','PrajNasa/Dhumaketu'};
        steel_rows = find(strcmp(steel.firmware, fw_names{fidx}));
        % match p
        steel_row_idx = find(steel.p_unstable(steel_rows) == p, 1);
        if ~isempty(steel_row_idx)
            r = steel_rows(steel_row_idx);
            best_Kp = steel.best_Kp(r); best_Kd = steel.best_Kd(r);
            best_steel_rate = steel.best_tuned_success(r);
        else
            best_Kp = NaN; best_Kd = NaN; best_steel_rate = NaN;
        end

        % FIXED_LQR with R*(u_max)
        R = Rstar(umax);
        K_star = lqr_design(p, cfg.plant.control_eff, cfg.plant.aero_damp, R);
        cfg.controllers.FIXED_LQR.K = K_star;
        lqr_rate = run_case('FIXED_LQR', cfg, p, n_seeds, succ_thr, realism);

        % JOINT_ADAPTIVE (as-shipped, but with this u_max)
        ja_rate = run_case('JOINT_ADAPTIVE', cfg, p, n_seeds, succ_thr, realism);

        % SIGMA_MRAC and PCH_LQR
        mrac_rate = run_case('SIGMA_MRAC', cfg, p, n_seeds, succ_thr, realism);
        pch_rate  = run_case('PCH_LQR', cfg, p, n_seeds, succ_thr, realism);

        fprintf('  p=%2d:  STEEL_PD=%.0f%%  FIXED_LQR*=%.0f%%  JOINT_ADPT=%.0f%%  SIGMA_MRAC=%.0f%%  PCH=%.0f%%\n', ...
            p, 100*best_steel_rate, 100*lqr_rate, 100*ja_rate, 100*mrac_rate, 100*pch_rate);

        results(end+1,:) = [umax, p, best_steel_rate, lqr_rate, ja_rate, mrac_rate, pch_rate]; %#ok<SAGROW>
    end
end

T = array2table(results, 'VariableNames', ...
    {'u_max_deg','p_unstable','steel_PD','FIXED_LQR_Rstar','JOINT_ADAPTIVE','SIGMA_MRAC','PCH_LQR'});
writetable(T, fullfile(here, 'results', 'head_to_head_unstable.csv'));
fprintf('\nSaved: experiments/results/head_to_head_unstable.csv\n');
