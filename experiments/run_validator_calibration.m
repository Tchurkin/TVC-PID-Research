function summary = run_validator_calibration()
%RUN_VALIDATOR_CALIBRATION  Validate the validator.
% Sample random (slew, u_max, p_unstable) configurations on the realistic
% plant. For each, ask recommend_envelope what it predicts, then run the
% sim ground truth. Score the validator's calibration:
%   - Did GO predictions actually succeed?
%   - Did NOGO predictions actually fail?
%   - Confusion matrix region vs ground-truth success rate.
% Also produces a p-sweep slice of the phase diagram.

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, 'validator'));
proj = fileparts(here);
addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src', 'utils'));

% ----- Build a randomized test set --------------------------------------
rng(42);
N_configs = 60;
p_grid    = [4 6 8 10 12];
slew_grid_deg = [10 15 25 40 60];   % gimbal deg/s
umax_grid_deg = [2 4 6 8 12];       % gimbal deg
servo_max_deg = 10;

configs = struct('p',{},'slew',{},'umax',{});
for k = 1:N_configs
    configs(k).p    = p_grid(randi(numel(p_grid)));
    configs(k).slew = slew_grid_deg(randi(numel(slew_grid_deg)));
    configs(k).umax = umax_grid_deg(randi(numel(umax_grid_deg)));
end

% ----- Common sim config ------------------------------------------------
cfg_base = rocket_defaults();
cfg_base.plant.aero_damp = 0.5;
cfg_base.plant.control_eff = 8.0;
cfg_base.plant.tau_act = 0.05;
cfg_base.t_end_demo = 3.0;

sc.t_end = 2.5;
sc.fault_time = 999;
sc.kind = 'validator_cal';
sc.control_eff_scale_post = 1; sc.aero_damp_scale_post = 1;
sc.tau_scale_post = 1; sc.slew_scale_post = 1;
sc.disturbance_amp = 0; sc.disturbance_freq_hz = 1;
sc.disturb_scale_post = 1; sc.disturb_bias_post = 0;
realism = struct('gust_std', 1.5, 'gust_tau', 0.30);

seeds = 1:6;
theta0_set = deg2rad([2 5 8 12]);
theta_fail = deg2rad(40);
trials_per_config = numel(seeds) * numel(theta0_set);

% ----- Code-unit conversion (mirror validator's normalization) ----------
gimbal_max_rad = deg2rad(servo_max_deg);
code_scale = 12.0 / gimbal_max_rad;
deg2code_slew = @(s_deg) deg2rad(s_deg) * code_scale;
deg2code_umax = @(u_deg) deg2rad(u_deg) * code_scale;

predicted = strings(N_configs, 1);
verdict   = strings(N_configs, 1);
R_used    = zeros(N_configs, 1);
succ_actual = zeros(N_configs, 1);
slew_code_log = zeros(N_configs, 1);
umax_code_log = zeros(N_configs, 1);

fprintf('=== VALIDATOR CALIBRATION (%d random configs) ===\n', N_configs);

% Suppress validator stdout via diary trick
log_path = fullfile(tempdir, 'validator_silence.txt');

for k = 1:N_configs
    c = configs(k);
    meas = struct('slew_deg_per_s', c.slew, 'servo_max_deg', servo_max_deg, ...
                  'p_est', c.p, 'keff_est', 8.0, 'damp_est', 0.2);
    diary(log_path); diary on;
    v = recommend_envelope(meas);
    diary off;
    predicted(k) = v.region;
    verdict(k)   = v.go_nogo;
    R_used(k)    = v.R_recommended;

    % Use the validator's RECOMMENDED u_max if it's <= bench u_max, else
    % cap at bench. This mirrors what an amateur would actually deploy.
    u_max_use_deg = min(v.u_max_recommended_deg, c.umax);
    if u_max_use_deg <= 0.05
        succ_actual(k) = 0;
        slew_code_log(k) = deg2code_slew(c.slew);
        umax_code_log(k) = 0;
        continue;
    end
    K = v.K;

    cfg = cfg_base;
    cfg.plant.p_unstable = c.p;
    cfg.plant.slew_max = deg2code_slew(c.slew);
    cfg.plant.u_max    = deg2code_umax(u_max_use_deg);
    cfg.controllers.FIXED_LQR.K = K;
    cfg.controllers.FIXED_LQR.K_nominal = K;
    cfg.controllers.FIXED_LQR.u_max = cfg.plant.u_max;
    slew_code_log(k) = cfg.plant.slew_max;
    umax_code_log(k) = cfg.plant.u_max;

    wins = 0;
    for ith = 1:numel(theta0_set)
        cfg.plant.theta0 = theta0_set(ith);
        for s = seeds
            seed = s + 100*ith + 7*k;
            out = simulate_case_realistic('FIXED_LQR', sc, cfg, seed, realism);
            if all(abs(out.theta) < theta_fail)
                wins = wins + 1;
            end
        end
    end
    succ_actual(k) = wins / trials_per_config;
end
if exist(log_path, 'file'), delete(log_path); end

% ----- Confusion / calibration tables -----------------------------------
fprintf('\n  Per-region success summary:\n');
regions = ["RESCUE", "FUNDAMENTAL", "INFEASIBLE"];
for r = regions
    sel = predicted == r;
    n = sum(sel);
    if n == 0
        fprintf('    %-12s : (no configs sampled)\n', r);
    else
        fprintf('    %-12s : n=%2d  mean_actual_success=%.3f  std=%.3f\n', ...
            r, n, mean(succ_actual(sel)), std(succ_actual(sel)));
    end
end

fprintf('\n  Per-verdict success summary:\n');
for v_name = ["GO", "MARGINAL", "NOGO"]
    sel = verdict == v_name;
    n = sum(sel);
    if n == 0
        fprintf('    %-9s : (none)\n', v_name);
    else
        fprintf('    %-9s : n=%2d  mean_actual_success=%.3f  >=0.80 rate=%.2f  <=0.50 rate=%.2f\n', ...
            v_name, n, mean(succ_actual(sel)), ...
            mean(succ_actual(sel) >= 0.80), mean(succ_actual(sel) <= 0.50));
    end
end

% Validator calibration scoring:
%   * GO should predict >=0.80 success (true positive)
%   * NOGO should predict <=0.50 success (true negative)
go_sel   = verdict == "GO";
nogo_sel = verdict == "NOGO";
n_go = sum(go_sel); n_nogo = sum(nogo_sel);

tp = sum(go_sel   & succ_actual >= 0.80);  fp = sum(go_sel   & succ_actual < 0.80);
tn = sum(nogo_sel & succ_actual <= 0.50);  fn = sum(nogo_sel & succ_actual > 0.50);

fprintf('\n  === CALIBRATION SCORE ===\n');
if n_go > 0
    fprintf('    GO precision (predict GO and actually >=0.80): %.2f  (%d/%d)\n', tp/n_go, tp, n_go);
end
if n_nogo > 0
    fprintf('    NOGO precision (predict NOGO and actually <=0.50): %.2f  (%d/%d)\n', tn/n_nogo, tn, n_nogo);
end

% ----- Per-p phase diagram slice ---------------------------------------
fprintf('\n  === PHASE-DIAGRAM PER-p ACTUAL SUCCESS (averaged across cells) ===\n');
for p_v = p_grid
    sel = arrayfun(@(c) c.p == p_v, configs);
    if any(sel)
        fprintf('    p=%2d : n=%2d  mean=%.3f  best=%.3f  worst=%.3f\n', ...
            p_v, sum(sel), mean(succ_actual(sel)), max(succ_actual(sel)), min(succ_actual(sel)));
    end
end

% ----- CSV --------------------------------------------------------------
fid = fopen(fullfile(here, 'results', 'validator_calibration.csv'), 'w');
fprintf(fid, 'p_unstable,slew_deg_per_s,umax_deg,predicted_region,verdict,R_used,actual_success\n');
for k = 1:N_configs
    fprintf(fid, '%d,%.2f,%.2f,%s,%s,%.4f,%.4f\n', ...
        configs(k).p, configs(k).slew, configs(k).umax, ...
        predicted(k), verdict(k), R_used(k), succ_actual(k));
end
fclose(fid);
fprintf('Wrote results/validator_calibration.csv\n');

summary.configs = configs;
summary.predicted = predicted;
summary.verdict = verdict;
summary.succ_actual = succ_actual;
end
