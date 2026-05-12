%% tune_params_B_industry.m
% Robust grid search for Direction B industry benchmark parameters.
% Optimizes ACA-ADRC significance over standard ADRC across 8/10/12/15 m/s.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
out_dir = fullfile(dir_root, 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
addpath(this_dir);

winds = [8 10 12 15];
SEED = 3030;
dt = 0.010;
t_end = 70.0;
N = round(t_end/dt);

l1.L1_period = 20.0;
l1.u_max = 4.0;

omega_c_grid = [0.35 0.42 0.50];
omega_o_grid = [1.7 2.0 2.3];
k_blend_grid = [0.60 0.70 0.80];
du_max_grid = [20 25 35];
lambda_aw_grid = [4 8 12];
k_aw_grid = [8 16 24];
k_sat_obs_grid = [0.0 2.0 5.0 8.0];

best = struct('score', inf);
total_cases = numel(omega_c_grid) * numel(omega_o_grid) * numel(k_blend_grid) * ...
    numel(du_max_grid) * numel(lambda_aw_grid) * numel(k_aw_grid) * numel(k_sat_obs_grid);
case_id = 0;

for omega_c = omega_c_grid
for omega_o = omega_o_grid
for k_blend = k_blend_grid
for du_max = du_max_grid
for lambda_aw = lambda_aw_grid
for k_aw = k_aw_grid
for k_sat_obs = k_sat_obs_grid
    case_id = case_id + 1;

    aca.b0 = 1.0;
    aca.Kp = omega_c^2;
    aca.Kd = 2*omega_c;
    aca.l1 = 3*omega_o;
    aca.l2 = 3*omega_o^2;
    aca.l3 = omega_o^3;
    aca.u_max = l1.u_max;
    aca.lambda_aw = lambda_aw;
    aca.k_aw = k_aw;
    aca.k_sat_obs = k_sat_obs;

    peak_ratio_l1 = zeros(1, numel(winds));
    rms_ratio_l1 = zeros(1, numel(winds));
    peak_ratio_adrc = zeros(1, numel(winds));
    rms_ratio_adrc = zeros(1, numel(winds));
    fail_gap_l1 = zeros(1, numel(winds));
    fail_gap_adrc = zeros(1, numel(winds));
    in5_gain_adrc = zeros(1, numel(winds));
    lag_rms = zeros(1, numel(winds));

    for wi = 1:numel(winds)
        [w_wind, tau_act, u_act_rate_max, V] = build_wind_case(winds(wi), N, dt, SEED + 10*wi);
        met_l1 = run_case(false, w_wind, tau_act, u_act_rate_max, V, dt, l1, aca, k_blend, du_max);
        met_ad = run_case_adrc(w_wind, tau_act, u_act_rate_max, V, dt, l1, aca, k_blend, du_max);
        met_ac = run_case(true,  w_wind, tau_act, u_act_rate_max, V, dt, l1, aca, k_blend, du_max);

        peak_ratio_l1(wi)   = met_ac.peak_ct / max(1e-6, met_l1.peak_ct);
        rms_ratio_l1(wi)    = met_ac.rms_ct  / max(1e-6, met_l1.rms_ct);
        peak_ratio_adrc(wi) = met_ac.peak_ct / max(1e-6, met_ad.peak_ct);
        rms_ratio_adrc(wi)  = met_ac.rms_ct  / max(1e-6, met_ad.rms_ct);
        fail_gap_l1(wi)     = met_l1.fail_flag - met_ac.fail_flag;
        fail_gap_adrc(wi)   = met_ad.fail_flag - met_ac.fail_flag;
        in5_gain_adrc(wi)   = met_ac.in5_pct - met_ad.in5_pct;
        lag_rms(wi)    = met_ac.cmd_lag_rms;
    end

    % Heavier weight on 12/15 m/s where the benchmark is hardest.
    w = [0.20 0.25 0.25 0.30];
    obj_peak_l1 = sum(w .* peak_ratio_l1);
    obj_rms_l1  = sum(w .* rms_ratio_l1);
    obj_peak_ad = sum(w .* peak_ratio_adrc);
    obj_rms_ad  = sum(w .* rms_ratio_adrc);
    obj_fail_l1 = 1 - min(1.0, mean(fail_gap_l1));
    obj_fail_ad = 1 - min(1.0, mean(fail_gap_adrc));
    obj_in5_ad  = 1 - min(1.0, max(0, mean(in5_gain_adrc))/30.0);

    obj_lag = min(1.0, mean(lag_rms) / 2.5);
    % Primary objective: ACA superiority vs standard ADRC while preserving
    % strong absolute performance vs L1.
    score = 0.25*obj_peak_l1 + 0.15*obj_rms_l1 + 0.20*obj_peak_ad + 0.15*obj_rms_ad + ...
            0.10*obj_fail_l1 + 0.10*obj_fail_ad + 0.03*obj_in5_ad + 0.02*obj_lag;

    if score < best.score
        best.score = score;
        best.omega_c = omega_c;
        best.omega_o = omega_o;
        best.k_blend = k_blend;
        best.du_max = du_max;
        best.lambda_aw = lambda_aw;
        best.k_aw = k_aw;
        best.k_sat_obs = k_sat_obs;
        best.peak_ratio_l1 = peak_ratio_l1;
        best.rms_ratio_l1 = rms_ratio_l1;
        best.peak_ratio_adrc = peak_ratio_adrc;
        best.rms_ratio_adrc = rms_ratio_adrc;
        best.fail_gap_l1 = fail_gap_l1;
        best.fail_gap_adrc = fail_gap_adrc;
        best.in5_gain_adrc = in5_gain_adrc;
        best.lag_rms = lag_rms;
        fprintf('  New best [%d/%d] score=%.4f wc=%.2f wo=%.2f kb=%.2f du=%.0f law=%.1f kaw=%.1f ksat=%.1f\n', ...
            case_id, total_cases, score, omega_c, omega_o, k_blend, du_max, lambda_aw, k_aw, k_sat_obs);
    end

end
end
end
end
end
end
end

save(fullfile(out_dir, 'tuned_params_B_industry.mat'), 'best');
fid = fopen(fullfile(out_dir, 'tuned_params_B_industry.csv'), 'w');
fprintf(fid, 'param,value\n');
fprintf(fid, 'score,%.6f\n', best.score);
fprintf(fid, 'omega_c,%.4f\n', best.omega_c);
fprintf(fid, 'omega_o,%.4f\n', best.omega_o);
fprintf(fid, 'k_blend,%.4f\n', best.k_blend);
fprintf(fid, 'du_max,%.4f\n', best.du_max);
fprintf(fid, 'lambda_aw,%.4f\n', best.lambda_aw);
fprintf(fid, 'k_aw,%.4f\n', best.k_aw);
fprintf(fid, 'k_sat_obs,%.4f\n', best.k_sat_obs);
fclose(fid);

fprintf('Direction B industry tuning complete. Best score=%.4f\n', best.score);

function met = run_case(use_aca, w_wind, tau_act, u_act_rate_max, V, dt, l1, aca, k_blend, du_max)
N = numel(w_wind);
y = 0; vy = 0; u_act = 0;
z = zeros(3,1); aw = 0;
u_prev = 0; u_cmd_prev = 0;

y_log = zeros(N,1);
sat_log = zeros(N,1);
u_cmd_log = zeros(N,1);
u_act_log = zeros(N,1);

for k = 1:N
    if ~use_aca
        [u_cmd, ~] = l1_guidance(y, vy, V, l1);
    else
        [u_l1, ~] = l1_guidance(y, vy, V, l1);
        [u_adrc, z, aw] = adrc_layer_aca(y, z, aw, u_prev, aca, dt);
        u_mix = (1-k_blend)*u_l1 + k_blend*u_adrc;
        du_lim = du_max * dt;
        u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
        u_cmd = max(-aca.u_max, min(aca.u_max, u_cmd));
    end

    u_prev = u_cmd;
    u_cmd_prev = u_cmd;

    du_act = (u_cmd - u_act)/tau_act;
    du_act = max(-u_act_rate_max, min(u_act_rate_max, du_act));
    u_act = u_act + dt*du_act;
    u_act = max(-l1.u_max, min(l1.u_max, u_act));

    vy = vy + dt*u_act;
    vy = max(-22, min(22, vy));
    y = y + dt*(vy + w_wind(k));

    y_log(k) = y;
    sat_log(k) = double(abs(u_cmd) >= 0.98*l1.u_max);
    u_cmd_log(k) = u_cmd;
    u_act_log(k) = u_act;
end

k0 = round(5.0/dt);
y_seg = y_log(k0:end);
met.peak_ct = max(abs(y_seg));
met.rms_ct = sqrt(mean(y_seg.^2));
met.sat_pct = 100*mean(sat_log(k0:end));
met.in5_pct = 100*mean(abs(y_seg) <= 5.0);
met.fail_flag = double(mean(abs(y_seg) > 20.0) > 0.10);
met.cmd_lag_rms = sqrt(mean((u_cmd_log(k0:end) - u_act_log(k0:end)).^2));
end

function met = run_case_adrc(w_wind, tau_act, u_act_rate_max, V, dt, l1, aca, k_blend, du_max)
N = numel(w_wind);
y = 0; vy = 0; u_act = 0;
z = zeros(3,1);
u_prev = 0; u_cmd_prev = 0;

y_log = zeros(N,1);
sat_log = zeros(N,1);
u_cmd_log = zeros(N,1);
u_act_log = zeros(N,1);

for k = 1:N
    [u_l1, ~] = l1_guidance(y, vy, V, l1);
    [u_adrc, z] = adrc_layer(y, z, u_prev, aca, dt);
    u_mix = (1-k_blend)*u_l1 + k_blend*u_adrc;
    du_lim = du_max * dt;
    u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
    u_cmd = max(-aca.u_max, min(aca.u_max, u_cmd));

    u_prev = u_cmd;
    u_cmd_prev = u_cmd;

    du_act = (u_cmd - u_act)/tau_act;
    du_act = max(-u_act_rate_max, min(u_act_rate_max, du_act));
    u_act = u_act + dt*du_act;
    u_act = max(-l1.u_max, min(l1.u_max, u_act));

    vy = vy + dt*u_act;
    vy = max(-22, min(22, vy));
    y = y + dt*(vy + w_wind(k));

    y_log(k) = y;
    sat_log(k) = double(abs(u_cmd) >= 0.98*l1.u_max);
    u_cmd_log(k) = u_cmd;
    u_act_log(k) = u_act;
end

k0 = round(5.0/dt);
y_seg = y_log(k0:end);
met.peak_ct = max(abs(y_seg));
met.rms_ct = sqrt(mean(y_seg.^2));
met.sat_pct = 100*mean(sat_log(k0:end));
met.in5_pct = 100*mean(abs(y_seg) <= 5.0);
met.fail_flag = double(mean(abs(y_seg) > 20.0) > 0.10);
met.cmd_lag_rms = sqrt(mean((u_cmd_log(k0:end) - u_act_log(k0:end)).^2));
end

function [w_wind, tau_act, u_act_rate_max, V] = build_wind_case(w_mean, N, dt, seed)
rng(seed);
V = 15.0;
tau_act_nom = 0.25;
act_deg_factor = (1.0 + 0.035 * max(0, w_mean - 5.0));
tau_act = tau_act_nom * act_deg_factor;
u_act_rate_nom = 18.0;
u_act_rate_max = u_act_rate_nom / max(1.0, act_deg_factor^1.35);

if w_mean <= 6
    sigma = 1.0; tau = 1.0;
elseif w_mean <= 10
    sigma = 1.8; tau = 0.8;
else
    sigma = 2.6; tau = 0.7;
end

a = exp(-dt/tau);
b = sigma*sqrt(1-a^2);
g = zeros(N,1);
for k = 2:N
    g(k) = a*g(k-1) + b*randn();
end
w_wind = w_mean + g;
end
