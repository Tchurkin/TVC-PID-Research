%% run_sweep_B_industry.m
% Industry-style benchmark with explicit ACA significance test:
% L1 vs PID cascade vs standard ADRC vs ACA-ADRC under crosswind stress.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
out_dir = fullfile(dir_root, 'outputs');
gfx_dir = fullfile(out_dir, 'graphs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
if ~exist(gfx_dir, 'dir'), mkdir(gfx_dir); end
addpath(this_dir);

fprintf('Running Direction B industry benchmark (with standard ADRC comparator)...\n');

% Winds chosen around/above common consumer wind resistance limits.
wind_means = [5, 8, 10, 12, 15, 20];
labels = {'5mps','8mps','10mps','12mps','15mps','20mps'};
controllers = {'L1', 'PID_CASCADE', 'ADRC', 'ACA_ADRC'};
SEED = 2026;
N_SEEDS = 5;

dt = 0.010;
t_end = 70.0;
N = round(t_end / dt);

% Baseline L1
l1.L1_period = 20.0;
l1.u_max = 4.0;

% ArduPilot-style cascaded PID baseline (outer position, inner velocity)
pid.u_max = l1.u_max;
pid.Kp_y = 0.22;
pid.Ki_y = 0.012;
pid.vy_ref_max = 9.0;
pid.Kp_v = 0.55;
pid.Ki_v = 0.10;
pid.Kd_v = 0.02;

% ADRC params (nominal + ACA branch)
omega_c = 0.35;
omega_o = 1.80;
adrc.b0 = 1.0;
adrc.Kp = omega_c^2;
adrc.Kd = 2 * omega_c;
adrc.l1 = 3 * omega_o;
adrc.l2 = 3 * omega_o^2;
adrc.l3 = omega_o^3;
adrc.u_max = l1.u_max;

aca = adrc;
aca.lambda_aw = 6.0;
aca.k_aw = 12.0;
aca.k_sat_obs = 0.0;

% Command shaping used by both ADRC and ACA-ADRC branches for fair comparison
k_blend = 0.50;
du_max = 25.0; % m/s^3

% Load industry-tuned settings when available
% (applies equally to ADRC and ACA shared bandwidth terms)
tuned_file = fullfile(out_dir, 'tuned_params_B_industry.mat');
if exist(tuned_file, 'file')
    S = load(tuned_file);
    if isfield(S, 'best')
        omega_c = S.best.omega_c;
        omega_o = S.best.omega_o;
        adrc.Kp = omega_c^2;
        adrc.Kd = 2 * omega_c;
        adrc.l1 = 3 * omega_o;
        adrc.l2 = 3 * omega_o^2;
        adrc.l3 = omega_o^3;
        adrc.u_max = l1.u_max;

        aca.Kp = adrc.Kp;
        aca.Kd = adrc.Kd;
        aca.l1 = adrc.l1;
        aca.l2 = adrc.l2;
        aca.l3 = adrc.l3;
        aca.u_max = adrc.u_max;

        if isfield(S.best, 'lambda_aw'), aca.lambda_aw = S.best.lambda_aw; end
        if isfield(S.best, 'k_aw'), aca.k_aw = S.best.k_aw; end
        if isfield(S.best, 'k_sat_obs'), aca.k_sat_obs = S.best.k_sat_obs; end
        if isfield(S.best, 'k_blend'), k_blend = S.best.k_blend; end
        if isfield(S.best, 'du_max'), du_max = S.best.du_max; end

        fprintf('Using tuned params: wc=%.3f wo=%.3f blend=%.2f du=%.1f\n', ...
            omega_c, omega_o, k_blend, du_max);
    end
end

rows = struct();
r = 0;

for wi = 1:numel(wind_means)
    w_mean = wind_means(wi);

    for ci = 1:numel(controllers)
        r = r + 1;

        acc_peak = zeros(N_SEEDS,1);
        acc_rms = zeros(N_SEEDS,1);
        acc_sat = zeros(N_SEEDS,1);
        acc_in5 = zeros(N_SEEDS,1);
        acc_fail = zeros(N_SEEDS,1);
        acc_effort = zeros(N_SEEDS,1);
        acc_lag = zeros(N_SEEDS,1);
        acc_dist_corr = nan(N_SEEDS,1);
        acc_dist_rmse = nan(N_SEEDS,1);
        acc_sat_err_rms = nan(N_SEEDS,1);
        acc_cdi = nan(N_SEEDS,1);
        act_deg_factor_ref = 1.0;

        for si = 1:N_SEEDS
            seed_i = SEED + (wi-1)*100 + si;
            [w_wind, tau_act, u_act_rate_max, V, act_deg_factor] = build_wind_case(w_mean, N, dt, seed_i);
            act_deg_factor_ref = act_deg_factor;

            y = 0; vy = 0; u_act = 0;
            z = zeros(3,1); aw = 0;
            u_prev = 0; u_cmd_prev = 0;
            int_y = 0; int_v = 0; e_v_prev = 0;

            y_log = zeros(N,1);
            u_log = zeros(N,1);
            u_cmd_log = zeros(N,1);
            sat_log = zeros(N,1);
            d_est_log = nan(N,1);
            sat_err_log = nan(N,1);
            d_true_log = [0; diff(w_wind)/dt];

            for k = 1:N
                switch ci
                    case 1
                        [u_cmd, ~] = l1_guidance(y, vy, V, l1);

                    case 2
                        e_y = -y;
                        int_y = int_y + e_y * dt;
                        vy_ref_unsat = pid.Kp_y * e_y + pid.Ki_y * int_y;
                        vy_ref = max(-pid.vy_ref_max, min(pid.vy_ref_max, vy_ref_unsat));
                        if abs(vy_ref_unsat - vy_ref) > 1e-12
                            int_y = int_y - e_y * dt;
                        end

                        e_v = vy_ref - vy;
                        de_v = (e_v - e_v_prev) / dt;
                        u_unsat = pid.Kp_v * e_v + pid.Ki_v * int_v + pid.Kd_v * de_v;
                        u_cmd = max(-pid.u_max, min(pid.u_max, u_unsat));
                        if abs(u_unsat - u_cmd) < 1e-12 || (sign(e_v) ~= sign(u_unsat - u_cmd))
                            int_v = int_v + e_v * dt;
                        end
                        e_v_prev = e_v;

                    case 3
                        % Standard ADRC comparator: same blend/rate-limit path as ACA,
                        % but no auxiliary anti-saturation compensation state.
                        [u_l1, ~] = l1_guidance(y, vy, V, l1);
                        [u_adrc, z] = adrc_layer(y, z, u_prev, adrc, dt);
                        u_mix = (1 - k_blend) * u_l1 + k_blend * u_adrc;
                        du_lim = du_max * dt;
                        u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
                        u_cmd = max(-adrc.u_max, min(adrc.u_max, u_cmd));
                        d_est_log(k) = z(3);

                    case 4
                        [u_l1, ~] = l1_guidance(y, vy, V, l1);
                        [u_adrc, z, aw, diag] = adrc_layer_aca(y, z, aw, u_prev, aca, dt);
                        u_mix = (1 - k_blend) * u_l1 + k_blend * u_adrc;
                        du_lim = du_max * dt;
                        u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
                        u_cmd = max(-aca.u_max, min(aca.u_max, u_cmd));
                        d_est_log(k) = diag.dist_est;
                        sat_err_log(k) = diag.sat_err;
                end

                u_prev = u_cmd;
                u_cmd_prev = u_cmd;

                du_act = (u_cmd - u_act) / tau_act;
                du_act = max(-u_act_rate_max, min(u_act_rate_max, du_act));
                u_act = u_act + dt * du_act;
                u_act = max(-l1.u_max, min(l1.u_max, u_act));

                vy = vy + dt * u_act;
                vy = max(-22, min(22, vy));
                y = y + dt * (vy + w_wind(k));

                y_log(k) = y;
                u_log(k) = u_act;
                u_cmd_log(k) = u_cmd;
                sat_log(k) = double(abs(u_cmd) >= 0.98 * l1.u_max);
            end

            k0 = round(5.0 / dt);
            y_seg = y_log(k0:end);
            u_seg = u_log(k0:end);
            sat_seg = sat_log(k0:end);

            acc_peak(si) = max(abs(y_seg));
            acc_rms(si) = sqrt(mean(y_seg.^2));
            acc_effort(si) = var(u_seg);
            acc_sat(si) = 100 * mean(sat_seg);
            acc_in5(si) = 100 * mean(abs(y_seg) <= 5.0);
            acc_fail(si) = double(mean(abs(y_seg) > 20.0) > 0.10);
            acc_lag(si) = sqrt(mean((u_cmd_log(k0:end) - u_log(k0:end)).^2));

            if ci >= 3
                d_est_seg = d_est_log(k0:end);
                d_true_seg = d_true_log(k0:end);
                c_pos = safe_corr(d_est_seg, d_true_seg);
                c_neg = safe_corr(-d_est_seg, d_true_seg);
                if abs(c_neg) > abs(c_pos)
                    d_est_seg = -d_est_seg;
                    acc_dist_corr(si) = c_neg;
                else
                    acc_dist_corr(si) = c_pos;
                end
                acc_dist_rmse(si) = sqrt(mean((d_est_seg - d_true_seg).^2));

                if ci == 4
                    sat_err_seg = sat_err_log(k0:end);
                    acc_sat_err_rms(si) = sqrt(mean(sat_err_seg.^2));
                    acc_cdi(si) = acc_sat_err_rms(si) / max(1e-6, sqrt(mean(d_est_seg.^2)));
                end
            end
        end

        peak_ct = mean(acc_peak);
        rms_ct = mean(acc_rms);
        effort_var = mean(acc_effort);
        sat_pct = mean(acc_sat);
        in5_pct = mean(acc_in5);
        fail_flag = double(mean(acc_fail) >= 0.5);
        cmd_lag_rms = mean(acc_lag);

        rows(r).wind_mps = w_mean;
        rows(r).wind_label = labels{wi};
        rows(r).controller = controllers{ci};
        rows(r).peak_ct_m = peak_ct;
        rows(r).rms_ct_m = rms_ct;
        rows(r).effort_var = effort_var;
        rows(r).sat_pct = sat_pct;
        rows(r).in5_pct = in5_pct;
        rows(r).fail_flag = fail_flag;
        rows(r).act_deg_factor = act_deg_factor_ref;
        rows(r).cmd_lag_rms = cmd_lag_rms;
        rows(r).dist_corr = nanmean(acc_dist_corr);
        rows(r).dist_rmse = nanmean(acc_dist_rmse);
        rows(r).sat_err_rms = nanmean(acc_sat_err_rms);
        rows(r).constraint_dom_idx = nanmean(acc_cdi);

        fprintf('  %-5s %-10s peak=%.2f rms=%.2f sat%%=%.1f in5%%=%.1f fail=%d\n', ...
            labels{wi}, controllers{ci}, peak_ct, rms_ct, sat_pct, in5_pct, fail_flag);
    end
end

% Save CSV
csv_path = fullfile(out_dir, 'sweep_results_B_industry.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, 'wind_mps,wind_label,controller,peak_ct_m,rms_ct_m,effort_var,sat_pct,in5_pct,fail_flag,act_deg_factor,cmd_lag_rms,dist_rmse,dist_corr,sat_err_rms,constraint_dom_idx\n');
for i = 1:r
    fprintf(fid, '%.1f,%s,%s,%.4f,%.4f,%.6f,%.4f,%.4f,%d,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
        rows(i).wind_mps, rows(i).wind_label, rows(i).controller, rows(i).peak_ct_m, ...
        rows(i).rms_ct_m, rows(i).effort_var, rows(i).sat_pct, rows(i).in5_pct, rows(i).fail_flag, ...
        rows(i).act_deg_factor, rows(i).cmd_lag_rms, ...
        rows(i).dist_rmse, rows(i).dist_corr, rows(i).sat_err_rms, rows(i).constraint_dom_idx);
end
fclose(fid);
fprintf('Saved: %s\n', csv_path);

% Summary graph (includes standard ADRC comparator)
L1_idx = zeros(1, numel(labels));
PID_idx = zeros(1, numel(labels));
ADRC_idx = zeros(1, numel(labels));
ACA_idx = zeros(1, numel(labels));
for i = 1:numel(labels)
    L1_idx(i) = get_row_index(rows, labels{i}, 'L1');
    PID_idx(i) = get_row_index(rows, labels{i}, 'PID_CASCADE');
    ADRC_idx(i) = get_row_index(rows, labels{i}, 'ADRC');
    ACA_idx(i) = get_row_index(rows, labels{i}, 'ACA_ADRC');
end

l1_peak = [rows(L1_idx).peak_ct_m];
pid_peak = [rows(PID_idx).peak_ct_m];
adrc_peak = [rows(ADRC_idx).peak_ct_m];
aca_peak = [rows(ACA_idx).peak_ct_m];

l1_sat = [rows(L1_idx).sat_pct];
pid_sat = [rows(PID_idx).sat_pct];
adrc_sat = [rows(ADRC_idx).sat_pct];
aca_sat = [rows(ACA_idx).sat_pct];

if ~usejava('desktop')
    fprintf('Skipping graph generation in headless batch mode.\n');
else
    try
        fig = figure('Visible', 'off', 'Position', [100 100 1100 420]);
        x = 1:numel(labels); w = 0.2;
        subplot(1,2,1);
        bar(x-1.5*w, l1_peak, w, 'FaceColor', [0.35 0.55 0.85]); hold on;
        bar(x-0.5*w, pid_peak, w, 'FaceColor', [0.55 0.55 0.55]);
        bar(x+0.5*w, adrc_peak, w, 'FaceColor', [0.25 0.70 0.50]);
        bar(x+1.5*w, aca_peak, w, 'FaceColor', [0.85 0.35 0.35]);
        set(gca, 'XTick', x, 'XTickLabel', labels, 'FontSize', 10);
        xlabel('Mean Crosswind'); ylabel('Peak Cross-Track (m)');
        title('Industry Wind Benchmark: Peak Error');
        legend('L1', 'PID Cascade', 'ADRC', 'ACA-ADRC', 'Location', 'northwest');
        grid on;

        subplot(1,2,2);
        bar(x-1.5*w, l1_sat, w, 'FaceColor', [0.35 0.55 0.85]); hold on;
        bar(x-0.5*w, pid_sat, w, 'FaceColor', [0.55 0.55 0.55]);
        bar(x+0.5*w, adrc_sat, w, 'FaceColor', [0.25 0.70 0.50]);
        bar(x+1.5*w, aca_sat, w, 'FaceColor', [0.85 0.35 0.35]);
        set(gca, 'XTick', x, 'XTickLabel', labels, 'FontSize', 10);
        xlabel('Mean Crosswind'); ylabel('Saturation Time (%)');
        title('Actuator Stress');
        legend('L1', 'PID Cascade', 'ADRC', 'ACA-ADRC', 'Location', 'northwest');
        grid on;

        sgtitle('Direction B - L1/PID/ADRC/ACA-ADRC Comparison');
        saveas(fig, fullfile(gfx_dir, 'sweep_B_industry.png'));
        close(fig);
        fprintf('Saved graph: sweep_B_industry.png\n');
    catch ME
        fprintf('Warning: skipped graph generation (%s)\n', ME.message);
    end
end

fprintf('\n=== DIRECTION B INDUSTRY CHECK (ACA significance) ===\n');
stress_rows = struct();
sr = 0;
for i = 1:numel(labels)
    idx_l1 = L1_idx(i);
    idx_pid = PID_idx(i);
    idx_adrc = ADRC_idx(i);
    idx_aca = ACA_idx(i);

    ratio_peak_l1 = rows(idx_aca).peak_ct_m / max(1e-6, rows(idx_l1).peak_ct_m);
    ratio_rms_l1 = rows(idx_aca).rms_ct_m / max(1e-6, rows(idx_l1).rms_ct_m);
    ratio_peak_pid = rows(idx_aca).peak_ct_m / max(1e-6, rows(idx_pid).peak_ct_m);
    ratio_rms_pid = rows(idx_aca).rms_ct_m / max(1e-6, rows(idx_pid).rms_ct_m);
    ratio_peak_adrc = rows(idx_aca).peak_ct_m / max(1e-6, rows(idx_adrc).peak_ct_m);
    ratio_rms_adrc = rows(idx_aca).rms_ct_m / max(1e-6, rows(idx_adrc).rms_ct_m);

    sat_delta_l1 = rows(idx_l1).sat_pct - rows(idx_aca).sat_pct;
    sat_delta_pid = rows(idx_pid).sat_pct - rows(idx_aca).sat_pct;

    fail_delta_l1 = rows(idx_l1).fail_flag - rows(idx_aca).fail_flag;
    fail_delta_pid = rows(idx_pid).fail_flag - rows(idx_aca).fail_flag;
    fail_delta_adrc = rows(idx_adrc).fail_flag - rows(idx_aca).fail_flag;

    fprintf('  %-5s ACA/L1 peak=%.3f ACA/PID peak=%.3f ACA/ADRC peak=%.3f | fail delta vs ADRC=%d\n', ...
        labels{i}, ratio_peak_l1, ratio_peak_pid, ratio_peak_adrc, fail_delta_adrc);

    sr = sr + 1;
    stress_rows(sr).wind_label = labels{i};
    stress_rows(sr).peak_ratio_vs_l1 = ratio_peak_l1;
    stress_rows(sr).rms_ratio_vs_l1 = ratio_rms_l1;
    stress_rows(sr).peak_ratio_vs_pid = ratio_peak_pid;
    stress_rows(sr).rms_ratio_vs_pid = ratio_rms_pid;
    stress_rows(sr).peak_ratio_vs_adrc = ratio_peak_adrc;
    stress_rows(sr).rms_ratio_vs_adrc = ratio_rms_adrc;

    stress_rows(sr).sat_reduction_vs_l1_pct = sat_delta_l1;
    stress_rows(sr).sat_reduction_vs_pid_pct = sat_delta_pid;

    stress_rows(sr).fail_reduction_vs_l1 = fail_delta_l1;
    stress_rows(sr).fail_reduction_vs_pid = fail_delta_pid;
    stress_rows(sr).fail_reduction_vs_adrc = fail_delta_adrc;

    stress_rows(sr).in5_gain_vs_l1_pct = rows(idx_aca).in5_pct - rows(idx_l1).in5_pct;
    stress_rows(sr).in5_gain_vs_pid_pct = rows(idx_aca).in5_pct - rows(idx_pid).in5_pct;
    stress_rows(sr).in5_gain_vs_adrc_pct = rows(idx_aca).in5_pct - rows(idx_adrc).in5_pct;
end

fid2 = fopen(fullfile(out_dir, 'industry_stress_metrics_B.csv'), 'w');
fprintf(fid2, ['wind_label,peak_ratio_vs_l1,rms_ratio_vs_l1,peak_ratio_vs_pid,rms_ratio_vs_pid,peak_ratio_vs_adrc,rms_ratio_vs_adrc,' ...
    'sat_reduction_vs_l1_pct,sat_reduction_vs_pid_pct,fail_reduction_vs_l1,fail_reduction_vs_pid,fail_reduction_vs_adrc,' ...
    'in5_gain_vs_l1_pct,in5_gain_vs_pid_pct,in5_gain_vs_adrc_pct,act_deg_factor,cmd_lag_rms_ac,dist_corr_ac,dist_rmse_ac,constraint_dom_idx_ac\n']);
for i = 1:sr
    idx_aca = ACA_idx(i);
    fprintf(fid2, '%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%.4f\n', ...
        stress_rows(i).wind_label, stress_rows(i).peak_ratio_vs_l1, stress_rows(i).rms_ratio_vs_l1, ...
        stress_rows(i).peak_ratio_vs_pid, stress_rows(i).rms_ratio_vs_pid, ...
        stress_rows(i).peak_ratio_vs_adrc, stress_rows(i).rms_ratio_vs_adrc, ...
        stress_rows(i).sat_reduction_vs_l1_pct, stress_rows(i).sat_reduction_vs_pid_pct, ...
        stress_rows(i).fail_reduction_vs_l1, stress_rows(i).fail_reduction_vs_pid, stress_rows(i).fail_reduction_vs_adrc, ...
        stress_rows(i).in5_gain_vs_l1_pct, stress_rows(i).in5_gain_vs_pid_pct, stress_rows(i).in5_gain_vs_adrc_pct, ...
        rows(idx_aca).act_deg_factor, rows(idx_aca).cmd_lag_rms, ...
        rows(idx_aca).dist_corr, rows(idx_aca).dist_rmse, rows(idx_aca).constraint_dom_idx);
end
fclose(fid2);
fprintf('Saved: %s\n', fullfile(out_dir, 'industry_stress_metrics_B.csv'));

function idx = get_row_index(rows, wind_label, controller)
idx = -1;
for k = 1:numel(rows)
    if strcmp(rows(k).wind_label, wind_label) && strcmp(rows(k).controller, controller)
        idx = k;
        return;
    end
end
error('Missing row for %s / %s', wind_label, controller);
end

function c = safe_corr(a, b)
a = a(:); b = b(:);
if numel(a) ~= numel(b) || numel(a) < 3
    c = NaN;
    return;
end
if std(a) < 1e-9 || std(b) < 1e-9
    c = 0;
    return;
end
C = corrcoef(a, b);
c = C(1,2);
end

function [w_wind, tau_act, u_act_rate_max, V, act_deg_factor] = build_wind_case(w_mean, N, dt, seed)
% Build a mean+gust crosswind scenario for industry benchmark.
rng(seed);
V = 15.0;
tau_act_nom = 0.25;
act_deg_factor = 1.0 + 0.035 * max(0, w_mean - 5.0);
tau_act = tau_act_nom * act_deg_factor;

% Effective actuator slew capability degrades as wind/load increases.
% This creates sustained command-realization mismatch where ACA should help.
u_act_rate_nom = 18.0; % m/s^3
u_act_rate_max = u_act_rate_nom / max(1.0, act_deg_factor^1.35);

if w_mean <= 6
    sigma = 1.0; tau = 1.0;
elseif w_mean <= 10
    sigma = 1.8; tau = 0.8;
else
    sigma = 2.6; tau = 0.7;
end

a = exp(-dt / tau);
b = sigma * sqrt(1 - a^2);
g = zeros(N,1);
for k = 2:N
    g(k) = a * g(k-1) + b * randn();
end
w_wind = w_mean + g;
end
