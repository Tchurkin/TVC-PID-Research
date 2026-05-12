%% run_sweep_adaptive.m
% Self-tuning ADRC benchmark with nominal and degraded actuator scenarios.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
out_dir = fullfile(dir_root, 'outputs');
gfx_dir = fullfile(out_dir, 'graphs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
if ~exist(gfx_dir, 'dir'), mkdir(gfx_dir); end
addpath(this_dir);

fprintf('Running adaptive benchmark sweep...\n');

wind_means = [5, 8, 10, 12, 15, 20];
labels = {'5mps','8mps','10mps','12mps','15mps','20mps'};
controllers = {'L1', 'PID_CASCADE', 'ADRC_FIXED', 'ADRC_ADAPTIVE'};
SEED = 4040;
N_SEEDS = 5;

dt = 0.010;
t_end = 70.0;
N = round(t_end / dt);

l1.L1_period = 20.0;
l1.u_max = 4.0;

pid.u_max = l1.u_max;
pid.Kp_y = 0.22;
pid.Ki_y = 0.012;
pid.vy_ref_max = 9.0;
pid.Kp_v = 0.55;
pid.Ki_v = 0.10;
pid.Kd_v = 0.02;

omega_c = 0.80;
omega_o = 3.20;
adrc.b0 = 1.0;
adrc.Kp = omega_c^2;
adrc.Kd = 2 * omega_c;
adrc.l1 = 3 * omega_o;
adrc.l2 = 3 * omega_o^2;
adrc.l3 = omega_o^3;
adrc.u_max = l1.u_max;

cfg.k_blend = 0.90;
cfg.du_max = 35.0;
cfg.k_blend_fault = 0.90;
cfg.du_max_fault = 35.0;
cfg.lambda_rls = 0.98;
cfg.drift_threshold = 0.15;
cfg.omega_o_rate = 1.0;
cfg.tau_exp = 0.55;
cfg.gain_exp = 0.35;
cfg.bw_floor = 0.35;
cfg.prbs_amp = 0.30;
cfg.prbs_duration = 3.0;
cfg.adapt_guard_s = 0.5;
cfg.degrade_time = 20.0;
cfg.degrade_tau_scale = 3.0;
cfg.degrade_gain_scale = 0.45;
cfg.degrade_rate_scale = 0.35;

% Load tuned adaptive parameters when available.
tuned_file = fullfile(out_dir, 'tuned_params_adaptive.mat');
if exist(tuned_file, 'file')
    S = load(tuned_file);
    if isfield(S, 'best')
        if isfield(S.best, 'lambda_rls'), cfg.lambda_rls = S.best.lambda_rls; end
        if isfield(S.best, 'drift_threshold'), cfg.drift_threshold = S.best.drift_threshold; end
        if isfield(S.best, 'omega_o_rate'), cfg.omega_o_rate = S.best.omega_o_rate; end
        if isfield(S.best, 'prbs_amp'), cfg.prbs_amp = S.best.prbs_amp; end
        if isfield(S.best, 'prbs_duration'), cfg.prbs_duration = S.best.prbs_duration; end
        fprintf('Using tuned adaptive params: lambda=%.3f drift=%.3f rate=%.2f prbs=%.2f/%.1fs\n', ...
            cfg.lambda_rls, cfg.drift_threshold, cfg.omega_o_rate, cfg.prbs_amp, cfg.prbs_duration);
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
        acc_sysid_gain_err = zeros(N_SEEDS,1);
        acc_sysid_tau_err = zeros(N_SEEDS,1);

        act_deg_factor_ref = 1.0;

        for si = 1:N_SEEDS
            seed_i = SEED + 100 * (wi-1) + si;
            [w_wind, tau_act, u_act_rate_max, V, act_deg_factor] = build_wind_case(w_mean, N, dt, seed_i);
            act_deg_factor_ref = act_deg_factor;

            met = simulate_case(controllers{ci}, w_wind, tau_act, u_act_rate_max, V, dt, l1, pid, adrc, cfg, false, seed_i);
            acc_peak(si) = met.peak_ct;
            acc_rms(si) = met.rms_ct;
            acc_sat(si) = met.sat_pct;
            acc_in5(si) = met.in5_pct;
            acc_fail(si) = met.fail_flag;
            acc_effort(si) = met.effort_var;
            acc_lag(si) = met.cmd_lag_rms;
            acc_sysid_gain_err(si) = met.sysid_gain_err;
            acc_sysid_tau_err(si) = met.sysid_tau_err;
        end

        rows(r).wind_mps = w_mean;
        rows(r).wind_label = labels{wi};
        rows(r).controller = controllers{ci};
        rows(r).peak_ct_m = mean(acc_peak);
        rows(r).rms_ct_m = mean(acc_rms);
        rows(r).effort_var = mean(acc_effort);
        rows(r).sat_pct = mean(acc_sat);
        rows(r).in5_pct = mean(acc_in5);
        rows(r).fail_flag = double(mean(acc_fail) >= 0.5);
        rows(r).act_deg_factor = act_deg_factor_ref;
        rows(r).cmd_lag_rms = mean(acc_lag);
        rows(r).sysid_gain_err = mean(acc_sysid_gain_err);
        rows(r).sysid_tau_err = mean(acc_sysid_tau_err);

        fprintf('  %-5s %-13s peak=%.2f rms=%.2f in5=%.1f fail=%d\n', ...
            labels{wi}, controllers{ci}, rows(r).peak_ct_m, rows(r).rms_ct_m, rows(r).in5_pct, rows(r).fail_flag);
    end
end

csv_path = fullfile(out_dir, 'sweep_results_adaptive.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, 'wind_mps,wind_label,controller,peak_ct_m,rms_ct_m,effort_var,sat_pct,in5_pct,fail_flag,act_deg_factor,cmd_lag_rms,sysid_gain_err,sysid_tau_err\n');
for i = 1:r
    fprintf(fid, '%.1f,%s,%s,%.4f,%.4f,%.6f,%.4f,%.4f,%d,%.4f,%.6f,%.6f,%.6f\n', ...
        rows(i).wind_mps, rows(i).wind_label, rows(i).controller, rows(i).peak_ct_m, rows(i).rms_ct_m, ...
        rows(i).effort_var, rows(i).sat_pct, rows(i).in5_pct, rows(i).fail_flag, rows(i).act_deg_factor, ...
        rows(i).cmd_lag_rms, rows(i).sysid_gain_err, rows(i).sysid_tau_err);
end
fclose(fid);
fprintf('Saved: %s\n', csv_path);

% Build stress metrics for scoreboard/council.
stress = struct();
sr = 0;
for wi = 1:numel(labels)
    iL1 = get_row_index(rows, labels{wi}, 'L1');
    iPID = get_row_index(rows, labels{wi}, 'PID_CASCADE');
    iFX = get_row_index(rows, labels{wi}, 'ADRC_FIXED');
    iAD = get_row_index(rows, labels{wi}, 'ADRC_ADAPTIVE');

    sr = sr + 1;
    stress(sr).wind_label = labels{wi};
    stress(sr).peak_ratio_vs_pid = rows(iAD).peak_ct_m / max(1e-6, rows(iPID).peak_ct_m);
    stress(sr).rms_ratio_vs_pid = rows(iAD).rms_ct_m / max(1e-6, rows(iPID).rms_ct_m);
    stress(sr).peak_ratio_vs_l1 = rows(iAD).peak_ct_m / max(1e-6, rows(iL1).peak_ct_m);
    stress(sr).rms_ratio_vs_l1 = rows(iAD).rms_ct_m / max(1e-6, rows(iL1).rms_ct_m);
    stress(sr).peak_ratio_vs_adrcfixed = rows(iAD).peak_ct_m / max(1e-6, rows(iFX).peak_ct_m);
    stress(sr).rms_ratio_vs_adrcfixed = rows(iAD).rms_ct_m / max(1e-6, rows(iFX).rms_ct_m);
    stress(sr).fail_reduction_vs_pid = rows(iPID).fail_flag - rows(iAD).fail_flag;
    stress(sr).fail_reduction_vs_adrcfixed = rows(iFX).fail_flag - rows(iAD).fail_flag;
    stress(sr).in5_gain_vs_pid_pct = rows(iAD).in5_pct - rows(iPID).in5_pct;
    stress(sr).in5_gain_vs_adrcfixed_pct = rows(iAD).in5_pct - rows(iFX).in5_pct;
    stress(sr).cmd_lag_rms_adaptive = rows(iAD).cmd_lag_rms;
    stress(sr).sysid_gain_err = rows(iAD).sysid_gain_err;
    stress(sr).sysid_tau_err = rows(iAD).sysid_tau_err;
end

stress_file = fullfile(out_dir, 'industry_stress_metrics_adaptive.csv');
fid2 = fopen(stress_file, 'w');
fprintf(fid2, 'wind_label,peak_ratio_vs_pid,rms_ratio_vs_pid,peak_ratio_vs_l1,rms_ratio_vs_l1,peak_ratio_vs_adrcfixed,rms_ratio_vs_adrcfixed,fail_reduction_vs_pid,fail_reduction_vs_adrcfixed,in5_gain_vs_pid_pct,in5_gain_vs_adrcfixed_pct,cmd_lag_rms_adaptive,sysid_gain_err,sysid_tau_err\n');
for i = 1:sr
    fprintf(fid2, '%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.4f,%.4f,%.6f,%.6f,%.6f\n', ...
        stress(i).wind_label, stress(i).peak_ratio_vs_pid, stress(i).rms_ratio_vs_pid, ...
        stress(i).peak_ratio_vs_l1, stress(i).rms_ratio_vs_l1, ...
        stress(i).peak_ratio_vs_adrcfixed, stress(i).rms_ratio_vs_adrcfixed, ...
        stress(i).fail_reduction_vs_pid, stress(i).fail_reduction_vs_adrcfixed, ...
        stress(i).in5_gain_vs_pid_pct, stress(i).in5_gain_vs_adrcfixed_pct, ...
        stress(i).cmd_lag_rms_adaptive, stress(i).sysid_gain_err, stress(i).sysid_tau_err);
end
fclose(fid2);
fprintf('Saved: %s\n', stress_file);

% Degradation scenario (money-shot benchmark).
trace_seed = 1;
deg_rows = struct();
dr = 0;
trace_pack = struct();
for ci = 1:numel(controllers)
    controller = controllers{ci};
    rec = zeros(N_SEEDS,1);
    post_rms = zeros(N_SEEDS,1);
    post_peak = zeros(N_SEEDS,1);
    sysid_gain_err = zeros(N_SEEDS,1);
    sysid_tau_err = zeros(N_SEEDS,1);

    for si = 1:N_SEEDS
        seed_i = 6060 + si;
        [w_wind, tau_act, u_act_rate_max, V] = build_wind_case(10, N, dt, seed_i);
        met = simulate_case(controller, w_wind, tau_act, u_act_rate_max, V, dt, l1, pid, adrc, cfg, true, seed_i);
        rec(si) = met.recovery_time_s;
        post_rms(si) = met.post_fault_rms;
        post_peak(si) = met.post_fault_peak;
        sysid_gain_err(si) = met.sysid_gain_err;
        sysid_tau_err(si) = met.sysid_tau_err;

        if si == trace_seed
            trace_pack.(controller) = met.trace;
        end
    end

    dr = dr + 1;
    deg_rows(dr).controller = controller;
    deg_rows(dr).recovery_time_s = mean(rec);
    deg_rows(dr).post_fault_rms = mean(post_rms);
    deg_rows(dr).post_fault_peak = mean(post_peak);
    deg_rows(dr).sysid_gain_err = mean(sysid_gain_err);
    deg_rows(dr).sysid_tau_err = mean(sysid_tau_err);

    fprintf('  Degrade %-13s rec=%.2fs postRMS=%.2f\n', controller, deg_rows(dr).recovery_time_s, deg_rows(dr).post_fault_rms);
end

deg_file = fullfile(out_dir, 'degradation_results_adaptive.csv');
fid3 = fopen(deg_file, 'w');
fprintf(fid3, 'controller,recovery_time_s,post_fault_rms,post_fault_peak,sysid_gain_err,sysid_tau_err\n');
for i = 1:dr
    fprintf(fid3, '%s,%.4f,%.4f,%.4f,%.6f,%.6f\n', deg_rows(i).controller, deg_rows(i).recovery_time_s, deg_rows(i).post_fault_rms, deg_rows(i).post_fault_peak, deg_rows(i).sysid_gain_err, deg_rows(i).sysid_tau_err);
end
fclose(fid3);
fprintf('Saved: %s\n', deg_file);

trace_pack.t = (0:N-1)' * dt;
trace_pack.cfg = cfg;
trace_pack.degrade_time = cfg.degrade_time;
save(fullfile(out_dir, 'degradation_trace_adaptive.mat'), 'trace_pack');
fprintf('Saved: %s\n', fullfile(out_dir, 'degradation_trace_adaptive.mat'));

function met = simulate_case(controller, w_wind, tau_act_nom, u_act_rate_nom, V, dt, l1, pid, adrc, cfg, use_degrade, seed)
N = numel(w_wind);
y = 0; vy = 0; u_act = 0;
z = zeros(3,1);
u_prev = 0; u_cmd_prev = 0;
int_y = 0; int_v = 0; e_v_prev = 0;

% Unknown baseline actuator uncertainty (manufacturing + wear spread).
tau_nom_true = tau_act_nom;
gain_nom_true = 1.0;
if use_degrade
    rng(seed + 12345);
    tau_nom_true = tau_act_nom * (1.0 + 0.30 * randn());
    tau_nom_true = max(0.12, min(0.70, tau_nom_true));
    gain_nom_true = 1.0 + 0.20 * randn();
    gain_nom_true = max(0.60, min(1.35, gain_nom_true));
end

adp = struct();
adp.t = 0;
adp.omega_o_eff = adrc.l1 / 3;
adp.gain_est = 1.0;
adp.tau_est = tau_nom_true;
adp.has_prev = false;
adp.rls = struct('theta', [exp(-dt/max(1e-3, tau_nom_true)); 1-exp(-dt/max(1e-3, tau_nom_true))], 'P', 50*eye(2));
adp.u_act_prev = 0;
adp.u_cmd_prev = 0;

sysid = struct('gain_est', 1.0, 'tau_est', tau_nom_true, 'fit_rmse', 0);
if strcmp(controller, 'ADRC_ADAPTIVE')
    sysid = run_preflight_sysid(dt, l1.u_max, cfg.prbs_amp, cfg.prbs_duration, tau_nom_true, gain_nom_true, u_act_rate_nom, seed);
    adp.gain_est = sysid.gain_est;
    adp.tau_est = sysid.tau_est;
end

apar = struct();
apar.b0_nominal = adrc.b0;
apar.gain_nom = max(0.3, sysid.gain_est);
apar.tau_nom = max(0.05, sysid.tau_est);
apar.omega_o_nom = adrc.l1 / 3;
apar.Kp = adrc.Kp;
apar.Kd = adrc.Kd;
apar.u_max = adrc.u_max;
apar.lambda_rls = cfg.lambda_rls;
apar.omega_o_rate = cfg.omega_o_rate;
apar.adapt_guard_s = cfg.adapt_guard_s;
apar.tau_exp = cfg.tau_exp;
apar.gain_exp = cfg.gain_exp;
apar.bw_floor = cfg.bw_floor;

trace.t = (0:N-1)' * dt;
trace.y = zeros(N,1);
trace.u_cmd = zeros(N,1);
trace.u_act = zeros(N,1);
trace.gain_est = nan(N,1);
trace.tau_est = nan(N,1);
trace.omega_o = nan(N,1);

for k = 1:N
    t = (k-1) * dt;
    switch controller
        case 'L1'
            [u_cmd, ~] = l1_guidance(y, vy, V, l1);

        case 'PID_CASCADE'
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

        case 'ADRC_FIXED'
            [u_l1, ~] = l1_guidance(y, vy, V, l1);
            [u_ad, z] = adrc_layer(y, z, u_prev, adrc, dt);
            u_mix = (1 - cfg.k_blend) * u_l1 + cfg.k_blend * u_ad;
            du_lim = cfg.du_max * dt;
            u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
            u_cmd = max(-adrc.u_max, min(adrc.u_max, u_cmd));

        case 'ADRC_ADAPTIVE'
            [u_l1, ~] = l1_guidance(y, vy, V, l1);
            [u_ad, z, adp, diag] = adrc_layer_adaptive(y, u_act, z, u_prev, adp, apar, dt);
            drift = abs(diag.gain_est - apar.gain_nom) / max(1e-6, apar.gain_nom);
            if drift > cfg.drift_threshold
                apar.omega_o_rate = cfg.omega_o_rate * 1.4;
                k_blend_eff = cfg.k_blend_fault;
                du_max_eff = cfg.du_max_fault;
            else
                apar.omega_o_rate = cfg.omega_o_rate;
                k_blend_eff = cfg.k_blend;
                du_max_eff = cfg.du_max;
            end
            u_mix = (1 - k_blend_eff) * u_l1 + k_blend_eff * u_ad;
            du_lim = du_max_eff * dt;
            u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
            u_cmd = max(-adrc.u_max, min(adrc.u_max, u_cmd));
            trace.gain_est(k) = diag.gain_est;
            trace.tau_est(k) = diag.tau_est;
            trace.omega_o(k) = diag.omega_o_eff;

        otherwise
            error('Unknown controller: %s', controller);
    end

    u_prev = u_cmd;
    u_cmd_prev = u_cmd;

    tau_act = tau_nom_true;
    gain_act = gain_nom_true;
    u_rate = u_act_rate_nom;
    if use_degrade && t >= cfg.degrade_time
        tau_act = tau_nom_true * cfg.degrade_tau_scale;
        gain_act = gain_nom_true * cfg.degrade_gain_scale;
        u_rate = u_act_rate_nom * cfg.degrade_rate_scale;
    end

    du_act = (gain_act * u_cmd - u_act) / tau_act;
    du_act = max(-u_rate, min(u_rate, du_act));
    u_act = u_act + dt * du_act;
    u_act = max(-l1.u_max, min(l1.u_max, u_act));

    vy = vy + dt * u_act;
    vy = max(-22, min(22, vy));
    y = y + dt * (vy + w_wind(k));

    trace.y(k) = y;
    trace.u_cmd(k) = u_cmd;
    trace.u_act(k) = u_act;
end

k0 = round(5.0 / dt);
y_seg = trace.y(k0:end);
met.peak_ct = max(abs(y_seg));
met.rms_ct = sqrt(mean(y_seg.^2));
met.effort_var = var(trace.u_act(k0:end));
met.sat_pct = 100 * mean(abs(trace.u_cmd(k0:end)) >= 0.98 * l1.u_max);
met.in5_pct = 100 * mean(abs(y_seg) <= 5.0);
met.fail_flag = double(mean(abs(y_seg) > 20.0) > 0.10);
met.cmd_lag_rms = sqrt(mean((trace.u_cmd(k0:end) - trace.u_act(k0:end)).^2));
met.sysid_gain_err = abs(sysid.gain_est - gain_nom_true);
met.sysid_tau_err = abs(sysid.tau_est - tau_nom_true);

if use_degrade
    kf = max(1, round(cfg.degrade_time / dt));
    kf2 = min(N, kf + round(8.0 / dt));
    y_post = trace.y(kf:kf2);
    met.post_fault_rms = sqrt(mean(y_post.^2));
    met.post_fault_peak = max(abs(y_post));
    met.recovery_time_s = recovery_time(trace.t, trace.y, cfg.degrade_time, 5.0, 2.0);
else
    met.post_fault_rms = NaN;
    met.post_fault_peak = NaN;
    met.recovery_time_s = NaN;
end

met.trace = trace;
end

function id = run_preflight_sysid(dt, u_max, prbs_amp, t_pre, tau_act, gain_act, u_rate_max, seed)
N = max(40, round(t_pre / dt));
rng(seed + 9000);
bits = sign(randn(N,1));
u_cmd = prbs_amp * u_max * bits;
u_act = zeros(N,1);
for k = 2:N
    du = (gain_act * u_cmd(k-1) - u_act(k-1)) / tau_act;
    du = max(-u_rate_max, min(u_rate_max, du));
    u_act(k) = u_act(k-1) + dt * du;
end
id = sys_id_preflight(u_cmd, u_act, dt);
end

function t_rec = recovery_time(t, y, t_fault, band, hold_s)
idx0 = find(t >= t_fault, 1, 'first');
if isempty(idx0)
    t_rec = NaN;
    return;
end

dt = mean(diff(t));
Nhold = max(1, round(hold_s / dt));
t_rec = NaN;
for k = idx0:(numel(y)-Nhold)
    if all(abs(y(k:k+Nhold-1)) <= band)
        t_rec = t(k) - t_fault;
        return;
    end
end
end

function idx = get_row_index(rows, wind_label, controller)
matches = find(arrayfun(@(rr) strcmp(rr.wind_label, wind_label) && strcmp(rr.controller, controller), rows));
if isempty(matches)
    error('Missing row for %s / %s', wind_label, controller);
end
idx = matches(1);
end

function [w_wind, tau_act, u_act_rate_max, V, act_deg_factor] = build_wind_case(w_mean, N, dt, seed)
rng(seed);
V = 15.0;
tau_act_nom = 0.25;
act_deg_factor = 1.0 + 0.035 * max(0, w_mean - 5.0);
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

a = exp(-dt / tau);
b = sigma * sqrt(1 - a^2);
g = zeros(N,1);
for k = 2:N
    g(k) = a * g(k-1) + b * randn();
end
w_wind = w_mean + g;
end
