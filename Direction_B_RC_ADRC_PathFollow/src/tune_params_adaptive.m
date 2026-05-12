%% tune_params_adaptive.m
% Grid search for adaptive ADRC parameters.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
out_dir = fullfile(dir_root, 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
addpath(this_dir);

fprintf('Running adaptive parameter tuning...\n');

lambda_grid = [0.96, 0.98, 0.995];
drift_grid = [0.10, 0.15, 0.20];
rate_grid = [0.5, 1.0, 2.0];
prbs_amp_grid = [0.15, 0.30];
prbs_dur_grid = [2.0, 3.0];

best = struct('score', inf);
case_id = 0;
total = numel(lambda_grid) * numel(drift_grid) * numel(rate_grid) * numel(prbs_amp_grid) * numel(prbs_dur_grid);

for lambda_rls = lambda_grid
for drift_threshold = drift_grid
for omega_o_rate = rate_grid
for prbs_amp = prbs_amp_grid
for prbs_duration = prbs_dur_grid
    case_id = case_id + 1;

    p = struct();
    p.lambda_rls = lambda_rls;
    p.drift_threshold = drift_threshold;
    p.omega_o_rate = omega_o_rate;
    p.prbs_amp = prbs_amp;
    p.prbs_duration = prbs_duration;

    [score, met] = evaluate_candidate(p);

    if score < best.score
        best = p;
        best.score = score;
        best.metrics = met;
        fprintf('  New best [%d/%d] score=%.4f lambda=%.3f drift=%.2f rate=%.2f prbs=%.2f/%.1fs\n', ...
            case_id, total, score, lambda_rls, drift_threshold, omega_o_rate, prbs_amp, prbs_duration);
    end
end
end
end
end
end

save(fullfile(out_dir, 'tuned_params_adaptive.mat'), 'best');
fid = fopen(fullfile(out_dir, 'tuned_params_adaptive.csv'), 'w');
fprintf(fid, 'param,value\n');
fprintf(fid, 'score,%.6f\n', best.score);
fprintf(fid, 'lambda_rls,%.6f\n', best.lambda_rls);
fprintf(fid, 'drift_threshold,%.6f\n', best.drift_threshold);
fprintf(fid, 'omega_o_rate,%.6f\n', best.omega_o_rate);
fprintf(fid, 'prbs_amp,%.6f\n', best.prbs_amp);
fprintf(fid, 'prbs_duration,%.6f\n', best.prbs_duration);
fprintf(fid, 'recovery_gain,%.6f\n', best.metrics.recovery_gain);
fprintf(fid, 'recovery_time_gain,%.6f\n', best.metrics.recovery_time_gain);
fprintf(fid, 'nominal_score,%.6f\n', best.metrics.nominal_score);
fprintf(fid, 'sysid_score,%.6f\n', best.metrics.sysid_score);
fclose(fid);

fprintf('Adaptive tuning complete. Best score=%.4f\n', best.score);

function [score, met] = evaluate_candidate(p)
SEEDS = 3;
dt = 0.01;
N = round(60 / dt);
wind_mean = 10;

l1.L1_period = 20.0;
l1.u_max = 4.0;

adrc.b0 = 1.0;
omega_c = 0.80;
omega_o = 3.20;
adrc.Kp = omega_c^2;
adrc.Kd = 2*omega_c;
adrc.l1 = 3*omega_o;
adrc.l2 = 3*omega_o^2;
adrc.l3 = omega_o^3;
adrc.u_max = l1.u_max;

cfg.k_blend = 0.90;
cfg.du_max = 35.0;
cfg.k_blend_fault = 0.90;
cfg.du_max_fault = 35.0;
cfg.lambda_rls = p.lambda_rls;
cfg.drift_threshold = p.drift_threshold;
cfg.omega_o_rate = p.omega_o_rate;
cfg.tau_exp = 0.55;
cfg.gain_exp = 0.35;
cfg.bw_floor = 0.35;
cfg.prbs_amp = p.prbs_amp;
cfg.prbs_duration = p.prbs_duration;
cfg.adapt_guard_s = 0.5;
cfg.degrade_time = 20.0;
cfg.degrade_tau_scale = 3.0;
cfg.degrade_gain_scale = 0.45;
cfg.degrade_rate_scale = 0.35;

rms_fix_deg = zeros(SEEDS,1);
rms_ad_deg = zeros(SEEDS,1);
rec_ad = zeros(SEEDS,1);
rms_fix_nom = zeros(SEEDS,1);
rms_ad_nom = zeros(SEEDS,1);
id_gain_err = zeros(SEEDS,1);
id_tau_err = zeros(SEEDS,1);

for si = 1:SEEDS
    seed = 8000 + si;
    [w_wind, tau_act, u_rate, V] = build_wind_case(wind_mean, N, dt, seed);

    m_fix_deg = run_adrc_case(false, true, w_wind, tau_act, u_rate, V, dt, l1, adrc, cfg, seed);
    m_ad_deg = run_adrc_case(true, true, w_wind, tau_act, u_rate, V, dt, l1, adrc, cfg, seed);
    m_fix_nom = run_adrc_case(false, false, w_wind, tau_act, u_rate, V, dt, l1, adrc, cfg, seed);
    m_ad_nom = run_adrc_case(true, false, w_wind, tau_act, u_rate, V, dt, l1, adrc, cfg, seed);

    rms_fix_deg(si) = m_fix_deg.post_fault_rms;
    rms_ad_deg(si) = m_ad_deg.post_fault_rms;
    rec_ad(si) = m_ad_deg.recovery_time_s;
    rms_fix_nom(si) = m_fix_nom.rms_ct;
    rms_ad_nom(si) = m_ad_nom.rms_ct;
    id_gain_err(si) = m_ad_deg.sysid_gain_err;
    id_tau_err(si) = m_ad_deg.sysid_tau_err;
end

recovery_gain = max(0, min(1, (mean(rms_fix_deg) - mean(rms_ad_deg)) / max(1e-6, mean(rms_fix_deg))));
recovery_time_score = max(0, min(1, 1 - mean(rec_ad) / 10.0));
nominal_score = max(0, min(1, 1 - abs(mean(rms_ad_nom) - mean(rms_fix_nom)) / max(0.5, mean(rms_fix_nom))));
sysid_score = max(0, min(1, 1 - (mean(id_gain_err) + mean(id_tau_err) / 0.25)));

met = struct();
met.recovery_gain = recovery_gain;
met.recovery_time_gain = recovery_time_score;
met.nominal_score = nominal_score;
met.sysid_score = sysid_score;

score = 1 - (0.5 * (0.6 * recovery_gain + 0.4 * recovery_time_score) + 0.3 * nominal_score + 0.2 * sysid_score);
end

function met = run_adrc_case(use_adaptive, use_degrade, w_wind, tau_act_nom, u_act_rate_nom, V, dt, l1, adrc, cfg, seed)
N = numel(w_wind);
y = 0; vy = 0; u_act = 0;
z = zeros(3,1);
u_prev = 0; u_cmd_prev = 0;

tau_nom_true = tau_act_nom;
gain_nom_true = 1.0;
if use_degrade
    rng(seed + 12345);
    tau_nom_true = tau_act_nom * (1.0 + 0.30 * randn());
    tau_nom_true = max(0.12, min(0.70, tau_nom_true));
    gain_nom_true = 1.0 + 0.20 * randn();
    gain_nom_true = max(0.60, min(1.35, gain_nom_true));
end

adp = struct('t', 0, 'omega_o_eff', adrc.l1 / 3, 'gain_est', 1.0, 'tau_est', tau_act_nom, ...
    'has_prev', false, 'rls', struct('theta', [exp(-dt/max(1e-3, tau_nom_true)); 1-exp(-dt/max(1e-3, tau_nom_true))], 'P', 50*eye(2)), ...
    'u_act_prev', 0, 'u_cmd_prev', 0);
adp.tau_est = tau_nom_true;

sysid = struct('gain_est', 1.0, 'tau_est', tau_nom_true);
if use_adaptive
    sysid = run_preflight_sysid(dt, l1.u_max, cfg.prbs_amp, cfg.prbs_duration, tau_nom_true, gain_nom_true, u_act_rate_nom, seed);
    adp.gain_est = sysid.gain_est;
    adp.tau_est = sysid.tau_est;
end

apar = struct('b0_nominal', adrc.b0, 'gain_nom', max(0.3, sysid.gain_est), 'tau_nom', max(0.05, sysid.tau_est), ...
    'omega_o_nom', adrc.l1 / 3, 'Kp', adrc.Kp, 'Kd', adrc.Kd, 'u_max', adrc.u_max, ...
    'lambda_rls', cfg.lambda_rls, 'omega_o_rate', cfg.omega_o_rate, 'adapt_guard_s', cfg.adapt_guard_s, ...
    'tau_exp', cfg.tau_exp, 'gain_exp', cfg.gain_exp, 'bw_floor', cfg.bw_floor);

y_log = zeros(N,1);
u_cmd_log = zeros(N,1);
u_act_log = zeros(N,1);

for k = 1:N
    t = (k-1) * dt;

    [u_l1, ~] = l1_guidance(y, vy, V, l1);
    if use_adaptive
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
    else
        [u_ad, z] = adrc_layer(y, z, u_prev, adrc, dt);
        k_blend_eff = cfg.k_blend;
        du_max_eff = cfg.du_max;
    end

    u_mix = (1 - k_blend_eff) * u_l1 + k_blend_eff * u_ad;
    du_lim = du_max_eff * dt;
    u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
    u_cmd = max(-adrc.u_max, min(adrc.u_max, u_cmd));

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

    y_log(k) = y;
    u_cmd_log(k) = u_cmd;
    u_act_log(k) = u_act;
end

k0 = round(5 / dt);
y_seg = y_log(k0:end);
met.rms_ct = sqrt(mean(y_seg.^2));
met.sysid_gain_err = abs(sysid.gain_est - gain_nom_true);
met.sysid_tau_err = abs(sysid.tau_est - tau_nom_true);
if use_degrade
    kf = max(1, round(cfg.degrade_time / dt));
    kf2 = min(N, kf + round(8.0 / dt));
    y_post = y_log(kf:kf2);
    met.post_fault_rms = sqrt(mean(y_post.^2));
    met.recovery_time_s = recovery_time((0:N-1)' * dt, y_log, cfg.degrade_time, 5.0, 2.0);
else
    met.post_fault_rms = NaN;
    met.recovery_time_s = NaN;
end
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
t_rec = 20;
for k = idx0:(numel(y)-Nhold)
    if all(abs(y(k:k+Nhold-1)) <= band)
        t_rec = t(k) - t_fault;
        return;
    end
end
end

function [w_wind, tau_act, u_act_rate_max, V] = build_wind_case(w_mean, N, dt, seed)
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
