function out = bench_to_autotune(csv_path, opts)
%BENCH_TO_AUTOTUNE  Bench CSV -> preflight verdict + tuned PD gains.
%
% This is the new primary workflow entry point for the project:
%   1. ingest measured actuator behavior from bench data,
%   2. convert it into a validator-style plant envelope,
%   3. tune PD gains on the realistic plant using that measured envelope,
%   4. report GO/MARGINAL/NOGO plus recommended gains.
%
% Required input:
%   csv_path  path to bench CSV compatible with bench_to_validator
%
% opts fields (all optional):
%   source                     'ino' | 'template' (auto if omitted)
%   linkage_servo_per_gimbal   default 4.0
%   servo_max_servo_deg        default 40.0 (=> 10 deg gimbal at 4:1)
%   p_est                      default 6.0   unstable pole estimate (1/s)
%   keff_est                   default 8.0
%   damp_est                   default 0.5
%   theta0_deg_set             default [3 6]
%   seeds                      default 1:4
%   t_end                      default 3.0
%   disturb_amp                default 0.10
%   disturb_freq_hz            default 0.80
%   Kp_grid                    default [1 3 10 30 100]
%   Kd_grid                    default [0.2 1 3 10 30]
%   success_rms_deg            default 10
%   success_peak_deg           default 40
%   success_end_deg            default 8
%   go_success                 default 0.80
%   marginal_success           default 0.40
%   compare_lqr                default true
%   endpoint_util_frac_override optional scalar in (0,1]
%   datasheet_slew_deg_per_s   optional for reality-gap reporting
%   realism                    optional struct passed to simulate_case_realistic
%
% Returns:
%   out.bench                 output of bench_to_validator
%   out.config                config used for tuning
%   out.autotune              tuned PD recommendation + predicted success
%   out.lqr_baseline          validator-LQR comparison on same measured plant

if nargin < 2, opts = struct(); end
opts = apply_defaults(opts, default_opts());

this = fileparts(mfilename('fullpath'));
proj = fileparts(this);
addpath(fullfile(proj, 'experiments'));
addpath(fullfile(proj, 'experiments', 'validator'));
addpath(genpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src')));

bench_opts = struct();
if isfield(opts, 'source') && ~isempty(opts.source)
    bench_opts.source = opts.source;
end
bench_opts.linkage_servo_per_gimbal = opts.linkage_servo_per_gimbal;
bench_opts.servo_max_servo_deg = opts.servo_max_servo_deg;
bench_opts.p_est_1_s = opts.p_est;
bench_opts.keff_est = opts.keff_est;
bench_opts.damp_est = opts.damp_est;
bench_opts.call_validator = true;

bench = bench_to_validator(csv_path, bench_opts);

if ~isfinite(bench.slew_deg_per_s) || bench.slew_deg_per_s <= 0
    error('bench_to_autotune:invalidBenchSlew', ...
        'Bench slew could not be estimated from %s. Re-run the bench sweep or inspect the CSV.', ...
        csv_path);
end

util_frac = opts.endpoint_util_frac_override;
if isempty(util_frac)
    util_frac = bench.endpoint_util_pct / 100.0;
end
if ~isfinite(util_frac)
    util_frac = 1.0;
end
util_frac = min(1.0, max(0.10, util_frac));

gimbal_max_rad = deg2rad(bench.servo_max_deg);
code_scale = 12.0 / max(1e-3, gimbal_max_rad);
effective_u_max_deg = bench.servo_max_deg * util_frac;
effective_u_max_code = 12.0 * util_frac;
measured_slew_code = deg2rad(bench.slew_deg_per_s) * code_scale;

cfg = rocket_defaults();
cfg.t_end_demo = opts.t_end;
cfg.plant.theta0 = 0;
cfg.plant.aero_damp = opts.damp_est;
cfg.plant.control_eff = opts.keff_est;
cfg.plant.keff_nom = opts.keff_est;
cfg.plant.p_unstable = opts.p_est;
cfg.plant.slew_max = measured_slew_code;
cfg.plant.u_max = effective_u_max_code;

cfg.controllers.FIXED_LQR.u_max = cfg.plant.u_max;
cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;

realism = opts.realism;

best_rate = -inf;
best_rms = inf;
best_peak = inf;
best_Kp = NaN;
best_Kd = NaN;

for Kp = opts.Kp_grid
    for Kd = opts.Kd_grid
        cfg.controllers.PID.Kp = Kp;
        cfg.controllers.PID.Kd = Kd;
        [rate, mean_rms_deg, mean_peak_deg] = evaluate_controller('PID', cfg, opts, realism);
        if rate > best_rate || (abs(rate - best_rate) < 1e-9 && mean_rms_deg < best_rms)
            best_rate = rate;
            best_rms = mean_rms_deg;
            best_peak = mean_peak_deg;
            best_Kp = Kp;
            best_Kd = Kd;
        end
    end
end

if best_rate >= opts.go_success
    final_verdict = "GO";
elseif best_rate >= opts.marginal_success
    final_verdict = "MARGINAL";
else
    final_verdict = "NOGO";
end

lqr_rate = NaN;
lqr_rms = NaN;
lqr_peak = NaN;
if opts.compare_lqr
    lqr_cfg = cfg;
    rec = bench.validator;
    lqr_u_max_code = deg2rad(rec.u_max_recommended_deg) * code_scale;
    lqr_cfg.plant.u_max = min(cfg.plant.u_max, lqr_u_max_code);
    lqr_cfg.controllers.FIXED_LQR.u_max = lqr_cfg.plant.u_max;
    lqr_cfg.controllers.FIXED_LQR.K = rec.K;
    if lqr_cfg.plant.u_max > 0.05
        [lqr_rate, lqr_rms, lqr_peak] = evaluate_controller('FIXED_LQR', lqr_cfg, opts, realism);
    else
        lqr_rate = 0.0;
        lqr_rms = 90.0;
        lqr_peak = 90.0;
    end
end

reality_gap = struct();
if isfield(opts, 'datasheet_slew_deg_per_s') && ~isempty(opts.datasheet_slew_deg_per_s)
    reality_gap.datasheet_slew_deg_per_s = opts.datasheet_slew_deg_per_s;
    reality_gap.measured_slew_deg_per_s = bench.slew_deg_per_s;
    reality_gap.slew_ratio_measured_to_datasheet = bench.slew_deg_per_s / max(1e-6, opts.datasheet_slew_deg_per_s);
else
    reality_gap = struct([]);
end

out.bench = bench;
out.config = struct( ...
    'p_est', opts.p_est, ...
    'keff_est', opts.keff_est, ...
    'damp_est', opts.damp_est, ...
    'effective_u_max_deg', effective_u_max_deg, ...
    'effective_u_max_code', effective_u_max_code, ...
    'measured_slew_code', measured_slew_code, ...
    'code_scale', code_scale, ...
    'theta0_deg_set', opts.theta0_deg_set, ...
    'seeds', opts.seeds);
out.autotune = struct( ...
    'controller', 'PD', ...
    'best_Kp', best_Kp, ...
    'best_Kd', best_Kd, ...
    'predicted_success', best_rate, ...
    'predicted_rms_deg', best_rms, ...
    'predicted_peak_deg', best_peak, ...
    'verdict', final_verdict);
out.lqr_baseline = struct( ...
    'predicted_success', lqr_rate, ...
    'predicted_rms_deg', lqr_rms, ...
    'predicted_peak_deg', lqr_peak, ...
    'validator_verdict', string(bench.validator.go_nogo));
if ~isempty(reality_gap)
    out.reality_gap = reality_gap;
end

fprintf('\n=== Bench -> Autotune Summary ===\n');
fprintf('  measured slew          : %.2f deg/s gimbal\n', bench.slew_deg_per_s);
fprintf('  effective u_max        : %.2f deg gimbal (%.0f%% util)\n', effective_u_max_deg, 100 * util_frac);
fprintf('  p_est                  : %.2f 1/s\n', opts.p_est);
fprintf('  best PD                : Kp=%.2f  Kd=%.2f\n', best_Kp, best_Kd);
fprintf('  PD predicted success   : %.2f  (%s)\n', best_rate, final_verdict);
if opts.compare_lqr
    fprintf('  validator LQR success  : %.2f  (%s)\n', lqr_rate, bench.validator.go_nogo);
end
if isfield(out, 'reality_gap')
    fprintf('  datasheet->measured slew ratio : %.2f\n', out.reality_gap.slew_ratio_measured_to_datasheet);
end
end


function [rate, mean_rms_deg, mean_peak_deg] = evaluate_controller(ctrl_name, cfg, opts, realism)
sc = rocket_scenario('nominal', cfg);
sc.t_end = opts.t_end;
sc.disturbance_amp = opts.disturb_amp;
sc.disturbance_freq_hz = opts.disturb_freq_hz;

wins = 0;
n_trials = 0;
rms_vals = zeros(numel(opts.seeds) * numel(opts.theta0_deg_set), 1);
peak_vals = zeros(numel(opts.seeds) * numel(opts.theta0_deg_set), 1);
idx = 0;

for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1;
        n_trials = n_trials + 1;
        try
            out = simulate_case_realistic(ctrl_name, sc, cfg, opts.seeds(iseed) + 100 * it, realism);
            theta_rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
            theta_peak_deg = rad2deg(max(abs(out.theta)));
            theta_end_deg = rad2deg(abs(out.theta(end)));
            if ~all(isfinite(out.theta))
                theta_rms_deg = 90;
                theta_peak_deg = 90;
                theta_end_deg = 90;
            else
                theta_rms_deg = min(theta_rms_deg, 90);
                theta_peak_deg = min(theta_peak_deg, 90);
                theta_end_deg = min(theta_end_deg, 90);
            end
            rms_vals(idx) = theta_rms_deg;
            peak_vals(idx) = theta_peak_deg;
            stable = all(isfinite(out.theta)) && ...
                     theta_rms_deg <= opts.success_rms_deg && ...
                     theta_peak_deg <= opts.success_peak_deg && ...
                     theta_end_deg <= opts.success_end_deg;
            wins = wins + double(stable);
        catch
            rms_vals(idx) = 90;
            peak_vals(idx) = 90;
        end
    end
end

rate = wins / n_trials;
mean_rms_deg = mean(rms_vals);
mean_peak_deg = mean(peak_vals);
end


function opts = apply_defaults(opts, defs)
fns = fieldnames(defs);
for i = 1:numel(fns)
    if ~isfield(opts, fns{i})
        opts.(fns{i}) = defs.(fns{i});
    end
end
end


function defs = default_opts()
defs.source = '';
defs.linkage_servo_per_gimbal = 4.0;
defs.servo_max_servo_deg = 40.0;
defs.p_est = 6.0;
defs.keff_est = 8.0;
defs.damp_est = 0.5;
defs.theta0_deg_set = [3 6];
defs.seeds = 1:4;
defs.t_end = 3.0;
defs.disturb_amp = 0.10;
defs.disturb_freq_hz = 0.80;
defs.Kp_grid = [1 3 10 30 100];
defs.Kd_grid = [0.2 1 3 10 30];
defs.success_rms_deg = 10.0;
defs.success_peak_deg = 40.0;
defs.success_end_deg = 8.0;
defs.go_success = 0.80;
defs.marginal_success = 0.40;
defs.compare_lqr = true;
defs.endpoint_util_frac_override = [];
defs.datasheet_slew_deg_per_s = [];
defs.realism = struct();
end