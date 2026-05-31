function out = process_gimbal_bench_serial(csv_path, opts)
%PROCESS_GIMBAL_BENCH_SERIAL  Ingest Serial-Plotter CSV produced by
%Firmware/feedback_servo_calibration.ino (BINARY_STEP_MODE = true) and
%feed it directly to experiments/validator/recommend_envelope.m.
%
% This complements the existing tools/process_gimbal_bench_test.m which
% expects the (time_s, cmd_deg, meas_deg) sysid template; this one
% expects the wider Plotter-mode header:
%
%   commanded_deg, actual_deg_est, slew_deg_per_s, slew_raw_counts_per_s,
%   transition_avg_slew_deg_s, transition_net_slew_deg_s,
%   rolling_up_net_slew_deg_s, rolling_down_net_slew_deg_s,
%   endpoint_util_pct[, fb_x_raw]
%
% Usage:
%   out = process_gimbal_bench_serial('data/bench/calib_run1.csv');
%   out = process_gimbal_bench_serial('...', struct('p_est', 8.0));
%
% Returns:
%   out.measured       summary of measured bench metrics
%   out.recommendation result from recommend_envelope

if nargin < 2, opts = struct(); end
opts = apply_defaults(opts, default_opts());

this = fileparts(mfilename('fullpath'));
proj = fileparts(this);
addpath(fullfile(proj, 'experiments'));
addpath(fullfile(proj, 'experiments', 'validator'));

fprintf('Reading %s\n', csv_path);
T = readtable(csv_path);
need = {'rolling_up_net_slew_deg_s', 'rolling_down_net_slew_deg_s', ...
        'endpoint_util_pct'};
for k = 1:numel(need)
    if ~ismember(need{k}, T.Properties.VariableNames)
        error('process_gimbal_bench_serial:missing_column', ...
            'Required column "%s" not found in %s. Flash the .ino in BINARY_STEP_MODE.', ...
            need{k}, csv_path);
    end
end

up_servo   = T.rolling_up_net_slew_deg_s;
down_servo = T.rolling_down_net_slew_deg_s;
util_pct   = T.endpoint_util_pct;

warm = up_servo > 0 & down_servo > 0;
if nnz(warm) < 5
    warning('Very few warm rows (%d). Bench data may be too short.', nnz(warm));
end
up_servo   = up_servo(warm);
down_servo = down_servo(warm);
util_pct   = util_pct(util_pct > 0);

N = numel(up_servo);
if opts.trim_first_n_transitions > 0 && N > opts.trim_first_n_transitions + 2
    skip = min(opts.trim_first_n_transitions, N - 3);
    up_servo = up_servo(skip+1:end);
    down_servo = down_servo(skip+1:end);
end

slew_up_servo_ds   = median(up_servo);
slew_down_servo_ds = median(down_servo);
slew_servo_ds      = min(slew_up_servo_ds, slew_down_servo_ds);   % conservative
util_avg           = mean(util_pct);

gimbal_slew_ds   = slew_servo_ds / opts.linkage_servo_per_gimbal;
gimbal_slew_up   = slew_up_servo_ds / opts.linkage_servo_per_gimbal;
gimbal_slew_down = slew_down_servo_ds / opts.linkage_servo_per_gimbal;

fprintf('\n--- Bench measurements (medians over %d warm transitions) ---\n', N);
fprintf('  servo slew (up/down) : %.2f / %.2f deg/s\n', slew_up_servo_ds, slew_down_servo_ds);
fprintf('  gimbal slew (up/down): %.3f / %.3f deg/s   [linkage %g:1]\n', ...
    gimbal_slew_up, gimbal_slew_down, opts.linkage_servo_per_gimbal);
fprintf('  endpoint utilization : %.1f %% (avg)\n', util_avg);
if util_avg < 60
    fprintf('  WARNING: low utilization (%.0f%%) suggests bench sweep span is\n', util_avg);
    fprintf('           limiting motion before the servo reaches its true slew.\n');
    fprintf('           Increase GIMBAL_SWEEP_HALF_SPAN_DEG in the .ino and re-run.\n');
end

meas = struct();
meas.slew_deg_per_s = gimbal_slew_ds;
meas.servo_max_deg  = opts.servo_max_deg;
meas.p_est          = opts.p_est;
meas.keff_est       = opts.keff_est;
meas.damp_est       = opts.damp_est;

fprintf('\n--- Pre-flight validator ---\n');
rec = recommend_envelope(meas);

out.measured.servo_slew_up_ds   = slew_up_servo_ds;
out.measured.servo_slew_down_ds = slew_down_servo_ds;
out.measured.gimbal_slew_ds     = gimbal_slew_ds;
out.measured.endpoint_util_pct  = util_avg;
out.measured.n_warm_transitions = N;
out.recommendation = rec;
end

function d = default_opts()
% Defaults match Firmware/feedback_servo_calibration.ino constants:
%   LINKAGE_SERVO_DEG_PER_GIMBAL_DEG = 4.0
%   GIMBAL_SWEEP_HALF_SPAN_DEG       = 5.0
d.linkage_servo_per_gimbal = 4.0;
d.servo_max_deg            = 5.0;
d.p_est                    = 8.0;
d.keff_est                 = 8.0;
d.damp_est                 = 0.5;
d.trim_first_n_transitions = 2;
end

function s = apply_defaults(s, d)
fn = fieldnames(d);
for k = 1:numel(fn)
    if ~isfield(s, fn{k}) || isempty(s.(fn{k}))
        s.(fn{k}) = d.(fn{k});
    end
end
end
