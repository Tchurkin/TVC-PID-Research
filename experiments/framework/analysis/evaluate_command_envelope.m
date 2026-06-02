function metrics = evaluate_command_envelope(cfg, sc, realism, opts)
%EVALUATE_COMMAND_ENVELOPE Local maneuverability envelope evaluator.

if nargin < 4 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'reference_cmd_deg'); opts.reference_cmd_deg = 20.0; end
if ~isfield(opts, 'step_cmd_start_s'); opts.step_cmd_start_s = 0.50; end
if ~isfield(opts, 'step_t_end'); opts.step_t_end = 3.0; end
if ~isfield(opts, 'settle_band_deg'); opts.settle_band_deg = 2.0; end
if ~isfield(opts, 'min_settle_window_s'); opts.min_settle_window_s = 0.20; end
if ~isfield(opts, 'fail_err_deg'); opts.fail_err_deg = 30.0; end
if ~isfield(opts, 'fail_theta_deg'); opts.fail_theta_deg = 60.0; end
if ~isfield(opts, 'overshoot_floor_deg'); opts.overshoot_floor_deg = 1.0; end
if ~isfield(opts, 'cmd_grid_deg'); opts.cmd_grid_deg = [5 10 15 20 25 30]; end
if ~isfield(opts, 'envelope_success_gate'); opts.envelope_success_gate = 0.50; end
if ~isfield(opts, 'envelope_rms_err_gate_deg'); opts.envelope_rms_err_gate_deg = 15.0; end
if ~isfield(opts, 'envelope_peak_err_gate_deg'); opts.envelope_peak_err_gate_deg = 35.0; end
if ~isfield(opts, 'envelope_cmd_start_s'); opts.envelope_cmd_start_s = 0.55; end
if ~isfield(opts, 'envelope_cmd_ramp_s'); opts.envelope_cmd_ramp_s = 0.75; end
if ~isfield(opts, 'envelope_cmd_hold_s'); opts.envelope_cmd_hold_s = 0.85; end
if ~isfield(opts, 'envelope_t_end')
    opts.envelope_t_end = max(3.0, opts.envelope_cmd_start_s + 2 * opts.envelope_cmd_ramp_s + opts.envelope_cmd_hold_s + 0.50);
end

cmd_success = zeros(numel(opts.cmd_grid_deg), 1);
cmd_rms = zeros(numel(opts.cmd_grid_deg), 1);
cmd_peak = zeros(numel(opts.cmd_grid_deg), 1);
cmd_u_sat = zeros(numel(opts.cmd_grid_deg), 1);
cmd_slew_sat = zeros(numel(opts.cmd_grid_deg), 1);
cmd_bounded_window = zeros(numel(opts.cmd_grid_deg), 1);

for ic = 1:numel(opts.cmd_grid_deg)
    sc_cmd = apply_pitch_program(sc, opts.cmd_grid_deg(ic), opts);
    m = evaluate_maneuver_cell(cfg, sc_cmd, realism, opts);
    cmd_success(ic) = m.success_rate;
    cmd_rms(ic) = m.rms_error_deg;
    cmd_peak(ic) = m.peak_error_deg;
    cmd_u_sat(ic) = m.actuator_sat_frac;
    cmd_slew_sat(ic) = m.slew_sat_frac;
    cmd_bounded_window(ic) = m.bounded_window_s;
end

pass_mask = cmd_success >= opts.envelope_success_gate & ...
    cmd_rms <= opts.envelope_rms_err_gate_deg & ...
    cmd_peak <= opts.envelope_peak_err_gate_deg;

if any(pass_mask)
    envelope_cmd_deg = max(opts.cmd_grid_deg(pass_mask));
else
    envelope_cmd_deg = 0;
end

[~, iref] = min(abs(opts.cmd_grid_deg - opts.reference_cmd_deg));
metrics = struct();
metrics.envelope_cmd_deg = envelope_cmd_deg;
metrics.envelope_cmd_ratio = envelope_cmd_deg / max(opts.cmd_grid_deg);
metrics.bounded_window_s = mean(cmd_bounded_window, 'omitnan');
metrics.bounded_window_norm = metrics.bounded_window_s / max(opts.step_t_end - opts.step_cmd_start_s, 1e-6);
metrics.overshoot_pct = 0;
metrics.actuator_sat_frac = mean(cmd_u_sat, 'omitnan');
metrics.slew_sat_frac = mean(cmd_slew_sat, 'omitnan');
metrics.success_like_rate = mean(cmd_success >= opts.envelope_success_gate, 'omitnan');
metrics.rms_err_deg = mean(cmd_rms, 'omitnan');
metrics.peak_err_deg = mean(cmd_peak, 'omitnan');
metrics.reference_success_rate = cmd_success(iref);
metrics.settled_frac = 0;
metrics.active_bottleneck = "";
metrics.cmd_success_curve = cmd_success;
metrics.cmd_rms_curve = cmd_rms;
metrics.cmd_peak_curve = cmd_peak;
metrics.cmd_grid_deg = opts.cmd_grid_deg;
end


function sc_cmd = apply_pitch_program(sc, cmd_deg, opts)
sc_cmd = sc;
sc_cmd.kind = "PITCH_PROGRAM_SHARED";
sc_cmd.t_end = opts.envelope_t_end;
sc_cmd.theta_ref_fun = @(t) pitch_ref_rad(t, cmd_deg, opts);
sc_cmd.q_ref_fun = @(t) pitch_rate_ref_rad_s(t, cmd_deg, opts);
end


function theta = pitch_ref_rad(t, cmd_deg, opts)
amp = deg2rad(cmd_deg);
t0 = opts.envelope_cmd_start_s;
t1 = t0 + opts.envelope_cmd_ramp_s;
t2 = t1 + opts.envelope_cmd_hold_s;
t3 = t2 + opts.envelope_cmd_ramp_s;
if t < t0
    theta = 0;
elseif t < t1
    theta = amp * (t - t0) / opts.envelope_cmd_ramp_s;
elseif t < t2
    theta = amp;
elseif t < t3
    theta = amp * (1 - (t - t2) / opts.envelope_cmd_ramp_s);
else
    theta = 0;
end
end


function q_ref = pitch_rate_ref_rad_s(t, cmd_deg, opts)
amp = deg2rad(cmd_deg);
t0 = opts.envelope_cmd_start_s;
t1 = t0 + opts.envelope_cmd_ramp_s;
t2 = t1 + opts.envelope_cmd_hold_s;
t3 = t2 + opts.envelope_cmd_ramp_s;
if t >= t0 && t < t1
    q_ref = amp / opts.envelope_cmd_ramp_s;
elseif t >= t2 && t < t3
    q_ref = -amp / opts.envelope_cmd_ramp_s;
else
    q_ref = 0;
end
end


function metrics = evaluate_maneuver_cell(cfg, sc, realism, opts)
n_trials = numel(opts.step_t_end) * 0 + numel(opts.step_cmd_start_s) * 0; %#ok<NASGU>
% Reuse the stability-style loop structure from the existing Exp5 runner.
if ~isfield(opts, 'theta0_deg_set') || isempty(opts.theta0_deg_set)
    opts.theta0_deg_set = [0 3];
end
if ~isfield(opts, 'seeds') || isempty(opts.seeds)
    opts.seeds = 1:3;
end

n_trials = numel(opts.theta0_deg_set) * numel(opts.seeds);
success_col = false(n_trials, 1);
rms_col = zeros(n_trials, 1);
peak_col = zeros(n_trials, 1);
end_col = zeros(n_trials, 1);
max_theta_col = zeros(n_trials, 1);
u_sat_col = zeros(n_trials, 1);
slew_sat_col = zeros(n_trials, 1);
settling_col = zeros(n_trials, 1);
bounded_window_col = zeros(n_trials, 1);

idx = 0;
for it = 1:numel(opts.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(opts.theta0_deg_set(it));
    for iseed = 1:numel(opts.seeds)
        idx = idx + 1;
        seed = opts.seeds(iseed) + 100 * it;
        try
            out = simulate_case_realistic('PID', sc, cfg, seed, realism);
            [success_col(idx), rms_col(idx), peak_col(idx), end_col(idx), max_theta_col(idx), ...
                u_sat_col(idx), slew_sat_col(idx), settling_col(idx), bounded_window_col(idx)] = score_maneuver_run(out, cfg, opts);
        catch
            success_col(idx) = false;
            rms_col(idx) = 90;
            peak_col(idx) = 90;
            end_col(idx) = 90;
            max_theta_col(idx) = 90;
            u_sat_col(idx) = 1;
            slew_sat_col(idx) = 1;
            settling_col(idx) = cfg.t_end_demo;
            bounded_window_col(idx) = 0;
        end
    end
end

metrics = struct();
metrics.success_rate = mean(success_col);
metrics.rms_error_deg = mean(rms_col);
metrics.peak_error_deg = mean(peak_col);
metrics.end_error_deg = mean(end_col);
metrics.max_theta_deg = mean(max_theta_col);
metrics.actuator_sat_frac = mean(u_sat_col);
metrics.slew_sat_frac = mean(slew_sat_col);
metrics.settling_time_s = mean(settling_col);
metrics.bounded_window_s = mean(bounded_window_col);
metrics.bounded_window_norm = metrics.bounded_window_s / max(opts.step_t_end - opts.step_cmd_start_s, 1e-6);
metrics.n_trials = n_trials;
end


function [success, rms_err_deg, peak_err_deg, end_err_deg, max_theta_deg, u_sat, slew_sat, t_settle, bounded_window_s] = score_maneuver_run(out, cfg, opts)
if ~all(isfinite(out.theta)) || ~all(isfinite(out.theta_ref))
    success = false;
    rms_err_deg = 90;
    peak_err_deg = 90;
    end_err_deg = 90;
    max_theta_deg = 90;
    u_sat = 1;
    slew_sat = 1;
    t_settle = cfg.t_end_demo;
    bounded_window_s = 0;
    return;
end

err = out.theta - out.theta_ref;
rms_err_deg = min(90, rad2deg(sqrt(mean(err .^ 2))));
peak_err_deg = min(90, rad2deg(max(abs(err))));
end_err_deg = min(90, rad2deg(abs(err(end))));
max_theta_deg = min(90, rad2deg(max(abs(out.theta))));
u_sat = mean(abs(out.u_cmd) >= 0.99 * cfg.plant.u_max);
slew_sat = mean(abs(diff(out.u_act)) / cfg.dt >= 0.99 * cfg.plant.slew_max);
t_settle = estimate_settling_time_tracking(out.time, err, deg2rad(2.0));
bounded_window_s = t_settle;

success = rms_err_deg <= opts.envelope_rms_err_gate_deg && ...
    peak_err_deg <= opts.envelope_peak_err_gate_deg;
end


function t_settle = estimate_settling_time_tracking(t, err, band_rad)
t_settle = t(end);
inside = abs(err) <= band_rad;
for k = 1:numel(t)
    if all(inside(k:end))
        t_settle = t(k);
        return;
    end
end
end
