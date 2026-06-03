function resultT = verify_physics_wiring()
%VERIFY_PHYSICS_WIRING Verify that sampled physical parameters move runtime dynamics.
%
% Task-1 verifier for the Exp1/Exp4 regime pipeline. The script sweeps the
% five recently wired physical parameters one at a time, evaluates a short
% open-loop response using the actual runtime simulator, exports a CSV plus
% markdown report, and errors if any parameter is inert or has the wrong sign.

rng(1, 'twister');

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
out_dir = fullfile(proj, 'outputs', 'sim_credibility');

addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(here, 'framework', 'plant'));

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

constants = verification_constants();
specs = verification_specs(default_rocket_params());

rows = cell(numel(specs), 17);
failed = strings(0, 1);

for i = 1:numel(specs)
    spec = specs(i);
    responses = zeros(size(spec.values));
    for j = 1:numel(spec.values)
        responses(j) = simulate_probe_response(spec, spec.values(j), constants);
    end

    [monotonic_pass, effect_pass, local_pass, notes] = assess_response(spec, responses, constants);
    overall_pass = monotonic_pass && effect_pass && local_pass;
    if ~overall_pass
        failed(end + 1, 1) = string(spec.parameter); %#ok<AGROW>
    end

    rows(i, :) = { ...
        string(spec.parameter), ...
        string(spec.context), ...
        spec.low_value, ...
        spec.ref_value, ...
        spec.high_value, ...
        responses(1), ...
        responses(2), ...
        responses(3), ...
        responses(3) - responses(1), ...
        min(abs(diff(responses))), ...
        string(spec.expected_direction), ...
        monotonic_pass, ...
        local_pass, ...
        effect_pass, ...
        overall_pass, ...
        string(spec.units), ...
        notes};
end

resultT = cell2table(rows, 'VariableNames', { ...
    'parameter', 'context', 'low_value', 'ref_value', 'high_value', ...
    'low_response', 'ref_response', 'high_response', ...
    'response_span', 'min_local_step', 'expected_direction', ...
    'monotonic_pass', 'local_step_pass', 'effect_size_pass', ...
    'overall_pass', 'response_units', 'notes'});

csv_path = fullfile(out_dir, 'physics_wiring_verification.csv');
report_path = fullfile(out_dir, 'PHYSICS_WIRING_VERIFICATION.md');

writetable(resultT, csv_path);
write_report(report_path, resultT, constants, failed);

fprintf('Saved: %s\n', csv_path);
fprintf('Saved: %s\n', report_path);

if ~isempty(failed)
    fail_list = strjoin(cellstr(failed), ', ');
    error('verify_physics_wiring:VerificationFailed', ...
        'Physics wiring verification failed for: %s. See %s', fail_list, report_path);
end
end


function constants = verification_constants()
constants.seed = 1;
constants.dt_horizon_s = 0.005;
constants.theta_probe_rad = 1.0;
constants.u_act_probe = 1.0;
constants.disturbance_probe = 1.0;
constants.tau_hold_s = 1e9;
constants.slew_hold = 1e9;
constants.abs_noise_floor = 1e-6;
constants.min_local_step = 1e-4;
constants.min_relative_step = 0.02;
constants.min_relative_span = 0.05;
constants.override_p_unstable = 1.0;
end


function specs = verification_specs(P)
specs = [ ...
    make_spec( ...
    'mass', 'disturbance_response_qdot', ...
    [0.70, 1.00, 1.30] * P.rocket.mass, ...
    'descending', 'rad/s^2'); ...
    make_spec( ...
    'Iyy', 'control_authority_qdot', ...
    [0.70, 1.00, 1.30] * P.rocket.Iyy, ...
    'descending', 'rad/s^2'); ...
    make_spec( ...
    'static_margin', 'geometry_qdot', ...
    [0.50, 1.00, 1.50] * P.rocket.static_margin, ...
    'ascending', 'rad/s^2'); ...
    make_spec( ...
    'Cm_alpha', 'geometry_qdot', ...
    [-1.50, 1.00, 0.50] * abs(P.rocket.Cm_alpha), ...
    'descending', 'rad/s^2'); ...
    make_spec( ...
    'thrust', 'control_authority_qdot', ...
    [0.70, 1.00, 1.30] * P.rocket.thrust, ...
    'ascending', 'rad/s^2')];

    % Convert the Cm_alpha sweep back to signed values after scaling magnitude.
specs(4).values = -abs(specs(4).values);
specs(4).low_value = specs(4).values(1);
specs(4).ref_value = specs(4).values(2);
specs(4).high_value = specs(4).values(3);
end


function spec = make_spec(parameter, context, values, expected_direction, units)
spec = struct();
spec.parameter = parameter;
spec.context = context;
spec.values = values(:)';
spec.low_value = spec.values(1);
spec.ref_value = spec.values(2);
spec.high_value = spec.values(3);
spec.expected_direction = expected_direction;
spec.units = units;
end


function response = simulate_probe_response(spec, value, constants)
P = default_rocket_params();
P.rocket.(spec.parameter) = value;

override = struct();
% Geometry parameters (static_margin, Cm_alpha) must drive the unstable pole
% through the real derivation path, so do NOT pin p_unstable for that context.
% mass/Iyy/thrust use a fixed p_unstable to isolate their scale channels.
if ~strcmp(spec.context, 'geometry_qdot')
    override.p_unstable = constants.override_p_unstable;
end
override.mass = P.rocket.mass;
override.Iyy = P.rocket.Iyy;
override.static_margin = P.rocket.static_margin;
override.Cm_alpha = P.rocket.Cm_alpha;
override.thrust = P.rocket.thrust;
override.t_end = constants.dt_horizon_s;

[cfg, sc, realism] = build_realistic_cfg(P, override);

cfg.controllers.PID.Kp = 0.0;
cfg.controllers.PID.Kd = 0.0;
cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;
cfg.plant.tau_act = constants.tau_hold_s;
cfg.plant.slew_max = constants.slew_hold;

realism.gyro_noise_std = 0.0;
realism.gyro_bias_init = 0.0;
realism.gyro_bias_rw = 0.0;
realism.gyro_quant_lsb = 0.0;
realism.bias_cal_s = 0.0;
realism.bias_cal_residual_frac = 0.0;
realism.theta_init_bias = 0.0;
realism.servo_deadband = 0.0;
realism.servo_backlash = 0.0;
realism.servo_pos_noise = 0.0;
realism.gust_std = 0.0;
realism.keff_drift_rate = 0.0;
realism.sensor_latency_steps = 1;

sc.startup_ramp_s = 0.0;
sc.fault_time = inf;
sc.disturbance_amp = 0.0;
sc.disturbance_freq_hz = 0.0;
sc.disturb_scale_post = 1.0;
sc.disturb_bias_post = 0.0;
sc.control_eff_scale_post = 1.0;
sc.aero_damp_scale_post = 1.0;
sc.tau_scale_post = 1.0;
sc.slew_scale_post = 1.0;

switch spec.context
    case 'control_authority_qdot'
        cfg.plant.theta0 = 0.0;
        cfg.plant.q0 = 0.0;
        cfg.plant.u_act0 = constants.u_act_probe;

    case 'disturbance_response_qdot'
        cfg.plant.theta0 = 0.0;
        cfg.plant.q0 = 0.0;
        cfg.plant.u_act0 = 0.0;
        sc.fault_time = 0.0;
        sc.disturb_bias_post = constants.disturbance_probe;

    case 'geometry_qdot'
        cfg.plant.theta0 = constants.theta_probe_rad;
        cfg.plant.q0 = 0.0;
        cfg.plant.u_act0 = 0.0;

    otherwise
        error('verify_physics_wiring:UnknownContext', 'Unknown context: %s', spec.context);
end

out = simulate_case_realistic('PID', sc, cfg, constants.seed, realism);
response = (out.q(2) - out.q(1)) / cfg.dt;
end


function [monotonic_pass, effect_pass, local_pass, notes] = assess_response(spec, responses, constants)
diffs = diff(responses);
ref_mag = max(abs(responses(2)), constants.abs_noise_floor);
local_floor = max(constants.min_local_step, constants.min_relative_step * ref_mag);
span_floor = max(constants.min_local_step, constants.min_relative_span * ref_mag);

switch spec.expected_direction
    case 'ascending'
        monotonic_pass = all(diffs > constants.abs_noise_floor);
    case 'descending'
        monotonic_pass = all(diffs < -constants.abs_noise_floor);
    otherwise
        error('verify_physics_wiring:UnknownDirection', 'Unknown direction: %s', spec.expected_direction);
end

local_pass = all(abs(diffs) >= local_floor);
effect_pass = abs(responses(3) - responses(1)) >= span_floor;

if monotonic_pass && local_pass && effect_pass
        notes = "PASS";
        return;
    end

parts = strings(0, 1);
if ~monotonic_pass
    parts(end + 1, 1) = "wrong sign or non-monotonic"; %#ok<AGROW>
end
if ~local_pass
    parts(end + 1, 1) = sprintf('local step below floor %.3e', local_floor); %#ok<AGROW>
end
if ~effect_pass
    parts(end + 1, 1) = sprintf('span below floor %.3e', span_floor); %#ok<AGROW>
end
notes = strjoin(cellstr(parts), '; ');
end


function write_report(report_path, resultT, constants, failed)
fid = fopen(report_path, 'w');
if fid < 0
    error('verify_physics_wiring:ReportOpenFailed', 'Could not open %s for writing.', report_path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# PHYSICS_WIRING_VERIFICATION\n\n');
if isempty(failed)
    fprintf(fid, '- Status: PASS\n');
    fprintf(fid, '- Interpretation: all five physical parameters moved the runtime derivative with the expected sign and a non-negligible magnitude.\n\n');
else
    fprintf(fid, '- Status: FAIL\n');
    fprintf(fid, '- Interpretation: at least one sampled physical parameter remains inert or asymmetric in the actual runtime path. Do not use Tasks 2-5 as paper evidence until this is resolved.\n\n');
    fprintf(fid, '- Failed parameters: %s\n\n', strjoin(cellstr(failed), ', '));
end

fprintf(fid, '## Verification Protocol\n');
fprintf(fid, '- Simulator: `simulate_case_realistic` with deterministic seed %d.\n', constants.seed);
fprintf(fid, '- Probe horizon: one runtime step (`dt = %.4f s`).\n', constants.dt_horizon_s);
fprintf(fid, '- Controller: zero-gain PID to isolate the plant derivative.\n');
fprintf(fid, '- Acceptance thresholds are defined once in `verification_constants` inside `experiments/verify_physics_wiring.m`.\n\n');

fprintf(fid, '## Results\n\n');
fprintf(fid, '| parameter | context | expected | low | ref | high | low resp | ref resp | high resp | overall | notes |\n');
fprintf(fid, '|---|---|---|---:|---:|---:|---:|---:|---:|---|---|\n');
for i = 1:height(resultT)
    fprintf(fid, '| %s | %s | %s | %.6g | %.6g | %.6g | %.6g | %.6g | %.6g | %s | %s |\n', ...
        resultT.parameter{i}, resultT.context{i}, resultT.expected_direction{i}, ...
        resultT.low_value(i), resultT.ref_value(i), resultT.high_value(i), ...
        resultT.low_response(i), resultT.ref_response(i), resultT.high_response(i), ...
        pass_fail(resultT.overall_pass(i)), escape_pipes(resultT.notes{i}));
end
fprintf(fid, '\n');

if any(strcmp(resultT.parameter, 'static_margin')) || any(strcmp(resultT.parameter, 'Cm_alpha'))
    fprintf(fid, '## Geometry Path Finding\n');
    fprintf(fid, '- `static_margin` and `Cm_alpha` are swept WITHOUT pinning `p_unstable`, so they drive the unstable pole through the real derivation path used by Exp1 (`p_unstable = estimate_p_unstable(static_margin, Cm_alpha, thrust)`).\n');
    fprintf(fid, '- The redundant `p_geom`/`p_geom_delta` channel (formerly computed by the identical formula as `p_unstable`, double-counting geometry) has been removed, so geometry now enters the derivative exactly once.\n\n');
end
end


function out = pass_fail(tf)
if tf
    out = 'PASS';
else
    out = 'FAIL';
end
end


function s = escape_pipes(s)
s = strrep(char(s), '|', '\\|');
end