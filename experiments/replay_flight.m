function out = replay_flight(flightLogPath, varargin)
%REPLAY_FLIGHT Replay a flight log through the simulator for digital-twin checks.
%
% Usage:
%   out = replay_flight(logPath)
%   out = replay_flight(logPath, 'paramEstimate', est, 'seed', 1)
%
% Required log channels: time, theta, q, u_cmd, u_act (name aliases supported).

p = inputParser;
addRequired(p, 'flightLogPath', @ischar);
addParameter(p, 'paramEstimate', struct(), @isstruct);
addParameter(p, 'seed', 1, @isnumeric);
addParameter(p, 'resultsDir', '', @ischar);
parse(p, flightLogPath, varargin{:});

logT = readtable(flightLogPath);
[t, theta_log, q_log, u_cmd_log, u_act_log, theta_ref_log] = normalize_log_channels(logT);

est = p.Results.paramEstimate;
if isempty(fieldnames(est))
    est = estimate_parameters(flightLogPath);
end

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(here, 'framework', 'plant'));

P = default_rocket_params();
override = struct();
override.t_end = t(end);
override.p_unstable = estimate_unstable_p(theta_log, q_log, t);
override.servo_slew = max(1, rad2deg(est.actuator_slew_rate));
override.deadband = est.deadband;
override.backlash = est.backlash;
override.tau_act = est.actuator_lag_s;
override.control_effectiveness = est.control_effectiveness;
override.aero_damp = est.aerodynamic_damping;

[cfg, sc, realism] = build_realistic_cfg(P, override);
cfg.dt = est.dt;
realism.sensor_latency_steps = max(1, round(P.rocket.latency));
realism.servo_deadband = est.deadband;
realism.servo_backlash = est.backlash;

cfg.controllers.PID.Kp = infer_gain(logT, 'Kp', 10.0);
cfg.controllers.PID.Kd = infer_gain(logT, 'Kd', 16.0);
cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;

% Reconstruct reference profile from log if available.
if ~all(theta_ref_log == 0)
    tt = t;
    rr = theta_ref_log;
    sc.theta_ref_fun = @(x) interp1(tt, rr, x, 'linear', 'extrap');
    sc.q_ref_fun = @(x) interp1(tt, gradient(rr, tt), x, 'linear', 'extrap');
end

sim = simulate_case_realistic('PID', sc, cfg, p.Results.seed, realism);

n = min(numel(t), numel(sim.time));
out = struct();
out.time = t(1:n);
out.theta_log = theta_log(1:n);
out.theta_pred = sim.theta(1:n);
out.q_log = q_log(1:n);
out.q_pred = sim.q(1:n);
out.u_cmd_log = u_cmd_log(1:n);
out.u_cmd_pred = sim.u_cmd(1:n);
out.u_act_log = u_act_log(1:n);
out.u_act_pred = sim.u_act(1:n);
out.theta_ref = theta_ref_log(1:n);
out.estimate = est;

if ~isempty(p.Results.resultsDir)
    if ~exist(p.Results.resultsDir, 'dir')
        mkdir(p.Results.resultsDir);
    end
    replayT = table(out.time, out.theta_log, out.theta_pred, out.q_log, out.q_pred, ...
        out.u_cmd_log, out.u_cmd_pred, out.u_act_log, out.u_act_pred, ...
        'VariableNames', {'time', 'theta_log', 'theta_pred', 'q_log', 'q_pred', ...
        'u_cmd_log', 'u_cmd_pred', 'u_act_log', 'u_act_pred'});
    writetable(replayT, fullfile(p.Results.resultsDir, 'flight_replay_trace.csv'));
end


function val = infer_gain(T, name, fallback)
if ismember(name, T.Properties.VariableNames)
    v = T.(name);
    val = median(v(isfinite(v)), 'omitnan');
    if ~isfinite(val)
        val = fallback;
    end
else
    val = fallback;
end


function p_unst = estimate_unstable_p(theta, q, t)
if numel(theta) < 20
    p_unst = 4;
    return;
end

dt = median(diff(t));
qdot = diff(q) / max(dt, 1e-6);
X = [theta(1:end-1), q(1:end-1), ones(numel(qdot),1)];
coef = X \ qdot;
ktheta = max(0, coef(1));
p_unst = sqrt(max(0, ktheta));
p_unst = min(max(p_unst, 0), 20);


function [t, theta, q, u_cmd, u_act, theta_ref] = normalize_log_channels(T)
name_map = lower(string(T.Properties.VariableNames));

idx_t = find_first(name_map, ["time", "t_s", "t"]);
idx_theta = find_first(name_map, ["theta", "theta_rad", "pitch_rad", "theta_meas", "pitch"]);
idx_q = find_first(name_map, ["q", "q_rad_s", "pitch_rate", "q_meas"]);
idx_ucmd = find_first(name_map, ["u_cmd", "cmd", "gimbal_cmd", "actuator_cmd"]);
idx_uact = find_first(name_map, ["u_act", "act", "gimbal_act", "actuator_pos", "u_act_meas"]);
idx_tref = find_first(name_map, ["theta_ref", "pitch_ref", "theta_target"]);

if any([idx_t, idx_theta, idx_q, idx_ucmd, idx_uact] == 0)
    error('replay_flight:missingColumns', ...
        'Missing required columns. Need time, theta, q, u_cmd, u_act compatible names.');
end

t = double(T{:, idx_t});
theta = double(T{:, idx_theta});
q = double(T{:, idx_q});
u_cmd = double(T{:, idx_ucmd});
u_act = double(T{:, idx_uact});
if idx_tref == 0
    theta_ref = zeros(size(theta));
else
    theta_ref = double(T{:, idx_tref});
end

if max(abs(theta), [], 'omitnan') > 2*pi
    theta = deg2rad(theta);
end
if max(abs(q), [], 'omitnan') > 20
    q = deg2rad(q);
end
if max(abs(theta_ref), [], 'omitnan') > 2*pi
    theta_ref = deg2rad(theta_ref);
end


function idx = find_first(names, candidates)
idx = 0;
for i = 1:numel(candidates)
    k = find(names == candidates(i), 1, 'first');
    if ~isempty(k)
        idx = k;
        return;
    end
end
