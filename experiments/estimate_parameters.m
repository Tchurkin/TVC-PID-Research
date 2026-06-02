function est = estimate_parameters(flightLogPath)
%ESTIMATE_PARAMETERS Estimate key physical parameters from a flight log.
% Required columns: time and actuator/attitude channels.
%
% Outputs (units in simulator code-space unless noted):
%   actuator_slew_rate
%   backlash
%   deadband
%   actuator_lag_s
%   control_effectiveness
%   aerodynamic_damping

T = readtable(flightLogPath);
[t, theta, q, u_cmd, u_act] = normalize_log_channels(T);

dt = median(diff(t));
if ~isfinite(dt) || dt <= 0
    error('estimate_parameters:badTime', 'Time vector is invalid or non-monotone.');
end

u_rate = diff(u_act) / dt;
qdot = diff(q) / dt;

% 95th percentile delivered rate is robust to spikes and sensor dropouts.
actuator_slew_rate = prctile(abs(u_rate), 95);

% Estimate deadband from command residual where actuator barely moves.
cmd_resid = abs(u_cmd(2:end) - u_act(1:end-1));
quiet_mask = abs(u_rate) <= max(1e-6, 0.05 * actuator_slew_rate);
if any(quiet_mask)
    deadband = median(cmd_resid(quiet_mask), 'omitnan');
else
    deadband = median(cmd_resid, 'omitnan');
end

% Estimate backlash from direction reversals where actuator response is delayed.
du_cmd = diff(u_cmd);
du_act = diff(u_act);
rev_idx = find(sign(du_cmd(2:end)) .* sign(du_cmd(1:end-1)) < 0) + 1;
bl = nan(numel(rev_idx), 1);
for i = 1:numel(rev_idx)
    k = rev_idx(i);
    j0 = max(1, k - 2);
    j1 = min(numel(du_act), k + 2);
    local_motion = abs(du_act(j0:j1));
    local_cmd = abs(du_cmd(j0:j1));
    if any(local_cmd > 0)
        bl(i) = max(0, median(local_cmd - local_motion, 'omitnan'));
    end
end
backlash = median(bl, 'omitnan');
if ~isfinite(backlash)
    backlash = 0;
end

% First-order lag: u_act_dot ~= (u_cmd - u_act) / tau
x = (u_cmd(2:end) - u_act(1:end-1));
y = u_rate;
valid = abs(x) > max(1e-6, 0.02 * max(abs(x)));
if any(valid)
    beta = x(valid) \ y(valid);
    actuator_lag_s = 1 / max(1e-6, beta);
else
    actuator_lag_s = 0.05;
end
actuator_lag_s = min(max(actuator_lag_s, 0.01), 0.40);

% Dynamics identification: qdot ~= keff*u_act - aero_damp*q + bias
X = [u_act(2:end), -q(1:end-1), ones(numel(qdot), 1)];
coef = X \ qdot;
control_effectiveness = coef(1);
aerodynamic_damping = coef(2);

% Clip to physically plausible ranges for stable downstream usage.
control_effectiveness = min(max(control_effectiveness, 0.1), 30.0);
aerodynamic_damping = min(max(aerodynamic_damping, 0.0), 10.0);

est = struct();
est.dt = dt;
est.actuator_slew_rate = actuator_slew_rate;
est.backlash = max(0, backlash);
est.deadband = max(0, deadband);
est.actuator_lag_s = actuator_lag_s;
est.control_effectiveness = control_effectiveness;
est.aerodynamic_damping = aerodynamic_damping;
est.log_path = string(flightLogPath);


function [t, theta, q, u_cmd, u_act] = normalize_log_channels(T)
name_map = lower(string(T.Properties.VariableNames));

idx_t = find_first(name_map, ["time", "t_s", "t"]);
idx_theta = find_first(name_map, ["theta", "theta_rad", "pitch_rad", "theta_meas", "pitch"]);
idx_q = find_first(name_map, ["q", "q_rad_s", "pitch_rate", "q_meas"]);
idx_ucmd = find_first(name_map, ["u_cmd", "cmd", "gimbal_cmd", "actuator_cmd"]);
idx_uact = find_first(name_map, ["u_act", "act", "gimbal_act", "actuator_pos", "u_act_meas"]);

if any([idx_t, idx_theta, idx_q, idx_ucmd, idx_uact] == 0)
    error('estimate_parameters:missingColumns', ...
        'Missing required columns. Need time, theta, q, u_cmd, u_act compatible names.');
end

t = double(T{:, idx_t});
theta = double(T{:, idx_theta});
q = double(T{:, idx_q});
u_cmd = double(T{:, idx_ucmd});
u_act = double(T{:, idx_uact});

% Unit normalization heuristics.
if max(abs(theta), [], 'omitnan') > 2*pi
    theta = deg2rad(theta);
end
if max(abs(q), [], 'omitnan') > 20
    q = deg2rad(q);
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
