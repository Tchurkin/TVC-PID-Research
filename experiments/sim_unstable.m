function out = sim_unstable(params)
%SIM_UNSTABLE  Minimal short-burn unstable TVC plant + linear feedback.
%
% Plant (linearized inverted-pendulum-like attitude dynamics):
%   theta_ddot = p^2 * theta - damp * theta_dot + keff * u_act + disturb
%   u_act dynamics: first-order with time constant tau_act, slew-rate limit,
%                   and position saturation at +/- u_max.
%   Sensor:  gyro noise + 1-step latency on (theta, q).
%
% params fields (all required unless noted):
%   p            unstable pole rate (1/s)   [destabilizing term coefficient sqrt]
%   damp         rate damping (1/s)
%   keff         control effectiveness (rad/s^2 per unit u)
%   tau_act      actuator time constant (s)
%   slew_max     actuator slew limit (units/s)
%   u_max        actuator saturation (units)
%   latency_n    sensor latency in samples
%   gyro_std     gyro white noise std (rad/s)
%   K            2x1 state feedback gain [Kth Kq] s.t. u = -K * [theta; q]
%   dt           timestep (s)
%   t_end        sim duration (s)
%   theta0       initial attitude (rad)
%   gust_std     additive disturbance std (rad/s^2), 0 to disable
%   gust_tau     disturbance correlation time (s)
%   p_traj       optional function handle p(t) for time-varying instability
%
% out fields: time, theta, q, u_cmd, u_act, params

if ~isfield(params, 'p_traj'), params.p_traj = []; end
if ~isfield(params, 'gust_std'), params.gust_std = 0; end
if ~isfield(params, 'gust_tau'), params.gust_tau = 0.3; end
if ~isfield(params, 'gyro_std'), params.gyro_std = 0; end
if ~isfield(params, 'latency_n'), params.latency_n = 1; end
if ~isfield(params, 'damp'), params.damp = 0.3; end
if ~isfield(params, 'theta0'), params.theta0 = 0; end
if ~isfield(params, 'use_rate_aw'), params.use_rate_aw = false; end  % rate-aware command shaper

dt = params.dt;
t  = (0:dt:params.t_end)';
N  = numel(t);

theta = zeros(N,1);
q     = zeros(N,1);
u_act = zeros(N,1);
u_cmd = zeros(N,1);

theta(1) = params.theta0;

% Latency buffer for measurements
lat = max(1, params.latency_n);
buf_theta = repmat(theta(1), lat, 1);
buf_q     = repmat(q(1),     lat, 1);

% Disturbance state
gust = 0;
if params.gust_std > 0 && params.gust_tau > 0
    gust_alpha = exp(-dt / params.gust_tau);
    gust_sigma = params.gust_std * sqrt(1 - gust_alpha^2);
else
    gust_alpha = 0; gust_sigma = 0;
end

K = params.K(:).';

for k = 2:N
    tk = t(k);

    % Time-varying instability if specified
    if ~isempty(params.p_traj)
        p_now = params.p_traj(tk);
    else
        p_now = params.p;
    end

    % --- Measurements with noise + latency
    th_meas = buf_theta(end) + params.gyro_std * 0 * randn();  % theta noise via integration
    q_meas  = buf_q(end)     + params.gyro_std * randn();

    % --- Controller
    u = -K * [th_meas; q_meas];
    u = max(-params.u_max, min(params.u_max, u));

    % --- Rate-aware anti-windup: clip u_cmd to the value that fully saturates
    % the actuator's slew rate. Beyond that, extra command magnitude does
    % nothing useful (the actuator is already moving at slew_max) but in
    % integrator-style controllers it would wind up. For our pure state-
    % feedback this should be a near-no-op -- which itself is a useful
    % falsification finding: if the dip persists, the wind-up is not the
    % issue (so the H1 effect is a plant/controller-design property).
    if params.use_rate_aw
        u_hi = u_act(k-1) + params.slew_max * params.tau_act;
        u_lo = u_act(k-1) - params.slew_max * params.tau_act;
        u = max(u_lo, min(u_hi, u));
    end
    u_cmd(k) = u;

    % --- Actuator: first-order + slew + saturation
    du_cmd = (u_cmd(k) - u_act(k-1)) / max(1e-6, params.tau_act);
    du = max(-params.slew_max, min(params.slew_max, du_cmd));
    u_act(k) = u_act(k-1) + dt * du;
    u_act(k) = max(-params.u_max, min(params.u_max, u_act(k)));

    % --- Disturbance
    gust = gust_alpha * gust + gust_sigma * randn();

    % --- Plant
    th_ddot = p_now^2 * theta(k-1) - params.damp * q(k-1) + params.keff * u_act(k) + gust;
    q(k)     = q(k-1) + dt * th_ddot;
    theta(k) = theta(k-1) + dt * q(k);

    % Push to latency buffer
    buf_theta = [buf_theta(2:end); theta(k)];
    buf_q     = [buf_q(2:end);     q(k)];
end

out.time   = t;
out.theta  = theta;
out.q      = q;
out.u_cmd  = u_cmd;
out.u_act  = u_act;
out.params = params;
end
