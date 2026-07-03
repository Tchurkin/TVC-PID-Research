function out = simulate_case_2dof(controller_name, sc, cfg, seed)
%SIMULATE_CASE_2DOF  Pitch + yaw two-axis TVC simulation.
%
%  Two parallel single-axis controllers, one per axis. Plant has light
%  inertial cross-coupling (yaw rate appears as bias torque in pitch
%  channel and vice versa) and a yaw-channel slew fault. Used to show
%  the JOINT_ADAPTIVE architecture generalizes axis-wise: each axis
%  identifies its own envelope independently and the cross-coupled
%  disturbance does not destabilize identification.
%
%  Plant per-axis (i in {pitch, yaw}):
%    theta_i_dot = q_i
%    q_i_dot     = keff_i * u_act_i - aero_damp * q_i + d_i(t) + couple_ji * q_j
%
%  Fault: yaw-channel slew envelope drops at sc.fault_time. Pitch is
%  unaffected by the fault but receives coupling from the yaw transient.

if nargin < 4, seed = 1; end
rng(seed);

dt = cfg.dt;
t  = (0:dt:sc.t_end)';
N  = numel(t);

axes_ = ["pitch", "yaw"];
state.theta   = zeros(N, 2);
state.q       = zeros(N, 2);
state.u_act   = zeros(N, 2);
state.u_cmd   = zeros(N, 2);
state.slew_est = nan(N, 2);
state.gain_scale = nan(N, 2);

% Per-axis initial conditions
state.theta(1, 1) = cfg.plant.theta0;                            % pitch IC
state.theta(1, 2) = deg2rad(2);                                  % small yaw IC

ctrl_name = upper(string(controller_name));
ctrl      = cfg.controllers.(ctrl_name);

% Independent controller state per axis
adapt = {struct(), struct()};

couple_gain = 0.30;   % cross-axis aero coupling (yaw rate -> pitch torque)

for k = 2:N
    tk = t(k);

    % Per-axis plant params (only yaw axis takes the slew fault)
    keff      = [cfg.plant.control_eff, cfg.plant.control_eff];
    aero_damp = [cfg.plant.aero_damp,   cfg.plant.aero_damp];
    tau_act   = [cfg.plant.tau_act,     cfg.plant.tau_act];
    slew_max  = [cfg.plant.slew_max,    cfg.plant.slew_max];

    if tk >= sc.fault_time
        slew_max(2) = slew_max(2) * sc.slew_scale_post;          % YAW slew degrades
    end

    % Disturbances: pitch gets a steady wind, yaw gets a stronger gust
    d = [sc.disturbance_amp * sin(2*pi*sc.disturbance_freq_hz*tk), ...
         1.5*sc.disturbance_amp * sin(2*pi*sc.disturbance_freq_hz*tk + pi/3)];

    for ax = 1:2
        switch ctrl_name
            case "FIXED_LQR"
                x = [state.theta(k-1, ax); state.q(k-1, ax)];
                u = max(-ctrl.u_max, min(ctrl.u_max, -ctrl.K * x));

            case "JOINT_ADAPTIVE"
                q_prev    = state.q(max(1, k-2), ax);
                uact_prev = state.u_act(max(1, k-2), ax);
                ucmd_prev = state.u_cmd(max(1, k-1), ax);
                [u, adapt{ax}, dgn] = lqr_layer_joint_adaptive( ...
                    state.theta(k-1, ax), state.q(k-1, ax), state.u_act(k-1, ax), ...
                    uact_prev, q_prev, ucmd_prev, adapt{ax}, ctrl, dt);
                state.slew_est(k, ax)   = dgn.slew_est;
                state.gain_scale(k, ax) = dgn.gain_scale;

            case "PCH_LQR"
                q_prev = state.q(max(1, k-2), ax);
                [u, adapt{ax}, dgn] = lqr_layer_pch_lqr( ...
                    state.theta(k-1, ax), state.q(k-1, ax), state.u_act(k-1, ax), ...
                    q_prev, adapt{ax}, ctrl, dt);
                state.slew_est(k, ax)   = dgn.slew_est;
                state.gain_scale(k, ax) = dgn.gain_scale;

            otherwise
                error('simulate_case_2dof:UnknownController', ...
                    '2-DOF demo supports FIXED_LQR, PCH_LQR, JOINT_ADAPTIVE');
        end
        state.u_cmd(k, ax) = u;

        % Actuator (1st-order lag, slew-limited) per axis
        du_cmd  = (state.u_cmd(k, ax) - state.u_act(k-1, ax)) / max(1e-5, tau_act(ax));
        du      = max(-slew_max(ax), min(slew_max(ax), du_cmd));
        state.u_act(k, ax) = state.u_act(k-1, ax) + dt * du;
        state.u_act(k, ax) = max(-cfg.plant.u_max, min(cfg.plant.u_max, state.u_act(k, ax)));
    end

    % Plant dynamics with cross-axis coupling
    other = [2, 1];
    for ax = 1:2
        cpl = couple_gain * state.q(k-1, other(ax));
        qdot = keff(ax) * state.u_act(k, ax) - aero_damp(ax) * state.q(k-1, ax) ...
             + d(ax) + cpl;
        state.q(k, ax)     = state.q(k-1, ax) + dt * qdot;
        state.theta(k, ax) = state.theta(k-1, ax) + dt * state.q(k, ax);
    end
end

out.time     = t;
out.theta    = state.theta;        % N x 2
out.q        = state.q;
out.u_cmd    = state.u_cmd;
out.u_act    = state.u_act;
out.slew_est = state.slew_est;
out.gain_scale = state.gain_scale;
out.controller = ctrl_name;
out.axes       = axes_;
out.seed       = seed;
end
