%% ADRC Layer Over L1 — Direction B (STUB)
% =========================================
% Adds an ADRC disturbance observer layer on top of the L1 guidance
% baseline to compensate for crosswind as a lumped disturbance.
%
% ARCHITECTURE:
%   Standard L1 outputs:  aileron_cmd_l1  (based on kinematic track error)
%   ADRC layer adds:      aileron_feedfwd (based on estimated wind disturbance)
%   Combined:             aileron_cmd = sat(aileron_cmd_l1 + aileron_feedfwd, limits)
%
% THE KEY IDEA:
%   Wind creates a persistent cross-track drift. L1 corrects this
%   reactively (sees the error, then responds). ADRC estimates the
%   *cause* (wind force on the airframe) from the lateral acceleration
%   residual and pre-compensates, cutting the lag.
%
% ESO FOR LATERAL AXIS:
%   State: [y, y_dot, d]  where d = lumped disturbance (wind + model error)
%   y     = cross-track position
%   y_dot = cross-track rate  
%   d     = wind disturbance acceleration (m/s^2)
%
%   z1_dot = z2 + L1*(y - z1)
%   z2_dot = z3 + b0*u + L2*(y - z1)
%   z3_dot =           + L3*(y - z1)
%
%   Feedforward: aileron_feedfwd = -z3 / b0_lateral
%
% ACTUATOR CONSTRAINTS:
%   Aileron deflection limit: delta_max (from RC airframe spec, default 25 deg)
%   Actuator PT2 bandwidth: typically 8-15 Hz for RC servo
%
% FUNCTION SIGNATURE (to implement):
%   [aileron_ff, z_new] = adrc_layer_step(y, aileron_l1, z_prev, params, dt)
%
% TODO (Phase 4 implementation):
%   1. Implement ESO discrete-time update
%   2. Compute feedforward correction
%   3. Apply actuator saturation to combined command
%   4. Validate: constant crosswind should produce near-zero z3 steady-state

function [u_cmd, z_new] = adrc_layer(y_ct, z_prev, u_prev, params, dt)
%ADRC_LAYER  ADRC crosstrack controller with wind disturbance estimation.
%
%  PLANT MODEL:
%    y_ct_dot  = vy_ct + w_wind     (wind is velocity disturbance, m/s)
%    vy_ct_dot = u                  (lateral accel command)
%  Equivalently, 2nd-order plant: y_ct_ddot = u + d
%    where d = dw_wind/dt + (coupling terms)  ~ 0 for slow wind
%
%  2nd-order ADRC (3rd-order ESO for 2nd-order plant):
%    z1 ~ y_ct,  z2 ~ y_ct_dot,  z3 ~ lumped disturbance d
%    z1_dot = z2 + l1*(y_ct - z1)
%    z2_dot = z3 + b0*u + l2*(y_ct - z1)
%    z3_dot =           + l3*(y_ct - z1)
%
%    b0 = 1.0 (since y_ct_ddot = u + d directly)
%    Control: u0 = Kp*(0 - z1) + Kd*(0 - z2)
%             u  = sat( (u0 - z3)/b0, +/-u_max )
%
%  ESO bandwidth omega_o >> omega_c:
%    l1 = 3*omega_o,  l2 = 3*omega_o^2,  l3 = omega_o^3
%
%  Inputs:
%    y_ct    - measured cross-track position (m)
%    z_prev  - [3x1] ESO state [z1; z2; z3]
%    u_prev  - previous control output (m/s^2)
%    params  - struct: .l1 .l2 .l3 .b0 .Kp .Kd .u_max
%    dt      - timestep (s)
%  Outputs:
%    u_cmd   - lateral acceleration command (m/s^2), saturated
%    z_new   - updated ESO state

% ESO update (Euler)
e1    = y_ct - z_prev(1);
z1    = z_prev(1) + dt * (z_prev(2) + params.l1 * e1);
z2    = z_prev(2) + dt * (z_prev(3) + params.b0 * u_prev + params.l2 * e1);
z3    = z_prev(3) + dt * (params.l3 * e1);
z_new = [z1; z2; z3];

% Control law: PD + disturbance cancellation
u0    = params.Kp * (0 - z1) + params.Kd * (0 - z2);
u_cmd = (u0 - z3) / params.b0;
u_cmd = max(-params.u_max, min(params.u_max, u_cmd));
end
