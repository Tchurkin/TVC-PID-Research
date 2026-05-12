function [u_cmd, z_new, aw_new, diag] = adrc_layer_aca(y_ct, z_prev, aw_prev, u_prev, params, dt)
%ADRC_LAYER_ACA Actuator-Constraint-Acknowledging ADRC for crosstrack control.
% Adds an auxiliary anti-windup/compensation state to handle saturation.
%
% Plant assumption: y_ddot = b0*u + d
% ESO: z=[z1 z2 z3]^T estimates y, y_dot, total disturbance d.
%
% Auxiliary compensation state aw tracks saturation error and relaxes command:
%   u_unsat = (u0 - z3)/b0 + aw
%   u_cmd   = sat(u_unsat)
%   e_sat   = u_cmd - u_unsat
%   aw_dot  = -lambda_aw*aw + k_aw*e_sat

% Optional saturation-aware observer injection gain.
if isfield(params, 'k_sat_obs')
	k_sat_obs = params.k_sat_obs;
else
	k_sat_obs = 0.0;
end

% ESO prediction/update
e1 = y_ct - z_prev(1);
z1 = z_prev(1) + dt * (z_prev(2) + params.l1 * e1);
z2 = z_prev(2) + dt * (z_prev(3) + params.b0 * u_prev + params.l2 * e1);
z3 = z_prev(3) + dt * (params.l3 * e1);
z_pred = [z1; z2; z3];

% Nominal ADRC law
u0 = params.Kp * (0 - z1) + params.Kd * (0 - z2);
u_unsat = (u0 - z3) / params.b0 + aw_prev;

% Saturation
u_cmd = max(-params.u_max, min(params.u_max, u_unsat));
e_sat = u_cmd - u_unsat;

% Auxiliary anti-windup/compensation state
aw_dot = -params.lambda_aw * aw_prev + params.k_aw * e_sat;
aw_new = aw_prev + dt * aw_dot;

% Saturation-aware observer correction.
% When the actuator clips, e_sat encodes command-realization mismatch;
% inject this into z2/z3 so disturbance estimate remains consistent.
z2 = z_pred(2) + dt * (k_sat_obs * e_sat);
z3 = z_pred(3) + dt * (0.5 * k_sat_obs * e_sat);
z_new = [z_pred(1); z2; z3];

% Diagnostic channels for research metrics
diag.dist_est = z3;
diag.u_unsat = u_unsat;
diag.sat_err = e_sat;
diag.aw = aw_new;
diag.k_sat_obs = k_sat_obs;
end
