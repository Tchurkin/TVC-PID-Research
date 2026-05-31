function [u_cmd, diag] = lqr_layer(theta, q_meas, u_act_meas, ctrl, dt)
%LQR_LAYER Baseline LQR full-state feedback for 1-DOF pitch (3-state).
%
%  Augmented state: x = [theta; q; u_act]
%  Plant:
%    A = [0,         1,          0        ]
%        [0, -aero_damp,   control_eff     ]
%        [0,         0,   -1/tau_act       ]
%    B = [0; 0; 1/tau_act]
%
%  LQR solves:  min ∫ x'Q3x + u'Ru dt
%  giving gain: K = lqr(A, B, Q3, R)  — 1x3 vector
%  Control law: u = -K * [theta; q; u_act]
%
%  Including u_act in the state means the controller naturally accounts
%  for how hard the actuator is currently being driven, reducing the
%  tendency to overcommand a saturated or lagging actuator.
%
%  ctrl must contain: .K (1x3), .u_max

if nargin < 5
    dt = 0.01; %#ok<NASGU>
end

if numel(ctrl.K) == 2
    x = [theta; q_meas];
else
    x = [theta; q_meas; u_act_meas];
end
u_cmd = -ctrl.K * x;
u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));

diag = struct();
diag.K = ctrl.K;
end
