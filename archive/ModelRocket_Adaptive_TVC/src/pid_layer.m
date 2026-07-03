function [u_cmd, state_new] = pid_layer(theta, state_prev, ctrl, dt)
%PID_LAYER Basic PID controller for pitch angle (setpoint = 0).
%
%  theta      - measured pitch angle (rad)
%  state_prev - struct: .integrator, .e_prev
%  ctrl       - struct: .Kp, .Ki, .Kd, .u_max, .i_lim
%  dt         - timestep (s)
%
%  Straightforward P-I-D with integrator anti-windup clamp.

state_new = state_prev;

e  = -theta;                                  % error = setpoint(0) - measurement
de = (e - state_prev.e_prev) / dt;            % derivative of error

state_new.integrator = state_prev.integrator + e * dt;

% Anti-windup: clamp integrator so Ki*I never exceeds output limit
i_lim = ctrl.i_lim;
state_new.integrator = max(-i_lim, min(i_lim, state_new.integrator));

u_cmd = ctrl.Kp * e  +  ctrl.Ki * state_new.integrator  +  ctrl.Kd * de;
u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));

state_new.e_prev = e;
end
