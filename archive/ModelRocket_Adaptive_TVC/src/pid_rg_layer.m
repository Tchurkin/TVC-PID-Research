function [u_cmd, state_new, diag] = pid_rg_layer(theta_meas, q_meas, u_act_meas, ...
                                                  state_prev, ctrl, dt)
%PID_RG_LAYER  PID inner loop + Scalar Reference Governor (SRG) on the command.
%
%  Standard Bemporad-1998 / Kolmanovsky-Gilbert-1999 SRG construction:
%   - The PID computes a raw desired actuator command r_raw.
%   - The governor produces v = v_prev + kappa*(r_raw - v_prev), kappa in [0,1].
%   - kappa is chosen as the largest value such that the predicted future
%     trajectory of the augmented state x = [theta; q; u_act], under the
%     assumption that v is held constant for the next H steps, satisfies
%     the safety constraints (|theta| <= theta_safety, |q| <= q_safety,
%     |v| <= u_max).
%
%  The predictor is open-loop (v held constant). For an unstable plant this
%  predictor diverges eventually, so the governor naturally rejects large
%  command jumps that would lead to rapid divergence. The governor's value
%  is in the regime where the inner PID is "almost good enough" and is
%  occasionally demanding actuator swings that produce slew-saturation-
%  driven divergence.
%
%  Compute budget (H=10, scalar bisection 8 iters): ~720 float multiplies
%  + ~240 comparisons per call. ~225 us on ATmega 16 MHz, <50 us on SAMD21.
%
%  Inputs:
%    theta_meas, q_meas, u_act_meas : current measured/estimated states
%    state_prev   : struct with .integrator, .e_prev, .v_prev
%    ctrl         : struct with .Kp .Ki .Kd .u_max .i_lim .rg
%       ctrl.rg fields (all required):
%         .H               horizon (steps), e.g. 10
%         .theta_safety    rad, attitude bound for predictor
%         .q_safety        rad/s, rate bound for predictor
%         .keff_assumed    plant model used by predictor (bench/sysid)
%         .aero_damp_assumed
%         .p_assumed       unstable pole rate (rad/s)
%         .tau_act_assumed actuator first-order time constant (s)
%         .bisect_iters    typically 6-10
%    dt           : timestep (s)
%
%  Outputs:
%    u_cmd        : governed command sent to the actuator
%    state_new    : updated state
%    diag         : struct with .kappa, .r_raw, .governed (bool)

state_new = state_prev;

% --- Inner PID ---------------------------------------------------------
e  = -theta_meas;
de = (e - state_prev.e_prev) / dt;
state_new.integrator = state_prev.integrator + e * dt;
state_new.integrator = max(-ctrl.i_lim, min(ctrl.i_lim, state_new.integrator));
r_raw = ctrl.Kp * e + ctrl.Ki * state_new.integrator + ctrl.Kd * de;
r_raw = max(-ctrl.u_max, min(ctrl.u_max, r_raw));
state_new.e_prev = e;

% --- Scalar Reference Governor -----------------------------------------
rg = ctrl.rg;

if ~isfield(state_prev, 'v_prev') || isempty(state_prev.v_prev)
    v_prev = u_act_meas;
else
    v_prev = state_prev.v_prev;
end

% Augmented continuous-time plant matrix
%   theta_dot = q
%   q_dot     = p^2 * theta - damp * q + keff * u_act
%   u_act_dot = (v - u_act) / tau_act   (linear, ignores slew sat in predictor)
p2   = rg.p_assumed^2;
damp = rg.aero_damp_assumed;
keff = rg.keff_assumed;
tau  = max(1e-5, rg.tau_act_assumed);

A_c = [0,   1,    0;
       p2, -damp, keff;
       0,   0,   -1/tau];
b_c = [0; 0; 1/tau];

% Euler discretization (sufficient at hobby dt ~ 5 ms)
A_d = eye(3) + dt * A_c;
b_d = dt * b_c;

x0 = [theta_meas; q_meas; u_act_meas];

% Try kappa = 1 (pass-through) first
v_full = r_raw;
if predict_feasible(x0, v_full, A_d, b_d, rg, ctrl.u_max)
    kappa = 1.0;
    governed = false;
else
    % Bisection on [0, 1]
    kappa_lo = 0.0;
    kappa_hi = 1.0;
    for ii = 1:rg.bisect_iters
        kappa_mid = 0.5 * (kappa_lo + kappa_hi);
        v_try = v_prev + kappa_mid * (r_raw - v_prev);
        if predict_feasible(x0, v_try, A_d, b_d, rg, ctrl.u_max)
            kappa_lo = kappa_mid;
        else
            kappa_hi = kappa_mid;
        end
    end
    kappa = kappa_lo;
    governed = true;
end

u_cmd = v_prev + kappa * (r_raw - v_prev);
u_cmd = max(-ctrl.u_max, min(ctrl.u_max, u_cmd));
state_new.v_prev = u_cmd;

if nargout >= 3
    diag.kappa = kappa;
    diag.r_raw = r_raw;
    diag.governed = governed;
end
end


function ok = predict_feasible(x0, v, A_d, b_d, rg, u_max_pos)
%PREDICT_FEASIBLE  Roll out H steps with v held constant; check constraints.
ok = true;
if abs(v) > u_max_pos
    ok = false; return;
end
x = x0;
for i = 1:rg.H
    x = A_d * x + b_d * v;
    if abs(x(1)) > rg.theta_safety
        ok = false; return;
    end
    if abs(x(2)) > rg.q_safety
        ok = false; return;
    end
end
end
