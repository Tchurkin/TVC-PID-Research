%% L1 Guidance Law Baseline — Direction B (STUB)
% ===============================================
% Implements the ArduPilot L1 guidance law for fixed-wing path following.
%
% ALGORITHM (from ArduPilot AP_L1_Control):
%   1. Find the closest point on the path to the aircraft.
%   2. Select a lookahead point L1 meters ahead on the path.
%   3. Compute the angle 'eta' between aircraft velocity and line to L1.
%   4. Lateral acceleration command: a_lat = 2 * V^2 * sin(eta) / L1
%   5. Convert to bank angle: phi_cmd = atan(a_lat / g)
%   6. Aileron deflection: from phi_cmd through a roll rate PD loop
%
% PARAMETERS:
%   L1_period   — L1 guidance period (s), typically 20 s (tunable)
%   L1_damping  — L1 damping ratio, typically 0.75
%   V_cruise    — cruise airspeed (m/s)
%
% NOTE ON ACTUATOR LIMITS:
%   Standard L1 does NOT model actuator bandwidth or saturation.
%   This is the gap that ADRC+L1 (in adrc_layer.m) addresses.
%
% FUNCTION SIGNATURE (to implement):
%   [aileron_cmd, crosstrack_err] = l1_guidance(pos, vel, waypoints, params, dt)
%
%   Inputs:
%     pos        — [x, y] aircraft position (m, ENU frame)
%     vel        — [vx, vy] aircraft velocity (m/s)
%     waypoints  — Nx2 array of [x, y] waypoints
%     params     — struct with L1_period, L1_damping, V_cruise
%     dt         — timestep (s)
%   Output:
%     aileron_cmd   — aileron deflection command (deg)
%     crosstrack_err — signed cross-track error (m, positive = right of track)
%
% TODO (Phase 4 implementation):
%   1. Implement lookahead point selection
%   2. Implement lateral acceleration command
%   3. Implement bank angle + aileron mapping
%   4. Test: straight-line track, no wind => crosstrack -> 0

function [u_cmd, ct_err] = l1_guidance(y_ct, vy_ct, V, params)
%L1_GUIDANCE  ArduPilot-equivalent L1 lateral guidance law.
%
%  For a straight track along the x-axis (y_ct = lateral deviation):
%    lookahead point is L1 ahead along track.
%    eta = signed angle from velocity vector to lookahead direction
%    lateral accel cmd: a_lat = 2*V^2*sin(eta)/L1
%    converted to control: u_cmd = a_lat (m/s^2 desired crosstrack accel)
%
%  The aircraft crosstrack model (Direction B):
%    y_ct_dot  = vy_ct + w_wind
%    vy_ct_dot = u (lateral acceleration command, limited by bank angle)
%  So u is the desired lateral acceleration, actuator-limited to u_max.
%
%  L1 = L1_period * V / (2*pi)  [m]  (guidance lookahead distance)
%
%  Inputs:
%    y_ct    - cross-track position (m, positive = right of track)
%    vy_ct   - cross-track velocity (m/s)
%    V       - aircraft speed (m/s)
%    params  - struct: .L1_period (s), .u_max (m/s^2)
%  Outputs:
%    u_cmd   - lateral acceleration command (m/s^2), saturated
%    ct_err  - cross-track error (same as y_ct, for logging)

L1 = params.L1_period * V / (2*pi);   % lookahead distance (m)

% Angle from velocity to lookahead (small-angle approximation for straight track)
% lookahead point is at [L1, -y_ct] relative to aircraft
eta = atan2(-y_ct, L1);

% Lateral acceleration command (L1 guidance law)
a_lat = 2 * V^2 * sin(eta) / L1;

% Saturate to max lateral acceleration (corresponds to max bank angle)
u_cmd  = max(-params.u_max, min(params.u_max, a_lat));
ct_err = y_ct;
end
