%% Crosswind Scenario Generator — Direction B (STUB)
% ====================================================
% Generates simulation scenario: straight-line waypoint track +
% sustained crosswind of specified speed, with optional turbulence overlay.
%
% SCENARIO GEOMETRY:
%   Waypoints: [0,0] -> [500, 0]  (500 m straight east)
%   Initial position: [0, 0] with initial heading east
%   Crosswind: constant northward component (positive = pushes aircraft south)
%
% THREE WIND CASES:
%   'low'      — 2 m/s crosswind (mild, achievable almost anywhere)
%   'medium'   — 5 m/s crosswind (typical field day, key test case)
%   'high'     — 8 m/s crosswind (gusty day, stress test)
%
% AIRFRAME DEFAULTS (generic trainer RC plane):
%   V_cruise   = 15  m/s   cruise airspeed
%   mass       = 1.5 kg
%   wing_area  = 0.3 m^2
%   Cl_alpha   = 4.5 rad^-1
%   CD0        = 0.04
%   AR         = 6.0
%
% FUNCTION SIGNATURE (to implement):
%   scenario = crosswind_scenario(wind_case, varargin)
%   Returns struct with: waypoints, wind_vec, airframe, t_end, dt
%
% TODO (Phase 4 implementation):
%   1. Build scenario struct with all parameters
%   2. Option to add Dryden turbulence overlay (reuse Direction A's Dryden)
%   3. Validate: no-wind case => aircraft tracks perfectly with any controller

function sc = crosswind_scenario(wind_case, dt, t_end, seed)
%CROSSWIND_SCENARIO  Build scenario struct for RC-plane crosswind simulation.
%
%  AIRCRAFT MODEL (simplified, generic trainer):
%    V_cruise = 15 m/s
%    u_max    = 4.0 m/s^2  (max lateral accel = g*tan(15 deg))
%    tau_act  = 0.25 s     (actuator/airframe bandwidth)
%
%  WIND MODEL:
%    Constant crosswind mean + AR(1) gusts
%    Crosswind acts directly on y_ct_dot (lateral drift velocity m/s)
%
%  wind_case options: 'low' (2 m/s), 'medium' (5 m/s), 'high' (8 m/s)
%
%  Returns struct sc with:
%    sc.V, sc.u_max, sc.tau_act, sc.dt, sc.N
%    sc.w_wind  - [Nx1] crosswind time series (m/s, positive = right of track)
%    sc.wind_mean - mean wind speed (m/s)

rng(seed);

switch lower(wind_case)
	case 'low';    w_mean = 2.0;   w_gust_sigma = 0.5;   w_tau = 1.0;
	case 'medium'; w_mean = 5.0;   w_gust_sigma = 1.5;   w_tau = 0.8;
	case 'high';   w_mean = 8.0;   w_gust_sigma = 3.0;   w_tau = 0.6;
	otherwise;     error('Unknown wind_case: %s', wind_case);
end

N = round(t_end / dt);

% AR(1) gust component
a = exp(-dt / w_tau);
b = w_gust_sigma * sqrt(1 - a^2);
gust = zeros(N, 1);
for k = 2:N
	gust(k) = a * gust(k-1) + b * randn();
end

sc.V         = 15.0;
sc.u_max     = 4.0;
sc.tau_act   = 0.25;
sc.dt        = dt;
sc.N         = N;
sc.w_wind    = w_mean + gust;    % total crosswind (m/s)
sc.wind_mean = w_mean;
sc.wind_case = wind_case;
end
