function [cfg, sc, realism] = build_realistic_cfg(P, override)
%BUILD_REALISTIC_CFG Build simulate_case_realistic inputs from shared params.

if nargin < 2
    override = struct();
end

cfg = rocket_defaults();
P_ref = default_rocket_params();

cfg.t_end_demo = pick_value(override, 't_end', P.analysis.t_end);
cfg.plant.aero_damp = pick_value(override, 'aero_damp', 0.5);
cfg.plant.control_eff = pick_value(override, 'control_effectiveness', P.rocket.control_effectiveness);
cfg.plant.keff_nom = cfg.plant.control_eff;
cfg.plant.p_unstable = pick_value(override, 'p_unstable', estimate_p_unstable(P));
cfg.plant.tau_act = pick_value(override, 'tau_act', 0.05);
cfg.plant.theta0 = 0;

% Keep physical design variables in runtime cfg so they can influence dynamics
% in a physically interpretable way (moment/inertia scaling + aerodynamic geometry).
cfg.plant.mass = pick_value(override, 'mass', P.rocket.mass);
cfg.plant.Iyy = pick_value(override, 'Iyy', P.rocket.Iyy);
cfg.plant.thrust = pick_value(override, 'thrust', P.rocket.thrust);
cfg.plant.static_margin = pick_value(override, 'static_margin', P.rocket.static_margin);
cfg.plant.Cm_alpha = pick_value(override, 'Cm_alpha', P.rocket.Cm_alpha);

cfg.plant.mass_scale = max(0.30, min(3.00, cfg.plant.mass / max(1e-6, P_ref.rocket.mass)));
cfg.plant.inertia_scale = max(0.20, min(5.00, P_ref.rocket.Iyy / max(1e-6, cfg.plant.Iyy)));
cfg.plant.thrust_scale = max(0.20, min(5.00, cfg.plant.thrust / max(1e-6, P_ref.rocket.thrust)));
cfg.plant.p_geom = estimate_p_unstable(P);
cfg.plant.p_geom_ref = estimate_p_unstable(P_ref);
cfg.plant.geom_delta_gain = pick_value(override, 'geom_delta_gain', 0.35);

max_gimbal_deg = max(1e-3, P.rocket.max_gimbal);
code_scale = 12.0 / max_gimbal_deg;

slew_deg_s = pick_value(override, 'servo_slew', P.rocket.servo_slew);
cfg.plant.slew_max = deg2rad(slew_deg_s) * code_scale;

u_frac = pick_value(override, 'u_max_frac', 1.0);
u_frac = max(0.05, min(1.0, u_frac));
cfg.plant.u_max = 12.0 * u_frac;

cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;

sc = rocket_scenario('nominal', cfg);
sc.t_end = cfg.t_end_demo;
sc.disturbance_amp = pick_value(override, 'disturbance_amp', P.analysis.disturbance_amp);
sc.disturbance_freq_hz = pick_value(override, 'disturbance_freq_hz', P.analysis.disturbance_freq_hz);

realism = struct();
realism.sensor_latency_steps = round(max(1, pick_value(override, 'latency', P.rocket.latency)));
realism.servo_deadband = pick_value(override, 'deadband', P.rocket.deadband);
realism.servo_backlash = pick_value(override, 'backlash', P.rocket.backlash);
realism.gust_std = pick_value(override, 'wind_strength', P.rocket.wind_strength);
realism.gust_tau = pick_value(override, 'gust_tau', 0.40);


function val = pick_value(S, key, fallback)
if isfield(S, key)
    val = S.(key);
else
    val = fallback;
end


function p = estimate_p_unstable(P)
if isfield(P, 'analysis') && isfield(P.analysis, 'p_unstable')
    p = P.analysis.p_unstable;
    return;
end

% Proxy from static margin, Cm_alpha, and thrust loading.
thrust = max(1e-6, P.rocket.thrust);
p = max(0.0, 10.0 * P.rocket.static_margin * abs(P.rocket.Cm_alpha) / 52.0 * sqrt(35.0 / thrust));
