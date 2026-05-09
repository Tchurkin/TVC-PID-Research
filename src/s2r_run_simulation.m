function run = s2r_run_simulation(cfg, launch_case, controller_name, mismatch)
% Simulate pitch-plane TVC dynamics using piecewise ODE45 integration.

if nargin < 4
    mismatch = struct();
end
model = apply_mismatch(cfg, launch_case, mismatch);

t = (0:cfg.sim.dt:cfg.sim.t_max)';
n = numel(t);

theta = zeros(n, 1);
q = zeros(n, 1);
vel = zeros(n, 1);
alt = zeros(n, 1);
delta = zeros(n, 1);
delta_dot = zeros(n, 1);
delta_cmd = zeros(n, 1);

theta(1) = deg2rad(launch_case.theta0_deg);
q(1) = deg2rad(launch_case.q0_deg_s);
vel(1) = max(launch_case.v0_mps, 1.0);
alt(1) = launch_case.h0_m;

z1 = theta(1);
z2 = q(1);
z3 = 0;
dhat = zeros(n, 1);

delay_steps = max(1, round(model.act.delay_s / cfg.sim.dt));
cmd_buffer = zeros(delay_steps, 1);

u_prev = 0;
delta_lim = deg2rad(cfg.vehicle.max_gimbal_deg);
for k = 1:n - 1
    tk = t(k);

    [mass_k, iyy_k] = mass_inertia(model, tk);
    thrust_k = thrust_interp(model, tk);

    y_theta = theta(k) + deg2rad(model.sensor.theta_noise_deg) * randn();
    y_q = q(k) + deg2rad(model.sensor.q_noise_deg_s) * randn();

    switch upper(controller_name)
        case 'PID'
            e = y_theta - deg2rad(cfg.sim.ref_theta_deg);
            u = -model.pid.kp * e - model.pid.kd * y_q;
        case 'ADRC'
            [u, z1, z2, z3] = adrc_control(model, y_theta, z1, z2, z3, u_prev, cfg.sim.dt);
            dhat(k) = z3;
        otherwise
            error('Unsupported controller: %s', controller_name);
    end

    u = max(min(u, delta_lim), -delta_lim);
    delta_cmd(k) = u;
    cmd_buffer = [u; cmd_buffer(1:end-1)]; %#ok<AGROW>
    u_delayed = cmd_buffer(end);

    xk = [theta(k); q(k); vel(k); alt(k); delta(k); delta_dot(k)];
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [~, xsol] = ode45(@(tt, xx) pitch_dynamics(tt, xx, model, u_delayed, mass_k, iyy_k, thrust_k), [tk, t(k+1)], xk, opts);
    xnext = xsol(end, :)';

    theta(k+1) = clip(xnext(1), deg2rad(85));
    q(k+1) = clip(xnext(2), deg2rad(900));
    vel(k+1) = max(xnext(3), 0.1);
    alt(k+1) = max(xnext(4), 0.0);
    delta(k+1) = clip(xnext(5), delta_lim);
    delta_dot(k+1) = clip(xnext(6), deg2rad(model.act.slew_deg_s));

    u_prev = u;
end

delta_cmd(end) = delta_cmd(end-1);
dhat(end) = dhat(end-1);

run.t = t;
run.theta_rad = theta;
run.q_rad_s = q;
run.v_mps = vel;
run.h_m = alt;
run.delta_cmd_rad = delta_cmd;
run.delta_act_rad = delta;
run.disturbance_est = dhat;
run.controller = upper(string(controller_name));
run.mismatch   = mismatch;
run.diag       = compute_run_diagnostics(run, model);

end

function [u, z1, z2, z3] = adrc_control(model, y_theta, z1, z2, z3, u_prev, dt)
e = z1 - y_theta;
z1dot = z2 - model.adrc.beta1 * e;
z2dot = z3 - model.adrc.beta2 * e + model.adrc.b0 * u_prev;
z3dot = -model.adrc.beta3 * e;

z1 = z1 + z1dot * dt;
z2 = z2 + z2dot * dt;
z3 = z3 + z3dot * dt;

v = -model.adrc.kp * z1 - model.adrc.kd * z2;
u = (v - z3) / max(model.adrc.b0, 1e-6);
end

function dx = pitch_dynamics(t, x, model, u_delayed, mass, iyy, thrust)
% States: [theta, q, v, h, delta, delta_dot]
theta = x(1);
q = x(2);
v = max(x(3), 0.1);
h = x(4);
delta = x(5);
delta_dot = x(6);

alpha_deg = rad2deg(theta);
re = model.vehicle.rho * v * model.vehicle.d_ref_m / model.vehicle.mu_air;
[cd, cm] = aero_lookup(model, alpha_deg, re);

qbar = 0.5 * model.vehicle.rho * v^2;
drag = qbar * model.vehicle.S_ref_m2 * cd;

aero_m = qbar * model.vehicle.S_ref_m2 * model.vehicle.L_ref_m * ...
    (cm - model.aero.Cmq * q * model.vehicle.L_ref_m / max(2 * v, 1e-3));

misalign = deg2rad(model.mismatch.thrust_misalignment_deg);
mtvc = thrust * model.vehicle.lever_arm_m * sin(delta + misalign);

gust = model.mismatch.wind_torque_nm * sin(2 * pi * 3.0 * t);

theta_dot = q;
q_dot = (mtvc + aero_m + gust) / max(iyy, 1e-6);

v_dot = (thrust * cos(delta + misalign) - drag) / max(mass, 1e-6) - model.vehicle.g0;
h_dot = v;

% 2nd-order actuator + deadband + slew-rate limit
cmd_err = u_delayed - delta;
if abs(cmd_err) < deg2rad(model.act.deadband_deg)
    u_eff = delta;
else
    u_eff = u_delayed - sign(cmd_err) * deg2rad(model.act.deadband_deg);
end

delta_ddot = model.act.wn_rad_s^2 * (u_eff - delta) - 2 * model.act.zeta * model.act.wn_rad_s * delta_dot;

slew_lim = deg2rad(model.act.slew_deg_s);
if abs(delta_dot) >= slew_lim && sign(delta_ddot) == sign(delta_dot)
    delta_ddot = 0;
end

dx = [theta_dot; q_dot; v_dot; h_dot; delta_dot; delta_ddot];
end

function model = apply_mismatch(cfg, launch_case, mismatch)
model.vehicle = cfg.vehicle;
model.aero = cfg.aero;
model.pid = cfg.pid;
model.act = cfg.actuator;
model.sensor = cfg.sensor;

model.adrc.kp = cfg.adrc.kp;
model.adrc.kd = cfg.adrc.kd;
omega_o = cfg.adrc.omega_o;
model.adrc.beta1 = 3 * omega_o;
model.adrc.beta2 = 3 * omega_o^2;
model.adrc.beta3 = omega_o^3;
model.adrc.b0 = cfg.motor.thrust_nominal_N * cfg.vehicle.lever_arm_m / cfg.vehicle.Iyy_wet;

model.motor.t = cfg.motor.thrust_curve_t;
model.motor.thrust = cfg.motor.thrust_curve_N * launch_case.motor_scale;

model.mismatch.wind_torque_nm = 0.0;
model.mismatch.thrust_misalignment_deg = 0.0;

% Apply each mismatch source only if the FidelityConfig toggle permits it.
% This lets incremental fidelity analysis selectively enable/disable sources.
fc = cfg.fidelity_config;

if isfield(mismatch, 'actuator_delay_s') && fc.use_actuator_delay
    model.act.delay_s = cfg.actuator.delay_s + mismatch.actuator_delay_s;
else
    model.act.delay_s = cfg.actuator.delay_s;
end
if isfield(mismatch, 'servo_slew_delta_deg_s') && fc.use_servo_slew
    model.act.slew_deg_s = max(5, cfg.actuator.slew_deg_s + mismatch.servo_slew_delta_deg_s);
else
    model.act.slew_deg_s = cfg.actuator.slew_deg_s;
end
if isfield(mismatch, 'deadband_delta_deg') && fc.use_deadband
    model.act.deadband_deg = max(0, cfg.actuator.deadband_deg + mismatch.deadband_delta_deg);
else
    model.act.deadband_deg = cfg.actuator.deadband_deg;
end
if isfield(mismatch, 'sensor_noise_scale') && fc.use_sensor_noise
    model.sensor.theta_noise_deg = cfg.sensor.theta_noise_deg * max(0.05, mismatch.sensor_noise_scale);
    model.sensor.q_noise_deg_s   = cfg.sensor.q_noise_deg_s   * max(0.05, mismatch.sensor_noise_scale);
end
if isfield(mismatch, 'inertia_scale') && fc.use_inertia
    model.vehicle.Iyy_wet = cfg.vehicle.Iyy_wet * mismatch.inertia_scale;
    model.vehicle.Iyy_dry = cfg.vehicle.Iyy_dry * mismatch.inertia_scale;
end
if isfield(mismatch, 'aero_scale') && fc.use_aero_coeff
    model.aero.Cd_table = cfg.aero.Cd_table * mismatch.aero_scale;
    model.aero.Cm_table = cfg.aero.Cm_table * mismatch.aero_scale;
end
if isfield(mismatch, 'thrust_misalignment_deg') && fc.use_thrust_misalignment
    model.mismatch.thrust_misalignment_deg = mismatch.thrust_misalignment_deg;
end
if isfield(mismatch, 'wind_torque_nm') && fc.use_wind_torque
    model.mismatch.wind_torque_nm = mismatch.wind_torque_nm;
end

end

function [mass, iyy] = mass_inertia(model, t)
frac = max(0, 1 - t / model.vehicle.burn_time_s);
mass = model.vehicle.mass_dry_kg + (model.vehicle.mass_wet_kg - model.vehicle.mass_dry_kg) * frac;
iyy = model.vehicle.Iyy_dry + (model.vehicle.Iyy_wet - model.vehicle.Iyy_dry) * frac;
end

function thrust = thrust_interp(model, t)
thrust = interp1(model.motor.t, model.motor.thrust, t, 'pchip', 0);
end

function [cd, cm] = aero_lookup(model, alpha_deg, re)
alpha = max(min(alpha_deg, max(model.aero.alpha_grid_deg)), min(model.aero.alpha_grid_deg));
re_c = max(min(re, max(model.aero.re_grid)), min(model.aero.re_grid));
cd = interp2(model.aero.re_grid, model.aero.alpha_grid_deg, model.aero.Cd_table, re_c, alpha, 'linear');
cm = interp2(model.aero.re_grid, model.aero.alpha_grid_deg, model.aero.Cm_table, re_c, alpha, 'linear');
end

function y = clip(x, lim)
y = max(min(x, lim), -lim);
end

function diag = compute_run_diagnostics(run, model)
% Recompute per-timestep torque components from saved state trajectories.
% Provides: TVC control torque, aero restoring torque, aero damping torque,
% and gust disturbance torque at every saved timestep.
n = numel(run.t);
tau_tvc      = zeros(n, 1);
tau_aero_res = zeros(n, 1);
tau_aero_dmp = zeros(n, 1);
tau_gust     = zeros(n, 1);

for k = 1:n
    tk  = run.t(k);
    v_k = max(run.v_mps(k), 0.1);
    q_k = run.q_rad_s(k);
    delta_k = run.delta_act_rad(k);

    thrust_k = thrust_interp(model, tk);
    misalign = deg2rad(model.mismatch.thrust_misalignment_deg);
    tau_tvc(k) = thrust_k * model.vehicle.lever_arm_m * sin(delta_k + misalign);

    alpha_deg = rad2deg(run.theta_rad(k));
    re = model.vehicle.rho * v_k * model.vehicle.d_ref_m / model.vehicle.mu_air;
    [~, cm] = aero_lookup(model, alpha_deg, re);
    qbar = 0.5 * model.vehicle.rho * v_k^2;
    S = model.vehicle.S_ref_m2;
    L = model.vehicle.L_ref_m;

    tau_aero_res(k) = qbar * S * L * cm;
    tau_aero_dmp(k) = -qbar * S * L * model.aero.Cmq * q_k * L / max(2 * v_k, 1e-3);
    tau_gust(k)     = model.mismatch.wind_torque_nm * sin(2 * pi * 3.0 * tk);
end

diag.tau_tvc_nm          = tau_tvc;
diag.tau_aero_restoring_nm = tau_aero_res;
diag.tau_aero_damping_nm   = tau_aero_dmp;
diag.tau_gust_nm           = tau_gust;
diag.tau_aero_nm           = tau_aero_res + tau_aero_dmp;
end
