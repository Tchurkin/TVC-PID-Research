function cfg = rocket_defaults()
%ROCKET_DEFAULTS  Minimal plant and controller config for keff instability demo.
%
%  Routh-Hurwitz stability for 3rd-order (plant + actuator lag, tau_act=0.30s):
%    stable when:  (aero_damp + 1/tau_act) * (aero_damp + keff*K2) > keff*K1
%                  boundary constant = (1.20 + 3.33) * 1.20 = 5.44
%
%  FIXED_PID (K2=0, Kp=0.44):  Nominal keff=8:  8*0.44=3.52 < 5.44  STABLE
%                               Fault   keff=16: 16*0.44=7.04 > 5.44  UNSTABLE
%  FIXED_LQR (K=[0.50,0.02]):  Nominal keff=8:  Routh=6.52 > 8*0.50=4.00  STABLE
%                               Fault   keff=16: Routh=6.89 < 16*0.50=8.00 UNSTABLE
%  ADAPTIVE  (K_nom=[1.58,0.53],gain_scale=8/16=0.5):
%            K_eff=[0.79,0.265]: Routh=24.6 > 16*0.79=12.6  STABLE

cfg.dt         = 0.005;
cfg.t_end_demo = 12.0;

% Plant (simulation truth model)
cfg.plant.theta0      = 0;
cfg.plant.q0          = 0.0;
cfg.plant.u_act0      = 0.0;
cfg.plant.aero_damp   = 1.20;
cfg.plant.p_unstable  = 0.0;     % static-instability natural frequency (rad/s); 0 = stable plant
cfg.plant.control_eff = 8.0;
cfg.plant.tau_act     = 0.05;    % 50 ms first-order actuator lag
% slew_max in code units. u_max=12 corresponds to max physical gimbal deflection.
% Current nominal stack targets hobby-relevant loaded gimbal slew, not the
% old severe-collapse placeholder. A typical 9 g-class micro servo is about
% 350-600 deg/s bare; with 4:1 linkage and 50-70% load derate this implies
% roughly 45-105 deg/s at the gimbal. We use 75 deg/s as the current nominal.
% In code units with +/-15 deg physical gimbal range and +/-12 command units,
% 75 deg/s maps to 75 * (12/15) = 60 units/s.
% Legacy severe-collapse/controller probe scripts explicitly override this to
% slower values when they want the old stress case.
cfg.plant.slew_max    = 60.0;    % nominal current target: 75 deg/s loaded gimbal rate
cfg.plant.u_max       = 12.0;
cfg.plant.keff_nom    = cfg.plant.control_eff;

cfg.thresholds.fail_theta_rad   = deg2rad(80);
cfg.thresholds.inband_theta_rad = deg2rad(2.0);

% FIXED_LQR: 2-state state-feedback designed via LQR on the augmented
% (plant + 50 ms actuator-lag) model with Q = diag(200, 1, 0.01), R = 0.5.
% K_uact term dropped (only theta and q fed back) -- standard simplification.
% Note: optimal LQR on this 2-state plant is mathematically identical
% to ideal PD; a separate FIXED_PID controller would add no information.
flqr.K         = [20.00, 3.14];
flqr.K_nominal = [20.00, 3.14];
flqr.keff_nom  = cfg.plant.keff_nom;
flqr.u_max     = cfg.plant.u_max;
cfg.controllers.FIXED_LQR = flqr;

% ADAPTIVE_KEFF_LQR: SAME nominal K as FIXED_LQR (identical pre-fault behavior),
% only differs by online keff identification and gain rescaling at runtime.
aklqr.K_nominal     = flqr.K_nominal;
aklqr.keff_nom      = cfg.plant.keff_nom;
aklqr.u_max         = cfg.plant.u_max;
aklqr.lambda_rls    = 0.97;
aklqr.adapt_guard_s = 0.50;
aklqr.keff_min      = 0.15 * cfg.plant.keff_nom;
aklqr.keff_max      = 5.50 * cfg.plant.keff_nom;
aklqr.delta_min     = 0.10;
aklqr.keff_beta     = 0.10;
aklqr.aero_damp     = cfg.plant.aero_damp;
aklqr.alpha_beta    = 0.25;
aklqr.min_scale     = 0.10;
aklqr.max_scale     = 4.00;
aklqr.id_dither_amp = 0.0;
aklqr.id_dither_hz  = 3.5;
cfg.controllers.ADAPTIVE_KEFF_LQR = aklqr;

% JOINT_ADAPTIVE: estimates keff AND slew envelope, scales gains by both.
% Adds bandwidth-matching scale (slew_est/slew_nominal) to existing keff scaling.
% Robust to slew degradation faults that fool keff-only adaptation.
jad = aklqr;                        % inherit all keff-adaptive settings
jad.tau_act_assumed = cfg.plant.tau_act;
jad.slew_nominal    = cfg.plant.slew_max;
jad.slew_min        = 0.10 * cfg.plant.slew_max;
jad.slew_alpha_sat  = 0.05;         % rate at which slew_est is pulled down when saturating
jad.slew_alpha_relax= 0.01;         % relaxation back to nominal when not saturating (~0.5 s tau)
jad.slew_scale_min  = 0.05;         % floor on gain scaling (avoid total shutoff)
jad.slew_health_keff_freeze = 0.70; % freeze keff scaling whenever slew envelope is materially degraded
jad.joint_min_scale = 0.60;         % tighten keff scaling range for robust realistic runs
jad.joint_max_scale = 1.40;
jad.keff_disable_theta_rad = deg2rad(10); % latch keff scaling off after large excursion
jad.sat_streak_min  = 10;           % consecutive saturated samples needed to commit (~50 ms)
jad.sat_noise_floor = 0.5;          % units/s additive margin on saturation gate (rejects sensor-noise false trips)
jad.sat_decay       = 0.10;         % asymmetric decay: gate counter drops slowly when not saturating
jad.slew_detect_pos_sat_frac = 0.92; % suppress false slew detection when actuator is position-limited
jad.gate_disturb_gain = 0.00;       % conservative gate avoids disturbance-driven false inferences
jad.disturb_alpha     = 0.03;       % LP smoothing for residual/disturbance proxy
jad.conf_min          = 0.25;       % retain adaptation but downweight low-confidence updates
jad.conf_resid_gain   = 4.0;        % penalize adaptation when model residual inflates
jad.conf_floor_blend  = 0.30;       % confidence has material effect on keff update rate
jad.safety_cmd_slew_frac = 10.0;    % moderate command shield for realistic nonlinearity protection
jad.theta_guard_rad      = deg2rad(70); % extreme-attitude guard threshold
cfg.controllers.JOINT_ADAPTIVE = jad;

% SLEW_ADAPTIVE: true single-channel comparator for JOINT.
% Uses the same slew-envelope identification + bandwidth scaling path,
% but keeps keff adaptation disabled (keff_scale = 1 always).
sad = flqr;
sad.K_nominal = flqr.K_nominal;
sad.u_max = cfg.plant.u_max;
sad.tau_act_assumed = cfg.plant.tau_act;
sad.slew_nominal = cfg.plant.slew_max;
sad.slew_min = 0.10 * cfg.plant.slew_max;
sad.slew_alpha_sat = 0.05;
sad.slew_alpha_relax = 0.01;
sad.slew_scale_min = 0.05;
sad.sat_streak_min = 10;
sad.sat_noise_floor = 0.5;
sad.sat_decay = 0.10;
sad.safety_cmd_slew_frac = 20.0;
sad.theta_guard_rad = deg2rad(70);
cfg.controllers.SLEW_ADAPTIVE = sad;

% SIGMA_MRAC: gradient adaptation of keff with sigma-mod leakage. The
% canonical *correct* single-parameter MRAC baseline -- not the bare-RLS
% strawman. References: Ioannou & Sun (1996); Narendra & Annaswamy (1989).
%   gamma_grad : adaptation rate (units 1/(u^2 * s))
%   sigma_mod  : leakage rate (1/s); pulls keff_est back to keff_nom when
%                regressor is non-PE (incl. saturation).
smrac = aklqr;                       % share LQR gain, scaling clamps, etc.
smrac.gamma_grad = 1.5;              % moderate adaptation rate
smrac.sigma_mod  = 0.5;              % leakage time-constant ~2 s
cfg.controllers.SIGMA_MRAC = smrac;

% PCH_LQR: LQR with Pseudo-Control Hedging (Johnson & Calise, 2000-2003).
% This is the controller a competent GNC engineer would reach for first.
% Knows the (assumed) actuator envelope and clips its own command rate to
% match. Adaptation gated off during hedging.
%   slew_max_assumed : the value PCH believes -- nominal, not adapted.
pch = aklqr;
pch.tau_act_assumed = cfg.plant.tau_act;
pch.slew_max_assumed = cfg.plant.slew_max;   % nominal value; PCH does NOT update
pch.gamma_grad      = 1.5;
pch.sigma_mod       = 0.5;
cfg.controllers.PCH_LQR = pch;

% PID_RG: PID inner loop + Scalar Reference Governor on the actuator command.
% The RG predicts the next H steps of the augmented plant state under the
% assumption that the governed command is held constant, and gates the PID
% output so |theta| and |q| stay within safety bounds. Plant model used by
% the predictor is parameterized from bench measurements (slew, tau_act)
% plus a preflight estimate of the unstable pole p. See pid_rg_layer.m.
prg = struct();
prg.Kp    = 1.0;                       % defaults; firmware audit overrides
prg.Ki    = 0.0;
prg.Kd    = 2.0;
prg.u_max = cfg.plant.u_max;
prg.i_lim = cfg.plant.u_max;
prg.rg.H                  = 10;        % horizon (steps)
prg.rg.theta_safety       = deg2rad(15);
prg.rg.q_safety           = 5.0;       % rad/s
prg.rg.keff_assumed       = cfg.plant.keff_nom;
prg.rg.aero_damp_assumed  = cfg.plant.aero_damp;
prg.rg.p_assumed          = cfg.plant.p_unstable;
prg.rg.tau_act_assumed    = cfg.plant.tau_act;
prg.rg.bisect_iters       = 8;
cfg.controllers.PID_RG = prg;

% PID_SLEW_AWARE: PID backbone with online slew-envelope estimation.
% Key idea: when actuator bandwidth collapses, reduce proportional
% aggressiveness harder than derivative damping. This keeps damping alive
% while avoiding the command pile-up that drives rate-limited divergence.
psa = struct();
psa.Kp    = 20.0;
psa.Ki    = 0.0;
psa.Kd    = 8.0;
psa.u_max = cfg.plant.u_max;
psa.i_lim = cfg.plant.u_max;
psa.tau_act_assumed = cfg.plant.tau_act;
psa.slew_nominal    = cfg.plant.slew_max;
psa.slew_min        = 0.10 * cfg.plant.slew_max;
psa.slew_alpha_sat  = 0.05;
psa.slew_alpha_relax= 0.01;
psa.sat_streak_min  = 10;
psa.sat_noise_floor = 0.5;
psa.sat_decay       = 0.10;
psa.kp_scale_min    = 0.50;
psa.kp_scale_power  = 1.0;
psa.kd_scale_min    = 0.70;
psa.kd_scale_power  = 0.50;
psa.integrator_bleed = 0.95;
psa.safety_cmd_slew_frac = inf;    % disable extra shield; slew estimator + gain schedule already shape demand
psa.theta_guard_rad      = deg2rad(70);
cfg.controllers.PID_SLEW_AWARE = psa;
end
