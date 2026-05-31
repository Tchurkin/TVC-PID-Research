function sc = rocket_scenario(kind, cfg)
%ROCKET_SCENARIO  Scenario parameters.

if nargin < 2, cfg = rocket_defaults(); end

sc.kind             = upper(string(kind));
sc.fault_time       = inf;
sc.t_end            = cfg.t_end_demo;
sc.disturbance_amp  = 0.0;
sc.disturbance_const = 0.0;
sc.disturbance_freq_hz = 0.8;
sc.startup_ramp_s           = 0.0;
sc.tau_scale_post          = 1.0;
sc.control_eff_scale_post  = 1.0;
sc.slew_scale_post         = 1.0;
sc.aero_damp_scale_post    = 1.0;
sc.disturb_scale_post      = 1.0;
sc.disturb_bias_post       = 0.0;

switch sc.kind
    case "NOMINAL"
        % No fault; small disturbance to verify all controllers stabilise.
        sc.disturbance_amp = 0.05;

    case "HIGH_KEFF_FAULT"
        % Motor authority decay: keff drops to 30% at fault_time.
        % Calibrated for realistic slew_max=12 (0.25 rad/s gimbal).
        % Pre-fault: light wind, all controllers handle cleanly.
        % Post-fault: fixed controller cannot keep up; adaptive identifies
        % the keff drop and increases gains while staying within slew envelope.
        sc.fault_time                = 4.0;
        sc.control_eff_scale_post    = 0.30;   % keff: 8 -> 2.4 (moderate motor decay)
        sc.disturbance_amp           = 0.30;
        sc.disturb_scale_post        = 4.0;    % gust amplifies to 1.2 amp at fault
        sc.disturbance_freq_hz       = 0.30;

    case "SLEW_DEGRADATION"
        % Servo slew rate degrades mid-flight (battery sag, hot servo, mechanical wear).
        % Pre-fault: nominal gimbal rate 0.25 rad/s (slew_max=12 code units).
        % Post-fault: gimbal rate drops to ~0.06 rad/s (slew_max=3 code units, 25%).
        % Sustained light wind throughout; small ICs so nominal is stable.
        % Expected behavior:
        %   FIXED_LQR          -> demands more slew than servo can deliver, limit-cycles
        %   ADAPTIVE_KEFF_LQR  -> misidentifies slew lag as keff drop, raises gains,
        %                         makes saturation worse (the 'naive adaptation hurts' plot)
        %   JOINT_ADAPTIVE     -> identifies slew envelope, rate-limits reference, settles
        sc.fault_time                = 4.0;
        sc.slew_scale_post           = 0.25;   % 0.25 rad/s -> 0.0625 rad/s gimbal
        sc.disturbance_amp           = 0.40;   % steady wind to keep loop excited
        sc.disturbance_freq_hz       = 0.40;

    otherwise
        error('rocket_scenario:UnknownKind', 'Unknown scenario: %s', sc.kind);
end
end
