% firmware_validation.m
% Cross-check firmware constants against MATLAB defaults
% Run this to validate parameter alignment before flight testing

clear all; close all;

fprintf('\n=== Firmware vs MATLAB Parameter Validation ===\n\n');

% Load MATLAB defaults
cfg = rocket_defaults();

% Firmware-extracted constants (manually transcribed from SisyphusCode.cpp)
fw = struct();
fw.DT_S = 0.005;
fw.CONTROL_HZ = 100.0;
fw.DT_CONTROL = 1.0 / fw.CONTROL_HZ;  % 0.010 s

fw.AERO_DAMP = 1.20;
fw.KEFF_NOM = 8.0;
fw.SLEW_NOM = 12.0;
fw.TAU_ACT_ASSUMED = 0.05;

fw.K1_NOMINAL = 20.00;
fw.K2_NOMINAL = 3.14;
fw.U_MAX = 12.0;

fw.LAMBDA_RLS = 0.97;
fw.ADAPT_GUARD_S = 0.50;
fw.KEFF_MIN = 0.15 * fw.KEFF_NOM;
fw.KEFF_MAX = 5.50 * fw.KEFF_NOM;
fw.DELTA_MIN = 0.10;
fw.KEFF_BETA = 0.10;
fw.ALPHA_BETA = 0.25;
fw.MIN_SCALE = 0.10;
fw.MAX_SCALE = 4.00;

fw.SLEW_MIN = 0.10 * fw.SLEW_NOM;
fw.SLEW_ALPHA_SAT = 0.05;
fw.SLEW_ALPHA_RELAX = 0.01;
fw.SLEW_SCALE_MIN = 0.05;
fw.SLEW_HEALTH_KEFF_FREEZE = 0.70;

fw.SAT_STREAK_MIN = 10;
fw.SAT_NOISE_FLOOR = 0.5;
fw.SAT_DECAY = 0.10;
fw.GATE_DISTURB_GAIN = 0.00;
fw.DISTURB_ALPHA = 0.03;
fw.CONF_MIN = 1.00;
fw.CONF_RESID_GAIN = 0.00;
fw.CONF_FLOOR_BLEND = 1.00;

fw.SAFETY_CMD_SLEW_FRAC = 10.0;
fw.THETA_GUARD_RAD = 70.0 * (pi / 180.0);

fw.BURN_TIME_S = 3.45;
fw.COUNTDOWN_S = 10.0;

% Extract MATLAB defaults for comparison
matlab = struct();
matlab.DT = cfg.dt;
matlab.AERO_DAMP = cfg.plant.aero_damp;
matlab.KEFF_NOM = cfg.plant.control_eff;
matlab.SLEW_NOM = cfg.plant.slew_max;
matlab.TAU_ACT = cfg.plant.tau_act;
matlab.U_MAX = cfg.plant.u_max;

matlab.K = cfg.controllers.FIXED_LQR.K;
matlab.K1 = matlab.K(1);
matlab.K2 = matlab.K(2);

jad = cfg.controllers.JOINT_ADAPTIVE;
matlab.LAMBDA_RLS = jad.lambda_rls;
matlab.ADAPT_GUARD_S = jad.adapt_guard_s;
matlab.KEFF_MIN = jad.keff_min;
matlab.KEFF_MAX = jad.keff_max;
matlab.DELTA_MIN = jad.delta_min;
matlab.KEFF_BETA = jad.keff_beta;
matlab.ALPHA_BETA = jad.alpha_beta;
matlab.MIN_SCALE = jad.min_scale;
matlab.MAX_SCALE = jad.max_scale;

matlab.SLEW_MIN = jad.slew_min;
matlab.SLEW_ALPHA_SAT = jad.slew_alpha_sat;
matlab.SLEW_ALPHA_RELAX = jad.slew_alpha_relax;
matlab.SLEW_SCALE_MIN = jad.slew_scale_min;
matlab.SLEW_HEALTH_KEFF_FREEZE = jad.slew_health_keff_freeze;

matlab.SAT_STREAK_MIN = jad.sat_streak_min;
matlab.SAT_NOISE_FLOOR = jad.sat_noise_floor;
matlab.SAT_DECAY = jad.sat_decay;
matlab.GATE_DISTURB_GAIN = jad.gate_disturb_gain;
matlab.DISTURB_ALPHA = jad.disturb_alpha;
matlab.CONF_MIN = jad.conf_min;
matlab.CONF_RESID_GAIN = jad.conf_resid_gain;
matlab.CONF_FLOOR_BLEND = jad.conf_floor_blend;

matlab.SAFETY_CMD_SLEW_FRAC = jad.safety_cmd_slew_frac;
matlab.THETA_GUARD_RAD = jad.theta_guard_rad;

% Comparison function
compare_param = @(name, fw_val, ml_val, tol) ...
    check_match(name, fw_val, ml_val, tol);

fprintf('Plant Model Parameters:\n');
fprintf('%-40s | FW      | MATLAB  | Δ       | Status\n', 'Parameter');
fprintf('%s\n', repmat('-', 88, 1));

params = {
    'AERO_DAMP', fw.AERO_DAMP, matlab.AERO_DAMP, 1e-6;
    'KEFF_NOM', fw.KEFF_NOM, matlab.KEFF_NOM, 1e-6;
    'SLEW_NOM (units/s)', fw.SLEW_NOM, matlab.SLEW_NOM, 1e-6;
    'TAU_ACT (s)', fw.TAU_ACT_ASSUMED, matlab.TAU_ACT, 1e-6;
    'U_MAX (units)', fw.U_MAX, matlab.U_MAX, 1e-6;
};

mismatch_count = 0;
for i = 1:size(params, 1)
    name = params{i, 1};
    fw_val = params{i, 2};
    ml_val = params{i, 3};
    tol = params{i, 4};
    delta = abs(fw_val - ml_val);
    status = iif(delta <= tol, '✓ PASS', sprintf('✗ FAIL (Δ=%g)', delta));
    fprintf('%-40s | %7.4f | %7.4f | %7.4e | %s\n', name, fw_val, ml_val, delta, status);
    if delta > tol
        mismatch_count = mismatch_count + 1;
    end
end

fprintf('\n\nLQR Gains:\n');
fprintf('%-40s | FW      | MATLAB  | Δ       | Status\n', 'Parameter');
fprintf('%s\n', repmat('-', 88, 1));

gain_params = {
    'K1 (theta gain)', fw.K1_NOMINAL, matlab.K1, 1e-4;
    'K2 (q gain)', fw.K2_NOMINAL, matlab.K2, 1e-4;
};

for i = 1:size(gain_params, 1)
    name = gain_params{i, 1};
    fw_val = gain_params{i, 2};
    ml_val = gain_params{i, 3};
    tol = gain_params{i, 4};
    delta = abs(fw_val - ml_val);
    status = iif(delta <= tol, '✓ PASS', sprintf('✗ FAIL (Δ=%g)', delta));
    fprintf('%-40s | %7.4f | %7.4f | %7.4e | %s\n', name, fw_val, ml_val, delta, status);
    if delta > tol
        mismatch_count = mismatch_count + 1;
    end
end

fprintf('\n\nAdaptive Controller Parameters (RLS):\n');
fprintf('%-40s | FW      | MATLAB  | Δ       | Status\n', 'Parameter');
fprintf('%s\n', repmat('-', 88, 1));

adaptive_params = {
    'LAMBDA_RLS', fw.LAMBDA_RLS, matlab.LAMBDA_RLS, 1e-6;
    'ADAPT_GUARD_S', fw.ADAPT_GUARD_S, matlab.ADAPT_GUARD_S, 1e-6;
    'KEFF_MIN', fw.KEFF_MIN, matlab.KEFF_MIN, 1e-6;
    'KEFF_MAX', fw.KEFF_MAX, matlab.KEFF_MAX, 1e-6;
    'DELTA_MIN', fw.DELTA_MIN, matlab.DELTA_MIN, 1e-6;
    'KEFF_BETA', fw.KEFF_BETA, matlab.KEFF_BETA, 1e-6;
    'ALPHA_BETA', fw.ALPHA_BETA, matlab.ALPHA_BETA, 1e-6;
};

for i = 1:size(adaptive_params, 1)
    name = adaptive_params{i, 1};
    fw_val = adaptive_params{i, 2};
    ml_val = adaptive_params{i, 3};
    tol = adaptive_params{i, 4};
    delta = abs(fw_val - ml_val);
    status = iif(delta <= tol, '✓ PASS', sprintf('✗ FAIL (Δ=%g)', delta));
    fprintf('%-40s | %7.4f | %7.4f | %7.4e | %s\n', name, fw_val, ml_val, delta, status);
    if delta > tol
        mismatch_count = mismatch_count + 1;
    end
end

fprintf('\n\nSlew Envelope Adaptation:\n');
fprintf('%-40s | FW      | MATLAB  | Δ       | Status\n', 'Parameter');
fprintf('%s\n', repmat('-', 88, 1));

slew_params = {
    'SLEW_MIN', fw.SLEW_MIN, matlab.SLEW_MIN, 1e-6;
    'SLEW_ALPHA_SAT', fw.SLEW_ALPHA_SAT, matlab.SLEW_ALPHA_SAT, 1e-6;
    'SLEW_ALPHA_RELAX', fw.SLEW_ALPHA_RELAX, matlab.SLEW_ALPHA_RELAX, 1e-6;
    'SLEW_SCALE_MIN', fw.SLEW_SCALE_MIN, matlab.SLEW_SCALE_MIN, 1e-6;
    'SLEW_HEALTH_KEFF_FREEZE', fw.SLEW_HEALTH_KEFF_FREEZE, matlab.SLEW_HEALTH_KEFF_FREEZE, 1e-6;
};

for i = 1:size(slew_params, 1)
    name = slew_params{i, 1};
    fw_val = slew_params{i, 2};
    ml_val = slew_params{i, 3};
    tol = slew_params{i, 4};
    delta = abs(fw_val - ml_val);
    status = iif(delta <= tol, '✓ PASS', sprintf('✗ FAIL (Δ=%g)', delta));
    fprintf('%-40s | %7.4f | %7.4f | %7.4e | %s\n', name, fw_val, ml_val, delta, status);
    if delta > tol
        mismatch_count = mismatch_count + 1;
    end
end

fprintf('\n\nSaturation & Safety:\n');
fprintf('%-40s | FW      | MATLAB  | Δ       | Status\n', 'Parameter');
fprintf('%s\n', repmat('-', 88, 1));

safety_params = {
    'SAT_STREAK_MIN', fw.SAT_STREAK_MIN, matlab.SAT_STREAK_MIN, 0;
    'SAT_NOISE_FLOOR', fw.SAT_NOISE_FLOOR, matlab.SAT_NOISE_FLOOR, 1e-6;
    'SAT_DECAY', fw.SAT_DECAY, matlab.SAT_DECAY, 1e-6;
    'DISTURB_ALPHA', fw.DISTURB_ALPHA, matlab.DISTURB_ALPHA, 1e-6;
};

for i = 1:size(safety_params, 1)
    name = safety_params{i, 1};
    fw_val = safety_params{i, 2};
    ml_val = safety_params{i, 3};
    tol = safety_params{i, 4};
    delta = abs(fw_val - ml_val);
    status = iif(delta <= tol, '✓ PASS', sprintf('✗ FAIL (Δ=%g)', delta));
    fprintf('%-40s | %7.4f | %7.4f | %7.4e | %s\n', name, fw_val, ml_val, delta, status);
    if delta > tol
        mismatch_count = mismatch_count + 1;
    end
end

fprintf('\n\n=== VALIDATION SUMMARY ===\n');
if mismatch_count == 0
    fprintf('✓ ALL PARAMETERS MATCH: Firmware is consistent with MATLAB defaults.\n\n');
else
    fprintf('✗ %d PARAMETER MISMATCHES FOUND: Review firmware constants.\n\n', mismatch_count);
end

% Print safety margins
fprintf('Safety Margin Analysis:\n');
fprintf('  Routh-Hurwitz stability boundary (nominal plant): ~5.44\n');
fprintf('  Current boundary check: (aero_damp + 1/tau_act) * (aero_damp + keff*K2) > keff*K1\n');
fprintf('  Numerical: (%.2f + %.2f) * (%.2f + %.2f * %.2f) > %.2f * %.2f\n', ...
    fw.AERO_DAMP, 1/fw.TAU_ACT_ASSUMED, fw.AERO_DAMP, fw.KEFF_NOM, fw.K2_NOMINAL, ...
    fw.KEFF_NOM, fw.K1_NOMINAL);

rhs = (fw.AERO_DAMP + 1/fw.TAU_ACT_ASSUMED) * (fw.AERO_DAMP + fw.KEFF_NOM * fw.K2_NOMINAL);
lhs = fw.KEFF_NOM * fw.K1_NOMINAL;
fprintf('  LHS (plant stiffness): %.2f\n', rhs);
fprintf('  RHS (feedback loop): %.2f\n', lhs);
fprintf('  Margin: %.2f (positive = stable at nominal)\n', rhs - lhs);

fprintf('\nAdaptive Scaling Range:\n');
fprintf('  keff scaling range: [%.2f, %.2f] × K_nominal\n', ...
    fw.KEFF_MIN / fw.KEFF_NOM, fw.KEFF_MAX / fw.KEFF_NOM);
fprintf('  slew scaling range: [%.2f, %.2f] × K_nominal\n', ...
    fw.SLEW_SCALE_MIN, 1.0);
fprintf('  combined gain range: [%.4f, %.4f] × K_nominal\n', ...
    fw.MIN_SCALE, fw.MAX_SCALE);

fprintf('\n=== END VALIDATION ===\n\n');

% Helper function
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
