function out = recommend_envelope(meas)
%RECOMMEND_ENVELOPE  Amateur-TVC pre-flight validator (v2, post-kill-test).
%
% Input struct `meas`:
%   meas.slew_deg_per_s    measured GIMBAL slew rate (deg/s, linkage applied)
%   meas.servo_max_deg     hard mechanical gimbal envelope (deg)
%   meas.p_est             estimated airframe instability rate (1/s)
%   meas.keff_est          control effectiveness (rad/s^2 per gimbal-rad) [default 8]
%   meas.damp_est          rate damping (1/s) [default 0.2]
%
% Returns:
%   out.region              'RESCUE' | 'FUNDAMENTAL' | 'INFEASIBLE'
%   out.u_max_recommended_deg
%   out.R_recommended       LQR R weight for the (u_max, p) pair
%   out.lqr_design_p        H2-conservative single-point design pole
%   out.K                   2x1 LQR gain on [theta(rad); q(rad/s)]
%   out.go_nogo             'GO' | 'MARGINAL' | 'NOGO'
%
% Built from experiments/HEADLINE_FINAL.md (post 4 kill-tests).
% Three-region phase diagram:
%   RESCUE      : high slew -> R*(u_max) tuning achieves >=95% success
%   FUNDAMENTAL : low slew  -> no fixed-R LQR design rescues; needs MPC/PCH
%   INFEASIBLE  : below absolute stabilizability; redesign required
%
% R*(u_max) lookup is from experiments/results/h1_bestk_kill.csv and
% experiments/results/realistic_h1_sweep.csv.

if ~isfield(meas, 'keff_est'), meas.keff_est = 8.0; end
if ~isfield(meas, 'damp_est'), meas.damp_est = 0.2; end

% --- Code-unit normalization -----------------------------------------
% Simple-sim study uses "code units" where one unit ~= 1/12 rad of gimbal
% deflection. Express both u_max and slew in the same code units.
gimbal_max_rad = deg2rad(meas.servo_max_deg);
code_scale = 12.0 / max(1e-3, gimbal_max_rad);
slew_code = deg2rad(meas.slew_deg_per_s) * code_scale;
umax_code_avail = 12.0;

% --- 1) Phase region classification ----------------------------------
% Empirical thresholds from realistic_h1_sweep at p_unstable=8:
%   slew_code>=35 -> RESCUE; slew_code=20 -> FUNDAMENTAL
% Scale linearly with p_est (gimbal must outrun the unstable mode).
slew_thresh_rescue      = 4.5 * meas.p_est;
slew_thresh_fundamental = 2.0 * meas.p_est;

if slew_code >= slew_thresh_rescue
    region = 'RESCUE';
elseif slew_code >= slew_thresh_fundamental
    region = 'FUNDAMENTAL';
else
    region = 'INFEASIBLE';
end

% --- 2) u_max recommendation -----------------------------------------
switch region
    case 'RESCUE'
        u_max_code = 0.85 * umax_code_avail;   % mechanical clamp
    case 'FUNDAMENTAL'
        u_max_code = 4.5;                       % empirical best-R peak
    case 'INFEASIBLE'
        u_max_code = 0;
end
u_max_rad = u_max_code / code_scale;
u_max_deg = rad2deg(u_max_rad);

% --- 3) R* lookup (from h1_bestk_kill at p_ref=10) -------------------
p_ref = 10;
p_scale = p_ref / max(1e-3, meas.p_est);
if u_max_code <= 2.5
    R_star = 0.25;
elseif u_max_code <= 4.5
    R_star = 0.50;
elseif u_max_code <= 8.5
    R_star = 4.0;
else
    R_star = 8.0;
end
R_star = R_star * p_scale;

% --- 4) LQR design (H2 conservative + R*) ----------------------------
lqr_design_p = meas.p_est;
K = design_lqr_unstable(lqr_design_p, meas.damp_est, meas.keff_est, ...
    diag([400 2]), R_star);

% --- 5) Verdict ------------------------------------------------------
required_slew_code = 2 * meas.p_est * u_max_code;
slew_margin = slew_code / max(1e-6, required_slew_code);
switch region
    case 'RESCUE',      verdict = 'GO';
    case 'FUNDAMENTAL', verdict = 'MARGINAL';
    case 'INFEASIBLE',  verdict = 'NOGO';
end

notes = sprintf([ ...
    'Region: %s\n' ...
    '  slew_code = %.2f  thresh_rescue = %.2f  thresh_fundamental = %.2f\n' ...
    '  R* = %.2f (p-scaled by %.2f)\n' ...
    '  u_max_rec = %.2f deg gimbal (%.2f code)\n' ...
    '  K = [%.3f  %.3f] on [theta(rad); q(rad/s)]\n' ...
    '  req slew (code) = 2*p*u_max = %.2f, measured = %.2f, margin = %.2f'], ...
    region, slew_code, slew_thresh_rescue, slew_thresh_fundamental, ...
    R_star, p_scale, u_max_deg, u_max_code, K(1), K(2), ...
    required_slew_code, slew_code, slew_margin);

out.region                = region;
out.u_max_recommended_deg = u_max_deg;
out.R_recommended         = R_star;
out.lqr_design_p          = lqr_design_p;
out.K                     = K;
out.go_nogo               = verdict;
out.slew_margin           = slew_margin;
out.notes                 = notes;

fprintf('=== TVC Pre-Flight Validator (v2) ===\n');
fprintf('  slew measured        : %.1f deg/s (gimbal)\n', meas.slew_deg_per_s);
fprintf('  servo mechanical max : %.1f deg gimbal\n', meas.servo_max_deg);
fprintf('  p_est                : %.2f 1/s\n', meas.p_est);
fprintf('  -> region            : %s\n', region);
fprintf('  -> verdict           : %s\n', verdict);
fprintf('  -> recommended u_max : %.2f deg gimbal\n', u_max_deg);
fprintf('  -> recommended R*    : %.2f\n', R_star);
fprintf('  -> LQR gain K        : [%.3f  %.3f]\n', K(1), K(2));
fprintf('  -> slew margin       : %.2f\n', slew_margin);
fprintf('  notes:\n%s\n', notes);
end
