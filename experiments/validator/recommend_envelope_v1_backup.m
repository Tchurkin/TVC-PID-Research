function out = recommend_envelope(meas)
%RECOMMEND_ENVELOPE  Amateur-TVC pre-flight validator.
%
% Input struct `meas` (from bench INO + airframe spreadsheet):
%   meas.slew_deg_per_s    measured servo slew rate (deg/s, GIMBAL deg if linkage applied)
%   meas.servo_max_deg     hard mechanical envelope of the gimbal (deg)
%   meas.p_est             estimated airframe instability rate (1/s) from CG/aero
%   meas.keff_est          control effectiveness (rad/s^2 per gimbal-rad), default 8
%   meas.damp_est          rate damping (1/s), default 0.2
%
% Returns:
%   out.u_max_recommended_deg   safe gimbal saturation (deg)
%   out.lqr_design_p            recommended p for LQR design (conservative)
%   out.K                       2x1 LQR gain on [theta(rad); q(rad/s)]
%   out.go_nogo                 'GO' | 'MARGINAL' | 'NOGO'
%   out.notes                   explanation string
%
% Backed by H1 (finite-authority optimum) and H2 (conservative-design rule)
% from experiments/HEADLINE_LOCKED.md.

if ~isfield(meas, 'keff_est'), meas.keff_est = 8.0; end
if ~isfield(meas, 'damp_est'), meas.damp_est = 0.2; end

% Convert slew to rad/s (gimbal radians per second)
slew_rad_s = deg2rad(meas.slew_deg_per_s);

% H1 fitted rule: u*_max(p, slew) ~ a*slew + b*p   (heuristic from h1_lock data)
% slew in rad/s, p in 1/s, result in gimbal-rad.
a = 0.06; b = 0.30;
u_max_rad = a * slew_rad_s + b * meas.p_est * (pi/180); % keep rad

% Convert back to degrees of gimbal
u_max_deg_rec = rad2deg(u_max_rad);

% Clamp by mechanical envelope
u_max_deg_rec = min(u_max_deg_rec, 0.85 * meas.servo_max_deg);
u_max_deg_rec = max(u_max_deg_rec, 1.5);   % don't go below ~1.5 deg authority

% H2 rule: design LQR at the conservative end of expected p range.
% If user only gave a single p_est, treat as "conservative" already.
lqr_design_p = meas.p_est;

K = design_lqr_unstable(lqr_design_p, meas.damp_est, meas.keff_est);

% Go/no-go: is the recommended envelope inside hardware bounds and
% is slew sufficient to chase a step at the unstable timescale?
required_slew_rad_s = 2 * meas.p_est * u_max_rad;   % rough rule
slew_margin = slew_rad_s / max(1e-6, required_slew_rad_s);

if u_max_deg_rec >= 1.5 && slew_margin > 1.2
    verdict = 'GO';
elseif slew_margin > 0.8
    verdict = 'MARGINAL';
else
    verdict = 'NOGO';
end

notes = sprintf( ...
    ['H1 rule -> u*_max = a*slew + b*p with a=%.2f, b=%.2f (gimbal rad)\n' ...
     'H2 rule -> design LQR at conservative p = %.2f (1/s)\n' ...
     'Required slew rate (rad/s) ~ 2*p*u_max = %.2f, measured = %.2f, margin = %.2f'], ...
    a, b, lqr_design_p, required_slew_rad_s, slew_rad_s, slew_margin);

out.u_max_recommended_deg = u_max_deg_rec;
out.lqr_design_p          = lqr_design_p;
out.K                     = K;
out.go_nogo               = verdict;
out.slew_margin           = slew_margin;
out.notes                 = notes;

fprintf('=== TVC Pre-Flight Validator ===\n');
fprintf('  slew measured        : %.1f deg/s\n', meas.slew_deg_per_s);
fprintf('  servo mechanical max : %.1f deg\n', meas.servo_max_deg);
fprintf('  p_est                : %.2f 1/s\n', meas.p_est);
fprintf('  -> recommended u_max : %.2f deg gimbal\n', u_max_deg_rec);
fprintf('  -> LQR gain K = [%.3f  %.3f]  (theta_rad, q_rad/s)\n', K(1), K(2));
fprintf('  -> slew margin       : %.2f\n', slew_margin);
fprintf('  -> verdict           : %s\n', verdict);
fprintf('  notes:\n%s\n', notes);
end
