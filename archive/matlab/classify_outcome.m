function [stabilized, metrics] = classify_outcome(out, theta_fail_rad)
%CLASSIFY_OUTCOME  Binary stabilization classifier + summary metrics.

if nargin < 2, theta_fail_rad = deg2rad(45); end

theta_max = max(abs(out.theta));
theta_end = abs(out.theta(end));

% Stabilized iff (a) never exceeds failure threshold AND
%                (b) end state is bounded (not still growing)
stabilized = (theta_max < theta_fail_rad) && (theta_end < theta_fail_rad);

% Additional check: divergence over last 20% of sim
N = numel(out.time);
i1 = max(1, round(0.8*N));
late_growth = abs(out.theta(end)) - max(abs(out.theta(i1:i1+min(10,N-i1))));
if late_growth > deg2rad(5)
    stabilized = false;
end

metrics.theta_max_deg = rad2deg(theta_max);
metrics.theta_end_deg = rad2deg(theta_end);
metrics.theta_rms_deg = rad2deg(sqrt(mean(out.theta.^2)));
metrics.u_act_max     = max(abs(out.u_act));
metrics.u_cmd_max     = max(abs(out.u_cmd));
metrics.u_sat_frac    = mean(abs(out.u_cmd) >= 0.99*out.params.u_max);
metrics.slew_sat_frac = mean(abs(diff(out.u_act))/out.params.dt >= 0.99*out.params.slew_max);
end
