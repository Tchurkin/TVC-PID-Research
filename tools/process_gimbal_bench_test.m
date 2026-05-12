function fit = process_gimbal_bench_test(csv_path, out_csv_path)
% Estimate actuator fit and directly measured bench metrics.
% Required columns: time_s, cmd_deg, meas_deg
%
% Returned fields include:
%   wn_rad_s, zeta, delay_s, rmse_rad
%   dt_s, positive_slew_deg_s, negative_slew_deg_s, max_abs_slew_deg_s
%   onset_delay_s, rise_time_10_90_s, deadband_deg, hysteresis_deg

T = readtable(csv_path);
t = double(T.time_s(:));
cmd_deg = double(T.cmd_deg(:));
meas_deg = double(T.meas_deg(:));
u = deg2rad(cmd_deg);
y = deg2rad(meas_deg);

dt = median(diff(t));
if dt <= 0
    error('Non-increasing time vector in bench test data.');
end

% Fit parameters p=[wn,zeta,tau] for G(s)=wn^2/(s^2+2*zeta*wn*s+wn^2)*e^{-tau s}
obj = @(p) fit_error(p, t, u, y);
p0 = [40, 0.7, 0.04];
lb = [5, 0.2, 0.0];
ub = [120, 1.8, 0.15];
opts = optimset('Display', 'off');
p = fminsearchbnd(obj, p0, lb, ub, opts);

fit.wn_rad_s = p(1);
fit.zeta = p(2);
fit.delay_s = p(3);
fit.rmse_rad = sqrt(obj(p));
fit.dt_s = dt;

bench_metrics = estimate_bench_metrics(t, cmd_deg, meas_deg);
fit.positive_slew_deg_s = bench_metrics.positive_slew_deg_s;
fit.negative_slew_deg_s = bench_metrics.negative_slew_deg_s;
fit.max_abs_slew_deg_s = bench_metrics.max_abs_slew_deg_s;
fit.onset_delay_s = bench_metrics.onset_delay_s;
fit.rise_time_10_90_s = bench_metrics.rise_time_10_90_s;
fit.deadband_deg = bench_metrics.deadband_deg;
fit.hysteresis_deg = bench_metrics.hysteresis_deg;

if nargin >= 2 && ~isempty(out_csv_path)
    out = table(fit.wn_rad_s, fit.zeta, fit.delay_s, fit.rmse_rad, fit.dt_s, ...
        fit.positive_slew_deg_s, fit.negative_slew_deg_s, fit.max_abs_slew_deg_s, ...
        fit.onset_delay_s, fit.rise_time_10_90_s, fit.deadband_deg, fit.hysteresis_deg, ...
        'VariableNames', {'wn_rad_s', 'zeta', 'delay_s', 'fit_rmse_rad', 'dt_s', ...
        'positive_slew_deg_s', 'negative_slew_deg_s', 'max_abs_slew_deg_s', ...
        'onset_delay_s', 'rise_time_10_90_s', 'deadband_deg', 'hysteresis_deg'});
    writetable(out, out_csv_path);
end

end

function metrics = estimate_bench_metrics(t, cmd_deg, meas_deg)
dt = median(diff(t));
meas_rate_deg_s = gradient(meas_deg, t);

metrics.positive_slew_deg_s = max([0; meas_rate_deg_s]);
metrics.negative_slew_deg_s = max([0; -meas_rate_deg_s]);
metrics.max_abs_slew_deg_s = max(abs(meas_rate_deg_s));

cmd_span = max(cmd_deg) - min(cmd_deg);
if cmd_span < 1e-6
    metrics.onset_delay_s = NaN;
    metrics.rise_time_10_90_s = NaN;
    metrics.deadband_deg = NaN;
    metrics.hysteresis_deg = NaN;
    return;
end

step_thresh = max(0.5, 0.08 * cmd_span);
motion_rate_thresh = max(2.0, 0.05 * metrics.max_abs_slew_deg_s);
step_idx = find(abs([0; diff(cmd_deg)]) >= step_thresh);

delay_vals = [];
rise_vals = [];
for ii = 1:numel(step_idx)
    k0 = step_idx(ii);
    if k0 >= numel(t)
        continue;
    end

    cmd_before = cmd_deg(max(k0 - 1, 1));
    cmd_after = cmd_deg(min(k0 + 1, numel(cmd_deg)));
    cmd_delta = cmd_after - cmd_before;
    if abs(cmd_delta) < step_thresh
        continue;
    end

    search_end = min(numel(t), k0 + max(5, round(1.0 / dt)));
    local_rate = meas_rate_deg_s(k0:search_end);
    onset_rel = find(abs(local_rate) >= motion_rate_thresh, 1, 'first');
    if ~isempty(onset_rel)
        delay_vals(end + 1, 1) = t(k0 + onset_rel - 1) - t(k0); %#ok<AGROW>
    end

    y0 = meas_deg(max(k0 - 1, 1));
    lo_val = y0 + 0.10 * cmd_delta;
    hi_val = y0 + 0.90 * cmd_delta;
    y_seg = meas_deg(k0:search_end);
    t_seg = t(k0:search_end);

    idx10 = first_crossing_index(y_seg, lo_val, sign(cmd_delta));
    idx90 = first_crossing_index(y_seg, hi_val, sign(cmd_delta));
    if ~isempty(idx10) && ~isempty(idx90) && idx90 >= idx10
        rise_vals(end + 1, 1) = t_seg(idx90) - t_seg(idx10); %#ok<AGROW>
    elseif ~isempty(idx10)
        % If 90%% is not reached in the window, preserve the partial result as NaN.
        rise_vals(end + 1, 1) = NaN; %#ok<AGROW>
    end

end

if isempty(delay_vals)
    metrics.onset_delay_s = NaN;
else
    metrics.onset_delay_s = median(delay_vals, 'omitnan');
end

if isempty(rise_vals)
    metrics.rise_time_10_90_s = NaN;
else
    metrics.rise_time_10_90_s = median(rise_vals, 'omitnan');
end

metrics.deadband_deg = estimate_deadband(cmd_deg, meas_rate_deg_s, motion_rate_thresh);
metrics.hysteresis_deg = estimate_hysteresis(cmd_deg, meas_deg);
end

function idx = first_crossing_index(y, target, direction_sign)
if direction_sign >= 0
    idx = find(y >= target, 1, 'first');
else
    idx = find(y <= target, 1, 'first');
end
end

function deadband_deg = estimate_deadband(cmd_deg, meas_rate_deg_s, motion_rate_thresh)
near_static = abs(meas_rate_deg_s) <= motion_rate_thresh;
near_zero_cmd = abs(cmd_deg) <= max(1.0, 0.25 * max(abs(cmd_deg)));
idx = near_static & near_zero_cmd;
if ~any(idx)
    deadband_deg = NaN;
    return;
end

deadband_deg = prctile(abs(cmd_deg(idx)), 90);
end

function hysteresis_deg = estimate_hysteresis(cmd_deg, meas_deg)
dcmd = [0; diff(cmd_deg)];
inc_idx = dcmd > 0;
dec_idx = dcmd < 0;
if nnz(inc_idx) < 5 || nnz(dec_idx) < 5
    hysteresis_deg = NaN;
    return;
end

cmd_min = max(min(cmd_deg(inc_idx)), min(cmd_deg(dec_idx)));
cmd_max = min(max(cmd_deg(inc_idx)), max(cmd_deg(dec_idx)));
if ~(cmd_max > cmd_min)
    hysteresis_deg = NaN;
    return;
end

grid_n = 25;
cmd_grid = linspace(cmd_min, cmd_max, grid_n)';
meas_inc = interp1(cmd_deg(inc_idx), meas_deg(inc_idx), cmd_grid, 'linear', NaN);
meas_dec = interp1(cmd_deg(dec_idx), meas_deg(dec_idx), cmd_grid, 'linear', NaN);
valid = isfinite(meas_inc) & isfinite(meas_dec);
if ~any(valid)
    hysteresis_deg = NaN;
    return;
end

hysteresis_deg = median(abs(meas_inc(valid) - meas_dec(valid)), 'omitnan');
end

function mse = fit_error(p, t, u, y)
wn = p(1); zeta = p(2); tau = p(3);

% Delay via sample shift and linear interpolation.
u_del = interp1(t, u, t - tau, 'linear', 'extrap');

x = [0; 0];
yhat = zeros(size(y));
for k = 1:numel(t)-1
    dt = t(k+1) - t(k);
    [~, xx] = ode45(@(~, xs) act_ode(xs, u_del(k), wn, zeta), [0 dt], x);
    x = xx(end, :)';
    yhat(k+1) = x(1);
end

e = y - yhat;
mse = mean(e.^2);
end

function dx = act_ode(x, u, wn, zeta)
y = x(1);
ydot = x(2);
yddot = wn^2 * (u - y) - 2 * zeta * wn * ydot;
dx = [ydot; yddot];
end

function x = fminsearchbnd(fun, x0, lb, ub, opts)
% Lightweight bounded wrapper around fminsearch.
y0 = inv_transform(x0, lb, ub);
y = fminsearch(@(yy) fun(transform(yy, lb, ub)), y0, opts);
x = transform(y, lb, ub);
end

function y = inv_transform(x, lb, ub)
xn = (x - lb) ./ (ub - lb);
xn = min(max(xn, 1e-6), 1 - 1e-6);
y = log(xn ./ (1 - xn));
end

function x = transform(y, lb, ub)
s = 1 ./ (1 + exp(-y));
x = lb + (ub - lb) .* s;
end
