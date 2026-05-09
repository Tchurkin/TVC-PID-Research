function fit = process_gimbal_bench_test(csv_path, out_csv_path)
% Estimate second-order actuator parameters from gimbal bench data.
% Required columns: time_s, cmd_deg, meas_deg

T = readtable(csv_path);
t = double(T.time_s(:));
u = deg2rad(double(T.cmd_deg(:)));
y = deg2rad(double(T.meas_deg(:)));

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

if nargin >= 2 && ~isempty(out_csv_path)
    out = table(fit.wn_rad_s, fit.zeta, fit.delay_s, fit.rmse_rad, ...
        'VariableNames', {'wn_rad_s', 'zeta', 'delay_s', 'fit_rmse_rad'});
    writetable(out, out_csv_path);
end

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
