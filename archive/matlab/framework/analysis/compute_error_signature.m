function sig = compute_error_signature(out, cfg, opts)
%COMPUTE_ERROR_SIGNATURE Build a structured mismatch vector from one trace.

if nargin < 3 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'fail_err_deg'); opts.fail_err_deg = 30.0; end
if ~isfield(opts, 'fail_theta_deg'); opts.fail_theta_deg = 60.0; end
if ~isfield(opts, 'oscillation_min_amp_deg'); opts.oscillation_min_amp_deg = 1.0; end

sig = struct();

if isempty(out) || ~isfield(out, 'time') || isempty(out.time)
    sig.actuator_saturation_frequency = NaN;
    sig.tracking_error_growth_rate = NaN;
    sig.divergence_time_s = NaN;
    sig.oscillation_amplitude_growth = NaN;
    sig.delay_induced_lag_s = NaN;
    sig.valid_trace = false;
    return;
end

if ~isfield(out, 'theta_ref') || isempty(out.theta_ref)
    out.theta_ref = zeros(size(out.theta));
end
if ~isfield(out, 'u_cmd') || isempty(out.u_cmd)
    out.u_cmd = zeros(size(out.theta));
end
if ~isfield(out, 'u_act') || isempty(out.u_act)
    out.u_act = zeros(size(out.theta));
end
if ~isfield(out, 'q') || isempty(out.q)
    out.q = zeros(size(out.theta));
end
if ~isfield(out, 'q_ref') || isempty(out.q_ref)
    out.q_ref = zeros(size(out.theta));
end

t = out.time(:);
theta = out.theta(:);
theta_ref = out.theta_ref(:);
u_cmd = out.u_cmd(:);
u_act = out.u_act(:);
err = theta - theta_ref;

sig.actuator_saturation_frequency = mean(abs(u_cmd) >= 0.99 * cfg.plant.u_max);
sig.tracking_error_growth_rate = local_growth_rate(t, abs(rad2deg(err)));
sig.divergence_time_s = local_divergence_time(t, err, theta, opts);
sig.oscillation_amplitude_growth = local_oscillation_growth(t, err, opts);
sig.delay_induced_lag_s = local_lag_estimate(t, theta_ref, theta);
sig.valid_trace = all(isfinite(theta)) && all(isfinite(theta_ref)) && all(isfinite(u_cmd)) && all(isfinite(u_act));
end


function slope = local_growth_rate(t, err_deg)
if numel(t) < 3 || all(~isfinite(err_deg))
    slope = NaN;
    return;
end

start_idx = max(1, round(0.5 * numel(t)));
idx = start_idx:numel(t);
if numel(idx) < 3
    slope = NaN;
    return;
end
x = t(idx) - t(idx(1));
y = err_deg(idx);
if all(~isfinite(y))
    slope = NaN;
    return;
end
p = polyfit(x(:), y(:), 1);
slope = p(1);
end


function t_div = local_divergence_time(t, err, theta, opts)
err_deg = abs(rad2deg(err));
theta_deg = abs(rad2deg(theta));
idx = find(err_deg > opts.fail_err_deg | theta_deg > opts.fail_theta_deg, 1, 'first');
if isempty(idx)
    t_div = t(end) - t(1);
else
    t_div = t(idx) - t(1);
end
end


function growth = local_oscillation_growth(t, err, opts)
if numel(t) < 5
    growth = NaN;
    return;
end

a = abs(rad2deg(err));
if all(~isfinite(a))
    growth = NaN;
    return;
end

t0 = t(1);
t1 = t0 + 0.35 * (t(end) - t0);
t2 = t0 + 0.75 * (t(end) - t0);
early = a(t <= t1);
late = a(t >= t2);
if isempty(early) || isempty(late)
    growth = NaN;
    return;
end

min_amp = max(opts.oscillation_min_amp_deg, 1.0);
early_peak = max(0, max(early) - min_amp);
late_peak = max(0, max(late) - min_amp);
growth = late_peak - early_peak;
end


function lag_s = local_lag_estimate(t, ref, y)
if numel(t) < 5 || all(~isfinite(ref)) || all(~isfinite(y))
    lag_s = NaN;
    return;
end

ref = ref(:) - mean(ref, 'omitnan');
y = y(:) - mean(y, 'omitnan');
if all(abs(ref) < 1e-9) || all(abs(y) < 1e-9)
    lag_s = NaN;
    return;
end

dt = median(diff(t));
max_lag = min(numel(t) - 1, max(5, round(0.5 / max(dt, 1e-6))));
lags = (-max_lag:max_lag)';
cc = nan(numel(lags), 1);
ref0 = ref - mean(ref, 'omitnan');
y0 = y - mean(y, 'omitnan');
for i = 1:numel(lags)
    lag = lags(i);
    if lag >= 0
        a = y0(1 + lag:end);
        b = ref0(1:end - lag);
    else
        lag2 = -lag;
        a = y0(1:end - lag2);
        b = ref0(1 + lag2:end);
    end
    if numel(a) < 3 || numel(b) < 3
        continue;
    end
    denom = sqrt(sum(a .^ 2) * sum(b .^ 2));
    if denom > 0
        cc(i) = sum(a .* b) / denom;
    end
end
[~, idx] = max(cc);
lag_s = lags(idx) * dt;
end
