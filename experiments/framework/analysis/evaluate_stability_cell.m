function metrics = evaluate_stability_cell(cfg, sc, realism, P)
%EVALUATE_STABILITY_CELL Evaluate one plant/controller cell over trial seeds.

n_trials = numel(P.analysis.theta0_deg_set) * numel(P.analysis.seeds);
success_col = false(n_trials, 1);
rms_col = zeros(n_trials, 1);
peak_col = zeros(n_trials, 1);
end_col = zeros(n_trials, 1);
max_theta_col = zeros(n_trials, 1);
u_sat_col = zeros(n_trials, 1);
slew_sat_col = zeros(n_trials, 1);
settling_col = zeros(n_trials, 1);
oscillation_col = zeros(n_trials, 1);

idx = 0;
for it = 1:numel(P.analysis.theta0_deg_set)
    cfg.plant.theta0 = deg2rad(P.analysis.theta0_deg_set(it));
    for iseed = 1:numel(P.analysis.seeds)
        idx = idx + 1;
        seed = P.analysis.seeds(iseed) + 100 * it;
        try
            out = simulate_case_realistic('PID', sc, cfg, seed, realism);
            [success_col(idx), rms_col(idx), peak_col(idx), end_col(idx), max_theta_col(idx), ...
                u_sat_col(idx), slew_sat_col(idx), settling_col(idx), oscillation_col(idx)] = score_run(out, cfg, P);
        catch
            success_col(idx) = false;
            rms_col(idx) = 90;
            peak_col(idx) = 90;
            end_col(idx) = 90;
            max_theta_col(idx) = 90;
            u_sat_col(idx) = 1;
            slew_sat_col(idx) = 1;
            settling_col(idx) = cfg.t_end_demo;
            oscillation_col(idx) = inf;
        end
    end
end

metrics = struct();
metrics.success_rate = mean(success_col);
metrics.rms_error_deg = mean(rms_col);
metrics.peak_error_deg = mean(peak_col);
metrics.end_error_deg = mean(end_col);
metrics.max_theta_deg = mean(max_theta_col);
metrics.u_cmd_sat_frac = mean(u_sat_col);
metrics.slew_sat_frac = mean(slew_sat_col);
metrics.settling_time_s = mean(settling_col);
metrics.oscillation_score = mean(oscillation_col);
metrics.n_trials = n_trials;


function [success, rms_error_deg, peak_error_deg, end_error_deg, max_theta_deg, u_sat_frac, slew_sat_frac, settling_time_s, oscillation_score] = score_run(out, cfg, P)
if ~all(isfinite(out.theta))
    success = false;
    rms_error_deg = 90;
    peak_error_deg = 90;
    end_error_deg = 90;
    max_theta_deg = 90;
    u_sat_frac = 1;
    slew_sat_frac = 1;
    settling_time_s = cfg.t_end_demo;
    oscillation_score = inf;
    return;
end

rms_error_deg = min(90, rad2deg(sqrt(mean(out.theta .^ 2))));
peak_error_deg = min(90, rad2deg(max(abs(out.theta))));
end_error_deg = min(90, rad2deg(abs(out.theta(end))));
max_theta_deg = min(90, rad2deg(max(abs(out.theta))));
u_sat_frac = mean(abs(out.u_cmd) >= 0.99 * cfg.plant.u_max);
slew_sat_frac = mean(abs(diff(out.u_act)) / cfg.dt >= 0.99 * cfg.plant.slew_max);
settling_time_s = estimate_settling_time(out.time, out.theta, deg2rad(2.0));
oscillation_score = estimate_oscillation_score(out.time, out.theta, deg2rad(1.0));

success = rms_error_deg <= P.analysis.success_rms_deg && ...
    peak_error_deg <= P.analysis.success_peak_deg && ...
    end_error_deg <= P.analysis.success_end_deg && ...
    max_theta_deg <= P.analysis.success_max_theta_deg;


function t_settle = estimate_settling_time(t, theta, band_rad)
t_settle = t(end);
inside = abs(theta) <= band_rad;
for k = 1:numel(t)
    if all(inside(k:end))
        t_settle = t(k);
        return;
    end
end


function score = estimate_oscillation_score(t, theta, min_amp_rad)
mask = t >= 0.50;
theta_eval = theta(mask);
t_eval = t(mask);
if numel(theta_eval) < 3
    score = 0;
    return;
end

theta_eval(abs(theta_eval) < min_amp_rad) = 0;
sgn = sign(theta_eval);
crossings = sum(sgn(1:end-1) .* sgn(2:end) < 0);
duration = max(1e-6, t_eval(end) - t_eval(1));
score = crossings / duration;
