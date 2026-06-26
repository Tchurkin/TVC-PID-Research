function best = autotune_pd_grid(cfg, sc, realism, P)
%AUTOTUNE_PD_GRID Grid-search PD gains for a single plant cell.

best = struct();
best.Kp = NaN;
best.Kd = NaN;
best.success_rate = -inf;
best.rms_error_deg = inf;
best.peak_error_deg = inf;
best.end_error_deg = inf;
best.max_theta_deg = inf;
best.u_cmd_sat_frac = 1.0;
best.slew_sat_frac = 1.0;
best.settling_time_s = inf;
best.oscillation_score = inf;

for Kp = P.tuning.Kp_grid
    for Kd = P.tuning.Kd_grid
        cfg_eval = configure_pid_controller(cfg, Kp, Kd);
        metrics = evaluate_stability_cell(cfg_eval, sc, realism, P);

        if is_better(metrics, best)
            best.Kp = Kp;
            best.Kd = Kd;
            best.success_rate = metrics.success_rate;
            best.rms_error_deg = metrics.rms_error_deg;
            best.peak_error_deg = metrics.peak_error_deg;
            best.end_error_deg = metrics.end_error_deg;
            best.max_theta_deg = metrics.max_theta_deg;
            best.u_cmd_sat_frac = metrics.u_cmd_sat_frac;
            best.slew_sat_frac = metrics.slew_sat_frac;
            best.settling_time_s = metrics.settling_time_s;
            best.oscillation_score = metrics.oscillation_score;
        end
    end
end


function yes = is_better(m, b)
tol = 1e-12;

if m.success_rate > b.success_rate + tol
    yes = true;
    return;
end

if abs(m.success_rate - b.success_rate) <= tol && m.rms_error_deg < b.rms_error_deg - tol
    yes = true;
    return;
end

if abs(m.success_rate - b.success_rate) <= tol && abs(m.rms_error_deg - b.rms_error_deg) <= tol ...
        && m.peak_error_deg < b.peak_error_deg - tol
    yes = true;
    return;
end

if abs(m.success_rate - b.success_rate) <= tol && abs(m.rms_error_deg - b.rms_error_deg) <= tol ...
        && abs(m.peak_error_deg - b.peak_error_deg) <= tol ...
        && (m.u_cmd_sat_frac + m.slew_sat_frac) < (b.u_cmd_sat_frac + b.slew_sat_frac) - tol
    yes = true;
    return;
end

if abs(m.success_rate - b.success_rate) <= tol && abs(m.rms_error_deg - b.rms_error_deg) <= tol ...
        && abs(m.peak_error_deg - b.peak_error_deg) <= tol ...
        && abs((m.u_cmd_sat_frac + m.slew_sat_frac) - (b.u_cmd_sat_frac + b.slew_sat_frac)) <= tol ...
        && m.settling_time_s < b.settling_time_s - tol
    yes = true;
    return;
end

yes = false;
