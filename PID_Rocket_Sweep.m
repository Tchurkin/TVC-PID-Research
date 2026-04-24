%% ========================================================================
%% TVC RESEARCH PIPELINE - PURE MATLAB (NO SIMULINK)
%% Regime-Dependent Limits of Simulation Fidelity Under Delay + Aero Uncertainty
%% ========================================================================

clear; close all; clc;

fprintf('\n===============================================================\n');
fprintf('TVC FIDELITY STUDY - CODE-ONLY SIMULATION PIPELINE\n');
fprintf('===============================================================\n\n');

%% ------------------------------------------------------------------------
%% 0) GLOBAL SETTINGS
%% ------------------------------------------------------------------------

cfg.dt = 0.002;                 % s
cfg.t_final = 4.0;              % s
cfg.gravity = 9.81;             % m/s^2

% Vehicle properties (edit with your measured values)
cfg.mass_kg = 1.85;
cfg.Iyy = 0.095;
cfg.lever_arm_m = 0.24;         % TVC moment arm
cfg.thrust_nom_N = 46.0;
cfg.drag_coeff = 0.18;
cfg.aero_damp_nom = 0.038;      % N*m*s/rad

% Controller gains (hold fixed during campaign)
cfg.kp = 8.2;
cfg.ki = 1.1;
cfg.kd = 0.35;
cfg.actuator_limit_deg = 10;

% Sweep variables from research abstract
cfg.tau_actuator_ms = [8, 15, 25, 40];
cfg.tau_sensor_filter_ms = [3, 8, 15, 25];
cfg.aero_damping_scale = [0.70, 0.85, 1.00, 1.15, 1.30];
cfg.thrust_scale = [0.90, 1.00, 1.10];

% Regimes in angular-rate space (deg/s)
cfg.rate_bins = [0, 80; 80, 180; 180, 320];
cfg.regime_names = {'LOW_RATE', 'MID_RATE', 'HIGH_RATE'};

% Safety / instability definitions
cfg.max_abs_angle_deg = 45;
cfg.max_abs_rate_deg_s = 450;

% Baseline for "simplified model" and ablation tests
cfg.base_tau_actuator_ms = 8;
cfg.base_tau_sensor_ms = 3;
cfg.base_aero_scale = 1.0;
cfg.base_thrust_scale = 1.0;

fprintf('Time step: %.4f s, horizon: %.1f s\n', cfg.dt, cfg.t_final);
fprintf('Design points (cases): %d\n\n', numel(cfg.tau_actuator_ms) * numel(cfg.tau_sensor_filter_ms) * numel(cfg.aero_damping_scale) * numel(cfg.thrust_scale));

%% ------------------------------------------------------------------------
%% 1) STEP PLAN (EXECUTION ORDER)
%% ------------------------------------------------------------------------

plan_steps = {
    '1. Freeze gains and physical constants used by all runs.'
    '2. Build uncertainty sweep over delay, aero damping, and thrust variability.'
    '3. Simulate high-fidelity truth trajectories in pure MATLAB code.'
    '4. Simulate simplified predictor trajectories with fixed baseline assumptions.'
    '5. Compute trajectory residuals and phase-lag divergence per regime.'
    '6. Run ablation: delay-only effect and aero-only effect.'
    '7. Estimate transition boundary where delay error overtakes aero error.'
    '8. Export tables/figures for STS writing and flight-test integration.'
};

fprintf('PROJECT EXECUTION PLAN:\n');
for i = 1:numel(plan_steps)
    fprintf('  %s\n', plan_steps{i});
end
fprintf('\n');

%% ------------------------------------------------------------------------
%% 2) BUILD SWEEP TABLE + FLIGHT MANIFEST TEMPLATE
%% ------------------------------------------------------------------------

sim_table = build_simulation_table(cfg);
flight_manifest = build_flight_manifest(15, cfg);

fprintf('Sweep table rows: %d\n', height(sim_table));
fprintf('Flight manifest rows: %d\n\n', height(flight_manifest));

%% ------------------------------------------------------------------------
%% 3) RUN CODE-ONLY FIDELITY SWEEP
%% ------------------------------------------------------------------------

results = run_fidelity_sweep(sim_table, cfg);

%% ------------------------------------------------------------------------
%% 4) ESTIMATE REGIME TRANSITION BOUNDARY
%% ------------------------------------------------------------------------

transition = estimate_transition_boundary(results);

fprintf('Boundary estimate (delay-dominated onset): %.1f deg/s\n', transition.boundary_rate_deg_s);
fprintf('Boundary confidence flag: %s\n\n', transition.confidence_flag);

%% ------------------------------------------------------------------------
%% 5) SAVE ARTIFACTS
%% ------------------------------------------------------------------------

out_dir = fullfile(pwd, 'outputs');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

writetable(sim_table, fullfile(out_dir, 'sim_design_table.csv'));
writetable(flight_manifest, fullfile(out_dir, 'flight_manifest_template.csv'));
writetable(results, fullfile(out_dir, 'fidelity_results_by_regime.csv'));
writetable(transition.summary_table, fullfile(out_dir, 'regime_transition_summary.csv'));

fprintf('Saved outputs:\n');
fprintf('  - outputs/sim_design_table.csv\n');
fprintf('  - outputs/flight_manifest_template.csv\n');
fprintf('  - outputs/fidelity_results_by_regime.csv\n');
fprintf('  - outputs/regime_transition_summary.csv\n\n');

%% ------------------------------------------------------------------------
%% 6) PLOTS FOR STS DRAFTING
%% ------------------------------------------------------------------------

plot_regime_divergence(results, transition);
plot_residual_map(results);

fprintf('Finished. Next: replace baseline constants with measured hardware values.\n\n');

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================

function tbl = build_simulation_table(cfg)
    [a, s, d, t] = ndgrid(cfg.tau_actuator_ms, ...
                         cfg.tau_sensor_filter_ms, ...
                         cfg.aero_damping_scale, ...
                         cfg.thrust_scale);
    tbl = table;
    tbl.case_id = (1:numel(a))';
    tbl.tau_actuator_ms = a(:);
    tbl.tau_sensor_ms = s(:);
    tbl.aero_scale = d(:);
    tbl.thrust_scale = t(:);
end

function manifest = build_flight_manifest(n_flights, cfg)
    manifest = table;
    manifest.flight_id = (1:n_flights)';
    manifest.regime = strings(n_flights, 1);
    manifest.target_rate_min_deg_s = nan(n_flights, 1);
    manifest.target_rate_max_deg_s = nan(n_flights, 1);
    manifest.wind_est_m_s = nan(n_flights, 1);
    manifest.mass_kg = nan(n_flights, 1);
    manifest.cg_from_nose_m = nan(n_flights, 1);
    manifest.notes = strings(n_flights, 1);

    idx = 1;
    per_regime = floor(n_flights / size(cfg.rate_bins, 1));
    for r = 1:size(cfg.rate_bins, 1)
        for j = 1:per_regime
            manifest.regime(idx) = string(cfg.regime_names{r});
            manifest.target_rate_min_deg_s(idx) = cfg.rate_bins(r, 1);
            manifest.target_rate_max_deg_s(idx) = cfg.rate_bins(r, 2);
            manifest.notes(idx) = "assign on launch day";
            idx = idx + 1;
        end
    end

    while idx <= n_flights
        r = size(cfg.rate_bins, 1);
        manifest.regime(idx) = string(cfg.regime_names{r});
        manifest.target_rate_min_deg_s(idx) = cfg.rate_bins(r, 1);
        manifest.target_rate_max_deg_s(idx) = cfg.rate_bins(r, 2);
        manifest.notes(idx) = "extra replicate";
        idx = idx + 1;
    end
end

function results = run_fidelity_sweep(sim_table, cfg)
    n_cases = height(sim_table);
    n_reg = size(cfg.rate_bins, 1);

    n_rows = n_cases * n_reg;
    results = table;
    results.case_id = zeros(n_rows, 1);
    results.regime_id = zeros(n_rows, 1);
    results.regime_name = strings(n_rows, 1);
    results.rate_center_deg_s = zeros(n_rows, 1);
    results.tau_actuator_ms = zeros(n_rows, 1);
    results.tau_sensor_ms = zeros(n_rows, 1);
    results.aero_scale = zeros(n_rows, 1);
    results.thrust_scale = zeros(n_rows, 1);
    results.rms_theta_residual_deg = zeros(n_rows, 1);
    results.rms_rate_residual_deg_s = zeros(n_rows, 1);
    results.phase_lag_divergence_ms = zeros(n_rows, 1);
    results.aero_only_residual_deg = zeros(n_rows, 1);
    results.delay_only_residual_deg = zeros(n_rows, 1);
    results.unstable_truth = false(n_rows, 1);
    results.dominant_source = strings(n_rows, 1);

    row = 1;
    for c = 1:n_cases
        case_params = sim_table(c, :);
        for r = 1:n_reg
            rate_center = mean(cfg.rate_bins(r, :));

            full = evaluate_case(case_params, rate_center, cfg, 'full');
            aero_only = evaluate_case(case_params, rate_center, cfg, 'aero_only');
            delay_only = evaluate_case(case_params, rate_center, cfg, 'delay_only');

            results.case_id(row) = case_params.case_id;
            results.regime_id(row) = r;
            results.regime_name(row) = string(cfg.regime_names{r});
            results.rate_center_deg_s(row) = rate_center;
            results.tau_actuator_ms(row) = case_params.tau_actuator_ms;
            results.tau_sensor_ms(row) = case_params.tau_sensor_ms;
            results.aero_scale(row) = case_params.aero_scale;
            results.thrust_scale(row) = case_params.thrust_scale;
            results.rms_theta_residual_deg(row) = full.rms_theta_residual_deg;
            results.rms_rate_residual_deg_s(row) = full.rms_rate_residual_deg_s;
            results.phase_lag_divergence_ms(row) = full.phase_lag_divergence_ms;
            results.aero_only_residual_deg(row) = aero_only.rms_theta_residual_deg;
            results.delay_only_residual_deg(row) = delay_only.rms_theta_residual_deg;
            results.unstable_truth(row) = full.unstable_truth;

            if results.delay_only_residual_deg(row) > results.aero_only_residual_deg(row)
                results.dominant_source(row) = "DELAY";
            else
                results.dominant_source(row) = "AERO";
            end
            row = row + 1;
        end
    end
end

function out = evaluate_case(case_params, rate_center_deg_s, cfg, mode)
    nominal = make_params(cfg.base_tau_actuator_ms, cfg.base_tau_sensor_ms, cfg.base_aero_scale, cfg.base_thrust_scale);

    switch mode
        case 'full'
            truth = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, case_params.aero_scale, case_params.thrust_scale);
        case 'aero_only'
            truth = make_params(cfg.base_tau_actuator_ms, cfg.base_tau_sensor_ms, case_params.aero_scale, case_params.thrust_scale);
        case 'delay_only'
            truth = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, cfg.base_aero_scale, cfg.base_thrust_scale);
        otherwise
            error('Unknown mode: %s', mode);
    end

    x0 = make_initial_state(rate_center_deg_s);
    truth_traj = simulate_tvc(x0, truth, cfg);
    pred_traj = simulate_tvc(x0, nominal, cfg);

    [rms_theta, rms_rate, lag_ms] = compute_divergence_metrics(truth_traj, pred_traj, cfg.dt);
    out.rms_theta_residual_deg = rms_theta;
    out.rms_rate_residual_deg_s = rms_rate;
    out.phase_lag_divergence_ms = lag_ms;
    out.unstable_truth = truth_traj.unstable;
end

function p = make_params(tau_act_ms, tau_sensor_ms, aero_scale, thrust_scale)
    p.tau_act_s = tau_act_ms / 1000;
    p.tau_sensor_s = tau_sensor_ms / 1000;
    p.aero_scale = aero_scale;
    p.thrust_scale = thrust_scale;
end

function x0 = make_initial_state(rate_center_deg_s)
    x0 = struct;
    x0.x_m = 0;
    x0.z_m = 0;
    x0.vx_m_s = 0;
    x0.vz_m_s = 0;
    x0.theta_rad = deg2rad(6);
    x0.q_rad_s = deg2rad(rate_center_deg_s);
    x0.delta_rad = 0;
    x0.theta_hat_rad = x0.theta_rad;
    x0.q_hat_rad_s = x0.q_rad_s;
    x0.i_err = 0;
end

function traj = simulate_tvc(x0, p, cfg)
    n = floor(cfg.t_final / cfg.dt) + 1;

    theta = zeros(n, 1);
    q = zeros(n, 1);
    delta = zeros(n, 1);

    x = x0.x_m;
    z = x0.z_m;
    vx = x0.vx_m_s;
    vz = x0.vz_m_s;
    th = x0.theta_rad;
    qk = x0.q_rad_s;
    dk = x0.delta_rad;
    th_hat = x0.theta_hat_rad;
    q_hat = x0.q_hat_rad_s;
    i_err = x0.i_err;

    unstable = false;
    delta_lim = deg2rad(cfg.actuator_limit_deg);

    for k = 1:n
        theta(k) = th;
        q(k) = qk;
        delta(k) = dk;

        err = -th_hat;
        i_err = i_err + err * cfg.dt;
        d_err = -q_hat;

        delta_cmd = cfg.kp * err + cfg.ki * i_err + cfg.kd * d_err;
        delta_cmd = min(max(delta_cmd, -delta_lim), delta_lim);

        tau_act = max(p.tau_act_s, 1e-4);
        dk = dk + ((delta_cmd - dk) / tau_act) * cfg.dt;

        thrust = cfg.thrust_nom_N * p.thrust_scale;
        fx_body = thrust * sin(dk);
        fz_body = thrust * cos(dk);

        c = cos(th);
        s = sin(th);
        fx_world = c * fx_body - s * fz_body;
        fz_world = s * fx_body + c * fz_body - cfg.mass_kg * cfg.gravity;

        drag_x = -cfg.drag_coeff * vx * abs(vx);
        drag_z = -cfg.drag_coeff * vz * abs(vz);

        ax = (fx_world + drag_x) / cfg.mass_kg;
        az = (fz_world + drag_z) / cfg.mass_kg;

        vx = vx + ax * cfg.dt;
        vz = vz + az * cfg.dt;
        x = x + vx * cfg.dt;
        z = z + vz * cfg.dt;

        m_tvc = thrust * cfg.lever_arm_m * sin(dk);
        m_aero = -(cfg.aero_damp_nom * p.aero_scale) * qk;
        qdot = (m_tvc + m_aero) / cfg.Iyy;

        qk = qk + qdot * cfg.dt;
        th = th + qk * cfg.dt;

        tau_sensor = max(p.tau_sensor_s, 1e-4);
        th_hat = th_hat + ((th - th_hat) / tau_sensor) * cfg.dt;
        q_hat = q_hat + ((qk - q_hat) / tau_sensor) * cfg.dt;

        if abs(rad2deg(th)) > cfg.max_abs_angle_deg || abs(rad2deg(qk)) > cfg.max_abs_rate_deg_s
            unstable = true;
        end
    end

    traj.t = (0:n-1)' * cfg.dt;
    traj.x = x;
    traj.z = z;
    traj.theta = theta;
    traj.q = q;
    traj.delta = delta;
    traj.unstable = unstable;
end

function [rms_theta_deg, rms_rate_deg_s, lag_ms] = compute_divergence_metrics(truth_traj, pred_traj, dt)
    th_err = rad2deg(truth_traj.theta - pred_traj.theta);
    q_err = rad2deg(truth_traj.q - pred_traj.q);

    rms_theta_deg = sqrt(mean(th_err .^ 2));
    rms_rate_deg_s = sqrt(mean(q_err .^ 2));

    [xc, lags] = xcorr(truth_traj.theta - mean(truth_traj.theta), pred_traj.theta - mean(pred_traj.theta), 'coeff');
    [~, idx] = max(xc);
    lag_s = lags(idx) * dt;
    lag_ms = abs(lag_s) * 1000;
end

function transition = estimate_transition_boundary(results)
    rates = unique(results.rate_center_deg_s);
    aero_m = zeros(numel(rates), 1);
    delay_m = zeros(numel(rates), 1);

    for i = 1:numel(rates)
        r = rates(i);
        idx = results.rate_center_deg_s == r;
        aero_m(i) = mean(results.aero_only_residual_deg(idx));
        delay_m(i) = mean(results.delay_only_residual_deg(idx));
    end

    diff_m = delay_m - aero_m;
    cross_idx = find(diff_m >= 0, 1, 'first');

    if isempty(cross_idx)
        boundary = rates(end);
        flag = "NO_CROSSING_IN_RANGE";
    elseif cross_idx == 1
        boundary = rates(1);
        flag = "DELAY_DOMINANT_FROM_START";
    else
        r1 = rates(cross_idx - 1);
        r2 = rates(cross_idx);
        d1 = diff_m(cross_idx - 1);
        d2 = diff_m(cross_idx);
        alpha = -d1 / (d2 - d1 + eps);
        boundary = r1 + alpha * (r2 - r1);
        flag = "INTERPOLATED";
    end

    transition.boundary_rate_deg_s = boundary;
    transition.confidence_flag = flag;
    transition.summary_table = table(rates, aero_m, delay_m, diff_m, 'VariableNames', ...
        {'rate_center_deg_s','mean_aero_residual_deg','mean_delay_residual_deg','delay_minus_aero_deg'});
end

function plot_regime_divergence(results, transition)
    rates = unique(results.rate_center_deg_s);
    aero_m = zeros(numel(rates), 1);
    delay_m = zeros(numel(rates), 1);

    for i = 1:numel(rates)
        idx = results.rate_center_deg_s == rates(i);
        aero_m(i) = mean(results.aero_only_residual_deg(idx));
        delay_m(i) = mean(results.delay_only_residual_deg(idx));
    end

    figure('Name', 'Regime Divergence Contributions');
    hold on; grid on;
    plot(rates, aero_m, 'o-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Aero-only residual');
    plot(rates, delay_m, 's-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Delay-only residual');
    xline(transition.boundary_rate_deg_s, '--r', 'LineWidth', 2, 'DisplayName', 'Estimated transition');
    xlabel('Angular-rate regime center (deg/s)');
    ylabel('Mean theta residual (deg RMS)');
    title('Aero vs Delay Error Contribution by Regime');
    legend('Location', 'northwest');
end

function plot_residual_map(results)
    figure('Name', 'Residual Map');
    scatter(results.rate_center_deg_s, results.rms_theta_residual_deg, 45, results.phase_lag_divergence_ms, 'filled');
    grid on;
    xlabel('Rate regime center (deg/s)');
    ylabel('Theta residual (deg RMS)');
    title('Simulation Fidelity Residual Map (Color = Phase Lag Divergence ms)');
    cb = colorbar;
    cb.Label.String = 'Phase lag divergence (ms)';
end