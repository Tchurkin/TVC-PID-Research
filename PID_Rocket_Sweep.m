%% ========================================================================
%% TVC RESEARCH PIPELINE - PURE MATLAB (NO SIMULINK)
%% Regime-Dependent Limits of Simulation Fidelity Under Delay + CP-CG Aero Instability
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
cfg.rho_kg_m3 = 1.18;
cfg.ref_area_m2 = 0.0031;
cfg.body_length_m = 1.05;
cfg.body_diameter_m = 0.075;
cfg.cg_from_nose_m = 0.58;
cfg.cp_base_from_nose_m = 0.67;
cfg.cn_alpha_nom = 2.4;
cfg.cm_q_nom = 0.22;
cfg.cp_alpha_gain_m_per_rad = 0.050;
cfg.cp_rate_gain_m_per_rad_s = 0.0010;
cfg.cp_low_speed_forward_shift_m = 0.060;
cfg.cp_speed_decay_m_s = 14.0;
cfg.delay_rate_coupling = 0.020; % s/rad, effective actuator-lag growth at high rate

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
cfg.bootstrap_iterations = 400;
cfg.random_seed = 42;
cfg.delay_ratio_strong = 2.5;
cfg.aero_ratio_strong = 0.7;
cfg.margin_near_zero_deg = 0.8;
cfg.model_mismatch_share_high = 0.10;
cfg.mismatch_threshold_for_aero_model = 0.05;
cfg.cp_ahead_share_alert = 0.12;

% Reproducible bootstrap confidence intervals across runs
rng(cfg.random_seed, 'twister');

fprintf('Time step: %.4f s, horizon: %.1f s\n', cfg.dt, cfg.t_final);
fprintf('Design points (cases): %d\n\n', numel(cfg.tau_actuator_ms) * numel(cfg.tau_sensor_filter_ms) * numel(cfg.aero_damping_scale) * numel(cfg.thrust_scale));

%% ------------------------------------------------------------------------
%% 1) STEP PLAN (EXECUTION ORDER)
%% ------------------------------------------------------------------------

plan_steps = {
    '1. Freeze gains and physical constants used by all runs.'
    '2. Build sweep over delay, CP migration dynamics, and thrust variability.'
    '3. Simulate high-fidelity truth trajectories in pure MATLAB code.'
    '4. Simulate simplified predictor trajectories with fixed baseline assumptions.'
    '5. Compute trajectory residuals and phase-lag divergence per regime.'
    '6. Run ablation: delay-only effect and aero-only effect.'
    '7. Restrict comparison to matched non-baseline perturbations.'
    '8. Estimate regime boundary with bootstrap confidence intervals.'
    '9. Export tables/figures for STS writing and flight-test integration.'
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
regime_stats = build_regime_statistics(results, cfg);

%% ------------------------------------------------------------------------
%% 4) ESTIMATE REGIME TRANSITION BOUNDARY
%% ------------------------------------------------------------------------

transition = estimate_transition_boundary(regime_stats);
study_summary = build_study_summary(regime_stats, transition);
sweep_reco = build_sweep_recommendation(cfg, study_summary);
regime_atlas = build_regime_atlas(results, regime_stats, cfg);
aero_threshold = build_aero_model_requirement_summary(results, cfg);
builder_guidance = build_builder_guidance(regime_atlas, cfg);

fprintf('Boundary estimate (delay-dominated onset): %.1f deg/s\n', transition.boundary_rate_deg_s);
fprintf('Boundary confidence flag: %s\n\n', transition.confidence_flag);
fprintf('Mean stability prediction mismatch (nominal vs truth): %.2f%%\n\n', 100 * mean(results.stability_mismatch));

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
writetable(regime_stats, fullfile(out_dir, 'regime_effect_summary.csv'));
writetable(transition.summary_table, fullfile(out_dir, 'regime_transition_summary.csv'));
writetable(study_summary, fullfile(out_dir, 'study_claim_summary.csv'));
writetable(sweep_reco, fullfile(out_dir, 'sweep_expansion_recommendation.csv'));
writetable(regime_atlas, fullfile(out_dir, 'control_limitation_regime_atlas.csv'));
writetable(aero_threshold, fullfile(out_dir, 'aerodynamic_modeling_thresholds.csv'));
writetable(builder_guidance, fullfile(out_dir, 'builder_modeling_guidance_by_regime.csv'));

fprintf('Saved outputs:\n');
fprintf('  - outputs/sim_design_table.csv\n');
fprintf('  - outputs/flight_manifest_template.csv\n');
fprintf('  - outputs/fidelity_results_by_regime.csv\n');
fprintf('  - outputs/regime_effect_summary.csv\n');
fprintf('  - outputs/regime_transition_summary.csv\n\n');
fprintf('  - outputs/study_claim_summary.csv\n\n');
fprintf('  - outputs/sweep_expansion_recommendation.csv\n\n');
fprintf('  - outputs/control_limitation_regime_atlas.csv\n\n');
fprintf('  - outputs/aerodynamic_modeling_thresholds.csv\n\n');
fprintf('  - outputs/builder_modeling_guidance_by_regime.csv\n\n');

%% ------------------------------------------------------------------------
%% 6) PLOTS FOR STS DRAFTING
%% ------------------------------------------------------------------------

plot_regime_divergence(regime_stats, transition);
plot_residual_map(results);
plot_stability_mismatch(regime_stats, transition);
plot_cp_instability_requirement(results, cfg);

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
    results.delay_perturbed = false(n_rows, 1);
    results.aero_perturbed = false(n_rows, 1);
    results.comparison_valid = false(n_rows, 1);
    results.effect_margin_deg = zeros(n_rows, 1);
    results.effect_ratio = zeros(n_rows, 1);
    results.unstable_pred_nominal = false(n_rows, 1);
    results.stability_mismatch = false(n_rows, 1);
    results.dominant_source = strings(n_rows, 1);
    results.control_limitation_regime = strings(n_rows, 1);
    results.cp_ahead_share_truth = zeros(n_rows, 1);
    results.mean_static_margin_cal_truth = zeros(n_rows, 1);
    results.mean_abs_alpha_deg_truth = zeros(n_rows, 1);
    results.aero_model_required = false(n_rows, 1);

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
            results.delay_perturbed(row) = case_params.tau_actuator_ms ~= cfg.base_tau_actuator_ms || ...
                                           case_params.tau_sensor_ms ~= cfg.base_tau_sensor_ms;
            results.aero_perturbed(row) = case_params.aero_scale ~= cfg.base_aero_scale || ...
                                          case_params.thrust_scale ~= cfg.base_thrust_scale;
            results.comparison_valid(row) = results.delay_perturbed(row) && results.aero_perturbed(row);
            results.effect_margin_deg(row) = results.delay_only_residual_deg(row) - results.aero_only_residual_deg(row);
            results.effect_ratio(row) = results.delay_only_residual_deg(row) / max(results.aero_only_residual_deg(row), eps);
            results.unstable_pred_nominal(row) = full.unstable_pred_nominal;
            results.stability_mismatch(row) = results.unstable_truth(row) ~= results.unstable_pred_nominal(row);
            results.cp_ahead_share_truth(row) = full.cp_ahead_share_truth;
            results.mean_static_margin_cal_truth(row) = full.mean_static_margin_cal_truth;
            results.mean_abs_alpha_deg_truth(row) = full.mean_abs_alpha_deg_truth;
            results.aero_model_required(row) = full.cp_ahead_share_truth >= cfg.cp_ahead_share_alert || ...
                                               results.stability_mismatch(row);

            if results.comparison_valid(row) && results.delay_only_residual_deg(row) > results.aero_only_residual_deg(row)
                results.dominant_source(row) = "DELAY";
            elseif results.comparison_valid(row)
                results.dominant_source(row) = "AERO";
            else
                results.dominant_source(row) = "UNMATCHED";
            end

            results.control_limitation_regime(row) = classify_case_regime(...
                results.effect_ratio(row), ...
                results.effect_margin_deg(row), ...
                results.stability_mismatch(row), ...
                results.comparison_valid(row), ...
                cfg);
            row = row + 1;
        end
    end
end

function label = classify_case_regime(effect_ratio, effect_margin_deg, stability_mismatch, comparison_valid, cfg)
    if ~comparison_valid
        label = "UNMATCHED";
        return;
    end

    if stability_mismatch && abs(effect_margin_deg) <= cfg.margin_near_zero_deg
        label = "MODEL_MISMATCH_LIMITED";
    elseif effect_ratio >= cfg.delay_ratio_strong
        label = "DELAY_LIMITED";
    elseif effect_ratio <= cfg.aero_ratio_strong
        label = "AERO_LIMITED";
    else
        label = "MIXED_INTERACTION_LIMITED";
    end
end

function regime_stats = build_regime_statistics(results, cfg)
    rates = unique(results.rate_center_deg_s);
    n = numel(rates);

    regime_stats = table;
    regime_stats.rate_center_deg_s = zeros(n, 1);
    regime_stats.valid_rows = zeros(n, 1);
    regime_stats.mean_aero_residual_deg = zeros(n, 1);
    regime_stats.mean_delay_residual_deg = zeros(n, 1);
    regime_stats.mean_effect_margin_deg = zeros(n, 1);
    regime_stats.median_effect_ratio = zeros(n, 1);
    regime_stats.share_delay_dominant = zeros(n, 1);
    regime_stats.margin_ci_low_deg = zeros(n, 1);
    regime_stats.margin_ci_high_deg = zeros(n, 1);
    regime_stats.delay_ci_low_deg = zeros(n, 1);
    regime_stats.delay_ci_high_deg = zeros(n, 1);
    regime_stats.aero_ci_low_deg = zeros(n, 1);
    regime_stats.aero_ci_high_deg = zeros(n, 1);
    regime_stats.unstable_share = zeros(n, 1);
    regime_stats.stability_mismatch_share = zeros(n, 1);
    regime_stats.mismatch_ci_low = zeros(n, 1);
    regime_stats.mismatch_ci_high = zeros(n, 1);
    regime_stats.mean_cp_ahead_share_truth = zeros(n, 1);
    regime_stats.mean_static_margin_cal_truth = zeros(n, 1);
    regime_stats.cp_ahead_ci_low = zeros(n, 1);
    regime_stats.cp_ahead_ci_high = zeros(n, 1);

    for i = 1:n
        idx = results.rate_center_deg_s == rates(i) & results.comparison_valid;
        rows = results(idx, :);

        regime_stats.rate_center_deg_s(i) = rates(i);
        regime_stats.valid_rows(i) = height(rows);

        if isempty(rows)
            regime_stats.mean_aero_residual_deg(i) = nan;
            regime_stats.mean_delay_residual_deg(i) = nan;
            regime_stats.mean_effect_margin_deg(i) = nan;
            regime_stats.median_effect_ratio(i) = nan;
            regime_stats.share_delay_dominant(i) = nan;
            regime_stats.margin_ci_low_deg(i) = nan;
            regime_stats.margin_ci_high_deg(i) = nan;
            regime_stats.delay_ci_low_deg(i) = nan;
            regime_stats.delay_ci_high_deg(i) = nan;
            regime_stats.aero_ci_low_deg(i) = nan;
            regime_stats.aero_ci_high_deg(i) = nan;
            regime_stats.unstable_share(i) = nan;
            regime_stats.stability_mismatch_share(i) = nan;
            regime_stats.mismatch_ci_low(i) = nan;
            regime_stats.mismatch_ci_high(i) = nan;
            regime_stats.mean_cp_ahead_share_truth(i) = nan;
            regime_stats.mean_static_margin_cal_truth(i) = nan;
            regime_stats.cp_ahead_ci_low(i) = nan;
            regime_stats.cp_ahead_ci_high(i) = nan;
            continue;
        end

        aero_vals = rows.aero_only_residual_deg;
        delay_vals = rows.delay_only_residual_deg;
        margin_vals = rows.effect_margin_deg;

        regime_stats.mean_aero_residual_deg(i) = mean(aero_vals);
        regime_stats.mean_delay_residual_deg(i) = mean(delay_vals);
        regime_stats.mean_effect_margin_deg(i) = mean(margin_vals);
        regime_stats.median_effect_ratio(i) = median(rows.effect_ratio);
        regime_stats.share_delay_dominant(i) = mean(rows.effect_margin_deg > 0);
        regime_stats.unstable_share(i) = mean(rows.unstable_truth);
        regime_stats.stability_mismatch_share(i) = mean(rows.stability_mismatch);
        regime_stats.mean_cp_ahead_share_truth(i) = mean(rows.cp_ahead_share_truth);
        regime_stats.mean_static_margin_cal_truth(i) = mean(rows.mean_static_margin_cal_truth);

        [regime_stats.margin_ci_low_deg(i), regime_stats.margin_ci_high_deg(i)] = bootstrap_mean_ci(margin_vals, cfg.bootstrap_iterations);
        [regime_stats.delay_ci_low_deg(i), regime_stats.delay_ci_high_deg(i)] = bootstrap_mean_ci(delay_vals, cfg.bootstrap_iterations);
        [regime_stats.aero_ci_low_deg(i), regime_stats.aero_ci_high_deg(i)] = bootstrap_mean_ci(aero_vals, cfg.bootstrap_iterations);
        [regime_stats.mismatch_ci_low(i), regime_stats.mismatch_ci_high(i)] = bootstrap_mean_ci(double(rows.stability_mismatch), cfg.bootstrap_iterations);
        [regime_stats.cp_ahead_ci_low(i), regime_stats.cp_ahead_ci_high(i)] = bootstrap_mean_ci(rows.cp_ahead_share_truth, cfg.bootstrap_iterations);
    end
end

function out = evaluate_case(case_params, rate_center_deg_s, cfg, mode)
    nominal = make_params(cfg.base_tau_actuator_ms, cfg.base_tau_sensor_ms, cfg.base_aero_scale, cfg.base_thrust_scale, false, false);

    switch mode
        case 'full'
            truth = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, case_params.aero_scale, case_params.thrust_scale, true, true);
        case 'aero_only'
            truth = make_params(cfg.base_tau_actuator_ms, cfg.base_tau_sensor_ms, case_params.aero_scale, case_params.thrust_scale, true, true);
        case 'delay_only'
            truth = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, cfg.base_aero_scale, cfg.base_thrust_scale, true, true);
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
    out.unstable_pred_nominal = pred_traj.unstable;
    out.cp_ahead_share_truth = mean(truth_traj.cp_minus_cg_m < 0);
    out.mean_static_margin_cal_truth = mean(truth_traj.cp_minus_cg_m / cfg.body_diameter_m);
    out.mean_abs_alpha_deg_truth = mean(abs(rad2deg(truth_traj.alpha_rad)));
end

function p = make_params(tau_act_ms, tau_sensor_ms, aero_scale, thrust_scale, use_high_fidelity_aero, use_delay_interaction)
    p.tau_act_s = tau_act_ms / 1000;
    p.tau_sensor_s = tau_sensor_ms / 1000;
    p.aero_scale = aero_scale;
    p.thrust_scale = thrust_scale;
    p.use_high_fidelity_aero = use_high_fidelity_aero;
    p.use_delay_interaction = use_delay_interaction;
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
    alpha_hist = zeros(n, 1);
    cp_minus_cg_hist = zeros(n, 1);

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
        if p.use_delay_interaction
            tau_act = tau_act * (1 + cfg.delay_rate_coupling * abs(q_hat));
        end
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
        speed = sqrt(vx * vx + vz * vz);

        if speed > 0.5
            gamma = atan2(vz, vx);
        else
            gamma = th;
        end
        alpha = wrap_to_pi_local(th - gamma);

        cp_from_nose = cfg.cp_base_from_nose_m;
        if p.use_high_fidelity_aero
            cp_from_nose = cp_from_nose ...
                - p.aero_scale * cfg.cp_alpha_gain_m_per_rad * abs(alpha) ...
                - p.aero_scale * cfg.cp_rate_gain_m_per_rad_s * abs(qk) ...
                - p.aero_scale * cfg.cp_low_speed_forward_shift_m * exp(-speed / cfg.cp_speed_decay_m_s);
        end

        cp_from_nose = min(cfg.body_length_m, max(0, cp_from_nose));
        cp_minus_cg = cp_from_nose - cfg.cg_from_nose_m;
        alpha_hist(k) = alpha;
        cp_minus_cg_hist(k) = cp_minus_cg;

        q_bar = 0.5 * cfg.rho_kg_m3 * speed * speed;
        cn_alpha = cfg.cn_alpha_nom * (0.8 + 0.2 * p.aero_scale);
        f_aero_normal = q_bar * cfg.ref_area_m2 * cn_alpha * alpha;
        m_aero_static = -f_aero_normal * cp_minus_cg;
        m_aero_damp = -q_bar * cfg.ref_area_m2 * cfg.body_length_m * cfg.body_length_m * cfg.cm_q_nom * p.aero_scale * qk;
        m_aero = m_aero_static + m_aero_damp;
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
    traj.alpha_rad = alpha_hist;
    traj.cp_minus_cg_m = cp_minus_cg_hist;
    traj.unstable = unstable;
end

function angle = wrap_to_pi_local(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

function [rms_theta_deg, rms_rate_deg_s, lag_ms] = compute_divergence_metrics(truth_traj, pred_traj, dt)
    th_err = rad2deg(truth_traj.theta - pred_traj.theta);
    q_err = rad2deg(truth_traj.q - pred_traj.q);

    rms_theta_deg = sqrt(mean(th_err .^ 2));
    rms_rate_deg_s = sqrt(mean(q_err .^ 2));

    lag_ms = estimate_lag_ms(truth_traj.theta, pred_traj.theta, dt);
end

function lag_ms = estimate_lag_ms(sig_a, sig_b, dt)
    % Base MATLAB lag estimator using normalized dot-product scanning.
    max_lag_samples = min(300, floor(numel(sig_a) / 3));
    a = sig_a - mean(sig_a);
    b = sig_b - mean(sig_b);

    best_score = -inf;
    best_lag = 0;

    for lag = -max_lag_samples:max_lag_samples
        if lag >= 0
            a_seg = a(1+lag:end);
            b_seg = b(1:end-lag);
        else
            k = -lag;
            a_seg = a(1:end-k);
            b_seg = b(1+k:end);
        end

        denom = norm(a_seg) * norm(b_seg) + eps;
        score = (a_seg' * b_seg) / denom;

        if score > best_score
            best_score = score;
            best_lag = lag;
        end
    end

    lag_ms = abs(best_lag * dt) * 1000;
end

function transition = estimate_transition_boundary(regime_stats)
    rates = regime_stats.rate_center_deg_s;
    diff_m = regime_stats.mean_effect_margin_deg;
    ci_low = regime_stats.margin_ci_low_deg;
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

    if all(ci_low > 0)
        sig_flag = "DELAY_DOMINANCE_SIGNIFICANT";
    elseif any(ci_low > 0)
        sig_flag = "PARTIAL_SIGNIFICANCE";
    else
        sig_flag = "NOT_SIGNIFICANT";
    end

    transition.boundary_rate_deg_s = boundary;
    transition.confidence_flag = string(flag) + "_" + string(sig_flag);
    transition.summary_table = regime_stats;
end

function summary = build_study_summary(regime_stats, transition)
    margin_vals = regime_stats.mean_effect_margin_deg;
    crossover_observed = any(margin_vals < 0) && any(margin_vals > 0);
    if crossover_observed
        alignment_flag = "DIRECT_BOUNDARY_IDENTIFIABLE";
        primary_question = "Where is the aero-to-delay instability transition boundary in TVC control?";
        alternative_question = "How does the delay-aero boundary shift with operating angular-rate regime?";
    else
        alignment_flag = "NO_CROSSOVER_IN_TESTED_SPACE_EXPAND_SWEEP";
        primary_question = "When do simplified TVC models make wrong stability decisions despite matched parameter tuning?";
        alternative_question = "Which control-limitation regimes dominate TVC prediction error across operating conditions?";
    end

    summary = table;
    summary.boundary_rate_deg_s = transition.boundary_rate_deg_s;
    summary.boundary_confidence_flag = string(transition.confidence_flag);
    summary.all_regimes_delay_dominant = all(regime_stats.mean_effect_margin_deg > 0);
    summary.all_regimes_margin_ci_above_zero = all(regime_stats.margin_ci_low_deg > 0);
    summary.mean_delay_to_aero_ratio = mean(regime_stats.median_effect_ratio, 'omitnan');
    summary.mean_stability_mismatch_share = mean(regime_stats.stability_mismatch_share, 'omitnan');
    summary.worst_regime_mismatch_share = max(regime_stats.stability_mismatch_share);
    summary.mean_cp_ahead_share_truth = mean(regime_stats.mean_cp_ahead_share_truth, 'omitnan');
    summary.worst_cp_ahead_share_truth = max(regime_stats.mean_cp_ahead_share_truth);
    summary.mean_static_margin_cal_truth = mean(regime_stats.mean_static_margin_cal_truth, 'omitnan');
    summary.crossover_observed = crossover_observed;
    summary.research_alignment_flag = alignment_flag;
    summary.dataset_readiness_flag = "SIM_ONLY_NEEDS_FLIGHT_VALIDATION";
    summary.primary_research_question = primary_question;
    summary.alternative_research_question = alternative_question;
end

function atlas = build_regime_atlas(results, regime_stats, cfg)
    rates = regime_stats.rate_center_deg_s;
    n = numel(rates);

    atlas = table;
    atlas.rate_center_deg_s = rates;
    atlas.delay_limited_share = zeros(n, 1);
    atlas.aero_limited_share = zeros(n, 1);
    atlas.mixed_interaction_share = zeros(n, 1);
    atlas.model_mismatch_limited_share = zeros(n, 1);
    atlas.unstable_truth_share = zeros(n, 1);
    atlas.stability_mismatch_share = zeros(n, 1);
    atlas.cp_ahead_share_truth = zeros(n, 1);
    atlas.mean_static_margin_cal_truth = zeros(n, 1);
    atlas.dominant_limitation_regime = strings(n, 1);
    atlas.novelty_opportunity_flag = strings(n, 1);

    for i = 1:n
        idx = results.rate_center_deg_s == rates(i) & results.comparison_valid;
        rows = results(idx, :);
        if isempty(rows)
            atlas.delay_limited_share(i) = nan;
            atlas.aero_limited_share(i) = nan;
            atlas.mixed_interaction_share(i) = nan;
            atlas.model_mismatch_limited_share(i) = nan;
            atlas.unstable_truth_share(i) = nan;
            atlas.stability_mismatch_share(i) = nan;
            atlas.cp_ahead_share_truth(i) = nan;
            atlas.mean_static_margin_cal_truth(i) = nan;
            atlas.dominant_limitation_regime(i) = "NO_DATA";
            atlas.novelty_opportunity_flag(i) = "NO_DATA";
            continue;
        end

        atlas.delay_limited_share(i) = mean(rows.control_limitation_regime == "DELAY_LIMITED");
        atlas.aero_limited_share(i) = mean(rows.control_limitation_regime == "AERO_LIMITED");
        atlas.mixed_interaction_share(i) = mean(rows.control_limitation_regime == "MIXED_INTERACTION_LIMITED");
        atlas.model_mismatch_limited_share(i) = mean(rows.control_limitation_regime == "MODEL_MISMATCH_LIMITED");
        atlas.unstable_truth_share(i) = mean(rows.unstable_truth);
        atlas.stability_mismatch_share(i) = mean(rows.stability_mismatch);
        atlas.cp_ahead_share_truth(i) = mean(rows.cp_ahead_share_truth);
        atlas.mean_static_margin_cal_truth(i) = mean(rows.mean_static_margin_cal_truth);

        shares = [atlas.delay_limited_share(i), atlas.aero_limited_share(i), atlas.mixed_interaction_share(i), atlas.model_mismatch_limited_share(i)];
        labels = ["DELAY_LIMITED", "AERO_LIMITED", "MIXED_INTERACTION_LIMITED", "MODEL_MISMATCH_LIMITED"];
        [~, k] = max(shares);
        atlas.dominant_limitation_regime(i) = labels(k);

        if atlas.model_mismatch_limited_share(i) >= cfg.model_mismatch_share_high
            atlas.novelty_opportunity_flag(i) = "SIMPLIFIED_MODEL_BREAKDOWN_ZONE";
        elseif atlas.cp_ahead_share_truth(i) >= cfg.cp_ahead_share_alert
            atlas.novelty_opportunity_flag(i) = "CP_AHEAD_EXPOSURE_ZONE";
        elseif atlas.mixed_interaction_share(i) >= 0.25
            atlas.novelty_opportunity_flag(i) = "DELAY_AERO_INTERACTION_ZONE";
        elseif atlas.aero_limited_share(i) >= 0.20
            atlas.novelty_opportunity_flag(i) = "AERO_SENSITIVITY_ZONE";
        else
            atlas.novelty_opportunity_flag(i) = "DELAY_DOMINANT_ZONE";
        end
    end
end

function threshold = build_aero_model_requirement_summary(results, cfg)
    rows = results(results.comparison_valid, :);

    edges = 0:0.05:1.0;
    n_bins = numel(edges) - 1;
    cp_center = zeros(n_bins, 1);
    mismatch_share = nan(n_bins, 1);
    count = zeros(n_bins, 1);

    for i = 1:n_bins
        lo = edges(i);
        hi = edges(i+1);
        idx = rows.cp_ahead_share_truth >= lo & rows.cp_ahead_share_truth < hi;
        bin_rows = rows(idx, :);
        cp_center(i) = 0.5 * (lo + hi);
        count(i) = height(bin_rows);
        if ~isempty(bin_rows)
            mismatch_share(i) = mean(bin_rows.stability_mismatch);
        end
    end

    valid = ~isnan(mismatch_share) & count >= 8;
    idx_need = find(valid & mismatch_share >= cfg.mismatch_threshold_for_aero_model, 1, 'first');
    if isempty(idx_need)
        cp_need = nan;
        rec = "CURRENT_SWEEP_BELOW_REQUIRED_MISMATCH_THRESHOLD";
    else
        cp_need = cp_center(idx_need);
        rec = "MODEL_CP_SHIFT_WHEN_CP_AHEAD_SHARE_EXCEEDS_THRESHOLD";
    end

    threshold = table;
    threshold.mismatch_threshold = cfg.mismatch_threshold_for_aero_model;
    threshold.cp_ahead_share_for_aero_model = cp_need;
    threshold.recommendation = rec;
    threshold.mean_cp_ahead_share = mean(rows.cp_ahead_share_truth, 'omitnan');
    threshold.worst_cp_ahead_share = max(rows.cp_ahead_share_truth);
    threshold.mean_static_margin_cal = mean(rows.mean_static_margin_cal_truth, 'omitnan');
    threshold.pct_cases_cp_ahead_over_alert = mean(rows.cp_ahead_share_truth >= cfg.cp_ahead_share_alert);
end

function guidance = build_builder_guidance(regime_atlas, cfg)
    n = height(regime_atlas);
    guidance = table;
    guidance.rate_center_deg_s = regime_atlas.rate_center_deg_s;
    guidance.dominant_limitation_regime = regime_atlas.dominant_limitation_regime;
    guidance.cp_ahead_share_truth = regime_atlas.cp_ahead_share_truth;
    guidance.stability_mismatch_share = regime_atlas.stability_mismatch_share;
    guidance.required_aero_model = strings(n, 1);
    guidance.required_delay_model = strings(n, 1);
    guidance.actionable_recommendation = strings(n, 1);

    for i = 1:n
        cp_share = regime_atlas.cp_ahead_share_truth(i);
        mismatch = regime_atlas.stability_mismatch_share(i);

        if cp_share >= cfg.cp_ahead_share_alert || mismatch >= cfg.mismatch_threshold_for_aero_model
            guidance.required_aero_model(i) = "MOVING_CP_REQUIRED";
        else
            guidance.required_aero_model(i) = "FIXED_CP_ACCEPTABLE";
        end

        if regime_atlas.delay_limited_share(i) >= 0.65
            guidance.required_delay_model(i) = "RATE_COUPLED_DELAY_MODEL_REQUIRED";
        else
            guidance.required_delay_model(i) = "FIRST_ORDER_DELAY_MODEL_ACCEPTABLE";
        end

        guidance.actionable_recommendation(i) = ...
            "Model delay first; include CP migration whenever CP-ahead exposure or stability mismatch exceeds threshold.";
    end
end

function reco = build_sweep_recommendation(cfg, study_summary)
    reco = table;
    reco.current_tau_actuator_ms = string(sprintf('[%g %g %g %g]', cfg.tau_actuator_ms));
    reco.current_tau_sensor_ms = string(sprintf('[%g %g %g %g]', cfg.tau_sensor_filter_ms));
    reco.current_aero_scale = string(sprintf('[%.2f %.2f %.2f %.2f %.2f]', cfg.aero_damping_scale));
    reco.current_thrust_scale = string(sprintf('[%.2f %.2f %.2f]', cfg.thrust_scale));

    if study_summary.crossover_observed
        reco.recommendation_flag = "CROSSOVER_FOUND_KEEP_CURRENT_SWEEP";
        reco.next_tau_actuator_ms = reco.current_tau_actuator_ms;
        reco.next_tau_sensor_ms = reco.current_tau_sensor_ms;
        reco.next_aero_scale = reco.current_aero_scale;
        reco.next_thrust_scale = reco.current_thrust_scale;
    else
        reco.recommendation_flag = "NO_CROSSOVER_EXPAND_FOR_AERO_DOMINANCE_SEARCH";
        reco.next_tau_actuator_ms = "[4 8 12 16 25]";
        reco.next_tau_sensor_ms = "[1 3 6 10 15]";
        reco.next_aero_scale = "[0.5 0.7 1.0 1.4 1.8 2.2]";
        reco.next_thrust_scale = "[0.85 0.95 1.00 1.05 1.15]";
    end
end

function plot_regime_divergence(regime_stats, transition)
    rates = regime_stats.rate_center_deg_s;
    aero_m = regime_stats.mean_aero_residual_deg;
    delay_m = regime_stats.mean_delay_residual_deg;
    lo = regime_stats.margin_ci_low_deg;
    hi = regime_stats.margin_ci_high_deg;

    figure('Name', 'Regime Divergence Contributions');
    tiledlayout(2, 1);

    nexttile;
    hold on; grid on;
    plot(rates, aero_m, 'o-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Aero-only residual');
    plot(rates, delay_m, 's-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Delay-only residual');
    xline(transition.boundary_rate_deg_s, '--r', 'LineWidth', 2, 'DisplayName', 'Estimated transition');
    xlabel('Angular-rate regime center (deg/s)');
    ylabel('Mean theta residual (deg RMS)');
    title('Aero vs Delay Error Contribution by Regime');
    legend('Location', 'northwest');

    nexttile;
    hold on; grid on;
    patch([rates; flipud(rates)], [lo; flipud(hi)], [0.85 0.9 1.0], ...
        'FaceAlpha', 0.35, 'EdgeColor', 'none', 'DisplayName', '95% bootstrap CI');
    plot(rates, regime_stats.mean_effect_margin_deg, 'd-', 'Color', [0 0.25 0.7], ...
        'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Delay minus aero');
    yline(0, '--k', 'LineWidth', 1.5, 'DisplayName', 'Equal contribution');
    xline(transition.boundary_rate_deg_s, '--r', 'LineWidth', 2, 'DisplayName', 'Estimated transition');
    xlabel('Angular-rate regime center (deg/s)');
    ylabel('Matched effect margin (deg RMS)');
    title('Matched Perturbation Margin with Confidence Interval');
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

function plot_stability_mismatch(regime_stats, transition)
    rates = regime_stats.rate_center_deg_s;
    mismatch = regime_stats.stability_mismatch_share;
    lo = regime_stats.mismatch_ci_low;
    hi = regime_stats.mismatch_ci_high;

    figure('Name', 'Stability Prediction Mismatch by Regime');
    hold on; grid on;
    patch([rates; flipud(rates)], [100*lo; flipud(100*hi)], [1.0 0.9 0.85], ...
        'FaceAlpha', 0.35, 'EdgeColor', 'none', 'DisplayName', '95% bootstrap CI');
    plot(rates, 100*mismatch, 'o-', 'Color', [0.8 0.25 0.1], 'LineWidth', 2, 'MarkerSize', 8, ...
        'DisplayName', 'Stability decision mismatch');
    xline(transition.boundary_rate_deg_s, '--k', 'LineWidth', 1.5, 'DisplayName', 'Estimated transition');
    xlabel('Angular-rate regime center (deg/s)');
    ylabel('Mismatch between truth and nominal (%)');
    title('When Simplified Model Makes Wrong Stability Decisions');
    legend('Location', 'northwest');
end

function plot_cp_instability_requirement(results, cfg)
    rows = results(results.comparison_valid, :);
    figure('Name', 'CP Ahead Exposure vs Stability Mismatch');
    hold on; grid on;

    scatter(rows.cp_ahead_share_truth, double(rows.stability_mismatch), 26, rows.rate_center_deg_s, 'filled');
    yline(cfg.mismatch_threshold_for_aero_model, '--r', 'LineWidth', 1.5, 'DisplayName', 'Mismatch threshold');
    xline(cfg.cp_ahead_share_alert, '--k', 'LineWidth', 1.5, 'DisplayName', 'CP-ahead alert');
    xlabel('Fraction of trajectory with CP ahead of CG');
    ylabel('Stability mismatch (0/1 case outcome)');
    title('When CP-Forward Exposure Forces Aerodynamic Modeling');
    cb = colorbar;
    cb.Label.String = 'Rate regime center (deg/s)';
end

function [ci_low, ci_high] = bootstrap_mean_ci(values, n_boot)
    values = values(:);
    n = numel(values);
    if n == 0
        ci_low = nan;
        ci_high = nan;
        return;
    end

    boot_means = zeros(n_boot, 1);
    for b = 1:n_boot
        idx = randi(n, n, 1);
        boot_means(b) = mean(values(idx));
    end

    boot_means = sort(boot_means);
    low_idx = max(1, round(0.025 * n_boot));
    high_idx = min(n_boot, round(0.975 * n_boot));
    ci_low = boot_means(low_idx);
    ci_high = boot_means(high_idx);
end