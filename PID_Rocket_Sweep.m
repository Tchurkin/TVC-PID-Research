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
% CP is treated as fixed within each flight, but varied across cases.
% aero_scale maps to a per-case CP position shift around cp_base_from_nose_m.
cfg.cp_case_shift_gain_m = 0.40;
cfg.delay_rate_coupling = 0.020; % s/rad, effective actuator-lag growth at high rate
cfg.actuator_slew_limit_deg_s = 900;

% Controller gains (hold fixed during campaign)
cfg.kp = 8.2;
cfg.ki = 1.1;
cfg.kd = 0.35;
cfg.actuator_limit_deg = 10;
cfg.controller_names = {'PID', 'ADRC', 'AI'};

% ADRC / DLESO settings for controller-comparison sweep
cfg.adrc_b0 = cfg.thrust_nom_N * cfg.lever_arm_m / cfg.Iyy;
cfg.adrc_wc = 4.0;
cfg.adrc_wo = 12.0;
cfg.adrc_ki_scale = 0.06;

% Auto-tuning settings for fair PID/ADRC comparison under same constraints.
cfg.auto_tune_controllers = true;
cfg.tune_max_cases = 18;
cfg.pid_kp_candidates = [5.5, 7.0, 8.2, 9.5, 11.0];
cfg.pid_ki_candidates = [0.0, 0.4, 0.8, 1.1, 1.5];
cfg.pid_kd_candidates = [0.15, 0.30, 0.45, 0.65, 0.90];
cfg.adrc_wc_candidates = [2.6, 3.4, 4.2, 5.0, 6.0];
cfg.adrc_wo_candidates = [8, 10, 12, 14, 16];
cfg.adrc_ki_scale_candidates = [0.02, 0.04, 0.06, 0.09, 0.12];
cfg.tune_rate_weight = 0.02;
cfg.tune_final_weight = 0.8;
cfg.tune_overshoot_weight = 1.0;
cfg.tune_oscillation_weight = 0.08;
cfg.tune_slew_tracking_weight = 0.35;

% AI controller settings (direct state->motor-angle RL policy)
cfg.ai_enabled = true;
cfg.ai_train_max_cases = 120;
cfg.ai_unstable_penalty = 100;
cfg.ai_use_rl_toolbox_preferred = true;
cfg.ai_rl_requested_episodes = 220;
cfg.ai_rl_alpha = 0.20;
cfg.ai_rl_gamma = 0.70;
cfg.ai_rl_epsilon_start = 0.35;
cfg.ai_rl_epsilon_end = 0.02;
cfg.ai_action_bins_deg = linspace(-cfg.actuator_limit_deg, cfg.actuator_limit_deg, 11);
cfg.ai_theta_edges_deg = [-45, -20, -10, -5, -2, -1, 0, 1, 2, 5, 10, 20, 45];
cfg.ai_q_edges_deg_s = [-450, -180, -90, -45, -20, -10, 0, 10, 20, 45, 90, 180, 450];
cfg.ai_delta_edges_deg = [-10, 10];

% AI-focused outputs only (keep the most relevant plots for this direction).
cfg.ai_focus_outputs_only = true;

% Sweep variables from research abstract
cfg.tau_actuator_ms = [8, 15, 25, 40];
cfg.tau_sensor_filter_ms = [3, 8, 15, 25];
cfg.processing_delay_ms = [1, 3, 6, 10];
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
cfg.base_processing_delay_ms = 1;
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
cfg.save_plot_files = true;
cfg.fast_mode = true;

% Optional legacy analyses are disabled in fast mode to keep iteration speed high.
cfg.include_flight_manifest = false;
cfg.include_component_ablation = false;
cfg.include_delay_breakdown = false;
cfg.include_extreme_exploration = false;
cfg.include_legacy_controller_summary_plot = false;
cfg.include_controller_showcases = false;
cfg.include_parameter_heatmaps = false;

if cfg.fast_mode
    cfg.bootstrap_iterations = 120;
    cfg.tau_actuator_ms = [8, 25];
    cfg.tau_sensor_filter_ms = [3, 15];
    cfg.processing_delay_ms = [1, 6];
    cfg.aero_damping_scale = [0.85, 1.00, 1.30];
    cfg.thrust_scale = [0.90, 1.10];
    cfg.ai_train_max_cases = 48;
    cfg.tune_max_cases = 12;
    cfg.ai_rl_requested_episodes = 120;
end

% Control bandwidth for phase-lag normalization of delay components.
% Actuator and sensor use first-order lag: phi = atan(2*pi*f_bw*tau).
% Processing uses ZOH approximation: phi = 180 * f_bw * T (degrees).
cfg.control_bandwidth_hz = 3.0;

% Reproducible bootstrap confidence intervals across runs
rng(cfg.random_seed, 'twister');

fprintf('Time step: %.4f s, horizon: %.1f s\n', cfg.dt, cfg.t_final);
fprintf('Design points (cases): %d\n\n', numel(cfg.tau_actuator_ms) * numel(cfg.tau_sensor_filter_ms) * numel(cfg.processing_delay_ms) * numel(cfg.aero_damping_scale) * numel(cfg.thrust_scale));

%% ------------------------------------------------------------------------
%% 1) STEP PLAN (EXECUTION ORDER)
%% ------------------------------------------------------------------------

plan_steps = {
    '1. Freeze gains and physical constants used by all runs.'
    '2. Build sweep over delay, fixed CP-CG placement across cases, and thrust variability.'
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
if cfg.include_flight_manifest
    flight_manifest = build_flight_manifest(15, cfg);
else
    flight_manifest = table;
end

fprintf('Sweep table rows: %d\n', height(sim_table));
if cfg.include_flight_manifest
    fprintf('Flight manifest rows: %d\n\n', height(flight_manifest));
else
    fprintf('Flight manifest rows: skipped in fast mode\n\n');
end

%% ------------------------------------------------------------------------
%% 3) RUN CODE-ONLY FIDELITY SWEEP
%% ------------------------------------------------------------------------

results = run_fidelity_sweep(sim_table, cfg);
regime_stats = build_regime_statistics(results, cfg);
if cfg.auto_tune_controllers
    fprintf('Auto-tuning PID and ADRC gains (slew-aware objective)...\n');
    [cfg, tuning_summary] = auto_tune_pid_adrc(sim_table, cfg);
    fprintf('  PID tuned to kp=%.3f, ki=%.3f, kd=%.3f\n', cfg.kp, cfg.ki, cfg.kd);
    fprintf('  ADRC tuned to wc=%.3f, wo=%.3f, ki_scale=%.3f\n\n', cfg.adrc_wc, cfg.adrc_wo, cfg.adrc_ki_scale);
else
    tuning_summary = table;
end
if cfg.ai_enabled
    fprintf('Training standalone AI controller model...\n');
    [ai_model, ai_training_summary] = train_ai_controller_model(sim_table, cfg);
    cfg.ai_model = ai_model;
    fprintf('AI training complete. Method: %s, Validation RMSE (angle RMS): %.2f deg\n\n', ai_model.training_method, ai_model.validation_rmse_angle_deg);
else
    cfg.ai_model = struct;
    ai_training_summary = table;
end
controller_results = run_controller_comparison_sweep(sim_table, cfg);
controller_summary = build_controller_summary(controller_results);
controller_uncertainty = build_controller_uncertainty_frontier(controller_results, cfg);

%% ------------------------------------------------------------------------
%% 4) ESTIMATE REGIME TRANSITION BOUNDARY
%% ------------------------------------------------------------------------

transition = estimate_transition_boundary(regime_stats);
study_summary = build_study_summary(regime_stats, transition);
sweep_reco = build_sweep_recommendation(cfg, study_summary);
regime_atlas = build_regime_atlas(results, regime_stats, cfg);
aero_threshold = build_aero_model_requirement_summary(results, cfg);
builder_guidance = build_builder_guidance(regime_atlas, cfg);
if cfg.include_delay_breakdown
    delay_breakdown = build_delay_breakdown_summary(results, cfg);
else
    delay_breakdown = table;
end

if cfg.include_extreme_exploration
    extreme_regimes = explore_extreme_regimes(cfg);
else
    extreme_regimes = table;
end

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

fprintf('Building unified cross-study summary...\n');
try
    unified_summary = build_unified_research_summary(study_summary, regime_atlas, controller_uncertainty, out_dir);
catch ME
    warning('Unified summary generation failed: %s', ME.message);
    unified_summary = build_unified_research_fallback(study_summary);
end
fprintf('Unified summary ready.\n\n');

writetable(sim_table, fullfile(out_dir, 'sim_design_table.csv'));
writetable(results, fullfile(out_dir, 'fidelity_results_by_regime.csv'));
writetable(regime_stats, fullfile(out_dir, 'regime_effect_summary.csv'));
writetable(transition.summary_table, fullfile(out_dir, 'regime_transition_summary.csv'));
writetable(study_summary, fullfile(out_dir, 'study_claim_summary.csv'));
writetable(sweep_reco, fullfile(out_dir, 'sweep_expansion_recommendation.csv'));
writetable(regime_atlas, fullfile(out_dir, 'control_limitation_regime_atlas.csv'));
writetable(aero_threshold, fullfile(out_dir, 'aerodynamic_modeling_thresholds.csv'));
writetable(builder_guidance, fullfile(out_dir, 'builder_modeling_guidance_by_regime.csv'));
writetable(controller_results, fullfile(out_dir, 'controller_results_by_regime.csv'));
writetable(controller_summary, fullfile(out_dir, 'controller_summary_by_regime.csv'));
writetable(controller_uncertainty, fullfile(out_dir, 'controller_uncertainty_frontier.csv'));
writetable(unified_summary, fullfile(out_dir, 'unified_research_summary.csv'));
if cfg.auto_tune_controllers
    writetable(tuning_summary, fullfile(out_dir, 'controller_autotuning_summary.csv'));
end
if cfg.ai_enabled
    writetable(ai_training_summary, fullfile(out_dir, 'ai_controller_training_summary.csv'));
end
if cfg.include_flight_manifest
    writetable(flight_manifest, fullfile(out_dir, 'flight_manifest_template.csv'));
end
if cfg.include_delay_breakdown
    writetable(delay_breakdown, fullfile(out_dir, 'delay_component_priority_summary.csv'));
end
if cfg.include_extreme_exploration
    writetable(extreme_regimes, fullfile(out_dir, 'extreme_regime_exploration.csv'));
end

fprintf('Saved outputs:\n');
fprintf('  - outputs/sim_design_table.csv\n');
fprintf('  - outputs/fidelity_results_by_regime.csv\n');
fprintf('  - outputs/regime_effect_summary.csv\n');
fprintf('  - outputs/regime_transition_summary.csv\n\n');
fprintf('  - outputs/study_claim_summary.csv\n\n');
fprintf('  - outputs/sweep_expansion_recommendation.csv\n\n');
fprintf('  - outputs/control_limitation_regime_atlas.csv\n\n');
fprintf('  - outputs/aerodynamic_modeling_thresholds.csv\n\n');
fprintf('  - outputs/builder_modeling_guidance_by_regime.csv\n\n');
fprintf('  - outputs/controller_results_by_regime.csv\n\n');
fprintf('  - outputs/controller_summary_by_regime.csv\n\n');
fprintf('  - outputs/controller_uncertainty_frontier.csv\n\n');
fprintf('  - outputs/unified_research_summary.csv\n\n');
if cfg.auto_tune_controllers
    fprintf('  - outputs/controller_autotuning_summary.csv\n\n');
end
if cfg.ai_enabled
    fprintf('  - outputs/ai_controller_training_summary.csv\n\n');
end
if cfg.include_flight_manifest
    fprintf('  - outputs/flight_manifest_template.csv\n\n');
end
if cfg.include_delay_breakdown
    fprintf('  - outputs/delay_component_priority_summary.csv\n\n');
end
if cfg.include_extreme_exploration
    fprintf('  - outputs/extreme_regime_exploration.csv\n\n');
end

%% ------------------------------------------------------------------------
%% 6) PLOTS FOR STS DRAFTING
%% ------------------------------------------------------------------------

fig_dir = fullfile(out_dir, 'figures');
if cfg.ai_focus_outputs_only
    plot_pid_adrc_ai_profile(sim_table, cfg, fig_dir);
    plot_ai_direction_summary(controller_summary, fig_dir, cfg);
else
    plot_regime_divergence(regime_stats, transition, fig_dir, cfg);
    plot_residual_map(results, fig_dir, cfg);
    plot_stability_mismatch(regime_stats, transition, fig_dir, cfg);
    plot_cp_instability_requirement(results, cfg, fig_dir);
    plot_regime_limitation_stack(regime_atlas, fig_dir, cfg);
    if cfg.include_parameter_heatmaps
        plot_parameter_heatmaps(results, fig_dir, cfg);
    end
    plot_cp_margin_distributions(results, fig_dir, cfg);
    if cfg.include_legacy_controller_summary_plot
        plot_controller_comparison_summary(controller_summary, fig_dir, cfg);
    end
    plot_controller_uncertainty_frontier(controller_uncertainty, fig_dir, cfg);
    if cfg.include_controller_showcases
        plot_controller_showcases(sim_table, cfg, fig_dir);
    end
    plot_pid_adrc_ai_profile(sim_table, cfg, fig_dir);
end

cleanup_disabled_artifacts(out_dir, fig_dir, cfg);

fprintf('Finished. Next: replace baseline constants with measured hardware values.\n\n');

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================

function tbl = build_simulation_table(cfg)
    [a, s, p, d, t] = ndgrid(cfg.tau_actuator_ms, ...
                         cfg.tau_sensor_filter_ms, ...
                         cfg.processing_delay_ms, ...
                         cfg.aero_damping_scale, ...
                         cfg.thrust_scale);
    tbl = table;
    tbl.case_id = (1:numel(a))';
    tbl.tau_actuator_ms = a(:);
    tbl.tau_sensor_ms = s(:);
    tbl.processing_delay_ms = p(:);
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
    results.processing_delay_ms = zeros(n_rows, 1);
    results.aero_scale = zeros(n_rows, 1);
    results.thrust_scale = zeros(n_rows, 1);
    results.rms_theta_residual_deg = zeros(n_rows, 1);
    results.rms_rate_residual_deg_s = zeros(n_rows, 1);
    results.phase_lag_divergence_ms = zeros(n_rows, 1);
    results.aero_only_residual_deg = zeros(n_rows, 1);
    results.delay_only_residual_deg = zeros(n_rows, 1);
    results.actuator_only_residual_deg = zeros(n_rows, 1);
    results.sensor_only_residual_deg = zeros(n_rows, 1);
    results.processing_only_residual_deg = zeros(n_rows, 1);
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
            if cfg.include_component_ablation
                actuator_only = evaluate_case(case_params, rate_center, cfg, 'actuator_only');
                sensor_only = evaluate_case(case_params, rate_center, cfg, 'sensor_only');
                processing_only = evaluate_case(case_params, rate_center, cfg, 'processing_only');
            else
                actuator_only.rms_theta_residual_deg = nan;
                sensor_only.rms_theta_residual_deg = nan;
                processing_only.rms_theta_residual_deg = nan;
            end

            results.case_id(row) = case_params.case_id;
            results.regime_id(row) = r;
            results.regime_name(row) = string(cfg.regime_names{r});
            results.rate_center_deg_s(row) = rate_center;
            results.tau_actuator_ms(row) = case_params.tau_actuator_ms;
            results.tau_sensor_ms(row) = case_params.tau_sensor_ms;
            results.processing_delay_ms(row) = case_params.processing_delay_ms;
            results.aero_scale(row) = case_params.aero_scale;
            results.thrust_scale(row) = case_params.thrust_scale;
            results.rms_theta_residual_deg(row) = full.rms_theta_residual_deg;
            results.rms_rate_residual_deg_s(row) = full.rms_rate_residual_deg_s;
            results.phase_lag_divergence_ms(row) = full.phase_lag_divergence_ms;
            results.aero_only_residual_deg(row) = aero_only.rms_theta_residual_deg;
            results.delay_only_residual_deg(row) = delay_only.rms_theta_residual_deg;
            results.actuator_only_residual_deg(row) = actuator_only.rms_theta_residual_deg;
            results.sensor_only_residual_deg(row) = sensor_only.rms_theta_residual_deg;
            results.processing_only_residual_deg(row) = processing_only.rms_theta_residual_deg;
            results.unstable_truth(row) = full.unstable_truth;
            results.delay_perturbed(row) = case_params.tau_actuator_ms ~= cfg.base_tau_actuator_ms || ...
                                           case_params.tau_sensor_ms ~= cfg.base_tau_sensor_ms || ...
                                           case_params.processing_delay_ms ~= cfg.base_processing_delay_ms;
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
    nominal = make_params(cfg.base_tau_actuator_ms, cfg.base_tau_sensor_ms, cfg.base_processing_delay_ms, cfg.base_aero_scale, cfg.base_thrust_scale, false, false);

    switch mode
        case 'full'
            truth = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, case_params.processing_delay_ms, case_params.aero_scale, case_params.thrust_scale, true, true);
        case 'aero_only'
            truth = make_params(cfg.base_tau_actuator_ms, cfg.base_tau_sensor_ms, cfg.base_processing_delay_ms, case_params.aero_scale, case_params.thrust_scale, true, true);
        case 'delay_only'
            truth = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, case_params.processing_delay_ms, cfg.base_aero_scale, cfg.base_thrust_scale, true, true);
        case 'actuator_only'
            truth = make_params(case_params.tau_actuator_ms, cfg.base_tau_sensor_ms, cfg.base_processing_delay_ms, cfg.base_aero_scale, cfg.base_thrust_scale, true, true);
        case 'sensor_only'
            truth = make_params(cfg.base_tau_actuator_ms, case_params.tau_sensor_ms, cfg.base_processing_delay_ms, cfg.base_aero_scale, cfg.base_thrust_scale, true, true);
        case 'processing_only'
            truth = make_params(cfg.base_tau_actuator_ms, cfg.base_tau_sensor_ms, case_params.processing_delay_ms, cfg.base_aero_scale, cfg.base_thrust_scale, true, true);
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

function results = run_controller_comparison_sweep(sim_table, cfg)
    n_cases = height(sim_table);
    n_reg = size(cfg.rate_bins, 1);
    n_ctrl = numel(cfg.controller_names);
    n_rows = n_cases * n_reg * n_ctrl;

    results = table;
    results.case_id = zeros(n_rows, 1);
    results.controller = strings(n_rows, 1);
    results.regime_name = strings(n_rows, 1);
    results.rate_center_deg_s = zeros(n_rows, 1);
    results.tau_actuator_ms = zeros(n_rows, 1);
    results.tau_sensor_ms = zeros(n_rows, 1);
    results.processing_delay_ms = zeros(n_rows, 1);
    results.aero_scale = zeros(n_rows, 1);
    results.thrust_scale = zeros(n_rows, 1);
    results.rms_angle_error_deg = zeros(n_rows, 1);
    results.rms_rate_deg_s = zeros(n_rows, 1);
    results.max_abs_angle_deg = zeros(n_rows, 1);
    results.max_abs_delta_deg = zeros(n_rows, 1);
    results.final_error_deg = zeros(n_rows, 1);
    results.unstable = false(n_rows, 1);
    results.precision_success = false(n_rows, 1);
    results.mean_abs_alpha_deg = zeros(n_rows, 1);
    results.cp_ahead_share = zeros(n_rows, 1);
    results.zero_crossings_error = zeros(n_rows, 1);
    results.slew_tracking_error_deg = zeros(n_rows, 1);
    results.overshoot_error_deg = zeros(n_rows, 1);

    row = 1;
    for c = 1:n_cases
        case_params = sim_table(c, :);
        full_params = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, case_params.processing_delay_ms, case_params.aero_scale, case_params.thrust_scale, true, true);
        for r = 1:n_reg
            rate_center = mean(cfg.rate_bins(r, :));
            x0 = make_initial_state(rate_center);
            for ctrl = 1:n_ctrl
                ctrl_name = string(cfg.controller_names{ctrl});
                run_params = full_params;
                if ctrl_name == "AI"
                    run_params.ai_model = cfg.ai_model;
                end
                traj = simulate_tvc(x0, run_params, cfg, ctrl_name);

                results.case_id(row) = case_params.case_id;
                results.controller(row) = ctrl_name;
                results.regime_name(row) = string(cfg.regime_names{r});
                results.rate_center_deg_s(row) = rate_center;
                results.tau_actuator_ms(row) = case_params.tau_actuator_ms;
                results.tau_sensor_ms(row) = case_params.tau_sensor_ms;
                results.processing_delay_ms(row) = case_params.processing_delay_ms;
                results.aero_scale(row) = case_params.aero_scale;
                results.thrust_scale(row) = case_params.thrust_scale;
                results.rms_angle_error_deg(row) = sqrt(mean(rad2deg(traj.err_rad) .^ 2));
                results.rms_rate_deg_s(row) = sqrt(mean(rad2deg(traj.q) .^ 2));
                results.max_abs_angle_deg(row) = max(abs(rad2deg(traj.theta)));
                results.max_abs_delta_deg(row) = max(abs(rad2deg(traj.delta)));
                results.final_error_deg(row) = rad2deg(traj.err_rad(end));
                results.unstable(row) = traj.unstable;
                results.precision_success(row) = ~traj.unstable && results.rms_angle_error_deg(row) <= 2.0 && abs(results.final_error_deg(row)) <= 1.0;
                results.mean_abs_alpha_deg(row) = mean(abs(rad2deg(traj.alpha_rad)));
                results.cp_ahead_share(row) = mean(traj.cp_minus_cg_m < 0);
                err_deg = rad2deg(traj.err_rad);
                results.zero_crossings_error(row) = count_zero_crossings(err_deg);
                results.slew_tracking_error_deg(row) = mean(abs(rad2deg(traj.delta_cmd_rad - traj.delta)));
                results.overshoot_error_deg(row) = max(0, max(err_deg));
                row = row + 1;
            end
        end
    end
end

function summary = build_controller_summary(controller_results)
    [groups, regime_name, controller] = findgroups(controller_results.regime_name, controller_results.controller);
    summary = table;
    summary.regime_name = regime_name;
    summary.controller = controller;
    summary.mean_rms_angle_error_deg = splitapply(@mean, controller_results.rms_angle_error_deg, groups);
    summary.mean_rms_rate_deg_s = splitapply(@mean, controller_results.rms_rate_deg_s, groups);
    summary.precision_success_share = splitapply(@mean, double(controller_results.precision_success), groups);
    summary.unstable_share = splitapply(@mean, double(controller_results.unstable), groups);
    summary.mean_max_abs_delta_deg = splitapply(@mean, controller_results.max_abs_delta_deg, groups);
    summary.mean_final_error_deg = splitapply(@mean, controller_results.final_error_deg, groups);
end

function summary = build_controller_uncertainty_frontier(controller_results, cfg)
    abs_mismatch = abs(controller_results.aero_scale - cfg.base_aero_scale);
    uncertainty_level = strings(height(controller_results), 1);

    for i = 1:numel(abs_mismatch)
        if abs_mismatch(i) < 1e-6
            uncertainty_level(i) = "NOMINAL";
        elseif abs_mismatch(i) <= 0.16
            uncertainty_level(i) = "MODERATE_MISMATCH";
        else
            uncertainty_level(i) = "HIGH_MISMATCH";
        end
    end

    controller_results = addvars(controller_results, abs_mismatch, uncertainty_level, 'NewVariableNames', {'abs_aero_mismatch', 'uncertainty_level'});
    [groups, uncertainty_level, controller] = findgroups(controller_results.uncertainty_level, controller_results.controller);

    summary = table;
    summary.uncertainty_level = uncertainty_level;
    summary.controller = controller;
    summary.mean_abs_aero_mismatch = splitapply(@mean, controller_results.abs_aero_mismatch, groups);
    summary.mean_rms_angle_error_deg = splitapply(@mean, controller_results.rms_angle_error_deg, groups);
    summary.mean_rms_rate_deg_s = splitapply(@mean, controller_results.rms_rate_deg_s, groups);
    summary.precision_success_share = splitapply(@mean, double(controller_results.precision_success), groups);
    summary.unstable_share = splitapply(@mean, double(controller_results.unstable), groups);
    summary.mean_max_abs_delta_deg = splitapply(@mean, controller_results.max_abs_delta_deg, groups);
    summary.mean_final_error_deg = splitapply(@mean, controller_results.final_error_deg, groups);

    level_order = ["NOMINAL"; "MODERATE_MISMATCH"; "HIGH_MISMATCH"];
    rank = zeros(height(summary), 1);
    for i = 1:height(summary)
        rank(i) = find(level_order == summary.uncertainty_level(i), 1, 'first');
    end
    summary = sortrows(addvars(summary, rank, 'Before', 1, 'NewVariableNames', 'uncertainty_rank'), 'uncertainty_rank');
end

function p = make_params(tau_act_ms, tau_sensor_ms, processing_delay_ms, aero_scale, thrust_scale, use_high_fidelity_aero, use_delay_interaction)
    p.tau_act_s = tau_act_ms / 1000;
    p.tau_sensor_s = tau_sensor_ms / 1000;
    p.processing_delay_s = processing_delay_ms / 1000;
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
    x0.adrc_z1 = x0.theta_hat_rad;
    x0.adrc_z2 = x0.q_hat_rad_s;
    x0.adrc_z3 = 0;
    x0.adrc_i_err = 0;
end

function traj = simulate_tvc(x0, p, cfg, controller_name)
    if nargin < 4
        controller_name = "PID";
    else
        controller_name = string(controller_name);
    end

    n = floor(cfg.t_final / cfg.dt) + 1;

    theta = zeros(n, 1);
    q = zeros(n, 1);
    delta = zeros(n, 1);
    err_hist = zeros(n, 1);
    delta_cmd_hist = zeros(n, 1);
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
    adrc_z1 = x0.adrc_z1;
    adrc_z2 = x0.adrc_z2;
    adrc_z3 = x0.adrc_z3;
    adrc_i_err = x0.adrc_i_err;
    pending_cmd = x0.delta_rad;
    processing_timer = 0;

    unstable = false;
    delta_lim = deg2rad(cfg.actuator_limit_deg);
    slew_rate_limit = deg2rad(cfg.actuator_slew_limit_deg_s);

    % CP is constant during a given flight/case, but differs across cases.
    cp_from_nose_case = cfg.cp_base_from_nose_m + cfg.cp_case_shift_gain_m * (p.aero_scale - 1.0);
    cp_from_nose_case = min(cfg.body_length_m, max(0, cp_from_nose_case));
    cp_minus_cg_case = cp_from_nose_case - cfg.cg_from_nose_m;

    for k = 1:n
        theta(k) = th;
        q(k) = qk;
        delta(k) = dk;

        err_true = -th;
        err_hist(k) = err_true;

        switch controller_name
            case "PID"
                err = -th_hat;
                i_err = i_err + err * cfg.dt;
                d_err = -q_hat;
                delta_cmd_raw = cfg.kp * err + cfg.ki * i_err + cfg.kd * d_err;

            case "ADRC"
                obs_err = adrc_z1 - th_hat;
                beta1 = 3 * cfg.adrc_wo;
                beta2 = 3 * cfg.adrc_wo^2;
                beta3 = cfg.adrc_wo^3;
                b0 = max(1e-3, cfg.adrc_b0 * p.thrust_scale);

                adrc_z1 = adrc_z1 + cfg.dt * (adrc_z2 - beta1 * obs_err);
                adrc_z2 = adrc_z2 + cfg.dt * (adrc_z3 + b0 * dk - beta2 * obs_err);
                adrc_z3 = adrc_z3 + cfg.dt * (-beta3 * obs_err);

                ep = adrc_z1;
                adrc_i_err = adrc_i_err + ep * cfg.dt;
                kp_adrc = cfg.adrc_wc^2;
                kd_adrc = 2 * cfg.adrc_wc;
                ki_adrc = cfg.adrc_ki_scale * cfg.adrc_wc^3;
                v = -kp_adrc * ep - kd_adrc * adrc_z2 - ki_adrc * adrc_i_err;
                delta_cmd_raw = (v - adrc_z3) / b0;

            case "AI"
                ai_model = struct;
                if isfield(p, 'ai_model')
                    ai_model = p.ai_model;
                end
                delta_cmd_raw = ai_policy_action(ai_model, th_hat, q_hat, dk, cfg);

            otherwise
                error('Unknown controller: %s', controller_name);
        end

        delta_cmd_raw = min(max(delta_cmd_raw, -delta_lim), delta_lim);
        delta_cmd_hist(k) = delta_cmd_raw;

        processing_timer = processing_timer - cfg.dt;
        if processing_timer <= 0
            pending_cmd = delta_cmd_raw;
            processing_timer = max(p.processing_delay_s, cfg.dt);
        end

        tau_act = max(p.tau_act_s, 1e-4);
        if p.use_delay_interaction
            tau_act = tau_act * (1 + cfg.delay_rate_coupling * abs(q_hat));
        end
        delta_rate_cmd = (pending_cmd - dk) / tau_act;
        delta_rate_cmd = min(max(delta_rate_cmd, -slew_rate_limit), slew_rate_limit);
        dk = dk + delta_rate_cmd * cfg.dt;
        dk = min(max(dk, -delta_lim), delta_lim);

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

        alpha_hist(k) = alpha;
        cp_minus_cg_hist(k) = cp_minus_cg_case;

        q_bar = 0.5 * cfg.rho_kg_m3 * speed * speed;
        cn_alpha = cfg.cn_alpha_nom * (0.8 + 0.2 * p.aero_scale);
        f_aero_normal = q_bar * cfg.ref_area_m2 * cn_alpha * alpha;
        m_aero_static = -f_aero_normal * cp_minus_cg_case;
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
    traj.err_rad = err_hist;
    traj.delta_cmd_rad = delta_cmd_hist;
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
        rec = "MODEL_FIXED_CP_CG_GEOMETRY_WHEN_CP_AHEAD_EXPOSURE_EXCEEDS_THRESHOLD";
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
            guidance.required_aero_model(i) = "CASE_SPECIFIC_CP_CG_MODEL_REQUIRED";
        else
            guidance.required_aero_model(i) = "FIXED_CP_ACCEPTABLE";
        end

        if regime_atlas.delay_limited_share(i) >= 0.65
            guidance.required_delay_model(i) = "RATE_COUPLED_DELAY_MODEL_REQUIRED";
        else
            guidance.required_delay_model(i) = "FIRST_ORDER_DELAY_MODEL_ACCEPTABLE";
        end

        guidance.actionable_recommendation(i) = ...
            "Model actuator and processing delay first; include case-specific CP-CG geometry whenever CP-ahead exposure or stability mismatch exceeds threshold.";
    end
end

function summary = build_delay_breakdown_summary(results, cfg)
    % Ranking is done two ways:
    %   (A) Raw residual magnitude - reflects importance across the specific
    %       ms ranges swept, but is NOT normalized to a fair common basis.
    %   (B) Phase-normalized sensitivity = residual / phase-lag-range-added
    %       at cfg.control_bandwidth_hz. This IS a fair ranking because it
    %       asks: how much does each delay type hurt per degree of phase lag
    %       it adds to the loop?
    %
    % Actuator and sensor use first-order lag phase:
    %   phi_deg = atan(2*pi*f_bw*tau) * 180/pi
    % Processing uses ZOH (sample-and-hold) phase:
    %   phi_deg = 180 * f_bw * T_s   (half-sample approximation)
    %
    % The sweep RANGE for each type determines the phase budget swept.
    % Baseline tau is subtracted so we measure the cost of increasing
    % delay beyond baseline, which is what the ablation residual captures.

    f_bw = cfg.control_bandwidth_hz;

    % Phase lag added relative to baseline, evaluated at worst-case (max) sweep value.
    phi_act_range_deg = (atan(2*pi*f_bw*(max(cfg.tau_actuator_ms)/1000)) - ...
                         atan(2*pi*f_bw*(cfg.base_tau_actuator_ms/1000))) * 180/pi;
    phi_sensor_range_deg = (atan(2*pi*f_bw*(max(cfg.tau_sensor_filter_ms)/1000)) - ...
                            atan(2*pi*f_bw*(cfg.base_tau_sensor_ms/1000))) * 180/pi;
    phi_proc_range_deg   = 180 * f_bw * (max(cfg.processing_delay_ms)/1000 - ...
                           cfg.base_processing_delay_ms/1000);

    rates = unique(results.rate_center_deg_s);
    n = numel(rates);

    summary = table;
    summary.rate_center_deg_s = rates;
    summary.mean_actuator_residual_deg = zeros(n, 1);
    summary.mean_sensor_residual_deg = zeros(n, 1);
    summary.mean_processing_residual_deg = zeros(n, 1);
    summary.phase_budget_actuator_deg = repmat(phi_act_range_deg, n, 1);
    summary.phase_budget_sensor_deg = repmat(phi_sensor_range_deg, n, 1);
    summary.phase_budget_processing_deg = repmat(phi_proc_range_deg, n, 1);
    summary.sensitivity_per_deg_phase_actuator = zeros(n, 1);
    summary.sensitivity_per_deg_phase_sensor = zeros(n, 1);
    summary.sensitivity_per_deg_phase_processing = zeros(n, 1);
    summary.raw_rank_1 = strings(n, 1);
    summary.raw_rank_2 = strings(n, 1);
    summary.raw_rank_3 = strings(n, 1);
    summary.normalized_rank_1 = strings(n, 1);
    summary.normalized_rank_2 = strings(n, 1);
    summary.normalized_rank_3 = strings(n, 1);
    summary.builder_priority = strings(n, 1);

    labels = ["ACTUATOR_RESPONSE", "SENSOR_FILTERING", "PROCESSING_CYCLE_TIME"];

    for i = 1:n
        rows = results(results.rate_center_deg_s == rates(i) & results.comparison_valid, :);
        raw = [mean(rows.actuator_only_residual_deg), ...
               mean(rows.sensor_only_residual_deg), ...
               mean(rows.processing_only_residual_deg)];
        phase_budgets = [phi_act_range_deg, phi_sensor_range_deg, phi_proc_range_deg];
        normalized = raw ./ max(phase_budgets, 1e-6);

        summary.mean_actuator_residual_deg(i) = raw(1);
        summary.mean_sensor_residual_deg(i) = raw(2);
        summary.mean_processing_residual_deg(i) = raw(3);
        summary.sensitivity_per_deg_phase_actuator(i) = normalized(1);
        summary.sensitivity_per_deg_phase_sensor(i) = normalized(2);
        summary.sensitivity_per_deg_phase_processing(i) = normalized(3);

        [~, raw_order] = sort(raw, 'descend');
        [~, norm_order] = sort(normalized, 'descend');
        summary.raw_rank_1(i) = labels(raw_order(1));
        summary.raw_rank_2(i) = labels(raw_order(2));
        summary.raw_rank_3(i) = labels(raw_order(3));
        summary.normalized_rank_1(i) = labels(norm_order(1));
        summary.normalized_rank_2(i) = labels(norm_order(2));
        summary.normalized_rank_3(i) = labels(norm_order(3));

        % Builder priority uses normalized ranking - physically fair.
        if normalized(norm_order(1)) >= 1.5 * normalized(norm_order(2))
            summary.builder_priority(i) = "MEASURE_" + labels(norm_order(1)) + "_FIRST";
        else
            summary.builder_priority(i) = "MEASURE_" + labels(norm_order(1)) + "_AND_" + labels(norm_order(2)) + "_EQUALLY";
        end
    end

    overall = table;
    overall.rate_center_deg_s = -1;
    overall.mean_actuator_residual_deg = mean(results.actuator_only_residual_deg(results.comparison_valid));
    overall.mean_sensor_residual_deg = mean(results.sensor_only_residual_deg(results.comparison_valid));
    overall.mean_processing_residual_deg = mean(results.processing_only_residual_deg(results.comparison_valid));
    overall.phase_budget_actuator_deg = phi_act_range_deg;
    overall.phase_budget_sensor_deg = phi_sensor_range_deg;
    overall.phase_budget_processing_deg = phi_proc_range_deg;
    raw = [overall.mean_actuator_residual_deg, overall.mean_sensor_residual_deg, overall.mean_processing_residual_deg];
    phase_budgets = [phi_act_range_deg, phi_sensor_range_deg, phi_proc_range_deg];
    normalized = raw ./ max(phase_budgets, 1e-6);
    overall.sensitivity_per_deg_phase_actuator = normalized(1);
    overall.sensitivity_per_deg_phase_sensor = normalized(2);
    overall.sensitivity_per_deg_phase_processing = normalized(3);
    [~, raw_order] = sort(raw, 'descend');
    [~, norm_order] = sort(normalized, 'descend');
    overall.raw_rank_1 = labels(raw_order(1));
    overall.raw_rank_2 = labels(raw_order(2));
    overall.raw_rank_3 = labels(raw_order(3));
    overall.normalized_rank_1 = labels(norm_order(1));
    overall.normalized_rank_2 = labels(norm_order(2));
    overall.normalized_rank_3 = labels(norm_order(3));
    overall.builder_priority = "OVERALL_PRIORITIZE_" + labels(norm_order(1));
    summary = [summary; overall];
end

function extreme = explore_extreme_regimes(cfg)
    rate_centers = [40; 130; 250; 380];
    tau_act_list = [4, 8, 16, 30];
    tau_sensor_list = [1, 3, 8, 16];
    processing_list = [0.5, 1, 4, 10];
    aero_list = [0.35, 0.60, 1.00, 1.50, 2.00, 2.60];
    thrust_list = [0.80, 0.95, 1.05, 1.20];

    n_rows = numel(rate_centers) * numel(tau_act_list) * numel(tau_sensor_list) * numel(processing_list) * numel(aero_list) * numel(thrust_list);
    rate_center_deg_s = zeros(n_rows, 1);
    tau_actuator_ms = zeros(n_rows, 1);
    tau_sensor_ms = zeros(n_rows, 1);
    processing_delay_ms = zeros(n_rows, 1);
    aero_scale = zeros(n_rows, 1);
    thrust_scale = zeros(n_rows, 1);
    effect_margin_deg = zeros(n_rows, 1);
    effect_ratio = zeros(n_rows, 1);
    cp_ahead_share_truth = zeros(n_rows, 1);
    mean_static_margin_cal_truth = zeros(n_rows, 1);
    stability_mismatch = false(n_rows, 1);
    aero_more_important_than_delay = false(n_rows, 1);
    max_cp_forward_m = zeros(n_rows, 1);
    max_cp_forward_cal = zeros(n_rows, 1);

    row = 1;
    for i = 1:numel(rate_centers)
        for j = 1:numel(tau_act_list)
            for k = 1:numel(tau_sensor_list)
                for m = 1:numel(processing_list)
                    for a = 1:numel(aero_list)
                        for t = 1:numel(thrust_list)
                            case_params = struct(...
                                'tau_actuator_ms', tau_act_list(j), ...
                                'tau_sensor_ms', tau_sensor_list(k), ...
                                'processing_delay_ms', processing_list(m), ...
                                'aero_scale', aero_list(a), ...
                                'thrust_scale', thrust_list(t));

                            full = evaluate_case(case_params, rate_centers(i), cfg, 'full');
                            aero_only = evaluate_case(case_params, rate_centers(i), cfg, 'aero_only');
                            delay_only = evaluate_case(case_params, rate_centers(i), cfg, 'delay_only');
                            cp_forward_m = max(0, cfg.cg_from_nose_m - min(cfg.body_length_m, max(0, cfg.cp_base_from_nose_m + cfg.cp_case_shift_gain_m * (case_params.aero_scale - 1.0))));

                            rate_center_deg_s(row) = rate_centers(i);
                            tau_actuator_ms(row) = tau_act_list(j);
                            tau_sensor_ms(row) = tau_sensor_list(k);
                            processing_delay_ms(row) = processing_list(m);
                            aero_scale(row) = aero_list(a);
                            thrust_scale(row) = thrust_list(t);
                            effect_margin_deg(row) = delay_only.rms_theta_residual_deg - aero_only.rms_theta_residual_deg;
                            effect_ratio(row) = delay_only.rms_theta_residual_deg / max(aero_only.rms_theta_residual_deg, eps);
                            cp_ahead_share_truth(row) = full.cp_ahead_share_truth;
                            mean_static_margin_cal_truth(row) = full.mean_static_margin_cal_truth;
                            stability_mismatch(row) = full.unstable_truth ~= full.unstable_pred_nominal;
                            aero_more_important_than_delay(row) = effect_margin_deg(row) < 0;
                            max_cp_forward_m(row) = cp_forward_m;
                            max_cp_forward_cal(row) = cp_forward_m / cfg.body_diameter_m;
                            row = row + 1;
                        end
                    end
                end
            end
        end
    end

    extreme = table(rate_center_deg_s, tau_actuator_ms, tau_sensor_ms, processing_delay_ms, aero_scale, thrust_scale, ...
        effect_margin_deg, effect_ratio, cp_ahead_share_truth, mean_static_margin_cal_truth, stability_mismatch, ...
        aero_more_important_than_delay, max_cp_forward_m, max_cp_forward_cal);
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

function plot_regime_divergence(regime_stats, transition, fig_dir, cfg)
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

    save_plot_if_enabled(gcf, fig_dir, '01_regime_divergence.png', cfg);
end

function plot_residual_map(results, fig_dir, cfg)
    figure('Name', 'Residual Map');
    scatter(results.rate_center_deg_s, results.rms_theta_residual_deg, 45, results.phase_lag_divergence_ms, 'filled');
    grid on;
    xlabel('Rate regime center (deg/s)');
    ylabel('Theta residual (deg RMS)');
    title('Simulation Fidelity Residual Map (Color = Phase Lag Divergence ms)');
    cb = colorbar;
    cb.Label.String = 'Phase lag divergence (ms)';

    save_plot_if_enabled(gcf, fig_dir, '02_residual_map.png', cfg);
end

function plot_stability_mismatch(regime_stats, transition, fig_dir, cfg)
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

    save_plot_if_enabled(gcf, fig_dir, '03_stability_mismatch.png', cfg);
end

function plot_cp_instability_requirement(results, cfg, fig_dir)
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

    save_plot_if_enabled(gcf, fig_dir, '04_cp_ahead_vs_mismatch.png', cfg);
end

function plot_regime_limitation_stack(regime_atlas, fig_dir, cfg)
    rates = regime_atlas.rate_center_deg_s;
    Y = [regime_atlas.delay_limited_share, regime_atlas.aero_limited_share, ...
         regime_atlas.mixed_interaction_share, regime_atlas.model_mismatch_limited_share];

    figure('Name', 'Control Limitation Regime Composition');
    b = bar(rates, Y, 'stacked', 'LineStyle', 'none'); %#ok<NASGU>
    grid on;
    xlabel('Rate regime center (deg/s)');
    ylabel('Share of cases');
    title('Control Limitation Composition by Regime');
    legend({'Delay-limited','Aero-limited','Mixed interaction','Model mismatch-limited'}, 'Location', 'eastoutside');
    ylim([0 1]);

    save_plot_if_enabled(gcf, fig_dir, '05_regime_limitation_stack.png', cfg);
end

function plot_parameter_heatmaps(results, fig_dir, cfg)
    rows = results(results.comparison_valid, :);
    tau_act_vals = unique(rows.tau_actuator_ms);
    tau_sens_vals = unique(rows.tau_sensor_ms);

    mismatch_map = nan(numel(tau_act_vals), numel(tau_sens_vals));
    cp_ahead_map = nan(numel(tau_act_vals), numel(tau_sens_vals));

    for i = 1:numel(tau_act_vals)
        for j = 1:numel(tau_sens_vals)
            idx = rows.tau_actuator_ms == tau_act_vals(i) & rows.tau_sensor_ms == tau_sens_vals(j);
            block = rows(idx, :);
            if ~isempty(block)
                mismatch_map(i, j) = mean(block.stability_mismatch);
                cp_ahead_map(i, j) = mean(block.cp_ahead_share_truth);
            end
        end
    end

    figure('Name', 'Delay Parameter Heatmaps');
    tiledlayout(1, 2);

    nexttile;
    imagesc(tau_sens_vals, tau_act_vals, mismatch_map);
    set(gca, 'YDir', 'normal');
    xlabel('Sensor delay (ms)');
    ylabel('Actuator delay (ms)');
    title('Stability mismatch share');
    colorbar;

    nexttile;
    imagesc(tau_sens_vals, tau_act_vals, cp_ahead_map);
    set(gca, 'YDir', 'normal');
    xlabel('Sensor delay (ms)');
    ylabel('Actuator delay (ms)');
    title('CP-ahead exposure share');
    colorbar;

    save_plot_if_enabled(gcf, fig_dir, '06_delay_heatmaps.png', cfg);
end

function plot_cp_margin_distributions(results, fig_dir, cfg)
    rows = results(results.comparison_valid, :);

    figure('Name', 'CP and Static Margin Distributions');
    tiledlayout(1, 2);

    nexttile;
    histogram(rows.cp_ahead_share_truth, 14);
    hold on;
    xline(cfg.cp_ahead_share_alert, '--r', 'LineWidth', 1.5);
    grid on;
    xlabel('CP-ahead exposure share');
    ylabel('Count');
    title('Distribution of CP-ahead exposure');

    nexttile;
    histogram(rows.mean_static_margin_cal_truth, 14);
    hold on;
    xline(0, '--k', 'LineWidth', 1.5);
    grid on;
    xlabel('Mean static margin (calibers)');
    ylabel('Count');
    title('Distribution of static margin');

    save_plot_if_enabled(gcf, fig_dir, '07_cp_static_margin_distributions.png', cfg);
end

function plot_controller_comparison_summary(summary, fig_dir, cfg)
    regimes = unique(summary.regime_name, 'stable');
    pid_unstable = zeros(numel(regimes), 1);
    adrc_unstable = zeros(numel(regimes), 1);
    pid_rms = zeros(numel(regimes), 1);
    adrc_rms = zeros(numel(regimes), 1);

    for i = 1:numel(regimes)
        rows = summary(summary.regime_name == regimes(i), :);
        pid_unstable(i) = rows.unstable_share(rows.controller == "PID");
        adrc_unstable(i) = rows.unstable_share(rows.controller == "ADRC");
        pid_rms(i) = rows.mean_rms_angle_error_deg(rows.controller == "PID");
        adrc_rms(i) = rows.mean_rms_angle_error_deg(rows.controller == "ADRC");
    end

    figure('Name', 'PID vs ADRC Summary');
    tiledlayout(1, 2);

    nexttile;
    bar(categorical(cellstr(regimes)), [pid_unstable, adrc_unstable], 'grouped');
    ylabel('Unstable share');
    title('Instability by regime');
    legend({'PID', 'ADRC'}, 'Location', 'southwest');
    ylim([0, 1]);
    grid on;

    nexttile;
    bar(categorical(cellstr(regimes)), [pid_rms, adrc_rms], 'grouped');
    ylabel('Mean RMS angle error (deg)');
    title('Tracking error by regime');
    legend({'PID', 'ADRC'}, 'Location', 'northwest');
    grid on;

    save_plot_if_enabled(gcf, fig_dir, '08_pid_vs_adrc_summary.png', cfg);
end

function plot_controller_uncertainty_frontier(summary, fig_dir, cfg)
    levels = ["NOMINAL"; "MODERATE_MISMATCH"; "HIGH_MISMATCH"];
    pid_unstable = zeros(numel(levels), 1);
    adrc_unstable = zeros(numel(levels), 1);
    pid_rms = zeros(numel(levels), 1);
    adrc_rms = zeros(numel(levels), 1);

    for i = 1:numel(levels)
        rows = summary(summary.uncertainty_level == levels(i), :);
        pid_unstable(i) = rows.unstable_share(rows.controller == "PID");
        adrc_unstable(i) = rows.unstable_share(rows.controller == "ADRC");
        pid_rms(i) = rows.mean_rms_angle_error_deg(rows.controller == "PID");
        adrc_rms(i) = rows.mean_rms_angle_error_deg(rows.controller == "ADRC");
    end

    figure('Name', 'Controller Robustness to Aerodynamic Uncertainty');
    tiledlayout(1, 2);

    nexttile;
    bar(categorical(cellstr(levels)), [pid_unstable, adrc_unstable], 'grouped');
    ylabel('Unstable share');
    title('Instability vs aerodynamic mismatch');
    legend({'PID', 'ADRC'}, 'Location', 'northwest');
    ylim([0, 1]);
    grid on;

    nexttile;
    bar(categorical(cellstr(levels)), [pid_rms, adrc_rms], 'grouped');
    ylabel('Mean RMS angle error (deg)');
    title('Tracking error vs aerodynamic mismatch');
    legend({'PID', 'ADRC'}, 'Location', 'northwest');
    grid on;

    save_plot_if_enabled(gcf, fig_dir, '11_controller_uncertainty_frontier.png', cfg);
end

function plot_controller_showcases(sim_table, cfg, fig_dir)
    base_idx = find(sim_table.tau_actuator_ms == cfg.base_tau_actuator_ms & ...
                    sim_table.tau_sensor_ms == cfg.base_tau_sensor_ms & ...
                    sim_table.processing_delay_ms == cfg.base_processing_delay_ms & ...
                    sim_table.aero_scale == cfg.base_aero_scale & ...
                    sim_table.thrust_scale == cfg.base_thrust_scale, 1, 'first');
    stress_score = sim_table.tau_actuator_ms + sim_table.tau_sensor_ms + 2 * sim_table.processing_delay_ms + ...
                   20 * abs(sim_table.aero_scale - cfg.base_aero_scale) + 20 * abs(sim_table.thrust_scale - cfg.base_thrust_scale);
    [~, stress_idx] = max(stress_score);

    showcase_cases = [base_idx, stress_idx];
    showcase_rates = [mean(cfg.rate_bins(2, :)), mean(cfg.rate_bins(end, :))];
    showcase_names = {'09_pid_vs_adrc_nominal_midrate.png', '10_pid_vs_adrc_stress_highrate.png'};
    showcase_titles = {'Nominal Case - Mid Rate', 'Stress Case - High Rate'};

    for i = 1:numel(showcase_cases)
        case_params = sim_table(showcase_cases(i), :);
        p = make_params(case_params.tau_actuator_ms, case_params.tau_sensor_ms, case_params.processing_delay_ms, case_params.aero_scale, case_params.thrust_scale, true, true);
        x0 = make_initial_state(showcase_rates(i));
        pid_traj = simulate_tvc(x0, p, cfg, "PID");
        adrc_traj = simulate_tvc(x0, p, cfg, "ADRC");

        figure('Name', showcase_titles{i});
        tiledlayout(3, 1);

        nexttile;
        plot(pid_traj.t, rad2deg(pid_traj.err_rad), 'LineWidth', 1.8);
        hold on;
        plot(adrc_traj.t, rad2deg(adrc_traj.err_rad), 'LineWidth', 1.8);
        yline(0, '--k');
        grid on;
        ylabel('Angle error (deg)');
        title(showcase_titles{i});
        legend({'PID', 'ADRC'}, 'Location', 'northeast');

        nexttile;
        plot(pid_traj.t, rad2deg(pid_traj.q), 'LineWidth', 1.8);
        hold on;
        plot(adrc_traj.t, rad2deg(adrc_traj.q), 'LineWidth', 1.8);
        yline(0, '--k');
        grid on;
        ylabel('Angular velocity (deg/s)');

        nexttile;
        plot(pid_traj.t, rad2deg(pid_traj.delta_cmd_rad), '--', 'LineWidth', 1.5);
        hold on;
        plot(pid_traj.t, rad2deg(pid_traj.delta), 'LineWidth', 1.8);
        plot(adrc_traj.t, rad2deg(adrc_traj.delta_cmd_rad), '--', 'LineWidth', 1.5);
        plot(adrc_traj.t, rad2deg(adrc_traj.delta), 'LineWidth', 1.8);
        yline(0, '--k');
        grid on;
        xlabel('Time (s)');
        ylabel('Correction (deg)');
        legend({'PID command', 'PID actual', 'ADRC command', 'ADRC actual'}, 'Location', 'northeast');

        save_plot_if_enabled(gcf, fig_dir, showcase_names{i}, cfg);
    end
end

function [model, training_summary] = train_ai_controller_model(sim_table, cfg)
    n_cases = height(sim_table);
    max_cases = min(cfg.ai_train_max_cases, n_cases);
    idx_cases = round(linspace(1, n_cases, max_cases));
    rates = mean(cfg.rate_bins, 2);
    n_rows = numel(idx_cases) * numel(rates);
    samples = struct('tau_actuator_ms', cell(n_rows,1), 'tau_sensor_ms', cell(n_rows,1), ...
                     'processing_delay_ms', cell(n_rows,1), 'aero_scale', cell(n_rows,1), 'thrust_scale', cell(n_rows,1), ...
                     'rate_center_deg_s', cell(n_rows,1));
    row = 1;
    for i = 1:numel(idx_cases)
        cp = sim_table(idx_cases(i), :);
        for r = 1:numel(rates)
            samples(row).tau_actuator_ms = cp.tau_actuator_ms;
            samples(row).tau_sensor_ms = cp.tau_sensor_ms;
            samples(row).processing_delay_ms = cp.processing_delay_ms;
            samples(row).aero_scale = cp.aero_scale;
            samples(row).thrust_scale = cp.thrust_scale;
            samples(row).rate_center_deg_s = rates(r);
            row = row + 1;
        end
    end

    n = numel(samples);
    n_train = max(1, floor(0.8 * n));
    train_idx = 1:n_train;
    val_idx = (n_train+1):n;
    if isempty(val_idx)
        val_idx = train_idx;
    end

    action_bins_deg = cfg.ai_action_bins_deg(:);
    n_actions = numel(action_bins_deg);
    n_theta = numel(cfg.ai_theta_edges_deg) - 1;
    n_q = numel(cfg.ai_q_edges_deg_s) - 1;
    n_delta = numel(cfg.ai_delta_edges_deg) - 1;
    n_states = n_theta * n_q * n_delta;
    action_prior = -0.002 * (action_bins_deg(:)'.^2);
    Q = repmat(action_prior, n_states, 1);

    % Warm-start with ADRC demonstrations (state->motor command), then RL fine-tunes.
    for ii = 1:numel(train_idx)
        cp = samples(train_idx(ii));
        p_demo = make_params(cp.tau_actuator_ms, cp.tau_sensor_ms, cp.processing_delay_ms, cp.aero_scale, cp.thrust_scale, true, true);
        x0_demo = make_initial_state(cp.rate_center_deg_s);
        demo = simulate_tvc(x0_demo, p_demo, cfg, "ADRC");
        for k = 1:numel(demo.t)
            s_idx = ai_state_index(demo.theta(k), demo.q(k), demo.delta(k), cfg);
            cmd_deg = rad2deg(demo.delta_cmd_rad(k));
            [~, a_idx] = min(abs(action_bins_deg - cmd_deg));
            Q(s_idx, a_idx) = Q(s_idx, a_idx) + 0.5;
        end
    end

    try
        rl_available = cfg.ai_use_rl_toolbox_preferred && license('test', 'Reinforcement_Learning_Toolbox') && ...
            (exist('rlDQNAgent', 'file') == 2 || exist('rlDQNAgent', 'class') == 8);
    catch
        rl_available = false;
    end

    % Q-learning over discretized flight states with direct motor-angle actions.
    alpha = cfg.ai_rl_alpha;
    gamma = cfg.ai_rl_gamma;
    eps_start = cfg.ai_rl_epsilon_start;
    eps_end = cfg.ai_rl_epsilon_end;
    episodes = max(1, cfg.ai_rl_requested_episodes);

    for ep = 1:episodes
        eps_now = eps_end + (eps_start - eps_end) * max(0, (episodes - ep) / max(episodes - 1, 1));
        order = train_idx(randperm(numel(train_idx)));
        for ii = 1:numel(order)
            cp = samples(order(ii));
            p = make_params(cp.tau_actuator_ms, cp.tau_sensor_ms, cp.processing_delay_ms, cp.aero_scale, cp.thrust_scale, true, true);

            x0 = make_initial_state(cp.rate_center_deg_s);
            th = x0.theta_rad;
            qk = x0.q_rad_s;
            dk = x0.delta_rad;
            th_hat = x0.theta_hat_rad;
            q_hat = x0.q_hat_rad_s;
            vx = x0.vx_m_s;
            vz = x0.vz_m_s;
            processing_timer = 0;
            pending_cmd = x0.delta_rad;

            cp_from_nose_case = cfg.cp_base_from_nose_m + cfg.cp_case_shift_gain_m * (p.aero_scale - 1.0);
            cp_from_nose_case = min(cfg.body_length_m, max(0, cp_from_nose_case));
            cp_minus_cg_case = cp_from_nose_case - cfg.cg_from_nose_m;

            delta_lim = deg2rad(cfg.actuator_limit_deg);
            slew_rate_limit = deg2rad(cfg.actuator_slew_limit_deg_s);
            n_steps = floor(cfg.t_final / cfg.dt) + 1;

            for k = 1:n_steps
                s_idx = ai_state_index(th_hat, q_hat, dk, cfg);
                if rand < eps_now
                    a = randi(n_actions);
                else
                    [~, a] = max(Q(s_idx, :));
                end

                delta_cmd_raw = deg2rad(action_bins_deg(a));
                delta_cmd_raw = min(max(delta_cmd_raw, -delta_lim), delta_lim);

                processing_timer = processing_timer - cfg.dt;
                if processing_timer <= 0
                    pending_cmd = delta_cmd_raw;
                    processing_timer = max(p.processing_delay_s, cfg.dt);
                end

                tau_act = max(p.tau_act_s, 1e-4);
                if p.use_delay_interaction
                    tau_act = tau_act * (1 + cfg.delay_rate_coupling * abs(q_hat));
                end
                delta_rate_cmd = (pending_cmd - dk) / tau_act;
                delta_rate_cmd = min(max(delta_rate_cmd, -slew_rate_limit), slew_rate_limit);
                dk = dk + delta_rate_cmd * cfg.dt;
                dk = min(max(dk, -delta_lim), delta_lim);

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

                m_tvc = thrust * cfg.lever_arm_m * sin(dk);
                speed = sqrt(vx * vx + vz * vz);
                if speed > 0.5
                    gamma_path = atan2(vz, vx);
                else
                    gamma_path = th;
                end
                alpha_aero = wrap_to_pi_local(th - gamma_path);
                q_bar = 0.5 * cfg.rho_kg_m3 * speed * speed;
                cn_alpha = cfg.cn_alpha_nom * (0.8 + 0.2 * p.aero_scale);
                f_aero_normal = q_bar * cfg.ref_area_m2 * cn_alpha * alpha_aero;
                m_aero_static = -f_aero_normal * cp_minus_cg_case;
                m_aero_damp = -q_bar * cfg.ref_area_m2 * cfg.body_length_m * cfg.body_length_m * cfg.cm_q_nom * p.aero_scale * qk;
                qdot = (m_tvc + m_aero_static + m_aero_damp) / cfg.Iyy;
                qk = qk + qdot * cfg.dt;
                th = th + qk * cfg.dt;

                tau_sensor = max(p.tau_sensor_s, 1e-4);
                th_hat = th_hat + ((th - th_hat) / tau_sensor) * cfg.dt;
                q_hat = q_hat + ((qk - q_hat) / tau_sensor) * cfg.dt;

                err_deg = rad2deg(th);
                rate_deg_s = rad2deg(qk);
                slew_gap_deg = rad2deg(delta_cmd_raw - dk);
                unstable = abs(err_deg) > cfg.max_abs_angle_deg || abs(rate_deg_s) > cfg.max_abs_rate_deg_s;
                reward = -(err_deg^2 + 0.02 * rate_deg_s^2 + 0.15 * slew_gap_deg^2 + 0.01 * rad2deg(delta_cmd_raw)^2);
                if unstable
                    reward = reward - cfg.ai_unstable_penalty;
                end

                s_next = ai_state_index(th_hat, q_hat, dk, cfg);
                td_target = reward + gamma * max(Q(s_next, :));
                Q(s_idx, a) = Q(s_idx, a) + alpha * (td_target - Q(s_idx, a));

                if unstable
                    break;
                end
            end
        end
    end

    if rl_available
        training_method = "Q_LEARNING_RL_TOOLBOX_AVAILABLE";
    else
        training_method = "Q_LEARNING_CUSTOM_RL";
    end

    model = struct;
    model.type = "Q_LEARNING_DIRECT_DELTA";
    model.Q = Q;
    model.action_bins_deg = action_bins_deg;
    model.theta_edges_deg = cfg.ai_theta_edges_deg;
    model.q_edges_deg_s = cfg.ai_q_edges_deg_s;
    model.delta_edges_deg = cfg.ai_delta_edges_deg;
    model.training_method = training_method;
    model.rl_toolbox_available = rl_available;

    val_rmse_angle = zeros(numel(val_idx), 1);
    for i = 1:numel(val_idx)
        cp = samples(val_idx(i));
        rate_center = cp.rate_center_deg_s;
        p = make_params(cp.tau_actuator_ms, cp.tau_sensor_ms, cp.processing_delay_ms, cp.aero_scale, cp.thrust_scale, true, true);
        p.ai_model = model;
        x0 = make_initial_state(rate_center);
        traj = simulate_tvc(x0, p, cfg, "AI");
        val_rmse_angle(i) = sqrt(mean(rad2deg(traj.err_rad).^2));
    end
    model.validation_rmse_angle_deg = mean(val_rmse_angle);

    training_summary = table;
    training_summary.train_rows = numel(train_idx);
    training_summary.validation_rows = numel(val_idx);
    training_summary.validation_rmse_angle_deg = model.validation_rmse_angle_deg;
    training_summary.training_method = string(model.training_method);
    training_summary.rl_toolbox_available = model.rl_toolbox_available;
    training_summary.ai_rl_alpha = alpha;
    training_summary.ai_rl_gamma = gamma;
    training_summary.ai_rl_episodes = episodes;
    training_summary.ai_actions = n_actions;
    training_summary.ai_state_bins = n_states;
end

function delta_cmd_raw = ai_policy_action(model, th_hat, q_hat, dk, cfg)
    if ~isstruct(model) || ~isfield(model, 'Q') || isempty(model.Q)
        delta_cmd_raw = 0;
        return;
    end
    s_idx = ai_state_index(th_hat, q_hat, dk, cfg);
    [~, a] = max(model.Q(s_idx, :));
    delta_cmd_raw = deg2rad(model.action_bins_deg(a));
end

function s_idx = ai_state_index(th_hat, q_hat, dk, cfg)
    th_deg = rad2deg(th_hat);
    q_deg_s = rad2deg(q_hat);
    d_deg = rad2deg(dk);

    i_th = bin_index_clamped(th_deg, cfg.ai_theta_edges_deg);
    i_q = bin_index_clamped(q_deg_s, cfg.ai_q_edges_deg_s);
    i_d = bin_index_clamped(d_deg, cfg.ai_delta_edges_deg);

    n_q = numel(cfg.ai_q_edges_deg_s) - 1;
    n_d = numel(cfg.ai_delta_edges_deg) - 1;
    s_idx = (i_th - 1) * n_q * n_d + (i_q - 1) * n_d + i_d;
end

function idx = bin_index_clamped(value, edges)
    idx = find(value >= edges(1:end-1) & value < edges(2:end), 1, 'first');
    if isempty(idx)
        if value < edges(1)
            idx = 1;
        else
            idx = numel(edges) - 1;
        end
    end
end

function plot_pid_adrc_ai_profile(sim_table, cfg, fig_dir)
    needed = ["PID", "ADRC", "AI"];
    have_all = all(ismember(needed, string(cfg.controller_names)));
    if ~have_all
        return;
    end

    % Mid-stress case for intuitive side-by-side comparison.
    score = sim_table.tau_actuator_ms + sim_table.tau_sensor_ms + 2 * sim_table.processing_delay_ms + 10 * abs(sim_table.aero_scale - cfg.base_aero_scale);
    [~, idx] = max(score);
    cp = sim_table(idx, :);
    rate_center = mean(cfg.rate_bins(2, :));
    x0 = make_initial_state(rate_center);

    p = make_params(cp.tau_actuator_ms, cp.tau_sensor_ms, cp.processing_delay_ms, cp.aero_scale, cp.thrust_scale, true, true);
    p_ai = p;
    if isfield(cfg, 'ai_model')
        p_ai.ai_model = cfg.ai_model;
    else
        p_ai.ai_model = struct;
    end

    pid_traj = simulate_tvc(x0, p, cfg, "PID");
    adrc_traj = simulate_tvc(x0, p, cfg, "ADRC");
    ai_traj = simulate_tvc(x0, p_ai, cfg, "AI");

    figure('Name', 'PID vs ADRC vs AI Profile');
    tiledlayout(3, 1);

    nexttile;
    plot(pid_traj.t, rad2deg(pid_traj.err_rad), 'LineWidth', 1.7); hold on;
    plot(adrc_traj.t, rad2deg(adrc_traj.err_rad), 'LineWidth', 1.7);
    plot(ai_traj.t, rad2deg(ai_traj.err_rad), 'LineWidth', 1.7);
    yline(0, '--k');
    grid on;
    ylabel('Angle error (deg)');
    title('Side-by-side: PID vs ADRC vs AI');
    legend({'PID', 'ADRC', 'AI'}, 'Location', 'northeast');

    nexttile;
    plot(pid_traj.t, rad2deg(pid_traj.q), 'LineWidth', 1.7); hold on;
    plot(adrc_traj.t, rad2deg(adrc_traj.q), 'LineWidth', 1.7);
    plot(ai_traj.t, rad2deg(ai_traj.q), 'LineWidth', 1.7);
    yline(0, '--k');
    grid on;
    ylabel('Angular velocity (deg/s)');

    nexttile;
    plot(pid_traj.t, rad2deg(pid_traj.delta), 'LineWidth', 1.7); hold on;
    plot(adrc_traj.t, rad2deg(adrc_traj.delta), 'LineWidth', 1.7);
    plot(ai_traj.t, rad2deg(ai_traj.delta), 'LineWidth', 1.7);
    yline(0, '--k');
    grid on;
    xlabel('Time (s)');
    ylabel('Correction (deg)');
    legend({'PID', 'ADRC', 'AI'}, 'Location', 'northeast');

    save_plot_if_enabled(gcf, fig_dir, '12_pid_vs_adrc_vs_ai_profile.png', cfg);
end

function plot_ai_direction_summary(summary, fig_dir, cfg)
    regimes = unique(summary.regime_name, 'stable');
    pid_rms = zeros(numel(regimes), 1);
    adrc_rms = zeros(numel(regimes), 1);
    ai_rms = zeros(numel(regimes), 1);

    for i = 1:numel(regimes)
        rows = summary(summary.regime_name == regimes(i), :);
        pid_rms(i) = rows.mean_rms_angle_error_deg(rows.controller == "PID");
        adrc_rms(i) = rows.mean_rms_angle_error_deg(rows.controller == "ADRC");
        ai_rms(i) = rows.mean_rms_angle_error_deg(rows.controller == "AI");
    end

    figure('Name', 'AI Direction Summary');
    bar(categorical(cellstr(regimes)), [pid_rms, adrc_rms, ai_rms], 'grouped');
    grid on;
    ylabel('Mean RMS angle error (deg)');
    title('AI-focused comparison: lower is better');
    legend({'PID', 'ADRC', 'AI'}, 'Location', 'northwest');
    save_plot_if_enabled(gcf, fig_dir, '13_ai_direction_error_summary.png', cfg);
end

function save_plot_if_enabled(fig_handle, fig_dir, file_name, cfg)
    if ~cfg.save_plot_files
        return;
    end
    if ~exist(fig_dir, 'dir')
        mkdir(fig_dir);
    end
    if isprop(fig_handle, 'ToolBar')
        fig_handle.ToolBar = 'none';
    end
    if isprop(fig_handle, 'MenuBar')
        fig_handle.MenuBar = 'none';
    end
    ax = findall(fig_handle, 'Type', 'axes');
    for k = 1:numel(ax)
        if isprop(ax(k), 'Toolbar')
            ax(k).Toolbar = [];
        end
    end
    drawnow;
    out_path = fullfile(fig_dir, file_name);
    try
        exportgraphics(fig_handle, out_path, 'Resolution', 180);
    catch
        saveas(fig_handle, out_path);
    end
    close(fig_handle);
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

function zc = count_zero_crossings(signal)
    s = signal(:);
    if numel(s) < 2
        zc = 0;
        return;
    end
    zc = sum(s(1:end-1) .* s(2:end) < 0);
end

function obj = controller_quality_objective(traj, cfg)
    err_deg = rad2deg(traj.err_rad);
    rms_theta = sqrt(mean(err_deg.^2));
    rms_rate = sqrt(mean(rad2deg(traj.q).^2));
    final_abs = abs(err_deg(end));
    overshoot = max(0, max(err_deg));
    osc = count_zero_crossings(err_deg(round(0.25 * numel(err_deg)):end));
    slew_track = mean(abs(rad2deg(traj.delta_cmd_rad - traj.delta)));
    obj = rms_theta + ...
          cfg.tune_rate_weight * rms_rate + ...
          cfg.tune_final_weight * final_abs + ...
          cfg.tune_overshoot_weight * overshoot + ...
          cfg.tune_oscillation_weight * osc + ...
          cfg.tune_slew_tracking_weight * slew_track + ...
          cfg.ai_unstable_penalty * double(traj.unstable);
end

function [cfg_out, summary] = auto_tune_pid_adrc(sim_table, cfg)
    cfg_out = cfg;
    n_cases = height(sim_table);
    use_cases = min(cfg.tune_max_cases, n_cases);
    case_idx = round(linspace(1, n_cases, use_cases));
    rate_idx = unique([1, ceil(size(cfg.rate_bins, 1)/2), size(cfg.rate_bins, 1)]);

    best_pid_obj = inf;
    best_pid = [cfg.kp, cfg.ki, cfg.kd];
    for kp = cfg.pid_kp_candidates
        for ki = cfg.pid_ki_candidates
            for kd = cfg.pid_kd_candidates
                cfg_trial = cfg;
                cfg_trial.kp = kp;
                cfg_trial.ki = ki;
                cfg_trial.kd = kd;
                total = 0;
                for c = case_idx
                    cp = sim_table(c, :);
                    p = make_params(cp.tau_actuator_ms, cp.tau_sensor_ms, cp.processing_delay_ms, cp.aero_scale, cp.thrust_scale, true, true);
                    for r = rate_idx
                        x0 = make_initial_state(mean(cfg.rate_bins(r, :)));
                        traj = simulate_tvc(x0, p, cfg_trial, "PID");
                        total = total + controller_quality_objective(traj, cfg_trial);
                    end
                end
                if total < best_pid_obj
                    best_pid_obj = total;
                    best_pid = [kp, ki, kd];
                end
            end
        end
    end

    best_adrc_obj = inf;
    best_adrc = [cfg.adrc_wc, cfg.adrc_wo, cfg.adrc_ki_scale];
    for wc = cfg.adrc_wc_candidates
        for wo = cfg.adrc_wo_candidates
            for ki_scale = cfg.adrc_ki_scale_candidates
                cfg_trial = cfg;
                cfg_trial.adrc_wc = wc;
                cfg_trial.adrc_wo = wo;
                cfg_trial.adrc_ki_scale = ki_scale;
                total = 0;
                for c = case_idx
                    cp = sim_table(c, :);
                    p = make_params(cp.tau_actuator_ms, cp.tau_sensor_ms, cp.processing_delay_ms, cp.aero_scale, cp.thrust_scale, true, true);
                    for r = rate_idx
                        x0 = make_initial_state(mean(cfg.rate_bins(r, :)));
                        traj = simulate_tvc(x0, p, cfg_trial, "ADRC");
                        total = total + controller_quality_objective(traj, cfg_trial);
                    end
                end
                if total < best_adrc_obj
                    best_adrc_obj = total;
                    best_adrc = [wc, wo, ki_scale];
                end
            end
        end
    end

    cfg_out.kp = best_pid(1);
    cfg_out.ki = best_pid(2);
    cfg_out.kd = best_pid(3);
    cfg_out.adrc_wc = best_adrc(1);
    cfg_out.adrc_wo = best_adrc(2);
    cfg_out.adrc_ki_scale = best_adrc(3);

    summary = table;
    summary.pid_kp = cfg_out.kp;
    summary.pid_ki = cfg_out.ki;
    summary.pid_kd = cfg_out.kd;
    summary.pid_objective = best_pid_obj;
    summary.adrc_wc = cfg_out.adrc_wc;
    summary.adrc_wo = cfg_out.adrc_wo;
    summary.adrc_ki_scale = cfg_out.adrc_ki_scale;
    summary.adrc_objective = best_adrc_obj;
    summary.tuning_cases_used = numel(case_idx);
    summary.tuning_rate_bins_used = numel(rate_idx);
end

function unified = build_unified_research_summary(study_summary, regime_atlas, controller_uncertainty, out_dir)
    frontier_path = fullfile(out_dir, 'adrc_bandwidth_frontier_summary.csv');

    % Main sweep conclusion: which limitation is most common in the tested space?
    delay_share = mean(regime_atlas.delay_limited_share, 'omitnan');
    aero_share = mean(regime_atlas.aero_limited_share, 'omitnan');
    mixed_share = mean(regime_atlas.mixed_interaction_share, 'omitnan');

    [~, idx_main] = max([delay_share, aero_share, mixed_share]);
    main_label = ["delay is usually the bottleneck"; "aerodynamics are usually the bottleneck"; "delay and aerodynamics both matter"];

    % Controller uncertainty summary from current run.
    high_rows = controller_uncertainty(controller_uncertainty.uncertainty_level == "HIGH_MISMATCH", :);
    pid_unstable_high = high_rows.unstable_share(high_rows.controller == "PID");
    adrc_unstable_high = high_rows.unstable_share(high_rows.controller == "ADRC");
    if isempty(pid_unstable_high)
        pid_unstable_high = nan;
    end
    if isempty(adrc_unstable_high)
        adrc_unstable_high = nan;
    end

    % ADRC benchmark frontier summary: hardware-feasible observer region by delay.
    frontier_available = false;
    delay_feasible_limit_ms = nan;
    observer_takeaway = "ADRC frontier file missing; run ADRC_DLESO_Benchmark for hardware-limit map.";
    if exist(frontier_path, 'file')
        frontier = readtable(frontier_path, 'TextType', 'string');
        if ~isempty(frontier)
            frontier_available = true;
            frontier_flags = string(frontier.recommendation_flag);
            frontier.is_feasible = frontier_flags == "FEASIBLE_REGION_FOUND";

            if isnumeric(frontier.actuator_delay_ms)
                delay_vals = frontier.actuator_delay_ms;
            else
                delay_vals = str2double(string(frontier.actuator_delay_ms));
            end

            valid = ~isnan(delay_vals);
            [g, delays] = findgroups(delay_vals(valid));
            feasible_share = splitapply(@mean, double(frontier.is_feasible(valid)), g);

            first_bad = find(feasible_share < 0.5, 1, 'first');
            if ~isempty(first_bad)
                delay_feasible_limit_ms = delays(first_bad);
                observer_takeaway = "above this delay, a robust ADRC tuning window mostly disappears";
            else
                observer_takeaway = "robust ADRC tuning windows remain available across tested delays";
            end
        end
    end

    if isnan(pid_unstable_high) || isnan(adrc_unstable_high)
        controller_takeaway = "high-mismatch PID-vs-ADRC comparison unavailable in this run";
        simple_takeaway_2 = "Controller comparison at high mismatch is missing in this run.";
    elseif adrc_unstable_high < pid_unstable_high
        controller_takeaway = "ADRC stays stable more often than PID when aero mismatch is high";
        simple_takeaway_2 = "In high mismatch cases here, ADRC is safer than PID.";
    elseif adrc_unstable_high > pid_unstable_high
        controller_takeaway = "PID stays stable more often than ADRC when aero mismatch is high";
        simple_takeaway_2 = "In high mismatch cases here, PID is safer than ADRC.";
    else
        controller_takeaway = "PID and ADRC show similar instability in high mismatch cases";
        simple_takeaway_2 = "In high mismatch cases here, PID and ADRC are similarly stable.";
    end

    unified = table;
    unified.unified_research_question = "How much model fidelity is needed, and when can adaptive observers compensate under hardware limits?";
    unified.main_pipeline_takeaway = main_label(idx_main);
    unified.controller_takeaway = controller_takeaway;
    unified.observer_takeaway = observer_takeaway;
    unified.main_boundary_rate_deg_s = study_summary.boundary_rate_deg_s(1);
    unified.main_mismatch_share = study_summary.mean_stability_mismatch_share(1);
    unified.pid_unstable_share_high_mismatch = pid_unstable_high;
    unified.adrc_unstable_share_high_mismatch = adrc_unstable_high;
    unified.adrc_frontier_available = frontier_available;
    unified.adrc_delay_feasible_limit_ms = delay_feasible_limit_ms;
    unified.simple_takeaway_1 = "First, identify if delay or aerodynamics is the bigger issue in your regime.";
    unified.simple_takeaway_2 = simple_takeaway_2;
    unified.simple_takeaway_3 = "But ADRC needs enough actuator speed; too much delay can remove good tuning choices.";
    unified.simple_takeaway_4 = "So: choose model fidelity first, then choose controller bandwidth that hardware can actually support.";
    unified.simple_takeaway_5 = "Use this as a design rule, not a universal law; validate with flight data next.";
end

function unified = build_unified_research_fallback(study_summary)
    unified = table;
    unified.unified_research_question = "How much model fidelity is needed, and when can adaptive observers compensate under hardware limits?";
    unified.main_pipeline_takeaway = "main model-fidelity summary available";
    unified.controller_takeaway = "controller uncertainty summary unavailable in fallback";
    unified.observer_takeaway = "observer frontier parsing failed; check benchmark CSV schema";
    unified.main_boundary_rate_deg_s = study_summary.boundary_rate_deg_s(1);
    unified.main_mismatch_share = study_summary.mean_stability_mismatch_share(1);
    unified.pid_unstable_share_high_mismatch = nan;
    unified.adrc_unstable_share_high_mismatch = nan;
    unified.adrc_frontier_available = false;
    unified.adrc_delay_feasible_limit_ms = nan;
    unified.simple_takeaway_1 = "Main sweep finished, but unified parser fell back.";
    unified.simple_takeaway_2 = "Re-check benchmark file format before trusting merged claims.";
    unified.simple_takeaway_3 = "Delay is still important in the tested fast-mode space.";
    unified.simple_takeaway_4 = "Use separate main/benchmark outputs until parser issue is fixed.";
    unified.simple_takeaway_5 = "Then rerun to regenerate unified summary.";
end

function cleanup_disabled_artifacts(out_dir, fig_dir, cfg)
    if ~cfg.include_flight_manifest
        delete_if_exists(fullfile(out_dir, 'flight_manifest_template.csv'));
    end
    if ~cfg.include_delay_breakdown
        delete_if_exists(fullfile(out_dir, 'delay_component_priority_summary.csv'));
    end
    if ~cfg.include_extreme_exploration
        delete_if_exists(fullfile(out_dir, 'extreme_regime_exploration.csv'));
    end
    if ~cfg.include_parameter_heatmaps
        delete_if_exists(fullfile(fig_dir, '06_delay_heatmaps.png'));
    end
    if ~cfg.include_legacy_controller_summary_plot
        delete_if_exists(fullfile(fig_dir, '08_pid_vs_adrc_summary.png'));
    end
    if ~cfg.include_controller_showcases
        delete_if_exists(fullfile(fig_dir, '09_pid_vs_adrc_nominal_midrate.png'));
        delete_if_exists(fullfile(fig_dir, '10_pid_vs_adrc_stress_highrate.png'));
    end
    if isfield(cfg, 'ai_focus_outputs_only') && cfg.ai_focus_outputs_only
        delete_if_exists(fullfile(fig_dir, '01_regime_divergence.png'));
        delete_if_exists(fullfile(fig_dir, '02_residual_map.png'));
        delete_if_exists(fullfile(fig_dir, '03_stability_mismatch.png'));
        delete_if_exists(fullfile(fig_dir, '04_cp_ahead_vs_mismatch.png'));
        delete_if_exists(fullfile(fig_dir, '05_regime_limitation_stack.png'));
        delete_if_exists(fullfile(fig_dir, '06_delay_heatmaps.png'));
        delete_if_exists(fullfile(fig_dir, '07_cp_static_margin_distributions.png'));
        delete_if_exists(fullfile(fig_dir, '08_pid_vs_adrc_summary.png'));
        delete_if_exists(fullfile(fig_dir, '09_pid_vs_adrc_nominal_midrate.png'));
        delete_if_exists(fullfile(fig_dir, '10_pid_vs_adrc_stress_highrate.png'));
        delete_if_exists(fullfile(fig_dir, '11_controller_uncertainty_frontier.png'));
        delete_if_exists(fullfile(fig_dir, '12_pid_vs_adrc_vs_ai_adrc_profile.png'));
        delete_if_exists(fullfile(fig_dir, '23_adrc_vs_pid_success_rate.png'));
        delete_if_exists(fullfile(fig_dir, '24_adrc_bandwidth_delay_noise_frontier.png'));
        delete_if_exists(fullfile(fig_dir, '25_adrc_observer_tradeoff.png'));
    end
end

function delete_if_exists(path_to_file)
    if exist(path_to_file, 'file')
        delete(path_to_file);
    end
end