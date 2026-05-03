%% SimToReal_TVC_Research.m
% ADRC vs PID for sim-to-real gap mitigation in statically unstable TVC rockets.
%
% This script provides:
% 1) Side-by-side PID vs ADRC under perfect simulation conditions.
% 2) Side-by-side PID vs ADRC under realistic disturbance/mismatch conditions.
% 3) PID auto-tuning in ideal conditions (to ensure fair theoretical baseline).
% 4) Model-real mismatch ablation across key factors with flight-region importance:
%    aero, loop delay, slew rate, changing MMI, CG shift, thrust profile.

clear; close all; clc;
rng(12); % reproducible

fprintf('\n======================================================================\n');
fprintf('ADRC vs PID: PERFECT vs REALISTIC + MISMATCH ABLATION\n');
fprintf('======================================================================\n\n');

%% ----------------------------- Configuration -----------------------------
cfg = struct();

% Time base
cfg.dt = 0.002;

% Vehicle nominal model (onboard)
cfg.mass0_nom = 1.85;
cfg.mass_prop_nom = 0.22;
cfg.burn_time = 1.65;
cfg.coast_extension_s = 3.5;
cfg.t_final = cfg.burn_time + cfg.coast_extension_s;
cfg.n = floor(cfg.t_final / cfg.dt) + 1;
cfg.t = (0:cfg.n - 1)' * cfg.dt;
cfg.Iyy_nom = 0.095;
cfg.thrust_nom = 46;
cfg.lever_arm = 0.24;
cfg.k_theta_nom = 1.30;      % destabilizing stiffness
cfg.k_q_nom = 0.24;          % damping
cfg.max_delta_deg = 10;

% Truth default (realistic)
cfg.mass0_truth = 1.75;
cfg.mass_prop_truth = 0.20;
cfg.Iyy_truth = 0.090;
cfg.thrust_truth_scale = 0.96;
cfg.k_theta_truth = 1.55;
cfg.k_q_truth = 0.19;

% Sensor
cfg.theta_noise_deg = 0.20;
cfg.q_noise_deg_s = 1.20;

% Disturbance profile
cfg.dist_step_time = 1.10;
cfg.dist_step_amp = 0.35;
cfg.aero_shift_time = 1.85;
cfg.aero_shift_gain = 1.35;
cfg.gust_freq_hz = 3.5;
cfg.gust_amp = 0.06;
cfg.cp_shift_gain = 1.30;
cfg.cg_shift_gain = 1.55;

% Actuator realistic regime (required 30-80 deg/s and ~40-50ms)
cfg.actuator_delay_nom_s = 0.045;
cfg.actuator_rate_nom_deg_s = 60;
cfg.servo_deadband_deg = 0.15;

% Controller references
cfg.theta_ref_deg = 0;
cfg.theta0_deg = 6.0;

% PID gains (will be tuned in ideal sim)
cfg.pid_kp = 5.5;
cfg.pid_kd = 0.9;

% ADRC/LESO
cfg.adrc_b0 = cfg.thrust_nom * cfg.lever_arm / cfg.Iyy_nom;
cfg.omega_o = 20;
cfg.beta1 = 3 * cfg.omega_o;
cfg.beta2 = 3 * cfg.omega_o^2;
cfg.beta3 = cfg.omega_o^3;
cfg.adrc_kp = 16;
cfg.adrc_kd = 5.5;

% Stability criteria
cfg.max_abs_theta_stable_deg = 45;
cfg.max_abs_q_stable_deg_s = 450;

% Output
out_dir = fullfile(pwd, 'outputs', 'sim_to_real');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
out_dir_graphs = fullfile(out_dir, 'graphs');
out_dir_sheets = fullfile(out_dir, 'sheets');
if ~exist(out_dir_graphs, 'dir')
    mkdir(out_dir_graphs);
end
if ~exist(out_dir_sheets, 'dir')
    mkdir(out_dir_sheets);
end

% Organize previously generated outputs into dedicated folders.
organize_existing_outputs(out_dir, out_dir_graphs, out_dir_sheets);

%% ---------------------- Build Perfect vs Realistic Configs ----------------
cfg_ideal = cfg;
cfg_ideal.mass0_truth = cfg_ideal.mass0_nom;
cfg_ideal.mass_prop_truth = cfg_ideal.mass_prop_nom;
cfg_ideal.Iyy_truth = cfg_ideal.Iyy_nom;
cfg_ideal.thrust_truth_scale = 1.00;
cfg_ideal.k_theta_truth = cfg_ideal.k_theta_nom;
cfg_ideal.k_q_truth = cfg_ideal.k_q_nom;
cfg_ideal.theta_noise_deg = 0.0;
cfg_ideal.q_noise_deg_s = 0.0;
cfg_ideal.dist_step_amp = 0.0;
cfg_ideal.gust_amp = 0.0;
cfg_ideal.aero_shift_gain = 1.0;

cfg_real = cfg; % keep realistic truth and disturbances

% User-editable rocket profile for explicit-axis sensitivity plots.
cfg_profile = struct();
cfg_profile.aero_scale = 1.00;
cfg_profile.loop_delay_ms = 45;
cfg_profile.slew_rate_deg_s = 60;
cfg_profile.mmi_inertia_scale = 1.00;
cfg_profile.cg_moment_coeff = 0.20;
cfg_profile.thrust_profile_amp = 0.10;
cfg_profile.gust_scale = 1.00;
cfg_profile.dist_step_scale = 1.00;

mismatch_ideal = struct('Iyy_scale', 1.0);
mismatch_real = struct('Iyy_scale', 1.20); % internal model error in realistic run

%% ---------------------- PID tuning in ideal conditions --------------------
fprintf('Auto-tuning PID in ideal simulation for fair theoretical baseline...\n');
[pid_kp_best, pid_kd_best, tune_cost] = tune_pid_ideal(cfg_ideal, mismatch_ideal);
cfg_ideal.pid_kp = pid_kp_best;
cfg_ideal.pid_kd = pid_kd_best;
cfg_real.pid_kp = pid_kp_best;
cfg_real.pid_kd = pid_kd_best;

fprintf('Best PID gains (ideal): Kp=%.3f, Kd=%.3f, cost=%.4f\n\n', pid_kp_best, pid_kd_best, tune_cost);

%% -------------------- Side-by-side: Perfect conditions --------------------
fprintf('Running side-by-side under PERFECT conditions...\n');

ideal_opts = default_run_opts(cfg_ideal);
ideal_delay_s = 0.0;
ideal_rate_deg_s = 1e6;

pid_ideal = run_flight('PID', cfg_ideal, mismatch_ideal, ideal_delay_s, ideal_rate_deg_s, ideal_opts);
adrc_ideal = run_flight('ADRC', cfg_ideal, mismatch_ideal, ideal_delay_s, ideal_rate_deg_s, ideal_opts);

pid_ideal_met = compute_metrics(pid_ideal, cfg_ideal);
adrc_ideal_met = compute_metrics(adrc_ideal, cfg_ideal);

%% ------------------- Side-by-side: Realistic conditions -------------------
fprintf('Running side-by-side under REALISTIC conditions...\n');

real_opts = default_run_opts(cfg_real);
real_delay_s = cfg_real.actuator_delay_nom_s;
real_rate_deg_s = cfg_real.actuator_rate_nom_deg_s;

pid_real = run_flight('PID', cfg_real, mismatch_real, real_delay_s, real_rate_deg_s, real_opts);
adrc_real = run_flight('ADRC', cfg_real, mismatch_real, real_delay_s, real_rate_deg_s, real_opts);

pid_real_met = compute_metrics(pid_real, cfg_real);
adrc_real_met = compute_metrics(adrc_real, cfg_real);

fprintf('\n--- Perfect Conditions ---\n');
print_summary('PID (ideal)', pid_ideal_met);
print_summary('ADRC (ideal)', adrc_ideal_met);

fprintf('--- Realistic Conditions ---\n');
print_summary('PID (realistic)', pid_real_met);
print_summary('ADRC (realistic)', adrc_real_met);

%% ------------------ LESO convergence in realistic scenario -----------------
conv_met = eso_convergence_metric(adrc_real, cfg_real);
fprintf('LESO disturbance convergence (realistic): %.3f s\n\n', conv_met.settle_s);

%% --------------- Factor ablation with flight-region importance ------------
fprintf('Running mismatch ablation across modeling factors...\n');
abl_tbl = mismatch_factor_ablation(cfg_ideal, cfg_real, pid_kp_best, pid_kd_best);
rank_tbl = rank_mismatch_factors(abl_tbl);

fprintf('Building high-dimensional disturbance importance atlas...\n');
atlas = build_disturbance_importance_atlas(cfg_real, pid_kp_best, pid_kd_best, out_dir);

fprintf('Building explicit-axis rocket-specific sensitivity sweeps...\n');
explicit = build_explicit_axis_sweeps(cfg_real, pid_kp_best, pid_kd_best, cfg_profile, out_dir);

fprintf('Computing normalized cross-factor importance (fair scaling)...\n');
norm_imp_tbl = build_normalized_importance_table(explicit.rank_tbl, atlas.global_tbl);

fprintf('Running Gemini-style 4-variable Monte Carlo ablation matrix...\n');
gemini = run_gemini_ablation_matrix(cfg_real, pid_kp_best, pid_kd_best, out_dir);
make_spaghetti_plot(cfg_real, pid_kp_best, pid_kd_best, out_dir);
hw_bw = compute_hardware_bandwidth_bound(cfg_real);
make_observer_bandwidth_figure(gemini.matrix_tbl, cfg_real.omega_o, hw_bw, out_dir);

%% -------------------- Existing robustness / HIL / forensic ---------------
fprintf('Running robustness boundary sweep...\n');
uncertainty_grid = 0:5:60;
rob_tbl = robustness_boundary_sweep(cfg_real, uncertainty_grid, real_delay_s, real_rate_deg_s);

fprintf('Running HIL-style latency/rate matrix...\n');
latency_set_ms = [40, 45, 50];
rate_set_deg_s = [30, 50, 80];
hil_tbl = hil_latency_rate_matrix(cfg_real, mismatch_real, latency_set_ms, rate_set_deg_s);

fprintf('Running residual forensic attribution...\n');
forensic_tbl = residual_forensics(cfg_real, mismatch_real);

fprintf('Building phase portrait / PSD / recovery envelope visuals...\n');
make_z3_forensic_figure(adrc_real, cfg_real, out_dir);
make_phase_portrait_figure(pid_real, adrc_real, cfg_real, out_dir);
make_control_psd_figure(pid_real, adrc_real, out_dir);
recovery_tbl = make_recovery_envelope(cfg_real, pid_kp_best, pid_kd_best, mismatch_real, real_delay_s, real_rate_deg_s, out_dir);

%% ------------------------- Save outputs -----------------------------------
side_tbl = table( ...
    string({'PID'; 'ADRC'; 'PID'; 'ADRC'}), ...
    string({'perfect'; 'perfect'; 'realistic'; 'realistic'}), ...
    [pid_ideal_met.theta_rmse_deg; adrc_ideal_met.theta_rmse_deg; pid_real_met.theta_rmse_deg; adrc_real_met.theta_rmse_deg], ...
    [pid_ideal_met.max_abs_theta_deg; adrc_ideal_met.max_abs_theta_deg; pid_real_met.max_abs_theta_deg; adrc_real_met.max_abs_theta_deg], ...
    [pid_ideal_met.max_abs_q_deg_s; adrc_ideal_met.max_abs_q_deg_s; pid_real_met.max_abs_q_deg_s; adrc_real_met.max_abs_q_deg_s], ...
    [pid_ideal_met.control_thd_pct; adrc_ideal_met.control_thd_pct; pid_real_met.control_thd_pct; adrc_real_met.control_thd_pct], ...
    [pid_ideal_met.is_stable; adrc_ideal_met.is_stable; pid_real_met.is_stable; adrc_real_met.is_stable], ...
    'VariableNames', {'controller', 'condition', 'theta_rmse_deg', 'max_abs_theta_deg', 'max_abs_q_deg_s', 'control_thd_pct', 'is_stable'});

conv_tbl = table(conv_met.disturbance_step_time_s, conv_met.settle_s, conv_met.final_error_abs, ...
    'VariableNames', {'disturbance_step_time_s', 'eso_settle_time_s', 'eso_final_abs_error'});

writetable(side_tbl, fullfile(out_dir_sheets, 'pid_adrc_side_by_side.csv'));
writetable(conv_tbl, fullfile(out_dir_sheets, 'eso_convergence_metrics.csv'));
writetable(abl_tbl, fullfile(out_dir_sheets, 'mismatch_ablation_regions.csv'));
writetable(rank_tbl, fullfile(out_dir_sheets, 'mismatch_factor_ranking.csv'));
writetable(atlas.samples_tbl, fullfile(out_dir_sheets, 'disturbance_atlas_samples.csv'));
writetable(atlas.global_tbl, fullfile(out_dir_sheets, 'disturbance_atlas_global_importance.csv'));
writetable(atlas.local_tbl, fullfile(out_dir_sheets, 'disturbance_atlas_local_importance.csv'));
writetable(atlas.pc1_loadings_tbl, fullfile(out_dir_sheets, 'pca_pc1_loadings.csv'));
writetable(atlas.pc1_bins_tbl, fullfile(out_dir_sheets, 'pca_stability_bins.csv'));
writetable(explicit.sweep_tbl, fullfile(out_dir_sheets, 'explicit_factor_sweeps.csv'));
writetable(explicit.rank_tbl, fullfile(out_dir_sheets, 'explicit_factor_ranking.csv'));
writetable(norm_imp_tbl, fullfile(out_dir_sheets, 'normalized_factor_importance.csv'));
writetable(gemini.matrix_tbl, fullfile(out_dir_sheets, 'gemini_ablation_matrix.csv'));
writetable(gemini.boundary_tbl, fullfile(out_dir_sheets, 'gemini_stability_boundary.csv'));
writetable(rob_tbl, fullfile(out_dir_sheets, 'robustness_boundary_sweep.csv'));
writetable(hil_tbl, fullfile(out_dir_sheets, 'hil_latency_rate_matrix.csv'));
writetable(forensic_tbl, fullfile(out_dir_sheets, 'observer_residual_forensics.csv'));
writetable(recovery_tbl, fullfile(out_dir_sheets, 'recovery_envelope.csv'));

key_tbl = make_key_findings_table(side_tbl, conv_tbl, rob_tbl, explicit.rank_tbl);
writetable(key_tbl, fullfile(out_dir_sheets, 'key_findings_summary.csv'));

make_side_by_side_figure(pid_ideal, adrc_ideal, pid_real, adrc_real, cfg_real, out_dir_graphs);
make_ablation_heatmap(abl_tbl, out_dir_graphs);
make_intuitive_summary_figure(side_tbl, conv_tbl, rob_tbl, explicit.rank_tbl, out_dir_graphs);
make_normalized_importance_figure(norm_imp_tbl, out_dir_graphs);

% Route earlier figure outputs and remaining legacy files into folders.
organize_existing_outputs(out_dir, out_dir_graphs, out_dir_sheets);

fprintf('\nSaved outputs:\n');
fprintf('  Sheets folder: %s\n', out_dir_sheets);
fprintf('  Graphs folder: %s\n', out_dir_graphs);
fprintf('  - %s\n', fullfile(out_dir_sheets, 'pid_adrc_side_by_side.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'mismatch_ablation_regions.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'mismatch_factor_ranking.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'disturbance_atlas_samples.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'disturbance_atlas_global_importance.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'disturbance_atlas_local_importance.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'pca_pc1_loadings.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'pca_stability_bins.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'explicit_factor_sweeps.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'explicit_factor_ranking.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'normalized_factor_importance.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'gemini_ablation_matrix.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'gemini_stability_boundary.csv'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'spaghetti_pitch_theta.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'side_by_side_conditions.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'mismatch_ablation_heatmap.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'disturbance_importance_atlas.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'pca_stability_map.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'explicit_factor_sweeps.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'normalized_factor_importance.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'gemini_gap_heatmap.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'observer_bandwidth_tradeoff.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'intuitive_findings_summary.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'phase_portrait_realistic.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'adrc_z3_forensic_signature.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'control_command_psd.png'));
fprintf('  - %s\n', fullfile(out_dir_graphs, 'recovery_envelope.png'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'key_findings_summary.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'eso_convergence_metrics.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'robustness_boundary_sweep.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'hil_latency_rate_matrix.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'observer_residual_forensics.csv'));
fprintf('  - %s\n', fullfile(out_dir_sheets, 'recovery_envelope.csv'));

fprintf('\nStudy complete.\n');

%% =============================== Functions ================================

function [kp_best, kd_best, best_cost] = tune_pid_ideal(cfg, mismatch)
    kp_grid = 2.0:0.5:10.0;
    kd_grid = 0.2:0.2:2.0;

    best_cost = inf;
    kp_best = cfg.pid_kp;
    kd_best = cfg.pid_kd;

    opts = default_run_opts(cfg);

    for kp = kp_grid
        for kd = kd_grid
            cfg_try = cfg;
            cfg_try.pid_kp = kp;
            cfg_try.pid_kd = kd;

            run = run_flight('PID', cfg_try, mismatch, 0.0, 1e6, opts);
            met = compute_metrics(run, cfg_try);

            cost = met.theta_rmse_deg + 0.02 * met.max_abs_q_deg_s + 0.005 * met.control_thd_pct;
            if ~met.is_stable
                cost = cost + 1e3;
            end

            if cost < best_cost
                best_cost = cost;
                kp_best = kp;
                kd_best = kd;
            end
        end
    end
end

function organize_existing_outputs(out_dir, out_dir_graphs, out_dir_sheets)
    if ~exist(out_dir, 'dir')
        return;
    end

    items = dir(out_dir);
    for i = 1:numel(items)
        if items(i).isdir
            continue;
        end

        src = fullfile(items(i).folder, items(i).name);
        [~, ~, ext] = fileparts(items(i).name);
        ext_l = lower(ext);

        if strcmp(ext_l, '.png')
            dst = fullfile(out_dir_graphs, items(i).name);
        elseif strcmp(ext_l, '.csv')
            dst = fullfile(out_dir_sheets, items(i).name);
        else
            continue;
        end

        if strcmp(src, dst)
            continue;
        end

        if exist(dst, 'file')
            delete(dst);
        end
        movefile(src, dst);
    end
end

function opts = default_run_opts(cfg)
    opts = struct();
    opts.enable_disturbance = true;
    opts.sensor_noise_scale = 1.0;
    opts.k_theta_scale = 1.0;
    opts.k_q_scale = 1.0;
    opts.thrust_profile_amp = 0.0;
    opts.thrust_profile_hz = 2.0;
    opts.cg_moment_coeff = 0.0;
    opts.mass_scale = 1.0;
    opts.prop_mass_scale = 1.0;
    opts.inertia_scale = 1.0;
    opts.dist_step_amp_scale = 1.0;
    opts.gust_amp_scale = 1.0;
    opts.aero_shift_gain = cfg.aero_shift_gain;
    opts.cp_shift_gain = cfg.cp_shift_gain;
    opts.cg_shift_gain = cfg.cg_shift_gain;
    opts.regime_shift_time = cfg.burn_time;
    opts.thrust_cutoff_scale = 0.0;
    opts.thrust_misalignment_deg = 0.0;
    opts.deadband_deg = cfg.servo_deadband_deg;
    opts.omega_o_override = NaN;
end

function run = run_flight(controller_mode, cfg, mismatch, actuator_delay_s, actuator_rate_deg_s, opts)
    n = cfg.n;
    dt = cfg.dt;
    t = cfg.t;

    x1 = zeros(n, 1); % theta
    x2 = zeros(n, 1); % q
    x1(1) = deg2rad(cfg.theta0_deg);

    z1 = zeros(n, 1);
    z2 = zeros(n, 1);
    z3 = zeros(n, 1);

    u_cmd = zeros(n, 1);
    delta = zeros(n, 1);
    f_total_true = zeros(n, 1);
    thrust_trace = zeros(n, 1);
    cp_shift_trace = ones(n, 1);
    cg_shift_trace = ones(n, 1);

    delay_steps = max(1, round(actuator_delay_s / dt));
    cmd_buffer = zeros(delay_steps, 1);

    delta_lim = deg2rad(cfg.max_delta_deg);
    delta_rate_lim = deg2rad(actuator_rate_deg_s);
    deadband_rad = deg2rad(max(opts.deadband_deg, 0));

    omega_o_eff = cfg.omega_o;
    if isfield(opts, 'omega_o_override') && ~isnan(opts.omega_o_override)
        omega_o_eff = max(1.0, opts.omega_o_override);
    end
    beta1 = 3 * omega_o_eff;
    beta2 = 3 * omega_o_eff^2;
    beta3 = omega_o_eff^3;

    for k = 1:n - 1
        tk = t(k);

        [m_truth, Iyy_truth] = truth_mass_inertia(cfg, tk);
        m_truth = m_truth * opts.mass_scale;
        Iyy_truth = Iyy_truth * opts.inertia_scale;

        thrust_base = cfg.thrust_nom * cfg.thrust_truth_scale;
        thrust_profile = 1 + opts.thrust_profile_amp * sin(2 * pi * opts.thrust_profile_hz * tk);
        if tk <= cfg.burn_time
            thrust_truth = thrust_base * thrust_profile;
        else
            thrust_truth = thrust_base * opts.thrust_cutoff_scale;
        end

        k_theta_truth = cfg.k_theta_truth * opts.k_theta_scale;
        if tk >= cfg.aero_shift_time
            k_theta_truth = k_theta_truth * opts.aero_shift_gain;
        end
        cp_gain = 1.0;
        cg_gain = 1.0;
        if tk >= opts.regime_shift_time
            cp_gain = opts.cp_shift_gain;
            cg_gain = opts.cg_shift_gain;
            k_theta_truth = k_theta_truth * cp_gain;
        end
        k_q_truth = cfg.k_q_truth * opts.k_q_scale;

        d_inj = 0;
        if opts.enable_disturbance
            if tk >= cfg.dist_step_time
                d_inj = d_inj + cfg.dist_step_amp * opts.dist_step_amp_scale;
            end
            d_inj = d_inj + cfg.gust_amp * opts.gust_amp_scale * sin(2 * pi * cfg.gust_freq_hz * tk);
        end

        y_theta = x1(k) + deg2rad(cfg.theta_noise_deg * opts.sensor_noise_scale) * randn();
        y_q = x2(k) + deg2rad(cfg.q_noise_deg_s * opts.sensor_noise_scale) * randn();

        Iyy_model = cfg.Iyy_nom * mismatch.Iyy_scale;
        b0 = cfg.adrc_b0 * (cfg.Iyy_nom / max(Iyy_model, 1e-6));

        switch upper(controller_mode)
            case 'PID'
                e = y_theta - deg2rad(cfg.theta_ref_deg);
                u_virtual = -cfg.pid_kp * e - cfg.pid_kd * y_q;
            case 'ADRC'
                e_obs = z1(k) - y_theta;
                z1dot = z2(k) - beta1 * e_obs;
                z2dot = z3(k) - beta2 * e_obs + b0 * u_cmd(max(k - 1, 1));
                z3dot = -beta3 * e_obs;

                z1(k + 1) = z1(k) + z1dot * dt;
                z2(k + 1) = z2(k) + z2dot * dt;
                z3(k + 1) = z3(k) + z3dot * dt;

                e1 = z1(k) - deg2rad(cfg.theta_ref_deg);
                v = -cfg.adrc_kp * e1 - cfg.adrc_kd * z2(k);
                u_virtual = (v - z3(k)) / max(b0, 1e-6);

                alpha_cmd = 0.65;
                u_virtual = (1 - alpha_cmd) * u_cmd(max(k - 1, 1)) + alpha_cmd * u_virtual;
            otherwise
                error('Unknown controller_mode: %s', controller_mode);
        end

        u_virtual = max(min(u_virtual, delta_lim), -delta_lim);
        cmd_buffer = [u_virtual; cmd_buffer(1:end - 1)];
        delayed_cmd = cmd_buffer(end);

        cmd_err = delayed_cmd - delta(k);
        if abs(cmd_err) < deadband_rad
            delayed_cmd_eff = delta(k);
        else
            delayed_cmd_eff = delayed_cmd - sign(cmd_err) * deadband_rad;
        end

        d_delta = delayed_cmd_eff - delta(k);
        d_delta = max(min(d_delta / dt, delta_rate_lim), -delta_rate_lim);
        delta(k + 1) = delta(k) + d_delta * dt;
        delta(k + 1) = max(min(delta(k + 1), delta_lim), -delta_lim);

        u_cmd(k) = delayed_cmd;

        misalign_rad = deg2rad(opts.thrust_misalignment_deg);
        m_tvc = thrust_truth * cfg.lever_arm * sin(delta(k) + misalign_rad);
        m_aero = +k_theta_truth * x1(k) - k_q_truth * x2(k);
        m_cg = opts.cg_moment_coeff * cg_gain * x1(k);

        x2dot = (m_tvc + m_aero + m_cg + d_inj) / max(Iyy_truth, 1e-6);
        x1dot = x2(k);

        x2(k + 1) = x2(k) + x2dot * dt;
        x1(k + 1) = x1(k) + x1dot * dt;

        b_true = thrust_truth * cfg.lever_arm / max(Iyy_truth, 1e-6);
        f_total_true(k) = x2dot - b_true * delta(k);
        thrust_trace(k) = thrust_truth;
        cp_shift_trace(k) = cp_gain;
        cg_shift_trace(k) = cg_gain;

        x1(k + 1) = max(min(x1(k + 1), deg2rad(80)), -deg2rad(80));
        x2(k + 1) = max(min(x2(k + 1), deg2rad(800)), -deg2rad(800));
    end

    run = struct();
    run.t = t;
    run.theta = x1;
    run.q = x2;
    run.delta = delta;
    run.u_cmd = u_cmd;
    run.z1 = z1;
    run.z2 = z2;
    run.z3 = z3;
    run.f_total_true = f_total_true;
    run.thrust_truth = thrust_trace;
    run.cp_shift_gain = cp_shift_trace;
    run.cg_shift_gain = cg_shift_trace;

    % --- Slew-rate saturation monitor ---
    slew_cmd = abs(diff(delta)) / dt;           % rad/s, length n-1
    sat_mask  = slew_cmd >= delta_rate_lim * 0.99;
    idx_boost_s = t(1:end-1) <= cfg.burn_time;
    run.slew_sat_frac       = mean(sat_mask);             % whole flight
    run.slew_sat_frac_boost = mean(sat_mask(idx_boost_s));% boost only

    run.controller = controller_mode;
end

function [m, Iyy] = truth_mass_inertia(cfg, t)
    if t <= cfg.burn_time
        frac = 1 - t / cfg.burn_time;
        m = cfg.mass0_truth + cfg.mass_prop_truth * frac;
    else
        m = cfg.mass0_truth;
    end
    Iyy = cfg.Iyy_truth * (m / (cfg.mass0_truth + cfg.mass_prop_truth));
end

function met = compute_metrics(run, cfg)
    theta_deg = rad2deg(run.theta);
    q_deg_s = rad2deg(run.q);
    u_deg = rad2deg(run.delta);
    idx_boost = run.t <= cfg.burn_time;
    if ~any(idx_boost)
        idx_boost = true(size(run.t));
    end

    met = struct();
    met.theta_rmse_deg = sqrt(mean(theta_deg .^ 2));
    met.max_abs_theta_deg = max(abs(theta_deg));
    met.max_abs_q_deg_s = max(abs(q_deg_s));
    max_abs_theta_boost_deg = max(abs(theta_deg(idx_boost)));
    max_abs_q_boost_deg_s = max(abs(q_deg_s(idx_boost)));
    met.is_stable = max_abs_theta_boost_deg < cfg.max_abs_theta_stable_deg && ...
                    max_abs_q_boost_deg_s < cfg.max_abs_q_stable_deg_s;
    met.control_thd_pct = control_thd_percent(u_deg);
end

function [rmse_boost, rmse_coast] = phase_rmse_theta(run, cfg)
    theta_deg = rad2deg(run.theta);
    idx_boost = run.t <= cfg.burn_time;
    idx_coast = run.t > cfg.burn_time;

    rmse_boost = sqrt(mean(theta_deg(idx_boost) .^ 2));
    rmse_coast = sqrt(mean(theta_deg(idx_coast) .^ 2));
end

function thd_pct = control_thd_percent(u_deg)
    x = u_deg(:) - mean(u_deg);
    N = numel(x);
    if N < 16
        thd_pct = NaN;
        return;
    end

    X = fft(x);
    P2 = abs(X / N);
    P1 = P2(1:floor(N / 2) + 1);
    P1(2:end - 1) = 2 * P1(2:end - 1);

    [~, k1] = max(P1(2:end));
    k1 = k1 + 1;
    fundamental = P1(k1);

    harm = P1;
    harm(k1) = 0;
    harm(1) = 0;
    rms_harm = sqrt(sum(harm .^ 2));

    thd_pct = 100 * rms_harm / max(fundamental, 1e-6);
end

function conv = eso_convergence_metric(run, cfg)
    t = run.t;
    f_hat = run.z3(:);
    f_true = run.f_total_true(:);

    pre_win = find(t >= max(0, cfg.dist_step_time - 0.20) & t < cfg.dist_step_time);
    post_win = find(t >= cfg.dist_step_time + 0.20 & t <= min(cfg.aero_shift_time - 0.05, cfg.t_final));

    if isempty(pre_win) || isempty(post_win)
        conv = struct('disturbance_step_time_s', cfg.dist_step_time, 'settle_s', NaN, 'final_error_abs', NaN);
        return;
    end

    f_pre = mean(f_true(pre_win));
    f_post = mean(f_true(post_win));
    step_amp = f_post - f_pre;
    target = f_pre + 0.90 * step_amp;
    band = 0.10 * max(abs(step_amp), 1e-4);

    idx_window = find(t >= cfg.dist_step_time & t <= min(cfg.aero_shift_time - 0.05, cfg.t_final));
    settle_idx = NaN;
    for kk = idx_window(:)'
        if abs(f_hat(kk) - target) <= band
            settle_idx = kk;
            break;
        end
    end

    if isnan(settle_idx)
        settle_s = NaN;
    else
        settle_s = t(settle_idx) - cfg.dist_step_time;
    end

    conv = struct();
    conv.disturbance_step_time_s = cfg.dist_step_time;
    conv.settle_s = settle_s;
    conv.final_error_abs = mean(abs(f_hat(post_win) - f_true(post_win)));
end

function tbl = mismatch_factor_ablation(cfg_ideal, cfg_real, pid_kp, pid_kd)
    factors = {'aero', 'loop_delay', 'slew_rate', 'mmi_inertia', 'cg_shift', 'thrust_profile'};
    controllers = {'PID', 'ADRC'};

    % Perfect baseline runs (for delta computation)
    cfg_base = cfg_ideal;
    cfg_base.pid_kp = pid_kp;
    cfg_base.pid_kd = pid_kd;

    opts_base = default_run_opts(cfg_base);
    mismatch_base = struct('Iyy_scale', 1.0);

    base_pid = run_flight('PID', cfg_base, mismatch_base, 0.0, 1e6, opts_base);
    base_adrc = run_flight('ADRC', cfg_base, mismatch_base, 0.0, 1e6, opts_base);

    [base_pid_boost, base_pid_coast] = phase_rmse_theta(base_pid, cfg_base);
    [base_adrc_boost, base_adrc_coast] = phase_rmse_theta(base_adrc, cfg_base);

    rows = [];
    labels = strings(0, 1);
    ctrls = strings(0, 1);

    for i = 1:numel(factors)
        for c = 1:numel(controllers)
            cfg_case = cfg_base;
            opts_case = default_run_opts(cfg_case);
            delay_s = 0.0;
            rate_deg_s = 1e6;
            mismatch = struct('Iyy_scale', 1.0);

            switch factors{i}
                case 'aero'
                    opts_case.k_theta_scale = cfg_real.k_theta_truth / cfg_ideal.k_theta_truth;
                    opts_case.k_q_scale = cfg_real.k_q_truth / cfg_ideal.k_q_truth;
                case 'loop_delay'
                    delay_s = cfg_real.actuator_delay_nom_s;
                case 'slew_rate'
                    rate_deg_s = cfg_real.actuator_rate_nom_deg_s;
                case 'mmi_inertia'
                    opts_case.inertia_scale = cfg_real.Iyy_truth / cfg_ideal.Iyy_truth;
                    mismatch.Iyy_scale = 1.20;
                case 'cg_shift'
                    opts_case.cg_moment_coeff = 0.45;
                case 'thrust_profile'
                    opts_case.thrust_profile_amp = 0.18;
                    opts_case.thrust_profile_hz = 2.4;
                otherwise
                    error('Unknown factor');
            end

            run_case = run_flight(controllers{c}, cfg_case, mismatch, delay_s, rate_deg_s, opts_case);
            met = compute_metrics(run_case, cfg_case);
            [rmse_boost, rmse_coast] = phase_rmse_theta(run_case, cfg_case);

            if strcmp(controllers{c}, 'PID')
                d_boost = rmse_boost - base_pid_boost;
                d_coast = rmse_coast - base_pid_coast;
                d_total = met.theta_rmse_deg - compute_metrics(base_pid, cfg_case).theta_rmse_deg;
            else
                d_boost = rmse_boost - base_adrc_boost;
                d_coast = rmse_coast - base_adrc_coast;
                d_total = met.theta_rmse_deg - compute_metrics(base_adrc, cfg_case).theta_rmse_deg;
            end

            rows = [rows; d_total, d_boost, d_coast, met.max_abs_theta_deg, met.control_thd_pct, met.is_stable]; %#ok<AGROW>
            labels(end + 1, 1) = string(factors{i}); %#ok<AGROW>
            ctrls(end + 1, 1) = string(controllers{c}); %#ok<AGROW>
        end
    end

    tbl = table(ctrls, labels, rows(:, 1), rows(:, 2), rows(:, 3), rows(:, 4), rows(:, 5), logical(rows(:, 6)), ...
        'VariableNames', {'controller', 'factor', 'delta_theta_rmse_total_deg', 'delta_theta_rmse_boost_deg', ...
        'delta_theta_rmse_coast_deg', 'max_abs_theta_deg', 'control_thd_pct', 'is_stable'});

    % Add region-importance flag to guide paper scope.
    region_flag = strings(height(tbl), 1);
    for k = 1:height(tbl)
        if abs(tbl.delta_theta_rmse_boost_deg(k)) > abs(tbl.delta_theta_rmse_coast_deg(k))
            region_flag(k) = "boost_dominant";
        else
            region_flag(k) = "coast_dominant";
        end
    end
    tbl.region_importance = region_flag;
end

function tbl = robustness_boundary_sweep(cfg, uncertainty_grid, delay_s, rate_deg_s)
    n = numel(uncertainty_grid);
    pid_stable = false(n, 1);
    adrc_stable = false(n, 1);

    opts = default_run_opts(cfg);

    for i = 1:n
        u = uncertainty_grid(i) / 100;
        mismatch = struct('Iyy_scale', 1 + u);

        opts_i = opts;
        opts_i.k_theta_scale = 1 + 0.7 * u;
        opts_i.k_q_scale = 1 - 0.4 * u;
        opts_i.inertia_scale = 1 + 0.6 * u;

        pid_run = run_flight('PID', cfg, mismatch, delay_s, rate_deg_s, opts_i);
        adrc_run = run_flight('ADRC', cfg, mismatch, delay_s, rate_deg_s, opts_i);

        pid_stable(i) = compute_metrics(pid_run, cfg).is_stable;
        adrc_stable(i) = compute_metrics(adrc_run, cfg).is_stable;
    end

    pid_boundary = stable_boundary(uncertainty_grid, pid_stable);
    adrc_boundary = stable_boundary(uncertainty_grid, adrc_stable);

    if isnan(pid_boundary)
        fprintf('  PID robustness boundary : < %d%% uncertainty\n', uncertainty_grid(1));
    else
        fprintf('  PID robustness boundary : %d%% uncertainty\n', round(pid_boundary));
    end
    if isnan(adrc_boundary)
        fprintf('  ADRC robustness boundary: < %d%% uncertainty\n', uncertainty_grid(1));
    else
        fprintf('  ADRC robustness boundary: %d%% uncertainty\n', round(adrc_boundary));
    end

    tbl = table(uncertainty_grid(:), pid_stable, adrc_stable, ...
        'VariableNames', {'uncertainty_percent', 'pid_stable', 'adrc_stable'});
end

function tbl = hil_latency_rate_matrix(cfg, mismatch, latency_set_ms, rate_set_deg_s)
    rows = [];
    opts = default_run_opts(cfg);
    for i = 1:numel(latency_set_ms)
        for j = 1:numel(rate_set_deg_s)
            run = run_flight('ADRC', cfg, mismatch, latency_set_ms(i) / 1000, rate_set_deg_s(j), opts);
            met = compute_metrics(run, cfg);
            rows = [rows; latency_set_ms(i), rate_set_deg_s(j), met.theta_rmse_deg, ...
                          met.max_abs_theta_deg, met.control_thd_pct, met.is_stable]; %#ok<AGROW>
        end
    end

    tbl = array2table(rows, 'VariableNames', ...
        {'latency_ms', 'rate_deg_s', 'theta_rmse_deg', 'max_abs_theta_deg', 'control_thd_pct', 'is_stable'});
end

function tbl = residual_forensics(cfg, mismatch)
    % Scenario A: actuator-lag dominant
    opts_a = default_run_opts(cfg);
    run_a = run_flight('ADRC', cfg, mismatch, 0.050, 35, opts_a);

    % Scenario B: aero-shift dominant
    opts_b = default_run_opts(cfg);
    opts_b.aero_shift_gain = 1.65;
    run_b = run_flight('ADRC', cfg, mismatch, 0.040, 80, opts_b);

    feat_a = residual_features(run_a);
    feat_b = residual_features(run_b);

    tbl = table( ...
        string({'actuator_lag_dominant'; 'aero_shift_dominant'}), ...
        [feat_a.rms; feat_b.rms], ...
        [feat_a.hf_ratio; feat_b.hf_ratio], ...
        [feat_a.lag_ms; feat_b.lag_ms], ...
        [feat_a.post_burn_bias; feat_b.post_burn_bias], ...
        [feat_a.post_burn_bias_ratio; feat_b.post_burn_bias_ratio], ...
        string({feat_a.signature_class; feat_b.signature_class}), ...
        'VariableNames', {'scenario', 'residual_rms', 'high_freq_energy_ratio', 'estimated_lag_ms', ...
        'post_burnout_bias', 'post_burnout_bias_ratio', 'dominant_signature'});
end

function feat = residual_features(run)
    r = run.z3 - run.f_total_true;
    r = r(:) - mean(r);
    t = run.t(:);

    dt = mean(diff(t));
    n = numel(r);

    burn_idx = find(run.thrust_truth(:) > 0, 1, 'last');
    if isempty(burn_idx)
        burn_idx = floor(0.3 * n);
    end
    post_start = min(n, burn_idx + round(0.10 / max(dt, 1e-6)));
    post_stop = min(n, burn_idx + round(0.80 / max(dt, 1e-6)));
    if post_stop <= post_start
        post_idx = post_start:n;
    else
        post_idx = post_start:post_stop;
    end

    N = numel(r);
    R = fft(r);
    P = abs(R / N);
    P = P(1:floor(N / 2) + 1);
    P(2:end - 1) = 2 * P(2:end - 1);

    cut = max(3, floor(0.25 * numel(P)));
    hf = sum(P(cut:end) .^ 2);
    lf = sum(P(2:cut - 1) .^ 2);

    max_lag_samp = min(round(0.25 / max(dt, 1e-6)), N - 1);
    x = run.z3(:) - mean(run.z3(:));
    y = run.f_total_true(:) - mean(run.f_total_true(:));
    lag_candidates = (-max_lag_samp:max_lag_samp)';
    corr_vals = zeros(size(lag_candidates));
    for ii = 1:numel(lag_candidates)
        lg = lag_candidates(ii);
        if lg >= 0
            xa = x(1 + lg:end);
            ya = y(1:end - lg);
        else
            xa = x(1:end + lg);
            ya = y(1 - lg:end);
        end
        denom = sqrt(sum(xa .^ 2) * sum(ya .^ 2));
        corr_vals(ii) = sum(xa .* ya) / max(denom, 1e-12);
    end
    [~, i_pk] = max(corr_vals);
    lag_ms = 1000 * lag_candidates(i_pk) * dt;

    post_burn_bias = mean(r(post_idx));
    post_burn_bias_ratio = abs(post_burn_bias) / max(std(r), 1e-8);

    signature_class = "mixed";
    if post_burn_bias_ratio >= 0.80 && abs(post_burn_bias) >= 0.01
        signature_class = "aero_shift_dominant";
    elseif (hf / max(lf, 1e-9)) >= 1.10 || abs(lag_ms) >= 15
        signature_class = "lag_dominant";
    end

    feat = struct();
    feat.rms = sqrt(mean(r .^ 2));
    feat.hf_ratio = hf / max(lf, 1e-9);
    feat.lag_ms = lag_ms;
    feat.post_burn_bias = post_burn_bias;
    feat.post_burn_bias_ratio = post_burn_bias_ratio;
    feat.signature_class = signature_class;
end

function make_side_by_side_figure(pid_ideal, adrc_ideal, pid_real, adrc_real, cfg, out_dir)
    f = figure('Position', [100, 80, 1400, 900], 'ToolBar', 'none');
    tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(pid_ideal.t, rad2deg(pid_ideal.theta), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); hold on;
    plot(adrc_ideal.t, rad2deg(adrc_ideal.theta), '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.8);
    yline(cfg.theta_ref_deg, ':k');
    grid on;
    ylabel('Theta (deg)');
    title('Perfect Simulation: PID vs ADRC');
    legend('PID', 'ADRC', 'Location', 'best');

    nexttile;
    plot(pid_real.t, rad2deg(pid_real.theta), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); hold on;
    plot(adrc_real.t, rad2deg(adrc_real.theta), '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.8);
    yline(cfg.theta_ref_deg, ':k');
    xline(cfg.dist_step_time, ':', 'Dist step');
    xline(cfg.aero_shift_time, ':', 'Aero shift');
    grid on;
    ylabel('Theta (deg)');
    title('Realistic Disturbance + Mismatch: PID vs ADRC');

    nexttile;
    plot(pid_ideal.t, rad2deg(pid_ideal.delta), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); hold on;
    plot(adrc_ideal.t, rad2deg(adrc_ideal.delta), '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.8);
    grid on;
    ylabel('Delta (deg)');
    title('Control Effort (Perfect)');

    nexttile;
    plot(pid_real.t, rad2deg(pid_real.delta), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); hold on;
    plot(adrc_real.t, rad2deg(adrc_real.delta), '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.8);
    grid on;
    ylabel('Delta (deg)');
    xlabel('Time (s)');
    title('Control Effort (Realistic)');

    sgtitle('Side-by-Side Controller Comparison: Theory vs Reality');
    saveas(f, fullfile(out_dir, 'side_by_side_conditions.png'));
    close(f);
end

function make_ablation_heatmap(abl_tbl, out_dir)
    factors = unique(abl_tbl.factor, 'stable');
    controllers = unique(abl_tbl.controller, 'stable');

    H = zeros(numel(factors), numel(controllers));
    for i = 1:numel(factors)
        for j = 1:numel(controllers)
            idx = abl_tbl.factor == factors(i) & abl_tbl.controller == controllers(j);
            H(i, j) = abl_tbl.delta_theta_rmse_total_deg(idx);
        end
    end

    f = figure('Position', [160, 180, 760, 520], 'ToolBar', 'none');
    imagesc(H);
    colormap('parula');
    colorbar;
    xticks(1:numel(controllers));
    xticklabels(cellstr(controllers));
    yticks(1:numel(factors));
    yticklabels(cellstr(factors));
    xlabel('Controller');
    ylabel('Mismatch factor');
    title('Ablation: Increase in Theta RMSE from Perfect Baseline (deg)');

    for i = 1:size(H, 1)
        for j = 1:size(H, 2)
            text(j, i, sprintf('%.2f', H(i, j)), 'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
        end
    end

    saveas(f, fullfile(out_dir, 'mismatch_ablation_heatmap.png'));
    close(f);
end

function print_summary(label, met)
    fprintf('%s\n', label);
    fprintf('  Theta RMSE         : %.3f deg\n', met.theta_rmse_deg);
    fprintf('  Max |theta|        : %.3f deg\n', met.max_abs_theta_deg);
    fprintf('  Max |q|            : %.3f deg/s\n', met.max_abs_q_deg_s);
    fprintf('  Control THD        : %.2f %%\n', met.control_thd_pct);
    fprintf('  Stable flight      : %s\n\n', string(logical(met.is_stable)));
end

function b = stable_boundary(grid, is_stable)
    idx = find(is_stable, 1, 'last');
    if isempty(idx)
        b = NaN;
    else
        b = grid(idx);
    end
end

function rank_tbl = rank_mismatch_factors(abl_tbl)
    factors = unique(abl_tbl.factor, 'stable');
    n = numel(factors);

    avg_total = zeros(n, 1);
    avg_boost = zeros(n, 1);
    avg_coast = zeros(n, 1);
    dominant_region = strings(n, 1);

    for i = 1:n
        idx = abl_tbl.factor == factors(i);
        avg_total(i) = mean(abs(abl_tbl.delta_theta_rmse_total_deg(idx)));
        avg_boost(i) = mean(abs(abl_tbl.delta_theta_rmse_boost_deg(idx)));
        avg_coast(i) = mean(abs(abl_tbl.delta_theta_rmse_coast_deg(idx)));

        if avg_boost(i) >= avg_coast(i)
            dominant_region(i) = "boost_dominant";
        else
            dominant_region(i) = "coast_dominant";
        end
    end

    rank_tbl = table(factors, avg_total, avg_boost, avg_coast, dominant_region, ...
        'VariableNames', {'factor', 'avg_abs_delta_total_deg', 'avg_abs_delta_boost_deg', 'avg_abs_delta_coast_deg', 'dominant_region'});

    rank_tbl = sortrows(rank_tbl, 'avg_abs_delta_total_deg', 'descend');
    rank_tbl.rank = (1:height(rank_tbl))';
    rank_tbl = movevars(rank_tbl, 'rank', 'Before', 'factor');
end

function atlas = build_disturbance_importance_atlas(cfg_real, pid_kp, pid_kd, out_dir)
    % Sample high-dimensional disturbance/mismatch space and estimate
    % factor importance globally and locally across the manifold.

    cfg_atlas = cfg_real;
    cfg_atlas.dt = 0.004;
    cfg_atlas.n = floor(cfg_atlas.t_final / cfg_atlas.dt) + 1;
    cfg_atlas.t = (0:cfg_atlas.n - 1)' * cfg_atlas.dt;
    cfg_atlas.pid_kp = pid_kp;
    cfg_atlas.pid_kd = pid_kd;

    factor_names = {
        'aero_scale', ...
        'loop_delay_ms', ...
        'slew_rate_deg_s', ...
        'mmi_inertia_scale', ...
        'cg_moment_coeff', ...
        'thrust_profile_amp', ...
        'gust_scale', ...
        'dist_step_scale'};

    ranges = [
        0.70, 1.60;   % aero scale
        0.0,  60.0;   % loop delay ms
        30.0, 120.0;  % slew rate
        0.75, 1.35;   % inertia/MMI scaling
        0.00, 0.80;   % CG shift moment coefficient
        0.00, 0.25;   % thrust ripple amplitude
        0.00, 2.00;   % gust multiplier
        0.00, 2.00];  % step disturbance multiplier

    n_samples = 220;
    d = size(ranges, 1);
    Xn = rand(n_samples, d);
    X = zeros(n_samples, d);
    for j = 1:d
        X(:, j) = ranges(j, 1) + Xn(:, j) * (ranges(j, 2) - ranges(j, 1));
    end

    % Ideal baselines for delta metrics
    base_opts = default_run_opts(cfg_atlas);
    mismatch_base = struct('Iyy_scale', 1.0);
    run_pid_base = run_flight('PID', cfg_atlas, mismatch_base, 0.0, 1e6, base_opts);
    run_adrc_base = run_flight('ADRC', cfg_atlas, mismatch_base, 0.0, 1e6, base_opts);
    met_pid_base = compute_metrics(run_pid_base, cfg_atlas);
    met_adrc_base = compute_metrics(run_adrc_base, cfg_atlas);

    y_pid = zeros(n_samples, 1);
    y_adrc = zeros(n_samples, 1);
    s_pid = zeros(n_samples, 1);
    s_adrc = zeros(n_samples, 1);

    for i = 1:n_samples
        opts = default_run_opts(cfg_atlas);
        opts.k_theta_scale = X(i, 1);
        opts.k_q_scale = max(0.4, 1.25 - 0.35 * X(i, 1));
        opts.inertia_scale = X(i, 4);
        opts.cg_moment_coeff = X(i, 5);
        opts.thrust_profile_amp = X(i, 6);
        opts.gust_amp_scale = X(i, 7);
        opts.dist_step_amp_scale = X(i, 8);

        delay_s = X(i, 2) / 1000;
        slew_deg_s = X(i, 3);

        mismatch = struct('Iyy_scale', 1.0);

        run_pid = run_flight('PID', cfg_atlas, mismatch, delay_s, slew_deg_s, opts);
        run_adrc = run_flight('ADRC', cfg_atlas, mismatch, delay_s, slew_deg_s, opts);

        met_pid = compute_metrics(run_pid, cfg_atlas);
        met_adrc = compute_metrics(run_adrc, cfg_atlas);

        y_pid(i) = met_pid.theta_rmse_deg - met_pid_base.theta_rmse_deg;
        y_adrc(i) = met_adrc.theta_rmse_deg - met_adrc_base.theta_rmse_deg;
        s_pid(i) = met_pid.is_stable;
        s_adrc(i) = met_adrc.is_stable;
    end

    % Standardize factors for regression and manifold mapping.
    Xz = zscore_cols(X);

    % Global importance from standardized linear model coefficients.
    bz_pid = Xz \ zscore_cols(y_pid);
    bz_adrc = Xz \ zscore_cols(y_adrc);

    global_tbl = table(string(factor_names(:)), abs(bz_pid(:)), abs(bz_adrc(:)), ...
        'VariableNames', {'factor', 'importance_pid', 'importance_adrc'});

    % 2D manifold via SVD-based PCA.
    [U, S, V] = svd(Xz, 'econ');
    score = U * S;
    pc1 = score(:, 1);
    pc2 = score(:, 2);

    % Make PC1 orientation consistent for interpretation (positive aero loading).
    if V(1, 1) < 0
        pc1 = -pc1;
        V(:, 1) = -V(:, 1);
    end

    pc1_loadings_tbl = table(string(factor_names(:)), V(:, 1), abs(V(:, 1)), ...
        'VariableNames', {'factor', 'pc1_loading', 'abs_pc1_loading'});
    pc1_loadings_tbl = sortrows(pc1_loadings_tbl, 'abs_pc1_loading', 'descend');
    pc1_loadings_tbl.rank = (1:height(pc1_loadings_tbl))';
    pc1_loadings_tbl = movevars(pc1_loadings_tbl, 'rank', 'Before', 'factor');

    % Local importance per cell in manifold.
    nx = 7;
    ny = 7;
    ex = linspace(min(pc1), max(pc1), nx + 1);
    ey = linspace(min(pc2), max(pc2), ny + 1);
    bx = discretize(pc1, ex);
    by = discretize(pc2, ey);

    rows = [];
    dom_pid = strings(n_samples, 1);
    dom_adrc = strings(n_samples, 1);

    for ix = 1:nx
        for iy = 1:ny
            idx = find(bx == ix & by == iy);
            if numel(idx) < (d + 2)
                continue;
            end

            Xi = Xz(idx, :);
            yp = y_pid(idx);
            ya = y_adrc(idx);

            bp = Xi \ zscore_cols(yp);
            ba = Xi \ zscore_cols(ya);

            [~, ip] = max(abs(bp));
            [~, ia] = max(abs(ba));

            rows = [rows; ix, iy, mean(pc1(idx)), mean(pc2(idx)), ip, ia, mean(abs(bp)), mean(abs(ba))]; %#ok<AGROW>

            dom_pid(idx) = string(factor_names{ip});
            dom_adrc(idx) = string(factor_names{ia});
        end
    end

    local_tbl = array2table(rows, 'VariableNames', ...
        {'cell_x', 'cell_y', 'pc1_center', 'pc2_center', 'dominant_idx_pid', 'dominant_idx_adrc', 'mean_abs_importance_pid', 'mean_abs_importance_adrc'});

    if ~isempty(local_tbl)
        idx_pid = round(local_tbl.dominant_idx_pid);
        idx_adrc = round(local_tbl.dominant_idx_adrc);
        idx_pid = max(1, min(numel(factor_names), idx_pid));
        idx_adrc = max(1, min(numel(factor_names), idx_adrc));

        local_tbl.dominant_factor_pid = string(factor_names(idx_pid))';
        local_tbl.dominant_factor_adrc = string(factor_names(idx_adrc))';
    else
        local_tbl.dominant_factor_pid = strings(0, 1);
        local_tbl.dominant_factor_adrc = strings(0, 1);
    end

    samples_tbl = table(pc1, pc2, ...
        X(:, 1), X(:, 2), X(:, 3), X(:, 4), X(:, 5), X(:, 6), X(:, 7), X(:, 8), ...
        y_pid, y_adrc, logical(s_pid), logical(s_adrc), dom_pid, dom_adrc, ...
        'VariableNames', {'pc1', 'pc2', 'aero_scale', 'loop_delay_ms', 'slew_rate_deg_s', 'mmi_inertia_scale', ...
        'cg_moment_coeff', 'thrust_profile_amp', 'gust_scale', 'dist_step_scale', 'delta_rmse_pid', 'delta_rmse_adrc', ...
        'stable_pid', 'stable_adrc', 'dominant_factor_pid', 'dominant_factor_adrc'});

    make_disturbance_atlas_figure(samples_tbl, global_tbl, factor_names, out_dir);
    pc1_bins_tbl = make_pca_stability_map_figure(samples_tbl, pc1_loadings_tbl, out_dir);

    atlas = struct();
    atlas.samples_tbl = samples_tbl;
    atlas.global_tbl = global_tbl;
    atlas.local_tbl = local_tbl;
    atlas.pc1_loadings_tbl = pc1_loadings_tbl;
    atlas.pc1_bins_tbl = pc1_bins_tbl;
end

function xz = zscore_cols(x)
    if isvector(x)
        mu = mean(x);
        sd = std(x);
        sd = max(sd, 1e-9);
        xz = (x - mu) / sd;
    else
        mu = mean(x, 1);
        sd = std(x, 0, 1);
        sd(sd < 1e-9) = 1;
        xz = (x - mu) ./ sd;
    end
end

function make_disturbance_atlas_figure(samples_tbl, global_tbl, factor_names, out_dir)
    [~, idx_pid] = ismember(samples_tbl.dominant_factor_pid, string(factor_names));
    [~, idx_adrc] = ismember(samples_tbl.dominant_factor_adrc, string(factor_names));

    f = figure('Position', [120, 80, 1300, 880], 'ToolBar', 'none');
    tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    scatter(samples_tbl.pc1, samples_tbl.pc2, 26, idx_pid, 'filled');
    grid on;
    xlabel('PC1 disturbance manifold');
    ylabel('PC2 disturbance manifold');
    title('Dominant Modeling Factor by Region (PID)');
    colormap(parula(numel(factor_names)));
    cb1 = colorbar;
    cb1.Ticks = 1:numel(factor_names);
    cb1.TickLabels = factor_names;

    nexttile;
    scatter(samples_tbl.pc1, samples_tbl.pc2, 26, idx_adrc, 'filled');
    grid on;
    xlabel('PC1 disturbance manifold');
    ylabel('PC2 disturbance manifold');
    title('Dominant Modeling Factor by Region (ADRC)');
    colormap(parula(numel(factor_names)));
    cb2 = colorbar;
    cb2.Ticks = 1:numel(factor_names);
    cb2.TickLabels = factor_names;

    nexttile;
    b = bar(categorical(global_tbl.factor), [global_tbl.importance_pid, global_tbl.importance_adrc], 'grouped');
    b(1).FaceColor = [0.85, 0.33, 0.10];
    b(2).FaceColor = [0.00, 0.45, 0.74];
    grid on;
    ylabel('Global standardized importance');
    title('Global Importance Across High-D Disturbance Space');
    legend('PID', 'ADRC', 'Location', 'best');

    nexttile;
    pid_stable_rate = mean(samples_tbl.stable_pid) * 100;
    adrc_stable_rate = mean(samples_tbl.stable_adrc) * 100;
    bar([pid_stable_rate, adrc_stable_rate], 'FaceColor', 'flat');
    set(gca, 'XTickLabel', {'PID', 'ADRC'});
    ylim([0, 100]);
    ylabel('Stable fraction (%)');
    title('Stability Coverage Over Disturbance Atlas');
    grid on;

    sgtitle('High-D Disturbance Importance Atlas');
    saveas(f, fullfile(out_dir, 'disturbance_importance_atlas.png'));
    close(f);
end

function pc1_bins_tbl = make_pca_stability_map_figure(samples_tbl, pc1_loadings_tbl, out_dir)
    % Build a single visual "proof" figure: PC1 stress axis vs stability/RMSE.
    pc1 = samples_tbl.pc1;
    rmse = samples_tbl.delta_rmse_adrc;
    stab = double(samples_tbl.stable_adrc);

    nb = 12;
    edges = linspace(min(pc1), max(pc1), nb + 1);
    c = 0.5 * (edges(1:end-1) + edges(2:end));
    sf = nan(nb, 1);
    rmean = nan(nb, 1);
    ncount = zeros(nb, 1);

    for i = 1:nb
        idx = pc1 >= edges(i) & pc1 < edges(i + 1);
        if i == nb
            idx = pc1 >= edges(i) & pc1 <= edges(i + 1);
        end
        ncount(i) = sum(idx);
        if any(idx)
            sf(i) = mean(stab(idx));
            rmean(i) = mean(rmse(idx));
        end
    end

    % Estimate a "cliff" as the first bin where stable fraction drops below 0.5.
    cliff_idx = find(sf < 0.5, 1, 'first');
    if isempty(cliff_idx)
        cliff_x = NaN;
    else
        cliff_x = c(cliff_idx);
    end

    f = figure('Position', [90, 90, 1380, 560], 'ToolBar', 'none');
    tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    scatter(pc1, rmse, 30, stab, 'filled'); hold on;
    colormap([0.85 0.2 0.2; 0.1 0.55 0.2]);
    cb = colorbar;
    cb.Ticks = [0, 1];
    cb.TickLabels = {'unstable', 'stable'};
    xlabel('PC1 (total system stress)');
    ylabel('ADRC \DeltaRMSE vs ideal (deg)');
    title('PCA Stability Map (samples)');
    grid on;
    if ~isnan(cliff_x)
        xline(cliff_x, '--k', 'Predictiveness cliff');
    end

    nexttile;
    yyaxis left;
    plot(c, sf, '-o', 'LineWidth', 1.8, 'Color', [0.1 0.55 0.2]); hold on;
    yline(0.5, ':k', '50% stable');
    ylabel('Stable fraction (ADRC)');
    yyaxis right;
    plot(c, rmean, '-s', 'LineWidth', 1.6, 'Color', [0.85 0.33 0.10]);
    ylabel('Mean \DeltaRMSE (deg)');
    xlabel('PC1 (total system stress)');
    title('Boundary Trend Along PC1');
    grid on;
    if ~isnan(cliff_x)
        xline(cliff_x, '--k');
    end

    nexttile;
    top = pc1_loadings_tbl(1:min(6, height(pc1_loadings_tbl)), :);
    b = barh(categorical(top.factor), top.abs_pc1_loading);
    b.FaceColor = [0.00 0.45 0.74];
    xlabel('|PC1 loading|');
    title('Dominant PC1 Contributors');
    grid on;

    sgtitle('Single-Figure Proof: Stress-Axis Stability Boundary');
    saveas(f, fullfile(out_dir, 'pca_stability_map.png'));
    close(f);

    pc1_bins_tbl = table(c(:), sf(:), rmean(:), ncount(:), ...
        'VariableNames', {'pc1_center', 'stable_fraction_adrc', 'mean_delta_rmse_adrc', 'sample_count'});
    pc1_bins_tbl.predictiveness_cliff_pc1 = repmat(cliff_x, height(pc1_bins_tbl), 1);
end

function explicit = build_explicit_axis_sweeps(cfg_real, pid_kp, pid_kd, profile, out_dir)
    cfg_s = cfg_real;
    cfg_s.dt = 0.004;
    cfg_s.n = floor(cfg_s.t_final / cfg_s.dt) + 1;
    cfg_s.t = (0:cfg_s.n - 1)' * cfg_s.dt;
    cfg_s.pid_kp = pid_kp;
    cfg_s.pid_kd = pid_kd;

    factor_defs = {
        'loop_delay_ms',      linspace(0, 60, 16);
        'slew_rate_deg_s',    linspace(30, 120, 16);
        'aero_scale',         linspace(0.7, 1.6, 16);
        'mmi_inertia_scale',  linspace(0.75, 1.35, 16);
        'cg_moment_coeff',    linspace(0.0, 0.8, 16);
        'thrust_profile_amp', linspace(0.0, 0.25, 16)};

    [base_pid, base_adrc] = evaluate_profile(cfg_s, profile);

    rows = [];
    factor_col = strings(0, 1);

    for i = 1:size(factor_defs, 1)
        fname = factor_defs{i, 1};
        gridv = factor_defs{i, 2};

        for k = 1:numel(gridv)
            p = profile;
            p.(fname) = gridv(k);

            [met_pid, met_adrc] = evaluate_profile(cfg_s, p);

            d_pid = met_pid.theta_rmse_deg - base_pid.theta_rmse_deg;
            d_adrc = met_adrc.theta_rmse_deg - base_adrc.theta_rmse_deg;

            rows = [rows; gridv(k), d_pid, d_adrc, met_pid.theta_rmse_deg, met_adrc.theta_rmse_deg, ...
                          met_pid.control_thd_pct, met_adrc.control_thd_pct, met_pid.is_stable, met_adrc.is_stable]; %#ok<AGROW>
            factor_col(end + 1, 1) = string(fname); %#ok<AGROW>
        end
    end

    sweep_tbl = table(factor_col, rows(:, 1), rows(:, 2), rows(:, 3), rows(:, 4), rows(:, 5), rows(:, 6), rows(:, 7), logical(rows(:, 8)), logical(rows(:, 9)), ...
        'VariableNames', {'factor', 'factor_value', 'signed_delta_rmse_pid', 'signed_delta_rmse_adrc', 'rmse_pid', 'rmse_adrc', 'thd_pid', 'thd_adrc', 'stable_pid', 'stable_adrc'});

    % Negative signed delta means RMSE improved relative to profile baseline.
    sweep_tbl.abs_delta_rmse_pid = abs(sweep_tbl.signed_delta_rmse_pid);
    sweep_tbl.abs_delta_rmse_adrc = abs(sweep_tbl.signed_delta_rmse_adrc);

    % Ranking for this specific rocket profile.
    fac = unique(sweep_tbl.factor, 'stable');
    n = numel(fac);
    imp_pid = zeros(n, 1);
    imp_adrc = zeros(n, 1);
    for i = 1:n
        idx = sweep_tbl.factor == fac(i);
        imp_pid(i) = mean(sweep_tbl.abs_delta_rmse_pid(idx));
        imp_adrc(i) = mean(sweep_tbl.abs_delta_rmse_adrc(idx));
    end

    rank_tbl = table(fac, imp_pid, imp_adrc, 'VariableNames', {'factor', 'importance_pid', 'importance_adrc'});
    rank_tbl = sortrows(rank_tbl, 'importance_adrc', 'descend');
    rank_tbl.rank_adrc = (1:height(rank_tbl))';
    rank_tbl = movevars(rank_tbl, 'rank_adrc', 'Before', 'factor');

    make_explicit_sweep_figure(sweep_tbl, rank_tbl, out_dir);

    explicit = struct();
    explicit.sweep_tbl = sweep_tbl;
    explicit.rank_tbl = rank_tbl;
end

function [met_pid, met_adrc] = evaluate_profile(cfg_s, profile)
    opts = default_run_opts(cfg_s);
    opts.k_theta_scale = profile.aero_scale;
    opts.k_q_scale = max(0.4, 1.25 - 0.35 * profile.aero_scale);
    opts.inertia_scale = profile.mmi_inertia_scale;
    opts.cg_moment_coeff = profile.cg_moment_coeff;
    opts.thrust_profile_amp = profile.thrust_profile_amp;
    opts.gust_amp_scale = profile.gust_scale;
    opts.dist_step_amp_scale = profile.dist_step_scale;

    delay_s = profile.loop_delay_ms / 1000;
    slew_deg_s = profile.slew_rate_deg_s;
    mismatch = struct('Iyy_scale', profile.mmi_inertia_scale);

    run_pid = run_flight('PID', cfg_s, mismatch, delay_s, slew_deg_s, opts);
    run_adrc = run_flight('ADRC', cfg_s, mismatch, delay_s, slew_deg_s, opts);

    met_pid = compute_metrics(run_pid, cfg_s);
    met_adrc = compute_metrics(run_adrc, cfg_s);
end

function make_explicit_sweep_figure(sweep_tbl, rank_tbl, out_dir)
    factors = cellstr(rank_tbl.factor);
    n = numel(factors);

    f = figure('Position', [90, 70, 1400, 920], 'ToolBar', 'none');
    tiledlayout(3, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

    for i = 1:min(n, 6)
        nexttile;
        idx = sweep_tbl.factor == string(factors{i});
        T = sortrows(sweep_tbl(idx, :), 'factor_value');
        plot(T.factor_value, T.signed_delta_rmse_pid, '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.6); hold on;
        plot(T.factor_value, T.signed_delta_rmse_adrc, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.8);
        yline(0, ':k');
        grid on;
        xlabel(strrep(factors{i}, '_', ' '));
        ylabel('Signed \Delta RMSE (deg)');
        title(sprintf('Sensitivity: %s', strrep(factors{i}, '_', ' ')));
        if i == 1
            legend('PID', 'ADRC', 'Location', 'best');
        end
    end

    nexttile([1 3]);
    bar(categorical(rank_tbl.factor), [rank_tbl.importance_pid, rank_tbl.importance_adrc], 'grouped');
    grid on;
    ylabel('Mean |\Delta RMSE| over sweep');
    title('Rocket-Specific Factor Importance Ranking');
    legend('PID', 'ADRC', 'Location', 'best');

    sgtitle('Explicit Axis Sensitivity Sweeps (No PCA Axes)');
    saveas(f, fullfile(out_dir, 'explicit_factor_sweeps.png'));
    close(f);
end

function key_tbl = make_key_findings_table(side_tbl, conv_tbl, rob_tbl, rank_tbl)
    pid_perfect = side_tbl(strcmp(side_tbl.controller, 'PID') & strcmp(side_tbl.condition, 'perfect'), :);
    adrc_perfect = side_tbl(strcmp(side_tbl.controller, 'ADRC') & strcmp(side_tbl.condition, 'perfect'), :);
    pid_real = side_tbl(strcmp(side_tbl.controller, 'PID') & strcmp(side_tbl.condition, 'realistic'), :);
    adrc_real = side_tbl(strcmp(side_tbl.controller, 'ADRC') & strcmp(side_tbl.condition, 'realistic'), :);

    pid_stable_pts = sum(rob_tbl.pid_stable == 1 | rob_tbl.pid_stable == true);
    adrc_stable_pts = sum(rob_tbl.adrc_stable == 1 | rob_tbl.adrc_stable == true);

    key_tbl = table();
    key_tbl.metric = string({
        'PID theta RMSE (perfect)';
        'ADRC theta RMSE (perfect)';
        'PID theta RMSE (realistic)';
        'ADRC theta RMSE (realistic)';
        'ADRC/PID RMSE ratio (realistic)';
        'LESO settle time (s)';
        'PID stable sweep points';
        'ADRC stable sweep points';
        'Top factor for ADRC (explicit sweep)'});

    key_tbl.value = string({
        sprintf('%.3f', pid_perfect.theta_rmse_deg);
        sprintf('%.3f', adrc_perfect.theta_rmse_deg);
        sprintf('%.3f', pid_real.theta_rmse_deg);
        sprintf('%.3f', adrc_real.theta_rmse_deg);
        sprintf('%.2f x', adrc_real.theta_rmse_deg / max(pid_real.theta_rmse_deg, 1e-9));
        sprintf('%.3f', conv_tbl.eso_settle_time_s(1));
        sprintf('%d', pid_stable_pts);
        sprintf('%d', adrc_stable_pts);
        char(rank_tbl.factor(1))});
end

function make_intuitive_summary_figure(side_tbl, conv_tbl, rob_tbl, rank_tbl, out_dir)
    pid_perfect = side_tbl(strcmp(side_tbl.controller, 'PID') & strcmp(side_tbl.condition, 'perfect'), :);
    adrc_perfect = side_tbl(strcmp(side_tbl.controller, 'ADRC') & strcmp(side_tbl.condition, 'perfect'), :);
    pid_real = side_tbl(strcmp(side_tbl.controller, 'PID') & strcmp(side_tbl.condition, 'realistic'), :);
    adrc_real = side_tbl(strcmp(side_tbl.controller, 'ADRC') & strcmp(side_tbl.condition, 'realistic'), :);

    pid_stable_pts = sum(rob_tbl.pid_stable == 1 | rob_tbl.pid_stable == true);
    adrc_stable_pts = sum(rob_tbl.adrc_stable == 1 | rob_tbl.adrc_stable == true);

    f = figure('Position', [120, 90, 1250, 780], 'ToolBar', 'none');
    tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    Y = [pid_perfect.theta_rmse_deg, adrc_perfect.theta_rmse_deg; ...
         pid_real.theta_rmse_deg, adrc_real.theta_rmse_deg];
    b = bar(Y, 'grouped');
    b(1).FaceColor = [0.85 0.33 0.10];
    b(2).FaceColor = [0.00 0.45 0.74];
    set(gca, 'XTickLabel', {'Perfect', 'Realistic'});
    ylabel('Theta RMSE (deg)');
    title('Controller Performance by Condition');
    legend('PID', 'ADRC', 'Location', 'northwest');
    grid on;

    nexttile;
    bar([pid_stable_pts, adrc_stable_pts], 'FaceColor', 'flat');
    set(gca, 'XTickLabel', {'PID', 'ADRC'});
    ylabel('Stable uncertainty points');
    title('Robustness Sweep Stability Count');
    grid on;

    nexttile;
    top = rank_tbl(1:min(6, height(rank_tbl)), :);
    bh = barh(categorical(top.factor), top.importance_adrc);
    bh.FaceColor = [0.00 0.45 0.74];
    xlabel('Mean |Signed \Delta RMSE| (deg)');
    title('What Matters Most to Model (ADRC, this rocket)');
    grid on;

    nexttile;
    axis off;
    txt = {
        'Main takeaway', ...
        sprintf('Perfect sim: PID %.2f deg, ADRC %.2f deg (both stable).', pid_perfect.theta_rmse_deg, adrc_perfect.theta_rmse_deg), ...
        sprintf('Realistic sim: PID %.2f deg (unstable), ADRC %.2f deg (stable).', pid_real.theta_rmse_deg, adrc_real.theta_rmse_deg), ...
        sprintf('LESO settle time after disturbance step: %.3f s.', conv_tbl.eso_settle_time_s(1)), ...
        'Note: negative signed \DeltaRMSE in sweeps means that factor value improved error', ...
        'relative to the baseline profile; RMSE itself is always non-negative.'};
    text(0.00, 0.95, txt, 'FontSize', 11, 'VerticalAlignment', 'top');

    sgtitle('Intuitive Findings Summary');
    saveas(f, fullfile(out_dir, 'intuitive_findings_summary.png'));
    close(f);
end

function gemini = run_gemini_ablation_matrix(cfg_real, pid_kp, pid_kd, out_dir)
    % Gemini-style Phase II: 4 controlled ablation variables with Monte Carlo.
    % Runs without measured data using synthetic priors already in cfg_real.

    cfg_mc = cfg_real;
    cfg_mc.dt = 0.004;
    cfg_mc.n = floor(cfg_mc.t_final / cfg_mc.dt) + 1;
    cfg_mc.t = (0:cfg_mc.n - 1)' * cfg_mc.dt;
    cfg_mc.pid_kp = pid_kp;
    cfg_mc.pid_kd = pid_kd;

    controllers = {'PID', 'ADRC'};
    ablations = {'A_inertia_error', 'B_aero_error', 'C_latency_error', 'D_thrust_misalignment', 'E_observer_bandwidth'};
    levels = 0:0.1:1.0;
    n_mc = 120; % lightweight Monte Carlo for fast iteration

    rows = [];
    c_names = strings(0, 1);
    a_names = strings(0, 1);

    for c = 1:numel(controllers)
        for a = 1:numel(ablations)
            for li = 1:numel(levels)
                lvl = levels(li);

                stable_count = 0;
                rmse_acc = 0;
                thd_acc = 0;
                omega_acc = 0;
                slew_sat_acc = 0;
                slew_sat_boost_acc = 0;

                for r = 1:n_mc
                    opts = default_run_opts(cfg_mc);
                    mismatch = struct('Iyy_scale', 1.0);
                    delay_s = cfg_mc.actuator_delay_nom_s;
                    slew_deg_s = cfg_mc.actuator_rate_nom_deg_s;

                    % Small nuisance randomization in all runs.
                    opts.gust_amp_scale = 0.8 + 0.4 * rand();
                    opts.dist_step_amp_scale = 0.8 + 0.4 * rand();
                    opts.thrust_profile_amp = 0.02 * rand();

                    switch ablations{a}
                        case 'A_inertia_error'
                            sgn = sign(randn());
                            sgn = max(sgn, 1e-9) / max(abs(sgn), 1e-9);
                            mismatch.Iyy_scale = 1 + sgn * 0.20 * lvl;
                            opts.inertia_scale = 1 + sgn * 0.20 * lvl;
                        case 'B_aero_error'
                            sgn = sign(randn());
                            sgn = max(sgn, 1e-9) / max(abs(sgn), 1e-9);
                            opts.k_theta_scale = 1 + sgn * 0.35 * lvl;
                            opts.k_q_scale = max(0.5, 1 - sgn * 0.20 * lvl);
                            opts.cg_moment_coeff = 0.6 * lvl;
                        case 'C_latency_error'
                            delay_s = cfg_mc.actuator_delay_nom_s + 0.020 * lvl; % +20ms max
                        case 'D_thrust_misalignment'
                            sgn = sign(randn());
                            sgn = max(sgn, 1e-9) / max(abs(sgn), 1e-9);
                            opts.thrust_misalignment_deg = sgn * 2.0 * lvl; % +/-2deg max
                        case 'E_observer_bandwidth'
                            % Sweep LESO bandwidth around nominal with noise-linked stress.
                            omega_scale = 0.50 + 1.50 * lvl;
                            opts.omega_o_override = cfg_mc.omega_o * omega_scale;
                            opts.sensor_noise_scale = 1.0 + 2.5 * lvl;
                        otherwise
                            error('Unknown ablation type.');
                    end

                    run = run_flight(controllers{c}, cfg_mc, mismatch, delay_s, slew_deg_s, opts);
                    met = compute_metrics(run, cfg_mc);

                    stable_count = stable_count + double(met.is_stable);
                    rmse_acc = rmse_acc + met.theta_rmse_deg;
                    thd_acc = thd_acc + met.control_thd_pct;
                    slew_sat_acc = slew_sat_acc + run.slew_sat_frac;
                    slew_sat_boost_acc = slew_sat_boost_acc + run.slew_sat_frac_boost;
                    if isnan(opts.omega_o_override)
                        omega_acc = omega_acc + cfg_mc.omega_o;
                    else
                        omega_acc = omega_acc + opts.omega_o_override;
                    end
                end

                stable_frac = stable_count / n_mc;
                mean_rmse = rmse_acc / n_mc;
                mean_thd = thd_acc / n_mc;
                mean_omega = omega_acc / n_mc;
                mean_slew_sat = slew_sat_acc / n_mc;
                mean_slew_sat_boost = slew_sat_boost_acc / n_mc;

                rows = [rows; lvl, mean_rmse, stable_frac, mean_thd, mean_omega, mean_slew_sat, mean_slew_sat_boost]; %#ok<AGROW>
                c_names(end + 1, 1) = string(controllers{c}); %#ok<AGROW>
                a_names(end + 1, 1) = string(ablations{a}); %#ok<AGROW>
            end
        end
    end

    matrix_tbl = table(c_names, a_names, rows(:, 1), rows(:, 2), rows(:, 3), rows(:, 4), rows(:, 5), rows(:, 6), rows(:, 7), ...
        'VariableNames', {'controller', 'ablation', 'uncertainty_level', 'mean_theta_rmse_deg', 'stable_fraction', 'mean_control_thd_pct', 'mean_observer_omega_o', 'mean_slew_sat_frac', 'mean_slew_sat_frac_boost'});

    % Stability boundary as highest level with >=80% stable fraction.
    b_rows = [];
    b_ctrl = strings(0, 1);
    b_abl = strings(0, 1);
    for c = 1:numel(controllers)
        for a = 1:numel(ablations)
            idx = matrix_tbl.controller == string(controllers{c}) & matrix_tbl.ablation == string(ablations{a});
            T = sortrows(matrix_tbl(idx, :), 'uncertainty_level');
            ok = T.uncertainty_level(T.stable_fraction >= 0.80);
            if isempty(ok)
                boundary = NaN;
            else
                boundary = max(ok);
            end
            b_rows = [b_rows; boundary]; %#ok<AGROW>
            b_ctrl(end + 1, 1) = string(controllers{c}); %#ok<AGROW>
            b_abl(end + 1, 1) = string(ablations{a}); %#ok<AGROW>
        end
    end

    boundary_tbl = table(b_ctrl, b_abl, b_rows, ...
        'VariableNames', {'controller', 'ablation', 'stability_boundary_level'});

    make_gemini_gap_heatmap(matrix_tbl, out_dir);

    gemini = struct();
    gemini.matrix_tbl = matrix_tbl;
    gemini.boundary_tbl = boundary_tbl;
end

function make_gemini_gap_heatmap(matrix_tbl, out_dir)
    abls = unique(matrix_tbl.ablation, 'stable');
    lvls = unique(matrix_tbl.uncertainty_level);

    H_pid = zeros(numel(abls), numel(lvls));
    H_adrc = zeros(numel(abls), numel(lvls));

    for i = 1:numel(abls)
        for j = 1:numel(lvls)
            idxp = matrix_tbl.controller == "PID" & matrix_tbl.ablation == abls(i) & abs(matrix_tbl.uncertainty_level - lvls(j)) < 1e-9;
            idxa = matrix_tbl.controller == "ADRC" & matrix_tbl.ablation == abls(i) & abs(matrix_tbl.uncertainty_level - lvls(j)) < 1e-9;

            if any(idxp)
                H_pid(i, j) = matrix_tbl.stable_fraction(find(idxp, 1, 'first'));
            end
            if any(idxa)
                H_adrc(i, j) = matrix_tbl.stable_fraction(find(idxa, 1, 'first'));
            end
        end
    end

    f = figure('Position', [130, 120, 1250, 520], 'ToolBar', 'none');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    imagesc(lvls, 1:numel(abls), H_pid);
    caxis([0 1]);
    colormap('turbo');
    colorbar;
    yticks(1:numel(abls));
    yticklabels(cellstr(abls));
    xlabel('Uncertainty level (0..1)');
    title('PID Stability Fraction Heatmap');

    nexttile;
    imagesc(lvls, 1:numel(abls), H_adrc);
    caxis([0 1]);
    colormap('turbo');
    colorbar;
    yticks(1:numel(abls));
    yticklabels(cellstr(abls));
    xlabel('Uncertainty level (0..1)');
    title('ADRC Stability Fraction Heatmap');

    sgtitle('Gemini-style Sim-to-Real Gap Heatmap (Monte Carlo)');
    saveas(f, fullfile(out_dir, 'gemini_gap_heatmap.png'));
    close(f);
end

function make_observer_bandwidth_figure(matrix_tbl, omega_nominal, hw_bw, out_dir)
    idx = matrix_tbl.controller == "ADRC" & matrix_tbl.ablation == "E_observer_bandwidth";
    T = sortrows(matrix_tbl(idx, :), 'mean_observer_omega_o');
    if isempty(T)
        return;
    end

    score = normalize01(T.mean_theta_rmse_deg) + 0.35 * normalize01(T.mean_control_thd_pct);
    [~, i_best] = min(score);

    % Hardware bound shaded region (alpha=3 to alpha=5 span)
    hw_lo = min(hw_bw.omega_bounds);  % alpha=5, most conservative
    hw_hi = max(hw_bw.omega_bounds);  % alpha=3, most permissive
    hw_mid = hw_bw.omega_bounds(hw_bw.alpha_vals == 4);  % alpha=4 centerline

    f = figure('Position', [130, 120, 1280, 560], 'ToolBar', 'none');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    yyaxis left;
    ylims_rmse = [min(T.mean_theta_rmse_deg)*0.98, max(T.mean_theta_rmse_deg)*1.02];
    patch([hw_lo hw_hi hw_hi hw_lo], [ylims_rmse(1) ylims_rmse(1) ylims_rmse(2) ylims_rmse(2)], ...
          [1.0 0.85 0.3], 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'DisplayName', 'HW bound (\alpha=3-5)');
    hold on;
    plot(T.mean_observer_omega_o, T.mean_theta_rmse_deg, '-o', 'LineWidth', 1.8, 'Color', [0.00 0.45 0.74], 'DisplayName', 'RMSE');
    xline(hw_mid, '--', ['\omega_{hw} (\alpha=4)=' num2str(hw_mid,'%.1f')], 'Color', [0.85 0.65 0.0], 'LineWidth', 1.4);
    ylim(ylims_rmse);
    ylabel('Mean theta RMSE (deg)');
    yyaxis right;
    plot(T.mean_observer_omega_o, T.mean_control_thd_pct, '-s', 'LineWidth', 1.6, 'Color', [0.85 0.33 0.10], 'DisplayName', 'Control THD');
    ylabel('Mean control THD (%)');
    xline(omega_nominal, ':k', 'Nominal');
    xline(T.mean_observer_omega_o(i_best), '--', 'Golden');
    xlabel('\omega_o (rad/s)');
    title('Observer Bandwidth Trade-off (ADRC)');
    grid on;
    legend('HW bound', 'RMSE', 'Control THD', 'Location', 'best');

    nexttile;
    ylims_stab = [0 1];
    patch([hw_lo hw_hi hw_hi hw_lo], [ylims_stab(1) ylims_stab(1) ylims_stab(2) ylims_stab(2)], ...
          [1.0 0.85 0.3], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    hold on;
    plot(T.mean_observer_omega_o, T.stable_fraction, '-o', 'LineWidth', 1.8, 'Color', [0.10 0.60 0.20]);
    yline(0.80, ':k', '80% stable');
    xline(omega_nominal, ':k', 'Nominal');
    xline(hw_mid, '--', ['\omega_{hw}=' num2str(hw_mid,'%.1f')], 'Color', [0.85 0.65 0.0], 'LineWidth', 1.4);
    xline(T.mean_observer_omega_o(i_best), '--', 'Golden');
    ylim([0 1]);
    xlabel('\omega_o (rad/s)');
    ylabel('Stable fraction');
    title(['Bandwidth vs Stability  |  HW bound: ' num2str(hw_lo,'%.1f') char(8211) num2str(hw_hi,'%.1f') ' rad/s']);
    grid on;

    sgtitle(['LESO Bandwidth Sensitivity  |  S_{max}=' num2str(round(rad2deg(hw_bw.S_max_rad_s))) ...
        char(176) '/s,  \sigma_{\theta}=' num2str(rad2deg(hw_bw.sigma_theta),'%.3f') char(176)]);
    saveas(f, fullfile(out_dir, 'observer_bandwidth_tradeoff.png'));
    close(f);
end

function hw_bw = compute_hardware_bandwidth_bound(cfg)
    % Hardware-constrained LESO bandwidth ceiling.
    % Derivation (slew-noise): omega_o^2 noise-driven command rate ~= omega_o^2 * sigma_theta
    % must not exceed S_max.  Simplified bound: omega_o <= sqrt(S_max / (alpha * sigma_theta)).
    S_max_rad_s = deg2rad(cfg.actuator_rate_nom_deg_s);  % rad/s
    sigma_theta = deg2rad(cfg.theta_noise_deg);           % rad (1-sigma IMU noise)
    alpha_vals  = [3, 4, 5];                              % conservative to permissive
    omega_bounds = sqrt(S_max_rad_s ./ (alpha_vals * sigma_theta));
    hw_bw.S_max_rad_s   = S_max_rad_s;
    hw_bw.sigma_theta   = sigma_theta;
    hw_bw.alpha_vals    = alpha_vals;
    hw_bw.omega_bounds  = omega_bounds;
    hw_bw.omega_nominal = cfg.omega_o;
    fprintf('--- Hardware Bandwidth Bound ---\n');
    fprintf('  S_max = %.3f rad/s (%.0f deg/s),  sigma_theta = %.5f rad (%.3f deg)\n', ...
        S_max_rad_s, cfg.actuator_rate_nom_deg_s, sigma_theta, cfg.theta_noise_deg);
    for k = 1:numel(alpha_vals)
        ratio = cfg.omega_o / omega_bounds(k);
        fprintf('  alpha=%d -> omega_o_max = %.2f rad/s  (nominal omega_o=%.0f is %.1fx the bound)\n', ...
            alpha_vals(k), omega_bounds(k), cfg.omega_o, ratio);
    end
    fprintf('  Finding: nominal omega_o sits ABOVE hardware ceiling -> slew saturation expected.\n');
end

function make_spaghetti_plot(cfg_real, pid_kp, pid_kd, out_dir)
    % Overlay many theta(t) trajectories to visualize robustness spread.
    cfg_sp = cfg_real;
    cfg_sp.dt = 0.004;
    cfg_sp.n = floor(cfg_sp.t_final / cfg_sp.dt) + 1;
    cfg_sp.t = (0:cfg_sp.n - 1)' * cfg_sp.dt;
    cfg_sp.pid_kp = pid_kp;
    cfg_sp.pid_kd = pid_kd;

    n_runs = 120;

    theta_pid = zeros(cfg_sp.n, n_runs);
    theta_adrc = zeros(cfg_sp.n, n_runs);

    for i = 1:n_runs
        opts = default_run_opts(cfg_sp);
        mismatch = struct('Iyy_scale', 1.0);

        % Randomized gap bundle: inertia/aero/delay/thrust misalignment + noise/gust
        sgn1 = sign(randn());
        if sgn1 == 0, sgn1 = 1; end
        sgn2 = sign(randn());
        if sgn2 == 0, sgn2 = 1; end

        mismatch.Iyy_scale = 1 + sgn1 * 0.20 * rand();
        opts.inertia_scale = 1 + sgn1 * 0.20 * rand();
        opts.k_theta_scale = 1 + sgn2 * 0.35 * rand();
        opts.k_q_scale = max(0.5, 1 - sgn2 * 0.20 * rand());
        opts.thrust_misalignment_deg = sgn2 * 2.0 * rand();
        opts.gust_amp_scale = 0.7 + 0.8 * rand();
        opts.dist_step_amp_scale = 0.7 + 0.8 * rand();
        opts.thrust_profile_amp = 0.03 + 0.12 * rand();

        delay_s = cfg_sp.actuator_delay_nom_s + 0.020 * rand();
        slew_deg_s = 30 + 90 * rand();

        r_pid = run_flight('PID', cfg_sp, mismatch, delay_s, slew_deg_s, opts);
        r_adrc = run_flight('ADRC', cfg_sp, mismatch, delay_s, slew_deg_s, opts);

        theta_pid(:, i) = rad2deg(r_pid.theta);
        theta_adrc(:, i) = rad2deg(r_adrc.theta);
    end

    f = figure('Position', [120, 120, 1300, 680], 'ToolBar', 'none');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(cfg_sp.t, theta_pid, 'Color', [0.85 0.33 0.10 0.18], 'LineWidth', 0.8); hold on;
    plot(cfg_sp.t, mean(theta_pid, 2), 'Color', [0.55 0.15 0.05], 'LineWidth', 2.3);
    yline(cfg_sp.theta_ref_deg, ':k');
    xline(cfg_sp.dist_step_time, ':', 'Dist step');
    xline(cfg_sp.aero_shift_time, ':', 'Aero shift');
    grid on;
    xlabel('Time (s)');
    ylabel('Pitch angle \theta (deg)');
    title(sprintf('PID Spaghetti (N=%d)', n_runs));

    nexttile;
    plot(cfg_sp.t, theta_adrc, 'Color', [0.00 0.45 0.74 0.18], 'LineWidth', 0.8); hold on;
    plot(cfg_sp.t, mean(theta_adrc, 2), 'Color', [0.00 0.20 0.45], 'LineWidth', 2.3);
    yline(cfg_sp.theta_ref_deg, ':k');
    xline(cfg_sp.dist_step_time, ':', 'Dist step');
    xline(cfg_sp.aero_shift_time, ':', 'Aero shift');
    grid on;
    xlabel('Time (s)');
    ylabel('Pitch angle \theta (deg)');
    title(sprintf('ADRC Spaghetti (N=%d)', n_runs));

    sgtitle('Monte Carlo Spaghetti Plot: Pitch Trajectory Dispersion');
    saveas(f, fullfile(out_dir, 'spaghetti_pitch_theta.png'));
    close(f);
end

function make_z3_forensic_figure(adrc_real, cfg, out_dir)
    t = adrc_real.t(:);
    z3 = adrc_real.z3(:);
    ftrue = adrc_real.f_total_true(:);
    resid = z3 - ftrue;

    f = figure('Position', [120, 110, 1200, 760], 'ToolBar', 'none');
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(t, z3, 'LineWidth', 1.7, 'Color', [0.00 0.45 0.74]); hold on;
    plot(t, ftrue, '--', 'LineWidth', 1.4, 'Color', [0.85 0.33 0.10]);
    xline(cfg.burn_time, '--k', 'Burnout');
    xline(cfg.aero_shift_time, ':k', 'Aero shift');
    ylabel('Disturbance (rad/s^2)');
    title('ADRC Disturbance Estimate Forensics: z_3 vs f_{true}');
    legend('z_3 estimate', 'f_{true}', 'Location', 'best');
    grid on;

    nexttile;
    plot(t, resid, 'LineWidth', 1.5, 'Color', [0.49 0.18 0.56]); hold on;
    yline(0, ':k');
    xline(cfg.burn_time, '--k', 'Burnout');
    ylabel('Residual z_3-f_{true}');
    title('Residual Trace (Forensic Signature Candidate)');
    grid on;

    nexttile;
    yyaxis left;
    plot(t, adrc_real.thrust_truth, 'LineWidth', 1.5, 'Color', [0.10 0.60 0.20]);
    ylabel('Thrust (N)');
    yyaxis right;
    plot(t, adrc_real.cp_shift_gain, '--', 'LineWidth', 1.3, 'Color', [0.85 0.33 0.10]); hold on;
    plot(t, adrc_real.cg_shift_gain, '-.', 'LineWidth', 1.3, 'Color', [0.00 0.45 0.74]);
    ylabel('Regime gain');
    xline(cfg.burn_time, '--k', 'Burnout');
    xlabel('Time (s)');
    title('Burnout Regime Shift Inputs (Thrust Cutoff + C_p/C_g Step Gains)');
    legend('Thrust', 'C_p gain', 'C_g gain', 'Location', 'best');
    grid on;

    sgtitle('Regime-Dependent Sim-to-Real Forensics');
    saveas(f, fullfile(out_dir, 'adrc_z3_forensic_signature.png'));
    close(f);
end

function make_phase_portrait_figure(pid_real, adrc_real, cfg, out_dir)
    idx_b_pid = pid_real.t <= cfg.burn_time;
    idx_c_pid = pid_real.t > cfg.burn_time;
    idx_b_adrc = adrc_real.t <= cfg.burn_time;
    idx_c_adrc = adrc_real.t > cfg.burn_time;
    i_burn_pid = find(idx_b_pid, 1, 'last');
    i_burn_adrc = find(idx_b_adrc, 1, 'last');

    f = figure('Position', [130, 120, 1180, 520], 'ToolBar', 'none');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(rad2deg(pid_real.theta(idx_b_pid)), rad2deg(pid_real.q(idx_b_pid)), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8); hold on;
    plot(rad2deg(pid_real.theta(idx_c_pid)), rad2deg(pid_real.q(idx_c_pid)), 'Color', [0.60 0.10 0.05], 'LineWidth', 1.8);
    scatter(rad2deg(pid_real.theta(i_burn_pid)), rad2deg(pid_real.q(i_burn_pid)), 40, 'k', 'filled');
    xlabel('Theta (deg)');
    ylabel('q (deg/s)');
    title('PID Phase Portrait (Boost \rightarrow Coast Transition)');
    legend('Boost branch', 'Coast branch', 'Burnout point', 'Location', 'best');
    grid on;

    nexttile;
    plot(rad2deg(adrc_real.theta(idx_b_adrc)), rad2deg(adrc_real.q(idx_b_adrc)), 'Color', [0.00 0.45 0.74], 'LineWidth', 1.8); hold on;
    plot(rad2deg(adrc_real.theta(idx_c_adrc)), rad2deg(adrc_real.q(idx_c_adrc)), 'Color', [0.00 0.20 0.45], 'LineWidth', 1.8);
    scatter(rad2deg(adrc_real.theta(i_burn_adrc)), rad2deg(adrc_real.q(i_burn_adrc)), 40, 'k', 'filled');
    xlabel('Theta (deg)');
    ylabel('q (deg/s)');
    title('ADRC Phase Portrait (Boost \rightarrow Coast Transition)');
    legend('Boost attractor approach', 'Coast attractor approach', 'Burnout point', 'Location', 'best');
    grid on;

    sgtitle('Regime Transition Phase Portrait: Boost to Coast Attractor Shift');
    saveas(f, fullfile(out_dir, 'phase_portrait_realistic.png'));
    close(f);
end

function make_control_psd_figure(pid_real, adrc_real, out_dir)
    [f_pid, p_pid] = estimate_psd_fft(rad2deg(pid_real.u_cmd), pid_real.t);
    [f_adrc, p_adrc] = estimate_psd_fft(rad2deg(adrc_real.u_cmd), adrc_real.t);

    f = figure('Position', [140, 120, 980, 520], 'ToolBar', 'none');
    semilogy(f_pid, p_pid, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); hold on;
    semilogy(f_adrc, p_adrc, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.7);
    xlim([0, 20]);
    xlabel('Frequency (Hz)');
    ylabel('PSD (arb units)');
    title('Control Command Spectrum (delta command)');
    legend('PID', 'ADRC', 'Location', 'best');
    grid on;
    saveas(f, fullfile(out_dir, 'control_command_psd.png'));
    close(f);
end

function tbl = make_recovery_envelope(cfg_real, pid_kp, pid_kd, mismatch_real, real_delay_s, real_rate_deg_s, out_dir)
    theta0_grid = 2:2:24;
    opts = default_run_opts(cfg_real);

    pid_ok = false(numel(theta0_grid), 1);
    adrc_ok = false(numel(theta0_grid), 1);

    for i = 1:numel(theta0_grid)
        cfg_i = cfg_real;
        cfg_i.theta0_deg = theta0_grid(i);
        cfg_i.pid_kp = pid_kp;
        cfg_i.pid_kd = pid_kd;

        r_pid = run_flight('PID', cfg_i, mismatch_real, real_delay_s, real_rate_deg_s, opts);
        r_adrc = run_flight('ADRC', cfg_i, mismatch_real, real_delay_s, real_rate_deg_s, opts);

        pid_ok(i) = compute_metrics(r_pid, cfg_i).is_stable;
        adrc_ok(i) = compute_metrics(r_adrc, cfg_i).is_stable;
    end

    tbl = table(theta0_grid(:), pid_ok, adrc_ok, 'VariableNames', {'theta0_deg', 'pid_stable', 'adrc_stable'});

    f = figure('Position', [150, 120, 980, 520], 'ToolBar', 'none');
    stairs(theta0_grid, double(pid_ok), '-o', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.6); hold on;
    stairs(theta0_grid, double(adrc_ok), '-o', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.8);
    ylim([-0.05, 1.05]);
    yticks([0 1]);
    yticklabels({'Unstable', 'Stable'});
    xlabel('Initial tilt theta_0 (deg)');
    ylabel('Recovery outcome');
    title('Recovery Envelope (Catch Radius)');
    legend('PID', 'ADRC', 'Location', 'best');
    grid on;
    saveas(f, fullfile(out_dir, 'recovery_envelope.png'));
    close(f);
end

function [f_hz, psd_u] = estimate_psd_fft(u, t)
    x = u(:) - mean(u(:));
    N = numel(x);
    dt = mean(diff(t));
    fs = 1 / max(dt, 1e-9);

    X = fft(x);
    P2 = (abs(X) .^ 2) / max(N * fs, 1e-9);
    K = floor(N / 2) + 1;
    psd_u = P2(1:K);
    if K > 2
        psd_u(2:end - 1) = 2 * psd_u(2:end - 1);
    end
    f_hz = (0:K - 1)' * fs / N;
end

function tbl = build_normalized_importance_table(explicit_rank_tbl, atlas_global_tbl)
    % Merge two importance views:
    % 1) explicit sweeps (profile-specific, absolute delta RMSE)
    % 2) standardized atlas coefficients (cross-factor normalized in high-D space)

    fac = unique([string(explicit_rank_tbl.factor); string(atlas_global_tbl.factor)], 'stable');
    n = numel(fac);

    exp_pid = zeros(n, 1);
    exp_adrc = zeros(n, 1);
    atl_pid = zeros(n, 1);
    atl_adrc = zeros(n, 1);

    for i = 1:n
        ie = find(string(explicit_rank_tbl.factor) == fac(i), 1, 'first');
        ia = find(string(atlas_global_tbl.factor) == fac(i), 1, 'first');

        if ~isempty(ie)
            exp_pid(i) = explicit_rank_tbl.importance_pid(ie);
            exp_adrc(i) = explicit_rank_tbl.importance_adrc(ie);
        end
        if ~isempty(ia)
            atl_pid(i) = atlas_global_tbl.importance_pid(ia);
            atl_adrc(i) = atlas_global_tbl.importance_adrc(ia);
        end
    end

    exp_pid_n = normalize01(exp_pid);
    exp_adrc_n = normalize01(exp_adrc);
    atl_pid_n = normalize01(atl_pid);
    atl_adrc_n = normalize01(atl_adrc);

    % Balanced fusion score: local explicit + global standardized atlas.
    fused_pid = 0.5 * exp_pid_n + 0.5 * atl_pid_n;
    fused_adrc = 0.5 * exp_adrc_n + 0.5 * atl_adrc_n;

    tbl = table(fac, exp_pid, exp_adrc, atl_pid, atl_adrc, exp_pid_n, exp_adrc_n, atl_pid_n, atl_adrc_n, fused_pid, fused_adrc, ...
        'VariableNames', {'factor', 'explicit_pid_raw', 'explicit_adrc_raw', 'atlas_pid_raw', 'atlas_adrc_raw', ...
        'explicit_pid_norm01', 'explicit_adrc_norm01', 'atlas_pid_norm01', 'atlas_adrc_norm01', ...
        'fused_pid_score', 'fused_adrc_score'});

    tbl = sortrows(tbl, 'fused_adrc_score', 'descend');
    tbl.rank_fused_adrc = (1:height(tbl))';
    tbl = movevars(tbl, 'rank_fused_adrc', 'Before', 'factor');
end

function y = normalize01(x)
    x = x(:);
    xmin = min(x);
    xmax = max(x);
    if xmax - xmin < 1e-12
        y = zeros(size(x));
    else
        y = (x - xmin) / (xmax - xmin);
    end
end

function make_normalized_importance_figure(tbl, out_dir)
    top = tbl(1:min(8, height(tbl)), :);

    f = figure('Position', [160, 120, 1200, 650], 'ToolBar', 'none');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    barh(categorical(top.factor), [top.explicit_adrc_norm01, top.atlas_adrc_norm01], 'grouped');
    xlabel('Normalized importance (0..1)');
    title('ADRC Importance: Explicit vs Standardized Atlas');
    legend('Explicit sweep norm', 'Atlas norm', 'Location', 'best');
    grid on;

    nexttile;
    barh(categorical(top.factor), top.fused_adrc_score, 'FaceColor', [0.00 0.45 0.74]);
    xlabel('Fused normalized importance');
    title('ADRC Fair-Scaling Priority (Fused Score)');
    grid on;

    sgtitle('Cross-Factor Fairness Check: Are We Scaling Equitably?');
    saveas(f, fullfile(out_dir, 'normalized_factor_importance.png'));
    close(f);
end
