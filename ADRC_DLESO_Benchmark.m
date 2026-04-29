%% ADRC_DLESO_Benchmark
% STS-focused benchmark harness: PID vs ADRC with DLESO under
% uncertainty, gust disturbance, and actuator nonlinearity.
%
% Outputs:
%   outputs/adrc_benchmark_results.csv
%   outputs/figures/23_adrc_vs_pid_success_rate.png
%
% This script is designed to run in base MATLAB without requiring Simulink.

clear; close all; clc;
rng(11);

cfg = default_config();
scenario_tbl = build_scenarios();
results = run_benchmark(cfg, scenario_tbl);
frontier_results = run_bandwidth_frontier(cfg, scenario_tbl);
frontier_summary = build_frontier_summary(frontier_results, cfg);
write_outputs(cfg, results, frontier_results, frontier_summary);
plot_summary(cfg, results, frontier_results, frontier_summary);

fprintf('\nADRC benchmark complete.\n');
fprintf('Results CSV: outputs/adrc_benchmark_results.csv\n');
fprintf('Frontier CSV: outputs/adrc_bandwidth_frontier_results.csv\n');
fprintf('Frontier Summary CSV: outputs/adrc_bandwidth_frontier_summary.csv\n');
fprintf('Figure: outputs/figures/23_adrc_vs_pid_success_rate.png\n\n');

%% ------------------------------------------------------------------------
function cfg = default_config()
    cfg.dt = 0.005;
    cfg.t_final = 6.0;
    cfg.n_steps = floor(cfg.t_final / cfg.dt);

    cfg.theta0_deg = 6.0;
    cfg.ref_deg = 0.0;

    cfg.nominal_b0 = 28.0;
    cfg.nominal_a = 2.6;     % positive -> open-loop unstable tendency
    cfg.damping = 2.0;

    cfg.sensor_noise_deg = 0.15;
    cfg.gyro_noise_deg_s = 0.6;

    cfg.wind_filter_hz = 2.5;
    cfg.wind_base_mag = 8.0; % disturbance acceleration scaling

    cfg.actuator.max_deflection_deg = 10.0;
    cfg.actuator.max_rate_deg_s = 240.0;
    cfg.actuator.backlash_deg = 0.18;
    cfg.actuator.time_constant_s = 0.045;

    % PID baseline gains
    cfg.pid.kp = 1.0;
    cfg.pid.ki = 0.25;
    cfg.pid.kd = 0.08;

    % ADRC gains and observer settings
    cfg.adrc.b0 = cfg.nominal_b0;
    cfg.adrc.wc = 7.0;    % controller bandwidth (rad/s)
    cfg.adrc.wo = 14.0;   % observer bandwidth (rad/s)
    cfg.adrc.ki_scale = 0.08;

    cfg.mc_trials = 36;

    cfg.frontier.wo_rad_s = [6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0];
    cfg.frontier.actuator_delay_ms = [30.0, 40.0, 50.0, 60.0];
    cfg.frontier.sensor_noise_scale = [0.8, 1.0, 1.25, 1.5, 1.75];
    cfg.frontier.scenario_names = ["CoP30_Wind25"; "CoP45_Wind40"];
    cfg.frontier.mc_trials = 18;
    cfg.frontier.success_threshold = 0.80;
    cfg.frontier.success_slack = 0.05;
    cfg.frontier.max_loop_utilization = 0.05;

    cfg.output_dir = fullfile(pwd, 'outputs');
    cfg.figure_dir = fullfile(cfg.output_dir, 'figures');
end

%% ------------------------------------------------------------------------
function tbl = build_scenarios()
    % stress levels roughly aligned with STS-style robustness claim structure
    scenario_name = [
        "Nominal"
        "CoP10_Wind10"
        "CoP15_Wind15"
        "CoP30_Wind25"
        "CoP45_Wind40"
        "CoP60_Wind55"
    ];
    cop_uncertainty = [0.00; 0.10; 0.15; 0.30; 0.45; 0.60];
    wind_scale = [0.00; 0.10; 0.15; 0.25; 0.40; 0.55];
    noise_scale = [1.00; 1.10; 1.20; 1.30; 1.45; 1.60];

    tbl = table(scenario_name, cop_uncertainty, wind_scale, noise_scale);
end

%% ------------------------------------------------------------------------
function results = run_benchmark(cfg, scenario_tbl)
    controllers = ["PID"; "ADRC"];

    row = 0;
    out = struct([]);

    for s = 1:height(scenario_tbl)
        for c = 1:numel(controllers)
            ctrl_name = controllers(c);

            stable = false(cfg.mc_trials, 1);
            rms_theta_deg = zeros(cfg.mc_trials, 1);
            sat_fraction = zeros(cfg.mc_trials, 1);
            mean_loop_us = zeros(cfg.mc_trials, 1);
            eso_error_rms = nan(cfg.mc_trials, 1);

            for k = 1:cfg.mc_trials
                trial = simulate_one(cfg, scenario_tbl(s, :), ctrl_name, k);
                stable(k) = trial.stable;
                rms_theta_deg(k) = trial.rms_theta_deg;
                sat_fraction(k) = trial.sat_fraction;
                mean_loop_us(k) = trial.mean_loop_us;
                eso_error_rms(k) = trial.eso_error_rms;
            end

            row = row + 1;
            out(row).scenario = scenario_tbl.scenario_name(s);
            out(row).controller = ctrl_name;
            out(row).success_rate = mean(stable);
            out(row).rms_theta_deg_mean = mean(rms_theta_deg);
            out(row).rms_theta_deg_std = std(rms_theta_deg);
            out(row).sat_fraction_mean = mean(sat_fraction);
            out(row).mean_loop_us = mean(mean_loop_us);
            out(row).eso_error_rms = mean(eso_error_rms, 'omitnan');
            out(row).cop_uncertainty = scenario_tbl.cop_uncertainty(s);
            out(row).wind_scale = scenario_tbl.wind_scale(s);
            out(row).noise_scale = scenario_tbl.noise_scale(s);
        end
    end

    results = struct2table(out);
end

%% ------------------------------------------------------------------------
function frontier = run_bandwidth_frontier(cfg, scenario_tbl)
    row = 0;
    out = struct([]);

    for s = 1:numel(cfg.frontier.scenario_names)
        scenario_name = cfg.frontier.scenario_names(s);
        scenario_idx = find(scenario_tbl.scenario_name == scenario_name, 1, 'first');
        if isempty(scenario_idx)
            error('Scenario %s not found in scenario table.', scenario_name);
        end

        base_scenario = scenario_tbl(scenario_idx, :);

        for d = 1:numel(cfg.frontier.actuator_delay_ms)
            delay_ms = cfg.frontier.actuator_delay_ms(d);

            for n_idx = 1:numel(cfg.frontier.sensor_noise_scale)
                noise_scale = cfg.frontier.sensor_noise_scale(n_idx);
                scenario_row = base_scenario;
                scenario_row.noise_scale = noise_scale;

                for w = 1:numel(cfg.frontier.wo_rad_s)
                    wo = cfg.frontier.wo_rad_s(w);
                    cfg_trial = cfg;
                    cfg_trial.actuator.time_constant_s = delay_ms / 1000;
                    cfg_trial.adrc.wo = wo;
                    cfg_trial.mc_trials = cfg.frontier.mc_trials;

                    stable = false(cfg.frontier.mc_trials, 1);
                    rms_theta_deg = zeros(cfg.frontier.mc_trials, 1);
                    sat_fraction = zeros(cfg.frontier.mc_trials, 1);
                    mean_loop_us = zeros(cfg.frontier.mc_trials, 1);
                    eso_error_rms = zeros(cfg.frontier.mc_trials, 1);

                    for k = 1:cfg.frontier.mc_trials
                        seed = 7000 + 1000 * s + 100 * d + 10 * n_idx + k;
                        trial = simulate_one(cfg_trial, scenario_row, "ADRC", seed);
                        stable(k) = trial.stable;
                        rms_theta_deg(k) = trial.rms_theta_deg;
                        sat_fraction(k) = trial.sat_fraction;
                        mean_loop_us(k) = trial.mean_loop_us;
                        eso_error_rms(k) = trial.eso_error_rms;
                    end

                    row = row + 1;
                    out(row).scenario = scenario_name;
                    out(row).controller = "ADRC";
                    out(row).cop_uncertainty = scenario_row.cop_uncertainty;
                    out(row).wind_scale = scenario_row.wind_scale;
                    out(row).sensor_noise_scale = noise_scale;
                    out(row).actuator_delay_ms = delay_ms;
                    out(row).adrc_wo_rad_s = wo;
                    out(row).success_rate = mean(stable);
                    out(row).rms_theta_deg_mean = mean(rms_theta_deg);
                    out(row).rms_theta_deg_std = std(rms_theta_deg);
                    out(row).sat_fraction_mean = mean(sat_fraction);
                    out(row).eso_error_rms = mean(eso_error_rms);
                    out(row).mean_loop_us = mean(mean_loop_us);
                    out(row).loop_utilization = mean(mean_loop_us) / (cfg_trial.dt * 1e6);
                end
            end
        end
    end

    frontier = struct2table(out);
end

%% ------------------------------------------------------------------------
function summary = build_frontier_summary(frontier_results, cfg)
    scenarios = unique(frontier_results.scenario, 'stable');
    delay_vals = unique(frontier_results.actuator_delay_ms, 'stable');
    noise_vals = unique(frontier_results.sensor_noise_scale, 'stable');

    n_rows = numel(scenarios) * numel(delay_vals) * numel(noise_vals);
    summary = table('Size', [n_rows, 11], ...
        'VariableTypes', {'string','double','double','double','double','double','double','double','double','double','string'}, ...
        'VariableNames', {'scenario','actuator_delay_ms','sensor_noise_scale','best_success_rate','recommended_wo_rad_s', ...
        'recommended_eso_error_rms','recommended_loop_utilization','feasible_wo_min_rad_s','feasible_wo_max_rad_s', ...
        'admissible_count','recommendation_flag'});

    row = 1;
    for s = 1:numel(scenarios)
        for d = 1:numel(delay_vals)
            for n_idx = 1:numel(noise_vals)
                idx = frontier_results.scenario == scenarios(s) & ...
                      frontier_results.actuator_delay_ms == delay_vals(d) & ...
                      frontier_results.sensor_noise_scale == noise_vals(n_idx);
                rows = sortrows(frontier_results(idx, :), 'adrc_wo_rad_s');
                admissible = rows.success_rate >= cfg.frontier.success_threshold & ...
                             rows.loop_utilization <= cfg.frontier.max_loop_utilization;

                summary.scenario(row) = scenarios(s);
                summary.actuator_delay_ms(row) = delay_vals(d);
                summary.sensor_noise_scale(row) = noise_vals(n_idx);
                summary.best_success_rate(row) = max(rows.success_rate);

                if any(admissible)
                    admissible_rows = rows(admissible, :);
                    near_best = admissible_rows(admissible_rows.success_rate >= max(admissible_rows.success_rate) - cfg.frontier.success_slack, :);
                    picked = near_best(1, :);

                    summary.recommended_wo_rad_s(row) = picked.adrc_wo_rad_s;
                    summary.recommended_eso_error_rms(row) = picked.eso_error_rms;
                    summary.recommended_loop_utilization(row) = picked.loop_utilization;
                    summary.feasible_wo_min_rad_s(row) = min(admissible_rows.adrc_wo_rad_s);
                    summary.feasible_wo_max_rad_s(row) = max(admissible_rows.adrc_wo_rad_s);
                    summary.admissible_count(row) = height(admissible_rows);
                    summary.recommendation_flag(row) = "FEASIBLE_REGION_FOUND";
                else
                    [~, best_idx] = max(rows.success_rate - 0.02 * rows.eso_error_rms);
                    picked = rows(best_idx, :);

                    summary.recommended_wo_rad_s(row) = picked.adrc_wo_rad_s;
                    summary.recommended_eso_error_rms(row) = picked.eso_error_rms;
                    summary.recommended_loop_utilization(row) = picked.loop_utilization;
                    summary.feasible_wo_min_rad_s(row) = nan;
                    summary.feasible_wo_max_rad_s(row) = nan;
                    summary.admissible_count(row) = 0;
                    summary.recommendation_flag(row) = "NO_FEASIBLE_REGION_USE_BEST_AVAILABLE";
                end

                row = row + 1;
            end
        end
    end
end

%% ------------------------------------------------------------------------
function trial = simulate_one(cfg, scenario_row, controller_name, seed)
    rng(1000 + seed);

    dt = cfg.dt;
    n = cfg.n_steps;

    theta = deg2rad(cfg.theta0_deg);
    q = 0.0;

    theta_ref = deg2rad(cfg.ref_deg);
    u_act = 0.0;
    u_prev = 0.0;

    pid_int = 0.0;
    adrc_int = 0.0;

    z = [theta; q; 0.0];  % [z1 z2 z3]'

    sat_hits = 0;
    diverged = false;
    theta_hist = zeros(n,1);
    d_true_hist = zeros(n,1);
    z3_hist = zeros(n,1);
    loop_us_hist = zeros(n,1);

    % scenario uncertainty and noise
    cop_u = scenario_row.cop_uncertainty;
    wind_scale = scenario_row.wind_scale;
    ns = scenario_row.noise_scale;

    % true plant parameters under uncertainty
    a_true = cfg.nominal_a * (1 + cop_u*(2*rand-1));
    b_true = cfg.nominal_b0 * (1 + 0.12*(2*rand-1));

    wind_state = 0.0;

    for i = 1:n
        t = i * dt;

        % stochastic gust process (first-order shaping as practical Dryden-like surrogate)
        [wind_state, gust] = wind_process(wind_state, cfg, wind_scale);

        % unknown total disturbance injected into rotational acceleration
        aero_nl = 0.55 * sin(2.2 * theta) + 0.09 * q * abs(q);
        d_true = cfg.wind_base_mag * gust + aero_nl;

        theta_meas = theta + deg2rad(cfg.sensor_noise_deg * ns) * randn;
        q_meas = q + deg2rad(cfg.gyro_noise_deg_s * ns) * randn;

        t0 = tic;
        switch controller_name
            case "PID"
                [u_cmd, pid_int] = pid_controller(theta_meas, q_meas, theta_ref, pid_int, cfg.pid, dt);
                eso_err = nan;

            case "ADRC"
                [z, u_cmd, adrc_int, eso_err] = adrc_dleso_controller(theta_meas, theta_ref, u_prev, z, adrc_int, cfg.adrc, dt, d_true);

            otherwise
                error('Unknown controller: %s', controller_name);
        end
        loop_us_hist(i) = toc(t0) * 1e6;

        [u_act, did_sat] = actuator_block(u_cmd, u_act, cfg.actuator, dt);
        sat_hits = sat_hits + did_sat;
        u_prev = u_act;

        q_dot = a_true * theta - cfg.damping * q + b_true * u_act + d_true;
        theta_dot = q;

        q = q + dt * q_dot;
        theta = theta + dt * theta_dot;

        theta_hist(i) = theta;
        d_true_hist(i) = d_true;
        z3_hist(i) = z(3);

        if abs(theta) > deg2rad(45)
            diverged = true;
            theta_hist(i:end) = theta;
            d_true_hist(i:end) = d_true;
            z3_hist(i:end) = z(3);
            break;
        end

        % Allow full-time evolution; divergence is defined by hard attitude blow-up.
    end

    rms_theta = sqrt(mean(theta_hist.^2));
    % STS-grade success criterion: high-precision tracking, not just non-divergence.
    stable = ~diverged && (rms_theta < deg2rad(1.5)) && (abs(theta_hist(end)) < deg2rad(2.5));

    trial.stable = stable;
    trial.rms_theta_deg = rad2deg(rms_theta);
    trial.sat_fraction = sat_hits / n;
    trial.mean_loop_us = mean(loop_us_hist);
    if controller_name == "ADRC"
        trial.eso_error_rms = sqrt(mean((z3_hist - d_true_hist).^2));
    else
        trial.eso_error_rms = nan;
    end
end

%% ------------------------------------------------------------------------
function [u_cmd, i_state] = pid_controller(theta, q, ref, i_state, gains, dt)
    e = ref - theta;
    i_state = i_state + e * dt;
    u_cmd = gains.kp * e + gains.ki * i_state - gains.kd * q;
end

%% ------------------------------------------------------------------------
function [z, u_cmd, i_state, eso_err] = adrc_dleso_controller(y, ref, u_prev, z, i_state, p, dt, d_true)
    wo = p.wo;
    wc = p.wc;

    beta1 = 3 * wo;
    beta2 = 3 * wo^2;
    beta3 = wo^3;

    % ESO update
    e = z(1) - y;
    z(1) = z(1) + dt * (z(2) - beta1 * e);
    z(2) = z(2) + dt * (z(3) + p.b0 * u_prev - beta2 * e);
    z(3) = z(3) + dt * (-beta3 * e);

    % ADRC control law
    ep = z(1) - ref;
    i_state = i_state + ep * dt;
    kp = wc^2;
    kd = 2 * wc;
    ki = p.ki_scale * wc^3;

    v = -kp * ep - kd * z(2) - ki * i_state;
    u_cmd = (v - z(3)) / p.b0;

    eso_err = z(3) - d_true;
end

%% ------------------------------------------------------------------------
function [x_next, gust] = wind_process(x, cfg, wind_scale)
    dt = cfg.dt;
    w = 2*pi*cfg.wind_filter_hz;

    % First-order shaping filter for turbulence-like stochastic input
    eta = randn;
    x_dot = -w * x + sqrt(2*w) * eta;
    x_next = x + dt * x_dot;

    % add occasional coherent gust pulse
    pulse = 0.0;
    if rand < 0.004
        pulse = 2.0 * (2*rand-1);
    end

    gust = wind_scale * (x_next + pulse);
end

%% ------------------------------------------------------------------------
function [u_out, did_sat] = actuator_block(u_cmd, u_prev, act, dt)
    max_u = deg2rad(act.max_deflection_deg);
    max_rate = deg2rad(act.max_rate_deg_s);
    backlash = deg2rad(act.backlash_deg);

    u_cmd = clamp(u_cmd, -max_u, max_u);

    du = u_cmd - u_prev;
    du = clamp(du, -max_rate*dt, max_rate*dt);
    u_rate = u_prev + du;

    if abs(u_rate - u_prev) < backlash
        u_bl = u_prev;
    else
        u_bl = u_rate;
    end

    alpha = dt / max(act.time_constant_s, dt);
    u_lag = u_prev + alpha * (u_bl - u_prev);

    u_out = clamp(u_lag, -max_u, max_u);
    did_sat = abs(u_out) >= (0.999 * max_u);
end

%% ------------------------------------------------------------------------
function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

%% ------------------------------------------------------------------------
function write_outputs(cfg, results, frontier_results, frontier_summary)
    if ~exist(cfg.output_dir, 'dir')
        mkdir(cfg.output_dir);
    end
    writetable(results, fullfile(cfg.output_dir, 'adrc_benchmark_results.csv'));
    writetable(frontier_results, fullfile(cfg.output_dir, 'adrc_bandwidth_frontier_results.csv'));
    writetable(frontier_summary, fullfile(cfg.output_dir, 'adrc_bandwidth_frontier_summary.csv'));
end

%% ------------------------------------------------------------------------
function plot_summary(cfg, results, frontier_results, frontier_summary)
    if ~exist(cfg.figure_dir, 'dir')
        mkdir(cfg.figure_dir);
    end

    scenarios = unique(results.scenario, 'stable');
    pid_vals = zeros(numel(scenarios),1);
    adrc_vals = zeros(numel(scenarios),1);

    for i = 1:numel(scenarios)
        s = scenarios(i);
        pid_vals(i) = results.success_rate(results.scenario == s & results.controller == "PID");
        adrc_vals(i) = results.success_rate(results.scenario == s & results.controller == "ADRC");
    end

    fig = figure('Name', 'ADRC vs PID Robustness');
    b = bar([pid_vals, adrc_vals], 'grouped');
    b(1).FaceColor = [0.75 0.35 0.35];
    b(2).FaceColor = [0.25 0.55 0.85];

    set(gca, 'XTickLabel', cellstr(scenarios));
    ylabel('Stability Success Rate');
    xlabel('Stress Scenario');
    title('Robustness Stress Test: PID vs ADRC (DLESO)');
    legend({'PID','ADRC'}, 'Location', 'southwest');
    ylim([0,1]);
    grid on;

    ax = gca;
    if isprop(ax, 'Toolbar')
        ax.Toolbar.Visible = 'off';
    end

    exportgraphics(fig, fullfile(cfg.figure_dir, '23_adrc_vs_pid_success_rate.png'), 'Resolution', 150);
    close(fig);

    plot_frontier_heatmaps(cfg, frontier_results, frontier_summary);
    plot_frontier_tradeoff(cfg, frontier_results);
end

%% ------------------------------------------------------------------------
function plot_frontier_heatmaps(cfg, frontier_results, frontier_summary)
    scenarios = unique(frontier_summary.scenario, 'stable');
    delay_vals = unique(frontier_summary.actuator_delay_ms, 'stable');
    noise_vals = unique(frontier_summary.sensor_noise_scale, 'stable');

    fig = figure('Name', 'ADRC Observer Frontier');
    tiledlayout(numel(scenarios), 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for s = 1:numel(scenarios)
        success_map = nan(numel(delay_vals), numel(noise_vals));
        wo_map = nan(numel(delay_vals), numel(noise_vals));

        for d = 1:numel(delay_vals)
            for n_idx = 1:numel(noise_vals)
                summary_idx = frontier_summary.scenario == scenarios(s) & ...
                    frontier_summary.actuator_delay_ms == delay_vals(d) & ...
                    frontier_summary.sensor_noise_scale == noise_vals(n_idx);
                rows = frontier_results(frontier_results.scenario == scenarios(s) & ...
                    frontier_results.actuator_delay_ms == delay_vals(d) & ...
                    frontier_results.sensor_noise_scale == noise_vals(n_idx), :);

                success_map(d, n_idx) = max(rows.success_rate);
                wo_map(d, n_idx) = frontier_summary.recommended_wo_rad_s(summary_idx);
            end
        end

        nexttile;
        imagesc(noise_vals, delay_vals, success_map);
        set(gca, 'YDir', 'normal');
        xlabel('Sensor noise scale');
        ylabel('Actuator delay (ms)');
        title(sprintf('%s max success rate', scenarios(s)));
        c = colorbar;
        c.Label.String = 'Success rate';

        nexttile;
        imagesc(noise_vals, delay_vals, wo_map);
        set(gca, 'YDir', 'normal');
        xlabel('Sensor noise scale');
        ylabel('Actuator delay (ms)');
        title(sprintf('%s recommended observer bandwidth', scenarios(s)));
        c = colorbar;
        c.Label.String = 'Recommended w_o (rad/s)';
    end

    exportgraphics(fig, fullfile(cfg.figure_dir, '24_adrc_bandwidth_delay_noise_frontier.png'), 'Resolution', 150);
    close(fig);
end

%% ------------------------------------------------------------------------
function plot_frontier_tradeoff(cfg, frontier_results)
    scenario_name = cfg.frontier.scenario_names(end);
    rows = frontier_results(frontier_results.scenario == scenario_name, :);

    fig = figure('Name', 'ADRC Frontier Tradeoff');
    scatter(rows.eso_error_rms, rows.success_rate, 70, rows.adrc_wo_rad_s, 'filled');
    grid on;
    xlabel('ESO disturbance-estimate RMS error');
    ylabel('Success rate');
    title(sprintf('%s observer tradeoff', scenario_name));
    cb = colorbar;
    cb.Label.String = 'Observer bandwidth w_o (rad/s)';

    exportgraphics(fig, fullfile(cfg.figure_dir, '25_adrc_observer_tradeoff.png'), 'Resolution', 150);
    close(fig);
end
