%% main_sim_to_real_diagnostic.m
% Diagnostic runner for quick testing of the s2r pipeline with minimal data.
% Runs only the first 1 launch with 10 MC samples per source.

clc;
fprintf('=== TVC S2R Diagnostic Run ===\n');
fprintf('Started: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% 0. Add paths
repo_root = fileparts(mfilename('fullpath'));
addpath(fullfile(repo_root, 'src'));
addpath(fullfile(repo_root, 'tools'));

%% 1. Build config and override for diagnostic
cfg = s2r_build_config();
cfg.execution.mode = "quick";
cfg.analysis.mc_runs_per_source = 10;    % Reduce from 120
cfg.analysis.stress_map_samples = 50;    % Reduce from 300
cfg.analysis.morris_trajectories = 2;    % Reduce from 5
cfg.analysis.max_launches = 1;           % Only 1 launch

rng(cfg.execution.rng_seed);
fprintf('Mode: diagnostic | Launches: %d | MC/source: %d\n', ...
    cfg.analysis.max_launches, cfg.analysis.mc_runs_per_source);

%% 2. Prepare outputs
out = s2r_prepare_output_dirs(cfg);

%% 3. Load telemetry
fprintf('Loading telemetry...\n');
telem = s2r_load_telemetry_dataset(cfg.paths.telemetry_dir);
fprintf('Total launches: %d\n', height(telem.launch_manifest));

%% 4. Run one launch manually to debug
fprintf('\n--- Testing first launch ---\n');
launch_id = telem.launch_manifest.launch_id(1);
fprintf('Launch: %s\n', launch_id);

% Build launch case
idx_l = telem.launch_manifest.launch_id == launch_id;
L = telem.launch_manifest(idx_l, :);
idx_t = telem.imu.launch_id == launch_id;
trace = telem.imu(idx_t, :);

launch_case.launch_id = launch_id;
launch_case.controller = upper(string(L.controller(1)));
launch_case.theta0_deg = trace.theta_deg(1);
launch_case.q0_deg_s = trace.q_deg_s(1);
launch_case.v0_mps = 4.0;
launch_case.h0_m = 0.0;
launch_case.wind_mps = double(L.wind_mps(1));
launch_case.payload_kg = double(L.payload_kg(1));
launch_case.motor_scale = 1.0;
launch_case.trace_real.t = double(trace.time_s);
launch_case.trace_real.theta_deg = double(trace.theta_deg);
launch_case.trace_real.q_deg_s = double(trace.q_deg_s);
launch_case.trace_real.delta_cmd_deg = double(trace.delta_cmd_deg);

fprintf('Running nominal simulation...\n');
tic;
run_nom = s2r_run_simulation(cfg, launch_case, launch_case.controller, struct());
nom_time = toc;
fprintf('  Nominal sim completed in %.2f sec\n', nom_time);

fprintf('Running one mismatch sample...\n');
tic;
mismatch.actuator_delay_s = 0.01;
run_mismatch = s2r_run_simulation(cfg, launch_case, launch_case.controller, mismatch);
mismatch_time = toc;
fprintf('  Mismatch sim completed in %.2f sec\n', mismatch_time);

% Estimate total time
total_est = (cfg.analysis.mc_runs_per_source * 8 * cfg.analysis.max_launches) * mean([nom_time, mismatch_time]);
fprintf('\nEstimated total time for full decomposition: %.1f sec (%.1f min)\n', total_est, total_est/60);

if total_est > 600
    fprintf('WARNING: Estimated time exceeds 10 minutes. Consider reducing MC samples further.\n');
end

fprintf('\n--- Diagnostic complete. ---\n');
