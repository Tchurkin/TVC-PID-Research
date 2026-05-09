%% main_sim_to_real_ultra_quick.m
% Ultra-quick test runner: 1 launch, 20 MC samples, minimal Morris/incremental.
% Should complete in ~2 minutes. For testing pipeline end-to-end before full runs.

clc;
fprintf('=== TVC S2R Ultra-Quick Test Run ===\n');
fprintf('Started: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% 0. Add source and tools directories to path
repo_root = fileparts(mfilename('fullpath'));
addpath(fullfile(repo_root, 'src'));
addpath(fullfile(repo_root, 'tools'));

%% 1. Build configuration with ultra_quick mode
cfg = s2r_build_config();
cfg.execution.mode = "ultra_quick";
cfg.analysis.max_launches = 1;
rng(cfg.execution.rng_seed);
fprintf('Mode: ultra_quick | Max launches: %d | MC/source: %d\n', ...
    cfg.analysis.max_launches, cfg.analysis.mc_runs_per_source);

%% 2. Prepare output directories
out = s2r_prepare_output_dirs(cfg);

%% 3. Load telemetry dataset
fprintf('Loading telemetry from: %s\n', cfg.paths.telemetry_dir);
telem = s2r_load_telemetry_dataset(cfg.paths.telemetry_dir);
fprintf('Launches available: %d\n', height(telem.launch_manifest));

%% 4. Run pipeline (replay + decomposition + analysis)
t0 = tic;
results = s2r_run_pipeline(cfg, telem, out);
elapsed = toc(t0);
fprintf('\nPipeline complete in %.1f seconds.\n', elapsed);

%% 5. Display summary to console
fprintf('\n--- Sensitivity Ranking (top sources by impact) ---\n');
disp(results.rank_tbl(:, {'rank', 'source', 'impact_score', 'mean_rmse_deg', 'agreement_rate'}));

fprintf('\n--- Fidelity Thresholds ---\n');
disp(results.fidelity_tbl);

if isfield(results, 'conclusion')
    fprintf('\n--- Central Conclusion ---\n%s\n', results.conclusion);
end

fprintf('\n=== Ultra-Quick Test Complete ===\n');
