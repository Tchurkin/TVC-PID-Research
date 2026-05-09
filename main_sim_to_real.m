%% main_sim_to_real.m
% Top-level runner for the TVC sim-to-real mismatch decomposition study.
%
% USAGE
%   Run this file from the repo root in MATLAB or from the command line:
%       matlab -batch "main_sim_to_real"
%
% EXECUTION MODES (set in s2r_build_config → cfg.execution.mode)
%   "quick"  – 120 MC runs/source, 300 stress-map samples. ~2–5 min.
%   "full"   – 1000 MC runs/source, 2000 stress-map samples. For paper runs.
%
% OUTPUTS
%   outputs/s2r_decomposition/sheets/  – all CSV result tables
%   outputs/s2r_decomposition/graphs/  – all PNG figures
%   outputs/s2r_decomposition/logs/    – run log with timing and config hash

clc;
fprintf('=== TVC Sim-to-Real Decomposition Pipeline ===\n');
fprintf('Started: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% 0. Add source and tools directories to path
repo_root = fileparts(mfilename('fullpath'));
addpath(fullfile(repo_root, 'src'));
addpath(fullfile(repo_root, 'tools'));

%% 1. Build configuration
cfg = s2r_build_config();
rng(cfg.execution.rng_seed);
fprintf('Mode: %s | RNG seed: %d\n', cfg.execution.mode, cfg.execution.rng_seed);

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

%% 6. Write run log
log_file = fullfile(out.logs, 'run_log.txt');
fid = fopen(log_file, 'w');
fprintf(fid, 'Run timestamp : %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, 'Mode          : %s\n', cfg.execution.mode);
fprintf(fid, 'RNG seed      : %d\n', cfg.execution.rng_seed);
fprintf(fid, 'MC runs/source: %d\n', cfg.analysis.mc_runs_per_source);
fprintf(fid, 'Elapsed (s)   : %.1f\n', elapsed);
fprintf(fid, 'Launches used : %d\n', height(results.replay_tbl));
fprintf(fid, 'Total MC samples: %d\n', height(results.sample_tbl));
fclose(fid);

fprintf('\nAll outputs written to: %s\n', out.root);
fprintf('Run log: %s\n', log_file);
fprintf('=== Done ===\n');
