%% SimToReal_TVC_Research.m
% TVC rocket sim-to-real mismatch decomposition pipeline.
% Research focus: quantify which physical effects dominate S2R breakdown
% and what minimum model fidelity preserves decision-useful simulation.

clear; close all; clc;

addpath(fullfile(pwd, 'src'));

cfg = s2r_build_config();
rng(cfg.execution.rng_seed);

out = s2r_prepare_output_dirs(cfg);
telem = s2r_load_telemetry_dataset(cfg.paths.telemetry_dir);

fprintf('\n===============================================================\n');
fprintf('TVC SIM-TO-REAL MISMATCH DECOMPOSITION PIPELINE\n');
fprintf('===============================================================\n');
fprintf('Mode: %s\n', cfg.execution.mode);
fprintf('Telemetry dir: %s\n\n', cfg.paths.telemetry_dir);

results = s2r_run_pipeline(cfg, telem, out);

log_path = fullfile(out.logs, 'run_summary.txt');
fid = fopen(log_path, 'w');
fprintf(fid, 'Run mode: %s\n', cfg.execution.mode);
fprintf(fid, 'RNG seed: %d\n', cfg.execution.rng_seed);
fprintf(fid, 'Launches processed: %d\n', height(results.replay_tbl));
fprintf(fid, 'Mean replay RMSE theta (deg): %.3f\n', mean(results.replay_tbl.rmse_theta_deg));
fprintf(fid, 'Mean replay trend corr: %.3f\n', mean(results.replay_tbl.trend_corr));
fprintf(fid, 'Overall stability match rate: %.3f\n', mean(double(results.replay_tbl.stability_match)));
fprintf(fid, 'Top sensitivity source: %s\n', string(results.rank_tbl.source(1)));
fclose(fid);

fprintf('Saved outputs:\n');
fprintf('  Graphs: %s\n', out.graphs);
fprintf('  Sheets: %s\n', out.sheets);
fprintf('  Logs  : %s\n', out.logs);
fprintf('  - %s\n', fullfile(out.sheets, 'sim_vs_real_replay_metrics.csv'));
fprintf('  - %s\n', fullfile(out.sheets, 'decomposition_samples.csv'));
fprintf('  - %s\n', fullfile(out.sheets, 'decomposition_summary.csv'));
fprintf('  - %s\n', fullfile(out.sheets, 'sensitivity_ranking.csv'));
fprintf('  - %s\n', fullfile(out.sheets, 'stress_map_points.csv'));
fprintf('  - %s\n', fullfile(out.sheets, 'fidelity_thresholds.csv'));
fprintf('  - %s\n', fullfile(out.graphs, 'sim_to_real_error_decomposition.png'));
fprintf('  - %s\n', fullfile(out.graphs, 'mismatch_sensitivity_ranking.png'));
fprintf('  - %s\n', fullfile(out.graphs, 'stress_map_pca.png'));
fprintf('  - %s\n', fullfile(out.graphs, 'fidelity_thresholds.png'));

fprintf('\nPipeline complete.\n');
