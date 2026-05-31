function summary = run_h1()
%RUN_H1  Execute the H1 coupling experiment and report.

here = fileparts(mfilename('fullpath'));
addpath(here);

opts = struct();
opts.p = 6.0;            % marginally unstable amateur scale
opts.slew_grid = linspace(3, 60, 14);
opts.umax_grid = linspace(2, 18, 14);
opts.seeds = 1:4;
opts.theta0_set = deg2rad([1 3 5 8]);

fprintf('=== H1 COUPLING EXPERIMENT (p=%.1f 1/s) ===\n', opts.p);
results = h1_coupling_sweep(opts);
summary = h1_analyze_coupling(results, 0.75);

% Save a quick text summary
outdir = fullfile(here, 'results');
fid = fopen(fullfile(outdir, 'h1_summary.txt'), 'w');
fprintf(fid, 'H1 coupling experiment, p=%.2f 1/s\n', opts.p);
fprintf(fid, 'grid: slew %dx u_max %d\n', numel(opts.slew_grid), numel(opts.umax_grid));
fprintf(fid, 'composed stable iff slew>=%.2f AND u_max>=%.2f\n', ...
    summary.slew_marginal_at_umax_max, summary.umax_marginal_at_slew_max);
fprintf(fid, 'coupling loss fraction: %.3f\n', summary.coupling_loss_frac);
fprintf(fid, 'overpredict cell rate: %.3f\n', summary.overpredict_cell_rate);
fclose(fid);
fprintf('Wrote results/h1_summary.txt\n');
end
