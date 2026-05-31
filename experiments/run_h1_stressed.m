function summary = run_h1_stressed()
%RUN_H1_STRESSED  Push instability + disturbance until both limits bind.

here = fileparts(mfilename('fullpath'));
addpath(here);

opts = struct();
opts.p           = 10.0;          % strongly unstable (harder amateur airframe)
opts.damp        = 0.2;
opts.keff        = 8.0;
opts.tau_act     = 0.05;
opts.latency_n   = 2;             % 10 ms loop delay
opts.gyro_std    = 0.03;
opts.dt          = 0.005;
opts.t_end       = 2.5;
opts.theta0_set  = deg2rad([2 5 8 12 16]);
opts.gust_std    = 2.5;           % aggressive wind
opts.gust_tau    = 0.30;
opts.seeds       = 1:6;
opts.slew_grid   = linspace(2, 50, 16);
opts.umax_grid   = linspace(0.5, 8, 16);
opts.theta_fail_rad = deg2rad(40);
opts.outdir      = fullfile(here, 'results');

fprintf('=== H1 STRESSED COUPLING (p=%.1f, gust=%.1f) ===\n', opts.p, opts.gust_std);
results = h1_coupling_sweep(opts);
summary = h1_analyze_coupling(results, 0.70);

fid = fopen(fullfile(opts.outdir, 'h1_stressed_summary.txt'), 'w');
fprintf(fid, 'H1 stressed: p=%.2f, gust=%.2f, latency_n=%d\n', ...
    opts.p, opts.gust_std, opts.latency_n);
fprintf(fid, 'composed marginals: slew=%.2f, u_max=%.2f\n', ...
    summary.slew_marginal_at_umax_max, summary.umax_marginal_at_slew_max);
fprintf(fid, 'coupling loss fraction: %.3f\n', summary.coupling_loss_frac);
fclose(fid);
end
