function summary = run_h1_verify()
%RUN_H1_VERIFY  Verify the non-monotone u_max optimum observed in stressed sweep.
% Uses finer u_max resolution and more seeds at the highest slew to isolate
% whether the dip is real or noise.

here = fileparts(mfilename('fullpath'));
addpath(here);

opts = struct();
opts.p           = 10.0;
opts.damp        = 0.2;
opts.keff        = 8.0;
opts.tau_act     = 0.05;
opts.latency_n   = 2;
opts.gyro_std    = 0.03;
opts.dt          = 0.005;
opts.t_end       = 2.5;
opts.theta0_set  = deg2rad([2 5 8 12 16]);
opts.gust_std    = 2.5;
opts.gust_tau    = 0.30;
opts.seeds       = 1:16;             % more seeds for tight estimates
opts.slew_grid   = [20 35 50 80];    % representative slews including very large
opts.umax_grid   = linspace(2, 12, 21);
opts.theta_fail_rad = deg2rad(40);
opts.outdir      = fullfile(here, 'results');

fprintf('=== H1 VERIFY (p=%.1f, gust=%.1f, more seeds, fine u_max) ===\n', ...
    opts.p, opts.gust_std);
results = h1_coupling_sweep(opts);

% Plot success rate vs u_max for each slew level
SR = results.success_rate;
fid = fopen(fullfile(opts.outdir, 'h1_verify_table.csv'), 'w');
fprintf(fid, 'slew,u_max,success_rate\n');
for is = 1:numel(opts.slew_grid)
    for iu = 1:numel(opts.umax_grid)
        fprintf(fid, '%.3f,%.3f,%.4f\n', ...
            opts.slew_grid(is), opts.umax_grid(iu), SR(is,iu));
    end
end
fclose(fid);

% Per-slew peak finding
fprintf('\n  slew    u_max_opt   peak_success   success_at_max_u\n');
for is = 1:numel(opts.slew_grid)
    row = SR(is,:);
    [pk, ipk] = max(row);
    last = row(end);
    fprintf('  %5.1f   %7.3f    %6.3f         %6.3f   (drop=%+.3f)\n', ...
        opts.slew_grid(is), opts.umax_grid(ipk), pk, last, last-pk);
end

summary.success_rate = SR;
summary.slew_grid = opts.slew_grid;
summary.umax_grid = opts.umax_grid;
end
