function summary = run_solver_invariance()
%RUN_SOLVER_INVARIANCE  Falsification test #4: re-run H1 with halved dt.
% If u*_max shifts by >10% or dip magnitude changes by >5pp, finding is
% a discretization artifact.

here = fileparts(mfilename('fullpath'));
addpath(here);

p_val = 10; damp = 0.2; keff = 8.0; tau_act = 0.05; slew = 45;
umax_grid = linspace(2, 12, 11);
seeds = 1:8; theta0_set = deg2rad([2 5 8 12]);
gust_std = 2.5; gust_tau = 0.30;
latency_s = 0.010;   % keep latency in seconds, not samples
t_end = 2.5;
gyro_std = 0.03;
theta_fail = deg2rad(40);
K = design_lqr_unstable(p_val, damp, keff, diag([400 2]), 0.5);
trials_per_cell = numel(seeds) * numel(theta0_set);

dt_set = [0.005, 0.0025, 0.00125];
fprintf('=== SOLVER INVARIANCE TEST (p=%d slew=%d) ===\n', p_val, slew);
fprintf('  Sweep u_max at multiple dt; if dip moves, finding is numerical\n\n');

all_succ = zeros(numel(dt_set), numel(umax_grid));
for id = 1:numel(dt_set)
    dt = dt_set(id);
    latency_n = max(1, round(latency_s / dt));
    succ = zeros(1, numel(umax_grid));
    for iu = 1:numel(umax_grid)
        wins = 0;
        for ith = 1:numel(theta0_set)
            for s = seeds
                rng(s + 100*ith + 7*iu);
                pr = struct('p', p_val, 'damp', damp, 'keff', keff, ...
                    'tau_act', tau_act, 'slew_max', slew, 'u_max', umax_grid(iu), ...
                    'latency_n', latency_n, 'gyro_std', gyro_std, 'K', K, ...
                    'dt', dt, 't_end', t_end, 'theta0', theta0_set(ith), ...
                    'gust_std', gust_std, 'gust_tau', gust_tau);
                out = sim_unstable(pr);
                st = classify_outcome(out, theta_fail);
                if st, wins = wins + 1; end
            end
        end
        succ(iu) = wins / trials_per_cell;
    end
    all_succ(id, :) = succ;
    [pk, ipk] = max(succ);
    fprintf('  dt=%.5fs (lat=%dsamples): u*=%5.2f peak=%.3f last=%.3f drop=%+.3f\n', ...
        dt, latency_n, umax_grid(ipk), pk, succ(end), succ(end)-pk);
end

% Compute differences
u_opts = zeros(1, numel(dt_set));
dips   = zeros(1, numel(dt_set));
for id = 1:numel(dt_set)
    [pk, ipk] = max(all_succ(id,:));
    u_opts(id) = umax_grid(ipk);
    dips(id) = all_succ(id, end) - pk;
end
u_opt_range = max(u_opts) - min(u_opts);
dip_range   = max(dips) - min(dips);
fprintf('\n  u*_max range across dt: %.2f (relative %.1f%%)\n', u_opt_range, 100*u_opt_range/mean(u_opts));
fprintf('  dip magnitude range:    %.3f\n', dip_range);
if u_opt_range/mean(u_opts) < 0.10 && dip_range < 0.05
    fprintf('  -> INVARIANT to solver step. Finding is not a discretization artifact.\n');
else
    fprintf('  -> NOT INVARIANT. Finding may be numerical.\n');
end

fid = fopen(fullfile(here, 'results', 'solver_invariance.csv'), 'w');
fprintf(fid, 'dt');
for iu = 1:numel(umax_grid), fprintf(fid, ',u_max=%g', umax_grid(iu)); end
fprintf(fid, '\n');
for id = 1:numel(dt_set)
    fprintf(fid, '%.5f', dt_set(id));
    for iu = 1:numel(umax_grid), fprintf(fid, ',%.4f', all_succ(id,iu)); end
    fprintf(fid, '\n');
end
fclose(fid);
fprintf('Wrote results/solver_invariance.csv\n');

summary.all_succ = all_succ;
summary.dt_set   = dt_set;
end
