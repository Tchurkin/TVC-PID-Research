function summary = run_h1_bestk_kill()
%RUN_H1_BESTK_KILL  Hostile-reviewer falsification test #2.
% For each u_max cell, sweep R over a wide range and take the BEST
% achievable success rate. This is a poor-man's saturated-LQR design.
% If best-K success is monotone in u_max, then the H1 dip is purely an
% artifact of using one fixed R across all u_max. If even best-K still
% dips, H1 is a real plant property (fundamental tradeoff).

here = fileparts(mfilename('fullpath'));
addpath(here);

p_val   = 10;
damp    = 0.2;
keff    = 8.0;
tau_act = 0.05;
slew    = 45;
umax_grid = linspace(2, 12, 11);
R_grid    = [0.1 0.25 0.5 1.0 2.0 4.0 8.0 16.0 32.0 64.0];
seeds   = 1:8;
theta0_set = deg2rad([2 5 8 12]);
gust_std = 2.5; gust_tau = 0.30;
dt = 0.005; t_end = 2.5;
latency_n = 2; gyro_std = 0.03;
theta_fail = deg2rad(40);
trials_per_cell = numel(seeds) * numel(theta0_set);

fprintf('=== H1 BEST-K KILL TEST  (p=%d, slew=%d) ===\n', p_val, slew);
fprintf('  At each u_max, take max success rate across R in {%s}\n\n', num2str(R_grid));

succ_default = zeros(1, numel(umax_grid));
succ_best    = zeros(1, numel(umax_grid));
R_best       = zeros(1, numel(umax_grid));
succ_grid    = zeros(numel(umax_grid), numel(R_grid));

for iu = 1:numel(umax_grid)
    u_max = umax_grid(iu);
    for iR = 1:numel(R_grid)
        K = design_lqr_unstable(p_val, damp, keff, diag([400 2]), R_grid(iR));
        wins = 0;
        for ith = 1:numel(theta0_set)
            for s = seeds
                rng(s + 100*ith + 7*iu + 13*iR);
                pr = struct('p', p_val, 'damp', damp, 'keff', keff, ...
                    'tau_act', tau_act, 'slew_max', slew, 'u_max', u_max, ...
                    'latency_n', latency_n, 'gyro_std', gyro_std, 'K', K, ...
                    'dt', dt, 't_end', t_end, 'theta0', theta0_set(ith), ...
                    'gust_std', gust_std, 'gust_tau', gust_tau);
                out = sim_unstable(pr);
                st = classify_outcome(out, theta_fail);
                if st, wins = wins + 1; end
            end
        end
        succ_grid(iu, iR) = wins / trials_per_cell;
    end
    [succ_best(iu), iRb] = max(succ_grid(iu, :));
    R_best(iu) = R_grid(iRb);
    succ_default(iu) = succ_grid(iu, R_grid==0.5);
end

fprintf('  u_max | succ(default R=0.5) | succ(best K) | R_best\n');
for iu = 1:numel(umax_grid)
    fprintf('  %5.2f  |   %.3f             |   %.3f       |  %.2f\n', ...
        umax_grid(iu), succ_default(iu), succ_best(iu), R_best(iu));
end

[pk_def, ipk_def] = max(succ_default);
last_def = succ_default(end);
[pk_best, ipk_best] = max(succ_best);
last_best = succ_best(end);

fprintf('\n  Default LQR:  peak=%.3f @ u=%.2f   last=%.3f   drop=%+.3f\n', ...
    pk_def, umax_grid(ipk_def), last_def, last_def-pk_def);
fprintf('  Best-K LQR:   peak=%.3f @ u=%.2f   last=%.3f   drop=%+.3f\n', ...
    pk_best, umax_grid(ipk_best), last_best, last_best-pk_best);

dip_def  = last_def  - pk_def;
dip_best = last_best - pk_best;

fprintf('\n=== VERDICT ===\n');
if dip_best > -0.05
    fprintf('  -> Best-K is MONOTONE (dip=%+.3f). H1 reduces to "use the right R for each u_max"\n', dip_best);
    fprintf('     i.e. the prescriptive finding holds: pair u_max with appropriately tuned K.\n');
elseif dip_best > dip_def + 0.10
    fprintf('  -> Best-K substantially reduces dip (%.3f -> %.3f).\n', dip_def, dip_best);
    fprintf('     H1 is partly controller-design, partly plant property.\n');
else
    fprintf('  -> Best-K STILL DIPS (%.3f -> %.3f). H1 is a fundamental plant property.\n', dip_def, dip_best);
end

% CSV
fid = fopen(fullfile(here, 'results', 'h1_bestk_kill.csv'), 'w');
fprintf(fid, 'u_max,succ_default,succ_best,R_best\n');
for iu = 1:numel(umax_grid)
    fprintf(fid, '%.4f,%.4f,%.4f,%.4f\n', umax_grid(iu), succ_default(iu), succ_best(iu), R_best(iu));
end
fclose(fid);
% Full grid
fid = fopen(fullfile(here, 'results', 'h1_bestk_grid.csv'), 'w');
fprintf(fid, 'u_max');
for iR = 1:numel(R_grid), fprintf(fid, ',R=%g', R_grid(iR)); end
fprintf(fid, '\n');
for iu = 1:numel(umax_grid)
    fprintf(fid, '%.4f', umax_grid(iu));
    for iR = 1:numel(R_grid), fprintf(fid, ',%.4f', succ_grid(iu, iR)); end
    fprintf(fid, '\n');
end
fclose(fid);
fprintf('Wrote results/h1_bestk_kill.csv and h1_bestk_grid.csv\n');

summary.succ_default = succ_default;
summary.succ_best    = succ_best;
summary.R_best       = R_best;
summary.umax_grid    = umax_grid;
end
