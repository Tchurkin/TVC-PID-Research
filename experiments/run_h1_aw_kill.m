function summary = run_h1_aw_kill()
%RUN_H1_AW_KILL  Hostile-reviewer falsification test #1.
% Re-runs the H1 lock sweep with a rate-aware anti-windup (AW) command shaper.
% If the non-monotone u_max dip vanishes with AW, the H1 finding reduces to
% "naive controller is slew-unaware" -- known result, not novel.
% If the dip persists with AW, H1 is a real plant-property finding.

here = fileparts(mfilename('fullpath'));
addpath(here);

P_set    = [8 10 12];
slew_set = [25 45 80];
umax_grid = linspace(2, 12, 21);
seeds    = 1:12;
theta0_set = deg2rad([2 5 8 12]);
gust_std = 2.5;
gust_tau = 0.30;
dt = 0.005;
t_end = 2.5;
latency_n = 2;
gyro_std = 0.03;
damp = 0.2;
keff = 8.0;
tau_act = 0.05;
theta_fail = deg2rad(40);
trials_per_cell = numel(seeds) * numel(theta0_set);

% Use the aggressive LQR (the one that showed the biggest H1 dip)
Q = diag([400, 2]); R = 0.5;

fprintf('=== H1 ANTI-WINDUP KILL TEST ===\n');
fprintf('  Comparing constraint-blind LQR vs LQR + rate-aware AW command shaper\n');
fprintf('  Aggressive Q=diag(400,2) R=0.5\n\n');

rows = [];
for ip = 1:numel(P_set)
    p_val = P_set(ip);
    K = design_lqr_unstable(p_val, damp, keff, Q, R);
    for is = 1:numel(slew_set)
        slew = slew_set(is);
        succ_off = zeros(1, numel(umax_grid));
        succ_on  = zeros(1, numel(umax_grid));
        for iu = 1:numel(umax_grid)
            wins_off = 0; wins_on = 0;
            for ith = 1:numel(theta0_set)
                for s = seeds
                    base = struct('p', p_val, 'damp', damp, 'keff', keff, ...
                        'tau_act', tau_act, 'slew_max', slew, ...
                        'u_max', umax_grid(iu), 'latency_n', latency_n, ...
                        'gyro_std', gyro_std, 'K', K, 'dt', dt, ...
                        't_end', t_end, 'theta0', theta0_set(ith), ...
                        'gust_std', gust_std, 'gust_tau', gust_tau);
                    % AW OFF
                    rng(s + 100*ith + 7*ip + 13*is);
                    base.use_rate_aw = false;
                    o1 = sim_unstable(base);
                    s1 = classify_outcome(o1, theta_fail);
                    if s1, wins_off = wins_off + 1; end
                    % AW ON (same seed for paired comparison)
                    rng(s + 100*ith + 7*ip + 13*is);
                    base.use_rate_aw = true;
                    o2 = sim_unstable(base);
                    s2 = classify_outcome(o2, theta_fail);
                    if s2, wins_on = wins_on + 1; end
                end
            end
            succ_off(iu) = wins_off / trials_per_cell;
            succ_on(iu)  = wins_on  / trials_per_cell;
        end

        [pk_off, ipk_off] = max(succ_off);
        [pk_on,  ipk_on]  = max(succ_on);
        last_off = succ_off(end); last_on = succ_on(end);
        drop_off = last_off - pk_off;
        drop_on  = last_on  - pk_on;
        uopt_off = umax_grid(ipk_off);
        uopt_on  = umax_grid(ipk_on);

        fprintf('  p=%2d slew=%3d :\n', p_val, slew);
        fprintf('    AW OFF :  u*=%5.2f  peak=%.3f  high-u=%.3f  drop=%+.3f\n', ...
            uopt_off, pk_off, last_off, drop_off);
        fprintf('    AW ON  :  u*=%5.2f  peak=%.3f  high-u=%.3f  drop=%+.3f   dip_change=%+.3f\n', ...
            uopt_on, pk_on, last_on, drop_on, drop_on - drop_off);

        rows(end+1, :) = [p_val, slew, uopt_off, pk_off, last_off, drop_off, ...
                          uopt_on, pk_on, last_on, drop_on, drop_on - drop_off]; %#ok<AGROW>
    end
end

% Aggregate verdict
drops_off = rows(:,6);
drops_on  = rows(:,10);
mean_drop_off = mean(drops_off);
mean_drop_on  = mean(drops_on);
max_drop_off = min(drops_off);  % most negative = worst dip
max_drop_on  = min(drops_on);

fprintf('\n=== VERDICT ===\n');
fprintf('  Mean dip (AW off): %+.3f   Mean dip (AW on): %+.3f   diff: %+.3f\n', ...
    mean_drop_off, mean_drop_on, mean_drop_on - mean_drop_off);
fprintf('  Worst dip (AW off): %+.3f  Worst dip (AW on): %+.3f\n', max_drop_off, max_drop_on);
if mean_drop_on > -0.05 && max_drop_on > -0.05
    fprintf('  -> RATE-AWARE AW KILLS THE DIP. H1 reduces to "naive controller is slew-unaware".\n');
elseif mean_drop_on > mean_drop_off + 0.10
    fprintf('  -> AW substantially reduces dip but does not eliminate it. H1 partially novel.\n');
else
    fprintf('  -> AW does NOT remove dip. H1 survives as a plant-property finding.\n');
end

% CSV
fid = fopen(fullfile(here, 'results', 'h1_aw_kill.csv'), 'w');
fprintf(fid, 'p,slew,uopt_off,peak_off,last_off,drop_off,uopt_on,peak_on,last_on,drop_on,dip_change\n');
for r = 1:size(rows,1)
    fprintf(fid, '%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', rows(r,:));
end
fclose(fid);
fprintf('Wrote results/h1_aw_kill.csv\n');

summary.rows = rows;
summary.mean_drop_off = mean_drop_off;
summary.mean_drop_on  = mean_drop_on;
end
