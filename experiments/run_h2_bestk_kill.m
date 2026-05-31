function summary = run_h2_bestk_kill()
%RUN_H2_BESTK_KILL  Apply the best-K (R-tuned) recipe to H2.
% Does the "conservative design wins" claim survive when both designs are
% allowed to optimize over R?

here = fileparts(mfilename('fullpath'));
addpath(here);

% Stressed config from run_h2_stress
u_max = 3.5; slew_max = 25; gust_std = 3.5;
theta0_set = deg2rad([2 5 8]);
seeds = 1:12;
p0 = 4; p1_grid = [8 10 12 14 16];
t_burn_grid = [4.0 2.0 1.0 0.5 0.25];
damp = 0.2; keff = 8.0; tau_act = 0.05;
latency_n = 2; gyro_std = 0.03; dt = 0.005;
theta_fail = deg2rad(40);
trials_per_cell = numel(seeds) * numel(theta0_set);

% R values to search per design
R_grid = [0.25 0.5 1.0 2.0 4.0 8.0 16.0 32.0];

design_set = {'p0', 'p0', 'mid', 'mid', 'p1', 'p1'};
R_set      = {0.5, 'best', 0.5, 'best', 0.5, 'best'};
labels     = {'p0/R=0.5','p0/best-R','mid/R=0.5','mid/best-R','p1/R=0.5','p1/best-R'};

fprintf('=== H2 BEST-K KILL TEST (stress: u_max=%g slew=%g gust=%g) ===\n', ...
    u_max, slew_max, gust_std);

results = struct();
for d = 1:numel(design_set)
    mode = design_set{d};
    Rmode = R_set{d};
    SR = zeros(numel(p1_grid), numel(t_burn_grid));
    for ip = 1:numel(p1_grid)
        p1 = p1_grid(ip);
        switch mode
            case 'p0',  p_design = p0;
            case 'mid', p_design = 0.5*(p0+p1);
            case 'p1',  p_design = p1;
        end
        for it = 1:numel(t_burn_grid)
            t_burn = t_burn_grid(it);
            t_end = t_burn + 0.8;
            p_traj = @(t) p0 + (p1 - p0) * min(max(t/t_burn, 0), 1);

            if ischar(Rmode) || isstring(Rmode)
                R_search = R_grid;
            else
                R_search = Rmode;
            end

            best_succ = 0;
            for iR = 1:numel(R_search)
                K = design_lqr_unstable(p_design, damp, keff, diag([400 2]), R_search(iR));
                wins = 0;
                for ith = 1:numel(theta0_set)
                    for s = seeds
                        rng(s + 100*ith + 7*d + 13*ip + 31*it + 51*iR);
                        pr = struct('p', p0, 'damp', damp, 'keff', keff, ...
                            'tau_act', tau_act, 'slew_max', slew_max, ...
                            'u_max', u_max, 'latency_n', latency_n, ...
                            'gyro_std', gyro_std, 'K', K, 'dt', dt, ...
                            't_end', t_end, 'theta0', theta0_set(ith), ...
                            'gust_std', gust_std, 'gust_tau', 0.30, ...
                            'p_traj', p_traj);
                        out = sim_unstable(pr);
                        st = classify_outcome(out, theta_fail);
                        if st, wins = wins + 1; end
                    end
                end
                sr = wins / trials_per_cell;
                if sr > best_succ, best_succ = sr; end
            end
            SR(ip, it) = best_succ;
        end
    end
    results.(matlab.lang.makeValidName(labels{d})) = SR;
    fprintf('\n  %s   (mean over grid = %.3f)\n', labels{d}, mean(SR(:)));
    fprintf('     p1\\t_burn |');
    for it = 1:numel(t_burn_grid), fprintf(' %5.2f', t_burn_grid(it)); end
    fprintf('\n');
    for ip = 1:numel(p1_grid)
        fprintf('     %6.1f   |', p1_grid(ip));
        for it = 1:numel(t_burn_grid), fprintf(' %5.2f', SR(ip,it)); end
        fprintf('\n');
    end
end

% Verdict
mean_p0_fixed  = mean(results.p0_R_0_5(:));
mean_p0_best   = mean(results.p0_best_R(:));
mean_mid_fixed = mean(results.mid_R_0_5(:));
mean_mid_best  = mean(results.mid_best_R(:));
mean_p1_fixed  = mean(results.p1_R_0_5(:));
mean_p1_best   = mean(results.p1_best_R(:));

fprintf('\n=== VERDICT ===\n');
fprintf('  fixed R=0.5     : p0=%.3f  mid=%.3f  p1=%.3f  -> %s wins\n', ...
    mean_p0_fixed, mean_mid_fixed, mean_p1_fixed, ...
    winner(mean_p0_fixed, mean_mid_fixed, mean_p1_fixed));
fprintf('  best-R per cell : p0=%.3f  mid=%.3f  p1=%.3f  -> %s wins\n', ...
    mean_p0_best, mean_mid_best, mean_p1_best, ...
    winner(mean_p0_best, mean_mid_best, mean_p1_best));

if mean_p1_best >= mean_p0_best - 0.05 && mean_p1_best >= mean_mid_best - 0.05
    fprintf('  -> With best-R, the "conservative design wins" rule COLLAPSES. H2 is fixed-R artifact.\n');
elseif mean_p0_best > mean_p1_best + 0.05
    fprintf('  -> H2 conservative-design rule SURVIVES even under best-R tuning.\n');
else
    fprintf('  -> H2 partially survives -- ranking changed but not by much.\n');
end

fid = fopen(fullfile(here, 'results', 'h2_bestk_kill.csv'), 'w');
fprintf(fid, 'design,R_mode,mean_success\n');
fprintf(fid, 'p0,0.5,%.4f\np0,best,%.4f\nmid,0.5,%.4f\nmid,best,%.4f\np1,0.5,%.4f\np1,best,%.4f\n', ...
    mean_p0_fixed, mean_p0_best, mean_mid_fixed, mean_mid_best, mean_p1_fixed, mean_p1_best);
fclose(fid);
fprintf('Wrote results/h2_bestk_kill.csv\n');

summary = results;
end

function w = winner(a,b,c)
[~, i] = max([a b c]);
opts = {'p0','mid','p1'};
w = opts{i};
end
