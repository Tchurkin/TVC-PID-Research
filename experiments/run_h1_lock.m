function summary = run_h1_lock()
%RUN_H1_LOCK  Lock the H1 discovery:
%   (a) verify non-monotone u_max optimum at multiple instability levels
%   (b) verify the optimum survives with a much gentler LQR design
%       (rules out "aggressive Q/R is to blame")
%   (c) emit a predictive table of u*_max vs (p, slew)

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

fprintf('=== H1 LOCK: scan over p in {%s}, slew in {%s}, two LQR designs ===\n', ...
    num2str(P_set), num2str(slew_set));

LQR_designs = struct( ...
    'name', {'aggressive', 'gentle'}, ...
    'Q',    {diag([400, 2]),  diag([80, 1])}, ...
    'R',    {0.5,             4.0});

table_rows = {};  %#ok<NASGU>
table_rows = [];

for d = 1:numel(LQR_designs)
    Q = LQR_designs(d).Q; R = LQR_designs(d).R;
    for ip = 1:numel(P_set)
        p_val = P_set(ip);
        K = design_lqr_unstable(p_val, damp, keff, Q, R);
        for is = 1:numel(slew_set)
            slew = slew_set(is);
            succ = zeros(1, numel(umax_grid));
            for iu = 1:numel(umax_grid)
                wins = 0;
                for ith = 1:numel(theta0_set)
                    for s = seeds
                        rng(s + 100*ith + 7*ip + 13*d);
                        pr = struct('p', p_val, 'damp', damp, 'keff', keff, ...
                            'tau_act', tau_act, 'slew_max', slew, ...
                            'u_max', umax_grid(iu), 'latency_n', latency_n, ...
                            'gyro_std', gyro_std, 'K', K, 'dt', dt, ...
                            't_end', t_end, 'theta0', theta0_set(ith), ...
                            'gust_std', gust_std, 'gust_tau', gust_tau);
                        out = sim_unstable(pr);
                        st = classify_outcome(out, theta_fail);
                        if st, wins = wins + 1; end
                    end
                end
                succ(iu) = wins / trials_per_cell;
            end
            [pk, ipk] = max(succ);
            last = succ(end);
            u_opt = umax_grid(ipk);
            drop  = last - pk;
            fprintf('  %-10s p=%2d slew=%3d  u*=%5.2f  peak=%.3f  high-u=%.3f  drop=%+.3f\n', ...
                LQR_designs(d).name, p_val, slew, u_opt, pk, last, drop);
            table_rows = [table_rows; ...
                {LQR_designs(d).name, p_val, slew, u_opt, pk, last, drop, succ}]; %#ok<AGROW>
        end
    end
end

% Write CSV
fid = fopen(fullfile(here, 'results', 'h1_lock_summary.csv'), 'w');
fprintf(fid, 'design,p,slew,u_opt,peak_success,success_at_u12,drop\n');
for r = 1:size(table_rows,1)
    fprintf(fid, '%s,%d,%d,%.3f,%.3f,%.3f,%.3f\n', ...
        table_rows{r,1}, table_rows{r,2}, table_rows{r,3}, ...
        table_rows{r,4}, table_rows{r,5}, table_rows{r,6}, table_rows{r,7});
end
fclose(fid);
fprintf('\nWrote results/h1_lock_summary.csv\n');

summary.table = table_rows;
summary.umax_grid = umax_grid;
end
