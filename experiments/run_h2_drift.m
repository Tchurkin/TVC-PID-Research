function summary = run_h2_drift()
%RUN_H2_DRIFT  Discovery sweep of instability drift rate.
% Compares three LQR design choices: at p0 (pre-burn), at midpoint, at p1 (worst).

here = fileparts(mfilename('fullpath'));
addpath(here);

modes = {'p0','mid','p1'};
all_results = struct();

for m = 1:numel(modes)
    fprintf('\n=== H2 DRIFT RATE, design=%s ===\n', modes{m});
    r = h2_driftrate_sweep(struct('design_mode', modes{m}));

    p1g = r.p1_grid; tbg = r.t_burn_grid;
    fprintf('  Success vs (rows=p1, cols=t_burn[s]):\n');
    fprintf('  p1\\t_burn |');
    for it = 1:numel(tbg), fprintf(' %6.2f', tbg(it)); end
    fprintf('\n');
    for ip = 1:numel(p1g)
        fprintf('  %6.1f    |', p1g(ip));
        for it = 1:numel(tbg), fprintf(' %6.2f', r.success_rate(ip,it)); end
        fprintf('\n');
    end
    fprintf('  Drift rate matrix (1/s^2):\n');
    for ip = 1:numel(p1g)
        fprintf('  %6.1f    |', p1g(ip));
        for it = 1:numel(tbg), fprintf(' %6.2f', r.drift_rate(ip,it)); end
        fprintf('\n');
    end

    all_results.(modes{m}) = r;
end

% CSV export, long form
fid = fopen(fullfile(here, 'results', 'h2_drift_long.csv'), 'w');
fprintf(fid, 'design_mode,p1,t_burn_s,drift_rate_per_s2,success_rate,med_t_first_fail_s\n');
for m = 1:numel(modes)
    r = all_results.(modes{m});
    for ip = 1:numel(r.p1_grid)
        for it = 1:numel(r.t_burn_grid)
            tfm = r.t_first_fail_med(ip,it);
            if isnan(tfm), tstr = 'NaN'; else, tstr = sprintf('%.4f', tfm); end
            fprintf(fid, '%s,%g,%g,%.4f,%.4f,%s\n', modes{m}, ...
                r.p1_grid(ip), r.t_burn_grid(it), r.drift_rate(ip,it), ...
                r.success_rate(ip,it), tstr);
        end
    end
end
fclose(fid);
fprintf('\nWrote results/h2_drift_long.csv\n');

summary = all_results;
end
