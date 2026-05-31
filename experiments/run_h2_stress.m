function summary = run_h2_stress()
%RUN_H2_STRESS  Stress-test H2 midpoint-design dominance.
% Tightens u_max toward H1 optimum-edge and increases disturbance to see
% whether midpoint-LQR still beats p0/p1-LQR designs.

here = fileparts(mfilename('fullpath'));
addpath(here);

modes = {'p0','mid','p1'};

% Stressed config: low u_max, big gust, slow slew
stress_opts = struct();
stress_opts.u_max    = 3.5;
stress_opts.slew_max = 25;
stress_opts.gust_std = 3.5;
stress_opts.theta0_set = deg2rad([2 5 8]);
stress_opts.seeds    = 1:12;
stress_opts.p1_grid  = [8 10 12 14 16];
stress_opts.t_burn_grid = [4.0 2.0 1.0 0.5 0.25];

fprintf('=== H2 STRESS: u_max=%g slew=%g gust=%g ===\n', ...
    stress_opts.u_max, stress_opts.slew_max, stress_opts.gust_std);

all_results = struct();
for m = 1:numel(modes)
    o = stress_opts; o.design_mode = modes{m};
    fprintf('\n  design=%s\n', modes{m});
    r = h2_driftrate_sweep(o);
    SR = r.success_rate;
    p1g = r.p1_grid; tbg = r.t_burn_grid;
    fprintf('   p1\\t_burn |');
    for it = 1:numel(tbg), fprintf(' %5.2f', tbg(it)); end
    fprintf('\n');
    for ip = 1:numel(p1g)
        fprintf('   %6.1f   |', p1g(ip));
        for it = 1:numel(tbg), fprintf(' %5.2f', SR(ip,it)); end
        fprintf('\n');
    end
    all_results.(modes{m}) = r;
end

% Head-to-head: midpoint vs others, per cell
fprintf('\nMidpoint advantage matrix (success_mid - max(success_p0, success_p1)):\n');
r_p0  = all_results.p0.success_rate;
r_mid = all_results.mid.success_rate;
r_p1  = all_results.p1.success_rate;
adv = r_mid - max(r_p0, r_p1);
p1g = all_results.mid.p1_grid; tbg = all_results.mid.t_burn_grid;
fprintf('   p1\\t_burn |');
for it = 1:numel(tbg), fprintf(' %5.2f', tbg(it)); end
fprintf('\n');
for ip = 1:numel(p1g)
    fprintf('   %6.1f   |', p1g(ip));
    for it = 1:numel(tbg), fprintf(' %+5.2f', adv(ip,it)); end
    fprintf('\n');
end

fprintf('\nMidpoint mean(success) over grid: p0=%.3f  mid=%.3f  p1=%.3f\n', ...
    mean(r_p0(:)), mean(r_mid(:)), mean(r_p1(:)));

% CSV
fid = fopen(fullfile(here, 'results', 'h2_stress_summary.csv'), 'w');
fprintf(fid, 'design,p1,t_burn,success\n');
for m = 1:numel(modes)
    r = all_results.(modes{m});
    for ip = 1:numel(r.p1_grid)
        for it = 1:numel(r.t_burn_grid)
            fprintf(fid, '%s,%g,%g,%.4f\n', modes{m}, ...
                r.p1_grid(ip), r.t_burn_grid(it), r.success_rate(ip,it));
        end
    end
end
fclose(fid);
fprintf('Wrote results/h2_stress_summary.csv\n');

summary = all_results;
end
