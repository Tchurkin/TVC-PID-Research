function summary = run_realistic_h1_sweep()
%RUN_REALISTIC_H1_SWEEP  Port H1 (best-R reframed) to the realistic plant.
% Uses ModelRocket_Adaptive_TVC/simulate_case_realistic with p_unstable set
% on top of the production realistic sensor/actuator/gust stack.
% Verifies the (u_max, R_optimal) pairing rule the unstable-sim discovered.

here = fileparts(mfilename('fullpath'));
addpath(here);
proj = fileparts(here);
addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src', 'utils'));

cfg = rocket_defaults();
% Make the plant statically unstable like the H1 study
cfg.plant.p_unstable = 8.0;
cfg.plant.aero_damp  = 0.5;       % moderate aero damping (still much less than stable default 1.2)
cfg.plant.control_eff = 8.0;
cfg.plant.tau_act     = 0.05;
cfg.t_end_demo        = 3.0;

% Scenario: no faults, just nominal disturbance
sc.t_end = 2.5;
sc.fault_time = 999;             % never
sc.kind = 'h1_realistic';
sc.control_eff_scale_post = 1;
sc.aero_damp_scale_post   = 1;
sc.tau_scale_post         = 1;
sc.slew_scale_post        = 1;
sc.disturbance_amp        = 0.0;
sc.disturbance_freq_hz    = 1.0;
sc.disturb_scale_post     = 1;
sc.disturb_bias_post      = 0;

realism = struct();
realism.gust_std = 1.5;
realism.gust_tau = 0.30;

umax_grid = [3 4 5 6 8 10 12];
slew_grid = [20 35 50];
R_grid    = [0.25 0.5 1.0 2.0 4.0 8.0 16.0 32.0];
seeds     = 1:8;
theta0_set = deg2rad([2 5 8 12]);
theta_fail = deg2rad(40);
trials_per_cell = numel(seeds) * numel(theta0_set);

p = cfg.plant.p_unstable;
damp = cfg.plant.aero_damp;
keff = cfg.plant.control_eff;

fprintf('=== REALISTIC-PLANT H1 SWEEP (p_unstable=%g) ===\n', p);
fprintf('  Uses production simulate_case_realistic with full sensor/actuator stack\n');
fprintf('  Compares fixed R=0.5 to best-R per (u_max, slew) cell\n\n');

succ_def  = zeros(numel(slew_grid), numel(umax_grid));
succ_best = zeros(numel(slew_grid), numel(umax_grid));
R_best    = zeros(numel(slew_grid), numel(umax_grid));

for is = 1:numel(slew_grid)
    cfg.plant.slew_max = slew_grid(is);
    for iu = 1:numel(umax_grid)
        cfg.plant.u_max = umax_grid(iu);
        % design candidate Ks
        Ks = cell(1, numel(R_grid));
        for iR = 1:numel(R_grid)
            Ks{iR} = design_lqr_unstable(p, damp, keff, diag([400 2]), R_grid(iR));
        end
        % evaluate
        succ_per_R = zeros(1, numel(R_grid));
        for iR = 1:numel(R_grid)
            % override fixed-LQR controller with this K
            cfg.controllers.FIXED_LQR.K = Ks{iR};
            cfg.controllers.FIXED_LQR.K_nominal = Ks{iR};
            cfg.controllers.FIXED_LQR.u_max = umax_grid(iu);
            wins = 0;
            for ith = 1:numel(theta0_set)
                cfg.plant.theta0 = theta0_set(ith);
                for s = seeds
                    seed = s + 100*ith + 7*is + 13*iu + 31*iR;
                    out = simulate_case_realistic('FIXED_LQR', sc, cfg, seed, realism);
                    th = out.theta;
                    if all(abs(th) < theta_fail)
                        wins = wins + 1;
                    end
                end
            end
            succ_per_R(iR) = wins / trials_per_cell;
        end
        succ_def(is, iu)  = succ_per_R(R_grid==0.5);
        [succ_best(is, iu), iRb] = max(succ_per_R);
        R_best(is, iu) = R_grid(iRb);
    end
    fprintf('  slew=%2d:\n', slew_grid(is));
    fprintf('    u_max:   '); fprintf(' %6g', umax_grid); fprintf('\n');
    fprintf('    def R0.5:'); fprintf(' %6.2f', succ_def(is,:)); fprintf('\n');
    fprintf('    best-R:  '); fprintf(' %6.2f', succ_best(is,:)); fprintf('\n');
    fprintf('    R*:      '); fprintf(' %6.2f', R_best(is,:));   fprintf('\n');
end

% Aggregate verdict
[pk_def, ~]  = max(succ_def(:));
[pk_best, ~] = max(succ_best(:));
mean_def  = mean(succ_def(:));
mean_best = mean(succ_best(:));
fprintf('\n  FixedR    mean=%.3f peak=%.3f\n', mean_def, pk_def);
fprintf('  Best-R    mean=%.3f peak=%.3f  (gap = %+.3f)\n', mean_best, pk_best, mean_best-mean_def);

% Check H1 dip survives at fixed R
fprintf('\n  H1 dip check at fixed R=0.5 (does success drop as u_max grows past optimum?):\n');
for is = 1:numel(slew_grid)
    [pk, ipk] = max(succ_def(is,:));
    fprintf('    slew=%2d: u*_max=%2g succ=%.2f -> at u_max=%2g succ=%.2f  drop=%+.3f\n', ...
        slew_grid(is), umax_grid(ipk), pk, umax_grid(end), succ_def(is,end), succ_def(is,end)-pk);
end

% Best-R monotonicity
fprintf('\n  Best-R monotonicity (should be non-decreasing in u_max if controller-tuning kills dip):\n');
for is = 1:numel(slew_grid)
    monotone = all(diff(succ_best(is,:)) >= -0.05);
    fprintf('    slew=%2d: %s\n', slew_grid(is), ternary(monotone, 'monotone in u_max', 'NON-monotone (dip survives)'));
end

% CSV outputs
res_dir = fullfile(here, 'results');
if ~exist(res_dir, 'dir'), mkdir(res_dir); end
fid = fopen(fullfile(res_dir, 'realistic_h1_sweep.csv'), 'w');
fprintf(fid, 'slew,u_max,succ_default_R0p5,succ_best_R,R_best\n');
for is = 1:numel(slew_grid)
    for iu = 1:numel(umax_grid)
        fprintf(fid, '%g,%g,%.4f,%.4f,%g\n', slew_grid(is), umax_grid(iu), ...
            succ_def(is,iu), succ_best(is,iu), R_best(is,iu));
    end
end
fclose(fid);
fprintf('\nWrote results/realistic_h1_sweep.csv\n');

summary.succ_def = succ_def;
summary.succ_best = succ_best;
summary.R_best = R_best;
summary.umax_grid = umax_grid;
summary.slew_grid = slew_grid;
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
