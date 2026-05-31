function summary = run_rescue_region_compare()
%RUN_RESCUE_REGION_COMPARE  Compare amateur prescriptive recipe to "proper" methods.
% Within the rescue region (high slew), compare:
%   - FIXED_LQR with default R=0.5 (naive amateur)
%   - FIXED_LQR with R*(u_max) from h1_bestk_kill (our prescriptive recipe)
%   - SIGMA_MRAC (canonical adaptive baseline)
%   - PCH_LQR (constraint-aware baseline, the "right" engineering answer)
% On the realistic plant with p_unstable=8.

here = fileparts(mfilename('fullpath'));
addpath(here);
proj = fileparts(here);
addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src'));
addpath(fullfile(proj, 'ModelRocket_Adaptive_TVC', 'src', 'utils'));

cfg = rocket_defaults();
cfg.plant.p_unstable = 8.0;
cfg.plant.aero_damp  = 0.5;
cfg.plant.control_eff = 8.0;
cfg.plant.tau_act     = 0.05;
cfg.plant.slew_max    = 50.0;     % rescue region (>= threshold)
cfg.t_end_demo        = 3.0;

sc.t_end = 2.5;
sc.fault_time = 999;
sc.kind = 'rescue_compare';
sc.control_eff_scale_post = 1;
sc.aero_damp_scale_post   = 1;
sc.tau_scale_post         = 1;
sc.slew_scale_post        = 1;
sc.disturbance_amp        = 0.0;
sc.disturbance_freq_hz    = 1.0;
sc.disturb_scale_post     = 1;
sc.disturb_bias_post      = 0;

realism = struct('gust_std', 1.5, 'gust_tau', 0.30);

p = cfg.plant.p_unstable;
damp = cfg.plant.aero_damp;
keff = cfg.plant.control_eff;
umax_grid = [4 5 6 8 10 12];
seeds = 1:8;
theta0_set = deg2rad([2 5 8 12]);
theta_fail = deg2rad(40);
trials_per_cell = numel(seeds) * numel(theta0_set);

% R*(u_max) lookup from h1_bestk_kill.csv (validated on realistic plant)
rstar = containers.Map( ...
    {3, 4, 5, 6, 8, 10, 12}, ...
    {0.5, 0.5, 4.0, 4.0, 4.0, 8.0, 8.0});

methods = {'FIXED_LQR_default', 'FIXED_LQR_R_star', 'SIGMA_MRAC', 'PCH_LQR'};
succ = zeros(numel(methods), numel(umax_grid));

fprintf('=== RESCUE REGION COMPARISON (p_unstable=%g, slew=%g, rescue region) ===\n', ...
    p, cfg.plant.slew_max);

for iu = 1:numel(umax_grid)
    cfg.plant.u_max = umax_grid(iu);
    K_default = design_lqr_unstable(p, damp, keff, diag([400 2]), 0.5);
    K_rstar   = design_lqr_unstable(p, damp, keff, diag([400 2]), rstar(umax_grid(iu)));

    for im = 1:numel(methods)
        mname = methods{im};
        cfg_m = cfg;
        switch mname
            case 'FIXED_LQR_default'
                cfg_m.controllers.FIXED_LQR.K = K_default;
                cfg_m.controllers.FIXED_LQR.K_nominal = K_default;
                cfg_m.controllers.FIXED_LQR.u_max = umax_grid(iu);
                ctrl_id = 'FIXED_LQR';
            case 'FIXED_LQR_R_star'
                cfg_m.controllers.FIXED_LQR.K = K_rstar;
                cfg_m.controllers.FIXED_LQR.K_nominal = K_rstar;
                cfg_m.controllers.FIXED_LQR.u_max = umax_grid(iu);
                ctrl_id = 'FIXED_LQR';
            case 'SIGMA_MRAC'
                cfg_m.controllers.SIGMA_MRAC.K_nominal = K_default;
                cfg_m.controllers.SIGMA_MRAC.u_max = umax_grid(iu);
                ctrl_id = 'SIGMA_MRAC';
            case 'PCH_LQR'
                cfg_m.controllers.PCH_LQR.K_nominal = K_default;
                cfg_m.controllers.PCH_LQR.u_max = umax_grid(iu);
                cfg_m.controllers.PCH_LQR.slew_max_assumed = cfg.plant.slew_max;
                cfg_m.controllers.PCH_LQR.tau_act_assumed  = cfg.plant.tau_act;
                ctrl_id = 'PCH_LQR';
        end
        wins = 0;
        for ith = 1:numel(theta0_set)
            cfg_m.plant.theta0 = theta0_set(ith);
            for s = seeds
                seed = s + 100*ith + 7*iu + 31*im;
                out = simulate_case_realistic(ctrl_id, sc, cfg_m, seed, realism);
                if all(abs(out.theta) < theta_fail)
                    wins = wins + 1;
                end
            end
        end
        succ(im, iu) = wins / trials_per_cell;
    end
    fprintf('  u_max=%2g: ', umax_grid(iu));
    for im = 1:numel(methods)
        fprintf(' %s=%.2f', methods{im}, succ(im, iu));
    end
    fprintf('\n');
end

fprintf('\n=== SUMMARY (mean success across u_max) ===\n');
for im = 1:numel(methods)
    fprintf('  %-20s : %.3f\n', methods{im}, mean(succ(im,:)));
end

fid = fopen(fullfile(here, 'results', 'rescue_region_compare.csv'), 'w');
fprintf(fid, 'u_max');
for im = 1:numel(methods), fprintf(fid, ',%s', methods{im}); end
fprintf(fid, '\n');
for iu = 1:numel(umax_grid)
    fprintf(fid, '%g', umax_grid(iu));
    for im = 1:numel(methods), fprintf(fid, ',%.4f', succ(im, iu)); end
    fprintf(fid, '\n');
end
fclose(fid);
fprintf('Wrote results/rescue_region_compare.csv\n');

summary.methods = methods;
summary.umax_grid = umax_grid;
summary.succ = succ;
end
