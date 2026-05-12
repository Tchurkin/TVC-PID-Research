%% tune_params_B.m
% Grid search for Direction B hybrid ADRC/L1 parameters.
% Saves best parameters to outputs/tuned_params_B.mat.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
out_dir  = fullfile(dir_root, 'outputs');
if ~exist(out_dir,'dir'), mkdir(out_dir); end
addpath(this_dir);

wind_cases = {'low','medium','high'};
SEED = 77;
dt = 0.010; t_end = 60.0;

% L1 baseline
l1_params.L1_period = 20.0;
l1_params.u_max = 4.0;

% Grid
omega_c_grid = [0.35 0.40 0.45];
omega_o_grid = [1.4 1.6 1.8];
k_adrc_grid  = [0.50 0.60 0.70];
du_max_grid  = [25 35 45];

best = struct('score', inf);

for omega_c = omega_c_grid
for omega_o = omega_o_grid
for k_adrc = k_adrc_grid
for du_max = du_max_grid
    adrc.b0 = 1.0;
    adrc.Kp = omega_c^2;
    adrc.Kd = 2*omega_c;
    adrc.l1 = 3*omega_o;
    adrc.l2 = 3*omega_o^2;
    adrc.l3 = omega_o^3;
    adrc.u_max = l1_params.u_max;

    blend.k_adrc = k_adrc;
    blend.du_max = du_max;

    peak_l1 = zeros(1,3); var_l1 = zeros(1,3);
    peak_h  = zeros(1,3); var_h  = zeros(1,3);

    for wi = 1:3
        sc = crosswind_scenario(wind_cases{wi}, dt, t_end, SEED+wi);
        [~, peak_l1(wi), var_l1(wi)] = run_case(false, sc, dt, l1_params, adrc, blend);
        [~, peak_h(wi),  var_h(wi)]  = run_case(true,  sc, dt, l1_params, adrc, blend);
    end

    peak_ratio = peak_h ./ max(1e-6, peak_l1);
    var_ratio  = var_h  ./ max(1e-6, var_l1);

    % Weighted objective: prioritize peak error, penalize excessive effort
    score = 0.6*(0.2*peak_ratio(1) + 0.4*peak_ratio(2) + 0.4*peak_ratio(3)) + ...
            0.4*(0.2*var_ratio(1)  + 0.4*var_ratio(2)  + 0.4*var_ratio(3));

    if score < best.score
        best.score = score;
        best.omega_c = omega_c;
        best.omega_o = omega_o;
        best.k_adrc = k_adrc;
        best.du_max = du_max;
        best.peak_ratio = peak_ratio;
        best.var_ratio = var_ratio;
    end
end
end
end
end

save(fullfile(out_dir,'tuned_params_B.mat'),'best');
fid = fopen(fullfile(out_dir,'tuned_params_B.csv'),'w');
fprintf(fid,'param,value\n');
fprintf(fid,'score,%.6f\n',best.score);
fprintf(fid,'omega_c,%.4f\n',best.omega_c);
fprintf(fid,'omega_o,%.4f\n',best.omega_o);
fprintf(fid,'k_adrc,%.4f\n',best.k_adrc);
fprintf(fid,'du_max,%.4f\n',best.du_max);
fclose(fid);

fprintf('Direction B tuning complete. Best score=%.4f\n', best.score);

function [ct_rms, ct_peak, u_var] = run_case(use_hybrid, sc, dt, l1_params, adrc, blend)
    N = sc.N;
    y_ct = 0; vy_ct = 0; u_actual = 0;
    z_adrc = zeros(3,1); u_prev = 0; u_cmd_prev = 0;
    y_log = zeros(N,1); u_log = zeros(N,1);

    for k=1:N
        if ~use_hybrid
            [u_cmd,~] = l1_guidance(y_ct, vy_ct, sc.V, l1_params);
        else
            [u_l1,~] = l1_guidance(y_ct, vy_ct, sc.V, l1_params);
            [u_adrc, z_adrc] = adrc_layer(y_ct, z_adrc, u_prev, adrc, dt);
            u_mix = (1-blend.k_adrc)*u_l1 + blend.k_adrc*u_adrc;
            du_lim = blend.du_max * dt;
            u_cmd = u_cmd_prev + max(-du_lim, min(du_lim, u_mix - u_cmd_prev));
        end

        u_prev = u_cmd;
        u_cmd_prev = u_cmd;

        u_actual = u_actual + dt*(u_cmd-u_actual)/sc.tau_act;
        u_actual = max(-sc.u_max, min(sc.u_max, u_actual));

        vy_ct = vy_ct + dt*u_actual;
        vy_ct = max(-20, min(20, vy_ct));
        y_ct = y_ct + dt*(vy_ct + sc.w_wind(k));

        y_log(k) = y_ct;
        u_log(k) = u_actual;
    end

    k_start = round(5.0/dt);
    ct_rms = sqrt(mean(y_log(k_start:end).^2));
    ct_peak = max(abs(y_log(k_start:end)));
    u_var = var(u_log(k_start:end));
end
