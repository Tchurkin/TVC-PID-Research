%% tune_params_C.m
% Grid search for Direction C monitor and retuner parameters.
% Saves best parameters to outputs/tuned_params_C.mat.

this_dir = fileparts(mfilename('fullpath'));
dir_root = fileparts(this_dir);
out_dir  = fullfile(dir_root, 'outputs');
if ~exist(out_dir,'dir'), mkdir(out_dir); end
addpath(this_dir);

% Shared plant/actuator
b0 = 112.5; c_damp = 1.0;
delta_max = 10*pi/180; slew_max = 92*pi/180;
wn_nom = 36.0; zeta_a = 0.82; delay_nom = 0.032;
wn_fault = 16.0; delay_fault = 0.080;
dt = 0.001; t_end = 8.0; N = round(t_end/dt);
fault_times = [1.0, 3.0, 5.0];

% Aggressive nominal controller to expose fault sensitivity
wn_cl = 7.0; zeta_cl = 0.75;
gains_nom.Kp = wn_cl^2 / b0;
gains_nom.Kd = (2*zeta_cl*wn_cl - c_damp) / b0;
gains_nom.Ki = 0.8 / b0;

% Grid
alpha_grid = [0.9, 1.2];
tau_smooth_grid = [0.010, 0.022];
delta_safe_deg_grid = [2.5, 3.0, 4.0];
trigger_margin_grid = [0.020, 0.030, 0.045];
kp_exp_grid = [0.9, 1.0];
kd_exp_grid = [0.60, 0.85];
ki_exp_grid = [2.0, 2.6];
probe_amp_deg_grid = [0.6, 1.0];
probe_len_s_grid = [0.08, 0.14];

best = struct('score', inf);
total_cases = numel(alpha_grid) * numel(tau_smooth_grid) * numel(delta_safe_deg_grid) * ...
    numel(trigger_margin_grid) * numel(kp_exp_grid) * numel(kd_exp_grid) * numel(ki_exp_grid) * ...
    numel(probe_amp_deg_grid) * numel(probe_len_s_grid);
case_id = 0;

for alpha = alpha_grid
for tau_smooth = tau_smooth_grid
for delta_safe_deg = delta_safe_deg_grid
for trigger_margin = trigger_margin_grid
for kp_exp = kp_exp_grid
for kd_exp = kd_exp_grid
for ki_exp = ki_exp_grid
for probe_amp_deg = probe_amp_deg_grid
for probe_len_s = probe_len_s_grid
    case_id = case_id + 1;
    ret_params.wn_nominal = wn_nom;
    ret_params.alpha = alpha;
    ret_params.tau_smooth = tau_smooth;
    ret_params.kp_exp = kp_exp;
    ret_params.kd_exp = kd_exp;
    ret_params.ki_exp = ki_exp;
    ret_params.min_scale = 0.18;
    ret_params.probe_amp_deg = probe_amp_deg;
    ret_params.probe_len_s = probe_len_s;
    delta_safe_max = delta_safe_deg * pi/180;

    mon_win = round(0.20/dt);
    mon_params.wn_nominal = wn_nom;
    mon_params.wn_threshold = 0.78 * wn_nom;
    mon_params.sensitivity = 4.0;
    mon_params.e_norm_floor = 0.08;
    mon_params.startup_guard_s = 0.5;
    mon_params.arm_time_s = 0.9;
    mon_params.trigger_margin = trigger_margin;
    mon_params.min_excitation_var = (0.001)^2;
    mon_params.N_confirm = round(0.05/dt);
    mon_params.tau_e_norm = 0.12;
    mon_params.tau_wn_est = 0.30;
    mon_params.wn_rate_limit = 60.0;

    max_no = zeros(1,3); max_yes = zeros(1,3);
    lat_yes = nan(1,3); wn_jump = zeros(1,3); fp_yes = zeros(1,3);

    for fi = 1:3
        cfg = struct('b0',b0,'c_damp',c_damp,'delta_max',delta_max,'slew_max',slew_max, ...
            'wn_nom',wn_nom,'zeta_a',zeta_a,'delay_nom',delay_nom,'wn_fault',wn_fault, ...
            'delay_fault',delay_fault,'dt',dt,'N',N,'gains_nom',gains_nom,'mon_win',mon_win, ...
            'mon_params',mon_params,'ret_params',ret_params,'delta_safe_max',delta_safe_max);
        [max_no(fi),~,~,~,~] = run_case(false, fault_times(fi), cfg);
        [max_yes(fi),lat_yes(fi),~,fp_yes(fi),wn_jump(fi)] = run_case(true, fault_times(fi), cfg);
    end

    % Objective: minimize adapted peak error, keep fast detection, smooth wn estimate
    peak_term = mean(max_yes) / max(1e-6, mean(max_no));
    lat_term = mean(min(0.5, max(0, lat_yes))) / 0.5;
    jump_term = mean(wn_jump) / 10.0;   % normalized by 10 rad/s jump
    false_term = min(1.0, mean(fp_yes));
    score = 0.50*peak_term + 0.30*lat_term + 0.12*jump_term + 0.08*false_term;

    if score < best.score
        best.score = score;
        best.alpha = alpha;
        best.tau_smooth = tau_smooth;
        best.delta_safe_deg = delta_safe_deg;
        best.trigger_margin = trigger_margin;
        best.kp_exp = kp_exp;
        best.kd_exp = kd_exp;
        best.ki_exp = ki_exp;
        best.probe_amp_deg = probe_amp_deg;
        best.probe_len_s = probe_len_s;
        best.max_no = max_no;
        best.max_yes = max_yes;
        best.lat_yes = lat_yes;
        best.wn_jump = wn_jump;
        fprintf('  New best [%d/%d] score=%.4f alpha=%.2f kpE=%.2f kdE=%.2f kiE=%.2f probe=%.1fdeg/%.2fs dsafe=%.1f\n', ...
            case_id, total_cases, score, alpha, kp_exp, kd_exp, ki_exp, probe_amp_deg, probe_len_s, delta_safe_deg);
    end
end
end
end
end
end
end
end
end
end

save(fullfile(out_dir,'tuned_params_C.mat'),'best');
fid = fopen(fullfile(out_dir,'tuned_params_C.csv'),'w');
fprintf(fid,'param,value\n');
fprintf(fid,'score,%.6f\n',best.score);
fprintf(fid,'alpha,%.4f\n',best.alpha);
fprintf(fid,'tau_smooth,%.4f\n',best.tau_smooth);
fprintf(fid,'delta_safe_deg,%.4f\n',best.delta_safe_deg);
fprintf(fid,'trigger_margin,%.4f\n',best.trigger_margin);
fprintf(fid,'kp_exp,%.4f\n',best.kp_exp);
fprintf(fid,'kd_exp,%.4f\n',best.kd_exp);
fprintf(fid,'ki_exp,%.4f\n',best.ki_exp);
fprintf(fid,'probe_amp_deg,%.4f\n',best.probe_amp_deg);
fprintf(fid,'probe_len_s,%.4f\n',best.probe_len_s);
fclose(fid);

fprintf('Direction C tuning complete. Best score=%.4f\n',best.score);

function [max_err_deg, detect_latency, settled_rms_deg, false_trig_count, max_wn_jump] = run_case(adapt, t_fault, cfg)
    theta=0; omega=0; delta_act=0; delta_vel=0; int_err=0;
    u_slew=0; gains=cfg.gains_nom;
    wn_cur=cfg.wn_nom; delay_cur=cfg.delay_nom;
    delay_buf = zeros(max(1, round(delay_cur/cfg.dt)),1);

    mon_state.cmd_buf = zeros(cfg.mon_win,1);
    mon_state.meas_buf = zeros(cfg.mon_win,1);
    mon_state.t_elapsed = 0;
    mon_state.confirm_count = 0;
    mon_state.wn_est = cfg.wn_nom;
    mon_state.e_ref = cfg.mon_params.e_norm_floor;
    mon_state.fault_latched = 0;
    mon_state.e_norm_lp = cfg.mon_params.e_norm_floor;
    mon_state.wn_lp = cfg.wn_nom;

    theta_log=zeros(cfg.N,1); r_log=zeros(cfg.N,1); wn_log=zeros(cfg.N,1);
    k_fault = round(t_fault/cfg.dt);
    k_detect = NaN;
    false_trig_count = 0;
    r_cmd = 0;
    r_rate_max_nom = 35*pi/180;
    r_rate_max_safe = 8*pi/180;
    lag_confirm = 0;
    lag_trigger = 0.6*pi/180;
    lag_N_confirm = round(0.015/cfg.dt);
    probe_count = 0;
    probe_len = max(1, round(cfg.ret_params.probe_len_s/cfg.dt));
    probe_amp = cfg.ret_params.probe_amp_deg*pi/180;
    probe_freq = 12.0;
    lag_probe_start = max(2, round(0.40*lag_N_confirm));

    prev_r_target = 0;
    for k=1:cfg.N
        t=(k-1)*cfg.dt;
        if t < 0.6
            r_target = 0;
        elseif t < 2.2
            r_target = 8*pi/180;
        elseif t < 3.8
            r_target = -8*pi/180;
        elseif t < 5.8
            r_target = 10*pi/180;
        else
            r_target = 0;
        end
        if adapt && mon_state.fault_latched
            r_rate_max = r_rate_max_safe;
        else
            r_rate_max = r_rate_max_nom;
        end
        dr_lim = r_rate_max * cfg.dt;
        r_cmd = r_cmd + max(-dr_lim, min(dr_lim, r_target - r_cmd));
        r = r_cmd;

        if k==k_fault
            wn_cur = cfg.wn_fault;
            delay_cur = cfg.delay_fault;
            N_new = max(1, round(delay_cur/cfg.dt));
            if N_new > length(delay_buf)
                delay_buf = [delay_buf; delay_buf(end)*ones(N_new-length(delay_buf),1)];
            else
                delay_buf = delay_buf(1:N_new);
            end
        end

        err = r - theta;
        u_unsat = gains.Kp*err + gains.Kd*(-omega) + gains.Ki*int_err;

        transition_probe = (abs(r_target) > 1e-9) && (abs(prev_r_target) > 1e-9) && (sign(r_target) ~= sign(prev_r_target));
        if adapt && (~mon_state.fault_latched) && (mon_state.t_elapsed > cfg.mon_params.arm_time_s)
            probe_suspect = (lag_confirm >= lag_probe_start) || ...
                (mon_state.e_norm_lp > mon_state.e_ref + 0.5*cfg.mon_params.trigger_margin);
            if (probe_suspect || transition_probe) && probe_count == 0
                probe_count = probe_len;
            end
        end
        if probe_count > 0
            u_unsat = u_unsat + probe_amp * sin(2*pi*probe_freq*t);
            probe_count = probe_count - 1;
        end
        if adapt && mon_state.fault_latched
            u_limit = cfg.delta_safe_max;
        else
            u_limit = cfg.delta_max;
        end
        u_raw = max(-u_limit, min(u_limit, u_unsat));

        is_sat = abs(u_unsat - u_raw) > 1e-12;
        if (~is_sat) || (sign(err) ~= sign(u_unsat - u_raw))
            int_err = int_err + err*cfg.dt;
        end

        du = max(-cfg.slew_max*cfg.dt, min(cfg.slew_max*cfg.dt, u_raw-u_slew));
        u_slew = u_slew + du;
        delay_buf = [u_slew; delay_buf(1:end-1)];
        u_del = delay_buf(end);

        dv = wn_cur^2*(u_del-delta_act) - 2*cfg.zeta_a*wn_cur*delta_vel;
        delta_vel = delta_vel + cfg.dt*dv;
        delta_act = delta_act + cfg.dt*delta_vel;
        delta = max(-cfg.delta_max, min(cfg.delta_max, delta_act));

        omega = omega + cfg.dt*(cfg.b0*delta - cfg.c_damp*omega);
        theta = theta + cfg.dt*omega;

        [wn_est_k, flag_k, mon_state] = actuator_health_monitor(u_slew, delta_act, mon_state, cfg.mon_params, cfg.dt);
        lag_err = abs(u_slew - delta_act);
        if (mon_state.t_elapsed > cfg.mon_params.arm_time_s) && (lag_err > lag_trigger) && (abs(u_slew) > 0.5*pi/180)
            lag_confirm = lag_confirm + 1;
        else
            lag_confirm = max(0, lag_confirm - 1);
        end
        if lag_confirm >= lag_N_confirm
            flag_k = 1;
            mon_state.fault_latched = 1;
        end
        if flag_k && k < k_fault
            false_trig_count = false_trig_count + 1;
        end
        if adapt && flag_k && (k>=k_fault)
            if isnan(k_detect)
                k_detect = k;
                int_err = 0;
            end
            gains = adaptive_retune(wn_est_k, cfg.gains_nom, cfg.ret_params, gains, cfg.dt);
        end

        theta_log(k)=theta; r_log(k)=r; wn_log(k)=wn_est_k;
        prev_r_target = r_target;
    end

    err_log = theta_log - r_log;
    k1 = k_fault; k2 = min(cfg.N, k_fault + round(4.0/cfg.dt));
    max_err_deg = max(abs(err_log(k1:k2)))*180/pi;
    settled_rms_deg = sqrt(mean(err_log(min(cfg.N,k2-round(1/cfg.dt)):k2).^2))*180/pi;
    if isnan(k_detect)
        detect_latency = NaN;
    else
        detect_latency = (k_detect-k_fault)*cfg.dt;
    end

    dwn = abs(diff(wn_log));
    max_wn_jump = max(dwn);
end
