function search_joint_params_robust_realistic(root_dir)
%SEARCH_JOINT_PARAMS_ROBUST_REALISTIC Tune JOINT over both fault families.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

data_dir = fullfile(root_dir, 'outputs', 'data');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end

cfg0 = rocket_defaults();
seeds = 1:4;
amps = [1.0 2.0 3.0 4.0];

freeze_grid = [0.20 0.35 0.50 0.70];
gate_grid   = [0.0 0.25 0.50];
shield_grid = [10 20 40];
scale_min_grid = [0.05 0.10];

rows = [];
for fr = freeze_grid
    for gd = gate_grid
        for sh = shield_grid
            for sm = scale_min_grid
                cfg = cfg0;
                cfg.controllers.JOINT_ADAPTIVE.slew_health_keff_freeze = fr;
                cfg.controllers.JOINT_ADAPTIVE.gate_disturb_gain = gd;
                cfg.controllers.JOINT_ADAPTIVE.safety_cmd_slew_frac = sh;
                cfg.controllers.JOINT_ADAPTIVE.slew_scale_min = sm;

                rms_slew = [];
                rms_keff = [];
                worst = 0;

                for amp = amps
                    % Slew scenario
                    scs = rocket_scenario("SLEW_DEGRADATION", cfg);
                    scs.t_end = 10.0;
                    scs.disturbance_amp = amp;
                    scs.disturbance_freq_hz = 1.5;

                    % keff scenario
                    sck = rocket_scenario("HIGH_KEFF_FAULT", cfg);
                    sck.t_end = 10.0;
                    sck.disturbance_amp = 0.30;
                    sck.disturb_scale_post = 4.0 * amp;

                    for sd = seeds
                        outs = simulate_case_realistic("JOINT_ADAPTIVE", scs, cfg, sd, struct());
                        pms = outs.time >= scs.fault_time;
                        rs = rad2deg(rms(outs.theta(pms)));
                        rms_slew(end+1) = rs; %#ok<AGROW>

                        outk = simulate_case_realistic("JOINT_ADAPTIVE", sck, cfg, sd, struct());
                        pmk = outk.time >= sck.fault_time;
                        rk = rad2deg(rms(outk.theta(pmk)));
                        rms_keff(end+1) = rk; %#ok<AGROW>

                        worst = max([worst, rs, rk]);
                    end
                end

                m_slew = mean(rms_slew, 'omitnan');
                m_keff = mean(rms_keff, 'omitnan');

                % Objective balances both regimes and strongly penalizes blow-ups.
                obj = mean(log10(1 + rms_slew), 'omitnan') + mean(log10(1 + rms_keff), 'omitnan');
                if worst > 100
                    obj = obj + 2.0;
                elseif worst > 50
                    obj = obj + 1.0;
                end

                rows(end+1,:) = [fr, gd, sh, sm, m_slew, m_keff, worst, obj]; %#ok<AGROW>
            end
        end
    end
end

T = array2table(rows, 'VariableNames', ...
    {'freeze_thr','disturb_gate','shield_frac','slew_scale_min','mean_rms_slew','mean_rms_keff','worst_rms','objective'});
T = sortrows(T, 'objective');

csv_path = fullfile(data_dir, 'joint_param_search_robust_realistic.csv');
writetable(T, csv_path);

fprintf('Saved: %s\n', csv_path);
fprintf('\nTop 12 candidates (lower objective better):\n');
disp(T(1:min(12,height(T)),:));
end
