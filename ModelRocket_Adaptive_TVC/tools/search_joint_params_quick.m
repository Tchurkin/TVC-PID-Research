function search_joint_params_quick(root_dir)
%SEARCH_JOINT_PARAMS_QUICK Coarse search for JOINT settings at key regimes.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

data_dir = fullfile(root_dir, 'outputs', 'data');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end

cfg0 = rocket_defaults();
seeds = 1:4;

freeze_grid = [0.0 0.1 0.2 0.4 0.7];
shield_grid = [10 20 40 100 1e6];
gate_grid   = [0.0 0.5];

rows = [];
for f = freeze_grid
    for s = shield_grid
        for g = gate_grid
            cfg = cfg0;
            cfg.controllers.JOINT_ADAPTIVE.slew_health_keff_freeze = f;
            cfg.controllers.JOINT_ADAPTIVE.safety_cmd_slew_frac = s;
            cfg.controllers.JOINT_ADAPTIVE.gate_disturb_gain = g;

            % keff case at amp=2
            sc_k = rocket_scenario("HIGH_KEFF_FAULT", cfg);
            sc_k.t_end = 10.0;
            sc_k.disturbance_amp = 0.30;
            sc_k.disturb_scale_post = 8.0;

            % slew case at amp=2
            sc_s = rocket_scenario("SLEW_DEGRADATION", cfg);
            sc_s.t_end = 10.0;
            sc_s.disturbance_amp = 2.0;
            sc_s.disturbance_freq_hz = 1.5;

            rms_k = nan(numel(seeds),1);
            rms_s = nan(numel(seeds),1);
            for i = 1:numel(seeds)
                outk = simulate_case("JOINT_ADAPTIVE", sc_k, cfg, seeds(i));
                pmk = outk.time >= sc_k.fault_time;
                rms_k(i) = rad2deg(rms(outk.theta(pmk)));

                outs = simulate_case("JOINT_ADAPTIVE", sc_s, cfg, seeds(i));
                pms = outs.time >= sc_s.fault_time;
                rms_s(i) = rad2deg(rms(outs.theta(pms)));
            end

            mk = mean(rms_k, 'omitnan');
            ms = mean(rms_s, 'omitnan');
            obj = mk + 5.0 * ms; % keep keff performance priority while penalizing slew degradation
            rows(end+1,:) = [f, s, g, mk, ms, obj]; %#ok<AGROW>
        end
    end
end

T = array2table(rows, 'VariableNames', ...
    {'freeze_thr','shield_frac','disturb_gate','mean_rms_keff_amp2','mean_rms_slew_amp2','objective'});
T = sortrows(T, 'objective');

csv_path = fullfile(data_dir, 'joint_param_search_quick.csv');
writetable(T, csv_path);

fprintf('Saved: %s\n', csv_path);
fprintf('\nTop 10 candidates (lower objective is better):\n');
disp(T(1:min(10,height(T)),:));
end
