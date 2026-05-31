function quick_slew_drop_sweep(root_dir)
%QUICK_SLEW_DROP_SWEEP Evaluate moderate slew degradation levels.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

cfg = rocket_defaults();
seeds = 1:8;

% Post-fault slew scales (1.0 means no degradation).
% 0.9 = 10%% drop, 0.5 = 50%% drop.
slew_scales = [0.90 0.80 0.70 0.60 0.50];
ctrls = ["FIXED_LQR", "ADAPTIVE_KEFF_LQR", "SLEW_ADAPTIVE", "JOINT_ADAPTIVE"];
pretty = ["Fixed LQR", "keff-only", "slew-aware", "Joint Adaptive"];

rows = [];

fprintf('Slew degradation sweep (realistic model, 8 seeds)\n');
fprintf('Scale  Drop%%   Fixed    keff-only   slew-aware   Joint\n');

for ss = slew_scales
    vals = nan(numel(ctrls), numel(seeds));
    for si = 1:numel(seeds)
        seed = seeds(si);

        sc = rocket_scenario("SLEW_DEGRADATION", cfg);
        sc.t_end = 10.0;
        sc.slew_scale_post = ss;
        % Keep disturbance consistent with robustness campaign style.
        sc.disturbance_amp = 1.5;
        sc.disturbance_freq_hz = 1.5;

        for ci = 1:numel(ctrls)
            out = simulate_case_realistic(ctrls(ci), sc, cfg, seed, struct());
            pm = out.time >= sc.fault_time;
            vals(ci, si) = rad2deg(rms(out.theta(pm)));
        end
    end

    m = mean(vals, 2, 'omitnan');
    s = std(vals, 0, 2, 'omitnan');
    for ci = 1:numel(ctrls)
        rows(end+1,:) = [ss, (1 - ss) * 100, ci, m(ci), s(ci)]; %#ok<AGROW>
    end

    fprintf('%4.2f   %5.1f   %7.2f   %9.2f   %10.2f   %7.2f\n', ...
        ss, (1 - ss) * 100, m(1), m(2), m(3), m(4));
end

T = array2table(rows, 'VariableNames', ...
    {'slew_scale_post','slew_drop_pct','ctrl_idx','mean_rms_deg','std_rms_deg'});
T.controller = strings(height(T),1);
for i = 1:height(T)
    T.controller(i) = pretty(T.ctrl_idx(i));
end

data_dir = fullfile(root_dir, 'outputs', 'data');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end
csv_path = fullfile(data_dir, 'slew_drop_sweep_10_to_50.csv');
writetable(T, csv_path);

fprintf('\nSaved data: %s\n', csv_path);
end
