function run_joint_decoupling_check(root_dir)
%RUN_JOINT_DECOUPLING_CHECK Targeted check for joint-controller cross-coupling.
%
% Compares controllers at fixed operating points to verify:
% 1) joint ~= keff-aware in pure keff-fault regime
% 2) joint ~= slew-aware in pure slew-fault regime
%
% Writes:
%   outputs/data/joint_decoupling_check.csv

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

data_dir = fullfile(root_dir, 'outputs', 'data');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end

cfg0 = rocket_defaults();
seeds = 1:8;

% Focus points: the reported problematic operating point and a matched slew case.
cases = [
    struct('name',"keff_amp2", 'kind',"HIGH_KEFF_FAULT", 'amp',2.0)
    struct('name',"slew_amp2", 'kind',"SLEW_DEGRADATION", 'amp',2.0)
];

rows = [];
for ci = 1:numel(cases)
    cdef = cases(ci);

    % Scenario setup
    sc = rocket_scenario(cdef.kind, cfg0);
    sc.t_end = 10.0;
    if cdef.kind == "SLEW_DEGRADATION"
        sc.disturbance_amp = cdef.amp;
        sc.disturbance_freq_hz = 1.5;
    else
        sc.disturbance_amp = 0.30;
        sc.disturb_scale_post = 4.0 * cdef.amp;
    end

    % Variants
    variants = {
        struct('name',"ADAPTIVE_KEFF_LQR", 'ctrl',"ADAPTIVE_KEFF_LQR", 'cfg',cfg0)
        struct('name',"JOINT_default", 'ctrl',"JOINT_ADAPTIVE", 'cfg',cfg0)
        struct('name',"JOINT_no_pos_guard", 'ctrl',"JOINT_ADAPTIVE", 'cfg',set_no_pos_guard(cfg0))
        struct('name',"JOINT_disturb_gate", 'ctrl',"JOINT_ADAPTIVE", 'cfg',set_disturb_gate(cfg0))
        struct('name',"JOINT_no_keff_freeze", 'ctrl',"JOINT_ADAPTIVE", 'cfg',set_no_keff_freeze(cfg0))
        struct('name',"JOINT_no_shield", 'ctrl',"JOINT_ADAPTIVE", 'cfg',set_no_shield(cfg0))
        struct('name',"JOINT_no_freeze_no_shield", 'ctrl',"JOINT_ADAPTIVE", 'cfg',set_no_freeze_no_shield(cfg0))
        struct('name',"JOINT_tuned_candidate", 'ctrl',"JOINT_ADAPTIVE", 'cfg',set_tuned_candidate(cfg0))
    };

    for vi = 1:numel(variants)
        v = variants{vi};
        for seed = seeds
            out = simulate_case(v.ctrl, sc, v.cfg, seed);
            pm = out.time >= sc.fault_time;
            rms_post = rad2deg(rms(out.theta(pm)));
            peak_post = rad2deg(max(abs(out.theta(pm))));

            pos_limited_frac = NaN;
            if isfield(out, 'pos_limited')
                pos_limited_frac = mean(out.pos_limited(pm), 'omitnan');
            end

            rows(end+1,:) = [ci, vi, seed, cdef.amp, rms_post, peak_post, pos_limited_frac]; %#ok<AGROW>
        end
    end
end

T = array2table(rows, 'VariableNames', ...
    {'case_idx','variant_idx','seed','disturb_amp','rms_post_deg','peak_post_deg','pos_limited_frac'});

case_names = strings(height(T),1);
variant_names = strings(height(T),1);
for i = 1:height(T)
    case_names(i) = cases(T.case_idx(i)).name;
    variant_names(i) = variants{T.variant_idx(i)}.name;
end
T.case_name = case_names;
T.variant_name = variant_names;
T = movevars(T, {'case_name','variant_name'}, 'Before', 'case_idx');

csv_path = fullfile(data_dir, 'joint_decoupling_check.csv');
writetable(T, csv_path);
fprintf('Saved: %s\n', csv_path);

fprintf('\n=== JOINT DECOUPLING CHECK (mean RMS post-fault, deg) ===\n');
for ci = 1:numel(cases)
    fprintf('--- %s ---\n', cases(ci).name);
    for vi = 1:numel(variants)
        m = T.case_idx == ci & T.variant_idx == vi;
        vals = T.rms_post_deg(m);
        fprintf('  %-20s  %7.3f\n', char(unique(T.variant_name(m))), mean(vals, 'omitnan'));
    end
end
end


function cfg = set_no_pos_guard(cfg)
cfg.controllers.JOINT_ADAPTIVE.slew_detect_pos_sat_frac = 2.0;
end

function cfg = set_disturb_gate(cfg)
cfg.controllers.JOINT_ADAPTIVE.gate_disturb_gain = 0.75;
end

function cfg = set_no_keff_freeze(cfg)
cfg.controllers.JOINT_ADAPTIVE.slew_health_keff_freeze = 0.0;
end

function cfg = set_no_shield(cfg)
cfg.controllers.JOINT_ADAPTIVE.safety_cmd_slew_frac = 1e6;
end

function cfg = set_no_freeze_no_shield(cfg)
cfg = set_no_keff_freeze(cfg);
cfg = set_no_shield(cfg);
end

function cfg = set_tuned_candidate(cfg)
% Less aggressive freeze and less restrictive shield for authority-loss cases,
% while retaining both mechanisms.
cfg.controllers.JOINT_ADAPTIVE.slew_health_keff_freeze = 0.40;
cfg.controllers.JOINT_ADAPTIVE.safety_cmd_slew_frac = 40.0;
cfg.controllers.JOINT_ADAPTIVE.gate_disturb_gain = 0.75;
end
