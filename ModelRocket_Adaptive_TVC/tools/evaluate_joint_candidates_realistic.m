function evaluate_joint_candidates_realistic(root_dir)
%EVALUATE_JOINT_CANDIDATES_REALISTIC Targeted JOINT tuning checks.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

out_dir = fullfile(root_dir, 'outputs', 'data');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

cfg0 = rocket_defaults();
seeds = 1:3;
amps = [1.0 2.0 3.0 4.0];

% [freeze_thr, gate_disturb_gain, shield_frac, slew_scale_min]
candidates = [
    0.10 0.50 20 0.05;  % current-ish tuned for keff
    0.70 0.00 10 0.10;  % conservative slew-protective
    0.70 0.25 10 0.10;
    0.50 0.25 20 0.10;
    0.35 0.25 20 0.10;
    0.50 0.00 20 0.10;
    0.35 0.00 20 0.10;
    0.20 0.25 20 0.10
];

rows = [];
for i = 1:size(candidates,1)
    fr = candidates(i,1);
    gd = candidates(i,2);
    sh = candidates(i,3);
    sm = candidates(i,4);

    cfg = cfg0;
    cfg.controllers.JOINT_ADAPTIVE.slew_health_keff_freeze = fr;
    cfg.controllers.JOINT_ADAPTIVE.gate_disturb_gain = gd;
    cfg.controllers.JOINT_ADAPTIVE.safety_cmd_slew_frac = sh;
    cfg.controllers.JOINT_ADAPTIVE.slew_scale_min = sm;

    slew_vals = [];
    keff_vals = [];
    worst = 0;

    for amp = amps
        scs = rocket_scenario("SLEW_DEGRADATION", cfg);
        scs.t_end = 10.0;
        scs.disturbance_amp = amp;
        scs.disturbance_freq_hz = 1.5;

        sck = rocket_scenario("HIGH_KEFF_FAULT", cfg);
        sck.t_end = 10.0;
        sck.disturbance_amp = 0.30;
        sck.disturb_scale_post = 4.0 * amp;

        for sd = seeds
            outs = simulate_case_realistic("JOINT_ADAPTIVE", scs, cfg, sd, struct());
            rs = rad2deg(rms(outs.theta(outs.time >= scs.fault_time)));
            slew_vals(end+1) = rs; %#ok<AGROW>

            outk = simulate_case_realistic("JOINT_ADAPTIVE", sck, cfg, sd, struct());
            rk = rad2deg(rms(outk.theta(outk.time >= sck.fault_time)));
            keff_vals(end+1) = rk; %#ok<AGROW>

            worst = max([worst, rs, rk]);
        end
    end

    m_slew = mean(slew_vals, 'omitnan');
    m_keff = mean(keff_vals, 'omitnan');
    p95_slew = prctile(slew_vals, 95);
    p95_keff = prctile(keff_vals, 95);

    score = mean(log10(1 + slew_vals), 'omitnan') + mean(log10(1 + keff_vals), 'omitnan');
    if worst > 100
        score = score + 2.0;
    elseif worst > 50
        score = score + 1.0;
    end

    rows(end+1,:) = [i, fr, gd, sh, sm, m_slew, m_keff, p95_slew, p95_keff, worst, score]; %#ok<AGROW>
    fprintf('Candidate %d done: score=%.3f, mean_slew=%.2f, mean_keff=%.2f, worst=%.2f\n', ...
        i, score, m_slew, m_keff, worst);
end

T = array2table(rows, 'VariableNames', ...
    {'idx','freeze_thr','disturb_gate','shield_frac','slew_scale_min', ...
     'mean_rms_slew','mean_rms_keff','p95_rms_slew','p95_rms_keff','worst_rms','score'});
T = sortrows(T, 'score');

csv_path = fullfile(out_dir, 'joint_candidate_eval_realistic.csv');
writetable(T, csv_path);

fprintf('\nSaved: %s\n', csv_path);
fprintf('\nRanked candidates:\n');
disp(T);
end
