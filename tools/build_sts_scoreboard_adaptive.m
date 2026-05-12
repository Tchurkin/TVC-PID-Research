%% build_sts_scoreboard_adaptive.m
% Build STS scoreboard for adaptive ADRC direction.

root_dir = fileparts(fileparts(mfilename('fullpath')));
out_dir = fullfile(root_dir, 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

stress_file = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'industry_stress_metrics_adaptive.csv');
deg_file = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'degradation_results_adaptive.csv');

if ~exist(stress_file, 'file')
    error('Missing adaptive stress metrics: %s', stress_file);
end
if ~exist(deg_file, 'file')
    error('Missing adaptive degradation metrics: %s', deg_file);
end

B = readtable(stress_file, 'TextType', 'string');
D = readtable(deg_file, 'TextType', 'string');

high_winds = ["10mps", "12mps", "15mps"];
mask_h = ismember(B.wind_label, high_winds);

peak_ratio_pid = mean(B.peak_ratio_vs_pid(mask_h));
rms_ratio_pid = mean(B.rms_ratio_vs_pid(mask_h));
fail_red_pid = mean(B.fail_reduction_vs_pid(mask_h));
in5_gain_pid = mean(B.in5_gain_vs_pid_pct(mask_h));

score_peak = clamp01(1 - peak_ratio_pid);
score_rms = clamp01(1 - rms_ratio_pid);
score_fail = clamp01(fail_red_pid);
score_in5 = clamp01(in5_gain_pid / 60.0);
B_score = 100 * (0.45 * score_peak + 0.15 * score_rms + 0.25 * score_fail + 0.15 * score_in5);

% Adaptive-specific score from degradation and identification quality.
row_fix = D(strcmp(D.controller, "ADRC_FIXED"), :);
row_ad = D(strcmp(D.controller, "ADRC_ADAPTIVE"), :);
if isempty(row_fix) || isempty(row_ad)
    error('degradation_results_adaptive.csv must include ADRC_FIXED and ADRC_ADAPTIVE rows.');
end

recovery_ratio = clamp01((row_fix.post_fault_rms - row_ad.post_fault_rms) / max(1e-6, row_fix.post_fault_rms));
recovery_time_gain = clamp01((row_fix.recovery_time_s - row_ad.recovery_time_s) / max(1e-6, row_fix.recovery_time_s));
r_nom = mean(B.rms_ratio_vs_adrcfixed(mask_h));
nominal_ratio = clamp01(1 - max(0, r_nom - 1.0) / 0.20);
sysid_quality = clamp01(1 - (mean(B.sysid_gain_err(mask_h)) + mean(B.sysid_tau_err(mask_h)) / 0.25));

adaptive_score = 100 * (0.35 * recovery_ratio + 0.30 * recovery_time_gain + 0.15 * nominal_ratio + 0.20 * sysid_quality);

overall_score = 0.60 * B_score + 0.40 * adaptive_score;

if overall_score >= 85
    readiness = "FINALIST_STRONG";
elseif overall_score >= 75
    readiness = "FINALIST_POSSIBLE";
elseif overall_score >= 65
    readiness = "HONORABLE_MENTION_RANGE";
else
    readiness = "NEEDS_MORE_REFINEMENT";
end

out_file = fullfile(out_dir, 'sts_scoreboard_adaptive.csv');
fid = fopen(out_file, 'w');
fprintf(fid, ['overall_score,readiness,B_score,adaptive_score,' ...
    'B_peak_ratio_vs_pid_highwind,B_rms_ratio_vs_pid_highwind,B_fail_reduction_vs_pid_highwind,B_in5_gain_vs_pid_highwind,' ...
    'adaptive_recovery_ratio,adaptive_recovery_time_gain,adaptive_nominal_ratio,adaptive_sysid_quality\n']);
fprintf(fid, '%.4f,%s,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
    overall_score, readiness, B_score, adaptive_score, peak_ratio_pid, rms_ratio_pid, fail_red_pid, in5_gain_pid, ...
    recovery_ratio, recovery_time_gain, nominal_ratio, sysid_quality);
fclose(fid);

fprintf('Saved: %s\n', out_file);
fprintf('Adaptive scoreboard: overall=%.2f B=%.2f adaptive=%.2f readiness=%s\n', overall_score, B_score, adaptive_score, readiness);

function y = clamp01(x)
y = min(1.0, max(0.0, x));
end
