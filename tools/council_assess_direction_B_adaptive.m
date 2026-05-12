%% council_assess_direction_B_adaptive.m
% Council-style assessment for adaptive Direction B.

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

T = readtable(stress_file, 'TextType', 'string');
D = readtable(deg_file, 'TextType', 'string');
high_wind = ismember(T.wind_label, ["10mps", "12mps", "15mps"]);

peak_vs_pid = mean(T.peak_ratio_vs_pid(high_wind));
rms_vs_pid = mean(T.rms_ratio_vs_pid(high_wind));
fail_red_pid = mean(T.fail_reduction_vs_pid(high_wind));
in5_gain_pid = mean(T.in5_gain_vs_pid_pct(high_wind));
lag_rms_ad = mean(T.cmd_lag_rms_adaptive(high_wind));

fix_row = D(strcmp(D.controller, "ADRC_FIXED"), :);
ad_row = D(strcmp(D.controller, "ADRC_ADAPTIVE"), :);
if isempty(fix_row) || isempty(ad_row)
    error('Need ADRC_FIXED and ADRC_ADAPTIVE rows in degradation_results_adaptive.csv');
end

recovery_ratio = clamp01((fix_row.post_fault_rms - ad_row.post_fault_rms) / max(1e-6, fix_row.post_fault_rms));
recovery_time_gain = clamp01((fix_row.recovery_time_s - ad_row.recovery_time_s) / max(1e-6, fix_row.recovery_time_s));
r_nom = mean(T.rms_ratio_vs_adrcfixed(high_wind));
nominal_retention = clamp01(1 - max(0, r_nom - 1.0) / 0.20);
sysid_quality = clamp01(1 - (mean(T.sysid_gain_err(high_wind)) + mean(T.sysid_tau_err(high_wind)) / 0.25));
adaptive_capability = 100 * (0.35 * recovery_ratio + 0.35 * recovery_time_gain + 0.15 * nominal_retention + 0.15 * sysid_quality);

novelty = 95;
rigor = 100 * (0.40 * clamp01(1 - peak_vs_pid) + 0.20 * clamp01(1 - rms_vs_pid) + 0.20 * clamp01(fail_red_pid) + 0.20 * clamp01(in5_gain_pid / 70));
impact = 100 * (0.45 * clamp01(1 - peak_vs_pid) + 0.25 * clamp01(fail_red_pid) + 0.30 * clamp01(recovery_ratio));
translatability = 100 * (0.45 * clamp01(1 - rms_vs_pid) + 0.25 * clamp01(1 - lag_rms_ad / 2.5) + 0.30 * clamp01(sysid_quality));

council_score = 0.25 * novelty + 0.25 * rigor + 0.20 * impact + 0.15 * translatability + 0.15 * adaptive_capability;

if council_score >= 85
    verdict = "B_IS_FINALIST_STRONG";
elseif council_score >= 75
    verdict = "B_IS_FINALIST_POSSIBLE";
else
    verdict = "B_NEEDS_MORE_EVIDENCE";
end

if (council_score >= 80) && (peak_vs_pid <= 0.30) && (recovery_ratio >= 0.25)
    main_decision = "MAKE_B_PRIMARY_DIRECTION";
else
    main_decision = "KEEP_B_CO_PRIMARY_WITH_C";
end

out_file = fullfile(out_dir, 'council_direction_B_adaptive_assessment.csv');
fid = fopen(out_file, 'w');
fprintf(fid, ['council_score,verdict,main_decision,novelty,rigor,impact,translatability,adaptive_capability,' ...
    'peak_vs_pid_highwind,rms_vs_pid_highwind,fail_reduction_vs_pid_highwind,in5_gain_vs_pid_highwind,' ...
    'recovery_ratio,recovery_time_gain,sysid_quality,cmd_lag_rms_adaptive_highwind\n']);
fprintf(fid, '%.4f,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
    council_score, verdict, main_decision, novelty, rigor, impact, translatability, adaptive_capability, ...
    peak_vs_pid, rms_vs_pid, fail_red_pid, in5_gain_pid, recovery_ratio, recovery_time_gain, sysid_quality, lag_rms_ad);
fclose(fid);

fprintf('Saved: %s\n', out_file);
fprintf('Adaptive council score=%.2f verdict=%s decision=%s\n', council_score, verdict, main_decision);

function y = clamp01(x)
y = min(1.0, max(0.0, x));
end
