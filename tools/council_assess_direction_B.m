%% council_assess_direction_B.m
% Council-style assessment for Direction B finalist readiness and main-direction decision.

root_dir = fileparts(fileparts(mfilename('fullpath')));
out_dir = fullfile(root_dir, 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

b_metrics = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'industry_stress_metrics_B.csv');
if ~exist(b_metrics, 'file')
    error('Missing B metrics file: %s', b_metrics);
end

T = readtable(b_metrics, 'TextType', 'string');
high_wind = ismember(T.wind_label, ["10mps","12mps","15mps"]);

peak_vs_pid = mean(T.peak_ratio_vs_pid(high_wind));
rms_vs_pid = mean(T.rms_ratio_vs_pid(high_wind));
fail_red_pid = mean(T.fail_reduction_vs_pid(high_wind));
in5_gain_pid = mean(T.in5_gain_vs_pid_pct(high_wind));
lag_rms_ac = mean(T.cmd_lag_rms_ac(high_wind));

% Council criteria scores (0-100)
novelty = 85; % ACA-ADRC + hard constraints + decomposition metrics
rigor = 100 * (0.45*clamp01(1-peak_vs_pid) + 0.20*clamp01(1-rms_vs_pid) + 0.20*clamp01(fail_red_pid) + 0.15*clamp01(in5_gain_pid/70));
impact = 100 * (0.6*clamp01(1-peak_vs_pid) + 0.4*clamp01(fail_red_pid));
translatability = 100 * (0.6*clamp01(1-rms_vs_pid) + 0.4*clamp01(1-lag_rms_ac/2.5));

council_score = 0.30*novelty + 0.30*rigor + 0.25*impact + 0.15*translatability;

if council_score >= 85
    verdict = "B_IS_FINALIST_STRONG";
elseif council_score >= 75
    verdict = "B_IS_FINALIST_POSSIBLE";
else
    verdict = "B_NEEDS_MORE_EVIDENCE";
end

% Main direction decision logic
% If B is at least FINALIST_POSSIBLE and outperforms PID strongly at high wind, make it primary.
if (council_score >= 75) && (peak_vs_pid <= 0.30) && (fail_red_pid >= 0.66)
    main_decision = "MAKE_B_PRIMARY_DIRECTION";
else
    main_decision = "KEEP_B_CO_PRIMARY_WITH_C";
end

out_file = fullfile(out_dir, 'council_direction_B_assessment.csv');
fid = fopen(out_file, 'w');
fprintf(fid, 'council_score,verdict,main_decision,novelty,rigor,impact,translatability,peak_vs_pid_highwind,rms_vs_pid_highwind,fail_reduction_vs_pid_highwind,in5_gain_vs_pid_highwind,cmd_lag_rms_ac_highwind\n');
fprintf(fid, '%.4f,%s,%s,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
    council_score, verdict, main_decision, novelty, rigor, impact, translatability, ...
    peak_vs_pid, rms_vs_pid, fail_red_pid, in5_gain_pid, lag_rms_ac);
fclose(fid);

fprintf('Saved: %s\n', out_file);
fprintf('Council score=%.2f verdict=%s decision=%s\n', council_score, verdict, main_decision);

function y = clamp01(x)
y = min(1.0, max(0.0, x));
end
