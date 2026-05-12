%% build_sts_scoreboard.m
% Builds a unified STS readiness scoreboard from B-main and C-companion outputs.

root_dir = fileparts(fileparts(mfilename('fullpath')));
out_dir = fullfile(root_dir, 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

b_file = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow', 'outputs', 'industry_stress_metrics_B.csv');
c_file = fullfile(root_dir, 'supporting', 'Direction_C_Companion', 'outputs', 'sweep_results_C.csv');

if ~exist(b_file, 'file')
    error('Missing B metrics file: %s', b_file);
end
if ~exist(c_file, 'file')
    error('Missing C metrics file: %s', c_file);
end

B = readtable(b_file, 'TextType', 'string');
C = readtable(c_file, 'TextType', 'string');

% -----------------------------
% Direction B STS score
% -----------------------------
high_winds = ["10mps", "12mps", "15mps"];
mask_h = ismember(B.wind_label, high_winds);

peak_ratio_pid = mean(B.peak_ratio_vs_pid(mask_h));
rms_ratio_pid = mean(B.rms_ratio_vs_pid(mask_h));
fail_red_pid = mean(B.fail_reduction_vs_pid(mask_h));
in5_gain_pid = mean(B.in5_gain_vs_pid_pct(mask_h));
% sat_reduction_vs_pid is excluded from B scoring: PID fails via integrator
% windup (never hits u_max), so ACA-ADRC correctly appears to "saturate more"
% while actually tracking. Sat comparison vs PID is misleading; sat vs L1 is
% kept in the MC CSV for reference but not in the composite score.

score_peak = clamp01(1 - peak_ratio_pid);
score_rms  = clamp01(1 - rms_ratio_pid);
score_fail = clamp01(fail_red_pid);
score_in5  = clamp01(in5_gain_pid / 60.0);

% Weights: peak 45%, fail 25%, rms 15%, in5 15% (sat weight redistributed)
B_score = 100 * (0.45*score_peak + 0.15*score_rms + 0.25*score_fail + 0.15*score_in5);

% -----------------------------
% Direction C STS score
% -----------------------------
is_adapt = strcmp(C.adaptation, "ADAPT");
is_no = strcmp(C.adaptation, "NO_ADAPT");
Cad = C(is_adapt, :);
Cno = C(is_no, :);

% Align by fault label
[labels, ia, ib] = intersect(Cad.fault_label, Cno.fault_label);
Cad = Cad(ia, :);
Cno = Cno(ib, :);

max_peak_adapt = max(Cad.max_pitch_dev_deg);
mean_latency = mean(Cad.detect_latency_s);
pass_mask = (Cad.max_pitch_dev_deg <= 5.0) & (Cad.detect_latency_s <= 0.5) & (Cno.max_pitch_dev_deg > 15.0);
pass_frac = mean(double(pass_mask));

score_c_peak = clamp01(1 - max(0, max_peak_adapt - 5.0) / 5.0);
score_c_lat = clamp01(1 - max(0, mean_latency - 0.5) / 0.5);
score_c_pass = clamp01(pass_frac);

C_score = 100 * (0.45*score_c_peak + 0.35*score_c_lat + 0.20*score_c_pass);

% -----------------------------
% Overall STS readiness score
% -----------------------------
overall_score = 0.65 * B_score + 0.35 * C_score;

if overall_score >= 85
    readiness = "FINALIST_STRONG";
elseif overall_score >= 75
    readiness = "FINALIST_POSSIBLE";
elseif overall_score >= 65
    readiness = "HONORABLE_MENTION_RANGE";
else
    readiness = "NEEDS_MORE_REFINEMENT";
end

score_file = fullfile(out_dir, 'sts_scoreboard.csv');
fid = fopen(score_file, 'w');
fprintf(fid, 'overall_score,readiness,B_score,C_score,B_peak_ratio_vs_pid_highwind,B_rms_ratio_vs_pid_highwind,B_fail_reduction_vs_pid_highwind,B_in5_gain_vs_pid_highwind,C_max_peak_adapt_deg,C_mean_latency_s,C_pass_fraction\n');
fprintf(fid, '%.4f,%s,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
    overall_score, readiness, B_score, C_score, peak_ratio_pid, rms_ratio_pid, ...
    fail_red_pid, in5_gain_pid, max_peak_adapt, mean_latency, pass_frac);
fclose(fid);

fprintf('Saved: %s\n', score_file);
fprintf('STS scoreboard: overall=%.2f  B=%.2f  C=%.2f  readiness=%s\n', ...
    overall_score, B_score, C_score, readiness);
fprintf('C pass labels: %s\n', strjoin(labels(pass_mask), ', '));

function y = clamp01(x)
y = min(1.0, max(0.0, x));
end
