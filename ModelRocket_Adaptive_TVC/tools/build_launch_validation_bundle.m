function bundle = build_launch_validation_bundle(flight_log_dir, root_dir)
%BUILD_LAUNCH_VALIDATION_BUNDLE Build proof-ready artifacts from real flight logs.
%   BUNDLE = BUILD_LAUNCH_VALIDATION_BUNDLE(FLIGHT_LOG_DIR, ROOT_DIR)
%   ingests all firmware CSV logs in FLIGHT_LOG_DIR, computes validation
%   metrics, compares against simulation references, and writes:
%     - outputs/flight_validation/launch_validation_summary.csv
%     - outputs/flight_validation/launch_validation_report.md
%     - outputs/flight_validation/graphs/*.png
%
%   The function is intentionally conservative: it separates real proof of
%   flight stability from direct evidence of slew-envelope identification.

if nargin < 2 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
if nargin < 1 || isempty(flight_log_dir)
    flight_log_dir = fullfile(root_dir, 'data', 'flight_logs');
end

out_dir = fullfile(root_dir, 'outputs', 'flight_validation');
graph_dir = fullfile(out_dir, 'graphs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
if ~exist(graph_dir, 'dir'), mkdir(graph_dir); end

files = dir(fullfile(flight_log_dir, '*.csv'));
if isempty(files)
    error('No flight log CSV files found in: %s', flight_log_dir);
end

sim_ref = load_sim_reference(root_dir);

rows = repmat(blank_row(), numel(files), 1);
for i = 1:numel(files)
    log_path = fullfile(files(i).folder, files(i).name);
    T = read_firmware_log(log_path);

    metrics = compute_flight_metrics(T, files(i).name, sim_ref);
    rows(i) = metrics;

    plot_path = fullfile(graph_dir, [strip_ext(files(i).name) '_overview.png']);
    generate_flight_overview_plot(T, metrics, plot_path);
end

summary_tbl = struct2table(rows);
summary_csv = fullfile(out_dir, 'launch_validation_summary.csv');
writetable(summary_tbl, summary_csv);

report_md = fullfile(out_dir, 'launch_validation_report.md');
write_markdown_report(report_md, summary_tbl, sim_ref);

bundle = struct();
bundle.summary_table = summary_tbl;
bundle.summary_csv = summary_csv;
bundle.report_markdown = report_md;
bundle.graph_dir = graph_dir;

fprintf('\n=== Launch Validation Bundle Complete ===\n');
fprintf('Logs analyzed: %d\n', numel(files));
fprintf('Summary CSV:   %s\n', summary_csv);
fprintf('Report MD:     %s\n', report_md);
fprintf('Graphs dir:    %s\n', graph_dir);
end

function T = read_firmware_log(log_path)
opts = detectImportOptions(log_path, 'NumHeaderLines', 0);
opts = setvaropts(opts, {'State', 'AbortReason'}, 'WhitespaceRule', 'preserve');
opts = setvaropts(opts, {'State', 'AbortReason'}, 'EmptyFieldRule', 'auto');
T = readtable(log_path, opts);

required_cols = {'TimeMs','State','AbortReason','ThetaRad','QRad_s','UAct','UCmd', ...
    'AltM','VertVelMps','HighAltM','KeffEst','SlewEst','GainScale','KeffTheta', ...
    'KeffQ','DemandRate','DemandRateDecoupled','AbsDuObs','Confidence', ...
    'Saturating','SatStreak','ActFeedback','ShieldSlew','ShieldAttitude','ComputeUs'};

missing = required_cols(~ismember(required_cols, T.Properties.VariableNames));
if ~isempty(missing)
    error('Log %s missing required columns: %s', log_path, strjoin(missing, ', '));
end

T = sortrows(T, 'TimeMs');

% Normalize state/abort text to simplify comparisons.
T.State = upper(string(T.State));
T.AbortReason = upper(string(T.AbortReason));

% Normalize booleans that may be imported as strings.
for name = {'Saturating','ActFeedback','ShieldSlew','ShieldAttitude'}
    c = name{1};
    T.(c) = normalize_bool(T.(c));
end
end

function x = normalize_bool(raw)
if isnumeric(raw)
    x = double(raw ~= 0);
    return;
end

if iscellstr(raw) || isstring(raw)
    s = upper(string(raw));
    x = zeros(size(s));
    x(s == "1" | s == "TRUE" | s == "T") = 1;
    return;
end

x = double(raw);
end

function metrics = compute_flight_metrics(T, flight_id, sim_ref)
metrics = blank_row();
metrics.flight_id = string(flight_id);
metrics.samples = height(T);

t_s = (double(T.TimeMs) - double(T.TimeMs(1))) / 1000.0;
metrics.duration_s = t_s(end);

if numel(t_s) > 1
    dt = diff(t_s);
    metrics.log_rate_hz = 1.0 / max(eps, median(dt(~isnan(dt) & isfinite(dt))));
else
    metrics.log_rate_hz = 0;
end

theta_deg = rad2deg(double(T.ThetaRad));
q_dps = rad2deg(double(T.QRad_s));

is_powered = T.State == "POWERED" | T.State == "POWERED_FLIGHT";
is_aborted = T.State == "ABORTED";

if any(is_powered)
    t_pow = t_s(is_powered);
    metrics.powered_duration_s = t_pow(end) - t_pow(1);
    metrics.rms_theta_powered_deg = rms_scalar(theta_deg(is_powered));
    metrics.peak_theta_powered_deg = max(abs(theta_deg(is_powered)));
    metrics.p95_abs_q_powered_dps = prctile(abs(q_dps(is_powered)), 95);
else
    metrics.powered_duration_s = 0;
    metrics.rms_theta_powered_deg = NaN;
    metrics.peak_theta_powered_deg = NaN;
    metrics.p95_abs_q_powered_dps = NaN;
end

metrics.peak_theta_deg = max(abs(theta_deg));
metrics.max_alt_m = max(double(T.HighAltM));
metrics.max_vert_vel_mps = max(double(T.VertVelMps));

metrics.abort_flag = double(any(is_aborted) || any(T.AbortReason ~= "NONE"));
metrics.abort_reason = infer_abort_reason(T.AbortReason);

% Adaptation evidence metrics.
if any(is_powered)
    idx_pow = find(is_powered);
    n_win = max(3, min(10, floor(numel(idx_pow) * 0.2)));
    head_idx = idx_pow(1:n_win);
    tail_idx = idx_pow(end - n_win + 1:end);
else
    n_win = max(3, min(10, floor(height(T) * 0.2)));
    head_idx = 1:n_win;
    tail_idx = (height(T) - n_win + 1):height(T);
end

keff_start = median(double(T.KeffEst(head_idx)), 'omitnan');
keff_end = median(double(T.KeffEst(tail_idx)), 'omitnan');
slew_start = median(double(T.SlewEst(head_idx)), 'omitnan');
slew_end = median(double(T.SlewEst(tail_idx)), 'omitnan');

metrics.keff_start = keff_start;
metrics.keff_end = keff_end;
metrics.keff_shift_pct = pct_change(keff_start, keff_end);
metrics.slew_start = slew_start;
metrics.slew_end = slew_end;
metrics.slew_shift_pct = pct_change(slew_start, slew_end);

metrics.sat_fraction = mean(double(T.Saturating) > 0.5);
metrics.max_sat_streak = max(double(T.SatStreak));
metrics.shield_slew_fraction = mean(double(T.ShieldSlew) > 0.5);
metrics.shield_att_fraction = mean(double(T.ShieldAttitude) > 0.5);
metrics.act_feedback_fraction = mean(double(T.ActFeedback) > 0.5);

metrics.compute_us_p50 = median(double(T.ComputeUs), 'omitnan');
metrics.compute_us_p95 = prctile(double(T.ComputeUs), 95);

% Simulation consistency checks (against JOINT realistic references).
if isnan(metrics.rms_theta_powered_deg)
    metrics.sim_consistent = 0;
    metrics.sim_margin_ratio = NaN;
else
    sim_limit = max(sim_ref.joint_slew_rms_p95_deg, 1.0) * 2.5;
    metrics.sim_margin_ratio = metrics.rms_theta_powered_deg / max(sim_limit, eps);
    metrics.sim_consistent = double(metrics.rms_theta_powered_deg <= sim_limit);
end

metrics.claim_level = classify_claim_level(metrics.act_feedback_fraction);
metrics.proof_score = compute_proof_score(metrics);
metrics.proof_tier = classify_proof_tier(metrics.proof_score);
end

function s = infer_abort_reason(abort_col)
valid = abort_col(abort_col ~= "NONE" & abort_col ~= "");
if isempty(valid)
    s = "NONE";
else
    s = valid(end);
end
end

function c = pct_change(a, b)
if ~isfinite(a) || abs(a) < eps
    c = NaN;
    return;
end
c = 100.0 * (b - a) / a;
end

function y = rms_scalar(x)
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = sqrt(mean(x.^2));
end
end

function score = compute_proof_score(m)
score = 0;

% Data quality (20)
if m.samples >= 120, score = score + 10; end
if m.log_rate_hz >= 15 && m.log_rate_hz <= 25, score = score + 10; end

% Safety outcome (30)
if m.abort_flag == 0, score = score + 30; end

% Control containment (25)
if ~isnan(m.rms_theta_powered_deg) && m.rms_theta_powered_deg <= 10
    score = score + 12;
elseif ~isnan(m.rms_theta_powered_deg) && m.rms_theta_powered_deg <= 15
    score = score + 6;
end
if ~isnan(m.peak_theta_powered_deg) && m.peak_theta_powered_deg <= 20
    score = score + 13;
elseif ~isnan(m.peak_theta_powered_deg) && m.peak_theta_powered_deg <= 30
    score = score + 6;
end

% Adaptation observability (15)
if abs(m.keff_shift_pct) >= 5 || abs(m.slew_shift_pct) >= 5 || m.sat_fraction >= 0.02
    score = score + 8;
end
if m.act_feedback_fraction >= 0.80
    score = score + 7;
elseif m.act_feedback_fraction > 0
    score = score + 4;
end

% Compute budget (10)
if m.compute_us_p95 <= 6000
    score = score + 10;
elseif m.compute_us_p95 <= 8000
    score = score + 5;
end

score = max(0, min(100, score));
end

function s = classify_proof_tier(score)
if score >= 85
    s = "STRONG";
elseif score >= 70
    s = "GOOD";
elseif score >= 50
    s = "PARTIAL";
else
    s = "WEAK";
end
end

function s = classify_claim_level(act_feedback_fraction)
if act_feedback_fraction >= 0.80
    s = "DIRECT_SLEW_EVIDENCE";
elseif act_feedback_fraction > 0
    s = "PARTIAL_SLEW_EVIDENCE";
else
    s = "OBSERVER_ONLY_SLEW_EVIDENCE";
end
end

function sim_ref = load_sim_reference(root_dir)
csv_path = fullfile(root_dir, 'outputs', 'data', 'realism_montecarlo.csv');
if ~exist(csv_path, 'file')
    warning('Simulation reference missing: %s', csv_path);
    sim_ref = struct( ...
        'joint_slew_rms_mean_deg', NaN, ...
        'joint_slew_rms_p95_deg', NaN, ...
        'joint_slew_pk_p95_deg', NaN, ...
        'joint_keff_rms_mean_deg', NaN, ...
        'joint_keff_pk_p95_deg', NaN);
    return;
end

T = readtable(csv_path);
req = {'scenario_idx','ctrl_idx','rms_post','pk_post'};
if ~all(ismember(req, T.Properties.VariableNames))
    error('Reference CSV missing required columns: %s', csv_path);
end

is_joint = double(T.ctrl_idx) == 5;
is_slew = double(T.scenario_idx) == 1;
is_keff = double(T.scenario_idx) == 2;

joint_slew_rms = double(T.rms_post(is_joint & is_slew));
joint_slew_pk = double(T.pk_post(is_joint & is_slew));
joint_keff_rms = double(T.rms_post(is_joint & is_keff));
joint_keff_pk = double(T.pk_post(is_joint & is_keff));

sim_ref = struct();
sim_ref.joint_slew_rms_mean_deg = mean(joint_slew_rms, 'omitnan');
sim_ref.joint_slew_rms_p95_deg = prctile(joint_slew_rms, 95);
sim_ref.joint_slew_pk_p95_deg = prctile(joint_slew_pk, 95);
sim_ref.joint_keff_rms_mean_deg = mean(joint_keff_rms, 'omitnan');
sim_ref.joint_keff_pk_p95_deg = prctile(joint_keff_pk, 95);
end

function generate_flight_overview_plot(T, metrics, out_path)
t = (double(T.TimeMs) - double(T.TimeMs(1))) / 1000.0;
theta_deg = rad2deg(double(T.ThetaRad));
q_dps = rad2deg(double(T.QRad_s));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1400 900]);

ax1 = subplot(2,2,1);
hold(ax1, 'on'); grid(ax1, 'on');
plot(ax1, t, theta_deg, 'LineWidth', 1.6, 'Color', [0.00 0.45 0.74]);
plot(ax1, t, q_dps, 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]);
xlabel(ax1, 'Time (s)');
ylabel(ax1, 'Deg / Deg/s');
legend(ax1, {'Theta (deg)','Q (deg/s)'}, 'Location', 'best');
title(ax1, sprintf('Attitude traces | abort=%s', string(metrics.abort_reason)));

ax2 = subplot(2,2,2);
hold(ax2, 'on'); grid(ax2, 'on');
plot(ax2, t, double(T.KeffEst), 'LineWidth', 1.5, 'Color', [0.00 0.50 0.20]);
plot(ax2, t, double(T.SlewEst), 'LineWidth', 1.5, 'Color', [0.49 0.18 0.56]);
plot(ax2, t, double(T.GainScale), 'LineWidth', 1.2, 'Color', [0.30 0.30 0.30]);
xlabel(ax2, 'Time (s)');
ylabel(ax2, 'Estimator value');
legend(ax2, {'KeffEst','SlewEst','GainScale'}, 'Location', 'best');
title(ax2, sprintf('Adaptive channels | claim=%s', string(metrics.claim_level)));

ax3 = subplot(2,2,3);
hold(ax3, 'on'); grid(ax3, 'on');
plot(ax3, t, double(T.DemandRate), 'LineWidth', 1.2, 'Color', [0.00 0.45 0.74]);
plot(ax3, t, double(T.AbsDuObs), 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]);
plot(ax3, t, double(T.Saturating), 'LineWidth', 1.0, 'Color', [0.63 0.08 0.18]);
xlabel(ax3, 'Time (s)');
ylabel(ax3, 'Rate / bool');
legend(ax3, {'DemandRate','AbsDuObs','Saturating'}, 'Location', 'best');
title(ax3, sprintf('Saturation diagnostics | sat frac %.2f', metrics.sat_fraction));

ax4 = subplot(2,2,4);
hold(ax4, 'on'); grid(ax4, 'on');
plot(ax4, t, double(T.AltM), 'LineWidth', 1.5, 'Color', [0.00 0.45 0.74]);
plot(ax4, t, double(T.HighAltM), '--', 'LineWidth', 1.2, 'Color', [0.47 0.67 0.19]);
plot(ax4, t, double(T.VertVelMps), 'LineWidth', 1.0, 'Color', [0.85 0.33 0.10]);
xlabel(ax4, 'Time (s)');
ylabel(ax4, 'm / m/s');
legend(ax4, {'AltM','HighAltM','VertVelMps'}, 'Location', 'best');
title(ax4, sprintf('Trajectory | proof score %.0f (%s)', metrics.proof_score, string(metrics.proof_tier)));

sgtitle(sprintf('Flight Overview: %s', string(metrics.flight_id)), 'FontWeight', 'bold');
exportgraphics(fig, out_path, 'Resolution', 180);
close(fig);
end

function write_markdown_report(report_md, summary_tbl, sim_ref)
fid = fopen(report_md, 'w');
if fid < 0
    error('Failed to open report for writing: %s', report_md);
end

cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# Launch Validation Report\n\n');
fprintf(fid, 'Generated on: %s\n\n', datestr(now, 31));

fprintf(fid, '## Simulation References (JOINT realistic Monte Carlo)\n\n');
fprintf(fid, '- Slew scenario mean RMS post-fault: %.2f deg\n', sim_ref.joint_slew_rms_mean_deg);
fprintf(fid, '- Slew scenario 95th RMS post-fault: %.2f deg\n', sim_ref.joint_slew_rms_p95_deg);
fprintf(fid, '- Slew scenario 95th peak post-fault: %.2f deg\n', sim_ref.joint_slew_pk_p95_deg);
fprintf(fid, '- High-keff scenario mean RMS post-fault: %.2f deg\n', sim_ref.joint_keff_rms_mean_deg);
fprintf(fid, '- High-keff scenario 95th peak post-fault: %.2f deg\n\n', sim_ref.joint_keff_pk_p95_deg);

fprintf(fid, '## Flight Summary\n\n');
fprintf(fid, '| Flight | Proof Tier | Score | Abort | RMS Powered (deg) | Peak Powered (deg) | ActFeedback Frac | Claim Level |\n');
fprintf(fid, '|---|---:|---:|---|---:|---:|---:|---|\n');
for i = 1:height(summary_tbl)
    fprintf(fid, '| %s | %s | %.0f | %s | %.2f | %.2f | %.2f | %s |\n', ...
        summary_tbl.flight_id(i), ...
        summary_tbl.proof_tier(i), ...
        summary_tbl.proof_score(i), ...
        summary_tbl.abort_reason(i), ...
        summary_tbl.rms_theta_powered_deg(i), ...
        summary_tbl.peak_theta_powered_deg(i), ...
        summary_tbl.act_feedback_fraction(i), ...
        summary_tbl.claim_level(i));
end
fprintf(fid, '\n');

fprintf(fid, '## Interpretation Rules\n\n');
fprintf(fid, '- `DIRECT_SLEW_EVIDENCE`: actuator feedback available for most of the flight.\n');
fprintf(fid, '- `PARTIAL_SLEW_EVIDENCE`: actuator feedback exists but not consistently.\n');
fprintf(fid, '- `OBSERVER_ONLY_SLEW_EVIDENCE`: no direct actuator measurement; flight still validates control stability but not direct slew-ID claim.\n\n');

fprintf(fid, '## Notes\n\n');
fprintf(fid, '- Proof score is a structured engineering metric, not a formal certificate.\n');
fprintf(fid, '- Use this report together with raw CSV logs and launch video to support external review.\n');
end

function row = blank_row()
row = struct( ...
    'flight_id', "", ...
    'samples', 0, ...
    'duration_s', NaN, ...
    'log_rate_hz', NaN, ...
    'powered_duration_s', NaN, ...
    'rms_theta_powered_deg', NaN, ...
    'peak_theta_powered_deg', NaN, ...
    'p95_abs_q_powered_dps', NaN, ...
    'peak_theta_deg', NaN, ...
    'max_alt_m', NaN, ...
    'max_vert_vel_mps', NaN, ...
    'abort_flag', 0, ...
    'abort_reason', "NONE", ...
    'keff_start', NaN, ...
    'keff_end', NaN, ...
    'keff_shift_pct', NaN, ...
    'slew_start', NaN, ...
    'slew_end', NaN, ...
    'slew_shift_pct', NaN, ...
    'sat_fraction', NaN, ...
    'max_sat_streak', NaN, ...
    'shield_slew_fraction', NaN, ...
    'shield_att_fraction', NaN, ...
    'act_feedback_fraction', NaN, ...
    'compute_us_p50', NaN, ...
    'compute_us_p95', NaN, ...
    'sim_consistent', 0, ...
    'sim_margin_ratio', NaN, ...
    'claim_level', "", ...
    'proof_score', NaN, ...
    'proof_tier', "");
end

function s = strip_ext(name)
[~, s, ~] = fileparts(name);
end