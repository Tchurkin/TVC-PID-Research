function out = bench_to_validator(csv_path, options)
%BENCH_TO_VALIDATOR  Convert bench-CSV (INO or template) into validator input.
%
%  Bridges:
%    - Firmware/feedback_servo_calibration.ino Serial Plotter / table CSV
%      (cols: commanded_deg, actual_deg_est, rolling_up_net_slew_deg_s,
%       rolling_down_net_slew_deg_s, endpoint_util_pct, ...)
%      ** values are in SERVO degrees / deg/s and converted via linkage ratio **
%    - data/bench/gimbal_bench_test_template.csv format
%      (cols: time_s, cmd_deg, meas_deg)
%      ** values are already in GIMBAL deg **
%  ...into the input struct expected by experiments/validator/recommend_envelope.m
%
%  options (struct, all optional):
%    .source                'ino' | 'template'   (auto-detect if omitted)
%    .linkage_servo_per_gimbal  default 4.0      (matches INO firmware)
%    .servo_max_servo_deg   mechanical half-throw, default 40 (= 10 deg gimbal)
%    .p_est_1_s             airframe instability rate, default 8
%    .keff_est              control effectiveness, default 8
%    .damp_est              rate damping, default 0.2
%    .call_validator        invoke recommend_envelope (default true)
%
%  Returns the struct fed to recommend_envelope, with diagnostics attached.

if nargin < 2, options = struct(); end
defaults = struct( ...
    'linkage_servo_per_gimbal', 4.0, ...
    'servo_max_servo_deg',      40.0, ...
    'p_est_1_s',                8.0, ...
    'keff_est',                 8.0, ...
    'damp_est',                 0.2, ...
    'call_validator',           true);
fn = fieldnames(defaults);
for k = 1:numel(fn)
    if ~isfield(options, fn{k}), options.(fn{k}) = defaults.(fn{k}); end
end

T = readtable(csv_path, 'VariableNamingRule', 'preserve');
names = string(T.Properties.VariableNames);

if ~isfield(options, 'source') || isempty(options.source)
    if any(names == "rolling_up_net_slew_deg_s") || any(names == "commanded_deg")
        options.source = 'ino';
    elseif any(names == "cmd_deg") && any(names == "meas_deg")
        options.source = 'template';
    else
        error('bench_to_validator:unknownFormat', ...
            'Could not detect CSV format. Columns: %s', strjoin(names, ', '));
    end
end

switch lower(options.source)
    case 'ino'
        d = parse_ino(T, options);
    case 'template'
        d = parse_template(T, options);
    otherwise
        error('Unknown source %s', options.source);
end

out.slew_deg_per_s          = d.slew_deg_per_s;
out.slew_positive_deg_per_s = d.slew_positive_deg_per_s;
out.slew_negative_deg_per_s = d.slew_negative_deg_per_s;
out.endpoint_util_pct       = d.endpoint_util_pct;
out.fit_rmse_rad            = d.fit_rmse_rad;
out.servo_max_deg           = options.servo_max_servo_deg / options.linkage_servo_per_gimbal;
out.p_est                   = options.p_est_1_s;
out.keff_est                = options.keff_est;
out.damp_est                = options.damp_est;
out.source                  = options.source;
out.csv_path                = csv_path;

fprintf('=== Bench CSV -> Validator Input ===\n');
fprintf('  file                   : %s\n', csv_path);
fprintf('  source format          : %s\n', options.source);
fprintf('  linkage servo/gimbal   : %.2f\n', options.linkage_servo_per_gimbal);
fprintf('  positive slew (gimbal) : %.2f deg/s\n', out.slew_positive_deg_per_s);
fprintf('  negative slew (gimbal) : %.2f deg/s\n', out.slew_negative_deg_per_s);
fprintf('  worst-case slew used   : %.2f deg/s   <- fed to validator\n', out.slew_deg_per_s);
fprintf('  endpoint utilization   : %.1f %%\n', out.endpoint_util_pct);
fprintf('  servo mechanical max   : %.2f deg gimbal\n', out.servo_max_deg);
fprintf('  cmd->meas RMSE         : %.4f rad\n', out.fit_rmse_rad);

if options.call_validator
    here = fileparts(mfilename('fullpath'));
    proj = fileparts(here);
    addpath(fullfile(proj, 'experiments'));
    addpath(fullfile(proj, 'experiments', 'validator'));
    meas = struct( ...
        'slew_deg_per_s', out.slew_deg_per_s, ...
        'servo_max_deg',  out.servo_max_deg, ...
        'p_est',          out.p_est, ...
        'keff_est',       out.keff_est, ...
        'damp_est',       out.damp_est);
    fprintf('\n');
    out.validator = recommend_envelope(meas);
end
end

% =========================================================================
function d = parse_ino(T, options)
servo_per_gimbal = options.linkage_servo_per_gimbal;
names = string(T.Properties.VariableNames);

up_servo = []; dn_servo = [];
if any(names == "rolling_up_net_slew_deg_s")
    v = T.("rolling_up_net_slew_deg_s");
    up_servo = v(v > 0.1);
end
if any(names == "rolling_down_net_slew_deg_s")
    v = T.("rolling_down_net_slew_deg_s");
    dn_servo = v(v > 0.1);
end
% Rolling values are cumulative averages -> last value is best estimate
if ~isempty(up_servo), d.slew_positive_deg_per_s = up_servo(end) / servo_per_gimbal;
else,                  d.slew_positive_deg_per_s = NaN; end
if ~isempty(dn_servo), d.slew_negative_deg_per_s = dn_servo(end) / servo_per_gimbal;
else,                  d.slew_negative_deg_per_s = NaN; end

cand = [d.slew_positive_deg_per_s, d.slew_negative_deg_per_s];
cand = cand(~isnan(cand) & cand > 0);
if isempty(cand)
    d.slew_deg_per_s = derive_slew_from_timeseries(T) / servo_per_gimbal;
else
    d.slew_deg_per_s = min(cand);
end

if any(names == "endpoint_util_pct")
    v = T.endpoint_util_pct; v = v(v > 1);
    if ~isempty(v), d.endpoint_util_pct = v(end); else, d.endpoint_util_pct = 0; end
else
    d.endpoint_util_pct = NaN;
end

if any(names == "commanded_deg") && any(names == "actual_deg_est")
    err_servo_deg = T.actual_deg_est - T.commanded_deg;
    err_gimbal_rad = deg2rad(err_servo_deg / servo_per_gimbal);
    d.fit_rmse_rad = sqrt(mean(err_gimbal_rad.^2, 'omitnan'));
else
    d.fit_rmse_rad = NaN;
end
end

% =========================================================================
function d = parse_template(T, ~)
t = T.time_s; cmd = T.cmd_deg; meas = T.meas_deg;
dcmd = [0; diff(cmd)];
edges = find(abs(dcmd) > 1e-6);
up_slews = []; dn_slews = [];
if numel(edges) >= 2
    for k = 1:numel(edges)-1
        i0 = edges(k); i1 = edges(k+1) - 1;
        if i1 - i0 < 2, continue; end
        dt = t(i1) - t(i0);
        if dt < 1e-3, continue; end
        net = (meas(i1) - meas(i0)) / dt;
        if dcmd(edges(k)) > 0, up_slews(end+1) = abs(net); %#ok<AGROW>
        else,                  dn_slews(end+1) = abs(net); %#ok<AGROW>
        end
    end
end
if isempty(up_slews), d.slew_positive_deg_per_s = NaN; else, d.slew_positive_deg_per_s = mean(up_slews); end
if isempty(dn_slews), d.slew_negative_deg_per_s = NaN; else, d.slew_negative_deg_per_s = mean(dn_slews); end

% Short template traces can miss one side of the step window; fall back to
% direct measured-rate estimates so the validator still sees a conservative slew.
if ~isfinite(d.slew_positive_deg_per_s) || ~isfinite(d.slew_negative_deg_per_s)
    meas_rate_deg_s = gradient(meas, t);
    if ~isfinite(d.slew_positive_deg_per_s)
        d.slew_positive_deg_per_s = max([0; meas_rate_deg_s]);
    end
    if ~isfinite(d.slew_negative_deg_per_s)
        d.slew_negative_deg_per_s = max([0; -meas_rate_deg_s]);
    end
end

cand = [d.slew_positive_deg_per_s, d.slew_negative_deg_per_s];
cand = cand(~isnan(cand) & cand > 0);
if isempty(cand), d.slew_deg_per_s = NaN; else, d.slew_deg_per_s = min(cand); end

pc = max(abs(cmd)); pm = max(abs(meas));
if pc > 1e-6, d.endpoint_util_pct = 100 * pm / pc; else, d.endpoint_util_pct = NaN; end

err_rad = deg2rad(meas - cmd);
d.fit_rmse_rad = sqrt(mean(err_rad.^2, 'omitnan'));
end

% =========================================================================
function slew_servo = derive_slew_from_timeseries(T)
names = string(T.Properties.VariableNames);
if ~all([any(names=="commanded_deg"), any(names=="actual_deg_est")])
    slew_servo = NaN; return;
end
cmd = T.commanded_deg; act = T.actual_deg_est;
dt = 0.010;  % BINARY_SAMPLE_MS default
slews = [];
for k = 2:numel(cmd)
    if cmd(k) ~= cmd(k-1)
        i1 = min(numel(act), k + 50);
        seg_t = (0:i1-k)' * dt;
        seg_a = act(k:i1);
        if numel(seg_t) > 3
            p = polyfit(seg_t, seg_a, 1);
            slews(end+1) = abs(p(1)); %#ok<AGROW>
        end
    end
end
if isempty(slews), slew_servo = NaN; else, slew_servo = median(slews); end
end
