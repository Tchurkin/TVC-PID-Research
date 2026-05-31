function run_current_vs_proposed_practice()
%RUN_CURRENT_VS_PROPOSED_PRACTICE  Build the headline tool-vs-status-quo
% comparison table from the existing firmware audit.
%
% Honest framing: the validator's GO verdict means "GO if you use the
% recommended PD gains." It does NOT mean "GO with whatever shipped gains
% your firmware has." So this comparison is between two different
% tuning recipes on the same plant cell, not between two preflight
% predictions of the same controller's outcome.
%
% For each audited open-source firmware:
%   - CURRENT PRACTICE: copy the firmware's shipped PID gains, fly.
%     Observed outcome = the audit success rate.
%   - PROPOSED PRACTICE: at the same (p, slew, u_max), the validator
%     issues GO/MARGINAL/NOGO + recommends a different gain. Prior
%     head-to-head work (head_to_head_unstable.csv) showed tuned PD on
%     the realistic plant outperforms shipped gains in the working cells.
%
% Output:
%   experiments/results/current_vs_proposed_practice.csv
%   experiments/results/graphs/current_vs_proposed_practice.png

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'validator'));
resdir = fullfile(here, 'results');
gfxdir = fullfile(resdir, 'graphs');
if ~exist(gfxdir, 'dir'); mkdir(gfxdir); end

T = readtable(fullfile(resdir, 'firmware_audit.csv'));

firmware_specs = struct( ...
    'tomkuttler_TVC_Flight', struct('slew_deg_s', 60,  'servo_max_deg', 5.0), ...
    'AdamMarciniak_FCV1',    struct('slew_deg_s', 90,  'servo_max_deg', 6.0), ...
    'PrajNasa_Dhumaketu',    struct('slew_deg_s', 90,  'servo_max_deg', 7.0));

firmware_keys = unique(T.firmware, 'stable');

rows = {};
for k = 1:numel(firmware_keys)
    fw = firmware_keys{k};
    key = matlab.lang.makeValidName(strrep(fw, '/', '_'));
    if isfield(firmware_specs, key)
        spec = firmware_specs.(key);
    else
        spec = struct('slew_deg_s', 90, 'servo_max_deg', 6.0);
    end

    sel = strcmp(T.firmware, fw);
    sub = T(sel, :);
    for r = 1:height(sub)
        p_val = sub.p_unstable(r);
        actual_rate = sub.success_rate(r);

        meas = struct('slew_deg_per_s', spec.slew_deg_s, ...
            'servo_max_deg', spec.servo_max_deg, ...
            'p_est', max(p_val, 0.1), 'keff_est', 8.0, 'damp_est', 0.5);
        evalc('rec = recommend_envelope(meas);');

        if actual_rate < 0.5
            actual_outcome = 'FAIL';
        elseif actual_rate < 0.85
            actual_outcome = 'MARGINAL';
        else
            actual_outcome = 'PASS';
        end

        proposed_call = rec.go_nogo;
        would_validator_warn = ismember(proposed_call, {'MARGINAL','NOGO'});
        would_validator_fix = strcmp(proposed_call, 'GO') && ...
            ~strcmp(actual_outcome, 'PASS');

        rows(end+1, :) = { ...
            string(fw), p_val, spec.slew_deg_s, spec.servo_max_deg, ...
            string(rec.region), string(proposed_call), ...
            actual_rate, string(actual_outcome), ...
            would_validator_warn, would_validator_fix}; %#ok<SAGROW>
    end
end

CT = cell2table(rows, 'VariableNames', { ...
    'firmware', 'p_unstable', 'assumed_slew_deg_s', 'assumed_servo_max_deg', ...
    'validator_region', 'proposed_practice_call', ...
    'actual_success_rate_shipped_gains', 'actual_outcome_with_shipped_gains', ...
    'would_validator_have_warned', 'would_validator_have_fixed_it'});
writetable(CT, fullfile(resdir, 'current_vs_proposed_practice.csv'));
fprintf('Saved: experiments/results/current_vs_proposed_practice.csv\n');

n_fail = sum(~strcmp(CT.actual_outcome_with_shipped_gains, 'PASS'));
n_fixable = sum(CT.would_validator_have_fixed_it);
n_warned = sum(CT.would_validator_have_warned);
fprintf('\n=== Current vs Proposed Practice Summary ===\n');
fprintf('Total audit cells:                                %d\n', height(CT));
fprintf('Cells where shipped gains failed or marginal:     %d\n', n_fail);
fprintf('  ...validator would have warned (MARGINAL/NOGO): %d\n', n_warned);
fprintf('  ...validator GO + different recommended gains:  %d  (would have replaced bad gains)\n', n_fixable);

set(groot, 'defaultFigureVisible', 'off');
fig = figure('Position', [80 80 1300 720], 'Color', 'w');
tl = tiledlayout(1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
tl.OuterPosition = [0 0.10 1 0.90];
nexttile;

p_values = unique(CT.p_unstable);
firmwares = unique(CT.firmware, 'stable');
Y = nan(numel(p_values), numel(firmwares));
verd_text = strings(numel(p_values), numel(firmwares));
for ip = 1:numel(p_values)
    for ifw = 1:numel(firmwares)
        sel = CT.p_unstable == p_values(ip) & strcmp(CT.firmware, firmwares(ifw));
        if any(sel)
            v = CT.actual_success_rate_shipped_gains(find(sel,1));
            if ~isfinite(v); v = 0; end
            Y(ip, ifw) = max(0, min(1, v));
            verd_text(ip, ifw) = CT.proposed_practice_call(find(sel,1));
        end
    end
end

b = bar(p_values, Y, 'grouped');
ylim([0 1.18]);
ylabel('Actual success rate using the firmware''s shipped gains');
xlabel('Airframe instability p (rad/s)');
title({'Current Open-Source Firmware Gains vs Proposed Preflight Workflow', ...
    'Bar = what shipped gains actually achieve.  Label = validator verdict + recommended different gains.'}, ...
    'FontSize', 13);
legend(strrep(firmwares, '_', '\_'), 'Location', 'northeast', 'FontSize', 10);
grid on;

for ifw = 1:numel(firmwares)
    xtips = b(ifw).XEndPoints;
    ytips = b(ifw).YEndPoints;
    for ip = 1:numel(p_values)
        v = char(verd_text(ip, ifw));
        if isempty(v); continue; end
        switch v
            case 'GO',       c = [0.10 0.55 0.20];
            case 'MARGINAL', c = [0.75 0.55 0.10];
            case 'NOGO',     c = [0.75 0.15 0.15];
            otherwise,       c = [0.30 0.30 0.30];
        end
        text(xtips(ip), ytips(ip) + 0.045, sprintf('VAL: %s', v), ...
            'HorizontalAlignment', 'center', 'FontSize', 8, ...
            'Color', c, 'FontWeight', 'bold');
    end
end

annotation('textbox', [0.05 0.005 0.90 0.06], 'String', ...
    ['Validator GO means "GO with the validator''s recommended PD gains," not with the firmware''s shipped gains. ' ...
     'Where the shipped gains fail and the validator says GO, the workflow''s value is gain replacement, not GO/NOGO triage.'], ...
    'EdgeColor', 'none', 'FontSize', 9, 'FontAngle', 'italic', ...
    'Color', [0.30 0.30 0.30], 'HorizontalAlignment', 'center');

out_path = fullfile(gfxdir, 'current_vs_proposed_practice.png');
try
    exportgraphics(fig, out_path, 'Resolution', 200);
catch
    saveas(fig, out_path);
end
close(fig);
fprintf('Saved: %s\n', out_path);
end
