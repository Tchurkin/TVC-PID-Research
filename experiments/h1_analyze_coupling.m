function summary = h1_analyze_coupling(results, success_threshold)
%H1_ANALYZE_COUPLING  Compare empirical stabilizable region to the AND of
% single-limit "marginal" regions, quantifying coupling.
%
% Method:
%   - Define stabilizable cell := success_rate >= success_threshold (default 0.8)
%   - For each row of slew, find smallest u_max where stabilizable holds.
%     Call this u_min_emp(slew).
%   - For each column of u_max, find smallest slew where stabilizable holds.
%     Call this s_min_emp(u_max).
%   - "Composed single-limit prediction": a cell (s,u) is composed-stabilizable
%     iff slew >= min_slew_overall AND u_max >= min_umax_overall, where
%     min_slew_overall and min_umax_overall are taken at the BEST of the
%     other axis (i.e. with the other constraint relaxed).
%   - Coupling metric: fraction of cells empirically UNSTABLE that the
%     composed prediction calls STABLE (over-prediction error).

if nargin < 2, success_threshold = 0.8; end

S = results.slew_grid;
U = results.umax_grid;
SR = results.success_rate;
stab = SR >= success_threshold;

fprintf('  success_rate min=%.2f max=%.2f mean=%.2f\n', ...
    min(SR(:)), max(SR(:)), mean(SR(:)));
fprintf('  cells stabilized: %d / %d\n', sum(stab(:)), numel(stab));

% Best per-axis (other axis relaxed)
slew_when_uMax_largest = find(stab(:, end), 1, 'first');
umax_when_slew_largest = find(stab(end, :), 1, 'first');

have_valid = ~isempty(slew_when_uMax_largest) && ~isempty(umax_when_slew_largest);

% Composed prediction grid
composed = false(size(stab));
if have_valid
    composed(slew_when_uMax_largest:end, umax_when_slew_largest:end) = true;
end

% Coupling: cells composed predicts stable but empirically unstable
overpredict_mask = composed & ~stab;
overpredict_rate = mean(overpredict_mask(:));

% Coupling area: cells that are inside composed-stable region but unstable
total_composed_stable = sum(composed(:));
coupling_loss_frac = sum(overpredict_mask(:)) / max(1, total_composed_stable);

summary.success_threshold      = success_threshold;
summary.composed_stable_mask   = composed;
summary.empirical_stable_mask  = stab;
summary.overpredict_mask       = overpredict_mask;
summary.overpredict_cell_rate  = overpredict_rate;
summary.coupling_loss_frac     = coupling_loss_frac;
if have_valid
    summary.slew_marginal_at_umax_max = S(slew_when_uMax_largest);
    summary.umax_marginal_at_slew_max = U(umax_when_slew_largest);
else
    summary.slew_marginal_at_umax_max = NaN;
    summary.umax_marginal_at_slew_max = NaN;
end

fprintf('\nH1 ANALYSIS  (success threshold = %.2f)\n', success_threshold);
fprintf('  Composed prediction: stable if slew>=%.2f AND u_max>=%.2f\n', ...
    summary.slew_marginal_at_umax_max, summary.umax_marginal_at_slew_max);
fprintf('  Composed-stable cells:           %d\n', total_composed_stable);
fprintf('  Of those, empirically unstable:  %d\n', sum(overpredict_mask(:)));
fprintf('  COUPLING LOSS FRACTION:          %.1f%%\n', 100*coupling_loss_frac);
if coupling_loss_frac > 0.10
    fprintf('  -> EVIDENCE FOR H1: single-limit composition OVER-predicts stability\n');
else
    fprintf('  -> NO STRONG EVIDENCE FOR H1: composed bound nearly matches empirical\n');
end
end
