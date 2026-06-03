function [cellT, sensT, regimeT, designRuleT] = run_exp5_threshold_discovery(maxCells)
%RUN_EXP5_THRESHOLD_DISCOVERY Compatibility wrapper for the isolated Exp5 layer.
%
% The old threshold-discovery implementation has been archived as
% run_exp5_threshold_discovery_legacy.m. The active Exp5 layer is now the
% regime-conditioned design-lever module.

if nargin < 1
	maxCells = inf;
end

[cellT, sensT, regimeT] = run_exp5_design_sensitivity(maxCells);
designRuleT = sensT;
end
