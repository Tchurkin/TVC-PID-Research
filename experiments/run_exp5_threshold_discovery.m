function [cellT, sensT, regimeT, designRuleT] = run_exp5_threshold_discovery()
%RUN_EXP5_THRESHOLD_DISCOVERY Compatibility wrapper for the isolated Exp5 layer.
%
% The old threshold-discovery implementation has been archived as
% run_exp5_threshold_discovery_legacy.m. The active Exp5 layer is now the
% regime-conditioned design-lever module.

[cellT, sensT, regimeT] = run_exp5_design_sensitivity();
designRuleT = sensT;
end
