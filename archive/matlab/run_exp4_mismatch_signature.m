function [trajT, summaryT, legacyT] = run_exp4_mismatch_signature(~)
%RUN_EXP4_MISMATCH_SIGNATURE Compatibility wrapper to the Exp4 fidelity ladder.
%
% Exp4 is now defined as: minimum simulator fidelity required to reproduce
% the highest-fidelity design decision.

warning('run_exp4_mismatch_signature:deprecated', ...
    ['Exp4 mismatch signature has been superseded by fidelity-ladder outputs. ' ...
     'Use run_exp4_fidelity_ladder for primary analysis.']);

[trajT, summaryT] = run_exp4_fidelity_ladder();
legacyT = trajT;
end
