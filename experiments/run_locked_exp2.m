function [cellT, trialT, regimeSummaryT] = run_locked_exp2()
%RUN_LOCKED_EXP2 Launch locked-direction Experiment 2 maneuverability envelope.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'framework', 'experiments'));
[cellT, trialT, regimeSummaryT] = run_exp2_maneuverability_envelope();