function T = run_locked_exp1()
%RUN_LOCKED_EXP1 Launch locked-direction Experiment 1 frontier sweep.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'framework', 'experiments'));
T = run_exp1_stability_authority_frontier();
