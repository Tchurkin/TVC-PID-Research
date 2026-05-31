function summary = run_h3()
%RUN_H3  Pre-launch identifiability robustness sweep.

here = fileparts(mfilename('fullpath'));
addpath(here);

fprintf('=== H3 ROBUSTNESS TO PARAMETER ESTIMATION ERROR ===\n');
results = h3_wiggle_identify(struct());

% Also call the validator with a representative bench measurement
fprintf('\n=== VALIDATOR DEMO ===\n');
meas = struct( ...
    'slew_deg_per_s', 200, ...     % typical hobby servo geared down via linkage
    'servo_max_deg',  8, ...
    'p_est',          10, ...
    'keff_est',       8.0, ...
    'damp_est',       0.2);
addpath(fullfile(here, 'validator'));
val = recommend_envelope(meas);

summary.h3 = results;
summary.validator_demo = val;
end
