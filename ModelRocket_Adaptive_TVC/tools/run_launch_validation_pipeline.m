function bundle = run_launch_validation_pipeline(flight_log_dir)
%RUN_LAUNCH_VALIDATION_PIPELINE Convenience entry point for post-flight proof.
%   BUNDLE = RUN_LAUNCH_VALIDATION_PIPELINE(FLIGHT_LOG_DIR)
%   reads firmware logs and generates summary CSV, markdown report, and plots.

root_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root_dir, 'src'));
addpath(fullfile(root_dir, 'tools'));

if nargin < 1 || isempty(flight_log_dir)
    flight_log_dir = fullfile(root_dir, 'data', 'flight_logs');
end

bundle = build_launch_validation_bundle(flight_log_dir, root_dir);
end