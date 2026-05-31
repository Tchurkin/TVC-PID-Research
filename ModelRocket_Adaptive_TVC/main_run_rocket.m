function main_run_rocket()
%MAIN_RUN_ROCKET  Entry point — generates the keff spike demonstration.

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'src'));
addpath(fullfile(root_dir, 'tools'));

if ~exist(fullfile(root_dir, 'outputs', 'graphs'), 'dir')
    mkdir(fullfile(root_dir, 'outputs', 'graphs'));
end

generate_high_keff_demo(root_dir);
end
