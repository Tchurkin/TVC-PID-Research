%% main_sts_gold_B.m
% B-first orchestration for STS submission artifacts.
% Runs Direction B full benchmark, optional compact Direction C companion,
% then builds council + scoreboard outputs and publication-grade graphs.

root_dir = fileparts(mfilename('fullpath'));

fprintf('=== STS GOLD PIPELINE (B-FIRST) ===\n');

%% Direction B (primary)
b_dir = fullfile(root_dir, 'Direction_B_RC_ADRC_PathFollow');
addpath(fullfile(b_dir, 'src'));

fprintf('\n[1/5] Tuning Direction B industry params...\n');
cd(b_dir);
tune_params_B_industry;

fprintf('[2/5] Running Direction B industry sweep...\n');
run_sweep_B_industry;

%% Direction C (companion, compact)
c_dir = fullfile(root_dir, 'supporting', 'Direction_C_Companion');
if exist(c_dir, 'dir')
    addpath(fullfile(c_dir, 'src'));
    fprintf('[3/5] Running Direction C companion sweep...\n');
    cd(c_dir);
    tune_params_C;
    fault_sweep;
else
    fprintf('[3/5] Direction C companion not found; skipping C sweep.\n');
end

%% Scoreboard + council + graphs
tools_dir = fullfile(root_dir, 'tools');
cd(tools_dir);
fprintf('[4/5] Building STS scoreboard + council assessment...\n');
build_sts_scoreboard;
council_assess_direction_B;

fprintf('[5/5] Generating STS-gold B graphs...\n');
generate_sts_gold_graphs_B;

cd(root_dir);
fprintf('\nPipeline complete. Review outputs/sts_gold and paper/B_Centered_STS_Gold_Draft.md\n');
