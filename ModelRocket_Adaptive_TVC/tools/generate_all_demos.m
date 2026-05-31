function generate_all_demos(root_dir)
%GENERATE_ALL_DEMOS  Run every paper figure end-to-end.

if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));
addpath(fullfile(root_dir, 'tools'));

fprintf('\n========== Slew degradation demo (HEADLINE) ==========\n');
generate_slew_demo(root_dir);

fprintf('\n========== Motor authority decay demo (SECONDARY) ==========\n');
generate_high_keff_demo(root_dir);

fprintf('\n========== Nominal recovery sanity check ==========\n');
generate_nominal_recovery_demo(root_dir);

fprintf('\n========== Multi-seed robustness sweep ==========\n');
generate_robustness_sweep(root_dir);

fprintf('\n========== 2-DOF cross-axis demo (yaw slew + pitch coupling) ==========\n');
generate_2dof_demo(root_dir);

fprintf('\n========== Realism stress test (noisy MEMS + Dryden gusts + servo nonlinearities) ==========\n');
generate_realism_demo(root_dir);

fprintf('\n========== Joint ablation study (what actually drives robustness) ==========\n');
generate_joint_ablation_demo(root_dir);

fprintf('\n========== Joint failure boundary + root-cause map ==========\n');
generate_joint_failure_boundary(root_dir);

fprintf('\n========== Safety-certified joint adaptation evidence ==========\n');
generate_joint_safety_cert_demo(root_dir);

fprintf('\nAll demos generated.\n');
end
