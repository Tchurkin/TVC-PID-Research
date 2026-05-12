%% Direction C — In-Flight TVC Actuator Degradation Detection + Autonomous Retuning
% ==================================================================================
% CLAIM: TVC rocket controllers are tuned to a bench-measured gimbal
%        characterization, but the gimbal degrades during flight (battery
%        voltage drop reduces servo torque, friction increases with
%        vibration, thermal effects soften gearbox compliance). Nobody
%        detects or compensates this at the student level — or at most
%        rocket scales. An online estimator tracks the current gimbal
%        bandwidth from the closed-loop residual; when it drops below a
%        threshold, the controller bandwidth is automatically reduced to
%        stay within the degraded actuator's capability.
%
% SOCIETAL FRAMING:
%   SpaceX Falcon 9 landing burn uses TVC as primary attitude authority.
%   An undetected actuator degradation at T-3s is catastrophic. For
%   small rockets, the same physics apply — it's just undiscovered.
%   This directly addresses the bench-to-flight gap.
%
% KEY EVIDENCE TARGET (semi-promising threshold):
%   Fault injected at T+3s: wn drops 36 -> 18 rad/s (50% degradation)
%   WITHOUT adaptation: pitch deviation > 15 deg or divergence
%   WITH adaptation triggered within 0.5s: pitch stays within 5 deg
%   Detection latency (samples to trigger): < 0.5s = semi-promising
%   False positive rate: 0 false triggers in no-fault runs
%
% SCRIPTS (fill in Phase 3):
%   src/actuator_health_monitor.m — online bandwidth estimator
%   src/adaptive_retune.m         — maps estimated wn to safe PID gains
%   src/fault_sweep.m             — injects faults and measures response
%
% SIMULINK MODEL:
%   sim/TVC_Rocket_Model_C.slx    — parent model + fault injection switch
%                                    + health monitor block
%
% SHARED ASSETS:
%   ../data/bench/assumed_gimbal_profile.csv  (baseline actuator params)
%   ../src/s2r_build_config.m                (s2r pipeline if reused)
%
% RUN ORDER:
%   1. fault_sweep  -> outputs/sweep_results_C.csv
%   2. main_run_C   (this file) plots and summarizes
% ==================================================================================

fprintf('=== Direction C: In-Flight TVC Actuator Degradation Detection + Autonomous Retuning ===\n\n');

%% Check that required scripts exist
src_dir = fullfile(fileparts(mfilename('fullpath')), 'src');
addpath(src_dir);

fprintf('=== Direction C: In-Flight TVC Actuator Degradation Detection + Autonomous Retuning ===\n\n');

%% Check that required scripts exist
required = { ...
    fullfile(fileparts(mfilename('fullpath')), 'src', 'actuator_health_monitor.m'), ...
    fullfile(fileparts(mfilename('fullpath')), 'src', 'adaptive_retune.m'), ...
    fullfile(fileparts(mfilename('fullpath')), 'src', 'fault_sweep.m'), ...
};

all_ready = true;
for k = 1:numel(required)
    if ~exist(required{k}, 'file')
        fprintf('  [STUB] %s  (not yet implemented)\n', required{k});
        all_ready = false;
    else
        fprintf('  [OK]   %s\n', required{k});
    end
end

fprintf('\n');
if all_ready
    fprintf('All scripts present. Running fault sweep...\n');
    fault_sweep;
else
    fprintf('Implementation pending. Fill in src/ scripts to enable sweep.\n');
    fprintf('\nPlanned fault sweep dimensions:\n');
    fprintf('  Fault type:   wn_drop_50pct | delay_increase | combined\n');
    fprintf('  Fault time:   T+1s | T+3s | T+5s\n');
    fprintf('  Adaptation:   disabled | enabled\n');
    fprintf('  Metric:       max_pitch_dev_deg | detection_latency_s | false_pos_count\n');
    fprintf('\nSemi-promising threshold:\n');
    fprintf('  WITH adaptation: max_pitch_dev_deg <= 5.0\n');
    fprintf('  WITHOUT adaptation: max_pitch_dev_deg > 15.0  (shows problem is real)\n');
    fprintf('  Detection latency <= 0.50 s\n');
    fprintf('  False positives in clean runs = 0\n');
end
