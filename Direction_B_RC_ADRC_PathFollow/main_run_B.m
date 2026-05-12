%% Direction B — RC-Plane Actuator-Constrained ADRC Path Following
% =================================================================
% CLAIM: ArduPilot's L1 guidance law assumes the airframe can respond
%        instantly to heading corrections — no actuator bandwidth model,
%        no disturbance observer for wind. An ADRC layer over L1 estimates
%        crosswind as a lumped disturbance and pre-compensates the heading
%        command before it hits the actuator, reducing cross-track error
%        without increasing aileron control effort.
%
% SOCIETAL FRAMING:
%   Autonomous cargo/delivery drones rely on GPS track accuracy in wind.
%   Even a 40% reduction in cross-track error under 5 m/s crosswind
%   directly translates to landing zone accuracy and airspace safety.
%
% KEY EVIDENCE TARGET (semi-promising threshold):
%   Cross-track error in 5 m/s crosswind:
%     ADRC+L1 peak deviation <= 0.60 x L1 peak deviation (>= 40% improvement)
%     ADRC+L1 aileron variance <= 1.10 x L1 aileron variance (no thrashing)
%   If ADRC reduces error but thrashes the aileron => NOT promising.
%
% SCRIPTS (fill in Phase 4):
%   src/l1_guidance.m          — L1 guidance law baseline (ArduPilot equivalent)
%   src/adrc_layer.m           — ADRC heading compensator over L1
%   src/crosswind_scenario.m   — generates wind + waypoint track scenario
%   src/run_sweep_B.m          — 3 crosswind speeds x 2 controllers sweep
%
% SIMULINK MODEL:
%   sim/rc_lateral_model.slx   — minimal 2-state fixed-wing lateral model
%                                 (sideslip beta + roll phi, PT2 actuator)
%
% ALTERNATIVELY:
%   Pure MATLAB script simulation (no Simulink) — lateral dynamics are
%   simple enough to integrate with ode45 in a script. Faster to build.
%
% SHARED ASSETS:
%   ../Sources/  (ArduPilot L1 reference: AP_L1_Control)
%
% RUN ORDER:
%   1. run_sweep_B  -> outputs/sweep_results_B.csv
%   2. main_run_B   (this file) plots and summarizes
% =================================================================

fprintf('=== Direction B: RC-Plane Actuator-Constrained ADRC Path Following ===\n\n');

%% Check that required scripts exist
src_dir = fullfile(fileparts(mfilename('fullpath')), 'src');
addpath(src_dir);

fprintf('=== Direction B: RC-Plane Actuator-Constrained ADRC Path Following ===\n\n');

%% Check that required scripts exist
required = { ...
    fullfile(fileparts(mfilename('fullpath')), 'src', 'l1_guidance.m'), ...
    fullfile(fileparts(mfilename('fullpath')), 'src', 'adrc_layer.m'), ...
    fullfile(fileparts(mfilename('fullpath')), 'src', 'crosswind_scenario.m'), ...
    fullfile(fileparts(mfilename('fullpath')), 'src', 'run_sweep_B.m'), ...
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
    fprintf('All scripts present. Running sweep...\n');
    run_sweep_B;
else
    fprintf('Implementation pending. Fill in src/ scripts to enable sweep.\n');
    fprintf('\nPlanned sweep dimensions:\n');
    fprintf('  Crosswind speed: 2 m/s | 5 m/s | 8 m/s\n');
    fprintf('  Controller:      L1    | ADRC+L1\n');
    fprintf('  Metric:          crosstrack_rms_m | peak_crosstrack_m | aileron_var_deg2\n');
    fprintf('\nSemi-promising threshold: ADRC peak_crosstrack <= 0.60 x L1 peak_crosstrack\n');
    fprintf('                          ADRC aileron_var <= 1.10 x L1 aileron_var\n');
end
