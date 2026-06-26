function smoke_validator()
%SMOKE_VALIDATOR  End-to-end test of validator + bench ingestion.
this = fileparts(mfilename('fullpath'));
proj = fileparts(this);
addpath(proj);
addpath(fullfile(proj, 'experiments'));
addpath(fullfile(proj, 'experiments', 'validator'));
addpath(fullfile(proj, 'tools'));

% Build a synthetic CSV matching the .ino plotter schema
N = 60;
commanded = 90 + 5*sin(0.1*(1:N)');
actual    = 90 + 5*sin(0.1*(1:N)' - 0.05);
zeroN     = zeros(N,1);
makeT = @(up, down) table(commanded, actual, zeroN, zeroN, ...
    200*ones(N,1), 180*ones(N,1), up*ones(N,1), down*ones(N,1), 92*ones(N,1), ...
    'VariableNames', {'commanded_deg','actual_deg_est','slew_deg_per_s', ...
    'slew_raw_counts_per_s','transition_avg_slew_deg_s', ...
    'transition_net_slew_deg_s','rolling_up_net_slew_deg_s', ...
    'rolling_down_net_slew_deg_s','endpoint_util_pct'});

mkdir_if_needed(fullfile(proj, 'data', 'bench'));

disp('================ A) HIGH-SLEW SERVO (RESCUE) ================');
writetable(makeT(220, 215), fullfile(proj,'data','bench','synth_serial_hi.csv'));
process_gimbal_bench_serial(fullfile(proj,'data','bench','synth_serial_hi.csv'), ...
    struct('p_est', 8.0));

disp(' ');
disp('================ B) MID-SLEW SERVO (FUNDAMENTAL) ================');
writetable(makeT(80, 75), fullfile(proj,'data','bench','synth_serial_mid.csv'));
process_gimbal_bench_serial(fullfile(proj,'data','bench','synth_serial_mid.csv'), ...
    struct('p_est', 8.0));

disp(' ');
disp('================ C) LOW-SLEW SERVO (INFEASIBLE) ================');
writetable(makeT(20, 18), fullfile(proj,'data','bench','synth_serial_lo.csv'));
process_gimbal_bench_serial(fullfile(proj,'data','bench','synth_serial_lo.csv'), ...
    struct('p_est', 8.0));
end

function mkdir_if_needed(p)
if ~exist(p, 'dir'), mkdir(p); end
end
