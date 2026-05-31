% run_boundary_topology_scan.m
% Analyze the structure of the success boundary produced by
% run_multidim_scaling_explore.m.
%
% If the boundary has holes, multiple success/fail transitions along an
% axis, or many local monotonicity violations, then the high-D failure
% surface is richer than a simple smooth threshold.

clear; clc;
here = fileparts(mfilename('fullpath'));
csv = fullfile(here, 'results', 'multidim_scaling_cells.csv');

if ~isfile(csv)
    error('run_boundary_topology_scan:MissingInput', ...
        'Missing %s. Run run_multidim_scaling_explore first.', csv);
end

T = readtable(csv);
T.success = T.best_success >= 0.50;

fprintf('=== BOUNDARY TOPOLOGY SCAN ===\n');
fprintf('Loaded %d cells\n', height(T));

summary_names = {};
summary_vals = [];

% Monotonicity along slew: success should not get worse as slew increases.
[slew_pairs, slew_viol] = count_axis_violations(T, {'p_unstable','latency_steps','gyro_noise_std'}, 'slew_code_units', +1);
[p_pairs, p_viol]       = count_axis_violations(T, {'latency_steps','gyro_noise_std','slew_code_units'}, 'p_unstable', -1);
[lat_pairs, lat_viol]   = count_axis_violations(T, {'p_unstable','gyro_noise_std','slew_code_units'}, 'latency_steps', -1);
[noi_pairs, noi_viol]   = count_axis_violations(T, {'p_unstable','latency_steps','slew_code_units'}, 'gyro_noise_std', -1);

summary_names(end+1:end+8) = { ...
    'slew_pairs', 'slew_violation_rate', ...
    'p_pairs', 'p_violation_rate', ...
    'latency_pairs', 'latency_violation_rate', ...
    'noise_pairs', 'noise_violation_rate'};
summary_vals(end+1:end+8) = [ ...
    slew_pairs, slew_viol / max(slew_pairs,1), ...
    p_pairs,    p_viol   / max(p_pairs,1), ...
    lat_pairs,  lat_viol / max(lat_pairs,1), ...
    noi_pairs,  noi_viol / max(noi_pairs,1)];

% Transition count along slew slices: >1 means island/hole behavior.
slice_rows = [];
p_vals = unique(T.p_unstable)';
lat_vals = unique(T.latency_steps)';
noi_vals = unique(T.gyro_noise_std)';
slew_vals = sort(unique(T.slew_code_units))';

multi_transition_count = 0;
hole_count = 0;

for il = 1:numel(lat_vals)
    for in = 1:numel(noi_vals)
        H = nan(numel(p_vals), numel(slew_vals));
        for ip = 1:numel(p_vals)
            for is = 1:numel(slew_vals)
                mask = T.p_unstable == p_vals(ip) & ...
                       T.latency_steps == lat_vals(il) & ...
                       abs(T.gyro_noise_std - noi_vals(in)) < 1e-12 & ...
                       T.slew_code_units == slew_vals(is);
                if any(mask)
                    H(ip,is) = double(T.success(find(mask,1)));
                end
            end
            row = H(ip,:);
            valid = isfinite(row);
            if nnz(valid) >= 2
                rowv = row(valid);
                transitions = nnz(diff(rowv) ~= 0);
                if transitions > 1
                    multi_transition_count = multi_transition_count + 1;
                end
                slice_rows(end+1,:) = [lat_vals(il), noi_vals(in), p_vals(ip), transitions]; %#ok<SAGROW>
            end
        end

        % Hole test in each p-slew slice: failed interior cell fully surrounded by success.
        for ip = 2:(numel(p_vals)-1)
            for is = 2:(numel(slew_vals)-1)
                if H(ip,is) == 0 && H(ip-1,is) == 1 && H(ip+1,is) == 1 && H(ip,is-1) == 1 && H(ip,is+1) == 1
                    hole_count = hole_count + 1;
                end
            end
        end
    end
end

summary_names(end+1:end+3) = {'multi_transition_slices', 'hole_count', 'total_cells'};
summary_vals(end+1:end+3) = [multi_transition_count, hole_count, height(T)];

summaryT = table(summary_names', summary_vals', 'VariableNames', {'metric','value'});
writetable(summaryT, fullfile(here, 'results', 'boundary_topology_summary.csv'));

sliceT = array2table(slice_rows, 'VariableNames', ...
    {'latency_steps','gyro_noise_std','p_unstable','slew_transition_count'});
writetable(sliceT, fullfile(here, 'results', 'boundary_topology_slices.csv'));

fprintf('Slew monotonicity violation rate:    %.3f (%d/%d)\n', slew_viol / max(slew_pairs,1), slew_viol, slew_pairs);
fprintf('p monotonicity violation rate:       %.3f (%d/%d)\n', p_viol / max(p_pairs,1), p_viol, p_pairs);
fprintf('Latency monotonicity violation rate: %.3f (%d/%d)\n', lat_viol / max(lat_pairs,1), lat_viol, lat_pairs);
fprintf('Noise monotonicity violation rate:   %.3f (%d/%d)\n', noi_viol / max(noi_pairs,1), noi_viol, noi_pairs);
fprintf('Multi-transition slices: %d\n', multi_transition_count);
fprintf('Interior holes found:    %d\n', hole_count);
fprintf('Saved: experiments/results/boundary_topology_summary.csv\n');
fprintf('Saved: experiments/results/boundary_topology_slices.csv\n');
fprintf('DONE.\n');


function [pair_count, violation_count] = count_axis_violations(T, group_cols, axis_col, direction)
% direction = +1 means success should be nondecreasing with axis.
% direction = -1 means success should be nonincreasing with axis.

pair_count = 0;
violation_count = 0;

[G, groups] = findgroups(T(:, group_cols)); %#ok<ASGLU>
for g = 1:max(G)
    sliceT = sortrows(T(G == g, :), axis_col);
    vals = sliceT.success;
    for i = 1:(height(sliceT) - 1)
        pair_count = pair_count + 1;
        delta = vals(i+1) - vals(i);
        if direction > 0
            if delta < 0
                violation_count = violation_count + 1;
            end
        else
            if delta > 0
                violation_count = violation_count + 1;
            end
        end
    end
end
end