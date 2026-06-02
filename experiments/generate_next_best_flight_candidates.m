function T = generate_next_best_flight_candidates()
%GENERATE_NEXT_BEST_FLIGHT_CANDIDATES
% Build a ranked candidate list for the next highest-information flights.

here = fileparts(mfilename('fullpath'));
results_dir = fullfile(here, 'results');
index_path = fullfile(results_dir, 'exp1_regime_index.csv');
exp4_path = fullfile(results_dir, 'exp4_first_correct_fidelity.csv');
exp5_path = fullfile(results_dir, 'exp5_design_lever_summary.csv');

if exist(index_path, 'file')
    idx = readtable(index_path, 'VariableNamingRule', 'preserve');
else
    idx = readtable(fullfile(results_dir, 'phase_diagram.csv'), 'VariableNamingRule', 'preserve');
    if ~ismember('rocket_id', idx.Properties.VariableNames)
        idx.rocket_id = compose("R%04d", (1:height(idx))');
    end
    if ~ismember('boundary_distance', idx.Properties.VariableNames)
        idx.boundary_distance = zeros(height(idx), 1);
    end
end

if exist(exp4_path, 'file')
    exp4 = readtable(exp4_path, 'VariableNamingRule', 'preserve');
else
    exp4 = table();
end

if exist(exp5_path, 'file')
    exp5 = readtable(exp5_path, 'VariableNamingRule', 'preserve');
else
    exp5 = table();
end

rows = cell(0, 8);

% 1) Boundary discriminators (highest value for regime claim).
frag = idx(strcmpi(string(idx.regime_label), 'FRAGILE'), :);
if ~isempty(frag)
    [~, ord] = sort(abs(frag.boundary_distance), 'ascend');
    pick = frag(ord(1:min(3, height(frag))), :);
    for i = 1:height(pick)
        rows(end + 1, :) = { ...
            sprintf('F_BOUNDARY_%02d', i), ...
            'Exp1 regime-boundary validation', ...
            0.95 - 0.05 * (i - 1), ...
            sprintf('Rocket %s near fragile boundary; keep hardware nominal', string(pick.rocket_id(i))), ...
            'theta,q,u_cmd,u_act,events,preflight_slew', ...
            'Observed outcome matches predicted fragile behavior (sensitive but recoverable)', ...
            'Observed outcome is consistently easy or consistently infeasible', ...
            string(pick.rocket_id(i))}; %#ok<AGROW>
    end
end

% 2) Fidelity-ladder discriminators.
if ~isempty(exp4)
    level_col = find_table_col(exp4, {'first_correct_fidelity_level'});
    if ~isempty(level_col)
        hard = exp4(exp4.(level_col) >= 4, :);
    else
        hard = table();
    end

    if ~isempty(hard)
        hard = sortrows(hard, level_col, 'descend');
        hard = hard(1:min(3, height(hard)), :);
        for i = 1:height(hard)
            rows(end + 1, :) = { ...
                sprintf('F_FIDELITY_%02d', i), ...
                'Exp4 minimum-fidelity claim', ...
                0.90 - 0.04 * (i - 1), ...
                sprintf('Rocket %s requiring high fidelity in sim replay', string(hard.rocket_id(i))), ...
                'theta,q,u_cmd,u_act,theta_ref,controller_gains', ...
                sprintf('Decision agreement appears first at L%d or above', hard.(level_col)(i) - 1), ...
                'Low-fidelity models already match observed decision', ...
                string(hard.rocket_id(i))}; %#ok<AGROW>
        end
    end
end

% 3) Design-lever validation flights.
if ~isempty(exp5)
    score_col = find_table_col(exp5, {'tradeoff_score'});
    best_col = find_table_col(exp5, {'best_design_lever'});
    imp_col = find_table_col(exp5, {'expected_improvement'});

    if ~isempty(score_col) && ~isempty(best_col) && ~isempty(imp_col)
        exp5 = sortrows(exp5, score_col, 'descend');
        pick = exp5(1:min(4, height(exp5)), :);
        for i = 1:height(pick)
            rows(end + 1, :) = { ...
                sprintf('F_LEVER_%02d', i), ...
                'Exp5 design-lever effectiveness', ...
                0.82 - 0.03 * (i - 1), ...
                sprintf('Apply %s on rocket %s', string(pick.(best_col)(i)), string(pick.rocket_id(i))), ...
                'paired before-after theta,q,u_cmd,u_act under same mission profile', ...
                sprintf('Measured deltas align with predicted improvement %.3f', pick.(imp_col)(i)), ...
                'No measurable improvement or opposite tradeoff direction', ...
                string(pick.rocket_id(i))}; %#ok<AGROW>
        end
    end
end

if isempty(rows)
    rows = {
        'F_INIT_01', 'Pipeline initialization', 0.50, ...
        'Nominal hardware, full telemetry stack', ...
        'theta,q,u_cmd,u_act,events', ...
        'Telemetry quality checks pass and baseline metrics reproducible', ...
        'Telemetry missing critical channels', ...
        'R0001'};
end

T = cell2table(rows, 'VariableNames', { ...
    'candidate_id', 'tested_claim', 'expected_information_gain', ...
    'required_hardware_configuration', 'required_measurements', ...
    'success_criterion', 'refutation_criterion', 'rocket_id'});

T = sortrows(T, 'expected_information_gain', 'descend');
out_path = fullfile(fileparts(here), 'next_best_flight_candidates.csv');
writetable(T, out_path);
fprintf('Saved: %s\n', out_path);
end


function name = find_table_col(T, candidates)
name = '';
if isempty(T)
    return;
end

vars = string(T.Properties.VariableNames);
vars_norm = lower(regexprep(vars, '[^a-z0-9]', ''));
for i = 1:numel(candidates)
    c = string(candidates{i});
    c_norm = lower(regexprep(c, '[^a-z0-9]', ''));
    idx = find(vars_norm == c_norm, 1, 'first');
    if ~isempty(idx)
        name = char(vars(idx));
        return;
    end
end
end
