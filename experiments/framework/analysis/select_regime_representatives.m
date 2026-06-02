function reps = select_regime_representatives(phaseT, target_per_regime)
%SELECT_REGIME_REPRESENTATIVES Pick boundary-aware representative cells.

if nargin < 2 || isempty(target_per_regime)
    target_per_regime = 1;
end

regimes = {'EASY', 'FRAGILE', 'INFEASIBLE'};
rows = cell(0, 12);
for i = 1:numel(regimes)
    regime = regimes{i};
    sub = phaseT(strcmpi(string(phaseT.regime_label), regime), :);
    if isempty(sub)
        continue;
    end

    others = phaseT(~strcmpi(string(phaseT.regime_label), regime), :);
    X = normalize_rows([double(sub.p_unstable), double(sub.servo_slew_deg_s), double(sub.max_gimbal_deg)]);
    boundary = closeness_to_set(sub, others);
    n_pick = min(height(sub), max(1, target_per_regime));
    picked = greedy_boundary_spread_pick(X, boundary, n_pick, 0.75);

    for k = 1:numel(picked)
        j = picked(k);
        rows(end + 1, :) = { ...
            string(sub.regime_label{j}), sub.p_unstable(j), sub.servo_slew_deg_s(j), sub.max_gimbal_deg(j), ...
            sub.best_u_max_frac(j), sub.best_Kp(j), sub.best_Kd(j), sub.nominal_success_rate(j), ...
            sub.under_success_rate(j), sub.over_success_rate(j), sub.robustness(j), boundary(j)}; %#ok<AGROW>
    end
end

reps = cell2table(rows, 'VariableNames', { ...
    'regime_label', 'p_unstable', 'servo_slew_deg_s', 'max_gimbal_deg', ...
    'best_u_max_frac', 'best_Kp', 'best_Kd', 'nominal_success_rate', ...
    'under_success_rate', 'over_success_rate', 'robustness', 'boundary_score'});
reps = sortrows(reps, {'regime_label', 'boundary_score'}, {'ascend', 'descend'});
end


function picked = greedy_boundary_spread_pick(X, boundary, n_pick, boundary_weight)
n = size(X, 1);
picked = zeros(n_pick, 1);
available = true(n, 1);
[~, first_idx] = max(boundary);
picked(1) = first_idx;
available(first_idx) = false;

for k = 2:n_pick
    avail_idx = find(available);
    if isempty(avail_idx)
        picked = picked(1:k-1);
        break;
    end

    b = normalize01(boundary(avail_idx));
    dmin = zeros(numel(avail_idx), 1);
    selX = X(picked(1:k-1), :);
    for i = 1:numel(avail_idx)
        di = sum(abs(selX - X(avail_idx(i), :)), 2);
        dmin(i) = min(di);
    end
    dmin = normalize01(dmin);
    score = boundary_weight * b + (1 - boundary_weight) * dmin;
    [~, imax] = max(score);
    idx = avail_idx(imax);
    picked(k) = idx;
    available(idx) = false;
end
end


function y = normalize_rows(X)
mu = min(X, [], 1);
sp = max(X, [], 1) - mu;
sp(sp < 1e-9) = 1;
y = (X - mu) ./ sp;
end


function y = normalize01(x)
x = double(x(:));
mn = min(x);
mx = max(x);
if abs(mx - mn) < 1e-9
    y = zeros(size(x));
else
    y = (x - mn) / (mx - mn);
end
end


function score = closeness_to_set(sub, others)
if isempty(others)
    score = ones(height(sub), 1);
    return;
end

A = [double(sub.p_unstable), double(sub.servo_slew_deg_s), double(sub.max_gimbal_deg)];
B = [double(others.p_unstable), double(others.servo_slew_deg_s), double(others.max_gimbal_deg)];
score = zeros(height(sub), 1);
for i = 1:height(sub)
    d = sum(abs(B - A(i, :)), 2);
    score(i) = 1 / max(1e-9, min(d));
end
end