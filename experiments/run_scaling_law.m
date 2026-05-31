% run_scaling_law.m
% Mine validator_calibration.csv for a quantitative scaling law:
%   slew_min(p) = minimum slew rate needed to keep mean stabilization success > thresh
% If this is a clean power law (slew_min ~ p^alpha) with low residual, that is a
% genuinely novel quantitative result for amateur TVC.
%
% Also extracts a Pareto frontier: (servo slew, max stable p) at fixed success.

clear; clc;

here = fileparts(mfilename('fullpath'));
csv  = fullfile(here, 'results', 'validator_calibration.csv');
T = readtable(csv);

fprintf('Loaded %d configs\n', height(T));
fprintf('p_unstable values: %s\n', mat2str(unique(T.p_unstable)'));
fprintf('slew values:       %s\n', mat2str(unique(T.slew_deg_per_s)'));
fprintf('umax values:       %s\n\n', mat2str(unique(T.umax_deg)'));

%% Build a binned heatmap: mean success per (p, slew) over all u_max
ps   = sort(unique(T.p_unstable));
sls  = sort(unique(T.slew_deg_per_s));
H = nan(numel(ps), numel(sls));
N = zeros(numel(ps), numel(sls));
for i = 1:numel(ps)
    for j = 1:numel(sls)
        mask = T.p_unstable == ps(i) & T.slew_deg_per_s == sls(j);
        if any(mask)
            H(i,j) = mean(T.actual_success(mask));
            N(i,j) = sum(mask);
        end
    end
end

fprintf('Mean success heatmap (rows = p, cols = slew):\n');
fprintf('         ');
for j=1:numel(sls); fprintf('  slew=%2d', sls(j)); end
fprintf('\n');
for i = 1:numel(ps)
    fprintf('  p=%2d  ', ps(i));
    for j = 1:numel(sls)
        if isnan(H(i,j)); fprintf('   .   '); else; fprintf('  %.2f ', H(i,j)); end
    end
    fprintf('\n');
end

%% Scaling law fit: for each p, what is the minimum slew at which success >= thresh?
% We do logistic regression of success vs slew at each p, then invert for the slew
% that gives success = 0.5 (the "phase boundary slew"), since pure thresholding is
% noisy with so few samples per cell.
thresh = 0.5;
slew_boundary = nan(numel(ps),1);
for i = 1:numel(ps)
    mask = T.p_unstable == ps(i);
    s = T.slew_deg_per_s(mask);
    y = T.actual_success(mask);
    if numel(unique(s)) < 2 || numel(unique(y > thresh)) < 2
        continue;
    end
    % Logistic regression: P(success>thresh) = sigmoid(b0 + b1*log(slew))
    yb = double(y >= thresh);
    X  = [ones(numel(s),1), log(s)];
    try
        b = glmfit(X(:,2), yb, 'binomial', 'link', 'logit');
        % b(1) intercept, b(2) slope on log(slew)
        % Boundary at p=0.5: b(1) + b(2)*log(s*) = 0 -> s* = exp(-b(1)/b(2))
        if b(2) > 0
            slew_boundary(i) = exp(-b(1)/b(2));
        end
    catch
        % fall back to median slew of the >= thresh group
        if any(yb==1); slew_boundary(i) = median(s(yb==1)); end
    end
end

fprintf('\nPhase-boundary slew (deg/s) per p (logistic fit at success>=%.1f):\n', thresh);
for i = 1:numel(ps)
    if isnan(slew_boundary(i))
        fprintf('  p=%2d : insufficient data\n', ps(i));
    else
        fprintf('  p=%2d : %6.2f deg/s\n', ps(i), slew_boundary(i));
    end
end

% Fit power law slew_min = a * p^alpha
valid = ~isnan(slew_boundary);
if sum(valid) >= 3
    lp = log(ps(valid));
    ls = log(slew_boundary(valid));
    coef = [ones(sum(valid),1), lp] \ ls;
    log_a = coef(1); alpha = coef(2); a = exp(log_a);

    pred = a * ps(valid).^alpha;
    ss_res = sum((slew_boundary(valid) - pred).^2);
    ss_tot = sum((slew_boundary(valid) - mean(slew_boundary(valid))).^2);
    R2 = 1 - ss_res/ss_tot;

    fprintf('\n=== POWER LAW FIT ===\n');
    fprintf('  slew_min(p) = %.3f * p^%.3f   (R^2 = %.3f)\n', a, alpha, R2);
    fprintf('  Predicted slew_min:\n');
    for i = 1:numel(ps)
        if valid(i)
            fprintf('    p=%2d  observed=%6.2f  predicted=%6.2f  err=%+5.2f\n', ...
                ps(i), slew_boundary(i), a*ps(i)^alpha, slew_boundary(i)-a*ps(i)^alpha);
        end
    end

    % Save
    outT = table(ps(valid), slew_boundary(valid), a*ps(valid).^alpha, ...
        'VariableNames', {'p_unstable','slew_min_observed','slew_min_predicted_powerlaw'});
    writetable(outT, fullfile(here,'results','scaling_law_slew_vs_p.csv'));

    metaT = table(a, alpha, R2, 'VariableNames', {'coef_a','exponent_alpha','R2'});
    writetable(metaT, fullfile(here,'results','scaling_law_fit.csv'));

    fprintf('\nSaved: experiments/results/scaling_law_slew_vs_p.csv\n');
    fprintf('Saved: experiments/results/scaling_law_fit.csv\n');
else
    fprintf('Not enough valid p points to fit a power law.\n');
end

%% Pareto frontier: at fixed success threshold, plot (slew, max stable p)
% For each slew bin, find the maximum p that achieves mean success >= 0.7
thresh_p = 0.7;
pareto_p = nan(numel(sls),1);
for j = 1:numel(sls)
    achievable = ps( ~isnan(H(:,j)) & H(:,j) >= thresh_p );
    if ~isempty(achievable); pareto_p(j) = max(achievable); end
end

fprintf('\nPareto frontier (max stable p_unstable at mean success >= %.1f):\n', thresh_p);
for j = 1:numel(sls)
    if isnan(pareto_p(j))
        fprintf('  slew=%2d : no p achieves\n', sls(j));
    else
        fprintf('  slew=%2d deg/s : p_max = %d\n', sls(j), pareto_p(j));
    end
end

paretoT = table(sls, pareto_p, 'VariableNames', {'slew_deg_per_s','max_stable_p_unstable'});
writetable(paretoT, fullfile(here,'results','pareto_servo_vs_p.csv'));
fprintf('Saved: experiments/results/pareto_servo_vs_p.csv\n');

fprintf('\nDONE.\n');
