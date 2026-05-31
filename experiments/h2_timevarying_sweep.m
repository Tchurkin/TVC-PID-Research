function results = h2_timevarying_sweep(opts)
%H2_TIMEVARYING_SWEEP  Sweep linear p(t) ramps during burn and test whether
% (a) the H1 finite-authority optimum u*_max shifts with p, and
% (b) a controller sized for nominal p fails when p(t) crosses a threshold.
%
% Ramp model:
%   p(t) = p0 + (p1 - p0) * clamp(t / t_burn, 0, 1)
%
% Hypothesis H2: A constant u_max sized for p0 (pre-burn) loses
% stabilization once p(t) exceeds an instability rate at which u*_max(p) > u_max.
% Equivalently, the fixed-envelope rocket has a deterministic mid-burn
% failure point predictable from H1 curves.

if nargin < 1, opts = struct(); end
def.damp       = 0.2;
def.keff       = 8.0;
def.tau_act    = 0.05;
def.slew_max   = 35;             % representative bench-feasible slew
def.latency_n  = 2;
def.gyro_std   = 0.03;
def.dt         = 0.005;
def.t_end      = 3.0;
def.t_burn     = 1.8;            % motor burn-out time
def.theta0_set = deg2rad([2 5 8]);
def.gust_std   = 2.5;
def.gust_tau   = 0.30;
def.seeds      = 1:8;
def.p0_grid    = [4 6 8];        % pre-burn instability
def.p1_grid    = [6 8 10 12 14]; % burnout instability
def.umax_grid  = [3.0 4.5 6.0 8.0 12.0];  % candidate fixed envelopes
def.theta_fail_rad = deg2rad(40);
def.outdir     = fullfile('experiments','results');
opts = apply_defaults(opts, def);

if ~exist(opts.outdir, 'dir'), mkdir(opts.outdir); end

n0 = numel(opts.p0_grid); n1 = numel(opts.p1_grid); nU = numel(opts.umax_grid);
trials_per_cell = numel(opts.seeds) * numel(opts.theta0_set);

success_rate = zeros(n0, n1, nU);
t_first_fail_med = nan(n0, n1, nU);  % median time of first |theta|>fail (sec)

fprintf('H2 time-varying sweep: %dx%dx%d (p0,p1,u_max), trials/cell=%d\n', ...
    n0, n1, nU, trials_per_cell);
t_start = tic;

for i0 = 1:n0
    p0 = opts.p0_grid(i0);
    for i1 = 1:n1
        p1 = opts.p1_grid(i1);
        % LQR sized at p0 (pre-flight): controller-blind to p drift
        K = design_lqr_unstable(p0, opts.damp, opts.keff);
        p_traj = @(t) p0 + (p1 - p0) * min(max(t / opts.t_burn, 0), 1);

        for iu = 1:nU
            wins = 0;
            tfails = [];
            for ith = 1:numel(opts.theta0_set)
                for s = opts.seeds
                    rng(s + 100*ith + 7*i0 + 31*i1);
                    pr = struct('p', p0, 'damp', opts.damp, 'keff', opts.keff, ...
                        'tau_act', opts.tau_act, 'slew_max', opts.slew_max, ...
                        'u_max', opts.umax_grid(iu), 'latency_n', opts.latency_n, ...
                        'gyro_std', opts.gyro_std, 'K', K, 'dt', opts.dt, ...
                        't_end', opts.t_end, 'theta0', opts.theta0_set(ith), ...
                        'gust_std', opts.gust_std, 'gust_tau', opts.gust_tau, ...
                        'p_traj', p_traj);
                    out = sim_unstable(pr);
                    [st, m] = classify_outcome(out, opts.theta_fail_rad);
                    if st
                        wins = wins + 1;
                    else
                        % time of first failure
                        idx = find(abs(out.theta) > opts.theta_fail_rad, 1, 'first');
                        if ~isempty(idx)
                            tfails(end+1) = out.time(idx); %#ok<AGROW>
                        end
                    end
                    m_unused = m; %#ok<NASGU>
                end
            end
            success_rate(i0,i1,iu) = wins / trials_per_cell;
            if ~isempty(tfails)
                t_first_fail_med(i0,i1,iu) = median(tfails);
            end
        end
    end
    fprintf('  p0=%g done (%.1fs elapsed)\n', p0, toc(t_start));
end

results.success_rate = success_rate;
results.t_first_fail_med = t_first_fail_med;
results.p0_grid = opts.p0_grid;
results.p1_grid = opts.p1_grid;
results.umax_grid = opts.umax_grid;
results.opts = opts;

fname = fullfile(opts.outdir, 'h2_timevarying.mat');
save(fname, '-struct', 'results');
fprintf('Saved %s (%.1fs total)\n', fname, toc(t_start));
end

function s = apply_defaults(s, d)
fn = fieldnames(d);
for i = 1:numel(fn)
    if ~isfield(s, fn{i}) || isempty(s.(fn{i}))
        s.(fn{i}) = d.(fn{i});
    end
end
end
