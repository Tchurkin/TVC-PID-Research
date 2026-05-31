function results = h1_coupling_sweep(opts)
%H1_COUPLING_SWEEP  Empirically map the stabilizability boundary in
% (slew_max, u_max) space for a fixed unstable plant, and compare to the
% predictions of single-limit theoretical bounds composed independently.
%
% Hypothesis H1: The joint stabilizable region is materially smaller than
% the intersection of the per-axis stabilizable regions, i.e. there is
% coupling between slew and saturation that single-limit theory ignores.
%
% Output: results struct with grids and outcome maps written to
%   experiments/results/h1_coupling_<p>.mat

if nargin < 1, opts = struct(); end
def.p          = 6.0;             % unstable pole rate (1/s); marginal amateur scale
def.damp       = 0.3;             % rate damping
def.keff       = 8.0;             % control effectiveness
def.tau_act    = 0.05;            % actuator lag
def.latency_n  = 1;               % sensor latency samples
def.gyro_std   = 0.015;           % rad/s
def.dt         = 0.005;
def.t_end      = 2.5;             % short-burn horizon
def.theta0_set = deg2rad([1 3 5 8 12]);  % initial perturbations
def.gust_std   = 0.30;            % rad/s^2
def.gust_tau   = 0.3;
def.seeds      = 1:6;
def.slew_grid  = linspace(3, 60, 18);    % units/s
def.umax_grid  = linspace(2, 18, 18);    % units
def.theta_fail_rad = deg2rad(35);
def.outdir     = fullfile('experiments','results');
opts = apply_defaults(opts, def);

if ~exist(opts.outdir, 'dir'), mkdir(opts.outdir); end

% LQR gain designed at NOMINAL (large) actuator authority so the controller
% itself is not the limit -- we are testing actuator-envelope limits.
K = design_lqr_unstable(opts.p, opts.damp, opts.keff);

nS = numel(opts.slew_grid);
nU = numel(opts.umax_grid);
nI = numel(opts.theta0_set);
nSeed = numel(opts.seeds);

success_rate = zeros(nS, nU);
theta_max_med = zeros(nS, nU);
trials_total  = nI * nSeed;

fprintf('H1 coupling sweep: p=%.2f, grid %dx%d, trials/cell=%d\n', ...
    opts.p, nS, nU, trials_total);

t_start = tic;
for is = 1:nS
    for iu = 1:nU
        wins = 0;
        thetas = zeros(trials_total,1);
        idx = 0;
        for ith = 1:nI
            for s = opts.seeds
                idx = idx + 1;
                rng(s + 100*ith);
                p = struct( ...
                    'p',        opts.p, ...
                    'damp',     opts.damp, ...
                    'keff',     opts.keff, ...
                    'tau_act',  opts.tau_act, ...
                    'slew_max', opts.slew_grid(is), ...
                    'u_max',    opts.umax_grid(iu), ...
                    'latency_n',opts.latency_n, ...
                    'gyro_std', opts.gyro_std, ...
                    'K',        K, ...
                    'dt',       opts.dt, ...
                    't_end',    opts.t_end, ...
                    'theta0',   opts.theta0_set(ith), ...
                    'gust_std', opts.gust_std, ...
                    'gust_tau', opts.gust_tau);
                out = sim_unstable(p);
                [stab, m] = classify_outcome(out, opts.theta_fail_rad);
                if stab, wins = wins + 1; end
                thetas(idx) = m.theta_max_deg;
            end
        end
        success_rate(is, iu) = wins / trials_total;
        theta_max_med(is, iu) = median(thetas);
    end
    fprintf('  slew row %2d/%2d done (%.1fs elapsed)\n', is, nS, toc(t_start));
end

results.opts          = opts;
results.K             = K;
results.slew_grid     = opts.slew_grid;
results.umax_grid     = opts.umax_grid;
results.success_rate  = success_rate;
results.theta_max_med = theta_max_med;
results.elapsed_s     = toc(t_start);

fname = sprintf('h1_coupling_p%02d.mat', round(opts.p*10));
save(fullfile(opts.outdir, fname), '-struct', 'results');
fprintf('Saved %s (%.1fs total)\n', fname, results.elapsed_s);
end

function s = apply_defaults(s, defs)
fns = fieldnames(defs);
for i = 1:numel(fns)
    if ~isfield(s, fns{i}), s.(fns{i}) = defs.(fns{i}); end
end
end
