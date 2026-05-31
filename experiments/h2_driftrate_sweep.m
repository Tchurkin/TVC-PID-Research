function results = h2_driftrate_sweep(opts)
%H2_DRIFTRATE_SWEEP  Sweep instability drift RATE during burn.
% Start at low p0 (easy), ramp linearly to high p1 over t_burn, then hold.
% LQR sized at p_design. Look for "fast drift hurts" signature -- the
% critical drift-rate threshold above which mid-burn loss occurs.
%
% Plant: p(t) = p0 + (p1-p0) * min(t/t_burn, 1)
% Drift rate (1/s^2): (p1-p0)/t_burn
%
% Hypothesis: there exists a critical drift rate above which a controller
% designed for p_design fails despite being adequate for both endpoints
% instantaneously.

if nargin < 1, opts = struct(); end
def.damp       = 0.2;
def.keff       = 8.0;
def.tau_act    = 0.05;
def.slew_max   = 35;
def.u_max      = 5.0;            % near H1 optimum, leaves headroom
def.latency_n  = 2;
def.gyro_std   = 0.03;
def.dt         = 0.005;
def.t_end_pad  = 0.8;            % seconds AFTER burnout to observe collapse
def.theta0_set = deg2rad([1 3]); % small initial states (easy)
def.gust_std   = 2.0;
def.gust_tau   = 0.30;
def.seeds      = 1:12;
def.p0         = 4;
def.p1_grid    = [6 8 10 12 14 16];
def.t_burn_grid= [4.0 2.0 1.0 0.5 0.25 0.12];  % slow -> fast drift
def.design_mode= 'p0';           % 'p0' | 'mid' | 'p1'
def.theta_fail_rad = deg2rad(40);
def.outdir     = fullfile('experiments','results');
opts = apply_defaults(opts, def);

if ~exist(opts.outdir, 'dir'), mkdir(opts.outdir); end

nP = numel(opts.p1_grid); nT = numel(opts.t_burn_grid);
trials_per_cell = numel(opts.seeds) * numel(opts.theta0_set);

success_rate = zeros(nP, nT);
t_first_fail_med = nan(nP, nT);
drift_rate = zeros(nP, nT);

fprintf('H2 drift-rate sweep: p0=%g, p1 x %d, t_burn x %d, design=%s, u_max=%g\n', ...
    opts.p0, nP, nT, opts.design_mode, opts.u_max);
t_start = tic;

for ip = 1:nP
    p1 = opts.p1_grid(ip);
    switch opts.design_mode
        case 'p0',  p_design = opts.p0;
        case 'mid', p_design = 0.5*(opts.p0 + p1);
        case 'p1',  p_design = p1;
        otherwise, error('design_mode');
    end
    K = design_lqr_unstable(p_design, opts.damp, opts.keff);

    for it = 1:nT
        t_burn = opts.t_burn_grid(it);
        drift_rate(ip,it) = (p1 - opts.p0) / t_burn;
        t_end = t_burn + opts.t_end_pad;
        p_traj = @(t) opts.p0 + (p1 - opts.p0) * min(max(t / t_burn, 0), 1);

        wins = 0;
        tfails = [];
        for ith = 1:numel(opts.theta0_set)
            for s = opts.seeds
                rng(s + 100*ith + 7*ip + 31*it);
                pr = struct('p', opts.p0, 'damp', opts.damp, 'keff', opts.keff, ...
                    'tau_act', opts.tau_act, 'slew_max', opts.slew_max, ...
                    'u_max', opts.u_max, 'latency_n', opts.latency_n, ...
                    'gyro_std', opts.gyro_std, 'K', K, 'dt', opts.dt, ...
                    't_end', t_end, 'theta0', opts.theta0_set(ith), ...
                    'gust_std', opts.gust_std, 'gust_tau', opts.gust_tau, ...
                    'p_traj', p_traj);
                out = sim_unstable(pr);
                [st, ~] = classify_outcome(out, opts.theta_fail_rad);
                if st
                    wins = wins + 1;
                else
                    idx = find(abs(out.theta) > opts.theta_fail_rad, 1, 'first');
                    if ~isempty(idx)
                        tfails(end+1) = out.time(idx); %#ok<AGROW>
                    end
                end
            end
        end
        success_rate(ip,it) = wins / trials_per_cell;
        if ~isempty(tfails)
            t_first_fail_med(ip,it) = median(tfails);
        end
    end
end

fprintf('  done (%.1fs)\n', toc(t_start));

results.success_rate = success_rate;
results.t_first_fail_med = t_first_fail_med;
results.drift_rate = drift_rate;
results.p0 = opts.p0;
results.p1_grid = opts.p1_grid;
results.t_burn_grid = opts.t_burn_grid;
results.design_mode = opts.design_mode;
results.u_max = opts.u_max;
results.opts = opts;

fname = fullfile(opts.outdir, sprintf('h2_driftrate_%s_u%g.mat', opts.design_mode, opts.u_max));
save(fname, '-struct', 'results');
fprintf('Saved %s\n', fname);
end

function s = apply_defaults(s, d)
fn = fieldnames(d);
for i = 1:numel(fn)
    if ~isfield(s, fn{i}) || isempty(s.(fn{i}))
        s.(fn{i}) = d.(fn{i});
    end
end
end
