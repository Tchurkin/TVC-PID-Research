function results = h3_wiggle_identify(opts)
%H3_WIGGLE_IDENTIFY  Pre-launch "wiggle test" identifies plant parameters.
%
% Procedure:
%   1. Hold rocket on bench at theta ~ 0 (no gravity-instability term active, i.e. p_test = 0).
%   2. Apply chirp / PRBS to gimbal u_cmd; record (theta, theta_dot, u_act).
%   3. Fit linear ARX/grey-box: theta_ddot = -damp*qdot + keff*u_act + noise.
%      (p^2 term is zero on bench because rocket not free-flying.)
%   4. ALSO measure: slew_meas, u_max_meas from saturation events in step response.
%   5. p is NOT measurable on bench -- comes from airframe spreadsheet
%      (CG margin, normal-force slope, thrust). H3 quantifies how much
%      flight-prediction error comes from p-misestimate vs keff/slew estimates.
%
% This sweep simulates the bench identification + flight prediction loop and
% measures: identification accuracy AND end-to-end success-prediction accuracy.

if nargin < 1, opts = struct(); end
def.damp_true   = 0.20;
def.keff_true   = 8.0;
def.tau_act     = 0.05;
def.slew_true   = 35;
def.u_max_true  = 5.0;
def.p_true      = 10.0;
def.gyro_std    = 0.03;
def.dt          = 0.005;
def.t_bench     = 1.5;          % bench excitation duration
def.t_flight    = 2.5;
def.gust_std    = 2.5;
def.gust_tau    = 0.30;
def.seeds       = 1:20;

% Parameter-estimate noise levels (simulate user error)
def.p_err_set     = [0 0.10 0.25 0.50];   % multiplicative error on p
def.keff_err_set  = [0 0.10 0.25 0.50];
def.slew_err_set  = [0];                  % bench measures this well
def.theta_fail_rad = deg2rad(40);
def.outdir      = fullfile('experiments','results');
opts = apply_defaults(opts, def);
if ~exist(opts.outdir, 'dir'), mkdir(opts.outdir); end

n_p = numel(opts.p_err_set); n_k = numel(opts.keff_err_set);

% Ground-truth flight success at well-tuned controller
K_true = design_lqr_unstable(opts.p_true, opts.damp_true, opts.keff_true);
sr_true = flight_sweep(K_true, opts.p_true, opts.keff_true, opts.slew_true, ...
                       opts.u_max_true, opts);

fprintf('H3 ground truth (perfect knowledge): success_rate=%.3f\n', sr_true);

success = zeros(n_p, n_k);
authority_chosen = zeros(n_p, n_k);

for ip = 1:n_p
    for ik = 1:n_k
        p_est    = opts.p_true   * (1 + opts.p_err_set(ip));
        keff_est = opts.keff_true * (1 + opts.keff_err_set(ik));

        % H1 rule: pick u_max near the empirical optimum.
        % Heuristic fit from H1 lock data: u*_max ~ 0.06 * slew + 0.30 * p
        u_max_sel = max(2.5, min(opts.u_max_true*2, 0.06 * opts.slew_true + 0.30 * p_est));
        authority_chosen(ip,ik) = u_max_sel;

        % H2 rule: design LQR at conservative end -> use p_est (don't oversize)
        K_est = design_lqr_unstable(p_est, opts.damp_true, keff_est);

        % Flight with TRUE plant but ESTIMATED controller and ESTIMATED u_max
        sr = flight_sweep(K_est, opts.p_true, opts.keff_true, opts.slew_true, ...
                          u_max_sel, opts);
        success(ip,ik) = sr;
    end
end

results.success = success;
results.authority_chosen = authority_chosen;
results.sr_true = sr_true;
results.p_err_set = opts.p_err_set;
results.keff_err_set = opts.keff_err_set;
results.opts = opts;

fname = fullfile(opts.outdir, 'h3_robustness.mat');
save(fname, '-struct', 'results');
fprintf('Saved %s\n', fname);

fprintf('\nSuccess vs (rows=p_err, cols=keff_err):\n');
fprintf('   p_err\\keff_err |');
for ik = 1:n_k, fprintf(' %6.2f', opts.keff_err_set(ik)); end
fprintf('\n');
for ip = 1:n_p
    fprintf('   %+5.2f         |', opts.p_err_set(ip));
    for ik = 1:n_k
        fprintf(' %6.3f', success(ip,ik));
    end
    fprintf('\n');
end
fprintf('\nu_max selected by H1 rule (rows=p_err):\n');
for ip = 1:n_p
    fprintf('   p_err=%+5.2f  u_max_sel=%.2f\n', opts.p_err_set(ip), authority_chosen(ip,1));
end
end

function sr = flight_sweep(K, p, keff, slew, u_max, opts)
theta0_set = deg2rad([2 5 8]);
wins = 0;
for ith = 1:numel(theta0_set)
    for s = opts.seeds
        rng(s + 100*ith);
        pr = struct('p', p, 'damp', opts.damp_true, 'keff', keff, ...
            'tau_act', opts.tau_act, 'slew_max', slew, ...
            'u_max', u_max, 'latency_n', 2, ...
            'gyro_std', opts.gyro_std, 'K', K, 'dt', opts.dt, ...
            't_end', opts.t_flight, 'theta0', theta0_set(ith), ...
            'gust_std', opts.gust_std, 'gust_tau', opts.gust_tau);
        out = sim_unstable(pr);
        st = classify_outcome(out, opts.theta_fail_rad);
        if st, wins = wins + 1; end
    end
end
sr = wins / (numel(theta0_set) * numel(opts.seeds));
end

function s = apply_defaults(s, d)
fn = fieldnames(d);
for i = 1:numel(fn)
    if ~isfield(s, fn{i}) || isempty(s.(fn{i}))
        s.(fn{i}) = d.(fn{i});
    end
end
end
