function benchmark_single_channel_realistic(root_dir)
if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(root_dir, 'src'));

cfg = rocket_defaults();
seeds = 1:3;
amps = [1.0 2.0 3.0 4.0];
ctrls = {'ADAPTIVE_KEFF_LQR','SLEW_ADAPTIVE'};

for c = 1:numel(ctrls)
    ctrl = ctrls{c};
    vals_slew = [];
    vals_keff = [];
    worst = 0;

    for amp = amps
        scs = rocket_scenario("SLEW_DEGRADATION", cfg);
        scs.t_end = 10.0;
        scs.disturbance_amp = amp;
        scs.disturbance_freq_hz = 1.5;

        sck = rocket_scenario("HIGH_KEFF_FAULT", cfg);
        sck.t_end = 10.0;
        sck.disturbance_amp = 0.30;
        sck.disturb_scale_post = 4.0 * amp;

        for sd = seeds
            outs = simulate_case_realistic(ctrl, scs, cfg, sd, struct());
            rs = rad2deg(rms(outs.theta(outs.time >= scs.fault_time)));
            vals_slew(end+1) = rs; %#ok<AGROW>

            outk = simulate_case_realistic(ctrl, sck, cfg, sd, struct());
            rk = rad2deg(rms(outk.theta(outk.time >= sck.fault_time)));
            vals_keff(end+1) = rk; %#ok<AGROW>

            worst = max([worst, rs, rk]);
        end
    end

    fprintf('%s: mean_slew=%.2f, mean_keff=%.2f, p95_slew=%.2f, p95_keff=%.2f, worst=%.2f\n', ...
        ctrl, mean(vals_slew), mean(vals_keff), prctile(vals_slew,95), prctile(vals_keff,95), worst);
end
end
