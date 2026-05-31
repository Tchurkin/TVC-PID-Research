function summary = run_h2()
%RUN_H2  Time-varying instability discovery run.
% Tests whether a fixed (u_max, K) sized for pre-burn p0 fails as p rises
% during the burn. Reports: success surface vs (p0, p1, u_max) and median
% time-to-failure when it fails.

here = fileparts(mfilename('fullpath'));
addpath(here);

fprintf('=== H2 TIME-VARYING INSTABILITY EXPERIMENT ===\n');
results = h2_timevarying_sweep(struct());

p0g = results.p0_grid; p1g = results.p1_grid; ug = results.umax_grid;

% Pretty per-(p0,p1) print: success vs u_max
fprintf('\nSuccess-rate slices (rows=u_max, cols=p1) for each p0:\n');
for i0 = 1:numel(p0g)
    fprintf('\n  p0 = %g  (LQR designed at this p)\n', p0g(i0));
    fprintf('  u_max\\p1 |');
    for i1 = 1:numel(p1g), fprintf(' %5g', p1g(i1)); end
    fprintf('\n');
    for iu = 1:numel(ug)
        fprintf('  %6.2f   |', ug(iu));
        for i1 = 1:numel(p1g)
            fprintf(' %5.2f', results.success_rate(i0,i1,iu));
        end
        fprintf('\n');
    end
end

% Identify "mid-burn collapse" cells: success drops as p1 grows for fixed (p0, u_max)
fprintf('\nMid-burn collapse signature (success at p1=p0 vs p1=max):\n');
collapse_rows = {};
for i0 = 1:numel(p0g)
    for iu = 1:numel(ug)
        % Find p1 closest to p0 (no drift)
        [~, idx0] = min(abs(p1g - p0g(i0)));
        sr_nodrift = results.success_rate(i0, idx0, iu);
        sr_maxdrift = results.success_rate(i0, end, iu);
        delta = sr_maxdrift - sr_nodrift;
        tfm = results.t_first_fail_med(i0, end, iu);
        fprintf('  p0=%g  u_max=%5.2f   no-drift=%.2f  max-drift=%.2f  delta=%+.2f  med_t_fail=%.2fs\n', ...
            p0g(i0), ug(iu), sr_nodrift, sr_maxdrift, delta, tfm);
        collapse_rows(end+1, :) = {p0g(i0), ug(iu), sr_nodrift, sr_maxdrift, delta, tfm}; %#ok<AGROW>
    end
end

fid = fopen(fullfile(here, 'results', 'h2_collapse_summary.csv'), 'w');
fprintf(fid, 'p0,u_max,success_no_drift,success_max_drift,delta,med_t_first_fail_s\n');
for r = 1:size(collapse_rows,1)
    v = collapse_rows(r,:);
    if isnan(v{6}), tstr = 'NaN'; else, tstr = sprintf('%.4f', v{6}); end
    fprintf(fid, '%g,%.3f,%.4f,%.4f,%.4f,%s\n', v{1}, v{2}, v{3}, v{4}, v{5}, tstr);
end
fclose(fid);
fprintf('\nWrote results/h2_collapse_summary.csv\n');

summary = results;
end
