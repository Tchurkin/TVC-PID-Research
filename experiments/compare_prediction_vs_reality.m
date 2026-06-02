function metrics = compare_prediction_vs_reality(flightLogPath, varargin)
%COMPARE_PREDICTION_VS_REALITY Quantify digital twin replay agreement.
%
% Outputs:
%   rmse_theta_deg, rmse_q_deg_s, rmse_u_act
%   verdict_log, verdict_pred, decision_match

p = inputParser;
addRequired(p, 'flightLogPath', @ischar);
addParameter(p, 'resultsDir', '', @ischar);
parse(p, flightLogPath, varargin{:});

est = estimate_parameters(flightLogPath);
rep = replay_flight(flightLogPath, 'paramEstimate', est, 'resultsDir', p.Results.resultsDir);

th_err = rep.theta_pred - rep.theta_log;
q_err = rep.q_pred - rep.q_log;
u_err = rep.u_act_pred - rep.u_act_log;

metrics = struct();
metrics.rmse_theta_deg = rad2deg(sqrt(mean(th_err .^ 2, 'omitnan')));
metrics.rmse_q_deg_s = rad2deg(sqrt(mean(q_err .^ 2, 'omitnan')));
metrics.rmse_u_act = sqrt(mean(u_err .^ 2, 'omitnan'));
metrics.actuator_slew_rate_est = est.actuator_slew_rate;
metrics.backlash_est = est.backlash;
metrics.deadband_est = est.deadband;
metrics.actuator_lag_s_est = est.actuator_lag_s;
metrics.control_effectiveness_est = est.control_effectiveness;
metrics.aerodynamic_damping_est = est.aerodynamic_damping;

verdict_log = classify_trace(rep.theta_log);
verdict_pred = classify_trace(rep.theta_pred);
metrics.verdict_log = verdict_log;
metrics.verdict_pred = verdict_pred;
metrics.decision_match = strcmp(verdict_log, verdict_pred);

if ~isempty(p.Results.resultsDir)
    if ~exist(p.Results.resultsDir, 'dir')
        mkdir(p.Results.resultsDir);
    end
    outT = struct2table(metrics, 'AsArray', true);
    writetable(outT, fullfile(p.Results.resultsDir, 'prediction_vs_reality_summary.csv'));
end


function verdict = classify_trace(theta)
rms_deg = rad2deg(sqrt(mean(theta .^ 2, 'omitnan')));
peak_deg = rad2deg(max(abs(theta)));
if peak_deg > 60 || rms_deg > 20
    verdict = 'NOGO';
elseif peak_deg > 25 || rms_deg > 8
    verdict = 'MARGINAL';
else
    verdict = 'GO';
end
