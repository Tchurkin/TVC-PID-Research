function [label, code] = classify_regime(nominal_metrics, robustness, P)
%CLASSIFY_REGIME Map robustness/quality outcomes to EASY/FRAGILE/INFEASIBLE.

q = P.classification;

easy_quality = nominal_metrics.success_rate >= q.easy_success && ...
    nominal_metrics.u_cmd_sat_frac <= q.easy_u_sat_frac && ...
    nominal_metrics.slew_sat_frac <= q.easy_slew_sat_frac && ...
    nominal_metrics.rms_error_deg <= q.easy_rms_error_deg && ...
    nominal_metrics.settling_time_s <= q.easy_settling_time_s && ...
    nominal_metrics.oscillation_score <= q.easy_oscillation_score;

fragile_quality = nominal_metrics.success_rate >= q.fragile_success && ...
    nominal_metrics.rms_error_deg <= q.fragile_rms_error_deg;

if robustness >= q.easy_robustness && easy_quality
    label = "EASY";
    code = 2;
elseif robustness > q.infeasible_robustness && fragile_quality
    label = "FRAGILE";
    code = 1;
else
    label = "INFEASIBLE";
    code = 0;
end
