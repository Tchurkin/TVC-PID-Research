function [state, diag] = rls_estimator(y_k, y_prev, u_prev, state, params, dt)
%RLS_ESTIMATOR Online ARX(1,1,1) estimator for actuator realization.
%   Model: y(k) = a*y(k-1) + b*u(k-1)

if nargin < 6
    error('rls_estimator:BadInput', 'Expected y_k, y_prev, u_prev, state, params, dt.');
end

if ~isfield(state, 'theta') || isempty(state.theta)
    state.theta = [0.90; 0.08];
end
if ~isfield(state, 'P') || isempty(state.P)
    state.P = 50 * eye(2);
end
if ~isfield(params, 'lambda_rls')
    params.lambda_rls = 0.98;
end

phi = [y_prev; u_prev];
y_hat = state.theta.' * phi;
err = y_k - y_hat;

denom = params.lambda_rls + phi.' * state.P * phi;
K = (state.P * phi) / max(1e-9, denom);
state.theta = state.theta + K * err;
state.P = (state.P - K * phi.' * state.P) / params.lambda_rls;

a_est = min(0.999, max(0.05, state.theta(1)));
b_est = state.theta(2);
gain_est = b_est / max(1e-6, 1 - a_est);

tau_est = -dt / log(a_est);
state.gain_est = max(0.2, min(2.0, gain_est));
state.tau_est = max(0.03, min(2.0, tau_est));
state.y_hat = y_hat;
state.err = err;

diag = struct();
diag.a_est = a_est;
diag.b_est = b_est;
diag.gain_est = state.gain_est;
diag.tau_est = state.tau_est;
diag.pred_err = err;
end
