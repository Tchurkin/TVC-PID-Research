function id = sys_id_preflight(u_cmd, u_act, dt)
%SYS_ID_PREFLIGHT Identify actuator gain and lag from startup excitation.
%   id = SYS_ID_PREFLIGHT(u_cmd, u_act, dt) fits an ARX(2,2,1) model:
%       y(k) = a1*y(k-1) + a2*y(k-2) + b1*u(k-1) + b2*u(k-2)
%   where y is realized actuator acceleration and u is commanded acceleration.

u_cmd = u_cmd(:);
u_act = u_act(:);
N = min(numel(u_cmd), numel(u_act));
if N < 20
    error('sys_id_preflight:InsufficientData', 'Need at least 20 samples for preflight sys-id.');
end

u_cmd = u_cmd(1:N);
u_act = u_act(1:N);

k = 3:N;
Phi = [u_act(k-1), u_act(k-2), u_cmd(k-1), u_cmd(k-2)];
y = u_act(k);

% Regularized LS improves numerical stability when startup excitation is short.
lambda_reg = 1e-6;
theta = (Phi.' * Phi + lambda_reg * eye(size(Phi,2))) \ (Phi.' * y);
rank_phi = rank(Phi);

if rank_phi < size(Phi,2)
    % Fallback to simpler ARX(1,1,1) model when regressors are collinear.
    k1 = 2:N;
    Phi1 = [u_act(k1-1), u_cmd(k1-1)];
    y1 = u_act(k1);
    th1 = (Phi1.' * Phi1 + lambda_reg * eye(2)) \ (Phi1.' * y1);
    a1 = th1(1);
    a2 = 0;
    b1 = th1(2);
    b2 = 0;
    y_hat = Phi1 * th1;
    fit_rmse = sqrt(mean((y1 - y_hat).^2));
    pole_mag = min(0.999, max(0.05, abs(a1)));
    den_dc = 1 - a1;
    if abs(den_dc) < 1e-6
        den_dc = sign(den_dc + eps) * 1e-6;
    end
    gain_est = b1 / den_dc;
else
    a1 = theta(1);
    a2 = theta(2);
    b1 = theta(3);
    b2 = theta(4);

    y_hat = Phi * theta;
    fit_rmse = sqrt(mean((y - y_hat).^2));

    den_dc = 1 - a1 - a2;
    if abs(den_dc) < 1e-6
        den_dc = sign(den_dc + eps) * 1e-6;
    end
    gain_est = (b1 + b2) / den_dc;

    A = [1, -a1, -a2];
    rts = roots(A);
    if isempty(rts)
        pole_mag = 0.95;
    else
        [~, idx] = max(abs(rts));
        pole_mag = abs(rts(idx));
    end
    pole_mag = min(0.999, max(0.05, pole_mag));
end

tau_est = -dt / log(pole_mag);

id = struct();
id.a1 = a1;
id.a2 = a2;
id.b1 = b1;
id.b2 = b2;
id.gain_est = max(0.2, min(2.0, gain_est));
id.tau_est = max(0.03, min(2.0, tau_est));
id.fit_rmse = fit_rmse;
id.samples = N;
end
