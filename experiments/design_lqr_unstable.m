function K = design_lqr_unstable(p, damp, keff, Q, R)
%DESIGN_LQR_UNSTABLE  Continuous LQR for unstable inverted-pendulum-like plant.
%
%   xdot = A x + B u,  A = [0 1; p^2 -damp], B = [0; keff]
%   u = -K x  with K from care(A,B,Q,R)
%
% Falls back to a closed-form pole-placement if care unavailable.

if nargin < 4, Q = diag([400, 2]); end
if nargin < 5, R = 0.5; end

A = [0 1; p^2 -damp];
B = [0; keff];

try
    [~, K, ~] = icare(A, B, Q, R);
    K = K(:).';
catch
    % Pole placement fallback: place poles at -2p, -3p
    desired = [-2*p, -3*p];
    K = place(A, B, desired);
end
end
