function [xhat, dhat] = kalman_step(y, u_prev, Aa, Ba, Ca, L, x0)
%#codegen
% MATLAB Function block: steady-state Kalman filter with input-disturbance states.
%   y      [2x1] measured joint angles [rad]  (encoder deg * pi/180)
%   u_prev [2x1] input APPLIED at the previous sample [V] (after the saturation, Unit Delay)
%   Aa, Ba, Ca, L, x0 : PARAMETERS  (est.Aa, est.Ba, est.Ca, est.L, initial state [6x1])
% Outputs (estimate at time k, after using y_k):
%   xhat [4x1] = [phi; dphi; theta; dtheta]     dhat [2x1] input disturbance [V]
% Model: xa = [x; d], x_{k+1} = A x + B (u + d), d_{k+1} = d, y = [phi; theta]

persistent xa
if isempty(xa)
    xa = x0;                       % start: robot at home, at rest, no disturbance
else
    xa = Aa*xa + Ba*u_prev;        % time update with the input actually applied at k-1
end
xa = xa + L*(y - Ca*xa);           % measurement update with y_k

xhat = xa(1:4);
dhat = xa(5:6);
end
