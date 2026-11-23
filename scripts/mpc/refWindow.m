function [Qw, Eyw, yw] = refWindow(k, Qref, Eyflat, yref, N)
%#codegen
% MATLAB Function block: preview window r_{k+1} ... r_{k+N} for the MPC.
%   k      : current sample index, 0-based (input port, e.g. from a counter)
%   Qref   = ref.q       [2 x n]  joint reference [rad]
%   Eyflat = ref.Eyflat  [4 x n]  contour/lag maps Ey = [n'; t']*J, column-major
%   yref   = ref.yref    [2 x n]  Ey*q_ref
%   N      = mpc.N
%   (Qref, Eyflat, yref, N: declare them as PARAMETERS in the Ports and Data Manager)
% Outputs, one column per horizon step j = 1..N:
%   Qw [2xN], Eyw [4xN] (reshape column j to 2x2 to get Ey_{k+j}), yw [2xN]

nS  = size(Qref, 2);
idx = min((k+2):(k+N+1), nS);   % MATLAB index 1 = sample k = 0  ->  r_{k+1} is index k+2

Qw  = Qref(:, idx);
Eyw = Eyflat(:, idx);
yw  = yref(:, idx);
end
