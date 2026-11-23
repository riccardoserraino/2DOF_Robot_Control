function [u, qp_it] = mpc_step(xhat, dhat, Eyw, yw, u_prev, Fq, Gq, Dm, E0, w, umax, qp_iter)
%#codegen
% MATLAB Function block: LTV MPC on contour / lag error, one QP per sample.
% Inputs (signals):
%   xhat   [4x1]  state estimate [phi; dphi; theta; dtheta]        (kalman_step)
%   dhat   [2x1]  input-disturbance estimate [V]                   (kalman_step)
%   Eyw    [4xN]  window of Ey_{k+j} = [n'; t'] * J  (column-major)  (refWindow)
%   yw     [2xN]  window of yref_{k+j} = Ey_{k+j} * q_ref           (refWindow)
%   u_prev [2x1]  input applied at the previous sample [V]  (after saturation, Unit Delay)
% Parameters: Fq = mpc.Fq, Gq = mpc.Gq, Dm = mpc.Dm, E0 = mpc.E0, w = mpc.w (struct),
%             umax = mpc.umax, qp_iter = mpc.qp_iter
% Outputs: u [2x1] voltage to apply [V],  qp_it = active-set iterations used
%
% Decision variable V = U + d (the plant sees u + d):  q_pred = Fq*x + Gq*V
%   cost = sum_j [ qc*e_c,j^2 + ql*e_l,j^2 ] + r*|dU|^2
%   e_j  = Ey_j * q_j - yref_j,    dU = Dm*V - E0*(u_prev + d)
%   -umax <= u = V - d <= umax   ->   d - umax <= V <= d + umax

N  = size(Eyw, 2);
nV = size(Gq, 2);
x  = xhat(:);  d = dhat(:);

% 1. Linearized contour/lag error along the horizon:  e = M*V + c
M = zeros(2*N, nV);
c = zeros(2*N, 1);
for j = 1:N
    r2 = 2*j-1:2*j;
    Ej = reshape(Eyw(:,j), 2, 2);
    M(r2, :) = Ej*Gq(r2, :);
    c(r2)    = Ej*(Fq(r2, :)*x) - yw(:,j);
end
Wv = repmat([w.qc; w.ql], N, 1);

% 2. QP:  min 0.5*V'*H*V + f'*V   s.t.  lb <= V <= ub
H = 2*(M'*(Wv.*M) + w.r*(Dm'*Dm));
f = 2*(M'*(Wv.*c) - w.r*(Dm'*(E0*(u_prev(:) + d))));
H = 0.5*(H + H');                                  % exact symmetry
lb = repmat(d - umax, nV/2, 1);
ub = repmat(d + umax, nV/2, 1);

[V, qp_it] = box_qp(H, f, lb, ub, qp_iter);

% 3. Receding horizon: apply only the first move
u = V(1:2) - d;
end

%% ------------------------------------------------------------------------
function [U, it] = box_qp(H, f, lb, ub, maxit)
% Exact solver for a small QP with box constraints (primal-dual active set).
% Starts from the unconstrained optimum: if it is feasible (no saturation),
% the answer is exact after one linear solve.
n  = numel(f);
U  = -(H\f);
it = 0;
lo = U < lb;
up = U > ub;
while (any(lo) || any(up)) && it < maxit
    it = it + 1;
    % Solve with the active variables fixed at their bound
    Hm = H;  rhs = -f;
    for i = 1:n
        if lo(i) || up(i)
            Hm(i, :) = 0;  Hm(i, i) = 1;
            if lo(i), rhs(i) = lb(i); else, rhs(i) = ub(i); end
        end
    end
    U = Hm\rhs;
    g = H*U + f;                                   % gradient = minus the multipliers
    % Keep a bound only if the gradient pushes outwards, add the violated ones
    lo_new = (U < lb - 1e-12) | (lo & g > 0);
    up_new = (U > ub + 1e-12) | (up & g < 0);
    if isequal(lo_new, lo) && isequal(up_new, up)
        break;                                     % KKT conditions satisfied
    end
    lo = lo_new;  up = up_new;
end
U = min(max(U, lb), ub);                           % safety clip
end
