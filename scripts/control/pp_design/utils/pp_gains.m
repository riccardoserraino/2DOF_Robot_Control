function g = pp_gains(m, p)
% PP_GAINS  Gains that put the three closed-loop poles exactly in p.
%
% HOW TO USE
%   g = pp_gains(mot(1), [-30+10i; -30-10i; -7]);
%   g.k1, g.k2  state-feedback gains (angle, speed)      g.Kx = [k1 k2]
%   g.ki        integral gain
%   g.L         observer gain (2 x 1)
%   g.beta_star reference weight that cancels the slowest real pole
%   g.beta      the two choices by name: g.beta.star, g.beta.one
%
% THEORY (pp_design_theory.pdf, section 3)
%   state z = [theta; omega; eI],  control u = -K*z  with  K = [k1, k2, ki]
%   closed loop  z' = (Ai - Bi*K) z.   acker(Ai, Bi, p) returns the K that makes the
%   eigenvalues of Ai - Bi*K equal to p (Ackermann's formula). It gives exactly the
%   same numbers as the polynomial matching of the theory:
%       k1 = tau*a1/b,   k2 = (tau*a2 - 1)/b,   ki = -tau*a0/b

%% 1. Controller gains
K = acker(m.Ai, m.Bi, p);
g.k1 = K(1);
g.k2 = K(2);
g.ki = K(3);
g.Kx = [g.k1, g.k2];
g.p  = p(:);                         % the poles as a column

%% 2. Reference weight beta (theory pdf, section 5)
% the step is  theta/r = (beta*a1*s + a0)/(s^3 + a2*s^2 + a1*s + a0): one zero at -a0/(beta*a1)
a  = real(poly(p));                  % [1, a2, a1, a0]
a1 = a(3);
a0 = a(4);
real_poles   = real(p(abs(imag(p)) < 1e-9));
lambda_slow  = min(abs(real_poles));        % speed of the slowest real pole
g.beta_star  = a0/(a1*lambda_slow);         % puts the zero on that pole
g.beta.star  = g.beta_star;
g.beta.one   = 1;

%% 3. Observer gain (theory pdf, section 3)
% observer poles: 6 times the fastest controller pole (the estimate must be faster than
% the loop), but at most 250 rad/s = 0.5/Ts (the digital controller cannot do more).
% The second observer pole is 25 % faster (acker needs two poles, they can also be equal).
o1 = min(6*max(abs(p)), 250);
o2 = 1.25*o1;
g.p_obs = [-o1; -o2];
% eig(A - L*C) = observer poles. acker works on (A', C') because eig(A - L*C) = eig(A' - C'*L')
g.L = acker(m.A', m.C', g.p_obs)';
end