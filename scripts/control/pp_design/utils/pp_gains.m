function g = pp_gains(m, p, req)
% PP_GAINS  Gains that put the three closed-loop poles exactly in p, and the observer.
%
% HOW TO USE
%   g = pp_gains(mot(1), [-30+10i; -30-10i; -7], req);
%   g.k1, g.k2  state-feedback gains (angle, speed)      g.Kx = [k1 k2]
%   g.ki        integral gain
%   g.L         observer gain (2 x 1),  g.p_obs its two poles
%   g.beta_star reference weight that cancels the slowest real pole
%   g.beta      the two choices by name: g.beta.star, g.beta.one
%   req is only read for the observer settings: req.obs_factor, req.obs_max
%   (defaults in pp_setup.m, changed in section 1 of design_pp.m). Nothing is set here.
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
% observer pole i = obs_factor(i) * (fastest controller pole), but never faster than
% obs_max(i). Separation principle: the closed-loop poles are the 3 controller poles
% plus these 2, so they do not change p; they change only how fast the estimation error
% dies, how much encoder noise reaches u, and how much a wrong model biases the estimate.
p_fast  = max(abs(p));
o       = min(req.obs_factor(:).*p_fast, req.obs_max(:));   % [rad/s], 2 x 1
g.p_obs = -o;
% eig(A - L*C) = observer poles. acker works on (A', C') because eig(A - L*C) = eig(A' - C'*L')
g.L = acker(m.A', m.C', g.p_obs)';
end
