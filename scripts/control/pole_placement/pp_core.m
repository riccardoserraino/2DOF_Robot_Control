function [res, ctrl] = pp_core(k_m, tau_m, des, cfg)
% PP_CORE  Pole placement + Luenberger observer of both joints: gains, requirement values
%          and Simulink variables. Used by pole_place.m and pp_tune.m, so the design you
%          tune and the design you run are computed by the same code.
%
% INPUTS
%   k_m, tau_m  [k_3165 k_3166], [tau_3165 tau_3166]   controller model [rad/(V s)], [s]
%   des         .wn [rad/s], .zeta [-], .lambda [rad/s] (= ifac*zeta*wn), .obs_factor (2x1) [-]
%   cfg         .Ts, .obs_max (2x1), .Td, .beta, .R, .q, .w_noise   (see pole_place.m)
% OUTPUTS
%   res   5 x 2 requirement values, one column per motor:
%         [peak voltage on a step of R deg; phase margin; Ms; chatter; slowest pole]
%         (the callers add the crossover wc as 6th row)
%   ctrl  ctrl(i).Cy (y -> u), .Cr (r -> u), .A, .B, .p_obs  for motor i (loop checks)
% It also writes in the base workspace every controller variable read by Model_pp_obs.slx
% and CNC_pp_obs.slx.
%
% THEORY
%   model (encoder degrees): theta' = omega, omega' = -omega/tau + (b/tau)*u, b = -k*180/pi
%   integral state eI' = r - theta. Gains: eig(Ai - Bi*[Kx Ki]) = controller poles (Ackermann).
%   Observer: eig(A - L*C) = observer poles (Ackermann on the dual system A', C').
%   Separation principle: closed-loop poles = controller poles + observer poles.

% controller poles (zeta > 1: the square root is imaginary and the pair becomes two real poles)
p = [-des.zeta*des.wn + 1i*des.wn*sqrt(1 - des.zeta^2);
     -des.zeta*des.wn - 1i*des.wn*sqrt(1 - des.zeta^2);
     -des.lambda];
mots = {'3165', '3166'};
res  = zeros(5, 2);

for i = 1:2
    k = k_m(i);  tau = tau_m(i);
    b = -k*180/pi;                                   % output in encoder degrees (sign!)
    A = [0, 1; 0, -1/tau];   B = [0; b/tau];   C = [1, 0];

    % controller and observer gains
    Ai = [A, zeros(2, 1); -C, 0];   Bi = [B; 0];
    K  = acker(Ai, Bi, p);
    Kx = K(1:2);   Ki = K(3);
    p_obs = -min(des.obs_factor(:)*max(abs(p)), cfg.obs_max(:));
    L  = acker(A', C', p_obs)';

    % controller as one block: states [xhat; eI], inputs r and y, output u
    Ac = [A - B*Kx - L*C, -B*Ki;  zeros(1, 3)];
    Cr = ss(Ac, [B*cfg.beta*Kx(1); 1], [-Kx, -Ki], cfg.beta*Kx(1));   % r -> u
    Cy = ss(Ac, [L; -1],               [-Kx, -Ki], 0);                 % y -> u

    % requirements: loop opened at the motor input, with the delay Td
    G  = ss(A, B, C, 0);   G.InputDelay = cfg.Td;
    Lp = -Cy*G;
    [~, PM] = margin(Lp);                                    % phase margin [deg]
    w  = logspace(-1, log10(pi/cfg.Ts), 500);
    Ms = max(abs(1./(1 + squeeze(freqresp(Lp, w)))));       % peak of |1/(1+L)|
    wh = logspace(log10(cfg.w_noise), log10(pi/cfg.Ts), 50);
    chat = max(abs(squeeze(freqresp(Cy, wh))))*cfg.q;      % [V] per encoder count
    Tu  = Cr*feedback(1, Cy*G, +1);                         % r -> u:  Cr/(1 - Cy*G)
    upk = cfg.R*max(abs(step(Tu, 1)));                      % [V] peak on a step of R deg
    res(:, i) = [upk; PM; Ms; chat; min(abs(p))];
    ctrl(i) = struct('Cy', Cy, 'Cr', Cr, 'A', A, 'B', B, 'p_obs', p_obs); %#ok<AGROW>

    % Simulink variables (names end in _3165 / _3166)
    n = mots{i};
    assignin('base', ['Kx_'    n], Kx);
    assignin('base', ['Ki_'    n], Ki);
    assignin('base', ['Kaw_'   n], -mean(abs(p))/Ki);       % anti-windup speed = mean|p|
    assignin('base', ['A_obs_' n], A - L*C);
    assignin('base', ['B_obs_' n], [B, L]);
    assignin('base', ['beta_'  n], cfg.beta);
    a = real(poly(p));
    assignin('base', ['lamf_'  n], a(4)/a(3));              % optional reference prefilter
    assignin('base', ['k_'   n '_load'], k);                % feedforward = controller model
    assignin('base', ['tau_' n '_load'], tau);
end
end
