%% POLE PLACEMENT - state-space model + pole placement + observer (both motors)
% Units = the Simulink signals: angle in DEGREES with the encoder sign
% (same plant as pid_tune.m), so the numbers go straight into CNC.slx.
% Results (workspace variables, one set per motor):
%   motor model: A_3165, B_3165, C_3165, D_3165   /  A_3166, B_3166, C_3166, D_3166
%   controller:  Kx_3165, Ki_3165                  /  Kx_3166, Ki_3166
%   observer:    A_obs_3165, B_obs_3165            /  A_obs_3166, B_obs_3166
clear; clc; close all
run('parameters.m')

% Design choices (same for both motors)
wn    = 40;                       % natural frequency [rad/s] -> speed
zeta  = 0.8;                      % damping -> overshoot
p_int = -zeta*wn;                 % integrator pole [rad/s]

motors = {'3165', '3166'};
k_all   = [k_3165_load,   k_3166_load];
tau_all = [tau_3165_load, tau_3166_load];

for i = 1:2
    k = k_all(i);  tau = tau_all(i);

    %% 1. State-space model of G(s) = b/(s(tau s + 1)),  x = [theta; omega]
    b = -k*180/pi;                % [deg/(V s)], minus = encoder sign
    A = [0 1; 0 -1/tau];
    B = [0; b/tau];
    C = [1 0];
    D = 0;                        % no direct feedthrough from voltage to angle

    %% 2. Augmented state with integral action: z = [theta; omega; eI], eI_dot = r - theta
    Ai = [A, zeros(2,1); -C, 0];
    Bi = [B; 0];

    %% 3. Pole placement: 2nd-order dominant pair + integrator pole
    p = [-zeta*wn + 1j*wn*sqrt(1-zeta^2), -zeta*wn - 1j*wn*sqrt(1-zeta^2), p_int];
    K = place(Ai, Bi, p);         % u = -K z  ->  eig(Ai - Bi K) = p

    %% 4. Observer (Luenberger): xhat_dot = A xhat + B u + L (y - C xhat)
    p_obs = [-4*wn, -5*wn];       % 4-5 times faster than the controller
    L = place(A', C', p_obs)';    % duality: eig(A - L C) = p_obs

    %% 5. Save the values for Simulink
    % Observer as State-Space block: xhat_dot = (A - L C) xhat + [B L] [u; y]
    % Block input: Mux [u (after saturation); y (measured angle)] -> output [theta_hat; omega_hat]
    pp(i).motor = motors{i};
    pp(i).A     = A;              % motor model (before pole placement)
    pp(i).B     = B;
    pp(i).C     = C;
    pp(i).D     = D;
    pp(i).Kx    = K(1:2);
    pp(i).Ki    = K(3);
    pp(i).A_obs = A - L*C;
    pp(i).B_obs = [B L];

    fprintf('\n===== motor %s =====\n', motors{i})
    fprintf('rank ctrb augmented = %d (must be 3), rank obsv = %d (must be 2)\n', ...
            rank(ctrb(Ai,Bi)), rank(obsv(A,C)))
    fprintf('Motor model:        A = %s,  B = %s,  C = %s,  D = %g\n', ...
            mat2str(A,6), mat2str(B,6), mat2str(C), D)
    fprintf('Kx = %s   Ki = %.4g\n', mat2str(pp(i).Kx,5), pp(i).Ki)
    fprintf('State-Space block:  A = %s\n', mat2str(pp(i).A_obs,6))
    fprintf('                    B = %s\n', mat2str(pp(i).B_obs,6))
    fprintf('                    C = eye(2),  D = zeros(2,2),  initial condition = [0;0]\n')
end

%% 6. Variables with the motor in the name (used by the Simulink blocks)
A_3165  = pp(1).A;    B_3165  = pp(1).B;    C_3165     = pp(1).C;       D_3165     = pp(1).D;
A_3166  = pp(2).A;    B_3166  = pp(2).B;    C_3166     = pp(2).C;       D_3166     = pp(2).D;
Kx_3165 = pp(1).Kx;   Ki_3165 = pp(1).Ki;   A_obs_3165 = pp(1).A_obs;   B_obs_3165 = pp(1).B_obs;
Kx_3166 = pp(2).Kx;   Ki_3166 = pp(2).Ki;   A_obs_3166 = pp(2).A_obs;   B_obs_3166 = pp(2).B_obs;
