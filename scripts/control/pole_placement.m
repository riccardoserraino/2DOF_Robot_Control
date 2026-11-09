%% POLE PLACEMENT + OBSERVER + FEEDFORWARD + ANTI-WINDUP (both motors)
% Motor model (same as the Simulink subsystem):
%   V -> k/(tau s + 1) -> 1/s -> Kout -> measured angle [deg, encoder sign]
%   => G(s) = b / ( s (tau s + 1) ),   b = k * Kout
% Control law implemented in the block diagram (motor 3165):
%   u = Kffv*dphi + Kffa*ddphi - Kx*(xhat - [phi; dphi]) - Ki*eI
%   eI_dot = (phi - y) + Kaw*(u_sat - u)        <- anti-windup
% Work in progress: the pole choice (wn, zeta) is still under study.

%% 0. Load the workspace
% pid.m clears the workspace, loads parameters.m and computes the PID gains
% (CNC_pp_obs.slx also contains the PID branch, so both sets are needed).
run('pid.m');

%% Design choices (same for both motors)
wn   = 40;            % [rad/s] natural frequency -> speed of response
zeta = 0.9;           % damping
% Vmax comes from parameters.m and must match the Saturation block (+-6 V)
Kout = [-180/pi, -180/pi];   % gain of the "-K-" block after the integrator in
                             % each motor subsystem: must be identical to it

motors  = {'3165', '3166'};
k_all   = [k_3165_load,   k_3166_load];
tau_all = [tau_3165_load, tau_3166_load];

for i = 1:2
    k = k_all(i);  tau = tau_all(i);

    %% 1. State-space model, x = [theta; omega]  (theta in deg)
    b = k*Kout(i);                % total gain [deg/(V s)], sign included
    A = [0 1; 0 -1/tau];
    B = [0; b/tau];
    C = [1 0];
    D = 0;

    %% 2. Augmented state with the integral of the error: eI_dot = r - theta
    Ai = [A, zeros(2,1); -C, 0];
    Bi = [B; 0];

    %% 3. Pole placement: dominant pair + integrator pole
    wd = wn*sqrt(1 - zeta^2);
    p  = [-zeta*wn + 1j*wd, -zeta*wn - 1j*wd, -zeta*wn];
    K  = place(Ai, Bi, p);        % u = -K z  ->  eig(Ai - Bi*K) = p

    %% 4. Luenberger observer (duality)
    L = place(A', C', [-4*wn, -5*wn])';   % eig(A - L*C) 4-5 times faster

    %% 5. Feedforward from the inverse model: tau*theta_dd + theta_d = b*u
    Kffv = 1/b;                   % gain on dphi
    Kffa = tau/b;                 % gain on ddphi

    %% 6. Anti-windup (back-calculation)
    Tt  = 1/(zeta*wn);            % time constant to "unload" the integrator
    Kaw = -1/(K(3)*Tt);           % eI_dot = e + Kaw*(u_sat - u)

    %% Save
    pp(i).A = A;  pp(i).B = B;  pp(i).C = C;  pp(i).D = D;
    pp(i).Kx = K(1:2);  pp(i).Ki = K(3);
    pp(i).A_obs = A - L*C;  pp(i).B_obs = [B L];   % input [u_sat; y]
    pp(i).Kffv = Kffv;  pp(i).Kffa = Kffa;  pp(i).Kaw = Kaw;

    fprintf('\n===== motor %s =====\n', motors{i})
    fprintf('rank ctrb augmented = %d (must be 3), rank obsv = %d (must be 2)\n', ...
            rank(ctrb(Ai,Bi)), rank(obsv(A,C)))
    fprintf('b = %.4g deg/(V s)\n', b)
    fprintf('Kx = %s   Ki = %.4g\n', mat2str(pp(i).Kx,5), pp(i).Ki)
    fprintf('Kffv = %.4g   Kffa = %.4g   Kaw = %.4g\n', Kffv, Kffa, Kaw)
    % Voltage requested by a 1 deg error (proportional part only)
    fprintf('|Kx(1)| = %.3g V/deg -> error that saturates: %.2f deg\n', ...
            abs(K(1)), Vmax/abs(K(1)))
end

% Remove loop temporaries so they do not clash with other scripts/models
clear k tau b A B C D Ai Bi K L p wd Kffv Kffa Tt Kaw i

%% Variables for Simulink
A_3165 = pp(1).A;  B_3165 = pp(1).B;  C_3165 = pp(1).C;  D_3165 = pp(1).D;
A_3166 = pp(2).A;  B_3166 = pp(2).B;  C_3166 = pp(2).C;  D_3166 = pp(2).D;

Kx_3165 = pp(1).Kx;  Ki_3165 = pp(1).Ki;  A_obs_3165 = pp(1).A_obs;  B_obs_3165 = pp(1).B_obs;
Kx_3166 = pp(2).Kx;  Ki_3166 = pp(2).Ki;  A_obs_3166 = pp(2).A_obs;  B_obs_3166 = pp(2).B_obs;

Kffv_3165 = pp(1).Kffv;  Kffa_3165 = pp(1).Kffa;  Kaw_3165 = pp(1).Kaw;
Kffv_3166 = pp(2).Kffv;  Kffa_3166 = pp(2).Kffa;  Kaw_3166 = pp(2).Kaw;