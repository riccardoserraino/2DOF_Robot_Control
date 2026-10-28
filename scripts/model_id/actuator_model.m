%% ACTUATOR MODEL - DC motor + gearbox (SRV02), from datasheet values
% Compares the full model (with inductance Lm) with the simplified one
% (Lm neglected) to justify the 2nd-order model theta/V = K/(s(tau*s+1)).
clear; clc; close all;
run('../parameters.m');

eta  = nm*ng;                % Motor * gearbox efficiency
Jtot = Jm_rotor + Jeq;       % Total inertia (Jm_rotor is negligible w.r.t. Jeq)
s    = tf('s');

%% 1. Full electromechanical model
% Electrical : I/V       = 1/(Lm*s + Rm)
% Torque     : T         = eta*Kg*kt*I
% Mechanical : omega/T   = 1/(Jtot*s + beta_eq)
% Back-EMF   : V_emf     = Kg*km*omega   (negative feedback on the voltage)
G_omega_full = feedback(eta*Kg*kt/((Lm*s + Rm)*(Jtot*s + beta_eq)), Kg*km);
G_theta_full = (180/pi) * G_omega_full/s;          % position [deg]

%% 2. Simplified model (Lm = 0 -> I = (V - V_emf)/Rm)
G_omega_simple = feedback(eta*Kg*kt/(Rm*(Jtot*s + beta_eq)), Kg*km);
G_theta_simple = (180/pi) * G_omega_simple/s;

%% 3. Characteristic poles [rad/s]
w_open   = beta_eq/Jtot;                                   % mechanics only
w_closed = (Rm*beta_eq + eta*Kg^2*kt*km)/(Rm*Jtot);        % with back-EMF = 1/tau
w_elec   = Rm/Lm;                                          % electrical

fprintf('--- CHARACTERISTIC POLES ---\n');
fprintf('Mechanical, no back-EMF : %9.2f rad/s = %7.2f Hz\n', w_open,   w_open/(2*pi));
fprintf('Mechanical + back-EMF   : %9.2f rad/s = %7.2f Hz  (tau = %.4f s)\n', ...
        w_closed, w_closed/(2*pi), 1/w_closed);
fprintf('Electrical              : %9.2f rad/s = %7.2f Hz\n', w_elec,   w_elec/(2*pi));

fprintf('\nPoles theta/V simplified: %s rad/s\n', mat2str(sort(abs(pole(G_theta_simple)))', 4));
fprintf('Poles theta/V full      : %s rad/s\n',   mat2str(sort(abs(pole(G_theta_full)))', 4));

%% 4. -3 dB bandwidth of omega/V (theta/V has an integrator -> infinite DC gain)
fprintf('\nBandwidth omega/V: simplified %.2f rad/s, full %.2f rad/s\n', ...
        bandwidth(G_omega_simple), bandwidth(G_omega_full));

%% 5. Bode theta/V: the two models overlap up to far beyond the control band
figure;
bode(G_theta_simple, 'b', G_theta_full, 'r--', {0.1, 1e5});
grid on;
legend('Simplified (L_m neglected)', 'Full (L_m included)', 'Location', 'southwest');
title('Bode: \theta/V_a');
