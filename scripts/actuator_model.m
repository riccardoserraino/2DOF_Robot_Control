clear; clc; close all

%% PARAMETERS
% Electrical motor + gearbox parameters
run('parameters.m')

eta = nm * ng;                  % Motor + gearbox efficiency
Jtot = Jm_rotor + Jeq;          % Total equivalent inertia
conv = 360/(2*pi);              % rad -> deg

%% TRANSFER FUNCTIONS

s = tf('s');

% ---------------------------------------------------------
% FULL ELECTROMECHANICAL MODEL
% ---------------------------------------------------------
% Electrical:
% I(s)/V(s) = 1/(Lm*s + Rm)
%
% Mechanical:
% omega(s)/T(s) = 1/(Jtot*s + beta_eq)
%
% Back-EMF:
% V_emf = km * Kg * omega
%
% Motor torque:
% T = kt * Kg * I

G_elec = 1/(Lm*s + Rm);

G_mech = 1/(Jtot*s + beta_eq);

% Open-loop electromechanical chain
G_omega_ol = Kg*kt*eta * G_elec * G_mech;

% Closed loop including back-EMF
G_omega_full = feedback(G_omega_ol, Kg*km);

% Position output
G_theta_full = conv * G_omega_full/s;


% ---------------------------------------------------------
% SIMPLIFIED MODEL - NEGLECT MOTOR INDUCTANCE
% ---------------------------------------------------------

% Electrical current approximation:
% I = V/Rm

G_simple = Kg*kt*eta/(Rm*(Jtot*s + beta_eq));

% Include back-EMF feedback
G_omega_simple = feedback(G_simple, Kg*km);

% Position
G_theta_simple = conv * G_omega_simple/s;


%% CALCULATIONS

% ---------------------------------------------------------
% SIMPLIFIED MODEL
% ---------------------------------------------------------

% Denominator:
%
% Rm*Jtot*s + Rm*beta_eq + Kg^2*kt*km

c1 = Rm * Jtot;
c0 = Rm * beta_eq + Kg^2 * kt * km;


% Mechanical pole without back-EMF
w_open = beta_eq/Jtot;


% Closed-loop pole
w_closed = c0/c1;


% Electrical pole
w_elec = Rm/Lm;


% Convert to Hz
f_open   = w_open/(2*pi);
f_closed = w_closed/(2*pi);
f_elec   = w_elec/(2*pi);


% Time constants
tau_open   = 1/w_open;
tau_closed = 1/w_closed;
tau_elec   = 1/w_elec;


%% POLES FROM TRANSFER FUNCTIONS

p_theta_simple = pole(G_theta_simple);
p_theta_full   = pole(G_theta_full);

% Sort from slowest to fastest
[~, i1] = sort(abs(p_theta_simple));
p_theta_simple = p_theta_simple(i1);

[~, i2] = sort(abs(p_theta_full));
p_theta_full = p_theta_full(i2);

f_theta_simple = abs(p_theta_simple)/(2*pi);
f_theta_full   = abs(p_theta_full)/(2*pi);


%% PRINT RESULTS

fprintf('========================================\n')
fprintf(' ELECTRICAL MOTOR MODEL\n')
fprintf('========================================\n')

% fprintf('\n--- MOTOR PARAMETERS ---\n')
% fprintf('Rm          = %.4f Ohm\n', Rm)
% fprintf('Lm          = %.6f H\n', Lm)
% fprintf('kt          = %.6f N*m/A\n', kt)
% fprintf('km          = %.6f V/(rad/s)\n', km)
% fprintf('Kg          = %.2f\n', Kg)
% fprintf('Jm_rotor    = %.3e kg*m^2\n', Jm_rotor)
% fprintf('Jeq         = %.3e kg*m^2\n', Jeq)
% fprintf('Jtot        = %.3e kg*m^2\n', Jtot)
% fprintf('beta_eq     = %.5f N*m*s/rad\n', beta_eq)


fprintf('\n--- MECHANICAL ---\n')

fprintf('Open-loop pole  : %8.2f rad/s = %6.2f Hz | tau = %.4f s\n', ...
    w_open, f_open, tau_open)

fprintf('Closed-loop pole: %8.2f rad/s = %6.2f Hz | tau = %.4f s\n', ...
    w_closed, f_closed, tau_closed)


fprintf('\n--- ELECTRICAL ---\n')

fprintf('Electrical pole : %8.2f rad/s = %6.2f Hz | tau = %.6f s\n', ...
    w_elec, f_elec, tau_elec)


fprintf('\n--- SIMPLIFIED MODEL: theta/Va POLES ---\n')

for k = 1:numel(p_theta_simple)
    fprintf('p%d = %10.2f rad/s = %8.2f Hz\n', ...
        k, abs(p_theta_simple(k)), f_theta_simple(k))
end


fprintf('\n--- FULL MODEL: theta/Va POLES ---\n')

for k = 1:numel(p_theta_full)
    fprintf('p%d = %10.2f rad/s = %8.2f Hz\n', ...
        k, abs(p_theta_full(k)), f_theta_full(k))
end


%% BODE: theta/Va

figure

bode(G_theta_simple, 'b', ...
      G_theta_full, 'r--', ...
      {0.1, 1e5})

grid on

legend('Simplified (Lm neglected)', ...
       'Full (Lm included)', ...
       'Location', 'southwest')

title('Bode: \theta/V_a')


%% CUTOFF FREQUENCIES

% -3 dB bandwidth of omega/Va
% theta/Va contains an integrator, therefore its DC gain is infinite.

wc_simple = bandwidth(G_omega_simple);
wc_full   = bandwidth(G_omega_full);

fc_simple = wc_simple/(2*pi);
fc_full   = wc_full/(2*pi);


fprintf('\n--- CUTOFF (-3 dB) of omega/Va ---\n')

fprintf('Simplified: %8.2f rad/s = %6.2f Hz\n', ...
    wc_simple, fc_simple)

fprintf('Full      : %8.2f rad/s = %6.2f Hz\n', ...
    wc_full, fc_full)