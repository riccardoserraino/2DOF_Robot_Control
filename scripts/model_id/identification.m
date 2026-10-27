%% IDENTIFICATION - grey-box model theta/V = K/(s*(tau*s + 1)) from multisine data
% Estimates K and tau on one experiment and validates them on another one
% (same harmonics, different random phases) in time and frequency domain.
% Requires data/multisine_design.mat (run multisine_design.m once).
clear; clc; close all;
run('../parameters.m');      % datasheet values (Rm, kt, km, Kg, eta, Jeq, beta_eq)
eta = nm*ng;                 % Motor * gearbox efficiency

%% 1. Select the experiment
motor    = '3166';           % '3165' or '3166'
load_tag = '_load';          % '' = motor alone, '_load' = five-bar attached
data_dir = '../../data/';

est = load([data_dir 'data_' motor '_seed30' load_tag '.mat']);   % estimation
val = load([data_dir 'data_' motor '_seed29' load_tag '.mat']);   % validation

z_est = make_iddata(est.data);
z_val = make_iddata(val.data);

%% 2. Estimation: Kp/(s*(1 + Tp1*s)), the 'P1I' structure imposes the integrator
opt  = procestOptions('InitialCondition', 'zero');   % motor at rest at t = 0
sysP = procest(z_est, 'P1I', opt);                   % Kp [deg/(V s)], Tp1 [s]

K_rad = sysP.Kp*pi/180;      % [rad/(V s)]  -> value to copy in parameters.m (k_*)
tau   = sysP.Tp1;            % [s]          -> value to copy in parameters.m (tau_*)

% Equivalent mechanical parameters (Rm, kt, km, Kg, eta fixed from datasheet)
c0       = eta*Kg*kt/K_rad;              % = Rm*beta + eta*kt*km*Kg^2
beta_est = (c0 - eta*kt*km*Kg^2)/Rm;
J_est    = tau*c0/Rm;

%% 3. Datasheet model, for comparison
s     = tf('s');
c0_d  = Rm*beta_eq + eta*kt*km*Kg^2;
K_d   = eta*Kg*kt/c0_d;                  % [rad/(V s)]
tau_d = Jeq*Rm/c0_d;                     % [s]
G_ds  = (180/pi) * K_d/(s*(tau_d*s + 1));
G_id  = (180/pi) * K_rad/(s*(tau*s + 1));

fprintf('\n--- IDENTIFIED MODEL (motor %s%s) ---\n', motor, load_tag);
fprintf('K   = %.3f rad/(V s)   (datasheet %.3f)\n', K_rad, K_d);
fprintf('tau = %.1f ms -> mechanical pole %.1f rad/s (datasheet %.1f rad/s)\n', ...
        tau*1e3, 1/tau, 1/tau_d);
fprintf('Estimated J_eq = %.3e kg m^2   (datasheet %.3e)\n', J_est, Jeq);
fprintf('Estimated beta = %.4f N m s/rad (datasheet %.4f)\n', beta_est, beta_eq);

%% 4. Time-domain validation on the other experiment
figure;
compare(z_val, sysP, G_ds);
legend('measurement', 'identified', 'datasheet', 'Location', 'best');

%% 5. Frequency-domain validation: measured FRF at the excited frequencies
ms = load([data_dir 'multisine_design.mat']);        % Tp, k, P_skip
[w_exc, G_meas] = frf_fft(z_est, ms.Tp, ms.k, ms.P_skip);

[mag_id, ph_id] = bode(G_id, w_exc);
fprintf('\n  w [rad/s] | |G| meas [dB] | |G| model [dB] | phase meas | phase model\n');
for i = 1:numel(w_exc)
    fprintf('  %8.2f | %11.2f | %11.2f | %8.1f | %8.1f\n', w_exc(i), ...
        20*log10(abs(G_meas(i))), 20*log10(mag_id(i)), ...
        angle(G_meas(i))*180/pi, ph_id(i));
end

figure;
bode(G_id, 'b', G_ds, 'r--', {1, 1e3}); hold on;
h = findobj(gcf, 'type', 'axes');                    % h(2) = magnitude, h(1) = phase
semilogx(h(2), w_exc, 20*log10(abs(G_meas)), 'ko');
semilogx(h(1), w_exc, angle(G_meas)*180/pi, 'ko');
legend('identified', 'datasheet', 'measurement');

%% ---------------- Local functions ----------------
function z = make_iddata(data)
    % Rows of data: [t; V; theta_deg]
    if size(data,1) == 3 && size(data,2) > 3, data = data.'; end
    t = data(:,1);  u = data(:,2);  y = data(:,3);
    y = -y;   % Encoder sign convention: V > 0 gives a negative angle
    z = iddata(y, u, mean(diff(t)), 'InputName', 'V', 'OutputName', 'theta', ...
               'InputUnit', 'V', 'OutputUnit', 'deg');
end

function [w, G] = frf_fft(z, Tp, k, P_skip)
    % FRF of a periodic multisine: each excited frequency falls exactly on an
    % FFT bin (no leakage); averaging over the periods reduces the noise.
    Np = round(Tp/z.Ts);                         % Samples per period
    P  = floor(size(z,1)/Np);                    % Recorded periods
    u  = z.u(P_skip*Np+1 : P*Np);                % Discard the transient periods
    y  = z.y(P_skip*Np+1 : P*Np);
    t  = (0:numel(y)-1)'*z.Ts;
    y  = y - polyval(polyfit(t, y, 1), t);       % Remove the integrator drift
    U  = mean(reshape(u, Np, []), 2);            % Average over periods
    Y  = mean(reshape(y, Np, []), 2);
    Uf = fft(U);  Yf = fft(Y);
    G  = Yf(k+1) ./ Uf(k+1);                     % Bin k <-> w = 2*pi*k/Tp
    G  = G(:).';
    w  = 2*pi*k/Tp;
end
