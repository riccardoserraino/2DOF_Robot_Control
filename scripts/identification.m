clear; clc; close all 
 
%% KNOWN PARAMETERS (SRV02 datasheet, high gear) 
% Rm, Lm, kt, km, Kg, nm, ng, Jeq, beta_eq 
run('parameters.m') 
eta = nm*ng;                       % motor * gearbox efficiency 
 
%% DATA: one experiment for estimation, a DIFFERENT one for validation 
est = load('../data/data_3166_seed30_load.mat');      % multisine seed A 
val = load('../data/data_3166_seed29_load.mat');      % multisine with different frequencies/phases 
plot(est.data(1,:), est.data(3,:)); grid on; 
 
z_est = make_iddata(est.data); 
z_val = make_iddata(val.data); 

%% ESTIMATION: Kp/(s(1+Tp1*s)) -- integrator imposed by the 'P1I' structure 
opt  = procestOptions('InitialCondition','zero');   % motor at rest at t=0 
sysP = procest(z_est, 'P1I', opt);                  % Kp in deg/(V s), Tp1 in s 
 
K_rad = sysP.Kp*pi/180;            % [rad/(V s)] 
tau   = sysP.Tp1;                  % [s] 
 
% Equivalent mechanical parameters (Rm, kt, km, Kg, eta fixed from datasheet) 
c0       = eta*Kg*kt/K_rad;        % = Rm*beta + eta*kt*km*Kg^2 
beta_est = (c0 - eta*kt*km*Kg^2)/Rm; 
J_est    = tau*c0/Rm; 
 
%% DATASHEET MODEL
s    = tf('s'); 
c0_d = Rm*beta_eq + eta*kt*km*Kg^2; 
K_d  = eta*Kg*kt/c0_d;             % [rad/(V s)] 
tau_d = Jeq*Rm/c0_d;               % [s] 
G_ds = (180/pi) * K_d / (s*(tau_d*s + 1)); 
 
% Identified model as a transfer function
G_id = (180/pi) * K_rad / (s*(tau*s + 1)); 
 
%% RESULTS 
fprintf('\n--- IDENTIFIED MODEL (P1I) ---\n') 
fprintf('K   = %.3f rad/(V s)   (datasheet %.3f)\n', K_rad, K_d) 
fprintf('tau = %.1f ms -> mechanical pole %.1f rad/s (datasheet %.1f rad/s)\n', ... 
        tau*1e3, 1/tau, 1/tau_d) 
fprintf('Estimated J_eq = %.3e kg m^2   (datasheet %.3e)\n', J_est, Jeq) 
fprintf('Estimated beta = %.4f N m s/rad (datasheet %.4f)\n', beta_est, beta_eq) 
 
%% TIME-DOMAIN VALIDATION on a different experiment 
figure 
compare(z_val, sysP, G_ds) 
legend('measurement','identified','datasheet','Location','best') 
 
%% FREQUENCY-DOMAIN VALIDATION: experimental FRF at excited frequencies 
% NEW experiments (periodic multisine from multisine_design.m): exact FFT. 
% OLD experiments (integer frequencies in rad/s, non-periodic): projection. 
periodic = true; 
 
if periodic 
    ms = load('../data/multisine_design.mat');           % Tp, k, P_skip 
    [w_exc, G_meas] = frf_fft(z_est, ms.Tp, ms.k, ms.P_skip); 
else 
    w_exc  = [5 8 12 17 20 23 27 32 35 40];      % [rad/s] 
    G_meas = frf_at(z_est, w_exc, 5);            % discard the first 5 s 
end 
 
[mag_id, ph_id] = bode(G_id, w_exc); 
fprintf('\n  w [rad/s] | |G| meas [dB] | |G| model [dB] | phase meas | phase model\n') 
for i = 1:numel(w_exc) 
    fprintf('  %8.2f | %11.2f | %11.2f | %8.1f | %8.1f\n', w_exc(i), ... 
        20*log10(abs(G_meas(i))), 20*log10(mag_id(i)), ... 
        angle(G_meas(i))*180/pi, ph_id(i)) 
end 
 
figure 
bode(G_id, 'b', G_ds, 'r--', {1, 1e3}); hold on 
h = findobj(gcf,'type','axes');                 % h(2) = magnitude, h(1) = phase 
semilogx(h(2), w_exc, 20*log10(abs(G_meas)), 'ko') 
semilogx(h(1), w_exc, angle(G_meas)*180/pi, 'ko') 
legend('identified','datasheet','measurement') 
 
%% --------------------------------------------------------------------- 
function z = make_iddata(data) 
    if size(data,1) == 3 && size(data,2) > 3, data = data.'; end 
    t = data(:,1); u = data(:,2); y = data(:,3); 
    y = -y;   % SIGN CONVENTION: for V>0 the encoder measures a negative angle. 
              % It is better to correct this once and for all in the encoder gain in Simulink. 
    z = iddata(y, u, mean(diff(t)), 'InputName','V', 'OutputName','theta', ... 
               'InputUnit','V', 'OutputUnit','deg'); 
end 
 
function G = frf_at(z, w, t_skip) 
    % Fourier projection at frequencies not on the FFT grid, with linear detrending 
    t = (0:size(z,1)-1)'*z.Ts; 
    k = t >= t_skip; 
    t = t(k);  u = z.u(k);  y = z.y(k); 
    y = y - polyval(polyfit(t, y, 1), t);       % remove integrator drift 
    G = zeros(size(w)); 
    for i = 1:numel(w) 
        e = exp(-1j*w(i)*t); 
        G(i) = sum(y.*e)/sum(u.*e); 
    end 
end 
 
function [w, G] = frf_fft(z, Tp, k, P_skip) 
    % FRF using FFT on a periodic multisine: frequencies exactly on FFT bins, 
    % averaged over steady-state periods to reduce noise. 
    Np = round(Tp/z.Ts);                         % samples per period 
    P  = floor(size(z,1)/Np);                    % recorded periods 
    u  = z.u(P_skip*Np+1 : P*Np); 
    y  = z.y(P_skip*Np+1 : P*Np); 
    t  = (0:numel(y)-1)'*z.Ts; 
    y  = y - polyval(polyfit(t, y, 1), t);       % residual drift (friction/offset) 
    U  = mean(reshape(u, Np, []), 2);            % average over periods 
    Y  = mean(reshape(y, Np, []), 2); 
    Uf = fft(U);  Yf = fft(Y); 
    G  = Yf(k+1) ./ Uf(k+1);                     % bin k <-> w = 2*pi*k/Tp 
    G  = G(:).'; 
    w  = 2*pi*k/Tp; 
end

