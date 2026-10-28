%% PP_DESIGN - choose the closed-loop poles of one motor and compare with the PID
% Steps: 1 model | 2 PID seen as pole placement | 3 choose the poles |
%        4 gains | 5 checks (step, disturbance, Bode) | 6 save for Simulink
% Units: degrees with the encoder sign (same as pid_tune.m and Simulink).
clear; clc; close all
run('../parameters.m')                % k_xxxx_load, tau_xxxx_load
run('pid.m')                  % contr_3165, contr_3166 (the PID used now)

motor = '3165';                    % '3165' or '3166'
ts    = 0.15;                      % wanted 2% settling time [s]
obs_f = 4;                         % observer poles = obs_f x fastest controller pole

if strcmp(motor,'3165'), k = k_3165_load; tau = tau_3165_load; Cpid = contr_3165;
else,                    k = k_3166_load; tau = tau_3166_load; Cpid = contr_3166; end

%% 1. Model: x = [theta; omega]
b  = -k*180/pi;                    % [deg/(V s)], minus = encoder sign
A  = [0 1; 0 -1/tau];  B = [0; b/tau];  C = [1 0];
G  = ss(A, B, C, 0);               % plant, voltage -> angle
Ai = [A zeros(2,1); -C 0];         % add the integrator state eI = int(r - theta)
Bi = [B; 0];
fprintf('plant poles: %s rad/s\n', mat2str(eig(A).', 3))
fprintf('closed loop: 3 controller poles (theta, omega, eI) + 2 observer poles\n')

%% 2. The PID seen as pole placement
% PID on e = r - theta:  u = Kp e + Ki int(e) + Kd de/dt
% with r = 0 this is u = -K z with K = [Kp, Kd, -Ki]  (derivative filter ignored)
K_pid = [Cpid.Kp, Cpid.Kd, -Cpid.Ki];
p_pid = eig(Ai - Bi*K_pid);
fprintf('PID gains: Kp %.4g  Ki %.4g  Kd %.4g\n', Cpid.Kp, Cpid.Ki, Cpid.Kd)
fprintf('PID closed-loop poles: %s rad/s\n', mat2str(p_pid.', 3))
Cpid_f = pid(Cpid.Kp, Cpid.Ki, Cpid.Kd, 1/100);   % with N = 100, as in Simulink

%% 3. Choose the poles
% 3 poles, no zeros (r enters only through the integrator) -> use a 3rd-order
% pattern and scale it to the wanted settling time.
% Bessel: no overshoot (0.8%) with the lowest voltage for a given ts.
% zeta-wn: the pattern used so far (pair zeta = 0.8 + integrator pole at -zeta*wn).
pn_bessel = [-1.0474+0.9993i; -1.0474-0.9993i; -1.3227];   % normalized Bessel (w0 = 1)
pn_zeta   = [-0.8+0.6i; -0.8-0.6i; -0.8];                  % normalized zeta = 0.8 pattern
p_bessel  = scale_poles(pn_bessel, ts);
p_zeta    = scale_poles(pn_zeta, ts);
p         = p_bessel;              % <- the poles used below (try p_zeta to compare)
fprintf('chosen poles: %s rad/s\n', mat2str(p.', 3))

%% 4. Gains: controller and observer
K     = place(Ai, Bi, p);  Kx = K(1:2);  Ki = K(3);         % u = -Kx x - Ki eI
p_obs = obs_f*max(abs(p))*[-1 -1.25];                     % two real, distinct poles
L     = place(A', C', p_obs)';                            % eig(A - L C) = p_obs
A_obs = A - L*C;  B_obs = [B L];                          % State-Space block of the observer
fprintf('Kx = %s   Ki = %.4g   observer poles %s\n', mat2str(Kx,4), Ki, mat2str(p_obs,4))

%% 5. Checks
Cy_pp = observer_ctrl(A, B, C, Kx, Ki, L);           % pole placement + observer, from y to u
K_z   = place(Ai, Bi, p_zeta);                          % same checks for the zeta-wn pattern
L_z   = place(A', C', obs_f*max(abs(p_zeta))*[-1 -1.25])';
Cy_z  = observer_ctrl(A, B, C, K_z(1:2), K_z(3), L_z);
t = linspace(0, 1, 2000)';

% (a) step of 10 deg through the integrator: shows the poles alone
cl  = ss(Ai - Bi*K, [0;0;1], [C 0; -K], 0);    % outputs theta and u
y   = step(10*cl, t);  s_pp = stepinfo(y(:,1), t, 10);
y_pid = step(10*feedback(Cpid_f*G, 1), t);  s_pid = stepinfo(y_pid, t, 10);
u_pid = step(10*feedback(Cpid_f, G), t);       % PID voltage (includes the derivative kick)

% (b) load disturbance: 0.5 V step at the motor input, peak angle error
d_pid = step(0.5*feedback(G, Cpid_f), t);     % y = G/(1 + C G) d
d_pp  = step(0.5*feedback(G, -Cy_pp), t);     % u = Cy y  ->  y = G/(1 - G Cy) d

% (c) Bode: loop broken at the motor input, crossover and margins
[gm_pid, pm_pid, ~, wc_pid] = margin(Cpid_f*G);
[gm_pp,  pm_pp,  ~, wc_pp ] = margin(-Cy_pp*G);
[~,      pm_z,   ~, wc_z  ] = margin(-Cy_z*G);

fprintf('\n              ts [s]  overshoot  max|u| 10deg  wc [rad/s]  PM [deg]  GM [dB]  peak err 0.5V\n')
fprintf('PID           %6.3f  %6.1f %%    %6.2f V   %8.1f  %8.1f  %7.1f  %8.2f deg\n', s_pid.SettlingTime, ...
        s_pid.Overshoot, max(abs(u_pid)), wc_pid, pm_pid, 20*log10(gm_pid), max(abs(d_pid)))
fprintf('pole place.   %6.3f  %6.1f %%    %6.2f V   %8.1f  %8.1f  %7.1f  %8.2f deg\n', s_pp.SettlingTime, ...
        s_pp.Overshoot, max(abs(y(:,2))), wc_pp, pm_pp, 20*log10(gm_pp), max(abs(d_pp)))
fprintf('(zeta-wn pattern at the same ts: wc %.1f rad/s, PM %.1f deg)\n', wc_z, pm_z)
fprintf('(PID step includes its zeros; pole-placement step shows the poles only)\n')

figure('Name', 'Step 10 deg'); plot(t, y_pid, t, y(:,1)); grid on
legend('PID', 'pole placement'); xlabel('t [s]'); ylabel('\theta [deg]')
figure('Name', 'Load disturbance 0.5 V'); plot(t, d_pid, t, d_pp); grid on
legend('PID', 'pole placement'); xlabel('t [s]'); ylabel('angle error [deg]')
figure('Name', 'Loop at the motor input'); bode(-Cy_pp*G, Cpid_f*G); grid on
legend('pole placement + observer', 'PID')
figure('Name', 'Poles'); plot(real(p_pid), imag(p_pid), 'kx', real(p), imag(p), 'bo', ...
     real(p_obs), imag(p_obs), 'rs', real(eig(A)), imag(eig(A)), 'g+', 'MarkerSize', 9, 'LineWidth', 1.5)
grid on; xlabel('Re'); ylabel('Im'); legend('PID', 'pole placement', 'observer', 'plant')

%% 6. Save for Simulink (variables with the motor in the name)
if strcmp(motor,'3165'), Kx_3165 = Kx; Ki_3165 = Ki; A_obs_3165 = A_obs; B_obs_3165 = B_obs;
else,                    Kx_3166 = Kx; Ki_3166 = Ki; A_obs_3166 = A_obs; B_obs_3166 = B_obs; end

%% ------------------------------------------------------------------------
function p = scale_poles(pn, ts)
% Scale a normalized pole pattern so the step settles (2%) in ts seconds.
% Multiplying all poles by w0 divides all times by w0.
Tn = zpk([], pn, real(prod(-pn)));                 % zero-free loop, unit DC gain
s  = stepinfo(Tn, 'SettlingTimeThreshold', 0.02);
p  = pn*s.SettlingTime/ts;
end

function Cy = observer_ctrl(A, B, C, Kx, Ki, L)
% Controller from y to u with r = 0: observer + integrator + state feedback
%   xhat' = A xhat + B u + L (y - C xhat),  eI' = -y,  u = -Kx xhat - Ki eI
Ac = [A - B*Kx - L*C, -B*Ki; zeros(1,2), 0];
Bc = [L; -1];
Cc = [-Kx, -Ki];
Cy = ss(Ac, Bc, Cc, 0);
end