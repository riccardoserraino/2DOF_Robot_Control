%% SS MODEL - state-space model of one motor + Kalman filter
% Output: ../data/ss_model_<motor>.mat  (used by ss_control.m, ss_tuning.m and Simulink)
clear; clc; close all
run('parameters.m')

motor = '3165';                   % TODO: '3165' or '3166'

%% 0. Parameters
K   = k_3165_load;                % k of the selected motor [rad/(V s)]
tau = tau_3165_load;              % tau of the selected motor [s]
b   = -K;                         % encoder sign: angle decreases for V > 0

Ts    = 0.002;                    % QUARC sample time [s]
Delta = 2*pi/4096;                % encoder resolution [rad]
datafile = '../data/data_3165_seed29_load.mat';   % TODO: validation experiment of the motor

fprintf('==================== SS MODEL - motor %s ====================\n', motor)

%% 1. Model from G(s), states = [theta; dtheta/dt]
% Theory: realization of G(s) + change of coordinates z = T x
% (observability canonical form: z1 = y, z2 = y_dot, valid since C*B = 0)
G = minreal(tf(b, [tau 1 0]));
[A, B, C, D] = ssdata(ss(G));

T = obsv(A, C);                   % T = [C; C*A]
A = T*A/T;  B = T*B;  C = C/T;    % now C = [1 0]
n = size(A,1);

%% 2. Model checks
% Theory: Kalman rank tests
assert(abs(D) < 1e-9, 'D must be zero');
fprintf('\n--- 1-2. Model ---\n')
fprintf('A = [%g %g; %g %g]   (expected [0 1; 0 %.2f])\n', A', -1/tau)
fprintf('B = [%g; %g]          (expected [0; %.2f])\n', B, b/tau)
fprintf('C = [%g %g]\n', C)
fprintf('rank ctrb = %d, rank obsv = %d  (must be %d)\n', rank(ctrb(A,B)), rank(obsv(A,C)), n)

figure('Name', '1. Model check')
bode(G, 'b', ss(A,B,C,0), 'r--'); grid on
legend('G(s) identified', 'state space'); title(['Model check - motor ' motor])

%% 3. Augmented state with input disturbance: x_a = [theta; omega; d]
% Theory: disturbance model d_dot = 0 (step-like input disturbance).
% d is observable (estimated by the filter) but NOT controllable: it is only
% compensated in the control law (u = ... - d_hat), not used in the K design.
Aa = [A, B; zeros(1,n), 0];
Ba = [B; 0];
Ca = [C, 0];
fprintf('\n--- 3. Augmented model ---\n')
fprintf('rank obsv augmented = %d  (must be %d)\n', rank(obsv(Aa,Ca)), n+1)

%% 4. Kalman filter (stationary Kalman-Bucy filter)
% Theory: Luenberger structure x_hat_dot = A x_hat + B u + L (y - C x_hat);
% L is the minimum error-covariance gain: L = P C' Rn^-1, with P from the
% filter Riccati eq.  A P + P A' - P C' Rn^-1 C P + Qn = 0.
% Continuous filter -> Qn, Rn are spectral densities (variance per sample * Ts).
Rn = Delta^2/12*Ts;               % quantization variance (uniform) as density
Qn = diag([0, 2.5e-2, 0.9]);      % TODO: tune [theta, omega, d] (see ss_tuning.m)

L = lqe(Aa, eye(n+1), Ca, Qn, Rn);
p_obs = eig(Aa - L*Ca);           % error dynamics e_dot = (Aa - L Ca) e

obs = ss(Aa - L*Ca, [Ba, L], eye(n+1), 0);   % inputs [u; y], outputs [theta; omega; d]
fprintf('\n--- 4. Kalman filter ---\n')
fprintf('L = [%.4g; %.4g; %.4g]\n', L)
fprintf('observer poles |p| = %s rad/s\n', mat2str(abs(p_obs).', 4))

%% 5. Offline test on recorded data (validation experiment)
data  = load(datafile);
t     = data.data(1,:).';
u     = data.data(2,:).';             % applied voltage [V]
theta = data.data(3,:).'*pi/180;      % measured angle [rad], encoder sign (same as b)

xhat = lsim(obs, [u, theta], t);
res  = theta - xhat(:,1);
fprintf('\n--- 5. Offline test (%s) ---\n', datafile)
fprintf('residual theta - theta_hat: RMS = %.2f encoder counts\n', rms(res)/Delta)
fprintf('d_hat: RMS = %.3f V  (RMS of u = %.3f V)\n', rms(xhat(:,3)), rms(u))

% Each estimate is overlaid with its reference (only theta is measured)
figure('Name', '2. Kalman filter - offline test')
ax1 = subplot(3,1,1); plot(t, theta, t, xhat(:,1), '--'); grid on
ylabel('\theta [rad]'); legend('measured', 'estimated'); title(['Kalman estimates - motor ' motor])
ax2 = subplot(3,1,2); plot(t, gradient(theta, t), t, xhat(:,2), '--'); grid on
ylabel('\omega [rad/s]'); legend('derivative of measured \theta', 'estimated')
ax3 = subplot(3,1,3); plot(t, u, t, xhat(:,3), '--'); grid on
ylabel('[V]'); legend('applied voltage u', 'estimated disturbance d'); xlabel('t [s]')
linkaxes([ax1 ax2 ax3], 'x')      % zoom on one plot zooms all three

%% 6. Save
obs_d = c2d(obs, Ts);             % ZOH discretization for the CNC (fixed step Ts)
save(['../data/ss_model_' motor '.mat'], ...
     'A', 'B', 'C', 'Aa', 'Ba', 'Ca', 'L', 'Qn', 'Rn', 'obs_d', 'Ts', 'motor')
fprintf('\nSaved ../data/ss_model_%s.mat\n', motor)
