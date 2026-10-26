%% SS CONTROL - state feedback with integral action (pole placement and LQR)
% Input:  ../data/ss_model_<motor>.mat    (from ss_model.m)
% Output: ../data/ss_control_<motor>.mat  (used by Simulink)
clear; clc; close all

motor = '3165';                   % '3165' or '3166'
load(['../data/ss_model_' motor '.mat'])
n = size(A,1);
r_step = 0.1;                     % TODO: largest expected reference step [rad]

fprintf('==================== SS CONTROL - motor %s ====================\n', motor)

%% 1. Augmented state with integral action: z = [theta; omega; eI], eI_dot = r - theta
% Theory: internal model principle (integrator -> zero steady-state error for
% step references/disturbances). Augmented with the integrator, NOT with d.
Ai = [A, zeros(n,1); -C, 0];
Bi = [B; 0];
fprintf('\n--- 1. Augmented model ---\n')
fprintf('rank ctrb augmented = %d  (must be %d)\n', rank(ctrb(Ai,Bi)), n+1)

%% 2. Pole placement (state feedback u = -K z, char. poly det(sI - Ai + Bi K) = desired)
% Theory: poles from specs with a 2nd-order dominant pair (overshoot -> zeta,
% bandwidth -> wn), integrator pole slower. place = robust eigenstructure
% assignment; for one input K is unique, so acker(Ai,Bi,p) (Ackermann) gives the same K.
wn    = 20;                       % natural frequency [rad/s]
zeta  = 0.8;                      % damping
p_int = -10;                      % integrator pole [rad/s]
p = [-zeta*wn + 1j*wn*sqrt(1-zeta^2), -zeta*wn - 1j*wn*sqrt(1-zeta^2), p_int];

K_pp = place(Ai, Bi, p);

%% 3. LQR with integral action (lqi = lqr on the augmented system)
% Theory: infinite-horizon LQ, min J = int(z'Qz + u'Ru) dt, K = R^-1 Bi' S,
% S from the control Riccati eq. Guaranteed margins with full state: GM [-6 dB, inf), PM >= 60 deg.
theta_max = 0.2;                  % max acceptable angle error [rad]
omega_max = 10;                   % max acceptable speed error [rad/s]
eI_max    = 0.05;                 % max acceptable integral error [rad s]
u_max     = 6;                    % voltage limit [V]

Q = diag([1/theta_max^2, 1/omega_max^2, 1/eI_max^2]);   % Bryson's rule
R = 1/u_max^2;
K_lqr = lqi(ss(A,B,C,0), Q, R);

%% 4. Results: gains, poles, margins, step response
% Closed loop with u = -Kx*(x - [r;0]) - Ki*eI, outputs [theta; u]
designs = {K_pp, K_lqr};
names   = {'pole placement', 'LQR'};
p_obs   = eig(Aa - L*Ca);         % observer poles (from ss_model.m)

fprintf('\n--- 4. Results (step r = %.2f rad) ---\n', r_step)
fprintf('%-15s %-28s %-24s %-8s %-8s %-9s %-8s\n', ...
        'design', 'K = [Kx Ki]', '|closed-loop poles|', 'GM[dB]', 'PM[deg]', 'Mp[%]', 'max|u|')

fig = figure('Name', '1. Pole placement vs LQR');
for k = 1:2
    K  = designs{k};
    pc = eig(Ai - Bi*K);
    [Gm, Pm] = margin(ss(Ai, Bi, K, 0));          % loop L = K (sI-Ai)^-1 Bi, broken at plant input
    cl = ss(Ai - Bi*K, Bi*K(1) + [0;0;1], [C 0; -K], [0; K(1)]);
    [y, t] = step(r_step*cl, 0.5);
    s = stepinfo(y(:,1), t);

    fprintf('%-15s %-28s %-24s %-8.1f %-8.1f %-9.1f %-8.2f\n', names{k}, ...
            mat2str(K,4), mat2str(abs(pc).',3), 20*log10(Gm), Pm, s.Overshoot, max(abs(y(:,2))))

    subplot(2,2,1); plot(t, y(:,1)); hold on
    subplot(2,2,3); plot(t, y(:,2)); hold on
    subplot(2,2,[2 4]); plot(real(pc), imag(pc), 'x', 'MarkerSize', 10, 'LineWidth', 2); hold on
end
subplot(2,2,1); yline(r_step, 'k:'); grid on; ylabel('\theta [rad]'); legend(names, 'Location', 'southeast')
title(['Step response - motor ' motor])
subplot(2,2,3); yline([-u_max u_max], 'r:'); grid on; ylabel('u [V]'); xlabel('t [s]')
subplot(2,2,[2 4]); plot(real(p_obs), imag(p_obs), 'ko', 'MarkerSize', 8); grid on
xlabel('Re'); ylabel('Im'); legend([names, {'observer'}], 'Location', 'northwest')
title('Poles (controller x, observer o)')

% Theory: separation principle -> closed loop with observer has poles
% eig(Ai - Bi K) U eig(Aa - L Ca). LQR + Kalman = LQG: the LQR margins above
% are NOT guaranteed with the observer (check the LQG loop, LTR if needed).
ratio = min(abs(p_obs)) / max(abs(eig(Ai - Bi*K_lqr)));
fprintf('\nslowest observer pole / fastest LQR pole = %.1f  (aim 3-5)\n', ratio)

%% 5. Save
K  = K_lqr;                       % choose K_pp or K_lqr
Kx = K(1:n);  Ki = K(n+1);
save(['../data/ss_control_' motor '.mat'], 'Kx', 'Ki', 'K_pp', 'K_lqr')
fprintf('Saved ../data/ss_control_%s.mat  (Kx = %s, Ki = %.4g)\n', motor, mat2str(Kx,4), Ki)
