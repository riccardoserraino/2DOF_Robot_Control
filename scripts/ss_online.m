function [u, xhat, K, L] = ss_online(phi_ref_deg, dphi_ref, u_ff, meas_deg, k, tau)
%SS_ONLINE  State-space control of one motor, everything computed online.
% Code for a MATLAB Function block in CNC.slx (one block per motor), Ts = 2 ms.
%   - model and gains computed at the first step (LQR or pole placement)
%   - Kalman filter with gain L updated at every step (predict / update)
% Only basic matrix operations: compatible with code generation (QUARC).
%
% Inputs  phi_ref_deg  joint reference [deg]        (Input3165 / Input3166)
%         dphi_ref     joint speed reference [rad/s] (From 'dphi' / 'dtheta')
%         u_ff         feedforward voltage [V]       (sum of the two feedforward gains)
%         meas_deg     encoder angle [deg]           (Measure3165 / Measure3166)
% Params  k, tau       identified model (block mask: k_3165_load, tau_3165_load)
% Outputs u            saturated voltage to the motor [V]
%         xhat         [theta_hat; omega_hat; d_hat]  (logging)
%         K            [Kx Ki] state-feedback gain     (logging)
%         L            current Kalman gain             (logging: converges to a constant)

%% Tuning constants (TODO: choose, e.g. with ss_tuning.m)
Ts     = 0.002;                         % sample time [s]
u_max  = 6;                             % voltage limit [V]
method = 1;                             % 1 = LQR, 2 = pole placement
Qc = diag([1/0.2^2, 1/10^2, 1/0.05^2]); % LQR weights on [theta; omega; eI] (Bryson)
Rc = 1/u_max^2;                         % LQR weight on u
wn = 40;  zeta = 0.8;  p_int = -10;     % pole placement: desired continuous poles
Qn = diag([0, 2.5e-2, 0.9]);            % Kalman process noise density [theta, omega, d]
Rn = (2*pi/4096)^2/12;                  % encoder quantization variance per sample [rad^2]

persistent Ad Bd Aa Ba Ca Qd Kx Ki xh P eI
if isempty(Ad)
    %% 1. Discrete model (exact ZOH of b/(s(tau s + 1))), states [theta; omega]
    b  = -k;                            % encoder sign: angle decreases for V > 0
    a  = exp(-Ts/tau);
    Ad = [1, tau*(1-a); 0, a];
    Bd = b*[Ts - tau*(1-a); 1-a];

    %% 2. Augmented with input disturbance d (d_dot = 0): x_a = [theta; omega; d]
    Aa = [Ad, Bd; 0 0 1];
    Ba = [Bd; 0];
    Ca = [1 0 0];
    Qd = Qn*Ts;                         % continuous density -> covariance per sample

    %% 3. Augmented with integral action: z = [theta; omega; eI], eI(k+1) = eI + Ts (r - theta)
    Ai = [Ad, [0; 0]; -Ts, 0, 1];
    Bi = [Bd; 0];

    %% 4. State-feedback gain
    if method == 1
        Kz = dlqr_riccati(Ai, Bi, Qc, Rc);                % LQR
    else
        r  = exp(-zeta*wn*Ts);  wd = wn*sqrt(1 - zeta^2);
        Kz = acker3(Ai, Bi, 2*r*cos(wd*Ts), r^2, exp(p_int*Ts));   % pole placement
    end
    Kx = Kz(1:2);  Ki = Kz(3);

    %% 5. Initial conditions
    xh = [meas_deg*pi/180; 0; 0];       % estimate: measured angle, at rest, no disturbance
    P  = diag([Rn, 1, 1]);              % initial uncertainty
    eI = 0;
end

theta   = meas_deg*pi/180;              % [rad]
phi_ref = phi_ref_deg*pi/180;           % [rad]

%% 6. Kalman UPDATE with the new measurement (gain L computed online)
L  = P*Ca' / (Ca*P*Ca' + Rn);           % Kalman gain (minimum error covariance)
xh = xh + L*(theta - Ca*xh);            % corrected estimate x_hat(k|k)
P  = (eye(3) - L*Ca)*P;                 % corrected covariance

%% 7. Control law: feedforward + state feedback + integral - disturbance
v = u_ff - Kx*(xh(1:2) - [phi_ref; dphi_ref]) - Ki*eI - xh(3);
u = min(max(v, -u_max), u_max);         % saturation
if u == v
    eI = eI + Ts*(phi_ref - theta);     % anti-windup: integrate only if not saturated
end

%% 8. Kalman PREDICT for the next step, with the saturated u
xhat = xh;
xh   = Aa*xh + Ba*u;
P    = Aa*P*Aa' + Qd;
K    = [Kx Ki];
end

%% ---------------------------------------------------------------------
function K = dlqr_riccati(A, B, Q, R)
% Discrete LQR: iterate the Riccati equation until convergence
%   S = Q + A'S(A - BK),  K = (R + B'SB)^-1 B'SA
S = Q;
K = zeros(1, 3);
for i = 1:3000
    K = (R + B'*S*B) \ (B'*S*A);
    S = Q + A'*S*(A - B*K);
end
end

function K = acker3(A, B, c1, c0, z3)
% Ackermann (3 states): K = [0 0 1] Wc^-1 Delta(A)
% desired poles: z^2 - c1 z + c0 (complex pair) and z3 (integrator)
Wc    = [B, A*B, A*A*B];                % controllability matrix
Delta = (A*A - c1*A + c0*eye(3)) * (A - z3*eye(3));
K     = [0 0 1] / Wc * Delta;
end
