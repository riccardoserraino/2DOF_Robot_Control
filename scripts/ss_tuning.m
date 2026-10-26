%% SS TUNING - compute the constants for ss_control.m and ss_model.m
% Reference = the PID already working on the robot. The script only computes:
%   1. wn, zeta, p_int                -> ss_control.m (pole placement)
%   2. theta_max, omega_max, eI_max   -> ss_control.m (LQR)
%   3. Qn, two approaches compared    -> ss_model.m   (Kalman filter)
%   4. (optional) systune gain K      -> ss_control.m
% At the end it prints what to put in Simulink: InitFcn, mask of ss_online,
% 'Tuning constants' of ss_online (one block per motor).
% Input: ../data/ss_model_<motor>.mat (run ss_model.m once first, any Qn)
clear; clc; close all
run('parameters.m')
run('pid_tune.m')                         % PID gains

motor = '3165';                           % TODO: '3165' or '3166'
speed = 1;                                % TODO: 1 = as fast as the PID, 1.5 = 50% faster
datafile = ['../data/data_' motor '_seed30_load.mat'];   % estimation data (ss_model.m validates on seed29)

load(['../data/ss_model_' motor '.mat'])  % A, B, C, Aa, Ba, Ca, Rn, Ts
Ai = [A, zeros(2,1); -C, 0];              % augmented with integrator (as in ss_control.m)
Bi = [B; 0];

%% 0. Target from the PID closed loop
if strcmp(motor, '3165'), Cp = contr_3165; Gp = sys_3165_load;
else,                     Cp = contr_3166; Gp = sys_3166_load;
end
Cp = pid(Cp.Kp, Cp.Ki, Cp.Kd, 0.01);      % derivative filter N = 100, as in Simulink
T_pid = feedback(Cp*Gp, 1);

wb = speed*bandwidth(T_pid);              % target closed-loop bandwidth [rad/s]
Mp = max(stepinfo(T_pid).Overshoot, 1)/100;   % target overshoot (at least 1%)

%% 1. Pole placement: specs -> 2nd-order dominant poles
% Theory: Mp = exp(-pi*zeta/sqrt(1-zeta^2)),  wb = wn*sqrt(1-2z^2+sqrt(4z^4-4z^2+2))
zeta  = -log(Mp)/sqrt(pi^2 + log(Mp)^2);
wn    = wb/sqrt(1 - 2*zeta^2 + sqrt(4*zeta^4 - 4*zeta^2 + 2));
p_int = -zeta*wn/4;                       % integrator pole 4x slower

%% 2. LQR: Bryson weights scaled by rho until the bandwidth equals wb
% Theory: only Q/R matters. Q/rho = same Bryson rule with all limits * sqrt(rho).
theta_max = 0.2;  omega_max = 10;  eI_max = 0.05;  u_max = 6;   % proportions (Bryson)
Q0 = diag([1/theta_max^2, 1/omega_max^2, 1/eI_max^2]);
R  = 1/u_max^2;

rho = logspace(-3, 3, 61);
bw  = zeros(size(rho));
for i = 1:numel(rho)
    K = lqi(ss(A,B,C,0), Q0/rho(i), R);
    bw(i) = bandwidth(ss(Ai - Bi*K, Bi*K(1) + [0;0;1], [C 0], 0));
end
[~, ir] = min(abs(log(bw/wb)));
rho_best = rho(ir);
K_lqr = lqi(ss(A,B,C,0), Q0/rho_best, R);
w_ctrl = max(abs(eig(Ai - Bi*K_lqr)));    % fastest controller pole

%% 3a. Kalman, approach A: speed rule, Qn = diag([0 q q]), observer 4x faster than the LQR
% Theory: separation principle; fast observer -> LQG loop close to the LQR loop.
q  = logspace(-6, 6, 121);
wo = zeros(size(q));
for i = 1:numel(q)
    L = lqe(Aa, eye(3), Ca, diag([0 q(i) q(i)]), Rn);
    wo(i) = min(abs(eig(Aa - L*Ca)));     % slowest observer pole
end
[~, iq] = min(abs(log(wo/(4*w_ctrl))));
Qn_A = diag([0 q(iq) q(iq)]);

%% 3b. Kalman, approach B: Qn estimated from data (maximum likelihood of the innovations)
% Theory: the Kalman gain is minimum-covariance only with the TRUE Qn.
% Choose Qn that makes the innovations e = y - C x_hat most consistent with
% their predicted variance S:  max logL = -1/2 sum( log(2 pi S) + e^2/S )
data = load(datafile);
u = data.data(2,:).';
y = data.data(3,:).'*pi/180;              % measured angle [rad], encoder sign

w_ref = gradient(smoothdata(y, 'sgolay', 25), Ts);
use   = abs(w_ref) > 0.5;                 % skip low speed (quantization not white)
use(1:round(0.5/Ts)) = false;             % skip the filter transient

negL = @(p) -loglik(Aa, Ba, Ca, diag([0 10^p(1) 10^p(2)]), Rn/Ts, Ts, u, y, use);
p    = fminsearch(negL, log10([2.5e-2 0.9]));     % search in log10, start from typical values
Qn_B = diag([0 10^p(1) 10^p(2)]);

% Comparison: observer speed of the two Qn
wA = min(abs(eig(Aa - lqe(Aa, eye(3), Ca, Qn_A, Rn)*Ca)));
wB = min(abs(eig(Aa - lqe(Aa, eye(3), Ca, Qn_B, Rn)*Ca)));

%% Plot: the knobs
figure('Name', 'Tuning knobs')
subplot(1,2,1); loglog(rho, bw, rho_best, bw(ir), 'ro'); yline(wb, 'k:'); grid on
xlabel('\rho'); ylabel('closed-loop bandwidth [rad/s]'); title('LQR')
subplot(1,2,2); loglog(q, wo, q(iq), wo(iq), 'ro'); hold on
yline(4*w_ctrl, 'k:'); yline(wB, 'b--'); grid on
xlabel('q'); ylabel('slowest observer pole [rad/s]'); title('Kalman')
legend('speed rule sweep', 'A: speed rule', '4x controller', 'B: max likelihood', 'Location', 'southeast')

%% 4. (Optional) systune: impose the requirements directly
% Not needed for the constants above. systune tunes K = [Kx Ki] by minimizing
% H-infinity norms that encode the requirements (speed, damping, voltage limit).
use_systune = false;                      % TODO: true to compute K_tuned
if use_systune
    r_step = 0.1;                         % largest expected reference step [rad]
    K0  = realp('K', K_lqr);              % tunable gain, start from the LQR
    CL0 = ss(Ai - Bi*K0, Bi*K0(1) + [0;0;1], [C 0; -K0], [0; K0(1)]);
    CL0.InputName = 'r';  CL0.OutputName = {'theta', 'u'};
    soft = [TuningGoal.StepTracking('r', 'theta', 1/wb), TuningGoal.Poles(0, 0.7, 10*wb)];
    hard =  TuningGoal.Gain('r', 'u', u_max/r_step);
    CL = systune(CL0, soft, hard);
    K_tuned = getBlockValue(CL, 'K');     % for ss_control.m (offline); ss_online computes its own K
end

%% 5. What to put in Simulink (printed at the end, ready to copy)
if wB >= 3*w_ctrl, Qn = Qn_B; w_obs = wB; qn_note = 'B: max likelihood';
else,              Qn = Qn_A; w_obs = wA; qn_note = 'A: speed rule (B too slow)';
end
blk = 'Controller 3165';  if strcmp(motor, '3166'), blk = 'Controller 1'; end
s = sqrt(rho_best);                       % LQR limits scaled by rho

fprintf('\n=================== SIMULINK - motor %s ===================\n', motor)
fprintf('\n(1) InitFcn of CNC.slx (Model Properties > Callbacks), once for both motors:\n')
fprintf('    run(''parameters.m'')     %% defines k_%s_load, tau_%s_load\n', motor, motor)

fprintf('\n(2) Mask of the MATLAB Function ''ss_online'' in subsystem ''%s'':\n', blk)
fprintf('    k   = k_%s_load\n', motor)
fprintf('    tau = tau_%s_load\n', motor)

fprintf('\n(3) ''Tuning constants'' section of ss_online in ''%s'' (this block only):\n', blk)
fprintf('    method = 1;                             %% 1 = LQR, 2 = pole placement\n')
fprintf('    Qc = diag([1/%.4g^2, 1/%.4g^2, 1/%.4g^2]);   %% LQR, bandwidth %.1f rad/s\n', ...
        theta_max*s, omega_max*s, eI_max*s, wb)
fprintf('    Rc = 1/u_max^2;\n')
fprintf('    wn = %.1f;  zeta = %.2f;  p_int = %.1f;     %% pole placement\n', wn, zeta, p_int)
fprintf('    Qn = diag([0, %.3g, %.3g]);   %% Kalman, %s, observer %.0f rad/s (%.1fx controller)\n', ...
        Qn(2,2), Qn(3,3), qn_note, w_obs, w_obs/w_ctrl)
fprintf('    %% other Qn: A = diag([0, %.3g, %.3g]) (%.0f rad/s),  B = diag([0, %.3g, %.3g]) (%.0f rad/s)\n', ...
        Qn_A(2,2), Qn_A(3,3), wA, Qn_B(2,2), Qn_B(3,3), wB)

fprintf('\n(4) Offline scripts (only for checks and report figures):\n')
fprintf('    ss_model.m:    Qn = diag([0, %.3g, %.3g]);\n', Qn(2,2), Qn(3,3))
fprintf('    ss_control.m:  wn = %.1f;  zeta = %.2f;  p_int = %.1f;\n', wn, zeta, p_int)
fprintf('    ss_control.m:  theta_max = %.4g;  omega_max = %.4g;  eI_max = %.4g;\n', ...
        theta_max*s, omega_max*s, eI_max*s)
if use_systune
    fprintf('    ss_control.m:  (optional, systune) K = %s;\n', mat2str(K_tuned, 5))
end

%% =====================================================================
function logL = loglik(Aa, Ba, Ca, Qn, Rd, Ts, u, y, use)
% Log-likelihood of the innovations for a continuous Qn (data sampled at Ts)
n  = size(Aa,1);
sysd = c2d(ss(Aa, Ba, Ca, 0), Ts);
F  = expm([-Aa, Qn; zeros(n), Aa']*Ts);          % Van Loan: continuous Qn -> per-sample Qd
Qd = F(n+1:end, n+1:end)'*F(1:n, n+1:end);  Qd = (Qd + Qd')/2;
try
    [M, P] = dlqe(sysd.A, eye(n), sysd.C, Qd, Rd);   % P = a-priori error covariance
catch
    logL = -Inf;  return                             % Riccati not solvable: discard
end
S = sysd.C*P*sysd.C' + Rd;                           % predicted innovation variance
pred = ss(sysd.A*(eye(n) - M*sysd.C), [sysd.B, sysd.A*M], -sysd.C, [0 1], Ts);  % output = innovation
e = lsim(pred, [u, y], (0:numel(y)-1)'*Ts, [y(1); 0; 0]);
logL = -0.5*sum(log(2*pi*S) + e(use).^2/S);
end
