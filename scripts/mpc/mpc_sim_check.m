%% MPC_SIM_CHECK - closed-loop test of the MPC in plain MATLAB (no Simulink)
% Uses exactly the same functions as the Simulink blocks (refWindow, kalman_step,
% mpc_step) on a "realistic" plant: *_plant gains, input delay Td, Coulomb
% friction, encoder quantization, optional step disturbance.
% Run prepare_mpc.m first (it leaves mpc, ref, est and the parameters in the workspace).

if ~exist('mpc', 'var') || ~exist('ref', 'var') || ~exist('est', 'var')
    error('Run prepare_mpc.m first.');
end

%% Settings
step_dist = true;     % 0.5 V input step on motor 3165 at t = 5 s (as in Model_MPC.slx)
umax_test = mpc.umax; % set lower (e.g. 1.5) to test the MPC with active constraints
nsub      = 20;       % plant integration sub-steps per sample

Ts = mpc.Ts;  N = mpc.N;  n = ref.n;
kp  = [k_3165_plant,   k_3166_plant];   taup = [tau_3165_plant, tau_3166_plant];
Td  = [Td_3165, Td_3166];                Vc   = [Vc_3165, Vc_3166];
h   = Ts/nsub;  nd = round(Td/h);        % delay in sub-steps
qdeg = 360/4096;                          % encoder resolution [deg]

%% Simulation
clear kalman_step                          % reset the persistent filter state
q  = [0; 0];  wk = [0; 0];                 % kinematic angles [rad] and speeds [rad/s]
ubuf = zeros(2, max(nd) + 1);              % input delay buffer (sub-step resolution)
u_prev = [0; 0];
Q = zeros(2, n);  U = zeros(2, n);  D = zeros(2, n);  IT = zeros(1, n);  TQP = zeros(1, n);
for k = 0:n-1
    % measurement (quantized encoder, converted to rad)
    y = deg2rad(qdeg*round(rad2deg(q)/qdeg));
    [xhat, dhat] = kalman_step(y, u_prev, est.Aa, est.Ba, est.Ca, est.L, zeros(6,1));
    [Qw, Eyw, yw] = refWindow(k, ref.q, ref.Eyflat, ref.yref, N);
    tic;
    [u, it] = mpc_step(xhat, dhat, Eyw, yw, u_prev, mpc.Fq, mpc.Gq, mpc.Dm, mpc.E0, ...
                       mpc.w, umax_test, mpc.qp_iter);
    TQP(k+1) = toc;
    u = min(max(u, -mpc.umax), mpc.umax);  % Saturation block
    % plant over one sample (ZOH input, delay, friction, disturbance)
    dist = [0; 0];
    if step_dist && k*Ts >= 5, dist(1) = 0.5; end
    for s = 1:nsub
        ubuf = [u, ubuf(:, 1:end-1)];
        for i = 1:2
            ui = ubuf(i, nd(i) + 1) + dist(i);
            wm = mpc.sgn(i)*wk(i);                                   % motor speed
            ueff = ui - Vc(i)*tanh(wm/w_eps);
            wm = wm + h*(kp(i)*ueff - wm)/taup(i);
            wk(i) = mpc.sgn(i)*wm;
        end
        q = q + h*wk;
    end
    Q(:, k+1) = q;  U(:, k+1) = u;  D(:, k+1) = dhat;  IT(k+1) = it;
    u_prev = u;
end

%% Contour error with the exact FK
P = zeros(2, n);
for k = 1:n, P(:,k) = FK_classic(Q(1,k), Q(2,k), l); end
Pref = ref.p(:, 1:n);
e_track = vecnorm(P - Pref);
% contour error = distance from the path (nearest point of the dense reference)
e_cont = zeros(1, n);
for k = 1:n
    e_cont(k) = min(vecnorm(Pref - P(:,k)));
end
fprintf('\n--- MPC closed loop (%s, umax = %.1f V) ---\n', ref.shape, umax_test);
fprintf('contour error: RMS %.3f mm, max %.3f mm\n', 1e3*sqrt(mean(e_cont.^2)), 1e3*max(e_cont));
fprintf('tracking error: RMS %.3f mm, max %.3f mm\n', 1e3*sqrt(mean(e_track.^2)), 1e3*max(e_track));
fprintf('|u| max = %.2f V, QP active-set iterations max = %d, QP time mean %.2f ms / max %.2f ms\n', ...
        max(abs(U(:))), max(IT), 1e3*mean(TQP), 1e3*max(TQP));

t = (0:n-1)*Ts;
figure('Name', 'MPC closed-loop check');
subplot(2,2,1); plot(Pref(1,:)*1e3, Pref(2,:)*1e3, 'k', P(1,:)*1e3, P(2,:)*1e3, 'r--');
axis equal; grid on; xlabel('x [mm]'); ylabel('y [mm]'); legend('reference', 'pen'); title('Path');
subplot(2,2,2); plot(t, e_cont*1e3, t, e_track*1e3); grid on;
xlabel('t [s]'); ylabel('[mm]'); legend('contour', 'tracking'); title('Errors');
subplot(2,2,3); plot(t, U); grid on; xlabel('t [s]'); ylabel('[V]'); legend('u_{3165}', 'u_{3166}'); title('Input');
subplot(2,2,4); plot(t, D); grid on; xlabel('t [s]'); ylabel('[V]'); legend('d_{3165}', 'd_{3166}'); title('Estimated disturbance');
