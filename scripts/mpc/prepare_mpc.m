%% PREPARE_MPC - builds everything the MPC needs (run from scripts/mpc)
% 1. loads parameters.m and the kinematics (FK_classic, J_classic)
% 2. generates the Cartesian trajectory      -> generate_trajectory.m
% 3. inverse kinematics along the trajectory -> q_ref (Newton, warm start)
% 4. Jacobian at every point + contour/lag output maps (linearized FK)
% 5. discrete motor model (4 states, MIMO) and prediction matrices Phi, Gamma
% 6. saves everything to data/mpc_<shape>.mat and leaves it in the workspace
%
% Conventions
%   q = [phi; theta] in rad, kinematic frame of kinematics.m
%   x = [phi; dphi; theta; dtheta]   (rad, rad/s),   u = [u_3165; u_3166] (V)
%   Contour/lag (linearized FK at the reference point of the same time step k):
%       e_k = Ey_k * q_k - yref_k,   Ey_k = [n_k'; t_k'] * J(q_ref_k),   yref_k = Ey_k * q_ref_k
%       e_k(1) = contour error (normal to the path), e_k(2) = lag error (along the path)

here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'parameters.m'));                 % l, Vmax, k_*_load, tau_*_load, ...
addpath(fullfile(here, '..', 'model_id'));                 % FK_classic.m, J_classic.m
addpath(here);                                             % block functions (mpc_step, kalman_step, refWindow)
if exist('FK_classic', 'file') ~= 2 || exist('J_classic', 'file') ~= 2
    error('FK_classic / J_classic not found: run scripts/model_id/kinematics.m first.');
end

%% 1. Settings
Ts    = 0.002;          % controller sample time [s] (same as the QUARC models)
N     = 50;             % prediction horizon [samples]  -> N*Ts = 0.1 s
Nu    = 10;             % control horizon [moves]; after Nu the input is held constant
shape = 'circle';       % 'triangle' | 'square' | 'circle'

topts = struct('center', [l; l], ...   % shapes centred on home (best-conditioned pose)
               'size',   0.09,   ...   % side / diameter [m]
               'T_seg',  0.2,      ...   % [s] per side
               'T_circle', 12,   ...   % [s] per circle
               'T_app',  2);           % [s] approach home -> first point

% Cost weights: FIRST GUESS, to be tuned in simulation.
% Units: errors in m, inputs in V. Scale: 0.1 mm of contour error costs about
% as much as a 1 V input step  ->  qc*(1e-4)^2 ~ r*1^2.
w.qc = 1e8;             % contour error [1/m^2]
w.ql = 1e5;             % lag error [1/m^2] (ql << qc: lagging is cheaper than leaving the path)
w.r  = 1;               % input increment Delta u [1/V^2]

% Kalman filter (state + input disturbance), first guess
kf.sig_u = 0.1;         % [V]   process noise on the input (model error)
kf.sig_d = 0.01;        % [V]   random walk of the input disturbance per sample
kf.sig_y = (2*pi/4096)/sqrt(12);   % [rad] encoder quantization noise

% Constraints
umax = Vmax;            % |u| <= 6 V (same as the Saturation blocks)

% Sign between voltage and kinematic angle (checked on Model_MPC.slx):
% motor model = k/(tau s+1) -> 1/s -> -180/pi (encoder deg), and the FK uses
% encoder deg * pi/180 directly  ->  V > 0 gives a negative kinematic angle.
sgn = [-1, -1];         % [3165 (phi), 3166 (theta)]

%% 2. Cartesian trajectory
traj = generate_trajectory(shape, Ts, topts);
n    = numel(traj.t);

%% 3. Inverse kinematics (Newton with warm start, starts from home)
q_ref = zeros(2, n);
ik_res = zeros(1, n);
q = [0; 0];                                     % home: where the approach starts
for k = 1:n
    for it = 1:20
        e = FK_classic(q(1), q(2), l) - traj.p(:,k);
        if norm(e) < 1e-12, break; end
        q = q - J_classic(q(1), q(2), l) \ e;
    end
    q_ref(:,k) = q;
    ik_res(k)  = norm(FK_classic(q(1), q(2), l) - traj.p(:,k));
end
if max(ik_res) > 1e-9
    error('IK did not converge (max residual %.2e m): check the trajectory/workspace.', max(ik_res));
end

%% 4. Jacobians, joint velocities, contour/lag maps
J_ref  = zeros(2, 2, n);
condJ  = zeros(1, n);
qd_ref = zeros(2, n);
Ey     = zeros(2, 2, n);
yref   = zeros(2, n);
for k = 1:n
    Jk = J_classic(q_ref(1,k), q_ref(2,k), l);             % exact (symbolic) Jacobian
    J_ref(:,:,k) = Jk;
    condJ(k)     = cond(Jk);
    qd_ref(:,k)  = Jk \ traj.pd(:,k);                       % joint velocities
    Ey(:,:,k)    = [traj.nr(:,k)'; traj.tg(:,k)'] * Jk;     % rows: contour, lag
    yref(:,k)    = Ey(:,:,k) * q_ref(:,k);
end
qdd_ref = gradient(qd_ref, Ts);                             % joint accelerations (checks / FF)

if max(condJ) > 10
    warning('cond(J) reaches %.1f: the trajectory gets close to a singularity.', max(condJ));
end

%% 5. Padding: hold the last point for N+1 samples (preview window at the end)
pad = N + 1;
q_ref   = [q_ref,   repmat(q_ref(:,end), 1, pad)];
qd_ref  = [qd_ref,  zeros(2, pad)];
qdd_ref = [qdd_ref, zeros(2, pad)];
yref    = [yref,    repmat(yref(:,end), 1, pad)];
Ey      = cat(3, Ey,    repmat(Ey(:,:,end),    1, 1, pad));
J_ref   = cat(3, J_ref, repmat(J_ref(:,:,end), 1, 1, pad));

%% 6. Discrete motor model (exact ZOH), x = [phi; dphi; theta; dtheta]
% Each joint: tau*w_dot + w = b*u, q_dot = w,  b = sgn*k  (model = *_load)
k_mod   = [k_3165_load,   k_3166_load];
tau_mod = [tau_3165_load, tau_3166_load];
Ablk = cell(1,2);  Bblk = cell(1,2);
for i = 1:2
    b = sgn(i)*k_mod(i);  tau = tau_mod(i);  a = exp(-Ts/tau);
    Ablk{i} = [1, tau*(1 - a);
               0, a];
    Bblk{i} = [b*(Ts - tau*(1 - a));
               b*(1 - a)];
end
A  = blkdiag(Ablk{:});                  % 4x4
B  = blkdiag(Bblk{:});                  % 4x2
Sq = [1 0 0 0; 0 0 1 0];                % q = Sq*x (extracts the angles)
nx = 4;  nu = 2;

%% 7. Prediction matrices:  X = Phi*x_k + Gamma*U,   Qpred = Sbar*X
% X = [x_{k+1}; ...; x_{k+N}]  (4N x 1),  U = [u_k; ...; u_{k+Nu-1}]  (2Nu x 1)
Phi   = zeros(nx*N, nx);
Gfull = zeros(nx*N, nu*N);
Apow  = eye(nx);
for i = 1:N
    Apow = A*Apow;                                          % A^i
    Phi((i-1)*nx+1:i*nx, :) = Apow;
    for j = 1:i
        Gfull((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j)*B;
    end
end
Mblk = zeros(nu*N, nu*Nu);                                  % input blocking: u_j = u_{Nu-1} for j >= Nu
for j = 1:N
    Mblk((j-1)*nu+1:j*nu, (min(j,Nu)-1)*nu+1:min(j,Nu)*nu) = eye(nu);
end
Gamma = Gfull*Mblk;                                         % 4N x 2Nu
Sbar  = kron(eye(N), Sq);                                   % 2N x 4N

% Delta u:  dU = Dm*U - E0*u_{k-1}
Dm = eye(nu*Nu) - kron(diag(ones(Nu-1,1), -1), eye(nu));
E0 = [eye(nu); zeros(nu*(Nu-1), nu)];

% Joint-angle predictions used by the MPC block:  q_pred = Fq*x_k + Gq*V
Fq = Sbar*Phi;                                              % 2N x 4
Gq = Sbar*Gamma;                                            % 2N x 2Nu

%% 8. Kalman filter with input-disturbance states (offset-free)
% xa = [x; d],  x_{k+1} = A x + B (u + d),  d_{k+1} = d,  y = Sq x
Aa = [A, B; zeros(nu, nx), eye(nu)];
Ba = [B; zeros(nu)];
Ca = [Sq, zeros(nu)];
if rank(obsv_mat(Aa, Ca)) < nx + nu
    error('Augmented model not observable: check the disturbance model.');
end
Qn = blkdiag(kf.sig_u^2 * (B*B'), kf.sig_d^2 * eye(nu));     % process noise
Rn = kf.sig_y^2 * eye(nu);                                  % measurement noise
P  = eye(nx + nu);
for it = 1:20000                                            % steady-state Riccati (no toolbox)
    Pm = Aa*P*Aa' + Qn;                                     % prediction
    Lk = Pm*Ca' / (Ca*Pm*Ca' + Rn);                         % filter gain
    Pn = (eye(nx + nu) - Lk*Ca)*Pm;                         % correction
    if norm(Pn - P, 'fro') < 1e-14*norm(P, 'fro'), P = Pn; break; end
    P = Pn;
end
est = struct('Aa', Aa, 'Ba', Ba, 'Ca', Ca, 'L', Lk, 'kf', kf);
fprintf('Kalman: |eig(A - L C A)| max = %.4f (must be < 1)\n', ...
        max(abs(eig((eye(nx + nu) - Lk*Ca)*Aa))));

%% 9. Pack, save, summary
mpc = struct('Ts', Ts, 'N', N, 'Nu', Nu, 'A', A, 'B', B, 'Sq', Sq, ...
             'Phi', Phi, 'Gamma', Gamma, 'Sbar', Sbar, 'Fq', Fq, 'Gq', Gq, ...
             'Dm', Dm, 'E0', E0, 'umax', umax, 'w', w, 'qp_iter', 15, ...
             'sgn', sgn, 'k', k_mod, 'tau', tau_mod);
ref = struct('t', traj.t, 'n', n, 'p', traj.p, 'tg', traj.tg, 'nr', traj.nr, ...
             'q', q_ref, 'qd', qd_ref, 'qdd', qdd_ref, ...
             'Jflat', reshape(J_ref, 4, []), ...            % [J11; J21; J12; J22]
             'Eyflat', reshape(Ey, 4, []), ...              % [Ey11; Ey21; Ey12; Ey22]
             'yref', yref, 'condJ', condJ, 'shape', traj.shape);

out = fullfile(here, '..', '..', 'data', ['mpc_' traj.shape '.mat']);
save(out, 'mpc', 'ref', 'traj', 'est');

fprintf('Trajectory "%s": %d samples (%.1f s), horizon N = %d (%.0f ms), Nu = %d\n', ...
        traj.shape, n, n*Ts, N, N*Ts*1e3, Nu);
fprintf('IK max residual = %.1e m,  cond(J) in [%.2f, %.2f]\n', max(ik_res), min(condJ), max(condJ));
fprintf('max |q_dot| = %.1f deg/s,  QP size: %d variables, %d input bounds\n', ...
        rad2deg(max(abs(qd_ref(:)))), nu*Nu, 2*nu*Nu);
fprintf('Saved %s\n', out);

%% 10. Plain variables used as PARAMETERS by the MATLAB Function blocks
REF_q    = ref.q;          % [2 x n] joint reference [rad]
REF_Ey   = ref.Eyflat;     % [4 x n] contour/lag maps
REF_yref = ref.yref;       % [2 x n]
REF_p    = [traj.p, repmat(traj.p(:,end), 1, pad)];   % [2 x n] Cartesian reference (logging)
qp_iter  = mpc.qp_iter;
x0_est   = zeros(nx + nu, 1);  % estimator initial state: home, at rest, no disturbance
% already in the workspace: N, Fq, Gq, Dm, E0, w, umax, Aa, Ba, Ca, Lk, Ts
Qref   = REF_q;
Eyflat = REF_Ey;
L      = Lk;
x0     = x0_est;

%% 11. Quick check plots
figure('Name', 'MPC reference');
subplot(2,2,1);
p_chk = zeros(2, n);
for k = 1:n, p_chk(:,k) = FK_classic(q_ref(1,k), q_ref(2,k), l); end
plot(traj.p(1,:)*1e3, traj.p(2,:)*1e3, 'k', 'LineWidth', 2); hold on;
plot(p_chk(1,:)*1e3, p_chk(2,:)*1e3, 'r--');
idx = 1:round(n/40):n;
quiver(traj.p(1,idx)*1e3, traj.p(2,idx)*1e3, traj.nr(1,idx), traj.nr(2,idx), 0.3, 'b');
axis equal; grid on; xlabel('x [mm]'); ylabel('y [mm]');
legend('trajectory', 'FK(q_{ref})', 'normal n', 'Location', 'best'); title('Path and normals');
subplot(2,2,2);
plot(traj.t, rad2deg(q_ref(:,1:n))); grid on; xlabel('t [s]'); ylabel('[deg]');
legend('\phi', '\theta'); title('Joint reference');
subplot(2,2,3);
plot(traj.t, rad2deg(qd_ref(:,1:n))); grid on; xlabel('t [s]'); ylabel('[deg/s]');
legend('d\phi', 'd\theta'); title('Joint velocity');
subplot(2,2,4);
plot(traj.t, condJ); grid on; xlabel('t [s]'); title('cond(J) along the path');

%% ------------------------------------------------------------------------
function O = obsv_mat(A, C)
% Observability matrix (avoids a toolbox dependency)
n = size(A, 1);  O = zeros(n*size(C, 1), n);  M = C;
for i = 1:n
    O((i-1)*size(C,1)+1:i*size(C,1), :) = M;  M = M*A;
end
end
