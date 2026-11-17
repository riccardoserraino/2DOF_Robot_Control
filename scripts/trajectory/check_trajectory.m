%% CHECK TRAJECTORY - joint angles, speeds and feedforward voltage of a path, before running it
%
% Runs one of the *_gen_smooth functions in MATLAB (no Simulink), converts the tip path
% into joint angles with the inverse kinematics of the four-bar linkage and checks:
%   - joint angles within the limit (+-20 deg, warning above q_warn)
%   - joint speed and acceleration
%   - feedforward voltage u_ff = (dq + tau*ddq)/k + Vc: how much of the 6 V the path
%     needs before the feedback adds anything
% The four *_gen_smooth.m files must be on the MATLAB path (same folder as this script).
% Theory: u_ff grows with speed (velocity term) and speed^2 (acceleration term), so
% doubling "speed" in a trajectory doubles the joint speeds and multiplies the joint
% accelerations by 4.

clear; clc; close all

%% 1. Settings
traj   = @square_gen_smooth;   % @square_gen_smooth, @triangle_gen_smooth, @circle_gen_smooth, @star_gen_smooth
t_end  = 12;                   % [s] simulated time (approach + a few laps)
Ts     = 0.002;                % [s] sample time
q_lim  = 20;                   % [deg] joint limit
q_warn = 18;                   % [deg] warning level (margin for the tracking error)
% motor model and geometry: read from parameters.m (plant values, as the controller uses).
% Assumption: phi = motor 3165, theta = motor 3166.
this = fileparts(mfilename('fullpath'));
run(fullfile(this, '..', 'parameters.m'));
k    = [k_3165_plant,   k_3166_plant];      % motor gains [rad/(V s)], [phi, theta]
tau  = [tau_3165_plant, tau_3166_plant];    % motor time constants [s]
Vc   = [Vc_3165,        Vc_3166];           % Coulomb friction [V] (0 if use_real_plant = false)
% l (link length) and Vmax (amplifier limit) also come from parameters.m
u_target = 3;                  % [V] feedforward voltage you want to use at the peak: the
                               %     script prints the "speed" that reaches it (section 6)

%% 2. Tip path
t = (0:Ts:t_end)';
N = numel(t);
P = zeros(N, 2);  Vt = zeros(N, 2);  At = zeros(N, 2);
for n = 1:N
    [P(n,1), P(n,2), Vt(n,1), Vt(n,2), At(n,1), At(n,2)] = traj(t(n));
end

%% 3. Inverse kinematics (Newton, starting from home q = [0 0] and then from the last q)
q  = zeros(N, 2);   % [rad] [phi, theta]
dq = zeros(N, 2);   % [rad/s]
qn = [0; 0];
for n = 1:N
    for it = 1:30
        r = fk(qn, l) - P(n, :)';
        if norm(r) < 1e-12, break, end
        qn = qn - jac(qn, l)\r;
    end
    q(n, :)  = qn';
    dq(n, :) = (jac(qn, l)\Vt(n, :)')';     % joint speed from the tip speed
end
ddq = [zeros(1, 2); diff(dq)/Ts];           % joint acceleration (numerical)

%% 4. Feedforward voltage of each joint (as in the Simulink feedforward)
u_ff = (abs(dq + tau.*ddq))./k + Vc.*(abs(dq) > 1e-3);

%% 5. Summary
qd = rad2deg(q);
names = {'phi  (left motor) ', 'theta (right motor)'};
fprintf('Trajectory: %s,  %g s\n\n', func2str(traj), t_end);
fprintf('%-20s %9s %9s %10s %12s %9s\n', 'joint', 'min [deg]', 'max [deg]', 'max speed', 'max accel', 'max u_ff');
fprintf('%-20s %9s %9s %10s %12s %9s\n', '', '', '', '[rad/s]', '[rad/s^2]', '[V]');
for i = 1:2
    fprintf('%-20s %9.1f %9.1f %10.2f %12.1f %9.2f\n', names{i}, min(qd(:, i)), max(qd(:, i)), ...
            max(abs(dq(:, i))), max(abs(ddq(:, i))), max(u_ff(:, i)));
end
fprintf('\ntip: max speed %.3f m/s, max acceleration %.2f m/s^2\n', ...
        max(vecnorm(Vt, 2, 2)), max(vecnorm(At, 2, 2)));
qmax = max(abs(qd(:)));
if qmax > q_lim
    fprintf('NOT OK: a joint reaches %.1f deg (limit %g): make the path smaller.\n', qmax, q_lim);
elseif qmax > q_warn
    fprintf('WARNING: a joint reaches %.1f deg (above %g, close to the limit).\n', qmax, q_warn);
else
    fprintf('OK: joints within %.1f deg (limit %g).\n', qmax, q_lim);
end
fprintf('Feedforward uses at most %.0f %% of the %g V (the rest is left to the feedback).\n', ...
        100*max(u_ff(:))/Vmax, Vmax);

%% 6. Speed that uses u_target volts of feedforward
% Over the laps the joint speeds scale with "speed" and the accelerations with speed^2:
%   u_ff(t) = |s*dq(t) + s^2*tau*ddq(t)|/k + Vc     (s = new speed / speed now)
% The bisection finds the s whose peak u_ff equals u_target (only the laps are scaled:
% the approach from home has a fixed time).
laps = t >= 1.5;                                   % t_app + t_wait of the functions
peak = @(s) max(max(abs(s*dq(laps, :) + s^2*tau.*ddq(laps, :))./k + Vc));
lo = 0.01;  hi = 50;
for it = 1:60
    mid = (lo + hi)/2;
    if peak(mid) < u_target, lo = mid; else, hi = mid; end
end
s_fac = lo;
fprintf(['\nTo use %.1f V of feedforward at the peak: multiply "speed" by %.2f\n' ...
         '  -> tip up to %.3f m/s and %.2f m/s^2, joints up to %.2f rad/s and %.1f rad/s^2\n'], ...
        u_target, s_fac, s_fac*max(vecnorm(Vt(laps, :), 2, 2)), s_fac^2*max(vecnorm(At(laps, :), 2, 2)), ...
        s_fac*max(max(abs(dq(laps, :)))), s_fac^2*max(max(abs(ddq(laps, :)))));
fprintf('  (the feedback adds its own voltage on top: keep u_target well below %g V)\n', Vmax);

%% 7. Plots
figure('Name', 'Trajectory check', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile; hold on; grid on; axis equal; box on
plot(P(:, 1)*1e3, P(:, 2)*1e3, 'LineWidth', 1.2);
plot(P(1, 1)*1e3, P(1, 2)*1e3, 'ko', 'MarkerFaceColor', 'k');
xlabel('x [mm]'); ylabel('y [mm]'); title('Tip path (dot = home)');

nexttile; hold on; grid on; box on
plot(t, qd, 'LineWidth', 1.2);
yline([-q_lim q_lim], 'r-');  yline([-q_warn q_warn], 'r:');
xlabel('t [s]'); ylabel('[deg]'); title('Joint angles'); legend('\phi', '\theta', 'Location', 'best');

nexttile; hold on; grid on; box on
plot(t, dq, 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('[rad/s]'); title('Joint speeds');

nexttile; hold on; grid on; box on
plot(t, u_ff, 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('[V]'); title('Feedforward voltage |u_{ff}|');


%% ---------------- local functions ----------------
function p = fk(q, l)
% Tip C of the four-bar linkage (kinematics.m): B = l[cos phi; sin phi],
% D = [2l - l sin theta; l cos theta], C on the perpendicular of BD.
phi = q(1);  th = q(2);
a = 2 - sin(th) - cos(phi);            % (Dx - Bx)/l
b = cos(th) - sin(phi);                % (Dy - By)/l
s = sqrt(a^2 + b^2);                   % |BD|/l
h = l*sqrt(1 - s^2/4);                 % distance of C from the midpoint of BD
p = [l/2*(2 - sin(th) + cos(phi)) - h*b/s;
     l/2*(cos(th) + sin(phi))     + h*a/s];
end

function J = jac(q, l)
% Jacobian dp/dq by central differences (accurate enough for the check).
e = 1e-7;
J = [fk(q + [e; 0], l) - fk(q - [e; 0], l), fk(q + [0; e], l) - fk(q - [0; e], l)]/(2*e);
end