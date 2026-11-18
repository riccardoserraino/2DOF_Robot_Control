%% CHECK TRAJECTORY - joint angles, speeds and feedforward voltage of a path, before running it
%
% Runs one of the *_gen_smooth functions in MATLAB and the selected Simulink model, converts the tip
% path into joint angles with the inverse kinematics of the four-bar linkage and checks:
%   - joint angles within the limit (+-20 deg, warning above q_warn)
%   - joint speed and acceleration
%   - estimated feedforward voltage, and total / saturated voltage from the Simulink plant
% The four *_gen_smooth.m files must be on the MATLAB path (same folder as this script).
% Theory: u_ff grows with speed (velocity term) and speed^2 (acceleration term), so
% doubling "speed" in a trajectory doubles the joint speeds and multiplies the joint
% accelerations by 4.
% MATH STEPS OF THE SCRIPT:
% 1. Generate the tip path (position, velocity, acceleration) from the traj file selected.
% 2. Inverse kinematics (Newton) to get joint angles and speeds. (first it gets joint angles with forward kinematics)
% 3. Joint acceleration (numerical differentiation of the speed).
% 4. Feedforward voltage of each joint (inverse formula from the controller model).
% 5. Run the selected Simulink model; summarize the simulated voltage and tracking.
% 6. Summary and speed recommendation from the analytical feedforward estimate.


clear; clc; close all

%% 1. Settings
traj   = @square_gen_smooth;   % @square_gen_smooth, @triangle_gen_smooth, @circle_gen_smooth, @star_gen_smooth
t_end  = 12;                   % [s] simulated time (approach + a few laps)
Ts     = 0.002;                % [s] sample time
q_lim  = 20;                   % [deg] joint limit
q_warn = 18;                   % [deg] warning level (margin for the tracking error)
model_name = 'Model_pp_obs';   % Simulink model in slx/
controller_setup = fullfile('scripts', 'control', 'pp_design', 'design_pp.m');
                               % setup script must publish Simulink variables to the base workspace
                               % use '' to simulate with variables already loaded
% To check another controller, set model_name and controller_setup to its model/setup pair.
% motor model and geometry: read from parameters.m (plant values, as the controller uses).
% Assumption: phi = motor 3165, theta = motor 3166.
% Run from the repository root so these relative paths resolve in MATLAB too.
this = fullfile(pwd, 'scripts', 'trajectory');
run('scripts/parameters.m');
if ~isempty(controller_setup)
    run_controller_setup(controller_setup);
end
k    = [k_3165_plant,   k_3166_plant];      % motor gains [rad/(V s)], [phi, theta]
tau  = [tau_3165_plant, tau_3166_plant];    % motor time constants [s]
Vc   = [Vc_3165,        Vc_3166];           % Coulomb friction [V] (0 if use_real_plant = false)
% l (link length) and Vmax (amplifier limit) also come from parameters.m
u_target = 3;                  % [V] feedforward voltage you want to use at the peak: the
                               %     script prints the "speed" that reaches it (section 6)
model_file = fullfile(this, '..', '..', 'slx', [model_name '.slx']);

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
fprintf('\n========== TRAJECTORY CHECK (MATLAB estimate) ==========\n');
fprintf('Path: %s | analysis duration: %.1f s\n\n', func2str(traj), t_end);
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

%% 6. Run the Simulink model and summarize its recorded outputs
fprintf('\n========== SIMULINK RUN ==========\n');
fprintf('Model: %s\n', model_file);
load_system(model_file);
setappdata(0, 'run_and_save_active', true);
sim_cleanup = onCleanup(@() setappdata(0, 'run_and_save_active', false));
simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
clear sim_cleanup
setappdata(0, 'run_and_save_active', false);
sim_t = simOut.get('u_3165').Time;
u_total = [sim_ts(simOut, 'u_3165', sim_t), sim_ts(simOut, 'u_3166', sim_t)];
u_plant = [sim_ts(simOut, 'usat_3165', sim_t), sim_ts(simOut, 'usat_3166', sim_t)];
xc_des = sim_ts(simOut, 'xc_des', sim_t);  yc_des = sim_ts(simOut, 'yc_des', sim_t);
xc_sim = sim_ts(simOut, 'xc_sim', sim_t);  yc_sim = sim_ts(simOut, 'yc_sim', sim_t);
u_ff_sim = interp1(t, u_ff, sim_t, 'linear', 'extrap'); % summary estimate; no separate FF plot
r_sim = [sim_ts(simOut, 'r_3165', sim_t), sim_ts(simOut, 'r_3166', sim_t)];
y_sim = [sim_ts(simOut, 'y_3165', sim_t), sim_ts(simOut, 'y_3166', sim_t)];
dt_sim = mean(diff(sim_t));             % Simulink logs these signals at a fixed sample time
dq_sim = gradient(r_sim, dt_sim);
fprintf('Completed: %.2f s | %d samples | sample time %.4f s\n\n', sim_t(end), numel(sim_t), dt_sim);
fprintf('%-20s %12s %12s %12s %16s\n', 'motor', 'peak cmd [V]', 'peak plant [V]', ...
        'peak FF est [V]', 'integral |u| [V s]');
for i = 1:2
    plant_use = trapz(sim_t, abs(u_plant(:, i)));
    fprintf('%-20s %12.3f %12.3f %12.3f %16.3f\n', names{i}, ...
        max(abs(u_total(:, i))), max(abs(u_plant(:, i))), max(abs(u_ff_sim(:, i))), plant_use);
end
fprintf('\nPlant peak: %.3f V / %.3f V limit (%.1f%%)\n', ...
    max(abs(u_plant(:))), Vmax, 100*max(abs(u_plant(:)))/Vmax);

%% 7. Speed that uses u_target volts of feedforward
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
fprintf('\n========== SPEED RECOMMENDATION ==========\n');
fprintf(['For %.1f V peak feedforward, scale trajectory speed by %.2f\n' ...
         '  Tip peaks: %.3f m/s, %.2f m/s^2\n' ...
         '  Joint peaks: %.2f rad/s, %.1f rad/s^2\n'], ...
        u_target, s_fac, s_fac*max(vecnorm(Vt(laps, :), 2, 2)), s_fac^2*max(vecnorm(At(laps, :), 2, 2)), ...
        s_fac*max(max(abs(dq(laps, :)))), s_fac^2*max(max(abs(ddq(laps, :)))));
fprintf('  Feedback adds voltage too; amplifier limit is %.1f V.\n', Vmax);

%% 8. Plots
figure('Name', 'Trajectory check - Simulink results', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile; hold on; grid on; axis equal; box on
plot(xc_des*1e3, yc_des*1e3, 'k--', 'LineWidth', 1.1);
plot(xc_sim*1e3, yc_sim*1e3, 'b-', 'LineWidth', 1.2);
xlabel('x [mm]'); ylabel('y [mm]'); title('Tip path: desired vs followed');
legend('desired path', 'followed by plant', 'Location', 'best');

nexttile; hold on; grid on; box on
plot(sim_t, r_sim, '--', 'LineWidth', 1.0);
plot(sim_t, y_sim, 'LineWidth', 1.0);
yline([-q_lim q_lim], 'r:');
xlabel('t [s]'); ylabel('[deg]'); title('Joint angles: reference and plant');
legend('r 3165', 'r 3166', 'y 3165', 'y 3166', 'joint limit', 'Location', 'best');

nexttile; hold on; grid on; box on
plot(sim_t, dq_sim, 'LineWidth', 1.0);
xlabel('t [s]'); ylabel('[deg/s]'); title('Joint speed from reference');
legend('3165', '3166', 'Location', 'best');

nexttile; hold on; grid on; box on
plot(sim_t, u_total, 'LineWidth', 1.0);
plot(sim_t, u_plant, '--', 'LineWidth', 1.0);
yline(Vmax, 'r--', 'LineWidth', 1.0);
yline(-Vmax, 'r--', 'LineWidth', 1.0);
xlabel('t [s]'); ylabel('[V]'); title('Voltage reference and saturation limits');
legend('u 3165', 'u 3166', 'plant 3165', 'plant 3166', 'Location', 'best');


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

function y = sim_ts(simOut, name, t)
% Read a Simulink To Workspace timeseries and align it to the requested time vector.
if ~any(strcmp(simOut.who, name))
    error('Model output "%s" is missing. Check the To Workspace blocks in the selected model.', name);
end
v = simOut.get(name);
if isa(v, 'timeseries')
    y = interp1(v.Time(:), squeeze(v.Data), t, 'linear', 'extrap');
elseif isstruct(v) && isfield(v, 'time') && isfield(v, 'signals')
    y = interp1(v.time(:), squeeze(v.signals.values), t, 'linear', 'extrap');
else
    error('Unsupported format for Simulink output "%s".', name);
end
y = y(:);
end

function run_controller_setup(setup_file)
% Run the selected controller's setup script in an isolated workspace so
% CLEAR/CLC/CLOSE ALL inside it cannot erase this checker's data or plots.
% The setup script is responsible for publishing its Simulink variables to base.
if ~isfile(setup_file)
    error('Controller setup script not found: %s', setup_file);
end
run(setup_file);
end
