%% SQUARE TRAJECTORY
% phi_input and theta_input are matrices containing angle reference and
% timestep for simulink input 
clear; clc; close all;
run('parameters.m');
run('kinematics.m');

function [q_sol, p_check, position_error, exitflag] = ...
    inverse_kinematic(p_des, q0, l, display_mode)

    err_fun = @(q) FK_classic(q(1), q(2), l) - p_des;

    opts = optimoptions('fsolve', ...
        'Display', display_mode, ...
        'FunctionTolerance', 1e-10, ...
        'StepTolerance', 1e-10);

    [q_sol, ~, exitflag] = fsolve(err_fun, q0, opts);

    p_check = FK_classic(q_sol(1), q_sol(2), l);
    position_error = norm(p_check - p_des);
end

%% Trajectory parameters
side = 0.05;       % Square side [m]
N_side = 50;       % Intervals per side
T = 5;             % Total time [s]

p_home = FK_classic(0, 0, l);
half_side = side/2;

corners = [p_home(1)-half_side, p_home(2)-half_side;
           p_home(1)+half_side, p_home(2)-half_side;
           p_home(1)+half_side, p_home(2)+half_side;
           p_home(1)-half_side, p_home(2)+half_side;
           p_home(1)-half_side, p_home(2)-half_side];

%% Generate Cartesian trajectory
x_traj = [];
y_traj = [];

for i = 1:4
    s = linspace(0, 1, N_side+1);
    x = corners(i,1) + s*(corners(i+1,1)-corners(i,1));
    y = corners(i,2) + s*(corners(i+1,2)-corners(i,2));

    if i > 1
        x = x(2:end);
        y = y(2:end);
    end

    x_traj = [x_traj, x];
    y_traj = [y_traj, y];
end

N = length(x_traj);
t = linspace(0, T, N);

%% Inverse kinematics
q_traj = zeros(2, N);
p_check_traj = zeros(2, N);
error_traj = zeros(1, N);

q_guess = [0; 0];

for k = 1:N
    p_des = [x_traj(k); y_traj(k)];

    [q_sol, p_check, pos_error, exitflag] = ...
        inverse_kinematic(p_des, q_guess, l, 'off');

    if exitflag <= 0 || pos_error > 1e-5
        error('\nIK failed at sample %d/%d. Error = %.3e m.', ...
              k, N, pos_error);
    end

    q_traj(:,k) = q_sol;
    p_check_traj(:,k) = p_check;
    error_traj(k) = pos_error;
    q_guess = q_sol;
end
fprintf('\nMaximum position error: %.3e m\n', max(error_traj));

%% Actuator reference signals for Simulink

phi_ref   = q_traj(1,:);    % Actuator A [rad]
theta_ref = q_traj(2,:);    % Actuator E [rad]

% Time vector associated with each reference sample
t_ref = t(:);

% Input matrices: [time, reference angle]
phi_input   = [t_ref, phi_ref(:)];
theta_input = [t_ref, theta_ref(:)];

% Sampling interval
dt_ref = t(2) - t(1);

fprintf('Number of samples: %d\n', N);
fprintf('Reference timestep: %.6f s\n', dt_ref);
fprintf('Reference frequency: %.2f Hz\n', 1/dt_ref);

%% Plot actuator angles
figure;
subplot(2,1,1);
plot(t, rad2deg(q_traj(1,:)), 'LineWidth', 1.5);
ylabel('\phi [deg]'); grid on;

subplot(2,1,2);
plot(t, rad2deg(q_traj(2,:)), 'LineWidth', 1.5);
ylabel('\theta [deg]'); xlabel('Time [s]'); grid on;

%% Plot Cartesian trajectory
figure;
plot(x_traj, y_traj, 'k--', 'LineWidth', 1.5);
hold on;
plot(p_check_traj(1,:), p_check_traj(2,:), 'b-', 'LineWidth', 1.5);
plot(p_home(1), p_home(2), 'ro', 'MarkerFaceColor', 'r');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('Desired', 'Achieved', 'Home', 'Location', 'best');

%% Plot position error
figure;
semilogy(t, max(error_traj, eps), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Error [m]');