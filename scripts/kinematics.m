
%% KINEMATICS SETUP - FORWARD KINEMATICS AND JACOBIAN
clear; clc; close all;
run('parameters.m');

%% 1. Symbolic forward kinematics
syms phi_s theta_s l_s real

a = 2 - sin(theta_s) - cos(phi_s);
b = cos(theta_s) - sin(phi_s);
s = sqrt(a^2 + b^2);
h = l_s*sqrt(1 - s^2/4);

xc = l_s/2*(2 - sin(theta_s) + cos(phi_s)) - h*b/s;
yc = l_s/2*(cos(theta_s) + sin(phi_s)) + h*a/s;

p_sym = [xc; yc];

%% 2. Symbolic Jacobian
J_sym = jacobian(p_sym, [phi_s, theta_s]);

%% 3. Generate standalone numerical functions
matlabFunction(p_sym, ...
    'Vars', {phi_s, theta_s, l_s}, ...
    'File', 'FK_classic', ...
    'Optimize', true);

matlabFunction(J_sym, ...
    'Vars', {phi_s, theta_s, l_s}, ...
    'File', 'J_classic', ...
    'Optimize', true);

fprintf('FK_classic.m and J_classic.m generated successfully.\n');


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


%% 4. Test forward kinematics and Jacobian
p_home = FK_classic(0, 0, l);
J_home = J_classic(0, 0, l);

fprintf('\n--- HOME CONFIGURATION ---\n');
fprintf('x_C = %.6f m\n', p_home(1));
fprintf('y_C = %.6f m\n', p_home(2));

disp('Jacobian at home:');
disp(J_home);

%% 5. Test inverse kinematics
p_des = [0.06; 0.06];
q0 = [0; 0];

[q_sol, p_check, position_error, exitflag] = ...
    inverse_kinematic(p_des, q0, l, 'iter');

fprintf('\n--- INVERSE KINEMATICS ---\n');
fprintf('phi   = %.6f rad = %.3f deg\n', ...
    q_sol(1), rad2deg(q_sol(1)));
fprintf('theta = %.6f rad = %.3f deg\n', ...
    q_sol(2), rad2deg(q_sol(2)));
fprintf('Achieved position: x = %.6f m, y = %.6f m\n', ...
    p_check(1), p_check(2));
fprintf('Position error = %.3e m\n', position_error);
fprintf('Solver exitflag = %d\n', exitflag);

if exitflag <= 0 || position_error > 1e-5
    warning('Convergence or position accuracy not achieved.');
end
