%% PLOTTER - desired vs real trajectory comparison
% Run after a CNC_* experiment: needs xc_real, yc_real, xc_des, yc_des
% (To Workspace blocks). The simulated curve is plotted only if out.xc_sim,
% out.yc_sim exist (Model_* simulation).

% Real vs desired trajectory
figure;
hold on
grid on
box on
plot(xc_real,yc_real);
plot(xc_des,yc_des);
axis equal
legend('real','desired'); xlabel('x [m]'); ylabel('y [m]');

% Simulated vs desired trajectory (only if a simulation output exists)
if exist('out', 'var') && any(strcmp(out.who, 'xc_sim'))
    figure;
    hold on
    grid on
    box on
    plot(out.xc_sim,out.yc_sim);
    plot(xc_des,yc_des);
    axis equal
    legend('simulated','desired'); xlabel('x [m]'); ylabel('y [m]');
end

% Absolute error per axis
figure;
error_x = abs(xc_des-xc_real);
subplot(2,1,1);
plot(error_x*1e3);
ylabel('|e_x| [mm]');
grid on;
box on;
error_y = abs(yc_des-yc_real);
subplot(2,1,2);
plot(error_y*1e3);
ylabel('|e_y| [mm]'); xlabel('sample');
grid on;
box on;

% Maximum error per axis
fprintf('max |e_x| = %.3f mm, max |e_y| = %.3f mm\n', ...
        max(error_x)*1e3, max(error_y)*1e3);
