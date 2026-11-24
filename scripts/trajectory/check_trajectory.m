%% CHECK TRAJECTORY - joint angles, speeds and voltage of a path, before running it
%
% Runs the path function in MATLAB and the selected Simulink model, converts the tip path
% into joint angles with the inverse kinematics of the four-bar linkage and checks:
%   - joint angles within the limit (+-20 deg, warning above q_warn)
%   - joint speed and acceleration, feedforward voltage (MATLAB estimate)
%   - from the simulation: peak voltage, time in saturation, tip tracking error
%   - the "speed" that gives each target feedforward voltage (to write in the path function)
% Theory: the motor model is tau*q'' + q' = k*u, so the feedforward is
%   u_ff = |q' + tau*q''|/k + Vc.
% A lap run s times faster has q' x s and q'' x s^2:  u_ff(s) = |s*q' + s^2*tau*q''|/k + Vc.
% MATH STEPS OF THE SCRIPT:
% 1. Lap time and simulation length, asked to the path function (t < 0).
% 2. Tip path (position, velocity, acceleration) from the path function.
% 3. Inverse kinematics (Newton) -> joint angles and speeds; acceleration by differences.
% 4. Feedforward voltage with the SAME k, tau, Vc as the Simulink feedforward blocks.
% 5. Summary of angles, speeds and voltages.
% 6. Simulink run (stop time = simulation length), voltage and tracking of the laps.
% 7. Speeds for the target voltages (bisection on the formula above).

% traj_id:
% 1/2 square, 3/4 circle
% 5/6 triangle, 7/8 star (odd = one speed, even = speed levels)

clear; clc; close all
% the path files (this folder) on the MATLAB path before any @function handle is made
this = fileparts(mfilename('fullpath'));
if isempty(this), this = fullfile(pwd, 'scripts', 'trajectory'); end
addpath(this);

%% 1. Settings
traj_id = 7;   

traj   = @(t) traj_select(t, traj_id);   % the same function the Simulink block calls
Ts     = 0.002;                 % [s] sample time
q_lim  = 24.4;                  % [deg] joint limit, computed with singularities_check.m
q_warn = 20;                    % [deg] warning level (margin for the tracking error)
u_targets = [3 6];        % [V] peak motor voltages wanted: section 7 prints the speeds
stop_from_path = true;          % true : simulate exactly the whole path (Stop Time = path length,
                                %        this run only, the model is not saved)
                                % false: keep the Stop Time saved in the model
model_name = 'Model_pp_obs';    % Simulink model in slx/
controller_setup = fullfile('scripts', 'control', 'pp_design', 'pole_place.m');   % or design_pp.m
                                % setup script that publishes the Simulink variables;
                                % '' = simulate with the variables already loaded
% Assumption: phi = motor 3165, theta = motor 3166.
% Run from the repository root so these relative paths resolve.
this = fullfile(pwd, 'scripts', 'trajectory');
model_file = fullfile(this, '..', '..', 'slx', [model_name '.slx']);
run('scripts/parameters.m');    % geometry l, Vmax, Vc, *_load
if ~isempty(controller_setup)
    run_controller_setup(controller_setup);
end

% Feedforward parameters: the SAME values the Simulink feedforward blocks use
% (k_xxxx_ff, tau_xxxx_ff, Vc_xxxx_ff written by the setup). Without them: the model *_load.
if exist('k_3165_ff', 'var')
    k   = [k_3165_ff,   k_3166_ff];      % [rad/(V s)] [phi, theta]
    tau = [tau_3165_ff, tau_3166_ff];    % [s]
    Vc  = [Vc_3165_ff,  Vc_3166_ff];     % [V]
else
    k   = [k_3165_load,   k_3166_load];
    tau = [tau_3165_load, tau_3166_load];
    Vc  = [Vc_3165,       Vc_3166];
end

% lap time and simulation length, asked to the path function itself (t < 0 = query)
[t_end, t0, T_lap, speed_now, t_laps_end, nLev] = traj(-1);
nLev   = max(nLev, 1);                     % one-speed paths answer 0
t_tail = t_end - t_laps_end;               % [s] still time after the last lap
lev = zeros(nLev, 4);                      % one row per level: [speed, start, end, shape code]
if nLev == 1
    lev(1, :) = [speed_now, t0, t_laps_end, 0];
else
    for i = 1:nLev
        [lev(i, 1), lev(i, 2), lev(i, 3), lev(i, 4)] = traj(-1 - i);   % shape 0 = one shape
    end
end
n_laps = round((lev(1, 3) - lev(1, 2))/T_lap);   % laps per level
T_lev  = (lev(:, 3) - lev(:, 2))/n_laps;         % [s] lap time of every level
fprintf('\n========== WORKSPACE LOADED ==========\n');
fprintf('Path: %d level(s), %d lap(s) each\n', nLev, n_laps);
for i = 1:nLev
    fprintf('  level %d %-8s: speed %5.2f, lap %.2f s, from %5.2f to %5.2f s\n', ...
            i, shape_name(lev(i, 4)), lev(i, 1), T_lev(i), lev(i, 2), lev(i, 3));
end
fprintf('MOTION ENDS AT   %.2f s\n', t_laps_end);
fprintf('STOP TIME TO SET %.2f s  (motion + %.1f s still at the corner)\n', t_end, t_tail);

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
u_ff = abs(dq + tau.*ddq)./k + Vc.*(abs(dq) > 1e-3);

%% 5. Summary
qd = rad2deg(q);
names = {'phi  (left motor) ', 'theta (right motor)'};
fprintf('\n========== TRAJECTORY CHECK (MATLAB estimate) ==========\n');
fprintf('%-20s %9s %9s %10s %12s %9s\n', 'joint', 'min [deg]', 'max [deg]', 'max speed', 'max accel', 'max u_ff');
fprintf('%-20s %9s %9s %10s %12s %9s\n', '', '', '', '[rad/s]', '[rad/s^2]', '[V]');
for i = 1:2
    fprintf('%-20s %9.1f %9.1f %10.2f %12.1f %9.2f\n', names{i}, min(qd(:, i)), max(qd(:, i)), ...
            max(abs(dq(:, i))), max(abs(ddq(:, i))), max(u_ff(:, i)));
end
qmax = max(abs(qd(:)));
if qmax > q_lim
    fprintf('NOT OK: a joint reaches %.1f deg (limit %g): make the path smaller.\n', qmax, q_lim);
elseif qmax > q_warn
    fprintf('WARNING: a joint reaches %.1f deg (above %g, close to the limit).\n', qmax, q_warn);
else
    fprintf('OK: joints within %.1f deg (limit %g). The speed does not change the angles.\n', qmax, q_lim);
end

%% 6. Run the Simulink model (stop time from the path function) and results of the laps
fprintf('\n========== SIMULINK RUN ==========\n');
fprintf('Model: %s\n', model_file);
load_system(model_file);
if stop_from_path
    set_param(model_name, 'StopTime', num2str(t_end));   % the whole path (not saved)
end
t_stop = str2double(get_param(model_name, 'StopTime'));
if isnan(t_stop)                                          % Stop Time written as a variable
    t_stop = evalin('base', get_param(model_name, 'StopTime'));
end
fprintf('Model Stop Time = %.2f s', t_stop);
if t_stop < t_laps_end
    fprintf('  -> the run ends before the motion does (%.2f s)\n', t_laps_end);
else
    fprintf('\n');
end
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
r_sim = [sim_ts(simOut, 'r_3165', sim_t), sim_ts(simOut, 'r_3166', sim_t)];
y_sim = [sim_ts(simOut, 'y_3165', sim_t), sim_ts(simOut, 'y_3166', sim_t)];
e_tip = 1e3*hypot(xc_sim - xc_des, yc_sim - yc_des);      % [mm] tracking error of the tip
fprintf('Completed: %.2f s | %d samples\n\n', sim_t(end), numel(sim_t));

% laps only (no approach): peak command (before the saturation), time beyond Vmax,
% tip error. If the command saturates for long, the motor cannot give the voltage the
% path needs: the joint falls behind and the tip leaves the path.
% one row per speed level (only the part of each level the simulation reached)
fprintf('%5s %-8s %6s | %8s %9s %9s | %14s\n', 'level', 'shape', 'speed', 'u_ff [V]', 'peak u [V]', ...
        'sat. [%]', 'tip err max/RMS [mm]');
for i = 1:nLev
    in_m = t >= lev(i, 2) & t < lev(i, 3);               % MATLAB samples of the level
    in_s = sim_t >= lev(i, 2) & sim_t < lev(i, 3);       % Simulink samples of the level
    if ~any(in_s)
        fprintf('%5d %-8s %6.2f |  not reached by the simulation (Stop Time)\n', ...
                i, shape_name(lev(i, 4)), lev(i, 1));
        continue
    end
    fprintf('%5d %-8s %6.2f | %8.2f %9.2f %9.1f | %7.2f / %5.2f\n', i, shape_name(lev(i, 4)), lev(i, 1), ...
            max(u_ff(in_m, :), [], 'all'), max(abs(u_total(in_s, :)), [], 'all'), ...
            100*mean(any(abs(u_total(in_s, :)) >= Vmax, 2)), ...
            max(e_tip(in_s)), sqrt(mean(e_tip(in_s).^2)));
end
fprintf('u_ff = feedforward (MATLAB); peak u = feedforward + feedback (limit %g V)\n', Vmax);

%% 7. Speeds that give the target feedforward voltages, for EVERY shape of the path
% Reference of a shape = the first lap of its slowest level (speed v_ref). A lap run s times
% faster gives u_ff(s) = |s*q' + s^2*tau*q''|/k + Vc with q', q'' of the reference lap.
% u_ff grows with s: a bisection finds s for each target; speed = s*v_ref (absolute value,
% to write in the path file: "speed" of a *_single file, or a row of "speeds").
codes = unique(lev(:, 4), 'stable');                       % shapes in order of appearance
fprintf('\n========== SPEED FOR A TARGET MOTOR VOLTAGE (approx.) ==========\n');
for g = 1:numel(codes)
    rows = find(lev(:, 4) == codes(g));
    [v_ref, j] = min(lev(rows, 1));   j = rows(j);         % slowest level of this shape
    ref  = t >= lev(j, 2) & t < lev(j, 2) + T_lev(j);      % its first lap
    peak = @(s) max(max(abs(s*dq(ref, :) + s^2*tau.*ddq(ref, :))./k + Vc));
    v_need = zeros(size(u_targets));
    for m = 1:numel(u_targets)
        lo = 0.01;  hi = 50;
        for it = 1:60
            mid = (lo + hi)/2;
            if peak(mid) < u_targets(m), lo = mid; else, hi = mid; end
        end
        v_need(m) = lo*v_ref;
    end
    T_need = T_lev(j)*v_ref./v_need;                       % [s] lap time at each speed
    fprintf('%s (reference: speed %.2f, %.2f V)\n', upper(shape_name(codes(g), 'path')), ...
            v_ref, peak(1));
    fprintf('%10s %8s %9s\n', 'peak [V]', 'speed', 'lap [s]');
    fprintf('%10.1f %8.2f %9.2f\n', [u_targets; v_need; T_need]);
    if codes(g) > 0
        fprintf('  row "%s" of speeds in traj_sequence.m:  %s\n', shape_name(codes(g)), ...
                mat2str(round([v_ref, v_need], 2)));
    else
        fprintf('  as a speeds list (first = reference):  %s\n', mat2str(round([v_ref, v_need], 2)));
    end
end
fprintf('(feedforward peak of the worse motor; the feedback adds a little on top)\n');
fprintf('After changing speeds, run again: STOP TIME TO SET is printed at the top.\n');

% name of the tested trajectory (numbers of traj_select.m)
traj_names = {'square_single', 'square_speeds', 'circle_single', 'triangle_single', ...
              'star_single', 'rhombus_single', 'traj_sequence'};
if traj_id >= 1 && traj_id <= numel(traj_names), tname = traj_names{traj_id};
else, tname = 'unknown'; end
fprintf('\n========== TRAJECTORY TESTED: %s (traj_id = %d) ==========\n', tname, traj_id);

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
plot(sim_t, e_tip, 'LineWidth', 1.0);
xline(lev(:, 2), 'k:');                                           % start of the laps
xlabel('t [s]'); ylabel('[mm]'); title('Tip tracking error (dotted: start of each level)');

nexttile; hold on; grid on; box on
plot(sim_t, u_total, 'LineWidth', 1.0);
plot(sim_t, u_plant, '--', 'LineWidth', 1.0);
yline(Vmax, 'r--', 'LineWidth', 1.0);
yline(-Vmax, 'r--', 'LineWidth', 1.0);
xline(lev(:, 2), 'k:');
xlabel('t [s]'); ylabel('[V]'); title('Voltage: command and plant');
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
% Run the controller setup in this function's workspace, so its CLEAR/CLC/CLOSE ALL
% cannot erase the data of this script. The setup writes its Simulink variables to base.
if ~isfile(setup_file)
    error('Controller setup script not found: %s', setup_file);
end
run(setup_file);
end

function n = shape_name(code, default)
% Name of a shape code of traj_sequence.m (0 = path with one shape)
names = {'square', 'circle', 'rhombus'};
if code >= 1 && code <= numel(names), n = names{code};
elseif nargin > 1, n = default;
else, n = '';
end
end
